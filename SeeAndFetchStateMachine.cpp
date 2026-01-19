#include "pch.h"

#include "SeeAndFetchStateMachine.h"
#include "SeeAndFetchJointPolicy.h"

#include "ArmKinematics.h"
#include "VisionOverlayService.h"

#include <cmath>

namespace
{
	const wchar_t* StateName(SeeAndFetchStateMachine::State s)
	{
		switch (s)
		{
		case SeeAndFetchStateMachine::State::Idle: return L"Idle";
		case SeeAndFetchStateMachine::State::Acquire: return L"Acquire";
		case SeeAndFetchStateMachine::State::Track: return L"Track";
		case SeeAndFetchStateMachine::State::Approach: return L"Approach";
		case SeeAndFetchStateMachine::State::Grasp: return L"Grasp";
		case SeeAndFetchStateMachine::State::Retreat: return L"Retreat";
	case SeeAndFetchStateMachine::State::SelectGoal: return L"SelectGoal";
	case SeeAndFetchStateMachine::State::FindGoalObject: return L"FindGoalObject";
	case SeeAndFetchStateMachine::State::SelectTerminal: return L"SelectTerminal";
	case SeeAndFetchStateMachine::State::GoAutoHome: return L"GoAutoHome";
	case SeeAndFetchStateMachine::State::ReadyToGrasp: return L"ReadyToGrasp";
	case SeeAndFetchStateMachine::State::GoTerminalPose: return L"GoTerminalPose";
		case SeeAndFetchStateMachine::State::Place: return L"Place";
		case SeeAndFetchStateMachine::State::ReturnHome: return L"ReturnHome";
		case SeeAndFetchStateMachine::State::Abort: return L"Abort";
		case SeeAndFetchStateMachine::State::EStop: return L"EStop";
		default: return L"?";
		}
	}

	inline bool CanIssueMoveNow(const SeeAndFetchStateMachine::Params& P, ULONGLONG now, ULONGLONG lockedUntil, ULONGLONG lastCmd)
	{
		if (lockedUntil != 0 && now < lockedUntil) return false;
		const int minInt = std::max(0, P.timing.minCommandIntervalMs);
		if (lastCmd != 0 && now > lastCmd && (now - lastCmd) < (ULONGLONG)minInt) return false;
		return true;
	}

	inline double Clamp(double v, double mn, double mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}

	inline double StepFromPx(double absErrPx, double kDegPerPx, double minDeg, double maxDeg)
	{
		return Clamp(kDegPerPx * absErrPx, minDeg, maxDeg);
	}
}

void SeeAndFetchStateMachine::Reset()
{
	ToIdle();
	m_hasTablePlaneBase = false;
	m_tablePlaneBase = VisionGeometry::Plane{};
}

void SeeAndFetchStateMachine::ToIdle()
{
	m_state = State::Idle;
	m_stableFrames = 0;
	m_lostFrames = 0;
	m_hasCachedPlanePoint = false;
	m_cachedPlanePointBase = VisionGeometry::Point3{};

	m_lockedUntilMs = 0;
	m_lastCmdMs = 0;
	m_pauseUntilMs = 0;
	m_pauseWasActive = false;
	m_centerStableFrames = 0;
	m_depthStableFrames = 0;
	m_hasLastDepthMm = false;
	m_lastDepthMm = 0.0;
	m_boxStableFrames = 0;
	m_hasBaseBoxArea = false;
	m_baseBoxAreaPx2 = 0;
	m_lastBoxAreaPx2 = 0;
	m_approachAttempt = 0;
	m_forceAdvanceStepsRemaining = 0;
	m_advanceSteps = 0;
	m_gripSteps = 0;
	m_gripCmdPos = 0;
	m_retreatDone = 0;
	m_graspAttempt = 0;
	m_retreatNextState = State::Idle;
	m_retreatTotalSteps = 0;
	m_retreatDeltaDeg = 0.0;

	m_placePhase = 0;
	m_placeDownSteps = 0;
	m_placeAttempt = 0;

	m_hasInitialPosePos = false;
	for (int j = 0; j <= MotionConfig::kJointCount; j++) m_initialPosePos[(size_t)j] = -1;
	m_hasTerminalPosePos = false;
	for (int j = 0; j <= MotionConfig::kJointCount; j++) m_terminalPosePos[(size_t)j] = -1;
	m_goPosePhase = 0;

	// FindGoalObject 相关
	m_hasConfirmedGoalPx = false;
	m_confirmedGoalU = 0.0;
	m_confirmedGoalV = 0.0;
	m_lockCuePhase = 0;
	m_lockCueBasePos = -1;
	m_lockCueTargetPos = -1;
	m_confirmOpenPhase = 0;
	// 注意：m_hasAutoHomePos 和 m_autoHomePos 不在这里重置，由外部 SetAutoHomePos() 设置
}

bool SeeAndFetchStateMachine::Tick(const Input& in, const UserCommand& cmd, Output& out)
{
	out = Output{};
	out.state = m_state;

	const ULONGLONG now = ::GetTickCount64();
	bool allowMoveDuringPause = false;

	// 最高优先级：急停
	if (cmd.eStop)
	{
		m_state = State::EStop;
		out.state = m_state;
		out.active = false;
		out.vsEnable = false;
		out.lockManualJog = false;
		out.reason = L"[EStop] requested.";
		return true;
	}

	// 取消：回到 Idle
	if (cmd.cancel)
	{
		ToIdle();
		m_pauseUntilMs = 0; // 清除暂停
		m_pauseWasActive = false;
		out.state = m_state;
		out.active = false;
		out.vsEnable = false;
		out.lockManualJog = false;
		out.reason = L"[Cancel] back to Idle.";
		return true;
	}

	// =========================================================
	// Point 手势粘滞暂停（仅 SelectGoal / SelectTerminal）
	// 逻辑：检测到 Point 后，保持暂停 2000ms，即使中间有帧没检测到
	// 注：暂停仅抑制舵机输出，不中断搜索流程
	// =========================================================
	const bool inTrackingState = (m_state == State::SelectGoal || m_state == State::SelectTerminal);
	bool pauseActive = false;
	if (inTrackingState)
	{
		// 检测到 Point 手势时，刷新暂停截止时间
		const bool isPointGesture = (in.hasHandLandmarks && in.handGesture == (int)VisionOverlayService::Gesture::Point);
		const bool isSearching = (in.pickState == 1); // PointPick FSM 正在搜索
		if (isPointGesture || isSearching)
		{
			m_pauseUntilMs = now + 2000; // 延长暂停 2000ms
		}

		// 如果仍在暂停期内，仅设置 pause 标记（不中断后续逻辑）
		if (m_pauseUntilMs > 0 && now < m_pauseUntilMs)
		{
			const bool enteringPause = !m_pauseWasActive;
			m_pauseWasActive = true;
			pauseActive = true;
			out.pauseTracking = true;
			out.requestVisionMode = 6; // HandLandmarks
			out.requestPointPickTarget = (m_state == State::SelectTerminal) ? 1 : 0;
			if (enteringPause)
			{
				out.requestPointPickReset = true; // 进入暂停时重置一次，确保开始搜索
			}
		}
		else
		{
			m_pauseWasActive = false;
		}
	}
	else
	{
		// 不在追踪状态时，清除暂停计时
		m_pauseUntilMs = 0;
		m_pauseWasActive = false;
	}

	// 锁定后取消暂停：一旦出现锁定框，允许输出（避免提示动作被压制）
	if (m_state == State::SelectGoal && in.pickState == 2)
	{
		pauseActive = false;
		m_pauseUntilMs = 0;
		m_pauseWasActive = false;
		out.pauseTracking = false;
	}

	const bool hasTarget = in.hasObs && (in.obs.hasTargetPx || in.obs.hasRay || in.obs.hasDepthMm);
	const bool hasDepth = in.hasObs && in.obs.hasDepthMm && in.obs.depthMm > 1e-3;

	// Auto flow common suggestions
	const bool autoActive = (m_state != State::Idle && m_state != State::Abort && m_state != State::EStop);
	out.lockManualJog = autoActive;
	out.requestVisionAruco = false;
	out.requestVisionMode = -1;
	out.requestPointPickTarget = -1;
	out.requestPointPickReset = false;
	out.requestGeminiReset = false;
	if (autoActive)
	{
		if (m_params.grabTestOnly)
		{
			out.requestVisionMode = 7; // VisionService::Mode::Gemini
		}
		else
		{
			// Prefer explicit range-driven mode selection
			const auto rm = m_params.approach.rangeMode;
			if (rm == Params::Approach::RangeMode::ArucoDepth)
			{
				out.requestVisionMode = 2; // VisionService::Mode::Aruco
			}
			else if (rm == Params::Approach::RangeMode::BboxArea)
			{
				// 只有在明确要求 Detector bbox 时才强制切换到 Detector；
				// 否则允许用户用 ColorTrack/HandSticker 等同样会产出 trackBox 的模式。
				if (m_params.approach.bboxRequireDetector)
				{
					out.requestVisionMode = 4; // VisionService::Mode::Detector
				}
			}
			else
			{
				// Auto: keep current unless user asked to force aruco
				if (m_params.preferArucoDuringAuto) out.requestVisionMode = 0; // Auto
			}
		}
	}

	switch (m_state)
	{
	case State::Idle:
	{
		out.active = false;
		out.vsEnable = false;
		out.reason = L"[Idle] waiting confirm.";
		if (cmd.confirm)
		{
			// 抓取测试模式：直接进入抓取流程
			if (m_params.grabTestOnly)
			{
				m_state = State::Acquire;
				out.requestVisionMode = 7; // VisionService::Mode::Gemini
				out.requestGeminiReset = true;
			}
			else
			{
				// 进入"手势锁定抓取物"流程（参考 fake_motion_code.md）
				m_state = State::SelectGoal;
			}
			m_stableFrames = 0;
			m_lostFrames = 0;
			m_centerStableFrames = 0;
			m_depthStableFrames = 0;
			m_hasLastDepthMm = false;
			m_lastDepthMm = 0.0;
			m_advanceSteps = 0;
			m_gripSteps = 0;
			m_gripCmdPos = 0;
			m_retreatDone = 0;
			m_goPosePhase = 0;
			m_hasInitialPosePos = false;
			m_hasTerminalPosePos = false;
			m_pauseUntilMs = 0; // 清除暂停状态
			out.state = m_state;
			out.reason = m_params.grabTestOnly ? L"[Acquire] grab test start." : L"[SelectGoal] start.";
		}
		break;
	}
	case State::SelectGoal:
	{
		if (m_params.grabTestOnly)
		{
			m_state = State::Acquire;
			out.state = m_state;
			out.requestVisionMode = 7; // VisionService::Mode::Gemini
			out.requestGeminiReset = true;
			out.reason = L"[Acquire] grab test redirect.";
			break;
		}
		out.active = true;
		out.vsEnable = false;
		out.requestVisionMode = 6; // VisionService::Mode::HandLandmarks
		out.requestPointPickTarget = 0; 

		// 锁定后夹爪提示（张合一次）：固定张合量200（保证明显）
		// 策略：先向"开"方向移动，再回到原位（视觉上更明显）
		bool cueMoveIssued = false;
		bool confirmMoveIssued = false;
		if (in.pickState != 2)
		{
			m_lockCuePhase = 0;
			m_lockCueBasePos = -1;
			m_lockCueTargetPos = -1;
		}
		else
		{
			// 锁定瞬间记录初始姿态
			if (!m_hasInitialPosePos && in.hasServoPos)
			{
				m_initialPosePos = in.servoPos;
				m_hasInitialPosePos = true;
			}

			const int j = std::max(1, std::min(MotionConfig::kJointCount, m_params.gripper.jointIndex));
			const int openPos = m_params.gripper.openPos;
			const int closePos = m_params.gripper.closePos;
			const int moveTime = std::max(400, m_params.gripper.closeMoveTimeMs); // 增加时间让动作更明显

			if (m_lockCuePhase == 0)
			{
				int basePos = (in.hasServoPos ? in.servoPos[(size_t)j] : -1);
				if (basePos < 0) basePos = (openPos + closePos) / 2; // 默认中间位置
				
				// 固定张合量：先向"开"方向移动200（视觉上更明显）
				const int delta = 200;
				int targetPos;
				if (closePos < openPos)
				{
					// close 值更小（如 close=350, open=650），向开方向是 +delta
					targetPos = basePos + delta;
				}
				else
				{
					// close 值更大（如 close=650, open=350），向开方向是 -delta
					targetPos = basePos - delta;
				}
				// 限制在合法范围
				targetPos = std::max(0, std::min(1000, targetPos));
				m_lockCueBasePos = basePos;
				m_lockCueTargetPos = targetPos;
			}

			if (m_lockCuePhase == 0 && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				out.hasMove = true;
				out.moveTimeMs = moveTime;
				out.jointToPos.clear();
				out.jointToPos.push_back({ j, m_lockCueTargetPos });
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
				m_lockCuePhase = 1;
				cueMoveIssued = true;
				allowMoveDuringPause = true;
				out.pauseTracking = false; // 允许夹爪动作下发
				out.reason = L"[SelectGoal] Locked: gripper cue (away).";
			}
			else if (m_lockCuePhase == 1 && now >= m_lockedUntilMs &&
			         CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				out.hasMove = true;
				out.moveTimeMs = moveTime;
				out.jointToPos.clear();
				out.jointToPos.push_back({ j, m_lockCueBasePos });
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
				m_lockCuePhase = 2;
				cueMoveIssued = true;
				allowMoveDuringPause = true;
				out.pauseTracking = false; // 允许夹爪动作下发
				out.reason = L"[SelectGoal] Locked: gripper cue (back).";
			}
		}

		// OpenPalm 取消：清除记录并从头开始
		if (in.pickState == 4)
		{
			m_hasInitialPosePos = false;
			for (int j = 0; j <= MotionConfig::kJointCount; j++) m_initialPosePos[(size_t)j] = -1;
			m_lockCuePhase = 0;
			m_lockCueBasePos = -1;
			m_lockCueTargetPos = -1;
			m_confirmOpenPhase = 0;
			out.requestPointPickReset = true;
			out.reason = L"[SelectGoal] Cancelled: reset and restart.";
		}

		// Pinch 确认：开爪到最大边界值，作为“已确认要抓取”的提示
		if (in.pickState == 3)
		{
			const int j = std::max(1, std::min(MotionConfig::kJointCount, m_params.gripper.jointIndex));
			const int openPos = m_params.gripper.openPos;
			const int moveTime = std::max(400, m_params.gripper.closeMoveTimeMs);
			if (m_confirmOpenPhase == 0 && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				out.hasMove = true;
				out.moveTimeMs = moveTime;
				out.jointToPos.clear();
				out.jointToPos.push_back({ j, openPos });
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
				m_confirmOpenPhase = 1;
				confirmMoveIssued = true;
				allowMoveDuringPause = true;
				out.pauseTracking = false;
				out.reason = L"[SelectGoal] Confirmed: open gripper.";
			}
			else if (m_confirmOpenPhase == 1 && now >= m_lockedUntilMs)
			{
				m_confirmOpenPhase = 2;
			}
		}

		// =========================================================
		// [重写] 极简手势追踪逻辑
		// 目标：仅使用 J1(水平) 和 J3(俯仰) 跟随手掌，严禁触碰其他关节
		// =========================================================

		// 1. 状态判断：未确认(3)且未取消(4)时允许跟随
		const bool isTracking = (in.pickState < 3);
		
		if (!cueMoveIssued && !confirmMoveIssued && isTracking && in.pKc && in.pMc && in.hasObs && in.obs.hasTargetPx && 
			CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const double cx = (double)in.frameW * 0.5 + (double)m_params.find.centerOffsetU;
			const double cy = (double)in.frameH * 0.5 + (double)m_params.find.centerOffsetV;
			const double errU = in.obs.u - cx;
			const double errV = in.obs.v - cy;

			const int db = std::max(0, m_params.find.deadbandPx);
			const bool inU = std::fabs(errU) <= (double)db;
			const bool inV = std::fabs(errV) <= (double)db;

			if (!inU || !inV)
			{
				std::vector<std::pair<int, int>> moves;
				std::wstring moveWhy;

				if (!inU)
				{
					const double step = StepFromPx(std::fabs(errU),
					                               m_params.find.yaw_kDegPerPx,
					                               m_params.find.yaw_minStepDeg,
					                               m_params.find.yaw_maxStepDeg);
					const double sgn = (errU > 0) ? 1.0 : -1.0;
					const double delta = sgn * (double)m_params.find.signJ1FromErrU * step;

					int pos = 0; 
					std::wstring w;
					if (SeeAndFetchJointPolicy::JointDeltaDegToServoPos(*in.pKc, *in.pMc, in.arm, 1, delta, pos, w))
					{
						const int curPos = in.servoPos[1];
						const int minPosDelta = std::max(0, m_params.find.minServoPosChange);
						if (curPos < 0 || minPosDelta <= 0 || std::abs(curPos - pos) >= minPosDelta)
						{
							moves.push_back({ 1, pos });
							moveWhy += L"J1 ";
						}
					}
				}

				if (!inV)
				{
					double step = StepFromPx(std::fabs(errV),
					                         m_params.find.pitch_kDegPerPx,
					                         m_params.find.pitch_minStepDeg,
					                         m_params.find.pitch_maxStepDeg);
					if (m_params.find.maxPitchStepDeg > 0.0 && step > m_params.find.maxPitchStepDeg)
					{
						step = m_params.find.maxPitchStepDeg;
					}

					const double sgn = (errV > 0) ? 1.0 : -1.0;
					const double delta = sgn * (double)m_params.find.signJ4FromErrV * step;

					int pos = 0;
					std::wstring w;
					if (SeeAndFetchJointPolicy::JointDeltaDegToServoPos(*in.pKc, *in.pMc, in.arm, 4, delta, pos, w))
					{
						const int curPos = in.servoPos[4];
						const int minPosDelta = std::max(0, m_params.find.minServoPosChange);
						if (curPos < 0 || minPosDelta <= 0 || std::abs(curPos - pos) >= minPosDelta)
						{
							moves.push_back({ 4, pos });
							moveWhy += L"J4 ";
						}
					}
				}

				if (!moves.empty())
				{
					out.hasMove = true;
					out.moveTimeMs = std::max(30, m_params.timing.defaultMoveTimeMs);
					out.jointToPos = moves;
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
					out.reason = L"[SelectGoal] Tracking: " + moveWhy;
				}
				else
				{
					out.reason = L"[SelectGoal] In deadband or move too small.";
				}
			}
			else
			{
				out.reason = L"[SelectGoal] Centered.";
			}
		}

		// 交互提示
		if (in.pickState == 3 && m_confirmOpenPhase >= 2) // Confirmed + cue done
		{
			// 只有确认后才跳转，且不再做任何自动对准，直接进下一步
			// 若尚未记录，补记当前位置作为 "initial_pos"
			if (!m_hasInitialPosePos && in.hasServoPos)
			{
				m_initialPosePos = in.servoPos;
				m_hasInitialPosePos = true;
			}
			
			// 跳过 FindGoalObject，直接进 SelectTerminal 或 Acquire
			// 这里为了简化流程，假设用户确认就是想抓了 -> 进 Acquire 也可以
			// 但原流程是选红点。我们保留 SelectTerminal 状态但清空逻辑?
			// 暂时跳转到 SelectTerminal
			m_state = State::SelectTerminal; 
			out.requestPointPickReset = true;
			out.state = m_state;
			out.reason = L"[SelectTerminal] Goal confirmed. Now point to destination.";
		}
		else
		{
			if (!cueMoveIssued && !isTracking) out.reason = L"[SelectGoal] Open palm to track.";
		}
		break;
	}
	case State::FindGoalObject:
	{
		// [重写] 此状态已废弃/跳过，直接进 SelectTerminal
		m_state = State::SelectTerminal;
		out.state = m_state;
		out.reason = L"[FindGoalObject] skipped -> SelectTerminal.";
		break;
	}
	case State::SelectTerminal:
	{
		if (m_params.grabTestOnly)
		{
			m_state = State::Acquire;
			out.state = m_state;
			out.requestVisionMode = 7; // VisionService::Mode::Gemini
			out.requestGeminiReset = true;
			out.reason = L"[Acquire] grab test redirect.";
			break;
		}
		out.active = true;
		out.vsEnable = false;
		out.requestVisionMode = 6; // HandLandmarks
		out.requestPointPickTarget = 0; // Gemini: 目标物识别

		// [改] 终点阶段改为“跟手 + 指向锁定标记物（Gemini）”
		// 1) 先跟随手指（同 SelectGoal）
		// 2) Point -> Gemini 锁定标记物；锁定后记录终点姿态
		const bool isTracking = (in.pickState < 3);
		if (isTracking && in.pKc && in.pMc && in.hasObs && in.obs.hasTargetPx &&
		    CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const double cx = (double)in.frameW * 0.5 + (double)m_params.find.centerOffsetU;
			const double cy = (double)in.frameH * 0.5 + (double)m_params.find.centerOffsetV;
			const double errU = in.obs.u - cx;
			const double errV = in.obs.v - cy;

			const int db = std::max(0, m_params.find.deadbandPx);
			if (std::fabs(errU) > db || std::fabs(errV) > db)
			{
				std::vector<std::pair<int, int>> moves;
				if (std::fabs(errU) > db)
				{
					const double step = StepFromPx(std::fabs(errU),
					                               m_params.find.yaw_kDegPerPx,
					                               m_params.find.yaw_minStepDeg,
					                               m_params.find.yaw_maxStepDeg);
					const double sgn = (errU > 0) ? 1.0 : -1.0;
					const double delta = sgn * (double)m_params.find.signJ1FromErrU * step;

					int pos = 0; std::wstring w;
					if (SeeAndFetchJointPolicy::JointDeltaDegToServoPos(*in.pKc, *in.pMc, in.arm, 1, delta, pos, w))
					{
						const int cur = in.servoPos[1];
						const int minPosDelta = std::max(0, m_params.find.minServoPosChange);
						if (cur < 0 || minPosDelta <= 0 || std::abs(cur - pos) >= minPosDelta) moves.push_back({ 1, pos });
					}
				}

				if (std::fabs(errV) > db)
				{
					double step = StepFromPx(std::fabs(errV),
					                         m_params.find.pitch_kDegPerPx,
					                         m_params.find.pitch_minStepDeg,
					                         m_params.find.pitch_maxStepDeg);
					if (m_params.find.maxPitchStepDeg > 0.0 && step > m_params.find.maxPitchStepDeg)
					{
						step = m_params.find.maxPitchStepDeg;
					}

					const double sgn = (errV > 0) ? 1.0 : -1.0;
					const double delta = sgn * (double)m_params.find.signJ4FromErrV * step;

					int pos = 0; std::wstring w;
					if (SeeAndFetchJointPolicy::JointDeltaDegToServoPos(*in.pKc, *in.pMc, in.arm, 4, delta, pos, w))
					{
						const int cur = in.servoPos[4];
						const int minPosDelta = std::max(0, m_params.find.minServoPosChange);
						if (cur < 0 || minPosDelta <= 0 || std::abs(cur - pos) >= minPosDelta) moves.push_back({ 4, pos });
					}
				}

				if (!moves.empty())
				{
					out.hasMove = true;
					out.moveTimeMs = std::max(30, m_params.timing.defaultMoveTimeMs);
					out.jointToPos = moves;
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
					out.reason = L"[SelectTerminal] Tracking hand (terminal).";
				}
			}
		}

		// 锁定标记物后，记录“瞄准终点”姿态
		if (in.pickState == 2 && in.hasServoPos)
		{
			m_terminalPosePos = in.servoPos;
			m_hasTerminalPosePos = true;
		}

		// 确认逻辑
		if (in.pickState == 3) // Pinch to confirm terminal
		{
			if (!m_hasTerminalPosePos && in.hasServoPos)
			{
				m_terminalPosePos = in.servoPos;
				m_hasTerminalPosePos = true;
			}
			// 确认后直接去 GoAutoHome (准备抓取)
			m_state = State::GoAutoHome;
			out.requestPointPickReset = true;
			out.state = m_state;
			out.reason = L"[GoAutoHome] Terminal confirmed.";
		}
		else
		{
			out.reason = L"[SelectTerminal] Point to marker, pinch to confirm.";
		}
		break;
	}
	case State::GoAutoHome:
	{
		// [重写] 极简自动归位
		// 不再等待和检测读回，为了流畅性，直接发命令然后跳转
		// 仍然保留 J1-J5 限制
		out.active = true;
		out.vsEnable = false;

		if (!m_hasAutoHomePos)
		{
			m_state = State::ReadyToGrasp;
			out.state = m_state;
			out.reason = L"[ReadyToGrasp] No AutoHome. Wait confirm.";
			break;
		}

		if (m_goPosePhase == 0 && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			out.hasMove = true;
			out.moveTimeMs = std::max(200, m_params.ret.returnTimeMs);
			out.jointToPos.clear();
			for (int j = 1; j <= 5; j++) // 仅 J1-J5
			{
				const int p = m_autoHomePos[(size_t)j];
				if (p >= 0) out.jointToPos.push_back({ j, p });
			}
			m_lastCmdMs = now;
			m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
			m_goPosePhase = 1;
			out.reason = L"[GoAutoHome] Moving...";
			break;
		}

		if (m_goPosePhase == 1 && m_lockedUntilMs != 0 && now >= m_lockedUntilMs)
		{
			m_state = State::ReadyToGrasp;
			out.state = m_state;
			m_goPosePhase = 0;
			out.reason = L"[ReadyToGrasp] AutoHome done. Wait confirm.";
			break;
		}

		out.reason = L"[GoAutoHome] Pacing...";
		break;
	}
	case State::ReadyToGrasp:
	{
		out.active = true;
		out.vsEnable = false;
		out.lockManualJog = false; // 允许手动自由检查
		out.reason = L"[ReadyToGrasp] Confirm to start grasp.";

		if (cmd.confirm)
		{
			m_state = State::Acquire;
			if (m_params.grabTestOnly)
			{
				out.requestVisionMode = 7; // VisionService::Mode::Gemini
				out.requestGeminiReset = true;
			}
			out.state = m_state;
			out.reason = L"[Acquire] Start grasp flow.";
		}
		break;
	}
	case State::Acquire:
	{
		out.active = true;
		out.vsEnable = false;

		if (hasTarget)
		{
			m_stableFrames++;
			m_lostFrames = 0;
		}
		else
		{
			m_lostFrames++;
			m_stableFrames = 0;
		}

		if (m_lostFrames > m_params.lostFramesToAbort)
		{
			m_state = State::Abort;
			out.state = m_state;
			out.vsEnable = false;
			out.active = false;
			out.reason = L"[Abort] acquire lost target.";
			break;
		}

		// Attempt to center while acquiring
		if (in.pKc && in.pMc && hasTarget && in.obs.hasTargetPx && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const auto step = SeeAndFetchJointPolicy::ComputeFindStep(m_params, *in.pKc, *in.pMc, in.arm, in.obs, in.frameW, in.frameH);
			if (step.ok && step.hasMove)
			{
				out.hasMove = true;
				out.moveTimeMs = step.moveTimeMs;
				out.jointToPos = step.jointToPos;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
			}
		}

		if (m_stableFrames >= m_params.acquireStableFrames)
		{
			m_state = State::Track;
			out.state = m_state;
			if (m_params.grabTestOnly)
			{
				out.requestGeminiReset = true;
			}
			out.reason = L"[Track] locked.";
		}
		else
		{
			out.reason = std::wstring(L"[Acquire] stable=") + std::to_wstring(m_stableFrames) +
			             L" lost=" + std::to_wstring(m_lostFrames);
		}
		break;
	}
	case State::Track:
	{
		out.active = true;
		out.vsEnable = false;

		if (!hasTarget)
		{
			m_lostFrames++;
			out.reason = std::wstring(L"[Track] lost=") + std::to_wstring(m_lostFrames);
			if (m_lostFrames > m_params.lostFramesToAbort)
			{
				m_state = State::Abort;
				out.state = m_state;
				out.vsEnable = false;
				out.active = false;
				out.reason = L"[Abort] track lost target.";
			}
			break;
		}
		m_lostFrames = 0;

		// Keep centering
		bool centeredNow = false;
		if (in.obs.hasTargetPx)
		{
			const double cx = (double)in.frameW * 0.5;
			const double cy = (double)in.frameH * 0.5;
			const double eu = in.obs.u - cx;
			const double ev = in.obs.v - cy;
			const int db = std::max(0, m_params.find.deadbandPx);
			centeredNow = (std::fabs(eu) <= (double)db && std::fabs(ev) <= (double)db);
		}
		if (centeredNow) m_centerStableFrames++;
		else m_centerStableFrames = 0;

		if (in.pKc && in.pMc && in.obs.hasTargetPx && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const auto step = SeeAndFetchJointPolicy::ComputeFindStep(m_params, *in.pKc, *in.pMc, in.arm, in.obs, in.frameW, in.frameH);
			if (step.ok && step.hasMove)
			{
				out.hasMove = true;
				out.moveTimeMs = step.moveTimeMs;
				out.jointToPos = step.jointToPos;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
			}
		}

		// 可选：锁定时缓存平面坐标（遮挡后导航基础）
		if (m_params.enablePlaneCache && !m_hasCachedPlanePoint && m_hasTablePlaneBase && in.hasObs && in.obs.hasRay)
		{
			VisionGeometry::Point3 hit{};
			double t = 0.0;
			std::wstring why;
			const VisionGeometry::Ray rCam{ in.obs.rayX, in.obs.rayY, in.obs.rayZ };
			if (ToolGeometry::ComputeRayPlaneHitInBase(in.arm, in.tool, rCam, m_tablePlaneBase, hit, t, &why))
			{
				m_hasCachedPlanePoint = true;
				m_cachedPlanePointBase = hit;
			}
		}

		out.hasCachedPlanePoint = m_hasCachedPlanePoint;
		out.cachedPlanePointBase = m_cachedPlanePointBase;
		out.reason = std::wstring(L"[Track] ok planeCache=") + (m_hasCachedPlanePoint ? L"Y" : L"N") +
		             L" centerStable=" + std::to_wstring(m_centerStableFrames);

		// Auto transition into Approach when centered is stable
		if (m_centerStableFrames >= std::max(1, m_params.find.stableCenterFrames))
		{
			m_state = State::Approach;
			out.state = m_state;
			if (m_params.grabTestOnly)
			{
				out.requestGeminiReset = true;
			}
			m_depthStableFrames = 0;
			m_hasLastDepthMm = false;
			m_lastDepthMm = 0.0;
			m_boxStableFrames = 0;
			m_hasBaseBoxArea = false;
			m_baseBoxAreaPx2 = 0;
			m_lastBoxAreaPx2 = 0;
			m_advanceSteps = 0;
			m_approachAttempt = 0;
			m_forceAdvanceStepsRemaining = 0;
			m_graspAttempt = 0;
			out.reason = L"[Approach] start.";
		}
		break;
	}
	case State::Approach:
	{
		out.active = true;
		out.vsEnable = false;

		if (!hasTarget || !in.obs.hasTargetPx)
		{
			m_lostFrames++;
			if (m_lostFrames > m_params.lostFramesToAbort)
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = L"[Abort] approach lost target.";
			}
			else
			{
				out.reason = std::wstring(L"[Approach] lost=") + std::to_wstring(m_lostFrames);
			}
			break;
		}
		m_lostFrames = 0;

		// Keep centering first
		bool centeredNow = false;
		{
			const double cx = (double)in.frameW * 0.5;
			const double cy = (double)in.frameH * 0.5;
			const double eu = in.obs.u - cx;
			const double ev = in.obs.v - cy;
			const int db = std::max(0, m_params.find.deadbandPx);
			centeredNow = (std::fabs(eu) <= (double)db && std::fabs(ev) <= (double)db);
		}

		// Depth stability bookkeeping
		if (hasDepth)
		{
			const double d = in.obs.depthMm;
			bool stable = true;
			if (m_hasLastDepthMm)
			{
				if (std::fabs(d - m_lastDepthMm) > (double)std::max(0, m_params.approach.depthMaxJumpMm))
				{
					stable = false;
				}
			}
			if (stable) m_depthStableFrames++;
			else m_depthStableFrames = 0;
			m_hasLastDepthMm = true;
			m_lastDepthMm = d;
		}
		else
		{
			m_depthStableFrames = 0;
			m_hasLastDepthMm = false;
		}

		// Box stability bookkeeping (area)
		const bool boxOk =
			in.hasBox &&
			in.boxW > 0 && in.boxH > 0 &&
			(!m_params.approach.bboxRequireDetector || in.boxClassId >= 0 || in.visionMode == 4);
		int areaPx2 = 0;
		if (boxOk)
		{
			areaPx2 = std::max(0, in.boxW) * std::max(0, in.boxH);
			bool stable = true;
			if (m_lastBoxAreaPx2 > 0)
			{
				if (std::abs(areaPx2 - m_lastBoxAreaPx2) > std::max(0, m_params.approach.boxAreaMaxJumpPx2))
				{
					stable = false;
				}
			}
			if (stable) m_boxStableFrames++;
			else m_boxStableFrames = 0;
			m_lastBoxAreaPx2 = areaPx2;
			if (!m_hasBaseBoxArea)
			{
				m_hasBaseBoxArea = true;
				m_baseBoxAreaPx2 = areaPx2;
			}
		}
		else
		{
			m_boxStableFrames = 0;
			m_lastBoxAreaPx2 = 0;
			m_hasBaseBoxArea = false;
			m_baseBoxAreaPx2 = 0;
		}

		auto bboxInRange = [&]() -> bool
		{
			if (!boxOk) return false;
			if (m_boxStableFrames < std::max(1, m_params.approach.boxStableFrames)) return false;
			const int absTh = std::max(0, m_params.approach.graspBoxAreaPx2);
			double scale = 0.0;
			if (m_params.approach.graspBoxScale_milli > 0)
			{
				scale = (double)m_params.approach.graspBoxScale_milli / 1000.0;
			}
			const int base = std::max(1, m_baseBoxAreaPx2);
			const int scaledTh = (scale > 0.0) ? (int)std::lround((double)base * scale) : 0;
			const int th = std::max(absTh, scaledTh);
			if (th <= 0) return false;
			return areaPx2 >= th;
		};

		auto depthInRange = [&]() -> bool
		{
			if (!hasDepth) return false;
			if (m_depthStableFrames < std::max(1, m_params.approach.depthStableFrames)) return false;
			return (int)std::lround(in.obs.depthMm) <= std::max(1, m_params.approach.graspDepthMm);
		};

		auto inRangeNow = [&]() -> bool
		{
			const auto rm = m_params.approach.rangeMode;
			if (rm == Params::Approach::RangeMode::ArucoDepth) return depthInRange();
			if (rm == Params::Approach::RangeMode::BboxArea) return bboxInRange();
			// Auto
			return depthInRange() || bboxInRange();
		};

		// 1) If not centered, send Find step
		if (in.pKc && in.pMc && !centeredNow && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const auto step = SeeAndFetchJointPolicy::ComputeFindStep(m_params, *in.pKc, *in.pMc, in.arm, in.obs, in.frameW, in.frameH);
			if (step.ok && step.hasMove)
			{
				out.hasMove = true;
				out.moveTimeMs = step.moveTimeMs;
				out.jointToPos = step.jointToPos;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
			}
			out.reason = std::wstring(L"[Approach] centering depthStable=") + std::to_wstring(m_depthStableFrames);
			break;
		}

		// 2) Forced advance steps (used by grasp retry)
		if (m_forceAdvanceStepsRemaining > 0)
		{
			if (m_advanceSteps >= std::max(1, m_params.approach.maxAdvanceSteps))
			{
				m_forceAdvanceStepsRemaining = 0;
			}
			else if (in.pKc && in.pMc && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				const auto step = SeeAndFetchJointPolicy::ComputeApproachStep(m_params, *in.pKc, *in.pMc, in.arm);
				if (step.ok && step.hasMove)
				{
					out.hasMove = true;
					out.moveTimeMs = step.moveTimeMs;
					out.jointToPos = step.jointToPos;
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
					m_advanceSteps++;
					m_forceAdvanceStepsRemaining--;
					out.reason = std::wstring(L"[Approach] forcedAdvance left=") + std::to_wstring(m_forceAdvanceStepsRemaining);
					break;
				}
			}
		}

		// 3) If reached, go grasp
		if (inRangeNow())
		{
			m_state = State::Grasp;
			out.state = m_state;
			if (m_params.grabTestOnly)
			{
				out.requestGeminiReset = true;
			}
			m_gripSteps = 0;
			m_gripCmdPos = 0;
			m_graspAttempt = std::max(0, m_graspAttempt);
			out.reason = L"[Grasp] enter.";
			break;
		}

		// 4) Otherwise advance by J2 steps (after range signal stable)
		const auto rm = m_params.approach.rangeMode;
		if (rm == Params::Approach::RangeMode::ArucoDepth && !hasDepth)
		{
			out.reason = L"[Approach] waiting depthMm (Aruco).";
			break;
		}
		if (rm == Params::Approach::RangeMode::BboxArea && !boxOk)
		{
			out.reason = L"[Approach] waiting bbox (Detector).";
			break;
		}
		if (rm == Params::Approach::RangeMode::ArucoDepth && m_depthStableFrames < std::max(1, m_params.approach.depthStableFrames))
		{
			out.reason = std::wstring(L"[Approach] depth unstable stableFrames=") + std::to_wstring(m_depthStableFrames);
			break;
		}
		if (rm == Params::Approach::RangeMode::BboxArea && m_boxStableFrames < std::max(1, m_params.approach.boxStableFrames))
		{
			out.reason = std::wstring(L"[Approach] bbox unstable stableFrames=") + std::to_wstring(m_boxStableFrames);
			break;
		}
		if (rm == Params::Approach::RangeMode::Auto)
		{
			const bool hasAny = (hasDepth && m_depthStableFrames >= std::max(1, m_params.approach.depthStableFrames)) ||
			                    (boxOk && m_boxStableFrames >= std::max(1, m_params.approach.boxStableFrames));
			if (!hasAny)
			{
				out.reason = L"[Approach] waiting stable depth/bbox.";
				break;
			}
		}
		if (m_advanceSteps >= std::max(1, m_params.approach.maxAdvanceSteps))
		{
			// retry
			if (m_approachAttempt + 1 < std::max(1, m_params.approach.maxAttempts))
			{
				m_approachAttempt++;
				m_state = State::Retreat;
				out.state = m_state;
				m_retreatDone = 0;
				m_retreatTotalSteps = std::max(0, m_params.approach.retryRetreatSteps);
				m_retreatNextState = State::Track;
				m_retreatDeltaDeg = -(double)m_params.approach.signJ2Advance * std::max(0.0, m_params.approach.j2AdvanceStepDeg);
				out.reason = std::wstring(L"[Retreat] retry attempt=") + std::to_wstring(m_approachAttempt);
			}
			else
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = L"[Abort] approach max steps reached.";
			}
			break;
		}
		if (in.pKc && in.pMc && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const auto step = SeeAndFetchJointPolicy::ComputeApproachStep(m_params, *in.pKc, *in.pMc, in.arm);
			if (step.ok && step.hasMove)
			{
				out.hasMove = true;
				out.moveTimeMs = step.moveTimeMs;
				out.jointToPos = step.jointToPos;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
				m_advanceSteps++;
			}
			out.reason = std::wstring(L"[Approach] advance step=") + std::to_wstring(m_advanceSteps) +
			             (hasDepth ? (L" depth=" + std::to_wstring((int)std::lround(in.obs.depthMm))) : L"") +
			             (boxOk ? (L" boxA=" + std::to_wstring(areaPx2)) : L"");
		}
		else
		{
			out.reason = L"[Approach] pacing.";
		}
		break;
	}
	case State::Grasp:
	{
		out.active = true;
		out.vsEnable = false;

		// Stepwise close gripper (J6 by default)
		const int j = std::max(1, std::min(MotionConfig::kJointCount, m_params.gripper.jointIndex));
		const int openPos = m_params.gripper.openPos;
		const int closePos = m_params.gripper.closePos;
		const int step = std::max(1, m_params.gripper.closeStepPos);
		const int maxSteps = std::max(1, m_params.gripper.maxCloseSteps);
		const int moveTime = std::max(60, m_params.gripper.closeMoveTimeMs);

		if (m_gripCmdPos == 0)
		{
			// Initialize from openPos (or current readback if provided)
			if (in.hasGripReadback && in.gripReadbackAgeMs <= (DWORD)std::max(0, m_params.gripper.stallDetectMaxAgeMs))
			{
				m_gripCmdPos = in.gripReadbackPos;
			}
			else
			{
				m_gripCmdPos = openPos;
			}
		}

		const int dir = (closePos < m_gripCmdPos) ? -1 : +1;
		int nextPos = m_gripCmdPos + dir * step;
		if ((dir < 0 && nextPos < closePos) || (dir > 0 && nextPos > closePos)) nextPos = closePos;

		if (CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			out.hasMove = true;
			out.moveTimeMs = moveTime;
			out.jointToPos.clear();
			out.jointToPos.push_back({ j, nextPos });
			m_gripCmdPos = nextPos;
			m_gripSteps++;
			m_lastCmdMs = now;
			m_lockedUntilMs = now + (ULONGLONG)std::max(0, moveTime);
		}

		bool graspDone = (m_gripCmdPos == closePos) || (m_gripSteps >= maxSteps);
		bool graspSuccess = true;
		if (m_params.gripper.enableStallDetect && in.hasGripReadback &&
		    in.gripReadbackAgeMs <= (DWORD)std::max(0, m_params.gripper.stallDetectMaxAgeMs))
		{
			// If readback lags behind commanded a lot, treat as stall (caught object)
			if (std::abs(in.gripReadbackPos - m_gripCmdPos) > std::max(0, m_params.gripper.stallDetectDeltaPos))
			{
				graspDone = true;
				graspSuccess = true;
			}
			else
			{
				// if fully closed but no stall, treat as fail
				if ((m_gripCmdPos == closePos) || (m_gripSteps >= maxSteps))
				{
					graspSuccess = false;
				}
			}
		}

		out.reason = std::wstring(L"[Grasp] step=") + std::to_wstring(m_gripSteps) +
		             L" pos=" + std::to_wstring(m_gripCmdPos);

		if (graspDone && !graspSuccess && m_params.gripper.enableStallDetect)
		{
			// retry: nudge forward then re-grasp
			m_graspAttempt++;
			if (m_graspAttempt < std::max(1, m_params.gripper.maxAttempts))
			{
				m_forceAdvanceStepsRemaining = std::max(1, m_params.gripper.advanceStepsOnFail);
				m_state = State::Approach;
				out.state = m_state;
				m_gripSteps = 0;
				m_gripCmdPos = 0;
				out.reason = std::wstring(L"[Approach] graspRetry attempt=") + std::to_wstring(m_graspAttempt);
				break;
			}
		}

		if (graspDone)
		{
			m_state = State::Retreat;
			out.state = m_state;
			m_retreatDone = 0;
			m_retreatTotalSteps = std::max(0, m_params.place.retreatSteps);
			if (m_params.grabTestOnly)
			{
				m_retreatNextState = State::ReturnHome;
				m_goPosePhase = 0;
			}
			else
			{
				// 抓取后优先移动到 terminal_pos（若已记录），再进入 Place
				if (m_hasTerminalPosePos)
				{
					m_retreatNextState = State::GoTerminalPose;
					m_goPosePhase = 0;
				}
				else
				{
					m_retreatNextState = State::Place;
				}
			}
			// retreat is opposite direction of approach advance (post-grasp lift away)
			m_retreatDeltaDeg = -(double)m_params.approach.signJ2Advance * std::max(0.0, m_params.approach.j2AdvanceStepDeg);
			out.reason = L"[Retreat] enter.";
		}
		break;
	}
	case State::Retreat:
	{
		out.active = true;
		out.vsEnable = false;

		const int steps = std::max(0, m_retreatTotalSteps);
		if (m_retreatDone >= steps)
		{
			m_state = m_retreatNextState;
			out.state = m_state;
			if (m_state == State::Place)
			{
				// init place sub-state machine
				m_placePhase = 0;
				m_placeDownSteps = 0;
				m_placeAttempt = 0;
				m_centerStableFrames = 0;
				m_depthStableFrames = 0;
				m_hasLastDepthMm = false;
				m_lastDepthMm = 0.0;
				m_boxStableFrames = 0;
				m_lastBoxAreaPx2 = 0;
				m_hasBaseBoxArea = false;
				m_baseBoxAreaPx2 = 0;
			}
			if (m_state == State::Track)
			{
				out.reason = L"[Track] retry.";
			}
			else if (m_state == State::ReturnHome)
			{
				out.reason = L"[ReturnHome] enter.";
			}
			else
			{
				out.reason = L"[Place] enter.";
			}
			break;
		}

		if (in.pKc && in.pMc && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			// retreat delta is determined when entering Retreat (may differ for place retry)
			const double deltaDeg = m_retreatDeltaDeg;
			int pos = -1;
			std::wstring why;
			if (SeeAndFetchJointPolicy::JointDeltaDegToServoPos(*in.pKc, *in.pMc, in.arm, 2, deltaDeg, pos, why))
			{
				out.hasMove = true;
				out.moveTimeMs = std::max(60, m_params.timing.defaultMoveTimeMs);
				out.jointToPos.clear();
				out.jointToPos.push_back({ 2, pos });
				m_retreatDone++;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
				out.reason = std::wstring(L"[Retreat] step=") + std::to_wstring(m_retreatDone) +
				             L"/" + std::to_wstring(steps);
			}
			else
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = std::wstring(L"[Abort] retreat failed: ") + why;
			}
		}
		else
		{
			out.reason = L"[Retreat] pacing.";
		}
		break;
	}
	case State::GoTerminalPose:
	{
		out.active = true;
		out.vsEnable = false;

		if (!m_hasTerminalPosePos)
		{
			// 没有终点快照，直接退化到 Place
			m_state = State::Place;
			out.state = m_state;
			out.reason = L"[Place] no terminal_pos, fallback.";
			break;
		}

		if (m_goPosePhase == 0 && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			out.hasMove = true;
			out.moveTimeMs = std::max(200, m_params.ret.returnTimeMs);
			out.jointToPos.clear();
			// 只恢复 J1-J5 的位置，不包括夹爪（J6），避免在移动到终点时意外开合夹爪
			for (int j = 1; j <= 5; j++)
			{
				const int p = m_terminalPosePos[(size_t)j];
				if (p >= 0) out.jointToPos.push_back({ j, p });
			}
			m_lastCmdMs = now;
			m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
			m_goPosePhase = 1;
			out.reason = L"[GoTerminalPose] moving...";
			break;
		}

		if (m_goPosePhase == 1 && m_lockedUntilMs != 0 && now >= m_lockedUntilMs)
		{
			m_state = State::Place;
			out.state = m_state;
			// init place sub-state machine (reuse existing init path)
			m_placePhase = 0;
			m_placeDownSteps = 0;
			m_placeAttempt = 0;
			m_centerStableFrames = 0;
			m_depthStableFrames = 0;
			m_hasLastDepthMm = false;
			m_lastDepthMm = 0.0;
			m_boxStableFrames = 0;
			m_lastBoxAreaPx2 = 0;
			m_hasBaseBoxArea = false;
			m_baseBoxAreaPx2 = 0;
			out.reason = L"[Place] enter after go terminal.";
			break;
		}

		out.reason = L"[GoTerminalPose] waiting...";
		break;
	}
	case State::Place:
	{
		out.active = true;
		out.vsEnable = false;

		const auto pm = m_params.place.mode;
		if (pm == Params::Place::Mode::SimpleOpen)
		{
			// Open gripper then go ReturnHome
			const int j = std::max(1, std::min(MotionConfig::kJointCount, m_params.gripper.jointIndex));
			if (CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				out.hasMove = true;
				out.moveTimeMs = std::max(80, m_params.gripper.closeMoveTimeMs);
				out.jointToPos.clear();
				out.jointToPos.push_back({ j, m_params.gripper.openPos });
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
			}
			m_state = State::ReturnHome;
			out.state = m_state;
			out.reason = L"[ReturnHome] enter.";
			break;
		}

		// RedDotVisual: request vision mode (default ColorTrack)
		out.requestVisionMode = m_params.place.visionMode;

		const bool hasObs = in.hasObs && in.obs.hasTargetPx;
		const bool hasDepth = in.hasObs && in.obs.hasDepthMm;
		const bool boxOk = in.hasBox && in.boxW > 0 && in.boxH > 0;
		const int areaPx2 = boxOk ? (std::max(0, in.boxW) * std::max(0, in.boxH)) : 0;

		// stability bookkeeping
		{
			// depth stable (reuse approach depth jitter threshold)
			if (hasDepth)
			{
				const double d = in.obs.depthMm;
				bool stable = true;
				if (m_hasLastDepthMm)
				{
					if (std::fabs(d - m_lastDepthMm) > (double)std::max(0, m_params.approach.depthMaxJumpMm)) stable = false;
				}
				if (stable) m_depthStableFrames++;
				else m_depthStableFrames = 0;
				m_lastDepthMm = d;
				m_hasLastDepthMm = true;
			}
			else
			{
				m_depthStableFrames = 0;
				m_hasLastDepthMm = false;
				m_lastDepthMm = 0.0;
			}

			// bbox stable (area)
			if (boxOk)
			{
				bool stable = true;
				if (m_lastBoxAreaPx2 > 0)
				{
					if (std::abs(areaPx2 - m_lastBoxAreaPx2) > std::max(0, m_params.place.boxAreaMaxJumpPx2)) stable = false;
				}
				if (stable) m_boxStableFrames++;
				else m_boxStableFrames = 0;
				m_lastBoxAreaPx2 = areaPx2;
				if (!m_hasBaseBoxArea)
				{
					m_hasBaseBoxArea = true;
					m_baseBoxAreaPx2 = areaPx2;
				}
			}
			else
			{
				m_boxStableFrames = 0;
				m_lastBoxAreaPx2 = 0;
				m_hasBaseBoxArea = false;
				m_baseBoxAreaPx2 = 0;
			}
		}

		auto depthInRange = [&]() -> bool
		{
			if (!hasDepth) return false;
			if (m_depthStableFrames < std::max(1, m_params.approach.depthStableFrames)) return false;
			return (int)std::lround(in.obs.depthMm) <= std::max(1, m_params.place.placeDepthMm);
		};
		auto boxInRange = [&]() -> bool
		{
			if (!boxOk) return false;
			if (m_boxStableFrames < std::max(1, m_params.place.boxStableFrames)) return false;
			const int absTh = std::max(0, m_params.place.placeBoxAreaPx2);
			double scale = 0.0;
			if (m_params.place.placeBoxScale_milli > 0) scale = (double)m_params.place.placeBoxScale_milli / 1000.0;
			const int base = std::max(1, m_baseBoxAreaPx2);
			const int scaledTh = (scale > 0.0) ? (int)std::lround((double)base * scale) : 0;
			const int th = std::max(absTh, scaledTh);
			if (th <= 0) return false;
			return areaPx2 >= th;
		};
		auto inRangeNow = [&]() -> bool
		{
			const auto rm = m_params.place.rangeMode;
			if (rm == Params::Place::RangeMode::ArucoDepth) return depthInRange();
			if (rm == Params::Place::RangeMode::BboxArea) return boxInRange();
			return depthInRange() || boxInRange();
		};

		// Phase 0: center on red dot
		if (m_placePhase == 0)
		{
			if (!hasObs)
			{
				out.reason = L"[Place] waiting target px.";
				break;
			}
			if (in.pKc && in.pMc && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				const auto step = SeeAndFetchJointPolicy::ComputeFindStep(m_params, *in.pKc, *in.pMc, in.arm, in.obs, in.frameW, in.frameH);
				if (!step.ok)
				{
					m_state = State::Abort;
					out.state = m_state;
					out.active = false;
					out.reason = std::wstring(L"[Abort] place center failed: ") + step.why;
					break;
				}
				if (step.centeredNow) m_centerStableFrames++;
				else m_centerStableFrames = 0;

				if (m_centerStableFrames >= std::max(1, m_params.place.centerStableFrames))
				{
					m_placePhase = 1;
					m_placeDownSteps = 0;
					m_depthStableFrames = 0;
					m_hasLastDepthMm = false;
					m_lastDepthMm = 0.0;
					m_boxStableFrames = 0;
					m_lastBoxAreaPx2 = 0;
					m_hasBaseBoxArea = false;
					m_baseBoxAreaPx2 = 0;
					out.reason = L"[Place] centered -> down.";
					break;
				}

				if (step.hasMove)
				{
					out.hasMove = true;
					out.moveTimeMs = step.moveTimeMs;
					out.jointToPos = step.jointToPos;
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
					out.reason = L"[Place] centering.";
				}
				else
				{
					out.reason = L"[Place] in deadband.";
				}
			}
			else
			{
				out.reason = L"[Place] centering pacing.";
			}
			break;
		}

		// Phase 1: down until in range
		if (inRangeNow())
		{
			m_placePhase = 2;
			out.reason = L"[Place] in range -> release.";
			break;
		}

		// Phase 2: release (open gripper) then go ReturnHome
		if (m_placePhase == 2)
		{
			const int j = std::max(1, std::min(MotionConfig::kJointCount, m_params.gripper.jointIndex));
			if (CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
			{
				out.hasMove = true;
				out.moveTimeMs = std::max(80, m_params.gripper.closeMoveTimeMs);
				out.jointToPos.clear();
				out.jointToPos.push_back({ j, m_params.gripper.openPos });
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
			}
			m_state = State::ReturnHome;
			out.state = m_state;
			out.reason = L"[ReturnHome] after place release.";
			break;
		}

		if (m_placeDownSteps >= std::max(1, m_params.place.maxDownSteps))
		{
			if (m_placeAttempt + 1 < std::max(1, m_params.place.maxAttempts))
			{
				m_placeAttempt++;
				m_state = State::Retreat;
				out.state = m_state;
				m_retreatDone = 0;
				m_retreatTotalSteps = std::max(0, m_params.place.retryRetreatSteps);
				m_retreatNextState = State::Place;
				m_retreatDeltaDeg = -(double)m_params.place.signJ2Down * std::max(0.0, m_params.place.j2DownStepDeg);
				m_placePhase = 0;
				m_centerStableFrames = 0;
				out.reason = std::wstring(L"[Place] down max -> retry attempt=") + std::to_wstring(m_placeAttempt);
			}
			else
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = L"[Abort] place down max steps reached.";
			}
			break;
		}

		if (m_params.place.rangeMode == Params::Place::RangeMode::ArucoDepth && !hasDepth)
		{
			out.reason = L"[Place] waiting depthMm.";
			break;
		}
		if (m_params.place.rangeMode == Params::Place::RangeMode::BboxArea && !boxOk)
		{
			out.reason = L"[Place] waiting bbox.";
			break;
		}
		if (m_params.place.rangeMode == Params::Place::RangeMode::ArucoDepth &&
		    m_depthStableFrames < std::max(1, m_params.approach.depthStableFrames))
		{
			out.reason = L"[Place] depth unstable.";
			break;
		}
		if (m_params.place.rangeMode == Params::Place::RangeMode::BboxArea &&
		    m_boxStableFrames < std::max(1, m_params.place.boxStableFrames))
		{
			out.reason = L"[Place] bbox unstable.";
			break;
		}

		if (in.pKc && in.pMc && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			const double deltaDeg = (double)m_params.place.signJ2Down * std::max(0.0, m_params.place.j2DownStepDeg);
			int pos = -1;
			std::wstring why;
			if (SeeAndFetchJointPolicy::JointDeltaDegToServoPos(*in.pKc, *in.pMc, in.arm, 2, deltaDeg, pos, why))
			{
				out.hasMove = true;
				out.moveTimeMs = std::max(60, m_params.timing.defaultMoveTimeMs);
				out.jointToPos.clear();
				out.jointToPos.push_back({ 2, pos });
				m_placeDownSteps++;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)std::max(0, m_params.timing.lockAfterMoveMs);
				out.reason = std::wstring(L"[Place] down step=") + std::to_wstring(m_placeDownSteps) +
				             (boxOk ? (L" boxA=" + std::to_wstring(areaPx2)) : L"");
			}
			else
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = std::wstring(L"[Abort] place down failed: ") + why;
			}
		}
		else
		{
			out.reason = L"[Place] pacing.";
		}
		break;
	}
	case State::ReturnHome:
	{
		out.active = true;
		out.vsEnable = false;

		if (m_params.ret.returnToStartPose && m_hasStartPosePos)
		{
			out.hasMove = true;
			out.moveTimeMs = std::max(200, m_params.ret.returnTimeMs);
			out.jointToPos.clear();
			// 只恢复 J1-J5 的位置，不包括夹爪（J6），避免在返回时意外开合夹爪
			for (int j = 1; j <= 5; j++)
			{
				const int p = m_startPosePos[(size_t)j];
				if (p >= 0) out.jointToPos.push_back({ j, p });
			}
			m_lastCmdMs = now;
			m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
		}

		ToIdle();
		out.state = m_state;
		out.active = false;
		out.lockManualJog = false;
		out.reason = L"[Done] return.";
		break;
	}
	case State::Abort:
	{
		out.active = false;
		out.vsEnable = false;
		out.reason = L"[Abort] waiting cancel.";
		// 用户取消后回 Idle（上面已处理）
		break;
	}
	case State::EStop:
	{
		out.active = false;
		out.vsEnable = false;
		out.reason = L"[EStop] clear to resume.";
		// 外部应通过 UI 决定如何解除急停（例如再次确认/解锁）
		break;
	}
	default:
	{
		out.active = false;
		out.vsEnable = false;
		out.reason = std::wstring(L"[") + StateName(m_state) + L"] not implemented.";
		break;
	}
	}

	out.state = m_state;
	// 暂停时仅抑制舵机输出，不影响搜索/状态推进
	if (pauseActive && !allowMoveDuringPause)
	{
		out.hasMove = false;
		out.jointToPos.clear();
		out.reason = (m_state == State::SelectTerminal)
			? L"[SelectTerminal] Paused: suppress servo output."
			: L"[SelectGoal] Paused: suppress servo output.";
	}
	if (m_hasCachedPlanePoint)
	{
		out.hasCachedPlanePoint = true;
		out.cachedPlanePointBase = m_cachedPlanePointBase;
	}

	// 尾部补一个简短状态前缀，方便日志统一检索
	out.reason = std::wstring(L"[") + StateName(m_state) + L"] " + out.reason;
	return true;
}









