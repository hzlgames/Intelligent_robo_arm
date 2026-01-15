#include "pch.h"

#include "SeeAndFetchStateMachine.h"
#include "SeeAndFetchJointPolicy.h"

#include "ArmKinematics.h"

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
	case SeeAndFetchStateMachine::State::SelectTerminal: return L"SelectTerminal";
	case SeeAndFetchStateMachine::State::GoInitialPose: return L"GoInitialPose";
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
}

bool SeeAndFetchStateMachine::Tick(const Input& in, const UserCommand& cmd, Output& out)
{
	out = Output{};
	out.state = m_state;

	const ULONGLONG now = ::GetTickCount64();

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
		out.state = m_state;
		out.active = false;
		out.vsEnable = false;
		out.lockManualJog = false;
		out.reason = L"[Cancel] back to Idle.";
		return true;
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
	if (autoActive)
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

	switch (m_state)
	{
	case State::Idle:
	{
		out.active = false;
		out.vsEnable = false;
		out.reason = L"[Idle] waiting confirm.";
		if (cmd.confirm)
		{
			// 进入“手势锁定抓取物”流程（参考 fake_motion_code.md）
			m_state = State::SelectGoal;
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
			out.state = m_state;
			out.reason = L"[SelectGoal] start.";
		}
		break;
	}
	case State::SelectGoal:
	{
		out.active = true;
		out.vsEnable = false;
		// 需要 HandLandmarks 来得到手势/指向，并用 Detector 候选做 PointPick
		out.requestVisionMode = 6; // VisionService::Mode::HandLandmarks
		out.requestPointPickTarget = 0; // Detector candidates only

		// Follow hand: keep fingertip near center (u,v in HandLandmarks mode is fingertip)
		if (in.pKc && in.pMc && in.hasObs && in.obs.hasTargetPx && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
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

		// Confirmed: record initial_pos and proceed to selecting terminal red dot
		if (in.pickState == 3 && in.hasPickBox)
		{
			if (!in.hasServoPos)
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = L"[Abort] no servoPos for initial_pos.";
				break;
			}
			m_initialPosePos = in.servoPos;
			m_hasInitialPosePos = true;
			// reset pick for next stage
			out.requestPointPickReset = true;
			m_state = State::SelectTerminal;
			out.state = m_state;
			out.reason = L"[SelectTerminal] goal confirmed; now pick red dot.";
		}
		else
		{
			out.reason = L"[SelectGoal] point to object; pinch to confirm; open palm to cancel.";
		}
		break;
	}
	case State::SelectTerminal:
	{
		out.active = true;
		out.vsEnable = false;
		// 仍用 HandLandmarks 获取手势，但 PointPick 目标改为“红点候选”
		out.requestVisionMode = 6; // HandLandmarks
		out.requestPointPickTarget = 1; // red dot

		// Follow hand
		if (in.pKc && in.pMc && in.hasObs && in.obs.hasTargetPx && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
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

		// Confirmed: record terminal_pos and go back to initial_pos before starting grab
		if (in.pickState == 3 && in.hasPickBox)
		{
			if (!in.hasServoPos)
			{
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
				out.reason = L"[Abort] no servoPos for terminal_pos.";
				break;
			}
			m_terminalPosePos = in.servoPos;
			m_hasTerminalPosePos = true;
			out.requestPointPickReset = true;
			m_state = State::GoInitialPose;
			out.state = m_state;
			m_goPosePhase = 0;
			out.reason = L"[GoInitialPose] terminal confirmed; go back to initial_pos.";
		}
		else
		{
			out.reason = L"[SelectTerminal] point to red dot; pinch to confirm; open palm to cancel.";
		}
		break;
	}
	case State::GoInitialPose:
	{
		out.active = true;
		out.vsEnable = false;

		if (!m_hasInitialPosePos)
		{
			m_state = State::Abort;
			out.state = m_state;
			out.active = false;
			out.reason = L"[Abort] missing initial_pos.";
			break;
		}

		// issue one absolute move, then wait for lock
		if (m_goPosePhase == 0 && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			out.hasMove = true;
			out.moveTimeMs = std::max(200, m_params.ret.returnTimeMs);
			out.jointToPos.clear();
			for (int j = 1; j <= MotionConfig::kJointCount; j++)
			{
				const int p = m_initialPosePos[(size_t)j];
				if (p >= 0) out.jointToPos.push_back({ j, p });
			}
			m_lastCmdMs = now;
			m_lockedUntilMs = now + (ULONGLONG)std::max(0, out.moveTimeMs);
			m_goPosePhase = 1;
			out.reason = L"[GoInitialPose] moving...";
			break;
		}

		if (m_goPosePhase == 1 && m_lockedUntilMs != 0 && now >= m_lockedUntilMs)
		{
			// start normal tracking/grab
			m_state = State::Acquire;
			out.state = m_state;
			m_stableFrames = 0;
			m_lostFrames = 0;
			m_goPosePhase = 0; // allow GoTerminalPose later
			out.reason = L"[Acquire] start after go initial.";
			break;
		}

		out.reason = L"[GoInitialPose] waiting...";
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
			out.reason = (m_state == State::Track) ? L"[Track] retry." : L"[Place] enter.";
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
			for (int j = 1; j <= MotionConfig::kJointCount; j++)
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
			for (int j = 1; j <= MotionConfig::kJointCount; j++)
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
	if (m_hasCachedPlanePoint)
	{
		out.hasCachedPlanePoint = true;
		out.cachedPlanePointBase = m_cachedPlanePointBase;
	}

	// 尾部补一个简短状态前缀，方便日志统一检索
	out.reason = std::wstring(L"[") + StateName(m_state) + L"] " + out.reason;
	return true;
}









