#include "pch.h"

#include "GrabTestController.h"

#include "ArmKinematics.h"

#include <cmath>

namespace
{
	inline bool CanIssueMoveNow(const GrabTestController::Params& P, ULONGLONG now, ULONGLONG lockedUntil, ULONGLONG lastCmd)
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

void GrabTestController::Reset()
{
	ToIdle();
}

void GrabTestController::ToIdle()
{
	m_state = State::Idle;
	m_stableFrames = 0;
	m_lostFrames = 0;
	m_hasEverTarget = false;
	m_centerStableFrames = 0;
	m_timeToFetchStableFrames = 0;
	m_advanceSteps = 0;
	m_phase = 0;
	m_waitObsAfterMove = false;
	m_lastObsTickMs = 0;
	m_lockedUntilMs = 0;
	m_lastCmdMs = 0;
	m_gripSteps = 0;
	m_gripCmdPos = 0;
	m_retreatDone = 0;
	m_retreatTotalSteps = 0;
	m_retreatDeltaDeg = 0.0;
}

bool GrabTestController::ComputeCenterStepJ1J4(const Input& in, std::vector<std::pair<int, int>>& moves, std::wstring& why)
{
	why.clear();
	if (!in.pKc || !in.pMc) return false;
	if (!in.arm.valid) { why = L"arm invalid"; return false; }
	if (!in.hasObs || !in.obs.hasTargetPx) { why = L"no target"; return false; }

	const double cx = (double)in.frameW * 0.5 + (double)m_params.find.centerOffsetU;
	const double cy = (double)in.frameH * 0.5 + (double)m_params.find.centerOffsetV;
	const double errU = in.obs.u - cx;
	const double errV = in.obs.v - cy;
	const int db = std::max(0, m_params.find.deadbandPx);
	const int minPosDelta = std::max(0, m_params.find.minServoPosChange);

	auto deltaToPos = [&](int joint, double deltaDeg, int& outPos) -> bool
	{
		if (!in.arm.valid) return false;
		ArmKinematics::JointAnglesRad q = in.arm.q;
		const double curDeg = q.q[joint] * (180.0 / 3.14159265358979323846);
		const double newDeg = curDeg + deltaDeg;
		q.q[joint] = newDeg * (3.14159265358979323846 / 180.0);
		return ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, joint, q.q[joint], outPos);
	};

	if (std::fabs(errU) > db)
	{
		const double step = StepFromPx(std::fabs(errU),
		                               m_params.find.yaw_kDegPerPx,
		                               m_params.find.yaw_minStepDeg,
		                               m_params.find.yaw_maxStepDeg);
		const double sgn = (errU > 0) ? 1.0 : -1.0;
		const double delta = sgn * (double)m_params.find.signJ1FromErrU * step;
		int pos = -1;
		if (deltaToPos(1, delta, pos))
		{
			int curPos = -1;
			(void)ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, 1, in.arm.q.q[1], curPos);
			if (curPos < 0 || minPosDelta <= 0 || std::abs(curPos - pos) >= minPosDelta)
			{
				moves.push_back({ 1, pos });
			}
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
		int pos = -1;
		if (deltaToPos(4, delta, pos))
		{
			int curPos = -1;
			(void)ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, 4, in.arm.q.q[4], curPos);
			if (curPos < 0 || minPosDelta <= 0 || std::abs(curPos - pos) >= minPosDelta)
			{
				moves.push_back({ 4, pos });
			}
		}
	}

	return !moves.empty();
}

bool GrabTestController::ComputeCenterStepJ3(const Input& in, std::vector<std::pair<int, int>>& moves, std::wstring& why)
{
	why.clear();
	if (!in.pKc || !in.pMc) return false;
	if (!in.arm.valid) { why = L"arm invalid"; return false; }
	if (!in.hasObs || !in.obs.hasTargetPx) { why = L"no target"; return false; }

	const double cx = (double)in.frameW * 0.5 + (double)m_params.find.centerOffsetU;
	const double cy = (double)in.frameH * 0.5 + (double)m_params.find.centerOffsetV;
	const double errV = in.obs.v - cy;
	const int db = std::max(0, m_params.find.deadbandPx);
	const int minPosDelta = std::max(0, m_params.find.minServoPosChange);

	if (std::fabs(errV) <= db) return false;

	const double step = StepFromPx(std::fabs(errV),
	                               m_params.find.j3_kDegPerPx,
	                               m_params.find.j3_minStepDeg,
	                               m_params.find.j3_maxStepDeg);
	const double sgn = (errV > 0) ? 1.0 : -1.0;
	const double delta = sgn * (double)m_params.find.signJ3FromErrV * step;

	ArmKinematics::JointAnglesRad q = in.arm.q;
	const double curDeg = q.q[3] * (180.0 / 3.14159265358979323846);
	const double newDeg = curDeg + delta;
	q.q[3] = newDeg * (3.14159265358979323846 / 180.0);
	int pos = -1;
	if (ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, 3, q.q[3], pos))
	{
		int curPos = -1;
		(void)ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, 3, in.arm.q.q[3], curPos);
		if (curPos < 0 || minPosDelta <= 0 || std::abs(curPos - pos) >= minPosDelta)
		{
			moves.push_back({ 3, pos });
		}
	}

	return !moves.empty();
}

bool GrabTestController::ComputeAdvanceStep(const Input& in, int& outPos, std::wstring& why)
{
	why.clear();
	outPos = -1;
	if (!in.pKc || !in.pMc) { why = L"no config"; return false; }
	if (!in.arm.valid) { why = L"arm invalid"; return false; }

	const double deltaDeg = (double)m_params.approach.signJ2Advance * std::max(0.0, m_params.approach.j2AdvanceStepDeg);
	ArmKinematics::JointAnglesRad q = in.arm.q;
	const double curDeg = q.q[2] * (180.0 / 3.14159265358979323846);
	const double newDeg = curDeg + deltaDeg;
	q.q[2] = newDeg * (3.14159265358979323846 / 180.0);
	if (!ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, 2, q.q[2], outPos))
	{
		why = L"JointRadToServoPos failed";
		return false;
	}
	return true;
}

bool GrabTestController::Tick(const Input& in, const UserCommand& cmd, Output& out)
{
	out = Output{};
	out.state = m_state;

	const ULONGLONG now = ::GetTickCount64();
	const bool hasTarget = in.hasObs && in.obs.hasTargetPx;

	if (cmd.eStop)
	{
		m_state = State::EStop;
		out.state = m_state;
		out.active = false;
		out.reason = L"[EStop] requested.";
		return true;
	}
	if (cmd.cancel)
	{
		ToIdle();
		out.state = m_state;
		out.active = false;
		out.reason = L"[Idle] cancelled.";
		return true;
	}

	switch (m_state)
	{
	case State::Idle:
	{
		out.active = false;
		out.reason = L"[Idle] waiting start.";
		if (cmd.start)
		{
			m_state = State::Acquire;
			m_stableFrames = 0;
			m_lostFrames = 0;
			m_hasEverTarget = false;
			m_centerStableFrames = 0;
			m_timeToFetchStableFrames = 0;
			m_advanceSteps = 0;
			m_gripSteps = 0;
			m_gripCmdPos = 0;
			m_phase = 0;
			m_waitObsAfterMove = false;
			m_lastObsTickMs = 0;
			out.state = m_state;
			out.reason = L"[Acquire] start.";
		}
		break;
	}
	case State::Acquire:
	{
		out.active = true;
		if (hasTarget) { m_stableFrames++; m_lostFrames = 0; }
		else { m_lostFrames++; m_stableFrames = 0; }
		if (hasTarget) m_hasEverTarget = true;
		if (hasTarget && in.obs.tickMs != 0) m_lastObsTickMs = in.obs.tickMs;

		if (m_hasEverTarget && m_lostFrames > m_params.lostFramesToAbort)
		{
			m_state = State::Abort;
			out.state = m_state;
			out.active = false;
			out.reason = L"[Abort] acquire lost target.";
			break;
		}

		if (hasTarget && CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			std::vector<std::pair<int, int>> moves;
			std::wstring why;
			if (ComputeCenterStepJ1J4(in, moves, why))
			{
				out.hasMove = true;
				out.moveTimeMs = std::max(30, m_params.timing.defaultMoveTimeMs);
				out.jointToPos = moves;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
				out.requestGeminiReset = true;
				m_waitObsAfterMove = true;
				m_lastObsTickMs = in.obs.tickMs;
			}
		}

		if (m_stableFrames >= m_params.acquireStableFrames)
		{
			m_state = State::Approach;
			out.state = m_state;
			m_phase = 0;
			m_timeToFetchStableFrames = 0;
			out.reason = L"[Approach] locked.";
		}
		else
		{
			out.reason = std::wstring(L"[Acquire] stable=") + std::to_wstring(m_stableFrames) +
			             L" lost=" + std::to_wstring(m_lostFrames);
		}
		break;
	}
	case State::Approach:
	{
		out.active = true;
		if (m_waitObsAfterMove)
		{
			if (in.hasObs && in.obs.tickMs != 0 && in.obs.tickMs != m_lastObsTickMs)
			{
				m_waitObsAfterMove = false;
			}
			else
			{
				out.reason = L"[Approach] waiting Gemini update.";
				break;
			}
		}
		if (!hasTarget)
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

		if (in.hasTimeToFetch && in.timeToFetch == 1)
		{
			m_timeToFetchStableFrames++;
		}
		else
		{
			m_timeToFetchStableFrames = 0;
		}

		if (m_timeToFetchStableFrames >= std::max(1, m_params.approach.timeToFetchStableFrames))
		{
			m_state = State::Grasp;
			out.state = m_state;
			m_gripSteps = 0;
			m_gripCmdPos = 0;
			out.reason = L"[Grasp] TimeToFetch=1.";
			break;
		}

		if (m_advanceSteps >= std::max(1, m_params.approach.maxAdvanceSteps))
		{
			m_state = State::Abort;
			out.state = m_state;
			out.active = false;
			out.reason = L"[Abort] approach max steps reached.";
			break;
		}

		const double cx = (double)in.frameW * 0.5 + (double)m_params.find.centerOffsetU;
		const double cy = (double)in.frameH * 0.5 + (double)m_params.find.centerOffsetV;
		const double errU = in.obs.u - cx;
		const double errV = in.obs.v - cy;
		const int coarse = std::max(0, m_params.find.coarseCenterPx);
		const bool coarseCentered = (std::fabs(errU) <= (double)coarse && std::fabs(errV) <= (double)coarse);

		if (CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			if (m_phase == 0)
			{
				std::vector<std::pair<int, int>> moves;
				std::wstring why;
				if (ComputeCenterStepJ1J4(in, moves, why))
				{
					out.hasMove = true;
					out.moveTimeMs = std::max(60, m_params.timing.defaultMoveTimeMs);
					out.jointToPos = moves;
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
					out.requestGeminiReset = true;
					m_waitObsAfterMove = true;
					m_lastObsTickMs = in.obs.tickMs;
					out.reason = L"[Approach] center J1/J4.";
					break;
				}
				if (coarseCentered)
				{
					m_phase = 1;
				}
				out.reason = L"[Approach] center check.";
				break;
			}
			else if (m_phase == 1)
			{
				int pos = -1;
				std::wstring why;
				if (ComputeAdvanceStep(in, pos, why))
				{
					out.hasMove = true;
					out.moveTimeMs = std::max(60, m_params.timing.defaultMoveTimeMs);
					out.jointToPos = { {2, pos} };
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
					m_advanceSteps++;
					out.requestGeminiReset = true;
					m_waitObsAfterMove = true;
					m_lastObsTickMs = in.obs.tickMs;
					m_phase = 2;
					out.reason = std::wstring(L"[Approach] J2 step=") + std::to_wstring(m_advanceSteps);
					break;
				}
				out.reason = std::wstring(L"[Approach] ") + why;
				break;
			}
			else
			{
				std::vector<std::pair<int, int>> moves;
				std::wstring why;
				if (ComputeCenterStepJ3(in, moves, why))
				{
					out.hasMove = true;
					out.moveTimeMs = std::max(60, m_params.timing.defaultMoveTimeMs);
					out.jointToPos = moves;
					m_lastCmdMs = now;
					m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
					out.requestGeminiReset = true;
					m_waitObsAfterMove = true;
					m_lastObsTickMs = in.obs.tickMs;
					out.reason = L"[Approach] center J3.";
					break;
				}
				if (coarseCentered)
				{
					m_phase = 1;
				}
				out.reason = L"[Approach] J3 check.";
				break;
			}
		}
		out.reason = L"[Approach] pacing.";
		break;
	}
	case State::Grasp:
	{
		out.active = true;
		const int j = std::max(1, std::min(MotionConfig::kJointCount, m_params.gripper.jointIndex));
		const int openPos = m_params.gripper.openPos;
		const int closePos = m_params.gripper.closePos;
		const int step = std::max(1, m_params.gripper.closeStepPos);
		const int maxSteps = std::max(1, m_params.gripper.maxCloseSteps);
		const int moveTime = std::max(60, m_params.gripper.closeMoveTimeMs);

		if (m_gripCmdPos == 0) m_gripCmdPos = openPos;
		const int dir = (closePos < m_gripCmdPos) ? -1 : +1;
		int nextPos = m_gripCmdPos + dir * step;
		if ((dir < 0 && nextPos < closePos) || (dir > 0 && nextPos > closePos)) nextPos = closePos;

		if (CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			out.hasMove = true;
			out.moveTimeMs = moveTime;
			out.jointToPos = { { j, nextPos } };
			m_gripCmdPos = nextPos;
			m_gripSteps++;
			m_lastCmdMs = now;
			m_lockedUntilMs = now + (ULONGLONG)std::max(0, moveTime);
		}

		out.reason = std::wstring(L"[Grasp] step=") + std::to_wstring(m_gripSteps) +
		             L" pos=" + std::to_wstring(m_gripCmdPos);

		const bool graspDone = (m_gripCmdPos == closePos) || (m_gripSteps >= maxSteps);
		if (graspDone)
		{
			m_state = State::Retreat;
			out.state = m_state;
			m_retreatDone = 0;
			m_retreatTotalSteps = 6;
			m_retreatDeltaDeg = -(double)m_params.approach.signJ2Advance * std::max(0.0, m_params.approach.j2AdvanceStepDeg);
			out.reason = L"[Retreat] enter.";
		}
		break;
	}
	case State::Retreat:
	{
		out.active = true;
		const int steps = std::max(0, m_retreatTotalSteps);
		if (m_retreatDone >= steps)
		{
			m_state = State::ReturnHome;
			out.state = m_state;
			out.reason = L"[ReturnHome] enter.";
			break;
		}
		if (CanIssueMoveNow(m_params, now, m_lockedUntilMs, m_lastCmdMs))
		{
			int pos = -1;
			std::wstring why;
			ArmKinematics::JointAnglesRad q = in.arm.q;
			const double curDeg = q.q[2] * (180.0 / 3.14159265358979323846);
			const double newDeg = curDeg + m_retreatDeltaDeg;
			q.q[2] = newDeg * (3.14159265358979323846 / 180.0);
			if (ArmKinematics::JointRadToServoPos(*in.pKc, in.pMc, 2, q.q[2], pos))
			{
				out.hasMove = true;
				out.moveTimeMs = std::max(60, m_params.timing.defaultMoveTimeMs);
				out.jointToPos = { { 2, pos } };
				m_retreatDone++;
				m_lastCmdMs = now;
				m_lockedUntilMs = now + (ULONGLONG)m_params.timing.lockAfterMoveMs;
				out.reason = std::wstring(L"[Retreat] step=") + std::to_wstring(m_retreatDone) +
				             L"/" + std::to_wstring(steps);
			}
			else
			{
				out.reason = L"[Abort] retreat failed.";
				m_state = State::Abort;
				out.state = m_state;
				out.active = false;
			}
		}
		else
		{
			out.reason = L"[Retreat] pacing.";
		}
		break;
	}
	case State::ReturnHome:
	{
		out.active = true;
		if (m_params.ret.returnToStartPose && m_hasStartPosePos)
		{
			out.hasMove = true;
			out.moveTimeMs = std::max(200, m_params.ret.returnTimeMs);
			out.jointToPos.clear();
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
		out.reason = L"[Done] return.";
		break;
	}
	case State::Abort:
		out.active = false;
		out.reason = L"[Abort] waiting cancel.";
		if (cmd.start)
		{
			m_state = State::Acquire;
			out.state = m_state;
			out.active = true;
			m_stableFrames = 0;
			m_lostFrames = 0;
			m_hasEverTarget = false;
			m_centerStableFrames = 0;
			m_timeToFetchStableFrames = 0;
			m_advanceSteps = 0;
			m_gripSteps = 0;
			m_gripCmdPos = 0;
			m_phase = 0;
			m_waitObsAfterMove = false;
			m_lastObsTickMs = 0;
			out.reason = L"[Acquire] restart.";
		}
		break;
	case State::EStop:
		out.active = false;
		out.reason = L"[EStop] clear to resume.";
		break;
	default:
		out.active = false;
		out.reason = L"[Unknown] not implemented.";
		break;
	}

	out.state = m_state;
	return true;
}

