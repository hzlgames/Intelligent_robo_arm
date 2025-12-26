#include "pch.h"

#include "SeeAndFetchStateMachine.h"

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
		case SeeAndFetchStateMachine::State::Place: return L"Place";
		case SeeAndFetchStateMachine::State::Abort: return L"Abort";
		case SeeAndFetchStateMachine::State::EStop: return L"EStop";
		default: return L"?";
		}
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
}

bool SeeAndFetchStateMachine::Tick(const Input& in, const UserCommand& cmd, Output& out)
{
	out = Output{};
	out.state = m_state;

	// 最高优先级：急停
	if (cmd.eStop)
	{
		m_state = State::EStop;
		out.state = m_state;
		out.active = false;
		out.vsEnable = false;
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
		out.reason = L"[Cancel] back to Idle.";
		return true;
	}

	const bool hasTarget = in.hasObs && (in.obs.hasTargetPx || in.obs.hasRay || in.obs.hasDepthMm);

	switch (m_state)
	{
	case State::Idle:
	{
		out.active = false;
		out.vsEnable = false;
		out.reason = L"[Idle] waiting confirm.";
		if (cmd.confirm)
		{
			m_state = State::Acquire;
			m_stableFrames = 0;
			m_lostFrames = 0;
			out.state = m_state;
			out.reason = L"[Acquire] start.";
		}
		break;
	}
	case State::Acquire:
	{
		out.active = true;
		out.vsEnable = true;
		out.vsMode = VisualServoMode::CenterTarget;
		out.vsAdvance = 0.0;

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
		out.vsEnable = true;
		out.vsMode = VisualServoMode::LookAndMove;
		out.vsAdvance = 0.0;

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
		out.reason = std::wstring(L"[Track] ok planeCache=") + (m_hasCachedPlanePoint ? L"Y" : L"N");
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


