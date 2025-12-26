#include "pch.h"

#include "JogController.h"

#include "ArmCommsService.h"

#include <algorithm>
#include <cmath>

namespace
{
	double Clamp(double v, double mn, double mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}

	double ClampStep(double v, double maxAbs)
	{
		if (v > maxAbs) return maxAbs;
		if (v < -maxAbs) return -maxAbs;
		return v;
	}
}

JogController::JogController()
{
	m_lastTick = ::GetTickCount64();
}

void JogController::Bind(MotionController* pMotion, KinematicsConfig* pKc)
{
	m_pMotion = pMotion;
	m_pKc = pKc;
}

bool JogController::BuildCurrentJointEstimate(const MotionConfig& mc,
                                              const KinematicsConfig& kc,
                                              ArmKinematics::JointAnglesRad& outQ)
{
	// 说明：为了让 Jog 的“择优解”更稳定，我们尽量从回读缓存估算当前关节角。
	// 若没有回读缓存，则退化为 homePos。
	for (int j = 0; j <= ArmKinematics::kJointCount; j++)
	{
		outQ.q[j] = 0.0;
	}

	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		const auto& jc = mc.Get(j);
		int pos = jc.homePos;
		if (jc.servoId >= 1 && jc.servoId <= 6)
		{
			uint16_t rb = 0;
			if (ArmCommsService::Instance().GetLastReadPos((uint8_t)jc.servoId, rb))
			{
				pos = (int)rb;
			}
		}

		double rad = 0.0;
		if (!ArmKinematics::ServoPosToJointRad(kc, &mc, j, pos, rad))
		{
			// 标定不足时给 0，后续 IK 仍可跑，但会更不稳定
			rad = 0.0;
		}
		outQ.q[j] = rad;
	}
	return true;
}

bool JogController::Tick(std::wstring& outWhy)
{
	outWhy.clear();
	if (!m_pMotion || !m_pKc)
	{
		outWhy = L"JogController 未绑定依赖（MotionController/KinematicsConfig）。";
		return false;
	}

	const ULONGLONG now = ::GetTickCount64();
	const int hz = (m_params.sendHz <= 0) ? 20 : m_params.sendHz;
	const ULONGLONG periodMs = (ULONGLONG)(1000 / hz);
	if (now - m_lastTick < periodMs)
	{
		return true; // 未到周期
	}
	m_lastTick = now;

	// deadman 未按住则不发送
	if (!m_input.active)
	{
		return true;
	}

	// 将输入转成速度（mm/s, deg/s）
	const double vx = Clamp(m_input.x, -1.0, 1.0) * m_params.speedMmPerSec;
	const double vy = Clamp(m_input.y, -1.0, 1.0) * m_params.speedMmPerSec;
	const double vz = Clamp(m_input.z, -1.0, 1.0) * m_params.speedMmPerSec;
	const double vp = Clamp(m_input.pitch, -1.0, 1.0) * m_params.pitchDegPerSec;

	const double dt = (double)periodMs / 1000.0;
	double dx = vx * dt;
	double dy = vy * dt;
	double dz = vz * dt;
	double dp = vp * dt;

	// 步长限制（每tick）
	dx = ClampStep(dx, m_params.maxStepMm);
	dy = ClampStep(dy, m_params.maxStepMm);
	dz = ClampStep(dz, m_params.maxStepMm);
	dp = ClampStep(dp, m_params.maxStepPitchDeg);

	// 先缓存上一帧 target。IK 失败时必须回滚，避免 target 漂移到不可达区域导致“越失败越不可达”。
	const auto prevTarget = m_target;
	ArmKinematics::PoseTarget nextTarget = m_target;
	nextTarget.x_mm += dx;
	nextTarget.y_mm += dy;
	nextTarget.z_mm += dz;
	nextTarget.pitch_deg += dp;

	// 读取当前关节角估算，用于 IK 择优
	ArmKinematics::JointAnglesRad qCur;
	BuildCurrentJointEstimate(m_pMotion->Config(), *m_pKc, qCur);

	// IK
	const auto ik = ArmKinematics::InverseKinematics(*m_pKc, &m_pMotion->Config(), nextTarget, &qCur);
	if (!ik.ok)
	{
		// IK 失败：回滚/恢复到上一次可达位置，避免 target 漂移导致持续失败。
		m_target = m_hasLastGoodTarget ? m_lastGoodTarget : prevTarget;
		outWhy = ik.reason.empty() ? L"IK 失败。" : ik.reason;
		return false;
	}

	// 软限位一致性：若最佳解超出软限位，则本次视为失败并回滚。
	// 否则下发阶段会被 MotionController clamp，导致“看起来在动但实际被裁剪”，闭环会持续积累误差。
	if (ik.chosenIndex >= 0 &&
	    ik.chosenIndex < (int)ik.candidates.size() &&
	    !ik.candidates[ik.chosenIndex].withinLimits)
	{
		m_target = m_hasLastGoodTarget ? m_lastGoodTarget : prevTarget;
		outWhy = ik.reason.empty() ? L"软限位抑制：目标超出关节限位范围。" : ik.reason;
		return false;
	}

	// 只有 IK 成功才提交本次 target
	m_target = nextTarget;
	m_lastGoodTarget = m_target;
	m_hasLastGoodTarget = true;

	// 关节角 -> 舵机位置
	ArmKinematics::ServoPos sp;
	std::wstring why;
	if (!ArmKinematics::JointAnglesToServoPos(*m_pKc, &m_pMotion->Config(), ik.chosenQ, sp, why))
	{
		outWhy = why.empty() ? L"角度转舵机位置失败。" : why;
		return false;
	}

	// “最新指令优先”：只清理 Move 队列，避免误清读回/脚本/其他指令
	ArmCommsService::Instance().ClearMoveQueue();

	std::vector<std::pair<int, int>> jointToPos;
	jointToPos.reserve(ArmKinematics::kJointCount);
	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		if (sp.pos[j] < 0) continue;
		jointToPos.push_back({ j, sp.pos[j] });
	}

	const int timeMs = (int)std::max<ULONGLONG>(periodMs, 30ULL); // 稍大于节拍，避免舵机抖动
	if (!m_pMotion->MoveJointsAbs(jointToPos, timeMs))
	{
		outWhy = L"下发失败：未配置 ServoId 或无有效关节目标。";
		return false;
	}

	return true;
}



