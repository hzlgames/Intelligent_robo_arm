#include "pch.h"

#include "VisualServoController.h"

#include <algorithm>
#include <cmath>

double VisualServoController::Clamp(double v, double mn, double mx)
{
	if (v < mn) return mn;
	if (v > mx) return mx;
	return v;
}

double VisualServoController::Sign0(double v)
{
	if (v > 0) return 1.0;
	if (v < 0) return -1.0;
	return 0.0;
}

double VisualServoController::Deadband(double v, double db)
{
	if (std::fabs(v) <= db) return 0.0;
	// 连续化：越过死区后减去死区幅度，避免突变
	return v - Sign0(v) * db;
}

double VisualServoController::Lerp(double a, double b, double t)
{
	return a + (b - a) * t;
}

void VisualServoController::MapCamVelToBase(double vxCam, double vyCam, double vzCam,
                                            double& outVxBase, double& outVyBase, double& outVzBase)
{
	// Cam: X右, Y下, Z前
	// Base: X右, Y前, Z上
	outVxBase = vxCam;
	outVyBase = vzCam;
	outVzBase = -vyCam;
}

void VisualServoController::SetEnabled(bool on)
{
	m_enabled = on;
	// 关闭时清掉滤波状态，避免下次打开瞬间跳变
	if (!on)
	{
		std::lock_guard<std::mutex> lk(m_mu);
		m_hasLastOut = false;
	}
}

void VisualServoController::UpdateObservation(const VisualObservation& obs)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_lastObs = obs;
}

void VisualServoController::SetAdvanceCommand(double advanceNorm)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_advanceNorm = Clamp(advanceNorm, -1.0, 1.0);
}

bool VisualServoController::ComputeOutput(VisualServoOutput& out) const
{
	out = VisualServoOutput{};

	if (!m_enabled)
	{
		out.active = false;
		out.reason = L"VisualServo disabled.";
		return true;
	}

	VisualObservation obs;
	double advance = 0.0;
	{
		std::lock_guard<std::mutex> lk(m_mu);
		obs = m_lastObs;
		advance = m_advanceNorm;
	}

	const ULONGLONG now = ::GetTickCount64();
	if (obs.tickMs == 0 || now < obs.tickMs)
	{
		out.active = false;
		out.reason = L"No observation yet.";
		return true;
	}
	const ULONGLONG age = now - obs.tickMs;
	if (age > (ULONGLONG)m_params.maxObsAgeMs)
	{
		out.active = false;
		out.reason = L"Observation stale.";
		return true;
	}
	if (obs.hasConfidence && obs.confidence < m_params.minConfidence)
	{
		out.active = false;
		out.reason = L"Low confidence.";
		return true;
	}

	// 1) 计算像素误差
	double errU = 0.0, errV = 0.0;
	if (obs.hasTargetPx && m_K.valid)
	{
		errU = obs.u - m_K.cx;
		errV = obs.v - m_K.cy;
	}
	else if (obs.hasTargetPx && !m_K.valid)
	{
		// 没有内参时仍允许跑：以 (0,0) 为中心
		errU = obs.u;
		errV = obs.v;
	}

	out.errU = errU;
	out.errV = errV;

	// 死区
	const double eU = Deadband(errU, m_params.deadbandPx);
	const double eV = Deadband(errV, m_params.deadbandPx);

	// 2) 在 Cam 坐标系中生成速度（mm/s）
	double vxCam = 0.0;
	double vyCam = 0.0;
	double vzCam = 0.0;

	const bool wantCenter = (m_mode == VisualServoMode::CenterTarget || m_mode == VisualServoMode::LookAndMove);
	if (wantCenter && obs.hasTargetPx)
	{
		if (obs.hasDepthMm && obs.depthMm > 1e-3 && m_K.valid && m_K.fx > 1e-3 && m_K.fy > 1e-3)
		{
			// 根据针孔模型近似：像素误差 -> 角误差 -> 平移速度
			vxCam = m_params.kPxToVel * eU * (obs.depthMm / m_K.fx);
			vyCam = m_params.kPxToVel * eV * (obs.depthMm / m_K.fy);
		}
		else
		{
			// 无深度/无内参：经验映射（单位当作 mm/s）
			vxCam = m_params.kPxToVel * eU;
			vyCam = m_params.kPxToVel * eV;
		}
	}

	// 3) 沿指向方向前进/后退（FollowRay / LookAndMove）
	const bool wantAdvance = (m_mode == VisualServoMode::FollowRay || m_mode == VisualServoMode::LookAndMove);
	if (wantAdvance)
	{
		bool canAdvance = true;
		if (m_mode == VisualServoMode::LookAndMove)
		{
			// Look-and-move：先居中，居中后才允许前进
			const double mag = std::fabs(errU) + std::fabs(errV);
			canAdvance = (mag <= (m_params.deadbandPx * 2.0));
		}

		if (canAdvance)
		{
			if (obs.hasRay)
			{
				// 归一化射线
				double rx = obs.rayX, ry = obs.rayY, rz = obs.rayZ;
				const double n = std::sqrt(rx * rx + ry * ry + rz * rz);
				if (n > 1e-6) { rx /= n; ry /= n; rz /= n; }
				else { rx = 0.0; ry = 0.0; rz = 1.0; }

				const double spd = m_params.raySpeedMmPerSec * Clamp(advance, -1.0, 1.0);
				vxCam += spd * rx;
				vyCam += spd * ry;
				vzCam += spd * rz;
			}
			else if (obs.hasDepthMm)
			{
				// 沿光轴 Z_cam 做距离闭环：depth -> desiredDepth
				const double dz = obs.depthMm - m_params.desiredDepthMm;
				const double dz2 = Deadband(dz, m_params.depthDeadbandMm);
				vzCam += Clamp(m_params.kDepthToVel * dz2, -m_params.raySpeedMmPerSec, m_params.raySpeedMmPerSec);
			}
		}
	}

	// 4) Cam 速度 -> Base 速度（先用默认映射；未来可替换为外参矩阵）
	double vxBase = 0.0, vyBase = 0.0, vzBase = 0.0;
	MapCamVelToBase(vxCam, vyCam, vzCam, vxBase, vyBase, vzBase);

	// 5) Base 速度 -> Jog 归一化输入
	const double sMax = std::max(1e-3, m_params.maxSpeedMmPerSec);
	out.x = Clamp(vxBase / sMax, -1.0, 1.0);
	out.y = Clamp(vyBase / sMax, -1.0, 1.0);
	out.z = Clamp(vzBase / sMax, -1.0, 1.0);

	// pitch（可选）
	if (m_params.enablePitchFromErrV && obs.hasTargetPx)
	{
		out.pitch = Clamp(m_params.kErrVToPitch * eV, -1.0, 1.0);
	}
	else
	{
		out.pitch = 0.0;
	}

	// active 判定：必须有目标（或有射线/深度闭环）且输出非零或 advance 非零
	out.active = m_enabled && (obs.hasTargetPx || obs.hasRay || obs.hasDepthMm) &&
	             ((std::fabs(out.x) + std::fabs(out.y) + std::fabs(out.z) + std::fabs(out.pitch)) > 1e-6 ||
	              std::fabs(advance) > 1e-6);

	out.reason = out.active ? L"OK" : L"Inactive (no target or zero command).";

	// 6) 低通滤波（减少抖动）
	if (m_params.filterAlpha > 0.0 && m_params.filterAlpha < 1.0)
	{
		std::lock_guard<std::mutex> lk(m_mu);
		if (m_hasLastOut)
		{
			const double a = m_params.filterAlpha;
			out.x = Lerp(m_lastOut.x, out.x, a);
			out.y = Lerp(m_lastOut.y, out.y, a);
			out.z = Lerp(m_lastOut.z, out.z, a);
			out.pitch = Lerp(m_lastOut.pitch, out.pitch, a);
		}
		m_lastOut = out;
		m_hasLastOut = true;
	}

	return true;
}

bool VisualServoController::ApplyToJog(JogController& jog, bool overrideManual, std::wstring& outWhy) const
{
	outWhy.clear();

	VisualServoOutput out;
	if (!ComputeOutput(out))
	{
		outWhy = L"ComputeOutput failed.";
		return false;
	}

	auto in = jog.GetInputState();
	const bool manualActive = in.active;

	if (!overrideManual && manualActive)
	{
		outWhy = L"Manual input active, visual servo suppressed.";
		return true;
	}

	JogController::InputState vin{};
	vin.active = out.active;
	vin.x = out.x;
	vin.y = out.y;
	vin.z = out.z;
	vin.pitch = out.pitch;
	jog.SetInputState(vin);

	// 同步速度上限（让视觉输出的归一化与实际速度一致）
	auto p = jog.GetParams();
	p.speedMmPerSec = m_params.maxSpeedMmPerSec;
	p.pitchDegPerSec = m_params.maxPitchDegPerSec;
	jog.SetParams(p);

	outWhy = out.reason;
	return true;
}


