#include "pch.h"

#include "SeeAndFetchJointPolicy.h"

#include "ArmKinematics.h"

#include <algorithm>
#include <cmath>

namespace
{
	constexpr double kPi = 3.14159265358979323846;
	inline double DegToRad(double d) { return d * (kPi / 180.0); }
	inline double RadToDeg(double r) { return r * (180.0 / kPi); }

	inline double Clamp(double v, double mn, double mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}

	inline double Sign0(double v)
	{
		if (v > 0.0) return 1.0;
		if (v < 0.0) return -1.0;
		return 0.0;
	}

	inline double StepFromPx(double absErrPx, double kDegPerPx, double minDeg, double maxDeg)
	{
		const double s = kDegPerPx * absErrPx;
		return Clamp(s, minDeg, maxDeg);
	}

	inline bool AddJointMove(const KinematicsConfig& kc,
	                         const MotionConfig& mc,
	                         const ArmStateEstimator::ArmState& arm,
	                         int joint,
	                         double deltaDeg,
	                         int minPosChange,
	                         std::vector<std::pair<int, int>>& outJointToPos,
	                         std::wstring& outWhy)
	{
		outWhy.clear();
		if (joint < 1 || joint > MotionConfig::kJointCount)
		{
			outWhy = L"invalid joint index.";
			return false;
		}
		if (!arm.valid)
		{
			outWhy = L"arm pose invalid (readback/kinematics not ready).";
			return false;
		}

		// 获取当前位置
		int curPos = -1;
		if (!ArmKinematics::JointRadToServoPos(kc, &mc, joint, arm.q.q[joint], curPos))
		{
			outWhy = L"JointRadToServoPos(current) failed.";
			return false;
		}

		ArmKinematics::JointAnglesRad q = arm.q; // copy
		const double curDeg = RadToDeg(q.q[joint]);
		const double newDeg = curDeg + deltaDeg;
		q.q[joint] = DegToRad(newDeg);

		int outPos = -1;
		if (!ArmKinematics::JointRadToServoPos(kc, &mc, joint, q.q[joint], outPos))
		{
			outWhy = L"JointRadToServoPos failed (check kinematics calibration).";
			return false;
		}

		// 检查位置变化是否足够大
		const int posDelta = std::abs(outPos - curPos);
		if (minPosChange > 0 && posDelta < minPosChange)
		{
			// 位置变化太小，舵机可能不响应，跳过此关节
			// 返回 true 表示没有错误，只是不产生动作
			return true;
		}

		outJointToPos.push_back({ joint, outPos });
		return true;
	}
}

bool SeeAndFetchJointPolicy::JointDeltaDegToServoPos(const KinematicsConfig& kc,
                                                     const MotionConfig& mc,
                                                     const ArmStateEstimator::ArmState& arm,
                                                     int joint,
                                                     double deltaDeg,
                                                     int& outPos,
                                                     std::wstring& outWhy)
{
	outWhy.clear();
	outPos = -1;
	if (joint < 1 || joint > MotionConfig::kJointCount)
	{
		outWhy = L"invalid joint index.";
		return false;
	}
	if (!arm.valid)
	{
		outWhy = L"arm pose invalid (readback/kinematics not ready).";
		return false;
	}

	ArmKinematics::JointAnglesRad q = arm.q;
	const double curDeg = RadToDeg(q.q[joint]);
	const double newDeg = curDeg + deltaDeg;
	q.q[joint] = DegToRad(newDeg);
	if (!ArmKinematics::JointRadToServoPos(kc, &mc, joint, q.q[joint], outPos))
	{
		outWhy = L"JointRadToServoPos failed (check kinematics calibration).";
		return false;
	}
	return true;
}

SeeAndFetchJointPolicy::StepResult SeeAndFetchJointPolicy::ComputeFindStep(const SeeAndFetchStateMachine::Params& P,
                                                                          const KinematicsConfig& kc,
                                                                          const MotionConfig& mc,
                                                                          const ArmStateEstimator::ArmState& arm,
                                                                          const VisualObservation& obs,
                                                                          UINT frameW,
                                                                          UINT frameH)
{
	StepResult r;
	r.ok = true;
	r.moveTimeMs = std::max(30, P.timing.defaultMoveTimeMs);

	if (!arm.valid)
	{
		r.ok = true;
		r.hasMove = false;
		r.why = L"Arm pose invalid; skip.";
		return r;
	}
	if (!obs.hasTargetPx)
	{
		r.ok = true;
		r.hasMove = false;
		r.why = L"No target px.";
		return r;
	}
	if (frameW == 0 || frameH == 0)
	{
		r.ok = true;
		r.hasMove = false;
		r.why = L"No frame size.";
		return r;
	}

	// 安全检查：观测坐标必须在帧范围内，否则可能是检测异常
	// 超出范围时不产生运动，避免关节突然跳变
	if (obs.u < 0.0 || obs.u > (double)frameW || obs.v < 0.0 || obs.v > (double)frameH)
	{
		r.ok = true;
		r.hasMove = false;
		r.why = L"Target px out of frame bounds; skip.";
		return r;
	}

	const double cx = (double)frameW * 0.5 + (double)P.find.centerOffsetU;
	const double cy = (double)frameH * 0.5 + (double)P.find.centerOffsetV;
	const double errU = obs.u - cx;
	const double errV = obs.v - cy;
	r.errU = errU;
	r.errV = errV;

	const int db = std::max(0, P.find.deadbandPx);
	const bool inU = std::fabs(errU) <= (double)db;
	const bool inV = std::fabs(errV) <= (double)db;
	r.centeredNow = inU && inV;
	if (r.centeredNow)
	{
		r.hasMove = false;
		r.why = L"Centered.";
		return r;
	}

	// Build one step that can include yaw + pitch.
	std::wstring why;
	std::vector<std::pair<int, int>> jointToPos;
	jointToPos.reserve(2);

	// ---- J1 yaw (errU) ----
	if (!inU)
	{
		const double sgn = Sign0(errU);
		const double step = StepFromPx(std::fabs(errU), P.find.yaw_kDegPerPx, P.find.yaw_minStepDeg, P.find.yaw_maxStepDeg);
		const double deltaDeg = (double)P.find.signJ1FromErrU * sgn * step;
		if (!AddJointMove(kc, mc, arm, 1, deltaDeg, P.find.minServoPosChange, jointToPos, why))
		{
			r.ok = false;
			r.hasMove = false;
			r.why = std::wstring(L"Find(J1) failed: ") + why;
			return r;
		}
	}

	// ---- J4 or J3 pitch (errV) ----
	// 使用滞后机制避免 J3/J4 频繁切换导致方向抖动
	if (!inV)
	{
		const double q4Deg = RadToDeg(arm.q.q[4]);
		const double absQ4 = std::fabs(q4Deg);
		const double threshold = std::fabs(P.find.j4PreferAbsDeg);
		const double hysteresis = std::fabs(P.find.j4SwitchHysteresisDeg);

		// 使用静态变量记住当前使用的关节（J4=true, J3=false）
		// 初始状态基于当前 J4 角度判断
		static bool s_usingJ4 = true;

		// 滞后切换逻辑：
		// - 如果当前使用 J4，只有当 |q4| > threshold + hysteresis 时才切换到 J3
		// - 如果当前使用 J3，只有当 |q4| < threshold - hysteresis 时才切换回 J4
		if (s_usingJ4)
		{
			// 当前使用 J4，检查是否需要切换到 J3
			if (absQ4 > threshold + hysteresis)
			{
				s_usingJ4 = false; // 切换到 J3
			}
		}
		else
		{
			// 当前使用 J3，检查是否需要切换回 J4
			if (absQ4 < threshold - hysteresis)
			{
				s_usingJ4 = true; // 切换回 J4
			}
			// 特殊情况：如果 hysteresis 很大，threshold - hysteresis 可能为负
			// 这种情况下，当 J4 回到接近零位时也应该切换回 J4
			else if (threshold <= hysteresis && absQ4 < 5.0)
			{
				s_usingJ4 = true;
			}
		}

		const int joint = s_usingJ4 ? 4 : 3;
		const int sign = s_usingJ4 ? P.find.signJ4FromErrV : P.find.signJ3FromErrV;
		const double sgn = Sign0(errV);
		double step = StepFromPx(std::fabs(errV), P.find.pitch_kDegPerPx, P.find.pitch_minStepDeg, P.find.pitch_maxStepDeg);
		
		// 应用 maxPitchStepDeg 限制：防止因误差计算异常导致关节突然跳变
		if (P.find.maxPitchStepDeg > 0.0 && step > P.find.maxPitchStepDeg)
		{
			step = P.find.maxPitchStepDeg;
		}
		
		const double deltaDeg = (double)sign * sgn * step;
		if (!AddJointMove(kc, mc, arm, joint, deltaDeg, P.find.minServoPosChange, jointToPos, why))
		{
			r.ok = false;
			r.hasMove = false;
			r.why = std::wstring(L"Find(J") + std::to_wstring(joint) + L") failed: " + why;
			return r;
		}
	}

	if (jointToPos.empty())
	{
		r.ok = true;
		r.hasMove = false;
		r.why = L"Deadband suppressed.";
		return r;
	}

	r.ok = true;
	r.hasMove = true;
	r.jointToPos = std::move(jointToPos);
	r.why = L"Find step.";
	return r;
}

SeeAndFetchJointPolicy::StepResult SeeAndFetchJointPolicy::ComputeApproachStep(const SeeAndFetchStateMachine::Params& P,
                                                                              const KinematicsConfig& kc,
                                                                              const MotionConfig& mc,
                                                                              const ArmStateEstimator::ArmState& arm)
{
	StepResult r;
	r.ok = true;
	r.moveTimeMs = std::max(30, P.timing.defaultMoveTimeMs);

	if (!arm.valid)
	{
		r.ok = true;
		r.hasMove = false;
		r.why = L"Arm pose invalid; skip.";
		return r;
	}

	std::wstring why;
	std::vector<std::pair<int, int>> jointToPos;
	jointToPos.reserve(1);

	const double stepDeg = std::max(0.0, P.approach.j2AdvanceStepDeg);
	const double deltaDeg = (double)P.approach.signJ2Advance * stepDeg;
	// Approach 步进通常较大，不需要最小位置变化过滤
	if (!AddJointMove(kc, mc, arm, 2, deltaDeg, 0, jointToPos, why))
	{
		r.ok = false;
		r.hasMove = false;
		r.why = std::wstring(L"Approach(J2) failed: ") + why;
		return r;
	}

	r.ok = true;
	r.hasMove = true;
	r.jointToPos = std::move(jointToPos);
	r.why = L"Approach step.";
	return r;
}


