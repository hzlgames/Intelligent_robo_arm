#include "pch.h"

#include "ArmKinematics.h"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <limits>

namespace
{
	constexpr double kPi = 3.14159265358979323846;
	constexpr double kEps = 1e-9;

	// 安全 acos：避免浮点误差导致 |x| > 1 时 nan
	double SafeAcos(double x)
	{
		if (x < -1.0) x = -1.0;
		if (x > 1.0) x = 1.0;
		return std::acos(x);
	}

	double Sq(double x) { return x * x; }
}

double ArmKinematics::DegToRad(double d)
{
	return d * (kPi / 180.0);
}

double ArmKinematics::RadToDeg(double r)
{
	return r * (180.0 / kPi);
}

double ArmKinematics::WrapToPi(double a)
{
	while (a > kPi) a -= 2.0 * kPi;
	while (a < -kPi) a += 2.0 * kPi;
	return a;
}

bool ArmKinematics::ServoPosToJointRad(const KinematicsConfig& kc,
                                       const MotionConfig* /*pMc*/,
                                       int nJoint,
                                       int pos,
                                       double& outQRad)
{
	// 统一映射逻辑：方向由标定数据的 k = (posAtPlusDeg - posAt0Deg) / plusDeg 决定。
	// k > 0: 角度增大时 pos 增大（正向）
	// k < 0: 角度增大时 pos 减小（反向）
	// 这样不需要额外的 invert/sign 开关。
	if (nJoint < 1 || nJoint > kJointCount) return false;
	const auto& c = kc.GetJoint(nJoint);
	if (c.plusDeg == 0) return false;
	const double posPerDeg = (static_cast<double>(c.posAtPlusDeg - c.posAt0Deg) / static_cast<double>(c.plusDeg));
	if (std::fabs(posPerDeg) < kEps) return false;

	// 由舵机位置反解角度（度）：angleDeg = (pos - posAt0Deg) / posPerDeg
	const double angleDeg = (static_cast<double>(pos - c.posAt0Deg) / posPerDeg);
	// 应用零位偏置
	const double degZeroAdjusted = angleDeg - c.zeroOffsetDeg;

	outQRad = DegToRad(degZeroAdjusted);
	return true;
}

bool ArmKinematics::JointRadToServoPos(const KinematicsConfig& kc,
                                       const MotionConfig* /*pMc*/,
                                       int nJoint,
                                       double qRad,
                                       int& outPos)
{
	// 统一映射逻辑：方向由标定数据的 k = (posAtPlusDeg - posAt0Deg) / plusDeg 决定。
	// pos = posAt0Deg + (angleDeg + zeroOffsetDeg) * posPerDeg
	if (nJoint < 1 || nJoint > kJointCount) return false;
	const auto& c = kc.GetJoint(nJoint);
	if (c.plusDeg == 0) return false;
	const double posPerDeg = (static_cast<double>(c.posAtPlusDeg - c.posAt0Deg) / static_cast<double>(c.plusDeg));
	if (std::fabs(posPerDeg) < kEps) return false;

	// 关节角（弧度）-> 角度
	const double angleDeg = RadToDeg(qRad);
	// 应用零位偏置（加回 zeroOffsetDeg）
	const double degWithOffset = angleDeg + c.zeroOffsetDeg;

	const double posF = static_cast<double>(c.posAt0Deg) + degWithOffset * posPerDeg;
	int pos = static_cast<int>(std::lround(posF));

	// 位置域做一个基础保护（最终仍由 MotionController 做 min/max 裁剪）
	if (pos < 0) pos = 0;
	if (pos > 1000) pos = 1000;
	outPos = pos;
	return true;
}

bool ArmKinematics::GetJointLimitRad(const KinematicsConfig& kc,
                                    const MotionConfig* pMc,
                                    int nJoint,
                                    double& outMinRad,
                                    double& outMaxRad)
{
	// 限位来自 MotionConfig 的 pos(min/max)，我们把它转换成 rad 范围。
	// 注意：pos->rad 可能是单调递减（posPerDeg 为负），因此要做 min/max 交换。
	if (!pMc) return false;
	if (nJoint < 1 || nJoint > kJointCount) return false;

	const auto& jc = pMc->Get(nJoint);
	if (jc.minPos == 0 && jc.maxPos == 0) return false;

	double rMin = 0.0, rMax = 0.0;
	if (!ServoPosToJointRad(kc, pMc, nJoint, jc.minPos, rMin)) return false;
	if (!ServoPosToJointRad(kc, pMc, nJoint, jc.maxPos, rMax)) return false;
	if (rMin > rMax) std::swap(rMin, rMax);
	outMinRad = rMin;
	outMaxRad = rMax;
	return true;
}

ArmKinematics::PoseTarget ArmKinematics::ForwardKinematics(const KinematicsConfig& kc,
                                                          const JointAnglesRad& q)
{
	const auto& L = kc.Links();

	// q2/q3/q4 为平面俯仰（统一绕 +X），pitch 为三者和
	const double q1 = q.q[1];
	const double q2 = q.q[2];
	const double q3 = q.q[3];
	const double q4 = q.q[4];

	const double a2 = q2;
	const double a23 = q2 + q3;
	const double a234 = q2 + q3 + q4;

	// 平面（Y-Z）中的前向距离与高度（以肩关节为原点）
	const double y_sh = 0.0;
	const double z_sh = L.L_base;

	const double y_el = y_sh + L.L_arm1 * std::cos(a2);
	const double z_el = z_sh + L.L_arm1 * std::sin(a2);

	const double y_wr = y_el + L.L_arm2 * std::cos(a23);
	const double z_wr = z_el + L.L_arm2 * std::sin(a23);

	const double y_ee = y_wr + L.L_wrist * std::cos(a234);
	const double z_ee = z_wr + L.L_wrist * std::sin(a234);

	// 绕 Z 把平面半径 y_ee 旋到 3D：Y为前，X为右
	const double r = y_ee;
	const double x = r * std::sin(q1);
	const double y = r * std::cos(q1);

	PoseTarget out;
	out.x_mm = x;
	out.y_mm = y;
	out.z_mm = z_ee;
	out.pitch_deg = RadToDeg(a234);
	return out;
}

bool ArmKinematics::JointAnglesToServoPos(const KinematicsConfig& kc,
                                         const MotionConfig* pMc,
                                         const JointAnglesRad& q,
                                         ServoPos& outPos,
                                         std::wstring& outWhy)
{
	outWhy.clear();
	for (int j = 0; j <= kJointCount; j++) outPos.pos[j] = -1;
	for (int j = 1; j <= kJointCount; j++)
	{
		// 仅当该关节在 MotionConfig 中已绑定 ServoId 时才输出
		if (pMc)
		{
			const int sid = pMc->Get(j).servoId;
			if (sid < 1 || sid > 6)
			{
				// Jog/IK 场景：未绑定时直接失败，提示用户先完成映射
				outWhy = L"关节未绑定舵机ID，请先在运动标定中设置 ServoId。";
				return false;
			}
		}

		int pos = 0;
		if (!JointRadToServoPos(kc, pMc, j, q.q[j], pos))
		{
			outWhy = L"关节标定参数无效（两点标定不足或除0），请检查 KinematicsConfig。";
			return false;
		}
		outPos.pos[j] = pos;
	}
	return true;
}

ArmKinematics::IkResult ArmKinematics::InverseKinematics(const KinematicsConfig& kc,
                                                         const MotionConfig* pMc,
                                                         const PoseTarget& target,
                                                         const JointAnglesRad* pQCurrent)
{
	IkResult r;
	r.ok = false;
	r.reason.clear();

	const auto& L = kc.Links();

	// 1) base yaw：注意 Y 为前，因此 atan2(x, y)
	const double q1 = std::atan2(target.x_mm, target.y_mm);

	// 2) 将目标投影到平面（r,z'）
	const double r_xy = std::sqrt(Sq(target.x_mm) + Sq(target.y_mm));
	const double zp = target.z_mm - L.L_base;
	const double pitch = DegToRad(target.pitch_deg);

	// 3) 计算“腕中心”（即 J4->J5 的起点），用于 2 连杆 IK
	const double r_wc = r_xy - L.L_wrist * std::cos(pitch);
	const double z_wc = zp - L.L_wrist * std::sin(pitch);

	const double L1 = L.L_arm1;
	const double L2 = L.L_arm2;
	const double d2 = Sq(r_wc) + Sq(z_wc);
	const double d = std::sqrt(d2);

	// 可达性：距离必须在 [|L1-L2|, L1+L2]
	const double dMin = std::fabs(L1 - L2);
	const double dMax = (L1 + L2);
	if (d > dMax + 1e-6 || d < dMin - 1e-6)
	{
		r.ok = false;
		r.reason = L"目标超出可达范围（两连杆无法到达腕中心）。";
		return r;
	}

	// 4) 求解肘角 q3（两解）
	const double cos_q3 = (d2 - Sq(L1) - Sq(L2)) / (2.0 * L1 * L2);
	const double q3a = SafeAcos(cos_q3);   // 肘下/肘上取决于坐标系，这里作为候选
	const double q3b = -q3a;

	auto evalCandidate = [&](double q2, double q3, double q4) -> IkSolution
	{
		IkSolution s;
		for (int i = 0; i <= kJointCount; i++) s.q.q[i] = 0.0;
		s.q.q[1] = WrapToPi(q1);
		s.q.q[2] = WrapToPi(q2);
		s.q.q[3] = WrapToPi(q3);
		s.q.q[4] = WrapToPi(q4);
		s.q.q[5] = 0.0; // 默认保持（上层可覆盖）

		// 代价：离当前姿态最近（Jog 最重要）
		if (pQCurrent)
		{
			double sum = 0.0;
			for (int j = 1; j <= 4; j++)
			{
				const double dq = WrapToPi(s.q.q[j] - pQCurrent->q[j]);
				sum += dq * dq;
			}
			s.cost = sum;
		}

		// 限位：用 MotionConfig 的 pos(min/max) 反解得到角度范围做粗判定
		s.withinLimits = true;
		if (pMc)
		{
			for (int j = 1; j <= 4; j++)
			{
				double mn = 0.0, mx = 0.0;
				if (GetJointLimitRad(kc, pMc, j, mn, mx))
				{
					if (s.q.q[j] < mn - 1e-6 || s.q.q[j] > mx + 1e-6)
					{
						s.withinLimits = false;
						break;
					}
				}
			}
		}
		return s;
	};

	auto solveQ2Q4 = [&](double q3) -> IkSolution
	{
		// q2 = atan2(z, r) - atan2(L2*sin(q3), L1 + L2*cos(q3))
		const double phi = std::atan2(z_wc, r_wc);
		const double psi = std::atan2(L2 * std::sin(q3), L1 + L2 * std::cos(q3));
		const double q2 = phi - psi;
		const double q4 = pitch - (q2 + q3);
		return evalCandidate(q2, q3, q4);
	};

	r.candidates.clear();
	// 5) 稳定择优策略（避免在两个解之间跳变）：
	// - 首先生成两个候选解
	// - 优先选择 withinLimits 的解
	// - 在同等条件下，选择离当前姿态最近（cost 最小）的解
	// - 如果 cost 相近（差异 < 阈值），则使用 Elbow-Up 作为决胜条件（保持构型稳定）
	IkSolution candA = solveQ2Q4(q3a);
	IkSolution candB = solveQ2Q4(q3b);

	const double zElA = L.L_base + L.L_arm1 * std::sin(candA.q.q[2]); // elbow after J2
	const double zElB = L.L_base + L.L_arm1 * std::sin(candB.q.q[2]);

	// [硬约束] J3 只允许"正角度"（按"物理角度"定义：RadToDeg(q) + zeroOffsetDeg >= 0）
	// 说明：J4 允许为负（用户需求），只限制 J3。
	auto allowBySign = [&](const IkSolution& s) -> bool
	{
		// 允许极小负数（数值误差）
		const double epsDeg = 1e-3;
		const double j3PhysDeg = RadToDeg(s.q.q[3]) + kc.GetJoint(3).zeroOffsetDeg;
		return (j3PhysDeg >= -epsDeg);  // 只限制 J3，J4 允许为负
	};

	// [稳定性约束] 防止"绕底座旋转"：当有当前姿态参考时，拒绝 J1 变化过大的解
	// 说明：当 J2+J3 接近伸直极限时，IK 可能通过大幅改变 J1 来"满足"目标，导致末端绕底座旋转
	auto allowByJ1Stability = [&](const IkSolution& s) -> bool
	{
		if (!pQCurrent) return true; // 无参考时不限制
		const double dJ1 = std::fabs(WrapToPi(s.q.q[1] - pQCurrent->q[1]));
		const double maxJ1ChangeDeg = 15.0; // 单步 J1 最大允许变化 15°
		return dJ1 <= DegToRad(maxJ1ChangeDeg);
	};

	r.candidates.reserve(2);
	if (allowBySign(candA) && allowByJ1Stability(candA)) r.candidates.push_back(candA);
	if (allowBySign(candB) && allowByJ1Stability(candB)) r.candidates.push_back(candB);

	if (r.candidates.empty())
	{
		r.ok = false;
		r.reason = L"IK 无可用解（J3 需为正，或 J1 变化过大被拒绝）。";
		return r;
	}

	// 择优逻辑：优先 withinLimits，次优 cost 最小，最后 Elbow-Up
	int bestIdx = 0;
	{
		// 若只剩一个候选，直接选它
		if (r.candidates.size() == 1)
		{
			bestIdx = 0;
		}
		else
		{
			// 此时 candidates 中一定是 {candA, candB} 的子集，且 size==2
			const IkSolution& A = r.candidates[0];
			const IkSolution& B = r.candidates[1];
			const bool aInLimits = A.withinLimits;
			const bool bInLimits = B.withinLimits;
			const double costA = A.cost;
			const double costB = B.cost;
			const double costThreshold = 0.01; // cost 差异阈值（弧度平方），约 5.7°

			if (aInLimits && !bInLimits)
			{
				bestIdx = 0;
			}
			else if (!aInLimits && bInLimits)
			{
				bestIdx = 1;
			}
			else
			{
				if (pQCurrent)
				{
					if (std::fabs(costA - costB) > costThreshold)
					{
						bestIdx = (costA <= costB) ? 0 : 1;
					}
					else
					{
						// cost 相近时，用 Elbow-Up 决胜（保持构型稳定）
						// 注意：Elbow-Up 判断使用原 candA/candB 的肘部高度（不依赖过滤后的顺序）
						bestIdx = (zElA >= zElB) ? 0 : 1;
					}
				}
				else
				{
					bestIdx = (zElA >= zElB) ? 0 : 1;
				}
			}
		}
	}

	const IkSolution& chosen = r.candidates[bestIdx];
	r.chosenIndex = bestIdx;
	r.chosenQ = chosen.q;
	r.ok = true;

	if (!chosen.withinLimits)
	{
		// 仍然返回 ok=true 方便上层展示，但给出原因（上层可选择拒绝发送）
		r.reason = L"存在可达解，但该解可能超出软限位范围。";
	}
	return r;
}







