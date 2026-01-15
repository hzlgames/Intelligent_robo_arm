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
                                       const MotionConfig* pMc,
                                       int nJoint,
                                       int pos,
                                       double& outQRad)
{
	if (nJoint < 1 || nJoint > kJointCount) return false;
	const auto& c = kc.GetJoint(nJoint);
	if (c.plusDeg == 0) return false;
	const double posPerDeg = (static_cast<double>(c.posAtPlusDeg - c.posAt0Deg) / static_cast<double>(c.plusDeg));
	if (std::fabs(posPerDeg) < kEps) return false;

	// 由舵机位置反解“物理角度（度）”
	double degPhysical = (static_cast<double>(pos - c.posAt0Deg) / posPerDeg);
	if (c.physicalInvert) degPhysical = -degPhysical;
	const double degZeroAdjusted = degPhysical - c.zeroOffsetDeg;

	// 轴向符号 +（可选）MotionConfig::invert 叠加修正
	int sign = KinematicsConfig::AxisSignForJoint(nJoint);
	if (pMc && pMc->Get(nJoint).invert) sign = -sign;

	outQRad = DegToRad(degZeroAdjusted) * static_cast<double>(sign);
	return true;
}

bool ArmKinematics::JointRadToServoPos(const KinematicsConfig& kc,
                                       const MotionConfig* pMc,
                                       int nJoint,
                                       double qRad,
                                       int& outPos)
{
	if (nJoint < 1 || nJoint > kJointCount) return false;
	const auto& c = kc.GetJoint(nJoint);
	if (c.plusDeg == 0) return false;
	const double posPerDeg = (static_cast<double>(c.posAtPlusDeg - c.posAt0Deg) / static_cast<double>(c.plusDeg));
	if (std::fabs(posPerDeg) < kEps) return false;

	int sign = KinematicsConfig::AxisSignForJoint(nJoint);
	if (pMc && pMc->Get(nJoint).invert) sign = -sign;
	const double degZeroAdjusted = RadToDeg(qRad) / static_cast<double>(sign);
	double degPhysical = degZeroAdjusted + c.zeroOffsetDeg;
	if (c.physicalInvert) degPhysical = -degPhysical;

	const double posF = static_cast<double>(c.posAt0Deg) + degPhysical * posPerDeg;
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
	// 5) 强制 Elbow-Up：
	// 数学上两支解通常成对存在，但工程上我们强制使用“肘部更高”的那支，以避免瞬时构型切换。
	IkSolution candA = solveQ2Q4(q3a);
	IkSolution candB = solveQ2Q4(q3b);

	const double zElA = L.L_base + L.L_arm1 * std::sin(candA.q.q[2]); // elbow after J2
	const double zElB = L.L_base + L.L_arm1 * std::sin(candB.q.q[2]);

	const bool pickA = (zElA >= zElB);
	const IkSolution elbowUp = pickA ? candA : candB;

	r.candidates.reserve(1);
	r.candidates.push_back(elbowUp);

	r.chosenIndex = 0;
	r.chosenQ = elbowUp.q;
	r.ok = true;

	if (!elbowUp.withinLimits)
	{
		// 仍然返回 ok=true 方便上层展示，但给出原因（上层可选择拒绝发送）
		r.reason = L"存在 Elbow-Up 可达解，但该解可能超出软限位范围。";
	}
	return r;
}







