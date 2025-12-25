#pragma once

#include <array>
#include <string>
#include <vector>

#include "KinematicsConfig.h"
#include "MotionConfig.h"

// 5DOF 机械臂运动学（J1..J5）
// - 坐标系：Base 右手系，Z向上，Y向前，X向右（与 Reference/mechanics.md 对齐）
// - J1: 绕 Base.Z（水平转台）
// - J2/J3/J4: 位于同一竖直平面内的 3R（肩/肘/腕俯仰），在数学模型中统一视为绕 +X 旋转
// - J5: 末端绕自身轴旋转（姿态/滚转用途），本版 IK 默认保持不变
//
// 说明：
// - 真实环境存在误差：本类输出“可解释的失败原因”，并允许通过 KinematicsConfig 的 zeroOffsetDeg 等参数修正。
// - 安全：最终舵机位置仍需由 MotionController 使用 MotionConfig 的 min/max 做软限位裁剪。
class ArmKinematics
{
public:
	static constexpr int kJointCount = KinematicsConfig::kJointCount; // 5

	struct PoseTarget
	{
		// Base 坐标系下的目标（mm / deg）
		double x_mm = 0.0;
		double y_mm = 0.0;
		double z_mm = 0.0;
		double pitch_deg = 0.0; // 末端俯仰（绕 +X），用于抓取姿态约束
	};

	struct JointAnglesRad
	{
		// 关节角（弧度），索引 1..5，0 不用
		std::array<double, kJointCount + 1> q{};
	};

	struct ServoPos
	{
		// 舵机位置（0..1000），索引 1..5，0 不用
		std::array<int, kJointCount + 1> pos{};
	};

	struct IkSolution
	{
		JointAnglesRad q;
		double cost = 0.0;       // 择优用：离当前姿态的距离
		bool withinLimits = true; // 是否在软限位范围内（基于 MotionConfig 估算）
	};

	struct IkResult
	{
		bool ok = false;
		std::wstring reason;
		std::vector<IkSolution> candidates;
		int chosenIndex = -1;
		JointAnglesRad chosenQ;
	};

public:
	ArmKinematics() = default;

	// FK：关节角 -> 末端位姿（目前返回 x/y/z + pitch）
	static PoseTarget ForwardKinematics(const KinematicsConfig& kc, const JointAnglesRad& q);

	// IK：末端位姿（x,y,z,pitch） -> 多解 + 择优
	// - mc 可为空：为空时不做限位判定，只做可达性/数学求解；建议传入 MotionConfig 以便更稳。
	// - qCurrent 可为空：用于“离当前姿态最近”的择优，提升 Jog 调试体验。
	static IkResult InverseKinematics(const KinematicsConfig& kc,
	                                 const MotionConfig* pMc,
	                                 const PoseTarget& target,
	                                 const JointAnglesRad* pQCurrent);

	// 角度 <-> 舵机位置（结合 KinematicsConfig 两点标定 + 轴向符号 + MotionConfig::invert）
	static bool JointRadToServoPos(const KinematicsConfig& kc,
	                               const MotionConfig* pMc,
	                               int nJoint,
	                               double qRad,
	                               int& outPos);
	static bool ServoPosToJointRad(const KinematicsConfig& kc,
	                               const MotionConfig* pMc,
	                               int nJoint,
	                               int pos,
	                               double& outQRad);

	// 将一组关节角转换为舵机位置（只处理 J1..J5；J5 若未配置 ServoId 则保持 -1）
	static bool JointAnglesToServoPos(const KinematicsConfig& kc,
	                                  const MotionConfig* pMc,
	                                  const JointAnglesRad& q,
	                                  ServoPos& outPos,
	                                  std::wstring& outWhy);

private:
	static double DegToRad(double d);
	static double RadToDeg(double r);
	static bool GetJointLimitRad(const KinematicsConfig& kc,
	                             const MotionConfig* pMc,
	                             int nJoint,
	                             double& outMinRad,
	                             double& outMaxRad);
	static double WrapToPi(double a);
};




