#pragma once

#include <string>

#include "ArmKinematics.h"
#include "KinematicsConfig.h"
#include "MotionController.h"

// ============================
// ArmStateEstimator
// ============================
// 目标：从“读回缓存（优先）/HomePos（fallback）”估计当前关节角，
//      为视觉协同中的 Cam↔Base 变换、射线几何、状态机提供一致的姿态来源。
class ArmStateEstimator
{
public:
	struct ArmState
	{
		bool valid = false;
		ArmKinematics::JointAnglesRad q{}; // 1..5
		ArmKinematics::PoseTarget joint5PoseBase{}; // FK 结果：J5轴心（模型末端）在 Base 下的位置与 pitch_deg
		double yawRad = 0.0;   // q1
		double pitchRad = 0.0; // q2+q3+q4（与 ArmKinematics::PoseTarget::pitch_deg 定义一致）
		double rollRad = 0.0;  // q5（可选；很多场景不用）
	};

	// 估计当前关节角：
	// - 若 servoId 有读回缓存，则用读回；否则使用 homePos。
	// - 若标定不足导致 pos->rad 失败，则 valid=false（但仍会填 0 作为输出）。
	static bool Estimate(const MotionController& motion,
	                     const KinematicsConfig& kc,
	                     ArmState& outState,
	                     std::wstring* outWhy = nullptr);
};


