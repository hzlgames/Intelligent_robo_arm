#pragma once

#include <Windows.h>

#include <string>
#include <vector>

#include "ArmStateEstimator.h"
#include "KinematicsConfig.h"
#include "MotionConfig.h"
#include "SeeAndFetchStateMachine.h"
#include "VisualServoTypes.h"

// ============================
// SeeAndFetchJointPolicy
// ============================
// 目标：实现 fake_motion_code.md 中的 FindObject/ApproachObject 的“单步决策”，
// 将 (像素误差/深度/当前姿态) -> (关节目标pos) 的转换封装起来，便于状态机复用与单元调试。
//
// 注意：
// - 本模块不直接下发运动；仅输出建议的 jointToPos + moveTimeMs。
// - 关节正负号可能因装配/相机方向不同而变化，统一通过 Params.find/approach 的 sign 开关配置。
class SeeAndFetchJointPolicy
{
public:
	// 将“某关节增量（deg）”转换为“目标舵机位置 pos”，用于 Retreat 等复用场景。
	// - deltaDeg：基于运动学模型角度（与 ArmKinematics 的 q 定义一致）
	// - outPos：转换后的舵机目标位置（0..1000）
	static bool JointDeltaDegToServoPos(const KinematicsConfig& kc,
	                                    const MotionConfig& mc,
	                                    const ArmStateEstimator::ArmState& arm,
	                                    int joint,
	                                    double deltaDeg,
	                                    int& outPos,
	                                    std::wstring& outWhy);

	struct StepResult
	{
		bool ok = true;                 // 计算是否成功（失败通常为标定不足/转换失败）
		bool hasMove = false;           // 是否需要下发一步动作
		int moveTimeMs = 0;             // 建议动作时长
		std::vector<std::pair<int, int>> jointToPos; // (jointIndex, servoPos)
		bool centeredNow = false;       // 本帧是否已居中（进入像素死区）
		double errU = 0.0;              // u-cx
		double errV = 0.0;              // v-cy
		std::wstring why;               // 可解释原因（用于日志/诊断）
	};

public:
	// FindObject：根据像素误差生成一步“居中动作”（可同时动 J1 + (J4 or J3)）
	static StepResult ComputeFindStep(const SeeAndFetchStateMachine::Params& P,
	                                 const KinematicsConfig& kc,
	                                 const MotionConfig& mc,
	                                 const ArmStateEstimator::ArmState& arm,
	                                 const VisualObservation& obs,
	                                 UINT frameW,
	                                 UINT frameH);

	// Approach：推进一步（主要动 J2）
	static StepResult ComputeApproachStep(const SeeAndFetchStateMachine::Params& P,
	                                     const KinematicsConfig& kc,
	                                     const MotionConfig& mc,
	                                     const ArmStateEstimator::ArmState& arm);
};


