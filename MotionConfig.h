#pragma once

#include <array>
#include <cstdint>
#include <string>

// Persistent per-joint calibration/configuration.
// Joint index is 1..6 (0 unused).
class MotionConfig
{
public:
	struct Joint
	{
		int servoId = 0;     // 1..6, 0 means unassigned
		int minPos = 0;      // 0..1000 (soft limit)
		int maxPos = 1000;   // 0..1000 (soft limit)
		int homePos = 500;   // recommended neutral pose
		// 注：方向信息已统一由 KinematicsConfig 的两点标定数据（k值符号）决定，
		//     不再需要 invert 开关。
	};

	static constexpr int kJointCount = 6;
	using JointArray = std::array<Joint, kJointCount + 1>;

	MotionConfig();

	void ResetDefaults();
	void LoadAll();
	void SaveAll() const;

	Joint& Get(int jointIndex) { return m_joints[jointIndex]; }
	const Joint& Get(int jointIndex) const { return m_joints[jointIndex]; }

	// Import existing Serial page soft limits (ServoLimits/MinX,MaxX) into joints
	// for joints that already have a valid servoId.
	void ImportLegacyServoLimitsForAssignedJoints();

	static std::wstring JointName(int jointIndex);

	// ========== 统一限位访问接口（通过 ServoId）==========
	// 通过 ServoId 查找对应的 Joint Index（1-6），未找到返回 0
	int FindJointByServoId(int servoId) const;

	// 通过 ServoId 获取限位值（若未绑定则返回默认 0/1000）
	bool GetLimitsByServoId(int servoId, int& outMin, int& outMax) const;

	// 通过 ServoId 设置限位值并保存（若未绑定则失败）
	bool SetLimitsByServoId(int servoId, int minPos, int maxPos);

	// 通过 ServoId 设置单个限位（min 或 max）并保存
	bool SetMinByServoId(int servoId, int minPos);
	bool SetMaxByServoId(int servoId, int maxPos);

	// 通过 ServoId 获取单个限位
	int GetMinByServoId(int servoId) const;
	int GetMaxByServoId(int servoId) const;

	// 裁剪位置到限位范围（通过 ServoId）
	int ClampByServoId(int servoId, int pos, bool* outClamped = nullptr) const;

private:
	static std::wstring SectionForJoint(int jointIndex);

private:
	JointArray m_joints{};
};


