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
		bool invert = false; // reserved for angle/delta conventions
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

private:
	static std::wstring SectionForJoint(int jointIndex);

private:
	JointArray m_joints{};
};


