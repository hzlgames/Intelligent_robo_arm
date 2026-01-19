#pragma once

#include <Windows.h>
#include <cstdint>
#include <vector>

#include "ArmProtocol.h"
#include "MotionConfig.h"

// High-level motion controller:
// - Joint-level API -> servo targets -> ArmProtocol::PackMove -> ArmCommsService queue
// - Direct servo or joint command forwarding
class MotionController
{
public:
	MotionController();

	MotionConfig& Config() { return m_cfg; }
	const MotionConfig& Config() const { return m_cfg; }

	void LoadConfig();
	void SaveConfig() const;
	void ResetDefaults();
	void ImportLegacyServoLimitsForAssignedJoints();

	// Direct control
	bool MoveServoAbs(int servoId, int pos, int timeMs);
	bool MoveServosAbs(const std::vector<std::pair<int, int>>& servoToPos, int timeMs);
	bool MoveJointAbs(int jointIndex, int pos, int timeMs);
	bool MoveJointsAbs(const std::vector<std::pair<int, int>>& jointToPos, int timeMs);
	bool MoveHome(int timeMs);

	// Readback request (optional)
	void RequestReadAllAssigned();

private:
	static int ClampPos(int v, int minV, int maxV);
	bool BuildServoTargetsFromJoints(const std::vector<std::pair<int, int>>& jointToPos,
	                                std::vector<ArmProtocol::ServoTarget>& out);

private:
	MotionConfig m_cfg;

	// (script playback removed)
};


