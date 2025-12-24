#pragma once

#include <Windows.h>
#include <array>
#include <cstdint>
#include <vector>

#include "ArmProtocol.h"
#include "MotionConfig.h"

// High-level motion controller:
// - Joint-level API -> servo targets -> ArmProtocol::PackMove -> ArmCommsService queue
// - Simple keyframe script playback (V1: one PackMove per keyframe)
class MotionController
{
public:
	struct Keyframe
	{
		int durationMs = 800;
		// Joint positions (1..6). Use -1 to keep \"not set\" (ignored).
		std::array<int, MotionConfig::kJointCount + 1> jointPos{};
	};

	MotionController();

	MotionConfig& Config() { return m_cfg; }
	const MotionConfig& Config() const { return m_cfg; }

	void LoadConfig();
	void SaveConfig() const;
	void ResetDefaults();
	void ImportLegacyServoLimitsForAssignedJoints();

	// Direct control
	bool MoveJointAbs(int jointIndex, int pos, int timeMs);
	bool MoveJointsAbs(const std::vector<std::pair<int, int>>& jointToPos, int timeMs);
	bool MoveHome(int timeMs);

	// Readback request (optional)
	void RequestReadAllAssigned();

	// Script playback
	void StartScript(std::vector<Keyframe> frames, bool loop);
	void StopScript();
	bool IsPlaying() const { return m_playing; }
	void Tick(); // call from a UI timer

private:
	static int ClampPos(int v, int minV, int maxV);
	bool BuildServoTargetsFromJoints(const std::vector<std::pair<int, int>>& jointToPos,
	                                std::vector<ArmProtocol::ServoTarget>& out);

private:
	MotionConfig m_cfg;

	bool m_playing = false;
	bool m_loop = false;
	std::vector<Keyframe> m_frames;
	size_t m_frameIndex = 0;
	ULONGLONG m_nextDue = 0; // GetTickCount64
};


