#pragma once

#include <array>
#include <string>
#include <vector>

#include "ArmStateEstimator.h"
#include "KinematicsConfig.h"
#include "MotionConfig.h"
#include "VisualServoTypes.h"

// ============================
// GrabTestController
// ============================
// 单抓测试：不依赖 SeeAndFetch；仅用 Gemini 识别 + TimeToFetch 决策抓取
class GrabTestController
{
public:
	enum class State
	{
		Idle = 0,
		Acquire,
		Track,
		Approach,
		Grasp,
		Retreat,
		ReturnHome,
		Abort,
		EStop,
	};

	struct Params
	{
		// 目标丢失判据
		int lostFramesToAbort = 10;
		int acquireStableFrames = 5;

		struct Timing
		{
			int minCommandIntervalMs = 120;
			int defaultMoveTimeMs = 220;
			int lockAfterMoveMs = 240;
		};
		Timing timing{};

		struct Find
		{
			int deadbandPx = 10;
			// 粗略对中阈值（像素）：允许较大误差也算“可继续”
			int coarseCenterPx = 60;
			int stableCenterFrames = 3;
			double yaw_kDegPerPx = 0.03;
			double yaw_minStepDeg = 1.5;
			double yaw_maxStepDeg = 3.5;
			double pitch_kDegPerPx = 0.03;
			double pitch_minStepDeg = 1.5;
			double pitch_maxStepDeg = 3.5;
			double maxPitchStepDeg = 8.0;
			int signJ1FromErrU = -1;
			int signJ4FromErrV = -1;
			// J3 垂直校正（用于 J2 下探后的再次居中）
			double j3_kDegPerPx = 0.03;
			double j3_minStepDeg = 1.5;
			double j3_maxStepDeg = 3.5;
			int signJ3FromErrV = -1;
			int centerOffsetU = 0;
			int centerOffsetV = 0;
			int minServoPosChange = 8;
		};
		Find find{};

		struct Approach
		{
			int timeToFetchStableFrames = 2;
			int maxAdvanceSteps = 60;
			double j2AdvanceStepDeg = 2.0;
			// J2 方向：1 表示“减小角度”
			int signJ2Advance = 1;
		};
		Approach approach{};

		struct Gripper
		{
			int jointIndex = 6;
			int openPos = 650;
			int closePos = 350;
			int closeStepPos = 25;
			int closeMoveTimeMs = 450;
			int maxCloseSteps = 12;
		};
		Gripper gripper{};

		struct Return
		{
			bool returnToStartPose = true;
			int returnTimeMs = 1200;
		};
		Return ret{};
	};

	struct UserCommand
	{
		bool start = false;
		bool cancel = false;
		bool eStop = false;
	};

	struct Input
	{
		ArmStateEstimator::ArmState arm{};
		const KinematicsConfig* pKc = nullptr;
		const MotionConfig* pMc = nullptr;
		VisualObservation obs{};
		bool hasObs = false;
		UINT frameW = 0;
		UINT frameH = 0;
		bool hasTimeToFetch = false;
		int timeToFetch = -1; // 0/1
	};

	struct Output
	{
		State state = State::Idle;
		bool active = false;
		bool hasMove = false;
		int moveTimeMs = 0;
		std::vector<std::pair<int, int>> jointToPos;
		bool requestGeminiReset = false;
		std::wstring reason;
	};

public:
	void Reset();
	void SetParams(const Params& p) { m_params = p; }
	Params GetParams() const { return m_params; }

	void SetStartPose(const std::array<int, MotionConfig::kJointCount + 1>& startPos, bool valid)
	{
		m_startPosePos = startPos;
		m_hasStartPosePos = valid;
	}

	State GetState() const { return m_state; }
	bool Tick(const Input& in, const UserCommand& cmd, Output& out);

private:
	bool ComputeCenterStepJ1J4(const Input& in, std::vector<std::pair<int, int>>& moves, std::wstring& why);
	bool ComputeCenterStepJ3(const Input& in, std::vector<std::pair<int, int>>& moves, std::wstring& why);
	bool ComputeAdvanceStep(const Input& in, int& outPos, std::wstring& why);
	void ToIdle();

private:
	Params m_params{};
	State m_state = State::Idle;

	int m_stableFrames = 0;
	int m_lostFrames = 0;
	bool m_hasEverTarget = false;
	int m_centerStableFrames = 0;
	int m_timeToFetchStableFrames = 0;
	int m_advanceSteps = 0;
	int m_phase = 0; // 0:中心(J1/J4) 1:J2下探 2:J3复位

	bool m_waitObsAfterMove = false;
	ULONGLONG m_lastObsTickMs = 0;

	ULONGLONG m_lockedUntilMs = 0;
	ULONGLONG m_lastCmdMs = 0;

	int m_gripSteps = 0;
	int m_gripCmdPos = 0;

	int m_retreatDone = 0;
	int m_retreatTotalSteps = 0;
	double m_retreatDeltaDeg = 0.0;

	bool m_hasStartPosePos = false;
	std::array<int, MotionConfig::kJointCount + 1> m_startPosePos{};
};

