#pragma once

#include <Windows.h>
#include <string>

#include "ArmStateEstimator.h"
#include "ToolConfig.h"
#include "ToolGeometry.h"
#include "VisualServoController.h"
#include "VisualServoTypes.h"

// ============================
// SeeAndFetchStateMachine（骨架）
// ============================
// 目标：把 “锁定/取消/急停/丢失回退/遮挡后导航” 的上层逻辑收敛到可解释的状态机。
// 本文件只提供骨架与统一接口，不直接驱动硬件；具体执行可由主界面/控制器选择接入点。
class SeeAndFetchStateMachine
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
		Place,
		Abort,
		EStop,
	};

	struct Params
	{
		// 丢失多少帧后认为目标丢失（Track/Approach 等状态会用到）
		int lostFramesToAbort = 10;

		// Acquire：需要连续稳定多少帧才进入 Track
		int acquireStableFrames = 5;

		// 是否启用“遮挡后导航”缓存（锁定时缓存 plane 坐标）
		bool enablePlaneCache = true;
	};

	struct UserCommand
	{
		bool confirm = false; // 确认/开始
		bool cancel = false;  // 取消
		bool eStop = false;   // 急停
	};

	struct Input
	{
		ArmStateEstimator::ArmState arm{};
		ToolConfig tool{};

		// 最新观测（可来自 detector/hand/aruco 等统一接口）
		VisualObservation obs{};
		bool hasObs = false;
	};

	struct Output
	{
		State state = State::Idle;
		bool active = false;

		// 建议的视觉伺服策略（由上层决定是否真正应用到 Jog）
		bool vsEnable = false;
		VisualServoMode vsMode = VisualServoMode::LookAndMove;
		double vsAdvance = 0.0; // [-1,1]

		// 目标缓存（用于 HUD/日志证据链）
		bool hasCachedPlanePoint = false;
		VisionGeometry::Point3 cachedPlanePointBase{}; // Base 坐标系（mm）

		// 可解释原因/状态摘要（用于日志与 UI）
		std::wstring reason;
	};

public:
	void Reset();
	void SetParams(const Params& p) { m_params = p; }
	Params GetParams() const { return m_params; }

	// 设置桌面平面（Base 坐标系）：n·X + d = 0
	void SetTablePlaneBase(const VisionGeometry::Plane& planeBase, bool valid)
	{
		m_tablePlaneBase = planeBase;
		m_hasTablePlaneBase = valid;
	}

	State GetState() const { return m_state; }

	// Tick：推进状态机一次（建议 20~50Hz）
	bool Tick(const Input& in, const UserCommand& cmd, Output& out);

private:
	void ToIdle();

private:
	Params m_params{};
	State m_state = State::Idle;

	// Acquire/Track bookkeeping
	int m_stableFrames = 0;
	int m_lostFrames = 0;

	// Cached plane point for occlusion-robust navigation
	bool m_hasCachedPlanePoint = false;
	VisionGeometry::Point3 m_cachedPlanePointBase{};

	// Table plane in Base
	bool m_hasTablePlaneBase = false;
	VisionGeometry::Plane m_tablePlaneBase{};
};


