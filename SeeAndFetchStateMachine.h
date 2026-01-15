#pragma once

#include <Windows.h>

#include <array>
#include <string>
#include <vector>

#include "ArmStateEstimator.h"
#include "MotionConfig.h"
#include "KinematicsConfig.h"
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
		// ===== 手势锁定阶段（参考 fake_motion_code.md）=====
		SelectGoal,      // 跟随手指（居中），并用 PointPick(Detector候选) 确认抓取目标
		SelectTerminal,  // 跟随手指（居中），并用 PointPick(红点候选) 确认放置终点
		GoInitialPose,   // 回到 initial_pos（开始抓取前的位置）
		Acquire,
		Track,
		Approach,
		Grasp,
		Retreat,
		GoTerminalPose,  // 抓取后移动到 terminal_pos（放置终点附近的姿态）
		Place,
		ReturnHome,
		Abort,
		EStop,
	};

	struct Params
	{
		// =========================
		// Global / bookkeeping
		// =========================
		// 丢失多少帧后认为目标丢失（Track/Approach 等状态会用到）
		int lostFramesToAbort = 10;

		// Acquire：需要连续稳定多少帧才进入 Track
		int acquireStableFrames = 5;

		// 是否启用“遮挡后导航”缓存（锁定时缓存 plane 坐标）
		bool enablePlaneCache = true;

		// 自动流程运行时，是否强制切到 ArUco 模式（便于确保 depthMm）
		// 注意：真正切换/恢复由 UI 层实现（状态机只给出建议）。
		bool preferArucoDuringAuto = true;

		// =========================
		// Timing / pacing
		// =========================
		struct Timing
		{
			// 两次下发 Move 之间的最小间隔（避免刷爆队列/让动作更可控）
			int minCommandIntervalMs = 120;

			// 默认单步动作时长（ms）
			int defaultMoveTimeMs = 220;

			// 某一步动作下发后，锁定多久不再发新步（通常 >= moveTimeMs）
			int lockAfterMoveMs = 240;
		};
		Timing timing{};

		// =========================
		// FindObject（对准/居中）
		// =========================
		struct Find
		{
			// 像素死区：误差进入死区视为已对准
			int deadbandPx = 10;

			// “进入死区”连续多少帧算稳定（用于从 Track 进入 Approach）
			int stableCenterFrames = 3;

			// 像素误差 -> 关节步长（度）
			// stepDeg = clamp(kDegPerPx * abs(errPx), minStepDeg, maxStepDeg)
			double yaw_kDegPerPx = 0.03;    // J1
			double yaw_minStepDeg = 0.6;
			double yaw_maxStepDeg = 3.5;

			double pitch_kDegPerPx = 0.03;  // J3/J4
			double pitch_minStepDeg = 0.6;
			double pitch_maxStepDeg = 3.5;

			// J4 负责竖直误差的“优先使用范围”（超出则改用 J3）
			// 用 abs(q4Deg) 判断，避免 J4 过度弯折损坏
			double j4PreferAbsDeg = 35.0;

			// 方向开关：不同装配/相机朝向可能导致正负号相反
			// - J1: errU>0（目标在右侧）时，希望 J1 增加还是减少
			int signJ1FromErrU = +1;
			// - J4/J3: errV>0（目标在下方）时，希望关节增加还是减少
			int signJ4FromErrV = +1;
			int signJ3FromErrV = +1;
		};
		Find find{};

		// =========================
		// ApproachObject（接近）
		// =========================
		struct Approach
		{
			// 判距模式：
			// - ArucoDepth：优先使用 depthMm
			// - BboxArea：使用跟踪框面积（px^2）/面积增量（倍数）判断进入可抓取范围
			// - Auto：有 depthMm 则用 depthMm；否则用 bbox
			enum class RangeMode
			{
				ArucoDepth = 0,
				BboxArea = 1,
				Auto = 2,
			};
			RangeMode rangeMode = RangeMode::ArucoDepth;

			// 判据：使用 ArUco 的 depthMm
			// depthMm <= graspDepthMm 认为已进入可抓取范围
			int graspDepthMm = 160;

			// 深度稳定：连续多少帧满足才允许推进（减少抖动导致的误推进）
			int depthStableFrames = 3;
			int depthMaxJumpMm = 40; // 相邻两帧 depth 跳变超过该阈值认为不稳定

			// 判据：使用 bbox 面积（px^2）或面积倍数
			// effectiveThresh = max(graspBoxAreaPx2, baseAreaPx2 * graspBoxScale)
			int graspBoxAreaPx2 = 30000;
			int graspBoxScale_milli = 0; // 0=禁用倍数门限；例如 2000=2.0x
			int boxStableFrames = 3;
			int boxAreaMaxJumpPx2 = 20000;

			// bbox 模式是否要求来自 Detector（避免 hand/aruco 框误用）
			bool bboxRequireDetector = true;

			// 最大推进步数（防止无限循环）
			int maxAdvanceSteps = 60;

			// 多次尝试：一次 Approach 达不到阈值则回退并重来
			int maxAttempts = 3;
			int retryRetreatSteps = 8; // 失败回退多少步（沿 J2 反向）

			// 推进一步：J2 变化量（度）
			double j2AdvanceStepDeg = 2.0;

			// 方向：推进(靠近目标)时 J2 该增加还是减少
			int signJ2Advance = +1;

			// 允许在推进过程中对 J1 做微调（减少左右漂移）
			bool enableJ1FineTune = true;
		};
		Approach approach{};

		// =========================
		// Grasp / Place（夹爪）
		// =========================
		struct Gripper
		{
			// 夹爪关节索引（工程中为 J6）
			int jointIndex = 6;

			// 开爪/关爪目标位置（舵机 pos: 0..1000）
			int openPos = 650;
			int closePos = 350;

			// 步进闭合：每次向 closePos 迈多少 pos
			int closeStepPos = 25;
			int closeMoveTimeMs = 450;
			int maxCloseSteps = 12;

			// “夹住/阻塞”判据（可选）：若读回位置与目标偏差较大，认为夹到物体
			int stallDetectDeltaPos = 10;
			int stallDetectMaxAgeMs = 800;
			bool enableStallDetect = false;

			// 抓取失败重试：若启用 stallDetect 且未检测到夹住，则推进若干步再重试夹取
			int maxAttempts = 2;
			int advanceStepsOnFail = 1;
		};
		Gripper gripper{};

		// =========================
		// Place / Return
		// =========================
		struct Place
		{
			enum class Mode
			{
				SimpleOpen = 0,  // 直接开爪（保持旧行为）
				RedDotVisual = 1 // 用“桌面红点”作为终点：居中->下降->开爪
			};
			Mode mode = Mode::SimpleOpen;

			// 红点识别模式：默认 ColorTrack(3)
			int visionMode = 3; // (int)VisionService::Mode::ColorTrack

			// 居中稳定帧数（与 FindObject 的 deadbandPx 配合）
			int centerStableFrames = 3;

			// 判距模式（放置阶段）：默认用 bbox 面积
			enum class RangeMode
			{
				ArucoDepth = 0,
				BboxArea = 1,
				Auto = 2,
			};
			RangeMode rangeMode = RangeMode::BboxArea;

			// ArUco depth 判据：depthMm <= placeDepthMm 认为到达放置高度
			int placeDepthMm = 180;

			// bbox 判据：area(px^2) >= max(placeBoxAreaPx2, baseArea*placeBoxScale)
			int placeBoxAreaPx2 = 24000;
			int placeBoxScale_milli = 0;
			int boxStableFrames = 3;
			int boxAreaMaxJumpPx2 = 20000;

			// 下降：J2 步进（度）
			int maxDownSteps = 30;
			double j2DownStepDeg = 2.0;
			int signJ2Down = +1;

			// 失败重试：若到达 maxDownSteps 仍未满足判距，则回退并重来
			int maxAttempts = 2;
			int retryRetreatSteps = 8;

			// 放置后抬起/回退：J2 反向走多少步（用于“抬离桌面”）
			int retreatSteps = 6;
		};
		Place place{};

		struct Return
		{
			// 是否回到“开始自动流程时”的关节位置快照
			bool returnToStartPose = true;
			int returnTimeMs = 1200;
		};
		Return ret{};
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

		// 运动学/标定配置（non-owning；由调用方保证生命周期）
		const KinematicsConfig* pKc = nullptr;
		const MotionConfig* pMc = nullptr;

		// 最新观测（可来自 detector/hand/aruco 等统一接口）
		VisualObservation obs{};
		bool hasObs = false;

		// 可选：目标跟踪框（px），用于 bbox 估距
		bool hasBox = false;
		int boxW = 0;
		int boxH = 0;
		int boxClassId = -1;
		int visionMode = 0; // (int)VisionService::Mode（避免 include VisionService.h）

		// 视觉帧尺寸（用于计算 (cx,cy) 与像素误差）
		UINT frameW = 0;
		UINT frameH = 0;

		// 可选：夹爪（servoId=6）读回位置，用于 stall detect
		bool hasGripReadback = false;
		int gripReadbackPos = 0;
		DWORD gripReadbackAgeMs = 0;

		// 当前舵机位置快照（pos:0..1000；joint 1..6）
		// 用于记录 initial_pos / terminal_pos。
		bool hasServoPos = false;
		std::array<int, MotionConfig::kJointCount + 1> servoPos{};

		// PointPick（由 VisionService 产生）：手势选物/选红点
		// state: 0=None,1=Searching,2=Locked,3=Confirmed,4=Cancelled
		int pickState = 0;
		bool hasPickBox = false;
		int pickBoxX = 0;
		int pickBoxY = 0;
		int pickBoxW = 0;
		int pickBoxH = 0;
	};

	struct Output
	{
		State state = State::Idle;
		bool active = false;

		// 建议的视觉伺服策略（由上层决定是否真正应用到 Jog）
		bool vsEnable = false;
		VisualServoMode vsMode = VisualServoMode::LookAndMove;
		double vsAdvance = 0.0; // [-1,1]

		// ===== 关节分步动作建议（本次实现主路径）=====
		bool hasMove = false;
		int moveTimeMs = 0;
		std::vector<std::pair<int, int>> jointToPos; // (jointIndex, servoPos)

		// 建议：自动流程期间应锁定手动输入
		bool lockManualJog = false;

		// 建议：自动流程期间应强制 Vision 切 ArUco（保证 depthMm）
		bool requestVisionAruco = false;

		// 更通用：请求 VisionService::Mode（-1=不请求）
		int requestVisionMode = -1;

		// PointPick 目标类型（由上层将其写入 VisionService::Params.pointPickTarget）
		// -1=不请求；0=Detector候选；1=红点候选
		int requestPointPickTarget = -1;
		// 请求重置 PointPick FSM（上层可通过 pointPickResetSeq++ 实现）
		bool requestPointPickReset = false;

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

	// 设置“开始自动流程时”的关节位置快照（用于 ReturnHome）
	void SetStartPose(const std::array<int, MotionConfig::kJointCount + 1>& startPos, bool valid)
	{
		m_startPosePos = startPos;
		m_hasStartPosePos = valid;
	}

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

	// Start pose snapshot (joint servo pos 1..6)
	bool m_hasStartPosePos = false;
	std::array<int, MotionConfig::kJointCount + 1> m_startPosePos{};

	// fake_motion_code.md: initial_pos / terminal_pos（由手势确认时记录）
	bool m_hasInitialPosePos = false;
	std::array<int, MotionConfig::kJointCount + 1> m_initialPosePos{};
	bool m_hasTerminalPosePos = false;
	std::array<int, MotionConfig::kJointCount + 1> m_terminalPosePos{};

	// GoInitialPose / GoTerminalPose pacing
	int m_goPosePhase = 0; // 0=not sent, 1=sent(wait)

	// pacing
	ULONGLONG m_lockedUntilMs = 0;
	ULONGLONG m_lastCmdMs = 0;
	int m_centerStableFrames = 0;
	int m_depthStableFrames = 0;
	bool m_hasLastDepthMm = false;
	double m_lastDepthMm = 0.0;
	int m_boxStableFrames = 0;
	bool m_hasBaseBoxArea = false;
	int m_baseBoxAreaPx2 = 0;
	int m_lastBoxAreaPx2 = 0;
	int m_approachAttempt = 0;
	int m_forceAdvanceStepsRemaining = 0;
	int m_advanceSteps = 0;
	int m_gripSteps = 0;
	int m_gripCmdPos = 0;
	int m_retreatDone = 0;
	int m_graspAttempt = 0;
	State m_retreatNextState = State::Idle;
	int m_retreatTotalSteps = 0;
	double m_retreatDeltaDeg = 0.0;

	// place (RedDotVisual)
	int m_placePhase = 0;      // 0=center, 1=down, 2=release
	int m_placeDownSteps = 0;
	int m_placeAttempt = 0;
};









