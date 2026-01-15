#pragma once

#include <Windows.h>
#include <array>
#include <string>

#include "ArmKinematics.h"
#include "KinematicsConfig.h"
#include "MotionController.h"

// JogController：主界面“按住持续移动”的摇杆式控制逻辑
// - 视觉伺服：沿用 Base 笛卡尔输入（XYZ + pitch）
// - 手动控制：使用 FPS 圆柱中间层（r/z/phi/baseYaw）进行积分，再统一落到 PoseTarget
//
// 设计要点：
// - 按住移动、松开即停（deadman）
// - 固定频率 Tick（例如 20Hz）对目标 Pose 积分
// - 每次 Tick：PoseTarget -> IK -> ServoPos -> 下发
// - “最新指令优先”：避免队列堆积导致的严重延迟（通过清理 Jog 队列实现）
// - 错误可解释：IK 失败/超限时，返回 reason，UI 可展示并停止继续发送
class JogController
{
public:
	// =========================
	// JogTrace：最小可观测性（用于排查“瞬移/方向错乱”）
	// =========================
	// 设计原则：
	// - 固定环形缓冲，避免频繁动态分配影响实时性
	// - 仅记录关键状态：输入、target、估计姿态来源、IK/下发结果
	// - 失败时由 UI 触发导出到文件，便于离线分析
	enum class TraceEvent
	{
		None = 0,
		SkipNotDue,         // 未到发送周期
		SkipInactive,       // deadman 未按住
		WaitReadback,       // 等待读回建立（保护：防止跳回home）
		NudgeJointSent,     // 单关节连续微调分支下发
		StepModeSent,       // 步进模式分支下发
		IkFailed,           // IK 失败（含回滚）
		LimitRejected,      // 软限位抑制（回滚）
		DeadbandSuppressed, // deadband 抑制（不下发）
		SendFailed,         // MoveJointsAbs 失败
		SendOk,             // 下发成功
	};

	struct TracePose
	{
		double x_mm = 0.0;
		double y_mm = 0.0;
		double z_mm = 0.0;
		double pitch_deg = 0.0;
	};

	struct TraceEntry
	{
		ULONGLONG tickMs = 0;       // GetTickCount64
		TraceEvent ev = TraceEvent::None;
		uint8_t inputKind = 0;      // 0=Cartesian, 1=Fps（避免引用 private enum）
		bool active = false;
		bool stepMode = false;

		// 输入（归一化）
		double fwd = 0.0, vert = 0.0, yaw = 0.0, pitch = 0.0;
		double x = 0.0, y = 0.0, z = 0.0, cartPitch = 0.0;

		// 读回使用情况（用于定位“读回陈旧/缺失导致跳变”）
		int usedReadbackCount = 0;  // 本次估计使用了多少路读回（0..5）
		std::array<int, ArmKinematics::kJointCount + 1> usedPos{}; // 1..5（本次估计用到的pos）
		std::array<uint8_t, ArmKinematics::kJointCount + 1> usedFromReadback{}; // 1=readback,0=home/fallback

		// 目标变化（本帧规划）
		TracePose prevTarget{};
		TracePose nextTarget{};

		// IK/下发结果摘要
		bool ikOk = false;
		bool withinLimits = true;
		int chosenIndex = -1;
		int maxDeltaPos = 0;
		int sendTimeMs = 0;

		// 原因（截断保存，避免动态分配）
		wchar_t why[160] = { 0 };
	};

	struct Params
	{
		// 发送频率（Hz）。实际发送仍会受 ArmCommsService Throttle 影响。
		int sendHz = 20;

		// 最大步长限制（每 tick），防止数值抖动导致跳变过大
		double maxStepMm = 1.5;       // mm/tick [降低]
		double maxStepPitchDeg = 1.0; // deg/tick [降低]
		double maxStepYawDeg = 1.5;   // deg/tick [降低]

		// 速度（由 UI 滑条给出）[全部降低到约 1/3]
		double speedMmPerSec = 18.0;    // mm/s (原 50)
		double yawDegPerSec = 30.0;     // deg/s (原 90)
		double pitchDegPerSec = 12.0;   // deg/s (原 30)

		// 离散步进模式（更适合状态机/视觉抓取：每次给一个可观步长，500~800ms 平滑到位，期间锁输入）
		bool stepMode = true;
		int stepDurationMs = 650;   // 默认步进时长（ms）
		int stepTranslateMs = 350;  // [微调] 平移（W/S/Q/E）更短的执行时长（ms），步长将按 speedMmPerSec 等比缩放以保持速度不变
		double stepMm = 12.0;       // 仅用于兜底：当 speedMmPerSec 无效时的默认步长（mm）
		double stepYawDeg = 6.0;    // 每步底座转角（deg）
		double stepPitchDeg = 4.0;  // 每步俯仰（deg）

		// 单关节连续微调（Yaw/Pitch）：每次下发的小段执行时长（ms），越小越“跟手”
		int jointNudgeMs = 120;
	};

	struct InputState
	{
		// deadman：是否按住（鼠标按下或特定按键）
		bool active = false;

		// 归一化输入 [-1,1]：
		// - x: 右为 +X
		// - y: 前为 +Y
		// - z: 上为 +Z
		// - pitch: 末端俯仰（+为抬头/向上）
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		double pitch = 0.0;
	};

	// FPS 腕关节坐标系控制输入（"FPS驾驶模式"）
	// - fwd: 腕关节水平伸展（W前/S后）→ 只影响 J2, J3
	// - vert: 腕关节垂直移动（E上/Q下）→ 只影响 J2, J3
	// - yaw: 底座转动（A左/D右）→ 只影响 J1
	// - pitch: 末端俯仰（R抬头/F低头）→ 只影响 J4
	struct FpsInputState
	{
		bool active = false;
		double fwd = 0.0;   // [-1,1]
		double vert = 0.0;  // [-1,1]
		double yaw = 0.0;   // [-1,1]
		double pitch = 0.0; // [-1,1]
	};

public:
	JogController();

	void SetParams(const Params& p) { m_params = p; }
	Params GetParams() const { return m_params; }

	// Base 笛卡尔 Jog 输入（视觉伺服/沿用旧接口）
	void SetInputState(const InputState& in);
	InputState GetInputState() const { return m_cartInput; }

	// 手动 FPS 输入（新接口）
	void SetFpsInputState(const FpsInputState& in);
	FpsInputState GetFpsInputState() const { return m_fpsInput; }

	// 设置当前目标（通常在启动 Jog 或收到外部定位结果时调用）
	void SetTargetPose(const ArmKinematics::PoseTarget& pose);
	ArmKinematics::PoseTarget GetTargetPose() const { return m_target; }

	// 绑定依赖（由主界面提供单例/成员）
	void Bind(MotionController* pMotion, KinematicsConfig* pKc);

	// 定时调用：负责积分 + 下发
	bool Tick(std::wstring& outWhy);

	// 停止 Jog（不再发送），但不做急停（急停由 UI 单独触发）
	void Stop();

	// ===== JogTrace API =====
	void EnableTrace(bool bEnable);
	bool IsTraceEnabled() const { return m_traceEnabled; }
	void ClearTrace();
	std::wstring DumpTraceText(int maxLines = 240) const; // UTF-16 文本（写盘时转 UTF-8）
	bool SaveTraceToFile(const std::wstring& path, std::wstring& outWhy) const;
	bool SaveTraceToDefaultFile(std::wstring& outPath, std::wstring& outWhy) const;

private:
	bool BuildCurrentJointEstimate(const MotionConfig& mc,
	                              const KinematicsConfig& kc,
	                              ArmKinematics::JointAnglesRad& outQ,
	                              std::array<int, ArmKinematics::kJointCount + 1>* pUsedPos,
	                              std::array<uint8_t, ArmKinematics::kJointCount + 1>* pUsedFromReadback);
	void SyncFpsStateFromTarget();
	ArmKinematics::PoseTarget StepTargetByCartesian(const InputState& in, double dt) const;
	ArmKinematics::PoseTarget StepTargetByFps(const FpsInputState& in, double dt, std::wstring& outWhy);

	void TracePush(const TraceEntry& e);
	static TracePose ToTracePose(const ArmKinematics::PoseTarget& p);

private:
	Params m_params;
	InputState m_cartInput;
	FpsInputState m_fpsInput;
	enum class InputKind { Cartesian, Fps };
	InputKind m_inputKind = InputKind::Cartesian;

	ArmKinematics::PoseTarget m_target{};
	ArmKinematics::PoseTarget m_lastGoodTarget{};
	bool m_hasLastGoodTarget = false;

	// FPS 腕关节状态（与 m_target 保持同步）
	// 注意：m_fps_r_mm / m_fps_z_mm 是腕关节位置，不是末端位置！
	bool m_fpsHasState = false;
	double m_fps_r_mm = 0.0;         // 腕关节水平距离（mm）
	double m_fps_z_mm = 0.0;         // 腕关节高度（mm）
	double m_fps_phi_deg = -45.0;    // 末端俯仰角（度）
	double m_fps_baseYaw_rad = 0.0;  // 底座旋转角（弧度）

	MotionController* m_pMotion = nullptr;
	KinematicsConfig* m_pKc = nullptr;

	ULONGLONG m_lastTick = 0;

	// 输出去抖：避免“每tick都发几乎一样的pos”导致舵机抖动/不响应
	std::array<int, ArmKinematics::kJointCount + 1> m_lastSentJointPos{}; // 1..5
	bool m_hasLastSentJointPos = false;

	// 离散步进锁（期间忽略输入；急停由 UI 全局处理）
	ULONGLONG m_stepLockedUntil = 0;

	// 最近一次姿态估计中，实际使用了多少路“回读”（0..5）
	// 用途：连接后回读尚未建立时，禁止把 homePos 当作“当前姿态”去规划（否则会出现“第一次输入就跳回竖直复位位”）
	int m_lastUsedReadbackCount = 0;

	// ===== JogTrace =====
	static constexpr size_t kTraceCap = 600; // 约 30 秒 @20Hz
	bool m_traceEnabled = true;
	std::array<TraceEntry, kTraceCap> m_trace{};
	size_t m_traceNext = 0;
	size_t m_traceCount = 0;

	// 单关节连续微调（Yaw/Pitch）内部积分状态：
	// - 目的：避免“每 tick 读回没更新 -> 反复下发同一个 pos”，导致看起来几乎不动
	// - 做法：在按键持续期间，以首次读回为起点，在本地累积目标关节角并连续下发
	bool m_nudgeActive = false;
	int m_nudgeJoint = 0;     // 当前微调的关节（1 或 4）
	double m_nudgeQRad = 0.0; // 当前微调关节的“目标角”（弧度）
};



