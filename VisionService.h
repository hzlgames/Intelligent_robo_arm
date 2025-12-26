#pragma once

#include <Windows.h>

#include <atomic>
#include <array>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class CPreview;
class VisualServoController;

#include "VisionDetector.h"
#include "VisionHandLandmarks.h"

// ============================
// VisionService
// ============================
// 独立视觉线程：从 CPreview 拉取最新 RGB32 帧（可丢帧，只保留最新），
// 运行轻量算法并输出 VisualObservation（通过 VisualServoController::UpdateObservation）。
//
// 设计目标：
// - 不阻塞 MF 预览线程（OnReadSample）
// - 低延迟：处理跟不上就跳过旧帧
// - 后续可替换为 ArUco/色块/检测/手势等模块，但输出接口不变
class VisionService
{
public:
	enum class Mode
	{
		Auto = 0,           // 有 ArUco 就用 ArUco；否则回退到 BrightestPoint
		BrightestPoint = 1, // 最亮点（最小依赖、用于验证管线）
		Aruco = 2,          // 强制 ArUco（未检测到则无输出）
		ColorTrack = 3,     // HSV 色块/形状追踪
		Detector = 4,       // 目标检测（DNN/ONNX 等）
		HandSticker = 5,    // 手势/指向（双色贴纸）
		HandLandmarks = 6,  // 手势关键点（Palm+Handpose ONNX）
	};

	struct Params
	{
		int processPeriodMs = 33; // 约 30Hz；处理慢时会自动“丢帧”
		int sampleStride = 8;    // 取样步长：越大越省 CPU，但定位越粗
		double emaAlpha = 0.35;  // 目标点 EMA 平滑（0..1）

		// ArUco：marker 边长（mm），用于 estimatePoseSingleMarkers 的尺度
		double arucoMarkerLengthMm = 40.0;

		// 深度分档阈值（mm）：depth < Near => Near；depth > Far => Far；否则 Mid
		int depthNearMm = 120;
		int depthFarMm = 220;

		// ===== 排除手部（为“指哪抓哪”做准备）=====
		// 启用后：在 Detector/Auto/ColorTrack/最亮点 等“找目标物”的模式里，
		// 先用 HandLandmarks 得到手的 bbox，再过滤掉与手框重叠的候选目标。
		bool excludeHand = true;
		int excludeHandInflatePx = 20;      // 手框扩张像素（更稳地排除手/手臂）
		double excludeHandMaxOverlap = 0.3; // overlap = intersectArea / boxArea；超过则认为“是手/被手遮挡”

		// ===== 指向选物（仅颜色反馈，不联动）=====
		// 逻辑：Point(只伸食指) -> 在指向附近找物体 -> 指向同一物体连续 hold>=3s 锁定 -> Pinch hold>=3s 确认 -> OpenPalm hold>=3s 取消
		bool pointPickEnabled = true;
		int pointPickMaxRayLenPx = 320;     // 指向“前方”最大距离（像素）
		int pointPickMaxRayPerpPx = 90;     // 到指向射线的最大垂距（像素）
		int pointPickMaxRadiusPx = 140;     // fallback：若射线匹配不到，允许在指尖附近的搜索半径
		int pointPickHoldLockMs = 3000;     // Point 持续指向同一物体 >= 3s -> 锁定
		int pointPickHoldConfirmMs = 3000;  // Pinch >= 3s -> 确认夹取
		int pointPickHoldCancelMs = 3000;   // OpenPalm >= 3s -> 取消
		int pointPickCancelFlashMs = 800;   // 取消后蓝框闪烁时间
		double pointPickIouSame = 0.5;      // 判定“同一物体”的 IoU 阈值（0..1）
	};

	struct Stats
	{
		bool running = false;
		bool hasFrame = false;
		UINT frameW = 0;
		UINT frameH = 0;
		double procFps = 0.0;
		ULONGLONG lastProcTickMs = 0;
	};

	// 供 HUD/调试读取的最近一次识别结果（线程安全、轻量、可丢帧）
	struct Result
	{
		ULONGLONG tickMs = 0;
		int mode = 0; // (int)VisionService::Mode

		bool hasTargetPx = false;
		double u = 0.0;
		double v = 0.0;
		bool hasConfidence = false;
		double confidence = 0.0;

		bool hasDepthMm = false;
		double depthMm = 0.0;

		// Detector bbox（若本帧来自 Detector）
		bool hasBox = false;
		int boxX = 0;
		int boxY = 0;
		int boxW = 0;
		int boxH = 0;
		int classId = -1;
	};

public:
	VisionService();
	~VisionService();

	void SetParams(const Params& p);
	Params GetParams() const;

	void SetMode(Mode m) { m_mode.store((int)m); }
	Mode GetMode() const { return (Mode)m_mode.load(); }

	// Detector 参数（可在运行中更新；内部加锁保证线程安全）
	void SetDetectorParams(const VisionDetector::Params& p);
	VisionDetector::Params GetDetectorParams() const;

	// HandLandmarks 参数（可在运行中更新；内部加锁保证线程安全）
	void SetHandParams(const VisionHandLandmarks::Params& p);
	VisionHandLandmarks::Params GetHandParams() const;

	// 绑定预览源（不持有指针；调用方需保证生命周期：Stop 后再销毁 Preview）
	void SetPreview(CPreview* preview);

	// 绑定输出目标（不持有指针；线程会调用 UpdateObservation）
	void SetVisualServo(VisualServoController* vs);

	// 启停线程
	void Start();
	void Stop();
	bool IsRunning() const { return m_running.load(); }

	// 仅控制“是否产出观测”（线程仍可保持运行）
	void SetEnabled(bool on);
	bool IsEnabled() const { return m_enabled.load(); }

	Stats GetStats() const;
	Result GetLastResult() const;

private:
	void ThreadMain();

private:
	mutable std::mutex m_mu;
	Params m_params{};

	CPreview* m_preview = nullptr;
	VisualServoController* m_vs = nullptr;

	std::atomic<bool> m_running{ false };
	std::atomic<bool> m_enabled{ true };
	std::atomic<int>  m_mode{ (int)Mode::Auto };
	std::thread m_th;

	// EMA 状态（线程内使用）
	double m_lastU = 0.0;
	double m_lastV = 0.0;
	bool m_hasLastUv = false;

	// Stats（加锁保护）
	mutable std::mutex m_statsMu;
	Stats m_stats{};

	// Last vision result（加锁保护，供 HUD 读取）
	mutable std::mutex m_resMu;
	Result m_lastResult{};

	// Detector
	mutable std::mutex m_detMu;
	VisionDetector m_detector;
	ULONGLONG m_lastDetLoadAttemptMs = 0;

	// HandLandmarks
	mutable std::mutex m_handMu;
	VisionHandLandmarks m_hand;
	ULONGLONG m_lastHandLoadAttemptMs = 0;
};


