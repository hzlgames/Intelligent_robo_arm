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


