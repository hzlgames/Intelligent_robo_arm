#pragma once

#include <array>
#include <mutex>
#include <string>

// VisionOverlayService：为“相机画面 HUD 叠加”提供线程安全的视觉结果快照。
// - VisionService 线程更新识别结果（检测框/深度/手势关键点等）
// - 渲染线程（DrawDevice::DrawOverlays）读取快照并绘制
class VisionOverlayService
{
public:
	struct Point2
	{
		double x = 0.0;
		double y = 0.0;
	};

	struct RectI
	{
		int x = 0;
		int y = 0;
		int w = 0;
		int h = 0;
	};

	enum class Gesture
	{
		Unknown = 0,
		OpenPalm = 1,
		Fist = 2,
		Point = 3,
		Pinch = 4,
	};

	struct Snapshot
	{
		// 更新时间戳（GetTickCount64）
		unsigned long long tickMs = 0;

		// 当前识别算法（VisionService::Mode 对应值；以 int 存储避免头文件耦合）
		int mode = 0;

		// 通用目标点
		bool hasTargetPx = false;
		double u = 0.0;
		double v = 0.0;
		bool hasConfidence = false;
		double confidence = 0.0;

		// 深度（mm）
		bool hasDepthMm = false;
		double depthMm = 0.0;
		int depthNearMm = 120;
		int depthFarMm = 220;

		// Detector: bbox
		bool hasBox = false;
		RectI box{};
		int classId = -1;

		// ArUco: corners (optional)
		bool hasArucoCorners = false;
		std::array<Point2, 4> arucoCorners{};

		// HandLandmarks: 21 points + gesture
		bool hasHandLandmarks = false;
		std::array<Point2, 21> handPts{};
		Gesture gesture = Gesture::Unknown;
		double pinchStrength = 0.0; // 0..1 (optional)

		// Debug text (kept short; may be empty)
		std::wstring note;
	};

	static VisionOverlayService& Instance();

	void Update(const Snapshot& s);
	Snapshot GetSnapshot() const;

private:
	VisionOverlayService() = default;

private:
	mutable std::mutex m_mu;
	Snapshot m_s;
};


