#pragma once

#include <Windows.h>

#include <array>
#include <string>
#include <vector>

// ============================
// VisionHandLandmarks
// ============================
// 基于 MediaPipe Hands (OpenCV Zoo) 的手部关键点推理：
// - palm_detection_mediapipe_2023feb.onnx  (检测手掌框 + 7个手掌关键点)
// - handpose_estimation_mediapipe_2023feb.onnx (输出 21 个关键点)
//
// 实现细节参考：
// - https://github.com/opencv/opencv_zoo/tree/main/models/palm_detection_mediapipe
// - https://github.com/opencv/opencv_zoo/tree/main/models/handpose_estimation_mediapipe
//
// 注意：为兼容中文路径，本模块也使用“Win32 读入内存 → readNetFromONNX(buffer)”的方式加载模型。
class VisionHandLandmarks
{
public:
	struct Params
	{
		std::wstring palmOnnxPath;     // palm_detection_mediapipe_2023feb.onnx
		std::wstring handposeOnnxPath; // handpose_estimation_mediapipe_2023feb.onnx

		float palmScoreThreshold = 0.6f;
		float palmNmsThreshold = 0.3f;
		int palmTopK = 5000;

		float handposeConfThreshold = 0.8f;
		// Pinch 判定阈值（归一化）：pinchDist < pinchThreshNorm * handScale => Pinch
		float pinchThreshNorm = 0.25f;
	};

	struct Palm
	{
		// [x1,y1,x2,y2] in original image coords
		float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		// 7 landmarks (x,y) in original image coords
		std::array<float, 14> lm{};
		float score = 0.0f;
	};

	struct Hand
	{
		bool valid = false;
		float confidence = 0.0f;
		// 21 keypoints (x,y) in original image coords
		std::array<float, 42> pts{};
		// Optional bbox from landmarks (x1,y1,x2,y2)
		float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	};

public:
	void SetParams(const Params& p);
	Params GetParams() const { return m_params; }

	bool EnsureLoaded(std::wstring& outErr);
	bool IsLoaded() const { return m_loaded; }

	// 输入一帧 BGR 图像，输出最优的一只手（若无则返回 false）
	bool DetectBest(const void* bgrData, int width, int height, int strideBytes, Hand& outHand);

private:
	static bool ReadFileToBufferW(const std::wstring& path, std::vector<BYTE>& out);

private:
	Params m_params{};
	bool m_loaded = false;
	void* m_impl = nullptr; // pimpl (OpenCV nets + anchors)
};


