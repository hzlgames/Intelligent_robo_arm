#pragma once

#include <Windows.h>

#include <string>
#include <vector>

// ============================
// VisionDetector
// ============================
// 目标检测后端（当前实现：优先用 OpenCV DNN；未来可替换为 ONNX Runtime 等）。
//
// 注意：为了兼容中文路径，本项目读取模型文件时优先用 Win32 API 读入内存，
// 再交给 OpenCV 从 buffer 加载（避免 OpenCV 直接用 std::string 路径读文件失败）。
class VisionDetector
{
public:
	struct Params
	{
		// ONNX 模型路径（建议放在 ASCII 路径，但这里也支持中文路径）
		std::wstring onnxPath;

		// DetectionModel 参数
		float confThreshold = 0.5f;
		float nmsThreshold = 0.4f;
		int inputW = 320;
		int inputH = 320;
		bool swapRB = true;
	};

	struct Detection
	{
		int classId = -1;
		float confidence = 0.0f;
		int x = 0;
		int y = 0;
		int w = 0;
		int h = 0;
	};

public:
	void SetParams(const Params& p);
	Params GetParams() const { return m_params; }

	// 尝试加载模型。若成功返回 true，否则 false，并给出错误原因（可空）。
	bool EnsureLoaded(std::wstring& outErr);
	bool IsLoaded() const { return m_loaded; }

	// 对一帧 BGR 图像做检测，返回是否得到有效目标（输出 best）。
	// 若未加载模型，返回 false。
	bool DetectBest(const void* bgrData, int width, int height, int strideBytes, Detection& best);

private:
	static bool ReadFileToBufferW(const std::wstring& path, std::vector<BYTE>& out);
	static std::string WStringToUtf8(const std::wstring& ws);

private:
	Params m_params{};
	bool m_loaded = false;

	// Opaque storage for OpenCV objects (kept as void* in header to avoid heavy includes)
	void* m_impl = nullptr;
};


