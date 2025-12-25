#include "pch.h"

#include "VisionDetector.h"

#include <algorithm>

#if defined(__has_include)
#if __has_include(<opencv2/core.hpp>) && __has_include(<opencv2/imgproc.hpp>) && __has_include(<opencv2/dnn.hpp>)
#define SMARTARM_HAS_OPENCV_DNN 1
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#endif
#endif

namespace
{
	struct Impl
	{
#if defined(SMARTARM_HAS_OPENCV_DNN) && SMARTARM_HAS_OPENCV_DNN
		cv::dnn::Net net;
		cv::dnn::DetectionModel model = cv::dnn::DetectionModel(net);
#endif
	};

	static Impl* AsImpl(void* p) { return reinterpret_cast<Impl*>(p); }
}

void VisionDetector::SetParams(const Params& p)
{
	m_params = p;
	// 更改参数时，强制重新加载（避免旧模型/旧输入尺寸不一致）
	if (m_impl)
	{
		delete AsImpl(m_impl);
		m_impl = nullptr;
	}
	m_loaded = false;
}

bool VisionDetector::ReadFileToBufferW(const std::wstring& path, std::vector<BYTE>& out)
{
	out.clear();
	if (path.empty()) return false;

	HANDLE h = ::CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	LARGE_INTEGER sz = {};
	if (!::GetFileSizeEx(h, &sz) || sz.QuadPart <= 0 || sz.QuadPart > (LONGLONG)(1024ll * 1024ll * 512ll))
	{
		::CloseHandle(h);
		return false;
	}

	out.resize((size_t)sz.QuadPart);
	DWORD readBytes = 0;
	const BOOL ok = ::ReadFile(h, out.data(), (DWORD)out.size(), &readBytes, nullptr);
	::CloseHandle(h);
	if (!ok || readBytes != out.size())
	{
		out.clear();
		return false;
	}
	return true;
}

std::string VisionDetector::WStringToUtf8(const std::wstring& ws)
{
	if (ws.empty()) return std::string();
	const int n = ::WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), (int)ws.size(), nullptr, 0, nullptr, nullptr);
	if (n <= 0) return std::string();
	std::string out;
	out.resize((size_t)n);
	// 注意：在 C++14/部分 MSVC STL 实现中，std::string::data() 返回 const char*，
	// 这里必须提供“可写缓冲区”，否则会触发 C2664（const char* 不能转 LPSTR）。
	::WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), (int)ws.size(), &out[0], n, nullptr, nullptr);
	return out;
}

bool VisionDetector::EnsureLoaded(std::wstring& outErr)
{
	outErr.clear();

	if (m_loaded)
	{
		return true;
	}

#if !(defined(SMARTARM_HAS_OPENCV_DNN) && SMARTARM_HAS_OPENCV_DNN)
	outErr = L"OpenCV DNN is not available in this build.";
	return false;
#else
	if (m_params.onnxPath.empty())
	{
		outErr = L"Detector ONNX path is empty.";
		return false;
	}

	std::vector<BYTE> buf;
	if (!ReadFileToBufferW(m_params.onnxPath, buf))
	{
		outErr = L"Failed to read ONNX file (path/permission).";
		return false;
	}

	try
	{
		auto* impl = new Impl();

		// 从 buffer 加载，避免中文路径导致 OpenCV 打不开文件
		std::vector<uchar> ubuf(buf.begin(), buf.end());
		impl->net = cv::dnn::readNetFromONNX(ubuf);
		impl->model = cv::dnn::DetectionModel(impl->net);
		impl->model.setInputSize(m_params.inputW, m_params.inputH);
		impl->model.setInputScale(1.0 / 255.0);
		impl->model.setInputMean(cv::Scalar(0, 0, 0));
		impl->model.setInputSwapRB(m_params.swapRB);

		// 替换旧 impl
		if (m_impl) delete AsImpl(m_impl);
		m_impl = impl;
		m_loaded = true;
		return true;
	}
	catch (...)
	{
		outErr = L"OpenCV failed to load the ONNX model (format not supported?).";
		return false;
	}
#endif
}

bool VisionDetector::DetectBest(const void* bgrData, int width, int height, int strideBytes, Detection& best)
{
	best = Detection{};
	if (!m_loaded || m_impl == nullptr)
	{
		return false;
	}

#if !(defined(SMARTARM_HAS_OPENCV_DNN) && SMARTARM_HAS_OPENCV_DNN)
	return false;
#else
	auto* impl = AsImpl(m_impl);
	if (!impl)
	{
		return false;
	}
	if (bgrData == nullptr || width <= 0 || height <= 0 || strideBytes <= 0)
	{
		return false;
	}

	try
	{
		cv::Mat bgr(height, width, CV_8UC3, const_cast<void*>(bgrData), (size_t)strideBytes);

		std::vector<int> classIds;
		std::vector<float> confidences;
		std::vector<cv::Rect> boxes;

		impl->model.detect(bgr, classIds, confidences, boxes, m_params.confThreshold, m_params.nmsThreshold);
		if (boxes.empty() || classIds.empty() || confidences.empty())
		{
			return false;
		}

		// 选取置信度最高的框
		int bi = 0;
		float bc = confidences[0];
		for (int i = 1; i < (int)confidences.size(); i++)
		{
			if (confidences[i] > bc)
			{
				bc = confidences[i];
				bi = i;
			}
		}

		const cv::Rect r = boxes[bi] & cv::Rect(0, 0, width, height);
		best.classId = classIds[bi];
		best.confidence = confidences[bi];
		best.x = r.x;
		best.y = r.y;
		best.w = r.width;
		best.h = r.height;
		return (best.w > 0 && best.h > 0);
	}
	catch (...)
	{
		return false;
	}
#endif
}


