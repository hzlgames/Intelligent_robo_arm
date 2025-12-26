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

		// ========== YOLOv5 手动解析 ==========
		// 1. 预处理：blobFromImage (resize, normalize, swapRB)
		cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0,
			cv::Size(m_params.inputW, m_params.inputH),
			cv::Scalar(0, 0, 0), m_params.swapRB, false);
		impl->net.setInput(blob);

		// 2. 前向推理
		cv::Mat output = impl->net.forward();

		// YOLOv5/Ultralytics ONNX 常见输出布局（float32）：
		// - [1, N, C] 其中 C=85 (4+1+80)
		// - [1, C, N]（转置布局）
		// - [N, C]
		// - [1, 1, N, 7]（SSD风格，旧 DetectionModel 期望）
		int anchors = 0;
		int cols = 0;
		bool transposed = false; // true => treat as [C, N]
		if (output.dims == 4 && output.size[3] == 7)
		{
			// SSD-style: leave to old path? 这里直接不支持，避免误判（项目当前目标是 YOLOv5n）
			return false;
		}
		else if (output.dims == 3)
		{
			const int d1 = output.size[1];
			const int d2 = output.size[2];
			// 经验：C 通常 <= 200，N 通常远大于 C
			if (d2 >= 6 && d2 <= 200 && d1 > d2) { anchors = d1; cols = d2; transposed = false; }
			else if (d1 >= 6 && d1 <= 200 && d2 > d1) { anchors = d2; cols = d1; transposed = true; }
			else
			{
				// 兜底：把较大维当作 anchors，较小维当作 cols
				if (d1 >= d2) { anchors = d1; cols = d2; transposed = false; }
				else { anchors = d2; cols = d1; transposed = true; }
			}
		}
		else if (output.dims == 2)
		{
			anchors = output.size[0];
			cols = output.size[1];
			transposed = false;
		}
		else
		{
			return false;
		}
		// 兼容：
		// - YOLOv5: cols = 85 => [cx,cy,w,h,obj,80 classes]
		// - YOLOv8: cols = 84 => [cx,cy,w,h,80 classes]（无 obj）
		const bool hasObj = (cols != 84) && (cols >= 6);
		const int classStart = hasObj ? 5 : 4;
		const int numClasses = cols - classStart;
		if (anchors <= 0 || cols < (classStart + 1) || numClasses <= 0) return false;

		// 解析访问器：把 output 看作 [anchors, cols]
		const float* data = (const float*)output.data;
		auto get = [&](int i, int j) -> float
		{
			if (!transposed)
			{
				// [anchors, cols] contiguous
				return data[i * cols + j];
			}
			// [cols, anchors] contiguous
			return data[j * anchors + i];
		};

		// 判断是否需要 sigmoid（若出现 <0 或 >1 的概率值，通常是 logits）
		bool useSigmoid = false;
		{
			const int sampleN = std::min(anchors, 256);
			float minP = 1e9f, maxP = -1e9f;
			const int probeCol = hasObj ? 4 : classStart;
			for (int i = 0; i < sampleN; i++)
			{
				minP = std::min(minP, get(i, probeCol));
				maxP = std::max(maxP, get(i, probeCol));
			}
			// objectness / class score 理论应在 [0,1]
			if (minP < -0.05f || maxP > 1.05f) useSigmoid = true;
		}
		auto sigmoid = [&](float x) -> float
		{
			// 简单 sigmoid；仅在必要时启用
			return 1.0f / (1.0f + std::exp(-x));
		};

		std::vector<int> classIds;
		std::vector<float> confidences;
		std::vector<cv::Rect> boxes;

		// 计算缩放因子（输入是 resize 到 inputW/inputH）
		const float scaleX = (float)width / (float)m_params.inputW;
		const float scaleY = (float)height / (float)m_params.inputH;

		for (int i = 0; i < anchors; i++)
		{
			float objectness = 1.0f;
			if (hasObj)
			{
				objectness = get(i, 4);
				if (useSigmoid) objectness = sigmoid(objectness);
				if (objectness < m_params.confThreshold) continue;
			}

			// 找最大类别分数
			int maxClassId = 0;
			float maxClassScore = get(i, classStart);
			for (int c = 1; c < numClasses; c++)
			{
				const float s = get(i, classStart + c);
				if (s > maxClassScore)
				{
					maxClassScore = s;
					maxClassId = c;
				}
			}
			if (useSigmoid) maxClassScore = sigmoid(maxClassScore);

			const float confidence = hasObj ? (objectness * maxClassScore) : maxClassScore;
			if (confidence < m_params.confThreshold)
				continue;

			// 解析边界框 (cx, cy, w, h) -> (x, y, w, h)
			float cxIn = get(i, 0);
			float cyIn = get(i, 1);
			float bwIn = get(i, 2);
			float bhIn = get(i, 3);
			// 若坐标明显是归一化（0..1），先转到 input 像素
			if (cxIn >= 0.0f && cxIn <= 1.5f && cyIn >= 0.0f && cyIn <= 1.5f && bwIn >= 0.0f && bwIn <= 1.5f && bhIn >= 0.0f && bhIn <= 1.5f)
			{
				cxIn *= (float)m_params.inputW;
				cyIn *= (float)m_params.inputH;
				bwIn *= (float)m_params.inputW;
				bhIn *= (float)m_params.inputH;
			}
			const float cx = cxIn * scaleX;
			const float cy = cyIn * scaleY;
			const float bw = bwIn * scaleX;
			const float bh = bhIn * scaleY;
			const int x = (int)(cx - bw * 0.5f);
			const int y = (int)(cy - bh * 0.5f);

			classIds.push_back(maxClassId);
			confidences.push_back(confidence);
			boxes.push_back(cv::Rect(x, y, (int)bw, (int)bh));
		}

		// 3. NMS
		std::vector<int> indices;
		cv::dnn::NMSBoxes(boxes, confidences, m_params.confThreshold, m_params.nmsThreshold, indices);

		if (indices.empty())
		{
			return false;
		}

		// 选取置信度最高的框
		int bi = indices[0];
		float bc = confidences[bi];
		for (size_t j = 1; j < indices.size(); j++)
		{
			const int idx = indices[j];
			if (confidences[idx] > bc)
			{
				bc = confidences[idx];
				bi = idx;
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

bool VisionDetector::DetectBestExcludingRect(const void* bgrData, int width, int height, int strideBytes,
                                             const RECT* excludeRect, float maxOverlap, Detection& best)
{
	if (excludeRect == nullptr)
	{
		return DetectBest(bgrData, width, height, strideBytes, best);
	}
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

	const float thr = std::max(0.0f, std::min(1.0f, maxOverlap));
	const cv::Rect ex = cv::Rect(excludeRect->left, excludeRect->top,
	                             std::max(0L, excludeRect->right - excludeRect->left),
	                             std::max(0L, excludeRect->bottom - excludeRect->top));

	try
	{
		cv::Mat bgr(height, width, CV_8UC3, const_cast<void*>(bgrData), (size_t)strideBytes);

		// 复用 DetectBest 的解析路径：这里简单调用 DetectAll 后再做排除
		std::vector<Detection> all;
		if (!DetectAll(bgr.data, width, height, (int)bgr.step, all))
		{
			return false;
		}

		// 遍历 NMS 后的检测结果，选第一个"未被排除"的框
		for (size_t j = 0; j < all.size(); j++)
		{
			const Detection& d = all[j];
			cv::Rect r = cv::Rect(d.x, d.y, d.w, d.h) & cv::Rect(0, 0, width, height);
			const int area = r.area();
			float overlap = 0.0f;
			if (area > 0 && ex.area() > 0)
			{
				const cv::Rect inter = r & ex;
				const int ia = inter.area();
				if (ia > 0) overlap = (float)ia / (float)area;
			}
			if (overlap <= thr)
			{
				best.classId = d.classId;
				best.confidence = d.confidence;
				best.x = r.x;
				best.y = r.y;
				best.w = r.width;
				best.h = r.height;
				return (best.w > 0 && best.h > 0);
			}
		}
		return false;
	}
	catch (...)
	{
		return false;
	}
#endif
}

bool VisionDetector::DetectAll(const void* bgrData, int width, int height, int strideBytes, std::vector<Detection>& out)
{
	out.clear();
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

		// ========== YOLOv5 手动解析 ==========
		cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0,
			cv::Size(m_params.inputW, m_params.inputH),
			cv::Scalar(0, 0, 0), m_params.swapRB, false);
		impl->net.setInput(blob);
		cv::Mat output = impl->net.forward();

		int anchors = 0;
		int cols = 0;
		bool transposed = false;
		if (output.dims == 3)
		{
			const int d1 = output.size[1];
			const int d2 = output.size[2];
			if (d2 >= 6 && d2 <= 200 && d1 > d2) { anchors = d1; cols = d2; transposed = false; }
			else if (d1 >= 6 && d1 <= 200 && d2 > d1) { anchors = d2; cols = d1; transposed = true; }
			else
			{
				if (d1 >= d2) { anchors = d1; cols = d2; transposed = false; }
				else { anchors = d2; cols = d1; transposed = true; }
			}
		}
		else if (output.dims == 2)
		{
			anchors = output.size[0];
			cols = output.size[1];
			transposed = false;
		}
		else
		{
			return false;
		}
		const bool hasObj = (cols != 84) && (cols >= 6);
		const int classStart = hasObj ? 5 : 4;
		const int numClasses = cols - classStart;
		if (anchors <= 0 || cols < (classStart + 1) || numClasses <= 0) return false;

		const float* data = (const float*)output.data;
		auto get = [&](int i, int j) -> float
		{
			if (!transposed) return data[i * cols + j];
			return data[j * anchors + i];
		};
		bool useSigmoid = false;
		{
			const int sampleN = std::min(anchors, 256);
			float minP = 1e9f, maxP = -1e9f;
			const int probeCol = hasObj ? 4 : classStart;
			for (int i = 0; i < sampleN; i++)
			{
				minP = std::min(minP, get(i, probeCol));
				maxP = std::max(maxP, get(i, probeCol));
			}
			if (minP < -0.05f || maxP > 1.05f) useSigmoid = true;
		}
		auto sigmoid = [&](float x) -> float { return 1.0f / (1.0f + std::exp(-x)); };

		const float scaleX = (float)width / (float)m_params.inputW;
		const float scaleY = (float)height / (float)m_params.inputH;

		std::vector<int> classIds;
		std::vector<float> confidences;
		std::vector<cv::Rect> boxes;

		for (int i = 0; i < anchors; i++)
		{
			float objectness = 1.0f;
			if (hasObj)
			{
				objectness = get(i, 4);
				if (useSigmoid) objectness = sigmoid(objectness);
				if (objectness < m_params.confThreshold) continue;
			}
			int maxClassId = 0;
			float maxClassScore = get(i, classStart);
			for (int c = 1; c < numClasses; c++)
			{
				const float s = get(i, classStart + c);
				if (s > maxClassScore) { maxClassScore = s; maxClassId = c; }
			}
			if (useSigmoid) maxClassScore = sigmoid(maxClassScore);
			const float confidence = hasObj ? (objectness * maxClassScore) : maxClassScore;
			if (confidence < m_params.confThreshold) continue;
			float cxIn = get(i, 0);
			float cyIn = get(i, 1);
			float bwIn = get(i, 2);
			float bhIn = get(i, 3);
			if (cxIn >= 0.0f && cxIn <= 1.5f && cyIn >= 0.0f && cyIn <= 1.5f && bwIn >= 0.0f && bwIn <= 1.5f && bhIn >= 0.0f && bhIn <= 1.5f)
			{
				cxIn *= (float)m_params.inputW;
				cyIn *= (float)m_params.inputH;
				bwIn *= (float)m_params.inputW;
				bhIn *= (float)m_params.inputH;
			}
			const float cx = cxIn * scaleX;
			const float cy = cyIn * scaleY;
			const float bw = bwIn * scaleX;
			const float bh = bhIn * scaleY;
			classIds.push_back(maxClassId);
			confidences.push_back(confidence);
			boxes.push_back(cv::Rect((int)(cx - bw * 0.5f), (int)(cy - bh * 0.5f), (int)bw, (int)bh));
		}

		std::vector<int> indices;
		cv::dnn::NMSBoxes(boxes, confidences, m_params.confThreshold, m_params.nmsThreshold, indices);
		if (indices.empty()) return false;
		std::sort(indices.begin(), indices.end(), [&](int a, int b) { return confidences[a] > confidences[b]; });

		out.reserve(indices.size());
		for (size_t j = 0; j < indices.size(); j++)
		{
			const int bi = indices[j];
			const cv::Rect r = boxes[bi] & cv::Rect(0, 0, width, height);
			if (r.width <= 0 || r.height <= 0) continue;
			Detection d{};
			d.classId = classIds[bi];
			d.confidence = confidences[bi];
			d.x = r.x;
			d.y = r.y;
			d.w = r.width;
			d.h = r.height;
			out.push_back(d);
		}
		return !out.empty();
	}
	catch (...)
	{
		out.clear();
		return false;
	}
#endif
}


