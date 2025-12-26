#include "pch.h"

#include "VisionHandLandmarks.h"

#include <algorithm>
#include <cmath>

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
		cv::dnn::Net palmNet;
		cv::dnn::Net handNet;
		std::vector<cv::Point2f> anchors;
#endif
	};

	static Impl* AsImpl(void* p) { return reinterpret_cast<Impl*>(p); }

	static inline float Sigmoid(float x)
	{
		return 1.0f / (1.0f + std::exp(-x));
	}

	static inline double Norm2(double x, double y)
	{
		return std::sqrt(x * x + y * y);
	}

#if defined(SMARTARM_HAS_OPENCV_DNN) && SMARTARM_HAS_OPENCV_DNN
	static bool CropAndPadFromPalm(
		const cv::Mat& image,
		const cv::Point2f& inTL,
		const cv::Point2f& inBR,
		bool for_rotation,
		cv::Mat& outImage,
		cv::Point2f& outTL,
		cv::Point2f& outBR,
		cv::Point2f& outBias)
	{
		// shift & enlarge factors from OpenCV Zoo mp_handpose.py
		const cv::Point2f preShift(0.0f, 0.0f);
		const float preEnlarge = 4.0f;
		const cv::Point2f shift(0.0f, -0.4f);
		const float enlarge = 3.0f;

		cv::Point2f tl = inTL;
		cv::Point2f br = inBR;
		cv::Point2f wh = br - tl;

		const cv::Point2f sv = for_rotation ? preShift : shift;
		tl += cv::Point2f(sv.x * wh.x, sv.y * wh.y);
		br += cv::Point2f(sv.x * wh.x, sv.y * wh.y);

		// enlarge around center
		const cv::Point2f center = (tl + br) * 0.5f;
		wh = br - tl;
		const float sc = for_rotation ? preEnlarge : enlarge;
		const cv::Point2f half = wh * (sc * 0.5f);
		tl = center - half;
		br = center + half;

		// clip to image bounds
		const float W = (float)image.cols;
		const float H = (float)image.rows;
		tl.x = std::max(0.0f, std::min(W, tl.x));
		tl.y = std::max(0.0f, std::min(H, tl.y));
		br.x = std::max(0.0f, std::min(W, br.x));
		br.y = std::max(0.0f, std::min(H, br.y));

		const int x1 = (int)std::floor(tl.x);
		const int y1 = (int)std::floor(tl.y);
		const int x2 = (int)std::ceil(br.x);
		const int y2 = (int)std::ceil(br.y);

		const int cw = std::max(0, x2 - x1);
		const int ch = std::max(0, y2 - y1);
		if (cw <= 1 || ch <= 1) return false;

		cv::Rect roi(x1, y1, cw, ch);
		roi &= cv::Rect(0, 0, image.cols, image.rows);
		if (roi.width <= 1 || roi.height <= 1) return false;

		cv::Mat cropped = image(roi).clone();

		// pad to square so that rotation does not crop corners
		const int side_len = for_rotation ? (int)std::ceil(Norm2((double)cropped.rows, (double)cropped.cols))
		                                  : std::max(cropped.rows, cropped.cols);
		const int pad_h = std::max(0, side_len - cropped.rows);
		const int pad_w = std::max(0, side_len - cropped.cols);
		const int left = pad_w / 2;
		const int top = pad_h / 2;
		const int right = pad_w - left;
		const int bottom = pad_h - top;

		cv::copyMakeBorder(cropped, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

		outTL = cv::Point2f((float)x1, (float)y1);
		outBR = cv::Point2f((float)(x1 + roi.width), (float)(y1 + roi.height));
		// bias: original_coord = crop_coord + bias
		outBias = outTL - cv::Point2f((float)left, (float)top);
		return true;
	}
#endif
}

void VisionHandLandmarks::SetParams(const Params& p)
{
	m_params = p;
	// force reload
	if (m_impl)
	{
		delete AsImpl(m_impl);
		m_impl = nullptr;
	}
	m_loaded = false;
}

bool VisionHandLandmarks::ReadFileToBufferW(const std::wstring& path, std::vector<BYTE>& out)
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

bool VisionHandLandmarks::EnsureLoaded(std::wstring& outErr)
{
	outErr.clear();
	if (m_loaded) return true;

#if !(defined(SMARTARM_HAS_OPENCV_DNN) && SMARTARM_HAS_OPENCV_DNN)
	outErr = L"OpenCV DNN is not available in this build.";
	return false;
#else
	if (m_params.palmOnnxPath.empty() || m_params.handposeOnnxPath.empty())
	{
		outErr = L"Hand models are not configured (PalmOnnxPath / HandposeOnnxPath).";
		return false;
	}

	std::vector<BYTE> bufPalm;
	std::vector<BYTE> bufHand;
	if (!ReadFileToBufferW(m_params.palmOnnxPath, bufPalm))
	{
		outErr = L"Failed to read palm ONNX file (path/permission).";
		return false;
	}
	if (!ReadFileToBufferW(m_params.handposeOnnxPath, bufHand))
	{
		outErr = L"Failed to read handpose ONNX file (path/permission).";
		return false;
	}

	try
	{
		auto* impl = new Impl();
		{
			std::vector<uchar> ubuf(bufPalm.begin(), bufPalm.end());
			impl->palmNet = cv::dnn::readNetFromONNX(ubuf);
		}
		{
			std::vector<uchar> ubuf(bufHand.begin(), bufHand.end());
			impl->handNet = cv::dnn::readNetFromONNX(ubuf);
		}

		// Generate anchors programmatically (matches OpenCV Zoo mp_palmdet.py)
		impl->anchors.clear();
		impl->anchors.reserve(2016);
		// layer 1: 24x24, 2 anchors per location
		for (int y = 0; y < 24; y++)
		{
			for (int x = 0; x < 24; x++)
			{
				const float cx = ((float)x + 0.5f) / 24.0f;
				const float cy = ((float)y + 0.5f) / 24.0f;
				impl->anchors.push_back(cv::Point2f(cx, cy));
				impl->anchors.push_back(cv::Point2f(cx, cy));
			}
		}
		// layer 2: 12x12, 6 anchors per location
		for (int y = 0; y < 12; y++)
		{
			for (int x = 0; x < 12; x++)
			{
				const float cx = ((float)x + 0.5f) / 12.0f;
				const float cy = ((float)y + 0.5f) / 12.0f;
				for (int k = 0; k < 6; k++)
				{
					impl->anchors.push_back(cv::Point2f(cx, cy));
				}
			}
		}

		if (m_impl) delete AsImpl(m_impl);
		m_impl = impl;
		m_loaded = true;
		return true;
	}
	catch (...)
	{
		outErr = L"OpenCV failed to load palm/handpose ONNX models.";
		return false;
	}
#endif
}

bool VisionHandLandmarks::DetectBest(const void* bgrData, int width, int height, int strideBytes, Hand& outHand)
{
	outHand = Hand{};
	if (!m_loaded || m_impl == nullptr) return false;

#if !(defined(SMARTARM_HAS_OPENCV_DNN) && SMARTARM_HAS_OPENCV_DNN)
	return false;
#else
	auto* impl = AsImpl(m_impl);
	if (!impl) return false;
	if (!bgrData || width <= 0 || height <= 0 || strideBytes <= 0) return false;

	const cv::Mat bgr(height, width, CV_8UC3, const_cast<void*>(bgrData), (size_t)strideBytes);

	// -------------------------
	// 1) Palm detect
	// -------------------------
	cv::Mat palmBlob;
	cv::Point2i pad_bias(0, 0);
	{
		const cv::Size input_size(192, 192);
		const float ratio = std::min((float)input_size.width / (float)width, (float)input_size.height / (float)height);

		cv::Mat resized;
		if (width != input_size.width || height != input_size.height)
		{
			const cv::Size ratio_size((int)std::lround((double)width * (double)ratio), (int)std::lround((double)height * (double)ratio));
			cv::resize(bgr, resized, ratio_size);
			const int pad_h = input_size.height - ratio_size.height;
			const int pad_w = input_size.width - ratio_size.width;
			pad_bias.x = pad_w / 2;
			pad_bias.y = pad_h / 2;
			cv::copyMakeBorder(resized, resized, pad_bias.y, pad_h - pad_bias.y, pad_bias.x, pad_w - pad_bias.x, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
		}
		else
		{
			resized = bgr;
		}

		cv::dnn::Image2BlobParams params;
		params.datalayout = cv::dnn::DNN_LAYOUT_NHWC;
		params.ddepth = CV_32F;
		params.mean = cv::Scalar::all(0);
		params.scalefactor = cv::Scalar::all(1.0 / 255.0);
		params.size = input_size;
		params.swapRB = true; // BGR->RGB
		params.paddingmode = cv::dnn::DNN_PMODE_NULL;
		palmBlob = cv::dnn::blobFromImageWithParams(resized, params);

		pad_bias.x = (int)std::lround((double)pad_bias.x / (double)ratio);
		pad_bias.y = (int)std::lround((double)pad_bias.y / (double)ratio);
	}

	impl->palmNet.setInput(palmBlob);
	std::vector<cv::Mat> palmOuts;
	impl->palmNet.forward(palmOuts, impl->palmNet.getUnconnectedOutLayersNames());
	if (palmOuts.size() < 2) return false;

	// Identify boxes+landmarks and scores by element count (robust to order)
	cv::Mat outBoxes = palmOuts[0];
	cv::Mat outScores = palmOuts[1];
	if (outBoxes.total() < outScores.total())
	{
		std::swap(outBoxes, outScores);
	}

	const int kPalmDims = 18;
	cv::Mat scores = outScores.reshape(1, (int)(outScores.total()));
	cv::Mat boxes = outBoxes.reshape(1, (int)(outBoxes.total() / kPalmDims));
	if (scores.rows != boxes.rows) return false;

	const float scale = (float)std::max(width, height);

	std::vector<float> candScores;
	std::vector<cv::Rect2f> candBoxes;
	std::vector<std::array<float, 14>> candLm;
	candScores.reserve(64);
	candBoxes.reserve(64);
	candLm.reserve(64);

	for (int i = 0; i < scores.rows && i < (int)impl->anchors.size(); i++)
	{
		const float s = Sigmoid(scores.at<float>(i, 0));
		if (s < m_params.palmScoreThreshold) continue;

		const cv::Mat row = boxes.row(i);
		const float dx = row.at<float>(0, 0);
		const float dy = row.at<float>(0, 1);
		const float dw = row.at<float>(0, 2);
		const float dh = row.at<float>(0, 3);

		const cv::Point2f anchor = impl->anchors[i];
		const cv::Point2f cxy(dx / 192.0f, dy / 192.0f);
		const cv::Point2f wh(dw / 192.0f, dh / 192.0f);

		const cv::Point2f xy1((cxy.x - wh.x * 0.5f + anchor.x) * scale - pad_bias.x,
		                      (cxy.y - wh.y * 0.5f + anchor.y) * scale - pad_bias.y);
		const cv::Point2f xy2((cxy.x + wh.x * 0.5f + anchor.x) * scale - pad_bias.x,
		                      (cxy.y + wh.y * 0.5f + anchor.y) * scale - pad_bias.y);

		cv::Rect2f r(xy1.x, xy1.y, xy2.x - xy1.x, xy2.y - xy1.y);
		if (r.width <= 1 || r.height <= 1) continue;

		std::array<float, 14> lm{};
		for (int j = 0; j < 7; j++)
		{
			const float ldx = row.at<float>(0, 4 + j * 2) / 192.0f + anchor.x;
			const float ldy = row.at<float>(0, 4 + j * 2 + 1) / 192.0f + anchor.y;
			lm[j * 2] = ldx * scale - pad_bias.x;
			lm[j * 2 + 1] = ldy * scale - pad_bias.y;
		}

		candScores.push_back(s);
		candBoxes.push_back(r);
		candLm.push_back(lm);
	}

	if (candBoxes.empty()) return false;

	std::vector<cv::Rect> boxesInt;
	boxesInt.reserve(candBoxes.size());
	for (const auto& r : candBoxes)
	{
		boxesInt.push_back(cv::Rect((int)r.x, (int)r.y, (int)r.width, (int)r.height));
	}

	std::vector<int> keep;
	cv::dnn::NMSBoxes(boxesInt, candScores, m_params.palmScoreThreshold, m_params.palmNmsThreshold, keep, 1.f, m_params.palmTopK);
	if (keep.empty()) return false;

	int bestIdx = keep[0];
	for (int idx : keep)
	{
		if (candScores[idx] > candScores[bestIdx]) bestIdx = idx;
	}

	Palm palm;
	{
		const cv::Rect2f r = candBoxes[bestIdx];
		palm.x1 = r.x;
		palm.y1 = r.y;
		palm.x2 = r.x + r.width;
		palm.y2 = r.y + r.height;
		palm.lm = candLm[bestIdx];
		palm.score = candScores[bestIdx];
	}

	// -------------------------
	// 2) Handpose (21 landmarks)
	// -------------------------
	{
		// Prepare palm bbox and landmarks in original coords
		const cv::Point2f palmTL(palm.x1, palm.y1);
		const cv::Point2f palmBR(palm.x2, palm.y2);

		cv::Mat cropPaddedBgr;
		cv::Point2f tl2, br2, bias;
		if (!CropAndPadFromPalm(bgr, palmTL, palmBR, true, cropPaddedBgr, tl2, br2, bias))
		{
			return false;
		}

		// Convert to RGB for the handpose pipeline
		cv::Mat cropRgb;
		cv::cvtColor(cropPaddedBgr, cropRgb, cv::COLOR_BGR2RGB);

		// bbox / landmarks in crop coords
		const cv::Point2f bboxTL = tl2 - bias;
		const cv::Point2f bboxBR = br2 - bias;

		std::array<cv::Point2f, 7> palmLm{};
		for (int j = 0; j < 7; j++)
		{
			const float x = palm.lm[j * 2];
			const float y = palm.lm[j * 2 + 1];
			palmLm[j] = cv::Point2f(x, y) - bias;
		}

		// rotation angle (matches mp_handpose.py)
		const cv::Point2f p1 = palmLm[0];
		const cv::Point2f p2 = palmLm[2];
		double radians = 3.14159265358979323846 / 2.0 - std::atan2(-(double)(p2.y - p1.y), (double)(p2.x - p1.x));
		radians = radians - 2.0 * 3.14159265358979323846 * std::floor((radians + 3.14159265358979323846) / (2.0 * 3.14159265358979323846));
		const double angleDeg = radians * 180.0 / 3.14159265358979323846;

		const cv::Point2f center = (bboxTL + bboxBR) * 0.5f;
		const cv::Mat rotMat = cv::getRotationMatrix2D(center, angleDeg, 1.0);

		cv::Mat rotatedRgb;
		cv::warpAffine(cropRgb, rotatedRgb, rotMat, cropRgb.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

		// rotate palm landmarks to get a tight bbox
		std::array<cv::Point2f, 7> rotatedLm{};
		for (int j = 0; j < 7; j++)
		{
			const cv::Point2f pt = palmLm[j];
			const double x = rotMat.at<double>(0, 0) * pt.x + rotMat.at<double>(0, 1) * pt.y + rotMat.at<double>(0, 2);
			const double y = rotMat.at<double>(1, 0) * pt.x + rotMat.at<double>(1, 1) * pt.y + rotMat.at<double>(1, 2);
			rotatedLm[j] = cv::Point2f((float)x, (float)y);
		}

		cv::Point2f rMin = rotatedLm[0];
		cv::Point2f rMax = rotatedLm[0];
		for (int j = 1; j < 7; j++)
		{
			rMin.x = std::min(rMin.x, rotatedLm[j].x);
			rMin.y = std::min(rMin.y, rotatedLm[j].y);
			rMax.x = std::max(rMax.x, rotatedLm[j].x);
			rMax.y = std::max(rMax.y, rotatedLm[j].y);
		}

		cv::Mat handCrop;
		cv::Point2f rtl, rbr, bias2;
		if (!CropAndPadFromPalm(rotatedRgb, rMin, rMax, false, handCrop, rtl, rbr, bias2))
		{
			return false;
		}

		// Build blob for handpose net (RGB, NHWC, 224x224, /255)
		cv::dnn::Image2BlobParams params;
		params.datalayout = cv::dnn::DNN_LAYOUT_NHWC;
		params.ddepth = CV_32F;
		params.mean = cv::Scalar::all(0);
		params.scalefactor = cv::Scalar::all(1.0 / 255.0);
		params.size = cv::Size(224, 224);
		params.swapRB = false; // already RGB
		params.paddingmode = cv::dnn::DNN_PMODE_NULL;
		cv::Mat handBlob = cv::dnn::blobFromImageWithParams(handCrop, params);

		impl->handNet.setInput(handBlob);
		std::vector<cv::Mat> handOuts;
		impl->handNet.forward(handOuts, impl->handNet.getUnconnectedOutLayersNames());
		if (handOuts.empty()) return false;

		cv::Mat lmMat, confMat;
		for (const auto& m : handOuts)
		{
			if (m.total() == 63 && lmMat.empty())
			{
				lmMat = m;
			}
			else if (m.total() == 1 && confMat.empty())
			{
				confMat = m;
			}
		}
		if (lmMat.empty() || confMat.empty()) return false;

		const float conf = confMat.reshape(1, 1).at<float>(0, 0);
		if (conf < m_params.handposeConfThreshold) return false;

		// decode landmarks
		const float* lmPtr = (const float*)lmMat.ptr<float>();
		if (!lmPtr) return false;

		// rotated palm bbox size (used for scaling) - use the bbox returned from CropAndPadFromPalm on rotatedRgb
		const float rotW = std::max(1.0f, rbr.x - rtl.x);
		const float rotH = std::max(1.0f, rbr.y - rtl.y);
		const float scalePx = std::max(rotW, rotH) / 224.0f;

		// rotation around origin for landmark coords
		const double ca = std::cos(angleDeg * 3.14159265358979323846 / 180.0);
		const double sa = std::sin(angleDeg * 3.14159265358979323846 / 180.0);

		// Invert affine transform for center mapping
		cv::Mat invRotMat;
		cv::invertAffineTransform(rotMat, invRotMat);

		// center of rotated bbox in rotatedRgb coords
		const cv::Point2f rotCenter = (rtl + rbr) * 0.5f;
		const double ocx = invRotMat.at<double>(0, 0) * rotCenter.x + invRotMat.at<double>(0, 1) * rotCenter.y + invRotMat.at<double>(0, 2);
		const double ocy = invRotMat.at<double>(1, 0) * rotCenter.x + invRotMat.at<double>(1, 1) * rotCenter.y + invRotMat.at<double>(1, 2);

		// Map to original image coords: (rotated_landmarks + original_center + bias)
		std::array<float, 42> pts{};
		float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
		for (int i = 0; i < 21; i++)
		{
			const float x0 = lmPtr[i * 3 + 0];
			const float y0 = lmPtr[i * 3 + 1];
			// const float z0 = lmPtr[i * 3 + 2];

			// center & scale (match mp_handpose.py)
			const double x = ((double)x0 - 112.0) * (double)scalePx;
			const double y = ((double)y0 - 112.0) * (double)scalePx;

			// rotate back by angle around origin
			const double xr = x * ca - y * sa;
			const double yr = x * sa + y * ca;

			// translate to crop coords, then to original coords
			const double xo = xr + ocx + (double)bias.x;
			const double yo = yr + ocy + (double)bias.y;

			pts[i * 2 + 0] = (float)xo;
			pts[i * 2 + 1] = (float)yo;
			minX = std::min(minX, (float)xo);
			minY = std::min(minY, (float)yo);
			maxX = std::max(maxX, (float)xo);
			maxY = std::max(maxY, (float)yo);
		}

		outHand.valid = true;
		outHand.confidence = conf;
		outHand.pts = pts;
		outHand.x1 = minX;
		outHand.y1 = minY;
		outHand.x2 = maxX;
		outHand.y2 = maxY;
		return true;
	}
#endif
}


