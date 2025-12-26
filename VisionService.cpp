#include "pch.h"

#include "VisionService.h"

#include "preview.h"
#include "VisualServoController.h"

#include "VisionDetector.h"
#include "VisionGeometry.h"
#include "VisionOverlayService.h"

#include <algorithm>
#include <cmath>

// Optional OpenCV support (vcpkg opencv4 provides core/imgproc; aruco depends on contrib build).
#if defined(__has_include)
#if __has_include(<opencv2/core.hpp>) && __has_include(<opencv2/imgproc.hpp>)
#define SMARTARM_HAS_OPENCV 1
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV && __has_include(<opencv2/calib3d.hpp>)
#define SMARTARM_HAS_OPENCV_CALIB3D 1
#include <opencv2/calib3d.hpp>
#endif
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV && __has_include(<opencv2/aruco.hpp>)
#define SMARTARM_HAS_OPENCV_ARUCO 1
#include <opencv2/aruco.hpp>
#endif
#endif

namespace
{
	inline double Clamp(double v, double mn, double mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}

	inline double Lerp(double a, double b, double t)
	{
		return a + (b - a) * t;
	}
}

VisionService::VisionService()
{
}

VisionService::~VisionService()
{
	Stop();
}

void VisionService::SetDetectorParams(const VisionDetector::Params& p)
{
	std::lock_guard<std::mutex> lk(m_detMu);
	m_detector.SetParams(p);
	m_lastDetLoadAttemptMs = 0;
}

VisionDetector::Params VisionService::GetDetectorParams() const
{
	std::lock_guard<std::mutex> lk(m_detMu);
	return m_detector.GetParams();
}

void VisionService::SetHandParams(const VisionHandLandmarks::Params& p)
{
	std::lock_guard<std::mutex> lk(m_handMu);
	m_hand.SetParams(p);
	m_lastHandLoadAttemptMs = 0;
}

VisionHandLandmarks::Params VisionService::GetHandParams() const
{
	std::lock_guard<std::mutex> lk(m_handMu);
	return m_hand.GetParams();
}

void VisionService::SetParams(const Params& p)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_params = p;
	m_params.processPeriodMs = std::max(5, m_params.processPeriodMs);
	m_params.sampleStride = std::max(1, m_params.sampleStride);
	m_params.emaAlpha = Clamp(m_params.emaAlpha, 0.0, 1.0);
	m_params.arucoMarkerLengthMm = std::max(1.0, m_params.arucoMarkerLengthMm);
	m_params.depthNearMm = std::max(1, m_params.depthNearMm);
	m_params.depthFarMm = std::max(m_params.depthNearMm + 1, m_params.depthFarMm);
	m_params.excludeHandInflatePx = std::max(0, m_params.excludeHandInflatePx);
	m_params.excludeHandMaxOverlap = Clamp(m_params.excludeHandMaxOverlap, 0.0, 1.0);
	m_params.pointPickMaxRayLenPx = std::max(10, m_params.pointPickMaxRayLenPx);
	m_params.pointPickMaxRayPerpPx = std::max(5, m_params.pointPickMaxRayPerpPx);
	m_params.pointPickMaxRadiusPx = std::max(5, m_params.pointPickMaxRadiusPx);
	m_params.pointPickHoldLockMs = std::max(100, m_params.pointPickHoldLockMs);
	m_params.pointPickHoldConfirmMs = std::max(100, m_params.pointPickHoldConfirmMs);
	m_params.pointPickHoldCancelMs = std::max(100, m_params.pointPickHoldCancelMs);
	m_params.pointPickCancelFlashMs = std::max(0, m_params.pointPickCancelFlashMs);
	m_params.pointPickIouSame = Clamp(m_params.pointPickIouSame, 0.0, 1.0);
}

VisionService::Params VisionService::GetParams() const
{
	std::lock_guard<std::mutex> lk(m_mu);
	return m_params;
}

void VisionService::SetPreview(CPreview* preview)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_preview = preview;
}

void VisionService::SetVisualServo(VisualServoController* vs)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_vs = vs;
}

void VisionService::SetEnabled(bool on)
{
	m_enabled.store(on);
}

VisionService::Stats VisionService::GetStats() const
{
	std::lock_guard<std::mutex> lk(m_statsMu);
	return m_stats;
}

VisionService::Result VisionService::GetLastResult() const
{
	std::lock_guard<std::mutex> lk(m_resMu);
	return m_lastResult;
}

void VisionService::Start()
{
	if (m_running.exchange(true))
	{
		return; // already running
	}

	{
		std::lock_guard<std::mutex> lk(m_statsMu);
		m_stats = Stats{};
		m_stats.running = true;
	}

	m_th = std::thread([this]() { ThreadMain(); });
}

void VisionService::Stop()
{
	if (!m_running.exchange(false))
	{
		return;
	}
	if (m_th.joinable())
	{
		m_th.join();
	}
	{
		std::lock_guard<std::mutex> lk(m_statsMu);
		m_stats.running = false;
	}
}

void VisionService::ThreadMain()
{
	ULONGLONG lastTick = ::GetTickCount64();
	unsigned frames = 0;
	ULONGLONG fpsTick = lastTick;

	// Reuse buffers in the loop (still a copy from preview, but avoids reallocation in our code).
	std::vector<BYTE> rgb;
	UINT w = 0, h = 0;

	// Exclude-hand cache (thread-local)
	RECT lastHandRect{};
	bool hasLastHandRect = false;
	ULONGLONG lastHandRectTickMs = 0;

	// PointPick state machine (thread-local)
	struct PickFsm
	{
		int state = 0; // 0=None,1=Searching,2=Locked,3=Confirmed,4=Cancelled
		VisionOverlayService::RectI box{};
		ULONGLONG stableSinceMs = 0;
		ULONGLONG missingSinceMs = 0;
		ULONGLONG pinchSinceMs = 0;
		ULONGLONG palmSinceMs = 0;
		ULONGLONG cancelUntilMs = 0;
		VisionOverlayService::RectI lastCand{};
		bool hasLastCand = false;
	};
	PickFsm pick{};
	ULONGLONG lastPickDetTickMs = 0;

	while (m_running.load())
	{
		Params p;
		CPreview* preview = nullptr;
		VisualServoController* vs = nullptr;
		{
			std::lock_guard<std::mutex> lk(m_mu);
			p = m_params;
			preview = m_preview;
			vs = m_vs;
		}

		if (!m_enabled.load() || preview == nullptr || vs == nullptr)
		{
			::Sleep(30);
			continue;
		}

		const bool ok = preview->CopyLastRgb(rgb, w, h);
		if (!ok || w == 0 || h == 0 || rgb.empty())
		{
			::Sleep(30);
			continue;
		}

		const Mode mode = (Mode)m_mode.load();

		bool hasTarget = false;
		double u = 0.0;
		double v = 0.0;
		double conf = 0.0;
		bool hasRay = false;
		double rayX = 0.0, rayY = 0.0, rayZ = 1.0;
		bool hasDepth = false;
		double depthMm = 0.0;
		VisionDetector::Detection detBox{};
		bool hasHandLm = false;
		VisionOverlayService::Gesture handGesture = VisionOverlayService::Gesture::Unknown;
		double handPinchStrength = 0.0;
		std::array<VisionOverlayService::Point2, 21> handPts{};
		bool hasArucoCorners = false;
		std::array<VisionOverlayService::Point2, 4> arucoCorners{};
		bool hasTrackBox = false;
		VisionOverlayService::RectI trackBox{};

		auto clampTrackBox = [&](int x, int y, int ww, int hh)
		{
			VisionOverlayService::RectI r{};
			if ((int)w <= 0 || (int)h <= 0) return r;
			int xx = std::max(0, std::min((int)w - 1, x));
			int yy = std::max(0, std::min((int)h - 1, y));
			int x2 = std::max(0, std::min((int)w, xx + std::max(0, ww)));
			int y2 = std::max(0, std::min((int)h, yy + std::max(0, hh)));
			r.x = xx;
			r.y = yy;
			r.w = std::max(0, x2 - xx);
			r.h = std::max(0, y2 - yy);
			return r;
		};

		auto rectIou = [&](const VisionOverlayService::RectI& a, const VisionOverlayService::RectI& b) -> double
		{
			if (a.w <= 0 || a.h <= 0 || b.w <= 0 || b.h <= 0) return 0.0;
			const int ax2 = a.x + a.w;
			const int ay2 = a.y + a.h;
			const int bx2 = b.x + b.w;
			const int by2 = b.y + b.h;
			const int ix1 = std::max(a.x, b.x);
			const int iy1 = std::max(a.y, b.y);
			const int ix2 = std::min(ax2, bx2);
			const int iy2 = std::min(ay2, by2);
			const int iw = std::max(0, ix2 - ix1);
			const int ih = std::max(0, iy2 - iy1);
			const double inter = (double)iw * (double)ih;
			const double uni = (double)a.w * (double)a.h + (double)b.w * (double)b.h - inter;
			if (uni <= 1e-6) return 0.0;
			return inter / uni;
		};

		auto updateHold = [&](bool cond, ULONGLONG& sinceMs, ULONGLONG now) -> void
		{
			if (cond)
			{
				if (sinceMs == 0) sinceMs = now;
			}
			else
			{
				sinceMs = 0;
			}
		};

		// ==========================
		// Optional: exclude hand region (for object target selection)
		// ==========================
		bool hasExcludeRect = false;
		RECT excludeRect{};
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
		if (p.excludeHand && mode != Mode::HandLandmarks && mode != Mode::HandSticker)
		{
			// only meaningful for “object target” modes
			const bool needExclude =
				(mode == Mode::Detector) || (mode == Mode::Auto) || (mode == Mode::ColorTrack) || (mode == Mode::BrightestPoint);
			const ULONGLONG nowEx = ::GetTickCount64();
			if (needExclude && (hasLastHandRect && (nowEx - lastHandRectTickMs) <= 120))
			{
				excludeRect = lastHandRect;
				hasExcludeRect = true;
			}
			else if (needExclude)
			{
				// Ensure hand model loaded occasionally
				bool loaded = false;
				{
					std::lock_guard<std::mutex> lk(m_handMu);
					loaded = m_hand.IsLoaded();
					if (!loaded && (nowEx - m_lastHandLoadAttemptMs > 1000))
					{
						m_lastHandLoadAttemptMs = nowEx;
						std::wstring err;
						(void)m_hand.EnsureLoaded(err);
						loaded = m_hand.IsLoaded();
					}
				}

				if (loaded)
				{
					try
					{
						cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
						cv::Mat bgr;
						cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

						VisionHandLandmarks::Hand hand;
						{
							std::lock_guard<std::mutex> lk(m_handMu);
							(void)m_hand.DetectBest(bgr.data, (int)bgr.cols, (int)bgr.rows, (int)bgr.step, hand);
						}
						if (hand.valid)
						{
							int x1 = (int)std::floor((double)hand.x1);
							int y1 = (int)std::floor((double)hand.y1);
							int x2 = (int)std::ceil((double)hand.x2);
							int y2 = (int)std::ceil((double)hand.y2);
							// inflate
							const int inf = p.excludeHandInflatePx;
							x1 -= inf; y1 -= inf; x2 += inf; y2 += inf;
							x1 = std::max(0, std::min((int)w - 1, x1));
							y1 = std::max(0, std::min((int)h - 1, y1));
							x2 = std::max(0, std::min((int)w, x2));
							y2 = std::max(0, std::min((int)h, y2));
							if (x2 > x1 && y2 > y1)
							{
								excludeRect.left = x1;
								excludeRect.top = y1;
								excludeRect.right = x2;
								excludeRect.bottom = y2;
								hasExcludeRect = true;
								lastHandRect = excludeRect;
								hasLastHandRect = true;
								lastHandRectTickMs = nowEx;
							}
						}
					}
					catch (...)
					{
						// ignore
					}
				}
			}
		}
#endif

		// =========
		// 1) ArUco
		// =========
#if defined(SMARTARM_HAS_OPENCV_ARUCO) && SMARTARM_HAS_OPENCV_ARUCO
		if (mode == Mode::Aruco || mode == Mode::Auto)
		{
			try
			{
				cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
				cv::Mat gray;
				cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);

				std::vector<int> ids;
				std::vector<std::vector<cv::Point2f>> corners;
				// OpenCV 4.12+：getPredefinedDictionary 返回 Dictionary（按值），detectMarkers 需要 Ptr<Dictionary>
				const auto dictVal = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
				const cv::Ptr<cv::aruco::Dictionary> dict = cv::makePtr<cv::aruco::Dictionary>(dictVal);
				// OpenCV 4.12+：DetectorParameters 是 struct，不再提供 ::create()
				const cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
				cv::aruco::detectMarkers(gray, dict, corners, ids, params);

				if (!ids.empty() && !corners.empty())
				{
					// 选取“最大周长”的 marker（通常离相机最近/最清晰）
					double bestPerim = -1.0;
					int bestIdx = 0;
					for (int i = 0; i < (int)corners.size(); i++)
					{
						double per = 0.0;
						for (int k = 0; k < 4; k++)
						{
							const cv::Point2f a = corners[i][k];
							const cv::Point2f b = corners[i][(k + 1) & 3];
							const cv::Point2f d = a - b;
							per += std::sqrt((double)d.x * (double)d.x + (double)d.y * (double)d.y);
						}
						if (per > bestPerim)
						{
							bestPerim = per;
							bestIdx = i;
						}
					}

					cv::Point2f c(0, 0);
					for (int k = 0; k < 4; k++) c += corners[bestIdx][k];
					c *= 0.25f;

					u = (double)c.x;
					v = (double)c.y;
					conf = Clamp(bestPerim / (double)std::max(1u, (w + h)), 0.0, 1.0);
					hasTarget = true;
					hasArucoCorners = true;
					for (int k = 0; k < 4; k++)
					{
						arucoCorners[k].x = (double)corners[bestIdx][k].x;
						arucoCorners[k].y = (double)corners[bestIdx][k].y;
					}
					// TrackBox: bounding rect of corners
					{
						double minX = arucoCorners[0].x, minY = arucoCorners[0].y, maxX = arucoCorners[0].x, maxY = arucoCorners[0].y;
						for (int k = 1; k < 4; k++)
						{
							minX = std::min(minX, arucoCorners[k].x);
							minY = std::min(minY, arucoCorners[k].y);
							maxX = std::max(maxX, arucoCorners[k].x);
							maxY = std::max(maxY, arucoCorners[k].y);
						}
						hasTrackBox = true;
						trackBox = clampTrackBox((int)std::floor(minX), (int)std::floor(minY),
						                         (int)std::ceil(maxX - minX), (int)std::ceil(maxY - minY));
					}

#if defined(SMARTARM_HAS_OPENCV_CALIB3D) && SMARTARM_HAS_OPENCV_CALIB3D
					// 估计 marker 位姿，从而得到一个“有尺度”的 depthMm（单位：mm）
					// 说明：这里用粗略内参（fx=w, fy=h, cx=w/2, cy=h/2）保证链路可跑；后续由标定替换。
					const double fx = (double)w;
					const double fy = (double)h;
					const double cx = (double)w * 0.5;
					const double cy = (double)h * 0.5;
					cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
					cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

					const float markerLengthMm = (float)p.arucoMarkerLengthMm;
					std::vector<cv::Vec3d> rvecs, tvecs;
					cv::aruco::estimatePoseSingleMarkers(corners, markerLengthMm, K, dist, rvecs, tvecs);
					if (bestIdx >= 0 && bestIdx < (int)tvecs.size())
					{
						// 先用 marker 原点的 Z 作为粗 depth
						depthMm = tvecs[bestIdx][2];

						// 进一步：用 plane + 像素射线求交，得到目标像素的深度（更贴近 u,v）
						if (bestIdx < (int)rvecs.size())
						{
							cv::Mat Rm;
							cv::Rodrigues(rvecs[bestIdx], Rm); // 3x3
							if (Rm.rows == 3 && Rm.cols == 3)
							{
								double Rarr[9] = {
									Rm.at<double>(0,0), Rm.at<double>(0,1), Rm.at<double>(0,2),
									Rm.at<double>(1,0), Rm.at<double>(1,1), Rm.at<double>(1,2),
									Rm.at<double>(2,0), Rm.at<double>(2,1), Rm.at<double>(2,2),
								};
								double tarr[3] = { tvecs[bestIdx][0], tvecs[bestIdx][1], tvecs[bestIdx][2] };

								VisionGeometry::Plane plane;
								if (VisionGeometry::PlaneFromMarkerPose(Rarr, tarr, plane))
								{
									CameraIntrinsics Ksimple;
									Ksimple.valid = true;
									Ksimple.fx = fx;
									Ksimple.fy = fy;
									Ksimple.cx = cx;
									Ksimple.cy = cy;

									VisionGeometry::Ray ray;
									if (VisionGeometry::PixelToRay(Ksimple, u, v, ray))
									{
										VisionGeometry::Point3 P;
										double tHit = 0.0;
										if (VisionGeometry::IntersectRayPlane(ray, plane, P, tHit))
										{
											depthMm = P.z;
										}
									}
								}
							}
						}

						hasDepth = (depthMm > 1e-3);
					}
#endif
				}
			}
			catch (...)
			{
				// 安全兜底：任何异常都不要影响预览线程；直接回退到其它模式。
				hasTarget = false;
			}

			if (!hasTarget && mode == Mode::Aruco)
			{
				// 强制 ArUco：没识别到就不输出
				::Sleep(30);
				continue;
			}
		}
#endif

		// =================
		// 2) HandSticker (双色贴纸：红=指尖，蓝=指根) -> (u,v) + rayXYZ
		// 3) ColorTrack (HSV 红色 blob) -> (u,v)
		// =================
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
		// HSV 颜色 blob 检测：返回最大连通区域的中心点与面积
		auto findBlob = [&](const cv::Mat& hsv, const cv::Scalar& lo, const cv::Scalar& hi,
		                    double& outU, double& outV, double& outArea, cv::Rect& outRect) -> bool
		{
			cv::Mat mask;
			cv::inRange(hsv, lo, hi, mask);

			const int k = 5;
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
			cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
			cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

			std::vector<std::vector<cv::Point>> contours;
			cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			if (contours.empty()) return false;

			double bestArea = 0.0;
			int bestIdx = -1;
			for (int i = 0; i < (int)contours.size(); i++)
			{
				const double a = cv::contourArea(contours[i]);
				if (a > bestArea)
				{
					bestArea = a;
					bestIdx = i;
				}
			}
			if (bestIdx < 0 || bestArea <= 25.0) return false;

			const cv::Moments mu = cv::moments(contours[bestIdx]);
			if (std::fabs(mu.m00) <= 1e-6) return false;

			outU = mu.m10 / mu.m00;
			outV = mu.m01 / mu.m00;
			outArea = bestArea;
			outRect = cv::boundingRect(contours[bestIdx]);
			return true;
		};

		// HandSticker：双色贴纸指向（强制模式）
		if (!hasTarget && mode == Mode::HandSticker)
		{
			try
			{
				cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
				cv::Mat bgr, hsv;
				cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
				cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

				// 红色（指尖）：双区间
				double ru = 0, rv = 0, ra = 0;
				double ru2 = 0, rv2 = 0, ra2 = 0;
				cv::Rect rbox, rbox2;
				const bool r1 = findBlob(hsv, cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255), ru, rv, ra, rbox);
				const bool r2 = findBlob(hsv, cv::Scalar(160, 100, 80), cv::Scalar(179, 255, 255), ru2, rv2, ra2, rbox2);
				if (r2 && (!r1 || ra2 > ra)) { ru = ru2; rv = rv2; ra = ra2; rbox = rbox2; }
				const bool hasRed = r1 || r2;

				// 蓝色（指根/掌根）
				double bu = 0, bv = 0, ba = 0;
				cv::Rect bboxBlue;
				const bool hasBlue = findBlob(hsv, cv::Scalar(90, 100, 80), cv::Scalar(130, 255, 255), bu, bv, ba, bboxBlue);

				if (hasRed)
				{
					u = ru;
					v = rv;
					conf = Clamp(ra / (double)((double)w * (double)h), 0.0, 1.0);
					hasTarget = true;
					hasTrackBox = true;
					trackBox = clampTrackBox(rbox.x, rbox.y, rbox.width, rbox.height);

					if (hasBlue)
					{
						// 生成一个近似 ray：用屏幕上的指向向量(dx,dy) + 1.0 的前向分量
						double rx = (ru - bu);
						double ry = (rv - bv);
						double rz = 1.0;
						const double n = std::sqrt(rx * rx + ry * ry + rz * rz);
						if (n > 1e-6) { rx /= n; ry /= n; rz /= n; }

						hasRay = true;
						rayX = rx;
						rayY = ry;
						rayZ = rz;
						conf = Clamp(std::min((double)conf, ba / (double)((double)w * (double)h)), 0.0, 1.0);
					}
				}
			}
			catch (...)
			{
				hasTarget = false;
			}

			// 强制 HandSticker：没识别到就不输出；识别到则正常进入发布阶段（用于 HUD 显示与 VS）
			if (!hasTarget)
			{
				::Sleep(30);
				continue;
			}
		}

		// ColorTrack：红色 blob（Auto/强制模式）
		if (!hasTarget && (mode == Mode::ColorTrack || mode == Mode::Auto))
		{
			try
			{
				cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
				cv::Mat bgr, hsv;
				cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
				cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

				double cu = 0, cvv = 0, ca = 0;
				double cu2 = 0, cv2 = 0, ca2 = 0;
				cv::Rect cbox, cbox2;
				const bool rr1 = findBlob(hsv, cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255), cu, cvv, ca, cbox);
				const bool rr2 = findBlob(hsv, cv::Scalar(160, 100, 80), cv::Scalar(179, 255, 255), cu2, cv2, ca2, cbox2);
				if (rr2 && (!rr1 || ca2 > ca)) { cu = cu2; cvv = cv2; ca = ca2; cbox = cbox2; }

				if (rr1 || rr2)
				{
					u = cu;
					v = cvv;
					conf = Clamp(ca / (double)((double)w * (double)h), 0.0, 1.0);
					hasTarget = true;
					hasTrackBox = true;
					trackBox = clampTrackBox(cbox.x, cbox.y, cbox.width, cbox.height);
				}
			}
			catch (...)
			{
				hasTarget = false;
			}

			if (!hasTarget && mode == Mode::ColorTrack)
			{
				::Sleep(30);
				continue;
			}
		}
#endif

		// =================
		// 3) BrightestPoint
		// =================
		if (!hasTarget && (mode == Mode::BrightestPoint || mode == Mode::Auto))
		{
			// 在采样网格上寻找最亮点（证明取帧链路工作）
			// 注意：RGB32 buffer 的字节序为 BGRA（device.cpp 中转换为 D3DCOLOR_XRGB）
			int bestX = (int)(w / 2);
			int bestY = (int)(h / 2);
			int bestVal = -1;

			const int stride = p.sampleStride;
			for (UINT yy = 0; yy < h; yy += (UINT)stride)
			{
				const BYTE* row = rgb.data() + (size_t)yy * (size_t)w * 4;
				for (UINT xx = 0; xx < w; xx += (UINT)stride)
				{
					if (hasExcludeRect)
					{
						if ((LONG)xx >= excludeRect.left && (LONG)xx < excludeRect.right &&
						    (LONG)yy >= excludeRect.top && (LONG)yy < excludeRect.bottom)
						{
							continue;
						}
					}
					const BYTE* px = row + (size_t)xx * 4;
					const int b = (int)px[0];
					const int g = (int)px[1];
					const int r = (int)px[2];
					const int val = r + g + b; // 0..765
					if (val > bestVal)
					{
						bestVal = val;
						bestX = (int)xx;
						bestY = (int)yy;
					}
				}
			}

			u = (double)bestX;
			v = (double)bestY;
			conf = Clamp((double)bestVal / 765.0, 0.0, 1.0);
			hasTarget = true;
			// TrackBox: fixed window around brightest point
			hasTrackBox = true;
			trackBox = clampTrackBox(bestX - 12, bestY - 12, 24, 24);
		}

		// ==============
		// 4) Detector(DNN)
		// ==============
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
		if (!hasTarget && (mode == Mode::Detector))
		{
			// 尝试按需加载（最多每 1s 尝试一次，避免频繁读文件）
			const ULONGLONG now2 = ::GetTickCount64();
			bool loaded = false;
			{
				std::lock_guard<std::mutex> lk(m_detMu);
				loaded = m_detector.IsLoaded();
				if (!loaded && (now2 - m_lastDetLoadAttemptMs > 1000))
				{
					m_lastDetLoadAttemptMs = now2;
					std::wstring err;
					(void)m_detector.EnsureLoaded(err);
					loaded = m_detector.IsLoaded();
				}
			}

			if (loaded)
			{
				try
				{
					cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
					cv::Mat bgr;
					cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

					VisionDetector::Detection det;
					{
						std::lock_guard<std::mutex> lk(m_detMu);
						if (m_detector.DetectBestExcludingRect(bgr.data, (int)bgr.cols, (int)bgr.rows, (int)bgr.step,
						                                      hasExcludeRect ? &excludeRect : nullptr,
						                                      (float)p.excludeHandMaxOverlap,
						                                      det))
						{
							u = (double)det.x + (double)det.w * 0.5;
							v = (double)det.y + (double)det.h * 0.5;
							conf = Clamp((double)det.confidence, 0.0, 1.0);
							hasTarget = true;
							detBox = det;
							hasTrackBox = true;
							trackBox = clampTrackBox(det.x, det.y, det.w, det.h);
						}
					}
					if (hasTarget)
					{
					}
				}
				catch (...)
				{
					hasTarget = false;
				}
			}

			if (!hasTarget)
			{
				::Sleep(30);
				continue;
			}
		}
#endif

		// ======================
		// 5) HandLandmarks (Palm + Handpose ONNX)
		// ======================
		if (!hasTarget && mode == Mode::HandLandmarks)
		{
			const ULONGLONG now3 = ::GetTickCount64();
			bool loaded = false;
			VisionHandLandmarks::Hand hand;
			VisionHandLandmarks::Params hp{};
			{
				std::lock_guard<std::mutex> lk(m_handMu);
				loaded = m_hand.IsLoaded();
				hp = m_hand.GetParams();
				if (!loaded && (now3 - m_lastHandLoadAttemptMs > 1000))
				{
					m_lastHandLoadAttemptMs = now3;
					std::wstring err;
					(void)m_hand.EnsureLoaded(err);
					loaded = m_hand.IsLoaded();
				}
			}

			if (loaded)
			{
				try
				{
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
					cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
					cv::Mat bgr;
					cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
					{
						std::lock_guard<std::mutex> lk(m_handMu);
						if (m_hand.DetectBest(bgr.data, (int)bgr.cols, (int)bgr.rows, (int)bgr.step, hand) && hand.valid)
						{
							// ok
						}
						else
						{
							hand = VisionHandLandmarks::Hand{};
						}
					}
					if (hand.valid)
					{
						// Use index fingertip as target
						const double ix = (double)hand.pts[8 * 2 + 0];
						const double iy = (double)hand.pts[8 * 2 + 1];
						u = ix;
						v = iy;
						conf = Clamp((double)hand.confidence, 0.0, 1.0);
						hasTarget = true;

						// Approximate pointing ray: from MCP(5) to TIP(8)
						const double bx = (double)hand.pts[5 * 2 + 0];
						const double by = (double)hand.pts[5 * 2 + 1];
						double rx = ix - bx;
						double ry = iy - by;
						double rz = 1.0;
						const double n = std::sqrt(rx * rx + ry * ry + rz * rz);
						if (n > 1e-6) { rx /= n; ry /= n; rz /= n; }
						hasRay = true;
						rayX = rx; rayY = ry; rayZ = rz;

						// Gesture classification (rule-based)
						const double x0 = (double)hand.pts[0 * 2 + 0];
						const double y0 = (double)hand.pts[0 * 2 + 1];
						const double handScale = std::max(10.0, std::max(std::fabs((double)hand.x2 - (double)hand.x1), std::fabs((double)hand.y2 - (double)hand.y1)));

						auto dist = [&](int a, int b) -> double
						{
							const double ax = (double)hand.pts[a * 2 + 0];
							const double ay = (double)hand.pts[a * 2 + 1];
							const double bx2 = (double)hand.pts[b * 2 + 0];
							const double by2 = (double)hand.pts[b * 2 + 1];
							const double dx = ax - bx2;
							const double dy = ay - by2;
							return std::sqrt(dx * dx + dy * dy);
						};

						auto distToWrist = [&](int a) -> double
						{
							const double ax = (double)hand.pts[a * 2 + 0];
							const double ay = (double)hand.pts[a * 2 + 1];
							const double dx = ax - x0;
							const double dy = ay - y0;
							return std::sqrt(dx * dx + dy * dy);
						};

						auto isExtended = [&](int tip, int pip) -> bool
						{
							const double dt = distToWrist(tip);
							const double dp2 = distToWrist(pip);
							return dt > dp2 * 1.08; // mild margin
						};

						const bool idxExt = isExtended(8, 6);
						const bool midExt = isExtended(12, 10);
						const bool ringExt = isExtended(16, 14);
						const bool pinkExt = isExtended(20, 18);
						const int extCount = (idxExt ? 1 : 0) + (midExt ? 1 : 0) + (ringExt ? 1 : 0) + (pinkExt ? 1 : 0);

						const double pinchDist = dist(4, 8);
						const double pinchThresh = std::max(5.0, (double)hp.pinchThreshNorm * handScale);
						const double pinchStrength = Clamp(1.0 - pinchDist / pinchThresh, 0.0, 1.0);

						VisionOverlayService::Gesture g = VisionOverlayService::Gesture::Unknown;
						if (pinchStrength > 0.0)
						{
							g = VisionOverlayService::Gesture::Pinch;
						}
						else if (idxExt && !midExt && !ringExt && !pinkExt)
						{
							g = VisionOverlayService::Gesture::Point;
						}
						else if (extCount >= 3)
						{
							g = VisionOverlayService::Gesture::OpenPalm;
						}
						else if (extCount == 0)
						{
							g = VisionOverlayService::Gesture::Fist;
						}

						hasHandLm = true;
						for (int i = 0; i < 21; i++)
						{
							handPts[i].x = (double)hand.pts[i * 2 + 0];
							handPts[i].y = (double)hand.pts[i * 2 + 1];
						}
						handGesture = g;
						handPinchStrength = pinchStrength;
						// TrackBox from landmarks bbox (if provided)
						{
							const int x1 = (int)std::floor((double)hand.x1);
							const int y1 = (int)std::floor((double)hand.y1);
							const int x2 = (int)std::ceil((double)hand.x2);
							const int y2 = (int)std::ceil((double)hand.y2);
							if (x2 > x1 && y2 > y1)
							{
								hasTrackBox = true;
								trackBox = clampTrackBox(x1, y1, x2 - x1, y2 - y1);
							}
						}

						// ==========================
						// Point->Lock->Confirm selection (visual feedback only)
						// ==========================
						{
							const ULONGLONG nowPick = ::GetTickCount64();

							// Cancel flash expiration
							if (pick.state == 4 && nowPick >= pick.cancelUntilMs)
							{
								pick.state = 0;
								pick.hasLastCand = false;
								pick.stableSinceMs = 0;
							}

							// Gesture holds
							updateHold(g == VisionOverlayService::Gesture::Pinch, pick.pinchSinceMs, nowPick);
							updateHold(g == VisionOverlayService::Gesture::OpenPalm, pick.palmSinceMs, nowPick);

							// OpenPalm cancels any state (after hold)
							if (pick.palmSinceMs != 0 && (nowPick - pick.palmSinceMs) >= (ULONGLONG)p.pointPickHoldCancelMs)
							{
								pick.state = 4; // cancelled flash
								pick.cancelUntilMs = nowPick + (ULONGLONG)p.pointPickCancelFlashMs;
								pick.palmSinceMs = 0;
								pick.pinchSinceMs = 0;
								pick.stableSinceMs = 0;
								pick.hasLastCand = false;
							}

							// Pinch confirms only when locked
							if (pick.state == 2 && pick.pinchSinceMs != 0 && (nowPick - pick.pinchSinceMs) >= (ULONGLONG)p.pointPickHoldConfirmMs)
							{
								pick.state = 3; // confirmed
								pick.pinchSinceMs = 0;
							}

							// Only Point drives searching/locking
							const bool isPointing = (g == VisionOverlayService::Gesture::Point);
							if (!isPointing)
							{
								// If not locked/confirmed, clear searching state quickly
								if (pick.state == 1)
								{
									pick.state = 0;
									pick.hasLastCand = false;
									pick.stableSinceMs = 0;
								}
							}

							// When pointing, try to find an object near finger direction (Detector required)
							if (p.pointPickEnabled && isPointing)
							{
								// Throttle detector calls (10Hz)
								if (nowPick - lastPickDetTickMs >= 100)
								{
									lastPickDetTickMs = nowPick;

									// Build exclude rect from hand bbox (inflate)
									RECT ex{};
									bool hasEx = false;
									{
										const int inf = p.excludeHandInflatePx;
										int x1 = (int)std::floor((double)hand.x1) - inf;
										int y1 = (int)std::floor((double)hand.y1) - inf;
										int x2 = (int)std::ceil((double)hand.x2) + inf;
										int y2 = (int)std::ceil((double)hand.y2) + inf;
										x1 = std::max(0, std::min((int)w - 1, x1));
										y1 = std::max(0, std::min((int)h - 1, y1));
										x2 = std::max(0, std::min((int)w, x2));
										y2 = std::max(0, std::min((int)h, y2));
										if (x2 > x1 && y2 > y1)
										{
											ex.left = x1; ex.top = y1; ex.right = x2; ex.bottom = y2;
											hasEx = true;
										}
									}

									// Ensure detector loaded occasionally
									bool detLoaded = false;
									{
										std::lock_guard<std::mutex> lk(m_detMu);
										detLoaded = m_detector.IsLoaded();
										if (!detLoaded && (nowPick - m_lastDetLoadAttemptMs > 1000))
										{
											m_lastDetLoadAttemptMs = nowPick;
											std::wstring err;
											(void)m_detector.EnsureLoaded(err);
											detLoaded = m_detector.IsLoaded();
										}
									}

									// Pick best near pointing ray
									const double sx = ix;
									const double sy = iy;
									double dirx = rayX;
									double diry = rayY;
									const double dn = std::sqrt(dirx * dirx + diry * diry);
									if (dn > 1e-6) { dirx /= dn; diry /= dn; }
									else { dirx = 1.0; diry = 0.0; }

									const int maxLen = p.pointPickMaxRayLenPx;
									const int maxPerp = p.pointPickMaxRayPerpPx;
									const int maxRad = p.pointPickMaxRadiusPx;

									// Convert once
									cv::Mat bgra2((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
									cv::Mat bgr2;
									cv::cvtColor(bgra2, bgr2, cv::COLOR_BGRA2BGR);

									bool hasCand = false;
									VisionOverlayService::RectI cand{};
									double bestScore = -1e18;

									// 1) Preferred: Detector candidates (if model is configured)
									if (detLoaded)
									{
										std::vector<VisionDetector::Detection> dets;
										{
											std::lock_guard<std::mutex> lk(m_detMu);
											(void)m_detector.DetectAll(bgr2.data, (int)bgr2.cols, (int)bgr2.rows, (int)bgr2.step, dets);
										}

										for (const auto& d : dets)
										{
											VisionOverlayService::RectI r = clampTrackBox(d.x, d.y, d.w, d.h);
											if (r.w <= 0 || r.h <= 0) continue;

											// exclude hand overlap
											if (hasEx && p.excludeHand)
											{
												const int rx2 = r.x + r.w;
												const int ry2 = r.y + r.h;
												const int ix1 = std::max(r.x, (int)ex.left);
												const int iy1 = std::max(r.y, (int)ex.top);
												const int ix2 = std::min(rx2, (int)ex.right);
												const int iy2 = std::min(ry2, (int)ex.bottom);
												const int iw2 = std::max(0, ix2 - ix1);
												const int ih2 = std::max(0, iy2 - iy1);
												const double inter = (double)iw2 * (double)ih2;
												const double area = (double)r.w * (double)r.h;
												const double overlap = (area > 1e-6) ? (inter / area) : 0.0;
												if (overlap > p.excludeHandMaxOverlap) continue;
											}

											const double cx = (double)r.x + (double)r.w * 0.5;
											const double cy2 = (double)r.y + (double)r.h * 0.5;
											const double dx = cx - sx;
											const double dy = cy2 - sy;
											const double t = dx * dirx + dy * diry;
											const double perp = std::fabs(dx * diry - dy * dirx);
											const double rad = std::sqrt(dx * dx + dy * dy);

											const bool okRay = (t >= 0.0 && t <= (double)maxLen && perp <= (double)maxPerp);
											const bool okRad = (rad <= (double)maxRad);
											if (!okRay && !okRad) continue;

											// score: prefer high confidence, then small perp, then small distance
											const double cconf = Clamp((double)d.confidence, 0.0, 1.0);
											const double s1 = cconf;
											const double s2 = -perp / (double)std::max(1, maxPerp);
											const double s3 = -(okRay ? (t / (double)std::max(1, maxLen)) : (rad / (double)std::max(1, maxRad)));
											const double score = s1 * 1.0 + s2 * 0.35 + s3 * 0.25;
											if (score > bestScore)
											{
												bestScore = score;
												cand = r;
												hasCand = true;
											}
										}
									}

									// 2) Fallback (no model / detector can't detect): edge/contour proposals near ray
									if (!hasCand)
									{
										// Build ROI mask: thick ray tube + fingertip radius, minus hand rect
										cv::Mat mask = cv::Mat::zeros((int)h, (int)w, CV_8UC1);
										const cv::Point p0((int)std::lround(sx), (int)std::lround(sy));
										const cv::Point p1((int)std::lround(sx + dirx * (double)maxLen), (int)std::lround(sy + diry * (double)maxLen));
										const int thick = std::max(6, maxPerp * 2);
										cv::line(mask, p0, p1, cv::Scalar(255), thick, cv::LINE_AA);
										cv::circle(mask, p0, maxRad, cv::Scalar(255), -1, cv::LINE_AA);
										if (hasEx)
										{
											const cv::Rect exr((int)ex.left, (int)ex.top, (int)(ex.right - ex.left), (int)(ex.bottom - ex.top));
											cv::rectangle(mask, exr, cv::Scalar(0), -1);
										}

										cv::Mat gray;
										cv::cvtColor(bgr2, gray, cv::COLOR_BGR2GRAY);
										cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);
										cv::Mat edges;
										cv::Canny(gray, edges, 60, 140);
										// limit to ROI
										edges &= mask;
										// connect edges a bit
										cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
										cv::dilate(edges, edges, k);

										std::vector<std::vector<cv::Point>> contours;
										cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
										for (const auto& cts : contours)
										{
											if (cts.size() < 10) continue;
											const cv::Rect rr = cv::boundingRect(cts);
											const int area = rr.area();
											if (area < 300) continue;
											if (rr.width >= (int)w || rr.height >= (int)h) continue;

											VisionOverlayService::RectI r = clampTrackBox(rr.x, rr.y, rr.width, rr.height);
											if (r.w <= 0 || r.h <= 0) continue;

											const double cx = (double)r.x + (double)r.w * 0.5;
											const double cy2 = (double)r.y + (double)r.h * 0.5;
											const double dx = cx - sx;
											const double dy = cy2 - sy;
											const double t = dx * dirx + dy * diry;
											const double perp = std::fabs(dx * diry - dy * dirx);
											const double rad = std::sqrt(dx * dx + dy * dy);
											const bool okRay = (t >= 0.0 && t <= (double)maxLen && perp <= (double)maxPerp);
											const bool okRad = (rad <= (double)maxRad);
											if (!okRay && !okRad) continue;

											// prefer larger contours but still close to ray
											const double aNorm = Clamp((double)area / (double)std::max(1, maxPerp * maxLen), 0.0, 1.0);
											const double s2 = -perp / (double)std::max(1, maxPerp);
											const double s3 = -(okRay ? (t / (double)std::max(1, maxLen)) : (rad / (double)std::max(1, maxRad)));
											const double score = aNorm * 0.8 + s2 * 0.45 + s3 * 0.25;
											if (score > bestScore)
											{
												bestScore = score;
												cand = r;
												hasCand = true;
											}
										}
									}

									// FSM update
									if (hasCand)
									{
										pick.missingSinceMs = 0;
										if (pick.state == 0 || pick.state == 4)
										{
											pick.state = 1; // searching
											pick.box = cand;
											pick.lastCand = cand;
											pick.hasLastCand = true;
											pick.stableSinceMs = nowPick;
										}
										else if (pick.state == 1)
										{
											double iou = pick.hasLastCand ? rectIou(pick.lastCand, cand) : 0.0;
											if (iou >= p.pointPickIouSame)
											{
												// stable
												if (pick.stableSinceMs != 0 && (nowPick - pick.stableSinceMs) >= (ULONGLONG)p.pointPickHoldLockMs)
												{
													pick.state = 2; // locked
													pick.box = cand;
												}
												else
												{
													pick.box = cand;
												}
											}
											else
											{
												// new candidate: reset timer
												pick.stableSinceMs = nowPick;
												pick.box = cand;
												pick.lastCand = cand;
												pick.hasLastCand = true;
											}
										}
										else if (pick.state == 2 || pick.state == 3)
										{
											// locked/confirmed: keep box updated while pointing
											pick.box = cand;
										}
									}
									else
									{
										// no candidate found: don't instantly reset; give it a short grace period
										if (pick.state == 1)
										{
											if (pick.missingSinceMs == 0) pick.missingSinceMs = nowPick;
											if ((nowPick - pick.missingSinceMs) > 600)
											{
												pick.state = 0;
												pick.hasLastCand = false;
												pick.stableSinceMs = 0;
												pick.missingSinceMs = 0;
											}
										}
									}
								}
							}
						}
					}
#endif
				}
				catch (...)
				{
					hasTarget = false;
				}
			}

			if (!hasTarget)
			{
				::Sleep(30);
				continue;
			}
		}

		// EMA 平滑，减少抖动
		if (p.emaAlpha > 0.0 && p.emaAlpha < 1.0)
		{
			if (m_hasLastUv)
			{
				u = Lerp(m_lastU, u, p.emaAlpha);
				v = Lerp(m_lastV, v, p.emaAlpha);
			}
			m_lastU = u;
			m_lastV = v;
			m_hasLastUv = true;
		}

		VisualObservation obs;
		obs.tickMs = ::GetTickCount64();
		obs.hasTargetPx = hasTarget;
		obs.u = u;
		obs.v = v;
		obs.hasDepthMm = hasDepth;
		obs.depthMm = depthMm;
		obs.hasConfidence = true;
		obs.confidence = conf;
		obs.hasRay = hasRay;
		obs.rayX = rayX;
		obs.rayY = rayY;
		obs.rayZ = rayZ;

		vs->UpdateObservation(obs);

		// Publish last result for HUD (thread-safe)
		{
			Result r;
			r.tickMs = obs.tickMs;
			r.mode = (int)mode;
			r.hasTargetPx = hasTarget;
			r.u = u;
			r.v = v;
			r.hasDepthMm = hasDepth;
			r.depthMm = depthMm;
			r.hasConfidence = true;
			r.confidence = conf;

			// Detector bbox
			if (mode == Mode::Detector && hasTarget)
			{
				r.hasBox = true;
				r.boxX = detBox.x;
				r.boxY = detBox.y;
				r.boxW = detBox.w;
				r.boxH = detBox.h;
				r.classId = detBox.classId;
			}

			{
				std::lock_guard<std::mutex> lk(m_resMu);
				m_lastResult = r;
			}

			// Update overlay snapshot (convert to overlay format)
			VisionOverlayService::Snapshot s;
			s.tickMs = (unsigned long long)r.tickMs;
			s.mode = r.mode;
			s.hasTargetPx = r.hasTargetPx;
			s.u = r.u;
			s.v = r.v;
			s.hasConfidence = r.hasConfidence;
			s.confidence = r.confidence;
			s.hasTrackBox = hasTrackBox;
			if (hasTrackBox)
			{
				s.trackBox = trackBox;
			}
			s.hasRay = hasRay;
			s.rayX = rayX;
			s.rayY = rayY;
			s.rayZ = rayZ;
			s.hasDepthMm = r.hasDepthMm;
			s.depthMm = r.depthMm;
			s.depthNearMm = p.depthNearMm;
			s.depthFarMm = p.depthFarMm;
			s.hasBox = r.hasBox;
			s.box.x = r.boxX;
			s.box.y = r.boxY;
			s.box.w = r.boxW;
			s.box.h = r.boxH;
			s.classId = r.classId;
			s.hasArucoCorners = hasArucoCorners;
			if (hasArucoCorners)
			{
				s.arucoCorners = arucoCorners;
			}
			s.hasHandLandmarks = hasHandLm;
			if (hasHandLm)
			{
				s.handPts = handPts;
				s.gesture = handGesture;
				s.pinchStrength = handPinchStrength;
			}
			// PointPick selection feedback
			s.selectState = pick.state;
			s.hasSelectBox = (pick.state != 0);
			if (s.hasSelectBox)
			{
				s.selectBox = pick.box;
			}
			VisionOverlayService::Instance().Update(s);
		}

		// Stats
		frames++;
		const ULONGLONG now = ::GetTickCount64();
		if (now - fpsTick >= 1000)
		{
			const double sec = std::max(0.001, (double)(now - fpsTick) / 1000.0);
			const double fps = (double)frames / sec;
			frames = 0;
			fpsTick = now;

			std::lock_guard<std::mutex> lk(m_statsMu);
			m_stats.hasFrame = true;
			m_stats.frameW = w;
			m_stats.frameH = h;
			m_stats.procFps = fps;
			m_stats.lastProcTickMs = now;
		}

		// pacing
		const ULONGLONG used = now - lastTick;
		lastTick = now;
		const int sleepMs = std::max(0, p.processPeriodMs - (int)used);
		if (sleepMs > 0)
		{
			::Sleep((DWORD)sleepMs);
		}
	}
}


