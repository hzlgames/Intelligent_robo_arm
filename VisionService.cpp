#include "pch.h"

#include "VisionService.h"

#include "preview.h"
#include "VisualServoController.h"

#include "VisionDetector.h"
#include "VisionGeometry.h"

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

void VisionService::SetParams(const Params& p)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_params = p;
	m_params.processPeriodMs = std::max(5, m_params.processPeriodMs);
	m_params.sampleStride = std::max(1, m_params.sampleStride);
	m_params.emaAlpha = Clamp(m_params.emaAlpha, 0.0, 1.0);
	m_params.arucoMarkerLengthMm = std::max(1.0, m_params.arucoMarkerLengthMm);
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
		// 2) Hand (双色贴纸：红=指尖，蓝=指根) -> (u,v) + rayXYZ
		// 3) ColorTrack (HSV 红色 blob) -> (u,v)
		// =================
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
		// HSV 颜色 blob 检测：返回最大连通区域的中心点与面积
		auto findBlob = [&](const cv::Mat& hsv, const cv::Scalar& lo, const cv::Scalar& hi,
		                    double& outU, double& outV, double& outArea) -> bool
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
			return true;
		};

		// Hand：双色贴纸指向（强制模式）
		if (!hasTarget && mode == Mode::Hand)
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
				const bool r1 = findBlob(hsv, cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255), ru, rv, ra);
				const bool r2 = findBlob(hsv, cv::Scalar(160, 100, 80), cv::Scalar(179, 255, 255), ru2, rv2, ra2);
				if (r2 && (!r1 || ra2 > ra)) { ru = ru2; rv = rv2; ra = ra2; }
				const bool hasRed = r1 || r2;

				// 蓝色（指根/掌根）
				double bu = 0, bv = 0, ba = 0;
				const bool hasBlue = findBlob(hsv, cv::Scalar(90, 100, 80), cv::Scalar(130, 255, 255), bu, bv, ba);

				if (hasRed)
				{
					u = ru;
					v = rv;
					conf = Clamp(ra / (double)((double)w * (double)h), 0.0, 1.0);
					hasTarget = true;

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

			::Sleep(30);
			continue;
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
				const bool rr1 = findBlob(hsv, cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255), cu, cvv, ca);
				const bool rr2 = findBlob(hsv, cv::Scalar(160, 100, 80), cv::Scalar(179, 255, 255), cu2, cv2, ca2);
				if (rr2 && (!rr1 || ca2 > ca)) { cu = cu2; cvv = cv2; ca = ca2; }

				if (rr1 || rr2)
				{
					u = cu;
					v = cvv;
					conf = Clamp(ca / (double)((double)w * (double)h), 0.0, 1.0);
					hasTarget = true;
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
						if (m_detector.DetectBest(bgr.data, (int)bgr.cols, (int)bgr.rows, (int)bgr.step, det))
						{
							u = (double)det.x + (double)det.w * 0.5;
							v = (double)det.y + (double)det.h * 0.5;
							conf = Clamp((double)det.confidence, 0.0, 1.0);
							hasTarget = true;
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


