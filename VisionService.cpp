#include "pch.h"

#include "VisionService.h"

#include "preview.h"

#include "VisionDetector.h"
#include "VisionGeometry.h"
#include "VisionOverlayService.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cwctype>
#include <string>
#include <vector>

#include <winhttp.h>
#pragma comment(lib, "winhttp.lib")

// Optional OpenCV support (vcpkg opencv4 provides core/imgproc; aruco depends on contrib build).
#if defined(__has_include)
#if __has_include(<opencv2/core.hpp>) && __has_include(<opencv2/imgproc.hpp>)
#define SMARTARM_HAS_OPENCV 1
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/imgcodecs.hpp>)
#include <opencv2/imgcodecs.hpp>
#endif
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

	static std::wstring TrimW(const std::wstring& s)
	{
		size_t start = 0;
		while (start < s.size() && std::iswspace(s[start])) start++;
		size_t end = s.size();
		while (end > start && std::iswspace(s[end - 1])) end--;
		return s.substr(start, end - start);
	}

	static std::string WStringToUtf8(const std::wstring& ws)
	{
		if (ws.empty()) return std::string();
		const int n = ::WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), (int)ws.size(), nullptr, 0, nullptr, nullptr);
		if (n <= 0) return std::string();
		std::string out;
		out.resize((size_t)n);
		::WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), (int)ws.size(), &out[0], n, nullptr, nullptr);
		return out;
	}

	static std::string TrimStr(const std::string& s)
	{
		size_t b = 0;
		while (b < s.size() && std::isspace((unsigned char)s[b])) b++;
		size_t e = s.size();
		while (e > b && std::isspace((unsigned char)s[e - 1])) e--;
		return s.substr(b, e - b);
	}

	static std::string StripMarkdownFence(const std::string& s)
	{
		std::string t = TrimStr(s);
		if (t.rfind("```", 0) == 0)
		{
			const size_t nl = t.find('\n');
			if (nl != std::string::npos) t = t.substr(nl + 1);
			t = TrimStr(t);
			if (t.size() >= 3 && t.compare(t.size() - 3, 3, "```") == 0)
			{
				t = t.substr(0, t.size() - 3);
			}
			t = TrimStr(t);
		}
		return t;
	}

	static bool ExtractFirstTextFromResponse(const std::string& resp, std::string& outText)
	{
		outText.clear();
		size_t pos = resp.find("\"text\"");
		if (pos == std::string::npos) return false;
		size_t colon = resp.find(':', pos);
		if (colon == std::string::npos) return false;
		size_t q = resp.find('"', colon);
		if (q == std::string::npos) return false;
		q++; // after opening quote
		std::string out;
		out.reserve(256);
		bool esc = false;
		for (size_t i = q; i < resp.size(); ++i)
		{
			char c = resp[i];
			if (esc)
			{
				switch (c)
				{
				case 'n': out.push_back('\n'); break;
				case 'r': out.push_back('\r'); break;
				case 't': out.push_back('\t'); break;
				case '\\': out.push_back('\\'); break;
				case '"': out.push_back('"'); break;
				case '/': out.push_back('/'); break;
				default: out.push_back(c); break;
				}
				esc = false;
				continue;
			}
			if (c == '\\')
			{
				esc = true;
				continue;
			}
			if (c == '"')
			{
				outText = out;
				return true;
			}
			out.push_back(c);
		}
		return false;
	}

	static std::wstring Utf8ToWString(const std::string& s)
	{
		if (s.empty()) return std::wstring();
		const int n = ::MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0);
		if (n <= 0) return std::wstring();
		std::wstring out;
		out.resize((size_t)n);
		::MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), &out[0], n);
		return out;
	}

	static std::wstring TruncateW(const std::wstring& s, size_t maxLen)
	{
		if (s.size() <= maxLen) return s;
		return s.substr(0, maxLen) + L"...";
	}

	static std::string Base64Encode(const unsigned char* data, size_t len)
	{
		static const char* kTable = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
		std::string out;
		out.reserve(((len + 2) / 3) * 4);
		size_t i = 0;
		while (i + 2 < len)
		{
			const unsigned v = (unsigned)data[i] << 16 | (unsigned)data[i + 1] << 8 | (unsigned)data[i + 2];
			out.push_back(kTable[(v >> 18) & 0x3F]);
			out.push_back(kTable[(v >> 12) & 0x3F]);
			out.push_back(kTable[(v >> 6) & 0x3F]);
			out.push_back(kTable[v & 0x3F]);
			i += 3;
		}
		if (i < len)
		{
			unsigned v = (unsigned)data[i] << 16;
			if (i + 1 < len) v |= (unsigned)data[i + 1] << 8;
			out.push_back(kTable[(v >> 18) & 0x3F]);
			out.push_back(kTable[(v >> 12) & 0x3F]);
			if (i + 1 < len) out.push_back(kTable[(v >> 6) & 0x3F]);
			else out.push_back('=');
			out.push_back('=');
		}
		return out;
	}

// WinHTTP 代理字符串规范化：
// 1) 支持输入 "http://127.0.0.1:7890" / "https://127.0.0.1:7890"
// 2) 支持输入 "127.0.0.1:7890"
// 3) 若已经是 WinHTTP 格式（含 "http=" 或 "https="）则原样返回
static std::wstring NormalizeWinHttpProxy(const std::wstring& proxy)
{
	std::wstring s = TrimW(proxy);
	if (s.empty()) return s;
	if (s.find(L"http=") != std::wstring::npos || s.find(L"https=") != std::wstring::npos)
	{
		return s;
	}
	const std::wstring httpPrefix = L"http://";
	const std::wstring httpsPrefix = L"https://";
	if (s.rfind(httpPrefix, 0) == 0)
	{
		s = s.substr(httpPrefix.size());
	}
	else if (s.rfind(httpsPrefix, 0) == 0)
	{
		s = s.substr(httpsPrefix.size());
	}
	// WinHTTP 建议格式："http=host:port;https=host:port"
	return L"http=" + s + L";https=" + s;
}

	static bool HttpPostJson(const std::wstring& host, const std::wstring& pathWithQuery,
	                         const std::string& body, std::string& outResponse, std::wstring& outErr,
	                         const std::wstring& proxy = L"")
	{
		outResponse.clear();
		outErr.clear();

	const std::wstring proxyNorm = NormalizeWinHttpProxy(proxy);
	DWORD accessType = WINHTTP_ACCESS_TYPE_DEFAULT_PROXY;
	const wchar_t* proxyName = WINHTTP_NO_PROXY_NAME;
	if (!proxyNorm.empty())
		{
			accessType = WINHTTP_ACCESS_TYPE_NAMED_PROXY;
		proxyName = proxyNorm.c_str();
		}

		HINTERNET hSession = WinHttpOpen(L"SmartArm/1.0",
		                                accessType,
		                                proxyName, WINHTTP_NO_PROXY_BYPASS, 0);
		if (!hSession)
		{
			outErr = L"WinHttpOpen failed, err=" + std::to_wstring(::GetLastError());
			return false;
		}
		WinHttpSetTimeouts(hSession, 10000, 10000, 30000, 30000);

		HINTERNET hConnect = WinHttpConnect(hSession, host.c_str(), INTERNET_DEFAULT_HTTPS_PORT, 0);
		if (!hConnect)
		{
			DWORD err = ::GetLastError();
			WinHttpCloseHandle(hSession);
			outErr = L"WinHttpConnect failed, err=" + std::to_wstring(err);
			return false;
		}

		HINTERNET hRequest = WinHttpOpenRequest(hConnect, L"POST", pathWithQuery.c_str(),
		                                        nullptr, WINHTTP_NO_REFERER,
		                                        WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
		if (!hRequest)
		{
			DWORD err = ::GetLastError();
			WinHttpCloseHandle(hConnect);
			WinHttpCloseHandle(hSession);
			outErr = L"WinHttpOpenRequest failed, err=" + std::to_wstring(err);
			return false;
		}

		const wchar_t* hdr = L"Content-Type: application/json\r\n";
		WinHttpAddRequestHeaders(hRequest, hdr, -1L, WINHTTP_ADDREQ_FLAG_ADD);

		BOOL ok = WinHttpSendRequest(hRequest, WINHTTP_NO_ADDITIONAL_HEADERS, 0,
		                             (LPVOID)body.data(), (DWORD)body.size(), (DWORD)body.size(), 0);
		if (!ok)
		{
			DWORD err = ::GetLastError();
			WinHttpCloseHandle(hRequest);
			WinHttpCloseHandle(hConnect);
			WinHttpCloseHandle(hSession);
			outErr = L"WinHTTP SendRequest failed, err=" + std::to_wstring(err);
			return false;
		}
		if (!WinHttpReceiveResponse(hRequest, nullptr))
		{
			DWORD err = ::GetLastError();
			WinHttpCloseHandle(hRequest);
			WinHttpCloseHandle(hConnect);
			WinHttpCloseHandle(hSession);
			outErr = L"WinHTTP ReceiveResponse failed, err=" + std::to_wstring(err);
			return false;
		}

		// Check HTTP status code
		DWORD statusCode = 0;
		DWORD dwSize = sizeof(statusCode);
		WinHttpQueryHeaders(hRequest, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		                    WINHTTP_HEADER_NAME_BY_INDEX, &statusCode, &dwSize, WINHTTP_NO_HEADER_INDEX);

		std::string resp;
		DWORD avail = 0;
		while (WinHttpQueryDataAvailable(hRequest, &avail) && avail > 0)
		{
			std::string buf;
			buf.resize(avail);
			DWORD read = 0;
			if (!WinHttpReadData(hRequest, &buf[0], avail, &read) || read == 0)
			{
				break;
			}
			resp.append(buf.data(), buf.data() + read);
		}

		WinHttpCloseHandle(hRequest);
		WinHttpCloseHandle(hConnect);
		WinHttpCloseHandle(hSession);

		if (statusCode != 200)
		{
			outErr = L"HTTP " + std::to_wstring(statusCode);
			if (!resp.empty())
			{
				std::string sub = resp.size() > 200 ? resp.substr(0, 200) + "..." : resp;
				std::wstring wsub(sub.begin(), sub.end()); 
				outErr += L" Body: " + wsub;
			}
			outResponse = resp; 
			return false;
		}

		outResponse = resp;
		return true;
	}

	static bool ParseGeminiBox(const std::string& resp, int imgW, int imgH,
	                           VisionOverlayService::RectI& outBox, double& outScore)
	{
		outBox = VisionOverlayService::RectI{};
		outScore = 0.0;

		auto findToken = [&](const char* token) -> size_t
		{
			size_t p = resp.find(token);
			return p;
		};

		size_t pos = findToken("box_2d");
		if (pos == std::string::npos) pos = findToken("\"box\"");
		if (pos == std::string::npos) pos = findToken("box");
		if (pos == std::string::npos) return false;

		std::vector<double> nums;
		nums.reserve(4);
		const char* s = resp.c_str();
		const char* end = s + resp.size();
		for (const char* p = s + pos; p < end && nums.size() < 4; )
		{
			if (std::isdigit((unsigned char)*p) || *p == '-' || *p == '.')
			{
				char* pEnd = nullptr;
				const double v = std::strtod(p, &pEnd);
				if (pEnd != p)
				{
					nums.push_back(v);
					p = pEnd;
					continue;
				}
			}
			p++;
		}
		if (nums.size() < 4) return false;

		// Optional score
		{
			size_t sp = resp.find("score", pos);
			if (sp != std::string::npos)
			{
				for (const char* p = s + sp; p < end; )
				{
					if (std::isdigit((unsigned char)*p) || *p == '-' || *p == '.')
					{
						char* pEnd = nullptr;
						const double v = std::strtod(p, &pEnd);
						if (pEnd != p) { outScore = v; break; }
					}
					p++;
				}
			}
		}

		double ymin = nums[0], xmin = nums[1], ymax = nums[2], xmax = nums[3];
		const double maxv = std::max(std::max(ymin, xmin), std::max(ymax, xmax));

		double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		if (maxv <= 1.5)
		{
			x1 = xmin * imgW; x2 = xmax * imgW;
			y1 = ymin * imgH; y2 = ymax * imgH;
		}
		else if (maxv <= 1000.0)
		{
			x1 = xmin / 1000.0 * imgW; x2 = xmax / 1000.0 * imgW;
			y1 = ymin / 1000.0 * imgH; y2 = ymax / 1000.0 * imgH;
		}
		else
		{
			x1 = xmin; x2 = xmax;
			y1 = ymin; y2 = ymax;
		}

		const int ix1 = std::max(0, std::min(imgW - 1, (int)std::floor(x1)));
		const int iy1 = std::max(0, std::min(imgH - 1, (int)std::floor(y1)));
		const int ix2 = std::max(0, std::min(imgW, (int)std::ceil(x2)));
		const int iy2 = std::max(0, std::min(imgH, (int)std::ceil(y2)));

		const int w = std::max(0, ix2 - ix1);
		const int h = std::max(0, iy2 - iy1);
		if (w <= 0 || h <= 0) return false;

		outBox.x = ix1;
		outBox.y = iy1;
		outBox.w = w;
		outBox.h = h;
		return true;
	}

	static bool ParseGeminiTimeToFetch(const std::string& resp, int& outVal)
	{
		outVal = -1;
		const char* tokens[] = { "TimeToFetch", "time_to_fetch", "timeToFetch" };
		size_t pos = std::string::npos;
		for (const char* t : tokens)
		{
			pos = resp.find(t);
			if (pos != std::string::npos) break;
		}
		if (pos == std::string::npos) return false;
		const char* s = resp.c_str();
		const char* end = s + resp.size();
		for (const char* p = s + pos; p < end; ++p)
		{
			if (*p == '0' || *p == '1')
			{
				outVal = (*p == '1') ? 1 : 0;
				return true;
			}
		}
		return false;
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
	m_params.pointPickTarget = std::max(0, std::min(1, m_params.pointPickTarget));
	// pointPickDetectorOnly / pointPickResetSeq 为逻辑开关/运行期字段，无需 clamp
	m_params.pointPickMaxRayLenPx = std::max(10, m_params.pointPickMaxRayLenPx);
	m_params.pointPickMaxRayPerpPx = std::max(5, m_params.pointPickMaxRayPerpPx);
	m_params.pointPickMaxRadiusPx = std::max(5, m_params.pointPickMaxRadiusPx);
	m_params.pointPickHoldLockMs = std::max(100, m_params.pointPickHoldLockMs);
	m_params.pointPickHoldConfirmMs = std::max(100, m_params.pointPickHoldConfirmMs);
	m_params.pointPickHoldCancelMs = std::max(100, m_params.pointPickHoldCancelMs);
	m_params.pointPickCancelFlashMs = std::max(0, m_params.pointPickCancelFlashMs);
	m_params.pointPickIouSame = Clamp(m_params.pointPickIouSame, 0.0, 1.0);

	m_params.geminiApiKey = TrimW(m_params.geminiApiKey);
	m_params.geminiProxy = TrimW(m_params.geminiProxy);
	if (m_params.geminiModel.empty()) m_params.geminiModel = L"gemini-3-flash-preview";
	m_params.geminiRequestIntervalMs = std::max(500, std::min(60000, m_params.geminiRequestIntervalMs));
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
		// 连续锁定计数：用于“3 次锁定事件后才启动指尖方向识别”
		int lockCount = 0;
		ULONGLONG lockHoldSinceMs = 0;
		bool pointReady = false;
	};
	PickFsm pick{};
	ULONGLONG lastPickDetTickMs = 0;
	int lastPickResetSeq = 0;

	// Gemini cache (thread-local)
	VisionOverlayService::RectI geminiBox{};
	bool geminiHasTarget = false;
	double geminiConf = 0.0;
	ULONGLONG lastGeminiReqTickMs = 0;
	std::wstring lastGeminiNote;
	int lastGeminiResetSeq = 0;
	bool geminiHasTimeToFetch = false;
	int geminiTimeToFetch = -1;

	while (m_running.load())
	{
		Params p;
		CPreview* preview = nullptr;
		{
			std::lock_guard<std::mutex> lk(m_mu);
			p = m_params;
			preview = m_preview;
		}

		// 运行期重置：用于“锁定抓取物 -> 再锁定终点红点”等多段流程
		if (p.pointPickResetSeq != lastPickResetSeq)
		{
			lastPickResetSeq = p.pointPickResetSeq;
			pick = PickFsm{};
			lastPickDetTickMs = 0;
		}
	if (p.geminiResetSeq != lastGeminiResetSeq)
	{
		lastGeminiResetSeq = p.geminiResetSeq;
		geminiBox = VisionOverlayService::RectI{};
		geminiHasTarget = false;
		geminiConf = 0.0;
		lastGeminiReqTickMs = 0;
		lastGeminiNote = L"Gemini: reset";
		geminiHasTimeToFetch = false;
		geminiTimeToFetch = -1;
	}

		if (!m_enabled.load() || preview == nullptr)
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

		// ======================
		// 3.5) Gemini (cloud object detection)
		// ======================
#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
		if (!hasTarget && mode == Mode::Gemini)
		{
			const ULONGLONG nowG = ::GetTickCount64();
			const int minInterval = std::max(500, p.geminiRequestIntervalMs);
			const ULONGLONG sinceLast = (nowG >= lastGeminiReqTickMs) ? (nowG - lastGeminiReqTickMs) : 0;
			const bool canRequest = (sinceLast >= (ULONGLONG)minInterval);

			if (canRequest)
			{
				lastGeminiReqTickMs = nowG;
				geminiHasTarget = false;
				geminiConf = 0.0;
				lastGeminiNote.clear();
				geminiHasTimeToFetch = false;
				geminiTimeToFetch = -1;

				if (!p.geminiApiKey.empty())
				{
					try
					{
						cv::Mat bgra((int)h, (int)w, CV_8UC4, rgb.data(), (size_t)w * 4);
						cv::Mat bgr;
						cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

						// Light downscale to reduce payload size
						const int maxSide = std::max(bgr.cols, bgr.rows);
						cv::Mat bgrSend = bgr;
						if (maxSide > 640)
						{
							const double scale = 640.0 / (double)maxSide;
							const int nw = std::max(1, (int)std::lround(bgr.cols * scale));
							const int nh = std::max(1, (int)std::lround(bgr.rows * scale));
							cv::resize(bgr, bgrSend, cv::Size(nw, nh), 0, 0, cv::INTER_AREA);
						}

						std::vector<uchar> jpg;
						std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 85 };
						if (cv::imencode(".jpg", bgrSend, jpg, params) && !jpg.empty())
						{
							const std::string b64 = Base64Encode(jpg.data(), jpg.size());
							std::string prompt = "Detect prominent objects in the image.";
							std::string schema =
								"{\"type\":\"object\",\"properties\":{"
								"\"objects\":{\"type\":\"array\",\"items\":{\"type\":\"object\",\"properties\":{"
								"\"label\":{\"type\":\"string\"},"
								"\"score\":{\"type\":\"number\"},"
								"\"box\":{\"type\":\"array\",\"items\":{\"type\":\"number\"},\"minItems\":4,\"maxItems\":4}"
								"},\"required\":[\"box\"]}}"
								"},\"required\":[\"objects\"]}";
							if (p.geminiEnableTimeToFetch)
							{
								prompt =
									"Detect prominent objects in the image, and decide if the gripper is close enough to fetch now. "
									"Return TimeToFetch=1 when it is safe to grasp, else 0.";
								schema =
									"{\"type\":\"object\",\"properties\":{"
									"\"TimeToFetch\":{\"type\":\"integer\"},"
									"\"objects\":{\"type\":\"array\",\"items\":{\"type\":\"object\",\"properties\":{"
									"\"label\":{\"type\":\"string\"},"
									"\"score\":{\"type\":\"number\"},"
									"\"box\":{\"type\":\"array\",\"items\":{\"type\":\"number\"},\"minItems\":4,\"maxItems\":4}"
									"},\"required\":[\"box\"]}}"
									"},\"required\":[\"TimeToFetch\",\"objects\"]}";
							}

							const std::string body =
								std::string("{\"contents\":[{\"role\":\"user\",\"parts\":[{\"text\":\"") +
								prompt + "\"},{\"inline_data\":{\"mime_type\":\"image/jpeg\",\"data\":\"" +
								b64 + "\"}}]}],\"generationConfig\":{\"response_mime_type\":\"application/json\","
								"\"response_json_schema\":" + schema + ",\"temperature\":1.0}}";

							std::wstring path = L"/v1beta/models/" + p.geminiModel +
								L":generateContent?key=" + p.geminiApiKey;
							lastGeminiNote = L"Gemini: POST /v1beta/models/" + p.geminiModel;

							std::string resp;
							std::wstring err;
							// Pass proxy if configured (read from profile, but we need to add it to Params first.
							// For now, let's just use the updated function signature.
							if (HttpPostJson(L"generativelanguage.googleapis.com", path, body, resp, err, p.geminiProxy))
							{
								VisionOverlayService::RectI box{};
								double score = 0.0;
								std::string textPayload;
								bool parsed = false;
								if (ExtractFirstTextFromResponse(resp, textPayload))
								{
									const std::string cleaned = StripMarkdownFence(textPayload);
									parsed = ParseGeminiBox(cleaned, (int)w, (int)h, box, score);
								}
								if (!parsed)
								{
									parsed = ParseGeminiBox(resp, (int)w, (int)h, box, score);
								}
								int ttf = -1;
								bool hasTtf = false;
								if (ExtractFirstTextFromResponse(resp, textPayload))
								{
									const std::string cleaned = StripMarkdownFence(textPayload);
									if (ParseGeminiTimeToFetch(cleaned, ttf)) hasTtf = true;
								}
								if (!hasTtf)
								{
									if (ParseGeminiTimeToFetch(resp, ttf)) hasTtf = true;
								}
								if (parsed)
								{
									geminiHasTarget = true;
									geminiBox = box;
									geminiConf = Clamp(score, 0.0, 1.0);
									lastGeminiNote = L"Gemini: OK (parsed)";
								}
								else
								{
									lastGeminiNote = L"Gemini: response parsed failed, len=" + std::to_wstring(resp.size());
									const std::wstring wresp = TruncateW(Utf8ToWString(resp), 160);
									if (!wresp.empty()) lastGeminiNote += L" " + wresp;
								}
								if (hasTtf)
								{
									geminiHasTimeToFetch = true;
									geminiTimeToFetch = ttf;
								}
							}
							else
							{
								lastGeminiNote = L"Gemini: HTTP failed " + err;
							}
						}
						else
						{
							lastGeminiNote = L"Gemini: JPEG encode failed";
						}
					}
					catch (...)
					{
						geminiHasTarget = false;
						lastGeminiNote = L"Gemini: exception during request";
					}
				}
				else
				{
					lastGeminiNote = L"Gemini: missing API key";
				}
			}
			else
			{
				const ULONGLONG waitMs = (sinceLast < (ULONGLONG)minInterval) ? ((ULONGLONG)minInterval - sinceLast) : 0;
				lastGeminiNote = L"Gemini: waiting " + std::to_wstring((unsigned long long)waitMs) + L"ms";
			}

			if (geminiHasTarget)
			{
				u = (double)geminiBox.x + (double)geminiBox.w * 0.5;
				v = (double)geminiBox.y + (double)geminiBox.h * 0.5;
				conf = Clamp(geminiConf, 0.0, 1.0);
				hasTarget = true;
				hasTrackBox = true;
				trackBox = clampTrackBox(geminiBox.x, geminiBox.y, geminiBox.w, geminiBox.h);
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
								// Point 中断：清空锁定计数
								pick.lockCount = 0;
								pick.lockHoldSinceMs = 0;
								pick.pointReady = false;
							}
							else
							{
								// 连续锁定事件计数：达到 3 次才启用指尖方向识别
								if (pick.lockHoldSinceMs == 0) pick.lockHoldSinceMs = nowPick;
								const int lockMs = std::max(100, p.pointPickHoldLockMs);
								const ULONGLONG elapsed = (nowPick >= pick.lockHoldSinceMs) ? (nowPick - pick.lockHoldSinceMs) : 0;
								if (elapsed >= (ULONGLONG)lockMs)
								{
									pick.lockCount++;
									if (pick.lockCount > 3) pick.lockCount = 3;
									pick.lockHoldSinceMs = nowPick;
								}
								pick.pointReady = (pick.lockCount >= 3);
							}

							// When pointing, try to find a target near finger direction
							if (p.pointPickEnabled && isPointing)
							{
								// 只要是 Point 手势，state 就至少是 1（搜索中），不需要等待节流
								// 这样确保画面显示 Point 时，pickState 立即变为 1
								if (pick.state == 0 || pick.state == 4)
								{
									pick.state = 1; // searching (pointing but not locked)
									pick.stableSinceMs = nowPick;
								}

								// Throttle PointPick calls (10Hz)
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

									// Target = Gemini 云端识别候选（指尖方向）
									if (p.pointPickTarget == 0)
									{
										// 只有“连续三次锁定时间”且仍为 Point，才开始发起 Gemini 识别
										if (pick.pointReady)
										{
											const ULONGLONG nowG = nowPick;
											const int minInterval = std::max(500, p.geminiRequestIntervalMs);
											const ULONGLONG sinceLast = (nowG >= lastGeminiReqTickMs) ? (nowG - lastGeminiReqTickMs) : 0;
											const bool canRequest = (sinceLast >= (ULONGLONG)minInterval);

											if (canRequest)
											{
												lastGeminiReqTickMs = nowG;
												geminiHasTarget = false;
												geminiConf = 0.0;
												lastGeminiNote.clear();

												if (!p.geminiApiKey.empty())
												{
													try
													{
														// 轻量缩放，减少请求体积
														const int maxSide = std::max(bgr2.cols, bgr2.rows);
														cv::Mat bgrSend = bgr2;
														if (maxSide > 640)
														{
															const double scale = 640.0 / (double)maxSide;
															const int nw = std::max(1, (int)std::lround(bgr2.cols * scale));
															const int nh = std::max(1, (int)std::lround(bgr2.rows * scale));
															cv::resize(bgr2, bgrSend, cv::Size(nw, nh), 0, 0, cv::INTER_AREA);
														}

														std::vector<uchar> jpg;
														std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 85 };
														if (cv::imencode(".jpg", bgrSend, jpg, params) && !jpg.empty())
														{
															const std::string b64 = Base64Encode(jpg.data(), jpg.size());
															const std::string prompt =
																"识别指尖指向附近的物体坐标，并返回最相关物体的 box 坐标。"
																"Detect the object near the fingertip direction and return its box coordinates.";

															const std::string schema =
																"{\"type\":\"object\",\"properties\":{"
																"\"objects\":{\"type\":\"array\",\"items\":{\"type\":\"object\",\"properties\":{"
																"\"label\":{\"type\":\"string\"},"
																"\"score\":{\"type\":\"number\"},"
																"\"box\":{\"type\":\"array\",\"items\":{\"type\":\"number\"},\"minItems\":4,\"maxItems\":4}"
																"},\"required\":[\"box\"]}}"
																"},\"required\":[\"objects\"]}";

															const std::string body =
																std::string("{\"contents\":[{\"role\":\"user\",\"parts\":[{\"text\":\"") +
																prompt + "\"},{\"inline_data\":{\"mime_type\":\"image/jpeg\",\"data\":\"" +
																b64 + "\"}}]}],\"generationConfig\":{\"response_mime_type\":\"application/json\","
																"\"response_json_schema\":" + schema + ",\"temperature\":1.0}}";

															std::wstring path = L"/v1beta/models/" + p.geminiModel +
																L":generateContent?key=" + p.geminiApiKey;
															lastGeminiNote = L"Gemini(PointPick): POST /v1beta/models/" + p.geminiModel;

															std::string resp;
															std::wstring err;
															if (HttpPostJson(L"generativelanguage.googleapis.com", path, body, resp, err, p.geminiProxy))
															{
																VisionOverlayService::RectI box{};
																double score = 0.0;
																std::string textPayload;
																bool parsed = false;
																if (ExtractFirstTextFromResponse(resp, textPayload))
																{
																	const std::string cleaned = StripMarkdownFence(textPayload);
																	parsed = ParseGeminiBox(cleaned, (int)w, (int)h, box, score);
																}
																if (!parsed)
																{
																	parsed = ParseGeminiBox(resp, (int)w, (int)h, box, score);
																}
																if (parsed)
																{
																	geminiHasTarget = true;
																	geminiBox = box;
																	geminiConf = Clamp(score, 0.0, 1.0);
																	lastGeminiNote = L"Gemini(PointPick): OK (parsed)";
																}
																else
																{
																	lastGeminiNote = L"Gemini(PointPick): response parsed failed, len=" + std::to_wstring(resp.size());
																}
															}
															else
															{
																lastGeminiNote = L"Gemini(PointPick): HTTP failed " + err;
															}
														}
														else
														{
															lastGeminiNote = L"Gemini(PointPick): JPEG encode failed";
														}
													}
													catch (...)
													{
														geminiHasTarget = false;
														lastGeminiNote = L"Gemini(PointPick): exception during request";
													}
												}
												else
												{
													lastGeminiNote = L"Gemini(PointPick): missing API key";
												}
											}

											if (geminiHasTarget)
											{
												VisionOverlayService::RectI r = clampTrackBox(geminiBox.x, geminiBox.y, geminiBox.w, geminiBox.h);
												if (r.w > 0 && r.h > 0)
												{
													bool accept = true;

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
														if (overlap > p.excludeHandMaxOverlap) accept = false;
													}

													if (accept)
													{
														const double cx = (double)r.x + (double)r.w * 0.5;
														const double cy2 = (double)r.y + (double)r.h * 0.5;
														const double dx = cx - sx;
														const double dy = cy2 - sy;
														const double t = dx * dirx + dy * diry;
														const double perp = std::fabs(dx * diry - dy * dirx);
														const double rad = std::sqrt(dx * dx + dy * dy);

														const bool okRay = (t >= 0.0 && t <= (double)maxLen && perp <= (double)maxPerp);
														const bool okRad = (rad <= (double)maxRad);
														if (okRay || okRad)
														{
															// score: prefer high confidence, then small perp, then small distance
															const double cconf = Clamp(geminiConf, 0.0, 1.0);
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
												}
											}
										}

										// 使用 Gemini 云端识别；若未就绪或无有效候选，则 hasCand 保持 false。
									}
									else
									{
										// Target = tabletop red dot: pick by red blob near pointing ray
										try
										{
											cv::Mat hsv;
											cv::cvtColor(bgr2, hsv, cv::COLOR_BGR2HSV);

											cv::Mat m1, m2, mask;
											cv::inRange(hsv, cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255), m1);
											cv::inRange(hsv, cv::Scalar(160, 100, 80), cv::Scalar(179, 255, 255), m2);
											mask = m1 | m2;

											const int ksz = 5;
											cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksz, ksz));
											cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
											cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

											std::vector<std::vector<cv::Point>> contours;
											cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
											if (!contours.empty())
											{
												double bestArea = 0.0;
												int bestIdx = -1;
												for (int i = 0; i < (int)contours.size(); i++)
												{
													const double a = cv::contourArea(contours[i]);
													if (a > bestArea) { bestArea = a; bestIdx = i; }
												}
												if (bestIdx >= 0 && bestArea > 25.0)
												{
													const cv::Rect rr = cv::boundingRect(contours[bestIdx]);
													VisionOverlayService::RectI r = clampTrackBox(rr.x, rr.y, rr.width, rr.height);
													if (r.w > 0 && r.h > 0)
													{
														const double cx = (double)r.x + (double)r.w * 0.5;
														const double cy2 = (double)r.y + (double)r.h * 0.5;
														const double dx = cx - sx;
														const double dy = cy2 - sy;
														const double t = dx * dirx + dy * diry;
														const double perp = std::fabs(dx * diry - dy * dirx);
														const double rad = std::sqrt(dx * dx + dy * dy);
														const bool okRay = (t >= 0.0 && t <= (double)maxLen && perp <= (double)maxPerp);
														const bool okRad = (rad <= (double)maxRad);
														if (okRay || okRad)
														{
															const double aNorm = Clamp(bestArea / (double)std::max(1, maxPerp * maxLen), 0.0, 1.0);
															const double s2 = -perp / (double)std::max(1, maxPerp);
															const double s3 = -(okRay ? (t / (double)std::max(1, maxLen)) : (rad / (double)std::max(1, maxRad)));
															const double score = aNorm * 0.8 + s2 * 0.45 + s3 * 0.25;
															bestScore = score;
															cand = r;
															hasCand = true;
														}
													}
												}
											}
										}
										catch (...)
										{
											hasCand = false;
										}
									}

									// FSM update
									if (hasCand)
									{
										pick.missingSinceMs = 0;
										if (pick.state == 1 && !pick.hasLastCand)
										{
											// 首次找到候选
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

			// Track box (for size-based range estimation etc.)
			if (hasTrackBox)
			{
				r.hasBox = true;
				r.boxX = trackBox.x;
				r.boxY = trackBox.y;
				r.boxW = trackBox.w;
				r.boxH = trackBox.h;
			}
			// Detector class id (only meaningful in Detector mode)
			if (mode == Mode::Detector && hasTarget)
			{
				r.classId = detBox.classId;
			}

			// PointPick selection (for See&Fetch: using hand gestures to lock target / red dot)
			r.pickState = pick.state;
			r.hasPickBox = (pick.state != 0 && pick.box.w > 0 && pick.box.h > 0);
			if (r.hasPickBox)
			{
				r.pickBoxX = pick.box.x;
				r.pickBoxY = pick.box.y;
				r.pickBoxW = pick.box.w;
				r.pickBoxH = pick.box.h;
			}

			// HandLandmarks gesture (for upper-level pause logic)
			r.hasHandLandmarks = hasHandLm;
			r.handGesture = (int)handGesture;
			r.hasGeminiTimeToFetch = geminiHasTimeToFetch;
			r.geminiTimeToFetch = geminiTimeToFetch;

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
			if (mode == Mode::Gemini && !lastGeminiNote.empty())
			{
				s.note = lastGeminiNote;
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


