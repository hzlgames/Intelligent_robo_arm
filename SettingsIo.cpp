#include "pch.h"

#include "SettingsIo.h"

#include "MotionConfig.h"

#include <windows.h>
#include <string>

namespace
{
	// INI schema version (bump if keys/sections change).
	constexpr int kIniVersion = 2;

	bool WriteStringW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, const std::wstring& value)
	{
		return ::WritePrivateProfileStringW(section, key, value.c_str(), iniPath.c_str()) != FALSE;
	}

	bool WriteIntW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int value)
	{
		return WriteStringW(iniPath, section, key, std::to_wstring(value));
	}

	bool WriteStringKeyW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, const std::wstring& value)
	{
		return WriteStringW(iniPath, section, key, value);
	}

	int ReadIntW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int fallback)
	{
		return static_cast<int>(::GetPrivateProfileIntW(section, key, static_cast<UINT>(fallback), iniPath.c_str()));
	}

	std::wstring ReadStringW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, const std::wstring& fallback)
	{
		wchar_t buf[2048] = {};
		::GetPrivateProfileStringW(section, key, fallback.c_str(), buf, ARRAYSIZE(buf), iniPath.c_str());
		return std::wstring(buf);
	}

	void ExportProfileInt(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int defaultValue)
	{
		const int v = AfxGetApp()->GetProfileInt(section, key, defaultValue);
		(void)WriteIntW(iniPath, section, key, v);
	}

	void ExportProfileString(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, const wchar_t* defaultValue)
	{
		const CString v = AfxGetApp()->GetProfileString(section, key, defaultValue);
		(void)WriteStringKeyW(iniPath, section, key, std::wstring(v.GetString()));
	}

	void ImportProfileInt(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int defaultValue)
	{
		// Fallback to current profile (if exists), otherwise provided default.
		const int current = AfxGetApp()->GetProfileInt(section, key, defaultValue);
		const int v = ReadIntW(iniPath, section, key, current);
		AfxGetApp()->WriteProfileInt(section, key, v);
	}

	void ImportProfileString(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, const wchar_t* defaultValue)
	{
		const CString current = AfxGetApp()->GetProfileString(section, key, defaultValue);
		const std::wstring v = ReadStringW(iniPath, section, key, std::wstring(current.GetString()));
		AfxGetApp()->WriteProfileString(section, key, v.c_str());
	}

	// Keep servo positions within a sane range to avoid accidental unsafe values.
	int ClampServoPos(int v)
	{
		if (v < 0) return 0;
		if (v > 1000) return 1000;
		return v;
	}
}

SettingsIo::Result SettingsIo::ExportToIni(const std::wstring& iniPath)
{
	Result r;
	if (iniPath.empty())
	{
		r.ok = false;
		r.error = L"INI path is empty.";
		return r;
	}

	// Ensure we can create/truncate the file (fail fast on permission/path issues).
	{
		HANDLE h = ::CreateFileW(iniPath.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (h == INVALID_HANDLE_VALUE)
		{
			r.ok = false;
			r.error = L"Cannot create file (check path/permissions).";
			return r;
		}
		::CloseHandle(h);
	}

	// Meta
	if (!WriteIntW(iniPath, L"Meta", L"Version", kIniVersion))
	{
		r.ok = false;
		r.error = L"Failed to write settings file.";
		return r;
	}

	// Throttle
	ExportProfileInt(iniPath, L"Throttle", L"Ms", 50);

	// Serial manual move panel
	ExportProfileInt(iniPath, L"ManualMove", L"Id", 1);
	ExportProfileInt(iniPath, L"ManualMove", L"Pos", 500);
	ExportProfileInt(iniPath, L"ManualMove", L"Step", 20);
	ExportProfileInt(iniPath, L"ManualMove", L"Time", 800);

	// Servo limits (legacy section used by Serial page; also imported by MotionConfig)
	for (int id = 1; id <= 6; id++)
	{
		CString kMin, kMax;
		kMin.Format(L"Min%d", id);
		kMax.Format(L"Max%d", id);
		const int minV = AfxGetApp()->GetProfileInt(L"ServoLimits", kMin, 0);
		const int maxV = AfxGetApp()->GetProfileInt(L"ServoLimits", kMax, 1000);
		(void)WriteIntW(iniPath, L"ServoLimits", kMin, ClampServoPos(minV));
		(void)WriteIntW(iniPath, L"ServoLimits", kMax, ClampServoPos(maxV));
	}

	// Camera overlay settings
	ExportProfileInt(iniPath, L"CameraOverlay", L"Mirror", 0);
	ExportProfileInt(iniPath, L"CameraOverlay", L"Crosshair", 0);
	ExportProfileInt(iniPath, L"CameraOverlay", L"Grid", 0);
	ExportProfileInt(iniPath, L"CameraOverlay", L"Rotation", 0);

	// Motion joints (J1..J6)
	for (int j = 1; j <= MotionConfig::kJointCount; j++)
	{
		CString sec;
		sec.Format(L"Motion\\J%d", j);
		ExportProfileInt(iniPath, sec, L"ServoId", 0);
		ExportProfileInt(iniPath, sec, L"Min", 0);
		ExportProfileInt(iniPath, sec, L"Max", 1000);
		ExportProfileInt(iniPath, sec, L"Home", 500);
		ExportProfileInt(iniPath, sec, L"Invert", 0);
	}

	// ===== Vision (Visual compute) =====
	// Mode: 0=Auto, 1=BrightestPoint, 2=Aruco, 3=ColorTrack, 4=Detector, 5=Hand
	ExportProfileInt(iniPath, L"Vision", L"Mode", 0);
	// AlgoEnabled: 0=手动(点击，不跑识别), 1=启用视觉识别（与 Mode 搭配）
	ExportProfileInt(iniPath, L"Vision", L"AlgoEnabled", 1);
	ExportProfileInt(iniPath, L"Vision", L"ProcessPeriodMs", 33);
	ExportProfileInt(iniPath, L"Vision", L"SampleStride", 8);
	ExportProfileInt(iniPath, L"Vision", L"EmaAlpha_milli", 350); // 0..1000

	// ArUco
	ExportProfileInt(iniPath, L"Vision\\Aruco", L"MarkerLengthMm", 40);

	// Detector (ONNX)
	ExportProfileString(iniPath, L"Vision\\Detector", L"OnnxPath", L"");
	ExportProfileInt(iniPath, L"Vision\\Detector", L"InputW", 320);
	ExportProfileInt(iniPath, L"Vision\\Detector", L"InputH", 320);
	ExportProfileInt(iniPath, L"Vision\\Detector", L"Conf_milli", 500); // 0..1000
	ExportProfileInt(iniPath, L"Vision\\Detector", L"Nms_milli", 400);  // 0..1000

	r.ok = true;
	return r;
}

SettingsIo::Result SettingsIo::ImportFromIni(const std::wstring& iniPath)
{
	Result r;
	if (iniPath.empty())
	{
		r.ok = false;
		r.error = L"INI path is empty.";
		return r;
	}
	if (::GetFileAttributesW(iniPath.c_str()) == INVALID_FILE_ATTRIBUTES)
	{
		r.ok = false;
		r.error = L"Settings file not found.";
		return r;
	}

	const int ver = ReadIntW(iniPath, L"Meta", L"Version", kIniVersion);
	(void)ver; // For now we accept v1+; future versions can branch here.

	// Throttle
	ImportProfileInt(iniPath, L"Throttle", L"Ms", 50);

	// ManualMove
	ImportProfileInt(iniPath, L"ManualMove", L"Id", 1);
	ImportProfileInt(iniPath, L"ManualMove", L"Pos", 500);
	ImportProfileInt(iniPath, L"ManualMove", L"Step", 20);
	ImportProfileInt(iniPath, L"ManualMove", L"Time", 800);

	// ServoLimits
	for (int id = 1; id <= 6; id++)
	{
		CString kMin, kMax;
		kMin.Format(L"Min%d", id);
		kMax.Format(L"Max%d", id);
		const int curMin = AfxGetApp()->GetProfileInt(L"ServoLimits", kMin, 0);
		const int curMax = AfxGetApp()->GetProfileInt(L"ServoLimits", kMax, 1000);
		const int minV = ClampServoPos(ReadIntW(iniPath, L"ServoLimits", kMin, curMin));
		const int maxV = ClampServoPos(ReadIntW(iniPath, L"ServoLimits", kMax, curMax));
		AfxGetApp()->WriteProfileInt(L"ServoLimits", kMin, minV);
		AfxGetApp()->WriteProfileInt(L"ServoLimits", kMax, maxV);
	}

	// CameraOverlay
	ImportProfileInt(iniPath, L"CameraOverlay", L"Mirror", 0);
	ImportProfileInt(iniPath, L"CameraOverlay", L"Crosshair", 0);
	ImportProfileInt(iniPath, L"CameraOverlay", L"Grid", 0);
	ImportProfileInt(iniPath, L"CameraOverlay", L"Rotation", 0);

	// Motion joints
	for (int j = 1; j <= MotionConfig::kJointCount; j++)
	{
		CString sec;
		sec.Format(L"Motion\\J%d", j);

		const int servoId = ReadIntW(iniPath, sec, L"ServoId", AfxGetApp()->GetProfileInt(sec, L"ServoId", 0));
		const int minV = ClampServoPos(ReadIntW(iniPath, sec, L"Min", AfxGetApp()->GetProfileInt(sec, L"Min", 0)));
		const int maxV = ClampServoPos(ReadIntW(iniPath, sec, L"Max", AfxGetApp()->GetProfileInt(sec, L"Max", 1000)));
		const int homeV = ClampServoPos(ReadIntW(iniPath, sec, L"Home", AfxGetApp()->GetProfileInt(sec, L"Home", 500)));
		const int invert = ReadIntW(iniPath, sec, L"Invert", AfxGetApp()->GetProfileInt(sec, L"Invert", 0)) ? 1 : 0;

		AfxGetApp()->WriteProfileInt(sec, L"ServoId", servoId);
		AfxGetApp()->WriteProfileInt(sec, L"Min", minV);
		AfxGetApp()->WriteProfileInt(sec, L"Max", maxV);
		AfxGetApp()->WriteProfileInt(sec, L"Home", homeV);
		AfxGetApp()->WriteProfileInt(sec, L"Invert", invert);
	}

	// ===== Vision =====
	ImportProfileInt(iniPath, L"Vision", L"Mode", 0);
	ImportProfileInt(iniPath, L"Vision", L"AlgoEnabled", 1);
	ImportProfileInt(iniPath, L"Vision", L"ProcessPeriodMs", 33);
	ImportProfileInt(iniPath, L"Vision", L"SampleStride", 8);
	ImportProfileInt(iniPath, L"Vision", L"EmaAlpha_milli", 350);

	ImportProfileInt(iniPath, L"Vision\\Aruco", L"MarkerLengthMm", 40);

	ImportProfileString(iniPath, L"Vision\\Detector", L"OnnxPath", L"");
	ImportProfileInt(iniPath, L"Vision\\Detector", L"InputW", 320);
	ImportProfileInt(iniPath, L"Vision\\Detector", L"InputH", 320);
	ImportProfileInt(iniPath, L"Vision\\Detector", L"Conf_milli", 500);
	ImportProfileInt(iniPath, L"Vision\\Detector", L"Nms_milli", 400);

	r.ok = true;
	return r;
}
