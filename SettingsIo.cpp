#include "pch.h"

#include "SettingsIo.h"

#include "MotionConfig.h"

#include <windows.h>
#include <string>

namespace
{
	// INI schema version (bump if keys/sections change).
	constexpr int kIniVersion = 1;

	bool WriteStringW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, const std::wstring& value)
	{
		return ::WritePrivateProfileStringW(section, key, value.c_str(), iniPath.c_str()) != FALSE;
	}

	bool WriteIntW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int value)
	{
		return WriteStringW(iniPath, section, key, std::to_wstring(value));
	}

	int ReadIntW(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int fallback)
	{
		return static_cast<int>(::GetPrivateProfileIntW(section, key, static_cast<UINT>(fallback), iniPath.c_str()));
	}

	void ExportProfileInt(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int defaultValue)
	{
		const int v = AfxGetApp()->GetProfileInt(section, key, defaultValue);
		(void)WriteIntW(iniPath, section, key, v);
	}

	void ImportProfileInt(const std::wstring& iniPath, const wchar_t* section, const wchar_t* key, int defaultValue)
	{
		// Fallback to current profile (if exists), otherwise provided default.
		const int current = AfxGetApp()->GetProfileInt(section, key, defaultValue);
		const int v = ReadIntW(iniPath, section, key, current);
		AfxGetApp()->WriteProfileInt(section, key, v);
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

	r.ok = true;
	return r;
}
