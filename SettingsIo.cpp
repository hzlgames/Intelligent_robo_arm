#include "pch.h"

#include "SettingsIo.h"

#include "MotionConfig.h"
#include "KinematicsConfig.h"

#include <windows.h>
#include <string>

namespace
{
	// INI schema version (bump if keys/sections change).
	constexpr int kIniVersion = 4;

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

	// AutoHome (连接后自动归位的初始角度)
	ExportProfileInt(iniPath, L"AutoHome", L"J1Deg", 0);
	ExportProfileInt(iniPath, L"AutoHome", L"J2Deg", -30);
	ExportProfileInt(iniPath, L"AutoHome", L"J3Deg", 60);
	ExportProfileInt(iniPath, L"AutoHome", L"J4Deg", 30);
	ExportProfileInt(iniPath, L"AutoHome", L"J5Deg", 0);

	// ===== Kinematics (Arm model calibration) =====
	// Links (mm)
	ExportProfileInt(iniPath, L"Kinematics\\Links", L"L_base_mm", 80);
	ExportProfileInt(iniPath, L"Kinematics\\Links", L"L_arm1_mm", 100);
	ExportProfileInt(iniPath, L"Kinematics\\Links", L"L_arm2_mm", 95);
	ExportProfileInt(iniPath, L"Kinematics\\Links", L"L_wrist_mm", 95);
	ExportProfileInt(iniPath, L"Kinematics\\Links", L"L_cam_mm", 55);

	// Safety (FPS constraints)
	ExportProfileInt(iniPath, L"Kinematics\\Safety", L"PitchMinDeg", -90);
	ExportProfileInt(iniPath, L"Kinematics\\Safety", L"PitchMaxDeg", 90);
	ExportProfileInt(iniPath, L"Kinematics\\Safety", L"ZMinMm", 20);
	ExportProfileInt(iniPath, L"Kinematics\\Safety", L"RMinMm", 30);
	ExportProfileInt(iniPath, L"Kinematics\\Safety", L"RMaxMm", 290);

	// Joint calib (J1..J5): two-point calibration + zero offset (milli-degree)
	// Defaults align with current Reference/mechanics.md sample values (can be overridden by profile/ini).
	const int defPosAt0Deg[6] = { 0, 500, 500, 500, 500, 500 };
	const int defPosAtPlusDeg[6] = { 0, 690, 320, 680, 700, 900 };
	const int defPlusDeg[6] = { 0, 45, 45, 45, 45, 90 };
	for (int j = 1; j <= 5; j++)
	{
		CString sec;
		sec.Format(L"Kinematics\\J%d", j);
		ExportProfileInt(iniPath, sec, L"PosAt0Deg", defPosAt0Deg[j]);
		ExportProfileInt(iniPath, sec, L"PosAtPlusDeg", defPosAtPlusDeg[j]);
		ExportProfileInt(iniPath, sec, L"PlusDeg", defPlusDeg[j]);
		ExportProfileInt(iniPath, sec, L"ZeroOffset_mdeg", 0);
		// Physical angle inversion (0/1)
		ExportProfileInt(iniPath, sec, L"PhysicalInvert", 0);
	}

	// ===== Tool (camera/gripper offsets) =====
	// 坐标：Cam（X右Y下Z前）
	ExportProfileInt(iniPath, L"Tool\\Offsets", L"Joint5ToCam_X_mm", 0);
	ExportProfileInt(iniPath, L"Tool\\Offsets", L"Joint5ToCam_Y_mm", -55);
	ExportProfileInt(iniPath, L"Tool\\Offsets", L"Joint5ToCam_Z_mm", 0);
	ExportProfileInt(iniPath, L"Tool\\Offsets", L"CamToGrip_X_mm", 0);
	ExportProfileInt(iniPath, L"Tool\\Offsets", L"CamToGrip_Y_mm", 0);
	ExportProfileInt(iniPath, L"Tool\\Offsets", L"CamToGrip_Z_mm", 40);

	// ===== Vision (Visual compute) =====
	// Mode: 0=Auto, 1=BrightestPoint, 2=Aruco, 3=ColorTrack, 4=Detector, 5=HandSticker, 6=HandLandmarks, 7=Gemini
	ExportProfileInt(iniPath, L"Vision", L"Mode", 0);
	// AlgoEnabled: 0=手动(点击，不跑识别), 1=启用视觉识别（与 Mode 搭配）
	ExportProfileInt(iniPath, L"Vision", L"AlgoEnabled", 1);
	// ProcEnabled: 1=允许视觉线程产出识别结果（与 VS Enable 解耦）
	ExportProfileInt(iniPath, L"Vision", L"ProcEnabled", 1);
	// NoDrive: 1=仅测试（默认不允许视觉输出驱动运动）
	ExportProfileInt(iniPath, L"Vision", L"NoDrive", 1);
	ExportProfileInt(iniPath, L"Vision", L"ProcessPeriodMs", 33);
	ExportProfileInt(iniPath, L"Vision", L"SampleStride", 8);
	ExportProfileInt(iniPath, L"Vision", L"EmaAlpha_milli", 350); // 0..1000
	// ExcludeHand: 1=目标选择时排除手部（用于“指哪抓哪”）
	ExportProfileInt(iniPath, L"Vision", L"ExcludeHand", 1);
	ExportProfileInt(iniPath, L"Vision", L"ExcludeHandInflatePx", 20);
	ExportProfileInt(iniPath, L"Vision", L"ExcludeHandOverlap_milli", 300); // 0..1000
	// PointPick (gesture-based pick flow)
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"Enabled", 1);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"MaxRayLenPx", 320);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"MaxRayPerpPx", 90);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"MaxRadiusPx", 140);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"HoldLockMs", 3000);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"HoldConfirmMs", 3000);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"HoldCancelMs", 3000);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"CancelFlashMs", 800);
	ExportProfileInt(iniPath, L"Vision\\PointPick", L"IouSame_milli", 500); // 0..1000

	// ArUco
	ExportProfileInt(iniPath, L"Vision\\Aruco", L"MarkerLengthMm", 40);

	// Depth binning (mm)
	ExportProfileInt(iniPath, L"Vision\\Depth", L"NearMm", 120);
	ExportProfileInt(iniPath, L"Vision\\Depth", L"FarMm", 220);

	// Detector (ONNX)
	// 说明：Detector 若不配置 OnnxPath 会无法启用；这里给出“推荐默认”路径，用户只需把模型文件放到该位置即可。
	// 推荐：yolov5n.onnx（输入 640x640）
	ExportProfileString(iniPath, L"Vision\\Detector", L"OnnxPath", L"models\\detector\\yolov5n.onnx");
	ExportProfileInt(iniPath, L"Vision\\Detector", L"InputW", 640);
	ExportProfileInt(iniPath, L"Vision\\Detector", L"InputH", 640);
	ExportProfileInt(iniPath, L"Vision\\Detector", L"Conf_milli", 500); // 0..1000
	ExportProfileInt(iniPath, L"Vision\\Detector", L"Nms_milli", 400);  // 0..1000

	// Gemini (cloud)
	ExportProfileString(iniPath, L"Vision\\Gemini", L"ApiKey", L"");
	ExportProfileString(iniPath, L"Vision\\Gemini", L"Model", L"gemini-3-flash-preview");
	ExportProfileInt(iniPath, L"Vision\\Gemini", L"IntervalMs", 2000);
	ExportProfileString(iniPath, L"Vision\\Gemini", L"Proxy", L"");

	// Hand (Palm + Landmarks ONNX)
	ExportProfileString(iniPath, L"Vision\\Hand", L"PalmOnnxPath", L"");
	ExportProfileString(iniPath, L"Vision\\Hand", L"LandmarkOnnxPath", L"");
	ExportProfileInt(iniPath, L"Vision\\Hand", L"PinchThreshNorm_milli", 250); // 0..1000

	// ===== SeeAndFetch (auto vision tracking/grasp/place state machine) =====
	// Global
	ExportProfileInt(iniPath, L"SeeAndFetch", L"PreferArucoDuringAuto", 1);
	ExportProfileInt(iniPath, L"SeeAndFetch", L"LostFramesToAbort", 10);
	ExportProfileInt(iniPath, L"SeeAndFetch", L"AcquireStableFrames", 5);
	ExportProfileInt(iniPath, L"SeeAndFetch", L"EnablePlaneCache", 1);
	// Timing
	ExportProfileInt(iniPath, L"SeeAndFetch\\Timing", L"MinCommandIntervalMs", 120);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Timing", L"DefaultMoveTimeMs", 220);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Timing", L"LockAfterMoveMs", 240);
	// Find
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"DeadbandPx", 10);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"StableCenterFrames", 3);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Yaw_kDegPerPx_milli", 30);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Yaw_MinStepDeg_milli", 600);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Yaw_MaxStepDeg_milli", 3500);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Pitch_kDegPerPx_milli", 30);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Pitch_MinStepDeg_milli", 600);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Pitch_MaxStepDeg_milli", 3500);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"J4PreferAbsDeg_milli", 35000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"SignJ1FromErrU", +1);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"SignJ4FromErrV", +1);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Find", L"SignJ3FromErrV", +1);
	// Approach
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"GraspDepthMm", 160);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"DepthStableFrames", 3);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"DepthMaxJumpMm", 40);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"RangeMode", 0); // 0=ArucoDepth,1=BboxArea,2=Auto
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"GraspBoxAreaPx2", 30000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"GraspBoxScale_milli", 0);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"BoxStableFrames", 3);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"BoxAreaMaxJumpPx2", 20000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"BboxRequireDetector", 1);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"MaxAdvanceSteps", 60);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"MaxAttempts", 3);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"RetryRetreatSteps", 8);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"J2AdvanceStepDeg_milli", 2000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"SignJ2Advance", +1);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"EnableJ1FineTune", 1);
	// Gripper
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"JointIndex", 6);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"OpenPos", 650);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"ClosePos", 350);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"CloseStepPos", 25);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"CloseMoveTimeMs", 450);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"MaxCloseSteps", 12);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"EnableStallDetect", 0);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"StallDetectDeltaPos", 10);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"StallDetectMaxAgeMs", 800);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"MaxAttempts", 2);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"AdvanceStepsOnFail", 1);
	// Place / Return
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"Mode", 0); // 0=SimpleOpen,1=RedDotVisual
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"VisionMode", 3); // default ColorTrack
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"CenterStableFrames", 3);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"RangeMode", 1); // 0=ArucoDepth,1=BboxArea,2=Auto
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"PlaceDepthMm", 180);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"PlaceBoxAreaPx2", 24000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"PlaceBoxScale_milli", 0);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"BoxStableFrames", 3);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"BoxAreaMaxJumpPx2", 20000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"MaxDownSteps", 30);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"J2DownStepDeg_milli", 2000);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"SignJ2Down", +1);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"MaxAttempts", 2);

	// ===== GrabTest (standalone Gemini fetch) =====
	ExportProfileInt(iniPath, L"GrabTest", L"Enabled", 0);
	ExportProfileInt(iniPath, L"GrabTest", L"LostFramesToAbort", 10);
	ExportProfileInt(iniPath, L"GrabTest", L"AcquireStableFrames", 5);
	ExportProfileInt(iniPath, L"GrabTest\\Timing", L"MinCommandIntervalMs", 120);
	ExportProfileInt(iniPath, L"GrabTest\\Timing", L"DefaultMoveTimeMs", 220);
	ExportProfileInt(iniPath, L"GrabTest\\Timing", L"LockAfterMoveMs", 240);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"DeadbandPx", 10);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"CoarseCenterPx", 60);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"StableCenterFrames", 3);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"Yaw_kDegPerPx_milli", 30);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"Yaw_MinStepDeg_milli", 600);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"Yaw_MaxStepDeg_milli", 3500);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"Pitch_kDegPerPx_milli", 30);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"Pitch_MinStepDeg_milli", 600);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"Pitch_MaxStepDeg_milli", 3500);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"MaxPitchStepDeg_milli", 8000);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"CenterOffsetU", 0);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"CenterOffsetV", 0);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"MinServoPosChange", 8);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"SignJ1FromErrU", -1);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"SignJ4FromErrV", -1);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"J3_kDegPerPx_milli", 30);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"J3_MinStepDeg_milli", 600);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"J3_MaxStepDeg_milli", 3500);
	ExportProfileInt(iniPath, L"GrabTest\\Find", L"SignJ3FromErrV", -1);
	ExportProfileInt(iniPath, L"GrabTest\\Approach", L"TimeToFetchStableFrames", 2);
	ExportProfileInt(iniPath, L"GrabTest\\Approach", L"MaxAdvanceSteps", 60);
	ExportProfileInt(iniPath, L"GrabTest\\Approach", L"J2AdvanceStepDeg_milli", 2000);
	ExportProfileInt(iniPath, L"GrabTest\\Approach", L"SignJ2Advance", -1);
	ExportProfileInt(iniPath, L"GrabTest\\Gripper", L"JointIndex", 6);
	ExportProfileInt(iniPath, L"GrabTest\\Gripper", L"OpenPos", 650);
	ExportProfileInt(iniPath, L"GrabTest\\Gripper", L"ClosePos", 350);
	ExportProfileInt(iniPath, L"GrabTest\\Gripper", L"CloseStepPos", 25);
	ExportProfileInt(iniPath, L"GrabTest\\Gripper", L"CloseMoveTimeMs", 450);
	ExportProfileInt(iniPath, L"GrabTest\\Gripper", L"MaxCloseSteps", 12);
	ExportProfileInt(iniPath, L"GrabTest\\Return", L"ReturnToStartPose", 1);
	ExportProfileInt(iniPath, L"GrabTest\\Return", L"ReturnTimeMs", 1200);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"RetryRetreatSteps", 8);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Place", L"RetreatSteps", 6);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Return", L"ReturnToStartPose", 1);
	ExportProfileInt(iniPath, L"SeeAndFetch\\Return", L"ReturnTimeMs", 1200);

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

	// AutoHome (连接后自动归位的初始角度)
	ImportProfileInt(iniPath, L"AutoHome", L"J1Deg", 0);
	ImportProfileInt(iniPath, L"AutoHome", L"J2Deg", -30);
	ImportProfileInt(iniPath, L"AutoHome", L"J3Deg", 60);
	ImportProfileInt(iniPath, L"AutoHome", L"J4Deg", 30);
	ImportProfileInt(iniPath, L"AutoHome", L"J5Deg", 0);

	// ===== Kinematics =====
	// Links (mm)
	{
		const wchar_t* sec = L"Kinematics\\Links";
		auto writeInt = [&](const wchar_t* key, int fallback)
		{
			const int cur = AfxGetApp()->GetProfileInt(sec, key, fallback);
			const int v = ReadIntW(iniPath, sec, key, cur);
			AfxGetApp()->WriteProfileInt(sec, key, v);
		};
		writeInt(L"L_base_mm", 80);
		writeInt(L"L_arm1_mm", 100);
		writeInt(L"L_arm2_mm", 95);
		writeInt(L"L_wrist_mm", 95);
		writeInt(L"L_cam_mm", 55);
	}

	// Safety
	{
		const wchar_t* sec = L"Kinematics\\Safety";
		auto writeInt = [&](const wchar_t* key, int fallback)
		{
			const int cur = AfxGetApp()->GetProfileInt(sec, key, fallback);
			const int v = ReadIntW(iniPath, sec, key, cur);
			AfxGetApp()->WriteProfileInt(sec, key, v);
		};
		writeInt(L"PitchMinDeg", -90);
		writeInt(L"PitchMaxDeg", 90);
		writeInt(L"ZMinMm", 20);
		writeInt(L"RMinMm", 30);
		writeInt(L"RMaxMm", 290);
	}

	// Joint calib (J1..J5)
	for (int j = 1; j <= 5; j++)
	{
		CString sec;
		sec.Format(L"Kinematics\\J%d", j);

		auto writeInt = [&](const wchar_t* key, int fallback)
		{
			const int cur = AfxGetApp()->GetProfileInt(sec, key, fallback);
			const int v = ReadIntW(iniPath, sec, key, cur);
			AfxGetApp()->WriteProfileInt(sec, key, v);
		};

		writeInt(L"PosAt0Deg", 500);
		writeInt(L"PosAtPlusDeg", 500);
		{
			const int curPlus = AfxGetApp()->GetProfileInt(sec, L"PlusDeg", 45);
			int plusDeg = ReadIntW(iniPath, sec, L"PlusDeg", curPlus);
			if (plusDeg == 0) plusDeg = 45; // avoid divide-by-zero in kinematics mapping
			AfxGetApp()->WriteProfileInt(sec, L"PlusDeg", plusDeg);
		}
		// milli-degree
		writeInt(L"ZeroOffset_mdeg", 0);
		// PhysicalInvert (0/1)
		writeInt(L"PhysicalInvert", 0);
	}

	// ===== Tool =====
	{
		const wchar_t* sec = L"Tool\\Offsets";
		auto writeInt = [&](const wchar_t* key, int fallback)
		{
			const int cur = AfxGetApp()->GetProfileInt(sec, key, fallback);
			const int v = ReadIntW(iniPath, sec, key, cur);
			AfxGetApp()->WriteProfileInt(sec, key, v);
		};
		writeInt(L"Joint5ToCam_X_mm", 0);
		writeInt(L"Joint5ToCam_Y_mm", -55);
		writeInt(L"Joint5ToCam_Z_mm", 0);
		writeInt(L"CamToGrip_X_mm", 0);
		writeInt(L"CamToGrip_Y_mm", 0);
		writeInt(L"CamToGrip_Z_mm", 40);
	}

	// ===== Vision =====
	ImportProfileInt(iniPath, L"Vision", L"Mode", 0);
	ImportProfileInt(iniPath, L"Vision", L"AlgoEnabled", 1);
	ImportProfileInt(iniPath, L"Vision", L"ProcEnabled", 1);
	ImportProfileInt(iniPath, L"Vision", L"NoDrive", 1);
	ImportProfileInt(iniPath, L"Vision", L"ProcessPeriodMs", 33);
	ImportProfileInt(iniPath, L"Vision", L"SampleStride", 8);
	ImportProfileInt(iniPath, L"Vision", L"EmaAlpha_milli", 350);
	ImportProfileInt(iniPath, L"Vision", L"ExcludeHand", 1);
	ImportProfileInt(iniPath, L"Vision", L"ExcludeHandInflatePx", 20);
	ImportProfileInt(iniPath, L"Vision", L"ExcludeHandOverlap_milli", 300);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"Enabled", 1);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"MaxRayLenPx", 320);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"MaxRayPerpPx", 90);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"MaxRadiusPx", 140);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"HoldLockMs", 3000);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"HoldConfirmMs", 3000);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"HoldCancelMs", 3000);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"CancelFlashMs", 800);
	ImportProfileInt(iniPath, L"Vision\\PointPick", L"IouSame_milli", 500);

	ImportProfileInt(iniPath, L"Vision\\Aruco", L"MarkerLengthMm", 40);

	ImportProfileInt(iniPath, L"Vision\\Depth", L"NearMm", 120);
	ImportProfileInt(iniPath, L"Vision\\Depth", L"FarMm", 220);

	ImportProfileString(iniPath, L"Vision\\Detector", L"OnnxPath", L"");
	ImportProfileInt(iniPath, L"Vision\\Detector", L"InputW", 320);
	ImportProfileInt(iniPath, L"Vision\\Detector", L"InputH", 320);
	ImportProfileInt(iniPath, L"Vision\\Detector", L"Conf_milli", 500);
	ImportProfileInt(iniPath, L"Vision\\Detector", L"Nms_milli", 400);

	ImportProfileString(iniPath, L"Vision\\Gemini", L"ApiKey", L"");
	ImportProfileString(iniPath, L"Vision\\Gemini", L"Model", L"gemini-3-flash-preview");
	ImportProfileInt(iniPath, L"Vision\\Gemini", L"IntervalMs", 2000);
	ImportProfileString(iniPath, L"Vision\\Gemini", L"Proxy", L"");

	ImportProfileString(iniPath, L"Vision\\Hand", L"PalmOnnxPath", L"");
	ImportProfileString(iniPath, L"Vision\\Hand", L"LandmarkOnnxPath", L"");
	ImportProfileInt(iniPath, L"Vision\\Hand", L"PinchThreshNorm_milli", 250);

	// ===== SeeAndFetch =====
	ImportProfileInt(iniPath, L"SeeAndFetch", L"PreferArucoDuringAuto", 1);
	ImportProfileInt(iniPath, L"SeeAndFetch", L"LostFramesToAbort", 10);
	ImportProfileInt(iniPath, L"SeeAndFetch", L"AcquireStableFrames", 5);
	ImportProfileInt(iniPath, L"SeeAndFetch", L"EnablePlaneCache", 1);
	// Timing
	ImportProfileInt(iniPath, L"SeeAndFetch\\Timing", L"MinCommandIntervalMs", 120);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Timing", L"DefaultMoveTimeMs", 220);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Timing", L"LockAfterMoveMs", 240);
	// Find
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"DeadbandPx", 10);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"StableCenterFrames", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Yaw_kDegPerPx_milli", 30);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Yaw_MinStepDeg_milli", 600);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Yaw_MaxStepDeg_milli", 3500);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Pitch_kDegPerPx_milli", 30);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Pitch_MinStepDeg_milli", 600);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"Pitch_MaxStepDeg_milli", 3500);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"J4PreferAbsDeg_milli", 35000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"SignJ1FromErrU", +1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"SignJ4FromErrV", +1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Find", L"SignJ3FromErrV", +1);
	// Approach
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"GraspDepthMm", 160);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"DepthStableFrames", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"DepthMaxJumpMm", 40);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"RangeMode", 0);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"GraspBoxAreaPx2", 30000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"GraspBoxScale_milli", 0);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"BoxStableFrames", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"BoxAreaMaxJumpPx2", 20000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"BboxRequireDetector", 1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"MaxAdvanceSteps", 60);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"MaxAttempts", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"RetryRetreatSteps", 8);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"J2AdvanceStepDeg_milli", 2000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"SignJ2Advance", +1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Approach", L"EnableJ1FineTune", 1);
	// Gripper
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"JointIndex", 6);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"OpenPos", 650);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"ClosePos", 350);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"CloseStepPos", 25);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"CloseMoveTimeMs", 450);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"MaxCloseSteps", 12);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"EnableStallDetect", 0);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"StallDetectDeltaPos", 10);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"StallDetectMaxAgeMs", 800);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"MaxAttempts", 2);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Gripper", L"AdvanceStepsOnFail", 1);
	// Place / Return
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"Mode", 0);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"VisionMode", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"CenterStableFrames", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"RangeMode", 1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"PlaceDepthMm", 180);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"PlaceBoxAreaPx2", 24000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"PlaceBoxScale_milli", 0);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"BoxStableFrames", 3);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"BoxAreaMaxJumpPx2", 20000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"MaxDownSteps", 30);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"J2DownStepDeg_milli", 2000);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"SignJ2Down", +1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"MaxAttempts", 2);

	// ===== GrabTest =====
	ImportProfileInt(iniPath, L"GrabTest", L"Enabled", 0);
	ImportProfileInt(iniPath, L"GrabTest", L"LostFramesToAbort", 10);
	ImportProfileInt(iniPath, L"GrabTest", L"AcquireStableFrames", 5);
	ImportProfileInt(iniPath, L"GrabTest\\Timing", L"MinCommandIntervalMs", 120);
	ImportProfileInt(iniPath, L"GrabTest\\Timing", L"DefaultMoveTimeMs", 220);
	ImportProfileInt(iniPath, L"GrabTest\\Timing", L"LockAfterMoveMs", 240);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"DeadbandPx", 10);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"CoarseCenterPx", 60);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"StableCenterFrames", 3);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"Yaw_kDegPerPx_milli", 30);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"Yaw_MinStepDeg_milli", 600);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"Yaw_MaxStepDeg_milli", 3500);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"Pitch_kDegPerPx_milli", 30);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"Pitch_MinStepDeg_milli", 600);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"Pitch_MaxStepDeg_milli", 3500);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"MaxPitchStepDeg_milli", 8000);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"CenterOffsetU", 0);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"CenterOffsetV", 0);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"MinServoPosChange", 8);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"SignJ1FromErrU", -1);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"SignJ4FromErrV", -1);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"J3_kDegPerPx_milli", 30);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"J3_MinStepDeg_milli", 600);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"J3_MaxStepDeg_milli", 3500);
	ImportProfileInt(iniPath, L"GrabTest\\Find", L"SignJ3FromErrV", -1);
	ImportProfileInt(iniPath, L"GrabTest\\Approach", L"TimeToFetchStableFrames", 2);
	ImportProfileInt(iniPath, L"GrabTest\\Approach", L"MaxAdvanceSteps", 60);
	ImportProfileInt(iniPath, L"GrabTest\\Approach", L"J2AdvanceStepDeg_milli", 2000);
	ImportProfileInt(iniPath, L"GrabTest\\Approach", L"SignJ2Advance", -1);
	ImportProfileInt(iniPath, L"GrabTest\\Gripper", L"JointIndex", 6);
	ImportProfileInt(iniPath, L"GrabTest\\Gripper", L"OpenPos", 650);
	ImportProfileInt(iniPath, L"GrabTest\\Gripper", L"ClosePos", 350);
	ImportProfileInt(iniPath, L"GrabTest\\Gripper", L"CloseStepPos", 25);
	ImportProfileInt(iniPath, L"GrabTest\\Gripper", L"CloseMoveTimeMs", 450);
	ImportProfileInt(iniPath, L"GrabTest\\Gripper", L"MaxCloseSteps", 12);
	ImportProfileInt(iniPath, L"GrabTest\\Return", L"ReturnToStartPose", 1);
	ImportProfileInt(iniPath, L"GrabTest\\Return", L"ReturnTimeMs", 1200);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"RetryRetreatSteps", 8);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Place", L"RetreatSteps", 6);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Return", L"ReturnToStartPose", 1);
	ImportProfileInt(iniPath, L"SeeAndFetch\\Return", L"ReturnTimeMs", 1200);

	r.ok = true;
	return r;
}
