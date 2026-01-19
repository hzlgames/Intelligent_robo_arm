
// 智能机械臂Dlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "智能机械臂.h"
#include "智能机械臂Dlg.h"
#include "afxdialogex.h"
#include "DiagnosticsSheet.h"
#include "SettingsIo.h"
#include "AppMessages.h"
#include "ArmCommsService.h"
#include "ArmStateEstimator.h"
#include "Resource.h"
#include "KinematicsOverlayService.h"

#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <climits>
#include <vector>

#if defined(__has_include)
#if __has_include(<opencv2/core.hpp>) && __has_include(<opencv2/imgproc.hpp>)
#define SMARTARM_HAS_OPENCV_HEADERS 1
#else
#define SMARTARM_HAS_OPENCV_HEADERS 0
#endif
#else
#define SMARTARM_HAS_OPENCV_HEADERS 0
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

namespace
{
	std::vector<CString> EnumerateComPortsFromRegistry()
	{
		std::vector<CString> out;
		HKEY hKey = nullptr;
		if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey) != ERROR_SUCCESS)
		{
			return out;
		}

		DWORD index = 0;
		WCHAR valueName[256];
		DWORD valueNameLen = ARRAYSIZE(valueName);
		BYTE data[256];
		DWORD dataLen = sizeof(data);
		DWORD type = 0;

		while (true)
		{
			valueNameLen = ARRAYSIZE(valueName);
			dataLen = sizeof(data);
			const LSTATUS s = RegEnumValueW(
				hKey,
				index,
				valueName,
				&valueNameLen,
				nullptr,
				&type,
				data,
				&dataLen);
			if (s != ERROR_SUCCESS) break;

			if (type == REG_SZ)
			{
				const wchar_t* str = reinterpret_cast<const wchar_t*>(data);
				if (str && wcslen(str) > 0)
				{
					out.push_back(CString(str));
				}
			}
			index++;
		}

		RegCloseKey(hKey);

		std::sort(out.begin(), out.end(), [](const CString& a, const CString& b) {
			auto toNum = [](const CString& s) -> int {
				if (s.GetLength() >= 4 && (s.Left(3).CompareNoCase(L"COM") == 0))
				{
					return _wtoi(s.Mid(3));
				}
				return 9999;
				};
			const int na = toNum(a);
			const int nb = toNum(b);
			if (na != nb) return na < nb;
			return a.CompareNoCase(b) < 0;
			});
		out.erase(std::unique(out.begin(), out.end(), [](const CString& a, const CString& b) {
			return a.CompareNoCase(b) == 0;
			}), out.end());
		return out;
	}

	CString LoadStrOr(UINT id, LPCWSTR fallback)
	{
		CString s;
		if (!s.LoadString(id))
		{
			s = fallback;
		}
		return s;
	}

	// 计算将 (srcW,srcH) 的画面按比例缩放后，居中放入 dstRect 的“有效显示区域”（letterbox）。
	CRect ComputeLetterboxRect(int srcW, int srcH, const CRect& dstRect)
	{
		if (srcW <= 0 || srcH <= 0)
		{
			return dstRect;
		}
		const int dw = dstRect.Width();
		const int dh = dstRect.Height();
		if (dw <= 0 || dh <= 0)
		{
			return dstRect;
		}

		// 如果以高度为基准缩放后宽度不超 dst，则竖向充满（左右留黑）；否则横向充满（上下留黑）
		// 条件等价于：srcW/srcH <= dw/dh -> srcW*dh <= dw*srcH
		int outW = 0;
		int outH = 0;
		if ((long long)srcW * (long long)dh <= (long long)dw * (long long)srcH)
		{
			outH = dh;
			outW = (int)((long long)dh * (long long)srcW / (long long)srcH);
		}
		else
		{
			outW = dw;
			outH = (int)((long long)dw * (long long)srcH / (long long)srcW);
		}

		const int left = dstRect.left + (dw - outW) / 2;
		const int top = dstRect.top + (dh - outH) / 2;
		return CRect(left, top, left + outW, top + outH);
	}
}


// C智能机械臂Dlg 对话框



C智能机械臂Dlg::C智能机械臂Dlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MY_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void C智能机械臂Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);

	// ===== 主界面：相机预览控件绑定 =====
	DDX_Control(pDX, IDC_MAIN_COMBO_CAMERA, m_comboMainCamera);
	DDX_Control(pDX, IDC_MAIN_BTN_REFRESH_CAM, m_btnMainCamRefresh);
	DDX_Control(pDX, IDC_MAIN_BTN_START_CAM, m_btnMainCamStart);
	DDX_Control(pDX, IDC_MAIN_BTN_STOP_CAM, m_btnMainCamStop);
	DDX_Control(pDX, IDC_MAIN_STATIC_CAM_STATUS, m_staticMainCamStatus);
	DDX_Control(pDX, IDC_MAIN_STATIC_CAM_INFO, m_staticMainCamInfo);
	DDX_Control(pDX, IDC_MAIN_STATIC_VIDEO, m_staticMainVideo);

	DDX_Control(pDX, IDC_MAIN_CHECK_MIRROR, m_chkMainMirror);
	DDX_Control(pDX, IDC_MAIN_CHECK_CROSSHAIR, m_chkMainCrosshair);
	DDX_Control(pDX, IDC_MAIN_CHECK_GRID, m_chkMainGrid);
	DDX_Control(pDX, IDC_MAIN_COMBO_ROTATION, m_comboMainRotation);

	// ===== 主界面：串口快捷入口 =====
	DDX_Control(pDX, IDC_MAIN_GROUP_SERIAL, m_grpMainSerial);
	DDX_Control(pDX, IDC_MAIN_COMBO_COMPORT, m_comboMainCom);
	DDX_Control(pDX, IDC_MAIN_BTN_REFRESH_COM, m_btnMainComRefresh);
	DDX_Control(pDX, IDC_MAIN_CHECK_SIMULATE, m_chkMainSimulate);
	DDX_Control(pDX, IDC_MAIN_BTN_CONNECT, m_btnMainComConnect);
	DDX_Control(pDX, IDC_MAIN_STATIC_SERIAL_STATUS, m_staticMainSerialStatus);

	// ===== 主界面：视觉跟随 =====
	DDX_Control(pDX, IDC_MAIN_GROUP_VS, m_grpMainVs);
	DDX_Control(pDX, IDC_MAIN_CHECK_VS_ENABLE, m_chkVsEnable);
	DDX_Control(pDX, IDC_MAIN_COMBO_VS_MODE, m_comboVsMode);
	DDX_Control(pDX, IDC_MAIN_SLIDER_VS_ADVANCE, m_sliderVsAdvance);
	DDX_Control(pDX, IDC_MAIN_CHECK_VS_OVERRIDE, m_chkVsOverride);
	DDX_Control(pDX, IDC_MAIN_CHECK_VS_NODRIVE, m_chkVsNoDrive);
	DDX_Control(pDX, IDC_MAIN_STATIC_VS_STATUS, m_staticVsStatus);

	// ===== 主界面：视觉识别（独立验收）=====
	DDX_Control(pDX, IDC_MAIN_GROUP_VISION, m_grpMainVision);
	DDX_Control(pDX, IDC_MAIN_CHECK_VISION_PROC, m_chkVisionProcEnable);
	DDX_Control(pDX, IDC_MAIN_LBL_VISION_ALGO, m_staticVisionAlgo);
	DDX_Control(pDX, IDC_MAIN_COMBO_VISION_ALGO, m_comboVisionAlgo);

	// ===== 主界面：状态区域控件 =====
	DDX_Control(pDX, IDC_MAIN_GROUP_STATUS, m_grpMainStatus);

	// ===== 主界面：姿态可视化面板 =====
	DDX_Control(pDX, IDC_MAIN_GROUP_POSTURE, m_grpMainPosture);
	DDX_Control(pDX, IDC_MAIN_STATIC_POSTURE, m_ctrlPosture);
	DDX_Control(pDX, IDC_MAIN_STATIC_POSE, m_staticMainPose);
	DDX_Control(pDX, IDC_MAIN_BTN_EMERGENCY_STOP, m_btnEmergencyStop);

	// ===== 主界面：See&Fetch（自动流程） =====
	DDX_Control(pDX, IDC_MAIN_CHECK_SF_ENABLE, m_chkSfEnable);
	DDX_Control(pDX, IDC_MAIN_CHECK_SF_GRABTEST, m_chkSfGrabTest);
	DDX_Control(pDX, IDC_MAIN_BTN_SF_START, m_btnSfStart);
	DDX_Control(pDX, IDC_MAIN_BTN_SF_CANCEL, m_btnSfCancel);
	DDX_Control(pDX, IDC_MAIN_BTN_SF_ESTOP, m_btnSfEStop);
	DDX_Control(pDX, IDC_MAIN_STATIC_SF_STATUS, m_staticSfStatus);
}

BEGIN_MESSAGE_MAP(C智能机械臂Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_DIAGNOSTICS, &C智能机械臂Dlg::OnBnClickedDiagnostics)
	ON_BN_CLICKED(IDC_BTN_EXPORT_PARAMS, &C智能机械臂Dlg::OnBnClickedExportParams)
	ON_BN_CLICKED(IDC_BTN_IMPORT_PARAMS, &C智能机械臂Dlg::OnBnClickedImportParams)

	// 主界面：相机预览
	ON_BN_CLICKED(IDC_MAIN_BTN_REFRESH_CAM, &C智能机械臂Dlg::OnBnClickedMainCamRefresh)
	ON_BN_CLICKED(IDC_MAIN_BTN_START_CAM, &C智能机械臂Dlg::OnBnClickedMainCamStart)
	ON_BN_CLICKED(IDC_MAIN_BTN_STOP_CAM, &C智能机械臂Dlg::OnBnClickedMainCamStop)
	ON_BN_CLICKED(IDC_MAIN_CHECK_MIRROR, &C智能机械臂Dlg::OnBnClickedMainCamRefresh)
	ON_BN_CLICKED(IDC_MAIN_CHECK_CROSSHAIR, &C智能机械臂Dlg::OnBnClickedMainCamRefresh)
	ON_BN_CLICKED(IDC_MAIN_CHECK_GRID, &C智能机械臂Dlg::OnBnClickedMainCamRefresh)
	ON_CBN_SELCHANGE(IDC_MAIN_COMBO_ROTATION, &C智能机械臂Dlg::OnBnClickedMainCamRefresh)

	// 主界面：急停
	ON_BN_CLICKED(IDC_MAIN_BTN_EMERGENCY_STOP, &C智能机械臂Dlg::OnBnClickedEmergencyStop)

	// 主界面：串口快捷入口
	ON_BN_CLICKED(IDC_MAIN_BTN_REFRESH_COM, &C智能机械臂Dlg::OnBnClickedMainSerialRefresh)
	ON_BN_CLICKED(IDC_MAIN_BTN_CONNECT, &C智能机械臂Dlg::OnBnClickedMainSerialConnect)
	ON_BN_CLICKED(IDC_MAIN_CHECK_SIMULATE, &C智能机械臂Dlg::OnBnClickedMainSerialSimulate)
	ON_CBN_SELCHANGE(IDC_MAIN_COMBO_COMPORT, &C智能机械臂Dlg::OnCbnSelChangeMainSerialCom)

	ON_WM_HSCROLL()
	ON_STN_CLICKED(IDC_MAIN_STATIC_VIDEO, &C智能机械臂Dlg::OnStnClickedMainVideo)
	ON_CBN_SELCHANGE(IDC_MAIN_COMBO_VISION_ALGO, &C智能机械臂Dlg::OnCbnSelChangeVisionAlgo)
	ON_BN_CLICKED(IDC_MAIN_CHECK_VISION_PROC, &C智能机械臂Dlg::OnBnClickedVisionProcEnable)
	ON_BN_CLICKED(IDC_MAIN_CHECK_VS_NODRIVE, &C智能机械臂Dlg::OnBnClickedVsNoDrive)

	// 主界面：See&Fetch
	ON_BN_CLICKED(IDC_MAIN_CHECK_SF_ENABLE, &C智能机械臂Dlg::OnBnClickedSfEnable)
	ON_BN_CLICKED(IDC_MAIN_CHECK_SF_GRABTEST, &C智能机械臂Dlg::OnBnClickedSfGrabTest)
	ON_BN_CLICKED(IDC_MAIN_BTN_SF_START, &C智能机械臂Dlg::OnBnClickedSfStart)
	ON_BN_CLICKED(IDC_MAIN_BTN_SF_CANCEL, &C智能机械臂Dlg::OnBnClickedSfCancel)
	ON_BN_CLICKED(IDC_MAIN_BTN_SF_ESTOP, &C智能机械臂Dlg::OnBnClickedSfEStop)

	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_GETMINMAXINFO()
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &C智能机械臂Dlg::OnSettingsImported)
END_MESSAGE_MAP()


// C智能机械臂Dlg 消息处理程序

BOOL C智能机械臂Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// 还原上次窗口大小/位置（允许用户自行调整窗口大小）
	LoadMainWindowPlacement();

	// ===== 主界面：串口快捷入口 =====
	LoadMainSerialSettings();
	RefreshMainComList();
	UpdateMainSerialStatusText();

	// ===== 主界面：相机预览初始化 =====
	m_bDestroying = false;
	m_bMainPreviewing = false;
	m_estimatedFps = 0.0f;
	m_lastFrameCount = 0;
	m_lastTickCount = ::GetTickCount();

	// Rotation 下拉框（0/90/180/270）
	m_comboMainRotation.ResetContent();
	m_comboMainRotation.AddString(L"0");
	m_comboMainRotation.AddString(L"90");
	m_comboMainRotation.AddString(L"180");
	m_comboMainRotation.AddString(L"270");
	m_comboMainRotation.SetCurSel(0);

	RefreshMainDeviceList();
	LoadMainOverlaySettings();
	ApplyMainOverlaySettings();
	UpdateMainCamStatusText();

	// ===== 视觉跟随（Visual Servo）UI 初始化 =====
	m_comboVsMode.ResetContent();
	m_comboVsMode.AddString(L"居中(Center)");
	m_comboVsMode.AddString(L"沿指向(FollowRay)");
	m_comboVsMode.AddString(L"先居中后推进(LookAndMove)");
	m_comboVsMode.SetCurSel(2);
	m_chkVsEnable.SetCheck(BST_UNCHECKED);
	m_chkVsOverride.SetCheck(BST_UNCHECKED);
	m_sliderVsAdvance.SetRange(-100, 100);
	m_sliderVsAdvance.SetTicFreq(20);
	m_sliderVsAdvance.SetPos(0);
	m_staticVsStatus.SetWindowTextW(L"VS:OFF");

	// ===== 视觉识别（VisionService::Mode）UI：来自 .rc（便于资源编辑器调整）=====
	if (m_comboVisionAlgo.GetSafeHwnd())
	{
		m_comboVisionAlgo.ResetContent();
		m_comboVisionAlgo.AddString(L"手动(点击)");                 // 0
		m_comboVisionAlgo.AddString(L"自动(Auto)");                 // 1
		m_comboVisionAlgo.AddString(L"最亮点(BrightestPoint)");     // 2
		m_comboVisionAlgo.AddString(L"色块(红)(ColorTrack)");       // 3
		m_comboVisionAlgo.AddString(L"ArUco");                      // 4
		m_comboVisionAlgo.AddString(L"Detector(ONNX/OpenCV DNN)");  // 5
		m_comboVisionAlgo.AddString(L"双色贴纸(HandSticker)");      // 6
		m_comboVisionAlgo.AddString(L"手部关键点(HandLandmarks)");  // 7
		m_comboVisionAlgo.AddString(L"Gemini(云端)");                // 8
		m_comboVisionAlgo.SetCurSel(1); // 默认 Auto（后续 LoadVisionSettingsFromProfile 会覆盖）
	}

	// ===== 主界面：运动初始化 =====
	m_motion.LoadConfig();
	m_kc.LoadAll();

	// ===== 主界面：姿态可视化初始化 =====
	if (m_ctrlPosture.GetSafeHwnd())
	{
		const auto& links = m_kc.Links();
		m_ctrlPosture.SetLinkLengths(links.L_base, links.L_arm1, links.L_arm2, links.L_wrist);
	}

	// ===== VisionService 初始化（独立视觉线程）=====
	m_vision.SetPreview(m_pMainPreview); // 可能为空；StartMainPreview 后会更新
	LoadVisionSettingsFromProfile();
	LoadSeeAndFetchSettingsFromProfile();
	LoadGrabTestSettingsFromProfile();
	m_tool.LoadAll();
	if (m_chkSfGrabTest.GetSafeHwnd())
	{
		const bool grabTest = AfxGetApp()->GetProfileInt(L"GrabTest", L"Enabled", 0) ? true : false;
		m_chkSfGrabTest.SetCheck(grabTest ? BST_CHECKED : BST_UNCHECKED);
		if (grabTest)
		{
			ApplyGrabTestVisionMode(true);
		}
	}
	// 识别启用由 profile/checkbox 控制；默认开启，但与 VS Enable 解耦
	m_vision.SetEnabled(m_chkVisionProcEnable.GetSafeHwnd() && (m_chkVisionProcEnable.GetCheck() == BST_CHECKED) && m_visionAlgoEnabled);
	m_vision.Start();

	// 初始姿态显示：优先使用读回估计（无 Jog 目标）
	{
		ArmStateEstimator::ArmState st{};
		(void)ArmStateEstimator::Estimate(m_motion, m_kc, st, nullptr);
		const auto pose0 = st.joint5PoseBase;
		CString s;
		s.Format(L"Pose: (X=%.0f,Y=%.0f,Z=%.0f,p=%.1f)",
		         pose0.x_mm, pose0.y_mm, pose0.z_mm, pose0.pitch_deg);
		m_staticMainPose.SetWindowTextW(s);
	}

	// FPS 计时器（每秒刷新一次）
	m_timerFps = SetTimer(1, 1000, nullptr);

	// 运动 Tick（20Hz）
	SetTimer(2, 50, nullptr);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void C智能机械臂Dlg::SyncVisionAlgoUiFromState()
{
	if (!m_comboVisionAlgo.GetSafeHwnd()) return;

	// 0: 手动(点击)
	if (!m_visionAlgoEnabled)
	{
		m_comboVisionAlgo.SetCurSel(0);
		return;
	}

	int sel = 1; // Auto
	switch (m_visionAlgoMode)
	{
	case VisionService::Mode::Auto: sel = 1; break;
	case VisionService::Mode::BrightestPoint: sel = 2; break;
	case VisionService::Mode::ColorTrack: sel = 3; break;
	case VisionService::Mode::Aruco: sel = 4; break;
	case VisionService::Mode::Detector: sel = 5; break;
	case VisionService::Mode::HandSticker: sel = 6; break;
	case VisionService::Mode::HandLandmarks: sel = 7; break;
	case VisionService::Mode::Gemini: sel = 8; break;
	default: sel = 1; break;
	}
	m_comboVisionAlgo.SetCurSel(sel);
}

void C智能机械臂Dlg::LoadVisionSettingsFromProfile()
{
	CWinApp* app = AfxGetApp();
	if (!app) return;

	// ProcEnabled：视觉线程是否产出识别结果（与 VS Enable 解耦）
	const int procOn = app->GetProfileInt(L"Vision", L"ProcEnabled", 1);
	if (m_chkVisionProcEnable.GetSafeHwnd())
	{
		m_chkVisionProcEnable.SetCheck(procOn ? BST_CHECKED : BST_UNCHECKED);
	}

	// NoDrive：仅测试（默认开启，避免无意联动运动）
	const int noDrive = app->GetProfileInt(L"Vision", L"NoDrive", 1);
	if (m_chkVsNoDrive.GetSafeHwnd())
	{
		m_chkVsNoDrive.SetCheck(noDrive ? BST_CHECKED : BST_UNCHECKED);
	}

	// AlgoEnabled：0=手动(点击)，1=启用视觉识别（与 Mode 搭配）
	m_visionAlgoEnabled = app->GetProfileInt(L"Vision", L"AlgoEnabled", 1) ? true : false;

	// Mode
	const int mode = app->GetProfileInt(L"Vision", L"Mode", 0);
	VisionService::Mode m = VisionService::Mode::Auto;
	if (mode == 1) m = VisionService::Mode::BrightestPoint;
	else if (mode == 2) m = VisionService::Mode::Aruco;
	else if (mode == 3) m = VisionService::Mode::ColorTrack;
	else if (mode == 4) m = VisionService::Mode::Detector;
	else if (mode == 5) m = VisionService::Mode::HandSticker;
	else if (mode == 6) m = VisionService::Mode::HandLandmarks;
	else if (mode == 7) m = VisionService::Mode::Gemini;
	m_vision.SetMode(m);
	m_visionAlgoMode = m;
	SyncVisionAlgoUiFromState();

	// Thread params
	VisionService::Params vp = m_vision.GetParams();
	vp.processPeriodMs = app->GetProfileInt(L"Vision", L"ProcessPeriodMs", vp.processPeriodMs);
	vp.sampleStride = app->GetProfileInt(L"Vision", L"SampleStride", vp.sampleStride);
	const int emaMilli = app->GetProfileInt(L"Vision", L"EmaAlpha_milli", (int)std::lround(vp.emaAlpha * 1000.0));
	vp.emaAlpha = (double)emaMilli / 1000.0;
	vp.arucoMarkerLengthMm = (double)app->GetProfileInt(L"Vision\\Aruco", L"MarkerLengthMm", (int)vp.arucoMarkerLengthMm);
	vp.depthNearMm = app->GetProfileInt(L"Vision\\Depth", L"NearMm", vp.depthNearMm);
	vp.depthFarMm = app->GetProfileInt(L"Vision\\Depth", L"FarMm", vp.depthFarMm);
	vp.excludeHand = app->GetProfileInt(L"Vision", L"ExcludeHand", 1) ? true : false;
	vp.excludeHandInflatePx = app->GetProfileInt(L"Vision", L"ExcludeHandInflatePx", vp.excludeHandInflatePx);
	{
		const int ovMilli = app->GetProfileInt(L"Vision", L"ExcludeHandOverlap_milli", (int)std::lround(vp.excludeHandMaxOverlap * 1000.0));
		vp.excludeHandMaxOverlap = (double)ovMilli / 1000.0;
	}
	// PointPick params
	vp.pointPickEnabled = app->GetProfileInt(L"Vision\\PointPick", L"Enabled", vp.pointPickEnabled ? 1 : 0) ? true : false;
	vp.pointPickMaxRayLenPx = app->GetProfileInt(L"Vision\\PointPick", L"MaxRayLenPx", vp.pointPickMaxRayLenPx);
	vp.pointPickMaxRayPerpPx = app->GetProfileInt(L"Vision\\PointPick", L"MaxRayPerpPx", vp.pointPickMaxRayPerpPx);
	vp.pointPickMaxRadiusPx = app->GetProfileInt(L"Vision\\PointPick", L"MaxRadiusPx", vp.pointPickMaxRadiusPx);
	vp.pointPickHoldLockMs = app->GetProfileInt(L"Vision\\PointPick", L"HoldLockMs", vp.pointPickHoldLockMs);
	vp.pointPickHoldConfirmMs = app->GetProfileInt(L"Vision\\PointPick", L"HoldConfirmMs", vp.pointPickHoldConfirmMs);
	vp.pointPickHoldCancelMs = app->GetProfileInt(L"Vision\\PointPick", L"HoldCancelMs", vp.pointPickHoldCancelMs);
	vp.pointPickCancelFlashMs = app->GetProfileInt(L"Vision\\PointPick", L"CancelFlashMs", vp.pointPickCancelFlashMs);
	{
		const int iouMilli = app->GetProfileInt(L"Vision\\PointPick", L"IouSame_milli", (int)std::lround(vp.pointPickIouSame * 1000.0));
		vp.pointPickIouSame = (double)iouMilli / 1000.0;
	}
	vp.geminiApiKey = std::wstring(app->GetProfileString(L"Vision\\Gemini", L"ApiKey", L"").GetString());
	vp.geminiModel = std::wstring(app->GetProfileString(L"Vision\\Gemini", L"Model", L"gemini-3-flash-preview").GetString());
	vp.geminiRequestIntervalMs = app->GetProfileInt(L"Vision\\Gemini", L"IntervalMs", vp.geminiRequestIntervalMs);
	vp.geminiProxy = std::wstring(app->GetProfileString(L"Vision\\Gemini", L"Proxy", L"").GetString());
	m_vision.SetParams(vp);

	// Detector params
	VisionDetector::Params dp = m_vision.GetDetectorParams();
	dp.onnxPath = std::wstring(app->GetProfileString(L"Vision\\Detector", L"OnnxPath", L"").GetString());
	dp.inputW = app->GetProfileInt(L"Vision\\Detector", L"InputW", dp.inputW);
	dp.inputH = app->GetProfileInt(L"Vision\\Detector", L"InputH", dp.inputH);
	const int confMilli = app->GetProfileInt(L"Vision\\Detector", L"Conf_milli", (int)std::lround(dp.confThreshold * 1000.0f));
	const int nmsMilli = app->GetProfileInt(L"Vision\\Detector", L"Nms_milli", (int)std::lround(dp.nmsThreshold * 1000.0f));
	dp.confThreshold = (float)confMilli / 1000.0f;
	dp.nmsThreshold = (float)nmsMilli / 1000.0f;

	// Detector 模型路径兜底：若用户未配置，则默认使用 models\\detector\\yolov5n.onnx
	// 注意：这里只“填默认 + 路径解析 + 写回配置”，不会强行要求文件存在；加载失败时 Detector 会自然返回无输出。
	{
		auto fileExists = [](const std::wstring& p) -> bool
		{
			if (p.empty()) return false;
			const DWORD a = ::GetFileAttributesW(p.c_str());
			return (a != INVALID_FILE_ATTRIBUTES) && ((a & FILE_ATTRIBUTE_DIRECTORY) == 0);
		};
		auto getExeDir = []() -> std::wstring
		{
			wchar_t buf[MAX_PATH] = {};
			DWORD n = ::GetModuleFileNameW(nullptr, buf, ARRAYSIZE(buf));
			std::wstring s(buf, buf + n);
			const size_t pos = s.find_last_of(L"\\/");
			if (pos != std::wstring::npos) s.resize(pos);
			return s;
		};
		const std::wstring exeDir = getExeDir();
		auto resolvePath = [&](const std::wstring& pathOrRel) -> std::wstring
		{
			if (fileExists(pathOrRel)) return pathOrRel;
			if (!exeDir.empty())
			{
				std::wstring p = exeDir + L"\\" + pathOrRel;
				if (fileExists(p)) return p;
			}
			return pathOrRel;
		};

		if (dp.onnxPath.empty())
		{
			dp.onnxPath = L"models\\detector\\yolov5n.onnx";
			// YOLOv5 常用输入：640x640（如果用户没填过，给默认）
			if (dp.inputW <= 0) dp.inputW = 640;
			if (dp.inputH <= 0) dp.inputH = 640;
		}
		dp.onnxPath = resolvePath(dp.onnxPath);
		// 若仍是“空/默认输入尺寸”，给一个合理默认（避免旧 ini 导出 320x320）
		if (dp.inputW == 320 && dp.inputH == 320 && dp.onnxPath.find(L"yolov5") != std::wstring::npos)
		{
			dp.inputW = 640;
			dp.inputH = 640;
		}

		app->WriteProfileString(L"Vision\\Detector", L"OnnxPath", CString(dp.onnxPath.c_str()));
		app->WriteProfileInt(L"Vision\\Detector", L"InputW", dp.inputW);
		app->WriteProfileInt(L"Vision\\Detector", L"InputH", dp.inputH);
	}
	m_vision.SetDetectorParams(dp);

	// HandLandmarks params（Palm+Handpose ONNX）
	{
		VisionHandLandmarks::Params hp = m_vision.GetHandParams();
		// 兼容历史键名：LandmarkOnnxPath（旧文档/旧导入） vs HandposeOnnxPath（更直观的新名字）
		// 说明：日志证据表明当前运行时路径为空（palmPathEmpty/handposePathEmpty=1），会导致模型永远无法 loaded，从而无任何叠加。
		std::wstring palmPath = std::wstring(app->GetProfileString(L"Vision\\Hand", L"PalmOnnxPath", L"").GetString());
		std::wstring handposePath = std::wstring(app->GetProfileString(L"Vision\\Hand", L"LandmarkOnnxPath", L"").GetString());
		if (handposePath.empty())
		{
			handposePath = std::wstring(app->GetProfileString(L"Vision\\Hand", L"HandposeOnnxPath", L"").GetString());
		}

		// 当未配置时，自动使用项目自带相对路径（你已放在 models\\hands）
		// 同时做“以 exe 目录为基准”的兼容：即使用户从资源管理器双击 exe，也能找到模型文件。
		auto fileExists = [](const std::wstring& p) -> bool
		{
			if (p.empty()) return false;
			const DWORD a = ::GetFileAttributesW(p.c_str());
			return (a != INVALID_FILE_ATTRIBUTES) && ((a & FILE_ATTRIBUTE_DIRECTORY) == 0);
		};
		auto getExeDir = []() -> std::wstring
		{
			wchar_t buf[MAX_PATH] = {};
			DWORD n = ::GetModuleFileNameW(nullptr, buf, ARRAYSIZE(buf));
			std::wstring s(buf, buf + n);
			const size_t pos = s.find_last_of(L"\\/");
			if (pos != std::wstring::npos) s.resize(pos);
			return s;
		};
		const std::wstring exeDir = getExeDir();
		auto resolvePath = [&](const std::wstring& pathOrRel) -> std::wstring
		{
			if (fileExists(pathOrRel)) return pathOrRel;
			if (!exeDir.empty())
			{
				std::wstring p = exeDir + L"\\" + pathOrRel;
				if (fileExists(p)) return p;
			}
			return pathOrRel; // 兜底：保留原值（后续 EnsureLoaded 会给出错误）
		};

		if (palmPath.empty())
		{
			palmPath = L"models\\hands\\palm_detection_mediapipe_2023feb.onnx";
		}
		if (handposePath.empty())
		{
			handposePath = L"models\\hands\\handpose_estimation_mediapipe_2023feb.onnx";
		}

		// 统一做一次路径解析（支持相对/绝对）
		palmPath = resolvePath(palmPath);
		handposePath = resolvePath(handposePath);

		hp.palmOnnxPath = palmPath;
		hp.handposeOnnxPath = handposePath;
		const int pinchMilli = app->GetProfileInt(L"Vision\\Hand", L"PinchThreshNorm_milli", (int)std::lround(hp.pinchThreshNorm * 1000.0f));
		hp.pinchThreshNorm = (float)pinchMilli / 1000.0f;
		m_vision.SetHandParams(hp);

		// 将兜底路径写回 profile，方便导出 ini / 下次启动直接生效
		app->WriteProfileString(L"Vision\\Hand", L"PalmOnnxPath", CString(hp.palmOnnxPath.c_str()));
		// 继续沿用旧键名导出（与现有 SettingsIo / 文档保持一致）
		app->WriteProfileString(L"Vision\\Hand", L"LandmarkOnnxPath", CString(hp.handposeOnnxPath.c_str()));
	}

	// Apply enable state immediately (thread can stay running, but output toggles)
	const bool procEnable = (m_chkVisionProcEnable.GetSafeHwnd() && m_chkVisionProcEnable.GetCheck() == BST_CHECKED);
	m_vision.SetEnabled(procEnable && m_visionAlgoEnabled);
}

void C智能机械臂Dlg::LoadSeeAndFetchSettingsFromProfile()
{
	CWinApp* app = AfxGetApp();
	if (!app) return;

	SeeAndFetchStateMachine::Params sp;

	// 注：方向由标定数据决定，不再需要符号设置

	// Global
	sp.preferArucoDuringAuto = app->GetProfileInt(L"SeeAndFetch", L"PreferArucoDuringAuto", 1) ? true : false;
	sp.lostFramesToAbort = app->GetProfileInt(L"SeeAndFetch", L"LostFramesToAbort", sp.lostFramesToAbort);
	sp.acquireStableFrames = app->GetProfileInt(L"SeeAndFetch", L"AcquireStableFrames", sp.acquireStableFrames);
	sp.enablePlaneCache = app->GetProfileInt(L"SeeAndFetch", L"EnablePlaneCache", sp.enablePlaneCache ? 1 : 0) ? true : false;

	// Timing
	sp.timing.minCommandIntervalMs = app->GetProfileInt(L"SeeAndFetch\\Timing", L"MinCommandIntervalMs", sp.timing.minCommandIntervalMs);
	sp.timing.defaultMoveTimeMs = app->GetProfileInt(L"SeeAndFetch\\Timing", L"DefaultMoveTimeMs", sp.timing.defaultMoveTimeMs);
	sp.timing.lockAfterMoveMs = app->GetProfileInt(L"SeeAndFetch\\Timing", L"LockAfterMoveMs", sp.timing.lockAfterMoveMs);

	// Find
	sp.find.deadbandPx = app->GetProfileInt(L"SeeAndFetch\\Find", L"DeadbandPx", sp.find.deadbandPx);
	sp.find.stableCenterFrames = app->GetProfileInt(L"SeeAndFetch\\Find", L"StableCenterFrames", sp.find.stableCenterFrames);
	{
		const int yawK = app->GetProfileInt(L"SeeAndFetch\\Find", L"Yaw_kDegPerPx_milli", (int)std::lround(sp.find.yaw_kDegPerPx * 1000.0));
		sp.find.yaw_kDegPerPx = (double)yawK / 1000.0;
		const int yawMin = app->GetProfileInt(L"SeeAndFetch\\Find", L"Yaw_MinStepDeg_milli", (int)std::lround(sp.find.yaw_minStepDeg * 1000.0));
		const int yawMax = app->GetProfileInt(L"SeeAndFetch\\Find", L"Yaw_MaxStepDeg_milli", (int)std::lround(sp.find.yaw_maxStepDeg * 1000.0));
		sp.find.yaw_minStepDeg = (double)yawMin / 1000.0;
		sp.find.yaw_maxStepDeg = (double)yawMax / 1000.0;

		const int pitK = app->GetProfileInt(L"SeeAndFetch\\Find", L"Pitch_kDegPerPx_milli", (int)std::lround(sp.find.pitch_kDegPerPx * 1000.0));
		sp.find.pitch_kDegPerPx = (double)pitK / 1000.0;
		const int pitMin = app->GetProfileInt(L"SeeAndFetch\\Find", L"Pitch_MinStepDeg_milli", (int)std::lround(sp.find.pitch_minStepDeg * 1000.0));
		const int pitMax = app->GetProfileInt(L"SeeAndFetch\\Find", L"Pitch_MaxStepDeg_milli", (int)std::lround(sp.find.pitch_maxStepDeg * 1000.0));
		sp.find.pitch_minStepDeg = (double)pitMin / 1000.0;
		sp.find.pitch_maxStepDeg = (double)pitMax / 1000.0;

		const int j4pref = app->GetProfileInt(L"SeeAndFetch\\Find", L"J4PreferAbsDeg_milli", (int)std::lround(sp.find.j4PreferAbsDeg * 1000.0));
		sp.find.j4PreferAbsDeg = (double)j4pref / 1000.0;

		// J3/J4 切换滞后：避免在边界附近频繁切换导致方向抖动
		const int j4hyst = app->GetProfileInt(L"SeeAndFetch\\Find", L"J4SwitchHysteresisDeg_milli", (int)std::lround(sp.find.j4SwitchHysteresisDeg * 1000.0));
		sp.find.j4SwitchHysteresisDeg = (double)j4hyst / 1000.0;

		// 方向符号：signJ1FromErrU 默认 -1（相机镜像补偿），其他保持 +1
		// 用户可通过配置覆盖这些值
		// 修改：J3/J4 默认也改为 -1，因為 errV>0 (下方) 需要减小 Pitch (低头)
		// [修正] J3 与 errV 同向时应“低头”，默认设为 -1
		sp.find.signJ1FromErrU = app->GetProfileInt(L"SeeAndFetch\\Find", L"SignJ1FromErrU", -1);
		sp.find.signJ4FromErrV = app->GetProfileInt(L"SeeAndFetch\\Find", L"SignJ4FromErrV", -1);
		sp.find.signJ3FromErrV = app->GetProfileInt(L"SeeAndFetch\\Find", L"SignJ3FromErrV", -1);

		// 单步 pitch 变化最大限制
		const int maxPitchMilli = app->GetProfileInt(L"SeeAndFetch\\Find", L"MaxPitchStepDeg_milli", (int)std::lround(sp.find.maxPitchStepDeg * 1000.0));
		sp.find.maxPitchStepDeg = (double)maxPitchMilli / 1000.0;

		// 中心偏移量
		sp.find.centerOffsetU = app->GetProfileInt(L"SeeAndFetch\\Find", L"CenterOffsetU", sp.find.centerOffsetU);
		sp.find.centerOffsetV = app->GetProfileInt(L"SeeAndFetch\\Find", L"CenterOffsetV", sp.find.centerOffsetV);

		// 舵机位置最小变化阈值（减少小变化指令导致的卡顿）
		sp.find.minServoPosChange = app->GetProfileInt(L"SeeAndFetch\\Find", L"MinServoPosChange", sp.find.minServoPosChange);
	}

	// Approach
	{
		const int rm = app->GetProfileInt(L"SeeAndFetch\\Approach", L"RangeMode", 0);
		if (rm == 1) sp.approach.rangeMode = SeeAndFetchStateMachine::Params::Approach::RangeMode::BboxArea;
		else if (rm == 2) sp.approach.rangeMode = SeeAndFetchStateMachine::Params::Approach::RangeMode::Auto;
		else sp.approach.rangeMode = SeeAndFetchStateMachine::Params::Approach::RangeMode::ArucoDepth;
	}
	sp.approach.graspDepthMm = app->GetProfileInt(L"SeeAndFetch\\Approach", L"GraspDepthMm", sp.approach.graspDepthMm);
	sp.approach.depthStableFrames = app->GetProfileInt(L"SeeAndFetch\\Approach", L"DepthStableFrames", sp.approach.depthStableFrames);
	sp.approach.depthMaxJumpMm = app->GetProfileInt(L"SeeAndFetch\\Approach", L"DepthMaxJumpMm", sp.approach.depthMaxJumpMm);
	sp.approach.graspBoxAreaPx2 = app->GetProfileInt(L"SeeAndFetch\\Approach", L"GraspBoxAreaPx2", sp.approach.graspBoxAreaPx2);
	sp.approach.graspBoxScale_milli = app->GetProfileInt(L"SeeAndFetch\\Approach", L"GraspBoxScale_milli", sp.approach.graspBoxScale_milli);
	sp.approach.boxStableFrames = app->GetProfileInt(L"SeeAndFetch\\Approach", L"BoxStableFrames", sp.approach.boxStableFrames);
	sp.approach.boxAreaMaxJumpPx2 = app->GetProfileInt(L"SeeAndFetch\\Approach", L"BoxAreaMaxJumpPx2", sp.approach.boxAreaMaxJumpPx2);
	sp.approach.bboxRequireDetector = app->GetProfileInt(L"SeeAndFetch\\Approach", L"BboxRequireDetector", sp.approach.bboxRequireDetector ? 1 : 0) ? true : false;
	sp.approach.maxAdvanceSteps = app->GetProfileInt(L"SeeAndFetch\\Approach", L"MaxAdvanceSteps", sp.approach.maxAdvanceSteps);
	sp.approach.maxAttempts = app->GetProfileInt(L"SeeAndFetch\\Approach", L"MaxAttempts", sp.approach.maxAttempts);
	sp.approach.retryRetreatSteps = app->GetProfileInt(L"SeeAndFetch\\Approach", L"RetryRetreatSteps", sp.approach.retryRetreatSteps);
	{
		const int j2step = app->GetProfileInt(L"SeeAndFetch\\Approach", L"J2AdvanceStepDeg_milli", (int)std::lround(sp.approach.j2AdvanceStepDeg * 1000.0));
		sp.approach.j2AdvanceStepDeg = (double)j2step / 1000.0;
	}
	// 注：方向由标定数据决定，符号固定为 +1
	sp.approach.signJ2Advance = +1;
	sp.approach.enableJ1FineTune = app->GetProfileInt(L"SeeAndFetch\\Approach", L"EnableJ1FineTune", sp.approach.enableJ1FineTune ? 1 : 0) ? true : false;

	// Gripper
	sp.gripper.jointIndex = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"JointIndex", sp.gripper.jointIndex);
	sp.gripper.openPos = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"OpenPos", sp.gripper.openPos);
	sp.gripper.closePos = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"ClosePos", sp.gripper.closePos);
	sp.gripper.closeStepPos = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"CloseStepPos", sp.gripper.closeStepPos);
	sp.gripper.closeMoveTimeMs = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"CloseMoveTimeMs", sp.gripper.closeMoveTimeMs);
	sp.gripper.maxCloseSteps = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"MaxCloseSteps", sp.gripper.maxCloseSteps);
	sp.gripper.enableStallDetect = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"EnableStallDetect", sp.gripper.enableStallDetect ? 1 : 0) ? true : false;
	sp.gripper.stallDetectDeltaPos = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"StallDetectDeltaPos", sp.gripper.stallDetectDeltaPos);
	sp.gripper.stallDetectMaxAgeMs = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"StallDetectMaxAgeMs", sp.gripper.stallDetectMaxAgeMs);
	sp.gripper.maxAttempts = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"MaxAttempts", sp.gripper.maxAttempts);
	sp.gripper.advanceStepsOnFail = app->GetProfileInt(L"SeeAndFetch\\Gripper", L"AdvanceStepsOnFail", sp.gripper.advanceStepsOnFail);

	// Place / Return
	{
		const int pm = app->GetProfileInt(L"SeeAndFetch\\Place", L"Mode", 0);
		if (pm == 1) sp.place.mode = SeeAndFetchStateMachine::Params::Place::Mode::RedDotVisual;
		else sp.place.mode = SeeAndFetchStateMachine::Params::Place::Mode::SimpleOpen;
	}
	sp.place.visionMode = app->GetProfileInt(L"SeeAndFetch\\Place", L"VisionMode", sp.place.visionMode);
	sp.place.centerStableFrames = app->GetProfileInt(L"SeeAndFetch\\Place", L"CenterStableFrames", sp.place.centerStableFrames);
	{
		const int rm = app->GetProfileInt(L"SeeAndFetch\\Place", L"RangeMode", 1);
		if (rm == 0) sp.place.rangeMode = SeeAndFetchStateMachine::Params::Place::RangeMode::ArucoDepth;
		else if (rm == 2) sp.place.rangeMode = SeeAndFetchStateMachine::Params::Place::RangeMode::Auto;
		else sp.place.rangeMode = SeeAndFetchStateMachine::Params::Place::RangeMode::BboxArea;
	}
	sp.place.placeDepthMm = app->GetProfileInt(L"SeeAndFetch\\Place", L"PlaceDepthMm", sp.place.placeDepthMm);
	sp.place.placeBoxAreaPx2 = app->GetProfileInt(L"SeeAndFetch\\Place", L"PlaceBoxAreaPx2", sp.place.placeBoxAreaPx2);
	sp.place.placeBoxScale_milli = app->GetProfileInt(L"SeeAndFetch\\Place", L"PlaceBoxScale_milli", sp.place.placeBoxScale_milli);
	sp.place.boxStableFrames = app->GetProfileInt(L"SeeAndFetch\\Place", L"BoxStableFrames", sp.place.boxStableFrames);
	sp.place.boxAreaMaxJumpPx2 = app->GetProfileInt(L"SeeAndFetch\\Place", L"BoxAreaMaxJumpPx2", sp.place.boxAreaMaxJumpPx2);
	sp.place.maxDownSteps = app->GetProfileInt(L"SeeAndFetch\\Place", L"MaxDownSteps", sp.place.maxDownSteps);
	{
		const int mdeg = app->GetProfileInt(L"SeeAndFetch\\Place", L"J2DownStepDeg_milli", (int)std::lround(sp.place.j2DownStepDeg * 1000.0));
		sp.place.j2DownStepDeg = (double)mdeg / 1000.0;
	}
	// 注：方向由标定数据决定，符号固定为 +1
	sp.place.signJ2Down = +1;
	sp.place.maxAttempts = app->GetProfileInt(L"SeeAndFetch\\Place", L"MaxAttempts", sp.place.maxAttempts);
	sp.place.retryRetreatSteps = app->GetProfileInt(L"SeeAndFetch\\Place", L"RetryRetreatSteps", sp.place.retryRetreatSteps);
	sp.place.retreatSteps = app->GetProfileInt(L"SeeAndFetch\\Place", L"RetreatSteps", sp.place.retreatSteps);
	sp.ret.returnToStartPose = app->GetProfileInt(L"SeeAndFetch\\Return", L"ReturnToStartPose", sp.ret.returnToStartPose ? 1 : 0) ? true : false;
	sp.ret.returnTimeMs = app->GetProfileInt(L"SeeAndFetch\\Return", L"ReturnTimeMs", sp.ret.returnTimeMs);

	m_sf.SetParams(sp);
}

void C智能机械臂Dlg::LoadGrabTestSettingsFromProfile()
{
	CWinApp* app = AfxGetApp();
	if (!app) return;

	GrabTestController::Params gp;
	gp.lostFramesToAbort = app->GetProfileInt(L"GrabTest", L"LostFramesToAbort", gp.lostFramesToAbort);
	gp.acquireStableFrames = app->GetProfileInt(L"GrabTest", L"AcquireStableFrames", gp.acquireStableFrames);

	gp.timing.minCommandIntervalMs = app->GetProfileInt(L"GrabTest\\Timing", L"MinCommandIntervalMs", gp.timing.minCommandIntervalMs);
	gp.timing.defaultMoveTimeMs = app->GetProfileInt(L"GrabTest\\Timing", L"DefaultMoveTimeMs", gp.timing.defaultMoveTimeMs);
	gp.timing.lockAfterMoveMs = app->GetProfileInt(L"GrabTest\\Timing", L"LockAfterMoveMs", gp.timing.lockAfterMoveMs);

	gp.find.deadbandPx = app->GetProfileInt(L"GrabTest\\Find", L"DeadbandPx", gp.find.deadbandPx);
	gp.find.coarseCenterPx = app->GetProfileInt(L"GrabTest\\Find", L"CoarseCenterPx", gp.find.coarseCenterPx);
	gp.find.stableCenterFrames = app->GetProfileInt(L"GrabTest\\Find", L"StableCenterFrames", gp.find.stableCenterFrames);
	{
		const int yawK = app->GetProfileInt(L"GrabTest\\Find", L"Yaw_kDegPerPx_milli", (int)std::lround(gp.find.yaw_kDegPerPx * 1000.0));
		gp.find.yaw_kDegPerPx = (double)yawK / 1000.0;
		const int yawMin = app->GetProfileInt(L"GrabTest\\Find", L"Yaw_MinStepDeg_milli", (int)std::lround(gp.find.yaw_minStepDeg * 1000.0));
		const int yawMax = app->GetProfileInt(L"GrabTest\\Find", L"Yaw_MaxStepDeg_milli", (int)std::lround(gp.find.yaw_maxStepDeg * 1000.0));
		gp.find.yaw_minStepDeg = (double)yawMin / 1000.0;
		gp.find.yaw_maxStepDeg = (double)yawMax / 1000.0;

		const int pitK = app->GetProfileInt(L"GrabTest\\Find", L"Pitch_kDegPerPx_milli", (int)std::lround(gp.find.pitch_kDegPerPx * 1000.0));
		gp.find.pitch_kDegPerPx = (double)pitK / 1000.0;
		const int pitMin = app->GetProfileInt(L"GrabTest\\Find", L"Pitch_MinStepDeg_milli", (int)std::lround(gp.find.pitch_minStepDeg * 1000.0));
		const int pitMax = app->GetProfileInt(L"GrabTest\\Find", L"Pitch_MaxStepDeg_milli", (int)std::lround(gp.find.pitch_maxStepDeg * 1000.0));
		gp.find.pitch_minStepDeg = (double)pitMin / 1000.0;
		gp.find.pitch_maxStepDeg = (double)pitMax / 1000.0;

		const int maxPitchMilli = app->GetProfileInt(L"GrabTest\\Find", L"MaxPitchStepDeg_milli", (int)std::lround(gp.find.maxPitchStepDeg * 1000.0));
		gp.find.maxPitchStepDeg = (double)maxPitchMilli / 1000.0;

		gp.find.centerOffsetU = app->GetProfileInt(L"GrabTest\\Find", L"CenterOffsetU", gp.find.centerOffsetU);
		gp.find.centerOffsetV = app->GetProfileInt(L"GrabTest\\Find", L"CenterOffsetV", gp.find.centerOffsetV);
		gp.find.minServoPosChange = app->GetProfileInt(L"GrabTest\\Find", L"MinServoPosChange", gp.find.minServoPosChange);
		gp.find.signJ1FromErrU = app->GetProfileInt(L"GrabTest\\Find", L"SignJ1FromErrU", gp.find.signJ1FromErrU);
		gp.find.signJ4FromErrV = app->GetProfileInt(L"GrabTest\\Find", L"SignJ4FromErrV", gp.find.signJ4FromErrV);
		const int j3K = app->GetProfileInt(L"GrabTest\\Find", L"J3_kDegPerPx_milli", (int)std::lround(gp.find.j3_kDegPerPx * 1000.0));
		gp.find.j3_kDegPerPx = (double)j3K / 1000.0;
		const int j3Min = app->GetProfileInt(L"GrabTest\\Find", L"J3_MinStepDeg_milli", (int)std::lround(gp.find.j3_minStepDeg * 1000.0));
		const int j3Max = app->GetProfileInt(L"GrabTest\\Find", L"J3_MaxStepDeg_milli", (int)std::lround(gp.find.j3_maxStepDeg * 1000.0));
		gp.find.j3_minStepDeg = (double)j3Min / 1000.0;
		gp.find.j3_maxStepDeg = (double)j3Max / 1000.0;
		gp.find.signJ3FromErrV = app->GetProfileInt(L"GrabTest\\Find", L"SignJ3FromErrV", gp.find.signJ3FromErrV);
	}

	gp.approach.timeToFetchStableFrames = app->GetProfileInt(L"GrabTest\\Approach", L"TimeToFetchStableFrames", gp.approach.timeToFetchStableFrames);
	gp.approach.maxAdvanceSteps = app->GetProfileInt(L"GrabTest\\Approach", L"MaxAdvanceSteps", gp.approach.maxAdvanceSteps);
	{
		const int j2step = app->GetProfileInt(L"GrabTest\\Approach", L"J2AdvanceStepDeg_milli", (int)std::lround(gp.approach.j2AdvanceStepDeg * 1000.0));
		gp.approach.j2AdvanceStepDeg = (double)j2step / 1000.0;
	}
	gp.approach.signJ2Advance = app->GetProfileInt(L"GrabTest\\Approach", L"SignJ2Advance", gp.approach.signJ2Advance);

	gp.gripper.jointIndex = app->GetProfileInt(L"GrabTest\\Gripper", L"JointIndex", gp.gripper.jointIndex);
	gp.gripper.openPos = app->GetProfileInt(L"GrabTest\\Gripper", L"OpenPos", gp.gripper.openPos);
	gp.gripper.closePos = app->GetProfileInt(L"GrabTest\\Gripper", L"ClosePos", gp.gripper.closePos);
	gp.gripper.closeStepPos = app->GetProfileInt(L"GrabTest\\Gripper", L"CloseStepPos", gp.gripper.closeStepPos);
	gp.gripper.closeMoveTimeMs = app->GetProfileInt(L"GrabTest\\Gripper", L"CloseMoveTimeMs", gp.gripper.closeMoveTimeMs);
	gp.gripper.maxCloseSteps = app->GetProfileInt(L"GrabTest\\Gripper", L"MaxCloseSteps", gp.gripper.maxCloseSteps);

	gp.ret.returnToStartPose = app->GetProfileInt(L"GrabTest\\Return", L"ReturnToStartPose", gp.ret.returnToStartPose ? 1 : 0) ? true : false;
	gp.ret.returnTimeMs = app->GetProfileInt(L"GrabTest\\Return", L"ReturnTimeMs", gp.ret.returnTimeMs);

	m_grabTest.SetParams(gp);
}

LRESULT C智能机械臂Dlg::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	if (m_bDestroying) return 0;

	// 1) 视觉参数（现有逻辑）
	LoadVisionSettingsFromProfile();
	LoadSeeAndFetchSettingsFromProfile();
	LoadGrabTestSettingsFromProfile();
	m_tool.LoadAll();
	if (m_chkSfGrabTest.GetSafeHwnd())
	{
		const bool grabTest = AfxGetApp()->GetProfileInt(L"GrabTest", L"Enabled", 0) ? true : false;
		m_chkSfGrabTest.SetCheck(grabTest ? BST_CHECKED : BST_UNCHECKED);
		if (grabTest)
		{
			ApplyGrabTestVisionMode(true);
		}
	}

	// 2) 运动学/工具参数：用户可能在"诊断中心->几何参数"或导入 ini 后修改了尺寸/偏置
	//    必须重新加载到内存，否则 Jog/IK 仍使用旧参数，表现会非常混乱。
	m_motion.LoadConfig();
	m_kc.LoadAll();

	// 3) 姿态可视化：更新连杆长度
	if (m_ctrlPosture.GetSafeHwnd())
	{
		const auto& links = m_kc.Links();
		m_ctrlPosture.SetLinkLengths(links.L_base, links.L_arm1, links.L_arm2, links.L_wrist);
	}

	// 3) 更新姿态显示（参数刷新后用读回估计）
	{
		ArmStateEstimator::ArmState st{};
		(void)ArmStateEstimator::Estimate(m_motion, m_kc, st, nullptr);
		const auto pose0 = st.joint5PoseBase;
		if (m_staticMainPose.GetSafeHwnd())
		{
			CString s;
			s.Format(L"Pose: (X=%.0f,Y=%.0f,Z=%.0f,p=%.1f) [参数已刷新]",
			         pose0.x_mm, pose0.y_mm, pose0.z_mm, pose0.pitch_deg);
			m_staticMainPose.SetWindowTextW(s);
		}
	}
	return 0;
}

void C智能机械臂Dlg::OnCbnSelChangeVisionAlgo()
{
	if (!m_comboVisionAlgo.GetSafeHwnd()) return;

	const int sel = m_comboVisionAlgo.GetCurSel();

	// 0: 手动(点击)
	if (sel <= 0)
	{
		m_visionAlgoEnabled = false;
	}
	else
	{
		m_visionAlgoEnabled = true;
		VisionService::Mode m = VisionService::Mode::Auto;
		if (sel == 2) m = VisionService::Mode::BrightestPoint;
		else if (sel == 3) m = VisionService::Mode::ColorTrack;
		else if (sel == 4) m = VisionService::Mode::Aruco;
		else if (sel == 5) m = VisionService::Mode::Detector;
		else if (sel == 6) m = VisionService::Mode::HandSticker;
		else if (sel == 7) m = VisionService::Mode::HandLandmarks;
		else if (sel == 8) m = VisionService::Mode::Gemini;
		else m = VisionService::Mode::Auto;

		m_vision.SetMode(m);
		m_visionAlgoMode = m;
	}

	// 立即应用开关（OnTimer 也会持续应用）
	const bool procEnable = (m_chkVisionProcEnable.GetSafeHwnd() && m_chkVisionProcEnable.GetCheck() == BST_CHECKED);
	m_vision.SetEnabled(procEnable && m_visionAlgoEnabled);

	// 持久化（便于下次启动/导出 ini）
	if (CWinApp* app = AfxGetApp())
	{
		app->WriteProfileInt(L"Vision", L"AlgoEnabled", m_visionAlgoEnabled ? 1 : 0);
		// 手动模式下也保留上次选择的 Mode（便于切回），这里仍写入当前 m_visionAlgoMode
		int mode = 0;
		switch (m_visionAlgoMode)
		{
		case VisionService::Mode::Auto: mode = 0; break;
		case VisionService::Mode::BrightestPoint: mode = 1; break;
		case VisionService::Mode::Aruco: mode = 2; break;
		case VisionService::Mode::ColorTrack: mode = 3; break;
		case VisionService::Mode::Detector: mode = 4; break;
		case VisionService::Mode::HandSticker: mode = 5; break;
		case VisionService::Mode::HandLandmarks: mode = 6; break;
		case VisionService::Mode::Gemini: mode = 7; break;
		default: mode = 0; break;
		}
		app->WriteProfileInt(L"Vision", L"Mode", mode);
	}
}

void C智能机械臂Dlg::OnBnClickedVisionProcEnable()
{
	// 视觉识别启用：仅控制 VisionService 是否产出观测/结果；线程仍常驻运行
	const bool procEnable = (m_chkVisionProcEnable.GetSafeHwnd() && m_chkVisionProcEnable.GetCheck() == BST_CHECKED);
	m_vision.SetEnabled(procEnable && m_visionAlgoEnabled);
	if (CWinApp* app = AfxGetApp())
	{
		app->WriteProfileInt(L"Vision", L"ProcEnabled", procEnable ? 1 : 0);
	}
}

void C智能机械臂Dlg::OnBnClickedVsNoDrive()
{
	const bool noDrive = (m_chkVsNoDrive.GetSafeHwnd() && m_chkVsNoDrive.GetCheck() == BST_CHECKED);
	if (CWinApp* app = AfxGetApp())
	{
		app->WriteProfileInt(L"Vision", L"NoDrive", noDrive ? 1 : 0);
	}
}

void C智能机械臂Dlg::OnBnClickedSfEnable()
{
	if (m_chkSfGrabTest.GetSafeHwnd() && m_chkSfGrabTest.GetCheck() == BST_CHECKED)
	{
		if (m_chkSfEnable.GetSafeHwnd())
		{
			m_chkSfEnable.SetCheck(BST_UNCHECKED);
		}
		m_sfEnabled = false;
		return;
	}
	m_sfEnabled = (m_chkSfEnable.GetSafeHwnd() && m_chkSfEnable.GetCheck() == BST_CHECKED);
	if (!m_sfEnabled)
	{
		m_sf.Reset();
		m_sfCmdConfirm = false;
		m_sfCmdCancel = false;
		m_sfCmdEStop = false;
		// restore vision mode if we had overridden it
		if (m_sfVisionOverridden)
		{
			m_vision.SetMode(m_sfPrevVisionMode);
			m_visionAlgoMode = m_sfPrevVisionMode;
			m_sfVisionOverridden = false;
		}
	}
}

void C智能机械臂Dlg::ApplyGrabTestVisionMode(bool on)
{
	if (!on) return;

	if (m_comboVisionAlgo.GetSafeHwnd())
	{
		m_comboVisionAlgo.SetCurSel(8); // Gemini(云端)
		OnCbnSelChangeVisionAlgo();
	}
	if (m_chkVisionProcEnable.GetSafeHwnd())
	{
		m_chkVisionProcEnable.SetCheck(BST_CHECKED);
		OnBnClickedVisionProcEnable();
	}
	{
		auto vp = m_vision.GetParams();
		vp.geminiEnableTimeToFetch = true;
		vp.geminiResetSeq++;
		m_vision.SetParams(vp);
	}
}

void C智能机械臂Dlg::OnBnClickedSfGrabTest()
{
	const bool on = (m_chkSfGrabTest.GetSafeHwnd() && m_chkSfGrabTest.GetCheck() == BST_CHECKED);
	if (CWinApp* app = AfxGetApp())
	{
		app->WriteProfileInt(L"GrabTest", L"Enabled", on ? 1 : 0);
	}
	LoadGrabTestSettingsFromProfile();
	if (on)
	{
		if (m_chkSfEnable.GetSafeHwnd())
		{
			m_chkSfEnable.SetCheck(BST_UNCHECKED);
			OnBnClickedSfEnable();
		}
		ApplyGrabTestVisionMode(true);
	}
	else
	{
		auto vp = m_vision.GetParams();
		vp.geminiEnableTimeToFetch = false;
		m_vision.SetParams(vp);
		m_grabTestEnabled = false;
		m_grabTestCmdStart = false;
		m_grabTestCmdCancel = false;
		m_grabTestCmdEStop = false;
	}
}

void C智能机械臂Dlg::OnBnClickedSfStart()
{
	const bool grabTestOn = (m_chkSfGrabTest.GetSafeHwnd() && m_chkSfGrabTest.GetCheck() == BST_CHECKED);
	if (grabTestOn)
	{
		// GrabTest: snapshot start pose for return
		std::array<int, MotionConfig::kJointCount + 1> startPos{};
		for (int j = 0; j <= MotionConfig::kJointCount; j++) startPos[(size_t)j] = -1;
		{
			const MotionConfig& mc = m_motion.Config();
			const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
			for (int j = 1; j <= MotionConfig::kJointCount; j++)
			{
				const auto& jc = mc.Get(j);
				int pos = jc.homePos;
				if (jc.servoId >= 1 && jc.servoId <= 6)
				{
					uint16_t rb = 0;
					DWORD age = 0;
					if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
					{
						pos = (int)rb;
					}
				}
				startPos[(size_t)j] = pos;
			}
		}
		m_grabTest.SetStartPose(startPos, true);
		m_grabTestEnabled = true;
		m_grabTestCmdStart = true;
		ApplyGrabTestVisionMode(true);
		return;
	}

	// 记录“开始时刻”的关节位置（用于 ReturnHome）
	std::array<int, MotionConfig::kJointCount + 1> startPos{};
	for (int j = 0; j <= MotionConfig::kJointCount; j++) startPos[(size_t)j] = -1;
	{
		const MotionConfig& mc = m_motion.Config();
		const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
		for (int j = 1; j <= MotionConfig::kJointCount; j++)
		{
			const auto& jc = mc.Get(j);
			int pos = jc.homePos;
			if (jc.servoId >= 1 && jc.servoId <= 6)
			{
				uint16_t rb = 0;
				DWORD age = 0;
				if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
				{
					pos = (int)rb;
				}
			}
			startPos[(size_t)j] = pos;
		}
	}
	m_sf.SetStartPose(startPos, true);

	// 设置 AutoHome 位置（连接后的初始姿态，用于 GoAutoHome）
	// 这个位置在 StartAutoHomeAfterConnect() 中已经计算好了
	{
		bool hasValidAutoHome = false;
		for (int j = 1; j <= MotionConfig::kJointCount; j++)
		{
			if (m_autoHomeTargetPos[(size_t)j] >= 0)
			{
				hasValidAutoHome = true;
				break;
			}
		}
		m_sf.SetAutoHomePos(m_autoHomeTargetPos, hasValidAutoHome);
	}

	// Enable + trigger confirm
	if (m_chkSfEnable.GetSafeHwnd())
	{
		m_chkSfEnable.SetCheck(BST_CHECKED);
	}
	m_sfEnabled = true;
	m_sfCmdConfirm = true;
}

void C智能机械臂Dlg::OnBnClickedSfCancel()
{
	const bool grabTestOn = (m_chkSfGrabTest.GetSafeHwnd() && m_chkSfGrabTest.GetCheck() == BST_CHECKED);
	if (grabTestOn)
	{
		m_grabTestCmdCancel = true;
		return;
	}
	m_sfCmdCancel = true;
}

void C智能机械臂Dlg::OnBnClickedSfEStop()
{
	const bool grabTestOn = (m_chkSfGrabTest.GetSafeHwnd() && m_chkSfGrabTest.GetCheck() == BST_CHECKED);
	if (grabTestOn)
	{
		m_grabTestCmdEStop = true;
		ArmCommsService::Instance().EmergencyStop();
		return;
	}
	m_sfCmdEStop = true;
	ArmCommsService::Instance().EmergencyStop();
}

BOOL C智能机械臂Dlg::PreTranslateMessage(MSG* pMsg)
{
	// Space 作为全局急停（Jog 阶段会进一步扩展按键映射）
	if (pMsg && pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_SPACE)
	{
		OnBnClickedEmergencyStop();
		return TRUE;
	}
	return CDialogEx::PreTranslateMessage(pMsg);
}

// ===== 主窗口：大小/位置持久化 =====
void C智能机械臂Dlg::LoadMainWindowPlacement()
{
	CWinApp* app = AfxGetApp();
	int x = app->GetProfileInt(L"MainWindow", L"X", INT_MIN);
	int y = app->GetProfileInt(L"MainWindow", L"Y", INT_MIN);
	int w = app->GetProfileInt(L"MainWindow", L"W", 0);
	int h = app->GetProfileInt(L"MainWindow", L"H", 0);
	if (x == INT_MIN || y == INT_MIN || w <= 0 || h <= 0)
	{
		return;
	}

	// 简单保护：避免离谱数据导致窗口不可见
	if (x < -2000 || y < -2000 || x > 20000 || y > 20000)
	{
		return;
	}

	// ===== 按 DPI 计算最小尺寸，确保恢复的窗口不会太小 =====
	HDC hdc = ::GetDC(GetSafeHwnd());
	const int dpiX = ::GetDeviceCaps(hdc, LOGPIXELSX);
	::ReleaseDC(GetSafeHwnd(), hdc);
	const double dpiScale = (double)dpiX / 96.0;
	const int minW = (int)std::lround(900.0 * dpiScale);
	const int minH = (int)std::lround(650.0 * dpiScale);
	if (w < minW) w = minW;
	if (h < minH) h = minH;

	// ===== 确保窗口在屏幕可见范围内（多显示器友好）=====
	// 获取虚拟屏幕边界（所有显示器组成的矩形）
	const int vsX = ::GetSystemMetrics(SM_XVIRTUALSCREEN);
	const int vsY = ::GetSystemMetrics(SM_YVIRTUALSCREEN);
	const int vsW = ::GetSystemMetrics(SM_CXVIRTUALSCREEN);
	const int vsH = ::GetSystemMetrics(SM_CYVIRTUALSCREEN);
	// 窗口至少有 100px 在屏幕内
	if (x + w < vsX + 100) x = vsX;
	if (y + h < vsY + 100) y = vsY;
	if (x > vsX + vsW - 100) x = vsX + vsW - w;
	if (y > vsY + vsH - 100) y = vsY + vsH - h;

	MoveWindow(x, y, w, h, FALSE);
}

void C智能机械臂Dlg::SaveMainWindowPlacement() const
{
	CWinApp* app = AfxGetApp();
	CRect rc;
	const_cast<C智能机械臂Dlg*>(this)->GetWindowRect(&rc);
	app->WriteProfileInt(L"MainWindow", L"X", rc.left);
	app->WriteProfileInt(L"MainWindow", L"Y", rc.top);
	app->WriteProfileInt(L"MainWindow", L"W", rc.Width());
	app->WriteProfileInt(L"MainWindow", L"H", rc.Height());
}

void C智能机械臂Dlg::OnBnClickedDiagnostics()
{
	CDiagnosticsSheet sheet(this);
	sheet.DoModal();
}

void C智能机械臂Dlg::OnBnClickedMainCamRefresh()
{
	RefreshMainDeviceList();
	ApplyMainOverlaySettings();
	UpdateMainCamStatusText();
}

void C智能机械臂Dlg::OnBnClickedMainCamStop()
{
	StopMainPreview();
	UpdateMainCamStatusText();
}

void C智能机械臂Dlg::OnBnClickedEmergencyStop()
{
	ArmCommsService::Instance().EmergencyStop();
	// 这里不弹框，避免影响实时操作；只更新状态文本。
	m_staticMainCamStatus.SetWindowTextW(L"已急停（队列已清空）");
}

void C智能机械臂Dlg::OnBnClickedMainCamStart()
{
	StartMainPreview();
}

// ===== 主界面：串口快捷入口 =====
void C智能机械臂Dlg::LoadMainSerialSettings()
{
	CWinApp* app = AfxGetApp();
	const int sim = app->GetProfileInt(L"MainSerial", L"Sim", 1);
	m_chkMainSimulate.SetCheck(sim ? BST_CHECKED : BST_UNCHECKED);

	// 先记住上次选择（RefreshMainComList 会尝试选中）
	const CString last = app->GetProfileString(L"MainSerial", L"Com", L"");
	if (!last.IsEmpty() && m_comboMainCom.GetSafeHwnd())
	{
		m_comboMainCom.SetWindowTextW(last);
	}
}

void C智能机械臂Dlg::SaveMainSerialSettings() const
{
	CWinApp* app = AfxGetApp();
	app->WriteProfileInt(L"MainSerial", L"Sim", (m_chkMainSimulate.GetCheck() == BST_CHECKED) ? 1 : 0);
	CString com;
	const_cast<CComboBox&>(m_comboMainCom).GetWindowTextW(com);
	app->WriteProfileString(L"MainSerial", L"Com", com);
}

void C智能机械臂Dlg::RefreshMainComList()
{
	if (!m_comboMainCom.GetSafeHwnd()) return;

	CString wanted;
	m_comboMainCom.GetWindowTextW(wanted);

	m_comboMainCom.ResetContent();
	const auto ports = EnumerateComPortsFromRegistry();
	for (const auto& p : ports)
	{
		m_comboMainCom.AddString(p);
	}

	if (!wanted.IsEmpty())
	{
		const int idx = m_comboMainCom.FindStringExact(-1, wanted);
		if (idx >= 0) m_comboMainCom.SetCurSel(idx);
	}
	if (m_comboMainCom.GetCurSel() < 0 && m_comboMainCom.GetCount() > 0)
	{
		m_comboMainCom.SetCurSel(0);
	}
}

void C智能机械臂Dlg::UpdateMainSerialStatusText()
{
	if (!m_staticMainSerialStatus.GetSafeHwnd()) return;

	auto& comms = ArmCommsService::Instance();
	CString s;
	if (!comms.IsConnected())
	{
		s = LoadStrOr(IDS_STATUS_DISCONNECTED, L"未连接");
		m_btnMainComConnect.SetWindowTextW(L"连接");
	}
	else
	{
		if (comms.IsSim())
		{
			s = LoadStrOr(IDS_STATUS_CONNECTED_SIM, L"已连接(模拟)");
		}
		else
		{
			CString com(comms.GetConnectedCom().c_str());
			if (!com.IsEmpty())
				s.Format(L"已连接(%s)", com.GetString());
			else
				s = LoadStrOr(IDS_STATUS_CONNECTED_REAL, L"已连接(真实)");
		}
		m_btnMainComConnect.SetWindowTextW(L"断开");

		// 自动归位状态（连接后先归位并验证到位才允许操作）
		if (!comms.IsSim())
		{
			if (m_autoHomeState == AutoHomeState::Moving)
			{
				s += L" 归位中…";
			}
			else if (m_autoHomeState == AutoHomeState::Ready)
			{
				s += L" 已就位";
			}
		}
	}
	m_staticMainSerialStatus.SetWindowTextW(s);

	// 已连接时禁用切换，避免误操作
	const BOOL enableSelect = comms.IsConnected() ? FALSE : TRUE;
	m_comboMainCom.EnableWindow(enableSelect);
	m_btnMainComRefresh.EnableWindow(enableSelect);
	m_chkMainSimulate.EnableWindow(enableSelect);
}

void C智能机械臂Dlg::OnBnClickedMainSerialRefresh()
{
	RefreshMainComList();
	SaveMainSerialSettings();
	UpdateMainSerialStatusText();
}

void C智能机械臂Dlg::OnBnClickedMainSerialSimulate()
{
	// 切换后端：若已连接先断开
	if (ArmCommsService::Instance().IsConnected())
	{
		ArmCommsService::Instance().Disconnect();
	}
	SaveMainSerialSettings();
	UpdateMainSerialStatusText();
}

void C智能机械臂Dlg::OnCbnSelChangeMainSerialCom()
{
	SaveMainSerialSettings();
}

void C智能机械臂Dlg::OnBnClickedMainSerialConnect()
{
	auto& comms = ArmCommsService::Instance();
	const bool useSim = (m_chkMainSimulate.GetCheck() == BST_CHECKED);

	if (!comms.IsConnected())
	{
		if (useSim)
		{
			comms.ConnectSim();
		}
		else
		{
			CString com;
			m_comboMainCom.GetWindowTextW(com);
			if (com.IsEmpty())
			{
				AfxMessageBox(LoadStrOr(IDS_MSG_SELECT_COM, L"请选择 COM 口。"));
				return;
			}
			if (!comms.ConnectReal(std::wstring(com)))
			{
				CString err(comms.GetLastErrorText().c_str());
				CString msg;
				msg.Format(L"打开串口失败：%s", err.GetString());
				AfxMessageBox(msg);
				return;
			}
		}

		// 连接成功后立刻请求一次全关节回读，尽快建立“当前姿态”估计。
		// 否则用户刚连接就按一下键，会因为回读尚未到达而退化到 homePos，表现为“首次输入跳回竖直复位位”。
		m_motion.RequestReadAllAssigned();

		// 连接后自动归位：只有判定“已就位”后才允许 Jog（用于规避首次动作瞬移）
		StartAutoHomeAfterConnect();
	}
	else
	{
		comms.Disconnect();
		m_autoHomeState = AutoHomeState::Idle;
	}

	SaveMainSerialSettings();
	UpdateMainSerialStatusText();
}

// ===== 自动归位实现 =====
void C智能机械臂Dlg::StartAutoHomeAfterConnect()
{
	auto& comms = ArmCommsService::Instance();

	// 初始位角度解释：与"诊断->FK测试"的默认一致，按 **舵机物理角（deg）** 输入。
	// 说明：J3 在机械定义中绕局部 X 反向旋转（Reference/mechanics.md），因此模型内部 q3 的符号与"物理角"相反。
	//      这里会先把 physicalDeg 转成模型关节角 q(rad)，再用统一的 JointRadToServoPos 映射到舵机位置。
	for (int i = 0; i <= MotionConfig::kJointCount; i++) m_autoHomeTargetPos[(size_t)i] = -1;

	auto DegToRad = [](double d) -> double { return d * (3.14159265358979323846 / 180.0); };
	// 简化后的角度转换：方向由标定数据的 k = (posAtPlusDeg - posAt0Deg) / plusDeg 决定
	auto PhysicalDegToJointRad = [&](int joint, double physicalDeg, double& outRad) -> bool
	{
		if (joint < 1 || joint > KinematicsConfig::kJointCount) return false;
		const auto& jc = m_kc.GetJoint(joint);
		// 应用零位偏置
		const double degZeroAdjusted = physicalDeg - jc.zeroOffsetDeg;
		outRad = DegToRad(degZeroAdjusted);
		return true;
	};

	// 物理角（deg），从 Profile (AutoHome/JxDeg) 读取，若不存在则使用默认值
	// 用户可在"诊断中心->自动归位"页面修改这些角度并保存
	const int qDegPhysicalDefault[6] = { 0, 0, -30, 60, 30, 0 };  // unused, J1, J2, J3, J4, J5
	int qDegPhysical[6];
	qDegPhysical[0] = 0;  // unused
	qDegPhysical[1] = AfxGetApp()->GetProfileInt(L"AutoHome", L"J1Deg", qDegPhysicalDefault[1]);
	qDegPhysical[2] = AfxGetApp()->GetProfileInt(L"AutoHome", L"J2Deg", qDegPhysicalDefault[2]);
	qDegPhysical[3] = AfxGetApp()->GetProfileInt(L"AutoHome", L"J3Deg", qDegPhysicalDefault[3]);
	qDegPhysical[4] = AfxGetApp()->GetProfileInt(L"AutoHome", L"J4Deg", qDegPhysicalDefault[4]);
	qDegPhysical[5] = AfxGetApp()->GetProfileInt(L"AutoHome", L"J5Deg", qDegPhysicalDefault[5]);

	// 计算 AutoHome 的关节角和舵机位置
	ArmKinematics::JointAnglesRad qHome{};
	for (int j = 1; j <= 5; j++)
	{
		double qRad = 0.0;
		// 先计算模型角度（这一步总是应该成功）
		if (PhysicalDegToJointRad(j, qDegPhysical[j], qRad))
		{
			qHome.q[j] = qRad;  // 始终设置关节角，用于 FK
		}
		// 再计算舵机位置（可能失败，但不影响 FK）
		int pos = -1;
		if (ArmKinematics::JointRadToServoPos(m_kc, &m_motion.Config(), j, qRad, pos))
		{
			m_autoHomeTargetPos[(size_t)j] = pos;
		}
	}

	// 模拟串口时：不需要真正移动舵机，但要把 Jog 目标姿态设置为 AutoHome 位置
	if (!comms.IsConnected() || comms.IsSim())
	{
		// 计算 AutoHome 对应的末端姿态，仅用于显示
		const auto poseHome = ArmKinematics::ForwardKinematics(m_kc, qHome);

		// 更新姿态显示
		if (m_staticMainPose.GetSafeHwnd())
		{
			CString s;
			s.Format(L"Pose: (X=%.0f,Y=%.0f,Z=%.0f,p=%.1f) [AutoHome]",
			         poseHome.x_mm, poseHome.y_mm, poseHome.z_mm, poseHome.pitch_deg);
			m_staticMainPose.SetWindowTextW(s);
		}

		m_autoHomeState = AutoHomeState::Ready; // 模拟/未连接不阻塞
		return;
	}

	m_autoHomeStartTick = ::GetTickCount64();
	m_autoHomeLastCmdTick = 0;
	m_autoHomeAttempt = 0;
	m_autoHomeState = AutoHomeState::Moving;

	UpdateMainSerialStatusText();
}

void C智能机械臂Dlg::TickAutoHome()
{
	auto& comms = ArmCommsService::Instance();
	if (!comms.IsConnected() || comms.IsSim()) return;
	if (m_autoHomeState != AutoHomeState::Moving) return;

	const ULONGLONG now = ::GetTickCount64();

	// 1) 周期性下发“强制归位”（首次立即发，之后每 1200ms 重发一次，最多重试 5 次）
	const ULONGLONG kResendMs = 1200;
	const int kMaxAttempts = 5;
	if (m_autoHomeAttempt < kMaxAttempts && (m_autoHomeLastCmdTick == 0 || (now - m_autoHomeLastCmdTick) >= kResendMs))
	{
		std::vector<std::pair<int, int>> jointToPos;
		for (int j = 1; j <= 5; j++)
		{
			const int p = m_autoHomeTargetPos[(size_t)j];
			if (p >= 0) jointToPos.push_back({ j, p });
		}
		ArmCommsService::Instance().ClearMoveQueue();
		const int timeMs = 900;
		(void)m_motion.MoveJointsAbs(jointToPos, timeMs);

		m_autoHomeAttempt++;
		m_autoHomeLastCmdTick = now;
	}

	// 2) 读取读回并判定到位（pos 误差 ±20）
	const int kTol = 20;
	bool allOk = true;
	int checked = 0;
	int okCount = 0;
	for (int j = 1; j <= 5; j++)
	{
		const auto& jc = m_motion.Config().Get(j);
		const int target = m_autoHomeTargetPos[(size_t)j];
		if (target < 0) continue;
		if (jc.servoId < 1 || jc.servoId > 6) continue;

		uint16_t rb = 0;
		DWORD age = 0;
		const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
		if (!ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) || age > staleMs)
		{
			allOk = false;
			continue;
		}
		const int diff = std::abs((int)rb - target);
		checked++;
		if (diff <= kTol) okCount++;
		else allOk = false;
	}

	if (checked > 0 && allOk)
	{
		m_autoHomeState = AutoHomeState::Ready;
		UpdateMainSerialStatusText();
		return;
	}
}

void C智能机械臂Dlg::OnBnClickedExportParams()
{
	CFileDialog dlg(FALSE, L"ini", L"arm-settings.ini",
		OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		L"INI Settings File (*.ini)|*.ini||",
		this);
	if (dlg.DoModal() != IDOK)
	{
		return;
	}

	const CString path = dlg.GetPathName();
	const auto res = SettingsIo::ExportToIni(std::wstring(path.GetString()));
	if (!res.ok)
	{
		CString msg(res.error.c_str());
		AfxMessageBox(msg.IsEmpty() ? L"Export failed." : msg);
		return;
	}
	AfxMessageBox(L"Export succeeded.");
}

void C智能机械臂Dlg::OnBnClickedImportParams()
{
	CFileDialog dlg(TRUE, L"ini", nullptr,
		OFN_HIDEREADONLY | OFN_FILEMUSTEXIST,
		L"INI Settings File (*.ini)|*.ini||",
		this);
	if (dlg.DoModal() != IDOK)
	{
		return;
	}

	const CString path = dlg.GetPathName();
	const auto res = SettingsIo::ImportFromIni(std::wstring(path.GetString()));
	if (!res.ok)
	{
		CString msg(res.error.c_str());
		AfxMessageBox(msg.IsEmpty() ? L"Import failed." : msg);
		return;
	}

	BroadcastSettingsImported();
	AfxMessageBox(L"Import succeeded. Diagnostics pages auto-refreshed.");
}

void C智能机械臂Dlg::OnTimer(UINT_PTR nIDEvent)
{
	if (m_bDestroying)
	{
		return;
	}

	// ===== Jog Tick（20Hz）=====
	if (nIDEvent == 2)
	{
		// 主界面必须持续泵通信队列：否则 Jog 下发只入队不发送（除非打开诊断页）
		ArmCommsService::Instance().Tick();
		// 连接后自动归位：未“已就位”前禁止 Jog（但仍允许读回与归位指令下发）
		TickAutoHome();

		// [闭环基础] 周期性请求舵机位置回读（主界面默认也要读回，否则只能"靠积分猜测姿态"，时间一长必乱）
		// 频率不宜过高：TX 有 throttle + Move 优先级；这里约 5Hz 足够用于 Jog 校正方向/状态。
		{
			static DWORD s_lastReadTick = 0;
			const DWORD now = ::GetTickCount();
			if (ArmCommsService::Instance().IsConnected() && (s_lastReadTick == 0 || (now - s_lastReadTick) >= 200))
			{
				m_motion.RequestReadAllAssigned();
				s_lastReadTick = now;
			}
		}

		// ===== 姿态可视化面板更新 =====
		static std::array<int, MotionConfig::kJointCount + 1> s_lastValidPos{};
		static std::array<bool, MotionConfig::kJointCount + 1> s_hasLastValid{};
		if (m_ctrlPosture.GetSafeHwnd())
		{
			const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);

			// 更新 J1..J5 关节状态
			for (int j = 1; j <= 5; j++)
			{
				const auto& jc = m_motion.Config().Get(j);
				int pos = jc.homePos;
				bool valid = false;

				if (jc.servoId >= 1 && jc.servoId <= 6)
				{
					uint16_t rb = 0;
					DWORD age = 0;
					if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
					{
						pos = (int)rb;
						valid = true;
						s_lastValidPos[(size_t)j] = pos;
						s_hasLastValid[(size_t)j] = true;
					}
					else if (s_hasLastValid[(size_t)j])
					{
						pos = s_lastValidPos[(size_t)j];
						valid = true;
					}
				}

				// 转换为物理角度（简化后：方向由标定数据的 k 值决定）
				double angleDeg = 0.0;
				double qRad = 0.0;
				if (ArmKinematics::ServoPosToJointRad(m_kc, &m_motion.Config(), j, pos, qRad))
				{
					// qRad 是关节角（弧度），转换为度并加回零位偏置
					angleDeg = qRad * (180.0 / 3.14159265358979323846);
					angleDeg += m_kc.GetJoint(j).zeroOffsetDeg;
				}

				m_ctrlPosture.SetJointState(j, pos, angleDeg, valid);

				// 目标位置：键盘 Jog 已移除，仅显示读回
				m_ctrlPosture.SetJointTargetPos(j, -1);
			}

			// 更新夹爪状态 (J6)
			{
				const auto& jc = m_motion.Config().Get(6);
				int pos = jc.homePos;
				bool valid = false;

				if (jc.servoId >= 1 && jc.servoId <= 6)
				{
					uint16_t rb = 0;
					DWORD age = 0;
					if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
					{
						pos = (int)rb;
						valid = true;
						s_lastValidPos[(size_t)6] = pos;
						s_hasLastValid[(size_t)6] = true;
					}
					else if (s_hasLastValid[(size_t)6])
					{
						pos = s_lastValidPos[(size_t)6];
						valid = true;
					}
				}

				// 夹爪开合位置从配置读取（后续可配置化）
				const int gripOpenPos = AfxGetApp()->GetProfileInt(L"SeeAndFetch\\Gripper", L"OpenPos", 640);
				const int gripClosePos = AfxGetApp()->GetProfileInt(L"SeeAndFetch\\Gripper", L"ClosePos", 100);
				m_ctrlPosture.SetGripperState(pos, gripOpenPos, gripClosePos, valid);
				m_ctrlPosture.SetJointTargetPos(6, -1); // 夹爪目前没有目标跟踪
			}

			// 刷新绘图
			m_ctrlPosture.RefreshDisplay();
		}

		// 预览窗口 Resize 防抖：拖拽缩放结束后再 Reset D3D，避免花屏/条纹
		if (m_pendingPreviewResize && m_pMainPreview && m_staticMainVideo.GetSafeHwnd())
		{
			const DWORD now = ::GetTickCount();
			if (now - m_lastSizeTick > 120)
			{
				CRect rc;
				m_staticMainVideo.GetClientRect(&rc);
				// 尺寸过小/最小化时跳过（防止 ResetDevice 进入异常状态）
				if (rc.Width() >= 64 && rc.Height() >= 64)
				{
					m_pMainPreview->ResizeVideo((WORD)rc.Width(), (WORD)rc.Height());
				}
				m_pendingPreviewResize = false;
			}
		}

		const bool grabTestOn = (m_chkSfGrabTest.GetSafeHwnd() && m_chkSfGrabTest.GetCheck() == BST_CHECKED);
		if (grabTestOn)
		{
			GrabTestController::Output gtOut;
			if (m_grabTestEnabled && IsAutoHomeReady())
			{
				GrabTestController::Input gin;
				gin.pKc = &m_kc;
				gin.pMc = &m_motion.Config();
				(void)ArmStateEstimator::Estimate(m_motion, m_kc, gin.arm, nullptr);

				// Vision observation (from VisionService last result)
				{
					const auto vr = m_vision.GetLastResult();
					VisualObservation obs;
					obs.tickMs = vr.tickMs;
					obs.hasTargetPx = vr.hasTargetPx;
					obs.u = vr.u;
					obs.v = vr.v;
					obs.hasDepthMm = vr.hasDepthMm;
					obs.depthMm = vr.depthMm;
					obs.hasConfidence = vr.hasConfidence;
					obs.confidence = vr.confidence;
					gin.obs = obs;

					// 检查观测值时效性：超过 500ms 认为过期，不使用
					const ULONGLONG nowObs = ::GetTickCount64();
					const ULONGLONG obsAge = (vr.tickMs > 0 && nowObs >= vr.tickMs) ? (nowObs - vr.tickMs) : 0;
					const bool obsStale = (obsAge > 500);
					gin.hasObs = (vr.tickMs != 0) && !obsStale;
					if (obsStale)
					{
						gin.obs.hasTargetPx = false;
					}

					gin.hasTimeToFetch = vr.hasGeminiTimeToFetch;
					gin.timeToFetch = vr.geminiTimeToFetch;
				}

				// Frame size
				{
					auto st = m_vision.GetStats();
					gin.frameW = st.frameW;
					gin.frameH = st.frameH;
					if ((gin.frameW == 0 || gin.frameH == 0) && m_pMainPreview && m_bMainPreviewing)
					{
						gin.frameW = (UINT)m_pMainPreview->GetWidth();
						gin.frameH = (UINT)m_pMainPreview->GetHeight();
					}
				}

				GrabTestController::UserCommand gcmd;
				gcmd.start = m_grabTestCmdStart;
				gcmd.cancel = m_grabTestCmdCancel;
				gcmd.eStop = m_grabTestCmdEStop;
				m_grabTestCmdStart = false;
				m_grabTestCmdCancel = false;
				m_grabTestCmdEStop = false;

				(void)m_grabTest.Tick(gin, gcmd, gtOut);

				if (gtOut.requestGeminiReset)
				{
					auto vp = m_vision.GetParams();
					vp.geminiResetSeq++;
					m_vision.SetParams(vp);
				}

				// Execute suggested move
				if (gtOut.hasMove && !gtOut.jointToPos.empty())
				{
					ArmCommsService::Instance().ClearMoveQueue();
					(void)m_motion.MoveJointsAbs(gtOut.jointToPos, gtOut.moveTimeMs);
				}
			}
			else
			{
				gtOut = GrabTestController::Output{};
				gtOut.state = m_grabTest.GetState();
			}

			// GrabTest status
			if (m_staticSfStatus.GetSafeHwnd())
			{
				CString sfs;
				if (!m_grabTestEnabled)
				{
					sfs = L"GT:OFF";
				}
				else
				{
					sfs.Format(L"GT:%d %s", (int)gtOut.state, gtOut.reason.c_str());
				}
				m_staticSfStatus.SetWindowTextW(sfs);
			}
		}
		else
		{
			// ===== See&Fetch 自动流程（关节分步）=====
			SeeAndFetchStateMachine::Output sfOut;
			if (m_sfEnabled && IsAutoHomeReady())
			{
				SeeAndFetchStateMachine::Input sin;
				sin.pKc = &m_kc;
				sin.pMc = &m_motion.Config();
				sin.tool = m_tool;
				(void)ArmStateEstimator::Estimate(m_motion, m_kc, sin.arm, nullptr);

				// Vision observation (from VisionService last result)
				{
					const auto vr = m_vision.GetLastResult();
					VisualObservation obs;
					obs.tickMs = vr.tickMs;
					obs.hasTargetPx = vr.hasTargetPx;
					obs.u = vr.u;
					obs.v = vr.v;
					obs.hasDepthMm = vr.hasDepthMm;
					obs.depthMm = vr.depthMm;
					obs.hasConfidence = vr.hasConfidence;
					obs.confidence = vr.confidence;
					sin.obs = obs;

					// 检查观测值时效性：超过 500ms 认为过期，不使用
					const ULONGLONG nowObs = ::GetTickCount64();
					const ULONGLONG obsAge = (vr.tickMs > 0 && nowObs >= vr.tickMs) ? (nowObs - vr.tickMs) : 0;
					const bool obsStale = (obsAge > 500);
					sin.hasObs = (vr.tickMs != 0) && !obsStale;
					if (obsStale)
					{
						// 观测过期时，强制清除目标标志，避免使用旧的 u,v 位置
						sin.obs.hasTargetPx = false;
					}

					// bbox/track box
					sin.hasBox = vr.hasBox;
					sin.boxW = vr.boxW;
					sin.boxH = vr.boxH;
					sin.boxClassId = vr.classId;
					sin.visionMode = vr.mode;

					// PointPick (gesture selection)
					sin.pickState = vr.pickState;
					sin.hasPickBox = vr.hasPickBox;
					sin.pickBoxX = vr.pickBoxX;
					sin.pickBoxY = vr.pickBoxY;
					sin.pickBoxW = vr.pickBoxW;
					sin.pickBoxH = vr.pickBoxH;
					sin.hasHandLandmarks = vr.hasHandLandmarks;
					sin.handGesture = vr.handGesture;
				}

				// Frame size
				{
					auto st = m_vision.GetStats();
					sin.frameW = st.frameW;
					sin.frameH = st.frameH;
					if ((sin.frameW == 0 || sin.frameH == 0) && m_pMainPreview && m_bMainPreviewing)
					{
						sin.frameW = (UINT)m_pMainPreview->GetWidth();
						sin.frameH = (UINT)m_pMainPreview->GetHeight();
					}
				}

				// Gripper readback (joint 6)
				{
					const auto& jc = m_motion.Config().Get(6);
					if (jc.servoId >= 1 && jc.servoId <= 6)
					{
						uint16_t rb = 0;
						DWORD age = 0;
						if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age))
						{
							sin.hasGripReadback = true;
							sin.gripReadbackPos = (int)rb;
							sin.gripReadbackAgeMs = age;
						}
					}
				}

				// Servo positions snapshot (for initial_pos / terminal_pos)
				{
					sin.hasServoPos = true;
					for (int j = 0; j <= MotionConfig::kJointCount; j++) sin.servoPos[(size_t)j] = -1;
					const MotionConfig& mc = m_motion.Config();
					const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
				for (int j = 1; j <= MotionConfig::kJointCount; j++)
					{
						const auto& jc = mc.Get(j);
						int pos = jc.homePos;
						if (jc.servoId >= 1 && jc.servoId <= 6)
						{
							uint16_t rb = 0;
							DWORD age = 0;
							if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
							{
								pos = (int)rb;
							s_lastValidPos[(size_t)j] = pos;
							s_hasLastValid[(size_t)j] = true;
						}
						else if (s_hasLastValid[(size_t)j])
						{
							pos = s_lastValidPos[(size_t)j];
							}
						}
						sin.servoPos[(size_t)j] = pos;
					}
				}

				SeeAndFetchStateMachine::UserCommand scmd;
				scmd.confirm = m_sfCmdConfirm;
				scmd.cancel = m_sfCmdCancel;
				scmd.eStop = m_sfCmdEStop;
				m_sfCmdConfirm = false;
				m_sfCmdCancel = false;
				m_sfCmdEStop = false;

				(void)m_sf.Tick(sin, scmd, sfOut);

				// Apply PointPick control (target + reset) to VisionService params
				{
					bool needUpdate = false;
					auto vp = m_vision.GetParams();
					if (sfOut.requestPointPickTarget >= 0 && vp.pointPickTarget != sfOut.requestPointPickTarget)
					{
						vp.pointPickTarget = sfOut.requestPointPickTarget;
						needUpdate = true;
					}
					if (sfOut.requestGeminiReset)
					{
						vp.geminiResetSeq++;
						needUpdate = true;
					}
					// 始终禁止“轮廓兜底”，确保候选来源可控（Gemini/红点各自负责）。
					// 红点目标不依赖 Detector/Gemini 候选框。
					if (vp.pointPickDetectorOnly != true)
					{
						vp.pointPickDetectorOnly = true;
						needUpdate = true;
					}
					if (sfOut.requestPointPickReset)
					{
						vp.pointPickResetSeq++;
						needUpdate = true;
					}
					if (needUpdate)
					{
						m_vision.SetParams(vp);
					}
				}

				// Apply vision mode override suggestion (prefer requestVisionMode if set)
				if (sfOut.requestVisionMode >= 0)
				{
					if (!m_sfVisionOverridden)
					{
						m_sfPrevVisionMode = m_visionAlgoMode;
						m_sfVisionOverridden = true;
					}
					const auto m = (VisionService::Mode)sfOut.requestVisionMode;
					m_vision.SetMode(m);
					m_visionAlgoMode = m;
					m_visionAlgoEnabled = true;
				}
				else if (sfOut.requestVisionAruco)
				{
					if (!m_sfVisionOverridden)
					{
						m_sfPrevVisionMode = m_visionAlgoMode;
						m_sfVisionOverridden = true;
					}
					m_vision.SetMode(VisionService::Mode::Aruco);
					m_visionAlgoMode = VisionService::Mode::Aruco;
					m_visionAlgoEnabled = true;
				}
				else if (m_sfVisionOverridden && m_sf.GetState() == SeeAndFetchStateMachine::State::Idle)
				{
					m_vision.SetMode(m_sfPrevVisionMode);
					m_visionAlgoMode = m_sfPrevVisionMode;
					m_sfVisionOverridden = false;
				}

				// Execute suggested move (directly via MotionController; Jog stays untouched)
				if (sfOut.pauseTracking)
				{
					ArmCommsService::Instance().ClearMoveQueue();
				}
				else if (sfOut.hasMove && !sfOut.jointToPos.empty())
				{
					ArmCommsService::Instance().ClearMoveQueue();
					(void)m_motion.MoveJointsAbs(sfOut.jointToPos, sfOut.moveTimeMs);
				}
			}
			else
			{
				// disabled: keep idle and restore vision override if needed
				if (m_sfVisionOverridden)
				{
					m_vision.SetMode(m_sfPrevVisionMode);
					m_visionAlgoMode = m_sfPrevVisionMode;
					m_sfVisionOverridden = false;
				}
				sfOut = SeeAndFetchStateMachine::Output{};
				sfOut.state = m_sf.GetState();
			}

			// See&Fetch status
			if (m_staticSfStatus.GetSafeHwnd())
			{
				CString sfs;
				if (!m_sfEnabled)
				{
					sfs = L"SF:OFF";
				}
				else
				{
					// keep it short; long reason is still available in logs/HUD later
					sfs.Format(L"SF:%d %s", (int)sfOut.state, sfOut.reason.c_str());
				}
				m_staticSfStatus.SetWindowTextW(sfs);
			}
		}

		// 更新 UI pose 文本（读回估计）
		ArmStateEstimator::ArmState st{};
		(void)ArmStateEstimator::Estimate(m_motion, m_kc, st, nullptr);
		const auto pose = st.joint5PoseBase;
		CString s;
		s.Format(L"Pose: (X=%.0f,Y=%.0f,Z=%.0f,p=%.1f)",
		         pose.x_mm, pose.y_mm, pose.z_mm, pose.pitch_deg);
		m_staticMainPose.SetWindowTextW(s);

		// 更新 HUD 叠加层状态（渲染线程会读取）
		{
			unsigned fps = 0, sinceMs = 0;
			theApp.GetSerialSendStats(fps, sinceMs);
			KinematicsOverlayService::Instance().UpdateSerialStats(fps, sinceMs);
			KinematicsOverlayService::Instance().UpdateJog(false, 0.0, 0.0, pose, true, L"");
			KinematicsOverlayService::Instance().UpdateVisualServo(false, false, false, 0, 0.0, 0.0, 0.0, L"");
		}

		CDialogEx::OnTimer(nIDEvent);
		return;
	}

	if (nIDEvent == 1 && m_bMainPreviewing)
	{
		if (m_pMainPreview)
		{
			const DWORD now = ::GetTickCount();
			const DWORD elapsed = now - m_lastTickCount;
			if (elapsed > 0)
			{
				const UINT currentFrameCount = m_pMainPreview->GetFrameCount();
				const UINT framesDelta = currentFrameCount - m_lastFrameCount;
				m_estimatedFps = (float)framesDelta * 1000.0f / (float)elapsed;
				m_lastFrameCount = currentFrameCount;
				m_lastTickCount = now;
			}
		}
		else
		{
			// Test source：保持一个可读的假 FPS
			m_estimatedFps = 30.0f;
		}
		UpdateMainCamStatusText();
	}

	if (nIDEvent == 1)
	{
		UpdateMainSerialStatusText();
	}

	CDialogEx::OnTimer(nIDEvent);
}

void C智能机械臂Dlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	UNREFERENCED_PARAMETER(nSBCode);
	UNREFERENCED_PARAMETER(nPos);
	UNREFERENCED_PARAMETER(pScrollBar);
	// 目前无需实时处理：OnTimer 会读取滑条。
	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void C智能机械臂Dlg::OnStnClickedMainVideo()
{
	// 点击画面设置目标点：用于在没有视觉模块时快速验证 VisualServo 闭环
	if (!m_staticMainVideo.GetSafeHwnd())
	{
		return;
	}

	CPoint pt;
	::GetCursorPos(&pt);
	m_staticMainVideo.ScreenToClient(&pt);

	CRect rc;
	m_staticMainVideo.GetClientRect(&rc);
	if (!rc.PtInRect(pt))
	{
		return;
	}

	// 将“窗口点击坐标”映射到“帧坐标”（与 CopyLastRgb/DrawDevice/HUD 坐标一致）
	int fw = 0, fh = 0;
	if (m_pMainPreview && m_bMainPreviewing)
	{
		fw = (int)m_pMainPreview->GetWidth();
		fh = (int)m_pMainPreview->GetHeight();
	}
	if (fw > 0 && fh > 0)
	{
		const CRect dst = ComputeLetterboxRect(fw, fh, rc);
		if (!dst.PtInRect(pt))
		{
			return; // 点击在黑边区域
		}
		const double nx = (double)(pt.x - dst.left) / (double)std::max(1, dst.Width());
		const double ny = (double)(pt.y - dst.top) / (double)std::max(1, dst.Height());
		pt.x = (int)std::lround(nx * (double)fw);
		pt.y = (int)std::lround(ny * (double)fh);
		// CPoint::x/y 是 LONG 类型；fw/fh 是 int。
		// 这里统一用 LONG，避免 std::min/max 因 LONG/int 混用导致模板推导失败（C2672）。
		const LONG xMin = 0;
		const LONG yMin = 0;
		const LONG xMax = (LONG)fw - 1;
		const LONG yMax = (LONG)fh - 1;
		pt.x = (LONG)std::max(xMin, std::min(xMax, pt.x));
		pt.y = (LONG)std::max(yMin, std::min(yMax, pt.y));
	}

	VisualObservation obs;
	obs.tickMs = ::GetTickCount64();
	obs.hasTargetPx = true;
	obs.u = (double)pt.x;
	obs.v = (double)pt.y;
	obs.hasConfidence = true;
	obs.confidence = 1.0;
}

void C智能机械臂Dlg::OnDestroy()
{
	m_bDestroying = true;

	// 停止视觉线程（避免其继续访问 Preview / VS 对象）
	m_vision.Stop();

	if (m_timerFps)
	{
		KillTimer(m_timerFps);
		m_timerFps = 0;
	}
	// Jog tick timer
	KillTimer(2);

	StopMainPreview();

	// 释放枚举到的激活对象
	for (auto& d : m_mainDevices)
	{
		if (d.pActivate)
		{
			d.pActivate->Release();
			d.pActivate = nullptr;
		}
	}
	m_mainDevices.clear();

	// 保存主界面设置（覆盖层/串口/窗口大小）
	SaveMainOverlaySettings();
	SaveMainSerialSettings();
	SaveMainWindowPlacement();

	CDialogEx::OnDestroy();
}

void C智能机械臂Dlg::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);

	// 最小化时不做布局/不触发 D3D Reset（避免花屏）
	if (nType == SIZE_MINIMIZED)
	{
		m_pendingPreviewResize = false;
		return;
	}

	if (!GetSafeHwnd() || cx <= 0 || cy <= 0)
	{
		return;
	}
	if (!m_staticMainVideo.GetSafeHwnd())
	{
		return;
	}

	// ===== DPI 缩放因子（基准 96 DPI）=====
	// 获取当前窗口的 DPI，按比例缩放所有布局常量
	HDC hdc = ::GetDC(GetSafeHwnd());
	const int dpiX = ::GetDeviceCaps(hdc, LOGPIXELSX);
	::ReleaseDC(GetSafeHwnd(), hdc);
	const double dpiScale = (double)dpiX / 96.0;
	auto scale = [dpiScale](int v) -> int { return (int)std::lround((double)v * dpiScale); };

	// 统一尺寸（按 DPI 缩放，避免 Win11/高缩放下按钮过小）
	const int margin = scale(12);    // 边距
	const int topBarH = scale(40);   // 顶栏高度
	const int btnH = scale(22);
	const int chkH = scale(18);
	const int comboH = scale(200);   // ComboBox 下拉区高度（不随 DPI 变化太多）
	const int bottomBtnH = scale(24);

	const int bottomBtnY = std::max(topBarH + margin, cy - margin - bottomBtnH);
	const int bottomAreaTop = bottomBtnY - margin;

	// ===== 右侧面板宽度：按比例分配，但有最小可读宽度 =====
	// 右侧面板占窗口宽度的 ~30%，但不低于 300px（DPI 缩放后），不高于 420px（DPI 缩放后）
	const int rightMinW = scale(300);
	const int rightMaxW = scale(420);
	int rightW = cx * 30 / 100;
	if (rightW < rightMinW) rightW = rightMinW;
	if (rightW > rightMaxW) rightW = rightMaxW;
	int rightX = cx - margin - rightW;
	if (rightX < margin) rightX = margin;

	// 用 DeferWindowPos 降低闪烁/减少"缩小再放大"绘制异常
	HDWP hdwp = BeginDeferWindowPos(50);
	auto defer = [&](CWnd& w, int x, int y, int wdt, int hgt)
	{
		if (!w.GetSafeHwnd()) return;
		hdwp = DeferWindowPos(hdwp, w.GetSafeHwnd(), nullptr, x, y, wdt, hgt, SWP_NOZORDER | SWP_NOACTIVATE);
	};
	auto deferId = [&](int id, int x, int y, int wdt, int hgt)
	{
		CWnd* p = GetDlgItem(id);
		if (!p || !p->GetSafeHwnd()) return;
		hdwp = DeferWindowPos(hdwp, p->GetSafeHwnd(), nullptr, x, y, wdt, hgt, SWP_NOZORDER | SWP_NOACTIVATE);
	};

	// ===== 顶栏（垂直居中）=====
	int x = margin;
	const int yTop = (topBarH - btnH) / 2 + scale(2);
	const int yChk = (topBarH - chkH) / 2 + scale(2);

	deferId(IDC_MAIN_LBL_CAMERA, x, yChk + scale(2), scale(40), chkH);
	x += scale(42);
	defer(m_comboMainCamera, x, yTop - 1, scale(180), comboH);
	x += scale(190);
	defer(m_btnMainCamRefresh, x, yTop, scale(45), btnH);
	x += scale(50);
	defer(m_btnMainCamStart, x, yTop, scale(45), btnH);
	x += scale(50);
	defer(m_btnMainCamStop, x, yTop, scale(45), btnH);
	x += scale(55); // 分隔

	defer(m_chkMainMirror, x, yChk, scale(45), chkH);
	x += scale(50);
	deferId(IDC_MAIN_LBL_ROTATION, x, yChk + scale(2), scale(35), chkH);
	x += scale(38);
	defer(m_comboMainRotation, x, yTop - 1, scale(55), comboH);
	x += scale(60);
	defer(m_chkMainCrosshair, x, yChk, scale(55), chkH);
	x += scale(60);
	defer(m_chkMainGrid, x, yChk, scale(55), chkH);

	// 右侧状态（锚定右上角）
	const int camInfoW = scale(120);
	const int camStatusW = scale(90);
	defer(m_staticMainCamInfo, cx - margin - camInfoW, yChk, camInfoW, chkH);
	defer(m_staticMainCamStatus, cx - margin - camInfoW - camStatusW - scale(5), yChk, camStatusW, chkH);

	// ===== 左侧区域：视频 =====
	// 视频区域（左侧，占满剩余高度）
	const int videoX = margin;
	const int videoY = topBarH + margin;
	const int videoW = std::max(scale(64), rightX - margin - margin);
	const int videoH = std::max(scale(64), bottomAreaTop - videoY);
	
	defer(m_staticMainVideo, videoX, videoY, videoW, videoH);

	// ===== 右侧面板（按 DPI 缩放）=====
	const int panelTop = topBarH + margin;
	const int serialH = scale(80);
	const int vsH = scale(105);      // 视觉跟随组
	const int visionH = scale(65);   // 视觉识别组
	const int statusH = scale(65);   // 状态组
	const int spacing = scale(8);    // 组间距
	const int innerPad = scale(12);  // 组内边距

	// 1. 串口组
	defer(m_grpMainSerial, rightX, panelTop, rightW, serialH);
	deferId(IDC_MAIN_LBL_SERIAL_COM, rightX + innerPad, panelTop + scale(22), scale(40), chkH);
	const int comComboW = std::max(scale(70), (rightW - scale(180)) / 2);
	defer(m_comboMainCom, rightX + innerPad + scale(42), panelTop + scale(20), comComboW, comboH);
	const int btnRefreshX = rightX + innerPad + scale(42) + comComboW + scale(5);
	defer(m_btnMainComRefresh, btnRefreshX, panelTop + scale(20), scale(45), btnH);
	defer(m_btnMainComConnect, btnRefreshX + scale(50), panelTop + scale(20), scale(45), btnH);
	defer(m_chkMainSimulate, rightX + innerPad, panelTop + scale(50), scale(55), chkH);
	defer(m_staticMainSerialStatus, rightX + innerPad + scale(60), panelTop + scale(50), rightW - innerPad * 2 - scale(60), chkH);

	// 2. 视觉跟随组
	const int vsY = panelTop + serialH + spacing;
	defer(m_grpMainVs, rightX, vsY, rightW, vsH);
	
	// Row 1: VS启用 + 模式
	defer(m_chkVsEnable, rightX + innerPad, vsY + scale(22), scale(55), chkH);
	deferId(IDC_MAIN_LBL_VS_MODE, rightX + innerPad + scale(58), vsY + scale(22), scale(35), chkH);
	defer(m_comboVsMode, rightX + innerPad + scale(95), vsY + scale(20), rightW - innerPad * 2 - scale(95), comboH);

	// Row 2: 覆盖 + 仅测试
	defer(m_chkVsOverride, rightX + innerPad, vsY + scale(45), scale(50), chkH);
	defer(m_chkVsNoDrive, rightX + innerPad + scale(55), vsY + scale(45), scale(55), chkH);

	// Row 3: 推进
	deferId(IDC_MAIN_LBL_VS_ADVANCE, rightX + innerPad, vsY + scale(66), scale(35), chkH);
	defer(m_sliderVsAdvance, rightX + innerPad + scale(38), vsY + scale(64), rightW - innerPad * 2 - scale(38), scale(18));

	// Row 4: 状态
	defer(m_staticVsStatus, rightX + innerPad, vsY + scale(84), rightW - innerPad * 2, chkH);

	// 3. 视觉识别组
	const int visionY = vsY + vsH + spacing;
	defer(m_grpMainVision, rightX, visionY, rightW, visionH);
	defer(m_chkVisionProcEnable, rightX + innerPad, visionY + scale(22), scale(65), chkH);
	defer(m_staticVisionAlgo, rightX + innerPad + scale(68), visionY + scale(22), scale(35), chkH);
	defer(m_comboVisionAlgo, rightX + innerPad + scale(105), visionY + scale(20), rightW - innerPad * 2 - scale(105), comboH);

	// 4. 状态组 + 姿态组
	const int panelBottom = bottomAreaTop;
	const int statusTop = visionY + visionH + spacing;

	// 状态组固定高度
	const int statusY = statusTop;
	defer(m_grpMainStatus, rightX, statusY, rightW, statusH);
	defer(m_staticMainPose, rightX + innerPad, statusY + scale(20), rightW - innerPad * 2, chkH);
	defer(m_btnEmergencyStop, rightX + innerPad, statusY + scale(40), rightW - innerPad * 2, scale(20));

	// 5. 姿态组（获得全部剩余空间）
	const int postureY = statusY + statusH + spacing;
	const int postureH = std::max(scale(100), panelBottom - postureY);
	if (postureY < panelBottom)
	{
		defer(m_grpMainPosture, rightX, postureY, rightW, postureH);
		defer(m_ctrlPosture, rightX + scale(5), postureY + scale(15), rightW - scale(10), std::max(scale(10), postureH - scale(20)));
	}

	// ===== 底栏按钮（按 DPI 缩放 + See&Fetch 控件自适应）=====
	const int botBtnW = scale(85);
	const int botBtnGap = scale(8);
	int bx = margin;
	deferId(IDC_BTN_DIAGNOSTICS, bx, bottomBtnY, botBtnW, bottomBtnH);
	bx += botBtnW + botBtnGap;
	deferId(IDC_BTN_EXPORT_PARAMS, bx, bottomBtnY, botBtnW, bottomBtnH);
	bx += botBtnW + botBtnGap;
	deferId(IDC_BTN_IMPORT_PARAMS, bx, bottomBtnY, botBtnW, bottomBtnH);
	bx += botBtnW + botBtnGap + scale(10); // 多留一点间距

	// See&Fetch 控件组
	const int sfChkW = scale(70);
	const int sfGrabChkW = scale(80);
	const int sfBtnW = scale(40);
	deferId(IDC_MAIN_CHECK_SF_ENABLE, bx, bottomBtnY + scale(3), sfChkW, chkH);
	bx += sfChkW + scale(4);
	deferId(IDC_MAIN_CHECK_SF_GRABTEST, bx, bottomBtnY + scale(3), sfGrabChkW, chkH);
	bx += sfGrabChkW + botBtnGap;
	deferId(IDC_MAIN_BTN_SF_START, bx, bottomBtnY, sfBtnW, bottomBtnH);
	bx += sfBtnW + scale(4);
	deferId(IDC_MAIN_BTN_SF_CANCEL, bx, bottomBtnY, sfBtnW, bottomBtnH);
	bx += sfBtnW + scale(4);
	deferId(IDC_MAIN_BTN_SF_ESTOP, bx, bottomBtnY, sfBtnW, bottomBtnH);
	bx += sfBtnW + botBtnGap;
	// SF 状态文本：占据剩余空间（右侧留出退出按钮位置）
	const int cancelBtnW = scale(75);
	const int sfStatusW = std::max(scale(60), cx - bx - margin - cancelBtnW - botBtnGap);
	deferId(IDC_MAIN_STATIC_SF_STATUS, bx, bottomBtnY + scale(4), sfStatusW, chkH);

	// 退出按钮（靠右）
	deferId(IDCANCEL, cx - margin - cancelBtnW, bottomBtnY, cancelBtnW, bottomBtnH);

	EndDeferWindowPos(hdwp);

	// 触发预览 resize（防抖，在 OnTimer 里真正 Reset D3D）
	if (m_pMainPreview)
	{
		m_pendingPreviewResize = true;
		m_lastSizeTick = ::GetTickCount();
	}
}

void C智能机械臂Dlg::OnGetMinMaxInfo(MINMAXINFO* lpMMI)
{
	// 最小窗口尺寸（按 DPI 缩放）：避免控件挤压到不可用
	if (lpMMI && GetSafeHwnd())
	{
		HDC hdc = ::GetDC(GetSafeHwnd());
		const int dpiX = ::GetDeviceCaps(hdc, LOGPIXELSX);
		::ReleaseDC(GetSafeHwnd(), hdc);
		const double dpiScale = (double)dpiX / 96.0;

		// 基准尺寸（96 DPI 下）：右侧面板最小 300px + 左侧视频最小 350px + 边距
		lpMMI->ptMinTrackSize.x = (LONG)std::lround(900.0 * dpiScale);
		lpMMI->ptMinTrackSize.y = (LONG)std::lround(650.0 * dpiScale);
	}
	CDialogEx::OnGetMinMaxInfo(lpMMI);
}

void C智能机械臂Dlg::RefreshMainDeviceList()
{
	// 清理旧设备
	for (auto& d : m_mainDevices)
	{
		if (d.pActivate)
		{
			d.pActivate->Release();
			d.pActivate = nullptr;
		}
	}
	m_mainDevices.clear();
	m_comboMainCamera.ResetContent();

	IMFAttributes* pAttributes = nullptr;
	IMFActivate** ppDevices = nullptr;
	UINT32 count = 0;

	HRESULT hr = MFCreateAttributes(&pAttributes, 1);
	if (SUCCEEDED(hr))
	{
		hr = pAttributes->SetGUID(
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
	}
	if (SUCCEEDED(hr))
	{
		hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
	}
	if (SUCCEEDED(hr))
	{
		for (UINT32 i = 0; i < count; i++)
		{
			WCHAR* szFriendlyName = nullptr;
			UINT32 cchName = 0;
			hr = ppDevices[i]->GetAllocatedString(
				MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
				&szFriendlyName,
				&cchName);
			if (SUCCEEDED(hr) && szFriendlyName)
			{
				DeviceInfo info;
				info.friendlyName = szFriendlyName;
				info.pActivate = ppDevices[i];
				ppDevices[i]->AddRef();
				m_mainDevices.push_back(info);
				m_comboMainCamera.AddString(szFriendlyName);
				CoTaskMemFree(szFriendlyName);
			}
			ppDevices[i]->Release();
		}
		CoTaskMemFree(ppDevices);
	}

	SafeRelease(&pAttributes);

	if (m_comboMainCamera.GetCount() > 0)
	{
		m_comboMainCamera.SetCurSel(0);
	}
}

void C智能机械臂Dlg::UpdateMainCamStatusText()
{
	CString status;
	if (!m_bMainPreviewing)
	{
		status = L"未预览";
	}
	else
	{
		if (m_pMainPreview)
		{
			status.Format(L"预览中 - %.1f FPS", m_estimatedFps);
		}
		else
		{
			status.Format(L"Test source - %.1f FPS", m_estimatedFps);
		}
	}
	m_staticMainCamStatus.SetWindowTextW(status);

	if (m_pMainPreview && m_bMainPreviewing)
	{
		const UINT w = m_pMainPreview->GetWidth();
		const UINT h = m_pMainPreview->GetHeight();
		CString info;
		info.Format(L"Resolution: %u x %u", w, h);
		m_staticMainCamInfo.SetWindowTextW(info);
	}
	else
	{
		m_staticMainCamInfo.SetWindowTextW(L"Resolution: N/A");
	}

	// 按钮启用状态
	m_btnMainCamStart.EnableWindow(!m_bMainPreviewing && m_comboMainCamera.GetCount() > 0);
	m_btnMainCamStop.EnableWindow(m_bMainPreviewing);
}

void C智能机械臂Dlg::StartMainPreview()
{
	if (m_bMainPreviewing)
	{
		return;
	}

	const int sel = m_comboMainCamera.GetCurSel();
	if (sel < 0 || sel >= (int)m_mainDevices.size())
	{
		m_staticMainCamStatus.SetWindowTextW(L"[错误] 未选择相机");
		return;
	}

	HWND hVideo = m_staticMainVideo.GetSafeHwnd();
	HWND hEvent = GetSafeHwnd();
	ASSERT(hVideo != NULL);
	ASSERT(hEvent != NULL);

	HRESULT hr = CPreview::CreateInstance(hVideo, hEvent, &m_pMainPreview);
	if (FAILED(hr))
	{
		// fallback：test source
		m_pMainPreview = nullptr;
		m_vision.SetPreview(nullptr);
		m_bMainPreviewing = true;
		m_estimatedFps = 0.0f;
		UpdateMainCamStatusText();
		return;
	}

	hr = m_pMainPreview->SetDevice(m_mainDevices[sel].pActivate);
	if (FAILED(hr))
	{
		SafeRelease(&m_pMainPreview);
		m_pMainPreview = nullptr;
		m_vision.SetPreview(nullptr);
		m_bMainPreviewing = true;
		m_estimatedFps = 0.0f;
		UpdateMainCamStatusText();
		return;
	}

	m_bMainPreviewing = true;
	m_lastFrameCount = 0;
	m_pMainPreview->ResetFrameCount();
	m_lastTickCount = ::GetTickCount();

	ApplyMainOverlaySettings();
	m_vision.SetPreview(m_pMainPreview);
	UpdateMainCamStatusText();
}

void C智能机械臂Dlg::StopMainPreview()
{
	// 先切断视觉线程的预览指针，避免 use-after-free
	m_vision.SetPreview(nullptr);

	if (m_pMainPreview)
	{
		m_pMainPreview->CloseDevice();
		SafeRelease(&m_pMainPreview);
	}
	m_bMainPreviewing = false;
	m_estimatedFps = 0.0f;
}

void C智能机械臂Dlg::LoadMainOverlaySettings()
{
	CWinApp* app = AfxGetApp();
	const int mirror = app->GetProfileInt(L"CameraOverlay", L"Mirror", 0);
	const int cross = app->GetProfileInt(L"CameraOverlay", L"Crosshair", 0);
	const int grid = app->GetProfileInt(L"CameraOverlay", L"Grid", 0);
	const int rot = app->GetProfileInt(L"CameraOverlay", L"Rotation", 0); // 0/90/180/270

	m_chkMainMirror.SetCheck(mirror ? BST_CHECKED : BST_UNCHECKED);
	m_chkMainCrosshair.SetCheck(cross ? BST_CHECKED : BST_UNCHECKED);
	m_chkMainGrid.SetCheck(grid ? BST_CHECKED : BST_UNCHECKED);

	int sel = 0;
	if (rot == 90) sel = 1;
	else if (rot == 180) sel = 2;
	else if (rot == 270) sel = 3;
	m_comboMainRotation.SetCurSel(sel);
}

void C智能机械臂Dlg::SaveMainOverlaySettings()
{
	CWinApp* app = AfxGetApp();
	app->WriteProfileInt(L"CameraOverlay", L"Mirror", (m_chkMainMirror.GetCheck() == BST_CHECKED) ? 1 : 0);
	app->WriteProfileInt(L"CameraOverlay", L"Crosshair", (m_chkMainCrosshair.GetCheck() == BST_CHECKED) ? 1 : 0);
	app->WriteProfileInt(L"CameraOverlay", L"Grid", (m_chkMainGrid.GetCheck() == BST_CHECKED) ? 1 : 0);

	int rot = 0;
	const int sel = m_comboMainRotation.GetCurSel();
	if (sel == 1) rot = 90;
	else if (sel == 2) rot = 180;
	else if (sel == 3) rot = 270;
	app->WriteProfileInt(L"CameraOverlay", L"Rotation", rot);
}

void C智能机械臂Dlg::ApplyMainOverlaySettings()
{
	if (!m_pMainPreview)
	{
		SaveMainOverlaySettings();
		return;
	}

	VideoOverlaySettings s;
	s.mirrorHorizontal = (m_chkMainMirror.GetCheck() == BST_CHECKED);
	s.showCrosshair = (m_chkMainCrosshair.GetCheck() == BST_CHECKED);
	s.showReferenceLines = (m_chkMainGrid.GetCheck() == BST_CHECKED);

	const int sel = m_comboMainRotation.GetCurSel();
	if (sel == 1) s.rotation = VideoRotation::Rotate90;
	else if (sel == 2) s.rotation = VideoRotation::Rotate180;
	else if (sel == 3) s.rotation = VideoRotation::Rotate270;
	else s.rotation = VideoRotation::None;

	m_pMainPreview->SetOverlaySettings(s);
	SaveMainOverlaySettings();
}

namespace
{
	struct BroadcastCtx
	{
		UINT msg = 0;
		WPARAM wParam = 0;
		LPARAM lParam = 0;
	};

	BOOL CALLBACK EnumChildProc(HWND hWnd, LPARAM lParam)
	{
		auto* ctx = reinterpret_cast<BroadcastCtx*>(lParam);
		if (!ctx) return FALSE;
		::PostMessageW(hWnd, ctx->msg, ctx->wParam, ctx->lParam);
		::EnumChildWindows(hWnd, EnumChildProc, lParam);
		return TRUE;
	}

	BOOL CALLBACK EnumThreadProc(HWND hWnd, LPARAM lParam)
	{
		auto* ctx = reinterpret_cast<BroadcastCtx*>(lParam);
		if (!ctx) return FALSE;
		::PostMessageW(hWnd, ctx->msg, ctx->wParam, ctx->lParam);
		::EnumChildWindows(hWnd, EnumChildProc, lParam);
		return TRUE;
	}
}

void C智能机械臂Dlg::BroadcastSettingsImported()
{
	BroadcastCtx ctx;
	ctx.msg = WM_APP_SETTINGS_IMPORTED;
	ctx.wParam = 0;
	ctx.lParam = 0;
	::EnumThreadWindows(::GetCurrentThreadId(), EnumThreadProc, reinterpret_cast<LPARAM>(&ctx));
}

void C智能机械臂Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void C智能机械臂Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR C智能机械臂Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

