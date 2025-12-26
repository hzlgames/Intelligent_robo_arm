
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
#include "Resource.h"
#include "KinematicsOverlayService.h"

#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <cmath>
#include <algorithm>
#include <climits>

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

	// ===== 主界面：Jog区域控件（先搭框架） =====
	DDX_Control(pDX, IDC_MAIN_GROUP_JOG, m_grpMainJog);
	DDX_Control(pDX, IDC_MAIN_GROUP_STATUS, m_grpMainStatus);
	DDX_Control(pDX, IDC_MAIN_STATIC_JOGPAD, m_staticMainJogPad);
	DDX_Control(pDX, IDC_MAIN_SLIDER_SPEED_MM, m_sliderSpeedMm);
	DDX_Control(pDX, IDC_MAIN_SLIDER_SPEED_PITCH, m_sliderSpeedPitch);
	DDX_Control(pDX, IDC_MAIN_STATIC_POSE, m_staticMainPose);
	DDX_Control(pDX, IDC_MAIN_BTN_EMERGENCY_STOP, m_btnEmergencyStop);
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

	// Jog 速度滑条（先提供UI；后续 JogController 读这些参数）
	m_sliderSpeedMm.SetRange(1, 200);       // 1~200 mm/s
	m_sliderSpeedMm.SetTicFreq(10);
	m_sliderSpeedMm.SetPos(50);
	m_sliderSpeedPitch.SetRange(1, 180);    // 1~180 deg/s
	m_sliderSpeedPitch.SetTicFreq(10);
	m_sliderSpeedPitch.SetPos(30);

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
		m_comboVisionAlgo.SetCurSel(1); // 默认 Auto（后续 LoadVisionSettingsFromProfile 会覆盖）
	}

	// ===== 主界面：运动与Jog 初始化 =====
	m_motion.LoadConfig();
	m_kc.LoadAll();
	m_jog.Bind(&m_motion, &m_kc);

	// ===== VisionService 初始化（独立视觉线程）=====
	m_vision.SetVisualServo(&m_vs);
	m_vision.SetPreview(m_pMainPreview); // 可能为空；StartMainPreview 后会更新
	LoadVisionSettingsFromProfile();
	// 识别启用由 profile/checkbox 控制；默认开启，但与 VS Enable 解耦
	m_vision.SetEnabled(m_chkVisionProcEnable.GetSafeHwnd() && (m_chkVisionProcEnable.GetCheck() == BST_CHECKED) && m_visionAlgoEnabled);
	m_vision.Start();

	// 以“当前姿态估计（优先回读，否则home）”的 FK 结果作为初始目标，避免一开始就不可达
	{
		ArmKinematics::JointAnglesRad qCur{};
		for (int j = 0; j <= ArmKinematics::kJointCount; j++) qCur.q[j] = 0.0;
		for (int j = 1; j <= ArmKinematics::kJointCount; j++)
		{
			const auto& jc = m_motion.Config().Get(j);
			int pos = jc.homePos;
			if (jc.servoId >= 1 && jc.servoId <= 6)
			{
				uint16_t rb = 0;
				if (ArmCommsService::Instance().GetLastReadPos((uint8_t)jc.servoId, rb))
				{
					pos = (int)rb;
				}
			}
			double rad = 0.0;
			if (!ArmKinematics::ServoPosToJointRad(m_kc, &m_motion.Config(), j, pos, rad))
			{
				rad = 0.0;
			}
			qCur.q[j] = rad;
		}
		const auto pose0 = ArmKinematics::ForwardKinematics(m_kc, qCur);
		m_jog.SetTargetPose(pose0);
		CString s;
		s.Format(L"Pose: (X=%.0f,Y=%.0f,Z=%.0f,p=%.1f)", pose0.x_mm, pose0.y_mm, pose0.z_mm, pose0.pitch_deg);
		m_staticMainPose.SetWindowTextW(s);
	}

	// FPS 计时器（每秒刷新一次）
	m_timerFps = SetTimer(1, 1000, nullptr);

	// Jog Tick（20Hz）
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

LRESULT C智能机械臂Dlg::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	if (m_bDestroying) return 0;
	LoadVisionSettingsFromProfile();
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
	const int x = app->GetProfileInt(L"MainWindow", L"X", INT_MIN);
	const int y = app->GetProfileInt(L"MainWindow", L"Y", INT_MIN);
	const int w = app->GetProfileInt(L"MainWindow", L"W", 0);
	const int h = app->GetProfileInt(L"MainWindow", L"H", 0);
	if (x == INT_MIN || y == INT_MIN || w <= 0 || h <= 0)
	{
		return;
	}

	// 简单保护：避免离谱数据导致窗口不可见
	if (x < -2000 || y < -2000 || x > 20000 || y > 20000)
	{
		return;
	}
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
	}
	else
	{
		comms.Disconnect();
	}

	SaveMainSerialSettings();
	UpdateMainSerialStatusText();
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
		// MotionController 也依赖 Tick（脚本/读回请求等）
		m_motion.Tick();

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

		// 读取键盘输入（WASD：平面；Q/E：上下；R/F：Pitch）
		// 说明：这里用 GetAsyncKeyState 读取“按住状态”，符合“按住持续移动”的手感。
		JogController::InputState in{};
		in.x = 0.0;
		in.y = 0.0;
		in.z = 0.0;
		in.pitch = 0.0;

		const auto keyDown = [](int vk) -> bool {
			return (::GetAsyncKeyState(vk) & 0x8000) != 0;
			};

		if (keyDown('A')) in.x -= 1.0;
		if (keyDown('D')) in.x += 1.0;
		if (keyDown('W')) in.y += 1.0;
		if (keyDown('S')) in.y -= 1.0;
		if (keyDown('E')) in.z += 1.0;
		if (keyDown('Q')) in.z -= 1.0;
		if (keyDown('R')) in.pitch += 1.0;
		if (keyDown('F')) in.pitch -= 1.0;

		// 鼠标摇杆：优先用于 X/Y（更接近“虚拟摇杆”手感）
		if (m_staticMainJogPad.GetSafeHwnd() && m_staticMainJogPad.IsActive())
		{
			in.x = m_staticMainJogPad.GetX();
			in.y = m_staticMainJogPad.GetY();
		}

		in.active = (std::fabs(in.x) + std::fabs(in.y) + std::fabs(in.z) + std::fabs(in.pitch)) > 0.0;

		// 同步滑条参数到 JogController（实时调参）
		JogController::Params p = m_jog.GetParams();
		double speedMul = 1.0;
		if (keyDown(VK_SHIFT))
		{
			speedMul = 2.0; // Shift 加速
		}
		p.speedMmPerSec = (double)m_sliderSpeedMm.GetPos() * speedMul;
		p.pitchDegPerSec = (double)m_sliderSpeedPitch.GetPos() * speedMul;
		m_jog.SetParams(p);

		// ===== 视觉跟随（Visual Servo）=====
		const bool vsEnable = (m_chkVsEnable.GetSafeHwnd() && m_chkVsEnable.GetCheck() == BST_CHECKED);
		m_vs.SetEnabled(vsEnable);
		// 视觉线程默认一直跑；这里用“识别启用”开关控制是否产出结果（与 VS Enable 解耦）。
		// 说明：当识别模式选“手动(点击)”时，m_visionAlgoEnabled=false，让你用鼠标点画面生成观测，
		// 不会被最亮点/色块等算法立刻覆盖。
		const bool procEnable = (m_chkVisionProcEnable.GetSafeHwnd() && m_chkVisionProcEnable.GetCheck() == BST_CHECKED);
		m_vision.SetEnabled(procEnable && m_visionAlgoEnabled);

		// Mode
		if (m_comboVsMode.GetSafeHwnd())
		{
			const int sel = m_comboVsMode.GetCurSel();
			if (sel == 0) m_vs.SetMode(VisualServoMode::CenterTarget);
			else if (sel == 1) m_vs.SetMode(VisualServoMode::FollowRay);
			else m_vs.SetMode(VisualServoMode::LookAndMove);
		}

		// Intrinsics：优先使用“帧分辨率”（与 CopyLastRgb / HUD 坐标一致）。
		// 若无预览对象（test source），退化为使用窗口大小。
		{
			int vw = 0, vh = 0;
			if (m_pMainPreview && m_bMainPreviewing)
			{
				vw = (int)m_pMainPreview->GetWidth();
				vh = (int)m_pMainPreview->GetHeight();
			}
			if ((vw <= 0 || vh <= 0) && m_staticMainVideo.GetSafeHwnd())
			{
				CRect rc;
				m_staticMainVideo.GetClientRect(&rc);
				vw = rc.Width();
				vh = rc.Height();
			}
			if (vw > 0 && vh > 0)
			{
				CameraIntrinsics K;
				K.valid = true;
				K.fx = (double)vw; // 粗略近似：先保证闭环能跑；后续由标定替换
				K.fy = (double)vh;
				K.cx = (double)vw * 0.5;
				K.cy = (double)vh * 0.5;
				m_vs.SetCameraIntrinsics(K);
			}
		}

		// Params: sync speed ceiling with main sliders
		{
			auto vp = m_vs.GetParams();
			vp.maxSpeedMmPerSec = p.speedMmPerSec;
			vp.maxPitchDegPerSec = p.pitchDegPerSec;
			vp.raySpeedMmPerSec = std::max(10.0, p.speedMmPerSec); // 沿指向推进速度默认跟随线速度
			m_vs.SetParams(vp);
		}

		// Advance command: [-1,1]
		double adv = 0.0;
		if (m_sliderVsAdvance.GetSafeHwnd())
		{
			adv = (double)m_sliderVsAdvance.GetPos() / 100.0;
		}
		m_vs.SetAdvanceCommand(adv);

		// Compute output once
		VisualServoOutput vsOut;
		m_vs.ComputeOutput(vsOut);

		// Decide whether to apply visual output to jog
		const bool overrideManual = (m_chkVsOverride.GetSafeHwnd() && m_chkVsOverride.GetCheck() == BST_CHECKED);
		const bool noDrive = (m_chkVsNoDrive.GetSafeHwnd() && m_chkVsNoDrive.GetCheck() == BST_CHECKED);
		const bool useVs = vsEnable && !noDrive && (overrideManual || !in.active);

		// manual input is the default
		m_jog.SetInputState(in);
		if (useVs)
		{
			JogController::InputState vin{};
			vin.active = vsOut.active;
			vin.x = vsOut.x;
			vin.y = vsOut.y;
			vin.z = vsOut.z;
			vin.pitch = vsOut.pitch;
			m_jog.SetInputState(vin);
		}

		// UI status (short)
		if (m_staticVsStatus.GetSafeHwnd())
		{
			CString s;
			if (!vsEnable)
			{
				s = L"VS:OFF";
			}
			else if (!useVs && in.active)
			{
				s = L"VS:suppressed (manual)";
			}
			else if (noDrive)
			{
				s = L"VS:TEST (no drive)";
			}
			else
			{
				const auto st = m_vision.GetStats();
				const wchar_t* algo = L"Manual";
				if (m_visionAlgoEnabled)
				{
					switch (m_visionAlgoMode)
					{
					case VisionService::Mode::Auto: algo = L"Auto"; break;
					case VisionService::Mode::BrightestPoint: algo = L"Brightest"; break;
					case VisionService::Mode::ColorTrack: algo = L"Color"; break;
					case VisionService::Mode::Aruco: algo = L"Aruco"; break;
					case VisionService::Mode::Detector: algo = L"Detector"; break;
						case VisionService::Mode::HandSticker: algo = L"Sticker"; break;
						case VisionService::Mode::HandLandmarks: algo = L"HandLM"; break;
					default: algo = L"Auto"; break;
					}
				}
				const wchar_t* cv = SMARTARM_HAS_OPENCV_HEADERS ? L"cv=Y" : L"cv=N";
				// 把 cv/proc/algo 放到前面：即使静态文本被截断，也能一眼看到 OpenCV 是否生效
				s.Format(L"VS:%s %s %s p=%.1f e(%.0f,%.0f) adv=%.2f",
				         vsOut.active ? L"ON" : L"IDLE",
				         cv,
				         algo,
				         st.procFps,
				         vsOut.errU, vsOut.errV,
				         adv);
			}
			m_staticVsStatus.SetWindowTextW(s);
		}

		std::wstring why;
		const bool ok = m_jog.Tick(why);
		if (!ok)
		{
			// 失败时停止继续发送，但不急停（允许用户松手恢复/调整）
			m_jog.Stop();
		}

		// 更新 UI pose 文本（先显示 target，后续 HUD 会更丰富）
		const auto pose = m_jog.GetTargetPose();
		CString s;
		s.Format(L"Pose: (X=%.0f,Y=%.0f,Z=%.0f,p=%.1f)%s%s",
		         pose.x_mm, pose.y_mm, pose.z_mm, pose.pitch_deg,
		         in.active ? L" [Jog]" : L"",
		         (!ok && !why.empty()) ? L" [IK失败]" : L"");
		m_staticMainPose.SetWindowTextW(s);

		// 更新 HUD 叠加层状态（渲染线程会读取）
		{
			unsigned fps = 0, sinceMs = 0;
			theApp.GetSerialSendStats(fps, sinceMs);
			KinematicsOverlayService::Instance().UpdateSerialStats(fps, sinceMs);
			KinematicsOverlayService::Instance().UpdateJog(in.active,
			                                              (m_staticMainJogPad.IsActive() ? m_staticMainJogPad.GetX() : in.x),
			                                              (m_staticMainJogPad.IsActive() ? m_staticMainJogPad.GetY() : in.y),
			                                              pose,
			                                              ok,
			                                              why);
			KinematicsOverlayService::Instance().UpdateVisualServo(
				vsEnable,
				useVs,
				vsOut.active,
				(int)m_vs.GetMode(),
				vsOut.errU,
				vsOut.errV,
				adv,
				vsOut.reason);
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
	m_vs.UpdateObservation(obs);
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

	// 统一尺寸（比资源默认稍大，避免 Win11/DPI 下按钮过小）
	const int margin = 15;   // 增加边距：更“松”
	const int topBarH = 50;  // 增加顶栏高度：更“松”
	const int btnH = 24;
	const int chkH = 20;
	const int comboH = 200;
	const int bottomBtnH = 26;

	const int bottomBtnY = std::max(topBarH + margin, cy - margin - bottomBtnH);
	const int bottomAreaTop = bottomBtnY - margin;

	// 右侧面板固定宽度（与资源一致略放大）
	const int rightW = 280; // 进一步加宽面板：给右侧更多空间
	int rightX = cx - margin - rightW;
	if (rightX < margin) rightX = margin;

	// 用 DeferWindowPos 降低闪烁/减少“缩小再放大”绘制异常
	HDWP hdwp = BeginDeferWindowPos(48);
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
	const int yTop = (topBarH - btnH) / 2 + 2;
	const int yChk = (topBarH - chkH) / 2 + 2;

	deferId(IDC_MAIN_LBL_CAMERA, x, yChk + 2, 40, chkH);
	x += 42;
	defer(m_comboMainCamera, x, yTop - 1, 220, comboH);
	x += 230;
	defer(m_btnMainCamRefresh, x, yTop, 50, btnH);
	x += 60;
	defer(m_btnMainCamStart, x, yTop, 50, btnH);
	x += 60;
	defer(m_btnMainCamStop, x, yTop, 50, btnH);
	x += 70; // 分隔

	defer(m_chkMainMirror, x, yChk, 50, chkH);
	x += 55;
	deferId(IDC_MAIN_LBL_ROTATION, x, yChk + 2, 40, chkH);
	x += 42;
	defer(m_comboMainRotation, x, yTop - 1, 60, comboH);
	x += 70;
	defer(m_chkMainCrosshair, x, yChk, 60, chkH);
	x += 70;
	defer(m_chkMainGrid, x, yChk, 60, chkH);

	// 右侧状态（锚定右上角）
	defer(m_staticMainCamInfo, cx - margin - 130, yChk, 130, chkH);
	defer(m_staticMainCamStatus, cx - margin - 130 - 100, yChk, 95, chkH);

	// ===== 视频区域 =====
	const int videoX = margin;
	const int videoY = topBarH + margin; // 增加与顶栏间距
	const int videoW = std::max(64, rightX - margin - videoX);
	const int videoH = std::max(64, bottomAreaTop - videoY);
	defer(m_staticMainVideo, videoX, videoY, videoW, videoH);

	// ===== 右侧面板 =====
	const int panelTop = topBarH + margin;
	const int serialH = 90;
	const int vsH = 115;     // 视觉跟随组：两行复选框 + 模式 + 推进 + 状态
	const int visionH = 75;  // 视觉识别组：识别启用 + 算法下拉
	const int statusH = 90;
	const int spacing = 12;

	// 串口组
	defer(m_grpMainSerial, rightX, panelTop, rightW, serialH);
	deferId(IDC_MAIN_LBL_SERIAL_COM, rightX + 15, panelTop + 25, 40, chkH);
	defer(m_comboMainCom, rightX + 60, panelTop + 23, 90, comboH);
	defer(m_btnMainComRefresh, rightX + 160, panelTop + 23, 50, btnH);
	defer(m_btnMainComConnect, rightX + 215, panelTop + 23, 50, btnH);
	defer(m_chkMainSimulate, rightX + 15, panelTop + 58, 60, chkH);
	defer(m_staticMainSerialStatus, rightX + 85, panelTop + 58, rightW - 100, chkH);

	// 视觉跟随组（只放“跟随相关”控件）
	const int vsY = panelTop + serialH + spacing;
	defer(m_grpMainVs, rightX, vsY, rightW, vsH);
	
	// Row 1: VS启用 + 模式
	defer(m_chkVsEnable, rightX + 15, vsY + 25, 65, chkH);
	deferId(IDC_MAIN_LBL_VS_MODE, rightX + 85, vsY + 25, 40, chkH);
	defer(m_comboVsMode, rightX + 125, vsY + 23, rightW - 140, comboH);

	// Row 2: 覆盖 + 仅测试
	defer(m_chkVsOverride, rightX + 15, vsY + 50, 55, chkH);
	defer(m_chkVsNoDrive, rightX + 75, vsY + 50, 60, chkH);

	// Row 3: 推进
	deferId(IDC_MAIN_LBL_VS_ADVANCE, rightX + 15, vsY + 74, 40, chkH);
	defer(m_sliderVsAdvance, rightX + 60, vsY + 72, rightW - 75, 20);

	// Row 4: 状态
	defer(m_staticVsStatus, rightX + 15, vsY + 94, rightW - 30, chkH);

	// 视觉识别组（只放“识别相关”控件）
	const int visionY = vsY + vsH + spacing;
	defer(m_grpMainVision, rightX, visionY, rightW, visionH);
	defer(m_chkVisionProcEnable, rightX + 15, visionY + 25, 70, chkH);
	defer(m_staticVisionAlgo, rightX + 90, visionY + 25, 40, chkH);
	defer(m_comboVisionAlgo, rightX + 130, visionY + 23, rightW - 145, comboH);

	// Jog 组（JogPad 强制正方形）
	const int jogY = visionY + visionH + spacing;
	// 计算可用高度，决定正方形边长（既要“正方形”，也要避免被挤爆）
	const int panelH = std::max(0, bottomAreaTop - panelTop);
	const int availForJog = std::max(0, panelH - serialH - spacing - vsH - spacing - visionH - spacing - statusH - spacing);

	const int padInnerMargin = 20;
	const int sliderAreaH = 70; // 两条滑条+标签的预算高度
	const int maxPadByWidth = rightW - padInnerMargin * 2;
	const int maxPadByHeight = std::max(60, availForJog - 25 - 10 - sliderAreaH - 10);
	const int padSide = std::max(120, std::min(maxPadByWidth, maxPadByHeight));

	const int jogGroupH = 25 + padSide + 10 + sliderAreaH + 10;
	defer(m_grpMainJog, rightX, jogY, rightW, jogGroupH);

	const int padX = rightX + padInnerMargin;
	const int padY = jogY + 25;
	defer(m_staticMainJogPad, padX, padY, padSide, padSide);

	const int sY1 = padY + padSide + 15;
	deferId(IDC_MAIN_LBL_SPEED_MM, rightX + 15, sY1, 90, chkH);
	defer(m_sliderSpeedMm, rightX + 110, sY1 - 2, rightW - 125, 20);
	deferId(IDC_MAIN_LBL_PITCH_SPEED, rightX + 15, sY1 + 30, 90, chkH);
	defer(m_sliderSpeedPitch, rightX + 110, sY1 + 28, rightW - 125, 20);

	// 状态组（放在 Jog 组下方）
	const int statusY = jogY + jogGroupH + spacing;
	defer(m_grpMainStatus, rightX, statusY, rightW, statusH);
	defer(m_staticMainPose, rightX + 15, statusY + 25, rightW - 30, chkH);
	defer(m_btnEmergencyStop, rightX + 15, statusY + 50, rightW - 30, 30);

	// ===== 底栏按钮 =====
	deferId(IDC_BTN_DIAGNOSTICS, margin, bottomBtnY, 100, bottomBtnH);
	deferId(IDC_BTN_EXPORT_PARAMS, margin + 110, bottomBtnY, 100, bottomBtnH);
	deferId(IDC_BTN_IMPORT_PARAMS, margin + 220, bottomBtnY, 100, bottomBtnH);
	deferId(IDCANCEL, cx - margin - 100, bottomBtnY, 100, bottomBtnH);

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
	// 最小窗口尺寸：避免控件挤压到不可用
	if (lpMMI)
	{
		lpMMI->ptMinTrackSize.x = 1024; // 右侧加宽 + 顶栏更松，避免顶部控件拥挤
		lpMMI->ptMinTrackSize.y = 720;  // 右侧 JogPad 正方形需要更多垂直空间
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

