
// 智能机械臂Dlg.h: 头文件
//

#pragma once

#include <string>
#include <vector>

#include <afxcmn.h> // CSliderCtrl

#include "Resource.h"
#include "preview.h"
#include "MotionController.h"
#include "KinematicsConfig.h"
#include "JogController.h"
#include "JogPadCtrl.h"
#include "VisualServoController.h"
#include "VisionService.h"

// C智能机械臂Dlg 对话框
class C智能机械臂Dlg : public CDialogEx
{
// 构造
public:
	C智能机械臂Dlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MY_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnBnClickedDiagnostics();
	afx_msg void OnBnClickedExportParams();
	afx_msg void OnBnClickedImportParams();
	afx_msg void OnBnClickedMainCamRefresh();
	afx_msg void OnBnClickedMainCamStart();
	afx_msg void OnBnClickedMainCamStop();
	afx_msg void OnBnClickedEmergencyStop();
	afx_msg void OnBnClickedMainSerialRefresh();
	afx_msg void OnBnClickedMainSerialConnect();
	afx_msg void OnBnClickedMainSerialSimulate();
	afx_msg void OnCbnSelChangeMainSerialCom();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnStnClickedMainVideo();
	afx_msg void OnCbnSelChangeVisionAlgo();
	afx_msg void OnBnClickedVisionProcEnable();
	afx_msg void OnBnClickedVsNoDrive();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnGetMinMaxInfo(MINMAXINFO* lpMMI);
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);
	DECLARE_MESSAGE_MAP()

private:
	void BroadcastSettingsImported();
	void LoadVisionSettingsFromProfile();
	void SyncVisionAlgoUiFromState();

	// ===== 主界面：相机预览 =====
private:
	struct DeviceInfo
	{
		std::wstring friendlyName;
		IMFActivate* pActivate = nullptr;
	};

	void RefreshMainDeviceList();
	void UpdateMainCamStatusText();
	void StartMainPreview();
	void StopMainPreview();
	void LoadMainOverlaySettings();
	void SaveMainOverlaySettings();
	void ApplyMainOverlaySettings();

	// ===== 主界面：串口快捷入口 =====
	void RefreshMainComList();
	void UpdateMainSerialStatusText();
	void LoadMainSerialSettings();
	void SaveMainSerialSettings() const;

	// 主窗口：大小/位置持久化
	void LoadMainWindowPlacement();
	void SaveMainWindowPlacement() const;

private:
	// Controls（主界面）
	CComboBox m_comboMainCamera;
	CButton   m_btnMainCamRefresh;
	CButton   m_btnMainCamStart;
	CButton   m_btnMainCamStop;
	CStatic   m_staticMainCamStatus;
	CStatic   m_staticMainCamInfo;
	CStatic   m_staticMainVideo;
	CButton   m_chkMainMirror;
	CButton   m_chkMainCrosshair;
	CButton   m_chkMainGrid;
	CComboBox m_comboMainRotation;

	CButton   m_grpMainSerial;
	CComboBox m_comboMainCom;
	CButton   m_btnMainComRefresh;
	CButton   m_chkMainSimulate;
	CButton   m_btnMainComConnect;
	CStatic   m_staticMainSerialStatus;

	// Visual servo (vision-follow)
	CButton   m_grpMainVs;
	CButton   m_chkVsEnable;
	CComboBox m_comboVsMode;
	CButton   m_chkVsOverride;
	CButton   m_chkVsNoDrive; // 仅测试：不允许视觉输出驱动 Jog
	CSliderCtrl m_sliderVsAdvance;
	CStatic   m_staticVsStatus;

	// Vision algorithm (recognition): controls in .rc
	CButton   m_grpMainVision;
	CButton   m_chkVisionProcEnable; // 识别启用（与 VS Enable 解耦）
	CStatic   m_staticVisionAlgo;
	CComboBox m_comboVisionAlgo;
	bool      m_visionAlgoEnabled = true;                // false => 手动(点击)，不运行视觉识别
	VisionService::Mode m_visionAlgoMode = VisionService::Mode::Auto;

	CButton   m_grpMainJog;
	CButton   m_grpMainStatus;

	CJogPadCtrl m_staticMainJogPad;
	CSliderCtrl m_sliderSpeedMm;
	CSliderCtrl m_sliderSpeedPitch;
	CStatic   m_staticMainPose;
	CButton   m_btnEmergencyStop;

	std::vector<DeviceInfo> m_mainDevices;
	CPreview* m_pMainPreview = nullptr;
	bool m_bMainPreviewing = false;
	bool m_bDestroying = false;
	UINT_PTR m_timerFps = 0;
	UINT m_lastFrameCount = 0;
	DWORD m_lastTickCount = 0;
	float m_estimatedFps = 0.0f;

	// 预览窗口缩放防抖：避免频繁 Reset D3D 导致花屏/条纹
	bool m_pendingPreviewResize = false;
	DWORD m_lastSizeTick = 0;

	// ===== 主界面：运动与Jog（后续HUD也会读取这些状态）=====
	MotionController m_motion;
	KinematicsConfig m_kc;
	JogController m_jog;

	// 视觉伺服：将视觉观测转换为 Jog 输入（未来视觉协同）
	VisualServoController m_vs;

	// 视觉线程：从预览拉帧并产出 VisualObservation（先提供基础验证管线）
	VisionService m_vision;
};
