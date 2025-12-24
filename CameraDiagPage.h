// CameraDiagPage.h : Camera diagnostics property page
// - Device enumeration
// - Current stream info (format/resolution/framerate estimation)
// - Screenshot, restart preview, device lost notification

#pragma once

#include "preview.h"
#include <vector>
#include <string>

class CCameraDiagPage : public CPropertyPage
{
    DECLARE_DYNAMIC(CCameraDiagPage)

public:
    CCameraDiagPage();
    virtual ~CCameraDiagPage();

#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_CAMERA_DIAG_PAGE };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    virtual void OnOK() {}
    virtual void OnCancel() {}

    DECLARE_MESSAGE_MAP()

    afx_msg void OnBnClickedRefresh();
    afx_msg void OnBnClickedStart();
    afx_msg void OnBnClickedStop();
    afx_msg void OnBnClickedScreenshot();
    afx_msg void OnBnClickedShowSettings();
    afx_msg void OnBnClickedResetSettings();
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnDestroy();
    afx_msg LRESULT OnPreviewError(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);
    afx_msg void OnSize(UINT nType, int cx, int cy);

private:
    void RefreshDeviceList();
    void UpdateStatusText();
    void AppendLog(const CString& line);
    void StartPreview();
    void StopPreview();
    void LoadOverlaySettings();
    void SaveOverlaySettings();
    void ApplyOverlaySettings();

    // Controls
    CComboBox   m_comboDevices;
    CButton     m_btnRefresh;
    CButton     m_btnStart;
    CButton     m_btnStop;
    CButton     m_btnScreenshot;
    CButton     m_chkMirror;
    CButton     m_chkCrosshair;
    CButton     m_chkRefLines;
    CComboBox   m_comboRotation;
    CStatic     m_staticVideo;
    CStatic     m_staticStatus;
    CStatic     m_staticInfo;
    CEdit       m_editLog;

    // Device enumeration
    struct DeviceInfo
    {
        std::wstring friendlyName;
        IMFActivate* pActivate;
    };
    std::vector<DeviceInfo> m_devices;

    // Preview state
    CPreview*   m_pPreview;
    bool        m_previewing;
    bool        m_destroying;   // Flag to prevent UI access during destruction

    // Frame rate tracking
    UINT        m_lastFrameCount;
    DWORD       m_lastTickCount;
    float       m_estimatedFps;

    UINT_PTR    m_timerId;
};

