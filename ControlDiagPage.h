#pragma once

#include <afxcmn.h>
#include <afxdlgs.h>

// Control/throttle diagnostics page (for send throttling observability)
class CControlDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CControlDiagPage)

public:
	CControlDiagPage();
	virtual ~CControlDiagPage();

protected:
	virtual void DoDataExchange(CDataExchange* pDX);
	virtual BOOL OnInitDialog();

	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnDestroy();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()

private:
	void AppendLog(const CString& line);
	void UpdateUi();

private:
	CSliderCtrl m_slider;
	CStatic m_txtThrottle;
	CStatic m_txtSendFps;
	CStatic m_txtLastSend;
	CEdit m_log;

	UINT_PTR m_timerId = 0;
	DWORD m_lastTick = 0;
	unsigned m_sendCount = 0;
	unsigned m_lastSendMs = 0;
	int m_throttleMs = 50;
};


