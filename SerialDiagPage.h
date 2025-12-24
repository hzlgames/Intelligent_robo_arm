#pragma once

#include <afxcmn.h>
#include <afxdlgs.h>

#include <vector>

#include "ArmCommsService.h"

// Serial diagnostics page:
// - enumerate COM ports
// - connect/disconnect
// - send sample protocol frames
// - monitor TX/RX as hex + parsed summaries
// - export log
class CSerialDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CSerialDiagPage)

public:
	CSerialDiagPage();
	virtual ~CSerialDiagPage();

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);

	afx_msg void OnBnClickedRefresh();
	afx_msg void OnBnClickedConnect();
	afx_msg void OnBnClickedClearLog();
	afx_msg void OnBnClickedExportLog();
	afx_msg void OnBnClickedSendMove();
	afx_msg void OnBnClickedSendReadAll();
	afx_msg void OnBnClickedMoveMinus();
	afx_msg void OnBnClickedMovePlus();
	afx_msg void OnBnClickedMoveSend();
	afx_msg void OnBnClickedReadId();
	afx_msg void OnBnClickedSetMin();
	afx_msg void OnBnClickedSetMax();
	afx_msg void OnBnClickedShowSettings();
	afx_msg void OnBnClickedClearSettings();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnCheckSimulate();
	afx_msg void OnDestroy();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()

private:
	void RefreshComList();
	void AppendLogLine(const CString& line);
	void UpdateStatusText();
	void DisconnectAll();

	int GetIntFromEdit(const CEdit& edit, int fallback) const;
	void SetIntToEdit(CEdit& edit, int v);
	void LoadManualControlsFromProfile();
	void SaveManualControlsToProfile();
	void LoadServoLimitsFromProfile();
	void SaveServoLimitToProfile(uint8_t id, bool isMin, int v);
	int ApplySafeClamp(uint8_t id, int pos, bool* clamped);

private:
	CComboBox m_comboCom;
	CEdit m_editLog;
	CButton m_checkSim;
	CStatic m_staticStatus;

	// Manual move / calibration widgets
	CEdit m_editMoveId;
	CEdit m_editMovePos;
	CEdit m_editMoveStep;
	CEdit m_editMoveTime;

	bool m_useSim = true;
	std::vector<CString> m_logLines;
	int m_logToken = 0;

	// Safe limits (ids 1..6)
	int m_minPos[7] = { 0 };
	int m_maxPos[7] = { 0 };
};


