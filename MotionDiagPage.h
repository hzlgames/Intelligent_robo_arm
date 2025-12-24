#pragma once

#include <afxcmn.h>
#include <afxdlgs.h>

#include <vector>

#include "MotionController.h"

// Motion diagnostics / control page:
// - joint calibration (servoId/min/max/home/invert)
// - joint move + Home
// - ReadAll (0x15)
// - demo keyframe script playback
// - shows shared comms logs
class CMotionDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CMotionDiagPage)

public:
	CMotionDiagPage();
	virtual ~CMotionDiagPage();

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);

	afx_msg void OnBnClickedRefreshCom();
	afx_msg void OnBnClickedConnect();
	afx_msg void OnCheckSimulate();

	afx_msg void OnCbnSelChangeJoint();
	afx_msg void OnBnClickedLoadAll();
	afx_msg void OnBnClickedSaveAll();
	afx_msg void OnBnClickedImportLegacyLimits();

	afx_msg void OnBnClickedMoveSelected();
	afx_msg void OnBnClickedHome();
	afx_msg void OnBnClickedReadAll();

	afx_msg void OnBnClickedDemoPlay();
	afx_msg void OnBnClickedStop();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnDestroy();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()

private:
	void RefreshComList();
	void UpdateStatusText();
	void AppendLogLine(const CString& line);

	void LoadSelectedJointToUi();
	void SaveSelectedJointFromUi();
	int GetIntFromEdit(const CEdit& edit, int fallback) const;
	void SetIntToEdit(CEdit& edit, int v);

	std::vector<MotionController::Keyframe> BuildDemoScript() const;

private:
	CComboBox m_comboCom;
	CButton m_checkSim;
	CStatic m_staticStatus;

	CComboBox m_comboJoint;
	CEdit m_editServoId;
	CEdit m_editMin;
	CEdit m_editMax;
	CEdit m_editHome;
	CButton m_checkInvert;

	CEdit m_editTarget;
	CEdit m_editTime;

	CButton m_checkLoop;
	CEdit m_editLog;

	bool m_useSim = true;
	int m_logToken = 0;
	UINT_PTR m_timerId = 0;

	MotionController m_motion;
	std::vector<CString> m_logLines;
};


