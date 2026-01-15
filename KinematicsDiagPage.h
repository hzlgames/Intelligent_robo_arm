#pragma once

#include <afxcmn.h>
#include <afxdlgs.h>

#include <string>

#include "KinematicsConfig.h"

// Kinematics diagnostics page:
// - edit link lengths (L_base/L_arm1/L_arm2/L_wrist/L_cam)
// - edit tool offsets (J5->Cam, Cam->Grip) in Cam coordinate
class CKinematicsDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CKinematicsDiagPage)

public:
	CKinematicsDiagPage();
	virtual ~CKinematicsDiagPage();

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);

	afx_msg void OnBnClickedLoad();
	afx_msg void OnBnClickedSaveApply();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()

private:
	void LoadFromProfileToUi();
	bool SaveFromUiToProfile(std::wstring& outWhy);
	void InitJointMapControls();

	int GetIntFromEdit(const CEdit& edit, int fallback) const;
	void SetIntToEdit(CEdit& edit, int v);

private:
	CEdit m_editLBase;
	CEdit m_editLArm1;
	CEdit m_editLArm2;
	CEdit m_editLWrist;
	CEdit m_editLCam;

	CEdit m_editJ5CamX;
	CEdit m_editJ5CamY;
	CEdit m_editJ5CamZ;
	CEdit m_editCamGripX;
	CEdit m_editCamGripY;
	CEdit m_editCamGripZ;

	// 关节角度映射参数（J1..J5）
	bool m_bJointMapInited = false;
	CEdit m_editJPos0[KinematicsConfig::kJointCount + 1]{};
	CEdit m_editJPosPlus[KinematicsConfig::kJointCount + 1]{};
	CEdit m_editJPlusDeg[KinematicsConfig::kJointCount + 1]{};
	CEdit m_editJZeroOffMdeg[KinematicsConfig::kJointCount + 1]{};
	CButton m_checkJPhysInv[KinematicsConfig::kJointCount + 1]{};
};


