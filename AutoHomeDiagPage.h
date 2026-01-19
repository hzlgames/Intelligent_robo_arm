#pragma once

#include <afxdlgs.h>

// 自动归位诊断页：配置连接后的初始姿态（物理角度）
// 用户可以调整 J1..J5 的初始角度，保存到 Profile（AutoHome/J1Deg..J5Deg）
// 连接成功后，主界面会先移动到这些角度，判定到位后才允许 Jog/SeeAndFetch 操作
class CAutoHomeDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CAutoHomeDiagPage)

public:
	CAutoHomeDiagPage();
	virtual ~CAutoHomeDiagPage();

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);

	afx_msg void OnBnClickedLoadDefaults();
	afx_msg void OnBnClickedSave();
	afx_msg void OnDestroy();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()

private:
	void LoadFromProfile();
	void SaveToProfile();
	void SetDefaultValues();
	void UpdateUiFromValues();
	void UpdateValuesFromUi();

	int GetIntFromEdit(const CEdit& edit, int fallback) const;
	void SetIntToEdit(CEdit& edit, int v);

private:
	// UI 控件
	CEdit m_editJ1Deg;
	CEdit m_editJ2Deg;
	CEdit m_editJ3Deg;
	CEdit m_editJ4Deg;
	CEdit m_editJ5Deg;

	// 当前值（物理角度，单位：deg）
	int m_j1Deg = 0;
	int m_j2Deg = -30;
	int m_j3Deg = 60;
	int m_j4Deg = 30;
	int m_j5Deg = 0;
};

