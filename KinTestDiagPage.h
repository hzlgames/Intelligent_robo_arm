#pragma once

#include <afxcmn.h>
#include <afxdlgs.h>

#include <string>

// FK test page:
// - user enters J1..J5 degrees
// - compute forward kinematics pose (x,y,z,pitch)
// - optionally show predicted servo positions (pos 0..1000) for calibration sanity-check
class CKinTestDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CKinTestDiagPage)

public:
	CKinTestDiagPage();
	virtual ~CKinTestDiagPage();

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);

	afx_msg void OnBnClickedCalc();
	afx_msg void OnBnClickedExec();
	afx_msg void OnBnClickedClear();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	DECLARE_MESSAGE_MAP()

private:
	void Compute(bool doExec);
	double GetDoubleFromEdit(const CEdit& edit, double fallback) const;
	void SetDoubleToEdit(CEdit& edit, double v);
	void SetOutputText(const std::wstring& text);

private:
	CEdit m_editJ1;
	CEdit m_editJ2;
	CEdit m_editJ3;
	CEdit m_editJ4;
	CEdit m_editJ5;
	CEdit m_editTime;
	CButton m_checkPhysical;
	CStatic m_staticOut;
};


