#include "pch.h"

#include "DiagnosticsSheet.h"

#include "SerialDiagPage.h"
#include "CameraDiagPage.h"

#include "ControlDiagPage.h"
#include "MotionDiagPage.h"
#include "KinematicsDiagPage.h"
#include "KinTestDiagPage.h"
#include "SeeAndFetchDiagPage.h"
#include "Resource.h"

CDiagnosticsSheet::CDiagnosticsSheet(CWnd* pParentWnd)
	: CPropertySheet(IDS_DIAG_TITLE, pParentWnd)
{
	// Remove Apply and Help buttons for a cleaner, beginner-friendly dialog.
	m_psh.dwFlags |= PSH_NOAPPLYNOW;
	m_psh.dwFlags &= ~PSH_HASHELP;

	m_serial = new CSerialDiagPage();
	m_camera = new CCameraDiagPage();
	m_control = new CControlDiagPage();
	m_motion = new CMotionDiagPage();
	m_kinematics = new CKinematicsDiagPage();
	m_kinTest = new CKinTestDiagPage();
	m_seeFetch = new CSeeAndFetchDiagPage();

	AddPage(m_serial);
	AddPage(m_camera);
	AddPage(m_control);
	AddPage(m_motion);
	AddPage(m_kinematics);
	AddPage(m_kinTest);
	AddPage(m_seeFetch);
}

CDiagnosticsSheet::~CDiagnosticsSheet()
{
	// Pages are owned by this sheet; delete explicitly.
	delete m_serial;
	delete m_camera;
	delete m_control;
	delete m_motion;
	delete m_kinematics;
	delete m_kinTest;
	delete m_seeFetch;
}

BOOL CDiagnosticsSheet::OnInitDialog()
{
	BOOL bResult = CPropertySheet::OnInitDialog();

	// 获取当前窗口位置和大小
	CRect rcSheet;
	GetWindowRect(&rcSheet);
	
	int nOldHeight = rcSheet.Height();
	int nNewHeight = nOldHeight * 2 / 3;
	int nDelta = nNewHeight - nOldHeight;

	// 1. 调整 Sheet 窗口高度
	SetWindowPos(NULL, 0, 0, rcSheet.Width(), nNewHeight, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);

	// 2. 移动底部按钮 (OK, Cancel)
	const int btnIds[] = { IDOK, IDCANCEL, ID_APPLY_NOW, IDHELP };
	for (int id : btnIds)
	{
		CWnd* pBtn = GetDlgItem(id);
		if (pBtn)
		{
			CRect rcBtn;
			pBtn->GetWindowRect(&rcBtn);
			ScreenToClient(&rcBtn);
			pBtn->SetWindowPos(NULL, rcBtn.left, rcBtn.top + nDelta, 0, 0, SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
		}
	}

	// 3. 调整 Tab Control 高度
	CWnd* pTab = GetTabControl();
	if (pTab)
	{
		CRect rcTab;
		pTab->GetWindowRect(&rcTab);
		ScreenToClient(&rcTab);
		pTab->SetWindowPos(NULL, 0, 0, rcTab.Width(), rcTab.Height() + nDelta, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
	}

	// 4. 调整当前活动 Page 的高度
	CPropertyPage* pPage = GetActivePage();
	if (pPage)
	{
		CRect rcPage;
		pPage->GetWindowRect(&rcPage);
		ScreenToClient(&rcPage);
		pPage->SetWindowPos(NULL, 0, 0, rcPage.Width(), rcPage.Height() + nDelta, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
	}

	// 居中显示
	CenterWindow();

	return bResult;
}
