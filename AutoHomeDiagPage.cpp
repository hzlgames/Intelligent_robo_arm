#include "pch.h"

#include "AutoHomeDiagPage.h"

#include "Resource.h"
#include "AppMessages.h"

IMPLEMENT_DYNAMIC(CAutoHomeDiagPage, CPropertyPage)

CAutoHomeDiagPage::CAutoHomeDiagPage()
	: CPropertyPage(IDD_PAGE_AUTOHOME, IDS_TAB_AUTOHOME)
{
}

CAutoHomeDiagPage::~CAutoHomeDiagPage()
{
}

void CAutoHomeDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_AUTOHOME_EDIT_J1, m_editJ1Deg);
	DDX_Control(pDX, IDC_AUTOHOME_EDIT_J2, m_editJ2Deg);
	DDX_Control(pDX, IDC_AUTOHOME_EDIT_J3, m_editJ3Deg);
	DDX_Control(pDX, IDC_AUTOHOME_EDIT_J4, m_editJ4Deg);
	DDX_Control(pDX, IDC_AUTOHOME_EDIT_J5, m_editJ5Deg);
}

BEGIN_MESSAGE_MAP(CAutoHomeDiagPage, CPropertyPage)
	ON_BN_CLICKED(IDC_AUTOHOME_BTN_LOAD_DEFAULTS, &CAutoHomeDiagPage::OnBnClickedLoadDefaults)
	ON_BN_CLICKED(IDC_AUTOHOME_BTN_SAVE, &CAutoHomeDiagPage::OnBnClickedSave)
	ON_WM_DESTROY()
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CAutoHomeDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CAutoHomeDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();
	LoadFromProfile();
	UpdateUiFromValues();
	return TRUE;
}

void CAutoHomeDiagPage::OnDestroy()
{
	CPropertyPage::OnDestroy();
}

LRESULT CAutoHomeDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	if (!GetSafeHwnd())
	{
		return 0;
	}
	LoadFromProfile();
	UpdateUiFromValues();
	return 0;
}

void CAutoHomeDiagPage::LoadFromProfile()
{
	m_j1Deg = AfxGetApp()->GetProfileInt(L"AutoHome", L"J1Deg", 0);
	m_j2Deg = AfxGetApp()->GetProfileInt(L"AutoHome", L"J2Deg", -30);
	m_j3Deg = AfxGetApp()->GetProfileInt(L"AutoHome", L"J3Deg", 60);
	m_j4Deg = AfxGetApp()->GetProfileInt(L"AutoHome", L"J4Deg", 30);
	m_j5Deg = AfxGetApp()->GetProfileInt(L"AutoHome", L"J5Deg", 0);
}

void CAutoHomeDiagPage::SaveToProfile()
{
	AfxGetApp()->WriteProfileInt(L"AutoHome", L"J1Deg", m_j1Deg);
	AfxGetApp()->WriteProfileInt(L"AutoHome", L"J2Deg", m_j2Deg);
	AfxGetApp()->WriteProfileInt(L"AutoHome", L"J3Deg", m_j3Deg);
	AfxGetApp()->WriteProfileInt(L"AutoHome", L"J4Deg", m_j4Deg);
	AfxGetApp()->WriteProfileInt(L"AutoHome", L"J5Deg", m_j5Deg);
}

void CAutoHomeDiagPage::SetDefaultValues()
{
	m_j1Deg = 0;
	m_j2Deg = -30;
	m_j3Deg = 60;
	m_j4Deg = 30;
	m_j5Deg = 0;
}

void CAutoHomeDiagPage::UpdateUiFromValues()
{
	SetIntToEdit(m_editJ1Deg, m_j1Deg);
	SetIntToEdit(m_editJ2Deg, m_j2Deg);
	SetIntToEdit(m_editJ3Deg, m_j3Deg);
	SetIntToEdit(m_editJ4Deg, m_j4Deg);
	SetIntToEdit(m_editJ5Deg, m_j5Deg);
}

void CAutoHomeDiagPage::UpdateValuesFromUi()
{
	m_j1Deg = GetIntFromEdit(m_editJ1Deg, 0);
	m_j2Deg = GetIntFromEdit(m_editJ2Deg, -30);
	m_j3Deg = GetIntFromEdit(m_editJ3Deg, 60);
	m_j4Deg = GetIntFromEdit(m_editJ4Deg, 30);
	m_j5Deg = GetIntFromEdit(m_editJ5Deg, 0);
}

int CAutoHomeDiagPage::GetIntFromEdit(const CEdit& edit, int fallback) const
{
	CString txt;
	const_cast<CEdit&>(edit).GetWindowTextW(txt);
	if (txt.IsEmpty()) return fallback;
	return _wtoi(txt);
}

void CAutoHomeDiagPage::SetIntToEdit(CEdit& edit, int v)
{
	CString txt;
	txt.Format(L"%d", v);
	edit.SetWindowTextW(txt);
}

void CAutoHomeDiagPage::OnBnClickedLoadDefaults()
{
	SetDefaultValues();
	UpdateUiFromValues();
}

void CAutoHomeDiagPage::OnBnClickedSave()
{
	UpdateValuesFromUi();
	SaveToProfile();
	AfxMessageBox(L"自动归位角度已保存。下次连接时生效。");
}

