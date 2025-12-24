#include "pch.h"

#include "ControlDiagPage.h"

#include "智能机械臂.h"
#include "resource.h"
#include "AppMessages.h"

IMPLEMENT_DYNAMIC(CControlDiagPage, CPropertyPage)

namespace
{
	constexpr UINT_PTR kTimer = 1;
	constexpr UINT kUiUpdateMs = 200;
}

CControlDiagPage::CControlDiagPage()
	: CPropertyPage(IDD_PAGE_CONTROL, IDS_TAB_CONTROL)
{
}

CControlDiagPage::~CControlDiagPage()
{
}

void CControlDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SLIDER_THROTTLE, m_slider);
	DDX_Control(pDX, IDC_STATIC_THROTTLE_VALUE, m_txtThrottle);
	DDX_Control(pDX, IDC_STATIC_SEND_FPS, m_txtSendFps);
	DDX_Control(pDX, IDC_STATIC_LAST_SEND_TIME, m_txtLastSend);
	DDX_Control(pDX, IDC_EDIT_THROTTLE_LOG, m_log);
}

BEGIN_MESSAGE_MAP(CControlDiagPage, CPropertyPage)
	ON_WM_HSCROLL()
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CControlDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CControlDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();

	m_slider.SetRange(30, 100);
	m_slider.SetTicFreq(5);

	// Load from profile
	m_throttleMs = AfxGetApp()->GetProfileInt(L"Throttle", L"Ms", 50);
	if (m_throttleMs < 30) m_throttleMs = 30;
	if (m_throttleMs > 100) m_throttleMs = 100;
	m_slider.SetPos(m_throttleMs);

	m_lastTick = GetTickCount();
	m_timerId = SetTimer(kTimer, kUiUpdateMs, nullptr);
	UpdateUi();
	AppendLog(L"[INFO] Throttle diagnostics initialized.");

	return TRUE;
}

void CControlDiagPage::OnDestroy()
{
	if (m_timerId)
	{
		KillTimer(m_timerId);
		m_timerId = 0;
	}
	CPropertyPage::OnDestroy();
}

LRESULT CControlDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	// Reload throttle from profile and refresh UI.
	m_throttleMs = AfxGetApp()->GetProfileInt(L"Throttle", L"Ms", 50);
	if (m_throttleMs < 30) m_throttleMs = 30;
	if (m_throttleMs > 100) m_throttleMs = 100;
	if (m_slider.GetSafeHwnd())
	{
		m_slider.SetPos(m_throttleMs);
	}
	UpdateUi();
	AppendLog(L"[INFO] Settings imported: Throttle reloaded.");
	return 0;
}

void CControlDiagPage::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	(void)nSBCode; (void)nPos;
	if (pScrollBar && pScrollBar->GetSafeHwnd() == m_slider.GetSafeHwnd())
	{
		m_throttleMs = m_slider.GetPos();
		AfxGetApp()->WriteProfileInt(L"Throttle", L"Ms", m_throttleMs);
		CString line;
		line.Format(L"[INFO] Throttle set to %d ms.", m_throttleMs);
		AppendLog(line);
		UpdateUi();
	}
	CPropertyPage::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CControlDiagPage::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == kTimer)
	{
		UpdateUi();
	}
	CPropertyPage::OnTimer(nIDEvent);
}

void CControlDiagPage::AppendLog(const CString& line)
{
	CString text;
	m_log.GetWindowTextW(text);
	if (!text.IsEmpty())
	{
		text += L"\r\n";
	}
	text += line;
	m_log.SetWindowTextW(text);
	m_log.LineScroll(m_log.GetLineCount());
}

void CControlDiagPage::UpdateUi()
{
	CString s;
	s.Format(L"%d", m_throttleMs);
	m_txtThrottle.SetWindowTextW(s);

	unsigned fps = 0;
	unsigned sinceMs = 0;
	theApp.GetSerialSendStats(fps, sinceMs);

	CString fpsText;
	fpsText.Format(L"%u fps", fps);
	m_txtSendFps.SetWindowTextW(fpsText);

	CString lastText;
	lastText.Format(L"%u ms", sinceMs);
	m_txtLastSend.SetWindowTextW(lastText);
}


