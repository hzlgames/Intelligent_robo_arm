#include "pch.h"

#include "DiagnosticsSheet.h"

#include "SerialDiagPage.h"
#include "CameraDiagPage.h"

#include "ControlDiagPage.h"
#include "MotionDiagPage.h"
#include "resource.h"

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

	AddPage(m_serial);
	AddPage(m_camera);
	AddPage(m_control);
	AddPage(m_motion);
}

CDiagnosticsSheet::~CDiagnosticsSheet()
{
	// Pages are owned by this sheet; delete explicitly.
	delete m_serial;
	delete m_camera;
	delete m_control;
	delete m_motion;
}


