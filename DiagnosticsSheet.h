#pragma once

#include <afxdlgs.h>

class CSerialDiagPage;
class CCameraDiagPage;
class CControlDiagPage;
class CMotionDiagPage;

// A simple diagnostics container using property sheet pages.
class CDiagnosticsSheet : public CPropertySheet
{
public:
	CDiagnosticsSheet(CWnd* pParentWnd);
	virtual ~CDiagnosticsSheet();

protected:
	CSerialDiagPage* m_serial = nullptr;
	CCameraDiagPage* m_camera = nullptr;
	CControlDiagPage* m_control = nullptr;
	CMotionDiagPage* m_motion = nullptr;
};


