#pragma once

#include <afxdlgs.h>

class CSerialDiagPage;
class CCameraDiagPage;
class CControlDiagPage;
class CMotionDiagPage;
class CKinematicsDiagPage;
class CKinTestDiagPage;
class CSeeAndFetchDiagPage;

// A simple diagnostics container using property sheet pages.
class CDiagnosticsSheet : public CPropertySheet
{
public:
	CDiagnosticsSheet(CWnd* pParentWnd);
	virtual ~CDiagnosticsSheet();

protected:
	virtual BOOL OnInitDialog();

	CSerialDiagPage* m_serial = nullptr;
	CCameraDiagPage* m_camera = nullptr;
	CControlDiagPage* m_control = nullptr;
	CMotionDiagPage* m_motion = nullptr;
	CKinematicsDiagPage* m_kinematics = nullptr;
	CKinTestDiagPage* m_kinTest = nullptr;
	CSeeAndFetchDiagPage* m_seeFetch = nullptr;
};
