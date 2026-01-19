#pragma once

#include <afxdlgs.h>

class CSerialDiagPage;
class CCameraDiagPage;
class CMotionDiagPage;
class CKinematicsDiagPage;
class CSeeAndFetchDiagPage;
class CAutoHomeDiagPage;

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
	CMotionDiagPage* m_motion = nullptr;
	CKinematicsDiagPage* m_kinematics = nullptr;
	CSeeAndFetchDiagPage* m_seeFetch = nullptr;
	CAutoHomeDiagPage* m_autoHome = nullptr;
};
