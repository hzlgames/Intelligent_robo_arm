#include "pch.h"

#include "MotionDiagPage.h"

#include "ArmCommsService.h"
#include "resource.h"
#include "AppMessages.h"

#include <algorithm>

IMPLEMENT_DYNAMIC(CMotionDiagPage, CPropertyPage)

namespace
{
	constexpr UINT_PTR kTimerPoll = 1;
	constexpr UINT kPollMs = 50;

	std::vector<CString> EnumerateComPortsFromRegistry()
	{
		std::vector<CString> out;
		HKEY hKey = nullptr;
		if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey) != ERROR_SUCCESS)
		{
			return out;
		}

		DWORD index = 0;
		WCHAR valueName[256];
		DWORD valueNameLen = ARRAYSIZE(valueName);
		BYTE data[256];
		DWORD dataLen = sizeof(data);
		DWORD type = 0;

		while (true)
		{
			valueNameLen = ARRAYSIZE(valueName);
			dataLen = sizeof(data);
			const LSTATUS s = RegEnumValueW(
				hKey,
				index,
				valueName,
				&valueNameLen,
				nullptr,
				&type,
				data,
				&dataLen);
			if (s != ERROR_SUCCESS) break;

			if (type == REG_SZ)
			{
				const wchar_t* str = reinterpret_cast<const wchar_t*>(data);
				if (str && wcslen(str) > 0)
				{
					out.push_back(CString(str));
				}
			}

			index++;
		}

		RegCloseKey(hKey);

		std::sort(out.begin(), out.end(), [](const CString& a, const CString& b) {
			auto toNum = [](const CString& s) -> int {
				if (s.GetLength() >= 4 && (s.Left(3).CompareNoCase(L"COM") == 0))
				{
					return _wtoi(s.Mid(3));
				}
				return 9999;
			};
			const int na = toNum(a);
			const int nb = toNum(b);
			if (na != nb) return na < nb;
			return a.CompareNoCase(b) < 0;
			});
		out.erase(std::unique(out.begin(), out.end(), [](const CString& a, const CString& b) {
			return a.CompareNoCase(b) == 0;
			}), out.end());

		return out;
	}
}

CMotionDiagPage::CMotionDiagPage()
	: CPropertyPage(IDD_PAGE_MOTION, IDS_TAB_MOTION)
{
}

CMotionDiagPage::~CMotionDiagPage()
{
}

void CMotionDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_MOTION_COMBO_COMPORT, m_comboCom);
	DDX_Control(pDX, IDC_MOTION_CHECK_SIMULATE, m_checkSim);
	DDX_Control(pDX, IDC_MOTION_STATIC_STATUS, m_staticStatus);

	DDX_Control(pDX, IDC_MOTION_COMBO_JOINT, m_comboJoint);
	DDX_Control(pDX, IDC_MOTION_EDIT_SERVOID, m_editServoId);
	DDX_Control(pDX, IDC_MOTION_EDIT_MIN, m_editMin);
	DDX_Control(pDX, IDC_MOTION_EDIT_MAX, m_editMax);
	DDX_Control(pDX, IDC_MOTION_EDIT_HOME, m_editHome);
	DDX_Control(pDX, IDC_MOTION_CHECK_INVERT, m_checkInvert);

	DDX_Control(pDX, IDC_MOTION_EDIT_TARGET, m_editTarget);
	DDX_Control(pDX, IDC_MOTION_EDIT_TIME, m_editTime);

	DDX_Control(pDX, IDC_MOTION_CHECK_LOOP, m_checkLoop);
	DDX_Control(pDX, IDC_MOTION_EDIT_LOG, m_editLog);
}

BEGIN_MESSAGE_MAP(CMotionDiagPage, CPropertyPage)
	ON_BN_CLICKED(IDC_MOTION_BTN_REFRESH_COM, &CMotionDiagPage::OnBnClickedRefreshCom)
	ON_BN_CLICKED(IDC_MOTION_BTN_CONNECT, &CMotionDiagPage::OnBnClickedConnect)
	ON_BN_CLICKED(IDC_MOTION_CHECK_SIMULATE, &CMotionDiagPage::OnCheckSimulate)

	ON_CBN_SELCHANGE(IDC_MOTION_COMBO_JOINT, &CMotionDiagPage::OnCbnSelChangeJoint)
	ON_BN_CLICKED(IDC_MOTION_BTN_LOAD_ALL, &CMotionDiagPage::OnBnClickedLoadAll)
	ON_BN_CLICKED(IDC_MOTION_BTN_SAVE_ALL, &CMotionDiagPage::OnBnClickedSaveAll)
	ON_BN_CLICKED(IDC_MOTION_BTN_IMPORT_LIMITS, &CMotionDiagPage::OnBnClickedImportLegacyLimits)

	ON_BN_CLICKED(IDC_MOTION_BTN_MOVE, &CMotionDiagPage::OnBnClickedMoveSelected)
	ON_BN_CLICKED(IDC_MOTION_BTN_HOME, &CMotionDiagPage::OnBnClickedHome)
	ON_BN_CLICKED(IDC_MOTION_BTN_READALL, &CMotionDiagPage::OnBnClickedReadAll)

	ON_BN_CLICKED(IDC_MOTION_BTN_DEMO_PLAY, &CMotionDiagPage::OnBnClickedDemoPlay)
	ON_BN_CLICKED(IDC_MOTION_BTN_STOP, &CMotionDiagPage::OnBnClickedStop)

	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CMotionDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CMotionDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();

	m_checkSim.SetCheck(BST_CHECKED);
	m_useSim = true;
	RefreshComList();
	UpdateStatusText();

	m_comboJoint.ResetContent();
	for (int j = 1; j <= MotionConfig::kJointCount; j++)
	{
		CString item(MotionConfig::JointName(j).c_str());
		m_comboJoint.AddString(item);
	}
	m_comboJoint.SetCurSel(0);

	m_motion.LoadConfig();
	LoadSelectedJointToUi();

	SetIntToEdit(m_editTime, 800);
	SetIntToEdit(m_editTarget, 500);
	m_checkLoop.SetCheck(BST_UNCHECKED);

	m_logToken = ArmCommsService::Instance().AddLogListener([this](const std::wstring& line) {
		this->AppendLogLine(CString(line.c_str()));
	});

	m_timerId = SetTimer(kTimerPoll, kPollMs, nullptr);
	return TRUE;
}

void CMotionDiagPage::OnDestroy()
{
	if (m_timerId)
	{
		KillTimer(m_timerId);
		m_timerId = 0;
	}
	if (m_logToken)
	{
		ArmCommsService::Instance().RemoveLogListener(m_logToken);
		m_logToken = 0;
	}
	m_motion.SaveConfig();
	CPropertyPage::OnDestroy();
}

LRESULT CMotionDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	// Reload motion calibration config from profile and refresh UI.
	if (!GetSafeHwnd())
	{
		return 0;
	}
	m_motion.LoadConfig();
	LoadSelectedJointToUi();
	AppendLogLine(L"[INFO] Settings imported: Motion config reloaded.");
	return 0;
}

void CMotionDiagPage::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == kTimerPoll)
	{
		ArmCommsService::Instance().Tick();
		m_motion.Tick();
	}
	CPropertyPage::OnTimer(nIDEvent);
}

void CMotionDiagPage::AppendLogLine(const CString& line)
{
	m_logLines.push_back(line);
	if (m_logLines.size() > 2000)
	{
		m_logLines.erase(m_logLines.begin(), m_logLines.begin() + (m_logLines.size() - 2000));
	}
	CString all;
	for (const auto& l : m_logLines)
	{
		all += l;
		all += L"\r\n";
	}
	m_editLog.SetWindowTextW(all);
	m_editLog.LineScroll(m_editLog.GetLineCount());
}

void CMotionDiagPage::RefreshComList()
{
	m_comboCom.ResetContent();
	const auto ports = EnumerateComPortsFromRegistry();
	for (const auto& p : ports)
	{
		m_comboCom.AddString(p);
	}
	if (!ports.empty())
	{
		m_comboCom.SetCurSel(0);
	}
}

void CMotionDiagPage::UpdateStatusText()
{
	CString s;
	const bool connected = ArmCommsService::Instance().IsConnected();
	const bool sim = ArmCommsService::Instance().IsSim();
	if (!connected)
	{
		s = L"Disconnected";
	}
	else
	{
		s = sim ? L"Connected (sim)" : L"Connected (real)";
	}
	m_staticStatus.SetWindowTextW(s);
}

int CMotionDiagPage::GetIntFromEdit(const CEdit& edit, int fallback) const
{
	CString txt;
	const_cast<CEdit&>(edit).GetWindowTextW(txt);
	if (txt.IsEmpty()) return fallback;
	return _wtoi(txt);
}

void CMotionDiagPage::SetIntToEdit(CEdit& edit, int v)
{
	CString txt;
	txt.Format(L"%d", v);
	edit.SetWindowTextW(txt);
}

void CMotionDiagPage::LoadSelectedJointToUi()
{
	const int sel = m_comboJoint.GetCurSel();
	const int j = sel + 1;
	if (j < 1 || j > MotionConfig::kJointCount) return;
	const auto& jc = m_motion.Config().Get(j);
	SetIntToEdit(m_editServoId, jc.servoId);
	SetIntToEdit(m_editMin, jc.minPos);
	SetIntToEdit(m_editMax, jc.maxPos);
	SetIntToEdit(m_editHome, jc.homePos);
	m_checkInvert.SetCheck(jc.invert ? BST_CHECKED : BST_UNCHECKED);
}

void CMotionDiagPage::SaveSelectedJointFromUi()
{
	const int sel = m_comboJoint.GetCurSel();
	const int j = sel + 1;
	if (j < 1 || j > MotionConfig::kJointCount) return;
	auto& jc = m_motion.Config().Get(j);
	jc.servoId = GetIntFromEdit(m_editServoId, 0);
	jc.minPos = GetIntFromEdit(m_editMin, 0);
	jc.maxPos = GetIntFromEdit(m_editMax, 1000);
	jc.homePos = GetIntFromEdit(m_editHome, 500);
	jc.invert = (m_checkInvert.GetCheck() == BST_CHECKED);
}

void CMotionDiagPage::OnBnClickedRefreshCom()
{
	RefreshComList();
}

void CMotionDiagPage::OnCheckSimulate()
{
	m_useSim = (m_checkSim.GetCheck() == BST_CHECKED);
	if (ArmCommsService::Instance().IsConnected())
	{
		ArmCommsService::Instance().Disconnect();
	}
	UpdateStatusText();
}

void CMotionDiagPage::OnBnClickedConnect()
{
	auto& comms = ArmCommsService::Instance();
	if (!comms.IsConnected())
	{
		if (m_useSim)
		{
			comms.ConnectSim();
		}
		else
		{
			CString com;
			m_comboCom.GetWindowTextW(com);
			if (com.IsEmpty())
			{
				AfxMessageBox(L"Please select a COM port.");
				return;
			}
			if (!comms.ConnectReal(std::wstring(com)))
			{
				CString err(comms.GetLastErrorText().c_str());
				CString msg;
				msg.Format(L"Failed to open serial: %s", err.GetString());
				AfxMessageBox(msg);
				return;
			}
		}
	}
	else
	{
		comms.Disconnect();
	}
	UpdateStatusText();
}

void CMotionDiagPage::OnCbnSelChangeJoint()
{
	SaveSelectedJointFromUi();
	LoadSelectedJointToUi();
}

void CMotionDiagPage::OnBnClickedLoadAll()
{
	m_motion.LoadConfig();
	LoadSelectedJointToUi();
	AppendLogLine(L"[INFO] Motion config loaded.");
}

void CMotionDiagPage::OnBnClickedSaveAll()
{
	SaveSelectedJointFromUi();
	m_motion.SaveConfig();
	AppendLogLine(L"[INFO] Motion config saved.");
}

void CMotionDiagPage::OnBnClickedImportLegacyLimits()
{
	SaveSelectedJointFromUi();
	m_motion.ImportLegacyServoLimitsForAssignedJoints();
	LoadSelectedJointToUi();
	AppendLogLine(L"[INFO] Imported ServoLimits for assigned joints.");
}

void CMotionDiagPage::OnBnClickedMoveSelected()
{
	SaveSelectedJointFromUi();
	const int sel = m_comboJoint.GetCurSel();
	const int j = sel + 1;
	const int pos = GetIntFromEdit(m_editTarget, 500);
	const int timeMs = GetIntFromEdit(m_editTime, 800);
	if (!m_motion.MoveJointAbs(j, pos, timeMs))
	{
		AppendLogLine(L"[WARN] MoveSelected: No ServoId configured.");
	}
}

void CMotionDiagPage::OnBnClickedHome()
{
	SaveSelectedJointFromUi();
	const int timeMs = GetIntFromEdit(m_editTime, 800);
	if (!m_motion.MoveHome(timeMs))
	{
		AppendLogLine(L"[WARN] Home: No valid ServoId configured.");
	}
}

void CMotionDiagPage::OnBnClickedReadAll()
{
	SaveSelectedJointFromUi();
	m_motion.RequestReadAllAssigned();
}

std::vector<MotionController::Keyframe> CMotionDiagPage::BuildDemoScript() const
{
	std::vector<MotionController::Keyframe> frames;
	frames.reserve(5);

	MotionController::Keyframe base;
	for (int j = 0; j <= MotionConfig::kJointCount; j++)
	{
		base.jointPos[j] = -1;
	}

	const auto& cfg = m_motion.Config();
	const int home2 = cfg.Get(2).homePos;
	const int home3 = cfg.Get(3).homePos;
	const int d = 60;

	auto make = [&](int p2, int p3) {
		MotionController::Keyframe k = base;
		k.durationMs = GetIntFromEdit(m_editTime, 800);
		k.jointPos[2] = p2;
		k.jointPos[3] = p3;
		return k;
	};

	frames.push_back(make(home2, home3));
	frames.push_back(make(home2 + d, home3 - d));
	frames.push_back(make(home2 - d, home3 + d));
	frames.push_back(make(home2 + d, home3 - d));
	frames.push_back(make(home2, home3));
	return frames;
}

void CMotionDiagPage::OnBnClickedDemoPlay()
{
	SaveSelectedJointFromUi();
	const bool loop = (m_checkLoop.GetCheck() == BST_CHECKED);
	auto frames = BuildDemoScript();
	m_motion.StartScript(std::move(frames), loop);
	AppendLogLine(loop ? L"[INFO] Demo script started (loop)." : L"[INFO] Demo script started.");
}

void CMotionDiagPage::OnBnClickedStop()
{
	m_motion.StopScript();
	ArmCommsService::Instance().EmergencyStop();
	AppendLogLine(L"[INFO] Script stopped.");
}
