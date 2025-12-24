#include "pch.h"

#include "SerialDiagPage.h"

#include "ArmProtocol.h"
#include "ArmCommsService.h"
#include "resource.h"
#include "智能机械臂.h"
#include "AppMessages.h"

#include <algorithm>

IMPLEMENT_DYNAMIC(CSerialDiagPage, CPropertyPage)

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

		// Dedup and sort (COM2 before COM10 etc.)
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

	CString LoadStrOr(UINT id, LPCWSTR fallback)
	{
		CString s;
		if (!s.LoadString(id))
		{
			s = fallback;
		}
		return s;
	}
}

CSerialDiagPage::CSerialDiagPage()
	: CPropertyPage(IDD_PAGE_SERIAL, IDS_TAB_SERIAL)
{
	for (int i = 0; i <= 6; i++)
	{
		m_minPos[i] = 0;
		m_maxPos[i] = 1000;
	}
}

CSerialDiagPage::~CSerialDiagPage()
{
}

void CSerialDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_COMPORT, m_comboCom);
	DDX_Control(pDX, IDC_EDIT_SERIAL_LOG, m_editLog);
	DDX_Control(pDX, IDC_CHECK_SIMULATE, m_checkSim);
	DDX_Control(pDX, IDC_STATIC_SERIAL_STATUS, m_staticStatus);
	DDX_Control(pDX, IDC_EDIT_MOVE_ID, m_editMoveId);
	DDX_Control(pDX, IDC_EDIT_MOVE_POS, m_editMovePos);
	DDX_Control(pDX, IDC_EDIT_MOVE_STEP, m_editMoveStep);
	DDX_Control(pDX, IDC_EDIT_MOVE_TIME, m_editMoveTime);
}

BEGIN_MESSAGE_MAP(CSerialDiagPage, CPropertyPage)
	ON_BN_CLICKED(IDC_BTN_REFRESH_COM, &CSerialDiagPage::OnBnClickedRefresh)
	ON_BN_CLICKED(IDC_BTN_SERIAL_CONNECT, &CSerialDiagPage::OnBnClickedConnect)
	ON_BN_CLICKED(IDC_BTN_SERIAL_CLEARLOG, &CSerialDiagPage::OnBnClickedClearLog)
	ON_BN_CLICKED(IDC_BTN_SERIAL_EXPORTLOG, &CSerialDiagPage::OnBnClickedExportLog)
	ON_BN_CLICKED(IDC_BTN_SERIAL_SEND_MOVE, &CSerialDiagPage::OnBnClickedSendMove)
	ON_BN_CLICKED(IDC_BTN_SERIAL_SEND_READALL, &CSerialDiagPage::OnBnClickedSendReadAll)
	ON_BN_CLICKED(IDC_BTN_MOVE_MINUS, &CSerialDiagPage::OnBnClickedMoveMinus)
	ON_BN_CLICKED(IDC_BTN_MOVE_PLUS, &CSerialDiagPage::OnBnClickedMovePlus)
	ON_BN_CLICKED(IDC_BTN_MOVE_SEND, &CSerialDiagPage::OnBnClickedMoveSend)
	ON_BN_CLICKED(IDC_BTN_READ_ID, &CSerialDiagPage::OnBnClickedReadId)
	ON_BN_CLICKED(IDC_BTN_SET_MIN, &CSerialDiagPage::OnBnClickedSetMin)
	ON_BN_CLICKED(IDC_BTN_SET_MAX, &CSerialDiagPage::OnBnClickedSetMax)
	ON_BN_CLICKED(IDC_BTN_SERIAL_SHOW_SETTINGS, &CSerialDiagPage::OnBnClickedShowSettings)
	ON_BN_CLICKED(IDC_BTN_SERIAL_CLEAR_SETTINGS, &CSerialDiagPage::OnBnClickedClearSettings)
	ON_BN_CLICKED(IDC_CHECK_SIMULATE, &CSerialDiagPage::OnCheckSimulate)
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CSerialDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CSerialDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();

	m_checkSim.SetCheck(BST_CHECKED);
	m_useSim = true;

	RefreshComList();
	UpdateStatusText();
	LoadManualControlsFromProfile();
	LoadServoLimitsFromProfile();
	// Subscribe to global comms logs
	m_logToken = ArmCommsService::Instance().AddLogListener([this](const std::wstring& line) {
		this->AppendLogLine(CString(line.c_str()));
	});
	SetTimer(kTimerPoll, kPollMs, nullptr);

	return TRUE;
}

void CSerialDiagPage::OnDestroy()
{
	KillTimer(kTimerPoll);
	SaveManualControlsToProfile();
	if (m_logToken)
	{
		ArmCommsService::Instance().RemoveLogListener(m_logToken);
		m_logToken = 0;
	}
	DisconnectAll();
	CPropertyPage::OnDestroy();
}

LRESULT CSerialDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	// Refresh from profile
	if (!GetSafeHwnd())
	{
		return 0;
	}
	LoadManualControlsFromProfile();
	LoadServoLimitsFromProfile();
	AppendLogLine(L"[INFO] Settings imported: Serial page reloaded.");
	return 0;
}

void CSerialDiagPage::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == kTimerPoll)
	{
		ArmCommsService::Instance().Tick();
	}
	CPropertyPage::OnTimer(nIDEvent);
}

void CSerialDiagPage::RefreshComList()
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

void CSerialDiagPage::AppendLogLine(const CString& line)
{
	m_logLines.push_back(line);
	// keep last ~2000 lines
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

void CSerialDiagPage::UpdateStatusText()
{
	CString s;
	const bool connected = ArmCommsService::Instance().IsConnected();
	const bool sim = ArmCommsService::Instance().IsSim();
	if (!connected)
	{
		s = LoadStrOr(IDS_STATUS_DISCONNECTED, L"Disconnected");
	}
	else
	{
		s = sim ? LoadStrOr(IDS_STATUS_CONNECTED_SIM, L"Connected (simulated)")
		        : LoadStrOr(IDS_STATUS_CONNECTED_REAL, L"Connected (real)");
	}
	m_staticStatus.SetWindowTextW(s);
}

void CSerialDiagPage::OnBnClickedRefresh()
{
	RefreshComList();
}

void CSerialDiagPage::OnCheckSimulate()
{
	m_useSim = (m_checkSim.GetCheck() == BST_CHECKED);
	if (ArmCommsService::Instance().IsConnected())
	{
		// Reconnect cleanly to avoid holding the previous backend.
		DisconnectAll();
		OnBnClickedConnect();
	}
	UpdateStatusText();
}

void CSerialDiagPage::DisconnectAll()
{
	ArmCommsService::Instance().Disconnect();
}

void CSerialDiagPage::OnBnClickedConnect()
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
				AfxMessageBox(LoadStrOr(IDS_MSG_SELECT_COM, L"Please select a COM port."));
				return;
			}
			if (!comms.ConnectReal(std::wstring(com)))
			{
				CString err(comms.GetLastErrorText().c_str());
				CString msg;
				CString fmt = LoadStrOr(IDS_MSG_OPEN_COM_FAILED, L"Failed to open serial port: %s");
				msg.Format(fmt, err.GetString());
				AfxMessageBox(msg);
				return;
			}
		}
	}
	else
	{
		DisconnectAll();
	}
	UpdateStatusText();
}

// ============== Sample demo buttons ==============

void CSerialDiagPage::OnBnClickedSendMove()
{
	std::vector<ArmProtocol::ServoTarget> servos;
	ArmProtocol::ServoTarget st;
	st.id = 1;
	st.position = 500;
	servos.push_back(st);
	ArmCommsService::Instance().EnqueueTx(ArmProtocol::PackMove(servos, 800));
}

void CSerialDiagPage::OnBnClickedSendReadAll()
{
	std::vector<uint8_t> ids = { 1, 2, 3, 4, 5, 6 };
	ArmCommsService::Instance().EnqueueTx(ArmProtocol::PackReadPosition(ids));
}

// ============== Manual move / calibration ==============

int CSerialDiagPage::GetIntFromEdit(const CEdit& edit, int fallback) const
{
	CString txt;
	const_cast<CEdit&>(edit).GetWindowTextW(txt);
	if (txt.IsEmpty()) return fallback;
	return _wtoi(txt);
}

void CSerialDiagPage::SetIntToEdit(CEdit& edit, int v)
{
	CString txt;
	txt.Format(L"%d", v);
	edit.SetWindowTextW(txt);
}

void CSerialDiagPage::LoadManualControlsFromProfile()
{
	const int id = AfxGetApp()->GetProfileInt(L"ManualMove", L"Id", 1);
	const int pos = AfxGetApp()->GetProfileInt(L"ManualMove", L"Pos", 500);
	const int step = AfxGetApp()->GetProfileInt(L"ManualMove", L"Step", 20);
	const int time = AfxGetApp()->GetProfileInt(L"ManualMove", L"Time", 800);
	SetIntToEdit(m_editMoveId, id);
	SetIntToEdit(m_editMovePos, pos);
	SetIntToEdit(m_editMoveStep, step);
	SetIntToEdit(m_editMoveTime, time);
}

void CSerialDiagPage::SaveManualControlsToProfile()
{
	AfxGetApp()->WriteProfileInt(L"ManualMove", L"Id", GetIntFromEdit(m_editMoveId, 1));
	AfxGetApp()->WriteProfileInt(L"ManualMove", L"Pos", GetIntFromEdit(m_editMovePos, 500));
	AfxGetApp()->WriteProfileInt(L"ManualMove", L"Step", GetIntFromEdit(m_editMoveStep, 20));
	AfxGetApp()->WriteProfileInt(L"ManualMove", L"Time", GetIntFromEdit(m_editMoveTime, 800));
}

void CSerialDiagPage::LoadServoLimitsFromProfile()
{
	for (int id = 1; id <= 6; id++)
	{
		CString keyMin, keyMax;
		keyMin.Format(L"Min%d", id);
		keyMax.Format(L"Max%d", id);
		m_minPos[id] = AfxGetApp()->GetProfileInt(L"ServoLimits", keyMin, 0);
		m_maxPos[id] = AfxGetApp()->GetProfileInt(L"ServoLimits", keyMax, 1000);
	}
}

void CSerialDiagPage::SaveServoLimitToProfile(uint8_t id, bool isMin, int v)
{
	if (id < 1 || id > 6) return;
	CString key;
	key.Format(isMin ? L"Min%d" : L"Max%d", (int)id);
	AfxGetApp()->WriteProfileInt(L"ServoLimits", key, v);
	if (isMin)
	{
		m_minPos[id] = v;
	}
	else
	{
		m_maxPos[id] = v;
	}
}

int CSerialDiagPage::ApplySafeClamp(uint8_t id, int pos, bool* clamped)
{
	if (id < 1 || id > 6)
	{
		if (clamped) *clamped = false;
		return pos;
	}
	int minV = m_minPos[id];
	int maxV = m_maxPos[id];
	if (minV > maxV) std::swap(minV, maxV);
	if (pos < minV)
	{
		if (clamped) *clamped = true;
		return minV;
	}
	if (pos > maxV)
	{
		if (clamped) *clamped = true;
		return maxV;
	}
	if (clamped) *clamped = false;
	return pos;
}

void CSerialDiagPage::OnBnClickedMoveMinus()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	int pos = GetIntFromEdit(m_editMovePos, 500);
	const int step = GetIntFromEdit(m_editMoveStep, 20);
	pos -= step;
	bool clamped = false;
	pos = ApplySafeClamp(static_cast<uint8_t>(id), pos, &clamped);
	SetIntToEdit(m_editMovePos, pos);
	if (clamped)
	{
		AppendLogLine(L"[WARN] 已触及安全边界 min，已夹值。");
	}
}

void CSerialDiagPage::OnBnClickedMovePlus()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	int pos = GetIntFromEdit(m_editMovePos, 500);
	const int step = GetIntFromEdit(m_editMoveStep, 20);
	pos += step;
	bool clamped = false;
	pos = ApplySafeClamp(static_cast<uint8_t>(id), pos, &clamped);
	SetIntToEdit(m_editMovePos, pos);
	if (clamped)
	{
		AppendLogLine(L"[WARN] 已触及安全边界 max，已夹值。");
	}
}

void CSerialDiagPage::OnBnClickedMoveSend()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	int pos = GetIntFromEdit(m_editMovePos, 500);
	const int time = GetIntFromEdit(m_editMoveTime, 800);
	bool clamped = false;
	pos = ApplySafeClamp(static_cast<uint8_t>(id), pos, &clamped);
	if (clamped)
	{
		SetIntToEdit(m_editMovePos, pos);
		AppendLogLine(L"[WARN] 已按安全边界夹值。");
	}

	std::vector<ArmProtocol::ServoTarget> servos;
	ArmProtocol::ServoTarget st;
	st.id = static_cast<uint8_t>(id);
	st.position = static_cast<uint16_t>(pos);
	servos.push_back(st);
	ArmCommsService::Instance().EnqueueTx(ArmProtocol::PackMove(servos, static_cast<uint16_t>(time)));
}

void CSerialDiagPage::OnBnClickedReadId()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	std::vector<uint8_t> ids;
	ids.push_back(static_cast<uint8_t>(id));
	ArmCommsService::Instance().EnqueueTx(ArmProtocol::PackReadPosition(ids));
}

void CSerialDiagPage::OnBnClickedSetMin()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	const int pos = GetIntFromEdit(m_editMovePos, 0);
	SaveServoLimitToProfile(static_cast<uint8_t>(id), true, pos);
	CString msg;
	msg.Format(L"[INFO] 舵机%d min=%d 已保存。", id, pos);
	AppendLogLine(msg);
}

void CSerialDiagPage::OnBnClickedSetMax()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	const int pos = GetIntFromEdit(m_editMovePos, 1000);
	SaveServoLimitToProfile(static_cast<uint8_t>(id), false, pos);
	CString msg;
	msg.Format(L"[INFO] 舵机%d max=%d 已保存。", id, pos);
	AppendLogLine(msg);
}

void CSerialDiagPage::OnBnClickedShowSettings()
{
	const int id = GetIntFromEdit(m_editMoveId, 1);
	const int pos = GetIntFromEdit(m_editMovePos, 500);
	const int step = GetIntFromEdit(m_editMoveStep, 20);
	const int time = GetIntFromEdit(m_editMoveTime, 800);
	const int throttle = AfxGetApp()->GetProfileInt(L"Throttle", L"Ms", 50);

	CString line;
	line.Format(L"[SET] ManualMove: id=%d pos=%d step=%d time=%dms | Throttle=%dms", id, pos, step, time, throttle);
	AppendLogLine(line);

	if (id >= 1 && id <= 6)
	{
		CString lim;
		lim.Format(L"[SET] Servo%d limits: min=%d max=%d (profile: ServoLimits/Min%d,Max%d)", id, m_minPos[id], m_maxPos[id], id, id);
		AppendLogLine(lim);
	}
	else
	{
		AppendLogLine(L"[SET] Servo limits: 当前 ID 不在 1..6，未显示范围。");
	}
}

void CSerialDiagPage::OnBnClickedClearSettings()
{
	// Popup menu with explicit clear actions (\"指定清除的参数\")
	enum : UINT
	{
		kCmdClearThisMin = 1,
		kCmdClearThisMax = 2,
		kCmdClearThisBoth = 3,
		kCmdClearAllLimits = 4,
		kCmdClearManualMove = 5,
	};

	CMenu menu;
	menu.CreatePopupMenu();
	menu.AppendMenuW(MF_STRING, kCmdClearThisMin, L"清除 当前ID 的 Min（恢复 0）");
	menu.AppendMenuW(MF_STRING, kCmdClearThisMax, L"清除 当前ID 的 Max（恢复 1000）");
	menu.AppendMenuW(MF_STRING, kCmdClearThisBoth, L"清除 当前ID 的 Min+Max（恢复默认）");
	menu.AppendMenuW(MF_SEPARATOR);
	menu.AppendMenuW(MF_STRING, kCmdClearAllLimits, L"清除 全部舵机范围（1..6 恢复默认）");
	menu.AppendMenuW(MF_SEPARATOR);
	menu.AppendMenuW(MF_STRING, kCmdClearManualMove, L"清除 手动面板参数（恢复默认）");

	CRect rc;
	GetDlgItem(IDC_BTN_SERIAL_CLEAR_SETTINGS)->GetWindowRect(&rc);
	const UINT cmd = menu.TrackPopupMenu(TPM_RETURNCMD | TPM_LEFTALIGN | TPM_TOPALIGN, rc.left, rc.bottom, this);
	if (cmd == 0)
	{
		return;
	}

	const int id = GetIntFromEdit(m_editMoveId, 1);
	switch (cmd)
	{
	case kCmdClearThisMin:
		SaveServoLimitToProfile(static_cast<uint8_t>(id), true, 0);
		AppendLogLine(L"[INFO] 已清除当前ID的 Min（恢复为 0）。");
		break;
	case kCmdClearThisMax:
		SaveServoLimitToProfile(static_cast<uint8_t>(id), false, 1000);
		AppendLogLine(L"[INFO] 已清除当前ID的 Max（恢复为 1000）。");
		break;
	case kCmdClearThisBoth:
		SaveServoLimitToProfile(static_cast<uint8_t>(id), true, 0);
		SaveServoLimitToProfile(static_cast<uint8_t>(id), false, 1000);
		AppendLogLine(L"[INFO] 已清除当前ID的 Min/Max（恢复默认 0..1000）。");
		break;
	case kCmdClearAllLimits:
		for (int sid = 1; sid <= 6; sid++)
		{
			SaveServoLimitToProfile(static_cast<uint8_t>(sid), true, 0);
			SaveServoLimitToProfile(static_cast<uint8_t>(sid), false, 1000);
		}
		AppendLogLine(L"[INFO] 已清除全部舵机范围（1..6 恢复默认 0..1000）。");
		break;
	case kCmdClearManualMove:
		AfxGetApp()->WriteProfileInt(L"ManualMove", L"Id", 1);
		AfxGetApp()->WriteProfileInt(L"ManualMove", L"Pos", 500);
		AfxGetApp()->WriteProfileInt(L"ManualMove", L"Step", 20);
		AfxGetApp()->WriteProfileInt(L"ManualMove", L"Time", 800);
		LoadManualControlsFromProfile();
		AppendLogLine(L"[INFO] 已清除手动面板参数（恢复默认）。");
		break;
	default:
		break;
	}
}

void CSerialDiagPage::OnBnClickedClearLog()
{
	m_logLines.clear();
	m_editLog.SetWindowTextW(L"");
}

void CSerialDiagPage::OnBnClickedExportLog()
{
	CFileDialog dlg(FALSE, L"txt", L"serial-log.txt", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, L"Text Files (*.txt)|*.txt||", this);
	if (dlg.DoModal() != IDOK)
	{
		return;
	}

	const CString path = dlg.GetPathName();
	CStdioFile f;
	if (!f.Open(path, CFile::modeCreate | CFile::modeWrite | CFile::typeText))
	{
		AfxMessageBox(LoadStrOr(IDS_MSG_EXPORT_FAILED, L"Export failed: cannot write file."));
		return;
	}

	for (const auto& l : m_logLines)
	{
		f.WriteString(l);
		f.WriteString(L"\r\n");
	}
	f.Close();
	AppendLogLine(L"[INFO] Log exported.");
}
