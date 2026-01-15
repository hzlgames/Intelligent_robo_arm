#include "pch.h"

#include "SeeAndFetchDiagPage.h"

#include "AppMessages.h"
#include "Resource.h"

#include <algorithm>

IMPLEMENT_DYNAMIC(CSeeAndFetchDiagPage, CPropertyPage)

namespace
{
	struct BroadcastCtx
	{
		UINT msg = 0;
		WPARAM wParam = 0;
		LPARAM lParam = 0;
	};

	BOOL CALLBACK EnumChildProc(HWND hWnd, LPARAM lParam)
	{
		auto* ctx = reinterpret_cast<BroadcastCtx*>(lParam);
		if (!ctx) return FALSE;
		::PostMessageW(hWnd, ctx->msg, ctx->wParam, ctx->lParam);
		::EnumChildWindows(hWnd, EnumChildProc, lParam);
		return TRUE;
	}

	BOOL CALLBACK EnumThreadProc(HWND hWnd, LPARAM lParam)
	{
		auto* ctx = reinterpret_cast<BroadcastCtx*>(lParam);
		if (!ctx) return FALSE;
		::PostMessageW(hWnd, ctx->msg, ctx->wParam, ctx->lParam);
		::EnumChildWindows(hWnd, EnumChildProc, lParam);
		return TRUE;
	}

	void BroadcastSettingsImportedToThread()
	{
		BroadcastCtx ctx;
		ctx.msg = WM_APP_SETTINGS_IMPORTED;
		ctx.wParam = 0;
		ctx.lParam = 0;
		::EnumThreadWindows(::GetCurrentThreadId(), EnumThreadProc, reinterpret_cast<LPARAM>(&ctx));
	}

	double ClampD(double v, double mn, double mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}
}

CSeeAndFetchDiagPage::CSeeAndFetchDiagPage()
	: CPropertyPage(IDD_PAGE_SEEANDFETCH, IDS_TAB_SEEANDFETCH)
{
	m_totalHeight = 0;
	m_scrollPos = 0;
}

CSeeAndFetchDiagPage::~CSeeAndFetchDiagPage()
{
}

void CSeeAndFetchDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CSeeAndFetchDiagPage, CPropertyPage)
	ON_BN_CLICKED(IDC_SF_BTN_LOAD, &CSeeAndFetchDiagPage::OnBnClickedLoad)
	ON_BN_CLICKED(IDC_SF_BTN_SAVE, &CSeeAndFetchDiagPage::OnBnClickedSaveApply)
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CSeeAndFetchDiagPage::OnSettingsImported)
	ON_WM_VSCROLL()
	ON_WM_MOUSEWHEEL()
	ON_WM_SIZE()
END_MESSAGE_MAP()

BOOL CSeeAndFetchDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();

	// 记录所有子控件的原始位置
	CaptureChildPositions();

	// 初始化滚动条
	UpdateScrollBars();

	LoadFromProfileToUi();
	return TRUE;
}

void CSeeAndFetchDiagPage::CaptureChildPositions()
{
	m_children.clear();
	m_totalHeight = 0;

	CWnd* pChild = GetWindow(GW_CHILD);
	while (pChild)
	{
		ChildInfo info;
		info.hwnd = pChild->GetSafeHwnd();
		pChild->GetWindowRect(&info.rcOriginal);
		ScreenToClient(&info.rcOriginal);

		m_children.push_back(info);

		if (info.rcOriginal.bottom > m_totalHeight)
		{
			m_totalHeight = info.rcOriginal.bottom;
		}

		pChild = pChild->GetNextWindow();
	}

	// 底部增加留白
	m_totalHeight += 20;
}

void CSeeAndFetchDiagPage::ApplyScroll(int newScrollPos)
{
	if (newScrollPos == m_scrollPos)
		return;

	const int delta = newScrollPos - m_scrollPos;
	m_scrollPos = newScrollPos;

	// 批量移动所有子控件
	HDWP hdwp = ::BeginDeferWindowPos((int)m_children.size());
	for (const auto& child : m_children)
	{
		if (!::IsWindow(child.hwnd))
			continue;

		// 计算新位置：原始位置 - 滚动偏移
		const int newY = child.rcOriginal.top - m_scrollPos;
		hdwp = ::DeferWindowPos(hdwp, child.hwnd, NULL,
			child.rcOriginal.left, newY,
			child.rcOriginal.Width(), child.rcOriginal.Height(),
			SWP_NOZORDER | SWP_NOACTIVATE);
	}
	::EndDeferWindowPos(hdwp);

	// 更新滚动条位置
	SetScrollPos(SB_VERT, m_scrollPos);

	// 强制重绘整个窗口
	Invalidate(TRUE);
}

LRESULT CSeeAndFetchDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	if (!GetSafeHwnd()) return 0;
	LoadFromProfileToUi();
	return 0;
}

void CSeeAndFetchDiagPage::OnBnClickedLoad()
{
	LoadFromProfileToUi();
}

void CSeeAndFetchDiagPage::OnBnClickedSaveApply()
{
	std::wstring why;
	if (!SaveFromUiToProfile(why))
	{
		CString msg(why.c_str());
		AfxMessageBox(msg.IsEmpty() ? L"保存失败：参数无效。" : msg);
		return;
	}
	BroadcastSettingsImportedToThread();
	AfxMessageBox(L"已保存并应用。");
}

int CSeeAndFetchDiagPage::GetIntFromEditId(int id, int fallback) const
{
	CString txt;
	GetDlgItemTextW(id, txt);
	txt.Trim();
	if (txt.IsEmpty()) return fallback;
	return _wtoi(txt);
}

void CSeeAndFetchDiagPage::SetIntToEditId(int id, int v)
{
	CString txt;
	txt.Format(L"%d", v);
	SetDlgItemTextW(id, txt);
}

bool CSeeAndFetchDiagPage::GetCheckFromId(int id) const
{
	const CWnd* w = GetDlgItem(id);
	if (!w) return false;
	return ((const_cast<CWnd*>(w))->SendMessageW(BM_GETCHECK) == BST_CHECKED);
}

void CSeeAndFetchDiagPage::SetCheckToId(int id, bool on)
{
	CheckDlgButton(id, on ? BST_CHECKED : BST_UNCHECKED);
}

void CSeeAndFetchDiagPage::LoadFromProfileToUi()
{
	CWinApp* app = AfxGetApp();
	if (!app) return;

	// Global
	SetCheckToId(IDC_SF_CHECK_PREFER_ARUCO, app->GetProfileInt(L"SeeAndFetch", L"PreferArucoDuringAuto", 1) ? true : false);
	SetIntToEditId(IDC_SF_EDIT_LOST_FRAMES, app->GetProfileInt(L"SeeAndFetch", L"LostFramesToAbort", 10));
	SetIntToEditId(IDC_SF_EDIT_ACQ_STABLE, app->GetProfileInt(L"SeeAndFetch", L"AcquireStableFrames", 5));
	SetCheckToId(IDC_SF_CHECK_PLANE_CACHE, app->GetProfileInt(L"SeeAndFetch", L"EnablePlaneCache", 1) ? true : false);

	// Timing
	SetIntToEditId(IDC_SF_EDIT_MIN_CMD_MS, app->GetProfileInt(L"SeeAndFetch\\Timing", L"MinCommandIntervalMs", 120));
	SetIntToEditId(IDC_SF_EDIT_DEF_MOVE_MS, app->GetProfileInt(L"SeeAndFetch\\Timing", L"DefaultMoveTimeMs", 220));
	SetIntToEditId(IDC_SF_EDIT_LOCK_AFTER_MS, app->GetProfileInt(L"SeeAndFetch\\Timing", L"LockAfterMoveMs", 240));

	// Find
	SetIntToEditId(IDC_SF_EDIT_FIND_DB, app->GetProfileInt(L"SeeAndFetch\\Find", L"DeadbandPx", 10));
	SetIntToEditId(IDC_SF_EDIT_FIND_STABLE, app->GetProfileInt(L"SeeAndFetch\\Find", L"StableCenterFrames", 3));
	SetIntToEditId(IDC_SF_EDIT_YAW_K_MILLI, app->GetProfileInt(L"SeeAndFetch\\Find", L"Yaw_kDegPerPx_milli", 30));
	SetIntToEditId(IDC_SF_EDIT_YAW_MIN_MDEG, app->GetProfileInt(L"SeeAndFetch\\Find", L"Yaw_MinStepDeg_milli", 600));
	SetIntToEditId(IDC_SF_EDIT_YAW_MAX_MDEG, app->GetProfileInt(L"SeeAndFetch\\Find", L"Yaw_MaxStepDeg_milli", 3500));
	SetIntToEditId(IDC_SF_EDIT_PITCH_K_MILLI, app->GetProfileInt(L"SeeAndFetch\\Find", L"Pitch_kDegPerPx_milli", 30));
	SetIntToEditId(IDC_SF_EDIT_PITCH_MIN_MDEG, app->GetProfileInt(L"SeeAndFetch\\Find", L"Pitch_MinStepDeg_milli", 600));
	SetIntToEditId(IDC_SF_EDIT_PITCH_MAX_MDEG, app->GetProfileInt(L"SeeAndFetch\\Find", L"Pitch_MaxStepDeg_milli", 3500));
	SetIntToEditId(IDC_SF_EDIT_J4PREF_MDEG, app->GetProfileInt(L"SeeAndFetch\\Find", L"J4PreferAbsDeg_milli", 35000));
	SetIntToEditId(IDC_SF_EDIT_SIGN_J1, app->GetProfileInt(L"SeeAndFetch\\Find", L"SignJ1FromErrU", +1));
	SetIntToEditId(IDC_SF_EDIT_SIGN_J4, app->GetProfileInt(L"SeeAndFetch\\Find", L"SignJ4FromErrV", +1));
	SetIntToEditId(IDC_SF_EDIT_SIGN_J3, app->GetProfileInt(L"SeeAndFetch\\Find", L"SignJ3FromErrV", +1));

	// Approach
	{
		const int rm = app->GetProfileInt(L"SeeAndFetch\\Approach", L"RangeMode", 0);
		int id = IDC_SF_RADIO_RANGE_ARUCO;
		if (rm == 1) id = IDC_SF_RADIO_RANGE_BBOX;
		else if (rm == 2) id = IDC_SF_RADIO_RANGE_AUTO;
		CheckRadioButton(IDC_SF_RADIO_RANGE_ARUCO, IDC_SF_RADIO_RANGE_AUTO, id);
	}
	SetIntToEditId(IDC_SF_EDIT_GRASP_DEPTH, app->GetProfileInt(L"SeeAndFetch\\Approach", L"GraspDepthMm", 160));
	SetIntToEditId(IDC_SF_EDIT_DEPTH_STABLE, app->GetProfileInt(L"SeeAndFetch\\Approach", L"DepthStableFrames", 3));
	SetIntToEditId(IDC_SF_EDIT_DEPTH_JUMP, app->GetProfileInt(L"SeeAndFetch\\Approach", L"DepthMaxJumpMm", 40));
	SetIntToEditId(IDC_SF_EDIT_MAX_ADV_STEPS, app->GetProfileInt(L"SeeAndFetch\\Approach", L"MaxAdvanceSteps", 60));
	SetIntToEditId(IDC_SF_EDIT_J2STEP_MDEG, app->GetProfileInt(L"SeeAndFetch\\Approach", L"J2AdvanceStepDeg_milli", 2000));
	SetIntToEditId(IDC_SF_EDIT_SIGN_J2, app->GetProfileInt(L"SeeAndFetch\\Approach", L"SignJ2Advance", +1));
	SetCheckToId(IDC_SF_CHECK_J1_FINE, app->GetProfileInt(L"SeeAndFetch\\Approach", L"EnableJ1FineTune", 1) ? true : false);
	SetIntToEditId(IDC_SF_EDIT_BOX_AREA, app->GetProfileInt(L"SeeAndFetch\\Approach", L"GraspBoxAreaPx2", 30000));
	SetIntToEditId(IDC_SF_EDIT_BOX_SCALE, app->GetProfileInt(L"SeeAndFetch\\Approach", L"GraspBoxScale_milli", 0));
	SetIntToEditId(IDC_SF_EDIT_BOX_STABLE, app->GetProfileInt(L"SeeAndFetch\\Approach", L"BoxStableFrames", 3));
	SetIntToEditId(IDC_SF_EDIT_BOX_JUMP, app->GetProfileInt(L"SeeAndFetch\\Approach", L"BoxAreaMaxJumpPx2", 20000));
	SetIntToEditId(IDC_SF_EDIT_APP_MAX_ATTEMPTS, app->GetProfileInt(L"SeeAndFetch\\Approach", L"MaxAttempts", 3));
	SetIntToEditId(IDC_SF_EDIT_APP_RETRY_RETREAT, app->GetProfileInt(L"SeeAndFetch\\Approach", L"RetryRetreatSteps", 8));
	SetCheckToId(IDC_SF_CHECK_REQUIRE_DET, app->GetProfileInt(L"SeeAndFetch\\Approach", L"BboxRequireDetector", 1) ? true : false);

	// Gripper
	SetIntToEditId(IDC_SF_EDIT_GRIP_JOINT, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"JointIndex", 6));
	SetIntToEditId(IDC_SF_EDIT_GRIP_OPEN, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"OpenPos", 650));
	SetIntToEditId(IDC_SF_EDIT_GRIP_CLOSE, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"ClosePos", 350));
	SetIntToEditId(IDC_SF_EDIT_GRIP_STEP, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"CloseStepPos", 25));
	SetIntToEditId(IDC_SF_EDIT_GRIP_MS, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"CloseMoveTimeMs", 450));
	SetIntToEditId(IDC_SF_EDIT_GRIP_MAXSTEPS, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"MaxCloseSteps", 12));
	SetCheckToId(IDC_SF_CHECK_STALL, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"EnableStallDetect", 0) ? true : false);
	SetIntToEditId(IDC_SF_EDIT_STALL_DELTA, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"StallDetectDeltaPos", 10));
	SetIntToEditId(IDC_SF_EDIT_STALL_AGE, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"StallDetectMaxAgeMs", 800));
	SetIntToEditId(IDC_SF_EDIT_GRASP_ATTEMPTS, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"MaxAttempts", 2));
	SetIntToEditId(IDC_SF_EDIT_GRASP_FAIL_ADV, app->GetProfileInt(L"SeeAndFetch\\Gripper", L"AdvanceStepsOnFail", 1));

	// Place / Return
	{
		const int pm = app->GetProfileInt(L"SeeAndFetch\\Place", L"Mode", 0);
		CheckRadioButton(IDC_SF_RADIO_PLACE_SIMPLE, IDC_SF_RADIO_PLACE_REDDOT, (pm == 1) ? IDC_SF_RADIO_PLACE_REDDOT : IDC_SF_RADIO_PLACE_SIMPLE);
	}
	SetIntToEditId(IDC_SF_EDIT_PLACE_VMODE, app->GetProfileInt(L"SeeAndFetch\\Place", L"VisionMode", 3));
	SetIntToEditId(IDC_SF_EDIT_PLACE_CENTER_STABLE, app->GetProfileInt(L"SeeAndFetch\\Place", L"CenterStableFrames", 3));
	{
		const int rm = app->GetProfileInt(L"SeeAndFetch\\Place", L"RangeMode", 1);
		int id = IDC_SF_RADIO_PLACE_RANGE_BBOX;
		if (rm == 0) id = IDC_SF_RADIO_PLACE_RANGE_ARUCO;
		else if (rm == 2) id = IDC_SF_RADIO_PLACE_RANGE_AUTO;
		CheckRadioButton(IDC_SF_RADIO_PLACE_RANGE_ARUCO, IDC_SF_RADIO_PLACE_RANGE_AUTO, id);
	}
	SetIntToEditId(IDC_SF_EDIT_PLACE_DEPTH, app->GetProfileInt(L"SeeAndFetch\\Place", L"PlaceDepthMm", 180));
	SetIntToEditId(IDC_SF_EDIT_PLACE_BOX_AREA, app->GetProfileInt(L"SeeAndFetch\\Place", L"PlaceBoxAreaPx2", 24000));
	SetIntToEditId(IDC_SF_EDIT_PLACE_BOX_SCALE, app->GetProfileInt(L"SeeAndFetch\\Place", L"PlaceBoxScale_milli", 0));
	SetIntToEditId(IDC_SF_EDIT_PLACE_BOX_STABLE, app->GetProfileInt(L"SeeAndFetch\\Place", L"BoxStableFrames", 3));
	SetIntToEditId(IDC_SF_EDIT_PLACE_BOX_JUMP, app->GetProfileInt(L"SeeAndFetch\\Place", L"BoxAreaMaxJumpPx2", 20000));
	SetIntToEditId(IDC_SF_EDIT_PLACE_MAX_DOWN, app->GetProfileInt(L"SeeAndFetch\\Place", L"MaxDownSteps", 30));
	SetIntToEditId(IDC_SF_EDIT_PLACE_J2DOWN_MDEG, app->GetProfileInt(L"SeeAndFetch\\Place", L"J2DownStepDeg_milli", 2000));
	SetIntToEditId(IDC_SF_EDIT_PLACE_SIGN_J2DOWN, app->GetProfileInt(L"SeeAndFetch\\Place", L"SignJ2Down", +1));
	SetIntToEditId(IDC_SF_EDIT_PLACE_MAX_ATTEMPTS, app->GetProfileInt(L"SeeAndFetch\\Place", L"MaxAttempts", 2));
	SetIntToEditId(IDC_SF_EDIT_PLACE_RETRY_RETREAT, app->GetProfileInt(L"SeeAndFetch\\Place", L"RetryRetreatSteps", 8));
	SetIntToEditId(IDC_SF_EDIT_RETREAT_STEPS, app->GetProfileInt(L"SeeAndFetch\\Place", L"RetreatSteps", 6));
	SetCheckToId(IDC_SF_CHECK_RETURN_START, app->GetProfileInt(L"SeeAndFetch\\Return", L"ReturnToStartPose", 1) ? true : false);
	SetIntToEditId(IDC_SF_EDIT_RETURN_MS, app->GetProfileInt(L"SeeAndFetch\\Return", L"ReturnTimeMs", 1200));
}

bool CSeeAndFetchDiagPage::SaveFromUiToProfile(std::wstring& outWhy)
{
	outWhy.clear();
	CWinApp* app = AfxGetApp();
	if (!app)
	{
		outWhy = L"AfxGetApp() 为空。";
		return false;
	}

	auto inRange = [&](int v, int mn, int mx, const wchar_t* name) -> bool
	{
		if (v < mn || v > mx)
		{
			outWhy = std::wstring(name) + L" 超出范围。";
			return false;
		}
		return true;
	};

	// Global
	const int lost = GetIntFromEditId(IDC_SF_EDIT_LOST_FRAMES, 10);
	const int acq = GetIntFromEditId(IDC_SF_EDIT_ACQ_STABLE, 5);
	if (!inRange(lost, 1, 200, L"LostFramesToAbort")) return false;
	if (!inRange(acq, 1, 200, L"AcquireStableFrames")) return false;
	app->WriteProfileInt(L"SeeAndFetch", L"PreferArucoDuringAuto", GetCheckFromId(IDC_SF_CHECK_PREFER_ARUCO) ? 1 : 0);
	app->WriteProfileInt(L"SeeAndFetch", L"EnablePlaneCache", GetCheckFromId(IDC_SF_CHECK_PLANE_CACHE) ? 1 : 0);
	app->WriteProfileInt(L"SeeAndFetch", L"LostFramesToAbort", lost);
	app->WriteProfileInt(L"SeeAndFetch", L"AcquireStableFrames", acq);

	// Timing
	const int minCmd = GetIntFromEditId(IDC_SF_EDIT_MIN_CMD_MS, 120);
	const int defMove = GetIntFromEditId(IDC_SF_EDIT_DEF_MOVE_MS, 220);
	const int lockAfter = GetIntFromEditId(IDC_SF_EDIT_LOCK_AFTER_MS, 240);
	if (!inRange(minCmd, 0, 5000, L"MinCommandIntervalMs")) return false;
	if (!inRange(defMove, 30, 60000, L"DefaultMoveTimeMs")) return false;
	if (!inRange(lockAfter, 0, 60000, L"LockAfterMoveMs")) return false;
	app->WriteProfileInt(L"SeeAndFetch\\Timing", L"MinCommandIntervalMs", minCmd);
	app->WriteProfileInt(L"SeeAndFetch\\Timing", L"DefaultMoveTimeMs", defMove);
	app->WriteProfileInt(L"SeeAndFetch\\Timing", L"LockAfterMoveMs", lockAfter);

	// Find
	const int db = GetIntFromEditId(IDC_SF_EDIT_FIND_DB, 10);
	const int stableC = GetIntFromEditId(IDC_SF_EDIT_FIND_STABLE, 3);
	if (!inRange(db, 0, 500, L"Find.DeadbandPx")) return false;
	if (!inRange(stableC, 1, 60, L"Find.StableCenterFrames")) return false;

	const int yawK = GetIntFromEditId(IDC_SF_EDIT_YAW_K_MILLI, 30);
	const int yawMin = GetIntFromEditId(IDC_SF_EDIT_YAW_MIN_MDEG, 600);
	const int yawMax = GetIntFromEditId(IDC_SF_EDIT_YAW_MAX_MDEG, 3500);
	const int pitK = GetIntFromEditId(IDC_SF_EDIT_PITCH_K_MILLI, 30);
	const int pitMin = GetIntFromEditId(IDC_SF_EDIT_PITCH_MIN_MDEG, 600);
	const int pitMax = GetIntFromEditId(IDC_SF_EDIT_PITCH_MAX_MDEG, 3500);
	const int j4pref = GetIntFromEditId(IDC_SF_EDIT_J4PREF_MDEG, 35000);
	if (!inRange(yawK, 0, 5000, L"Find.Yaw_kDegPerPx_milli")) return false;
	if (!inRange(yawMin, 0, 90000, L"Find.Yaw_MinStepDeg_milli")) return false;
	if (!inRange(yawMax, yawMin, 90000, L"Find.Yaw_MaxStepDeg_milli")) return false;
	if (!inRange(pitK, 0, 5000, L"Find.Pitch_kDegPerPx_milli")) return false;
	if (!inRange(pitMin, 0, 90000, L"Find.Pitch_MinStepDeg_milli")) return false;
	if (!inRange(pitMax, pitMin, 90000, L"Find.Pitch_MaxStepDeg_milli")) return false;
	if (!inRange(j4pref, 0, 90000, L"Find.J4PreferAbsDeg_milli")) return false;
	const int sJ1 = GetIntFromEditId(IDC_SF_EDIT_SIGN_J1, +1);
	const int sJ4 = GetIntFromEditId(IDC_SF_EDIT_SIGN_J4, +1);
	const int sJ3 = GetIntFromEditId(IDC_SF_EDIT_SIGN_J3, +1);
	if (!(sJ1 == 1 || sJ1 == -1)) { outWhy = L"SignJ1FromErrU 只能为 +1 或 -1。"; return false; }
	if (!(sJ4 == 1 || sJ4 == -1)) { outWhy = L"SignJ4FromErrV 只能为 +1 或 -1。"; return false; }
	if (!(sJ3 == 1 || sJ3 == -1)) { outWhy = L"SignJ3FromErrV 只能为 +1 或 -1。"; return false; }

	app->WriteProfileInt(L"SeeAndFetch\\Find", L"DeadbandPx", db);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"StableCenterFrames", stableC);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"Yaw_kDegPerPx_milli", yawK);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"Yaw_MinStepDeg_milli", yawMin);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"Yaw_MaxStepDeg_milli", yawMax);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"Pitch_kDegPerPx_milli", pitK);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"Pitch_MinStepDeg_milli", pitMin);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"Pitch_MaxStepDeg_milli", pitMax);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"J4PreferAbsDeg_milli", j4pref);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"SignJ1FromErrU", sJ1);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"SignJ4FromErrV", sJ4);
	app->WriteProfileInt(L"SeeAndFetch\\Find", L"SignJ3FromErrV", sJ3);

	// Approach
	int rangeMode = 0;
	if (IsDlgButtonChecked(IDC_SF_RADIO_RANGE_BBOX) == BST_CHECKED) rangeMode = 1;
	else if (IsDlgButtonChecked(IDC_SF_RADIO_RANGE_AUTO) == BST_CHECKED) rangeMode = 2;
	const int graspDepth = GetIntFromEditId(IDC_SF_EDIT_GRASP_DEPTH, 160);
	const int depStable = GetIntFromEditId(IDC_SF_EDIT_DEPTH_STABLE, 3);
	const int depJump = GetIntFromEditId(IDC_SF_EDIT_DEPTH_JUMP, 40);
	const int maxAdv = GetIntFromEditId(IDC_SF_EDIT_MAX_ADV_STEPS, 60);
	const int j2Step = GetIntFromEditId(IDC_SF_EDIT_J2STEP_MDEG, 2000);
	const int sJ2 = GetIntFromEditId(IDC_SF_EDIT_SIGN_J2, +1);
	const int boxArea = GetIntFromEditId(IDC_SF_EDIT_BOX_AREA, 30000);
	const int boxScale = GetIntFromEditId(IDC_SF_EDIT_BOX_SCALE, 0);
	const int boxStable = GetIntFromEditId(IDC_SF_EDIT_BOX_STABLE, 3);
	const int boxJump = GetIntFromEditId(IDC_SF_EDIT_BOX_JUMP, 20000);
	const int maxAttempts = GetIntFromEditId(IDC_SF_EDIT_APP_MAX_ATTEMPTS, 3);
	const int retryRetreat = GetIntFromEditId(IDC_SF_EDIT_APP_RETRY_RETREAT, 8);
	if (!inRange(graspDepth, 1, 5000, L"Approach.GraspDepthMm")) return false;
	if (!inRange(depStable, 1, 120, L"Approach.DepthStableFrames")) return false;
	if (!inRange(depJump, 0, 5000, L"Approach.DepthMaxJumpMm")) return false;
	if (!inRange(maxAdv, 1, 10000, L"Approach.MaxAdvanceSteps")) return false;
	if (!inRange(j2Step, 0, 90000, L"Approach.J2AdvanceStepDeg_milli")) return false;
	if (!inRange(boxArea, 0, 5000000, L"Approach.GraspBoxAreaPx2")) return false;
	if (!inRange(boxScale, 0, 10000, L"Approach.GraspBoxScale_milli")) return false;
	if (!inRange(boxStable, 1, 120, L"Approach.BoxStableFrames")) return false;
	if (!inRange(boxJump, 0, 5000000, L"Approach.BoxAreaMaxJumpPx2")) return false;
	if (!inRange(maxAttempts, 1, 50, L"Approach.MaxAttempts")) return false;
	if (!inRange(retryRetreat, 0, 200, L"Approach.RetryRetreatSteps")) return false;
	if (!(sJ2 == 1 || sJ2 == -1)) { outWhy = L"SignJ2Advance 只能为 +1 或 -1。"; return false; }
	if (!inRange(rangeMode, 0, 2, L"Approach.RangeMode")) return false;
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"RangeMode", rangeMode);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"GraspDepthMm", graspDepth);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"DepthStableFrames", depStable);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"DepthMaxJumpMm", depJump);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"MaxAdvanceSteps", maxAdv);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"MaxAttempts", maxAttempts);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"RetryRetreatSteps", retryRetreat);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"J2AdvanceStepDeg_milli", j2Step);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"SignJ2Advance", sJ2);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"EnableJ1FineTune", GetCheckFromId(IDC_SF_CHECK_J1_FINE) ? 1 : 0);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"GraspBoxAreaPx2", boxArea);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"GraspBoxScale_milli", boxScale);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"BoxStableFrames", boxStable);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"BoxAreaMaxJumpPx2", boxJump);
	app->WriteProfileInt(L"SeeAndFetch\\Approach", L"BboxRequireDetector", GetCheckFromId(IDC_SF_CHECK_REQUIRE_DET) ? 1 : 0);

	// Gripper
	const int gripJoint = GetIntFromEditId(IDC_SF_EDIT_GRIP_JOINT, 6);
	const int gripOpen = GetIntFromEditId(IDC_SF_EDIT_GRIP_OPEN, 650);
	const int gripClose = GetIntFromEditId(IDC_SF_EDIT_GRIP_CLOSE, 350);
	const int gripStep = GetIntFromEditId(IDC_SF_EDIT_GRIP_STEP, 25);
	const int gripMs = GetIntFromEditId(IDC_SF_EDIT_GRIP_MS, 450);
	const int gripMaxSteps = GetIntFromEditId(IDC_SF_EDIT_GRIP_MAXSTEPS, 12);
	const int stallDelta = GetIntFromEditId(IDC_SF_EDIT_STALL_DELTA, 10);
	const int stallAge = GetIntFromEditId(IDC_SF_EDIT_STALL_AGE, 800);
	const int gAttempts = GetIntFromEditId(IDC_SF_EDIT_GRASP_ATTEMPTS, 2);
	const int gAdvFail = GetIntFromEditId(IDC_SF_EDIT_GRASP_FAIL_ADV, 1);
	if (!inRange(gripJoint, 1, 6, L"Gripper.JointIndex")) return false;
	if (!inRange(gripOpen, 0, 1000, L"Gripper.OpenPos")) return false;
	if (!inRange(gripClose, 0, 1000, L"Gripper.ClosePos")) return false;
	if (!inRange(gripStep, 1, 1000, L"Gripper.CloseStepPos")) return false;
	if (!inRange(gripMs, 30, 60000, L"Gripper.CloseMoveTimeMs")) return false;
	if (!inRange(gripMaxSteps, 1, 1000, L"Gripper.MaxCloseSteps")) return false;
	if (!inRange(stallDelta, 0, 1000, L"Gripper.StallDetectDeltaPos")) return false;
	if (!inRange(stallAge, 0, 5000, L"Gripper.StallDetectMaxAgeMs")) return false;
	if (!inRange(gAttempts, 1, 20, L"Gripper.MaxAttempts")) return false;
	if (!inRange(gAdvFail, 0, 20, L"Gripper.AdvanceStepsOnFail")) return false;
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"JointIndex", gripJoint);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"OpenPos", gripOpen);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"ClosePos", gripClose);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"CloseStepPos", gripStep);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"CloseMoveTimeMs", gripMs);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"MaxCloseSteps", gripMaxSteps);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"EnableStallDetect", GetCheckFromId(IDC_SF_CHECK_STALL) ? 1 : 0);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"StallDetectDeltaPos", stallDelta);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"StallDetectMaxAgeMs", stallAge);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"MaxAttempts", gAttempts);
	app->WriteProfileInt(L"SeeAndFetch\\Gripper", L"AdvanceStepsOnFail", gAdvFail);

	// Place / Return
	int placeMode = 0;
	if (IsDlgButtonChecked(IDC_SF_RADIO_PLACE_REDDOT) == BST_CHECKED) placeMode = 1;
	int placeRangeMode = 1;
	if (IsDlgButtonChecked(IDC_SF_RADIO_PLACE_RANGE_ARUCO) == BST_CHECKED) placeRangeMode = 0;
	else if (IsDlgButtonChecked(IDC_SF_RADIO_PLACE_RANGE_AUTO) == BST_CHECKED) placeRangeMode = 2;

	const int placeVisionMode = GetIntFromEditId(IDC_SF_EDIT_PLACE_VMODE, 3);
	const int placeCenterStable = GetIntFromEditId(IDC_SF_EDIT_PLACE_CENTER_STABLE, 3);
	const int placeDepth = GetIntFromEditId(IDC_SF_EDIT_PLACE_DEPTH, 180);
	const int placeBoxArea = GetIntFromEditId(IDC_SF_EDIT_PLACE_BOX_AREA, 24000);
	const int placeBoxScale = GetIntFromEditId(IDC_SF_EDIT_PLACE_BOX_SCALE, 0);
	const int placeBoxStable = GetIntFromEditId(IDC_SF_EDIT_PLACE_BOX_STABLE, 3);
	const int placeBoxJump = GetIntFromEditId(IDC_SF_EDIT_PLACE_BOX_JUMP, 20000);
	const int placeMaxDown = GetIntFromEditId(IDC_SF_EDIT_PLACE_MAX_DOWN, 30);
	const int placeJ2Down = GetIntFromEditId(IDC_SF_EDIT_PLACE_J2DOWN_MDEG, 2000);
	const int placeSignJ2Down = GetIntFromEditId(IDC_SF_EDIT_PLACE_SIGN_J2DOWN, +1);
	const int placeMaxAttempts = GetIntFromEditId(IDC_SF_EDIT_PLACE_MAX_ATTEMPTS, 2);
	const int placeRetryRetreat = GetIntFromEditId(IDC_SF_EDIT_PLACE_RETRY_RETREAT, 8);
	const int retreatSteps = GetIntFromEditId(IDC_SF_EDIT_RETREAT_STEPS, 6);
	const int retMs = GetIntFromEditId(IDC_SF_EDIT_RETURN_MS, 1200);
	if (!inRange(placeMode, 0, 1, L"Place.Mode")) return false;
	if (!inRange(placeVisionMode, 0, 10, L"Place.VisionMode")) return false;
	if (!inRange(placeCenterStable, 1, 120, L"Place.CenterStableFrames")) return false;
	if (!inRange(placeRangeMode, 0, 2, L"Place.RangeMode")) return false;
	if (!inRange(placeDepth, 1, 5000, L"Place.PlaceDepthMm")) return false;
	if (!inRange(placeBoxArea, 0, 5000000, L"Place.PlaceBoxAreaPx2")) return false;
	if (!inRange(placeBoxScale, 0, 10000, L"Place.PlaceBoxScale_milli")) return false;
	if (!inRange(placeBoxStable, 1, 120, L"Place.BoxStableFrames")) return false;
	if (!inRange(placeBoxJump, 0, 5000000, L"Place.BoxAreaMaxJumpPx2")) return false;
	if (!inRange(placeMaxDown, 1, 10000, L"Place.MaxDownSteps")) return false;
	if (!inRange(placeJ2Down, 0, 90000, L"Place.J2DownStepDeg_milli")) return false;
	if (!(placeSignJ2Down == 1 || placeSignJ2Down == -1)) { outWhy = L"Place.SignJ2Down 只能为 +1 或 -1。"; return false; }
	if (!inRange(placeMaxAttempts, 1, 50, L"Place.MaxAttempts")) return false;
	if (!inRange(placeRetryRetreat, 0, 200, L"Place.RetryRetreatSteps")) return false;
	if (!inRange(retreatSteps, 0, 10000, L"Place.RetreatSteps")) return false;
	if (!inRange(retMs, 30, 60000, L"Return.ReturnTimeMs")) return false;
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"Mode", placeMode);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"VisionMode", placeVisionMode);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"CenterStableFrames", placeCenterStable);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"RangeMode", placeRangeMode);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"PlaceDepthMm", placeDepth);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"PlaceBoxAreaPx2", placeBoxArea);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"PlaceBoxScale_milli", placeBoxScale);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"BoxStableFrames", placeBoxStable);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"BoxAreaMaxJumpPx2", placeBoxJump);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"MaxDownSteps", placeMaxDown);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"J2DownStepDeg_milli", placeJ2Down);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"SignJ2Down", placeSignJ2Down);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"MaxAttempts", placeMaxAttempts);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"RetryRetreatSteps", placeRetryRetreat);
	app->WriteProfileInt(L"SeeAndFetch\\Place", L"RetreatSteps", retreatSteps);
	app->WriteProfileInt(L"SeeAndFetch\\Return", L"ReturnToStartPose", GetCheckFromId(IDC_SF_CHECK_RETURN_START) ? 1 : 0);
	app->WriteProfileInt(L"SeeAndFetch\\Return", L"ReturnTimeMs", retMs);

	return true;
}

void CSeeAndFetchDiagPage::OnSize(UINT nType, int cx, int cy)
{
	CPropertyPage::OnSize(nType, cx, cy);
	UpdateScrollBars();
}

void CSeeAndFetchDiagPage::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	SCROLLINFO si = {};
	si.cbSize = sizeof(SCROLLINFO);
	si.fMask = SIF_ALL;
	GetScrollInfo(SB_VERT, &si);

	int nNewPos = si.nPos;
	switch (nSBCode)
	{
	case SB_TOP: nNewPos = si.nMin; break;
	case SB_BOTTOM: nNewPos = si.nMax; break;
	case SB_LINEUP: nNewPos -= 20; break;
	case SB_LINEDOWN: nNewPos += 20; break;
	case SB_PAGEUP: nNewPos -= si.nPage; break;
	case SB_PAGEDOWN: nNewPos += si.nPage; break;
	case SB_THUMBTRACK: nNewPos = nPos; break;
	default: break;
	}

	// Clamp
	const int maxPos = si.nMax - (int)si.nPage + 1;
	if (nNewPos < si.nMin) nNewPos = si.nMin;
	if (nNewPos > maxPos) nNewPos = maxPos;
	if (maxPos < 0) nNewPos = 0;

	ApplyScroll(nNewPos);

	CPropertyPage::OnVScroll(nSBCode, nPos, pScrollBar);
}

BOOL CSeeAndFetchDiagPage::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	if (zDelta != 0)
	{
		// 每次滚轮移动 3 行
		const int lineSize = 20;
		const int nLines = 3;
		int delta = 0;
		if (zDelta > 0) delta = -lineSize * nLines; // 向上滚，位置减小
		else delta = lineSize * nLines;             // 向下滚，位置增加

		SCROLLINFO si = {};
		si.cbSize = sizeof(SCROLLINFO);
		si.fMask = SIF_ALL;
		GetScrollInfo(SB_VERT, &si);

		int nNewPos = m_scrollPos + delta;
		const int maxPos = si.nMax - (int)si.nPage + 1;
		if (nNewPos < si.nMin) nNewPos = si.nMin;
		if (nNewPos > maxPos) nNewPos = maxPos;
		if (maxPos < 0) nNewPos = 0;

		ApplyScroll(nNewPos);
	}
	return CPropertyPage::OnMouseWheel(nFlags, zDelta, pt);
}

void CSeeAndFetchDiagPage::UpdateScrollBars()
{
	CRect rc;
	GetClientRect(&rc);
	const int h = rc.Height();

	if (h <= 0) return;

	SCROLLINFO si = {};
	si.cbSize = sizeof(SCROLLINFO);
	si.fMask = SIF_ALL;
	si.nMin = 0;
	si.nMax = m_totalHeight;
	si.nPage = h;
	si.nPos = m_scrollPos;

	// 如果内容高度 > 窗口高度，则显示滚动条
	if (m_totalHeight > h)
	{
		SetScrollInfo(SB_VERT, &si, TRUE);
		ShowScrollBar(SB_VERT, TRUE);

		// 确保滚动位置有效
		const int maxPos = m_totalHeight - h;
		if (m_scrollPos > maxPos)
		{
			ApplyScroll(maxPos);
		}
	}
	else
	{
		// 不需要滚动条，重置位置
		if (m_scrollPos != 0)
		{
			ApplyScroll(0);
		}
		si.nPos = 0;
		SetScrollInfo(SB_VERT, &si, TRUE);
		ShowScrollBar(SB_VERT, FALSE);
	}
}
