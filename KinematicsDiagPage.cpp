#include "pch.h"

#include "KinematicsDiagPage.h"

#include "AppMessages.h"
#include "KinematicsConfig.h"
#include "ToolConfig.h"
#include "Resource.h"

#include <algorithm>
#include <cmath>

IMPLEMENT_DYNAMIC(CKinematicsDiagPage, CPropertyPage)

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
}

CKinematicsDiagPage::CKinematicsDiagPage()
	: CPropertyPage(IDD_PAGE_KINEMATICS, IDS_TAB_KINEMATICS)
{
}

CKinematicsDiagPage::~CKinematicsDiagPage()
{
}

void CKinematicsDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_KIN_EDIT_L_BASE, m_editLBase);
	DDX_Control(pDX, IDC_KIN_EDIT_L_ARM1, m_editLArm1);
	DDX_Control(pDX, IDC_KIN_EDIT_L_ARM2, m_editLArm2);
	DDX_Control(pDX, IDC_KIN_EDIT_L_WRIST, m_editLWrist);
	DDX_Control(pDX, IDC_KIN_EDIT_L_CAM, m_editLCam);

	DDX_Control(pDX, IDC_KIN_EDIT_J5CAM_X, m_editJ5CamX);
	DDX_Control(pDX, IDC_KIN_EDIT_J5CAM_Y, m_editJ5CamY);
	DDX_Control(pDX, IDC_KIN_EDIT_J5CAM_Z, m_editJ5CamZ);
	DDX_Control(pDX, IDC_KIN_EDIT_CAMGRIP_X, m_editCamGripX);
	DDX_Control(pDX, IDC_KIN_EDIT_CAMGRIP_Y, m_editCamGripY);
	DDX_Control(pDX, IDC_KIN_EDIT_CAMGRIP_Z, m_editCamGripZ);
}

BEGIN_MESSAGE_MAP(CKinematicsDiagPage, CPropertyPage)
	ON_BN_CLICKED(IDC_KIN_BTN_LOAD, &CKinematicsDiagPage::OnBnClickedLoad)
	ON_BN_CLICKED(IDC_KIN_BTN_SAVE, &CKinematicsDiagPage::OnBnClickedSaveApply)
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CKinematicsDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CKinematicsDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();
	InitJointMapControls();
	LoadFromProfileToUi();
	return TRUE;
}

LRESULT CKinematicsDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	if (!GetSafeHwnd()) return 0;
	InitJointMapControls();
	LoadFromProfileToUi();
	return 0;
}

void CKinematicsDiagPage::OnBnClickedLoad()
{
	InitJointMapControls();
	LoadFromProfileToUi();
}

void CKinematicsDiagPage::OnBnClickedSaveApply()
{
	std::wstring why;
	if (!SaveFromUiToProfile(why))
	{
		CString msg(why.c_str());
		AfxMessageBox(msg.IsEmpty() ? L"保存失败：参数无效。" : msg);
		return;
	}

	// 广播：让主界面/Jog/其它诊断页立即刷新
	BroadcastSettingsImportedToThread();
	AfxMessageBox(L"已保存并应用。建议进行一次轻微 Jog 验证是否可达。");
}

int CKinematicsDiagPage::GetIntFromEdit(const CEdit& edit, int fallback) const
{
	CString txt;
	const_cast<CEdit&>(edit).GetWindowTextW(txt);
	txt.Trim();
	if (txt.IsEmpty()) return fallback;
	return _wtoi(txt);
}

void CKinematicsDiagPage::SetIntToEdit(CEdit& edit, int v)
{
	CString txt;
	txt.Format(L"%d", v);
	edit.SetWindowTextW(txt);
}

void CKinematicsDiagPage::InitJointMapControls()
{
	if (m_bJointMapInited) return;
	if (!GetSafeHwnd()) return;

	for (int j = 1; j <= KinematicsConfig::kJointCount; j++)
	{
		// 资源ID按 joint 连续排列（见 Resource.h）
		m_editJPos0[j].SubclassDlgItem(IDC_KIN_EDIT_J1_POS0 + (j - 1), this);
		m_editJPosPlus[j].SubclassDlgItem(IDC_KIN_EDIT_J1_POSPLUS + (j - 1), this);
		m_editJPlusDeg[j].SubclassDlgItem(IDC_KIN_EDIT_J1_PLUSDEG + (j - 1), this);
		m_editJZeroOffMdeg[j].SubclassDlgItem(IDC_KIN_EDIT_J1_ZEROOFF_MDEG + (j - 1), this);
		m_checkJPhysInv[j].SubclassDlgItem(IDC_KIN_CHECK_J1_PHYSINV + (j - 1), this);
	}

	m_bJointMapInited = true;
}

void CKinematicsDiagPage::LoadFromProfileToUi()
{
	KinematicsConfig kc;
	kc.LoadAll();
	const auto& L = kc.Links();

	SetIntToEdit(m_editLBase, (int)std::lround(L.L_base));
	SetIntToEdit(m_editLArm1, (int)std::lround(L.L_arm1));
	SetIntToEdit(m_editLArm2, (int)std::lround(L.L_arm2));
	SetIntToEdit(m_editLWrist, (int)std::lround(L.L_wrist));
	SetIntToEdit(m_editLCam, (int)std::lround(L.L_cam));

	ToolConfig tc;
	tc.LoadAll();
	const auto& j5c = tc.Joint5ToCam_Cam();
	const auto& c2g = tc.CamToGripper_Cam();

	SetIntToEdit(m_editJ5CamX, (int)std::lround(j5c.x));
	SetIntToEdit(m_editJ5CamY, (int)std::lround(j5c.y));
	SetIntToEdit(m_editJ5CamZ, (int)std::lround(j5c.z));
	SetIntToEdit(m_editCamGripX, (int)std::lround(c2g.x));
	SetIntToEdit(m_editCamGripY, (int)std::lround(c2g.y));
	SetIntToEdit(m_editCamGripZ, (int)std::lround(c2g.z));

	// Joint mapping（原始对应关系参数）
	for (int j = 1; j <= KinematicsConfig::kJointCount; j++)
	{
		const auto& jc = kc.GetJoint(j);
		SetIntToEdit(m_editJPos0[j], jc.posAt0Deg);
		SetIntToEdit(m_editJPosPlus[j], jc.posAtPlusDeg);
		SetIntToEdit(m_editJPlusDeg[j], jc.plusDeg);
		SetIntToEdit(m_editJZeroOffMdeg[j], (int)std::lround(jc.zeroOffsetDeg * 1000.0));
		m_checkJPhysInv[j].SetCheck(jc.physicalInvert ? BST_CHECKED : BST_UNCHECKED);
	}
}

bool CKinematicsDiagPage::SaveFromUiToProfile(std::wstring& outWhy)
{
	outWhy.clear();

	auto validPos = [&](int v, int minV, int maxV, const wchar_t* name) -> bool
	{
		if (v < minV || v > maxV)
		{
			outWhy = std::wstring(name) + L" 超出合理范围。";
			return false;
		}
		return true;
	};

	const int lBase = GetIntFromEdit(m_editLBase, 80);
	const int lArm1 = GetIntFromEdit(m_editLArm1, 100);
	const int lArm2 = GetIntFromEdit(m_editLArm2, 95);
	const int lWrist = GetIntFromEdit(m_editLWrist, 95);
	const int lCam = GetIntFromEdit(m_editLCam, 55);

	// 基础保护：避免输入 0/负数导致 IK/FK 崩坏
	if (!validPos(lBase, 10, 500, L"L_base")) return false;
	if (!validPos(lArm1, 10, 500, L"L_arm1")) return false;
	if (!validPos(lArm2, 10, 500, L"L_arm2")) return false;
	if (!validPos(lWrist, 10, 500, L"L_wrist")) return false;
	if (!validPos(lCam, 0, 300, L"L_cam")) return false;

	KinematicsConfig kc;
	kc.LoadAll();
	auto& L = kc.Links();
	L.L_base = (double)lBase;
	L.L_arm1 = (double)lArm1;
	L.L_arm2 = (double)lArm2;
	L.L_wrist = (double)lWrist;
	L.L_cam = (double)lCam;

	// Joint mapping（原始对应关系参数）
	auto validJointInt = [&](int v, int minV, int maxV, const wchar_t* name) -> bool
	{
		if (v < minV || v > maxV)
		{
			outWhy = std::wstring(name) + L" 超出合理范围。";
			return false;
		}
		return true;
	};

	for (int j = 1; j <= KinematicsConfig::kJointCount; j++)
	{
		const int pos0 = GetIntFromEdit(m_editJPos0[j], kc.GetJoint(j).posAt0Deg);
		const int posPlus = GetIntFromEdit(m_editJPosPlus[j], kc.GetJoint(j).posAtPlusDeg);
		int plusDeg = GetIntFromEdit(m_editJPlusDeg[j], kc.GetJoint(j).plusDeg);
		if (plusDeg == 0) plusDeg = 45; // 防止除0
		const int zeroMdeg = GetIntFromEdit(m_editJZeroOffMdeg[j], (int)std::lround(kc.GetJoint(j).zeroOffsetDeg * 1000.0));
		const bool physInv = (m_checkJPhysInv[j].GetCheck() == BST_CHECKED);

		if (!validJointInt(pos0, 0, 1000, L"Pos0")) return false;
		if (!validJointInt(posPlus, 0, 1000, L"Pos+")) return false;
		if (!validJointInt(plusDeg, 1, 180, L"Deg+")) return false;
		if (!validJointInt(zeroMdeg, -90000, 90000, L"Zero(mdeg)")) return false;

		auto& jc = kc.GetJoint(j);
		jc.posAt0Deg = pos0;
		jc.posAtPlusDeg = posPlus;
		jc.plusDeg = plusDeg;
		jc.zeroOffsetDeg = (double)zeroMdeg / 1000.0;
		jc.physicalInvert = physInv;
	}

	// 一次性保存（links + joints + safety）
	kc.SaveAll();

	ToolConfig tc;
	tc.LoadAll();
	auto& j5c = tc.Joint5ToCam_Cam();
	auto& c2g = tc.CamToGripper_Cam();
	j5c.x = (double)GetIntFromEdit(m_editJ5CamX, (int)j5c.x);
	j5c.y = (double)GetIntFromEdit(m_editJ5CamY, (int)j5c.y);
	j5c.z = (double)GetIntFromEdit(m_editJ5CamZ, (int)j5c.z);
	c2g.x = (double)GetIntFromEdit(m_editCamGripX, (int)c2g.x);
	c2g.y = (double)GetIntFromEdit(m_editCamGripY, (int)c2g.y);
	c2g.z = (double)GetIntFromEdit(m_editCamGripZ, (int)c2g.z);
	tc.SaveAll();

	return true;
}


