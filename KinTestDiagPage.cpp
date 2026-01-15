#include "pch.h"

#include "KinTestDiagPage.h"

#include "AppMessages.h"
#include "ArmCommsService.h"
#include "ArmKinematics.h"
#include "KinematicsConfig.h"
#include "MotionConfig.h"
#include "MotionController.h"
#include "Resource.h"

#include <cmath>
#include <sstream>
#include <vector>

IMPLEMENT_DYNAMIC(CKinTestDiagPage, CPropertyPage)

namespace
{
	constexpr double kPi = 3.14159265358979323846;
	double DegToRad(double d) { return d * (kPi / 180.0); }

	// 以 ServoPosToJointRad 的逻辑为准：physicalDeg -> (minus zeroOffset) -> apply sign(invert) -> rad
	bool PhysicalDegToJointRad(const KinematicsConfig& kc,
	                           const MotionConfig& mc,
	                           int joint,
	                           double physicalDeg,
	                           double& outRad)
	{
		if (joint < 1 || joint > KinematicsConfig::kJointCount) return false;
		const auto& jc = kc.GetJoint(joint);
		// 注意：Physical 模式下用户输入的是“物理角”。若该关节 physicalInvert 开启，则先把物理角取反再进入模型坐标。
		if (jc.physicalInvert) physicalDeg = -physicalDeg;
		double degZeroAdjusted = physicalDeg - jc.zeroOffsetDeg;
		int sign = KinematicsConfig::AxisSignForJoint(joint);
		if (mc.Get(joint).invert) sign = -sign;
		outRad = DegToRad(degZeroAdjusted) * (double)sign;
		return true;
	}
}

CKinTestDiagPage::CKinTestDiagPage()
	: CPropertyPage(IDD_PAGE_KIN_TEST, IDS_TAB_KIN_TEST)
{
}

CKinTestDiagPage::~CKinTestDiagPage()
{
}

void CKinTestDiagPage::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_KTEST_EDIT_J1, m_editJ1);
	DDX_Control(pDX, IDC_KTEST_EDIT_J2, m_editJ2);
	DDX_Control(pDX, IDC_KTEST_EDIT_J3, m_editJ3);
	DDX_Control(pDX, IDC_KTEST_EDIT_J4, m_editJ4);
	DDX_Control(pDX, IDC_KTEST_EDIT_J5, m_editJ5);
	DDX_Control(pDX, IDC_KTEST_EDIT_TIME, m_editTime);
	DDX_Control(pDX, IDC_KTEST_CHECK_PHYSICAL, m_checkPhysical);
	DDX_Control(pDX, IDC_KTEST_STATIC_OUT, m_staticOut);
}

BEGIN_MESSAGE_MAP(CKinTestDiagPage, CPropertyPage)
	ON_BN_CLICKED(IDC_KTEST_BTN_CALC, &CKinTestDiagPage::OnBnClickedCalc)
	ON_BN_CLICKED(IDC_KTEST_BTN_EXEC, &CKinTestDiagPage::OnBnClickedExec)
	ON_BN_CLICKED(IDC_KTEST_BTN_CLEAR, &CKinTestDiagPage::OnBnClickedClear)
	ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CKinTestDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CKinTestDiagPage::OnInitDialog()
{
	CPropertyPage::OnInitDialog();

	// defaults
	SetDoubleToEdit(m_editJ1, 0.0);
	SetDoubleToEdit(m_editJ2, 0.0);
	SetDoubleToEdit(m_editJ3, 0.0);
	SetDoubleToEdit(m_editJ4, 0.0);
	SetDoubleToEdit(m_editJ5, 0.0);
	SetDoubleToEdit(m_editTime, 800.0);
	m_checkPhysical.SetCheck(BST_CHECKED);
	SetOutputText(L"输入 J1..J5 角度（度），点击“计算FK”。\r\n默认按“舵机物理角”解释（含 zeroOffset/Invert/轴向符号）。");

	return TRUE;
}

LRESULT CKinTestDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	if (!GetSafeHwnd()) return 0;
	// 不强制改用户输入，仅提示“配置已更新”
	SetOutputText(L"[INFO] 已检测到设置刷新（标定/几何参数可能已变化）。\r\n请重新点击“计算FK”以使用最新参数。");
	return 0;
}

double CKinTestDiagPage::GetDoubleFromEdit(const CEdit& edit, double fallback) const
{
	CString txt;
	const_cast<CEdit&>(edit).GetWindowTextW(txt);
	txt.Trim();
	if (txt.IsEmpty()) return fallback;
	return _wtof(txt);
}

void CKinTestDiagPage::SetDoubleToEdit(CEdit& edit, double v)
{
	CString s;
	s.Format(L"%.2f", v);
	edit.SetWindowTextW(s);
}

void CKinTestDiagPage::SetOutputText(const std::wstring& text)
{
	if (!m_staticOut.GetSafeHwnd()) return;
	m_staticOut.SetWindowTextW(text.c_str());
}

void CKinTestDiagPage::OnBnClickedClear()
{
	SetDoubleToEdit(m_editJ1, 0.0);
	SetDoubleToEdit(m_editJ2, 0.0);
	SetDoubleToEdit(m_editJ3, 0.0);
	SetDoubleToEdit(m_editJ4, 0.0);
	SetDoubleToEdit(m_editJ5, 0.0);
	SetDoubleToEdit(m_editTime, 800.0);
	SetOutputText(L"已清空。");
}

void CKinTestDiagPage::OnBnClickedCalc()
{
	Compute(false);
}

void CKinTestDiagPage::OnBnClickedExec()
{
	Compute(true);
}

void CKinTestDiagPage::Compute(bool doExec)
{
	KinematicsConfig kc;
	kc.LoadAll();

	MotionConfig mc;
	mc.LoadAll();

	const double d1 = GetDoubleFromEdit(m_editJ1, 0.0);
	const double d2 = GetDoubleFromEdit(m_editJ2, 0.0);
	const double d3 = GetDoubleFromEdit(m_editJ3, 0.0);
	const double d4 = GetDoubleFromEdit(m_editJ4, 0.0);
	const double d5 = GetDoubleFromEdit(m_editJ5, 0.0);
	const int timeMs = (int)std::lround(GetDoubleFromEdit(m_editTime, 800.0));

	const bool physical = (m_checkPhysical.GetCheck() == BST_CHECKED);

	ArmKinematics::JointAnglesRad q{};
	for (int i = 0; i <= ArmKinematics::kJointCount; i++) q.q[i] = 0.0;

	bool ok = true;
	if (physical)
	{
		ok = ok && PhysicalDegToJointRad(kc, mc, 1, d1, q.q[1]);
		ok = ok && PhysicalDegToJointRad(kc, mc, 2, d2, q.q[2]);
		ok = ok && PhysicalDegToJointRad(kc, mc, 3, d3, q.q[3]);
		ok = ok && PhysicalDegToJointRad(kc, mc, 4, d4, q.q[4]);
		ok = ok && PhysicalDegToJointRad(kc, mc, 5, d5, q.q[5]);
	}
	else
	{
		// 用户输入即为“模型关节角 q（度）”：直接 deg->rad，不额外应用 zeroOffset/invert/sign
		q.q[1] = DegToRad(d1);
		q.q[2] = DegToRad(d2);
		q.q[3] = DegToRad(d3);
		q.q[4] = DegToRad(d4);
		q.q[5] = DegToRad(d5);
	}

	if (!ok)
	{
		SetOutputText(L"[ERR] 角度转换失败（请检查关节编号/配置）。");
		return;
	}

	const auto pose = ArmKinematics::ForwardKinematics(kc, q);

	// 同时给出“按当前标定推算的舵机pos”，方便核对角度标定
	std::wstringstream ss;
	ss.setf(std::ios::fixed);
	ss.precision(1);
	ss << L"FK Pose (Base):\r\n";
	ss << L"  X=" << pose.x_mm << L" mm\r\n";
	ss << L"  Y=" << pose.y_mm << L" mm\r\n";
	ss << L"  Z=" << pose.z_mm << L" mm\r\n";
	ss << L"  Pitch=" << pose.pitch_deg << L" deg\r\n";
	ss << L"\r\n";

	ss << L"JointRad（模型内部使用，rad）:\r\n";
	ss.precision(3);
	ss << L"  q1=" << q.q[1] << L"  q2=" << q.q[2] << L"  q3=" << q.q[3] << L"  q4=" << q.q[4] << L"  q5=" << q.q[5] << L"\r\n";
	ss << L"\r\n";

	ss << L"Predicted ServoPos（0..1000，按当前两点标定+Invert/轴向）:\r\n";
	std::vector<std::pair<int, int>> jointToPos;
	jointToPos.reserve(ArmKinematics::kJointCount);
	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		int pos = 0;
		if (ArmKinematics::JointRadToServoPos(kc, &mc, j, q.q[j], pos))
		{
			ss << L"  J" << j << L": " << pos << L"\r\n";
			jointToPos.push_back({ j, pos });
		}
		else
		{
			ss << L"  J" << j << L": (failed)\r\n";
		}
	}

	if (doExec)
	{
		ss << L"\r\n";
		if (!ArmCommsService::Instance().IsConnected())
		{
			ss << L"[ERR] 未连接串口：请在主界面或“诊断->运动”先连接，再执行。\r\n";
			SetOutputText(ss.str());
			return;
		}

		// 直接下发关节目标（不经过 IK）：角度 -> ServoPos -> MoveJointsAbs
		ArmCommsService::Instance().ClearMoveQueue();
		MotionController motion;
		motion.LoadConfig();
		const int t = std::max(0, std::min(timeMs, 60000));
		if (!motion.MoveJointsAbs(jointToPos, t))
		{
			ss << L"[ERR] 下发失败：可能 ServoId 未配置，或没有有效关节目标。\r\n";
		}
		else
		{
			ss << L"[OK] 已下发目标角度（按当前标定映射）。timeMs=" << t << L"\r\n";
		}
	}

	SetOutputText(ss.str());
}


