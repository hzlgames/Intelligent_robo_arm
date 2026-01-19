#include "pch.h"

#include "KinematicsConfig.h"

#include <afxwin.h>
#include <algorithm>
#include <cmath>

namespace
{
	// 将 double 以定点数方式保存，避免 GetProfileString/浮点解析带来的本地化问题。
	// scale=1000 表示保留 0.001 精度，例如 deg*1000。
	int ReadScaledInt(const std::wstring& sec, const wchar_t* key, int fallback)
	{
		return AfxGetApp()->GetProfileInt(sec.c_str(), key, fallback);
	}

	void WriteScaledInt(const std::wstring& sec, const wchar_t* key, int v)
	{
		AfxGetApp()->WriteProfileInt(sec.c_str(), key, v);
	}

	double ReadScaledDouble(const std::wstring& sec, const wchar_t* key, double fallback, int scale)
	{
		const int fb = static_cast<int>(fallback * static_cast<double>(scale));
		const int v = ReadScaledInt(sec, key, fb);
		return static_cast<double>(v) / static_cast<double>(scale);
	}

	void WriteScaledDouble(const std::wstring& sec, const wchar_t* key, double v, int scale)
	{
		const int iv = static_cast<int>(v * static_cast<double>(scale));
		WriteScaledInt(sec, key, iv);
	}
}

KinematicsConfig::KinematicsConfig()
{
	// 默认值：来自 Reference/mechanics.md 的当前标定记录（后续你可随时改 ini 覆盖）。
	//
	// 关节线性标定表（两点）：
	// - k = (posAtPlusDeg - posAt0Deg) / plusDeg，方向由 k 的符号决定
	// - J1: 0°=500, +45°=690 => k > 0（正向）
	// - J2: 0°=500, +45°=320 => k < 0（反向，角度增大时 pos 减小）
	// - J3: 0°=500, +45°=680 => k > 0（正向）
	// - J4: 0°=500, +45°=700 => k > 0（正向）
	// - J5: 0°=500, +90°=900 => k > 0（正向）
	m_joints[1] = JointCalib{ 500, 690, 45, 0.0 };
	m_joints[2] = JointCalib{ 500, 320, 45, 0.0 };
	m_joints[3] = JointCalib{ 500, 680, 45, 0.0 };
	m_joints[4] = JointCalib{ 500, 700, 45, 0.0 };
	m_joints[5] = JointCalib{ 500, 900, 90, 0.0 };
}

std::wstring KinematicsConfig::SectionLinks()
{
	// 注意：使用与 MotionConfig 类似的“带反斜杠的 section”，便于 SettingsIo 导入导出统一处理。
	return L"Kinematics\\Links";
}

std::wstring KinematicsConfig::SectionForJoint(int nJoint)
{
	CString s;
	s.Format(L"Kinematics\\J%d", nJoint);
	return std::wstring(s.GetString());
}

std::wstring KinematicsConfig::SectionSafety()
{
	return L"Kinematics\\Safety";
}

void KinematicsConfig::LoadAll()
{
	// 1) 连杆长度（mm）：用整数保存即可，满足大多数标定精度。
	{
		const std::wstring sec = SectionLinks();
		m_links.L_base = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"L_base_mm", static_cast<int>(m_links.L_base)));
		m_links.L_arm1 = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"L_arm1_mm", static_cast<int>(m_links.L_arm1)));
		m_links.L_arm2 = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"L_arm2_mm", static_cast<int>(m_links.L_arm2)));
		m_links.L_wrist = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"L_wrist_mm", static_cast<int>(m_links.L_wrist)));
		m_links.L_cam = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"L_cam_mm", static_cast<int>(m_links.L_cam)));
	}

	// 1.5) Safety（用于手动 FPS 模式的积分限制）
	{
		const std::wstring sec = SectionSafety();
		const int defRMax = (int)std::lround(m_links.L_arm1 + m_links.L_arm2 + m_links.L_wrist);
		m_safety.pitchMinDeg = AfxGetApp()->GetProfileInt(sec.c_str(), L"PitchMinDeg", m_safety.pitchMinDeg);
		m_safety.pitchMaxDeg = AfxGetApp()->GetProfileInt(sec.c_str(), L"PitchMaxDeg", m_safety.pitchMaxDeg);
		m_safety.zMinMm = AfxGetApp()->GetProfileInt(sec.c_str(), L"ZMinMm", m_safety.zMinMm);
		m_safety.rMinMm = AfxGetApp()->GetProfileInt(sec.c_str(), L"RMinMm", m_safety.rMinMm);
		m_safety.rMaxMm = AfxGetApp()->GetProfileInt(sec.c_str(), L"RMaxMm", defRMax);

		if (m_safety.pitchMinDeg > m_safety.pitchMaxDeg) std::swap(m_safety.pitchMinDeg, m_safety.pitchMaxDeg);
		if (m_safety.zMinMm < 0) m_safety.zMinMm = 0;
		if (m_safety.rMinMm < 0) m_safety.rMinMm = 0;
		if (m_safety.rMaxMm < m_safety.rMinMm + 10) m_safety.rMaxMm = m_safety.rMinMm + 10;
	}

	// 2) 关节两点标定 + 零位偏置
	for (int j = 1; j <= kJointCount; j++)
	{
		const std::wstring sec = SectionForJoint(j);
		JointCalib c = m_joints[j];

		c.posAt0Deg = AfxGetApp()->GetProfileInt(sec.c_str(), L"PosAt0Deg", c.posAt0Deg);
		c.posAtPlusDeg = AfxGetApp()->GetProfileInt(sec.c_str(), L"PosAtPlusDeg", c.posAtPlusDeg);
		c.plusDeg = AfxGetApp()->GetProfileInt(sec.c_str(), L"PlusDeg", c.plusDeg);
		if (c.plusDeg == 0) c.plusDeg = 45; // 保护：避免除 0

		// zeroOffsetDeg 以 milli-degree 存储
		c.zeroOffsetDeg = ReadScaledDouble(sec, L"ZeroOffset_mdeg", c.zeroOffsetDeg, 1000);

		m_joints[j] = c;
	}
}

void KinematicsConfig::SaveAll() const
{
	// 1) 连杆长度（mm）
	{
		const std::wstring sec = SectionLinks();
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"L_base_mm", static_cast<int>(m_links.L_base));
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"L_arm1_mm", static_cast<int>(m_links.L_arm1));
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"L_arm2_mm", static_cast<int>(m_links.L_arm2));
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"L_wrist_mm", static_cast<int>(m_links.L_wrist));
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"L_cam_mm", static_cast<int>(m_links.L_cam));
	}

	// 1.5) Safety
	{
		const std::wstring sec = SectionSafety();
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"PitchMinDeg", m_safety.pitchMinDeg);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"PitchMaxDeg", m_safety.pitchMaxDeg);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"ZMinMm", m_safety.zMinMm);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"RMinMm", m_safety.rMinMm);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"RMaxMm", m_safety.rMaxMm);
	}

	// 2) 关节标定
	for (int j = 1; j <= kJointCount; j++)
	{
		const std::wstring sec = SectionForJoint(j);
		const JointCalib& c = m_joints[j];
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"PosAt0Deg", c.posAt0Deg);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"PosAtPlusDeg", c.posAtPlusDeg);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"PlusDeg", c.plusDeg);
		WriteScaledDouble(sec, L"ZeroOffset_mdeg", c.zeroOffsetDeg, 1000);
	}
}



