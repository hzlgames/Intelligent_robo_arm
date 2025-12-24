#include "pch.h"

#include "MotionConfig.h"

#include <afxwin.h>

MotionConfig::MotionConfig()
{
	ResetDefaults();
}

void MotionConfig::ResetDefaults()
{
	for (int j = 0; j <= kJointCount; j++)
	{
		m_joints[j] = Joint{};
	}
}

std::wstring MotionConfig::SectionForJoint(int jointIndex)
{
	CString s;
	s.Format(L"Motion\\J%d", jointIndex);
	return std::wstring(s.GetString());
}

std::wstring MotionConfig::JointName(int jointIndex)
{
	switch (jointIndex)
	{
	case 1: return L"J1(BaseYaw)";
	case 2: return L"J2(Shoulder)";
	case 3: return L"J3(Elbow)";
	case 4: return L"J4(Wrist)";
	case 5: return L"J5(HeadPan)";
	case 6: return L"J6(HeadTilt)";
	default: return L"J?(Unknown)";
	}
}

void MotionConfig::LoadAll()
{
	for (int j = 1; j <= kJointCount; j++)
	{
		const std::wstring sec = SectionForJoint(j);
		Joint d{};
		d.servoId = AfxGetApp()->GetProfileInt(sec.c_str(), L"ServoId", 0);
		d.minPos = AfxGetApp()->GetProfileInt(sec.c_str(), L"Min", 0);
		d.maxPos = AfxGetApp()->GetProfileInt(sec.c_str(), L"Max", 1000);
		d.homePos = AfxGetApp()->GetProfileInt(sec.c_str(), L"Home", 500);
		d.invert = (AfxGetApp()->GetProfileInt(sec.c_str(), L"Invert", 0) != 0);
		m_joints[j] = d;
	}
}

void MotionConfig::SaveAll() const
{
	for (int j = 1; j <= kJointCount; j++)
	{
		const std::wstring sec = SectionForJoint(j);
		const Joint& d = m_joints[j];
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"ServoId", d.servoId);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"Min", d.minPos);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"Max", d.maxPos);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"Home", d.homePos);
		AfxGetApp()->WriteProfileInt(sec.c_str(), L"Invert", d.invert ? 1 : 0);
	}
}

void MotionConfig::ImportLegacyServoLimitsForAssignedJoints()
{
	for (int j = 1; j <= kJointCount; j++)
	{
		Joint& d = m_joints[j];
		if (d.servoId < 1 || d.servoId > 6) continue;
		CString keyMin, keyMax;
		keyMin.Format(L"Min%d", d.servoId);
		keyMax.Format(L"Max%d", d.servoId);
		d.minPos = AfxGetApp()->GetProfileInt(L"ServoLimits", keyMin, d.minPos);
		d.maxPos = AfxGetApp()->GetProfileInt(L"ServoLimits", keyMax, d.maxPos);
	}
}


