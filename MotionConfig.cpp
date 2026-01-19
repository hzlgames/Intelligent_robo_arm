#include "pch.h"

#include "MotionConfig.h"

#include <afxwin.h>
#include <algorithm>

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

// ========== 统一限位访问接口实现 ==========

int MotionConfig::FindJointByServoId(int servoId) const
{
	if (servoId < 1 || servoId > 6) return 0;
	for (int j = 1; j <= kJointCount; j++)
	{
		if (m_joints[j].servoId == servoId)
		{
			return j;
		}
	}
	return 0;
}

bool MotionConfig::GetLimitsByServoId(int servoId, int& outMin, int& outMax) const
{
	const int j = FindJointByServoId(servoId);
	if (j == 0)
	{
		// 未绑定时返回默认值
		outMin = 0;
		outMax = 1000;
		return false;
	}
	outMin = m_joints[j].minPos;
	outMax = m_joints[j].maxPos;
	return true;
}

bool MotionConfig::SetLimitsByServoId(int servoId, int minPos, int maxPos)
{
	const int j = FindJointByServoId(servoId);
	if (j == 0) return false;

	m_joints[j].minPos = minPos;
	m_joints[j].maxPos = maxPos;

	// 立即保存到 profile
	const std::wstring sec = SectionForJoint(j);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Min", minPos);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Max", maxPos);
	return true;
}

bool MotionConfig::SetMinByServoId(int servoId, int minPos)
{
	const int j = FindJointByServoId(servoId);
	if (j == 0) return false;

	m_joints[j].minPos = minPos;
	const std::wstring sec = SectionForJoint(j);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Min", minPos);
	return true;
}

bool MotionConfig::SetMaxByServoId(int servoId, int maxPos)
{
	const int j = FindJointByServoId(servoId);
	if (j == 0) return false;

	m_joints[j].maxPos = maxPos;
	const std::wstring sec = SectionForJoint(j);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Max", maxPos);
	return true;
}

int MotionConfig::GetMinByServoId(int servoId) const
{
	const int j = FindJointByServoId(servoId);
	if (j == 0) return 0;
	return m_joints[j].minPos;
}

int MotionConfig::GetMaxByServoId(int servoId) const
{
	const int j = FindJointByServoId(servoId);
	if (j == 0) return 1000;
	return m_joints[j].maxPos;
}

int MotionConfig::ClampByServoId(int servoId, int pos, bool* outClamped) const
{
	int minV = 0, maxV = 1000;
	GetLimitsByServoId(servoId, minV, maxV);
	if (minV > maxV) std::swap(minV, maxV);

	if (pos < minV)
	{
		if (outClamped) *outClamped = true;
		return minV;
	}
	if (pos > maxV)
	{
		if (outClamped) *outClamped = true;
		return maxV;
	}
	if (outClamped) *outClamped = false;
	return pos;
}


