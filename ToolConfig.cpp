#include "pch.h"

#include "ToolConfig.h"

#include <afxwin.h>

std::wstring ToolConfig::SectionOffsets()
{
	return L"Tool\\Offsets";
}

void ToolConfig::LoadAll()
{
	const std::wstring sec = SectionOffsets();

	m_joint5ToCam_Cam.x = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"Joint5ToCam_X_mm", (int)m_joint5ToCam_Cam.x));
	m_joint5ToCam_Cam.y = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"Joint5ToCam_Y_mm", (int)m_joint5ToCam_Cam.y));
	m_joint5ToCam_Cam.z = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"Joint5ToCam_Z_mm", (int)m_joint5ToCam_Cam.z));

	m_camToGripper_Cam.x = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"CamToGrip_X_mm", (int)m_camToGripper_Cam.x));
	m_camToGripper_Cam.y = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"CamToGrip_Y_mm", (int)m_camToGripper_Cam.y));
	m_camToGripper_Cam.z = static_cast<double>(AfxGetApp()->GetProfileInt(sec.c_str(), L"CamToGrip_Z_mm", (int)m_camToGripper_Cam.z));
}

void ToolConfig::SaveAll() const
{
	const std::wstring sec = SectionOffsets();

	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Joint5ToCam_X_mm", (int)m_joint5ToCam_Cam.x);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Joint5ToCam_Y_mm", (int)m_joint5ToCam_Cam.y);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"Joint5ToCam_Z_mm", (int)m_joint5ToCam_Cam.z);

	AfxGetApp()->WriteProfileInt(sec.c_str(), L"CamToGrip_X_mm", (int)m_camToGripper_Cam.x);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"CamToGrip_Y_mm", (int)m_camToGripper_Cam.y);
	AfxGetApp()->WriteProfileInt(sec.c_str(), L"CamToGrip_Z_mm", (int)m_camToGripper_Cam.z);
}









