#include "pch.h"

#include "KinematicsOverlayService.h"

KinematicsOverlayService& KinematicsOverlayService::Instance()
{
	static KinematicsOverlayService s;
	return s;
}

void KinematicsOverlayService::UpdateJog(bool jogActive, double joyX, double joyY,
                                         const ArmKinematics::PoseTarget& target,
                                         bool ikOk, const std::wstring& ikWhy)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_s.jogActive = jogActive;
	m_s.joyX = joyX;
	m_s.joyY = joyY;
	m_s.target = target;
	m_s.ikOk = ikOk;
	m_s.ikWhy = ikWhy;
}

void KinematicsOverlayService::UpdateSerialStats(unsigned fps, unsigned sinceMs)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_s.sendFps = fps;
	m_s.sinceLastSendMs = sinceMs;
}

void KinematicsOverlayService::UpdateVisualServo(bool enabled, bool applied, bool active, int mode,
                                                 double errU, double errV, double advance,
                                                 const std::wstring& why)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_s.vsEnabled = enabled;
	m_s.vsApplied = applied;
	m_s.vsActive = active;
	m_s.vsMode = mode;
	m_s.vsErrU = errU;
	m_s.vsErrV = errV;
	m_s.vsAdvance = advance;
	m_s.vsWhy = why;
}

KinematicsOverlayService::Snapshot KinematicsOverlayService::GetSnapshot() const
{
	std::lock_guard<std::mutex> lk(m_mu);
	return m_s;
}



