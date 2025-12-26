#include "pch.h"

#include "VisionOverlayService.h"

VisionOverlayService& VisionOverlayService::Instance()
{
	static VisionOverlayService s;
	return s;
}

void VisionOverlayService::Update(const Snapshot& s)
{
	std::lock_guard<std::mutex> lk(m_mu);
	m_s = s;
}

VisionOverlayService::Snapshot VisionOverlayService::GetSnapshot() const
{
	std::lock_guard<std::mutex> lk(m_mu);
	return m_s;
}


