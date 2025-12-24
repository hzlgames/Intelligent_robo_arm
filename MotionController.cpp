#include "pch.h"

#include "MotionController.h"

#include "ArmCommsService.h"
#include "ArmProtocol.h"

#include <algorithm>

MotionController::MotionController()
{
	LoadConfig();
}

void MotionController::LoadConfig()
{
	m_cfg.LoadAll();
}

void MotionController::SaveConfig() const
{
	m_cfg.SaveAll();
}

void MotionController::ResetDefaults()
{
	m_cfg.ResetDefaults();
}

void MotionController::ImportLegacyServoLimitsForAssignedJoints()
{
	m_cfg.ImportLegacyServoLimitsForAssignedJoints();
}

int MotionController::ClampPos(int v, int minV, int maxV)
{
	if (minV > maxV) std::swap(minV, maxV);
	if (v < minV) return minV;
	if (v > maxV) return maxV;
	return v;
}

bool MotionController::BuildServoTargetsFromJoints(const std::vector<std::pair<int, int>>& jointToPos,
                                                   std::vector<ArmProtocol::ServoTarget>& out)
{
	out.clear();
	out.reserve(jointToPos.size());
	for (const auto& jp : jointToPos)
	{
		const int joint = jp.first;
		const int rawPos = jp.second;
		if (joint < 1 || joint > MotionConfig::kJointCount) continue;

		const auto& jc = m_cfg.Get(joint);
		if (jc.servoId < 1 || jc.servoId > 6) continue;

		const int safePos = ClampPos(rawPos, jc.minPos, jc.maxPos);
		ArmProtocol::ServoTarget st;
		st.id = static_cast<uint8_t>(jc.servoId);
		st.position = static_cast<uint16_t>(safePos);
		out.push_back(st);
	}
	return !out.empty();
}

bool MotionController::MoveJointAbs(int jointIndex, int pos, int timeMs)
{
	std::vector<std::pair<int, int>> v;
	v.push_back({ jointIndex, pos });
	return MoveJointsAbs(v, timeMs);
}

bool MotionController::MoveJointsAbs(const std::vector<std::pair<int, int>>& jointToPos, int timeMs)
{
	std::vector<ArmProtocol::ServoTarget> servos;
	if (!BuildServoTargetsFromJoints(jointToPos, servos))
	{
		return false;
	}
	if (timeMs < 0) timeMs = 0;
	if (timeMs > 60000) timeMs = 60000;
	ArmCommsService::Instance().EnqueueTx(ArmProtocol::PackMove(servos, static_cast<uint16_t>(timeMs)));
	return true;
}

bool MotionController::MoveHome(int timeMs)
{
	std::vector<std::pair<int, int>> joints;
	joints.reserve(MotionConfig::kJointCount);
	for (int j = 1; j <= MotionConfig::kJointCount; j++)
	{
		const auto& jc = m_cfg.Get(j);
		if (jc.servoId < 1 || jc.servoId > 6) continue;
		joints.push_back({ j, jc.homePos });
	}
	return MoveJointsAbs(joints, timeMs);
}

void MotionController::RequestReadAllAssigned()
{
	std::vector<uint8_t> ids;
	ids.reserve(MotionConfig::kJointCount);
	for (int j = 1; j <= MotionConfig::kJointCount; j++)
	{
		const int sid = m_cfg.Get(j).servoId;
		if (sid < 1 || sid > 6) continue;
		if (std::find(ids.begin(), ids.end(), static_cast<uint8_t>(sid)) == ids.end())
		{
			ids.push_back(static_cast<uint8_t>(sid));
		}
	}
	if (ids.empty())
	{
		// fallback: request 1..6
		ids = { 1,2,3,4,5,6 };
	}
	ArmCommsService::Instance().EnqueueTx(ArmProtocol::PackReadPosition(ids));
}

void MotionController::StartScript(std::vector<Keyframe> frames, bool loop)
{
	m_frames = std::move(frames);
	m_loop = loop;
	m_frameIndex = 0;
	m_playing = !m_frames.empty();
	m_nextDue = ::GetTickCount64();
}

void MotionController::StopScript()
{
	m_playing = false;
	m_frames.clear();
	m_frameIndex = 0;
	m_nextDue = 0;
}

void MotionController::Tick()
{
	if (!m_playing) return;
	if (m_frames.empty())
	{
		m_playing = false;
		return;
	}
	const ULONGLONG now = ::GetTickCount64();
	if (now < m_nextDue) return;

	const Keyframe& kf = m_frames[m_frameIndex];
	std::vector<std::pair<int, int>> joints;
	for (int j = 1; j <= MotionConfig::kJointCount; j++)
	{
		const int p = kf.jointPos[j];
		if (p < 0) continue;
		joints.push_back({ j, p });
	}
	(void)MoveJointsAbs(joints, kf.durationMs);

	// schedule next
	ULONGLONG delta = (kf.durationMs > 0) ? static_cast<ULONGLONG>(kf.durationMs) : 0ULL;
	m_nextDue = now + delta;

	m_frameIndex++;
	if (m_frameIndex >= m_frames.size())
	{
		if (m_loop)
		{
			m_frameIndex = 0;
		}
		else
		{
			StopScript();
		}
	}
}


