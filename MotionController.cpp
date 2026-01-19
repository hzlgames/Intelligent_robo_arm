#include "pch.h"

#include "MotionController.h"

#include "ArmCommsService.h"
#include "ArmProtocol.h"

#include <algorithm>
#include <cstdio>

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

bool MotionController::MoveServoAbs(int servoId, int pos, int timeMs)
{
	std::vector<std::pair<int, int>> v;
	v.push_back({ servoId, pos });
	return MoveServosAbs(v, timeMs);
}

bool MotionController::MoveServosAbs(const std::vector<std::pair<int, int>>& servoToPos, int timeMs)
{
	std::vector<ArmProtocol::ServoTarget> servos;
	servos.reserve(servoToPos.size());
	for (const auto& sp : servoToPos)
	{
		const int id = sp.first;
		const int rawPos = sp.second;
		if (id < 1 || id > 6) continue;
		ArmProtocol::ServoTarget st;
		st.id = static_cast<uint8_t>(id);
		st.position = static_cast<uint16_t>(ClampPos(rawPos, 0, 1000));
		servos.push_back(st);
	}
	if (servos.empty())
	{
		return false;
	}
	if (timeMs < 0) timeMs = 0;
	if (timeMs > 60000) timeMs = 60000;
	ArmCommsService::Instance().EnqueueMove(ArmProtocol::PackMove(servos, static_cast<uint16_t>(timeMs)));
	return true;
}

bool MotionController::MoveJointsAbs(const std::vector<std::pair<int, int>>& jointToPos, int timeMs)
{
	std::vector<ArmProtocol::ServoTarget> servos;
	if (!BuildServoTargetsFromJoints(jointToPos, servos))
	{
		return false;
	}
	std::vector<std::pair<int, int>> servoToPos;
	servoToPos.reserve(servos.size());
	for (const auto& st : servos)
	{
		servoToPos.push_back({ (int)st.id, (int)st.position });
	}
	return MoveServosAbs(servoToPos, timeMs);
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
	ArmCommsService::Instance().EnqueueRead(ArmProtocol::PackReadPosition(ids));
}

