#include "pch.h"

#include "ArmStateEstimator.h"

#include "ArmCommsService.h"

bool ArmStateEstimator::Estimate(const MotionController& motion,
                                 const KinematicsConfig& kc,
                                 ArmState& outState,
                                 std::wstring* outWhy)
{
	if (outWhy) outWhy->clear();

	outState = ArmState{};

	const MotionConfig& mc = motion.Config();

	bool ok = true;
	std::wstring why;

	for (int j = 0; j <= ArmKinematics::kJointCount; j++)
	{
		outState.q.q[j] = 0.0;
	}

	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		const auto& jc = mc.Get(j);
		int pos = jc.homePos;
		if (jc.servoId >= 1 && jc.servoId <= 6)
		{
			uint16_t rb = 0;
			if (ArmCommsService::Instance().GetLastReadPos((uint8_t)jc.servoId, rb))
			{
				pos = (int)rb;
			}
		}

		double rad = 0.0;
		if (!ArmKinematics::ServoPosToJointRad(kc, &mc, j, pos, rad))
		{
			ok = false;
			rad = 0.0;
			if (why.empty())
			{
				why = L"关节标定不足：pos->rad 失败（请检查 KinematicsConfig 两点标定/PlusDeg）。";
			}
		}
		outState.q.q[j] = rad;
	}

	outState.yawRad = outState.q.q[1];
	outState.pitchRad = outState.q.q[2] + outState.q.q[3] + outState.q.q[4];
	outState.rollRad = outState.q.q[5];
	outState.joint5PoseBase = ArmKinematics::ForwardKinematics(kc, outState.q);
	outState.valid = ok;

	if (!ok && outWhy)
	{
		*outWhy = why;
	}

	return ok;
}


