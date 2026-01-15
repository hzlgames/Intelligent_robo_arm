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
			DWORD age = 0;
			const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
			if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
			{
				pos = (int)rb;
			}
			else
			{
				// 连接状态下：读回过期/缺失时直接标记 invalid，避免 VS 使用“homePos推算的姿态”导致几何方向突变
				if (ArmCommsService::Instance().IsConnected())
				{
					ok = false;
					if (why.empty())
					{
						why = L"读回过期/缺失：为避免姿态估计突变，暂不提供有效姿态（valid=false）。";
					}
				}
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


