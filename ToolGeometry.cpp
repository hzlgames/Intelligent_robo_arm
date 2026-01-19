#include "pch.h"

#include "ToolGeometry.h"

#include <cmath>

namespace
{
	bool IntersectRayPlaneAt(const VisionGeometry::Point3& origin,
	                         const VisionGeometry::Ray& dir,
	                         const VisionGeometry::Plane& plane,
	                         VisionGeometry::Point3& outP,
	                         double& outT)
	{
		const double denom = plane.nx * dir.x + plane.ny * dir.y + plane.nz * dir.z;
		if (std::fabs(denom) <= 1e-9) return false;
		const double t = -(plane.nx * origin.x + plane.ny * origin.y + plane.nz * origin.z + plane.d) / denom;
		if (t <= 0.0) return false;
		outT = t;
		outP.x = origin.x + dir.x * t;
		outP.y = origin.y + dir.y * t;
		outP.z = origin.z + dir.z * t;
		return true;
	}
}

namespace ToolGeometry
{
	bool ComputeCameraOriginInBase(const ArmStateEstimator::ArmState& arm,
	                               const ToolConfig& tool,
	                               VisionGeometry::Point3& outCamBase,
	                               std::wstring* outWhy)
	{
		if (outWhy) outWhy->clear();
		if (!arm.valid)
		{
			if (outWhy) *outWhy = L"ArmState 无效（读回/标定不足）。";
			return false;
		}

		// J5 轴心（运动学末端）在 Base 下的位置（mm）
		const VisionGeometry::Point3 pJ5{ arm.joint5PoseBase.x_mm, arm.joint5PoseBase.y_mm, arm.joint5PoseBase.z_mm };

		// J5->Cam 偏置：Cam 坐标系向量，旋转到 Base 并相加
		VisionGeometry::Point3 offBase{};
		const auto& offCam = tool.Joint5ToCam_Cam();
		if (!VisionGeometry::MapCamVectorToBase_YawPitch(arm.yawRad, arm.pitchRad,
		                                                 VisionGeometry::Point3{ offCam.x, offCam.y, offCam.z },
		                                                 offBase))
		{
			if (outWhy) *outWhy = L"Cam->Base 向量变换失败。";
			return false;
		}

		outCamBase = VisionGeometry::Point3{ pJ5.x + offBase.x, pJ5.y + offBase.y, pJ5.z + offBase.z };
		return true;
	}

	bool ComputeGripperPointInBase(const ArmStateEstimator::ArmState& arm,
	                               const ToolConfig& tool,
	                               VisionGeometry::Point3& outGripBase,
	                               std::wstring* outWhy)
	{
		if (outWhy) outWhy->clear();

		VisionGeometry::Point3 camBase{};
		std::wstring why;
		if (!ComputeCameraOriginInBase(arm, tool, camBase, &why))
		{
			if (outWhy) *outWhy = why;
			return false;
		}

		VisionGeometry::Point3 offBase{};
		const auto& offCam = tool.CamToGripper_Cam();
		if (!VisionGeometry::MapCamVectorToBase_YawPitch(arm.yawRad, arm.pitchRad,
		                                                 VisionGeometry::Point3{ offCam.x, offCam.y, offCam.z },
		                                                 offBase))
		{
			if (outWhy) *outWhy = L"Cam->Base 向量变换失败。";
			return false;
		}

		outGripBase = VisionGeometry::Point3{ camBase.x + offBase.x, camBase.y + offBase.y, camBase.z + offBase.z };
		return true;
	}

	bool ComputeRayPlaneHitInBase(const ArmStateEstimator::ArmState& arm,
	                              const ToolConfig& tool,
	                              const VisionGeometry::Ray& rayCam,
	                              const VisionGeometry::Plane& planeBase,
	                              VisionGeometry::Point3& outHitBase,
	                              double& outT,
	                              std::wstring* outWhy)
	{
		if (outWhy) outWhy->clear();

		VisionGeometry::Point3 camBase{};
		std::wstring why;
		if (!ComputeCameraOriginInBase(arm, tool, camBase, &why))
		{
			if (outWhy) *outWhy = why;
			return false;
		}

		VisionGeometry::Ray rayBase{};
		if (!VisionGeometry::MapCamRayToBase_YawPitch(arm.yawRad, arm.pitchRad, rayCam, rayBase))
		{
			if (outWhy) *outWhy = L"Ray Cam->Base 变换失败。";
			return false;
		}

		if (!IntersectRayPlaneAt(camBase, rayBase, planeBase, outHitBase, outT))
		{
			if (outWhy) *outWhy = L"Ray-Plane 无交点（平行或在相机后方）。";
			return false;
		}

		return true;
	}
}













