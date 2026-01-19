#include "pch.h"

#include "VisionGeometry.h"

#include <cmath>

namespace VisionGeometry
{
	static inline double Norm3(double x, double y, double z)
	{
		return std::sqrt(x * x + y * y + z * z);
	}

	bool PixelToRay(const CameraIntrinsics& K, double u, double v, Ray& outRay)
	{
		if (!K.valid || K.fx == 0.0 || K.fy == 0.0)
		{
			return false;
		}
		const double x = (u - K.cx) / K.fx;
		const double y = (v - K.cy) / K.fy;
		const double z = 1.0;
		const double n = Norm3(x, y, z);
		if (n <= 1e-9) return false;
		outRay.x = x / n;
		outRay.y = y / n;
		outRay.z = z / n;
		return true;
	}

	bool IntersectRayPlane(const Ray& ray, const Plane& plane, Point3& outP, double& outT)
	{
		const double denom = plane.nx * ray.x + plane.ny * ray.y + plane.nz * ray.z;
		if (std::fabs(denom) <= 1e-9)
		{
			return false; // parallel
		}
		const double t = -plane.d / denom;
		if (t <= 0.0)
		{
			return false; // behind camera
		}
		outT = t;
		outP.x = ray.x * t;
		outP.y = ray.y * t;
		outP.z = ray.z * t;
		return true;
	}

	bool PlaneFromMarkerPose(const double R[9], const double t[3], Plane& outPlane)
	{
		if (!R || !t) return false;

		// marker 平面法向在 marker 坐标中为 (0,0,1)
		// n_cam = R * n_marker = R * (0,0,1) = 第3列
		const double nx = R[2];
		const double ny = R[5];
		const double nz = R[8];

		const double nn = Norm3(nx, ny, nz);
		if (nn <= 1e-9) return false;

		outPlane.nx = nx / nn;
		outPlane.ny = ny / nn;
		outPlane.nz = nz / nn;

		// 平面过点 t（marker原点），所以 d = -n·t
		outPlane.d = -(outPlane.nx * t[0] + outPlane.ny * t[1] + outPlane.nz * t[2]);
		return true;
	}

	bool MapCamVectorToBase_YawPitch(double yawRad, double pitchRad, const Point3& vCam, Point3& outVBase)
	{
		// Step0: cam->base0 (yaw=0,pitch=0) 固定映射
		Point3 v0 = MapCamPointToBase_Default(vCam);

		// Step1: Rx(pitch) about +X_base
		const double cp = std::cos(pitchRad);
		const double sp = std::sin(pitchRad);
		const double v1x = v0.x;
		const double v1y = v0.y * cp - v0.z * sp;
		const double v1z = v0.y * sp + v0.z * cp;

		// Step2: Rz(-yaw) about +Z_base（与视觉观测坐标约定一致）
		const double cy = std::cos(yawRad);
		const double sy = std::sin(yawRad);
		outVBase.x = v1x * cy + v1y * sy;
		outVBase.y = -v1x * sy + v1y * cy;
		outVBase.z = v1z;
		return true;
	}
}


