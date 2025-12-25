#pragma once

#include "VisualServoTypes.h"

// ============================
// VisionGeometry
// ============================
// 目标：提供“像素→射线→平面交点”的通用几何函数，后续抓取/放置/标定都会复用。
namespace VisionGeometry
{
	struct Ray
	{
		// 相机坐标系：X右、Y下、Z前（与 VisualServoTypes.h 一致）
		double x = 0.0;
		double y = 0.0;
		double z = 1.0;
	};

	struct Plane
	{
		// 平面方程：n·X + d = 0（相机坐标系）
		double nx = 0.0;
		double ny = 0.0;
		double nz = 1.0;
		double d = 0.0;
	};

	struct Point3
	{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};

	// 像素(u,v) -> 相机射线（归一化）
	bool PixelToRay(const CameraIntrinsics& K, double u, double v, Ray& outRay);

	// 相机原点(0,0,0)发出射线，与平面求交。返回交点（t>0）。
	bool IntersectRayPlane(const Ray& ray, const Plane& plane, Point3& outP, double& outT);

	// 从 ArUco/marker 位姿估计平面（marker 坐标系 Z=0 的平面）在相机坐标系下的 Plane。
	// 输入：旋转矩阵 R（marker->cam）与平移向量 t（marker 原点在 cam 中坐标）。
	bool PlaneFromMarkerPose(const double R[9], const double t[3], Plane& outPlane);

	// 默认 Cam->Base 点映射（用于无外参时先跑通链路；与 VisualServoController 的速度映射一致）
	// Base: X右, Y前, Z上
	inline Point3 MapCamPointToBase_Default(const Point3& pCam)
	{
		return Point3{ pCam.x, pCam.z, -pCam.y };
	}
}


