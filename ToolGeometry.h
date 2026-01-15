#pragma once

#include <string>

#include "ArmStateEstimator.h"
#include "ToolConfig.h"
#include "VisionGeometry.h"

// ============================
// ToolGeometry
// ============================
// 目标：把“当前姿态 + 工具偏置 + 相机射线”统一到 Base 坐标系下，便于 SeeAndFetch 做平面导航/遮挡后继续运动。
namespace ToolGeometry
{
	// 计算相机光心在 Base 下的位置（mm）
	bool ComputeCameraOriginInBase(const ArmStateEstimator::ArmState& arm,
	                               const ToolConfig& tool,
	                               VisionGeometry::Point3& outCamBase,
	                               std::wstring* outWhy = nullptr);

	// 计算“爪尖中心”在 Base 下的位置（mm）
	bool ComputeGripperPointInBase(const ArmStateEstimator::ArmState& arm,
	                               const ToolConfig& tool,
	                               VisionGeometry::Point3& outGripBase,
	                               std::wstring* outWhy = nullptr);

	// Ray-plane 求交（在 Base 坐标系）：
	// - rayCam：Cam 坐标系射线方向（单位向量更好，但不强制）
	// - planeBase：Base 坐标系平面 n·X + d = 0
	bool ComputeRayPlaneHitInBase(const ArmStateEstimator::ArmState& arm,
	                              const ToolConfig& tool,
	                              const VisionGeometry::Ray& rayCam,
	                              const VisionGeometry::Plane& planeBase,
	                              VisionGeometry::Point3& outHitBase,
	                              double& outT,
	                              std::wstring* outWhy = nullptr);
}









