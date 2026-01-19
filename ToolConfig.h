#pragma once

#include <string>

// ============================
// ToolConfig（末端工具/相机/夹爪偏置）
// ============================
// 目标：把“相机/夹爪相对末端”的固定偏置集中管理，供视觉抓取/射线几何/状态机复用。
//
// 坐标约定：使用 Cam 坐标系（与 VisualServoTypes 一致）
// - X 右、Y 下、Z 前（光轴）
class ToolConfig
{
public:
	struct Vec3Mm
	{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};

	ToolConfig() = default;

	void LoadAll();
	void SaveAll() const;

	// 从 J5 轴心（运动学模型末端点）到相机光心的偏置（Cam 坐标系）
	// 默认：假设相机安装在“上方”约 55mm（Y 向下为正，因此上方为负）
	const Vec3Mm& Joint5ToCam_Cam() const { return m_joint5ToCam_Cam; }
	Vec3Mm& Joint5ToCam_Cam() { return m_joint5ToCam_Cam; }

	// 从相机光心到“爪尖中心”的偏置（Cam 坐标系）
	// 默认：零位时爪-镜距离约 40mm，先按光轴方向 +Z 偏移（可通过配置修正）
	const Vec3Mm& CamToGripper_Cam() const { return m_camToGripper_Cam; }
	Vec3Mm& CamToGripper_Cam() { return m_camToGripper_Cam; }

private:
	static std::wstring SectionOffsets();

private:
	Vec3Mm m_joint5ToCam_Cam{ 0.0, -55.0, 0.0 };
	Vec3Mm m_camToGripper_Cam{ 0.0, 0.0, 40.0 };
};













