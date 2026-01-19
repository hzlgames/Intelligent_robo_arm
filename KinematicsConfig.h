#pragma once

#include <array>
#include <string>

// 运动学配置（可通过注册表Profile + SettingsIo导入导出的INI进行持久化）。
// 目标：把“会变化的标定参数”全部收敛到这里，便于后续反复改标定而不改代码。
//
// 说明：
// - 本项目的舵机控制位置域默认假定为 0..1000（见 ArmProtocol::ServoTarget::position）
// - mechanics.md 提供了两点标定（0°与+θ°），我们用“两点+θ”计算线性映射
// - 由于实际装配存在偏差，额外预留 zeroOffsetDeg（关节零位偏置）用于后续标定/微调
class KinematicsConfig
{
public:
	// 机械臂关节数量（运动学链）：J1..J5（J6 作为兼容保留，不进入运动学）
	static constexpr int kJointCount = 5;

	struct LinkLengthsMm
	{
		// 单位：mm（来自 Reference/mechanics.md，可后续修改）
		double L_base = 80.0;   // 底座高度：地面到 J2 轴心
		double L_arm1 = 100.0;  // 大臂：J2->J3
		double L_arm2 = 95.0;   // 小臂：J3->J4
		double L_wrist = 95.0;  // 腕长：J4->J5
		double L_cam = 55.0;    // 摄像头偏移（先占位，后续视觉外参可更精细）
	};

	struct JointCalib
	{
		// 两点标定：pos@0deg 与 pos@+deg，用于线性映射 angleDeg <-> pos
		// 方向信息完全由这两个值决定：
		// - k = (posAtPlusDeg - posAt0Deg) / plusDeg
		// - 如果 k > 0，说明角度增大时 pos 增大（正向）
		// - 如果 k < 0，说明角度增大时 pos 减小（反向）
		// 这样不需要额外的 invert/sign 开关，方向由标定数据本身隐含。
		int posAt0Deg = 500;
		int posAtPlusDeg = 500;
		int plusDeg = 45; // 例如 45/90

		// 关节零位偏置（度）：用于修正"舵机0°"与"几何模型0°"的差异（装配/安装导致）
		// 默认 0；未来可通过视觉/标定拟合得到。
		double zeroOffsetDeg = 0.0;
	};

	// FPS/手动控制安全参数（用于约束 r/z/pitch 的积分范围）
	struct SafetyParams
	{
		// 俯仰角限制（度）
		// 说明：早期为了避免在 ±90°附近“看起来敏感”，曾把范围收得很窄。
		// 但实际物理使用中，末端确实可能需要“低于地平线”的姿态（> 90°）。
		// 因此默认放宽到 ±170°；若你的机构确实无法达到，请在 ini/诊断界面里再收紧。
		int pitchMinDeg = -170;
		int pitchMaxDeg = 170;

		// 最小高度（mm）
		int zMinMm = 20;

		// 工作半径（mm）
		int rMinMm = 30;
		int rMaxMm = 290;
	};

	KinematicsConfig();

	// 从 Profile（注册表）加载/保存
	void LoadAll();
	void SaveAll() const;

	const LinkLengthsMm& Links() const { return m_links; }
	LinkLengthsMm& Links() { return m_links; }

	const JointCalib& GetJoint(int nJoint) const { return m_joints[nJoint]; } // 1..5
	JointCalib& GetJoint(int nJoint) { return m_joints[nJoint]; }

	const SafetyParams& Safety() const { return m_safety; }
	SafetyParams& Safety() { return m_safety; }

private:
	static std::wstring SectionLinks();
	static std::wstring SectionForJoint(int nJoint);
	static std::wstring SectionSafety();

private:
	LinkLengthsMm m_links{};
	std::array<JointCalib, kJointCount + 1> m_joints{}; // 0 unused
	SafetyParams m_safety{};
};







