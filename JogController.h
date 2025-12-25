#pragma once

#include <Windows.h>
#include <string>

#include "ArmKinematics.h"
#include "KinematicsConfig.h"
#include "MotionController.h"

// JogController：主界面“按住持续移动”的摇杆式控制逻辑（Base 笛卡尔空间）
//
// 设计要点：
// - 按住移动、松开即停（deadman）
// - 固定频率 Tick（例如 20Hz）对目标 Pose 积分
// - 每次 Tick：PoseTarget -> IK -> ServoPos -> 下发
// - “最新指令优先”：避免队列堆积导致的严重延迟（通过清理 Jog 队列实现）
// - 错误可解释：IK 失败/超限时，返回 reason，UI 可展示并停止继续发送
class JogController
{
public:
	struct Params
	{
		// 发送频率（Hz）。实际发送仍会受 ArmCommsService Throttle 影响。
		int sendHz = 20;

		// 最大步长限制（每 tick），防止数值抖动导致跳变过大
		double maxStepMm = 3.0;     // mm/tick
		double maxStepPitchDeg = 2.0; // deg/tick

		// 速度（由 UI 滑条给出）
		double speedMmPerSec = 50.0;
		double pitchDegPerSec = 30.0;
	};

	struct InputState
	{
		// deadman：是否按住（鼠标按下或特定按键）
		bool active = false;

		// 归一化输入 [-1,1]：
		// - x: 右为 +X
		// - y: 前为 +Y
		// - z: 上为 +Z
		// - pitch: 末端俯仰（+为抬头/向上）
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		double pitch = 0.0;
	};

public:
	JogController();

	void SetParams(const Params& p) { m_params = p; }
	Params GetParams() const { return m_params; }

	void SetInputState(const InputState& in) { m_input = in; }
	InputState GetInputState() const { return m_input; }

	// 设置当前目标（通常在启动 Jog 或收到外部定位结果时调用）
	void SetTargetPose(const ArmKinematics::PoseTarget& pose) { m_target = pose; }
	ArmKinematics::PoseTarget GetTargetPose() const { return m_target; }

	// 绑定依赖（由主界面提供单例/成员）
	void Bind(MotionController* pMotion, KinematicsConfig* pKc);

	// 定时调用：负责积分 + 下发
	bool Tick(std::wstring& outWhy);

	// 停止 Jog（不再发送），但不做急停（急停由 UI 单独触发）
	void Stop() { m_input.active = false; }

private:
	bool BuildCurrentJointEstimate(const MotionConfig& mc, const KinematicsConfig& kc, ArmKinematics::JointAnglesRad& outQ);

private:
	Params m_params;
	InputState m_input;
	ArmKinematics::PoseTarget m_target{};

	MotionController* m_pMotion = nullptr;
	KinematicsConfig* m_pKc = nullptr;

	ULONGLONG m_lastTick = 0;
};



