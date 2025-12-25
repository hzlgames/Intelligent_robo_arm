#pragma once

#include <Windows.h>

#include <mutex>
#include <string>

#include "JogController.h"
#include "VisualServoTypes.h"

// ============================
// VisualServoController
// ============================
//
// 目标：把“画面中的目标/指向变化”转成机械臂可执行的笛卡尔 Jog 输入（XYZ + pitch），
// 用于未来视觉协同抓取的“Look-and-move / 跟随 / 沿指向方向前进后退”。
//
// 设计原则：
// - 仅做“控制量计算”，不绑定具体视觉算法（检测/分割/手势均可）。
// - 输出保持与 JogController::InputState 同形状，便于直接接入现有 Jog 系统。
// - 允许缺失深度：无深度时用“像素误差 → 速度”的经验映射，靠闭环迭代逼近。

enum class VisualServoMode
{
	// 只做“居中”：将目标拉到画面中心（u->cx, v->cy）
	CenterTarget = 0,

	// 沿指向射线前进/后退，同时可选保持居中
	FollowRay = 1,

	// Look-and-move：优先居中，达到阈值后再沿光轴/射线前进（抓取常用）
	LookAndMove = 2,
};

class VisualServoController
{
public:
	struct Params
	{
		// 输出更新：允许观测的最大延迟（ms）。超过则输出 inactive。
		DWORD maxObsAgeMs = 200;

		// 置信度阈值（若观测提供 hasConfidence）
		double minConfidence = 0.2;

		// 像素死区：误差小于该值视为 0
		double deadbandPx = 4.0;

		// 像素误差 → 平移速度（Cam坐标）增益
		// 有深度时：v_cam_x ≈ kPxToVel * errU * depth / fx
		// 无深度时：v_cam_x ≈ kPxToVel * errU
		double kPxToVel = 0.8; // (mm/s)/px 或 (mm/s)/(px * depth/fx)

		// 深度控制：期望距离（mm）与增益
		double desiredDepthMm = 160.0;
		double kDepthToVel = 0.8; // (mm/s)/mm
		double depthDeadbandMm = 10.0;

		// 沿射线前进/后退：最大速度（mm/s）
		double raySpeedMmPerSec = 60.0;

		// 速度饱和：用于归一化输出
		double maxSpeedMmPerSec = 80.0;
		double maxPitchDegPerSec = 40.0;

		// 可选：用垂直像素误差驱动 pitch（很多抓取场景并不需要，默认关闭）
		bool enablePitchFromErrV = false;
		double kErrVToPitch = 0.02; // (normalized pitch)/px

		// 低通滤波（0..1）：越小越平滑，越大越跟手
		double filterAlpha = 0.35;
	};

public:
	void SetEnabled(bool on);
	bool IsEnabled() const { return m_enabled; }

	void SetMode(VisualServoMode m) { m_mode = m; }
	VisualServoMode GetMode() const { return m_mode; }

	void SetParams(const Params& p) { m_params = p; }
	Params GetParams() const { return m_params; }

	void SetCameraIntrinsics(const CameraIntrinsics& k) { m_K = k; }
	CameraIntrinsics GetCameraIntrinsics() const { return m_K; }

	// 视觉线程/主线程都可以调用（内部加锁）
	void UpdateObservation(const VisualObservation& obs);

	// 设置“沿指向方向前进/后退”的用户意图（[-1,1]）
	// 未来可由手势（例如 pinch/scroll）或 UI 控件提供。
	void SetAdvanceCommand(double advanceNorm);

	// 核心：计算一次输出（不会修改 JogController）。
	bool ComputeOutput(VisualServoOutput& out) const;

	// 便捷：把输出直接写入 JogController（会设置 InputState，并同步速度上限）
	// - 若 overrideManual=true：即使用户在按键/拖摇杆，也会被视觉输出覆盖
	// - 若 overrideManual=false：用户手动输入优先，视觉仅在用户“没按住”时生效
	bool ApplyToJog(JogController& jog, bool overrideManual, std::wstring& outWhy) const;

private:
	static double Clamp(double v, double mn, double mx);
	static double Sign0(double v);
	static double Deadband(double v, double db);
	static double Lerp(double a, double b, double t);

	// 默认的 Cam->Base 速度映射（在没有外参时用于先跑通闭环）
	// 假设相机光轴 +Z_cam 指向 base +Y（向前），相机 +X_cam 对齐 base +X，
	// 相机 +Y_cam（向下）对齐 base -Z（向上为负）。
	static void MapCamVelToBase(double vxCam, double vyCam, double vzCam,
	                            double& outVxBase, double& outVyBase, double& outVzBase);

private:
	bool m_enabled = false;
	VisualServoMode m_mode = VisualServoMode::LookAndMove;
	Params m_params{};
	CameraIntrinsics m_K{};

	mutable std::mutex m_mu;
	VisualObservation m_lastObs{};
	double m_advanceNorm = 0.0;

	// 输出滤波状态
	mutable VisualServoOutput m_lastOut{};
	mutable bool m_hasLastOut = false;
};


