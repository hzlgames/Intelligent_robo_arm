#pragma once

#include <Windows.h>

#include <string>

// ============================
// 视觉协同：数据类型约定
// ============================
//
// 目标：为未来“计算机视觉 + 机械臂运动学”的联动预留稳定接口。
//
// 坐标约定（建议统一采用 OpenCV 常用相机坐标系）：
// - 像素坐标：u 向右为正，v 向下为正
// - 相机坐标系 Cam：X 向右为正，Y 向下为正，Z 向前（光轴方向）为正
//
// 机械臂 Base 坐标系（本项目约定）：
// - X 向右，Y 向前，Z 向上（右手系）

// 相机内参（针孔模型）
struct CameraIntrinsics
{
	bool valid = false;
	double fx = 0.0;
	double fy = 0.0;
	double cx = 0.0;
	double cy = 0.0;
};

// 视觉系统输出（每帧/每次更新）
struct VisualObservation
{
	// 观测时间戳（GetTickCount64）
	ULONGLONG tickMs = 0;

	// 目标像素位置（例如目标中心点/指尖落点/检测框中心）
	bool hasTargetPx = false;
	double u = 0.0;
	double v = 0.0;

	// 目标深度（mm）：可来自深度相机、结构光、双目、或单目尺度估计。
	bool hasDepthMm = false;
	double depthMm = 0.0;

	// 指向射线（单位向量，Cam 坐标系）：例如“手势指向方向”或“目标射线”
	bool hasRay = false;
	double rayX = 0.0;
	double rayY = 0.0;
	double rayZ = 1.0;

	// 置信度（0..1），用于滤除低质量观测（可选）
	bool hasConfidence = false;
	double confidence = 0.0;
};

// 视觉伺服输出：用于直接喂给 JogController 的归一化输入
// 注意：这里刻意保持与 JogController::InputState 一致的 [-1,1] 约定，
// 这样未来可以“无侵入”接入现有 JogController。
struct VisualServoOutput
{
	bool active = false;

	// 归一化输入 [-1, 1]：x/y/z/pitch
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double pitch = 0.0;

	// 用于 HUD/日志：像素误差（u-cx, v-cy）
	double errU = 0.0;
	double errV = 0.0;

	// Debug：本次输出的解释
	std::wstring reason;
};


