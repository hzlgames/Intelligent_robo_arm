#pragma once

#include <mutex>
#include <string>

#include "ArmKinematics.h"

// KinematicsOverlayService：为“相机画面 HUD 叠加”提供线程安全的状态快照。
// - UI/Jog 线程更新状态（目标Pose、输入、IK状态等）
// - 渲染线程（DrawDevice::DrawOverlays）读取快照并绘制
class KinematicsOverlayService
{
public:
	struct Snapshot
	{
		bool jogActive = false;
		double joyX = 0.0; // [-1,1]
		double joyY = 0.0; // [-1,1]

		bool ikOk = true;
		std::wstring ikWhy;

		ArmKinematics::PoseTarget target{};

		unsigned sendFps = 0;
		unsigned sinceLastSendMs = 0;

		// Visual servo (vision-follow) debug overlay
		bool vsEnabled = false;
		bool vsApplied = false; // whether visual command is currently driving jog (vs overrides manual)
		bool vsActive = false;  // visual output active
		int  vsMode = 0;        // VisualServoMode enum value (kept as int to avoid extra include)
		double vsErrU = 0.0;    // px
		double vsErrV = 0.0;    // px
		double vsAdvance = 0.0; // [-1,1]
		std::wstring vsWhy;
	};

	static KinematicsOverlayService& Instance();

	void UpdateJog(bool jogActive, double joyX, double joyY,
	               const ArmKinematics::PoseTarget& target,
	               bool ikOk, const std::wstring& ikWhy);

	void UpdateSerialStats(unsigned fps, unsigned sinceMs);

	void UpdateVisualServo(bool enabled, bool applied, bool active, int mode,
	                      double errU, double errV, double advance,
	                      const std::wstring& why);

	Snapshot GetSnapshot() const;

private:
	KinematicsOverlayService() = default;

private:
	mutable std::mutex m_mu;
	Snapshot m_s;
};



