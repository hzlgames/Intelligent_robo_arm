#include "pch.h"

#include "JogController.h"

#include "ArmCommsService.h"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <vector>

namespace
{
	double Clamp(double v, double mn, double mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}

	double ClampStep(double v, double maxAbs)
	{
		if (v > maxAbs) return maxAbs;
		if (v < -maxAbs) return -maxAbs;
		return v;
	}

	constexpr double kPi = 3.14159265358979323846;
	double DegToRad(double d) { return d * (kPi / 180.0); }
	double RadToDeg(double r) { return r * (180.0 / kPi); }

	double Sign(double v)
	{
		if (v > 0.0) return 1.0;
		if (v < 0.0) return -1.0;
		return 0.0;
	}

	// 将 UTF-16 文本写入 UTF-8 文件（带 BOM），便于你直接用记事本打开。
	bool WriteUtf8File(const std::wstring& path, const std::wstring& text, std::wstring& outWhy)
	{
		outWhy.clear();
		HANDLE h = ::CreateFileW(path.c_str(),
			GENERIC_WRITE,
			FILE_SHARE_READ,
			nullptr,
			CREATE_ALWAYS,
			FILE_ATTRIBUTE_NORMAL,
			nullptr);
		if (h == INVALID_HANDLE_VALUE)
		{
			outWhy = L"CreateFileW失败。";
			return false;
		}

		// UTF-8 BOM
		const uint8_t bom[3] = { 0xEF, 0xBB, 0xBF };
		DWORD w = 0;
		(void)::WriteFile(h, bom, 3, &w, nullptr);

		if (text.empty())
		{
			::CloseHandle(h);
			return true;
		}

		int cb = ::WideCharToMultiByte(CP_UTF8, 0, text.c_str(), (int)text.size(), nullptr, 0, nullptr, nullptr);
		if (cb <= 0)
		{
			::CloseHandle(h);
			outWhy = L"WideCharToMultiByte失败。";
			return false;
		}
		std::vector<char> buf((size_t)cb);
		(void)::WideCharToMultiByte(CP_UTF8, 0, text.c_str(), (int)text.size(), buf.data(), cb, nullptr, nullptr);
		(void)::WriteFile(h, buf.data(), (DWORD)buf.size(), &w, nullptr);
		::CloseHandle(h);
		return true;
	}

	std::wstring TraceEventToText(JogController::TraceEvent ev)
	{
		switch (ev)
		{
		case JogController::TraceEvent::SkipNotDue: return L"SkipNotDue";
		case JogController::TraceEvent::SkipInactive: return L"SkipInactive";
		case JogController::TraceEvent::WaitReadback: return L"WaitReadback";
		case JogController::TraceEvent::NudgeJointSent: return L"NudgeJointSent";
		case JogController::TraceEvent::StepModeSent: return L"StepModeSent";
		case JogController::TraceEvent::IkFailed: return L"IkFailed";
		case JogController::TraceEvent::LimitRejected: return L"LimitRejected";
		case JogController::TraceEvent::DeadbandSuppressed: return L"DeadbandSuppressed";
		case JogController::TraceEvent::SendFailed: return L"SendFailed";
		case JogController::TraceEvent::SendOk: return L"SendOk";
		default: return L"None";
		}
	}
}

JogController::JogController()
{
	m_lastTick = ::GetTickCount64();
	for (int j = 0; j <= ArmKinematics::kJointCount; j++)
	{
		m_lastSentJointPos[(size_t)j] = -999999;
	}
	ClearTrace();
}

void JogController::SetInputState(const InputState& in)
{
	m_cartInput = in;
	m_inputKind = InputKind::Cartesian;
}

void JogController::SetFpsInputState(const FpsInputState& in)
{
	m_fpsInput = in;
	m_inputKind = InputKind::Fps;
}

void JogController::Stop()
{
	m_cartInput.active = false;
	m_fpsInput.active = false;
}

void JogController::EnableTrace(bool bEnable)
{
	m_traceEnabled = bEnable;
}

void JogController::ClearTrace()
{
	m_traceNext = 0;
	m_traceCount = 0;
	for (size_t i = 0; i < kTraceCap; i++)
	{
		m_trace[i] = TraceEntry{};
	}
}

void JogController::TracePush(const TraceEntry& e)
{
	if (!m_traceEnabled) return;
	m_trace[m_traceNext] = e;
	m_traceNext = (m_traceNext + 1) % kTraceCap;
	if (m_traceCount < kTraceCap) m_traceCount++;
}

JogController::TracePose JogController::ToTracePose(const ArmKinematics::PoseTarget& p)
{
	TracePose t{};
	t.x_mm = p.x_mm;
	t.y_mm = p.y_mm;
	t.z_mm = p.z_mm;
	t.pitch_deg = p.pitch_deg;
	return t;
}

std::wstring JogController::DumpTraceText(int maxLines) const
{
	const int lines = (maxLines <= 0) ? 0 : maxLines;
	std::wstring out;
	out.reserve((size_t)lines * 160);

	out += L"=== JogTrace ===\r\n";
	out += L"tickMs,ev,kind,active,step,usedRb,prev(x,y,z,p),next(x,y,z,p),ikOk,withinLimits,chosen,maxDelta,timeMs,why\r\n";

	const size_t n = m_traceCount;
	if (n == 0) return out;

	// 从最旧到最新输出
	size_t start = (m_traceNext + kTraceCap - n) % kTraceCap;
	int printed = 0;
	for (size_t i = 0; i < n; i++)
	{
		if (printed >= lines) break;
		const TraceEntry& e = m_trace[(start + i) % kTraceCap];

		wchar_t buf[512] = { 0 };
		(void)swprintf_s(buf, _countof(buf),
			L"%llu,%s,%u,%d,%d,%d,(%.1f,%.1f,%.1f,%.1f),(%.1f,%.1f,%.1f,%.1f),%d,%d,%d,%d,%d,%s\r\n",
			(unsigned long long)e.tickMs,
			TraceEventToText(e.ev).c_str(),
			(unsigned)e.inputKind,
			e.active ? 1 : 0,
			e.stepMode ? 1 : 0,
			e.usedReadbackCount,
			e.prevTarget.x_mm, e.prevTarget.y_mm, e.prevTarget.z_mm, e.prevTarget.pitch_deg,
			e.nextTarget.x_mm, e.nextTarget.y_mm, e.nextTarget.z_mm, e.nextTarget.pitch_deg,
			e.ikOk ? 1 : 0,
			e.withinLimits ? 1 : 0,
			e.chosenIndex,
			e.maxDeltaPos,
			e.sendTimeMs,
			e.why);
		out += buf;
		printed++;
	}

	// 额外输出：最后一次估计的 pos 来源（便于你看“是否一直在用homePos/是否读回缺失”）
	out += L"\r\n--- lastEstimateUsedPos ---\r\n";
	{
		// 输出最新一条记录携带的 usedPos/usedFromReadback（如果有）
		const TraceEntry& e = m_trace[(m_traceNext + kTraceCap - 1) % kTraceCap];
		for (int j = 1; j <= ArmKinematics::kJointCount; j++)
		{
			wchar_t b2[128] = { 0 };
			(void)swprintf_s(b2, _countof(b2),
				L"J%d: pos=%d source=%s\r\n",
				j,
				e.usedPos[(size_t)j],
				(e.usedFromReadback[(size_t)j] ? L"readback" : L"home/fallback"));
			out += b2;
		}
	}
	return out;
}

bool JogController::SaveTraceToFile(const std::wstring& path, std::wstring& outWhy) const
{
	const std::wstring text = DumpTraceText(240);
	return WriteUtf8File(path, text, outWhy);
}

bool JogController::SaveTraceToDefaultFile(std::wstring& outPath, std::wstring& outWhy) const
{
	outWhy.clear();
	wchar_t modulePath[MAX_PATH] = { 0 };
	const DWORD n = ::GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
	{
		outWhy = L"GetModuleFileNameW失败。";
		return false;
	}

	std::wstring dir(modulePath);
	const size_t pos = dir.find_last_of(L"\\/");
	if (pos != std::wstring::npos) dir = dir.substr(0, pos);
	outPath = dir + L"\\jog_trace_last.txt";
	return SaveTraceToFile(outPath, outWhy);
}

void JogController::Bind(MotionController* pMotion, KinematicsConfig* pKc)
{
	m_pMotion = pMotion;
	m_pKc = pKc;
}

void JogController::SetTargetPose(const ArmKinematics::PoseTarget& pose)
{
	m_target = pose;
	SyncFpsStateFromTarget();
}

bool JogController::BuildCurrentJointEstimate(const MotionConfig& mc,
                                              const KinematicsConfig& kc,
                                              ArmKinematics::JointAnglesRad& outQ,
                                              std::array<int, ArmKinematics::kJointCount + 1>* pUsedPos,
                                              std::array<uint8_t, ArmKinematics::kJointCount + 1>* pUsedFromReadback)
{
	// 说明：为了让 Jog 的“择优解”更稳定，我们尽量从回读缓存估算当前关节角。
	// 若没有回读缓存：
	// - 未连接：退化为 homePos（合理）
	// - 已连接：优先使用“最近一次下发的舵机目标(m_lastSentJointPos)”做保守估计，避免突然跳回 homePos 导致姿态估计猛变。
	for (int j = 0; j <= ArmKinematics::kJointCount; j++)
	{
		outQ.q[j] = 0.0;
	}
	if (pUsedPos)
	{
		for (int j = 0; j <= ArmKinematics::kJointCount; j++) (*pUsedPos)[(size_t)j] = 0;
	}
	if (pUsedFromReadback)
	{
		for (int j = 0; j <= ArmKinematics::kJointCount; j++) (*pUsedFromReadback)[(size_t)j] = 0;
	}

	int usedRb = 0;
	const bool connected = ArmCommsService::Instance().IsConnected();
	const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		const auto& jc = mc.Get(j);
		int pos = jc.homePos;
		bool fromRb = false;

		// 1) 优先：新鲜读回
		if (jc.servoId >= 1 && jc.servoId <= 6)
		{
			uint16_t rb = 0;
			DWORD age = 0;
			if (ArmCommsService::Instance().GetLastReadPosEx((uint8_t)jc.servoId, rb, age) && age <= staleMs)
			{
				pos = (int)rb;
				usedRb++;
				fromRb = true;
			}
		}

		// 2) 连接状态下：读回不可用/过期时，优先使用最近一次下发的目标位置（比 homePos 更稳定）
		if (connected && !fromRb && m_hasLastSentJointPos)
		{
			const int lastSent = m_lastSentJointPos[(size_t)j];
			if (lastSent >= 0 && lastSent <= 1000)
			{
				pos = lastSent;
			}
		}

		if (pUsedPos) (*pUsedPos)[(size_t)j] = pos;
		if (pUsedFromReadback) (*pUsedFromReadback)[(size_t)j] = (fromRb ? 1 : 0);

		double rad = 0.0;
		if (!ArmKinematics::ServoPosToJointRad(kc, &mc, j, pos, rad))
		{
			// 标定不足时给 0，后续 IK 仍可跑，但会更不稳定
			rad = 0.0;
		}
		outQ.q[j] = rad;
	}

	// 保存：供 Tick 决策（回读不足时不允许用 homePos 规划）
	m_lastUsedReadbackCount = usedRb;

	return true;
}

void JogController::SyncFpsStateFromTarget()
{
	// [重构] 将 m_target 映射到 (r_wc, z_wc, phi, baseYaw)。
	// 使用腕关节位置作为控制目标，这样 R/F (pitch) 只会影响 J4，符合用户直觉。
	const double x = m_target.x_mm;
	const double y = m_target.y_mm;
	const double r_ee = std::sqrt(x * x + y * y); // 末端水平距离
	const double z_ee = m_target.z_mm;            // 末端高度
	const double phiRad = DegToRad(m_target.pitch_deg);

	// 从末端位置反算腕关节位置
	const double L_wrist = m_pKc ? m_pKc->Links().L_wrist : 95.0;
	m_fps_r_mm = r_ee - L_wrist * std::cos(phiRad);   // 腕关节水平距离
	m_fps_z_mm = z_ee - L_wrist * std::sin(phiRad);   // 腕关节高度
	m_fps_phi_deg = m_target.pitch_deg;

	// r 很小会导致 yaw 不稳定：此时保持原 yaw（允许继续旋转脱离奇点）
	if (r_ee > 1e-6)
	{
		m_fps_baseYaw_rad = std::atan2(x, y); // 与 ArmKinematics::q1 定义一致
	}
	m_fpsHasState = true;
}

ArmKinematics::PoseTarget JogController::StepTargetByCartesian(const InputState& in, double dt) const
{
	// 将输入转成速度（mm/s, deg/s）
	const double vx = Clamp(in.x, -1.0, 1.0) * m_params.speedMmPerSec;
	const double vy = Clamp(in.y, -1.0, 1.0) * m_params.speedMmPerSec;
	const double vz = Clamp(in.z, -1.0, 1.0) * m_params.speedMmPerSec;
	const double vp = Clamp(in.pitch, -1.0, 1.0) * m_params.pitchDegPerSec;

	double dx = vx * dt;
	double dy = vy * dt;
	double dz = vz * dt;
	double dp = vp * dt;

	// 步长限制（每tick）
	dx = ClampStep(dx, m_params.maxStepMm);
	dy = ClampStep(dy, m_params.maxStepMm);
	dz = ClampStep(dz, m_params.maxStepMm);
	dp = ClampStep(dp, m_params.maxStepPitchDeg);

	ArmKinematics::PoseTarget nextTarget = m_target;
	nextTarget.x_mm += dx;
	nextTarget.y_mm += dy;
	nextTarget.z_mm += dz;
	nextTarget.pitch_deg += dp;
	return nextTarget;
}

ArmKinematics::PoseTarget JogController::StepTargetByFps(const FpsInputState& in, double dt, std::wstring& outWhy)
{
	outWhy.clear();
	if (!m_pKc)
	{
		outWhy = L"KinematicsConfig 未绑定。";
		return m_target;
	}

	if (!m_fpsHasState)
	{
		SyncFpsStateFromTarget();
	}

	// 安全参数（可通过 ini 导入导出覆盖）
	const auto& safe = m_pKc->Safety();
	const double pitchMin = (double)safe.pitchMinDeg;
	const double pitchMax = (double)safe.pitchMaxDeg;
	const double zMin = (double)safe.zMinMm;
	const double rMin = (double)safe.rMinMm;
	const double rMax = (double)safe.rMaxMm;
	const double L_wrist = m_pKc->Links().L_wrist;
	const double L_base = m_pKc->Links().L_base;
	const double L1 = m_pKc->Links().L_arm1;
	const double L2 = m_pKc->Links().L_arm2;

	// [重构] 简化控制模型：
	// - W/S (fwd)：沿“视线方向”移动腕关节点（会随当前俯仰角旋转）
	// - Q/E (vert)：垂直于“视线方向”移动腕关节点（会随当前俯仰角旋转）
	// - A/D (yaw)：控制底座旋转（注意方向：D=右转=yaw 减小，A=左转=yaw 增加）
	// - R/F (pitch)：只控制末端俯仰角（只有 J4 会动）
	const double fwdVel = Clamp(in.fwd, -1.0, 1.0) * m_params.speedMmPerSec;     // mm/s
	const double vertVel = Clamp(in.vert, -1.0, 1.0) * m_params.speedMmPerSec;   // mm/s
	// [FIX] A/D 方向相反：取反 yaw 输入
	const double yawVelDeg = Clamp(-in.yaw, -1.0, 1.0) * m_params.yawDegPerSec;  // deg/s
	// [FIX] 你反馈 R/F 方向反了：这里把 pitch 输入取反即可
	const double pitchVelDeg = Clamp(-in.pitch, -1.0, 1.0) * m_params.pitchDegPerSec; // deg/s

	// 以“腕关节点”为控制目标，但位移使用“随俯仰旋转的局部坐标系”：
	// 这样 W/S 是沿当前视线推进，Q/E 是垂直于视线的上下移动（更符合文档与直觉）。
	const double phiRad = DegToRad(m_fps_phi_deg);
	double dr = (fwdVel * std::cos(phiRad) - vertVel * std::sin(phiRad)) * dt; // 腕关节水平位移
	double dz = (fwdVel * std::sin(phiRad) + vertVel * std::cos(phiRad)) * dt; // 腕关节垂直位移
	double dYawDeg = yawVelDeg * dt;
	double dPhiDeg = pitchVelDeg * dt;

	// 步长限制（每tick）
	dr = ClampStep(dr, m_params.maxStepMm);
	dz = ClampStep(dz, m_params.maxStepMm);
	dYawDeg = ClampStep(dYawDeg, m_params.maxStepYawDeg);
	dPhiDeg = ClampStep(dPhiDeg, m_params.maxStepPitchDeg);

	// 更新腕关节位置状态
	m_fps_r_mm += dr;     // 腕关节水平距离
	m_fps_z_mm += dz;     // 腕关节高度
	m_fps_baseYaw_rad += DegToRad(dYawDeg);
	m_fps_phi_deg += dPhiDeg;

	// 安全限制
	m_fps_phi_deg = Clamp(m_fps_phi_deg, pitchMin, pitchMax);
	m_fps_z_mm = std::max(m_fps_z_mm, zMin);
	m_fps_r_mm = Clamp(m_fps_r_mm, rMin, rMax);

	// [关键修复] 腕关节点可达域约束（避免 IK 失败导致状态漂移/猛烈跳变）
	// IK 内部的两连杆约束为：d ∈ [|L1-L2|, L1+L2]
	// 其中 d = sqrt(r_wc^2 + (z_wc - L_base)^2)（肩关节为原点）
	{
		const double dzp = (m_fps_z_mm - L_base);
		const double d = std::sqrt(m_fps_r_mm * m_fps_r_mm + dzp * dzp);
		const double dMin = std::fabs(L1 - L2);
		const double dMax = (L1 + L2);

		if (d > dMax + 1e-6)
		{
			const double s = (d > 1e-9) ? (dMax / d) : 0.0;
			m_fps_r_mm = m_fps_r_mm * s;
			m_fps_z_mm = L_base + dzp * s;
		}
		else if (d < dMin - 1e-6)
		{
			// d 太小接近折叠奇点：将其推到 dMin 圆上（优先保持当前方向）
			if (d > 1e-9)
			{
				const double s = dMin / d;
				m_fps_r_mm = m_fps_r_mm * s;
				m_fps_z_mm = L_base + dzp * s;
			}
			else
			{
				m_fps_r_mm = dMin;
				m_fps_z_mm = L_base;
			}
		}
		// 再做一次 zMin（避免被投影拉到地面以下）
		m_fps_z_mm = std::max(m_fps_z_mm, zMin);
	}

	// 从腕关节位置 + 俯仰角计算末端位置
	const double phiRad2 = DegToRad(m_fps_phi_deg);
	const double r_ee = m_fps_r_mm + L_wrist * std::cos(phiRad2);  // 末端水平距离
	const double z_ee = m_fps_z_mm + L_wrist * std::sin(phiRad2);  // 末端高度

	ArmKinematics::PoseTarget nextTarget{};
	nextTarget.x_mm = r_ee * std::sin(m_fps_baseYaw_rad);
	nextTarget.y_mm = r_ee * std::cos(m_fps_baseYaw_rad);
	nextTarget.z_mm = z_ee;
	nextTarget.pitch_deg = m_fps_phi_deg;
	return nextTarget;
}

bool JogController::Tick(std::wstring& outWhy)
{
	outWhy.clear();
	if (!m_pMotion || !m_pKc)
	{
		outWhy = L"JogController 未绑定依赖（MotionController/KinematicsConfig）。";
		return false;
	}

	const ULONGLONG now = ::GetTickCount64();
	const int hz = (m_params.sendHz <= 0) ? 20 : m_params.sendHz;
	const ULONGLONG periodMs = (ULONGLONG)(1000 / hz);
	if (now - m_lastTick < periodMs)
	{
		// 这类日志会非常密集，默认不记录（除非你后续明确要求）
		return true; // 未到周期
	}
	m_lastTick = now;

	// deadman 未按住则不发送
	const bool active = (m_inputKind == InputKind::Fps) ? m_fpsInput.active : m_cartInput.active;
	if (!active)
	{
		TraceEntry te{};
		te.tickMs = now;
		te.ev = TraceEvent::SkipInactive;
		te.inputKind = (m_inputKind == InputKind::Fps) ? 1 : 0;
		te.active = false;
		te.stepMode = (m_inputKind == InputKind::Fps) ? m_params.stepMode : false;
		te.prevTarget = ToTracePose(m_target);
		te.nextTarget = te.prevTarget;
		TracePush(te);
		return true;
	}

	auto FillInputToTrace = [&](TraceEntry& te)
	{
		te.inputKind = (m_inputKind == InputKind::Fps) ? 1 : 0;
		te.active = true;
		te.stepMode = (m_inputKind == InputKind::Fps) ? m_params.stepMode : false;
		if (m_inputKind == InputKind::Fps)
		{
			te.fwd = m_fpsInput.fwd;
			te.vert = m_fpsInput.vert;
			te.yaw = m_fpsInput.yaw;
			te.pitch = m_fpsInput.pitch;
		}
		else
		{
			te.x = m_cartInput.x;
			te.y = m_cartInput.y;
			te.z = m_cartInput.z;
			te.cartPitch = m_cartInput.pitch;
		}
	};

	// FPS：优先处理“单关节连续微调”（Yaw/Pitch）。要求：只动 J1 或 J4，其他关节不动。
	// 说明：你反馈“俯仰/底座太慢且会带动其它关节”，因此这里绕开 IK，直接对单关节角做小增量并下发。
	if (m_inputKind == InputKind::Fps)
	{
		const bool hasYawPitch = (std::fabs(m_fpsInput.yaw) + std::fabs(m_fpsInput.pitch)) > 0.0;
		const bool hasTranslate = (std::fabs(m_fpsInput.fwd) + std::fabs(m_fpsInput.vert)) > 0.0;

		// 若平移步进正在执行中：为避免打断多关节动作，期间忽略其它输入（急停仍由 UI 处理）
		if (m_stepLockedUntil != 0 && now < m_stepLockedUntil) return true;

		// 单关节优先：只要有 yaw/pitch，就先做单关节连续微调（不与平移叠加）
		if (hasYawPitch && !hasTranslate)
		{
			ArmKinematics::JointAnglesRad qCur;
			std::array<int, ArmKinematics::kJointCount + 1> usedPos{};
			std::array<uint8_t, ArmKinematics::kJointCount + 1> usedFromRb{};
			BuildCurrentJointEstimate(m_pMotion->Config(), *m_pKc, qCur, &usedPos, &usedFromRb);

			// 回读不足时不动作（避免用 homePos 推断当前姿态导致“跳回复位位”）
			const int kMinUsedReadback = 3;
			if (ArmCommsService::Instance().IsConnected() && m_lastUsedReadbackCount < kMinUsedReadback)
			{
				outWhy = L"等待舵机回读建立（避免首次输入触发回零跳变）。";
				TraceEntry te{};
				te.tickMs = now;
				FillInputToTrace(te);
				te.ev = TraceEvent::WaitReadback;
				te.prevTarget = ToTracePose(m_target);
				te.nextTarget = te.prevTarget;
				te.usedReadbackCount = m_lastUsedReadbackCount;
				te.usedPos = usedPos;
				te.usedFromReadback = usedFromRb;
				(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
				TracePush(te);
				return true;
			}

			// dt：按固定 tick 频率计算；这里用“连续速度”描述每秒变化量
			const double dt = (double)periodMs / 1000.0;

			// [需求] 俯仰/底座速度至少 2 倍：这里直接乘以 2.0（不改 UI 滑条逻辑）
			const double kFast = 2.0;

			// 方向保持与连续 FPS 模式一致：对输入取反（已由你验证 A/D、R/F 方向正确）
			const double yawVelDeg = Clamp(-m_fpsInput.yaw, -1.0, 1.0) * m_params.yawDegPerSec * kFast;
			const double pitchVelDeg = Clamp(-m_fpsInput.pitch, -1.0, 1.0) * m_params.pitchDegPerSec * kFast;

			int nJoint = 0;
			double dDeg = 0.0;
			if (std::fabs(m_fpsInput.yaw) >= std::fabs(m_fpsInput.pitch))
			{
				nJoint = 1; // J1：底座
				dDeg = yawVelDeg * dt;
			}
			else
			{
				nJoint = 4; // J4：腕俯仰
				dDeg = pitchVelDeg * dt;
			}

			// 初始化/切换关节：用“当前读回角”作为微调起点
			if (!m_nudgeActive || m_nudgeJoint != nJoint)
			{
				m_nudgeActive = true;
				m_nudgeJoint = nJoint;
				m_nudgeQRad = qCur.q[nJoint];
			}

			if (nJoint != 0 && std::fabs(dDeg) > 1e-6)
			{
				// 本地积分：不依赖“每 tick 的读回变化”，保证按住键能连续动
				m_nudgeQRad += DegToRad(dDeg);
				auto qNew = qCur;
				qNew.q[nJoint] = m_nudgeQRad;

				int posNew = -1;
				if (!ArmKinematics::JointRadToServoPos(*m_pKc, &m_pMotion->Config(), nJoint, qNew.q[nJoint], posNew))
				{
					outWhy = L"单关节角度转舵机位置失败。";
					TraceEntry te{};
					te.tickMs = now;
					FillInputToTrace(te);
					te.ev = TraceEvent::SendFailed;
					te.prevTarget = ToTracePose(m_target);
					te.nextTarget = te.prevTarget;
					te.usedReadbackCount = m_lastUsedReadbackCount;
					te.usedPos = usedPos;
					te.usedFromReadback = usedFromRb;
					(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
					TracePush(te);
					return false;
				}

				std::vector<std::pair<int, int>> jointToPos;
				jointToPos.push_back({ nJoint, posNew });

				ArmCommsService::Instance().ClearMoveQueue();
				// 执行时长尽量贴近 tick 周期，保证“连续跟手”；过长会导致反复清队列而看起来几乎不动
				const int timeMs = std::max(40, (int)periodMs);
				m_pMotion->MoveJointsAbs(jointToPos, timeMs);

				// 更新目标用于 UI 显示/后续平移起点
				m_target = ArmKinematics::ForwardKinematics(*m_pKc, qNew);
				m_lastGoodTarget = m_target;
				m_hasLastGoodTarget = true;
				SyncFpsStateFromTarget();

				TraceEntry te{};
				te.tickMs = now;
				FillInputToTrace(te);
				te.ev = TraceEvent::NudgeJointSent;
				te.prevTarget = ToTracePose(m_target);
				te.nextTarget = te.prevTarget;
				te.usedReadbackCount = m_lastUsedReadbackCount;
				te.usedPos = usedPos;
				te.usedFromReadback = usedFromRb;
				te.sendTimeMs = timeMs;
				TracePush(te);
				return true;
			}

			return true;
		}
		else
		{
			// 松开微调键或进入平移：退出微调积分状态
			m_nudgeActive = false;
			m_nudgeJoint = 0;
			m_nudgeQRad = 0.0;
		}
	}

	// =========================
	// 离散步进模式：每次给出可观步长，并用 500~800ms 平滑执行；执行期间锁输入（只允许急停）
	// =========================
	if (m_inputKind == InputKind::Fps && m_params.stepMode)
	{
		// 以读回 FK 作为每步起点，避免积分漂移
		ArmKinematics::JointAnglesRad qCur;
		std::array<int, ArmKinematics::kJointCount + 1> usedPos{};
		std::array<uint8_t, ArmKinematics::kJointCount + 1> usedFromRb{};
		BuildCurrentJointEstimate(m_pMotion->Config(), *m_pKc, qCur, &usedPos, &usedFromRb);

		// 关键保护：连接后回读尚未建立时，BuildCurrentJointEstimate 会退化为 homePos。
		// 这会导致“第一次按键/摇杆输入就把机械臂拉回竖直复位位”。这里直接等待回读稳定再允许步进。
		// 你明确要求“初始状态允许不是复位状态”，因此宁可先不动。
		{
			const int kMinUsedReadback = 3; // 至少 3 路回读认为“姿态可信”
			if (ArmCommsService::Instance().IsConnected() && m_lastUsedReadbackCount < kMinUsedReadback)
			{
				outWhy = L"等待舵机回读建立（避免首次输入触发回零跳变）。";
				TraceEntry te{};
				te.tickMs = now;
				FillInputToTrace(te);
				te.ev = TraceEvent::WaitReadback;
				te.prevTarget = ToTracePose(m_target);
				te.nextTarget = te.prevTarget;
				te.usedReadbackCount = m_lastUsedReadbackCount;
				te.usedPos = usedPos;
				te.usedFromReadback = usedFromRb;
				(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
				TracePush(te);
				return true;
			}
		}

		const auto poseCur = ArmKinematics::ForwardKinematics(*m_pKc, qCur);

		const double L_base = m_pKc->Links().L_base;
		const double L1 = m_pKc->Links().L_arm1;
		const double L2 = m_pKc->Links().L_arm2;
		const double L_wrist = m_pKc->Links().L_wrist;

		// 基础量：r、yaw、phi 及腕关节中心（用于“俯仰只动腕”）
		const double r_ee = std::sqrt(poseCur.x_mm * poseCur.x_mm + poseCur.y_mm * poseCur.y_mm);
		double yawRad = std::atan2(poseCur.x_mm, poseCur.y_mm);
		double phiDeg = poseCur.pitch_deg;
		{
			// 保护：phi 接近 ±90° 会导致数值敏感
			phiDeg = Clamp(phiDeg, -89.0, 89.0);
		}
		const double phiRad = DegToRad(phiDeg);
		double r_wc = r_ee - L_wrist * std::cos(phiRad);
		double z_wc = poseCur.z_mm - L_wrist * std::sin(phiRad);

		// 选择本步动作：仅处理平移（W/S/Q/E）。
		// yaw/pitch 已在上方的“单关节连续微调”分支处理。
		const double fwdS = Sign(m_fpsInput.fwd);
		const double vertS = Sign(m_fpsInput.vert);

		enum class StepKind { None = 0, Fwd = 3, Vert = 4 };
		StepKind kind = StepKind::None;
		if (fwdS != 0.0) kind = StepKind::Fwd;
		else if (vertS != 0.0) kind = StepKind::Vert;
		else return true;

		// [需求] 平移动作更短以利于微调，但“速度不变”
		// 速度由 speedMmPerSec 控制，步长 = speed * time
		const int translateMs = std::max(120, m_params.stepTranslateMs);
		const double stepMm = (m_params.speedMmPerSec > 0.0)
			? (m_params.speedMmPerSec * ((double)translateMs / 1000.0))
			: m_params.stepMm;

		// [按你的最新需求] FPS 平移使用“末端朝向坐标系”（在 r-z 平面内）：
		// - W/S：沿末端指向前后（视角前后） -> (dr, dz) = step * (cos(phi), sin(phi))
		// - Q/E：垂直于末端指向移动（在 r-z 平面内） -> (dr, dz) = step * (-sin(phi), cos(phi))
		{
			const double pr = DegToRad(phiDeg);
			const double c = std::cos(pr);
			const double s = std::sin(pr);
			double dr = 0.0;
			double dz = 0.0;
			if (kind == StepKind::Fwd)
			{
				dr = (fwdS * stepMm) * c;
				dz = (fwdS * stepMm) * s;
			}
			else if (kind == StepKind::Vert)
			{
				dr = (vertS * stepMm) * (-s);
				dz = (vertS * stepMm) * (c);
			}
			r_wc += dr;
			z_wc += dz;
		}

		// 腕关节中心可达域投影（防止 IK 失败导致跳变）
		{
			const double dzp = (z_wc - L_base);
			const double d = std::sqrt(r_wc * r_wc + dzp * dzp);
			const double dMin = std::fabs(L1 - L2);
			const double dMax = (L1 + L2);
			if (d > dMax + 1e-6)
			{
				const double s = (d > 1e-9) ? (dMax / d) : 0.0;
				r_wc = r_wc * s;
				z_wc = L_base + dzp * s;
			}
			else if (d < dMin - 1e-6)
			{
				if (d > 1e-9)
				{
					const double s = dMin / d;
					r_wc = r_wc * s;
					z_wc = L_base + dzp * s;
				}
				else
				{
					r_wc = dMin;
					z_wc = L_base;
				}
			}
		}

		// 由腕关节中心 + phi 反推末端目标
		const double phiRad2 = DegToRad(phiDeg);
		const double r_ee2 = r_wc + L_wrist * std::cos(phiRad2);
		const double z_ee2 = z_wc + L_wrist * std::sin(phiRad2);

		ArmKinematics::PoseTarget next{};
		next.x_mm = r_ee2 * std::sin(yawRad);
		next.y_mm = r_ee2 * std::cos(yawRad);
		next.z_mm = z_ee2;
		next.pitch_deg = phiDeg;

		const auto ik = ArmKinematics::InverseKinematics(*m_pKc, &m_pMotion->Config(), next, &qCur);

		if (!ik.ok)
		{
			outWhy = ik.reason.empty() ? L"IK 失败（step mode）。" : ik.reason;
			TraceEntry te{};
			te.tickMs = now;
			FillInputToTrace(te);
			te.ev = TraceEvent::IkFailed;
			te.prevTarget = ToTracePose(m_target);
			te.nextTarget = ToTracePose(next);
			te.usedReadbackCount = m_lastUsedReadbackCount;
			te.usedPos = usedPos;
			te.usedFromReadback = usedFromRb;
			te.ikOk = false;
			(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
			TracePush(te);
			return false;
		}

		ArmKinematics::ServoPos sp;
		std::wstring why;
		if (!ArmKinematics::JointAnglesToServoPos(*m_pKc, &m_pMotion->Config(), ik.chosenQ, sp, why))
		{
			outWhy = why.empty() ? L"角度转舵机位置失败（step mode）。" : why;
			TraceEntry te{};
			te.tickMs = now;
			FillInputToTrace(te);
			te.ev = TraceEvent::SendFailed;
			te.prevTarget = ToTracePose(m_target);
			te.nextTarget = ToTracePose(next);
			te.usedReadbackCount = m_lastUsedReadbackCount;
			te.usedPos = usedPos;
			te.usedFromReadback = usedFromRb;
			te.ikOk = true;
			(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
			TracePush(te);
			return false;
		}

		std::vector<std::pair<int, int>> jointToPos;
		jointToPos.reserve(ArmKinematics::kJointCount);
		for (int j = 1; j <= ArmKinematics::kJointCount; j++)
		{
			if (sp.pos[j] < 0) continue;
			jointToPos.push_back({ j, sp.pos[j] });
		}

		ArmCommsService::Instance().ClearMoveQueue();

		// 平移：用更短的执行时长（微调），但步长已按速度等比缩放，所以“速度不变”
		const int timeMs = std::max(120, std::min(800, translateMs));
		if (!m_pMotion->MoveJointsAbs(jointToPos, timeMs))
		{
			outWhy = L"下发失败（step mode）：未配置 ServoId 或无有效关节目标。";
			TraceEntry te{};
			te.tickMs = now;
			FillInputToTrace(te);
			te.ev = TraceEvent::SendFailed;
			te.prevTarget = ToTracePose(m_target);
			te.nextTarget = ToTracePose(next);
			te.usedReadbackCount = m_lastUsedReadbackCount;
			te.usedPos = usedPos;
			te.usedFromReadback = usedFromRb;
			te.ikOk = true;
			te.sendTimeMs = timeMs;
			(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
			TracePush(te);
			return false;
		}

		// 更新内部目标与锁
		m_target = next;
		m_lastGoodTarget = m_target;
		m_hasLastGoodTarget = true;
		SyncFpsStateFromTarget();
		for (int j = 1; j <= ArmKinematics::kJointCount; j++)
		{
			m_lastSentJointPos[(size_t)j] = sp.pos[j];
		}
		m_hasLastSentJointPos = true;
		m_stepLockedUntil = now + (ULONGLONG)timeMs;

		TraceEntry te{};
		te.tickMs = now;
		FillInputToTrace(te);
		te.ev = TraceEvent::StepModeSent;
		te.prevTarget = ToTracePose(m_target);
		te.nextTarget = ToTracePose(next);
		te.usedReadbackCount = m_lastUsedReadbackCount;
		te.usedPos = usedPos;
		te.usedFromReadback = usedFromRb;
		te.ikOk = true;
		te.sendTimeMs = timeMs;
		TracePush(te);
		return true;
	}

	const double dt = (double)periodMs / 1000.0;

	// 先缓存上一帧 target。IK 失败时必须回滚，避免 target 漂移到不可达区域导致“越失败越不可达”。
	const auto prevTarget = m_target;

	// [闭环] FPS 模式下：每次先用“当前读回估计姿态(FK)”重置状态，再做 delta。
	// 目的：避免长期积分导致的漂移（你反馈“开始正常，后来就乱”典型就是无反馈积分漂移）。
	ArmKinematics::JointAnglesRad qCur;
	if (m_inputKind == InputKind::Fps)
	{
		BuildCurrentJointEstimate(m_pMotion->Config(), *m_pKc, qCur, nullptr, nullptr);
		const auto poseCur = ArmKinematics::ForwardKinematics(*m_pKc, qCur);
		// 读回姿态不要每 tick 都覆盖：否则小步进会被量化吞掉（看起来几乎不动），且读回噪声会导致抖动。
		// - 无输入：总是同步，确保松手后状态稳定
		// - 有输入：最多 5Hz 同步一次，用于纠偏漂移
		static ULONGLONG s_lastSync = 0;
		const bool wantSync = (!m_fpsInput.active) || (s_lastSync == 0) || ((now - s_lastSync) >= 200);
		if (wantSync)
		{
			// [A3] 同步护栏：只有当“读回足够新鲜且差异不大”时才用 poseCur 覆盖 target。
			// 目的：避免偶发读回异常/过期导致 target 猛跳。
			const DWORD staleMs = (DWORD)AfxGetApp()->GetProfileInt(L"Readback", L"StaleMs", 800);
			const double maxSnapMm = (double)AfxGetApp()->GetProfileInt(L"Readback", L"MaxTargetSnapMm", 40);
			const double maxSnapPitchDeg = (double)AfxGetApp()->GetProfileInt(L"Readback", L"MaxTargetSnapPitchDeg", 15);

			bool allow = true;
			// 连接后至少 3 路新鲜读回才认为姿态可信（防止用 home/lastSent 去“校正” target）
			if (ArmCommsService::Instance().IsConnected() && m_lastUsedReadbackCount < 3)
			{
				allow = false;
			}
			else
			{
				// 差异阈值
				const double dx = poseCur.x_mm - m_target.x_mm;
				const double dy = poseCur.y_mm - m_target.y_mm;
				const double dz = poseCur.z_mm - m_target.z_mm;
				const double dxyz = std::sqrt(dx * dx + dy * dy + dz * dz);
				const double dp = std::fabs(poseCur.pitch_deg - m_target.pitch_deg);
				if (dxyz > maxSnapMm || dp > maxSnapPitchDeg)
				{
					allow = false;
				}
			}

			// 额外：若读回中出现过期（age>staleMs），也拒绝同步
			if (allow)
			{
				const MotionConfig& mc = m_pMotion->Config();
				for (int j = 1; j <= ArmKinematics::kJointCount; j++)
				{
					const int sid = mc.Get(j).servoId;
					if (sid < 1 || sid > 6) continue;
					uint16_t rb = 0;
					DWORD age = 0;
					if (!ArmCommsService::Instance().GetLastReadPosEx((uint8_t)sid, rb, age) || age > staleMs)
					{
						allow = false;
						break;
					}
				}
			}

			if (allow)
			{
				m_target = poseCur;
				SyncFpsStateFromTarget();
				s_lastSync = now;
			}
		}
	}

	ArmKinematics::PoseTarget nextTarget = m_target;
	std::wstring stepWhy;

	TraceEntry te{};
	te.tickMs = now;
	FillInputToTrace(te);
	te.prevTarget = ToTracePose(m_target);

	// [关键修复] FPS 模式下保存积分状态；若 IK/限位失败则回滚，避免内部状态“漂移”造成猛烈跳变。
	const bool isFps = (m_inputKind == InputKind::Fps);
	const double prevFpsR = m_fps_r_mm;
	const double prevFpsZ = m_fps_z_mm;
	const double prevFpsPhi = m_fps_phi_deg;
	const double prevFpsYaw = m_fps_baseYaw_rad;

	if (m_inputKind == InputKind::Fps)
	{
		nextTarget = StepTargetByFps(m_fpsInput, dt, stepWhy);
	}
	else
	{
		nextTarget = StepTargetByCartesian(m_cartInput, dt);
	}
	te.nextTarget = ToTracePose(nextTarget);

	// 读取当前关节角估算，用于 IK 择优（FPS 模式已提前构建并用于 FK 同步）
	if (m_inputKind != InputKind::Fps)
	{
		std::array<int, ArmKinematics::kJointCount + 1> usedPos{};
		std::array<uint8_t, ArmKinematics::kJointCount + 1> usedFromRb{};
		BuildCurrentJointEstimate(m_pMotion->Config(), *m_pKc, qCur, &usedPos, &usedFromRb);
		te.usedReadbackCount = m_lastUsedReadbackCount;
		te.usedPos = usedPos;
		te.usedFromReadback = usedFromRb;
	}

	// IK
	const auto ik = ArmKinematics::InverseKinematics(*m_pKc, &m_pMotion->Config(), nextTarget, &qCur);
	te.ikOk = ik.ok;
	te.chosenIndex = ik.chosenIndex;
	if (ik.chosenIndex >= 0 && ik.chosenIndex < (int)ik.candidates.size())
	{
		te.withinLimits = ik.candidates[ik.chosenIndex].withinLimits;
	}

	if (!ik.ok)
	{
		// 回滚 FPS 积分状态（否则下一次可达时会突然跳变）
		if (isFps)
		{
			m_fps_r_mm = prevFpsR;
			m_fps_z_mm = prevFpsZ;
			m_fps_phi_deg = prevFpsPhi;
			m_fps_baseYaw_rad = prevFpsYaw;
		}

		// IK 失败：回滚/恢复到上一次可达位置，避免 target 漂移导致持续失败。
		m_target = m_hasLastGoodTarget ? m_lastGoodTarget : prevTarget;
		outWhy = ik.reason.empty() ? L"IK 失败。" : ik.reason;
		te.ev = TraceEvent::IkFailed;
		(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
		TracePush(te);
		return false;
	}

	// 软限位一致性：若最佳解超出软限位，则本次视为失败并回滚。
	// 否则下发阶段会被 MotionController clamp，导致“看起来在动但实际被裁剪”，闭环会持续积累误差。
	if (ik.chosenIndex >= 0 &&
	    ik.chosenIndex < (int)ik.candidates.size() &&
	    !ik.candidates[ik.chosenIndex].withinLimits)
	{
		if (isFps)
		{
			m_fps_r_mm = prevFpsR;
			m_fps_z_mm = prevFpsZ;
			m_fps_phi_deg = prevFpsPhi;
			m_fps_baseYaw_rad = prevFpsYaw;
		}
		m_target = m_hasLastGoodTarget ? m_lastGoodTarget : prevTarget;
		outWhy = ik.reason.empty() ? L"软限位抑制：目标超出关节限位范围。" : ik.reason;
		te.ev = TraceEvent::LimitRejected;
		(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
		TracePush(te);
		return false;
	}

	// 只有 IK 成功才提交本次 target
	m_target = nextTarget;
	m_lastGoodTarget = m_target;
	m_hasLastGoodTarget = true;
	SyncFpsStateFromTarget();

	// 关节角 -> 舵机位置
	ArmKinematics::ServoPos sp;
	std::wstring why;
	if (!ArmKinematics::JointAnglesToServoPos(*m_pKc, &m_pMotion->Config(), ik.chosenQ, sp, why))
	{
		outWhy = why.empty() ? L"角度转舵机位置失败。" : why;
		return false;
	}

	// 输出 deadband：变化太小就不发（避免舵机不响应/抖动），让目标在内部累计到足够变化再发
	{
		int maxDelta = 0;
		for (int j = 1; j <= ArmKinematics::kJointCount; j++)
		{
			if (sp.pos[j] < 0) continue;
			const int prev = m_lastSentJointPos[(size_t)j];
			if (m_hasLastSentJointPos && prev > -999000)
			{
				maxDelta = std::max(maxDelta, std::abs(sp.pos[j] - prev));
			}
			else
			{
				maxDelta = std::max(maxDelta, 999);
			}
		}

		const int kMinDeltaPos = 2;
		if (m_hasLastSentJointPos && maxDelta < kMinDeltaPos)
		{
			outWhy = L"deadband: delta too small";
			te.ev = TraceEvent::DeadbandSuppressed;
			te.maxDeltaPos = maxDelta;
			(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
			TracePush(te);
			return true;
		}
		te.maxDeltaPos = maxDelta;
	}

	// "最新指令优先"：只清理 Move 队列，避免误清读回/脚本/其他指令
	ArmCommsService::Instance().ClearMoveQueue();

	std::vector<std::pair<int, int>> jointToPos;
	jointToPos.reserve(ArmKinematics::kJointCount);
	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		if (sp.pos[j] < 0) continue;
		jointToPos.push_back({ j, sp.pos[j] });
	}

	const int timeMs = (int)std::max<ULONGLONG>(periodMs, 30ULL); // 稍大于节拍，避免舵机抖动
	if (!m_pMotion->MoveJointsAbs(jointToPos, timeMs))
	{
		outWhy = L"下发失败：未配置 ServoId 或无有效关节目标。";
		te.ev = TraceEvent::SendFailed;
		te.sendTimeMs = timeMs;
		(void)wcsncpy_s(te.why, outWhy.c_str(), _TRUNCATE);
		TracePush(te);
		return false;
	}

	for (int j = 1; j <= ArmKinematics::kJointCount; j++)
	{
		m_lastSentJointPos[(size_t)j] = sp.pos[j];
	}
	m_hasLastSentJointPos = true;

	te.ev = TraceEvent::SendOk;
	te.sendTimeMs = timeMs;
	TracePush(te);
	return true;
}



