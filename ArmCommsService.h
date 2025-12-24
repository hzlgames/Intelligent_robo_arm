#pragma once

#include <Windows.h>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "FakeSerialPort.h"
#include "SerialPortWin32.h"

namespace ArmProtocol { struct ParsedFrame; }

// Unified comms service for both Serial and Motion pages.
// - Single place for connect/disconnect (real/sim)
// - Throttled TX queue (Throttle\\Ms)
// - RX polling + protocol parsing + readback cache
// - Broadcast logs and parsed frames to multiple listeners
class ArmCommsService
{
public:
	using LogListener = std::function<void(const std::wstring& line)>;
	using FrameListener = std::function<void(const ArmProtocol::ParsedFrame& f)>;
	using SendStatsCallback = std::function<void()>;

	static ArmCommsService& Instance();

	// Global send stats callback (for Control page FPS display)
	void SetSendStatsCallback(SendStatsCallback cb);

	// Connection lifecycle
	bool ConnectSim();
	bool ConnectReal(const std::wstring& comName, DWORD baud = CBR_9600);
	void Disconnect();
	bool IsConnected() const { return m_connected; }
	bool IsSim() const { return m_useSim; }
	std::wstring GetConnectedCom() const { return m_connectedCom; }
	std::wstring GetLastErrorText() const { return m_lastError; }

	// Periodic pump: call from a UI timer (e.g., 20~50ms).
	void Tick();

	// TX queue
	void ClearTxQueue();
	void EnqueueTx(std::vector<uint8_t> bytes);
	void EmergencyStop(); // clears queue; optional future: send hold position

	// RX / readback
	bool GetLastReadPos(uint8_t id, uint16_t& outPos) const;
	void ClearReadback();

	// Listeners (caller must remove on destroy)
	int AddLogListener(LogListener cb);
	void RemoveLogListener(int token);
	int AddFrameListener(FrameListener cb);
	void RemoveFrameListener(int token);

private:
	ArmCommsService() = default;
	ArmCommsService(const ArmCommsService&) = delete;
	ArmCommsService& operator=(const ArmCommsService&) = delete;

	int GetThrottleMs() const;
	void PumpTx();
	void PollRx();
	void TxBytesNow(const std::vector<uint8_t>& bytes);

	void LogLine(const std::wstring& line);

private:
	bool m_connected = false;
	bool m_useSim = true;
	std::wstring m_connectedCom;
	std::wstring m_lastError;

	SerialPortWin32 m_real;
	FakeSerialPort m_fake;

	// TX throttling + queue
	std::deque<std::vector<uint8_t>> m_txQueue;
	DWORD m_lastTxTick = 0;

	// RX parsing buffer
	std::vector<uint8_t> m_rxBuf;

	// Readback cache (ids 1..6)
	uint16_t m_lastReadPos[7] = { 0 };
	bool m_lastReadValid[7] = { false };

	struct LogSub { int id; LogListener cb; };
	struct FrameSub { int id; FrameListener cb; };
	int m_nextSubId = 1;
	std::vector<LogSub> m_logSubs;
	std::vector<FrameSub> m_frameSubs;

	SendStatsCallback m_sendStatsCb;
};


