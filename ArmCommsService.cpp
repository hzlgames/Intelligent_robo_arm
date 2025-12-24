#include "pch.h"

#include "ArmCommsService.h"

#include "ArmProtocol.h"

#include <algorithm>

namespace
{
	CString FrameSummary(const ArmProtocol::ParsedFrame& f)
	{
		CString s;
		if (f.cmd == ArmProtocol::Command::Move)
		{
			s.Format(L"cmd=0x03 Move time=%ums servos=%zu", (unsigned)f.timeMs, f.servos.size());
		}
		else if (f.cmd == ArmProtocol::Command::ReadPosition)
		{
			if (f.isReadResponse)
			{
				CString detail;
				for (size_t i = 0; i < f.servos.size(); i++)
				{
					CString item;
					item.Format(L"%u=%u", (unsigned)f.servos[i].id, (unsigned)f.servos[i].position);
					if (i > 0) detail += L", ";
					detail += item;
				}
				s.Format(L"cmd=0x15 RESP [%s]", detail.GetString());
			}
			else
			{
				s.Format(L"cmd=0x15 REQ ids=%zu", f.readIds.size());
			}
		}
		else
		{
			s.Format(L"cmd=0x%02X (unknown)", (unsigned)(uint8_t)f.cmd);
		}
		return s;
	}
}

ArmCommsService& ArmCommsService::Instance()
{
	static ArmCommsService s;
	return s;
}

void ArmCommsService::SetSendStatsCallback(SendStatsCallback cb)
{
	m_sendStatsCb = std::move(cb);
}

bool ArmCommsService::ConnectSim()
{
	m_lastError.clear();
	m_connectedCom.clear();
	m_useSim = true;
	if (!m_fake.IsOpen())
	{
		m_fake.Open();
	}
	m_connected = true;
	LogLine(L"[INFO] Connected (simulated).");
	return true;
}

bool ArmCommsService::ConnectReal(const std::wstring& comName, DWORD baud)
{
	m_lastError.clear();
	m_connectedCom.clear();
	m_useSim = false;
	m_fake.Close();
	if (!m_real.Open(comName, baud))
	{
		m_lastError = m_real.GetLastErrorText();
		std::wstring msg = L"[ERR] Failed to open serial port: " + m_lastError;
		LogLine(msg);
		m_connected = false;
		return false;
	}
	m_connectedCom = comName;
	m_connected = true;
	LogLine(L"[INFO] Connected (real).");
	return true;
}

void ArmCommsService::Disconnect()
{
	if (m_connected)
	{
		LogLine(L"[INFO] Disconnected.");
	}
	m_connected = false;
	m_connectedCom.clear();
	m_real.Close();
	m_fake.Close();
	ClearTxQueue();
}

void ArmCommsService::Tick()
{
	PumpTx();
	PollRx();
}

void ArmCommsService::ClearTxQueue()
{
	m_txQueue.clear();
}

void ArmCommsService::EmergencyStop()
{
	ClearTxQueue();
	LogLine(L"[WARN] EmergencyStop: TX queue cleared.");
}

bool ArmCommsService::GetLastReadPos(uint8_t id, uint16_t& outPos) const
{
	if (id < 1 || id > 6) return false;
	if (!m_lastReadValid[id]) return false;
	outPos = m_lastReadPos[id];
	return true;
}

void ArmCommsService::ClearReadback()
{
	for (int i = 0; i <= 6; i++)
	{
		m_lastReadPos[i] = 0;
		m_lastReadValid[i] = false;
	}
}

int ArmCommsService::AddLogListener(LogListener cb)
{
	const int id = m_nextSubId++;
	m_logSubs.push_back(LogSub{ id, std::move(cb) });
	return id;
}

void ArmCommsService::RemoveLogListener(int token)
{
	m_logSubs.erase(std::remove_if(m_logSubs.begin(), m_logSubs.end(),
		[token](const LogSub& s) { return s.id == token; }), m_logSubs.end());
}

int ArmCommsService::AddFrameListener(FrameListener cb)
{
	const int id = m_nextSubId++;
	m_frameSubs.push_back(FrameSub{ id, std::move(cb) });
	return id;
}

void ArmCommsService::RemoveFrameListener(int token)
{
	m_frameSubs.erase(std::remove_if(m_frameSubs.begin(), m_frameSubs.end(),
		[token](const FrameSub& s) { return s.id == token; }), m_frameSubs.end());
}

int ArmCommsService::GetThrottleMs() const
{
	return AfxGetApp()->GetProfileInt(L"Throttle", L"Ms", 50);
}

void ArmCommsService::EnqueueTx(std::vector<uint8_t> bytes)
{
	m_txQueue.push_back(std::move(bytes));
}

void ArmCommsService::PumpTx()
{
	if (m_txQueue.empty())
	{
		return;
	}
	const DWORD now = GetTickCount();
	const int throttle = GetThrottleMs();
	const DWORD elapsed = now - m_lastTxTick;
	if (m_lastTxTick != 0 && elapsed < static_cast<DWORD>(throttle))
	{
		return;
	}
	auto bytes = std::move(m_txQueue.front());
	m_txQueue.pop_front();
	TxBytesNow(bytes);
	m_lastTxTick = GetTickCount();
}

void ArmCommsService::TxBytesNow(const std::vector<uint8_t>& bytes)
{
	if (!m_connected)
	{
		LogLine(L"[WARN] Not connected.");
		return;
	}

	{
		const std::wstring hex = ArmProtocol::ToHex(bytes.data(), bytes.size());
		std::wstring line = L"[TX] " + hex;
		LogLine(line);
	}

	if (m_useSim)
	{
		if (!m_fake.IsOpen()) m_fake.Open();
		m_fake.Write(bytes.data(), bytes.size());
	}
	else
	{
		if (!m_real.WriteBytes(bytes.data(), bytes.size(), nullptr))
		{
			m_lastError = m_real.GetLastErrorText();
			std::wstring line = L"[ERR] Write failed: " + m_lastError;
			LogLine(line);
		}
	}

	// Notify stats callback (for Control page FPS display)
	if (m_sendStatsCb) m_sendStatsCb();
}

void ArmCommsService::PollRx()
{
	if (!m_connected)
	{
		return;
	}

	std::vector<uint8_t> chunk;
	if (m_useSim)
	{
		chunk = m_fake.ReadAvailable();
	}
	else
	{
		chunk = m_real.ReadAvailable();
	}
	if (chunk.empty())
	{
		return;
	}

	{
		const std::wstring hex = ArmProtocol::ToHex(chunk.data(), chunk.size());
		std::wstring line = L"[RX] " + hex;
		LogLine(line);
	}

	m_rxBuf.insert(m_rxBuf.end(), chunk.begin(), chunk.end());
	while (!m_rxBuf.empty())
	{
		ArmProtocol::ParsedFrame f;
		size_t consumed = 0;
		const bool ok = ArmProtocol::TryParseOne(m_rxBuf.data(), m_rxBuf.size(), f, consumed);
		if (consumed > 0)
		{
			m_rxBuf.erase(m_rxBuf.begin(), m_rxBuf.begin() + static_cast<ptrdiff_t>(consumed));
		}
		if (!ok)
		{
			if (consumed == 0) break;
			continue;
		}

		{
			CString sum = FrameSummary(f);
			std::wstring line = L"[PARSE] " + std::wstring(sum.GetString());
			LogLine(line);
		}

		for (const auto& sub : m_frameSubs)
		{
			if (sub.cb) sub.cb(f);
		}

		if (f.cmd == ArmProtocol::Command::ReadPosition && f.isReadResponse)
		{
			for (const auto& s : f.servos)
			{
				if (s.id >= 1 && s.id <= 6)
				{
					m_lastReadPos[s.id] = s.position;
					m_lastReadValid[s.id] = true;
				}
			}
		}
	}
}

void ArmCommsService::LogLine(const std::wstring& line)
{
	for (const auto& sub : m_logSubs)
	{
		if (sub.cb) sub.cb(line);
	}
}
