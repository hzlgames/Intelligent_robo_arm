#include "pch.h"

#include "FakeSerialPort.h"
#include "ArmProtocol.h"

#include <algorithm>
#include <random>

namespace
{
	std::mt19937& Rng()
	{
		// Fixed seed for reproducibility. If you prefer true randomness, use std::random_device.
		static std::mt19937 rng(0xC0FFEEu);
		return rng;
	}

	inline double Clamp01(double v)
	{
		if (v < 0.0) return 0.0;
		if (v > 1.0) return 1.0;
		return v;
	}
}

FakeSerialPort::FakeSerialPort()
{
	Reset();
}

void FakeSerialPort::Reset()
{
	m_in.clear();
	m_pending.clear();
	m_stats = Stats{};
	for (int i = 0; i <= 6; i++)
	{
		m_pos[i] = 500;
	}
}

void FakeSerialPort::SetFaultConfig(const FaultConfig& cfg)
{
	m_fault = cfg;
	if (m_fault.maxDelayMs < m_fault.minDelayMs)
	{
		std::swap(m_fault.maxDelayMs, m_fault.minDelayMs);
	}
	m_fault.dropRate = Clamp01(m_fault.dropRate);
	m_fault.corruptRate = Clamp01(m_fault.corruptRate);
}

FakeSerialPort::FaultConfig FakeSerialPort::GetFaultConfig() const
{
	return m_fault;
}

bool FakeSerialPort::Open()
{
	m_open = true;
	return true;
}

void FakeSerialPort::Close()
{
	m_open = false;
}

bool FakeSerialPort::IsOpen() const
{
	return m_open;
}

uint64_t FakeSerialPort::NowTick() const
{
	return ::GetTickCount64();
}

uint32_t FakeSerialPort::RandDelayMs()
{
	if (m_fault.minDelayMs == m_fault.maxDelayMs)
	{
		return m_fault.minDelayMs;
	}
	std::uniform_int_distribution<uint32_t> dist(m_fault.minDelayMs, m_fault.maxDelayMs);
	return dist(Rng());
}

bool FakeSerialPort::Chance(double p)
{
	if (p <= 0.0) return false;
	if (p >= 1.0) return true;
	std::uniform_real_distribution<double> dist(0.0, 1.0);
	return dist(Rng()) < p;
}

void FakeSerialPort::MaybeCorrupt(std::vector<uint8_t>& bytes)
{
	if (bytes.empty()) return;
	if (!Chance(m_fault.corruptRate)) return;

	std::uniform_int_distribution<size_t> idxDist(0, bytes.size() - 1);
	const size_t idx = idxDist(Rng());
	bytes[idx] ^= 0xFF; // flip bits
	m_stats.responsesCorrupted++;
}

void FakeSerialPort::Write(const uint8_t* data, size_t len)
{
	if (!m_open || !data || len == 0)
	{
		return;
	}

	m_stats.bytesWritten += len;
	m_in.insert(m_in.end(), data, data + len);

	// Parse as many frames as possible from the input buffer.
	while (!m_in.empty())
	{
		ArmProtocol::ParsedFrame frame;
		size_t consumed = 0;
		const bool ok = ArmProtocol::TryParseOne(m_in.data(), m_in.size(), frame, consumed);

		if (consumed > 0)
		{
			m_in.erase(m_in.begin(), m_in.begin() + static_cast<ptrdiff_t>(consumed));
		}

		if (!ok)
		{
			// Need more data, or we dropped junk. If nothing was consumed, stop to avoid a loop.
			if (consumed == 0)
			{
				break;
			}
			continue;
		}

		m_stats.framesParsed++;

		if (Chance(m_fault.dropRate))
		{
			m_stats.framesDropped++;
			continue;
		}

		switch (frame.cmd)
		{
		case ArmProtocol::Command::Move:
		{
			for (const auto& s : frame.servos)
			{
				if (s.id <= 6)
				{
					m_pos[s.id] = s.position;
				}
			}
		}
		break;
		case ArmProtocol::Command::ReadPosition:
		{
			// If parsed as a request: readIds not empty and isReadResponse=false
			if (!frame.isReadResponse && !frame.readIds.empty())
			{
				std::vector<ArmProtocol::ServoTarget> servos;
				servos.reserve(frame.readIds.size());
				for (uint8_t id : frame.readIds)
				{
					ArmProtocol::ServoTarget st;
					st.id = id;
					st.position = (id <= 6) ? m_pos[id] : 0;
					servos.push_back(st);
				}

				// Build response: 0x15 response has len=n*3+3
				std::vector<uint8_t> resp;
				const uint8_t n = static_cast<uint8_t>(std::min<size_t>(servos.size(), 0xFF));
				const uint8_t respLen = static_cast<uint8_t>(n * 3 + 3);
				resp.reserve(static_cast<size_t>(2 + respLen));
				resp.push_back(0x55);
				resp.push_back(0x55);
				resp.push_back(respLen);
				resp.push_back(static_cast<uint8_t>(ArmProtocol::Command::ReadPosition));
				resp.push_back(n);
				for (size_t i = 0; i < n; i++)
				{
					resp.push_back(servos[i].id);
					resp.push_back(static_cast<uint8_t>(servos[i].position & 0xFF));
					resp.push_back(static_cast<uint8_t>((servos[i].position >> 8) & 0xFF));
				}

				if (!Chance(m_fault.dropRate))
				{
					MaybeCorrupt(resp);

					Pending p;
					p.dueTick = NowTick() + RandDelayMs();
					p.bytes = std::move(resp);
					m_pending.push_back(std::move(p));
					m_stats.responsesQueued++;
				}
				else
				{
					m_stats.framesDropped++;
				}
			}
		}
		break;
		default:
			break;
		}
	}
}

std::vector<uint8_t> FakeSerialPort::ReadAvailable()
{
	std::vector<uint8_t> out;
	if (!m_open)
	{
		return out;
	}

	const uint64_t now = NowTick();
	while (!m_pending.empty())
	{
		if (m_pending.front().dueTick > now)
		{
			break;
		}
		auto& bytes = m_pending.front().bytes;
		out.insert(out.end(), bytes.begin(), bytes.end());
		m_pending.pop_front();
	}

	m_stats.bytesRead += out.size();
	return out;
}

uint16_t FakeSerialPort::GetServoPosition(uint8_t id) const
{
	if (id <= 6) return m_pos[id];
	return 0;
}

FakeSerialPort::Stats FakeSerialPort::GetStats() const
{
	return m_stats;
}


