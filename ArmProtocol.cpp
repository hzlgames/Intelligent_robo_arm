#include "pch.h"

#include "ArmProtocol.h"

#include <algorithm>
#include <sstream>
#include <string>

namespace
{
	constexpr uint8_t kHeader0 = 0x55;
	constexpr uint8_t kHeader1 = 0x55;

	inline void AppendU16LE(std::vector<uint8_t>& out, uint16_t v)
	{
		out.push_back(static_cast<uint8_t>(v & 0xFF));
		out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
	}

	inline uint16_t ReadU16LE(const uint8_t* p)
	{
		return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
	}

	inline bool HasHeaderAt(const uint8_t* b, size_t n, size_t i)
	{
		return i + 1 < n && b[i] == kHeader0 && b[i + 1] == kHeader1;
	}
}

std::vector<uint8_t> ArmProtocol::PackMove(const std::vector<ServoTarget>& servos, uint16_t timeMs)
{
	std::vector<uint8_t> out;
	const uint8_t n = static_cast<uint8_t>(std::min<size_t>(servos.size(), 0xFF));

	// len = n*3 + 5 (from the len byte itself to end-of-frame)
	const uint8_t len = static_cast<uint8_t>(n * 3 + 5);

	out.reserve(static_cast<size_t>(2 + len));
	out.push_back(kHeader0);
	out.push_back(kHeader1);
	out.push_back(len);
	out.push_back(static_cast<uint8_t>(Command::Move));
	out.push_back(n);
	AppendU16LE(out, timeMs);

	for (size_t i = 0; i < n; i++)
	{
		out.push_back(servos[i].id);
		AppendU16LE(out, servos[i].position);
	}
	return out;
}

std::vector<uint8_t> ArmProtocol::PackReadPosition(const std::vector<uint8_t>& ids)
{
	std::vector<uint8_t> out;
	const uint8_t n = static_cast<uint8_t>(std::min<size_t>(ids.size(), 0xFF));
	const uint8_t len = static_cast<uint8_t>(n + 3); // read request: len = n + 3

	out.reserve(static_cast<size_t>(2 + len));
	out.push_back(kHeader0);
	out.push_back(kHeader1);
	out.push_back(len);
	out.push_back(static_cast<uint8_t>(Command::ReadPosition));
	out.push_back(n);
	for (size_t i = 0; i < n; i++)
	{
		out.push_back(ids[i]);
	}
	return out;
}

bool ArmProtocol::TryParseOne(const uint8_t* buffer, size_t bufferLen, ParsedFrame& out, size_t& consumed)
{
	consumed = 0;
	if (!buffer || bufferLen < 4)
	{
		return false;
	}

	// Allow junk prefix: search for header
	size_t start = 0;
	while (start + 1 < bufferLen && !HasHeaderAt(buffer, bufferLen, start))
	{
		start++;
	}
	if (start > 0)
	{
		consumed = start; // drop junk
		return false;
	}

	if (bufferLen < 3)
	{
		return false;
	}

	const uint8_t len = buffer[2];
	const size_t frameSize = static_cast<size_t>(2 + len);
	if (frameSize < 4) // minimum: header(2)+len(1)+cmd(1)
	{
		consumed = 2; // drop header to avoid a loop
		return false;
	}
	if (bufferLen < frameSize)
	{
		return false; // not enough bytes for a full frame yet
	}

	const uint8_t cmd = buffer[3];
	out = ParsedFrame{};
	out.cmd = static_cast<Command>(cmd);

	if (out.cmd == Command::Move)
	{
		// header2 + len + cmd + count + time(2) + n*(id+pos(2))
		if (frameSize < 7)
		{
			consumed = frameSize;
			return false;
		}
		const uint8_t n = buffer[4];
		out.timeMs = ReadU16LE(buffer + 5);
		const size_t expected = static_cast<size_t>(2 + (n * 3 + 5));
		// Even if len mismatches, attempt best-effort parsing for diagnostics.
		const size_t base = 7;
		const size_t maxEntries = (frameSize >= base) ? ((frameSize - base) / 3) : 0;
		const size_t entries = std::min<size_t>(n, maxEntries);
		out.servos.reserve(entries);
		for (size_t i = 0; i < entries; i++)
		{
			const size_t off = base + i * 3;
			ServoTarget st;
			st.id = buffer[off + 0];
			st.position = ReadU16LE(buffer + off + 1);
			out.servos.push_back(st);
		}
		(void)expected;
	}
	else if (out.cmd == Command::ReadPosition)
	{
		if (frameSize < 5)
		{
			consumed = frameSize;
			return false;
		}
		const uint8_t n = buffer[4];
		const size_t payload = frameSize - 5;

		// Read request: payload == n
		// Read response: payload == n*3
		if (payload == static_cast<size_t>(n))
		{
			out.isReadResponse = false;
			out.readIds.assign(buffer + 5, buffer + 5 + n);
		}
		else
		{
			out.isReadResponse = true;
			const size_t entries = std::min<size_t>(n, payload / 3);
			out.servos.reserve(entries);
			for (size_t i = 0; i < entries; i++)
			{
				const size_t off = 5 + i * 3;
				ServoTarget st;
				st.id = buffer[off + 0];
				st.position = ReadU16LE(buffer + off + 1);
				out.servos.push_back(st);
			}
		}
	}
	else
	{
		// Unknown command: skip the frame (tolerant parsing for diagnostics)
	}

	consumed = frameSize;
	return true;
}

std::wstring ArmProtocol::ToHex(const uint8_t* data, size_t len)
{
	static const wchar_t* kHex = L"0123456789ABCDEF";
	std::wstring out;
	out.reserve(len * 3);
	for (size_t i = 0; i < len; i++)
	{
		const uint8_t b = data[i];
		out.push_back(kHex[(b >> 4) & 0xF]);
		out.push_back(kHex[b & 0xF]);
		if (i + 1 < len) out.push_back(L' ');
	}
	return out;
}


