#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Arm serial protocol pack/parse (based on Reference/SerialPort.h).
// Frame format:
//   [0]=0x55 [1]=0x55 [2]=len [3]=cmd ...
// len counts bytes from [2] (including [2]) to end of frame, so total frame size = 2 + len.
namespace ArmProtocol
{
	struct ServoTarget
	{
		uint8_t id = 0;
		uint16_t position = 0; // default assumption: 0..1000; upper layer can enforce limits/calibration.
	};

	enum class Command : uint8_t
	{
		Move = 0x03,
		ReadPosition = 0x15,
	};

	struct ParsedFrame
	{
		Command cmd = Command::Move;
		uint16_t timeMs = 0;                 // Move only
		std::vector<ServoTarget> servos;     // Move/ReadPositionResponse
		std::vector<uint8_t> readIds;        // ReadPositionRequest
		bool isReadResponse = false;
	};

	// Pack: move N servos to target positions within timeMs
	std::vector<uint8_t> PackMove(const std::vector<ServoTarget>& servos, uint16_t timeMs);

	// Pack: read N servo positions
	std::vector<uint8_t> PackReadPosition(const std::vector<uint8_t>& ids);

	// Parse: attempt to parse one frame from a byte stream; supports junk prefix + partial frames.
	// - consumed: bytes to drop from the front of buffer (may be 0)
	// - returns true if a full frame was parsed into out
	bool TryParseOne(const uint8_t* buffer, size_t bufferLen, ParsedFrame& out, size_t& consumed);

	// Utility: bytes to HEX string (for diagnostics)
	std::wstring ToHex(const uint8_t* data, size_t len);
}


