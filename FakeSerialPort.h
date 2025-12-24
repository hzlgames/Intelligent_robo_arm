#pragma once

#include <cstdint>
#include <deque>
#include <vector>

// In-process serial simulator (no hardware required).
// - Write(): ingest outgoing bytes and parse protocol frames
// - ReadAvailable(): returns any due response bytes (pollable via a UI timer)
class FakeSerialPort
{
public:
	struct FaultConfig
	{
		// Response delay range in milliseconds. min=max means fixed delay.
		uint32_t minDelayMs = 10;
		uint32_t maxDelayMs = 10;

		// 0..1: probability to drop a request/response (stress testing)
		double dropRate = 0.0;

		// 0..1: probability to corrupt one random response byte (parser robustness)
		double corruptRate = 0.0;
	};

	struct Stats
	{
		uint64_t bytesWritten = 0;
		uint64_t bytesRead = 0;
		uint64_t framesParsed = 0;
		uint64_t framesDropped = 0;
		uint64_t responsesQueued = 0;
		uint64_t responsesCorrupted = 0;
	};

	FakeSerialPort();

	void Reset();
	void SetFaultConfig(const FaultConfig& cfg);
	FaultConfig GetFaultConfig() const;

	// Open/close (to mimic a real serial port lifecycle)
	bool Open();
	void Close();
	bool IsOpen() const;

	// Write outgoing bytes (may contain multiple frames / partial frames / junk)
	void Write(const uint8_t* data, size_t len);

	// Read: pop all currently available response bytes (may be empty)
	std::vector<uint8_t> ReadAvailable();

	// Current simulated servo position (ids 1..6 are meaningful)
	uint16_t GetServoPosition(uint8_t id) const;

	Stats GetStats() const;

private:
	struct Pending
	{
		uint64_t dueTick = 0; // GetTickCount64()
		std::vector<uint8_t> bytes;
	};

	uint64_t NowTick() const;
	uint32_t RandDelayMs();
	bool Chance(double p);
	void MaybeCorrupt(std::vector<uint8_t>& bytes);

private:
	bool m_open = false;
	FaultConfig m_fault{};
	Stats m_stats{};

	// Input buffer (handles partial/concatenated frames)
	std::vector<uint8_t> m_in;

	// Scheduled responses
	std::deque<Pending> m_pending;

	// Servo positions (1..6), default 500
	uint16_t m_pos[7] = { 0 };
};


