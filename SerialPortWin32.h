#pragma once

#include <Windows.h>
#include <cstdint>
#include <string>
#include <vector>

// Minimal Win32 serial port wrapper for diagnostics (non-overlapped, pollable reads).
class SerialPortWin32
{
public:
	SerialPortWin32() = default;
	~SerialPortWin32();

	SerialPortWin32(const SerialPortWin32&) = delete;
	SerialPortWin32& operator=(const SerialPortWin32&) = delete;

	bool Open(const std::wstring& comName, DWORD baud = CBR_9600);
	void Close();
	bool IsOpen() const { return m_h != INVALID_HANDLE_VALUE; }

	bool WriteBytes(const uint8_t* data, size_t len, DWORD* bytesWritten = nullptr);
	std::vector<uint8_t> ReadAvailable(size_t maxBytes = 4096);

	std::wstring GetLastErrorText() const { return m_lastError; }

private:
	HANDLE m_h = INVALID_HANDLE_VALUE;
	std::wstring m_lastError;
};


