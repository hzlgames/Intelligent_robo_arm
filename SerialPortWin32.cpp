#include "pch.h"

#include "SerialPortWin32.h"

#include <algorithm>
#include <sstream>

namespace
{
	std::wstring FormatWin32Error(DWORD err)
	{
		LPWSTR msg = nullptr;
		const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
		const DWORD len = FormatMessageW(flags, nullptr, err, 0, (LPWSTR)&msg, 0, nullptr);
		std::wstring s;
		if (len && msg)
		{
			s.assign(msg, msg + len);
			LocalFree(msg);
		}
		else
		{
			std::wstringstream ss;
			ss << L"Win32Error=" << err;
			s = ss.str();
		}
		return s;
	}

	std::wstring NormalizeComName(const std::wstring& com)
	{
		// Accept "COM3" or "\\\\.\\COM3"
		if (com.rfind(L"\\\\.\\", 0) == 0)
		{
			return com;
		}
		return L"\\\\.\\" + com;
	}
}

SerialPortWin32::~SerialPortWin32()
{
	Close();
}

bool SerialPortWin32::Open(const std::wstring& comName, DWORD baud)
{
	Close();
	m_lastError.clear();

	const std::wstring path = NormalizeComName(comName);
	m_h = CreateFileW(
		path.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		nullptr,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		nullptr);

	if (m_h == INVALID_HANDLE_VALUE)
	{
		m_lastError = FormatWin32Error(GetLastError());
		return false;
	}

	DCB dcb{};
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(m_h, &dcb))
	{
		m_lastError = FormatWin32Error(GetLastError());
		Close();
		return false;
	}

	dcb.BaudRate = baud;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;

	if (!SetCommState(m_h, &dcb))
	{
		m_lastError = FormatWin32Error(GetLastError());
		Close();
		return false;
	}

	COMMTIMEOUTS to{};
	// Non-blocking-ish: ReadFile returns immediately with available bytes.
	to.ReadIntervalTimeout = MAXDWORD;
	to.ReadTotalTimeoutConstant = 0;
	to.ReadTotalTimeoutMultiplier = 0;
	to.WriteTotalTimeoutConstant = 200;
	to.WriteTotalTimeoutMultiplier = 10;
	SetCommTimeouts(m_h, &to);

	PurgeComm(m_h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	return true;
}

void SerialPortWin32::Close()
{
	if (m_h != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_h);
		m_h = INVALID_HANDLE_VALUE;
	}
}

bool SerialPortWin32::WriteBytes(const uint8_t* data, size_t len, DWORD* bytesWritten)
{
	if (bytesWritten) *bytesWritten = 0;
	if (!IsOpen() || !data || len == 0)
	{
		return false;
	}

	DWORD written = 0;
	const BOOL ok = WriteFile(m_h, data, static_cast<DWORD>(len), &written, nullptr);
	if (bytesWritten) *bytesWritten = written;
	if (!ok)
	{
		m_lastError = FormatWin32Error(GetLastError());
		return false;
	}
	return true;
}

std::vector<uint8_t> SerialPortWin32::ReadAvailable(size_t maxBytes)
{
	std::vector<uint8_t> out;
	if (!IsOpen() || maxBytes == 0)
	{
		return out;
	}

	DWORD errors = 0;
	COMSTAT st{};
	if (!ClearCommError(m_h, &errors, &st))
	{
		m_lastError = FormatWin32Error(GetLastError());
		return out;
	}

	const DWORD avail = st.cbInQue;
	if (avail == 0)
	{
		return out;
	}

	const size_t want = (maxBytes < static_cast<size_t>(avail)) ? maxBytes : static_cast<size_t>(avail);
	const DWORD toRead = static_cast<DWORD>(want);
	out.resize(toRead);

	DWORD got = 0;
	const BOOL ok = ReadFile(m_h, out.data(), toRead, &got, nullptr);
	if (!ok)
	{
		m_lastError = FormatWin32Error(GetLastError());
		out.clear();
		return out;
	}
	out.resize(got);
	return out;
}


