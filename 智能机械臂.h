
// 智能机械臂.h: PROJECT_NAME 应用程序的主头文件
//

#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含 'pch.h' 以生成 PCH"
#endif

#include "resource.h"		// 主符号

#include <deque>
#include <mutex>

// C智能机械臂App:
// 有关此类的实现，请参阅 智能机械臂.cpp
//

class C智能机械臂App : public CWinApp
{
public:
	C智能机械臂App();

// 重写
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

	// Diagnostics: record serial send events (for Control page stats).
	void RecordSerialSend();
	void GetSerialSendStats(unsigned& outFps, unsigned& outSinceMs) const;

// 实现

	DECLARE_MESSAGE_MAP()

private:
	// Media Foundation / COM initialization flags (for camera preview).
	bool m_needCoUninitialize = false;
	bool m_needMfShutdown = false;

	// Rolling 1-second window of serial send ticks (GetTickCount64).
	mutable std::mutex m_sendMu;
	mutable std::deque<ULONGLONG> m_sendTicks;
	mutable ULONGLONG m_lastSendTick = 0;
};

extern C智能机械臂App theApp;
