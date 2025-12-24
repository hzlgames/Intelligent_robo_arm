
// 智能机械臂.cpp: 定义应用程序的类行为。
//

#include "pch.h"
#include "framework.h"
#include "智能机械臂.h"
#include "智能机械臂Dlg.h"

#include "ArmCommsService.h"

#include <mfapi.h>
#include <objbase.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// C智能机械臂App

BEGIN_MESSAGE_MAP(C智能机械臂App, CWinApp)
	ON_COMMAND(ID_HELP, &CWinApp::OnHelp)
END_MESSAGE_MAP()


// C智能机械臂App 构造

C智能机械臂App::C智能机械臂App()
{
	// 支持重新启动管理器
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_RESTART;

	// TODO: 在此处添加构造代码，
	// 将所有重要的初始化放置在 InitInstance 中
}


// 唯一的 C智能机械臂App 对象

C智能机械臂App theApp;


// C智能机械臂App 初始化

BOOL C智能机械臂App::InitInstance()
{
	// 如果应用程序存在以下情况，Windows XP 上需要 InitCommonControlsEx()
	// 使用 ComCtl32.dll 版本 6 或更高版本来启用可视化方式，
	//则需要 InitCommonControlsEx()。  否则，将无法创建窗口。
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// 将它设置为包括所有要在应用程序中使用的
	// 公共控件类。
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	// Initialize COM + Media Foundation for camera preview (required by MF source reader).
	// Without this, SetDevice may fail with CO_E_NOTINITIALIZED (0x800401F0).
	{
		const HRESULT hrCo = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
		if (SUCCEEDED(hrCo))
		{
			// S_OK or S_FALSE both require CoUninitialize().
			m_needCoUninitialize = true;
		}
		// If RPC_E_CHANGED_MODE, COM is already initialized with a different model; do not uninitialize.

		const HRESULT hrMf = MFStartup(MF_VERSION);
		if (SUCCEEDED(hrMf))
		{
			m_needMfShutdown = true;
		}
	}


	AfxEnableControlContainer();

	// Setup ArmCommsService stats callback (for Control page FPS display)
	ArmCommsService::Instance().SetSendStatsCallback([this]() {
		this->RecordSerialSend();
	});

	// 创建 shell 管理器，以防对话框包含
	// 任何 shell 树视图控件或 shell 列表视图控件。
	CShellManager *pShellManager = new CShellManager;

	// 激活“Windows Native”视觉管理器，以便在 MFC 控件中启用主题
	CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows));

	// 标准初始化
	// 如果未使用这些功能并希望减小
	// 最终可执行文件的大小，则应移除下列
	// 不需要的特定初始化例程
	// 更改用于存储设置的注册表项
	// TODO: 应适当修改该字符串，
	// 例如修改为公司或组织名
	SetRegistryKey(_T("应用程序向导生成的本地应用程序"));

	C智能机械臂Dlg dlg;
	m_pMainWnd = &dlg;
	INT_PTR nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: 在此放置处理何时用
		//  “确定”来关闭对话框的代码
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: 在此放置处理何时用
		//  “取消”来关闭对话框的代码
	}
	else if (nResponse == -1)
	{
		TRACE(traceAppMsg, 0, "警告: 对话框创建失败，应用程序将意外终止。\n");
		TRACE(traceAppMsg, 0, "警告: 如果您在对话框上使用 MFC 控件，则无法 #define _AFX_NO_MFC_CONTROLS_IN_DIALOGS。\n");
	}

	// 删除上面创建的 shell 管理器。
	if (pShellManager != nullptr)
	{
		delete pShellManager;
	}

#if !defined(_AFXDLL) && !defined(_AFX_NO_MFC_CONTROLS_IN_DIALOGS)
	ControlBarCleanUp();
#endif

	// 由于对话框已关闭，所以将返回 FALSE 以便退出应用程序，
	//  而不是启动应用程序的消息泵。
	return FALSE;
}

int C智能机械臂App::ExitInstance()
{
	// Match InitInstance initialization order.
	if (m_needMfShutdown)
	{
		MFShutdown();
		m_needMfShutdown = false;
	}
	if (m_needCoUninitialize)
	{
		CoUninitialize();
		m_needCoUninitialize = false;
	}
	return CWinApp::ExitInstance();
}

void C智能机械臂App::RecordSerialSend()
{
	const ULONGLONG now = ::GetTickCount64();
	std::lock_guard<std::mutex> lk(m_sendMu);
	m_lastSendTick = now;
	m_sendTicks.push_back(now);
	// keep last 1s
	while (!m_sendTicks.empty() && (now - m_sendTicks.front()) > 1000ULL)
	{
		m_sendTicks.pop_front();
	}
}

void C智能机械臂App::GetSerialSendStats(unsigned& outFps, unsigned& outSinceMs) const
{
	const ULONGLONG now = ::GetTickCount64();
	std::lock_guard<std::mutex> lk(m_sendMu);
	while (!m_sendTicks.empty() && (now - m_sendTicks.front()) > 1000ULL)
	{
		m_sendTicks.pop_front();
	}
	outFps = static_cast<unsigned>(m_sendTicks.size());
	outSinceMs = (m_lastSendTick == 0) ? 0u : static_cast<unsigned>(now - m_lastSendTick);
}

