
// 智能机械臂Dlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "智能机械臂.h"
#include "智能机械臂Dlg.h"
#include "afxdialogex.h"
#include "DiagnosticsSheet.h"
#include "SettingsIo.h"
#include "AppMessages.h"

#include <windows.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// C智能机械臂Dlg 对话框



C智能机械臂Dlg::C智能机械臂Dlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MY_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void C智能机械臂Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(C智能机械臂Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_DIAGNOSTICS, &C智能机械臂Dlg::OnBnClickedDiagnostics)
	ON_BN_CLICKED(IDC_BTN_EXPORT_PARAMS, &C智能机械臂Dlg::OnBnClickedExportParams)
	ON_BN_CLICKED(IDC_BTN_IMPORT_PARAMS, &C智能机械臂Dlg::OnBnClickedImportParams)
END_MESSAGE_MAP()


// C智能机械臂Dlg 消息处理程序

BOOL C智能机械臂Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void C智能机械臂Dlg::OnBnClickedDiagnostics()
{
	CDiagnosticsSheet sheet(this);
	sheet.DoModal();
}

void C智能机械臂Dlg::OnBnClickedExportParams()
{
	CFileDialog dlg(FALSE, L"ini", L"arm-settings.ini",
		OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		L"INI Settings File (*.ini)|*.ini||",
		this);
	if (dlg.DoModal() != IDOK)
	{
		return;
	}

	const CString path = dlg.GetPathName();
	const auto res = SettingsIo::ExportToIni(std::wstring(path.GetString()));
	if (!res.ok)
	{
		CString msg(res.error.c_str());
		AfxMessageBox(msg.IsEmpty() ? L"Export failed." : msg);
		return;
	}
	AfxMessageBox(L"Export succeeded.");
}

void C智能机械臂Dlg::OnBnClickedImportParams()
{
	CFileDialog dlg(TRUE, L"ini", nullptr,
		OFN_HIDEREADONLY | OFN_FILEMUSTEXIST,
		L"INI Settings File (*.ini)|*.ini||",
		this);
	if (dlg.DoModal() != IDOK)
	{
		return;
	}

	const CString path = dlg.GetPathName();
	const auto res = SettingsIo::ImportFromIni(std::wstring(path.GetString()));
	if (!res.ok)
	{
		CString msg(res.error.c_str());
		AfxMessageBox(msg.IsEmpty() ? L"Import failed." : msg);
		return;
	}

	BroadcastSettingsImported();
	AfxMessageBox(L"Import succeeded. Diagnostics pages auto-refreshed.");
}

namespace
{
	struct BroadcastCtx
	{
		UINT msg = 0;
		WPARAM wParam = 0;
		LPARAM lParam = 0;
	};

	BOOL CALLBACK EnumChildProc(HWND hWnd, LPARAM lParam)
	{
		auto* ctx = reinterpret_cast<BroadcastCtx*>(lParam);
		if (!ctx) return FALSE;
		::PostMessageW(hWnd, ctx->msg, ctx->wParam, ctx->lParam);
		::EnumChildWindows(hWnd, EnumChildProc, lParam);
		return TRUE;
	}

	BOOL CALLBACK EnumThreadProc(HWND hWnd, LPARAM lParam)
	{
		auto* ctx = reinterpret_cast<BroadcastCtx*>(lParam);
		if (!ctx) return FALSE;
		::PostMessageW(hWnd, ctx->msg, ctx->wParam, ctx->lParam);
		::EnumChildWindows(hWnd, EnumChildProc, lParam);
		return TRUE;
	}
}

void C智能机械臂Dlg::BroadcastSettingsImported()
{
	BroadcastCtx ctx;
	ctx.msg = WM_APP_SETTINGS_IMPORTED;
	ctx.wParam = 0;
	ctx.lParam = 0;
	::EnumThreadWindows(::GetCurrentThreadId(), EnumThreadProc, reinterpret_cast<LPARAM>(&ctx));
}

void C智能机械臂Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void C智能机械臂Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR C智能机械臂Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

