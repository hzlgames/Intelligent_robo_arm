// CameraDiagPage.cpp : Camera diagnostics property page implementation

#include "pch.h"
#include "Resource.h"
#include "CameraDiagPage.h"
#include "MFCaptureD3D.h"
#include "AppMessages.h"

#include <mfapi.h>
#include <mfidl.h>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "d3d9.lib")

IMPLEMENT_DYNAMIC(CCameraDiagPage, CPropertyPage)

CCameraDiagPage::CCameraDiagPage()
    : CPropertyPage(IDD_CAMERA_DIAG_PAGE, IDS_TAB_CAMERA)
    , m_pPreview(nullptr)
    , m_previewing(false)
    , m_lastFrameCount(0)
    , m_lastTickCount(0)
    , m_estimatedFps(0.0f)
    , m_timerId(0)
    , m_destroying(false)
{
}

CCameraDiagPage::~CCameraDiagPage()
{
    m_destroying = true;

    // Kill timer if still running
    if (m_timerId && GetSafeHwnd())
    {
        KillTimer(m_timerId);
        m_timerId = 0;
    }

    // Stop preview without touching UI
    if (m_pPreview)
    {
        m_pPreview->CloseDevice();
        SafeRelease(&m_pPreview);
    }
    m_previewing = false;

    // Release device activation objects
    for (auto& d : m_devices)
    {
        if (d.pActivate)
        {
            d.pActivate->Release();
            d.pActivate = nullptr;
        }
    }
    m_devices.clear();
}

void CCameraDiagPage::DoDataExchange(CDataExchange* pDX)
{
    CPropertyPage::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_COMBO_CAMERA, m_comboDevices);
    DDX_Control(pDX, IDC_BTN_REFRESH_CAM, m_btnRefresh);
    DDX_Control(pDX, IDC_BTN_START_CAM, m_btnStart);
    DDX_Control(pDX, IDC_BTN_STOP_CAM, m_btnStop);
    DDX_Control(pDX, IDC_BTN_SCREENSHOT, m_btnScreenshot);
    DDX_Control(pDX, IDC_CHECK_MIRROR, m_chkMirror);
    DDX_Control(pDX, IDC_CHECK_CROSSHAIR, m_chkCrosshair);
    DDX_Control(pDX, IDC_CHECK_REFLINES, m_chkRefLines);
    DDX_Control(pDX, IDC_COMBO_ROTATION, m_comboRotation);
    DDX_Control(pDX, IDC_STATIC_VIDEO, m_staticVideo);
    DDX_Control(pDX, IDC_STATIC_CAM_STATUS, m_staticStatus);
    DDX_Control(pDX, IDC_STATIC_CAM_INFO, m_staticInfo);
    DDX_Control(pDX, IDC_EDIT_CAM_LOG, m_editLog);
}

BEGIN_MESSAGE_MAP(CCameraDiagPage, CPropertyPage)
    ON_BN_CLICKED(IDC_BTN_REFRESH_CAM, &CCameraDiagPage::OnBnClickedRefresh)
    ON_BN_CLICKED(IDC_BTN_START_CAM, &CCameraDiagPage::OnBnClickedStart)
    ON_BN_CLICKED(IDC_BTN_STOP_CAM, &CCameraDiagPage::OnBnClickedStop)
    ON_BN_CLICKED(IDC_BTN_SCREENSHOT, &CCameraDiagPage::OnBnClickedScreenshot)
    ON_BN_CLICKED(IDC_BTN_CAM_SHOW_SETTINGS, &CCameraDiagPage::OnBnClickedShowSettings)
    ON_BN_CLICKED(IDC_BTN_CAM_RESET_SETTINGS, &CCameraDiagPage::OnBnClickedResetSettings)
    ON_BN_CLICKED(IDC_CHECK_MIRROR, &CCameraDiagPage::OnBnClickedRefresh)
    ON_BN_CLICKED(IDC_CHECK_CROSSHAIR, &CCameraDiagPage::OnBnClickedRefresh)
    ON_BN_CLICKED(IDC_CHECK_REFLINES, &CCameraDiagPage::OnBnClickedRefresh)
    ON_CBN_SELCHANGE(IDC_COMBO_ROTATION, &CCameraDiagPage::OnBnClickedRefresh)
    ON_WM_TIMER()
    ON_WM_DESTROY()
    ON_WM_SIZE()
    ON_MESSAGE(WM_APP_PREVIEW_ERROR, &CCameraDiagPage::OnPreviewError)
    ON_MESSAGE(WM_APP_SETTINGS_IMPORTED, &CCameraDiagPage::OnSettingsImported)
END_MESSAGE_MAP()

BOOL CCameraDiagPage::OnInitDialog()
{
    CPropertyPage::OnInitDialog();

    RefreshDeviceList();

    // Rotation combo
    m_comboRotation.ResetContent();
    m_comboRotation.AddString(L"0");
    m_comboRotation.AddString(L"90");
    m_comboRotation.AddString(L"180");
    m_comboRotation.AddString(L"270");
    m_comboRotation.SetCurSel(0);

    LoadOverlaySettings();
    ApplyOverlaySettings();
    UpdateStatusText();

    // Start timer for FPS estimation (every 1 second)
    m_timerId = SetTimer(1, 1000, nullptr);
    m_lastTickCount = GetTickCount();

    return TRUE;
}

void CCameraDiagPage::OnDestroy()
{
    m_destroying = true;

    if (m_timerId)
    {
        KillTimer(m_timerId);
        m_timerId = 0;
    }

    // Stop preview without UI updates (already destroying)
    if (m_pPreview)
    {
        m_pPreview->CloseDevice();
        SafeRelease(&m_pPreview);
    }
    m_previewing = false;

    CPropertyPage::OnDestroy();
}

void CCameraDiagPage::RefreshDeviceList()
{
    // Clear old devices
    for (auto& d : m_devices)
    {
        if (d.pActivate)
        {
            d.pActivate->Release();
        }
    }
    m_devices.clear();
    m_comboDevices.ResetContent();

    // Enumerate video capture devices
    IMFAttributes* pAttributes = nullptr;
    IMFActivate** ppDevices = nullptr;
    UINT32 count = 0;

    HRESULT hr = MFCreateAttributes(&pAttributes, 1);
    if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
        );
    }

    if (SUCCEEDED(hr))
    {
        hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
    }

    if (SUCCEEDED(hr))
    {
        for (UINT32 i = 0; i < count; i++)
        {
            WCHAR* szFriendlyName = nullptr;
            UINT32 cchName = 0;

            hr = ppDevices[i]->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                &szFriendlyName,
                &cchName
            );

            if (SUCCEEDED(hr) && szFriendlyName)
            {
                DeviceInfo info;
                info.friendlyName = szFriendlyName;
                info.pActivate = ppDevices[i];
                ppDevices[i]->AddRef(); // Keep a reference

                m_devices.push_back(info);
                m_comboDevices.AddString(szFriendlyName);

                CoTaskMemFree(szFriendlyName);
            }

            ppDevices[i]->Release();
        }
        CoTaskMemFree(ppDevices);
    }

    SafeRelease(&pAttributes);

    if (m_comboDevices.GetCount() > 0)
    {
        m_comboDevices.SetCurSel(0);
    }

    CString logLine;
    logLine.Format(L"[INFO] Found %d camera(s).", (int)m_devices.size());
    AppendLog(logLine);
}

void CCameraDiagPage::UpdateStatusText()
{
    CString status;
    if (!m_previewing)
    {
        status = L"Not previewing";
    }
    else
    {
        if (m_pPreview)
        {
            status.Format(L"Previewing - %.1f FPS", m_estimatedFps);
        }
        else
        {
            status.Format(L"Test source - %.1f FPS", m_estimatedFps);
        }
    }
    m_staticStatus.SetWindowTextW(status);

    // Update info
    if (m_pPreview && m_previewing)
    {
        UINT w = m_pPreview->GetWidth();
        UINT h = m_pPreview->GetHeight();
        CString info;
        info.Format(L"Resolution: %u x %u", w, h);
        m_staticInfo.SetWindowTextW(info);
    }
    else
    {
        m_staticInfo.SetWindowTextW(L"Resolution: N/A");
    }

    // Update button states
    m_btnStart.EnableWindow(!m_previewing && m_comboDevices.GetCount() > 0);
    m_btnStop.EnableWindow(m_previewing);
    m_btnScreenshot.EnableWindow(m_previewing);
}

void CCameraDiagPage::AppendLog(const CString& line)
{
    CString text;
    m_editLog.GetWindowTextW(text);
    if (!text.IsEmpty())
    {
        text += L"\r\n";
    }
    text += line;
    m_editLog.SetWindowTextW(text);
    m_editLog.LineScroll(m_editLog.GetLineCount());
}

void CCameraDiagPage::OnBnClickedRefresh()
{
    RefreshDeviceList();
    ApplyOverlaySettings();
    UpdateStatusText();
}

void CCameraDiagPage::OnBnClickedStart()
{
    StartPreview();
}

void CCameraDiagPage::OnBnClickedStop()
{
    StopPreview();
    UpdateStatusText();
}

void CCameraDiagPage::StartPreview()
{
    if (m_previewing)
    {
        return;
    }

    int sel = m_comboDevices.GetCurSel();
    if (sel < 0 || sel >= (int)m_devices.size())
    {
        AppendLog(L"[ERROR] No camera selected.");
        return;
    }

    // Get the HWND of the video static control
    HWND hVideo = m_staticVideo.GetSafeHwnd();
    HWND hEvent = GetSafeHwnd();

    HRESULT hr = CPreview::CreateInstance(hVideo, hEvent, &m_pPreview);
    if (FAILED(hr))
    {
        CString msg;
        msg.Format(L"[ERROR] CPreview::CreateInstance failed (hr=0x%08X)", hr);
        AppendLog(msg);
        // Fallback to test source
        AppendLog(L"[INFO] Switching to test video source.");
        m_previewing = true;
        m_estimatedFps = 0.0f;
        UpdateStatusText();
        return;
    }

    hr = m_pPreview->SetDevice(m_devices[sel].pActivate);
    if (FAILED(hr))
    {
        CString msg;
        msg.Format(L"[ERROR] SetDevice failed (hr=0x%08X)", hr);
        AppendLog(msg);
        SafeRelease(&m_pPreview);

        // Fallback to test source (no camera/driver issue)
        AppendLog(L"[INFO] Switching to test video source.");
        m_previewing = true;
        m_estimatedFps = 0.0f;
        UpdateStatusText();
        return;
    }

    m_previewing = true;
    m_lastFrameCount = 0;
    m_pPreview->ResetFrameCount();
    m_lastTickCount = GetTickCount();

    ApplyOverlaySettings();

    CString devName;
    m_comboDevices.GetWindowTextW(devName);
    CString logLine;
    logLine.Format(L"[INFO] Started preview: %s", devName.GetString());
    AppendLog(logLine);

    UpdateStatusText();
}

void CCameraDiagPage::LoadOverlaySettings()
{
    CWinApp* app = AfxGetApp();
    const int mirror = app->GetProfileInt(L"CameraOverlay", L"Mirror", 0);
    const int cross = app->GetProfileInt(L"CameraOverlay", L"Crosshair", 0);
    const int grid = app->GetProfileInt(L"CameraOverlay", L"Grid", 0);
    const int rot = app->GetProfileInt(L"CameraOverlay", L"Rotation", 0); // 0/90/180/270

    m_chkMirror.SetCheck(mirror ? BST_CHECKED : BST_UNCHECKED);
    m_chkCrosshair.SetCheck(cross ? BST_CHECKED : BST_UNCHECKED);
    m_chkRefLines.SetCheck(grid ? BST_CHECKED : BST_UNCHECKED);

    int sel = 0;
    if (rot == 90) sel = 1;
    else if (rot == 180) sel = 2;
    else if (rot == 270) sel = 3;
    m_comboRotation.SetCurSel(sel);
}

void CCameraDiagPage::SaveOverlaySettings()
{
    CWinApp* app = AfxGetApp();
    app->WriteProfileInt(L"CameraOverlay", L"Mirror", (m_chkMirror.GetCheck() == BST_CHECKED) ? 1 : 0);
    app->WriteProfileInt(L"CameraOverlay", L"Crosshair", (m_chkCrosshair.GetCheck() == BST_CHECKED) ? 1 : 0);
    app->WriteProfileInt(L"CameraOverlay", L"Grid", (m_chkRefLines.GetCheck() == BST_CHECKED) ? 1 : 0);

    int rot = 0;
    const int sel = m_comboRotation.GetCurSel();
    if (sel == 1) rot = 90;
    else if (sel == 2) rot = 180;
    else if (sel == 3) rot = 270;
    app->WriteProfileInt(L"CameraOverlay", L"Rotation", rot);
}

void CCameraDiagPage::ApplyOverlaySettings()
{
    if (!m_pPreview)
    {
        SaveOverlaySettings();
        return;
    }

    VideoOverlaySettings s;
    s.mirrorHorizontal = (m_chkMirror.GetCheck() == BST_CHECKED);
    s.showCrosshair = (m_chkCrosshair.GetCheck() == BST_CHECKED);
    s.showReferenceLines = (m_chkRefLines.GetCheck() == BST_CHECKED);

    const int sel = m_comboRotation.GetCurSel();
    if (sel == 1) s.rotation = VideoRotation::Rotate90;
    else if (sel == 2) s.rotation = VideoRotation::Rotate180;
    else if (sel == 3) s.rotation = VideoRotation::Rotate270;
    else s.rotation = VideoRotation::None;

    m_pPreview->SetOverlaySettings(s);
    SaveOverlaySettings();
}

void CCameraDiagPage::StopPreview()
{
    if (m_pPreview)
    {
        m_pPreview->CloseDevice();
        SafeRelease(&m_pPreview);
    }
    m_previewing = false;
    m_estimatedFps = 0.0f;

    // Only update UI if not destroying
    if (!m_destroying && GetSafeHwnd())
    {
        AppendLog(L"[INFO] Preview stopped.");
        UpdateStatusText();
    }
}

void CCameraDiagPage::OnBnClickedScreenshot()
{
    if (!m_previewing)
    {
        return;
    }

    // Get client rect of video window
    CRect rc;
    m_staticVideo.GetClientRect(&rc);
    int w = rc.Width();
    int h = rc.Height();

    if (w <= 0 || h <= 0)
    {
        AppendLog(L"[ERROR] Invalid video window size.");
        return;
    }

    // Create a compatible DC and bitmap
    HDC hdcScreen = ::GetDC(m_staticVideo.GetSafeHwnd());
    HDC hdcMem = CreateCompatibleDC(hdcScreen);
    HBITMAP hBitmap = CreateCompatibleBitmap(hdcScreen, w, h);
    HGDIOBJ hOld = SelectObject(hdcMem, hBitmap);

    // If real preview is running, BitBlt from the video window.
    // If we're in test-source mode, draw a simple pattern to the bitmap.
    if (m_pPreview)
    {
        BitBlt(hdcMem, 0, 0, w, h, hdcScreen, 0, 0, SRCCOPY);
    }
    else
    {
        // Fill background
        HBRUSH bg = CreateSolidBrush(RGB(0, 0, 0));
        RECT rr{ 0, 0, w, h };
        FillRect(hdcMem, &rr, bg);
        DeleteObject(bg);

        // Draw moving blocks based on tick
        const DWORD t = GetTickCount();
        const int bx = (int)(t % 200);
        HBRUSH b1 = CreateSolidBrush(RGB(0, 128, 255));
        RECT r1{ bx, 10, bx + 80, 60 };
        FillRect(hdcMem, &r1, b1);
        DeleteObject(b1);

        SetTextColor(hdcMem, RGB(255, 255, 255));
        SetBkMode(hdcMem, TRANSPARENT);
        TextOutW(hdcMem, 10, h - 20, L"TEST SOURCE", 11);
    }

    SelectObject(hdcMem, hOld);
    DeleteDC(hdcMem);
    ::ReleaseDC(m_staticVideo.GetSafeHwnd(), hdcScreen);

    // Save to file
    CFileDialog dlg(FALSE, L"bmp", L"screenshot.bmp",
        OFN_OVERWRITEPROMPT | OFN_HIDEREADONLY,
        L"Bitmap Files (*.bmp)|*.bmp||");

    if (dlg.DoModal() == IDOK)
    {
        CString path = dlg.GetPathName();

        // Get bitmap info
        BITMAP bmp;
        GetObject(hBitmap, sizeof(BITMAP), &bmp);

        BITMAPFILEHEADER bmfHeader;
        BITMAPINFOHEADER bi;

        bi.biSize = sizeof(BITMAPINFOHEADER);
        bi.biWidth = bmp.bmWidth;
        bi.biHeight = bmp.bmHeight;
        bi.biPlanes = 1;
        bi.biBitCount = 24;
        bi.biCompression = BI_RGB;
        bi.biSizeImage = 0;
        bi.biXPelsPerMeter = 0;
        bi.biYPelsPerMeter = 0;
        bi.biClrUsed = 0;
        bi.biClrImportant = 0;

        DWORD dwBmpSize = ((bmp.bmWidth * bi.biBitCount + 31) / 32) * 4 * bmp.bmHeight;

        HANDLE hDIB = GlobalAlloc(GHND, dwBmpSize);
        if (hDIB)
        {
            char* lpbitmap = (char*)GlobalLock(hDIB);

            HDC hdcTemp = ::GetDC(NULL);
            GetDIBits(hdcTemp, hBitmap, 0, (UINT)bmp.bmHeight, lpbitmap,
                (BITMAPINFO*)&bi, DIB_RGB_COLORS);
            ::ReleaseDC(NULL, hdcTemp);

            DWORD dwSizeofDIB = dwBmpSize + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
            bmfHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
            bmfHeader.bfSize = dwSizeofDIB;
            bmfHeader.bfType = 0x4D42; // "BM"

            CFile file;
            if (file.Open(path, CFile::modeCreate | CFile::modeWrite))
            {
                file.Write(&bmfHeader, sizeof(BITMAPFILEHEADER));
                file.Write(&bi, sizeof(BITMAPINFOHEADER));
                file.Write(lpbitmap, dwBmpSize);
                file.Close();

                CString msg;
                msg.Format(L"[INFO] Screenshot saved: %s", path.GetString());
                AppendLog(msg);
            }
            else
            {
                AppendLog(L"[ERROR] Failed to save screenshot.");
            }

            GlobalUnlock(hDIB);
            GlobalFree(hDIB);
        }
    }

    DeleteObject(hBitmap);
}

void CCameraDiagPage::OnBnClickedShowSettings()
{
	CWinApp* app = AfxGetApp();
	const int mirror = app->GetProfileInt(L"CameraOverlay", L"Mirror", 0);
	const int cross = app->GetProfileInt(L"CameraOverlay", L"Crosshair", 0);
	const int grid = app->GetProfileInt(L"CameraOverlay", L"Grid", 0);
	const int rot = app->GetProfileInt(L"CameraOverlay", L"Rotation", 0);

	CString line;
	line.Format(L"[SET] CameraOverlay: Mirror=%d Crosshair=%d Grid=%d Rotation=%d", mirror, cross, grid, rot);
	AppendLog(line);
}

void CCameraDiagPage::OnBnClickedResetSettings()
{
	// Restore defaults and persist
	CWinApp* app = AfxGetApp();
	app->WriteProfileInt(L"CameraOverlay", L"Mirror", 0);
	app->WriteProfileInt(L"CameraOverlay", L"Crosshair", 0);
	app->WriteProfileInt(L"CameraOverlay", L"Grid", 0);
	app->WriteProfileInt(L"CameraOverlay", L"Rotation", 0);

	// Update UI and apply
	m_chkMirror.SetCheck(BST_UNCHECKED);
	m_chkCrosshair.SetCheck(BST_UNCHECKED);
	m_chkRefLines.SetCheck(BST_UNCHECKED);
	m_comboRotation.SetCurSel(0);
	ApplyOverlaySettings();

	AppendLog(L"[INFO] CameraOverlay reset to defaults (Mirror/Crosshair/Grid=0, Rotation=0).");
}

void CCameraDiagPage::OnTimer(UINT_PTR nIDEvent)
{
    // Ignore timer events if destroying
    if (m_destroying)
    {
        return;
    }

    if (nIDEvent == 1 && m_previewing)
    {
        if (m_pPreview)
        {
            DWORD now = GetTickCount();
            DWORD elapsed = now - m_lastTickCount;
            if (elapsed > 0)
            {
                UINT currentFrameCount = m_pPreview->GetFrameCount();
                UINT framesDelta = currentFrameCount - m_lastFrameCount;
                m_estimatedFps = (float)framesDelta * 1000.0f / (float)elapsed;
                m_lastFrameCount = currentFrameCount;
                m_lastTickCount = now;
            }
        }
        else
        {
            // Test source: just animate a fake FPS value
            m_estimatedFps = 30.0f;
        }
        UpdateStatusText();
    }

    CPropertyPage::OnTimer(nIDEvent);
}

LRESULT CCameraDiagPage::OnPreviewError(WPARAM wParam, LPARAM /*lParam*/)
{
    // Ignore if destroying
    if (m_destroying)
    {
        return 0;
    }

    HRESULT hr = (HRESULT)wParam;
    CString msg;
    msg.Format(L"[ERROR] Preview error (hr=0x%08X)", hr);
    AppendLog(msg);

    StopPreview();
    return 0;
}

LRESULT CCameraDiagPage::OnSettingsImported(WPARAM /*wParam*/, LPARAM /*lParam*/)
{
	// Refresh overlay UI from profile, then apply to preview (if running).
	if (m_destroying || !GetSafeHwnd())
	{
		return 0;
	}

	LoadOverlaySettings();
	ApplyOverlaySettings();
	AppendLog(L"[INFO] Settings imported: CameraOverlay reloaded.");
	return 0;
}

void CCameraDiagPage::OnSize(UINT nType, int cx, int cy)
{
    CPropertyPage::OnSize(nType, cx, cy);

    if (m_pPreview && m_staticVideo.GetSafeHwnd())
    {
        CRect rc;
        m_staticVideo.GetClientRect(&rc);
        m_pPreview->ResizeVideo((WORD)rc.Width(), (WORD)rc.Height());
    }
}

