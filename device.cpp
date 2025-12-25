// device.cpp : Manages the Direct3D device (adapted from MFCaptureD3D sample)

#include "pch.h"
#include "MFCaptureD3D.h"
#include "BufferLock.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

// OpenCV（用于在 RGB32 buffer 上叠加 HUD；若未安装 OpenCV，则自动退化为无HUD）
#if defined(__has_include)
#if __has_include(<opencv2/core.hpp>) && __has_include(<opencv2/imgproc.hpp>)
#define SMARTARM_HAS_OPENCV 1
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif
#endif

#include "KinematicsOverlayService.h"

const DWORD NUM_BACK_BUFFERS = 2;

void TransformImage_RGB24(
    BYTE*       pDest,
    LONG        lDestStride,
    const BYTE* pSrc,
    LONG        lSrcStride,
    DWORD       dwWidthInPixels,
    DWORD       dwHeightInPixels
    );

void TransformImage_RGB32(
    BYTE*       pDest,
    LONG        lDestStride,
    const BYTE* pSrc,
    LONG        lSrcStride,
    DWORD       dwWidthInPixels,
    DWORD       dwHeightInPixels
    );

void TransformImage_YUY2(
    BYTE*       pDest,
    LONG        lDestStride,
    const BYTE* pSrc,
    LONG        lSrcStride,
    DWORD       dwWidthInPixels,
    DWORD       dwHeightInPixels
    );

void TransformImage_NV12(
    BYTE* pDst, 
    LONG dstStride, 
    const BYTE* pSrc, 
    LONG srcStride,
    DWORD dwWidthInPixels,
    DWORD dwHeightInPixels
    );


RECT    LetterBoxRect(const RECT& rcSrc, const RECT& rcDst);
RECT    CorrectAspectRatio(const RECT& src, const MFRatio& srcPAR);
HRESULT GetDefaultStride(IMFMediaType *pType, LONG *plStride);


inline LONG Width(const RECT& r)
{
    return r.right - r.left;
}

inline LONG Height(const RECT& r)
{
    return r.bottom - r.top;
}


// Static table of output formats and conversion functions.
struct ConversionFunction
{
    GUID               subtype;
    IMAGE_TRANSFORM_FN xform;
};


ConversionFunction   g_FormatConversions[] =
{
    { MFVideoFormat_RGB32, TransformImage_RGB32 },
    { MFVideoFormat_RGB24, TransformImage_RGB24 },
    { MFVideoFormat_YUY2,  TransformImage_YUY2  },      
    { MFVideoFormat_NV12,  TransformImage_NV12  }
};

const DWORD   g_cFormats = ARRAYSIZE(g_FormatConversions);


//-------------------------------------------------------------------
// Constructor
//-------------------------------------------------------------------

DrawDevice::DrawDevice() : 
    m_hwnd(NULL),
    m_pD3D(NULL),
    m_pDevice(NULL),
    m_pSwapChain(NULL),
    m_subtype(GUID_NULL),
    m_format(D3DFMT_UNKNOWN),
    m_width(0),
    m_height(0),
    m_lDefaultStride(0),
    m_interlace(MFVideoInterlace_Unknown),
    m_convertFn(NULL)
{
    m_PixelAR.Denominator = m_PixelAR.Numerator = 1; 

    ZeroMemory(&m_d3dpp, sizeof(m_d3dpp));
}


//-------------------------------------------------------------------
// Destructor
//-------------------------------------------------------------------

DrawDevice::~DrawDevice()
{
    DestroyDevice();
}


//-------------------------------------------------------------------
// GetFormat
//
// Get a supported output format by index.
//-------------------------------------------------------------------

HRESULT DrawDevice::GetFormat(DWORD index, GUID *pSubtype) const
{
    if (index < g_cFormats)
    {
        *pSubtype = g_FormatConversions[index].subtype;
        return S_OK;
    }
    return MF_E_NO_MORE_TYPES;
}


//-------------------------------------------------------------------
//  IsFormatSupported
//
//  Query if a format is supported.
//-------------------------------------------------------------------

BOOL DrawDevice::IsFormatSupported(REFGUID subtype) const
{
    for (DWORD i = 0; i < g_cFormats; i++)
    {
        if (subtype == g_FormatConversions[i].subtype)
        {
            return TRUE;
        }
    }
    return FALSE;
}




//-------------------------------------------------------------------
// CreateDevice
//
// Create the Direct3D device.
//-------------------------------------------------------------------

HRESULT DrawDevice::CreateDevice(HWND hwnd)
{
    if (m_pDevice)
    {
        return S_OK;
    }

    // Create the Direct3D object.
    if (m_pD3D == NULL)
    {
        m_pD3D = Direct3DCreate9(D3D_SDK_VERSION);

        if (m_pD3D == NULL)
        {
            return E_FAIL;
        }
    }


    HRESULT hr = S_OK;
    D3DPRESENT_PARAMETERS pp = { 0 };
    D3DDISPLAYMODE mode = { 0 };
    RECT rcClient = { 0, 0, 0, 0 };

    hr = m_pD3D->GetAdapterDisplayMode(
        D3DADAPTER_DEFAULT, 
        &mode
        );

    if (FAILED(hr)) { goto done; }

    hr = m_pD3D->CheckDeviceType(
        D3DADAPTER_DEFAULT,
        D3DDEVTYPE_HAL,
        mode.Format,
        D3DFMT_X8R8G8B8,
        TRUE    // windowed
        );

    if (FAILED(hr)) { goto done; }

    pp.BackBufferFormat = D3DFMT_X8R8G8B8;
    pp.SwapEffect = D3DSWAPEFFECT_COPY;
    pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;  
    pp.Windowed = TRUE;
    pp.hDeviceWindow = hwnd;

    // Important: In windowed mode, resizing the video window requires Reset with a valid backbuffer size.
    // Some drivers behave poorly when BackBufferWidth/Height stay at 0 during WM_SIZE (can cause stale/garbage).
    // Use the current client size as the initial backbuffer size.
    if (GetClientRect(hwnd, &rcClient))
    {
        const UINT w = (UINT)std::max<LONG>(1, rcClient.right - rcClient.left);
        const UINT h = (UINT)std::max<LONG>(1, rcClient.bottom - rcClient.top);
        pp.BackBufferWidth = w;
        pp.BackBufferHeight = h;
    }

    hr = m_pD3D->CreateDevice(
        D3DADAPTER_DEFAULT,
        D3DDEVTYPE_HAL,
        hwnd,
        D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE,
        &pp,
        &m_pDevice
        );

    if (FAILED(hr)) { goto done; }

    m_hwnd = hwnd;
    m_d3dpp = pp;

done:
    return hr;
}

//-------------------------------------------------------------------
// SetConversionFunction
//
// Set the conversion function for the specified video format.
//-------------------------------------------------------------------

HRESULT DrawDevice::SetConversionFunction(REFGUID subtype)
{
    m_convertFn = NULL;

    for (DWORD i = 0; i < g_cFormats; i++)
    {
        if (g_FormatConversions[i].subtype == subtype)
        {
            m_convertFn = g_FormatConversions[i].xform;
            return S_OK;
        }
    }

    return MF_E_INVALIDMEDIATYPE;
}


//-------------------------------------------------------------------
// SetVideoType
//
// Set the video format.  
//-------------------------------------------------------------------

HRESULT DrawDevice::SetVideoType(IMFMediaType *pType)
{
    HRESULT hr = S_OK;
    GUID subtype = { 0 };
    MFRatio PAR = { 0 };

    // Find the video subtype.
    hr = pType->GetGUID(MF_MT_SUBTYPE, &subtype);

    if (FAILED(hr)) { goto done; }

    // Choose a conversion function.
    // (This also validates the format type.)

    hr = SetConversionFunction(subtype); 
    
    if (FAILED(hr)) { goto done; }

    //
    // Get some video attributes.
    //

    // Get the frame size.
    hr = MFGetAttributeSize(pType, MF_MT_FRAME_SIZE, &m_width, &m_height);
    
    if (FAILED(hr)) { goto done; }

    
    // Get the interlace mode. Default: assume progressive.
    m_interlace = (MFVideoInterlaceMode)MFGetAttributeUINT32(
        pType,
        MF_MT_INTERLACE_MODE, 
        MFVideoInterlace_Progressive
        );

    // Get the image stride.
    hr = GetDefaultStride(pType, &m_lDefaultStride);

    if (FAILED(hr)) { goto done; }

    // Get the pixel aspect ratio. Default: Assume square pixels (1:1)
    hr = MFGetAttributeRatio(
        pType, 
        MF_MT_PIXEL_ASPECT_RATIO, 
        (UINT32*)&PAR.Numerator, 
        (UINT32*)&PAR.Denominator
        );

    if (SUCCEEDED(hr))
    {
        m_PixelAR = PAR;
    }
    else
    {
        m_PixelAR.Numerator = m_PixelAR.Denominator = 1;
    }

    m_subtype = subtype;
    m_format = (D3DFORMAT)subtype.Data1;
    m_lastRgb.clear();
    m_lastRgb.resize(static_cast<size_t>(m_width) * static_cast<size_t>(m_height) * 4);

    // Create Direct3D swap chains.

    hr = CreateSwapChains();

    if (FAILED(hr)) { goto done; }


    // Update the destination rectangle for the correct
    // aspect ratio.

    UpdateDestinationRect();

done:
    if (FAILED(hr))
    {
        m_format = D3DFMT_UNKNOWN;
        m_convertFn = NULL;
    }
    return hr;
}

//-------------------------------------------------------------------
//  UpdateDestinationRect
//
//  Update the destination rectangle for the current window size.
//  The destination rectangle is letterboxed to preserve the 
//  aspect ratio of the video image.
//-------------------------------------------------------------------

void DrawDevice::UpdateDestinationRect()
{
    RECT rcClient;
    RECT rcSrc = { 0, 0, (LONG)m_width, (LONG)m_height };

    GetClientRect(m_hwnd, &rcClient);

    rcSrc = CorrectAspectRatio(rcSrc, m_PixelAR);

    m_rcDest = LetterBoxRect(rcSrc, rcClient);
}


//-------------------------------------------------------------------
// CreateSwapChains
//
// Create Direct3D swap chains.
//-------------------------------------------------------------------

HRESULT DrawDevice::CreateSwapChains()
{
    HRESULT hr = S_OK;

    D3DPRESENT_PARAMETERS pp = { 0 };

    SafeRelease(&m_pSwapChain);

    pp.BackBufferWidth  = m_width;
    pp.BackBufferHeight = m_height;
    pp.Windowed = TRUE;
    pp.SwapEffect = D3DSWAPEFFECT_FLIP;
    pp.hDeviceWindow = m_hwnd;
    pp.BackBufferFormat = D3DFMT_X8R8G8B8;
    pp.Flags = 
        D3DPRESENTFLAG_VIDEO | D3DPRESENTFLAG_DEVICECLIP |
        D3DPRESENTFLAG_LOCKABLE_BACKBUFFER;
    pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
    pp.BackBufferCount = NUM_BACK_BUFFERS;

    hr = m_pDevice->CreateAdditionalSwapChain(&pp, &m_pSwapChain);

    return hr;
}


//-------------------------------------------------------------------
// DrawFrame
//
// Draw the video frame.
//-------------------------------------------------------------------

HRESULT DrawDevice::DrawFrame(IMFMediaBuffer *pBuffer)
{
    if (m_convertFn == NULL)
    {
        return MF_E_INVALIDREQUEST;
    }

    HRESULT hr = S_OK;
    BYTE *pbScanline0 = NULL;
    LONG lStride = 0;
    D3DLOCKED_RECT lr;

    IDirect3DSurface9 *pSurf = NULL;
    IDirect3DSurface9 *pBB = NULL;

    if (m_pDevice == NULL || m_pSwapChain == NULL)
    {
        return S_OK;
    }

    VideoBufferLock buffer(pBuffer);    // Helper object to lock the video buffer.

    hr = TestCooperativeLevel();

    if (FAILED(hr)) { goto done; }

    // Lock the video buffer. This method returns a pointer to the first scan
    // line in the image, and the stride in bytes.

    hr = buffer.LockBuffer(m_lDefaultStride, m_height, &pbScanline0, &lStride);

    if (FAILED(hr)) { goto done; }

    // Get the swap-chain surface.
    hr = m_pSwapChain->GetBackBuffer(0, D3DBACKBUFFER_TYPE_MONO, &pSurf);

    if (FAILED(hr)) { goto done; }

    // Lock the swap-chain surface.
    hr = pSurf->LockRect(&lr, NULL, D3DLOCK_NOSYSLOCK );

    if (FAILED(hr)) { goto done; }


    // Convert the frame. This also copies it to the Direct3D surface.
    
    m_convertFn(
        (BYTE*)lr.pBits,
        lr.Pitch,
        pbScanline0,
        lStride,
        m_width,
        m_height
        );

    // 重要：缓存“最新一帧 RGB32”供视觉线程读取（CPreview::CopyLastRgb）。
    // 之前的实现只在开启镜像/旋转时更新 m_lastRgb，导致默认 Rot=0 且未镜像时
    // VisionService 永远拿不到帧（CopyLastRgb 返回 false），从而看不到目标点/误差箭头。
    // 这里先缓存“变换前”的原始帧；若后续应用了镜像/旋转，会在后面的变换分支里覆盖为“变换后”帧。
    if (!m_lastRgb.empty())
    {
        const UINT w = m_width;
        const UINT h = m_height;
        for (UINT y = 0; y < h; y++)
        {
            const BYTE* row = (const BYTE*)lr.pBits + static_cast<size_t>(y) * static_cast<size_t>(lr.Pitch);
            memcpy(m_lastRgb.data() + (static_cast<size_t>(y) * static_cast<size_t>(w) * 4),
                   row,
                   static_cast<size_t>(w) * 4);
        }
    }

    hr = pSurf->UnlockRect();

    if (FAILED(hr)) { goto done; }


    // Color fill the back buffer.
    hr = m_pDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &pBB);

    if (FAILED(hr)) { goto done; }

    hr = m_pDevice->ColorFill(pBB, NULL, D3DCOLOR_XRGB(0, 0, 0x80));

    if (FAILED(hr)) { goto done; }


    // Apply orientation transforms (mirror/rotate) on the swap chain surface.
    if (m_overlay.mirrorHorizontal || m_overlay.rotation != VideoRotation::None)
    {
        D3DLOCKED_RECT lrTx;
        hr = pSurf->LockRect(&lrTx, NULL, 0);
        if (SUCCEEDED(hr))
        {
            const UINT w = m_width;
            const UINT h = m_height;

            // Copy surface to a tight src buffer (DWORD pixels).
            std::vector<DWORD> src;
            src.resize(static_cast<size_t>(w) * static_cast<size_t>(h));
            for (UINT y = 0; y < h; y++)
            {
                const BYTE* row = (const BYTE*)lrTx.pBits + static_cast<size_t>(y) * lrTx.Pitch;
                memcpy(&src[static_cast<size_t>(y) * w], row, static_cast<size_t>(w) * 4);
            }

            std::vector<DWORD> dst;
            dst.resize(static_cast<size_t>(w) * static_cast<size_t>(h), 0);

            const VideoRotation rot = m_overlay.rotation;
            const UINT rotW = (rot == VideoRotation::Rotate90 || rot == VideoRotation::Rotate270) ? h : w;
            const UINT rotH = (rot == VideoRotation::Rotate90 || rot == VideoRotation::Rotate270) ? w : h;

            const float scaleX = (rotW > 0) ? (float)w / (float)rotW : 1.0f;
            const float scaleY = (rotH > 0) ? (float)h / (float)rotH : 1.0f;
            const float scale = (scaleX < scaleY) ? scaleX : scaleY;

            const int outW = (int)(rotW * scale + 0.5f);
            const int outH = (int)(rotH * scale + 0.5f);
            const int offX = (int)((int)w - outW) / 2;
            const int offY = (int)((int)h - outH) / 2;

            for (UINT y = 0; y < h; y++)
            {
                for (UINT x = 0; x < w; x++)
                {
                    if ((int)x < offX || (int)x >= offX + outW || (int)y < offY || (int)y >= offY + outH)
                    {
                        dst[static_cast<size_t>(y) * w + x] = 0; // black
                        continue;
                    }

                    // Map destination pixel to rotated-space coordinate (nearest neighbor).
                    const float u = ((float)((int)x - offX)) / scale;
                    const float v = ((float)((int)y - offY)) / scale;
                    int ox = (int)(u + 0.5f);
                    int oy = (int)(v + 0.5f);

                    if (ox < 0) ox = 0;
                    if (oy < 0) oy = 0;
                    if (ox >= (int)rotW) ox = (int)rotW - 1;
                    if (oy >= (int)rotH) oy = (int)rotH - 1;

                    // Mirror in rotated-space (horizontal flip).
                    if (m_overlay.mirrorHorizontal)
                    {
                        ox = (int)rotW - 1 - ox;
                    }

                    int sx = 0, sy = 0;
                    switch (rot)
                    {
                    case VideoRotation::None:
                        sx = ox; sy = oy;
                        break;
                    case VideoRotation::Rotate180:
                        sx = (int)w - 1 - ox;
                        sy = (int)h - 1 - oy;
                        break;
                    case VideoRotation::Rotate90:
                        // out width = h, out height = w
                        sx = oy;
                        sy = (int)h - 1 - ox;
                        break;
                    case VideoRotation::Rotate270:
                        sx = (int)w - 1 - oy;
                        sy = ox;
                        break;
                    default:
                        sx = ox; sy = oy;
                        break;
                    }

                    if (sx < 0) sx = 0;
                    if (sy < 0) sy = 0;
                    if (sx >= (int)w) sx = (int)w - 1;
                    if (sy >= (int)h) sy = (int)h - 1;

                    dst[static_cast<size_t>(y) * w + x] = src[static_cast<size_t>(sy) * w + (UINT)sx];
                }
            }

            // Write dst back to surface
            for (UINT y = 0; y < h; y++)
            {
                BYTE* row = (BYTE*)lrTx.pBits + static_cast<size_t>(y) * lrTx.Pitch;
                memcpy(row, &dst[static_cast<size_t>(y) * w], static_cast<size_t>(w) * 4);
            }

            // Update last RGB copy (tight, RGB32)
            if (!m_lastRgb.empty())
            {
                for (UINT y = 0; y < h; y++)
                {
                    memcpy(m_lastRgb.data() + (static_cast<size_t>(y) * w * 4),
                        &dst[static_cast<size_t>(y) * w],
                        static_cast<size_t>(w) * 4);
                }
            }

            pSurf->UnlockRect();
        }
    }

    // Draw overlays (crosshair, reference lines) directly onto the swap-chain surface.
    // NOTE: Drawing on the device backbuffer via GetDC is unreliable; this surface is lockable.
    DrawOverlays(pSurf);

    // Blit the frame.

    hr = m_pDevice->StretchRect(pSurf, NULL, pBB, &m_rcDest, D3DTEXF_LINEAR);
    
    if (FAILED(hr)) { goto done; }

    // Present the frame.
    
    hr = m_pDevice->Present(NULL, NULL, NULL, NULL);
    

done:
    SafeRelease(&pBB);
    SafeRelease(&pSurf);
    return hr;
}

//-------------------------------------------------------------------
// TestCooperativeLevel
//
// Test the cooperative-level status of the Direct3D device.
//-------------------------------------------------------------------

HRESULT DrawDevice::TestCooperativeLevel()
{
    if (m_pDevice == NULL)
    {
        return E_FAIL;
    }

    HRESULT hr = S_OK;

    // Check the current status of D3D9 device.
    hr = m_pDevice->TestCooperativeLevel();

    switch (hr)
    {
    case D3D_OK:
        break;

    case D3DERR_DEVICELOST:
        hr = S_OK;
        break;

    case D3DERR_DEVICENOTRESET:
        hr = ResetDevice();
        break;

    default:
        // Some other failure.
        break;
    }

    return hr;
}


//-------------------------------------------------------------------
// ResetDevice
//
// Resets the Direct3D device.
//-------------------------------------------------------------------

HRESULT DrawDevice::ResetDevice()
{
    HRESULT hr = S_OK;

    if (m_pDevice)
    {
        D3DPRESENT_PARAMETERS d3dpp = m_d3dpp;

        // Keep backbuffer size in sync with the current video window client area.
        // This prevents artifacts when the user resizes the main window (child video hwnd changes size).
        RECT rcClient = {};
        if (m_hwnd && GetClientRect(m_hwnd, &rcClient))
        {
            const UINT w = (UINT)std::max<LONG>(1, rcClient.right - rcClient.left);
            const UINT h = (UINT)std::max<LONG>(1, rcClient.bottom - rcClient.top);
            d3dpp.BackBufferWidth = w;
            d3dpp.BackBufferHeight = h;
        }
        d3dpp.hDeviceWindow = m_hwnd;

        hr = m_pDevice->Reset(&d3dpp);

        if (FAILED(hr))
        {
            DestroyDevice();
        }
        else
        {
            // Persist the last good present params.
            m_d3dpp = d3dpp;
        }
    }

    if (m_pDevice == NULL)
    {
        hr = CreateDevice(m_hwnd);

        if (FAILED(hr)) { goto done; }
    }

    if ((m_format != D3DFMT_UNKNOWN) && m_pDevice)
    {
        hr = CreateSwapChains();
        if (FAILED(hr)) { goto done; }

        UpdateDestinationRect();

        // Clear once after reset to avoid showing garbage before the next sample arrives.
        m_pDevice->Clear(0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 0), 1.0f, 0);
        m_pDevice->Present(NULL, NULL, NULL, NULL);
    }


done:
    return hr;
}


//-------------------------------------------------------------------
// DestroyDevice
//
// Release all Direct3D resources.
//-------------------------------------------------------------------

void DrawDevice::DestroyDevice()
{
    SafeRelease(&m_pSwapChain);
    SafeRelease(&m_pDevice);
    SafeRelease(&m_pD3D);
}

bool DrawDevice::CopyLastRgb(std::vector<BYTE>& outRgb, UINT& outW, UINT& outH) const
{
    if (m_width == 0 || m_height == 0 || m_lastRgb.empty())
    {
        return false;
    }
    outW = m_width;
    outH = m_height;
    outRgb = m_lastRgb;
    return true;
}



//-------------------------------------------------------------------
// Static functions
//-------------------------------------------------------------------


//-------------------------------------------------------------------
// LetterBoxRect
//
// Given a source rectangle (rcSrc) and a destination rectangle (rcDst),
// returns a letterboxed rectangle within the destination.
//-------------------------------------------------------------------

RECT LetterBoxRect(const RECT &rcSrc, const RECT &rcDst)
{
    int iSrcWidth  = Width(rcSrc);
    int iSrcHeight = Height(rcSrc);

    int iDstWidth  = Width(rcDst);
    int iDstHeight = Height(rcDst);

    int iDstLBWidth;
    int iDstLBHeight;

    if (MulDiv(iSrcWidth, iDstHeight, iSrcHeight) <= iDstWidth) 
    {
        // Column letterboxing ("pillar box")

        iDstLBWidth  = MulDiv(iDstHeight, iSrcWidth, iSrcHeight);
        iDstLBHeight = iDstHeight;
    }
    else 
    {
        // Row letterboxing.

        iDstLBWidth  = iDstWidth;
        iDstLBHeight = MulDiv(iDstWidth, iSrcHeight, iSrcWidth);
    }


    // Create a centered rectangle within the current destination rect

    LONG left = rcDst.left + ((iDstWidth - iDstLBWidth) / 2);
    LONG top = rcDst.top + ((iDstHeight - iDstLBHeight) / 2);

    RECT rc;
    SetRect(&rc, left, top, left + iDstLBWidth, top + iDstLBHeight);

    return rc;
}



//-----------------------------------------------------------------------------
// CorrectAspectRatio
//
// Converts a rectangle from the source's pixel aspect ratio (PAR) to 1:1 PAR.
// Returns the corrected rectangle.
//
// For example, a 720 x 486 rect with a 1:9 : 1 PAR becomes 720 x 540. 
//-----------------------------------------------------------------------------

RECT CorrectAspectRatio(const RECT& src, const MFRatio& srcPAR)
{
    // Start with a rectangle the same size as src, but offset to the origin (0,0).
    RECT rc = {0, 0, src.right - src.left, src.bottom - src.top};

    if ((srcPAR.Numerator != 1) || (srcPAR.Denominator != 1))
    {
        // Discarding PAR.Numerator vs Denominator ratio
        if (srcPAR.Numerator > srcPAR.Denominator)
        {
            // The source has "wide" pixels, so stretch the width.
            rc.right = MulDiv(rc.right, srcPAR.Numerator, srcPAR.Denominator);
        }
        else if (srcPAR.Numerator < srcPAR.Denominator)
        {
            // The source has "tall" pixels, so stretch the height.
            rc.bottom = MulDiv(rc.bottom, srcPAR.Denominator, srcPAR.Numerator);
        }
        // else: Source is 1:1 PAR. No action.
    }

    return rc;
}


//-----------------------------------------------------------------------------
// GetDefaultStride
//
// Gets the default stride for a video frame, assuming no extra padding bytes.
//
//-----------------------------------------------------------------------------

HRESULT GetDefaultStride(IMFMediaType *pType, LONG *plStride)
{
    LONG lStride = 0;

    // Try to get the default stride from the media type.
    HRESULT hr = pType->GetUINT32(MF_MT_DEFAULT_STRIDE, (UINT32*)&lStride);
    if (FAILED(hr))
    {
        // Attribute not set. Try to calculate the default stride.
        GUID subtype = GUID_NULL;

        UINT32 width = 0;
        UINT32 height = 0;

        // Get the subtype and the image size.
        hr = pType->GetGUID(MF_MT_SUBTYPE, &subtype);
        if (SUCCEEDED(hr))
        {
            hr = MFGetAttributeSize(pType, MF_MT_FRAME_SIZE, &width, &height);
        }
        if (SUCCEEDED(hr))
        {
            hr = MFGetStrideForBitmapInfoHeader(subtype.Data1, width, &lStride);
        }

        // Set the attribute for later reference.
        if (SUCCEEDED(hr))
        {
            (void)pType->SetUINT32(MF_MT_DEFAULT_STRIDE, UINT32(lStride));
        }
    }
    if (SUCCEEDED(hr))
    {
        *plStride = lStride;
    }
    return hr;
}


//-----------------------------------------------------------------------------
// Image transformation functions
//
// These functions convert YUV or RGB images to RGB32 format.
//-----------------------------------------------------------------------------

__forceinline BYTE Clip(int clr)
{
    return (BYTE)(clr < 0 ? 0 : ( clr > 255 ? 255 : clr ));
}

__forceinline RGBQUAD ConvertYCrCbToRGB(
    int y,
    int cr,
    int cb
    )
{
    RGBQUAD rgbq;

    int c = y - 16;
    int d = cb - 128;
    int e = cr - 128;

    rgbq.rgbRed =   Clip(( 298 * c           + 409 * e + 128) >> 8);
    rgbq.rgbGreen = Clip(( 298 * c - 100 * d - 208 * e + 128) >> 8);
    rgbq.rgbBlue =  Clip(( 298 * c + 516 * d           + 128) >> 8);

    return rgbq;
}


//-------------------------------------------------------------------
// TransformImage_RGB24 
//
// RGB-24 to RGB-32
//-------------------------------------------------------------------

void TransformImage_RGB24(
    BYTE*       pDest,
    LONG        lDestStride,
    const BYTE* pSrc,
    LONG        lSrcStride,
    DWORD       dwWidthInPixels,
    DWORD       dwHeightInPixels
    )
{
    for (DWORD y = 0; y < dwHeightInPixels; y++)
    {
        RGBTRIPLE *pSrcPel = (RGBTRIPLE*)pSrc;
        DWORD *pDestPel = (DWORD*)pDest;

        for (DWORD x = 0; x < dwWidthInPixels; x++)
        {
            pDestPel[x] = D3DCOLOR_XRGB(
                pSrcPel[x].rgbtRed,
                pSrcPel[x].rgbtGreen,
                pSrcPel[x].rgbtBlue
                );
        }

        pSrc += lSrcStride;
        pDest += lDestStride;
    }
}

//-------------------------------------------------------------------
// TransformImage_RGB32
//
// RGB-32 to RGB-32 
//
// Note: This function is needed to copy the image from system
// memory to the Direct3D surface.
//-------------------------------------------------------------------

void TransformImage_RGB32(
    BYTE*       pDest,
    LONG        lDestStride,
    const BYTE* pSrc,
    LONG        lSrcStride,
    DWORD       dwWidthInPixels,
    DWORD       dwHeightInPixels
    )
{
    MFCopyImage(pDest, lDestStride, pSrc, lSrcStride, dwWidthInPixels * 4, dwHeightInPixels);
}

//-------------------------------------------------------------------
// TransformImage_YUY2 
//
// YUY2 to RGB-32
//-------------------------------------------------------------------

void TransformImage_YUY2(
    BYTE*       pDest,
    LONG        lDestStride,
    const BYTE* pSrc,
    LONG        lSrcStride,
    DWORD       dwWidthInPixels,
    DWORD       dwHeightInPixels
    )
{
    for (DWORD y = 0; y < dwHeightInPixels; y++)
    {
        RGBQUAD *pDestPel = (RGBQUAD*)pDest;

        for (DWORD x = 0; x < dwWidthInPixels; x += 2)
        {
            // Byte order is U0 Y0 V0 Y1
            int y0 = (int)pSrc[1];
            int u0 = (int)pSrc[0];
            int y1 = (int)pSrc[3];
            int v0 = (int)pSrc[2];

            pDestPel[x] = ConvertYCrCbToRGB(y0, v0, u0);
            pDestPel[x + 1] = ConvertYCrCbToRGB(y1, v0, u0);

            pSrc += 4;
        }

        pSrc += lSrcStride - (dwWidthInPixels * 2);
        pDest += lDestStride;
    }

}

//-------------------------------------------------------------------
// TransformImage_NV12
//
// NV12 to RGB-32
//-------------------------------------------------------------------

void TransformImage_NV12(
    BYTE* pDst, 
    LONG dstStride, 
    const BYTE* pSrc, 
    LONG srcStride,
    DWORD dwWidthInPixels,
    DWORD dwHeightInPixels
    )
{
    const BYTE* lpBitsY = pSrc;
    const BYTE* lpBitsCb = lpBitsY  + (dwHeightInPixels * srcStride);
    const BYTE* lpBitsCr = lpBitsCb + 1;

    for (DWORD y = 0; y < dwHeightInPixels; y += 2)
    {
        const BYTE* lpLineY1 = lpBitsY;
        const BYTE* lpLineY2 = lpBitsY + srcStride;
        const BYTE* lpLineCr = lpBitsCr;
        const BYTE* lpLineCb = lpBitsCb;

        LPBYTE lpDibLine1 = pDst;
        LPBYTE lpDibLine2 = pDst + dstStride;

        for (DWORD x = 0; x < dwWidthInPixels; x += 2)
        {
            int  y0 = (int)lpLineY1[0];
            int  y1 = (int)lpLineY1[1];
            int  y2 = (int)lpLineY2[0];
            int  y3 = (int)lpLineY2[1];
            int  cb = (int)lpLineCb[0];
            int  cr = (int)lpLineCr[0];

            RGBQUAD r = ConvertYCrCbToRGB(y0, cr, cb);
            lpDibLine1[0] = r.rgbBlue;
            lpDibLine1[1] = r.rgbGreen;
            lpDibLine1[2] = r.rgbRed;
            lpDibLine1[3] = 0;

            r = ConvertYCrCbToRGB(y1, cr, cb);
            lpDibLine1[4] = r.rgbBlue;
            lpDibLine1[5] = r.rgbGreen;
            lpDibLine1[6] = r.rgbRed;
            lpDibLine1[7] = 0;

            r = ConvertYCrCbToRGB(y2, cr, cb);
            lpDibLine2[0] = r.rgbBlue;
            lpDibLine2[1] = r.rgbGreen;
            lpDibLine2[2] = r.rgbRed;
            lpDibLine2[3] = 0;

            r = ConvertYCrCbToRGB(y3, cr, cb);
            lpDibLine2[4] = r.rgbBlue;
            lpDibLine2[5] = r.rgbGreen;
            lpDibLine2[6] = r.rgbRed;
            lpDibLine2[7] = 0;

            lpLineY1 += 2;
            lpLineY2 += 2;
            lpLineCr += 2;
            lpLineCb += 2;

            lpDibLine1 += 8;
            lpDibLine2 += 8;
        }

        pDst += (2 * dstStride);
        lpBitsY   += (2 * srcStride);
        lpBitsCr  += srcStride;
        lpBitsCb  += srcStride;
    }
}


//-------------------------------------------------------------------
// ApplyMirror
//
// Apply horizontal mirror to an RGB32 buffer in-place.
//-------------------------------------------------------------------

void DrawDevice::ApplyMirror(BYTE* pRgb, UINT w, UINT h)
{
    if (!m_overlay.mirrorHorizontal || pRgb == nullptr || w == 0 || h == 0)
    {
        return;
    }

    LONG stride = (LONG)(w * 4);
    for (UINT y = 0; y < h; y++)
    {
        DWORD* row = (DWORD*)(pRgb + y * stride);
        for (UINT x = 0; x < w / 2; x++)
        {
            std::swap(row[x], row[w - 1 - x]);
        }
    }
}


//-------------------------------------------------------------------
// DrawOverlays
//
// Draw crosshair and reference lines on the back buffer using GDI.
//-------------------------------------------------------------------

void DrawDevice::DrawOverlays(IDirect3DSurface9* pBB)
{
    if (!m_overlay.showCrosshair && !m_overlay.showReferenceLines)
    {
		// 即使没有十字线/网格，也允许绘制运动 HUD（未来可单独开关）
    }

    if (pBB == nullptr)
    {
        return;
    }

    // Lock the surface and draw into the RGB32 buffer.
    D3DLOCKED_RECT lr = {};
    HRESULT hr = pBB->LockRect(&lr, NULL, 0);
    if (FAILED(hr) || lr.pBits == nullptr || lr.Pitch <= 0)
    {
        return;
    }

    const int w = (int)m_width;
    const int h = (int)m_height;
    if (w <= 0 || h <= 0)
    {
        pBB->UnlockRect();
        return;
    }

    const auto toD3dColor = [](COLORREF c) -> DWORD
    {
        return (DWORD)D3DCOLOR_XRGB(GetRValue(c), GetGValue(c), GetBValue(c));
    };

    const DWORD colCross = toD3dColor(m_overlay.crosshairColor);
    const DWORD colGrid = toD3dColor(m_overlay.refLineColor);

    const int cx = w / 2;
    const int cy = h / 2;

    auto setPixel = [&](int x, int y, DWORD c)
    {
        if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
        DWORD* row = (DWORD*)((BYTE*)lr.pBits + (size_t)y * (size_t)lr.Pitch);
        row[x] = c;
    };

    auto drawHLine = [&](int y, DWORD c, int thickness, bool dotted)
    {
        for (int t = -thickness; t <= thickness; t++)
        {
            int yy = y + t;
            if ((unsigned)yy >= (unsigned)h) continue;
            for (int x = 0; x < w; x++)
            {
                if (dotted && ((x & 3) != 0)) continue;
                setPixel(x, yy, c);
            }
        }
    };

    auto drawVLine = [&](int x, DWORD c, int thickness, bool dotted)
    {
        for (int t = -thickness; t <= thickness; t++)
        {
            int xx = x + t;
            if ((unsigned)xx >= (unsigned)w) continue;
            for (int y = 0; y < h; y++)
            {
                if (dotted && ((y & 3) != 0)) continue;
                setPixel(xx, y, c);
            }
        }
    };

    if (m_overlay.showCrosshair)
    {
        drawHLine(cy, colCross, 1, false);
        drawVLine(cx, colCross, 1, false);

        // Small center box
        const int ss = 10;
        for (int x = cx - ss; x <= cx + ss; x++)
        {
            setPixel(x, cy - ss, colCross);
            setPixel(x, cy + ss, colCross);
        }
        for (int y = cy - ss; y <= cy + ss; y++)
        {
            setPixel(cx - ss, y, colCross);
            setPixel(cx + ss, y, colCross);
        }
    }

    if (m_overlay.showReferenceLines)
    {
        // Rule-of-thirds grid (dotted).
        const int x1 = w / 3;
        const int x2 = (2 * w) / 3;
        const int y1 = h / 3;
        const int y2 = (2 * h) / 3;

        drawVLine(x1, colGrid, 0, true);
        drawVLine(x2, colGrid, 0, true);
        drawHLine(y1, colGrid, 0, true);
        drawHLine(y2, colGrid, 0, true);
    }

	// =========================================================
	// Visual Servo：目标点 + 误差箭头（不依赖 OpenCV 的兜底绘制路径）
	// =========================================================
	// 只在 OpenCV 不可用时启用，避免与 OpenCV HUD 重复，也满足“OpenCV 生效就不需要自己画”的诉求。
#if !(defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV)
	{
		const auto snap = KinematicsOverlayService::Instance().GetSnapshot();
		if (snap.vsEnabled)
		{
			// 颜色：正在由视觉驱动则高亮绿色，否则灰色（仅提示已启用但未覆盖手动）
			const DWORD colVs = snap.vsApplied ? toD3dColor(RGB(0, 255, 0)) : toD3dColor(RGB(200, 200, 200));

			const int cxp = w / 2;
			const int cyp = h / 2;

			// 目标点 = 画面中心 + 像素误差（与 VisualServoController 约定一致）
			int tx = cxp + (int)std::lround(snap.vsErrU);
			int ty = cyp + (int)std::lround(snap.vsErrV);

			// Clamp
			if (tx < 0) tx = 0;
			if (ty < 0) ty = 0;
			if (tx >= w) tx = w - 1;
			if (ty >= h) ty = h - 1;

			// 画一条中心->目标的直线（Bresenham）
			auto drawLine = [&](int x0, int y0, int x1, int y1, DWORD c, int thickness)
			{
				int dx = std::abs(x1 - x0);
				int sx = (x0 < x1) ? 1 : -1;
				int dy = -std::abs(y1 - y0);
				int sy = (y0 < y1) ? 1 : -1;
				int err = dx + dy;

				while (true)
				{
					for (int oy = -thickness; oy <= thickness; oy++)
					{
						for (int ox = -thickness; ox <= thickness; ox++)
						{
							setPixel(x0 + ox, y0 + oy, c);
						}
					}
					if (x0 == x1 && y0 == y1) break;
					const int e2 = 2 * err;
					if (e2 >= dy) { err += dy; x0 += sx; }
					if (e2 <= dx) { err += dx; y0 += sy; }
				}
			};

			// 画一个空心圆（简单采样）
			auto drawCircle = [&](int x, int y, int r, DWORD c)
			{
				// 采样 0..360 度，每 5 度一个点，足够用于 HUD
				for (int a = 0; a < 360; a += 5)
				{
					const double rad = (double)a * 3.14159265358979323846 / 180.0;
					const int px = x + (int)std::lround((double)r * std::cos(rad));
					const int py = y + (int)std::lround((double)r * std::sin(rad));
					setPixel(px, py, c);
				}
			};

			// 画箭头主体
			drawLine(cxp, cyp, tx, ty, colVs, 1);

			// 画箭头头部（两条短线）
			{
				const double vx = (double)(tx - cxp);
				const double vy = (double)(ty - cyp);
				const double len = std::sqrt(vx * vx + vy * vy);
				if (len > 1e-3)
				{
					const double ux = vx / len;
					const double uy = vy / len;
					// 头部长度与夹角（可调）
					const double headLen = 16.0;
					const double ca = std::cos(25.0 * 3.14159265358979323846 / 180.0);
					const double sa = std::sin(25.0 * 3.14159265358979323846 / 180.0);

					// 左右两翼：对单位向量做旋转
					const double lx = ux * ca - uy * sa;
					const double ly = ux * sa + uy * ca;
					const double rx = ux * ca + uy * sa;
					const double ry = -ux * sa + uy * ca;

					const int xL = tx - (int)std::lround(lx * headLen);
					const int yL = ty - (int)std::lround(ly * headLen);
					const int xR = tx - (int)std::lround(rx * headLen);
					const int yR = ty - (int)std::lround(ry * headLen);

					drawLine(tx, ty, xL, yL, colVs, 1);
					drawLine(tx, ty, xR, yR, colVs, 1);
				}
			}

			// 画目标点圆圈
			drawCircle(tx, ty, 6, colVs);
			drawCircle(tx, ty, 7, colVs);
		}
	}
#endif

#if defined(SMARTARM_HAS_OPENCV) && SMARTARM_HAS_OPENCV
	// ==========================
	// HUD 叠加（OpenCV）
	// ==========================
	// 注意：OpenCV 的 putText 默认不支持中文字体，这里使用英文/符号，避免乱码。
	{
		cv::Mat img(h, w, CV_8UC4, lr.pBits, (size_t)lr.Pitch);
		const auto snap = KinematicsOverlayService::Instance().GetSnapshot();

		// 左上角 HUD 文本
		char buf1[256] = {};
		sprintf_s(buf1, "Jog:%s  IK:%s", snap.jogActive ? "ON" : "OFF", snap.ikOk ? "OK" : "FAIL");
		cv::putText(img, buf1, cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255, 0), 2, cv::LINE_AA);

		char buf2[256] = {};
		sprintf_s(buf2, "X=%.0f  Y=%.0f  Z=%.0f  P=%.1f",
		          snap.target.x_mm, snap.target.y_mm, snap.target.z_mm, snap.target.pitch_deg);
		cv::putText(img, buf2, cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255, 0), 1, cv::LINE_AA);

		char buf3[256] = {};
		sprintf_s(buf3, "TX:%u fps  last:%u ms", snap.sendFps, snap.sinceLastSendMs);
		cv::putText(img, buf3, cv::Point(12, 72), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(180, 255, 180, 0), 1, cv::LINE_AA);

		// Jog 向量指示（从一个小原点画箭头）
		const cv::Point o(30, 105);
		const cv::Point e(o.x + (int)std::lround(snap.joyX * 25.0), o.y - (int)std::lround(snap.joyY * 25.0));
		cv::circle(img, o, 3, cv::Scalar(200, 200, 200, 0), -1, cv::LINE_AA);
		cv::arrowedLine(img, o, e, cv::Scalar(0, 180, 255, 0), 2, cv::LINE_AA);

		// Visual Servo：目标点 + 误差箭头（OpenCV 版本）
		if (snap.vsEnabled)
		{
			const int cxp = w / 2;
			const int cyp = h / 2;
			const cv::Point c(cxp, cyp);
			const cv::Point t(cxp + (int)std::lround(snap.vsErrU), cyp + (int)std::lround(snap.vsErrV));

			// Clamp to frame
			auto clampPt = [&](cv::Point p) -> cv::Point {
				p.x = std::max(0, std::min(w - 1, p.x));
				p.y = std::max(0, std::min(h - 1, p.y));
				return p;
			};
			const cv::Point tc = clampPt(t);

			const cv::Scalar col = snap.vsApplied ? cv::Scalar(0, 255, 0, 0) : cv::Scalar(200, 200, 200, 0);
			cv::circle(img, tc, 5, col, 2, cv::LINE_AA);
			cv::arrowedLine(img, c, tc, col, 2, cv::LINE_AA, 0, 0.15);

			const char* modeStr = "Center";
			if (snap.vsMode == 1) modeStr = "FollowRay";
			else if (snap.vsMode == 2) modeStr = "Look&Move";

			char bufVs[256] = {};
			sprintf_s(bufVs, "VS:%s %s e(%.0f,%.0f) adv=%.2f",
			          snap.vsActive ? "ON" : "IDLE",
			          modeStr,
			          snap.vsErrU, snap.vsErrV,
			          snap.vsAdvance);
			cv::putText(img, bufVs, cv::Point(12, 120), cv::FONT_HERSHEY_SIMPLEX, 0.55,
			            snap.vsApplied ? cv::Scalar(0, 255, 0, 0) : cv::Scalar(200, 200, 200, 0),
			            1, cv::LINE_AA);
		}

		// IK 失败原因（只显示短提示）
		if (!snap.ikOk)
		{
			cv::putText(img, "IK FAIL", cv::Point(12, 96), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 0), 2, cv::LINE_AA);
		}
	}
#endif

    pBB->UnlockRect();
}

