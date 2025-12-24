//////////////////////////////////////////////////////////////////////////
//
// DrawDevice: Direct3D9 presenter for MF video frames (RGB32 conversion).
// Adapted from Microsoft's MFCaptureD3D sample, with small extensions:
// - expose current width/height/subtype
// - keep a copy of the latest RGB32 frame for screenshots/diagnostics
// - orientation transforms: mirror (horizontal flip), rotate (0/90/180/270)
// - overlay helpers: crosshair, reference lines
//
//////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>

// Function pointer for the function that transforms the image.
typedef void (*IMAGE_TRANSFORM_FN)(
	BYTE*       pDest,
	LONG        lDestStride,
	const BYTE* pSrc,
	LONG        lSrcStride,
	DWORD       dwWidthInPixels,
	DWORD       dwHeightInPixels
	);

// Orientation/overlay settings
enum class VideoRotation { None = 0, Rotate90 = 90, Rotate180 = 180, Rotate270 = 270 };

struct VideoOverlaySettings
{
	bool mirrorHorizontal = false;
	VideoRotation rotation = VideoRotation::None;
	bool showCrosshair = false;
	bool showReferenceLines = false;
	COLORREF crosshairColor = RGB(255, 0, 0);     // Red
	COLORREF refLineColor = RGB(0, 255, 0);       // Green
};

class DrawDevice
{
private:

	HWND                    m_hwnd;

	IDirect3D9* m_pD3D;
	IDirect3DDevice9* m_pDevice;
	IDirect3DSwapChain9* m_pSwapChain;

	D3DPRESENT_PARAMETERS   m_d3dpp;

	// Format information
	GUID                    m_subtype;
	D3DFORMAT               m_format;
	UINT                    m_width;
	UINT                    m_height;
	LONG                    m_lDefaultStride;
	MFRatio                 m_PixelAR;
	MFVideoInterlaceMode    m_interlace;
	RECT                    m_rcDest;

	// Drawing
	IMAGE_TRANSFORM_FN      m_convertFn;

	// Last RGB frame copy (stride = width*4)
	std::vector<BYTE>       m_lastRgb;

	// Orientation/overlay
	VideoOverlaySettings    m_overlay;

private:

	HRESULT TestCooperativeLevel();
	HRESULT SetConversionFunction(REFGUID subtype);
	HRESULT CreateSwapChains();
	void    UpdateDestinationRect();
	void    UpdateLastRgb(const BYTE* src, LONG srcStride);
	void    ApplyMirror(BYTE* pRgb, UINT w, UINT h);
	void    DrawOverlays(IDirect3DSurface9* pBB);

public:

	DrawDevice();
	virtual ~DrawDevice();

	HRESULT CreateDevice(HWND hwnd);
	HRESULT ResetDevice();
	void    DestroyDevice();

	HRESULT SetVideoType(IMFMediaType* pType);
	HRESULT DrawFrame(IMFMediaBuffer* pBuffer);

	BOOL     IsFormatSupported(REFGUID subtype) const;
	HRESULT  GetFormat(DWORD index, GUID* pSubtype)  const;

	UINT GetWidth() const { return m_width; }
	UINT GetHeight() const { return m_height; }
	GUID GetSubtype() const { return m_subtype; }

	// Copies the latest RGB32 frame into outRgb. Returns false if not ready yet.
	bool CopyLastRgb(std::vector<BYTE>& outRgb, UINT& outW, UINT& outH) const;

	// Orientation/overlay control
	void SetOverlaySettings(const VideoOverlaySettings& s) { m_overlay = s; }
	VideoOverlaySettings GetOverlaySettings() const { return m_overlay; }
	void SetMirrorHorizontal(bool mirror) { m_overlay.mirrorHorizontal = mirror; }
	void SetRotation(VideoRotation r) { m_overlay.rotation = r; }
	void SetShowCrosshair(bool show) { m_overlay.showCrosshair = show; }
	void SetShowReferenceLines(bool show) { m_overlay.showReferenceLines = show; }
};


