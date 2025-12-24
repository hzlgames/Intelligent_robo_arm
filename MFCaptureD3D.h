//////////////////////////////////////////////////////////////////////////
//
// Minimal shared header for MF + D3D9 preview code (adapted from Microsoft's
// MFCaptureD3D sample, used here without the original WinMain/UI pieces).
//
//////////////////////////////////////////////////////////////////////////

#pragma once

#include <new>
#include <windows.h>
#include <d3d9.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>

#include <strsafe.h>
#include <assert.h>

#include <ks.h>
#include <ksmedia.h>
#include <Dbt.h>

template <class T> void SafeRelease(T** ppT)
{
	if (ppT && *ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

#include "device.h"
#include "preview.h"


