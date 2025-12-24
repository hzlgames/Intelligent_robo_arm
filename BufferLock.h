//////////////////////////////////////////////////////////////////////////
//
// Adapted from Microsoft's MFCaptureD3D sample: helper to lock MF buffers.
//
//////////////////////////////////////////////////////////////////////////

#pragma once

class VideoBufferLock
{
public:
	VideoBufferLock(IMFMediaBuffer* pBuffer) : m_p2DBuffer(NULL), m_bLocked(FALSE)
	{
		m_pBuffer = pBuffer;
		m_pBuffer->AddRef();

		// Query for the 2-D buffer interface. OK if this fails.
		(void)m_pBuffer->QueryInterface(IID_PPV_ARGS(&m_p2DBuffer));
	}

	~VideoBufferLock()
	{
		UnlockBuffer();
		SafeRelease(&m_pBuffer);
		SafeRelease(&m_p2DBuffer);
	}

	HRESULT LockBuffer(
		LONG  lDefaultStride,
		DWORD dwHeightInPixels,
		BYTE** ppbScanLine0,
		LONG* plStride)
	{
		HRESULT hr = S_OK;

		// Use the 2-D version if available.
		if (m_p2DBuffer)
		{
			hr = m_p2DBuffer->Lock2D(ppbScanLine0, plStride);
		}
		else
		{
			BYTE* pData = NULL;

			hr = m_pBuffer->Lock(&pData, NULL, NULL);
			if (SUCCEEDED(hr))
			{
				*plStride = lDefaultStride;
				if (lDefaultStride < 0)
				{
					*ppbScanLine0 = pData + abs(lDefaultStride) * (dwHeightInPixels - 1);
				}
				else
				{
					*ppbScanLine0 = pData;
				}
			}
		}

		m_bLocked = (SUCCEEDED(hr));

		return hr;
	}

	void UnlockBuffer()
	{
		if (m_bLocked)
		{
			if (m_p2DBuffer)
			{
				(void)m_p2DBuffer->Unlock2D();
			}
			else
			{
				(void)m_pBuffer->Unlock();
			}
			m_bLocked = FALSE;
		}
	}

private:
	IMFMediaBuffer* m_pBuffer;
	IMF2DBuffer* m_p2DBuffer;

	BOOL m_bLocked;
};


