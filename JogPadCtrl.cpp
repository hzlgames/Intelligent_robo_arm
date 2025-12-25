#include "pch.h"

#include "JogPadCtrl.h"

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <algorithm>
#include <cmath>

BEGIN_MESSAGE_MAP(CJogPadCtrl, CStatic)
	ON_WM_PAINT()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_CAPTURECHANGED()
END_MESSAGE_MAP()

namespace
{
	int ClampInt(int v, int mn, int mx)
	{
		if (v < mn) return mn;
		if (v > mx) return mx;
		return v;
	}
}

void CJogPadCtrl::ResetCenter()
{
	m_bActive = false;
	m_x = 0.0;
	m_y = 0.0;
	Invalidate(FALSE);
}

void CJogPadCtrl::OnPaint()
{
	CPaintDC dc(this);

	CRect rc;
	GetClientRect(&rc);
	dc.FillSolidRect(&rc, RGB(15, 15, 15));

	// 圆盘范围
	const int w = rc.Width();
	const int h = rc.Height();
	const int cx = rc.left + w / 2;
	const int cy = rc.top + h / 2;
	const int radius = std::max(10, std::min(w, h) / 2 - 8);

	CPen penBorder(PS_SOLID, 1, RGB(80, 80, 80));
	CPen* pOldPen = dc.SelectObject(&penBorder);
	CBrush brushNone;
	brushNone.CreateStockObject(NULL_BRUSH);
	CBrush* pOldBrush = dc.SelectObject(&brushNone);

	dc.Ellipse(cx - radius, cy - radius, cx + radius, cy + radius);

	// 十字
	CPen penCross(PS_SOLID, 1, RGB(60, 60, 60));
	dc.SelectObject(&penCross);
	dc.MoveTo(cx - radius, cy);
	dc.LineTo(cx + radius, cy);
	dc.MoveTo(cx, cy - radius);
	dc.LineTo(cx, cy + radius);

	// 摇杆“旋钮”位置
	const int knobR = 8;
	const int kx = cx + (int)std::lround(m_x * (double)(radius - knobR));
	const int ky = cy - (int)std::lround(m_y * (double)(radius - knobR)); // 上为正

	CBrush brushKnob(m_bActive ? RGB(0, 180, 255) : RGB(120, 120, 120));
	dc.SelectObject(&brushKnob);
	dc.SelectObject(&penBorder);
	dc.Ellipse(kx - knobR, ky - knobR, kx + knobR, ky + knobR);

	// 文字提示
	dc.SetBkMode(TRANSPARENT);
	dc.SetTextColor(RGB(220, 220, 220));
	dc.TextOutW(rc.left + 6, rc.top + 6, m_bActive ? L"拖动中" : L"按住拖动");

	dc.SelectObject(pOldBrush);
	dc.SelectObject(pOldPen);
}

void CJogPadCtrl::OnLButtonDown(UINT nFlags, CPoint point)
{
	SetCapture();
	m_bActive = true;
	UpdateFromPoint(point);
	CStatic::OnLButtonDown(nFlags, point);
}

void CJogPadCtrl::OnLButtonUp(UINT nFlags, CPoint point)
{
	if (GetCapture() == this)
	{
		ReleaseCapture();
	}
	m_bActive = false;
	m_x = 0.0;
	m_y = 0.0;
	Invalidate(FALSE);
	CStatic::OnLButtonUp(nFlags, point);
}

void CJogPadCtrl::OnMouseMove(UINT nFlags, CPoint point)
{
	if (m_bActive && GetCapture() == this)
	{
		UpdateFromPoint(point);
	}
	CStatic::OnMouseMove(nFlags, point);
}

void CJogPadCtrl::OnCaptureChanged(CWnd* pWnd)
{
	// 防止异常丢失捕获导致“卡住按下状态”
	UNREFERENCED_PARAMETER(pWnd);
	if (m_bActive)
	{
		m_bActive = false;
		m_x = 0.0;
		m_y = 0.0;
		Invalidate(FALSE);
	}
	CStatic::OnCaptureChanged(pWnd);
}

void CJogPadCtrl::UpdateFromPoint(CPoint point)
{
	CRect rc;
	GetClientRect(&rc);

	const int w = rc.Width();
	const int h = rc.Height();
	const int cx = rc.left + w / 2;
	const int cy = rc.top + h / 2;
	const int radius = std::max(10, std::min(w, h) / 2 - 8);

	const int dx = point.x - cx;
	const int dy = point.y - cy;

	const double dist = std::sqrt((double)dx * (double)dx + (double)dy * (double)dy);
	double nx = 0.0;
	double ny = 0.0;
	if (dist > 1e-6)
	{
		// 截断到圆盘范围内
		const double k = std::min(1.0, (double)radius / dist);
		nx = ((double)dx * k) / (double)radius;
		ny = (-(double)dy * k) / (double)radius; // 上为正
	}

	// 安全夹取 [-1,1]
	nx = std::max(-1.0, std::min(1.0, nx));
	ny = std::max(-1.0, std::min(1.0, ny));

	m_x = nx;
	m_y = ny;
	Invalidate(FALSE);
}



