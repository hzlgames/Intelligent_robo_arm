#pragma once

#include <afxwin.h>

// 简单虚拟摇杆控件：鼠标按住拖动，输出归一化 (-1..1) 的 X/Y。
// - X：向右为正
// - Y：向上为正（对应 Base 坐标系的 +Y“向前”，我们在主界面里将其映射为 +Y）
// - 按住为 active，松开回中
class CJogPadCtrl : public CStatic
{
public:
	CJogPadCtrl() = default;
	virtual ~CJogPadCtrl() = default;

	bool IsActive() const { return m_bActive; }
	double GetX() const { return m_x; }
	double GetY() const { return m_y; }

	void ResetCenter();

protected:
	afx_msg void OnPaint();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnCaptureChanged(CWnd* pWnd);

	DECLARE_MESSAGE_MAP()

private:
	void UpdateFromPoint(CPoint point);

private:
	bool m_bActive = false;
	double m_x = 0.0; // [-1,1]
	double m_y = 0.0; // [-1,1]
};



