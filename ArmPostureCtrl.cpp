#include "pch.h"
#include "ArmPostureCtrl.h"
#include <cmath>
#include <algorithm>

IMPLEMENT_DYNAMIC(CArmPostureCtrl, CStatic)

CArmPostureCtrl::CArmPostureCtrl()
{
	// 初始化所有关节为无效状态
	for (auto& j : m_joints)
	{
		j.servoPos = 500;
		j.angleDeg = 0.0;
		j.valid = false;
	}
}

CArmPostureCtrl::~CArmPostureCtrl()
{
}

BEGIN_MESSAGE_MAP(CArmPostureCtrl, CStatic)
	ON_WM_PAINT()
	ON_WM_ERASEBKGND()
END_MESSAGE_MAP()

void CArmPostureCtrl::SetJointState(int jointIndex, int servoPos, double angleDeg, bool valid)
{
	if (jointIndex < 1 || jointIndex > 6) return;
	m_joints[(size_t)jointIndex].servoPos = servoPos;
	m_joints[(size_t)jointIndex].angleDeg = angleDeg;
	m_joints[(size_t)jointIndex].valid = valid;
}

void CArmPostureCtrl::SetJointTargetPos(int jointIndex, int targetPos)
{
	if (jointIndex < 1 || jointIndex > 6) return;
	m_joints[(size_t)jointIndex].targetPos = targetPos;
}

void CArmPostureCtrl::SetGripperState(int servoPos, int openPos, int closePos, bool valid)
{
	m_joints[6].servoPos = servoPos;
	m_joints[6].valid = valid;
	m_gripperOpenPos = openPos;
	m_gripperClosePos = closePos;
}

void CArmPostureCtrl::SetLinkLengths(double L_base, double L_arm1, double L_arm2, double L_wrist)
{
	m_L_base = L_base;
	m_L_arm1 = L_arm1;
	m_L_arm2 = L_arm2;
	m_L_wrist = L_wrist;
}

void CArmPostureCtrl::RefreshDisplay()
{
	if (GetSafeHwnd())
	{
		Invalidate(FALSE);
	}
}

BOOL CArmPostureCtrl::OnEraseBkgnd(CDC* pDC)
{
	// 避免闪烁
	return TRUE;
}

void CArmPostureCtrl::OnPaint()
{
	CPaintDC dc(this);

	CRect rcClient;
	GetClientRect(&rcClient);

	// 双缓冲绘图
	CDC memDC;
	CBitmap memBmp;
	memDC.CreateCompatibleDC(&dc);
	memBmp.CreateCompatibleBitmap(&dc, rcClient.Width(), rcClient.Height());
	CBitmap* pOldBmp = memDC.SelectObject(&memBmp);

	// 填充背景
	memDC.FillSolidRect(&rcClient, m_colorBackground);

	// 布局：垂直三段式 (上:俯视 | 中:侧视 | 下:数值)
	const int padding = 5;
	const int valuesHeight = 50;  // 底部数值区域高度 (两行文字，紧凑点)
	
	// 剩余高度分配给 TopView 和 SideView
	// 侧视图需要更多高度展示机械臂伸展，给它 60%，俯视图 40%
	const int availH = std::max(10, rcClient.Height() - valuesHeight - padding * 4);
	const int topH = (int)(availH * 0.4); 
	const int sideH = availH - topH;

	CRect rcTopView(padding, padding, rcClient.Width() - padding, padding + topH);
	CRect rcSideView(padding, padding + topH + padding, rcClient.Width() - padding, padding + topH + padding + sideH);
	CRect rcValues(padding, rcClient.Height() - valuesHeight - padding, rcClient.Width() - padding, rcClient.Height() - padding);

	// 绘制各部分
	DrawTopView(&memDC, rcTopView);
	DrawSideView(&memDC, rcSideView);
	DrawAngleValues(&memDC, rcValues);

	// 复制到屏幕
	dc.BitBlt(0, 0, rcClient.Width(), rcClient.Height(), &memDC, 0, 0, SRCCOPY);

	memDC.SelectObject(pOldBmp);
}

void CArmPostureCtrl::DrawTopView(CDC* pDC, const CRect& rect)
{
	// 绘制边框
	CPen penBorder(PS_SOLID, 1, m_colorGrid);
	CPen* pOldPen = pDC->SelectObject(&penBorder);
	pDC->Rectangle(rect);

	// 标题
	pDC->SetBkMode(TRANSPARENT);
	pDC->SetTextColor(m_colorText);
	CFont fontTitle;
	fontTitle.CreateFont(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
	CFont* pOldFont = pDC->SelectObject(&fontTitle);
	pDC->TextOutW(rect.left + 5, rect.top + 3, L"底座旋转 (J1 俯视)");

	// 计算圆心和半径
	const int margin = 20; // 减小边距
	const int cx = rect.CenterPoint().x;
	const int cy = rect.CenterPoint().y + 8;
	int radius = std::min(rect.Width(), rect.Height()) / 2 - margin;

	if (radius < 10) radius = 10; // 即使很小也画一点

	// 绘制刻度盘
	CPen penDial(PS_SOLID, 1, m_colorGrid);
	pDC->SelectObject(&penDial);

	// 外圈
	pDC->Ellipse(cx - radius, cy - radius, cx + radius, cy + radius);

	// 刻度线 (每30度)
	for (int deg = 0; deg < 360; deg += 30)
	{
		const double rad = DegToRad((double)deg - 90.0); // -90 使 0° 朝上
		const int r1 = (deg % 90 == 0) ? (int)(radius * 0.75) : (int)(radius * 0.85);
		const int r2 = radius;
		const int x1 = cx + (int)(r1 * cos(rad));
		const int y1 = cy + (int)(r1 * sin(rad));
		const int x2 = cx + (int)(r2 * cos(rad));
		const int y2 = cy + (int)(r2 * sin(rad));
		pDC->MoveTo(x1, y1);
		pDC->LineTo(x2, y2);
	}

	// 刻度标注
	CFont fontScale;
	fontScale.CreateFont(11, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
	pDC->SelectObject(&fontScale);
	pDC->SetTextColor(m_colorTextDim);

	const wchar_t* labels[] = { L"0°", L"90°", L"180°", L"-90°" };
	const int labelAngles[] = { -90, 0, 90, 180 }; // 屏幕坐标系
	for (int i = 0; i < 4; i++)
	{
		const double rad = DegToRad((double)labelAngles[i]);
		const int lx = cx + (int)((radius + 12) * cos(rad)) - 12;
		const int ly = cy + (int)((radius + 12) * sin(rad)) - 7;
		pDC->TextOutW(lx, ly, labels[i]);
	}

	// 绘制机械臂指向（J1 角度）
	// 注意：屏幕坐标系 Y 轴向下，所以需要取反 j1Deg 才能正确显示逆时针为正
	const double j1Deg = m_joints[1].angleDeg;
	const double j1DegDraw = -j1Deg;  // 取反以匹配俯视图（逆时针为正）
	const bool j1Valid = m_joints[1].valid;

	// 角度弧线
	if (std::fabs(j1Deg) > 0.5)
	{
		CPen penArc(PS_SOLID, 2, j1Valid ? m_colorAngleArc : m_colorInvalid);
		pDC->SelectObject(&penArc);

		const int arcR = (int)(radius * 0.4);
		const double startRad = DegToRad(-90.0);
		const double endRad = DegToRad(-90.0 + j1DegDraw);
		const double step = (j1DegDraw > 0) ? 2.0 : -2.0;

		int prevX = cx + (int)(arcR * cos(startRad));
		int prevY = cy + (int)(arcR * sin(startRad));
		for (double d = step; std::fabs(d) <= std::fabs(j1DegDraw) + 0.1; d += step)
		{
			const double curRad = DegToRad(-90.0 + d);
			const int curX = cx + (int)(arcR * cos(curRad));
			const int curY = cy + (int)(arcR * sin(curRad));
			pDC->MoveTo(prevX, prevY);
			pDC->LineTo(curX, curY);
			prevX = curX;
			prevY = curY;
		}
	}

	// 机械臂指向箭头
	const double armRad = DegToRad(-90.0 + j1DegDraw);
	const int armLen = (int)(radius * 0.7);

	CPen penArm(PS_SOLID, 4, j1Valid ? m_colorArm1 : m_colorInvalid);
	pDC->SelectObject(&penArm);

	const int armEndX = cx + (int)(armLen * cos(armRad));
	const int armEndY = cy + (int)(armLen * sin(armRad));
	pDC->MoveTo(cx, cy);
	pDC->LineTo(armEndX, armEndY);

	// 箭头头部
	const double arrowAngle = 25.0;
	const int arrowLen = 12;
	for (int sign = -1; sign <= 1; sign += 2)
	{
		const double aRad = armRad + DegToRad(180.0 + sign * arrowAngle);
		const int ax = armEndX + (int)(arrowLen * cos(aRad));
		const int ay = armEndY + (int)(arrowLen * sin(aRad));
		pDC->MoveTo(armEndX, armEndY);
		pDC->LineTo(ax, ay);
	}

	// 中心圆点（底座）
	CBrush brushBase(m_colorBase);
	CBrush* pOldBrush = pDC->SelectObject(&brushBase);
	pDC->Ellipse(cx - 8, cy - 8, cx + 8, cy + 8);
	pDC->SelectObject(pOldBrush);

	// 角度数值
	CString strAngle;
	strAngle.Format(L"%.1f°", j1Deg);
	pDC->SetTextColor(j1Valid ? m_colorAngleArc : m_colorInvalid);
	pDC->SelectObject(&fontTitle);
	CSize sz = pDC->GetTextExtent(strAngle);
	pDC->TextOutW(cx - sz.cx / 2, cy + radius + 15, strAngle);

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldFont);
}

void CArmPostureCtrl::DrawSideView(CDC* pDC, const CRect& rect)
{
	// 绘制边框
	CPen penBorder(PS_SOLID, 1, m_colorGrid);
	CPen* pOldPen = pDC->SelectObject(&penBorder);
	pDC->Rectangle(rect);

	// 标题
	pDC->SetBkMode(TRANSPARENT);
	pDC->SetTextColor(m_colorText);
	CFont fontTitle;
	fontTitle.CreateFont(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
	CFont* pOldFont = pDC->SelectObject(&fontTitle);
	pDC->TextOutW(rect.left + 5, rect.top + 3, L"机械臂侧视 (J2-J4 + 夹爪)");

	// 计算绘图区域
	const int margin = 20; // 减小边距
	const int drawLeft = rect.left + margin;
	const int drawRight = rect.right - margin;
	const int drawTop = rect.top + margin;
	const int drawBottom = rect.bottom - margin - 15;
	const int drawW = drawRight - drawLeft;
	const int drawH = drawBottom - drawTop;

	if (drawW < 10 || drawH < 10)
	{
		pDC->SelectObject(pOldPen);
		pDC->SelectObject(pOldFont);
		return;
	}

	// 计算比例尺：将机械臂完全展开时能放入绘图区
	const double totalReach = m_L_base + m_L_arm1 + m_L_arm2 + m_L_wrist;
	const double scale = std::min((double)drawW * 0.9, (double)drawH * 0.9) / totalReach;

	// 基座位置（绘图区左下角偏右）
	const int baseX = drawLeft + (int)(drawW * 0.3);
	const int baseY = drawBottom;

	// 绘制地面线
	CPen penGround(PS_DOT, 1, m_colorGrid);
	pDC->SelectObject(&penGround);
	pDC->MoveTo(drawLeft, baseY);
	pDC->LineTo(drawRight, baseY);

	// 绘制参考网格线（水平）
	for (int i = 1; i <= 4; i++)
	{
		const int y = baseY - (int)(i * 50 * scale);
		if (y < drawTop) break;
		pDC->MoveTo(drawLeft, y);
		pDC->LineTo(drawRight, y);
	}

	// 获取关节角度
	const double j2Deg = m_joints[2].angleDeg;
	const double j3Deg = m_joints[3].angleDeg;
	const double j4Deg = m_joints[4].angleDeg;
	const bool j2Valid = m_joints[2].valid;
	const bool j3Valid = m_joints[3].valid;
	const bool j4Valid = m_joints[4].valid;

	// 计算各关节位置（2D 侧视：X=水平伸展，Y=垂直高度，Y向上为正）
	// J2 在底座顶部
	const double x0 = 0.0;
	const double y0 = m_L_base;

	// J2 -> J3: 大臂
	// 累积角度：从垂直向上开始，j2 正方向使臂向前倾
	const double theta2 = DegToRad(90.0 - j2Deg); // 90° 为竖直向上
	const double x1 = x0 + m_L_arm1 * cos(theta2);
	const double y1 = y0 + m_L_arm1 * sin(theta2);

	// J3 -> J4: 小臂
	// j3 相对于 j2 的累积
	const double theta3 = theta2 - DegToRad(j3Deg);
	const double x2 = x1 + m_L_arm2 * cos(theta3);
	const double y2 = y1 + m_L_arm2 * sin(theta3);

	// J4 -> 末端: 腕部
	const double theta4 = theta3 - DegToRad(j4Deg);
	const double x3 = x2 + m_L_wrist * cos(theta4);
	const double y3 = y2 + m_L_wrist * sin(theta4);

	// 转换为屏幕坐标
	auto toScreenX = [&](double x) -> int { return baseX + (int)(x * scale); };
	auto toScreenY = [&](double y) -> int { return baseY - (int)(y * scale); };

	const int sx0 = baseX;
	const int sy0 = baseY;
	const int sx1 = baseX;
	const int sy1 = toScreenY(m_L_base);
	const int sx2 = toScreenX(x1);
	const int sy2 = toScreenY(y1);
	const int sx3 = toScreenX(x2);
	const int sy3 = toScreenY(y2);
	const int sx4 = toScreenX(x3);
	const int sy4 = toScreenY(y3);

	// 绘制底座
	CPen penBase(PS_SOLID, 6, m_colorBase);
	pDC->SelectObject(&penBase);
	pDC->MoveTo(sx0, sy0);
	pDC->LineTo(sx1, sy1);

	// 绘制大臂 (J2->J3)
	CPen penArm1(PS_SOLID, 5, j2Valid ? m_colorArm1 : m_colorInvalid);
	pDC->SelectObject(&penArm1);
	pDC->MoveTo(sx1, sy1);
	pDC->LineTo(sx2, sy2);

	// 绘制小臂 (J3->J4)
	CPen penArm2(PS_SOLID, 4, j3Valid ? m_colorArm2 : m_colorInvalid);
	pDC->SelectObject(&penArm2);
	pDC->MoveTo(sx2, sy2);
	pDC->LineTo(sx3, sy3);

	// 绘制腕部 (J4->End)
	CPen penWrist(PS_SOLID, 3, j4Valid ? m_colorWrist : m_colorInvalid);
	pDC->SelectObject(&penWrist);
	pDC->MoveTo(sx3, sy3);
	pDC->LineTo(sx4, sy4);

	// 绘制关节圆点
	auto drawJoint = [&](int x, int y, COLORREF color, int r) {
		CBrush brush(color);
		CBrush* pOld = pDC->SelectObject(&brush);
		CPen pen(PS_SOLID, 1, color);
		pDC->SelectObject(&pen);
		pDC->Ellipse(x - r, y - r, x + r, y + r);
		pDC->SelectObject(pOld);
	};

	drawJoint(sx1, sy1, m_colorBase, 6);      // J2 肩关节
	drawJoint(sx2, sy2, m_colorArm1, 5);      // J3 肘关节
	drawJoint(sx3, sy3, m_colorArm2, 4);      // J4 腕关节

	// 绘制夹爪
	const bool gripValid = m_joints[6].valid;
	const int gripPos = m_joints[6].servoPos;

	// 计算夹爪开合程度 (0.0=闭合, 1.0=张开)
	double gripOpen = 0.5;
	if (m_gripperOpenPos != m_gripperClosePos)
	{
		gripOpen = (double)(gripPos - m_gripperClosePos) / (double)(m_gripperOpenPos - m_gripperClosePos);
		gripOpen = std::max(0.0, std::min(1.0, gripOpen));
	}

	// 夹爪颜色插值
	COLORREF gripColor = gripValid ? 
		RGB(
			(int)(GetRValue(m_colorGripperClose) * (1.0 - gripOpen) + GetRValue(m_colorGripperOpen) * gripOpen),
			(int)(GetGValue(m_colorGripperClose) * (1.0 - gripOpen) + GetGValue(m_colorGripperOpen) * gripOpen),
			(int)(GetBValue(m_colorGripperClose) * (1.0 - gripOpen) + GetBValue(m_colorGripperOpen) * gripOpen)
		) : m_colorInvalid;

	// 绘制夹爪（两个小线段表示爪子）
	CPen penGrip(PS_SOLID, 3, gripColor);
	pDC->SelectObject(&penGrip);

	const double gripAngle = DegToRad(30.0 * gripOpen); // 张开角度
	const int gripLen = (int)(20 * scale);
	if (gripLen > 5)
	{
		// 左爪
		const double gripRad1 = theta4 + gripAngle;
		const int gx1 = sx4 + (int)(gripLen * cos(gripRad1));
		const int gy1 = sy4 - (int)(gripLen * sin(gripRad1));
		pDC->MoveTo(sx4, sy4);
		pDC->LineTo(gx1, gy1);

		// 右爪
		const double gripRad2 = theta4 - gripAngle;
		const int gx2 = sx4 + (int)(gripLen * cos(gripRad2));
		const int gy2 = sy4 - (int)(gripLen * sin(gripRad2));
		pDC->MoveTo(sx4, sy4);
		pDC->LineTo(gx2, gy2);
	}

	// 末端点
	drawJoint(sx4, sy4, gripColor, 4);

	// 绘制角度标注
	CFont fontSmall;
	fontSmall.CreateFont(11, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
	pDC->SelectObject(&fontSmall);

	CString str;
	
	// J2 角度
	pDC->SetTextColor(j2Valid ? m_colorArm1 : m_colorInvalid);
	str.Format(L"J2:%.1f°", j2Deg);
	pDC->TextOutW(sx1 - 35, sy1 - 5, str);

	// J3 角度
	pDC->SetTextColor(j3Valid ? m_colorArm2 : m_colorInvalid);
	str.Format(L"J3:%.1f°", j3Deg);
	pDC->TextOutW(sx2 + 5, sy2 - 15, str);

	// J4 角度
	pDC->SetTextColor(j4Valid ? m_colorWrist : m_colorInvalid);
	str.Format(L"J4:%.1f°", j4Deg);
	pDC->TextOutW(sx3 + 5, sy3 + 5, str);

	// 夹爪状态
	pDC->SetTextColor(gripValid ? gripColor : m_colorInvalid);
	str.Format(L"Grip:%.0f%%", gripOpen * 100.0);
	pDC->TextOutW(sx4 + 5, sy4 - 5, str);

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldFont);
}

void CArmPostureCtrl::DrawAngleValues(CDC* pDC, const CRect& rect)
{
	// 绘制边框
	CPen penBorder(PS_SOLID, 1, m_colorGrid);
	CPen* pOldPen = pDC->SelectObject(&penBorder);
	pDC->Rectangle(rect);

	pDC->SetBkMode(TRANSPARENT);

	// 紧凑字体
	CFont fontHeader;
	fontHeader.CreateFont(11, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");

	CFont fontValue;
	fontValue.CreateFont(10, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");

	CFont* pOldFont = pDC->SelectObject(&fontHeader);

	const int cols = 6;
	const int colWidth = rect.Width() / cols;
	// 动态计算行高：4行 = 表头 + 目标位置(T) + 返回位置(R) + 角度
	const int rowHeight = rect.Height() / 4;
	const int y0 = rect.top + 2;

	// 表头
	const wchar_t* headers[] = { L"J1", L"J2", L"J3", L"J4", L"J5", L"Grip" };
	pDC->SetTextColor(m_colorTextDim);
	for (int i = 0; i < cols; i++)
	{
		const int x = rect.left + i * colWidth + 3;
		pDC->TextOutW(x, y0, headers[i]);
	}

	// 数值行
	pDC->SelectObject(&fontValue);
	const int y1 = y0 + rowHeight;      // 目标位置 (T)
	const int y2 = y1 + rowHeight;      // 返回位置 (R)
	const int y3 = y2 + rowHeight;      // 角度

	// 目标位置颜色（蓝绿色）
	const COLORREF colorTarget = RGB(80, 200, 200);
	// 返回位置颜色（绿色）
	const COLORREF colorReturn = RGB(100, 220, 100);

	for (int i = 1; i <= 6; i++)
	{
		const int x = rect.left + (i - 1) * colWidth + 3;
		const auto& j = m_joints[(size_t)i];

		// 目标位置 (T:xxx)
		CString strTarget;
		if (j.targetPos >= 0)
		{
			strTarget.Format(L"T:%d", j.targetPos);
			pDC->SetTextColor(colorTarget);
		}
		else
		{
			strTarget = L"T:---";
			pDC->SetTextColor(m_colorTextDim);
		}
		pDC->TextOutW(x, y1, strTarget);

		// 返回位置 (R:xxx)
		CString strPos;
		strPos.Format(L"R:%d", j.servoPos);
		pDC->SetTextColor(j.valid ? colorReturn : m_colorInvalid);
		pDC->TextOutW(x, y2, strPos);

		// 角度（夹爪显示开合百分比）
		CString strAngle;
		if (i == 6)
		{
			double gripOpen = 0.5;
			if (m_gripperOpenPos != m_gripperClosePos)
			{
				gripOpen = (double)(j.servoPos - m_gripperClosePos) / (double)(m_gripperOpenPos - m_gripperClosePos);
				gripOpen = std::max(0.0, std::min(1.0, gripOpen));
			}
			strAngle.Format(L"%.0f%%", gripOpen * 100.0);
		}
		else
		{
			strAngle.Format(L"%.1f", j.angleDeg);
		}
		pDC->SetTextColor(j.valid ? m_colorAngleArc : m_colorInvalid);
		pDC->TextOutW(x, y3, strAngle);
	}

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldFont);
}

