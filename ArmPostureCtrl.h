#pragma once

#include <afxwin.h>
#include <array>

// 机械臂姿态可视化控件
// 显示：1) 底座旋转俯视图  2) 机械臂平面侧视图  3) 当前角度数值
class CArmPostureCtrl : public CStatic
{
	DECLARE_DYNAMIC(CArmPostureCtrl)

public:
	CArmPostureCtrl();
	virtual ~CArmPostureCtrl();

	// 关节数据结构
	struct JointState
	{
		int servoPos = 500;       // 舵机位置 (0..1000) - 读回值
		int targetPos = -1;       // 目标位置 (-1=无数据)
		double angleDeg = 0.0;    // 对应物理角度 (度)
		bool valid = false;       // 是否有有效读回
	};

	// 设置关节状态 (J1..J5, 索引 1..5; J6 为夹爪)
	void SetJointState(int jointIndex, int servoPos, double angleDeg, bool valid);

	// 设置关节目标位置（发送值）
	void SetJointTargetPos(int jointIndex, int targetPos);

	// 设置夹爪状态 (J6)
	void SetGripperState(int servoPos, int openPos, int closePos, bool valid);

	// 设置连杆长度 (用于绘图比例)
	void SetLinkLengths(double L_base, double L_arm1, double L_arm2, double L_wrist);

	// 刷新显示
	void RefreshDisplay();

protected:
	DECLARE_MESSAGE_MAP()
	afx_msg void OnPaint();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);

private:
	// 绘制底座俯视图 (J1 旋转)
	void DrawTopView(CDC* pDC, const CRect& rect);

	// 绘制机械臂侧视图 (J2, J3, J4, 夹爪)
	void DrawSideView(CDC* pDC, const CRect& rect);

	// 绘制角度数值表
	void DrawAngleValues(CDC* pDC, const CRect& rect);

	// 辅助：角度转弧度
	static double DegToRad(double d) { return d * 3.14159265358979323846 / 180.0; }

private:
	// 关节状态 (0 unused, 1..5 为运动学关节, 6 为夹爪)
	std::array<JointState, 7> m_joints{};

	// 夹爪参数
	int m_gripperOpenPos = 640;
	int m_gripperClosePos = 100;

	// 连杆长度 (mm) - 用于绘图比例计算
	double m_L_base = 80.0;
	double m_L_arm1 = 100.0;
	double m_L_arm2 = 95.0;
	double m_L_wrist = 95.0;

	// 绘图颜色
	COLORREF m_colorBackground = RGB(30, 30, 35);
	COLORREF m_colorGrid = RGB(60, 60, 70);
	COLORREF m_colorBase = RGB(100, 100, 110);
	COLORREF m_colorArm1 = RGB(70, 130, 200);
	COLORREF m_colorArm2 = RGB(90, 170, 230);
	COLORREF m_colorWrist = RGB(110, 200, 180);
	COLORREF m_colorGripper = RGB(220, 150, 80);
	COLORREF m_colorGripperOpen = RGB(100, 200, 100);
	COLORREF m_colorGripperClose = RGB(200, 100, 100);
	COLORREF m_colorText = RGB(220, 220, 230);
	COLORREF m_colorTextDim = RGB(140, 140, 150);
	COLORREF m_colorAngleArc = RGB(255, 200, 80);
	COLORREF m_colorInvalid = RGB(120, 60, 60);
};

