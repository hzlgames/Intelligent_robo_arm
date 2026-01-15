#pragma once

#include <afxdlgs.h>

#include <string>
#include <vector>

// See&Fetch（自动视觉追踪/接近/抓取/放置）诊断页：
// - 所有阈值/步长/边界参数可在此编辑并保存到 Profile（支持 ini 导入导出）。
// - 保存后会广播 WM_APP_SETTINGS_IMPORTED，让主界面/状态机即时刷新。
// - [新增] 支持垂直滚动，以适应大量参数配置。
class CSeeAndFetchDiagPage : public CPropertyPage
{
	DECLARE_DYNAMIC(CSeeAndFetchDiagPage)

public:
	CSeeAndFetchDiagPage();
	virtual ~CSeeAndFetchDiagPage();

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);

	afx_msg void OnBnClickedLoad();
	afx_msg void OnBnClickedSaveApply();
	afx_msg LRESULT OnSettingsImported(WPARAM wParam, LPARAM lParam);

	// 滚动支持
	afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnSize(UINT nType, int cx, int cy);

	DECLARE_MESSAGE_MAP()

private:
	void LoadFromProfileToUi();
	bool SaveFromUiToProfile(std::wstring& outWhy);

	int GetIntFromEditId(int id, int fallback) const;
	void SetIntToEditId(int id, int v);
	bool GetCheckFromId(int id) const;
	void SetCheckToId(int id, bool on);

	// 滚动相关
	struct ChildInfo
	{
		HWND hwnd;
		CRect rcOriginal; // 原始位置（相对于对话框客户区）
	};
	std::vector<ChildInfo> m_children;
	int m_totalHeight = 0;
	int m_scrollPos = 0;

	void CaptureChildPositions();
	void ApplyScroll(int newScrollPos);
	void UpdateScrollBars();
};
