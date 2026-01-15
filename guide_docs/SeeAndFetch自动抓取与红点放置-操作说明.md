# See&Fetch 自动抓取与红点放置 - 操作说明

更新时间：2025-12-31

> 本文面向“现场操作者/调参者”，目标是让你在不改代码的情况下完成：
> - **Detector 识别目标 → 自动接近 → 自动夹取**
> - **对准桌面红点（ColorTrack）→ 自动下降 → 自动放置（开爪）**
>
> 说明：
> - 自动流程运行期间会**锁定键盘 Jog**（避免手动与自动指令互相打架）。
> - 所有阈值/步长/次数均可在“诊断界面”实时调整，并可通过 `.ini` 导入导出持久化。

---

## 1. 界面入口与按钮

### 1.1 主界面（开始/取消/启用）
- **启用 See&Fetch**：勾选主界面上的 See&Fetch 相关开关（启用后状态机会在定时器 Tick 中运行）。
- **开始**：点击“开始”进入自动流程。
- **取消**：点击“取消”立即中止并回到 Idle。
- **急停**：若你工程里已接入 EStop（或后续扩展），应优先使用急停逻辑（比 Cancel 更强硬）。

> 建议：第一次调试先使用“模拟串口/模拟机械臂”，确认逻辑正确后再上真机。

---

## 2. 两套视觉用途（你当前的工作方式）

### 2.1 抓取目标：Detector
- **目标识别**：使用 `VisionService::Mode::Detector` 产生目标中心点 (u,v) 与检测框 bbox。
- **距离/到位判断**：支持 ArUco 深度或 bbox 面积（见第 4 节）。

### 2.2 放置终点：桌面红点（ColorTrack）
- **终点识别**：使用 `VisionService::Mode::ColorTrack` 识别桌面红色标记点，产生 (u,v) 与红色 blob 的跟踪框（trackBox/bbox）。
- **放置到位判断**：推荐使用 bbox 面积（红点越近，框面积通常越大）。

---

## 3. 诊断界面（参数调节与保存）

### 3.1 打开诊断页
在诊断面板中找到 **See&Fetch** 页面（用于集中调参）。

### 3.2 保存/加载/导入导出
- **加载**：从当前 Profile/配置读取到 UI。
- **保存并应用**：写回配置并广播“设置已导入”消息，使主流程热更新。
- **`.ini` 导入导出**：通过设置导入导出功能持久化（`[SeeAndFetch]` 相关分组）。

---

## 4. 抓取阶段（Approach）——如何在“无 ArUco”时判距

抓取阶段的关键目标是：**先对准（J1/J3/J4），再以 J2 步进方式接近，直到满足“可抓取距离”判据**。

### 4.1 RangeMode（判距模式）
在 See&Fetch 的 **Approach(接近)** 区域选择：
- **ArucoDepth**：只使用 `depthMm` 判距（需要 ArUco）。
- **BboxArea**：只使用 bbox 面积判距（推荐用于 Detector 抓取）。
- **Auto**：有 `depthMm` 用深度，没有则用 bbox。

### 4.2 BboxArea 判距（推荐给 Detector）
当选择 **BboxArea/Auto** 时：
- **GraspBoxAreaPx2**：绝对面积阈值（px²）。达到/超过即认为进入可抓取范围。
- **GraspBoxScale_milli**：相对倍数阈值（相对进入 Approach 时的“基准面积”）。例如 2000=2.0x；填 0 表示禁用倍数门限。
- **BoxStableFrames / BoxAreaMaxJumpPx2**：稳定帧数与跳变过滤（抑制抖动误判）。
- **RequireDetector**：建议抓取目标时勾选，避免把其它模式的框当成抓取目标。

### 4.3 多次尝试（Approach 重试）
- **MaxAdvanceSteps**：最大推进步数（防止无限推进）。
- **MaxAttempts / RetryRetreatSteps**：
  - 推进超过最大步数仍未达到判据 → 自动进入 Retreat 回退 `RetryRetreatSteps` 步 → 回到 Track 再次对准 → 再推进。

---

## 5. 放置阶段（Place）——桌面红点终点

### 5.1 Place.Mode
在 **Place(放置终点: 桌面红点)** 区域选择：
- **SimpleOpen**：保持旧行为（到 Place 直接开爪）。
- **RedDotVisual**：启用“红点终点放置”流程（推荐）。

### 5.2 RedDotVisual 流程分解（便于你调参定位问题）
RedDotVisual 内部按 3 个小阶段运行：
- **(A) 对准红点**：复用 FindObject 逻辑，用 J1/J3/J4 将红点居中。
- **(B) 下降**：以 J2 固定步进向下移动，直到满足“到位判据”。
- **(C) 开爪并返回**：开爪后进入 ReturnHome。

### 5.3 红点识别模式（VisionMode）
- **VisionMode**：默认填 **3**（ColorTrack）。
  - 0=Auto，1=BrightestPoint，2=Aruco，3=ColorTrack，4=Detector，5=HandSticker，6=HandLandmarks

### 5.4 红点对准稳定
- **CStable**：对准红点后需要连续稳定多少帧才进入下降。
- 对准的“像素死区”仍使用 FindObject 的 **Deadband(px)**。

### 5.5 下降到位判据（RangeMode）
Place 的 RangeMode 与 Approach 类似：
- **BboxArea（推荐）**：用红点 bbox 面积判断到位
- **ArucoDepth**：用深度判断到位（若放置区也有 ArUco）
- **Auto**：两者任一满足即可

参数含义：
- **PlaceBoxAreaPx2 / PlaceBoxScale_milli / BoxStableFrames / BoxAreaMaxJumpPx2**：与抓取阶段同理，只是用于“放置到位”。
- **PlaceDepthMm**：深度到位阈值（仅 ArUcoDepth/Auto 且有深度时有效）。

### 5.6 下降步进与方向
核心参数：
- **MaxDownSteps**：最多下降多少步。
- **J2DownStepDeg_milli**：每次下降步长（毫度；2000=2.0°）。
- **SignJ2Down**：下降方向（只能 +1 或 -1）。如果方向反了（越降越抬），把符号改成相反即可。

### 5.7 Place 重试（很重要）
- **MaxAttempts / RetryRetreatSteps**
  - 下降到 `MaxDownSteps` 仍未到位 → 自动回退 `RetryRetreatSteps` 步 → 回到“对准红点” → 再次下降。

### 5.8 LiftRetreat（放置前/后抬离）
- **RetreatSteps**：用于“夹取后先抬离目标/离开桌面”的回退步数（沿 J2 反向）。

---

## 6. 推荐的起步参数（保守、安全优先）

> 下列仅作为“起步值”，你需要按现场相机视角、红点尺寸、机械臂姿态与桌高做微调。

### 6.1 抓取（Detector + BboxArea）
- RangeMode：**BboxArea**
- RequireDetector：**勾选**
- GraspBoxAreaPx2：先从 **30000** 试起（看框面积到多少时你觉得“差不多能抓”）
- GraspBoxScale_milli：**0**
- BoxStableFrames：**3**
- BoxAreaMaxJumpPx2：**20000**
- MaxAdvanceSteps：**60**
- MaxAttempts：**3**
- RetryRetreatSteps：**8**
- J2AdvanceStepDeg：**2.0°**（对应 2000 mdeg）

### 6.2 放置（RedDotVisual + ColorTrack + BboxArea）
- Place.Mode：**RedDotVisual**
- VisionMode：**3（ColorTrack）**
- Place.RangeMode：**BboxArea**
- PlaceBoxAreaPx2：先从 **24000** 试起
- PlaceBoxScale_milli：**0**
- BoxStableFrames：**3**
- BoxAreaMaxJumpPx2：**20000**
- MaxDownSteps：**30**
- J2DownStepDeg：**2.0°**（对应 2000 mdeg）
- SignJ2Down：根据现场试一次决定正负
- MaxAttempts：**2**
- RetryRetreatSteps：**8**

---

## 7. 常见问题排查（快速定位）

### 7.1 红点能识别但放置不下降
- 检查 Place.Mode 是否为 **RedDotVisual**
- 检查 VisionMode 是否为 **3（ColorTrack）**
- 检查 FindObject 的 Deadband 是否过小导致一直“对不准”（无法进入下降阶段）

### 7.2 红点对准了，但下降一直不停/或很快 Abort
- **到位阈值不合适**：
  - BboxArea：`PlaceBoxAreaPx2` 设得太大 → 永远到不了
  - Depth：`PlaceDepthMm` 设得太小 → 永远到不了
- **MaxDownSteps 太小**：几步就耗尽导致频繁重试或 Abort

### 7.3 下降方向反了
- 修改 **SignJ2Down**（+1 ↔ -1）

### 7.4 抓取阶段误判“到位”太早/太晚
- 调整 **GraspBoxAreaPx2**（或启用/调整 **GraspBoxScale_milli**）
- 增大 **BoxStableFrames** 或减小 **BoxAreaMaxJumpPx2** 以抑制抖动误判


