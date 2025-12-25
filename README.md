# 智能机械臂控制软件 (Smart Robotic Arm Control)

[![Platform](https://img.shields.io/badge/Platform-Windows-blue.svg)](https://www.microsoft.com/windows)
[![IDE](https://img.shields.io/badge/IDE-Visual%20Studio%202022-purple.svg)](https://visualstudio.microsoft.com/)
[![Framework](https://img.shields.io/badge/Framework-MFC-red.svg)](https://learn.microsoft.com/en-us/cpp/mfc/mfc-desktop-applications)

这是一个基于 **MFC (Microsoft Foundation Classes)** 开发的 6 自由度智能机械臂控制与诊断软件。该软件专为新手联调与多人协作设计，支持无硬件模式下的串口模拟、摄像头预览适配以及统一的参数共享机制。

## **直接上手教程**
**点击.sln调试，直接查看[调试指南](guide_docs/调试流程.md)跟着来即可**

## 🌟 核心功能

- **串口诊断与模拟**：
  - 支持真实串口 (COM) 与 **内置模拟串口** 切换。
  - 具备协议打包/解析能力（`0x03` 写舵机，`0x15` 读位置）。
  - 提供单舵机步进调试面板，支持现场设置安全限位 (最小/最大)。
- **摄像头预览 (Media Foundation)**：
  - 基于原生 Windows Media Foundation 实现，无需 OpenCV 等第三方库。
  - 支持 **镜像、旋转 (90/180/270°)、十字线与九宫格** 叠加显示。
  - 具备一键截图功能，方便对比安装位置。
- **运动控制与联动 (已全界面汉化)**：
  - 实现逻辑关节 (**底座、肩、肘、腕、云台**) 与物理舵机 ID 的映射。
  - 支持多关节同步执行、**一键归位**。
  - 内置演示脚本回放功能，支持循环演示动作。
- **参数共享 (多人协作专用)**：
  - **导出/导入设置**：支持将所有硬件标定（限位、映射、节流、摄像头方向）导出为 `.ini` 文件，方便通过 Git 与队友同步。

## 🛠 开发与运行环境

- **操作系统**：Windows 10/11
- **开发工具**：Visual Studio 2022（安装“使用 C++ 的桌面开发”+ MFC 组件）
- **编译配置**：`Debug | x64` (推荐) 或 `Release | x64`
- **依赖管理**：使用 **vcpkg manifest**（项目根目录 `vcpkg.json`）自动安装/引用第三方库：
  - `eigen3`（运动学/几何）
  - `opencv4[contrib]`（视觉：Aruco/色块/DNN/HUD 等）
  - `ceres`（标定优化）

## 🚀 快速上手

1.  **编译运行**：用 VS2022 打开 `智能机械臂.sln`，按 `F5` 运行。
    - 第一次编译可能会自动安装 vcpkg 依赖（时间取决于网络与电脑性能）。
2.  **开启诊断**：主界面点击 **“诊断(D)...”** 进入调试中心。
3.  **详细指南**：
    - [调试流程详解 (新手必读)](guide_docs/调试流程.md)：从模拟串口到真机联调。
    - [机械参数标定指南 (定位移动必备)](guide_docs/机械标定.md)：测量 $L_{base}, L_{arm}$ 等核心物理参数。
4.  **无硬件调试**：在 `串口` 页勾选“使用模拟串口”，点击“连接”，即可发送指令并观察模拟回包。
5.  **导入参数**：如果你是第一次拉取本项目，请点击主界面的 **“导入参数...”** 并选择仓库中的 `.ini` 文件，以获得最新的标定结果。
6.  **确认 OpenCV 已生效**：运行后看主界面右侧 `VS:` 状态行：
    - `cv=Y`：OpenCV 已进入编译环境（Aruco/色块/Detector 等可用）
    - `cv=N`：说明 OpenCV 头文件未被编译器找到，按调试指南的“让 OpenCV 真正生效”章节排查

## 📂 项目结构

- `ArmCommsService.*`: 核心通信层，管理 TX/RX 队列与节流。
- `MotionController.*`: 运动逻辑层，负责关节映射与安全检查。
- `SettingsIo.*`: 参数导出/导入模块。
- `*DiagPage.*`: 各功能诊断页 UI 实现。
- `Reference/`: 包含硬件协议说明与技术参考文档。
- `guide_docs/`: [详细的调试与标定指南目录](guide_docs/)。
- `progress/`: 开发阶段成果记录。

## 🤝 协作建议

在提交代码到 GitHub 之前，建议先在主界面点击 **“导出参数...”**。生成的配置文件可以帮助你的队友快速同步机械臂的物理参数，避免由于硬件安装差异导致的撞墙或超限问题。

### 🧩 队友网络慢怎么办（QQ 传输离线加速）

如果队友安装 vcpkg 依赖速度很慢，可以把你电脑上已下载/已安装的部分通过 QQ 传输给队友加速。推荐做法已写在：
- `guide_docs/调试流程.md` → “网络慢怎么办？（支持你用 QQ 传输离线加速）”

---
*Created by [hzlgames](https://github.com/hzlgames)*
