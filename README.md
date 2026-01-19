# 智能机械臂 (Smart Robotic Arm)

MFC 6自由度机械臂控制软件，集成运动学、视觉（OpenCV）与模拟仿真。

## 📦 环境配置 (Prerequisites)

1.  **Visual Studio 2022**
    *   安装负载：**使用 C++ 的桌面开发 (Desktop development with C++)**
    *   必选组件：**MFC for Latest v143 build tools (x86 & x64)**

> **注意**：确保 VS 安装了 MFC 组件，否则项目无法加载。

## 🏗️ 依赖管理 (Dependencies)

本项目使用 **vcpkg manifest** 模式自动管理依赖。`vcpkg.json` 文件定义了所需库：
*   `eigen3` (运动学计算)
*   `opencv4[contrib]` (计算机视觉)
*   `ceres` (非线性优化)

**你不需要手动下载或配置这些库路径。**

## 🚀 编译与运行 (Build & Run)

1.  在 Visual Studio 2022 中打开 `智能机械臂.sln`。
2.  设置解决方案配置为 **Debug** 或 **Release**，平台选择 **x64**。
3.  点击 **生成 (Build)** 或按 **F5** 运行。

> **初次编译提醒**：
> VS 会自动调用 vcpkg 下载并编译所有依赖库。这可能需要 **10-30分钟**，具体取决于网络状况。请耐心等待，不要中断。

## 📁 项目关键文件

*   `SeeAndFetchStateMachine.*`: 视觉抓取状态机逻辑
*   `MotionController.*`: 机械臂运动控制核心
*   `VisionService.*`: 视觉处理服务
*   `models/`: 包含 YOLO 和 MediaPipe 的 ONNX 模型文件
*   `guide_docs/`: [详细调试指南](guide_docs/)

---
*Created for 软件技术基础课程*
