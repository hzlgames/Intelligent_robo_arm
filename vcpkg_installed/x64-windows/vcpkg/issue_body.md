Package: eigen3:x64-windows@3.4.1#1

**Host Environment**

- Host: x64-windows
- Compiler: MSVC 19.44.35221.0
- CMake Version: 3.31.10
-    vcpkg-tool version: 2025-12-16-44bb3ce006467fc13ba37ca099f64077b8bbf84d
    vcpkg-scripts version: 5d57f5a0a5 2025-12-24 (35 hours ago)

**To Reproduce**

`vcpkg install `

**Failure logs**

```
-- Using cached libeigen-eigen-3.4.1.tar.gz
-- Cleaning sources at C:/vcpkg/buildtrees/eigen3/src/3.4.1-0ff0cf177d.clean. Use --editable to skip cleaning for the packages you specify.
-- Extracting source C:/vcpkg/downloads/libeigen-eigen-3.4.1.tar.gz
-- Using source at C:/vcpkg/buildtrees/eigen3/src/3.4.1-0ff0cf177d.clean
-- Configuring x64-windows
-- Building x64-windows-dbg
-- Building x64-windows-rel
-- Fixing pkgconfig file: C:/vcpkg/packages/eigen3_x64-windows/lib/pkgconfig/eigen3.pc
-- Using cached msys2-mingw-w64-x86_64-pkgconf-1~2.4.3-1-any.pkg.tar.zst
-- Using cached msys2-msys2-runtime-3.6.2-2-x86_64.pkg.tar.zst
-- Using msys root at C:/vcpkg/downloads/tools/msys2/9272adbcaf19caef

```

**Additional context**

<details><summary>vcpkg.json</summary>

```
{
  "name": "smart-robotic-arm-mfc",
  "version-string": "0.1.0",
  "description": "MFC智能机械臂控制软件：运动学(Eigen) + 视觉叠加(OpenCV) + 标定优化(Ceres)",
  "dependencies": [
    "eigen3",
    {
      "name": "opencv4",
      "features": [
        "contrib"
      ]
    },
    "ceres"
  ]
}

```
</details>
