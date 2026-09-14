# 环境要求

## 当前最小骨架需要的环境

`i2r_rlc` 当前是最小 Qt Widgets + CMake 骨架，需要以下环境：

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Windows |
| IDE | Visual Studio 2022 |
| C++ 工具链 | MSVC C++ toolchain |
| 构建系统 | CMake |
| Qt | Qt 6 MSVC 2022 64-bit，优先使用 Qt 6 |
| Qt 模块 | Qt Core、Qt Gui、Qt Widgets |
| vcpkg | 使用 manifest 模式管理第三方库 |
| vcpkg 默认依赖 | concurrentqueue、cppzmq / ZeroMQ、Eigen3、fmt、magic-enum、PCL 1.15.1、tinyspline |
| vcpkg feature 依赖 | `ibvs-yolo` 启用 CPU 版 ONNX Runtime 官方预编译包 |

建议 Qt 安装目录格式：

```text
C:\Qt\6.x.x\msvc2022_64
```

`CMAKE_PREFIX_PATH` 应指向 Qt prefix，而不是 `bin` 或 `lib` 子目录，例如：

```text
C:\Qt\6.x.x\msvc2022_64
```

## CMake 配置方式

推荐先设置 Qt prefix 环境变量，后续脚本和 preset 都会优先读取它：

```powershell
$env:QT_MSVC2022_64="C:\Qt\6.7.3\msvc2022_64"
```

可以使用普通 `cmake` 并显式传入 `CMAKE_PREFIX_PATH`：

```powershell
cmake -S . -B build -DCMAKE_PREFIX_PATH="C:\Qt\6.x.x\msvc2022_64"
cmake --build build --config Debug
```

也可以使用 Qt 提供的 `qt-cmake.bat`：

```powershell
C:\Qt\6.x.x\msvc2022_64\bin\qt-cmake.bat -S . -B build
cmake --build build --config Debug
```

建议在 “Developer PowerShell for VS 2022” 中运行上述命令，这样 `cl.exe`、`msbuild` 等 MSVC 工具更容易被正确加入 `PATH`。

## vcpkg manifest 依赖

当前项目根目录使用 `vcpkg.json` 进入 vcpkg manifest 模式。默认通过 vcpkg 管理 `cppzmq`、`eigen3`、官方 PCL 1.15.1 port 以及若干轻量工具库，用于后续 RobotSim 通信、定位、视觉伺服、矩阵计算和点云算法迁移。不要把旧工程或 RobotSim 中的大型依赖清单一次性复制进当前 Qt 项目。

IBVS YOLO 使用 manifest feature `ibvs-yolo` 按需启用 CPU 版 ONNX Runtime。普通 Qt/PCL/ZeroMQ 构建不启用该 feature，不会自动安装 ONNX Runtime。

使用 vcpkg 前，应设置 `VCPKG_ROOT`，并确保该目录下存在 `scripts\buildsystems\vcpkg.cmake`：

```powershell
$env:VCPKG_ROOT="C:\path\to\vcpkg"
```

当前共享 CMake preset 通过以下 cache variable 接入 vcpkg toolchain，并显式使用 `x64-windows` triplet：

```cmake
CMAKE_TOOLCHAIN_FILE=$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake
VCPKG_TARGET_TRIPLET=x64-windows
```

普通 preset 不启用项目 overlay port。Eigen3 和 PCL 都走 vcpkg 官方 port，因为官方 PCL 1.15.1 在安装阶段会校验 `pcl_*.pc` 文件，并需要 Eigen3 提供 `eigen3.pc`。因此不能继续使用关闭 `EIGEN_BUILD_PKGCONFIG` 的 Eigen3 overlay。

preset 不设置 `VCPKG_FORCE_SYSTEM_BINARIES=1`。该变量会强制 vcpkg 使用系统环境中的 7zip、CMake 等辅助工具；如果 Windows 当前 `PATH` 中没有 `7z.exe`，manifest 安装会在提取 7zip 阶段失败。保持该变量未设置时，vcpkg 可以按需下载并使用自己管理的辅助工具。

配置 preset 时，vcpkg 会根据根目录 `vcpkg.json` 安装第三方库到项目本地的 `vcpkg_installed/`，该目录不应提交到 Git。CMake 侧使用 `find_package(...)` 和导入 target，不手写第三方库 include 目录。仓库中残留的 legacy `vcpkg-ports/pcl` 不会捕获官方 PCL port，因为普通 preset 和脚本不设置 `VCPKG_OVERLAY_PORTS`；只有 `qt-debug-opencv-onnx` 指向单独的 `vcpkg-ports/onnxruntime` overlay port。

## PCL 1.15.1 定位算法依赖

PCL 通过 vcpkg 官方 port 管理，当前本机 `C:\dev\vcpkg\ports\pcl` 为 `1.15.1#1`。根目录 `vcpkg.json` 使用 `{ "name": "pcl", "default-features": false }`，不启用 `visualization`、`vtk`、`openni2`、`qt`、`cuda`、`apps`、`examples`、`tools` 等扩展 feature。

当前 PCL 是默认依赖，但不是完整桌面工具生态。它面向旧项目全局定位、局部定位涉及的本地点云算法，CMake 查找这些组件：

```text
pcl_common
pcl_io
pcl_kdtree
pcl_octree
pcl_search
pcl_filters
pcl_sample_consensus
pcl_features
pcl_registration
pcl_segmentation
pcl_surface
pcl_keypoints
```

当前明确不启用：

```text
pcl_visualization
VTK
OpenNI / OpenNI2
Qt-based PCL viewer
CUDA / GPU
PCL apps
PCL examples
PCL tools
```

PCL 仍会拉入 Boost、FLANN、Qhull、libpng、nanoflann 等点云算法所需依赖，构建时间和磁盘占用明显高于 Eigen3，但在不启用 `visualization/vtk` 的前提下，不应再出现 VTK 源码构建导致的十几 GB buildtrees 占用。当前项目只用拆分后的 PCL smoke tests 验证 `find_package(PCL)`、PCD IO、KD-tree、滤波、法向估计、RANSAC/分割、ICP、MLS surface、ISS keypoints 和 legacy nonlinear ICP 组合链路，不把 PCL 接入主程序或业务模块。

本机 vcpkg 下载部分依赖时可能遇到 Windows Schannel 证书吊销检查离线问题。当前 preset 因此通过 vcpkg asset source 指定外部下载命令 `curl.exe --ssl-no-revoke`，只影响该项目的 vcpkg 资源下载，不修改系统证书策略。

当前 PCL 验证目标：

- `qt-debug-no-opencv` 应能自动安装并找到官方 `pcl:x64-windows@1.15.1#1`。
- `vcpkg_installed/x64-windows/lib` 中应出现上述 PCL 算法模块对应的 `.lib`。
- `ctest` 应通过 `i2r_rlc_pcl_core_io_filter_tests`、`i2r_rlc_pcl_features_segmentation_tests`、`i2r_rlc_pcl_registration_tests`、`i2r_rlc_pcl_surface_keypoints_tests`、`i2r_rlc_pcl_localization_icp_compat_tests` 和现有测试。
- vcpkg 安装输出不应出现 VTK、OpenNI 或 PCL visualization。

## ZeroMQ / cppzmq RobotSim 通信依赖

ZeroMQ 通过 vcpkg 官方 `cppzmq` port 管理，当前项目 baseline 解析到 `cppzmq 4.11.0`，它会自动引入底层 `zeromq 4.3.5#2`。当前不直接在 manifest 中声明 `zeromq`，也不启用 `curve`、`sodium`、`draft`、`websockets` 等额外 feature。

该依赖用于后续当前 Qt 应用与 `RobotSim` 通信。`RobotSimCOMM` 使用 `<zmq.hpp>`、`<zmq_addon.hpp>`、ROUTER/DEALER multipart 模式，以及 `zmq::recv_multipart` / `zmq::send_multipart`。当前项目只新增 `i2r_rlc_zeromq_smoke_tests` 验证 `cppzmq` 能被 CMake 找到、链接和运行，并模拟 RobotSim 的三帧请求结构：client id、空分隔帧、JSON payload。

当前 ZeroMQ 验证目标：

- `qt-debug-no-opencv` 应能自动安装并找到 `cppzmq:x64-windows@4.11.0`。
- vcpkg 安装输出应包含传递依赖 `zeromq:x64-windows@4.3.5#2`。
- `ctest` 应通过 `i2r_rlc_zeromq_smoke_tests` 和现有测试。
- 当前不实现完整 RobotSim client，不把 cppzmq 链到主程序、UI 或现有业务模块。

## ONNX Runtime / IBVS YOLO 依赖

ONNX Runtime 通过 vcpkg manifest feature `ibvs-yolo` 管理，当前使用项目 overlay port 安装 Microsoft 官方预编译 Windows x64 CPU 包 `onnxruntime-win-x64-1.23.2.zip`。这样仍由 vcpkg 负责下载、校验、安装和提供 CMake package，但避免在本机从源码编译 ONNX Runtime。当前不启用 DirectML、CUDA、OpenVINO、TensorRT 或其他推理加速后端。该依赖只服务 `Ibvs` 模块中的可选 `IbvsYoloDetector`，不接入 UI、实时控制流程或其他模块。

专用 preset `qt-debug-opencv-onnx` 同时启用外部 OpenCV 3.4.x 和 ONNX Runtime：

```text
I2R_RLC_ENABLE_OPENCV=ON
I2R_RLC_ENABLE_ONNX_RUNTIME=ON
VCPKG_MANIFEST_FEATURES=ibvs-yolo
VCPKG_OVERLAY_PORTS=${sourceDir}/vcpkg-ports/onnxruntime
OpenCV_DIR=E:/Libs/OPENCV3/opencv/build/x64/vc15/lib
```

当前 ONNX Runtime 验证目标：

- `qt-debug-opencv-onnx` 应能自动安装并找到 overlay 管理的 ONNX Runtime CPU 预编译包。
- `i2r_rlc_onnxruntime_smoke_tests` 应能创建 `Ort::Env`，验证头文件、导入库和运行时 DLL 链路。
- `i2r_rlc_ibvs_yolo_smoke_tests` 使用 `C:\Feature_detection\onnx_workpiece\best.onnx` 加载模型，并对一张内存图像完成一次推理。
- YOLO 模型文件是运行数据，不复制进仓库；旧 DirectML/GPU 方案暂不迁入当前项目。

也可以在普通 PowerShell 中使用标准化构建脚本。脚本会自动查找 VS2022、调用 `VsDevCmd.bat -arch=x64 -host_arch=x64`，然后再执行 CMake/Ninja：

```powershell
.\scripts\build_vs2022_qt.ps1 -QtPrefixPath "C:\Qt\6.7.3\msvc2022_64" -VcpkgRoot "C:\path\to\vcpkg" -BuildDir "out/build/codex-debug" -Configuration "Debug"
```

如果当前 PowerShell 执行策略禁止运行 `.ps1`，可以使用进程级临时绕过，不会修改系统策略：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build_vs2022_qt.ps1 -QtPrefixPath "C:\Qt\6.7.3\msvc2022_64" -VcpkgRoot "C:\path\to\vcpkg" -BuildDir "out/build/codex-debug" -Configuration "Debug"
```

因此，普通 PowerShell 或 Codex shell 中直接找不到 `cl.exe`、`msbuild`，不一定代表本机环境损坏；它通常只表示当前 shell 尚未加载 VS2022 开发环境。优先使用 `build_vs2022_qt.ps1` 或 Visual Studio 2022 自带的 CMake 工作流。

## 当前阶段不需要的依赖

阶段 7 前置任务开始，OpenCV 3.4.x 作为可选构建依赖接入，用于后续 `vision` 模块离线预处理和旧算法迁移 smoke test。cppzmq 已作为 RobotSim 通信依赖交给 vcpkg manifest 管理，Eigen3 已作为轻量基础数学依赖交给 vcpkg manifest 管理，PCL 1.15.1 已作为定位算法无可视化依赖接入 smoke test，ONNX Runtime 已作为 IBVS YOLO 可选 feature 接入。除 OpenCV、cppzmq、Eigen3、PCL 算法版和 CPU 版 ONNX Runtime 外，当前仍不需要：

- Vimba SDK；
- Basler Pylon SDK；
- HALCON；
- OpenCascade；
- VTK；
- DX200 真实通信环境。

这些依赖不应被加入当前最小骨架，否则会增加配置和构建风险。尤其不要把 RobotSim 示例中的 OpenCascade、FCL、OMPL、libigl、TBB 等依赖直接迁入当前 Qt 应用，除非后续功能边界已经明确需要。

## OpenCV 3.4.x 要求

旧 MFC 工程使用 Visual Studio 2017 和 OpenCV 3.4.x。为了降低后续焊缝跟踪算法迁移时的行为差异，当前 Qt/CMake 重构工程继续使用 OpenCV 3.4.x，不升级到 OpenCV 4.x。当前本机接受并使用 `E:\Libs\OPENCV3` 下的 OpenCV 3.4.5。

OpenCV 暂不由 vcpkg 管理。当前仍通过 `OpenCV_DIR` 指向外部 OpenCV 3.4.x CMake package，以保持 legacy 算法兼容约束。

本机推荐配置：

```text
OpenCV root: E:\Libs\OPENCV3\opencv
OpenCV_DIR:  E:\Libs\OPENCV3\opencv\build\x64\vc15\lib
OpenCV bin:  E:\Libs\OPENCV3\opencv\build\x64\vc15\bin
```

`OpenCV_DIR` 必须指向包含 `OpenCVConfig.cmake` 的目录。不要通过 Visual Studio 属性页手工添加“附加包含目录/附加库目录”，本工程必须通过 CMake cache、环境变量或 preset 传入：

```powershell
$env:OPENCV_DIR="E:\Libs\OPENCV3\opencv\build\x64\vc15\lib"
```

或：

```powershell
cmake -S . -B out/build/qt-debug-opencv -DOpenCV_DIR="E:\Libs\OPENCV3\opencv\build\x64\vc15\lib" -DI2R_RLC_ENABLE_OPENCV=ON
```

运行 `i2r_rlc_opencv_smoke_tests` 时，如果提示找不到 `opencv_world34*.dll`，请将以下目录加入当前进程 `PATH`：

```powershell
$env:PATH="E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;$env:PATH"
```

## 运行时 DLL 路径要求

`CMAKE_PREFIX_PATH` 和 `OpenCV_DIR` 只解决编译期 `find_package` 与链接配置问题。运行 `i2r_rlc.exe` 或 `i2r_rlc_opencv_smoke_tests.exe` 时，Windows 仍需要通过运行时 `PATH` 找到 Qt 和 OpenCV DLL。

当前推荐运行时路径：

```text
Qt Debug DLL: C:\Qt\6.7.3\msvc2022_64\bin
OpenCV DLL:   E:\Libs\OPENCV3\opencv\build\x64\vc15\bin
```

如果 `Qt-Debug` preset 能运行，而 `qt-debug-opencv` preset 报找不到 `Qt6Widgetsd.dll`，说明 OpenCV preset 的运行环境 `PATH` 与可运行的 Qt preset 不一致，不代表 Qt 安装损坏。`qt-debug-opencv` 和 `qt-debug-no-opencv` preset 应通过 `environment` 补充 Qt `bin`；OpenCV preset 还应补充 OpenCV `bin`。

推荐使用以下方式之一设置运行时 DLL 搜索路径：

- 使用 `CMakePresets.json` 中 preset 的 `environment`；
- 使用 `build_vs2022_qt.ps1 -RunAfterBuild`，脚本会在运行前临时补充 Qt/OpenCV `bin` 到当前进程 `PATH`；
- 在当前 PowerShell 进程中手动设置 `PATH`。

不建议把 Qt 或 OpenCV DLL 复制到源码目录。如果用户选择修改系统全局 `PATH`，需要重启 VS2022 后当前 IDE 会话才会读取新的环境变量。

如果 OpenCV 3.4.x vc15 预编译库在 VS2022/v143 下出现链接、Debug/Release 库不匹配或运行时问题，建议后续使用 VS2022 重新编译 OpenCV 3.4.x，而不是直接升级到 OpenCV 4.x。

如需临时禁用 OpenCV：

```powershell
cmake -S . -B out/build/qt-debug-no-opencv -DI2R_RLC_ENABLE_OPENCV=OFF
```

## 后续阶段可能需要的依赖

| 依赖 | 计划用途 | 接入策略 |
| --- | --- | --- |
| Eigen3 | 定位、视觉伺服、矩阵和几何计算 | 当前已通过 vcpkg manifest 接入；先作为基础数学库，不迁入旧工程大依赖链。 |
| cppzmq / ZeroMQ | 当前 Qt 应用与 RobotSim 的 TCP multipart 通信 | 当前通过 vcpkg 官方 `cppzmq` port 接入，只做 smoke test，不实现完整 RobotSim client。 |
| OpenCV 3.4.x | 图像处理、离线图片测试、焊缝检测算法 | 第七阶段前置任务开始作为可选依赖接入；当前本机使用 OpenCV 3.4.5。 |
| Vimba SDK | 焊缝跟踪主相机 | 后续在 `camera::VimbaTrackingCameraBackend` 中隔离接入。 |
| Basler Pylon SDK | IBVS 外置相机 | 后续在 `camera::BaslerIbvsCameraBackend` 中隔离接入。 |
| HALCON | 旧算法兼容 | 仅当旧算法确实必须依赖时，作为 optional legacy adapter。 |
| PCL 1.15.1 | 点云基础数据结构、全局定位与局部定位算法预研 | 当前通过 vcpkg 官方 port 默认接入，关闭默认 features，只做 smoke test，不接入主业务。 |
| OpenCascade/VTK | 真实数字孪生、几何显示 | 仅当后续接入真实数字孪生时需要。 |
| YOLO/ONNX Runtime | IBVS 焊枪/工件 ROI 模型 | 当前通过 vcpkg feature `ibvs-yolo` 接入 CPU 版，只做 smoke test，不接实时流程。 |
| DX200 通信环境 | 真实机器人联调 | 必须在协议语义、dry-run、用户授权和安全验证完成后接入。 |

## 编码要求

为避免 Visual Studio 2022 / MSVC 下中文注释乱码：

- 新建或修改的文本文件使用 UTF-8 或 UTF-8 with BOM；
- C++ 源码优先使用 UTF-8 with BOM；
- CMake 中 MSVC 目标必须添加 `/utf-8`；
- 不创建 ANSI、GBK 或依赖系统区域设置的编码；
- 不在同一文件内混用编码；
- 中文注释使用 Doxygen 风格，字段单位应在注释或字段名中明确写出，例如 `mm`、`m`、`deg`、`pixel`。

当前 `CMakeLists.txt` 已为 `i2r_rlc` 目标添加：

```cmake
target_compile_options(i2r_rlc
    PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
)
```

当前 `.editorconfig` 已要求使用 UTF-8 with BOM，以便 VS2022 更稳定地识别中文注释。
