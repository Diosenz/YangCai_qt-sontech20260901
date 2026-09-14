# 环境检查报告

## 用户本机结果

用户本机已在 Visual Studio 2022 中完成 Qt Widgets + CMake 最小骨架的配置、生成和构建。

| 检查项 | 状态 | 检测值 | 说明 | 建议修复 |
| --- | --- | --- | --- | --- |
| cmake | PASS | VS2022 CMake integration | 已完成 CMake Configure 和 Generate。 | 无。 |
| cl.exe | PASS | MSVC 19.44 | MSVC C++ toolchain 可用。 | 无。 |
| msbuild | PASS | Visual Studio 2022 build environment | VS2022 已成功 Build。 | 无。 |
| ninja | PASS | Ninja generator | Generator 为 Ninja。 | 无。 |
| git | PASS | available / optional | Git 不是当前构建必需项。 | 无。 |
| QtPrefixPath | PASS | `C:\Qt\6.7.3\msvc2022_64` | Qt prefix 已确认。 | 无。 |
| Qt6Config.cmake | PASS | under `C:\Qt\6.7.3\msvc2022_64` | Qt 6 CMake package 可用。 | 无。 |
| qt-cmake.bat | PASS | under Qt `bin` directory | 可用于 Qt CMake 配置。 | 无。 |
| windeployqt.exe | PASS | under Qt `bin` directory | 后续部署时使用；当前开发构建不强制运行。 | 无。 |
| Qt DLL runtime path | PASS / preset-specific | `C:\Qt\6.7.3\msvc2022_64\bin` | `Qt-Debug` 已可运行；`qt-debug-opencv` 曾缺少该运行时 PATH，现已在 preset environment 和脚本中补充。 | 若仍缺 `Qt6Widgetsd.dll`，确认 preset environment 或当前进程 PATH。 |
| CMakeLists.txt | PASS | `i2r_rlc/CMakeLists.txt` exists | 工程文件存在。 | 无。 |
| Qt Widgets | PASS | present in CMakeLists.txt | 当前目标链接 Qt Core 和 Qt Widgets。 | 无。 |
| CMAKE_AUTOMOC | PASS | present in CMakeLists.txt | Qt QObject 自动 moc 已启用。 | 无。 |
| UTF-8 / MSVC `/utf-8` | PASS | present in CMakeLists.txt | MSVC 目标已添加 `/utf-8`。 | 无。 |
| OpenCV option | PASS | `I2R_RLC_ENABLE_OPENCV` | 第七阶段前置任务新增 OpenCV 可选依赖开关。 | 无。 |
| OpenCV_DIR | PASS | `E:\Libs\OPENCV3\opencv\build\x64\vc15\lib` | Codex 环境检查确认 `OpenCVConfig.cmake` 存在。 | 在 VS2022 preset 中选择 `qt-debug` 或手动设置 `OpenCV_DIR`。 |
| OpenCV DLL runtime path | PASS / preset-specific | `E:\Libs\OPENCV3\opencv\build\x64\vc15\bin` | Codex 环境检查确认目录存在；preset environment 已加入该目录。 | `$env:PATH="E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;$env:PATH"` |
| CMake configure | PASS | VS2022 Configure succeeded | 配置为 `Qt-Debug`。 | 无。 |
| CMake generate | PASS | VS2022 Generate succeeded | 生成目标为 `i2r_rlc.exe`。 | 无。 |
| CMake build | PASS | `[20/20] Linking CXX executable i2r_rlc.exe` | Build 输出显示全部生成已成功。 | 无。 |
| WrapVulkanHeaders | WARN | optional package missing | 这是 Qt Widgets 当前阶段的可选提示，可暂时忽略。 | 后续需要 Vulkan 相关能力时再处理。 |

## 已确认构建配置

| Item | Value |
| --- | --- |
| Qt prefix | `C:\Qt\6.7.3\msvc2022_64` |
| Generator | Ninja |
| MSVC version | 19.44 |
| CMake configuration | Qt-Debug |
| Target executable | `i2r_rlc.exe` |
| Build result | PASS |

## 如何重新运行

在用户本机 `i2r_rlc` 目录运行：

```powershell
.\scripts\check_environment.ps1 -QtPrefixPath "C:\Qt\6.7.3\msvc2022_64" -BuildDir "build" -Configuration "Debug" -RunBuildCheck
```

也可以继续使用 Visual Studio 2022 的 CMake Configure/Generate/Build 工作流。

## 标准本地构建脚本

当前已新增标准化 PowerShell 构建脚本，用于普通 PowerShell、Codex shell 或未手动加载 MSVC 环境的终端。脚本会自动查找 VS2022，并调用 `VsDevCmd.bat -arch=x64 -host_arch=x64` 后再配置和构建。

推荐先设置：

```powershell
$env:QT_MSVC2022_64="C:\Qt\6.7.3\msvc2022_64"
```

然后运行：

```powershell
.\scripts\build_vs2022_qt.ps1 -BuildDir "out/build/codex-debug" -Configuration "Debug"
```

也可以显式指定 Qt prefix：

```powershell
.\scripts\build_vs2022_qt.ps1 -QtPrefixPath "C:\Qt\6.7.3\msvc2022_64" -BuildDir "out/build/codex-debug" -Configuration "Debug"
```

普通 PowerShell 中 `cl.exe` 或 `msbuild` 不在 `PATH` 时，不应直接判定 VS2022 安装损坏。只要 VS2022 内部已能 Configure/Generate/Build，Codex 或普通 shell 应优先通过 `build_vs2022_qt.ps1` 加载开发环境再构建。

如果 PowerShell 执行策略禁止直接运行 `.ps1`，可使用进程级临时绕过：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build_vs2022_qt.ps1 -QtPrefixPath "C:\Qt\6.7.3\msvc2022_64" -BuildDir "out/build/codex-debug" -Configuration "Debug"
```

## Codex Shell 验证

本轮在 Codex 当前 PowerShell 中执行脚本时，首次直接运行 `.ps1` 被 PowerShell 执行策略拦截；使用 `-ExecutionPolicy Bypass` 后，脚本已成功找到 Qt prefix、Qt6Config.cmake、VS2022、VS CMake、VS Ninja 和 MSVC `cl.exe`。

当前 Codex shell 中 CMake configure 在 `out/build/codex-debug` 下连续超时，停在 CMake compiler try-compile 阶段；这与用户本机 VS2022 已成功 Configure/Generate/Build 的结果不冲突。建议用户在本机普通 PowerShell 或 Developer PowerShell for VS2022 中使用同一脚本复验。

## 配置测试目标

第四阶段收尾新增测试目标：

```text
i2r_rlc_config_tests
```

该目标只依赖 Qt Core，不依赖 Qt Widgets。测试覆盖 `SensorCalibrationParser` 的完整解析、关键字段映射、`More Info` 和日期行忽略、未知行容错、缺失字段失败、非法数值失败，以及 `ConfigRepository` 对 `config/calibration/SENSORPARA.txt` 和 `data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt` 的查找顺序。

在 VS2022 中可单独构建并运行 `i2r_rlc_config_tests`。命令行可在构建后运行：

```powershell
.\out\build\codex-debug\i2r_rlc_config_tests.exe
```

或使用 CTest：

```powershell
ctest --test-dir out/build/qt-debug -C Debug --output-on-failure
```

如果 Codex shell 继续出现 configure/build 超时，请在用户本机 VS2022 或 Developer PowerShell for VS2022 中运行测试目标。

## 离线图像测试目标

第五阶段新增测试目标：

```text
i2r_rlc_offline_camera_tests
```

该目标只依赖 Qt Core 和 Qt Gui，不依赖 Qt Widgets。测试覆盖无效目录、空目录、离线图片按文件名排序、`QImage` 读取、`frameId` 递增和 reset 后回到第一帧。

在 VS2022 中可单独构建并运行 `i2r_rlc_offline_camera_tests`。命令行可在构建后运行：

```powershell
.\out\build\codex-debug\i2r_rlc_offline_camera_tests.exe
```

或使用 CTest：

```powershell
ctest --test-dir out/build/qt-debug -C Debug --output-on-failure
```

## OpenCV 冒烟测试目标

第七阶段前置任务新增 OpenCV 3.4.x smoke test 目标：

```text
i2r_rlc_opencv_smoke_tests
```

该目标仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成。推荐在 VS2022 中选择 `qt-debug-opencv` preset，或手动设置：

```text
OpenCV_DIR=E:/Libs/OPENCV3/opencv/build/x64/vc15/lib
I2R_RLC_ENABLE_OPENCV=ON
```

测试只包含 OpenCV `core` / `imgproc` 头文件，创建小 `cv::Mat` 并调用 `cv::cvtColor` / `cv::mean`。它不读取真实图像，不实现焊缝检测算法，不访问旧工程。

如果运行时报找不到 `opencv_world34*.dll`，将以下目录加入当前进程 `PATH`：

```powershell
$env:PATH="E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;$env:PATH"
```

如果 Debug 模式提示找不到 `opencv_world34*d.lib` 或 `opencv_world34*d.dll`，这是 OpenCV 3.4.x Debug 库缺失或与 VS2022/v143 不匹配的问题。建议先尝试 Release，或后续使用 VS2022 重新编译 OpenCV 3.4.x Debug/Release；不要直接升级到 OpenCV 4.x。

## qt-debug-opencv 的运行时 PATH 修复

用户已确认 `Qt-Debug` preset 可以运行 `i2r_rlc.exe`，但 `Build Qt Debug + OpenCV` / `qt-debug-opencv` 运行时报缺少 `Qt6Widgetsd.dll`。这说明 Qt 安装本身可用，问题在于 OpenCV preset 的运行时 `PATH` 没有继承或包含 Qt Debug DLL 所在目录。

已补充 `qt-debug-opencv` preset 的运行环境：

```json
"environment": {
  "QTDIR": "D:/QT/6.7.3/msvc2022_64",
  "PATH": "D:/QT/6.7.3/msvc2022_64/bin;E:/Libs/OPENCV3/opencv/build/x64/vc15/bin;$penv{PATH}"
}
```

`qt-debug-no-opencv` 也补充了 Qt `bin`。构建脚本 `build_vs2022_qt.ps1` 在 `-RunAfterBuild` 前会重新将 Qt `bin` 和 OpenCV `bin` 加入当前进程 `PATH`，并检查 `Qt6Widgetsd.dll` / `Qt6Widgets.dll` 与 `opencv_world34*.dll` 是否存在。环境检查脚本也会区分编译期 CMake package 路径和运行期 DLL 搜索路径。

不建议把 DLL 复制到源码目录。如果用户选择修改全局 `PATH`，需要重启 VS2022。

Codex 当前 shell 已确认：

- `E:\Libs\OPENCV3\opencv\build\x64\vc15\lib\OpenCVConfig.cmake` 存在；
- `E:\Libs\OPENCV3\opencv\build\x64\vc15\bin` 存在；
- 构建脚本能找到 Qt、OpenCV、VS2022、Ninja 和 `cl.exe`；
- CMake configure 仍在当前 Codex shell 中超时，停在既有的 configure 阶段。最终构建结果仍应以用户 VS2022 为准。

用户已在 VS2022 中确认 `qt-debug-opencv` / `Build Qt Debug + OpenCV` preset 可用，`i2r_rlc_opencv_smoke_tests` 输出：

```text
PASS: OpenCV smoke test
OpenCV version: 3.4.5
Mean gray value: 22
```

`i2r_rlc.exe` 也已能在 VS2022 中运行并显示 UI，前一轮 Qt/OpenCV DLL runtime PATH 问题已修复。

## OpenCV 视觉预处理目标

第七阶段新增基础预处理链路：

```text
QImage -> cv::Mat -> grayscale -> mean/min/max -> WeldVisionPipelineResult
```

该阶段曾新增 `i2r_rlc_vision_opencv_tests` 覆盖 Qt 图像适配；后续架构整理已经移除 `OpenCvImageUtils` 和对应测试，`QImage -> cv::Mat` 的适配改由 `ui/UiController` 在调用 vision 算法前完成。当前 `vision` 模块不再暴露 Qt/QImage adapter。历史测试当时覆盖：

- 构造小尺寸 `QImage`；
- 转换为独立拥有内存的 `cv::Mat`；
- 灰度化；
- 计算 mean/min/max；
- 调用旧 `VisionPipeline::processFrame` 适配入口；
- 检查 `WeldVisionPipelineResult` 中的 `meanGrayValue`、`minGrayValue`、`maxGrayValue`、`imageChannels` 和 `preprocessingSucceeded`。

当前仍未实现焊缝检测、中心线提取、KCF/STC、坡口检测或真实纠偏控制。若运行时找不到 OpenCV DLL，仍需确保 `E:\Libs\OPENCV3\opencv\build\x64\vc15\bin` 位于当前进程 `PATH`。

