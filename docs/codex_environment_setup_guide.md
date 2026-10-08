# Codex 跨机器环境配置与运行指导

## 目标和结论

本文档是给另一台 Windows 电脑上的 Codex 读取的总入口。目标是让 Codex 能按当前仓库的真实配置完成环境检查、依赖安装、CMake 配置、VS2022 生成、测试和常见问题处理，最终能够运行本项目主程序 `i2r_rlc.exe`。

结论先写清楚：在新电脑已经具备 Windows、网络、磁盘空间、必要安装包或下载权限、以及用户允许安装软件/改环境变量的前提下，Codex 可以按本文档完成配置和验证。Codex 不能凭空绕过缺失的 VS2022/Qt/OpenCV 安装器、公司网络限制、管理员权限、硬件 SDK 授权、真实相机或 RobotSim 场景操作；遇到这些外部条件时，Codex 应停止猜测，向用户报告缺哪一项和下一步要用户确认什么。

当前推荐的最小可运行环境是：

| 项目 | 推荐值 |
| --- | --- |
| 操作系统 | Windows 10/11 x64 |
| IDE / 编译器 | Visual Studio 2022，安装 Desktop development with C++ |
| 构建入口 | Visual Studio 2022 CMake 工作流优先 |
| CMake generator | Ninja |
| Qt | `C:\Qt\6.7.3\msvc2022_64` |
| vcpkg | 设置 `VCPKG_ROOT`，例如 `C:\dev\vcpkg` |
| vcpkg triplet | `x64-windows` |
| OpenCV | 外部 OpenCV 3.4.3，`C:\opencv\build\x64\vc15\lib` |
| 主 preset | `qt-debug` |
| 主程序 target | `i2r_rlc` |
| 主程序 exe | `out\build\qt-debug\i2r_rlc.exe` |

不要把 `oldproject/` 或 `RobotSim/` 当作要编译的当前项目模块。它们是只读参考。

## Codex 能否独立完成的边界

Codex 可以完成：

- 读取仓库文档、`CMakePresets.json`、`vcpkg.json` 和 CMake 文件；
- 检查 VS2022、Qt、vcpkg、7-Zip、OpenCV 是否存在；
- 在已有权限下设置当前进程环境变量；
- 通过 VS2022 CMake 工作流触发 configure/build；
- 在命令行兜底时运行 configure、CTest 和 smoke test；
- 诊断 DLL 缺失、OpenCV 版本错误、vcpkg 下载失败、`.ninja_lock`、C 盘空间不足和残留进程；
- 给出缺失软件、路径、权限、网络、硬件的明确修复清单。

Codex 不能默认独立完成：

- 静默安装 Visual Studio、Qt、OpenCV、Basler pylon、Vimba X 等带 GUI 或许可证的软件；
- 修改系统级环境变量后让已经打开的 VS 立即生效；
- 在没有网络或代理权限时完成 vcpkg 下载；
- 在没有管理员权限时清理被系统或 IDE 锁住的目录；
- 在没有真实硬件时验证相机采集；
- 在 RobotSim 未加载正确场景时证明 Qt 侧协议错误。

因此，新电脑配置的成功标准不是“Codex 不需要用户参与”，而是“Codex 能按本文档快速判断当前机器缺什么、配置到哪一步、第一条失败原因是什么，并把可自动完成的部分做完”。

## 历史已验证事实

以下是本项目之前实际跑通过或踩过的环境事实，新电脑配置时应优先复用这些经验：

| 项目 | 已验证结论 |
| --- | --- |
| VS2022 GUI 构建 | 在本机通过 Computer Use 操控已打开的 `qt-sontech - Microsoft Visual Studio`，快捷键触发过“全部生成”，输出窗口显示 `全部生成 已成功`。 |
| VS 快捷键 | `F7` 可作为“全部生成/生成当前配置”的入口；`Alt+B` 在本机曾直接触发“全部生成”，不要把它当作只展开菜单。 |
| 主 Debug build tree | `out\build\qt-debug` 是当前应优先保留和复用的主构建目录。 |
| 全量测试证据 | 之前非沙箱环境执行 `ctest --test-dir out/build/qt-debug -C Debug --output-on-failure` 曾达到 `26/26 passed`。这个数字是历史证据，不保证当前代码永远仍是 26 个测试。 |
| OpenCV 策略 | 项目继续使用外部 OpenCV 3.4.3，默认路径 `C:\opencv\build\x64\vc15\lib` 和 `C:\opencv\build\x64\vc15\bin`；不要自动升级到 OpenCV 4。 |
| 相机 SDK 策略 | Basler/Vimba X 是可选 vendor camera 后端，默认关闭；未启用真实相机后端时，不要求安装 pylon 或 Vimba X。 |
| ONNX 策略 | `qt-debug-opencv-onnx` 历史上是重依赖路径，主要耗时在 vcpkg restore。当前 `CMakePresets.json` 不把它作为默认共享 preset，新电脑首验不要启用 ONNX。 |
| C 盘清理 | 历史上应优先保留 `qt-debug` 和 `.vs`，先删历史 build tree；只有用户明确接受更慢重建时才删 `.vs` 或整个 `out\build`。 |

如果新电脑上的结果和上表不一致，以新电脑当前事实为准，但 Codex 必须在汇报中说明差异，例如“当前 CMakePresets 已无旧 `qt-debug-opencv` 名称，所以按 `qt-debug` 执行”。

## Codex 开始前必须读取

Codex 在新机器上接手本仓库后，先读取以下文件，再执行任何配置或修改：

```text
AGENTS.md
docs/codex_environment_setup_guide.md
docs/environment_requirements.md
docs/environment_check_report.md
docs/camera_commcamera_vendor_integration.md
docs/robotsim_minimal_joint_test_2026_06_25.md
CMakePresets.json
vcpkg.json
```

读取后 Codex 应先输出一个短结论，说明当前任务属于哪一种：

| 类型 | 动作 |
| --- | --- |
| 只配置环境 | 不改源码，只检查/安装/配置/构建/测试。 |
| 修复环境脚本 | 先读脚本和当前 preset，确认旧名称后再最小修改脚本。 |
| 修复源码编译错误 | 先读 `docs/I2R_Cpp_Codex_Rules.md` 和相关模块，再改代码。 |
| Git 分支同步 | 先读 `docs/collaboration_git_workflow.md`，拉取、冲突、验证、推送分阶段处理。 |

如果任务涉及修改 C++，再读取：

```text
docs/I2R_Cpp_Codex_Rules.md
```

如果任务涉及 `Twin` / `integrate` / `master` / `dev` 分支同步、合并或推送，再读取：

```text
docs/collaboration_git_workflow.md
```

## 安装顺序

安装前先确认磁盘空间。PCL、Boost、vcpkg buildtrees、OpenCV 和 VS 组件会占用较多 C 盘空间：

```powershell
Get-PSDrive C
```

建议 C 盘至少预留 30 GB 可用空间。低于 15 GB 时，Codex 应先提示用户清理空间，不要直接启动 vcpkg/PCL 大依赖恢复。

### 1. Visual Studio 2022

安装 Visual Studio 2022，并勾选：

- Desktop development with C++；
- MSVC v143 x64/x86 build tools；
- Windows 10/11 SDK；
- CMake tools for Windows；
- Ninja，通常随 VS CMake 工具一起可用。

验证方式：

```powershell
Get-Command cl -ErrorAction SilentlyContinue
Get-Command cmake -ErrorAction SilentlyContinue
Get-Command ninja -ErrorAction SilentlyContinue
```

普通 PowerShell 找不到 `cl.exe` 不一定代表 VS 安装错误。优先在 VS2022 中打开仓库，或使用 Developer PowerShell for VS 2022 验证。

更可靠的 VS 安装检查：

```powershell
$vswhere1="${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vswhere2="$env:ProgramFiles\Microsoft Visual Studio\Installer\vswhere.exe"
Test-Path $vswhere1
Test-Path $vswhere2
```

如果存在 `vswhere.exe`，可查询 VS2022 安装路径：

```powershell
& $vswhere1 -latest -version "[17.0,18.0)" -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
```

然后确认：

```text
<VS安装目录>\Common7\Tools\VsDevCmd.bat
```

存在。

### 2. Qt 6.7.3

安装 Qt MSVC 2022 64-bit kit，推荐路径固定为：

```text
C:\Qt\6.7.3\msvc2022_64
```

必须存在：

```text
C:\Qt\6.7.3\msvc2022_64\lib\cmake\Qt6\Qt6Config.cmake
C:\Qt\6.7.3\msvc2022_64\bin\Qt6Widgetsd.dll
C:\Qt\6.7.3\msvc2022_64\bin\windeployqt.exe
```

可设置环境变量：

```powershell
$env:QT_MSVC2022_64="C:\Qt\6.7.3\msvc2022_64"
$env:QTDIR="C:\Qt\6.7.3\msvc2022_64"
```

### 3. vcpkg

准备 vcpkg，例如：

```text
C:\dev\vcpkg
```

设置：

```powershell
$env:VCPKG_ROOT="C:\dev\vcpkg"
```

必须存在：

```text
C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake
```

本项目使用 manifest 模式，依赖来自根目录 `vcpkg.json`：

- `concurrentqueue`
- `cppzmq`
- `eigen3`
- `fmt`
- `magic-enum`
- `tinyspline`
- `pcl`，关闭默认 features

因为 `CMakePresets.json` 设置了：

```text
VCPKG_FORCE_SYSTEM_BINARIES=1
```

新机器还应安装 7-Zip，并确保 `7z.exe` 在 `PATH` 中：

```text
C:\Program Files\7-Zip
```

检查命令：

```powershell
Get-Command 7z.exe -ErrorAction SilentlyContinue
Test-Path "$env:VCPKG_ROOT\downloads"
Test-Path "$env:VCPKG_ROOT\buildtrees"
Test-Path "$env:VCPKG_ROOT\packages"
```

如果 `downloads/buildtrees/packages` 不存在，vcpkg 通常会自动创建。若创建失败并出现 `Access is denied`，说明当前 shell 对 `VCPKG_ROOT` 没有足够写权限，Codex 应要求用户换到有权限的位置或授权一次性修复权限，不要改仓库源码。

### 4. OpenCV 3.4.3

当前项目要求 OpenCV 3.4.x，不接受 OpenCV 4.x。推荐路径：

```text
C:\opencv
C:\opencv\build\x64\vc15\lib
C:\opencv\build\x64\vc15\bin
```

必须存在：

```text
C:\opencv\build\x64\vc15\lib\OpenCVConfig.cmake
C:\opencv\build\x64\vc15\bin\opencv_world343*.dll
```

可设置：

```powershell
$env:OPENCV_DIR="C:\opencv\build\x64\vc15\lib"
$env:PATH="C:\opencv\build\x64\vc15\bin;$env:PATH"
```

检查 OpenCV 版本和 DLL：

```powershell
Test-Path "C:\opencv\build\x64\vc15\lib\OpenCVConfig.cmake"
Get-ChildItem "C:\opencv\build\x64\vc15\bin" -Filter "opencv_world343*.dll" -ErrorAction SilentlyContinue
```

如果只有 OpenCV 4 的目录或 `opencv_world4*.dll`，不要把它接进本项目。当前迁移阶段要求 OpenCV 3.4.3，以减少旧算法行为差异。

如果新机器暂时没有 OpenCV，可先使用 `qt-debug-no-opencv` 做基础 Qt/vcpkg 验证，但完整项目当前默认路径应使用 `qt-debug`。

### 5. 可选真实相机 SDK

默认构建不启用真实相机 vendor 后端。只有需要真实相机 SDK smoke test 时才安装：

| SDK | 默认路径 / 变量 |
| --- | --- |
| Basler pylon | `C:\Program Files\Basler\pylon` |
| pylon CMake package | `C:\Program Files\Basler\pylon\Development\CMake` |
| pylon runtime | `C:\Program Files\Basler\pylon\Runtime\x64` |
| Allied Vision Vimba X | `C:\Program Files\Allied Vision\Vimba X` |
| Vimba API | `C:\Program Files\Allied Vision\Vimba X\api` |

相关 CMake 开关默认关闭：

```text
I2R_RLC_ENABLE_VENDOR_CAMERAS=OFF
I2R_RLC_ENABLE_CAMERA_ALLIED=OFF
I2R_RLC_ENABLE_CAMERA_BASLER=OFF
```

启用 Basler 时可能需要：

```powershell
$env:PYLON_ROOT="C:\Program Files\Basler\pylon"
$env:GENICAM_GENTL64_PATH="C:\Program Files\Basler\pylon\Runtime\x64"
```

启用 Allied 时可能需要：

```powershell
$env:VIMBA_X_HOME="C:\Program Files\Allied Vision\Vimba X"
```

### 6. 可选 ONNX Runtime / IBVS YOLO

默认不启用 ONNX Runtime：

```text
I2R_RLC_ENABLE_ONNX_RUNTIME=OFF
```

只有验证 IBVS YOLO 时才启用：

```text
I2R_RLC_ENABLE_ONNX_RUNTIME=ON
VCPKG_MANIFEST_FEATURES=ibvs-yolo
VCPKG_OVERLAY_PORTS=${sourceDir}/vcpkg-ports/onnxruntime
```

YOLO smoke test 默认模型路径是：

```text
C:\Feature_detection\onnx_workpiece\best.onnx
```

该模型是运行数据，不提交到仓库。新机器没有模型时不要启用 YOLO 测试。

## CMakePresets 当前事实

当前 `CMakePresets.json` 有以下 configure presets：

| Preset | Build dir | OpenCV | ONNX | 用途 |
| --- | --- | --- | --- | --- |
| `qt-debug` | `out/build/qt-debug` | ON | OFF | 当前主 Debug 配置 |
| `qt-release` | `out/build/qt-release` | ON | OFF | Release 配置 |
| `qt-debug-no-opencv` | `out/build/qt-debug-no-opencv` | OFF | OFF | 缺 OpenCV 时的基础诊断配置 |

当前没有 `qt-debug-opencv` preset。旧文档或旧脚本里出现该名称时，应按当前 `CMakePresets.json` 修正为 `qt-debug`。

`qt-debug` 关键配置：

```text
CMAKE_PREFIX_PATH=C:/Qt/6.7.3/msvc2022_64
CMAKE_TOOLCHAIN_FILE=$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake
VCPKG_TARGET_TRIPLET=x64-windows
OpenCV_DIR=C:/opencv/build/x64/vc15/lib
I2R_RLC_ENABLE_OPENCV=ON
I2R_RLC_ENABLE_ONNX_RUNTIME=OFF
PATH=C:/Qt/6.7.3/msvc2022_64/bin;C:/opencv/build/x64/vc15/bin;$penv{PATH}
```

## 推荐构建流程

### 首选：Visual Studio 2022

Codex 需要生成或验证 MSVC/Ninja 目标时，优先使用 Visual Studio 2022，而不是普通 PowerShell 裸跑完整构建。

步骤：

1. 在 VS2022 中打开仓库根目录 `C:\newSontech\qt-sontech`。
2. 等待 VS CMake Configure 完成。
3. 在 VS 顶部 CMake 配置下拉框选择 `qt-debug`。
4. 触发“全部生成”，本机通常可用 `F7`。
5. 观察输出窗口，确认类似“全部生成 已成功”的最终状态。
6. 启动项选择 `i2r_rlc.exe`，不要选择 `.lib` 模块或测试 target。

如果需要让 Codex 操作 VS，Codex 应主动读取 VS 当前配置和输出窗口，不要只根据文件名猜测当前配置。

VS 输出窗口判定规则：

- 看到 `Configuring done` / `Generating done` 只能说明 CMake 配置生成完成，不等于项目已编译；
- 看到 `[x/y] Linking CXX executable i2r_rlc.exe` 或 VS 的 `全部生成 已成功`，才说明主程序构建成功；
- 如果输出窗口超过约 2 分钟没有新内容，Codex 应转入进程/锁文件/磁盘/网络诊断；
- 如果 VS 当前配置不是 `qt-debug`，先在 VS 配置下拉框确认和切换，不要直接假设。

### 命令行仅作为兜底

只有用户明确要求命令行，或 Computer Use / VS 操作不可用时，才用 Developer PowerShell for VS 2022 或已加载 `VsDevCmd.bat` 的 shell。

配置命令示例：

```powershell
$env:VCPKG_ROOT="C:\dev\vcpkg"
$env:Path="C:\Program Files\7-Zip;C:\Qt\6.7.3\msvc2022_64\bin;C:\opencv\build\x64\vc15\bin;$env:Path"
cmake --preset qt-debug
```

构建命令示例：

```powershell
cmake --build --preset qt-debug --config Debug
```

测试命令示例：

```powershell
ctest --test-dir out\build\qt-debug -C Debug --output-on-failure
```

普通 PowerShell 不要反复裸跑完整 MSVC/Ninja 构建。如果出现长时间无输出，应先诊断进程和锁文件。

如果必须在普通 PowerShell 中临时加载 VS 开发环境，可用以下模式。注意路径要按实际 VS 版本替换：

```powershell
$vsDevCmd="C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
cmd /c "`"$vsDevCmd`" -arch=x64 -host_arch=x64 && cmake --preset qt-debug"
```

如果 VS 是 Professional、Enterprise 或 BuildTools，路径中的 `Community` 应替换为对应版本。更稳妥方式是先用 `vswhere.exe` 查真实安装路径。

命令行配置前，Codex 应先设置当前进程 PATH：

```powershell
$env:VCPKG_ROOT="C:\dev\vcpkg"
$env:VCPKG_FORCE_SYSTEM_BINARIES="1"
$env:X_VCPKG_ASSET_SOURCES="clear;x-script,curl.exe --ssl-no-revoke --retry 8 --retry-all-errors --retry-delay 5 -L {url} --output {dst}"
$env:PATH="C:\Program Files\7-Zip;C:\Qt\6.7.3\msvc2022_64\bin;C:\opencv\build\x64\vc15\bin;$env:PATH"
```

## 环境检查

基础检查：

```powershell
Test-Path "C:\Qt\6.7.3\msvc2022_64\lib\cmake\Qt6\Qt6Config.cmake"
Test-Path "C:\opencv\build\x64\vc15\lib\OpenCVConfig.cmake"
Test-Path "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
Get-Command 7z.exe -ErrorAction SilentlyContinue
```

建议 Codex 在新电脑上把检查结果整理成这个表：

| 检查项 | 命令 | 通过条件 |
| --- | --- | --- |
| VS2022 | `vswhere` 或检查 `VsDevCmd.bat` | 找到 VS2022 且有 C++ workload |
| CMake | `cmake --version` | 能输出版本 |
| Ninja | `ninja --version` | 能输出版本 |
| MSVC | `cl` 或 VS Developer shell | 能看到 MSVC 版本 |
| Qt package | `Test-Path ...Qt6Config.cmake` | 返回 True |
| Qt runtime | `Test-Path ...Qt6Widgetsd.dll` | 返回 True |
| vcpkg | `Test-Path "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"` | 返回 True |
| 7-Zip | `Get-Command 7z.exe` | 能找到 exe |
| OpenCV package | `Test-Path ...OpenCVConfig.cmake` | 返回 True |
| OpenCV runtime | `Get-ChildItem ...opencv_world343*.dll` | 至少找到一个 DLL |

如果某项失败，先修该项。不要在 Qt、vcpkg、OpenCV 基础路径未通过时启动完整 CMake configure。

仓库里存在历史检查脚本：

```text
scripts/check_environment.ps1
scripts/build_vs2022_qt.ps1
scripts/run_cmake_watchdog.ps1
```

注意：这些脚本里部分内容可能仍保留旧选项名或旧 exe 名，例如 `NEWSONTECH_ENABLE_OPENCV`、`newSontech.exe`、`qt-debug-opencv`。新机器上优先以 `CMakePresets.json` 和本文件为准。使用脚本前，Codex 应先检查脚本是否已经同步到当前 `I2R_RLC_*` 选项和 `i2r_rlc.exe`。

如果用户要求修复这些脚本，最小修复方向是：

| 旧内容 | 应改为 |
| --- | --- |
| `NEWSONTECH_ENABLE_OPENCV` | `I2R_RLC_ENABLE_OPENCV` |
| `newSontech.exe` | `i2r_rlc.exe` |
| `qt-debug-opencv` | 当前 `CMakePresets.json` 里的 `qt-debug` |

修脚本后应单独汇报，不要把脚本修复和源码编译错误混在一起。

## 测试策略

### 第一层：配置和生成

目标：确认 VS2022、Qt、vcpkg、OpenCV 能被 CMake 找到。

通过条件：

- VS CMake configure 成功；
- `out\build\qt-debug\CMakeCache.txt` 存在；
- `out\build\qt-debug\build.ninja` 存在；
- vcpkg 依赖安装在 build tree 下的 `vcpkg_installed\x64-windows`。

如果配置失败，先打开 `CMakeCache.txt` 和 CMake 输出，确认失败属于哪一类：

| 失败类型 | 典型信号 | 处理 |
| --- | --- | --- |
| Qt 缺失 | 找不到 `Qt6Config.cmake` | 修 `CMAKE_PREFIX_PATH` / Qt 安装 |
| vcpkg 缺失 | 找不到 toolchain file | 修 `VCPKG_ROOT` |
| 7-Zip 缺失 | `7z.exe not found` | 安装 7-Zip 或加入 PATH |
| OpenCV 缺失 | `OpenCV was not found` | 修 `OpenCV_DIR` 或临时用 `qt-debug-no-opencv` |
| OpenCV 版本错 | 找到 OpenCV 4.x | 换 OpenCV 3.4.3 |
| 下载失败 | curl / TLS / proxy / timeout | 检查网络、代理、证书策略 |
| 权限失败 | `Access is denied` | 换有权限目录或请用户授权 |

### 第二层：主程序构建和启动

目标：确认 `i2r_rlc.exe` 可生成和启动。

通过条件：

- VS 输出显示生成成功；
- `out\build\qt-debug\i2r_rlc.exe` 存在；
- 启动后能显示 Qt Widgets 主窗口；
- 未报缺少 `Qt6Widgetsd.dll`、`opencv_world343*.dll` 或 vcpkg DLL。

### 第三层：CTest

推荐先跑全部测试：

```powershell
ctest --test-dir out\build\qt-debug -C Debug --output-on-failure
```

历史上该命令曾在非沙箱环境达到 `26/26 passed`。新机器上测试数量可能因为当前代码变化而不同，不要把“必须刚好 26 个”写死；应以“所有已注册测试通过，或失败项有清晰第一错误”为准。

如果需要分块排查，可按模块筛选：

```powershell
ctest --test-dir out\build\qt-debug -C Debug -R "i2r_rlc_config|i2r_rlc_main_widget" --output-on-failure
ctest --test-dir out\build\qt-debug -C Debug -R "i2r_rlc_ibvs" --output-on-failure
ctest --test-dir out\build\qt-debug -C Debug -R "i2r_rlc_pcl" --output-on-failure
ctest --test-dir out\build\qt-debug -C Debug -R "i2r_rlc_zeromq|i2r_rlc_digital_twin" --output-on-failure
ctest --test-dir out\build\qt-debug -C Debug -R "i2r_rlc_vision" --output-on-failure
```

常用 smoke 目标包括：

| 目标 | 目的 |
| --- | --- |
| `i2r_rlc_opencv_smoke_tests` | OpenCV 3.4.3 头文件、库和 DLL 链路 |
| `i2r_rlc_pcl_core_io_filter_tests` | PCL core/io/filter 基础链路 |
| `i2r_rlc_zeromq_smoke_tests` | cppzmq / ZeroMQ 基础链路 |
| `i2r_rlc_digital_twin_tests` | DigitalTwin 服务解析和 transport 边界 |
| `i2r_rlc_ibvs_core_tests` | IBVS 基础逻辑 |
| `i2r_rlc_main_widget_panel_tests` | UI 面板基础状态 |

如果只想证明项目能运行，不要先启用 ONNX、真实相机 SDK 或 RobotSim 联调。

如果 CTest 找不到测试可执行文件，先确认对应 target 是否已经在当前 build tree 中构建。历史上 `qt-debug-no-opencv` 出现过 CTest 注册存在但测试 exe 不存在的情况，这类问题优先按 build tree 不完整处理，不要直接判定源码失败。

## RobotSim 可选联调

RobotSim 是外部运行包，不参与当前 Qt 项目构建。推荐外部包路径：

```text
C:\Code\tongxinceshi\RobotSimWork-Johnston-win64
```

启动时必须以 `bin` 为工作目录：

```powershell
Start-Process `
  -FilePath 'C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin\RobotSim.exe' `
  -WorkingDirectory 'C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin'
```

RobotSim 控制台会打印：

```text
COMMServer Port: <port>
```

把端口填入：

```text
config\digital_twin\robotsim_endpoint.txt
```

或用 probe 手工验证：

```powershell
$json='["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1013]]'
.\out\build\qt-debug\tests\i2r_rlc_robotsim_probe.exe <port> $json
```

批量查询：

```powershell
$json='[["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1013]],["SENSOR_LSL_VALUE_RET","GET_WELD_SEAM_PHOTO",[1013]]]'
.\out\build\qt-debug\tests\i2r_rlc_robotsim_probe.exe <port> $json
```

期望返回：

- `GET_LSL_PHOTO`：约 29 个激光点，每个点为 `[[x_mm,y_mm,z_mm],[u_px,v_px]]`；
- `GET_WELD_SEAM_PHOTO`：焊缝图像点，例如 `[[1034,374]]`。

如果返回 `[null,null]` 或空数组，先检查 RobotSim 是否真正加载了英文路径下的场景，传感器 ID 是否为 `1013`，不要先改 Qt 项目协议。

## 常见问题处理

### 找不到 `cl.exe`

结论：普通 PowerShell 没加载 VS 开发环境。

处理：

1. 优先用 VS2022 打开仓库并构建；
2. 或使用 Developer PowerShell for VS 2022；
3. 或先调用 `VsDevCmd.bat -arch=x64 -host_arch=x64`。

不要因为普通 shell 找不到 `cl.exe` 就重装整个项目依赖。

### 找不到 Qt DLL

典型报错：

```text
Qt6Widgetsd.dll was not found
```

处理：

```powershell
$env:PATH="C:\Qt\6.7.3\msvc2022_64\bin;$env:PATH"
```

VS 中应使用 preset 的 environment。不要把 Qt DLL 复制到源码目录。

### 找不到 OpenCV DLL

典型报错：

```text
opencv_world343*.dll was not found
```

处理：

```powershell
$env:PATH="C:\opencv\build\x64\vc15\bin;$env:PATH"
```

如果 CMake 报 OpenCV 版本为 4.x，说明机器上找错了 OpenCV。当前项目应使用 3.4.3，或临时选择 `qt-debug-no-opencv`。

### vcpkg 下载或解压失败

优先检查：

```powershell
Test-Path "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
Get-Command 7z.exe -ErrorAction SilentlyContinue
```

本项目 preset 已设置：

```text
X_VCPKG_ASSET_SOURCES=clear;x-script,curl.exe --ssl-no-revoke --retry 8 --retry-all-errors --retry-delay 5 -L {url} --output {dst}
```

如果新机器网络或证书策略拦截下载，需要让用户确认代理、网络和 vcpkg 缓存策略。不要把依赖源码直接复制进仓库。

### C 盘空间不足

vcpkg + PCL + OpenCV build tree 会占用较多空间。优先确认：

```powershell
Get-PSDrive C
```

清理策略：

1. 保留当前活跃 build tree，例如 `out\build\qt-debug`；
2. 优先删除历史 build tree；
3. 不默认删除 `.vs`，除非用户明确接受重新生成 VS 状态和更慢的后续打开速度；
4. 删除前确认 VS 已关闭，避免锁文件残留。

### `.ninja_lock` 删除失败或构建卡住

先检查进程：

```powershell
Get-Process devenv,cmd,cmake,ninja,cl,link,vcpkg -ErrorAction SilentlyContinue |
    Select-Object Id,ProcessName,CPU,StartTime,Path
```

如果确认是本轮中断遗留的构建进程，可按 PID 停止：

```powershell
Get-Process -Id <pid1>,<pid2> -ErrorAction SilentlyContinue | Stop-Process -Force
```

确认没有相关进程后，再删锁：

```powershell
Remove-Item out\build\qt-debug\.ninja_lock -Force
```

不要在已有卡住构建未清理时反复启动新的完整构建。

### 构建超过 2 分钟无输出

处理顺序：

1. 观察 VS 输出窗口是否仍有新增输出；
2. 查看是否存在高 CPU 的 `cl.exe` / `link.exe`；
3. 如果只有低 CPU 的 `cmake` / `ninja`，转入锁文件和 vcpkg 诊断；
4. 不要继续空等，也不要连续发起多个 build。

### MSVC `C1128` section 超限

当前根 CMake helper 已统一添加 `/bigobj`。如果某个新 target 仍报：

```text
C1128: 节数超过对象文件格式限制
```

检查该 target 是否调用了：

```cmake
i2r_rlc_apply_common_options(<target>)
```

不要在单个源文件里绕过这个问题。

### Ninja 漏记 MSVC 头文件依赖

`/showIncludes` 的输出语言必须与 Ninja 的 `msvc_deps_prefix` 一致。
如果配置时识别的是中文前缀，而实际编译输出英文，Ninja 可能记录零个头文件依赖。
随后修改按值成员类型时，入口对象未重新编译，可能与新库的对象布局不一致，
并在构造或“选择工件”清理状态时发生内存访问错误。

根 CMake 为 MSVC + Ninja 的每次编译固定 `VSLANG=1033`，并同步设置英文依赖前缀。
该设置同时覆盖既有 build tree 的旧探测结果。首次应用会因编译命令变化重新编译；
无需删除构建目录。不要仅手改生成的 `rules.ninja`，也不要只在配置 shell 设置语言。

在已配置的 Developer 环境完成 Debug 应用构建后，串行运行以下检查：

```powershell
python tests/test_msvc_build_contract.py out/build/debug-hdrive --probe-header
# 使用与 run-hdrive.ps1 相同的 Qt PATH 和 QT_PLUGIN_PATH 环境：
./tests/test_workpiece_application_lifecycle.ps1 -Executable ./out/build/debug-hdrive/UI/i2r_rlc.exe
```

第一个检查需要 Python 3 和 EXE 对应的 MSVC Debug PDB，验证实际依赖记录、
间接头文件变动的增量编译，以及入口和构造函数活动类型大小一致。
`--probe-header` 暂时改变头文件时间戳并在 finally 恢复，只执行 Ninja dry-run，
运行时不得有并发生成。
第二个检查需要 Windows UI Automation 和已有可用工件配置，
只通过 InvokePattern 操作测试创建的应用进程，禁用设备自动启动，
验证两次选择工件均完成在线/离线配置并正常退出。
这两个产物级检查需要上述环境前提，故不挂入通用 CTest。

## 新机器验收清单

Codex 完成配置后，应向用户汇报以下事实：

- VS2022 / MSVC / CMake / Ninja 是否可用；
- Qt prefix 是否为 `C:\Qt\6.7.3\msvc2022_64`；
- `VCPKG_ROOT` 是否有效；
- 7-Zip 是否在 PATH；
- OpenCV 3.4.3 的 `OpenCVConfig.cmake` 和 DLL 是否存在；
- 当前使用的 CMake preset；
- `i2r_rlc.exe` 是否成功生成；
- 主程序是否能启动；
- CTest 是否运行，运行了全部还是部分，失败项是什么；
- 是否跳过了真实相机、ONNX、RobotSim 等可选项。

最小成功标准：

```text
qt-debug configure PASS
qt-debug build PASS
i2r_rlc.exe starts
ctest basic/smoke tests PASS or failed tests have clear first error
```

如果没有达到最小成功标准，先解决第一条 configure/build/runtime 错误，不要同时修改多个模块或引入新依赖。
