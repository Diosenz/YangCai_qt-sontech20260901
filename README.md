# i2r_rlc

`i2r_rlc` 是机器人定位与纠偏系统的 Qt Widgets + CMake 项目。当前分支正在把旧 MFC `CRobotleadingDlg` 相关能力重构为模块独立生成、线程边界清晰、可测试的现代 C++ 应用。

## 当前状态

- 主程序 target：`i2r_rlc`，VS2022 中启动调试时请选择 `i2r_rlc.exe`。
- 根 CMake 只负责全局选项、依赖发现和 `add_subdirectory`；每个模块维护自己的 `CMakeLists.txt`。
- UI 层采用 MVC：`MainWidget` 是 QWidget View，`UiController` 负责界面状态和用户意图转发。
- `main.cpp` 创建并回收模块线程集合，真实业务逻辑留在各模块内。
- `oldproject/` 和 `RobotSim/` 只读参考，不参与当前 Qt 项目构建。
- 当前仍是重构骨架：真实相机、真实 DX200 运动、完整焊缝识别、定位求解、RobotSim 交互和 IBVS 算法需要继续接入。

## 模块结构

当前生产模块固定为：

- `src/ui`：最上层 MVC，包含 QWidget、Controller、日志/配置轻量支撑和运行时组装边界。
- `src/ibvs`：视觉伺服模块。
- `src/control`：焊接纠偏与流程控制模块。
- `src/vision`：焊缝识别模块，只承载焊接相关视觉算法。
- `src/localization`：定位模块，承载全局/局部定位和 PCL 点云配准边界；公共接口不暴露 PCL 类型。
- `src/RobotComm`：机器人通信生产库，替代旧 `robot`/`communication` 边界。
- `src/camera`：相机采集模块。
- `src/digital_twin`：数字孪生模块，用于 RobotSim 对接和仿真参考数据边界。

不要重新引入 `core`、`infrastructure`、`app`、`robot`、`communication` 作为生产模块。跨模块数据结构归属于实际业务模块的公开头文件。

## CMake Presets

推荐在 Visual Studio 2022 中打开仓库根目录，选择以下 preset：

- `qt-debug`：Qt 6.7.3 + OpenCV 3.4.x + Allied Vision，适合当前 VS 全量生成。
- `qt-debug-no-opencv`：禁用 OpenCV，用于只验证模块骨架。
- `qt-release`：Release 全量生成。

关键选项：

```text
I2R_RLC_ENABLE_OPENCV=ON|OFF
I2R_RLC_ENABLE_TESTS=ON|OFF
```

OpenCV 3.4.3 默认路径：

```text
OpenCV_DIR=E:/Libs/OPENCV3/opencv/build/x64/vc15/lib
OpenCV bin=E:/Libs/OPENCV3/opencv/build/x64/vc15/bin
```

Qt 默认路径：

```text
D:/QT/6.7.3/msvc2022_64
```

## 构建与启动

VS2022 中：

1. 打开本仓库根目录。
2. 选择 `qt-debug` 或 `qt-debug-no-opencv` preset。
3. 执行“全部生成”。
4. 在启动项中选择 `i2r_rlc.exe`，不要选择 `.lib` 模块或测试 target 作为调试启动项。

命令行配置示例：

```powershell
cmake --preset qt-debug
cmake --build out/build/qt-debug --config Debug
```

本项目依赖 vcpkg manifest。`vcpkg.json` 目前包含 Qt 外的算法/通信基础依赖，例如 `cppzmq`、`concurrentqueue`、`fmt`、`magic-enum`、`eigen3`、`pcl`。

## 迁移到另一台电脑

迁移后先在 `CMakePresets.json` 中核对以下本机路径：

- `CMAKE_PREFIX_PATH` / `QTDIR`：Qt MSVC 2022 64-bit 根目录；
- `CMAKE_TOOLCHAIN_FILE` / `VCPKG_ROOT`：vcpkg 根目录；
- `OpenCV_DIR`：包含 `OpenCVConfig.cmake` 的 OpenCV 3.4.x 目录；
- `PATH`：Qt、OpenCV 和 Allied Vision 的 DLL 目录。

不要迁移 `.vs/`、`out/`、`build/` 和 `vcpkg_installed/`。如果已经复制，首次生成前删除这些机器相关缓存，再让 CMake/vcpkg 重新生成。

## 测试

测试 target 统一放在 `tests/`，通过 `tests/CMakeLists.txt` 挂入。当前测试覆盖：

- 配置和标定解析；
- 离线相机；
- vision pipeline / OpenCV smoke；
- 焊缝检测接口 stub；
- control workflow；
- localization no-op service；
- PCL smoke tests；
- ZeroMQ / RobotComm smoke tests。

CTest 示例：

```powershell
ctest --test-dir out/build/qt-debug -C Debug --output-on-failure
```

当前阶段如果 VS 已能生成并启动 `i2r_rlc.exe`，优先按 VS 报出的具体编译错误逐项修复，不建议长时间等待卡住的 Ninja/CMake wrapper 进程。

## 文档入口

- `docs/new_architecture.md`：当前模块化架构总览。
- `docs/layered_system_architecture.md`：三层功能架构和模块映射。
- `docs/threading_design.md`：main 管线程和模块 worker 边界。
- `docs/localization_visual_servo_digital_twin_scope.md`：定位、IBVS、数字孪生职责边界。
- `docs/collaboration_git_workflow.md`：Twin / integrate 协作流程。
- `docs/I2R_Cpp_Codex_Rules.md`：C++ 详细规则库。
