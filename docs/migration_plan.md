# 迁移计划

## 原则

不要把 `CRobotleadingDlg` 直接翻译成一个 `QWidget`。legacy 类混合了 UI、设备、算法、状态和 workflow 协调职责。迁移必须先把行为抽取到模块中，再让 Qt UI 通过 signal、slot 和类型化服务接口调用这些模块。

第一个功能目标是焊缝跟踪主流程。数字孪生和 IBVS 不是首版完整实现目标。如果焊缝跟踪 workflow 需要它们的信息，应先定义 API 名称、数据结构和 mock/no-op 服务。骨架阶段不要引入 OpenCascade/VTK viewer、RobotSim internals、完整 Basler IBVS 控制或 HALCON 依赖。基于 PCL 的点云求解归属 `localization`。

## 阶段 1：文档与架构确认

| 项目 | 描述 |
| --- | --- |
| 目标 | 固化 legacy 理解，并确认目标架构。 |
| 输入 | 第一阶段审查；`robotleadingDlg.h/.cpp`；`SWv1.rc`；`RStdAfx.h/.cpp`；`RobotCtrl.cpp`；`PicCapture.cpp`；用户关于相机、DX200、运行时数据、首版范围和 `SENSORPARA.txt` 的确认。 |
| 输出 | `i2r_rlc/docs` 下的 review、dependency map、migration plan、new architecture、threading design、risk log、runtime data layout、calibration file format。 |
| Legacy files | `SWv1/robotleadingDlg.*`, `SWv1/SWv1.rc`, `Robotleading/robotleading_method/*`, `SWv1/PicCapture.cpp`, legacy runtime files such as `data/Robotleadingdata/BASIC/SENSORPARA.txt`. |
| New target modules | `docs` only. |
| 风险 | DX200 协议语义和部分标定字段仍是暂定。 |
| 验证 | 用户审查已记录的范围门控和安全默认值。 |

## 阶段 2：Qt Widgets + CMake 最小骨架

| 项目 | 描述 |
| --- | --- |
| 目标 | 创建独立 Qt Widgets 可执行程序，包含顶层 `MainWidget`、`UiController`、mock 服务和清晰启动/关闭流程。 |
| 输入 | 已确认的架构文档和运行时数据布局。 |
| 输出 | 可启动和干净退出的最小应用，显示占位状态，仅使用 mock/offline backend，不发送真实设备命令。 |
| Legacy files | None copied; only references to UI inventory. |
| New target modules | `app`, `ui`, `ui` support classes and `localization` calibration data, top-level build files. |
| 风险 | 本机 Qt/MSVC/CMake 环境可能与假设不同。 |
| 验证 | Configure、build、run、close；确认不需要真实相机或机器人 SDK；确认 `MainWidget` 不包含设备逻辑。 |

## 阶段 3：数据模型抽象

| 项目 | 描述 |
| --- | --- |
| 目标 | 用显式 value type 替代 legacy 全局状态。 |
| 输入 | `RStdAfx.h/.cpp` 全局变量和 workflow 标志；`SENSORPARA.txt` 字段清单。 |
| 输出 | `RobotState`、`CameraFrame`、`WeldVisionPipelineResult`、`SensorCalibrationData`、`WorkpieceProfile`、`TrackingState`、`CorrectionCommand`、`IbvsState`、`DigitalTwinState` 等纯 C++ 模型。 |
| Legacy files | `Robotleading/robotleading_method/RStdAfx.h`, `RStdAfx.cpp`, selected uses in `robotleadingDlg.cpp`, `SENSORPARA.txt`. |
| New target modules | module-owned public data headers. |
| 风险 | 部分全局变量编码了未文档化的隐式状态机切换。 |
| 验证 | 对模型默认值、单位标签、状态快照和必要序列化增加单元测试。 |

## 阶段 4：配置管理

| 项目 | 描述 |
| --- | --- |
| 目标 | 用可校验配置替代硬编码路径和内嵌工件/JBI 列表。本阶段是分层架构的数据与配置支撑，不是孤立功能。 |
| 输入 | `OnInitDialog()` 中的硬编码路径；`SENSORPARA.txt`；`RBCALIB.DAT`；`data/IBVS`；`.weldment` 和 JBI 列表。 |
| 输出 | `ConfigRepository`、`AppPaths`、运行时目录策略、`SENSORPARA.txt` parser、传感器/工位标定 parser 或 parser 接口、工件注册表 schema。 |
| Legacy files | `SWv1/robotleadingDlg.cpp`, `SWv1/PicCapture.cpp`, `RStdAfx.h/.cpp`, `data/Robotleadingdata/BASIC/SENSORPARA.txt`. |
| New target modules | `ui` support classes and `localization` calibration data. |
| 风险 | Legacy 文件可能包含未文档化格式和混合单位；`pose_CinT[6]` 仍未知。 |
| 验证 | 只读定位代表性 legacy 文件；验证路径解析不依赖当前工作目录。parser 实现可以后续补充。 |

Phase 4 supports multiple functional layers:

- application layer: system calibration, online correction, offline correction, global localization, and local localization need validated calibration/configuration inputs;
- algorithm layer: image-point to space-point conversion, visual simulation, local localization, and correction calculations need `SensorCalibrationData` with explicit units;
- middleware layer: camera, robot, and future IPC adapters need validated runtime paths and configuration;
- interface layer: UI should show configuration readiness and errors, but must not parse files directly.

下一步实现仍应是 `AppPaths`、`ConfigRepository` 和 `SENSORPARA.txt` parser。不应引入真实 Vimba/Basler/HALCON/DX200/OpenCascade/VTK/YOLO/ONNX 依赖；PCL 保留给 `localization` 边界。

## 阶段 5：机器人通信协议封装

| 项目 | 描述 |
| --- | --- |
| 目标 | 从 `RobotCtrl` 中抽取 DX200 机器人通信概念，转为无 UI 依赖接口，并默认 mock/dry-run。 |
| 输入 | `RobotCtrl::InitTCP()`、`RobotCtrl::RevRobotData()`、`RobotCtrl::SendBuff()`、已知 DX200 控制器类型，以及未经显式授权不得发送 live motion 命令的安全要求。 |
| 输出 | RobotComm client 接口、DX200 adapter 边界、dry-run 行为、消息序列化、类型化命令和状态更新。 |
| Legacy files | `Robotleading/robotleading_method/RobotCtrl.h`, `RobotCtrl.cpp`. |
| New target modules | `RobotComm`, module-owned public data headers. |
| 风险 | DX200 协议字段语义和命令时序仍未知。 |
| 验证 | 有记录字符串后编写 parser/serializer 测试；增加 mock TCP 集成测试；UI 必须显示 dry-run 模式。真实硬件发送推迟到用户显式授权和验证之后。 |

## 阶段 6：相机接口抽象

| 项目 | 描述 |
| --- | --- |
| 目标 | 将已确认的相机角色封装到显式采集接口后。 |
| 输入 | Vimba `ApiController`、Basler/Pylon `PicCapture`、旧 HALCON framegrabber 引用、离线图像。 |
| 输出 | `ICameraDevice`、`CameraWorker`、`VimbaTrackingCameraBackend`、`BaslerIbvsCameraBackend`、`OfflineImageCameraBackend`、可选 `HalconCameraBackend` legacy adapter。 |
| Legacy files | `SWv1/robotleadingDlg.cpp`, `SWv1/PicCapture.cpp`, `Robotleading/robotleading_method/Vimba_SDK/*`. |
| New target modules | `camera`, current `ui` support classes, module-owned public data headers. |
| 风险 | Vimba 焊缝跟踪和 Basler IBVS 在时序、标定和处理需求上不同。如果需要 legacy adapter，HALCON/newmat 冲突仍存在。 |
| 验证 | 先从 `OfflineImageCameraBackend` 开始；随后接入 Vimba 焊缝跟踪。Basler 在 IBVS 排期前保持在接口之后。`HalconCameraBackend` 默认不构建。 |

## 阶段 7：无设备算法迁移

| 项目 | 描述 |
| --- | --- |
| Goal | Move pure or mostly pure algorithms behind testable functions, prioritizing the weld tracking pipeline. |
| Input | `getCurrentCenterLine`, `ConvertImgPt2SP`, `GaborFilter`, `ImagePro*`, `STC`, `KCF`, `GapDetector`, selected calibration math. |
| Output | `vision` and `control` modules accepting explicit inputs and returning typed results. Use OpenCV-first implementations. HALCON is only an isolated compatibility adapter when unavoidable. |
| Legacy files | `SWv1/robotleadingDlg.cpp`, `SWv1/CImageDenoise.*`, `SWv1/GapDetector.*`, `Robotleading/robotleading_method/ImagePro*`, `STC.*`, `kcftracker.*`, selected `SWv1/IBVS.*` math only if needed. |
| New target modules | `vision`, `control`, module-owned public data, `tests`. |
| Risk | Implicit units and legacy global state may affect numeric output. |
| Verification | Golden-image tests, calibration conversion tests, numeric tolerance tests, no UI/device dependencies. |

## 阶段 8：线程模型重构

| 项目 | 描述 |
| --- | --- |
| Goal | Replace raw `CreateThread`, `while(true)`, and multimedia timers with managed Qt workers. |
| Input | `SnapThread`, `OpencvThread`, `IBVSThread`, `virtualSceneThreadFunc`, `TCPTimerProc`. |
| Output | `CameraWorker`, `VisionWorker`, `RobotCommunicationWorker`, optional/mock `DigitalTwinWorker`, `CorrectionControlWorker` for weld tracking first, safe stop protocol. |
| Legacy files | `SWv1/robotleadingDlg.cpp`, `SWv1/PicCapture.cpp`. |
| New target modules | `camera`, `vision`, `RobotComm`, `control`, `digital_twin`, `ui` support classes. |
| Risk | Existing behavior depends on shared globals and timing side effects. |
| Verification | Start/stop stress tests, no worker touches QWidget, clean process shutdown, dry-run robot default. |

## 阶段 9：UI 拆分

| 项目 | 描述 |
| --- | --- |
| Goal | Build a QWidget UI that displays state and sends commands without owning business logic. |
| Input | `IDD_ROBOTLEADING_DIALOG` UI inventory and weld tracking workflow needs. |
| Output | `MainWidget` plus smaller widgets such as camera panel, robot status panel, weld tracking control panel, calibration panel, and log panel. |
| Legacy files | `SWv1/SWv1.rc`, `SWv1/resource.h`, `robotleadingDlg.h/.cpp` UI handlers. |
| New target modules | `ui`, `app`. |
| Risk | Users may expect exact legacy layout; exact copying would preserve poor workflows. |
| Verification | UI smoke tests, signal/slot tests, manual workflow review with user. |

## 阶段 10：业务 Workflow 状态机

| 项目 | 描述 |
| --- | --- |
| Goal | Replace scattered flags with explicit process states and transitions for the weld tracking main workflow. |
| Input | `virtualStatus`, `sMode`, `sStart`, `traflag`, `corflag`, `job_done`, `posflag`, online/offline flags. |
| Output | `COnlineCorrectionController` covering OnlineCor change 主链路的初始检测、反向完成等待、在线就绪、运行、停止和错误状态。Digital twin and IBVS states exist only as optional service availability/status markers in the first release. |
| Legacy files | `robotleadingDlg.cpp`, `RStdAfx.h/.cpp`, `OnlineCor.cpp`, `OffLineCor.cpp`, `PicCapture.cpp`. |
| New target modules | module-owned public data, `control`, `ui`. |
| Risk | Legacy state transitions are implicit and partially undocumented. |
| Verification | Transition table tests; simulated robot/camera events; fault injection. |

## 阶段 11：集成与验证

| 项目 | 描述 |
| --- | --- |
| Goal | Integrate workers, algorithms, UI, and configured devices in controlled steps. |
| Input | Completed modules and test fixtures. |
| Output | Validated standalone Qt application with documented limitations. |
| Legacy files | Read-only comparison only. |
| New target modules | All runtime modules plus `tests`. |
| Risk | Hardware timing, camera SDK compatibility, and robot safety constraints. |
| Verification | Offline replay, mock robot, Vimba camera test, robot dry-run, controlled DX200 test only after explicit authorization, shutdown/restart test. |

## 实现前建议门控

Phase 2 can begin now with a Qt Widgets + CMake minimal skeleton because the first-release scope is defined as weld tracking main workflow first, using mock/offline services and no real robot motion.

Confirmed gates:

- 使用当前 workspace 中实际目录 `i2r_rlc`；不要创建大小写不同的重复目录。
- 首版优先焊缝跟踪主流程。
- Digital twin and IBVS are interface/mock/no-op only in the first skeleton.
- Camera design is split into `VimbaTrackingCameraBackend`, `BaslerIbvsCameraBackend`, `OfflineImageCameraBackend`, and optional `HalconCameraBackend`.
- Robot design includes `DX200RobotClient`, `RobotComm dry-run`, `RobotComm dry-run`, `ProtocolParser`, and `ProtocolSerializer`.
- 真实 DX200 硬件命令发送推迟到协议语义、配置和用户授权确认之后。
- Runtime data paths are owned by `AppPaths` and `ConfigRepository`, not the current working directory.

Remaining gates before real hardware integration:

- Detailed DX200 protocol field semantics and recorded sample messages.
- Confirmation of any available DX200 simulator, pendant-side test mode, or dry-run environment.
- Confirmation of which legacy algorithms truly require HALCON.
- Confirmation of `pose_CinT[6]` meaning in `SENSORPARA.txt`.

