# Legacy 架构审查：CRobotleadingDlg

## 范围

本文总结 `CRobotleadingDlg` 在旧系统中的职责，并记录 Qt Widgets 迁移时必须避免的设计问题。

`CRobotleadingDlg` 不是简单 UI dialog。在旧 MFC 应用中，它同时协调 UI、相机采集、机器人通信、图像处理、标定、离线纠偏、在线纠偏、数字孪生、IBVS 集成、配置、日志、定时器和 worker thread。

## 关键 Legacy 文件

| 区域 | 文件/路径 |
| --- | --- |
| 主 dialog 声明 | `SWv1/robotleadingDlg.h` |
| 主 dialog 实现 | `SWv1/robotleadingDlg.cpp` |
| MFC dialog 资源 | `SWv1/SWv1.rc` |
| 资源 ID | `SWv1/resource.h` |
| 共享 legacy 全局变量 | `Robotleading/robotleading_method/RStdAfx.h` |
| 全局变量定义 | `Robotleading/robotleading_method/RStdAfx.cpp` |
| 机器人 TCP/控制类 | `Robotleading/robotleading_method/RobotCtrl.cpp` |
| Basler/IBVS 采集路径 | `SWv1/PicCapture.cpp` |

## 实际职责

| 职责 | Legacy 行为 |
| --- | --- |
| UI | 持有 MFC dialog、控件、子 dialog、message map、命令 handler、timer、显示更新和日志 UI。 |
| 相机采集 | 通过不同代码路径协调 Vimba/AVT、旧 HALCON framegrabber 和 Basler/Pylon 采集。 |
| 机器人通信 | 启动并轮询 `RobotCtrl`；该类是 MFC `CButton` 子类，但同时管理 TCP 通信和全局机器人状态。 |
| 图像处理 | 调用焊缝检测、中心线提取、Gabor/filtering、图像点到空间点转换、坡口检测、跟踪和匹配逻辑。 |
| 标定 | 读取并使用 `SENSORPARA.txt` 等标定文件，同时打开标定相关 UI。 |
| 离线纠偏 | 协调模型/点匹配和示教路径修正。 |
| 在线纠偏 | 组合当前机器人状态、相机帧、检测结果和纠偏命令生成。 |
| 数字孪生 | 持有或协调虚拟机器人、虚拟线激光、焊件模型、点云、ICP/定位和仿真线程行为。 |
| IBVS | 与 Basler/Pylon 采集、IBVS solver、特征检测器和 LVS/IBVS 共享全局状态交互。 |
| 配置 | 硬编码或初始化大量路径、作业名、模型名和运行时文件。 |
| 日志 | 在 dialog 相关代码中重定向/收集日志和调试输出。 |

## 相机与图像显示

legacy 代码包含三条相机/采集相关路径，目前目标职责已澄清：

- 通过 `ApiController` 和 `WM_FRAME_READY` 获取 Vimba/AVT 异步帧：焊缝跟踪和数字孪生线结构光图像的主路径。
- 通过 `PicCapture(cv::Mat&)` 获取 Basler/Pylon 图像：IBVS 外置相机路径。
- 通过 `SnapThread` 使用 HALCON framegrabber：非优先 legacy 路径，只有旧算法或旧数据格式需要时才考虑作为可选 adapter。

新图像处理应尽量优先使用 OpenCV。如果后续确实需要 HALCON，也必须保持隔离。

## 机器人与流程状态

机器人状态通过 `sMode`、`sServoPower`、`sStart`、`sHold`、`traflag`、`corflag`、`curx`、`cury`、`curz`、`curRx`、`curRy`、`curRz` 及其 `_r` 变体等全局变量传递。这些值由 `RobotCtrl::RevRobotData()` 更新，并被 UI、图像处理、数字孪生和 IBVS 逻辑读取。

机器人控制器已确认为 Yaskawa DX200，但详细协议字段语义仍待确认。新设计必须默认使用 `RobotComm dry-run`；live `DX200RobotClient` 命令发送必须推迟到显式授权和验证之后。

## 数字孪生

该 dialog 持有仿真对象并启动 `virtualSceneThreadFunc()`。该函数持续读取机器人/全局状态，生成虚拟激光线，更新点云，执行离线或在线模型修正，并向 dialog post/send MFC 消息。

数字孪生不是首版完整目标。新项目应先定义 `DigitalTwinService` API 和 mock/no-op 实现，不把 OpenCascade/VTK viewer 或 RobotSim 内部依赖引入最小骨架。基于 PCL 的点云求解归属独立 `localization` 模块。

## IBVS

IBVS 行为分散在 `PicCapture.cpp`、`IBVS.*`、`YoloDetector`、`RedFeatureDetector` 等检测器、手眼标定代码以及全局 LVS/IBVS 状态中。它使用 Basler/Pylon 作为外置相机路径。

IBVS 不是首版完整目标。新项目应先定义 `IbvsService`/`IbvsState` 接口和 mock/no-op 行为。

## 配置与运行时数据

legacy 代码依赖 `data/Robotleadingdata`、`data/IBVS`、`models`、`output` 等路径。这些路径通常相对于工作目录。

`SENSORPARA.txt` 已确认是需要支持的 legacy 标定文件。新设计应将其映射到 `SensorCalibrationData` 或 `CalibrationData`，并显式保留 m、deg、pixel 等单位。运行时路径应通过 `AppPaths` 解析，并通过 `ConfigRepository` 加载。

## 主要 Legacy 设计问题

| 问题 | 影响 |
| --- | --- |
| UI 与业务逻辑耦合 | Dialog handler 会启动设备、运行 workflow、修改全局状态并执行文件 IO。 |
| 全局变量过多 | 产生隐藏依赖、竞态和较差的可测试性。 |
| 线程生命周期不清晰 | raw thread、`while(true)` 循环、全局标志和不完整清理可能导致关闭卡死。 |
| UI 线程可能阻塞 | 相机启动、重试循环、关闭循环和同步设备操作可能冻结 UI。 |
| 设备控制与 UI 事件混合 | MFC button handler 直接影响机器人、相机和 workflow 状态。 |
| 数据路径硬编码 | 部署和测试依赖当前工作目录及旧数据布局。 |
| 资源生命周期不完整 | raw pointer、SDK handle、`HANDLE`、`FILE*` 和仿真对象都手动管理。 |
| include/build 边界脆弱 | `PicCapture.cpp` 存在 HALCON/newmat 和仿真头文件的 include 顺序冲突。 |
| 代码难以测试 | 算法依赖 UI、全局变量、文件、设备和可变共享状态。 |

## 迁移含义

Qt 应用必须围绕服务和显式数据流重新设计：

- `MainWidget` only displays state and emits user intent.
- `UiController` coordinates services.
- 类型化模型和单位信息放在归属模块的 public header 中。
- `camera` owns Vimba, Basler, offline, and optional HALCON backends.
- `RobotComm` owns DX200 parser/client abstractions and dry-run defaults.
- `vision` owns OpenCV-first weld tracking algorithms.
- `control` owns weld tracking workflow state.
- `digital_twin` and `IBVS` start as interface/mock/no-op services.

正确迁移方式是行为抽取和模块化重设计，而不是逐行 MFC-to-Qt 翻译。
