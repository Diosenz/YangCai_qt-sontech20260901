# 线程设计

## 设计规则

不要复制 legacy 线程模型。Qt 应用不应使用 raw `CreateThread`、脱离生命周期管理的 `while(true)` 循环、`timeSetEvent`，也不应让 worker 代码直接操作 UI widget。

UI 线程只负责：

- 创建和更新 widget；
- 接收用户输入；
- 显示当前状态、图像、overlay 和错误；
- 通过 signal 请求后端命令。

所有长时间运行或阻塞性工作都必须运行在 worker 中。worker 绝不能调用 QWidget 方法。

## 线程模型与分层架构

worker 模型不是新的功能层，而是贯穿分层架构的运行时执行机制。

| 线程/worker | 层关系 | 说明 |
| --- | --- | --- |
| UI thread | Interface layer only | Owns `MainWidget`, widget painting, user input, and display updates. It must not perform camera acquisition, robot communication, heavy algorithms, or config parsing with noticeable latency. |
| `CameraWorker` | Middleware layer and driver-adapter boundary | Owns camera backend execution such as offline replay now and Vimba/Basler adapters later. It serves the application layer by emitting frame data. |
| `VisionWorker` | Algorithm layer | Runs weld detection, tracking, image-to-space conversion inputs, and future OpenCV-first vision logic. |
| `RobotCommunicationWorker` | Middleware layer | Owns PC-side robot communication, protocol polling/events, mock/dry-run now, and future DX200 adapter later. |
| `CorrectionControlWorker` | Application layer and algorithm layer bridge | Coordinates closed-loop correction decisions using robot state, vision results, calibration, and safety gates. |
| `DigitalTwinWorker` | Algorithm layer, optional | Future visual simulation and point-cloud/model updates. First version remains mock/no-op. |
| `LogWorker` | Cross-cutting support | Optional async logging support. |

该映射保留原 worker 设计意图，同时明确线程是执行边界，不是业务归属边界。

## Worker 概览

| Worker | 是否必需 | 职责 |
| --- | --- | --- |
| `CameraWorker` | Yes | Own one camera backend, acquire frames, emit frame snapshots. |
| `VisionWorker` | Yes | Consume frames and robot pose snapshots, run weld detection/centerline/gap logic. |
| `RobotCommunicationWorker` | Yes | Own selected robot client. Defaults to mock/dry-run; real DX200 is explicitly enabled later. |
| `DigitalTwinWorker` | Optional | First release uses interface/mock/no-op only. Real geometry worker comes later. |
| `CorrectionControlWorker` | Optional but recommended | First release serves weld tracking main workflow only; IBVS remains interface/status only. |
| `LogWorker` | Optional | Asynchronous file logging and diagnostic output. |

## CameraWorker

| Item | Design |
| --- | --- |
| Inputs | Camera configuration, backend selection, start/stop commands. |
| Outputs | `frameReady(Frame)`; `cameraStateChanged(CameraState)`; `cameraError(AppError)`. |
| Startup | Created by `UiController`, moved to a dedicated `QThread`, started through a queued slot. |
| Stop | Queued `stop()` sets cancellation flag, stops SDK acquisition, drains queue, releases camera, emits `stopped()`. |
| Exception handling | Catch SDK errors, convert to `AppError`, transition to stopped/error state. |
| Prohibited | Must not draw with `HDC`, create `CImage`, or update QWidget. |

Camera backend roles:

| Backend | Role | First-release status |
| --- | --- | --- |
| `OfflineImageCameraBackend` | Reads image sequences or replay data for early workflow and algorithm tests. | Preferred initial backend. |
| `VimbaTrackingCameraBackend` | Vimba/AVT camera for weld tracking and digital-twin line-structured-light images. | First real camera backend for weld tracking after skeleton. |
| `BaslerIbvsCameraBackend` | Basler/Pylon external camera for IBVS. | Interface/backend name may exist; full integration deferred with IBVS. |
| `HalconCameraBackend` | Optional legacy framegrabber adapter only if old algorithms or old data formats require HALCON. | Default disabled; not part of minimal skeleton. |

帧 payload 应包含：

- image data, preferably `cv::Mat` wrapped safely or converted to a UI-safe image copy at the UI boundary;
- timestamp;
- frame number;
- camera id;
- optional exposure/frame-rate metadata;
- calibration profile id when applicable.

## VisionWorker

| Item | Design |
| --- | --- |
| Inputs | `Frame`, latest `RobotState`, `SensorCalibrationData`, `VisionSettings`, optional mock digital twin data. |
| Outputs | `visionResultReady(WeldVisionPipelineResult)`, `trackingDataReady(TrackingObservation)`, `visionError(AppError)`. |
| Startup | Runs in its own `QThread` or thread pool depending on frame rate. |
| Stop | Stop accepting frames, finish or cancel current frame, clear bounded queues. |
| Exception handling | Per-frame errors should not crash the worker; emit structured errors and continue if safe. |
| Prohibited | Must not read or mutate UI controls; must not depend on `g_pMain`. |

worker 应避免无界队列。如果处理速度慢于相机采集，默认策略应根据控制需求选择 drop-old-frame 或 keep-latest-frame。

OpenCV 是优先图像处理依赖。HALCON 只能出现在可选兼容 adapter 后面。

## RobotCommunicationWorker

| Item | Design |
| --- | --- |
| Inputs | Robot endpoint/configuration, connect/disconnect requests, typed `RobotCommand`s. |
| Outputs | `robotStateChanged(RobotState)`, `robotConnectionChanged(ConnectionState)`, `robotError(AppError)`, `dryRunCommandRecorded(RobotCommand)`. |
| Startup | Owns the selected robot client in worker thread. Default client is `RobotComm dry-run` or `RobotComm dry-run`. |
| Stop | Stop polling/sending, close mock or socket transport, emit stopped. Real safe hold/stop behavior must be designed before live DX200 is enabled. |
| Exception handling | Protocol parse failures and socket errors produce `AppError`; reconnect behavior must be explicit. |
| Prohibited | Must not inherit QWidget, know about buttons, or send live motion by default. |

真实 DX200 通信保持禁用，除非同时满足：

- configuration explicitly selects `DX200RobotClient`;
- a UI-visible authorization or equivalent runtime gate has been satisfied;
- protocol parser/serializer behavior has been validated;
- the operator/user has approved live hardware communication.

legacy `timeSetEvent` 轮询应改为以下两种之一：

- a `QTimer` inside this worker thread for periodic protocol exchange; or
- event-driven socket reads plus scheduled command sends.

## DigitalTwinWorker

| Item | Design |
| --- | --- |
| Inputs | `RobotState`, `WorkpieceProfile`, `DigitalTwinSettings`, correction results. |
| Outputs | `digitalTwinStateChanged(ServiceState)`, optional `virtualLaserReady(VirtualLaserFrame)` later, optional `pointCloudReady(PointCloudSnapshot)` later. |
| Startup | First release should not start a heavy worker; use mock/no-op service. Real worker starts only when digital twin is enabled in a later phase. |
| Stop | Mock/no-op stop is immediate. Future real implementation uses cooperative cancellation and releases long-running RobotSim IPC or localization resources in worker thread. |
| Exception handling | Geometry/model errors should disable digital twin without killing robot/camera workers. |
| Prohibited | Must not call QWidget methods. Digital twin must not own localization solving or direct UI viewer resources. |

该 worker 后续替代 `virtualSceneThreadFunc()`。它应使用显式状态，而不是 `while(true)` 加全局变量。

## CorrectionControlWorker

| Item | Design |
| --- | --- |
| Inputs | `RobotState`, `WeldVisionPipelineResult`, `TrackingObservation`, workflow commands, optional `LvsObservation`. |
| Outputs | `correctionCommandReady(RobotCommand)`, `onlineCorrectionStateChanged(OnlineCorrectionPhase)`, `controlDebugData(ControlTrace)`, `controlError(AppError)`. |
| Startup | First release starts only for weld tracking main workflow, offline replay, and dry-run correction validation. |
| Stop | Cooperative cancellation; emit final state; clear session buffers. |
| Exception handling | Safety errors trigger workflow error state. Live stop/hold command behavior is deferred until real DX200 communication is authorized. |
| Prohibited | Must not own camera SDKs, sockets, or widgets. |

IBVS behavior in the first release:

- keep `IbvsService` interface and `IbvsState`;
- allow UI/status display to show unavailable/mock/no-op;
- do not implement the full Basler IBVS control loop;
- do not issue robot motion based on IBVS data.

该 worker 应持有会话内数据，替代 legacy 代码中 `DetPtsList`、`aRealPointList`、`aVirtualPointList`、`g_lvs_*`、`g_ibvs_*` 等全局变量。

## LogWorker

| Item | Design |
| --- | --- |
| Inputs | Log events, diagnostic traces, optional binary/image save requests. |
| Outputs | `logWriteFailed(AppError)`, optional `logFileRotated(Path)`. |
| Startup | Optional; started by the runtime support layer if async logging is enabled. |
| Stop | Flush queue, close files, emit stopped. |
| Exception handling | File IO failure should not crash control loops; report and degrade gracefully. |
| Prohibited | Must not redirect process-wide stdout/stderr unless explicitly required. |

## Signal/Slot 数据传递

跨线程通信使用 queued connection。

推荐 signal：

```text
CameraWorker::frameReady(Frame)
RobotCommunicationWorker::robotStateChanged(RobotState)
RobotCommunicationWorker::dryRunCommandRecorded(RobotCommand)
VisionWorker::visionResultReady(WeldVisionPipelineResult)
DigitalTwinWorker::digitalTwinStateChanged(ServiceState)
CorrectionControlWorker::correctionCommandReady(RobotCommand)
UiController::viewStateChanged(MainViewState)
UiController::errorRaised(AppError)
```

推荐命令 slot：

```text
CameraWorker::start(CameraConfig)
CameraWorker::stop()
RobotCommunicationWorker::connectRobot(RobotConnectionConfig)
RobotCommunicationWorker::sendCommand(RobotCommand)
VisionWorker::processFrame(Frame, RobotStateSnapshot)
DigitalTwinWorker::updateRobotState(RobotState)
CorrectionControlWorker::startWeldTrackingSession(WorkflowCommand)
CorrectionControlWorker::stopSession()
```

大图像不应被反复复制。实现可以使用共享所有权或 move 语义，但所有权必须清晰；worker 发出帧后不得再修改该帧，除非它持有私有副本。

## 安全退出策略

关闭顺序应显式：

1. UI disables start/action buttons and emits application stop.
2. `UiController` requests workflow stop.
3. `CorrectionControlWorker` stops issuing commands.
4. `RobotCommunicationWorker` records/stops dry-run commands and closes mock/socket transport.
5. `CameraWorker` stops acquisition and releases SDK resources.
6. `VisionWorker` drains or discards pending frames.
7. `DigitalTwinWorker` stops mock/no-op or simulation resources.
8. `LogWorker` flushes and closes files.
9. Threads quit and are waited with bounded timeouts.
10. Remaining failures are logged and surfaced to the user.

每个 worker 应暴露：

- `stopRequested` flag or equivalent;
- `stopped()` signal;
- idempotent `stop()`;
- bounded cleanup time;
- structured error reporting.

## 明确禁止项

- Worker 线程绝不能调用 QWidget 方法。
- 不允许没有取消机制的 raw 无限循环。
- Qt 实现中不允许使用 `CreateThread`。
- 机器人通信不允许使用 `timeSetEvent`。
- 不允许跨 worker 共享全局可变图像缓冲。
- 不允许把进程级状态标志作为主要 workflow 机制。
- 不允许在 UI 线程中阻塞式启动硬件。
- 默认不允许发送 live DX200 运动命令。
- 最小骨架不引入 OpenCascade/VTK viewer 或 RobotSim 内部依赖；PCL 点云求解归属 localization 模块边界。
- 除非显式启用可选 legacy adapter，否则不引入 HALCON 依赖。

## Legacy 映射

| Legacy 机制 | 新机制 |
| --- | --- |
| `CreateThread(... SnapThread ...)` | Optional `HalconCameraBackend` only if retained; not default. |
| `CreateThread(... OpencvThread ...)` | `VimbaTrackingCameraBackend` + `CameraWorker` + `VisionWorker` + UI image display signal. |
| `CreateThread(... IBVSThread ...)` | Later `BaslerIbvsCameraBackend` + `IbvsService`; first release mock/no-op only. |
| `virtualSceneThreadFunc()` | Later `DigitalTwinWorker`; first release mock/no-op only. |
| `timeSetEvent(... TCPTimerProc ...)` | `RobotCommunicationWorker` with worker-thread `QTimer` or socket events; dry-run/mock by default. |
| `SendMessage/PostMessage` | Qt queued signal/slot or controller event dispatch. |

