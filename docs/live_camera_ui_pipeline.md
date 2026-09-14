# 实时相机、KCF 与 UI 显示链路

## 当前链路

```text
ICameraDevice
  -> CCameraWorkerAdapter（Camera QThread，默认约 30 fps）
  -> CameraFrame
     -> CRuntimeCoordinator::frameReady
        -> UiController::handleRuntimeFrameReady
           -> MainWidget::showFrame（实时原图）
     -> CVisionTrackingWorkerAdapter::submitFrame
        -> CLegacyOnlineKcfDetector
        -> RuntimeTrackingResult
           -> UiController::showRuntimeTrackingResult
              -> MainWidget::showFrame（KCF 效果图 + 绿色十字）
```

算法没有直接操作 QWidget。绿色十字由 runtime 可视化适配层绘制：

- 中心为 KCF 输出焊缝图像点；
- 横、竖方向各 30 pixel；
- 绿色；
- 线宽 2 pixel。

## 接入 Allied/Vimba 相机

启用相机后端：

```text
-DI2R_RLC_ENABLE_OPENCV=ON
-DI2R_RLC_ENABLE_CAMERA_ALLIED=ON
```

在 `UiController::initialize()` 之前注入设备：

```cpp
#include "AlliedCameraDevice.h"

auto device = std::make_unique<i2r_rlc::camera::AlliedCameraDevice>();
controller.setLiveCameraDevice(std::move(device), 33);
controller.initialize();
```

`33 ms` 约为 30 fps。设备所有权会转交 Camera worker，设备的 `start()`、
`latestFrame()` 和 `stop()` 都在 Camera 线程执行。

## 相机帧与 RobotState 时间同步设计

正式在线运动时不能简单使用“算法处理时最新的 RobotState”。推荐实现：

1. 使用进程统一的 `std::chrono::steady_clock`。
2. 相机 SDK 回调收到帧时记录 `captureTimestampNs`，不要在 UI 收到帧时才记录。
3. RobotComm 完整解析每条 D 后立即记录 `receivedTimestampNs`。
4. Robot worker 保存最近 1~2 秒的 `TimedRobotState` 环形队列。
5. Vision 收到相机帧时，按帧时间查找前后两条机器人状态。
6. 位置和变位机轴角做线性插值；姿态转换为四元数后做 SLERP。
7. 将插值后的机器人状态与图像帧绑定，后续投影始终使用该快照。
8. 找不到包围帧时间的状态，或最近时间差超过配置阈值时丢弃该帧。

建议诊断字段：

```text
cameraTimestampNs
robotStateBeforeTimestampNs
robotStateAfterTimestampNs
cameraRobotSkewMs
interpolated
```

首轮现场测试可先把允许时间差设为 `30 ms`，记录统计后再调整。若相机支持硬件
时间戳、编码器触发或机器人触发信号，应优先使用硬件同步，再由主机时间戳做诊断。

## 初始检测完成语义

初始阶段不由 Vision 产生 `detectionFinished`：

- 反向扫描模式：机器人完成反向扫描后在 D 回包中把
  `onlineReverseCompleteFlag` 置 1。
- IBVS 起点模式：IBVS 引导到起始点后，机器人同样把该字段置 1。

OnlineCor 收到该状态后：

- 普通反向扫描要求初始检测队列非空；
- IBVS 起点模式允许直接使用机器人当前位姿规划；
- 随后生成 O 示教点命令。
