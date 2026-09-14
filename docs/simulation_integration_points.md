# 仿真接口对接清单

本文整理新工程中仍需要仿真或离线回放输入的部分，说明旧工程中的对应名称、需要提供的数据，以及新工程应该插入的位置。旧工程 `SonTech/` 只作为只读参考，不在旧工程内修改。

## 1. 总览

当前新工程的真实相机、RobotComm、在线/离线纠偏控制器已经拆成独立 worker。真正需要手动对接仿真的地方主要有两类：

1. 视觉仿真：给在线/离线 KCF 扫描检测提供旧工程 VirtualLaserSensor 等价输出。
2. 机器人/控制柜仿真：在不连接真实机器人时提供 D 回包状态、命令回执和若干握手信号。

如果后续接入真实相机和真实机器人，机器人状态和命令回执不需要仿真；但目前 `det_type=2` 的在线/离线视觉路径仍需要仿真 observation，除非继续补真实图像检测入口。

## 2. 视觉仿真：DigitalTwin observation 与 VirtualLaserSensor 兼容回调

### 2.1 新工程入口

新工程当前优先使用 `DigitalTwin` 产出的轻量视觉 observation。`Vision` 不直接连接
RobotSim，也不直接 include RobotSim transport/service；RobotSim 通信只允许发生在
`DigitalTwin` 模块内。运行时可通过：

- `DigitalTwin/include/DigitalTwinVisionTypes.h`
  - `DigitalTwinVisionLaserObservation`
  - `makeVisionLaserObservation(const DigitalTwinVirtualReferenceResult&)`
- `UI/include/RuntimeCoordinator.h`
  - `CRuntimeCoordinator::publishDigitalTwinVisionObservation(...)`
- `UI/src/RuntimeWorkerAdapters.cpp`
  - `CVisionTrackingWorkerAdapter::updateDigitalTwinVisionObservation(...)`
  - `CVisionTrackingWorkerAdapter::makeObservation(...)`

接入。`publishDigitalTwinVisionObservation(...)` 使用 queued 调用投递到 Vision worker
线程；worker 只保存最近一份按值拷贝的快照，不保存外部指针或引用。

旧 VirtualLaserSensor 风格回调仍保留为手动/离线仿真兼容路径：

- `UI/include/RuntimeTrackingTypes.h`
  - `SimulatedVisionCallbacks`
- `UI/src/RuntimeWorkerAdapters.cpp`
  - `CVisionTrackingWorkerAdapter::makeObservation()`
- `UI/include/RuntimeCoordinator.h`
  - `CRuntimeCoordinator::setSimulatedCallbacks(...)`

当 DigitalTwin observation 有效时，优先使用 DigitalTwin；只有没有有效
DigitalTwin observation 时才调用 callbacks。callbacks 调用顺序固定为：

```text
toNextWeldSeam()
-> generate_simu_laser1()
-> getSeamPoint2d()
-> getCurrentVirtualLine()
-> getSeamIndex()
-> getScanLineChangePoint()
```

其中前五个是旧路径核心语义；`getScanLineChangePoint()` 是为旧 `sim_scan_line_change_point` 增补的输入。

### 2.2 需要实现的回调

| 新工程回调 | 旧工程名称/等价来源 | 需要提供什么 | 单位/格式 | 用途 | 新工程插入点 |
| --- | --- | --- | --- | --- | --- |
| `toNextWeldSeam()` | `VirtualLaserSensor->toNextWeldSeam()` | 推进仿真传感器到当前/下一条焊缝扫描面。 | 无返回值。 | 复刻旧 OpencvThread 每帧先推进仿真焊缝的活动路径。 | `SimulatedVisionCallbacks::toNextWeldSeam` |
| `generate_simu_laser1()` | `VirtualLaserSensor->generate_simu_laser1()` | 生成当前相机/机器人姿态下的仿真激光线。 | `bool`，true 表示当前帧可用。 | KCF 初始化模板、中心线/gap 检测、空间投影前置数据。 | `SimulatedVisionCallbacks::generate_simu_laser1` |
| `getSeamPoint2d(...)` | `VirtualLaserSensor->getSeamPoint2d(sp)` | 当前焊缝点图像坐标。 | `digital_twin::DigitalTwinImagePoint2D_px { x_px, y_px }`，单位 pixel。 | 固定 `250x200` ROI 中心、绿色十字显示、KCF 跟踪初值。 | `SimulatedVisionCallbacks::getSeamPoint2d` |
| `getCurrentVirtualLine()` | 旧工程中 `generate_simu_laser1()` 后得到的当前仿真激光线；离线保存为 `simulatedLaser.txt` | 当前帧仿真激光线点列。 | `std::vector<digital_twin::DigitalTwinImagePoint2D_px>`，单位 pixel，至少 2 点。 | 生成 KCF 仿真模板；gap 检测局部中心线也依赖该类点列。 | `SimulatedVisionCallbacks::getCurrentVirtualLine` |
| `getSeamIndex()` | `VirtualLaserSensor->getSeamIndex()` | 当前焊缝段/扫描线编号。 | `int`，无效为 `-1`。 | `OnlineExceptionHandling_beta` 等价逻辑中判断仿真扫描面/焊缝段变化；无切换点时可降级触发分段。 | `SimulatedVisionCallbacks::getSeamIndex` |
| `getScanLineChangePoint()` | 旧全局变量 `sim_scan_line_change_point` | 扫描线/焊缝段变化处的空间点。 | `std::optional<digital_twin::DigitalTwinPoint3D_mm>`，单位 mm。 | 两段 ICP 分段定位 `curve_index`，对应旧 `OnlineExceptionHandling_beta` 对 `sim_scan_line_change_point` 的处理。 | `SimulatedVisionCallbacks::getScanLineChangePoint` |

### 2.3 旧工程参考位置

| 旧工程文件 | 参考内容 |
| --- | --- |
| `SonTech/sontech-works-version-storage/SWv1/robotleadingDlg.cpp` | `VirtualLaserSensor->toNextWeldSeam()`、`generate_simu_laser1()`、`getSeamPoint2d(sp)`、`getSeamIndex()` 的活动调用。 |
| `SonTech/sontech-works-version-storage/SWv1/robotleadingDlg.cpp` | `sim_scan_line_change_point` 全局变量定义与更新；`OnlineExceptionHandling_beta(...)` 中消费该点。 |
| `SonTech/sontech-works-version-storage/SWv1/3DSimulationDlg.cpp` | 三维仿真对 `plslsens->toNextWeldSeam()`、`generate_simu_laser1()`、`getSeamPoint2d(...)` 的调用。 |
| `SonTech/sontech-works-version-storage/SWv1/robotleadingDlg.cpp` | `simulatedLaser.txt`、`simulatedWeldpt.txt` 写出逻辑。 |

### 2.4 推荐接法

如果是接离线仿真文件：

- `simulatedWeldpt.txt` → `getSeamPoint2d()`
- `simulatedLaser.txt` → `getCurrentVirtualLine()`
- 帧索引推进 → 建议放在 `toNextWeldSeam()` 或 `generate_simu_laser1()` 内，但要保证每帧只推进一次。
- 没有焊缝段编号时，`getSeamIndex()` 返回 `-1`。
- 没有切换点时，`getScanLineChangePoint()` 返回 `std::nullopt`。

如果是接实时仿真引擎：

- 推荐由 `DigitalTwin` 查询 RobotSim，得到 `DigitalTwinVirtualReferenceResult`。
- 调用 `digital_twin::makeVisionLaserObservation(...)` 转成
  `DigitalTwinVisionLaserObservation`。
- 调用 `CRuntimeCoordinator::publishDigitalTwinVisionObservation(...)` 投递给 Vision worker。
- `frameId` 能对应相机帧时优先同帧使用；`frameId < 0` 或无法严格匹配时，
  worker 使用最新有效 observation，并在结果 message 中标注 fallback。
- 查询失败不要清空当前帧数据，只记录失败；下一帧仍按最新有效快照或 callbacks 兜底。

如果是接手动/离线兼容 callbacks：

- callbacks 持有的仿真对象应由 `UiController` 或专门的 runtime adapter 持有。
- 在 `UiController::initialize()` 里、`m_runtimeCoordinator.start()` 前调用：

```cpp
i2r_rlc::ui::SimulatedVisionCallbacks callbacks;
callbacks.toNextWeldSeam = ...;
callbacks.generate_simu_laser1 = ...;
callbacks.getSeamPoint2d = ...;
callbacks.getCurrentVirtualLine = ...;
callbacks.getSeamIndex = ...;
callbacks.getScanLineChangePoint = ...;
m_runtimeCoordinator.setSimulatedCallbacks(callbacks);
```

如果希望从 `main()` 或外部模块设置，建议给 `UiController` 增加一个轻量转发接口：

```cpp
void UiController::setSimulatedVisionCallbacks(
    const SimulatedVisionCallbacks& callbacks)
{
    m_runtimeCoordinator.setSimulatedCallbacks(callbacks);
}
```

### 2.5 Vision 内部对仿真激光线的依赖：KCF / Gabor / gap

注意：DigitalTwin observation 或 `getCurrentVirtualLine()` 返回的仿真激光线不是只
为了让 runtime 生成空间点。当前 Vision 的旧在线检测实现会把同一条线作为
`DigitalTwinVisionLaserObservation::laserLineImagePoints_px`，同时喂给 KCF、Gabor
和 gap 前置链路。Vision 内部如需旧 `MyLaserImagePoint`，只在 `.cpp` 内部局部转换，
不再把它作为跨模块仿真公共 API。

新工程对应文件：

| 文件 | 入口 | 与仿真激光线的关系 |
| --- | --- | --- |
| `Vision/include/track/LegacyOnlineKcfDetector.h` | `DigitalTwinVisionLaserObservation` | 消费 `weldPointImage_px`、`laserLineImagePoints_px`、`seamIndex`。 |
| `Vision/src/track/LegacyOnlineKcfDetector.cpp` | `CLegacyOnlineKcfDetector::process(...)` | 在线 KCF/Gabor/gap 的主入口。 |
| `Vision/src/preprocess/ImageDenoise.cpp` | `CImageDenoise::gaborDenoise(...)` | 用仿真激光线和仿真焊点估计 Gabor 方向。 |
| `Vision/src/stripe/DpStripeCenterlineDetector.cpp` | `CDpStripeCenterlineDetector::detectCenterline(...)` | gap 时从真实/回放灰度图检测中心线。 |
| `Vision/src/feature/PointCloudGapWidthDetector.cpp` | `CPointCloudGapWidthDetector::detect(...)` | 把投影后的中心线点云用于 mm gap 计算。 |

#### 2.5.1 KCF 使用仿真激光线做模板

KCF 不是从整图盲找焊缝点。当前旧路径是：

```text
simulatedWeldPoint
-> centeredRoi(simulatedWeldPoint, 250, 200)
simulatedLaserPoints
-> simulatedTemplate(fullImageSize, simulatedLaserPoints)
-> CKcfTracker::init(roi, template)
```

也就是说：

- `getSeamPoint2d()` 提供 KCF 初始 ROI 的中心点，单位 pixel。
- `getCurrentVirtualLine()` 提供 KCF 初始化模板，单位 pixel。
- `simulatedLaserPoints` 至少 2 个点，否则 `CLegacyOnlineKcfDetector` 直接返回 `simulated observation is invalid`。
- KCF 每超过 50 帧会按旧语义重新初始化一次。

因此你接仿真时，仿真激光线必须和当前图像帧、当前机器人/相机姿态对应。否则 KCF 模板会和真实图像错位。

#### 2.5.2 Gabor 使用仿真激光线估计滤波方向

Gabor/Otsu 路径也依赖同一条仿真激光线。当前流程在 `LegacyOnlineKcfDetector.cpp::enhancedImage(...)` 中：

```text
gray8(currentRoi)
-> CImageDenoise::gaborDenoise(roiGray, simulatedLaserPoints, simulatedWeldPoint)
-> Otsu 二值化
-> 按旧语义 y 偏移 -5 pixel 写回原图
-> KCF update(processedImage)
```

`CImageDenoise::estimateGaborFilterAngle(...)` 的旧兼容逻辑是：

1. 在 `simulatedLaserPoints` 中按 `x` 方向距离寻找最接近 `simulatedWeldPoint.px` 的点，作为焊点索引。
2. 在焊点索引左右各取 `neighborWindow` 个点的平均位置。
3. 按旧公式使用 `atan(dy / dx)`，不使用 `atan2`。
4. 计算：

```text
FilterAngle = LeftAngle + (180 - RightAngle - LeftAngle) / 2
```

所以 Gabor 对仿真线的要求比 KCF 更高：

- 点列必须是整幅图像坐标，不是 ROI 局部坐标。
- 点列最好沿激光线连续排序。
- 焊点左右都要有邻域点；否则会失败为 `missing left neighbor points` 或 `missing right neighbor points`。
- 焊点应在仿真激光线附近，尤其是 `x` 方向附近。

Gabor 失败时，KCF 不一定失败，但会退化为：

```text
KCF succeeded; Gabor enhancement unavailable
```

这会影响真实图像增强、KCF 稳定性和后续 gap ROI 的可靠性。

#### 2.5.3 gap 不直接用仿真线算宽度，但受 KCF/Gabor 间接影响

当前 gap 宽度不是直接从 `simulatedLaserPoints` 计算。它的链路是：

```text
KCF/Gabor 输出 weldPointPixel
-> weldPointPixel 周围 400x400 ROI
-> CDpStripeCenterlineDetector 从灰度图检测中心线
-> SENSORPARA + RobotState + RBCALIB 投影为变位机坐标点云
-> CPointCloudGapWidthDetector 计算 mm gap
```

但 gap 仍然依赖仿真激光线的质量，因为：

- 仿真焊点决定 KCF 初始 ROI。
- 仿真激光线决定 KCF 模板。
- 仿真激光线和焊点决定 Gabor 方向。
- KCF/Gabor 输出的 `weldPointPixel` 决定 gap 的 `400x400` ROI。

如果仿真线不准确，gap 常见表现是：

```text
KCF/Gabor succeeded; gap degraded to 0 mm: ...
```

这按旧工程语义不会进入 Error，样本仍有效，gap 写 `0.0 mm`。

#### 2.5.4 仿真激光线最小质量要求

| 要求 | 原因 |
| --- | --- |
| 与当前相机帧同一时刻、同一坐标系 | KCF 模板和真实图像必须对齐。 |
| 使用整图 pixel 坐标 | KCF 模板绘制和 Gabor 方向估计都按整图坐标工作。 |
| 至少 2 点 | KCF 最低有效条件。 |
| 焊点左右有足够邻域点 | Gabor 方向估计需要左右邻域。 |
| 点列沿激光线连续排序 | Gabor 左右邻域和模板绘制更稳定。 |
| 焊点在仿真线附近 | Gabor 按 X 方向最近点寻找焊点索引。 |
| 点列覆盖 `250x200` KCF ROI 附近 | 避免 KCF 模板与跟踪区域不一致。 |
| 真实/回放图像中对应区域有可检测激光条纹 | gap 的中心线仍来自灰度图，不是直接来自仿真线。 |

如果你接的是旧 `simulatedLaser.txt`，建议每帧返回完整线，而不是只返回 ROI 内点；模板绘制会自动按图像范围裁剪，Gabor 也需要焊点左右上下文。

## 3. 视觉检测触发状态仿真

视觉 worker 不会无条件处理每帧，它会先复刻旧 OpencvThread 顶层触发条件。

### 3.1 新工程判断位置

- `Control/src/TrackingTriggerPolicy.cpp`
  - `CTrackingTriggerPolicy::evaluate(...)`

### 3.2 需要仿真的 RobotState 字段

| 新工程字段 | 旧工程/机器人字段 | 需要值 | 作用 |
| --- | --- | --- | --- |
| `RobotState::traflag` | `traflag` | `1` | 表示机器人轨迹/检测流程处于可采集状态。 |
| `RobotState::sMode` | `sMode` | `2` | PLAY 模式。 |
| `RobotState::sHold` | `sHold` | `0` | 机器人未暂停。 |
| `RobotState::sStart` | `sStart` | `1` | JBI 程序运行中。 |
| `RuntimeTrackingOptions::virtualStatus` | 旧 `m_virtual_status` | `OnlineRecord` 或 `OnlineRecordWithLocalization` | 等价旧 `VS_ONLINE_REC / VS_ONLINE_REC_MOD`。 |

新工程插入点：

- 有真实 RobotComm：不用手动插，`D` 回包解析后自动投递。
- 无真实机器人：调用 `CRuntimeCoordinator::publishRobotState(state)`，或实现 fake `IRobotCommandTransport` 在每次事务里返回带这些字段的 `RobotState`。

## 4. 在线纠偏流程需要仿真的机器人信号

在线纠偏本身不应该伪造算法结果，但没有真实机器人时需要仿真 D 回包中的阶段信号。

### 4.1 初始检测结束

| 新工程字段 | 旧工程字段/语义 | 需要值 | 新工程消费位置 |
| --- | --- | --- | --- |
| `RobotState::onlineReverseCompleteFlag` | `onlineReverseCompleteFlag` | 从 `0` 到非 0 | `Control/src/OnlineCorrectionController.cpp::onRobotStateChanged()` |

说明：

- 反向扫描完成由机器人置 `onlineReverseCompleteFlag=1`。
- IBVS 引导到起始点后也由机器人置 `onlineReverseCompleteFlag=1`。
- 新工程做了边沿/锁存处理；仿真时应先给一次 `0`，再给 `1`，不要一直只给 `1`。

### 4.2 O 示教点确认

| 新工程字段 | 旧工程语义 | 需要值 | 作用 |
| --- | --- | --- | --- |
| `RobotState::offline` | `D.offline` | `5` | 表示整批 O 示教点更新完成。 |

新工程消费位置：

- `OnlineCorrectionController::onRobotCommandReceipt(...)`
- `OfflineCorrectionWorkerAdapter::handleRobotCommandReceipt(...)`

仿真方式：

- fake transport 的 `transact(...)` 返回 `RobotCommandReceipt{ sent=true, responseReceived=true, completed=true }`，同时写出 `state.offline=5`。

### 4.3 在线分段结束

| 新工程字段 | 旧工程语义 | 触发条件 | 作用 |
| --- | --- | --- | --- |
| `RobotState::corflag` | 纠偏/分段标志 | `2` | 中间段结束。 |
| `RobotState::detflag` | 检测结束标志 | `2` | 与 `corflag=2` 同时到位，触发中间段结束。 |
| `RobotState::corflag` | 纠偏/分段标志 | `3` | 最终段结束。 |
| `RobotState::detflag` | 检测结束标志 | `3` | 与 `corflag=3` 同时到位，触发最终段结束。 |
| `RobotState::detflag` | 检测复位 | `0` | 结束 P 确认后允许进入下一段或 Completed。 |

新工程消费位置：

- `Control/src/OnlineCorrectionController.cpp::onRobotStateChanged()`

仿真注意：

- `corflag/detflag=2/2` 或 `3/3` 要只触发一次。
- 收到结束 P 的回包后，再给 `detflag=0`，否则状态机会停在等待复位阶段。

## 5. 离线纠偏需要仿真的机器人信号

旧工程离线纠偏检测会自动监听机器人端检测结束信号，然后自动匹配计算并发送 O。新工程已保留该语义。

| 新工程字段 | 旧工程语义 | 需要值 | 新工程消费位置 |
| --- | --- | --- | --- |
| `RobotState::detflag` | 机器人端离线检测结束 | `2` 或 `3` | `UI/src/RuntimeWorkerAdapters.cpp::COfflineCorrectionWorkerAdapter::updateRobotState()` |
| `RobotState::detflag` | 检测信号释放 | `0` | 释放锁存，允许下一轮。 |
| `RobotState::offline` | O 示教点更新完成 | `5` | `COfflineCorrectionWorkerAdapter::handleRobotCommandReceipt()` |

仿真时序建议：

```text
开始离线纠偏检测
-> 持续发布满足视觉触发条件的 RobotState + CameraFrame
-> 发布 detflag=2 或 detflag=3
-> Offline worker 自动 stopTracking + computeCorrection + 发送 O
-> fake transport 返回 D.offline=5
-> 发布 detflag=0
```

## 6. 系统标定采集需要仿真的机器人信号

系统标定只迁移采集功能，不迁移 HALCON 标定计算。

| 新工程字段 | 旧工程语义 | 需要值 | 新工程消费位置 |
| --- | --- | --- | --- |
| `RobotState::s_Bcalib` | 自动标定采集触发 | `0 -> 1` 上升沿采一张 | `Control/src/CalibrationCaptureController.cpp::onRobotStateChanged()` |
| `RobotState::s_Bcalib` | 释放锁存 | `1 -> 0` | 允许下一张继续触发。 |
| 当前相机帧 | 旧 OpencvThread 当前图像 | 有效 `CameraFrame` / `cv::Mat` | `UiController::processAutomaticCalibrationRobotState()` |
| 机器人位姿 | `curx_r...curS2` | 有效姿态与变位机轴 | 写入 `CalibOriginData.txt`。 |

仿真注意：

- 自动标定红框预览阶段不采集；点击确认后才进入等待 `s_Bcalib`。
- `s_Bcalib` 持续为 1 不应重复采集，必须回 0 后再给 1。

## 7. 保存示教轨迹需要仿真的机器人信号

保存示教轨迹复刻旧 `RobotCtrl::RevRobotData()` 中 `s_Bsavetraj` 自动记录语义。

| 新工程字段 | 旧工程语义 | 需要值 | 新工程消费位置 |
| --- | --- | --- | --- |
| `RobotState::s_Bsavetraj` | 保存轨迹中 | `1` | `Control/src/TeachTrajectoryRecorder.cpp::onRobotStateChanged()` |
| `RobotState::s_Bsavetraj` | 轨迹段结束 | 非 `1` | 追加末端 `_W/_T` 点并释放段锁存。 |
| `RobotState::sStart` | JBI 运行中 | `1` | 追加 `AllPts.txt`。 |
| `curx_r/cury_r/curz_r` | 当前机器人位置 | um | 写入 `_W/_T` 时转 mm。 |
| `curRx_r/curRy_r/curRz_r/curS1/curS2` | 姿态/变位机轴 | deg*10000 | 写入 `_W/_T` 时转 deg。 |

新工程输出：

- `data/trajectory_capture/SeamTraj.txt`
- `data/trajectory_capture/SeamTeach.txt`
- `data/trajectory_capture/AllPts.txt`
- 保存后复制为 `<name>_W.txt` / `<name>_T.txt`

## 8. RobotComm fake transport

如果没有真实控制柜，需要仿真旧 `N/O/P/C/J` 请求和 `D` 回包事务。

### 8.1 新工程入口

- `RobotComm/include/RobotCommandTransport.h`
  - `IRobotCommandTransport`
- `UI/include/RuntimeCoordinator.h`
  - `CRuntimeCoordinator(std::unique_ptr<IRobotCommandTransport>)`

测试中已有示例：

- `tests/test_runtime_tracking_flow.cpp`
- `tests/test_robot_legacy_commands.cpp`
- `tests/test_offline_runtime_worker_flow.cpp`

### 8.2 需要仿真的命令类型

| 命令 | 旧工程/协议 | 仿真需要做什么 |
| --- | --- | --- |
| `N` | 周期状态轮询 | 返回完整 `D` 状态，驱动 UI、视觉触发、各 worker 状态机。 |
| `J` | 启动控制柜 JBI | 记录请求字符串，返回 D；可按场景把 `sStart=1`。 |
| `C` | 自动标定下一点 | 记录请求字符串，返回 D；随后由仿真状态把 `s_Bcalib` 拉高。 |
| `O` | 离线/在线初始示教点更新 | 逐条记录 O；最终点后返回或发布 `offline=5`。 |
| `P` | 在线实时纠偏量 | 记录最新 P；分段结束 P 后应配合 `detflag` 复位。 |

推荐 fake transport 行为：

```text
connect() -> true
isConnected() -> true
transact(type, request, pointIndex, state):
  保存 request 原文
  根据 type 和当前脚本场景修改 *state
  返回 RobotCommandReceipt{sent=true, responseReceived=true, completed=...}
```

## 9. 相机帧仿真/离线回放

相机帧不是旧 VirtualLaserSensor 回调的一部分，但视觉链路必须有图像。

| 输入 | 新工程入口 | 说明 |
| --- | --- | --- |
| 离线图片目录 | replay/测试内部入口 | 用于人工检查算法烟测；当前主 UI 不再暴露旧“图像/机器人”页。 |
| 单帧/脚本帧 | `CRuntimeCoordinator::publishCameraFrame(frame)` | 测试或自定义仿真可直接发布 `CameraFrame`。 |
| 真实 Allied 相机 | `UiController::setLiveCameraDevice(AlliedCameraDevice)` | 需要 Vimba X SDK 和真实相机。 |

注意：当前 `det_type=2` 的 runtime KCF 仍在 `makeObservation()` 里要求
DigitalTwin 或 VirtualLaserSensor 等价 observation。也就是说，只有真实相机图像而没有
DigitalTwin observation，也没有上述视觉仿真回调时，在线/离线纠偏检测会返回：

```text
simulated observation unavailable
```

若要完全摆脱仿真，需要在 `CVisionTrackingWorkerAdapter::processDetType2()` 中增加真实图像检测分支，用真实 KCF/Gabor/中心线结果生成：

- `weldPointPixel`
- `current laser line`
- `stationPoint`
- 可选 `gapWidthMm`

然后不再调用 `makeObservation()`。

## 10. 建议的手动接入顺序

1. 先接 RobotComm fake transport 或真实机器人 D 状态，确认 UI 状态监控、J/C/N/O/P 字符串可见。
2. 接 CameraFrame：离线图片或真实 Allied 相机，确认 UI 能显示实时/回放图像。
3. 优先接 `DigitalTwinVisionLaserObservation`；如果暂时无法接 DigitalTwin，再接
   `SimulatedVisionCallbacks` 六个兼容回调，先让 `det_type=2` 输出绿色十字和 station point。
4. 在线纠偏仿真：
   - 先发布 `onlineReverseCompleteFlag: 0 -> 1`
   - fake O 最终回 `offline=5`
   - 在线阶段发布 `corflag/detflag`
5. 离线纠偏仿真：
   - 检测中发布 `detflag=2/3`
   - fake O 最终回 `offline=5`
6. 系统标定仿真：
   - 自动模式按 `s_Bcalib: 0 -> 1 -> 0 -> 1...`
7. 保存示教轨迹仿真：
   - 发布连续位姿和 `s_Bsavetraj`，检查 `SeamTraj.txt / SeamTeach.txt`。

## 11. 最小对接核对表

- [ ] 有有效图像帧：`CameraFrame.image` 或 `previewImage` 非空。
- [ ] `RobotState` 满足视觉触发：`traflag=1, sMode=2, sHold=0, sStart=1`。
- [ ] `virtualStatus` 为 `OnlineRecord` 或 `OnlineRecordWithLocalization`。
- [ ] 优先提供 `DigitalTwinVisionLaserObservation.valid=true`；若走兼容回调，
      `generate_simu_laser1()` 返回 true。
- [ ] `weldPointImage_px` 或 `getSeamPoint2d()` 返回 true 且坐标在图像范围内。
- [ ] `laserLineImagePoints_px` 或 `getCurrentVirtualLine()` 返回整图 pixel 坐标下的当前仿真激光线，至少 2 个点，用于 KCF 模板初始化。
- [ ] 仿真激光线与焊点同帧、同坐标系，且覆盖 `250x200` KCF ROI。
- [ ] 仿真激光线在焊点左右有足够连续邻域点，用于 Gabor 方向估计；不要只返回 ROI 局部裁剪线。
- [ ] 回放/真实图像在焊点附近 `400x400` 区域有可检测激光条纹，否则 gap 会按旧语义降级为 `0.0 mm`。
- [ ] 需要两段 ICP 时，`scanLineChangePoint_mm` 或 `getScanLineChangePoint()` 返回有效 mm 空间点。
- [ ] 在线初始检测结束由 `onlineReverseCompleteFlag` 边沿触发。
- [ ] O 更新完成由 `offline=5` 确认。
- [ ] 离线检测自动结束由 `detflag=2/3` 触发。
- [ ] 自动标定采集由 `s_Bcalib` 上升沿触发。
- [ ] 保存示教轨迹由 `s_Bsavetraj` 和连续位姿驱动。
