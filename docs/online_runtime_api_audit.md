# OnlineCor、OpenCV 线程与仿真 API 核查

## OpenCV 线程分层结论

当前在线检测没有把 QWidget 显示代码放入 Vision 算法：

- `Vision/CLegacyOnlineKcfDetector` 是纯 C++/OpenCV 服务，不依赖 Qt Widgets，不操作控件。
- `CVisionTrackingWorkerAdapter` 位于 runtime 适配层，在独立 QThread 中调用算法，并把结果转换为 `RuntimeTrackingResult`。
- `RuntimeTrackingResult::overlayImage` 是算法效果图 DTO；算法本身只返回 `cv::Mat processedImage`。
- `UiController` 负责把结果交给 `MainWidget`；`MainWidget` 才执行 `QPixmap::fromImage()` 和控件刷新。

因此在线 KCF 路径满足 Model/Service、Controller、View 分离。需要注意：

- `CVisionTrackingWorkerAdapter` 当前放在 UI 模块，属于应用适配器而非 Vision 算法，但不访问 QWidget。
- `UiController::handleFrameReady()` 还保留一条通用视觉预览流水线，用于离线页面展示；它不是 OnlineCor KCF 算法实现。
- `CameraFrame` 和 runtime DTO 使用 `QImage`，属于 Qt 数据类型依赖，不等于算法与 UI 控件耦合。

## J/C 旧协议

旧 `RobotCtrl::RevRobotData()` 使用：

```text
C e_robot,s_tracker,s_laser,s_filter,s_offline,OfflineStartCount,setHold,FirstDetSuccess
```

```text
J e_robot,s_tracker,s_laser,s_filter,s_offline,OfflineStartCount,setHold,FirstDetSuccess,jobname
```

每次请求后都读取并解析完整 D 回包。新工程对应：

- `CRobotProtocolCodec::encodeCalibrationAdvance()`
- `CRobotProtocolCodec::encodeJobStart()`
- `CRobotCommWorkerAdapter::sendCalibrationCommand()`
- `CRobotCommWorkerAdapter::sendJobStartCommand()`
- `CRuntimeCoordinator::sendCalibrationCommand()`
- `CRuntimeCoordinator::startRobotJob()`

J 命令保留旧 `sStart == 0` 前置条件；作业名不得为空、包含逗号或空白字符。

## 当前 OnlineCor 必须提供的仿真 API

下表是当前 `SimulatedVisionCallbacks` 每帧固定调用顺序。

| 顺序 | 新回调 | 原工程名称 | 作用 |
|---|---|---|---|
| 1 | `toNextWeldSeam` | `SwLSLSensor::toNextWeldSeam()` | 推进虚拟传感器到当前机器人位姿对应的下一扫描状态/焊缝位置。 |
| 2 | `generate_simu_laser1` | `SwLSLSensor::generate_simu_laser1()` | 生成当前帧仿真激光线；返回 false 时丢弃该帧。 |
| 3 | `getSeamPoint2d` | `SwLSLSensor::getSeamPoint2d(MyLaserImagePoint&)` | 返回仿真焊缝交点的图像坐标，供 250×200 KCF ROI 初始化和卡尔曼约束使用。 |
| 4 | `getCurrentVirtualLine` | 原工程组合：`SwLSLSensor::get_output_point2d()` / `CRobotleadingDlg::currentVirtualLine` | 返回当前仿真激光线二维点列，供模板绘制、Gabor 方向约束和 gap 先验使用。原工程没有确认存在同名 `getCurrentVirtualLine()` 方法。 |
| 5 | `getSeamIndex` | `SwLSLSensor::getSeamIndex()` | 返回扫描面或焊缝段编号，供 `OnlineExceptionHandling_beta` 迁移逻辑判断段变化。 |

这五项是当前标准 KCF OnlineCor runtime 的最小必需集合。

## 完全复刻旧 gap/仿真行为时需要增加的 API

| 原工程名称 | 作用 | 当前状态 |
|---|---|---|
| `SwLSLSensor::getWeldingSeamAssistInfo(SwWeldingSeamAssistInfo&)` | 获取右侧工件世界坐标法向，用于 gap PCA 方向和激光平面/工件平面交线方向。 | 尚未加入 `SimulatedVisionCallbacks`；当前 gap 使用通用点云算法。 |
| `SwLSLSensor::get_output_point3d()` | 获取当前仿真激光三维点，用于虚拟点云、局部定位和 ICP。 | OnlineCor KCF 最小链路不需要。 |
| `SwLSLSensor::generate_simu_laser()` | 旧仿真激光生成路径，主要用于虚拟点云或其他检测模式。 | 当前生产路径固定使用 `generate_simu_laser1()`。 |
| `SwLSLSensor::generate_simu_laser1(tmpTargetFaces)` | 按目标面集合生成仿真激光。 | 仅旧局部定位/特定工件路径使用。 |
| `SwLSLSensor::checkLaserSeamIntersection(...)` | 判断激光平面与焊缝是否相交。 | 非当前 OnlineCor KCF 必需。 |
| `SwLSLSensor::getTCPPosition_world(...)` | 获取仿真 TCP 世界坐标位置。 | 旧局部定位和扫描判断使用。 |
| `SwLSLSensor::getSeamLaserPlaneAngle()` | 判断是否可获得焊缝与激光平面夹角。 | 旧 gap 角度补偿代码当前被注释。 |
| `SwLSLSensor::getSeamAngle()` | 获取焊缝与激光平面夹角，单位 deg。 | 旧 gap 角度补偿代码当前被注释。 |

## 不是仿真 API、但对接时必须提供的数据

| 数据/接口 | 原工程来源 | 新工程用途 |
|---|---|---|
| 相机帧 | `m_ApiController.GetFrame()` → `CopyToImage_cv(..., cv_image)` | 发布为 `CameraFrame`，每帧送入 Vision worker。 |
| 同帧机器人位姿 | `curx_r...curRz_r` / `pose_TinB_nowGrab` | 构造 `toolInRobotBase`；位置 um，姿态 deg×10000。 |
| 变位机轴角 | `curS1/curS2` | 动态构造 `robotBaseToPositionerTcp`；单位 deg×10000。 |
| SENSORPARA | `SENSORPARA.txt` | 相机内参、激光平面和 `cameraInTool`。 |
| RBCALIB | `RBCALIB.DAT` 的 `////SRANG` | 机器人基座与变位机标定及 O 点坐标转换。 |
| 机器人状态/段结束 | D 回包中的 `traflag/corflag/detflag/offline/onlineReverseCompleteFlag` | 启动许可、O 完成、分段结束和反向检测完成。 |

## 旧对话框函数的职责映射

这些函数不是建议直接作为新仿真接口暴露，而是旧行为的定位入口：

| 原工程函数 | 作用 |
|---|---|
| `CRobotleadingDlg::CalthelaserpositionLoop()` | 旧在线检测总入口，包含检测、投影、gap、工件补偿和全局状态写入。 |
| `CRobotleadingDlg::getDetImagePoint(...)` | 根据 `det_type` 获取实际焊缝图像点。 |
| `CRobotleadingDlg::ConvertImgPt2SP(...)` | 图像点投影到当前变位机坐标。 |
| `CRobotleadingDlg::StationPos2RobotPos(...)` | 变位机坐标位姿转换到机器人基坐标。 |
| `CRobotleadingDlg::RobotPos2StationPos(...)` | 机器人基坐标位姿转换到变位机坐标。 |
| `CRobotleadingDlg::getWeldGap(...)` | 400×400 ROI 的旧 gap 检测入口。 |
| `CRobotleadingDlg::buildPiecewiseSlopePrior(...)` | 从仿真激光线构造分段斜率先验。 |
| `CRobotleadingDlg::getCurrentCenterLine(...)` | 提取当前实际激光中心线。 |
| `CRobotleadingDlg::Offcorstart_OnChangeVirtualStatus(...)` | 切换旧虚拟场景在线记录状态。新工程应由显式状态机替代。 |
