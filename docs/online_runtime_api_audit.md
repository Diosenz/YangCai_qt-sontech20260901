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


## 在线二维检测逐帧诊断（2026-10-20）

本诊断用于定位绿色十字抖动最先出现的环节。它只读取算法阶段快照，不改变 KCF、
Kalman、投影、ICP、轨迹或机器人命令。`Vision` 负责数值来源；runtime 负责绑定冻结
上下文；在线 worker 的记录器负责文本持久化；独立可回收线程负责图像编码和写盘。

### 配置与生效边界

`config/workflows/online_vision_diagnostics.txt` 在每次启动在线纠偏时读取，与会话配置
一起固定；修改不影响正在运行的会话。文件缺失时采用以下默认值；未知键、重复键和
非法数值会报配置错误，解析失败不修改已有配置快照。

| 键 | 默认值 | 范围与含义 |
| --- | --- | --- |
| `enabled` | `true` | 新增诊断总开关；关闭后原有会话日志仍写入 |
| `images_enabled` | `true` | 无损原图、增强图成对采样开关 |
| `image_sample_every_n` | `15` | 1–1000000；从首个到达检测器的帧开始按处理序号采样，不按相机 frameId 取模 |
| `max_image_pairs` | `30` | 0–10000；每会话图像对提交上限，0 关闭图像 |
| `max_image_bytes` | `33554432` | 0–1073741824；PNG 实际写入字节预算，0 关闭图像 |

默认最多对前 450 个到达检测器的处理帧进行 30 次间隔采样，可能因无增强图、
队列满或预算不足而少于 30 对。需要短时逐帧观察时，可设 `image_sample_every_n=1`
并减小 `max_image_pairs`。这是有限采样窗口，不是异常前后自动录像；窗口后数值仍逐帧记录。
配置不会自动清理任何历史会话或图像。

### 输出与字段解释

新增文件位于原有 `output/online_correction/<session>/` 下：

- `vision_diagnostics.tsv`：每个送达在线 worker 的跟踪结果一行，包括无效和未触发结果。
  相机背压替换的 pending 帧、未进入 Vision 的同步失败帧不产生检测记录；该文件不是相机全帧清单。
- `vision_images/sample_<序号>_session_<会话>_frame_<帧号>_source.png`：送入处理的原图。
- 同名 `_enhanced.png`：KCF 实际输入的整幅图，包含 ROI 内 Gabor/Otsu 和 ROI 外原始灰度。
- `vision_images/image_manifest.tsv`：后台写入完成状态。TSV 中 `queued` 仅表示已入队，
  必须以 manifest 中 `saved` 和两个可读取的 PNG 确认保存完成。
- `session_summary.txt`：增加诊断 schema、配置快照、行数、图像提交数及后台错误。

数值表通过 `session_id/frame_id/purpose` 关联本帧；关键阶段如下：

| 字段组 | 语义 |
| --- | --- |
| `sim_x/y_px` | 本帧仿真焊缝先验点 |
| `response_x/y_px` | KCF 响应产生的候选点，尚未执行大跳变拒绝和融合 |
| `before_fusion_x/y_px` | 大跳变拒绝及边界处理之后、Kalman 融合之前的 KCF 点 |
| `final_x/y_px` | 融合及其跳变保护之后的实际输出，即绿色十字和投影输入 |
| `response_peak` | 本帧相关响应峰值，不是成功概率 |
| `normalized_peak/observation_noise` | 本次实际融合使用的峰值归一化结果和观测噪声 |
| `kcf_weight_x/y` | 实际 Kalman 增益对角项，不重新估计；`prior_weight_x/y` 为 1 减该值 |
| `fusion_applied/kalman_seeded` | 区分实际融合与首帧状态初始化；未融合时权重和噪声留空 |
| `template_reinitialized/frames_since_initialize` | 模板重建标记和本帧更新前计数，重建帧计数为 0 |
| `correlation_jump_rejected/fusion_jump_rejected` | 区分响应候选点与融合结果是否被保护逻辑退回；此时权重不代表最终点的线性组合 |
| `gabor_succeeded/enhancement_roi_*/final_roi_*/anchor_*` | 预处理可用性、实际增强范围、输出窗口和模板归一化锚点 |
| `positioner_tcp_*/robot_base_*/station_base_*` | 同一最终像素点在变位机 TCP、机器人固定基座和固定变位机基座中的投影，单位 mm |
| `robot_x/y/z_mm/robot_rx/ry/rz_deg/s1/s2_deg` | 本帧查询 RobotSim、构建投影时实际使用的冻结机器人状态，位置 mm、角度 deg |

`camera_timestamp_ms` 来自 CameraFrame 的本机采集/读取时间；`robot_state_received_ms`
是 coordinator 收到该状态的本机时间。二者不等于硬件曝光时间或控制柜采样时间。
`processing_started/finished_ms` 是 Vision 工作线程的处理区间，`record_timestamp_ms`
是数值写入时间。全部使用 Unix 毫秒；未知时间和未产生的阶段数值留空，不以零伪装有效值。

### 资源、错误与生命周期

图像后台队列最多两对待写图，另有一对正在编码；队列满立即返回 `queue_full`，
不等待 PNG 编码。图像隐式共享跨线程传递，OpenCV 增强图在离开 Vision 前复制，
不会引用失效 Mat 缓冲区。磁盘可用空间低于 512 MiB 或剩余预算不足时记录
`budget_or_disk_limit` 并跳过整对图像。预算不包含数值表和小型 manifest。

写入错误、编码错误保存在 manifest，manifest 自身失败也会通过记录器错误与会话摘要暴露；
后台错误只首次向运行状态报告，数值记录继续尝试。图像落盘失败可能留下部分 PNG，
状态为 `write_failed`，不会删除或覆盖，不能把文件存在视为成对保存成功。

停止、重启和析构都会回收图像线程；会话结束排空有界队列后 join。每次新会话
使用独立目录、采样计数和预算，不沿用上一会话待写任务。图像编码不在 UI、Vision
或在线控制线程执行；数值 TSV 仍沿用现有在线 worker 的同步追加方式，其实时开销
及 Windows 文件系统上的表现需要现场验收。


默认图像预算为每会话 30 对、合计 32 MiB，便于将数值记录与少量图像一同提交到
GitHub。逐帧数值记录不受图像预算影响。多次会话仍会累积仓库体积；需要扩大采样
范围时可临时调整配置，完整图像也可单独打包分享。本机 `output/backups/` 和
`output/validation/` 已忽略，备份与验证程序不会作为新增文件自动混入提交。
