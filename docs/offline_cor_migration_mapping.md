# OfflineCor 有匹配主链路迁移映射

## 迁移范围

本轮只迁移 `SWv1/OffLineCor.cpp/.h` 中当前使用的有匹配离线纠偏主链路，并已补齐
UI / Vision / RobotComm runtime 闭环。核心算法仍位于 `Control`，运行时编排位于
`UI::CRuntimeCoordinator` 和 worker adapter，不包含 MFC Timer、HALCON 或旧调试文件写入。

离线视觉输入限定为旧 `OpencvThread` 的扫描检测路径 `det_type==2`，与在线纠偏共用
`CVisionTrackingWorkerAdapter::processDetType2()` / `CLegacyOnlineKcfDetector`，不恢复
端点检测 `det_type=1/3/4/5`。

## 函数映射

| 旧函数/状态 | 新工程承接 | 说明 |
| --- | --- | --- |
| `OnOffradio4()` | `Ui::MainWidget::offlineCorrectionArchiveSelected` + `COfflineCorrectionController::selectArchiveFile()` | UI 只选择 `_W.txt` 或 `_T.txt`，Control 自动查找伴随文件。 |
| `OnOffcorstart()` 开始扫描 | `COfflineCorrectionController::startDetection()` | 不直接启动相机或 Vision；等待外部检测事件输入。 |
| `OnOffcorstart()` 扫描完成后计算 | `finishDetection()` + `computeCorrection()` | 显式状态转换替代旧按钮二态和 timer。 |
| `OfflineRun` / `OfflineStartCount` | `COfflineCorrectionWorkerAdapter::updateRobotState()` + `CRobotCommWorkerAdapter::setOfflineStartCount()` | 保留旧控制柜握手；必须确认本轮 `sStart==1` 且扫描停止后才结束检测，`detflag` 不作为离线总流程完成条件。 |
| `OfflineTraMatching()` 有匹配分支 | `COfflineCorrectionTrajectoryMatcher::match()` | 只保留有匹配模式。 |
| `ICPmatch()` 当前实际路径 | `COfflineCorrectionTrajectoryMatcher::match()` + `CIcpRegistrationMatcher::alignPointToPoint()` | 读取模型/示教点，丢弃检测点前 5%，平滑，整体 point-to-point ICP，变换示教点。 |
| `GenerateSeamTraj()` | `COfflineCorrectionTrajectoryMatcher::loadArchive()` | 新工程直接读取存档 `_W/_T`，不再从 `AllPts/RobotPose_new` 生成中间文件。 |
| `SendNewTeachPts()` | `RobotCorrectionCommand` + `CRobotCommWorkerAdapter::sendCorrectionCommand()` | 输出 `offlineSignal=4`、`pointCount`、示教点和变位机点，逐条发送旧 `O` 请求并等待 `D.offline==5`。 |
| 旧 `OpencvThread` 顶层触发条件 | `CTrackingTriggerPolicy` + `CVisionTrackingWorkerAdapter` | 保留 `traflag==1`、`sMode==2`、`sHold==0`、`sStart==1` 和虚拟记录状态判断；离线模式不设置 `s_online`。 |
| 旧按钮二态 | `UiController::onStartOfflineCorrectionRequested()` | 第一次点击启动检测；检测中第二次点击只登记结束请求，扫描 JBI 停止后才执行 ICP 并发送 `O`。 |

## Runtime 接口映射

| 新接口 | 作用 |
| --- | --- |
| `CRuntimeCoordinator::configureOfflineCorrectionArchive(path, options)` | UI 选择 `_W/_T` 后配置离线纠偏存档。 |
| `CRuntimeCoordinator::startOfflineCorrectionDetection()` | 进入离线检测目的，启动 Vision tracking。仅支持 `det_type==2`。 |
| `CRuntimeCoordinator::finishOfflineCorrectionDetectionAndCompute()` | 停止 Vision tracking，执行 ICP，生成并发送 `O`。 |
| `CRuntimeCoordinator::resetOfflineCorrection()` | 重置离线状态并停止 Vision tracking。 |
| `RuntimeTrackingPurpose::OfflineCorrection` | 将 KCF 输出只路由给 Offline worker，避免误喂 OnlineCor。 |

## 不迁入内容

- `DirectMatch()`、`GenerateNomatchTraj()`、`OnOffradio2()` 和 `Matchoff` 子对话框。
- `ADDOFFSET`、补偿点配置、自动焊接 job 启动。
- MFC `MessageBox`、`Timer`、对话框状态、旧临时调试文件。
- HALCON 显示对象或任何 HALCON 标定/图像接口。
- 无匹配离线纠偏和旧端点检测 `det_type=1/3/4/5`。

## 数据与单位

- 存档模型点 `_W.txt` 和示教点 `_T.txt` 支持 `mm/deg` 或旧协议整数单位；当首个坐标绝对值大于 `10000` 时按 `um` 或 `deg*10000` 归一化。
- 检测点事件输入为 `WeldSpacePoint`，单位 mm。
- 输出机器人命令中位置为 `um`，姿态与变位机轴角为 `deg*10000`。
