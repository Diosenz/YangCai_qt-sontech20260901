# 在线纠偏一键全自动两阶段启动说明

## Legacy 依据

旧工程把工件 JBI 分成两组：

- `WPOfflineJBIList`：全局定位扫描程序。
- `WPOnlineJBIList`：在线纠偏程序。

`CRobotleadingDlg::OnChangeVirtualStatus()` 先把 `WPOfflineJBIList[wpindex]`
写入 `jobname_dt` 并置 `jobstart=1`；`RobotCtrl.cpp` 在
`sStart == 0 && jobstart == 1` 时发送旧 `J` 命令。第一段程序运行后，
旧虚拟场景循环通过 `job_done == 1 && sStart == 0` 判断作业结束，执行全局定位，
再切到在线识别状态并发送第二个 JBI。

## Qt 当前实现

Qt 工程现在在 `UiController` 中显式表达该链路：

1. 点击在线纠偏后，先使用当前工件的 `offlineLocateJbi` 进入全局定位启动门禁。
2. 门禁通过后发送第一次 `J` 命令，并启动 `GlobalLocalization` 视觉跟踪目的。
3. 作业完成判定使用 `CCorrectionRobotJobMonitor`：必须先看到 `sStart==1`，
   再看到 `sStart==0`，才认为全局定位 JBI 完成。
4. 第一段运行期间收集 `RuntimeTrackingResult` 中的三维点作为真实扫描点。
5. 第一段结束后调用 `LocalizationApplicationService::runGlobalIcpRuntime()`；
   该入口只接受真实扫描点和 RobotSim/DigitalTwin 虚拟参考，不使用 dry-run 点云。
6. 全局定位成功后，把修正矩阵写入在线纠偏配置，再进入第二次启动门禁。
7. 第二次门禁通过后发送 `onlineJbiForStartMode()` 选出的在线 JBI，并启动现有
   在线纠偏初始检测、示教点 O 命令和实时 P 命令流程。

## 失败处理

以下情况不会启动第二段在线 JBI：

- 当前工件缺少全局定位 JBI 或在线 JBI。
- 第一段全局定位 JBI 未采集到有效三维点。
- 未获得可用 RobotSim/DigitalTwin 虚拟参考。
- 全局 ICP 失败。
- 面板关闭、停止或 shutdown 时 workflow 被取消。

当前实现不恢复旧工程 `VirtualWorkpiece` 的写回对象；全局定位结果通过
`OnlineCorrectionOptions::globalCorrectionRowMajor` 注入在线纠偏配置，供
`GlobalCorrection` 起始方式使用。
