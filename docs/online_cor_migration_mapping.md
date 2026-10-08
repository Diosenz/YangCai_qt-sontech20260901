# OnlineCor 在线纠偏迁移映射

## Legacy 阅读结论

本轮以 `SWv1/OnlineCor.cpp/.h`、`SWv1/robotleadingDlg.cpp/.h` 和
`SWv1/RobotCtrl.cpp` 为只读依据。确认的活动链路包括：

- `OnStartinitdet_change()`、`OnlineReverseDetectMatch()`；
- `OnlineTeachTrajectoryInitial()`、`correctTeachTraj()`、`correctTeachPts()`；
- `OnlineExceptionHandling_beta()`；
- `CorrectTaughtPathO()` 的 `ControlMethod==0`；
- `RobotCtrl.cpp` 中 N/O/P 请求与完整 D 状态回包。

`ReadStationPosition()` 从 `RBCALIB.DAT` 的 `////SRANG` 读取六维标定；
`RobotPos2StationPos()` 和 `StationPos2RobotPos()` 使用变位机两轴角动态转换坐标系。

旧 `OnlineReverseDetectMatch()` 将 `initPtMatchType` 硬编码为 0，活动路径只返回
`D_initdata` 最后一点；随后 `correctTeachTraj()` 和 `correctTeachPts()` 用该点减去
当前示教分段首点，并把差值平移到整段轨迹与全部示教点。旧文件中虽保留
`initPtMatchType==1` 的 ICP 试验分支，但该分支未启用，且注释已经指出扫描到其他
焊缝点时可能错误匹配。本次不恢复该试验分支，而是在新规划器内采用当前 `_T`
分段约束、折线连续投影和反向方向校验。

## 新工程映射

| 旧职责 | 新工程承接 | 分类 |
| --- | --- | --- |
| OnlineCor 隐式全局状态 | `COnlineCorrectionController` + `OnlineCorrectionPhase` | 必须重写 |
| `_W/_T` 读取、初始轨迹修正 | `COnlineCorrectionTrajectoryPlanner` | 适配后复用 |
| `OnlineExceptionHandling_beta()` 活动 ICP 分支 | `COnlineCorrectionExceptionProcessor` | 适配后复用 |
| `CorrectTaughtPathO()` 控制方法 0 | `COnlineCorrectionPathFollower` | 适配后复用 |
| `ReadStationPosition()` | `CPositionerCalibrationParser` | 适配后复用 |
| 变位机/机器人双向位姿变换 | `CPositionerKinematics` | 适配后复用 |
| N/O/P 编码、D 解码 | `CRobotProtocolCodec` | 直接复刻协议 |
| socket/ZeroMQ 请求响应 | `IRobotCommandTransport`、`CClientRobotCommandTransport` | 重写边界 |
| MFC Timer 和按钮回调 | Qt worker 事件与 signal/slot | 过时，重写 |
| HALCON 显示、调试文件 | 不迁移 | 过时 |
| `onlineFlipFlag`、`s_TraOrCor==0` | 不迁移 | 明确排除 |

## 2026-10-08 坐标与会话标定边界

- Vision 的在线检测点以 `WeldProjectionTargetFrame::PositionerTcp` 输出，含义是
  变位机 TCP 坐标、单位 mm。`LegacyOnlineKcfDetector` 的空间投影不直接承担
  在线纠偏内部坐标转换。
- 在线入口 `COnlineCorrectionController::appendDetectionEvent()` 在进入异常处理器
  前，将每个有效样本沿 `toolTcpInLastLinkMm` 转换到末节坐标；在线缓存保存末节
  坐标。初始检测缓存仍保存 TCP 坐标，由
  `COnlineCorrectionTrajectoryPlanner` 在起点规划时完成一次同样的转换，避免
  P001/初始轨迹重复补偿。
- 本次在线配置解析出的 `PositionerCalibration` 是会话 Source of Truth。
  `CRuntimeCoordinator` 在显式配置边界读取或接收一次快照，并将同一快照交给
  Vision 与 Control；分段切换继续使用该快照，只有下一次显式配置才重新读取
  `RBCALIB.DAT`。配置快照存在但矩阵、偏移或输入点含非有限值时拒绝该数据或会话。
- 因此在线闭环的 Observable Behavior 是：同一会话中 Vision 产生的 TCP 点与
  Control 消费的末节点之间只有入口处一次、方向明确的 TCP→末节转换；零物理偏差
  不产生纠偏输出，标定文件在会话中途变化不改变当前会话结果。

## 协议边界

`CRobotProtocolCodec` 逐字符保持旧协议格式：

```text
N e_robot,s_tracker,s_laser,s_filter,s_offline,OfflineStartCount,setHold,FirstDetSuccess,Online_endflag
O e_robot,s_tracker,s_laser,s_filter,s_offline,OfflineStartCount,setHold,FirstDetSuccess,PvarNum,x,y,z,rx,ry,rz,s1,s2
P e_robot,s_tracker,s_laser,s_filter,s_offline,OfflineStartCount,setHold,FirstDetSuccess,Online_endflag,d_x,d_y,d_z,d_rx,d_ry,d_rz,g_spd_override_cmd,g_speed_mode
```

位置使用 `um`，姿态和变位机角使用 `deg*10000`。每条请求都读取完整 D 回包；
D 必须包含 32 个字段。O 按原索引逐点发送，只有最终点发送完成且
`D.offline==5` 才确认整批更新完成。

## 仿真边界

保留以下旧函数名，不在 Control 中重命名或调用：

1. `toNextWeldSeam()`
2. `generate_simu_laser1()`
3. `getSeamPoint2d()`
4. `getCurrentVirtualLine()`
5. `getSeamIndex()`

默认回调为空操作、`false`、无效点、空点列和 `-1`。`getSeamIndex()` 写入检测样本，
用于异常处理器识别仿真扫描面或焊缝段变化。

## 当前限制

- UI 已提供“在线纠偏配置”入口，依次选择 `*_W.txt`、`*_T.txt` 和
  `RBCALIB.DAT`；提交前会同步校验文件名、轨迹解析结果和 `////SRANG`。
- UI 未完成有效配置时，“启动焊缝跟踪”会返回明确错误，不会启动 Vision
  tracking，也不会发送 O/P。
- 未连接真实控制柜时不会发送 O/P。
- 异常处理器只承接已确认的活动 ICP 边界，不迁移旧调试文件和未启用分支。
- 自动测试使用 fake transport 覆盖 N/O/P、完整 D 状态语义、`offline==5`、
  P 最新值覆盖和分段结束闭环；真实控制柜联调仍需在设备环境执行。
- 初始反扫匹配仅解决 `_W` 进度与局部位置残差。JBI 中从检测结束位姿到首个动态
  P 点的安全接近路径和控制柜碰撞灵敏度仍需独立验证。
- 2026-09-14 实机记录确认：`offline==5` 时机器人仍在前往 P001；P001 后的首次
  `ENDDET` 位于 `STARTCOR` 之前，不能解释为在线段最终结束。当前状态机增加
  `WaitingForInitialApproach` 和 `WaitingForOnlineStart`，只有观察到
  `corflag/detflag==1/1` 才启动在线 KCF；STARTCOR 后的 ENDDET 才发送最终 P。
- 最终 P 和 detflag 复位只关闭 KCF，不再立即结束会话。实际路径继续记录到
  `sStart==0`，从而保留 JBI 退出段或报警/保持状态。当前 JBI 后续固定 C 点与
  动态 P001 的几何连续性仍是实机验证边界，本次不修改机器人程序。
- 2026-09-14 的 JBI 与运行记录进一步确认：首次 `ENDDETPRG`
  在 `STARTCOR` 之前把 `traflag` 复位为 0，且 SMOVC 在线段不再重新置 1。
  Runtime 现在只在初始反扫要求 `traflag==1`；观察到 STARTCOR 后，
  用 `OnlineRunning` 锁存授权后续在线帧。暂停检测时恢复实时相机画面，
  不再把结束阶段误显示为“等待首个完整检测帧”。
# 2026-06-23 OnlineCor / OpenCV 闭环补全

- `_W/_T` 统一按当前变位机坐标系、mm 保存；生成 O 前通过
  `CPositionerKinematics::stationPoseToRobotPose()` 转为机器人基坐标，
  再编码为 um 与 deg*10000。任一点缺少有效 `s1/s2` 时整批拒绝。
- `RBCALIB.DAT` 固定由
  `config/calibration/RBCALIB.DAT` 自动加载，UI 仅选择 `_W.txt/_T.txt`。
- `CWeldProjectionContextBuilder` 使用 SENSORPARA、逐帧 RobotState 和
  RBCALIB 构造 Camera→Tool→RobotBase→当前变位机坐标变换；按 2026-10-08
  用户指定，相机外参改为 `Rz*Ry*Rx`，机器人工具姿态同样采用 `Rz*Ry*Rx`。
  SENSORPARA 的角度数值与平移不变；该约定不再沿用旧 HALCON `gba`。
  变位机安装标定和外轴变换保持原有顺序。相机外参解释的变化会改变视觉
  三维点与后续 P001 XYZ，需以对应 TCP 的现场位置验证物理正确性。
- 生产 runtime 使用纯 Vision `CLegacyOnlineKcfDetector`：250x200 ROI、
  仿真模板、Gabor+Otsu、Y=-5 pixel 写回、超过 50 帧重建，只启用标准 KCF。
- gap 链使用 400x400 局部中心线和
  `CPointCloudGapWidthDetector`；失败降级为 `0.0 mm` 并保留样本。
- `corflag/detflag` 已翻译为锁存状态机：
  `SegmentFinishing -> WaitingForDetectionReset`；中间段自动加载下一段，
  最终段进入 `Completed`。
- RobotComm 回执保留原始 N/O/P 请求、D 回包和错误分类；状态面板显示机器人
  标志、位姿、变位机轴角、插补模式和事务计数，N/D 日志按秒摘要。
- `penzui` 702 组直投验收报告输出到
  `output/reports/penzui_station_trajectory.csv`。

# 2026-06-24 OnlineExceptionHandling_beta / CorrectTaughtPathO 补齐

- `OnlineExceptionHandling_beta()` 已扩展为带旧状态的
  `COnlineCorrectionExceptionProcessor`：保留正常检测段缓存、`curveFlag`、
  `matchTime`、上一匹配末点、检测段到全局示教轨迹索引等状态。
- `OnlineCorrectionPlannedTrajectory` 增加 `AllTpMatched` 与
  `AllTp_segement_line_and_curve` 等价字段：
  `allMatchedPathPoints` 和 `pathSegments`；当前 `_W/_T` 坐标仍为变位机坐标、mm。
- 仿真输入在旧五个函数之后补充 `getScanLineChangePoint()`，对应旧
  `sim_scan_line_change_point`；旧五个函数名及前五步调用顺序保持不变。
- 异常处理器按旧活动分支区分单段 ICP 与两段 ICP：
  两段使用 `max_iter=50`、检测抽样 2/3，单段使用 `max_iter=60`、
  检测抽样 3，`min_delta=5e-4`；单段保持旧 `match_type==1`
  的逆变换语义，两段保持正向模型到检测段语义。
- 匹配输出保留闭环拼接、上次末点平移连续和二次匹配旋转连续修正；
  gap 按旧 `MapGapsToProcessedPts` 语义只从当前检测段映射到预测点。
- `CorrectTaughtPathO()` 已拆入 `COnlineCorrectionPathFollower` 的三段逻辑：
  目标点选择、`rp1/rp2/rp3/rp4` 纠偏向量计算、控制方法输出。
- `ControlMethod` 映射为：
  - `0 / LegacyPidThreshold`：旧活动 PID，`kp=3, ki=0.2, kd=7`，十点历史保存方向分量；
  - `1 / LegacyFuzzyAdaptivePid`：保留旧模糊 PID 分支入口、输入归一化和参数应用位置；当前未恢复旧 `fisMatrixFile.txt` 运行时依赖，默认 FIS 增量为 0；
  - `2 / LegacyReservedNoop`：旧空分支，仍发 P，但三轴纠偏保持 0。
- 输出限幅保持旧语义：上一输出非零且跳变超过 `60 um` 时沿用上次值，
  或绝对值超过 `80 um` 时沿用上次值；gap 调速保持 1/2 mm 归一化、
  1000 档量化和 `cur_interpolation==2` 时 `speedMode=1`。
