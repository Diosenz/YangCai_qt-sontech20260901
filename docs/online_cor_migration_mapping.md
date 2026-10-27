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

## 2026-10-27 实拍首捕与显示卡顿修复

论文跟踪链的初始相关响应与同帧模板自匹配不能证明首捕位置正确。主动轮廓及仿真
形状匹配只提供搜索提案，初始化前须由当前原图窄条纹及稳定转折确认；未确认时不
训练真实身份模板，也不给出可控制空间点。凹角提案可在纵向反射的原图中校核后映射
回原坐标，不能因为独立几何模块只支持向下凸角而永久拒绝正常凹角。原图确认位置
与主动轮廓提案在现有位置不确定度内一致时保留提案锚点，避免无谓改变跟踪初值。

该首捕身份校核不授予独立几何资格，亦不把完整双侧拟合作为后续每帧输出、更新或
控制的统一前提。既有逐曝光身份、响应、模板事务及空间投影规则继续生效；本次未
恢复虚实灰度相关 0.7 的仿真模板准入门槛，真实历史身份比较与仿真准入仍是不同边界。
反射校核只在有界扩大搜索周期启动时占用一次预算，后续曝光推进既有候选游标，
避免每帧重复校核占满两次预算、使搜索永久 pending 而吞掉恢复失败证据。空白图像
在搜索完成仍无支持且可靠轨迹耗尽后，继续通过原控制规则请求 HOLD，不新增帧数停机门槛。

三模板双特征训练保持原目标函数、迭代次数、残差检查及 SVD 备用。独立频率行在
OpenCV 内并行求解，任务独占输出行及求解缓冲；同一待提交曝光、同一 ROI 的 HOG/灰度
特征复用，ROI 校正、提交及重新初始化时维护缓存有效性，不复用其他图像的特征。

UI 主图继续显示检测结果所对应的同一曝光及其叠加；检测等待期间，右上角独立小窗
持续显示新相机帧，明确标注“无检测叠加”、预览帧号、主图检测帧号和检测曝光年龄。
相机刷新不覆盖主图，检测更新也同步小窗归属文字；返回普通相机显示时隐藏小窗。
这解除预览等待检测的问题，不将尚未计算的检测点画到新图，也不承诺现场显示帧率。

回归原图和来源哈希保存在 `tests/fixtures/online_tracking_20261027/`，包括真实在线
2070/7264 首捕及 3801—4171 稀疏曝光。部分投影上下文借自 3937 帧，只用于像素
回归，不验证毫米/TCP 精度。另以实拍 1712 图纵向翻转覆盖相反折点方向，不生成条纹。

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

- `OnlineExceptionHandling_beta()` 的活动配准语义由
  `COnlineCorrectionExceptionProcessor` 承接；2026-10-18 按当前逐帧单点入口修正为
  拟合历史与未输出 pending 分离累计，保留完整窗口后 80%，而不是每帧替换历史。
- `OnlineCorrectionPlannedTrajectory` 增加 `AllTpMatched` 与
  `AllTp_segement_line_and_curve` 等价字段：
  `allMatchedPathPoints` 和 `pathSegments`；当前 `_W/_T` 坐标仍为变位机坐标、mm。
- 仿真输入在旧五个函数之后补充 `getScanLineChangePoint()`，对应旧
  `sim_scan_line_change_point`；旧五个函数名及前五步调用顺序保持不变。
- 相邻 `_T` 点在当前程序段内确定几何子段；未知 seam 使用完整当前段。已知边界
  两侧分别求解/原始释放，避免一侧条件不足时被另一侧成功结果吞掉。抽样统一每三点
  并补末点，抽样后至少五点，窗口与新批跨度至少 2 mm；段尾不足则原样释放。
  首几何段 `max_iter=60`，后续为 50，`min_delta=5e-4`。统一沿用单段
  `match_type==1` 的检测到模型求解、逆变换模型到检测语义。
- 输出只截本批精确投影端点之间的有序折线；连续模型相位只向前，不自动闭环，
  短批前向搜索受本批弧长约束。不再搬用会重放历史的闭环拼接、上次末点平移和
  二次旋转补丁。未启用旧 `keepRemoveFailPoint`，其零坐标删除标记会误删合法点。
  gap 仍只从当前 pending 映射到预测点。端点不相容时原样回退；相位大幅滑移但本批
  正向投影相容时保留原始点并重定位游标，避免一次拒绝后永久无法再次配准。
- 在线模型及跟随参考复用起点规划后的全局变换/首点平移；
  `config/workflows/online_correction.txt` 的 `enable_online_icp=true/false`
  在下次启动生效，false 原始点旁路，诊断写入既有运行记录。
- `CorrectTaughtPathO()` 已拆入 `COnlineCorrectionPathFollower` 的三段逻辑：
  目标点选择、`rp1/rp2/rp3/rp4` 纠偏向量计算、控制方法输出。
- `ControlMethod` 映射为：
  - `0 / LegacyPidThreshold`：旧活动 PID，`kp=3, ki=0.2, kd=7`，十点历史保存方向分量；
  - `1 / LegacyFuzzyAdaptivePid`：保留旧模糊 PID 分支入口、输入归一化和参数应用位置；当前未恢复旧 `fisMatrixFile.txt` 运行时依赖，默认 FIS 增量为 0；
  - `2 / LegacyReservedNoop`：旧空分支，仍发 P，但三轴纠偏保持 0。
- 输出限幅保持旧语义：上一输出非零且跳变超过 `60 um` 时沿用上次值，
  或绝对值超过 `80 um` 时沿用上次值；gap 调速保持 1/2 mm 归一化、
  1000 档量化和 `cur_interpolation==2` 时 `speedMode=1`。


## 2026-10-25 实测优先证据策略正式接入

本节更新正式在线行为，优先于本文此前的 legacy 映射描述。用户已明确授权修改
现有 KCF 并接入此前离线验证过的策略。未改动 oldproject、RobotSim 或通信协议。

- `CVisionTrackingWorkerAdapter::processDetType2` 在在线纠偏显式启用
  `LegacyOnlineKcfOptions::enableEvidencePolicy`。实际运行路径是原主程序的
  coordinator → Vision worker → detector → `COnlineSeamEvidence`，无需启动 Python。
- `COnlineSeamEvidence` 与回放 `DetectorReplay` 共用生产 C++。先检验同目标原图
  几何、双侧母材及连续新曝光，再训练/重建 KCF 模板。首次建立和缺测后恢复默认
  各需两份新曝光；重复/乱序曝光不能刷新锚点、确认计数或模板。
- 身份走廊及母材阈值来自既有回放候选，尚未标定现场公差。走廊外候选不自动替换
  现有目标；显式重新配置跟踪会清空旧身份。不能以高相关分数证明焊缝准确。
- 仿真查询失败时，在线链保留同会话冻结原图和 RobotState，以无效的同帧参考 DTO
  继续图像检测，绝不借用上一帧仿真。缺失 RobotState、投影/标定无效仍拒绝空间输出。
  离线扫描原有的强同步门禁未改变。新增 `ImageOnlyReady` 明确关联参考缺失时的
  UI pending → consumed 门禁；迟到响应仍由 requestId/session/frame 隔离。
- 续接同时要求显式时间和沿工件焊缝的弧长预算，累计绝对进度差，反向也消耗行程。
  参考版本切换必须提供旧版本在当前曝光的投影桥接，不能把模型绝对偏移当作运动。
  估计、单侧约束和恢复过渡都不更新可靠锚点或预算。
- **当前生产 transport 没有逐曝光的工件沿缝弧长及旧版本同曝光桥接。** 两个续接
  预算默认均为零，正式程序因此仅释放确认的原图实测；不能把 TCP 净位移替代弧长，
  也不能宣称预焊遮挡已完成连续生产续接。回放可显式提供这些输入检验续接规则。
- 像素可用、空间有效、独立校验、控制资格分别表达。母材估计/外观续跟/纯预测/
  恢复过渡只可显示像素，不投影为业务空间样本，也不送给独立 ICP 或 Control。
- 正式 `COnlineCorrectionWorkerAdapter` 启用 `preserveMeasuredReference`。
  轨迹 ICP 可保留原有拟合/缓存，但实际释放点逐点等于同批原始实测点；未采用模型
  轨迹以 `model_proposal_not_released` 单独归档。初始定位和独立表面 ICP 不受影响。
- 图像实测为绿色；估计及控制参考线与激光平面的显示交点为黄色。控制参考交点
  仍是旧控制预测的显示，不能当成新检测点；本次没有重新设计失测停机/降速策略。
- `vision_diagnostics.tsv` schema 升为 8，追加证据资格、锚帧、确认次数、母材秩、
  身份分数和预算诊断；已有列保留。ICP 点记录 schema 为 3，增加未释放模型候选种类。

### 2026-10-26 现场复测修正

- 图像身份残差走廊默认由 12 调为 24 px，适用于当前测试阶段，尚不是标定后的物理公差。
  仿真参考相对位移用于移动搜索中心；正常扫描总位移不再直接触发参考冲突。
  跨出身份模板覆盖范围的参考跳变只撤销参考续接，不能否决历史走廊里的原图身份。
- 连续新曝光确认只用于首次建锚和失测后的恢复。稳定跟踪已通过当前原图几何、
  投影及身份检验时直接更新锚点，不因绝对帧间位移重新变成首个确认曝光。
- 测试阶段原图拟合最小支持率由 0.75 调为 0.55，适应部分反光/缺列；双侧至少
  25 列支撑、1.8 px 残差、多窗口几何一致性和逐曝光身份确认仍保留。该参数
  是测试默认值，不能视为经过物理精度标定的生产公差。
- 正式在线仍限制每帧最多两次几何拟合。扩大搜索优先检查当前搜索提案附近角点；
  稳定候选经过连续新曝光复验后交给证据层，不再等待整幅图全部提案耗尽。
  已发现的同帧竞争分支、重复曝光、身份不符和投影失败仍不能进入控制。
- UI 缺测时不再把 TCP 附近短直线与激光平面的无限延长交点画成 KCF 点，也不写入
  KCF 坐标栏。控制参考仍在独立业务目标坐标区显示其来源与序号；本次不改变
  控制层原有缺测预测、通信协议或机器人命令策略。

以上覆盖前节关于黄色“控制参考交点”的旧显示说明。首丢诊断窗口及 schema 9
保留用于本轮现场验收，采集限制见 `online_correction_state_machine.md`。

验证入口：生产策略使用 `replay.py manifest.jsonl --production --output <新目录>`；
`--evidence` 保留为此前 Python 策略对照，不能将它的结果代替生产 C++ 验收。
Windows 在原 Qt Debug 配置中生成主程序 `i2r_rlc`，并运行
`i2r_rlc_online_kcf_evidence_worker_tests`、`i2r_rlc_online_correction_tests` 及
在线/离线 frame sync 测试；Mac 辅助结果不能替代 MSVC 和必要运行验证。

### 2026-10-26 预焊遮挡期间的连续 KCF 跟随

根据本轮明确的业务要求，正式在线阶段建锚后的 KCF 必须逐曝光输出，供 UI 和 TCP
跟随共同使用。本节覆盖此前“估计只显示像素、全部禁止进入 Control”的正式策略说明。

- 反向初扫仍需独立图像几何确认；进入 `onlineControlWindowActive` 后，Vision 显式启用
  `enableContinuousOutput`。没有可靠初始锚、重复曝光或无效投影不能凭空建立控制点。
- 连续业务输出与可靠图像身份锚分别保存。反光/预焊造成原图精定位退化时，保留已确认
  接缝位置，以同版本相邻曝光的参考相对运动续接；参考缺失、突跳或无桥接的版本切换
  保持现有连续位置，不采用模型绝对坐标，也不使用控制参考线与激光平面的无限延长交点。
- 当前图像具有可靠外观支持时允许小幅修正。续接及恢复默认每曝光最多移动 8 px；恢复
  实测超过此步长时显示/释放平滑过渡点，原始几何候选仍单独保留。该步长是测试参数，
  不代表机器人空间误差界限，长时间遮挡下的物理准确性仍需现场验证。
- 正式 Control 显式启用 `allowEstimatedReference`，连续 KCF 带 `estimated=true` 进入
  实际跟随队列，旁路 ICP；初扫仍拒绝估计。估计不能训练 KCF 身份模板、更新独立实测
  预测锚或取得独立表面/焊点验收资格。`preserveMeasuredReference` 仍保留原检测位置，
  不以 ICP 模型候选覆盖它们。
- UI 使用同帧 KCF 输出画点和坐标，实测绿色、续接/过渡黄色，并标注连续跟随来源。
  诊断 JSON 的 evidence_options 记录连续模式及其步长，旧归档缺省仍保持原策略。
- 外观身份匹配点到几何候选的局部容差由 4 px 调为 6 px；7424 帧记录显示 4.50 px
  残差导致拒绝，但该帧原始曝光未保存，不能把日志分析当成该帧原图回放验收。

验证覆盖最新保存的 15 张真实首丢曝光、合成预焊凸起、连续遮挡、参考突跳与版本切换，
并经过实际 Vision worker、Control 跟随队列和 UI 显示测试。截图 7143/7424 的原始曝光
没有在本次采样中落盘，现场预焊期间的位置误差仍须复测。

### 2026-10-26 反光后的实测身份恢复与测试门限

后续复测 `20261026_143801_317` 显示，反扫 251 帧中 250 帧原图几何通过，但只有
173 帧取得资格；在线 685 帧具有有效原图几何，只有 75 帧最终输出实测。原图拟合
失败和外观身份拒绝应分别诊断，不能以连续估计输出率代替实测恢复率。

- 独立原图身份检验始终使用最后实测锚及曝光参考的关联提案，续接输出不再替换其
  搜索中心。参考无效时仍保留历史实测走廊的检验机会，避免估计漂移后长期锁住原图。
- 外观退化但当前完整双支几何仍通过、与可靠锚的左右支方向一致且位于关联走廊时，
  允许两份新曝光重新确认身份。最低外观支持仍保留；只用当前原图几何累计确认，
  不能用估计位置自证。恢复确认期间继续提供带来源的 KCF 跟随点。
- 测试阶段默认值调整如下；归档中的显式旧配置继续按旧值回放，不被静默覆盖：

| 门限 | 调整前 | 当前测试默认 |
| --- | --- | --- |
| 整块模板相关分数下限 | 0.70 | 0.55 |
| 左右局部相关分数下限 | 0.55 | 0.40 |
| 最佳匹配唯一性差值下限 | 0.04 | 0.02 |
| 候选到匹配点容差 | 6 px | 10 px |
| 预算续接模式的恢复步长 | 3 px | 6 px |
| 每侧支撑列数下限 | 25 | 20 |
| 几何支撑比例下限 | 0.55 | 0.50 |
| 位置不确定度上限 | 6 px | 8 px |
| 歧义列比例上限 | 0.40 | 0.50 |

当前诊断中拟合残差通常已经低于 1.8 px，因此该门限不变；24 px 身份走廊和连续
跟随每曝光 8 px 步长仍保留。母材门禁默认关闭，不重新启用旧宽窗口母材拟合器。
重复曝光、错误投影、无图、远处无关目标和完全竞争条纹仍不能取得实测资格。

所有 KCF 标记统一绿色，包括续接/恢复点；文字与日志保留实测、估计、过渡来源，
颜色不授予独立验收资格。会话摘要同步使用连续 KCF 策略名称，移除过期 measured-only 描述。

回归增加 15 张实际首丢曝光的反扫尾部恢复、9 张稀疏保存在线图像的身份恢复，以及
续接偏离后原图重新接管的闭环序列。在线稀疏图像缺少完整曝光变换，回放明确使用
标注的静态投影夹具，只验证像素身份，不声称复现完整在线闭环或物理控制精度；
首三张图完整搜索用于建锚，之后使用正式的每帧最多两次拟合预算。


### 2026-10-26 在线局部定位连续拒绝后的受限恢复

现场位姿停止刷新来自局部候选持续被拒绝。会话中连续至少两次表面候选拒绝后，
允许下一批全新曝光进入受限恢复：仅训练双向覆盖阈值最多降低五个百分点
（默认 70% 到 65%），训练残差至少改善 15%，且不低于已有配置的改善要求。
完整帧留出覆盖仍为原阈值，写后新曝光验证保持原规则。

恢复候选的单步平移限制为原预算与 10 mm 的较小值，旋转限制为原预算与 3°
的较小值；完整残差、空间跨度、真实焊点反证、写前配对、归档与累计位移预算
继续生效。无共同可观测修正方向的窗口仍拒绝，下一批新曝光窗口依次增加四帧，
直到配置上限（默认 8 → 12 → 16）。每次尝试后原窗口均移出，不重复重算。
接受候选、更换会话、模型代次或工件位姿版本后清除拒绝计数和扩窗状态。

归档新增 `preceding_surface_rejections`、`surface_recovery_enabled`、
`training_coverage_recovery_used` 和 `rejection_reasons`，多个拒绝原因保留，
真实焊点否决不会覆盖此前的表面拒绝原因。

真实归档 10535、10744、10950、11164 构成回归：前三批仍拒绝；11164 的训练虚拟
覆盖为 0.695122、训练残差改善约 34%，留出两向覆盖通过，连续拒绝后允许作为
待写后验证候选。离线回放不代表后续现场窗口均能写回，仍需现场验证。
