# 独立“全局定位”按钮集成说明（weldment 虚拟点云 + 受限 ICP）

## 目标

在“定位”面板中点击“全局定位”按钮后，UI 一键完成：

1. 校验当前工件及其“全局定位 JBI”；
2. 通过 RobotComm 旧 J 协议启动示教器中的扫描程序；
3. 扫描期间通过全局定位视觉跟踪（`RuntimeTrackingPurpose::GlobalLocalization`）
   逐帧采集真实激光点，投影到变位机 TCP 坐标系；
4. JBI 结束（`sStart` 1->0 边沿）后，用 `WeldmentVirtualPointCloudGenerator`
   从 `.weldment` 模型生成虚拟参考点云（对应旧 SWv1 `stepICPforOffline()` 与
   `ComTest/virPoints.cpp` 的采样逻辑）；
5. 调用 `LocalizationApplicationService::runGlobalIcpRuntime()`，由
   `LocalizationGlobalIcpPipeline` + `LocalizationService` 执行受限自由度 ICP
   （对应旧 SWv1 方法 B `updateModelOfflineICP()`），并输出真实/虚拟/修正后
   点云与 4x4 修正矩阵到 `output/point_clouds/`。

## 新增/修改文件

- `DigitalTwin/include/WeldmentVirtualPointCloudGenerator.h`
- `DigitalTwin/src/WeldmentVirtualPointCloudGenerator.cpp`
  - OpenCASCADE 解析 `.weldment`（STEP 几何 + 尾部 `[weldment_parameters]` 配置）
  - 按定位段/定位基准面 UV 随机采样，采样率 0.03 点/mm²（最少 10 点/面）
  - 输出变位机 TCP 坐标系下的 mm 点云（`fix_pose` 已应用）
- `DigitalTwin/include/WeldmentLocalizationVirtualReferenceProvider.h`
- `DigitalTwin/src/WeldmentLocalizationVirtualReferenceProvider.cpp`
  - 实现 `localization::ILocalizationVirtualReferenceProvider`
  - 把生成的点云填充到 `LocalizationVirtualReferenceSnapshot.workpiecePointCloud_mm`
- `DigitalTwin/CMakeLists.txt`：`find_package(OpenCASCADE REQUIRED)`，新增源文件
- `vcpkg.json`：新增 `opencascade` 依赖（`platform: windows`）
- `UI/include/MainWidget.h` / `UI/src/MainWidget.cpp`：
  - “全局定位”按钮信号 `runGlobalIcpDryRunRequested` -> `globalLocateRequested`
- `UI/include/UiController.h` / `UI/src/UiController.cpp`：
  - 新增 `StandaloneGlobalLocateWorkflowState` 状态机
    （Idle -> WaitingRobotStartGate -> JobRunning -> Applying -> Failed）
  - 新增 `CorrectionRobotStartKind::GlobalLocateStandalone`
  - 新增入口/状态处理/采点/虚拟点云生成/ICP/失败处理函数
- `models/weldment/fengjikechukou20_zheng.weldment`、
  `models/weldment/fengjikechukou20_fan.weldment`：从旧 SWv1 复制，
  与 `config/workpieces/workpiece_selection.txt` 默认工件对应

## 坐标系约定

- 真实点：视觉跟踪 `LegacyOnlineKcfDetector` 使用
  `WeldProjectionTargetFrame::PositionerTcp` 输出（mm），与虚拟点同系。
- 虚拟点：`WeldmentVirtualPointCloudGenerator` 采样 CAD 定位面并应用
  `fix_pose`，坐标系为变位机 TCP（mm）。
- ICP 输入必须同系同单位；当前单段全局 ICP 使用默认自由度
  `x/y/z/rz`（`rx/ry` 锁定），与旧工程典型约束一致。

## 运行前置条件

- 已选择工件（`config/workpieces/workpiece_selection.txt`），
  且 `offlineLocateJbi`、`weldmentModel` 非空；
- `models/weldment/<weldmentModel>` 存在；
- 机器人已连接（收到有效 D 回包）、PLAY 模式、伺服 ON、程序未运行；
- 相机与视觉跟踪链路可用（未采集到点时定位会显式失败）。

## 风险与后续

- 虚拟点云生成与 ICP 当前在 UI 线程同步执行，大模型可能造成界面短暂卡顿；
  后续可把“生成虚拟点云 + ICP”移到 QtConcurrent 后台线程。
- 当前按 weldment `position_segment_N` 执行分段受限 ICP；真实扫描段数量
  必须与虚拟定位段数量一致，否则流程显式失败并提示检查 JBI 段标志。
- `opencascade` 是新增的第三方依赖，构建前需 `vcpkg install`；
  旧 SWv1 的几何定位（`vloc::computeLocation`）不在本轮范围。

## 2026-08-08 补充：自产虚拟激光先验（不再依赖 RobotSim 才能采真实点）

### 背景
qt-sontech 的全局定位真实采点链路（Vision worker `processDetType2`）需要
“仿真激光线 2D 点 + 仿真焊缝点 2D 点”作为 KCF 跟踪先验（ROI + 模板）。
此前该先验只能来自 RobotSim 联调（`updateDigitalTwinVisionObservation`），
否则 `makeObservation` 失败、帧被丢弃，导致全局定位无法独立完整运行。

### 新增
- `DigitalTwin/include/WeldmentVirtualLaserPriorGenerator.h`
- `DigitalTwin/src/WeldmentVirtualLaserPriorGenerator.cpp`
  - 输入：weldment 路径、机器人 TCP 位姿、变位机两轴角、SENSORPARA 标定、
    RBCALIB（变位机基在机器人基下的位姿）、图像宽高；
  - 算法：
    1. 读取 STEP 模型并应用 fix_pose（变位机 TCP 系）；
    2. 构造与 `CWeldProjectionContextBuilder` 完全一致的
       相机→工具→机器人基→变位机TCP 变换链（正逆自洽）；
    3. 把相机系激光平面 (a,b,c) 变换到变位机 TCP 系，与定位面求交
       （`BRepAlgoAPI_Section`），采样交线点；
    4. 3D 交线点投影回图像（针孔 + 逆向 kappa），得到仿真激光线 2D 点；
    5. 焊缝点 = 激光平面与模型焊缝边（seam_list 边索引）最近点投影；
       seam 缺失时回退为激光线 2D 质心；
  - 输出 `DigitalTwinVisionLaserObservation`，经现有
    `CRuntimeCoordinator::publishDigitalTwinVisionObservation` 通道注入
    Vision worker。

### UI 接入（UiController）
- 全局定位 JobRunning 期间用 60ms QTimer 定期以最新机器人/变位机位姿
  生成并发布先验（`startGlobalLocatePriorTimer` / `publishGlobalLocateVirtualPrior`）；
- JBI 结束、失败、取消、停止时统一停止定时器（`stopGlobalLocatePriorTimer`）。

### 说明
- 本轮目标是让“全局定位”脱离 RobotSim 也能完整跑通；真实采点仍为 KCF
  每帧一个焊缝点。若要复刻 SWv1“仿真激光线引导 Gabor + Steger 提取整条
  真实激光线并逐点重建”，可作为下一步（Vision 已具备 Steger/Gabor/
  LaserStripeExtractor 组件，需要新增一条全局定位专用的线提取 pipeline）。

## 2026-08-08 补充（第二轮）：整条激光线重建 + 逐段 ICP

### 1. 真实点云获取改为 SWv1 式“整条激光线逐像素重建”
- 之前全局定位复用在线纠偏的 KCF 链路，每帧只输出一个焊缝点。现已新增全局定位专用路径：
  - `RuntimeTrackingOptions.globalLocateLineExtraction`（`startGlobalLocalizationTracking` 自动置 true）；
  - Vision worker 新增 `processGlobalLocateLineExtraction()`：
    1. 用自产仿真激光线（`WeldmentVirtualLaserPriorGenerator`）确定 ROI（包围盒外扩 80px）；
    2. `CImageDenoise::gaborDenoise(gray, simPoints, weldPoint)` 做仿真线引导的方向性 Gabor 滤波（对应 SWv1 `GaborFilter`）；
    3. `CStegerStripeDetector::detect(...)` 提取整条真实激光中心线（对应 SWv1 `getCurrentCenterLine`/Steger）；
    4. 逐像素 `projectCvPoint(..., PositionerTcp)` 重建整条线 3D 点，写入 `RuntimeTrackingResult.linePoints`；
  - `RuntimeTrackingResult` 新增 `linePoints`（整条线点）与 `posflag`（当前定位段标志）。

### 2. 真实点云按 posflag 切段
- `UiController::collectStandaloneGlobalLocateScanPoint` 改为：整条线点优先，按 `posflag` 边沿切段
  （0->1 开新段，1->0 结算当前段），累积到 `m_standaloneGlobalLocateSegments`；
  段外点（posflag==0）只计入统计，不参与定位（与 SWv1 一致）。

### 3. 逐段 ICP（对应 SWv1 updateModelOfflineICP 的 stepICPforOffline）
- `WeldmentVirtualPointCloudGenerator` 现在输出：
  - `segmentPoints_mm`：每个定位段独立的虚拟点云；
  - `segmentDofMasks`：每段自由度掩码（由段配置 x/y/z/rx/ry/rz 转换）；
  - `points_mm`：全部定位面合并点云（兼容单段场景）。
- `LocalizationApplicationService` 新增 `runGlobalIcpRuntimeSegmented()`：
  组装 `GlobalIcpPipelineRequest.segments`（每段 = 真实点 + 该段虚拟点 + 段级 DOF），
  由 `LocalizationGlobalIcpPipeline` 逐段执行受限 ICP 后组合修正矩阵。
- `UiController::applyStandaloneGlobalLocalization`：真实段 ↔ 虚拟段按段序一一对应。

## 2026-08-10 修正：严格对齐 SWv1 分段 ICP 与现场诊断

### SWv1 对照结论

- `stepICPforOffline(posSegCount, transM)` 的 source 是
  `aRealPointListSizeforICP[posSegCount]` 指定的本段真实点云；target 只从
  `position_face_dof_list[posSegCount].first` 指定的本段 CAD 定位面采样，
  不是用整件虚拟点云。
- `icpnl.align(*transformedPC, transM)` 把上一段完整的 real-to-virtual
  结果作为下一段初值；本段结束后 `transM = getFinalTransformation()`。
  因此最终修正是最后累计结果的逆，不是把各段独立逆矩阵相乘。
- 旧流程依赖 JBI 让 `traflag` 检测窗口与 `STARTPOSSEG/ENDPOSSEG` 对齐；
  分段边界只记录定位扫描区间，不存在“把整趟段外点统一并入第一段”的语义。

### 当前实现修正

- `LocalizationGlobalIcpPipeline` 在有分段参考时不再强制要求额外默认虚拟点云；
  每段以上一段完整 ICP 结果作为 `initialGuess`，最终输出最后累计结果的逆。
- `UiController` 同时填入完整 weldment 虚拟快照，作为全部点云残差和最终修正
  输出的统一参考；真实/虚拟段数不一致时显式失败，不再 `min` 截断或单段回退。
- `posflag==0` 的重建点只计入丢弃诊断，不参与任何定位段；非零 `posflag`
  按 SWv1 兼容规则统一视为 1。
- 每次运行创建独立的
  `output/point_clouds/global_locate_<timestamp>/`，保存全部/逐段真实点云、
  全部/逐段虚拟点云、最终修正点云、4x4 修正矩阵和完成/失败摘要。
- 控制台按启动门禁、JBI 启动、虚拟激光先验、真实点采集、JBI 结束、
  虚拟点云生成和逐段 ICP 输出阶段日志。0 点失败信息包含触发拒绝数、
  图像处理失败数、先验成功/失败数和最后失败原因。
- 全局定位线提取先走 Gabor + Steger；默认灰度阈值失败后在仿真 ROI 内以
  80 重试，Gabor 不可用或仍失败时再用原灰度图回退，并在日志中标明阶段。

## 2026-08-10 现场日志复盘与第二轮修正

现场一次完整 JBI 运行得到 `triggerRejected=1292`、`invalid=2306`、
`priorSuccess=0`。其中拒绝帧发生在 `traflag!=1` 的非检测窗口，本身不代表
图像算法失败；2306 个无效帧则暴露出虚拟激光先验被错误当成了真实采点的
硬前置条件：weldment 求交失败后，代码在进入 Steger 前直接返回
`simulated observation unavailable`。

本轮调整如下（后续 2026-08-10 的 SWv1 严格对齐修正已取代“无先验整图回退”策略）：

- 曾尝试在先验不可用时对整幅真实灰度图执行 Steger；现场进一步确认全局
  定位必须以 SWv1 的 OCCT 虚拟激光线为参考后，已取消该回退，详见下一节。
- STEP 读取与 OpenCASCADE 激光平面求交移入独立的
  `digital-twin-prior-worker` 线程；UI 定时器只允许一个在途请求，避免每
  60 ms 堆积耗时任务并阻塞相机画面。
- 全局定位期间不再逐帧向 UI 日志框写入“未触发/无效/有效”消息，改为原因
  变化时和固定采集间隔输出摘要；相机原始帧显示通道保持独立。
- 真实和虚拟点云输出统一改为 ASCII PLY，文件头明确坐标系为
  `positioner_tcp`、单位为 `millimeter`。
- JBI 结束后先生成、记录并保存 weldment 虚拟点云，再检查真实点云是否为
  空；因此即使真实采集失败，也能看到 weldment 解析结果和虚拟 PLY 文件。

## 2026-08-10 修正：恢复 SWv1 OCCT 虚拟线结构光与 JBI 双层状态语义

### 标定数据核对

- `config/calibration/RBCALIB.DAT` 与 SWv1 实际读取的
  `data/Robotleadingdata/StationPos/RBCALIB.DAT` 内容一致。
- 原 `config/calibration/SENSORPARA.txt` 是 2023-07-04 的旧参数，与 SWv1
  当前 `data/BASIC/SENSORPARA.txt`（2026-03-29）不一致。现已同步焦距、
  kappa、像元尺寸、主点、激光平面和相机在工具中的完整位姿参数。
- 启动及每次全局定位的控制台日志会打印 SENSORPARA、RBCALIB 的绝对路径；
  传感器日志同时打印焦距、主点和激光平面系数，便于确认运行电脑读取的文件。

### SWv1 虚拟激光算法对照结论

- 激光平面方程为 `a*x+b*y+c*z+1=0`（相机系、长度单位 m），并不穿过
  相机原点。旧 Qt 实现把平面原点错误设为相机原点，是无交点和投影越界的
  直接原因之一。
- `SwLSLSensor::generate_simu_laser1()` 不以“CAD 平面截完整定位面”的离散
  截线作为图像参考。它以相机原点在激光平面上的投影点为射线原点，在
  `[-15°, 15°)` 内按 `0.075°` 生成 400 条射线，每条射线在 0～400 mm
  范围内与完整 OCCT 工件求最近交点，最后用 SENSORPARA 的旧工程公式投影。
- 当前 `WeldmentVirtualLaserPriorGenerator` 已复现上述射线原点、方向正负、
  扇形角度、最近交点和投影取整语义；STEP/fix_pose 在专用线程中按 weldment
  路径缓存。全局定位门禁等待阶段即开始预热模型，避免首个检测窗口才读 STEP。
- 全局定位真实线提取重新要求有效的 OCCT 虚拟激光参考；单次新位姿求交失败
  时保留上一份成功参考，行为与 SWv1 传感器成员缓存一致，不再退化为无先验
  整图识别。

### JBI 开始、检测窗口、分段和结束

- 机器人 `D` 报文第 10 个状态字段是 `traflag`，由
  `STARTDETPRG/ENDDETPRG` 控制；第 31 个字段是 `posflag`，由
  `STARTPOSSEG/ENDPOSSEG` 控制。Qt 与 SWv1 使用相同的 32 字段顺序，并把
  任意非零 `posflag` 归一化为 1。
- `traflag==1` 只表示当前处在需要采图的检测窗口。一个定位段可以包含多个
  检测窗口；窗口之间的普通 MOVJ（例如点4、点6）期间 `traflag==0` 是正确
  状态，不能据此结束定位段。
- `posflag` 的 0→1 开始真实定位段，1→0 结束并保存该段。即使当前图像因
  `traflag==0` 被拒绝，Qt 仍传递并处理 `posflag` 边沿。因此用户给出的 JBI
  会形成两个 ICP 定位段：第一段合并点3、点5、点7三个检测窗口，第二段
  使用点10的一个检测窗口；JBI 的 `sStart` 从 1 回到 0 后才进入整体点云保存
  和分段 ICP。

## 2026-08-11 修正：相机帧绑定采集时状态与两定位面可靠分段

现场日志出现约 9.9 万真实点但始终只有“定位段1”。JBI 已确认包含两对
`STARTPOSSEG/ENDPOSSEG`。进一步检查线程链后确认根因不在 JBI：相机帧与
机器人状态分别排入 Vision worker，Steger 处理较慢时，旧实现会在“处理该帧”
时读取最新 `traflag/posflag`。因此较早拍摄的图像可能使用数百毫秒后的机器人
状态，产生以下错误：

- 非 `STARTDETPRG` 窗口拍摄的图像可能在处理时遇到 `traflag=1`，被错误采点；
- 两个定位面之间短暂的 `posflag=0` 可能被第二次 `posflag=1` 覆盖，导致第二面
  仍写入第一面；
- JBI 结束时 Vision 队列仍有旧图像，界面点数和机器人实际位置不同步。

当前修正：

- `RuntimeCoordinator` 在相机帧到达时按值冻结当时最近的完整 `RobotState`；
  Vision worker 只使用该快照判定 `traflag` 和进行位姿投影，不再读取处理时状态。
- Coordinator 直接监听 RobotComm 的 `posflag` 边沿，并为每次 0→1 的
  `STARTPOSSEG` 分配递增定位面序号。该序号与图像、机器人状态一起投递；即使
  为控制负载跳过了中间非检测帧，第二面仍明确标记为 `segmentIndex=2`。
- UI 同时直接统计 RobotComm 的 `STARTPOSSEG` 边沿并打印 JBI 行号。即使某个
  定位面所有图像提取均失败，也会保留该面的空点云槽并报告“该面无有效三维点”，
  不再把它误判为 JBI 只有一个定位面。
- Vision 同一时刻最多一帧在处理、一个最新帧待处理；新帧覆盖旧待处理帧，
  避免无界积压。实时相机显示不经过该限流链，因此不会因此卡顿。
- 真实点只有同时满足 `traflag==1` 且 `segmentIndex>0` 才写入当前定位面。
  第一定位面的三个 `STARTDETPRG` 窗口仍合并到面1，第二定位面的一个窗口写入面2。
- 代码中不存在 10 万点上限。点数只在上述采点窗口内增长；窗口间 MOVJ 和
  `ENDPOSSEG` 后停止增长是预期行为。UI 的“当前定位面”由 `segmentIndex`
  更新，即使该面当前 Steger 提取失败，也会从面1切换到面2。
- 实时图像现在由 `MainWidget` 在缩放后的显示 pixmap 上叠加黄色 OCCT 虚拟
  激光参考线；只绘制参考线，不复制或修改原始相机帧，且超过 100 pixel 的
  模型遮挡断口不会被错误连线。
- SWv1 面扫描 ROI 外扩值为 100 pixel，Qt 已从 80 调整为 100，以降低参考线
  存在小量投影偏差时 Steger 只得到 0～1 个候选点的概率。

## 2026-08-11 修正：补齐变位机工装 TCP、按当前定位面生成先验并恢复 SWv1 点云清理

针对现场出现 `modelHits>0, imageHits=0`、虚拟线与真实线距离很远、真实/虚拟
定位面整体相隔较远以及 ICP 无对应点的问题，重新逐矩阵核对了
`initialVirtualInformation()`、`generate_simu_laser1()`、`updateRealPointCloud()` 和
`stepICPforOffline()`。结论与修正如下：

- SWv1 在变位机末端安装了 `conf/positiontool20211124-jiyan.tool`，其 TCP 为
  `(0,0,0.231)m`。工件是该 TCP 的子对象，真实扫描点也通过
  `virtualSlaveRobot->get_endAbsPO().i()` 转到同一个 TCP 系。原 Qt 坐标链只有
  RBCALIB 和两轴旋转，漏掉 231 mm 工装 TCP，导致虚拟激光投影与两类点云出现
  共同的大尺度偏移。当前 `PositionerCalibration` 在解析 RBCALIB 时恢复该固定
  TCP，真实点投影和 OCCT 虚拟激光使用同一变换链；通用的
  `RobotPos2StationPos/StationPos2RobotPos` 兼容换算仍保持旧公式，不额外引入偏置。
- SWv1 的虚拟传感器始终在完整工件上求每条射线的最近交点，然后仅当该最近
  交点的 face id 属于当前 `position_face_dof_list[positionSegCount]` 时保留。
  Qt 现解析 weldment 中每个 `position_segment_N` 的 face id，并在
  `traflag==1 && posflag!=0` 时只发布当前 STARTPOSSEG 对应的先验；定位面外不再
  周期性求交和报警，异步返回的上一定位面先验也不会被下一定位面误用。
- SWv1 全局采点调用 `updateRealPointCloud(curPose, 5)`，即每 5 个中心线像素取
  一个三维点。分段 ICP 前只处理真实点云：FPS 保留 1/5，再做 SOR
  (`MeanK=5`, `StddevMulThresh=2`)；第二次 FPS 在该旧函数中已被注释。Qt 已恢复
  这套全局专用预处理，并把最大对应距离设为“PCL 默认”（与 SWv1 未调用
  `setMaxCorrespondenceDistance` 一致），避免通用 10 mm 阈值在初值稍偏时直接
  报 `Not enough correspondences`。
- 输出目录继续保存原始真实 PLY，同时新增每面及汇总的
  `global_locate_real*_preprocessed.ply`。控制台会打印相机外参、231 mm 工装 TCP、
  当前定位面/face 数、虚拟线命中数量和 ICP 预处理前后点数，便于在运行电脑上
  区分“标定/坐标链错误”“当前定位面没有命中”和“真实点离群”三类问题。

### 4. RobotSim 数据与 SWv1 内置虚拟数据的对应
见下方“RobotSim 与 SWv1 数据对比”。

## RobotSim 与 SWv1 数据对比
- SWv1 没有 RobotSim，其“仿真数据”由内置 OCCT 虚拟激光传感器（SwLSLSensor）与场景实时自产。
- qt-sontech 的 DigitalTwin 对接外部 RobotSim，经 ZeroMQ 返回 `DigitalTwinVirtualReferenceResult`。
- 字段对应：
  | SWv1 内置虚拟数据 | RobotSim / qt-sontech |
  | --- | --- |
  | 仿真激光线 2D（generate_simu_laser1，用于 Gabor/ROI） | laserLineImagePoints_px（实测 29 点） |
  | 仿真激光线 3D（get_output_point3d） | laserLinePoints_mm |
  | 焊缝点 2D（checkLaserSeamIntersection） | seamPointImage_px（实测如 (1034,374)） |
  | 虚拟局部点云（aVirtualPointList） | localPointCloud_mm |
  | CAD 定位面采样点云（stepICPforOffline 内采样，ICP target） | RobotSim 不提供；由 WeldmentVirtualPointCloudGenerator 自产 |
  | 工件模型位姿 / VirtualWorkpiece / OCCT 三维场景 | referencePose；qt-sontech 无 OCCT 场景写回 |
- 差异：SWv1 先验自产且随帧实时；RobotSim 是外部进程查询、数据点少（激光线 29 点）。
  本轮全局定位已改为完全自产先验与虚拟点云，RobotSim 仅作为可选在线数据源保留。
