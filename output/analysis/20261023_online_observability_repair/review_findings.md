# 独立审查与边界

## 已修复阻断

- estimator：可见性边界平移不能独自产生焊点 X；不确定度必须作用主坐标，而非仅 KCF 质量。作者修正后由 detector_integration 复审，最新 estimator 回归已通过。
- recovery：无效 dt 前不能先推进 raw 差分；UI 早退须断 raw 邻接。新测试取得5项真实RED，修正后continuity和UI专项通过。
- geometry：候选referenceVerified后无交点不得回退；Bezier共面不能误当有限无交；现测试通过。
- worker：绑定变化清窗；非surface独立评价仅一次，稳定收益不可覆盖旧拒绝，总时间包含末尾门禁。
- history：candidateVirtualPoint不得通过observe/promote新锚入库。两项真实RED后修复；history专项通过。
- evidence：失败候选交点为null，旧点变换只作带来源的诊断；每个有限候选保留全部门禁结论。

## 周边离线专项失败（本轮未修改其生产和夹具）

safety_regression_tests.log：13项中12通过，offline_frame_sync专项失败。只读调用链核实：

- tests/test_correction_robot_sim_frame_sync.cpp:79 默认sStart=1，:475先发布状态，:490之后才startCorrection，:543只等WaitingForDetection。
- UI/src/RuntimeWorkerAdapters.cpp:3294-3317离线startDetection重置preflight但不启动Vision；:3393起要求后续D帧sStart=0/B040=0才:3443-3448发startTrackingRequested。sStart1先到直接Error。
- UI/src/RuntimeCoordinator.cpp:1730-1741仅收到该signal才启offline Vision；:2046起purposeNone会丢弃帧。故本组未进入本次online detector路径。
- tests/test_offline_runtime_worker_flow.cpp:183-190已有idle D后running D正向序列。
- RuntimeCoordinator.cpp与本轮前备份SHA256一致；offline worker cpp/header类段逐字一致。归因为既有夹具与当前预检契约冲突，未做修改前二进制对照，不将静态诊断表述为全量基线回归。

不为通过测试放宽生产启动门禁，也不扩改本轮之外的离线流程。需要独立任务更新该fixture：post-start idle→armed→running；无RobotState case应按预检拒绝定义，不能发idle后仍声称无状态。

## 保留限制

- 全步/半步/四分步共享生产2500ms上限，configure也钳制此上限；不能靠测试设10000解释为生产可超时。多候选在某些点云规模上可能耗尽预算并拒绝。
- 五秒后验证据窗、新曝光/连续3帧要求、结案冷却及清窗仍在；本轮不删门禁来提高频率。
- 多交点或无法复现当前参考则保留原门；未知写入不继续自动更新。
- 单图共同形变与刚体平移同构仍不可辨识；肩部estimated可以参与现有TCP跟随，不是防碰撞保证。
- 当前故障连续原图不完整，未启动设备/主程序/RobotSim，现场结果未验收。
- Notion使用此前完整正文快照，本轮在线正文读取未成功。