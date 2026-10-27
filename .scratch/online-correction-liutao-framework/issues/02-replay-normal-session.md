# 02: 贯通可重放的正常在线纠偏会话

Category: enhancement
Status: ready-for-human

**Parent:** [01 统一实施事项](01-implement-and-validate.md)。

**What to build:** 维护人员能够用同一份连续曝光与仿真输入，重放当前正常在线纠偏会话，看到从生产检测、真实空间投影、完整控制队列到模拟机器人通信的关联结果。先完成本机已有实现的针对性核实和最小接入准备，为后续复用提供明确依据，而不是先另写算法或另一套业务模拟器。

**Blocked by:** None (can start immediately)。

## Acceptance criteria

- [x] 建立论文机制、师兄候选工程、当前 Qt 已迁移能力与正式活跃路径的对应清单，区分已实现、被旁路、需要适配和确实缺失的能力；记录版本依据和选择理由。
- [x] 优先使用已存在的 runtime 会话入口、生产 worker 与设备传输替身，必要的接口整理先完成并保持现有行为；不以 Python 证据策略替代生产决策。
- [x] 一个正常有序会话可从原图、当帧姿态、仿真参考和机器人反馈，运行到空间参考、完整队列消费、实际发出的模拟命令及回执。
- [x] 明确关闭工件在线点云 ICP，仍能重放当前基础行为；单独记录控制层轨迹 ICP 的配置。
- [x] 输出统一的逐观测关联报告，包含实际选用算法、来源、参考版本、释放点与命令；保留现有行为作为后续同输入对照，不要求旧算法此时已满足最终精度。
- [x] 合成已知真值与真实曝光材料明确区分，真值只进入评分器；缺少原图或曝光姿态不补造为已验证输入。
- [x] 正常会话可重复得到相同业务结论，已有相关回归保持通过；旧来源及父事项不修改，不删除历史材料。

## Further notes

遵守[完整规格](../spec.md)和[初步复用线索](../source-evidence.md)。本任务中的小范围准备必须服务于这条完整可验证路径，不开展大范围重构，也不执行真实设备动作。

## Comments

- 2026-10-26：用户确认拆分后发布；增加先核实并复用师兄已有实现的要求。

### 2026-10-26：软件会话验收通过，待人工审阅

实现提交：`549b56870bd0f341eecf7482dcae70a4baedae0c`。本票软件验收已通过；`ready-for-human` 表示待审阅，不代表论文算法、现场严重失测保护或真实 TCP 精度已完成。父事项 01 保留不变。

- 论文机制、两个师兄候选实现与当前 Qt 活跃/旁路能力的对应及内容哈希，见[生产会话与复用依据](../../../docs/online_correction_session_replay.md)和[实施前研究](../implementation-research.md)。当前模式如实报告为几何证据与标准 KCF；没有提前声称三模板双核已在正式路径生效。
- 用户已授权在既有 qt-debug build tree 中使用 VS2022 Developer 环境构建和离线验证。本票由主代理串行构建，不建新依赖树，不启动真实设备。
- RED：同一个 `i2r_rlc_runtime_tracking_flow_tests --normal-session-replay-contract` 已实际运行；生产检测、曝光姿态投影、O/P 握手、真实前视队列消费与最终状态断言先通过，仅因缺少统一报告失败，见[RED 日志](../build-records/02-red-contract.log)。随后最小实现新增可选 recorder 报告和真实活跃路径标识，不替换检测或控制算法。
- GREEN：MSVC [runtime 目标构建](../build-records/20261026_191548_664923_i2r_rlc_runtime_tracking_flow_tests.log)成功；同一 contract [第一次](../build-records/20261026_192117_273741_i2r_rlc_runtime_tracking_flow_tests_normal-session-replay-contract.log)和[第二次](../build-records/20261026_192127_259898_i2r_rlc_runtime_tracking_flow_tests_normal-session-replay-contract.log)均 exit 0。两次独立目录的报告均为 28 个观察、24 个释放点、48 个消费目标、50 个生成命令事件、27 个实际 O/P 请求回执，最终均为 completed。
- 报告测试严格检查曝光 1～28、冻结时间、会话 902、参考版本 17、实际算法非 unknown、事件序号连续；实际回执中的请求列表必须与外部模拟 transport 真正收到的 O/P 请求逐项同序一致。50 个生成命令事件不冒充 50 次实际发送；通信合并后的实际发送由 27 个回执单独记录。
- 原有 [worker 闭环](../build-records/20261026_192128_865511_i2r_rlc_runtime_tracking_flow_tests_worker-closed-loop-contract.log)、[精定位/投影恢复](../build-records/20261026_192128_980694_i2r_rlc_runtime_tracking_flow_tests_seam-refinement-contract.log)均 exit 0；[recorder 专项目标构建](../build-records/20261026_192133_823625_i2r_rlc_online_correction_run_recorder_tests.log)及[专用回归](../build-records/20261026_192201_630337_i2r_rlc_online_correction_run_recorder_tests.log)也通过。

保留的证据边界：

1. 本用例是固定有序合成曝光、冻结位姿与仿真输入，经生产 worker 公开接口的确定性业务会话；不把重复形状的合成场景冒充真实预焊连续曝光。真值只供外部断言，不注入检测器。coordinator 跨线程调度与迟到事件继续由原有专项覆盖。
2. 工件在线点云 ICP 在组合中不实例化、不写回；控制层轨迹 ICP 明确开启并尝试求解，保留实测参考。Control recorder 不拥有工件 ICP 开关证据，所以统一报告中的该开关为 null，不能由控制层 enableOnlineIcp 推断工件 ICP 已开。
3. 控制样本目前未传播原始曝光身份，因此释放/控制事件的 source_exposure 明确为 null；这是同会话逐事件关联与实际队列消费报告，不宣称已经具备每个释放点到原曝光的一对一溯源。不会用最近帧补造出处。
4. 真实后半段连续预焊输入、对应曝光姿态和独立焊缝真值不足的问题尚存；没有声称解决截图漂移，或完成真实检测轨迹及 TCP 平均 2 mm／最大 5 mm 精度验收。后续任务继续在同一会话边界补场景与评分。
5. 只新增/修改本票必要源码、测试和文档；未删除历史材料、未修改旧来源及父事项、未调用鼠标、未进行真实设备动作。

- 2026-10-26 最终组合进度：补充当前状态：后续实现已将原始样本session/frame/revision贯通缓冲、release及实际控制队列首/方向两端；预测与模型重采样仍为null，不按最新帧补造。原“来源均null”段落保留为历史。本票待人工审阅。最终组合证据见[软件验证报告](../10-software-validation.md)：20261027_014203矩阵33/33（28专项+5完整目标）、主程序013805与runtime测试014116均MSVC成功；Spec/Standards跟进无尚未修复的软件阻断项。此结果不等于现场2/5mm或停弧验收。
