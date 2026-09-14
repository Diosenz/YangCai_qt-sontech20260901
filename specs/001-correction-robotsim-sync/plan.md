# Implementation Plan: 在线/离线纠偏逐帧 RobotSim 同步

**Branch**: `detached-or-unavailable` | **Date**: 2026-08-31 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/001-correction-robotsim-sync/spec.md`

## Summary

在现有 UI runtime 编排边界上，将已经为全局定位使用的“冻结相机帧 + RobotState 后逐帧查询 RobotSim”公共机制扩展到正式离线和在线纠偏。协调器为一次正式纠偏维护跨内部 Vision 暂停的单调会话 ID，以一个 current 和一个 latest-pending 槽提供有界背压；RobotSim 结果在进入 Vision 前校验 request/session/frame，有效同帧结果与冻结帧一起提交既有 Vision 入口，Vision 回传 UI 运行时关联元数据后才确认消费并进入现有 Online/Offline worker。

实现先完成公共同步骨架和离线专项闭环，离线成功、背压、失败、停止、重启和迟到返回全部通过后再启用在线目的。连续第 3 帧同步链故障先失效会话，再复用现有 Vision/RobotComm 停止和纠偏 worker reset/fail 路径；不修改 Vision、Localization、IBVS 算法、Control 命令规则或全局定位专属信号/段号语义。

## Technical Context

**Language/Version**: C++17
**Primary Dependencies**: CMake 3.20+；Qt 5/6 Core、Gui、Widgets；DigitalTwin 现有 cppzmq/RobotSim facade；Vision 现有 OpenCV 3.4.x 路径（不改算法）
**Testing/Evaluation**: `tests/` CTest；脚本化 `IRobotSimTransport`；Qt event-loop/thread integration tests；现有 DigitalTwin、runtime/global-localization、online/offline controller、start-gate 和 MainWidget 回归
**Target Platform**: Windows x64，Visual Studio 2022 / MSVC，Visual Studio 管理的 CMake/Ninja Developer 环境
**Performance Goals**: 任意输入速率下 current<=1、pending<=1；停止后新增逐帧查询=0；错帧、旧会话路由和旧 observation 复用=0；不设新的单帧延迟目标
**Constraints**: 运行时边界快照而非曝光硬同步；一次正式纠偏只有一个活动 session；在线内部暂停不换 session；同步失败阈值固定 3；普通实时 RobotSim 查询在纠偏期间暂停但 latest RobotState 继续缓存；不得使用手动缓存回退；不得修改全局定位专属行为或 IBVS；不得产生新依赖
**Scale/Scope**: UI runtime/worker/UiController/MainWidget，DigitalTwin 只复用现有服务和测试接缝，RobotComm 只复用停止状态；新增一组在线/离线逐帧专项测试和一份 legacy 证据文档

## Ownership and Integration Impact

**Owning Module/Developer Area**: `UI` 运行时编排为主责；`DigitalTwin` 提供已有 RobotSim 查询边界。`Vision`、`Localization`、`IBVS` 算法与 `Control` 命令规则保持只读/兼容。
**Shared Areas Touched**: `UI/`、`tests/`、`docs/`、`tests/CMakeLists.txt`；可能只为测试注入扩展 UI worker 对现有 `IRobotSimTransport` 的构造路径。`DigitalTwin/` 生产接口预计无需修改。
**Coordination Required**: UI 共享运行时类型增加 correction session/purpose 关联和结构化同步通知；实现前需由相关维护者确认该 UI 公共 DTO/信号变化不会与另一开发者的 Vision/Control 工作冲突。若发现必须改变 Vision、Localization、IBVS 或 DigitalTwin 公共契约，按规格 FR-027 暂停并另行评估。
**Integration Target**: 当前工作区无 Git 元数据，目标分支不可判定。合入前按 `docs/collaboration_git_workflow.md` 确认实际集成分支、同步冲突并在 Visual Studio 2022 中完成全部规定验证。

## Change and Evaluation Design

**Variation Point**: 跟踪 purpose 是否要求逐帧 RobotSim 先验，以及外部 RobotSim 查询的成功、阻塞、失败和迟到时序。该变化点已经由现有 RuntimeCoordinator purpose 路由和 `IRobotSimTransport` 边界证明。
**Stable Contract**: 帧到达时冻结 CameraFrame、RobotState validity/value、purpose、session 和流程上下文；逐帧 observation 必须 valid 且 frameId 匹配；Vision 结果必须 session/purpose/frame 全匹配；同步错误保留具体原因；Control 继续消费原有 `RuntimeTrackingResult` 检测字段和命令语义。
**Baseline**: 当前代码已实现 latest-pending 背压和全局定位逐帧 RobotSim 查询；在线/离线仍走 `submitFrameWithRobotState()`，可能依赖 Vision 最近 observation/legacy callback，UI 普通实时缓存不证明与 Vision 当前帧一致。
**Candidate**: 在 RuntimeCoordinator 增加独立 correction-session 生命周期和 correction frame correlation；复用同一 DigitalTwin worker 的 `generateRobotSimVirtualPrior()`；在 UI worker adapter 外层给 Vision 结果附加 correlation；增加 correction-only sync update signal、模式门禁、失败计数和完整停止/恢复普通实时同步。
**Shared Evaluation**: 离线和在线使用同一脚本化 transport、帧序列 100..105、相同冻结状态、相同错误矩阵和相同指标。离线 CTest 必须先通过，在线测试显式依赖离线测试；现有 runtime tracking flow 保护全局定位，现有 controller 测试保护命令规则。
**Experiment Isolation**: N/A。本特性是默认正式纠偏行为变更，不新增实验实现、配置开关或算法候选。

## Constitution Check

*GATE: Pre-research and post-design both PASS.*

- [x] The boundary corresponds to an evidenced variation point; stable code is not abstracted speculatively. RobotSim 外部通信和按 purpose 路由是现有变化点；不新增顶层模块、factory 或 plugin。
- [x] The owning module and any cross-owner/shared-area impact are explicit and coordinated. 主改 UI runtime；共享 DTO/信号和测试/CMake 需要维护者确认；Vision/Control/Localization/IBVS 算法不改。
- [x] Baseline and candidate use the same contract, data, preprocessing, metrics, and thresholds. 两条纠偏路径共用同一 observation、Vision 入口、输入序列、错误矩阵和 0 mismatch/1+1 bound/3-failure 阈值。
- [x] Refactor-preserved behavior and intentional behavior changes are separately identified. 规格和本计划分别列出全局定位/算法/命令保持项与正式模式、逐帧同步、三连停止等变化。
- [x] Experimental code cannot silently change the default production path and is reproducible. 非实验；fake transport 只在 tests 注入，生产默认 transport 不变。
- [x] Added architectural elements have a concrete benefit and a simpler alternative was considered. 新增结构化 sync update 和 session correlation 用于跨线程同帧证明与迟到隔离；多个松散信号、frameId-only 和缓存回退均不能满足契约。
- [x] Verification uses the shared `tests/`/CTest/replay path and matches behavioral risk. 专项覆盖阻塞、错帧、失败、停止、重启、迟到和命令抑制，并运行全局定位和状态机保护回归。

### Post-design gate notes

- Data model keeps session metadata in UI runtime rather than contaminating DigitalTwin/Vision domain types.
- Contracts preserve the global-localization notification and segment-index path unchanged.
- The only test substitution boundary is existing `IRobotSimTransport`; no duplicate provider interface is introduced.
- Full stop invalidates correlation before queued stop/reset operations, making failure containment observable and race-safe.
- Legacy source is unavailable in the current workspace; VR-001 therefore uses the documented evidence gap plus existing migration documents, current implementation, and regression tests as the accepted baseline. Legacy source verification remains explicitly skipped and is not represented as completed.

## Project Structure

```text
specs/001-correction-robotsim-sync/
├── spec.md
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
├── contracts/
│   ├── correction-frame-sync-contract.md
│   └── correction-ui-state-contract.md
└── checklists/
    └── requirements.md
```

**Affected Repository Paths**:

```text
UI/include/RuntimeTrackingTypes.h
UI/include/RuntimeCoordinator.h
UI/src/RuntimeCoordinator.cpp
UI/include/RuntimeWorkerAdapters.h
UI/src/RuntimeWorkerAdapters.cpp
UI/include/UiController.h
UI/src/UiController.cpp
UI/include/MainWidget.h
UI/src/MainWidget.cpp
UI/CMakeLists.txt                         # only if a new UI source is necessary; prefer existing files
tests/test_correction_robot_sim_frame_sync.cpp
tests/test_digital_twin_service.cpp
tests/test_runtime_tracking_flow.cpp       # regression assertions only; do not change global behavior
tests/test_main_widget_panels.cpp
tests/CMakeLists.txt
docs/correction_robot_sim_frame_sync_legacy_notes.md
```

Protected paths:

```text
Vision/                                    # no algorithm changes
Localization/                              # no algorithm changes
IBVS/                                      # no changes
Control/                                   # no algorithm/command-rule changes
oldproject/                                # read-only if later available
RobotSim/                                  # read-only
```

**Structure Decision**: 现有 UI runtime 已拥有相机、RobotState、DigitalTwin worker、Vision worker 和 correction worker 的线程路由，因此它是唯一能在不跨线程共享引用的前提下冻结并关联一帧数据的位置。DigitalTwin 保持 RobotSim facade，Vision 继续只处理输入，Control 继续只处理检测业务事件。纠偏 session/update 类型放入现有 UI runtime types；不新增顶层模块。测试通过已有 transport 接口注入外部时序。

## Detailed Design

### Runtime correlation and state

- 在 UI runtime types 增加 correction sync state/update 和 Vision correlation 字段；现有检测字段及默认值保持兼容。
- RuntimeCoordinator 私有地保存单调 `nextCorrectionSessionId`、活动 session/purpose、failure count、current/pending 冻结帧上下文和逐帧 requestId。
- session 在正式 `startOnlineTracking()` / `startOfflineCorrectionDetection()` 建立；在线 worker 的临时 `stopTrackingRequested` 只暂停 Vision，不结束正式 correction session。
- controller Completed/Error、用户 stop/reset、三连同步失败才结束 session。

### Frame dispatch

- `enqueueVisionFrame()` 一次性冻结 hasRobotState/value、purpose、session 和全局定位既有 segment context。
- 保持一个 current + 一个 latest pending。对 correction，current 从 RobotSim 查询一直占用到匹配 Vision 结果；对 global localization 保持现有专属路由和段号语义。
- correction 无有效 RobotState或真实 RobotSim 未连接时直接构造同步失败，不派发零状态查询。

### RobotSim routing

- 普通 realtime state synchronization 在 correction session active 时只更新 latest RobotState，不派发请求；开始会话时清 pending realtime，结束时用 latest state 恢复。
- 逐帧 correction 和 global localization 复用现有 worker query，但 handler 根据 frozen purpose/session 路由；global-only signal 只在 GlobalLocalization 路径发出。
- observation 在 Vision 前校验 valid、requestId、session、purpose 和 frameId；失败保留原 message，错帧追加 expected/actual。

### Vision and Control routing

- 为 correction 使用一个 worker-adapter 入口或显式 correlation 参数，在不改变 Vision 算法的前提下给 `RuntimeTrackingResult` 回填 sessionId/purpose。
- `routeTrackingResult()` 先校验 current correlation；只有匹配结果才能 emit UI consumed、更新 RobotComm detection status、进入对应 correction worker、释放 current。
- no-target 是匹配的 Vision 业务结果：确认 observation 已消费，但 Control 仍按现有 valid/sample 规则忽略，不影响同步失败计数。

### Failure and stop

- failure 1/2：清 current observation，发布具体错误，释放 current 并继续 latest pending。
- sync success：在提交 Vision 前 failure count=0，发布 ObservationReady。
- failure 3：先失效 session/current/pending/request，再停止 Vision/Robot tracking，调用 online worker reset 或 offline worker fail/reset 适配，阻止迟到结果进入 Control，发布 StoppedAfterFailures，恢复 ordinary realtime。
- 所有用户 stop、panel close、controller terminal update 复用同一 session cleanup，清除 UI pending observation。

### UI

- UiController 在发送 OnlineGlobalLocate/OnlineCorrection/Offline JBI 前验证真实 RobotSim 状态；独立 GlobalLocateStandalone 不改变。
- MainWidget 在线/离线页各增加最小同步状态 label，避免现有 controller state text 覆盖。
- UiController 维护 pending observation key 和 last consumed frameId；只按 session/purpose/frame 三键确认，失败/停止/restart 清 pending。

### Test seams and order

- 将 `RobotSimDigitalTwinService(std::unique_ptr<IRobotSimTransport>)` 的现有注入能力透传到 worker/runtime 测试构造路径，默认构造仍使用真实 transport。
- 新专项 executable 支持 `--offline` 与 `--online`；CMake 注册两个测试，并设置 online `DEPENDS` offline。
- scripted transport 记录查询计数、冻结关节请求和阻塞释放；测试观察 sync update、tracking result、controller update 和 robot command record。
- wrong-frame 使用生产 observation 校验路径直接注入 DTO；不修改 RobotSim 协议制造不存在的行为。

## Implementation Sequence

### Phase A - Evidence and baseline gate

1. 定位只读 legacy 源码并读取其更近层级 `AGENTS.md`；记录在线/离线启动、停止、错误与命令抑制发现。
2. 记录当前相关测试基线，不修改全局定位测试预期。
3. 确认共享 UI runtime DTO/信号变更获得相关维护者协调。

Exit: VR-001 文档存在；若 legacy 仍不可用，记录检索范围和缺口，并在用户接受替代基线后继续。最终报告将源码核对标记为 skipped。

### Phase B - Shared contract and service protection

1. 增加 UI runtime correlation/sync update 值类型和 metatype 注册。
2. 增加 observation valid/frameId/error 保留测试，确认 DigitalTwin 生产接口无需改变。
3. 透传现有 transport 注入供专项测试使用，不改变默认生产构造。

Exit: DigitalTwin service tests and existing runtime tests pass.

### Phase C - Offline P1 end-to-end

1. 实现 correction session、冻结上下文、1+1 backpressure、普通 realtime pause/resume。
2. 仅为 OfflineCorrection 启用逐帧 query/validation/Vision correlation。
3. 实现 failure count、three-failure full stop 和 offline UI pending/consumed/error 状态。
4. 完成 offline 专项全部场景和指标。

Exit: offline success/backpressure/failure/stop/restart/late tests 100% pass；否则禁止启用 online。

### Phase D - Online P2 reuse

1. 将已经通过的同一 contract 启用到 OnlineCorrection，不复制第二套队列或失败状态机。
2. 保证 online 内部 Vision pause/resume 保持同一 session，no-target 不计同步失败。
3. 验证 third-failure 后无新 P 命令、old-session result 不进入新 Control session。

Exit: online 专项 100% pass，且 CTest 依赖证明离线先通过。

### Phase E - UI gate and full regression

1. 在线/离线 JBI 前增加真实状态同步模式门禁与明确文案。
2. 完成两个页面最小同步状态显示和清理语义。
3. 运行 quickstart 中 shared/global-localization/controller/UI 回归，审查 protected path 变更数。

Exit: 所有 SC 指标满足，验证报告区分 passed/failed/skipped 和现场限制。

## Risks and Mitigations

| Risk | Evidence | Mitigation |
|---|---|---|
| 旧 Vision 结果在 restart 后到达 | 当前 result 只有 frameId，coordinator 先清 in-flight | 增加 session/purpose correlation，结果不匹配不得释放 current。 |
| online 内部暂停被误认为 session stop | controller 多阶段会多次 start/stopTracking | correction session 与 `m_trackingPurpose`/Vision active 分离，终态才结束。 |
| 普通 realtime 与逐帧查询争用单 worker | 当前两者共享 m_robotSimWorker | correction active 阻止普通续派；latest state 仅缓存，stop 后恢复。 |
| 三连失败只停 Vision，Control 仍活动 | current stop lambda 主要清 tracking purpose | 失效 session 后同时停止 Robot/vision 并 reset/fail correction worker。 |
| UI controller 状态文本覆盖 sync 错误 | online/offline state label 被 controller update 重写 | 新增独立最小 sync label，不重构完整面板。 |
| 测试无法控制真实 RobotSim | 外部进程/网络时序不可重复 | 复用 `IRobotSimTransport` 脚本 fake，生产默认不变。 |
| Legacy 源码当前缺失 | workspace 无 `oldproject/` | 文档记录精确缺口；以迁移文档、当前实现和回归测试作为替代基线，最终将源码核对标记为 skipped，不伪称已验证。 |

## Complexity Tracking

No Constitution violations or temporary exceptions are planned.
