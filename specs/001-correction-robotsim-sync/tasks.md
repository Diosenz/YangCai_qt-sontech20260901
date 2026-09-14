# Tasks: 在线/离线纠偏逐帧 RobotSim 同步

**Input**: `specs/001-correction-robotsim-sync/` 下的规格、计划、研究、数据模型、契约和验证指南

**Organization**: 任务按实施证据基线、共享同步基础、离线 P1、模式/UI P1、在线 P2、回归与交付证据组织。测试是规格明确要求的交付内容；离线专项必须先通过，之后才允许开始在线专项。

## Format: `[ID] [P?] [US?] Description`

- **[P]**: 前置任务完成后可与同阶段标记任务并行，且不修改相同文件
- **[US1]**: 离线纠偏逐帧同步与安全停止
- **[US2]**: 在线纠偏复用同一同步契约
- **[US3]**: 正式模式门禁与同帧 UI 确认
- 每个任务都给出精确仓库路径和完成证据

## Phase 1: 实施门禁、所有权与基线

**Purpose**: 在任何业务代码修改前补齐 legacy 证据、共享区域协调和可比较基线。

- [X] T001 读取 `oldproject/sontech-works-version-storage/AGENTS.md`，在其规则指导下只读检索 `oldproject/sontech-works-version-storage/` 中在线/离线纠偏的启动、停止、错误处理及命令抑制实现，并把函数/文件证据与保持项写入 `docs/correction_robot_sim_frame_sync_legacy_notes.md`；若只读源码不可用，则记录准确检索范围和证据缺口，以现有迁移文档、当前实现和回归测试作为替代基线，并将源码核对明确标记为 skipped，不得伪称已由 legacy 源码验证
- [X] T002 阅读 `docs/I2R_Cpp_Codex_Rules.md` 与 `docs/localization_visual_servo_digital_twin_scope.md` 的适用章节，确认 `UI/include/RuntimeTrackingTypes.h` 的共享 DTO/信号变更已与相关维护者协调，并通过 Visual Studio 2022 记录 `tests/test_digital_twin_service.cpp`、`tests/test_runtime_tracking_flow.cpp`、`tests/test_correction_robot_start_gate.cpp`、`tests/test_main_widget_panels.cpp`、`tests/test_online_correction_controller.cpp`、`tests/test_offline_correction.cpp` 和 `tests/test_localization_laser_line_observation.cpp` 的变更前结果到 `docs/correction_robot_sim_frame_sync_validation.md`；完成证据为环境、目标名、通过/失败/跳过及既有失败原因均可复核，且未修改测试预期

**Checkpoint**: Legacy 核对已完成、共享 UI 所有权已协调、基线可复核；任一项缺失都阻止生产实现。

---

## Phase 2: 共享逐帧同步边界

**Purpose**: 建立在线和离线共同依赖的值类型、测试接缝与唯一有界运行时状态机，同时保持全局定位专属语义不变。

- [X] T003 在 `UI/include/RuntimeTrackingTypes.h` 定义 correction purpose/session、Vision correlation 和结构化 sync update 值类型及 Qt metatype，在 `UI/include/RuntimeWorkerAdapters.h`、`UI/src/RuntimeWorkerAdapters.cpp`、`UI/include/RuntimeCoordinator.h`、`UI/src/RuntimeCoordinator.cpp` 透传现有 `IRobotSimTransport` 测试注入且保持生产默认构造不变，并在 `tests/test_digital_twin_service.cpp` 验证成功/失败 frameId、invalid/empty 状态和原始错误信息保持；完成证据为 `i2r_rlc_digital_twin_tests` 与现有 runtime 测试通过且 DigitalTwin 生产公共接口无需改变
- [X] T004 在 `UI/include/RuntimeCoordinator.h`、`UI/src/RuntimeCoordinator.cpp`、`UI/include/RuntimeWorkerAdapters.h` 和 `UI/src/RuntimeWorkerAdapters.cpp` 实现一个共享 correction session、按值冻结 frame/RobotState/purpose/context、一个 current 加一个 latest-pending 槽、request/session/purpose/frame 校验、普通实时 RobotSim 派发暂停/恢复及统一失效清理；在 `tests/test_runtime_tracking_flow.cpp` 只增加保护性断言，完成证据为 current<=1、pending<=1、旧 request/session 不改变当前状态，且既有全局定位 prior 通知和段号断言原样通过

**Checkpoint**: 共享边界可测试、没有第二套队列/transport 抽象，全局定位行为未改变。

---

## Phase 3: User Story 1 - 离线纠偏逐帧同步与安全停止 (Priority: P1)

**Goal**: 每个离线检测帧使用到达运行时边界时冻结的有效 RobotState 查询同帧 RobotSim observation，并在失败、停止和重启下保持有界且隔离。

**Independent Test**: 脚本化 transport 对离线 purpose 覆盖阻塞、成功、缺状态、查询失败、无效/空返回、错 frameId、停止、重启、RobotSim/Vision 迟到返回、三连失败及普通实时恢复；所有 Vision 输入同帧、容量不超过 1+1、停止后查询为 0。

- [X] T005 [US1] 创建 `tests/test_correction_robot_sim_frame_sync.cpp` 的 `--offline` 专项夹具与计数器，并在 `tests/CMakeLists.txt` 注册 `i2r_rlc_correction_robot_sim_offline_frame_sync_tests`，覆盖帧 100 阻塞时 101..105 仅保留 105、无 RobotState 不查询、错误/空/无效/错帧不复用旧 observation、1/2 次失败后成功清零、3 次失败完整停止、查询/Vision 阻塞时停止、停止后重启与两类迟到结果、普通实时暂停/恢复；完成证据为每个验收分支都有独立断言，并在 T006 前能明确暴露尚未实现的离线行为
- [X] T006 [US1] 在 `UI/include/RuntimeCoordinator.h`、`UI/src/RuntimeCoordinator.cpp`、`UI/include/RuntimeWorkerAdapters.h` 和 `UI/src/RuntimeWorkerAdapters.cpp` 仅为 `OfflineCorrection` 启用 T004 的逐帧查询与 Vision correlation，按同步链故障维护固定阈值 3、成功 observation 在 Vision 业务前清零、第三次失败先失效会话再复用既有 offline fail/reset 与 Vision/RobotComm 完整停止路径，并确保清理或迟到结果不产生 O 命令；完成证据为 T005 离线专项全部通过且输出 frame mismatch=0、max current=1、max pending=1、post-stop query=0、old-session route=0、stale observation reuse=0

**Checkpoint**: 离线专项 100% 通过；未达到此检查点时禁止执行 T008 或 T009。

---

## Phase 4: User Story 3 - 正式模式门禁与同帧 UI 确认 (Priority: P1)

**Goal**: 正式纠偏只能在 RobotSim 真实状态同步模式启动，并让操作人员看到与 Vision 实际消费严格同会话、同 purpose、同 frameId 的同步状态。

**Independent Test**: 使用合成 correction sync update 和现有控制器门禁测试，不依赖在线纠偏实现即可验证手动模式在 JBI 前被拒、三键匹配才确认、失败/替换/停止/重启清 pending、三连失败原因可见。

- [X] T007 [US3] 在 `UI/include/UiController.h`、`UI/src/UiController.cpp` 实现在线/离线 JBI 前真实状态同步门禁及 `(sessionId,purpose,frameId)` pending/consumed 匹配，在 `UI/include/MainWidget.h`、`UI/src/MainWidget.cpp` 为两个页面增加独立的同步状态、最近确认 frameId、最近错误显示，并扩展 `tests/test_correction_robot_start_gate.cpp` 与 `tests/test_main_widget_panels.cpp` 覆盖手动/连接中/未连接拒绝、独立全局定位不变、错会话/错帧不确认、失败/替换/停止/重启清理和三连失败文案；完成证据为两个现有测试目标通过且全局定位/普通实时/手动 observation 均不能写入纠偏 pending

**Checkpoint**: 离线 P1 与公共模式/UI P1 均可独立验收；这是建议的 MVP 边界，但不代表完整特性完成。

---

## Phase 5: User Story 2 - 在线纠偏复用同一同步契约 (Priority: P2)

**Goal**: 在离线专项通过后，把同一会话、背压、失败和 UI 契约启用到在线纠偏，不复制状态机且不改变在线分段或 P 命令规则。

**Independent Test**: 在离线 CTest 成功后运行在线 purpose 的同一脚本化矩阵，额外验证内部 Vision 暂停/恢复不换 session、no-target 不计同步失败、第三次失败后无 P 命令、旧会话结果不进入新 Control 会话。

- [X] T008 [US2] 在 T006 离线专项 100% 通过后，扩展 `tests/test_correction_robot_sim_frame_sync.cpp` 的 `--online` 场景，并在 `tests/CMakeLists.txt` 注册 `i2r_rlc_correction_robot_sim_online_frame_sync_tests` 且用 CTest `DEPENDS` 显式依赖 `i2r_rlc_correction_robot_sim_offline_frame_sync_tests`；覆盖离线同等矩阵及在线内部暂停/恢复、no-target、第三次失败命令抑制和旧 session Control 隔离，完成证据为在线测试无法在离线失败时被当作有效验收且能暴露尚未启用的在线路径
- [X] T009 [US2] 在 `UI/include/RuntimeCoordinator.h`、`UI/src/RuntimeCoordinator.cpp`、`UI/include/RuntimeWorkerAdapters.h` 和 `UI/src/RuntimeWorkerAdapters.cpp` 将 T006 已验证的同一逐帧同步路径启用到 `OnlineCorrection`，保持在线内部 Vision pause/resume 的 correction sessionId 不变、Vision no-target 只作为同步成功后的业务结果、第三次同步故障先失效会话再走既有 online reset/完整停止且不产生 P 命令；完成证据为离线专项仍先通过、随后在线专项全部通过，且 `tests/test_online_correction_controller.cpp` 的既有分段和命令触发断言无需改变

**Checkpoint**: 在线与离线专项均 100% 通过，所有有效 Vision/UI consumed 关联错配为 0。

---

## Phase 6: 全量回归、范围审计与交付证据

**Purpose**: 证明共享基础设施兼容、明确现场或工具限制，并核对受保护路径零修改。

- [X] T010 [P] 通过已打开的 Visual Studio 2022 在当前 Qt Debug CMake 配置完成生成，并按 `specs/001-correction-robotsim-sync/quickstart.md` 顺序运行 `tests/test_digital_twin_service.cpp`、离线专项、在线专项、`tests/test_runtime_tracking_flow.cpp`、`tests/test_correction_robot_start_gate.cpp`、`tests/test_main_widget_panels.cpp`、`tests/test_online_correction_controller.cpp`、`tests/test_offline_correction.cpp` 和 `tests/test_localization_laser_line_observation.cpp` 对应目标；完成证据为 VS 最终生成状态、各目标通过/失败/跳过和离线先于在线的时间顺序均被保存，不以修改回归预期换取通过
- [X] T011 [P] 按 `docs/collaboration_git_workflow.md` 识别实际集成目标并审查完整变更列表，确认 `IBVS/`、`oldproject/`、`RobotSim/` 文件变更为 0，`Vision/`、`Localization/` 算法及 `Control/` 命令规则为 0 变更，全局定位专属接口/信号/数据流无语义变化，并把路径级审计和共享 UI 影响写入 `docs/correction_robot_sim_frame_sync_validation.md`
- [X] T012 汇总 T010 与 T011 到 `docs/correction_robot_sim_frame_sync_validation.md`，分别记录离线/在线成功、背压、失败、停止、重启、迟到返回、失败恢复/三连停止指标，标明现场 RobotSim 检查和 legacy 证据的 passed/failed/skipped 状态，并逐项核对 `specs/001-correction-robotsim-sync/spec.md` 的 SC-001 至 SC-010；完成证据为所有必需自动验收通过、任何未完成现场项不被描述为成功、且完整特性只有在离线和在线均满足后才标记完成

**Checkpoint**: 自动化证据、范围保护和剩余现场限制均可审计，且没有把缺失证据或失败伪装为成功。

---

## Dependencies and Execution Order

### Hard gates

- T001 是实施证据基线；legacy 源码不可用时允许使用已记录的证据缺口和替代基线继续，但最终验证必须将源码核对标记为 skipped。
- T002 → T003 → T004 建立基线、共享值类型/测试接缝和唯一同步状态机。
- T004 → T005 → T006 完成并验证离线 P1；T006 未通过时禁止 T008/T009。
- T006 → T007 完成全部 P1 交付（离线链路以及模式/UI 契约）。
- T006 + T007 → T008 → T009 才能启用和验收在线 P2。
- T009 → T010/T011；T010 与 T011 可并行，二者完成后执行 T012。

### User story dependency graph

```text
T001 Legacy Gate
  -> T002 Baseline/Coordination
    -> T003 Shared Types/Test Seam
      -> T004 Shared Session + 1/1 Backpressure
        -> US1: T005 Tests -> T006 Offline Implementation
          -> US3: T007 Mode Gate + Same-frame UI
            -> US2: T008 Tests -> T009 Online Implementation
              -> T010 Build/Regression ┐
              -> T011 Scope Audit      ├-> T012 Evidence Handoff
                                       ┘
```

### Parallel opportunities

- T010 与 T011 在 T009 完成后可并行：T010 只运行构建/测试并收集执行证据，T011 只做变更范围与契约审计并更新文档。
- 其余任务刻意串行，因为共享 `RuntimeCoordinator`/worker 文件、测试先行依赖、离线先于在线的验收门禁或 UI 共享契约会造成真实冲突。

## Independent Verification Summary

| Story | 独立完成标准 |
|---|---|
| US1 | `i2r_rlc_correction_robot_sim_offline_frame_sync_tests` 独立覆盖成功、1+1 背压、同步错误、停止、重启、RobotSim/Vision 迟到、三连停止和普通实时恢复；六项关键错配/泄漏计数均为 0。 |
| US3 | 现有 start-gate 与 MainWidget 测试可仅凭模式状态和合成 sync update 证明 JBI 前拒绝、三键确认、pending 清理及最小 UI 状态，无需在线实现。 |
| US2 | 在线专项复用同一夹具并由 CTest 依赖离线专项，额外证明内部暂停不换 session、no-target 不计失败、三连失败无 P 命令及旧结果不进入新 Control 会话。 |

## Suggested MVP

完成 T001-T007：交付并验证离线逐帧同步、正式模式门禁和同帧 UI 确认。该 MVP 满足所有 P1 场景，但由于规格明确要求在线与离线均完成，不能把它标记为整个阶段完成。

## Notes

- 所有 C++ 修改均须保持 UTF-8（C++ 文件优先 UTF-8 with BOM）、4 空格缩进和必要的跨线程/生命周期注释。
- 不得为了通过本特性修改全局定位回归预期，或修改 `Vision/`、`Localization/`、`IBVS/` 算法及 `Control/` 命令规则。
- 若实现暴露必须改变 Vision、Localization、IBVS 或 DigitalTwin 公共契约的缺陷，停止当前任务并按 FR-027 单独评估。
- 构建与测试优先由 Visual Studio 2022 执行；不在普通 PowerShell 裸跑完整 MSVC/Ninja 构建。
