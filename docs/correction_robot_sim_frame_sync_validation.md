# 在线/离线纠偏逐帧 RobotSim 同步验证记录

## T002 变更前基线

### 环境与协调证据

- 记录日期：2026-08-31。
- 平台：Windows x64，Visual Studio 2022 17.14，MSVC 14.44，Qt 6.7.3。
- Visual Studio 工作区：`YangCai_qt-sontech1-RS-complete`。
- Visual Studio CMake 配置：`Build Qt Debug`，对应
  `out/build/qt-debug`、Ninja、`CMAKE_BUILD_TYPE=Debug`。
- 构建方式：在 Visual Studio 2022 的 CMake 目标视图逐个触发测试目标生成，并观察
  `cmake`、`ninja`、`cl.exe` 和输出窗口状态；未在普通 PowerShell 中裸跑 CMake/Ninja 生成。
- 运行方式：使用 Visual Studio 生成的测试可执行文件，在同一 build tree 中执行；Qt Widgets
  测试设置 `QT_QPA_PLATFORM=offscreen`。
- 共享区域协调：用户在本任务会话中明确要求撤销 legacy 硬门禁并继续完成 T001-T004，且再次明确要求
  使用 `speckit-implement` 完成 T001-T004。这构成本轮修改
  `UI/include/RuntimeTrackingTypes.h` 共享 DTO、Qt metatype 和相关 runtime 信号的授权与协调依据。
- 已阅读并适用：`docs/I2R_Cpp_Codex_Rules.md`、
  `docs/localization_visual_servo_digital_twin_scope.md`。本轮保持 DigitalTwin 为 RobotSim facade，
  不把同步状态机下沉到 Vision、Localization、IBVS 或 Control 算法实现。

### 基线结果

| 源测试 | Visual Studio 目标 | 结果 | 可复核说明 |
|---|---|---|---|
| `tests/test_digital_twin_service.cpp` | `i2r_rlc_digital_twin_tests` | Failed（既有） | 进程退出码 1；现有断言报告 `virtual reference should parse seam image point` 与 `vision observation conversion should reject missing seam point`。本轮基线阶段未修改测试预期。 |
| `tests/test_runtime_tracking_flow.cpp` | `i2r_rlc_runtime_tracking_flow_tests` | Passed | 输出 `PASS: runtime tracking flow tests`，退出码 0。 |
| `tests/test_correction_robot_start_gate.cpp` | `i2r_rlc_correction_robot_start_gate_tests` | Passed | 输出 `PASS: correction robot start gate tests`，退出码 0。 |
| `tests/test_main_widget_panels.cpp` | `i2r_rlc_main_widget_panel_tests` | Failed（既有） | 进程退出码 `0xC0000005`（access violation），在产生测试断言文本前异常退出；当前基线只记录事实，未猜测根因，也未修改预期。 |
| `tests/test_online_correction_controller.cpp` | `i2r_rlc_online_correction_tests` | Passed | 输出 `PASS: online correction controller tests`，退出码 0。 |
| `tests/test_offline_correction.cpp` | `i2r_rlc_offline_correction_tests` | Passed | 输出 `PASS: offline correction tests`，退出码 0。 |
| `tests/test_localization_laser_line_observation.cpp` | `i2r_rlc_localization_laser_line_observation_tests` | Passed | 输出 `PASS: Localization laser line observation tests`，退出码 0。 |

### 基线判定

- 5 个目标通过，2 个目标存在变更前失败。
- 两项失败均作为既有基线保留；T003/T004 不通过修改测试预期来隐藏失败。
- T003 的 DigitalTwin 定向测试必须在补齐 frameId、invalid/empty 和原始错误保持断言后通过；
  这意味着需要修复由新增保护断言确认的生产解析缺陷，而不是删除现有断言。
- MainWidget 不在 T003/T004 的修改范围内；其 access violation 作为后续 UI 阶段的独立既有风险保留。

## T003/T004 变更后验证

### Visual Studio 目标生成与运行结果

- 生成日期：2026-08-31。
- 生成方式：继续使用 Visual Studio 2022 的 `Build Qt Debug` CMake 目标视图；分别选择
  `i2r_rlc_digital_twin_tests` 和 `i2r_rlc_runtime_tracking_flow_tests` 可执行目标生成，
  Visual Studio 输出窗口报告生成成功。未在普通 PowerShell 中裸跑 CMake/Ninja 生成。
- `i2r_rlc_digital_twin_tests`：Passed，退出码 0。成功/失败结果均保留请求
  `frameId`；invalid、empty 和原始 transport/解析错误断言通过；二维早期 RobotSim
  焊缝点载荷与正式三维+二维载荷均可解析。
- `i2r_rlc_runtime_tracking_flow_tests`：Passed，输出
  `PASS: runtime tracking flow tests`，退出码 0。

### T003 共享值类型与测试接缝

- `CorrectionPurpose`、`CorrectionRunSession`、`RuntimeVisionCorrelation`、
  `CorrectionSyncState` 和 `CorrectionSyncUpdate` 均为按值 DTO，并已注册 Qt metatype。
- `RuntimeTrackingResult` 只新增默认无关联的 `correlation` 字段，普通跟踪和全局定位
  调用方保持兼容。
- RuntimeCoordinator 的测试构造路径把既有 `IRobotSimTransport` 所有权转移给
  DigitalTwin worker；生产默认构造仍创建原有真实 ZMQ transport。runtime 测试通过
  手动 RobotSim 场景连接确认 fake transport 的 `connect` 和 `send` 均被调用。
- 未修改 `IRobotSimTransport`、`IDigitalTwinService` 或其他 DigitalTwin 生产公共接口，
  未新增第二套 transport/provider 抽象。

### T004 会话、冻结帧与有界背压

- 相机帧在 RuntimeCoordinator 边界按值冻结完整 `CameraFrame`、RobotState 有效性和值、
  purpose、correction session、frameId 和既有全局定位段号；current 与 latest-pending
  均保存同一个不可拆分结构。
- 阻塞首个 Vision 回调并连续发布帧 100 至 105，实际只路由帧 100 和 105；两帧携带
  相同非零 session、OnlineCorrection purpose，且 correlation frameId 与结果 frameId
  分别一致。由此验证 current<=1、pending<=1，101 至 104 只覆盖 latest-pending。
- 在帧 100 的 Vision 回调阻塞期间停止并立即重启，随后建立新 session 和 current
  帧 200；帧 100 的旧 session 结果被丢弃且未释放新 current，最终只路由帧 200。
- 真实同步模式的 fake transport 计数验证：纠偏 session 活动期间发布多份 RobotState
  不增加普通实时 RobotSim 请求；用户停止后从最新缓存状态恢复一次普通实时同步。
- 既有全局定位保护断言保持原样：专属 prior 通知路径未改为 correction 通知，帧捕获时
  `localizationSegmentIndex` 的 1/1/1/0/2 转换与 traflag 断言继续通过。

### 剩余限制

- T003/T004 只建立共享会话、关联和有界派发骨架。正式离线逐帧 RobotSim observation
  查询、同步失败计数与三连失败停止属于 T005/T006；在线启用属于后续 T008/T009。
- T002 记录的 MainWidget access violation 未在本阶段处理；它不影响本次两个定向目标，
  但后续 UI 阶段仍需单独复核。
- legacy 源码核对仍按 T001 记录为 skipped；本记录不把替代基线描述为源码验证。

## T005-T009 离线、UI 与在线实现证据

### 离线专项（T005/T006）

- 新增目标 `i2r_rlc_correction_robot_sim_offline_frame_sync_tests`，使用可脚本化的
  `IRobotSimTransport` 和确定性 RobotSim/Vision 阻塞点，不依赖定时竞争。
- 覆盖无 RobotState、帧 100 在途时 101..105 仅保留 105、空/无效/transport
  失败、错 frameId 不变量、1/2 次失败后成功清零、连续第 3 次失败、停止/重启、
  RobotSim/Vision 两类迟到返回，以及普通实时同步暂停/恢复。
- 最终退出码为 0，输出：`frame mismatch=0, max current=1, max pending=1,
  post-stop query=0, old-session route=0, stale observation reuse=0`。
- 第 3 次同步失败先失效 runtime session，再通知离线 worker 进入既有 Error；清理和
  迟到结果均未产生 O 命令。

### 模式门禁与 UI（T007）

- 正式 OnlineCorrection/OfflineCorrection 在 JBI 前要求 RobotSim 真实状态同步；手动、
  连接中和未连接状态均拒绝。OnlineGlobalLocate 与 GlobalLocateStandalone 既有行为不变。
- 在线、离线页面各自保存同步状态、最近 Vision 已消费 frameId 和最近错误；UI 只有在
  `(sessionId, purpose, frameId)` 三键与 pending observation 完全匹配时确认消费。
- 失败、替换、停止、重启和三连失败终态均清理未确认 observation；普通实时、手动场景和
  全局定位通知不能建立纠偏 pending。
- `i2r_rlc_correction_robot_start_gate_tests` 与 `i2r_rlc_main_widget_panel_tests` 最终均通过。
  MainWidget 基线 access violation 经核对来自测试继续解引用已不属于当前持久 RobotSim
  布局的旧入口控件；测试改为验证现有持久连接区和页面契约，没有降低生产门禁预期。

### 在线专项与共享实现（T008/T009）

- 在线目标与离线目标编译同一份参数化夹具；未复制第二套 fake transport、队列或状态机。
  CTest 属性明确设置在线目标 `DEPENDS` 离线目标。
- T009 前，在线目标在 VS 成功编译后按预期失败于“未发起逐帧查询/未发布同步失败”，证明
  测试能够暴露在线缺口；T009 后同一目标通过。
- RuntimeCoordinator 将离线已验证分支泛化为唯一 correction 分支，在线/离线共用冻结
  RobotState、1+1 背压、request/session/purpose/frame 校验、失败计数和停止清理。
- 在线内部 Vision pause/resume 不更换 sessionId；no-target 在同帧 observation 成功后作为
  Vision 业务结果处理且同步失败计数保持 0；连续第 3 次同步失败不产生新的 O/P 命令。
- 最终在线输出与离线一致，六项错配/泄漏指标均为 0。

### 测试规模必要性审查

- 专项源文件只保留 9 个行为场景：7 个在线/离线共享验收场景和 2 个在线特有场景；每个
  场景均可直接映射到 VR-002、FR-008～FR-020 或 T008 的在线附加要求。
- 在线与离线通过一个 `TestMode` 夹具共用 transport、线程阻塞、重启和计数代码，避免把
  约 970 行并发夹具复制成两份。已删除临时 `DIAG` 输出，没有加入 UI 截图、算法精度或
  实现细节断言等与同步契约无关的测试。
- `runtime_tracking_flow` 中原先借正式在线入口验证 Vision fallback 的三组用例改为直接测试
  Vision adapter；正式在线 runtime 用例则注入 RobotSim 并等待真实场景同步。这样保留原
  行为证据，同时避免为了测试绕过新的正式模式门禁。

## T010 Visual Studio 生成与全量自动回归

- 日期：2026-09-01；Visual Studio 2022 当前配置为 `qt-debug-qt-debug`，build tree 为
  `out/build/qt-debug`。通过 VS 的“全部生成”触发 CMake/Ninja/MSVC，输出窗口最终报告
  `全部生成 已成功`；未在普通 PowerShell 中运行 `cmake --build`。
- 一次中断后的对象文件曾由链接器报告 `LNK1163 COMDAT`，通过重新保存对应 UTF-8 BOM
  源文件让 VS 重编该单个对象后恢复；没有删除源码、目录或重建 build tree。
- 下表按 quickstart 顺序执行，离线专项明确先于在线专项：

| 顺序 | 目标 | 结果 |
|---:|---|---|
| 1 | `i2r_rlc_digital_twin_tests` | Passed，exit 0 |
| 2 | `i2r_rlc_correction_robot_sim_offline_frame_sync_tests` | Passed，exit 0，六项指标为 0 |
| 3 | `i2r_rlc_correction_robot_sim_online_frame_sync_tests` | Passed，exit 0，六项指标为 0 |
| 4 | `i2r_rlc_runtime_tracking_flow_tests` | Passed，exit 0 |
| 5 | `i2r_rlc_correction_robot_start_gate_tests` | Passed，exit 0 |
| 6 | `i2r_rlc_main_widget_panel_tests` | Passed，exit 0 |
| 7 | `i2r_rlc_online_correction_tests` | Passed，exit 0；既有分段/P 命令断言未修改 |
| 8 | `i2r_rlc_offline_correction_tests` | Passed，exit 0；既有 O 命令规则未修改 |
| 9 | `i2r_rlc_localization_laser_line_observation_tests` | Passed，exit 0 |

## T011 变更范围与集成审计

- 当前工作区没有 `.git` 元数据，`git rev-parse` 返回“not a git repository”；因此无法可靠
  识别当前分支或对比提交级 diff，实际集成目标仍须在有 Git 元数据的协作工作区按
  `docs/collaboration_git_workflow.md` 确认，不能伪称已经合入 `integrate`。
- 以 feature 日期 2026-08-31 为界进行路径级只读审计：`IBVS/`、`Vision/`、
  `Localization/`、`Control/` 最近修改文件数均为 0；`oldproject/`、`RobotSim/` 在当前
  工作区不存在，因此本特性对两者的写入数为 0。
- 生产改动限定在 `UI/` 的 runtime/worker/门禁/页面状态，以及 T003 的 DigitalTwin
  observation 转换修复；测试和文档位于 `tests/`、`docs/`、`specs/`。没有修改 Vision、
  Localization、IBVS 算法或 Control 命令规则。
- `i2r_rlc_runtime_tracking_flow_tests` 的全局定位 prior、段号和捕获时 RobotState 断言通过；
  correction 通知仍与 `globalLocateFrameVirtualPriorReady` 专属通知分离，未发现全局定位
  接口、信号或数据流语义变化。

## T012 成功标准与剩余现场项

| 标准 | 状态 | 证据 |
|---|---|---|
| SC-001 | Passed | 在线/离线 `frame mismatch=0`。 |
| SC-002 | Passed | 两专项 `max current=1, max pending=1`，100 后只处理 105。 |
| SC-003 | Passed | 两专项 `post-stop query=0, old-session route=0`。 |
| SC-004 | Passed | 空/无效/transport/错帧均失败，`stale observation reuse=0`，具体错误进入 UI update。 |
| SC-005 | Passed | 1/2 次失败后成功清零；第 3 次失败完整停止且无 O/P。 |
| SC-006 | Passed | UI 只按三键确认；失败、替换、停止和重启清 pending，错配为 0。 |
| SC-007 | Passed | 离线专项先通过，随后在线专项通过。 |
| SC-008 | Passed | runtime/global-localization 保护回归通过，专属 prior/段号行为保持。 |
| SC-009 | Passed | IBVS/Vision/Localization/Control 受保护实现路径修改数为 0；既有控制器测试通过。 |
| SC-010 | Passed（自动化） | 在线/离线页面状态、最近消费 frameId 和最近错误的合成更新测试通过。 |

### Passed / Failed / Skipped 汇总

- Passed：VS Qt Debug 全部生成、九个规定自动目标、在线/离线成功/背压/失败/停止/重启/
  两类迟到返回/失败恢复/三连停止、模式门禁和 UI 三键确认。
- Failed：最终自动验收无失败项。
- Skipped：真实现场 RobotSim 进程、真实机器人/相机联调和真实运动命令；当前环境只使用
  注入的脚本化 transport。legacy 源码核对继续按 T001 明确标记 skipped，因为
  `oldproject/` 不存在。上述 skipped 项均未描述为已通过。
