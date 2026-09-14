# Quickstart Validation: 在线/离线纠偏逐帧 RobotSim 同步

## Goal

按“离线先通过、在线后通过”的顺序验证同帧绑定、背压、同步故障、三连失败停止、停止/重启隔离、普通实时同步恢复和 UI 确认。本文是验证指南，不包含实现代码。

## Prerequisites

- Windows x64、Visual Studio 2022、项目现有 Qt/CMake/Ninja Developer 环境。
- 使用 Visual Studio 当前已配置的 `qt-debug` 或对应 Qt Debug 配置；不要在普通 PowerShell 裸跑完整 MSVC/Ninja 构建。
- 自动专项测试使用脚本化 RobotSim transport，不要求启动真实 RobotSim、真实相机或真实机器人。
- 现场联调前需连接 RobotSim 真实场景，并确认 UI 显示真实状态同步连接成功。
- 实施开始前已完成 `docs/correction_robot_sim_frame_sync_legacy_notes.md` 中的 legacy 证据记录；只读源码不可用时使用已接受的替代基线继续，并在最终结果中将源码核对标记为 skipped，不得宣称源码验证通过。

## Artifact references

- 行为规格：[spec.md](spec.md)
- 数据与状态：[data-model.md](data-model.md)
- 帧同步契约：[contracts/correction-frame-sync-contract.md](contracts/correction-frame-sync-contract.md)
- UI 状态契约：[contracts/correction-ui-state-contract.md](contracts/correction-ui-state-contract.md)

## Build in Visual Studio 2022

1. 打开 `qt-sontech - Microsoft Visual Studio`。
2. 核对工具栏当前 CMake 配置是预期 Qt Debug 配置。
3. 使用 `F7` 触发全部生成。
4. 在输出窗口确认最终状态为“全部生成 已成功”；若超过约 2 分钟没有新输出，停止等待并按仓库 `AGENTS.md` 检查残留 cmake/ninja/cl/link 与 `.ninja_lock`。
5. 不因本特性启用 ONNX Runtime 或创建新的完整 CLI build tree。

## Automated validation order

在 Visual Studio 的测试入口按以下顺序运行。新增专项测试的 CTest 依赖必须保证 online 测试依赖 offline 测试成功，不能反序执行。

### 1. Service contract protection

运行：

- `i2r_rlc_digital_twin_tests`

Expected:

- success 和 failure 均保留请求 frameId；
- observation 转换保留具体 RobotSim 错误；
- invalid/empty result 不变成 valid；
- 现有调用方断言保持通过。

### 2. Offline correction frame-sync suite (must pass first)

运行：

- `i2r_rlc_correction_robot_sim_offline_frame_sync_tests`

Required cases:

1. valid state + valid observation：Vision 输入 frameId 与相机帧相同；UI 先 pending 后 consumed；
2. no RobotState：查询计数不增加，错误明确，不使用零状态；
3. blocked frame 100 + incoming 101..105：在途=1、pending=1，释放后只处理 105；
4. query failure、empty/invalid observation、wrong frameId：当前帧失败，上一帧 observation 复用次数为 0；
5. one/two failures then success：流程继续且 failure count 回到 0；
6. three consecutive failures：离线 worker 进入安全错误/停止状态，不生成 O 命令；
7. stop while query/Vision is blocked：停止后新查询数为 0，pending 清空；
8. stop then restart：旧 session 的 RobotSim/Vision 结果均被丢弃，不清除新 current；
9. ordinary realtime pending before start：纠偏期间不续派，停止后用 latest RobotState 恢复。

Offline suite 必须输出独立计数证据：最大 current、最大 pending、停止后查询数、旧 session 路由数、frame mismatch 数、stale observation reuse 数。

### 3. Online correction frame-sync suite (only after offline passes)

运行：

- `i2r_rlc_correction_robot_sim_online_frame_sync_tests`

重复 offline 的成功、背压、错误、停止、重启和迟到返回场景，并额外验证：

- 在线内部 Vision 暂停/恢复不改变 correctionSessionId；
- RobotSim/Vision/UI consumed 使用同一 sessionId 和 frameId；
- no-target Vision 结果不增加同步失败计数；
- 第 3 次同步故障停止后不产生新的 P 命令；
- 上一在线会话的迟到结果不能触发新会话 Control 事件。

### 4. Shared and no-regression tests

依次运行：

- `i2r_rlc_runtime_tracking_flow_tests`
- `i2r_rlc_correction_robot_start_gate_tests`
- `i2r_rlc_main_widget_panel_tests`
- `i2r_rlc_online_correction_tests`
- `i2r_rlc_offline_correction_tests`
- `i2r_rlc_localization_laser_line_observation_tests`

Expected:

- 全局定位仍只走其专属 prior 通知和段号语义；
- 在线/离线既有状态机与命令触发测试无需修改预期；
- 手动/真实 RobotSim 连接 UI 既有行为保持；
- UI 新增的纠偏同步区能显示 status、last consumed frameId 和 last error。

## Automated acceptance metrics

| Metric | Required result |
|---|---:|
| Valid Vision input frame mismatches | 0 |
| Max current correction frames | 1 |
| Max pending correction frames | 1 |
| Per-frame queries started after stop | 0 |
| Old-session RobotSim results routed to new session | 0 |
| Old-session Vision results routed to new session | 0 |
| Previous valid observation reused after failure | 0 |
| UI consumed session/frame mismatches | 0 |
| Three-failure stop success rate | 100% |
| Offline required cases before online | 100% pass |
| Online required cases | 100% pass |

## UI/manual validation without real motion commands

以下步骤只验证启动门禁和显示，不要求触发真实纠偏运动命令：

1. 连接“手动仿真场景”。尝试启动离线和在线纠偏。
   - Expected: 两者均在发送 JBI 前被拒绝；提示明确要求真实状态同步模式。
2. 连接“真实场景”，但不发布有效 RobotState；通过专项测试入口提交一帧。
   - Expected: 当前帧显示缺少有效 RobotState，不能显示成功或使用零状态。
3. 在受控 fake/测试运行中让 observation 到达但暂缓 Vision 结果。
   - Expected: UI 显示 pending，不更新 last consumed frameId。
4. 返回同 session/frame Vision 结果。
   - Expected: UI 标记 consumed，并更新 last consumed frameId。
5. 返回 wrong session 或 wrong frame Vision 结果。
   - Expected: UI 不确认，不清除属于当前新会话的 pending。
6. 停止或重启。
   - Expected: pending 清空，旧结果不可见，普通实时 RobotSim 显示随后恢复。

## Real RobotSim integration check

自动测试全部通过后再做现场查询验证：

1. 连接 RobotSim 真实场景并确认端口、robotId、positionerId、sensorId 正确。
2. 使用不触发真实纠偏命令的受控检测阶段提交少量帧。
3. 对每帧记录 camera frameId、query frameId、Vision correlation 和 UI consumed frameId。
4. 确认四者完全一致，且 RobotSim 断开时错误原因原样可见。
5. 停止检测，确认查询计数不再增加；重新启动后确认旧结果未进入新 session。

## Change-scope inspection

实施验证结束前检查变更列表：

- `IBVS/` 文件变更数必须为 0；
- `Vision/` 与 `Localization/` 算法文件变更数必须为 0；
- 全局定位专属接口、信号和实现不应发生语义修改；
- `Control/` 算法和命令规则不应修改；纠偏同步停止适配留在 UI runtime worker 边界；
- `oldproject/` 和 `RobotSim/` 文件变更数必须为 0。

## Evidence report

验证报告分别记录：

- build configuration 和 Visual Studio 最终状态；
- offline suite 逐场景结果和指标；
- online suite 逐场景结果和指标；
- shared/global-localization regression；
- skipped 项及缺少的设备、场景或 legacy 证据；
- 仍存风险和现场限制。
