# 在线/离线纠偏逐帧 RobotSim 同步 Legacy 核对记录

## 核对状态

- 核对日期：2026-08-31。
- 任务：`specs/001-correction-robotsim-sync/tasks.md` 中的 T001。
- 状态：**替代证据已接受 / Legacy 源码核对 skipped**。
- 证据限制：当前实施环境未提供只读 legacy 工程，无法从源码核对在线/离线纠偏的启动、停止、错误处理和命令抑制行为。
- 处置决定：用户于 2026-08-31 明确撤销源码缺失硬门禁，同意以现有迁移文档、当前实现和回归测试作为替代基线继续实施。

本记录说明已确认的证据缺口和替代基线决定。T001 的“证据记录”部分据此完成，但 legacy 源码核对仍为 skipped；既有摘要文档不会被表述为本次源码核对结果。

## 预期证据位置

T001 指定的只读证据根目录及其规则文件应为：

```text
oldproject/sontech-works-version-storage/
oldproject/sontech-works-version-storage/AGENTS.md
```

根据当前规格和既有迁移文档，后续源码核对至少应覆盖 `CRobotleadingDlg` / `robotleadingDlg` 相关实现，并记录下列行为对应的真实函数、文件和分支条件：

1. 在线纠偏启动以及 JBI 发送前的状态门禁；
2. 离线纠偏启动以及扫描检测进入条件；
3. 用户停止、流程完成和错误终止时的线程、状态与待发送命令清理；
4. RobotSim/视觉/机器人通信错误后的命令抑制，尤其是不得继续产生 O/P 类纠偏命令；
5. 在线内部暂停/恢复与完整会话停止之间的差异；
6. 离线检测结束、匹配计算和命令发送之间的先后关系。

上述条目是待核对清单，不是已经由本次 legacy 源码确认的事实。

## 本次只读检查证据

在仓库根目录
`D:\Users\Administrator\Desktop\WorkStation\YangCai_qt-sontech1-RS-complete`
执行了以下只读检查：

- 枚举仓库根目录：未发现 `oldproject/` 和 `RobotSim/` 目录；
- 检查精确路径 `oldproject/sontech-works-version-storage/AGENTS.md`：文件不存在；
- 在 `oldproject/` 下查找 `AGENTS.md`：由于 `oldproject/` 不存在而无法检索；
- 枚举 `D:\Users\Administrator\Desktop\WorkStation` 的相邻工作区，并检查每个工作区下的
  `oldproject/sontech-works-version-storage/AGENTS.md` 和
  `sontech-works-version-storage/AGENTS.md`：均不存在；
- 在整个 `D:\Users\Administrator\Desktop\WorkStation` 下按目录名查找
  `sontech-works-version-storage`：未找到匹配目录；
- 在同一范围按文件名查找 `RobotleadingDlg` / `robotleadingDlg`：只找到当前及相邻工作区中的
  `docs/robotleadingdlg_dependency_map.md`，未找到 `.h`、`.cpp` 或其他 legacy 源文件。

规格材料也明确记录了同一缺口：

- `specs/001-correction-robotsim-sync/spec.md` 的 VR-001 要求在源码不可用时明确记录缺口，并将源码核对标记为 skipped；
- `specs/001-correction-robotsim-sync/research.md` 的“Legacy 证据状态”指出当前 workspace 没有
  `oldproject/`，既有摘要不能替代实施前源码证据；
- `specs/001-correction-robotsim-sync/plan.md` 接受迁移文档、当前实现和回归测试组成的替代基线；
- `specs/001-correction-robotsim-sync/tasks.md` 要求 T001 先记录证据缺口，但不再因源码缺失阻止后续实施。

## 既有摘要的使用边界

仓库内存在 `docs/online_offline_correction_start_gate_notes.md`、
`docs/online_correction_state_machine.md`、`docs/offline_correction_state_machine.md` 等迁移记录。
这些文件可以在 legacy 源码恢复后用于确定检索入口和比对保持项，但当前只能作为二手资料，不能替代 T001 要求的函数/文件源码证据。

因此，本次未把二手文档中的函数名、控制条件或命令语义重新表述为“已核对”的 legacy 结论。

## 对 T002-T004 的影响

依据用户明确决定和修订后的 VR-001，当前状态调整为：

- T001 可按“证据缺口已记录、源码核对 skipped”勾选；
- T002 可使用现有迁移文档、当前实现和变更前回归结果建立替代基线；
- T003 可在不改变 DigitalTwin 生产公共接口的前提下实现共享 DTO、Qt metatype 和 transport 测试注入；
- T004 可在现有回归保护下实现 correction session、有界帧槽、请求关联、普通实时暂停/恢复和统一清理。

后续实现仍必须通过当前代码审查、保护性测试和错误后命令抑制验证约束风险，且不得把替代证据描述为 legacy 源码证明。

## 解除阻塞所需输入

请将只读 legacy 工程恢复到任务指定位置：

```text
oldproject/sontech-works-version-storage/
```

其中必须包含该目录适用的 `AGENTS.md` 以及在线/离线纠偏相关源码。恢复后应先完整读取更近层级规则，再按“预期证据位置”中的核对清单补充真实函数名、文件路径、关键条件、错误分支和必须保持的命令抑制语义。该补充用于把 legacy 源码核对从 skipped 更新为 verified，不再作为 T002-T004 的实施前置条件。
