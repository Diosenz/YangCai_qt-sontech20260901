# Feature Specification: 在线/离线纠偏逐帧 RobotSim 同步

**Feature Branch**: `未创建（当前工作区无 Git 元数据）`
**Created**: 2026-08-31
**Status**: Draft
**Input**: User description: "为在线和离线纠偏补齐逐帧 RobotSim 虚拟参考绑定、运行控制、错误处理和可验证 UI 状态，先完成离线，再完成在线。"

## Engineering Objective *(mandatory)*

**Change Point**: 正式在线和离线纠偏由“可能使用最近缓存的 RobotSim observation”变为“每个被处理帧使用运行时边界冻结的机器人状态查询并消费同帧 observation”，并具有有界背压、会话隔离、失败停止和同帧 UI 确认语义。
**Owning Module/Area**: 运行时纠偏编排为主责；DigitalTwin 提供既有 RobotSim 虚拟参考能力；UI 展示纠偏同步状态。
**Shared Impact**: 纠偏运行时状态、DigitalTwin 公共查询结果、UI 状态、RobotComm 提供的最新 RobotState、共享测试与文档。Vision 只使用既有输入契约，不改变算法实现。
**Behavior to Preserve**: 离线检测算法和命令生成规则、在线分段及纠偏命令触发条件、手动仿真场景的非严格同步展示能力、全局定位的专属接口/信号/数据流/运行行为、现有调用方对共享服务的行为，以及全部 IBVS 行为均保持不变。
**Intentional Behavior Changes**: 正式纠偏仅允许真实状态同步模式；正式纠偏逐帧输入禁止回退到缓存 observation；连续 3 帧同步链故障通过既有完整停止路径结束任务；纠偏运行时暂停普通 RobotSim 实时展示查询并在停止后恢复；UI 仅确认 Vision 实际消费的同会话同帧 observation。
**Out of Scope**: 全局定位的新功能或重构；Vision、Localization、IBVS 算法及 IBVS 任何内容的修改；RobotSim 内部场景、几何和传感器算法；二维/三维点数差异；焊缝点插入二维点列的适配；回放、fake transport、完整无设备仿真；RobotSim 自动启动/加载；真实机器人运动命令；完整模式生命周期或可视化面板重构。

## Scenarios & Acceptance *(mandatory)*

### Scenario 1 - 离线纠偏逐帧同步与安全停止 (Priority: P1)

操作人员在 RobotSim 真实状态同步模式下启动离线纠偏。每个被处理的相机帧在到达运行时协调边界时，冻结该帧、当时最新的有效机器人状态和离线流程上下文；系统以这份状态查询 RobotSim，并将相机帧、机器人状态和同帧 observation 作为不可拆分的一组交给 Vision。离线算法和命令规则保持不变。

**Why this priority**: 离线链路是本特性的首个交付和验证关口，可在不改变正式在线业务规则的前提下证明同帧绑定、背压、失败处理和停止语义。
**Independent Verification**: 使用可控制查询阻塞、成功、失败、错帧和迟到返回的离线专项测试，验证所有 Vision 输入的 frameId、一在途一待处理上限、会话隔离和完整停止结果。

**Acceptance Scenarios**:

1. **Given** RobotSim 已连接、模式为真实状态同步且已有有效 RobotState，**When** 离线纠偏收到相机帧，**Then** Vision 收到的相机帧、冻结 RobotState 和 observation 属于同一运行会话，且 observation frameId 与相机帧 frameId 相同。
2. **Given** 帧 100 正在当前会话中处理且帧 101 至 105 陆续到达，**When** 帧 100 尚未完成，**Then** 系统完成帧 100，期间最多保存一个可覆盖的待处理帧，并在其后只处理最新帧 105。
3. **Given** 离线纠偏正在运行，**When** 操作人员停止流程，**Then** 系统清除未派发帧、当前帧关联状态和未确认 UI observation，不再发起新的逐帧查询，且任何迟到结果均不得产生检测成功或纠偏命令。
4. **Given** 首帧到达但尚无有效 RobotState，**When** 系统处理该帧，**Then** 该帧以明确原因报告同步失败，不使用默认零状态、不查询为成功，也不使用旧 observation。

### Scenario 2 - 在线纠偏复用同一同步契约 (Priority: P2)

离线专项场景全部通过后，操作人员启动在线纠偏。在线帧遵循相同的冻结、逐帧查询、同帧消费、有界背压和会话隔离契约，同时保留既有在线分段、检测结果含义和纠偏命令触发条件。

**Why this priority**: 在线流程会产生实际纠偏业务输出，必须建立在已经验证的离线同步契约之上，并重点防止停止、重启和迟到结果污染新会话。
**Independent Verification**: 在离线专项通过后运行在线专项测试，覆盖成功、积压、同步失败、停止、立即重启以及 RobotSim/Vision 迟到结果，并核对没有旧会话结果或新增命令进入新会话。

**Acceptance Scenarios**:

1. **Given** 在线纠偏在真实状态同步模式下运行且有效相机帧和 RobotState 到达，**When** 当前帧完成同步并由 Vision 处理，**Then** RobotSim 查询所用状态、Vision 输入和 UI 确认均对应同一会话和同一 frameId。
2. **Given** 上一在线会话停止后立即启动新会话，**When** 上一会话的 RobotSim 或 Vision 结果迟到，**Then** 结果被丢弃，既不路由到新会话，也不清除、覆盖或完成新会话的当前状态。
3. **Given** 在线纠偏停止，**When** 停止流程完成，**Then** 当前帧、最新待处理帧、在途状态和未确认 UI observation 均被清理，不再发起逐帧查询，也不因清理或迟到结果生成纠偏命令。
4. **Given** 连续两帧发生同步链故障后下一帧同步成功，**When** 成功 observation 与当前帧匹配，**Then** 连续同步失败计数清零，流程可继续且 Vision 的正常“未检测到目标”结果不计为同步失败。

### Scenario 3 - 正式模式门禁与同帧 UI 确认 (Priority: P1)

正式在线和离线纠偏只能在 RobotSim 真实状态同步模式下运行。UI 可以暂存纠偏当前帧的 observation，但只有 Vision 返回同一运行会话、同一 frameId 的结果后，才能将其标记为“Vision 已消费”。纠偏专用通知与全局定位专属通知保持独立。

**Why this priority**: 模式门禁防止非严格缓存进入正式纠偏；同帧确认让操作人员能够验证界面数据确为 Vision 实际消费的数据，而不是其他姿态的旧缓存。
**Independent Verification**: 通过模式门禁、错会话、错 frameId、当前帧失败、停止和重启测试，核对 UI 的同步状态、最近确认 frameId、最近错误及未确认 observation 清理行为。

**Acceptance Scenarios**:

1. **Given** RobotSim 处于手动仿真场景模式，**When** 用户尝试启动正式在线或离线纠偏，**Then** 系统拒绝启动，并明确说明必须切换到真实状态同步模式。
2. **Given** UI 已暂存当前纠偏 observation，**When** Vision 返回不同会话或不同 frameId 的结果，**Then** UI 不将该 observation 标记为已消费，并清除或保留失败原因而不冒充成功。
3. **Given** UI 已暂存但尚未确认的 observation，**When** 当前帧失败、流程停止、流程重启或该 observation 被更新帧取代，**Then** 未确认 observation 被清除，且不得跨帧或跨会话复用。
4. **Given** 正式纠偏期间 RobotSim 连接中断，**When** 连续第 3 帧发生同步链故障，**Then** 第 3 帧明确失败，系统通过既有完整停止路径终止当前任务、清理纠偏状态并向 UI 报告具体原因，不产生新的纠偏命令。

### Edge Cases

- 首帧到达时没有有效 RobotState；状态缺失必须是明确同步失败，不能隐式生成默认状态。
- RobotSim 返回通信错误、空数据、无效数据或与请求不同的 frameId；当前帧失败且不得复用上一帧成功数据。
- RobotSim 或 Vision 慢于相机帧输入；容量始终限制为一个当前帧和一个可覆盖的最新待处理帧。
- 在 RobotSim 查询或 Vision 处理期间停止，并在停止后立即重启；旧会话结果不能影响新会话。
- 普通实时展示查询尚有待处理状态时启动纠偏；正式纠偏期间不得继续派发普通查询，停止后按最新机器人状态恢复普通查询。
- UI 已收到 observation 但 Vision 尚未返回时发生失败、停止或重启；该 observation 不能被确认或跨会话保留。
- Vision 完成但业务上未检测到目标；该结果不属于同步链故障，也不能触发连续同步失败停止。

## Requirements *(mandatory)*

### Functional and Contract Requirements

- **FR-001**: 系统 MUST 在相机帧到达运行时协调边界时，按值冻结相机帧、当时最新的 RobotState、当前纠偏流程上下文和当前运行会话标识。
- **FR-002**: 系统 MUST 将阶段一同步保证定义为“运行时边界的最新状态快照”，MUST NOT 将其描述为机器人状态与相机曝光时刻的硬同步。
- **FR-003**: 当前帧的 RobotSim 查询 MUST 始终使用 FR-001 冻结的同一份 RobotState；派发、返回或 Vision 消费阶段 MUST NOT 改用更新后的状态。
- **FR-004**: 正式在线和离线纠偏 MUST 仅在 RobotSim 真实状态同步模式下启动；手动仿真场景模式 MUST 被拒绝并显示明确切换说明。
- **FR-005**: 手动仿真场景现有的 UI 展示和显式非严格同步调试能力 MAY 保留，但 MUST NOT 成为正式纠偏逐帧输入或回退来源。
- **FR-006**: 正式在线和离线纠偏 MUST 为每个被处理帧查询逐帧 RobotSim observation，MUST NOT 使用 UI、算法或其他工作流缓存的最近 observation 替代当前帧结果。
- **FR-007**: 提交给 Vision 的有效 observation MUST 与相机帧属于同一运行会话，且两者 frameId MUST 相同；相机帧、冻结 RobotState、流程上下文和 observation MUST 作为不可拆分的当前帧输入处理。
- **FR-008**: 没有有效 RobotState 时，系统 MUST 让当前帧以具体原因同步失败，MUST NOT 使用默认零状态发起 RobotSim 查询，也 MUST NOT 将该帧表示为成功。
- **FR-009**: RobotSim 断开、查询失败、空或无效返回、明确通信错误及 frameId 错配 MUST 使当前帧明确失败，并保留可展示的原始具体原因；MUST NOT 复用上一帧有效 observation。
- **FR-010**: 系统 MUST 为每次正式纠偏启动建立不同的运行会话；停止或重启前产生的迟到 RobotSim 与 Vision 结果 MUST 被丢弃，且 MUST NOT 改变新会话状态。
- **FR-011**: 同一纠偏会话任一时刻 MUST 最多存在一个当前处理帧和一个可覆盖的最新待处理帧；新的待处理帧 MUST 替换旧待处理帧，且请求或图像积压 MUST NOT 随运行时间增长。
- **FR-012**: 当前帧仍属于活动会话时，系统 MUST 允许其完成；完成后如存在多个到达帧，只处理其中最新的待处理帧。
- **FR-013**: 停止在线或离线纠偏后，系统 MUST 清除尚未派发的帧，并 MUST NOT 再发起新的逐帧 RobotSim 查询。
- **FR-014**: 停止流程 MUST 清理当前帧关联状态、最新待处理帧、在途请求状态和 UI 未确认 observation；清理及迟到结果 MUST NOT 生成新的检测成功或纠偏命令。
- **FR-015**: 纠偏运行期间，系统 MUST 暂停派发普通 RobotSim 实时展示查询，同时继续接收并保存最新 RobotState；纠偏停止后 MUST 恢复普通实时查询。
- **FR-016**: 普通实时查询与纠偏逐帧查询切换时，旧的普通查询状态或结果 MUST NOT 被当作纠偏当前帧 observation，纠偏结果也 MUST NOT 被错误复用于其他会话。
- **FR-017**: 在线和离线纠偏 MUST 仅使用各自 RobotState 中的流程状态和各自既有纠偏上下文，MUST NOT 依赖全局定位维护的段号。
- **FR-018**: 连续同步失败计数 MUST 仅包含无有效 RobotState、RobotSim 断开、查询失败、无效或空返回、通信错误和 frameId 错配；Vision 正常执行后的“未检测到目标”等业务结果 MUST NOT 计入。
- **FR-019**: 任意一次有效且 frameId 匹配的逐帧同步成功 MUST 将连续同步失败计数清零。
- **FR-020**: 连续第 3 帧同步链故障 MUST 通过既有完整停止路径终止当前在线或离线纠偏任务、清理相关状态、向 UI 报告具体原因，并阻止新的纠偏命令；阈值 MUST 固定为 3，且本阶段不得新增用户配置入口。
- **FR-021**: 在线和离线纠偏 MUST 共用一致的纠偏逐帧 observation 通知语义，该通知 MUST 与全局定位专属通知相互独立。
- **FR-022**: UI MAY 暂存当前纠偏 observation，但只有收到同一运行会话且同一 frameId 的 Vision 结果后，MUST 才能将其标记为“Vision 已消费”并更新为最近成功帧。
- **FR-023**: 当前帧失败、流程停止、流程重启或未确认 observation 被更新帧取代时，UI MUST 清除未确认 observation；会话或 frameId 不匹配时 MUST NOT 标记为已消费。
- **FR-024**: 阶段一 UI MUST 至少展示当前纠偏同步状态、最近确认由 Vision 消费的 frameId 和最近一次逐帧查询错误。
- **FR-025**: 离线逐帧专项验收 MUST 全部通过后才能开始在线逐帧专项验收；只有两类流程的全部必需验收均通过，本特性才可视为完成。
- **FR-026**: 本特性 MUST 保持离线算法和命令生成规则、在线分段与命令触发条件、共享服务现有调用方行为、全局定位专属行为和全部 IBVS 行为兼容。
- **FR-027**: 如完成本特性必须改变 Vision、Localization 或 IBVS 的既有算法或公共契约，实施 MUST 暂停并单独提出缺陷、影响和替代方案评估，不得在本特性内隐式扩大范围。

### Evidence Requirements

- **VR-001**: 实施前 MUST 尝试查阅相关 legacy 在线/离线纠偏行为，并在 `docs/` 中记录对本特性数据绑定、停止、错误和命令语义有影响的发现；若只读 legacy 工程在实施环境中不可用，MUST 记录准确检索范围和证据缺口，并 MAY 在用户明确接受后以现有迁移文档、当前实现和回归测试作为替代基线继续实施。最终证据 MUST 将 legacy 源码核对标记为 skipped，MUST NOT 伪称已经完成源码验证。
- **VR-002**: MUST 建立在线/离线纠偏逐帧 RobotSim 专项测试，覆盖阻塞、成功、查询失败、无效/空返回、错 frameId、停止、立即重启及 RobotSim/Vision 迟到返回。
- **VR-003**: 相关共享服务测试 MUST 验证 frameId 和具体错误信息得到保留，并验证现有调用方行为保持兼容。
- **VR-004**: MUST 运行现有全局定位回归测试作为共享基础设施保护，不得修改这些测试来适应纠偏实现。
- **VR-005**: 验证证据 MUST 分别记录离线和在线的成功、背压、失败、停止、重启、迟到结果以及连续失败恢复/停止结果；在线证据 MUST 位于离线专项全部通过之后。
- **VR-006**: 变更审查 MUST 证明全局定位专属实现、Vision/Localization/IBVS 算法和 IBVS 文件均未被修改。

## Key Entities

- **纠偏运行会话**: 一次正式在线或离线纠偏从成功启动到完整停止的隔离边界；用于拒绝停止或重启前的迟到结果。
- **冻结帧组**: 相机帧、该帧到达运行时边界时的 RobotState 快照、该纠偏流程上下文、运行会话和 frameId 的不可拆分组合。
- **逐帧 observation**: 由冻结 RobotState 查询得到的 RobotSim 虚拟参考；包含有效性、frameId、二维/三维虚拟激光线、虚拟焊缝点和具体错误结果。
- **当前帧与最新待处理帧**: 有界背压状态；当前帧最多一个，待处理帧最多一个且可被更晚帧覆盖。
- **同步失败状态**: 最近错误和连续同步失败计数；只统计同步链故障，成功同步清零，第 3 次连续故障触发完整停止。
- **UI observation 确认状态**: 当前未确认 observation、最近确认消费的 frameId、当前同步状态和最近逐帧查询错误；确认需要会话与 frameId 同时匹配。

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 离线和在线专项样本中，提交给 Vision 的有效 observation 与对应相机帧 frameId 错配数均为 0。
- **SC-002**: 在持续高输入速率测试的整个运行期，系统观测到的容量上限始终不超过 1 个当前处理帧加 1 个最新待处理帧，积压数量不随测试时长增长。
- **SC-003**: 每种纠偏流程完成停止后，新增逐帧 RobotSim 查询数为 0；停止或重启前迟到结果进入新会话的数量为 0。
- **SC-004**: RobotSim 失败、无效返回或错帧测试中，复用上一帧有效 observation 的次数为 0，且每次失败均可由操作人员看到具体原因。
- **SC-005**: 所有连续 3 帧同步故障样本均通过完整停止路径终止当前任务；所有单次或连续两次故障后成功的样本均继续处理并将失败计数清零。
- **SC-006**: UI 标记为“Vision 已消费”的 observation 与对应 Vision 结果在运行会话和 frameId 上的错配数为 0；失败、停止、重启后跨帧或跨会话保留的未确认 observation 数为 0。
- **SC-007**: 离线专项的成功、背压、失败、停止、重启和迟到结果场景 100% 通过后，在线同类专项场景 100% 通过。
- **SC-008**: 现有全局定位保护性回归测试保持 100% 通过，且审查确认其产品行为、专属接口与实现逻辑没有改变。
- **SC-009**: 审查确认 IBVS 文件变更数为 0，Vision 与 Localization 算法实现变更数为 0；离线和在线既有算法结果含义及命令触发条件保持不变。
- **SC-010**: 在每个验收流程中，操作人员无需查看内部日志即可从 UI 确认当前同步状态、最近确认消费的 frameId 和最近一次逐帧查询错误。

## Assumptions and Dependencies

- RobotSim 通信、真实状态同步、虚拟参考解析以及模式识别的现有基础能力可继续复用。
- Camera 和 RobotComm 能分别提供相机帧与最新 RobotState，但当前不存在共同硬件时间戳契约；因此本阶段只保证运行时边界快照。
- Vision 已存在消费同帧相机图像、RobotState 和 RobotSim observation 的稳定入口，且无需改变算法实现。
- 在线纠偏、离线纠偏与全局定位不会同时运行；如现有运行门禁不保证该假设，计划阶段必须暴露风险而不能修改全局定位专属行为。
- 现有完整停止路径能够安全终止对应纠偏任务并阻止后续命令；本特性复用该产品行为。
- RobotState、observation 和 Vision 结果均能携带或关联足以验证运行会话与 frameId 的信息；若现有公共契约不能表达该信息，必须按 FR-027 单独评估影响。
- 当前工作区未提供只读 `oldproject/` 目录；本规格仅将 legacy 核对定义为实施前证据要求，不把现有文档摘要冒充源码核对结果。
