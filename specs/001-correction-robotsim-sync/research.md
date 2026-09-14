# Phase 0 Research: 在线/离线纠偏逐帧 RobotSim 同步

## 1. 变化边界归属

**Decision**: 将逐帧冻结、RobotSim 查询路由、会话隔离、背压和同步失败计数放在现有 `CRuntimeCoordinator` 运行时边界；继续由现有 DigitalTwin worker 执行阻塞 RobotSim 通信，由现有 Vision worker 消费同帧输入。

**Rationale**: 当前协调器已经在相机帧到达时按值保存 `CameraFrame` 和最新 `RobotState`，并已实现“一个当前帧 + 一个可覆盖最新待处理帧”。全局定位也已经通过同一 DigitalTwin worker 按帧查询 RobotSim。缺口是纠偏目的没有进入该分支，而不是缺少新的顶层模块或算法服务。

**Alternatives considered**:

- 新建顶层同步模块：会重复 UI 运行时编排和 DigitalTwin 线程所有权，违反现有八模块边界。
- 把同步逻辑放入 Vision 或 Control：会让算法/业务状态机拥有 RobotSim 生命周期或相机队列，不符合现有职责。
- 由 UI 缓存最近 observation 后提交：无法保证冻结 RobotState、frameId 和 observation 属于同一帧，明确被规格禁止。

## 2. 冻结时点与有效 RobotState

**Decision**: 冻结点保持在 `enqueueVisionFrame()` 接收相机帧的运行时协调边界；冻结组合至少包含帧、`m_hasLatestRobotState`、RobotState 值、跟踪目的、纠偏会话 ID 和现有流程上下文。`m_hasLatestRobotState == false` 时当前帧直接同步失败，不调用 RobotSim。

**Rationale**: 这是现有代码最早同时可见相机帧和最新 RobotState 的边界，也与规格明确的“非曝光时刻硬同步”一致。RobotState 类型没有独立 `valid` 字段，现有 `m_hasLatestRobotState` 才是是否收到有效状态的事实来源。

**Alternatives considered**:

- 在 RobotSim 派发时重新读取最新状态：会改变已冻结帧的姿态。
- 以零初始化 RobotState 查询：会把缺失状态伪装为有效姿态。
- 修改 Camera/RobotComm 增加硬件时间戳：超出阶段一范围，且当前没有共同时间戳契约。

## 3. 纠偏会话与迟到结果关联

**Decision**: 每次正式在线或离线纠偏启动生成单调递增、非零的 `correctionSessionId`。会话独立于 Vision 的临时启停，在线多阶段暂停/恢复仍属于同一纠偏会话。纠偏帧提交给 Vision 时附带 UI 运行时关联元数据；`RuntimeTrackingResult` 回传同一 sessionId、purpose 和 frameId。RobotSim 结果同时校验单调 requestId、当前会话和 frameId。

**Rationale**: 当前 RobotSim requestId 能拒绝多数旧查询，但 Vision 结果只有 frameId；停止后立即重启时，旧 Vision 结果可能清除或完成新会话的在途状态。在线状态机还会在发送 O、等待回包和分段间暂停 Vision，因此不能把每次 `startTrackingRequested` 当作新纠偏会话。

**Alternatives considered**:

- 只校验 frameId：相机重启或测试输入可能复用 frameId，不能证明跨会话隔离。
- 修改 DigitalTwin observation 增加 sessionId：sessionId 属于调用方运行会话，不属于 RobotSim 虚拟参考领域数据。
- 修改 Vision 算法数据结构：没有必要；关联元数据可由 UI worker adapter 在算法调用外层携带。

## 4. 有界背压和当前帧生命周期

**Decision**: 当前帧生命周期从取出待处理帧开始，覆盖 RobotSim 查询和 Vision 处理，直到匹配的 Vision 结果或明确同步失败。期间新帧只覆盖唯一待处理槽；当前帧完成后只派发最后到达的待处理帧。

**Rationale**: 这直接满足帧 100 完成后只处理 105 的验收语义，也避免 RobotSim 查询完成后在 Vision 线程继续堆积多帧。

**Alternatives considered**:

- RobotSim 查询完成即释放在途槽：Vision 较慢时仍会形成无界 Vision 队列。
- 固定长度大于一的 FIFO：会处理已经过时的中间姿态，且不符合“最新待处理帧”契约。
- 丢弃当前帧并抢占：外部查询和 Vision 算法不能安全取消，且规格要求当前会话中的帧 100 完成。

## 5. RobotSim 查询与普通实时展示互斥

**Decision**: 复用现有 `generateRobotSimVirtualPrior()` 逐帧查询能力；纠偏会话活动期间继续缓存最新 RobotState，但不创建或续派普通实时展示同步请求，并且普通实时结果不得进入纠偏专用 UI 通道。会话结束后用最新 RobotState 恢复普通实时同步。

**Rationale**: DigitalTwin worker 串行拥有 RobotSim 服务，复用现有请求可保持协议、单位、frameId 和错误语义。暂停普通派发可避免它与正式逐帧查询争用同一连接，又不丢失恢复时所需的最新机器人状态。

**Alternatives considered**:

- 新建第二个 RobotSim socket：会让同一场景被两条连接并发驱动，并扩大生命周期范围。
- 纠偏继续使用普通实时缓存：无法证明缓存姿态与相机帧一致。
- 纠偏期间停止接收 RobotState：会导致恢复后使用旧状态，也会影响现有控制状态机。

## 6. observation 验证与错误保留

**Decision**: 协调器在调用 Vision 前验证 observation 有效、frameId 等于冻结帧且结果属于当前 request/session。任一失败均保留 DigitalTwin 原始 message；frameId 错配在原始消息后追加 expected/actual 诊断。失败帧不调用 Vision、不读取旧 observation。

**Rationale**: `DigitalTwinVisionLaserObservation` 已按值保留 frameId、有效性和 message，Vision 的同步入口也会拒绝错帧，但其当前通用错误会收敛为 `simulated observation unavailable`。在协调器先验证可同时满足“不进入 Vision”和“用户看到具体原因”。

**Alternatives considered**:

- 仅依赖 Vision 内部拒绝：原始错误和错帧细节会丢失，UI 也无法区分 RobotSim 同步故障与视觉业务结果。
- 失败时复用最新有效 observation：会把其他姿态数据冒充当前帧成功。
- 在 DigitalTwin 服务内校验纠偏会话：该服务不知道 UI 运行会话，职责不匹配。

## 7. 连续失败与完整停止路径

**Decision**: 协调器只对同步链故障递增连续计数；有效同帧 observation 在提交 Vision 前清零计数。第 3 次连续故障先失效会话和帧状态，再通过现有 Vision/RobotComm 停止路由以及 online/offline worker 的安全重置/失败入口终止任务，最后发布终止状态并恢复普通实时同步。

**Rationale**: 先失效会话可保证随后到达的 RobotSim/Vision 结果不能产生新业务事件。在线 worker 可复用现有 `reset()` 清除控制器命令状态，离线 worker 可复用现有 `fail()` 清除待发送命令；无需修改 Control 算法或命令触发规则。Vision 正常返回“未检测到目标”发生在同步成功之后，因此不进入同步失败计数。

**Alternatives considered**:

- 只停止 Vision：Control 仍可能保留活动状态或待发命令，形成半运行状态。
- 在第 1 次失败立即停止：与固定阈值 3 和允许恢复的要求冲突。
- 把“未检测到目标”计为同步失败：混淆外部同步故障和视觉业务结果。

## 8. 纠偏专用 UI 通知

**Decision**: 在 UI 运行时类型中定义一个纠偏专用同步更新 DTO，通过单一信号同时服务在线和离线。事件至少区分 querying、observation-ready、vision-consumed、failed、cleared/stopped，携带 purpose、sessionId、frameId、可选 observation、连续失败数和 message。UiController 只在 sessionId 与 frameId 同时匹配时确认消费，并在失败/停止/重启时清除未确认值。

**Rationale**: 现有全局定位信号是专属产品通道，不能改语义或复用为纠偏通知。单个结构化 DTO 能把跨线程状态、关联键、错误和清理语义保持为一个契约，避免多个松散信号在 UI 侧重新拼接。

**Alternatives considered**:

- 复用全局定位 `globalLocateFrameVirtualPriorReady`：会改变专属接口语义并产生跨工作流耦合。
- 只写日志：无法可靠展示最近确认 frameId，也不能验证 pending/consumed 状态。
- 分别为在线和离线定义两套信号：会重复完全相同的同步契约。

## 9. 正式模式启动门禁

**Decision**: 在 UiController 发送在线/离线 JBI 之前检查真实场景连接状态；手动仿真、连接中或未连接均拒绝正式启动并显示“切换到真实状态同步模式”的明确说明。在线一键流程在第一段全局定位启动前检查，并在第二段在线 JBI 前再次检查；独立全局定位行为不改变。

**Rationale**: 当前 UI 已分别维护真实场景连接、手动连接和 pending 状态，并且现有机器人启动门禁发生在发送 JBI 之前。在同一边界增加 RobotSim 模式前置条件可以避免先启动机器人程序再发现正式同步不可用。

**Alternatives considered**:

- 只在首帧失败：JBI 已发送，不符合“拒绝启动”。
- 修改全局定位连接门禁：规格明确禁止改变其专属行为。
- 把手动缓存当作降级路径：正式纠偏明确禁止。

## 10. 专项测试的可控制依赖

**Decision**: 沿用现有 `IRobotSimTransport` 可替换边界，为 DigitalTwin worker/RuntimeCoordinator 增加测试可注入的构造路径；生产默认仍创建真实 transport。脚本化 fake transport 控制阻塞、成功、通信失败和迟到返回。错 frameId 验证使用生产 observation 校验函数和 Vision 同步入口的直接测试，不修改 RobotSim 协议或测试全局定位实现。

**Rationale**: 传输层本来就是外部依赖变化点，现有 `RobotSimDigitalTwinService` 已支持注入 `IRobotSimTransport`。把该能力透传到现有 worker 是最小测试接缝，不需要新增通用 provider/factory。阻塞 fake 能验证真实线程排队、停止和重启顺序。

**Alternatives considered**:

- 启动真实 RobotSim 做自动测试：不可重复，无法稳定制造错帧和迟到结果。
- 新建第二套 fake DigitalTwin 服务接口：与现有 transport 变化边界重复。
- 只测试纯状态函数：不能证明跨线程迟到结果和查询计数。

## 11. Legacy 证据状态

**Decision**: 使用现有 `docs/online_offline_correction_start_gate_notes.md`、在线/离线状态机文档、当前实现和回归测试作为替代基线；实施第一项仍先尝试定位只读 legacy 源码，并将检索范围和缺失状态写入独立 docs 记录。用户已明确接受在源码不可用时撤销阻塞，但最终证据必须把 legacy 源码核对标记为 skipped。

**Rationale**: 当前 workspace 没有 `oldproject/`，不能诚实声称完成源码复核。已有文档、当前实现和回归测试足以形成可审计的替代基线；这不等同于 legacy 源码验证，因此保留 skipped 状态和缺口说明。

**Alternatives considered**:

- 把现有摘要当作已完成源码复核：违反证据完整性。
- 因只读目录缺失而停止全部规划：当前代码与已记录契约足以确定设计边界；真正实现仍保留前置门禁。

## 12. Agent context 更新脚本

**Decision**: 不手工改写根 `AGENTS.md`。计划阶段结束时检查并尝试仓库约定的 agent-context 脚本；若脚本仍不存在，在完成报告中记录工具链缺口。

**Rationale**: 当前 `.specify/scripts/powershell/` 只包含 setup/check/common 脚本，没有 `update-agent-context.ps1`。手工猜测脚本行为可能覆盖项目级规则，风险高于收益。

**Alternatives considered**:

- 手工追加技术栈到 `AGENTS.md`：该文件是共享工程治理入口，不应在缺少项目脚本时猜测性修改。

## Resolved Unknowns

- Language/target: C++17，Windows x64，Visual Studio 2022/MSVC，Qt Core/Gui/Widgets，CMake。
- External integration: 复用 DigitalTwin 的 cppzmq/RobotSim facade 和 `IRobotSimTransport` 测试接缝。
- Queue bound: 1 个当前帧 + 1 个可覆盖的最新待处理帧，当前帧覆盖查询与 Vision 全生命周期。
- Session scope: 一次正式纠偏任务，跨在线内部阶段暂停，不跨停止或重启。
- Failure threshold: 固定 3；只统计同步链故障，同帧成功清零。
- Algorithm scope: Vision、Localization、IBVS 算法和 Control 命令规则均不改。
- Remaining evidence limitation: legacy 源码仍不可用；VR-001 的精确缺口已经记录，源码核对在最终证据中保持 skipped，不再阻塞实现。
