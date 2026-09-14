# Contract: Correction Frame Synchronization

## Purpose

定义正式在线/离线纠偏在 RuntimeCoordinator、DigitalTwin worker 和 Vision worker 之间的逐帧同步契约。该契约不改变全局定位专属信号、Vision 算法输入含义或 Control 命令规则。

## Start contract

正式纠偏启动前必须同时满足：

1. RobotSim 已处于真实状态同步连接；
2. 不是手动仿真场景连接或连接切换中；
3. 现有机器人启动门禁通过；
4. 在线/离线既有配置前置条件通过。

不满足第 1 或 2 项时必须在发送 JBI 前拒绝，并明确要求切换到真实状态同步模式。独立全局定位不使用本纠偏门禁。

## Frame arrival contract

对活动纠偏会话中的每个相机帧：

1. 在 RuntimeCoordinator 收到帧时一次性按值冻结 frame、hasRobotState、RobotState、purpose、sessionId 和流程上下文。
2. 若 current 为空，将冻结上下文提升为 current；否则整体覆盖 latest pending。
3. pending 不得进入 Qt worker 队列，直到 current 完成。
4. 后续 RobotState 更新只更新全局 latest cache，不修改 current/pending 快照。

## RobotSim request contract

仅 current 可以发起逐帧查询。请求必须携带：

- 单调非零 requestId；
- current.frameId；
- current 冻结的 RobotState；
- 在线/离线自己的 purpose/context；
- correction sessionId 由协调器上下文保存，不写入 RobotSim 协议。

若没有有效 RobotState 或 RobotSim 不在真实连接状态，当前帧直接失败，不发送默认零状态请求。

## RobotSim result contract

结果进入 Vision 前必须同时满足：

- requestId 等于 current requestId；
- current.sessionId 等于活动纠偏 sessionId；
- current.purpose 等于活动纠偏 purpose；
- observation.valid 为 true；
- observation.frameId 等于 current.frameId。

任一条件失败：

- 不调用 Vision；
- 不读取任何缓存 observation；
- 保留具体错误；
- 更新同步失败计数；
- request/session 不匹配的纯迟到结果静默丢弃，不得修改新会话计数或状态。

## Vision submission contract

通过验证后，RuntimeCoordinator 调用现有同帧 Vision 入口并附加 UI 运行时关联元数据：

```text
frame
frozen RobotState
localizationSegmentIndex = -1 for correction
same observation
correctionSessionId
trackingPurpose
```

同步成功在 Vision 业务处理前将连续同步失败计数清零。Vision 是否检测到目标不改变同步成功事实。

## Vision result contract

Vision 结果只有在 sessionId、purpose、frameId 与 current 全部相等时才可路由：

- 发布 VisionConsumed UI 更新；
- 把现有检测结果送入对应 online/offline worker；
- 清空 current；
- 派发 latest pending。

不匹配结果不得释放 current、覆盖 UI、进入 Control 或生成命令。

## Backpressure contract

任意时刻：

```text
current <= 1
latest pending <= 1
queued per-frame RobotSim requests outside current = 0
queued Vision frames outside current = 0
```

帧 100 current 时到达 101..105，100 完成后只允许 105 成为下一 current。

## Stop and restart contract

停止顺序必须是：

1. 先失效活动 sessionId，并禁止新帧和新逐帧查询；
2. 清除 pending、current、requestId 和未确认 UI observation；
3. 停止 Vision tracking 和 RobotComm tracking 状态；
4. 安全重置/失败对应纠偏 worker，清除待生成命令；
5. 以最新 RobotState 恢复普通实时 RobotSim 查询；
6. 发布终止状态。

restart 必须生成新 sessionId。旧 RobotSim/Vision 结果即使 frameId 相同也不能命中新会话。

## Failure threshold contract

- NoRobotState、断开、查询/通信失败、无效/空结果、错 frameId：计数 +1。
- 有效同帧 observation：计数清零。
- Vision no-target 或业务拒绝：不计数。
- 第 3 次连续同步故障：执行完整停止，不产生新纠偏命令。

## Ordinary realtime coexistence

纠偏会话 active 时：

- RobotState latest cache 继续更新；
- 不创建或续派普通实时展示请求；
- 已存在普通请求的结果不能进入纠偏通道；
- correction current 使用独立 requestId/context。

会话结束后，用最新 RobotState 恢复 ordinary latest-wins 同步。

