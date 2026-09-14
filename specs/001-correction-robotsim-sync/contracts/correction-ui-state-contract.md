# Contract: Correction RobotSim UI State

## Channel isolation

在线和离线共用 `Correction Sync Update` 语义；该通道不得复用或改写：

- 全局定位逐帧 prior 通知；
- 普通 RobotSim 实时展示通知；
- 手动仿真场景轮询通知。

## Event schema

| Event state | Required keys | Observation | UI effect |
|---|---|---|---|
| Idle | purpose=None | none | 无活动纠偏同步。 |
| Querying | sessionId, purpose, frameId | none | 显示当前帧正在查询。 |
| ObservationReady | sessionId, purpose, frameId | required, valid, same frameId | 保存为唯一 pending；显示等待 Vision 消费。 |
| VisionConsumed | sessionId, purpose, frameId | same value or correlation | 仅匹配 pending 时更新最近确认 frameId 并清除 pending。 |
| Failed | sessionId, purpose, frameId, message, failure count | none | 清除该帧 pending；显示具体错误。 |
| Cleared | sessionId, purpose, optional frameId | none | 停止、暂停或更新替换时清除未确认值。 |
| StoppedAfterFailures | sessionId, purpose, message, count=3 | none | 清除 pending，显示终止原因和已停止状态。 |

## Matching rules

UI 只允许以下全等关系确认消费：

```text
pending.sessionId == consumed.sessionId
pending.purpose == consumed.purpose
pending.frameId == consumed.frameId
```

任一字段不匹配：

- 不更新最近确认 frameId；
- 不把 pending 标记为成功；
- 不清除属于更新会话的 pending；
- 可记录诊断，但不能显示为 Vision 已消费。

## Replacement and clearing

- 新 ObservationReady 到达前必须清除不属于同一 `(sessionId, frameId)` 的旧 pending。
- 当前帧 Failed 时清除对应 pending。
- Vision tracking 暂停、用户停止、controller 完成/错误或 session restart 时清除所有 pending。
- 普通实时/手动/全局定位 observation 不能写入纠偏 pending。

## Minimum presentation

在线和离线页面各自至少显示：

1. 当前纠偏同步状态；
2. 最近一次被 Vision 确认消费的 frameId；
3. 最近一次逐帧查询错误；
4. 连续同步失败数（可与状态同行展示，用于解释第 3 次停止）。

阶段一不要求新增完整点数面板或重构图像叠加。在线/离线既有业务状态文本继续由原控制器更新，纠偏同步文本使用独立小型状态区域，避免互相覆盖。

## User-visible messages

- 手动模式启动拒绝：必须包含“正式在线/离线纠偏需要 RobotSim 真实状态同步模式”。
- 无 RobotState：必须明确是当前帧缺少有效机器人状态，不能显示查询成功。
- RobotSim 错误：显示 DigitalTwin 原始具体原因；错帧追加 expected/actual frameId。
- 三连失败停止：必须包含最近故障原因、连续失败数 3 和任务已通过完整停止流程终止。

