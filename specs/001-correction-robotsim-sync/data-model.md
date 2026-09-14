# Data Model: 在线/离线纠偏逐帧 RobotSim 同步

## 1. Correction Run Session（纠偏运行会话）

一次正式在线或离线纠偏从被运行时接受启动，到完成、失败、用户停止或重置为止的隔离边界。在线内部的 Vision 暂停、发送示教点、等待机器人回包和下一分段检测不会创建新会话。

### Fields

| Field | Type / Domain | Rules |
|---|---|---|
| `sessionId` | 非零单调整数 | 每次正式启动生成新值；0 表示无纠偏会话；停止后不得复用旧值。 |
| `purpose` | `OnlineCorrection` / `OfflineCorrection` | 会话建立后不可改变；全局定位不属于纠偏会话。 |
| `active` | boolean | 只有 active 会话可以接收新纠偏帧和路由结果。 |
| `consecutiveSyncFailures` | 0..3 | 同帧有效 observation 清零；只统计同步链故障；到 3 触发完整停止。 |
| `lastSyncError` | text | 保存最近一次具体同步错误；成功不会把历史错误伪装为当前失败。 |
| `visionTrackingActive` | boolean | 可在同一在线会话内暂停/恢复，不决定 sessionId。 |

### State transitions

```text
Inactive
  -> Starting (正式启动已通过 RobotSim/机器人门禁)
  -> Active (纠偏控制器接受启动)
  -> ActivePaused (在线内部等待 O/D 或分段切换，仍是同一会话)
  -> Active (Vision 恢复)
  -> Completed | Failed | Stopped
  -> Inactive
```

任意终态必须先使 `active=false`，再清除帧状态和派发停止动作，保证迟到结果不可见。

## 2. Frozen Frame Context（冻结帧上下文）

相机帧到达运行时协调边界时形成的按值快照。它是 RobotSim 查询、Vision 输入和 UI 关联的单一来源。

### Fields

| Field | Type / Domain | Rules |
|---|---|---|
| `frame` | CameraFrame value | 必须保存原始 frameId 和图像值语义；不得在派发时重新取最新帧。 |
| `frameId` | signed frame integer | 与 `frame.frameId` 相同；当前处理期间不可改变。 |
| `hasRobotState` | boolean | 来自帧到达瞬间的运行时状态有效标志。false 时禁止 RobotSim 查询。 |
| `robotState` | RobotState value | 仅在 `hasRobotState=true` 时可用于查询和 Vision；后续状态不得覆盖。 |
| `purpose` | tracking purpose | 取帧到达时的 Online/Offline/Global purpose；纠偏不能使用全局定位段号。 |
| `sessionId` | correction session integer | 在线/离线必须等于活动会话；全局定位为 0。 |
| `localizationSegmentIndex` | integer | 仅全局定位使用；在线/离线固定为 -1。 |
| `requestId` | 非零单调整数或 0 | 逐帧 RobotSim 查询派发时赋值；未派发为 0。 |

### Validation

- 纠偏上下文有效要求：session active、purpose 与 session 一致、frameId 非负、`hasRobotState=true`。
- 当前帧一旦派发，在 RobotSim 或 Vision 返回前不得被新帧替换。
- 待处理上下文允许被更晚相机帧整体替换，不能只替换图像而保留旧 RobotState。

## 3. Bounded Frame Slots（有界帧槽）

| Slot | Cardinality | Lifecycle |
|---|---:|---|
| Current / in-flight | 0..1 | 从 pending 移入开始，覆盖 RobotSim 查询和 Vision 处理，直到成功结果、明确失败或会话停止。 |
| Latest pending | 0..1 | 当前帧占用时保存最新到达的完整 Frozen Frame Context；新帧整体覆盖旧 pending。 |

### Invariants

- `currentCount <= 1`。
- `pendingCount <= 1`。
- current 完成后只取当时的 latest pending。
- session 终止时两个槽同时清空。
- pending 从未派发过 RobotSim 查询；停止时清除它不会产生取消请求。

## 4. Per-frame RobotSim Observation（逐帧 RobotSim observation）

复用现有 DigitalTwin observation 值对象，不增加 UI sessionId 到 RobotSim 领域类型。

### Relevant fields and rules

| Field | Rules |
|---|---|
| `valid` | false 表示当前帧同步失败；不得提交 Vision。 |
| `frameId` | 必须等于 Frozen Frame Context.frameId。 |
| `laserLineImagePoints_px` | 有效性继续遵守 DigitalTwin 既有转换契约。 |
| `laserLinePoints_mm` | 保留 RobotSim 返回值；本特性不处理二维/三维点数差异。 |
| `weldPointImage_px` / validity | 保留现有适配行为。 |
| `message` | 失败时保留原始具体原因；错帧时追加 expected/actual。 |

### Relationships

```text
Correction Run Session 1 ---- 0..1 Current Frozen Frame
Correction Run Session 1 ---- 0..1 Latest Pending Frozen Frame
Frozen Frame 1 ---- 0..1 Per-frame Observation
Frozen Frame + Observation 1 ---- 0..1 Vision Result
```

## 5. Vision Result Correlation（Vision 结果关联）

在 UI 运行时输出元数据中关联 Vision 结果，不修改 Vision 算法领域结果含义。

### Fields

| Field | Rules |
|---|---|
| `correctionSessionId` | 纠偏提交时复制；普通/全局路径为 0。 |
| `trackingPurpose` | 纠偏提交时复制；用于避免在线结果路由到离线或反向。 |
| `frameId` | 必须与当前 Frozen Frame 和 observation 一致。 |
| Existing detection fields | 保持原有 valid、sample、gap、message 和命令触发语义。 |

### Acceptance rule

只有 `(sessionId, purpose, frameId)` 与当前在途上下文全部相等，结果才可以：

1. 将 UI pending observation 标记为 Vision 已消费；
2. 进入对应 Online/Offline worker；
3. 释放 current 槽并派发 latest pending。

任何不匹配结果只被丢弃，不能清空或覆盖新会话状态。

## 6. Correction Sync Update（纠偏同步更新）

在线和离线共用的 UI 通知 DTO，与全局定位专属信号独立。

### Fields

| Field | Rules |
|---|---|
| `state` | `Idle`, `Querying`, `ObservationReady`, `VisionConsumed`, `Failed`, `Cleared`, `StoppedAfterFailures`。 |
| `purpose` | Online/Offline；Idle 可为 None。 |
| `sessionId` | 非零事件必须携带；UI 用于跨会话隔离。 |
| `frameId` | 帧相关事件必须携带；会话级停止可为 -1。 |
| `hasObservation` | 仅 ObservationReady/VisionConsumed 为 true。 |
| `observation` | 按值保存当前逐帧 observation；不能来自普通实时缓存。 |
| `consecutiveSyncFailures` | 当前会话计数，范围 0..3。 |
| `message` | 当前状态说明或具体错误。 |

### UI confirmation state

UI 为在线和离线分别显示同一 DTO 的投影，但只维护一个与活动会话对应的 pending observation：

- ObservationReady：替换 pending，状态显示“等待 Vision 消费”。
- VisionConsumed：仅 sessionId/frameId 与 pending 相同才更新最近确认 frameId，并清除 pending。
- Failed/Cleared/Stopped：清除 pending；Failed 更新最近错误。
- 新 sessionId：先清除旧 pending 和旧会话未确认状态。

## 7. Sync Failure（同步链故障）

### Included categories

- NoRobotState
- RobotSimDisconnected
- QueryFailed
- InvalidOrEmptyObservation
- FrameIdMismatch
- CommunicationError

### Excluded categories

- Vision 已正常运行但未检测到目标
- 纠偏算法业务上拒绝样本
- Control 状态机正常完成或分段暂停

### Transition rule

```text
failure -> count + 1 -> clear current observation
count 1/2 -> release current -> process latest pending when present
valid matching observation -> count = 0 -> submit Vision
count 3 -> invalidate session -> clear both slots -> complete stop -> report reason
```

## 8. Normal Realtime RobotSim State（普通实时 RobotSim 状态）

| Field / State | Correction inactive | Correction active |
|---|---|---|
| Latest RobotState cache | update | update |
| Pending ordinary state | latest-wins | cleared / not created |
| New ordinary query dispatch | allowed | prohibited |
| Ordinary UI result | allowed | 不得作为纠偏 observation；纠偏专用 UI 不接收 |
| On correction stop | N/A | 以最新 RobotState 恢复普通查询 |

