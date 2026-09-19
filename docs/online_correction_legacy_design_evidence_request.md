# 在线纠偏旧工程设计证据调查请求（D2–D6）

## 1. 调查目的

请在“重构前项目/旧工程”中进行只读调查，为当前项目的在线纠偏初始轨迹缺陷补齐设计证据。

本任务不是让你修改旧工程或提出补丁，而是回答当前实现前仍未确认的 D2–D6 产品/算法语义，并给出可复核的源码证据。

必须区分：

- `ACTIVE_PRODUCTION_PATH`：旧工程实际会执行的活动路径；
- `INACTIVE_EXPERIMENT`：被常量、模式或条件关闭的实验分支；
- `COMMENT_ONLY`：仅存在于注释或设计说明；
- `HISTORICAL_ARTIFACT`：旧文件或变量仍存在，但当前活动路径未使用；
- `INFERENCE`：无法直接证明、只能推测的结论。

禁止把 `INACTIVE_EXPERIMENT`、`COMMENT_ONLY` 或 `INFERENCE` 写成正式产品事实。

## 2. 调查约束

- 只读旧工程源码、配置、轨迹文件、JBI、Git 历史和历史日志。
- 不修改任何代码、测试、轨迹文件或文档。
- 不生成 patch。
- 不运行会连接真实机器人或发送 N/O/P/J 指令的程序。
- 不根据当前重构项目的文档反推旧工程语义。
- 不以“看起来合理”代替证据。
- 优先使用文件名、函数名、变量名和精确行号证明结论。
- 引用代码时只摘录必要片段，不要转储大文件。
- 若不同版本行为不同，必须分别给出 commit/tag/分支和差异。

## 3. 当前已经确认的事实

以下事实无需重新争论，但可以用旧工程证据补强：

1. 当前重构项目 `COnlineCorrectionTrajectoryPlanner::planInitialCorrection()` 使用：

   ```text
   initialPoint - correctedSegment.front()
   ```

   对当前 `_W` 分段进行整体 XYZ 平移。

2. 当前 `_T` 也使用自身段首计算整体 XYZ 平移。

3. 历史 session `20260914_174544_959` 的 277 个真实初始检测样本，当前算法产生约 `103.360048 mm` 的整体平移。

4. 独立 `_W` 连续折线投影得到约：

   ```text
   matchedPathPointIndex = 162
   segmentFraction = 0.279970047638
   projectionDistance = 1.614820100 mm
   localResidual = (-1.437455625, -0.735259331, -0.027549940) mm
   reverseDirectionCosine = 0.996410057
   scanSpan = 4.670802565 mm
   ```

5. 当前历史第一条 O payload 与当前 Planner replay 完全一致。

6. 当前生产 Planner 没有真正执行五个 `initialPathMatch` 安全门限。

7. 软件缺陷已确认，但“该 Planner 缺陷导致现场实际碰撞”尚未完全证明。

8. D1 已由负责人正式确认：

   - `_T` 第九列是 segment 标识字段；
   - 第九列值 `0` 表示 segment 0；
   - 该字段可以参与 `segmentIndex` 对 `_T` 段的选择。

D1 只确认 `_T` 的 segment label 语义，不证明 `_T` 到 `_W` 的路径范围或进度映射。

## 4. 当前参考数据

当前重构项目中的历史 fixture：

```text
_T:
/Users/yangcai0130/Desktop/YangCai_qt-sontech20260901-main/data/WorkPiece/20260404CHUFENGKOU1_T.txt

_W:
/Users/yangcai0130/Desktop/YangCai_qt-sontech20260901-main/data/WorkPiece/20260404CHUFENGKOU1_W.txt

RBCALIB:
/Users/yangcai0130/Desktop/YangCai_qt-sontech20260901-main/config/calibration/RBCALIB.DAT

session:
/Users/yangcai0130/Desktop/YangCai_qt-sontech20260901-main/output/online_correction/20260914_174544_959
```

该 session 没有持久化 `startTeachIndex/endTeachIndex`。历史 replay 使用 `_T[0..1]`、`_W[0..341]` 只是为了复现当时记录的 342 个 corrected path points，不能作为生产硬编码或既定产品语义。

## 5. 优先检查的旧工程入口

请先定位并追踪以下函数、变量及其真实调用者：

```text
OnStartinitdet_change
OnlineReverseDetectMatch
OnlineTeachTrajectoryInitial
correctTeachTraj
correctTeachPts
OnStartoncor_change
OnFullonlinecor
OnlineExceptionHandling_beta
CorrectTaughtPathO
ReadStationPosition
RobotPos2StationPos
StationPos2RobotPos
```

同时搜索以下旧变量或等价变量：

```text
correctStartPtType
initPtMatchType
D_initdata
AllTpMatched
AllTp_segement_line_and_curve
curve_index
matchTime
preDetLstPtToAllTpId
segDetFstPtToAllTpId
sim_scan_line_change_point
```

变量名可能存在拼写差异。请通过调用链和数据流确认，不要只依赖文本命中。

## 6. D2：`_T -> _W` 合法分段范围

### 必须回答

1. 旧工程如何读取 `_T` 第九列？变量正式名称和类型是什么？
2. `segmentIndex`、`curve_index` 或其他段号从哪里产生、何时递增、何时复位？
3. segment 0 在旧工程中如何确定对应的 `_T` 起止行？
4. 当前 `_T` 段如何映射到 `_W`：

   - 显式索引；
   - 共享采样序号；
   - 首尾最近点；
   - 累计距离/路径进度；
   - 独立 mapping 表；
   - 还是没有映射？

5. 是否存在保存或恢复 `_W` 起止索引的字段、文件或全局变量？
6. 自交、重复焊缝或首尾坐标相近时，旧工程如何避免映射到错误支路？
7. 多个 `_T` 点是否被验证位于同一 `_W` 连续区间？
8. 区间端点是闭区间还是开区间？
9. mapping 缺失或失败时，旧工程是停止、回退、搜索整个 `_W`，还是继续发送 O？
10. 请确认这些行为属于活动生产路径还是未启用实验。

### 必须给出的证据

- `_T` 解析函数和字段顺序；
- segment 选择调用链；
- `_T` 起止点到 `_W` 范围的计算代码；
- 范围的实际消费者；
- 相关配置/文件格式样例；
- 如有 Git 历史，说明该 mapping 何时引入或改变。

### D2 输出

```text
D2_LEGACY_AUTHORITY
D2_T_SEGMENT_SELECTION
D2_W_RANGE_START
D2_W_RANGE_END
D2_BOUNDARY_SEMANTICS
D2_MAPPING_FAILURE_BEHAVIOR
D2_ACTIVE_OR_INACTIVE
D2_CONFIDENCE
D2_MISSING_EVIDENCE
```

## 7. D3：投影或匹配候选筛选顺序

当前重构项目尚未实现连续投影，必须从旧工程确认是否已有正式候选选择规则。

### 必须回答

1. `OnlineReverseDetectMatch()` 的活动 `initPtMatchType` 是什么？
2. 活动路径是否只取 `D_initdata.back()`，还是会进行最近点/ICP/投影？
3. 如果存在 `initPtMatchType==1` 或 ICP 分支：

   - 是否能在生产配置中进入；
   - 候选范围是当前 segment 还是完整 `_W`；
   - 是先按距离选择，还是先按方向过滤；
   - 是否检查扫描方向；
   - 是否有最大距离、最小跨度、剩余路径等门限；
   - 多个候选如何打破平局；
   - 退化线段如何处理？

4. 是否存在连续线段投影，还是只比较离散点？
5. 是否存在注释明确指出跨焊缝或错误支路匹配风险？

### 证据分类要求

分别输出活动路径和实验路径，不得混合：

```text
D3_ACTIVE_PATH
D3_INACTIVE_EXPERIMENT
```

### D3 输出

```text
D3_ACTIVE_CANDIDATE_ORDER
D3_EXPERIMENTAL_CANDIDATE_ORDER
D3_DIRECTION_FILTER_ORDER
D3_DISTANCE_FILTER_ORDER
D3_TIE_BREAK_RULE
D3_DEGENERATE_SEGMENT_RULE
D3_ACTIVE_OR_INACTIVE
D3_CONFIDENCE
D3_OWNER_DECISION_STILL_REQUIRED
```

若旧工程没有正式规则，明确输出：

```text
D3_LEGACY_RULE_NOT_FOUND
```

## 8. D4：匹配后的剩余轨迹来源

### 必须追踪

从初始反向检测结束开始，追踪到全部 O 点生成：

```text
D_initdata / initial point
-> OnlineReverseDetectMatch
-> OnlineTeachTrajectoryInitial
-> correctTeachTraj
-> correctTeachPts
-> O command buffer / RobotCtrl
```

### 必须回答

1. 首个动态 O 的 XYZ、RX、RY、RZ、S1、S2 分别来自 `_W`、`_T`、当前 RobotState 还是其他变量？
2. 后续 O 点来自 `_T` 全段、剩余 `_T`、`_W` 采样，还是固定数量关键点？
3. `_T` 中每个点是否存在对应的 `_W` 索引或连续路径进度？
4. 如果匹配发生在 `_W` 中间，如何确定第一个尚未经过的 `_T` 点？
5. 是否会重新发送已经经过的路径？
6. `segmentFraction` 等价概念是否存在？若不存在，请明确说明。
7. 同一个局部偏差是否作用于所有后续 O 点？
8. 首个 O 与后续第一个 O 的 XYZ、姿态、S1/S2 连续性是否检查？阈值是什么？
9. JBI 中 P001/P002 与 O 点索引的对应关系是什么？只要求数据对应关系，不要扩大到修改 JBI。

### D4 输出

```text
D4_FIRST_O_SOURCE_XYZ
D4_FIRST_O_SOURCE_ORIENTATION
D4_FIRST_O_SOURCE_S1_S2
D4_SUBSEQUENT_O_SOURCE
D4_T_TO_W_PROGRESS_MAPPING
D4_ALREADY_PASSED_PATH_HANDLING
D4_RESIDUAL_APPLICATION
D4_CONTINUITY_CHECK
D4_ACTIVE_OR_INACTIVE
D4_CONFIDENCE
D4_OWNER_DECISION_STILL_REQUIRED
```

## 9. D5：GlobalCorrection 的完整姿态语义

### 必须回答

1. 旧工程 GlobalCorrection 对应的模式、枚举或 `correctStartPtType` 值是什么？
2. 全局修正矩阵的变量名、生成方、矩阵方向和坐标系是什么？
3. 它表示：

   - virtual-to-real；
   - real-to-virtual；
   - model-to-station；
   - station-to-robot；
   - 还是其他变换？

4. 矩阵是完整 6DOF 刚体变换，还是产品只允许平移/绕 Z 旋转？
5. XYZ 的实际计算公式是什么？
6. RX/RY/RZ 的实际组合方式是什么：

   - 旋转矩阵左乘；
   - 右乘；
   - 只增加 RZ/yaw；
   - 直接保留原姿态；
   - 或重新求逆解？

7. 欧拉角顺序、角度范围和奇异点处理是什么？
8. S1/S2 是保持原 `_T/_W` 值、插值，还是通过运动学重新求解？
9. `RobotPos2StationPos()` 和 `StationPos2RobotPos()` 的矩阵组合顺序是什么？
10. GlobalCorrection 后进行路径匹配时，使用变换前还是变换后的 `_W`？
11. 当前旧工程行为是否只是历史兼容，还是经过产品确认的正式语义？

### D5 输出

```text
D5_MODE_SELECTOR
D5_MATRIX_NAME
D5_MATRIX_PRODUCER
D5_MATRIX_DIRECTION
D5_INPUT_FRAME
D5_OUTPUT_FRAME
D5_XYZ_FORMULA
D5_ORIENTATION_FORMULA
D5_EULER_ORDER
D5_S1_S2_RULE
D5_MATCH_BEFORE_OR_AFTER_GLOBAL_TRANSFORM
D5_ACTIVE_OR_INACTIVE
D5_CONFIDENCE
D5_OWNER_DECISION_STILL_REQUIRED
```

## 10. D6：三列 `_W` 的旧工程生命周期

### 必须回答

1. 旧工程支持哪些 `_W` 行格式：3 列、6 列、8 列、9 列或其他？
2. 每一列的正式字段名和单位是什么？
3. 三列 `_W` 用于哪些功能和模式？
4. 三列 `_W` 进入 InitType/GlobalCorrection 时，首个 O 的 RX/RY/RZ/S1/S2 从哪里获得？
5. 是否由 `_T` 补齐？如果是，如何保证与匹配 `_W` 位置一致？
6. 缺失姿态或外轴时是否会拒绝、使用零值、继承段首，还是继续发送？
7. 是否存在设备、工件或配置明确依赖三列 `_W`？
8. 九列 `_W` 是后续新增格式还是旧工程原生格式？请给 Git 历史或文件样例。

### D6 输出

```text
D6_SUPPORTED_W_FORMATS
D6_COLUMN_DEFINITIONS
D6_THREE_COLUMN_CALLERS
D6_FIRST_O_MISSING_FIELDS_SOURCE
D6_FAILURE_OR_FALLBACK_BEHAVIOR
D6_PRODUCTION_DEPENDENCIES
D6_FORMAT_HISTORY
D6_ACTIVE_OR_INACTIVE
D6_CONFIDENCE
D6_OWNER_DECISION_STILL_REQUIRED
```

## 11. 附加证据：碰撞阶段边界

仅在旧工程/JBI/日志已有证据时补充，不要重新制造事故：

```text
FINISHREVERSEDET
-> MOVL P001
-> ENDDETPRG
-> STARTCOR
-> SMOVC
```

请明确：

- P001 对应哪一条 O/Pvar；
- STARTCOR 前后的算法边界；
- `OnlineCorrectionPathFollower` 等价旧逻辑是否只在 STARTCOR 后生效；
- 旧记录能否证明事故发生在 MOVL P001 阶段。

若证据不足，必须输出：

```text
COLLISION_PHASE_NOT_PROVEN
```

## 12. 证据质量规则

每条核心结论至少提供：

```text
file:
function:
line(s):
commit/tag if available:
execution condition:
evidence classification:
quoted behavior:
conclusion:
```

判断活动路径时必须说明：

- 调用入口；
- 条件变量；
- 默认配置值；
- 是否被硬编码关闭；
- 是否最终到达 RobotCtrl/O command。

如果找不到证据，不要补充设计，应写：

```text
NOT_FOUND
NOT_PROVEN
OWNER_DECISION_REQUIRED
```

## 13. 最终回传格式

请严格按以下章节输出：

```text
LEGACY_SOURCE_PROVENANCE

ACTIVE_INITIAL_CORRECTION_CALL_CHAIN

D1_CONFIRMATION_CROSS_CHECK

D2_SEGMENT_MAPPING_EVIDENCE

D3_PROJECTION_CANDIDATE_EVIDENCE

D4_REMAINING_TRAJECTORY_EVIDENCE

D5_GLOBAL_CORRECTION_EVIDENCE

D6_THREE_COLUMN_W_EVIDENCE

COLLISION_PHASE_EVIDENCE

CONFIRMED_LEGACY_SEMANTICS

INACTIVE_OR_EXPERIMENTAL_SEMANTICS

CONTRADICTIONS_BETWEEN_VERSIONS

MISSING_EVIDENCE

OWNER_DECISIONS_STILL_REQUIRED

IMPLICATION_FOR_TRACK_A

IMPLICATION_FOR_TRACK_B

GO_NO_GO_FOR_IMPLEMENTATION_PLAN
```

阶段门规则：

- 只有 D2–D6 的核心语义都能由活动生产路径或负责人正式定义证明，才可建议 `GO`。
- 任一项只能从未启用实验、注释或推断得到，必须保持 `NO_GO`。
- 即使 Planner 软件缺陷得到确认，也不能据此宣布现场碰撞因果已经证明。

## 14. 特别提醒

- 不要把历史 replay 的 `_W[0..341]` 当成旧工程既定分段，除非旧源码或同次运行记录明确保存了该范围。
- 不要把 `initPtMatchType==1` 的 ICP 分支当成活动生产算法，除非证明默认配置确实进入该分支。
- 不要因为旧工程继续发送 O，就把该行为认定为安全设计。
- 不要把当前重构项目文档中描述的 constrained matching 反向归因于旧工程。
- 如果旧工程本身没有定义安全行为，请明确指出这是需要产品负责人决定的新设计，而不是“恢复旧逻辑”。

