# 在线纠偏旧工程设计证据调查报告（D2-D6）

调查对象是用户提供的只读目录：

```text
/Users/yangcai0130/Desktop/SontechWorks_Weld_Merge-main
```

该目录没有 `.git`，因此没有可核验的 commit、tag 或分支历史。以下结论只基于当前目录中的源码和轨迹文件，不运行程序、不连接机器人、不发送任何 N/O/P/J 指令。

## LEGACY_SOURCE_PROVENANCE

- 工程形态：MFC/Visual Studio 工程，入口为 `SWv1/SWv1.vcxproj`；不是 Qt 重构工程。
- 主要证据文件：
  - `SWv1/OnlineCor.h/.cpp`
  - `SWv1/robotleadingDlg.cpp`
  - `SWv1/RobotCtrl.cpp`
  - `Robotleading/robotleading_method/RStdAfx.h/.cpp`
  - `SWv1/data/Robotleadingdata/WorkPiece/*_{T,W}.txt`
- 版本证据：目录无 `.git`；源码头部只有人工日期注释，不能替代 commit/tag。
- 证据分类：源码中实际可达的分支标记为 `ACTIVE_PRODUCTION_PATH`；硬编码不可达、注释掉或只存在于旧注释中的分支标记为 `INACTIVE_EXPERIMENT` 或 `COMMENT_ONLY`；无法由源码直接命名的产品语义标记为 `INFERENCE`。

## ACTIVE_INITIAL_CORRECTION_CALL_CHAIN

活动链如下：

```text
COnlineCor::OnBnFULLONLINECOR()
  -> OnFullonlinecor()
  -> OnStartinitdet_change()
  -> OnlineReverseDetectMatch(tempDetInitdata)
  -> OnlineTeachTrajectoryInitial()
  -> correctTeachTraj(tempDetInitdata)
  -> correctTeachPts(tempDetInitdata)
  -> RobotCtrl::RevRobotData() 的 O payload
  -> 后续 RobotCtrl::RevRobotData() 的 P payload / CorrectTaughtPathO()
```

证据：`SWv1/OnlineCor.cpp:3470-3551` 初始化并进入 `OnFullonlinecor`；`SWv1/OnlineCor.cpp:3396-3417` 在检测完成后进入 `OnStartinitdet_change`；`SWv1/OnlineCor.cpp:2643-2849` 调用反向匹配、轨迹整体修正和示教点修正；`SWv1/RobotCtrl.cpp:318-341` 以 `s_offline == 4` 消费 `CTeachPts/CStationTeachPts` 发送 O；`SWv1/RobotCtrl.cpp:267-297` 在 `s_online == 2` 时发送 P。

默认起始点模式在 `SWv1/OnlineCor.cpp:153-185` 被设置为 `correctStartPtType::InitType`。本目录中没有找到把它改成 `GolabalCor` 或 `IBVSStart` 的生产入口；因此“默认在线纠偏”只能确认是 `InitType`，其他模式必须由外部配置/未提供代码确认。

## D1_CONFIRMATION_CROSS_CHECK

- `SWv1/OnlineCor.cpp:624-719` 和 `728-819` 的 `GetData_T` 读取前三列 XYZ，随后依次读取 RX、RY、RZ、S1、S2 和第九列 `segment`。
- 第九列没有独立结构体字段名，临时变量正式名称是 `temp_count`；语义由注释 `// segment` 和比较条件 `Online_sergement < atof(temp_count.c_str())` 给出。
- 当当前 `Online_sergement` 为 0 且第九列为 0 时，条件不成立，行属于 segment 0；遇到大于当前段号的值时，前一行被记为 `endID`。
- 结论：D1 关于 `_T` 第九列是 segment label、值 0 表示 segment 0 的确认与源码一致。它不证明 `_T` 到 `_W` 的产品级进度映射。

## D2_SEGMENT_MAPPING_EVIDENCE

### D2_LEGACY_AUTHORITY

`GetData_T` 是 `_T` 段号和 `_T` 行范围的权威读取函数：`SWv1/OnlineCor.cpp:624-719`、`728-819`。字段顺序是 `x y z rx ry rz s1 s2 segment`；前三列被保留到 `D`，姿态和外轴在该函数中只是被跳过，姿态另由 `getData_R` 读取。

### D2_T_SEGMENT_SELECTION

- `startID` 和 `endID` 是 `COnlineCor` 成员，声明于 `SWv1/OnlineCor.h:119-123`，初始值在 `SWv1/OnlineCor.cpp:163-185` 设为 0。
- `GetData_T` 在前一段结束后令 `startID = endID + 1`，首段令 `startID = 0`（`SWv1/OnlineCor.cpp:640-647`、`743-750`）。
- 读取过程中首次发现 `Online_sergement < segment` 时，令 `endID = counter - 1`（`SWv1/OnlineCor.cpp:661-673`、`765-776`）。若当前段一直读到文件末尾，`startID >= endID` 时令 `endID = counter - 1`（`SWv1/OnlineCor.cpp:686-689`、`788-790`）。
- 段完成后 `Online_sergement++`；整件焊缝完成时复位为 0，并清空 `startID/endID`（`SWv1/OnlineCor.cpp:1865-1932`）。

### D2_W_RANGE_START / D2_W_RANGE_END

`OnlineTeachTrajectoryInitial` 先用 `GetData_S` 读完整 `_W` 几何点，再用 `_T[startID]` 与 `_T[endID]` 在完整 `AllTp` 中各做一次最近离散点搜索：

- `_W` 起点：`findnearestRangePts(0, AllTp.size()/3, AllTp, Teach_Pt[3*startID..])`，证据 `SWv1/OnlineCor.cpp:2223-2235`、`2305-2309`。
- `_W` 终点：同一调用方式对 `Teach_Pt[endID]`，证据同上。
- 实际复制区间是 `for (int i = start_index; i <= end_index; ++i)`，证据 `SWv1/OnlineCor.cpp:2310-2317`。
- 没有独立 mapping 表、共享采样序号、累计距离或保存的 `_W` 起止字段。`AllTp_segement_line_and_curve` 只在内存中保存本次最近索引切出的向量，声明于 `RStdAfx.h:287-293`。

### D2_BOUNDARY_SEMANTICS

- `findnearestRangePts` 的搜索范围是半开 `[s,e)`，函数在 `SWv1/OnlineCor.cpp:3427-3463` 中用 `for (k=s; k<e; ++k)` 实现；相等距离不更新，因此平局保留先遇到的点。
- 生成 `teach_point_segement` 时是闭区间 `[start_index,end_index]`。若 `start_index > end_index`，当前活动代码不会交换端点，循环为空；旧的交换/分段代码位于注释或已停用路径。
- 没有检查多个 `_T` 点是否位于同一 `_W` 连续支路，也没有自交/重复焊缝支路判别、方向过滤、投影参数或跨度门限。

### D2_MAPPING_FAILURE_BEHAVIOR

- segment label 缺失或大于所有已有段号时，`GetData_T` 会落入“读到最后一段”的 `endID = counter - 1` 回退（`SWv1/OnlineCor.cpp:686-689`、`788-790`）。这会让 `_T` 范围扩展到文件尾，不能证明这是有意的安全策略。
- 最近点失败时 `findnearestRangePts` 仅在 `s>e` 返回 `-1`（`SWv1/OnlineCor.cpp:3434-3437`）；调用者没有检查 `-1`，随后直接用它构造区间。空文件也没有显式拒绝。由此只能确认“没有失败门禁”，不能把继续发送 O 认定为安全设计。
- `D2_MAPPING_FAILURE_BEHAVIOR = NOT_PROVEN / unsafe fall-through observed`。

### D2_ACTIVE_OR_INACTIVE / D2_CONFIDENCE / D2_MISSING_EVIDENCE

- `D2_ACTIVE_OR_INACTIVE = ACTIVE_PRODUCTION_PATH`：`OnlineTeachTrajectoryInitial` 在 `OnlineReverseDetectMatch` 中实际调用（`SWv1/OnlineCor.cpp:1971-1975`）。
- `D2_CONFIDENCE = high`，针对“全量 `_W` 最近离散点 + 闭区间复制”结论；针对安全失败语义为 `low`。
- `D2_MISSING_EVIDENCE`：无 Git 历史、无持久化 `_W` 起止索引、无产品格式/段映射说明、无现场配置可证明自交/重复支路保护。

## D3_PROJECTION_CANDIDATE_EVIDENCE

### D3_ACTIVE_PATH

- `OnlineReverseDetectMatch` 内部把 `initPtMatchType` 硬编码为 0，只有 `m_correctStartPtType == IBVSStart` 时改为 2（`SWv1/OnlineCor.cpp:1961-1969`）。默认 `InitType` 因此进入 0。
- 类型 0 只取 `D_initdata[S_num-1]`，即反向扫描最后一个检测点（`SWv1/OnlineCor.cpp:2014-2035`）。没有最近点、连续投影、ICP、方向或跨度筛选。
- 类型 2 只取当前机器人点 `curx/cury/curz`（`SWv1/OnlineCor.cpp:2192-2205`），仅适用于 IBVSStart。
- `D3_ACTIVE_CANDIDATE_ORDER = single-source selection: D_initdata.back() (InitType), or current RobotState (IBVSStart)`。
- `D3_DIRECTION_FILTER_ORDER = none`；`D3_DISTANCE_FILTER_ORDER = none`；`D3_TIE_BREAK_RULE = not applicable`；`D3_DEGENERATE_SEGMENT_RULE = not applicable`。

### D3_INACTIVE_EXPERIMENT

`initPtMatchType == 1` 分支是源码中的 ICP 实验分支，但在当前文件中不可达：局部变量在 `SWv1/OnlineCor.cpp:1967` 初始化为 0，未发现生产配置写入 1。该分支对模型前 10% 和后 10% 采样、检测轨迹每 5 点采样，然后 ICP（`SWv1/OnlineCor.cpp:2037-2110`）；`match_type` 又硬编码为 1，最终把 `afterMatchedTeach[0]` 作为起点（`SWv1/OnlineCor.cpp:2104-2124`、`2174-2185`）。原本寻找匹配末点的代码被注释（`SWv1/OnlineCor.cpp:2174-2182`）。

因此：`D3_EXPERIMENTAL_CANDIDATE_ORDER = ICP branch sample/fit, then first transformed model point`；它不是当前活动生产规则。

### D3_ACTIVE_OR_INACTIVE / D3_CONFIDENCE / D3_OWNER_DECISION_STILL_REQUIRED

- `D3_ACTIVE_OR_INACTIVE = ACTIVE_PRODUCTION_PATH` 对类型 0/2；`INACTIVE_EXPERIMENT` 对类型 1。
- `D3_CONFIDENCE = high`。
- `D3_LEGACY_RULE_NOT_FOUND`：旧工程没有找到连续线段投影、扫描方向过滤、最大距离、最小跨度、剩余路径或跨焊缝防错的正式活动规则。
- `D3_OWNER_DECISION_STILL_REQUIRED = yes`：需要负责人决定新实现是保留旧的末点语义，还是引入带门限的连续路径匹配。

## D4_REMAINING_TRAJECTORY_EVIDENCE

### D4_FIRST_O_SOURCE_XYZ / ORIENTATION / S1_S2

- 活动在线纠偏完成初始检测后，`OnStartinitdet_change` 在 `SWv1/OnlineCor.cpp:2821-2849` 依次调用 `OnlineReverseDetectMatch`、`correctTeachTraj`、`correctTeachPts`。
- `correctTeachPts` 读取当前工件 `_T.txt`，不是 `_W.txt`（`SWv1/OnlineCor.cpp:3819-3825`），并计算 `numOfTps = endID-startID+1`（`3827-3828`）。
- `_T` 的 XYZ、RX/RY/RZ、S1/S2 来自 `getData_R` 的八列解析（`SWv1/OnlineCor.cpp:3646-3721`）。
- 全局/初始对齐后，第一个 O 对应 `_T[startID]` 的修正结果：XYZ 先应用可选的 `deviationOfReverseTPs`，再加 `DetInitdata - sp[startID].xyz` 的统一平移（`SWv1/OnlineCor.cpp:3866-3915`）。姿态在正式 GlobalCorrection 路径只增加 Rz；S1/S2 保持 `_T` 值并经坐标转换（`3892-3899`、`3916-3928`）。
- 因此 `D4_FIRST_O_SOURCE_XYZ = corrected _T[startID]`，`D4_FIRST_O_SOURCE_ORIENTATION = _T[startID]`（GlobalCorrection 仅 Rz 增量），`D4_FIRST_O_SOURCE_S1_S2 = _T[startID]`。

### D4_SUBSEQUENT_O_SOURCE / D4_T_TO_W_PROGRESS_MAPPING

- `correctTeachPts` 对 `i=startID .. startID+numOfTps-1` 逐点写入 `CTeachPts` 和 `CStationTeachPts`，并令 `PvarNum=numOfTps`（`SWv1/OnlineCor.cpp:3919-3931`）。
- `RobotCtrl::RevRobotData` 在 `s_offline==4` 时每次以 `P_countId` 发送一个 O，字段来自 `CTeachPts[6*P_countId..]` 与 `CStationTeachPts[2*P_countId..]`（`SWv1/RobotCtrl.cpp:318-341`）。因此后续 O 来源是剩余 `_T` 行，不是 `_W` 采样点或固定数量关键点。
- `_T -> _W` 只有 `OnlineTeachTrajectoryInitial` 中的两个全量最近点用于截取几何 `_W`；没有保存到 O 发送结构，也没有 `segmentFraction` 或连续进度变量。`Teach_point_count` 只用于后续离散 `AllTpMatched` 追踪（`SWv1/robotleadingDlg.cpp:16971-17020`）。
- 已经过的路径没有基于投影进度的显式跳过规则；首个 O 仍是 `_T[startID]` 对齐后的点。`D4_ALREADY_PASSED_PATH_HANDLING = NOT_FOUND`。

### D4_RESIDUAL_APPLICATION / D4_CONTINUITY_CHECK

- `correctTeachTraj` 对当前 `teach_point_segement` 的全部几何点施加同一个 XYZ 平移，证据 `SWv1/OnlineCor.cpp:3758-3809`。
- `correctTeachPts` 对当前 `_T` 段全部姿态点施加同一个 XYZ 平移；GlobalCorrection 正式路径只把变换矩阵的 Rz 增量叠加到 Rz，Rx/Ry 注释掉（`SWv1/OnlineCor.cpp:3884-3900`）。
- 没有发现首个 O 与下一个 O 的姿态/外轴连续性阈值检查，也没有“第一 O 必须等于动态检测点”的单独门禁。`D4_CONTINUITY_CHECK = NOT_FOUND`。
- 反向扫描后的连续跟踪不是 O 生成：`robotleadingDlg.cpp:4738-4746` 根据 `g_simPriorCorrectionFlowMode` 调用 `CorrectTaughtPathNew` 或活动的 `CorrectTaughtPathO`；后者把 `d_x/d_y/d_z` 交给 RobotCtrl 的 P payload（`robotleadingDlg.cpp:17876-17898`、`SWv1/RobotCtrl.cpp:267-297`）。

### D4_ACTIVE_OR_INACTIVE / D4_CONFIDENCE / D4_OWNER_DECISION_STILL_REQUIRED

- `D4_ACTIVE_OR_INACTIVE = ACTIVE_PRODUCTION_PATH`，除 `testCorFlag == false` 的常规起始点分支外；`testCorFlag` 在 `SWv1/OnlineCor.cpp:3830` 硬编码为 true，因此该分支是 `INACTIVE_EXPERIMENT`。
- `D4_CONFIDENCE = high` 对 O 来源和统一平移；对“已走路径是否安全跳过”和连续性为 `low`。
- `D4_OWNER_DECISION_STILL_REQUIRED = yes`：必须决定 O 是否应由 `_T` 段首开始、是否需要连续进度映射，以及初始 O 的姿态/外轴一致性门限。

## D5_GLOBAL_CORRECTION_EVIDENCE

### D5_MODE_SELECTOR / MATRIX_NAME / MATRIX_PRODUCER

- 枚举定义于 `SWv1/OnlineCor.h:49-55`：`InitType`、拼写为 `GolabalCor` 的全局修正、`IBVSStart`。
- 全局矩阵变量为 `deviationOfReverseTPs`，声明于 `SWv1/robotleadingDlg.h:71`，定义注释于 `SWv1/robotleadingDlg.cpp:596`。
- 全局定位生产函数 `CRobotleadingDlg::updateModelOfflineICP()` 在 `SWv1/robotleadingDlg.cpp:11048` 开始；它计算 `Arin0 = Ar * A0.i()`（`11117-11147`），将平移从米转换为毫米后调用 `transformSB2SCur`，结果写入 `deviationOfReverseTPs`（`11228-11234`）。
- `OnFullonlinecor` 仅在 `GolabalCor` 或仿真先验模式下读取 `_R.txt` 并执行 `correctReverseTP`（`SWv1/OnlineCor.cpp:3364-3372`、`3399-3405`）。

### D5_MATRIX_DIRECTION / INPUT_FRAME / OUTPUT_FRAME

- 源码直接证明的矩阵关系是 `Ar * A0^-1`：从记录的起始虚拟位姿 `A0` 到结束/修正位姿 `Ar` 的齐次变换，然后通过 `transformSB2SCur` 共轭转换到当前变位机动系（`SWv1/robotleadingDlg.cpp:11108-11110`、`11134`、`SWv1/robotleadingDlg.cpp:15780-15816`）。
- `correctReverseTP`、`correctTeachTraj` 和 `correctTeachPts` 将该矩阵左乘到变位机坐标中的示教点（`SWv1/OnlineCor.cpp:3558-3603`、`3758-3777`、`3866-3882`）。
- 因此 `D5_INPUT_FRAME = 变位机定系/全局定位结果经 transformSB2SCur 后的动系输入`，`D5_OUTPUT_FRAME = 变位机动系下修正后的示教点`。
- “virtual-to-real”或“model-to-station”这样的产品命名在代码中没有正式声明；`D5_MATRIX_DIRECTION` 的可证明部分是 `Ar*A0^-1`，完整产品语义仍为 `INFERENCE / OWNER_DECISION_REQUIRED`。

### D5_XYZ_FORMULA / ORIENTATION_FORMULA / EULER_ORDER

- XYZ：齐次矩阵左乘点，`TPtrans = deviationOfReverseTPs * TPtrans`，随后在正式初始对齐中再加 `DetInitdata - sp[startID].xyz` 的统一 XYZ 平移（`SWv1/OnlineCor.cpp:3772-3799`、`3868-3915`）。
- 正式 GlobalCorrection 姿态：`rMat2rpy` 使用 `eulerAngles(2,1,0)`，返回 RPY 顺序 `{roll,pitch,yaw}`，并只将 yaw 增量加到 `sp[..+5]`（`SWv1/OnlineCor.cpp:3728-3748`、`3892-3899`）。
- 仿真先验实验模式不是同一语义：`ApplySimPriorPoseRotation` 先将原 RPY 转为 `Rz*Ry*Rx`，再做 `R_transform * R_original` 后反解（`SWv1/OnlineCor.cpp:53-101`）。它由 `g_simPriorCorrectionFlowMode == 1` 控制（`48-51`），属于独立实验/仿真路径。
- `D5_S1_S2_RULE = 保持 _T/_R 的外轴值，只通过 StationPos2RobotPos 转换`，证据 `SWv1/OnlineCor.cpp:3919-3928`、`SWv1/robotleadingDlg.cpp:14474-14533`。
- `ReadStationPosition` 从 `RBCALIB.DAT` 读取 `////SRANG` 的六个值，使用负平移和负角构造 `Hom_Sb2Rb`（`SWv1/robotleadingDlg.cpp:4006-4047`）。正逆转换组合分别见 `RobotPos2StationPos` `14425-14464` 和 `StationPos2RobotPos` `14474-14533`；两者使用 ZYX 提取顺序并在反解时处理 pitch 超范围。
- `D5_MATCH_BEFORE_OR_AFTER_GLOBAL_TRANSFORM = 先用原始 _T/_W 做段范围最近点映射，之后才将全局矩阵应用到反向点/当前段轨迹`；证据 `OnlineTeachTrajectoryInitial:2223-2397` 与 `correctReverseTP/correctTeachTraj:3558-3603、3758-3777`。

### D5_ACTIVE_OR_INACTIVE / D5_CONFIDENCE / D5_OWNER_DECISION_STILL_REQUIRED

- `D5_ACTIVE_OR_INACTIVE = ACTIVE_PRODUCTION_PATH` 仅限明确设置 `GolabalCor` 的路径；默认构造为 `InitType`，本目录没有找到模式设置入口，因此默认是否进入 GlobalCorrection 不能从本目录完全证明。
- `D5_CONFIDENCE = high` 对矩阵计算、左乘、XYZ/Rz 实现；`low` 对产品命名和默认启用状态。
- `D5_OWNER_DECISION_STILL_REQUIRED = yes`：需负责人确认矩阵的产品方向/坐标契约，以及是否继续只修正 Rz。

## D6_THREE_COLUMN_W_EVIDENCE

### D6_SUPPORTED_W_FORMATS / D6_COLUMN_DEFINITIONS

`GetData_S` 的活动解析函数位于 `SWv1/OnlineCor.cpp:829-917`、`926-1014`：读取前三列 XYZ，随后尝试读取 RX、RY、RZ、S1、S2 和第九列 time；实际保留的只有 XYZ，第九列首次非零时仅用于 `StepTcpV = distance/time`。没有列数校验或显式格式版本。

目录内实际存在 3、6、8、9 列 `_W` 文件，例如：

- 3 列：`SWv1/data/Robotleadingdata/WorkPiece/FJTESTOUT_W.txt`；
- 6 列：`0713fangzhentest_W.txt`；
- 8 列：`20211101test_W.txt`；
- 9 列：`20260404CHUFENGKOU1_W.txt`，样例为 `x y z rx ry rz s1 s2 time`。

因此 `D6_SUPPORTED_W_FORMATS = 源码容忍至少三列，历史数据实际出现 3/6/8/9 列；没有正式格式枚举`。这是文件样例事实，不等于每种格式都经过产品验收。

### D6_THREE_COLUMN_CALLERS / D6_FIRST_O_MISSING_FIELDS_SOURCE

- `_W` 的活动调用者是 `OnlineTeachTrajectoryInitial -> GetData_S`（`SWv1/OnlineCor.cpp:2223-2235`）。三列只提供几何 `_W`，足以进入最近点截取，但 `_W` 的姿态和外轴不会进入 O。
- O 的姿态和外轴来自 `_T` 的 `getData_R`，不是 `_W`（`SWv1/OnlineCor.cpp:3819-3825`、`3646-3721`）。因此三列 `_W` 缺失字段不会由 `_T` 在 `_W` 解析阶段补齐；它们完全绕过 `_W` 的 O 生成。
- 若 `_T` 缺少 RX/RY/RZ/S1/S2，`getData_R` 对空字符串调用 `atof`，结果为 0；源码没有拒绝、继承段首或姿态连续性门禁。`D6_FIRST_O_MISSING_FIELDS_SOURCE = _T/getData_R 的零值副作用，非正式 fallback`。

### D6_FAILURE_OR_FALLBACK_BEHAVIOR / D6_PRODUCTION_DEPENDENCIES

- 解析失败或空文件没有明确错误返回；`D` 为空时仍访问 `D_XPOS[0]`，属于未验证的异常路径（`SWv1/OnlineCor.cpp:893-901` 等）。
- 生产在线纠偏依赖 `_W` 的 XYZ 几何和 `_T` 的姿态/外轴；三列 `_W` 在几何上可被读取，但无法证明任何具体产品工件正式依赖三列格式。文件目录同时含大量 6/8/9 列历史样例。
- `D6_FAILURE_OR_FALLBACK_BEHAVIOR = NOT_PROVEN / zero-or-undefined behavior observed`。

### D6_FORMAT_HISTORY / D6_ACTIVE_OR_INACTIVE / D6_CONFIDENCE / D6_OWNER_DECISION_STILL_REQUIRED

- `D6_FORMAT_HISTORY = NOT_FOUND`：无 Git 历史，人工文件日期和代码注释不足以证明 3/6/8/9 列格式的引入版本。
- `D6_ACTIVE_OR_INACTIVE = ACTIVE_PRODUCTION_PATH` 对 `GetData_S` 的前三列读取；具体三列工件是否为当前生产配置，`NOT_PROVEN`。
- `D6_CONFIDENCE = high` 对读取/丢弃字段的源码事实；`low` 对格式产品契约。
- `D6_OWNER_DECISION_STILL_REQUIRED = yes`：需定义 `_W` 正式 schema、缺失姿态/外轴的拒绝规则，以及是否允许仅几何 `_W`。

## COLLISION_PHASE_EVIDENCE

在提供目录中没有找到 `FINISHREVERSEDET`、`ENDDETPRG`、`STARTCOR` 或可把 `P001/P002` 与 O payload 对齐的 JBI 运行日志。`SMOVC` 只出现在坐标规划器的模式枚举/编辑逻辑中，不能证明在线纠偏碰撞阶段。

可确认的边界只有：`s_offline == 4` 触发 O 更新（`SWv1/RobotCtrl.cpp:318-341`），在线跟踪后在 `traflag == 1 && ImgPro_type == 3 && s_online != 1` 时发送 P（`SWv1/RobotCtrl.cpp:267-297`），`CorrectTaughtPathO` 在 `robotleadingDlg.cpp:4738-4746` 被调用。

```text
COLLISION_PHASE_NOT_PROVEN
```

不能由现有源码证明事故发生在 MOVL P001 阶段，也不能证明 P001/P002 与哪一个具体 O/Pvar 一一对应。

## CONFIRMED_LEGACY_SEMANTICS

1. 默认 `InitType` 的初始点是反向检测点集最后一个点 `D_initdata.back()`。
2. `_T` 第九列参与当前 `Online_sergement` 的 `startID/endID` 选择；缺少持久化 `_W` 范围映射。
3. `_T` 段首尾通过全量 `_W` 的离散最近点切出几何段，复制时包含两端。
4. 当前活动 ICP 初始匹配分支不是 `initPtMatchType==1`；该值在函数内硬编码为 0。
5. O 更新来自 `_T` 段的姿态/外轴和修正后的 XYZ，按 `P_countId` 逐个发送；在线连续偏差通过 P payload 发送。
6. 正式 GlobalCorrection 路径对 XYZ 使用齐次矩阵/统一平移，对姿态只叠加 Rz；S1/S2 保持 `_T` 值并转换坐标系。
7. `_W` 活动读取只保留 XYZ，历史文件实际出现 3/6/8/9 列；没有正式 schema 校验。

## INACTIVE_OR_EXPERIMENTAL_SEMANTICS

- `initPtMatchType==1` ICP 分支：代码存在但当前局部变量硬编码不可达。
- `testCorFlag == false` 的“直接用检测点和当前姿态构造 P1”分支：硬编码为 true，不是当前活动路径（`SWv1/OnlineCor.cpp:3830-3864`）。
- `UseSimPriorCorrectionFlow()` 与 `ApplySimPriorPoseRotation()`：由 `g_simPriorCorrectionFlowMode==1` 控制，是仿真先验姿态路径，不应当冒充正式 GlobalCorrection 语义。
- 2024 年旧的按拐角/中间示教点分段、局部最近点限制和反向末点搜索均保留在注释中；2025 年活动代码明确改为单段全量最近点切片（`SWv1/OnlineCor.cpp:2262-2304`、`2357-2389`）。

## CONTRADICTIONS_BETWEEN_VERSIONS

1. 注释中的旧实现曾计划用 `_T` 中间点将 `_W` 分成多段并限制搜索区间，但活动代码在 2025 注释中标为“不进行分段”，改为全量 `_W` 两次最近点搜索。
2. 注释中的 `initPtMatchType==1` 计划通过反向检测轨迹匹配并寻找末点；活动实现把该末点查找注释掉，并且默认永远走类型 0。
3. `correctTeachPts` 的常规分支仍存在，但 `testCorFlag` 恒为 true；实际活动路径不使用“检测点 + 当前姿态”直接构造 P1 的方案。
4. `_W` 文件格式随历史样例变化（3/6/8/9 列），但解析器没有版本字段或列数契约；不能从文件共存推导产品正式格式。

## MISSING_EVIDENCE

- 无 `.git`，无法确认任何行为的 commit/tag/分支差异。
- 无在线运行日志能把 `_T` 段、`_W` 索引、O payload、JBI P001/P002 和机器人实际执行阶段关联起来。
- 无产品配置能证明 `GolabalCor` 是否在真实生产入口被设置；当前构造函数默认是 `InitType`。
- 无失败测试或设备记录证明 mapping 缺失时系统应停止、回退还是禁止发送 O。
- 无证据证明自交/重复焊缝、多候选、退化线段的安全策略。
- 无证据证明三列 `_W` 的具体工件属于当前生产配置，也无格式引入历史。

## OWNER_DECISIONS_STILL_REQUIRED

1. 定义 `_T -> _W` 的正式连续路径映射、区间边界、方向和失败处理；不能沿用无门禁的全量最近点作为新安全契约。
2. 决定初始点匹配是否继续使用 `D_initdata.back()`，还是采用连续投影/方向/距离/跨度门限。
3. 明确 O 更新的首点来源、已通过路径的跳过规则、姿态/外轴连续性阈值及拒绝条件。
4. 正式命名并确认 GlobalCorrection 的矩阵方向、输入/输出坐标系、姿态组合方式及是否只允许 Rz。
5. 定义 `_W` 的正式列 schema，以及缺失姿态/外轴时的拒绝策略。
6. 补充 JBI/现场日志关联，才能判断碰撞阶段和 P001/P002 因果。

## IMPLICATION_FOR_TRACK_A

Track A（产品/算法语义）目前 `NO_GO`。D2 的映射失败行为、D3 的候选规则、D4 的进度和连续性、D5 的产品变换方向、D6 的格式契约均至少有一项只能由缺失证据或推断得到，不能据此冻结新产品语义。

## IMPLICATION_FOR_TRACK_B

Track B（实现）可以继续做证据保全、日志和测试夹具，但不应在未完成负责人决策前把旧的“末点 + 最近离散段 + 整体平移 + 继续发 O”直接实现为安全行为。任何兼容实现必须显式记录上述缺口，并对映射失败、缺失姿态和空轨迹采取拒绝或停止策略，具体策略仍需负责人批准。

## GO_NO_GO_FOR_IMPLEMENTATION_PLAN

```text
NO_GO
```

理由：D2-D6 的活动代码路径已核对，但不是所有核心产品语义都被旧工程正式定义；多个关键点只有缺陷行为、实验分支或推断，没有足够证据进入新的实现计划。该报告不证明历史现场碰撞因果。
