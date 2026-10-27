# 在线检测器图像分支回放

## 2026-10-27 预焊后历史锁偏回归

`test_preweld_field_recovery.py` 通过生产 `process` 验证：保留旧历史后仍能恢复当前可见转折，
以及参考缺失、局部校正缺少原身份条纹支持时不能授予错误点控制资格。
`tests/fixtures/preweld_tracking_20261027/` 保存 1987、4727 两张未修改实拍原图及来源哈希。
这是稀疏历史对照，不是完整连续预焊重演；缺失仿真折线，投影显式借用 3377 曝光，
只验像素。20 px 是人工近似标注的故障回归容差，不是检测参数或毫米/TCP 精度公差。
在配置原生程序目录后，可执行 `python -m unittest discover -s tests/diagnostics/online_detector_replay -p test_preweld_field_recovery.py -v`。

## 2026-10-27 真实在线图像 TDD 回归

`test_online_field_bootstrap.py` 在公开 process 边界覆盖 2070/7264 错误首捕、
3801—4171 实拍曝光恢复，以及实拍 1712 图纵向翻转后的相反折点方向。原图、
来源哈希和显式借用的像素回归投影位于 `tests/fixtures/online_tracking_20261027/`。
不把稀疏归档视为完整连续相机会话，也不据此声称 TCP 或毫米精度。

同一测试调用公开 tracker benchmark 比较响应峰、PSR、双特征权重和位置与优化前
固定基线的一致性。3937 实拍图同时代替参考外观，仅用于稳定触发三模板计算负载，
不代表现场仿真真值。250 ms 是本机 Debug 训练中位数回归预算，不是算法门限；
应在无并行构建/回放负载时单独运行，硬件不同需重新建立性能基线。

```powershell
python -m unittest discover -s tests/diagnostics/online_detector_replay -p test_online_field_bootstrap.py -v
```

UI 的 `i2r_rlc_main_widget_panel_tests --live-preview-contract` 通过运行时 Qt 入口
验证真实原图刷新、检测曝光不被替换、帧号归属同步和返回普通预览后的清除。
设置 `I2R_UI_REPLAY_SNAPSHOT` 可保存离屏主图及小窗布局，不连接设备或操作鼠标。

Windows 的可选 MSVC/CMake 构建与运行见 [WINDOWS_README.md](WINDOWS_README.md)。
显式设置 `I2R_KCF_REPLAY_BIN_DIR` 后，全部入口只调用预编译程序，不自动编译。

单入口组合验收（任务10）见 INTEGRATED_README.md；通过 make_integrated_demo.py 生成相连场景、分层报告及真实归档TCP关联缺口清单。

此入口调用真实 `CLegacyOnlineKcfDetector::process`，一个会话内保留同一个检测器。
使用原生产 KCF、身份匹配、原图几何搜索和图像到空间投影，不调用运动设备。
它区别于 `online_correction_repro` 的单帧 refiner 回放，也区别于方案状态机原型。

## 范围

- 当前生产图像分支：image measurement、GapCenter、仿真仅搜索提案；不启用旧融合分支。
- `bounded_search=false` 为初始化完整搜索，`true` 为正式期间分帧搜索；每帧显式输入。
- macOS clang + 已安装 OpenCV4、Qt6Core、Eigen 仅作辅助验证，不构建 CMake/Ninja/MSVC。
  正式环境仍为 Windows/VS2022/OpenCV3.4.x；此处不承诺 ABI/数值完全相同。
- PCL 未作为依赖安装。`GapDisabledAdapter` **只**补足未启用的间隙链接边界；
  请求 `enable_gap=true` 会非零退出，误调用该边界会抛异常。
  不替代任何已运行的图像算法。未验证 PCL 间隙、UI runtime 外层门禁、控制或设备闭环。
- 公开诊断仅包含选出的 raw refinement，不包含所有内部搜索候选；不猜造候选列表。
  raw 有效不等于最终接纳，`image_observation_accepted` 和 `available` 分别表示图像接纳与投影后可用。
  `training_committed` 是原检测器实际诊断，未实现未来任务的身份、训练或预算策略。

## 运行

在仓库根目录执行，输出路径必须尚不存在，保留已有结果：

```sh
python3 tests/diagnostics/online_detector_replay/test_replay.py
python3 tests/diagnostics/online_detector_replay/make_demo.py --output output/analysis/detector_demo_run1
python3 tests/diagnostics/online_detector_replay/replay.py output/analysis/detector_demo_run1/synthetic.jsonl --output output/analysis/detector_repeat_run1
```

依赖从 `pkg-config` 发现，编译缓存默认在 `output/analysis/detector_replay_build`，可用 `--cache` 改位置。
无下载、安装或绝对开发机路径。测试的临时证据保留在系统临时目录，不清理文件。

## 会话输入 JSONL

2026-10-27 的论文路径诊断可显式设置 `paper_tracking=true`，直接调用当前生产
`processPaperTracking`；省略时保留既有回放行为。`detector_valid` 和
`detector_pixel` 始终保留检测器原始业务结果，`paper_diagnostics` 提供当前身份、
肩部匹配及响应统计。`probe_bootstrap=true` 仅额外读取当前图像初始化提案，
不改变检测器状态。现有归档没有完整仿真折线时，不能声称重建了三模板输入。

本机两轮现场诊断入口为 `retest_20261027.py` 和
`retest_shoulder_20261027.py`，使用现有 `I2R_KCF_REPLAY_BIN_DIR` 环境。
前者回放第一轮首曝光并检查左侧转折近似标注；后者复用实际生产肩部匹配函数，
从第二轮归档重建身份与历史预测，并校验输出与归档一致。后者还需要本机
`output/analysis/field_retest_20261027/` 内已编译的探针及源码哈希清单。
默认非零退出表示现场错误仍可复现；`--center response` 仅改变搜索中心，
另加 `--ranking response` 才改变候选排序参考。这些因果对照不表示生产程序已修复，
15 px 人工近似标注范围也不是精度验收。该窗口缺少更早的全部连续图像，不能冒充完整 KCF 会话回放。

修复回归使用 `test_paper_field_repair.py` 及 `tests/fixtures/paper_tracking_20261027/`：
覆盖首帧左转折、黑图不能污染初始实拍模型、后续窗口漂移、低灰度相关仿真仍可用及单模板焊前统计。
在设置上述原生程序目录后，执行
`python -m unittest discover -s tests/diagnostics/online_detector_replay -p test_paper_field_repair.py`。
前两张在线图缺少归档曝光变换，夹具显式借用后续曝光投影；这些断言只评价像素，
不用于空间精度或 TCP 评分。重复图像的34曝光统计测试是受控软件测试，不是34张现场连续图。
历史肩部探针的源码哈希随生产修复失效是预期行为；它仅保留旧错误的因果证据，
不作为修复后程序的验证入口。

一行一个曝光观测对象，按给定顺序处理，不插值、不补造曝光：

- `image`：相对 manifest 的原灰度图路径；明确缺图为 `null`。声明存在但读不到的图非零退出。
- `kind`：`real` 或 `synthetic`；未提供时输出未知。
- `frame_id`、`timestamp_ms`、`reference_revision`：可省略/null；归档缺失保留 null。
  检测器底层的 frame=-1、revision=0 兼容默认值不被包装成归档事实。
  帧身份和版本必须为 JSON 可精确表示的整数，拒绝小数身份及错误开关类型。
- `sim_point`：可选两个像素数值，仅作为输入仿真搜索提案。
- `seam_index`：可选接缝编号，未知传 -1。
- `bounded_search`：布尔值；缺省 false（初始化完整扫描），正式连续回放显式 true。
- `enable_gap`：只能 false/省略。
- `projection`：可省略，底层将拒绝缺失上下文，不构造有效空间点。
  真正回放三维需提供完整 `calibration`、`camera_in_tool`、`tool_in_robot_base`、
  `robot_base_to_positioner_tcp` 及 `provenance`。标定键见 demo 生成器；变换为9个行优先旋转和3个毫米平移。
  显式合成测试可用 `fixture=synthetic_identity_1mm_per_pixel`，禁止用于标为 real 的帧。

Demo 的两张真实 PNG 使用已存在 legacy 测试中的诊断标定和**声明为非曝光姿态的单位变换**，
目的仅让真实投影代码可运行，并不声称恢复原曝光链路或物理精度。
真实图分别是独立会话，不把稀疏图片拼成连续采样。归档时间和参考版本未知。

## 输出与复现

- `results.jsonl`：原始几何点、原始 KCF 点、最终点、图像/空间有效性、位置来源、可信状态、
  拒绝 message/status、几何质量、真实 KCF 峰值/训练诊断、曝光/版本和输入图哈希。
- `metadata.json`：实际 HEAD、基线 430aaa2、生产源相对基线的差异、逐文件 SHA-256、
  OpenCV/Qt/Eigen 版本和输入 manifest 哈希。工作树即使未提交，源哈希仍记录实际编译字节。
- `viewer.html`：可独立打开的逐帧图像与像素标记/诊断，PNG 原图内嵌，兼容 PGM 输入。
- `manifest.jsonl` 和 `frame-*.png`：可搬迁复放的完整输入，图像是 lossless 灰度 PNG。
  输入编码可能从 PGM/彩色 PNG 规范化为灰度 PNG；结果同时保留本轮原文件 SHA，
  不把 PNG 编码哈希相同作为像素行为一致的条件。

重复性比较最终坐标/来源/可用性/诊断，而不是编译时间或文件编码。
合成顶点已知，可检查像素误差；真实归档原验收值可作回归参考，不等同于真实焊缝物理真值。
TCP 正在跟随检测点，不能把 TCP 自身当作独立检测真值。

## 后续任务复用

`DetectorReplay.cpp` 是主要会话入口；后续行为在相同公开 process/result 边界验收，
不依赖内部调用顺序。当前固定生产源没有修改；复用时保留 baseline 元数据与 raw/final 分离，
不得把后处理日志标记误称为已撤销实际 KCF 训练。真实新帧资格和训练资格由后续任务实现。

## 可选离线证据会话（任务03）

增加 `--evidence` 可启用最终业务证据契约。默认仍输出原始生产基线。
`--policy-config` 读取显式候选配置，目前支持 `anchor_confirmation_frames`（默认2）。
确认帧必须是严格递增的有效曝光ID，独立几何由生产最终接纳结果决定；
仿真点缺失不拦截有效几何。重复/未知曝光、无效投影不刷新可靠锚点。
源结果完整保留在 `raw_detector`；`anchor`、`anchor_refreshed`、`can_independent_validate`
和 `confirmation_count` 是离线契约结果。参数未现场标定，首次确认期间会不可用。

任务04提供下述受限续接；任务06增加逐曝光身份验收与模板事务，详见后文。
每次生产 process 内部仍按原规则训练；原始训练事实不会被后处理改写。


## 有限续接（任务04）

仍使用 `--evidence`，两个预算必须在 `--policy-config` 中显式给出：
`max_prediction_ms` 和 `max_prediction_travel_mm`，均为正数。
默认 null **不启用预测**；例示数值只属于离线候选，不是现场公差。

```sh
python3 tests/diagnostics/online_detector_replay/test_evidence_session.py
python3 tests/diagnostics/online_detector_replay/make_continuation_demo.py --output output/analysis/continuation_demo_run1
```

除已确认独立几何锚点外，预测每帧需要：

- 严格新增曝光身份和严格递增的 `timestamp_ms`。失测年龄从最后可靠锚的原曝光计时；
  重复/倒序/未知曝光不增加确认次数，不输出可用预测，也不刷新锚点。
- `weld_progress_mm`：调用方给出的**当前曝光关联的工件沿缝弧长坐标**，单位 mm；
  `progress_exposure_frame_id` 必须等于本帧有效整数 ID，布尔值不算 ID。
- `progress_reference`：非空、稳定的计量参考 ID，必须标识工件坐标系、冻结焊缝路径及路径版本；
  参考变化会锁定距离未知，直到新的独立几何建立新锚。
- `progress_provenance`：非空来源声明，说明弧长从实际曝光姿态/外轴反馈及冻结工件路径如何得到，
  或说明它是明确的外部计量/合成契约。调用者负责计量器的正确性与帧关联。
  本任务**没有**实现从机器人反馈推导弧长的计量器。
  不接受用 TCP 端点净位移、配置速度、帧数、CAD 像素变化或真值像素当行程的解释。
- `reference_point_at_exposure`、`reference_exposure_frame_id` 和 `reference_revision`：
  当前参考版本在当前曝光姿态下的几何参考像素。它不是独立焊点。
  同版本只把相邻已知曝光的参考相对变化加入图像锚的预测。

距离累计相邻有效工件弧长增量的**绝对值之和**，所以往返不会抵消；
TCP 在机器人基座几乎不动而外轴转动时，应由上游工件相对计量提供非零行程。
演示使用明示的调用方进度输入，不声称由附带的 S2 数值实际算出了行程。

任一预算在 `>=` 时到限。缺时间/进度、时钟回退、路径参考变化会锁定相应计量失败，
后续预测或相似度结果不能清零；新的确认独立几何可另建锚。
`prediction_travel_known=false` 时累计数字只是缺口前后的已知部分，不能当完整行程。

换参考版本时，必须额外提供：
`previous_reference_revision`、`previous_reference_point_at_exposure` 和
`previous_reference_exposure_frame_id`。这是**旧参考版本在当前曝光同姿态下重新求得的几何**。
不能提供上一曝光的旧点冒充当前曝光旧点。
只累计“旧版本当前曝光点－旧版本上次曝光点”，再把新版本当前点作为后续参考基准；
绝不累计新旧绝对 CAD 位置差。没有这份同姿态桥接时，真实锚点保留，参考预测不可用。
桥接也不重置年龄和行程。参考丢失时该帧不可预测；同版本后续重新出现时，
可从保留的同版本参考继续计算相对变化，仍受原预算与计量缺口约束。

输出 `source=bounded_prediction`、`trust=estimated_not_independent`、
`prediction_elapsed_ms`、`prediction_travel_mm`、已知性、拒绝原因和参考映射诊断。
预测超出图像范围也不可用。此实现只输出**像素预测**：
`positioner_tcp_mm=null`、`space_available=false`、`control_eligible=false`，
不使用固定像素毫米比例造空间点，不给 TCP 执行资格，也不能参与独立焊点校验。
原始生产结果完整保留在 `raw_detector`；事务保留/丢弃与实际训练事件分开记录。

演示另有预算内误差反例：真实检测锚之后，隐藏的合成接缝运动与参考相对运动不一致，
预测仍在预算内但误差约12.36px。真值只进入独立评估器，未传给检测/策略。
这证明预算合格只意味着有限续接条件满足，不能作为位置准确或可执行的证明。

## 身份验收和模板事务（任务06）

`--evidence` 现在逐曝光建立短检测会话：重放上一个**最终确认的独立几何**原图及原曝光上下文，
再运行当前原图。当前帧只有通过曝光身份、投影、几何及同目标身份验收，并完成原有确认，
才晋升为下一曝光的可靠 checkpoint。默认 baseline 不使用该运输，仍是单检测器完整 manifest。
这是一种成本较高的离线事务，不是实时实现，也不等价于恢复全部历史 KCF 数值状态。

`raw_detector.training_committed` 保留真实训练事件；`temporary_training_discarded=true` 表示
该临时会话的训练没有保留到下一曝光，不能解释为原 process 没有训练。
`checkpoint_source_frame` 是本帧实际使用的旧可靠帧；
`template_retained_for_next_exposure` 只在最终独立几何晋升时为 true。
checkpoint 冷启动必须重新检测到原锚附近（候选4px以内），否则图像身份路径 fail closed，
仅允许仍满足旧预算的已有预测。所有输入与临时 manifest 都保留，不删除历史材料。

身份 helper 读取原图像素与历史确认 patch，在历史目标的局部运动走廊内匹配双侧形状；
几何候选同时需要历史距离、当前局部匹配、左右分支 patch 一致性及匹配唯一性。
可用同版参考只提供历史附近的相对运动提案，不能用大幅参考位移授权远端同形目标。
更亮或 NCC=1 的远候选仍会拒绝，不能通过连续出现累计夺取身份。
参考相对运动超出可验收身份走廊时，`reference_identity_conflict` 也阻止参考依赖预测，
保留旧锚和累计预算；冲突锁定到新的独立几何确认，不把换来源标签当作避免拖点。
这只表示超出该离线候选尺度后的不确定性过大，不断言参考运动在物理上必然错误。
首锚确认也要求连续候选身份一致；第一帧无先验目标时的初始化本身不证明母材语义。

外观输出额外需要真实 KCF 响应和 KCF 位置与历史局部匹配相容。
它标为 `source=appearance_tracking`、`trust=tracked_not_independent`，
只在原时间/行程/参考预算满足时输出像素，不刷新锚、确认次数、预算或 checkpoint，
没有空间点、控制资格或独立校验资格。身份冲突原因保留在 `identity_evidence.reasons`。

显式离线候选参数（均未现场标定）：

- `identity_corridor_pixel=12`，整数1～128；
- `identity_min_patch_score=.70`、`identity_min_branch_score=.55`；
- `identity_min_uniqueness=.04`（排除同一峰附近8px之后的分数差）；
- `identity_min_kcf_peak=.20`。

固定97×65 patch、左右各32列、候选与局部匹配4px一致性，也只是本离线设计的候选尺度。
当前未实现遮挡后的远距离自动换目标；无法区分的近邻同形纹理、真实形状突变或大运动可能拒绝，
预算内预测仍不保证精度。相似度不是目标语义证明。缺少真实连续遮挡数据，不能外推现场误切换率。

```sh
python3 tests/diagnostics/online_detector_replay/test_identity_continuation.py
python3 tests/diagnostics/online_detector_replay/test_identity_mother_recovery.py
```

测试保留可搬迁 viewer 与完整原图/manifest；可用 `KCF_IDENTITY_EVIDENCE` 环境变量指定证据父目录。
亮反光竞争、多个同形候选、正常运动、恢复与模板丢弃、首锚身份一致性、参考诱导和真值标签不敏感
均通过公开 `replay.py --evidence`，不 mock 原图、KCF 或 refiner。
外观演示另用显式 `.60` patch 候选参数，以容许交叉亮纹污染；该序列输出约3px偏差，
默认 `.70` 会更保守地降级预测。这不构成推荐阈值或现场精度结论。
