# 在线检测器图像分支回放

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

任务04已加入下述受限续接；恢复身份验收与模板回滚尚未实现。生产检测器仍按原规则运行；
业务后置拒绝不意味着撤销了原始KCF训练，不能声称任务06已实现。


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
原始生产结果完整保留在 `raw_detector`，其内部训练仍未被撤回。

演示另有预算内误差反例：真实检测锚之后，隐藏的合成接缝运动与参考相对运动不一致，
预测仍在预算内但误差约12.36px。真值只进入独立评估器，未传给检测/策略。
这证明预算合格只意味着有限续接条件满足，不能作为位置准确或可执行的证明。
