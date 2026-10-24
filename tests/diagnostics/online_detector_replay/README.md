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

此阶段尚未实现续接、恢复与模板回滚。生产检测器仍按原规则运行；
业务后置拒绝不意味着撤销了原始KCF训练，不能声称任务06已实现。
