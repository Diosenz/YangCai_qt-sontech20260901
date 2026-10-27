# 01 性能基线实施与证据

日期：2026-10-27。源码起点：`8b523c769321218cc09f587dc307e017cf62e7ca`。

## 调整内容

| 入口 | 原行为 → 新行为 | 目的与边界 |
|---|---|---|
| `LegacyOnlineKcfDetector::process` | 无分段耗时 → 可选互斥阶段计时 | 只读诊断；默认关闭，正式在线随既有诊断开关启用；不改模型、训练条件或资格 |
| `RuntimeWorkerAdapters::processDetType2` | 只有处理起止 → 增加图像复制/灰度准备耗时 | 单调时钟，不以 PNG 写盘或 UI 时间替代算法时间 |
| `RuntimeCoordinator::routeTrackingResult` | 无接收时间 → 关联校验通过后记录接收时间 | 在同步状态通知和 UI 回调前采样，覆盖 worker 到运行时的交付等待 |
| `OnlineCorrectionRunRecorder::recordTrackingResult` | 会话 JSON 无 performance → 可选 performance 对象 | 未知时间/未测量阶段为 null；原 TSV 列顺序与 schema 9 保持兼容 |
| `performance_report.py` | 无公开统计入口 → 按预标注区间固定 10 s 窗口报告 | 预测、重复曝光、缺少当前图像支持分开；尾窗单列，缺时戳不填零 |
| 构建 | 只有现存 Debug 树 → 新建既定 `qt-release` 树 | 复用已有依赖，关闭 manifest 自动安装，未改共享 preset |

阶段 total 为生产 paper tracking 调用内的 steady_clock 时间；初始化含首次图像搜索和模板建立；simulation 含参考外观生成与提交；tracking 为响应计算；validation_projection 含几何、身份、恢复搜索和空间投影；training 为既有训练提交；bookkeeping 为结果状态整理。恢复中的重新初始化/响应单独归类，不双重计时。入口外的灰度准备、运行时等待与交付另计。

等待为曝光到 worker 开始的总等待，包含调度及参考同步，不能单独视为 RobotSim RPC 用时。epoch 时间为本机时钟，算法阶段为单调时钟；逆序序列报告为异常，不截成零。runtime_received 指 coordinator 收到并校验本结果，不能代表下游控制线程消费或设备执行完成。

## 同输入 Debug/Release 对照

环境：Windows 10，VS2022/MSVC，Qt 6.7.3、外部 OpenCV 3.4.3；完整标识及所有输入、源码、程序哈希见 `output/analysis/online_correction_optimization/baseline-isolated/comparison.json`。该组测量时未并行运行测试或构建。较早 `baseline/` 在回归测试并行期间产生，保留但不用于此表。

同一 `paper_tracking_20261027/online.jsonl` 的17帧稀疏实拍，两个独立进程重复；冷启动为每个进程首帧，稳定负载为随后16帧，包含难帧而不按检测成功筛选。

| 检测器 ms | Debug 两次 | Release 两次 |
|---|---|---|
| 首帧 total | 630.90 / 630.82 | 119.43 / 121.47 |
| 稳定 total 平均 | 297.18 / 297.22 | 59.10 / 59.77 |
| 稳定 total P95（nearest-rank） | 793.48 / 790.56 | 120.48 / 120.59 |
| 稳定 tracking 平均 | 103.15 / 103.53 | 23.46 / 23.60 |
| 稳定 validation_projection 平均 | 121.23 / 121.04 | 19.67 / 19.79 |
| 稳定 training 平均 | 72.55 / 72.40 | 15.61 / 16.05 |

该夹具缺完整仿真条纹，主要为单模板。独立三模板固定负载使用同一3937实拍图同时代替参考外观，四轮两次重复，记录为 `tracker-debug.json` / `tracker-release.json`。Debug update+train 约285.5～388.9 ms；Release约75.9～84.2 ms。Release训练约52.5～60.3 ms，训练是三模板负载的重要热点。该基准不代表实际仿真外观或持续现场曝光。

相同输入的所有离散输出一致；数值绝对容差1e-5、相对容差1e-7，Debug/Release及计时开关对照无超限差异。进程墙钟包含加载图像、PNG编码、输出与启动，不能与 detector total 混同。计时开关各两次：Debug全进程约6501～6518 ms，Release约1527～1579 ms；本次未见可分离的显著计时开销，不能将噪声内的负差称作加速。

## 自动验证

- 新性能快照测试曾在旧程序 RED（缺performance），新程序 GREEN；同三帧真实 process 验证计时开关不改位置、来源、资格及训练。
- 记录器新断言 RED 后 GREEN，验证已知起止、未知交付为null、无诊断整对象为null，不挪用记录器写入时间。
- 报告四个测试 GREEN：不同曝光去重、预测/无效/未知支持保留、完整10秒及尾窗、边界半开区间、逆序/缺失时间不造零延迟、图像准备分布、跨窗30秒长缺口及空间间隔。
- 新增 `--performance-delivery-contract` 精简运行时入口，覆盖单个在途和最新等待曝光的真实生产装配边界及接收时间。

规范审查1项（链式赋值）已修正；规格审查2项（跨窗缺口、图像准备汇总）均先补RED测试后修正为GREEN。运行时交付专项和记录器通过MSVC Debug原生测试。Release离线全量101项：88通过，13因既有surface_replay固定旧源码与当前 `LocalizationSurfaceValidation.h` 不一致被拦截；该文件及固定检查不在本次diff中，未放宽来源验证或改动表面算法。原始输出保留于 `regression-release.log`，不能将该轮描述为全量通过。

复现命令见三个工具的 `--help` 和已有显式原生构建清单机制；`benchmark_performance.py --debug-dir out/build/qt-debug/tests/diagnostics/native/Debug --release-dir out/build/qt-release/tests/diagnostics/native/Release --output <保留的新输出目录>`。

## 尚未现场验收

现有20261027会话已生成 `current-performance.json`：未预标注区间，不授予正常段验收；历史会话没有 unified_session.jsonl，只能沿用原TSV只读分析，不能伪造新schema的接收时刻。新的生产接收时间需下次人工运行后才有真实样本。

本项未证明现场10/15 Hz、曝光到业务P95≤200 ms、UI25帧/s或TCP改善。未调整ROI、训练频率、PID、标定或80/60 μm限幅；下一项优先交换实时主画面与同曝光快照，再对三模板训练减少重复运算作等价对照。
