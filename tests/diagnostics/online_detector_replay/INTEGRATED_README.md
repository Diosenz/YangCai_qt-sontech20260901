# 单入口组合离线验收

任务10公开边界已由用户确认：图像manifest JSONL经组合CLI到报告；同一工件坐标、冻结路径版本和显式沿缝位置关联后续TCP。测试不假造生产接口或运动。

运行demo：python3 tests/diagnostics/online_detector_replay/make_integrated_demo.py --output output/analysis/integrated_demo_new

运行测试：python3 -m unittest discover -s tests/diagnostics/online_detector_replay -p test_integrated_replay.py

复现：python3 tests/diagnostics/online_detector_replay/integrated_replay.py manifest.jsonl --context context.json --tcp tcp.jsonl --evaluation evaluation.json --policy-config policy.json --control-config control-config.json --output output/analysis/integrated_repeat_new

所有输出必须是新目录。--cache可复用macOS辅助缓存。无新增依赖、无CMake/MSVC构建、无生产修改。

## 数据流

一个组合合成会话连续经过正常、预焊附加弧顶、单侧母材、亮反光竞争、同曝光参考桥接、参考缺失及恢复。真实CLegacyOnlineKcfDetector::process及生产投影图像分支通过既有replay.py --evidence运行；EvidenceSession开启身份、母材和恢复门禁。再调用ValidationReplaySession和ControlReplaySession，实际归档母材后验、控制跟随器及可运行时的轨迹ICP保留真实状态。

evaluation的真值和scenario只进入评分端。独立校验candidate_point_by_frame是待检模型候选，不是焊缝真值；demo从独立模型线按显式材料s给出，不从evaluation读取。未定残差门槛保持threshold_pending。辅助估计、预测和恢复过渡均无空间/控制资格，不能借统一KCF名称进入独立校验。

## 关联契约

manifest每条记录可提供唯一observation_id及material_position：coordinate_frame=PositionerTcp、workpiece_frame、path_id、整数path_revision和有限s_mm。s由调用者提供，不从TCP位移、XYZ或真值推导。TCP JSONL提供record_id、kind、timestamp_ms、material_position和positioner_tcp_mm。

只接受同kind、同坐标系、同工件、同冻结路径版本、完全相同显式s且严格晚于曝光的唯一TCP记录。多次经过不能自动挑选；缺条件保持null。不用最近折线、最近时间、逐行相减、同时刻坐标或隐式插值。reference_to_tcp_distance_mm仅为同材料位置执行差异，不是焊缝准确性，也未宣称横向分解。

像素、原始投影、最终KCF、释放参考、独立资格及TCP关联在trace每条记录内共享identity。无材料位置或曝光时间不猜关联。真实14591/15248各自单帧会话，时间、版本、空间姿态及材料位置未知保持null，不拼成连续预焊序列。

固定基准430aaa2中有真实TCP归档。报告读取20261023_194602_142的actual_robot_path.txt、online_follow_targets.txt、vision_diagnostics.tsv及session_summary.txt，记录条数、表头和SHA256。缺逐曝光/材料位置映射及独立焊缝真值，因此不做现场精度评分；TCP永不是真值。

## 输出及限制

index.html提供逐帧表及原图viewer；report.json保留全输入、分层结果、失败与资格；report.md列生产/Windows待办；reproducibility.json记录生产基准、当前revision、源码/产物哈希及依赖。可搬迁manifest、原图、policy、context、evaluation及tcp在输出目录；复现到新目录，不覆盖历史证据。

分母包含首锚确认、失效、恢复及困难帧。有来源真值误差与无真值诊断分开，无truth kind不生成准确性数字。预算内预测仍可能偏离；恢复原始测量与过渡误差分开。轨迹ICP最近折线差异仅诊断，不当同材料真值。demo弦偏差2mm为调用真实快照的显式离线候选，不是现场允许值；释放实测点保持原值，弦偏差另报。

现场最大横向偏差待定。真实连续数据、曝光姿态、沿缝计量、独立真值、估计点执行策略、完整队列/连续PID及Windows VS2022/MSVC/OpenCV3.4验证未完成。交付不含生产集成、设备运动或焊接验收。

合成material s及weld_progress是调用方给定的配对标签/冻结参数，用来测试关联契约，不是由3D曲线验证的真实弧长。41帧是相连策略压力输入，不是物理相机-工件-机器人连续仿真；未实现现场沿缝计量器。即使外层context声明synthetic_fixture，真实/名义投影仍不能获得合成空间控制或独立校验资格。
