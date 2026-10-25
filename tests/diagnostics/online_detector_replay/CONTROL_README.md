# 实测 KCF 主导控制参考：独立离线回放

这是任务 09 的离线参考释放适配器，未改生产 Control、Vision、UI。
Python 的 `ControlReplaySession.replay` 消费任务 03/04 的公开证据结果；
C++ 调用真实 `COnlineCorrectionPathFollower::updateReference` 与变位机运动学，
输出首周期局部参考和预计 P 增量，不包含机器人通信或发送接口。

## 已实现行为

- 只有新曝光、geometry、independent_geometry、measured、已确认锚点且有效的
  工件坐标点可以释放。预测、相似度、重复/倒序曝光不能靠 available 位取得资格。
- 保留完整原始证据（含像素/原图测量）、原始实测 3D、释放点、每点处理原因。
  默认释放点与实测一致。处理提案超限时保留原实测并记录拒绝，模型不能默认替换。
- 处理允许改变量 `max_processing_displacement_mm` 和局部参考弦偏差
  `max_reference_chord_deviation_mm` 均为显式、未经现场标定的候选参数，默认 0。
  超出弦偏差的曲线不被静默用端点直线替代，拒绝生成该快照的 P。
- 释放点是 PositionerTcp，加入一次工装 TCP 偏移变成末节坐标后交真实跟随器。
  教点输入明确为末节系。跟随器根据当前外轴和反馈转换，得到局部垂足、前向垂足及 P。
- 模型线对照使用调用者提供的局部模型线，同一跟随器新实例计算。
- 输入足够样本及模型时，另行执行真实 `COnlineCorrectionExceptionProcessor::process`
  （flush=true），调用生产 `CIcpRegistrationMatcher` 与 libicp；保留 ICP 尝试/应用标志、
  拟合矩阵、RMS、原始点及模型替换释放点，并把释放点交相同真实跟随器新实例计算 P。
  `model_counterfactual` 为直接模型线快照，`actual_icp_baseline` 才是原轨迹 ICP 路径。
  此对照不包含在线控制器完整生命周期、机器人通信或连续执行。
- 模型折线最近距离仅描述几何差异；不逐行对应，不冒充同沿缝位置真值误差。

## 输入与范围

命令行读取检测证据 JSONL 与控制上下文 JSON：

```sh
python3 tests/diagnostics/online_detector_replay/control_replay.py evidence.jsonl context.json --output control-result.json
python3 -m unittest discover -s tests/diagnostics/online_detector_replay -p test_control_replay.py
```

上下文必需项：`coordinate_frame: PositionerTcp`、`calibration_valid: true`、
`tool_tcp_in_last_link_mm`、`station_base_translation_mm`、`robot_pose_mm_deg`、
`axes_deg`、`teach_last_link_mm`；可选 `model_positioner_tcp_mm`。
`spatial_provenance` 目前仅接受 `kind: synthetic_fixture` 加明确的 `frame_ids`。
未知或仅 nominal 的真实曝光不能因布尔有效位被用于控制验收。
当前只验证明确合成几何契约，不接纳现场曝光进入参考释放；真实投影标定与曝光关联
须由后续正式接入验证。因此回放结果不构成现场精度或执行安全声明。

局部参考快照采用首、末已验收点，必须满足候选弦偏差限制；不是完整队列消费、
实时目标选择、连续 PID 历史或焊接执行。C++ 对照默认使用生产的控制参数；
打印的 P 是该快照全新跟随器的首周期本地结果。

macOS 使用现有 clang/Qt6/Eigen 辅助编译，不安装依赖，不替代 VS2022/MSVC 验证。
构建以源码和依赖标志散列缓存。测试含真实图像检测入口 → 证据会话 → 控制参考，
以及独立闭式 90° S2、231mm 工装偏移、非零基座平移算例；真值不进入决策。

## 真实轨迹 ICP 对照验收

新增算例使用16个跨度15mm的非共线曲线实测点，原异常处理器每三点抽样后仍有6点，
调用真实process且flush。实测点的交替法向偏差构成模型无法用刚体完全吸收的变化；
不以该构造判断哪条曲线是现场真值。
ICP实际应用，原实测到其释放折线最近距离最大约0.393316mm；新的实测主导释放保持原值。
两路局部垂足不同，但首周期P受量化影响均为[-1,1,0]μm，这不构成轨迹相同或执行误差保证。
更多点/模型不足时，真实原处理器也可能不尝试或拒绝ICP，回放如实保留状态。
