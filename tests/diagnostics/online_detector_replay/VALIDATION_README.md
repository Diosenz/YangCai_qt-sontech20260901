# 独立焊点资格与母材后验：离线定位验证入口

任务08增加公开 `ValidationReplaySession.replay(evidence, context)`，消费已有真实检测
→证据会话的结果，不改生产 UI、Localization、Vision 或控制流程。
生产 surface 接口不接受焊点；这里明确新增离线焊点资格与候选点检验适配器，
并实际执行既有母材后验入口，不宣称已经接入线上定位验证。

## 可观察行为

- 每条记录保留原始证据，检查实际原图几何验收、几何来源、原始曝光身份及顺序。
  `image_geometry_eligible` 只表示这张原图有独立几何证据，不能单独获得空间校验资格。
- 工件空间焊点纳入另需 measured/geometry/independent_geometry、可靠同曝光锚点、
  原始与最终3D一致、有效空间上下文和同曝光候选位置。
  `available` 或 `can_independent_validate` 为真本身不构成资格。
- 相似度、母材辅助估计、纯预测、恢复过渡不能承担独立焊点检验，即使其位置数值
  与可靠实测相同。按曝光给定的候选模型点只作为待检验对象，不用来证明测量独立。
- 默认焊点公差未知，已纳入记录返回 `threshold_pending`，不擅自设定现场阈值。
  显式离线候选公差可产生 accepted/rejected；无合格记录返回 unavailable，不算通过。
- 所有输入都实际执行归档10466的生产 `surface_validation::evaluate/measure/`
  `compareFreshSurfaceWithCurrentReference` 和 `COnlineLocalSurfaceVerification::observe`。
  无合格焊点也不关闭该表面路径；焊点和表面结论分开保留，不发明整体自动晋升规则。
- 原归档4帧的配对点残差全部改善，但后几帧覆盖不足，后验不晋升。
  内核逐项验证重算与归档误差小于1e-7；输出展示的数字沿用原诊断stdout精度。

## 使用

```sh
python3 tests/diagnostics/online_detector_replay/validation_replay.py evidence.jsonl context.json --output validation-result.json
python3 -m unittest discover -s tests/diagnostics/online_detector_replay -p test_validation_replay.py
```

`evidence.jsonl` 可使用 `replay.py --evidence` 的实际结果。
context 包含 `coordinate_frame: PositionerTcp`、`spatial_provenance` 与
`candidate_point_by_frame`（曝光ID字符串→待检验候选3D毫米位置）。
`spatial_provenance` 目前只接纳 `kind: synthetic_fixture` 加明确 `frame_ids`。
因此真实图像即便有有效二维几何，nominal/未知投影也不能参与真实工件独立空间后验；
真实标定与曝光位姿身份适配仍未实现，不声称本轮获得现场空间精度验证。

`--max-weld-residual-mm` 为可选候选门槛；未提供就保留待定状态。
已有归档表面脚本通过Git固定版本materialize原始数据及源代码，并辅助clang编译。
不重新求解ICP，不写工件位姿，不发送运动，不安装依赖，不替代VS2022/MSVC验证。

测试覆盖相同几何但来源改变引起实际纳入/排除、合格焊点接受、所有估计家族隔离、
未知空间降级、重复曝光、归档覆盖反例、真实图像检测→证据→定位验证的贯通。
