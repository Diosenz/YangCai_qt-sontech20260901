# KCF 生产接入核查记录 — 2026-10-25

源码已在当前项目工作区修改，未提交/推送。当前 Git 基线是 430aaa2；已将此前
b68614b 的回放源码、构建入口及 0.4 歧义门槛同步到该工作区。没有切换分支、
删除文件或修改 oldproject/RobotSim；已有未提交内容保留。修改前备份位于
output/backups/kcf_production_integration_20261025/。

## 实际接入范围

1. 正式主程序 Vision worker 显式启用 COnlineSeamEvidence；原图身份、母材线、
   新曝光确认先于 KCF 模板训练。重复和乱序不能重新建立锚点。
2. coordinator 在有冻结 RobotState、仅缺同帧仿真时仍投递原图。ImageOnlyReady
   让 UI 保持该曝光 pending/consumed 关联；离线扫描强同步策略保留。
3. 像素显示与独立几何/控制资格分开，估计不生成控制空间样本。黄色表示估计或
   控制参考显示交点；绿色只用于测量显示。
4. 正式 Control 启用 preserveMeasuredReference：实际释放点等于该批原始测量。
   拟合后的模型候选另存为 model_proposal_not_released。
5. replay.py --production 直接调用同一 C++ 策略；--evidence 仍为历史 Python
   对照，不能用来代替生产策略验证。生产不依赖 Python。

## 已实际执行的辅助验证

环境：macOS / clang++ / Qt 6.9.0 / OpenCV 4.11.0 / Eigen 5.0.1。
这与生产 Windows/MSVC + OpenCV 3.4.x 环境不同。

| 检查 | 结果 | 记录 |
| --- | --- | --- |
| 全部离线回归（当时含 11 项新生产检查） | 73 项，1 项平台跳过，其余通过 | replay_regression_final.log |
| 最终生产策略回归，含新 CLI | 12/12 通过 | production_policy_tests_final.log |
| 实际 CVisionTrackingWorkerAdapter 的 C++ 测试 | 通过，诊断关闭/无 CAD/恢复/重复曝光 | worker_tests.log |
| 实际 Control controller 全部回归及实测保留测试 | 通过 | controller_tests.log |
| 实际记录器及新增 schema 字段回归 | 通过 | recorder_tests.log |
| 原始因果复现探针 | 12/12 符合预期；固定旧生产基线仍为 RED | causal_probes.log |
| 五个改动的 UI/运行链源文件与相关契约测试 | clang 语法检查通过 | *_syntax.log |
| git diff --check | 通过 | 本次命令输出 |

新的生产检查覆盖原图无仿真确认、模板隔离、重复曝光、重捕获、时间/反向弧长
到限、参考版本桥接、行程证据缺失、标定投影失效、单侧母材约束和分帧搜索。
实际 worker 辅助测试编译真实 RuntimeWorkerAdapters.cpp，仅链接测试所需类，
不启动硬件。它不等于完整 coordinator/UI/机器人闭环运行。

## 仍需验证与明确限制

- 尚未在 Windows/VS2022/MSVC 生成并运行改动后的主程序。修改过的完整 runtime、
  frame sync、UI presentation 契约目前只完成语法检查，仍须运行 Windows targets。
- 生产 transport 缺逐曝光工件沿缝弧长和旧参考版本同曝光桥接，两个续接预算默认
  为零。因此生产只释放确认的原图实测，完整预焊遮挡连续续接尚未具备运行输入。
- 没有用合成时序证明现场毫米精度，也没有将稀疏真实原图冒充连续曝光。
  初始/恢复确认帧数、身份走廊及母材阈值仍须现场标定。
- 原有控制参考预测仍存在，本次仅区分显示及阻止估计点冒充新实测；没有重新定义
  缺测停机、减速或焊枪跨预焊凸起的高度策略。

Windows 请同步本次工作区改动后，在既有 Qt Debug 配置生成 i2r_rlc 及测试目标。
详细的行为与验证入口见 docs/online_cor_migration_mapping.md 同日章节，以及
 tests/diagnostics/online_detector_replay/WINDOWS_README.md 的 --production 说明。
