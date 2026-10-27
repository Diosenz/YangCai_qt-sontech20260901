# 03 三模板训练中的等价 FFT 复用

2026-10-27，代码起点 `df77d22`。

## 调整位置与行为

`Vision/src/track/KcfTracker.cpp` 的三模板 `computeKernels` 原来分别计算12组核相关，同一特征在不同核中重复做FFT和平方范数。现在一次调用内对三模板的HOG/灰度六组特征各计算一次频谱与范数，供该次核相关复用。通道求和顺序、共轭方向、shift、浮点转换、归一化、高斯函数和后续训练优化器保持原实现；尺寸条件不满足仍走原核函数。

缓存是函数局部值，返回即释放，不跨曝光、ROI、参考版本或会话。未调整ROI、模型、可信判据、训练条件/频率、滤波、控制参数或80/60μm限幅。代价是该次调用保留六组频谱的短生命周期内存。

## 测量结果

固定实拍3937图，经公开 `--tracker-benchmark` 入口，除首轮外为三模板；同配置独立运行两次，排除首轮后取中位数：

| 配置 | 优化前训练 ms | 优化后训练 ms | 优化前 update+train ms | 优化后 update+train ms |
|---|---|---|---|---|
| Debug第1次 | 220.2193 | 179.5377 | 324.5068 | 282.5556 |
| Debug第2次 | 213.7127 | 181.5850 | 316.1328 | 284.5520 |
| Release第1次 | 54.6251 | 35.3046 | 77.8060 | 58.7441 |
| Release第2次 | 54.3273 | 36.0931 | 77.8445 | 59.2677 |

Release训练耗时降低约34～35%，update+train降低约24%。输入图也作为替代参考外观，结果不代表现场仿真参考正确或连续吞吐已经达标。对17帧完整process的Debug/Release前后逐字段比较无差异；该夹具主要走单模板，完整process耗时基本不变，不能拿三模板收益代替所有帧收益。

可复核原始报告：`output/analysis/online_correction_optimization/baseline-isolated/tracker-{debug,release}.json` 和 `fft-reuse-comparison/tracker-{debug,release}.json`；计时开关对照为后者目录的 `comparison.json`。去除计时字段后前后结果无差异。

## 验证与审查

- 新 `test_equivalent_training_performance.py` 先RED：原实现训练中位54.63ms，超过本机Release固定负载50ms预算；新实现GREEN约34.55ms。该预算是本机回归条件，不是现场10Hz验收。
- 对照已归档训练结果：位置误差小于0.001px，响应/PSR/双特征权重容差abs=1e-6、rel=1e-7；核数及训练提交保持一致。
- Debug与Release诊断目标均构建成功；正式Release程序、运行时与记录器测试目标经VS2022 Developer命令行构建成功。程序为 `out/build/qt-release/UI/i2r_rlc.exe`，未启动设备。
- Release运行时专项9项全部通过：结果交付、正常会话、参考切换、重复曝光、新曝光、重启、强反光、严重失测、带偏差恢复；记录器测试通过。证据 `fft-runtime-regression.json`。
- Release离线全量103项：90通过，7失败/6错误均为既有SurfaceReplay固定旧源码与当前 `LocalizationSurfaceValidation.h` 不匹配。未放宽校验或改动该算法；不能表述为全套通过。证据 `fft-full-regression.log`。
- 规范与规格两路审查均无可行动问题。

现场正常段固定10秒有效新曝光率、曝光到运行时结果P95、空间间距与持续缺口仍待新Release连续采集。本项完成等价优化及离线验证，尚未证明现场10Hz/200ms达标。
