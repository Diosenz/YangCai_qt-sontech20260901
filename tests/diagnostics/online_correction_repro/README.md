# 在线纠偏诊断回放

这是保留用于问题诊断的独立回放入口，不属于生产代码或默认 CMake/CTest 门禁。
基线失败是需要捕获的现场现象，不表示已批准改变生产验收策略。

## 运行

Windows 原生程序及环境变量见 [Windows 说明](../online_detector_replay/WINDOWS_README.md)。
未显式选择原生程序时，仅 macOS 可使用已安装的 clang++、pkg-config、OpenCV 4、Qt6Core 和 Eigen辅助编译。
Windows 在 VS 中预先构建；回放脚本不调用 MSVC/Ninja、不访问设备。
Windows 生产构建、测试与运行仍应通过 VS2022 完成。

从仓库根目录运行：

```sh
python3 tests/diagnostics/online_correction_repro/verify_replays.py
```

该命令核对所有证据及单变量探针，成功时自身返回 0；生产基线仍为 RED。
单独复现基线（预期退出码 1）：

```sh
python3 tests/diagnostics/online_correction_repro/replay.py --case field
python3 tests/diagnostics/online_correction_repro/surface_replay.py
```

退出码 1 表示捕获到测量缺失或写后几何未确认，2 表示输入/证据/回放错误。
最小化入口分别增加 `--minimized`：保留算法的 201×121 实际搜索窗，或保留触发
后验拒绝的三份坏帧。进一步删减搜索窗会改变生产配置；本入口不声称逐像素最小化。

## 证据边界

- 源码、图片、点云全部通过 Git 固定到 `430aaa204fd5319dd7f697d3de2b4e1faddbb06f`，
  保存原始字节及 SHA256，不依赖当前工作树是否更新到该提交。
- 原图来自 ICP 尝试 10726 的归档，内部来源帧为 **10653**。搜索提示使用该帧
  逐帧表中的 KCF 原始坐标 `(443.790314,609.482733)`。
- 10689 帧没有保存原图；它的逐帧记录单独保存，不冒充完整相机重放。
- 精定位入口调用原始 `COnlineSeamRefiner::refine` 和灰度重心工具；没有重放整个
  KCF 时序、扩大搜索调度、UI 或机器人控制。
- 表面入口实际重算 `evaluate`、`measure`、`compareFreshSurfaceWithCurrentReference`，
  再交给原始 `COnlineLocalSurfaceVerification`。输入使用已归档的 PCL 候选；
  不重新求解 PCL，也不重发 RobotSim 位姿事务。
- 20%→60% 的歧义探针、70%→40% 的覆盖探针仅用于因果区分，未修改生产参数，
  不能视作已经验证的修复。

结果、编译产物及字节一致的源码快照保存在
`output/analysis/online_correction_repro_20261024/`。该目录明确保留为诊断证据，
不提交构建产物或自动发布到远端。
