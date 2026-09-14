# 定位模块 ICP 第一轮实现说明

## 目标

本轮只落地 `Localization` 模块中的两条定位能力：

- 全局受限自由度 ICP：对应旧项目 `updateModelOfflineICP()` 和 `stepICPforOffline()` 的算法语义。
- 局部滑窗 ICP 修正：对应旧项目 `VS_ONLINE_REC_MOD` 中累计真实/虚拟局部点云后由 `OnUpdateModelPosition()` 修正工件位姿的语义。

几何定位不在本轮范围内。旧项目 `updateGeoDataVec()`、`updateModelOffline()`、`vloc::computeLocation()` 只作为历史参考，不迁入当前实现。

## 旧项目依据

旧项目全局 ICP 的核心链路是：

```text
aRealPointList + virtualWeldment->position_face_dof_list
    -> updateModelOfflineICP()
        -> stepICPforOffline(...)
            -> 真实点云预处理
            -> 参考面点云生成
            -> IterativeClosestPointNonLinear + SwWarpPointRigid 	
	//IterativeClosestPointNonLinear 是 PCL 提供的非线性迭代最近点算法,可自定义的自由度约束
	//SwWarpPointRigid：根据传入的自由度掩码，只让允许的维度参与优化，被锁住的维度参数固定不变
	//之前 Weldment 结构体里，当前定位段对应的自由度位掩码 WarpType
        -> 更新 VirtualWorkpiece base pose
```

旧项目局部修正的核心链路是：

```text
drawVirtualLaserLine()
    -> generateVirtualPointCloud(...)
    -> updateRealPointCloud(...)
    -> 滑窗累计约 20 帧
    -> OnUpdateModelPosition()
        -> 受限 ICP
        -> 更新 VirtualWorkpiece base pose 和几何版本号
```

新项目保留这两条链路的输入输出语义，不保留 MFC 消息、全局变量、OCCT/VTK 显示对象、`VirtualWorkpiece` 直接写入或真实机器人命令。

## 当前实现边界

- `LocalizationTypes.h` 定义公开值对象：mm 点云、六自由度位姿、4x4 刚体变换、自由度 mask、ICP 参数和诊断信息。
- `LocalizationService` 是真实服务入口，通过 `runLocalization()` 按 `LocalizationMode::Global` / `Local` 分发。
- `LocalizationLocalCorrectionPipeline` 是局部定位模块内离线流程封装：校验窗口点云、组装 `Local` 请求、
  执行 ICP、取逆矩阵修正虚拟窗口点云，并输出前后残差。
- PCL 只出现在 `LocalizationService.cpp` 内部；公共接口不暴露 `pcl::PointCloud`。
- 受限自由度通过内部 `RestrictedRigidWarp` 实现，替代旧项目 `SwWarpPointRigid`。
- `LocalizationPointProjector` 提供离线图像点到 mm 点云的转换，承接旧 `updateRealPointCloud()` 的标定公式。
- 当前模块内已覆盖旧项目首次局部 ICP 快照的核心离线流程；UI 状态位、线程同步、
  `VirtualWorkpiece` 实际写回、虚拟激光重生成和几何版本号递增仍属于后续集成层工作。

## 真实回放数据说明

`data/IBVS/pointcloud/全局ICP` 中的 `firstForICP` / `firstAfterICP` 文件名来自旧项目
`OnUpdateModelPosition()` 的首次点云快照，实际对应局部/在线修正链路，不是
`updateModelOfflineICP()` 的离线全局分步定位输出。

旧项目保存该快照时的活跃 ICP 分支为：

```text
真实点云 -> FPS 保留 1/5 -> SOR(meanK=5, stddev=2) -> FPS 保留 1/5
虚拟点云 -> 不降采样
IterativeClosestPointNonLinear(real -> virtual)
SwWarpPointRigid((WarpType)39)  // x/y/z/rz 可动，rx/ry 锁定
transformationEpsilon = 1e-5
不显式设置 max correspondence distance
求得 real->virtual 矩阵后取逆，更新 virtual point cloud 和 VirtualWorkpiece pose
```

新项目为这类旧数据保留 `IcpPreprocessingProfile::CompatibilityFirstLocalIcp`，用于回放和
兼容性验证；默认 `Generic` 策略仍用于普通定位调用，避免把旧项目的激进采样策略强加给
所有全局/局部 ICP 请求。

## 数据与单位约定

- ICP 输入点云统一使用同一工作坐标系下的 mm 坐标。
- 旧项目通常把真实点和虚拟点最终整理到变位机 TCP 坐标系下再做 ICP；当前接口不强绑定具体坐标系，但调用方必须保证真实点和参考点同系同单位。
- `LocalizationPointProjector` 输入 `SENSORPARA.txt` 标定、图像像素点和工具位姿，输出 mm 点云。
- 当前投影工具的位姿旋转按 `Rz * Ry * Rx` 组合。旧 HALCON `create_pose` 的 `abg/gba` 轴序如需完全复刻，应只替换该工具内部矩阵构造，不影响 ICP 服务接口。

## 风险与后续

- 当前仓库缺少稳定的 localization 真实点云 fixture，因此第一轮测试使用合成点云验证算法方向、自由度约束、失败路径和投影公式。
- 旧项目中全局 ICP 曾使用极大迭代次数；新实现默认限制迭代次数，避免离线测试或 VS 构建卡住。
- 旧项目首次局部 ICP 的 FPS/SOR/FPS 预处理会显著改变参与配准的真实点集合。真实数据回放若要和旧 after 逐点对齐，应显式使用 `CompatibilityFirstLocalIcp`；若只验证定位效果，可继续比较修正后点云到真实点云的残差。
- `DigitalTwin` 仍是 facade/no-op 状态，局部修正暂不直接请求 RobotSim 虚拟点云。
- 后续接真实数据时，应新增 replay reader，并在数据缺失时输出 `SKIP`，不让普通 CI 硬失败。
