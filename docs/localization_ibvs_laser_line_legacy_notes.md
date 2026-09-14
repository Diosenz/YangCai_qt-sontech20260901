# 定位与 IBVS 激光线链路 legacy 摘要

## 目的

本文记录本次补齐 `Localization` 与 `IBVS` 激光线观测链路时参考的旧项目行为，避免后续继续依赖口头记忆或再次硬编码旧工程细节。

## legacy 关键链路

旧项目 `robotleadingDlg::virtualSceneThreadFunc` 的局部定位链路大致为：

1. RobotSim/虚拟传感器生成虚拟激光线。
2. 从虚拟焊缝点或虚拟线附近生成 ROI。
3. 根据虚拟激光线方向估计 Gabor 方向并预滤波。
4. 在 ROI 内执行 Hessian/Steger 风格中心线提取，阈值约为 64。
5. 将 ROI 内中心线坐标还原到整图坐标。
6. 将真实中心线按 stride 投影为真实点云，同时保留虚拟点云。
7. 按滑窗帧数累计局部真实/虚拟点云，超过窗口上限时丢弃最旧帧。

## 当前项目落地边界

- `Localization` 拥有自己的 `LocalizationLaserLineObservation`，负责定位侧的中心线提取、点云投影输入封装和局部点云滑窗。
- `IBVS` 拥有自己的 `IbvsLaserLineObservation`，负责视觉伺服侧的中心线提取和 `IbvsLvsObservation` 适配。
- 两套实现不互相依赖，算法步骤保持一致，后续可以分别独立演进。
- `Vision/` 和 `Control/` 不作为本链路的公共模块或依赖边界。
- RobotSim 只需要提供工件模型/OCCT 以及虚拟激光线、虚拟局部点云等原始仿真数据；工件选择、工艺参数、定位和视觉伺服策略留在主项目。

## 当前限制

- 中心线提取入口已实现，但真实在线链路仍需要相机帧、RobotSim 虚拟参考和标定/位姿在运行时组装。
- RobotSim 若尚未暴露虚拟激光线/点云命令，`DigitalTwin` 会返回明确 unavailable，不生成替代数据。
- 当前 UI 只读取默认工件 id，不提供下拉选择；后续加下拉框时只需更新当前 `workpieceId`。
