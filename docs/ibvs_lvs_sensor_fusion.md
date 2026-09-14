# IBVS + LVS 传感融合框架说明

## 当前边界

本轮实现只覆盖 IBVS 模块内的纯算法框架和离线验证。LVS 观测被建模为 `IbvsLvsObservation`，由上层或后续适配层把 `Vision`、`DigitalTwin` 或真实采集结果转换进来；`IBVS` 公共头文件不直接包含 `Vision` 类型。

起始点融合输出仍是现有 4 维 `desiredFeature`，继续复用 J0/OLS、KF/Jacobian 更新和 `IbvsVelocitySolver`。完整 LVS 贴缝控制由 `IbvsUnifiedVelocitySolver` 承载：它把 4 行图像任务、3 行 LVS 空间约束和可选 XYZ 位置范围约束堆叠成统一阻尼最小二乘问题。本轮仍不接真实相机线程、RobotComm 或在线闭环。

## 融合策略

`IbvsSensorFusion::fuseDesiredFeature()` 先校验红色双特征点观测。红点无效时返回 `INVALID_ARGUMENT`，不进入速度求解。

LVS 观测满足置信度、中心线质量和最大帧差要求时，融合层优先使用 LVS 起始点；若起始点不可用但焊缝点有效，则用焊缝点作为起始点先验，再调用现有 `computeDesiredFeature()` 生成期望特征。

LVS 缺失、低置信、过期或几何计算失败时，融合层降级到调用方传入的 fallback `desiredFeature`，同时输出 `fallbackReason` 和放大的 `channelSigmasPx`，使速度链路可以继续运行并保留诊断信息。

旧项目 `computeSeamError()`、`computeLVSJacobian()` 和 `computeUnifiedVelocity()` 已迁移为 IBVS 内的纯算法接口。工件相关的 `H1/H2/h2/u_bar` 手调值不写死在几何计算中，而是通过 `makeLegacyWorkpieceFeatureGeometryParameters()` 返回旧项目最终出风口焊接姿态 preset；调用方仍可切回自动方向和自动 `h2`。

## 后续扩展

真实在线接入建议放在上层服务或流程编排层：相机/视觉模块负责产出激光中心线和焊缝特征，适配层转换为 `IbvsLvsObservation`，IBVS 继续只消费自有值对象。完整融合控制若要使用空间点、线结构光 3D 约束或多源可变维观测，应另行扩展观测模型、Jacobian 和速度求解器。
