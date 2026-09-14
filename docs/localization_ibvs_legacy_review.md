# localization / ibvs 旧项目专项复盘

## 目的和结论

本文只复盘旧项目中实际参与 `localization` 和 `ibvs` 运行链路的内容，用于指导当前 Qt/CMake 项目的第一轮离线算法验证。复盘不等于迁移旧代码，尤其不把 MFC UI、线程函数、全局变量通信、RobotCtrl 发送命令、RobotSim/OCCT/VTK 场景刷新逻辑带入新项目。

结论如下：

- `localization` 第一轮应保留三类算法语义：几何特征全局定位、受限自由度 ICP 全局配准、滑窗式局部 ICP 修正。
- `ibvs` 第一轮应保留在线主链中的特征误差、雅可比初始化/更新、LVS 贴缝约束、统一速度求解、限幅/安全约束和收敛判断。
- `IBVS.cpp` 中很多函数属于测试、调参、诊断或 A/B 对照路径，不能默认进入新项目第一轮。
- 离线验证可以使用旧项目 `SWv1/data/IBVS`、`SWv1/data/Robotleadingdata` 和模型/点云数据，但新模块接口必须保持正式边界：`localization` 不暴露 PCL，`ibvs` 不直接调用机器人通信。

## 复盘来源

主要旧项目锚点：

- `oldproject/sontech-works-version-storage/SWv1/robotleadingDlg.cpp`
- `oldproject/sontech-works-version-storage/SWv1/robotleadingDlg.h`
- `oldproject/sontech-works-version-storage/SWv1/PicCapture.cpp`
- `oldproject/sontech-works-version-storage/SWv1/IBVS.h`
- `oldproject/sontech-works-version-storage/SWv1/IBVS.cpp`
- `oldproject/sontech-works-version-storage/SwBase/virtualLocation.h`
- `oldproject/sontech-works-version-storage/SwBase/virtualLocation.cpp`
- `oldproject/sontech-works-version-storage/Robotleading/robotleading_method/RStdAfx.h`

辅助参考：

- `docs/localization_visual_servo_digital_twin_scope.md`
- `docs/robotleadingdlg_dependency_map.md`

## 调用链

### IBVS 在线主链

```text
CRobotleadingDlg::StartIBVSSession(...)
    -> CreateThread(..., IBVSThread, ...)
        -> IBVSThread(...)
            -> PicCapture(cv_imageEx)
                -> 初始化 IBVS 参数、手眼标定、期望特征、雅可比
                -> 每帧提取特征 [u1, v1, u2, v2]
                -> jacobianEK / jacobianLinear / getCurrentJacobian
                -> 读取 LVS 焊缝约束全局状态
                -> computeUnifiedVelocity 或 computeUnifiedVelocityLegacy
                -> setNextPosition
                -> 写回 ibvs_x/y/z/rx/ry/rz 和 ibvs_vx/vy/vz
```

旧链路中 `robotleadingDlg.cpp` 负责启动线程和生成 LVS/局部定位观测，`PicCapture.cpp` 负责 Basler 图像采集、特征检测和 IBVS 控制计算，`IBVS.cpp` 承载数学求解器。新项目应拆成离线输入、`ibvs` 算法服务和 dry-run 输出，不保留 `CreateThread`、全局变量写回和真实机器人命令通道。

### 全局定位：几何特征方式

```text
真实扫描线 / 实际点云 aRealPointList
    -> updateGeoDataVec(posSegCount)
        -> vloc::processGeoData(...)
        -> geoDataVec[posSegCount]
    -> updateModelOffline()
        -> vloc::computeLocation(geoDataVec, virtualWeldment)
        -> VirtualWorkpiece->set_base_pose(...)
```

此链路的核心是从实际点云提取定位元件特征向量，再按工件模型中的定位面、定位元件和自由度约束迭代求位姿。新项目第一轮可把 `Weldment/OCCT` 模型侧输入替换成离线参考特征和约束描述，不把旧场景对象作为 public API。

### 全局定位：受限 ICP 方式

```text
aRealPointList + virtualWeldment->position_face_dof_list
    -> updateModelOfflineICP()
        -> onceICPforOffline(...) 或 stepICPforOffline(...)
            -> 生成/采样虚拟面点云
            -> 真实点云 FPS + SOR
            -> IterativeClosestPointNonLinear + SwWarpPointRigid
        -> VirtualWorkpiece->set_base_pose(...)
```

此链路使用 PCL，且通过自定义 warp 限制可运动自由度。新项目应把 PCL 保留在 `localization` 实现层，公开接口只表达真实点云、参考点云、自由度约束和位姿结果。

### VS_IBVS 下的局部定位

```text
robotleadingDlg VS_IBVS 分支
    -> VirtualLaserSensor->generate_simu_laser1()
    -> drawVirtualLaserLine(false)
    -> 根据仿真焊缝点生成 ROI
    -> GaborFilter + getCurrentCenterLine
    -> currentRealLine
    -> generateVirtualPointCloud(roiVirtualLine, ...)
    -> updateRealPointCloud(curPose, ...)
    -> 累积约 20 帧后 SendMessage(WM_UPDATE_MODEL_POSITION)
        -> OnUpdateModelPosition(...)
            -> 局部 ICP 修正 VirtualWorkpiece pose
            -> 更新 g_lvs_geometry_version
```

局部定位在旧项目里混合了虚拟传感、真实图像中心线、点云生成、滑窗维护、消息触发和 UI/场景刷新。新项目只应迁移算法语义：输入真实线/点云、虚拟参考线/点云、当前位姿和窗口策略，输出局部位姿修正及诊断。

## 方法分类

分类规则：

- A：实际运行链路核心，第一轮设计候选。
- B：被 A 类方法直接依赖，可作为实现细节或离线工具候选。
- C：测试、离线调参、诊断、A/B 对照或可选策略，第一轮不默认迁移。
- D：历史残留、UI/线程/通信/可视化耦合，第一轮不迁移。

### localization 方法

| 方法或区域 | 分类 | 结论 |
| --- | --- | --- |
| `updateGeoDataVec` | A | 几何特征定位入口，提取真实点云中的定位元件特征向量。 |
| `vloc::processGeoData` | B | 特征提取实现细节，依赖 PCL 采样、滤波、法向/几何估计。 |
| `updateModelOffline` | A | 几何约束全局定位入口，调用 `vloc::computeLocation`。 |
| `vloc::computeLocation` | A | 旧项目几何约束求解核心，组合 point/line/plane 约束和自由度限制。 |
| `point2line` / `point2plane` / `lineParallel2plane` / `lineParallel2Line` | B | 几何约束迭代基础算子，可按语义重写为纯 Eigen 实现。 |
| `updateModelOfflineICP` | A | 受限自由度 ICP 全局定位入口。 |
| `stepICPforOffline` | A | 分定位段 ICP，支持每段不同自由度约束。 |
| `onceICPforOffline` | C | 面扫式一次性 ICP 变体，第一轮可作为对照，不默认实现。 |
| `SwWarpPointRigid` | B | PCL 受限自由度 ICP 的关键扩展，适合在 `localization` 内部重写/隔离。 |
| `drawVirtualLaserLine` / `generateVirtualPointCloud` / `updateRealPointCloud` | A | 局部定位的数据生成链路核心，但新项目只保留输入输出语义。 |
| `OnUpdateModelPosition` | A | 局部 ICP 修正入口，迁移为算法服务调用，不保留 MFC 消息。 |
| OCCT `AIS_PointCloud`、`g_IC` 显示、`VirtualWorkpiece->update_parts` | D | 旧可视化和场景刷新，不进入新项目第一轮算法模块。 |

### ibvs 方法

| 方法或区域 | 分类 | 结论 |
| --- | --- | --- |
| `IBVS::setK` | A | 设置相机内参，第一轮需要。 |
| `IBVS::setHandEyeTransform` | A | 手眼标定输入，第一轮需要。 |
| `IBVS::setDesiredFeature` | A | 直接设置期望特征，离线验证优先使用。 |
| `IBVS::setDesiredFeatureFromImage` | C | 依赖 YOLO/红色特征检测，第一轮可作为数据准备工具，不作为核心接口。 |
| `IBVS::setDesiredFeatureFromGeometry` | B | 在线链路实际使用，可作为离线生成期望特征的候选。 |
| `IBVS::computeInitialJacobianOLS` | A | 正交激励或在线样本初始化 J0 的核心。 |
| `IBVS::kfInitWithJ0` | A | 雅可比滤波器初始化核心。 |
| `IBVS::jacobianEK` | A | 在线雅可比更新主路径。 |
| `IBVS::getCurrentJacobian` | A | 运动量或特征不可靠时复用当前雅可比。 |
| `IBVS::jacobianLinear` | B | 旧链路作为 bootstrap / 对比使用，可保留为备用。 |
| `IBVS::evaluateJacobianQuality` | B | 质量门控和诊断，第一轮可输出诊断但不强绑定流程。 |
| `IBVS::computeSeamError` | A | LVS 贴缝误差核心。 |
| `IBVS::computeLVSJacobian` | A | LVS 约束雅可比核心。 |
| `IBVS::computeUnifiedVelocity` | A | 当前主求解器，第一轮优先实现。 |
| `IBVS::computeUnifiedVelocityLegacy` | C | A/B 对照路径，不默认作为新项目主实现。 |
| `IBVS::computeCameraVelocity` | C | 早期单任务求解器，可用于单元测试对照，不作为第一轮主链。 |
| `IBVS::setLvsAxisWeighting` | B | 末端阶段和轴向权重策略，第一轮可保留配置入口但默认关闭。 |
| `IBVS::setPosConstraintByWorkpiece` / `setPosConstraint` / `isPosInRange` | A | 位置约束，第一轮需要。 |
| `IBVS::setSafetyConstraint` / `isSafetyConstraintEnabled` / `isNextPositionSafe` | A | dry-run 控制建议的安全边界，第一轮需要。 |
| `IBVS::setNextPosition` | A | 根据速度和 `dt` 生成 dry-run 下一位姿，第一轮需要。 |
| `IBVS::errorNorm` / `isFullyConverged` | A | 收敛判断，第一轮需要。 |
| `kfConfigureChannelwiseR`、`setNISGateThresholds`、`performSelfCheck`、`adaptiveAdjustParameters`、`shortWindowReset`、`estimateChannelSigmasOffline` | C | 调参/诊断增强，不进入第一轮最小实现。 |
| `PicCapture` 中 `processOrthogonalDataForJacobian`、`loadJacobianFromFile` | C | 离线数据准备工具，可单独复刻为测试工具，不放进核心 `ibvs` 服务。 |
| `YoloDetector`、`RedFeatureDetector`、`InitPointDetector` | C | 特征来源候选；第一轮若已有离线特征序列，可不接入。 |
| `IBVSThread`、`PicCapture` 相机循环、`s_task_ibvs`、`g_ibvs_*`、`g_lvs_*` | D | 线程和全局状态通信，不迁移。 |

## 输入输出和坐标单位

### localization 输入

| 输入 | 旧项目来源 | 新项目建议 |
| --- | --- | --- |
| 真实图像中心线 | `currentRealLine`，由 Gabor + Steger 中心线得到 | 离线图像提取结果或预存 2D 点序列。 |
| 真实点云 | `updateRealPointCloud` 将 2D 线点、相机参数、机器人位姿转成变位机 TCP 坐标下 3D 点 | `localization` 离线输入使用统一点云值对象，单位 mm 或 m 必须显式。 |
| 虚拟参考线/点云 | `VirtualLaserSensor`、`generateVirtualPointCloud`、`Weldment` 模型采样 | 由 `digital_twin` 未来提供；当前离线验证用文件或测试夹具提供。 |
| 定位元件特征 | `geoDataVec`，每段为 6 维向量 | 定义为定位特征观测，不直接暴露 PCL。 |
| 工件模型约束 | `virtualWeldment->position_face_element_list`、`position_face_dof_list` | 新项目用轻量约束描述：参考点/线/面、定位元素类型、自由度 mask。 |
| 当前机器人/变位机位姿 | `curx_r / 1e6`、角度 `/1e4`、`curS1/curS2` | 离线轨迹或 pose snapshot，单位写入字段名或注释。 |
| 标定 | `SENSORPARA_4`、`SENSORPARA.txt`、`RBCALIB.DAT` | 复用当前项目标定数据结构，必要时新增 localization 内部转换。 |

### localization 输出

| 输出 | 旧项目行为 | 新项目建议 |
| --- | --- | --- |
| 全局位姿修正 | 写入 `globalLocRes` 并更新 `VirtualWorkpiece` base pose | 返回 `LocalizationResult`，包含 4x4 pose、状态、误差、迭代次数、诊断。 |
| 局部位姿修正 | `OnUpdateModelPosition` 修改虚拟工件位姿并更新几何版本 | 返回局部 pose delta 和可信度，由后续 `control` 决定是否采纳。 |
| 点云/ICP 诊断 | 可保存 PCD、日志、可视化 | 只作为 debug data，不进入公共接口主字段。 |

### ibvs 输入

| 输入 | 旧项目来源 | 新项目建议 |
| --- | --- | --- |
| 当前特征 | `[u1, v1, u2, v2]`，由 YOLO ROI + 红色特征检测得到 | 第一轮优先使用离线特征序列，后续再接 `vision`。 |
| 期望特征 | `Desired_image` 或几何起始点推导 | 支持直接输入期望特征；图像/几何推导做离线工具。 |
| 相机内参 | `setK(2318.8, 2318.8, 720, 540)` | 来自标定配置，不写死。 |
| 手眼标定 | `data/IBVS/HandEyeCalibration.txt` | 转换成明确的外参矩阵输入。 |
| 机器人当前位姿 | `curx_r/cury_r/curz_r`、`curRx_r/...` | 离线 pose snapshot，不依赖 RobotComm。 |
| LVS 焊缝约束 | `g_seamPoint_world`、`g_seamDirection_world`、`g_tcpPosition_world` | 定义为可选 `LvsConstraintObservation`。 |
| 安全/位置约束 | 工件类型映射和显式 XYZ 上下界 | 定义为配置，不从 UI/global 读取。 |

### ibvs 输出

| 输出 | 旧项目行为 | 新项目建议 |
| --- | --- | --- |
| 速度建议 | `final_velocity`，单位 m/s，控制台以 mm/s 展示 | 返回 dry-run `velocity_mps`。 |
| 下一位姿 | `setNextPosition` 后写 `ibvs_x/y/z/rx/ry/rz` | 返回 `nextPose` 或 `deltaPose`，不发送机器人命令。 |
| 收敛状态 | 特征误差、LVS 误差、连续帧数 | 返回 `status`、`featureErrorPx`、`lvsErrorMm`、`stopReason`。 |
| 诊断 | KF、NIS、Jacobian、LVS debug 文件 | 第一轮保留结构化 debug，可不落文件。 |

## 可复用算法清单

### localization 第一轮候选

1. 几何特征定位：`processGeoData` 语义 + `computeLocation` 约束求解。
2. 受限自由度 ICP：真实点云预处理、虚拟参考点云、自由度约束 warp。
3. 局部滑窗 ICP：真实线/虚拟线 ROI 点云、窗口累积、局部 pose delta。
4. 坐标转换：图像点到传感器/机器人/变位机坐标的转换，但必须从旧 HALCON/newmat 调用中抽成值对象和 Eigen 计算。

### ibvs 第一轮候选

1. 两点图像特征误差模型：`f = [u1, v1, u2, v2]^T`。
2. 初始雅可比：优先支持直接加载/传入 J0 和 OLS 样本估计。
3. 在线雅可比更新：`jacobianEK` 语义。
4. LVS 贴缝约束：`D = (I - l*l^T) * (P - P0)` 和 `J_LVS = I - l*l^T`。
5. 统一加权阻尼最小二乘速度求解：以 `computeUnifiedVelocity` 为主。
6. 位置/安全约束：XYZ 范围、速度限幅、下一位姿裁剪、收敛判定。

## 不迁移清单

第一轮明确不迁移：

- `CRobotleadingDlg` 的 UI、按钮状态、消息映射、`SendMessage` 触发方式。
- `IBVSThread`、`PicCapture` 的相机采集循环、Pylon 设备生命周期。
- `s_task_ibvs`、`g_ibvs_active`、`g_lvs_*` 等全局变量通信形态。
- `RobotCtrl` 或 WithinRobot 下的真实命令发送逻辑。
- OCCT/VTK/AIS 显示对象、`g_IC` 场景刷新、`VirtualWorkpiece->update_parts`。
- `data/IBVS/testresult`、`Photo/`、`output/` 下运行日志和可视化输出。
- `IBVS::computeUnifiedVelocityLegacy` 作为主路径。
- `performSelfCheck`、自适应 KF 调参、NIS/ACF 调参工具作为第一轮核心功能。
- 未被主链路调用的 detector 测试程序和 `test_InitPointDetector.cpp`。

## 离线验证数据清单

### localization 最小数据集

| 验证目标 | 最少需要的数据 |
| --- | --- |
| 几何特征全局定位 | 工件定位约束描述、若干 `geoData` 观测、期望位姿或可人工检查的参考位姿。 |
| ICP 全局配准 | 真实点云、虚拟/模型参考点云、初始位姿、自由度约束、期望配准误差阈值。 |
| 局部定位 | 连续帧真实中心线或真实局部点云、对应虚拟线/虚拟点云、每帧 pose snapshot、窗口长度。 |
| 图像点转 3D | `SENSORPARA.txt` / `SENSORPARA_4` 字段、相机到工具外参、机器人 pose。 |

旧项目可参考数据位置：

- `SWv1/data/Robotleadingdata/BASIC/SENSORPARA.txt`
- `SWv1/data/Robotleadingdata/StationPos/RBCALIB.DAT`
- `SWv1/models/weldment/*.weldment`
- `SWv1/Final.pcd`
- 旧运行中写出的 `output/rawPCreal.pcd`、`output/rawPCvirtual.pcd`、`output/rawPCtransReal.pcd` 只作为调试格式参考，不作为必需输入。

### ibvs 最小数据集

| 验证目标 | 最少需要的数据 |
| --- | --- |
| 期望特征验证 | 直接给定 `f_desired = [u1, v1, u2, v2]`，或使用 `Desired_image` 生成。 |
| 雅可比初始化 | `Init_Jacobi.txt` 或 `delta_s + delta_F` 样本。 |
| 在线误差收敛 | 连续帧 `current_feature`、当前 pose、期望特征、控制周期 `dt`。 |
| LVS 融合 | 焊缝点 `P0`、方向 `l`、TCP 点 `P`、`g_lvs_data_valid` 等价状态。 |
| 安全约束 | XYZ 上下界、最大速度、停止阈值、连续帧数。 |

旧项目可参考数据位置：

- `SWv1/data/IBVS/HandEyeCalibration.txt`
- `SWv1/data/IBVS/Orthogonal/Init_Jacobi.txt`
- `SWv1/data/IBVS/welding_sample/*.txt`
- `SWv1/data/IBVS/curve_sample/*.txt`
- `SWv1/data/IBVS/init_sample/*.txt`
- `SWv1/data/IBVS/onnx_workpiece/*/best.onnx`

第一轮如果已有离线特征序列，可以暂不使用 ONNX 和红色特征检测，直接验证控制律方向、限幅和收敛逻辑。

## 新项目第一轮开发建议

### localization

1. 定义离线输入结构：真实点云/真实线、虚拟参考点云/参考线、约束描述、初始位姿、标定快照。
2. 实现纯接口层：全局定位请求、局部定位请求、定位结果、诊断信息。
3. 先做几何特征全局定位或 ICP 全局配准二选一的最小验证，不同时铺开全部算法。
4. PCL 实现放在 `localization` 私有实现文件中，测试只检查结果位姿和误差，不检查 PCL 类型。
5. 局部定位等全局定位验证稳定后再加，窗口策略先固定，不做实时线程。

### ibvs

1. 定义 `IbvsRequest` / `IbvsResult` 语义：当前特征、期望特征、当前 pose、可选 LVS 观测、安全约束、`dt`。
2. 第一轮优先实现直接特征输入，不接 Basler、YOLO、RedFeatureDetector。
3. 以 `computeUnifiedVelocity` 为主求解器，`computeUnifiedVelocityLegacy` 仅做对照测试。
4. 输出 dry-run 速度和下一位姿，不写 RobotComm，不写全局状态。
5. 测试覆盖：误差方向、速度限幅、LVS 参与/不参与差异、约束裁剪、收敛停止。

## 待确认点

- 旧项目 `Desired_image` 目录在当前 checkout 中未作为稳定输入列出；如果要复用图像期望特征，需要确认实际样本来源。
- `SENSORPARA_4` 中各字段的单位和 `pose_CinT[6]` 语义仍需结合现有标定文档再次确认。
- 旧项目 `Weldment` 模型中定位面、定位元素和自由度编码需要在新项目中转换成轻量 schema，不能直接依赖旧类。
- `IBVS` 的第一轮是否需要在线 EK/KF，取决于离线数据是否包含足够的 `delta_s / delta_F`；若没有，先用固定 J0 验证控制律。
