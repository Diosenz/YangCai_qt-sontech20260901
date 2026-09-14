# 纯视觉 IBVS 模块维护计划

## 目标

本文件维护当前 Qt/CMake 项目中“不含 LVS 的纯视觉 IBVS”拆分计划、legacy 对应关系和后续测试入口。当前阶段已经拆出纯算法库，并补充最小 smoke 测试；仍不接 UI、RobotComm、真实相机线程和机器人写回。

## 模块拆分

| 新文件 | 职责 | 依赖边界 |
| --- | --- | --- |
| `Ibvs/include/IbvsTypes.h` | 公共值对象、错误状态、诊断结构、Eigen 类型别名 | 已拆出；只依赖 Eigen/STL |
| `Ibvs/include/IbvsFeatureGeometry.h` / `Ibvs/src/IbvsFeatureGeometry.cpp` | 根据焊枪双特征点、焊缝起始点和几何比例生成期望特征 | 已拆出；不读图、不访问相机/机器人 |
| `Ibvs/include/IbvsJacobianEstimator.h` / `Ibvs/src/IbvsJacobianEstimator.cpp` | OLS 初始雅可比、KF 在线更新、线性雅可比、NIS 门控、短窗重估、质量评价 | 已拆出；不包含 LVS 约束和全局变量 |
| `Ibvs/include/IbvsVelocitySolver.h` / `Ibvs/src/IbvsVelocitySolver.cpp` | WLS 白化、SVD、LM 阻尼伪逆、速度限幅和诊断 | 已拆出；输出 dry-run 平移速度，不推进机器人位姿 |
| `Ibvs/include/IbvsRedFeatureDetector.h` / `Ibvs/src/IbvsRedFeatureDetector.cpp` | HSV 红色 mask、形态学、连通域筛选、双点排序 | 已拆出；仅 `I2R_RLC_ENABLE_OPENCV=ON` 时编译 |
| `Ibvs/include/IbvsStartPointDetector.h` / `Ibvs/src/IbvsStartPointDetector.cpp` | 已有焊缝点/边线输入下的端点、求交、曲线最近点起始点算法 | 已拆出；先不迁 CAD 模板对齐搜索 |
| `Ibvs/include/IbvsOfflineDataReader.h` / `Ibvs/src/IbvsOfflineDataReader.cpp` | 读取特征序列、机器人位移/位姿、初始雅可比、ROI、速度对照 txt | 已拆出；函数只收路径参数，不内置绝对路径 |
| `Ibvs/include/IbvsYoloDetector.h` / `Ibvs/src/IbvsYoloDetector.cpp` | ONNX Runtime YOLO wrapper | 已拆出可选入口；通过 vcpkg feature `ibvs-yolo` 和 overlay port 管理 CPU 版 ONNX Runtime 预编译包 |

## 旧项目优先参考表

| legacy 文件/函数 | 新模块位置 | 处理方式 |
| --- | --- | --- |
| `SWv1/IBVS.cpp::setDesiredFeatureFromGeometry` | `IbvsFeatureGeometry` | 保留几何关系，移除临时硬编码方向和控制台输出，`H1/H2` 参数化为 `30.0/24.0` |
| `SWv1/IBVS.cpp::computeInitialJacobianOLS` | `IbvsJacobianEstimator` | 保留 RowMajor 状态展开和阻尼最小二乘估计 |
| `SWv1/IBVS.cpp::kfInitWithJ0`、`jacobianEK` | `IbvsJacobianEstimator` | 保留 KF 状态、Joseph 协方差更新、NIS 门控、短窗历史 |
| `SWv1/IBVS.cpp::jacobianLinear` | `IbvsJacobianEstimator` | 保留相机内参和手眼旋转映射 |
| `SWv1/IBVS.cpp::computeCameraVelocity` | `IbvsVelocitySolver` | 保留 WLS、SVD、LM 阻尼和速度限幅，改为结构化诊断 |
| `SWv1/RedFeatureDetector.*` | `IbvsRedFeatureDetector` | 第一轮迁移 HSV mask、连通域、几何过滤和 x 排序核心 |
| `SWv1/InitPointDetector.*` 起始点函数 | `IbvsStartPointDetector` | 只迁端点/求交/曲线最近点算法入口 |
| `SWv1/YoloDetector.*` | `IbvsYoloDetector` | 作为可选 ONNX 能力，不阻塞无 OpenCV/无 ONNX 构建 |
| `SWv1/IBVS.cpp::computeUnifiedVelocity*`、`computeSeamError` | 暂缓 | 属于 LVS/统一约束路径，本轮明确跳过 |
| `SWv1/PicCapture.cpp` 线程、全局变量、文件输出、机器人回写 | 暂缓 | 只作为调用链参考，不迁入算法模块 |

## Feature_detection 参考表

| 参考点 | 用途 |
| --- | --- |
| `C:\Feature_detection\FeatureDetection\FeatureDetection\IBVS.*` | 与旧项目版本对照 OLS/KF/速度解的离线验证实现 |
| `C:\Feature_detection\onnx_workpiece\best.onnx` | YOLO 焊枪/工件 ROI 模型输入；当前已确认文件存在 |
| `Desired_image\Images_0100.jpg`、`Images\Images_0100.jpg`、`Orthogonal\Images\Images_00.jpg` | YOLO ROI、红色特征点和期望特征桥接的真实离线图像输入 |
| `Orthogonal\Feature_coordinates.txt` | 后续测试 J0 OLS 的特征增量输入 |
| `Orthogonal\IBVSRobotData.txt` | 后续测试 J0 OLS 的平移激励输入 |
| `Orthogonal\Init_Jacobi.txt` | 离线小链路测试读取 4x3 初始雅可比，并忽略 residual/condition/sample count 元数据 |
| `data\Feature_coordinates.txt` | 离线小链路测试在线特征序列回放；基础数据为 4 列无 frame 格式 |
| `data\ROI_data.txt` | 离线小链路测试 ROI/header 读取和全局特征点一致性；真实格式为 `Frame,ROI_X,ROI_Y,ROI_W,ROI_H,...` |
| `在线数据\v_data.txt` | 离线小链路测试速度序列读取；基础数据为 3 列 `vx vy vz` |
| `在线数据\Camera_Position.txt` | 离线小链路测试相机位姿序列读取，并在小回放中按最短序列对齐 |

## 分项测试计划

1. 数据类型默认值：检查 `IbvsTypes.h` 默认单位、默认阈值和错误状态。已由 `i2r_rlc_ibvs_core_tests` 最小覆盖。
2. txt reader：分别读取特征序列、机器人位移、相机位姿、初始雅可比、ROI、速度序列。已由 `i2r_rlc_ibvs_core_tests` 使用临时 txt 最小覆盖；当前兼容无 frame 4 列特征、带 frame 5 列特征、带 frame ROI、3 列速度、7 列带 frame 速度和 `Init_Jacobi.txt` 元数据行。
3. 红色双点：用合成 ROI 图像验证 HSV mask、连通域过滤和左右排序。已由 `i2r_rlc_ibvs_opencv_tests` 在 OpenCV preset 下最小覆盖。
4. 期望特征：用已知双特征点和焊缝起始点验证 `f_desired=[u1*,v1*,u2*,v2*]`。已由 `i2r_rlc_ibvs_core_tests` 最小覆盖。
5. 起始点辅助算法：验证端点选取、直线求交和曲线最近点。已由 `i2r_rlc_ibvs_core_tests` 最小覆盖。
6. J0 OLS：用正交激励样本验证残差、条件数和雅可比矩阵非零。已由 `i2r_rlc_ibvs_core_tests` 使用合成样本最小覆盖。
7. KF 更新：验证小运动量跳过、一次正常 KF 更新、当前雅可比读回。已由 `i2r_rlc_ibvs_core_tests` 最小覆盖；NIS 拒绝/放大待后续扩大覆盖。
8. 速度求解：验证零矩阵保护、正常速度求解、速度限幅和收敛判断。已由 `i2r_rlc_ibvs_core_tests` 最小覆盖。
9. 离线小链路测试：新增 `i2r_rlc_ibvs_offline_replay_tests`，在本机存在 `C:\Feature_detection` 时读取基础真实 txt 数据；数据缺失时输出 skip，不影响普通 CI/smoke tests。
10. 可选 YOLO：开启 ONNX Runtime 后验证焊枪 ROI 检测和 NMS。当前使用 `qt-debug-opencv-onnx` preset 启用 vcpkg feature `ibvs-yolo`，并通过 `vcpkg-ports/onnxruntime` 安装 Microsoft 官方预编译 CPU 包；模型文件存在，旧 `Feature_detection` 代码输出解析格式为 `[1,N,6]`。当前 smoke 测试保留黑图推理，并额外读取真实离线图片验证至少一帧产生合法 ROI。
11. 图像小链路：新增 `i2r_rlc_ibvs_image_chain_tests`，把 YOLO ROI、红色双点和 `IbvsFeatureGeometry` 串成离线图像诊断链；输出 annotated 图、ROI crop、red mask、逐帧 CSV 和 summary。
12. 起始点离线诊断：新增 `i2r_rlc_ibvs_start_point_offline_tests`，只测试当前新项目已有端点、直线交点和曲线最近点纯算法；输出点集 CSV 和叠加图，不迁入 legacy CAD/PnP/模板对齐大流程。
13. 速度诊断留档：新增 `i2r_rlc_ibvs_velocity_diagnostics_tests`，读取在线特征、相机位置和 `Init_Jacobi.txt`，保存逐帧误差、速度、NIS、限幅标记 CSV 与曲线图；速度触顶只作为诊断信号。

## 离线小链路测试记录

本轮新增的 `i2r_rlc_ibvs_offline_replay_tests` 仍是分项验证，不接 UI、RobotComm、真实相机线程、LVS、ONNX YOLO、真实图像检测或机器人运动闭环。

覆盖范围：

- 正交 J0：读取 `Orthogonal\Feature_coordinates.txt` 和 `Orthogonal\IBVSRobotData.txt`，按 legacy `PicCapture.cpp::processOrthogonalDataForJacobian` 使用相邻帧差分生成 `delta_s` / `delta_F`。筛选规则为 `motion_norm > 1e-5 && motion_norm < 0.05 && feature_magnitude > 0.1 && feature_magnitude < 100`。
- J0 对照：用 `computeInitialJacobianOLS(..., 1e-6)` 重算 4x3 初始雅可比，与 `Orthogonal\Init_Jacobi.txt` 读取结果做相对误差比较，同时检查样本数、残差和条件数为有限值。
- 在线 reader：读取 `data\Feature_coordinates.txt`、`data\ROI_data.txt`、`在线数据\v_data.txt`、`在线数据\Camera_Position.txt`，验证行数、ROI 合法性、ROI 局部点加偏移后的全局点与特征文件前若干帧一致。
- KF/速度小回放：用 `Init_Jacobi.txt` 初始化 KF，取在线特征和相机位置前 N 帧做差分，运行 `jacobianEK` 与 `computeCameraVelocity`。当前检查可运行、非 NaN/Inf、部分帧产生有效更新、速度 obey max velocity，并输出总帧数、有效更新帧数、跳过帧数、速度限幅帧数、速度均值/最大值、误差均值/最大值、NIS 均值/最大值。
- 当前验证基线：`i2r_rlc_ibvs_core_tests`、`i2r_rlc_ibvs_offline_replay_tests`、`i2r_rlc_ibvs_opencv_tests` 已在 `qt-debug-opencv` 构建树通过。`Orthogonal` 样本数为 62，J0 残差约 6.01886，条件数约 1.57437；小回放 79 帧全部更新，`failedUpdates=0`，`validVelocity=79`，速度最大值达到 `maxCameraVelocityMps=0.02`。
- 速度触顶解释：当前小回放使用第一帧在线特征作为期望特征，平均特征误差约 153 px；速度触顶优先作为诊断信号，说明此阶段只能证明 txt 离线链路可运行，不能证明控制增益或期望特征策略已经适合真实闭环。

## YOLO 验证记录

- 已确认 `C:\Feature_detection\onnx_workpiece\best.onnx` 存在，大小约 80 MB。
- 已确认旧项目 `C:\Feature_detection\FeatureDetection\FeatureDetection\YoloDetector.cpp` 使用 ONNX Runtime CPU/DirectML，输入为 640，输出按 `[1,N,6]` 解析，字段为 `cx, cy, w, h, objectness, class_score`。当前 `IbvsYoloDetector.cpp` 的基础解析假设与旧代码一致。
- 当前 `vcpkg.json` 通过 feature `ibvs-yolo` 声明 ONNX Runtime；普通构建不启用该 feature，专用 `qt-debug-opencv-onnx` preset 才通过 overlay port 安装和验证官方预编译 CPU 包。
- 已确认旧项目 `.vcxproj` 指向的 `E:\Libs\Microsoft.ML.OnnxRuntime.DirectML.1.15.1` 在当前机器上不存在。
- 当前机器 `C:\Windows\System32\onnxruntime.dll` 不是可用开发包，不能提供头文件、导入库或 CMake target。
- 已确认当前 ONNX Runtime smoke 可输出版本并通过；本机已验证版本为 `1.23.2`。
- `i2r_rlc_ibvs_yolo_smoke_tests` 当前测试路径为：
  - 黑图推理：验证模型加载和 ONNX Runtime 推理链路不崩溃，允许 0 检出。
  - 真实离线图：依次测试 `Desired_image\Images_0100.jpg`、`Images\Images_0100.jpg`、`Orthogonal\Images\Images_00.jpg`。
  - 若模型或全部真实图缺失，则输出 `SKIP`，不影响没有 `C:\Feature_detection` 的机器。
  - 若模型和至少一张真实图存在，则必须至少一张图产生合法 ROI，否则测试失败。
  - 每个检测结果要求 score 有限且位于 `[0, 1]`，ROI 宽高为正，且 ROI 与原图边界有有效交集。

因此当前 YOLO smoke 验证边界是“vcpkg 管理的 CPU 版预编译依赖 + wrapper 编译 + 单模型推理 + 真实离线图 ROI smoke test”。旧 DirectML/GPU 路线暂不迁入；YOLO ROI 与红色特征点、期望特征的组合由独立的离线图像链路测试覆盖，仍不接入 IBVS 控制回放或机器人闭环。

## 分模块离线可视化测试记录

新增分模块测试统一把人工复核数据写入：

```text
output/ibvs_offline_validation/<test_name>/<timestamp>/
```

该目录已被 `.gitignore` 忽略，不作为版本库基准数据提交。

- `i2r_rlc_ibvs_image_chain_tests`：使用真实离线图像和 `best.onnx`，输出 `annotated_*.jpg`、`roi_*.jpg`、`red_mask_*.png`、`metrics.csv`、`summary.txt`。验收为至少一帧合法 YOLO ROI，且至少一帧 ROI 内红色双点 accepted；失败帧仍保留图像供人工复核。
- `i2r_rlc_ibvs_start_point_offline_tests`：使用可控离线点集验证端点、交点和曲线最近点算法，输出 `endpoint_start_point.jpg`、`intersection_start_point.jpg`、`curve_nearest_start_point.jpg`、点集 CSV、`metrics.csv`、`summary.txt`。验收为输出点有限、几何结果与预期一致。
- `i2r_rlc_ibvs_velocity_diagnostics_tests`：读取 `data\Feature_coordinates.txt`、`在线数据\Camera_Position.txt` 和 `Orthogonal\Init_Jacobi.txt`，输出 `metrics.csv`、`error_norm_px.png`、`velocity_norm_mps.png`、`nis.png`、`limit_flags.png`、`summary.txt`。验收为无 NaN/Inf、`failedUpdates=0`、存在有效速度解且速度 obey `maxCameraVelocityMps`。

## 本轮 smoke 验证入口

当前测试目标已经接入 CMake，但 Codex 侧不再反复发起完整 MSVC/Ninja 构建；完整验证优先在 VS IDE 或用户的 VS Developer PowerShell 中执行。

no-opencv 核心测试：

```powershell
cmake --preset qt-debug-no-opencv
cmake --build --preset qt-debug-no-opencv --target i2r_rlc_ibvs_core_tests --verbose
ctest --test-dir out\build\qt-debug-no-opencv -C Debug -R i2r_rlc_ibvs_core_tests --output-on-failure
```

OpenCV smoke 测试：

```powershell
cmake --preset qt-debug-opencv
cmake --build --preset qt-debug-opencv --target i2r_rlc_ibvs_opencv_tests --verbose
ctest --test-dir out\build\qt-debug-opencv -C Debug -R i2r_rlc_ibvs_opencv_tests --output-on-failure
```

ONNX / YOLO smoke 测试：

```powershell
cmake --preset qt-debug-opencv-onnx
cmake --build --preset qt-debug-opencv-onnx --target i2r_rlc_onnxruntime_smoke_tests --verbose
cmake --build --preset qt-debug-opencv-onnx --target i2r_rlc_ibvs_yolo_smoke_tests --verbose
ctest --test-dir out\build\qt-debug-opencv-onnx -C Debug -R "i2r_rlc_(onnxruntime_smoke|ibvs_yolo_smoke)_tests" --output-on-failure
```

IBVS 分模块离线可视化测试：

```powershell
ctest --test-dir out\build\qt-debug-opencv-onnx -C Debug -R "i2r_rlc_(onnxruntime_smoke|ibvs_yolo_smoke|ibvs_image_chain|ibvs_start_point_offline|ibvs_velocity_diagnostics)_tests" --output-on-failure
ctest --test-dir out\build\qt-debug-opencv -C Debug -R "i2r_rlc_ibvs_(core|opencv|offline_replay)_tests" --output-on-failure
```

若由 Codex 做短检查，只允许先做干运行确认目标依赖，不进入真实编译：

```powershell
cmake --build --preset qt-debug-no-opencv --target i2r_rlc_ibvs_core_tests -- -n -d explain
```

## 暂缓项

- LVS、`computeUnifiedVelocity`、`computeLVSJacobian`、`computeSeamError`。
- 真实机器人命令、位姿推进、RobotComm 写回。
- UI 交互、相机线程、完整闭环运行。
- 路径配置化和运行时参数配置文件。
- CAD 模板对齐、PNP、完整曲线焊缝起始点大算法。

## 当前实现假设

- 旧项目 `SWv1` 作为第一参考，`C:\Feature_detection` 用于补充离线验证细节。
- 新函数不内置测试路径；后续测试阶段可以把 `C:\Feature_detection` 下的绝对路径作为参数传入。
- `ibvs` 保持视觉伺服专属边界，不吸收 localization 点云定位、不直接管理 camera/RobotComm 生命周期。
