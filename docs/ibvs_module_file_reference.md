# IBVS 模块文件与函数说明

本文档说明根级 `IBVS` 模块中各文件的职责、主要类型、公开函数和对应测试入口。IBVS 当前以静态库 `i2r_rlc_ibvs` 构建，核心算法始终编译；OpenCV 和 ONNX Runtime 相关能力由 CMake 选项按需加入。

## 构建结构

| 文件 | 作用 |
| --- | --- |
| `IBVS/CMakeLists.txt` | 定义静态库 `i2r_rlc_ibvs`，配置 include 路径、Eigen、OpenCV 和 ONNX Runtime 依赖；模块公共接口不依赖 Qt。 |
| `IBVS/include/ibvs/*.h` | 模块公开接口，供测试、UI 或后续服务层调用。 |
| `IBVS/src/*.cpp` | 模块实现文件，包含算法实现和文件内辅助函数。 |

核心源文件始终参与编译：

- `NullIbvsService.cpp`
- `IbvsFeatureGeometry.cpp`
- `IbvsJacobianEstimator.cpp`
- `IbvsOfflineDataReader.cpp`
- `IbvsStartPointDetector.cpp`
- `IbvsVelocitySolver.cpp`

启用 `I2R_RLC_ENABLE_OPENCV` 后追加：

- `IbvsRedFeatureDetector.cpp`
- `IbvsStartPointImageDetector.cpp`

启用 `I2R_RLC_ENABLE_ONNX_RUNTIME` 后追加：

- `IbvsYoloDetector.cpp`

## 公共类型文件

### `IbvsTypes.h`

定义 IBVS 模块共享的数据结构、控制参数、估计参数和通用结果类型。

主要类型别名：

| 名称 | 含义 |
| --- | --- |
| `IbvsFeatureVector` | 4 维图像特征向量，通常为 `[u1, v1, u2, v2]`。 |
| `IbvsVelocityVector` | 3 维相机平移速度向量。 |
| `IbvsPoseVector` | 6 维位姿向量。 |
| `IbvsJacobianMatrix` | 4x3 图像特征到相机平移的雅可比矩阵。 |
| `IbvsStateVector` | 12 维 KF 状态，展开存储 4x3 雅可比。 |
| `IbvsStateCovariance` | 12x12 KF 状态协方差。 |
| `IbvsObservationCovariance` | 4x4 观测协方差。 |
| `IbvsKalmanGain` | 12x4 Kalman 增益。 |

主要结构：

| 类型 | 作用 | 主要函数 |
| --- | --- | --- |
| `IbvsStatus` | 算法返回状态枚举。 | 无 |
| `IbvsError` | 结构化错误信息。 | `isOk()` |
| `PixelPoint` | 像素点，字段为 `u/v`。 | `isFinite()` |
| `PixelRoi` | 像素 ROI，字段为 `x/y/width/height`。 | `isValid()` |
| `PixelFeaturePair` | 焊枪双特征点。 | `isValid()`、`distancePx()`、`toVector()` |
| `IbvsCameraIntrinsics` | 相机内参。 | `isValid()`、`toMatrix()` |
| `IbvsHandEyeCalibration` | 手眼标定旋转和平移。 | 无 |
| `IbvsFeatureGeometryParameters` | 期望特征生成参数。 | 无 |
| `IbvsControlParameters` | 速度控制参数。 | 无 |
| `IbvsKalmanConfig` | KF 雅可比估计参数。 | 无 |
| `IbvsJacobianOlsResult` | OLS 初始雅可比结果。 | 无 |
| `IbvsJacobianUpdateResult` | KF 单帧更新结果。 | 无 |
| `IbvsJacobianQuality` | 雅可比质量诊断。 | `isGood()`、`isAcceptable()` |
| `IbvsVelocitySolveResult` | 速度求解结果。 | 无 |
| `IbvsFeatureGeometryResult` | 期望特征生成结果。 | 无 |

`IbvsFeatureGeometryParameters` 和 `IbvsFeatureGeometryResult` 定义在
`IbvsFeatureGeometry.h` 中，避免让 `IbvsTypes.h` 承载 feature geometry 专属类型。
`IbvsConfig.h` 集中保存 feature geometry 默认参数常量。

## 核心算法文件

### `IbvsFeatureGeometry.h/.cpp`

根据当前焊枪双特征点和焊缝起始点生成 IBVS 期望特征。

公开函数：

| 函数 | 作用 |
| --- | --- |
| `computeDesiredFeature(const PixelFeaturePair&, const PixelPoint&, double, const IbvsFeatureGeometryParameters&)` | 自由函数，计算期望双特征点和 4 维期望特征向量。 |

实现要点：

- 检查当前双特征点、起始点和参数有效性。
- 计算双点方向向量和期望点偏移。
- 返回 `desiredPoints`、`desiredFeature`、`unitDirection`、`h1Px`、`h2Px`。

### `IbvsJacobianEstimator.h/.cpp`

估计、维护和诊断 IBVS 4x3 平移雅可比。

公开类：

- `IbvsJacobianEstimator`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `IbvsJacobianEstimator()` | 使用默认 KF 参数构造估计器。 |
| `IbvsJacobianEstimator(const IbvsKalmanConfig&)` | 使用指定 KF 参数构造估计器。 |
| `configure(const IbvsKalmanConfig&)` | 更新 KF 参数。 |
| `config() const` | 返回当前 KF 参数。 |
| `configureChannelwiseR(const Eigen::Vector4d&)` | 设置按通道观测噪声。 |
| `setNisGateThresholds(double, double, double)` | 设置 NIS 拒绝、放大和放大倍数阈值。 |
| `kfInitWithJ0(const IbvsJacobianMatrix&)` | 用已有初始雅可比初始化 KF 状态。 |
| `computeInitialJacobianOLS(...) const` | 用位移增量和特征增量通过 OLS 估计初始雅可比。 |
| `kfInitWithSamples(...)` | 先 OLS 估计 J0，再初始化 KF 状态。 |
| `jacobianEK(const Eigen::Vector3d&, const Eigen::Vector4d&)` | 根据相机位移增量和图像特征增量做单帧 KF 更新。 |
| `jacobianLinear(double, const PixelFeaturePair&, const IbvsCameraIntrinsics&, const IbvsHandEyeCalibration&) const` | 根据当前图像点、深度和手眼旋转计算线性雅可比。 |
| `getCurrentJacobian() const` | 返回当前 4x3 雅可比。 |
| `getState() const` | 返回 12 维 KF 状态。 |
| `getCovariance() const` | 返回状态协方差。 |
| `getObservationCovariance() const` | 返回观测协方差。 |
| `getLastNis() const` | 返回最近一次 NIS。 |
| `evaluateJacobianQuality() const` | 输出条件数、秩、奇异值和预测误差等质量诊断。 |
| `shortWindowReset(std::size_t, double)` | 用最近窗口样本重新估计雅可比。 |
| `estimateChannelSigmasOffline(...) const` | 根据离线残差估计各特征通道噪声。 |

重要内部函数：

| 函数 | 作用 |
| --- | --- |
| `makeH(const Eigen::Vector3d&)` | 根据位移增量构造 KF 观测矩阵。 |
| `stateToJacobian(const IbvsStateVector&)` | 将 12 维状态还原为 4x3 雅可比。 |
| `jacobianToState(const IbvsJacobianMatrix&)` | 将 4x3 雅可比展开为 12 维状态。 |
| `ensureInitialized()` | 保证 KF 状态已初始化。 |
| `computeNis(...) const` | 计算归一化创新平方。 |
| `recentRmse() const` | 计算最近残差 RMSE。 |
| `recentNisMean() const` | 计算最近 NIS 均值。 |
| `pushRecentSample(...)` | 保存短窗样本和诊断量。 |
| `trimHistory()` | 限制历史样本长度。 |

### `IbvsVelocitySolver.h/.cpp`

根据当前特征误差和雅可比矩阵求解 3 维相机平移速度。

公开类：

- `IbvsVelocitySolver`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `IbvsVelocitySolver()` | 使用默认控制参数构造速度求解器。 |
| `IbvsVelocitySolver(const IbvsControlParameters&)` | 使用指定控制参数构造速度求解器。 |
| `setControlParameters(const IbvsControlParameters&)` | 更新速度求解参数。 |
| `controlParameters() const` | 返回当前速度求解参数。 |
| `setDesiredFeature(const IbvsFeatureVector&)` | 设置期望 4 维图像特征。 |
| `desiredFeature() const` | 返回期望特征。 |
| `computeCameraVelocity(const IbvsJacobianMatrix&, const IbvsFeatureVector&) const` | 用 WLS、SVD 和 LM 阻尼伪逆求解相机速度，并执行限幅。 |
| `featureError(const IbvsFeatureVector&) const` | 计算当前特征与期望特征的误差向量。 |
| `errorNorm(const IbvsFeatureVector&) const` | 计算特征误差范数。 |
| `isConverged(const IbvsFeatureVector&) const` | 根据停止阈值判断是否收敛。 |

## 数据读取文件

### `IbvsOfflineDataReader.h/.cpp`

读取 IBVS 文本数据，用于离线回放、诊断和测试。

公开模板/结构：

| 类型 | 作用 |
| --- | --- |
| `IbvsDataSeries<T>` | 带错误状态的数据序列。 |
| `IbvsJacobianFileResult` | 初始雅可比文件读取结果。 |

公开类：

- `IbvsOfflineDataReader`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `readFeatureCoordinates(const std::string&)` | 读取 4 维图像特征序列，支持无 frame 和带 frame 行。 |
| `readRobotTranslations(const std::string&)` | 读取 3 维机器人平移序列。 |
| `readCameraPoses(const std::string&)` | 读取 6 维相机位姿序列。 |
| `readInitialJacobian(const std::string&)` | 读取初始 4x3 雅可比矩阵。 |
| `readRois(const std::string&)` | 读取 ROI 序列，支持无 frame 和带 frame 行。 |
| `readVelocitySeries(const std::string&)` | 读取 3 维速度序列，支持 3 列和 7 列诊断格式。 |

重要内部函数：

| 函数 | 作用 |
| --- | --- |
| `parseNumbers(const std::string&)` | 将一行文本解析为数字列表，支持逗号和分号分隔，跳过注释行。 |
| `makeOpenError(const std::string&)` | 生成文件打开失败错误。 |
| `openInputFile(const std::string&)` | 用 UTF-8 路径打开输入文件。 |

## 起始点识别文件

### `IbvsStartPointDetector.h/.cpp`

提供焊缝起始点的几何算法，不依赖 OpenCV 图像输入。

公开类型：

| 类型 | 作用 |
| --- | --- |
| `IbvsLineModel` | 二维直线模型，形式为 `a*x + b*y + c = 0`。 |
| `IbvsStartPointResult` | 起始点、终点和拟合直线结果。 |

公开类：

- `IbvsStartPointDetector`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `fitLineTls(const std::vector<PixelPoint>&)` | 对点集做 TLS 直线拟合。 |
| `computeEndpointStartPoint(const std::vector<PixelPoint>&, bool)` | 用端点法计算起点和终点。 |
| `computeIntersectionStartPoint(const IbvsLineModel&, const IbvsLineModel&)` | 计算两条直线交点作为起始点。 |
| `computeCurveNearestStartPoint(const std::vector<PixelPoint>&, const PixelPoint&)` | 从曲线点中选择靠近焊枪端点的起始点。 |

重要内部函数：

| 函数 | 作用 |
| --- | --- |
| `squaredDistance(const PixelPoint&, const PixelPoint&)` | 计算二维点距离平方。 |

### `IbvsStartPointImageDetector.h/.cpp`

基于工件图像、模板点和焊缝采样点完成起始点图像识别链路。该文件依赖 OpenCV，只在 `I2R_RLC_ENABLE_OPENCV=ON` 时编译。

公开类型：

| 类型 | 作用 |
| --- | --- |
| `IbvsStartPointWorkpieceShape` | 工件形态枚举：直线焊缝或曲线焊缝。 |
| `IbvsStartPointStrategy` | 起始点策略枚举：端点、交点、曲线固定弧长比例。 |
| `IbvsTemplateAlignmentParameters` | 三层模板匹配搜索范围、Chamfer 参数和候选数量。 |
| `IbvsStartPointImageDetectorConfig` | 单个工件的图像链路配置。 |
| `IbvsStartPointImageResult` | 图像链路输出，包含状态、点集、位姿、诊断指标和可视化图。 |

公开类：

- `IbvsStartPointImageDetector`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `detect(const cv::Mat&, const std::vector<std::vector<cv::Point3f>>&, const std::vector<std::vector<cv::Point3f>>&, const IbvsStartPointImageDetectorConfig&) const` | 在裁切工件图上完成边缘提取、模板对齐、焊缝点提取和起始点计算。 |

重要内部结构：

| 类型 | 作用 |
| --- | --- |
| `AlignmentCandidate` | 模板匹配候选位姿扰动和代价。 |
| `PoseAlignmentResult` | 模板对齐结果和投影点。 |
| `SubpixelEdgeCandidate` | 法向搜索得到的亚像素边缘候选点。 |
| `SubpixelEdgeSet` | 焊缝点和第二边点集合。 |

重要内部函数分组：

| 分组 | 函数 | 作用 |
| --- | --- | --- |
| 基础转换 | `makeCameraMatrix()`、`toColumnMat()`、`toVec3d()`、`toPixelPoint()`、`toCvDrawPoint()` | OpenCV、Eigen/自定义像素类型之间的数据转换。 |
| 投影与采样 | `computePerturbedPose()`、`projectEdges()`、`sampleProjectedEdges()`、`projectAndSampleEdges()` | 将 3D 模板点按位姿投影到图像，并按间隔采样。 |
| 模板匹配 | `chamferCost()`、`computeInlierRatio()`、`keepBestCandidates()`、`evaluateCandidate()`、`evaluateGrid()`、`evaluateAroundSeeds()`、`alignCurveTemplate()` | 三层 DT-Chamfer 搜索，得到最优 `rvec/tvec` 和代价指标。 |
| 边缘预处理 | `extractEdges()`、`makeDistanceMap()` | 生成边缘图和距离变换图。 |
| 焊缝点提取 | `bilinearSample64()`、`tangentAt()`、`percentile95()`、`robustLineFilter()`、`extractDistanceFallbackPoints()`、`extractSubpixelEdgePoints()` | 沿先验法向搜索 Step/Ridge 候选，直线件做鲁棒直线过滤，必要时回退到距离图搜索。 |
| 曲线与起点 | `makeCurveExtractArc()`、`lineFitRmse()`、`curveFixedArcStartPoint()`、`isPointInImage()` | 处理曲线弧段、计算拟合误差和起点合法性。 |
| 可视化 | `drawPolyline()`、`drawTemplateOverlay()`、`drawContoursOverlay()`、`drawWeldOverlay()` | 生成模板、轮廓、焊缝点和起始点叠加图。 |

## 图像检测文件

### `IbvsRedFeatureDetector.h/.cpp`

检测焊枪红色双特征点。该文件依赖 OpenCV，只在 `I2R_RLC_ENABLE_OPENCV=ON` 时编译。

公开类型：

| 类型 | 作用 |
| --- | --- |
| `IbvsRedBlobCandidate` | 单个红色 blob 候选，记录质心、面积、形状、颜色和评分。 |
| `IbvsRedFeatureDetectionResult` | 双点检测结果，包含是否 accepted、特征点、距离和候选列表。 |
| `IbvsRedFeatureDetectorConfig` | 红色分割和候选筛选参数。 |

公开类：

- `IbvsRedFeatureDetector`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `IbvsRedFeatureDetector()` | 使用默认参数构造检测器。 |
| `IbvsRedFeatureDetector(const IbvsRedFeatureDetectorConfig&)` | 使用指定参数构造检测器。 |
| `detect(const cv::Mat&) const` | 输入 BGR 图像，输出红色双特征点检测结果。 |

重要内部函数：

| 函数 | 作用 |
| --- | --- |
| `applyGammaCorrection(const cv::Mat&) const` | 图像 gamma 校正。 |
| `extractRedMask(const cv::Mat&, cv::Mat*) const` | HSV 红色区域分割。 |
| `extractCandidates(const cv::Mat&, const cv::Mat&) const` | 从 mask 中提取红色 blob 候选。 |
| `selectBestPair(const std::vector<IbvsRedBlobCandidate>&) const` | 从候选中选择最合理的双点组合。 |
| `clamp01(double)` | 将分数限制到 `[0, 1]`。 |
| `blobScore(...)` | 计算 blob 候选评分。 |
| `pointDistance(...)` | 计算两个像素点距离。 |

### `IbvsYoloDetector.h/.cpp`

封装 ONNX Runtime YOLO 推理，用于输出焊枪 ROI 候选。该文件依赖 OpenCV 和 ONNX Runtime，只在 `I2R_RLC_ENABLE_ONNX_RUNTIME=ON` 时编译。

公开类型：

| 类型 | 作用 |
| --- | --- |
| `IbvsYoloDetection` | 单个 YOLO 检测结果，包含 `cv::Rect2f box` 和 `score`。 |
| `IbvsYoloDetectorConfig` | 输入尺寸、置信度阈值、NMS 阈值和最大检测数。 |

公开类：

- `IbvsYoloDetector`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `IbvsYoloDetector(const std::string&, const IbvsYoloDetectorConfig&)` | 加载 ONNX 模型并创建 detector。 |
| `~IbvsYoloDetector()` | 释放 ONNX Runtime 资源。 |
| `detect(const cv::Mat&) const` | 输入 BGR 图像，输出 ROI 候选。 |

实现结构：

| 类型/函数 | 作用 |
| --- | --- |
| `IbvsYoloDetector::Impl` | PImpl 实现，隐藏 ONNX Runtime 细节。 |
| `Impl::detect(const cv::Mat&)` | 完成图像预处理、推理、输出解析和 NMS。 |
| `Impl::iou(const cv::Rect2f&, const cv::Rect2f&)` | 计算两个 ROI 的 IoU，用于 NMS。 |

## 服务边界文件

### `IIbvsService.h`

定义 IBVS 服务接口，供 UI 或上层调度模块以服务状态方式感知 IBVS。

公开类型：

| 类型 | 作用 |
| --- | --- |
| `IbvsServiceState` | 服务状态：`Unavailable`、`Idle`、`Running`、`Error`。 |
| `IbvsStartResult` | 服务启动结果，包含状态和消息。 |

公开接口：

| 类型/函数 | 作用 |
| --- | --- |
| `IIbvsService` | IBVS 服务抽象基类。 |
| `virtual ~IIbvsService()` | 虚析构函数。 |
| `start()` | 启动服务或返回当前服务状态。 |

### `NullIbvsService.h/.cpp`

提供 IBVS 空服务实现，用于应用组装时保留服务边界。

公开类：

- `NullIbvsService`

公开函数：

| 函数 | 作用 |
| --- | --- |
| `start() override` | 返回当前空服务状态和提示信息。 |

## 测试文件

| 测试文件 | 测试目标 | 覆盖内容 |
| --- | --- | --- |
| `tests/test_ibvs_core.cpp` | `i2r_rlc_ibvs_core_tests` | 核心类型、期望特征、起始点几何、reader 临时文件、OLS/KF、速度求解。 |
| `tests/test_ibvs_offline_replay.cpp` | `i2r_rlc_ibvs_offline_replay_tests` | 真实 txt 数据读取、Orthogonal J0 对照、在线特征/ROI/速度/位姿解析、小规模 KF 和速度回放。 |
| `tests/test_ibvs_opencv.cpp` | `i2r_rlc_ibvs_opencv_tests` | OpenCV 条件下红色双点检测 smoke。 |
| `tests/test_ibvs_torch_feature_crops.cpp` | `i2r_rlc_ibvs_torch_feature_crops_tests` | 焊枪裁切图中的红色双特征点提取和可视化留档。 |
| `tests/test_ibvs_start_point_offline.cpp` | `i2r_rlc_ibvs_start_point_offline_tests` | 端点、交点和曲线最近点等起始点几何算法。 |
| `tests/test_ibvs_start_point_image_chain.cpp` | `i2r_rlc_ibvs_start_point_image_chain_tests` | 四个工件裁切图的边缘提取、模板对齐、焊缝点提取和起始点识别。 |
| `tests/test_ibvs_velocity_diagnostics.cpp` | `i2r_rlc_ibvs_velocity_diagnostics_tests` | KF 更新、速度求解、误差/速度/NIS/限幅逐帧诊断。 |
| `tests/test_ibvs_yolo_smoke.cpp` | `i2r_rlc_ibvs_yolo_smoke_tests` | YOLO 模型加载、黑图推理和真实图合法 ROI 检出。 |
| `tests/test_ibvs_image_chain.cpp` | `i2r_rlc_ibvs_image_chain_tests` | YOLO ROI、红色双点和期望特征的真实图像小链路。 |

## 主要数据流

### 文本回放链路

1. `IbvsOfflineDataReader` 读取特征、ROI、速度、相机位姿和初始雅可比。
2. `IbvsJacobianEstimator` 使用 OLS 或文件 J0 初始化 4x3 雅可比。
3. `jacobianEK()` 根据相邻帧相机位移和特征增量更新雅可比。
4. `IbvsVelocitySolver` 根据当前特征误差和雅可比求解相机速度。
5. 测试记录误差、NIS、速度、限幅和失败状态。

### 图像感知链路

1. `IbvsYoloDetector` 从原图中检测焊枪 ROI。
2. `IbvsRedFeatureDetector` 在 ROI 或裁切图中检测红色双特征点。
3. `IbvsStartPointImageDetector` 在工件裁切图中识别焊缝起始点。
4. `IbvsFeatureGeometry` 根据红色双点和起始点生成期望特征。
5. 结果进入 `IbvsVelocitySolver` 的误差和速度求解入口。

## 当前边界

- `ibvs` 只提供算法库和服务边界，不直接管理 UI、真实相机线程、RobotComm 或机器人运动闭环。
- OpenCV 能力通过 `I2R_RLC_ENABLE_OPENCV` 控制；ONNX Runtime 能力通过 `I2R_RLC_ENABLE_ONNX_RUNTIME` 控制。
- 当前图像测试以离线图片和裁切图为输入，输出过程图、CSV 和 summary，便于人工复核。
- 速度诊断已经验证数值链路可运行，但限幅比例和期望特征策略仍需在进入闭环前继续分析。
