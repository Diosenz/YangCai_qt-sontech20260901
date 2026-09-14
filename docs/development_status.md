# 开发状态

## 目的

本文档是 `i2r_rlc` 的项目状态快照，用于在 Codex 上下文压缩、中断恢复或新任务开始时快速恢复关键约束、阶段状态、构建状态和下一步方向。后续每轮任务结束时应优先更新本文档。

## 当前项目目标

- 从旧 MFC 工程 `SontechCodex/SonTech` 中抽离 `CRobotleadingDlg` 相关功能。
- 在 `SontechCodex/i2r_rlc` 中重构为独立 Qt Widgets + CMake 工业视觉软件。
- 首版优先焊缝跟踪主流程。
- 数字孪生和 IBVS 首版只保留接口、mock 或 no-op 实现。
- 迁移不是 MFC 到 Qt 的逐行翻译，而是重新拆分 UI、应用流程、算法、相机、机器人通信、配置和线程边界。

## 当前硬性约束

- `SontechCodex/SonTech` 是旧 MFC 工程，只读，绝对不允许修改。
- 所有新增、修改、文档、CMake 和 Qt 代码只能位于 `SontechCodex/i2r_rlc`。
- 实际新工程目录名是 `i2r_rlc`，不要创建大小写不同的重复目录。
- 顶层窗口使用 `QWidget`，当前为 `ui/MainWidget`。
- 不使用 `QMainWindow`。
- 不使用 qmake，只使用 CMake。
- CMake/MSVC 目标必须保持 `/utf-8`。
- 文本文件使用 UTF-8 或 UTF-8 with BOM，避免 VS2022 中文乱码。
- 公共类、接口、重要函数和重要数据结构使用 Doxygen 风格注释。
- 除必要技术关键字、类名、函数名、模块名、协议字段或标准术语外，注释使用中文。
- 默认使用 `RobotComm dry-run`。
- 在协议语义、dry-run、用户授权和安全验证完成前，不允许发送真实 DX200 运动命令。
- 当前不接入真实 Vimba、Basler、HALCON、DX200、OpenCascade、VTK、YOLO/ONNX；A3 起若图像点到空间点重建或焊接点云/轨迹匹配确实需要，`vision/geometry` 可按焊接纠偏用途使用 Eigen/PCL，但仍不允许 HALCON。
- `vision` 算法迁移边界不使用 Qt 类型，不 include Qt，不使用 `QImage`、`QString`、`QFile`、`QObject`、`QWidget`；`QImage -> cv::Mat`、相机 SDK buffer -> `cv::Mat`、文件读取 -> `cv::Mat` 等转换由 `ui` / `camera` / app adapter 层在调用 `vision` 前完成。
- OpenCV 3.4.3 已用于 vision 算法迁移；`QImage -> cv::Mat` 的适配由 `ui/UiController` 完成，不再通过 `vision` 内部 adapter。
- `IWeldDetector`、`StubWeldDetector`、`OpenCvImageUtils` 和旧 `VisionPipeline` 适配类已移除；当前 `vision` 算法编排入口统一为 `CWeldVisionAlgorithmPipeline`。

## 已完成阶段

| 阶段 | 状态 | 说明 |
| --- | --- | --- |
| 阶段 1：旧 `CRobotleadingDlg` 只读审查 | Done | 已识别旧窗口是 UI、相机、机器人通信、图像处理、标定、纠偏、数字孪生、IBVS 和配置的综合调度器。 |
| 阶段 2：架构文档与迁移计划 | Done | 已建立 legacy review、dependency map、migration plan、new architecture、threading design、risk log、runtime data layout、calibration file format 等文档。 |
| 阶段 3：Qt Widgets + CMake 最小骨架 | Done | 已创建可在 VS2022 中打开、配置、构建、运行的最小 Qt Widgets 应用骨架。 |
| 阶段 4：运行时路径、配置仓库和标定解析 | Done | 已实现 `AppPaths`、`ConfigRepository`、`SensorCalibrationParser` 和示例 `SENSORPARA.txt`。 |
| 阶段 4 主程序验证 | Done | 用户已在 VS2022 中运行主程序，验证启动、运行时目录和 `config/calibration/SENSORPARA.txt` 加载成功。 |
| 阶段 4 轻量测试 | Done | 已新增 `i2r_rlc_config_tests`。 |
| 阶段 5：离线图像输入与显示通路 | Done | 用户已在 VS2022 中验证离线图像可以显示。 |
| 阶段 5 UI 固定布局调整 | Done | 图像显示画布已按旧 MFC `IDC_INPUT_HWINDOW` 的 `646:482` 比例调整，当前为 `775 x 578`。 |
| 阶段 5 离线相机测试 | Done | 已新增 `i2r_rlc_offline_camera_tests`。 |
| 阶段 6：离线帧到早期视觉适配接口 | Superseded | 早期曾打通 `CameraFrame -> VisionPipeline -> WeldVisionPipelineResult` 数据流；该旧适配类和 `i2r_rlc_vision_pipeline_tests` 已移除。当前由 `UiController -> CWeldVisionAlgorithmPipeline -> UiFrameProcessingResult` 承接。 |
| 阶段 6 收尾：视觉处理状态到 WorkflowController | Superseded / removed | 旧 no-op `WeldTrackingWorkflowController` 与 `WorkflowState` 已删除；UI 当前通过 `COnlineCorrectionController` 显示在线纠偏阶段，视觉帧处理只更新普通 UI 状态和日志。 |
| 阶段 7 前置：OpenCV 3.4.3 CMake 接入 | Done / Runtime PATH fix applied | 已新增 `I2R_RLC_ENABLE_OPENCV`、`qt-debug-opencv` preset 和 `i2r_rlc_opencv_smoke_tests`；用户发现 OpenCV preset 运行时缺 `Qt6Widgetsd.dll`，已补充 preset 和脚本的 Qt/OpenCV runtime PATH。 |
| 阶段 7：OpenCV 基础图像预处理接口 | Superseded | 早期 `OpenCvImageUtils` 和 `i2r_rlc_vision_opencv_tests` 已移除；Qt 图像适配由 `UiController` 承担，`vision` 只接收上层准备好的 `cv::Mat`。 |
| 阶段 8 前置：焊缝检测结果结构与最小算法接口 | Superseded | `WeldVisionPipelineResult`、`CWeldVisionAlgorithmPipeline`、`IWeldDetector`、`StubWeldDetector` 已移除；vision 对外算法入口统一为 `CWeldVisionAlgorithmPipeline`。 |
| dev 到 integrate 的 vision/control 复用审查 | Done / Docs only | 已新增 `docs/dev_to_integrate_vision_control_reuse_audit.md`。本轮只读审查 `origin/dev` 中可迁移到 `origin/integrate` 新架构 `vision` 与 `control` 的文档、代码、测试和旧工程行为；未修改 `src`、`tests`、`CMakeLists.txt`、`CMakePresets.json`，未连接真实设备，未发送真实 TCP。下一步建议先迁移 vision/control 文档，再迁移 OpenCV-only vision 内容和 control dry-run workflow。 |
| V2-A0：旧工程焊接视觉算法审查与 HALCON 去依赖化迁移方案 | Done / Docs only | 已新增并修正 `docs/vision_legacy_algorithm_migration_audit.md`。本轮只读审查旧工程 `robotleadingDlg`、`robotleading_method`、`SWv1/ImagePro`、`CImageDenoise`、`GapDetector`、KCF/STC、HALCON、PCL/ICP/点云与 `CalthelaserpositionLoop`；当前 `dev` 阶段只处理 `vision` / `control`，Codex 本阶段不实现 `localization`、`ibvs`、`digital_twin`。与焊接纠偏直接相关的图像点到空间、ICP、点云匹配、PID/fuzzy PID、纠偏量和速度覆盖阶段性放入 `vision`；`control` 暂时不承载 PID/fuzzy PID 代码实现，后续是否拆分再决定。未修改 `src`、`tests`、`CMakeLists.txt`、`CMakePresets.json`；当前仍未接入 HALCON、真实相机、DX200，也未发送 TCP。下一步建议进入 V2-A1 OpenCV-only 与焊接纠偏观测量代码迁移设计。 |
| V2-A1.0：vision 算法逐步迁移路线图 | Done / Docs only | 已新增并修正 `docs/vision_algorithm_migration_sequence.md`。本轮固定 `vision` 内部推荐结构：`common`、`preprocess`、`stripe`、`feature`、`tracking`、`geometry`、`correction`、`pipeline`；固定旧函数到新类的迁移顺序、每阶段测试 target 建议、HALCON 剥离策略和“不整文件复制”规则。已明确 `OpenCvImageUtils` 只作为旧阶段 Qt/QImage 适配工具，不进入 V2-A1 算法迁移路线；`vision` 不 include Qt，不使用 `QImage`、`QString`、`QFile`、`QObject`、`QWidget`，只接收上层准备好的 `cv::Mat` 或数值结构。未修改 `src`、`tests`、CMake 或 `SonTech`；未接入 HALCON，未连接真实设备。下一步进入 V2-A1.1：纯 OpenCV `cv::Mat` 输入的 `common` + `preprocess`。 |
| V2-A1.1：common + CImagePreprocessor | Done / VS2022 verified | 已新增 `VisionImageTypes` 与纯 `cv::Mat` 输入的 `CImagePreprocessor`，并新增 `i2r_rlc_vision_image_preprocessor_tests`。用户已在 VS2022 中确认该测试返回 0。本轮只实现 ROI 校验/裁剪、灰度转换、8-bit 单通道归一化和灰度统计；未使用 `OpenCvImageUtils`，未在新增 vision 代码中引入 Qt，未接入 HALCON、PCL、真实相机或 DX200。 |
| V2-A1.2：CImageDenoise + CImageMorphologyUtils | Done / VS2022 verified | 已新增纯 `cv::Mat` 输入的 `CImageMorphologyUtils` 与 `CImageDenoise`，并新增 `i2r_rlc_vision_image_denoise_tests`。用户已在 VS2022 中确认 `i2r_rlc_vision_image_denoise_tests` 与 `i2r_rlc_vision_image_preprocessor_tests` 通过。本轮抽取旧 `CImageDenoise`、`ImagePro`、`ImageProFeng` 中基础降噪、Gabor 和形态学语义，不复制旧文件，不使用 `OpenCvImageUtils`，不写调试图，不接 Qt/HALCON/PCL/真实相机/DX200/TCP。未实现 `CLaserStripeExtractor`、`ImagePro::LaserWidthEstimate`、`CSubpixelCenterlineTools`、`GapDetector`、KCF/STC、ICP、PID/fuzzy PID 或 pipeline。 |
| V2-A1.2R：按旧 GaborFilter 语义修正 CImageDenoise | Implemented / Pending VS2022 re-verification | 已保留原通用 `CImageDenoise::gaborDenoise`，并新增 `GaborFilterOptions`、`GaborFilterAngleResult`、`MyLaserImagePoint` 最小兼容结构和 `i2r_rlc_vision_gabor_filter_tests`。本轮确认旧 `GaborFilter(cv::Mat&, WeldType, vector<MyLaserImagePoint>, WeldPt)` 中 `WeldType` 只做有效枚举检查，不改变滤波算法路径，因此新接口不引入 `WeldType`。新路径按 `SimPts + WeldPt` 计算 `pos_th`，默认参数对齐旧工程 `kernel_size=21`、`pos_sigma=4`、`pos_lm=90`、`pos_psi=90`，并复刻 `Process_P / mkKernel / filter2D(CV_8U)`；不使用 `SwMySensor.h`，不写 `gabor.png` / `gabor255.png`，不接 Qt/HALCON/PCL/真实相机/DX200/TCP。用户已验证上一版逻辑通过，本轮仅调整 API 命名、Doxygen 注释和实现注释，需重新运行新 target 名称。 |
| V2-A1.3：CLaserStripeExtractor | Done / VS2022 verified | 已新增纯 `cv::Mat` 输入的 `CLaserStripeExtractor`，并新增 `i2r_rlc_vision_laser_stripe_extractor_tests`。用户已在 VS2022 中确认 `i2r_rlc_vision_laser_stripe_extractor_tests`、`i2r_rlc_vision_image_preprocessor_tests`、`i2r_rlc_vision_image_denoise_tests` 通过。本轮旧来源为 `ImagePro::LaserWidthEstimate` 的列统计/宽度估计语义和 `GapDetector::extractCandidatesOneColumn` 的列候选峰语义；只实现 mask、阈值、有效像素比例、候选 ROI、列宽度统计和整数像素局部峰候选。不实现中心线、灰度重心、抛物线亚像素、Hessian/Steger、二阶 DP、完整 GapDetector、KCF/STC、ICP、PID/fuzzy PID 或 pipeline；未使用 `OpenCvImageUtils`，未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A1.4：CSubpixelCenterlineTools | Done / VS2022 verified | 已新增纯 `cv::Mat` 和数值输入的 `CSubpixelCenterlineTools`，并新增 `i2r_rlc_vision_subpixel_centerline_tests`。用户已在 VS2022 中确认 `i2r_rlc_vision_subpixel_centerline_tests`、`i2r_rlc_vision_image_preprocessor_tests`、`i2r_rlc_vision_image_denoise_tests`、`i2r_rlc_vision_laser_stripe_extractor_tests` 通过。本轮旧来源为 `GapDetector::SubpixelParabola`、`RobustColumnScale`、`extractCandidatesOneColumn` 局部峰思想，以及 `ImageProShare` LOG/CoM/LOGCore 中心点语义；只实现抛物线亚像素、灰度重心、局部分位尺度、局部对比度和质量评价。不实现完整中心线 detector、二阶 DP、Hessian/Steger 完整算法、完整 GapDetector、CWeldFeatureDetector、ICP、PID/fuzzy PID、KCF/STC 或 pipeline；未使用 `OpenCvImageUtils`，未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A1.5：中心线最小版 | Superseded / removed | 早期最小中心线实现只覆盖列峰、亚像素和简单连续性过滤，已被 `CDpStripeCenterlineDetector::detectCenterline` 取代。旧最小版头文件、源文件和实现依赖已删除，中心线公共结果类型保留在 `LaserCenterlineTypes.h`。 |
| V2-A1.5B：中心线 replay 批量评估 | Updated to DP | `i2r_rlc_vision_centerline_replay_tests` 已改为调用 `CDpStripeCenterlineDetector::detectCenterline`，继续生成 `output/reports/centerline_replay_report.csv`。该 replay 现在评估 GapDetector 正式 DP 中心线，而不是早期最小版。 |
| V2-A2：CWeldFeatureDetector / CGapFeatureDetector 最小版本 | Done / VS2022 verified | 已新增 `WeldFeatureTypes`、`CGapFeatureDetector`、`CWeldFeatureDetector` 和 `i2r_rlc_vision_gap_feature_detector_tests`。用户已在 VS2022 中确认 V2-A2 最小版本验证通过。本轮旧来源为 `ImagePro`、`ImageProFeng`、`ImageProDong`、`ImageProShare`、`GapDetector` 中的焊缝点、坡口/间隙、左右边缘检测语义；只实现最小 OpenCV-only 行截面左右边缘、gap 宽度和 weldPoint 中点候选。不实现完整 ImagePro 多焊缝类型检测、BSnake/NURBS/XLD、完整 GapDetector、完整二阶 DP、Hessian/Steger、ICP、图像点到空间、PID/fuzzy PID、KCF/STC 或 WeldVisionPipeline；未使用 `OpenCvImageUtils`，未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2B：CGapFeatureDetector replay 批量评估 | Updated to DP | `i2r_rlc_vision_gap_feature_replay_tests` 已改为固定执行 `CDpStripeCenterlineDetector::detectCenterline -> CWeldFeatureDetector::detect`，并生成 `output/reports/gap_feature_replay_report.csv`。该链路现在使用旧 `GapDetector::extractLaserCenterline` 的二阶 DP 迁移实现。 |
| V2-A2.1：CGapFeatureDetector 旧工程 gap/edge 语义增强 | Done / VS2022 verified | 已增强 `CGapFeatureDetector` 的 2D gap/edge 候选能力，新增 `GapEdgeCandidate`、左右候选数量、失败原因、选中行和边缘对比度诊断字段，并扩展 `i2r_rlc_vision_gap_feature_detector_tests`。用户已在 VS2022 中确认验证通过。本轮旧来源固定为 `ImagePro::ButtSeamDet`、`ButtStartPtDet`、`ImageProShare::split_line/ransac_line/line_interfast` 以及 `robotleadingDlg::getWeldGap` 中 ROI/seam 附近搜索语义；本轮不做 replay 批量评估，不实现完整二阶 DP、完整 GapDetector、PCL/computeGapNew、图像点到空间、T/对接/搭接完整分类、HALCON XLD、BSnake、NURBS、KCF/STC 或 WeldVisionPipeline；未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2.2：ButtSeam 对接焊缝语义最小版本 | Superseded / removed | 该阶段的最小对接候选属于早期抽象，旧工程没有同名生产入口；当前对接焊缝语义由 `CActiveContourWeldFeatureDetector` 承接。早期 detector、测试 target 和独立语义结果结构已从生产代码移除，`CWeldFeatureDetector` 仅保留 gap、endpoint、groove 聚合。 |
| V2-A2.3：线段/RANSAC 几何工具最小版本 | Done / VS2022 verified | 已新增 `CLineGeometryTools` 和 `i2r_rlc_vision_line_geometry_tests`，迁移旧 `ImageProShare::split_line/ransac_line/line_interfast` 的低阶点列分段、确定性 RANSAC 直线拟合和两线交点语义。用户已在 VS2022 中确认验证通过。本轮只新增 OpenCV-only / STL 点列工具；当前由主动轮廓、起点检测和仿真线配准等模块复用，不再服务早期最小 T/Butt/Lap detector。未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2.4：TJoint T 型焊缝语义最小版本 | Superseded / removed | 该阶段的最小 T 型候选属于早期抽象，旧工程没有同名生产入口；当前 T 型焊缝语义由 `CActiveContourWeldFeatureDetector` 承接，包含曲线极值、ROI、split/RANSAC、交点和历史约束。早期 detector 与测试 target 已移除。 |
| V2-A2.5：LapSeam 搭接焊缝语义最小版本 | Superseded / removed | 该阶段的最小搭接候选属于早期抽象，旧工程没有同名生产入口；当前搭接焊缝语义由 `CActiveContourWeldFeatureDetector` 承接。早期 detector 与测试 target 已移除。 |
| V2-A2.6：CWeldEndpointDetector 起点/终点语义最小版本 | Done / VS2022 verified | 已新增 `WeldEndpointResult`、`CWeldEndpointDetector` 和 `i2r_rlc_vision_endpoint_detector_tests`，并增强 `CWeldFeatureDetector::detect`，在现有 gap 结果旁输出 2D 起点/终点候选。用户已在 VS2022 中确认验证通过。本轮旧来源为 `ImagePro::StartPtDet`、`EndPtsFinding` 和 `ButtStartPtDet` 的起点/终点候选语义；只从中心线有效点列提取首尾 endpoint，并用 `gapFeature.weldPoint` 做基础一致性检查。不迁 `getDetImagePoint` / `GetDetImagePt` 路由，不做机器人坐标转换，不实现完整 GapDetector、geometry/ICP、PID/fuzzy PID、KCF/STC 或 WeldVisionPipeline；未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2.7：CWeldSkeletonFeatureTools 骨架语义最小版本 | Done / VS2022 verified | 已新增 `WeldSkeletonExtractionOptions`、`WeldSkeletonFeatureResult`、`CWeldSkeletonFeatureTools` 和 `i2r_rlc_vision_skeleton_feature_tests`。用户已在 VS2022 中确认验证通过。本轮旧来源为 `ImageProFeng::SkeletonExtraction` 和 `ImagePro` 中端点/轮廓相关 skeleton 语义；只实现 OpenCV-only 二值化、小连通域过滤、Zhang-Suen 细化、骨架点列、端点/分叉点和连通分量统计。不迁完整 T/Lap/坡口复杂语义，不接 BSnake/NURBS/XLD、geometry/ICP、PID/fuzzy PID、KCF/STC 或 WeldVisionPipeline；未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2.8：CGrooveEdgeDetector 坡口/复杂间隙边缘最小版本 | Done / VS2022 verified | 已新增 `GrooveEdgeResult`、`CGrooveEdgeDetector` 和 `i2r_rlc_vision_groove_edge_detector_tests`。用户已在 VS2022 中确认验证通过。本轮旧来源为 `ImagePro`、`GapDetector` 中坡口/间隙/左右边缘语义、`ImageProFeng::SkeletonExtraction` 以及 `ImageProShare::split_line/ransac_line/line_interfast` 低阶几何语义；只实现 OpenCV-only 多中心线点采样、左右边缘点列拟合、V 型坡口和平行 gap 基础候选。不迁完整 Butt/T/Lap、BSnake/NURBS/XLD、图像点到空间、ICP、PID/fuzzy PID、KCF/STC 或 WeldVisionPipeline；未接 Qt/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2.9：CWeldFeatureDetector 语义融合与 Groove 接入 | Done / VS2022 verified | 已将 `CGrooveEdgeDetector` 作为辅助结果接入统一 `WeldFeatureResult::grooveFeature`，并增强 `CWeldFeatureDetector::detect` 输出 Butt/TJoint/Lap/Endpoint/Groove 支持诊断。用户已在 VS2022 中确认 `i2r_rlc_vision_feature_fusion_tests` 验证通过。本轮只做 feature 层结果融合，不新增 `WeldSeamType::Groove`，不进入 pipeline/control/geometry，不实现完整 Butt/T/Lap/Groove 复杂语义、图像点到空间、ICP、PID/fuzzy PID、KCF/STC 或真实设备接入。 |
| V2-A2.10：ImageProShare 完整迁移与 HALCON 剥离 | Done / VS2022 verified | 已新增 `CLaserLineResponseTools`、`CContourControlPointTools`，并增强 `CLineGeometryTools`。本轮将旧 `ImageProShare::LOG_new/CoM_new/LOGCore/EfficientMask/DragOfflinePt/BSnakeGrow/BSnake_HandleConPt/split_line/ransac_line/ransac_demo/line_interfast` 从“语义覆盖”推进为新工程可调用 OpenCV/C++ API；以 `cv::Mat`、`std::vector`、`cv::Point2d` 和状态结构替代裸指针、`HTuple/HObject/XLD/NURBS`，新增 `i2r_rlc_vision_image_pro_share_tools_tests`。用户已在 VS2022 中确认该测试通过。本轮新增 `docs/image_pro_share_migration_mapping.md` 记录旧数据结构/函数到新文件、新结构和新函数的映射。未使用 `OpenCvImageUtils`，未接 Qt 类型/HALCON/PCL/真实相机/DX200/TCP。 |
| V2-A2.10R：ImageProShare NURBS 修正 | Done / VS2022 verified | 已将 `CContourControlPointTools::interpolateControlCurve` 从 Catmull-Rom 采样修正为 tinyspline NURBS/B-spline 采样；`vcpkg.json` 新增 `tinyspline`，CMake 新增 `tinysplinecxx::tinysplinecxx` 链接。该修正用于更贴近旧 HALCON `GenContourNurbsXld / GenNurbsInterp` 语义，但 public API 仍只暴露 `std::vector<cv::Point2d>` 和状态结构，不暴露 HALCON 或 tinyspline 类型。已扩展 `i2r_rlc_vision_image_pro_share_tools_tests` 的 NURBS 曲线采样、2 点线性 fallback、重复点去重和控制点不足边界；用户已在 VS2022 中确认验证通过。未接 HALCON、Qt 类型、PCL、真实相机、DX200 或 TCP。 |
| V2-A2.10P：ImageProShare 注释补强与 penzui 离线测试 | Implemented / Pending VS2022 verification | 已为 `CLaserLineResponseTools`、`CContourControlPointTools`、`CLineGeometryTools` 中 ImageProShare 迁移接口补充 Doxygen 参数、单位、旧函数来源和失败边界说明，并在 `.cpp` 中补充 LOGCore、LOG_new、CoM_new、DragOfflinePt、BSnakeGrow、tinyspline NURBS 和 line_interfast 关键实现注释。已新增 `i2r_rlc_vision_image_pro_share_replay_tests`，读取 `data/replay/penzui/imagecatch`、`simulatedLaser.txt`、`simulatedWeldpt.txt` 做离线 2D 工具 replay，并生成 `output/reports/image_pro_share_replay_report.csv`；缺少离线数据时输出 SKIP 并返回 0。本轮未修改算法行为，未接 HALCON、Qt 类型、PCL、真实相机、DX200 或 TCP。 |
| V2-A2.11：主动轮廓焊缝检测完整迁移 | Implemented / Pending VS2022 verification | 已新增 `CActiveContourWeldFeatureDetector`、`ActiveContourDetectionOptions`、`ActiveContourDetectionState`、`ActiveContourDetectionResult`、`LaserWidthProfile` 和 `ActiveContourCurve`，用通用命名迁移旧主动轮廓检测链路的 active runtime 行为：激光宽度估计、初始曲线、BSnake 风格迭代、T/对接/搭接分支和多帧兜底。新增 `docs/active_contour_weld_feature_migration.md` 与 `i2r_rlc_vision_active_contour_feature_tests`。本轮不修改旧工程，不新增第三方库，不接 HALCON、Qt 类型、PCL、真实相机、DX200 或 TCP；`CWeldFeatureDetector` 默认行为不变。 |
| V2-A2.11P：主动轮廓 penzui replay 测试 | Implemented / Pending VS2022 verification | 已新增 `i2r_rlc_vision_active_contour_replay_tests`，读取 `data/replay/penzui/imagecatch` 与 `simulatedWeldpt.txt`，对前 40 帧真实 replay 数据执行 `CActiveContourWeldFeatureDetector`，并生成 `output/reports/active_contour_replay_report.csv`。测试记录焊缝类型、焊缝点、gap 端点、曲线点数量、跨帧历史兜底、置信度和仿真焊缝点距离诊断；缺少 replay 数据时输出 SKIP 并返回 0。本轮未接 HALCON、Qt 类型、PCL、真实相机、DX200 或 TCP。 |
| V2-A2.11R：主动轮廓控制点处理补强 | Implemented / Codex target verified | 已按审查结果补强 `CContourControlPointTools::adjustOfflineControlPoints` 与 `CActiveContourWeldFeatureDetector`：`iterateCurve()` 在 LOG 贪婪迭代前先执行 `BSnake_HandleConPt` 等价控制点修正；控制点工具补齐拖拽失败删除、临时曲线暗点补控制点、同列去重、30 pixel 大间距补点和左右增长；`initializeCurve()` 已从旧的简化 LOG/CoM 优先路径补成 `InitBSnake_new` 流程级复刻，覆盖阈值分割、连通域等价填充、closing、骨架化、分支剔除、重叠/浮空段筛选、每列中心、`CoM_new` 亚像素修正和 tinyspline 曲线生成。已扩展 `i2r_rlc_vision_image_pro_share_tools_tests` 与 `i2r_rlc_vision_active_contour_feature_tests`，并为 ImageProShare OpenCV 测试补充 CTest runtime PATH。未修改旧工程，未新增第三方库，未接 HALCON、Qt 类型、PCL、真实相机、DX200 或 TCP。 |
| V2-A2.12：ImageProFeng 主链路迁移 | Implemented / Codex target verified | 已新增 `CDirectionalMorphologyFilter`、`SimulatedLaserObservation`、`CSimulatedLaserReplayParser`、`CSimulatedLaserRegistrationMatcher`、`CSimulatedLaserTemplateMatcher`，并补充公共 `CWeldSkeletonFeatureTools::extractLaserSkeleton`。本轮迁移旧 `ImageProFeng` 仿真激光线主链路：实时 observation 输入、`$ / WeldImgPnt / OriLaserImgPnts / #` 兼容 replay 解析、左右方向形态学增强、骨架中心线提取、HoughLinesP 备用骨架、T 型 coarse matching、T 型分段/RANSAC、LoGSke 风格局部搜索、affine 匹配和模板匹配。模板匹配已将 `meiqua/shape_based_matching` 以最小源码集 vendoring 到 `Vision/third_party/shape_based_matching`，默认优先使用 shape_based_matching，失败时自动退回现有 OpenCV Chamfer；许可证声明见 `THIRD_PARTY_NOTICES.md`。`SimulatedLaserWeldDetector::detect` 已从公共接口剔除，replay 改为分别调用配准入口和模板入口；新增默认关闭的 `I2R_RLC_ENABLE_SHAPE_MATCHING_EXPERIMENT`、`i2r_rlc_vision_shape_matching_experiment_tests` 和 `i2r_rlc_vision_skeleton_halcon_diff_experiment_tests`，用于单独实验分支比较 HALCON baseline、Chamfer、shape_based_matching 以及 HALCON skeleton / 当前 Zhang-Suen 流程差异。可选实验 target 已改为 bounded diagnostic，避免 CTest 超时；Codex 已在 VS2022 环境下验证 `image_pro_feng`、`skeleton_feature`、`active_contour_feature` 和两个可选实验共 6 个 target 全部通过。HALCON shape model 句柄、显示对象、旧 MFC 全局状态、对接/搭接仿真分支和 NURBS/BSnake 不在本轮重复迁移。未修改旧工程，未接 HALCON、Qt 类型、PCL、真实相机、DX200 或 TCP。 |
| V2-A2.12S：HALCON-like Skeleton 输出修正 | Implemented / Codex target verified | 已修正 `CWeldSkeletonFeatureTools::extractLaserSkeleton` 输出语义：`skeletonMask/skeletonPoints` 现在保留 HALCON-like 完整单像素骨架和分叉，新增 `centerLinePoints/refinedCenterPoints` 给仿真线配准、主动轮廓初始化等后续算法使用；region pruning、LOG 支撑筛选和 Hough fallback 改为只影响派生中心线，不再覆盖完整骨架。同步更新 ImageProFeng 配准入口、主动轮廓初始化和隔离 skeleton 对比报告，避免把“骨架算子差异”和“业务中心线压缩差异”混在一起。未修改旧工程，未新增 HALCON 或 ximgproc 依赖。 |
| V2-A2.12T：ImageProFeng T 几何骨架路径修正 | Implemented / Codex target verified | 已补充 `WeldSkeletonFeatureResult::orderedPathPoints`，用于旧 Feng `SkeletonExtraction -> T_SeamCoarseMatching` 的有序主骨架路径语义：从完整 skeleton 中按连续性、灰度和 LOG 支撑逐列选择真实骨架点并做 CoM 修正，不再对同列多骨架点求平均。`CSimulatedLaserRegistrationMatcher::match` 改为使用该点列，并关闭 T 几何分支中的 Hough 两线平均兜底，避免 V/T/角接结构被压成水平伪中心线。隔离 `reports_branch_comparison` 已按 penzui/chufengkoufan 前 30% 与 zhixian 全量重跑覆盖。未修改旧工程，未新增依赖。 |
| V2-A2.12U：shape_based_matching 归位与 SWv1 ImagePro 审查 | Implemented / Codex target verified | 已将 vendored `shape_based_matching` 从工程根部移动到 `Vision/third_party/shape_based_matching`，并更新 vision CMake、第三方声明和 ImageProFeng 映射文档，明确该第三方只属于 vision 模块。新增 `docs/sw_image_pro_migration_audit.md`，按已迁移、语义相近但需补差异、未迁移需评估和不进入生产四类审查 `SWv1/ImagePro.h/.cpp`；结论是 LOG/CoM、主动轮廓 T/对接/搭接、共享几何等主语义已有承接，但 `TempleMatching/SeamTypeDetect` 多模板分类、端点检测完整入口、SAM/ISFFA/Steger 和图像质量指标仍需按生产路径单独补迁评估。未修改旧工程，未新增依赖。 |
| V2-A2.13：SWv1 ImagePro 有机迁移与算法管线入口 | Implemented / Codex target verified | 已新增 OpenCV-only `CWeldVisionAlgorithmPipeline`、`WeldVisionTask`、`WeldVisionPipelineOptions/State/Result`，用现代任务承接旧 `GetDetImagePt(type)` 路由；`type=3` 进入主动轮廓生产主路径，`type=4` 进入端点任务，`type=0/1/2` 建立 SAM/ISFFA/Steger 任务入口。旧 `VisionPipeline`、`IWeldDetector`、`StubWeldDetector` 和 `OpenCvImageUtils` 已移除；`UiController` 负责 Qt/CameraFrame 到 `cv::Mat` 的适配并直接调用算法管线。推荐 include 入口为 `vision/geometry/LineGeometryTools.h`、`vision/stripe/WeldSkeletonFeatureTools.h`、`vision/feature/SimulatedLaserWeldDetector.h`；旧 `vision/feature` 兼容转发头已删除，避免继续形成错误模块归属。新增 `i2r_rlc_vision_algorithm_pipeline_tests` 并更新 weld detection 预期。未修改旧工程，未引入 HALCON。 |
| V2-A2.14：SWv1 ImagePro 剩余算法补迁 | Implemented / Codex target verified | 已按 vision 模块职责补入旧 `ImagePro` 剩余入口：`vision/stripe` 新增 `CSamStripeDetector`、`CIsffaStripeDetector`、`CStegerStripeDetector`、`CStripeLineRepairTools`、`CStripeResponseEvaluationTools`，承接 `LOGandSAM`、`LOGandISFFA`、`Steger`、`SingleCol_LOG` 和 `RDA`；`vision/feature` 新增 `CWeldStartPointDetector`，承接 `StartPtDet/StartPtDet_Area/ButtStartPtDet/ButtEndPtsFinding` 的端点状态机；`vision/preprocess` 新增 `CImageQualityEvaluator`，补齐 `Deviation/Laplace/Energy/Brenner/Tenegrad`。旧 `TempleMatching + SeamTypeDetect` 依赖 HALCON `.shm` 模板资产，当前没有可用的新模板配置，已从生产 API 和测试 target 中移除。`CWeldVisionAlgorithmPipeline` 中 `type=0/1/2/4` 已不再 unsupported，分别调用 SAM、ISFFA、Steger 和端点检测入口。新增并验证 `i2r_rlc_vision_stripe_sam_tests`、`i2r_rlc_vision_stripe_isffa_tests`、`i2r_rlc_vision_stripe_steger_tests`、`i2r_rlc_vision_start_point_tests`、`i2r_rlc_vision_image_quality_tests`，以及扩展后的 `i2r_rlc_vision_algorithm_pipeline_tests`；新增 `i2r_rlc_vision_sw_image_pro_replay_tests`，对 `penzui/chufengkoufan/zhixian` 做 bounded replay，输出到 CTest `Testing/Temporary/sw_image_pro_replay/` 下的 CSV 和各算法 overlay。验证不修改旧工程，不引入 HALCON，Steger 按 OpenCV 工程替代实现，仍需 replay 校准阈值。 |
| V2-A2.15：Vision 路径归位、最小 ICP 剔除与 ImagePro 函数映射 | Implemented / Pending regression | 已硬删除旧兼容转发头，并将几何、骨架和仿真激光线检测分别归入 `geometry`、`stripe`、`feature` 的真实模块路径；独立匹配模块已取消。已剔除提交 `ca03d02` 中的最小 Eigen point-to-point `WeldIcpMatcher`、源文件、头文件和 `i2r_rlc_vision_icp_matcher_tests` target；保留 `WeldGeometryTypes`、`CWeldImageToSpaceProjector` 和 PCL/localization 兼容内容。新增 `docs/sw_image_pro_function_mapping.md`，按旧 `SWv1/ImagePro.h/.cpp` 函数族映射到新 `preprocess/stripe/feature/geometry/pipeline` API，并同步更新迁移审查与路线图文档。 |
| V2-A2.16：主动轮廓归入 stripe 与焊缝类型数字分支清理 | Implemented / Pending regression | 已新增 `vision/stripe/CActiveContourStripeExtractor`，将 `LaserWidthEstimate / InitBSnake / BSnake / LOGandSnake` 的 Snake 条纹提取能力从 feature 语义中拆出；`CActiveContourWeldFeatureDetector` 只消费条纹曲线并负责 T/对接/搭接焊缝特征分支。已删除旧数字分支字段和映射，焊缝类型选择只保留显式 `ActiveContourSeamMode::{TJoint,Butt,Lap}` 或 `Automatic` 三候选取最优。新增 `i2r_rlc_vision_active_contour_stripe_zhixian_replay_tests`，只对 `data/replay/zhixian` 全量帧执行条纹曲线提取并输出 CSV/overlay，不绘制仿真激光线或仿真焊缝点。 |
| V2-A3.0：Geometry 基础、图像点到空间点重建与 ICP 迁移边界 | Done / VS2022 verified | 已新增 `WeldGeometryTypes`、`CWeldImageToSpaceProjector` 和 `i2r_rlc_vision_image_to_space_tests`。用户已在 VS2022 中确认 `i2r_rlc_vision_image_to_space_tests` 验证通过。本阶段用 Eigen 实现针孔相机射线与观测平面求交、feature 点投影和刚体变换；允许后续 geometry 按焊接纠偏用途使用 Eigen/PCL，但 A3.0 未接 PCL 到 vision 业务代码。旧 `robotleading_method/libicp` 轨迹匹配 ICP 保留为后续真实需求重新迁移来源；不使用 HALCON，不实现 ICP/PID/KCF/STC、真实相机、DX200 或 TCP。 |
| V2-A3.0G：SWv1 GapDetector 焊缝宽度检测迁移 | Implemented / Pending regression | 已将旧 `SWv1/GapDetector` 拆分迁移：`Vision/stripe` 新增 `CDpStripeCenterlineDetector` 承接二阶 DP 激光中心线提取；`Vision/geometry` 复用 `CWeldImageToSpaceProjector::projectImagePoints` 承接点云重建；`Vision/feature` 新增 `CPointCloudGapWidthDetector`，内部使用 PCL SOR、欧式聚类和 Eigen PCA/局部拟合计算 3D 焊缝宽度。新增 `docs/gap_detector_migration_mapping.md`，新增测试 target `i2r_rlc_vision_dp_stripe_detector_tests`、`i2r_rlc_vision_point_cloud_gap_width_tests`、`i2r_rlc_vision_gap_width_replay_tests`，并扩展 `i2r_rlc_vision_image_to_space_tests`。本轮不保留 HALCON，不迁旧调试写文件、MFC 全局状态或应用层调用流程；PCL 只在 feature 实现中使用，公共 API 不暴露 PCL 类型。 |
| V2-A3.1：旧 `robotleading_method/libicp` 整库迁入与 Vision ICP wrapper | Implemented / Codex target verified | 已将旧 `robotleading_method/libicp/src` 的 `icp`、`icpPointToPoint`、`icpPointToPlane`、`kdtree`、`matrix` 核心源码归位到 `Vision/third_party/libicp`，不迁 demo、旧 CMake 或 Matlab wrapper。公共入口已归入 `Vision/include/geometry/CIcpRegistrationMatcher` 与 `IcpRegistrationOptions/IcpRegistrationResult`，覆盖 point-to-point、point-to-plane、affine、affineBDD、BDD、scale，支持 `std::vector<WeldSpacePoint>` 和 `WeldPointCloud` 输入，geometry 公共 API 不暴露 `RMatrix::Matrix`、裸 `double*`、kd-tree 或 Boost 类型。KCF 平滑已改为 `Vision/include/track/FilterKalman.h`，使用 libicp 的 `RMatrix::Matrix` 复刻旧 `Filter_Kalman(2)` 更新语义。旧 libicp 的 3D affine/affineBDD 矩阵方程存在行数不匹配风险，wrapper 已将该组合返回 invalid，测试改用二维数据覆盖 affine/affineBDD/BDD、三维数据覆盖 point-to-point/point-to-plane/scale。已通过 `i2r_rlc_vision_libicp_tests` 与 `i2r_rlc_vision_kcf_tracker_tests`。 |
| V2-C1：OnlineCor change 主链路现代化迁移 | Implemented / Codex target verified | 已新增 `COnlineCorrectionController`、`COnlineCorrectionTrajectoryPlanner` 和 `OnlineCorrectionTypes`，将旧 `SWv1/OnlineCor.cpp` 当前使用的 `OnStartinitdet_change/OnStartoncor_change/OnFullonlinecor/OnTimer case 3` 主链路拆成 Control 事件驱动状态机；RobotComm 新增 `RobotCorrectionCommand` / `TeachPointUpdateCommand` DTO，保留机器人回包字段 `onlineReverseCompleteFlag`，不迁入 `onlineFlipFlag`。检测点与 gap 已收敛为 `OnlineCorrectionDetectionSample`，`GlobalCorrection` 枚举唯一决定是否应用全局修正矩阵；旧 no-op `WeldTrackingWorkflowController` / `WorkflowState` 已删除，UI 改为调用 `COnlineCorrectionController`。第一版只生成待发送示教点命令，不真实发送机器人协议；不迁 MFC UI、HALCON 显示、旧调试文件写入或未使用旧分支。映射文档见 `docs/online_cor_migration_mapping.md`；已通过 `i2r_rlc_online_correction_tests`、`i2r_rlc_robot_comm_client_tests`、`i2r_rlc_vision_image_to_space_tests` 和 `i2r_rlc_vision_libicp_tests`。 |
| V2-C1R：在线/离线纠偏启动门禁 | Implemented / Pending VS2022 verification | 已按旧 `robotleadingDlg::OnChangeVirtualStatus()` 的 `sStart != 0 || sMode != 2 || sServoPower != 1` while 语义新增 `CCorrectionRobotStartGate`，并将 Qt 在线/离线纠偏启动改为 pending gate：未收到有效 D 回包或机器人未停止、非 PLAY、伺服未开时反复提示，满足后才发送 JBI 并启动检测。记录见 `docs/online_offline_correction_start_gate_notes.md`，新增 `i2r_rlc_correction_robot_start_gate_tests`。 |
| V2-CalibPose：自动标定位姿模板与修正下发 | Implemented / Pending VS2022 verification | 已复查旧 `Calibration::OnAutocalib()`、`OnPosok()` 和 `COffLineCor::SendNewTeachPts_Calib()`，补齐 Qt 工程中缺失的 `CalibPose.txt` 模板点位下发、按当前 D 回包位姿整体修正、写 `Data/CALID/4 TWO_STEP/CalibPose_new.txt` 和复用旧 `O` 命令下发流程。模板文件已从旧工程复制到 `config/calibration/CalibPose.txt`；新增 `CAutomaticCalibrationPoseController`、`i2r_rlc_automatic_calibration_pose_tests`，并更新 `docs/calibration_capture_migration_mapping.md` 与 `docs/runtime_data_layout.md`。 |

## 当前构建状态

| 项目 | 当前值 |
| --- | --- |
| IDE | Visual Studio 2022 |
| Qt prefix | `C:\Qt\6.7.3\msvc2022_64` |
| CMake configuration | `Qt-Debug` |
| Generator | Ninja |
| Main target | `i2r_rlc.exe` |
| Config test target | `i2r_rlc_config_tests` |
| Offline camera test target | `i2r_rlc_offline_camera_tests` |
| Online correction test target | `i2r_rlc_online_correction_tests` |
| OpenCV smoke test target | `i2r_rlc_opencv_smoke_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CImagePreprocessor test target | `i2r_rlc_vision_image_preprocessor_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CImageDenoise test target | `i2r_rlc_vision_image_denoise_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CLaserStripeExtractor test target | `i2r_rlc_vision_laser_stripe_extractor_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CSubpixelCenterlineTools test target | `i2r_rlc_vision_subpixel_centerline_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| DP centerline test target | `i2r_rlc_vision_centerline_detector_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| DP centerline replay test target | `i2r_rlc_vision_centerline_replay_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CGapFeatureDetector test target | `i2r_rlc_vision_gap_feature_detector_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CGapFeatureDetector replay test target | `i2r_rlc_vision_gap_feature_replay_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| ButtSeam feature test target | `i2r_rlc_vision_butt_seam_feature_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| Line geometry test target | `i2r_rlc_vision_line_geometry_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| TJoint feature test target | `i2r_rlc_vision_t_joint_feature_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| LapSeam feature test target | `i2r_rlc_vision_lap_seam_feature_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| WeldEndpoint test target | `i2r_rlc_vision_endpoint_detector_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| WeldSkeleton feature test target | `i2r_rlc_vision_skeleton_feature_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| CGrooveEdgeDetector test target | `i2r_rlc_vision_groove_edge_detector_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| WeldFeature fusion test target | `i2r_rlc_vision_feature_fusion_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| Image-to-space geometry test target | `i2r_rlc_vision_image_to_space_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| ImageProShare replay test target | `i2r_rlc_vision_image_pro_share_replay_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| ImageProFeng tools test target | `i2r_rlc_vision_image_pro_feng_tools_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| ImageProFeng penzui replay test target | `i2r_rlc_vision_image_pro_feng_replay_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| Shape matching experiment target | `i2r_rlc_vision_shape_matching_experiment_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 且 `I2R_RLC_ENABLE_SHAPE_MATCHING_EXPERIMENT=ON` 时生成 |
| Skeleton HALCON diff experiment target | `i2r_rlc_vision_skeleton_halcon_diff_experiment_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 且 `I2R_RLC_ENABLE_SHAPE_MATCHING_EXPERIMENT=ON` 时生成 |
| Active contour feature test target | `i2r_rlc_vision_active_contour_feature_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| Active contour penzui replay test target | `i2r_rlc_vision_active_contour_replay_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| Vision algorithm pipeline test target | `i2r_rlc_vision_algorithm_pipeline_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 SAM stripe test target | `i2r_rlc_vision_stripe_sam_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 ISFFA stripe test target | `i2r_rlc_vision_stripe_isffa_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 Steger stripe test target | `i2r_rlc_vision_stripe_steger_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 start point test target | `i2r_rlc_vision_start_point_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 seam type template test target | `i2r_rlc_vision_seam_type_template_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 image quality test target | `i2r_rlc_vision_image_quality_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成 |
| SWv1 ImagePro replay smoke target | `i2r_rlc_vision_sw_image_pro_replay_tests`，仅在 `I2R_RLC_ENABLE_OPENCV=ON` 时生成；输出到 CTest `Testing/Temporary/sw_image_pro_replay/` |
| Main runtime verification | PASS，用户确认主程序可启动，成功加载 `SENSORPARA.txt`，可显示离线图像，固定图像比例可接受。 |
| Codex shell result | 脚本能找到 Qt、VS2022、VS CMake、VS Ninja 和 `cl.exe`，但 CMake configure 可能在 compiler try-compile 阶段超时。 |
| OpenCV_DIR | 推荐 `E:\Libs\OPENCV3\opencv\build\x64\vc15\lib` |
| OpenCV bin | 推荐 `E:\Libs\OPENCV3\opencv\build\x64\vc15\bin` |
| Runtime PATH for qt-debug-opencv | `D:\QT\6.7.3\msvc2022_64\bin;E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;$penv{PATH}` |
| OpenCV smoke verification | PASS，当前本机 OpenCV preset 指向 OpenCV 3.4.5，仍保持 OpenCV 3.4.x 兼容边界。 |
| OpenCV preprocessing verification | V2-A1.1～V2-A1.5B 基础工具、中心线 detector 和 replay 已由用户在 VS2022 中确认通过：`i2r_rlc_vision_image_preprocessor_tests`、`i2r_rlc_vision_image_denoise_tests`、`i2r_rlc_vision_laser_stripe_extractor_tests`、`i2r_rlc_vision_subpixel_centerline_tests`、`i2r_rlc_vision_centerline_detector_tests`、`i2r_rlc_vision_centerline_replay_tests`。V2-A1.2R 已新增并重命名为 `i2r_rlc_vision_gabor_filter_tests`，待用户在 VS2022 中重新验证。V2-A2 最小版本和 V2-A2.1 gap/edge 语义增强已由用户在 VS2022 中确认通过：`i2r_rlc_vision_gap_feature_detector_tests`。V2-A2B 已新增 `i2r_rlc_vision_gap_feature_replay_tests`。V2-A2.2 ButtSeam 最小语义已由用户在 VS2022 中确认通过：`i2r_rlc_vision_butt_seam_feature_tests`。V2-A2.3 线段/RANSAC 几何工具已由用户在 VS2022 中确认通过：`i2r_rlc_vision_line_geometry_tests`。V2-A2.4 TJoint 最小语义已由用户在 VS2022 中确认通过：`i2r_rlc_vision_t_joint_feature_tests`。V2-A2.5 LapSeam 最小语义已由用户在 VS2022 中确认通过：`i2r_rlc_vision_lap_seam_feature_tests`。V2-A2.6 Endpoint 最小语义已由用户在 VS2022 中确认通过：`i2r_rlc_vision_endpoint_detector_tests`。V2-A2.7 Skeleton 最小语义已由用户在 VS2022 中确认通过：`i2r_rlc_vision_skeleton_feature_tests`。V2-A2.8 CGrooveEdgeDetector 最小语义已由用户在 VS2022 中确认通过：`i2r_rlc_vision_groove_edge_detector_tests`。V2-A2.9 feature fusion 已由用户在 VS2022 中确认通过：`i2r_rlc_vision_feature_fusion_tests`。V2-A2.10 ImageProShare 完整迁移已由用户在 VS2022 中确认通过：`i2r_rlc_vision_image_pro_share_tools_tests`。V2-A2.10R 已将该 target 改为 tinyspline NURBS/B-spline 曲线采样，并已由用户在 VS2022 中确认通过。V2-A2.10P 已新增 `i2r_rlc_vision_image_pro_share_replay_tests`，待用户在 VS2022 中验证。V2-A2.11 已新增 `i2r_rlc_vision_active_contour_feature_tests`，待用户在 VS2022 中验证。V2-A2.11P 已新增 `i2r_rlc_vision_active_contour_replay_tests`，待用户在 VS2022 中验证。V2-A2.11R 已由 Codex 构建并直接验证 `i2r_rlc_vision_active_contour_feature_tests`，CTest 已验证 `i2r_rlc_vision_image_pro_share_tools_tests` 和 `i2r_rlc_vision_active_contour_replay_tests`；现有 build 目录完整 CTest 需重新配置测试运行时 PATH 后再跑。V2-A3.0 image-to-space 已由用户在 VS2022 中确认通过：`i2r_rlc_vision_image_to_space_tests`。V2-A3.1 libicp wrapper 已由 Codex 构建并通过 `i2r_rlc_vision_libicp_tests`。 |

用户已确认主程序日志显示：

```text
已加载传感器标定文件：f=0.012139 m, cx=675.819942 pixel, cy=494.484186 pixel
```

构建结论：

- 用户本机 VS2022 构建结果优先于 Codex shell 结果。
- 普通 PowerShell 可优先使用 `scripts/build_vs2022_qt.ps1` 自动加载 VS2022 环境。
- 如果 PowerShell 执行策略阻止 `.ps1`，使用进程级 `-ExecutionPolicy Bypass`。

## 当前 UI 布局状态

- 当前 UI 使用固定尺寸 `MainWidget`，不允许用户自由缩放主窗口。
- 当前布局参考旧 MFC `IDD_ROBOTLEADING_DIALOG`：左侧大图像区、右侧状态监控区、底部按钮区和日志区。
- 旧 MFC `SWv1.rc` 中 `IDD_ROBOTLEADING_DIALOG` 为 `865 x 539` dialog unit。
- 旧 MFC 主图像控件为 `IDC_INPUT_HWINDOW`，位置和尺寸为 `7,6,646,482` dialog unit，是左侧主要显示区域。
- `IDC_INPUT_HWINDOW` 参考比例为 `646:482`，约 `1.340`。
- 当前离线相机图像为 `1292 x 964`，比例约 `1.340`，与旧 MFC 主图像控件比例基本一致。
- 当前 Qt 主窗口固定为 `1080 x 840`，图像显示画布固定为 `775 x 578`，约 `1.3408`，按旧 MFC 主图像控件比例等比放大。
- 右侧状态区高度与图像显示分组对齐，控制按钮区和日志区固定放在下方。
- 当前不实现 FitToWidget、FitWidth、OriginalSize、鼠标坐标映射、overlay 点线/ROI 或复杂窗口自适应。
- 当前图片只在固定深色区域内保持宽高比显示，不拉伸变形。

## 当前数据流

第五阶段离线图像显示数据流：

```text
OfflineImageCameraDevice -> CameraService -> UiController -> MainWidget
```

第六阶段早期视觉 no-op 数据流已废弃。历史链路曾为：

```text
OfflineImageCameraDevice
-> CameraService
-> UiController
-> VisionPipeline
-> WeldVisionPipelineResult
-> UiController
-> MainWidget 状态/日志显示
```

当前旧 `VisionPipeline` 已移除。`UiController` 负责 `CameraFrame/QImage` 到
`CV_8UC1 cv::Mat` 的适配，并直接调用 `CWeldVisionAlgorithmPipeline` 后填充
`WeldVisionPipelineResult`。

当前在线纠偏 UI 状态数据流：

```text
UiController
-> COnlineCorrectionController
-> OnlineCorrectionUpdate / OnlineCorrectionPhase
-> MainWidget 状态区 / 日志区
```

当前 `COnlineCorrectionController` 承接在线纠偏状态机。视觉帧处理结果不再推进 no-op workflow；后续在线纠偏闭环通过 `OnlineCorrectionDetectionEvent::samples` 显式输入检测点和可选 gap。

第七阶段前置 OpenCV smoke test 数据流：

```text
CMake OpenCV_DIR
-> find_package(OpenCV 3.4.3)
-> i2r_rlc_opencv_smoke_tests
-> cv::Mat / cv::cvtColor / cv::mean
```

第七阶段 OpenCV 基础预处理数据流是旧阶段过渡链路，相关 `OpenCvImageUtils`
适配工具和测试已移除。当前等价职责由 `UiController` 承担：

```text
CameraFrame(QImage)
-> UiController
-> cv::Mat
-> grayscale
-> mean/min/max
-> WeldVisionPipelineResult
```

当前 OpenCV 已进入 `vision` 模块的基础预处理链路和 SWv1 ImagePro 主路由编排；
旧 `VisionPipeline` 不再存在，真实算法组合由 `CWeldVisionAlgorithmPipeline`
继续向 `preprocess/stripe/feature/geometry` 子模块分发。

V2-A1 起，新的 `vision` 算法输入边界改为：

```text
ui / camera / app adapter
-> cv::Mat
-> vision common/preprocess/stripe/feature/geometry/correction/pipeline
```

`vision` 不负责 `QImage -> cv::Mat`、相机 SDK buffer -> `cv::Mat`、文件读取 -> `cv::Mat`，也不在算法接口中暴露 Qt 类型。

第八阶段前置焊缝检测接口数据流已被 V2-A2.13 的算法管线替代。当前 OpenCV 启用时的数据流为：

```text
UiController
-> CWeldVisionAlgorithmPipeline
-> ActiveContour / Endpoint / Unsupported legacy task
-> WeldVisionPipelineResult
-> WeldVisionPipelineResult
-> UiController
-> MainWidget 状态区 / 日志区
```

旧 `StubWeldDetector` 已移除；OpenCV 算法不可用时由调用层根据
`WeldVisionPipelineStatus` 处理 unsupported/invalid/error 状态。

## 当前关键模块

| 模块/文件 | 职责 |
| --- | --- |
| `Ui/MainWidget.*` | QWidget 顶层界面，只显示状态、图像和日志，并发出用户意图。当前采用固定尺寸布局。 |
| `Ui/UiController.*` | 应用协调层，连接 UI、配置仓库、offline camera、`CWeldVisionAlgorithmPipeline`、mock/dry-run 服务和日志输出；同时承担 `QImage -> cv::Mat` 的上层适配。 |
| `Ui/AppPaths.*` | 运行时路径集中管理，不依赖当前工作目录。 |
| `Ui/ConfigRepository.*` | 配置入口，优先查找新版 `SENSORPARA.txt`，再 fallback 到 legacy 兼容路径。 |
| `Ui/SensorCalibrationParser.*` | legacy `SENSORPARA.txt` 解析器，返回结构化结果，不弹窗。 |
| `Localization/include/CalibrationData.h` | `SensorCalibrationData` 数据模型，显式保留 m、deg、pixel 等单位。 |
| `Camera/include/CameraFrame.h` | 离线图像帧数据模型，当前以 `QImage` 承载图像，不依赖 Qt Widgets。 |
| `Camera/OfflineImageCameraDevice.*` | 离线图像目录读取后端，支持 png、jpg、jpeg、bmp，不接 Vimba/Basler。 |
| `Camera/CameraService.*` | 相机服务门面，封装 offline backend，并为后续 `CameraWorker` 留出边界。 |
| `RobotComm/*` | 默认 dry-run 机器人实现，不发送真实 DX200 命令。 |
| `Vision/pipeline/CWeldVisionAlgorithmPipeline.*` | SWv1 ImagePro 现代算法编排入口，承接旧 `GetDetImagePt(type)` 路由并调用各 vision 子模块；这是当前唯一 vision 算法管线。 |
| `tests/test_opencv_smoke.cpp` | OpenCV 3.4.3 CMake 接入冒烟测试，不读取真实图像，不实现算法。 |
| `Vision/include/stripe/LaserStripeExtractor.h`、`Vision/src/stripe/LaserStripeExtractor.cpp` | V2-A1.3 激光条纹基础提取工具，只接收 CV_8UC1 `cv::Mat`，输出 mask、列统计、候选 ROI、有效像素比例和整数像素局部峰候选。 |
| `tests/test_vision_laser_stripe_extractor.cpp` | CLaserStripeExtractor 合成图轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `Vision/include/stripe/SubpixelCenterlineTools.h`、`Vision/src/stripe/SubpixelCenterlineTools.cpp` | V2-A1.4 中心线基础数学工具，只接收 CV_8UC1 `cv::Mat` 或数值结构，输出抛物线亚像素、灰度重心、局部尺度、局部对比度和质量评价结果。 |
| `tests/test_vision_subpixel_centerline_tools.cpp` | CSubpixelCenterlineTools 合成图轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `Vision/include/stripe/DpStripeCenterlineDetector.h`、`Vision/src/stripe/DpStripeCenterlineDetector.cpp` | GapDetector 正式中心线检测器，承接旧 `extractLaserCenterline` 的列候选、亚像素、虚拟候选、二阶 DP、方向先验和断裂统计。 |
| `Vision/include/stripe/LaserCenterlineTypes.h` | 中心线公共结果类型，供 gap、endpoint、groove 和 replay 共享。 |
| `tests/test_vision_centerline_detector.cpp` | `CDpStripeCenterlineDetector::detectCenterline` 合成图轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_centerline_replay.cpp` | DP centerline replay 批量评估测试，扫描 `data/replay` 并输出 `output/reports/centerline_replay_report.csv`；不依赖 Qt、HALCON、PCL、真实相机或 DX200。 |
| `Vision/include/feature/WeldFeatureTypes.h` | V2-A2 焊接特征基础类型，定义 weld point、edge pair、gap result、weld feature result 和检测选项；只使用 pixel 和 0~1 confidence。 |
| `Vision/include/feature/GapFeatureDetector.h`、`Vision/src/feature/GapFeatureDetector.cpp` | V2-A2/A2.1 gap/左右边缘检测器，基于 CV_8UC1 图像和中心线先验，在局部多行窗口内收集左右边缘候选，并按对比度、宽度范围、中心线距离和行距选择 edge pair。 |
| `Vision/include/geometry/LineGeometryTools.h`、`Vision/src/geometry/LineGeometryTools.cpp` | V2-A2.3 线段几何工具，提供点列递归分段、最小二乘线拟合、确定性 RANSAC 线拟合和两线交点计算。 |
| `Vision/include/feature/WeldEndpointDetector.h`、`Vision/src/feature/WeldEndpointDetector.cpp` | V2-A2.6 焊缝起点/终点最小候选检测器，基于中心线有效点列和当前 feature 结果输出 2D start/end endpoint。 |
| `Vision/include/stripe/WeldSkeletonFeatureTools.h`、`Vision/src/stripe/WeldSkeletonFeatureTools.cpp` | V2-A2.7 焊接骨架基础工具，基于 CV_8UC1 mask/灰度图输出 skeleton mask、骨架点列、端点/分叉点和连通性统计。 |
| `Vision/include/feature/GrooveEdgeDetector.h`、`Vision/src/feature/GrooveEdgeDetector.cpp` | V2-A2.8 坡口/复杂间隙边缘最小候选检测器，基于中心线多点采样、左右边缘点列拟合、骨架质量提示和线段几何判断 V 型坡口或平行 gap 基础形态。 |
| `Vision/include/feature/WeldFeatureDetector.h`、`Vision/src/feature/WeldFeatureDetector.cpp` | V2-A2 焊接特征薄封装，当前组合 `CGapFeatureDetector`、`CWeldEndpointDetector` 和 `CGrooveEdgeDetector` 输出统一 `WeldFeatureResult`；T/Butt/Lap 语义由主动轮廓检测器承接。 |
| `Vision/include/stripe/ActiveContourStripeExtractor.h`、`Vision/src/stripe/ActiveContourStripeExtractor.cpp` | 主动轮廓条纹提取器，负责激光宽度估计、初始曲线、控制点修正和 BSnake 风格迭代，只输出条纹曲线和质量状态。 |
| `Vision/include/feature/ActiveContourWeldFeatureDetector.h`、`Vision/src/feature/ActiveContourWeldFeatureDetector.cpp` | 主动轮廓焊缝检测器，消费 stripe 层曲线输出，并负责 T/对接/搭接焊缝点、gap 点、拟合线和历史备用结果。 |
| `Vision/include/preprocess/GaborFilterTypes.h`、`Vision/include/preprocess/ImageDenoise.h`、`Vision/src/preprocess/ImageDenoise.cpp` | V2-A1.2R GaborFilter 兼容路径，按旧 `CImageDenoise::GaborFilter(MyLaserImagePoint, WeldPt)` 从仿真点和焊点估算滤波角度，并复刻旧 `mkKernel -> Process_P -> filter2D(CV_8U)`；不引入 `WeldType`、`SwMySensor.h`、Qt、HALCON 或 PCL。 |
| `Vision/include/geometry/WeldGeometryTypes.h`、`Vision/include/geometry/WeldImageToSpaceProjector.h`、`Vision/src/geometry/WeldImageToSpaceProjector.cpp` | V2-A3.0 几何基础类型和图像点到空间点重建工具，基于 Eigen 实现针孔相机射线与平面求交、feature 点投影和空间点刚体变换。 |
| `tests/test_vision_gap_feature_detector.cpp` | CGapFeatureDetector 合成图轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_gap_feature_replay.cpp` | CGapFeatureDetector replay 批量评估测试，扫描 `data/replay` 并输出 `output/reports/gap_feature_replay_report.csv`；逐张固定执行中心线检测和焊接特征检测，单张失败不中断整体 replay。 |
| `tests/test_vision_line_geometry.cpp` | CLineGeometryTools 合成点列轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_endpoint_detector.cpp` | CWeldEndpointDetector 合成中心线轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_skeleton_feature.cpp` | CWeldSkeletonFeatureTools 合成 mask 轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_groove_edge_detector.cpp` | CGrooveEdgeDetector 合成图轻量测试，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_feature_fusion.cpp` | CWeldFeatureDetector 聚合轻量测试，覆盖 gap、endpoint、groove 辅助结果和全 invalid 边界，不依赖真实图像、Qt、HALCON、PCL 或真实相机。 |
| `tests/test_vision_image_to_space.cpp` | 图像点到空间点重建轻量测试，覆盖 pinhole 投影、feature 点投影、非法内参/平面、平行射线、相机后方交点和刚体变换。 |
| `tests/test_vision_image_pro_share_replay.cpp` | ImageProShare 迁移工具 penzui 离线 replay 测试，抽样读取采集图片、仿真激光线和仿真焊缝点，生成 `output/reports/image_pro_share_replay_report.csv`。 |
| `tests/test_vision_active_contour_feature.cpp` | 主动轮廓焊缝检测合成图轻量测试，覆盖激光宽度、流程级初始曲线、分支/浮空段筛选、控制点修正/增长、轮廓迭代、T/对接/搭接分支和历史兜底。 |
| `tests/test_vision_active_contour_replay.cpp` | 主动轮廓 penzui 离线 replay 测试，读取真实采集图片和仿真焊缝点，生成 `output/reports/active_contour_replay_report.csv`。 |
| `Control/OnlineCorrectionController.*` | 在线纠偏主流程控制器，接收检测事件和机器人状态并输出纠偏阶段与命令 DTO。 |
| `DigitalTwin/NullDigitalTwinService.*` | 数字孪生 no-op 服务。 |
| `Ibvs/NullIbvsService.*` | IBVS no-op 服务。 |

## 运行时数据与 Replay 状态

- 默认标定文件路径：`config/calibration/SENSORPARA.txt`。
- legacy 兼容路径：`data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt`。
- 离线图像建议放置位置：`data/replay/` 或其子目录。
- 当前离线图像格式：`png`、`jpg`、`jpeg`、`bmp`。
- 当前旧阶段离线图像读取仍使用 Qt `QImage`；V2-A1 算法迁移要求 `ui` / `camera` / app adapter 层先转换为 `cv::Mat`，`vision` 算法不接收 Qt 类型。
- 新增 penzui 离线 replay fixture：`data/replay/penzui/imagecatch` 存放 `image_*.png` 采集图片；`robotPos.txt` 存放每帧机器人和变位机位姿；`SENSORPARA.txt` 存放标定数据；`simulatedLaser.txt` 存放每帧仿真激光线 2D 点对；`simulatedWeldpt.txt` 存放每帧仿真焊缝点 2D 点对。
- `i2r_rlc_vision_image_pro_share_replay_tests` 仅使用 penzui 图片、`simulatedLaser.txt` 和 `simulatedWeldpt.txt` 做 ImageProShare 迁移工具离线 2D replay，报告路径为 `output/reports/image_pro_share_replay_report.csv`；当前不使用 `robotPos.txt` 和 `SENSORPARA.txt` 做空间重建。
- `MainWidget` 可以打开 `QFileDialog` 选择目录，但不直接读取图片；图片读取由 `OfflineImageCameraDevice` 完成。
- `MainWidget` 不直接调用 vision 算法；视觉处理由 `UiController` 串联，并调用 `CWeldVisionAlgorithmPipeline`。

## 后续建议步骤

1. 在用户本机 VS2022 中选择 `qt-debug-opencv` preset，重新 Configure/Generate/Build。
2. 运行 `i2r_rlc.exe`，选择离线图像目录并点击“下一帧”，确认日志区输出算法管线状态。
4. 后续真实算法扩展仍应先保持在 `vision` 模块中，并继续使用 mock/dry-run 机器人。
5. 如果 VS2022/v143 使用 vc15 预编译库出现链接或运行问题，后续用 VS2022 重新编译 OpenCV 3.4.3，而不是直接升级 OpenCV 4.x。
6. 暂不接入真实 Vimba、Basler、DX200、HALCON。
7. V2-A1.5B 已由用户在 VS2022 中验证通过，V2-A1 基础工具与中心线链路已收口；当前仍禁止 Qt 类型进入 `vision`，禁止 HALCON、PCL、真实相机、DX200 和真实 TCP。
8. V2-A2.1 已由用户在 VS2022 中验证通过。
9. V2-A2.2 已由用户在 VS2022 中验证通过。
10. 当前 vision 算法迁移计划整体约完成 `45%~47%`，剩余约 `53%~55%`；若只看 OpenCV-only 2D 基础链路，约完成 `70%` 左右。详见 `docs/vision_algorithm_migration_progress_audit.md`。
11. V2-A2.3 已由用户在 VS2022 中验证通过。
12. V2-A2.4 已由用户在 VS2022 中验证通过。
13. V2-A2.6 已由用户在 VS2022 中验证通过。
14. V2-A2.7 已由用户在 VS2022 中验证通过。
15. V2-A2.8 已由用户在 VS2022 中验证通过。
16. V2-A2.9 已由用户在 VS2022 中验证通过。
17. V2-A3.0 已由用户在 VS2022 中验证通过；V2-A3.1 已完成旧 `libicp` 核心源码迁入和 `CIcpRegistrationMatcher` wrapper，Codex 已验证 `i2r_rlc_vision_libicp_tests`。下一步如需使用 ICP，应单独把 wrapper 接入 pipeline、GapDetector 或在线纠偏流程，仍禁止 HALCON。

## 未关闭问题

| 问题 | 状态 | 说明 |
| --- | --- | --- |
| DX200 协议字段语义 | Open | 真实机器人接入前阻塞；不阻塞 offline/mock 流程。 |
| DX200 dry-run 或仿真环境 | Open | 当前默认 `RobotComm dry-run`。 |
| `pose_CinT[6]` 精确语义 | Open | 暂作 legacy extra / timestamp-like 字段保存。 |
| HALCON 是否必须兼容 | Open | OpenCV 优先；HALCON 仅作为 optional legacy adapter。 |
| Codex shell CMake configure 超时 | Open | 用户 VS2022 构建结果优先；脚本和文档已记录绕行方法。 |
| OpenCV 3.4.3 vc15 与 VS2022/v143 兼容性 | Open | 先用 vc15 预编译库做 smoke test；如有链接或运行问题，建议用 VS2022 重新编译 OpenCV 3.4.3。 |
| `qt-debug-opencv` 运行时 DLL PATH | Fix applied / verify in VS2022 | 已在 preset environment 中加入 Qt bin 和 OpenCV bin，并在脚本运行前补 PATH；如仍缺 `Qt6Widgetsd.dll`，检查 VS2022 是否刷新了 preset。 |
| 焊缝检测真实算法 | In progress | 当前真实算法入口为 `CWeldVisionAlgorithmPipeline`；后续新增算法必须留在 `vision` 模块，不进入 UI。 |

## 后续 Codex 恢复检查清单

当上下文被压缩或新 Codex 接手时，先读取：

1. `AGENTS.md`
2. `i2r_rlc/AGENTS.md`
3. `i2r_rlc/docs/development_status.md`
4. 与当前任务相关的专项文档，例如 `migration_plan.md`、`new_architecture.md`、`threading_design.md`、`runtime_data_layout.md`、`calibration_file_format.md`

恢复后首先确认：

- 当前任务是否允许修改文件。
- 是否只允许修改 `i2r_rlc`。
- 是否禁止修改 `Sontech` / `SonTech`。
- 是否禁止真实设备、重依赖或业务功能扩展。
- 是否需要优先以用户 VS2022 构建结果为准，而不是依赖 Codex shell 构建结果。




