/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        SimulatedLaserWeldDetector.h
 * @brief       基于仿真激光线的焊缝定位接口
 * @details     提供形状模板匹配、ICP配准仿真激光线的焊缝检测能力。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifndef I2R_RLC_VISION_FEATURE_SIMULATED_LASER_WELD_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_SIMULATED_LASER_WELD_DETECTOR_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "DigitalTwinVisionTypes.h"
#include "feature/WeldFeatureTypes.h"
#include "geometry/LineGeometryTools.h"
#include "preprocess/DirectionalMorphologyFilter.h"

namespace i2r_rlc::vision {

/**
 * @brief 离线 replay 仿真输入解析结果。
 *
 * 该结构只用于测试或回放层读取 `data/replay/penzui` 文本，运行时算法入口
 * 直接接收 DigitalTwin 提供的轻量 Vision 观测。
 */
struct SimulatedLaserReplayParseResult {
    /// @brief true 表示至少解析出一帧有效激光线。
    bool valid = false;

    /// @brief 解析得到的单帧观测集合。
    std::vector<digital_twin::DigitalTwinVisionLaserObservation> observations;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 仿真线匹配参数。
 */
struct SimulatedLaserMatchingOptions {
    /// @brief 当前支持的焊缝类型。
    WeldSeamType seamType = WeldSeamType::TJoint;

    /// @brief 方向形态学增强参数。
    DirectionalMorphologyOptions morphologyOptions;

    /// @brief 骨架提取阈值。
    int skeletonThresholdValue = 100;

    /// @brief T 型 split_line 最大点线距离，单位 pixel。
    double splitDistancePixel = 4.0;

    /// @brief RANSAC 迭代次数。
    int ransacIterations = 60;

    /// @brief RANSAC 点线距离阈值，单位 pixel。
    double ransacDistanceThresholdPixel = 2.0;

    /// @brief RANSAC 最小内点比例，范围 0~1。
    double minInlierRatio = 0.50;

    /// @brief 沿仿真线执行 LoGSke 风格搜索的采样步长，单位 point。
    int logSearchPointStep = 10;

    /// @brief 沿仿真点同列搜索的半高，单位 pixel。
    int logSearchHalfHeightPixel = 20;

    /// @brief LoGSke 候选点最低灰度。
    int logSearchMinGrayValue = 80;

    /// @brief 低于该置信度时输出 invalid，范围 0~1。
    double minConfidence = 0.10;

    /// @brief true 表示骨架失败时允许 HoughLinesP 备用路径。
    bool enableHoughSkeletonFallback = true;

    /// @brief 模板搜索平移半径，单位 pixel。
    int templateSearchRadiusPixel = 12;

    /// @brief 模板搜索平移步长，单位 pixel。
    int templateSearchStepPixel = 2;

    /// @brief 模板旋转搜索半角，单位 degree。
    double templateAngleRangeDegree = 4.0;

    /// @brief 模板旋转搜索步长，单位 degree。
    double templateAngleStepDegree = 2.0;

    /// @brief 模板 Chamfer 得分低于该值时不更新状态，范围 0~1。
    double templateMinScore = 0.35;

    /// @brief Chamfer 得分归一化距离上限，单位 pixel。
    double templateMaxDistancePixel = 12.0;

    /// @brief true 表示模板匹配优先使用 shape_based_matching 后端。
    bool enableShapeBasedTemplateMatching = true;

    /// @brief true 表示 shape_based_matching 失败后允许退回 Chamfer 后端。
    bool enableChamferTemplateFallback = true;

    /// @brief shape_based_matching 最低相似度，单位为 0~100 的库内分数。
    double shapeBasedMinSimilarity = 50.0;

    /// @brief shape_based_matching 模板图尺寸，单位 pixel，建议使用奇数。
    int shapeBasedTemplateSizePixel = 101;

    /// @brief shape_based_matching 搜索 ROI 半径，单位 pixel。
    int shapeBasedRoiHalfSizePixel = 90;

    /// @brief shape_based_matching 梯度弱阈值，对应 LINE2D 特征提取。
    double shapeBasedWeakThreshold = 30.0;

    /// @brief shape_based_matching 梯度强阈值，对应 LINE2D 特征提取。
    double shapeBasedStrongThreshold = 60.0;

    /// @brief shape_based_matching 金字塔步长层级，单位 pixel。
    std::vector<int> shapeBasedPyramidLevels = {4, 8};
};

/**
 * @brief 仿真线匹配结果。
 */
struct SimulatedLaserMatchingResult {
    /// @brief true 表示得到有效匹配。
    bool valid = false;

    /// @brief 匹配后的焊缝点，单位 pixel。
    cv::Point2d weldPoint;

    /// @brief 原始仿真线仿射后的点列，单位 pixel。
    std::vector<cv::Point2d> matchedLaserPoints;

    /// @brief coarse 阶段得到的仿真线点列，单位 pixel。
    std::vector<cv::Point2d> coarseMatchedLaserPoints;

    /// @brief 从真实图像提取的骨架中心线，单位 pixel。
    std::vector<cv::Point2d> skeletonPoints;

    /// @brief T 型分段 RANSAC 拟合线。
    std::vector<WeldLineSegment> fittedLines;

    /// @brief 两段真实线交点，单位 pixel。
    cv::Point2d intersectionPoint;

    /// @brief 最大残差，单位 pixel。
    double maxResidualPixel = 0.0;

    /// @brief 最小残差，单位 pixel。
    double minResidualPixel = 0.0;

    /// @brief 平均残差，单位 pixel。
    double meanResidualPixel = 0.0;

    /// @brief 均方根残差，单位 pixel。
    double rmsResidualPixel = 0.0;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示 T 型 coarse matching 成功。
    bool coarseValid = false;

    /// @brief true 表示使用 HoughLinesP 骨架备用路径。
    bool usedHoughFallback = false;

    /// @brief true 表示使用上一帧模板 Chamfer 匹配备用。
    bool usedTemplateFallback = false;

    /// @brief 模板 Chamfer 得分，范围 0~1。
    double templateScore = 0.0;

    /// @brief 模板匹配角度，单位 degree。
    double templateAngleDegree = 0.0;

    /// @brief 实际使用的模板匹配方法，例如 shape_based_matching 或 chamfer_fallback。
    std::string templateMethod;

    /// @brief true 表示 shape_based_matching 失败后使用 Chamfer 备用。
    bool usedChamferFallback = false;

    /// @brief 模板模型生成耗时，单位 ms；不适用时为 0。
    double templateModelBuildMs = 0.0;

    /// @brief 模板注册耗时，单位 ms；不适用时为 0。
    double templateAddMs = 0.0;

    /// @brief 模板搜索耗时，单位 ms。
    double templateMatchMs = 0.0;

    /// @brief 模板匹配总耗时，单位 ms。
    double templateTotalMs = 0.0;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 离线 replay 仿真数据解析器。
 *
 * 解析器只处理调用方传入的文本内容，不直接读文件。运行时不应通过本类
 * 提供仿真点，而应由 DigitalTwin 或外部兼容层构造 Vision 观测 DTO。
 */
class CSimulatedLaserReplayParser final {
public:
    /**
     * @brief 从激光线文本和可选焊缝点文本解析 replay 观测集合。
     */
    static SimulatedLaserReplayParseResult parse(
        const std::string &laserText,
        const std::string &weldPointText);
};

/**
 * @brief 基于 OpenCV 距离变换的仿真激光线模板匹配。
 *
 * 该类对应旧 `TemplateMatching_change` 的检测入口。输入为当前灰度图、
 * 一帧仿真线和仿真焊缝点，输出模板平移/旋转后的焊缝点、得分和角度。
 * 默认优先使用 shape_based_matching 的 OpenCV LINE2D 风格后端；当该后端
 * 无法建模、无匹配或分数不足时，可自动退回 Chamfer 工程替代路径。
 * 本类不生成或保存 HALCON 模型句柄。
 */
class CSimulatedLaserTemplateMatcher final {
public:
    /**
     * @brief 在灰度图中匹配一帧仿真线模板。
     */
    static SimulatedLaserMatchingResult match(
        const cv::Mat &grayImage,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const SimulatedLaserMatchingOptions &options);
};

/**
 * @brief 仿真激光线与真实骨架配准工具。
 *
 * 该工具对应旧 `EmulateImagePt` 中基于仿真激光线的检测入口，迁移
 * `T_SeamCoarseMatching`、`LoGSke`、`MatchSimLaser` 和 `AffineMatchLaser`
 * 的主语义：先按 T 型几何粗配准，再用同列 LoG 搜索和 affine 最小二乘
 * 精配准。调用方显式选择是否再执行模板匹配，不在本类内部隐藏备用策略。
 */
class CSimulatedLaserRegistrationMatcher final {
public:
    /**
     * @brief 执行仿真线到真实激光线的配准。
     */
    static SimulatedLaserMatchingResult match(
        const cv::Mat &grayImage,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const SimulatedLaserMatchingOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_SIMULATED_LASER_WELD_DETECTOR_H

