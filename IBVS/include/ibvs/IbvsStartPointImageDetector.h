/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsStartPointImageDetector.h
 * @brief     IBVS 起始点图像识别链路。
 * @details   本文件定义工件图像中的模板对齐、焊缝边缘提取和起始点计算接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_START_POINT_IMAGE_DETECTOR_H
#define I2R_RLC_IBVS_IBVS_START_POINT_IMAGE_DETECTOR_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ibvs/IbvsStartPointDetector.h"
#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 工件几何形状，用于选择起始点识别策略。
 */
enum class IbvsStartPointWorkpieceShape
{
    Straight,
    Curve
};

/**
 * @brief 起始点几何计算策略。
 */
enum class IbvsStartPointStrategy
{
    Endpoint,
    Intersection,
    CurveFixedArcFraction
};

/**
 * @brief 模板投影对齐的多层搜索参数。
 */
struct IbvsTemplateAlignmentParameters
{
    double level0DeltaXMinMm = -100.0;
    double level0DeltaXMaxMm = 100.0;
    double level0DeltaXStepMm = 20.0;
    double level0DeltaYMinMm = -100.0;
    double level0DeltaYMaxMm = 100.0;
    double level0DeltaYStepMm = 20.0;
    double level0DeltaPsiMinDeg = -90.0;
    double level0DeltaPsiMaxDeg = 90.0;
    double level0DeltaPsiStepDeg = 5.0;
    int level0TopN = 20;
    double level1DeltaXRangeMm = 20.0;
    double level1DeltaXStepMm = 5.0;
    double level1DeltaYRangeMm = 20.0;
    double level1DeltaYStepMm = 5.0;
    double level1DeltaPsiRangeDeg = 10.0;
    double level1DeltaPsiStepDeg = 1.0;
    int level1SeedCount = 5;
    int level1TopN = 5;
    double level2DeltaXRangeMm = 5.0;
    double level2DeltaXStepMm = 1.0;
    double level2DeltaYRangeMm = 5.0;
    double level2DeltaYStepMm = 1.0;
    double level2DeltaPsiRangeDeg = 3.0;
    double level2DeltaPsiStepDeg = 0.5;
    int level2SeedCount = 3;
    double chamferTauPx = 8.0;
    double inlierEpsilonPx = 3.0;
    double templateSampleIntervalPx = 5.0;
};

/**
 * @brief 起始点图像识别链路配置。
 */
struct IbvsStartPointImageDetectorConfig
{
    std::string workpieceId;
    IbvsStartPointWorkpieceShape shape = IbvsStartPointWorkpieceShape::Straight;
    IbvsStartPointStrategy strategy = IbvsStartPointStrategy::Endpoint;
    IbvsCameraIntrinsics intrinsics{ 2318.8, 2318.8, 720.0, 540.0 };
    cv::Vec3d rvec0 = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d tvec0 = cv::Vec3d(0.0, 0.0, 1.0);
    IbvsTemplateAlignmentParameters alignment;
    double straightEdgeSearchHalfWidthPx = 18.0;
    double curveEdgeSearchHalfWidthPx = 24.0;
    double maxEdgeDistancePx = 3.0;
    double curveStartArcFraction = 0.95;
    int minWeldPointCount = 8;
};

/**
 * @brief 起始点图像识别链路结果和诊断中间量。
 */
struct IbvsStartPointImageResult
{
    IbvsError error;
    bool success = false;
    bool edgeExtractionSucceeded = false;
    bool templateAlignmentSucceeded = false;
    bool weldExtractionSucceeded = false;
    bool startPointSucceeded = false;
    std::string failureReason;
    PixelPoint startPoint;
    PixelPoint endPoint;
    IbvsLineModel weldLine;
    IbvsLineModel secondEdgeLine;
    cv::Vec3d rvecOpt = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d tvecOpt = cv::Vec3d(0.0, 0.0, 0.0);
    double bestDeltaXMm = 0.0;
    double bestDeltaYMm = 0.0;
    double bestDeltaPsiDeg = 0.0;
    double templateCostPx = 0.0;
    double secondBestTemplateCostPx = 0.0;
    double templateCostMarginPx = 0.0;
    double templateInlierRatio = 0.0;
    double weldFitErrorPx = 0.0;
    std::vector<PixelPoint> curveInitialTemplatePoints;
    std::vector<PixelPoint> curveAlignedTemplatePoints;
    std::vector<PixelPoint> weldPriorPoints;
    std::vector<PixelPoint> secondEdgePriorPoints;
    std::vector<PixelPoint> weldPoints;
    std::vector<PixelPoint> secondEdgePoints;
    cv::Mat edges;
    cv::Mat contoursOverlay;
    cv::Mat curveInitialTemplateOverlay;
    cv::Mat curveAlignedTemplateOverlay;
    cv::Mat weldPriorOverlay;
    cv::Mat weldCandidatesOverlay;
    cv::Mat weldSubpixelOverlay;
    cv::Mat startPointOverlay;
};

/**
 * @brief 已裁切工件图上的模板匹配和起始点识别链路。
 */
class IbvsStartPointImageDetector
{
public:
    /**
     * @brief 在已裁切工件图上执行模板对齐、边缘提取和起始点计算。
     */
    IbvsStartPointImageResult detect(
        const cv::Mat& bgrImage,
        const std::vector<std::vector<cv::Point3f>>& curveTemplateEdges,
        const std::vector<std::vector<cv::Point3f>>& weldSampleEdges,
        const IbvsStartPointImageDetectorConfig& config) const;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_START_POINT_IMAGE_DETECTOR_H
