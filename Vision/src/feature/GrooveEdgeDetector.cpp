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
 * @file        GrooveEdgeDetector.cpp
 * @brief       坡口边缘特征检测实现
 * @details     提供 T 型、对接、搭接、gap、端点和仿真激光定位等焊缝语义检测能力。
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
#include <algorithm>
#include <cmath>
#include <vector>

#include "feature/GapFeatureDetector.h"
#include "feature/GrooveEdgeDetector.h"
#include "geometry/LineGeometryTools.h"
#include "stripe/WeldSkeletonFeatureTools.h"

namespace i2r_rlc::vision {

namespace {

/// @brief 坡口边缘候选最少配对数量。
constexpr int kMinEdgePairCount = 2;
/// @brief 识别 V 型坡口所需最小夹角，单位为 degree。
constexpr double kMinVGrooveAngleDegree = 12.0;
/// @brief 判定近似平行 gap 的最大夹角，单位为 degree。
constexpr double kParallelAngleDegree = 8.0;

/**
 * @brief 将比例或置信度夹紧到 [0,1]。
 * @param value 原始数值。
 * @return 归一化后的比例。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 判断坡口检测输入图像是否为有效灰度图。
 * @param gray8 输入图像。
 * @return 图像非空且类型为 CV_8UC1 时返回 true。
 */
bool isValidGrayImage(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 校验坡口检测参数是否处于有效范围。
 * @param options 焊缝特征检测参数。
 * @return 参数不会导致窗口越界或无效阈值时返回 true。
 */
bool isValidOptions(const WeldFeatureDetectionOptions &options)
{
    return options.searchHalfWindowY >= 0
        && options.minEdgeContrast >= 0
        && options.minEdgeContrast <= 255
        && options.minGapWidthPixel >= 0
        && options.maxGapWidthPixel >= options.minGapWidthPixel
        && options.minConfidence >= 0.0
        && options.minConfidence <= 1.0
        && options.edgeSearchHalfWidthPixel >= 0;
}

/**
 * @brief 根据 gap 宽度与配置范围计算宽度置信度。
 * @param widthPixel 候选坡口宽度，单位为 pixel。
 * @param options 宽度上下限配置。
 * @return [0,1] 宽度置信度。
 */
double widthConfidence(
    double widthPixel,
    const WeldFeatureDetectionOptions &options)
{
    if (widthPixel < static_cast<double>(options.minGapWidthPixel)
        || widthPixel > static_cast<double>(options.maxGapWidthPixel))
    {
        return 0.0;
    }

    const double allowedWidth =
        static_cast<double>(options.maxGapWidthPixel - options.minGapWidthPixel);
    if (allowedWidth <= 0.0)
    {
        return 1.0;
    }

    const double centerWidth =
        0.5 * static_cast<double>(
            options.minGapWidthPixel + options.maxGapWidthPixel);
    const double normalizedDistance =
        std::fabs(widthPixel - centerWidth) / (0.5 * allowedWidth);
    return clampRatio(1.0 - 0.35 * normalizedDistance);
}

/**
 * @brief 构造带标签的焊缝特征点。
 * @param xPixel 点 x 坐标，单位为 pixel。
 * @param yPixel 点 y 坐标，单位为 pixel。
 * @param confidence 原始置信度。
 * @param label 特征标签。
 * @return 按置信度标记有效性的特征点。
 */
WeldFeaturePoint makePoint(
    double xPixel,
    double yPixel,
    double confidence,
    const std::string &label)
{
    WeldFeaturePoint point;
    point.xPixel = xPixel;
    point.yPixel = yPixel;
    point.confidence = clampRatio(confidence);
    point.valid = point.confidence > 0.0;
    point.label = label;
    return point;
}

/**
 * @brief 求多个特征点的平均位置与平均置信度。
 * @param points 待平均点集合，坐标单位为 pixel。
 * @param label 输出点标签。
 * @return 平均后的特征点；空集合返回 invalid 点。
 */
WeldFeaturePoint averagePoint(
    const std::vector<WeldFeaturePoint> &points,
    const std::string &label)
{
    WeldFeaturePoint result;
    result.label = label;
    if (points.empty())
    {
        return result;
    }

    double xSum = 0.0;
    double ySum = 0.0;
    double confidenceSum = 0.0;
    for (const WeldFeaturePoint &point : points)
    {
        xSum += point.xPixel;
        ySum += point.yPixel;
        confidenceSum += point.confidence;
    }

    const double count = static_cast<double>(points.size());
    result.xPixel = xSum / count;
    result.yPixel = ySum / count;
    result.confidence = clampRatio(confidenceSum / count);
    result.valid = result.confidence > 0.0;
    return result;
}

/**
 * @brief 收集中心线中的有效点指针，避免复制大对象。
 * @param centerline 中心线结果，点坐标单位为 pixel。
 * @return 指向 centerline 内部 valid 点的指针集合；不得超过 centerline 生命周期使用。
 */
std::vector<const LaserCenterlinePoint *> collectValidCenterlinePoints(
    const LaserCenterlineResult &centerline)
{
    std::vector<const LaserCenterlinePoint *> points;
    points.reserve(centerline.points.size());
    for (const LaserCenterlinePoint &point : centerline.points)
    {
        if (point.valid)
        {
            points.push_back(&point);
        }
    }
    return points;
}

/**
 * @brief 计算左右边缘直线夹角，失败时返回 0。
 * @param leftLine 左侧边缘线。
 * @param rightLine 右侧边缘线。
 * @return 夹角，单位为 degree。
 */
double lineAngleOrZero(
    const WeldLineSegment &leftLine,
    const WeldLineSegment &rightLine)
{
    const WeldLineIntersectionResult intersection =
        CLineGeometryTools::intersectLines(leftLine, rightLine);
    return intersection.valid ? intersection.angleDegree : 0.0;
}

/**
 * @brief 根据左右线斜率和夹角分类坡口剖面。
 * @param leftLine 左侧边缘线。
 * @param rightLine 右侧边缘线。
 * @param angleDegree 左右线夹角，单位为 degree。
 * @return profile 标签，如 v_groove、parallel_gap 或 complex_gap。
 */
std::string classifyProfile(
    const WeldLineSegment &leftLine,
    const WeldLineSegment &rightLine,
    double angleDegree)
{
    if (!leftLine.valid || !rightLine.valid)
    {
        return "unknown";
    }

    if (leftLine.vertical && rightLine.vertical)
    {
        return "parallel_gap";
    }

    if (!leftLine.vertical
        && !rightLine.vertical
        && leftLine.slope * rightLine.slope < 0.0
        && angleDegree >= kMinVGrooveAngleDegree)
    {
        return "v_groove";
    }

    if (angleDegree <= kParallelAngleDegree)
    {
        return "parallel_gap";
    }

    return "complex_gap";
}

/**
 * @brief 将剖面类别和夹角转换为坡口分类置信度。
 * @param profileLabel 剖面分类标签。
 * @param angleDegree 左右线夹角，单位为 degree。
 * @return [0,1] 分类置信度。
 */
double profileConfidence(
    const std::string &profileLabel,
    double angleDegree)
{
    if (profileLabel == "v_groove")
    {
        return clampRatio(0.75 + 0.25 * std::min(angleDegree, 90.0) / 90.0);
    }

    if (profileLabel == "parallel_gap")
    {
        return 0.55;
    }

    if (profileLabel == "complex_gap")
    {
        return 0.70;
    }

    return 0.25;
}

/**
 * @brief 通过骨架提取结果为坡口检测提供弱置信度提示。
 * @param gray8 输入灰度图，要求 CV_8UC1。
 * @return [0,1] 骨架支持置信度。
 */
double skeletonConfidenceHint(const cv::Mat &gray8)
{
    WeldSkeletonExtractionOptions skeletonOptions;
    skeletonOptions.inputAlreadyBinary = false;
    skeletonOptions.thresholdValue = 128;
    skeletonOptions.minComponentArea = 3;
    skeletonOptions.maxIterations = 60;
    skeletonOptions.minConfidence = 0.0;

    const WeldSkeletonFeatureResult skeletonResult =
        CWeldSkeletonFeatureTools::extract(gray8, skeletonOptions);
    if (skeletonResult.skeletonPoints.empty())
    {
        return 0.65;
    }

    return skeletonResult.valid ? 1.0 : 0.80;
}

} // namespace

GrooveEdgeResult CGrooveEdgeDetector::detect(
    const cv::Mat &gray8,
    const LaserCenterlineResult &centerline,
    const WeldFeatureDetectionOptions &options)
{
    GrooveEdgeResult result;
    if (!isValidGrayImage(gray8))
    {
        result.message = gray8.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        result.profileLabel = "unknown";
        return result;
    }

    if (!centerline.valid || centerline.points.empty())
    {
        result.message = "empty centerline";
        result.profileLabel = "unknown";
        return result;
    }

    if (!isValidOptions(options))
    {
        result.message = "invalid options";
        result.profileLabel = "unknown";
        return result;
    }

    const std::vector<const LaserCenterlinePoint *> centerlinePoints =
        collectValidCenterlinePoints(centerline);
    if (centerlinePoints.empty())
    {
        result.message = "no valid centerline point";
        result.profileLabel = "unknown";
        return result;
    }

    std::vector<WeldFeaturePoint> leftPoints;
    std::vector<WeldFeaturePoint> rightPoints;
    std::vector<WeldFeaturePoint> centerPoints;
    double widthSum = 0.0;
    double contrastSum = 0.0;

    for (const LaserCenterlinePoint *centerlinePoint : centerlinePoints)
    {
        const WeldEdgePair edgePair = CGapFeatureDetector::detectEdgesOnRow(
            gray8,
            centerlinePoint->xPixel,
            centerlinePoint->yPixel,
            options);
        if (!edgePair.valid)
        {
            continue;
        }

        leftPoints.push_back(edgePair.left);
        rightPoints.push_back(edgePair.right);
        centerPoints.push_back(makePoint(
            0.5 * (edgePair.left.xPixel + edgePair.right.xPixel),
            0.5 * (edgePair.left.yPixel + edgePair.right.yPixel),
            edgePair.confidence,
            "groove_center"));
        widthSum += edgePair.widthPixel;

        const double leftContrast = CGapFeatureDetector::calculateEdgeContrast(
            gray8,
            static_cast<int>(std::floor(edgePair.left.xPixel + 0.5)),
            static_cast<int>(std::floor(edgePair.left.yPixel + 0.5)),
            options.searchHalfWindowY);
        const double rightContrast = CGapFeatureDetector::calculateEdgeContrast(
            gray8,
            static_cast<int>(std::floor(edgePair.right.xPixel + 0.5)),
            static_cast<int>(std::floor(edgePair.right.yPixel + 0.5)),
            options.searchHalfWindowY);
        contrastSum += 0.5 * (leftContrast + rightContrast);
    }

    result.edgePairCount = static_cast<int>(leftPoints.size());
    if (result.edgePairCount < kMinEdgePairCount)
    {
        result.message = "not enough groove edge pairs";
        result.profileLabel = "unknown";
        return result;
    }

    result.widthPixel = widthSum / static_cast<double>(result.edgePairCount);
    const double widthScore = widthConfidence(result.widthPixel, options);
    if (widthScore <= 0.0)
    {
        result.message = "groove width is out of range";
        result.profileLabel = "unknown";
        return result;
    }

    const WeldLineSegment leftLine =
        CLineGeometryTools::fitLineLeastSquares(leftPoints);
    const WeldLineSegment rightLine =
        CLineGeometryTools::fitLineLeastSquares(rightPoints);
    if (!leftLine.valid || !rightLine.valid)
    {
        result.message = "groove edge line fitting failed";
        result.profileLabel = "unknown";
        return result;
    }

    result.leftEdge = averagePoint(leftPoints, "groove_left_edge");
    result.rightEdge = averagePoint(rightPoints, "groove_right_edge");
    result.centerPoint = averagePoint(centerPoints, "groove_center");
    result.edgeContrastGrayValue =
        contrastSum / static_cast<double>(result.edgePairCount);
    result.lineAngleDegree = lineAngleOrZero(leftLine, rightLine);
    result.profileLabel = classifyProfile(
        leftLine,
        rightLine,
        result.lineAngleDegree);

    const double contrastScore = clampRatio(result.edgeContrastGrayValue / 255.0);
    const double pairCountScore = clampRatio(
        static_cast<double>(result.edgePairCount) / 5.0);
    const double shapeScore = profileConfidence(
        result.profileLabel,
        result.lineAngleDegree);
    const double skeletonScore = skeletonConfidenceHint(gray8);
    result.confidence = clampRatio(
        0.25 * widthScore
        + 0.25 * contrastScore
        + 0.20 * pairCountScore
        + 0.20 * shapeScore
        + 0.10 * skeletonScore);
    result.valid = result.leftEdge.valid
        && result.rightEdge.valid
        && result.centerPoint.valid
        && result.confidence >= options.minConfidence;
    if (!result.valid)
    {
        result.message = "groove edge confidence is low";
        return result;
    }

    result.message = result.profileLabel == "v_groove"
        ? "v groove edge detected"
        : (result.profileLabel == "parallel_gap"
               ? "parallel gap edge detected"
               : "complex groove edge detected");
    return result;
}

} // namespace i2r_rlc::vision
