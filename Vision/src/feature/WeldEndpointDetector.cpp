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
 * @file        WeldEndpointDetector.cpp
 * @brief       焊缝端点检测实现
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

#include "feature/WeldEndpointDetector.h"

namespace i2r_rlc::vision {

namespace {

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
 * @brief 判断端点检测输入图像是否为有效灰度图。
 * @param gray8 输入图像。
 * @return 图像非空且类型为 CV_8UC1 时返回 true。
 */
bool isValidGrayImage(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 收集中心线中的有效点并复制为连续数组。
 * @param centerline 中心线结果，点坐标单位为 pixel。
 * @return 仅包含 valid 点的中心线点集合。
 */
std::vector<LaserCenterlinePoint> collectValidCenterlinePoints(
    const LaserCenterlineResult &centerline)
{
    std::vector<LaserCenterlinePoint> points;
    points.reserve(centerline.points.size());
    for (const LaserCenterlinePoint &point : centerline.points)
    {
        if (point.valid)
        {
            points.push_back(point);
        }
    }
    return points;
}

/**
 * @brief 计算两个焊缝特征点的欧氏距离。
 * @param lhs 第一个点，坐标单位为 pixel。
 * @param rhs 第二个点，坐标单位为 pixel。
 * @return 两点距离，单位为 pixel。
 */
double distancePixel(
    const WeldFeaturePoint &lhs,
    const WeldFeaturePoint &rhs)
{
    const double dx = lhs.xPixel - rhs.xPixel;
    const double dy = lhs.yPixel - rhs.yPixel;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 将中心线端点转换为焊缝特征点。
 * @param centerlinePoint 中心线端点，坐标单位为 pixel。
 * @param label 特征标签，指向静态字符串或调用方保证生命周期的字符串。
 * @return 标记为有效的端点特征。
 */
WeldFeaturePoint makeEndpointPoint(
    const LaserCenterlinePoint &centerlinePoint,
    const char *label)
{
    WeldFeaturePoint point;
    point.xPixel = centerlinePoint.xPixel;
    point.yPixel = centerlinePoint.yPixel;
    point.confidence = clampRatio(centerlinePoint.confidence);
    point.valid = true;
    point.label = label;
    return point;
}

/**
 * @brief 根据起止端点间距评估端点分离可信度。
 * @param startPoint 起点，坐标单位为 pixel。
 * @param endPoint 终点，坐标单位为 pixel。
 * @return [0,1] 置信度，距离过小时返回 0。
 */
double endpointSeparationConfidence(
    const WeldFeaturePoint &startPoint,
    const WeldFeaturePoint &endPoint)
{
    const double distance = distancePixel(startPoint, endPoint);
    if (distance <= 1.0)
    {
        return 0.0;
    }

    return clampRatio(distance / 20.0);
}

/**
 * @brief 检查端点与焊点候选的最小距离约束。
 * @param endpoint 当前端点检测结果。
 * @param featureResult 已聚合的焊缝特征结果。
 * @param options 端点与焊点距离约束配置。
 * @return [0,1] 一致性置信度。
 */
double weldPointConsistencyConfidence(
    const WeldEndpointResult &endpoint,
    const WeldFeatureResult &featureResult,
    const WeldFeatureDetectionOptions &options)
{
    const WeldFeaturePoint *weldPoint = nullptr;
    if (featureResult.gapFeature.valid
        && featureResult.gapFeature.weldPoint.valid)
    {
        weldPoint = &featureResult.gapFeature.weldPoint;
    }

    if (weldPoint == nullptr)
    {
        return 0.85;
    }

    const double minDistance = std::max(
        0.0,
        options.minEndpointDistanceFromWeldPointPixel);
    const double startDistance = distancePixel(endpoint.startPoint, *weldPoint);
    const double endDistance = distancePixel(endpoint.endPoint, *weldPoint);
    if (startDistance < minDistance || endDistance < minDistance)
    {
        return 0.0;
    }

    const double nearestDistance = std::min(startDistance, endDistance);
    return clampRatio(0.70 + 0.30 * nearestDistance / 20.0);
}

} // namespace

WeldEndpointResult CWeldEndpointDetector::detect(
    const cv::Mat &gray8,
    const LaserCenterlineResult &centerline,
    const WeldFeatureResult &featureResult,
    const WeldFeatureDetectionOptions &options)
{
    WeldEndpointResult result;
    if (!isValidGrayImage(gray8))
    {
        result.message = gray8.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }

    if (!centerline.valid)
    {
        result.message = "empty centerline";
        return result;
    }

    const std::vector<LaserCenterlinePoint> points =
        collectValidCenterlinePoints(centerline);
    const int minPointCount = std::max(2, options.minEndpointPointCount);
    if (static_cast<int>(points.size()) < minPointCount)
    {
        result.message = "not enough centerline points";
        return result;
    }

    result.startPoint = makeEndpointPoint(points.front(), "start_endpoint");
    result.endPoint = makeEndpointPoint(points.back(), "end_endpoint");

    const double separationScore = endpointSeparationConfidence(
        result.startPoint,
        result.endPoint);
    if (separationScore <= 0.0)
    {
        result.message = "endpoint separation is too small";
        return result;
    }

    const double weldPointScore = weldPointConsistencyConfidence(
        result,
        featureResult,
        options);
    if (weldPointScore <= 0.0)
    {
        result.message = "endpoint conflicts with weld point";
        return result;
    }

    const double pointScore = 0.5 * (
        result.startPoint.confidence + result.endPoint.confidence);
    const double centerlineScore = centerline.meanConfidence > 0.0
        ? centerline.meanConfidence
        : pointScore;
    const double featureScore = featureResult.valid
        ? std::max(0.25, featureResult.confidence)
        : 0.55;
    result.confidence = clampRatio(
        0.35 * pointScore
        + 0.25 * centerlineScore
        + 0.20 * separationScore
        + 0.10 * weldPointScore
        + 0.10 * featureScore);
    result.valid = result.confidence >= options.minConfidence;
    result.message = result.valid
        ? "endpoint candidates detected"
        : "endpoint confidence is low";
    return result;
}

} // namespace i2r_rlc::vision
