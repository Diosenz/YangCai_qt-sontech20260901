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
 * @file        StripeLineRepairTools.cpp
 * @brief       条纹中心线修复工具
 * @details     提供激光条纹、骨架、中心线和主动轮廓曲线提取能力，不承担焊缝语义分类。
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

#include "geometry/LineGeometryTools.h"
#include "stripe/StripeLineRepairTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 判断输入是否为可扫描的 8 位单通道灰度图。
 * @param gray8 输入图像。
 * @return 图像非空且类型为 CV_8UC1 时返回 true。
 */
bool isSupportedGray(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 根据检测配置得到裁剪后的有效 ROI。
 * @param gray8 输入灰度图。
 * @param options 条纹检测参数，useRoi 为 false 时使用整图。
 * @return 位于图像内部的 ROI，单位为 pixel。
 */
cv::Rect normalizedRoi(const cv::Mat &gray8, const StripeDetectionOptions &options)
{
    const cv::Rect imageRect(0, 0, gray8.cols, gray8.rows);
    if (!options.useRoi)
    {
        return imageRect;
    }
    return options.roi & imageRect;
}

/**
 * @brief 将 OpenCV 点转换为通用焊缝特征点。
 * @param point 中心线点，x/y 单位为 pixel。
 * @return 标记为有效的 stripe_centerline 特征点。
 */
WeldFeaturePoint toFeaturePoint(const cv::Point2d &point)
{
    WeldFeaturePoint featurePoint;
    featurePoint.xPixel = point.x;
    featurePoint.yPixel = point.y;
    featurePoint.confidence = 1.0;
    featurePoint.valid = true;
    featurePoint.label = "stripe_centerline";
    return featurePoint;
}

/**
 * @brief 批量转换中心线点为通用焊缝特征点。
 * @param points 中心线点集合，x/y 单位为 pixel。
 * @return 可供几何拟合工具复用的特征点集合。
 */
std::vector<WeldFeaturePoint> toFeaturePoints(
    const std::vector<cv::Point2d> &points)
{
    std::vector<WeldFeaturePoint> featurePoints;
    featurePoints.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        featurePoints.push_back(toFeaturePoint(point));
    }
    return featurePoints;
}

/**
 * @brief 将比例或置信度夹紧到 [0,1]。
 * @param value 原始数值。
 * @return 归一化后的比例。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

} // namespace

std::vector<cv::Point2d> CStripeLineRepairTools::extractThresholdColumnCenters(
    const cv::Mat &gray8,
    const StripeDetectionOptions &options,
    std::vector<double> *responseByColumn)
{
    std::vector<cv::Point2d> points;
    if (!isSupportedGray(gray8))
    {
        return points;
    }

    if (responseByColumn != nullptr)
    {
        responseByColumn->assign(gray8.cols, 0.0);
    }

    const cv::Rect roi = normalizedRoi(gray8, options);
    for (int x = roi.x; x < roi.x + roi.width; ++x)
    {
        double weightedRowSum = 0.0;
        double graySum = 0.0;
        double maxGray = 0.0;
        for (int y = roi.y; y < roi.y + roi.height; ++y)
        {
            const double gray = static_cast<double>(
                gray8.at<unsigned char>(y, x));
            if (gray < options.thresholdGrayValue)
            {
                continue;
            }

            weightedRowSum += gray * static_cast<double>(y);
            graySum += gray;
            maxGray = std::max(maxGray, gray);
        }

        if (responseByColumn != nullptr)
        {
            (*responseByColumn)[x] = maxGray;
        }
        if (graySum <= 0.0)
        {
            continue;
        }

        points.emplace_back(
            static_cast<double>(x),
            weightedRowSum / graySum);
    }

    return points;
}

std::vector<cv::Point2d> CStripeLineRepairTools::smoothCenterline(
    const std::vector<cv::Point2d> &points,
    int windowRadius)
{
    if (windowRadius <= 0 || points.size() < 3U)
    {
        return points;
    }

    std::vector<cv::Point2d> smoothed = points;
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const int begin = std::max<int>(0, static_cast<int>(index) - windowRadius);
        const int end = std::min<int>(
            static_cast<int>(points.size()) - 1,
            static_cast<int>(index) + windowRadius);
        double ySum = 0.0;
        int count = 0;
        for (int local = begin; local <= end; ++local)
        {
            ySum += points[static_cast<std::size_t>(local)].y;
            ++count;
        }
        if (count > 0)
        {
            smoothed[index].y = ySum / static_cast<double>(count);
        }
    }

    return smoothed;
}

std::vector<StripeSegment> CStripeLineRepairTools::splitByContinuity(
    const std::vector<cv::Point2d> &points,
    const StripeDetectionOptions &options)
{
    std::vector<StripeSegment> segments;
    if (points.empty())
    {
        return segments;
    }

    int startIndex = 0;
    const auto pushSegment = [&](int begin, int end) {
        if (end < begin)
        {
            return;
        }
        StripeSegment segment;
        segment.startIndex = begin;
        segment.endIndex = end;
        segment.points.assign(
            points.begin() + begin,
            points.begin() + end + 1);
        segment.valid =
            static_cast<int>(segment.points.size()) >= options.minSegmentPointCount;
        segment.confidence = segment.valid ? 0.5 : 0.0;
        segments.push_back(segment);
    };

    for (std::size_t index = 1; index < points.size(); ++index)
    {
        const cv::Point2d &previous = points[index - 1U];
        const cv::Point2d &current = points[index];
        const bool breakByColumn =
            current.x - previous.x > options.maxColumnGapPixel;
        const bool breakByJump =
            std::abs(current.y - previous.y) > options.maxYJumpPixel;
        if (breakByColumn || breakByJump)
        {
            pushSegment(startIndex, static_cast<int>(index) - 1);
            startIndex = static_cast<int>(index);
        }
    }
    pushSegment(startIndex, static_cast<int>(points.size()) - 1);
    return segments;
}

std::vector<cv::Point2d> CStripeLineRepairTools::repairLinearGaps(
    const std::vector<cv::Point2d> &points,
    const StripeDetectionOptions &options,
    std::vector<cv::Point2d> *insertedPoints)
{
    if (insertedPoints != nullptr)
    {
        insertedPoints->clear();
    }
    if (points.size() < 2U)
    {
        return points;
    }

    std::vector<cv::Point2d> repaired;
    repaired.reserve(points.size());
    repaired.push_back(points.front());
    for (std::size_t index = 1; index < points.size(); ++index)
    {
        const cv::Point2d previous = points[index - 1U];
        const cv::Point2d current = points[index];
        const int dx = static_cast<int>(std::round(current.x - previous.x));
        if (dx > 1 && dx <= options.repairMaxGapPixel)
        {
            for (int step = 1; step < dx; ++step)
            {
                const double ratio = static_cast<double>(step)
                    / static_cast<double>(dx);
                const cv::Point2d inserted(
                    previous.x + static_cast<double>(step),
                    previous.y + ratio * (current.y - previous.y));
                repaired.push_back(inserted);
                if (insertedPoints != nullptr)
                {
                    insertedPoints->push_back(inserted);
                }
            }
        }
        repaired.push_back(current);
    }
    return repaired;
}

std::vector<StripeSegment> CStripeLineRepairTools::fitSegments(
    const std::vector<cv::Point2d> &points,
    const StripeDetectionOptions &options)
{
    std::vector<StripeSegment> segments =
        splitByContinuity(points, options);
    for (StripeSegment &segment : segments)
    {
        if (static_cast<int>(segment.points.size()) < options.minSegmentPointCount)
        {
            segment.valid = false;
            continue;
        }

        const std::vector<WeldFeaturePoint> featurePoints =
            toFeaturePoints(segment.points);
        const WeldLineSplitResult split =
            CLineGeometryTools::splitPolylineByLegacyRule(
                featurePoints,
                options.splitDistancePixel);
        const WeldLineSegmentsResult fitted =
            CLineGeometryTools::fitSegmentsRansac(
                featurePoints,
                split.valid ? split.splitIndices : std::vector<int>{0, static_cast<int>(featurePoints.size()) - 1},
                40,
                2.5,
                0.50);
        if (!fitted.lines.empty())
        {
            segment.fittedLine = fitted.lines.front();
            segment.confidence = clampRatio(segment.fittedLine.confidence);
            segment.valid = segment.fittedLine.valid;
        }
        else
        {
            segment.fittedLine = CLineGeometryTools::fitLineLeastSquares(featurePoints);
            segment.confidence = clampRatio(segment.fittedLine.confidence);
            segment.valid = segment.fittedLine.valid;
        }
    }
    return segments;
}

cv::Rect CStripeLineRepairTools::estimateCenterlineRoi(
    const std::vector<cv::Point2d> &points,
    const cv::Size &imageSize,
    int paddingPixel)
{
    if (points.empty() || imageSize.width <= 0 || imageSize.height <= 0)
    {
        return cv::Rect();
    }

    double minX = points.front().x;
    double maxX = points.front().x;
    double minY = points.front().y;
    double maxY = points.front().y;
    for (const cv::Point2d &point : points)
    {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }

    const int pad = std::max(0, paddingPixel);
    const int x = std::max(0, static_cast<int>(std::floor(minX)) - pad);
    const int y = std::max(0, static_cast<int>(std::floor(minY)) - pad);
    const int right = std::min(
        imageSize.width,
        static_cast<int>(std::ceil(maxX)) + pad + 1);
    const int bottom = std::min(
        imageSize.height,
        static_cast<int>(std::ceil(maxY)) + pad + 1);
    return cv::Rect(x, y, std::max(0, right - x), std::max(0, bottom - y));
}

} // namespace i2r_rlc::vision
