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
 * @file        LineGeometryTools.cpp
 * @brief       线段几何计算工具
 * @details     提供焊接视觉中的二维几何、投影和线段计算能力，输入输出统一使用像素或空间坐标结构。
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
#include <limits>

#include "geometry/LineGeometryTools.h"

namespace i2r_rlc::vision {

namespace {

constexpr double kMinLineLengthPixel = 1.0e-6;
constexpr double kParallelEpsilon = 1.0e-8;

/**
 * @brief 将置信度或比例值裁剪到 0~1。
 * @param value 输入值。
 * @return 裁剪后的值，范围为 0~1。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 收集有效焊缝特征点的 OpenCV 坐标。
 * @param points 输入特征点列，坐标单位 pixel。
 * @return 有效点对应的 OpenCV 点列，保持输入顺序。
 */
std::vector<cv::Point2d> collectValidPoints(
    const std::vector<WeldFeaturePoint> &points)
{
    std::vector<cv::Point2d> validPoints;
    validPoints.reserve(points.size());
    for (const WeldFeaturePoint &point : points)
    {
        if (!point.valid)
        {
            continue;
        }

        validPoints.push_back(cv::Point2d(point.xPixel, point.yPixel));
    }
    return validPoints;
}

/**
 * @brief 计算两个二维点的欧氏距离。
 * @param a 第一个点，单位 pixel。
 * @param b 第二个点，单位 pixel。
 * @return 两点距离，单位 pixel。
 */
double distanceBetweenPoints(const cv::Point2d &a, const cv::Point2d &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 计算点到直线的垂直距离。
 * @param point 待测点，单位 pixel。
 * @param lineStart 直线起点，单位 pixel。
 * @param lineEnd 直线终点，单位 pixel。
 * @return 垂直距离，单位 pixel；退化直线退回点到起点距离。
 */
double distancePointToLine(
    const cv::Point2d &point,
    const cv::Point2d &lineStart,
    const cv::Point2d &lineEnd)
{
    const double dx = lineEnd.x - lineStart.x;
    const double dy = lineEnd.y - lineStart.y;
    const double denominator = std::sqrt(dx * dx + dy * dy);
    if (denominator <= kMinLineLengthPixel)
    {
        return distanceBetweenPoints(point, lineStart);
    }

    const double numerator = std::fabs(
        dy * point.x - dx * point.y
        + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x);
    return numerator / denominator;
}

/**
 * @brief 递归执行旧 split_line 风格的最大偏离点切分。
 * @param points 输入折线点列，坐标单位 pixel。
 * @param beginIndex 当前区间起点索引。
 * @param endIndex 当前区间终点索引。
 * @param maxDistancePixel 触发切分的距离阈值，单位 pixel。
 * @param splitIndices 输出切分索引集合，必须非空。
 * @param observedMaxDistance 输出观察到的最大偏离距离，可为空。
 */
void splitPolylineRecursive(
    const std::vector<cv::Point2d> &points,
    int beginIndex,
    int endIndex,
    double maxDistancePixel,
    std::vector<int> *splitIndices,
    double *observedMaxDistance)
{
    if (endIndex - beginIndex < 2)
    {
        return;
    }

    // 旧 split_line 的核心是“用首尾点连成一条线，找到离这条线最远
    // 的中间点”。若最大距离超过阈值，就在该点递归切分折线。
    double maxDistance = 0.0;
    int maxDistanceIndex = -1;
    for (int index = beginIndex + 1; index < endIndex; ++index)
    {
        const double distance = distancePointToLine(
            points[index],
            points[beginIndex],
            points[endIndex]);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            maxDistanceIndex = index;
        }
    }

    if (observedMaxDistance != nullptr)
    {
        *observedMaxDistance = std::max(*observedMaxDistance, maxDistance);
    }

    if (maxDistanceIndex < 0 || maxDistance <= maxDistancePixel)
    {
        return;
    }

    splitIndices->push_back(maxDistanceIndex);
    splitPolylineRecursive(
        points,
        beginIndex,
        maxDistanceIndex,
        maxDistancePixel,
        splitIndices,
        observedMaxDistance);
    splitPolylineRecursive(
        points,
        maxDistanceIndex,
        endIndex,
        maxDistancePixel,
        splitIndices,
        observedMaxDistance);
}

/**
 * @brief 将 OpenCV 点包装为有效焊缝特征点。
 * @param point 输入点，单位 pixel。
 * @return 有效特征点，置信度为 1。
 */
WeldFeaturePoint makeFeaturePoint(const cv::Point2d &point)
{
    WeldFeaturePoint featurePoint;
    featurePoint.xPixel = point.x;
    featurePoint.yPixel = point.y;
    featurePoint.confidence = 1.0;
    featurePoint.valid = true;
    featurePoint.label = "line_point";
    return featurePoint;
}

/**
 * @brief 将 OpenCV 点列批量包装为焊缝特征点。
 * @param points 输入点列，坐标单位 pixel。
 * @return 有效特征点列，保持输入顺序。
 */
std::vector<WeldFeaturePoint> toFeaturePoints(
    const std::vector<cv::Point2d> &points)
{
    std::vector<WeldFeaturePoint> featurePoints;
    featurePoints.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        featurePoints.push_back(makeFeaturePoint(point));
    }
    return featurePoints;
}

/**
 * @brief 从点列中收集指定闭区间内的有效点。
 * @param points 输入特征点列，坐标单位 pixel。
 * @param beginIndex 起始索引，越界时裁剪到 0。
 * @param endIndex 结束索引，越界时裁剪到末尾。
 * @return 区间内有效特征点列。
 */
std::vector<WeldFeaturePoint> collectSegmentPoints(
    const std::vector<WeldFeaturePoint> &points,
    int beginIndex,
    int endIndex)
{
    std::vector<WeldFeaturePoint> segmentPoints;
    if (points.empty())
    {
        return segmentPoints;
    }

    const int begin = std::max(0, beginIndex);
    const int end = std::min(
        static_cast<int>(points.size()) - 1,
        endIndex);
    for (int index = begin; index <= end; ++index)
    {
        if (points[index].valid)
        {
            segmentPoints.push_back(points[index]);
        }
    }
    return segmentPoints;
}

/**
 * @brief 用两个端点构造线段模型。
 * @param start 起点，单位 pixel。
 * @param end 终点，单位 pixel。
 * @return 线段模型；长度过短时 valid=false。
 */
WeldLineSegment makeLineFromTwoPoints(
    const cv::Point2d &start,
    const cv::Point2d &end)
{
    WeldLineSegment line;
    line.start = start;
    line.end = end;
    line.lengthPixel = distanceBetweenPoints(start, end);
    if (line.lengthPixel <= kMinLineLengthPixel)
    {
        line.message = "line length is too short";
        return line;
    }

    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    line.vertical = std::fabs(dx) <= kParallelEpsilon;
    if (line.vertical)
    {
        line.slope = 0.0;
        line.intercept = start.x;
    }
    else
    {
        line.slope = dy / dx;
        line.intercept = start.y - line.slope * start.x;
    }
    line.valid = true;
    line.confidence = 1.0;
    line.message = "line fitted";
    return line;
}

/**
 * @brief 计算点到线段模型所在直线的距离。
 * @param point 待测点，单位 pixel。
 * @param line 线段模型。
 * @return 距离，单位 pixel；无效线段返回最大 double。
 */
double distancePointToSegmentLine(
    const cv::Point2d &point,
    const WeldLineSegment &line)
{
    if (!line.valid)
    {
        return std::numeric_limits<double>::infinity();
    }

    if (line.vertical)
    {
        return std::fabs(point.x - line.intercept);
    }

    return std::fabs(line.slope * point.x - point.y + line.intercept)
        / std::sqrt(line.slope * line.slope + 1.0);
}

double lineDirectionAngleRad(const WeldLineSegment &line)
{
    if (line.vertical)
    {
        return 0.5 * 3.14159265358979323846;
    }

    return std::atan(line.slope);
}

} // namespace

WeldLineSplitResult CLineGeometryTools::splitPolylineByDistance(
    const std::vector<WeldFeaturePoint> &points,
    double maxDistancePixel)
{
    WeldLineSplitResult result;
    if (maxDistancePixel < 0.0)
    {
        result.message = "invalid split distance";
        return result;
    }

    const std::vector<cv::Point2d> validPoints = collectValidPoints(points);
    if (validPoints.size() < 2)
    {
        result.message = "not enough valid points";
        return result;
    }

    result.splitIndices.push_back(0);
    splitPolylineRecursive(
        validPoints,
        0,
        static_cast<int>(validPoints.size()) - 1,
        maxDistancePixel,
        &result.splitIndices,
        &result.maxDistancePixel);
    result.splitIndices.push_back(static_cast<int>(validPoints.size()) - 1);
    std::sort(result.splitIndices.begin(), result.splitIndices.end());
    result.splitIndices.erase(
        std::unique(result.splitIndices.begin(), result.splitIndices.end()),
        result.splitIndices.end());
    result.valid = result.splitIndices.size() >= 2;
    result.message = result.valid ? "polyline split" : "split failed";
    return result;
}

WeldLineSplitResult CLineGeometryTools::splitPolylineByMaxDeviation(
    const std::vector<WeldFeaturePoint> &points,
    double maxDistancePixel)
{
    return splitPolylineByDistance(points, maxDistancePixel);
}

WeldLineSplitResult CLineGeometryTools::splitPolylineByLegacyRule(
    const std::vector<WeldFeaturePoint> &points,
    double maxDistancePixel)
{
    return splitPolylineByMaxDeviation(points, maxDistancePixel);
}

WeldLineSegment CLineGeometryTools::fitLineLeastSquares(
    const std::vector<WeldFeaturePoint> &points)
{
    const std::vector<cv::Point2d> validPoints = collectValidPoints(points);
    WeldLineSegment line;
    if (validPoints.size() < 2)
    {
        line.message = "not enough valid points";
        return line;
    }

    double meanX = 0.0;
    double meanY = 0.0;
    for (const cv::Point2d &point : validPoints)
    {
        meanX += point.x;
        meanY += point.y;
    }
    meanX /= static_cast<double>(validPoints.size());
    meanY /= static_cast<double>(validPoints.size());

    double sumXX = 0.0;
    double sumXY = 0.0;
    double minProjection = std::numeric_limits<double>::infinity();
    double maxProjection = -std::numeric_limits<double>::infinity();
    for (const cv::Point2d &point : validPoints)
    {
        const double dx = point.x - meanX;
        const double dy = point.y - meanY;
        sumXX += dx * dx;
        sumXY += dx * dy;
    }

    if (sumXX <= kParallelEpsilon)
    {
        // 点列 x 方差极小时按竖直线处理。旧 HALCON FitLineContourXld
        // 可以直接输出线参数，新结构用 vertical + intercept 表示 x 常量。
        line.vertical = true;
        line.slope = 0.0;
        line.intercept = meanX;
        for (const cv::Point2d &point : validPoints)
        {
            minProjection = std::min(minProjection, point.y);
            maxProjection = std::max(maxProjection, point.y);
        }
        line.start = cv::Point2d(meanX, minProjection);
        line.end = cv::Point2d(meanX, maxProjection);
    }
    else
    {
        // 非竖直线使用标准最小二乘拟合 y = kx + b，并用输入点的 x
        // 范围生成线段端点，保证下游 line_interfast 可按线段长度选择。
        line.vertical = false;
        line.slope = sumXY / sumXX;
        line.intercept = meanY - line.slope * meanX;
        for (const cv::Point2d &point : validPoints)
        {
            minProjection = std::min(minProjection, point.x);
            maxProjection = std::max(maxProjection, point.x);
        }
        line.start = cv::Point2d(
            minProjection,
            line.slope * minProjection + line.intercept);
        line.end = cv::Point2d(
            maxProjection,
            line.slope * maxProjection + line.intercept);
    }

    line.lengthPixel = distanceBetweenPoints(line.start, line.end);
    if (line.lengthPixel <= kMinLineLengthPixel)
    {
        line.message = "line length is too short";
        return line;
    }

    double errorSum = 0.0;
    for (const cv::Point2d &point : validPoints)
    {
        errorSum += distancePointToSegmentLine(point, line);
    }
    const double meanError = errorSum / static_cast<double>(validPoints.size());
    line.confidence = clampRatio(1.0 / (1.0 + meanError));
    line.valid = true;
    line.message = "line fitted";
    return line;
}

WeldLineSegment CLineGeometryTools::fitLineRansac(
    const std::vector<WeldFeaturePoint> &points,
    int iterations,
    double distanceThresholdPixel,
    double minInlierRatio)
{
    const std::vector<cv::Point2d> validPoints = collectValidPoints(points);
    WeldLineSegment bestLine;
    if (validPoints.size() < 2)
    {
        bestLine.message = "not enough valid points";
        return bestLine;
    }
    if (iterations <= 0 || distanceThresholdPixel < 0.0
        || minInlierRatio < 0.0 || minInlierRatio > 1.0)
    {
        bestLine.message = "invalid ransac options";
        return bestLine;
    }

    int bestInlierCount = 0;
    std::vector<cv::Point2d> bestInliers;
    // 旧 ransac_demo 使用随机采样。
    // 新实现采用确定性采样序列：每轮由 iter 推导两个不同点索引。
    for (int iter = 0; iter < iterations; ++iter)
    {
        const int firstIndex =
            iter % static_cast<int>(validPoints.size());
        const int step = 1 + (iter * 2) % (
            static_cast<int>(validPoints.size()) - 1);
        const int secondIndex =
            (firstIndex + step) % static_cast<int>(validPoints.size());
        if (firstIndex == secondIndex)
        {
            continue;
        }

        const WeldLineSegment candidate = makeLineFromTwoPoints(
            validPoints[firstIndex],
            validPoints[secondIndex]);
        if (!candidate.valid)
        {
            continue;
        }

        std::vector<cv::Point2d> inliers;
        inliers.reserve(validPoints.size());
        for (const cv::Point2d &point : validPoints)
        {
            if (distancePointToSegmentLine(point, candidate)
                <= distanceThresholdPixel)
            {
                inliers.push_back(point);
            }
        }

        if (static_cast<int>(inliers.size()) <= bestInlierCount)
        {
            continue;
        }

        bestInlierCount = static_cast<int>(inliers.size());
        bestInliers = inliers;
    }

    const double inlierRatio =
        static_cast<double>(bestInlierCount)
        / static_cast<double>(validPoints.size());
    if (bestInlierCount < 2 || inlierRatio < minInlierRatio)
    {
        bestLine.message = "not enough ransac inliers";
        return bestLine;
    }

    // RANSAC 只负责选内点；最终线段仍用内点集合做最小二乘重拟合，
    // 这对应旧工程“先鲁棒筛点，再 FitLineContourXld 拟合”的语义。
    bestLine = fitLineLeastSquares(toFeaturePoints(bestInliers));
    if (bestLine.valid)
    {
        bestLine.confidence = clampRatio(
            0.5 * bestLine.confidence + 0.5 * inlierRatio);
        bestLine.message = "ransac line fitted";
    }
    return bestLine;
}

WeldLineSegmentsResult CLineGeometryTools::fitSegmentsRansac(
    const std::vector<WeldFeaturePoint> &points,
    const std::vector<int> &splitIndices,
    int iterations,
    double distanceThresholdPixel,
    double minInlierRatio)
{
    WeldLineSegmentsResult result;
    if (points.size() < 2 || splitIndices.size() < 2)
    {
        result.message = "not enough points or split indices";
        return result;
    }

    std::vector<int> sortedIndices = splitIndices;
    std::sort(sortedIndices.begin(), sortedIndices.end());
    sortedIndices.erase(
        std::unique(sortedIndices.begin(), sortedIndices.end()),
        sortedIndices.end());

    // ransac_line 的输入是 split_line 输出的分段边界。每两个相邻边界
    // 形成一段点列，分别做 RANSAC 拟合，得到旧 returnLines 等价结果。
    for (int index = 1; index < static_cast<int>(sortedIndices.size());
         ++index)
    {
        const int beginIndex = sortedIndices[index - 1];
        const int endIndex = sortedIndices[index];
        if (endIndex - beginIndex < 2)
        {
            continue;
        }

        const std::vector<WeldFeaturePoint> segmentPoints =
            collectSegmentPoints(points, beginIndex, endIndex);
        const WeldLineSegment line = fitLineRansac(
            segmentPoints,
            iterations,
            distanceThresholdPixel,
            minInlierRatio);
        if (line.valid)
        {
            result.lines.push_back(line);
        }
    }

    result.valid = !result.lines.empty();
    result.message = result.valid
        ? "segments fitted"
        : "no valid segment line";
    return result;
}

WeldLineIntersectionResult CLineGeometryTools::intersectLines(
    const WeldLineSegment &lineA,
    const WeldLineSegment &lineB)
{
    WeldLineIntersectionResult result;
    if (!lineA.valid || !lineB.valid)
    {
        result.message = "invalid input line";
        return result;
    }

    if (lineA.vertical && lineB.vertical)
    {
        result.message = "parallel lines";
        return result;
    }

    if (lineA.vertical)
    {
        // 竖直线没有 y = kx + b 形式，使用 x 常量代入另一条线。
        result.point.x = lineA.intercept;
        result.point.y = lineB.slope * result.point.x + lineB.intercept;
    }
    else if (lineB.vertical)
    {
        result.point.x = lineB.intercept;
        result.point.y = lineA.slope * result.point.x + lineA.intercept;
    }
    else
    {
        const double denominator = lineA.slope - lineB.slope;
        if (std::fabs(denominator) <= kParallelEpsilon)
        {
            result.message = "parallel lines";
            return result;
        }
        result.point.x = (lineB.intercept - lineA.intercept) / denominator;
        result.point.y = lineA.slope * result.point.x + lineA.intercept;
    }

    double angle = std::fabs(
        lineDirectionAngleRad(lineA) - lineDirectionAngleRad(lineB));
    const double pi = 3.14159265358979323846;
    while (angle > pi)
    {
        angle -= pi;
    }
    if (angle > 0.5 * pi)
    {
        angle = pi - angle;
    }
    result.angleDegree = angle * 180.0 / pi;
    result.valid = true;
    result.message = "line intersection detected";
    return result;
}

WeldLineIntersectionResult CLineGeometryTools::intersectLinesNearReferenceX(
    const std::vector<WeldLineSegment> &lines,
    double referenceXPixel)
{
    WeldLineIntersectionResult result;
    if (lines.size() < 2)
    {
        result.message = "not enough input lines";
        return result;
    }

    if (lines.size() == 2)
    {
        return intersectLines(lines[0], lines[1]);
    }

    int splitLineIndex = -1;
    double minDistanceToReference = std::numeric_limits<double>::infinity();
    // 旧 line_interfast 会先在多段线中寻找 maxplace 附近的折点。
    // 这里用相邻线交点离 referenceXPixel 的距离来定位该折点。
    for (int index = 0; index < static_cast<int>(lines.size()) - 1; ++index)
    {
        const WeldLineIntersectionResult adjacentIntersection =
            intersectLines(lines[index], lines[index + 1]);
        if (!adjacentIntersection.valid)
        {
            continue;
        }

        const double distanceToReference = std::fabs(
            adjacentIntersection.point.x - referenceXPixel);
        if (distanceToReference < minDistanceToReference)
        {
            minDistanceToReference = distanceToReference;
            splitLineIndex = index;
        }
    }
    if (splitLineIndex < 0)
    {
        result.message = "no adjacent intersection";
        return result;
    }

    int leftLineIndex = 0;
    int rightLineIndex = static_cast<int>(lines.size()) - 1;
    double maxLeftLength = -1.0;
    double maxRightLength = -1.0;
    // 定位折点后，旧函数从左侧和右侧分别选最长线段求交点，
    // 目的是减少短噪声段对交点的影响。
    for (int index = 0; index <= splitLineIndex; ++index)
    {
        if (lines[index].valid && lines[index].lengthPixel > maxLeftLength)
        {
            maxLeftLength = lines[index].lengthPixel;
            leftLineIndex = index;
        }
    }
    for (int index = static_cast<int>(lines.size()) - 1;
         index > splitLineIndex;
         --index)
    {
        if (lines[index].valid && lines[index].lengthPixel > maxRightLength)
        {
            maxRightLength = lines[index].lengthPixel;
            rightLineIndex = index;
        }
    }

    const WeldLineIntersectionResult selectedIntersection =
        intersectLines(lines[leftLineIndex], lines[rightLineIndex]);
    if (selectedIntersection.valid)
    {
        return selectedIntersection;
    }

    // 旧 line_interfast 在多段线场景中按 maxplace 左右选择最长线段。
    // 迁移到通用点列后，合成或噪声数据可能让最长线段组合近似平行；
    // 此时保留 maxplace 语义，回退到参考列附近的有效交点。
    WeldLineIntersectionResult bestIntersection;
    double bestDistanceToReference = std::numeric_limits<double>::infinity();
    for (int leftIndex = 0; leftIndex <= splitLineIndex; ++leftIndex)
    {
        for (int rightIndex = splitLineIndex + 1;
             rightIndex < static_cast<int>(lines.size());
             ++rightIndex)
        {
            const WeldLineIntersectionResult candidateIntersection =
                intersectLines(lines[leftIndex], lines[rightIndex]);
            if (!candidateIntersection.valid)
            {
                continue;
            }

            const double distanceToReference = std::fabs(
                candidateIntersection.point.x - referenceXPixel);
            if (distanceToReference < bestDistanceToReference)
            {
                bestDistanceToReference = distanceToReference;
                bestIntersection = candidateIntersection;
            }
        }
    }

    if (bestIntersection.valid)
    {
        bestIntersection.message = "line intersection detected near reference";
        return bestIntersection;
    }

    return selectedIntersection;
}

} // namespace i2r_rlc::vision

