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
 * @file        SimulatedLaserWeldDetector.cpp
 * @brief       仿真激光线焊缝定位实现
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
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>

#include <cctype>
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>

#include "feature/SimulatedLaserWeldDetector.h"
#include "line2Dup.h"
#include "stripe/LaserLineResponseTools.h"
#include "stripe/WeldSkeletonFeatureTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 判断仿真匹配链路是否可处理输入图像。
 * @param image 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 */
bool isValidGrayImage(const cv::Mat &image)
{
    return !image.empty() && image.type() == CV_8UC1;
}

/**
 * @brief 计算两个图像点之间的欧氏距离。
 * @param left 第一个点，单位 pixel。
 * @param right 第二个点，单位 pixel。
 * @return 两点距离，单位 pixel。
 */
double distanceBetween(const cv::Point2d &left, const cv::Point2d &right)
{
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 将 DigitalTwin 图像点转换为 OpenCV 点。
 * @param point DigitalTwin 图像点，坐标单位 pixel。
 * @return OpenCV 二维点，坐标单位 pixel。
 */
cv::Point2d toCvPoint(const digital_twin::DigitalTwinImagePoint2D_px &point)
{
    return cv::Point2d(point.x_px, point.y_px);
}

/**
 * @brief 将 OpenCV 点转换为 DigitalTwin 图像点。
 * @param point OpenCV 二维点，坐标单位 pixel。
 * @return DigitalTwin 图像点，坐标单位 pixel。
 */
digital_twin::DigitalTwinImagePoint2D_px toDigitalTwinImagePoint(
    const cv::Point2d &point)
{
    digital_twin::DigitalTwinImagePoint2D_px imagePoint;
    imagePoint.x_px = point.x;
    imagePoint.y_px = point.y;
    return imagePoint;
}

/**
 * @brief 批量转换 DigitalTwin 图像点列为 OpenCV 点列。
 * @param points 输入点列，坐标单位 pixel。
 * @return OpenCV 点列，保持原始顺序。
 */
std::vector<cv::Point2d> toCvPoints(
    const std::vector<digital_twin::DigitalTwinImagePoint2D_px> &points)
{
    std::vector<cv::Point2d> output;
    output.reserve(points.size());
    for (const auto &point : points)
    {
        output.push_back(toCvPoint(point));
    }
    return output;
}

/**
 * @brief 批量转换 OpenCV 点列为 DigitalTwin 图像点列。
 * @param points 输入点列，坐标单位 pixel。
 * @return DigitalTwin 图像点列，保持原始顺序。
 */
std::vector<digital_twin::DigitalTwinImagePoint2D_px> toDigitalTwinImagePoints(
    const std::vector<cv::Point2d> &points)
{
    std::vector<digital_twin::DigitalTwinImagePoint2D_px> output;
    output.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        output.push_back(toDigitalTwinImagePoint(point));
    }
    return output;
}

/**
 * @brief 计算点到线段所在直线的垂距。
 * @param point 待测点，单位 pixel。
 * @param start 直线参考起点，单位 pixel。
 * @param end 直线参考终点，单位 pixel。
 * @return 垂直距离，单位 pixel；退化线段返回 0。
 */
double distancePointToLine(
    const cv::Point2d &point,
    const cv::Point2d &start,
    const cv::Point2d &end)
{
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double denominator = std::sqrt(dx * dx + dy * dy);
    if (denominator <= std::numeric_limits<double>::epsilon())
    {
        return 0.0;
    }

    return std::abs(dy * point.x - dx * point.y + end.x * start.y
        - end.y * start.x) / denominator;
}

/**
 * @brief 查找点列相对首尾直线的最大偏离点索引。
 * @param points 输入点列，坐标单位 pixel。
 * @return 最大偏离点索引；点数不足时返回 -1。
 * @note 用于 T 型仿真线粗切分，语义对应旧 split_line 的初始分裂点。
 */
int findSplitPointIndex(const std::vector<cv::Point2d> &points)
{
    if (points.size() < 3U)
    {
        return -1;
    }

    int splitIndex = 0;
    double maxDistance = -1.0;
    for (int index = 0; index < static_cast<int>(points.size()); ++index)
    {
        const double distance =
            distancePointToLine(points[index], points.front(), points.back());
        if (distance > maxDistance)
        {
            maxDistance = distance;
            splitIndex = index;
        }
    }
    return splitIndex;
}

/**
 * @brief 将 OpenCV 点列包装为线几何工具使用的焊缝特征点。
 * @param points 输入点列，坐标单位 pixel。
 * @return 按 x 坐标升序排列的有效特征点列。
 */
std::vector<WeldFeaturePoint> toFeaturePoints(
    const std::vector<cv::Point2d> &points)
{
    std::vector<WeldFeaturePoint> featurePoints;
    featurePoints.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        WeldFeaturePoint featurePoint;
        featurePoint.xPixel = point.x;
        featurePoint.yPixel = point.y;
        featurePoint.confidence = 1.0;
        featurePoint.valid = true;
        featurePoint.label = "skeleton";
        featurePoints.push_back(featurePoint);
    }

    std::sort(
        featurePoints.begin(),
        featurePoints.end(),
        [](const WeldFeaturePoint &left, const WeldFeaturePoint &right) {
            return left.xPixel < right.xPixel;
        });
    return featurePoints;
}

/**
 * @brief 从焊缝特征点中提取有效 OpenCV 点。
 * @param points 输入特征点列，坐标单位 pixel。
 * @return 有效点对应的 OpenCV 点列，保持输入顺序。
 */
std::vector<cv::Point2d> toPoints(
    const std::vector<WeldFeaturePoint> &points)
{
    std::vector<cv::Point2d> output;
    output.reserve(points.size());
    for (const WeldFeaturePoint &point : points)
    {
        if (point.valid)
        {
            output.push_back(cv::Point2d(point.xPixel, point.yPixel));
        }
    }
    return output;
}

/**
 * @brief 在 RANSAC 不足时用局部点段拟合 fallback 线段。
 * @param points 已排序特征点列，坐标单位 pixel。
 * @param beginIndex 起始索引，越界时会裁剪到有效范围。
 * @param endIndex 结束索引，越界时会裁剪到有效范围。
 * @param options 仿真线匹配参数，提供 RANSAC 阈值和置信度约束。
 * @return fallback 拟合线段；样本不足时返回 invalid。
 */
WeldLineSegment makeFallbackLine(
    const std::vector<WeldFeaturePoint> &points,
    int beginIndex,
    int endIndex,
    const SimulatedLaserMatchingOptions &options)
{
    std::vector<WeldFeaturePoint> subset;
    beginIndex = std::max(0, beginIndex);
    endIndex = std::min(endIndex, static_cast<int>(points.size()) - 1);
    for (int index = beginIndex; index <= endIndex; ++index)
    {
        subset.push_back(points[static_cast<std::size_t>(index)]);
    }

    WeldLineSegment line = CLineGeometryTools::fitLineRansac(
        subset,
        options.ransacIterations,
        options.ransacDistanceThresholdPixel,
        options.minInlierRatio);
    if (!line.valid)
    {
        line = CLineGeometryTools::fitLineLeastSquares(subset);
    }
    return line;
}

/**
 * @brief 基于骨架点和参考焊缝点拟合 T 型两侧真实线段。
 * @param skeletonPoints 真实图像骨架点，坐标单位 pixel，要求按 x 排序。
 * @param referencePoint 仿真焊缝参考点，单位 pixel。
 * @param options 仿真线匹配参数，提供 split/RANSAC 阈值。
 * @return 拟合线段集合；常规分段失败时返回左右 fallback 线段。
 */
std::vector<WeldLineSegment> fitTJointLines(
    const std::vector<WeldFeaturePoint> &skeletonPoints,
    const cv::Point2d &referencePoint,
    const SimulatedLaserMatchingOptions &options)
{
    const WeldLineSplitResult splitResult =
        CLineGeometryTools::splitPolylineByMaxDeviation(
            skeletonPoints,
            options.splitDistancePixel);
    WeldLineSegmentsResult segments;
    if (splitResult.valid)
    {
        segments = CLineGeometryTools::fitSegmentsRansac(
            skeletonPoints,
            splitResult.splitIndices,
            options.ransacIterations,
            options.ransacDistanceThresholdPixel,
            options.minInlierRatio);
    }

    if (segments.valid && segments.lines.size() >= 2U)
    {
        return segments.lines;
    }

    int nearestIndex = 0;
    double minDistance = std::numeric_limits<double>::max();
    for (int index = 0; index < static_cast<int>(skeletonPoints.size()); ++index)
    {
        const double distance =
            std::abs(skeletonPoints[static_cast<std::size_t>(index)].xPixel
                - referencePoint.x);
        if (distance < minDistance)
        {
            minDistance = distance;
            nearestIndex = index;
        }
    }

    const int margin = std::max(5, static_cast<int>(skeletonPoints.size()) / 8);
    std::vector<WeldLineSegment> lines;
    lines.push_back(makeFallbackLine(
        skeletonPoints,
        0,
        std::max(0, nearestIndex - margin),
        options));
    lines.push_back(makeFallbackLine(
        skeletonPoints,
        std::min(static_cast<int>(skeletonPoints.size()) - 1,
            nearestIndex + margin),
        static_cast<int>(skeletonPoints.size()) - 1,
        options));
    return lines;
}

/**
 * @brief 根据线模型计算指定列处的图像点。
 * @param line 输入线段模型，坐标单位 pixel。
 * @param xPixel 查询列坐标，单位 pixel。
 * @return 线段模型上的点；竖直线返回其常量 x 和起点 y。
 */
cv::Point2d evaluateLineAtX(const WeldLineSegment &line, double xPixel)
{
    if (line.vertical)
    {
        return cv::Point2d(line.intercept, line.start.y);
    }
    return cv::Point2d(xPixel, line.slope * xPixel + line.intercept);
}

/**
 * @brief 根据仿真点位于 split 点左右选择真实拟合线目标点。
 * @param modelPoint 仿真线点，单位 pixel。
 * @param modelSplitPoint 仿真焊缝 split 点，单位 pixel。
 * @param lines 真实图像拟合线段集合。
 * @return 与仿真点同列的真实目标点；线段不足时返回原仿真点。
 */
cv::Point2d targetPointForModel(
    const cv::Point2d &modelPoint,
    const cv::Point2d &modelSplitPoint,
    const std::vector<WeldLineSegment> &lines)
{
    if (lines.size() < 2U)
    {
        return modelPoint;
    }

    const WeldLineSegment &line =
        modelPoint.x <= modelSplitPoint.x ? lines.front() : lines.back();
    return evaluateLineAtX(line, modelPoint.x);
}

/**
 * @brief 构造 LoGSke 拖拽接口需要的逐行激光宽度数组。
 * @param rowCount 图像行数，单位 pixel。
 * @return 固定宽度数组；当前仿真线匹配使用保守宽度 8 pixel。
 */
std::vector<int> makeLaserWidthByRow(int rowCount)
{
    return std::vector<int>(static_cast<std::size_t>(rowCount), 8);
}

/**
 * @brief 沿真实激光响应拖拽一个仿真点并追加配准点对。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param modelPoint 仿真点，单位 pixel。
 * @param options LoGSke 搜索半高和灰度阈值参数。
 * @param sourcePoints 输出源点集合，必须非空。
 * @param targetPoints 输出目标点集合，必须非空。
 */
void appendLogSearchPair(
    const cv::Mat &grayImage,
    const cv::Point2d &modelPoint,
    const SimulatedLaserMatchingOptions &options,
    std::vector<cv::Point2d> *sourcePoints,
    std::vector<cv::Point2d> *targetPoints)
{
    if (modelPoint.x < 0.0 || modelPoint.x >= grayImage.cols
        || modelPoint.y < 0.0 || modelPoint.y >= grayImage.rows)
    {
        return;
    }

    const std::vector<int> laserWidthByRow =
        makeLaserWidthByRow(grayImage.rows);
    const LaserDraggedPointResult dragged =
        CLaserLineResponseTools::dragPointToLaserLine(
            grayImage,
            laserWidthByRow,
            cv::Point2i(
                static_cast<int>(std::round(modelPoint.x)),
                static_cast<int>(std::round(modelPoint.y))),
            options.logSearchHalfHeightPixel,
            options.logSearchMinGrayValue,
            nullptr);
    if (!dragged.valid)
    {
        return;
    }

    sourcePoints->push_back(modelPoint);
    targetPoints->push_back(dragged.point);
}

/**
 * @brief 估计二维仿射变换。
 * @param sourcePoints 源点集合，坐标单位 pixel。
 * @param targetPoints 目标点集合，坐标单位 pixel。
 * @param affine 输出 2x3 仿射矩阵，必须非空。
 * @return true 表示样本数量足够且最小二乘求解成功。
 */
bool estimateAffine(
    const std::vector<cv::Point2d> &sourcePoints,
    const std::vector<cv::Point2d> &targetPoints,
    Eigen::Matrix<double, 2, 3> *affine)
{
    if (sourcePoints.size() < 3U || sourcePoints.size() != targetPoints.size()
        || affine == nullptr)
    {
        return false;
    }

    Eigen::MatrixXd matrix(2 * static_cast<int>(sourcePoints.size()), 6);
    Eigen::VectorXd vector(2 * static_cast<int>(sourcePoints.size()));
    matrix.setZero();
    for (int index = 0; index < static_cast<int>(sourcePoints.size()); ++index)
    {
        const cv::Point2d &source =
            sourcePoints[static_cast<std::size_t>(index)];
        const cv::Point2d &target =
            targetPoints[static_cast<std::size_t>(index)];
        matrix(index, 0) = source.x;
        matrix(index, 1) = source.y;
        matrix(index, 2) = 1.0;
        matrix(index + static_cast<int>(sourcePoints.size()), 3) = source.x;
        matrix(index + static_cast<int>(sourcePoints.size()), 4) = source.y;
        matrix(index + static_cast<int>(sourcePoints.size()), 5) = 1.0;
        vector(index) = target.x;
        vector(index + static_cast<int>(sourcePoints.size())) = target.y;
    }

    const Eigen::VectorXd solution =
        matrix.colPivHouseholderQr().solve(vector);
    if (solution.size() != 6)
    {
        return false;
    }

    (*affine)(0, 0) = solution(0);
    (*affine)(0, 1) = solution(1);
    (*affine)(0, 2) = solution(2);
    (*affine)(1, 0) = solution(3);
    (*affine)(1, 1) = solution(4);
    (*affine)(1, 2) = solution(5);
    return true;
}

/**
 * @brief 用二维仿射矩阵变换单个点。
 * @param affine 2x3 仿射矩阵。
 * @param point 输入点，单位 pixel。
 * @return 变换后的点，单位 pixel。
 */
cv::Point2d transformPoint(
    const Eigen::Matrix<double, 2, 3> &affine,
    const cv::Point2d &point)
{
    const Eigen::Vector3d source(point.x, point.y, 1.0);
    const Eigen::Vector2d target = affine * source;
    return cv::Point2d(target.x(), target.y());
}

/**
 * @brief 用二维仿射矩阵批量变换点列。
 * @param affine 2x3 仿射矩阵。
 * @param points 输入点列，单位 pixel。
 * @return 变换后的点列，保持输入顺序。
 */
std::vector<cv::Point2d> transformPoints(
    const Eigen::Matrix<double, 2, 3> &affine,
    const std::vector<cv::Point2d> &points)
{
    std::vector<cv::Point2d> output;
    output.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        output.push_back(transformPoint(affine, point));
    }
    return output;
}

/**
 * @brief 计算线段方向角。
 * @param start 起点，单位 pixel。
 * @param end 终点，单位 pixel。
 * @return 方向角，单位 rad。
 */
double lineAngle(const cv::Point2d &start, const cv::Point2d &end)
{
    return std::atan2(end.y - start.y, end.x - start.x);
}

/**
 * @brief 绕指定中心旋转单个点。
 * @param point 输入点，单位 pixel。
 * @param center 旋转中心，单位 pixel。
 * @param angleRadian 旋转角，单位 rad。
 * @return 旋转后的点，单位 pixel。
 */
cv::Point2d rotateAround(
    const cv::Point2d &point,
    const cv::Point2d &center,
    double angleRadian)
{
    const double cosValue = std::cos(angleRadian);
    const double sinValue = std::sin(angleRadian);
    const double x = point.x - center.x;
    const double y = point.y - center.y;
    return cv::Point2d(
        center.x + cosValue * x - sinValue * y,
        center.y + sinValue * x + cosValue * y);
}

/**
 * @brief 对点列执行绕点旋转后平移。
 * @param points 输入点列，单位 pixel。
 * @param rotationCenter 旋转中心，单位 pixel。
 * @param angleRadian 旋转角，单位 rad。
 * @param translation 平移量，单位 pixel。
 * @return 变换后的点列，保持输入顺序。
 */
std::vector<cv::Point2d> rotateAndTranslatePoints(
    const std::vector<cv::Point2d> &points,
    const cv::Point2d &rotationCenter,
    double angleRadian,
    const cv::Point2d &translation)
{
    std::vector<cv::Point2d> output;
    output.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        const cv::Point2d rotated =
            rotateAround(point, rotationCenter, angleRadian);
        output.push_back(cv::Point2d(
            rotated.x + translation.x,
            rotated.y + translation.y));
    }
    return output;
}

/**
 * @brief 对单个点执行绕点旋转后平移。
 * @param point 输入点，单位 pixel。
 * @param rotationCenter 旋转中心，单位 pixel。
 * @param angleRadian 旋转角，单位 rad。
 * @param translation 平移量，单位 pixel。
 * @return 变换后的点，单位 pixel。
 */
cv::Point2d rotateAndTranslatePoint(
    const cv::Point2d &point,
    const cv::Point2d &rotationCenter,
    double angleRadian,
    const cv::Point2d &translation)
{
    const cv::Point2d rotated = rotateAround(point, rotationCenter, angleRadian);
    return cv::Point2d(rotated.x + translation.x, rotated.y + translation.y);
}

/**
 * @brief 将角度归一化到 [-pi, pi]。
 * @param angleRadian 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad。
 */
double normalizeAngle(double angleRadian)
{
    while (angleRadian > CV_PI)
    {
        angleRadian -= 2.0 * CV_PI;
    }
    while (angleRadian < -CV_PI)
    {
        angleRadian += 2.0 * CV_PI;
    }
    return angleRadian;
}

/**
 * @brief 根据 T 型两侧线段对仿真线执行粗旋转和平移对齐。
 * @param laserPoints 仿真激光线点列，单位 pixel。
 * @param weldPoint 仿真焊缝点，单位 pixel。
 * @param hasWeldPoint true 表示 weldPoint 来自有效输入。
 * @param modelSplitPoint 仿真 split 点，单位 pixel。
 * @param lines 真实图像中拟合得到的左右线段。
 * @param intersectionPoint 真实左右线段交点，单位 pixel。
 * @param coarsePoints 输出粗对齐后的仿真线点列，必须非空。
 * @param coarseWeldPoint 输出粗对齐后的焊缝点，必须非空。
 * @return true 表示粗对齐成功并生成点列。
 */
bool coarseAlignTJoint(
    const std::vector<cv::Point2d> &laserPoints,
    const cv::Point2d &weldPoint,
    bool hasWeldPoint,
    const cv::Point2d &modelSplitPoint,
    const std::vector<WeldLineSegment> &lines,
    const cv::Point2d &intersectionPoint,
    std::vector<cv::Point2d> *coarsePoints,
    cv::Point2d *coarseWeldPoint)
{
    if (laserPoints.size() < 5U || lines.size() < 2U
        || coarsePoints == nullptr || coarseWeldPoint == nullptr)
    {
        return false;
    }

    const cv::Point2d leftModel = laserPoints.front();
    const cv::Point2d rightModel = laserPoints.back();
    const double modelLeftAngle = lineAngle(leftModel, modelSplitPoint);
    const double modelRightAngle = lineAngle(modelSplitPoint, rightModel);
    const double imageLeftAngle =
        lineAngle(lines.front().start, lines.front().end);
    const double imageRightAngle =
        lineAngle(lines.back().start, lines.back().end);

    double angleDeltaLeft = normalizeAngle(imageLeftAngle - modelLeftAngle);
    double angleDeltaRight = normalizeAngle(imageRightAngle - modelRightAngle);
    if (std::abs(angleDeltaLeft - angleDeltaRight) > CV_PI)
    {
        if (angleDeltaLeft < angleDeltaRight)
        {
            angleDeltaLeft += 2.0 * CV_PI;
        }
        else
        {
            angleDeltaRight += 2.0 * CV_PI;
        }
    }

    const double angleDelta = 0.5 * (angleDeltaLeft + angleDeltaRight);
    const cv::Point2d rotatedSplit =
        rotateAround(modelSplitPoint, modelSplitPoint, angleDelta);
    const cv::Point2d translation(
        intersectionPoint.x - rotatedSplit.x,
        intersectionPoint.y - rotatedSplit.y);
    *coarsePoints = rotateAndTranslatePoints(
        laserPoints,
        modelSplitPoint,
        angleDelta,
        translation);
    *coarseWeldPoint = rotateAndTranslatePoint(
        hasWeldPoint ? weldPoint : modelSplitPoint,
        modelSplitPoint,
        angleDelta,
        translation);
    return !coarsePoints->empty();
}

/**
 * @brief 计算点到目标点集的最近距离。
 * @param point 待评估点，单位 pixel。
 * @param targetPoints 目标点集，坐标单位 pixel。
 * @return 最近距离，单位 pixel；目标为空时返回最大 double。
 */
double nearestDistance(
    const cv::Point2d &point,
    const std::vector<cv::Point2d> &targetPoints)
{
    double minDistance = std::numeric_limits<double>::max();
    for (const cv::Point2d &targetPoint : targetPoints)
    {
        minDistance = std::min(minDistance, distanceBetween(point, targetPoint));
    }
    return minDistance;
}

/**
 * @brief 根据匹配点到目标点集的最近距离填充残差统计。
 * @param matchedPoints 已变换的仿真线点列，单位 pixel。
 * @param targetPoints 真实目标点集，单位 pixel。
 * @param result 待写入的匹配结果，必须非空。
 */
void fillResidualStats(
    const std::vector<cv::Point2d> &matchedPoints,
    const std::vector<cv::Point2d> &targetPoints,
    SimulatedLaserMatchingResult *result)
{
    if (matchedPoints.empty() || targetPoints.empty() || result == nullptr)
    {
        return;
    }

    result->minResidualPixel = std::numeric_limits<double>::max();
    result->maxResidualPixel = 0.0;
    double sum = 0.0;
    double squaredSum = 0.0;
    for (const cv::Point2d &point : matchedPoints)
    {
        const double residual = nearestDistance(point, targetPoints);
        result->minResidualPixel =
            std::min(result->minResidualPixel, residual);
        result->maxResidualPixel =
            std::max(result->maxResidualPixel, residual);
        sum += residual;
        squaredSum += residual * residual;
    }

    const double count = static_cast<double>(matchedPoints.size());
    result->meanResidualPixel = sum / count;
    result->rmsResidualPixel = std::sqrt(squaredSum / count);
}

/**
 * @brief 从 replay 文本行中提取所有数值。
 * @param line 单行文本，允许包含旧格式中的标记字符。
 * @return 按出现顺序解析出的 double 数组。
 */
std::vector<double> parseNumbersFromLine(const std::string &line)
{
    std::string normalized;
    normalized.reserve(line.size());
    for (std::size_t index = 0U; index < line.size(); ++index)
    {
        const char character = line[index];
        const bool numeric =
            std::isdigit(static_cast<unsigned char>(character))
            || character == '.'
            || character == '-'
            || character == '+';
        normalized.push_back(numeric ? character : ' ');
    }

    std::istringstream lineStream(normalized);
    std::vector<double> values;
    double value = 0.0;
    while (lineStream >> value)
    {
        values.push_back(value);
    }
    return values;
}

/**
 * @brief 去除字符串首尾空白字符。
 * @param text 输入文本。
 * @return 去除首尾空白后的文本。
 */
std::string trim(std::string text)
{
    while (!text.empty()
        && std::isspace(static_cast<unsigned char>(text.front())))
    {
        text.erase(text.begin());
    }
    while (!text.empty()
        && std::isspace(static_cast<unsigned char>(text.back())))
    {
        text.pop_back();
    }
    return text;
}

/**
 * @brief 判断字符串是否以指定前缀开头。
 * @param text 输入文本。
 * @param prefix 前缀文本。
 * @return true 表示 text 从位置 0 开始匹配 prefix。
 */
bool startsWith(const std::string &text, const std::string &prefix)
{
    return text.rfind(prefix, 0U) == 0U;
}

/**
 * @brief 将多行点坐标文本解析为点行集合。
 * @param text replay 或模板中的点坐标文本。
 * @return 每行一组 OpenCV 点，坐标单位 pixel。
 */
std::vector<std::vector<cv::Point2d>> parsePointRows(const std::string &text)
{
    std::vector<std::vector<cv::Point2d>> rows;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line))
    {
        const std::vector<double> values = parseNumbersFromLine(line);
        std::vector<cv::Point2d> points;
        for (std::size_t index = 0U; index + 1U < values.size(); index += 2U)
        {
            points.push_back(cv::Point2d(
                values[index],
                values[index + 1U]));
        }

        if (!points.empty())
        {
            rows.push_back(points);
        }
    }
    return rows;
}

/**
 * @brief 将数值数组按 x/y 成对追加为 DigitalTwin 图像点。
 * @param values 数值数组，偶数位为 x，奇数位为 y，单位 pixel。
 * @param points 输出点列，必须非空。
 */
void assignPointPairs(
    const std::vector<double> &values,
    std::vector<digital_twin::DigitalTwinImagePoint2D_px> *points)
{
    if (points == nullptr)
    {
        return;
    }

    for (std::size_t index = 0U; index + 1U < values.size(); index += 2U)
    {
        digital_twin::DigitalTwinImagePoint2D_px point;
        point.x_px = values[index];
        point.y_px = values[index + 1U];
        points->push_back(point);
    }
}

/**
 * @brief 解析旧 replay block 文本为仿真激光观测序列。
 * @param text replay block 原始文本。
 * @return 解析结果；至少一帧包含激光点时 valid=true。
 * @note `$` 行开始新帧，`#` 行结束当前帧；其它行按旧字段前缀解析。
 */
SimulatedLaserReplayParseResult parseReplayBlockText(
    const std::string &text)
{
    SimulatedLaserReplayParseResult result;
    std::istringstream input(text);
    std::string line;
    digital_twin::DigitalTwinVisionLaserObservation currentFrame;
    bool inFrame = false;
    int nextFrameIndex = 0;
    while (std::getline(input, line))
    {
        const std::string trimmed = trim(line);
        if (trimmed.empty())
        {
            continue;
        }

        if (trimmed[0] == '$')
        {
            if (inFrame && !currentFrame.laserLineImagePoints_px.empty())
            {
                currentFrame.valid =
                    currentFrame.laserLineImagePoints_px.size() >= 2U
                    && currentFrame.hasWeldPointImage_px;
                result.observations.push_back(currentFrame);
            }

            currentFrame = digital_twin::DigitalTwinVisionLaserObservation();
            currentFrame.frameId = nextFrameIndex++;
            const std::vector<double> values = parseNumbersFromLine(trimmed);
            if (!values.empty())
            {
                currentFrame.frameId =
                    static_cast<qint64>(std::llround(values.front()));
            }
            inFrame = true;
            continue;
        }

        if (trimmed[0] == '#')
        {
            if (inFrame && !currentFrame.laserLineImagePoints_px.empty())
            {
                currentFrame.valid =
                    currentFrame.laserLineImagePoints_px.size() >= 2U
                    && currentFrame.hasWeldPointImage_px;
                result.observations.push_back(currentFrame);
            }
            currentFrame = digital_twin::DigitalTwinVisionLaserObservation();
            inFrame = false;
            continue;
        }

        if (!inFrame)
        {
            currentFrame = digital_twin::DigitalTwinVisionLaserObservation();
            currentFrame.frameId = nextFrameIndex++;
            inFrame = true;
        }

        const std::vector<double> values = parseNumbersFromLine(trimmed);
        if (startsWith(trimmed, "WeldImgPnt"))
        {
            if (values.size() >= 2U)
            {
                currentFrame.weldPointImage_px.x_px = values[0];
                currentFrame.weldPointImage_px.y_px = values[1];
                currentFrame.hasWeldPointImage_px = true;
            }
        }
        else if (startsWith(trimmed, "OriLaserImgPnts"))
        {
            assignPointPairs(values, &currentFrame.laserLineImagePoints_px);
        }
        else
        {
            // 旧 replay 中的 RobotPose 等附加字段当前不参与 Vision 算法。
        }
    }

    if (inFrame && !currentFrame.laserLineImagePoints_px.empty())
    {
        currentFrame.valid =
            currentFrame.laserLineImagePoints_px.size() >= 2U
            && currentFrame.hasWeldPointImage_px;
        result.observations.push_back(currentFrame);
    }

    result.valid = !result.observations.empty();
    result.message = result.valid
        ? "simulated laser replay block parsed"
        : "no valid replay block frames";
    return result;
}

/**
 * @brief 计算点集质心。
 * @param points 输入点集，坐标单位 pixel。
 * @return 点集平均坐标；输入为空时返回 (0, 0)。
 */
cv::Point2d centroidOf(const std::vector<cv::Point2d> &points)
{
    cv::Point2d center(0.0, 0.0);
    if (points.empty())
    {
        return center;
    }

    for (const cv::Point2d &point : points)
    {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= static_cast<double>(points.size());
    center.y /= static_cast<double>(points.size());
    return center;
}

/**
 * @brief 将模板点列按角度和平移转换到候选位置。
 * @param points 模板点列，坐标单位 pixel。
 * @param center 旋转中心，单位 pixel。
 * @param angleDegree 旋转角，单位 degree。
 * @param dx x 方向平移，单位 pixel。
 * @param dy y 方向平移，单位 pixel。
 * @return 转换后的模板点列。
 */
std::vector<cv::Point2d> transformTemplatePoints(
    const std::vector<cv::Point2d> &points,
    const cv::Point2d &center,
    double angleDegree,
    double dx,
    double dy)
{
    return rotateAndTranslatePoints(
        points,
        center,
        angleDegree * CV_PI / 180.0,
        cv::Point2d(dx, dy));
}

/**
 * @brief 计算模板点列在距离变换图上的 Chamfer 得分。
 * @param distanceImage 真实边缘/骨架距离变换图，类型为 `CV_32F`。
 * @param points 已变换模板点列，坐标单位 pixel。
 * @param maxDistancePixel 距离截断上限，单位 pixel。
 * @return 归一化得分，范围 0~1，越大表示越贴近真实线。
 */
double chamferScore(
    const cv::Mat &distanceImage,
    const std::vector<cv::Point2d> &points,
    double maxDistancePixel)
{
    if (distanceImage.empty() || points.empty() || maxDistancePixel <= 0.0)
    {
        return 0.0;
    }

    double sumDistance = 0.0;
    int count = 0;
    for (const cv::Point2d &point : points)
    {
        const int x = static_cast<int>(std::round(point.x));
        const int y = static_cast<int>(std::round(point.y));
        if (x < 0 || y < 0 || x >= distanceImage.cols
            || y >= distanceImage.rows)
        {
            sumDistance += maxDistancePixel;
            ++count;
            continue;
        }

        sumDistance += std::min(
            maxDistancePixel,
            static_cast<double>(distanceImage.at<float>(y, x)));
        ++count;
    }

    if (count == 0)
    {
        return 0.0;
    }

    const double meanDistance = sumDistance / static_cast<double>(count);
    return std::max(0.0, std::min(1.0, 1.0 - meanDistance / maxDistancePixel));
}

/**
 * @brief 规范化 shape template 图像尺寸。
 * @param requestedSize 调用方请求的模板尺寸，单位 pixel。
 * @return 不小于 11 的奇数尺寸，单位 pixel。
 */
int normalizeTemplateSize(int requestedSize)
{
    int size = std::max(11, requestedSize);
    if (size % 2 == 0)
    {
        ++size;
    }
    return size;
}

/**
 * @brief 以焊缝点为中心构造模板 polyline。
 * @param laserPoints 仿真激光线点列，坐标单位 pixel。
 * @param weldPoint 仿真焊缝点，坐标单位 pixel。
 * @param templateSize 模板图像尺寸，单位 pixel。
 * @return 平移到模板局部坐标系的整数 polyline 点列。
 */
std::vector<cv::Point> buildTemplatePolyline(
    const std::vector<cv::Point2d> &laserPoints,
    const cv::Point2d &weldPoint,
    int templateSize)
{
    std::vector<cv::Point> points;
    const int center = templateSize / 2;
    points.reserve(laserPoints.size());
    for (const cv::Point2d &point : laserPoints)
    {
        points.push_back(cv::Point(
            static_cast<int>(std::round(point.x - weldPoint.x + center)),
            static_cast<int>(std::round(point.y - weldPoint.y + center))));
    }
    return points;
}

/**
 * @brief 构造 shape_based_matching 使用的二值模板图。
 * @param laserPoints 仿真激光线点列，坐标单位 pixel。
 * @param weldPoint 仿真焊缝点，坐标单位 pixel。
 * @param templateSize 模板图像尺寸，单位 pixel。
 * @return `CV_8UC1` 模板图；焊缝点被放置在模板中心。
 */
cv::Mat buildShapeTemplateImage(
    const std::vector<cv::Point2d> &laserPoints,
    const cv::Point2d &weldPoint,
    int templateSize)
{
    // 旧 HALCON shape model 由仿真激光线生成模板；这里把焊缝点放到
    // 模板中心，下游匹配点即可直接还原为焊缝点坐标。
    cv::Mat image = cv::Mat::zeros(templateSize, templateSize, CV_8UC1);
    const std::vector<cv::Point> points =
        buildTemplatePolyline(laserPoints, weldPoint, templateSize);
    for (std::size_t index = 1U; index < points.size(); ++index)
    {
        cv::line(image, points[index - 1U], points[index], cv::Scalar(255), 1);
    }
    if (points.size() == 1U)
    {
        cv::circle(image, points.front(), 1, cv::Scalar(255), cv::FILLED);
    }

    cv::Mat wide;
    cv::dilate(
        image,
        wide,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    return wide;
}

/**
 * @brief 围绕焊缝点构造模板匹配 ROI。
 * @param grayImage 输入灰度图，必须非空。
 * @param weldPoint 焊缝点，单位 pixel。
 * @param halfSize ROI 半径，单位 pixel。
 * @param minimumSize ROI 最小边长，单位 pixel。
 * @param roi 输出 ROI，必须非空。
 * @return true 表示 ROI 在图像内且尺寸足够容纳模板。
 */
bool buildTemplateRoi(
    const cv::Mat &grayImage,
    const cv::Point2d &weldPoint,
    int halfSize,
    int minimumSize,
    cv::Rect *roi)
{
    if (roi == nullptr || grayImage.empty())
    {
        return false;
    }

    const int radius = std::max(minimumSize / 2 + 1, halfSize);
    const int minX = std::max(
        0,
        static_cast<int>(std::floor(weldPoint.x - static_cast<double>(radius))));
    const int minY = std::max(
        0,
        static_cast<int>(std::floor(weldPoint.y - static_cast<double>(radius))));
    const int maxX = std::min(
        grayImage.cols,
        static_cast<int>(std::ceil(weldPoint.x + static_cast<double>(radius) + 1.0)));
    const int maxY = std::min(
        grayImage.rows,
        static_cast<int>(std::ceil(weldPoint.y + static_cast<double>(radius) + 1.0)));
    if (maxX - minX < minimumSize || maxY - minY < minimumSize)
    {
        return false;
    }

    *roi = cv::Rect(minX, minY, maxX - minX, maxY - minY);
    return true;
}

/**
 * @brief 将模板匹配耗时累计到目标结果。
 * @param source 源匹配结果。
 * @param target 目标匹配结果，必须非空。
 */
void copyTemplateTiming(
    const SimulatedLaserMatchingResult &source,
    SimulatedLaserMatchingResult *target)
{
    if (target == nullptr)
    {
        return;
    }

    target->templateModelBuildMs += source.templateModelBuildMs;
    target->templateAddMs += source.templateAddMs;
    target->templateMatchMs += source.templateMatchMs;
    target->templateTotalMs += source.templateTotalMs;
}

/**
 * @brief 使用 shape_based_matching 后端匹配仿真激光模板。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param observation 单帧仿真激光观测，需包含焊缝点。
 * @param options 模板尺寸、ROI、金字塔和相似度阈值等匹配参数。
 * @return 匹配结果；失败时 valid=false 并携带 message。
 * @note 该路径替代旧 HALCON shape model 的模板创建和局部搜索语义。
 */
SimulatedLaserMatchingResult matchByShapeBased(
    const cv::Mat &grayImage,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const SimulatedLaserMatchingOptions &options)
{
    // 1. 输入校验：shape_based_matching 需要仿真焊缝点作为模板中心。
    SimulatedLaserMatchingResult result;
    result.templateMethod = "shape_based_matching";
    if (!isValidGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }
    if (!observation.hasWeldPointImage_px)
    {
        result.message = "shape_based_matching requires simulated weld point";
        return result;
    }

    const std::vector<cv::Point2d> templateLaserPoints =
        toCvPoints(observation.laserLineImagePoints_px);
    if (templateLaserPoints.size() < 3U)
    {
        result.message = "not enough template laser points";
        return result;
    }

    const cv::Point2d templateWeldPoint =
        toCvPoint(observation.weldPointImage_px);
    const int templateSize =
        normalizeTemplateSize(options.shapeBasedTemplateSizePixel);
    cv::Rect roi;
    if (!buildTemplateRoi(
            grayImage,
            templateWeldPoint,
            options.shapeBasedRoiHalfSizePixel,
            templateSize,
            &roi))
    {
        result.message = "shape_based_matching ROI is smaller than template";
        return result;
    }

    // 2. 模板生成：把仿真激光线画到固定尺寸模板图，焊缝点位于中心。
    const cv::Mat templateImage = buildShapeTemplateImage(
        templateLaserPoints,
        templateWeldPoint,
        templateSize);
    cv::Mat templateMask;
    cv::threshold(templateImage, templateMask, 0.0, 255.0, cv::THRESH_BINARY);
    cv::dilate(
        templateMask,
        templateMask,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));

    cv::Mat roiImage = grayImage(roi).clone();
    const int paddedWidth = ((roiImage.cols + 31) / 32) * 32;
    const int paddedHeight = ((roiImage.rows + 31) / 32) * 32;
    cv::Mat padded = cv::Mat::zeros(paddedHeight, paddedWidth, CV_8UC1);
    roiImage.copyTo(padded(cv::Rect(0, 0, roiImage.cols, roiImage.rows)));

    // 3. 角度模板注册：对应 HALCON CreateShapeModel 的 OpenCV 替代。
    shape_based_matching::shapeInfo_producer shapes(templateImage, templateMask);
    const double angleRange = std::abs(options.templateAngleRangeDegree);
    if (angleRange <= 1.0e-6)
    {
        shapes.angle_range = {0.0f};
    }
    else
    {
        shapes.angle_range = {
            static_cast<float>(-angleRange),
            static_cast<float>(angleRange)};
    }
    shapes.angle_step =
        static_cast<float>(std::max(0.1, options.templateAngleStepDegree));
    shapes.scale_range = {1.0f};

    const auto modelStart = std::chrono::steady_clock::now();
    shapes.produce_infos();
    const auto modelStop = std::chrono::steady_clock::now();
    result.templateModelBuildMs =
        std::chrono::duration<double, std::milli>(
            modelStop - modelStart).count();

    std::vector<int> pyramidLevels = options.shapeBasedPyramidLevels;
    if (pyramidLevels.empty())
    {
        pyramidLevels = {4, 8};
    }

    std::map<int, shape_based_matching::shapeInfo_producer::Info>
        infosByTemplate;
    const std::string classId = "simulated_laser";
    const auto addStart = std::chrono::steady_clock::now();
    line2Dup::Detector detector(
        128,
        pyramidLevels,
        static_cast<float>(options.shapeBasedWeakThreshold),
        static_cast<float>(options.shapeBasedStrongThreshold));
    for (const auto &info : shapes.infos)
    {
        const int templateId = detector.addTemplate(
            shapes.src_of(info),
            classId,
            shapes.mask_of(info));
        if (templateId >= 0)
        {
            infosByTemplate.emplace(templateId, info);
        }
    }
    const auto addStop = std::chrono::steady_clock::now();
    result.templateAddMs =
        std::chrono::duration<double, std::milli>(
            addStop - addStart).count();

    if (detector.numTemplates(classId) <= 0)
    {
        result.templateTotalMs = result.templateModelBuildMs
            + result.templateAddMs + result.templateMatchMs;
        result.message = "shape_based_matching failed to create templates";
        return result;
    }

    // 4. ROI 搜索：旧工程在仿真焊缝点附近局部找 shape model；这里保持
    //    同样的局部搜索边界，避免全图误匹配。
    const std::vector<std::string> classIds{classId};
    const auto matchStart = std::chrono::steady_clock::now();
    const std::vector<line2Dup::Match> matches = detector.match(
        padded,
        static_cast<float>(options.shapeBasedMinSimilarity),
        classIds);
    const auto matchStop = std::chrono::steady_clock::now();
    result.templateMatchMs =
        std::chrono::duration<double, std::milli>(
            matchStop - matchStart).count();
    result.templateTotalMs = result.templateModelBuildMs
        + result.templateAddMs + result.templateMatchMs;

    if (matches.empty())
    {
        result.message = "shape_based_matching found no match";
        return result;
    }

    const line2Dup::Match &match = matches.front();
    const std::vector<line2Dup::Template> &templates =
        detector.getTemplates(classId, match.template_id);
    if (templates.empty())
    {
        result.message = "shape_based_matching returned empty template pyramid";
        return result;
    }

    const double center = static_cast<double>(templateSize / 2);
    result.weldPoint = cv::Point2d(
        static_cast<double>(roi.x + match.x - templates.front().tl_x) + center,
        static_cast<double>(roi.y + match.y - templates.front().tl_y) + center);
    result.templateScore = static_cast<double>(match.similarity) / 100.0;
    const auto infoIt = infosByTemplate.find(match.template_id);
    if (infoIt != infosByTemplate.end())
    {
        result.templateAngleDegree = static_cast<double>(infoIt->second.angle);
    }

    const bool pointInsideRoi = result.weldPoint.x >= roi.x
        && result.weldPoint.y >= roi.y
        && result.weldPoint.x < roi.x + roi.width
        && result.weldPoint.y < roi.y + roi.height;
    if (match.similarity < options.shapeBasedMinSimilarity || !pointInsideRoi)
    {
        result.message = "shape_based_matching score or point is invalid";
        return result;
    }

    // 5. 结果还原：模板中心就是仿真焊缝点，因此按匹配角度旋转仿真线，
    //    再平移到匹配焊缝点即可得到 matched laser。
    result.matchedLaserPoints = rotateAndTranslatePoints(
        templateLaserPoints,
        templateWeldPoint,
        result.templateAngleDegree * CV_PI / 180.0,
        cv::Point2d(
            result.weldPoint.x - templateWeldPoint.x,
            result.weldPoint.y - templateWeldPoint.y));
    result.confidence = result.templateScore;
    result.usedTemplateFallback = true;
    result.valid = true;
    result.message = "simulated laser matched by shape_based_matching";
    return result;
}

/**
 * @brief 使用 Chamfer 距离搜索匹配仿真激光模板。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param observation 单帧仿真激光观测。
 * @param options 平移、角度、距离上限和最低得分等匹配参数。
 * @return 匹配结果；低于 `templateMinScore` 时 valid=false。
 * @note 该路径是 shape_based_matching 后端失败或关闭时的轻量备用路径。
 */
SimulatedLaserMatchingResult matchByChamfer(
    const cv::Mat &grayImage,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const SimulatedLaserMatchingOptions &options)
{
    // 1. 输入校验：Chamfer 备用路径只依赖距离变换和仿真线点列。
    const auto totalStart = std::chrono::steady_clock::now();
    SimulatedLaserMatchingResult result;
    result.templateMethod = "chamfer";
    if (!isValidGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }

    const std::vector<cv::Point2d> templateLaserPoints =
        toCvPoints(observation.laserLineImagePoints_px);
    if (templateLaserPoints.size() < 3U)
    {
        result.message = "not enough template laser points";
        return result;
    }

    // 2. 生成距离图：二值亮线的反图做 distanceTransform，Chamfer 得分越高
    //    表示模板点越贴近真实亮线。
    cv::Mat brightMask;
    cv::threshold(
        grayImage,
        brightMask,
        static_cast<double>(std::max(1, options.logSearchMinGrayValue)),
        255.0,
        cv::THRESH_BINARY);
    cv::Mat inverseMask;
    cv::bitwise_not(brightMask, inverseMask);
    cv::Mat distanceImage;
    cv::distanceTransform(inverseMask, distanceImage, cv::DIST_L2, 3);

    const cv::Point2d center = centroidOf(templateLaserPoints);
    double bestScore = -1.0;
    double bestAngle = 0.0;
    double bestDx = 0.0;
    double bestDy = 0.0;
    const int searchRadius = std::max(0, options.templateSearchRadiusPixel);
    const int searchStep = std::max(1, options.templateSearchStepPixel);
    const double angleStep = std::max(0.1, options.templateAngleStepDegree);

    // 3. 小范围平移/旋转搜索：作为 shape_based_matching 失败后的轻量备用。
    const auto matchStart = std::chrono::steady_clock::now();
    for (double angle = -std::abs(options.templateAngleRangeDegree);
         angle <= std::abs(options.templateAngleRangeDegree) + 1.0e-6;
         angle += angleStep)
    {
        for (int dy = -searchRadius; dy <= searchRadius; dy += searchStep)
        {
            for (int dx = -searchRadius; dx <= searchRadius; dx += searchStep)
            {
                const std::vector<cv::Point2d> transformed =
                    transformTemplatePoints(
                        templateLaserPoints,
                        center,
                        angle,
                        static_cast<double>(dx),
                        static_cast<double>(dy));
                const double score = chamferScore(
                    distanceImage,
                    transformed,
                    options.templateMaxDistancePixel);
                if (score > bestScore)
                {
                    bestScore = score;
                    bestAngle = angle;
                    bestDx = static_cast<double>(dx);
                    bestDy = static_cast<double>(dy);
                }
            }
        }
    }
    const auto matchStop = std::chrono::steady_clock::now();
    result.templateMatchMs =
        std::chrono::duration<double, std::milli>(
            matchStop - matchStart).count();

    result.templateScore = std::max(0.0, bestScore);
    result.templateAngleDegree = bestAngle;
    result.usedTemplateFallback = true;
    result.matchedLaserPoints = transformTemplatePoints(
        templateLaserPoints,
        center,
        bestAngle,
        bestDx,
        bestDy);
    const cv::Point2d templateWeldPoint = observation.hasWeldPointImage_px
        ? toCvPoint(observation.weldPointImage_px)
        : centroidOf(templateLaserPoints);
    result.weldPoint = transformTemplatePoints(
        std::vector<cv::Point2d>{templateWeldPoint},
        center,
        bestAngle,
        bestDx,
        bestDy).front();
    result.confidence = result.templateScore;
    result.valid = result.templateScore >= options.templateMinScore;
    result.message = result.valid
        ? "simulated laser matched by chamfer"
        : "simulated template score is too low";
    const auto totalStop = std::chrono::steady_clock::now();
    result.templateTotalMs =
        std::chrono::duration<double, std::milli>(
            totalStop - totalStart).count();
    return result;
}

} // namespace

/**
 * @brief 解析离线 replay 文本为仿真激光观测。
 * @param laserText 激光线 replay 文本，支持 block 格式或逐行点列格式。
 * @param weldPointText 焊缝点 replay 文本，逐行取第一个点。
 * @return 解析结果；无有效帧时 valid=false。
 * @note 运行时实时链路不调用该解析器，外部线程应直接提供观测结构。
 */
SimulatedLaserReplayParseResult CSimulatedLaserReplayParser::parse(
    const std::string &laserText,
    const std::string &weldPointText)
{
    // 1. 兼容 replay 块格式：该解析器只服务离线测试，运行时由 DigitalTwin
    //    或兼容回调层直接提供 Vision observation。
    SimulatedLaserReplayParseResult result;
    if (laserText.find("WeldImgPnt") != std::string::npos
        || laserText.find("OriLaserImgPnts") != std::string::npos
        || laserText.find('$') != std::string::npos)
    {
        result = parseReplayBlockText(laserText);
        if (result.valid)
        {
            return result;
        }
    }

    // 2. 普通 replay 文本：每行是一帧激光线，焊缝点文本每行取第一个点。
    const std::vector<std::vector<cv::Point2d>> laserRows =
        parsePointRows(laserText);
    const std::vector<std::vector<cv::Point2d>> weldRows =
        parsePointRows(weldPointText);
    if (laserRows.empty())
    {
        result.message = "empty simulated laser text";
        return result;
    }

    for (int index = 0; index < static_cast<int>(laserRows.size()); ++index)
    {
        digital_twin::DigitalTwinVisionLaserObservation frame;
        frame.frameId = index;
        frame.laserLineImagePoints_px =
            toDigitalTwinImagePoints(laserRows[static_cast<std::size_t>(index)]);
        if (index < static_cast<int>(weldRows.size())
            && !weldRows[static_cast<std::size_t>(index)].empty())
        {
            frame.weldPointImage_px =
                toDigitalTwinImagePoint(
                    weldRows[static_cast<std::size_t>(index)].front());
            frame.hasWeldPointImage_px = true;
        }
        frame.valid =
            frame.laserLineImagePoints_px.size() >= 2U
            && frame.hasWeldPointImage_px;
        result.observations.push_back(frame);
    }

    result.valid = !result.observations.empty();
    result.message = result.valid
        ? "simulated laser replay parsed"
        : "no valid simulated laser frames";
    return result;
}

/**
 * @brief 使用模板匹配估计仿真激光线在真实图像中的位置。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param observation 单帧仿真激光观测。
 * @param options shape_based_matching 和 Chamfer fallback 参数。
 * @return 模板匹配结果；主路径失败时可按配置退回 Chamfer。
 */
SimulatedLaserMatchingResult CSimulatedLaserTemplateMatcher::match(
    const cv::Mat &grayImage,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const SimulatedLaserMatchingOptions &options)
{
    // 1. 主路径：对应旧 TemplateMatching_change 的 shape model 语义，
    //    使用 shape_based_matching 做 OpenCV 工程替代。
    SimulatedLaserMatchingResult shapeResult;
    if (options.enableShapeBasedTemplateMatching)
    {
        shapeResult = matchByShapeBased(grayImage, observation, options);
        if (shapeResult.valid)
        {
            return shapeResult;
        }
    }

    // 2. 备用路径：shape_based_matching 失败时才调用当前 Chamfer 方法。
    if (!options.enableChamferTemplateFallback)
    {
        if (options.enableShapeBasedTemplateMatching)
        {
            return shapeResult;
        }

        SimulatedLaserMatchingResult result;
        result.message =
            "shape_based_matching disabled and chamfer fallback disabled";
        return result;
    }

    SimulatedLaserMatchingResult chamferResult =
        matchByChamfer(grayImage, observation, options);
    if (options.enableShapeBasedTemplateMatching)
    {
        copyTemplateTiming(shapeResult, &chamferResult);
        chamferResult.usedChamferFallback = true;
        chamferResult.templateMethod = "chamfer_fallback";
        chamferResult.message = std::string("shape_based_matching failed: ")
            + shapeResult.message + "; chamfer: " + chamferResult.message;
    }

    return chamferResult;
}

/**
 * @brief 使用骨架/LoGSke 点对配准 T 型仿真激光线。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param observation 单帧仿真激光观测。
 * @param options 骨架阈值、split/RANSAC、LoGSke 和置信度参数。
 * @return T 型配准结果；骨架不足或仿射估计失败时 valid=false。
 */
SimulatedLaserMatchingResult CSimulatedLaserRegistrationMatcher::match(
    const cv::Mat &grayImage,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const SimulatedLaserMatchingOptions &options)
{
    // 1. 输入校验：配准路径消费实时仿真观测，离线 replay 只在测试层转换。
    SimulatedLaserMatchingResult result;
    if (!isValidGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }

    const std::vector<cv::Point2d> laserPoints =
        toCvPoints(observation.laserLineImagePoints_px);
    const cv::Point2d weldPoint = toCvPoint(observation.weldPointImage_px);
    if (laserPoints.size() < 5U)
    {
        result.message = "not enough simulated laser points";
        return result;
    }

    // 2. 骨架输入：T 几何分支必须直接跟随真实亮线骨架。方向形态学
    //    增强可能在 V/T 结构上产生差分边缘，本分支不再用它生成主路径。
    const cv::Mat skeletonInput = grayImage;

    // 3. 公共骨架提取：完整 skeletonMask 保留 HALCON-like 骨架；
    //    T 几何分支读取旧 Feng 风格 orderedPathPoints，避免按列平均
    //    把两条斜线压成水平伪中心线。
    LaserSkeletonExtractionOptions skeletonOptions;
    skeletonOptions.thresholdValue = options.skeletonThresholdValue;
    skeletonOptions.minComponentArea = 20;
    skeletonOptions.enableLaserRegionPruning = true;
    skeletonOptions.enableHoughFallback = false;
    skeletonOptions.enableLogSupportedComponentFilter = true;
    skeletonOptions.enableOrderedPathExtraction = true;
    const WeldSkeletonFeatureResult skeleton =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(
            skeletonInput,
            skeletonOptions);
    const std::vector<cv::Point2d> &skeletonPathPoints =
        skeleton.orderedPathPoints;
    if (!skeleton.valid || skeletonPathPoints.size() < 5U)
    {
        result.skeletonPoints = skeletonPathPoints;
        result.message = "failed to extract ordered laser skeleton path";
        return result;
    }

    result.skeletonPoints = skeletonPathPoints;
    result.usedHoughFallback = false;
    const std::vector<WeldFeaturePoint> skeletonFeaturePoints =
        toFeaturePoints(skeletonPathPoints);
    const cv::Point2d referencePoint =
        observation.hasWeldPointImage_px ? weldPoint : laserPoints[
            static_cast<std::size_t>(std::max(0,
                findSplitPointIndex(laserPoints)))];
    // 4. T 型分段和 RANSAC：沿真实骨架分段拟合，随后在参考列附近求交点。
    result.fittedLines =
        fitTJointLines(skeletonFeaturePoints, referencePoint, options);
    if (result.fittedLines.size() < 2U
        || !result.fittedLines.front().valid
        || !result.fittedLines.back().valid)
    {
        result.message = "failed to fit T joint lines";
        return result;
    }

    const WeldLineIntersectionResult intersection =
        CLineGeometryTools::intersectLinesNearReferenceX(
            result.fittedLines,
            referencePoint.x);
    result.intersectionPoint = intersection.valid
        ? intersection.point
        : evaluateLineAtX(result.fittedLines.front(), referencePoint.x);

    // 5. 粗配准：用仿真线最大分裂点对应真实交点，左右段角度平均补偿。
    const int splitIndex = std::max(0, findSplitPointIndex(laserPoints));
    const cv::Point2d modelSplitPoint =
        laserPoints[static_cast<std::size_t>(splitIndex)];
    cv::Point2d coarseWeldPoint;
    result.coarseValid = coarseAlignTJoint(
        laserPoints,
        weldPoint,
        observation.hasWeldPointImage_px,
        modelSplitPoint,
        result.fittedLines,
        result.intersectionPoint,
        &result.coarseMatchedLaserPoints,
        &coarseWeldPoint);
    const std::vector<cv::Point2d> &matchingSourcePoints =
        result.coarseValid ? result.coarseMatchedLaserPoints : laserPoints;
    const cv::Point2d matchingSplitPoint = result.coarseValid
        ? result.coarseMatchedLaserPoints[static_cast<std::size_t>(splitIndex)]
        : modelSplitPoint;
    const cv::Point2d matchingWeldPoint =
        result.coarseValid ? coarseWeldPoint : weldPoint;

    // 6. LoGSke 风格精配准：沿仿真线采样点做同列拖拽，得到 affine 点对。
    std::vector<cv::Point2d> sourcePairs;
    std::vector<cv::Point2d> targetPairs;
    for (int index = 0; index < static_cast<int>(matchingSourcePoints.size());
         index += std::max(1, options.logSearchPointStep))
    {
        appendLogSearchPair(
            grayImage,
            matchingSourcePoints[static_cast<std::size_t>(index)],
            options,
            &sourcePairs,
            &targetPairs);
    }
    if (sourcePairs.size() < 3U)
    {
        for (int index = 0; index < static_cast<int>(matchingSourcePoints.size());
             index += std::max(1, options.logSearchPointStep))
        {
            const cv::Point2d &modelPoint =
                matchingSourcePoints[static_cast<std::size_t>(index)];
            sourcePairs.push_back(modelPoint);
            targetPairs.push_back(targetPointForModel(
                modelPoint,
                matchingSplitPoint,
                result.fittedLines));
        }
    }

    // 7. affine 最小二乘：点对不足时退回到交点平移，保证输出稳定可诊断。
    Eigen::Matrix<double, 2, 3> affine;
    if (!estimateAffine(sourcePairs, targetPairs, &affine))
    {
        affine << 1.0, 0.0, result.intersectionPoint.x - matchingSplitPoint.x,
            0.0, 1.0, result.intersectionPoint.y - matchingSplitPoint.y;
    }

    result.matchedLaserPoints.reserve(matchingSourcePoints.size());
    for (const cv::Point2d &point : matchingSourcePoints)
    {
        result.matchedLaserPoints.push_back(transformPoint(affine, point));
    }
    result.weldPoint = observation.hasWeldPointImage_px
        ? transformPoint(affine, matchingWeldPoint)
        : transformPoint(affine, matchingSplitPoint);
    fillResidualStats(
        result.matchedLaserPoints,
        toPoints(skeletonFeaturePoints),
        &result);

    result.confidence = std::max(
        0.0,
        std::min(1.0, 1.0 - result.meanResidualPixel / 30.0));
    result.valid = result.confidence >= options.minConfidence;
    result.message = result.valid
        ? "simulated T joint laser matched"
        : "simulated laser residual is too high";
    return result;
}

} // namespace i2r_rlc::vision


