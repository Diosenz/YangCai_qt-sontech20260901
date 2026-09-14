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
 * @file        ActiveContourWeldFeatureDetector.cpp
 * @brief       主动轮廓焊缝特征检测实现
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

#include <opencv2/imgproc.hpp>

#include "feature/ActiveContourWeldFeatureDetector.h"

namespace i2r_rlc::vision {

namespace {

constexpr double kEpsilon = 1.0e-6;

/**
 * @brief 将比例或置信度限制到 0~1 范围。
 * @param value 输入比例或置信度，无量纲 ratio。
 * @return 限制到 `[0, 1]` 后的值。
 * @note 用于统一 confidence、gapWidth ratio 等归一化结果的边界。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 检查主动轮廓算法输入是否为非空 CV_8UC1 灰度图。
 * @param gray8 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 * @note 主动轮廓检测不在本层执行 UI 图像、彩色图或相机帧格式转换。
 */
bool isValidGrayImage(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 判断整数像素坐标是否位于图像范围内。
 * @param gray8 用于提供宽高边界的输入图像。
 * @param xPixel x 坐标，单位为 pixel。
 * @param yPixel y 坐标，单位为 pixel。
 * @return true 表示坐标位于 `[0, cols)` 与 `[0, rows)` 范围内。
 */
bool isInsideImage(const cv::Mat &gray8, int xPixel, int yPixel)
{
    return xPixel >= 0
        && xPixel < gray8.cols
        && yPixel >= 0
        && yPixel < gray8.rows;
}

/**
 * @brief 安全读取灰度值，越界时返回 0 作为暗背景。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param xPixel x 坐标，单位为 pixel。
 * @param yPixel y 坐标，单位为 pixel。
 * @return 像素灰度值，范围为 0~255；越界时返回 0。
 * @note gap 扫描和邻域检测将越界区域视为暗背景，减少调用侧重复边界判断。
 */
int grayAt(const cv::Mat &gray8, int xPixel, int yPixel)
{
    if (!isInsideImage(gray8, xPixel, yPixel))
    {
        return 0;
    }

    return gray8.at<unsigned char>(yPixel, xPixel);
}

/**
 * @brief 根据 pixel 坐标、置信度和标签构造焊缝特征点。
 * @param point 输入点，坐标单位为 pixel。
 * @param confidence 输入置信度，范围期望为 0~1。
 * @param label 诊断标签。
 * @return 填充坐标、置信度、有效标志和标签后的焊缝特征点。
 * @note 置信度会通过 `clampRatio` 限制到 0~1，置信度大于 0 时标记为有效。
 */
WeldFeaturePoint makeFeaturePoint(
    const cv::Point2d &point,
    double confidence,
    const std::string &label)
{
    WeldFeaturePoint featurePoint;
    featurePoint.xPixel = point.x;
    featurePoint.yPixel = point.y;
    featurePoint.confidence = clampRatio(confidence);
    featurePoint.valid = featurePoint.confidence > 0.0;
    featurePoint.label = label;
    return featurePoint;
}

/**
 * @brief 按 x/y 排序并删除完全重复的曲线点。
 * @param points 输入曲线点，坐标单位为 pixel。
 * @return 按 x/y 升序排序且去除完全重复坐标后的点列。
 * @note 该排序保证 ROI 选点、gap 扫描和线段拟合使用稳定的曲线顺序。
 */
std::vector<cv::Point2d> sortedUniquePoints(
    const std::vector<cv::Point2d> &points)
{
    std::vector<cv::Point2d> sortedPoints = points;
    std::sort(
        sortedPoints.begin(),
        sortedPoints.end(),
        [](const cv::Point2d &left, const cv::Point2d &right) {
            if (std::fabs(left.x - right.x) <= 1.0e-6)
            {
                return left.y < right.y;
            }
            return left.x < right.x;
        });
    sortedPoints.erase(
        std::unique(
            sortedPoints.begin(),
            sortedPoints.end(),
            [](const cv::Point2d &left, const cv::Point2d &right) {
                return std::fabs(left.x - right.x) <= 1.0e-6
                    && std::fabs(left.y - right.y) <= 1.0e-6;
            }),
        sortedPoints.end());
    return sortedPoints;
}

/**
 * @brief 将 OpenCV 点列转换为带统一标签的焊缝特征点。
 * @param points 输入点列，坐标单位为 pixel。
 * @param label 写入每个特征点的诊断标签。
 * @return 置信度为 1.0 的焊缝特征点集合。
 * @note 用于把曲线候选点交给旧几何工具接口处理。
 */
std::vector<WeldFeaturePoint> toFeaturePoints(
    const std::vector<cv::Point2d> &points,
    const std::string &label)
{
    std::vector<WeldFeaturePoint> featurePoints;
    featurePoints.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        featurePoints.push_back(makeFeaturePoint(point, 1.0, label));
    }
    return featurePoints;
}

/**
 * @brief 从主动轮廓曲线中截取动态 ROI 内的候选点。
 * @param curve 当前主动轮廓曲线，坐标单位为 pixel。
 * @param centerXPixel ROI 中心 x 坐标，单位为 pixel。
 * @param halfWidthPixel ROI 半宽，单位为 pixel。
 * @param label 输出候选点的诊断标签。
 * @return ROI 内的候选特征点集合；曲线无效或半宽非法时返回空集合。
 * @note T 型、对接和搭接分支都通过该函数围绕历史参考 x 截取局部曲线。
 */
std::vector<WeldFeaturePoint> selectPointsInRoi(
    const ActiveContourCurve &curve,
    double centerXPixel,
    int halfWidthPixel,
    const std::string &label)
{
    std::vector<WeldFeaturePoint> roiPoints;
    if (!curve.valid || halfWidthPixel <= 0)
    {
        return roiPoints;
    }

    const double minX = centerXPixel - static_cast<double>(halfWidthPixel);
    const double maxX = centerXPixel + static_cast<double>(halfWidthPixel);
    for (const cv::Point2d &point : curve.curvePoints)
    {
        if (point.x < minX || point.x > maxX)
        {
            continue;
        }
        roiPoints.push_back(makeFeaturePoint(point, 1.0, label));
    }
    return roiPoints;
}

/**
 * @brief 对 ROI 点列按旧折线切分语义执行 RANSAC 分段拟合。
 * @param points ROI 内候选点，坐标单位为 pixel。
 * @param splitDistancePixel 折线切分距离阈值，单位为 pixel。
 * @param ransacDistancePixel RANSAC 点线距离阈值，单位为 pixel。
 * @param options 检测参数，提供 RANSAC 迭代次数和最小内点比例。
 * @return 拟合出的线段集合；点数不足或切分失败时返回空集合。
 * @note 该函数组合 `splitPolylineByMaxDeviation` 与 `fitSegmentsRansac`，
 * 保留旧折线分段再拟合的处理顺序。
 * @warning 输入点少于 3 个时无法形成稳定分段线。
 */
std::vector<WeldLineSegment> fitRoiLines(
    const std::vector<WeldFeaturePoint> &points,
    double splitDistancePixel,
    double ransacDistancePixel,
    const ActiveContourDetectionOptions &options)
{
    if (points.size() < 3)
    {
        return std::vector<WeldLineSegment>();
    }

    const WeldLineSplitResult splitResult =
        CLineGeometryTools::splitPolylineByMaxDeviation(
            points,
            splitDistancePixel);
    if (!splitResult.valid)
    {
        return std::vector<WeldLineSegment>();
    }

    const WeldLineSegmentsResult lineResult =
        CLineGeometryTools::fitSegmentsRansac(
            points,
            splitResult.splitIndices,
            options.ransacIterations,
            ransacDistancePixel,
            options.ransacMinInlierRatio);
    return lineResult.lines;
}

/**
 * @brief 提取相邻拟合线交点作为焊缝候选特征点。
 * @param lines 已拟合线段，坐标单位为 pixel。
 * @param confidence 赋给交点的置信度，范围为 0~1。
 * @return 有效相邻线交点集合；平行或无效线段会被跳过。
 * @note T 型和搭接分支使用交点表达折角或焊缝候选位置。
 */
std::vector<WeldFeaturePoint> extractIntersections(
    const std::vector<WeldLineSegment> &lines,
    double confidence)
{
    std::vector<WeldFeaturePoint> points;
    for (int index = 0; index + 1 < static_cast<int>(lines.size()); ++index)
    {
        const WeldLineIntersectionResult intersection =
            CLineGeometryTools::intersectLines(lines[index], lines[index + 1]);
        if (!intersection.valid)
        {
            continue;
        }
        points.push_back(makeFeaturePoint(
            intersection.point,
            confidence,
            "active_contour_feature_point"));
    }
    return points;
}

/**
 * @brief 计算当前曲线最接近参考列的局部极值列。
 * @param curvePoints 主动轮廓曲线采样点，坐标单位为 pixel。
 * @param referenceXPixel 历史参考列，单位为 pixel。
 * @return 最近局部极值的 x 坐标；没有极值时返回 `referenceXPixel`。
 * @note 复刻旧 `LocalMinMaxFunct1d` 的使用语义：先找 y(x) 的局部
 * min/max，再选择距离上一帧焊缝点最近的极值作为 T 型 ROI 中心。
 */
double findNearestExtremumX(
    const std::vector<cv::Point2d> &curvePoints,
    double referenceXPixel)
{
    if (curvePoints.size() < 3U)
    {
        return referenceXPixel;
    }

    std::vector<cv::Point2d> points = sortedUniquePoints(curvePoints);
    if (points.size() < 3U)
    {
        return referenceXPixel;
    }

    std::vector<double> extremaX;
    for (std::size_t index = 1U; index + 1U < points.size();)
    {
        std::size_t plateauEnd = index;
        while (plateauEnd + 1U < points.size()
               && std::fabs(points[plateauEnd + 1U].y - points[index].y)
                    <= kEpsilon)
        {
            ++plateauEnd;
        }

        const std::size_t leftIndex = index - 1U;
        const std::size_t rightIndex = plateauEnd + 1U;
        if (rightIndex >= points.size())
        {
            break;
        }

        const double centerY = points[index].y;
        const bool isLocalMin =
            centerY <= points[leftIndex].y && centerY <= points[rightIndex].y
            && (centerY < points[leftIndex].y || centerY < points[rightIndex].y);
        const bool isLocalMax =
            centerY >= points[leftIndex].y && centerY >= points[rightIndex].y
            && (centerY > points[leftIndex].y || centerY > points[rightIndex].y);
        if (isLocalMin || isLocalMax)
        {
            extremaX.push_back(
                0.5 * (points[index].x + points[plateauEnd].x));
        }
        index = plateauEnd + 1U;
    }

    if (extremaX.empty())
    {
        return referenceXPixel;
    }

    return *std::min_element(
        extremaX.begin(),
        extremaX.end(),
        [referenceXPixel](double left, double right) {
            return std::fabs(left - referenceXPixel)
                < std::fabs(right - referenceXPixel);
        });
}

/**
 * @brief 按旧对接分支构造 ROI 首尾与相邻拟合线交点序列。
 * @param roiPoints ROI 内曲线点，坐标单位为 pixel。
 * @param lines ROI 拟合线段集合。
 * @return 旧 `endpts` 等价点列：首点、相邻线交点、尾点。
 */
std::vector<cv::Point2d> buildLineEndpointCandidates(
    const std::vector<WeldFeaturePoint> &roiPoints,
    const std::vector<WeldLineSegment> &lines)
{
    std::vector<cv::Point2d> endpoints;
    if (roiPoints.empty())
    {
        return endpoints;
    }

    endpoints.push_back(cv::Point2d(
        roiPoints.front().xPixel,
        roiPoints.front().yPixel));
    for (int index = 0; index + 1 < static_cast<int>(lines.size()); ++index)
    {
        const WeldLineIntersectionResult intersection =
            CLineGeometryTools::intersectLines(lines[index], lines[index + 1]);
        if (intersection.valid)
        {
            endpoints.push_back(intersection.point);
        }
    }
    endpoints.push_back(cv::Point2d(
        roiPoints.back().xPixel,
        roiPoints.back().yPixel));
    return endpoints;
}

struct ButtGapScanAnchors {
    /// @brief 左侧 gap 扫描锚点列，单位 pixel。
    double leftGapX = 0.0;

    /// @brief 右侧 gap 扫描锚点列，单位 pixel。
    double rightGapX = 0.0;

    /// @brief true 表示锚点来自相邻拟合线交点；false 表示使用端点 margin fallback。
    bool fromLineEndpoints = false;
};

/**
 * @brief 按旧对接分支选择左右 gap 扫描锚点。
 * @param endpoints ROI 首尾与相邻拟合线交点序列，坐标单位为 pixel。
 * @param roiPoints ROI 内曲线点，坐标单位为 pixel。
 * @param options 对接 gap 默认偏移参数。
 * @return 左右 gap 扫描锚点；端点不足时退回首尾 margin。
 */
ButtGapScanAnchors chooseButtGapScanAnchors(
    const std::vector<cv::Point2d> &endpoints,
    const std::vector<WeldFeaturePoint> &roiPoints,
    const ActiveContourDetectionOptions &options)
{
    ButtGapScanAnchors anchors;
    if (endpoints.size() >= 5U)
    {
        anchors.leftGapX = endpoints[1U].x;
        anchors.rightGapX = endpoints[endpoints.size() - 2U].x;
        anchors.fromLineEndpoints = true;
        return anchors;
    }

    anchors.leftGapX = std::floor(roiPoints.front().xPixel)
        + static_cast<double>(options.buttGapEndpointMarginPixel);
    anchors.rightGapX = std::floor(roiPoints.back().xPixel)
        - static_cast<double>(options.buttGapEndpointMarginPixel);
    return anchors;
}

/**
 * @brief 计算线段在指定 x 坐标处的 y 坐标，单位为 pixel。
 * @param line 输入线段，坐标单位为 pixel。
 * @param xPixel 查询 x 坐标，单位为 pixel。
 * @return 线段模型在该 x 坐标处的 y 坐标，单位为 pixel。
 * @note 竖直线没有唯一斜率表达，返回线段起止 y 的平均值作为扫描参考。
 */
double lineYAt(const WeldLineSegment &line, double xPixel)
{
    if (line.vertical)
    {
        return 0.5 * (line.start.y + line.end.y);
    }

    return line.slope * xPixel + line.intercept;
}

/**
 * @brief 判断对接 gap 边缘附近是否符合中心暗、下方不连续的旧规则。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param xPixel 候选 x 坐标，单位为 pixel。
 * @param yPixel 候选 y 坐标，单位为 pixel。
 * @param brightThreshold 亮点判定阈值，范围为 0~255。
 * @return true 表示候选附近满足暗 gap 模式。
 * @note 该模式复刻旧工程对接 gap 扫描中“中心暗、下方亮点不足”的判断。
 * @warning 候选点靠近图像边界时直接返回 false，避免邻域访问越界。
 */
bool hasDarkGapPattern(
    const cv::Mat &gray8,
    int xPixel,
    int yPixel,
    int brightThreshold)
{
    if (xPixel <= 0 || yPixel <= 0
        || xPixel >= gray8.cols - 1 || yPixel >= gray8.rows - 1)
    {
        return false;
    }

    int brightNearCenter = 0;
    int brightLower = 0;
    const int centerOffsets[6][2] = {
        {0, 0}, {-1, 0}, {1, 0}, {1, -1}, {1, 1}, {0, -1}
    };
    const int lowerOffsets[2][2] = {
        {-1, 1}, {0, 1}
    };

    for (const auto &offset : centerOffsets)
    {
        if (grayAt(gray8, xPixel + offset[0], yPixel + offset[1])
            > brightThreshold)
        {
            ++brightNearCenter;
        }
    }
    for (const auto &offset : lowerOffsets)
    {
        if (grayAt(gray8, xPixel + offset[0], yPixel + offset[1])
            > brightThreshold)
        {
            ++brightLower;
        }
    }

    return brightNearCenter == 0 && brightLower < 2;
}

/**
 * @brief 检查 gap 候选点前方是否存在亮桥，避免把短暂暗斑误判为边缘。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param xPixel 候选 x 坐标，单位为 pixel。
 * @param yPixel 候选 y 坐标，单位为 pixel。
 * @param stepX 扫描方向，正数表示向右，负数表示向左。
 * @param brightThreshold 亮点判定阈值，范围为 0~255。
 * @return true 表示候选点前方 2~4 pixel 范围存在连续亮桥。
 * @note 亮桥表示暗斑可能只是局部噪声，不应作为 gap 边缘。
 */
bool hasBrightBridge(
    const cv::Mat &gray8,
    int xPixel,
    int yPixel,
    int stepX,
    int brightThreshold)
{
    int brightCount = 0;
    for (int offset = 2; offset <= 4; ++offset)
    {
        const int candidateX = xPixel + stepX * offset;
        if (grayAt(gray8, candidateX, yPixel) > brightThreshold
            || grayAt(gray8, candidateX, yPixel - 1) > brightThreshold
            || grayAt(gray8, candidateX, yPixel + 1) > brightThreshold)
        {
            ++brightCount;
        }
    }
    return brightCount > 2;
}

/**
 * @brief 沿拟合线扫描对接 gap 的单侧边缘点。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param line 用于预测扫描 y 坐标的拟合线段。
 * @param beginXPixel 起始 x 坐标，单位为 pixel。
 * @param endXPixel 结束 x 坐标，单位为 pixel。
 * @param stepX 扫描步长和方向，单位为 pixel。
 * @param brightThreshold 亮点判定阈值，范围为 0~255。
 * @param label 输出 gap 端点标签。
 * @return 找到的 gap 边缘点；未找到时返回 `valid=false` 的点。
 * @note 找到暗 gap 后会回退 `2 * stepX`，对应旧扫描中从暗区回到边缘前侧的语义。
 * @warning `stepX` 必须由调用方保证非 0，否则扫描循环没有有效推进。
 */
WeldFeaturePoint scanGapEdge(
    const cv::Mat &gray8,
    const WeldLineSegment &line,
    int beginXPixel,
    int endXPixel,
    int stepX,
    int brightThreshold,
    const std::string &label)
{
    for (int xPixel = beginXPixel;
         stepX > 0 ? xPixel < endXPixel : xPixel > endXPixel;
         xPixel += stepX)
    {
        const int yPixel = static_cast<int>(std::floor(lineYAt(line, xPixel)));
        if (!hasDarkGapPattern(gray8, xPixel, yPixel, brightThreshold))
        {
            continue;
        }

        if (hasBrightBridge(gray8, xPixel, yPixel, stepX, brightThreshold))
        {
            xPixel += 3 * stepX;
            continue;
        }

        const cv::Point2d point(
            static_cast<double>(xPixel - 2 * stepX),
            lineYAt(line, static_cast<double>(xPixel - 2 * stepX)));
        return makeFeaturePoint(point, 0.80, label);
    }

    WeldFeaturePoint point;
    point.label = label;
    return point;
}

/**
 * @brief 判断当前焊缝点附近是否仍有明显亮激光，用于触发历史备用。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param point 当前焊缝点，坐标单位为 pixel。
 * @param brightThreshold 亮点判定阈值，范围为 0~255。
 * @return true 表示点本身或上下 3 pixel 邻域仍有明显亮激光。
 * @note 对接 gap 点若仍处于亮激光区域，通常表示本帧定位不可信。
 */
bool isBrightAtWeldPoint(
    const cv::Mat &gray8,
    const WeldFeaturePoint &point,
    int brightThreshold)
{
    if (!point.valid)
    {
        return false;
    }

    const int xPixel = static_cast<int>(std::lround(point.xPixel));
    const int yPixel = static_cast<int>(std::lround(point.yPixel));
    int upperSum = 0;
    int lowerSum = 0;
    for (int offset = 1; offset <= 3; ++offset)
    {
        upperSum += grayAt(gray8, xPixel, yPixel - offset);
        lowerSum += grayAt(gray8, xPixel, yPixel + offset);
    }
    return grayAt(gray8, xPixel, yPixel) > brightThreshold
        || upperSum > brightThreshold
        || lowerSum > brightThreshold;
}

/**
 * @brief 判断历史焊缝点 x 坐标是否保持单调趋势。
 * @param history 最近焊缝点历史，坐标单位为 pixel。
 * @param positive true 表示检查单调不减，false 表示检查严格递减。
 * @return true 表示至少 6 帧历史满足指定方向的单调趋势。
 * @note 单调趋势用于历史预测时估计短时运动方向。
 */
bool hasMonotonicHistory(
    const std::deque<WeldFeaturePoint> &history,
    bool positive)
{
    if (history.size() < 6U)
    {
        return false;
    }

    for (std::size_t index = 1U; index < history.size(); ++index)
    {
        const double delta =
            history[index].xPixel - history[index - 1U].xPixel;
        if (positive && delta < 0.0)
        {
            return false;
        }
        if (!positive && delta >= 0.0)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 根据最近历史点预测短时备用焊缝点。
 * @param history 最近焊缝点历史，坐标单位为 pixel。
 * @return 预测焊缝点；历史为空时返回无效点。
 * @note 历史少于 6 帧时仅复用最后一个点；历史单调时按平均 x 增量外推一帧。
 * @warning 历史预测置信度最高限制为 0.55，只适合短时遮挡或跳变备用。
 */
WeldFeaturePoint predictFromHistory(
    const std::deque<WeldFeaturePoint> &history)
{
    if (history.empty())
    {
        return WeldFeaturePoint();
    }

    WeldFeaturePoint predicted = history.back();
    predicted.label = "active_contour_history_fallback";
    predicted.confidence = std::min(predicted.confidence, 0.55);

    if (history.size() < 6U)
    {
        return predicted;
    }

    if (!hasMonotonicHistory(history, true)
        && !hasMonotonicHistory(history, false))
    {
        return predicted;
    }

    double deltaSum = 0.0;
    for (std::size_t index = 1U; index < history.size(); ++index)
    {
        deltaSum += history[index].xPixel - history[index - 1U].xPixel;
    }
    predicted.xPixel += deltaSum / static_cast<double>(history.size() - 1U);
    return predicted;
}

/**
 * @brief 判断当前结果是否相对上一帧发生超限跳变。
 * @param state 跨帧状态，包含上一帧有效焊缝点和帧计数。
 * @param result 当前检测结果，坐标单位为 pixel。
 * @param options 跳变阈值和启用帧数参数。
 * @return true 表示当前结果超过对应焊缝类型的跳变阈值。
 * @note T 型立即启用跳变过滤；搭接在稳定期后才启用跳变过滤。
 */
bool shouldFallbackForJump(
    const ActiveContourDetectionState &state,
    const ActiveContourDetectionResult &result,
    const ActiveContourDetectionOptions &options)
{
    if (!state.lastWeldPoint.valid || !result.weldPoint.valid)
    {
        return false;
    }

    if (result.seamType == WeldSeamType::TJoint)
    {
        return std::fabs(result.weldPoint.xPixel
                         - state.lastWeldPoint.xPixel)
            > options.maxTJointJumpPixel;
    }

    if (result.seamType == WeldSeamType::Lap
        && state.frameCount > options.lapJumpFilterStartFrame)
    {
        return std::fabs(result.weldPoint.xPixel
                         - state.lastWeldPoint.xPixel)
            > options.maxLapJumpPixel;
    }

    return false;
}

/**
 * @brief 在点无效、点附近过亮或跳变过大时应用历史备用。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param options 历史备用和亮点判定参数。
 * @param state 跨帧状态，提供最近焊缝点历史。
 * @param result 当前检测结果，必要时会被历史预测点覆盖。
 * @note 该函数只在至少 6 帧历史存在时尝试备用，避免首帧阶段过早预测。
 * @warning `result=nullptr` 时直接返回；历史备用可能滞后真实焊缝位置。
 */
void applyHistoryFallbackIfNeeded(
    const cv::Mat &gray8,
    const ActiveContourDetectionOptions &options,
    const ActiveContourDetectionState &state,
    ActiveContourDetectionResult *result)
{
    if (result == nullptr || state.recentWeldPoints.size() < 6U)
    {
        return;
    }

    // 历史备用触发条件来自旧 active runtime 的跨帧稳定性需求：
    // 点无效、点附近仍是亮激光或相对上一帧跳变过大时，短时使用历史预测。
    const bool invalidPoint = !result->weldPoint.valid;
    const bool brightAtPoint = isBrightAtWeldPoint(
        gray8,
        result->weldPoint,
        options.gapBrightThresholdGrayValue);
    if (!invalidPoint && !brightAtPoint
        && !shouldFallbackForJump(state, *result, options))
    {
        return;
    }

    result->weldPoint = predictFromHistory(state.recentWeldPoints);
    result->valid = result->weldPoint.valid;
    result->confidence = result->valid
        ? result->weldPoint.confidence
        : result->confidence;
    result->usedHistoryFallback = result->valid;
    if (result->valid)
    {
        result->message = "active contour history fallback used";
    }
}

/**
 * @brief 将有效焊缝点写回跨帧状态并裁剪历史长度。
 * @param options 历史长度参数，单位为 frame。
 * @param result 当前检测结果，必须包含有效焊缝点才会写入。
 * @param state 待更新的跨帧状态。
 * @note `recentWeldPoints` 只保留最近 `historySize` 帧，供 ROI 参考和备用预测使用。
 * @warning `result` 或 `state` 为空、焊缝点无效时不会更新历史。
 */
void updateStateHistory(
    const ActiveContourDetectionOptions &options,
    ActiveContourDetectionResult *result,
    ActiveContourDetectionState *state)
{
    if (state == nullptr || result == nullptr || !result->weldPoint.valid)
    {
        return;
    }

    // recentWeldPoints 只保留最近 historySize 帧，单位为 pixel；
    // 过长历史会让遮挡恢复后的预测滞后。
    state->lastWeldPoint = result->weldPoint;
    state->recentWeldPoints.push_back(result->weldPoint);
    const int historySize = std::max(1, options.historySize);
    while (static_cast<int>(state->recentWeldPoints.size()) > historySize)
    {
        state->recentWeldPoints.pop_front();
    }
}

/**
 * @brief 优先使用上一帧焊缝点 x 坐标，否则使用图像中心作为 ROI 参考。
 * @param state 跨帧状态，可能包含上一帧有效焊缝点。
 * @param imageWidthPixel 图像宽度，单位为 pixel。
 * @return ROI 参考 x 坐标，单位为 pixel。
 * @note 首帧或历史无效时使用图像中心，下游帧使用上一帧焊缝点增强稳定性。
 */
double referenceXFromStateOrImage(
    const ActiveContourDetectionState &state,
    int imageWidthPixel)
{
    if (state.lastWeldPoint.valid)
    {
        return state.lastWeldPoint.xPixel;
    }
    return 0.5 * static_cast<double>(imageWidthPixel);
}

/**
 * @brief 在主动轮廓曲线 ROI 中检测 T 型焊缝交点。
 * @param curve 当前主动轮廓曲线，坐标单位为 pixel。
 * @param options T 型 ROI、折线切分、RANSAC 和置信度参数。
 * @param state 跨帧状态，用于确定 ROI 参考位置。
 * @param imageWidthPixel 图像宽度，单位为 pixel。
 * @return T 型焊缝检测结果；线段或交点不足时 `valid=false`。
 * @note T 型分支使用 ROI 内曲线分段线的交点作为焊缝点，并输出两侧 k/b 参数。
 * @warning ROI 点不足或 RANSAC 未得到两条有效线段时无法输出有效焊缝点。
 */
ActiveContourDetectionResult detectTJoint(
    const ActiveContourCurve &curve,
    const ActiveContourDetectionOptions &options,
    const ActiveContourDetectionState &state,
    int imageWidthPixel)
{
    ActiveContourDetectionResult result;
    result.seamType = WeldSeamType::TJoint;
    // 1. ROI 选择：先按旧 LocalMinMaxFunct1d 语义选取距离历史点最近的
    // 曲线极值作为中心候选；无极值时退回上一帧点或图像中心。
    const double referenceX = referenceXFromStateOrImage(state, imageWidthPixel);
    const double tCenterX = findNearestExtremumX(
        curve.curvePoints,
        referenceX);
    const int halfWidth = state.frameCount <= 1
        ? options.firstFrameRoiHalfWidthPixel
        : options.tJointRoiHalfWidthPixel;
    const std::vector<WeldFeaturePoint> roiPoints = selectPointsInRoi(
        curve,
        tCenterX,
        halfWidth,
        "active_contour_t_joint_point");
    result.fittedLines = fitRoiLines(
        roiPoints,
        options.tJointSplitDistancePixel,
        options.tJointRansacDistancePixel,
        options);
    if (result.fittedLines.size() < 2U)
    {
        result.message = "t joint active contour lines not found";
        return result;
    }

    // 2. T 型特征：旧流程依赖两侧线段交点作为焊缝点，
    // 这里选择最接近极值中心候选的交点以保持跨帧稳定。
    const WeldLineIntersectionResult intersection =
        CLineGeometryTools::intersectLinesNearReferenceX(
            result.fittedLines,
            tCenterX);
    if (!intersection.valid)
    {
        result.message = "t joint active contour intersection not found";
        return result;
    }

    // 3. 结果质量：T 型分支当前使用固定经验置信度 0.80，
    // slopeParameters 保留两侧直线 k/b 供旧语义兼容和诊断。
    const WeldLineSegment &leftLine = result.fittedLines.front();
    const WeldLineSegment &rightLine = result.fittedLines.back();
    result.slopeParameters.push_back(leftLine.slope);
    result.slopeParameters.push_back(leftLine.intercept);
    result.slopeParameters.push_back(rightLine.slope);
    result.slopeParameters.push_back(rightLine.intercept);
    result.weldPoint = makeFeaturePoint(
        intersection.point,
        0.80,
        "active_contour_t_joint_weld_point");
    result.valid = result.weldPoint.valid
        && result.weldPoint.confidence >= options.minConfidence;
    result.confidence = result.valid ? result.weldPoint.confidence : 0.0;
    result.message = result.valid
        ? "active contour t joint detected"
        : "active contour t joint confidence is low";
    return result;
}

/**
 * @brief 在主动轮廓曲线 ROI 中检测对接焊缝 gap 中点。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param curve 当前主动轮廓曲线，坐标单位为 pixel。
 * @param options 对接 ROI、拟合、gap 扫描和置信度参数。
 * @param state 跨帧状态，用于确定 ROI 参考位置。
 * @return 对接焊缝检测结果；gap 任一端点缺失时 `valid=false`。
 * @note 对接分支沿拟合线扫描左右 gap 边缘，并取两端点中点作为焊缝点。
 * @warning gapWidth 置信度是图像尺度下的相对指标，不代表真实世界间隙宽度。
 */
ActiveContourDetectionResult detectButt(
    const cv::Mat &gray8,
    const ActiveContourCurve &curve,
    const ActiveContourDetectionOptions &options,
    const ActiveContourDetectionState &state)
{
    ActiveContourDetectionResult result;
    result.seamType = WeldSeamType::Butt;
    // 1. ROI 选择与线段拟合：对接焊缝只在参考 x 附近分析主动轮廓点，
    // 避免远端噪声线段参与 gap 扫描。
    const double referenceX =
        referenceXFromStateOrImage(state, gray8.cols);
    const int halfWidth = state.frameCount <= 1
        ? options.firstFrameRoiHalfWidthPixel
        : options.buttRoiHalfWidthPixel;
    const std::vector<WeldFeaturePoint> roiPoints = selectPointsInRoi(
        curve,
        referenceX,
        halfWidth,
        "active_contour_butt_point");
    result.fittedLines = fitRoiLines(
        roiPoints,
        options.buttSplitDistancePixel,
        options.buttRansacDistancePixel,
        options);
    if (result.fittedLines.empty() || roiPoints.size() < 2U)
    {
        result.message = "butt active contour lines not found";
        return result;
    }

    // 2. gap 边缘扫描：从 ROI 两侧向中间搜索“中心暗、前方无亮桥”的断裂模式，
    // 复刻旧工程用暗 gap 定位对接缝边缘的语义。
    const WeldLineSegment &leftLine = result.fittedLines.front();
    const WeldLineSegment &rightLine = result.fittedLines.back();
    const double maxX = roiPoints.back().xPixel;
    const std::vector<cv::Point2d> endpoints =
        buildLineEndpointCandidates(roiPoints, result.fittedLines);
    const ButtGapScanAnchors scanAnchors =
        chooseButtGapScanAnchors(endpoints, roiPoints, options);
    const int leftScanBegin = static_cast<int>(std::floor(
        scanAnchors.leftGapX
        - static_cast<double>(options.buttGapScanPaddingPixel)));
    const int rightScanBegin = static_cast<int>(std::floor(
        scanAnchors.rightGapX
        + static_cast<double>(options.buttGapScanPaddingPixel)));
    result.leftGapPoint = scanGapEdge(
        gray8,
        leftLine,
        std::max(1, leftScanBegin),
        static_cast<int>(std::floor(maxX)) - 1,
        1,
        options.gapBrightThresholdGrayValue,
        "active_contour_left_gap");
    if (!result.leftGapPoint.valid)
    {
        result.message = "left butt gap edge not found";
        return result;
    }

    result.rightGapPoint = scanGapEdge(
        gray8,
        rightLine,
        std::min(gray8.cols - 2, rightScanBegin),
        static_cast<int>(std::ceil(result.leftGapPoint.xPixel)) + 1,
        -1,
        options.gapBrightThresholdGrayValue,
        "active_contour_right_gap");
    if (!result.rightGapPoint.valid)
    {
        result.message = "right butt gap edge not found";
        return result;
    }

    // 3. 焊缝点与置信度：对接焊缝点取左右 gap 端点中点；
    // gapWidth 越大说明断裂越明显，按图像宽度比例归一化到 0~1。
    const cv::Point2d weldPoint(
        0.5 * (result.leftGapPoint.xPixel + result.rightGapPoint.xPixel),
        0.5 * (result.leftGapPoint.yPixel + result.rightGapPoint.yPixel));
    const double gapWidth = std::fabs(
        result.rightGapPoint.xPixel - result.leftGapPoint.xPixel);
    result.weldPoint = makeFeaturePoint(
        weldPoint,
        clampRatio(gapWidth / std::max(1.0, 0.25 * gray8.cols)),
        "active_contour_butt_weld_point");
    result.featurePoints = extractIntersections(result.fittedLines, 0.70);
    result.valid = result.weldPoint.valid
        && result.weldPoint.confidence >= options.minConfidence;
    result.confidence = result.valid ? result.weldPoint.confidence : 0.0;
    result.message = result.valid
        ? "active contour butt seam detected"
        : "active contour butt seam confidence is low";
    return result;
}

/**
 * @brief 在主动轮廓曲线 ROI 中检测搭接焊缝最低交点。
 * @param curve 当前主动轮廓曲线，坐标单位为 pixel。
 * @param options 搭接 ROI、折线切分、RANSAC 和置信度参数。
 * @param state 跨帧状态，用于确定 ROI 参考位置。
 * @param imageWidthPixel 图像宽度，单位为 pixel。
 * @return 搭接焊缝检测结果；线段或交点不足时 `valid=false`。
 * @note 搭接分支取候选交点中 y 最大的点作为焊缝点，沿用图像坐标向下增大的语义。
 * @warning 多个反光折角可能产生候选交点，最终质量仍依赖 ROI 和历史跳变过滤。
 */
ActiveContourDetectionResult detectLap(
    const ActiveContourCurve &curve,
    const ActiveContourDetectionOptions &options,
    const ActiveContourDetectionState &state,
    int imageWidthPixel)
{
    ActiveContourDetectionResult result;
    result.seamType = WeldSeamType::Lap;
    // 1. ROI 选择与拟合：搭接分支同样围绕历史参考点截取曲线点，
    // 使用较宽切分阈值容忍搭接曲线的折线变化。
    const double referenceX = referenceXFromStateOrImage(state, imageWidthPixel);
    const int halfWidth = state.frameCount <= 1
        ? options.firstFrameRoiHalfWidthPixel
        : options.lapRoiHalfWidthPixel;
    const std::vector<WeldFeaturePoint> roiPoints = selectPointsInRoi(
        curve,
        referenceX,
        halfWidth,
        "active_contour_lap_point");
    result.fittedLines = fitRoiLines(
        roiPoints,
        options.lapSplitDistancePixel,
        options.lapRansacDistancePixel,
        options);
    if (result.fittedLines.size() < 2U)
    {
        result.message = "lap active contour lines not found";
        return result;
    }

    result.featurePoints = extractIntersections(result.fittedLines, 0.75);
    if (result.featurePoints.empty())
    {
        result.message = "lap active contour feature points not found";
        return result;
    }

    // 2. 搭接特征：旧语义中搭接焊缝点取候选交点里 y 最大的点，
    // 即图像坐标下更靠下的折角点。
    const auto maxRowIt = std::max_element(
        result.featurePoints.begin(),
        result.featurePoints.end(),
        [](const WeldFeaturePoint &left, const WeldFeaturePoint &right) {
            return left.yPixel < right.yPixel;
        });
    result.weldPoint = *maxRowIt;
    result.weldPoint.label = "active_contour_lap_weld_point";
    result.slopeParameters.push_back(result.fittedLines.back().slope);
    result.slopeParameters.push_back(result.fittedLines.back().intercept);
    result.valid = result.weldPoint.valid
        && result.weldPoint.confidence >= options.minConfidence;
    result.confidence = result.valid ? result.weldPoint.confidence : 0.0;
    result.message = result.valid
        ? "active contour lap seam detected"
        : "active contour lap seam confidence is low";
    return result;
}

/**
 * @brief 从多个候选分支中选择最高置信度的有效结果。
 * @param results T 型、对接、搭接等候选检测结果。
 * @return 最高置信度的有效结果；若全无效则返回首个候选保留诊断信息。
 * @note Automatic 模式使用该函数合并多个焊缝类型候选。
 */
ActiveContourDetectionResult chooseBestResult(
    const std::vector<ActiveContourDetectionResult> &results)
{
    ActiveContourDetectionResult bestResult;
    // Automatic 模式只比较 valid 候选的 confidence；若全失败，
    // 返回首个候选以保留诊断 message，便于调用方了解失败分支。
    for (const ActiveContourDetectionResult &result : results)
    {
        if (!result.valid)
        {
            continue;
        }
        if (!bestResult.valid || result.confidence > bestResult.confidence)
        {
            bestResult = result;
        }
    }
    if (!bestResult.valid && !results.empty())
    {
        bestResult = results.front();
    }
    return bestResult;
}

} // namespace

/**
 * @brief 基于主动轮廓曲线检测 T 型、对接或搭接焊缝点。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param mode 焊缝检测模式；Automatic 会比较多个候选分支。
 * @param options 检测、ROI、拟合、gap 扫描和历史备用参数。
 * @param state 调用方持有的跨帧状态；不得为 nullptr。
 * @return 主动轮廓焊缝检测结果；失败时 `valid=false`。
 * @note `state` 复用旧 active runtime 的跨帧语义，保存宽度、曲线、
 * 上一帧焊缝点和历史点，不包含 UI、设备或线程对象。
 * @warning 历史备用会降低置信度，只用于短时遮挡、反光或跳变场景。
 */
ActiveContourDetectionResult CActiveContourWeldFeatureDetector::detect(
    const cv::Mat &gray8,
    ActiveContourSeamMode mode,
    const ActiveContourDetectionOptions &options,
    ActiveContourDetectionState *state)
{
    ActiveContourDetectionResult result;
    // 1. 输入检查：state 由调用方按相机流生命周期持有，不能为空。
    if (state == nullptr)
    {
        result.message = "state is required";
        return result;
    }
    if (!isValidGrayImage(gray8))
    {
        result.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return result;
    }

    // 2. 条纹提取：Snake/BSnake 本体由 stripe 模块负责，本层只消费曲线。
    using StripeExtractor = CActiveContourStripeExtractor;
    const ActiveContourStripeResult stripeResult =
        StripeExtractor::extract(gray8, options, state);
    if (!stripeResult.valid)
    {
        result.activeCurve = stripeResult.activeCurve;
        result.message = stripeResult.message;
        return result;
    }
    const ActiveContourCurve &iteratedCurve = stripeResult.activeCurve;

    // 3. 分支检测：显式模式直接走指定焊缝类型，Automatic 同时计算多个候选。
    if (mode == ActiveContourSeamMode::TJoint)
    {
        result = detectTJoint(iteratedCurve, options, *state, gray8.cols);
    }
    else if (mode == ActiveContourSeamMode::Butt)
    {
        result = detectButt(gray8, iteratedCurve, options, *state);
    }
    else if (mode == ActiveContourSeamMode::Lap)
    {
        result = detectLap(iteratedCurve, options, *state, gray8.cols);
    }
    else
    {
        std::vector<ActiveContourDetectionResult> candidates;
        candidates.push_back(detectTJoint(
            iteratedCurve,
            options,
            *state,
            gray8.cols));
        candidates.push_back(detectButt(
            gray8,
            iteratedCurve,
            options,
            *state));
        candidates.push_back(detectLap(
            iteratedCurve,
            options,
            *state,
            gray8.cols));
        result = chooseBestResult(candidates);
    }

    result.activeCurve = iteratedCurve;
    // 5. 历史备用与状态更新：点无效、附近过亮或跳变过大时短时预测；
    // recentWeldPoints 保存的是 pixel 坐标历史，用于下一帧 ROI 和备用。
    applyHistoryFallbackIfNeeded(gray8, options, *state, &result);
    updateStateHistory(options, &result, state);
    return result;
}

} // namespace i2r_rlc::vision


