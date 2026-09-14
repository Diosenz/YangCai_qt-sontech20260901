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
 * @file        ContourControlPointTools.cpp
 * @brief       主动轮廓控制点处理工具
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
#include <exception>

#include <tinysplinecxx.h>

#include "feature/ContourControlPointTools.h"
#include "stripe/LaserLineResponseTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 检查输入是否为算法层唯一接受的灰度图格式。
 * @param grayImage 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 * @note 控制点工具只处理灰度图，不在算法层执行格式转换。
 */
bool isSupportedGrayImage(const cv::Mat &grayImage)
{
    return !grayImage.empty() && grayImage.type() == CV_8UC1;
}

/**
 * @brief 写入可选预处理状态，避免调用方必须传入状态指针。
 * @param status 可选输出状态指针，可以为 nullptr。
 * @param value 要写入的状态值。
 * @note 该 helper 用于保持 public API 中 `status` 参数的空指针容忍语义。
 */
void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

/**
 * @brief 判断整数像素坐标是否位于图像范围内。
 * @param grayImage 用于提供宽高边界的输入图像。
 * @param xPixel x 坐标，单位为 pixel。
 * @param yPixel y 坐标，单位为 pixel。
 * @return true 表示坐标位于 `[0, cols)` 与 `[0, rows)` 范围内。
 */
bool isInsideImage(const cv::Mat &grayImage, int xPixel, int yPixel)
{
    return xPixel >= 0
        && xPixel < grayImage.cols
        && yPixel >= 0
        && yPixel < grayImage.rows;
}

/**
 * @brief 检查指定像素是否在图像内且灰度达到阈值。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param xPixel x 坐标，单位为 pixel。
 * @param yPixel y 坐标，单位为 pixel。
 * @param minGrayValue 最小灰度阈值，范围为 0~255。
 * @return true 表示像素在图像内且灰度不低于阈值。
 * @note 该函数先做边界检查，避免调用方在局部搜索时重复判断越界。
 */
bool isBrightEnough(
    const cv::Mat &grayImage,
    int xPixel,
    int yPixel,
    int minGrayValue)
{
    return isInsideImage(grayImage, xPixel, yPixel)
        && grayImage.at<unsigned char>(yPixel, xPixel) >= minGrayValue;
}

/**
 * @brief 按 x/y 排序控制点，并按旧工程同列去重语义删除重复列。
 * @param points 输入控制点，坐标单位为 pixel。
 * @return 按 x 升序排列且同一 x 列只保留一个点的控制点集合。
 * @note 对应旧 `BSnake_HandleConPt` 中 `TupleSortIndex` 排序和同列删除语义。
 */
std::vector<cv::Point2d> sortedUniquePoints(
    const std::vector<cv::Point2d> &points)
{
    // 旧 BSnake_HandleConPt 会通过 TupleSortIndex 按列排序，并删除同列
    // 重复控制点。这里用 x/y 双关键字排序，保留点列从左到右的曲线顺序。
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
                return std::fabs(left.x - right.x) <= 1.0e-6;
            }),
        sortedPoints.end());
    return sortedPoints;
}

/**
 * @brief 将 sub-pixel 点四舍五入为整数像素点。
 * @param point 输入 sub-pixel 点，坐标单位为 pixel。
 * @return 四舍五入后的整数像素坐标。
 * @note 旧工程的拖拽和灰度判断使用整数像素访问，曲线点进入这些步骤前需要量化。
 */
cv::Point2i roundedPoint(const cv::Point2d &point)
{
    return cv::Point2i(
        static_cast<int>(std::lround(point.x)),
        static_cast<int>(std::lround(point.y)));
}

/**
 * @brief 复刻旧 DragOfflinePt：在同列附近搜索亮激光线并返回拖拽点。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param laserWidthByRow 每一行激光宽度估计，单位为 pixel。
 * @param startPoint 离线控制点的整数像素位置，单位为 pixel。
 * @param options 拖拽搜索半高和候选灰度阈值等参数。
 * @param draggedPoint 输出拖回激光线后的点，坐标单位为 pixel。
 * @return true 表示成功找到并验证亮激光候选点。
 * @note 该函数委托 `CLaserLineResponseTools::dragPointToLaserLine` 执行同列搜索。
 * @warning `draggedPoint=nullptr`、起点越界或拖拽结果亮度不足时返回 false。
 */
bool tryDragOfflinePoint(
    const cv::Mat &grayImage,
    const std::vector<int> &laserWidthByRow,
    const cv::Point2i &startPoint,
    const ContourControlPointOptions &options,
    cv::Point2d *draggedPoint)
{
    if (draggedPoint == nullptr
        || !isInsideImage(grayImage, startPoint.x, startPoint.y))
    {
        return false;
    }

    VisionPreprocessStatus dragStatus = VisionPreprocessStatus::Succeeded;
    const LaserDraggedPointResult dragged =
        CLaserLineResponseTools::dragPointToLaserLine(
            grayImage,
            laserWidthByRow,
            startPoint,
            options.dragSearchHalfHeightPixel,
            options.dragMinGrayValue,
            &dragStatus);
    if (dragStatus != VisionPreprocessStatus::Succeeded || !dragged.valid)
    {
        return false;
    }

    const cv::Point2i draggedPixel = roundedPoint(dragged.point);
    if (!isBrightEnough(
            grayImage,
            draggedPixel.x,
            draggedPixel.y,
            options.dragMinGrayValue))
    {
        return false;
    }

    *draggedPoint = dragged.point;
    return true;
}

/**
 * @brief 扫描临时曲线暗点，并把可拖回激光线的点追加为修正控制点。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param laserWidthByRow 每一行激光宽度估计，单位为 pixel。
 * @param temporaryCurve 当前控制点生成的临时曲线。
 * @param options 曲线离线阈值、拖拽范围和扫描数量限制。
 * @param controlPoints 待追加修正点的控制点集合，坐标单位为 pixel。
 * @note 对应旧 `BSnake_HandleConPt` 中扫描临时 NURBS/XLD 曲线暗点并补控制点的语义。
 * @warning `controlPoints=nullptr`、临时曲线无效或曲线点过多时直接跳过扫描。
 */
void appendCurveOfflineCorrections(
    const cv::Mat &grayImage,
    const std::vector<int> &laserWidthByRow,
    const ControlPointCurveResult &temporaryCurve,
    const ContourControlPointOptions &options,
    std::vector<cv::Point2d> *controlPoints)
{
    if (controlPoints == nullptr
        || !temporaryCurve.valid
        || options.curveScanMaxPointCount <= 0
        || static_cast<int>(temporaryCurve.curvePoints.size())
            >= options.curveScanMaxPointCount)
    {
        return;
    }

    for (const cv::Point2d &curvePoint : temporaryCurve.curvePoints)
    {
        const cv::Point2i point = roundedPoint(curvePoint);
        if (!isInsideImage(grayImage, point.x, point.y)
            || isBrightEnough(
                grayImage,
                point.x,
                point.y,
                options.curveOfflineMinGrayValue))
        {
            continue;
        }

        cv::Point2d draggedPoint;
        if (tryDragOfflinePoint(
                grayImage,
                laserWidthByRow,
                point,
                options,
                &draggedPoint))
        {
            controlPoints->push_back(draggedPoint);
        }
    }
}

/**
 * @brief 对控制点之间超过阈值的横向间距补充中间控制点。
 * @param curvePoints 临时曲线采样点，坐标单位为 pixel。
 * @param maxGapPixel 允许的最大相邻控制点横向间距，单位为 pixel。
 * @param controlPoints 待扩展的控制点集合，坐标单位为 pixel。
 * @note 优先使用间隔内曲线点的平均坐标补点；没有曲线点时退化为线性中点。
 * @warning `controlPoints=nullptr` 或 `maxGapPixel<=0` 时不会修改输入集合。
 */
void insertWideGapMidpoints(
    const std::vector<cv::Point2d> &curvePoints,
    int maxGapPixel,
    std::vector<cv::Point2d> *controlPoints)
{
    if (controlPoints == nullptr || maxGapPixel <= 0)
    {
        return;
    }

    const std::vector<cv::Point2d> sortedPoints =
        sortedUniquePoints(*controlPoints);
    if (sortedPoints.size() < 2U)
    {
        *controlPoints = sortedPoints;
        return;
    }

    std::vector<cv::Point2d> expandedPoints;
    expandedPoints.reserve(sortedPoints.size() * 2U);
    for (std::size_t index = 0U; index + 1U < sortedPoints.size(); ++index)
    {
        const cv::Point2d &left = sortedPoints[index];
        const cv::Point2d &right = sortedPoints[index + 1U];
        expandedPoints.push_back(left);
        if (right.x - left.x <= static_cast<double>(maxGapPixel))
        {
            continue;
        }

        double xSum = 0.0;
        double ySum = 0.0;
        int pointCount = 0;
        for (const cv::Point2d &curvePoint : curvePoints)
        {
            if (curvePoint.x <= left.x || curvePoint.x >= right.x)
            {
                continue;
            }
            xSum += curvePoint.x;
            ySum += curvePoint.y;
            ++pointCount;
        }

        expandedPoints.push_back(
            pointCount > 0
                ? cv::Point2d(
                    xSum / static_cast<double>(pointCount),
                    ySum / static_cast<double>(pointCount))
                : cv::Point2d(
                    0.5 * (left.x + right.x),
                    0.5 * (left.y + right.y)));
    }
    expandedPoints.push_back(sortedPoints.back());
    *controlPoints = sortedUniquePoints(expandedPoints);
}

/**
 * @brief 两个控制点时执行 HALCON NURBS 兼容的直线采样。
 * @param firstPoint 起点，坐标单位为 pixel。
 * @param lastPoint 终点，坐标单位为 pixel。
 * @param samplesPerSegment 线段采样数量，单位为 point，必须大于 0。
 * @return 首尾闭合包含的直线采样点，坐标单位为 pixel。
 * @note tinyspline 三次插值需要更多控制点，两个点时按旧工程等价语义退化为直线。
 */
std::vector<cv::Point2d> sampleLinearCurve(
    const cv::Point2d &firstPoint,
    const cv::Point2d &lastPoint,
    int samplesPerSegment)
{
    // tinyspline 至少需要足够控制点才能生成三次曲线。两个控制点时，
    // HALCON NURBS 在工程上等价于首尾之间的直线采样，因此单独处理。
    std::vector<cv::Point2d> curvePoints;
    curvePoints.reserve(static_cast<std::size_t>(samplesPerSegment) + 1U);
    for (int sample = 0; sample <= samplesPerSegment; ++sample)
    {
        const double ratio = static_cast<double>(sample)
            / static_cast<double>(samplesPerSegment);
        curvePoints.push_back(
            firstPoint + (lastPoint - firstPoint) * ratio);
    }
    return curvePoints;
}

/**
 * @brief 使用 tinyspline 对三个及以上控制点生成二维 B-spline 采样点。
 * @param controlPoints 输入控制点，坐标单位为 pixel。
 * @param samplesPerSegment 每段采样数量，单位为 point，必须大于 0。
 * @return tinyspline 采样得到的曲线点，坐标单位为 pixel。
 * @note 该函数是旧 HALCON `GenContourNurbsXld / GenNurbsInterp` 的内部替代实现。
 * @warning tinyspline 异常由调用方捕获并转换为无效插值结果。
 */
std::vector<cv::Point2d> sampleTinysplineCurve(
    const std::vector<cv::Point2d> &controlPoints,
    int samplesPerSegment)
{
    std::vector<tinyspline::real> flattenedPoints;
    flattenedPoints.reserve(controlPoints.size() * 2U);
    for (const cv::Point2d &point : controlPoints)
    {
        flattenedPoints.push_back(static_cast<tinyspline::real>(point.x));
        flattenedPoints.push_back(static_cast<tinyspline::real>(point.y));
    }

    /*
     * 旧工程通过 HALCON `GenContourNurbsXld / GenNurbsInterp` 从控制点
     * 得到 XLD 采样轮廓。这里使用 tinyspline 的二维自然三次 B-spline
     * 插值来保留“控制点 -> NURBS/B-spline 曲线采样点”的核心语义，
     * 同时避免 HALCON 类型进入 vision 公共接口。
     */
    const tinyspline::BSpline spline =
        tinyspline::BSpline::interpolateCubicNatural(flattenedPoints, 2U);

    // 旧 HALCON XLD 返回一串轮廓采样点，不暴露参数 t。新实现只需要
    // 下游视觉算法可消费的点列，因此按控制点段数给出稳定采样数量。
    const std::size_t sampleCount =
        1U + (controlPoints.size() - 1U)
            * static_cast<std::size_t>(samplesPerSegment);
    const std::vector<tinyspline::real> sampledValues =
        spline.sample(sampleCount);

    std::vector<cv::Point2d> curvePoints;
    curvePoints.reserve(sampledValues.size() / 2U);
    for (std::size_t index = 0; index + 1U < sampledValues.size(); index += 2U)
    {
        curvePoints.push_back(cv::Point2d(
            static_cast<double>(sampledValues[index]),
            static_cast<double>(sampledValues[index + 1U])));
    }
    return curvePoints;
}

} // namespace

/**
 * @brief 按旧 `BSnakeGrow` 的左右三邻域策略增长控制点。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param startPoint 增长起点，单位为 pixel。
 * @param direction 增长方向，向左或向右。
 * @param options 增长阈值和采样步长。
 * @param status 可选输出状态。
 * @return 增长得到的控制点；未增长出点时 `valid=false`。
 * @note 该函数只执行局部三邻域跟踪，不做图像读取、格式转换或全局路径搜索。
 * @warning 起点越界、空图、非灰度图或采样步长非法时返回无效结果。
 */
ControlPointGrowResult CContourControlPointTools::growLaserControlPoints(
    const cv::Mat &grayImage,
    const cv::Point2i &startPoint,
    ControlPointGrowDirection direction,
    const ContourControlPointOptions &options,
    VisionPreprocessStatus *status)
{
    ControlPointGrowResult result;
    // 1. 输入检查：控制点增长只接收 CV_8UC1，避免算法层混入格式转换。
    if (!isSupportedGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input image"
            : "input image must be CV_8UC1";
        setStatus(
            status,
            grayImage.empty()
                ? VisionPreprocessStatus::EmptyInput
                : VisionPreprocessStatus::InvalidGrayImage);
        return result;
    }
    if (!isInsideImage(grayImage, startPoint.x, startPoint.y)
        || options.growSampleStepPixel <= 0)
    {
        result.message = "invalid control point grow options";
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    int currentX = startPoint.x;
    int currentY = startPoint.y;
    int searchLength = 0;
    const int stepX =
        direction == ControlPointGrowDirection::Left ? -1 : 1;

    // 2. 三邻域跟踪：复刻旧 BSnakeGrow 的当前行、上一行、下一行顺序，
    // 只在下一列局部寻找亮点，保持旧工程对连续激光线的偏好。
    for (;;)
    {
        const int candidateX = currentX + stepX;
        if (candidateX <= 0 || candidateX >= grayImage.cols - 1)
        {
            break;
        }

        bool moved = false;
        const int nextRows[3] = {currentY, currentY - 1, currentY + 1};
        // BSnakeGrow 不是全局路径搜索，它只看下一列的三个候选行。
        // 顺序保留旧实现偏好：先保持当前行，再尝试上一行和下一行。
        for (int rowIndex = 0; rowIndex < 3; ++rowIndex)
        {
            const int candidateY = nextRows[rowIndex];
            if (isBrightEnough(
                    grayImage,
                    candidateX,
                    candidateY,
                    options.growMinGrayValue))
            {
                currentX = candidateX;
                currentY = candidateY;
                moved = true;
                break;
            }
        }
        if (!moved)
        {
            break;
        }

        ++searchLength;
        if (searchLength == options.growSampleStepPixel)
        {
            // 旧函数每走 5 个像素采样一个控制点，避免控制点过密导致
            // 下游 NURBS/XLD 曲线抖动；步长在 options 中保留可调。
            result.points.push_back(cv::Point2d(currentX, currentY));
            searchLength = 0;
        }
    }

    if (direction == ControlPointGrowDirection::Left)
    {
        // 3. 输出顺序归一：左向增长先从右往左得到点，反转后保持 x 升序。
        std::reverse(result.points.begin(), result.points.end());
    }
    // 4. 状态汇总：valid 只表示至少采到一个控制点，坐标单位仍为 pixel。
    result.valid = !result.points.empty();
    result.message = result.valid
        ? "control points grown"
        : "no control point grown";
    setStatus(
        status,
        result.valid
            ? VisionPreprocessStatus::Succeeded
            : VisionPreprocessStatus::InsufficientPoints);
    return result;
}

/**
 * @brief 修正离线控制点并输出插值曲线。
 * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
 * @param laserWidthByRow 每一行激光宽度估计，单位为 pixel。
 * @param controlPoints 初始控制点，单位为 pixel。
 * @param options 离线判定、拖拽、补点、增长和插值参数。
 * @param status 可选输出状态。
 * @return 修正后的控制点和曲线采样点；输入不足或插值失败时 `valid=false`。
 * @note 该函数承接旧 `BSnake_HandleConPt` 的离线拖拽、暗点补点、
 * 大间距补点、左右 BSnakeGrow 和 NURBS/XLD 曲线替代语义。
 * @warning 离线点拖拽失败会删除控制点，最终控制点少于 2 个时无法生成曲线。
 */
ControlPointCurveResult CContourControlPointTools::adjustOfflineControlPoints(
    const cv::Mat &grayImage,
    const std::vector<int> &laserWidthByRow,
    const std::vector<cv::Point2d> &controlPoints,
    const ContourControlPointOptions &options,
    VisionPreprocessStatus *status)
{
    ControlPointCurveResult result;
    // 1. 输入检查：宽度数组必须覆盖图像行数，拖拽时需要逐行宽度作为搜索尺度。
    if (!isSupportedGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input image"
            : "input image must be CV_8UC1";
        setStatus(
            status,
            grayImage.empty()
                ? VisionPreprocessStatus::EmptyInput
                : VisionPreprocessStatus::InvalidGrayImage);
        return result;
    }
    if (controlPoints.size() < 2
        || static_cast<int>(laserWidthByRow.size()) < grayImage.rows)
    {
        result.message = "not enough control points";
        setStatus(status, VisionPreprocessStatus::InsufficientPoints);
        return result;
    }

    std::vector<cv::Point2d> adjustedPoints;
    adjustedPoints.reserve(controlPoints.size());
    // 2. 离线控制点修正：暗点先尝试 DragOfflinePt 等价拖拽，
    // 拖拽失败则删除，避免明显错误点污染下游曲线。
    for (const cv::Point2d &point : controlPoints)
    {
        const cv::Point2i pixel = roundedPoint(point);
        const int xPixel = pixel.x;
        const int yPixel = pixel.y;
        if (!isInsideImage(grayImage, xPixel, yPixel))
        {
            continue;
        }

        cv::Point2d adjustedPoint = point;
        if (grayImage.at<unsigned char>(yPixel, xPixel)
            < options.offlineMinGrayValue)
        {
            // 旧 BSnake_HandleConPt 会把离线控制点拖回激光线；若找不到
            // 合法亮点，则删除该控制点，避免暗点继续污染 NURBS。
            if (!tryDragOfflinePoint(
                    grayImage,
                    laserWidthByRow,
                    pixel,
                    options,
                    &adjustedPoint))
            {
                continue;
            }
        }
        adjustedPoints.push_back(adjustedPoint);
    }

    adjustedPoints = sortedUniquePoints(adjustedPoints);
    if (adjustedPoints.size() < 2U)
    {
        result.message = "not enough adjusted control points";
        setStatus(status, VisionPreprocessStatus::InsufficientPoints);
        return result;
    }

    VisionPreprocessStatus interpolationStatus =
        VisionPreprocessStatus::Succeeded;
    // 3. 临时曲线生成：先用当前控制点得到曲线，供下游扫描曲线暗点使用。
    ControlPointCurveResult temporaryCurve = interpolateControlCurve(
        adjustedPoints,
        options.interpolationSamplesPerSegment,
        &interpolationStatus);
    appendCurveOfflineCorrections(
        grayImage,
        laserWidthByRow,
        temporaryCurve,
        options,
        &adjustedPoints);

    adjustedPoints = sortedUniquePoints(adjustedPoints);
    if (temporaryCurve.valid)
    {
        // 4. 大间距补点：旧 BSnake_HandleConPt 会对超过 30 pixel 的控制点间隔补点，
        // 这里优先使用临时曲线内的平均位置，缺少曲线点时使用线性中点。
        insertWideGapMidpoints(
            temporaryCurve.curvePoints,
            options.maxControlPointGapPixel,
            &adjustedPoints);
    }

    if (adjustedPoints.size() >= 2)
    {
        const cv::Point2i firstPoint(
            static_cast<int>(std::lround(adjustedPoints.front().x)),
            static_cast<int>(std::lround(adjustedPoints.front().y)));
        const cv::Point2i lastPoint(
            static_cast<int>(std::lround(adjustedPoints.back().x)),
            static_cast<int>(std::lround(adjustedPoints.back().y)));

        // 5. 端部增长：复刻旧流程在左右端点继续调用 BSnakeGrow，
        // 让控制点覆盖更完整的激光曲线范围。
        const ControlPointGrowResult leftGrow = growLaserControlPoints(
            grayImage,
            firstPoint,
            ControlPointGrowDirection::Left,
            options,
            nullptr);
        const ControlPointGrowResult rightGrow = growLaserControlPoints(
            grayImage,
            lastPoint,
            ControlPointGrowDirection::Right,
            options,
            nullptr);

        // 旧函数会在控制点两端继续调用 BSnakeGrow 扩展曲线端部。这里将
        // 左端增长点插入前方，右端增长点追加到末尾，再统一排序去重。
        adjustedPoints.insert(
            adjustedPoints.begin(),
            leftGrow.points.begin(),
            leftGrow.points.end());
        adjustedPoints.insert(
            adjustedPoints.end(),
            rightGrow.points.begin(),
            rightGrow.points.end());
        adjustedPoints = sortedUniquePoints(adjustedPoints);
    }

    // 6. 最终插值：用修正、补点和端部增长后的控制点输出 tinyspline 曲线点列。
    result = interpolateControlCurve(
        adjustedPoints,
        options.interpolationSamplesPerSegment,
        status);
    if (result.valid)
    {
        result.message = "offline control points adjusted";
    }
    return result;
}

/**
 * @brief 对控制点执行 tinyspline NURBS/B-spline 曲线采样。
 * @param controlPoints 输入控制点，单位为 pixel。
 * @param samplesPerSegment 每段采样数量，单位为 point，必须大于 0。
 * @param status 可选输出状态。
 * @return 排序去重后的控制点和曲线采样点；失败时 `valid=false`。
 * @note 两个控制点退化为直线采样，三个及以上控制点使用 tinyspline
 * 自然三次 B-spline，替代旧 HALCON `GenContourNurbsXld / GenNurbsInterp`。
 * @warning 控制点不足、采样数量非法或 tinyspline 抛出异常时返回无效结果。
 */
ControlPointCurveResult CContourControlPointTools::interpolateControlCurve(
    const std::vector<cv::Point2d> &controlPoints,
    int samplesPerSegment,
    VisionPreprocessStatus *status)
{
    ControlPointCurveResult result;
    // 1. 输入检查：至少两个唯一控制点才能定义曲线或直线采样。
    if (samplesPerSegment <= 0 || controlPoints.size() < 2)
    {
        result.message = "invalid interpolation input";
        setStatus(status, VisionPreprocessStatus::InsufficientPoints);
        return result;
    }

    result.controlPoints = sortedUniquePoints(controlPoints);
    if (result.controlPoints.size() < 2)
    {
        result.message = "not enough unique control points";
        setStatus(status, VisionPreprocessStatus::InsufficientPoints);
        return result;
    }

    try
    {
        // 2. 曲线采样：两个点使用线性 fallback，三个及以上点进入 tinyspline。
        if (result.controlPoints.size() == 2)
        {
            result.curvePoints = sampleLinearCurve(
                result.controlPoints.front(),
                result.controlPoints.back(),
                samplesPerSegment);
        }
        else
        {
            // 三个及以上控制点使用 tinyspline 自然三次 B-spline 插值。
            // 这是替代 HALCON NURBS/XLD 的实现点，public API 仍只返回点列。
            result.curvePoints = sampleTinysplineCurve(
                result.controlPoints,
                samplesPerSegment);
        }
    }
    catch (const std::exception &)
    {
        result.message = "tinyspline interpolation failed";
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    if (result.curvePoints.empty())
    {
        result.message = "tinyspline interpolation produced no points";
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    // 3. 结果语义：curvePoints 是下游视觉算法消费的 pixel 点列，
    // 不承诺与 HALCON XLD 采样点逐点一致。
    result.valid = true;
    result.message = result.controlPoints.size() == 2
        ? "linear NURBS-compatible control curve interpolated"
        : "tinyspline NURBS control curve interpolated";
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return result;
}

} // namespace i2r_rlc::vision

