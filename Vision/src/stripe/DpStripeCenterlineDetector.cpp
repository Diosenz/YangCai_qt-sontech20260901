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
 * @file        DpStripeCenterlineDetector.cpp
 * @brief       动态规划激光中心线检测实现
 * @details     使用 OpenCV 完成列峰候选、亚像素细化和二阶 DP 路径选择，
 *              对应旧 GapDetector::extractLaserCenterline 的运行时语义。
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
#include <cstdint>
#include <limits>
#include <numeric>

#include <opencv2/imgproc.hpp>

#include "stripe/DpStripeCenterlineDetector.h"

namespace i2r_rlc::vision {
namespace {

/**
 * @brief DP 单列候选点，使用 y 像素坐标和局部响应分数描述中心线可能位置。
 */
struct DpStripeCandidate {
    /// @brief 候选中心行坐标，单位为 pixel，允许包含亚像素偏移。
    double yPixel = -1.0;
    /// @brief 候选局部响应得分，值越大表示该列峰值越可靠。
    double score = 0.0;
    /// @brief 是否为缺失列占位候选，用于保持 DP 状态连续。
    bool dummy = false;
};

/**
 * @brief 将一阶状态对映射到扁平数组下标。
 * @param previous 前一列候选索引。
 * @param current 当前列候选索引。
 * @param stateCount 每列候选状态数量。
 * @return DP 代价数组中的线性下标。
 */
int getStateIndex(int previous, int current, int stateCount)
{
    return previous * stateCount + current;
}

/**
 * @brief 将列号和状态对映射到回溯数组下标。
 * @param columnIndex 当前列在 ROI 内的序号。
 * @param previous 前一列候选索引。
 * @param current 当前列候选索引。
 * @param stateCount 每列候选状态数量。
 * @return 回溯数组中的线性下标。
 */
int getBackIndex(int columnIndex, int previous, int current, int stateCount)
{
    return (columnIndex * stateCount + previous) * stateCount + current;
}

/**
 * @brief 将浮点数夹紧到指定闭区间。
 * @param value 输入值。
 * @param minValue 最小允许值。
 * @param maxValue 最大允许值。
 * @return 夹紧后的值。
 */
double clampDouble(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

/**
 * @brief 计算带上限的绝对残差，避免方向先验异常值支配 DP 代价。
 * @param value 原始残差。
 * @param delta 最大惩罚幅度。
 * @return 不超过 delta 的绝对值。
 */
double clippedAbs(double value, double delta)
{
    const double absValue = std::abs(value);
    return absValue < delta ? absValue : delta;
}

/**
 * @brief 使用三点抛物线拟合估计列峰亚像素偏移。
 * @param left 上一行灰度或响应。
 * @param center 中心行灰度或响应。
 * @param right 下一行灰度或响应。
 * @return 限制在 [-0.5,0.5] pixel 的亚像素偏移。
 */
double subpixelParabola(double left, double center, double right)
{
    const double denominator = left - 2.0 * center + right;
    if (std::abs(denominator) < 1e-6) {
        return 0.0;
    }
    return clampDouble(0.5 * (left - right) / denominator, -0.5, 0.5);
}

/**
 * @brief 计算单列鲁棒灰度尺度，用分位数抑制孤立高亮噪声。
 * @param gray8 输入灰度图，必须为 CV_8UC1。
 * @param x 扫描列坐标，单位为 pixel。
 * @param yMin 扫描起始行，单位为 pixel。
 * @param yMax 扫描结束行，单位为 pixel。
 * @param quantile 分位数，范围通常为 [0,1]。
 * @return 指定列的参考灰度值。
 */
int robustColumnScale(
    const cv::Mat &gray8,
    int x,
    int yMin,
    int yMax,
    double quantile)
{
    int histogram[256] = {0};
    int total = 0;
    for (int y = yMin; y <= yMax; ++y) {
        ++histogram[gray8.ptr<uchar>(y)[x]];
        ++total;
    }
    if (total <= 0) {
        return 0;
    }

    const int target =
        static_cast<int>(std::floor(quantile * static_cast<double>(total - 1)));
    int cumulative = 0;
    for (int value = 0; value < 256; ++value) {
        cumulative += histogram[value];
        if (cumulative > target) {
            return value;
        }
    }
    return 255;
}

/**
 * @brief 计算单列 ROI 内最大灰度，作为分位数失效时的兜底尺度。
 * @param gray8 输入灰度图，必须为 CV_8UC1。
 * @param x 扫描列坐标，单位为 pixel。
 * @param yMin 扫描起始行，单位为 pixel。
 * @param yMax 扫描结束行，单位为 pixel。
 * @return 最大灰度值。
 */
int columnMaximum(
    const cv::Mat &gray8,
    int x,
    int yMin,
    int yMax)
{
    int maximum = 0;
    for (int y = yMin; y <= yMax; ++y) {
        maximum = std::max(maximum, static_cast<int>(gray8.ptr<uchar>(y)[x]));
    }
    return maximum;
}

/**
 * @brief 按 y 方向最小间距抑制同列重复候选。
 * @param sortedCandidates 已按 score 降序排列的候选列表。
 * @param minSeparationPixel 最小 y 方向间距，单位为 pixel。
 * @param maxKeep 最多保留候选数量。
 * @return 抑制后的候选列表，仍保持得分优先顺序。
 */
std::vector<DpStripeCandidate> suppressByYDistance(
    const std::vector<DpStripeCandidate> &sortedCandidates,
    double minSeparationPixel,
    int maxKeep)
{
    std::vector<DpStripeCandidate> kept;
    kept.reserve(static_cast<size_t>(std::max(0, maxKeep)));
    for (const auto &candidate : sortedCandidates) {
        bool farEnough = true;
        for (const auto &keptCandidate : kept) {
            if (std::abs(candidate.yPixel - keptCandidate.yPixel) <
                minSeparationPixel) {
                farEnough = false;
                break;
            }
        }
        if (farEnough) {
            kept.push_back(candidate);
            if (static_cast<int>(kept.size()) >= maxKeep) {
                break;
            }
        }
    }
    return kept;
}

/**
 * @brief 从单列灰度剖面中提取局部峰候选。
 * @param gray8 输入灰度图，必须为 CV_8UC1。
 * @param x 扫描列坐标，单位为 pixel。
 * @param yMin ROI 起始行，单位为 pixel。
 * @param yMax ROI 结束行，单位为 pixel。
 * @param options DP 条纹检测参数。
 * @return 当前列按可靠性筛选后的候选点集合。
 */
std::vector<DpStripeCandidate> extractCandidatesOneColumn(
    const cv::Mat &gray8,
    int x,
    int yMin,
    int yMax,
    const DpStripeDetectionOptions &options)
{
    std::vector<DpStripeCandidate> candidates;
    candidates.reserve(16);

    int referenceGray = robustColumnScale(gray8, x, yMin, yMax, 0.90);
    if (referenceGray <= 0) {
        referenceGray = columnMaximum(gray8, x, yMin, yMax);
    }
    if (referenceGray <= 0) {
        return candidates;
    }

    const int threshold = static_cast<int>(
        std::floor(options.adaptiveThresholdRatio * referenceGray));
    const double prominenceThreshold = std::max(
        options.prominenceAbsMinGray,
        options.prominenceRatio * static_cast<double>(referenceGray));

    for (int y = yMin + 1; y <= yMax - 1; ++y) {
        const double upper = gray8.ptr<uchar>(y - 1)[x];
        const double center = gray8.ptr<uchar>(y)[x];
        const double lower = gray8.ptr<uchar>(y + 1)[x];

        if (center < threshold) {
            continue;
        }
        if (!(center >= upper && center >= lower)) {
            continue;
        }

        const double prominence = center - std::min(upper, lower);
        if (prominence < prominenceThreshold ||
            prominence < options.minProminenceGray) {
            continue;
        }

        DpStripeCandidate candidate;
        candidate.yPixel = static_cast<double>(y) +
            subpixelParabola(upper, center, lower);
        candidate.score = prominence / (static_cast<double>(referenceGray) + 1.0);
        candidate.dummy = false;
        candidates.push_back(candidate);
    }

    if (candidates.empty()) {
        return candidates;
    }

    const auto byScore = [](const DpStripeCandidate &left,
                            const DpStripeCandidate &right) {
        return left.score > right.score;
    };

    const int maxCandidates = std::max(1, options.maxCandidatesPerColumn);
    const int preKeep = std::min(
        static_cast<int>(candidates.size()),
        std::max(8, 3 * maxCandidates));
    if (static_cast<int>(candidates.size()) > preKeep) {
        std::nth_element(
            candidates.begin(), candidates.begin() + preKeep, candidates.end(),
            byScore);
        candidates.resize(static_cast<size_t>(preKeep));
    }

    std::sort(candidates.begin(), candidates.end(), byScore);
    candidates = suppressByYDistance(
        candidates, options.candidateMinSeparationPixel, maxCandidates);
    if (static_cast<int>(candidates.size()) > maxCandidates) {
        candidates.resize(static_cast<size_t>(maxCandidates));
    }
    return candidates;
}

/**
 * @brief 计算单个候选的观测代价。
 * @param candidate 候选点或缺失占位。
 * @param options 代价权重和缺失惩罚参数。
 * @return 越小越优的 unary 代价。
 */
double unaryCost(
    const DpStripeCandidate &candidate,
    const DpStripeDetectionOptions &options)
{
    if (candidate.dummy) {
        return options.missingPenalty;
    }
    return -options.intensityWeight * candidate.score;
}

/**
 * @brief 计算相邻两列之间的一阶平滑和方向先验代价。
 * @param previous 前一列候选。
 * @param current 当前列候选。
 * @param options DP 平滑与先验参数。
 * @param columnIndex 当前列在 ROI 内的序号。
 * @return 越小越优的一阶转移代价；不允许跳变时返回 infinity。
 */
double firstOrderCost(
    const DpStripeCandidate &previous,
    const DpStripeCandidate &current,
    const DpStripeDetectionOptions &options,
    int columnIndex)
{
    if (previous.dummy || current.dummy) {
        return options.missingPenalty;
    }

    const double dy = current.yPixel - previous.yPixel;
    if (std::abs(dy) > options.maxJumpPixel) {
        return std::numeric_limits<double>::infinity();
    }

    double cost = options.slopeWeight * std::abs(dy);
    if (options.useDirectionPrior &&
        columnIndex >= 0 &&
        columnIndex < static_cast<int>(options.slopePriorByColumn.size())) {
        const double weight = options.priorWeightByColumn.empty()
            ? 1.0
            : options.priorWeightByColumn[static_cast<size_t>(columnIndex)];
        const double residual =
            dy - options.slopePriorByColumn[static_cast<size_t>(columnIndex)];
        cost += options.directionPriorWeight * weight *
            clippedAbs(residual, options.directionDeltaPixel);
    }
    return cost;
}

/**
 * @brief 计算三列窗口上的二阶曲率代价。
 * @param twoColumnsBefore 前两列候选。
 * @param previous 前一列候选。
 * @param current 当前列候选。
 * @param options DP 曲率、跳变和方向先验参数。
 * @param columnIndex 当前列在 ROI 内的序号。
 * @return 越小越优的二阶转移代价；非法跳变返回 infinity。
 */
double secondOrderCost(
    const DpStripeCandidate &twoColumnsBefore,
    const DpStripeCandidate &previous,
    const DpStripeCandidate &current,
    const DpStripeDetectionOptions &options,
    int columnIndex)
{
    if (twoColumnsBefore.dummy || previous.dummy || current.dummy) {
        return options.missingPenalty;
    }

    const double dy1 = previous.yPixel - twoColumnsBefore.yPixel;
    const double dy2 = current.yPixel - previous.yPixel;
    if (std::abs(dy2) > options.maxJumpPixel) {
        return std::numeric_limits<double>::infinity();
    }

    double curvatureWeight = options.curvatureWeight;
    if (options.seamXPixel >= 0 &&
        std::abs(columnIndex - options.seamXPixel) <= options.seamBandPixel) {
        curvatureWeight *= 0.2;
    }

    if (options.relaxCurvatureByPrior &&
        options.useDirectionPrior &&
        columnIndex > 0 &&
        columnIndex < static_cast<int>(options.slopePriorByColumn.size())) {
        const double slopeDelta = std::abs(
            options.slopePriorByColumn[static_cast<size_t>(columnIndex)] -
            options.slopePriorByColumn[static_cast<size_t>(columnIndex - 1)]);
        if (slopeDelta > options.cornerSlopeThreshold) {
            curvatureWeight *= options.cornerCurvatureScale;
        }
    }

    double cost = options.slopeWeight * std::abs(dy2) +
        curvatureWeight * std::abs(dy2 - dy1);
    if (options.useDirectionPrior &&
        columnIndex >= 0 &&
        columnIndex < static_cast<int>(options.slopePriorByColumn.size())) {
        const double expectedSlope =
            options.slopePriorByColumn[static_cast<size_t>(columnIndex)];
        const double weight = options.priorWeightByColumn.empty()
            ? 1.0
            : options.priorWeightByColumn[static_cast<size_t>(columnIndex)];
        const double actualSlope =
            (current.yPixel - twoColumnsBefore.yPixel) / 2.0;
        cost += options.directionPriorWeight * weight *
            clippedAbs(actualSlope - expectedSlope, options.directionDeltaPixel);
    }
    return cost;
}

/**
 * @brief 将请求 ROI 规范化并裁剪到图像范围内。
 * @param gray8 输入灰度图。
 * @param requestedRoi 请求处理区域，空或非正尺寸表示整图。
 * @return 与图像相交后的 ROI。
 */
cv::Rect normalizedRoi(const cv::Mat &gray8, const cv::Rect &requestedRoi)
{
    cv::Rect roi = requestedRoi;
    if (roi.width <= 0 || roi.height <= 0) {
        roi = cv::Rect(0, 0, gray8.cols, gray8.rows);
    }
    return roi & cv::Rect(0, 0, gray8.cols, gray8.rows);
}

/**
 * @brief 统计中心线有效点之间的大列间隔比例。
 * @param points 中心线点集合，x/y 单位为 pixel。
 * @param maxColumnGap 超过该列间隔视为断裂，单位为 pixel。
 * @return 断裂间隔数量占有效相邻间隔数量的比例。
 */
double calculateBrokenStripeRatio(
    const std::vector<LaserCenterlinePoint> &points,
    double maxColumnGap)
{
    if (points.size() < 2U) {
        return 0.0;
    }

    int validGapCount = 0;
    int brokenGapCount = 0;
    const LaserCenterlinePoint *previous = nullptr;
    for (const auto &point : points) {
        if (!point.valid) {
            continue;
        }
        if (previous != nullptr) {
            ++validGapCount;
            if (point.xPixel - previous->xPixel > maxColumnGap) {
                ++brokenGapCount;
            }
        }
        previous = &point;
    }

    if (validGapCount <= 0) {
        return 0.0;
    }
    return static_cast<double>(brokenGapCount) /
        static_cast<double>(validGapCount);
}

/**
 * @brief 将旧 StripeCenterlineResult 适配为统一 LaserCenterlineResult。
 * @param gray8 原始灰度图，用于恢复 ROI。
 * @param options DP 检测参数。
 * @param stripeResult DP 输出的中心线结果。
 * @return 带 confidence、断裂率和统一点结构的中心线结果。
 */
LaserCenterlineResult toLaserCenterlineResult(
    const cv::Mat &gray8,
    const DpStripeDetectionOptions &options,
    const StripeCenterlineResult &stripeResult)
{
    LaserCenterlineResult result;
    result.valid = stripeResult.valid;
    result.roi = gray8.empty()
        ? cv::Rect()
        : normalizedRoi(gray8, options.roi);
    result.processingTimeMs = stripeResult.processingTimeMs;
    result.message = stripeResult.message;
    result.validColumnRatio = stripeResult.qualityScore;

    result.points.reserve(stripeResult.centerlinePoints.size());
    double confidenceSum = 0.0;
    int validPointCount = 0;
    for (const cv::Point2d &point : stripeResult.centerlinePoints) {
        LaserCenterlinePoint centerlinePoint;
        centerlinePoint.xPixel = point.x;
        centerlinePoint.yPixel = point.y;

        double response = 0.0;
        const int responseIndex =
            static_cast<int>(std::round(point.x)) - result.roi.x;
        if (responseIndex >= 0 &&
            responseIndex < static_cast<int>(
                stripeResult.responseByColumn.size())) {
            response =
                stripeResult.responseByColumn[static_cast<size_t>(responseIndex)];
        }
        centerlinePoint.response = response;
        centerlinePoint.localContrast = response;
        centerlinePoint.confidence = clampDouble(response, 0.0, 1.0);
        if (centerlinePoint.confidence <= 0.0) {
            centerlinePoint.confidence =
                clampDouble(stripeResult.qualityScore, 0.0, 1.0);
        }
        centerlinePoint.valid = true;
        result.points.push_back(centerlinePoint);
        confidenceSum += centerlinePoint.confidence;
        ++validPointCount;
    }

    result.meanConfidence = validPointCount > 0
        ? confidenceSum / static_cast<double>(validPointCount)
        : 0.0;
    result.brokenStripeRatio = calculateBrokenStripeRatio(result.points, 1.5);
    return result;
}

} // namespace

StripeCenterlineResult CDpStripeCenterlineDetector::detect(
    const cv::Mat &gray8,
    const DpStripeDetectionOptions &options)
{
    const auto startTime = std::chrono::steady_clock::now();
    StripeCenterlineResult result;

    if (gray8.empty() || gray8.type() != CV_8UC1) {
        result.message = "输入图像必须是非空 CV_8UC1";
        return result;
    }

    const cv::Rect roi = normalizedRoi(gray8, options.roi);
    if (roi.width <= 2 || roi.height <= 2) {
        result.message = "DP 条纹 ROI 过小";
        return result;
    }

    int yMin = options.yMinPixel < 0 ? 0 : options.yMinPixel;
    int yMax = options.yMaxPixel < 0 ? roi.height - 1 : options.yMaxPixel;
    yMin = std::max(0, std::min(yMin, roi.height - 1));
    yMax = std::max(0, std::min(yMax, roi.height - 1));
    if (yMax - yMin < 2) {
        result.message = "DP 条纹 y 范围过小";
        return result;
    }

    cv::Mat roiImage = gray8(roi).clone();
    if (options.blurSigma > 0.0) {
        cv::GaussianBlur(
            roiImage, roiImage, cv::Size(0, 0), options.blurSigma,
            options.blurSigma, cv::BORDER_REPLICATE);
    }

    const int columnCount = roi.width;
    const int candidateCount = std::max(1, options.maxCandidatesPerColumn) + 1;
    std::vector<DpStripeCandidate> candidates(
        static_cast<size_t>(columnCount * candidateCount));
    std::vector<double> unary(
        static_cast<size_t>(columnCount * candidateCount), 0.0);

    for (int x = 0; x < columnCount; ++x) {
        candidates[static_cast<size_t>(x * candidateCount)] =
            DpStripeCandidate{-1.0, 0.0, true};

        const auto columnCandidates =
            extractCandidatesOneColumn(roiImage, x, yMin, yMax, options);
        int fill = 1;
        for (const auto &candidate : columnCandidates) {
            if (fill >= candidateCount) {
                break;
            }
            candidates[static_cast<size_t>(x * candidateCount + fill)] =
                candidate;
            ++fill;
        }
        for (; fill < candidateCount; ++fill) {
            candidates[static_cast<size_t>(x * candidateCount + fill)] =
                DpStripeCandidate{-1.0, 0.0, true};
        }
    }

    for (int x = 0; x < columnCount; ++x) {
        for (int i = 0; i < candidateCount; ++i) {
            unary[static_cast<size_t>(x * candidateCount + i)] =
                unaryCost(
                    candidates[static_cast<size_t>(x * candidateCount + i)],
                    options);
        }
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> dp0(static_cast<size_t>(candidateCount), infinity);
    for (int i = 0; i < candidateCount; ++i) {
        dp0[static_cast<size_t>(i)] = unary[static_cast<size_t>(i)];
    }

    std::vector<double> dpPrevious(
        static_cast<size_t>(candidateCount * candidateCount), infinity);
    std::vector<double> dpCurrent(
        static_cast<size_t>(candidateCount * candidateCount), infinity);
    std::vector<std::uint8_t> back;
    if (columnCount >= 3) {
        back.assign(
            static_cast<size_t>((columnCount - 2) * candidateCount * candidateCount),
            0);
    }

    if (columnCount >= 2) {
        for (int previous = 0; previous < candidateCount; ++previous) {
            for (int current = 0; current < candidateCount; ++current) {
                const auto &left =
                    candidates[static_cast<size_t>(previous)];
                const auto &right =
                    candidates[static_cast<size_t>(candidateCount + current)];
                dpPrevious[static_cast<size_t>(
                    getStateIndex(previous, current, candidateCount))] =
                    dp0[static_cast<size_t>(previous)] +
                    unary[static_cast<size_t>(candidateCount + current)] +
                    firstOrderCost(left, right, options, 1);
            }
        }
    }

    for (int x = 2; x < columnCount; ++x) {
        std::fill(dpCurrent.begin(), dpCurrent.end(), infinity);
        for (int previous = 0; previous < candidateCount; ++previous) {
            for (int current = 0; current < candidateCount; ++current) {
                double bestCost = infinity;
                int bestBefore = 0;
                for (int before = 0; before < candidateCount; ++before) {
                    const double previousCost =
                        dpPrevious[static_cast<size_t>(
                            getStateIndex(before, previous, candidateCount))];
                    if (!std::isfinite(previousCost)) {
                        continue;
                    }

                    const auto &candidateBefore = candidates[static_cast<size_t>(
                        (x - 2) * candidateCount + before)];
                    const auto &candidatePrevious = candidates[static_cast<size_t>(
                        (x - 1) * candidateCount + previous)];
                    const auto &candidateCurrent = candidates[static_cast<size_t>(
                        x * candidateCount + current)];
                    const double transition = secondOrderCost(
                        candidateBefore, candidatePrevious, candidateCurrent,
                        options, x);
                    if (!std::isfinite(transition)) {
                        continue;
                    }

                    const double total = previousCost +
                        unary[static_cast<size_t>(x * candidateCount + current)] +
                        transition;
                    if (total < bestCost) {
                        bestCost = total;
                        bestBefore = before;
                    }
                }

                dpCurrent[static_cast<size_t>(
                    getStateIndex(previous, current, candidateCount))] =
                    bestCost;
                back[static_cast<size_t>(
                    getBackIndex(x - 2, previous, current, candidateCount))] =
                    static_cast<std::uint8_t>(bestBefore);
            }
        }
        dpPrevious.swap(dpCurrent);
    }

    std::vector<int> chosen(static_cast<size_t>(columnCount), 0);
    if (columnCount == 1) {
        chosen[0] = static_cast<int>(
            std::min_element(dp0.begin(), dp0.end()) - dp0.begin());
    } else {
        int bestPrevious = 0;
        int bestCurrent = 0;
        double bestCost = infinity;
        for (int previous = 0; previous < candidateCount; ++previous) {
            for (int current = 0; current < candidateCount; ++current) {
                const double cost = dpPrevious[static_cast<size_t>(
                    getStateIndex(previous, current, candidateCount))];
                if (cost < bestCost) {
                    bestCost = cost;
                    bestPrevious = previous;
                    bestCurrent = current;
                }
            }
        }

        chosen[static_cast<size_t>(columnCount - 2)] = bestPrevious;
        chosen[static_cast<size_t>(columnCount - 1)] = bestCurrent;
        for (int x = columnCount - 1; x >= 2; --x) {
            const int current = chosen[static_cast<size_t>(x)];
            const int previous = chosen[static_cast<size_t>(x - 1)];
            chosen[static_cast<size_t>(x - 2)] = back[static_cast<size_t>(
                getBackIndex(x - 2, previous, current, candidateCount))];
        }
    }

    result.centerlinePoints.reserve(static_cast<size_t>(columnCount));
    result.responseByColumn.assign(static_cast<size_t>(columnCount), 0.0);
    for (int x = 0; x < columnCount; ++x) {
        const int candidateIndex = chosen[static_cast<size_t>(x)];
        const auto &candidate = candidates[static_cast<size_t>(
            x * candidateCount + candidateIndex)];
        if (candidate.dummy) {
            continue;
        }
        const cv::Point2d point(
            static_cast<double>(roi.x + x),
            static_cast<double>(roi.y) + candidate.yPixel);
        result.centerlinePoints.push_back(point);
        result.responseByColumn[static_cast<size_t>(x)] = candidate.score;
    }

    result.valid =
        static_cast<int>(result.centerlinePoints.size()) >=
        std::max(1, options.minValidPointCount);
    result.qualityScore = columnCount > 0
        ? static_cast<double>(result.centerlinePoints.size()) /
            static_cast<double>(columnCount)
        : 0.0;
    result.message = result.valid
        ? "DP 条纹中心线检测成功"
        : "DP 条纹有效点不足";

    const auto endTime = std::chrono::steady_clock::now();
    result.processingTimeMs =
        std::chrono::duration<double, std::milli>(endTime - startTime).count();
    return result;
}

LaserCenterlineResult CDpStripeCenterlineDetector::detectCenterline(
    const cv::Mat &gray8,
    const DpStripeDetectionOptions &options)
{
    const StripeCenterlineResult stripeResult = detect(gray8, options);
    return toLaserCenterlineResult(gray8, options, stripeResult);
}

} // namespace i2r_rlc::vision
