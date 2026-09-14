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
 * @file        IsffaStripeDetector.cpp
 * @brief       ISFFA 激光条纹检测实现
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
#include <chrono>

#include "stripe/IsffaStripeDetector.h"
#include "stripe/StripeLineRepairTools.h"
#include "stripe/StripeResponseEvaluationTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 判断输入是否满足 ISFFA 检测器的灰度图约束。
 * @param gray8 输入图像，要求非空 CV_8UC1。
 * @return 输入可被列中心线提取流程处理时返回 true。
 */
bool isSupportedGray(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 将质量分数夹紧到 [0,1]。
 * @param value 原始分数。
 * @return 可作为 confidence/ratio 使用的归一化值。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 更新跨帧检测状态，保留上一帧有效中心线用于缺口修复。
 * @param result 本帧 ISFFA 检测结果。
 * @param state 可选状态对象，生命周期由调用方持有。
 */
void updateState(
    const StripeCenterlineResult &result,
    StripeDetectionState *state)
{
    if (state == nullptr)
    {
        return;
    }
    if (result.valid)
    {
        state->lastValidCenterlinePoints = result.centerlinePoints;
        state->consecutiveFailureCount = 0;
    }
    else
    {
        ++state->consecutiveFailureCount;
    }
}

} // namespace

StripeCenterlineResult CIsffaStripeDetector::detect(
    const cv::Mat &gray8,
    const StripeDetectionOptions &options,
    StripeDetectionState *state)
{
    const auto startTime = std::chrono::steady_clock::now();
    StripeCenterlineResult result;
    if (!isSupportedGray(gray8))
    {
        result.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        updateState(result, state);
        return result;
    }

    // 1. FindROI/LOGandISFFA 初始点：ISFFA 旧路径更强调 ROI 内分段，
    // 这里先用列灰度重心形成稳定候选，再围绕候选估计 ROI。
    std::vector<double> responseByColumn;
    std::vector<cv::Point2d> points =
        CStripeLineRepairTools::extractThresholdColumnCenters(
            gray8,
            options,
            &responseByColumn);
    if (static_cast<int>(points.size()) < options.minTotalPointCount)
    {
        result.responseByColumn = responseByColumn;
        result.message = "ISFFA initial centerline has insufficient points";
        result.processingTimeMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - startTime).count();
        updateState(result, state);
        return result;
    }

    // 2. ISFFALink：连接短缺口；旧代码按 SplitLines 补线，新实现按相邻
    // 列距和线性插值补齐，保留补点用于调试和 replay。
    std::vector<cv::Point2d> repairedPoints;
    points = CStripeLineRepairTools::repairLinearGaps(
        points,
        options,
        &repairedPoints);

    // 3. ISFFA 分段拟合：分段线越稳定，RDA 分数越高。
    result.segments = CStripeLineRepairTools::fitSegments(points, options);
    double segmentScoreSum = 0.0;
    int validSegmentCount = 0;
    for (const StripeSegment &segment : result.segments)
    {
        if (!segment.valid)
        {
            continue;
        }
        ++validSegmentCount;
        segmentScoreSum +=
            CStripeResponseEvaluationTools::calculateResidualDistributionAgreement(
                segment.points,
                segment.fittedLine,
                std::max(2.0, options.splitDistancePixel));
    }

    // 4. 输出统一结果。ISFFA 相比 SAM 更看重分段拟合质量。
    result.centerlinePoints = points;
    result.repairedPoints = repairedPoints;
    result.responseByColumn = responseByColumn;
    const double coverage = clampRatio(
        static_cast<double>(points.size()) / static_cast<double>(gray8.cols));
    const double rdaScore = validSegmentCount > 0
        ? clampRatio(segmentScoreSum / static_cast<double>(validSegmentCount))
        : 0.0;
    result.qualityScore = clampRatio(0.35 * coverage + 0.65 * rdaScore);
    result.valid = validSegmentCount > 0
        && static_cast<int>(points.size()) >= options.minTotalPointCount;
    result.message = result.valid
        ? "ISFFA stripe centerline detected"
        : "ISFFA stripe centerline has no valid fitted segment";

    const auto stopTime = std::chrono::steady_clock::now();
    result.processingTimeMs =
        std::chrono::duration<double, std::milli>(stopTime - startTime).count();
    updateState(result, state);
    return result;
}

} // namespace i2r_rlc::vision
