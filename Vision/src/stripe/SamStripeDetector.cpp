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
 * @file        SamStripeDetector.cpp
 * @brief       SAM 激光条纹检测实现
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

#include "stripe/LaserLineResponseTools.h"
#include "stripe/SamStripeDetector.h"
#include "stripe/StripeLineRepairTools.h"
#include "stripe/StripeResponseEvaluationTools.h"

namespace i2r_rlc::vision {

namespace {

double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

bool isSupportedGray(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

std::vector<cv::Point2d> extractLogOrThresholdCenters(
    const cv::Mat &gray8,
    const StripeDetectionOptions &options,
    std::vector<double> *responseByColumn)
{
    if (responseByColumn != nullptr)
    {
        responseByColumn->assign(gray8.cols, 0.0);
    }

    // 1. LOG 列峰：对应旧 LOGandSAM 的第一步。旧代码只对亮点列做
    // LOGCore 响应，新实现先尝试 LOG/CoM，点数不足再退回灰度重心。
    std::vector<int> widthByRow(
        gray8.rows,
        std::max(1, options.defaultLaserWidthPixel));
    LaserLogPeakOptions logOptions;
    logOptions.rowMinPixel = options.useRoi ? options.roi.y : 0;
    logOptions.rowMaxPixel = options.useRoi
        ? options.roi.y + options.roi.height
        : gray8.rows;
    logOptions.columnStepPixel = std::max(1, options.logColumnStepPixel);
    logOptions.logThreshold = options.logThreshold;
    logOptions.requiredPeakGrayValue = 255;

    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const LaserLogPeakResult logResult =
        CLaserLineResponseTools::detectLogPeaks(
            gray8,
            widthByRow,
            logOptions,
            &status);
    std::vector<cv::Point2d> logPoints;
    if (logResult.valid)
    {
        const LaserCenterOfMassResult comResult =
            CLaserLineResponseTools::refineCenterOfMass(
                gray8,
                logResult.maxRowByColumn,
                widthByRow,
                std::max(2, options.defaultLaserWidthPixel * 2),
                &status);
        for (int x = 0; x < gray8.cols; ++x)
        {
            const double row = comResult.valid
                ? comResult.refinedRowByColumn[static_cast<std::size_t>(x)]
                : static_cast<double>(logResult.maxRowByColumn[static_cast<std::size_t>(x)]);
            if (row >= 0.0)
            {
                logPoints.emplace_back(static_cast<double>(x), row);
            }
            if (responseByColumn != nullptr
                && static_cast<int>(logResult.maxResponseByColumn.size()) > x)
            {
                (*responseByColumn)[static_cast<std::size_t>(x)] =
                    std::max(0.0, logResult.maxResponseByColumn[static_cast<std::size_t>(x)]);
            }
        }
    }

    if (static_cast<int>(logPoints.size()) >= options.minTotalPointCount)
    {
        return logPoints;
    }

    // 2. 灰度列重心备用：旧 SAM 路径面对 LOG 不稳定图像也会进入补线
    // 和合并逻辑，这里用阈值重心保证 type=0 入口可稳定输出候选。
    return CStripeLineRepairTools::extractThresholdColumnCenters(
        gray8,
        options,
        responseByColumn);
}

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

StripeCenterlineResult CSamStripeDetector::detect(
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

    // 1. LOG/CoM 或灰度列重心提取中心线候选。
    std::vector<double> responseByColumn;
    std::vector<cv::Point2d> points =
        extractLogOrThresholdCenters(gray8, options, &responseByColumn);

    // 2. SAMRemoveNFill：短缺口线性补点。
    std::vector<cv::Point2d> repairedPoints;
    points = CStripeLineRepairTools::repairLinearGaps(
        points,
        options,
        &repairedPoints);

    // 3. SAMMerge：对局部抖动做轻量平滑，减少伪分裂。
    points = CStripeLineRepairTools::smoothCenterline(
        points,
        options.smoothWindowRadius);

    // 4. SAMSplit + RANSAC：输出中心线分段。
    result.segments = CStripeLineRepairTools::fitSegments(points, options);
    result.centerlinePoints = points;
    result.repairedPoints = repairedPoints;
    result.responseByColumn = responseByColumn;

    int validSegmentCount = 0;
    double segmentConfidenceSum = 0.0;
    for (const StripeSegment &segment : result.segments)
    {
        if (segment.valid)
        {
            ++validSegmentCount;
            segmentConfidenceSum += segment.confidence;
        }
    }

    const double coverage = gray8.cols > 0
        ? clampRatio(static_cast<double>(points.size()) / static_cast<double>(gray8.cols))
        : 0.0;
    const double responseScore =
        CStripeResponseEvaluationTools::calculateMeanResponseScore(
            responseByColumn,
            255.0);
    const double segmentScore = validSegmentCount > 0
        ? clampRatio(segmentConfidenceSum / static_cast<double>(validSegmentCount))
        : 0.0;
    result.qualityScore = clampRatio(
        0.40 * coverage + 0.30 * responseScore + 0.30 * segmentScore);
    result.valid =
        static_cast<int>(points.size()) >= options.minTotalPointCount
        && validSegmentCount > 0;
    result.message = result.valid
        ? "SAM stripe centerline detected"
        : "SAM stripe centerline has insufficient valid segments";

    const auto stopTime = std::chrono::steady_clock::now();
    result.processingTimeMs =
        std::chrono::duration<double, std::milli>(stopTime - startTime).count();
    updateState(result, state);
    return result;
}

} // namespace i2r_rlc::vision

