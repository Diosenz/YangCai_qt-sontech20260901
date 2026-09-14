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
 * @file        StegerStripeDetector.cpp
 * @brief       Steger 风格条纹检测实现
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
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "stripe/StegerStripeDetector.h"
#include "stripe/StripeLineRepairTools.h"
#include "stripe/StripeResponseEvaluationTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 判断输入是否满足 Steger 检测器的灰度图约束。
 * @param gray8 输入图像，要求非空 CV_8UC1。
 * @return 输入可被按列扫描时返回 true。
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
 * @brief 根据配置裁剪 Steger 扫描 ROI。
 * @param gray8 输入灰度图。
 * @param options 检测参数，useRoi 为 false 时使用整图。
 * @return 位于图像内部的 ROI，单位为 pixel。
 */
cv::Rect normalizedRoi(const cv::Mat &gray8, const StripeDetectionOptions &options)
{
    const cv::Rect imageRect(0, 0, gray8.cols, gray8.rows);
    return options.useRoi ? (options.roi & imageRect) : imageRect;
}

/**
 * @brief 用三点抛物线估计 ridge 峰值的亚像素行偏移。
 * @param left 上一行灰度。
 * @param center 当前行灰度。
 * @param right 下一行灰度。
 * @return 限制在 [-0.5,0.5] pixel 的偏移量。
 */
double parabolicOffset(double left, double center, double right)
{
    const double denominator = left - 2.0 * center + right;
    if (std::abs(denominator) < 1.0e-6)
    {
        return 0.0;
    }
    return std::max(-0.5, std::min(0.5, 0.5 * (left - right) / denominator));
}

/**
 * @brief 更新跨帧检测状态，保留上一帧有效中心线用于后续修复。
 * @param result 本帧 Steger 检测结果。
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

StripeCenterlineResult CStegerStripeDetector::detect(
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

    // 1. 高斯平滑 + Hessian 二阶响应。旧 Steger 依赖 HALCON lines_gauss，
    // 这里以 OpenCV 二阶导数近似亮 ridge 的法向响应。
    cv::Mat gray64;
    gray8.convertTo(gray64, CV_64F);
    cv::Mat blurred;
    const double sigma = std::max(0.1, options.stegerSigma);
    cv::GaussianBlur(gray64, blurred, cv::Size(), sigma, sigma);

    cv::Mat dyy;
    cv::Sobel(blurred, dyy, CV_64F, 0, 2, 3);

    const cv::Rect roi = normalizedRoi(gray8, options);
    result.responseByColumn.assign(gray8.cols, 0.0);
    std::vector<cv::Point2d> points;
    for (int x = roi.x; x < roi.x + roi.width; ++x)
    {
        double bestScore = 0.0;
        int bestRow = -1;
        for (int y = std::max(roi.y + 1, 1);
             y < std::min(roi.y + roi.height - 1, gray8.rows - 1);
             ++y)
        {
            const double gray = static_cast<double>(
                gray8.at<unsigned char>(y, x));
            if (gray < options.thresholdGrayValue)
            {
                continue;
            }

            // 亮线在行方向上通常是局部极大值；-dyy 越大，ridge 越明显。
            const double ridgeResponse = -dyy.at<double>(y, x);
            if (ridgeResponse < options.stegerMinRidgeResponse)
            {
                continue;
            }
            if (gray8.at<unsigned char>(y, x)
                < gray8.at<unsigned char>(y - 1, x)
                || gray8.at<unsigned char>(y, x)
                    < gray8.at<unsigned char>(y + 1, x))
            {
                continue;
            }

            const double score = ridgeResponse + gray / 255.0;
            if (score > bestScore)
            {
                bestScore = score;
                bestRow = y;
            }
        }

        result.responseByColumn[static_cast<std::size_t>(x)] = bestScore;
        if (bestRow < 0)
        {
            continue;
        }

        const double left = static_cast<double>(
            gray8.at<unsigned char>(bestRow - 1, x));
        const double center = static_cast<double>(
            gray8.at<unsigned char>(bestRow, x));
        const double right = static_cast<double>(
            gray8.at<unsigned char>(bestRow + 1, x));
        points.emplace_back(
            static_cast<double>(x),
            static_cast<double>(bestRow) + parabolicOffset(left, center, right));
    }

    // 2. 与旧 type=2 生产入口对齐：亚像素点仍进入同一套补线和分段结构。
    std::vector<cv::Point2d> repairedPoints;
    points = CStripeLineRepairTools::repairLinearGaps(
        points,
        options,
        &repairedPoints);
    points = CStripeLineRepairTools::smoothCenterline(
        points,
        options.smoothWindowRadius);
    result.segments = CStripeLineRepairTools::fitSegments(points, options);
    result.centerlinePoints = points;
    result.repairedPoints = repairedPoints;

    int validSegments = 0;
    double confidenceSum = 0.0;
    for (const StripeSegment &segment : result.segments)
    {
        if (segment.valid)
        {
            ++validSegments;
            confidenceSum += segment.confidence;
        }
    }
    const double responseScore =
        CStripeResponseEvaluationTools::calculateMeanResponseScore(
            result.responseByColumn,
            120.0);
    const double segmentScore = validSegments > 0
        ? confidenceSum / static_cast<double>(validSegments)
        : 0.0;
    result.qualityScore = clampRatio(0.45 * responseScore + 0.55 * segmentScore);
    result.valid =
        static_cast<int>(points.size()) >= options.minTotalPointCount
        && validSegments > 0;
    result.message = result.valid
        ? "Steger-like stripe centerline detected"
        : "Steger-like stripe centerline has insufficient ridge points";

    const auto stopTime = std::chrono::steady_clock::now();
    result.processingTimeMs =
        std::chrono::duration<double, std::milli>(stopTime - startTime).count();
    updateState(result, state);
    return result;
}

} // namespace i2r_rlc::vision
