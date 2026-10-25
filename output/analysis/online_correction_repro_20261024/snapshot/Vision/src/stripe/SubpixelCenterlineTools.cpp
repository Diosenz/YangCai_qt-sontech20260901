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
 * @file        SubpixelCenterlineTools.cpp
 * @brief       亚像素中心线计算工具
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

#include "stripe/SubpixelCenterlineTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 写入可选预处理状态。
 * @param status 可选状态指针；为空时忽略。
 * @param value 要写入的状态值。
 */
void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

/**
 * @brief 判断图像是否为可处理灰度图。
 * @param grayImage 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 */
bool isValidGrayImage(const cv::Mat &grayImage)
{
    return !grayImage.empty() && grayImage.type() == CV_8UC1;
}

/**
 * @brief 将比例值裁剪到 0~1。
 * @param value 输入比例。
 * @return 裁剪后的比例。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 将抛物线亚像素偏移限制到相邻像素范围。
 * @param value 输入偏移，单位 pixel。
 * @return 裁剪到 [-0.5, 0.5] 的偏移。
 */
double clampSubpixelOffset(double value)
{
    return std::max(-0.5, std::min(0.5, value));
}

/**
 * @brief 判断像素坐标是否落在图像内。
 * @param grayImage 输入图像。
 * @param xPixel x 坐标，单位 pixel。
 * @param yPixel y 坐标，单位 pixel。
 * @return true 表示坐标有效。
 */
bool isValidPoint(const cv::Mat &grayImage, int xPixel, int yPixel)
{
    return xPixel >= 0
        && yPixel >= 0
        && xPixel < grayImage.cols
        && yPixel < grayImage.rows;
}

} // namespace

SubpixelParabolaResult CSubpixelCenterlineTools::fitParabola(
    double leftValue,
    double centerValue,
    double rightValue,
    double centerPositionPixel)
{
    SubpixelParabolaResult result;
    result.positionPixel = centerPositionPixel;

    const double denominator = leftValue - 2.0 * centerValue + rightValue;
    if (std::fabs(denominator) < 1e-9)
    {
        return result;
    }

    result.offsetPixel = clampSubpixelOffset(
        0.5 * (leftValue - rightValue) / denominator);
    result.positionPixel = centerPositionPixel + result.offsetPixel;
    result.curvature = std::fabs(denominator);

    const double sideMax = std::max(leftValue, rightValue);
    const double prominence = std::max(0.0, centerValue - sideMax);
    result.confidenceRatio = clampRatio(prominence / 255.0);
    result.valid = true;
    return result;
}

SubpixelParabolaResult CSubpixelCenterlineTools::refinePeakWithParabola(
    const cv::Mat &grayImage,
    const LaserStripeColumnPeak &peak,
    VisionPreprocessStatus *status)
{
    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return SubpixelParabolaResult();
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return SubpixelParabolaResult();
    }

    if (!isValidPoint(grayImage, peak.xPixel, peak.yPixel)
        || peak.yPixel <= 0
        || peak.yPixel >= grayImage.rows - 1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return SubpixelParabolaResult();
    }

    const double leftValue =
        grayImage.at<unsigned char>(peak.yPixel - 1, peak.xPixel);
    const double centerValue =
        grayImage.at<unsigned char>(peak.yPixel, peak.xPixel);
    const double rightValue =
        grayImage.at<unsigned char>(peak.yPixel + 1, peak.xPixel);

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return fitParabola(leftValue, centerValue, rightValue, peak.yPixel);
}

int CSubpixelCenterlineTools::robustColumnScale(
    const cv::Mat &grayImage,
    int xPixel,
    int yMinPixel,
    int yMaxPixel,
    double quantileRatio,
    VisionPreprocessStatus *status)
{
    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return 0;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return 0;
    }

    if (xPixel < 0
        || xPixel >= grayImage.cols
        || yMinPixel < 0
        || yMaxPixel >= grayImage.rows
        || yMinPixel > yMaxPixel
        || quantileRatio < 0.0
        || quantileRatio > 1.0)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return 0;
    }

    int histogram[256] = { 0 };
    int totalCount = 0;
    for (int yPixel = yMinPixel; yPixel <= yMaxPixel; ++yPixel)
    {
        const int value = grayImage.at<unsigned char>(yPixel, xPixel);
        ++histogram[value];
        ++totalCount;
    }

    const int targetIndex = static_cast<int>(
        std::floor(quantileRatio * static_cast<double>(totalCount - 1)));
    int cumulativeCount = 0;
    for (int grayValue = 0; grayValue < 256; ++grayValue)
    {
        cumulativeCount += histogram[grayValue];
        if (cumulativeCount > targetIndex)
        {
            setStatus(status, VisionPreprocessStatus::Succeeded);
            return grayValue;
        }
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return 255;
}

GrayCentroidResult CSubpixelCenterlineTools::calculateGrayCentroid(
    const cv::Mat &grayImage,
    int xPixel,
    int centerYPixel,
    int halfWindowPixel,
    int backgroundGrayValue,
    VisionPreprocessStatus *status)
{
    GrayCentroidResult result;

    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return result;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return result;
    }

    if (!isValidPoint(grayImage, xPixel, centerYPixel)
        || halfWindowPixel < 1
        || backgroundGrayValue < 0
        || backgroundGrayValue > 255)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    const int yBegin = std::max(0, centerYPixel - halfWindowPixel);
    const int yEnd = std::min(grayImage.rows - 1, centerYPixel + halfWindowPixel);

    double weightedPositionSum = 0.0;
    double weightSum = 0.0;
    for (int yPixel = yBegin; yPixel <= yEnd; ++yPixel)
    {
        const int grayValue = grayImage.at<unsigned char>(yPixel, xPixel);
        const double weight = static_cast<double>(
            std::max(0, grayValue - backgroundGrayValue));
        weightedPositionSum += static_cast<double>(yPixel) * weight;
        weightSum += weight;
    }

    if (weightSum <= 0.0)
    {
        setStatus(status, VisionPreprocessStatus::Succeeded);
        return result;
    }

    result.valid = true;
    result.centerYPixel = weightedPositionSum / weightSum;
    result.totalWeight = weightSum;
    result.confidenceRatio = clampRatio(
        1.0 - std::fabs(result.centerYPixel - centerYPixel)
        / static_cast<double>(halfWindowPixel));

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return result;
}

double CSubpixelCenterlineTools::calculateLocalContrast(
    const cv::Mat &grayImage,
    int xPixel,
    int centerYPixel,
    int innerHalfWindowPixel,
    int outerHalfWindowPixel,
    VisionPreprocessStatus *status)
{
    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return 0.0;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return 0.0;
    }

    if (!isValidPoint(grayImage, xPixel, centerYPixel)
        || innerHalfWindowPixel < 0
        || outerHalfWindowPixel <= innerHalfWindowPixel)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return 0.0;
    }

    const int yBegin = std::max(0, centerYPixel - outerHalfWindowPixel);
    const int yEnd = std::min(grayImage.rows - 1, centerYPixel + outerHalfWindowPixel);

    double innerSum = 0.0;
    int innerCount = 0;
    double outerSum = 0.0;
    int outerCount = 0;

    for (int yPixel = yBegin; yPixel <= yEnd; ++yPixel)
    {
        const int distance = yPixel >= centerYPixel
            ? yPixel - centerYPixel
            : centerYPixel - yPixel;
        const double grayValue =
            static_cast<double>(grayImage.at<unsigned char>(yPixel, xPixel));
        if (distance <= innerHalfWindowPixel)
        {
            innerSum += grayValue;
            ++innerCount;
        }
        else
        {
            outerSum += grayValue;
            ++outerCount;
        }
    }

    if (innerCount == 0 || outerCount == 0)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return 0.0;
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return std::max(0.0,
                    innerSum / static_cast<double>(innerCount)
                    - outerSum / static_cast<double>(outerCount));
}

LocalCenterlineQuality CSubpixelCenterlineTools::evaluateLocalQuality(
    const cv::Mat &grayImage,
    const LaserStripeColumnPeak &peak,
    int centroidHalfWindowPixel,
    int backgroundHalfWindowPixel,
    VisionPreprocessStatus *status)
{
    LocalCenterlineQuality quality;
    quality.xPixel = peak.xPixel;
    quality.centerYPixel = peak.yPixel;

    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return quality;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return quality;
    }

    if (!isValidPoint(grayImage, peak.xPixel, peak.yPixel)
        || centroidHalfWindowPixel < 1
        || backgroundHalfWindowPixel <= centroidHalfWindowPixel)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return quality;
    }

    VisionPreprocessStatus localStatus = VisionPreprocessStatus::Succeeded;
    const int backgroundGrayValue = robustColumnScale(
        grayImage,
        peak.xPixel,
        std::max(0, peak.yPixel - backgroundHalfWindowPixel),
        std::min(grayImage.rows - 1, peak.yPixel + backgroundHalfWindowPixel),
        0.25,
        &localStatus);
    if (localStatus != VisionPreprocessStatus::Succeeded)
    {
        setStatus(status, localStatus);
        return quality;
    }

    const GrayCentroidResult centroid = calculateGrayCentroid(
        grayImage,
        peak.xPixel,
        peak.yPixel,
        centroidHalfWindowPixel,
        backgroundGrayValue,
        &localStatus);
    if (localStatus != VisionPreprocessStatus::Succeeded)
    {
        setStatus(status, localStatus);
        return quality;
    }

    const double contrastGrayValue = calculateLocalContrast(
        grayImage,
        peak.xPixel,
        peak.yPixel,
        centroidHalfWindowPixel,
        backgroundHalfWindowPixel,
        &localStatus);
    if (localStatus != VisionPreprocessStatus::Succeeded)
    {
        setStatus(status, localStatus);
        return quality;
    }

    quality.valid = centroid.valid && contrastGrayValue > 0.0;
    quality.centerYPixel = centroid.valid
        ? centroid.centerYPixel
        : static_cast<double>(peak.yPixel);
    quality.peakGrayValue =
        static_cast<double>(grayImage.at<unsigned char>(peak.yPixel, peak.xPixel));
    quality.backgroundGrayValue = static_cast<double>(backgroundGrayValue);
    quality.contrastGrayValue = contrastGrayValue;
    quality.contrastRatio = clampRatio(contrastGrayValue / 255.0);
    quality.qualityRatio = quality.valid
        ? clampRatio(0.5 * quality.contrastRatio + 0.5 * centroid.confidenceRatio)
        : 0.0;

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return quality;
}

} // namespace i2r_rlc::vision
