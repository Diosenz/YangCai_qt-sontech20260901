/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_subpixel_centerline_tools.cpp
 * @brief       vision subpixel centerline tools 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include "stripe/LaserStripeExtractor.h"
#include "stripe/SubpixelCenterlineTools.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

} // namespace

int main()
{
    using i2r_rlc::vision::LaserStripeColumnPeak;
    using i2r_rlc::vision::CLaserStripeExtractor;
    using i2r_rlc::vision::LaserStripeThresholdOptions;
    using i2r_rlc::vision::CSubpixelCenterlineTools;
    using i2r_rlc::vision::VisionPreprocessStatus;

    const auto parabola = CSubpixelCenterlineTools::fitParabola(
        100.0,
        200.0,
        160.0,
        5.0);
    if (!expectTrue(parabola.valid)
        || !expectNear(parabola.offsetPixel, 0.2142857, 0.0001)
        || !expectNear(parabola.positionPixel, 5.2142857, 0.0001)
        || !expectTrue(parabola.confidenceRatio > 0.0))
    {
        return 1;
    }

    const auto flatParabola = CSubpixelCenterlineTools::fitParabola(
        100.0,
        100.0,
        100.0,
        4.0);
    if (!expectTrue(!flatParabola.valid)
        || !expectNear(flatParabola.positionPixel, 4.0, 0.0001))
    {
        return 2;
    }

    cv::Mat image = cv::Mat::zeros(12, 5, CV_8UC1);
    for (int yPixel = 0; yPixel < image.rows; ++yPixel)
    {
        image.at<unsigned char>(yPixel, 2) =
            static_cast<unsigned char>(10 + yPixel * 10);
    }
    image.at<unsigned char>(4, 2) = 120;
    image.at<unsigned char>(5, 2) = 220;
    image.at<unsigned char>(6, 2) = 180;

    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const int scale = CSubpixelCenterlineTools::robustColumnScale(
        image,
        2,
        0,
        image.rows - 1,
        0.90,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(scale == 120))
    {
        return 3;
    }

    LaserStripeThresholdOptions extractOptions;
    extractOptions.thresholdGrayValue = 150;
    extractOptions.useRelativeThreshold = false;
    extractOptions.maxPeaksPerColumn = 2;
    const auto peaks = CLaserStripeExtractor::extractColumnPeaks(
        image,
        2,
        extractOptions,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(!peaks.empty())
        || !expectTrue(peaks[0].yPixel == 5))
    {
        return 4;
    }

    const auto refined = CSubpixelCenterlineTools::refinePeakWithParabola(
        image,
        peaks[0],
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(refined.valid)
        || !expectTrue(refined.positionPixel > 5.0)
        || !expectTrue(refined.positionPixel < 5.5))
    {
        return 5;
    }

    const auto centroid = CSubpixelCenterlineTools::calculateGrayCentroid(
        image,
        2,
        5,
        2,
        50,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(centroid.valid)
        || !expectTrue(centroid.centerYPixel > 5.0)
        || !expectTrue(centroid.centerYPixel < 6.0)
        || !expectTrue(centroid.confidenceRatio > 0.5))
    {
        return 6;
    }

    const double contrast = CSubpixelCenterlineTools::calculateLocalContrast(
        image,
        2,
        5,
        0,
        3,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(contrast > 50.0))
    {
        return 7;
    }

    const auto quality = CSubpixelCenterlineTools::evaluateLocalQuality(
        image,
        peaks[0],
        2,
        4,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(quality.valid)
        || !expectTrue(quality.qualityRatio > 0.0)
        || !expectTrue(quality.qualityRatio <= 1.0)
        || !expectTrue(quality.contrastRatio > 0.0))
    {
        return 8;
    }

    const int invalidScale = CSubpixelCenterlineTools::robustColumnScale(
        image,
        -1,
        0,
        1,
        0.5,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InvalidParameter)
        || !expectTrue(invalidScale == 0))
    {
        return 9;
    }

    std::printf("PASS: vision subpixel centerline tools tests\n");
    return 0;
}
