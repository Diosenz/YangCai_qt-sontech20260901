/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_laser_stripe_extractor.cpp
 * @brief       vision laser stripe extractor 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "stripe/LaserStripeExtractor.h"

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
    using i2r_rlc::vision::LaserStripeExtractionResult;
    using i2r_rlc::vision::CLaserStripeExtractor;
    using i2r_rlc::vision::LaserStripeThresholdOptions;
    using i2r_rlc::vision::VisionPreprocessStatus;

    LaserStripeThresholdOptions options;
    options.thresholdGrayValue = 180;
    options.useRelativeThreshold = false;
    options.minForegroundPixelsPerColumn = 2;
    options.roiPaddingPixel = 1;
    options.maxPeaksPerColumn = 2;

    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const cv::Mat emptyMask = CLaserStripeExtractor::createMask(
        cv::Mat(),
        options,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::EmptyInput)
        || !expectTrue(emptyMask.empty()))
    {
        return 1;
    }

    cv::Mat stripeImage = cv::Mat::zeros(20, 30, CV_8UC1);
    cv::rectangle(stripeImage, cv::Rect(5, 8, 10, 4), cv::Scalar(220), -1);
    stripeImage.at<unsigned char>(2, 2) = 255;

    const cv::Mat mask = CLaserStripeExtractor::createMask(
        stripeImage,
        options,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(!mask.empty())
        || !expectTrue(mask.type() == CV_8UC1)
        || !expectTrue(mask.rows == 20)
        || !expectTrue(mask.cols == 30)
        || !expectTrue(mask.at<unsigned char>(8, 5) == 255)
        || !expectTrue(mask.at<unsigned char>(0, 0) == 0))
    {
        return 2;
    }

    const LaserStripeExtractionResult result = CLaserStripeExtractor::extract(
        stripeImage,
        options,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(!result.mask.empty())
        || !expectTrue(result.foregroundPixelCount == 41)
        || !expectNear(result.foregroundRatio, 41.0 / 600.0, 0.0001))
    {
        return 3;
    }

    if (!expectTrue(result.columnProfiles.size() == 30)
        || !expectTrue(result.columnProfiles[5].hasStripe)
        || !expectTrue(result.columnProfiles[5].firstYPixel == 8)
        || !expectTrue(result.columnProfiles[5].lastYPixel == 11)
        || !expectTrue(result.columnProfiles[5].stripeWidthPixel == 4)
        || !expectTrue(!result.columnProfiles[2].hasStripe))
    {
        return 4;
    }

    if (!expectTrue(result.roiEstimate.validColumnCount == 10)
        || !expectNear(result.roiEstimate.validColumnRatio, 10.0 / 30.0, 0.0001)
        || !expectNear(result.roiEstimate.meanStripeWidthPixel, 4.0, 0.0001)
        || !expectTrue(result.roiEstimate.roi.xPixel == 4)
        || !expectTrue(result.roiEstimate.roi.yPixel == 7)
        || !expectTrue(result.roiEstimate.roi.widthPixel == 12)
        || !expectTrue(result.roiEstimate.roi.heightPixel == 6))
    {
        return 5;
    }

    cv::Mat peakImage = cv::Mat::zeros(9, 3, CV_8UC1);
    peakImage.at<unsigned char>(2, 1) = 210;
    peakImage.at<unsigned char>(5, 1) = 240;
    peakImage.at<unsigned char>(6, 1) = 100;
    const auto peaks = CLaserStripeExtractor::extractColumnPeaks(
        peakImage,
        1,
        options,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(peaks.size() == 2)
        || !expectTrue(peaks[0].yPixel == 5)
        || !expectTrue(peaks[0].grayValue == 240)
        || !expectNear(peaks[0].scoreRatio, 240.0 / 255.0, 0.0001))
    {
        return 6;
    }

    LaserStripeThresholdOptions invalidOptions = options;
    invalidOptions.relativeThresholdRatio = 1.5;
    const cv::Mat invalidMask = CLaserStripeExtractor::createMask(
        stripeImage,
        invalidOptions,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InvalidParameter)
        || !expectTrue(invalidMask.empty()))
    {
        return 7;
    }

    std::printf("PASS: vision laser stripe extractor tests\n");
    return 0;
}
