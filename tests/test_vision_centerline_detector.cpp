/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_centerline_detector.cpp
 * @brief       vision centerline detector 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include <opencv2/core.hpp>

#include "stripe/DpStripeCenterlineDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

cv::Mat makeHorizontalStripeImage(void)
{
    cv::Mat image = cv::Mat::zeros(40, 60, CV_8UC1);
    for (int xPixel = 5; xPixel < 55; ++xPixel) {
        image.at<unsigned char>(19, xPixel) = 120;
        image.at<unsigned char>(20, xPixel) = 240;
        image.at<unsigned char>(21, xPixel) = 200;
    }
    return image;
}

cv::Mat makeInclinedStripeImage(void)
{
    cv::Mat image = cv::Mat::zeros(40, 60, CV_8UC1);
    for (int xPixel = 5; xPixel < 55; ++xPixel) {
        const int yPixel = 8 + xPixel / 5;
        image.at<unsigned char>(yPixel - 1, xPixel) = 100;
        image.at<unsigned char>(yPixel, xPixel) = 235;
        image.at<unsigned char>(yPixel + 1, xPixel) = 180;
    }
    return image;
}

i2r_rlc::vision::DpStripeDetectionOptions makeDefaultOptions(void)
{
    i2r_rlc::vision::DpStripeDetectionOptions options;
    options.blurSigma = 0.0;
    options.adaptiveThresholdRatio = 0.30;
    options.minProminenceGray = 4.0;
    options.prominenceAbsMinGray = 1.0;
    options.maxCandidatesPerColumn = 4;
    options.maxJumpPixel = 6.0;
    options.minValidPointCount = 20;
    return options;
}

int countValidPoints(
    const std::vector<i2r_rlc::vision::LaserCenterlinePoint> &points)
{
    int validCount = 0;
    for (const auto &point : points) {
        if (point.valid) {
            ++validCount;
        }
    }
    return validCount;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CDpStripeCenterlineDetector;

    const auto options = makeDefaultOptions();

    const auto emptyResult =
        CDpStripeCenterlineDetector::detectCenterline(cv::Mat(), options);
    if (!expectTrue(!emptyResult.valid) ||
        !expectTrue(emptyResult.points.empty())) {
        return 1;
    }

    const cv::Mat colorImage = cv::Mat::zeros(12, 16, CV_8UC3);
    const auto colorResult =
        CDpStripeCenterlineDetector::detectCenterline(colorImage, options);
    if (!expectTrue(!colorResult.valid) ||
        !expectTrue(colorResult.points.empty())) {
        return 2;
    }

    const auto horizontalResult =
        CDpStripeCenterlineDetector::detectCenterline(
            makeHorizontalStripeImage(), options);
    if (!expectTrue(horizontalResult.valid) ||
        !expectTrue(horizontalResult.points.size() >= 45U) ||
        !expectTrue(horizontalResult.validColumnRatio > 0.75) ||
        !expectTrue(horizontalResult.validColumnRatio <= 1.0) ||
        !expectTrue(horizontalResult.meanConfidence > 0.0) ||
        !expectTrue(horizontalResult.meanConfidence <= 1.0) ||
        !expectTrue(horizontalResult.processingTimeMs >= 0.0)) {
        return 3;
    }

    if (!expectTrue(horizontalResult.points.front().xPixel <
                    horizontalResult.points.back().xPixel) ||
        !expectNear(horizontalResult.points.front().yPixel, 20.0, 0.55)) {
        return 4;
    }

    const auto inclinedResult =
        CDpStripeCenterlineDetector::detectCenterline(
            makeInclinedStripeImage(), options);
    if (!expectTrue(inclinedResult.valid) ||
        !expectTrue(inclinedResult.points.front().yPixel <
                    inclinedResult.points.back().yPixel)) {
        return 5;
    }

    cv::Mat brokenImage = makeHorizontalStripeImage();
    for (int xPixel = 25; xPixel <= 31; ++xPixel) {
        brokenImage.at<unsigned char>(19, xPixel) = 0;
        brokenImage.at<unsigned char>(20, xPixel) = 0;
        brokenImage.at<unsigned char>(21, xPixel) = 0;
    }
    const auto brokenResult =
        CDpStripeCenterlineDetector::detectCenterline(brokenImage, options);
    if (!expectTrue(brokenResult.valid) ||
        !expectTrue(brokenResult.brokenStripeRatio > 0.0) ||
        !expectTrue(brokenResult.brokenStripeRatio <= 1.0)) {
        return 6;
    }

    cv::Mat noisyImage = makeHorizontalStripeImage();
    noisyImage.at<unsigned char>(35, 30) = 255;
    const auto noisyResult =
        CDpStripeCenterlineDetector::detectCenterline(noisyImage, options);
    if (!expectTrue(noisyResult.valid) ||
        !expectTrue(countValidPoints(noisyResult.points) >= 45)) {
        return 7;
    }

    i2r_rlc::vision::DpStripeDetectionOptions roiOptions = options;
    roiOptions.roi = cv::Rect(5, 10, 50, 20);
    roiOptions.minValidPointCount = 15;
    const auto roiResult =
        CDpStripeCenterlineDetector::detectCenterline(
            makeHorizontalStripeImage(), roiOptions);
    if (!expectTrue(roiResult.valid) ||
        !expectTrue(roiResult.roi.x == 5) ||
        !expectTrue(roiResult.roi.y == 10) ||
        !expectTrue(roiResult.points.front().xPixel >= 5.0)) {
        return 8;
    }

    std::printf("PASS: vision centerline detector tests\n");
    return 0;
}
