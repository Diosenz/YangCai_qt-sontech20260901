/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_dp_stripe_detector.cpp
 * @brief       vision dp stripe detector 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

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

cv::Mat makeSingleRidgeImage(void)
{
    cv::Mat image(80, 120, CV_8UC1, cv::Scalar(8));
    for (int x = 8; x < 112; ++x) {
        const int y = 28 + static_cast<int>(std::round(0.12 * x));
        cv::line(image, cv::Point(x, y - 1), cv::Point(x, y + 1), cv::Scalar(235), 1);
    }
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0.6);
    return image;
}

cv::Mat makeAmbiguousRidgeImage(void)
{
    cv::Mat image(90, 120, CV_8UC1, cv::Scalar(6));
    for (int x = 5; x < 115; ++x) {
        const int slopedY = 22 + static_cast<int>(std::round(0.22 * x));
        cv::line(image, cv::Point(x, slopedY - 1), cv::Point(x, slopedY + 1), cv::Scalar(230), 1);
        cv::line(image, cv::Point(x, 63), cv::Point(x, 65), cv::Scalar(230), 1);
    }
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0.6);
    return image;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CDpStripeCenterlineDetector;
    using i2r_rlc::vision::DpStripeDetectionOptions;

    DpStripeDetectionOptions options;
    options.minValidPointCount = 80;
    options.maxJumpPixel = 8.0;
    options.useDirectionPrior = false;

    const auto singleResult =
        CDpStripeCenterlineDetector::detect(makeSingleRidgeImage(), options);
    if (!expectTrue(singleResult.valid)
        || !expectTrue(singleResult.centerlinePoints.size() >= 90U)
        || !expectTrue(singleResult.qualityScore > 0.70)
        || !expectNear(singleResult.centerlinePoints.front().y, 29.0, 3.0))
    {
        return 1;
    }

    DpStripeDetectionOptions priorOptions;
    priorOptions.minValidPointCount = 80;
    priorOptions.maxCandidatesPerColumn = 4;
    priorOptions.maxJumpPixel = 8.0;
    priorOptions.useDirectionPrior = true;
    priorOptions.slopePriorByColumn.assign(120, 0.22);
    priorOptions.priorWeightByColumn.assign(120, 1.0);
    priorOptions.directionPriorWeight = 12.0;
    priorOptions.intensityWeight = 1.0;

    const auto priorResult =
        CDpStripeCenterlineDetector::detect(makeAmbiguousRidgeImage(), priorOptions);
    if (!expectTrue(priorResult.valid)
        || !expectTrue(priorResult.centerlinePoints.size() >= 90U))
    {
        return 2;
    }

    const double firstY = priorResult.centerlinePoints.front().y;
    const double lastY = priorResult.centerlinePoints.back().y;
    if (!expectTrue(lastY > firstY + 15.0)
        || !expectTrue(lastY < 58.0))
    {
        return 3;
    }

    DpStripeDetectionOptions roiOptions = options;
    roiOptions.roi = cv::Rect(20, 10, 60, 50);
    roiOptions.minValidPointCount = 40;
    const auto roiResult =
        CDpStripeCenterlineDetector::detect(makeSingleRidgeImage(), roiOptions);
    if (!expectTrue(roiResult.valid)
        || !expectTrue(roiResult.centerlinePoints.front().x >= 20.0)
        || !expectTrue(roiResult.centerlinePoints.back().x <= 79.0))
    {
        return 4;
    }

    std::printf("PASS: vision DP stripe detector tests\n");
    return 0;
}
