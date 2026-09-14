/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_stripe_isffa.cpp
 * @brief       vision stripe isffa 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "stripe/IsffaStripeDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

cv::Mat makeBrokenStripeImage(void)
{
    cv::Mat image(80, 140, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(10, 30), cv::Point(60, 35), cv::Scalar(255), 3);
    cv::line(image, cv::Point(72, 36), cv::Point(130, 42), cv::Scalar(255), 3);
    return image;
}

} // namespace

int main()
{
    i2r_rlc::vision::StripeDetectionOptions options;
    options.thresholdGrayValue = 100;
    options.minTotalPointCount = 20;
    options.minSegmentPointCount = 5;
    options.repairMaxGapPixel = 20;
    options.maxColumnGapPixel = 8;

    i2r_rlc::vision::StripeDetectionState state;
    const auto result = i2r_rlc::vision::CIsffaStripeDetector::detect(
        makeBrokenStripeImage(),
        options,
        &state);

    if (!expectTrue(result.valid)
        || !expectTrue(result.centerlinePoints.size() >= 90U)
        || !expectTrue(!result.repairedPoints.empty())
        || !expectTrue(result.qualityScore > 0.0))
    {
        return 1;
    }

    std::printf("PASS: vision stripe ISFFA tests\n");
    return 0;
}
