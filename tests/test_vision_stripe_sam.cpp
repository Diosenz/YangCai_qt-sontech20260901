/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_stripe_sam.cpp
 * @brief       vision stripe sam 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "stripe/SamStripeDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

cv::Mat makeStripeImage(void)
{
    cv::Mat image(80, 120, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(8, 28), cv::Point(112, 36), cv::Scalar(255), 3);
    return image;
}

} // namespace

int main()
{
    i2r_rlc::vision::StripeDetectionOptions options;
    options.thresholdGrayValue = 100;
    options.minTotalPointCount = 20;
    options.minSegmentPointCount = 5;

    i2r_rlc::vision::StripeDetectionState state;
    const auto result = i2r_rlc::vision::CSamStripeDetector::detect(
        makeStripeImage(),
        options,
        &state);

    if (!expectTrue(result.valid)
        || !expectTrue(result.centerlinePoints.size() >= 80U)
        || !expectTrue(!result.segments.empty())
        || !expectTrue(result.qualityScore > 0.0)
        || !expectTrue(!state.lastValidCenterlinePoints.empty()))
    {
        return 1;
    }

    std::printf("PASS: vision stripe SAM tests\n");
    return 0;
}
