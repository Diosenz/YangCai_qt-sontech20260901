/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_stripe_steger.cpp
 * @brief       vision stripe steger 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "stripe/StegerStripeDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

cv::Mat makeRidgeImage(void)
{
    cv::Mat image(90, 120, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(8, 46), cv::Point(112, 46), cv::Scalar(255), 5);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0.8);
    return image;
}

} // namespace

int main()
{
    i2r_rlc::vision::StripeDetectionOptions options;
    options.thresholdGrayValue = 80;
    options.minTotalPointCount = 20;
    options.minSegmentPointCount = 5;
    options.stegerMinRidgeResponse = 1.0;

    i2r_rlc::vision::StripeDetectionState state;
    const auto result = i2r_rlc::vision::CStegerStripeDetector::detect(
        makeRidgeImage(),
        options,
        &state);

    if (!expectTrue(result.valid)
        || !expectTrue(result.centerlinePoints.size() >= 80U)
        || !expectTrue(result.qualityScore > 0.0))
    {
        return 1;
    }

    std::printf("PASS: vision stripe Steger tests\n");
    return 0;
}
