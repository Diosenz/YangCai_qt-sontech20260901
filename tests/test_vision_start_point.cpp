/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_start_point.cpp
 * @brief       vision start point 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "feature/WeldStartPointDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

cv::Mat makeTJointImage(void)
{
    cv::Mat image(70, 90, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 20), cv::Point(45, 45), cv::Scalar(255), 3);
    cv::line(image, cv::Point(45, 45), cv::Point(84, 18), cv::Scalar(255), 3);
    return image;
}

} // namespace

int main()
{
    i2r_rlc::vision::WeldStartPointOptions options;
    options.seamMode = i2r_rlc::vision::ActiveContourSeamMode::TJoint;
    options.activeContourOptions.initialThresholdGrayValue = 80;
    options.activeContourOptions.minConfidence = 0.05;
    options.minConfidence = 0.05;

    i2r_rlc::vision::WeldStartPointState state;
    const auto result = i2r_rlc::vision::CWeldStartPointDetector::detect(
        makeTJointImage(),
        options,
        &state);

    if (!expectTrue(result.valid)
        || !expectTrue(result.startPoint.valid)
        || !expectTrue(result.endPoint.valid)
        || !expectTrue(result.activeContourResult.activeCurve.curvePoints.size() >= 2U)
        || !expectTrue(!state.recentEndpointResults.empty()))
    {
        std::printf("message=%s\n", result.message.c_str());
        return 1;
    }

    std::printf("PASS: vision start point tests\n");
    return 0;
}
