/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_quality.cpp
 * @brief       vision image quality 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "preprocess/ImageQualityEvaluator.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

cv::Mat makeEdgeImage(void)
{
    cv::Mat image(64, 64, CV_8UC1, cv::Scalar(20));
    cv::rectangle(image, cv::Rect(16, 16, 32, 32), cv::Scalar(230), -1);
    return image;
}

} // namespace

int main()
{
    const cv::Mat flat(64, 64, CV_8UC1, cv::Scalar(100));
    const cv::Mat edge = makeEdgeImage();
    const i2r_rlc::vision::ImageQualityOptions options;

    const auto flatLaplace = i2r_rlc::vision::CImageQualityEvaluator::evaluate(
        flat,
        i2r_rlc::vision::ImageQualityMetric::Laplace,
        options);
    const auto edgeLaplace = i2r_rlc::vision::CImageQualityEvaluator::evaluate(
        edge,
        i2r_rlc::vision::ImageQualityMetric::Laplace,
        options);
    const auto all = i2r_rlc::vision::CImageQualityEvaluator::evaluateAll(
        edge,
        options);

    if (!expectTrue(flatLaplace.valid)
        || !expectTrue(edgeLaplace.valid)
        || !expectTrue(edgeLaplace.value > flatLaplace.value)
        || !expectTrue(all.size() == 5U)
        || !expectTrue(i2r_rlc::vision::CImageQualityEvaluator::metricToKey(
            i2r_rlc::vision::ImageQualityMetric::Tenegrad) == "Tenegrad"))
    {
        return 1;
    }

    std::printf("PASS: vision image quality tests\n");
    return 0;
}
