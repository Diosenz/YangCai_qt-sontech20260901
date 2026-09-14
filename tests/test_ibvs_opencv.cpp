/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_opencv.cpp
 * @brief       ibvs opencv 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>

#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsRedFeatureDetector.h"

namespace {

using namespace i2r_rlc::ibvs;

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool expectNear(double actual, double expected, double tolerance, const char* message)
{
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message
                  << ", actual=" << actual
                  << ", expected=" << expected << '\n';
        return false;
    }

    return true;
}

bool testRedFeatureDetector()
{
    cv::Mat image(100, 120, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(image, cv::Point(35, 50), 8, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::circle(image, cv::Point(75, 50), 8, cv::Scalar(0, 0, 255), cv::FILLED);

    IbvsRedFeatureDetectorConfig config;
    config.applyGamma = false;
    config.applyBilateral = false;
    config.minPairDistancePx = 20.0;
    config.maxPairDistancePx = 60.0;

    const IbvsRedFeatureDetector detector(config);
    const auto result = detector.detect(image);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "red feature detection should succeed");
    ok &= expectTrue(result.accepted, "red feature pair should be accepted");
    ok &= expectTrue(
        result.featurePoints.first.u < result.featurePoints.second.u,
        "red feature pair should be sorted by x");
    ok &= expectNear(result.featurePoints.first.u, 35.0, 1.0, "left point u should match");
    ok &= expectNear(result.featurePoints.second.u, 75.0, 1.0, "right point u should match");
    ok &= expectNear(result.distancePx, 40.0, 2.0, "red feature distance should match");
    return ok;
}

} // namespace

/**
 * @brief OpenCV 条件编译下的 IBVS 红色双点 smoke 测试入口。
 */
int main()
{
    if (!testRedFeatureDetector()) {
        return 1;
    }

    std::cout << "PASS: IBVS OpenCV tests\n";
    return 0;
}
