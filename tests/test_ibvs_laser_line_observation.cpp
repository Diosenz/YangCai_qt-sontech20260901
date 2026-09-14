/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_laser_line_observation.cpp
 * @brief       ibvs laser line observation 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsLaserLineObservation.h"
#include "ibvs/IbvsSensorFusion.h"

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

cv::Mat makeLaserImage()
{
    cv::Mat image = cv::Mat::zeros(160, 220, CV_8UC1);
    cv::line(image, cv::Point(50, 30), cv::Point(170, 110), cv::Scalar(255), 3);
    return image;
}

IbvsLaserPrior makePrior()
{
    IbvsLaserPrior prior;
    prior.frameIndex = 9;
    prior.hasVirtualWeldPoint = true;
    prior.virtualWeldPoint = { 110.0, 70.0 };
    prior.roiExpandPx = 80;
    for (int i = 0; i <= 12; ++i)
    {
        prior.virtualLaserLine.push_back({
            50.0 + static_cast<double>(i) * 10.0,
            30.0 + static_cast<double>(i) * (80.0 / 12.0)
        });
    }
    return prior;
}

bool testExtractor()
{
    IbvsLaserExtractionOptions options;
    options.enableGabor = true;
    const IbvsLaserObservationResult result =
        IbvsLaserLineExtractor::extractObservation(
            makeLaserImage(),
            makePrior(),
            options);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "IBVS laser extraction should succeed");
    ok &= expectTrue(!result.centerline.empty(), "IBVS centerline should contain points");
    ok &= expectTrue(result.observation.hasStartPoint, "IBVS LVS should have start point");
    ok &= expectTrue(result.observation.hasWeldPoint, "IBVS LVS should have weld point");
    ok &= expectTrue(
        result.observation.centerlineQuality > 0.0,
        "IBVS LVS quality should be positive");
    return ok;
}

bool testSensorFusionFallback()
{
    const IbvsLaserObservationResult extracted =
        IbvsLaserLineExtractor::extractObservation(
            makeLaserImage(),
            makePrior(),
            IbvsLaserExtractionOptions());

    IbvsTorchObservation torch;
    torch.featurePoints = { { 40.0, 20.0 }, { 16.0, 20.0 } };
    torch.confidence = 0.9;
    torch.frameIndex = 9;

    IbvsLvsObservation lowQuality = extracted.observation;
    lowQuality.confidence = 0.01;
    lowQuality.centerlineQuality = 0.01;

    IbvsFusionConfig config;
    config.minLvsConfidence = 0.5;
    config.minLvsCenterlineQuality = 0.5;
    const IbvsFeatureVector fallback = IbvsFeatureVector::Zero();
    const IbvsFusionResult result =
        IbvsSensorFusion::fuseDesiredFeature(
            torch,
            lowQuality,
            fallback,
            config);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "IBVS fusion should remain valid");
    ok &= expectTrue(!result.usedLvs, "low quality LVS should fall back");
    ok &= expectTrue(result.usedFallback, "low quality LVS should mark fallback");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testExtractor();
    ok &= testSensorFusionFallback();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: IBVS laser line observation tests\n";
    return 0;
}
