/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_localization_laser_line_observation.cpp
 * @brief       localization laser line observation 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>

#include <opencv2/imgproc.hpp>

#include "LocalizationLaserLineObservation.h"

namespace {

using namespace i2r_rlc::localization;

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

LocalizationLaserPrior makePrior()
{
    LocalizationLaserPrior prior;
    prior.frameId = 7;
    prior.hasVirtualWeldPoint = true;
    prior.virtualWeldPoint_px = { 110.0, 70.0, 7, 1.0 };
    prior.roiExpand_px = 80;
    for (int i = 0; i <= 12; ++i)
    {
        prior.virtualLaserLine_px.push_back({
            50.0 + static_cast<double>(i) * 10.0,
            30.0 + static_cast<double>(i) * (80.0 / 12.0),
            7,
            1.0
        });
    }
    return prior;
}

SensorCalibrationData makeCalibration()
{
    SensorCalibrationData calibration;
    calibration.focalLengthM = 0.01;
    calibration.sxM = 1e-5;
    calibration.syM = 1e-5;
    calibration.cxPixel = 110.0;
    calibration.cyPixel = 80.0;
    calibration.laserPlaneA = 0.0;
    calibration.laserPlaneB = 0.0;
    calibration.laserPlaneC = -100.0;
    return calibration;
}

bool testExtractor()
{
    LocalizationCenterlineExtractionOptions options;
    options.enableGabor = true;
    const LocalizationCenterlineExtractionResult result =
        LocalizationLaserLineExtractor::extractCenterline(
            makeLaserImage(),
            makePrior(),
            options);

    bool ok = true;
    ok &= expectTrue(result.succeeded, "localization centerline should extract");
    ok &= expectTrue(!result.centerline_px.empty(), "centerline should contain points");
    ok &= expectTrue(result.quality > 0.0, "centerline quality should be positive");
    ok &= expectTrue(result.roiWidth_px > 0, "ROI should be populated");
    ok &= expectTrue(
        !result.roiVirtualLaserLine_px.empty(),
        "virtual line should be filtered into ROI");
    return ok;
}

bool testPointCloudWindow()
{
    LocalizationCenterlineExtractionOptions extractionOptions;
    const LocalizationCenterlineExtractionResult centerline =
        LocalizationLaserLineExtractor::extractCenterline(
            makeLaserImage(),
            makePrior(),
            extractionOptions);

    LocalizationPointCloudWindowOptions windowOptions;
    windowOptions.realPointStride = 4;
    windowOptions.slidingWindowFrameCount = 2;

    ImageProjectionRequest projection;
    projection.calibration = makeCalibration();

    LocalizationLaserPointCloudWindow window;
    LocalizationPointCloudWindowSnapshot snapshot;
    for (int frame = 0; frame < 3; ++frame)
    {
        LocalizationPointCloudFrameRequest request;
        request.frameId = frame;
        request.centerline = centerline;
        request.realProjection = projection;
        request.options = windowOptions;
        request.virtualFramePoints_mm = {
            { static_cast<double>(frame), 0.0, 0.0 },
            { static_cast<double>(frame), 1.0, 0.0 }
        };
        snapshot = window.appendFrame(request);
    }

    bool ok = true;
    ok &= expectTrue(snapshot.available, "point cloud window should be available");
    ok &= expectTrue(
        snapshot.windowFrameCount == 2,
        "point cloud window should drop the oldest frame");
    ok &= expectTrue(
        !snapshot.realWindowPoints_mm.empty(),
        "point cloud window should contain real points");
    ok &= expectTrue(
        snapshot.virtualWindowPoints_mm.size() == 4U,
        "point cloud window should contain two virtual frames");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testExtractor();
    ok &= testPointCloudWindow();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: Localization laser line observation tests\n";
    return 0;
}
