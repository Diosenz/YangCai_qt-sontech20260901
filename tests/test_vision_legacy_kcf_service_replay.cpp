/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_legacy_kcf_service_replay.cpp
 * @brief       vision legacy kcf service replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "track/LegacyOnlineKcfDetector.h"

namespace {

namespace fs = std::filesystem;

std::vector<i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px> parseLine(
    const std::string &line)
{
    std::vector<i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px> points;
    std::istringstream stream(line);
    double x = 0.0;
    double y = 0.0;
    while (stream >> x >> y) {
        points.push_back({x, y});
    }
    return points;
}

i2r_rlc::vision::WeldProjectionContext makeContext()
{
    i2r_rlc::vision::WeldProjectionContext context;
    context.calibration.focalLengthM = 0.0122258;
    context.calibration.kappa = -2503.95;
    context.calibration.sxM = 3.75202e-06;
    context.calibration.syM = 3.75e-06;
    context.calibration.intrinsics.fxPixel =
        context.calibration.focalLengthM / context.calibration.sxM;
    context.calibration.intrinsics.fyPixel =
        context.calibration.focalLengthM / context.calibration.syM;
    context.calibration.intrinsics.cxPixel = 672.386;
    context.calibration.intrinsics.cyPixel = 513.657;
    context.calibration.laserPlaneA = 0.334870;
    context.calibration.laserPlaneB = 10.819546;
    context.calibration.laserPlaneC = -4.521574;
    context.calibration.valid = true;
    context.valid = true;
    return context;
}

} // namespace

int main()
{
    const fs::path replay =
        fs::path(I2R_RLC_PROJECT_SOURCE_DIR) / "data" / "replay" / "penzui";
    std::ifstream weldInput(replay / "simulatedWeldpt.txt");
    std::ifstream laserInput(replay / "simulatedLaser.txt");
    i2r_rlc::vision::CLegacyOnlineKcfDetector detector;
    const auto context = makeContext();

    int processed = 0;
    for (int frame = 0; frame < 8; ++frame) {
        double weldX = 0.0;
        double weldY = 0.0;
        std::string laserLine;
        if (!(weldInput >> weldX >> weldY) || !std::getline(laserInput, laserLine)) {
            break;
        }
        cv::Mat gray = cv::imread(
            (replay / "imagecatch" / ("image_" + std::to_string(frame) + ".png")).string(),
            cv::IMREAD_GRAYSCALE);
        i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation observation;
        observation.weldPointImage_px = {weldX, weldY};
        observation.hasWeldPointImage_px = true;
        observation.laserLineImagePoints_px = parseLine(laserLine);
        observation.seamIndex = 0;
        observation.valid = true;
        const auto result = detector.process(
            gray,
            observation,
            context,
            i2r_rlc::vision::LegacyOnlineKcfOptions{});
        if (gray.empty() || result.trackedRoi.empty()
            || result.processedImage.empty()) {
            std::cerr << "legacy KCF service smoke failed at frame " << frame
                      << ": " << result.message << '\n';
            return 1;
        }
        ++processed;
    }
    if (processed != 8) {
        return 2;
    }
    std::cout << "PASS: shared legacy KCF service replay frames=" << processed << '\n';
    return 0;
}
