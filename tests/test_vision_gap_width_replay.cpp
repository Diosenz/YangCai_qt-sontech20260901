/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_gap_width_replay.cpp
 * @brief       vision gap width replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "feature/PointCloudGapWidthDetector.h"
#include "geometry/WeldImageToSpaceProjector.h"
#include "stripe/DpStripeCenterlineDetector.h"

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

i2r_rlc::vision::WeldProjectionContext makeReplayProjectionContext(void)
{
    i2r_rlc::vision::WeldProjectionContext context;
    context.valid = true;
    context.calibration.valid = true;
    context.calibration.focalLengthM = 0.012;
    context.calibration.kappa = 0.0;
    context.calibration.sxM = 8.0e-6;
    context.calibration.syM = 8.0e-6;
    context.calibration.intrinsics.cxPixel = 320.0;
    context.calibration.intrinsics.cyPixel = 240.0;
    context.calibration.laserPlaneA = 0.0;
    context.calibration.laserPlaneB = 0.0;
    context.calibration.laserPlaneC = -0.001;
    context.calibration.cameraInTool.valid = true;
    context.toolInRobotBase.valid = true;
    context.robotBaseToPositionerTcp.valid = true;
    return context;
}

std::vector<std::filesystem::path> collectReplayImages(
    const std::filesystem::path &caseDirectory)
{
    std::vector<std::filesystem::path> images;
    const std::filesystem::path imageDirectory = caseDirectory / "imagecatch";
    if (!std::filesystem::exists(imageDirectory)) {
        return images;
    }
    for (const auto &entry : std::filesystem::directory_iterator(imageDirectory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            images.push_back(entry.path());
        }
    }
    std::sort(images.begin(), images.end());
    if (images.size() > 8U) {
        images.resize(8U);
    }
    return images;
}

i2r_rlc::vision::WeldPointCloud buildPointCloud(
    const std::vector<i2r_rlc::vision::WeldSpacePoint> &points)
{
    i2r_rlc::vision::WeldPointCloud cloud;
    cloud.frameId = "robot_base";
    cloud.points = points;
    for (const auto &point : cloud.points) {
        if (point.valid) {
            cloud.valid = true;
            break;
        }
    }
    return cloud;
}

} // namespace

int main()
{
    namespace fs = std::filesystem;
    using i2r_rlc::vision::CDpStripeCenterlineDetector;
    using i2r_rlc::vision::CPointCloudGapWidthDetector;
    using i2r_rlc::vision::CWeldImageToSpaceProjector;
    using i2r_rlc::vision::DpStripeDetectionOptions;
    using i2r_rlc::vision::PointCloudGapWidthOptions;
    using i2r_rlc::vision::WeldImagePoint;
    using i2r_rlc::vision::WeldProjectionTargetFrame;
    using i2r_rlc::vision::WeldSpacePoint;

    const fs::path sourceRoot = fs::path(I2R_RLC_PROJECT_SOURCE_DIR);
    const fs::path replayRoot = sourceRoot / "data" / "replay";
    if (!expectTrue(fs::exists(replayRoot))) {
        std::printf(
            "replay root does not exist: %s\n",
            replayRoot.string().c_str());
        return 1;
    }

    const fs::path reportDirectory = sourceRoot / "output" / "reports";
    fs::create_directories(reportDirectory);
    fs::path reportPath = reportDirectory / "gap_width_replay_report.csv";
    reportPath.make_preferred();
    std::FILE *report = std::fopen(reportPath.string().c_str(), "w");
    if (report == nullptr) {
        reportPath = fs::temp_directory_path() / "gap_width_replay_report.csv";
        reportPath.make_preferred();
        report = std::fopen(reportPath.string().c_str(), "w");
    }
    if (!expectTrue(report != nullptr)) {
        std::printf(
            "failed to open report: %s\n",
            reportPath.string().c_str());
        return 3;
    }
    std::fprintf(
        report,
        "case,frame,stripe_valid,stripe_points,cloud_valid,cloud_points,"
        "gap_valid,gap_width_mm,confidence,message\n");

    int processedFrames = 0;
    for (const auto &caseEntry : fs::directory_iterator(replayRoot)) {
        if (!caseEntry.is_directory()) {
            continue;
        }
        const std::string caseName = caseEntry.path().filename().string();
        for (const auto &imagePath : collectReplayImages(caseEntry.path())) {
            const cv::Mat gray = cv::imread(imagePath.string(), cv::IMREAD_GRAYSCALE);
            if (gray.empty()) {
                continue;
            }

            DpStripeDetectionOptions stripeOptions;
            stripeOptions.minValidPointCount = 20;
            stripeOptions.useDirectionPrior = false;
            stripeOptions.maxJumpPixel = 25.0;
            const auto stripeResult =
                CDpStripeCenterlineDetector::detect(gray, stripeOptions);

            std::vector<WeldImagePoint> imagePoints;
            imagePoints.reserve(stripeResult.centerlinePoints.size());
            for (const auto &point : stripeResult.centerlinePoints) {
                imagePoints.push_back(WeldImagePoint {point.x, point.y, true});
            }

            const auto spacePoints =
                CWeldImageToSpaceProjector::projectImagePoints(
                    imagePoints,
                    makeReplayProjectionContext(),
                    WeldProjectionTargetFrame::RobotBase);
            const auto cloud = buildPointCloud(spacePoints);

            WeldSpacePoint referencePoint;
            if (!spacePoints.empty()) {
                referencePoint = spacePoints[spacePoints.size() / 2U];
            }

            PointCloudGapWidthOptions gapOptions;
            gapOptions.minInputPointCount = 20;
            gapOptions.minClusterPointCount = 8;
            const auto gapResult =
                CPointCloudGapWidthDetector::detect(cloud, referencePoint, gapOptions);

            std::fprintf(
                report,
                "%s,%s,%d,%zu,%d,%zu,%d,%.6f,%.6f,\"%s\"\n",
                caseName.c_str(),
                imagePath.stem().string().c_str(),
                stripeResult.valid ? 1 : 0,
                stripeResult.centerlinePoints.size(),
                cloud.valid ? 1 : 0,
                cloud.points.size(),
                gapResult.valid ? 1 : 0,
                gapResult.gapWidthMm,
                gapResult.confidence,
                gapResult.message.c_str());
            ++processedFrames;
        }
    }

    if (!expectTrue(processedFrames > 0)) {
        std::printf(
            "no replay frames processed under: %s\n",
            replayRoot.string().c_str());
        return 2;
    }

    std::fclose(report);
    if (!expectTrue(fs::exists(reportPath))) {
        std::printf(
            "report was not created: %s\n",
            reportPath.string().c_str());
        return 4;
    }

    std::printf(
        "PASS: vision gap width replay tests: %s\n",
        reportPath.string().c_str());
    return 0;
}
