/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_offline_correction.cpp
 * @brief       offline correction 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "OfflineCorrectionController.h"

namespace {

namespace control = i2r_rlc::control;
namespace vision = i2r_rlc::vision;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::filesystem::path makeTempCaseDirectory()
{
    const auto root = std::filesystem::temp_directory_path()
        / "i2r_rlc_offline_correction_tests";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    return root;
}

vision::WeldSpacePoint makePoint(double x, double y, double z)
{
    return vision::WeldSpacePoint{ x, y, z, true };
}

std::vector<vision::WeldSpacePoint> makeModelPoints()
{
    std::vector<vision::WeldSpacePoint> points;
    for (int i = 0; i < 24; ++i) {
        const double x = static_cast<double>(i) * 4.0;
        const double y = std::sin(static_cast<double>(i) * 0.3) * 6.0;
        const double z = std::cos(static_cast<double>(i) * 0.2) * 2.0;
        points.push_back(makePoint(x, y, z));
    }
    return points;
}

void writeArchive(
    const std::filesystem::path& directory,
    const std::vector<vision::WeldSpacePoint>& modelPoints)
{
    std::ofstream model(directory / "case_W.txt");
    for (const auto& point : modelPoints) {
        model << point.xMm << ' ' << point.yMm << ' ' << point.zMm << '\n';
    }

    std::ofstream teach(directory / "case_T.txt");
    for (int i = 0; i < 5; ++i) {
        const auto& point = modelPoints[static_cast<std::size_t>(i * 4)];
        teach << point.xMm << ' ' << point.yMm << ' ' << point.zMm << ' '
              << 1.0 + i << ' ' << 2.0 + i << ' ' << 3.0 + i << ' '
              << 10.0 + i << ' ' << 20.0 + i << '\n';
    }
}

control::OfflineCorrectionDetectionEvent makeDetectionEvent(
    const std::vector<vision::WeldSpacePoint>& modelPoints,
    const vision::WeldSpacePoint& offset)
{
    control::OfflineCorrectionDetectionEvent event;
    for (const auto& point : modelPoints) {
        control::OfflineCorrectionDetectionSample sample;
        sample.point = makePoint(
            point.xMm + offset.xMm,
            point.yMm + offset.yMm,
            point.zMm + offset.zMm);
        event.samples.push_back(sample);
    }
    return event;
}

void testMatchedCorrectionProducesRobotCommand()
{
    const auto directory = makeTempCaseDirectory();
    const auto modelPoints = makeModelPoints();
    writeArchive(directory, modelPoints);

    control::COfflineCorrectionController controller;
    control::OfflineCorrectionOptions options;
    options.smoothingRadius = 0;
    controller.configure(options);

    auto update = controller.selectArchiveFile((directory / "case_W.txt").string());
    require(update.succeeded, "archive selection should succeed");
    update = controller.startDetection();
    require(update.succeeded, "start detection should succeed");

    const auto offset = makePoint(0.4, -0.25, 0.15);
    update = controller.appendDetectionEvent(makeDetectionEvent(modelPoints, offset));
    require(update.succeeded, "append detection should succeed");
    update = controller.finishDetection();
    require(update.succeeded, "finish detection should succeed");
    update = controller.computeCorrection();
    require(update.succeeded, "compute correction should succeed");
    require(update.robotCommand.valid, "robot command should be valid");
    require(update.robotCommand.offlineSignal == 4, "offline signal should be 4");
    require(update.robotCommand.pointCount == 5, "point count should equal teach count");
    require(update.robotCommand.teachPoints.size() == update.robotCommand.stationAxes.size(),
            "teach points and station axes should be one-to-one");

    const auto& firstTeach = update.robotCommand.teachPoints.front();
    const auto expected = modelPoints.front();
    require(std::abs(firstTeach.xUm - static_cast<long>((expected.xMm + offset.xMm) * 1000.0)) < 500,
            "corrected x should be close to translated teach point");
    require(std::abs(firstTeach.yUm - static_cast<long>((expected.yMm + offset.yMm) * 1000.0)) < 500,
            "corrected y should be close to translated teach point");
    require(std::abs(firstTeach.zUm - static_cast<long>((expected.zMm + offset.zMm) * 1000.0)) < 500,
            "corrected z should be close to translated teach point");
    require(firstTeach.rxDeg10000 == 10000, "rx should be converted to deg*10000");
    require(update.robotCommand.stationAxes.front().valid, "station axis should be valid");
    require(update.robotCommand.stationAxes.front().s1Deg10000 == 100000,
            "station s1 should be converted to deg*10000");
}

void testMissingArchiveFails()
{
    control::COfflineCorrectionController controller;
    controller.configure(control::OfflineCorrectionOptions{});
    const auto update = controller.selectArchiveFile("missing_W.txt");
    require(!update.succeeded, "missing archive should fail");
    require(!update.robotCommand.valid, "missing archive should not produce command");
}

void testInsufficientDetectionFails()
{
    const auto directory = makeTempCaseDirectory();
    const auto modelPoints = makeModelPoints();
    writeArchive(directory, modelPoints);

    control::COfflineCorrectionController controller;
    controller.configure(control::OfflineCorrectionOptions{});
    auto update = controller.selectArchiveFile((directory / "case_T.txt").string());
    require(update.succeeded, "archive selection by T file should succeed");
    update = controller.startDetection();
    require(update.succeeded, "start detection should succeed");

    control::OfflineCorrectionDetectionEvent event;
    event.samples.push_back(control::OfflineCorrectionDetectionSample{ makePoint(1.0, 2.0, 3.0) });
    update = controller.appendDetectionEvent(event);
    require(update.succeeded, "single detection event is accepted before compute");
    update = controller.computeCorrection();
    require(!update.succeeded, "insufficient detection should fail during compute");
    require(!update.robotCommand.valid, "failed compute should not produce command");
}

} // namespace

int main()
{
    try {
        testMatchedCorrectionProducesRobotCommand();
        testMissingArchiveFails();
        testInsufficientDetectionFails();
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }

    std::cout << "PASS: offline correction tests\n";
    return 0;
}
