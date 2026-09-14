/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_robot_sim_virtual_reference_cache.cpp
 * @brief       robot sim virtual reference cache 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>

#include "RobotSimVirtualReferenceCache.h"

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool expectNear(
    const double actual,
    const double expected,
    const double tolerance,
    const char* message)
{
    if (std::abs(actual - expected) > tolerance) {
        std::cerr
            << "FAIL: " << message
            << ", actual=" << actual
            << ", expected=" << expected
            << '\n';
        return false;
    }

    return true;
}

bool testRobotSimResultFeedsLocalizationAndIbvs()
{
    i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult result;
    result.available = true;
    result.workpieceId = QStringLiteral("robotsim-workpiece");
    result.laserLinePoints_mm = {
        {143.0, 6.0, 95.0},
        {144.0, 7.0, 96.0}
    };
    result.laserLineImagePoints_px = {
        {812.0, 904.0},
        {823.0, 874.0}
    };
    result.hasSeamPointImage_px = true;
    result.seamPointImage_px = {1034.0, 374.0};
    result.message = QStringLiteral("RobotSim fake response");

    const i2r_rlc::ui::RobotSimVirtualReferenceCache cache =
        i2r_rlc::ui::makeRobotSimVirtualReferenceCache(result, 7, 1013);

    bool ok = true;
    ok &= expectTrue(cache.available, "cache should be available");
    ok &= expectTrue(cache.localizationSnapshot.available,
                     "localization snapshot should be available");
    ok &= expectTrue(cache.localizationSnapshot.laserLinePoints_mm.size() == 2U,
                     "localization should receive 3D laser points");
    ok &= expectTrue(cache.hasIbvsLaserPrior,
                     "IBVS laser prior should be available");
    ok &= expectTrue(cache.ibvsLaserPrior.virtualLaserLine.size() == 2U,
                     "IBVS should receive image laser line");
    ok &= expectTrue(cache.ibvsLaserPrior.hasVirtualWeldPoint,
                     "IBVS should receive seam image point");
    ok &= expectTrue(cache.ibvsLvsObservation.has_value(),
                     "IBVS LVS observation should be generated from prior");
    if (cache.ibvsLvsObservation.has_value()) {
        ok &= expectNear(cache.ibvsLvsObservation->startPoint.u, 812.0, 1e-9,
                         "IBVS start point u should come from first laser point");
        ok &= expectNear(cache.ibvsLvsObservation->weldPoint.u, 1034.0, 1e-9,
                         "IBVS weld point u should come from seam point");
    }
    return ok;
}

bool testEmptyRobotSimResultDoesNotFeedModules()
{
    i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult result;
    result.available = true;
    result.message = QStringLiteral("RobotSim empty response");

    const i2r_rlc::ui::RobotSimVirtualReferenceCache cache =
        i2r_rlc::ui::makeRobotSimVirtualReferenceCache(result, 8, 1013);

    bool ok = true;
    ok &= expectTrue(cache.available, "raw cache should record RobotSim availability");
    ok &= expectTrue(!cache.localizationSnapshot.available,
                     "localization snapshot should reject empty 3D data");
    ok &= expectTrue(!cache.hasIbvsLaserPrior,
                     "IBVS prior should reject empty image data");
    ok &= expectTrue(!cache.ibvsLvsObservation.has_value(),
                     "IBVS LVS observation should be absent for empty data");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testRobotSimResultFeedsLocalizationAndIbvs();
    ok &= testEmptyRobotSimResultDoesNotFeedModules();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: RobotSim virtual reference cache tests\n";
    return 0;
}
