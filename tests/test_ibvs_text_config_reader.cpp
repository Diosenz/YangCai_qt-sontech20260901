/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_text_config_reader.cpp
 * @brief       ibvs text config reader 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "ibvs/IbvsTextConfigReader.h"

namespace {

using namespace i2r_rlc::ibvs;

constexpr double kTolerance = 1e-9;

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool expectNear(
    double actual,
    double expected,
    double tolerance,
    const char* message)
{
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message
                  << ", actual=" << actual
                  << ", expected=" << expected << '\n';
        return false;
    }

    return true;
}

bool expectVectorNear(
    const Eigen::VectorXd& actual,
    const Eigen::VectorXd& expected,
    double tolerance,
    const char* message)
{
    if ((actual - expected).norm() > tolerance) {
        std::cerr << "FAIL: " << message
                  << ", actual=[" << actual.transpose()
                  << "], expected=[" << expected.transpose() << "]\n";
        return false;
    }

    return true;
}

std::string repoConfigPath(const std::string& fileName)
{
    const std::filesystem::path root(I2R_RLC_PROJECT_SOURCE_DIR);
    return (root / "config" / "IBVS" / fileName).string();
}

std::string repoWorkpieceConfigPath(const std::string& fileName)
{
    const std::filesystem::path root(I2R_RLC_PROJECT_SOURCE_DIR);
    return (root / "config" / "workpieces" / fileName).string();
}

bool testRuntimeDefaults()
{
    const IbvsRuntimeTextConfigResult result =
        IbvsTextConfigReader::readRuntimeDefaults(
            repoConfigPath("ibvs_runtime_defaults.txt"));

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "runtime defaults should load");
    ok &= expectNear(
        result.config.featureGeometry.h1Mm,
        21.0,
        kTolerance,
        "runtime H1 should come from txt");
    ok &= expectNear(
        result.config.featureGeometry.h2Mm,
        24.0,
        kTolerance,
        "runtime H2 should come from txt");
    ok &= expectNear(
        result.config.featureGeometry.overrideH2Px,
        38.6409,
        kTolerance,
        "runtime h2 pixel override should come from txt");
    ok &= expectNear(
        result.config.featureGeometry.overrideUnitDirection.u,
        -0.7513,
        kTolerance,
        "runtime unit direction u should come from txt");
    ok &= expectNear(
        result.config.featureGeometry.overrideUnitDirection.v,
        0.6599,
        kTolerance,
        "runtime unit direction v should come from txt");
    ok &= expectNear(
        result.config.unifiedVelocity.maxVelocityMps,
        0.05,
        kTolerance,
        "unified velocity cap should come from txt");
    ok &= expectNear(
        result.config.lvsConstraint.betaLvs,
        1.0,
        kTolerance,
        "LVS beta should come from txt");
    ok &= expectVectorNear(
        result.config.fusion.baseChannelSigmasPx,
        (Eigen::Vector4d() << 0.55, 0.38, 0.55, 0.38).finished(),
        kTolerance,
        "fusion channel sigmas should come from txt");
    ok &= expectNear(
        result.config.control.maxCameraVelocityMps,
        0.5,
        kTolerance,
        "control velocity cap should come from txt");
    ok &= expectNear(
        result.config.kalman.nisRejectThreshold,
        13.28,
        kTolerance,
        "kalman NIS reject threshold should come from txt");
    return ok;
}

bool testWorkpieceProfiles()
{
    const IbvsWorkpieceProfilesResult result =
        IbvsTextConfigReader::readWorkpieceProfiles(
            repoConfigPath("workpiece_profiles.txt"));
    const IbvsWorkpieceProfile* profile =
        IbvsTextConfigReader::findProfileForLegacyType(result.profiles, 4);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "workpiece profiles should load");
    ok &= expectTrue(profile != NULL, "legacy workpiece type 4 should be found");
    if (profile == NULL)
    {
        return false;
    }

    ok &= expectTrue(
        profile->hasPositionConstraint,
        "workpiece type 4 should have txt position constraint");
    ok &= expectNear(
        profile->positionConstraint.xMinMm,
        905.2,
        kTolerance,
        "workpiece type 4 x min should come from txt");
    ok &= expectNear(
        profile->positionConstraint.xMaxMm,
        905.4,
        kTolerance,
        "workpiece type 4 x max should come from txt");
    ok &= expectNear(
        profile->positionConstraint.yMinMm,
        -18.94,
        kTolerance,
        "workpiece type 4 y min should come from txt");
    ok &= expectNear(
        profile->positionConstraint.yMaxMm,
        -18.74,
        kTolerance,
        "workpiece type 4 y max should come from txt");
    ok &= expectNear(
        profile->positionConstraint.zMinMm,
        -253.7,
        kTolerance,
        "workpiece type 4 z min should come from txt");
    ok &= expectNear(
        profile->positionConstraint.zMaxMm,
        -253.5,
        kTolerance,
        "workpiece type 4 z max should come from txt");
    ok &= expectNear(
        profile->positionConstraint.lambdaPos,
        2.0,
        kTolerance,
        "workpiece type 4 lambda should come from txt");
    ok &= expectTrue(
        profile->hasFeatureGeometry,
        "workpiece type 4 should have txt feature geometry override");
    ok &= expectNear(
        profile->featureGeometry.overrideH2Px,
        38.6409,
        kTolerance,
        "workpiece type 4 h2 override should come from txt");
    return ok;
}

bool testSharedWorkpieceProfiles()
{
    const IbvsWorkpieceProfilesResult result =
        IbvsTextConfigReader::readWorkpieceProfiles(
            repoWorkpieceConfigPath("workpiece_profiles.txt"));
    const IbvsWorkpieceProfile* defaultProfile =
        IbvsTextConfigReader::findProfileById(
            result.profiles,
            result.defaultWorkpieceId);
    const IbvsWorkpieceProfile* legacyProfile =
        IbvsTextConfigReader::findProfileForLegacyType(result.profiles, 4);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "shared workpiece profiles should load");
    ok &= expectTrue(
        result.profiles.size() == 4U,
        "shared workpiece profiles should contain four profiles");
    ok &= expectTrue(
        result.defaultWorkpieceId == "chufengkouxiao",
        "shared default workpiece should match current behavior");
    ok &= expectTrue(
        defaultProfile != NULL,
        "shared default workpiece id should resolve");
    ok &= expectTrue(
        legacyProfile != NULL,
        "legacy workpiece type 4 should resolve in shared profiles");
    if (legacyProfile != NULL)
    {
        ok &= expectTrue(
            legacyProfile->id == "chufengkouxiao",
            "legacy type 4 should map to chufengkouxiao");
        ok &= expectTrue(
            legacyProfile->robotSimModelId == "chufengkouxiao",
            "shared profile should include robotsim model id");
        ok &= expectTrue(
            legacyProfile->calibrationFile == "config/calibration/SENSORPARA.txt",
            "shared profile should include calibration file");
        ok &= expectTrue(
            legacyProfile->localizationRoiExpandPx == 250,
            "shared profile should include localization ROI");
        ok &= expectTrue(
            legacyProfile->localizationIcpDof == "x,y,z,rz",
            "shared profile should include ICP DOF mask text");
        ok &= expectTrue(
            legacyProfile->hasLvsFusionThresholds,
            "shared profile should include LVS fusion thresholds");
    }
    return ok;
}

bool testFallbackReaders()
{
    const std::string missingPath = repoConfigPath("missing_ibvs_config.txt");
    const IbvsRuntimeTextConfigResult runtime =
        IbvsTextConfigReader::readRuntimeDefaultsWithFallback(
            missingPath,
            repoConfigPath("ibvs_runtime_defaults.txt"));
    const IbvsWorkpieceProfilesResult profiles =
        IbvsTextConfigReader::readWorkpieceProfilesWithFallback(
            missingPath,
            repoConfigPath("workpiece_profiles.txt"));
    const IbvsWorkpieceProfile* profile =
        IbvsTextConfigReader::findProfileForLegacyType(profiles.profiles, 4);

    bool ok = true;
    ok &= expectTrue(runtime.error.isOk(), "runtime fallback should load");
    ok &= expectTrue(profiles.error.isOk(), "profile fallback should load");
    ok &= expectTrue(profile != NULL, "fallback profiles should contain workpiece type 4");
    return ok;
}

} // namespace

/**
 * @brief IBVS txt 配置读取 smoke 测试入口。
 */
int main()
{
    bool ok = true;
    ok &= testRuntimeDefaults();
    ok &= testWorkpieceProfiles();
    ok &= testSharedWorkpieceProfiles();
    ok &= testFallbackReaders();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: IBVS text config reader tests\n";
    return 0;
}
