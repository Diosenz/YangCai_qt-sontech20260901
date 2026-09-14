/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_online_service.cpp
 * @brief       ibvs online service 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include "ibvs/IbvsOnlineService.h"

namespace {

using namespace i2r_rlc::ibvs;

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

IbvsServiceConfig makeConfig()
{
    IbvsServiceConfig config;
    config.runtimeDefaultsPath = repoConfigPath("ibvs_runtime_defaults.txt");
    config.workpieceProfilesPath = repoWorkpieceConfigPath("workpiece_profiles.txt");
    config.workpieceId = "chufengkouxiao";
    config.workpieceType = -1;
    return config;
}

IbvsOnlineFrameInput makeFrameInput()
{
    IbvsTorchObservation torch;
    torch.featurePoints = { { 40.0, 20.0 }, { 16.0, 20.0 } };
    torch.confidence = 0.9;
    torch.frameIndex = 10;
    torch.depthM = -1.0;

    IbvsLvsObservation lvs;
    lvs.startPoint = { 10.0, 10.0 };
    lvs.hasStartPoint = true;
    lvs.confidence = 0.9;
    lvs.centerlineQuality = 0.8;
    lvs.frameIndex = 10;

    IbvsLvsConstraint lvsConstraint;
    lvsConstraint.enabled = true;
    lvsConstraint.jacobian =
        IbvsUnifiedVelocitySolver::computeLvsJacobian(Eigen::Vector3d::UnitX());
    lvsConstraint.errorM << 0.0, 0.003, -0.002;
    lvsConstraint.lambdaD = 1.0;
    lvsConstraint.alphaLvs = 1.0;
    lvsConstraint.omegaLvs = 1.0;
    lvsConstraint.betaLvs = 1.0;

    IbvsPositionConstraint position;
    position.enabled = true;
    position.currentXMm = 905.3;
    position.currentYMm = -18.84;
    position.currentZMm = -253.6;
    position.xMinMm = 905.2;
    position.xMaxMm = 905.4;
    position.yMinMm = -18.94;
    position.yMaxMm = -18.74;
    position.zMinMm = -253.7;
    position.zMaxMm = -253.5;

    IbvsOnlineFrameInput input;
    input.frameIndex = 10;
    input.currentFeature = torch.featurePoints.toVector();
    input.fallbackDesiredFeature = IbvsFeatureVector::Zero();
    input.cameraJacobian << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0;
    input.torchObservation = torch;
    input.lvsObservation = lvs;
    input.lvsConstraint = lvsConstraint;
    input.positionConstraint = position;
    return input;
}

bool testConfigureAndDryRun()
{
    IbvsOnlineService service;
    const IbvsStartResult configured = service.configure(makeConfig());
    const IbvsStartResult started = service.startDryRun();
    const IbvsOnlineStepResult step = service.processFrame(makeFrameInput());

    bool ok = true;
    ok &= expectTrue(
        configured.state == IbvsServiceState::Configured,
        "configure should enter Configured state");
    ok &= expectTrue(
        started.state == IbvsServiceState::RunningDryRun,
        "startDryRun should enter RunningDryRun state");
    ok &= expectTrue(step.error.isOk(), "dry-run step should succeed");
    ok &= expectTrue(step.velocityMps.allFinite(), "dry-run velocity should be finite");
    ok &= expectTrue(step.rowCount == 7, "dry-run step should use image and LVS rows");
    ok &= expectTrue(step.usedLvsPrior, "dry-run step should use LVS prior");
    ok &= expectTrue(step.usedLvsConstraint, "dry-run step should use LVS constraint");
    ok &= expectTrue(
        !step.usedPositionConstraint,
        "in-range position should not append position rows");
    ok &= expectTrue(
        step.dryRunCommand.find("DRYRUN IBVS_VELOCITY_MPS") == 0,
        "dry-run command should be a command draft");
    return ok;
}

bool testInvalidTorchFailsCleanly()
{
    IbvsOnlineService service;
    service.configure(makeConfig());
    service.startDryRun();

    IbvsOnlineFrameInput input = makeFrameInput();
    input.torchObservation->featurePoints.second =
        input.torchObservation->featurePoints.first;
    input.currentFeature = input.torchObservation->featurePoints.toVector();
    const IbvsOnlineStepResult step = service.processFrame(input);

    return expectTrue(
        step.error.status == IbvsStatus::INVALID_ARGUMENT,
        "invalid torch observation should fail without solving velocity");
}

bool testStopIsIdempotentAndBlocksFrames()
{
    IbvsOnlineService service;
    service.configure(makeConfig());
    service.startDryRun();
    const IbvsStartResult firstStop = service.stop();
    const IbvsStartResult secondStop = service.stop();
    const IbvsOnlineStepResult step = service.processFrame(makeFrameInput());

    bool ok = true;
    ok &= expectTrue(
        firstStop.state == IbvsServiceState::DryRunReady,
        "first stop should return to DryRunReady");
    ok &= expectTrue(
        secondStop.state == IbvsServiceState::DryRunReady,
        "second stop should be idempotent");
    ok &= expectTrue(
        step.error.status == IbvsStatus::RUNTIME_ERROR,
        "stopped service should reject frames");
    return ok;
}

bool testInvalidInputFails()
{
    IbvsOnlineService service;
    service.configure(makeConfig());
    service.startDryRun();

    IbvsOnlineFrameInput input = makeFrameInput();
    input.currentFeature(0) = std::numeric_limits<double>::quiet_NaN();
    const IbvsOnlineStepResult step = service.processFrame(input);

    return expectTrue(
        step.error.status == IbvsStatus::INVALID_ARGUMENT,
        "invalid numeric input should fail cleanly");
}

} // namespace

/**
 * @brief IBVS online dry-run 服务测试入口。
 */
int main()
{
    bool ok = true;
    ok &= testConfigureAndDryRun();
    ok &= testInvalidTorchFailsCleanly();
    ok &= testStopIsIdempotentAndBlocksFrames();
    ok &= testInvalidInputFails();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: IBVS online service tests\n";
    return 0;
}
