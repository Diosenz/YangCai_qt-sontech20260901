/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_sensor_fusion.cpp
 * @brief       ibvs sensor fusion 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include "ibvs/IbvsFeatureGeometry.h"
#include "ibvs/IbvsSensorFusion.h"
#include "ibvs/IbvsVelocitySolver.h"

namespace {

using namespace i2r_rlc::ibvs;

constexpr double kTolerance = 1e-9;

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool expectVectorNear(
    const Eigen::VectorXd& actual,
    const Eigen::VectorXd& expected,
    const double tolerance,
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

bool containsText(const std::string& value, const std::string& needle)
{
    return value.find(needle) != std::string::npos;
}

IbvsFusionConfig makeConfig()
{
    IbvsFusionConfig config;
    config.minTorchConfidence = 0.2;
    config.minLvsConfidence = 0.3;
    config.minLvsCenterlineQuality = 0.4;
    config.maxFrameDelta = 2;
    config.preferLvs = true;
    config.lvsChannelSigmaScale = 0.5;
    config.fallbackChannelSigmaScale = 2.0;
    config.baseChannelSigmasPx << 1.0, 2.0, 3.0, 4.0;
    config.geometryParameters.h1Mm = 30.0;
    config.geometryParameters.h2Mm = 24.0;
    config.geometryParameters.useDepthBasedH2 = false;
    config.geometryParameters.minFeatureDistancePx = 1.0;
    config.geometryParameters.useOverrideH2Px = false;
    config.geometryParameters.useOverrideUnitDirection = false;
    return config;
}

IbvsTorchObservation makeTorchObservation()
{
    IbvsTorchObservation observation;
    observation.featurePoints = { { 40.0, 20.0 }, { 16.0, 20.0 } };
    observation.confidence = 0.9;
    observation.frameIndex = 10;
    observation.depthM = -1.0;
    return observation;
}

IbvsLvsObservation makeLvsObservation()
{
    IbvsLvsObservation observation;
    observation.startPoint = { 10.0, 10.0 };
    observation.weldPoint = { 11.0, 12.0 };
    observation.hasStartPoint = true;
    observation.hasWeldPoint = true;
    observation.confidence = 0.9;
    observation.centerlineQuality = 0.8;
    observation.frameIndex = 10;
    return observation;
}

IbvsFeatureVector makeFallbackDesiredFeature()
{
    IbvsFeatureVector feature;
    feature << 100.0, 101.0, 102.0, 103.0;
    return feature;
}

bool testValidLvsUsesGeometry()
{
    const IbvsFusionConfig config = makeConfig();
    const IbvsTorchObservation torch = makeTorchObservation();
    const IbvsLvsObservation lvs = makeLvsObservation();
    const IbvsFeatureVector fallback = makeFallbackDesiredFeature();
    const IbvsFeatureGeometryResult expected = computeDesiredFeature(
        torch.featurePoints,
        lvs.startPoint,
        torch.depthM,
        config.geometryParameters);

    const IbvsFusionResult result = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        lvs,
        fallback,
        config);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "valid LVS fusion should succeed");
    ok &= expectTrue(result.usedLvs, "valid LVS fusion should use LVS");
    ok &= expectTrue(!result.usedFallback, "valid LVS fusion should not fallback");
    ok &= expectVectorNear(
        result.desiredFeature,
        expected.desiredFeature,
        kTolerance,
        "LVS desired feature should match computeDesiredFeature");
    ok &= expectVectorNear(
        result.channelSigmasPx,
        (Eigen::Vector4d() << 0.5, 1.0, 1.5, 2.0).finished(),
        kTolerance,
        "LVS channel sigmas should be scaled");
    return ok;
}

bool testLowConfidenceFallsBack()
{
    const IbvsFusionConfig config = makeConfig();
    const IbvsTorchObservation torch = makeTorchObservation();
    IbvsLvsObservation lvs = makeLvsObservation();
    lvs.confidence = 0.1;
    const IbvsFeatureVector fallback = makeFallbackDesiredFeature();

    const IbvsFusionResult result = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        lvs,
        fallback,
        config);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "low confidence LVS should not fail");
    ok &= expectTrue(!result.usedLvs, "low confidence LVS should not be used");
    ok &= expectTrue(result.usedFallback, "low confidence LVS should fallback");
    ok &= expectVectorNear(
        result.desiredFeature,
        fallback,
        kTolerance,
        "fallback desired feature should be preserved");
    ok &= expectTrue(
        containsText(result.fallbackReason, "confidence"),
        "fallback reason should mention confidence");
    ok &= expectVectorNear(
        result.channelSigmasPx,
        (Eigen::Vector4d() << 2.0, 4.0, 6.0, 8.0).finished(),
        kTolerance,
        "fallback channel sigmas should be scaled");
    return ok;
}

bool testInvalidTorchFails()
{
    const IbvsFusionConfig config = makeConfig();
    IbvsTorchObservation torch = makeTorchObservation();
    torch.featurePoints = { { 10.0, 10.0 }, { 10.0, 10.0 } };
    const IbvsLvsObservation lvs = makeLvsObservation();

    const IbvsFusionResult result = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        lvs,
        makeFallbackDesiredFeature(),
        config);

    bool ok = true;
    ok &= expectTrue(
        result.error.status == IbvsStatus::INVALID_ARGUMENT,
        "invalid torch observation should return INVALID_ARGUMENT");
    ok &= expectTrue(!result.usedLvs, "invalid torch should not use LVS");
    ok &= expectTrue(!result.usedFallback, "invalid torch should not fallback");
    return ok;
}

bool testStaleLvsFallsBack()
{
    const IbvsFusionConfig config = makeConfig();
    const IbvsTorchObservation torch = makeTorchObservation();
    IbvsLvsObservation lvs = makeLvsObservation();
    lvs.frameIndex = 4;
    const IbvsFeatureVector fallback = makeFallbackDesiredFeature();

    const IbvsFusionResult result = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        lvs,
        fallback,
        config);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "stale LVS should not fail");
    ok &= expectTrue(!result.usedLvs, "stale LVS should not be used");
    ok &= expectTrue(result.usedFallback, "stale LVS should fallback");
    ok &= expectVectorNear(
        result.desiredFeature,
        fallback,
        kTolerance,
        "stale LVS should preserve fallback desired feature");
    ok &= expectTrue(
        containsText(result.fallbackReason, "stale"),
        "fallback reason should mention stale LVS");
    return ok;
}

bool testWeldPointCanProvideLvsPrior()
{
    const IbvsFusionConfig config = makeConfig();
    const IbvsTorchObservation torch = makeTorchObservation();
    IbvsLvsObservation lvs = makeLvsObservation();
    lvs.hasStartPoint = false;
    const IbvsFeatureGeometryResult expected = computeDesiredFeature(
        torch.featurePoints,
        lvs.weldPoint,
        torch.depthM,
        config.geometryParameters);

    const IbvsFusionResult result = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        lvs,
        makeFallbackDesiredFeature(),
        config);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "weld point LVS prior should succeed");
    ok &= expectTrue(result.usedLvs, "weld point prior should use LVS");
    ok &= expectVectorNear(
        result.desiredFeature,
        expected.desiredFeature,
        kTolerance,
        "weld point prior should feed computeDesiredFeature");
    return ok;
}

bool testFusionFeedsVelocitySolver()
{
    const IbvsFusionConfig config = makeConfig();
    const IbvsTorchObservation torch = makeTorchObservation();
    const IbvsFusionResult fusion = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        makeLvsObservation(),
        makeFallbackDesiredFeature(),
        config);

    IbvsControlParameters control;
    control.desiredFeature = fusion.desiredFeature;
    control.useChannelwiseR = true;
    control.channelSigmasPx = fusion.channelSigmasPx;
    control.lambda = 0.5;
    control.maxCameraVelocityMps = 0.02;

    IbvsJacobianMatrix jacobian;
    jacobian << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0;

    const IbvsFeatureVector currentFeature = torch.featurePoints.toVector();
    const IbvsVelocitySolveResult velocity =
        IbvsVelocitySolver(control).computeCameraVelocity(
            jacobian,
            currentFeature);

    bool ok = true;
    ok &= expectTrue(fusion.error.isOk(), "fusion should succeed before velocity solve");
    ok &= expectTrue(velocity.error.isOk(), "velocity solve should consume fused feature");
    ok &= expectTrue(
        velocity.velocityMps.allFinite(),
        "velocity result should be finite");
    ok &= expectTrue(
        velocity.velocityMps.norm() > 0.0,
        "velocity should be non-zero for non-zero error");
    ok &= expectTrue(
        velocity.velocityLimited,
        "large fused feature error should trigger velocity limit");
    ok &= expectVectorNear(
        velocity.featureErrorPx,
        currentFeature - fusion.desiredFeature,
        kTolerance,
        "feature error should be current minus fused desired feature");
    return ok;
}

bool testInvalidFallbackIsReported()
{
    const IbvsFusionConfig config = makeConfig();
    const IbvsTorchObservation torch = makeTorchObservation();
    IbvsFeatureVector fallback = makeFallbackDesiredFeature();
    fallback(0) = std::numeric_limits<double>::quiet_NaN();

    const IbvsFusionResult result = IbvsSensorFusion::fuseDesiredFeature(
        torch,
        std::nullopt,
        fallback,
        config);

    return expectTrue(
        result.error.status == IbvsStatus::INVALID_ARGUMENT,
        "invalid fallback desired feature should be reported");
}

} // namespace

/**
 * @brief IBVS/LVS 传感融合纯算法 smoke 测试入口。
 */
int main()
{
    bool ok = true;
    ok &= testValidLvsUsesGeometry();
    ok &= testLowConfidenceFallsBack();
    ok &= testInvalidTorchFails();
    ok &= testStaleLvsFallsBack();
    ok &= testWeldPointCanProvideLvsPrior();
    ok &= testFusionFeedsVelocitySolver();
    ok &= testInvalidFallbackIsReported();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: IBVS sensor fusion tests\n";
    return 0;
}
