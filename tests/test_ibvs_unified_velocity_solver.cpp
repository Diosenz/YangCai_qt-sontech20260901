/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_unified_velocity_solver.cpp
 * @brief       ibvs unified velocity solver 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "ibvs/IbvsFeatureGeometry.h"
#include "ibvs/IbvsUnifiedVelocitySolver.h"
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

bool expectMatrixNear(
    const Eigen::MatrixXd& actual,
    const Eigen::MatrixXd& expected,
    const double tolerance,
    const char* message)
{
    if ((actual - expected).norm() > tolerance) {
        std::cerr << "FAIL: " << message
                  << ", actual=\n" << actual
                  << "\nexpected=\n" << expected << '\n';
        return false;
    }

    return true;
}

IbvsJacobianMatrix makeCameraJacobian()
{
    IbvsJacobianMatrix jacobian;
    jacobian << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0;
    return jacobian;
}

IbvsFeatureVector makeCurrentFeature()
{
    IbvsFeatureVector feature;
    feature << 5.0, -3.0, 2.0, 1.0;
    return feature;
}

IbvsLvsConstraint makeLvsConstraint()
{
    IbvsLvsConstraint constraint;
    constraint.enabled = true;
    constraint.jacobian =
        IbvsUnifiedVelocitySolver::computeLvsJacobian(
            Eigen::Vector3d::UnitX());
    constraint.errorM << 0.0, 0.003, -0.002;
    constraint.lambdaD = 2.0;
    constraint.alphaLvs = 1.0;
    constraint.omegaLvs = 1.0;
    constraint.betaLvs = 1.0;
    constraint.useAxisWeights = true;
    constraint.axisWeights << 1.0, 1.5, 2.0;
    return constraint;
}

IbvsUnifiedVelocityConfig makeConfig()
{
    IbvsUnifiedVelocityConfig config;
    config.lambdaF = 0.5;
    config.dampingKappa = 0.2;
    config.minDamping = 1e-4;
    config.maxVelocityMps = 0.05;
    config.lvsTaskScaleMin = 200.0;
    config.lvsTaskScaleMax = 5000.0;
    return config;
}

bool testSeamErrorAndJacobian()
{
    const IbvsSeamLineInfo invalidSeam;
    const Eigen::Vector3d invalidError =
        IbvsUnifiedVelocitySolver::computeSeamError(
            Eigen::Vector3d(1.0, 2.0, 3.0),
            invalidSeam);

    IbvsSeamLineInfo seam;
    seam.valid = true;
    seam.p0M << 1.0, 0.0, 0.0;
    seam.direction << 1.0, 0.0, 0.0;
    const Eigen::Vector3d point(1.0, 2.0, -3.0);
    const Eigen::Vector3d error =
        IbvsUnifiedVelocitySolver::computeSeamError(point, seam);
    const Eigen::Matrix3d jacobian =
        IbvsUnifiedVelocitySolver::computeLvsJacobian(
            Eigen::Vector3d(2.0, 0.0, 0.0));

    bool ok = true;
    ok &= expectVectorNear(
        invalidError,
        Eigen::Vector3d::Zero(),
        kTolerance,
        "invalid seam should produce zero error");
    ok &= expectVectorNear(
        error,
        (Eigen::Vector3d() << 0.0, 2.0, -3.0).finished(),
        kTolerance,
        "seam error should project perpendicular to direction");
    ok &= expectVectorNear(
        jacobian * Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::Zero(),
        kTolerance,
        "LVS jacobian should null seam direction");
    ok &= expectMatrixNear(
        jacobian,
        jacobian.transpose(),
        kTolerance,
        "LVS jacobian should be symmetric");
    ok &= expectTrue(jacobian.allFinite(), "LVS jacobian should be finite");
    return ok;
}

bool testImageOnlyMatchesVelocitySolverDirection()
{
    const IbvsJacobianMatrix jacobian = makeCameraJacobian();
    const IbvsFeatureVector current = makeCurrentFeature();
    const IbvsFeatureVector desired = IbvsFeatureVector::Zero();
    IbvsUnifiedVelocityConfig config = makeConfig();

    IbvsControlParameters control;
    control.lambda = config.lambdaF;
    control.dampingKappa = config.dampingKappa;
    control.minDamping = config.minDamping;
    control.maxCameraVelocityMps = config.maxVelocityMps;
    control.desiredFeature = desired;

    const IbvsVelocitySolveResult base =
        IbvsVelocitySolver(control).computeCameraVelocity(jacobian, current);
    const IbvsUnifiedVelocityResult unified =
        IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
            jacobian,
            current,
            desired,
            IbvsLvsConstraint(),
            IbvsPositionConstraint(),
            config);

    bool ok = true;
    ok &= expectTrue(base.error.isOk(), "base velocity solve should succeed");
    ok &= expectTrue(unified.error.isOk(), "unified image-only solve should succeed");
    ok &= expectTrue(unified.rowCount == 4, "image-only solve should use four rows");
    ok &= expectTrue(!unified.usedLvs, "image-only solve should not use LVS");
    ok &= expectTrue(
        unified.velocityMps.allFinite(),
        "unified image-only velocity should be finite");
    ok &= expectTrue(
        unified.velocityMps.dot(base.velocityMps) > 0.0,
        "unified image-only velocity should follow base solver direction");
    ok &= expectNear(
        unified.velocityMps.norm(),
        base.velocityMps.norm(),
        1e-8,
        "unified image-only velocity norm should match base solver");
    return ok;
}

bool testSevenRowLvsSolve()
{
    const IbvsUnifiedVelocityResult result =
        IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
            makeCameraJacobian(),
            makeCurrentFeature(),
            IbvsFeatureVector::Zero(),
            makeLvsConstraint(),
            IbvsPositionConstraint(),
            makeConfig());

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "LVS unified solve should succeed");
    ok &= expectTrue(result.usedLvs, "LVS unified solve should use LVS");
    ok &= expectTrue(result.rowCount == 7, "LVS unified solve should stack seven rows");
    ok &= expectTrue(result.velocityMps.allFinite(), "LVS velocity should be finite");
    ok &= expectTrue(std::isfinite(result.camResidualFinalNorm), "camera residual should be finite");
    ok &= expectTrue(std::isfinite(result.lvsResidualFinalNorm), "LVS residual should be finite");
    ok &= expectTrue(result.lvsTaskScale >= 200.0, "LVS scale should use lower clamp");
    return ok;
}

bool testZeroLvsWeightFallsBackToImageTask()
{
    IbvsLvsConstraint lvs = makeLvsConstraint();
    lvs.omegaLvs = 0.0;

    const IbvsUnifiedVelocityResult result =
        IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
            makeCameraJacobian(),
            makeCurrentFeature(),
            IbvsFeatureVector::Zero(),
            lvs,
            IbvsPositionConstraint(),
            makeConfig());

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "zero-weight LVS solve should succeed");
    ok &= expectTrue(!result.usedLvs, "zero-weight LVS should not be applied");
    ok &= expectTrue(result.rowCount == 4, "zero-weight LVS should keep image rows only");
    return ok;
}

bool testPositionRowsAndDirection()
{
    IbvsPositionConstraint position;
    position.enabled = true;
    position.currentXMm = -1.0;
    position.currentYMm = 2.0;
    position.currentZMm = 0.0;
    position.xMinMm = 0.0;
    position.xMaxMm = 1.0;
    position.yMinMm = -1.0;
    position.yMaxMm = 1.0;
    position.zMinMm = -1.0;
    position.zMaxMm = 1.0;
    position.lambdaPos = 5.0;
    position.omegaPos = 1000.0;

    IbvsUnifiedVelocityConfig config = makeConfig();
    config.lambdaF = 0.0;
    config.maxVelocityMps = 1.0;

    const IbvsUnifiedVelocityResult result =
        IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
            makeCameraJacobian(),
            IbvsFeatureVector::Zero(),
            IbvsFeatureVector::Zero(),
            IbvsLvsConstraint(),
            position,
            config);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "position constrained solve should succeed");
    ok &= expectTrue(result.usedPositionConstraint, "position rows should be active");
    ok &= expectTrue(result.rowCount == 6, "two out-of-range axes should add two rows");
    ok &= expectTrue(
        result.velocityMps(0) > 0.0,
        "below x minimum should request positive x velocity");
    ok &= expectTrue(
        result.velocityMps(1) < 0.0,
        "above y maximum should request negative y velocity");

    position.currentZMm = 2.0;
    const IbvsUnifiedVelocityResult threeAxisResult =
        IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
            makeCameraJacobian(),
            IbvsFeatureVector::Zero(),
            IbvsFeatureVector::Zero(),
            makeLvsConstraint(),
            position,
            config);
    ok &= expectTrue(
        threeAxisResult.rowCount == 10,
        "LVS plus three position axes should stack ten rows");
    return ok;
}

bool testPositionPreset()
{
    const IbvsPositionConstraint constraint =
        IbvsUnifiedVelocitySolver::makePositionConstraintForWorkpiece(4);

    bool ok = true;
    ok &= expectTrue(constraint.enabled, "workpiece preset should be enabled");
    ok &= expectNear(constraint.xMinMm, 905.2, 1e-9, "workpiece 4 x min should match legacy");
    ok &= expectNear(constraint.xMaxMm, 905.4, 1e-9, "workpiece 4 x max should match legacy");
    ok &= expectNear(constraint.yMinMm, -18.94, 1e-9, "workpiece 4 y min should match legacy");
    ok &= expectNear(constraint.yMaxMm, -18.74, 1e-9, "workpiece 4 y max should match legacy");
    ok &= expectNear(constraint.zMinMm, -253.7, 1e-9, "workpiece 4 z min should match legacy");
    ok &= expectNear(constraint.zMaxMm, -253.5, 1e-9, "workpiece 4 z max should match legacy");
    ok &= expectNear(constraint.lambdaPos, 2.0, 1e-12, "workpiece lambda should match legacy");
    return ok;
}

bool testLegacyFeatureGeometryPreset()
{
    const PixelFeaturePair featurePoints{ { 40.0, 20.0 }, { 16.0, 20.0 } };
    const PixelPoint startPoint{ 10.0, 10.0 };
    const IbvsFeatureGeometryParameters legacy =
        makeLegacyWorkpieceFeatureGeometryParameters();
    const IbvsFeatureGeometryResult result = computeDesiredFeature(
        featurePoints,
        startPoint,
        -1.0,
        legacy);

    const double directionNorm =
        std::sqrt(0.7513 * 0.7513 + 0.6599 * 0.6599);
    const double unitU = -0.7513 / directionNorm;
    const double unitV = 0.6599 / directionNorm;
    const double h2Px = 38.6409;
    const double h1Px = h2Px * (21.0 / 24.0);
    const IbvsFeatureVector expected =
        (IbvsFeatureVector()
            << startPoint.u + unitU * (h1Px + h2Px),
               startPoint.v + unitV * (h1Px + h2Px),
               startPoint.u + unitU * h1Px,
               startPoint.v + unitV * h1Px).finished();

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "legacy geometry preset should succeed");
    ok &= expectNear(result.h1Px, h1Px, kTolerance, "legacy h1 should follow H1/H2");
    ok &= expectNear(result.h2Px, h2Px, kTolerance, "legacy h2 should use override");
    ok &= expectVectorNear(
        result.desiredFeature,
        expected,
        kTolerance,
        "legacy desired feature should use hand-tuned direction");

    IbvsFeatureGeometryParameters automatic;
    automatic.h1Mm = 30.0;
    automatic.h2Mm = 24.0;
    automatic.useOverrideH2Px = false;
    automatic.useOverrideUnitDirection = false;
    const IbvsFeatureGeometryResult automaticResult = computeDesiredFeature(
        featurePoints,
        startPoint,
        -1.0,
        automatic);
    ok &= expectTrue(automaticResult.error.isOk(), "automatic geometry should still succeed");
    ok &= expectNear(
        automaticResult.h2Px,
        featurePoints.distancePx(),
        kTolerance,
        "automatic geometry should still use feature distance");
    return ok;
}

} // namespace

/**
 * @brief IBVS/LVS 统一速度求解纯算法 smoke 测试入口。
 */
int main()
{
    bool ok = true;
    ok &= testSeamErrorAndJacobian();
    ok &= testImageOnlyMatchesVelocitySolverDirection();
    ok &= testSevenRowLvsSolve();
    ok &= testZeroLvsWeightFallsBackToImageTask();
    ok &= testPositionRowsAndDirection();
    ok &= testPositionPreset();
    ok &= testLegacyFeatureGeometryPreset();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: IBVS unified velocity solver tests\n";
    return 0;
}
