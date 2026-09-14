/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_core.cpp
 * @brief       ibvs core 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ibvs/IbvsFeatureGeometry.h"
#include "ibvs/IbvsJacobianEstimator.h"
#include "ibvs/IbvsOfflineDataReader.h"
#include "ibvs/IbvsStartPointDetector.h"
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

bool expectNear(double actual, double expected, const char* message)
{
    if (std::abs(actual - expected) > kTolerance) {
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

bool expectJacobianNear(
    const IbvsJacobianMatrix& actual,
    const IbvsJacobianMatrix& expected,
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

bool writeTextFile(const std::filesystem::path& path, const std::string& content)
{
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "FAIL: cannot write file " << path.string() << '\n';
        return false;
    }

    file << content;
    return true;
}

IbvsJacobianMatrix makeReferenceJacobian()
{
    IbvsJacobianMatrix jacobian;
    jacobian << 10.0, 1.0, -2.0,
        0.5, 8.0, 1.5,
        9.0, -1.0, 2.0,
        -0.5, 7.0, 3.0;
    return jacobian;
}

std::vector<Eigen::Vector3d> makeMotionSamples()
{
    return {
        Eigen::Vector3d(0.001, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.001, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.001),
        Eigen::Vector3d(0.001, 0.001, 0.0),
        Eigen::Vector3d(0.001, 0.0, 0.001),
        Eigen::Vector3d(0.0, 0.001, 0.001)
    };
}

std::vector<Eigen::Vector4d> makeFeatureSamples(
    const IbvsJacobianMatrix& jacobian,
    const std::vector<Eigen::Vector3d>& motions)
{
    std::vector<Eigen::Vector4d> features;
    features.reserve(motions.size());
    for (const Eigen::Vector3d& motion : motions) {
        features.push_back(jacobian * motion);
    }
    return features;
}

bool testTypes()
{
    bool ok = true;
    const PixelPoint point{ 12.0, 34.0 };
    const PixelRoi roi{ 1, 2, 30, 40 };
    const PixelFeaturePair pair{ { 0.0, 0.0 }, { 3.0, 4.0 } };
    const IbvsCameraIntrinsics intrinsics{ 100.0, 120.0, 50.0, 60.0 };

    ok &= expectTrue(point.isFinite(), "pixel point should be finite");
    ok &= expectTrue(roi.isValid(), "ROI should be valid");
    ok &= expectNear(pair.distancePx(), 5.0, "feature pair distance should match");
    ok &= expectTrue(pair.isValid(1.0), "feature pair should be valid");
    ok &= expectVectorNear(
        pair.toVector(),
        (IbvsFeatureVector() << 0.0, 0.0, 3.0, 4.0).finished(),
        kTolerance,
        "feature vector should preserve point order");
    ok &= expectTrue(intrinsics.isValid(), "camera intrinsics should be valid");
    ok &= expectNear(intrinsics.toMatrix()(0, 0), 100.0, "fx should map into K");
    return ok;
}

bool testFeatureGeometry()
{
    IbvsFeatureGeometryParameters parameters;
    parameters.h1Mm = 30.0;
    parameters.h2Mm = 24.0;
    parameters.useOverrideH2Px = false;
    parameters.useOverrideUnitDirection = false;

    const PixelFeaturePair current{ { 40.0, 20.0 }, { 16.0, 20.0 } };
    const PixelPoint start{ 10.0, 10.0 };
    const auto result = computeDesiredFeature(
        current,
        start,
        -1.0,
        parameters);
    const auto invalidStartResult = computeDesiredFeature(
        current,
        PixelPoint{ std::numeric_limits<double>::infinity(), 10.0 },
        -1.0,
        parameters);

    bool ok = true;
    ok &= expectTrue(result.error.isOk(), "feature geometry should succeed");
    ok &= expectNear(result.h2Px, 24.0, "h2 should use current feature distance");
    ok &= expectNear(result.h1Px, 30.0, "h1 should follow H1/H2 ratio");
    ok &= expectNear(result.desiredPoints.second.u, 40.0, "p2* u should match");
    ok &= expectNear(result.desiredPoints.second.v, 10.0, "p2* v should match");
    ok &= expectNear(result.desiredPoints.first.u, 64.0, "p1* u should match");
    ok &= expectNear(result.desiredPoints.first.v, 10.0, "p1* v should match");
    ok &= expectTrue(
        invalidStartResult.error.status == IbvsStatus::DEGENERATE_GEOMETRY,
        "non-finite start point should return structured geometry error");
    return ok;
}

bool testStartPointDetector()
{
    const std::vector<PixelPoint> linePoints{
        { 0.0, 0.0 },
        { 10.0, 0.0 },
        { 20.0, 0.0 }
    };

    const auto endpoint =
        IbvsStartPointDetector::computeEndpointStartPoint(linePoints, true);
    const IbvsLineModel verticalEdge{ 1.0, 0.0, -5.0, true };
    const auto intersection =
        IbvsStartPointDetector::computeIntersectionStartPoint(
            endpoint.line,
            verticalEdge);
    const auto nearest =
        IbvsStartPointDetector::computeCurveNearestStartPoint(
            linePoints,
            PixelPoint{ 18.0, 1.0 });

    bool ok = true;
    ok &= expectTrue(endpoint.error.isOk(), "endpoint start point should succeed");
    ok &= expectNear(endpoint.startPoint.u, 0.0, "left endpoint should be selected");
    ok &= expectNear(endpoint.endPoint.u, 20.0, "right endpoint should be end point");
    ok &= expectTrue(intersection.error.isOk(), "intersection start point should succeed");
    ok &= expectNear(intersection.startPoint.u, 5.0, "intersection u should match");
    ok &= expectNear(intersection.startPoint.v, 0.0, "intersection v should match");
    ok &= expectTrue(nearest.error.isOk(), "curve nearest start point should succeed");
    ok &= expectNear(nearest.startPoint.u, 20.0, "nearest curve point should match");
    return ok;
}

bool testJacobianEstimator()
{
    const IbvsJacobianMatrix referenceJacobian = makeReferenceJacobian();
    const auto motions = makeMotionSamples();
    const auto featureSamples = makeFeatureSamples(referenceJacobian, motions);

    IbvsJacobianEstimator estimator;
    const auto ols = estimator.computeInitialJacobianOLS(
        motions,
        featureSamples,
        0.0);

    bool ok = true;
    ok &= expectTrue(ols.error.isOk(), "OLS should succeed");
    ok &= expectJacobianNear(
        ols.jacobian,
        referenceJacobian,
        1e-7,
        "OLS should recover reference jacobian");

    estimator.kfInitWithJ0(referenceJacobian);
    ok &= expectJacobianNear(
        estimator.getCurrentJacobian(),
        referenceJacobian,
        kTolerance,
        "kfInitWithJ0 should store current jacobian");

    const auto skipped = estimator.jacobianEK(
        Eigen::Vector3d::Zero(),
        Eigen::Vector4d::Zero());
    ok &= expectTrue(skipped.error.isOk(), "small motion update should succeed");
    ok &= expectTrue(skipped.skippedBySmallMotion, "small motion should be skipped");

    const auto updated = estimator.jacobianEK(
        motions.front(),
        referenceJacobian * motions.front());
    ok &= expectTrue(updated.error.isOk(), "normal KF update should succeed");
    ok &= expectTrue(updated.updated, "normal KF update should report updated");

    const IbvsCameraIntrinsics intrinsics{ 100.0, 120.0, 50.0, 60.0 };
    const PixelFeaturePair featurePoints{ { 60.0, 70.0 }, { 80.0, 90.0 } };
    const IbvsHandEyeCalibration handEye;
    const auto linear = estimator.jacobianLinear(
        2.0,
        featurePoints,
        intrinsics,
        handEye);
    ok &= expectTrue(linear.norm() > 1e-9, "linear jacobian should be non-zero");
    return ok;
}

bool testVelocitySolver()
{
    IbvsControlParameters parameters;
    parameters.desiredFeature = IbvsFeatureVector::Zero();
    parameters.maxCameraVelocityMps = 0.01;

    IbvsVelocitySolver solver(parameters);
    IbvsFeatureVector current;
    current << 100.0, 0.0, 0.0, 0.0;

    bool ok = true;
    const auto zeroResult = solver.computeCameraVelocity(
        IbvsJacobianMatrix::Zero(),
        current);
    ok &= expectTrue(
        zeroResult.error.status == IbvsStatus::SINGULAR_MATRIX,
        "zero jacobian should be rejected");
    ok &= expectNear(
        zeroResult.velocityMps.norm(),
        0.0,
        "zero jacobian should return zero velocity");

    IbvsJacobianMatrix jacobian;
    jacobian << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0;
    const auto velocityResult = solver.computeCameraVelocity(jacobian, current);
    ok &= expectTrue(velocityResult.error.isOk(), "velocity solve should succeed");
    ok &= expectTrue(
        velocityResult.velocityMps.norm() > 0.0,
        "velocity should be non-zero");
    ok &= expectTrue(
        velocityResult.velocityLimited,
        "large error should trigger velocity limit");
    ok &= expectNear(
        velocityResult.velocityMps.norm(),
        parameters.maxCameraVelocityMps,
        "velocity norm should be limited");

    IbvsFeatureVector smallFeature;
    smallFeature << 1e-4, 0.0, 0.0, 0.0;
    ok &= expectTrue(solver.isConverged(smallFeature), "small error should converge");
    return ok;
}

bool testOfflineDataReader()
{
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "i2r_rlc_ibvs_core_tests";
    std::filesystem::create_directories(dir);

    const auto featurePath = dir / "Feature_coordinates.txt";
    const auto robotPath = dir / "IBVSRobotData.txt";
    const auto cameraPath = dir / "Camera_Position.txt";
    const auto jacobianPath = dir / "Init_Jacobi.txt";
    const auto roiPath = dir / "ROI_data.txt";
    const auto velocityPath = dir / "v_data.txt";

    bool ok = true;
    ok &= writeTextFile(featurePath, "1 2 3 4\n99,5,6,7,8\n");
    ok &= writeTextFile(robotPath, "0.1 0.2 0.3 9 9 9\n");
    ok &= writeTextFile(cameraPath, "1 2 3 4 5 6\n");
    ok &= writeTextFile(
        jacobianPath,
        "1 2 3\n4 5 6\n7 8 9\n10 11 12\n"
        "# Residual: 6.0\n# Condition Number: 1.5\n");
    ok &= writeTextFile(
        roiPath,
        "Frame,ROI_X,ROI_Y,ROI_W,ROI_H\n4,1,2,30,40\n");
    ok &= writeTextFile(velocityPath, "9,0.01,0.02,0.03,0.04,0.05,1\n");
    if (!ok) {
        return false;
    }

    const auto features =
        IbvsOfflineDataReader::readFeatureCoordinates(featurePath.string());
    const auto robots =
        IbvsOfflineDataReader::readRobotTranslations(robotPath.string());
    const auto cameras =
        IbvsOfflineDataReader::readCameraPoses(cameraPath.string());
    const auto jacobian =
        IbvsOfflineDataReader::readInitialJacobian(jacobianPath.string());
    const auto rois = IbvsOfflineDataReader::readRois(roiPath.string());
    const auto velocities =
        IbvsOfflineDataReader::readVelocitySeries(velocityPath.string());

    ok &= expectTrue(features.error.isOk(), "feature file should parse");
    ok &= expectTrue(features.values.size() == 2, "feature file should have two rows");
    ok &= expectNear(features.values[1](3), 8.0, "feature value should parse comma");
    ok &= expectTrue(robots.error.isOk(), "robot translation file should parse");
    ok &= expectNear(robots.values.front()(2), 0.3, "robot z should parse");
    ok &= expectTrue(cameras.error.isOk(), "camera pose file should parse");
    ok &= expectNear(cameras.values.front()(5), 6.0, "camera rz should parse");
    ok &= expectTrue(jacobian.error.isOk(), "jacobian file should parse");
    ok &= expectNear(jacobian.jacobian(3, 2), 12.0, "jacobian value should parse");
    ok &= expectTrue(rois.error.isOk(), "ROI file should parse");
    ok &= expectTrue(rois.values.front().isValid(), "ROI should be valid");
    ok &= expectTrue(velocities.error.isOk(), "velocity file should parse");
    ok &= expectNear(velocities.values.front()(1), 0.02, "velocity y should parse");
    return ok;
}

} // namespace

/**
 * @brief 纯视觉 IBVS 核心算法 smoke 测试入口。
 */
int main()
{
    bool ok = true;
    ok &= testTypes();
    ok &= testFeatureGeometry();
    ok &= testStartPointDetector();
    ok &= testJacobianEstimator();
    ok &= testVelocitySolver();
    ok &= testOfflineDataReader();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: IBVS core tests\n";
    return 0;
}
