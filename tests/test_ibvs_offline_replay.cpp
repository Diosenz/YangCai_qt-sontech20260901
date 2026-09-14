/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_offline_replay.cpp
 * @brief       ibvs offline replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ibvs/IbvsJacobianEstimator.h"
#include "ibvs/IbvsOfflineDataReader.h"
#include "ibvs/IbvsVelocitySolver.h"

namespace {

using namespace i2r_rlc::ibvs;

struct OrthogonalSamples
{
    std::vector<Eigen::Vector3d> deltaM;
    std::vector<Eigen::Vector4d> deltaFeaturePx;
};

struct ReplayStats
{
    std::size_t totalFrames = 0;
    std::size_t updatedFrames = 0;
    std::size_t skippedFrames = 0;
    std::size_t failedUpdateFrames = 0;
    std::size_t validVelocityFrames = 0;
    std::size_t velocityLimitedFrames = 0;
    std::size_t nisFrames = 0;
    double averageVelocityMps = 0.0;
    double maxVelocityMps = 0.0;
    double averageErrorPx = 0.0;
    double maxErrorPx = 0.0;
    double averageNis = 0.0;
    double maxNis = 0.0;
};

struct OrthogonalJ0Stats
{
    std::size_t sampleCount = 0;
    double residualNorm = 0.0;
    double conditionEstimate = -1.0;
    double relativeError = 0.0;
};

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

bool expectFinite(double value, const char* message)
{
    if (!std::isfinite(value)) {
        std::cerr << "FAIL: " << message << ", value=" << value << '\n';
        return false;
    }

    return true;
}

double computeRelativeJacobianError(
    const IbvsJacobianMatrix& actual,
    const IbvsJacobianMatrix& expected)
{
    const double denominator = std::max(expected.norm(), 1.0);
    return (actual - expected).norm() / denominator;
}

std::vector<double> parseNumbers(const std::string& line)
{
    const std::size_t firstContent = line.find_first_not_of(" \t\r\n");
    if (firstContent == std::string::npos || line[firstContent] == '#') {
        return {};
    }

    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), ';', ' ');

    std::istringstream stream(normalized);
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value) {
        values.push_back(value);
    }
    return values;
}

std::vector<std::vector<double>> readNumericRows(const std::string& path)
{
    std::ifstream file(std::filesystem::u8path(path));
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> values = parseNumbers(line);
        if (!values.empty()) {
            rows.push_back(values);
        }
    }
    return rows;
}

bool requiredFileExists(const std::string& path)
{
    return std::filesystem::exists(std::filesystem::u8path(path));
}

bool hasRequiredDataset(
    const std::vector<std::string>& requiredFiles,
    std::string* missingPath)
{
    for (const std::string& path : requiredFiles) {
        if (!requiredFileExists(path)) {
            if (missingPath != nullptr) {
                *missingPath = path;
            }
            return false;
        }
    }

    return true;
}

OrthogonalSamples makeOrthogonalSamples(
    const std::vector<IbvsFeatureVector>& features,
    const std::vector<Eigen::Vector3d>& poses)
{
    OrthogonalSamples samples;
    const std::size_t count = std::min(features.size(), poses.size());
    for (std::size_t i = 1; i < count; ++i) {
        const Eigen::Vector3d deltaM = poses[i] - poses[i - 1];
        const Eigen::Vector4d deltaFeature = features[i] - features[i - 1];
        const double motionNorm = deltaM.norm();
        const double featureMagnitude = deltaFeature.norm();

        if (motionNorm > 1e-5
            && motionNorm < 0.05
            && featureMagnitude > 0.1
            && featureMagnitude < 100.0) {
            samples.deltaM.push_back(deltaM);
            samples.deltaFeaturePx.push_back(deltaFeature);
        }
    }

    return samples;
}

bool compareJacobian(
    const IbvsJacobianMatrix& actual,
    const IbvsJacobianMatrix& expected,
    double relativeTolerance,
    const char* message)
{
    const double relativeError =
        computeRelativeJacobianError(actual, expected);
    if (relativeError > relativeTolerance) {
        std::cerr << "FAIL: " << message
                  << ", relativeError=" << relativeError
                  << "\nactual=\n" << actual
                  << "\nexpected=\n" << expected << '\n';
        return false;
    }

    return true;
}

void printReplayDiagnosis(const ReplayStats& stats)
{
    if (stats.totalFrames == 0) {
        return;
    }

    const double updateRatio =
        static_cast<double>(stats.updatedFrames)
        / static_cast<double>(stats.totalFrames);
    const double limitRatio = stats.validVelocityFrames > 0
        ? static_cast<double>(stats.velocityLimitedFrames)
            / static_cast<double>(stats.validVelocityFrames)
        : 0.0;

    std::cout << "INFO: replay diagnostics"
              << ", updateRatio=" << updateRatio
              << ", velocityLimitRatio=" << limitRatio
              << ", averageVelocityMps=" << stats.averageVelocityMps
              << ", maxVelocityMps=" << stats.maxVelocityMps
              << ", averageErrorPx=" << stats.averageErrorPx
              << ", maxErrorPx=" << stats.maxErrorPx
              << ", averageNis=" << stats.averageNis
              << ", maxNis=" << stats.maxNis << '\n';

    if (limitRatio > 0.8 && stats.averageErrorPx > 100.0) {
        std::cout << "INFO: replay velocity limit note="
                  << "high feature error with first-frame desired feature; "
                  << "current test proves offline chain viability, not final "
                  << "control gain suitability\n";
    } else if (limitRatio > 0.8) {
        std::cout << "INFO: replay velocity limit note="
                  << "configured maxCameraVelocityMps is the dominant cap\n";
    } else {
        std::cout << "INFO: replay velocity limit note="
                  << "velocity cap is not dominant in this replay window\n";
    }
}

ReplayStats runSmallReplay(
    const std::vector<IbvsFeatureVector>& features,
    const std::vector<IbvsPoseVector>& cameraPoses,
    const IbvsJacobianMatrix& initialJacobian)
{
    IbvsKalmanConfig config;
    config.epsUpdateM = 1e-7;
    config.nisRejectThreshold = 1e12;
    config.nisInflateThreshold = 1e11;
    config.maxHistorySize = 80;

    IbvsJacobianEstimator estimator(config);
    estimator.kfInitWithJ0(initialJacobian);

    IbvsControlParameters control;
    control.desiredFeature = features.front();
    control.maxCameraVelocityMps = 0.02;
    IbvsVelocitySolver solver(control);

    ReplayStats stats;
    const std::size_t frameCount =
        std::min<std::size_t>({ features.size(), cameraPoses.size(), 80 });
    double errorSum = 0.0;
    double velocitySum = 0.0;
    double nisSum = 0.0;

    for (std::size_t i = 1; i < frameCount; ++i) {
        const Eigen::Vector3d previous =
            cameraPoses[i - 1].head<3>() * 1e-6;
        const Eigen::Vector3d current =
            cameraPoses[i].head<3>() * 1e-6;
        const Eigen::Vector3d deltaM = current - previous;
        const Eigen::Vector4d deltaFeature = features[i] - features[i - 1];

        const auto update = estimator.jacobianEK(deltaM, deltaFeature);
        ++stats.totalFrames;
        if (!update.error.isOk()) {
            ++stats.failedUpdateFrames;
        }
        if (update.updated) {
            ++stats.updatedFrames;
        }
        if (update.skippedBySmallMotion) {
            ++stats.skippedFrames;
        }
        if (std::isfinite(update.nis) && update.nis >= 0.0) {
            ++stats.nisFrames;
            nisSum += update.nis;
            stats.maxNis = std::max(stats.maxNis, update.nis);
        }

        const auto velocity =
            solver.computeCameraVelocity(update.jacobian, features[i]);
        if (velocity.error.isOk()) {
            ++stats.validVelocityFrames;
            const double velocityNorm = velocity.velocityMps.norm();
            velocitySum += velocityNorm;
            stats.maxVelocityMps = std::max(stats.maxVelocityMps, velocityNorm);
            if (velocity.velocityLimited) {
                ++stats.velocityLimitedFrames;
            }
        }

        const double errorNorm = solver.errorNorm(features[i]);
        errorSum += errorNorm;
        stats.maxErrorPx = std::max(stats.maxErrorPx, errorNorm);
    }

    if (stats.totalFrames > 0) {
        stats.averageErrorPx =
            errorSum / static_cast<double>(stats.totalFrames);
    }
    if (stats.validVelocityFrames > 0) {
        stats.averageVelocityMps =
            velocitySum / static_cast<double>(stats.validVelocityFrames);
    }
    if (stats.nisFrames > 0) {
        stats.averageNis = nisSum / static_cast<double>(stats.nisFrames);
    }

    return stats;
}

bool testOrthogonalJ0(
    const std::string& featurePath,
    const std::string& robotPath,
    const std::string& jacobianPath,
    OrthogonalJ0Stats* stats)
{
    const auto features =
        IbvsOfflineDataReader::readFeatureCoordinates(featurePath);
    const auto robotTranslations =
        IbvsOfflineDataReader::readRobotTranslations(robotPath);
    const auto fileJacobian =
        IbvsOfflineDataReader::readInitialJacobian(jacobianPath);

    bool ok = true;
    ok &= expectTrue(features.error.isOk(), "Orthogonal features should parse");
    ok &= expectTrue(
        robotTranslations.error.isOk(),
        "Orthogonal robot translations should parse");
    ok &= expectTrue(
        fileJacobian.error.isOk(),
        "Orthogonal Init_Jacobi should parse");
    ok &= expectTrue(
        features.values.size() == 65,
        "Orthogonal feature row count should be 65");
    ok &= expectTrue(
        robotTranslations.values.size() == 65,
        "Orthogonal robot row count should be 65");
    ok &= expectTrue(
        fileJacobian.jacobian.norm() > 1e-6,
        "Init_Jacobi should be non-zero");
    if (!ok) {
        return false;
    }

    const OrthogonalSamples samples =
        makeOrthogonalSamples(features.values, robotTranslations.values);
    ok &= expectTrue(
        samples.deltaM.size() >= 6,
        "Orthogonal filtered sample count should be at least 6");

    IbvsJacobianEstimator estimator;
    const auto ols = estimator.computeInitialJacobianOLS(
        samples.deltaM,
        samples.deltaFeaturePx,
        1e-6);
    ok &= expectTrue(ols.error.isOk(), "Orthogonal OLS should succeed");
    ok &= expectTrue(ols.jacobian.norm() > 1e-6, "Orthogonal OLS should be non-zero");
    ok &= expectFinite(ols.residualNorm, "Orthogonal residual should be finite");
    ok &= expectFinite(
        ols.conditionEstimate,
        "Orthogonal condition estimate should be finite");
    ok &= compareJacobian(
        ols.jacobian,
        fileJacobian.jacobian,
        2e-5,
        "OLS J0 should match Init_Jacobi");

    if (stats != nullptr) {
        stats->sampleCount = samples.deltaM.size();
        stats->residualNorm = ols.residualNorm;
        stats->conditionEstimate = ols.conditionEstimate;
        stats->relativeError = computeRelativeJacobianError(
            ols.jacobian,
            fileJacobian.jacobian);
    }

    std::cout << "INFO: Orthogonal samples=" << samples.deltaM.size()
              << ", residual=" << ols.residualNorm
              << ", condition=" << ols.conditionEstimate
              << ", relativeJ0Error="
              << computeRelativeJacobianError(
                  ols.jacobian,
                  fileJacobian.jacobian)
              << '\n';
    return ok;
}

bool testOnlineReaders(
    const std::string& featurePath,
    const std::string& roiPath,
    const std::string& velocityPath,
    const std::string& cameraPath)
{
    const auto features =
        IbvsOfflineDataReader::readFeatureCoordinates(featurePath);
    const auto rois = IbvsOfflineDataReader::readRois(roiPath);
    const auto velocities =
        IbvsOfflineDataReader::readVelocitySeries(velocityPath);
    const auto cameraPoses =
        IbvsOfflineDataReader::readCameraPoses(cameraPath);

    bool ok = true;
    ok &= expectTrue(features.error.isOk(), "online features should parse");
    ok &= expectTrue(rois.error.isOk(), "online ROI records should parse");
    ok &= expectTrue(velocities.error.isOk(), "v_data should parse");
    ok &= expectTrue(cameraPoses.error.isOk(), "Camera_Position should parse");
    ok &= expectTrue(features.values.size() == 101, "online feature row count");
    ok &= expectTrue(rois.values.size() == 101, "online ROI row count");
    ok &= expectTrue(velocities.values.size() == 319, "v_data row count");
    ok &= expectTrue(cameraPoses.values.size() == 319, "camera pose row count");
    if (!ok) {
        return false;
    }

    const std::vector<std::vector<double>> roiRows = readNumericRows(roiPath);
    const std::size_t checkCount =
        std::min<std::size_t>({ roiRows.size(), features.values.size(), 8 });
    for (std::size_t i = 0; i < checkCount; ++i) {
        const PixelRoi& roi = rois.values[i];
        const std::vector<double>& row = roiRows[i];
        ok &= expectTrue(roi.isValid(), "ROI should be valid");
        ok &= expectNear(roi.x + row[5], row[9], 1e-3, "global c1 x");
        ok &= expectNear(roi.y + row[6], row[10], 1e-3, "global c1 y");
        ok &= expectNear(roi.x + row[7], row[11], 1e-3, "global c2 x");
        ok &= expectNear(roi.y + row[8], row[12], 1e-3, "global c2 y");
        ok &= expectNear(features.values[i](0), row[9], 1e-6, "feature c1 x");
        ok &= expectNear(features.values[i](1), row[10], 1e-6, "feature c1 y");
        ok &= expectNear(features.values[i](2), row[11], 1e-6, "feature c2 x");
        ok &= expectNear(features.values[i](3), row[12], 1e-6, "feature c2 y");
    }

    return ok;
}

bool testSmallReplay(
    const std::string& onlineFeaturePath,
    const std::string& cameraPath,
    const std::string& jacobianPath)
{
    const auto features =
        IbvsOfflineDataReader::readFeatureCoordinates(onlineFeaturePath);
    const auto cameraPoses =
        IbvsOfflineDataReader::readCameraPoses(cameraPath);
    const auto initialJacobian =
        IbvsOfflineDataReader::readInitialJacobian(jacobianPath);

    bool ok = true;
    ok &= expectTrue(features.error.isOk(), "replay features should parse");
    ok &= expectTrue(cameraPoses.error.isOk(), "replay camera poses should parse");
    ok &= expectTrue(initialJacobian.error.isOk(), "replay J0 should parse");
    if (!ok) {
        return false;
    }

    const ReplayStats stats = runSmallReplay(
        features.values,
        cameraPoses.values,
        initialJacobian.jacobian);
    ok &= expectTrue(stats.totalFrames > 0, "replay should process frames");
    ok &= expectTrue(stats.updatedFrames > 0, "replay should update some frames");
    ok &= expectTrue(
        stats.failedUpdateFrames == 0,
        "replay KF updates should not report errors");
    ok &= expectTrue(
        stats.validVelocityFrames > 0,
        "replay should produce valid velocity solves");
    ok &= expectFinite(stats.maxVelocityMps, "max velocity should be finite");
    ok &= expectFinite(
        stats.averageVelocityMps,
        "average velocity should be finite");
    ok &= expectFinite(stats.averageErrorPx, "average error should be finite");
    ok &= expectFinite(stats.maxErrorPx, "max error should be finite");
    ok &= expectTrue(stats.nisFrames > 0, "replay should produce NIS records");
    ok &= expectFinite(stats.averageNis, "average NIS should be finite");
    ok &= expectFinite(stats.maxNis, "max NIS should be finite");
    ok &= expectTrue(
        stats.maxVelocityMps <= 0.0200001,
        "velocity should obey configured max velocity");

    std::cout << "INFO: replay total=" << stats.totalFrames
              << ", updated=" << stats.updatedFrames
              << ", skipped=" << stats.skippedFrames
              << ", failedUpdates=" << stats.failedUpdateFrames
              << ", validVelocity=" << stats.validVelocityFrames
              << ", velocityLimited=" << stats.velocityLimitedFrames
              << ", nisFrames=" << stats.nisFrames
              << ", maxVelocityMps=" << stats.maxVelocityMps
              << ", averageErrorPx=" << stats.averageErrorPx << '\n';
    printReplayDiagnosis(stats);
    return ok;
}

bool testFrameCompatibleReaderFormats()
{
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path()
        / "i2r_rlc_ibvs_offline_replay_tests";
    std::filesystem::create_directories(dir);

    const std::filesystem::path featurePath = dir / "Feature_coordinates.txt";
    const std::filesystem::path velocityPath = dir / "v_data.txt";
    {
        std::ofstream featureFile(featurePath);
        featureFile << "0,10,20,30,40\n";
    }
    {
        std::ofstream velocityFile(velocityPath);
        velocityFile << "7 0.1 0.2 0.3 0.4 0.5 1\n";
    }

    const auto features =
        IbvsOfflineDataReader::readFeatureCoordinates(featurePath.string());
    const auto velocities =
        IbvsOfflineDataReader::readVelocitySeries(velocityPath.string());

    bool ok = true;
    ok &= expectTrue(features.error.isOk(), "frame feature format should parse");
    ok &= expectTrue(velocities.error.isOk(), "frame velocity format should parse");
    ok &= expectNear(features.values.front()(0), 10.0, 1e-12, "frame feature x1");
    ok &= expectNear(features.values.front()(3), 40.0, 1e-12, "frame feature y2");
    ok &= expectNear(velocities.values.front()(0), 0.1, 1e-12, "frame velocity vx");
    ok &= expectNear(velocities.values.front()(2), 0.3, 1e-12, "frame velocity vz");
    return ok;
}

} // namespace

/**
 * @brief 使用本机 Feature_detection txt 数据执行纯视觉 IBVS 离线小链路验证。
 */
int main()
{
    const std::string root = u8"C:/Feature_detection/FeatureDetection";
    const std::string orthogonal =
        root + u8"/FeatureDetection/Orthogonal";
    const std::string onlineData =
        root + u8"/FeatureDetection/data";
    const std::string runtimeData =
        root + u8"/在线数据";

    const std::string orthogonalFeature =
        orthogonal + "/Feature_coordinates.txt";
    const std::string orthogonalRobot =
        orthogonal + "/IBVSRobotData.txt";
    const std::string orthogonalJacobian =
        orthogonal + "/Init_Jacobi.txt";
    const std::string onlineFeature =
        onlineData + "/Feature_coordinates.txt";
    const std::string onlineRoi =
        onlineData + "/ROI_data.txt";
    const std::string velocityPath =
        runtimeData + "/v_data.txt";
    const std::string cameraPath =
        runtimeData + "/Camera_Position.txt";

    const std::vector<std::string> requiredFiles{
        orthogonalFeature,
        orthogonalRobot,
        orthogonalJacobian,
        onlineFeature,
        onlineRoi,
        velocityPath,
        cameraPath
    };
    std::string missingPath;
    if (!hasRequiredDataset(requiredFiles, &missingPath)) {
        std::cout << "SKIP: C:/Feature_detection offline IBVS data missing: "
                  << missingPath << '\n';
        return 0;
    }

    bool ok = true;
    OrthogonalJ0Stats orthogonalStats;
    ok &= testOrthogonalJ0(
        orthogonalFeature,
        orthogonalRobot,
        orthogonalJacobian,
        &orthogonalStats);
    ok &= testOnlineReaders(
        onlineFeature,
        onlineRoi,
        velocityPath,
        cameraPath);
    ok &= testSmallReplay(
        onlineFeature,
        cameraPath,
        orthogonalJacobian);
    ok &= testFrameCompatibleReaderFormats();

    if (!ok) {
        return 1;
    }

    std::cout << "INFO: verification baseline"
              << ", orthogonalSamples=" << orthogonalStats.sampleCount
              << ", residual=" << orthogonalStats.residualNorm
              << ", condition=" << orthogonalStats.conditionEstimate
              << ", relativeJ0Error=" << orthogonalStats.relativeError << '\n';
    std::cout << "PASS: IBVS offline replay tests\n";
    return 0;
}
