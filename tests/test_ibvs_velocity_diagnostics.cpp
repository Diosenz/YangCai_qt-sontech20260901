/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      test_ibvs_velocity_diagnostics.cpp
 * @brief     IBVS KF 和速度求解的离线诊断留档测试。
 * @details   读取 `C:\Feature_detection` txt 数据，保存逐帧 CSV 和误差/速度/NIS
 *            诊断曲线；只验证数值链路，不执行机器人闭环。
 * @author    I2R Laboratory
 * @version   1.0
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsJacobianEstimator.h"
#include "ibvs/IbvsOfflineDataReader.h"
#include "ibvs/IbvsVelocitySolver.h"

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

namespace {

using namespace i2r_rlc::ibvs;

struct VelocityFrameMetric
{
    int frame = 0;
    double deltaMNorm = 0.0;
    double errorNormPx = 0.0;
    double velocityNormMps = 0.0;
    double nis = -1.0;
    bool updated = false;
    bool skipped = false;
    bool failedUpdate = false;
    bool validVelocity = false;
    bool velocityLimited = false;
};

std::string makeTimestamp(void)
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(now);
    std::tm localTime;
    localtime_s(&localTime, &timeValue);

    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::filesystem::path makeArtifactDir(void)
{
    const std::filesystem::path root =
        std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "output"
        / "ibvs_offline_validation"
        / "i2r_rlc_ibvs_velocity_diagnostics_tests"
        / makeTimestamp();
    std::filesystem::create_directories(root);
    return root;
}

bool hasRequiredDataset(
    const std::vector<std::string>& requiredFiles,
    std::string* missingPath)
{
    for (const std::string& path : requiredFiles)
    {
        if (!std::filesystem::exists(std::filesystem::u8path(path)))
        {
            if (missingPath != nullptr)
            {
                *missingPath = path;
            }
            return false;
        }
    }
    return true;
}

bool writeMetricsCsv(
    const std::filesystem::path& path,
    const std::vector<VelocityFrameMetric>& metrics)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        std::cerr << "FAIL: cannot write metrics csv: " << path.string() << '\n';
        return false;
    }

    file << "frame,delta_m_norm,error_norm_px,velocity_norm_mps,nis,"
            "updated,skipped,failed_update,valid_velocity,velocity_limited\n";
    for (const VelocityFrameMetric& metric : metrics)
    {
        file << metric.frame << ','
             << metric.deltaMNorm << ','
             << metric.errorNormPx << ','
             << metric.velocityNormMps << ','
             << metric.nis << ','
             << (metric.updated ? 1 : 0) << ','
             << (metric.skipped ? 1 : 0) << ','
             << (metric.failedUpdate ? 1 : 0) << ','
             << (metric.validVelocity ? 1 : 0) << ','
             << (metric.velocityLimited ? 1 : 0) << '\n';
    }
    return true;
}

double finiteMax(const std::vector<double>& values, double fallback)
{
    double maxValue = fallback;
    for (double value : values)
    {
        if (std::isfinite(value))
        {
            maxValue = std::max(maxValue, value);
        }
    }
    return maxValue;
}

void drawPlot(
    const std::filesystem::path& path,
    const std::vector<double>& values,
    double yMax,
    const cv::Scalar& color)
{
    cv::Mat plot(360, 720, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(plot, cv::Rect(50, 30, 660, 300), cv::Scalar(210, 210, 210), 1);
    if (values.size() > 1 && yMax > 0.0)
    {
        std::vector<cv::Point> points;
        points.reserve(values.size());
        for (std::size_t i = 0; i < values.size(); ++i)
        {
            const double normalized =
                std::max(0.0, std::min(1.0, values[i] / yMax));
            const int x = 50 + static_cast<int>(
                std::lround(660.0 * static_cast<double>(i)
                            / static_cast<double>(values.size() - 1)));
            const int y = 330 - static_cast<int>(std::lround(300.0 * normalized));
            points.emplace_back(x, y);
        }
        cv::polylines(plot, points, false, color, 2, cv::LINE_AA);
    }
    cv::imwrite(path.string(), plot);
}

} // namespace

/**
 * @brief 离线 KF 更新和速度求解诊断测试入口。
 */
int main(void)
{
    const std::string root = u8"C:/Feature_detection/FeatureDetection";
    const std::string orthogonal = root + u8"/FeatureDetection/Orthogonal";
    const std::string onlineData = root + u8"/FeatureDetection/data";
    const std::string runtimeData = root + u8"/在线数据";

    const std::string onlineFeature =
        onlineData + "/Feature_coordinates.txt";
    const std::string cameraPath =
        runtimeData + "/Camera_Position.txt";
    const std::string jacobianPath =
        orthogonal + "/Init_Jacobi.txt";

    std::string missingPath;
    if (!hasRequiredDataset({ onlineFeature, cameraPath, jacobianPath }, &missingPath))
    {
        std::cout << "SKIP: C:/Feature_detection velocity data missing: "
                  << missingPath << '\n';
        return 0;
    }

    const auto features =
        IbvsOfflineDataReader::readFeatureCoordinates(onlineFeature);
    const auto cameraPoses =
        IbvsOfflineDataReader::readCameraPoses(cameraPath);
    const auto initialJacobian =
        IbvsOfflineDataReader::readInitialJacobian(jacobianPath);

    if (!features.error.isOk()
        || !cameraPoses.error.isOk()
        || !initialJacobian.error.isOk())
    {
        std::cerr << "FAIL: velocity diagnostics input parsing failed\n";
        return 1;
    }

    const std::filesystem::path artifactDir = makeArtifactDir();

    IbvsKalmanConfig kalmanConfig;
    kalmanConfig.epsUpdateM = 1e-7;
    kalmanConfig.nisRejectThreshold = 1e12;
    kalmanConfig.nisInflateThreshold = 1e11;
    kalmanConfig.maxHistorySize = 80;

    IbvsJacobianEstimator estimator(kalmanConfig);
    estimator.kfInitWithJ0(initialJacobian.jacobian);

    IbvsControlParameters control;
    control.desiredFeature = features.values.front();
    control.maxCameraVelocityMps = 0.02;
    IbvsVelocitySolver solver(control);

    std::vector<VelocityFrameMetric> metrics;
    const std::size_t frameCount =
        std::min<std::size_t>({ features.values.size(), cameraPoses.values.size(), 80 });
    metrics.reserve(frameCount);

    for (std::size_t i = 1; i < frameCount; ++i)
    {
        const Eigen::Vector3d previous =
            cameraPoses.values[i - 1].head<3>() * 1e-6;
        const Eigen::Vector3d current =
            cameraPoses.values[i].head<3>() * 1e-6;
        const Eigen::Vector3d deltaM = current - previous;
        const Eigen::Vector4d deltaFeature =
            features.values[i] - features.values[i - 1];

        const auto update = estimator.jacobianEK(deltaM, deltaFeature);
        const auto velocity =
            solver.computeCameraVelocity(update.jacobian, features.values[i]);

        VelocityFrameMetric metric;
        metric.frame = static_cast<int>(i);
        metric.deltaMNorm = deltaM.norm();
        metric.errorNormPx = solver.errorNorm(features.values[i]);
        metric.nis = update.nis;
        metric.updated = update.updated;
        metric.skipped = update.skippedBySmallMotion;
        metric.failedUpdate = !update.error.isOk();
        metric.validVelocity = velocity.error.isOk();
        metric.velocityLimited = velocity.velocityLimited;
        if (velocity.error.isOk())
        {
            metric.velocityNormMps = velocity.velocityMps.norm();
        }
        metrics.push_back(metric);
    }

    std::size_t failedUpdates = 0;
    std::size_t validVelocity = 0;
    std::size_t velocityLimited = 0;
    double errorSum = 0.0;
    double velocitySum = 0.0;
    double maxError = 0.0;
    double maxVelocity = 0.0;
    std::vector<double> errors;
    std::vector<double> velocities;
    std::vector<double> nisValues;
    std::vector<double> limitFlags;

    for (const VelocityFrameMetric& metric : metrics)
    {
        failedUpdates += metric.failedUpdate ? 1 : 0;
        validVelocity += metric.validVelocity ? 1 : 0;
        velocityLimited += metric.velocityLimited ? 1 : 0;
        errorSum += metric.errorNormPx;
        velocitySum += metric.velocityNormMps;
        maxError = std::max(maxError, metric.errorNormPx);
        maxVelocity = std::max(maxVelocity, metric.velocityNormMps);
        errors.push_back(metric.errorNormPx);
        velocities.push_back(metric.velocityNormMps);
        nisValues.push_back(std::max(0.0, metric.nis));
        limitFlags.push_back(metric.velocityLimited ? 1.0 : 0.0);

        if (!std::isfinite(metric.errorNormPx)
            || !std::isfinite(metric.velocityNormMps)
            || !std::isfinite(metric.deltaMNorm))
        {
            std::cerr << "FAIL: velocity diagnostics produced non-finite value\n";
            return 1;
        }
    }

    if (!writeMetricsCsv(artifactDir / "metrics.csv", metrics))
    {
        return 1;
    }

    drawPlot(
        artifactDir / "error_norm_px.png",
        errors,
        finiteMax(errors, 1.0),
        cv::Scalar(0, 0, 220));
    drawPlot(
        artifactDir / "velocity_norm_mps.png",
        velocities,
        std::max(0.02, finiteMax(velocities, 0.02)),
        cv::Scalar(220, 0, 0));
    drawPlot(
        artifactDir / "nis.png",
        nisValues,
        finiteMax(nisValues, 1.0),
        cv::Scalar(0, 160, 0));
    drawPlot(
        artifactDir / "limit_flags.png",
        limitFlags,
        1.0,
        cv::Scalar(0, 120, 220));

    const double averageError = metrics.empty()
        ? 0.0
        : errorSum / static_cast<double>(metrics.size());
    const double averageVelocity = validVelocity == 0
        ? 0.0
        : velocitySum / static_cast<double>(validVelocity);
    const double limitRatio = validVelocity == 0
        ? 0.0
        : static_cast<double>(velocityLimited) / static_cast<double>(validVelocity);

    std::ofstream summary(artifactDir / "summary.txt");
    summary << "frames=" << metrics.size() << '\n'
            << "failedUpdates=" << failedUpdates << '\n'
            << "validVelocity=" << validVelocity << '\n'
            << "velocityLimited=" << velocityLimited << '\n'
            << "velocityLimitRatio=" << limitRatio << '\n'
            << "averageErrorPx=" << averageError << '\n'
            << "maxErrorPx=" << maxError << '\n'
            << "averageVelocityMps=" << averageVelocity << '\n'
            << "maxVelocityMps=" << maxVelocity << '\n'
            << "artifactDir=" << artifactDir.string() << '\n';

    if (metrics.empty())
    {
        std::cerr << "FAIL: velocity diagnostics processed no frames\n";
        return 1;
    }
    if (failedUpdates != 0)
    {
        std::cerr << "FAIL: velocity diagnostics KF update failure count="
                  << failedUpdates << '\n';
        return 1;
    }
    if (validVelocity == 0)
    {
        std::cerr << "FAIL: velocity diagnostics produced no valid velocities\n";
        return 1;
    }
    if (maxVelocity > control.maxCameraVelocityMps + 1e-7)
    {
        std::cerr << "FAIL: velocity exceeds configured max velocity\n";
        return 1;
    }

    std::cout << "INFO: velocity diagnostics artifacts="
              << artifactDir.string() << '\n';
    std::cout << "INFO: velocity diagnostics frames=" << metrics.size()
              << ", failedUpdates=" << failedUpdates
              << ", validVelocity=" << validVelocity
              << ", velocityLimitRatio=" << limitRatio
              << ", maxVelocityMps=" << maxVelocity
              << ", averageErrorPx=" << averageError << '\n';
    std::cout << "PASS: IBVS velocity diagnostics tests\n";
    return 0;
}
