/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OfflineCorrectionTrajectoryMatcher.cpp
 * @brief       离线纠偏轨迹匹配器实现
 * @details     使用纯 C++ 文件解析和 Vision ICP 封装承接旧 OffLineCor 有匹配主链路。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <sstream>

#include <Eigen/Core>

#include "geometry/IcpRegistrationMatcher.h"
#include "OfflineCorrectionTrajectoryMatcher.h"

namespace i2r_rlc::control {
namespace {

/**
 * @brief 单行数值解析结果。
 */
struct NumericLine {
    /// @brief 当前行解析出的所有浮点数，单位由调用场景决定。
    std::vector<double> values;
};

/**
 * @brief 离线检测点清洗结果及各阶段数量。
 */
struct PreparedDetectionPoints {
    std::vector<i2r_rlc::vision::WeldSpacePoint> points;
    std::size_t validCount = 0;
    std::size_t spatialSampledCount = 0;
    std::size_t clusteredCount = 0;
};

/**
 * @brief 判断文本行是否只包含空白字符。
 * @param line 原始文本行。
 * @return 空白行返回 true。
 */
bool isNearlyEmpty(const std::string &line)
{
    return line.find_first_not_of(" \t\r\n") == std::string::npos;
}

/**
 * @brief 按空白分隔读取纯数值文本文件。
 * @param path 文件路径。
 * @return 每个非空行的数值列表，无法解析的字段会在该行停止。
 */
std::vector<NumericLine> readNumericFile(const std::filesystem::path &path)
{
    std::vector<NumericLine> lines;
    std::ifstream stream(path);
    std::string line;
    while (std::getline(stream, line)) {
        if (isNearlyEmpty(line)) {
            continue;
        }
        std::stringstream parser(line);
        NumericLine numericLine;
        double value = 0.0;
        while (parser >> value) {
            numericLine.values.push_back(value);
        }
        if (!numericLine.values.empty()) {
            lines.push_back(numericLine);
        }
    }
    return lines;
}

/**
 * @brief 根据首个坐标值推断旧文件位置单位换算比例。
 * @param firstX 首个 X 坐标原始值。
 * @return 原始值除以该比例后得到 mm；旧机器人 um 数据返回 1000。
 */
double positionScaleToMm(const double firstX)
{
    return std::abs(firstX) > 10000.0 ? 1000.0 : 1.0;
}

/**
 * @brief 根据首个角度值推断旧文件角度单位换算比例。
 * @param firstAngle 首个姿态或轴角原始值。
 * @return 原始值除以该比例后得到 deg；旧协议 deg * 10000 返回 10000。
 */
double angleScaleToDeg(const double firstAngle)
{
    return std::abs(firstAngle) > 10000.0 ? 10000.0 : 1.0;
}

/**
 * @brief 将浮点值四舍五入为机器人协议整数。
 * @param value 原始浮点值。
 * @return long 类型整数。
 */
long toLongRounded(const double value)
{
    return static_cast<long>(std::llround(value));
}

/**
 * @brief 构造有效焊缝空间点。
 * @param xMm X 坐标，单位为 mm。
 * @param yMm Y 坐标，单位为 mm。
 * @param zMm Z 坐标，单位为 mm。
 * @return valid 为 true 的 WeldSpacePoint。
 */
i2r_rlc::vision::WeldSpacePoint makePoint(
    const double xMm,
    const double yMm,
    const double zMm)
{
    return i2r_rlc::vision::WeldSpacePoint{ xMm, yMm, zMm, true };
}

/**
 * @brief 根据一个存档路径推导配套 _W.txt 或 _T.txt 路径。
 * @param archivePath 用户选择的存档路径。
 * @param suffix 目标后缀，例如 "_W.txt" 或 "_T.txt"。
 * @return 同目录下同前缀配套文件路径。
 */
std::filesystem::path companionPath(
    const std::filesystem::path &archivePath,
    const std::string &suffix)
{
    std::string stem = archivePath.stem().string();
    const auto underscore = stem.rfind('_');
    if (underscore != std::string::npos) {
        stem = stem.substr(0, underscore);
    }
    return archivePath.parent_path() / (stem + suffix);
}

/**
 * @brief 对检测点做简单滑动平均平滑。
 * @param points 输入点，坐标单位为 mm。
 * @param radius 平滑窗口半径，单位为点数。
 * @return 平滑后的点集合，长度保持不变。
 */
std::vector<i2r_rlc::vision::WeldSpacePoint> smoothPoints(
    const std::vector<i2r_rlc::vision::WeldSpacePoint> &points,
    const int radius)
{
    if (radius <= 0 || points.size() < 3) {
        return points;
    }

    std::vector<i2r_rlc::vision::WeldSpacePoint> smoothed;
    smoothed.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        const std::size_t begin =
            index < static_cast<std::size_t>(radius) ? 0 : index - static_cast<std::size_t>(radius);
        const std::size_t end =
            std::min(points.size() - 1, index + static_cast<std::size_t>(radius));
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        int count = 0;
        for (std::size_t sample = begin; sample <= end; ++sample) {
            if (!points[sample].valid) {
                continue;
            }
            x += points[sample].xMm;
            y += points[sample].yMm;
            z += points[sample].zMm;
            ++count;
        }
        if (count == 0) {
            smoothed.push_back(points[index]);
        } else {
            smoothed.push_back(makePoint(
                x / static_cast<double>(count),
                y / static_cast<double>(count),
                z / static_cast<double>(count)));
        }
    }
    return smoothed;
}

double imageDistance(
    const OfflineCorrectionDetectionSample &left,
    const OfflineCorrectionDetectionSample &right)
{
    const double dx = left.imageXPixel - right.imageXPixel;
    const double dy = left.imageYPixel - right.imageYPixel;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<OfflineCorrectionDetectionSample> spatialSampleDetectionPoints(
    const std::vector<OfflineCorrectionDetectionSample> &samples,
    const double minimumStepMm)
{
    if (minimumStepMm <= 0.0 || samples.size() <= 5U) {
        return samples;
    }

    std::vector<OfflineCorrectionDetectionSample> sampled;
    sampled.reserve(samples.size());
    for (const auto &sample : samples) {
        if (sampled.size() < 5U) {
            sampled.push_back(sample);
            continue;
        }

        const auto &last = sampled.back().point;
        if (std::abs(sample.point.xMm - last.xMm) > minimumStepMm
            || std::abs(sample.point.yMm - last.yMm) > minimumStepMm
            || std::abs(sample.point.zMm - last.zMm) > minimumStepMm) {
            sampled.push_back(sample);
        }
    }
    return sampled;
}

double adaptiveImageClusterRadius(
    const std::vector<OfflineCorrectionDetectionSample> &samples,
    const int minimumPoints)
{
    if (minimumPoints < 2
        || samples.size() < static_cast<std::size_t>(minimumPoints)) {
        return 0.0;
    }

    std::vector<double> neighborDistances;
    neighborDistances.reserve(samples.size());
    for (const auto &sample : samples) {
        std::vector<double> distances;
        distances.reserve(samples.size());
        for (const auto &candidate : samples) {
            distances.push_back(imageDistance(sample, candidate));
        }
        const auto neighborIndex = static_cast<std::size_t>(minimumPoints - 1);
        std::nth_element(
            distances.begin(),
            distances.begin() + static_cast<std::ptrdiff_t>(neighborIndex),
            distances.end());
        neighborDistances.push_back(distances[neighborIndex]);
    }

    std::sort(neighborDistances.begin(), neighborDistances.end());
    const double median = neighborDistances[neighborDistances.size() / 2U];
    if (!std::isfinite(median) || median <= 0.0) {
        return 0.0;
    }

    // 旧 ClusterAnalysis 根据 k 邻域距离自动估计半径。这里保留相同的
    // 2.5 倍密集轨迹尺度，同时用中位数避免单个侧面误点放大半径。
    return median * 2.5;
}

std::vector<OfflineCorrectionDetectionSample> filterImageDensityOutliers(
    const std::vector<OfflineCorrectionDetectionSample> &samples,
    const int minimumPoints)
{
    if (minimumPoints < 2
        || samples.size() < static_cast<std::size_t>(minimumPoints)) {
        return samples;
    }
    if (std::any_of(samples.begin(), samples.end(), [](const auto &sample) {
            return !sample.imagePointValid
                || !std::isfinite(sample.imageXPixel)
                || !std::isfinite(sample.imageYPixel);
        })) {
        return samples;
    }

    const double radius = adaptiveImageClusterRadius(samples, minimumPoints);
    if (radius <= 0.0) {
        return samples;
    }

    constexpr int kUnvisited = -2;
    constexpr int kNoise = -1;
    std::vector<int> labels(samples.size(), kUnvisited);
    int clusterId = 0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (labels[index] != kUnvisited) {
            continue;
        }

        std::vector<std::size_t> neighbors;
        for (std::size_t candidate = 0; candidate < samples.size(); ++candidate) {
            if (imageDistance(samples[index], samples[candidate]) <= radius) {
                neighbors.push_back(candidate);
            }
        }
        if (neighbors.size() < static_cast<std::size_t>(minimumPoints)) {
            labels[index] = kNoise;
            continue;
        }

        labels[index] = clusterId;
        std::vector<bool> queued(samples.size(), false);
        for (const std::size_t neighbor : neighbors) {
            queued[neighbor] = true;
        }
        for (std::size_t neighborOffset = 0;
             neighborOffset < neighbors.size();
             ++neighborOffset) {
            const std::size_t neighbor = neighbors[neighborOffset];
            if (labels[neighbor] == kNoise) {
                labels[neighbor] = clusterId;
            }
            if (labels[neighbor] != kUnvisited) {
                continue;
            }

            labels[neighbor] = clusterId;
            std::vector<std::size_t> expanded;
            for (std::size_t candidate = 0;
                 candidate < samples.size();
                 ++candidate) {
                if (imageDistance(samples[neighbor], samples[candidate]) <= radius) {
                    expanded.push_back(candidate);
                }
            }
            if (expanded.size() >= static_cast<std::size_t>(minimumPoints)) {
                for (const std::size_t candidate : expanded) {
                    if (!queued[candidate]) {
                        queued[candidate] = true;
                        neighbors.push_back(candidate);
                    }
                }
            }
        }
        ++clusterId;
    }

    std::vector<OfflineCorrectionDetectionSample> clustered;
    clustered.reserve(samples.size());
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (labels[index] >= 0) {
            clustered.push_back(samples[index]);
        }
    }
    return clustered;
}

/**
 * @brief 筛选、裁剪并平滑离线检测点，作为 ICP 输入。
 * @param samples 原始检测样本，坐标单位为 mm。
 * @param options 头部丢弃比例和平滑半径配置。
 * @return 预处理后的有效检测点集合。
 */
PreparedDetectionPoints prepareDetectionPoints(
    const std::vector<OfflineCorrectionDetectionSample> &samples,
    const OfflineCorrectionOptions &options)
{
    PreparedDetectionPoints prepared;
    std::vector<OfflineCorrectionDetectionSample> validSamples;
    validSamples.reserve(samples.size());
    for (const auto &sample : samples) {
        if (sample.point.valid
            && std::isfinite(sample.point.xMm)
            && std::isfinite(sample.point.yMm)
            && std::isfinite(sample.point.zMm)) {
            validSamples.push_back(sample);
        }
    }

    prepared.validCount = validSamples.size();
    auto sampled = spatialSampleDetectionPoints(
        validSamples,
        options.minimumSpatialStepMm);
    prepared.spatialSampledCount = sampled.size();
    sampled = filterImageDensityOutliers(
        sampled,
        options.imageClusterMinimumPoints);
    prepared.clusteredCount = sampled.size();

    prepared.points.reserve(sampled.size());
    for (const auto &sample : sampled) {
        prepared.points.push_back(sample.point);
    }

    const auto discardCount = static_cast<std::size_t>(
        std::floor(static_cast<double>(prepared.points.size())
                   * options.discardLeadingDetectionRatio));
    if (discardCount > 0 && discardCount < prepared.points.size()) {
        prepared.points.erase(
            prepared.points.begin(),
            prepared.points.begin() + static_cast<std::ptrdiff_t>(discardCount));
    }
    prepared.points = smoothPoints(prepared.points, options.smoothingRadius);
    return prepared;
}

/**
 * @brief 使用 ICP 输出矩阵变换示教点位置。
 * @param point 输入示教点，坐标单位为 mm。
 * @param matrix 4x4 齐次变换矩阵。
 * @return 变换后的空间点，单位为 mm。
 */
i2r_rlc::vision::WeldSpacePoint transformPosition(
    const i2r_rlc::vision::WeldSpacePoint &point,
    const Eigen::Matrix4d &matrix)
{
    const Eigen::Vector4d source(point.xMm, point.yMm, point.zMm, 1.0);
    const Eigen::Vector3d transformed = (matrix * source).head<3>();
    return makePoint(transformed.x(), transformed.y(), transformed.z());
}

} // namespace

OfflineCorrectionArchive COfflineCorrectionTrajectoryMatcher::loadArchive(
    const std::string &archiveFilePath)
{
    OfflineCorrectionArchive archive;
    const std::filesystem::path archivePath(archiveFilePath);
    if (!std::filesystem::exists(archivePath)) {
        archive.message = "离线纠偏存档文件不存在。";
        return archive;
    }

    std::filesystem::path modelPath = archivePath;
    std::filesystem::path teachPath = archivePath;
    const std::string stem = archivePath.stem().string();
    if (stem.size() >= 2 && stem.substr(stem.size() - 2) == "_W") {
        teachPath = companionPath(archivePath, "_T.txt");
    } else if (stem.size() >= 2 && stem.substr(stem.size() - 2) == "_T") {
        modelPath = companionPath(archivePath, "_W.txt");
    } else {
        modelPath = companionPath(archivePath, "_W.txt");
        teachPath = companionPath(archivePath, "_T.txt");
    }

    if (!std::filesystem::exists(modelPath)) {
        archive.message = "离线纠偏模型轨迹 _W.txt 不存在。";
        return archive;
    }
    if (!std::filesystem::exists(teachPath)) {
        archive.message = "离线纠偏示教点 _T.txt 不存在。";
        return archive;
    }

    const auto modelLines = readNumericFile(modelPath);
    const auto teachLines = readNumericFile(teachPath);
    if (modelLines.size() < 5 || teachLines.size() < 1) {
        archive.message = "离线纠偏存档点数不足。";
        return archive;
    }

    const double modelPositionScale = positionScaleToMm(modelLines.front().values[0]);
    for (const auto &line : modelLines) {
        if (line.values.size() < 3) {
            continue;
        }
        archive.modelPoints.push_back(makePoint(
            line.values[0] / modelPositionScale,
            line.values[1] / modelPositionScale,
            line.values[2] / modelPositionScale));
    }

    const double teachPositionScale = positionScaleToMm(teachLines.front().values[0]);
    const double teachAngleScale =
        teachLines.front().values.size() >= 6 ? angleScaleToDeg(teachLines.front().values[3]) : 1.0;
    for (const auto &line : teachLines) {
        if (line.values.size() < 3) {
            continue;
        }
        archive.teachPositions.push_back(makePoint(
            line.values[0] / teachPositionScale,
            line.values[1] / teachPositionScale,
            line.values[2] / teachPositionScale));

        if (line.values.size() >= 6) {
            archive.teachOrientationsDeg.push_back(makePoint(
                line.values[3] / teachAngleScale,
                line.values[4] / teachAngleScale,
                line.values[5] / teachAngleScale));
        } else {
            archive.teachOrientationsDeg.push_back(makePoint(0.0, 0.0, 0.0));
        }

        robot_comm::StationAxisUpdateCommand axis;
        if (line.values.size() >= 8) {
            const double stationScale = angleScaleToDeg(line.values[6]);
            axis.s1Deg10000 = toLongRounded(line.values[6] / stationScale * 10000.0);
            axis.s2Deg10000 = toLongRounded(line.values[7] / stationScale * 10000.0);
            axis.valid = true;
        }
        archive.stationAxes.push_back(axis);
    }

    if (archive.modelPoints.size() < 5 || archive.teachPositions.empty()
        || archive.teachPositions.size() != archive.teachOrientationsDeg.size()) {
        archive.message = "离线纠偏存档有效点不足。";
        return archive;
    }

    archive.valid = true;
    archive.modelFilePath = modelPath.string();
    archive.teachFilePath = teachPath.string();
    archive.message = "离线纠偏存档读取完成。";
    return archive;
}

OfflineCorrectionTrajectoryMatchResult COfflineCorrectionTrajectoryMatcher::match(
    const OfflineCorrectionArchive &archive,
    const std::vector<OfflineCorrectionDetectionSample> &detectionSamples,
    const OfflineCorrectionOptions &options)
{
    OfflineCorrectionTrajectoryMatchResult result;
    if (!archive.valid) {
        result.message = archive.message.empty() ? "离线纠偏存档无效。" : archive.message;
        return result;
    }

    const auto prepared = prepareDetectionPoints(detectionSamples, options);
    const auto &detectionPoints = prepared.points;
    if (detectionPoints.size() < 5U) {
        result.message = "离线纠偏点清洗后不足 5 个：有效="
            + std::to_string(prepared.validCount)
            + "，降采样=" + std::to_string(prepared.spatialSampledCount)
            + "，聚类=" + std::to_string(prepared.clusteredCount)
            + "，最终=" + std::to_string(detectionPoints.size()) + "。";
        return result;
    }

    i2r_rlc::vision::IcpRegistrationOptions icpOptions;
    icpOptions.dimension = 3;
    icpOptions.maxIterations = options.maxIcpIterations;
    icpOptions.minDelta = options.minIcpDeltaMm;
    const auto icpResult = i2r_rlc::vision::CIcpRegistrationMatcher::alignPointToPoint(
        detectionPoints,
        archive.modelPoints,
        icpOptions);
    if (!icpResult.valid) {
        result.message = icpResult.message;
        return result;
    }

    result.diagnostics.valid = true;
    result.diagnostics.modelFilePath = archive.modelFilePath;
    result.diagnostics.teachFilePath = archive.teachFilePath;
    result.diagnostics.detectionPoints = detectionPoints;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            const std::size_t index = static_cast<std::size_t>(row * 4 + column);
            result.diagnostics.modelToDetectionTransformRowMajor[index] =
                icpResult.transformMatrix(row, column);
        }
    }
    result.diagnostics.transformedModelPoints.reserve(archive.modelPoints.size());
    for (const auto &modelPoint : archive.modelPoints) {
        result.diagnostics.transformedModelPoints.push_back(
            transformPosition(modelPoint, icpResult.transformMatrix));
    }

    result.robotCommand.requestTeachPointUpdate = true;
    result.robotCommand.offlineSignal = 4;
    result.robotCommand.pointCount = static_cast<int>(archive.teachPositions.size());
    result.robotCommand.teachPoints.reserve(archive.teachPositions.size());
    result.robotCommand.stationAxes.reserve(archive.stationAxes.size());
    result.diagnostics.transformedTeachPoints.reserve(archive.teachPositions.size());

    for (std::size_t index = 0; index < archive.teachPositions.size(); ++index) {
        const auto transformed = transformPosition(
            archive.teachPositions[index],
            icpResult.transformMatrix);
        result.diagnostics.transformedTeachPoints.push_back(transformed);
        const auto &orientation = archive.teachOrientationsDeg[index];

        robot_comm::TeachPointUpdateCommand teachPoint;
        teachPoint.xUm = toLongRounded(transformed.xMm * 1000.0);
        teachPoint.yUm = toLongRounded(transformed.yMm * 1000.0);
        teachPoint.zUm = toLongRounded(transformed.zMm * 1000.0);
        teachPoint.rxDeg10000 = toLongRounded(orientation.xMm * 10000.0);
        teachPoint.ryDeg10000 = toLongRounded(orientation.yMm * 10000.0);
        teachPoint.rzDeg10000 = toLongRounded(orientation.zMm * 10000.0);
        teachPoint.valid = true;
        result.robotCommand.teachPoints.push_back(teachPoint);

        if (index < archive.stationAxes.size()) {
            result.robotCommand.stationAxes.push_back(archive.stationAxes[index]);
        } else {
            result.robotCommand.stationAxes.push_back(robot_comm::StationAxisUpdateCommand{});
        }
    }

    result.robotCommand.valid = true;
    result.robotCommand.message = "离线纠偏示教点更新命令已生成。";
    result.report.valid = true;
    result.report.modelPointCount = static_cast<int>(archive.modelPoints.size());
    result.report.detectionPointCount = static_cast<int>(detectionPoints.size());
    result.report.teachPointCount = result.robotCommand.pointCount;
    result.report.meanErrorMm = icpResult.meanErrorMm;
    result.report.rmsErrorMm = icpResult.rmsErrorMm;
    result.report.confidence = icpResult.confidence;
    result.report.message = icpResult.message;
    result.valid = true;
    result.message = "离线纠偏有匹配计算完成：原始="
        + std::to_string(detectionSamples.size())
        + "，降采样=" + std::to_string(prepared.spatialSampledCount)
        + "，聚类=" + std::to_string(prepared.clusteredCount)
        + "，ICP=" + std::to_string(detectionPoints.size()) + "。";
    return result;
}

} // namespace i2r_rlc::control
