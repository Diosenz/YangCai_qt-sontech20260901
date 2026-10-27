/**
 * @file LocalizationSurfaceValidation.cpp
 * @brief 局部母材表面可观测模式投影与分帧验收，不替代生产 ICP。
 */
#include "LocalizationSurfaceValidation.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace i2r_rlc::localization::surface_validation {
namespace {

bool validOptions(const Options &options)
{
    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    const auto ratio = [&positive](double value) {
        return positive(value) && value <= 1.0;
    };
    return options.minimumPointCount >= 6
        && options.maximumPointCount >= options.minimumPointCount
        && options.maximumPointCount <= 4096
        && options.normalNeighborCount >= 6
        && options.normalNeighborCount <= options.minimumPointCount
        && ratio(options.minimumNormalSupportRatio)
        && ratio(options.maximumNormalPlanarityRatio)
        && positive(options.minimumNeighborhoodSecondVarianceMm2)
        && ratio(options.minimumRelativeModeStrength)
        && ratio(options.robustKeepRatio)
        && positive(options.coverageDistanceMm)
        && ratio(options.minimumBidirectionalCoverage)
        && positive(options.maximumResidualMm)
        && ratio(options.minimumImprovementRatio)
        && std::isfinite(options.maximumHoldoutWorseningMm)
        && options.maximumHoldoutWorseningMm >= 0.0;
}

bool validCloud(const std::vector<Point3D_mm> &points, const Options &options)
{
    return points.size() >= static_cast<std::size_t>(options.minimumPointCount)
        && points.size() <= static_cast<std::size_t>(options.maximumPointCount)
        && std::all_of(points.begin(), points.end(), [](const Point3D_mm &point) {
            return std::isfinite(point.x_mm) && std::isfinite(point.y_mm)
                && std::isfinite(point.z_mm);
        });
}

Eigen::Vector3d vector(const Point3D_mm &point)
{
    return {point.x_mm, point.y_mm, point.z_mm};
}

std::vector<double> nearestDistances(const std::vector<Point3D_mm> &source,
    const std::vector<Point3D_mm> &target)
{
    std::vector<double> result;
    result.reserve(source.size());
    for (const auto &point : source) {
        double best = std::numeric_limits<double>::max();
        for (const auto &other : target) {
            best = std::min(best, (vector(point) - vector(other)).squaredNorm());
        }
        result.push_back(std::sqrt(best));
    }
    return result;
}

double mean(const std::vector<double> &values, std::size_t count)
{
    double sum = 0.0;
    for (std::size_t index = 0; index < count; ++index) { sum += values[index]; }
    return sum / static_cast<double>(count);
}

double robustMean(std::vector<double> values, double keepRatio)
{
    std::sort(values.begin(), values.end());
    const auto count = std::max<std::size_t>(1,
        static_cast<std::size_t>(std::ceil(values.size() * keepRatio)));
    return mean(values, count);
}

Metrics measureClouds(const std::vector<Point3D_mm> &real,
    const std::vector<Point3D_mm> &reference, const Options &options)
{
    Metrics result;
    if (!validOptions(options) || !validCloud(real, options)
        || !validCloud(reference, options)) { return result; }
    const auto forward = nearestDistances(real, reference);
    const auto reverse = nearestDistances(reference, real);
    result.realToVirtualMeanMm = mean(forward, forward.size());
    result.virtualToRealMeanMm = mean(reverse, reverse.size());
    result.robustSymmetricResidualMm = 0.5
        * (robustMean(forward, options.robustKeepRatio)
            + robustMean(reverse, options.robustKeepRatio));
    const auto covered = [&options](const std::vector<double> &distances) {
        return static_cast<double>(std::count_if(distances.begin(), distances.end(),
            [&options](double distance) { return distance <= options.coverageDistanceMm; }))
            / static_cast<double>(distances.size());
    };
    result.realCoverageRatio = covered(forward);
    result.virtualCoverageRatio = covered(reverse);
    result.valid = std::isfinite(result.robustSymmetricResidualMm)
        && std::isfinite(result.realToVirtualMeanMm)
        && std::isfinite(result.virtualToRealMeanMm);
    return result;
}

bool readCandidate(const RigidTransform &candidate, Eigen::Matrix3d &rotation,
    Eigen::Vector3d &translation, double &yaw)
{
    const auto &matrix = candidate.rowMajor;
    if (!std::all_of(matrix.begin(), matrix.end(),
        [](double value) { return std::isfinite(value); })) { return false; }
    rotation << matrix[0], matrix[1], matrix[2], matrix[4], matrix[5], matrix[6],
        matrix[8], matrix[9], matrix[10];
    translation = {matrix[3], matrix[7], matrix[11]};
    yaw = std::atan2(matrix[4], matrix[0]);
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return (rotation - expected).norm() <= 1e-5
        && std::abs(matrix[12]) <= 1e-8 && std::abs(matrix[13]) <= 1e-8
        && std::abs(matrix[14]) <= 1e-8 && std::abs(matrix[15] - 1.0) <= 1e-8;
}

/**
 * @brief 从有局部二维展开的近邻求法向，拒绝近共线邻域及非平面局部。
 * @details 法向由图像重建点估计，未获得 CAD 面身份；重复点及狭窄切片可能
 * 降低支持率，此时拒绝比把数值噪声当完整几何约束更保守。
 */
bool information(const std::vector<Point3D_mm> &cloud,
    const Eigen::Vector3d &origin, double scale, const Options &options,
    Eigen::Matrix4d &projector, std::array<double, 4> &strengths)
{
    Eigen::Matrix4d informationMatrix = Eigen::Matrix4d::Zero();
    std::size_t normalCount = 0;
    for (const auto &point : cloud) {
        const Eigen::Vector3d center = vector(point);
        std::vector<std::pair<double, std::size_t>> nearest;
        nearest.reserve(cloud.size());
        for (std::size_t index = 0; index < cloud.size(); ++index) {
            nearest.emplace_back((vector(cloud[index]) - center).squaredNorm(), index);
        }
        const auto count = static_cast<std::size_t>(options.normalNeighborCount);
        if (nearest.size() > count) {
            std::nth_element(nearest.begin(), nearest.begin() + count, nearest.end());
            nearest.resize(count);
        }
        Eigen::Vector3d localMean = Eigen::Vector3d::Zero();
        for (const auto &item : nearest) { localMean += vector(cloud[item.second]); }
        localMean /= static_cast<double>(nearest.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &item : nearest) {
            const Eigen::Vector3d delta = vector(cloud[item.second]) - localMean;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(nearest.size());
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normalSolver(covariance);
        if (normalSolver.info() != Eigen::Success) { continue; }
        const auto values = normalSolver.eigenvalues();
        if (!values.allFinite()
            || values[1] < options.minimumNeighborhoodSecondVarianceMm2
            || values[0] > values[1] * options.maximumNormalPlanarityRatio) { continue; }
        const Eigen::Vector3d normal = normalSolver.eigenvectors().col(0);
        const Eigen::Vector3d lever = center - origin;
        Eigen::Vector4d row;
        row << normal.x(), normal.y(), normal.z(),
            (normal.y() * lever.x() - normal.x() * lever.y()) / scale;
        informationMatrix += row * row.transpose();
        ++normalCount;
    }
    if (normalCount < static_cast<std::size_t>(options.minimumPointCount)
        || static_cast<double>(normalCount) / cloud.size()
            < options.minimumNormalSupportRatio) { return false; }
    informationMatrix /= static_cast<double>(normalCount);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(informationMatrix);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()
        || solver.eigenvalues()[3] <= 1e-12) { return false; }
    projector.setZero();
    for (int index = 0; index < 4; ++index) {
        const double strength = std::sqrt(std::max(0.0,
            solver.eigenvalues()[index] / solver.eigenvalues()[3]));
        strengths[static_cast<std::size_t>(3 - index)] = strength;
        if (strength >= options.minimumRelativeModeStrength) {
            const Eigen::Vector4d mode = solver.eigenvectors().col(index);
            projector += mode * mode.transpose();
        }
    }
    return true;
}

RigidTransform writeTransform(const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation)
{
    RigidTransform result;
    auto &matrix = result.rowMajor;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            matrix[static_cast<std::size_t>(row * 4 + column)] = rotation(row, column);
        }
        matrix[static_cast<std::size_t>(row * 4 + 3)] = translation[row];
    }
    return result;
}

std::vector<Point3D_mm> transformed(const std::vector<Point3D_mm> &points,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
    std::vector<Point3D_mm> result;
    result.reserve(points.size());
    for (const auto &point : points) {
        const Eigen::Vector3d moved = rotation * vector(point) + translation;
        result.push_back({moved.x(), moved.y(), moved.z()});
    }
    return result;
}

bool goodCoverage(const Metrics &metrics, const Options &options)
{
    return metrics.valid && metrics.realCoverageRatio >= options.minimumBidirectionalCoverage
        && metrics.virtualCoverageRatio >= options.minimumBidirectionalCoverage
        && metrics.robustSymmetricResidualMm <= options.maximumResidualMm;
}

Result evaluateClouds(const Request &request)
{
    Result result;
    const auto &options = request.options;
    result.trainingBefore = measureClouds(request.trainingReal_mm, request.trainingVirtual_mm, options);
    result.holdoutBefore = measureClouds(request.holdoutReal_mm, request.holdoutVirtual_mm, options);
    if (!result.trainingBefore.valid || !result.holdoutBefore.valid) {
        result.message = QStringLiteral("表面训练/留出点云或参数无效，拒绝候选。");
        return result;
    }
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double yaw = 0.0;
    if (!readCandidate(request.candidateVirtualToReal, rotation, translation, yaw)) {
        result.message = QStringLiteral("表面候选不是有限 XYZ+Rz 刚体变换。");
        return result;
    }
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for (const auto &point : request.trainingVirtual_mm) { origin += vector(point); }
    origin /= static_cast<double>(request.trainingVirtual_mm.size());
    double spread = 0.0;
    for (const auto &point : request.trainingVirtual_mm) {
        spread += (vector(point).head<2>() - origin.head<2>()).squaredNorm();
    }
    const double scale = std::max(1.0,
        std::sqrt(spread / static_cast<double>(request.trainingVirtual_mm.size())));
    Eigen::Matrix4d realProjector;
    Eigen::Matrix4d virtualProjector;
    std::array<double, 4> realStrengths{};
    std::array<double, 4> virtualStrengths{};
    if (!information(request.trainingReal_mm, origin, scale, options, realProjector, realStrengths)
        || !information(request.trainingVirtual_mm, origin, scale, options,
            virtualProjector, virtualStrengths)) {
        result.message = QStringLiteral("母材法向支撑不足或局部近共线，拒绝表面候选。");
        return result;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> commonSolver(realProjector + virtualProjector);
    if (commonSolver.info() != Eigen::Success) {
        result.message = QStringLiteral("表面共同约束分解失败。");
        return result;
    }
    Eigen::Matrix4d commonProjector = Eigen::Matrix4d::Zero();
    for (int index = 0; index < 4; ++index) {
        result.relativeModeStrengths[static_cast<std::size_t>(index)] = std::min(
            realStrengths[static_cast<std::size_t>(index)],
            virtualStrengths[static_cast<std::size_t>(index)]);
        // 两个已筛选子空间都须支持该模式；容忍小角度差但不相加制造满秩。
        if (commonSolver.eigenvalues()[index] >= 1.98) {
            const Eigen::Vector4d mode = commonSolver.eigenvectors().col(index);
            commonProjector += mode * mode.transpose();
            ++result.observableRank;
        }
    }
    if (result.observableRank == 0) {
        result.message = QStringLiteral("真实/虚拟母材没有共同可用修正方向。");
        return result;
    }
    Eigen::Vector4d candidate;
    candidate.head<3>() = rotation * origin + translation - origin;
    candidate[3] = scale * yaw;
    const Eigen::Vector4d observed = commonProjector * candidate;
    const double observedYaw = observed[3] / scale;
    const Eigen::Matrix3d observedRotation =
        Eigen::AngleAxisd(observedYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d observedTranslation = origin + observed.head<3>() - observedRotation * origin;
    result.observableVirtualToReal = writeTransform(observedRotation, observedTranslation);
    result.hasObservableCorrection = true;
    result.projected = (candidate - observed).norm() > 1e-6;
    result.trainingAfter = measureClouds(request.trainingReal_mm,
        transformed(request.trainingVirtual_mm, observedRotation, observedTranslation), options);
    result.holdoutAfter = measureClouds(request.holdoutReal_mm,
        transformed(request.holdoutVirtual_mm, observedRotation, observedTranslation), options);
    result.accepted = goodCoverage(result.trainingAfter, options)
        && goodCoverage(result.holdoutAfter, options)
        && result.trainingBefore.robustSymmetricResidualMm > 1e-6
        && result.trainingAfter.robustSymmetricResidualMm
            <= result.trainingBefore.robustSymmetricResidualMm * (1.0 - options.minimumImprovementRatio)
        && result.holdoutAfter.robustSymmetricResidualMm
            <= result.holdoutBefore.robustSymmetricResidualMm + options.maximumHoldoutWorseningMm;
    if (!result.accepted && options.allowTrainingCoverageRecovery) {
        auto trainingOptions = options;
        // 仅给训练覆盖 5 个百分点余量；强收益和完整留出约束不以覆盖放宽替代。
        trainingOptions.minimumBidirectionalCoverage = std::max(0.01,
            options.minimumBidirectionalCoverage - 0.05);
        const bool strongGain = result.trainingBefore.robustSymmetricResidualMm > 1e-6
            && result.trainingAfter.robustSymmetricResidualMm
                <= result.trainingBefore.robustSymmetricResidualMm
                    * (1.0 - std::max(0.15, options.minimumImprovementRatio));
        result.trainingCoverageRecoveryUsed = goodCoverage(result.trainingAfter, trainingOptions)
            && goodCoverage(result.holdoutAfter, options) && strongGain
            && result.holdoutAfter.robustSymmetricResidualMm
                <= result.holdoutBefore.robustSymmetricResidualMm + options.maximumHoldoutWorseningMm;
        result.accepted = result.trainingCoverageRecoveryUsed;
    }
    result.message = result.trainingCoverageRecoveryUsed
        ? QStringLiteral("训练覆盖轻微不足、强收益及完整留出通过，采用受限恢复候选；仍待写后新帧验证。")
        : result.accepted
        ? QStringLiteral("表面共同可观测方向投影及分帧留出验收通过；冻结切片近似，仍待写回后新帧验证。")
        : QStringLiteral("投影后表面收益、双向覆盖或分帧留出验收未通过。");
    return result;
}

} // namespace

PairedSurfaceMetrics compareFreshSurface(const std::vector<Point3D_mm> &freshReal,
    const std::vector<Point3D_mm> &frozenVirtual, const RigidTransform &correction,
    const Options &options)
{
    PairedSurfaceMetrics result;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double yaw = 0.0;
    if (!validOptions(options) || !validCloud(freshReal, options)
        || !validCloud(frozenVirtual, options)
        || !readCandidate(correction, rotation, translation, yaw)) { return result; }
    struct Patch {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
    };
    std::vector<Patch> patches;
    const auto count = static_cast<std::size_t>(options.normalNeighborCount);
    for (const auto &point : frozenVirtual) {
        std::vector<std::pair<double, std::size_t>> nearest;
        for (std::size_t index = 0; index < frozenVirtual.size(); ++index) {
            nearest.emplace_back((vector(point) - vector(frozenVirtual[index])).squaredNorm(), index);
        }
        if (nearest.size() > count) {
            std::nth_element(nearest.begin(), nearest.begin() + count, nearest.end());
            nearest.resize(count);
        }
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto &item : nearest) { center += vector(frozenVirtual[item.second]); }
        center /= static_cast<double>(nearest.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &item : nearest) {
            const Eigen::Vector3d delta = vector(frozenVirtual[item.second]) - center;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(nearest.size());
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()
            || solver.eigenvalues()[1] < options.minimumNeighborhoodSecondVarianceMm2
            || solver.eigenvalues()[0] > solver.eigenvalues()[1]
                * options.maximumNormalPlanarityRatio) { continue; }
        patches.push_back({center, solver.eigenvectors().col(0)});
    }
    if (patches.size() < static_cast<std::size_t>(options.minimumPointCount)
        || static_cast<double>(patches.size()) / frozenVirtual.size()
            < options.minimumNormalSupportRatio) { return result; }
    const double radiusSquared = options.coverageDistanceMm * options.coverageDistanceMm;
    for (const auto &point : freshReal) {
        const Eigen::Vector3d real = vector(point);
        const Patch *selected = nullptr;
        double nearest = std::numeric_limits<double>::max();
        // 同一个冻结面身份同时支持原位和候选，禁止两侧各自选择有利的不同面。
        for (const auto &patch : patches) {
            const double beforeDistance = (real - patch.center).squaredNorm();
            const double afterDistance = (real - (rotation * patch.center + translation)).squaredNorm();
            if (beforeDistance <= radiusSquared && afterDistance <= radiusSquared
                && afterDistance < nearest) {
                nearest = afterDistance;
                selected = &patch;
            }
        }
        if (selected == nullptr) { continue; }
        result.previousResidualMm += std::abs(selected->normal.dot(real - selected->center));
        result.currentResidualMm += std::abs((rotation * selected->normal).dot(
            real - (rotation * selected->center + translation)));
        ++result.supportedPointCount;
    }
    result.supportedRatio = static_cast<double>(result.supportedPointCount) / freshReal.size();
    if (result.supportedPointCount > 0) {
        result.previousResidualMm /= result.supportedPointCount;
        result.currentResidualMm /= result.supportedPointCount;
    }
    result.valid = result.supportedPointCount >= options.minimumPointCount
        && result.supportedRatio >= options.minimumBidirectionalCoverage
        && std::isfinite(result.previousResidualMm) && std::isfinite(result.currentResidualMm);
    return result;
}

PairedSurfaceMetrics compareFreshSurfaceWithCurrentReference(
    const std::vector<Point3D_mm> &freshReal,
    const std::vector<Point3D_mm> &currentVirtual,
    const std::vector<Point3D_mm> &frozenVirtual,
    const RigidTransform &correction, const Options &options)
{
    PairedSurfaceMetrics result;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double yaw = 0.0;
    if (!validOptions(options) || !validCloud(freshReal, options)
        || !validCloud(currentVirtual, options) || !validCloud(frozenVirtual, options)
        || !readCandidate(correction, rotation, translation, yaw)
        || !std::isfinite(options.maximumModelContinuationMm)
        || options.maximumModelContinuationMm <= 0.0
        || options.maximumModelContinuationMm > 10.0
        || !std::isfinite(options.maximumModelPlaneDeviationMm)
        || options.maximumModelPlaneDeviationMm <= 0.0
        || options.maximumModelPlaneDeviationMm > 0.05) { return result; }

    struct Patch {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        Eigen::Vector3d currentCenter;
        Eigen::Vector3d currentNormal;
        Eigen::Matrix<double, 3, 2> currentTangents;
        Eigen::Vector2d minimumTangent;
        Eigen::Vector2d maximumTangent;
    };
    std::vector<Patch> patches;
    const auto count = static_cast<std::size_t>(options.normalNeighborCount);
    for (const auto &point : frozenVirtual) {
        std::vector<std::pair<double, std::size_t>> nearest;
        nearest.reserve(frozenVirtual.size());
        for (std::size_t index = 0; index < frozenVirtual.size(); ++index) {
            nearest.emplace_back((vector(point) - vector(frozenVirtual[index])).squaredNorm(), index);
        }
        if (nearest.size() > count) {
            std::nth_element(nearest.begin(), nearest.begin() + count, nearest.end());
            nearest.resize(count);
        }
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto &item : nearest) { center += vector(frozenVirtual[item.second]); }
        center /= static_cast<double>(nearest.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &item : nearest) {
            const Eigen::Vector3d delta = vector(frozenVirtual[item.second]) - center;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(nearest.size());
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()
            || solver.eigenvalues()[1] < options.minimumNeighborhoodSecondVarianceMm2
            || solver.eigenvalues()[0] > solver.eigenvalues()[1]
                * options.maximumNormalPlanarityRatio) { continue; }
        const Eigen::Vector3d normal = solver.eigenvectors().col(0);
        const Eigen::Matrix<double, 3, 2> tangents = solver.eigenvectors().rightCols<2>();
        Eigen::Vector2d minimumTangent = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector2d maximumTangent = -minimumTangent;
        for (const auto &item : nearest) {
            const Eigen::Vector2d coordinate = tangents.transpose()
                * (vector(frozenVirtual[item.second]) - center);
            minimumTangent = minimumTangent.cwiseMin(coordinate);
            maximumTangent = maximumTangent.cwiseMax(coordinate);
        }
        patches.push_back({center, normal, rotation * center + translation, rotation * normal,
            rotation * tangents, minimumTangent, maximumTangent});
    }
    result.frozenPatchCount = static_cast<int>(patches.size());
    result.status = FreshSurfaceComparisonStatus::InsufficientFrozenSupport;
    if (patches.size() < static_cast<std::size_t>(options.minimumPointCount)
        || static_cast<double>(patches.size()) / frozenVirtual.size()
            < options.minimumNormalSupportRatio) { return result; }

    // 只由当前模型确定对应面；不能按真实点选择最有利的候选面。
    // 法向容差独立于真实覆盖；切向足迹仅由冻结近邻确定，距足迹外延最多10mm。
    // 使用到矩形的欧氏距离，避免两个切轴各放宽10mm形成更大的对角预算。
    const double continuationSquared = options.maximumModelContinuationMm
        * options.maximumModelContinuationMm;
    std::vector<const Patch *> currentPatches(currentVirtual.size(), nullptr);
    for (std::size_t index = 0; index < currentVirtual.size(); ++index) {
        double bestTangent = std::numeric_limits<double>::max();
        for (const auto &patch : patches) {
            const Eigen::Vector3d delta = vector(currentVirtual[index]) - patch.currentCenter;
            const double normalDistance = std::abs(patch.currentNormal.dot(delta));
            const Eigen::Vector2d tangentCoordinate = patch.currentTangents.transpose() * delta;
            const Eigen::Vector2d outside = (patch.minimumTangent - tangentCoordinate)
                .cwiseMax(tangentCoordinate - patch.maximumTangent).cwiseMax(0.0);
            const double continuationDistanceSquared = outside.squaredNorm();
            const double tangentSquared = tangentCoordinate.squaredNorm();
            if (normalDistance <= options.maximumModelPlaneDeviationMm
                && continuationDistanceSquared <= continuationSquared && tangentSquared < bestTangent) {
                currentPatches[index] = &patch;
                bestTangent = tangentSquared;
            }
        }
        if (currentPatches[index] != nullptr) { ++result.currentModelSupportedPointCount; }
    }
    result.currentModelSupportedRatio = static_cast<double>(result.currentModelSupportedPointCount)
        / currentVirtual.size();
    const double coverageSquared = options.coverageDistanceMm * options.coverageDistanceMm;
    for (const auto &point : freshReal) {
        const Eigen::Vector3d real = vector(point);
        double nearestDistance = std::numeric_limits<double>::max();
        std::size_t nearestIndex = 0;
        for (std::size_t index = 0; index < currentVirtual.size(); ++index) {
            const double distance = (real - vector(currentVirtual[index])).squaredNorm();
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestIndex = index;
            }
        }
        const Patch *selected = currentPatches[nearestIndex];
        if (nearestDistance > coverageSquared || selected == nullptr) { continue; }
        result.previousResidualMm += std::abs(selected->normal.dot(real - selected->center));
        result.currentResidualMm += std::abs(selected->currentNormal.dot(real - selected->currentCenter));
        ++result.supportedPointCount;
    }
    result.supportedRatio = static_cast<double>(result.supportedPointCount) / freshReal.size();
    if (result.supportedPointCount > 0) {
        result.previousResidualMm /= result.supportedPointCount;
        result.currentResidualMm /= result.supportedPointCount;
    }
    const bool modelSupported = result.currentModelSupportedPointCount >= options.minimumPointCount
        && result.currentModelSupportedRatio >= options.minimumBidirectionalCoverage;
    result.valid = modelSupported && result.supportedPointCount >= options.minimumPointCount
        && result.supportedRatio >= options.minimumBidirectionalCoverage
        && std::isfinite(result.previousResidualMm) && std::isfinite(result.currentResidualMm);
    result.status = result.valid ? FreshSurfaceComparisonStatus::Succeeded
        : (modelSupported ? FreshSurfaceComparisonStatus::InsufficientRealSupport
            : FreshSurfaceComparisonStatus::InsufficientCurrentModelSupport);
    return result;
}

Metrics measure(const std::vector<Point3D_mm> &real,
    const std::vector<Point3D_mm> &reference, const Options &options)
{
    try { return measureClouds(real, reference, options); }
    catch (const std::exception &) { return {}; }
}

Result evaluate(const Request &request)
{
    try { return evaluateClouds(request); }
    catch (const std::exception &) {
        Result result;
        result.message = QStringLiteral("表面验收计算异常，拒绝本次候选。");
        return result;
    }
}

} // namespace i2r_rlc::localization::surface_validation
