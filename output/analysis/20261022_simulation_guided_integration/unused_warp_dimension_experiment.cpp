/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationService.cpp
 * @brief     定位服务实现。
 * @details   本文件实现全局和局部受限 ICP 求解，PCL 类型仅在模块内部使用。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

#include "LocalizationService.h"

namespace i2r_rlc::localization {

namespace {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

class RestrictedRigidWarp final : public pcl::registration::WarpPointRigid<Point, Point, float> {
public:
    explicit RestrictedRigidWarp(const IcpDofMask& dofMask)
        : pcl::registration::WarpPointRigid<Point, Point, float>(parameterCount(dofMask))
        , m_dofMask(dofMask)
    {
    }

    void setParam(const Eigen::VectorXf& parameter) override
    {
        // 冻结自由度不能仍占优化参数：恒零雅可比列会使LM在纯轴向位移时停在单位阵。
        // 优化器只看允许模式，内部再展开为既有XYZ+四元数向量分量语义。
        Eigen::VectorXf masked = Eigen::VectorXf::Zero(6);
        const std::array<bool, 6> allowed{m_dofMask.allowX, m_dofMask.allowY,
            m_dofMask.allowZ, m_dofMask.allowRx, m_dofMask.allowRy, m_dofMask.allowRz};
        int sourceIndex = 0;
        for (std::size_t axis = 0; axis < allowed.size(); ++axis) {
            if (allowed[axis]) { masked[static_cast<int>(axis)] = parameter[sourceIndex++]; }
        }

        this->transform_matrix_.setZero();
        this->transform_matrix_(0, 3) = masked[0];
        this->transform_matrix_(1, 3) = masked[1];
        this->transform_matrix_(2, 3) = masked[2];
        this->transform_matrix_(3, 3) = 1.0F;

        const float rotationNormSquared =
            masked[3] * masked[3] + masked[4] * masked[4] + masked[5] * masked[5];
        Eigen::Quaternion<float> quaternion(0.0F, masked[3], masked[4], masked[5]);
        quaternion.w() = std::sqrt(std::max(0.0F, 1.0F - rotationNormSquared));
        quaternion.normalize();
        this->transform_matrix_.topLeftCorner(3, 3) = quaternion.toRotationMatrix();
    }

private:
    static int parameterCount(const IcpDofMask &mask)
    {
        const int count = static_cast<int>(mask.allowX) + static_cast<int>(mask.allowY)
            + static_cast<int>(mask.allowZ) + static_cast<int>(mask.allowRx)
            + static_cast<int>(mask.allowRy) + static_cast<int>(mask.allowRz);
        // 全冻结保留一个无作用参数，兼容旧调用的单位修正，避免零维LM输入。
        return std::max(1, count);
    }

    IcpDofMask m_dofMask;
};

CloudPtr toCloud(const std::vector<Point3D_mm>& points)
{
    auto cloud = CloudPtr(new Cloud);
    cloud->points.reserve(points.size());
    for (const Point3D_mm& point : points) {
        cloud->push_back(Point{
            static_cast<float>(point.x_mm),
            static_cast<float>(point.y_mm),
            static_cast<float>(point.z_mm)
        });
    }

    cloud->is_dense = true;
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    return cloud;
}

std::vector<Point3D_mm> fromCloud(const Cloud& cloud)
{
    std::vector<Point3D_mm> points;
    points.reserve(cloud.size());
    for (const Point& point : cloud.points) {
        points.push_back({point.x, point.y, point.z});
    }
    return points;
}

void downsampleByStride(CloudPtr cloud, const int targetPointCount)
{
    if (targetPointCount <= 0 || static_cast<int>(cloud->size()) <= targetPointCount) {
        return;
    }

    Cloud sampled;
    sampled.points.reserve(static_cast<std::size_t>(targetPointCount));

    const double step =
        static_cast<double>(cloud->size()) / static_cast<double>(targetPointCount);
    for (int index = 0; index < targetPointCount; ++index) {
        const auto sourceIndex = static_cast<std::size_t>(
            std::min<double>(std::floor(index * step), static_cast<double>(cloud->size() - 1)));
        sampled.push_back((*cloud)[sourceIndex]);
    }

    sampled.is_dense = true;
    sampled.width = static_cast<std::uint32_t>(sampled.size());
    sampled.height = 1;
    *cloud = sampled;
}

void downsampleByFarthestPoint(CloudPtr cloud, const int targetPointCount)
{
    if (targetPointCount <= 0 || targetPointCount > static_cast<int>(cloud->size())) {
        return;
    }

    const int rawPointCount = static_cast<int>(cloud->size());
    std::vector<int> selectedIndices(static_cast<std::size_t>(targetPointCount), 0);
    std::vector<bool> selected(static_cast<std::size_t>(rawPointCount), false);
    std::vector<double> nearestDistanceSquared(
        static_cast<std::size_t>(rawPointCount),
        std::numeric_limits<double>::max());

    int previousIndex = 0;
    selected[0] = true;
    for (int index = 1; index < targetPointCount; ++index) {
        const Point& previousPoint = (*cloud)[static_cast<std::size_t>(previousIndex)];
        double bestDistanceSquared = 0.0;
        int bestIndex = 0;

        for (int candidateIndex = 0; candidateIndex < rawPointCount; ++candidateIndex) {
            if (selected[static_cast<std::size_t>(candidateIndex)]) {
                continue;
            }

            const Point& candidate = (*cloud)[static_cast<std::size_t>(candidateIndex)];
            const double dx = candidate.x - previousPoint.x;
            const double dy = candidate.y - previousPoint.y;
            const double dz = candidate.z - previousPoint.z;
            const double distanceSquared = dx * dx + dy * dy + dz * dz;
            double& nearest = nearestDistanceSquared[static_cast<std::size_t>(candidateIndex)];
            if (distanceSquared < nearest) {
                nearest = distanceSquared;
            }
            if (nearest > bestDistanceSquared) {
                bestDistanceSquared = nearest;
                bestIndex = candidateIndex;
            }
        }

        previousIndex = bestIndex;
        selectedIndices[static_cast<std::size_t>(index)] = bestIndex;
        selected[static_cast<std::size_t>(bestIndex)] = true;
    }

    Cloud sampled;
    sampled.points.reserve(selectedIndices.size());
    for (const int selectedIndex : selectedIndices) {
        sampled.push_back((*cloud)[static_cast<std::size_t>(selectedIndex)]);
    }

    sampled.is_dense = true;
    sampled.width = static_cast<std::uint32_t>(sampled.size());
    sampled.height = 1;
    *cloud = sampled;
}

void applySorFilter(CloudPtr cloud, const IcpParameters& parameters)
{
    if (!parameters.enableSorFilter || parameters.sorMeanK <= 1
        || static_cast<int>(cloud->size()) <= parameters.sorMeanK) {
        return;
    }

    pcl::StatisticalOutlierRemoval<Point> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(parameters.sorMeanK);
    filter.setStddevMulThresh(parameters.sorStddevMulThresh);

    Cloud filtered;
    filter.filter(filtered);
    filtered.is_dense = true;
    filtered.width = static_cast<std::uint32_t>(filtered.size());
    filtered.height = 1;
    *cloud = filtered;
}

void applyCompatibilityFirstLocalIcpPreprocessing(CloudPtr source)
{
    // 兼容旧版局部 ICP 回放数据：只处理真实点云，执行 FPS -> SOR -> FPS。
    downsampleByFarthestPoint(source, static_cast<int>(source->size()) / 5);

    IcpParameters sorParameters;
    sorParameters.enableSorFilter = true;
    sorParameters.sorMeanK = 5;
    sorParameters.sorStddevMulThresh = 2.0;
    applySorFilter(source, sorParameters);

    downsampleByFarthestPoint(source, static_cast<int>(source->size()) / 5);
}

void applyCompatibilitySwv1GlobalIcpPreprocessing(CloudPtr source)
{
    // SWv1 stepICPforOffline(): 真实点云 FPS 保留 1/5，再做 MeanK=5、
    // StddevMulThresh=2 的统计离群点剔除；第二次 FPS 在旧代码中已注释。
    downsampleByFarthestPoint(source, static_cast<int>(source->size()) / 5);

    IcpParameters sorParameters;
    sorParameters.enableSorFilter = true;
    sorParameters.sorMeanK = 5;
    sorParameters.sorStddevMulThresh = 2.0;
    applySorFilter(source, sorParameters);
}

Eigen::Matrix4f toEigenMatrix(const RigidTransform& transform)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) = static_cast<float>(transform.rowMajor[row * 4 + column]);
        }
    }
    return matrix;
}

RigidTransform toRigidTransform(const Eigen::Matrix4f& matrix)
{
    RigidTransform transform;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            transform.rowMajor[row * 4 + column] = matrix(row, column);
        }
    }
    return transform;
}

Pose6D toPose6D(const Eigen::Matrix4f& matrix)
{
    const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0).cast<double>();

    double rx_rad = 0.0;
    double ry_rad = 0.0;
    double rz_rad = 0.0;
    const double sy = -rotation(2, 0);
    if (std::abs(sy) < 1.0 - std::numeric_limits<double>::epsilon()) {
        ry_rad = std::asin(sy);
        rx_rad = std::atan2(rotation(2, 1), rotation(2, 2));
        rz_rad = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        ry_rad = sy > 0.0 ? 3.14159265358979323846 / 2.0 : -3.14159265358979323846 / 2.0;
        rx_rad = 0.0;
        rz_rad = std::atan2(-rotation(0, 1), rotation(1, 1));
    }

    return {
        matrix(0, 3),
        matrix(1, 3),
        matrix(2, 3),
        rx_rad * kRadiansToDegrees,
        ry_rad * kRadiansToDegrees,
        rz_rad * kRadiansToDegrees
    };
}

LocalizationResult makeBaseResult(
    const LocalizationRequest& request,
    const int realPointCount,
    const int referencePointCount)
{
    LocalizationResult result;
    result.mode = request.mode;
    result.frameId = request.frameId;
    result.realPointCount = realPointCount;
    result.virtualPointCount = referencePointCount;
    result.diagnostics.sourcePointCount = realPointCount;
    result.diagnostics.targetPointCount = referencePointCount;
    return result;
}

LocalizationResult failResult(
    const LocalizationRequest& request,
    const int realPointCount,
    const int referencePointCount,
    const QString& message)
{
    LocalizationResult result = makeBaseResult(request, realPointCount, referencePointCount);
    result.status = LocalizationStatus::Failed;
    result.message = message;
    return result;
}

LocalizationResult runRestrictedIcp(
    const LocalizationRequest& request,
    const std::vector<Point3D_mm>& realPoints,
    const std::vector<Point3D_mm>& referencePoints,
    const IcpDofMask& dofMask,
    const IcpParameters& parameters,
    const RigidTransform& initialGuess)
{
    const int realPointCount = static_cast<int>(realPoints.size());
    const int referencePointCount = static_cast<int>(referencePoints.size());
    const int minPointCount = std::max(3, parameters.minPointCount);
    if (realPointCount < minPointCount || referencePointCount < minPointCount) {
        return failResult(
            request,
            realPointCount,
            referencePointCount,
            QStringLiteral("ICP 点云不足：真实点 %1，参考点 %2，最少需要 %3。")
                .arg(realPointCount)
                .arg(referencePointCount)
                .arg(minPointCount));
    }

    CloudPtr source = toCloud(realPoints);
    CloudPtr target = toCloud(referencePoints);
    if (parameters.preprocessingProfile == IcpPreprocessingProfile::CompatibilityFirstLocalIcp) {
        applyCompatibilityFirstLocalIcpPreprocessing(source);
    } else if (parameters.preprocessingProfile
               == IcpPreprocessingProfile::CompatibilitySwv1GlobalIcp) {
        applyCompatibilitySwv1GlobalIcpPreprocessing(source);
    } else {
        downsampleByStride(source, parameters.downsampleTargetPointCount);
        downsampleByStride(target, parameters.downsampleTargetPointCount);
        applySorFilter(source, parameters);
    }

    if (static_cast<int>(source->size()) < minPointCount || static_cast<int>(target->size()) < minPointCount) {
        LocalizationResult result = failResult(
            request,
            realPointCount,
            referencePointCount,
            QStringLiteral("ICP 预处理后点云不足：真实点 %1，参考点 %2。")
                .arg(static_cast<int>(source->size()))
                .arg(static_cast<int>(target->size())));
        result.diagnostics.filteredSourcePointCount = static_cast<int>(source->size());
        result.preprocessedRealPoints_mm = fromCloud(*source);
        return result;
    }

    using TransformationEstimation =
        pcl::registration::TransformationEstimationLM<Point, Point, float>;

    pcl::IterativeClosestPointNonLinear<Point, Point> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    if (parameters.maximumIterations > 0) {
        icp.setMaximumIterations(parameters.maximumIterations);
    }
    icp.setTransformationEpsilon(parameters.transformationEpsilon);
    if (parameters.maxCorrespondenceDistance_mm > 0.0) {
        icp.setMaxCorrespondenceDistance(parameters.maxCorrespondenceDistance_mm);
    }

    auto estimation = TransformationEstimation::Ptr(new TransformationEstimation);
    auto warp = pcl::registration::WarpPointRigid<Point, Point, float>::Ptr(
        new RestrictedRigidWarp(dofMask));
    estimation->setWarpFunction(warp);
    icp.setTransformationEstimation(estimation);

    Cloud aligned;
    icp.align(aligned, toEigenMatrix(initialGuess));

    LocalizationResult result = makeBaseResult(request, realPointCount, referencePointCount);
    result.diagnostics.filteredSourcePointCount = static_cast<int>(source->size());
    result.preprocessedRealPoints_mm = fromCloud(*source);
    result.iterationCount = 0;

    if (!icp.hasConverged()) {
        result.diagnostics.converged = false;
        result.status = LocalizationStatus::Failed;
        result.message = QStringLiteral("ICP 未收敛。");
        return result;
    }

    Eigen::Matrix4f transform = icp.getFinalTransformation();
    double selectedFitnessScore_mm2 = icp.getFitnessScore();
    result.diagnostics.coarseFitnessScore_mm2 = selectedFitnessScore_mm2;

    if (parameters.preprocessingProfile
        == IcpPreprocessingProfile::CompatibilitySwv1GlobalIcp
        && std::isfinite(selectedFitnessScore_mm2)) {
        // SWv1 兼容粗配准负责扩大收敛域；粗配准成功后，以其完整变换为初值，
        // 收紧对应距离再做一次精配准。精配准失败或全点 RMSE 变差时保留粗配准。
        const double coarseRmse_mm =
            std::sqrt(std::max(0.0, selectedFitnessScore_mm2));
        const double refinementMaxCorrespondenceDistance_mm = std::clamp(
            coarseRmse_mm * 2.5,
            3.0,
            20.0);
        result.diagnostics.refinementAttempted = true;
        result.diagnostics.refinementMaxCorrespondenceDistance_mm =
            refinementMaxCorrespondenceDistance_mm;

        icp.setMaxCorrespondenceDistance(
            refinementMaxCorrespondenceDistance_mm);
        const int refinementIterations = parameters.maximumIterations > 0
            ? std::min(parameters.maximumIterations, 1000)
            : 1000;
        icp.setMaximumIterations(refinementIterations);
        if (parameters.transformationEpsilon > 0.0) {
            icp.setTransformationEpsilon(
                std::min(parameters.transformationEpsilon, 1e-7));
        }

        Cloud refined;
        icp.align(refined, transform);
        result.diagnostics.refinementConverged = icp.hasConverged();
        if (icp.hasConverged()) {
            const double refinementFitnessScore_mm2 = icp.getFitnessScore();
            result.diagnostics.refinementFitnessScore_mm2 =
                refinementFitnessScore_mm2;
            if (std::isfinite(refinementFitnessScore_mm2)
                && refinementFitnessScore_mm2 <= selectedFitnessScore_mm2) {
                transform = icp.getFinalTransformation();
                selectedFitnessScore_mm2 = refinementFitnessScore_mm2;
                result.diagnostics.refinementAccepted = true;
            }
        }
    }

    result.diagnostics.converged = true;
    result.diagnostics.fitnessScore_mm2 = selectedFitnessScore_mm2;
    const Pose6D correction = toPose6D(transform);

    result.status = LocalizationStatus::Succeeded;
    result.hasPoseCorrection = true;
    result.correctionX_mm = correction.x_mm;
    result.correctionY_mm = correction.y_mm;
    result.correctionZ_mm = correction.z_mm;
    result.correctionRx_deg = correction.rx_deg;
    result.correctionRy_deg = correction.ry_deg;
    result.correctionRz_deg = correction.rz_deg;
    result.residual_mm = std::sqrt(std::max(0.0, result.diagnostics.fitnessScore_mm2));
    result.correctionTransform = toRigidTransform(transform);
    result.message = request.mode == LocalizationMode::Global
        ? QStringLiteral("全局受限 ICP 定位完成。")
        : QStringLiteral("局部滑窗 ICP 修正完成。");

    return result;
}

} // namespace

LocalizationStartResult LocalizationService::start()
{
    return {
        LocalizationServiceState::Idle,
        QStringLiteral("LocalizationService ready：全局 ICP 与局部滑窗 ICP 可用于离线验证。")
    };
}

LocalizationResult LocalizationService::runLocalization(const LocalizationRequest& request)
{
    if (request.mode == LocalizationMode::Global) {
        return runRestrictedIcp(
            request,
            request.globalIcp.realPoints_mm,
            request.globalIcp.referencePoints_mm,
            request.globalIcp.dofMask,
            request.globalIcp.parameters,
            request.globalIcp.initialGuess);
    }

    return runRestrictedIcp(
        request,
        request.localCorrection.realWindowPoints_mm,
        request.localCorrection.referenceWindowPoints_mm,
        request.localCorrection.dofMask,
        request.localCorrection.parameters,
        RigidTransform{});
}

} // namespace i2r_rlc::localization
