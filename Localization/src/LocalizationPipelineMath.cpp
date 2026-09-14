/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationPipelineMath.cpp
 * @brief     定位流程内部几何计算实现。
 * @details   本文件实现 pipeline 共享的点云残差和刚体变换辅助函数。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/LU>

#include "LocalizationPipelineMath.h"

namespace i2r_rlc::localization::pipeline_math {

namespace {

double squaredDistance(const Point3D_mm& first, const Point3D_mm& second)
{
    const double dx_mm = first.x_mm - second.x_mm;
    const double dy_mm = first.y_mm - second.y_mm;
    const double dz_mm = first.z_mm - second.z_mm;
    return dx_mm * dx_mm + dy_mm * dy_mm + dz_mm * dz_mm;
}

Eigen::Matrix4d toEigenMatrix(const RigidTransform& transform)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) =
                transform.rowMajor[static_cast<std::size_t>(row * 4 + column)];
        }
    }
    return matrix;
}

RigidTransform toRigidTransform(const Eigen::Matrix4d& matrix)
{
    RigidTransform transform;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            transform.rowMajor[static_cast<std::size_t>(row * 4 + column)] = matrix(row, column);
        }
    }
    return transform;
}

Point3D_mm transformPoint(const Point3D_mm& point, const Eigen::Matrix4d& transform)
{
    const Eigen::Vector4d input(point.x_mm, point.y_mm, point.z_mm, 1.0);
    const Eigen::Vector4d output = transform * input;
    return {output.x(), output.y(), output.z()};
}

} // namespace

double meanNearestDistance(
    const std::vector<Point3D_mm>& source,
    const std::vector<Point3D_mm>& target)
{
    if (source.empty() || target.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double totalDistance_mm = 0.0;
    for (const Point3D_mm& sourcePoint : source) {
        double bestDistanceSquared_mm2 = std::numeric_limits<double>::max();
        for (const Point3D_mm& targetPoint : target) {
            bestDistanceSquared_mm2 = std::min(
                bestDistanceSquared_mm2,
                squaredDistance(sourcePoint, targetPoint));
        }
        totalDistance_mm += std::sqrt(bestDistanceSquared_mm2);
    }

    return totalDistance_mm / static_cast<double>(source.size());
}

double nearestNeighborRmse(
    const std::vector<Point3D_mm>& source,
    const std::vector<Point3D_mm>& target)
{
    if (source.empty() || target.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double totalSquaredDistance_mm2 = 0.0;
    for (const Point3D_mm& sourcePoint : source) {
        double bestDistanceSquared_mm2 = std::numeric_limits<double>::max();
        for (const Point3D_mm& targetPoint : target) {
            bestDistanceSquared_mm2 = std::min(
                bestDistanceSquared_mm2,
                squaredDistance(sourcePoint, targetPoint));
        }
        totalSquaredDistance_mm2 += bestDistanceSquared_mm2;
    }

    return std::sqrt(
        totalSquaredDistance_mm2 / static_cast<double>(source.size())
    );
}

RigidTransform inverseRigidTransform(const RigidTransform& transform)
{
    return toRigidTransform(toEigenMatrix(transform).inverse());
}

std::vector<Point3D_mm> transformPoints(
    const std::vector<Point3D_mm>& points,
    const RigidTransform& transform)
{
    const Eigen::Matrix4d matrix = toEigenMatrix(transform);
    std::vector<Point3D_mm> transformed;
    transformed.reserve(points.size());
    for (const Point3D_mm& point : points) {
        transformed.push_back(transformPoint(point, matrix));
    }

    return transformed;
}

RigidTransform composeRigidTransforms(
    const RigidTransform& lhs,
    const RigidTransform& rhs)
{
    return toRigidTransform(toEigenMatrix(lhs) * toEigenMatrix(rhs));
}

} // namespace i2r_rlc::localization::pipeline_math
