/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        IcpRegistrationMatcher.cpp
 * @brief       libicp 配准封装实现
 * @details     将 Vision 几何点集转换为 libicp 连续数组，调用旧 ICP 内核，
 *              再把 R/t/scale 和残差统计转换回新工程公共结构。
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
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

#include <Eigen/Core>

#include "geometry/IcpRegistrationMatcher.h"
#include "libicp/icpPointToPlane.h"
#include "libicp/icpPointToPoint.h"

namespace i2r_rlc::vision {
namespace {

/**
 * @brief libicp 输入前的有效点和连续坐标缓冲区。
 *
 * `points` 保留原始 mm 坐标用于结果统计，`values` 按 libicp 要求展开为
 * `[x,y,(z)]` 连续 double 数组。
 */
struct PreparedCloud {
    std::vector<WeldSpacePoint> points;
    std::vector<double> values;
};

/**
 * @brief 将外部维度选项标准化为 libicp 支持的 2D/3D。
 * @param options 配准选项。
 * @return `2` 或 `3`，非 2D 输入默认按 3D 处理。
 */
int normalizeDimension(const IcpRegistrationOptions &options)
{
    return options.dimension == 2 ? 2 : 3;
}

/**
 * @brief 过滤无效点并展开为 libicp 连续数组。
 * @param points 输入点集，坐标单位为 mm。
 * @param dimension 目标维度，必须为 2 或 3。
 * @return 可直接传给 libicp 的点云缓冲。
 */
PreparedCloud prepareCloud(const std::vector<WeldSpacePoint> &points, const int dimension)
{
    PreparedCloud prepared;
    prepared.values.reserve(points.size() * static_cast<std::size_t>(dimension));
    for (const auto &point : points) {
        if (!point.valid) {
            continue;
        }
        prepared.points.push_back(point);
        prepared.values.push_back(point.xMm);
        prepared.values.push_back(point.yMm);
        if (dimension == 3) {
            prepared.values.push_back(point.zMm);
        }
    }
    return prepared;
}

/**
 * @brief 创建 libicp 使用的单位旋转矩阵。
 * @param dimension 矩阵维度，必须为 2 或 3。
 * @return `dimension x dimension` 单位矩阵。
 */
RMatrix::Matrix makeIdentityMatrix(const int dimension)
{
    return RMatrix::Matrix::eye(dimension);
}

/**
 * @brief 创建 libicp 使用的零平移向量。
 * @param dimension 向量维度，必须为 2 或 3。
 * @return `dimension x 1` 零矩阵。
 */
RMatrix::Matrix makeZeroTranslation(const int dimension)
{
    return RMatrix::Matrix(dimension, 1);
}

/**
 * @brief 将 libicp R/t 转换为齐次 4x4 矩阵。
 * @param rotation libicp 输出旋转或仿射矩阵。
 * @param translation libicp 输出平移向量，单位 mm。
 * @param dimension 有效维度，2D 时 z 轴保持单位变换。
 * @return Vision 公共结构使用的 4x4 变换矩阵。
 */
Eigen::Matrix4d toMatrix4d(
    const RMatrix::Matrix &rotation,
    const RMatrix::Matrix &translation,
    const int dimension)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (int row = 0; row < dimension; ++row) {
        for (int col = 0; col < dimension; ++col) {
            matrix(row, col) = rotation.val[row][col];
        }
        matrix(row, 3) = translation.val[row][0];
    }
    return matrix;
}

/**
 * @brief 从 libicp 尺度矩阵读取三维尺度向量。
 * @param scale libicp 输出尺度矩阵。
 * @param dimension 有效维度，未使用维度保持 1。
 * @return 三维尺度向量。
 */
Eigen::Vector3d toScaleVector(const RMatrix::Matrix &scale, const int dimension)
{
    Eigen::Vector3d result = Eigen::Vector3d::Ones();
    for (int i = 0; i < dimension; ++i) {
        result(i) = scale.val[i][i];
    }
    return result;
}

/**
 * @brief 将 4x4 变换矩阵拆成 WeldRigidTransform。
 * @param matrix 齐次变换矩阵，平移单位为 mm。
 * @return 标记为 valid 的刚体变换。
 */
WeldRigidTransform toRigidTransform(const Eigen::Matrix4d &matrix)
{
    WeldRigidTransform transform;
    transform.rotation = matrix.block<3, 3>(0, 0);
    transform.translationMm = matrix.block<3, 1>(0, 3);
    transform.valid = true;
    return transform;
}

/**
 * @brief 将焊缝空间点转为 Eigen 向量。
 * @param point 输入点，坐标单位为 mm。
 * @return 三维向量。
 */
Eigen::Vector3d toVector3d(const WeldSpacePoint &point)
{
    return {point.xMm, point.yMm, point.zMm};
}

/**
 * @brief 按指定维度计算平方欧氏距离。
 * @param lhs 左点，坐标单位为 mm。
 * @param rhs 右点，坐标单位为 mm。
 * @param dimension 使用的维度，2D 时忽略 z。
 * @return 平方距离，单位 mm^2。
 */
double squaredDistanceByDimension(
    const Eigen::Vector3d &lhs,
    const Eigen::Vector3d &rhs,
    const int dimension)
{
    double distance = 0.0;
    for (int i = 0; i < dimension; ++i) {
        const double delta = lhs(i) - rhs(i);
        distance += delta * delta;
    }
    return distance;
}

/**
 * @brief 根据最近邻残差填充 ICP 结果统计。
 * @param result 输入输出配准结果。
 * @param targetPoints 目标/模型点集，坐标单位为 mm。
 * @param sourcePoints 源点集，坐标单位为 mm。
 * @param dimension 残差统计使用维度。
 * @param inlierDistanceMm 内点距离阈值，单位 mm；非正数表示所有点计为内点。
 */
void fillResidualStatistics(
    IcpRegistrationResult &result,
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const int dimension,
    const double inlierDistanceMm)
{
    if (targetPoints.empty() || sourcePoints.empty()) {
        return;
    }

    double minError = std::numeric_limits<double>::max();
    double maxError = 0.0;
    double sumError = 0.0;
    double sumSquaredError = 0.0;
    int inlierCount = 0;
    int validCount = 0;

    for (const auto &sourcePoint : sourcePoints) {
        const Eigen::Vector4d source(
            sourcePoint.xMm,
            sourcePoint.yMm,
            dimension == 2 ? 0.0 : sourcePoint.zMm,
            1.0);
        const Eigen::Vector3d transformed =
            (result.transformMatrix * source).head<3>();

        double nearestSquared = std::numeric_limits<double>::max();
        for (const auto &targetPoint : targetPoints) {
            nearestSquared = std::min(
                nearestSquared,
                squaredDistanceByDimension(
                    transformed,
                    toVector3d(targetPoint),
                    dimension));
        }

        const double error = std::sqrt(nearestSquared);
        minError = std::min(minError, error);
        maxError = std::max(maxError, error);
        sumError += error;
        sumSquaredError += error * error;
        ++validCount;
        if (inlierDistanceMm <= 0.0 || error <= inlierDistanceMm) {
            ++inlierCount;
        }
    }

    if (validCount == 0) {
        return;
    }

    result.minErrorMm = minError;
    result.maxErrorMm = maxError;
    result.meanErrorMm = sumError / static_cast<double>(validCount);
    result.rmsErrorMm = std::sqrt(sumSquaredError / static_cast<double>(validCount));
    result.inlierRatio =
        static_cast<double>(inlierCount) / static_cast<double>(validCount);
    result.confidence =
        std::max(0.0, std::min(1.0, result.inlierRatio / (1.0 + result.meanErrorMm)));
}

/**
 * @brief 将公共选项中的 point-to-point 兼容开关传入旧 libicp。
 * @param icp 点到点 ICP 实例。
 * @param options 公共配准选项。
 */
void applyPointToPointFlags(
    CIcpPointToPoint &icp,
    const IcpRegistrationOptions &options)
{
    if (options.keepRotation) {
        icp.keepRotation();
    }
    if (options.keepConnect) {
        icp.keepConnect();
    }
    if (options.keepRemoveFailPoint) {
        icp.keepRemoveFailPoint();
    }
}

/**
 * @brief 执行公共入口共享的 ICP 调度流程。
 * @param targetPoints 目标/模型点集，坐标单位为 mm。
 * @param sourcePoints 待配准点集，坐标单位为 mm。
 * @param options 配准选项。
 * @param algorithm 要调用的旧 libicp 算法分支。
 * @return 标准化后的配准结果和残差统计。
 */
IcpRegistrationResult runIcp(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options,
    const IcpRegistrationAlgorithm algorithm)
{
    IcpRegistrationResult result;
    const int dimension = normalizeDimension(options);

    // 旧 libicp 的仿射分支只完成了二维矩阵方程，三维输入在这里显式拦截。
    if (dimension == 3
        && (algorithm == IcpRegistrationAlgorithm::Affine
            || algorithm == IcpRegistrationAlgorithm::AffineBidirectionalDistance)) {
        result.message = "旧 libicp 的 affine/affineBDD 三维分支矩阵方程不完整，请使用二维输入或 point-to-point/scale 三维配准。";
        return result;
    }

    const auto target = prepareCloud(targetPoints, dimension);
    const auto source = prepareCloud(sourcePoints, dimension);

    // libicp 要求模型点和模板点数量足够稳定建立最近邻与线性方程。
    if (target.points.size() < 5 || source.points.size() < 5) {
        result.message = "ICP 输入有效点数不足，至少需要 5 个目标点和 5 个源点。";
        return result;
    }

    RMatrix::Matrix rotation = makeIdentityMatrix(dimension);
    RMatrix::Matrix translation = makeZeroTranslation(dimension);
    RMatrix::Matrix scale = makeIdentityMatrix(dimension);
    double maxDistance = 0.0;
    double minDistance = 0.0;
    double meanDistance = 0.0;
    double rmsDistance = 0.0;

    // 按算法选择旧 libicp 分支；公共结果结构在调用后统一转换。
    if (algorithm == IcpRegistrationAlgorithm::PointToPlane) {
        CIcpPointToPlane icp(
            const_cast<double *>(target.values.data()),
            static_cast<int32_t>(target.points.size()),
            dimension,
            options.maxIterations,
            options.minDelta,
            options.normalNeighborCount,
            options.normalFlatness);
        icp.fit(
            const_cast<double *>(source.values.data()),
            static_cast<int32_t>(source.points.size()),
            rotation,
            translation,
            options.inlierDistanceMm);
    } else {
        CIcpPointToPoint icp(
            const_cast<double *>(target.values.data()),
            static_cast<int32_t>(target.points.size()),
            dimension,
            options.maxIterations,
            options.minDelta);
        applyPointToPointFlags(icp, options);

        switch (algorithm) {
        case IcpRegistrationAlgorithm::PointToPoint:
            icp.fit(
                const_cast<double *>(source.values.data()),
                static_cast<int32_t>(source.points.size()),
                rotation,
                translation,
                options.inlierDistanceMm);
            break;
        case IcpRegistrationAlgorithm::Affine:
            icp.fitAffine(
                const_cast<double *>(source.values.data()),
                static_cast<int32_t>(source.points.size()),
                rotation,
                translation,
                options.inlierDistanceMm,
                maxDistance,
                minDistance,
                meanDistance,
                rmsDistance);
            break;
        case IcpRegistrationAlgorithm::AffineBidirectionalDistance:
            icp.fitAffineBDD(
                const_cast<double *>(source.values.data()),
                static_cast<int32_t>(source.points.size()),
                rotation,
                translation,
                options.inlierDistanceMm,
                maxDistance,
                minDistance,
                meanDistance,
                rmsDistance);
            break;
        case IcpRegistrationAlgorithm::BidirectionalDistance:
            icp.fitBDD(
                const_cast<double *>(source.values.data()),
                static_cast<int32_t>(source.points.size()),
                rotation,
                translation,
                options.inlierDistanceMm);
            break;
        case IcpRegistrationAlgorithm::WithScale:
            icp.fit_scale(
                const_cast<double *>(source.values.data()),
                static_cast<int32_t>(source.points.size()),
                rotation,
                translation,
                scale,
                options.inlierDistanceMm);
            break;
        case IcpRegistrationAlgorithm::PointToPlane:
            break;
        }
    }

    // 将旧 RMatrix 输出转换回新工程的 Eigen/Vision 类型，并补充尺度。
    result.valid = true;
    result.transformMatrix = toMatrix4d(rotation, translation, dimension);
    if (algorithm == IcpRegistrationAlgorithm::WithScale) {
        result.scale = toScaleVector(scale, dimension);
        for (int i = 0; i < dimension; ++i) {
            result.transformMatrix(i, i) *= result.scale(i);
        }
    }
    result.transform = toRigidTransform(result.transformMatrix);
    if (maxDistance > 0.0 || meanDistance > 0.0 || rmsDistance > 0.0) {
        result.maxErrorMm = maxDistance;
        result.minErrorMm = minDistance;
        result.meanErrorMm = meanDistance;
        result.rmsErrorMm = rmsDistance;
    } else {
        // 旧接口未返回残差统计的分支，使用变换后的源点到目标最近邻距离补算。
        fillResidualStatistics(
            result,
            target.points,
            source.points,
            dimension,
            options.inlierDistanceMm);
    }
    if (result.confidence <= 0.0) {
        result.confidence =
            std::max(0.0, std::min(1.0, 1.0 / (1.0 + result.meanErrorMm)));
    }
    result.message = "ICP 配准完成。";
    return result;
}

/**
 * @brief 从点云结构中提取有效空间点。
 * @param pointCloud 输入点云，坐标单位为 mm。
 * @return 仅包含 `valid=true` 的点序列。
 */
std::vector<WeldSpacePoint> validPointsFromCloud(const WeldPointCloud &pointCloud)
{
    std::vector<WeldSpacePoint> points;
    points.reserve(pointCloud.points.size());
    for (const auto &point : pointCloud.points) {
        if (point.valid) {
            points.push_back(point);
        }
    }
    return points;
}

} // namespace

IcpRegistrationResult CIcpRegistrationMatcher::alignPointToPoint(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options)
{
    return runIcp(
        targetPoints,
        sourcePoints,
        options,
        IcpRegistrationAlgorithm::PointToPoint);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignPointToPlane(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options)
{
    return runIcp(
        targetPoints,
        sourcePoints,
        options,
        IcpRegistrationAlgorithm::PointToPlane);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignAffine(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options)
{
    return runIcp(targetPoints, sourcePoints, options, IcpRegistrationAlgorithm::Affine);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignAffineBidirectionalDistance(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options)
{
    return runIcp(
        targetPoints,
        sourcePoints,
        options,
        IcpRegistrationAlgorithm::AffineBidirectionalDistance);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignBidirectionalDistance(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options)
{
    return runIcp(
        targetPoints,
        sourcePoints,
        options,
        IcpRegistrationAlgorithm::BidirectionalDistance);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignWithScale(
    const std::vector<WeldSpacePoint> &targetPoints,
    const std::vector<WeldSpacePoint> &sourcePoints,
    const IcpRegistrationOptions &options)
{
    return runIcp(
        targetPoints,
        sourcePoints,
        options,
        IcpRegistrationAlgorithm::WithScale);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignPointToPoint(
    const WeldPointCloud &targetPointCloud,
    const WeldPointCloud &sourcePointCloud,
    const IcpRegistrationOptions &options)
{
    return alignPointToPoint(
        validPointsFromCloud(targetPointCloud),
        validPointsFromCloud(sourcePointCloud),
        options);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignPointToPlane(
    const WeldPointCloud &targetPointCloud,
    const WeldPointCloud &sourcePointCloud,
    const IcpRegistrationOptions &options)
{
    return alignPointToPlane(
        validPointsFromCloud(targetPointCloud),
        validPointsFromCloud(sourcePointCloud),
        options);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignAffine(
    const WeldPointCloud &targetPointCloud,
    const WeldPointCloud &sourcePointCloud,
    const IcpRegistrationOptions &options)
{
    return alignAffine(
        validPointsFromCloud(targetPointCloud),
        validPointsFromCloud(sourcePointCloud),
        options);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignAffineBidirectionalDistance(
    const WeldPointCloud &targetPointCloud,
    const WeldPointCloud &sourcePointCloud,
    const IcpRegistrationOptions &options)
{
    return alignAffineBidirectionalDistance(
        validPointsFromCloud(targetPointCloud),
        validPointsFromCloud(sourcePointCloud),
        options);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignBidirectionalDistance(
    const WeldPointCloud &targetPointCloud,
    const WeldPointCloud &sourcePointCloud,
    const IcpRegistrationOptions &options)
{
    return alignBidirectionalDistance(
        validPointsFromCloud(targetPointCloud),
        validPointsFromCloud(sourcePointCloud),
        options);
}

IcpRegistrationResult CIcpRegistrationMatcher::alignWithScale(
    const WeldPointCloud &targetPointCloud,
    const WeldPointCloud &sourcePointCloud,
    const IcpRegistrationOptions &options)
{
    return alignWithScale(
        validPointsFromCloud(targetPointCloud),
        validPointsFromCloud(sourcePointCloud),
        options);
}

} // namespace i2r_rlc::vision
