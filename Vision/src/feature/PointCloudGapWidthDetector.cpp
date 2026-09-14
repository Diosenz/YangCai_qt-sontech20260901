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
 * @file        PointCloudGapWidthDetector.cpp
 * @brief       点云焊缝宽度检测实现
 * @details     使用 PCL 完成点云滤波和聚类，使用 Eigen 完成 PCA frame、
 *              局部二维拟合和宽度计算，不依赖 HALCON。
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
#include <numeric>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "feature/PointCloudGapWidthDetector.h"

namespace i2r_rlc::vision {
namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

/**
 * @brief 局部截面坐标中的点。
 *
 * `p2` 表示 PCA 截面二维坐标，单位 mm；`p3` 保留原三维点，单位 mm。
 */
struct LocalPoint2D {
    Eigen::Vector2d p2 = Eigen::Vector2d::Zero();
    Eigen::Vector3d p3 = Eigen::Vector3d::Zero();
};

/**
 * @brief 判断空间点是否可参与点云宽度计算。
 * @param point 输入空间点，坐标单位 mm。
 * @return true 表示标记有效且三维坐标有限。
 */
bool isFinitePoint(const WeldSpacePoint &point)
{
    return point.valid &&
        std::isfinite(point.xMm) &&
        std::isfinite(point.yMm) &&
        std::isfinite(point.zMm);
}

/**
 * @brief 将 Vision 空间点转换为 PCL 点。
 * @param point 输入空间点，坐标单位 mm。
 * @return PCL `PointXYZ`。
 */
PointT toPclPoint(const WeldSpacePoint &point)
{
    PointT pclPoint;
    pclPoint.x = static_cast<float>(point.xMm);
    pclPoint.y = static_cast<float>(point.yMm);
    pclPoint.z = static_cast<float>(point.zMm);
    return pclPoint;
}

/**
 * @brief 将 Eigen 三维点转换为 Vision 空间点。
 * @param point 输入三维点，坐标单位 mm。
 * @return 标记为有效的 WeldSpacePoint。
 */
WeldSpacePoint toWeldSpacePoint(const Eigen::Vector3d &point)
{
    WeldSpacePoint result;
    result.xMm = point.x();
    result.yMm = point.y();
    result.zMm = point.z();
    result.valid = true;
    return result;
}

/**
 * @brief 将 PCL 点转换为 Eigen 向量。
 * @param point PCL 点，坐标单位 mm。
 * @return Eigen 三维向量。
 */
Eigen::Vector3d toEigen(const PointT &point)
{
    return Eigen::Vector3d(
        static_cast<double>(point.x),
        static_cast<double>(point.y),
        static_cast<double>(point.z));
}

/**
 * @brief 从 Vision 点云提取有限点并转换为 PCL 点云。
 * @param pointCloud 输入点云，坐标单位 mm。
 * @return PCL 点云智能指针；无有效点时为空点云。
 */
CloudT::Ptr toPclCloud(const WeldPointCloud &pointCloud)
{
    CloudT::Ptr cloud(new CloudT);
    cloud->height = 1;
    cloud->is_dense = false;
    for (const auto &point : pointCloud.points) {
        if (isFinitePoint(point)) {
            cloud->push_back(toPclPoint(point));
        }
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    return cloud;
}

/**
 * @brief 按参考点附近窗口过滤点云。
 * @param cloud 输入 PCL 点云。
 * @param referencePoint 参考焊缝点，坐标单位 mm。
 * @param options 窗口半宽参数，单位 mm。
 * @return 窗口过滤后的点云。
 */
CloudT::Ptr applyReferenceWindow(
    const CloudT::Ptr &cloud,
    const WeldSpacePoint &referencePoint,
    const PointCloudGapWidthOptions &options)
{
    CloudT::Ptr filtered(new CloudT);
    if (!cloud || cloud->empty() || !isFinitePoint(referencePoint)) {
        return filtered;
    }

    for (const auto &point : cloud->points) {
        if (std::abs(static_cast<double>(point.x) - referencePoint.xMm) <=
                options.referenceWindowHalfXmm &&
            std::abs(static_cast<double>(point.y) - referencePoint.yMm) <=
                options.referenceWindowHalfYmm &&
            std::abs(static_cast<double>(point.z) - referencePoint.zMm) <=
                options.referenceWindowHalfZmm) {
            filtered->push_back(point);
        }
    }
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
}

/**
 * @brief 执行 PCL 统计离群点滤波。
 * @param cloud 输入 PCL 点云。
 * @param options SOR 参数。
 * @return 滤波后的点云；输入不足时返回输入拷贝或空点云。
 */
CloudT::Ptr applySor(
    const CloudT::Ptr &cloud,
    const PointCloudGapWidthOptions &options)
{
    CloudT::Ptr filtered(new CloudT);
    if (!cloud || cloud->empty()) {
        return filtered;
    }
    if (static_cast<int>(cloud->size()) <= std::max(2, options.sorMeanK)) {
        *filtered = *cloud;
        return filtered;
    }

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(std::max(2, options.sorMeanK));
    sor.setStddevMulThresh(options.sorStddevMulThresh);
    sor.setNegative(false);
    sor.filter(*filtered);
    return filtered;
}

PointCloudGapFrame computePcaFrame(const CloudT::Ptr &cloud)
{
    PointCloudGapFrame frame;
    if (!cloud || cloud->size() < 3) {
        return frame;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &point : cloud->points) {
        centroid += toEigen(point);
    }
    centroid /= static_cast<double>(cloud->size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : cloud->points) {
        const Eigen::Vector3d centered = toEigen(point) - centroid;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(std::max<size_t>(1, cloud->size() - 1));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return frame;
    }

    Eigen::Vector3d e1 = solver.eigenvectors().col(2).normalized();
    Eigen::Vector3d e2 = solver.eigenvectors().col(1).normalized();
    Eigen::Vector3d e3 = solver.eigenvectors().col(0).normalized();

    if (e1.head<2>().squaredNorm() < e2.head<2>().squaredNorm()) {
        std::swap(e1, e2);
    }
    e2 = e3.cross(e1).normalized();

    frame.e1 = e1;
    frame.e2 = e2;
    frame.e3 = e3;
    frame.valid = true;
    return frame;
}

bool extractTwoLargestClusters(
    const CloudT::Ptr &cloud,
    const PointCloudGapWidthOptions &options,
    CloudT::Ptr *leftCluster,
    CloudT::Ptr *rightCluster)
{
    leftCluster->reset(new CloudT);
    rightCluster->reset(new CloudT);
    if (!cloud || cloud->empty()) {
        return false;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setSearchMethod(tree);
    extractor.setClusterTolerance(options.clusterToleranceMm);
    extractor.setMinClusterSize(std::max(1, options.minClusterPointCount));
    extractor.setMaxClusterSize(static_cast<int>(cloud->size()));
    extractor.extract(clusterIndices);
    if (clusterIndices.size() < 2) {
        return false;
    }

    std::sort(
        clusterIndices.begin(), clusterIndices.end(),
        [](const pcl::PointIndices &left, const pcl::PointIndices &right) {
            return left.indices.size() > right.indices.size();
        });

    const auto assignCluster =
        [](const CloudT::Ptr &source,
           const pcl::PointIndices &indices,
           const CloudT::Ptr &target) {
            target->clear();
            for (const int index : indices.indices) {
                target->push_back(source->points[static_cast<size_t>(index)]);
            }
            target->width = static_cast<std::uint32_t>(target->size());
            target->height = 1;
            target->is_dense = false;
        };

    const auto meanIndex = [](const pcl::PointIndices &indices) {
        const int sum = std::accumulate(
            indices.indices.begin(), indices.indices.end(), 0);
        return static_cast<double>(sum) /
            static_cast<double>(std::max<size_t>(1, indices.indices.size()));
    };

    const pcl::PointIndices *first = &clusterIndices[0];
    const pcl::PointIndices *second = &clusterIndices[1];
    if (meanIndex(*first) > meanIndex(*second)) {
        std::swap(first, second);
    }

    assignCluster(cloud, *first, *leftCluster);
    assignCluster(cloud, *second, *rightCluster);
    return !(*leftCluster)->empty() && !(*rightCluster)->empty();
}

std::vector<LocalPoint2D> projectToLocal(
    const CloudT::Ptr &cloud,
    const Eigen::Vector3d &origin,
    const Eigen::Vector3d &widthAxis,
    const Eigen::Vector3d &tangentAxis)
{
    std::vector<LocalPoint2D> points;
    if (!cloud) {
        return points;
    }
    points.reserve(cloud->size());
    for (const auto &point : cloud->points) {
        const Eigen::Vector3d p3 = toEigen(point);
        const Eigen::Vector3d delta = p3 - origin;
        LocalPoint2D localPoint;
        localPoint.p2.x() = delta.dot(tangentAxis);
        localPoint.p2.y() = delta.dot(widthAxis);
        localPoint.p3 = p3;
        points.push_back(localPoint);
    }
    return points;
}

bool fitLine2D(
    const std::vector<Eigen::Vector2d> &points,
    Eigen::Vector2d *linePoint,
    Eigen::Vector2d *direction)
{
    if (points.size() < 2) {
        return false;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &point : points) {
        centroid += point;
    }
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto &point : points) {
        const Eigen::Vector2d centered = point - centroid;
        covariance += centered * centered.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    *linePoint = centroid;
    *direction = solver.eigenvectors().col(1).normalized();
    return true;
}

bool computeWidthFromClusters(
    const CloudT::Ptr &leftCluster,
    const CloudT::Ptr &rightCluster,
    const PointCloudGapFrame &frame,
    const WeldSpacePoint &referenceWeldPoint,
    const PointCloudGapWidthOptions &options,
    PointCloudGapWidthResult *result)
{
    if (!frame.valid || !isFinitePoint(referenceWeldPoint)) {
        result->message = "点云局部坐标系或参考焊缝点无效";
        return false;
    }

    const Eigen::Vector3d origin(
        referenceWeldPoint.xMm,
        referenceWeldPoint.yMm,
        referenceWeldPoint.zMm);
    auto left2D = projectToLocal(leftCluster, origin, frame.e1, frame.e2);
    auto right2D = projectToLocal(rightCluster, origin, frame.e1, frame.e2);
    if (left2D.size() < 2 || right2D.size() < 2) {
        result->message = "局部投影点数不足";
        return false;
    }

    const auto byNorm = [](const LocalPoint2D &left, const LocalPoint2D &right) {
        return left.p2.squaredNorm() < right.p2.squaredNorm();
    };
    std::sort(left2D.begin(), left2D.end(), byNorm);
    std::sort(right2D.begin(), right2D.end(), byNorm);

    const int leftNeighborCount = std::min<int>(
        std::max(2, options.localNeighborCount),
        static_cast<int>(left2D.size()));
    const int rightNeighborCount = std::min<int>(
        std::max(2, options.localNeighborCount),
        static_cast<int>(right2D.size()));

    std::vector<Eigen::Vector2d> leftNeighbors;
    std::vector<Eigen::Vector2d> rightNeighbors;
    leftNeighbors.reserve(static_cast<size_t>(leftNeighborCount));
    rightNeighbors.reserve(static_cast<size_t>(rightNeighborCount));
    for (int i = 0; i < leftNeighborCount; ++i) {
        leftNeighbors.push_back(left2D[static_cast<size_t>(i)].p2);
    }
    for (int i = 0; i < rightNeighborCount; ++i) {
        rightNeighbors.push_back(right2D[static_cast<size_t>(i)].p2);
    }

    Eigen::Vector2d leftLinePoint;
    Eigen::Vector2d leftDirection;
    Eigen::Vector2d rightLinePoint;
    Eigen::Vector2d rightDirection;
    if (!fitLine2D(leftNeighbors, &leftLinePoint, &leftDirection) ||
        !fitLine2D(rightNeighbors, &rightLinePoint, &rightDirection)) {
        result->message = "左右边界局部直线拟合失败";
        return false;
    }

    if (leftDirection.dot(rightDirection) < 0.0) {
        rightDirection = -rightDirection;
    }

    const Eigen::Vector2d qLeft = left2D.front().p2;
    const Eigen::Vector2d qRight = right2D.front().p2;
    const Eigen::Vector2d leftRepresentative =
        leftLinePoint + (qLeft - leftLinePoint).dot(leftDirection) *
        leftDirection;
    const Eigen::Vector2d rightRepresentative =
        rightLinePoint + (qRight - rightLinePoint).dot(rightDirection) *
        rightDirection;

    const Eigen::Vector2d leftStar =
        leftLinePoint +
        (rightRepresentative - leftLinePoint).dot(leftDirection) *
            leftDirection;

    result->gapWidthMm = (leftRepresentative - leftStar).norm();
    result->leftBoundaryPoint = toWeldSpacePoint(
        origin +
        frame.e2 * leftRepresentative.x() +
        frame.e1 * leftRepresentative.y());
    result->rightBoundaryPoint = toWeldSpacePoint(
        origin +
        frame.e2 * rightRepresentative.x() +
        frame.e1 * rightRepresentative.y());
    result->boundaryParallelScore =
        std::min(1.0, std::abs(leftDirection.dot(rightDirection)));
    return std::isfinite(result->gapWidthMm);
}

} // namespace

PointCloudGapWidthResult CPointCloudGapWidthDetector::detect(
    const WeldPointCloud &pointCloud,
    const WeldSpacePoint &referenceWeldPoint,
    const PointCloudGapWidthOptions &options)
{
    PointCloudGapWidthResult result;
    CloudT::Ptr cloud = toPclCloud(pointCloud);
    result.inputPointCount = static_cast<int>(cloud->size());

    if (result.inputPointCount < std::max(2, options.minInputPointCount)) {
        result.message = "输入点云有效点数不足";
        return result;
    }
    if (!isFinitePoint(referenceWeldPoint)) {
        result.message = "参考焊缝空间点无效";
        return result;
    }

    CloudT::Ptr filtered = options.enableReferenceWindowFilter
        ? applyReferenceWindow(cloud, referenceWeldPoint, options)
        : cloud;
    filtered = applySor(filtered, options);
    result.filteredPointCount = static_cast<int>(filtered->size());
    if (result.filteredPointCount < std::max(2, options.minInputPointCount)) {
        result.message = "滤波后点数不足";
        return result;
    }

    result.frame = computePcaFrame(filtered);
    if (!result.frame.valid) {
        result.message = "PCA 局部坐标系计算失败";
        return result;
    }

    CloudT::Ptr leftCluster;
    CloudT::Ptr rightCluster;
    if (!extractTwoLargestClusters(
            filtered, options, &leftCluster, &rightCluster)) {
        result.message = "点云聚类未得到左右两簇";
        return result;
    }

    result.leftClusterPointCount = static_cast<int>(leftCluster->size());
    result.rightClusterPointCount = static_cast<int>(rightCluster->size());
    if (!computeWidthFromClusters(
            leftCluster, rightCluster, result.frame, referenceWeldPoint,
            options, &result)) {
        return result;
    }

    const double clusterBalance =
        static_cast<double>(
            std::min(result.leftClusterPointCount, result.rightClusterPointCount)) /
        static_cast<double>(
            std::max(result.leftClusterPointCount, result.rightClusterPointCount));
    const double pointRatio =
        static_cast<double>(result.filteredPointCount) /
        static_cast<double>(std::max(result.inputPointCount, 1));

    result.confidence = std::max(
        0.0,
        std::min(1.0, 0.45 * result.boundaryParallelScore +
                      0.35 * clusterBalance +
                      0.20 * std::min(1.0, pointRatio)));
    result.valid = std::isfinite(result.gapWidthMm) && result.gapWidthMm >= 0.0;
    result.message = result.valid
        ? "点云焊缝宽度检测成功"
        : "点云焊缝宽度无效";
    return result;
}

} // namespace i2r_rlc::vision
