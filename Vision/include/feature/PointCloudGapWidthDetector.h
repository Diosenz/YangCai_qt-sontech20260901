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
 * @file        PointCloudGapWidthDetector.h
 * @brief       点云焊缝宽度检测接口
 * @details     承接旧 GapDetector 中的点云滤波、聚类、PCA 局部截面和
 *              宽度计算流程。公共接口不暴露 PCL 类型。
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
#ifndef I2R_RLC_VISION_FEATURE_POINT_CLOUD_GAP_WIDTH_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_POINT_CLOUD_GAP_WIDTH_DETECTOR_H

#include <string>

#include <Eigen/Core>

#include "geometry/WeldGeometryTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 点云局部焊缝宽度坐标系。
 *
 * e1 为宽度方向，e2 为截面内切向方向，e3 为点云所在局部平面法向。
 * 向量均在输入点云坐标系下表达。
 */
struct PointCloudGapFrame {
    /// @brief 宽度方向单位向量。
    Eigen::Vector3d e1 = Eigen::Vector3d::UnitX();
    /// @brief 截面切向单位向量。
    Eigen::Vector3d e2 = Eigen::Vector3d::UnitY();
    /// @brief 局部平面法向单位向量。
    Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
    /// @brief true 表示三个方向向量可用于宽度计算。
    bool valid = false;
};

/**
 * @brief 点云焊缝宽度检测参数。
 *
 * 点云坐标单位统一为 mm，距离阈值同样使用 mm。
 */
struct PointCloudGapWidthOptions {
    /// @brief 统计离群点滤波近邻数量。
    int sorMeanK = 20;

    /// @brief 统计离群点滤波标准差倍数。
    double sorStddevMulThresh = 1.0;

    /// @brief 欧式聚类距离阈值，单位 mm。
    double clusterToleranceMm = 0.5;

    /// @brief 聚类最小点数。
    int minClusterPointCount = 10;

    /// @brief 局部左右边界拟合使用的最近邻点数。
    int localNeighborCount = 20;

    /// @brief 输入点云最少有效点数。
    int minInputPointCount = 12;

    /// @brief 是否启用参考点附近窗口过滤。
    bool enableReferenceWindowFilter = false;

    /// @brief 参考点附近 X 半宽，单位 mm。
    double referenceWindowHalfXmm = 5.0;

    /// @brief 参考点附近 Y 半宽，单位 mm。
    double referenceWindowHalfYmm = 5.0;

    /// @brief 参考点附近 Z 半宽，单位 mm。
    double referenceWindowHalfZmm = 5.0;
};

/**
 * @brief 点云焊缝宽度检测结果。
 */
struct PointCloudGapWidthResult {
    /// @brief true 表示成功得到 3D 焊缝宽度。
    bool valid = false;

    /// @brief 焊缝宽度，单位 mm。
    double gapWidthMm = 0.0;

    /// @brief 左侧代表边界点，单位 mm。
    WeldSpacePoint leftBoundaryPoint;

    /// @brief 右侧代表边界点，单位 mm。
    WeldSpacePoint rightBoundaryPoint;

    /// @brief PCA 得到的局部坐标系。
    PointCloudGapFrame frame;

    /// @brief 输入有效点数。
    int inputPointCount = 0;

    /// @brief SOR 后有效点数。
    int filteredPointCount = 0;

    /// @brief 左侧聚类点数。
    int leftClusterPointCount = 0;

    /// @brief 右侧聚类点数。
    int rightClusterPointCount = 0;

    /// @brief 左右局部边界线的方向一致性，范围 0~1。
    double boundaryParallelScore = 0.0;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 基于三维点云的焊缝宽度检测器。
 */
class CPointCloudGapWidthDetector final {
public:
    /**
     * @brief 从已重建点云和参考焊缝点计算 3D 宽度。
     * @param pointCloud 输入焊缝点云，坐标单位 mm。
     * @param referenceWeldPoint 参考焊缝点，坐标单位 mm。
     * @param options 点云滤波、聚类和局部截面参数。
     * @return 点云焊缝宽度检测结果。
     */
    static PointCloudGapWidthResult detect(
        const WeldPointCloud &pointCloud,
        const WeldSpacePoint &referenceWeldPoint,
        const PointCloudGapWidthOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_POINT_CLOUD_GAP_WIDTH_DETECTOR_H
