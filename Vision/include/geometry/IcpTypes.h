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
 * @file        IcpTypes.h
 * @brief       ICP 配准公共数据类型
 * @details     定义 Vision 模块对外使用的 ICP 选项和结果。公共接口不暴露
 *              libicp 的 RMatrix、裸数组、kd-tree 或 Boost 类型。
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
#ifndef I2R_RLC_VISION_GEOMETRY_ICP_TYPES_H
#define I2R_RLC_VISION_GEOMETRY_ICP_TYPES_H

#include <string>

#include <Eigen/Core>

#include "geometry/WeldGeometryTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief ICP 配准算法类型。
 */
enum class IcpRegistrationAlgorithm {
    PointToPoint,
    PointToPlane,
    Affine,
    AffineBidirectionalDistance,
    BidirectionalDistance,
    WithScale
};

/**
 * @brief libicp 配准选项。
 */
struct IcpRegistrationOptions {
    /// @brief 默认算法类型；显式 align* 入口会覆盖该字段。
    IcpRegistrationAlgorithm algorithm = IcpRegistrationAlgorithm::PointToPoint;

    /// @brief 点维度，支持 2 或 3。
    int dimension = 3;

    /// @brief 最大迭代次数。
    int maxIterations = 100;

    /// @brief 收敛阈值，单位与输入点坐标一致。
    double minDelta = 1.0e-5;

    /// @brief 内点距离阈值，单位 mm；小于等于 0 表示使用全部点。
    double inlierDistanceMm = -1.0;

    /// @brief point-to-plane 法线估计邻域数量。
    int normalNeighborCount = 10;

    /// @brief point-to-plane 平坦度阈值，沿用 libicp 语义。
    double normalFlatness = 5.0;

    /// @brief 保持旋转约束，对应旧 keepRotation。
    bool keepRotation = false;

    /// @brief 保持连接约束，对应旧 keepConnect。
    bool keepConnect = false;

    /// @brief 删除失败点约束，对应旧 keepRemoveFailPoint。
    bool keepRemoveFailPoint = false;
};

/**
 * @brief ICP 配准结果。
 */
struct IcpRegistrationResult {
    /// @brief true 表示配准得到有效变换。
    bool valid = false;

    /// @brief 刚体变换视图；affine 路径中保存线性部分和位移。
    WeldRigidTransform transform;

    /// @brief 齐次矩阵，单位 mm，用于保存 affine/scale 路径的完整结果。
    Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();

    /// @brief scale 路径输出的尺度，非 scale 路径为 1。
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();

    /// @brief 最大残差，单位 mm。
    double maxErrorMm = 0.0;

    /// @brief 最小残差，单位 mm。
    double minErrorMm = 0.0;

    /// @brief 平均残差，单位 mm。
    double meanErrorMm = 0.0;

    /// @brief 均方根残差，单位 mm。
    double rmsErrorMm = 0.0;

    /// @brief 内点比例，范围 0~1。
    double inlierRatio = 0.0;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明。
    std::string message;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_GEOMETRY_ICP_TYPES_H
