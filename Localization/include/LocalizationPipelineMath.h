/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationPipelineMath.h
 * @brief     定位流程内部几何计算工具。
 * @details   本文件声明 pipeline 共享的点云残差和刚体变换辅助函数。该工具命名空间
 *            仅供 Localization 模块内部 pipeline 组合使用。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_PIPELINE_MATH_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_PIPELINE_MATH_H

#include <vector>

#include "LocalizationTypes.h"

namespace i2r_rlc::localization::pipeline_math {

/**
 * @brief 计算 source 到 target 的最近邻平均距离，单位 mm。
 */
double meanNearestDistance(
    const std::vector<Point3D_mm>& source,
    const std::vector<Point3D_mm>& target);

/**
 * @brief 计算 source 到 target 的最近邻均方根误差，单位 mm。
 *
 * 每个 source 点取 target 中的最近邻，以这些对应距离的平方均值开根号。
 */
double nearestNeighborRmse(
    const std::vector<Point3D_mm>& source,
    const std::vector<Point3D_mm>& target);

/**
 * @brief 求刚体变换逆矩阵。
 */
RigidTransform inverseRigidTransform(const RigidTransform& transform);

/**
 * @brief 按刚体变换转换点云。
 */
std::vector<Point3D_mm> transformPoints(
    const std::vector<Point3D_mm>& points,
    const RigidTransform& transform);

/**
 * @brief 组合两个刚体变换，返回 lhs * rhs。
 */
RigidTransform composeRigidTransforms(
    const RigidTransform& lhs,
    const RigidTransform& rhs);

} // namespace i2r_rlc::localization::pipeline_math

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_PIPELINE_MATH_H
