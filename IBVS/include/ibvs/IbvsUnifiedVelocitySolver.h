/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsUnifiedVelocitySolver.h
 * @brief     IBVS 与 LVS 统一约束速度求解器。
 * @details   本文件定义相机图像任务、LVS 贴缝约束和 XYZ 位置范围约束的纯算法接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_IBVS_IBVS_UNIFIED_VELOCITY_SOLVER_H
#define I2R_RLC_IBVS_IBVS_UNIFIED_VELOCITY_SOLVER_H

#include <Eigen/Dense>

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 直线焊缝在三维空间中的轻量表达。
 */
struct IbvsSeamLineInfo
{
    Eigen::Vector3d p0M = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
    bool valid = false;
};

/**
 * @brief LVS 贴缝约束。
 */
struct IbvsLvsConstraint
{
    bool enabled = false;
    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d errorM = Eigen::Vector3d::Zero();
    double lambdaD = 1.0;
    double alphaLvs = 1.0;
    double omegaLvs = 1.0;
    double betaLvs = 1.0;
    bool useAxisWeights = false;
    Eigen::Vector3d axisWeights = Eigen::Vector3d::Ones();
};

/**
 * @brief XYZ 位置范围约束，位置单位为 mm。
 */
struct IbvsPositionConstraint
{
    bool enabled = false;
    double currentXMm = 0.0;
    double currentYMm = 0.0;
    double currentZMm = 0.0;
    double xMinMm = 0.0;
    double xMaxMm = 0.0;
    double yMinMm = 0.0;
    double yMaxMm = 0.0;
    double zMinMm = 0.0;
    double zMaxMm = 0.0;
    double lambdaPos = 2.0;
    double omegaPos = 1.0;
};

/**
 * @brief 统一速度求解参数。
 */
struct IbvsUnifiedVelocityConfig
{
    double lambdaF = 1.0;
    double dampingKappa = 0.2;
    double minDamping = 1e-4;
    double maxVelocityMps = 0.05;
    double lvsTaskScaleMin = 200.0;
    double lvsTaskScaleMax = 5000.0;
};

/**
 * @brief 统一速度求解结果与诊断信息。
 */
struct IbvsUnifiedVelocityResult
{
    IbvsError error;
    IbvsVelocityVector velocityMps = IbvsVelocityVector::Zero();
    Eigen::Vector4d featureErrorPx = Eigen::Vector4d::Zero();
    Eigen::Vector3d singularValues = Eigen::Vector3d::Zero();
    bool usedLvs = false;
    bool usedPositionConstraint = false;
    int rowCount = 0;
    double dampingMu = 0.0;
    bool velocityLimited = false;
    double lvsTaskScale = 0.0;
    double camResidualZeroNorm = 0.0;
    double camResidualFinalNorm = 0.0;
    double lvsResidualZeroNorm = 0.0;
    double lvsResidualFinalNorm = 0.0;
    double positionResidualFinalNorm = 0.0;
};

/**
 * @brief IBVS/LVS/位置约束统一阻尼最小二乘求解器。
 */
class IbvsUnifiedVelocitySolver
{
public:
    /**
     * @brief 计算 LVS 贴缝误差 D = (I - l*l^T) * (P - P0)。
     */
    static Eigen::Vector3d computeSeamError(
        const Eigen::Vector3d& pointM,
        const IbvsSeamLineInfo& seam);

    /**
     * @brief 计算 LVS 投影雅可比 J = I - l*l^T。
     */
    static Eigen::Matrix3d computeLvsJacobian(
        const Eigen::Vector3d& direction);

    /**
     * @brief 按兼容策略计算 LVS 任务尺度。
     */
    static double computeLvsTaskScale(
        const IbvsJacobianMatrix& cameraJacobian,
        const IbvsUnifiedVelocityConfig& config);

    /**
     * @brief 根据兼容工件编号返回 XYZ 位置范围约束 preset。
     */
    static IbvsPositionConstraint makePositionConstraintForWorkpiece(
        int workpieceType);

    /**
     * @brief 求解图像任务、LVS 贴缝约束和可选位置约束的统一速度。
     */
    static IbvsUnifiedVelocityResult computeUnifiedVelocity(
        const IbvsJacobianMatrix& cameraJacobian,
        const IbvsFeatureVector& currentFeature,
        const IbvsFeatureVector& desiredFeature,
        const IbvsLvsConstraint& lvsConstraint,
        const IbvsPositionConstraint& positionConstraint,
        const IbvsUnifiedVelocityConfig& config);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_UNIFIED_VELOCITY_SOLVER_H
