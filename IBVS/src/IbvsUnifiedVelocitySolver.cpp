/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsUnifiedVelocitySolver.cpp
 * @brief     IBVS 与 LVS 统一约束速度求解实现。
 * @details   本文件实现相机图像任务、LVS 贴缝约束和 XYZ 位置范围约束的统一求解。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

#include "ibvs/IbvsUnifiedVelocitySolver.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double MIN_NORM = 1e-9;

bool isFiniteFeature(const IbvsFeatureVector& feature)
{
    for (int i = 0; i < feature.size(); ++i)
    {
        if (!std::isfinite(feature(i)))
        {
            return false;
        }
    }

    return true;
}

bool isFiniteConfig(const IbvsUnifiedVelocityConfig& config)
{
    return std::isfinite(config.lambdaF)
        && std::isfinite(config.dampingKappa)
        && config.dampingKappa >= 0.0
        && std::isfinite(config.minDamping)
        && config.minDamping > 0.0
        && std::isfinite(config.maxVelocityMps)
        && config.maxVelocityMps >= 0.0
        && std::isfinite(config.lvsTaskScaleMin)
        && std::isfinite(config.lvsTaskScaleMax)
        && config.lvsTaskScaleMin > 0.0
        && config.lvsTaskScaleMax >= config.lvsTaskScaleMin;
}

bool isValidLvsConstraint(const IbvsLvsConstraint& constraint)
{
    return constraint.jacobian.allFinite()
        && constraint.errorM.allFinite()
        && std::isfinite(constraint.lambdaD)
        && std::isfinite(constraint.alphaLvs)
        && std::isfinite(constraint.omegaLvs)
        && std::isfinite(constraint.betaLvs)
        && constraint.axisWeights.allFinite()
        && (constraint.axisWeights.array() > 0.0).all();
}

bool isFinitePositionConstraint(const IbvsPositionConstraint& constraint)
{
    return std::isfinite(constraint.currentXMm)
        && std::isfinite(constraint.currentYMm)
        && std::isfinite(constraint.currentZMm)
        && std::isfinite(constraint.xMinMm)
        && std::isfinite(constraint.xMaxMm)
        && std::isfinite(constraint.yMinMm)
        && std::isfinite(constraint.yMaxMm)
        && std::isfinite(constraint.zMinMm)
        && std::isfinite(constraint.zMaxMm)
        && std::isfinite(constraint.lambdaPos)
        && std::isfinite(constraint.omegaPos)
        && constraint.xMinMm <= constraint.xMaxMm
        && constraint.yMinMm <= constraint.yMaxMm
        && constraint.zMinMm <= constraint.zMaxMm;
}

} // namespace

Eigen::Vector3d IbvsUnifiedVelocitySolver::computeSeamError(
    const Eigen::Vector3d& pointM,
    const IbvsSeamLineInfo& seam)
{
    if (!seam.valid
        || !pointM.allFinite()
        || !seam.p0M.allFinite()
        || !seam.direction.allFinite()
        || seam.direction.norm() <= MIN_NORM)
    {
        return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d direction = seam.direction.normalized();
    const Eigen::Matrix3d projection =
        Eigen::Matrix3d::Identity() - direction * direction.transpose();
    return projection * (pointM - seam.p0M);
}

Eigen::Matrix3d IbvsUnifiedVelocitySolver::computeLvsJacobian(
    const Eigen::Vector3d& direction)
{
    if (!direction.allFinite() || direction.norm() <= MIN_NORM)
    {
        return Eigen::Matrix3d::Zero();
    }

    const Eigen::Vector3d normalizedDirection = direction.normalized();
    return Eigen::Matrix3d::Identity()
        - normalizedDirection * normalizedDirection.transpose();
}

double IbvsUnifiedVelocitySolver::computeLvsTaskScale(
    const IbvsJacobianMatrix& cameraJacobian,
    const IbvsUnifiedVelocityConfig& config)
{
    double cameraNorm = cameraJacobian.allFinite() ? cameraJacobian.norm() : 0.0;
    if (!std::isfinite(cameraNorm) || cameraNorm <= 1e-12)
    {
        cameraNorm = config.lvsTaskScaleMin;
    }

    return std::max(
        config.lvsTaskScaleMin,
        std::min(cameraNorm, config.lvsTaskScaleMax));
}

IbvsPositionConstraint
IbvsUnifiedVelocitySolver::makePositionConstraintForWorkpiece(
    int workpieceType)
{
    double xCenterMm = 0.0;
    double yCenterMm = 0.0;
    double zCenterMm = 0.0;
    double xToleranceMm = 10.0;
    double yToleranceMm = 10.0;
    double zToleranceMm = 5.0;

    switch (workpieceType)
    {
    case 0:
        xCenterMm = 600.0;
        yCenterMm = -50.0;
        zCenterMm = -275.0;
        xToleranceMm = 100.0;
        yToleranceMm = 100.0;
        zToleranceMm = 2.0;
        break;
    case 1:
    case 6:
        xCenterMm = 894.0;
        yCenterMm = -43.0;
        zCenterMm = -219.6;
        xToleranceMm = 100.0;
        yToleranceMm = 100.0;
        zToleranceMm = 2.0;
        break;
    case 2:
    case 3:
        xCenterMm = 839.0;
        yCenterMm = -16.0;
        zCenterMm = -245.8;
        xToleranceMm = 100.0;
        yToleranceMm = 100.0;
        zToleranceMm = 2.0;
        break;
    case 4:
        xCenterMm = 905.3;
        yCenterMm = -18.84;
        zCenterMm = -253.6;
        xToleranceMm = 0.1;
        yToleranceMm = 0.1;
        zToleranceMm = 0.1;
        break;
    default:
        break;
    }

    IbvsPositionConstraint constraint;
    constraint.enabled = true;
    constraint.xMinMm = xCenterMm - xToleranceMm;
    constraint.xMaxMm = xCenterMm + xToleranceMm;
    constraint.yMinMm = yCenterMm - yToleranceMm;
    constraint.yMaxMm = yCenterMm + yToleranceMm;
    constraint.zMinMm = zCenterMm - zToleranceMm;
    constraint.zMaxMm = zCenterMm + zToleranceMm;
    constraint.lambdaPos = 2.0;
    return constraint;
}

IbvsUnifiedVelocityResult IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
    const IbvsJacobianMatrix& cameraJacobian,
    const IbvsFeatureVector& currentFeature,
    const IbvsFeatureVector& desiredFeature,
    const IbvsLvsConstraint& lvsConstraint,
    const IbvsPositionConstraint& positionConstraint,
    const IbvsUnifiedVelocityConfig& config)
{
    IbvsUnifiedVelocityResult result;
    result.featureErrorPx = currentFeature - desiredFeature;

    if (!isFiniteConfig(config)
        || !cameraJacobian.allFinite()
        || cameraJacobian.norm() <= MIN_NORM
        || !isFiniteFeature(currentFeature)
        || !isFiniteFeature(desiredFeature))
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "unified velocity input is invalid"
        };
        return result;
    }

    const double omegaLvsEffective =
        std::max(0.0, lvsConstraint.betaLvs)
        * std::max(0.0, lvsConstraint.omegaLvs);
    const bool appendLvs = lvsConstraint.enabled && omegaLvsEffective > 0.0;
    if (appendLvs && !isValidLvsConstraint(lvsConstraint))
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "lvs constraint is invalid"
        };
        return result;
    }

    double exM = 0.0;
    double eyM = 0.0;
    double ezM = 0.0;
    bool xOut = false;
    bool yOut = false;
    bool zOut = false;
    const double omegaPosEffective = std::max(0.0, positionConstraint.omegaPos);
    if (positionConstraint.enabled && omegaPosEffective > 0.0)
    {
        if (!isFinitePositionConstraint(positionConstraint))
        {
            result.error = {
                IbvsStatus::INVALID_ARGUMENT,
                "position constraint is invalid"
            };
            return result;
        }

        if (positionConstraint.currentXMm < positionConstraint.xMinMm)
        {
            exM = (positionConstraint.xMinMm
                - positionConstraint.currentXMm) / 1000.0;
            xOut = true;
        }
        else if (positionConstraint.currentXMm > positionConstraint.xMaxMm)
        {
            exM = (positionConstraint.xMaxMm
                - positionConstraint.currentXMm) / 1000.0;
            xOut = true;
        }

        if (positionConstraint.currentYMm < positionConstraint.yMinMm)
        {
            eyM = (positionConstraint.yMinMm
                - positionConstraint.currentYMm) / 1000.0;
            yOut = true;
        }
        else if (positionConstraint.currentYMm > positionConstraint.yMaxMm)
        {
            eyM = (positionConstraint.yMaxMm
                - positionConstraint.currentYMm) / 1000.0;
            yOut = true;
        }

        if (positionConstraint.currentZMm < positionConstraint.zMinMm)
        {
            ezM = (positionConstraint.zMinMm
                - positionConstraint.currentZMm) / 1000.0;
            zOut = true;
        }
        else if (positionConstraint.currentZMm > positionConstraint.zMaxMm)
        {
            ezM = (positionConstraint.zMaxMm
                - positionConstraint.currentZMm) / 1000.0;
            zOut = true;
        }
    }

    const int positionRows = (xOut ? 1 : 0) + (yOut ? 1 : 0) + (zOut ? 1 : 0);
    const int totalRows = 4 + (appendLvs ? 3 : 0) + positionRows;
    Eigen::MatrixXd jAll = Eigen::MatrixXd::Zero(totalRows, 3);
    Eigen::MatrixXd jBase = Eigen::MatrixXd::Zero(totalRows, 3);
    Eigen::VectorXd bAll = Eigen::VectorXd::Zero(totalRows);
    Eigen::VectorXd bBase = Eigen::VectorXd::Zero(totalRows);

    jBase.topRows<4>() = cameraJacobian;
    bBase.head<4>() = -config.lambdaF * result.featureErrorPx;
    jAll.topRows<4>() = jBase.topRows<4>();
    bAll.head<4>() = bBase.head<4>();

    int row = 4;
    Eigen::Matrix3d lvsAxisWeights = Eigen::Matrix3d::Identity();
    if (appendLvs)
    {
        if (lvsConstraint.useAxisWeights)
        {
            lvsAxisWeights.diagonal() = lvsConstraint.axisWeights;
        }

        result.lvsTaskScale = computeLvsTaskScale(cameraJacobian, config);
        const Eigen::Matrix3d lvsJacobian =
            lvsAxisWeights
            * (result.lvsTaskScale
                * lvsConstraint.alphaLvs
                * lvsConstraint.jacobian);
        const Eigen::Vector3d lvsTarget =
            -lvsAxisWeights
            * (result.lvsTaskScale
                * lvsConstraint.alphaLvs
                * lvsConstraint.lambdaD
                * lvsConstraint.errorM);
        const double sqrtOmegaLvs = std::sqrt(omegaLvsEffective);
        jBase.block(row, 0, 3, 3) = lvsJacobian;
        bBase.segment(row, 3) = lvsTarget;
        jAll.block(row, 0, 3, 3) = sqrtOmegaLvs * lvsJacobian;
        bAll.segment(row, 3) = sqrtOmegaLvs * lvsTarget;
        row += 3;
        result.usedLvs = true;
    }

    const double sqrtOmegaPos = std::sqrt(omegaPosEffective);
    if (xOut)
    {
        jBase.row(row) << 1.0, 0.0, 0.0;
        bBase(row) = positionConstraint.lambdaPos * exM;
        jAll.row(row) = sqrtOmegaPos * jBase.row(row);
        bAll(row) = sqrtOmegaPos * bBase(row);
        ++row;
    }
    if (yOut)
    {
        jBase.row(row) << 0.0, 1.0, 0.0;
        bBase(row) = positionConstraint.lambdaPos * eyM;
        jAll.row(row) = sqrtOmegaPos * jBase.row(row);
        bAll(row) = sqrtOmegaPos * bBase(row);
        ++row;
    }
    if (zOut)
    {
        jBase.row(row) << 0.0, 0.0, 1.0;
        bBase(row) = positionConstraint.lambdaPos * ezM;
        jAll.row(row) = sqrtOmegaPos * jBase.row(row);
        bAll(row) = sqrtOmegaPos * bBase(row);
        ++row;
    }
    result.usedPositionConstraint = positionRows > 0;
    result.rowCount = totalRows;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        jAll,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::JacobiSVD<Eigen::MatrixXd> svdBase(
        jBase,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd baseSingularValues = svdBase.singularValues();
    if (singularValues.size() == 0
        || baseSingularValues.size() == 0
        || singularValues.maxCoeff() <= MIN_NORM)
    {
        result.error = {
            IbvsStatus::SINGULAR_MATRIX,
            "unified jacobian is singular"
        };
        return result;
    }

    for (int i = 0; i < result.singularValues.size()
        && i < singularValues.size(); ++i)
    {
        result.singularValues(i) = singularValues(i);
    }

    const double sigmaMaxBase = baseSingularValues.maxCoeff();
    result.dampingMu = std::max(
        config.dampingKappa * sigmaMaxBase * sigmaMaxBase,
        config.minDamping);

    Eigen::MatrixXd pseudoInverse = Eigen::MatrixXd::Zero(3, totalRows);
    for (int i = 0; i < 3 && i < singularValues.size(); ++i)
    {
        const double singularValue = singularValues(i);
        if (singularValue > MIN_NORM)
        {
            const double damped =
                singularValue
                / (singularValue * singularValue + result.dampingMu);
            pseudoInverse += damped
                * svd.matrixV().col(i)
                * svd.matrixU().col(i).transpose();
        }
    }

    result.velocityMps = pseudoInverse * bAll;
    if (!result.velocityMps.allFinite())
    {
        result.error = {
            IbvsStatus::RUNTIME_ERROR,
            "unified velocity is not finite"
        };
        result.velocityMps.setZero();
        return result;
    }

    const double velocityNorm = result.velocityMps.norm();
    if (config.maxVelocityMps > 0.0 && velocityNorm > config.maxVelocityMps)
    {
        result.velocityMps *= config.maxVelocityMps / velocityNorm;
        result.velocityLimited = true;
    }

    result.camResidualZeroNorm = (config.lambdaF * result.featureErrorPx).norm();
    result.camResidualFinalNorm =
        (cameraJacobian * result.velocityMps
            + config.lambdaF * result.featureErrorPx).norm();
    if (result.usedLvs)
    {
        const Eigen::Vector3d weightedZero =
            lvsAxisWeights
            * (result.lvsTaskScale
                * lvsConstraint.alphaLvs
                * lvsConstraint.lambdaD
                * lvsConstraint.errorM);
        const Eigen::Vector3d weightedFinal =
            lvsAxisWeights
            * (result.lvsTaskScale
                * lvsConstraint.alphaLvs
                * (lvsConstraint.jacobian * result.velocityMps
                    + lvsConstraint.lambdaD * lvsConstraint.errorM));
        result.lvsResidualZeroNorm = weightedZero.norm();
        result.lvsResidualFinalNorm = weightedFinal.norm();
    }

    double positionResidualSq = 0.0;
    if (xOut)
    {
        const double residual = result.velocityMps(0)
            - positionConstraint.lambdaPos * exM;
        positionResidualSq += residual * residual;
    }
    if (yOut)
    {
        const double residual = result.velocityMps(1)
            - positionConstraint.lambdaPos * eyM;
        positionResidualSq += residual * residual;
    }
    if (zOut)
    {
        const double residual = result.velocityMps(2)
            - positionConstraint.lambdaPos * ezM;
        positionResidualSq += residual * residual;
    }
    result.positionResidualFinalNorm = std::sqrt(positionResidualSq);

    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

} // namespace i2r_rlc::ibvs
