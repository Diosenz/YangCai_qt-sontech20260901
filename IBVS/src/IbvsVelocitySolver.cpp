/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsVelocitySolver.cpp
 * @brief     IBVS 平移速度求解实现。
 * @details   本文件实现基于 WLS 和 LM 阻尼伪逆的相机平移速度求解。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/SVD>

#include "ibvs/IbvsVelocitySolver.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double MIN_JACOBIAN_NORM = 1e-9;
constexpr double MIN_SINGULAR_VALUE = 1e-9;

} // namespace

IbvsVelocitySolver::IbvsVelocitySolver() = default;

IbvsVelocitySolver::IbvsVelocitySolver(const IbvsControlParameters& parameters)
    : m_parameters(parameters)
{
}

void IbvsVelocitySolver::setControlParameters(
    const IbvsControlParameters& parameters)
{
    m_parameters = parameters;
}

const IbvsControlParameters& IbvsVelocitySolver::controlParameters() const
{
    return m_parameters;
}

void IbvsVelocitySolver::setDesiredFeature(
    const IbvsFeatureVector& desiredFeature)
{
    m_parameters.desiredFeature = desiredFeature;
}

const IbvsFeatureVector& IbvsVelocitySolver::desiredFeature() const
{
    return m_parameters.desiredFeature;
}

IbvsVelocitySolveResult IbvsVelocitySolver::computeCameraVelocity(
    const IbvsJacobianMatrix& jacobian,
    const IbvsFeatureVector& currentFeature) const
{
    IbvsVelocitySolveResult result;
    result.featureErrorPx = featureError(currentFeature);

    if (jacobian.norm() < MIN_JACOBIAN_NORM)
    {
        result.error = {
            IbvsStatus::SINGULAR_MATRIX,
            "jacobian is near zero"
        };
        return result;
    }

    IbvsJacobianMatrix effectiveJacobian = jacobian;
    Eigen::Vector4d effectiveError = result.featureErrorPx;
    if (m_parameters.useChannelwiseR)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (m_parameters.channelSigmasPx(i) <= 0.0)
            {
                result.error = {
                    IbvsStatus::INVALID_ARGUMENT,
                    "channel sigma must be positive when WLS is enabled"
                };
                return result;
            }
        }

        const Eigen::Vector4d weights =
            m_parameters.channelSigmasPx.cwiseInverse();
        const Eigen::Matrix4d whitening = weights.asDiagonal();
        effectiveJacobian = whitening * jacobian;
        effectiveError = whitening * effectiveError;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        Eigen::MatrixXd(effectiveJacobian),
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singularValues = svd.singularValues();

    if (singularValues.size() < 3 || singularValues.maxCoeff() < MIN_SINGULAR_VALUE)
    {
        result.error = {
            IbvsStatus::SINGULAR_MATRIX,
            "jacobian singular values are too small"
        };
        return result;
    }

    result.singularValues << singularValues(0), singularValues(1), singularValues(2);
    result.conditionNumber = singularValues(0)
        / std::max(singularValues(2), MIN_SINGULAR_VALUE);

    const double sigmaMax = singularValues(0);
    double dampingMu = m_parameters.dampingKappa * sigmaMax * sigmaMax;
    dampingMu = std::max(dampingMu, m_parameters.minDamping);
    result.dampingMu = dampingMu;

    Eigen::Matrix3d dampedSigmaInv = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
    {
        const double s = singularValues(i);
        dampedSigmaInv(i, i) = s / (s * s + dampingMu);
    }

    const Eigen::MatrixXd pseudoInverse =
        svd.matrixV() * dampedSigmaInv * svd.matrixU().transpose();
    result.velocityMps = -m_parameters.lambda
        * (pseudoInverse * effectiveError);

    const double velocityNorm = result.velocityMps.norm();
    if (m_parameters.maxCameraVelocityMps > 0.0
        && velocityNorm > m_parameters.maxCameraVelocityMps)
    {
        result.velocityMps *= m_parameters.maxCameraVelocityMps / velocityNorm;
        result.velocityLimited = true;
    }

    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

Eigen::Vector4d IbvsVelocitySolver::featureError(
    const IbvsFeatureVector& currentFeature) const
{
    return currentFeature - m_parameters.desiredFeature;
}

double IbvsVelocitySolver::errorNorm(
    const IbvsFeatureVector& currentFeature) const
{
    return featureError(currentFeature).norm();
}

bool IbvsVelocitySolver::isConverged(
    const IbvsFeatureVector& currentFeature) const
{
    return errorNorm(currentFeature) < m_parameters.stopThresholdPx;
}

} // namespace i2r_rlc::ibvs
