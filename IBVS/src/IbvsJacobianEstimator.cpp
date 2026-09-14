/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsJacobianEstimator.cpp
 * @brief     IBVS 雅可比估计和滤波更新实现。
 * @details   本文件实现 OLS 初始雅可比估计、Kalman 更新、线性雅可比和质量诊断。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "ibvs/IbvsJacobianEstimator.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr std::size_t MIN_OLS_SAMPLE_COUNT = 6;
constexpr double MIN_DEPTH_M = 1e-9;
constexpr double MIN_EIGENVALUE = 1e-9;
constexpr double MIN_VARIANCE = 1e-9;

} // namespace

IbvsJacobianEstimator::IbvsJacobianEstimator()
{
    configure(m_config);
}

IbvsJacobianEstimator::IbvsJacobianEstimator(const IbvsKalmanConfig& config)
{
    configure(config);
}

void IbvsJacobianEstimator::configure(const IbvsKalmanConfig& config)
{
    m_config = config;
    m_processNoise = IbvsStateCovariance::Identity() * m_config.q;
    m_covariance = IbvsStateCovariance::Identity() * m_config.p0;
    if (m_config.useChannelwiseR)
    {
        configureChannelwiseR(m_config.channelSigmasPx);
    }
    else
    {
        const double variance = m_config.rPix * m_config.rPix;
        m_observationNoise = IbvsObservationCovariance::Identity() * variance;
    }
}

const IbvsKalmanConfig& IbvsJacobianEstimator::config() const
{
    return m_config;
}

void IbvsJacobianEstimator::configureChannelwiseR(
    const Eigen::Vector4d& channelSigmasPx)
{
    m_config.channelSigmasPx = channelSigmasPx;
    m_config.useChannelwiseR = true;
    m_observationNoise =
        channelSigmasPx.array().square().matrix().asDiagonal();
}

void IbvsJacobianEstimator::setNisGateThresholds(
    double rejectThreshold,
    double inflateThreshold,
    double inflateFactor)
{
    m_config.nisRejectThreshold = rejectThreshold;
    m_config.nisInflateThreshold = inflateThreshold;
    m_config.nisInflateFactor = inflateFactor;
}

void IbvsJacobianEstimator::kfInitWithJ0(
    const IbvsJacobianMatrix& initialJacobian)
{
    m_state = jacobianToState(initialJacobian);
    m_initialized = true;
}

IbvsJacobianOlsResult IbvsJacobianEstimator::computeInitialJacobianOLS(
    const std::vector<Eigen::Vector3d>& deltaSamplesM,
    const std::vector<Eigen::Vector4d>& deltaFeatureSamplesPx,
    double mu) const
{
    IbvsJacobianOlsResult result;
    result.sampleCount = deltaSamplesM.size();

    if (deltaSamplesM.size() != deltaFeatureSamplesPx.size()
        || deltaSamplesM.size() < MIN_OLS_SAMPLE_COUNT)
    {
        result.error = {
            IbvsStatus::NOT_ENOUGH_SAMPLES,
            "OLS requires matching delta motion and feature samples"
        };
        return result;
    }

    const int sampleCount = static_cast<int>(deltaSamplesM.size());
    Eigen::MatrixXd stackedH(4 * sampleCount, 12);
    Eigen::VectorXd stackedDeltaFeature(4 * sampleCount);

    for (int i = 0; i < sampleCount; ++i)
    {
        stackedH.block<4, 12>(4 * i, 0) = makeH(deltaSamplesM[i]);
        stackedDeltaFeature.segment<4>(4 * i) = deltaFeatureSamplesPx[i];
    }

    Eigen::Matrix<double, 12, 12> normalMatrix =
        stackedH.transpose() * stackedH;
    normalMatrix.diagonal().array() += std::max(mu, 0.0);
    const Eigen::Matrix<double, 12, 1> rightHandSide =
        stackedH.transpose() * stackedDeltaFeature;

    Eigen::LDLT<Eigen::Matrix<double, 12, 12>> ldlt(normalMatrix);
    if (ldlt.info() != Eigen::Success)
    {
        result.error = {
            IbvsStatus::SINGULAR_MATRIX,
            "OLS normal matrix decomposition failed"
        };
        return result;
    }

    const Eigen::Matrix<double, 12, 1> state = ldlt.solve(rightHandSide);
    if (ldlt.info() != Eigen::Success)
    {
        result.error = {
            IbvsStatus::SINGULAR_MATRIX,
            "OLS normal matrix solve failed"
        };
        return result;
    }

    result.jacobian = stateToJacobian(state);
    result.residualNorm = (stackedH * state - stackedDeltaFeature).norm();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> eig(
        normalMatrix);
    if (eig.info() == Eigen::Success)
    {
        const double minEigenValue = eig.eigenvalues().minCoeff();
        const double maxEigenValue = eig.eigenvalues().maxCoeff();
        result.conditionEstimate = minEigenValue > MIN_EIGENVALUE
            ? maxEigenValue / minEigenValue
            : std::numeric_limits<double>::infinity();
    }

    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

IbvsJacobianOlsResult IbvsJacobianEstimator::kfInitWithSamples(
    const std::vector<Eigen::Vector3d>& deltaSamplesM,
    const std::vector<Eigen::Vector4d>& deltaFeatureSamplesPx,
    double mu)
{
    IbvsJacobianOlsResult result =
        computeInitialJacobianOLS(deltaSamplesM, deltaFeatureSamplesPx, mu);
    if (result.error.isOk())
    {
        kfInitWithJ0(result.jacobian);
    }
    return result;
}

IbvsJacobianUpdateResult IbvsJacobianEstimator::jacobianEK(
    const Eigen::Vector3d& deltaM,
    const Eigen::Vector4d& deltaFeaturePx)
{
    ensureInitialized();

    IbvsJacobianUpdateResult result;
    IbvsStateVector predictedState = m_state;
    IbvsStateCovariance predictedCovariance = m_covariance + m_processNoise;
    const double motionNorm = deltaM.norm();

    if (motionNorm < m_config.epsUpdateM)
    {
        m_state = predictedState;
        const double traceCap = m_config.p0 * 12.0 * 200.0;
        if (predictedCovariance.trace() <= traceCap)
        {
            m_covariance = predictedCovariance;
        }

        result.error = { IbvsStatus::OK, std::string() };
        result.jacobian = getCurrentJacobian();
        result.skippedBySmallMotion = true;
        return result;
    }

    const Eigen::Matrix<double, 4, 12> h = makeH(deltaM);
    result.innovation = deltaFeaturePx - h * predictedState;

    Eigen::Matrix4d innovationCovariance =
        h * predictedCovariance * h.transpose() + m_observationNoise;
    const double nis = computeNis(result.innovation, innovationCovariance);
    m_lastNis = nis;
    m_lastRejectedByNis = false;
    m_lastInflatedByNis = false;
    result.nis = nis;

    bool shouldUpdate = true;
    IbvsObservationCovariance effectiveObservationNoise = m_observationNoise;
    if (nis > m_config.nisRejectThreshold)
    {
        shouldUpdate = false;
        m_lastRejectedByNis = true;
        result.rejectedByNis = true;
    }
    else if (nis > m_config.nisInflateThreshold)
    {
        effectiveObservationNoise *= m_config.nisInflateFactor;
        innovationCovariance =
            h * predictedCovariance * h.transpose()
            + effectiveObservationNoise;
        m_lastInflatedByNis = true;
        result.inflatedRByNis = true;
    }

    if (shouldUpdate)
    {
        Eigen::LDLT<Eigen::Matrix4d> ldlt(innovationCovariance);
        if (ldlt.info() != Eigen::Success)
        {
            m_state = predictedState;
            m_covariance = predictedCovariance;
            result.error = {
                IbvsStatus::SINGULAR_MATRIX,
                "innovation covariance decomposition failed"
            };
            result.jacobian = getCurrentJacobian();
            return result;
        }

        const Eigen::Matrix4d innovationCovarianceInv =
            ldlt.solve(Eigen::Matrix4d::Identity());
        result.kalmanGain =
            predictedCovariance * h.transpose() * innovationCovarianceInv;
        m_state = predictedState + result.kalmanGain * result.innovation;

        const IbvsStateCovariance identity =
            IbvsStateCovariance::Identity();
        const IbvsStateCovariance iMinusKh =
            identity - result.kalmanGain * h;
        m_covariance =
            iMinusKh * predictedCovariance * iMinusKh.transpose()
            + result.kalmanGain
            * effectiveObservationNoise
            * result.kalmanGain.transpose();

        result.updated = true;
        pushRecentSample(
            deltaM,
            deltaFeaturePx,
            result.innovation,
            nis,
            result.innovation.norm());
    }
    else
    {
        m_state = predictedState;
        m_covariance = predictedCovariance;
    }

    result.error = { IbvsStatus::OK, std::string() };
    result.jacobian = getCurrentJacobian();
    return result;
}

IbvsJacobianMatrix IbvsJacobianEstimator::jacobianLinear(
    double depthM,
    const PixelFeaturePair& featurePoints,
    const IbvsCameraIntrinsics& intrinsics,
    const IbvsHandEyeCalibration& handEye) const
{
    if (depthM <= MIN_DEPTH_M
        || !featurePoints.isValid(1e-9)
        || !intrinsics.isValid())
    {
        return IbvsJacobianMatrix::Zero();
    }

    const double u1 = featurePoints.first.u - intrinsics.cx;
    const double v1 = featurePoints.first.v - intrinsics.cy;
    const double u2 = featurePoints.second.u - intrinsics.cx;
    const double v2 = featurePoints.second.v - intrinsics.cy;

    IbvsJacobianMatrix imageJacobian;
    imageJacobian << intrinsics.fx, 0.0, -u1,
        0.0, intrinsics.fy, -v1,
        intrinsics.fx, 0.0, -u2,
        0.0, intrinsics.fy, -v2;
    imageJacobian *= 1.0 / depthM;

    return imageJacobian * handEye.rotation;
}

IbvsJacobianMatrix IbvsJacobianEstimator::getCurrentJacobian() const
{
    return stateToJacobian(m_state);
}

IbvsStateVector IbvsJacobianEstimator::getState() const
{
    return m_state;
}

IbvsStateCovariance IbvsJacobianEstimator::getCovariance() const
{
    return m_covariance;
}

IbvsObservationCovariance
IbvsJacobianEstimator::getObservationCovariance() const
{
    return m_observationNoise;
}

double IbvsJacobianEstimator::getLastNis() const
{
    return m_lastNis;
}

IbvsJacobianQuality IbvsJacobianEstimator::evaluateJacobianQuality() const
{
    IbvsJacobianQuality quality;
    const IbvsJacobianMatrix jacobian = getCurrentJacobian();

    Eigen::JacobiSVD<IbvsJacobianMatrix> svd(jacobian);
    const Eigen::Vector3d singularValues = svd.singularValues();
    quality.minSingularValue = singularValues(2);
    quality.conditionNumber =
        singularValues(0) / (singularValues(2) + 1e-12);
    quality.frobeniusNorm = jacobian.norm();
    quality.predictionRmsePx = recentRmse();
    quality.nisMean = recentNisMean();

    for (int i = 0; i < 3; ++i)
    {
        if (singularValues(i) > singularValues(0) * 1e-6)
        {
            ++quality.effectiveRank;
        }
    }

    return quality;
}

IbvsJacobianOlsResult IbvsJacobianEstimator::shortWindowReset(
    std::size_t windowSize,
    double mu)
{
    if (m_recentDeltaM.size() < windowSize || windowSize < MIN_OLS_SAMPLE_COUNT)
    {
        IbvsJacobianOlsResult result;
        result.error = {
            IbvsStatus::NOT_ENOUGH_SAMPLES,
            "not enough recent samples for short-window reset"
        };
        result.sampleCount = m_recentDeltaM.size();
        return result;
    }

    const auto startM = m_recentDeltaM.end()
        - static_cast<std::ptrdiff_t>(windowSize);
    const auto startF = m_recentDeltaFeaturePx.end()
        - static_cast<std::ptrdiff_t>(windowSize);
    const std::vector<Eigen::Vector3d> deltaM(startM, m_recentDeltaM.end());
    const std::vector<Eigen::Vector4d> deltaFeature(
        startF,
        m_recentDeltaFeaturePx.end());

    IbvsJacobianOlsResult result =
        computeInitialJacobianOLS(deltaM, deltaFeature, mu);
    if (result.error.isOk() && result.jacobian.norm() > 1e-6)
    {
        kfInitWithJ0(result.jacobian);
        m_covariance = IbvsStateCovariance::Identity() * m_config.p0;
    }
    return result;
}

Eigen::Vector4d IbvsJacobianEstimator::estimateChannelSigmasOffline(
    const std::vector<Eigen::Vector3d>& deltaSamplesM,
    const std::vector<Eigen::Vector4d>& deltaFeatureSamplesPx,
    const IbvsJacobianMatrix& initialJacobian) const
{
    Eigen::Vector4d sigmas = Eigen::Vector4d::Zero();
    const std::size_t sampleCount = deltaSamplesM.size();
    if (sampleCount != deltaFeatureSamplesPx.size() || sampleCount < 2)
    {
        return sigmas;
    }

    for (int channel = 0; channel < 4; ++channel)
    {
        std::vector<double> residuals;
        residuals.reserve(sampleCount);
        for (std::size_t i = 0; i < sampleCount; ++i)
        {
            const Eigen::Vector4d predicted = initialJacobian * deltaSamplesM[i];
            residuals.push_back(deltaFeatureSamplesPx[i](channel) - predicted(channel));
        }

        const double mean =
            std::accumulate(residuals.begin(), residuals.end(), 0.0)
            / residuals.size();
        double variance = 0.0;
        for (double residual : residuals)
        {
            const double diff = residual - mean;
            variance += diff * diff;
        }
        variance /= static_cast<double>(residuals.size() - 1);
        sigmas(channel) = std::sqrt(std::max(variance, 0.0));
    }

    return sigmas;
}

Eigen::Matrix<double, 4, 12> IbvsJacobianEstimator::makeH(
    const Eigen::Vector3d& deltaM)
{
    Eigen::Matrix<double, 4, 12> h =
        Eigen::Matrix<double, 4, 12>::Zero();
    for (int row = 0; row < 4; ++row)
    {
        h.block<1, 3>(row, row * 3) = deltaM.transpose();
    }
    return h;
}

IbvsJacobianMatrix IbvsJacobianEstimator::stateToJacobian(
    const IbvsStateVector& state)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 3, Eigen::RowMajor>>(
        state.data());
}

IbvsStateVector IbvsJacobianEstimator::jacobianToState(
    const IbvsJacobianMatrix& jacobian)
{
    const Eigen::Matrix<double, 4, 3, Eigen::RowMajor> rowMajorJacobian =
        jacobian;
    return Eigen::Map<const IbvsStateVector>(rowMajorJacobian.data());
}

void IbvsJacobianEstimator::ensureInitialized()
{
    if (m_initialized)
    {
        return;
    }

    m_state.setZero();
    m_covariance = IbvsStateCovariance::Identity() * m_config.p0;
    m_initialized = true;
}

double IbvsJacobianEstimator::computeNis(
    const Eigen::Vector4d& innovation,
    const Eigen::Matrix4d& innovationCovariance) const
{
    Eigen::LDLT<Eigen::Matrix4d> ldlt(innovationCovariance);
    if (ldlt.info() != Eigen::Success)
    {
        return -1.0;
    }

    const Eigen::Vector4d solved = ldlt.solve(innovation);
    return innovation.dot(solved);
}

double IbvsJacobianEstimator::recentRmse() const
{
    if (m_recentResiduals.empty())
    {
        return 0.0;
    }

    double sumSquare = 0.0;
    for (double residual : m_recentResiduals)
    {
        sumSquare += residual * residual;
    }
    return std::sqrt(sumSquare / static_cast<double>(m_recentResiduals.size()));
}

double IbvsJacobianEstimator::recentNisMean() const
{
    if (m_recentNis.empty())
    {
        return 4.0;
    }

    return std::accumulate(m_recentNis.begin(), m_recentNis.end(), 0.0)
        / static_cast<double>(m_recentNis.size());
}

void IbvsJacobianEstimator::pushRecentSample(
    const Eigen::Vector3d& deltaM,
    const Eigen::Vector4d& deltaFeaturePx,
    const Eigen::Vector4d& innovation,
    double nis,
    double residualNorm)
{
    m_recentDeltaM.push_back(deltaM);
    m_recentDeltaFeaturePx.push_back(deltaFeaturePx);
    m_recentInnovations.push_back(innovation);
    m_recentResiduals.push_back(residualNorm);
    if (nis > 0.0)
    {
        m_recentNis.push_back(nis);
    }
    trimHistory();
}

void IbvsJacobianEstimator::trimHistory()
{
    const std::size_t maxHistorySize =
        std::max<std::size_t>(m_config.maxHistorySize, 1);
    while (m_recentDeltaM.size() > maxHistorySize)
    {
        m_recentDeltaM.erase(m_recentDeltaM.begin());
    }
    while (m_recentDeltaFeaturePx.size() > maxHistorySize)
    {
        m_recentDeltaFeaturePx.erase(m_recentDeltaFeaturePx.begin());
    }
    while (m_recentInnovations.size() > maxHistorySize)
    {
        m_recentInnovations.erase(m_recentInnovations.begin());
    }
    while (m_recentResiduals.size() > maxHistorySize)
    {
        m_recentResiduals.erase(m_recentResiduals.begin());
    }
    while (m_recentNis.size() > maxHistorySize)
    {
        m_recentNis.erase(m_recentNis.begin());
    }
}

} // namespace i2r_rlc::ibvs
