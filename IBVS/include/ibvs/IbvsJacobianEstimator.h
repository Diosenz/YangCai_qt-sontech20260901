/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsJacobianEstimator.h
 * @brief     IBVS 雅可比估计和滤波更新。
 * @details   本文件定义 4x3 平移雅可比的初始化、递推更新和质量诊断。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_JACOBIAN_ESTIMATOR_H
#define I2R_RLC_IBVS_IBVS_JACOBIAN_ESTIMATOR_H

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 估计和维护 IBVS 的 4x3 平移雅可比。
 */
class IbvsJacobianEstimator
{
public:
    IbvsJacobianEstimator();
    explicit IbvsJacobianEstimator(const IbvsKalmanConfig& config);

    /**
     * @brief 更新 Kalman 雅可比估计参数并重建噪声矩阵。
     */
    void configure(const IbvsKalmanConfig& config);

    /**
     * @brief 返回当前 Kalman 雅可比估计参数。
     */
    const IbvsKalmanConfig& config() const;

    /**
     * @brief 使用四个特征通道的标准差配置观测噪声矩阵。
     */
    void configureChannelwiseR(const Eigen::Vector4d& channelSigmasPx);

    /**
     * @brief 设置 NIS 拒绝、观测噪声放大和放大倍数阈值。
     */
    void setNisGateThresholds(
        double rejectThreshold,
        double inflateThreshold,
        double inflateFactor);

    /**
     * @brief 使用已有 4x3 雅可比初始化 Kalman 状态。
     */
    void kfInitWithJ0(const IbvsJacobianMatrix& initialJacobian);

    /**
     * @brief 使用运动增量和图像特征增量，通过 OLS 估计初始雅可比。
     */
    IbvsJacobianOlsResult computeInitialJacobianOLS(
        const std::vector<Eigen::Vector3d>& deltaSamplesM,
        const std::vector<Eigen::Vector4d>& deltaFeatureSamplesPx,
        double mu) const;

    /**
     * @brief 先通过 OLS 估计初始雅可比，再写入 Kalman 状态。
     */
    IbvsJacobianOlsResult kfInitWithSamples(
        const std::vector<Eigen::Vector3d>& deltaSamplesM,
        const std::vector<Eigen::Vector4d>& deltaFeatureSamplesPx,
        double mu);

    /**
     * @brief 使用增量位移和特征增量更新 Kalman 雅可比。
     */
    IbvsJacobianUpdateResult jacobianEK(
        const Eigen::Vector3d& deltaM,
        const Eigen::Vector4d& deltaFeaturePx);

    /**
     * @brief 根据当前像素点、深度和手眼旋转直接计算线性雅可比。
     */
    IbvsJacobianMatrix jacobianLinear(
        double depthM,
        const PixelFeaturePair& featurePoints,
        const IbvsCameraIntrinsics& intrinsics,
        const IbvsHandEyeCalibration& handEye) const;

    /**
     * @brief 返回当前 4x3 雅可比矩阵。
     */
    IbvsJacobianMatrix getCurrentJacobian() const;

    /**
     * @brief 返回当前 12 维 Kalman 状态。
     */
    IbvsStateVector getState() const;

    /**
     * @brief 返回当前 Kalman 状态协方差。
     */
    IbvsStateCovariance getCovariance() const;

    /**
     * @brief 返回当前 4 维观测噪声协方差。
     */
    IbvsObservationCovariance getObservationCovariance() const;

    /**
     * @brief 返回最近一次 Kalman 更新的 NIS 值。
     */
    double getLastNis() const;

    /**
     * @brief 根据条件数、秩、奇异值和近期残差评估雅可比质量。
     */
    IbvsJacobianQuality evaluateJacobianQuality() const;

    /**
     * @brief 使用最近窗口样本重新估计雅可比并重置 Kalman 状态。
     */
    IbvsJacobianOlsResult shortWindowReset(std::size_t windowSize, double mu);

    /**
     * @brief 根据离线预测残差估计四个特征通道的标准差。
     */
    Eigen::Vector4d estimateChannelSigmasOffline(
        const std::vector<Eigen::Vector3d>& deltaSamplesM,
        const std::vector<Eigen::Vector4d>& deltaFeatureSamplesPx,
        const IbvsJacobianMatrix& initialJacobian) const;

private:
    static Eigen::Matrix<double, 4, 12> makeH(const Eigen::Vector3d& deltaM);
    static IbvsJacobianMatrix stateToJacobian(const IbvsStateVector& state);
    static IbvsStateVector jacobianToState(const IbvsJacobianMatrix& jacobian);

    void ensureInitialized();
    double computeNis(
        const Eigen::Vector4d& innovation,
        const Eigen::Matrix4d& innovationCovariance) const;
    double recentRmse() const;
    double recentNisMean() const;
    void pushRecentSample(
        const Eigen::Vector3d& deltaM,
        const Eigen::Vector4d& deltaFeaturePx,
        const Eigen::Vector4d& innovation,
        double nis,
        double residualNorm);
    void trimHistory();

    IbvsKalmanConfig m_config;
    IbvsStateVector m_state = IbvsStateVector::Zero();
    IbvsStateCovariance m_covariance = IbvsStateCovariance::Identity() * 100.0;
    IbvsStateCovariance m_processNoise = IbvsStateCovariance::Identity() * 1e-3;
    IbvsObservationCovariance m_observationNoise =
        IbvsObservationCovariance::Identity() * 625.0;
    bool m_initialized = false;
    double m_lastNis = -1.0;
    bool m_lastRejectedByNis = false;
    bool m_lastInflatedByNis = false;
    std::vector<Eigen::Vector3d> m_recentDeltaM;
    std::vector<Eigen::Vector4d> m_recentDeltaFeaturePx;
    std::vector<Eigen::Vector4d> m_recentInnovations;
    std::vector<double> m_recentResiduals;
    std::vector<double> m_recentNis;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_JACOBIAN_ESTIMATOR_H
