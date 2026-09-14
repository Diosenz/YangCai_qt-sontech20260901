/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsTypes.h
 * @brief     IBVS 模块公共值对象与诊断结构。
 * @details   本文件定义算法接口共享的数据类型、参数和结果对象。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_TYPES_H
#define I2R_RLC_IBVS_IBVS_TYPES_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace i2r_rlc::ibvs {

using IbvsFeatureVector = Eigen::Matrix<double, 4, 1>;
using IbvsVelocityVector = Eigen::Matrix<double, 3, 1>;
using IbvsPoseVector = Eigen::Matrix<double, 6, 1>;
using IbvsJacobianMatrix = Eigen::Matrix<double, 4, 3>;
using IbvsStateVector = Eigen::Matrix<double, 12, 1>;
using IbvsStateCovariance = Eigen::Matrix<double, 12, 12>;
using IbvsObservationCovariance = Eigen::Matrix<double, 4, 4>;
using IbvsKalmanGain = Eigen::Matrix<double, 12, 4>;

/**
 * @brief 算法函数返回状态。
 */
enum class IbvsStatus
{
    OK,
    EMPTY_INPUT,
    INVALID_ARGUMENT,
    NOT_ENOUGH_SAMPLES,
    DEGENERATE_GEOMETRY,
    SINGULAR_MATRIX,
    FILE_OPEN_FAILED,
    PARSE_ERROR,
    DEPENDENCY_DISABLED,
    RUNTIME_ERROR
};

/**
 * @brief 统一错误信息。
 */
struct IbvsError
{
    IbvsStatus status = IbvsStatus::OK;
    std::string message;

    bool isOk() const;
};

/**
 * @brief 像素坐标点，单位为 pixel。
 */
struct PixelPoint
{
    double u = 0.0;
    double v = 0.0;

    bool isFinite() const;
};

/**
 * @brief 像素 ROI，单位为 pixel。
 */
struct PixelRoi
{
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;

    bool isValid() const;
};

/**
 * @brief 焊枪红色双特征点，坐标单位为 pixel。
 */
struct PixelFeaturePair
{
    PixelPoint first;
    PixelPoint second;

    bool isValid(double minDistancePx) const;
    double distancePx() const;
    IbvsFeatureVector toVector() const;
};

/**
 * @brief 相机内参，焦距和主点单位均为 pixel。
 */
struct IbvsCameraIntrinsics
{
    double fx = 1.0;
    double fy = 1.0;
    double cx = 0.0;
    double cy = 0.0;

    bool isValid() const;
    Eigen::Matrix3d toMatrix() const;
};

/**
 * @brief 手眼标定中的机器人到相机旋转和平移。
 */
struct IbvsHandEyeCalibration
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

/**
 * @brief IBVS 速度控制参数。
 */
struct IbvsControlParameters
{
    double lambda = 1.0;
    double stopThresholdPx = 1e-3;
    double dampingKappa = 0.2;
    double minDamping = 1e-4;
    double maxCameraVelocityMps = 0.5;
    bool useChannelwiseR = false;
    Eigen::Vector4d channelSigmasPx = Eigen::Vector4d(0.55, 0.38, 0.55, 0.38);
    IbvsFeatureVector desiredFeature = IbvsFeatureVector::Zero();
};

/**
 * @brief KF 雅可比估计参数。
 */
struct IbvsKalmanConfig
{
    double q = 1e-3;
    double rPix = 25.0;
    double p0 = 100.0;
    double epsUpdateM = 1e-6;
    double nisRejectThreshold = 13.28;
    double nisInflateThreshold = 9.49;
    double nisInflateFactor = 3.0;
    std::size_t maxHistorySize = 80;
    bool useChannelwiseR = false;
    Eigen::Vector4d channelSigmasPx = Eigen::Vector4d(0.55, 0.38, 0.55, 0.38);
};

/**
 * @brief OLS 初始雅可比估计结果。
 */
struct IbvsJacobianOlsResult
{
    IbvsError error;
    IbvsJacobianMatrix jacobian = IbvsJacobianMatrix::Zero();
    double residualNorm = 0.0;
    double conditionEstimate = -1.0;
    std::size_t sampleCount = 0;
};

/**
 * @brief KF 单帧更新结果。
 */
struct IbvsJacobianUpdateResult
{
    IbvsError error;
    IbvsJacobianMatrix jacobian = IbvsJacobianMatrix::Zero();
    Eigen::Vector4d innovation = Eigen::Vector4d::Zero();
    IbvsKalmanGain kalmanGain = IbvsKalmanGain::Zero();
    double nis = -1.0;
    bool updated = false;
    bool skippedBySmallMotion = false;
    bool rejectedByNis = false;
    bool inflatedRByNis = false;
};

/**
 * @brief 雅可比质量诊断。
 */
struct IbvsJacobianQuality
{
    double conditionNumber = std::numeric_limits<double>::infinity();
    int effectiveRank = 0;
    double frobeniusNorm = 0.0;
    double minSingularValue = 0.0;
    double predictionRmsePx = 0.0;
    double nisMean = 4.0;

    bool isGood() const;
    bool isAcceptable() const;
};

/**
 * @brief 速度求解结果。
 */
struct IbvsVelocitySolveResult
{
    IbvsError error;
    IbvsVelocityVector velocityMps = IbvsVelocityVector::Zero();
    Eigen::Vector4d featureErrorPx = Eigen::Vector4d::Zero();
    Eigen::Vector3d singularValues = Eigen::Vector3d::Zero();
    double conditionNumber = std::numeric_limits<double>::infinity();
    double dampingMu = 0.0;
    bool velocityLimited = false;
};

inline bool IbvsError::isOk() const
{
    return status == IbvsStatus::OK;
}

inline bool PixelPoint::isFinite() const
{
    return std::isfinite(u) && std::isfinite(v);
}

inline bool PixelRoi::isValid() const
{
    return width > 0 && height > 0;
}

inline double PixelFeaturePair::distancePx() const
{
    const double du = first.u - second.u;
    const double dv = first.v - second.v;
    return std::sqrt(du * du + dv * dv);
}

inline bool PixelFeaturePair::isValid(double minDistancePx) const
{
    return first.isFinite()
        && second.isFinite()
        && distancePx() > minDistancePx;
}

inline IbvsFeatureVector PixelFeaturePair::toVector() const
{
    IbvsFeatureVector feature;
    feature << first.u, first.v, second.u, second.v;
    return feature;
}

inline bool IbvsCameraIntrinsics::isValid() const
{
    return std::isfinite(fx)
        && std::isfinite(fy)
        && std::abs(fx) > 1e-12
        && std::abs(fy) > 1e-12
        && std::isfinite(cx)
        && std::isfinite(cy);
}

inline Eigen::Matrix3d IbvsCameraIntrinsics::toMatrix() const
{
    Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
    k(0, 0) = fx;
    k(1, 1) = fy;
    k(0, 2) = cx;
    k(1, 2) = cy;
    return k;
}

inline bool IbvsJacobianQuality::isGood() const
{
    return conditionNumber < 100.0
        && effectiveRank == 3
        && minSingularValue > 1e-3
        && predictionRmsePx < 15.0
        && nisMean > 2.0
        && nisMean < 6.0;
}

inline bool IbvsJacobianQuality::isAcceptable() const
{
    return conditionNumber < 1e4
        && effectiveRank >= 2
        && frobeniusNorm > 1e-9
        && predictionRmsePx < 30.0;
}

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_TYPES_H
