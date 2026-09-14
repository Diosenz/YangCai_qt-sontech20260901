/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsFeatureGeometry.h
 * @brief     由焊枪特征和焊缝起始点生成 IBVS 期望特征。
 * @details   本文件定义期望特征的像素几何计算入口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_FEATURE_GEOMETRY_H
#define I2R_RLC_IBVS_IBVS_FEATURE_GEOMETRY_H

#include "ibvs/IbvsConfig.h"
#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 期望特征几何参数。
 */
struct IbvsFeatureGeometryParameters
{
    double h1Mm = FEATURE_GEOMETRY_H1_MM_DEFAULT;
    double h2Mm = FEATURE_GEOMETRY_H2_MM_DEFAULT;
    double focalLengthPx = FEATURE_GEOMETRY_FOCAL_LENGTH_PX_DEFAULT;
    double torchTiltScale = FEATURE_GEOMETRY_TORCH_TILT_SCALE_DEFAULT;
    bool useDepthBasedH2 = FEATURE_GEOMETRY_USE_DEPTH_BASED_H2_DEFAULT;
    double minFeatureDistancePx =
        FEATURE_GEOMETRY_MIN_FEATURE_DISTANCE_PX_DEFAULT;
    bool useOverrideH2Px = true;
    double overrideH2Px = FEATURE_GEOMETRY_LEGACY_H2_PX_DEFAULT;
    bool useOverrideUnitDirection = true;
    PixelPoint overrideUnitDirection{
        FEATURE_GEOMETRY_LEGACY_UNIT_U_DEFAULT,
        FEATURE_GEOMETRY_LEGACY_UNIT_V_DEFAULT
    };
};

/**
 * @brief 期望特征生成结果。
 */
struct IbvsFeatureGeometryResult
{
    IbvsError error;
    PixelFeaturePair desiredPoints;
    IbvsFeatureVector desiredFeature = IbvsFeatureVector::Zero();
    PixelPoint unitDirection;
    double h1Px = 0.0;
    double h2Px = 0.0;
};

/**
 * @brief 根据当前焊枪双特征点和焊缝起始点计算期望特征。
 */
IbvsFeatureGeometryResult computeDesiredFeature(
    const PixelFeaturePair& featurePoints,
    const PixelPoint& startPoint,
    double depthM,
    const IbvsFeatureGeometryParameters& parameters);

/**
 * @brief 返回出风口工件默认焊接姿态使用的兼容几何参数。
 */
IbvsFeatureGeometryParameters makeLegacyWorkpieceFeatureGeometryParameters();

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_FEATURE_GEOMETRY_H
