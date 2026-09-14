/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsFeatureGeometry.cpp
 * @brief     IBVS 期望特征像素几何计算实现。
 * @details   本文件根据当前焊枪双特征点、焊缝起始点和几何参数生成期望特征。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <cmath>

#include "ibvs/IbvsFeatureGeometry.h"

namespace i2r_rlc::ibvs {

IbvsFeatureGeometryResult computeDesiredFeature(
    const PixelFeaturePair& featurePoints,
    const PixelPoint& startPoint,
    double depthM,
    const IbvsFeatureGeometryParameters& parameters)
{
    IbvsFeatureGeometryResult result;

    if (!featurePoints.isValid(parameters.minFeatureDistancePx)
        || !startPoint.isFinite())
    {
        result.error = {
            IbvsStatus::DEGENERATE_GEOMETRY,
            "feature points or weld start point are invalid"
        };
        return result;
    }

    if (parameters.h2Mm <= 0.0 || parameters.h1Mm < 0.0)
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "feature geometry physical distances are invalid"
        };
        return result;
    }

    const double du = featurePoints.first.u - featurePoints.second.u;
    const double dv = featurePoints.first.v - featurePoints.second.v;
    const double dNorm = std::sqrt(du * du + dv * dv);
    double unitU = du / dNorm;
    double unitV = dv / dNorm;
    if (parameters.useOverrideUnitDirection)
    {
        if (!parameters.overrideUnitDirection.isFinite())
        {
            result.error = {
                IbvsStatus::INVALID_ARGUMENT,
                "override unit direction is invalid"
            };
            return result;
        }

        const double overrideNorm = std::sqrt(
            parameters.overrideUnitDirection.u * parameters.overrideUnitDirection.u
            + parameters.overrideUnitDirection.v * parameters.overrideUnitDirection.v);
        if (overrideNorm <= parameters.minFeatureDistancePx
            || !std::isfinite(overrideNorm))
        {
            result.error = {
                IbvsStatus::INVALID_ARGUMENT,
                "override unit direction norm is invalid"
            };
            return result;
        }

        unitU = parameters.overrideUnitDirection.u / overrideNorm;
        unitV = parameters.overrideUnitDirection.v / overrideNorm;
    }

    double h2Px = dNorm;
    if (parameters.useOverrideH2Px)
    {
        h2Px = parameters.overrideH2Px;
    }
    else if (parameters.useDepthBasedH2)
    {
        if (depthM <= 0.0 || !std::isfinite(depthM))
        {
            result.error = {
                IbvsStatus::INVALID_ARGUMENT,
                "depth mode requires a positive depth value in meters"
            };
            return result;
        }

        const double depthMm = depthM * 1000.0;
        h2Px = parameters.focalLengthPx
            * parameters.h2Mm
            * parameters.torchTiltScale
            / depthMm;
    }

    if (h2Px <= parameters.minFeatureDistancePx || !std::isfinite(h2Px))
    {
        result.error = {
            IbvsStatus::DEGENERATE_GEOMETRY,
            "computed desired feature spacing is invalid"
        };
        return result;
    }

    const double h1Px = h2Px * (parameters.h1Mm / parameters.h2Mm);
    const PixelPoint p2Star{
        startPoint.u + unitU * h1Px,
        startPoint.v + unitV * h1Px
    };
    const PixelPoint p1Star{
        startPoint.u + unitU * (h1Px + h2Px),
        startPoint.v + unitV * (h1Px + h2Px)
    };

    result.error = { IbvsStatus::OK, std::string() };
    result.desiredPoints = { p1Star, p2Star };
    result.desiredFeature = result.desiredPoints.toVector();
    result.unitDirection = { unitU, unitV };
    result.h1Px = h1Px;
    result.h2Px = h2Px;
    return result;
}

IbvsFeatureGeometryParameters makeLegacyWorkpieceFeatureGeometryParameters()
{
    IbvsFeatureGeometryParameters parameters;
    return parameters;
}

} // namespace i2r_rlc::ibvs
