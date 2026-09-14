/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsSensorFusion.cpp
 * @brief     IBVS 与 LVS 观测融合实现。
 * @details   本文件实现只依赖 IBVS 自有类型的期望特征融合与降级逻辑。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <cmath>

#include "ibvs/IbvsSensorFusion.h"

namespace i2r_rlc::ibvs {

namespace {

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

bool isFiniteSigmaVector(const Eigen::Vector4d& sigmas)
{
    for (int i = 0; i < sigmas.size(); ++i)
    {
        if (!std::isfinite(sigmas(i)) || sigmas(i) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

bool hasValidConfidence(double confidence)
{
    return std::isfinite(confidence) && confidence >= 0.0;
}

int absoluteFrameDelta(int lhs, int rhs)
{
    const int delta = lhs - rhs;
    return delta < 0 ? -delta : delta;
}

Eigen::Vector4d scaledSigmas(
    const IbvsFusionConfig& config,
    double scale)
{
    return config.baseChannelSigmasPx * scale;
}

IbvsFusionResult fallbackResult(
    const IbvsFeatureVector& fallbackDesiredFeature,
    const IbvsFusionConfig& config,
    const std::string& reason)
{
    IbvsFusionResult result;
    if (!isFiniteFeature(fallbackDesiredFeature))
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "fallback desired feature is invalid"
        };
        result.fallbackReason = reason;
        return result;
    }

    result.error = { IbvsStatus::OK, std::string() };
    result.desiredFeature = fallbackDesiredFeature;
    result.desiredPoints = {
        { fallbackDesiredFeature(0), fallbackDesiredFeature(1) },
        { fallbackDesiredFeature(2), fallbackDesiredFeature(3) }
    };
    result.channelSigmasPx =
        scaledSigmas(config, config.fallbackChannelSigmaScale);
    result.usedFallback = true;
    result.fallbackReason = reason;
    return result;
}

} // namespace

IbvsFusionResult IbvsSensorFusion::fuseDesiredFeature(
    const IbvsTorchObservation& torchObservation,
    const std::optional<IbvsLvsObservation>& lvsObservation,
    const IbvsFeatureVector& fallbackDesiredFeature,
    const IbvsFusionConfig& config)
{
    IbvsFusionResult result;

    if (config.maxFrameDelta < 0
        || !hasValidConfidence(config.minTorchConfidence)
        || !hasValidConfidence(config.minLvsConfidence)
        || !hasValidConfidence(config.minLvsCenterlineQuality)
        || !isFiniteSigmaVector(config.baseChannelSigmasPx)
        || !std::isfinite(config.lvsChannelSigmaScale)
        || config.lvsChannelSigmaScale <= 0.0
        || !std::isfinite(config.fallbackChannelSigmaScale)
        || config.fallbackChannelSigmaScale <= 0.0)
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "fusion config is invalid"
        };
        return result;
    }

    if (!torchObservation.featurePoints.isValid(
            config.geometryParameters.minFeatureDistancePx)
        || !std::isfinite(torchObservation.confidence)
        || torchObservation.confidence < config.minTorchConfidence)
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "torch observation is invalid"
        };
        return result;
    }

    if (!config.preferLvs)
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs fusion is disabled");
    }

    if (!lvsObservation.has_value())
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs observation is missing");
    }

    const IbvsLvsObservation& lvs = lvsObservation.value();
    if (!std::isfinite(lvs.confidence)
        || lvs.confidence < config.minLvsConfidence)
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs confidence is below threshold");
    }

    if (!std::isfinite(lvs.centerlineQuality)
        || lvs.centerlineQuality < config.minLvsCenterlineQuality)
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs centerline quality is below threshold");
    }

    if (torchObservation.frameIndex >= 0
        && lvs.frameIndex >= 0
        && absoluteFrameDelta(torchObservation.frameIndex, lvs.frameIndex)
            > config.maxFrameDelta)
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs observation is stale");
    }

    PixelPoint selectedStartPoint;
    bool hasSelectedStartPoint = false;
    if (lvs.hasStartPoint && lvs.startPoint.isFinite())
    {
        selectedStartPoint = lvs.startPoint;
        hasSelectedStartPoint = true;
    }
    else if (lvs.hasWeldPoint && lvs.weldPoint.isFinite())
    {
        selectedStartPoint = lvs.weldPoint;
        hasSelectedStartPoint = true;
    }

    if (!hasSelectedStartPoint)
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs start point is invalid");
    }

    const IbvsFeatureGeometryResult geometry = computeDesiredFeature(
        torchObservation.featurePoints,
        selectedStartPoint,
        torchObservation.depthM,
        config.geometryParameters);
    if (!geometry.error.isOk())
    {
        return fallbackResult(
            fallbackDesiredFeature,
            config,
            "lvs geometry failed: " + geometry.error.message);
    }

    result.error = { IbvsStatus::OK, std::string() };
    result.desiredFeature = geometry.desiredFeature;
    result.desiredPoints = geometry.desiredPoints;
    result.channelSigmasPx =
        scaledSigmas(config, config.lvsChannelSigmaScale);
    result.usedLvs = true;
    return result;
}

} // namespace i2r_rlc::ibvs
