/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimVirtualReferenceCache.cpp
 * @brief     UI 层 RobotSim 虚拟参考缓存实现。
 * @details   本文件只做 DigitalTwin 结果到 Localization/IBVS 值对象的转换。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 29 日
 */

#include <algorithm>
#include <limits>

#include "RobotSimVirtualReferenceCache.h"

namespace i2r_rlc::ui {

namespace {

int toFrameIndex(const qint64 frameId)
{
    if (frameId < 0) {
        return -1;
    }

    const qint64 maxFrameIndex =
        static_cast<qint64>(std::numeric_limits<int>::max());
    return static_cast<int>(std::min(frameId, maxFrameIndex));
}

i2r_rlc::localization::Point3D_mm toLocalizationPoint(
    const i2r_rlc::digital_twin::DigitalTwinPoint3D_mm& point)
{
    return {
        point.x_mm,
        point.y_mm,
        point.z_mm
    };
}

i2r_rlc::ibvs::PixelPoint toIbvsPoint(
    const i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px& point)
{
    return {
        point.x_px,
        point.y_px
    };
}

} // namespace

std::optional<i2r_rlc::ibvs::IbvsLvsObservation> makeIbvsLvsObservation(
    const i2r_rlc::ibvs::IbvsLaserPrior& prior)
{
    if (prior.virtualLaserLine.empty() && !prior.hasVirtualWeldPoint) {
        return std::nullopt;
    }

    i2r_rlc::ibvs::IbvsLvsObservation observation;
    observation.frameIndex = prior.frameIndex;
    if (!prior.virtualLaserLine.empty()) {
        observation.startPoint = prior.virtualLaserLine.front();
        observation.hasStartPoint = observation.startPoint.isFinite();
        observation.centerlineQuality =
            prior.virtualLaserLine.size() > 1U ? 0.8 : 0.2;
    }
    if (prior.hasVirtualWeldPoint) {
        observation.weldPoint = prior.virtualWeldPoint;
        observation.hasWeldPoint = observation.weldPoint.isFinite();
    }
    observation.confidence = observation.hasStartPoint || observation.hasWeldPoint
        ? 0.9
        : 0.0;

    if (!observation.hasStartPoint && !observation.hasWeldPoint) {
        return std::nullopt;
    }
    return observation;
}

RobotSimVirtualReferenceCache makeRobotSimVirtualReferenceCache(
    const i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult& result,
    const qint64 frameId,
    const int sensorId)
{
    RobotSimVirtualReferenceCache cache;
    cache.available = result.available;
    cache.sensorId = sensorId;
    cache.frameId = frameId;
    cache.message = result.message;

    cache.localizationSnapshot.available = result.available
        && (!result.localPointCloud_mm.empty()
            || !result.laserLinePoints_mm.empty());
    cache.localizationSnapshot.referenceId = result.workpieceId;
    cache.localizationSnapshot.referencePose = {
        result.referencePose.x_mm,
        result.referencePose.y_mm,
        result.referencePose.z_mm,
        result.referencePose.rx_deg,
        result.referencePose.ry_deg,
        result.referencePose.rz_deg
    };
    cache.localizationSnapshot.message = result.message;
    for (const auto& point : result.laserLineImagePoints_px) {
        cache.localizationSnapshot.laserLineImagePoints_px.push_back({
            point.x_px,
            point.y_px
        });
    }
    for (const auto& point : result.laserLinePoints_mm) {
        cache.localizationSnapshot.laserLinePoints_mm.push_back(
            toLocalizationPoint(point));
    }
    for (const auto& point : result.localPointCloud_mm) {
        cache.localizationSnapshot.workpiecePointCloud_mm.push_back(
            toLocalizationPoint(point));
    }

    cache.ibvsLaserPrior.frameIndex = toFrameIndex(frameId);
    for (const auto& point : result.laserLineImagePoints_px) {
        cache.ibvsLaserPrior.virtualLaserLine.push_back(toIbvsPoint(point));
    }
    if (result.hasSeamPointImage_px) {
        cache.ibvsLaserPrior.hasVirtualWeldPoint = true;
        cache.ibvsLaserPrior.virtualWeldPoint =
            toIbvsPoint(result.seamPointImage_px);
    }
    cache.hasIbvsLaserPrior = result.available
        && (!cache.ibvsLaserPrior.virtualLaserLine.empty()
            || cache.ibvsLaserPrior.hasVirtualWeldPoint);
    if (cache.hasIbvsLaserPrior) {
        cache.ibvsLvsObservation =
            makeIbvsLvsObservation(cache.ibvsLaserPrior);
    }

    return cache;
}

} // namespace i2r_rlc::ui
