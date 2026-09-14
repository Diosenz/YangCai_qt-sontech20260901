/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      DigitalTwinLocalizationVirtualReferenceProvider.cpp
 * @brief     Localization 虚拟参考适配器实现。
 * @details   本文件实现 DigitalTwin 虚拟参考结果到 Localization 快照的字段转换；
 *            不创建替代点云，也不执行定位计算。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include "DigitalTwinLocalizationVirtualReferenceProvider.h"

namespace i2r_rlc::digital_twin {

namespace {

DigitalTwinPose6D toDigitalTwinPose(
    const localization::Pose6D& pose)
{
    return {
        pose.x_mm,
        pose.y_mm,
        pose.z_mm,
        pose.rx_deg,
        pose.ry_deg,
        pose.rz_deg
    };
}

localization::Point3D_mm toLocalizationPoint(
    const DigitalTwinPoint3D_mm& point)
{
    return {
        point.x_mm,
        point.y_mm,
        point.z_mm
    };
}

} // namespace

DigitalTwinLocalizationVirtualReferenceProvider::
    DigitalTwinLocalizationVirtualReferenceProvider(IDigitalTwinService& service)
    : m_service(service)
{
}

localization::LocalizationVirtualReferenceSnapshot
DigitalTwinLocalizationVirtualReferenceProvider::requestSnapshot(
    const localization::LocalizationVirtualReferenceRequest& request)
{
    localization::LocalizationVirtualReferenceSnapshot snapshot;
    snapshot.available = false;
    snapshot.referenceId = request.referenceId;
    snapshot.referencePose = request.workpiecePose;

    const DigitalTwinStatusSnapshot currentStatus = m_service.status();
    if (currentStatus.state == DigitalTwinServiceState::Unavailable) {
        snapshot.message = QStringLiteral("RobotSim 未连接，无法提供定位虚拟参考。");
        return snapshot;
    }

    DigitalTwinVirtualReferenceRequest digitalTwinRequest;
    digitalTwinRequest.frameId = request.frameId;
    digitalTwinRequest.workpieceId = request.referenceId;
    digitalTwinRequest.workpiecePose = toDigitalTwinPose(request.workpiecePose);
    digitalTwinRequest.needLaserLineImagePoints = request.needLaserLinePoints;
    digitalTwinRequest.needLaserLinePoints = request.needLaserLinePoints;
    digitalTwinRequest.needLocalPointCloud = request.needWorkpiecePointCloud;
    digitalTwinRequest.needWeldSeamPoint = true;

    const DigitalTwinVirtualReferenceResult result =
        m_service.requestVirtualReference(digitalTwinRequest);
    snapshot.message = result.message;
    snapshot.available = result.available;
    snapshot.referenceId = result.workpieceId.isEmpty()
        ? request.referenceId
        : result.workpieceId;
    for (const DigitalTwinImagePoint2D_px& point : result.laserLineImagePoints_px) {
        snapshot.laserLineImagePoints_px.push_back({ point.x_px, point.y_px });
    }
    for (const DigitalTwinPoint3D_mm& point : result.laserLinePoints_mm) {
        snapshot.laserLinePoints_mm.push_back(toLocalizationPoint(point));
    }
    for (const DigitalTwinPoint3D_mm& point : result.localPointCloud_mm) {
        snapshot.workpiecePointCloud_mm.push_back(toLocalizationPoint(point));
    }

    if (!snapshot.available) {
        return snapshot;
    }

    if (snapshot.workpiecePointCloud_mm.empty()
        && snapshot.laserLinePoints_mm.empty()
        && snapshot.laserLineImagePoints_px.empty()) {
        snapshot.available = false;
        snapshot.message = QStringLiteral("RobotSim 返回成功但未包含虚拟参考数据。");
    }
    return snapshot;
}

} // namespace i2r_rlc::digital_twin
