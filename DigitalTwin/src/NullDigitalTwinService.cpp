/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullDigitalTwinService.cpp
 * @brief     数字孪生空服务实现。
 * @details   本文件实现 DigitalTwin 服务不可用时的结构化返回，便于 UI 和上层
 *            业务在未连接 RobotSim 时保持明确状态。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include "NullDigitalTwinService.h"

namespace i2r_rlc::digital_twin {

namespace {

QString unavailableMessage()
{
    return QStringLiteral("DigitalTwinService 不可用：未连接 RobotSim。");
}

} // namespace

DigitalTwinStartResult NullDigitalTwinService::start()
{
    return {
        DigitalTwinServiceState::Unavailable,
        QStringLiteral("DigitalTwinService 不可用：未连接 RobotSim；定位求解不放在 digital_twin 中。")
    };
}

DigitalTwinStartResult NullDigitalTwinService::start(const RobotSimEndpointConfig& endpoint)
{
    Q_UNUSED(endpoint)
    return start();
}

void NullDigitalTwinService::stop()
{
}

DigitalTwinStatusSnapshot NullDigitalTwinService::status() const
{
    DigitalTwinStatusSnapshot snapshot;
    snapshot.state = DigitalTwinServiceState::Unavailable;
    snapshot.lastError = unavailableMessage();
    return snapshot;
}

DigitalTwinCommandResult NullDigitalTwinService::setRobotJoints(
    int robotId,
    const std::vector<double>& jointValues)
{
    Q_UNUSED(robotId)
    Q_UNUSED(jointValues)
    return { false, unavailableMessage() };
}

DigitalTwinCommandResult NullDigitalTwinService::setRobotTcpPose(
    int robotId,
    const DigitalTwinPose6D& pose)
{
    Q_UNUSED(robotId)
    Q_UNUSED(pose)
    return { false, unavailableMessage() };
}

DigitalTwinPoseResult NullDigitalTwinService::getObjectPose(int objectId)
{
    Q_UNUSED(objectId)
    DigitalTwinPoseResult result;
    result.available = false;
    result.message = unavailableMessage();
    return result;
}

DigitalTwinCommandResult NullDigitalTwinService::setObjectPose(
    int objectId,
    const DigitalTwinPose6D& pose,
    bool isRelative,
    bool updateChildren)
{
    Q_UNUSED(objectId)
    Q_UNUSED(pose)
    Q_UNUSED(isRelative)
    Q_UNUSED(updateChildren)
    return { false, unavailableMessage() };
}

DigitalTwinCommandResult NullDigitalTwinService::selectWorkpiece(
    const DigitalTwinWorkpieceSelection& selection)
{
    Q_UNUSED(selection)
    return { false, unavailableMessage() };
}

DigitalTwinVirtualReferenceResult NullDigitalTwinService::requestVirtualReference(
    const DigitalTwinVirtualReferenceRequest& request)
{
    DigitalTwinVirtualReferenceResult result;
    result.available = false;
    result.supported = false;
    result.frameId = request.frameId;
    result.workpieceId = request.workpieceId;
    result.referencePose = request.workpiecePose;
    result.message = QStringLiteral("RobotSim 未连接，无法提供虚拟激光线或局部点云。");
    return result;
}

} // namespace i2r_rlc::digital_twin
