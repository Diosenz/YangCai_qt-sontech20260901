/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullDigitalTwinService.h
 * @brief     数字孪生空服务实现。
 * @details   本文件定义 DigitalTwin 服务不可用时的显式空实现，调用方可获得
 *            结构化不可用状态而不需要判空。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_NULL_DIGITAL_TWIN_SERVICE_H
#define I2R_RLC_DIGITAL_TWIN_NULL_DIGITAL_TWIN_SERVICE_H

#include "IDigitalTwinService.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief 数字孪生空实现。
 */
class NullDigitalTwinService final : public IDigitalTwinService {
public:
    DigitalTwinStartResult start() override;
    DigitalTwinStartResult start(const RobotSimEndpointConfig& endpoint) override;
    void stop() override;
    DigitalTwinStatusSnapshot status() const override;
    DigitalTwinCommandResult setRobotJoints(
        int robotId,
        const std::vector<double>& jointValues) override;
    DigitalTwinCommandResult setRobotTcpPose(
        int robotId,
        const DigitalTwinPose6D& pose) override;
    DigitalTwinPoseResult getObjectPose(int objectId) override;
    DigitalTwinCommandResult setObjectPose(
        int objectId,
        const DigitalTwinPose6D& pose,
        bool isRelative,
        bool updateChildren) override;
    DigitalTwinCommandResult selectWorkpiece(
        const DigitalTwinWorkpieceSelection& selection) override;
    DigitalTwinVirtualReferenceResult requestVirtualReference(
        const DigitalTwinVirtualReferenceRequest& request) override;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_NULL_DIGITAL_TWIN_SERVICE_H
