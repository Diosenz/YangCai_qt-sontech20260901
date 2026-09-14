/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimDigitalTwinService.h
 * @brief     RobotSim 数字孪生服务实现。
 * @details   本文件声明基于 RobotSim JSON 通信协议的服务实现，负责状态同步、
 *            命令转换和虚拟参考请求，不拥有 RobotSim 进程生命周期。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_DIGITAL_TWIN_SERVICE_H
#define I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_DIGITAL_TWIN_SERVICE_H

#include <memory>

#include <QJsonDocument>

#include "IDigitalTwinService.h"
#include "IRobotSimTransport.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief 基于 RobotSim JSON 通信协议的数字孪生服务。
 *
 * 本服务位于主项目和外部 RobotSim 进程之间，只做状态同步和协议转换。
 * 它不拥有 RobotSim 进程生命周期，也不承担定位求解或虚拟点云生成。
 */
class RobotSimDigitalTwinService final : public IDigitalTwinService {
public:
    RobotSimDigitalTwinService();
    explicit RobotSimDigitalTwinService(std::unique_ptr<IRobotSimTransport> transport);
    ~RobotSimDigitalTwinService() override;

    RobotSimDigitalTwinService(const RobotSimDigitalTwinService&) = delete;
    RobotSimDigitalTwinService& operator=(const RobotSimDigitalTwinService&) = delete;

    DigitalTwinStartResult start() override;
    DigitalTwinStartResult start(const RobotSimEndpointConfig& endpoint) override;
    void stop() override;
    DigitalTwinStatusSnapshot status() const override;
    DigitalTwinCommandResult setRobotJoints(
        int robotId,
        const std::vector<double>& jointValues) override;

    /**
     * @brief 连续发送机器人和变位机的无返回值关节向量请求。
     *
     * 两组关节值按 RobotSim 主动关节顺序传入，单位为 rad（转动关节）或
     * mm（移动关节）。拆成两个单请求可避免 void-only batch 的空响应留在
     * DEALER 接收队列中，影响下一次有返回值的虚拟参考请求。
     */
    DigitalTwinCommandResult synchronizeJointVectors(
        int robotId,
        const std::vector<double>& robotJointValues,
        int positionerId,
        const std::vector<double>& positionerJointValues);
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

private:
    DigitalTwinCommandResult sendCommand(const QJsonDocument& request);
    DigitalTwinCommandResult sendCommandRaw(const QByteArray& payload);
    RobotSimTransportResult sendQuery(const QJsonDocument& request);
    RobotSimTransportResult sendQueryRaw(const QByteArray& payload);
    void updateFromTransportResult(
        const RobotSimTransportResult& result,
        DigitalTwinServiceState successState);

    std::unique_ptr<IRobotSimTransport> m_transport;
    DigitalTwinStatusSnapshot m_status;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_DIGITAL_TWIN_SERVICE_H
