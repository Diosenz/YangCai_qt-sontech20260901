/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimZmqTransport.h
 * @brief     RobotSim ZeroMQ 传输实现。
 * @details   本文件声明 DEALER socket 的生命周期和 RobotSim ROUTER/DEALER
 *            帧格式处理，业务协议由上层 DigitalTwin 服务构造。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_ZMQ_TRANSPORT_H
#define I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_ZMQ_TRANSPORT_H

#include <memory>

#include "IRobotSimTransport.h"

namespace zmq {
class context_t;
class socket_t;
} // namespace zmq

namespace i2r_rlc::digital_twin {

/**
 * @brief RobotSim ZeroMQ transport。
 *
 * 本类只维护 DEALER socket 生命周期和 RobotSim ROUTER/DEALER 帧格式，不解析
 * 业务语义。业务命令由 `RobotSimDigitalTwinService` 构造。
 */
class RobotSimZmqTransport final : public IRobotSimTransport {
public:
    RobotSimZmqTransport();
    ~RobotSimZmqTransport() override;

    RobotSimZmqTransport(const RobotSimZmqTransport&) = delete;
    RobotSimZmqTransport& operator=(const RobotSimZmqTransport&) = delete;

    RobotSimTransportResult connect(const RobotSimEndpointConfig& endpoint) override;
    void disconnect() override;
    bool isConnected() const override;
    RobotSimTransportResult send(
        const QJsonDocument& request,
        bool expectResponse) override;
    RobotSimTransportResult sendRaw(
        const QByteArray& payload,
        bool expectResponse) override;

private:
    RobotSimTransportResult sendPayload(
        const QByteArray& payload,
        bool expectResponse);

    std::unique_ptr<zmq::context_t> m_context;
    std::unique_ptr<zmq::socket_t> m_socket;
    RobotSimEndpointConfig m_endpoint;
    bool m_connected = false;
    bool m_loggedSingleJointRequest = false;
    bool m_loggedBatchJointRequest = false;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_ZMQ_TRANSPORT_H
