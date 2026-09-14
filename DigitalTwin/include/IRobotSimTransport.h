/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IRobotSimTransport.h
 * @brief     RobotSim 通信传输抽象。
 * @details   本文件定义 DigitalTwin 服务使用的 JSON 请求传输边界，生产实现和
 *            测试替身实现均通过该接口注入。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_I_ROBOT_SIM_TRANSPORT_H
#define I2R_RLC_DIGITAL_TWIN_I_ROBOT_SIM_TRANSPORT_H

#include <QByteArray>
#include <QJsonDocument>
#include <QString>

#include "IDigitalTwinService.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief RobotSim 通信结果。
 *
 * `payload` 保存 RobotSim 有返回值命令的原始 JSON 文本；无返回值命令只表达发送是否成功。
 */
struct RobotSimTransportResult {
    bool succeeded = false;
    QString payload;
    QString message;
    qint64 elapsedMs = -1;
};

/**
 * @brief RobotSim 通信抽象。
 *
 * 生产实现使用 ZeroMQ DEALER 连接 RobotSim 的 ROUTER 服务；测试实现可替换为
 * 测试替身 transport 来验证协议生成，不需要启动 RobotSim。
 */
class IRobotSimTransport {
public:
    virtual ~IRobotSimTransport() = default;

    /**
     * @brief 连接 RobotSim 端点。
     */
    virtual RobotSimTransportResult connect(const RobotSimEndpointConfig& endpoint) = 0;

    /**
     * @brief 断开连接并释放 socket。
     */
    virtual void disconnect() = 0;

    /**
     * @brief 返回当前 socket 是否处于已连接状态。
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief 发送 RobotSim JSON 请求。
     *
     * `expectResponse` 为 true 时等待 RobotSim 回传 JSON；当前 RobotSim 只对
     * `_VALUE_RET` 类通知回传响应。
     */
    virtual RobotSimTransportResult send(
        const QJsonDocument& request,
        bool expectResponse) = 0;

    /**
     * @brief 发送已经按 RobotSim 协议序列化的 UTF-8 JSON。
     *
     * 默认实现兼容测试替身；生产 transport 覆盖本函数，直接发送由
     * nlohmann::json 按 RobotSim 通信格式生成的载荷。
     */
    virtual RobotSimTransportResult sendRaw(
        const QByteArray& payload,
        bool expectResponse)
    {
        return send(QJsonDocument::fromJson(payload), expectResponse);
    }
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_I_ROBOT_SIM_TRANSPORT_H
