/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimZmqTransport.cpp
 * @brief     RobotSim ZeroMQ 传输实现。
 * @details   本文件实现 DEALER socket 的连接、发送、接收和超时处理；
 *            JSON 业务语义由 DigitalTwin 服务层负责。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include <vector>

#include <iterator>
#include <QByteArray>
#include <QDebug>
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QJsonObject>
#include <zmq_addon.hpp>
#include <zmq.hpp>

#include "RobotSimZmqTransport.h"

namespace i2r_rlc::digital_twin {

namespace {

QString endpointText(const RobotSimEndpointConfig& endpoint)
{
    return QStringLiteral("tcp://%1:%2").arg(endpoint.host).arg(endpoint.port);
}

bool payloadFromDealerFrames(
    const std::vector<zmq::message_t>& frames,
    QString *payload,
    QString *message)
{
    if (payload == nullptr || message == nullptr) {
        return false;
    }
    payload->clear();
    message->clear();

    // RobotSim 官方示例约定响应固定为 [空分隔帧, 数据帧]。
    // 这里不再用 back() 容忍未知信封，避免把异常帧误判为合法业务响应。
    if (frames.size() != 2U) {
        *message = QStringLiteral(
            "RobotSim 响应帧数量错误：期望 2 帧，实际 %1 帧。")
            .arg(static_cast<qulonglong>(frames.size()));
        return false;
    }
    if (frames[0].size() != 0U) {
        *message = QStringLiteral(
            "RobotSim 响应第 0 帧不是空分隔帧，长度为 %1 字节。")
            .arg(static_cast<qulonglong>(frames[0].size()));
        return false;
    }

    const zmq::message_t& payloadFrame = frames[1];
    *payload = QString::fromUtf8(
        static_cast<const char*>(payloadFrame.data()),
        static_cast<qsizetype>(payloadFrame.size()));
    if (payload->isEmpty()) {
        *message = QStringLiteral("RobotSim 响应数据帧为空。");
        return false;
    }
    return true;
}

} // namespace

RobotSimZmqTransport::RobotSimZmqTransport()
    : m_context(std::make_unique<zmq::context_t>(1))
{
}

RobotSimZmqTransport::~RobotSimZmqTransport()
{
    disconnect();
}

RobotSimTransportResult RobotSimZmqTransport::connect(
    const RobotSimEndpointConfig& endpoint)
{
    disconnect();

    if (endpoint.host.trimmed().isEmpty() || endpoint.port == 0) {
        return {
            false,
            {},
            QStringLiteral("RobotSim endpoint 无效：host 或 port 为空。"),
            -1
        };
    }

    try {
        m_socket = std::make_unique<zmq::socket_t>(
            *m_context,
            zmq::socket_type::dealer);
        m_socket->set(zmq::sockopt::linger, 0);
        m_socket->set(zmq::sockopt::immediate, 1);
        m_socket->set(zmq::sockopt::rcvtimeo, endpoint.requestTimeoutMs);
        m_socket->set(zmq::sockopt::sndtimeo, endpoint.requestTimeoutMs);
        m_socket->connect(endpointText(endpoint).toStdString());
    } catch (const zmq::error_t& error) {
        m_socket.reset();
        return {
            false,
            {},
            QStringLiteral("连接 RobotSim 失败：%1").arg(error.what()),
            -1
        };
    }

    m_endpoint = endpoint;
    m_connected = true;
    m_loggedSingleJointRequest = false;
    m_loggedBatchJointRequest = false;
    return {
        true,
        {},
        QStringLiteral("已连接 RobotSim：%1").arg(endpointText(endpoint)),
        0
    };
}

void RobotSimZmqTransport::disconnect()
{
    if (m_socket) {
        try {
            m_socket->close();
        } catch (...) {
        }
        m_socket.reset();
    }

    m_connected = false;
}

bool RobotSimZmqTransport::isConnected() const
{
    return m_connected;
}

RobotSimTransportResult RobotSimZmqTransport::send(
    const QJsonDocument& request,
    bool expectResponse)
{
    return sendPayload(request.toJson(QJsonDocument::Compact), expectResponse);
}

RobotSimTransportResult RobotSimZmqTransport::sendRaw(
    const QByteArray& payload,
    bool expectResponse)
{
    return sendPayload(payload, expectResponse);
}

RobotSimTransportResult RobotSimZmqTransport::sendPayload(
    const QByteArray& payload,
    bool expectResponse)
{
    if (!m_connected || !m_socket) {
        return {
            false,
            {},
            QStringLiteral("RobotSim transport 未连接。"),
            -1
        };
    }

    QElapsedTimer timer;
    timer.start();

    if (payload.contains("\"SET_Q_VEC\"")) {
        const bool isBatch = payload.startsWith("[[");
        bool& logged = isBatch
            ? m_loggedBatchJointRequest
            : m_loggedSingleJointRequest;
        if (!logged) {
            qInfo().noquote()
                << (isBatch
                        ? "RobotSim TX batch:"
                        : "RobotSim TX joint:")
                << QString::fromUtf8(payload);
            logged = true;
        }
    }

    try {
        std::vector<zmq::message_t> requestFrames;
        requestFrames.emplace_back();
        requestFrames.emplace_back(payload.constData(), static_cast<size_t>(payload.size()));

        if (!zmq::send_multipart(*m_socket, requestFrames, zmq::send_flags::none)) {
            m_connected = false;
            return {
                false,
                {},
                QStringLiteral("发送 RobotSim 请求失败。"),
                timer.elapsed()
            };
        }

        if (!expectResponse) {
            return {
                true,
                {},
                QStringLiteral("RobotSim 请求已发送。"),
                timer.elapsed()
            };
        }

        zmq::pollitem_t items[] = {
            { *m_socket, 0, ZMQ_POLLIN, 0 }
        };
        if (zmq::poll(items, 1, m_endpoint.requestTimeoutMs) <= 0
            || !(items[0].revents & ZMQ_POLLIN)) {
            return {
                false,
                {},
                QStringLiteral("等待 RobotSim 响应超时。"),
                timer.elapsed()
            };
        }

        std::vector<zmq::message_t> responseFrames;
        if (!zmq::recv_multipart(
                *m_socket,
                std::back_inserter(responseFrames),
                zmq::recv_flags::none)) {
            return {
                false,
                {},
                QStringLiteral("接收 RobotSim 响应失败。"),
                timer.elapsed()
            };
        }

        QString responsePayload;
        QString responseFrameMessage;
        if (!payloadFromDealerFrames(
                responseFrames,
                &responsePayload,
                &responseFrameMessage)) {
            return {
                false,
                {},
                responseFrameMessage,
                timer.elapsed()
            };
        }

        return {
            true,
            responsePayload,
            QStringLiteral("RobotSim 响应成功。"),
            timer.elapsed()
        };
    } catch (const zmq::error_t& error) {
        m_connected = false;
        return {
            false,
            {},
            QStringLiteral("RobotSim ZeroMQ 异常：%1").arg(error.what()),
            timer.elapsed()
        };
    }
}

} // namespace i2r_rlc::digital_twin
