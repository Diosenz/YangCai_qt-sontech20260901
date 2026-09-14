/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 * 
 * @file        Client.cpp
 * @brief       机器人控制柜 ZeroMQ 客户端实现。
 * @details     负责连接、断开和按 ZMQ_STREAM 帧格式发送/接收控制柜消息。
 * 
 * @version     1.1
 * @author      I2R Laboratory
 * @date        2026 年 6 月 15 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.1      <td>I2R Lab     <td>260615      <td>同步 ZMQ_STREAM 通信模式</tr>
 * </table>
 * 
 ******************************************************************************/

#include <utility>

#include <fmt/format.h>

#include "Client.h"
#include "Config.h"
#include "RobotProtocolCodec.h"
#include "RobotStateSnap.h"

using namespace i2r_rlc::robot_comm;

std::expected<void, Client::Error> Client::connect(
    std::string_view robotIP, uint16_t robotPort)
{
    disconnect();

    m_robotIP = robotIP;
    m_robotPort = robotPort;

    try { m_socket = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::stream); }
    catch (...) { return std::unexpected(Error::SOCKET_CREATE_FAILED); }

    m_socket->set(zmq::sockopt::rcvtimeo, config::SOCKET_CONNECT_TIMEOUT);
    m_socket->set(zmq::sockopt::sndtimeo, config::SOCKET_CONNECT_TIMEOUT);
    m_socket->set(zmq::sockopt::linger, 0);

    try { m_socket->connect(fmt::format("tcp://{}:{}", robotIP, robotPort)); }
    catch (...) { return std::unexpected(Error::SOCKET_CONNECT_FAILED); }

    if (!waitPeerID())
    {
        disconnect();
        return std::unexpected(Error::SOCKET_CONNECT_FAILED);
    }

    m_isConnected.store(true, std::memory_order_release);
    return {};
}

void Client::disconnect()
{
    if (m_socket)
    {
        try
        {
            if (m_isConnected.load(std::memory_order_acquire) && !m_peerID.empty())
            {
                zmq::message_t peerId(m_peerID.data(), m_peerID.size());
                zmq::message_t emptyData;
                m_socket->send(peerId, zmq::send_flags::sndmore);
                m_socket->send(emptyData, zmq::send_flags::none);
            }

            m_socket->close();
        }
        catch (...) { fmt::print("Client::disconnect() FAILED"); }
        m_socket.reset();
    }

    m_peerID.clear();
    m_isConnected.store(false, std::memory_order_release);
}

std::expected<void, Client::Error> Client::sendAndReceive(std::string_view message)
{
    const auto result = sendAndReceiveDetailed(message);
    if (!result) {
        return std::unexpected(result.error());
    }
    return {};
}

std::expected<Client::TransactionResult, Client::Error>
Client::sendAndReceiveDetailed(std::string_view message)
{
    if (!m_isConnected.load(std::memory_order_acquire) || !m_socket || m_peerID.empty())
    {
        return std::unexpected(Error::SOCKET_NOT_CONNECTED);
    }

    try
    {
        zmq::message_t peerId(m_peerID.data(), m_peerID.size());
        if (!m_socket->send(peerId, zmq::send_flags::sndmore))
        {
            m_isConnected.store(false, std::memory_order_release);
            return std::unexpected(Error::SEND_FAILED);
        }

        if (zmq::message_t request(message); !m_socket->send(request, zmq::send_flags::none))
        {
            m_isConnected.store(false, std::memory_order_release);
            return std::unexpected(Error::SEND_FAILED);
        }
        
        if (zmq::message_t recvIdentity; !m_socket->recv(recvIdentity))
        {
            m_isConnected.store(false, std::memory_order_release);
            return std::unexpected(Error::RECV_FAILED);
        }

        if (zmq::message_t response; !m_socket->recv(response, zmq::recv_flags::none))
        {
            m_isConnected.store(false, std::memory_order_release);
            return std::unexpected(Error::RECV_FAILED);
        }
        else
        {
            if (response.empty())
            {
                m_isConnected.store(false, std::memory_order_release);
                return std::unexpected(Error::RESPONSE_EMPTY);
            }

            const auto& responseStr = response.to_string();
            RobotState state;
            if (!CRobotProtocolCodec::decodeRobotState(responseStr, &state)) {
                return std::unexpected(Error::RESPONSE_FORMAT_ERROR);
            }
            RobotStateSnap::instance().updateState(state);
            return TransactionResult{state, responseStr};
        }
    }
    catch (...)
    {
        m_isConnected.store(false, std::memory_order_release);
        return std::unexpected(Error::ZMQ_ERROR);
    }
}

RobotState Client::lastState() const
{
    return RobotStateSnap::instance().getState();
}

bool Client::waitPeerID()
{
    zmq::message_t identity;
    zmq::message_t emptyData;

    if (!m_socket->recv(identity)) return false;
    if (!m_socket->recv(emptyData)) return false;
    if (identity.empty()) return false;
    if (!emptyData.empty()) return false;

    m_peerID.assign(
        static_cast<const char *>(identity.data()),
        identity.size());
    return true;
}
