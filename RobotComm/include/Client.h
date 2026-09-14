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
 * @file        Client.h
 * @brief       机器人控制柜 ZeroMQ 客户端。
 * @details     该客户端以单例形式维护控制柜连接，供 ClientQueue 串行发送控制消息。
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

#pragma once
#ifndef I2R_RLC_ROBOT_COMM_CLIENT_H
#define I2R_RLC_ROBOT_COMM_CLIENT_H

#include <atomic>
#include <memory>
#include <string_view>
#include <string>

#include <expected>
#include <zmq.hpp>

#include "Config.h"
#include "RobotState.h"

namespace i2r_rlc::robot_comm
{
    class ClientQueue;

    class Client final
    {
        friend class ClientQueue;

    public:
        enum class Error
        {
            UNKNOWN_ERROR,
            SOCKET_CREATE_FAILED,
            SOCKET_CONNECT_FAILED,
            SOCKET_NOT_CONNECTED,
            SEND_FAILED,
            RECV_FAILED,
            RESPONSE_EMPTY,
            RESPONSE_FORMAT_ERROR,
            ZMQ_ERROR
        };

    private:
        std::unique_ptr<zmq::context_t> m_context = std::make_unique<zmq::context_t>(1);
        std::unique_ptr<zmq::socket_t> m_socket;
        std::string m_peerID;
        std::string m_robotIP = config::ROBOT_IP;
        uint16_t m_robotPort = config::ROBOT_PORT;
        std::atomic<bool> m_isConnected = false;

    private:
        explicit Client() = default;
        ~Client() { disconnect(); }

    public:
        Client(const Client&) = delete;
        Client(Client&&) = delete;
        Client& operator=(const Client&) = delete;
        Client& operator=(Client&&) = delete;

        static Client& instance()
        {
            static Client s_instance;
            return s_instance;
        }

        std::expected<void, Error> connect(
            std::string_view robotIP = config::ROBOT_IP,
            uint16_t robotPort = config::ROBOT_PORT);
        void disconnect();
        bool isConnected() const { return m_isConnected.load(std::memory_order_acquire); }
        
    public:
        struct TransactionResult
        {
            RobotState state;
            std::string response;
        };

        std::expected<void, Error> sendAndReceive(std::string_view message);
        std::expected<TransactionResult, Error> sendAndReceiveDetailed(
            std::string_view message);
        RobotState lastState() const;

    private:
        bool waitPeerID();
    };
}

#endif // !I2R_RLC_ROBOT_COMM_CLIENT_H
