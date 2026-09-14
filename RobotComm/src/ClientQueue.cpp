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
 * @file        ClientQueue.cpp
 * @brief       机器人控制消息发送队列实现。
 * @details     后台线程消费字符串消息，并调用 Client 完成同步发送和状态接收。
 * 
 * @version     1.1
 * @author      I2R Laboratory
 * @date        2026 年 6 月 15 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.1      <td>I2R Lab     <td>260615      <td>改为直接消费字符串消息</tr>
 * </table>
 * 
 ******************************************************************************/

#include <chrono>

#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>

#include "Client.h"
#include "ClientQueue.h"
#include "Config.h"

using namespace i2r_rlc::robot_comm;

ClientQueue::ClientQueue()
{
    m_isRunning.store(true, std::memory_order_release);
    m_thread = std::thread(&ClientQueue::run, this);
}

ClientQueue::~ClientQueue()
{
    m_isRunning.store(false, std::memory_order_release);
    if (m_thread.joinable())    m_thread.join();
}

void ClientQueue::run()
{
    moodycamel::ConsumerToken consumerToken(m_queueMessage);

    while (m_isRunning.load(std::memory_order_acquire))
    {
        if (std::string message;
            m_queueMessage.wait_dequeue_timed(consumerToken, message,
                std::chrono::milliseconds(config::CLIENT_QUEUE_TIMEOUT)) && !message.empty())
        {
            if (auto result = Client::instance().sendAndReceive(message))
            {
                fmt::print("Client::instance().sendAndReceive() SUCCESS\n");
            }
            else
            {
                fmt::print("Client::instance().sendAndReceive() ERROR: {}\n",
                    magic_enum::enum_name(result.error()));
            }
        }
    }
}
