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
 * @file        ClientQueue.h
 * @brief       机器人控制消息发送队列。
 * @details     队列后台线程串行消费字符串消息，并委托 Client 发送到控制柜。
 * 
 * @version     1.1
 * @author      I2R Laboratory
 * @date        2026 年 6 月 15 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.1      <td>I2R Lab     <td>260615      <td>改为直接排队字符串消息</tr>
 * </table>
 * 
 ******************************************************************************/

#pragma once
#ifndef I2R_RLC_ROBOT_COMM_CLIENT_QUEUE_H
#define I2R_RLC_ROBOT_COMM_CLIENT_QUEUE_H

#include <atomic>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include <concepts>
#include <concurrentqueue/moodycamel/blockingconcurrentqueue.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <iterator>
#include <ranges>

namespace i2r_rlc::robot_comm
{
    /**
     * @brief 机器人发送消息队列。
     *
     * 队列后台线程串行消费字符串命令，并通过 Client 发送到机器人控制柜。
     * 新版通信模块不再维护 Notif 派生类型，调用方需要在入队前明确消息类型字母和参数。
     */
    class ClientQueue final
    {
    private:
        moodycamel::BlockingConcurrentQueue<std::string> m_queueMessage;
        std::thread m_thread;                           ///< 操作器线程
        std::atomic<bool> m_isRunning = false;          ///< 线程运行标志

    private:
        explicit ClientQueue();
        ~ClientQueue();

    public:
        ClientQueue(const ClientQueue&) = delete;
        ClientQueue(ClientQueue&&) = delete;
        ClientQueue& operator=(const ClientQueue&) = delete;
        ClientQueue& operator=(ClientQueue&&) = delete;

        static ClientQueue& instance()
        {
            static ClientQueue s_instance;
            return s_instance;
        }

        void enqueueMessage(const std::string& message) { m_queueMessage.enqueue(message); }

        /**
         * @brief 构造单条控制柜消息并入队。
         *
         * 消息格式由控制柜协议约定：首字符表示消息类型，参数使用逗号连接。
         * @note        使用例：
         *  ClientQueue::instance().enqueueMessage('N', 42, 3.14, 'a', "asdfjkl");
         */
        template<typename... Args>
        void enqueueMessage(char ch, Args&&... args)
        {
            enqueueMessage(fmt::format("{} {}",
                ch,
                fmt::join(std::forward_as_tuple(std::forward<Args>(args)...), ",")));
        }

        /**
         * @brief 消息原子批量入队。
         */
        template<typename MessageContainer>
        requires std::ranges::sized_range<MessageContainer> &&
            std::same_as<std::ranges::range_value_t<MessageContainer>, std::string>
        void enqueueBulkMessage(MessageContainer&& messages)
        {
            m_queueMessage.enqueue_bulk(
                std::make_move_iterator(messages.begin()),
                messages.size());
        }

    private:
        void run();
    };
}

#endif // !I2R_RLC_ROBOT_COMM_CLIENT_QUEUE_H
