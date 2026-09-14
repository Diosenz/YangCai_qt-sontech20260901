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
 * @file        RobotStateSnap.h
 * @brief       机器人状态快照缓存。
 * @details     保存最近一次从控制柜接收的状态帧，供其他模块读取。
 * 
 * @version     1.1
 * @author      I2R Laboratory
 * @date        2026 年 6 月 15 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.1      <td>I2R Lab     <td>260615      <td>合并 RobotState 数据结构</tr>
 * </table>
 * 
 ******************************************************************************/

#pragma once
#ifndef I2R_RLC_ROBOT_COMM_ROBOT_STATE_SNAP_H
#define I2R_RLC_ROBOT_COMM_ROBOT_STATE_SNAP_H

#include <mutex>

#include <shared_mutex>

#include "RobotState.h"

namespace i2r_rlc::robot_comm
{
    /**
     * @brief 机器人状态快照缓存。
     *
     * 通信线程收到控制柜状态帧后写入本对象，其他模块可读取最近一次状态。
     * 本类只保存快照，不负责连接、收包或运动控制。
     */
    class RobotStateSnap final
    {
    private:
        mutable std::shared_mutex m_mutex;
        RobotState m_state;

    private:
        explicit RobotStateSnap() = default;
        ~RobotStateSnap() = default;

    public:
        RobotStateSnap(const RobotStateSnap&) = delete;
        RobotStateSnap(RobotStateSnap&&) = delete;
        RobotStateSnap& operator=(const RobotStateSnap&) = delete;
        RobotStateSnap& operator=(RobotStateSnap&&) = delete;

        static RobotStateSnap& instance()
        {
            static RobotStateSnap s_instance;
            return s_instance;
        }

        void updateState(const RobotState& state)
        {
            std::unique_lock lock(m_mutex);
            m_state = state;
        }

        RobotState getState() const
        {
            std::shared_lock lock(m_mutex);
            return m_state;
        }
    };
}

#endif // !I2R_RLC_ROBOT_COMM_ROBOT_STATE_SNAP_H
