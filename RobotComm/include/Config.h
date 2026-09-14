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
 * @file        Config.h
 * @brief       RobotComm 默认通信配置。
 * @details     定义控制柜默认端点和 ZeroMQ 超时等编译期常量。
 * 
 * @version     1.1
 * @author      I2R Laboratory
 * @date        2026 年 6 月 15 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.1      <td>I2R Lab     <td>260615      <td>同步消息队列超时命名</tr>
 * </table>
 * 
 ******************************************************************************/

#pragma once
#ifndef I2R_RLC_ROBOT_COMM_CONFIG_H
#define I2R_RLC_ROBOT_COMM_CONFIG_H

#include <stdint.h>

namespace i2r_rlc::robot_comm::config
{
    inline constexpr int CLIENT_QUEUE_TIMEOUT = 100;    ///< ClientQueue 消息队列阻塞时间 (ms)
    inline constexpr auto ROBOT_IP = "192.168.255.1";   ///< 机器人控制柜默认 IP。
    inline constexpr uint16_t ROBOT_PORT = 11000;       ///< 机器人控制柜默认通信端口。
    inline constexpr int SOCKET_CONNECT_TIMEOUT = 1000; ///< ZeroMQ 发送/接收超时时间 (ms)。
}

#endif // !I2R_RLC_ROBOT_COMM_CONFIG_H
