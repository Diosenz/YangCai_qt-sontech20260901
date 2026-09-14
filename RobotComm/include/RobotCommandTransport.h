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
 * @file        RobotCommandTransport.h
 * @brief       机器人命令传输抽象
 * @details     隔离真实 ZeroMQ 客户端与自动测试使用的替代传输。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifndef I2R_RLC_ROBOT_COMM_ROBOT_COMMAND_TRANSPORT_H
#define I2R_RLC_ROBOT_COMM_ROBOT_COMMAND_TRANSPORT_H

#include <cstdint>
#include <string_view>
#include <string>

#include "OnlineCorrectionCommand.h"
#include "RobotState.h"

namespace i2r_rlc::robot_comm {

/**
 * @brief 机器人命令传输接口。
 */
class IRobotCommandTransport {
public:
    virtual ~IRobotCommandTransport() = default;

    /// @brief 建立控制柜连接；只在 RobotComm 线程调用。
    virtual bool connect(std::string_view robotIp, std::uint16_t robotPort) = 0;
    /// @brief 断开控制柜连接；必须可重复调用。
    virtual void disconnect() = 0;
    /// @brief 查询当前连接状态。
    virtual bool isConnected() const = 0;
    /**
     * @brief 串行发送一条请求并读取对应 D 回包。
     * @param state 成功时写入最新完整状态，可为空。
     */
    virtual RobotCommandReceipt transact(
        RobotCommandType type,
        const std::string &request,
        int pointIndex,
        RobotState *state) = 0;
};

/**
 * @brief 基于 RobotComm Client 的真实传输实现。
 */
class CClientRobotCommandTransport final : public IRobotCommandTransport {
public:
    /// @brief 使用现有 Client 单例建立真实连接。
    bool connect(std::string_view robotIp, std::uint16_t robotPort) override;
    /// @brief 断开现有 Client 连接。
    void disconnect() override;
    /// @brief 查询现有 Client 连接状态。
    bool isConnected() const override;
    /// @brief 通过现有 Client 完成请求/D 回包事务。
    RobotCommandReceipt transact(
        RobotCommandType type,
        const std::string &request,
        int pointIndex,
        RobotState *state) override;
};

} // namespace i2r_rlc::robot_comm

#endif // I2R_RLC_ROBOT_COMM_ROBOT_COMMAND_TRANSPORT_H
