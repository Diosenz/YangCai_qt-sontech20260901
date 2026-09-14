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
 * @file        RobotCommandTransport.cpp
 * @brief       机器人命令真实传输实现
 * @details     调用现有 ZeroMQ Client 完成请求和 D 状态回包处理。
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
#include "Client.h"
#include "RobotCommandTransport.h"

namespace i2r_rlc::robot_comm {
namespace {

RobotCommandError toCommandError(Client::Error error)
{
    switch (error) {
    case Client::Error::SOCKET_NOT_CONNECTED:
        return RobotCommandError::NotConnected;
    case Client::Error::SOCKET_CONNECT_FAILED:
    case Client::Error::SOCKET_CREATE_FAILED:
        return RobotCommandError::ConnectFailed;
    case Client::Error::SEND_FAILED:
        return RobotCommandError::SendFailed;
    case Client::Error::RECV_FAILED:
        return RobotCommandError::ReceiveFailed;
    case Client::Error::RESPONSE_EMPTY:
        return RobotCommandError::EmptyResponse;
    case Client::Error::RESPONSE_FORMAT_ERROR:
        return RobotCommandError::InvalidResponse;
    default:
        return RobotCommandError::TransportError;
    }
}

std::string commandErrorText(Client::Error error)
{
    switch (error) {
    case Client::Error::SOCKET_NOT_CONNECTED:
        return "socket is not connected";
    case Client::Error::SOCKET_CONNECT_FAILED:
        return "socket connect failed";
    case Client::Error::SOCKET_CREATE_FAILED:
        return "socket create failed";
    case Client::Error::SEND_FAILED:
        return "send failed";
    case Client::Error::RECV_FAILED:
        return "receive timeout or failed";
    case Client::Error::RESPONSE_EMPTY:
        return "D response is empty";
    case Client::Error::RESPONSE_FORMAT_ERROR:
        return "D response format is invalid";
    case Client::Error::ZMQ_ERROR:
        return "ZeroMQ transport error";
    default:
        return "unknown robot transport error";
    }
}

} // namespace

bool CClientRobotCommandTransport::connect(
    std::string_view robotIp,
    std::uint16_t robotPort)
{
    return Client::instance().connect(robotIp, robotPort).has_value();
}

void CClientRobotCommandTransport::disconnect()
{
    Client::instance().disconnect();
}

bool CClientRobotCommandTransport::isConnected() const
{
    return Client::instance().isConnected();
}

RobotCommandReceipt CClientRobotCommandTransport::transact(
    RobotCommandType type,
    const std::string &request,
    int pointIndex,
    RobotState *state)
{
    RobotCommandReceipt receipt;
    receipt.type = type;
    receipt.pointIndex = pointIndex;
    receipt.request = request;

    const auto result = Client::instance().sendAndReceiveDetailed(request);
    receipt.sent = result.has_value();
    receipt.responseReceived = result.has_value();
    if (!result) {
        receipt.error = toCommandError(result.error());
        receipt.message = commandErrorText(result.error());
        return receipt;
    }

    receipt.response = result->response;
    if (state != nullptr) {
        *state = result->state;
        receipt.completed =
            type != RobotCommandType::TeachPointUpdate || state->offline == 5;
    } else {
        receipt.completed = type != RobotCommandType::TeachPointUpdate;
    }
    receipt.message = "robot request completed";
    return receipt;
}

} // namespace i2r_rlc::robot_comm
