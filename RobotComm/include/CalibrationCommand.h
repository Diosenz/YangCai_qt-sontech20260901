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
 * @file        CalibrationCommand.h
 * @brief       标定采集机器人命令数据结构
 * @details     定义标定采集流程中由 Control 生成、RobotComm 后续发送的握手命令。
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
#ifndef I2R_RLC_ROBOT_COMM_CALIBRATION_COMMAND_H
#define I2R_RLC_ROBOT_COMM_CALIBRATION_COMMAND_H

#include <string>

namespace i2r_rlc::robot_comm {

/**
 * @brief 标定采集握手命令。
 *
 * 该结构承接旧工程 `s_Calibreset = 1` 的语义：PC 保存当前标定样本后，
 * 通知机器人进入下一标定位。RobotComm 将其编码为旧 C 请求，并读取对应 D 回包。
 */
struct RobotCalibrationCommand {
    /// @brief true 表示需要请求机器人进入下一标定位。
    bool requestNextCalibrationPosition = false;

    /// @brief 旧 `s_Calibreset` 语义，1 表示当前点采集完成。
    int calibResetSignal = 0;

    /// @brief 当前已完成的样本数量，单位为组。
    int capturedSampleCount = 0;

    /// @brief true 表示命令内容完整。
    bool valid = false;

    /// @brief 状态说明，用于 dry-run 日志或测试。
    std::string message;
};

} // namespace i2r_rlc::robot_comm

#endif // I2R_RLC_ROBOT_COMM_CALIBRATION_COMMAND_H
