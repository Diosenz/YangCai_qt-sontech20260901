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
 * @file        RobotProtocolCodec.h
 * @brief       机器人控制柜旧协议编码器
 * @details     按旧 RobotCtrl.cpp 的字段顺序生成 N、O、P 请求字符串。
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
#ifndef I2R_RLC_ROBOT_COMM_ROBOT_PROTOCOL_CODEC_H
#define I2R_RLC_ROBOT_COMM_ROBOT_PROTOCOL_CODEC_H

#include <string_view>
#include <string>

#include "CalibrationCommand.h"
#include "JobCommand.h"
#include "OnlineCorrectionCommand.h"
#include "RobotState.h"

namespace i2r_rlc::robot_comm {

/**
 * @brief 机器人控制柜旧字符串协议编码器。
 */
class CRobotProtocolCodec final {
public:
    /// @brief 生成旧 N 状态轮询字符串。
    static std::string encodeStatusPoll(const RobotProtocolContext &context);

    /**
     * @brief 生成在线作业启动前的一次 N 初始化请求。
     * N 的第九字段是控制柜 OnlineCor_initflag；置 1 请求清除 B005。
     * 与 P 的 OnlineCor_endflag 语义不同，不改变共享上下文或普通轮询。
     */
    static std::string encodeOnlineCorrectionInitialization(
        const RobotProtocolContext &context);

    /// @brief 生成单个示教点的旧 O 更新字符串。
    static std::string encodeTeachPointUpdate(
        const RobotProtocolContext &context,
        int pointCount,
        const TeachPointUpdateCommand &teachPoint,
        const StationAxisUpdateCommand &stationAxis);

    /// @brief 生成旧 P 实时纠偏字符串。
    static std::string encodeRealtimeCorrection(
        const RobotProtocolContext &context,
        const RobotRealtimeCorrectionCommand &command);

    /// @brief 生成旧 C 自动标定“进入下一位置”字符串。
    static std::string encodeCalibrationAdvance(
        const RobotProtocolContext &context);

    /**
     * @brief 生成旧 J 作业启动字符串。
     * @param jobName 控制柜中的 JBI 作业名，不得包含空白字符或逗号。
     * @return 参数无效时返回空字符串。
     */
    static std::string encodeJobStart(
        const RobotProtocolContext &context,
        std::string_view jobName);

    /**
     * @brief 解析控制柜完整 D 状态回包。
     * @param response 完整回包字符串，允许末尾带空白字符。
     * @param state 输出机器人状态；解析失败时不修改。
     * @return true 表示命令字和 32 个字段均完整有效。
     */
    static bool decodeRobotState(
        const std::string &response,
        RobotState *state);
};

} // namespace i2r_rlc::robot_comm

#endif // I2R_RLC_ROBOT_COMM_ROBOT_PROTOCOL_CODEC_H
