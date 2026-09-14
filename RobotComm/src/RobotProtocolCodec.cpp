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
 * @file        RobotProtocolCodec.cpp
 * @brief       机器人控制柜旧协议编码实现
 * @details     使用十进制整数和固定逗号顺序生成下位机可解析的请求。
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
#include <algorithm>
#include <cstdio>

#include <cctype>
#include <fmt/format.h>

#include "RobotProtocolCodec.h"

namespace i2r_rlc::robot_comm {

std::string CRobotProtocolCodec::encodeStatusPoll(const RobotProtocolContext &context)
{
    return fmt::format(
        "N {},{},{},{},{},{},{},{},{}",
        context.robotEnabled,
        context.trackerEnabled,
        context.laserEnabled,
        context.filterEnabled,
        context.offlineSignal,
        context.offlineStartCount,
        context.holdRequested,
        context.firstDetectionSucceeded,
        context.onlineEndFlag);
}

std::string CRobotProtocolCodec::encodeTeachPointUpdate(
    const RobotProtocolContext &context,
    int pointCount,
    const TeachPointUpdateCommand &teachPoint,
    const StationAxisUpdateCommand &stationAxis)
{
    return fmt::format(
        "O {},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        context.robotEnabled,
        context.trackerEnabled,
        context.laserEnabled,
        context.filterEnabled,
        context.offlineSignal,
        context.offlineStartCount,
        context.holdRequested,
        context.firstDetectionSucceeded,
        pointCount,
        teachPoint.xUm,
        teachPoint.yUm,
        teachPoint.zUm,
        teachPoint.rxDeg10000,
        teachPoint.ryDeg10000,
        teachPoint.rzDeg10000,
        stationAxis.s1Deg10000,
        stationAxis.s2Deg10000);
}

std::string CRobotProtocolCodec::encodeRealtimeCorrection(
    const RobotProtocolContext &context,
    const RobotRealtimeCorrectionCommand &command)
{
    return fmt::format(
        "P {},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        context.robotEnabled,
        context.trackerEnabled,
        context.laserEnabled,
        context.filterEnabled,
        context.offlineSignal,
        context.offlineStartCount,
        context.holdRequested,
        context.firstDetectionSucceeded,
        context.onlineEndFlag,
        command.dxUm,
        command.dyUm,
        command.dzUm,
        command.drxDeg10000,
        command.dryDeg10000,
        command.drzDeg10000,
        command.speedOverride,
        command.speedMode);
}

std::string CRobotProtocolCodec::encodeCalibrationAdvance(
    const RobotProtocolContext &context)
{
    return fmt::format(
        "C {},{},{},{},{},{},{},{}",
        context.robotEnabled,
        context.trackerEnabled,
        context.laserEnabled,
        context.filterEnabled,
        context.offlineSignal,
        context.offlineStartCount,
        context.holdRequested,
        context.firstDetectionSucceeded);
}

std::string CRobotProtocolCodec::encodeJobStart(
    const RobotProtocolContext &context,
    std::string_view jobName)
{
    if (jobName.empty()
        || std::any_of(
            jobName.begin(),
            jobName.end(),
            [](char character) {
                return character == ','
                    || std::isspace(static_cast<unsigned char>(character)) != 0;
            })) {
        return {};
    }

    return fmt::format(
        "J {},{},{},{},{},{},{},{},{}",
        context.robotEnabled,
        context.trackerEnabled,
        context.laserEnabled,
        context.filterEnabled,
        context.offlineSignal,
        context.offlineStartCount,
        context.holdRequested,
        context.firstDetectionSucceeded,
        jobName);
}

bool CRobotProtocolCodec::decodeRobotState(
    const std::string &response,
    RobotState *state)
{
    if (state == nullptr) {
        return false;
    }

    RobotState parsed;
    int consumed = 0;
    const int fieldCount = std::sscanf(
        response.c_str(),
        "D %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,"
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d%n",
        &parsed.comnum,
        &parsed.sMode,
        &parsed.sRemote,
        &parsed.sServoPower,
        &parsed.sStart,
        &parsed.sHold,
        &parsed.sIsAlarm,
        &parsed.usJobLine,
        &parsed.usStep,
        &parsed.traflag,
        &parsed.corflag,
        &parsed.offline,
        &parsed.detflag,
        &parsed.s_BuffL,
        &parsed.s_Bcalib,
        &parsed.s_Bsavetraj,
        &parsed.OfflineRun,
        &parsed.curx_r,
        &parsed.cury_r,
        &parsed.curz_r,
        &parsed.curRx_r,
        &parsed.curRy_r,
        &parsed.curRz_r,
        &parsed.curS1,
        &parsed.curS2,
        &parsed.cur_interpolation,
        &parsed.command_speed_x,
        &parsed.command_speed_y,
        &parsed.command_speed_z,
        &parsed.CreateJOB_status,
        &parsed.posflag,
        &parsed.onlineReverseCompleteFlag,
        &consumed);
    if (fieldCount != 32 || consumed <= 0) {
        return false;
    }

    for (std::size_t index = static_cast<std::size_t>(consumed);
         index < response.size();
         ++index) {
        if (std::isspace(static_cast<unsigned char>(response[index])) == 0) {
            return false;
        }
    }

    // 与 SWv1 RobotCtrl::RevRobotData 保持一致：控制器回传的 posflag 在
    // 部分固件上可能不是严格的 0/1，任何非零值都表示 STARTPOSSEG 有效。
    parsed.posflag = parsed.posflag == 0 ? 0 : 1;

    *state = parsed;
    return true;
}

} // namespace i2r_rlc::robot_comm
