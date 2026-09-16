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
 * @file        TrackingTriggerPolicy.cpp
 * @brief       在线跟踪触发策略实现
 * @details     以显式状态快照替代旧 OpencvThread 中直接读取全局变量的触发判断。
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
#include "TrackingTriggerPolicy.h"

namespace i2r_rlc::control {

namespace {

TrackingTriggerDecision reject(const std::string &key, const std::string &message)
{
    TrackingTriggerDecision decision;
    decision.shouldStartTracking = false;
    decision.reasonKey = key;
    decision.message = message;
    return decision;
}

} // namespace

TrackingTriggerDecision CTrackingTriggerPolicy::evaluate(const TrackingTriggerInput &input)
{
    const robot_comm::RobotState &state = input.robotState;
    if (input.testStart == 1) {
        return reject("test_start_active", "testStart is active");
    }
    if (input.requireDetectionWindow && state.traflag != 1) {
        return reject("traflag_not_ready", "traflag is not 1");
    }
    if (state.sMode != 2) {
        return reject("mode_not_play", "sMode is not 2");
    }
    if (state.sHold != 0) {
        return reject("robot_hold", "sHold is not 0");
    }
    if (state.sStart != 1) {
        return reject("program_not_started", "sStart is not 1");
    }
    if (input.virtualStatus != TrackingVirtualStatus::OnlineRecordWithLocalization
        && input.virtualStatus != TrackingVirtualStatus::OnlineRecord) {
        return reject("virtual_status_not_online_record", "virtual status does not request online record");
    }

    TrackingTriggerDecision decision;
    decision.shouldStartTracking = true;
    decision.reasonKey = "ready";
    decision.message = "tracking trigger accepted";
    return decision;
}

} // namespace i2r_rlc::control
