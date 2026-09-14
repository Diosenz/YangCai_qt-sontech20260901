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
 * @file        TrackingTriggerPolicy.h
 * @brief       在线跟踪触发策略
 * @details     将旧 OpencvThread 中依赖机器人状态和虚拟状态的检测触发条件收口为纯 C++ 判断。
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
#ifndef I2R_RLC_CONTROL_TRACKING_TRIGGER_POLICY_H
#define I2R_RLC_CONTROL_TRACKING_TRIGGER_POLICY_H

#include <string>

#include "RobotState.h"

namespace i2r_rlc::control {

/**
 * @brief 旧虚拟场景状态中与在线识别相关的最小集合。
 */
enum class TrackingVirtualStatus {
    None = 0,
    OnlineRecordWithLocalization = 1,
    OnlineRecord = 2
};

/**
 * @brief 在线跟踪触发请求。
 */
struct TrackingTriggerInput {
    /// @brief 旧 testStart 标志，1 表示测试线程占用检测入口。
    int testStart = 0;
    /// @brief 当前机器人状态快照。
    robot_comm::RobotState robotState;
    /// @brief 旧虚拟状态。
    TrackingVirtualStatus virtualStatus = TrackingVirtualStatus::None;
};

/**
 * @brief 在线跟踪触发结果。
 */
struct TrackingTriggerDecision {
    /// @brief true 表示应向视觉线程投递一次跟踪检测任务。
    bool shouldStartTracking = false;
    /// @brief 稳定英文 key，用于日志和测试断言。
    std::string reasonKey = "not_evaluated";
    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 复刻旧 OpencvThread 的检测启动条件。
 */
class CTrackingTriggerPolicy final {
public:
    /**
     * @brief 判断当前帧是否满足在线跟踪检测触发条件。
     * @param input 机器人状态、测试标志和虚拟状态。
     * @return 触发判断结果。
     */
    static TrackingTriggerDecision evaluate(const TrackingTriggerInput &input);
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_TRACKING_TRIGGER_POLICY_H
