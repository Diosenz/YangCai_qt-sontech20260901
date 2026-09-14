/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CorrectionRobotStartGate.h
 * @brief       纠偏启动机器人状态门禁
 * @details     复刻旧工程启动 JBI 前的 sStart/sMode/sServoPower 状态检查。
 ******************************************************************************/
#ifndef I2R_RLC_UI_CORRECTION_ROBOT_START_GATE_H
#define I2R_RLC_UI_CORRECTION_ROBOT_START_GATE_H

#include <string>

#include "RobotState.h"

namespace i2r_rlc::ui {

/**
 * @brief 纠偏启动门禁判定结果。
 *
 * `ready` 为 true 时，UI 才允许向 RobotComm 发送 J 命令。`reasonKey`
 * 用于测试和日志定位，`message` 面向工程诊断。
 */
struct CorrectionRobotStartGateDecision {
    bool ready = false;
    std::string reasonKey;
    std::string message;
};

/**
 * @brief 正式纠偏启动前的 RobotSim 连接模式快照。
 *
 * 该值对象只描述 UI 已确认的连接事实，不主动连接设备。真实场景连接中、
 * 未连接或手动仿真模式都不能满足在线/离线纠偏的逐帧同步前置条件。
 */
struct CorrectionRobotSimStartState {
    bool realSceneConnected = false;
    bool realSceneConnectionPending = false;
    bool manualModeRequested = false;
    bool manualConnectionPending = false;
    bool manualConnected = false;
};

/**
 * @brief 判断在线/离线纠偏启动前机器人是否满足旧工程前置状态。
 */
class CCorrectionRobotStartGate final {
public:
    static CorrectionRobotStartGateDecision evaluate(
        bool validDReceived,
        const robot_comm::RobotState &state);

    /**
     * @brief 检查正式纠偏是否已连接真实状态同步模式。
     *
     * `formalCorrection` 为 false 时保持独立全局定位等既有入口不变；只有
     * 在线/离线 JBI 启动才要求真实场景已连接且不存在手动/连接中状态。
     */
    static CorrectionRobotStartGateDecision evaluateRobotSim(
        bool formalCorrection,
        const CorrectionRobotSimStartState &state);

private:
    CCorrectionRobotStartGate() = delete;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_CORRECTION_ROBOT_START_GATE_H
