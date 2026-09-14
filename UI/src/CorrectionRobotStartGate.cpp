/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CorrectionRobotStartGate.cpp
 * @brief       纠偏启动机器人状态门禁实现
 ******************************************************************************/
#include "CorrectionRobotStartGate.h"

namespace i2r_rlc::ui {

namespace {

CorrectionRobotStartGateDecision reject(
    const std::string &reasonKey,
    const std::string &message)
{
    CorrectionRobotStartGateDecision decision;
    decision.ready = false;
    decision.reasonKey = reasonKey;
    decision.message = message;
    return decision;
}

} // namespace

CorrectionRobotStartGateDecision CCorrectionRobotStartGate::evaluate(
    bool validDReceived,
    const robot_comm::RobotState &state)
{
    if (!validDReceived) {
        return reject(
            "robot_state_unavailable",
            "valid D response has not been received");
    }
    if (state.sStart != 0) {
        return reject("robot_running", "sStart is not 0");
    }
    if (state.sMode != 2) {
        return reject("mode_not_play", "sMode is not 2");
    }
    if (state.sServoPower != 1) {
        return reject("servo_not_on", "sServoPower is not 1");
    }

    CorrectionRobotStartGateDecision decision;
    decision.ready = true;
    decision.reasonKey = "ready";
    decision.message = "robot is ready to start correction job";
    return decision;
}

CorrectionRobotStartGateDecision CCorrectionRobotStartGate::evaluateRobotSim(
    bool formalCorrection,
    const CorrectionRobotSimStartState &state)
{
    if (!formalCorrection) {
        CorrectionRobotStartGateDecision decision;
        decision.ready = true;
        decision.reasonKey = "not_required";
        decision.message = "this workflow does not require correction frame synchronization";
        return decision;
    }
    if (state.manualModeRequested
        || state.manualConnectionPending
        || state.manualConnected) {
        return reject(
            "robot_sim_manual_mode",
            "RobotSim manual mode cannot provide real-state frame synchronization");
    }
    if (state.realSceneConnectionPending) {
        return reject(
            "robot_sim_real_scene_pending",
            "RobotSim real-scene connection is still pending");
    }
    if (!state.realSceneConnected) {
        return reject(
            "robot_sim_real_scene_disconnected",
            "RobotSim real-scene synchronization is not connected");
    }

    CorrectionRobotStartGateDecision decision;
    decision.ready = true;
    decision.reasonKey = "ready";
    decision.message = "RobotSim real-scene synchronization is ready";
    return decision;
}

} // namespace i2r_rlc::ui
