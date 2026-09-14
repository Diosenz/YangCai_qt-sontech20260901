/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_correction_robot_start_gate.cpp
 * @brief       纠偏启动机器人状态门禁测试入口
 * @details     验证在线/离线纠偏发送 JBI 前的旧工程状态检查语义。
 ******************************************************************************/
#include <iostream>

#include "CorrectionRobotJobMonitor.h"
#include "CorrectionRobotStartGate.h"

namespace {

bool expectTrue(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

i2r_rlc::robot_comm::RobotState readyRobotState()
{
    i2r_rlc::robot_comm::RobotState state;
    state.sStart = 0;
    state.sMode = 2;
    state.sServoPower = 1;
    return state;
}

} // namespace

int main()
{
    using i2r_rlc::ui::CCorrectionRobotJobMonitor;
    using i2r_rlc::ui::CCorrectionRobotStartGate;

    bool ok = true;

    i2r_rlc::robot_comm::RobotState state = readyRobotState();
    ok &= expectTrue(
        !CCorrectionRobotStartGate::evaluate(false, state).ready,
        "missing valid D response should block correction start");

    state = readyRobotState();
    state.sMode = 1;
    ok &= expectTrue(
        !CCorrectionRobotStartGate::evaluate(true, state).ready,
        "non-PLAY mode should block correction start");

    state = readyRobotState();
    state.sServoPower = 0;
    ok &= expectTrue(
        !CCorrectionRobotStartGate::evaluate(true, state).ready,
        "servo off should block correction start");

    state = readyRobotState();
    state.sStart = 1;
    ok &= expectTrue(
        !CCorrectionRobotStartGate::evaluate(true, state).ready,
        "running robot job should block correction start");

    state = readyRobotState();
    ok &= expectTrue(
        CCorrectionRobotStartGate::evaluate(true, state).ready,
        "stopped PLAY robot with servo on should allow correction start");

    i2r_rlc::ui::CorrectionRobotSimStartState robotSimState;
    ok &= expectTrue(
        !CCorrectionRobotStartGate::evaluateRobotSim(true, robotSimState).ready,
        "formal correction should reject disconnected real-scene RobotSim");
    robotSimState.realSceneConnectionPending = true;
    ok &= expectTrue(
        CCorrectionRobotStartGate::evaluateRobotSim(true, robotSimState)
                .reasonKey == "robot_sim_real_scene_pending",
        "formal correction should distinguish a pending real-scene connection");
    robotSimState.realSceneConnectionPending = false;
    robotSimState.realSceneConnected = true;
    robotSimState.manualModeRequested = true;
    ok &= expectTrue(
        !CCorrectionRobotStartGate::evaluateRobotSim(true, robotSimState).ready,
        "manual RobotSim mode should reject formal correction even if stale real state is set");
    robotSimState.manualModeRequested = false;
    ok &= expectTrue(
        CCorrectionRobotStartGate::evaluateRobotSim(true, robotSimState).ready,
        "connected real-scene synchronization should allow formal correction");
    robotSimState.realSceneConnected = false;
    robotSimState.manualConnected = true;
    ok &= expectTrue(
        CCorrectionRobotStartGate::evaluateRobotSim(false, robotSimState).ready,
        "standalone global localization must retain its existing independent gate");

    state = readyRobotState();
    state.sHold = 1;
    state.sIsAlarm = 1;
    state.sRemote = 0;
    ok &= expectTrue(
        CCorrectionRobotStartGate::evaluate(true, state).ready,
        "hold, alarm and remote fields are outside the legacy start gate");

    CCorrectionRobotJobMonitor monitor;
    state = readyRobotState();
    monitor.arm();
    ok &= expectTrue(
        !monitor.update(state),
        "job monitor should not complete before seeing sStart=1");
    state.sStart = 1;
    ok &= expectTrue(
        !monitor.update(state),
        "job monitor should only arm completion after seeing the job running");
    state.sStart = 0;
    ok &= expectTrue(
        monitor.update(state),
        "job monitor should complete on sStart 1->0 edge");
    ok &= expectTrue(
        !monitor.armed(),
        "job monitor should reset after completion");

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: correction robot start gate tests\n";
    return 0;
}
