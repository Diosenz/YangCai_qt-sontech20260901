/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_tracking_trigger_policy.cpp
 * @brief       tracking trigger policy 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include "TrackingTriggerPolicy.h"

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
    state.traflag = 1;
    state.sMode = 2;
    state.sHold = 0;
    state.sStart = 1;
    return state;
}

} // namespace

int main()
{
    using i2r_rlc::control::CTrackingTriggerPolicy;
    using i2r_rlc::control::TrackingTriggerInput;
    using i2r_rlc::control::TrackingVirtualStatus;

    bool ok = true;

    TrackingTriggerInput input;
    input.robotState = readyRobotState();
    input.virtualStatus = TrackingVirtualStatus::OnlineRecord;
    ok &= expectTrue(
        CTrackingTriggerPolicy::evaluate(input).shouldStartTracking,
        "ready state should trigger tracking");

    input.testStart = 1;
    ok &= expectTrue(
        !CTrackingTriggerPolicy::evaluate(input).shouldStartTracking,
        "testStart should block tracking");
    input.testStart = 0;

    input.robotState.sHold = 1;
    ok &= expectTrue(
        !CTrackingTriggerPolicy::evaluate(input).shouldStartTracking,
        "sHold should block tracking");
    input.robotState = readyRobotState();

    input.virtualStatus = TrackingVirtualStatus::None;
    ok &= expectTrue(
        !CTrackingTriggerPolicy::evaluate(input).shouldStartTracking,
        "virtual status none should block tracking");

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: tracking trigger policy tests\n";
    return 0;
}
