/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_robot_protocol_codec.cpp
 * @brief       robot protocol codec 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>
#include <string>

#include "RobotProtocolCodec.h"

namespace {

bool expectEqual(
    const std::string &actual,
    const std::string &expected,
    const char *message)
{
    if (actual != expected) {
        std::cerr << "FAIL: " << message << "\nactual:   " << actual
                  << "\nexpected: " << expected << '\n';
        return false;
    }
    return true;
}

} // namespace

int main()
{
    using namespace i2r_rlc::robot_comm;

    RobotProtocolContext context;
    context.robotEnabled = 1;
    context.trackerEnabled = 2;
    context.laserEnabled = 3;
    context.filterEnabled = 4;
    context.offlineSignal = 5;
    context.offlineStartCount = 6;
    context.holdRequested = 7;
    context.firstDetectionSucceeded = 8;
    context.onlineEndFlag = 9;

    bool ok = true;
    ok &= expectEqual(
        CRobotProtocolCodec::encodeStatusPoll(context),
        "N 1,2,3,4,5,6,7,8,9",
        "N protocol must match legacy format");
    ok &= expectEqual(
        CRobotProtocolCodec::encodeStatusPoll(RobotProtocolContext{}),
        "N 0,0,0,0,0,0,0,0,0",
        "N protocol must preserve zero fields");
    ok &= expectEqual(
        CRobotProtocolCodec::encodeOnlineCorrectionInitialization(context),
        "N 1,2,3,4,5,6,7,8,1",
        "online initialization must use N.init rather than P.end");
    ok &= expectEqual(
        std::to_string(context.onlineEndFlag),
        "9",
        "online initialization must not mutate the shared P context");

    TeachPointUpdateCommand point{
        -10, 20, -30, 40, -50, 60, true
    };
    StationAxisUpdateCommand axis{ -70, 80, true };
    ok &= expectEqual(
        CRobotProtocolCodec::encodeTeachPointUpdate(context, 11, point, axis),
        "O 1,2,3,4,5,6,7,8,11,-10,20,-30,40,-50,60,-70,80",
        "O protocol must match legacy format");
    point.rxDeg10000 = 0;
    ok &= expectEqual(
        CRobotProtocolCodec::encodeTeachPointUpdate(context, 11, point, axis),
        "O 1,2,3,4,5,6,7,8,11,-10,20,-30,0,-50,60,-70,80",
        "zero RX must remain an explicit target value on the wire");

    RobotRealtimeCorrectionCommand realtime;
    realtime.dxUm = -101;
    realtime.dyUm = 102;
    realtime.dzUm = -103;
    realtime.drxDeg10000 = 104;
    realtime.dryDeg10000 = -105;
    realtime.drzDeg10000 = 106;
    realtime.speedOverride = 7000;
    realtime.speedMode = 1;
    ok &= expectEqual(
        CRobotProtocolCodec::encodeRealtimeCorrection(context, realtime),
        "P 1,2,3,4,5,6,7,8,9,-101,102,-103,104,-105,106,7000,1",
        "P protocol must match legacy format");
    ok &= expectEqual(
        CRobotProtocolCodec::encodeCalibrationAdvance(context),
        "C 1,2,3,4,5,6,7,8",
        "C protocol must match legacy format");
    ok &= expectEqual(
        CRobotProtocolCodec::encodeJobStart(context, "ONLINE01"),
        "J 1,2,3,4,5,6,7,8,ONLINE01",
        "J protocol must match legacy format");
    ok &= expectEqual(
        CRobotProtocolCodec::encodeJobStart(context, "BAD JOB"),
        "",
        "J protocol must reject whitespace in job name");
    ok &= expectEqual(
        CRobotProtocolCodec::encodeJobStart(context, "BAD,JOB"),
        "",
        "J protocol must reject comma in job name");

    RobotState state;
    ok &= expectEqual(
        CRobotProtocolCodec::decodeRobotState(
            "D 1,2,3,4,5,6,7,8,9,10,11,5,13,14,-15,16,17,"
            "-18000,19000,-20000,21000,-22000,23000,-24000,25000,"
            "2,27000,-28000,29000,30,31,32\r\n",
            &state)
            ? "valid"
            : "invalid",
        "valid",
        "complete D response should parse");
    ok &= expectEqual(
        std::to_string(state.offline),
        "5",
        "D offline field should preserve completion acknowledgement");
    ok &= expectEqual(
        std::to_string(state.curx_r),
        "-18000",
        "D signed position field should parse");
    ok &= expectEqual(
        std::to_string(state.posflag),
        "1",
        "D non-zero posflag should normalize to active positioning segment");
    ok &= expectEqual(
        CRobotProtocolCodec::decodeRobotState("D 1,2,3", &state)
            ? "valid"
            : "invalid",
        "invalid",
        "truncated D response must fail");
    ok &= expectEqual(
        CRobotProtocolCodec::decodeRobotState(
            "D 1,2,3,4,5,6,7,8,9,10,11,5,13,14,-15,16,17,"
            "-18000,19000,-20000,21000,-22000,23000,-24000,25000,"
            "2,27000,-28000,29000,30,31,32,33",
            &state)
            ? "valid"
            : "invalid",
        "invalid",
        "D response with extra fields must fail");

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: robot protocol codec tests\n";
    return 0;
}
