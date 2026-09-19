/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_online_correction_controller.cpp
 * @brief       online correction controller 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include "OnlineCorrectionController.h"

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectNear(double actual, double expected, double eps, const char* message)
{
    if (std::abs(actual - expected) > eps) {
        std::cerr << "FAIL: " << message << " actual=" << actual << " expected=" << expected << '\n';
        return false;
    }
    return true;
}

std::filesystem::path makeTempDir()
{
    auto dir = std::filesystem::temp_directory_path() / "i2r_rlc_online_correction_tests";
    std::filesystem::create_directories(dir);
    return dir;
}

void writeText(const std::filesystem::path& path, const std::string& text)
{
    std::ofstream output(path);
    output << text;
}

i2r_rlc::control::OnlineCorrectionPlannedTrajectory makeTrajectory()
{
    const auto dir = makeTempDir();
    const auto workPath = dir / "case_W.txt";
    const auto teachPath = dir / "case_T.txt";

    writeText(workPath,
        "0 0 0\n"
        "50 0 0\n"
        "100 0 0\n");
    writeText(teachPath,
        "0 0 0 1 2 3 0 0\n"
        "100 0 0 4 5 6 0 0\n");

    i2r_rlc::control::OnlineCorrectionTrajectoryRequest request;
    request.workPathFile = workPath.string();
    request.teachPoseFile = teachPath.string();
    request.segmentIndex = 0;
    return i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::loadTrajectory(request);
}

i2r_rlc::vision::PositionerCalibration makePositionerCalibration()
{
    i2r_rlc::vision::PositionerCalibration calibration;
    calibration.stationBaseInRobotBase = Eigen::Affine3d::Identity();
    calibration.valid = true;
    calibration.message = "test calibration";
    return calibration;
}

bool testTrajectoryParsingAndCommandUnits()
{
    auto trajectory = makeTrajectory();
    bool ok = true;
    ok &= expectTrue(trajectory.valid, "trajectory should load");
    ok &= expectTrue(trajectory.segmentPathPoints.size() == 3, "segment should include work path points");

    i2r_rlc::control::OnlineCorrectionOptions options;
    i2r_rlc::control::OnlineCorrectionSessionState state;
    state.initialDetectionSamples.push_back({ { 10.0, 20.0, 30.0, true }, std::nullopt });

    i2r_rlc::robot_comm::RobotState robotState;
    robotState.onlineReverseCompleteFlag = 1;

    auto plan = i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        trajectory, options, state, robotState);

    ok &= expectTrue(plan.valid, "plan should be valid");
    ok &= expectTrue(plan.command.valid, "command should be valid");
    ok &= expectTrue(plan.command.pointCount == 2, "two teach points should be generated");
    ok &= expectTrue(plan.command.teachPoints[0].xUm == 10000, "x should convert mm to um");
    ok &= expectTrue(plan.command.teachPoints[0].yUm == 20000, "y should convert mm to um");
    ok &= expectTrue(plan.command.teachPoints[0].zUm == 30000, "z should convert mm to um");
    ok &= expectTrue(plan.command.teachPoints[0].rxDeg10000 == 10000, "rx should convert deg to deg10000");
    ok &= expectTrue(plan.command.stationAxes[0].s1Deg10000 == 0, "s1 should convert deg to deg10000");
    return ok;
}

bool testControllerStateFlow()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    controller.configure(options, trajectory, makePositionerCalibration());

    bool ok = true;
    auto update = controller.startInitialDetection();
    ok &= expectTrue(update.phase == i2r_rlc::control::OnlineCorrectionPhase::InitialDetectionRunning,
                     "initial detection should start");

    i2r_rlc::control::OnlineCorrectionDetectionEvent event;
    event.samples.push_back({ { 10.0, 0.0, 0.0, true }, std::nullopt });
    event.detectionFinished = true;
    update = controller.appendDetectionEvent(event);
    ok &= expectTrue(update.phase == i2r_rlc::control::OnlineCorrectionPhase::WaitingForReverseCompletion,
                     "finished detection should wait for reverse completion");

    i2r_rlc::robot_comm::RobotState robotState;
    update = controller.onDetectionFinished(robotState);
    ok &= expectTrue(!update.succeeded, "missing reverse flag should not generate command");

    robotState.onlineReverseCompleteFlag = 1;
    update = controller.onRobotStateChanged(robotState);
    ok &= expectTrue(update.succeeded, "reverse flag should generate command");
    ok &= expectTrue(
        update.phase == i2r_rlc::control::OnlineCorrectionPhase::SendingInitialTeachPoints,
        "controller should send initial teach points");
    ok &= expectTrue(update.command.valid, "online ready update should carry command");

    i2r_rlc::robot_comm::RobotCommandReceipt receipt;
    receipt.type = i2r_rlc::robot_comm::RobotCommandType::TeachPointUpdate;
    receipt.sent = true;
    receipt.responseReceived = true;
    receipt.completed = true;
    receipt.pointIndex = 0;
    robotState.offline = 5;
    update = controller.onRobotCommandReceipt(receipt, robotState);
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::WaitingInitialTeachPointAck,
        "non-final teach point acknowledgement must not start online correction");

    receipt.pointIndex = 1;
    update = controller.onRobotCommandReceipt(receipt, robotState);
    ok &= expectTrue(update.phase == i2r_rlc::control::OnlineCorrectionPhase::OnlineRunning,
                     "teach point acknowledgement should start online correction");
    ok &= expectTrue(update.startTracking, "online correction should request tracking");

    i2r_rlc::control::OnlineCorrectionDetectionEvent onlineEvent;
    onlineEvent.samples.push_back({ { 10.0, 1.0, 0.0, true }, std::nullopt, 0 });
    onlineEvent.samples.push_back({ { 20.0, 1.0, 0.0, true }, std::nullopt, 0 });
    onlineEvent.samples.push_back({ { 30.0, 1.0, 0.0, true }, std::nullopt, 0 });
    onlineEvent.samples.push_back({ { 40.0, 1.0, 0.0, true }, std::nullopt, 0 });
    update = controller.appendDetectionEvent(onlineEvent);
    ok &= expectTrue(update.realtimeCommand.valid,
                     "online samples should generate realtime correction");

    update = controller.finishOnlineSegment();
    ok &= expectTrue(
        update.phase == i2r_rlc::control::OnlineCorrectionPhase::SegmentFinishing,
        "finish request should enter segment finishing");
    ok &= expectTrue(
        update.realtimeCommand.onlineEndFlag == 1,
        "final realtime command should carry Online_endflag");

    i2r_rlc::robot_comm::RobotCommandReceipt finalReceipt;
    finalReceipt.type =
        i2r_rlc::robot_comm::RobotCommandType::RealtimeCorrection;
    finalReceipt.sent = true;
    finalReceipt.responseReceived = true;
    finalReceipt.completed = true;
    update = controller.onRobotCommandReceipt(finalReceipt, robotState);
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::WaitingForDetectionReset,
        "final P acknowledgement should wait for detflag reset");
    ok &= expectTrue(update.stopTracking,
                     "acknowledged segment should stop vision tracking");
    robotState.detflag = 0;
    update = controller.onRobotStateChanged(robotState);
    ok &= expectTrue(
        update.phase == i2r_rlc::control::OnlineCorrectionPhase::Completed,
        "detflag reset should complete the final segment");
    return ok;
}

bool testControllerRejectsMissingCalibration()
{
    i2r_rlc::control::COnlineCorrectionController controller;
    controller.configure(
        i2r_rlc::control::OnlineCorrectionOptions{},
        makeTrajectory());

    const auto update = controller.startInitialDetection();
    bool ok = true;
    ok &= expectTrue(!update.succeeded,
                     "missing RBCALIB calibration should reject start");
    ok &= expectTrue(
        update.phase == i2r_rlc::control::OnlineCorrectionPhase::Error,
        "missing RBCALIB calibration should enter error phase");
    ok &= expectTrue(!update.startTracking,
                     "invalid configuration must not start vision tracking");
    return ok;
}

bool testReverseCompletionRequiresFreshEdge()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    controller.configure(options, trajectory, makePositionerCalibration());

    i2r_rlc::robot_comm::RobotState state;
    state.onlineReverseCompleteFlag = 1;
    controller.onRobotStateChanged(state);
    auto update = controller.startInitialDetection();

    i2r_rlc::control::OnlineCorrectionDetectionEvent event;
    event.samples.push_back({ { 10.0, 0.0, 0.0, true }, std::nullopt });
    controller.appendDetectionEvent(event);

    update = controller.onRobotStateChanged(state);
    bool ok = true;
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::InitialDetectionRunning,
        "stale reverse completion high level must not finish a new cycle");

    state.onlineReverseCompleteFlag = 0;
    controller.onRobotStateChanged(state);
    state.onlineReverseCompleteFlag = 1;
    update = controller.onRobotStateChanged(state);
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::SendingInitialTeachPoints,
        "fresh reverse completion rising edge should finish initial detection");
    return ok;
}

bool testTeachPointBatchKeepsLegacyMissingStationAxes()
{
    auto trajectory = makeTrajectory();
    trajectory.stationAxes[1].valid = false;
    i2r_rlc::control::OnlineCorrectionSessionState state;
    state.initialDetectionSamples.push_back(
        {{10.0, 0.0, 0.0, true}, std::nullopt});
    i2r_rlc::robot_comm::RobotState robotState;
    robotState.onlineReverseCompleteFlag = 1;
    const auto plan =
        i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
            trajectory,
            i2r_rlc::control::OnlineCorrectionOptions{},
            state,
            robotState,
            makePositionerCalibration());
    return expectTrue(
        plan.valid && plan.command.teachPoints.size() == 2
            && plan.command.stationAxes.size() == 2,
        "legacy O generation should continue with zero-valued missing positioner axes");
}

bool testPositionerRoundTrip()
{
    constexpr double pi = 3.14159265358979323846;
    auto calibration = makePositionerCalibration();
    calibration.stationPoseMmDeg = { 100.0, -50.0, 25.0, 5.0, -3.0, 8.0 };
    const Eigen::Translation3d translation(
        -calibration.stationPoseMmDeg[0],
        -calibration.stationPoseMmDeg[1],
        -calibration.stationPoseMmDeg[2]);
    const Eigen::AngleAxisd rotationX(
        -calibration.stationPoseMmDeg[3] * pi / 180.0,
        Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd rotationY(
        -calibration.stationPoseMmDeg[4] * pi / 180.0,
        Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rotationZ(
        -calibration.stationPoseMmDeg[5] * pi / 180.0,
        Eigen::Vector3d::UnitZ());
    calibration.stationBaseInRobotBase =
        Eigen::Affine3d((rotationX * rotationY * rotationZ).toRotationMatrix())
        * translation;

    i2r_rlc::vision::PositionerPose stationPose{
        12.0, 34.0, 56.0, 1.0, 2.0, 3.0, true
    };
    const auto robotPose =
        i2r_rlc::vision::CPositionerKinematics::stationPoseToRobotPose(
            stationPose, 15.0, -20.0, calibration);
    const auto roundTrip =
        i2r_rlc::vision::CPositionerKinematics::robotPoseToStationPose(
            robotPose, 15.0, -20.0, calibration);

    bool ok = true;
    ok &= expectTrue(robotPose.valid && roundTrip.valid,
                     "positioner conversions should stay valid");
    ok &= expectNear(roundTrip.xMm, stationPose.xMm, 1e-8,
                     "positioner round-trip x");
    ok &= expectNear(roundTrip.yMm, stationPose.yMm, 1e-8,
                     "positioner round-trip y");
    ok &= expectNear(roundTrip.zMm, stationPose.zMm, 1e-8,
                     "positioner round-trip z");
    return ok;
}

bool testRbcalibGoldenTranslation()
{
    const auto rbcalibPath = makeTempDir() / "RBCALIB.DAT";
    writeText(rbcalibPath, "header\n////SRANG 10,20,30,0,0,0\n");

    const auto calibration =
        i2r_rlc::vision::CPositionerCalibrationParser::parseFile(
            rbcalibPath.string());
    const i2r_rlc::vision::PositionerPose stationPose{
        1.0, 2.0, 3.0, 0.0, 0.0, 0.0, true
    };
    const auto robotPose =
        i2r_rlc::vision::CPositionerKinematics::stationPoseToRobotPose(
            stationPose, 0.0, 0.0, calibration);

    bool ok = true;
    ok &= expectTrue(calibration.valid, "RBCALIB SRANG should parse");
    ok &= expectTrue(robotPose.valid, "golden station pose should convert");
    ok &= expectNear(robotPose.xMm, 11.0, 1e-9,
                     "legacy SRANG translation should add robot-base x");
    ok &= expectNear(robotPose.yMm, 22.0, 1e-9,
                     "legacy SRANG translation should add robot-base y");
    ok &= expectNear(robotPose.zMm, 33.0, 1e-9,
                     "legacy SRANG translation should add robot-base z");
    return ok;
}

bool testExceptionProcessorUsesSeamIndexAndIcp()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    const std::vector<i2r_rlc::control::OnlineCorrectionPoint> reference{
        { 0.0, 0.0, 0.0, true },
        { 10.0, 0.0, 0.0, true },
        { 0.0, 10.0, 0.0, true },
        { 0.0, 0.0, 10.0, true },
        { 10.0, 10.0, 10.0, true }
    };
    const std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> first{
        { { 2.0, 3.0, 4.0, true }, std::nullopt, 0 },
        { { 12.0, 3.0, 4.0, true }, std::nullopt, 0 },
        { { 2.0, 13.0, 4.0, true }, std::nullopt, 0 },
        { { 2.0, 3.0, 14.0, true }, std::nullopt, 0 },
        { { 12.0, 13.0, 14.0, true }, std::nullopt, 0 }
    };
    auto result = processor.process(first, reference);

    auto changed = first;
    for (auto& sample : changed) {
        sample.simulatedSeamIndex = 1;
        sample.gapWidthMm = 1.5F;
    }
    changed.push_back({ { 7.0, 8.0, 9.0, true }, 2.0F, 1 });
    result = processor.process(changed, reference);

    bool ok = true;
    ok &= expectTrue(result.valid, "changed simulated seam should be processed");
    ok &= expectTrue(result.segmentChanged,
                     "getSeamIndex change should mark segment boundary");
    ok &= expectTrue(!result.samples.empty(),
                     "successful ICP should output processed legacy samples");
    ok &= expectTrue(result.samples.front().point.valid,
                     "ICP predicted sample should be valid");
    return ok;
}

std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> makeFollowerSamples(
    double firstX,
    double y,
    int seamIndex = 0)
{
    return {
        { { firstX, y, 0.0, true }, std::nullopt, seamIndex },
        { { firstX + 10.0, y, 0.0, true }, std::nullopt, seamIndex },
        { { firstX + 20.0, y, 0.0, true }, std::nullopt, seamIndex },
        { { firstX + 30.0, y, 0.0, true }, std::nullopt, seamIndex }
    };
}

bool testPathFollowerRemovesRepeatedTarget()
{
    i2r_rlc::control::COnlineCorrectionPathFollower follower;
    const std::vector<i2r_rlc::control::OnlineCorrectionPoint> teachTrajectory{
        { 0.0, 0.0, 0.0, true },
        { 100.0, 0.0, 0.0, true }
    };
    auto samples = makeFollowerSamples(10.0, 1.0);
    i2r_rlc::robot_comm::RobotState robotState;
    i2r_rlc::control::OnlineCorrectionFollowResult result;
    for (int index = 0; index < 11; ++index) {
        result = follower.update(
            robotState,
            i2r_rlc::control::OnlineCorrectionOptions{},
            makePositionerCalibration(),
            teachTrajectory,
            &samples,
            false);
    }

    bool ok = true;
    ok &= expectTrue(result.message == "repeated online target point removed",
                     "eleventh identical target should be removed");
    ok &= expectTrue(samples.size() == 3,
                     "duplicate target removal should consume one point");
    return ok;
}

bool testPathFollowerKeepsDirectionReversalLikeLegacyActiveCode()
{
    i2r_rlc::control::COnlineCorrectionPathFollower follower;
    const std::vector<i2r_rlc::control::OnlineCorrectionPoint> teachTrajectory{
        { 0.0, 0.0, 0.0, true },
        { 100.0, 0.0, 0.0, true }
    };
    i2r_rlc::robot_comm::RobotState robotState;
    for (int index = 0; index < 10; ++index) {
        auto samples = makeFollowerSamples(10.0 + index, 1.0);
        follower.update(
            robotState,
            i2r_rlc::control::OnlineCorrectionOptions{},
            makePositionerCalibration(),
            teachTrajectory,
            &samples,
            false);
    }

    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> reversed{
        { { -10.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { -20.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { -30.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { -40.0, 1.0, 0.0, true }, std::nullopt, 0 }
    };
    const auto result = follower.update(
        robotState,
        i2r_rlc::control::OnlineCorrectionOptions{},
        makePositionerCalibration(),
        teachTrajectory,
        &reversed,
        false);

    bool ok = true;
    ok &= expectTrue(result.valid,
                     "legacy active code should keep generating correction for reversed direction");
    ok &= expectTrue(reversed.size() == 4,
                     "legacy active code should not consume reversed point by direction filter");
    return ok;
}

bool testPathFollowerLimitsOutputJump()
{
    i2r_rlc::control::COnlineCorrectionPathFollower follower;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.maxCorrectionJumpUm = 1;
    const std::vector<i2r_rlc::control::OnlineCorrectionPoint> teachTrajectory{
        { 0.0, 0.0, 0.0, true },
        { 100.0, 0.0, 0.0, true }
    };
    i2r_rlc::robot_comm::RobotState robotState;
    auto positive = makeFollowerSamples(10.0, 1.0);
    const auto first = follower.update(
        robotState,
        options,
        makePositionerCalibration(),
        teachTrajectory,
        &positive,
        false);
    auto negative = makeFollowerSamples(11.0, -1.0);
    const auto second = follower.update(
        robotState,
        options,
        makePositionerCalibration(),
        teachTrajectory,
        &negative,
        false);

    bool ok = true;
    ok &= expectTrue(first.valid && second.valid,
                     "jump-limit scenario should generate commands");
    ok &= expectTrue(second.command.dyUm == first.command.dyUm,
                     "excessive correction jump should keep previous output");
    return ok;
}

bool testRealtimePathFollowerLimitsAndGapSpeed()
{
    i2r_rlc::control::COnlineCorrectionPathFollower follower;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableGapDetection = true;
    const std::vector<i2r_rlc::control::OnlineCorrectionPoint> teachTrajectory{
        { 0.0, 0.0, 0.0, true },
        { 100.0, 0.0, 0.0, true }
    };
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> samples{
        { { 10.0, 1.0, 0.0, true }, 2.0F, 0 },
        { { 20.0, 1.0, 0.0, true }, 2.0F, 0 },
        { { 30.0, 1.0, 0.0, true }, 2.0F, 0 },
        { { 40.0, 1.0, 0.0, true }, 2.0F, 0 }
    };
    i2r_rlc::robot_comm::RobotState robotState;
    robotState.cur_interpolation = 2;

    const auto result = follower.update(
        robotState,
        options,
        makePositionerCalibration(),
        teachTrajectory,
        &samples,
        false);

    bool ok = true;
    ok &= expectTrue(result.valid,
                     "path follower should generate realtime correction");
    ok &= expectTrue(std::abs(result.command.dxUm) <= 80,
                     "realtime x increment should respect limit");
    ok &= expectTrue(std::abs(result.command.dyUm) <= 80,
                     "realtime y increment should respect limit");
    ok &= expectTrue(std::abs(result.command.dzUm) <= 80,
                     "realtime z increment should respect limit");
    ok &= expectTrue(result.command.speedOverride == 6000,
                     "2 mm normalized gap should select 60 percent speed");
    ok &= expectTrue(result.command.speedMode == 1,
                     "linear interpolation should set legacy speed mode");
    return ok;
}

bool testPathFollowerControlMethodTwoIsLegacyNoop()
{
    i2r_rlc::control::COnlineCorrectionPathFollower follower;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.controlMethod =
        i2r_rlc::control::OnlineCorrectionControlMethod::LegacyReservedNoop;
    const std::vector<i2r_rlc::control::OnlineCorrectionPoint> teachTrajectory{
        { 0.0, 0.0, 0.0, true },
        { 100.0, 0.0, 0.0, true }
    };
    auto samples = makeFollowerSamples(10.0, 1.0);
    i2r_rlc::robot_comm::RobotState robotState;
    const auto result = follower.update(
        robotState,
        options,
        makePositionerCalibration(),
        teachTrajectory,
        &samples,
        false);

    bool ok = true;
    ok &= expectTrue(result.valid, "ControlMethod==2 should still emit a P command");
    ok &= expectTrue(result.command.dxUm == 0 && result.command.dyUm == 0
                         && result.command.dzUm == 0,
                     "ControlMethod==2 should keep legacy zero correction output");
    return ok;
}

bool testIbvsStartModeUsesRobotState()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.startMode = i2r_rlc::control::OnlineCorrectionStartMode::IBVSStart;

    i2r_rlc::control::OnlineCorrectionSessionState state;
    state.startMode = options.startMode;

    i2r_rlc::robot_comm::RobotState robotState;
    robotState.curx_r = 21000;
    robotState.cury_r = 22000;
    robotState.curz_r = 23000;

    auto plan = i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        trajectory, options, state, robotState);

    bool ok = true;
    ok &= expectTrue(plan.valid, "IBVS mode should use robot current point");
    ok &= expectTrue(plan.command.teachPoints[0].xUm == 21000, "IBVS x should come from robot state");
    ok &= expectTrue(plan.command.teachPoints[0].yUm == 22000, "IBVS y should come from robot state");
    ok &= expectTrue(plan.command.teachPoints[0].zUm == 23000, "IBVS z should come from robot state");
    return ok;
}

bool testGlobalCorrectionModeAppliesTransform()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.startMode = i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection;
    options.globalCorrectionRowMajor = {
        1.0, 0.0, 0.0, 10.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    i2r_rlc::control::OnlineCorrectionSessionState state;
    state.initialDetectionSamples.push_back({ { 20.0, 0.0, 0.0, true }, std::nullopt });
    i2r_rlc::robot_comm::RobotState robotState;
    robotState.onlineReverseCompleteFlag = 1;

    auto plan = i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        trajectory, options, state, robotState);

    bool ok = true;
    ok &= expectTrue(plan.valid, "global correction mode should plan");
    ok &= expectTrue(plan.command.teachPoints[0].xUm == 20000,
                     "global transform plus initial alignment should set first point to detected point");
    ok &= expectNear(plan.correctedSegmentPath.front().xMm, 20.0, 1e-9,
                     "corrected segment should start at detected point");
    return ok;
}

bool testDetectionSamplesKeepGapWithPoint()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableGapDetection = true;
    controller.configure(options, trajectory, makePositionerCalibration());
    controller.startInitialDetection();

    i2r_rlc::control::OnlineCorrectionDetectionEvent event;
    event.samples.push_back({ { 10.0, 0.0, 0.0, true }, 1.5F });
    event.samples.push_back({ { 20.0, 0.0, 0.0, false }, std::nullopt });
    event.samples.push_back({ { 30.0, 0.0, 0.0, true }, 2.5F });

    const auto update = controller.appendDetectionEvent(event);

    bool ok = true;
    ok &= expectTrue(update.succeeded, "valid samples with gap should be accepted");
    ok &= expectTrue(controller.state().initialDetectionSamples.size() == 2,
                     "only valid samples should be stored");
    ok &= expectTrue(controller.state().initialDetectionSamples[0].gapWidthMm.has_value(),
                     "stored sample should keep gap width");
    ok &= expectNear(*controller.state().initialDetectionSamples[0].gapWidthMm, 1.5, 1e-6,
                     "first gap should stay paired with first valid point");
    ok &= expectNear(*controller.state().initialDetectionSamples[1].gapWidthMm, 2.5, 1e-6,
                     "second gap should stay paired with second valid point");
    return ok;
}

bool testGapCanBeOptionalWhenDisabled()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableGapDetection = false;
    controller.configure(options, trajectory, makePositionerCalibration());
    controller.startInitialDetection();

    i2r_rlc::control::OnlineCorrectionDetectionEvent event;
    event.samples.push_back({ { 10.0, 0.0, 0.0, true }, std::nullopt });

    const auto update = controller.appendDetectionEvent(event);

    bool ok = true;
    ok &= expectTrue(update.succeeded, "sample without gap should be accepted when gap detection is disabled");
    ok &= expectTrue(controller.state().initialDetectionSamples.size() == 1,
                     "sample should be stored");
    ok &= expectTrue(!controller.state().initialDetectionSamples[0].gapWidthMm.has_value(),
                     "gap should remain empty when disabled");
    return ok;
}

bool testMissingGapFailsWhenEnabled()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableGapDetection = true;
    controller.configure(options, trajectory, makePositionerCalibration());
    controller.startInitialDetection();

    i2r_rlc::control::OnlineCorrectionDetectionEvent event;
    event.samples.push_back({ { 10.0, 0.0, 0.0, true }, std::nullopt });

    const auto update = controller.appendDetectionEvent(event);

    bool ok = true;
    ok &= expectTrue(!update.succeeded, "missing gap should fail when gap detection is enabled");
    ok &= expectTrue(update.phase == i2r_rlc::control::OnlineCorrectionPhase::Error,
                     "missing gap should put controller into error phase");
    ok &= expectTrue(controller.state().initialDetectionSamples.empty(),
                     "invalid event should not append sample");
    return ok;
}

bool testInitTypeDoesNotApplyGlobalMatrix()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.startMode = i2r_rlc::control::OnlineCorrectionStartMode::InitType;
    options.globalCorrectionRowMajor = {
        0.0, -1.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    i2r_rlc::control::OnlineCorrectionSessionState state;
    state.initialDetectionSamples.push_back({ { 20.0, 0.0, 0.0, true }, std::nullopt });
    i2r_rlc::robot_comm::RobotState robotState;
    robotState.onlineReverseCompleteFlag = 1;

    auto plan = i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        trajectory, options, state, robotState);

    bool ok = true;
    ok &= expectTrue(plan.valid, "InitType should plan without global correction");
    ok &= expectNear(plan.correctedSegmentPath.front().xMm, 20.0, 1e-9,
                     "InitType should align start to detected point");
    ok &= expectNear(plan.correctedSegmentPath.back().xMm, 120.0, 1e-9,
                     "InitType should not rotate path by global matrix");
    ok &= expectNear(plan.correctedSegmentPath.back().yMm, 0.0, 1e-9,
                     "InitType y should remain unchanged");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testTrajectoryParsingAndCommandUnits();
    ok &= testControllerStateFlow();
    ok &= testControllerRejectsMissingCalibration();
    ok &= testReverseCompletionRequiresFreshEdge();
    ok &= testTeachPointBatchKeepsLegacyMissingStationAxes();
    ok &= testPositionerRoundTrip();
    ok &= testRbcalibGoldenTranslation();
    ok &= testExceptionProcessorUsesSeamIndexAndIcp();
    ok &= testPathFollowerRemovesRepeatedTarget();
    ok &= testPathFollowerKeepsDirectionReversalLikeLegacyActiveCode();
    ok &= testPathFollowerLimitsOutputJump();
    ok &= testRealtimePathFollowerLimitsAndGapSpeed();
    ok &= testPathFollowerControlMethodTwoIsLegacyNoop();
    ok &= testIbvsStartModeUsesRobotState();
    ok &= testGlobalCorrectionModeAppliesTransform();
    ok &= testDetectionSamplesKeepGapWithPoint();
    ok &= testGapCanBeOptionalWhenDisabled();
    ok &= testMissingGapFailsWhenEnabled();
    ok &= testInitTypeDoesNotApplyGlobalMatrix();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: online correction controller tests\n";
    return 0;
}
