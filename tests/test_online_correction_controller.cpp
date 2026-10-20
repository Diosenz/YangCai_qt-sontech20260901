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
    robotState.curRx_r = 120000;

    auto plan = i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        trajectory, options, state, robotState);

    ok &= expectTrue(plan.valid, "plan should be valid");
    ok &= expectTrue(plan.command.valid, "command should be valid");
    ok &= expectTrue(plan.command.pointCount == 2, "two teach points should be generated");
    ok &= expectTrue(plan.command.teachPoints[0].xUm == 10000, "x should convert mm to um");
    ok &= expectTrue(plan.command.teachPoints[0].yUm == 20000, "y should convert mm to um");
    ok &= expectTrue(plan.command.teachPoints[0].zUm == 30000, "z should convert mm to um");
    ok &= expectTrue(plan.command.teachPoints[0].rxDeg10000 == robotState.curRx_r,
                     "P001 rx should use the reverse-completion robot pose");
    ok &= expectTrue(plan.command.stationAxes[0].s1Deg10000 == 0, "s1 should convert deg to deg10000");
    return ok;
}

bool testTrajectoryIcpSegmentsFollowAdjacentTeachPoints()
{
    const auto dir = makeTempDir();
    const auto workPath = dir / "icp_segments_W.txt";
    const auto teachPath = dir / "icp_segments_T.txt";
    writeText(workPath,
        "0 0 0\n10 0 0\n20 0 0\n30 0 0\n40 0 0\n"
        "50 0 0\n60 0 0\n70 0 0\n80 0 0\n");
    writeText(teachPath,
        "0 0 0 0 0 0 0 0 0\n"
        "20 0 0 0 0 0 0 0 0\n"
        "40 0 0 0 0 0 0 0 0\n"
        "60 0 0 0 0 0 0 0 1\n"
        "80 0 0 0 0 0 0 0 1\n");
    i2r_rlc::control::OnlineCorrectionTrajectoryRequest request;
    request.workPathFile = workPath.string();
    request.teachPoseFile = teachPath.string();
    request.segmentIndex = 0;
    const auto first =
        i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::loadTrajectory(request);
    bool ok = expectTrue(first.valid && first.pathSegments.size() == 2U,
                         "ICP references should follow adjacent teach points in the active program segment");
    if (first.pathSegments.size() == 2U) {
        ok &= expectTrue(first.pathSegments[0].size() == 3U
                             && first.pathSegments[1].size() == 3U,
                         "adjacent teach references should include their shared endpoint");
        ok &= expectNear(first.pathSegments[1].front().xMm, 20.0, 1.0e-12,
                         "second seam reference should begin at the middle teach point");
        ok &= expectNear(first.pathSegments[1].back().xMm, 40.0, 1.0e-12,
                         "second seam reference should stop at the active segment end");
    }
    request.segmentIndex = 1;
    const auto second =
        i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::loadTrajectory(request);
    ok &= expectTrue(second.valid && second.pathSegments.size() == 1U,
                     "the next program segment should not retain earlier ICP references");
    if (second.pathSegments.size() == 1U) {
        ok &= expectNear(second.pathSegments.front().front().xMm, 60.0, 1.0e-12,
                         "next segment ICP reference should use its local first seam");
    }
    return ok;
}

bool testReverseP001OrientationChangesOnlyFirstTeachPoint()
{
    auto trajectory = makeTrajectory();
    trajectory.stationAxes[0] = { 12.0, -34.0, true };
    trajectory.stationAxes[1] = { 12.0, 46.0, true };
    auto calibration = makePositionerCalibration();
    calibration.stationBaseInRobotBase = Eigen::Affine3d(
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix());

    i2r_rlc::control::OnlineCorrectionSessionState state;
    state.initialDetectionSamples.push_back(
        { { 10.0, 20.0, 30.0, true }, std::nullopt });
    i2r_rlc::robot_comm::RobotState firstFeedback;
    firstFeedback.curRx_r = -1501234;
    firstFeedback.curRy_r = 234567;
    firstFeedback.curRz_r = -456789;
    auto secondFeedback = firstFeedback;
    secondFeedback.curRx_r = -1400000;
    secondFeedback.curRy_r = 123456;
    secondFeedback.curRz_r = 345678;

    bool ok = true;
    for (const auto startMode : {
             i2r_rlc::control::OnlineCorrectionStartMode::InitType,
             i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection }) {
        i2r_rlc::control::OnlineCorrectionOptions options;
        options.startMode = startMode;
        if (startMode == i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection) {
            options.globalCorrectionRowMajor = {
                0.0, -1.0, 0.0, 10.0,
                1.0,  0.0, 0.0, 20.0,
                0.0,  0.0, 1.0, 30.0,
                0.0,  0.0, 0.0, 1.0
            };
        }
        const auto firstPlan =
            i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
                trajectory, options, state, firstFeedback, calibration);
        const auto secondPlan =
            i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
                trajectory, options, state, secondFeedback, calibration);
        ok &= expectTrue(firstPlan.valid && secondPlan.valid,
                         "both reverse P001 plans should be valid");
        if (!firstPlan.valid || !secondPlan.valid) {
            continue;
        }

        const auto &first = firstPlan.command.teachPoints.front();
        const auto &second = secondPlan.command.teachPoints.front();
        ok &= expectTrue(first.rxDeg10000 == firstFeedback.curRx_r
                             && first.ryDeg10000 == firstFeedback.curRy_r
                             && first.rzDeg10000 == firstFeedback.curRz_r,
                         "P001 RobotBase orientation should equal first feedback");
        ok &= expectTrue(second.rxDeg10000 == secondFeedback.curRx_r
                             && second.ryDeg10000 == secondFeedback.curRy_r
                             && second.rzDeg10000 == secondFeedback.curRz_r,
                         "P001 RobotBase orientation should follow changed feedback");
        ok &= expectTrue(first.xUm == second.xUm
                             && first.yUm == second.yUm
                             && first.zUm == second.zUm,
                         "P001 position should not depend on feedback orientation");
        ok &= expectTrue(firstPlan.command.stationAxes[0].s1Deg10000
                             == secondPlan.command.stationAxes[0].s1Deg10000
                             && firstPlan.command.stationAxes[0].s2Deg10000
                                 == secondPlan.command.stationAxes[0].s2Deg10000,
                         "P001 target station axes should not depend on feedback orientation");
        const auto &laterFirst = firstPlan.command.teachPoints[1];
        const auto &laterSecond = secondPlan.command.teachPoints[1];
        ok &= expectTrue(laterFirst.xUm == laterSecond.xUm
                             && laterFirst.yUm == laterSecond.yUm
                             && laterFirst.zUm == laterSecond.zUm
                             && laterFirst.rxDeg10000 == laterSecond.rxDeg10000
                             && laterFirst.ryDeg10000 == laterSecond.ryDeg10000
                             && laterFirst.rzDeg10000 == laterSecond.rzDeg10000,
                         "later teach point should retain template behavior");
    }
    return ok;
}

bool testControllerStateFlow()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableOnlineIcp = false;
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
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::WaitingForInitialApproach,
        "teach point acknowledgement should wait for the P001 approach");
    robotState.corflag = 2;
    robotState.detflag = 2;
    update = controller.onRobotStateChanged(robotState);
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::WaitingForOnlineStart,
        "P001 ENDDET should wait for STARTCOR");
    robotState.corflag = 1;
    robotState.detflag = 1;
    update = controller.onRobotStateChanged(robotState);
    ok &= expectTrue(update.phase == i2r_rlc::control::OnlineCorrectionPhase::OnlineRunning,
                     "STARTCOR should start online correction");
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
    state.curRx_r = 110000;
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
    state.curRx_r = 220000;
    controller.onRobotStateChanged(state);
    state.onlineReverseCompleteFlag = 1;
    state.curRx_r = 330000;
    update = controller.onRobotStateChanged(state);
    ok &= expectTrue(
        update.phase
            == i2r_rlc::control::OnlineCorrectionPhase::SendingInitialTeachPoints,
        "fresh reverse completion rising edge should finish initial detection");
    ok &= expectTrue(update.command.valid
                         && update.command.teachPoints.front().rxDeg10000
                             == state.curRx_r,
                     "P001 should use orientation from the fresh reverse-completion state");
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

std::vector<i2r_rlc::control::OnlineCorrectionPoint> makeIcpReference()
{
    std::vector<i2r_rlc::control::OnlineCorrectionPoint> points;
    for (int index = 0; index <= 30; ++index) {
        const double value = static_cast<double>(index);
        points.push_back({
            2.0 * value,
            0.12 * value * value,
            1.2 * std::sin(0.45 * value),
            true
        });
    }
    return points;
}

i2r_rlc::control::OnlineCorrectionDetectionSample makeIcpSample(
    const i2r_rlc::control::OnlineCorrectionPoint &point,
    int seamIndex)
{
    const double cosine = std::cos(0.012);
    const double sine = std::sin(0.012);
    return {
        { cosine * point.xMm - sine * point.yMm + 0.2,
          sine * point.xMm + cosine * point.yMm - 0.3,
          point.zMm + 0.25,
          true },
        1.5F,
        seamIndex
    };
}

i2r_rlc::control::OnlineCorrectionPlannedTrajectory makeIcpTrajectory()
{
    auto trajectory = makeTrajectory();
    trajectory.allPathPoints = makeIcpReference();
    trajectory.segmentPathPoints = trajectory.allPathPoints;
    trajectory.allMatchedPathPoints = trajectory.allPathPoints;
    trajectory.pathSegments = { trajectory.segmentPathPoints };
    trajectory.segmentStartPathIndex = 0;
    trajectory.segmentEndPathIndex =
        static_cast<int>(trajectory.segmentPathPoints.size()) - 1;
    return trajectory;
}

bool testExceptionProcessorAccumulatesSingleFramesAndAppliesIcp()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    const auto reference = makeIcpReference();
    bool ok = true;
    i2r_rlc::control::OnlineCorrectionExceptionResult result;
    for (std::size_t index = 0; index < 11U; ++index) {
        result = processor.process({ makeIcpSample(reference[index], -1) }, reference);
        ok &= expectTrue(result.valid, "a buffered single frame should be accepted");
        if (index < 10U) {
            ok &= expectTrue(!result.icpAttempted && !result.icpApplied,
                             "fewer than five stride-three samples should wait");
            ok &= expectTrue(result.samples.empty(),
                             "waiting frames must not masquerade as an ICP result");
            ok &= expectTrue(result.bufferedSampleCount == index + 1U,
                             "single-frame history must accumulate without replacement");
        }
    }

    ok &= expectTrue(result.icpAttempted && result.icpApplied,
                     "eleven raw points with the final endpoint should really invoke ICP");
    ok &= expectTrue(!result.samples.empty(), "applied ICP should emit a model subrange");
    ok &= expectTrue(std::isfinite(result.rmsErrorMm) && result.rmsErrorMm < 0.05,
                     "known rigid transform should have a small finite ICP residual");
    ok &= expectTrue(result.batches.size() == 1U,
                     "one released batch must have one diagnostic snapshot");
    if (result.batches.size() == 1U) {
        const auto &batch = result.batches.front();
        ok &= expectTrue(batch.attempted && batch.applied && batch.detectionToModel.has_value()
                             && batch.pendingSamples.size() == 11U && batch.fitPoints.size() >= 5U
                             && batch.releasedSamples.size() == result.samples.size(),
                         "diagnostics must preserve solver inputs and actual released targets");
        if (batch.detectionToModel.has_value()) {
            const auto &matrix = *batch.detectionToModel;
            const auto &point = batch.pendingSamples.front().point;
            const double z = matrix[8] * point.xMm + matrix[9] * point.yMm
                + matrix[10] * point.zMm + matrix[11];
            ok &= expectNear(z, reference.front().zMm, 0.05,
                             "row-major diagnostic matrix must map detection back to model");
        }
    }
    double maximumPredictionError = 0.0;
    for (const auto &sample : result.samples) {
        double nearest = 1.0e9;
        for (const auto &point : reference) {
            const auto expected = makeIcpSample(point, -1).point;
            const double dx = sample.point.xMm - expected.xMm;
            const double dy = sample.point.yMm - expected.yMm;
            const double dz = sample.point.zMm - expected.zMm;
            nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        maximumPredictionError = std::max(maximumPredictionError, nearest);
        ok &= expectTrue(sample.gapWidthMm.has_value(),
                         "ICP output should retain the associated gap measurement");
    }
    ok &= expectTrue(maximumPredictionError < 0.05,
                     "model-to-detection inverse transform must recover the known offset");
    if (result.samples.empty()) {
        return false;
    }
    const double previousLastX = result.samples.back().point.xMm;
    for (std::size_t index = 11U; index < 14U; ++index) {
        result = processor.process({ makeIcpSample(reference[index], -1) }, reference);
        if (index < 13U) {
            ok &= expectTrue(!result.icpAttempted && result.samples.empty(),
                             "retained history should wait for three new points");
        }
    }
    ok &= expectTrue(result.icpApplied && !result.samples.empty(),
                     "a retained window should support the next single-frame batch");
    if (!result.samples.empty()) {
        ok &= expectTrue(result.samples.front().point.xMm > previousLastX + 0.5,
                         "new ICP output must not replay the retained history");
    }
    return ok;
}

bool testExceptionProcessorWaitsForDensePendingSpan()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    const auto reference = makeIcpReference();
    i2r_rlc::control::OnlineCorrectionExceptionResult result;
    for (std::size_t index = 0; index < 11U; ++index) {
        result = processor.process({ makeIcpSample(reference[index], -1) }, reference);
    }
    bool ok = expectTrue(result.icpApplied,
                         "dense pending regression requires an applied initial window");
    for (int index = 1; index <= 3; ++index) {
        const double fraction = 0.05 * static_cast<double>(index);
        const auto &first = reference[10];
        const auto &last = reference[11];
        const i2r_rlc::control::OnlineCorrectionPoint point{
            first.xMm + fraction * (last.xMm - first.xMm),
            first.yMm + fraction * (last.yMm - first.yMm),
            first.zMm + fraction * (last.zMm - first.zMm),
            true
        };
        result = processor.process({ makeIcpSample(point, -1) }, reference);
        ok &= expectTrue(!result.icpAttempted && !result.icpApplied
                             && result.samples.empty()
                             && result.bufferedSampleCount == static_cast<std::size_t>(index),
                         "three dense new frames must wait for their own two-millimetre span");
    }
    result = processor.process({ makeIcpSample(reference[12], -1) }, reference);
    ok &= expectTrue(result.icpAttempted && result.icpApplied && !result.samples.empty(),
                     "pending dense frames should register after their own span becomes sufficient");
    return ok;
}

bool testExceptionProcessorDoesNotReplayOverlappingModelRange()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    const auto reference = makeIcpReference();
    i2r_rlc::control::OnlineCorrectionExceptionResult result;
    for (std::size_t index = 0; index < 11U; ++index) {
        result = processor.process({ { reference[index], std::nullopt, -1 } }, reference);
    }
    bool ok = expectTrue(result.icpApplied && !result.samples.empty(),
                         "overlap regression requires an applied initial model range");
    if (result.samples.empty()) {
        return false;
    }
    const double previousLastX = result.samples.back().point.xMm;
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> overlapping;
    for (std::size_t index = 9U; index < 14U; ++index) {
        overlapping.push_back({ reference[index], std::nullopt, -1 });
    }
    result = processor.process(overlapping, reference);
    ok &= expectTrue(result.icpApplied && result.samples.size() == 3U,
                     "an overlapping detection batch should emit only the three unreleased model vertices");
    for (const auto &sample : result.samples) {
        ok &= expectTrue(sample.point.xMm > previousLastX + 1.0e-5,
                         "cross-batch model progress must strictly advance beyond the released endpoint");
    }
    for (std::size_t index = 0; index < result.samples.size() && index < 3U; ++index) {
        ok &= expectNear(result.samples[index].point.xMm, reference[index + 11U].xMm,
                         1.0e-5, "identity-transform overlap should retain the known next model vertex");
        ok &= expectNear(result.samples[index].point.yMm, reference[index + 11U].yMm,
                         1.0e-5, "overlap clipping should preserve the model geometry");
    }
    processor.reset();
    for (std::size_t index = 0; index < 11U; ++index) {
        result = processor.process({ { reference[index], std::nullopt, -1 } }, reference);
    }
    ok &= expectTrue(result.icpApplied && !result.samples.empty(),
                     "reset should permit the model start to be registered in a new session");
    if (!result.samples.empty()) {
        ok &= expectNear(result.samples.front().point.xMm, reference.front().xMm, 1.0e-5,
                         "reset must clear the cross-batch model cursor");
    }
    return ok;
}

bool testExceptionProcessorRecoversAfterRawPhaseReanchor()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    std::vector<i2r_rlc::control::OnlineCorrectionPoint> reference;
    constexpr double PI = 3.14159265358979323846;
    for (int index = 0; index <= 120; ++index) {
        const double angle = 2.0 * PI * static_cast<double>(index) / 120.0;
        reference.push_back({ 20.0 * std::cos(angle), 20.0 * std::sin(angle),
                              0.5 * std::sin(2.0 * angle), true });
    }
    i2r_rlc::control::OnlineCorrectionExceptionResult result;
    for (std::size_t index = 0; index < 11U; ++index) {
        result = processor.process({ { reference[index], std::nullopt, -1 } }, reference);
    }
    bool ok = expectTrue(result.icpApplied,
                         "phase reanchor regression requires an established model cursor");
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> skipped;
    for (std::size_t index = 60U; index < 64U; ++index) {
        skipped.push_back({ reference[index], std::nullopt, -1 });
    }
    result = processor.process(skipped, reference);
    ok &= expectTrue(result.icpAttempted && !result.icpApplied
                         && result.samples.size() == skipped.size()
                         && result.message.find("reanchor") != std::string::npos,
                     "a compatible large phase shift should release raw points and reanchor");
    for (std::size_t index = 0; index < result.samples.size() && index < skipped.size(); ++index) {
        ok &= expectNear(result.samples[index].point.xMm, skipped[index].point.xMm,
                         1.0e-12, "phase-shift fallback must preserve every raw point");
    }
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> following;
    for (std::size_t index = 64U; index < 67U; ++index) {
        following.push_back({ reference[index], std::nullopt, -1 });
    }
    result = processor.process(following, reference);
    ok &= expectTrue(result.icpAttempted && result.icpApplied && !result.samples.empty(),
                     "the next nearby batch should recover ICP after raw phase reanchor");
    return ok;
}

bool testExceptionProcessorNearClosedPathDoesNotEmitFullLoop()
{
    std::vector<i2r_rlc::control::OnlineCorrectionPoint> reference;
    constexpr double PI = 3.14159265358979323846;
    for (int index = 0; index <= 120; ++index) {
        const double fraction = static_cast<double>(index) / 120.0;
        const double angle = 2.0 * PI * fraction;
        const double radius = 20.0 + 0.6 * std::sin(3.0 * angle)
            + 0.2 * std::cos(7.0 * angle);
        reference.push_back({ radius * std::cos(angle), radius * std::sin(angle),
                              0.5 * std::sin(2.0 * angle) + 0.02 * fraction, true });
    }
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> shortBatch;
    for (std::size_t index = 4U; index < 14U; ++index) {
        shortBatch.push_back({ reference[index], std::nullopt, -1 });
    }
    shortBatch.push_back({ reference[119], std::nullopt, -1 });
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    const auto result = processor.process(shortBatch, reference);
    bool ok = expectTrue(result.valid && result.icpAttempted,
                         "near-closed-path regression must exercise the real ICP solver");
    ok &= expectTrue(!result.samples.empty() && result.samples.size() <= 20U,
                     "a short batch across a near closure must not expand into almost the entire loop");
    double outputLengthMm = 0.0;
    for (std::size_t index = 1U; index < result.samples.size(); ++index) {
        const auto &first = result.samples[index - 1U].point;
        const auto &last = result.samples[index].point;
        const double dx = last.xMm - first.xMm;
        const double dy = last.yMm - first.yMm;
        const double dz = last.zMm - first.zMm;
        outputLengthMm += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ok &= expectTrue(outputLengthMm < 30.0,
                     "near-closure forward search must respect the short detection batch's arc budget");
    return ok;
}

bool testExceptionProcessorSpanFlushAndReset()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    const auto trajectory = makeIcpTrajectory();
    bool ok = true;
    i2r_rlc::control::OnlineCorrectionExceptionResult result;
    for (int index = 0; index < 30; ++index) {
        const double value = static_cast<double>(index);
        result = processor.process(
            { { { 0.03 * value, 0.002 * value, 0.0, true }, 2.0F, -1 } },
            trajectory,
            false);
        ok &= expectTrue(!result.icpAttempted && result.samples.empty(),
                         "a sub-two-millimetre window should not force geometric registration");
    }
    result = processor.process({}, trajectory, true);
    ok &= expectTrue(result.valid && !result.icpApplied && result.samples.size() == 30U,
                     "tail flush must release all under-spanned raw points without ICP");
    ok &= expectTrue(result.batches.size() == 1U && result.batches.front().flush
                         && !result.batches.front().detectionToModel.has_value()
                         && result.batches.front().releasedSamples.size() == 30U,
                     "raw tail fallback must be recorded without a fictitious transform");
    for (std::size_t index = 0; index < result.samples.size(); ++index) {
        ok &= expectNear(result.samples[index].point.xMm,
                         0.03 * static_cast<double>(index),
                         1.0e-12,
                         "flushed raw points should keep their order and values");
    }
    result = processor.process({}, trajectory, true);
    ok &= expectTrue(result.samples.empty() && result.batches.empty(),
                     "a second flush must not duplicate a tail or diagnostics");

    for (std::size_t index = 0; index < 10U; ++index) {
        processor.process({ makeIcpSample(trajectory.segmentPathPoints[index], 0) },
                          trajectory,
                          false);
    }
    processor.reset();
    result = processor.process({ makeIcpSample(trajectory.segmentPathPoints[10], 0) },
                               trajectory,
                               false);
    ok &= expectTrue(!result.icpAttempted && result.samples.empty()
                         && result.bufferedSampleCount == 1U,
                     "reset must clear history and all pending samples");
    result = processor.process({}, trajectory, true);
    ok &= expectTrue(result.samples.size() == 1U,
                     "reset flush should emit only the new session point");
    return ok;
}

bool testExceptionProcessorKeepsInsufficientSeamSides()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    auto trajectory = makeIcpTrajectory();
    trajectory.pathSegments.push_back(trajectory.segmentPathPoints);
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> emitted;
    bool sawBoundary = false;
    bool ok = true;
    for (int index = 0; index < 7; ++index) {
        const int seam = index < 4 ? 0 : 1;
        auto sample = makeIcpSample(trajectory.segmentPathPoints[index], seam);
        const auto result = processor.process({ sample }, trajectory, false);
        sawBoundary |= result.segmentChanged;
        emitted.insert(emitted.end(), result.samples.begin(), result.samples.end());
        ok &= expectTrue(!result.icpApplied,
                         "undersized seam sides must not be reported as registered");
    }
    const auto flushed = processor.process({}, trajectory, true);
    emitted.insert(emitted.end(), flushed.samples.begin(), flushed.samples.end());
    ok &= expectTrue(sawBoundary, "a known seam index edge should be observable");
    ok &= expectTrue(emitted.size() == 7U,
                     "seam transition and final flush must not lose either undersized side");
    for (std::size_t index = 0; index < emitted.size() && index < 7U; ++index) {
        const int seam = index < 4U ? 0 : 1;
        const auto expected = makeIcpSample(trajectory.segmentPathPoints[index], seam);
        ok &= expectNear(emitted[index].point.xMm, expected.point.xMm, 1.0e-10,
                         "insufficient seam sides should keep raw ordering");
        ok &= expectTrue(emitted[index].simulatedSeamIndex == seam,
                         "seam transition must preserve each sample's seam metadata");
    }
    return ok;
}

bool testExceptionProcessorRetriesAfterSeamChange()
{
    i2r_rlc::control::COnlineCorrectionExceptionProcessor processor;
    auto trajectory = makeIcpTrajectory();
    auto nextSegment = trajectory.segmentPathPoints;
    for (auto &point : nextSegment) {
        point.xMm += 100.0;
        point.yMm -= 50.0;
    }
    trajectory.pathSegments.push_back(nextSegment);
    trajectory.allPathPoints.insert(trajectory.allPathPoints.end(),
                                    nextSegment.begin(), nextSegment.end());
    trajectory.allMatchedPathPoints = trajectory.allPathPoints;
    bool ok = true;
    for (std::size_t index = 0; index < 4U; ++index) {
        processor.process({ makeIcpSample(trajectory.segmentPathPoints[index], 0) },
                          trajectory, false);
    }
    i2r_rlc::control::OnlineCorrectionExceptionResult result;
    for (std::size_t index = 0; index < 11U; ++index) {
        result = processor.process({ makeIcpSample(nextSegment[index], 1) },
                                   trajectory, false);
        if (index == 0U) {
            ok &= expectTrue(result.segmentChanged && result.samples.size() == 4U,
                             "a seam change should release the insufficient preceding side");
            for (const auto &sample : result.samples) {
                ok &= expectTrue(sample.simulatedSeamIndex == 0,
                                 "the released preceding side must keep its seam identity");
            }
        } else if (index < 10U) {
            ok &= expectTrue(result.samples.empty() && !result.icpAttempted,
                             "the new seam must keep accumulating after its first frame");
        }
    }
    ok &= expectTrue(result.icpAttempted && result.icpApplied && !result.samples.empty(),
                     "a seam that was too short at its edge must retry ICP after accumulating");
    for (const auto &sample : result.samples) {
        ok &= expectTrue(sample.simulatedSeamIndex == 1,
                         "retried ICP must emit only the new seam's pending samples");
    }
    return ok;
}

bool enterOnlineCorrection(
    i2r_rlc::control::COnlineCorrectionController *controller,
    i2r_rlc::robot_comm::RobotState *robotState,
    const i2r_rlc::control::OnlineCorrectionPoint &initialPoint)
{
    controller->startInitialDetection();
    i2r_rlc::control::OnlineCorrectionDetectionEvent initial;
    initial.samples.push_back({ initialPoint, std::nullopt });
    initial.detectionFinished = true;
    controller->appendDetectionEvent(initial);
    robotState->onlineReverseCompleteFlag = 1;
    auto update = controller->onRobotStateChanged(*robotState);
    if (!update.command.valid) {
        return false;
    }
    i2r_rlc::robot_comm::RobotCommandReceipt receipt;
    receipt.type = i2r_rlc::robot_comm::RobotCommandType::TeachPointUpdate;
    receipt.sent = true;
    receipt.responseReceived = true;
    receipt.completed = true;
    receipt.pointIndex = update.command.pointCount - 1;
    robotState->offline = 5;
    controller->onRobotCommandReceipt(receipt, *robotState);
    robotState->corflag = 2;
    robotState->detflag = 2;
    controller->onRobotStateChanged(*robotState);
    robotState->corflag = 1;
    robotState->detflag = 1;
    update = controller->onRobotStateChanged(*robotState);
    return update.phase == i2r_rlc::control::OnlineCorrectionPhase::OnlineRunning;
}

bool testControllerOnlineIcpSwitchAndFlush()
{
    auto trajectory = makeIcpTrajectory();
    bool ok = true;
    for (const bool enabled : { false, true }) {
        i2r_rlc::control::COnlineCorrectionController controller;
        i2r_rlc::control::OnlineCorrectionOptions options;
        options.enableOnlineIcp = enabled;
        controller.configure(options, trajectory, makePositionerCalibration());
        i2r_rlc::robot_comm::RobotState robotState;
        ok &= expectTrue(enterOnlineCorrection(
                             &controller, &robotState, trajectory.segmentPathPoints.front()),
                         "ICP switch scenario should pass the real STARTCOR gates");
        for (std::size_t index = 0; index < 11U; ++index) {
            i2r_rlc::control::OnlineCorrectionDetectionEvent event;
            event.samples.push_back(makeIcpSample(trajectory.segmentPathPoints[index], -1));
            const auto update = controller.appendDetectionEvent(event);
            ok &= expectTrue(update.state.onlineIcp.enabled == enabled,
                             "diagnostics should expose the configured online ICP switch");
            if (!enabled) {
                ok &= expectTrue(!update.state.onlineIcp.attempted
                                     && !update.state.onlineIcp.applied
                                     && update.state.onlineIcp.bufferedSampleCount == 0U,
                                 "disabled online ICP must bypass fitting and buffering");
                ok &= expectTrue(update.onlineIcpBatches.size() == 1U
                                     && !update.onlineIcpBatches.front().enabled
                                     && !update.onlineIcpBatches.front().detectionToModel.has_value(),
                                 "disabled ICP must record raw releases without a fake matrix");
            } else if (index < 10U) {
                ok &= expectTrue(!update.state.onlineIcp.attempted
                                     && !update.state.onlineIcp.applied
                                     && update.state.onlineIcp.bufferedSampleCount == index + 1U,
                                 "controller must preserve one-frame ICP accumulation");
                ok &= expectTrue(update.onlineIcpBatches.empty(),
                                 "waiting status must not duplicate a released batch");
            } else {
                ok &= expectTrue(update.state.onlineIcp.attempted && update.state.onlineIcp.applied,
                                 "enabled controller path must really apply ICP");
                ok &= expectTrue(update.onlineIcpBatches.size() == 1U
                                     && update.onlineIcpBatches.front().applied,
                                 "controller must forward the actual ICP batch snapshot");
            }
        }
        i2r_rlc::control::OnlineCorrectionDetectionEvent tail;
        tail.samples.push_back(makeIcpSample(trajectory.segmentPathPoints[11], -1));
        controller.appendDetectionEvent(tail);
        const auto finished = controller.finishOnlineSegment();
        ok &= expectTrue(finished.realtimeCommand.valid
                             && finished.realtimeCommand.onlineEndFlag == 1,
                         "ICP pending tail must preserve the final P end flag");
        ok &= expectTrue(finished.state.onlineIcp.bufferedSampleCount == 0U,
                         "segment finish should flush the pending ICP tail");
    }
    return ok;
}

bool testControllerIcpUsesGlobalAndInitialCorrectionReference()
{
    const auto trajectory = makeIcpTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableOnlineIcp = true;
    options.startMode = i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection;
    options.globalCorrectionRowMajor = {
        0.5, -0.8660254037844386, 0.0, 80.0,
        0.8660254037844386, 0.5, 0.0, 120.0,
        0.0, 0.0, 1.0, 15.0,
        0.0, 0.0, 0.0, 1.0
    };
    controller.configure(options, trajectory, makePositionerCalibration());
    i2r_rlc::robot_comm::RobotState robotState;
    const i2r_rlc::control::OnlineCorrectionPoint initial{
        400.0, -300.0, 25.0, true
    };
    bool ok = expectTrue(enterOnlineCorrection(&controller, &robotState, initial),
                         "corrected-reference scenario should start online correction");
    const auto corrected = controller.state().correctedSegmentPath;
    if (corrected.size() < 11U) {
        return expectTrue(false, "corrected online ICP reference must be complete");
    }
    robotState.curx_r = 400000;
    robotState.cury_r = -300000;
    robotState.curz_r = 25000;
    controller.onRobotStateChanged(robotState);
    i2r_rlc::control::OnlineCorrectionUpdate update;
    for (std::size_t index = 0; index < 11U; ++index) {
        auto point = corrected[index];
        point.xMm += 0.2;
        point.yMm -= 0.3;
        point.zMm += 0.25;
        i2r_rlc::control::OnlineCorrectionDetectionEvent event;
        event.samples.push_back({ point, std::nullopt, -1 });
        update = controller.appendDetectionEvent(event);
    }
    ok &= expectTrue(update.state.onlineIcp.applied,
                     "online ICP must accept the globally rotated, start-aligned reference");
    ok &= expectTrue(update.state.onlineIcp.rmsErrorMm < 0.05,
                     "active ICP reference should require only the small online residual correction");
    ok &= expectTrue(!update.state.onlineDetectionSamples.empty(),
                     "active corrected reference should produce an online model queue");
    for (const auto &sample : update.state.onlineDetectionSamples) {
        double nearest = 1.0e9;
        for (const auto &point : corrected) {
            const double dx = sample.point.xMm - point.xMm - 0.2;
            const double dy = sample.point.yMm - point.yMm + 0.3;
            const double dz = sample.point.zMm - point.zMm - 0.25;
            nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        ok &= expectTrue(nearest < 0.05,
                         "online predictions must stay in the corrected reference coordinates");
    }
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

bool testPathFollowerKeepsForwardTeachCursor()
{
    i2r_rlc::control::COnlineCorrectionPathFollower follower;
    std::vector<i2r_rlc::control::OnlineCorrectionPoint> teachTrajectory;
    for (int index = 0; index <= 150; ++index) {
        teachTrajectory.push_back({ static_cast<double>(index), 0.0, 0.0, true });
    }
    for (int index = 149; index >= 0; --index) {
        teachTrajectory.push_back({ static_cast<double>(index), 0.0, 0.0, true });
    }

    i2r_rlc::robot_comm::RobotState robotState;
    robotState.curx_r = 90000;
    robotState.cury_r = 0;
    robotState.curz_r = 0;
    auto firstSamples = makeFollowerSamples(91.0, 1.0);
    const auto first = follower.update(
        robotState,
        i2r_rlc::control::OnlineCorrectionOptions{},
        makePositionerCalibration(),
        teachTrajectory,
        &firstSamples,
        false);

    robotState.curx_r = 11000;
    std::vector<i2r_rlc::control::OnlineCorrectionDetectionSample> secondSamples{
        { { 10.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { 0.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { -10.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { -20.0, 1.0, 0.0, true }, std::nullopt, 0 }
    };
    const auto second = follower.update(
        robotState,
        i2r_rlc::control::OnlineCorrectionOptions{},
        makePositionerCalibration(),
        teachTrajectory,
        &secondSamples,
        false);

    bool ok = true;
    ok &= expectTrue(first.valid && second.valid,
                     "forward teach cursor scenario should generate commands");
    ok &= expectTrue(first.taughtPointIndex >= 80 && first.taughtPointIndex <= 100,
                     "first teach match should be near the current forward branch");
    ok &= expectTrue(second.taughtPointIndex >= first.taughtPointIndex,
                     "teach match should not jump back to an earlier branch");
    return ok;
}

bool testControllerUsesCorrectedOnlineDirectionReference()
{
    auto trajectory = makeTrajectory();
    i2r_rlc::control::COnlineCorrectionController controller;
    i2r_rlc::control::OnlineCorrectionOptions options;
    options.enableOnlineIcp = false;
    controller.configure(options, trajectory, makePositionerCalibration());
    controller.startInitialDetection();

    i2r_rlc::control::OnlineCorrectionDetectionEvent initial;
    initial.samples.push_back({ { 100.0, 0.0, 0.0, true }, std::nullopt });
    initial.detectionFinished = true;
    controller.appendDetectionEvent(initial);

    i2r_rlc::robot_comm::RobotState robotState;
    robotState.onlineReverseCompleteFlag = 1;
    auto update = controller.onRobotStateChanged(robotState);
    i2r_rlc::robot_comm::RobotCommandReceipt receipt;
    receipt.type = i2r_rlc::robot_comm::RobotCommandType::TeachPointUpdate;
    receipt.sent = true;
    receipt.responseReceived = true;
    receipt.completed = true;
    receipt.pointIndex = 1;
    robotState.offline = 5;
    controller.onRobotCommandReceipt(receipt, robotState);

    robotState.corflag = 2;
    robotState.detflag = 2;
    controller.onRobotStateChanged(robotState);
    robotState.corflag = 1;
    robotState.detflag = 1;
    update = controller.onRobotStateChanged(robotState);

    i2r_rlc::control::OnlineCorrectionDetectionEvent online;
    online.samples = {
        { { 101.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { 111.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { 121.0, 1.0, 0.0, true }, std::nullopt, 0 },
        { { 131.0, 1.0, 0.0, true }, std::nullopt, 0 }
    };
    robotState.curx_r = 100000;
    controller.onRobotStateChanged(robotState);
    update = controller.appendDetectionEvent(online);

    bool ok = true;
    ok &= expectTrue(update.phase == i2r_rlc::control::OnlineCorrectionPhase::OnlineRunning,
                     "controller should be online before tracking reference check");
    ok &= expectTrue(update.taughtPointIndex == 0,
                     "online direction should use the corrected segment reference");
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
    ok &= expectTrue(result.diagnostic.valid,
                     "a generated P must expose its actual follow geometry");
    ok &= expectNear(result.diagnostic.first.xMm, 10.0, 1.0e-9,
                     "diagnostic first point must be the actual queue target");
    ok &= expectNear(result.diagnostic.directionPoint.xMm, 40.0, 1.0e-9,
                     "diagnostic direction point must be the actual line endpoint");
    ok &= expectNear(result.diagnostic.foot.yMm, 1.0, 1.0e-9,
                     "diagnostic foot must preserve the actual detection line offset");
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
    robotState.curRx_r = 120000;

    auto plan = i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        trajectory, options, state, robotState);

    bool ok = true;
    ok &= expectTrue(plan.valid, "IBVS mode should use robot current point");
    ok &= expectTrue(plan.command.teachPoints[0].xUm == 21000, "IBVS x should come from robot state");
    ok &= expectTrue(plan.command.teachPoints[0].yUm == 22000, "IBVS y should come from robot state");
    ok &= expectTrue(plan.command.teachPoints[0].zUm == 23000, "IBVS z should come from robot state");
    ok &= expectTrue(plan.command.teachPoints[0].rxDeg10000 == 10000,
                     "IBVS orientation should retain its existing template source");
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

int main(int argc, char *argv[])
{
    // 单独证明首点合同；默认入口仍执行全部回归，不隐藏既有失败。
    if (argc == 2 && std::string(argv[1]) == "--p001-contract") {
        bool ok = testTrajectoryParsingAndCommandUnits();
        ok &= testReverseP001OrientationChangesOnlyFirstTeachPoint();
        ok &= testReverseCompletionRequiresFreshEdge();
        ok &= testIbvsStartModeUsesRobotState();
        ok &= testGlobalCorrectionModeAppliesTransform();
        ok &= testInitTypeDoesNotApplyGlobalMatrix();
        if (!ok) {
            return 1;
        }
        std::cout << "PASS: P001 source and compatibility contract tests\n";
        return 0;
    }

    bool ok = true;
    ok &= testTrajectoryParsingAndCommandUnits();
    ok &= testTrajectoryIcpSegmentsFollowAdjacentTeachPoints();
    ok &= testReverseP001OrientationChangesOnlyFirstTeachPoint();
    ok &= testControllerStateFlow();
    ok &= testControllerRejectsMissingCalibration();
    ok &= testReverseCompletionRequiresFreshEdge();
    ok &= testTeachPointBatchKeepsLegacyMissingStationAxes();
    ok &= testPositionerRoundTrip();
    ok &= testRbcalibGoldenTranslation();
    ok &= testExceptionProcessorAccumulatesSingleFramesAndAppliesIcp();
    ok &= testExceptionProcessorWaitsForDensePendingSpan();
    ok &= testExceptionProcessorDoesNotReplayOverlappingModelRange();
    ok &= testExceptionProcessorRecoversAfterRawPhaseReanchor();
    ok &= testExceptionProcessorNearClosedPathDoesNotEmitFullLoop();
    ok &= testExceptionProcessorSpanFlushAndReset();
    ok &= testExceptionProcessorKeepsInsufficientSeamSides();
    ok &= testExceptionProcessorRetriesAfterSeamChange();
    ok &= testControllerOnlineIcpSwitchAndFlush();
    ok &= testControllerIcpUsesGlobalAndInitialCorrectionReference();
    ok &= testPathFollowerRemovesRepeatedTarget();
    ok &= testPathFollowerKeepsDirectionReversalLikeLegacyActiveCode();
    ok &= testPathFollowerKeepsForwardTeachCursor();
    ok &= testControllerUsesCorrectedOnlineDirectionReference();
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
