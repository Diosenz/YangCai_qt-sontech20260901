/**
 * @file test_online_correction_coordinate_contract.cpp
 * @brief 验证视觉 TCP 点进入在线控制后的坐标、门禁及会话标定契约。
 * @details 仅运行离线计算。期望位置采用闭式旋转和手算值，不调用生产逆变换生成期望。
 */
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include <QCoreApplication>

#include "OnlineCorrectionController.h"
#include "RuntimeWorkerAdapters.h"
#include "geometry/WeldImageToSpaceProjector.h"
#include "geometry/WeldProjectionContextBuilder.h"

namespace {
namespace control = i2r_rlc::control;
namespace robot = i2r_rlc::robot_comm;
namespace ui = i2r_rlc::ui;
namespace vision = i2r_rlc::vision;

bool check(bool value, const char *message)
{
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return value;
}

bool near(double actual, double expected)
{
    return std::isfinite(actual) && std::abs(actual - expected) < 1.0e-8;
}

control::OnlineCorrectionPlannedTrajectory trajectory()
{
    control::OnlineCorrectionPlannedTrajectory value;
    value.valid = true;
    value.startTeachIndex = 0;
    value.endTeachIndex = 1;
    value.allPathPoints = {{0, 0, 0, true}, {50, 0, 0, true}, {100, 0, 0, true}};
    value.segmentPathPoints = value.allPathPoints;
    value.allMatchedPathPoints = value.allPathPoints;
    value.teachPoses = {{0, 0, 0, 0, 0, 0, true}, {100, 0, 0, 0, 0, 0, true}};
    value.stationAxes = {{0, 0, true}, {0, 0, true}};
    return value;
}

vision::PositionerCalibration calibration(const Eigen::Vector3d &offset, bool rotated)
{
    vision::PositionerCalibration value;
    value.valid = true;
    value.toolTcpInLastLinkMm = offset;
    Eigen::Matrix3d mount = Eigen::Matrix3d::Identity();
    if (rotated) {
        // 固定安装 Rz(90)，动态外轴 Ry(90)*Rz(90)。
        mount << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    }
    value.stationBaseInRobotBase.linear() = mount.transpose();
    value.stationBaseInRobotBase.translation() =
        -mount.transpose() * Eigen::Vector3d(100, 200, 300);
    return value;
}

robot::RobotState feedback(bool rotated)
{
    robot::RobotState value;
    value.curx_r = 100000;
    value.cury_r = 200000;
    value.curz_r = 300000;
    if (rotated) {
        value.curS1 = 900000;
        value.curS2 = 900000;
        value.curRx_r = 900000;
        value.curRz_r = 1800000;
    }
    value.sStart = 1;
    return value;
}

vision::WeldSensorCalibration sensor()
{
    vision::WeldSensorCalibration value;
    value.valid = true;
    value.focalLengthM = 0.01;
    value.sxM = value.syM = 0.00001;
    value.cxPixel = 160;
    value.cyPixel = 120;
    value.laserPlaneC = -1;
    value.cameraZInToolM = -1; // 激光平面 z=1000 mm，平移后位于工具 z=0。
    return value;
}

control::OnlineCorrectionDetectionEvent projectedEvent(
    const vision::PositionerCalibration &cal,
    const robot::RobotState &state, double x, double y)
{
    const auto built = vision::CWeldProjectionContextBuilder::build(sensor(), cal, state);
    const auto projected = vision::CWeldImageToSpaceProjector::projectImagePoint(
        {160 + x, 120 + y, true}, built.context,
        vision::WeldProjectionTargetFrame::PositionerTcp);
    control::OnlineCorrectionDetectionEvent event;
    event.samples.push_back({{projected.spacePoint.xMm, projected.spacePoint.yMm,
                              projected.spacePoint.zMm, projected.valid}, std::nullopt});
    return event;
}

robot::RobotCommandReceipt receipt(robot::RobotCommandType type, int index)
{
    robot::RobotCommandReceipt value;
    value.type = type;
    value.pointIndex = index;
    value.sent = value.responseReceived = value.completed = true;
    return value;
}

/** @brief 每种起点模式走完整启动握手，确认初始点只转换一次。 */
bool arm(control::COnlineCorrectionController &controller,
         const vision::PositionerCalibration &cal, bool rotated,
         control::OnlineCorrectionStartMode mode)
{
    auto path = trajectory();
    if (rotated) {
        path.stationAxes = {{90, 90, true}, {90, 90, true}};
    }
    control::OnlineCorrectionOptions options;
    options.startMode = mode;
    controller.configure(options, path, cal);
    auto state = feedback(rotated);
    controller.onRobotStateChanged(state);
    bool ok = check(controller.startInitialDetection().startTracking, "initial detection starts");
    auto initial = projectedEvent(cal, state, 0, 0);
    initial.detectionFinished = true;
    auto update = controller.appendDetectionEvent(initial);
    ok &= check(!update.realtimeCommand.valid, "initial phase emits no ordinary P");
    ok &= check(near(controller.state().initialDetectionSamples.front().point.zMm,
                     -cal.toolTcpInLastLinkMm.z()), "initial cache remains TCP coordinates");
    state.onlineReverseCompleteFlag = 1;
    update = controller.onRobotStateChanged(state);
    ok &= check(update.command.valid && update.command.teachPoints.front().xUm == 100000
                    && update.command.teachPoints.front().yUm == 200000
                    && update.command.teachPoints.front().zUm == 300000,
                "P001 equals hand-calculated RobotBase point with one offset conversion");
    state.offline = 5;
    update = controller.onRobotCommandReceipt(receipt(robot::RobotCommandType::TeachPointUpdate, 1), state);
    ok &= check(update.phase == control::OnlineCorrectionPhase::WaitingForInitialApproach
                    && !update.startTracking, "O acknowledgement waits for P001 approach");
    ok &= check(!controller.appendDetectionEvent(initial).realtimeCommand.valid,
                "no ordinary P before STARTCOR");
    state.corflag = state.detflag = 2;
    update = controller.onRobotStateChanged(state);
    ok &= check(update.phase == control::OnlineCorrectionPhase::WaitingForOnlineStart,
                "first ENDDET is an approach boundary");
    state.corflag = state.detflag = 1;
    update = controller.onRobotStateChanged(state);
    ok &= check(update.phase == control::OnlineCorrectionPhase::OnlineRunning
                    && update.startTracking, "STARTCOR opens online control window");
    return ok;
}

bool testGeometryAndLifecycle()
{
    bool ok = true;
    for (const auto offset : {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 231),
                              Eigen::Vector3d(7, -11, 231)}) {
        for (const bool rotated : {false, true}) {
            for (const auto mode : {control::OnlineCorrectionStartMode::InitType,
                                   control::OnlineCorrectionStartMode::GlobalCorrection,
                                   control::OnlineCorrectionStartMode::IBVSStart}) {
                const auto cal = calibration(offset, rotated);
                control::COnlineCorrectionController controller;
                ok &= arm(controller, cal, rotated, mode);
                for (const double deviation : {0.0, 1.0, -1.0}) {
                    // 新控制历史才应在零几何误差时立即输出零。
                    controller.reset();
                    ok &= arm(controller, cal, rotated, mode);
                    control::OnlineCorrectionUpdate update;
                    for (int index = 1; index <= 3; ++index) {
                        const double x = 50 + index * 10;
                        const auto event = projectedEvent(cal, feedback(rotated), x, deviation);
                        update = controller.appendDetectionEvent(event); // 生产每次只有一个样本。
                        ok &= check(near(event.samples.front().point.zMm, -offset.z()),
                                    "input DTO remains in TCP frame");
                        const auto &queue = controller.state().onlineDetectionSamples;
                        ok &= check(!queue.empty() && near(queue.back().point.xMm, x)
                                        && near(queue.back().point.yMm, deviation)
                                        && near(queue.back().point.zMm, 0),
                                    "ICP fallback/cache uses last-link coordinates exactly once");
                        if (index == 1) {
                            ok &= check(!update.realtimeCommand.valid, "one sample waits for direction");
                        }
                    }
                    const auto &command = update.realtimeCommand;
                    ok &= check(command.valid, "single-sample events accumulate to a valid P");
                    if (deviation == 0) {
                        ok &= check(command.dxUm == 0 && command.dyUm == 0 && command.dzUm == 0,
                                    "zero physical deviation produces zero correction");
                    } else {
                        const long component = rotated ? command.dzUm : command.dyUm;
                        ok &= check(component * deviation > 0,
                                    "positive/negative physical deviation has correct RobotBase direction");
                    }
                    ok &= check(command.drxDeg10000 == 0 && command.dryDeg10000 == 0
                                    && command.drzDeg10000 == 0,
                                "coordinate conversion does not introduce orientation correction");
                }
                auto state = feedback(rotated);
                state.corflag = state.detflag = 3;
                auto update = controller.onRobotStateChanged(state);
                ok &= check(update.realtimeCommand.valid && update.realtimeCommand.onlineEndFlag == 1,
                            "final ENDDET generates final P");
                ok &= check(!controller.appendDetectionEvent(projectedEvent(cal, state, 50, 1)).realtimeCommand.valid,
                            "finishing rejects new ordinary P");
                controller.onRobotCommandReceipt(receipt(robot::RobotCommandType::RealtimeCorrection, 0), state);
                state.detflag = 0;
                update = controller.onRobotStateChanged(state);
                ok &= check(update.phase == control::OnlineCorrectionPhase::WaitingForRobotProgramStop,
                            "detection reset still waits for robot program stop");
                state.sStart = 0;
                update = controller.onRobotStateChanged(state);
                ok &= check(update.phase == control::OnlineCorrectionPhase::Completed,
                            "program stop completes lifecycle");
                controller.reset();
                ok &= check(controller.state().onlineDetectionSamples.empty()
                                && controller.state().initialDetectionSamples.empty(), "reset clears sample caches");
            }
        }
    }
    return ok;
}

bool testInvalidInputs()
{
    bool ok = true;
    const auto good = calibration(Eigen::Vector3d(0, 0, 231), false);
    for (int kind = 0; kind < 3; ++kind) {
        auto invalid = good;
        if (kind == 0) invalid.valid = false;
        if (kind == 1) invalid.toolTcpInLastLinkMm.x() = std::numeric_limits<double>::quiet_NaN();
        if (kind == 2) invalid.stationBaseInRobotBase.matrix()(0, 0) = std::numeric_limits<double>::infinity();
        control::COnlineCorrectionController controller;
        controller.configure({}, trajectory(), invalid);
        const auto update = controller.startInitialDetection();
        ok &= check(!update.succeeded && !update.startTracking && !update.command.valid
                        && !update.realtimeCommand.valid, "invalid/nonfinite calibration rejects start");
    }
    for (const bool initial : {false, true}) {
        for (int kind = 0; kind < 3; ++kind) {
            control::COnlineCorrectionController controller;
            if (initial) {
                controller.configure({}, trajectory(), good);
                controller.startInitialDetection();
            } else {
                ok &= arm(controller, good, false, control::OnlineCorrectionStartMode::InitType);
            }
            auto event = projectedEvent(good, feedback(false), 10, 0);
            if (kind == 0) event.samples.front().point.valid = false;
            if (kind == 1) event.samples.front().point.xMm = std::numeric_limits<double>::quiet_NaN();
            if (kind == 2) event.samples.front().point.zMm = std::numeric_limits<double>::infinity();
            const auto update = controller.appendDetectionEvent(event);
            const auto &queue = initial ? controller.state().initialDetectionSamples
                                        : controller.state().onlineDetectionSamples;
            ok &= check(queue.empty() && !update.realtimeCommand.valid,
                        "invalid/NaN/Inf sample cannot enter geometric state or produce ordinary P");
        }
    }
    return ok;
}

/** @brief 文件在分段间变更时，本轮标定必须保持；显式新配置才采用下一版本。 */
bool testWorkerCalibrationSnapshot(QCoreApplication &app)
{
    const auto dir = std::filesystem::temp_directory_path() / "i2r_rlc_coordinate_contract";
    std::filesystem::create_directories(dir);
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.options.startMode = control::OnlineCorrectionStartMode::IBVSStart;
    configuration.trajectoryRequest.workPathFile = (dir / "snapshot_W.txt").string();
    configuration.trajectoryRequest.teachPoseFile = (dir / "snapshot_T.txt").string();
    configuration.rbcalibFile = (dir / "snapshot_RBCALIB.DAT").string();
    std::ofstream(configuration.trajectoryRequest.workPathFile) << "0 0 0\n100 0 0\n200 0 0\n";
    std::ofstream(configuration.trajectoryRequest.teachPoseFile)
        << "0 0 0 0 0 0 0 0 0\n100 0 0 0 0 0 0 0 0\n"
           "100 0 0 0 0 0 0 0 1\n200 0 0 0 0 0 0 0 1\n";
    std::ofstream(configuration.rbcalibFile) << "////SRANG 0,0,0,0,0,0\n";
    ui::COnlineCorrectionWorkerAdapter worker;
    control::OnlineCorrectionUpdate last;
    QObject::connect(&worker, &ui::COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
                     &app, [&](const control::OnlineCorrectionUpdate &update) { last = update; });
    bool ok = check(worker.configureForStart(configuration), "snapshot A configures worker");
    auto state = robot::RobotState{};
    worker.startInitialDetection();
    state.onlineReverseCompleteFlag = 1;
    worker.updateRobotState(state);
    state.offline = 5;
    worker.handleRobotCommandReceipt(receipt(robot::RobotCommandType::TeachPointUpdate, 1), state);
    state.corflag = state.detflag = 2;
    worker.updateRobotState(state);
    state.corflag = state.detflag = 1;
    worker.updateRobotState(state);
    std::ofstream(configuration.rbcalibFile) << "////SRANG 100,0,0,0,0,0\n";
    state.corflag = state.detflag = 2;
    worker.updateRobotState(state);
    worker.handleRobotCommandReceipt(receipt(robot::RobotCommandType::RealtimeCorrection, 0), state);
    state.detflag = 0;
    worker.updateRobotState(state);
    ok &= check(last.phase == control::OnlineCorrectionPhase::InitialDetectionRunning
                    && last.state.segmentIndex == 1, "next segment starts in same session");
    state.onlineReverseCompleteFlag = 0;
    worker.updateRobotState(state);
    state.onlineReverseCompleteFlag = 1;
    worker.updateRobotState(state);
    ok &= check(last.command.valid && near(last.state.correctedSegmentPath.front().xMm, 0),
                "next segment retains calibration A after file changes to B");
    worker.reset();
    ok &= check(worker.configureForStart(configuration), "explicit new configuration loads B");
    worker.startInitialDetection();
    state.onlineReverseCompleteFlag = 0;
    worker.updateRobotState(state);
    state.onlineReverseCompleteFlag = 1;
    worker.updateRobotState(state);
    ok &= check(last.command.valid && near(last.state.correctedSegmentPath.front().xMm, -100),
                "new session explicitly adopts calibration B");
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    bool ok = testGeometryAndLifecycle();
    ok &= testInvalidInputs();
    ok &= testWorkerCalibrationSnapshot(app);
    std::cout << (ok ? "coordinate contract GREEN\n" : "coordinate contract RED\n");
    return ok ? 0 : 1;
}
