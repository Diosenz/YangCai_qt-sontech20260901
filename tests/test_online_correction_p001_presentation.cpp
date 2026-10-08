/**
 * @file        test_online_correction_p001_presentation.cpp
 * @brief       在线纠偏最终 P001 UI 展示状态测试
 */
#include <iostream>

#include "OnlineCorrectionP001Presentation.h"

namespace {

bool expectTrue(const bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

i2r_rlc::vision::PositionerCalibration makeCalibration()
{
    i2r_rlc::vision::PositionerCalibration calibration;
    calibration.valid = true;
    return calibration;
}

i2r_rlc::control::OnlineCorrectionUpdate makeP001Update()
{
    i2r_rlc::control::OnlineCorrectionUpdate update;
    update.phase =
        i2r_rlc::control::OnlineCorrectionPhase::SendingInitialTeachPoints;
    update.command.valid = true;
    update.command.pointCount = 1;

    i2r_rlc::robot_comm::TeachPointUpdateCommand point;
    point.xUm = 123456;
    point.yUm = -234567;
    point.zUm = 345678;
    point.rxDeg10000 = 123456;
    point.ryDeg10000 = -234567;
    point.rzDeg10000 = 345678;
    point.valid = true;
    update.command.teachPoints.push_back(point);

    i2r_rlc::robot_comm::StationAxisUpdateCommand axis;
    axis.s1Deg10000 = 456789;
    axis.s2Deg10000 = -567891;
    axis.valid = true;
    update.command.stationAxes.push_back(axis);
    return update;
}

bool testUnitConversionAndLatch()
{
    i2r_rlc::ui::COnlineCorrectionP001Presentation presentation;
    bool ok = expectTrue(
        presentation.text() == QStringLiteral("P001：未计算"),
        "initial state must not show zero coordinates");

    auto malformed = makeP001Update();
    malformed.command.stationAxes.clear();
    presentation.update(malformed, makeCalibration());
    ok &= expectTrue(
        presentation.text() == QStringLiteral("P001：未计算"),
        "missing station axis must not latch P001");

    malformed = makeP001Update();
    malformed.command.teachPoints.clear();
    presentation.update(malformed, makeCalibration());
    ok &= expectTrue(
        presentation.text() == QStringLiteral("P001：未计算"),
        "missing teach point must not latch P001");

    presentation.update(makeP001Update(), makeCalibration());
    const QString generated = presentation.text();
    ok &= expectTrue(generated.contains(QStringLiteral("X = 123.456 mm")),
                     "x must convert um to mm with three decimals");
    ok &= expectTrue(generated.contains(QStringLiteral("Y = -234.567 mm")),
                     "signed y must convert um to mm");
    ok &= expectTrue(generated.contains(QStringLiteral("Z = 345.678 mm")),
                     "z must convert um to mm with three decimals");
    ok &= expectTrue(generated.contains(QStringLiteral("RX = 12.3456 deg")),
                     "rx must convert deg10000 to degrees");
    ok &= expectTrue(generated.contains(QStringLiteral("RY = -23.4567 deg")),
                     "signed ry must convert deg10000 to degrees");
    ok &= expectTrue(generated.contains(QStringLiteral("RZ = 34.5678 deg")),
                     "rz must convert deg10000 to degrees");
    ok &= expectTrue(generated.contains(QStringLiteral("S1 = 45.6789 deg")),
                     "s1 must convert deg10000 to degrees");
    ok &= expectTrue(generated.contains(QStringLiteral("S2 = -56.7891 deg")),
                     "s2 must convert deg10000 to degrees");
    ok &= expectTrue(generated.contains(QStringLiteral("状态 = 已生成待确认")),
                     "generated P001 must wait for confirmation");

    i2r_rlc::control::OnlineCorrectionUpdate emptyUpdate;
    emptyUpdate.phase =
        i2r_rlc::control::OnlineCorrectionPhase::WaitingInitialTeachPointAck;
    presentation.update(emptyUpdate, makeCalibration());
    ok &= expectTrue(presentation.text() == generated,
                     "empty command update must preserve latched P001");

    auto laterCommand = makeP001Update();
    laterCommand.command.teachPoints.front().xUm = 999999;
    presentation.update(laterCommand, makeCalibration());
    ok &= expectTrue(presentation.text() == generated,
                     "later command must not replace the first latched P001");
    return ok;
}

bool testWrittenFailedAndClearStates()
{
    i2r_rlc::ui::COnlineCorrectionP001Presentation presentation;
    presentation.update(makeP001Update(), makeCalibration());

    i2r_rlc::control::OnlineCorrectionUpdate writtenUpdate;
    writtenUpdate.phase =
        i2r_rlc::control::OnlineCorrectionPhase::WaitingForInitialApproach;
    presentation.update(writtenUpdate, makeCalibration());
    const QString written = presentation.text();
    bool ok = expectTrue(written.contains(QStringLiteral("状态 = 已写入")),
                         "successful O confirmation must show written");
    ok &= expectTrue(written.contains(QStringLiteral("X = 123.456 mm")),
                     "written state must preserve coordinates");

    presentation.clear();
    presentation.update(makeP001Update(), makeCalibration());
    i2r_rlc::control::OnlineCorrectionUpdate failedUpdate;
    failedUpdate.succeeded = false;
    failedUpdate.phase = i2r_rlc::control::OnlineCorrectionPhase::Error;
    presentation.update(failedUpdate, makeCalibration());
    const QString failed = presentation.text();
    ok &= expectTrue(failed.contains(QStringLiteral("状态 = 写入失败")),
                     "failed O confirmation must show write failure");
    ok &= expectTrue(failed.contains(QStringLiteral("S2 = -56.7891 deg")),
                     "write failure must preserve coordinates");

    i2r_rlc::control::OnlineCorrectionUpdate newSession;
    newSession.phase =
        i2r_rlc::control::OnlineCorrectionPhase::InitialDetectionRunning;
    presentation.update(newSession, makeCalibration());
    ok &= expectTrue(
        presentation.text() == QStringLiteral("P001：未计算"),
        "new session must clear the previous P001");

    presentation.update(makeP001Update(), makeCalibration());
    presentation.clear();
    ok &= expectTrue(
        presentation.text() == QStringLiteral("P001：未计算"),
        "explicit stop or reset must clear the previous P001");
    return ok;
}

bool testFixedStationBasePoseAndCalibrationLatch()
{
    constexpr double pi = 3.14159265358979323846;
    auto calibration = makeCalibration();
    // 文件位姿为变位机基座在机器人基座中的位姿，转换使用其逆矩阵。
    const Eigen::Affine3d stationInRobot =
        Eigen::Translation3d(100.0, 200.0, 300.0)
        * Eigen::AngleAxisd(pi / 6.0, Eigen::Vector3d::UnitZ());
    calibration.stationBaseInRobotBase = stationInRobot.inverse();
    calibration.toolTcpInLastLinkMm = Eigen::Vector3d(0.0, 0.0, 231.0);

    auto update = makeP001Update();
    auto &point = update.command.teachPoints.front();
    point.xUm = 101000;
    point.yUm = 201732;
    point.zUm = 303000;
    point.rxDeg10000 = 100000;
    point.ryDeg10000 = 200000;
    point.rzDeg10000 = 400000;

    i2r_rlc::ui::COnlineCorrectionP001Presentation presentation;
    presentation.update(update, calibration);
    const QString generated = presentation.text();
    bool ok = expectTrue(generated.contains(QStringLiteral(
        "P001（变位机固定基座 StationBase） X = 1.732 mm  Y = 1.000 mm  Z = 3.000 mm")),
        "fixed station base must use inverse installation pose without EX axes or TCP offset");
    ok &= expectTrue(generated.contains(QStringLiteral(
        "RX = 10.0000 deg  RY = 20.0000 deg  RZ = 10.0000 deg")),
        "orientation must be expressed in the fixed station base");
    ok &= expectTrue(generated.contains(QStringLiteral(
        "P001（RobotBase） X = 101.000 mm  Y = 201.732 mm  Z = 303.000 mm")),
        "station display must preserve the original robot base target");
    ok &= expectTrue(point.xUm == 101000 && point.rzDeg10000 == 400000,
        "display conversion must not mutate the command");

    i2r_rlc::control::OnlineCorrectionUpdate emptyUpdate;
    emptyUpdate.phase =
        i2r_rlc::control::OnlineCorrectionPhase::WaitingInitialTeachPointAck;
    presentation.update(emptyUpdate, makeCalibration());
    ok &= expectTrue(presentation.text() == generated,
        "later calibration and empty commands must preserve both latched poses");
    presentation.clear();
    presentation.update(update, makeCalibration());
    ok &= expectTrue(presentation.text().contains(QStringLiteral(
        "P001（变位机固定基座 StationBase） X = 101.000 mm  Y = 201.732 mm  Z = 303.000 mm")),
        "a new session must use its own calibration");
    return ok;
}

bool testInvalidStationCalibrationPreservesRobotPose()
{
    i2r_rlc::ui::COnlineCorrectionP001Presentation presentation;
    i2r_rlc::vision::PositionerCalibration invalidCalibration;
    presentation.update(makeP001Update(), invalidCalibration);
    const QString text = presentation.text();
    bool ok = expectTrue(text.contains(QStringLiteral(
        "P001（变位机固定基座 StationBase）：不可用（RBCALIB 标定无效）")),
        "invalid calibration must not display fabricated station coordinates");
    ok &= expectTrue(text.contains(QStringLiteral("X = 123.456 mm")),
        "invalid station calibration must preserve the robot base display");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testUnitConversionAndLatch();
    ok &= testWrittenFailedAndClearStates();
    ok &= testFixedStationBasePoseAndCalibrationLatch();
    ok &= testInvalidStationCalibrationPreservesRobotPose();
    if (!ok) {
        return 1;
    }
    std::cout << "PASS: online correction P001 presentation tests\n";
    return 0;
}
