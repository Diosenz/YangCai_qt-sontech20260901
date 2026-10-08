/**
 * @file        OnlineCorrectionP001Presentation.cpp
 * @brief       在线纠偏最终 P001 的 UI 展示状态实现
 */
#include "OnlineCorrectionP001Presentation.h"

namespace i2r_rlc::ui {

void COnlineCorrectionP001Presentation::clear()
{
    m_teachPoint = {};
    m_stationAxis = {};
    m_stationBasePose = {};
    m_writeState = WriteState::NotCalculated;
    m_hasP001 = false;
}

void COnlineCorrectionP001Presentation::update(
    const control::OnlineCorrectionUpdate &update,
    const vision::PositionerCalibration &calibration)
{
    if (update.phase == control::OnlineCorrectionPhase::Idle
        || update.phase
            == control::OnlineCorrectionPhase::InitialDetectionRunning) {
        clear();
        return;
    }

    const auto &command = update.command;
    const bool hasValidP001 = command.valid
        && command.pointCount > 0
        && !command.teachPoints.empty()
        && !command.stationAxes.empty()
        && command.teachPoints.front().valid
        && command.stationAxes.front().valid;
    if (!m_hasP001 && hasValidP001) {
        m_teachPoint = command.teachPoints.front();
        m_stationAxis = command.stationAxes.front();
        const vision::PositionerPose robotPose{
            static_cast<double>(m_teachPoint.xUm) / 1000.0,
            static_cast<double>(m_teachPoint.yUm) / 1000.0,
            static_cast<double>(m_teachPoint.zUm) / 1000.0,
            static_cast<double>(m_teachPoint.rxDeg10000) / 10000.0,
            static_cast<double>(m_teachPoint.ryDeg10000) / 10000.0,
            static_cast<double>(m_teachPoint.rzDeg10000) / 10000.0,
            true};
        // 外轴置零只用于取得固定基座变换，不改变实际 EX001 或机器人目标。
        m_stationBasePose = vision::CPositionerKinematics::robotPoseToStationPose(
            robotPose, 0.0, 0.0, calibration);
        m_writeState = WriteState::PendingConfirmation;
        m_hasP001 = true;
    }

    if (!m_hasP001) {
        return;
    }
    if (m_writeState == WriteState::PendingConfirmation
        && update.phase
            == control::OnlineCorrectionPhase::WaitingForInitialApproach) {
        m_writeState = WriteState::Written;
    } else if (m_writeState == WriteState::PendingConfirmation
               && !update.succeeded
               && update.phase == control::OnlineCorrectionPhase::Error) {
        m_writeState = WriteState::WriteFailed;
    }
}

QString COnlineCorrectionP001Presentation::text() const
{
    if (!m_hasP001) {
        return QStringLiteral("P001：未计算");
    }

    QString stateText;
    switch (m_writeState) {
    case WriteState::PendingConfirmation:
        stateText = QStringLiteral("已生成待确认");
        break;
    case WriteState::Written:
        stateText = QStringLiteral("已写入");
        break;
    case WriteState::WriteFailed:
        stateText = QStringLiteral("写入失败");
        break;
    case WriteState::NotCalculated:
        stateText = QStringLiteral("未计算");
        break;
    }

    QString result = QStringLiteral(
               "P001（RobotBase） X = %1 mm  Y = %2 mm  Z = %3 mm\n"
               "RX = %4 deg  RY = %5 deg  RZ = %6 deg  "
               "S1 = %7 deg  S2 = %8 deg  状态 = %9")
        .arg(static_cast<double>(m_teachPoint.xUm) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(m_teachPoint.yUm) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(m_teachPoint.zUm) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(m_teachPoint.rxDeg10000) / 10000.0, 0, 'f', 4)
        .arg(static_cast<double>(m_teachPoint.ryDeg10000) / 10000.0, 0, 'f', 4)
        .arg(static_cast<double>(m_teachPoint.rzDeg10000) / 10000.0, 0, 'f', 4)
        .arg(static_cast<double>(m_stationAxis.s1Deg10000) / 10000.0, 0, 'f', 4)
        .arg(static_cast<double>(m_stationAxis.s2Deg10000) / 10000.0, 0, 'f', 4)
        .arg(stateText);

    if (!m_stationBasePose.valid) {
        return result + QStringLiteral(
            "\nP001（变位机固定基座 StationBase）：不可用（RBCALIB 标定无效）");
    }

    result += QStringLiteral(
                  "\nP001（变位机固定基座 StationBase） "
                  "X = %1 mm  Y = %2 mm  Z = %3 mm\n"
                  "RX = %4 deg  RY = %5 deg  RZ = %6 deg")
        .arg(m_stationBasePose.xMm, 0, 'f', 3)
        .arg(m_stationBasePose.yMm, 0, 'f', 3)
        .arg(m_stationBasePose.zMm, 0, 'f', 3)
        .arg(m_stationBasePose.rxDeg, 0, 'f', 4)
        .arg(m_stationBasePose.ryDeg, 0, 'f', 4)
        .arg(m_stationBasePose.rzDeg, 0, 'f', 4);
    return result;
}

} // namespace i2r_rlc::ui
