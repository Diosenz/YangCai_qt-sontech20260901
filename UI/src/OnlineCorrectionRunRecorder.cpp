/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionRunRecorder.cpp
 * @brief       在线纠偏运行数据记录器实现
 ******************************************************************************/
#include "OnlineCorrectionRunRecorder.h"

#include <algorithm>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>

namespace i2r_rlc::ui {

namespace {

constexpr auto kSummaryFileName = "session_summary.txt";
constexpr auto kInitialCorrectionFileName = "initial_correction.txt";
constexpr auto kRealtimeCorrectionFileName = "realtime_correction.txt";
constexpr auto kActualRobotPathFileName = "actual_robot_path.txt";

QString timestampText()
{
    return QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
}

QString timestampMsText()
{
    return QString::number(QDateTime::currentMSecsSinceEpoch());
}

QString sanitizeField(QString value)
{
    value.replace('\t', ' ');
    value.replace('\r', ' ');
    value.replace('\n', ' ');
    return value;
}

QString phaseText(const control::OnlineCorrectionPhase phase)
{
    switch (phase) {
    case control::OnlineCorrectionPhase::Idle:
        return QStringLiteral("idle");
    case control::OnlineCorrectionPhase::InitialDetectionRunning:
        return QStringLiteral("initial_detection_running");
    case control::OnlineCorrectionPhase::WaitingForReverseCompletion:
        return QStringLiteral("waiting_reverse_completion");
    case control::OnlineCorrectionPhase::PreparingInitialTrajectory:
        return QStringLiteral("preparing_initial_trajectory");
    case control::OnlineCorrectionPhase::SendingInitialTeachPoints:
        return QStringLiteral("sending_initial_teach_points");
    case control::OnlineCorrectionPhase::WaitingInitialTeachPointAck:
        return QStringLiteral("waiting_initial_teach_point_ack");
    case control::OnlineCorrectionPhase::WaitingForInitialApproach:
        return QStringLiteral("waiting_initial_approach");
    case control::OnlineCorrectionPhase::WaitingForOnlineStart:
        return QStringLiteral("waiting_online_start");
    case control::OnlineCorrectionPhase::OnlineRunning:
        return QStringLiteral("online_running");
    case control::OnlineCorrectionPhase::SegmentFinishing:
        return QStringLiteral("segment_finishing");
    case control::OnlineCorrectionPhase::WaitingForDetectionReset:
        return QStringLiteral("waiting_detection_reset");
    case control::OnlineCorrectionPhase::WaitingForRobotProgramStop:
        return QStringLiteral("waiting_robot_program_stop");
    case control::OnlineCorrectionPhase::Completed:
        return QStringLiteral("completed");
    case control::OnlineCorrectionPhase::Error:
        return QStringLiteral("error");
    default:
        return QStringLiteral("unknown");
    }
}

QString startModeText(const control::OnlineCorrectionStartMode mode)
{
    switch (mode) {
    case control::OnlineCorrectionStartMode::InitType:
        return QStringLiteral("init_type");
    case control::OnlineCorrectionStartMode::GlobalCorrection:
        return QStringLiteral("global_correction");
    case control::OnlineCorrectionStartMode::IBVSStart:
        return QStringLiteral("ibvs_start");
    default:
        return QStringLiteral("unknown");
    }
}

QString boolText(const bool value)
{
    return value ? QStringLiteral("1") : QStringLiteral("0");
}

QString numberText(const double value, const int precision = 6)
{
    return QString::number(value, 'f', precision);
}

QString optionalNumberText(const std::optional<float> &value)
{
    return value.has_value() ? numberText(static_cast<double>(*value)) : QString();
}

QString pointValue(
    const control::OnlineCorrectionPoint *point,
    const int coordinate)
{
    if (point == nullptr || !point->valid) {
        return {};
    }
    if (coordinate == 0) {
        return numberText(point->xMm);
    }
    if (coordinate == 1) {
        return numberText(point->yMm);
    }
    return numberText(point->zMm);
}

} // namespace

bool COnlineCorrectionRunRecorder::beginSession(
    const QString &outputRootDirectory,
    const RuntimeOnlineCorrectionConfiguration &configuration,
    QString *errorMessage)
{
    m_active = false;
    m_sessionDirectory.clear();
    m_lastError.clear();
    m_startedAtMs = QDateTime::currentMSecsSinceEpoch();
    m_robotStateCount = 0;
    m_trackingResultCount = 0;
    m_correctedPathPointCount = 0;
    m_initialCommandCount = 0;
    m_realtimeCommandCount = 0;
    m_receiptCount = 0;

    if (outputRootDirectory.trimmed().isEmpty()) {
        m_lastError = QStringLiteral("在线纠偏记录根目录为空。");
        if (errorMessage != nullptr) {
            *errorMessage = m_lastError;
        }
        return false;
    }

    QDir rootDirectory(outputRootDirectory);
    if (!rootDirectory.mkpath(QStringLiteral("."))) {
        m_lastError = QStringLiteral("无法创建在线纠偏记录根目录：%1")
                          .arg(outputRootDirectory);
        if (errorMessage != nullptr) {
            *errorMessage = m_lastError;
        }
        return false;
    }

    const QString baseName = QDateTime::currentDateTime().toString(
        QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    QString sessionName = baseName;
    int suffix = 1;
    while (QFileInfo::exists(rootDirectory.filePath(sessionName))) {
        sessionName = QStringLiteral("%1_%2").arg(baseName).arg(suffix);
        ++suffix;
    }
    m_sessionDirectory = rootDirectory.filePath(sessionName);
    if (!QDir().mkpath(m_sessionDirectory)) {
        m_lastError = QStringLiteral("无法创建在线纠偏会话目录：%1")
                          .arg(m_sessionDirectory);
        if (errorMessage != nullptr) {
            *errorMessage = m_lastError;
        }
        m_sessionDirectory.clear();
        return false;
    }

    const bool created =
        createSessionFile(
            QString::fromLatin1(kSummaryFileName),
            QStringLiteral("key\tvalue\n"))
        && createSessionFile(
            QString::fromLatin1(kInitialCorrectionFileName),
            QStringLiteral(
                "timestamp_ms\ttime\tevent\tphase\tsegment_index\tpoint_index\t"
                "raw_x_mm\traw_y_mm\traw_z_mm\ttcp_offset_x_mm\t"
                "tcp_offset_y_mm\ttcp_offset_z_mm\tconverted_x_mm\t"
                "converted_y_mm\tconverted_z_mm\taligned_first_x_mm\t"
                "aligned_first_y_mm\taligned_first_z_mm\trobot_x_mm\t"
                "robot_y_mm\trobot_z_mm\trobot_rx_deg\trobot_ry_deg\t"
                "robot_rz_deg\ts1_deg\ts2_deg\tmessage\n"))
        && createSessionFile(
            QString::fromLatin1(kRealtimeCorrectionFileName),
            QStringLiteral(
                "timestamp_ms\ttime\tevent\tphase\tvision_valid\tvision_x_mm\t"
                "vision_y_mm\tvision_z_mm\tgap_width_mm\tseam_index\tdx_um\t"
                "dy_um\tdz_um\tspeed_override\tspeed_mode\tonline_end_flag\t"
                "sent\tresponse_received\tcompleted\terror_code\trequest\t"
                "response\tmessage\n"))
        && createSessionFile(
            QString::fromLatin1(kActualRobotPathFileName),
            QStringLiteral(
                "timestamp_ms\ttime\tphase\tx_mm\ty_mm\tz_mm\trx_deg\try_deg\t"
                "rz_deg\ts1_deg\ts2_deg\tinterpolation\tcommand_speed_x\t"
                "command_speed_y\tcommand_speed_z\ts_start\ts_hold\talarm\t"
                "traflag\tcorflag\tdetflag\toffline\tjob_line\tstep\n"));
    if (!created) {
        if (errorMessage != nullptr) {
            *errorMessage = m_lastError;
        }
        return false;
    }

    m_active = true;
    const bool summaryWritten =
        appendSummary(QStringLiteral("session_state"), QStringLiteral("running"))
        && appendSummary(QStringLiteral("start_time"), timestampText())
        && appendSummary(
            QStringLiteral("start_timestamp_ms"),
            QString::number(m_startedAtMs))
        && appendSummary(
            QStringLiteral("start_mode"),
            startModeText(configuration.options.startMode))
        && appendSummary(
            QStringLiteral("work_path_file"),
            QString::fromStdString(configuration.trajectoryRequest.workPathFile))
        && appendSummary(
            QStringLiteral("teach_pose_file"),
            QString::fromStdString(configuration.trajectoryRequest.teachPoseFile))
        && appendSummary(
            QStringLiteral("rbcalib_file"),
            QString::fromStdString(configuration.rbcalibFile))
        && appendSummary(
            QStringLiteral("initial_match_max_distance_mm"),
            numberText(configuration.options.initialPathMatchMaxDistanceMm))
        && appendSummary(
            QStringLiteral("initial_match_direction_sample_count"),
            QString::number(
                configuration.options.initialPathMatchDirectionSampleCount))
        && appendSummary(
            QStringLiteral("initial_match_min_scan_span_mm"),
            numberText(configuration.options.initialPathMatchMinScanSpanMm))
        && appendSummary(
            QStringLiteral("initial_match_min_reverse_direction_cosine"),
            numberText(
                configuration.options.initialPathMatchMinReverseDirectionCosine))
        && appendSummary(
            QStringLiteral("initial_match_min_remaining_length_mm"),
            numberText(
                configuration.options.initialPathMatchMinRemainingLengthMm));
    if (!summaryWritten) {
        m_active = false;
        if (errorMessage != nullptr) {
            *errorMessage = m_lastError;
        }
        return false;
    }

    return true;
}

bool COnlineCorrectionRunRecorder::recordRobotState(
    const robot_comm::RobotState &state,
    const control::OnlineCorrectionPhase phase)
{
    if (!m_active) {
        return true;
    }

    const QStringList fields{
        timestampMsText(),
        timestampText(),
        phaseText(phase),
        numberText(static_cast<double>(state.curx_r) / 1000.0),
        numberText(static_cast<double>(state.cury_r) / 1000.0),
        numberText(static_cast<double>(state.curz_r) / 1000.0),
        numberText(static_cast<double>(state.curRx_r) / 10000.0),
        numberText(static_cast<double>(state.curRy_r) / 10000.0),
        numberText(static_cast<double>(state.curRz_r) / 10000.0),
        numberText(static_cast<double>(state.curS1) / 10000.0),
        numberText(static_cast<double>(state.curS2) / 10000.0),
        QString::number(state.cur_interpolation),
        QString::number(state.command_speed_x),
        QString::number(state.command_speed_y),
        QString::number(state.command_speed_z),
        QString::number(state.sStart),
        QString::number(state.sHold),
        QString::number(state.sIsAlarm),
        QString::number(state.traflag),
        QString::number(state.corflag),
        QString::number(state.detflag),
        QString::number(state.offline),
        QString::number(state.usJobLine),
        QString::number(state.usStep)
    };
    const bool written = appendFields(
        QString::fromLatin1(kActualRobotPathFileName), fields);
    if (written) {
        ++m_robotStateCount;
    }
    return written;
}

bool COnlineCorrectionRunRecorder::recordTrackingResult(
    const RuntimeTrackingResult &result,
    const control::OnlineCorrectionPhase phase)
{
    if (!m_active) {
        return true;
    }

    const auto &point = result.sample.point;
    const QStringList fields{
        timestampMsText(),
        timestampText(),
        QStringLiteral("vision_sample"),
        phaseText(phase),
        boolText(result.valid && point.valid),
        point.valid ? numberText(point.xMm) : QString(),
        point.valid ? numberText(point.yMm) : QString(),
        point.valid ? numberText(point.zMm) : QString(),
        optionalNumberText(result.sample.gapWidthMm),
        QString::number(result.simulatedSeamIndex),
        {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
        sanitizeField(result.message)
    };
    const bool written = appendFields(
        QString::fromLatin1(kRealtimeCorrectionFileName), fields);
    if (written) {
        ++m_trackingResultCount;
    }
    return written;
}

bool COnlineCorrectionRunRecorder::recordUpdate(
    const control::OnlineCorrectionUpdate &update,
    const vision::PositionerCalibration &positionerCalibration)
{
    if (!m_active) {
        return true;
    }

    bool written = true;
    if (update.command.valid) {
        const auto &pathMatch = update.state.initialPathMatch;
        if (pathMatch.valid) {
            written = appendSummary(
                QStringLiteral("initial_match_work_point_index"),
                QString::number(pathMatch.matchedPathPointIndex + 1)) && written;
            written = appendSummary(
                QStringLiteral("initial_match_segment_fraction"),
                numberText(pathMatch.segmentFraction)) && written;
            written = appendSummary(
                QStringLiteral("initial_match_distance_mm"),
                numberText(pathMatch.distanceMm)) && written;
            written = appendSummary(
                QStringLiteral("initial_match_offset_mm"),
                QStringLiteral("%1,%2,%3")
                    .arg(numberText(pathMatch.correctionOffset.xMm),
                         numberText(pathMatch.correctionOffset.yMm),
                         numberText(pathMatch.correctionOffset.zMm))) && written;
            written = appendSummary(
                QStringLiteral("initial_match_reverse_direction_cosine"),
                pathMatch.directionValidated
                    ? numberText(pathMatch.reverseDirectionCosine)
                    : QStringLiteral("not_validated")) && written;
        }
        const control::OnlineCorrectionPoint *rawPoint = nullptr;
        if (!update.state.initialDetectionSamples.empty()) {
            rawPoint = &update.state.initialDetectionSamples.back().point;
        }
        control::OnlineCorrectionPoint convertedPoint;
        if (rawPoint != nullptr && rawPoint->valid) {
            convertedPoint = *rawPoint;
            convertedPoint.xMm += positionerCalibration.toolTcpInLastLinkMm.x();
            convertedPoint.yMm += positionerCalibration.toolTcpInLastLinkMm.y();
            convertedPoint.zMm += positionerCalibration.toolTcpInLastLinkMm.z();
        }
        const control::OnlineCorrectionPoint *alignedPoint =
            update.state.correctedSegmentPath.empty()
                ? nullptr
                : &update.state.correctedSegmentPath.front();

        for (std::size_t index = 0;
             index < update.state.correctedSegmentPath.size();
             ++index) {
            const auto &correctedPoint = update.state.correctedSegmentPath[index];
            const QStringList fields{
                timestampMsText(),
                timestampText(),
                QStringLiteral("corrected_path_point"),
                phaseText(update.phase),
                QString::number(update.state.segmentIndex),
                QString::number(index + 1),
                pointValue(rawPoint, 0),
                pointValue(rawPoint, 1),
                pointValue(rawPoint, 2),
                numberText(positionerCalibration.toolTcpInLastLinkMm.x()),
                numberText(positionerCalibration.toolTcpInLastLinkMm.y()),
                numberText(positionerCalibration.toolTcpInLastLinkMm.z()),
                pointValue(&convertedPoint, 0),
                pointValue(&convertedPoint, 1),
                pointValue(&convertedPoint, 2),
                pointValue(&correctedPoint, 0),
                pointValue(&correctedPoint, 1),
                pointValue(&correctedPoint, 2),
                {}, {}, {}, {}, {}, {}, {}, {},
                sanitizeField(QString::fromStdString(update.message))
            };
            const bool pointWritten = appendFields(
                QString::fromLatin1(kInitialCorrectionFileName), fields);
            written = pointWritten && written;
            if (pointWritten) {
                ++m_correctedPathPointCount;
            }
        }

        const int commandPointCount = std::min(
            update.command.pointCount,
            static_cast<int>(update.command.teachPoints.size()));
        for (int index = 0; index < commandPointCount; ++index) {
            const auto &robotPoint = update.command.teachPoints[
                static_cast<std::size_t>(index)];
            const robot_comm::StationAxisUpdateCommand *stationAxis =
                index < static_cast<int>(update.command.stationAxes.size())
                    ? &update.command.stationAxes[static_cast<std::size_t>(index)]
                    : nullptr;
            const QStringList fields{
                timestampMsText(),
                timestampText(),
                QStringLiteral("teach_point"),
                phaseText(update.phase),
                QString::number(update.state.segmentIndex),
                QString::number(index + 1),
                pointValue(rawPoint, 0),
                pointValue(rawPoint, 1),
                pointValue(rawPoint, 2),
                numberText(positionerCalibration.toolTcpInLastLinkMm.x()),
                numberText(positionerCalibration.toolTcpInLastLinkMm.y()),
                numberText(positionerCalibration.toolTcpInLastLinkMm.z()),
                pointValue(&convertedPoint, 0),
                pointValue(&convertedPoint, 1),
                pointValue(&convertedPoint, 2),
                pointValue(alignedPoint, 0),
                pointValue(alignedPoint, 1),
                pointValue(alignedPoint, 2),
                numberText(static_cast<double>(robotPoint.xUm) / 1000.0),
                numberText(static_cast<double>(robotPoint.yUm) / 1000.0),
                numberText(static_cast<double>(robotPoint.zUm) / 1000.0),
                numberText(static_cast<double>(robotPoint.rxDeg10000) / 10000.0),
                numberText(static_cast<double>(robotPoint.ryDeg10000) / 10000.0),
                numberText(static_cast<double>(robotPoint.rzDeg10000) / 10000.0),
                stationAxis != nullptr
                    ? numberText(static_cast<double>(stationAxis->s1Deg10000) / 10000.0)
                    : QString(),
                stationAxis != nullptr
                    ? numberText(static_cast<double>(stationAxis->s2Deg10000) / 10000.0)
                    : QString(),
                sanitizeField(QString::fromStdString(update.message))
            };
            const bool commandWritten = appendFields(
                QString::fromLatin1(kInitialCorrectionFileName), fields);
            written = commandWritten && written;
            if (commandWritten) {
                ++m_initialCommandCount;
            }
        }
    }

    if (update.realtimeCommand.valid) {
        const auto &command = update.realtimeCommand;
        const QStringList fields{
            timestampMsText(),
            timestampText(),
            QStringLiteral("command_generated"),
            phaseText(update.phase),
            {}, {}, {}, {}, {}, {},
            QString::number(command.dxUm),
            QString::number(command.dyUm),
            QString::number(command.dzUm),
            QString::number(command.speedOverride),
            QString::number(command.speedMode),
            QString::number(command.onlineEndFlag),
            {}, {}, {}, {}, {}, {},
            sanitizeField(QString::fromStdString(command.message))
        };
        const bool commandWritten = appendFields(
            QString::fromLatin1(kRealtimeCorrectionFileName), fields);
        written = commandWritten && written;
        if (commandWritten) {
            ++m_realtimeCommandCount;
        }
    }

    return written;
}

bool COnlineCorrectionRunRecorder::recordCommandReceipt(
    const robot_comm::RobotCommandReceipt &receipt,
    const robot_comm::RobotState &state,
    const control::OnlineCorrectionPhase phase)
{
    Q_UNUSED(state);
    if (!m_active) {
        return true;
    }

    bool written = true;
    if (receipt.type == robot_comm::RobotCommandType::RealtimeCorrection) {
        const QStringList fields{
            timestampMsText(),
            timestampText(),
            QStringLiteral("command_receipt"),
            phaseText(phase),
            {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
            boolText(receipt.sent),
            boolText(receipt.responseReceived),
            boolText(receipt.completed),
            QString::number(static_cast<int>(receipt.error)),
            sanitizeField(QString::fromStdString(receipt.request)),
            sanitizeField(QString::fromStdString(receipt.response)),
            sanitizeField(QString::fromStdString(receipt.message))
        };
        written = appendFields(
            QString::fromLatin1(kRealtimeCorrectionFileName), fields);
    } else if (receipt.type == robot_comm::RobotCommandType::TeachPointUpdate) {
        const QStringList fields{
            timestampMsText(),
            timestampText(),
            QStringLiteral("command_receipt"),
            phaseText(phase),
            {},
            QString::number(receipt.pointIndex + 1),
            {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
            sanitizeField(
                QStringLiteral("sent=%1 response_received=%2 completed=%3 error=%4 request=%5 response=%6 message=%7")
                    .arg(boolText(receipt.sent))
                    .arg(boolText(receipt.responseReceived))
                    .arg(boolText(receipt.completed))
                    .arg(static_cast<int>(receipt.error))
                    .arg(sanitizeField(QString::fromStdString(receipt.request)))
                    .arg(sanitizeField(QString::fromStdString(receipt.response)))
                    .arg(sanitizeField(QString::fromStdString(receipt.message))))
        };
        written = appendFields(
            QString::fromLatin1(kInitialCorrectionFileName), fields);
    }

    if (written) {
        ++m_receiptCount;
    }
    return written;
}

bool COnlineCorrectionRunRecorder::finishSession(
    const QString &resultText,
    const QString &message)
{
    if (!m_active) {
        return true;
    }

    const qint64 finishedAtMs = QDateTime::currentMSecsSinceEpoch();
    bool written = true;
    written = appendSummary(QStringLiteral("session_state"), resultText) && written;
    written = appendSummary(QStringLiteral("end_time"), timestampText()) && written;
    written = appendSummary(
        QStringLiteral("end_timestamp_ms"), QString::number(finishedAtMs)) && written;
    written = appendSummary(
        QStringLiteral("duration_ms"), QString::number(finishedAtMs - m_startedAtMs)) && written;
    written = appendSummary(
        QStringLiteral("robot_state_count"), QString::number(m_robotStateCount)) && written;
    written = appendSummary(
        QStringLiteral("tracking_result_count"), QString::number(m_trackingResultCount)) && written;
    written = appendSummary(
        QStringLiteral("corrected_path_point_count"),
        QString::number(m_correctedPathPointCount)) && written;
    written = appendSummary(
        QStringLiteral("initial_command_count"), QString::number(m_initialCommandCount)) && written;
    written = appendSummary(
        QStringLiteral("realtime_command_count"), QString::number(m_realtimeCommandCount)) && written;
    written = appendSummary(
        QStringLiteral("command_receipt_count"), QString::number(m_receiptCount)) && written;
    written = appendSummary(QStringLiteral("message"), sanitizeField(message)) && written;
    m_active = false;
    return written;
}

bool COnlineCorrectionRunRecorder::isActive() const
{
    return m_active;
}

QString COnlineCorrectionRunRecorder::sessionDirectory() const
{
    return m_sessionDirectory;
}

QString COnlineCorrectionRunRecorder::lastError() const
{
    return m_lastError;
}

bool COnlineCorrectionRunRecorder::createSessionFile(
    const QString &fileName,
    const QString &header)
{
    const QString path = QDir(m_sessionDirectory).filePath(fileName);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        setFileError(QStringLiteral("创建"), path);
        return false;
    }
    if (file.write(header.toUtf8()) != header.toUtf8().size()) {
        setFileError(QStringLiteral("写入表头"), path);
        return false;
    }
    return true;
}

bool COnlineCorrectionRunRecorder::appendFields(
    const QString &fileName,
    const QStringList &fields)
{
    const QString path = QDir(m_sessionDirectory).filePath(fileName);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append)) {
        setFileError(QStringLiteral("追加"), path);
        return false;
    }
    const QByteArray line = fields.join('\t').toUtf8() + '\n';
    if (file.write(line) != line.size()) {
        setFileError(QStringLiteral("写入"), path);
        return false;
    }
    return true;
}

bool COnlineCorrectionRunRecorder::appendSummary(
    const QString &key,
    const QString &value)
{
    return appendFields(
        QString::fromLatin1(kSummaryFileName),
        QStringList{sanitizeField(key), sanitizeField(value)});
}

void COnlineCorrectionRunRecorder::setFileError(
    const QString &operation,
    const QString &path)
{
    m_lastError = QStringLiteral("%1在线纠偏记录文件失败：%2")
                      .arg(operation, path);
}

} // namespace i2r_rlc::ui
