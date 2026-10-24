/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionRunRecorder.cpp
 * @brief       在线纠偏运行数据记录器实现
 ******************************************************************************/
#include "OnlineCorrectionRunRecorder.h"

#include <algorithm>
#include <exception>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>

namespace i2r_rlc::ui {

namespace {

constexpr auto kVisionDiagnosticFileName = "vision_diagnostics.tsv";
constexpr auto kSummaryFileName = "session_summary.txt";
constexpr auto kInitialCorrectionFileName = "initial_correction.txt";
constexpr auto kRealtimeCorrectionFileName = "realtime_correction.txt";
constexpr auto kActualRobotPathFileName = "actual_robot_path.txt";
constexpr auto kIcpBatchesFileName = "online_icp_batches.txt";
constexpr auto kIcpPointsFileName = "online_icp_points.txt";
constexpr auto kFollowTargetsFileName = "online_follow_targets.txt";

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

/** @brief 记录层防止新增辅助来源因调用方漏标而被归档为独立实测。 */
bool estimatedPoint(const RuntimeTrackingResult &result)
{
    return result.sample.estimated
        || result.pointSource == vision::OnlineSeamPointSource::GuidedEstimate;
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
    if (m_active && !finishSession(QStringLiteral("restarted"),
                                  QStringLiteral("记录器开始新的会话。"))) {
        if (errorMessage != nullptr) *errorMessage = m_lastError;
        return false;
    }
    m_imageWriter.reset();
    m_visionDiagnosticOptions = configuration.visionDiagnostics;
    m_visionDiagnosticCount = 0;
    m_imageSubmissionCount = 0;
    m_imageBucketSubmissionCounts = {};
    m_imageErrorReported = false;
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
    m_updateCount = 0;
    m_icpBatchCount = 0;

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

    QString matrixHeader;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrixHeader += QStringLiteral("\tm%1%2").arg(row).arg(column);
        }
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
                "response\tmessage\testimated\tpoint_source\tdegraded_frames\n"))
        && createSessionFile(
            QString::fromLatin1(kActualRobotPathFileName),
            QStringLiteral(
                "timestamp_ms\ttime\tphase\tx_mm\ty_mm\tz_mm\trx_deg\try_deg\t"
                "rz_deg\ts1_deg\ts2_deg\tinterpolation\tcommand_speed_x\t"
                "command_speed_y\tcommand_speed_z\ts_start\ts_hold\talarm\t"
                "traflag\tcorflag\tdetflag\toffline\tjob_line\tstep\n"))
        && createSessionFile(
            QString::fromLatin1(kIcpBatchesFileName),
            QStringLiteral(
                "timestamp_ms\tupdate_index\tbatch_index\tsegment_index\tphase\t"
                "enabled\tattempted\tapplied\tflush\tfit_count\tmodel_count\t"
                "pending_count\treleased_count\trms_mm\tmatrix_present")
                + matrixHeader + QStringLiteral("\tmessage\n"))
        && createSessionFile(
            QString::fromLatin1(kIcpPointsFileName),
            QStringLiteral("batch_index\tkind\tpoint_index\tx_mm\ty_mm\tz_mm\tvalid\t"
                           "gap_width_mm\tseam_index\testimated\n"))
        && createSessionFile(
            QString::fromLatin1(kFollowTargetsFileName),
            QStringLiteral(
                "timestamp_ms\tupdate_index\tsegment_index\tphase\ttaught_point_index\t"
                "s1_deg\ts2_deg\tcurrent_x_mm\tcurrent_y_mm\tcurrent_z_mm\t"
                "first_x_mm\tfirst_y_mm\tfirst_z_mm\tdirection_x_mm\tdirection_y_mm\t"
                "direction_z_mm\tfoot_x_mm\tfoot_y_mm\tfoot_z_mm\tfuture_foot_x_mm\t"
                "future_foot_y_mm\tfuture_foot_z_mm\tdx_um\tdy_um\tdz_um\tonline_end_flag\n"));
    if (!created) {
        if (errorMessage != nullptr) {
            *errorMessage = m_lastError;
        }
        return false;
    }

    if (m_visionDiagnosticOptions.enabled) {
        if (!createSessionFile(QString::fromLatin1(kVisionDiagnosticFileName), QStringLiteral(
        "record_timestamp_ms\tsession_id\tframe_id\tpurpose\tphase\ttrigger_accepted\tresult_valid\tcamera_timestamp_ms\t"
        "robot_state_received_ms\tprocessing_started_ms\tprocessing_finished_ms\talgorithm_attempted\thas_response\ttracker_completed\tkalman_seeded\tfusion_applied\t"
        "template_reinitialized\tframes_since_initialize\tgabor_succeeded\timage_width_px\timage_height_px\tsim_x_px\tsim_y_px\tresponse_x_px\t"
        "response_y_px\tbefore_fusion_x_px\tbefore_fusion_y_px\tfinal_x_px\tfinal_y_px\tresponse_peak\tnormalized_peak\tobservation_noise\t"
        "kcf_weight_x\tkcf_weight_y\tprior_weight_x\tprior_weight_y\tcorrelation_jump_rejected\tfusion_jump_rejected\tenhancement_roi_x\tenhancement_roi_y\t"
        "enhancement_roi_width\tenhancement_roi_height\tfinal_roi_x\tfinal_roi_y\tfinal_roi_width\tfinal_roi_height\tanchor_x\tanchor_y\t"
        "positioner_tcp_x_mm\tpositioner_tcp_y_mm\tpositioner_tcp_z_mm\trobot_base_x_mm\trobot_base_y_mm\trobot_base_z_mm\tstation_base_x_mm\tstation_base_y_mm\t"
        "station_base_z_mm\trobot_x_mm\trobot_y_mm\trobot_z_mm\trobot_rx_deg\trobot_ry_deg\trobot_rz_deg\ts1_deg\t"
        "s2_deg\ttraflag\tcorflag\tdetflag\tposflag\tjob_line\tsource_image\tenhanced_image\t"
        "image_request_status\timage_sample_bucket\tmessage\t"
        "geometry_enabled\tgeometry_attempted\tgeometry_horizontal_valid\tgeometry_valid\tgeometry_status\t"
        "geometry_x_px\tgeometry_y_px\tgeometry_left_x_px\tgeometry_left_y_px\tgeometry_right_x_px\tgeometry_right_y_px\t"
        "geometry_support_ratio\tgeometry_fit_rms_px\tgeometry_x_platform_width_px\tgeometry_height_jump_px\t"
        "geometry_quality_score\tgeometry_left_support_columns\tgeometry_right_support_columns\tgeometry_elapsed_ms\t"
        "coarse_final_x_px\tcoarse_final_y_px\tgeometry_ambiguous_column_ratio\tpoint_source\tdegraded_frames\ttransition_limited\testimated\t"
        "guided_attempted\tguided_valid\tguided_has_trusted_anchor\tguided_prediction_budget_valid\t"
        "guided_shoulder_support\tguided_gap_center_supported\tguided_ambiguous\tguided_raw_jump_rejected\t"
        "guided_degraded_frames\tguided_anchor_age_ms\tguided_reference_travel_px\t"
        "guided_left_support_columns\tguided_right_support_columns\tguided_ambiguous_columns\tguided_max_branch_rms_px\t"
        "guided_prediction_x_px\tguided_prediction_y_px\tguided_x_px\tguided_y_px\tworkpiece_pose_revision\t"
        "online_observation_fusion_enabled\traw_geometry_horizontal_valid\traw_geometry_valid\traw_geometry_status\t"
        "raw_geometry_x_px\traw_geometry_y_px\tcoarse_observation_quality\tcoarse_observation_used\t"
        "observation_weight_x\tobservation_weight_y\tguided_reacquisition_pending\tguided_reacquired\t"
        "guided_reacquisition_blocked_previous_branch\tguided_reacquisition_frames\t"
        "guided_image_continuity_active\tguided_image_observation_quality\tguided_model_translation_suppressed\t"
        "kcf_online_observation_mode\tkcf_training_deferred\tkcf_training_committed\t"
        "geometry_has_branch_directions\tgeometry_left_slope\tgeometry_right_slope\tkcf_real_search_fallback_used\t"
        "image_measurement_output_enabled\timage_observation_accepted\timage_search_expanded\n"))) {
            if (errorMessage != nullptr) *errorMessage = m_lastError;
            return false;
        }
        if (m_visionDiagnosticOptions.imagesEnabled
            && m_visionDiagnosticOptions.maxImagePairs > 0
            && m_visionDiagnosticOptions.maxImageBytes > 0) {
            if (!QDir(m_sessionDirectory).mkpath(QStringLiteral("vision_images"))
                || !createSessionFile(QStringLiteral("vision_images/image_manifest.tsv"),
                                      QStringLiteral("stem\tstatus\tencoded_bytes\n"))) {
                if (m_lastError.isEmpty()) {
                    m_lastError = QStringLiteral("无法创建诊断图像目录。");
                }
                if (errorMessage != nullptr) *errorMessage = m_lastError;
                return false;
            }
            try {
                m_imageWriter = std::make_unique<CTrackingDiagnosticImageWriter>(
                    QDir(m_sessionDirectory).filePath(QStringLiteral("vision_images")),
                    m_visionDiagnosticOptions.maxImageBytes);
            } catch (const std::exception &exception) {
                m_lastError = QStringLiteral("无法启动诊断图像写入线程：%1")
                    .arg(QString::fromUtf8(exception.what()));
                if (errorMessage != nullptr) {
                    *errorMessage = m_lastError;
                }
                return false;
            }
        }
    }
    m_active = true;
    const bool summaryWritten =
        appendSummary(QStringLiteral("session_state"), QStringLiteral("running"))
        && appendSummary(QStringLiteral("vision_diagnostics_schema"), QStringLiteral("7"))
        && appendSummary(QStringLiteral("vision_diagnostics_enabled"), boolText(m_visionDiagnosticOptions.enabled))
        && appendSummary(QStringLiteral("vision_images_enabled"), boolText(m_visionDiagnosticOptions.imagesEnabled))
        && appendSummary(QStringLiteral("vision_image_sample_every_n"), QString::number(m_visionDiagnosticOptions.imageSampleEveryN))
        && appendSummary(QStringLiteral("vision_max_image_pairs"), QString::number(m_visionDiagnosticOptions.maxImagePairs))
        && appendSummary(QStringLiteral("vision_max_image_bytes"), QString::number(m_visionDiagnosticOptions.maxImageBytes))
        && appendSummary(QStringLiteral("vision_image_sampling_policy"),
                         QStringLiteral("startup/online_sampling_early/middle/late/anomaly=3/6/6/6/9; quadratic_backoff; anomaly_minimum_interval_ms=60000; raw_deviation_px=10; layers_are_not_physical_seam_segments"))
        && appendSummary(QStringLiteral("icp_diagnostics_schema"), QStringLiteral("2"))
        && appendSummary(QStringLiteral("start_time"), timestampText())
        && appendSummary(
            QStringLiteral("start_timestamp_ms"),
            QString::number(m_startedAtMs))
        && appendSummary(
            QStringLiteral("start_mode"),
            startModeText(configuration.options.startMode))
        && appendSummary(
            QStringLiteral("enable_online_icp"),
            configuration.options.enableOnlineIcp
                ? QStringLiteral("true") : QStringLiteral("false"))
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
        sanitizeField(result.message), boolText(estimatedPoint(result)),
        QString::number(static_cast<int>(result.pointSource)), QString::number(result.degradedFrames)
    };
    const bool written = appendFields(
        QString::fromLatin1(kRealtimeCorrectionFileName), fields);
    if (written) {
        ++m_trackingResultCount;
    }
    const bool diagnosticWritten = recordVisionDiagnostic(result, phase);
    return written && diagnosticWritten;
}

bool COnlineCorrectionRunRecorder::recordVisionDiagnostic(
    const RuntimeTrackingResult &result, control::OnlineCorrectionPhase phase)
{
    if (!m_visionDiagnosticOptions.enabled) {
        return true;
    }
    ++m_visionDiagnosticCount;
    const CRuntimeTrackingDiagnostic empty;
    const auto &diagnostic = result.diagnostic.has_value() ? *result.diagnostic : empty;
    const auto &algorithm = diagnostic.algorithm;
    const auto &tracker = algorithm.tracker;
    const auto &state = diagnostic.robotState;
    const bool hasContext = result.diagnostic.has_value();
    QStringList fields{
        timestampMsText(), QString::number(result.correlation.sessionId),
        QString::number(result.frameId), QString::number(static_cast<int>(result.correlation.purpose)),
        phaseText(phase), boolText(result.triggerAccepted), boolText(result.valid)
    };
    for (const auto time : {diagnostic.cameraTimestampMs, diagnostic.robotStateReceivedMs,
                            diagnostic.processingStartedMs, diagnostic.processingFinishedMs}) {
        fields << (time >= 0 ? QString::number(time) : QString());
    }
    fields << boolText(algorithm.attempted) << boolText(tracker.hasResponse)
           << boolText(tracker.completed) << boolText(tracker.kalmanSeeded)
           << boolText(tracker.fusionApplied) << boolText(algorithm.templateReinitialized)
           << (algorithm.attempted ? QString::number(algorithm.framesSinceInitialize) : QString())
           << boolText(algorithm.gaborSucceeded)
           << (algorithm.imageSize.width > 0 ? QString::number(algorithm.imageSize.width) : QString())
           << (algorithm.imageSize.height > 0 ? QString::number(algorithm.imageSize.height) : QString());
    const auto addImagePoint = [&fields](const cv::Point2d &point, bool valid) {
        fields << (valid ? numberText(point.x) : QString())
               << (valid ? numberText(point.y) : QString());
    };
    addImagePoint(algorithm.simulatedPoint, algorithm.simulatedPointAvailable);
    addImagePoint(algorithm.responsePoint, tracker.completed);
    addImagePoint(algorithm.beforeFusionPoint, tracker.completed);
    addImagePoint(hasContext ? algorithm.finalPoint
        : cv::Point2d(result.imageXPixel, result.imageYPixel), result.valid);
    fields << (tracker.hasResponse ? numberText(tracker.responsePeak, 12) : QString());
    for (const double value : {tracker.normalizedPeak, tracker.observationNoise,
                              tracker.gainX, tracker.gainY, 1.0 - tracker.gainX, 1.0 - tracker.gainY}) {
        fields << (tracker.fusionApplied ? numberText(value, 12) : QString());
    }
    fields << boolText(tracker.correlationJumpRejected) << boolText(tracker.fusionJumpRejected);
    for (const cv::Rect2f roi : {cv::Rect2f(algorithm.enhancementRoi), tracker.finalRoi}) {
        for (const double value : {static_cast<double>(roi.x), static_cast<double>(roi.y),
                                  static_cast<double>(roi.width), static_cast<double>(roi.height)}) {
            fields << (algorithm.attempted ? numberText(value) : QString());
        }
    }
    addImagePoint(algorithm.templateAnchor, algorithm.attempted);
    for (const auto &point : {result.sample.point, result.detectedRobotBasePoint,
                              result.detectedStationBasePoint}) {
        fields << pointValue(&point, 0) << pointValue(&point, 1) << pointValue(&point, 2);
    }
    for (const double value : {state.curx_r / 1000.0, state.cury_r / 1000.0, state.curz_r / 1000.0,
                              state.curRx_r / 10000.0, state.curRy_r / 10000.0, state.curRz_r / 10000.0,
                              state.curS1 / 10000.0, state.curS2 / 10000.0}) {
        fields << (hasContext ? numberText(value) : QString());
    }
    for (const int value : {state.traflag, state.corflag, state.detflag, state.posflag, state.usJobLine}) {
        fields << (hasContext ? QString::number(value) : QString());
    }
    QString sourcePath;
    QString enhancedPath;
    QString imageStatus = QStringLiteral("not_sampled");
    if (!m_imageWriter) {
        imageStatus = QStringLiteral("disabled");
    } else if (!diagnostic.sourceImage.isNull() && !diagnostic.enhancedImage.isNull()) {
        const int bucket = diagnostic.imageSampleBucket;
        const bool stageMatches = bucket >= 0 && bucket < 5
            && ((bucket == 0 && !result.correlation.onlineControlWindowActive)
                || (bucket > 0 && result.correlation.onlineControlWindowActive));
        if (!stageMatches) {
            imageStatus = QStringLiteral("invalid_sampling_context");
        } else if (m_imageSubmissionCount < static_cast<quint64>(m_visionDiagnosticOptions.maxImagePairs)
            && m_imageBucketSubmissionCounts[bucket]
                < COnlineVisionImageSampling::pairBudget(m_visionDiagnosticOptions, bucket)) {
            const QString stem = QStringLiteral("sample_%1_session_%2_frame_%3")
                .arg(m_visionDiagnosticCount).arg(result.correlation.sessionId).arg(result.frameId);
            imageStatus = m_imageWriter->submit(stem, diagnostic.sourceImage, diagnostic.enhancedImage,
                bucket, COnlineVisionImageSampling::byteBudget(m_visionDiagnosticOptions, bucket));
            ++m_imageSubmissionCount;
            ++m_imageBucketSubmissionCounts[bucket];
            if (imageStatus == QStringLiteral("queued")) {
                sourcePath = QStringLiteral("vision_images/%1_source.png").arg(stem);
                enhancedPath = QStringLiteral("vision_images/%1_enhanced.png").arg(stem);
            }
        } else {
            imageStatus = QStringLiteral("pair_limit");
        }
    }
    fields << sourcePath << enhancedPath << imageStatus
           << (diagnostic.imageSampleBucket >= 0 ? QString::number(diagnostic.imageSampleBucket) : QString())
           << sanitizeField(result.message);
    const auto &geometry = algorithm.refinement;
    fields << boolText(algorithm.seamRefinementEnabled)
           << boolText(algorithm.refinementAttempted)
           << boolText(geometry.horizontalValid) << boolText(geometry.valid)
           << (algorithm.refinementAttempted
               ? QString::number(static_cast<int>(geometry.status)) : QString());
    addImagePoint(geometry.point, geometry.valid);
    addImagePoint(geometry.leftEndpoint, geometry.horizontalValid);
    addImagePoint(geometry.rightEndpoint, geometry.horizontalValid);
    for (const double value : {geometry.supportRatio, geometry.fitRmsPixel,
                              geometry.xUncertaintyPixel, geometry.heightJumpPixel,
                              geometry.confidence}) {
        fields << (algorithm.refinementAttempted ? numberText(value) : QString());
    }
    fields << (algorithm.refinementAttempted ? QString::number(geometry.leftSupportColumns) : QString())
           << (algorithm.refinementAttempted ? QString::number(geometry.rightSupportColumns) : QString())
           << (algorithm.refinementAttempted ? numberText(algorithm.refinementElapsedMs) : QString());
    addImagePoint(algorithm.coarseFinalPoint, tracker.completed);
    fields << (algorithm.refinementAttempted ? numberText(geometry.ambiguousColumnRatio) : QString());
    fields << QString::number(static_cast<int>(result.pointSource))
           << QString::number(result.degradedFrames)
           << boolText(algorithm.continuity.transitionLimited) << boolText(estimatedPoint(result));
    const auto &guided = algorithm.guided;
    fields << boolText(guided.attempted) << boolText(guided.valid)
           << boolText(guided.hasTrustedAnchor) << boolText(guided.predictionBudgetValid)
           << boolText(guided.hasRawShoulderSupport) << boolText(guided.localGapCenterSupported)
           << boolText(guided.ambiguous) << boolText(guided.rawJumpRejected)
           << QString::number(guided.degradedFrames)
           << QString::number(guided.anchorAgeMs) << numberText(guided.referenceTravelPixel)
           << QString::number(guided.leftSupportColumns) << QString::number(guided.rightSupportColumns)
           << QString::number(guided.ambiguousColumns) << numberText(guided.maximumBranchRmsPixel);
    addImagePoint(guided.prediction, guided.valid);
    addImagePoint(guided.point, guided.valid);
    fields << QString::number(result.correlation.workpiecePoseRevision);
    // 原图候选与接受后的几何分别保存，避免时序否决掩盖本帧真实检测。
    const auto &rawGeometry = algorithm.rawRefinement;
    fields << boolText(algorithm.onlineObservationFusionEnabled)
           << boolText(rawGeometry.horizontalValid) << boolText(rawGeometry.valid)
           << (algorithm.refinementAttempted
               ? QString::number(static_cast<int>(rawGeometry.status)) : QString());
    addImagePoint(rawGeometry.point, rawGeometry.valid);
    fields << (algorithm.onlineObservationFusionEnabled
                   ? numberText(algorithm.coarseObservationQuality) : QString())
           << boolText(algorithm.coarseObservationUsed)
           << (algorithm.onlineObservationFusionEnabled
                   ? numberText(algorithm.continuity.observationWeightX) : QString())
           << (algorithm.onlineObservationFusionEnabled
                   ? numberText(algorithm.continuity.observationWeightY) : QString())
           << boolText(guided.reacquisitionPending) << boolText(guided.reacquired)
           << boolText(guided.reacquisitionBlockedByPreviousBranch)
           << QString::number(guided.reacquisitionFrames)
           << boolText(guided.imageContinuityActive) << numberText(guided.imageObservationQuality)
           << boolText(guided.modelTranslationSuppressed)
           << boolText(algorithm.tracker.onlineObservationMode)
           << boolText(algorithm.tracker.trainingDeferred)
           << boolText(algorithm.tracker.trainingCommitted)
           << boolText(algorithm.refinement.valid && algorithm.refinement.hasBranchDirections)
           << (algorithm.refinement.valid && algorithm.refinement.hasBranchDirections
               ? numberText(algorithm.refinement.leftSlope) : QString())
           << (algorithm.refinement.valid && algorithm.refinement.hasBranchDirections
               ? numberText(algorithm.refinement.rightSlope) : QString())
           << boolText(algorithm.realSearchFallbackUsed)
           // 测量身份是运行链值对象的一部分，不能因可选诊断关闭而丢失。
           << boolText(result.imageMeasurementOutputEnabled)
           << boolText(result.imageObservationAccepted)
           << boolText(algorithm.imageSearchExpanded);
    const bool written = appendFields(QString::fromLatin1(kVisionDiagnosticFileName), fields);
    if (m_imageWriter && !m_imageErrorReported && !m_imageWriter->lastError().isEmpty()) {
        m_imageErrorReported = true;
        m_lastError = m_imageWriter->lastError();
        return false;
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

    ++m_updateCount;
    bool written = recordIcpDiagnostics(update);
    if (update.onlineIcpUpdated) {
        const auto &icp = update.state.onlineIcp;
        const QString diagnostics = QStringLiteral(
            "enabled=%1 attempted=%2 applied=%3 buffered=%4 rms_mm=%5 reason=%6")
                                        .arg(boolText(icp.enabled))
                                        .arg(boolText(icp.attempted))
                                        .arg(boolText(icp.applied))
                                        .arg(QString::number(
                                            static_cast<qulonglong>(icp.bufferedSampleCount)))
                                        .arg(numberText(icp.rmsErrorMm))
                                        .arg(QString::fromStdString(icp.message));
        const QStringList fields{
            timestampMsText(),
            timestampText(),
            QStringLiteral("online_icp"),
            phaseText(update.phase),
            {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
            sanitizeField(diagnostics)
        };
        written = appendFields(
            QString::fromLatin1(kRealtimeCorrectionFileName), fields) && written;
    }
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

bool COnlineCorrectionRunRecorder::recordIcpDiagnostics(
    const control::OnlineCorrectionUpdate &update)
{
    if (!update.onlineIcpUpdated) {
        return true;
    }
    bool written = true;
    QByteArray batchRows;
    QByteArray pointRows;
    const QString now = timestampMsText();
    for (const auto &batch : update.onlineIcpBatches) {
        const QString batchIndex = QString::number(++m_icpBatchCount);
        QStringList fields{
            now, QString::number(m_updateCount), batchIndex,
            QString::number(update.state.segmentIndex), phaseText(update.phase),
            boolText(batch.enabled), boolText(batch.attempted), boolText(batch.applied),
            boolText(batch.flush),
            QString::number(static_cast<qulonglong>(batch.fitPoints.size())),
            QString::number(static_cast<qulonglong>(batch.modelPoints.size())),
            QString::number(static_cast<qulonglong>(batch.pendingSamples.size())),
            QString::number(static_cast<qulonglong>(batch.releasedSamples.size())),
            batch.rmsErrorMm.has_value() ? numberText(*batch.rmsErrorMm, 9) : QString(),
            boolText(batch.detectionToModel.has_value())
        };
        for (std::size_t index = 0; index < 16U; ++index) {
            fields.push_back(batch.detectionToModel.has_value()
                ? numberText((*batch.detectionToModel)[index], 12) : QString());
        }
        fields.push_back(sanitizeField(QString::fromStdString(batch.message)));
        batchRows += fields.join('\t').toUtf8() + '\n';
        const auto appendPoint = [&pointRows, &batchIndex](
            const QString &kind, std::size_t index,
            const control::OnlineCorrectionPoint &point,
            const std::optional<float> &gap, const QString &seam, const QString &estimated) {
            const QStringList row{
                batchIndex, kind, QString::number(static_cast<qulonglong>(index)),
                numberText(point.xMm), numberText(point.yMm), numberText(point.zMm),
                boolText(point.valid), optionalNumberText(gap), seam, estimated
            };
            pointRows += row.join('\t').toUtf8() + '\n';
        };
        for (std::size_t index = 0; index < batch.fitPoints.size(); ++index) {
            appendPoint(QStringLiteral("fit_input"), index, batch.fitPoints[index], {}, {}, {});
        }
        for (std::size_t index = 0; index < batch.modelPoints.size(); ++index) {
            appendPoint(QStringLiteral("reference_model"), index, batch.modelPoints[index], {}, {}, {});
        }
        for (std::size_t index = 0; index < batch.pendingSamples.size(); ++index) {
            const auto &sample = batch.pendingSamples[index];
            appendPoint(QStringLiteral("raw_pending"), index, sample.point,
                        sample.gapWidthMm, QString::number(sample.simulatedSeamIndex), boolText(sample.estimated));
        }
        for (std::size_t index = 0; index < batch.releasedSamples.size(); ++index) {
            const auto &sample = batch.releasedSamples[index];
            appendPoint(QStringLiteral("released_target"), index, sample.point,
                        sample.gapWidthMm, QString::number(sample.simulatedSeamIndex), boolText(sample.estimated));
        }
    }
    if (!batchRows.isEmpty()) {
        written = appendData(QString::fromLatin1(kIcpBatchesFileName), batchRows) && written;
        written = appendData(QString::fromLatin1(kIcpPointsFileName), pointRows) && written;
    }
    const auto &follow = update.followDiagnostic;
    if (follow.valid) {
        QStringList fields{
            now, QString::number(m_updateCount), QString::number(update.state.segmentIndex),
            phaseText(update.phase), QString::number(update.taughtPointIndex),
            numberText(follow.s1Deg), numberText(follow.s2Deg)
        };
        for (const auto &point : {follow.current, follow.first, follow.directionPoint,
                                  follow.foot, follow.futureFoot}) {
            fields << pointValue(&point, 0) << pointValue(&point, 1) << pointValue(&point, 2);
        }
        fields << QString::number(update.realtimeCommand.dxUm)
               << QString::number(update.realtimeCommand.dyUm)
               << QString::number(update.realtimeCommand.dzUm)
               << QString::number(update.realtimeCommand.onlineEndFlag);
        written = appendFields(QString::fromLatin1(kFollowTargetsFileName), fields) && written;
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

    if (m_imageWriter) {
        m_imageWriter->stop();
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
    written = appendSummary(
        QStringLiteral("icp_batch_count"), QString::number(m_icpBatchCount)) && written;
    if (m_imageWriter && !m_imageWriter->lastError().isEmpty()) {
        m_lastError = m_imageWriter->lastError();
        written = appendSummary(QStringLiteral("vision_image_error"), m_lastError) && written;
        written = false;
    }
    written = appendSummary(QStringLiteral("vision_diagnostic_count"), QString::number(m_visionDiagnosticCount)) && written;
    written = appendSummary(QStringLiteral("vision_image_submission_count"), QString::number(m_imageSubmissionCount)) && written;
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
    // Non-vision events have no sample provenance; keep the appended columns empty.
    if (fileName == QString::fromLatin1(kRealtimeCorrectionFileName) && fields.size() == 23) {
        auto row = fields;
        row << QString() << QString() << QString();
        return appendData(fileName, row.join('\t').toUtf8() + '\n');
    }
    return appendData(fileName, fields.join('\t').toUtf8() + '\n');
}

bool COnlineCorrectionRunRecorder::appendData(
    const QString &fileName,
    const QByteArray &data)
{
    const QString path = QDir(m_sessionDirectory).filePath(fileName);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append)) {
        setFileError(QStringLiteral("追加"), path);
        return false;
    }
    if (file.write(data) != data.size()) {
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
