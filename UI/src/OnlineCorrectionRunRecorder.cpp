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
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace i2r_rlc::ui {

namespace {

constexpr auto kVisionDiagnosticFileName = "vision_diagnostics.tsv";

/** @brief 本地来源只描述上位机因果；缺失不以最新目标补造。 */
QJsonValue sessionCommandTrace(const std::optional<robot_comm::RobotLocalCommandTrace> &trace)
{
    if (!trace.has_value()) { return QJsonValue(QJsonValue::Null); }
    return QJsonObject{
        {QStringLiteral("session_id"), QString::number(trace->sessionId)},
        {QStringLiteral("command_id"), QString::number(trace->commandId)},
        {QStringLiteral("target_generation"), trace->targetGeneration.has_value()
            ? QJsonValue(QString::number(*trace->targetGeneration)) : QJsonValue(QJsonValue::Null)}};
}

/** @brief 原始曝光关联；缺失即 null，不从记录到达顺序推导因果。 */
QJsonValue sessionExposure(const std::optional<control::OnlineCorrectionExposureIdentity> &identity)
{
    if (!identity.has_value()) { return QJsonValue(QJsonValue::Null); }
    return QJsonObject{
        {QStringLiteral("session_id"), QString::number(identity->sessionId)},
        {QStringLiteral("frame_id"), QString::number(identity->frameId)},
        {QStringLiteral("reference_revision"), QString::number(identity->referenceRevision)}};
}
constexpr auto kSummaryFileName = "session_summary.txt";
constexpr auto kInitialCorrectionFileName = "initial_correction.txt";
constexpr auto kRealtimeCorrectionFileName = "realtime_correction.txt";
constexpr auto kActualRobotPathFileName = "actual_robot_path.txt";
constexpr auto kIcpBatchesFileName = "online_icp_batches.txt";
constexpr auto kIcpPointsFileName = "online_icp_points.txt";
constexpr auto kFollowTargetsFileName = "online_follow_targets.txt";
constexpr auto kSessionEventsFileName = "unified_session.jsonl";

/** @brief 记录点的真实有效标志；无效坐标不写成零值真值。 */
QJsonValue sessionPoint(const control::OnlineCorrectionPoint &point)
{
    if (!point.valid) { return QJsonValue(QJsonValue::Null); }
    return QJsonArray{point.xMm, point.yMm, point.zMm};
}

/** @brief 工程单位的机器人反馈，不把接收时间当成控制柜曝光时钟。 */
QJsonObject sessionRobotState(const robot_comm::RobotState &state)
{
    return QJsonObject{
        {QStringLiteral("tcp_mm"), QJsonArray{state.curx_r / 1000.0,
            state.cury_r / 1000.0, state.curz_r / 1000.0}},
        {QStringLiteral("rotation_deg"), QJsonArray{state.curRx_r / 10000.0,
            state.curRy_r / 10000.0, state.curRz_r / 10000.0}},
        {QStringLiteral("station_deg"), QJsonArray{state.curS1 / 10000.0, state.curS2 / 10000.0}},
        {QStringLiteral("s_start"), state.sStart}, {QStringLiteral("s_hold"), state.sHold},
        {QStringLiteral("traflag"), state.traflag}, {QStringLiteral("corflag"), state.corflag},
        {QStringLiteral("detflag"), state.detflag}};
}

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
        || result.pointSource == vision::OnlineSeamPointSource::GuidedEstimate
        || result.pointSource == vision::OnlineSeamPointSource::RecoveryTransition;
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
    m_lossImageWriter.reset();
    m_lossHistory.clear();
    m_lossTriggered = {};
    m_lossUnavailableTriggered = {};
    m_lossSeenValid = {};
    m_lossStage = -1;
    m_lossRejected = m_lossPostRemaining = 0;
    m_lossTriggerFrame = m_lossLastFrame = -1;
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
    m_sessionEventCount = 0;
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
    if (m_visionDiagnosticOptions.enabled
        && !createSessionFile(QString::fromLatin1(kSessionEventsFileName), {})) {
        if (errorMessage != nullptr) { *errorMessage = m_lastError; }
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
        "image_measurement_output_enabled\timage_observation_accepted\timage_search_expanded\t"
        "evidence_policy_enabled\tkcf_pixel_available\tcontrol_eligible\tevidence_fresh_exposure\t"
        "evidence_anchor_frame\tevidence_anchor_refreshed\tevidence_confirmation_count\t"
        "evidence_mother_rank\tevidence_identity_score\tevidence_elapsed_ms\tevidence_travel_mm\tevidence_message\tdebug_kcf_20261026_gates\n"))) {
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
                if (m_visionDiagnosticOptions.lossWindowEnabled) {
                    const QString lossDirectory = QDir(m_sessionDirectory).filePath(QStringLiteral("loss_window_images"));
                    if (!QDir().mkpath(lossDirectory)
                        || !createSessionFile(QStringLiteral("loss_window_images/image_manifest.tsv"),
                            QStringLiteral("stem\tstatus\tencoded_bytes\n"))
                        || !createSessionFile(QStringLiteral("loss_window_context.jsonl"), {})
                        || !createSessionFile(QStringLiteral("loss_window_index.tsv"),
                            QStringLiteral("debug_tag\ttrigger_frame\tframe_id\trole\trequest_status\tsource_image\n"))) {
                        if (errorMessage != nullptr) *errorMessage = m_lastError;
                        return false;
                    }
                    m_lossImageWriter = std::make_unique<CTrackingDiagnosticImageWriter>(
                        lossDirectory, 64LL * 1024 * 1024, 32);
                }
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
        && appendSummary(QStringLiteral("vision_diagnostics_schema"), QStringLiteral("9"))
        && appendSummary(QStringLiteral("debug_kcf_20261026_loss_window"),
            m_lossImageWriter ? QStringLiteral("pre=6;post=8;first_rejection_and_first_pixel_unavailable_per_stage;never_acquired_after_8_rejections;raw_history=16MiB;disk=64MiB;queue=32;processed_exposures_only;control_snapshot_at_result_receipt") : QStringLiteral("disabled"))
        && appendSummary(QStringLiteral("kcf_policy"), QStringLiteral("production_cpp_evidence_v2; online_continuous_kcf; measured_or_estimated_following; independent_geometry_only_icp"))
        && appendSummary(QStringLiteral("preserve_measured_reference"), boolText(configuration.options.preserveMeasuredReference))
        && appendSummary(QStringLiteral("vision_diagnostics_enabled"), boolText(m_visionDiagnosticOptions.enabled))
        && appendSummary(QStringLiteral("vision_images_enabled"), boolText(m_visionDiagnosticOptions.imagesEnabled))
        && appendSummary(QStringLiteral("vision_image_sample_every_n"), QString::number(m_visionDiagnosticOptions.imageSampleEveryN))
        && appendSummary(QStringLiteral("vision_max_image_pairs"), QString::number(m_visionDiagnosticOptions.maxImagePairs))
        && appendSummary(QStringLiteral("vision_max_image_bytes"), QString::number(m_visionDiagnosticOptions.maxImageBytes))
        && appendSummary(QStringLiteral("vision_image_sampling_policy"),
                         QStringLiteral("startup/online_sampling_early/middle/late/anomaly=3/6/6/6/9; quadratic_backoff; anomaly_minimum_interval_ms=60000; raw_deviation_px=10; layers_are_not_physical_seam_segments"))
        && appendSummary(QStringLiteral("icp_diagnostics_schema"), QStringLiteral("3"))
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

    return appendSessionEvent(QJsonObject{
        {QStringLiteral("event"), QStringLiteral("session_start")},
        {QStringLiteral("schema"), 1},
        {QStringLiteral("trajectory_icp_enabled"), configuration.options.enableOnlineIcp},
        {QStringLiteral("preserve_measured_reference"), configuration.options.preserveMeasuredReference},
        // 工件配准由 coordinator 管理；Control recorder 不具备其开关证据。
        {QStringLiteral("workpiece_online_icp_enabled"), QJsonValue(QJsonValue::Null)},
        {QStringLiteral("exposure_attribution"), QStringLiteral("observations_only; releases_not_propagated")}});
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
    return appendSessionEvent(QJsonObject{
        {QStringLiteral("event"), QStringLiteral("robot_state")},
        {QStringLiteral("phase"), phaseText(phase)},
        {QStringLiteral("state"), sessionRobotState(state)}}) && written;
}

bool COnlineCorrectionRunRecorder::recordTrackingResult(
    const RuntimeTrackingResult &result,
    const control::OnlineCorrectionPhase phase)
{
    return recordTrackingResult(result, phase, control::OnlineCorrectionTargetSnapshot{});
}

bool COnlineCorrectionRunRecorder::recordTrackingResult(
    const RuntimeTrackingResult &result,
    const control::OnlineCorrectionPhase phase,
    const control::OnlineCorrectionTargetSnapshot &target)
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
    const bool lossWritten = recordLossWindow(result, phase, target);
    QJsonObject observation{
        {QStringLiteral("event"), QStringLiteral("observation")},
        {QStringLiteral("phase"), phaseText(phase)},
        {QStringLiteral("session_id"), QString::number(result.correlation.sessionId)},
        {QStringLiteral("frame_id"), QString::number(result.frameId)},
        {QStringLiteral("reference_revision"), QString::number(result.correlation.workpiecePoseRevision)},
        {QStringLiteral("active_algorithm"), result.activeAlgorithm},
        {QStringLiteral("point_source"), static_cast<int>(result.pointSource)},
        {QStringLiteral("valid"), result.valid},
        {QStringLiteral("pixel_available"), result.kcfPixelAvailable},
        {QStringLiteral("control_eligible"), result.controlEligible},
        {QStringLiteral("localization_evidence"), QJsonObject{
            {QStringLiteral("input_unavailable"), result.localizationEvidence.inputUnavailable},
            {QStringLiteral("attempted"), result.localizationEvidence.attempted},
            {QStringLiteral("current_image_supported"), result.localizationEvidence.currentImageSupported},
            {QStringLiteral("image_constrained_estimate"), result.localizationEvidence.imageConstrainedEstimate},
            {QStringLiteral("recovery_attempted"), result.localizationEvidence.recoveryAttempted},
            {QStringLiteral("recovery_failed"), result.localizationEvidence.recoveryFailed}}},
        {QStringLiteral("estimated"), estimatedPoint(result)},
        {QStringLiteral("pixel"), result.kcfPixelAvailable
            ? QJsonValue(QJsonArray{result.imageXPixel, result.imageYPixel})
            : QJsonValue(QJsonValue::Null)},
        {QStringLiteral("space_parent_tcp_mm"), sessionPoint(result.sample.point)},
        {QStringLiteral("message"), result.message}};
    observation.insert(QStringLiteral("performance"), QJsonValue(QJsonValue::Null));
    if (result.diagnostic.has_value()) {
        const auto &diagnostic = *result.diagnostic;
        const auto timeValue = [](qint64 value) {
            return value >= 0 ? QJsonValue(QString::number(value)) : QJsonValue(QJsonValue::Null);
        };
        const auto &performance = diagnostic.algorithm.performance;
        observation.insert(QStringLiteral("performance"), QJsonObject{
            {QStringLiteral("processing_started_ms"), timeValue(diagnostic.processingStartedMs)},
            {QStringLiteral("processing_finished_ms"), timeValue(diagnostic.processingFinishedMs)},
            {QStringLiteral("runtime_received_ms"), timeValue(diagnostic.runtimeReceivedMs)},
            {QStringLiteral("image_preparation_ms"), diagnostic.imagePreparationMs >= 0
                ? QJsonValue(diagnostic.imagePreparationMs) : QJsonValue(QJsonValue::Null)},
            {QStringLiteral("detector"), performance.available ? QJsonValue(QJsonObject{
                {QStringLiteral("total_ms"), performance.totalMs},
                {QStringLiteral("input_bookkeeping_ms"), performance.inputBookkeepingMs},
                {QStringLiteral("simulation_ms"), performance.simulationMs},
                {QStringLiteral("initialization_ms"), performance.initializationMs},
                {QStringLiteral("tracking_ms"), performance.trackingMs},
                {QStringLiteral("validation_projection_ms"), performance.validationProjectionMs},
                {QStringLiteral("training_ms"), performance.trainingMs}}) : QJsonValue(QJsonValue::Null)}});
        observation.insert(QStringLiteral("camera_timestamp_ms"), QString::number(diagnostic.cameraTimestampMs));
        observation.insert(QStringLiteral("exposure_robot_state"), sessionRobotState(diagnostic.robotState));
        observation.insert(QStringLiteral("robot_state_received_ms"), QString::number(diagnostic.robotStateReceivedMs));
        const auto &algorithm = diagnostic.algorithm;
        // 原始已验收图像观测与融合先验单列；未执行融合时不补造先验或增益。
        observation.insert(QStringLiteral("position_fusion"), QJsonObject{
            {QStringLiteral("applied"), algorithm.positionFusionApplied},
            {QStringLiteral("trusted_prior"), algorithm.simulationPositionTrusted},
            {QStringLiteral("observation_px"), algorithm.imageObservationAccepted
                ? QJsonValue(QJsonArray{algorithm.positionObservationPoint.x,
                    algorithm.positionObservationPoint.y}) : QJsonValue(QJsonValue::Null)},
            {QStringLiteral("prior_px"), algorithm.positionFusionApplied
                ? QJsonValue(QJsonArray{algorithm.positionPriorPoint.x, algorithm.positionPriorPoint.y})
                : QJsonValue(QJsonValue::Null)},
            {QStringLiteral("local_bias_px"), algorithm.positionFusionApplied
                ? QJsonValue(QJsonArray{algorithm.simulationLocalBias.x, algorithm.simulationLocalBias.y})
                : QJsonValue(QJsonValue::Null)},
            {QStringLiteral("gain"), algorithm.positionFusionApplied
                ? QJsonValue(QJsonArray{algorithm.positionGainX, algorithm.positionGainY})
                : QJsonValue(QJsonValue::Null)}});
        // 同曝光原始仿真坐标和实际外观取样锚点分开记录，不能把重定位当位姿写回。
        observation.insert(QStringLiteral("simulation_prior"), QJsonObject{
            {QStringLiteral("source_point_px"), algorithm.simulatedPointAvailable
                ? QJsonValue(QJsonArray{algorithm.simulatedPoint.x, algorithm.simulatedPoint.y})
                : QJsonValue(QJsonValue::Null)},
            {QStringLiteral("position_in_image"), algorithm.simulationPositionInImage},
            {QStringLiteral("shape_available"), algorithm.simulationShapeAvailable},
            {QStringLiteral("template_rebased"), algorithm.simulationTemplateRebased},
            {QStringLiteral("template_point_px"), algorithm.simulationShapeAvailable
                ? QJsonValue(QJsonArray{algorithm.simulationTemplatePoint.x, algorithm.simulationTemplatePoint.y})
                : QJsonValue(QJsonValue::Null)}});
    }
    return appendSessionEvent(observation) && written && diagnosticWritten && lossWritten;
}

bool COnlineCorrectionRunRecorder::submitLossFrame(
    const CLossFrame &frame, qint64 triggerFrame, const QString &role)
{
    const QString status = frame.source.isNull() ? QStringLiteral("source_missing_or_oversize")
        : m_lossImageWriter->submit(frame.stem, frame.source, QImage(), 0, 64LL * 1024 * 1024);
    const bool indexed = appendFields(QStringLiteral("loss_window_index.tsv"), {
        QStringLiteral("[DEBUG-kcf-20261026]"), QString::number(triggerFrame),
        QString::number(frame.frameId), role, status,
        QStringLiteral("loss_window_images/%1_source.png").arg(frame.stem)});
    const bool contextWritten = appendData(QStringLiteral("loss_window_context.jsonl"), frame.context);
    return indexed && contextWritten;
}

bool COnlineCorrectionRunRecorder::recordLossWindow(
    const RuntimeTrackingResult &result, control::OnlineCorrectionPhase phase,
    const control::OnlineCorrectionTargetSnapshot &target)
{
    if (!m_lossImageWriter || !result.diagnostic.has_value() || !result.triggerAccepted
        || result.correlation.purpose != CorrectionPurpose::OnlineCorrection) {
        return true;
    }
    const int stage = phase == control::OnlineCorrectionPhase::InitialDetectionRunning ? 0
        : (phase == control::OnlineCorrectionPhase::OnlineRunning ? 1 : -1);
    if (stage < 0) { return true; }
    if (m_lossStage != stage) {
        m_lossStage = stage;
        m_lossHistory.clear();
        m_lossRejected = m_lossPostRemaining = 0;
        m_lossLastFrame = -1;
    }
    if (m_lossTriggered[stage] && m_lossUnavailableTriggered[stage]
        && m_lossPostRemaining == 0) { return true; }
    // 重复/乱序结果保留在数值日志中，不能伪装成窗口里的新增曝光。
    if (result.frameId < 0 || result.frameId <= m_lossLastFrame) { return true; }
    m_lossLastFrame = result.frameId;
    const auto &diagnostic = *result.diagnostic;
    const auto &algorithm = diagnostic.algorithm;
    const auto &evidence = algorithm.evidence;
    CLossFrame frame;
    frame.frameId = result.frameId;
    frame.stem = QStringLiteral("stage_%1_session_%2_frame_%3")
        .arg(stage).arg(result.correlation.sessionId).arg(result.frameId);
    if (diagnostic.lossWindowSourceImage.sizeInBytes() <= 16LL * 1024 * 1024) {
        frame.source = diagnostic.lossWindowSourceImage;
    }
    QJsonObject context;
    context["debug_tag"] = QStringLiteral("[DEBUG-kcf-20261026]");
    context["image"] = QStringLiteral("loss_window_images/%1_source.png").arg(frame.stem);
    context["image_availability"] = QStringLiteral("consult loss_window_index.tsv and image_manifest.tsv; queued is not saved");
    context["kind"] = QStringLiteral("real");
    context["frame_id"] = result.frameId;
    context["session_id"] = static_cast<qint64>(result.correlation.sessionId);
    context["timestamp_ms"] = diagnostic.cameraTimestampMs;
    context["robot_state_received_ms"] = diagnostic.robotStateReceivedMs;
    context["phase"] = phaseText(phase);
    context["reference_revision"] = static_cast<qint64>(result.correlation.workpiecePoseRevision);
    context["bounded_search"] = result.correlation.onlineControlWindowActive;
    context["evidence_policy"] = result.evidencePolicyEnabled;
    context["valid"] = result.valid;
    context["control_eligible"] = result.controlEligible;
    context["gate_detail"] = QString::fromStdString(evidence.debugDetail);
    const auto &options = evidence.debugOptions;
    context["evidence_options"] = QJsonObject{
        {"enable_mother_constraints", options.enableMotherConstraints},
        {"enable_continuous_output", options.enableContinuousOutput},
        {"max_continuous_step_pixel", options.maxContinuousStepPixel},
        {"anchor_confirmation_frames", options.anchorConfirmationFrames},
        {"recovery_confirmation_frames", options.recoveryConfirmationFrames},
        {"identity_corridor_pixel", options.identityCorridorPixel},
        {"identity_min_patch_score", options.identityMinPatchScore},
        {"identity_min_branch_score", options.identityMinBranchScore},
        {"identity_min_uniqueness", options.identityMinUniqueness},
        {"max_recovery_step_pixel", options.recoveryMaxStepPixel},
        {"max_prediction_ms", options.maxPredictionMs},
        {"max_prediction_travel_mm", options.maxPredictionTravelMm}};
    if (algorithm.simulatedPointAvailable) {
        const QJsonArray reference{algorithm.simulatedPoint.x, algorithm.simulatedPoint.y};
        context["sim_point"] = reference;
        context["reference_point_at_exposure"] = reference;
        context["reference_exposure_frame_id"] = result.frameId;
    }
    // 记录完整曝光投影链，不用单位位姿或收到结果时的新位姿替代。
    const auto transform = [](const vision::WeldRigidTransform &value) {
        QJsonArray array;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) { array.append(value.rotation(row, column)); }
        }
        for (int axis = 0; axis < 3; ++axis) { array.append(value.translationMm(axis)); }
        return array;
    };
    if (result.displayProjectionContext.has_value() && result.displayProjectionContext->valid) {
        const auto &projection = *result.displayProjectionContext;
        const auto &calibration = projection.calibration;
        context["projection"] = QJsonObject{
            {"provenance", QStringLiteral("frozen exposure context from Vision worker")},
            {"calibration", QJsonObject{{"focal_length_m", calibration.focalLengthM},
                {"sx_m", calibration.sxM}, {"sy_m", calibration.syM},
                {"cx_pixel", calibration.intrinsics.cxPixel}, {"cy_pixel", calibration.intrinsics.cyPixel},
                {"kappa", calibration.kappa}, {"laser_plane_a", calibration.laserPlaneA},
                {"laser_plane_b", calibration.laserPlaneB}, {"laser_plane_c", calibration.laserPlaneC}}},
            {"camera_in_tool", transform(calibration.cameraInTool)},
            {"tool_in_robot_base", transform(projection.toolInRobotBase)},
            {"robot_base_to_positioner_tcp", transform(projection.robotBaseToPositionerTcp)}};
    }
    const auto point = [](const control::OnlineCorrectionPoint &value) {
        return QJsonObject{{"valid", value.valid}, {"xyz_mm", QJsonArray{value.xMm, value.yMm, value.zMm}}};
    };
    context["control_reference"] = QJsonObject{
        {"association", QStringLiteral("controller_snapshot_before_consuming_this_result; NOT exposure-time or UI-time target")},
        {"session_id", static_cast<qint64>(target.sessionId)}, {"generation", static_cast<qint64>(target.generation)},
        {"valid", target.valid}, {"source", static_cast<int>(target.source)},
        {"geometry_valid", target.geometry.valid}, {"first", point(target.geometry.first)},
        {"direction_point", point(target.geometry.directionPoint)},
        {"target", point(target.targetPointInLastLink_mm)}};
    if (result.displayPositionerCalibration.has_value()) {
        const auto &offset = result.displayPositionerCalibration->toolTcpInLastLinkMm;
        context["positioner_tool_tcp_offset_mm"] = QJsonArray{offset.x(), offset.y(), offset.z()};
    }
    frame.context = QJsonDocument(context).toJson(QJsonDocument::Compact) + '\n';
    const bool accepted = result.valid && (!result.evidencePolicyEnabled || result.controlEligible);
    m_lossSeenValid[stage] = m_lossSeenValid[stage] || accepted;
    m_lossRejected = accepted ? 0 : m_lossRejected + 1;
    const bool pixelUnavailable = !(result.evidencePolicyEnabled ? result.kcfPixelAvailable : result.valid);
    const bool firstRejection = !m_lossTriggered[stage] && !accepted
        && (m_lossSeenValid[stage] || m_lossRejected >= 8);
    // 可见估计失去控制资格可能先触发首拒绝；仍为随后真正的像素消失保留一次窗口。
    // 两类共用16MiB历史及64MiB写盘预算，不改变检测和控制判据。
    const bool firstUnavailable = !m_lossUnavailableTriggered[stage] && pixelUnavailable
        && (m_lossSeenValid[stage] || m_lossRejected >= 8);
    bool written = true;
    if (firstRejection || (firstUnavailable && m_lossPostRemaining == 0)) {
        m_lossTriggered[stage] = true;
        m_lossUnavailableTriggered[stage] = m_lossUnavailableTriggered[stage] || pixelUnavailable;
        m_lossTriggerFrame = result.frameId;
        for (const auto &previous : m_lossHistory) {
            written = submitLossFrame(previous, m_lossTriggerFrame, QStringLiteral("before")) && written;
        }
        written = submitLossFrame(frame, m_lossTriggerFrame, QStringLiteral("trigger")) && written;
        m_lossPostRemaining = 8;
        m_lossHistory.clear();
    } else if (m_lossPostRemaining > 0) {
        const bool startsUnavailableWindow = !m_lossUnavailableTriggered[stage] && pixelUnavailable;
        m_lossUnavailableTriggered[stage] = m_lossUnavailableTriggered[stage] || pixelUnavailable;
        written = submitLossFrame(frame, m_lossTriggerFrame, QStringLiteral("after")) && written;
        // 两窗口重叠时不重复写当前帧，但仍保留首次像素消失之后的八帧恢复证据。
        m_lossPostRemaining = startsUnavailableWindow ? 8 : m_lossPostRemaining - 1;
    } else if (!m_lossTriggered[stage] || !m_lossUnavailableTriggered[stage]) {
        m_lossHistory.push_back(std::move(frame));
        qint64 bytes = 0;
        for (const auto &previous : m_lossHistory) { bytes += previous.source.sizeInBytes(); }
        while (!m_lossHistory.empty() && (m_lossHistory.size() > 6U || bytes > 16LL * 1024 * 1024)) {
            bytes -= m_lossHistory.front().source.sizeInBytes();
            m_lossHistory.pop_front();
        }
    }
    return written;
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
    const auto &evidence = algorithm.evidence;
    fields << boolText(result.evidencePolicyEnabled) << boolText(result.kcfPixelAvailable)
           << boolText(result.controlEligible) << boolText(evidence.freshExposure)
           << QString::number(evidence.anchorFrameId) << boolText(evidence.anchorRefreshed)
           << QString::number(evidence.confirmationCount) << QString::number(evidence.motherSupportRank)
           << numberText(evidence.identityScore) << numberText(evidence.elapsedMs)
           << numberText(evidence.travelMm) << sanitizeField(QString::fromStdString(evidence.message))
           << sanitizeField(QString::fromStdString(evidence.debugDetail));
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

    return recordSessionUpdate(update) && written;
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
        for (std::size_t index = 0; index < batch.modelProposalSamples.size(); ++index) {
            const auto &sample = batch.modelProposalSamples[index];
            appendPoint(QStringLiteral("model_proposal_not_released"), index, sample.point,
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
    return appendSessionEvent(QJsonObject{
        {QStringLiteral("event"), QStringLiteral("receipt")},
        {QStringLiteral("local_trace"), sessionCommandTrace(receipt.localTrace)},
        {QStringLiteral("local_transaction_id"), receipt.localTransactionId == 0
            ? QJsonValue(QJsonValue::Null) : QJsonValue(QString::number(receipt.localTransactionId))},
        {QStringLiteral("execution_semantics"), QStringLiteral("unknown_execution")},
        {QStringLiteral("delivery"), receipt.responseReceived
            ? QStringLiteral("response_received") : QStringLiteral("delivery_unknown")},
        {QStringLiteral("phase"), phaseText(phase)},
        {QStringLiteral("type"), static_cast<int>(receipt.type)},
        {QStringLiteral("point_index"), receipt.pointIndex},
        {QStringLiteral("sent"), receipt.sent},
        {QStringLiteral("response_received"), receipt.responseReceived},
        {QStringLiteral("completed"), receipt.completed},
        {QStringLiteral("request"), QString::fromStdString(receipt.request)},
        {QStringLiteral("response"), QString::fromStdString(receipt.response)},
        {QStringLiteral("error"), static_cast<int>(receipt.error)},
        {QStringLiteral("state"), sessionRobotState(state)}}) && written;
}

bool COnlineCorrectionRunRecorder::recordOperationStop(
    quint64 sessionId,
    const robot_comm::RobotOperationStopState &state)
{
    const QString status = state.status == robot_comm::RobotOperationStopStatus::HoldConfirmed
        ? QStringLiteral("hold_confirmed")
        : state.status == robot_comm::RobotOperationStopStatus::Unconfirmed
            ? QStringLiteral("unconfirmed") : QStringLiteral("pending");
    return appendSessionEvent(QJsonObject{
        {QStringLiteral("event"), QStringLiteral("operation_stop")},
        {QStringLiteral("session_id"), QString::number(sessionId)},
        {QStringLiteral("request_generation"), QString::number(state.requestGeneration)},
        {QStringLiteral("status"), status},
        {QStringLiteral("hold_request_latched"), state.holdRequestLatched},
        {QStringLiteral("arc_stop_verified"), state.arcStopVerified},
        {QStringLiteral("error"), static_cast<int>(state.error)},
        {QStringLiteral("message"), QString::fromStdString(state.message)}});
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
    if (m_lossImageWriter) {
        m_lossImageWriter->stop();
        m_lossHistory.clear();
    }
    const qint64 finishedAtMs = QDateTime::currentMSecsSinceEpoch();
    bool written = true;
    if (m_lossImageWriter && !m_lossImageWriter->lastError().isEmpty()) {
        m_lastError = m_lossImageWriter->lastError();
        written = appendSummary(QStringLiteral("debug_kcf_20261026_loss_image_error"), m_lastError);
        written = false;
    }
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
    written = appendSessionEvent(QJsonObject{
        {QStringLiteral("event"), QStringLiteral("session_end")},
        {QStringLiteral("result"), resultText}, {QStringLiteral("message"), message}}) && written;
    m_active = false;
    return written;
}

bool COnlineCorrectionRunRecorder::recordSessionUpdate(const control::OnlineCorrectionUpdate &update)
{
    if (!m_visionDiagnosticOptions.enabled) { return true; }
    bool written = appendSessionEvent(QJsonObject{
        {QStringLiteral("event"), QStringLiteral("control_update")},
        {QStringLiteral("update_index"), QString::number(m_updateCount)},
        {QStringLiteral("phase"), phaseText(update.phase)},
        {QStringLiteral("queued_samples"), static_cast<qint64>(update.state.onlineDetectionSamples.size())},
        {QStringLiteral("request_operation_stop"), update.requestOperationStop},
        {QStringLiteral("source_exposure"), QJsonValue(QJsonValue::Null)},
        {QStringLiteral("message"), QString::fromStdString(update.message)}});
    int batchIndex = 0;
    for (const auto &batch : update.onlineIcpBatches) {
        ++batchIndex;
        int pointIndex = 0;
        for (const auto &sample : batch.releasedSamples) {
            written = appendSessionEvent(QJsonObject{
                {QStringLiteral("event"), QStringLiteral("release")},
                {QStringLiteral("update_index"), QString::number(m_updateCount)},
                {QStringLiteral("batch_index"), batchIndex},
                {QStringLiteral("point_index"), pointIndex++},
                {QStringLiteral("source_exposure"), sessionExposure(sample.sourceExposure)},
                {QStringLiteral("space_last_link_mm"), sessionPoint(sample.point)},
                {QStringLiteral("estimated"), sample.estimated},
                {QStringLiteral("trajectory_icp_attempted"), batch.attempted},
                {QStringLiteral("trajectory_icp_applied"), batch.applied}}) && written;
        }
    }
    if (update.followDiagnostic.valid) {
        const auto &follow = update.followDiagnostic;
        written = appendSessionEvent(QJsonObject{
            {QStringLiteral("event"), QStringLiteral("control_target")},
            {QStringLiteral("update_index"), QString::number(m_updateCount)},
            {QStringLiteral("target_generation"), QString::number(update.state.controlTarget.generation)},
            {QStringLiteral("source_exposure"), sessionExposure(follow.firstSourceExposure)},
            {QStringLiteral("direction_source_exposure"), sessionExposure(follow.directionSourceExposure)},
            {QStringLiteral("target_source"), static_cast<int>(update.state.controlTarget.source)},
            {QStringLiteral("current_last_link_mm"), sessionPoint(follow.current)},
            {QStringLiteral("line_first_mm"), sessionPoint(follow.first)},
            {QStringLiteral("line_direction_point_mm"), sessionPoint(follow.directionPoint)},
            {QStringLiteral("future_foot_mm"), sessionPoint(follow.futureFoot)}}) && written;
    }
    if (update.command.valid) {
        QJsonArray points;
        const int count = std::min(update.command.pointCount, static_cast<int>(update.command.teachPoints.size()));
        for (int index = 0; index < count; ++index) {
            const auto &point = update.command.teachPoints[static_cast<std::size_t>(index)];
            points.append(QJsonArray{point.xUm / 1000.0, point.yUm / 1000.0, point.zUm / 1000.0});
        }
        written = appendSessionEvent(QJsonObject{
            {QStringLiteral("event"), QStringLiteral("command")},
            {QStringLiteral("update_index"), QString::number(m_updateCount)},
            {QStringLiteral("type"), QStringLiteral("teach_points")},
            {QStringLiteral("delivery"), QStringLiteral("generated_not_sent")},
            {QStringLiteral("points_robot_base_mm"), points}}) && written;
    }
    if (update.realtimeCommand.valid) {
        const auto &command = update.realtimeCommand;
        written = appendSessionEvent(QJsonObject{
            {QStringLiteral("local_trace"), sessionCommandTrace(command.localTrace)},
            {QStringLiteral("event"), QStringLiteral("command")},
            {QStringLiteral("update_index"), QString::number(m_updateCount)},
            {QStringLiteral("type"), QStringLiteral("realtime")},
            {QStringLiteral("target_generation"), update.followDiagnostic.valid
                ? QJsonValue(QString::number(update.state.controlTarget.generation))
                : QJsonValue(QJsonValue::Null)},
            {QStringLiteral("delivery"), QStringLiteral("generated_not_sent")},
            {QStringLiteral("increment_um"), QJsonArray{static_cast<double>(command.dxUm),
                static_cast<double>(command.dyUm), static_cast<double>(command.dzUm)}},
            {QStringLiteral("speed_override"), static_cast<double>(command.speedOverride)},
            {QStringLiteral("online_end_flag"), command.onlineEndFlag}}) && written;
    }
    return written;
}

bool COnlineCorrectionRunRecorder::appendSessionEvent(QJsonObject event)
{
    if (!m_active || !m_visionDiagnosticOptions.enabled) { return true; }
    event.insert(QStringLiteral("sequence"), static_cast<qint64>(++m_sessionEventCount));
    event.insert(QStringLiteral("recorded_at_ms"), timestampMsText());
    return appendData(QString::fromLatin1(kSessionEventsFileName),
        QJsonDocument(event).toJson(QJsonDocument::Compact) + '\n');
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
