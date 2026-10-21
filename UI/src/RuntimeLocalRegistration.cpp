/** @file RuntimeLocalRegistration.cpp
 * @brief 在线表面定位的后台调度、代次隔离及 RobotSim 写回。
 * @details 与主视觉、机器人控制分离；最多一个采样任务和一个候选，不累计任务队列。
 */
#include "RuntimeCoordinator.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include <QDebug>
#include <QDateTime>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QPointer>
#include <QSaveFile>
#include <QSettings>
#include <QTimer>

#include "AppPaths.h"

namespace i2r_rlc::ui {
namespace {

constexpr qint64 MAXIMUM_LOCAL_RESULT_AGE_MS = 4000;
constexpr int LOCAL_VERIFICATION_TIMEOUT_MS = 5000;

localization::Point3D_mm localPoint(const control::OnlineCorrectionPoint &point)
{
    return {point.xMm, point.yMm, point.zMm};
}

bool usableIndependentPoint(const RuntimeTrackingResult &result)
{
    const auto &point = result.independentWeldPointInParent_mm;
    return result.valid && result.triggerAccepted && point.valid
        && std::isfinite(point.xMm) && std::isfinite(point.yMm) && std::isfinite(point.zMm);
}

bool usableIndependentReference(
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    qint64 frameId, int parentId)
{
    const auto &point = observation.weldPoint_mm;
    return observation.valid && observation.frameId == frameId
        && parentId > 0 && observation.referenceObjectId == parentId
        && observation.hasWeldPoint_mm && std::isfinite(point.x_mm)
        && std::isfinite(point.y_mm) && std::isfinite(point.z_mm);
}

// 在 RobotSim worker 线程写入事务快照，不在 UI 线程执行磁盘操作。
bool saveLocalWritebackSnapshot(const QString &path, const QString &phase,
    const CGlobalLocatePriorWorkerAdapter::LocalRegistrationSafetySnapshot &snapshot,
    const digital_twin::DigitalTwinCommandResult &command, const QJsonArray &samples)
{
    QJsonObject object;
    object.insert(QStringLiteral("phase"), phase);
    object.insert(QStringLiteral("succeeded"), command.succeeded);
    object.insert(QStringLiteral("message"), command.message);
    object.insert(QStringLiteral("post_write_samples"), samples);
    object.insert(QStringLiteral("has_pending_pose"), snapshot.hasPendingPose);
    object.insert(QStringLiteral("pending_pose_confirmed"), snapshot.pendingPoseConfirmed);
    object.insert(QStringLiteral("rollback_attempted"), snapshot.rollbackAttempted);
    object.insert(QStringLiteral("verified_by_independent_observation"), snapshot.verifiedByIndependentObservation);
    object.insert(QStringLiteral("workpiece_id"), snapshot.workpieceId);
    object.insert(QStringLiteral("parent_id"), snapshot.parentId);
    object.insert(QStringLiteral("epoch"), QString::number(snapshot.pendingEpoch));
    object.insert(QStringLiteral("scene_revision"), QString::number(snapshot.sceneRevision));
    object.insert(QStringLiteral("cumulative_translation_mm"), snapshot.cumulativeTranslationMm);
    object.insert(QStringLiteral("cumulative_rotation_deg"), snapshot.cumulativeRotationDeg);
    object.insert(QStringLiteral("maximum_keypoint_displacement_mm"), snapshot.maximumKeypointDisplacementMm);
    const auto addMatrix = [&object](const char *key, const localization::RigidTransform &transform) {
        QJsonArray matrix;
        for (double value : transform.rowMajor) { matrix.append(value); }
        object.insert(QString::fromLatin1(key), matrix);
    };
    addMatrix("baseline_parent_from_workpiece", snapshot.baselineParentFromWorkpiece);
    addMatrix("verified_parent_from_workpiece", snapshot.verifiedParentFromWorkpiece);
    addMatrix("before_parent_from_workpiece", snapshot.beforeParentFromWorkpiece);
    addMatrix("pending_parent_from_workpiece", snapshot.pendingParentFromWorkpiece);
    QSaveFile file(path + QStringLiteral(".") + phase + QStringLiteral(".json"));
    const QByteArray bytes = QJsonDocument(object).toJson();
    return file.open(QIODevice::WriteOnly)
        && file.write(bytes) == bytes.size() && file.commit();
}

/** @brief 非法现场参数停用自动写回；不静默使用危险的零间隔或无界点数。 */
COnlineLocalRegistrationWorker::Options readLocalOptions(QString &error)
{
    COnlineLocalRegistrationWorker::Options options;
    QSettings settings(AppPaths::fromDefaultLocation().digitalTwinConfigDir()
        .filePath(QStringLiteral("online_local_registration.ini")), QSettings::IniFormat);
    options.enabled = settings.value(QStringLiteral("enabled"), true).toBool();
    const auto integer = [&settings, &error](const char *key, int &value, int low, int high) {
        bool ok = false;
        const QString name = QString::fromLatin1(key);
        const int candidate = settings.value(name, value).toInt(&ok);
        if (!ok || candidate < low || candidate > high) {
            error = QStringLiteral("局部定位配置 %1 超出允许范围 [%2,%3]。").arg(name).arg(low).arg(high);
        } else {
            value = candidate;
        }
    };
    const auto real = [&settings, &error](const char *key, double &value, double low, double high) {
        bool ok = false;
        const QString name = QString::fromLatin1(key);
        const double candidate = settings.value(name, value).toDouble(&ok);
        if (!ok || !std::isfinite(candidate) || candidate < low || candidate > high) {
            error = QStringLiteral("局部定位配置 %1 无效或超出范围。").arg(name);
        } else {
            value = candidate;
        }
    };
    integer("sample_interval_ms", options.sampleIntervalMs, 100, 2000);
    integer("minimum_match_interval_ms", options.minimumMatchIntervalMs, 1000, 30000);
    integer("minimum_frames", options.minimumFrames, 4, 32);
    integer("maximum_frames", options.maximumFrames, 4, 32);
    integer("maximum_points_per_cloud", options.maximumPointsPerCloud, 64, 1024);
    integer("maximum_iterations", options.maximumIterations, 5, 80);
    integer("maximum_solve_time_ms", options.maximumSolveTimeMs, 20, 2500);
    real("max_correspondence_distance_mm", options.maximumCorrespondenceDistanceMm, 0.1, 10.0);
    real("maximum_translation_mm", options.maximumTranslationMm, 0.01, 30.0);
    real("maximum_rotation_deg", options.maximumRotationDeg, 0.01, 10.0);
    real("minimum_improvement_ratio", options.minimumImprovementRatio, 0.01, 0.9);
    real("maximum_residual_mm", options.maximumResidualMm, 0.01, 5.5);
    real("maximum_independent_error_mm", options.maximumIndependentErrorMm, 0.01, 10.0);
    real("maximum_independent_worsening_mm", options.maximumIndependentWorseningMm, 0.0, 0.5);
    real("coverage_distance_mm", options.coverageDistanceMm, 0.1, 10.0);
    real("minimum_bidirectional_coverage", options.minimumBidirectionalCoverage, 0.5, 1.0);
    real("minimum_matched_spread_ratio", options.minimumMatchedSpreadRatio, 0.1, 1.0);
    integer("maximum_archived_attempts_per_session", options.maximumArchivedAttemptsPerSession, 1, 1024);
    if (options.maximumFrames < options.minimumFrames || settings.status() != QSettings::NoError) {
        error = QStringLiteral("局部定位配置读取失败或最大窗口小于最小窗口。");
    }
    if (!error.isEmpty()) {
        options.enabled = false;
    }
    return options;
}

} // namespace

void CRuntimeCoordinator::startLocalRegistrationWorker()
{
    QString error;
    m_localRegistrationOptions = readLocalOptions(error);
    QSettings safetySettings(AppPaths::fromDefaultLocation().digitalTwinConfigDir()
        .filePath(QStringLiteral("online_local_registration.ini")), QSettings::IniFormat);
    const auto readLimit = [&safetySettings, &error](const char *key, double &limit, double maximum) {
        bool ok = false;
        const double value = safetySettings.value(QString::fromLatin1(key), limit).toDouble(&ok);
        if (!ok || !std::isfinite(value) || value <= 0.0 || value > maximum) {
            error = QStringLiteral("局部定位累计保护参数 %1 无效。").arg(QString::fromLatin1(key));
        } else { limit = value; }
    };
    readLimit("maximum_cumulative_translation_mm", m_localRegistrationSafetyOptions.maximumCumulativeTranslationMm, 100.0);
    readLimit("maximum_cumulative_rotation_deg", m_localRegistrationSafetyOptions.maximumCumulativeRotationDeg, 30.0);
    readLimit("maximum_keypoint_displacement_mm", m_localRegistrationSafetyOptions.maximumKeypointDisplacementMm, 100.0);
    if (!error.isEmpty()) { m_localRegistrationOptions.enabled = false; }
    m_localRegistrationConfigurationValid = error.isEmpty();
    if (!error.isEmpty()) {
        emit statusMessage(error);
    }
    m_localRegistrationWorker = new COnlineLocalRegistrationWorker(nullptr);
    m_localRegistrationWorker->configure(m_localRegistrationOptions);
    m_localRegistrationWorker->moveToThread(&m_localRegistrationThread);
    connect(&m_localRegistrationThread, &QThread::finished,
        m_localRegistrationWorker, &QObject::deleteLater);
    m_localRegistrationThread.setObjectName(QStringLiteral("online-local-registration"));
    m_localRegistrationThread.start();
    m_localRegistrationClock.start();
    m_lastLocalRegistrationAttemptMs = -1;
    ++m_localRegistrationRequestId;
    m_localRegistrationBusy = false;
    m_localRegistrationWritebackBusy = false;
    resetOnlineLocalRegistration();
    emit statusMessage(QStringLiteral("在线局部定位：%1，采样间隔 %2 ms，配准间隔至少 %3 ms，窗口 %4~%5 帧，每侧最多 %6 点，对应距离 %7 mm，最大平移修正 %8 mm，最大旋转 %9 度，配准预算 %10 ms，候选有效期 %11 ms，残差上限 %12 mm，最低改善 %13%。")
        .arg(m_localRegistrationOptions.enabled ? QStringLiteral("启用") : QStringLiteral("停用"))
        .arg(m_localRegistrationOptions.sampleIntervalMs)
        .arg(m_localRegistrationOptions.minimumMatchIntervalMs)
        .arg(m_localRegistrationOptions.minimumFrames)
        .arg(m_localRegistrationOptions.maximumFrames)
        .arg(m_localRegistrationOptions.maximumPointsPerCloud)
        .arg(m_localRegistrationOptions.maximumCorrespondenceDistanceMm)
        .arg(m_localRegistrationOptions.maximumTranslationMm)
        .arg(m_localRegistrationOptions.maximumRotationDeg)
        .arg(m_localRegistrationOptions.maximumSolveTimeMs)
        .arg(MAXIMUM_LOCAL_RESULT_AGE_MS)
        .arg(m_localRegistrationOptions.maximumResidualMm)
        .arg(m_localRegistrationOptions.minimumImprovementRatio * 100.0));
}

bool CRuntimeCoordinator::onlineLocalRegistrationEnabled() const
{
    return m_localRegistrationOptions.enabled;
}

vision::KcfFusionOptions CRuntimeCoordinator::onlineFusionOptions() const
{
    return m_trackingOptions.fusion;
}

double CRuntimeCoordinator::onlineLocalIcpDistance() const
{
    return m_localRegistrationOptions.maximumCorrespondenceDistanceMm;
}

void CRuntimeCoordinator::setOnlineLocalIcpDistance(double distanceMm)
{
    const double normalized = std::isfinite(distanceMm)
        ? std::clamp(distanceMm, 0.1, 10.0) : 10.0;
    if (m_localRegistrationOptions.maximumCorrespondenceDistanceMm == normalized) {
        return;
    }
    m_localRegistrationOptions.maximumCorrespondenceDistanceMm = normalized;
    // 旧阈值产生的候选必须失效；保留在途事务 busy、配准冷却和写回故障保护。
    resetOnlineLocalRegistration();
    if (m_localRegistrationWorker != nullptr) {
        const auto options = m_localRegistrationOptions;
        QMetaObject::invokeMethod(m_localRegistrationWorker,
            [worker = m_localRegistrationWorker, options]() {
                worker->configure(options);
            }, Qt::QueuedConnection);
    }
    emit statusMessage(QStringLiteral("在线局部 ICP 最大对应距离已调整为 %1 mm，重新积累匹配窗口。")
        .arg(normalized, 0, 'f', 1));
}

double CRuntimeCoordinator::onlineLocalIcpTranslationLimit() const
{
    return m_localRegistrationOptions.maximumTranslationMm;
}

void CRuntimeCoordinator::setOnlineLocalIcpTranslationLimit(double translationMm)
{
    const double normalized = std::isfinite(translationMm)
        ? std::clamp(translationMm, 0.01, 30.0) : 30.0;
    if (m_localRegistrationOptions.maximumTranslationMm == normalized) {
        return;
    }
    m_localRegistrationOptions.maximumTranslationMm = normalized;
    // 不调整搜索距离和质量门槛；旧候选必须按新限幅重新求解和验收。
    resetOnlineLocalRegistration();
    if (m_localRegistrationWorker != nullptr) {
        const auto options = m_localRegistrationOptions;
        QMetaObject::invokeMethod(m_localRegistrationWorker,
            [worker = m_localRegistrationWorker, options]() {
                worker->configure(options);
            }, Qt::QueuedConnection);
    }
    emit statusMessage(QStringLiteral("在线局部 ICP 最大平移修正已调整为 %1 mm，重新积累匹配窗口。")
        .arg(normalized, 0, 'f', 2));
}

void CRuntimeCoordinator::updateOnlineProcessingOptions(
    bool localEnabled, const vision::KcfFusionOptions &fusion)
{
    if (localEnabled && !m_localRegistrationConfigurationValid) {
        localEnabled = false;
        emit statusMessage(QStringLiteral("局部定位配置无效，无法通过界面开启；请修正配置后重新启动。"));
    }
    vision::KcfFusionOptions normalized = fusion;
    switch (normalized.mode) {
    case vision::KcfFusionMode::AutomaticKalman:
    case vision::KcfFusionMode::ManualWeight:
    case vision::KcfFusionMode::KcfOnly:
        break;
    default:
        normalized.mode = vision::KcfFusionMode::AutomaticKalman;
        break;
    }
    normalized.simulationWeight = std::isfinite(normalized.simulationWeight)
        ? std::max(0.0, std::min(1.0, normalized.simulationWeight)) : 0.5;
    const bool fusionChanged = m_trackingOptions.fusion.mode != normalized.mode
        || m_trackingOptions.fusion.simulationWeight != normalized.simulationWeight;
    m_trackingOptions.fusion = normalized;
    if (fusionChanged && m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(m_visionWorker,
            [worker = m_visionWorker, normalized]() {
                worker->updateOnlineFusionOptions(normalized);
            }, Qt::QueuedConnection);
    }
    if (m_localRegistrationOptions.enabled != localEnabled) {
        m_localRegistrationOptions.enabled = localEnabled;
        // 先失效写回令牌，再排队清空窗口；保留在途计算 busy 和写回故障保护。
        resetOnlineLocalRegistration();
        if (m_localRegistrationWorker != nullptr) {
            const auto options = m_localRegistrationOptions;
            QMetaObject::invokeMethod(m_localRegistrationWorker,
                [worker = m_localRegistrationWorker, options]() {
                    worker->configure(options);
                }, Qt::QueuedConnection);
        }
        emit statusMessage(localEnabled
            ? (m_localRegistrationFaulted
                ? QStringLiteral("在线 ICP 已选择开启，但本会话位姿写回故障保护仍生效。")
                : QStringLiteral("在线 ICP 局部定位已开启，重新积累真实局部点云。"))
            : QStringLiteral("在线 ICP 局部定位已关闭，已撤销待执行工件位姿更新。"));
    }
}

void CRuntimeCoordinator::resetOnlineLocalRegistration()
{
    if (m_localVerification.has_value()) {
        // 用户停止、切换配置或失去状态时仅撤销事务；不可在撤销后自动操作场景。
        m_localVerification.reset();
        m_localRegistrationFaulted = true;
        emit statusMessage(QStringLiteral("局部更新后验验证已撤销，本会话暂停自动更新；工件未自动恢复。"));
    }
    ++m_localRegistrationGeneration;
    if (m_localRegistrationGeneration == 0) {
        ++m_localRegistrationGeneration;
    }
    m_pendingLocalCorrection.reset();
    m_lastLocalRegistrationSampleMs = -1;
    if (m_robotSimWorker != nullptr) {
        m_robotSimWorker->setLocalRegistrationEpoch(
            m_robotSimConnected && !m_localRegistrationFaulted
                && m_localRegistrationOptions.enabled
                && m_trackingPurpose == RuntimeTrackingPurpose::OnlineCorrection
                && m_hasLatestRobotState && m_latestRobotState.sStart == 1
                && m_latestRobotState.sHold == 0 && m_latestRobotState.sMode == 2
                && m_correctionSession.active && m_correctionSession.onlineControlWindowActive
                ? m_localRegistrationGeneration : 0);
    }
    if (m_localRegistrationWorker != nullptr) {
        QMetaObject::invokeMethod(m_localRegistrationWorker,
            [worker = m_localRegistrationWorker]() { worker->reset(); }, Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::submitOnlineLocalRegistration(const FrozenVisionFrameContext &context,
    const RuntimeTrackingResult &trackingResult)
{
    const qint64 wallTimeMs = QDateTime::currentMSecsSinceEpoch();
    if (context.robotStateReceivedMs < 0 || m_latestRobotStateReceivedMs < 0
        || wallTimeMs - context.robotStateReceivedMs > 500
        || wallTimeMs - m_latestRobotStateReceivedMs > 500) {
        if (m_localRegistrationRobotStateFresh) {
            m_localRegistrationRobotStateFresh = false;
            resetOnlineLocalRegistration();
        }
        return;
    }
    // 先撤销失效主观测对应的在途候选，再检查 busy/采样节流；否则旧候选可能
    // 在本帧 KCF 丢失之后才完成，并错误进入场景写回。
    if (m_localRegistrationOptions.enabled && !m_localRegistrationFaulted
        && !m_localVerification.has_value() && m_correctionSession.active
        && m_trackingPurpose == RuntimeTrackingPurpose::OnlineCorrection
        && context.correctionSessionId == m_correctionSession.sessionId
        && context.modelGeneration == m_localRegistrationGeneration
        && (trackingResult.frameId != context.frame.frameId
            || !usableIndependentPoint(trackingResult)
            || !context.synchronizedObservation.has_value()
            || !usableIndependentReference(*context.synchronizedObservation,
                context.frame.frameId, m_globalLocateRobotSimScene.workpieceParentId))) {
        resetOnlineLocalRegistration();
        return;
    }
    if (!m_localRegistrationOptions.enabled || m_localRegistrationFaulted || m_localVerification.has_value()
        || m_localRegistrationBusy || m_localRegistrationWritebackBusy
        || m_pendingLocalCorrection.has_value() || m_localRegistrationWorker == nullptr
        || !m_robotSimConnected || m_latestRobotState.sStart != 1
        || m_latestRobotState.sHold != 0 || m_latestRobotState.sMode != 2
        || !m_correctionSession.active || !m_correctionSession.onlineControlWindowActive
        || m_trackingPurpose != RuntimeTrackingPurpose::OnlineCorrection
        || context.correctionSessionId != m_correctionSession.sessionId
        || context.modelGeneration != m_localRegistrationGeneration
        || !context.synchronizedObservation.has_value()
        || !m_onlinePositionerCalibration.has_value() || !m_onlinePositionerCalibration->valid) {
        return;
    }
    const auto &observation = *context.synchronizedObservation;
    if (trackingResult.frameId != context.frame.frameId || !usableIndependentPoint(trackingResult)
        || !usableIndependentReference(observation, context.frame.frameId,
            m_globalLocateRobotSimScene.workpieceParentId)) {
        // 主跟踪失效不能继续积累局部窗口，更不能用受仿真先验影响的融合点自证正确。
        resetOnlineLocalRegistration();
        return;
    }
    if (m_globalLocateRobotSimScene.workpieceParentId <= 0
        || observation.referenceObjectId != m_globalLocateRobotSimScene.workpieceParentId
        || observation.laserLinePoints_mm.size() < 8U) {
        return;
    }
    const qint64 now = m_localRegistrationClock.elapsed();
    if (m_lastLocalRegistrationAttemptMs >= 0
        && now - m_lastLocalRegistrationAttemptMs < m_localRegistrationOptions.minimumMatchIntervalMs) {
        return;
    }
    if (m_lastLocalRegistrationSampleMs >= 0
        && now - m_lastLocalRegistrationSampleMs < m_localRegistrationOptions.sampleIntervalMs) {
        return;
    }
    COnlineLocalRegistrationWorker::Input input;
    input.sessionId = context.correctionSessionId;
    input.generation = context.modelGeneration;
    input.frame = context.frame;
    input.robotState = context.robotState;
    input.observation = observation;
    input.trackingOptions = onlineTrackingOptions();
    input.projectionContext = m_trackingProjectionContext;
    input.independentObservation.valid = true;
    input.independentObservation.frameId = context.frame.frameId;
    input.independentObservation.realWeldPointInParent_mm = localPoint(trackingResult.independentWeldPointInParent_mm);
    input.independentObservation.virtualWeldPointInParent_mm = {
        observation.weldPoint_mm.x_mm, observation.weldPoint_mm.y_mm, observation.weldPoint_mm.z_mm};
    input.archiveDirectory = m_localRegistrationArchiveDirectory;
    m_lastLocalRegistrationSampleMs = now;
    m_localRegistrationInputMs = now;
    m_localRegistrationBusy = true;
    const quint64 requestId = ++m_localRegistrationRequestId;
    const QPointer<CRuntimeCoordinator> coordinator(this);
    QMetaObject::invokeMethod(m_localRegistrationWorker,
        [coordinator, worker = m_localRegistrationWorker, input, requestId]() {
            COnlineLocalRegistrationWorker::Result result;
            result.sessionId = input.sessionId;
            result.generation = input.generation;
            result.frameId = input.frame.frameId;
            try {
                result = worker->process(input);
            } catch (const std::exception &exception) {
                result.attempted = true;
                result.message = QStringLiteral("局部定位异常：%1").arg(QString::fromUtf8(exception.what()));
            } catch (...) {
                result.attempted = true;
                result.message = QStringLiteral("局部定位未知异常，跳过本次修正。");
            }
            if (!coordinator.isNull()) {
                QMetaObject::invokeMethod(coordinator, [coordinator, requestId, result]() {
                    if (!coordinator.isNull()) {
                        coordinator->handleOnlineLocalRegistrationResult(requestId, result);
                    }
                }, Qt::QueuedConnection);
            }
        }, Qt::QueuedConnection);
}

void CRuntimeCoordinator::handleOnlineLocalRegistrationResult(
    quint64 requestId, const COnlineLocalRegistrationWorker::Result &result)
{
    if (requestId != m_localRegistrationRequestId) {
        return;
    }
    m_localRegistrationBusy = false;
    // 热调参数会撤销结果，但已经发生的求解仍应计入冷却，避免连续调参增加负载。
    if (result.attempted && m_running && !m_stopping) {
        m_lastLocalRegistrationAttemptMs = m_localRegistrationClock.elapsed();
    }
    if (!m_running || m_stopping || !m_correctionSession.active
        || m_trackingPurpose != RuntimeTrackingPurpose::OnlineCorrection
        || !m_correctionSession.onlineControlWindowActive
        || result.sessionId != m_correctionSession.sessionId
        || result.generation != m_localRegistrationGeneration) {
        return;
    }
    if (result.attempted) {
        const QString message = QStringLiteral("[在线局部定位] frame=%1，窗口=%2 帧/%3 点，残差 %4→%5 mm，采集及处理=%6 ms，求解=%7 ms：%8")
            .arg(result.frameId).arg(result.frameCount).arg(result.pointCount)
            .arg(result.beforeResidualMm, 0, 'f', 4).arg(result.afterResidualMm, 0, 'f', 4)
            .arg(result.elapsedMs).arg(result.solveElapsedMs).arg(result.message);
        qInfo().noquote() << message;
        emit statusMessage(message);
    }
    if (result.accepted
        && m_localRegistrationClock.elapsed() - m_localRegistrationInputMs <= MAXIMUM_LOCAL_RESULT_AGE_MS) {
        m_pendingLocalCorrection = result;
        dispatchOnlineLocalRegistrationWriteback();
    }
}

void CRuntimeCoordinator::verifyOnlineLocalRegistration(
    const FrozenVisionFrameContext &context, const RuntimeTrackingResult &trackingResult)
{
    if (!m_localVerification.has_value()) { return; }
    auto &verification = *m_localVerification;
    if (!m_running || m_stopping || !m_correctionSession.active
        || context.correctionSessionId != verification.sessionId
        || context.modelGeneration != verification.generation) {
        return;
    }
    // 不能使用写回前曝光的缓存帧验证写回后的效果。
    if (context.frame.frameId <= verification.sourceFrameId
        || !context.frame.timestamp.isValid()
        || context.frame.timestamp.toMSecsSinceEpoch() <= verification.completedAtMs) {
        return;
    }
    localization::Point3D_mm virtualPoint;
    bool valid = trackingResult.frameId == context.frame.frameId
        && usableIndependentPoint(trackingResult)
        && context.synchronizedObservation.has_value();
    if (context.synchronizedObservation.has_value()) {
        const auto &observation = *context.synchronizedObservation;
        valid = valid && usableIndependentReference(observation, context.frame.frameId,
            m_globalLocateRobotSimScene.workpieceParentId);
        virtualPoint = {observation.weldPoint_mm.x_mm, observation.weldPoint_mm.y_mm,
            observation.weldPoint_mm.z_mm};
    }
    verification.check->observe(context.frame.frameId, valid,
        localPoint(trackingResult.independentWeldPointInParent_mm), virtualPoint);
    if (verification.samples.size() < 64) {
        verification.samples.append(QJsonObject{
            {QStringLiteral("frame_id"), QString::number(context.frame.frameId)},
            {QStringLiteral("capture_time_ms"), QString::number(context.frame.timestamp.toMSecsSinceEpoch())},
            {QStringLiteral("valid"), valid},
            {QStringLiteral("current_error_mm"), verification.check->currentErrorMm()},
            {QStringLiteral("counterfactual_previous_error_mm"), verification.check->previousErrorMm()}});
    }
    if (verification.check->decision() == COnlineLocalRegistrationVerification::Decision::Rollback) {
        verification.rollbackRequested = true;
    }
}

bool CRuntimeCoordinator::finishOnlineLocalRegistrationVerification()
{
    if (!m_localVerification.has_value()) { return false; }
    const auto verification = *m_localVerification;
    if (!verification.rollbackRequested
        && verification.check->decision() == COnlineLocalRegistrationVerification::Decision::Waiting) {
        return false;
    }
    if (!m_running || m_stopping || !m_correctionSession.active
        || verification.sessionId != m_correctionSession.sessionId
        || verification.generation != m_localRegistrationGeneration
        || !m_localRegistrationOptions.enabled || !m_robotSimConnected
        || m_robotSimWorker == nullptr || m_latestRobotState.sStart != 1
        || m_latestRobotState.sHold != 0 || m_latestRobotState.sMode != 2
        || QDateTime::currentMSecsSinceEpoch() - m_latestRobotStateReceivedMs > 500) {
        // 停止或外部状态不可确认时，不自动回滚覆盖场景。
        resetOnlineLocalRegistration();
        return false;
    }
    if (m_visionFrameInFlight || m_robotSimPriorInFlightRequestId != 0) { return false; }
    m_localRegistrationWritebackBusy = true;
    m_robotSimWorker->setLocalRegistrationStateDeadline(m_latestRobotStateReceivedMs + 500);
    const quint64 requestId = m_localRegistrationRequestId;
    const QPointer<CRuntimeCoordinator> coordinator(this);
    QMetaObject::invokeMethod(m_robotSimWorker,
        [coordinator, worker = m_robotSimWorker, verification, requestId]() {
            const bool rollback = verification.rollbackRequested;
            const auto command = rollback
                ? worker->rollbackLocalRegistrationCorrection(verification.generation)
                : worker->markLocalRegistrationVerified(verification.generation);
            const bool archived = saveLocalWritebackSnapshot(verification.archivePath,
                rollback ? QStringLiteral("rollback") : QStringLiteral("verified"),
                worker->localRegistrationSafetySnapshot(), command, verification.samples);
            if (!coordinator.isNull()) {
                QMetaObject::invokeMethod(coordinator,
                    [coordinator, verification, command, archived, rollback, requestId]() {
                        if (coordinator.isNull()
                            || requestId != coordinator->m_localRegistrationRequestId) { return; }
                        coordinator->m_localRegistrationWritebackBusy = false;
                        if (!coordinator->m_running || coordinator->m_stopping
                            || !coordinator->m_localVerification.has_value()
                            || verification.sessionId != coordinator->m_correctionSession.sessionId
                            || verification.generation != coordinator->m_localRegistrationGeneration) {
                            return;
                        }
                        coordinator->m_localVerification.reset();
                        // 恢复成功也保持暂停，防止重复进入同一个错误匹配区域。
                        if (rollback || !command.succeeded || !archived) {
                            coordinator->m_localRegistrationFaulted = true;
                        }
                        coordinator->resetOnlineLocalRegistration();
                        coordinator->m_pendingVisionFrame.reset();
                        const QString message = QStringLiteral("[在线局部定位] %1：%2；事务归档%3。%4")
                            .arg(rollback ? QStringLiteral("后验验证失败，受保护恢复") : QStringLiteral("后验验证"))
                            .arg(command.message)
                            .arg(archived ? QStringLiteral("成功") : QStringLiteral("失败"))
                            .arg(coordinator->m_localRegistrationFaulted
                                ? QStringLiteral("本会话自动更新暂停，纠偏继续。")
                                : QStringLiteral("已通过连续新帧独立检查，重新采样。"));
                        qInfo().noquote() << message;
                        emit coordinator->statusMessage(message);
                        coordinator->dispatchPendingVisionFrame();
                    }, Qt::QueuedConnection);
            }
        }, Qt::QueuedConnection);
    return true;
}

bool CRuntimeCoordinator::dispatchOnlineLocalRegistrationWriteback()
{
    if (m_localRegistrationWritebackBusy) {
        return true;
    }
    if (m_localVerification.has_value()) {
        return finishOnlineLocalRegistrationVerification();
    }
    if (!m_pendingLocalCorrection.has_value()) {
        return false;
    }
    const auto result = *m_pendingLocalCorrection;
    if (!m_running || m_stopping || m_localRegistrationFaulted
        || m_trackingPurpose != RuntimeTrackingPurpose::OnlineCorrection
        || !m_correctionSession.active || !m_correctionSession.onlineControlWindowActive
        || result.sessionId != m_correctionSession.sessionId
        || result.generation != m_localRegistrationGeneration
        || m_localRegistrationClock.elapsed() - m_localRegistrationInputMs > MAXIMUM_LOCAL_RESULT_AGE_MS
        || QDateTime::currentMSecsSinceEpoch() - m_latestRobotStateReceivedMs > 500
        || m_latestRobotState.sStart != 1 || m_latestRobotState.sHold != 0
        || m_latestRobotState.sMode != 2
        || m_robotSimWorker == nullptr || !m_robotSimConnected) {
        m_pendingLocalCorrection.reset();
        return false;
    }
    if (m_visionFrameInFlight || m_robotSimPriorInFlightRequestId != 0) {
        return false;
    }
    m_pendingLocalCorrection.reset();
    m_localRegistrationWritebackBusy = true;
    const quint64 requestId = m_localRegistrationRequestId;
    m_robotSimWorker->setLocalRegistrationStateDeadline(m_latestRobotStateReceivedMs + 500);
    const QPointer<CRuntimeCoordinator> coordinator(this);
    QMetaObject::invokeMethod(m_robotSimWorker,
        [coordinator, worker = m_robotSimWorker, result, requestId]() {
            QElapsedTimer timer;
            timer.start();
            worker->setLocalRegistrationKeypointsInParent({result.independentObservation.virtualWeldPointInParent_mm});
            const auto written = worker->applyLocalRegistrationCorrection(result.generation, result.correction);
            const bool writeAttempted = worker->localRegistrationWriteAttempted();
            const bool archived = saveLocalWritebackSnapshot(result.archivePath, QStringLiteral("writeback"),
                worker->localRegistrationSafetySnapshot(), written, {});
            const qint64 elapsedMs = timer.elapsed();
            if (!coordinator.isNull()) {
                QMetaObject::invokeMethod(coordinator, [coordinator, result, requestId, written, writeAttempted, archived, elapsedMs]() {
                    if (coordinator.isNull() || !coordinator->m_running || coordinator->m_stopping
                        || requestId != coordinator->m_localRegistrationRequestId) {
                        return;
                    }
                    coordinator->m_localRegistrationWritebackBusy = false;
                    if (!coordinator->m_correctionSession.active
                        || result.sessionId != coordinator->m_correctionSession.sessionId) {
                        return;
                    }
                    // 开关切换只能撤销未发送的事务；已经尝试写入的结果仍需处理。
                    if (result.generation != coordinator->m_localRegistrationGeneration
                        && !writeAttempted) {
                        return;
                    }
                    if (!written.succeeded || !archived) {
                        coordinator->m_localRegistrationFaulted = true;
                    }
                    // 已发送的旧代次结果不能开启新一轮，也不能在停止/切换后自动恢复。
                    if (written.succeeded && archived
                        && result.generation == coordinator->m_localRegistrationGeneration) {
                        LocalVerification verification;
                        verification.sessionId = result.sessionId;
                        verification.generation = result.generation;
                        verification.completedAtMs = QDateTime::currentMSecsSinceEpoch();
                        verification.sourceFrameId = result.frameId;
                        verification.archivePath = result.archivePath;
                        COnlineLocalRegistrationVerification::Options options;
                        options.maximumErrorMm = coordinator->m_localRegistrationOptions.maximumIndependentErrorMm;
                        options.maximumWorseningMm = coordinator->m_localRegistrationOptions.maximumIndependentWorseningMm;
                        verification.check.emplace(result.correction, options);
                        coordinator->m_localVerification = verification;
                        QTimer::singleShot(LOCAL_VERIFICATION_TIMEOUT_MS, coordinator,
                            [coordinator, sessionId = result.sessionId, generation = result.generation]() {
                                if (!coordinator.isNull() && coordinator->m_localVerification.has_value()
                                    && coordinator->m_localVerification->sessionId == sessionId
                                    && coordinator->m_localVerification->generation == generation) {
                                    coordinator->m_localVerification->rollbackRequested = true;
                                    coordinator->dispatchPendingVisionFrame();
                                }
                            });
                    } else {
                        if (writeAttempted) { coordinator->m_localRegistrationFaulted = true; }
                        coordinator->resetOnlineLocalRegistration();
                    }
                    coordinator->m_pendingVisionFrame.reset();
                    const QString message = QStringLiteral("[在线局部定位] 工件写回%1，耗时=%2 ms：%3%4")
                        .arg(written.succeeded ? QStringLiteral("指令已回读确认（几何尚未验证）") : QStringLiteral("未确认"))
                        .arg(elapsedMs).arg(written.message)
                        .arg(!archived ? QStringLiteral("；事务归档失败，本轮自动更新停用。")
                            : (coordinator->m_localRegistrationFaulted
                                ? QStringLiteral("；本轮自动更新停用，纠偏继续。")
                                : QStringLiteral("；等待后续新采样验证。")));
                    qInfo().noquote() << message;
                    emit coordinator->statusMessage(message);
                    if (coordinator->m_localVerification.has_value()) {
                        emit coordinator->statusMessage(QStringLiteral("局部工件指令已回读，等待后续新采样验证；验证完成前暂停追加修正。"));
                    }
                    coordinator->dispatchPendingVisionFrame();
                }, Qt::QueuedConnection);
            }
        }, Qt::QueuedConnection);
    return true;
}

} // namespace i2r_rlc::ui
