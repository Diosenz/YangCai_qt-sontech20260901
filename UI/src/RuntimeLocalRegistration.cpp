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
#include <QMetaObject>
#include <QPointer>
#include <QSettings>

#include "AppPaths.h"

namespace i2r_rlc::ui {
namespace {

constexpr qint64 MAXIMUM_LOCAL_RESULT_AGE_MS = 1500;

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
    integer("maximum_solve_time_ms", options.maximumSolveTimeMs, 20, 1000);
    real("max_correspondence_distance_mm", options.maximumCorrespondenceDistanceMm, 0.1, 10.0);
    real("maximum_translation_mm", options.maximumTranslationMm, 0.01, 10.0);
    real("maximum_rotation_deg", options.maximumRotationDeg, 0.01, 2.0);
    real("minimum_improvement_ratio", options.minimumImprovementRatio, 0.01, 0.9);
    real("maximum_residual_mm", options.maximumResidualMm, 0.01, 5.0);
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
    emit statusMessage(QStringLiteral("在线局部定位：%1，采样间隔 %2 ms，配准间隔至少 %3 ms，窗口 %4~%5 帧，每侧最多 %6 点，对应距离 %7 mm，最大平移修正 %8 mm。")
        .arg(m_localRegistrationOptions.enabled ? QStringLiteral("启用") : QStringLiteral("停用"))
        .arg(m_localRegistrationOptions.sampleIntervalMs)
        .arg(m_localRegistrationOptions.minimumMatchIntervalMs)
        .arg(m_localRegistrationOptions.minimumFrames)
        .arg(m_localRegistrationOptions.maximumFrames)
        .arg(m_localRegistrationOptions.maximumPointsPerCloud)
        .arg(m_localRegistrationOptions.maximumCorrespondenceDistanceMm)
        .arg(m_localRegistrationOptions.maximumTranslationMm));
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
        ? std::clamp(distanceMm, 0.1, 10.0) : 5.0;
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
        ? std::clamp(translationMm, 0.01, 10.0) : 5.0;
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

void CRuntimeCoordinator::submitOnlineLocalRegistration(const FrozenVisionFrameContext &context)
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
    if (!m_localRegistrationOptions.enabled || m_localRegistrationFaulted
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

bool CRuntimeCoordinator::dispatchOnlineLocalRegistrationWriteback()
{
    if (m_localRegistrationWritebackBusy) {
        return true;
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
            const auto written = worker->applyLocalRegistrationCorrection(result.generation, result.correction);
            const bool writeAttempted = worker->localRegistrationWriteAttempted();
            const qint64 elapsedMs = timer.elapsed();
            if (!coordinator.isNull()) {
                QMetaObject::invokeMethod(coordinator, [coordinator, result, requestId, written, writeAttempted, elapsedMs]() {
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
                    if (!written.succeeded) {
                        coordinator->m_localRegistrationFaulted = true;
                    }
                    // 无论成功与否都不再次应用同一矩阵；失败可能已发送但未回读确认。
                    coordinator->resetOnlineLocalRegistration();
                    coordinator->m_pendingVisionFrame.reset();
                    const QString message = QStringLiteral("[在线局部定位] 工件写回%1，耗时=%2 ms：%3%4")
                        .arg(written.succeeded ? QStringLiteral("已确认") : QStringLiteral("未确认"))
                        .arg(elapsedMs).arg(written.message)
                        .arg(written.succeeded ? QString() : QStringLiteral("；本轮自动更新停用，纠偏继续。"));
                    qInfo().noquote() << message;
                    emit coordinator->statusMessage(message);
                }, Qt::QueuedConnection);
            }
        }, Qt::QueuedConnection);
    return true;
}

} // namespace i2r_rlc::ui
