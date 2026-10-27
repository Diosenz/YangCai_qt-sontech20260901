/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        RuntimeCoordinator.cpp
 * @brief       Qt 运行时线程编排器实现
 * @details     创建运行时 worker 线程并通过 queued signal/slot 保证跨线程同步边界。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <utility>

#include <QDateTime>
#include <QMetaObject>
#include <QPointer>
#include <QTimer>

#include "RuntimeCoordinator.h"
#include "AppPaths.h"

namespace i2r_rlc::ui {

namespace {

constexpr qint64 ONLINE_CAMERA_FRAME_MAXIMUM_AGE_MS = 1000;
constexpr int ONLINE_CAMERA_PREFLIGHT_TIMEOUT_MS = 3000;
constexpr int ONLINE_WORKER_ARM_TIMEOUT_MS = 3000;
constexpr int ONLINE_PROTOCOL_PREFLIGHT_TIMEOUT_MS = 5000;
constexpr int ONLINE_JOB_RECEIPT_TIMEOUT_MS = 5000;
constexpr int ONLINE_JOB_MOTION_TIMEOUT_MS = 15000;
constexpr int ONLINE_DETECTION_WINDOW_TIMEOUT_MS = 60000;

void invokeWorkerBlocking(QObject *worker, const char *member)
{
    if (worker == nullptr) {
        return;
    }

    const Qt::ConnectionType connectionType =
        worker->thread() == QThread::currentThread()
        ? Qt::DirectConnection
        : Qt::BlockingQueuedConnection;
    QMetaObject::invokeMethod(worker, member, connectionType);
}

} // namespace

CRuntimeCoordinator::CRuntimeCoordinator(QObject *parent)
    : CRuntimeCoordinator(
          std::make_unique<robot_comm::CClientRobotCommandTransport>(),
          parent)
{
}

CRuntimeCoordinator::CRuntimeCoordinator(
    std::unique_ptr<robot_comm::IRobotCommandTransport> robotTransport,
    QObject *parent)
    : CRuntimeCoordinator(
          std::move(robotTransport),
          std::unique_ptr<digital_twin::IRobotSimTransport>{},
          parent)
{
}

CRuntimeCoordinator::CRuntimeCoordinator(
    std::unique_ptr<robot_comm::IRobotCommandTransport> robotTransport,
    std::unique_ptr<digital_twin::IRobotSimTransport> robotSimTransport,
    QObject *parent)
    : QObject(parent)
    , m_robotTransport(std::move(robotTransport))
    , m_robotSimTransport(std::move(robotSimTransport))
{
    registerMetaTypes();
}

CRuntimeCoordinator::~CRuntimeCoordinator()
{
    stop();
}

void CRuntimeCoordinator::start()
{
    if (m_running) {
        return;
    }

    m_cameraWorker = new CCameraWorkerAdapter(
        std::move(m_cameraDevice),
        m_cameraCaptureIntervalMs);
    m_robotWorker = new CRobotCommWorkerAdapter(std::move(m_robotTransport));
    m_visionWorker = new CVisionTrackingWorkerAdapter;
    m_globalLocatePriorWorker = new CGlobalLocatePriorWorkerAdapter;
    m_globalLocalizationWorker = new CGlobalLocalizationWorkerAdapter;
    // 生产路径保留 worker 的默认真实 transport；只有测试显式注入时才转移
    // 替代 transport，避免为了测试引入第二套 DigitalTwin 服务抽象。
    m_robotSimWorker = m_robotSimTransport
        ? new CGlobalLocatePriorWorkerAdapter(std::move(m_robotSimTransport))
        : new CGlobalLocatePriorWorkerAdapter;
    m_onlineCorrectionWorker = new COnlineCorrectionWorkerAdapter;
    m_offlineCorrectionWorker = new COfflineCorrectionWorkerAdapter;
    m_teachTrajectoryWorker = new CTeachTrajectoryRecorderWorkerAdapter;
    m_visionWorker->setSimulatedCallbacks(m_callbacks);

    m_cameraWorker->moveToThread(&m_cameraThread);
    m_robotWorker->moveToThread(&m_robotThread);
    m_visionWorker->moveToThread(&m_visionThread);
    m_globalLocatePriorWorker->moveToThread(&m_globalLocatePriorThread);
    m_globalLocalizationWorker->moveToThread(&m_globalLocalizationThread);
    m_robotSimWorker->moveToThread(&m_robotSimThread);
    m_onlineCorrectionWorker->moveToThread(&m_onlineCorrectionThread);
    m_offlineCorrectionWorker->moveToThread(&m_offlineCorrectionThread);
    m_teachTrajectoryWorker->moveToThread(&m_teachTrajectoryThread);

    connect(&m_cameraThread, &QThread::finished, m_cameraWorker, &QObject::deleteLater);
    connect(&m_robotThread, &QThread::finished, m_robotWorker, &QObject::deleteLater);
    connect(&m_visionThread, &QThread::finished, m_visionWorker, &QObject::deleteLater);
    connect(
        &m_globalLocatePriorThread,
        &QThread::finished,
        m_globalLocatePriorWorker,
        &QObject::deleteLater);
    connect(
        &m_globalLocalizationThread,
        &QThread::finished,
        m_globalLocalizationWorker,
        &QObject::deleteLater);
    connect(
        &m_robotSimThread,
        &QThread::finished,
        m_robotSimWorker,
        &QObject::deleteLater);
    connect(&m_onlineCorrectionThread, &QThread::finished, m_onlineCorrectionWorker, &QObject::deleteLater);
    connect(&m_offlineCorrectionThread, &QThread::finished, m_offlineCorrectionWorker, &QObject::deleteLater);
    connect(&m_teachTrajectoryThread, &QThread::finished, m_teachTrajectoryWorker, &QObject::deleteLater);

    connectWorkers();

    m_cameraThread.setObjectName(QStringLiteral("camera-worker"));
    m_robotThread.setObjectName(QStringLiteral("robotcomm-worker"));
    m_visionThread.setObjectName(QStringLiteral("vision-tracking-worker"));
    m_globalLocatePriorThread.setObjectName(QStringLiteral("digital-twin-prior-worker"));
    m_globalLocalizationThread.setObjectName(
        QStringLiteral("global-localization-worker"));
    m_robotSimThread.setObjectName(QStringLiteral("robotsim-sync-worker"));
    m_onlineCorrectionThread.setObjectName(QStringLiteral("online-correction-worker"));
    m_offlineCorrectionThread.setObjectName(QStringLiteral("offline-correction-worker"));
    m_teachTrajectoryThread.setObjectName(QStringLiteral("teach-trajectory-worker"));

    m_cameraThread.start();
    m_robotThread.start();
    m_visionThread.start();
    m_globalLocatePriorThread.start();
    m_globalLocalizationThread.start();
    m_robotSimThread.start();
    m_onlineCorrectionThread.start();
    m_offlineCorrectionThread.start();
    m_teachTrajectoryThread.start();
    m_running = true;
    startLocalRegistrationWorker();

    emit statusMessage(QStringLiteral(
        "运行时线程已启动；相机和 RobotComm 等待 UI 自动启动/连接"));
}

void CRuntimeCoordinator::stop()
{
    if (!m_running) {
        return;
    }

    m_stopping = true;
    cancelOnlineCorrectionStart();
    invalidateCorrectionSession(QStringLiteral("运行时停止，纠偏会话已清理。"));
    m_trackingPurpose = RuntimeTrackingPurpose::None;
    resetVisionFrameDispatch();
    invokeWorkerBlocking(m_visionWorker, "stopTracking");
    invokeWorkerBlocking(m_onlineCorrectionWorker, "reset");
    invokeWorkerBlocking(m_offlineCorrectionWorker, "reset");
    invokeWorkerBlocking(m_teachTrajectoryWorker, "reset");
    invokeWorkerBlocking(m_robotSimWorker, "resetRobotSim");
    invokeWorkerBlocking(m_cameraWorker, "stop");
    invokeWorkerBlocking(m_robotWorker, "disconnectRobot");

    shutdownThread(m_cameraThread);
    shutdownThread(m_robotThread);
    shutdownThread(m_visionThread);
    shutdownThread(m_globalLocatePriorThread);
    shutdownThread(m_globalLocalizationThread);
    shutdownThread(m_localRegistrationThread);
    shutdownThread(m_robotSimThread);
    shutdownThread(m_onlineCorrectionThread);
    shutdownThread(m_offlineCorrectionThread);
    shutdownThread(m_teachTrajectoryThread);
    m_cameraWorker = nullptr;
    m_robotWorker = nullptr;
    m_visionWorker = nullptr;
    m_globalLocatePriorWorker = nullptr;
    m_globalLocalizationWorker = nullptr;
    m_localRegistrationWorker = nullptr;
    m_localRegistrationBusy = false;
    m_robotSimWorker = nullptr;
    m_onlineCorrectionWorker = nullptr;
    m_offlineCorrectionWorker = nullptr;
    m_teachTrajectoryWorker = nullptr;
    m_trackingPurpose = RuntimeTrackingPurpose::None;
    m_robotSimConnected = false;
    m_robotSimManualModeRequested = false;
    m_robotSimConnectionActiveOrPending = false;
    m_hasPendingRobotSimSyncState = false;
    m_robotSimSyncInFlightRequestId = 0;
    m_robotSimHasSynchronizationStatus = false;
    m_robotSimLastSynchronizationMessage.clear();
    m_hasLatestCameraFrame = false;
    m_lastCameraFrameReceivedMs = 0;
    m_running = false;
    m_stopping = false;
    emit statusMessage(QStringLiteral("运行时线程已停止"));
}

void CRuntimeCoordinator::setSimulatedCallbacks(const SimulatedVisionCallbacks &callbacks)
{
    m_callbacks = callbacks;
    if (m_visionWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_visionWorker,
        [worker = m_visionWorker, callbacks]() {
            worker->setSimulatedCallbacks(callbacks);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::configureTracking(
    const RuntimeTrackingOptions &options,
    const vision::WeldProjectionContext &projectionContext)
{
    m_trackingOptions = options;
    m_trackingProjectionContext = projectionContext;
    resetOnlineLocalRegistration();
    if (m_visionWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_visionWorker,
        "configure",
        Qt::QueuedConnection,
        Q_ARG(i2r_rlc::ui::RuntimeTrackingOptions, options),
        Q_ARG(i2r_rlc::vision::WeldProjectionContext, projectionContext));
}

void CRuntimeCoordinator::configureOfflineCorrectionArchive(
    const std::string &archiveFilePath,
    const control::OfflineCorrectionOptions &options)
{
    if (m_offlineCorrectionWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_offlineCorrectionWorker,
        [worker = m_offlineCorrectionWorker, archiveFilePath, options]() {
            worker->configure(archiveFilePath, options);
        },
        Qt::QueuedConnection);
}

bool CRuntimeCoordinator::setCameraDevice(
    std::unique_ptr<camera::ICameraDevice> device,
    int captureIntervalMs)
{
    if (m_running || !device) {
        return false;
    }
    m_cameraDevice = std::move(device);
    m_cameraCaptureIntervalMs = std::max(1, captureIntervalMs);
    return true;
}

void CRuntimeCoordinator::startCamera()
{
    if (m_cameraWorker == nullptr) {
        emit statusMessage(QStringLiteral("实时相机启动失败：Camera worker 尚未启动"));
        return;
    }

    QMetaObject::invokeMethod(m_cameraWorker, "start", Qt::QueuedConnection);
}

void CRuntimeCoordinator::configureOnlineCorrection(
    const RuntimeOnlineCorrectionConfiguration &configuration)
{
    RuntimeOnlineCorrectionConfiguration resolved = configuration;
    if (!resolved.positionerCalibrationSnapshot.has_value()) {
        resolved.positionerCalibrationSnapshot =
            vision::CPositionerCalibrationParser::parseFile(resolved.rbcalibFile);
    }
    m_onlinePositionerCalibration = resolved.positionerCalibrationSnapshot;
    resetOnlineLocalRegistration();
    if (m_onlineCorrectionWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_onlineCorrectionWorker,
        "configure",
        Qt::QueuedConnection,
        Q_ARG(i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration, resolved));
}

RuntimeTrackingOptions CRuntimeCoordinator::onlineTrackingOptions() const
{
    RuntimeTrackingOptions options = m_trackingOptions;
    options.visionDiagnostics = m_pendingOnlineCorrectionConfiguration.visionDiagnostics;
    if (m_onlinePositionerCalibration.has_value()) {
        options.positionerCalibration = *m_onlinePositionerCalibration;
    }
    return options;
}

void CRuntimeCoordinator::connectRobot()
{
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(m_robotWorker, "connectRobot", Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::setCameraExposureUs(int exposureTimeUs)
{
    if (m_cameraWorker == nullptr) {
        emit statusMessage(QStringLiteral("相机曝光设置失败：Camera worker 尚未启动"));
        return;
    }
    QMetaObject::invokeMethod(
        m_cameraWorker,
        "setExposureTimeUs",
        Qt::QueuedConnection,
        Q_ARG(int, exposureTimeUs));
}

void CRuntimeCoordinator::setLaserEnabled(bool enabled)
{
    if (m_robotWorker == nullptr) {
        emit statusMessage(QStringLiteral("激光开关设置失败：RobotComm worker 尚未启动"));
        return;
    }
    QMetaObject::invokeMethod(
        m_robotWorker,
        "setLaserEnabled",
        Qt::QueuedConnection,
        Q_ARG(bool, enabled));
}

void CRuntimeCoordinator::sendCalibrationCommand(
    const robot_comm::RobotCalibrationCommand &command)
{
    if (m_robotWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_robotWorker,
        [worker = m_robotWorker, command]() {
            worker->sendCalibrationCommand(command);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::sendCorrectionCommand(
    const robot_comm::RobotCorrectionCommand &command)
{
    if (m_robotWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_robotWorker,
        [worker = m_robotWorker, command]() {
            worker->sendCorrectionCommand(command);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::startRobotJob(const std::string &jobName)
{
    if (m_robotWorker == nullptr) {
        return;
    }
    robot_comm::RobotJobStartCommand command;
    command.jobName = jobName;
    command.requestStart = true;
    command.valid = true;
    command.message = "request robot job start";
    QMetaObject::invokeMethod(
        m_robotWorker,
        [worker = m_robotWorker, command]() {
            worker->sendJobStartCommand(command);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::publishCameraFrame(const camera::CameraFrame &frame)
{
    if (m_cameraWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_cameraWorker,
        "publishFrame",
        Qt::QueuedConnection,
        Q_ARG(i2r_rlc::camera::CameraFrame, frame));
}

void CRuntimeCoordinator::publishRobotState(const robot_comm::RobotState &state)
{
    m_latestRobotStateReceivedMs = QDateTime::currentMSecsSinceEpoch();
    m_latestRobotState = state;
    m_hasLatestRobotState = true;
    updateGlobalLocalizationSegmentState(state);
    if (m_robotWorker == nullptr) {
        return;
    }
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_visionWorker,
            "updateRobotState",
            Qt::QueuedConnection,
            Q_ARG(i2r_rlc::robot_comm::RobotState, state));
    }
    if (m_onlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_onlineCorrectionWorker,
            "updateRobotState",
            Qt::QueuedConnection,
            Q_ARG(i2r_rlc::robot_comm::RobotState, state));
    }
    if (m_offlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_offlineCorrectionWorker,
            "updateRobotState",
            Qt::QueuedConnection,
            Q_ARG(i2r_rlc::robot_comm::RobotState, state));
    }
    if (m_teachTrajectoryWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_teachTrajectoryWorker,
            "updateRobotState",
            Qt::QueuedConnection,
            Q_ARG(i2r_rlc::robot_comm::RobotState, state));
    }
    QMetaObject::invokeMethod(
        m_robotWorker,
        "updateState",
        Qt::QueuedConnection,
        Q_ARG(i2r_rlc::robot_comm::RobotState, state));
}

void CRuntimeCoordinator::publishDigitalTwinVisionObservation(
    const digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    if (m_visionWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_visionWorker,
        "updateDigitalTwinVisionObservation",
        Qt::QueuedConnection,
        Q_ARG(i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation, observation));
}

void CRuntimeCoordinator::requestGlobalLocateVirtualPrior(
    quint64 requestId,
    const digital_twin::WeldmentVirtualLaserRequest &request)
{
    if (!m_running || m_stopping || m_globalLocatePriorWorker == nullptr) {
        return;
    }
    QMetaObject::invokeMethod(
        m_globalLocatePriorWorker,
        [worker = m_globalLocatePriorWorker, requestId, request]() {
            worker->generateVirtualPrior(requestId, request);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::setGlobalLocateVirtualLaserSource(
    GlobalLocateVirtualLaserSource source)
{
    m_globalLocateVirtualLaserSource = source;
}

void CRuntimeCoordinator::setGlobalLocateVirtualPointCloudSource(
    GlobalLocateVirtualPointCloudSource source)
{
    m_globalLocateVirtualPointCloudSource = source;
}

void CRuntimeCoordinator::connectRobotSimManual(
    const digital_twin::RobotSimEndpointConfig &endpoint,
    const digital_twin::RobotSimSceneConfig &scene)
{
    m_robotSimManualModeRequested = true;
    m_robotSimConnectionActiveOrPending = true;
    m_robotSimConnected = false;
    resetOnlineLocalRegistration();
    m_hasPendingRobotSimSyncState = false;
    m_robotSimSyncInFlightRequestId = 0;
    m_robotSimPriorInFlightRequestId = 0;
    m_robotSimHasSynchronizationStatus = false;
    m_robotSimLastSynchronizationMessage.clear();
    if (!m_running || m_stopping || m_robotSimWorker == nullptr) {
        m_robotSimConnectionActiveOrPending = false;
        emit robotSimManualConnectionStateChanged(
            false,
            QStringLiteral("RobotSim 后台线程尚未启动。"));
        return;
    }

    QMetaObject::invokeMethod(
        m_robotSimWorker,
        [worker = m_robotSimWorker, endpoint, scene]() {
            worker->connectRobotSimManual(endpoint, scene);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::connectRobotSim(
    const digital_twin::RobotSimEndpointConfig &endpoint,
    const digital_twin::RobotSimSceneConfig &scene)
{
    m_robotSimManualModeRequested = false;
    m_robotSimConnectionActiveOrPending = true;
    m_globalLocateRobotSimEndpoint = endpoint;
    m_globalLocateRobotSimScene = scene;
    m_robotSimConnected = false;
    resetOnlineLocalRegistration();
    m_hasPendingRobotSimSyncState = m_hasLatestRobotState;
    if (m_hasLatestRobotState) {
        m_pendingRobotSimSyncState = m_latestRobotState;
    }
    m_robotSimSyncInFlightRequestId = 0;
    if (!m_running || m_stopping || m_robotSimWorker == nullptr) {
        m_robotSimConnectionActiveOrPending = false;
        return;
    }
    if (!m_hasLatestRobotState) {
        m_robotSimConnectionActiveOrPending = false;
        m_hasPendingRobotSimSyncState = false;
        QMetaObject::invokeMethod(
            m_robotSimWorker,
            [worker = m_robotSimWorker]() {
                worker->resetRobotSim();
            },
            Qt::QueuedConnection);
        emit robotSimConnectionStateChanged(
            false,
            QStringLiteral(
                "尚未从控制柜获得有效机器人状态，不能同步 RobotSim 初始位姿。"));
        return;
    }

    const robot_comm::RobotState initialState = m_latestRobotState;
    QMetaObject::invokeMethod(
        m_robotSimWorker,
        [worker = m_robotSimWorker, endpoint, scene, initialState]() {
            worker->connectRobotSim(endpoint, scene, initialState);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::resetRobotSimConnection()
{
    m_robotSimConnectionActiveOrPending = false;
    m_robotSimConnected = false;
    resetOnlineLocalRegistration();
    m_robotSimManualModeRequested = false;
    m_hasPendingRobotSimSyncState = false;
    m_robotSimSyncInFlightRequestId = 0;
    m_robotSimPriorInFlightRequestId = 0;
    m_robotSimHasSynchronizationStatus = false;
    m_robotSimLastSynchronizationMessage.clear();
    if (!m_running || m_stopping || m_robotSimWorker == nullptr) {
        return;
    }

    QMetaObject::invokeMethod(
        m_robotSimWorker,
        "resetRobotSim",
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::startOnlineTracking()
{
    if (!m_running || m_stopping || m_onlineCorrectionWorker == nullptr) {
        return;
    }

    if (m_onlineOperationHoldLatched) {
        const QString message = QStringLiteral(
            "作业 HOLD 已锁存，请先人工确认并处理控制柜保持后重新启动；停弧未验证。");
        emit onlineCorrectionStartFailed(message);
        emit statusMessage(message);
        return;
    }

    beginCorrectionSession(CorrectionPurpose::OnlineCorrection);
    QMetaObject::invokeMethod(
        m_onlineCorrectionWorker,
        "startInitialDetection",
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::startOnlineCorrectionJob(
    const RuntimeOnlineCorrectionConfiguration &configuration,
    const std::string &jobName)
{
    if (m_onlineOperationHoldLatched) {
        const QString message = QStringLiteral(
            "作业 HOLD 已锁存，请先人工确认并处理控制柜保持后重新启动；停弧未验证。");
        emit onlineCorrectionStartFailed(message);
        emit statusMessage(message);
        return;
    }
    cancelOnlineCorrectionStart();
    m_pendingOnlineCorrectionConfiguration = configuration;
    if (!m_pendingOnlineCorrectionConfiguration.positionerCalibrationSnapshot.has_value()) {
        m_pendingOnlineCorrectionConfiguration.positionerCalibrationSnapshot =
            vision::CPositionerCalibrationParser::parseFile(configuration.rbcalibFile);
    }
    m_pendingOnlineCorrectionJobName = jobName;
    m_onlineCorrectionStartPhase =
        OnlineCorrectionStartPhase::WaitingForCameraFrame;

    if (!m_running || m_stopping
        || m_cameraWorker == nullptr
        || m_visionWorker == nullptr
        || m_robotWorker == nullptr
        || m_onlineCorrectionWorker == nullptr) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：Camera、Vision、RobotComm 或在线控制 worker 未就绪。"));
        return;
    }
    if (jobName.empty()) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：在线 JBI 名称为空。"));
        return;
    }
    if (m_correctionSession.active) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：已有在线或离线纠偏会话正在执行，未替换现有会话。"));
        return;
    }
    if (!m_hasLatestRobotState
        || m_latestRobotState.sStart != 0
        || m_latestRobotState.sMode != 2
        || m_latestRobotState.sServoPower != 1) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：机器人状态已在门禁通过后变化，要求 sStart=0、sMode=2、sServoPower=1。"));
        return;
    }
    if (m_latestRobotState.offline == 4) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：机器人端存在未完成的 O 示教点批次（offline=4），为避免 P001/P002 错位，未发送 JBI。"));
        return;
    }
    if (!m_robotSimConnected || m_robotSimManualModeRequested) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：RobotSim 真实场景未连接或仍处于手动仿真模式。"));
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const bool hasFreshFrame = m_hasLatestCameraFrame
        && nowMs - m_lastCameraFrameReceivedMs
            <= ONLINE_CAMERA_FRAME_MAXIMUM_AGE_MS;
    if (hasFreshFrame) {
        beginOnlineCorrectionArm();
        return;
    }

    const QString message = QStringLiteral(
        "在线启动 1/7：等待相机首个新鲜完整帧，尚未发送 JBI。可据此区分相机无帧与机器人未进入检测窗口。");
    emit onlineCorrectionStartProgress(message);
    emit statusMessage(message);
    armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase::WaitingForCameraFrame,
        ONLINE_CAMERA_PREFLIGHT_TIMEOUT_MS,
        QStringLiteral("在线纠偏启动失败：3 秒内未收到有效相机帧，未发送 JBI。"));
}

void CRuntimeCoordinator::cancelOnlineCorrectionStart()
{
    if (m_onlineCorrectionStartPhase == OnlineCorrectionStartPhase::Idle) {
        return;
    }

    const QString message = QStringLiteral("在线纠偏启动事务已取消。");
    const quint64 sessionId = m_onlineCorrectionStartSessionId;
    clearOnlineCorrectionStart();
    const bool ownsOnlineSession = sessionId != 0
        && m_correctionSession.active
        && m_correctionSession.sessionId == sessionId
        && m_correctionSession.purpose == CorrectionPurpose::OnlineCorrection;
    if (ownsOnlineSession) {
        invalidateCorrectionSession(message);
        if (m_visionWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_visionWorker, "stopTracking", Qt::QueuedConnection);
        }
        if (m_robotWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_robotWorker,
                [worker = m_robotWorker]() {
                    worker->setTrackingRequested(false);
                },
                Qt::QueuedConnection);
        }
        if (m_onlineCorrectionWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_onlineCorrectionWorker,
                [worker = m_onlineCorrectionWorker, message]() {
                    worker->failSynchronization(message);
                },
                Qt::QueuedConnection);
        }
    }
}

void CRuntimeCoordinator::beginOnlineCorrectionArm()
{
    if (m_onlineCorrectionStartPhase
        != OnlineCorrectionStartPhase::WaitingForCameraFrame) {
        return;
    }
    if (!m_hasLatestRobotState
        || m_latestRobotState.sStart != 0
        || m_latestRobotState.sMode != 2
        || m_latestRobotState.sServoPower != 1
        || !m_robotSimConnected
        || m_robotSimManualModeRequested) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：相机就绪时机器人或 RobotSim 条件已变化，未发送 JBI。"));
        return;
    }
    if (m_latestRobotState.offline == 4) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：机器人端 O 示教点批次在相机预检期间变为未完成状态（offline=4），未发送 JBI。"));
        return;
    }

    beginCorrectionSession(CorrectionPurpose::OnlineCorrection);
    m_onlineCorrectionStartSessionId = m_correctionSession.sessionId;
    m_onlineCorrectionStartPhase =
        OnlineCorrectionStartPhase::WaitingForConfiguration;
    m_trackingPurpose = RuntimeTrackingPurpose::OnlineCorrection;
    setCorrectionVisionTrackingActive(true);
    resetVisionFrameDispatch();

    const RuntimeOnlineCorrectionConfiguration configuration =
        m_pendingOnlineCorrectionConfiguration;
    m_onlinePositionerCalibration = configuration.positionerCalibrationSnapshot;
    const quint64 sessionId = m_onlineCorrectionStartSessionId;
    const QPointer<CRuntimeCoordinator> coordinator(this);
    QMetaObject::invokeMethod(
        m_onlineCorrectionWorker,
        [coordinator,
         worker = m_onlineCorrectionWorker,
         configuration,
         sessionId]() {
            const bool succeeded = worker->configureForStart(configuration);
            if (coordinator.isNull()) {
                return;
            }
            QMetaObject::invokeMethod(
                coordinator,
                [coordinator, sessionId, succeeded]() {
                    if (!coordinator.isNull()) {
                        coordinator->handleOnlineConfigurationReady(
                            sessionId, succeeded);
                    }
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);

    const QString message = QStringLiteral(
        "在线启动 2/7：相机和 RobotSim 预检通过，正在校验在线轨迹与 RBCALIB，尚未发送 JBI。");
    emit onlineCorrectionStartProgress(message);
    emit statusMessage(message);
    armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase::WaitingForConfiguration,
        ONLINE_WORKER_ARM_TIMEOUT_MS,
        QStringLiteral("在线纠偏启动失败：在线轨迹与 RBCALIB 在 3 秒内未完成校验，未发送 JBI。"));
}

void CRuntimeCoordinator::handleOnlineConfigurationReady(
    quint64 sessionId, bool succeeded)
{
    if (m_onlineCorrectionStartPhase
            != OnlineCorrectionStartPhase::WaitingForConfiguration
        || sessionId == 0
        || sessionId != m_onlineCorrectionStartSessionId
        || !m_correctionSession.active
        || m_correctionSession.sessionId != sessionId) {
        return;
    }
    if (!succeeded) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：在线轨迹或 RBCALIB 无效，未发送 JBI。"));
        return;
    }

    m_onlineCorrectionStartPhase = OnlineCorrectionStartPhase::WaitingForVision;
    const QString message = QStringLiteral(
        "在线启动 3/7：在线配置校验通过，正在等待 Vision worker 确认就绪，尚未发送 JBI。");
    emit onlineCorrectionStartProgress(message);
    emit statusMessage(message);
    armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase::WaitingForVision,
        ONLINE_WORKER_ARM_TIMEOUT_MS,
        QStringLiteral("在线纠偏启动失败：Vision worker 3 秒内未确认就绪，未发送 JBI。"));

    const QPointer<CRuntimeCoordinator> coordinator(this);
    QMetaObject::invokeMethod(
        m_visionWorker,
        [coordinator,
         worker = m_visionWorker,
         options = onlineTrackingOptions(),
         projectionContext = m_trackingProjectionContext,
         state = m_latestRobotState,
         hasState = m_hasLatestRobotState,
         sessionId]() {
            worker->configure(options, projectionContext);
            if (hasState) {
                worker->updateRobotState(state);
            }
            worker->startTracking();
            if (coordinator.isNull()) {
                return;
            }
            QMetaObject::invokeMethod(
                coordinator,
                [coordinator, sessionId]() {
                    if (!coordinator.isNull()) {
                        coordinator->handleOnlineVisionArmed(sessionId);
                    }
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::handleOnlineVisionArmed(quint64 sessionId)
{
    if (m_onlineCorrectionStartPhase
            != OnlineCorrectionStartPhase::WaitingForVision
        || sessionId == 0
        || sessionId != m_onlineCorrectionStartSessionId
        || !m_correctionSession.active
        || m_correctionSession.sessionId != sessionId) {
        return;
    }

    m_onlineCorrectionStartPhase =
        OnlineCorrectionStartPhase::WaitingForProtocol;
    const QString message = QStringLiteral(
        "在线启动 4/7：Vision 已就绪，正在用 N/D 确认 tracker=1、laser=1，尚未发送 JBI。");
    emit onlineCorrectionStartProgress(message);
    emit statusMessage(message);
    armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase::WaitingForProtocol,
        ONLINE_PROTOCOL_PREFLIGHT_TIMEOUT_MS,
        QStringLiteral("在线纠偏启动失败：tracker/laser 的 N/D 预检 5 秒内未完成，未发送 JBI。"));

    const QPointer<CRuntimeCoordinator> coordinator(this);
    QMetaObject::invokeMethod(
        m_robotWorker,
        [coordinator, worker = m_robotWorker, sessionId]() {
            const bool succeeded = worker->ensureOnlineCorrectionReady();
            if (coordinator.isNull()) {
                return;
            }
            QMetaObject::invokeMethod(
                coordinator,
                [coordinator, sessionId, succeeded]() {
                    if (!coordinator.isNull()) {
                        coordinator->handleOnlineProtocolReady(
                            sessionId, succeeded);
                    }
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::handleOnlineProtocolReady(
    quint64 sessionId, bool succeeded)
{
    if (m_onlineCorrectionStartPhase
            != OnlineCorrectionStartPhase::WaitingForProtocol
        || sessionId == 0
        || sessionId != m_onlineCorrectionStartSessionId
        || !m_correctionSession.active
        || m_correctionSession.sessionId != sessionId) {
        return;
    }
    if (!succeeded) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：携带 tracker=1、laser=1 的 N 未获得有效 D 回包，未发送 JBI。"));
        return;
    }
    if (!m_hasLatestRobotState
        || m_latestRobotState.sStart != 0
        || m_latestRobotState.sMode != 2
        || m_latestRobotState.sServoPower != 1) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：协议预检后机器人状态不再满足 sStart=0、PLAY、SERVO ON，未发送 JBI。"));
        return;
    }
    if (m_latestRobotState.offline == 4) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：协议预检发现机器人端仍有未完成的 O 示教点批次（offline=4），未发送 JBI。"));
        return;
    }

    m_onlineCorrectionStartBaselineJobLine =
        m_latestRobotState.usJobLine;
    m_onlineCorrectionStartPhase =
        OnlineCorrectionStartPhase::WaitingForJobReceipt;
    const QString message = QStringLiteral(
        "在线启动 5/7：tracker/laser 的 N/D 已确认，正在发送在线 JBI：%1")
                                .arg(QString::fromStdString(
                                    m_pendingOnlineCorrectionJobName));
    emit onlineCorrectionStartProgress(message);
    emit statusMessage(message);
    armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase::WaitingForJobReceipt,
        ONLINE_JOB_RECEIPT_TIMEOUT_MS,
        QStringLiteral("在线纠偏启动失败：J 命令 5 秒内未返回可解析 D 回包。"));
    startRobotJob(m_pendingOnlineCorrectionJobName);
}

void CRuntimeCoordinator::handleOnlineJobReceipt(
    const robot_comm::RobotCommandReceipt &receipt,
    const robot_comm::RobotState &state)
{
    if (m_onlineCorrectionStartPhase
            != OnlineCorrectionStartPhase::WaitingForJobReceipt
        || receipt.type != robot_comm::RobotCommandType::JobStart) {
        return;
    }

    const std::string expectedSuffix =
        std::string(",") + m_pendingOnlineCorrectionJobName;
    const bool matchesJob = receipt.request.size() >= expectedSuffix.size()
        && receipt.request.compare(
               receipt.request.size() - expectedSuffix.size(),
               expectedSuffix.size(),
               expectedSuffix) == 0;
    if (!matchesJob
        || !receipt.sent
        || !receipt.responseReceived
        || !receipt.completed
        || receipt.error != robot_comm::RobotCommandError::None) {
        const QString detail = receipt.message.empty()
            ? QStringLiteral("J 命令未完成")
            : QString::fromStdString(receipt.message);
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：J 命令事务未通过校验（job=%1，原因=%2）。")
                                      .arg(QString::fromStdString(
                                               m_pendingOnlineCorrectionJobName),
                                           detail));
        return;
    }

    m_onlineCorrectionStartPhase =
        OnlineCorrectionStartPhase::WaitingForJobMotion;
    const QString message = QStringLiteral(
        "在线启动 6/7：J/D 事务已确认，等待 sStart 从 0 变为 1 且 JBI 行号有效。");
    emit onlineCorrectionStartProgress(message);
    emit statusMessage(QStringLiteral(
        "%1 request=%2 response=%3")
                           .arg(message,
                                QString::fromStdString(receipt.request),
                                QString::fromStdString(receipt.response)));
    armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase::WaitingForJobMotion,
        ONLINE_JOB_MOTION_TIMEOUT_MS,
        QStringLiteral(
            "在线纠偏启动失败：J/D 已返回，但 15 秒内未观察到 sStart=1 和有效 JBI 行号。请检查控制柜是否接受该作业名及启动条件。"));
    processOnlineCorrectionStartState(state);
}

void CRuntimeCoordinator::processOnlineCorrectionStartState(
    const robot_comm::RobotState &state)
{
    const bool waitingForRobotExecution =
        m_onlineCorrectionStartPhase
            == OnlineCorrectionStartPhase::WaitingForJobMotion
        || m_onlineCorrectionStartPhase
            == OnlineCorrectionStartPhase::WaitingForDetectionWindow;
    if (waitingForRobotExecution
        && (!m_hasLatestCameraFrame
            || QDateTime::currentMSecsSinceEpoch()
                    - m_lastCameraFrameReceivedMs
                > ONLINE_CAMERA_PREFLIGHT_TIMEOUT_MS)) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：等待机器人进入检测窗口期间相机已连续 3 秒无有效帧。JBI 可能仍在运行，请在示教器确认并按需停止。"));
        return;
    }

    if (m_onlineCorrectionStartPhase
        == OnlineCorrectionStartPhase::WaitingForJobMotion) {
        if (state.sStart == 0) {
            return;
        }
        const bool jobLineProgressed = state.usJobLine > 0
            && (m_onlineCorrectionStartBaselineJobLine <= 0
                || state.usJobLine
                    != m_onlineCorrectionStartBaselineJobLine
                || state.traflag == 1);
        if (state.sStart == 1 && !jobLineProgressed) {
            return;
        }
        if (state.sStart != 1) {
            failOnlineCorrectionStart(QStringLiteral(
                "在线纠偏启动失败：控制柜返回了异常运行状态，sStart=%1，JBI行=%2，启动前行号=%3。")
                                          .arg(state.sStart)
                                          .arg(state.usJobLine)
                                          .arg(m_onlineCorrectionStartBaselineJobLine));
            return;
        }

        m_onlineCorrectionStartPhase =
            OnlineCorrectionStartPhase::WaitingForDetectionWindow;
        const QString message = QStringLiteral(
            "在线启动 7/7：已确认机器人运行（sStart=1，JBI行=%1），等待 STARTDETPRG/traflag=1。")
                                    .arg(state.usJobLine);
        emit onlineCorrectionStartProgress(message);
        emit statusMessage(message);
        armOnlineCorrectionStartTimeout(
            OnlineCorrectionStartPhase::WaitingForDetectionWindow,
            ONLINE_DETECTION_WINDOW_TIMEOUT_MS,
            QStringLiteral(
                "在线纠偏启动失败：机器人已运行，但 60 秒内未进入 STARTDETPRG（traflag=1）。请检查 JBI 检测窗口及其握手条件；JBI 可能仍在运行，请在示教器确认并按需停止。"));
    }

    if (m_onlineCorrectionStartPhase
        != OnlineCorrectionStartPhase::WaitingForDetectionWindow) {
        return;
    }
    if (state.sStart == 0) {
        failOnlineCorrectionStart(QStringLiteral(
            "在线纠偏启动失败：JBI 在进入 STARTDETPRG 前已经结束。请检查在线 JBI 流程和作业名。"));
        return;
    }
    if (state.traflag != 1) {
        return;
    }

    const quint64 sessionId = m_onlineCorrectionStartSessionId;
    const QString jobName = QString::fromStdString(
        m_pendingOnlineCorrectionJobName);
    clearOnlineCorrectionStart();
    emit statusMessage(QStringLiteral(
        "在线启动完成：job=%1，session=%2，sStart=%3，JBI行=%4，traflag=%5。")
                           .arg(jobName)
                           .arg(sessionId)
                           .arg(state.sStart)
                           .arg(state.usJobLine)
                           .arg(state.traflag));
    emit onlineCorrectionStartSucceeded();
    QMetaObject::invokeMethod(
        m_onlineCorrectionWorker,
        "startInitialDetection",
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::armOnlineCorrectionStartTimeout(
    OnlineCorrectionStartPhase phase,
    int timeoutMs,
    const QString &message)
{
    const quint64 generation = ++m_onlineCorrectionStartGeneration;
    QTimer::singleShot(
        timeoutMs,
        this,
        [this, generation, phase, message]() {
            if (generation != m_onlineCorrectionStartGeneration
                || phase != m_onlineCorrectionStartPhase) {
                return;
            }
            QString diagnostic = message;
            if (m_hasLatestRobotState) {
                diagnostic += QStringLiteral(
                    " 当前状态：sStart=%1，sMode=%2，sServoPower=%3，JBI行=%4，traflag=%5。")
                                  .arg(m_latestRobotState.sStart)
                                  .arg(m_latestRobotState.sMode)
                                  .arg(m_latestRobotState.sServoPower)
                                  .arg(m_latestRobotState.usJobLine)
                                  .arg(m_latestRobotState.traflag);
            }
            const qint64 frameAgeMs = m_hasLatestCameraFrame
                ? QDateTime::currentMSecsSinceEpoch()
                    - m_lastCameraFrameReceivedMs
                : -1;
            diagnostic += frameAgeMs >= 0
                ? QStringLiteral(" 最近相机帧距今 %1 ms。").arg(frameAgeMs)
                : QStringLiteral(" 本轮运行尚未收到有效相机帧。");
            failOnlineCorrectionStart(diagnostic);
        });
}

void CRuntimeCoordinator::failOnlineCorrectionStart(const QString &message)
{
    const quint64 sessionId = m_onlineCorrectionStartSessionId;
    clearOnlineCorrectionStart();
    const bool ownsOnlineSession = sessionId != 0
        && m_correctionSession.active
        && m_correctionSession.sessionId == sessionId
        && m_correctionSession.purpose == CorrectionPurpose::OnlineCorrection;
    if (ownsOnlineSession) {
        invalidateCorrectionSession(message);
        if (m_visionWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_visionWorker, "stopTracking", Qt::QueuedConnection);
        }
        if (m_robotWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_robotWorker,
                [worker = m_robotWorker]() {
                    worker->setTrackingRequested(false);
                },
                Qt::QueuedConnection);
        }
        if (m_onlineCorrectionWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_onlineCorrectionWorker,
                [worker = m_onlineCorrectionWorker, message]() {
                    worker->failSynchronization(message);
                },
                Qt::QueuedConnection);
        }
    }
    emit statusMessage(message);
    emit onlineCorrectionStartFailed(message);
}

void CRuntimeCoordinator::clearOnlineCorrectionStart()
{
    ++m_onlineCorrectionStartGeneration;
    m_onlineCorrectionStartPhase = OnlineCorrectionStartPhase::Idle;
    m_pendingOnlineCorrectionConfiguration = {};
    m_pendingOnlineCorrectionJobName.clear();
    m_onlineCorrectionStartSessionId = 0;
    m_onlineCorrectionStartBaselineJobLine = 0;
}

void CRuntimeCoordinator::startGlobalLocalizationTracking()
{
    GlobalLocalizationTrackingSources sources;
    sources.laserSource = m_globalLocateVirtualLaserSource;
    sources.pointCloudSource = m_globalLocateVirtualPointCloudSource;
    startGlobalLocalizationTracking(sources);
}

void CRuntimeCoordinator::startGlobalLocalizationTracking(
    const GlobalLocalizationTrackingSources& sources)
{
    if (!m_running || m_stopping) {
        return;
    }
    m_activeGlobalLocalizationSources = sources;
    m_trackingPurpose = RuntimeTrackingPurpose::GlobalLocalization;
    resetOnlineLocalRegistration();
    m_globalLocalizationDrainRequested = false;
    resetVisionFrameDispatch();
    m_globalLocalizationLastPosflag = 0;
    m_globalLocalizationCurrentSegmentIndex = 0;
    m_globalLocalizationSegmentSequence = 0;
    RuntimeTrackingOptions globalLocateOptions = m_trackingOptions;
    globalLocateOptions.globalLocateLineExtraction = true;
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_visionWorker,
            [worker = m_visionWorker,
             options = globalLocateOptions,
             projectionContext = m_trackingProjectionContext,
             state = m_latestRobotState,
             hasState = m_hasLatestRobotState]() {
                worker->configure(options, projectionContext);
                if (hasState) {
                    worker->updateRobotState(state);
                }
                worker->startTracking();
            },
            Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker]() {
                worker->setTrackingRequested(true);
            },
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::stopGlobalLocalizationTracking()
{
    if (m_trackingPurpose != RuntimeTrackingPurpose::GlobalLocalization) {
        return;
    }
    m_trackingPurpose = RuntimeTrackingPurpose::None;
    m_globalLocalizationDrainRequested = false;
    resetVisionFrameDispatch();
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(m_visionWorker, "stopTracking", Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker]() {
                worker->setTrackingRequested(false);
            },
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::stopOnlineTracking()
{
    cancelOnlineCorrectionStart();
    invalidateCorrectionSession(QStringLiteral("在线纠偏已停止。"));
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(m_visionWorker, "stopTracking", Qt::QueuedConnection);
    }
    if (m_onlineCorrectionWorker != nullptr && !m_onlineOperationHoldLatched) {
        // 普通检测停止可重置控制会话；已进入作业保持时，报告由 HOLD
        // 反馈或运行时退出结束，收起面板不能提前关闭它。
        QMetaObject::invokeMethod(
            m_onlineCorrectionWorker,
            "reset",
            Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker]() {
                worker->setTrackingRequested(false);
            },
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::requestOnlineOperationStop()
{
    m_onlineOperationHoldLatched = true;
    // 作业停止撤销所有检测关联，但记录器须保留到真实 HOLD 回执归档。
    clearOnlineCorrectionStart();
    invalidateCorrectionSession(QStringLiteral("已请求作业 HOLD，等待控制器反馈。"));
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(m_visionWorker, "stopTracking", Qt::QueuedConnection);
    }
    if (m_onlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(m_onlineCorrectionWorker, "stopForOperation", Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        // 只发布原子意图，不能等待机器人线程完成整批 O 后才收到停止。
        m_robotWorker->publishControlledHoldIntent();
        QMetaObject::invokeMethod(m_robotWorker, "requestControlledHold", Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::finishOnlineSegment()
{
    if (m_onlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_onlineCorrectionWorker, "finishOnlineSegment", Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::startOfflineCorrectionDetection()
{
    if (!m_running || m_stopping || m_offlineCorrectionWorker == nullptr) {
        return;
    }

    m_pendingOfflineCorrectionJobName.clear();
    beginCorrectionSession(CorrectionPurpose::OfflineCorrection);
    QMetaObject::invokeMethod(
        m_offlineCorrectionWorker,
        [worker = m_offlineCorrectionWorker,
         detType = m_trackingOptions.detType]() {
            worker->startDetection(detType);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::startOfflineCorrectionJob(const std::string &jobName)
{
    if (!m_pendingOfflineCorrectionJobName.empty()) {
        emit statusMessage(QStringLiteral("离线纠偏正在准备启动，请勿重复提交。"));
        return;
    }
    if (!m_running
        || m_stopping
        || m_offlineCorrectionWorker == nullptr
        || jobName.empty()) {
        emit statusMessage(QStringLiteral(
            "离线纠偏启动失败：运行时未就绪或 JBI 名称为空。"));
        return;
    }

    beginCorrectionSession(CorrectionPurpose::OfflineCorrection);
    m_pendingOfflineCorrectionJobName = jobName;
    QMetaObject::invokeMethod(
        m_offlineCorrectionWorker,
        [worker = m_offlineCorrectionWorker,
         detType = m_trackingOptions.detType]() {
            worker->startDetection(detType);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::finishOfflineCorrectionDetectionAndCompute()
{
    if (m_offlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_offlineCorrectionWorker,
            "finishDetectionAndCompute",
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::resetOfflineCorrection()
{
    m_pendingOfflineCorrectionJobName.clear();
    invalidateCorrectionSession(QStringLiteral("离线纠偏已重置。"));
    if (m_offlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_offlineCorrectionWorker,
            "reset",
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::configureTeachTrajectoryTempDirectory(
    const QString &tempDirectory)
{
    if (m_teachTrajectoryWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_teachTrajectoryWorker,
            "configureTempDirectory",
            Qt::QueuedConnection,
            Q_ARG(QString, tempDirectory));
    }
}

void CRuntimeCoordinator::startTeachTrajectoryRecording()
{
    if (m_teachTrajectoryWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_teachTrajectoryWorker,
            "startRecording",
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::saveTeachTrajectory(
    const QString &outputDirectory,
    const QString &baseName)
{
    if (m_teachTrajectoryWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_teachTrajectoryWorker,
            "saveCapturedTrajectory",
            Qt::QueuedConnection,
            Q_ARG(QString, outputDirectory),
            Q_ARG(QString, baseName));
    }
}

void CRuntimeCoordinator::resetTeachTrajectoryRecording()
{
    if (m_teachTrajectoryWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_teachTrajectoryWorker,
            "reset",
            Qt::QueuedConnection);
    }
}

void CRuntimeCoordinator::registerMetaTypes()
{
    qRegisterMetaType<camera::CameraFrame>("i2r_rlc::camera::CameraFrame");
    qRegisterMetaType<robot_comm::RobotState>("i2r_rlc::robot_comm::RobotState");
    qRegisterMetaType<robot_comm::RobotCorrectionCommand>(
        "i2r_rlc::robot_comm::RobotCorrectionCommand");
    qRegisterMetaType<robot_comm::RobotRealtimeCorrectionCommand>(
        "i2r_rlc::robot_comm::RobotRealtimeCorrectionCommand");
    qRegisterMetaType<robot_comm::RobotCalibrationCommand>(
        "i2r_rlc::robot_comm::RobotCalibrationCommand");
    qRegisterMetaType<robot_comm::RobotJobStartCommand>(
        "i2r_rlc::robot_comm::RobotJobStartCommand");
    qRegisterMetaType<robot_comm::RobotCommandReceipt>(
        "i2r_rlc::robot_comm::RobotCommandReceipt");
    qRegisterMetaType<robot_comm::RobotOperationStopState>(
        "i2r_rlc::robot_comm::RobotOperationStopState");
    qRegisterMetaType<robot_comm::RobotCommunicationDiagnostics>(
        "i2r_rlc::robot_comm::RobotCommunicationDiagnostics");
    qRegisterMetaType<control::OnlineCorrectionUpdate>(
        "i2r_rlc::control::OnlineCorrectionUpdate");
    qRegisterMetaType<control::OfflineCorrectionUpdate>(
        "i2r_rlc::control::OfflineCorrectionUpdate");
    qRegisterMetaType<control::TeachTrajectoryRecorderSnapshot>(
        "i2r_rlc::control::TeachTrajectoryRecorderSnapshot");
    qRegisterMetaType<control::OfflineCorrectionOptions>(
        "i2r_rlc::control::OfflineCorrectionOptions");
    qRegisterMetaType<control::OnlineCorrectionDetectionEvent>(
        "i2r_rlc::control::OnlineCorrectionDetectionEvent");
    qRegisterMetaType<control::OnlineCorrectionOptions>(
        "i2r_rlc::control::OnlineCorrectionOptions");
    qRegisterMetaType<control::OnlineCorrectionPlannedTrajectory>(
        "i2r_rlc::control::OnlineCorrectionPlannedTrajectory");
    qRegisterMetaType<vision::WeldProjectionContext>(
        "i2r_rlc::vision::WeldProjectionContext");
    qRegisterMetaType<digital_twin::DigitalTwinVisionLaserObservation>(
        "i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation");
    qRegisterMetaType<digital_twin::DigitalTwinVirtualReferenceResult>(
        "i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult");
    qRegisterMetaType<RuntimeTrackingOptions>("i2r_rlc::ui::RuntimeTrackingOptions");
    qRegisterMetaType<CorrectionPurpose>("i2r_rlc::ui::CorrectionPurpose");
    qRegisterMetaType<CorrectionRunSession>("i2r_rlc::ui::CorrectionRunSession");
    qRegisterMetaType<RuntimeVisionCorrelation>(
        "i2r_rlc::ui::RuntimeVisionCorrelation");
    qRegisterMetaType<CorrectionSyncState>(
        "i2r_rlc::ui::CorrectionSyncState");
    qRegisterMetaType<CorrectionSyncUpdate>(
        "i2r_rlc::ui::CorrectionSyncUpdate");
    qRegisterMetaType<RuntimeOnlineCorrectionConfiguration>(
        "i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration");
    qRegisterMetaType<RuntimeTrackingResult>("i2r_rlc::ui::RuntimeTrackingResult");
    qRegisterMetaType<StandaloneGlobalLocalizationRequest>(
        "i2r_rlc::ui::StandaloneGlobalLocalizationRequest");
    qRegisterMetaType<StandaloneGlobalLocalizationResult>(
        "i2r_rlc::ui::StandaloneGlobalLocalizationResult");
}

void CRuntimeCoordinator::connectWorkers()
{
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::operationStopRequested,
        this, [this](quint64 sessionId, const QString &reason) {
            if (!m_running || m_stopping || !m_correctionSession.active
                || sessionId != m_correctionSession.sessionId
                || m_correctionSession.purpose != CorrectionPurpose::OnlineCorrection) { return; }
            emit statusMessage(reason);
            requestOnlineOperationStop();
        }, Qt::QueuedConnection);
    connect(m_cameraWorker, &CCameraWorkerAdapter::frameReady,
            this, &CRuntimeCoordinator::handleCameraFrame,
            Qt::QueuedConnection);
    connect(m_cameraWorker, &CCameraWorkerAdapter::statusChanged,
            this, &CRuntimeCoordinator::statusMessage);

    connect(m_robotWorker, &CRobotCommWorkerAdapter::robotStateReady,
            this, &CRuntimeCoordinator::handleRobotState,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::robotStateReady,
            m_visionWorker, &CVisionTrackingWorkerAdapter::updateRobotState,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::robotStateReady,
            m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::updateRobotState,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::robotStateReady,
            m_offlineCorrectionWorker, &COfflineCorrectionWorkerAdapter::updateRobotState,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::robotStateReady,
            m_teachTrajectoryWorker, &CTeachTrajectoryRecorderWorkerAdapter::updateRobotState,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::statusChanged,
            this, &CRuntimeCoordinator::statusMessage);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::diagnosticsReady,
            this, &CRuntimeCoordinator::robotDiagnosticsReady);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::operationStopUpdated,
            this, &CRuntimeCoordinator::onlineOperationStopUpdated);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::operationStopUpdated,
            m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::recordOperationStop,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::commandReceiptReady,
            m_onlineCorrectionWorker,
            &COnlineCorrectionWorkerAdapter::handleRobotCommandReceipt,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::commandReceiptReady,
            m_offlineCorrectionWorker,
            &COfflineCorrectionWorkerAdapter::handleRobotCommandReceipt,
            Qt::QueuedConnection);
    connect(m_robotWorker, &CRobotCommWorkerAdapter::commandReceiptReady,
            this, &CRuntimeCoordinator::handleOnlineJobReceipt,
            Qt::QueuedConnection);

    connect(m_visionWorker, &CVisionTrackingWorkerAdapter::trackingResultReady,
            this, &CRuntimeCoordinator::routeTrackingResult);
    connect(m_visionWorker, &CVisionTrackingWorkerAdapter::statusChanged,
            this, &CRuntimeCoordinator::statusMessage);
    connect(
        m_globalLocatePriorWorker,
        &CGlobalLocatePriorWorkerAdapter::virtualPriorReady,
        this,
        &CRuntimeCoordinator::globalLocateVirtualPriorReady,
        Qt::QueuedConnection);
    connect(
        m_globalLocalizationWorker,
        &CGlobalLocalizationWorkerAdapter::resultReady,
        this,
        [this](
            quint64 requestId,
            const StandaloneGlobalLocalizationResult &localizationResult) {
            if (!localizationResult.succeeded) {
                emit standaloneGlobalLocalizationReady(
                    requestId,
                    localizationResult);
                return;
            }
            if (!m_running || m_stopping || m_robotSimWorker == nullptr) {
                auto failedResult = localizationResult;
                failedResult.succeeded = false;
                failedResult.message = QStringLiteral(
                    "全局定位完成，但 RobotSim 后台线程未运行，工件位姿未更新。");
                failedResult.warningMessages.push_back(failedResult.message);
                emit standaloneGlobalLocalizationReady(requestId, failedResult);
                return;
            }

            const auto resultSnapshot =
                std::make_shared<StandaloneGlobalLocalizationResult>(
                    localizationResult);
            const QPointer<CRuntimeCoordinator> coordinator(this);
            CGlobalLocatePriorWorkerAdapter *const robotSimWorker =
                m_robotSimWorker;
            QMetaObject::invokeMethod(
                robotSimWorker,
                [coordinator, robotSimWorker, requestId, resultSnapshot]() {
                    const auto commandResult =
                        robotSimWorker->applyGlobalLocalizationCorrection(
                            resultSnapshot->correctionTransform);
                    if (coordinator.isNull()) {
                        return;
                    }
                    QMetaObject::invokeMethod(
                        coordinator,
                        [coordinator, requestId, resultSnapshot, commandResult]() {
                            if (coordinator.isNull()) {
                                return;
                            }
                            auto completedResult = *resultSnapshot;
                            if (!commandResult.succeeded) {
                                completedResult.succeeded = false;
                                completedResult.message = QStringLiteral(
                                    "全局定位完成，但 RobotSim 工件位姿更新失败：%1")
                                    .arg(commandResult.message);
                                completedResult.warningMessages.push_back(
                                    completedResult.message);
                            } else {
                                completedResult.message += QStringLiteral(
                                    " %1").arg(commandResult.message);
                                completedResult.infoMessages.push_back(
                                    commandResult.message);
                            }
                            emit coordinator->standaloneGlobalLocalizationReady(
                                requestId,
                                completedResult);
                        },
                        Qt::QueuedConnection);
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimVirtualPriorReady,
        this,
        &CRuntimeCoordinator::handleRobotSimVirtualPriorReady,
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimRealtimeVirtualPriorReady,
        this,
        &CRuntimeCoordinator::handleRobotSimRealtimeVirtualPriorReady,
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimRealtimeVirtualReferenceReady,
        this,
        &CRuntimeCoordinator::handleRobotSimRealtimeVirtualReferenceReady,
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimConnectionStateChanged,
        this,
        &CRuntimeCoordinator::handleRobotSimConnectionStateChanged,
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimManualConnectionStateChanged,
        this,
        &CRuntimeCoordinator::robotSimManualConnectionStateChanged,
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimManualVirtualReferenceReady,
        this,
        &CRuntimeCoordinator::robotSimManualVirtualReferenceReady,
        Qt::QueuedConnection);
    connect(
        m_robotSimWorker,
        &CGlobalLocatePriorWorkerAdapter::robotSimStateSynchronized,
        this,
        &CRuntimeCoordinator::handleRobotSimStateSynchronized,
        Qt::QueuedConnection);

    connect(
        m_onlineCorrectionWorker,
        &COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
        this,
        [this](const control::OnlineCorrectionUpdate &update) {
            const quint64 targetSessionId = update.state.controlTarget.sessionId;
            if (targetSessionId != 0 && (!m_correctionSession.active
                    || targetSessionId != m_correctionSession.sessionId)) {
                return;
            }
            if (update.requestOperationStop) {
                // 严重失测必须在 Error 的普通会话失效路径前锁存 HOLD。
                // 拒绝无身份或旧会话请求，防止迟到反馈停止新的作业。
                if (m_running && !m_stopping && m_correctionSession.active
                    && targetSessionId == m_correctionSession.sessionId
                    && targetSessionId != 0
                    && m_correctionSession.purpose == CorrectionPurpose::OnlineCorrection) {
                    emit onlineCorrectionUpdated(update);
                    emit statusMessage(QString::fromStdString(update.message));
                    requestOnlineOperationStop();
                }
                return;
            }
            if (m_correctionSession.active
                && m_correctionSession.purpose
                    == CorrectionPurpose::OnlineCorrection) {
                // STARTCOR 是在线 SMOVC 窗口的权威门禁。当前实机 JBI
                // 会在 STARTCOR 前执行 ENDDETPRG，因此在线阶段 traflag=0
                // 不能被继续解释为“禁止跟踪”。
                const bool wasOnline = m_correctionSession.onlineControlWindowActive;
                m_correctionSession.onlineControlWindowActive =
                    update.phase
                    == control::OnlineCorrectionPhase::OnlineRunning;
                if (wasOnline != m_correctionSession.onlineControlWindowActive) {
                    resetOnlineLocalRegistration();
                }
            }
            emit onlineCorrectionUpdated(update);
            if (update.phase == control::OnlineCorrectionPhase::Completed
                || update.phase == control::OnlineCorrectionPhase::Error) {
                // Vision 的阶段性暂停不能清除机器人侧 tracker 许可。
                // 只有整次在线纠偏终止后，才退出机器人 STARTCOR 门禁。
                if (m_robotWorker != nullptr) {
                    QMetaObject::invokeMethod(
                        m_robotWorker,
                        [worker = m_robotWorker]() {
                            worker->setTrackingRequested(false);
                        },
                        Qt::QueuedConnection);
                }
                invalidateCorrectionSession(
                    QString::fromStdString(update.message));
            }
        });
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::robotCommandReady,
            m_robotWorker, &CRobotCommWorkerAdapter::sendCorrectionCommand,
            Qt::QueuedConnection);
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::realtimeCommandReady,
            m_robotWorker, &CRobotCommWorkerAdapter::sendRealtimeCorrectionCommand,
            Qt::QueuedConnection);
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::startTrackingRequested,
            this, [this]() {
                if (!m_running || m_stopping) {
                    return;
                }
                if (!m_correctionSession.active
                    || m_correctionSession.purpose
                        != CorrectionPurpose::OnlineCorrection) {
                    return;
                }
                m_trackingPurpose = RuntimeTrackingPurpose::OnlineCorrection;
                setCorrectionVisionTrackingActive(true);
                resetOnlineLocalRegistration();
                resetVisionFrameDispatch();
                emitCorrectionSyncState(
                    CorrectionSyncState::Idle,
                    -1,
                    m_correctionSession.onlineControlWindowActive
                        ? QStringLiteral(
                              "STARTCOR 已确认，等待首个在线 KCF 完整帧。")
                        : QStringLiteral(
                              "初始检测窗口已启动，等待首个完整帧。"));
                if (m_visionWorker != nullptr) {
                    RuntimeTrackingOptions options = onlineTrackingOptions();
                    options.requireDetectionWindow =
                        !m_correctionSession.onlineControlWindowActive;
                    QMetaObject::invokeMethod(
                        m_visionWorker,
                        [worker = m_visionWorker,
                         options,
                         projectionContext = m_trackingProjectionContext,
                         state = m_latestRobotState,
                         hasState = m_hasLatestRobotState]() {
                            worker->configure(options, projectionContext);
                            if (hasState) {
                                worker->updateRobotState(state);
                            }
                            worker->startTracking();
                        },
                        Qt::QueuedConnection);
                }
            });
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::startTrackingRequested,
            m_robotWorker, [worker = m_robotWorker]() {
                worker->setTrackingRequested(true);
            },
            Qt::QueuedConnection);
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::stopTrackingRequested,
            this, [this]() {
                if (!m_running || m_stopping) {
                    return;
                }
                if (m_trackingPurpose == RuntimeTrackingPurpose::OnlineCorrection) {
                    m_trackingPurpose = RuntimeTrackingPurpose::None;
                }
                // 在线控制器会在反向检测、示教点下发等阶段临时暂停 Vision。
                // 这些阶段仍属于同一次正式纠偏，不得更换或失效 sessionId。
                setCorrectionVisionTrackingActive(false);
                resetOnlineLocalRegistration();
                resetVisionFrameDispatch();
                if (m_correctionSession.active
                    && m_correctionSession.purpose
                        == CorrectionPurpose::OnlineCorrection) {
                    emitCorrectionSyncState(
                        CorrectionSyncState::TrackingPaused,
                        -1,
                        QStringLiteral(
                            "在线 Vision tracking 已暂停，已恢复实时相机画面。"));
                }
                if (m_visionWorker != nullptr) {
                    QMetaObject::invokeMethod(
                        m_visionWorker, "stopTracking", Qt::QueuedConnection);
                }
            });
    connect(m_onlineCorrectionWorker, &COnlineCorrectionWorkerAdapter::statusChanged,
            this, &CRuntimeCoordinator::statusMessage);

    connect(
        m_offlineCorrectionWorker,
        &COfflineCorrectionWorkerAdapter::offlineCorrectionUpdated,
        this,
        [this](const control::OfflineCorrectionUpdate &update) {
            emit offlineCorrectionUpdated(update);
            if (update.phase == control::OfflineCorrectionPhase::Completed
                || update.phase == control::OfflineCorrectionPhase::Error) {
                invalidateCorrectionSession(
                    QString::fromStdString(update.message));
            }
        });
    connect(m_offlineCorrectionWorker, &COfflineCorrectionWorkerAdapter::robotCommandReady,
            m_robotWorker, &CRobotCommWorkerAdapter::sendCorrectionCommand,
            Qt::QueuedConnection);
    connect(m_offlineCorrectionWorker, &COfflineCorrectionWorkerAdapter::offlineStartCountChanged,
            m_robotWorker, &CRobotCommWorkerAdapter::setOfflineStartCount,
            Qt::QueuedConnection);
    connect(m_offlineCorrectionWorker,
            &COfflineCorrectionWorkerAdapter::offlineStartCountChanged,
            this,
            &CRuntimeCoordinator::offlineCorrectionHandshakeChanged,
            Qt::QueuedConnection);
    connect(m_offlineCorrectionWorker, &COfflineCorrectionWorkerAdapter::startTrackingRequested,
            this, [this]() {
                if (!m_running || m_stopping) {
                    return;
                }
                if (!m_correctionSession.active
                    || m_correctionSession.purpose
                        != CorrectionPurpose::OfflineCorrection) {
                    return;
                }
                m_trackingPurpose = RuntimeTrackingPurpose::OfflineCorrection;
                setCorrectionVisionTrackingActive(true);
                resetVisionFrameDispatch();
                if (m_visionWorker != nullptr) {
                    const quint64 sessionId = m_correctionSession.sessionId;
                    QMetaObject::invokeMethod(
                        m_visionWorker,
                        [worker = m_visionWorker,
                         coordinator = this,
                         sessionId,
                         options = m_trackingOptions,
                         projectionContext = m_trackingProjectionContext,
                         state = m_latestRobotState,
                         hasState = m_hasLatestRobotState]() {
                            worker->configure(options, projectionContext);
                            if (hasState) {
                                worker->updateRobotState(state);
                            }
                            worker->startTracking();
                            QMetaObject::invokeMethod(
                                coordinator,
                                [coordinator, sessionId]() {
                                    coordinator->handleOfflineVisionArmed(sessionId);
                                },
                                Qt::QueuedConnection);
                        },
                        Qt::QueuedConnection);
                } else {
                    m_pendingOfflineCorrectionJobName.clear();
                    emit statusMessage(QStringLiteral(
                        "离线纠偏启动失败：Vision worker 不可用"));
                }
                if (m_robotWorker != nullptr) {
                    QMetaObject::invokeMethod(
                        m_robotWorker,
                        [worker = m_robotWorker]() {
                            worker->setTrackingRequested(true);
                        },
                        Qt::QueuedConnection);
                }
            });
    connect(m_offlineCorrectionWorker, &COfflineCorrectionWorkerAdapter::stopTrackingRequested,
            this, [this]() {
                if (!m_running || m_stopping) {
                    return;
                }
                if (m_trackingPurpose == RuntimeTrackingPurpose::OfflineCorrection) {
                    m_trackingPurpose = RuntimeTrackingPurpose::None;
                }
                // 检测结束后的 ICP 与命令确认仍属于当前离线会话；这里只暂停
                // Vision 入帧，最终 Completed/Error 或用户 reset 才清理 session。
                setCorrectionVisionTrackingActive(false);
                resetVisionFrameDispatch();
                if (m_visionWorker != nullptr) {
                    QMetaObject::invokeMethod(
                        m_visionWorker, "stopTracking", Qt::QueuedConnection);
                }
                if (m_robotWorker != nullptr) {
                    QMetaObject::invokeMethod(
                        m_robotWorker,
                        [worker = m_robotWorker]() {
                            worker->setTrackingRequested(false);
                        },
                        Qt::QueuedConnection);
                }
            });
    connect(m_offlineCorrectionWorker, &COfflineCorrectionWorkerAdapter::statusChanged,
            this, &CRuntimeCoordinator::statusMessage);

    connect(m_teachTrajectoryWorker,
            &CTeachTrajectoryRecorderWorkerAdapter::teachTrajectoryRecorderUpdated,
            this,
            &CRuntimeCoordinator::teachTrajectoryRecorderUpdated);
    connect(m_teachTrajectoryWorker, &CTeachTrajectoryRecorderWorkerAdapter::statusChanged,
            this, &CRuntimeCoordinator::statusMessage);
}

void CRuntimeCoordinator::handleCameraFrame(const camera::CameraFrame &frame)
{
    if (!frame.image.isNull() || !frame.previewImage.isNull()) {
        m_hasLatestCameraFrame = true;
        m_lastCameraFrameReceivedMs = QDateTime::currentMSecsSinceEpoch();
        if (m_onlineCorrectionStartPhase
            == OnlineCorrectionStartPhase::WaitingForCameraFrame) {
            beginOnlineCorrectionArm();
        }
    }
    emit frameReady(frame);
    enqueueVisionFrame(frame);
}

void CRuntimeCoordinator::finishGlobalLocalizationTracking()
{
    if (m_trackingPurpose != RuntimeTrackingPurpose::GlobalLocalization) {
        emit globalLocalizationTrackingDrained();
        return;
    }
    m_globalLocalizationDrainRequested = true;
    if (!m_visionFrameInFlight && m_pendingVisionFrame.has_value()) {
        dispatchPendingVisionFrame();
    }
    completeGlobalLocalizationTrackingDrain();
}

void CRuntimeCoordinator::applyStandaloneGlobalLocalization(
    quint64 requestId,
    StandaloneGlobalLocalizationRequest request)
{
    if (!m_running || m_stopping || m_globalLocalizationWorker == nullptr) {
        StandaloneGlobalLocalizationResult result;
        result.message = QStringLiteral("全局定位工作线程未运行。");
        emit standaloneGlobalLocalizationReady(requestId, result);
        return;
    }

    const auto requestSnapshot =
        std::make_shared<StandaloneGlobalLocalizationRequest>(
            std::move(request));
    QMetaObject::invokeMethod(
        m_globalLocalizationWorker,
        [worker = m_globalLocalizationWorker, requestId, requestSnapshot]() {
            worker->apply(requestId, *requestSnapshot);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::handleRobotState(const robot_comm::RobotState &state)
{
    const bool localMotionGateChanged = !m_hasLatestRobotState
        || state.sStart != m_latestRobotState.sStart
        || state.sHold != m_latestRobotState.sHold
        || state.sMode != m_latestRobotState.sMode;
    m_latestRobotStateReceivedMs = QDateTime::currentMSecsSinceEpoch();
    m_latestRobotState = state;
    m_hasLatestRobotState = true;
    if ((localMotionGateChanged || !m_localRegistrationRobotStateFresh)
        && m_correctionSession.active) {
        resetOnlineLocalRegistration();
    }
    m_localRegistrationRobotStateFresh = true;
    processOnlineCorrectionStartState(state);
    updateOfflineSeamWindow(state);
    if (m_robotSimConnected && canRunRealtimeRobotSimSynchronization()) {
        m_pendingRobotSimSyncState = state;
        m_hasPendingRobotSimSyncState = true;
        dispatchPendingRobotSimStateSynchronization();
    }
    updateGlobalLocalizationSegmentState(state);
    emit robotStateReady(state);
}

void CRuntimeCoordinator::updateOfflineSeamWindow(
    const robot_comm::RobotState &state)
{
    if (!m_correctionSession.active
        || m_correctionSession.purpose != CorrectionPurpose::OfflineCorrection
        || !m_correctionSession.visionTrackingActive) {
        return;
    }
    if (state.sStart == 1 && state.traflag == 1) {
        m_offlineSeamWindowEntered = true;
        return;
    }
    if (!m_offlineSeamWindowEntered || state.traflag != 0) {
        return;
    }

    // ENDDET 只结束采点；保留离线会话和 B040/JBI 完成握手。
    // 先释放关联键，再通知 UI，迟到的 RobotSim/KCF 结果不能恢复旧叠加。
    setCorrectionVisionTrackingActive(false);
    m_trackingPurpose = RuntimeTrackingPurpose::None;
    resetVisionFrameDispatch();
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_visionWorker, "stopTracking", Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker]() {
                worker->setTrackingRequested(false);
            },
            Qt::QueuedConnection);
    }
    const QString message = QStringLiteral(
        "离线焊缝扫描结束：已停止 KCF，恢复实时图像；等待 JBI 完成后执行 ICP。");
    emitCorrectionSyncState(
        CorrectionSyncState::OfflineSeamWindowClosed, -1, message);
    emit statusMessage(message);
}

bool CRuntimeCoordinator::canRunRealtimeRobotSimSynchronization() const
{
    // 纠偏会话期间由逐帧同步独占 RobotSim，结束会话后恢复实时同步。
    return !m_correctionSession.active;
}

void CRuntimeCoordinator::dispatchPendingRobotSimStateSynchronization()
{
    if (!m_robotSimConnected
        || !m_hasPendingRobotSimSyncState
        || !canRunRealtimeRobotSimSynchronization()
        || m_robotSimSyncInFlightRequestId != 0
        || m_robotSimPriorInFlightRequestId != 0
        || m_robotSimWorker == nullptr) {
        return;
    }

    const robot_comm::RobotState state = m_pendingRobotSimSyncState;
    m_hasPendingRobotSimSyncState = false;
    m_robotSimSyncInFlightRequestId = ++m_robotSimSyncNextRequestId;
    QMetaObject::invokeMethod(
        m_robotSimWorker,
        [worker = m_robotSimWorker,
         requestId = m_robotSimSyncInFlightRequestId,
         state]() {
            worker->synchronizeRobotSimState(requestId, state);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::handleRobotSimConnectionStateChanged(
    bool connected,
    const QString &message)
{
    if (!m_robotSimConnectionActiveOrPending
        || m_robotSimManualModeRequested) {
        m_robotSimConnected = false;
        m_robotSimSyncInFlightRequestId = 0;
        m_hasPendingRobotSimSyncState = false;
        return;
    }
    m_robotSimConnected = connected;
    resetOnlineLocalRegistration();
    m_robotSimConnectionActiveOrPending = connected;
    m_robotSimHasSynchronizationStatus = false;
    m_robotSimLastSynchronizationMessage.clear();
    if (!connected) {
        m_robotSimSyncInFlightRequestId = 0;
        m_hasPendingRobotSimSyncState = false;
    } else if (m_hasLatestRobotState
               && canRunRealtimeRobotSimSynchronization()) {
        m_pendingRobotSimSyncState = m_latestRobotState;
        m_hasPendingRobotSimSyncState = true;
        dispatchPendingRobotSimStateSynchronization();
    }
    emit robotSimConnectionStateChanged(connected, message);
}

void CRuntimeCoordinator::handleRobotSimStateSynchronized(
    quint64 requestId,
    bool succeeded,
    const QString &message)
{
    if (!m_robotSimConnectionActiveOrPending
        || m_robotSimManualModeRequested) {
        m_robotSimSyncInFlightRequestId = 0;
        return;
    }
    if (requestId == 0 || requestId != m_robotSimSyncInFlightRequestId) {
        return;
    }
    m_robotSimSyncInFlightRequestId = 0;
    if (!m_robotSimHasSynchronizationStatus
        || succeeded != m_robotSimLastSynchronizationSucceeded
        || message != m_robotSimLastSynchronizationMessage) {
        m_robotSimHasSynchronizationStatus = true;
        m_robotSimLastSynchronizationSucceeded = succeeded;
        m_robotSimLastSynchronizationMessage = message;
        emit robotSimStateSynchronizationChanged(succeeded, message);
    }
    if (canRunRealtimeRobotSimSynchronization()) {
        dispatchPendingRobotSimStateSynchronization();
    }
}

void CRuntimeCoordinator::handleRobotSimRealtimeVirtualPriorReady(
    quint64 requestId,
    const digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    if (!m_robotSimConnectionActiveOrPending
        || m_robotSimManualModeRequested
        || !canRunRealtimeRobotSimSynchronization()) {
        return;
    }
    if (requestId != 0 && requestId != m_robotSimSyncInFlightRequestId) {
        return;
    }
    emit robotSimRealtimeVirtualPriorReady(observation);
}

void CRuntimeCoordinator::handleRobotSimRealtimeVirtualReferenceReady(
    quint64 requestId,
    const digital_twin::DigitalTwinVirtualReferenceResult &reference)
{
    if (!m_robotSimConnectionActiveOrPending
        || m_robotSimManualModeRequested
        || !canRunRealtimeRobotSimSynchronization()) {
        return;
    }
    if (requestId != 0 && requestId != m_robotSimSyncInFlightRequestId) {
        return;
    }
    emit robotSimRealtimeVirtualReferenceReady(reference);
}

void CRuntimeCoordinator::enqueueVisionFrame(const camera::CameraFrame &frame)
{
    if (m_trackingPurpose == RuntimeTrackingPurpose::None
        || (m_trackingPurpose == RuntimeTrackingPurpose::GlobalLocalization
            && m_globalLocalizationDrainRequested)
        || m_visionWorker == nullptr) {
        return;
    }

    FrozenVisionFrameContext context;
    context.frame = frame;
    context.robotState = m_latestRobotState;
    context.robotStateReceivedMs = m_latestRobotStateReceivedMs;
    context.hasRobotState = m_hasLatestRobotState;
    context.purpose = m_trackingPurpose;
    context.localizationSegmentIndex =
        m_trackingPurpose == RuntimeTrackingPurpose::GlobalLocalization
        ? m_globalLocalizationCurrentSegmentIndex
        : -1;
    if (m_trackingPurpose == RuntimeTrackingPurpose::OnlineCorrection
        || m_trackingPurpose == RuntimeTrackingPurpose::OfflineCorrection) {
        const CorrectionPurpose expectedPurpose =
            m_trackingPurpose == RuntimeTrackingPurpose::OnlineCorrection
            ? CorrectionPurpose::OnlineCorrection
            : CorrectionPurpose::OfflineCorrection;
        if (!m_correctionSession.active
            || !m_correctionSession.visionTrackingActive
            || m_correctionSession.purpose != expectedPurpose) {
            return;
        }
        context.correctionSessionId = m_correctionSession.sessionId;
        context.onlineControlWindowActive = m_correctionSession.onlineControlWindowActive;
    }

    // current 正在处理时，后到帧只能整体覆盖唯一 latest-pending 槽。
    // 这里按值冻结完整上下文，禁止新图像与旧 RobotState/session 混配。
    m_pendingVisionFrame = std::move(context);
    dispatchPendingVisionFrame();
}

void CRuntimeCoordinator::dispatchPendingVisionFrame()
{
    if (dispatchOnlineLocalRegistrationWriteback()) {
        return;
    }
    if (m_visionFrameInFlight
        || !m_pendingVisionFrame.has_value()
        || m_trackingPurpose == RuntimeTrackingPurpose::None
        || m_visionWorker == nullptr) {
        return;
    }

    FrozenVisionFrameContext context = std::move(*m_pendingVisionFrame);
    m_pendingVisionFrame.reset();
    if (context.purpose != m_trackingPurpose) {
        return;
    }
    if (context.purpose == RuntimeTrackingPurpose::OnlineCorrection
        || context.purpose == RuntimeTrackingPurpose::OfflineCorrection) {
        const CorrectionPurpose expectedPurpose =
            context.purpose == RuntimeTrackingPurpose::OnlineCorrection
            ? CorrectionPurpose::OnlineCorrection
            : CorrectionPurpose::OfflineCorrection;
        if (!m_correctionSession.active
            || !m_correctionSession.visionTrackingActive
            || m_correctionSession.sessionId != context.correctionSessionId
            || m_correctionSession.purpose != expectedPurpose) {
            return;
        }
    }

    m_currentVisionFrame = std::move(context);
    m_visionFrameInFlight = true;
    FrozenVisionFrameContext &current = *m_currentVisionFrame;
    current.modelGeneration = m_localRegistrationGeneration;
    current.workpiecePoseRevision = m_localWorkpiecePoseRevision;
    const bool isCorrection =
        current.purpose == RuntimeTrackingPurpose::OnlineCorrection
        || current.purpose == RuntimeTrackingPurpose::OfflineCorrection;
    const bool needsGlobalLocalizationRobotSimPrior =
        current.purpose == RuntimeTrackingPurpose::GlobalLocalization
        && (m_activeGlobalLocalizationSources.laserSource
                == GlobalLocateVirtualLaserSource::RobotSim
            || m_activeGlobalLocalizationSources.pointCloudSource
                == GlobalLocateVirtualPointCloudSource::RobotSimLaserScan)
        && current.robotState.traflag == 1
        && current.robotState.posflag != 0
        && current.localizationSegmentIndex > 0;
    const bool needsRobotSimPrior =
        isCorrection || needsGlobalLocalizationRobotSimPrior;
    if (isCorrection && (!current.hasRobotState || !m_robotSimConnected)) {
        failCorrectionFrameSynchronization(
            current.frame.frameId,
            !current.hasRobotState
                ? QStringLiteral("当前相机帧没有冻结的 RobotState，未发起 RobotSim 查询。")
                : QStringLiteral("RobotSim 真实场景未连接，未发起纠偏逐帧查询。"));
        return;
    }
    const bool requiresDetectionWindow =
        current.purpose != RuntimeTrackingPurpose::OnlineCorrection
        || !m_correctionSession.onlineControlWindowActive;
    if (isCorrection
        && (current.robotState.sStart != 1
            || (requiresDetectionWindow
                && current.robotState.traflag != 1))) {
        // 初始扫描和离线纠偏只在 STARTDETPRG 窗口采帧。
        // 在线 SMOVC 由已锁存的 STARTCOR 状态授权，兼容该窗口
        // 期间 traflag 保持 0 的控制柜 JBI。
        m_currentVisionFrame.reset();
        m_visionFrameInFlight = false;
        dispatchPendingVisionFrame();
        return;
    }
    if (needsRobotSimPrior && m_robotSimWorker != nullptr) {
        current.requestId = ++m_robotSimPriorNextRequestId;
        m_robotSimPriorInFlightRequestId = current.requestId;
        if (isCorrection) {
            emitCorrectionSyncState(
                CorrectionSyncState::Querying,
                current.frame.frameId,
                QStringLiteral("正在按冻结 RobotState 查询 RobotSim 同帧 observation。"));
        }
        QMetaObject::invokeMethod(
            m_robotSimWorker,
            [worker = m_robotSimWorker,
             requestId = current.requestId,
             frameId = current.frame.frameId,
             state = current.robotState,
             localizationSegmentIndex = current.localizationSegmentIndex]() {
                worker->generateRobotSimVirtualPrior(
                    requestId,
                    frameId,
                    state,
                    localizationSegmentIndex);
            },
            Qt::QueuedConnection);
        return;
    }
    if (isCorrection) {
        failCorrectionFrameSynchronization(
            current.frame.frameId,
            QStringLiteral("RobotSim 工作线程不可用，未发起纠偏逐帧查询。"));
        return;
    }

    const RuntimeVisionCorrelation correlation = makeVisionCorrelation(current);
    QMetaObject::invokeMethod(
        m_visionWorker,
        [worker = m_visionWorker,
         frame = current.frame,
         state = current.robotState,
         localizationSegmentIndex = current.localizationSegmentIndex,
         correlation]() {
            worker->submitFrameWithRobotStateAndCorrelation(
                frame, state, localizationSegmentIndex, correlation);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::resetVisionFrameDispatch()
{
    m_pendingVisionFrame.reset();
    m_currentVisionFrame.reset();
    m_visionFrameInFlight = false;
    m_globalLocalizationDrainRequested = false;
    m_robotSimPriorInFlightRequestId = 0;
}

void CRuntimeCoordinator::handleOfflineVisionArmed(quint64 sessionId)
{
    if (sessionId == 0 || sessionId != m_correctionSession.sessionId) {
        // 旧 Vision 线程任务可能晚于用户重启返回；它不得消费新会话的 JBI。
        return;
    }
    if (m_pendingOfflineCorrectionJobName.empty()) {
        return;
    }
    if (!m_running
        || m_stopping
        || !m_correctionSession.active
        || !m_correctionSession.visionTrackingActive
        || m_correctionSession.purpose != CorrectionPurpose::OfflineCorrection
        || m_trackingPurpose != RuntimeTrackingPurpose::OfflineCorrection) {
        m_pendingOfflineCorrectionJobName.clear();
        return;
    }

    emit statusMessage(QStringLiteral(
        "离线纠偏 Vision 已就绪，正在确认激光 ON 指令。"));
    if (m_robotWorker == nullptr) {
        handleOfflineLaserReady(sessionId, false);
        return;
    }
    QMetaObject::invokeMethod(
        m_robotWorker,
        [this, worker = m_robotWorker, sessionId]() {
            const bool succeeded = worker->ensureOfflineLaserEnabled();
            QMetaObject::invokeMethod(
                this,
                [this, sessionId, succeeded]() {
                    handleOfflineLaserReady(sessionId, succeeded);
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::handleOfflineLaserReady(
    quint64 sessionId, bool succeeded)
{
    if (!m_running || m_stopping
        || sessionId == 0
        || sessionId != m_correctionSession.sessionId
        || !m_correctionSession.active
        || m_correctionSession.purpose != CorrectionPurpose::OfflineCorrection
        || !m_correctionSession.visionTrackingActive
        || m_pendingOfflineCorrectionJobName.empty()) {
        return;
    }
    if (!succeeded) {
        const QString message = QStringLiteral(
            "离线纠偏启动失败：激光 ON 指令通信确认失败，未启动 JBI。");
        invalidateCorrectionSession(message);
        if (m_offlineCorrectionWorker != nullptr) {
            QMetaObject::invokeMethod(
                m_offlineCorrectionWorker,
                [worker = m_offlineCorrectionWorker, message]() {
                    worker->failSynchronization(message);
                },
                Qt::QueuedConnection);
        }
        emit statusMessage(message);
        return;
    }
    const std::string jobName = m_pendingOfflineCorrectionJobName;
    m_pendingOfflineCorrectionJobName.clear();
    emit statusMessage(QStringLiteral(
        "离线纠偏预检、Vision 和激光 ON 指令确认完成，发送 JBI：%1")
                           .arg(QString::fromStdString(jobName)));
    startRobotJob(jobName);
}

void CRuntimeCoordinator::beginCorrectionSession(CorrectionPurpose purpose)
{
    if (purpose == CorrectionPurpose::None) {
        return;
    }

    if (m_correctionSession.active) {
        // 显式再次启动表示替换旧运行。先让旧关联不可用，再建立新 ID；
        // 即使旧 Vision 结果随后到达，也只能携带旧 sessionId 被丢弃。
        invalidateCorrectionSession(
            QStringLiteral("旧纠偏会话已由新启动请求替换。"),
            false);
    }
                     
    ++m_nextCorrectionSessionId;
    if (m_nextCorrectionSessionId == 0) {
        // 0 永远表示“无会话”。理论上的整数回绕也不能破坏该不变量。
        ++m_nextCorrectionSessionId;
    }
    m_correctionSession = {};
    m_offlineSeamWindowEntered = false;
    m_correctionSession.sessionId = m_nextCorrectionSessionId;
    m_correctionSession.purpose = purpose;
    m_correctionSession.active = true;
    if (purpose == CorrectionPurpose::OnlineCorrection && m_onlineCorrectionWorker != nullptr) {
        const quint64 sessionId = m_correctionSession.sessionId;
        QMetaObject::invokeMethod(m_onlineCorrectionWorker,
            [worker = m_onlineCorrectionWorker, sessionId]() {
                worker->setControlSessionId(sessionId);
            }, Qt::QueuedConnection);
    }
    m_localRegistrationFaulted = false;
    m_localRegistrationRecovery.reset();
    resetOnlineLocalRegistration();
    m_localRegistrationArchiveDirectory = AppPaths::fromDefaultLocation().outputDir().filePath(
        QStringLiteral("online_local_registration/%1_session_%2")
            .arg(QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")))
            .arg(m_correctionSession.sessionId));
    if (m_robotSimWorker != nullptr) {
        const auto safetyOptions = m_localRegistrationSafetyOptions;
        QMetaObject::invokeMethod(m_robotSimWorker,
            [worker = m_robotSimWorker, safetyOptions]() {
                worker->configureLocalRegistrationSafety(safetyOptions);
            }, Qt::QueuedConnection);
    }

    // KCF 工作期间独占逐帧同步通道。清除尚未派发的普通实时状态，但保留
    // m_latestRobotState，待纠偏会话结束后从最新快照恢复实时显示。
    m_hasPendingRobotSimSyncState = false;
    resetVisionFrameDispatch();
    emitCorrectionSyncState(
        CorrectionSyncState::Idle,
        -1,
        QStringLiteral("纠偏运行会话已建立，等待 Vision 跟踪启动。"));
}

void CRuntimeCoordinator::setCorrectionVisionTrackingActive(bool active)
{
    if (!m_correctionSession.active) {
        return;
    }
    m_correctionSession.visionTrackingActive = active;
}

void CRuntimeCoordinator::invalidateCorrectionSession(
    const QString &message,
    bool resumeRealtimeSynchronization)
{
    if (!m_correctionSession.active) {
        return;
    }

    // 必须先关闭 active，再清除 current/pending/request。这样同一事件循环中
    // 随后到达的旧 RobotSim/Vision 结果也无法通过会话匹配。
    m_correctionSession.active = false;
    m_correctionSession.visionTrackingActive = false;
    m_correctionSession.onlineControlWindowActive = false;
    resetOnlineLocalRegistration();
    m_pendingOfflineCorrectionJobName.clear();
    if (m_trackingPurpose == RuntimeTrackingPurpose::OnlineCorrection
        || m_trackingPurpose == RuntimeTrackingPurpose::OfflineCorrection) {
        m_trackingPurpose = RuntimeTrackingPurpose::None;
    }
    resetVisionFrameDispatch();
    emitCorrectionSyncState(CorrectionSyncState::Cleared, -1, message);
    if (resumeRealtimeSynchronization) {
        resumeRealtimeRobotSimSynchronization();
    }
}

void CRuntimeCoordinator::resumeRealtimeRobotSimSynchronization()
{
    if (!m_running
        || m_stopping
        || !canRunRealtimeRobotSimSynchronization()
        || !m_robotSimConnected
        || !m_hasLatestRobotState) {
        return;
    }

    m_pendingRobotSimSyncState = m_latestRobotState;
    m_hasPendingRobotSimSyncState = true;
    dispatchPendingRobotSimStateSynchronization();
}

void CRuntimeCoordinator::emitCorrectionSyncState(
    CorrectionSyncState state,
    qint64 frameId,
    const QString &message,
    const digital_twin::DigitalTwinVisionLaserObservation *observation)
{
    CorrectionSyncUpdate update;
    update.state = state;
    update.purpose = m_correctionSession.purpose;
    update.sessionId = m_correctionSession.sessionId;
    update.frameId = frameId;
    update.consecutiveSyncFailures =
        m_correctionSession.consecutiveSyncFailures;
    update.message = message;
    if (observation != nullptr) {
        update.hasObservation = isUsableCorrectionObservationForFrame(
            frameId, *observation);
        if (update.hasObservation) {
            update.observation = *observation;
        }
    }
    emit correctionSyncUpdated(update);
}

void CRuntimeCoordinator::failCorrectionFrameSynchronization(
    qint64 frameId,
    const QString &message)
{
    constexpr int kMaximumConsecutiveFailures = 3;
    if (!m_correctionSession.active
        || !m_currentVisionFrame.has_value()
        || m_currentVisionFrame->frame.frameId != frameId
        || makeVisionCorrelation(*m_currentVisionFrame).purpose
            != m_correctionSession.purpose) {
        return;
    }

    if (m_correctionSession.purpose == CorrectionPurpose::OnlineCorrection
        && m_currentVisionFrame->hasRobotState && m_visionWorker != nullptr) {
        // 仿真是可缺失的搜索参考；仍将冻结的原图/RobotState 交给视觉线程。
        // 保留在途关联直到 Vision 返回，迟到的 RobotSim 回包由 requestId=0 隔离。
        m_robotSimPriorInFlightRequestId = 0;
        m_currentVisionFrame->requestId = 0;
        const auto observation = digital_twin::makeInvalidVisionLaserObservation(frameId, message);
        m_currentVisionFrame->synchronizedObservation = observation;
        const auto correlation = makeVisionCorrelation(*m_currentVisionFrame);
        ++m_correctionSession.consecutiveSyncFailures;
        m_correctionSession.lastSyncError = message;
        emitCorrectionSyncState(CorrectionSyncState::Failed, frameId, message);
        // 信号订阅方可能同步停止会话；先重验身份再向 worker 投递。
        if (!currentFrameMatches(correlation)) { return; }
        emitCorrectionSyncState(CorrectionSyncState::ImageOnlyReady, frameId,
            QStringLiteral("同帧仿真参考不可用，继续当前原图独立检测。"));
        if (!currentFrameMatches(correlation)) { return; }
        const auto frame = m_currentVisionFrame->frame;
        const auto state = m_currentVisionFrame->robotState;
        const int segment = m_currentVisionFrame->localizationSegmentIndex;
        QMetaObject::invokeMethod(m_visionWorker,
            [worker = m_visionWorker, frame, state, segment, observation, correlation]() {
                worker->submitFrameWithRobotStateObservationAndCorrelation(
                    frame, state, segment, observation, correlation);
            }, Qt::QueuedConnection);
        return;
    }
    m_robotSimPriorInFlightRequestId = 0;
    m_currentVisionFrame.reset();
    m_visionFrameInFlight = false;
    ++m_correctionSession.consecutiveSyncFailures;
    m_correctionSession.lastSyncError = message;
    emitCorrectionSyncState(CorrectionSyncState::Failed, frameId, message);

    if (m_correctionSession.consecutiveSyncFailures
        < kMaximumConsecutiveFailures) {
        dispatchPendingVisionFrame();
        return;
    }

    // purpose 在失效前按值保存；invalidate 会保留 session 快照供终态 UI
    // 关联，但 current/pending/request 会先清除，业务 worker 的 Error 重入
    // 因此无法重新接受已停止会话的任何结果。
    const CorrectionPurpose failedPurpose = m_correctionSession.purpose;
    const QString correctionName =
        failedPurpose == CorrectionPurpose::OnlineCorrection
        ? QStringLiteral("在线纠偏")
        : QStringLiteral("离线纠偏");
    const QString terminalMessage = QStringLiteral(
        "RobotSim 连续 3 帧同步失败，%1已完整停止：%2")
        .arg(correctionName, message);
    invalidateCorrectionSession(terminalMessage);
    emitCorrectionSyncState(
        CorrectionSyncState::StoppedAfterFailures,
        frameId,
        terminalMessage);
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_visionWorker, "stopTracking", Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker]() {
                worker->setTrackingRequested(false);
            },
            Qt::QueuedConnection);
    }
    if (failedPurpose == CorrectionPurpose::OnlineCorrection
        && m_onlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_onlineCorrectionWorker,
            [worker = m_onlineCorrectionWorker, terminalMessage]() {
                worker->failSynchronization(terminalMessage);
            },
            Qt::QueuedConnection);
    } else if (failedPurpose == CorrectionPurpose::OfflineCorrection
               && m_offlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_offlineCorrectionWorker,
            [worker = m_offlineCorrectionWorker, terminalMessage]() {
                worker->failSynchronization(terminalMessage);
            },
            Qt::QueuedConnection);
    }
}

RuntimeVisionCorrelation CRuntimeCoordinator::makeVisionCorrelation(
    const FrozenVisionFrameContext &context) const
{
    RuntimeVisionCorrelation correlation;
    correlation.sessionId = context.correctionSessionId;
    correlation.frameId = context.frame.frameId;
    correlation.onlineControlWindowActive = context.onlineControlWindowActive;
    correlation.workpiecePoseRevision = context.workpiecePoseRevision;
    if (context.purpose == RuntimeTrackingPurpose::OnlineCorrection) {
        correlation.purpose = CorrectionPurpose::OnlineCorrection;
    } else if (context.purpose == RuntimeTrackingPurpose::OfflineCorrection) {
        correlation.purpose = CorrectionPurpose::OfflineCorrection;
    }
    return correlation;
}

bool CRuntimeCoordinator::currentFrameMatches(
    const RuntimeVisionCorrelation &correlation) const
{
    if (!m_correctionSession.active
        || !m_currentVisionFrame.has_value()) {
        return false;
    }

    const FrozenVisionFrameContext &current = *m_currentVisionFrame;
    const RuntimeVisionCorrelation expected = makeVisionCorrelation(current);
    return correlation.sessionId != 0
        && correlation.sessionId == expected.sessionId
        && correlation.purpose == expected.purpose
        && correlation.frameId == expected.frameId
        && correlation.workpiecePoseRevision == expected.workpiecePoseRevision
        && correlation.sessionId == m_correctionSession.sessionId
        && correlation.purpose == m_correctionSession.purpose;
}

void CRuntimeCoordinator::handleRobotSimVirtualPriorReady(
    quint64 requestId,
    const digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    if (requestId == 0
        || requestId != m_robotSimPriorInFlightRequestId
        || !m_visionFrameInFlight
        || !m_currentVisionFrame.has_value()
        || m_currentVisionFrame->requestId != requestId) {
        return;
    }
    if (observation.frameId != m_currentVisionFrame->frame.frameId) {
        if (m_currentVisionFrame->purpose
                == RuntimeTrackingPurpose::OnlineCorrection
            || m_currentVisionFrame->purpose
                == RuntimeTrackingPurpose::OfflineCorrection) {
            failCorrectionFrameSynchronization(
                m_currentVisionFrame->frame.frameId,
                QStringLiteral("RobotSim observation 帧号不匹配：期望 %1，实际 %2。")
                    .arg(m_currentVisionFrame->frame.frameId)
                    .arg(observation.frameId));
        }
        return;
    }

    if (m_currentVisionFrame->purpose
            == RuntimeTrackingPurpose::OnlineCorrection
        || m_currentVisionFrame->purpose
            == RuntimeTrackingPurpose::OfflineCorrection) {
        const RuntimeVisionCorrelation correlation =
            makeVisionCorrelation(*m_currentVisionFrame);
        if (!currentFrameMatches(correlation)) {
            return;
        }
        if (!isUsableCorrectionObservationForFrame(
                m_currentVisionFrame->frame.frameId,
                observation)) {
            const QString detail = observation.message.isEmpty()
                ? QStringLiteral("RobotSim 未返回有效的同帧 observation。")
                : observation.message;
            failCorrectionFrameSynchronization(observation.frameId, detail);
            return;
        }

        // 成功必须在 Vision 业务执行前清零失败计数；随后发布的
        // ObservationReady 与 VisionConsumed 都携带同一份按值 observation。
        m_correctionSession.consecutiveSyncFailures = 0;
        m_correctionSession.lastSyncError.clear();
        m_robotSimPriorInFlightRequestId = 0;
        m_currentVisionFrame->requestId = 0;
        m_currentVisionFrame->synchronizedObservation = observation;
        emitCorrectionSyncState(
            CorrectionSyncState::ObservationReady,
            observation.frameId,
            QStringLiteral("RobotSim 同帧 observation 已就绪。"),
            &observation);

        if (!m_correctionSession.active
            || !m_currentVisionFrame.has_value()
            || !currentFrameMatches(correlation)
            || m_visionWorker == nullptr) {
            return;
        }
        const camera::CameraFrame frame = m_currentVisionFrame->frame;
        const robot_comm::RobotState state = m_currentVisionFrame->robotState;
        QMetaObject::invokeMethod(
            m_visionWorker,
            [worker = m_visionWorker,
             frame,
             state,
             observation,
             correlation]() {
                worker->submitFrameWithRobotStateObservationAndCorrelation(
                    frame, state, -1, observation, correlation);
            },
            Qt::QueuedConnection);
        return;
    }
    if (m_currentVisionFrame->purpose
        != RuntimeTrackingPurpose::GlobalLocalization) {
        return;
    }
    m_robotSimPriorInFlightRequestId = 0;
    m_currentVisionFrame->requestId = 0;
    dispatchPendingRobotSimStateSynchronization();
    emit globalLocateFrameVirtualPriorReady(observation);

    if (m_visionWorker == nullptr
        || m_trackingPurpose != RuntimeTrackingPurpose::GlobalLocalization) {
        m_currentVisionFrame.reset();
        m_visionFrameInFlight = false;
        completeGlobalLocalizationTrackingDrain();
        return;
    }

    const camera::CameraFrame frame = m_currentVisionFrame->frame;
    const robot_comm::RobotState state = m_currentVisionFrame->robotState;
    const int localizationSegmentIndex =
        m_currentVisionFrame->localizationSegmentIndex;
    if (m_activeGlobalLocalizationSources.laserSource
        == GlobalLocateVirtualLaserSource::RobotSim) {
        QMetaObject::invokeMethod(
            m_visionWorker,
            [worker = m_visionWorker,
             frame,
             state,
             localizationSegmentIndex,
             observation]() {
                worker->submitFrameWithRobotStateAndObservation(
                    frame,
                    state,
                    localizationSegmentIndex,
                    observation);
            },
            Qt::QueuedConnection);
        return;
    }

    // RobotSim 仅作为配准点云来源时，真实激光提取仍消费 OCCT 先验。
    QMetaObject::invokeMethod(
        m_visionWorker,
        [worker = m_visionWorker,
         frame,
         state,
         localizationSegmentIndex]() {
            worker->submitFrameWithRobotState(
                frame,
                state,
                localizationSegmentIndex);
        },
        Qt::QueuedConnection);
}

void CRuntimeCoordinator::completeGlobalLocalizationTrackingDrain()
{
    if (!m_globalLocalizationDrainRequested
        || m_visionFrameInFlight
        || m_pendingVisionFrame.has_value()) {
        return;
    }

    m_globalLocalizationDrainRequested = false;
    m_trackingPurpose = RuntimeTrackingPurpose::None;
    if (m_visionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_visionWorker, "stopTracking", Qt::QueuedConnection);
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker]() {
                worker->setTrackingRequested(false);
            },
            Qt::QueuedConnection);
    }
    emit globalLocalizationTrackingDrained();
}

void CRuntimeCoordinator::updateGlobalLocalizationSegmentState(
    const robot_comm::RobotState &state)
{
    if (m_trackingPurpose != RuntimeTrackingPurpose::GlobalLocalization) {
        return;
    }

    const int posflag = state.posflag == 0 ? 0 : 1;
    if (posflag == m_globalLocalizationLastPosflag) {
        return;
    }
    m_globalLocalizationLastPosflag = posflag;
    if (posflag == 1) {
        m_globalLocalizationCurrentSegmentIndex =
            ++m_globalLocalizationSegmentSequence;
    } else {
        m_globalLocalizationCurrentSegmentIndex = 0;
    }
}

void CRuntimeCoordinator::routeTrackingResult(const RuntimeTrackingResult &result)
{
    if (!m_running || m_stopping) {
        return;
    }
    if (!m_currentVisionFrame.has_value() || !m_visionFrameInFlight) {
        return;
    }

    const FrozenVisionFrameContext current = *m_currentVisionFrame;
    const bool isCorrection =
        current.purpose == RuntimeTrackingPurpose::OnlineCorrection
        || current.purpose == RuntimeTrackingPurpose::OfflineCorrection;
    if (isCorrection) {
        // 迟到结果必须完整匹配 session/purpose/frame。匹配失败时尤其不能
        // 清除 current，否则旧会话结果会错误地释放新会话的在途槽。
        if (!currentFrameMatches(result.correlation)
            || result.frameId != current.frame.frameId) {
            return;
        }
    } else if (current.purpose != RuntimeTrackingPurpose::GlobalLocalization
               || result.frameId != current.frame.frameId
               || (m_trackingPurpose
                       != RuntimeTrackingPurpose::GlobalLocalization
                   && !m_globalLocalizationDrainRequested)) {
        return;
    }

    const qint64 resultReceivedMs = QDateTime::currentMSecsSinceEpoch();
    const auto synchronizedObservation = current.synchronizedObservation;
    m_currentVisionFrame.reset();
    m_visionFrameInFlight = false;
    if (current.purpose == RuntimeTrackingPurpose::OnlineCorrection) {
        verifyOnlineLocalRegistration(current, result);
        submitOnlineLocalRegistration(current, result);
    }
    if (isCorrection) {
        emitCorrectionSyncState(
            CorrectionSyncState::VisionConsumed,
            result.frameId,
            QStringLiteral("Vision 已消费同会话同帧输入。"),
            synchronizedObservation.has_value()
                ? &synchronizedObservation.value()
                : nullptr);
    }

    RuntimeTrackingResult routedResult = result;
    if (current.purpose == RuntimeTrackingPurpose::OnlineCorrection
        && m_pendingOnlineCorrectionConfiguration.visionDiagnostics.enabled
        && !routedResult.diagnostic.has_value()) {
        routedResult.diagnostic.emplace();
    }
    if (routedResult.diagnostic.has_value()) {
        routedResult.diagnostic->runtimeReceivedMs = resultReceivedMs;
        routedResult.diagnostic->cameraTimestampMs = current.frame.timestamp.isValid()
            ? current.frame.timestamp.toMSecsSinceEpoch() : -1;
        routedResult.diagnostic->robotState = current.robotState;
        routedResult.diagnostic->robotStateReceivedMs = current.robotStateReceivedMs;
    }
    RuntimeTrackingResult displayResult = routedResult;
    // 图像只交给记录线程；UI 使用独立的同帧原图展示路径。
    if (displayResult.diagnostic.has_value()) {
        displayResult.diagnostic->sourceImage = {};
        displayResult.diagnostic->enhancedImage = {};
    }
    if (isCorrection && synchronizedObservation.has_value()) {
        displayResult.frameContext = RuntimeTrackingFrameContext{
            current.frame, synchronizedObservation.value()};
    }
    emit trackingResultReady(displayResult);
    // UI 可在公开结果回调中立即停止并重新启动。发出信号前的校验不能
    // 授权回调之后的路由，否则旧样本会排在新会话的控制初始化之后。
    if (isCorrection
        && (!m_correctionSession.active
            || current.correctionSessionId != m_correctionSession.sessionId
            || result.correlation.purpose != m_correctionSession.purpose)) {
        dispatchPendingVisionFrame();
        return;
    }
    if (m_robotWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_robotWorker,
            [worker = m_robotWorker, result]() {
                worker->updateDetectionStatus(result);
            },
            Qt::QueuedConnection);
    }
    if (current.purpose == RuntimeTrackingPurpose::OnlineCorrection
        && m_onlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_onlineCorrectionWorker,
            [worker = m_onlineCorrectionWorker, routedResult]() {
                worker->appendTrackingResult(routedResult);
            },
            Qt::QueuedConnection);
    } else if (current.purpose == RuntimeTrackingPurpose::OfflineCorrection
               && m_offlineCorrectionWorker != nullptr) {
        QMetaObject::invokeMethod(
            m_offlineCorrectionWorker,
            [worker = m_offlineCorrectionWorker, result]() {
                worker->appendTrackingResult(result);
            },
            Qt::QueuedConnection);
    }
    dispatchPendingVisionFrame();
    completeGlobalLocalizationTrackingDrain();
}

void CRuntimeCoordinator::shutdownThread(QThread &thread)
{
    thread.quit();
    thread.wait();
}

} // namespace i2r_rlc::ui
