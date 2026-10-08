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
 * @file        RuntimeWorkerAdapters.h
 * @brief       运行时线程 worker 适配器
 * @details     提供 Qt 线程中的相机、机器人通信、视觉跟踪和控制状态机事件适配层。
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
#ifndef I2R_RLC_UI_RUNTIME_WORKER_ADAPTERS_H
#define I2R_RLC_UI_RUNTIME_WORKER_ADAPTERS_H

#include <memory>
#include <optional>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>

#include "CalibrationCommand.h"
#include "CameraService.h"
#include "geometry/WeldImageToSpaceProjector.h"
#include "JobCommand.h"
#include "LocalizationApplicationService.h"
#include "OfflineCorrectionController.h"
#include "OnlineCorrectionRunRecorder.h"
#include "OnlineCorrectionController.h"
#include "IRobotSimTransport.h"
#include "RobotCommandTransport.h"
#include "RobotProtocolCodec.h"
#include "RobotSimDigitalTwinService.h"
#include "RobotSimSceneConfigReader.h"
#include "RuntimeTrackingTypes.h"
#include "TeachTrajectoryRecorder.h"
#include "MotomanMh12InverseKinematics.h"
#include "WeldmentVirtualLaserPriorGenerator.h"
#include "WeldmentVirtualPointCloudGenerator.h"
#include "track/LegacyOnlineKcfDetector.h"

namespace i2r_rlc::ui {

/**
 * @brief 相机线程适配器。
 *
 * 第一版只提供线程生命周期和状态事件；真实采集后端继续由 Camera 模块演进。
 */
class CCameraWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CCameraWorkerAdapter(QObject *parent = nullptr);
    explicit CCameraWorkerAdapter(
        std::unique_ptr<camera::ICameraDevice> device,
        int captureIntervalMs = 33,
        QObject *parent = nullptr);

public slots:
    void start();
    void stop();
    void publishFrame(const camera::CameraFrame &frame);
    void captureLatestFrame();
    /// @brief 设置真实相机曝光时间，单位 us；未注入相机时返回明确状态。
    void setExposureTimeUs(int exposureTimeUs);

signals:
    void frameReady(const camera::CameraFrame &frame);
    void statusChanged(const QString &message);

private:
    std::unique_ptr<camera::ICameraDevice> m_device;
    QTimer *m_captureTimer = nullptr;
    int m_captureIntervalMs = 33;
};

/**
 * @brief 机器人通信线程适配器。
 */
class CRobotCommWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CRobotCommWorkerAdapter(QObject *parent = nullptr);
    explicit CRobotCommWorkerAdapter(
        std::unique_ptr<robot_comm::IRobotCommandTransport> transport,
        QObject *parent = nullptr);

public slots:
    void connectRobot();
    void disconnectRobot();
    void pollRobotState();
    void updateState(const robot_comm::RobotState &state);
    /// @brief 更新旧 N/O/P 协议上下文中的激光开关状态。
    void setLaserEnabled(bool enabled);
    /**
     * @brief 在机器人线程确认激光 ON 指令已通过本次 N/D 事务发送成功。
     * 关闭时先开启；已开启时重新确认通信，不表示设备实际出光反馈。
     */
    bool ensureOfflineLaserEnabled();
    /**
     * @brief 在线启动前确认最新停止状态，再请求清除上轮 P001 就绪标志。
     *
     * tracker/laser 置 1；普通 N/D 检查成功且程序停止、无未完成 O 批次后，
     * 再发送一次 N.init=1。D 不含 B005 读回，true 只确认初始化事务成功；
     * 实际变量清除仍依赖匹配的控制柜实现，作业启动由后续 J/D 确认。
     */
    bool ensureOnlineCorrectionReady();
    void setTrackingRequested(bool enabled);
    /// @brief 更新旧协议 OfflineStartCount 握手值，并由后续 N/O/P 请求携带。
    void setOfflineStartCount(int count);
    void updateDetectionStatus(const RuntimeTrackingResult &result);
    void sendCorrectionCommand(const robot_comm::RobotCorrectionCommand &command);
    void sendRealtimeCorrectionCommand(
        const robot_comm::RobotRealtimeCorrectionCommand &command);
    /// @brief 发送旧 C 自动标定下一位置命令。
    void sendCalibrationCommand(
        const robot_comm::RobotCalibrationCommand &command);
    /// @brief 发送旧 J 作业启动命令。
    void sendJobStartCommand(
        const robot_comm::RobotJobStartCommand &command);

signals:
    void robotStateReady(const robot_comm::RobotState &state);
    void commandReceiptReady(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &state);
    void diagnosticsReady(
        const robot_comm::RobotCommunicationDiagnostics &diagnostics);
    void statusChanged(const QString &message);

private:
    robot_comm::RobotProtocolContext protocolContext() const;
    void ensurePollTimer();
    void ensureReconnectTimer();
    void flushRealtimeCorrectionCommand();
    void recordDiagnostics(const robot_comm::RobotCommandReceipt &receipt);
    bool shouldReconnectForFailure(
        const robot_comm::RobotCommandReceipt &receipt) const;
    void stopPollingAfterTransportFailure(bool emitSnapshot = true);
    void scheduleReconnectAfterTransportFailure();
    void attemptReconnect();

    std::unique_ptr<robot_comm::IRobotCommandTransport> m_transport;
    QTimer *m_pollTimer = nullptr;
    QTimer *m_reconnectTimer = nullptr;
    robot_comm::RobotState m_state;
    robot_comm::RobotRealtimeCorrectionCommand m_latestRealtimeCommand;
    int m_trackerEnabled = 0;
    int m_laserEnabled = 1;
    int m_filterEnabled = 0;
    int m_offlineSignal = 0;
    int m_offlineStartCount = 0;
    int m_firstDetectionSucceeded = 0;
    bool m_keepConnectionRequested = false;
    bool m_realtimeFlushScheduled = false;
    int m_reconnectAttemptCount = 0;
    robot_comm::RobotCommunicationDiagnostics m_diagnostics;
};

/**
 * @brief 视觉跟踪线程适配器。
 *
 * 只承接旧 det_type==2 的仿真辅助 Gabor + KCF/MKCF 检测链。
 */
class CVisionTrackingWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CVisionTrackingWorkerAdapter(QObject *parent = nullptr);

    void setSimulatedCallbacks(const SimulatedVisionCallbacks &callbacks);

public slots:
    void configure(
        const RuntimeTrackingOptions &options,
        const vision::WeldProjectionContext &projectionContext);
    void updateRobotState(const robot_comm::RobotState &state);
    /// @brief 按值接收 DigitalTwin 线程发布的 Vision 仿真观测快照。
    void updateDigitalTwinVisionObservation(
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void startTracking();
    void stopTracking();
    void submitFrame(const camera::CameraFrame &frame);
    /**
     * @brief 使用相机曝光时刻对应的机器人状态处理一帧。
     *
     * 该接口避免耗时视觉队列读取“处理时最新状态”，从而把非检测窗口图像
     * 错误归入点云，或漏掉连续定位面之间的 posflag 边沿。
     */
    void submitFrameWithRobotState(
        const camera::CameraFrame &frame,
        const robot_comm::RobotState &state,
        int localizationSegmentIndex);
    /**
     * @brief 使用冻结状态和纠偏关联键处理一帧。
     *
     * correlation 只附着到本次同步调用产生的结果，不写入跨帧缓存。这样即使
     * worker 队列中存在停止/重启事件，旧帧也不会借用新会话的关联信息。
     */
    void submitFrameWithRobotStateAndCorrelation(
        const camera::CameraFrame &frame,
        const robot_comm::RobotState &state,
        int localizationSegmentIndex,
        const RuntimeVisionCorrelation &correlation);
    /**
     * @brief 使用同一相机帧绑定的机器人状态和 RobotSim 观测执行检测。
     *
     * 无效或帧号不匹配的观测不会回退到旧先验，防止真实线被错误位姿引导。
     */
    void submitFrameWithRobotStateAndObservation(
        const camera::CameraFrame &frame,
        const robot_comm::RobotState &state,
        int localizationSegmentIndex,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    /**
     * @brief 使用冻结状态、同帧 RobotSim observation 和纠偏关联键处理一帧。
     */
    void submitFrameWithRobotStateObservationAndCorrelation(
        const camera::CameraFrame &frame,
        const robot_comm::RobotState &state,
        int localizationSegmentIndex,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const RuntimeVisionCorrelation &correlation);

signals:
    void trackingResultReady(const RuntimeTrackingResult &result);
    void statusChanged(const QString &message);

private:
    RuntimeTrackingResult processDetType2(const camera::CameraFrame &frame);
    /// @brief 全局定位专用：仿真激光线引导 Gabor + Steger 提取整条真实激光线并逐点投影。
    RuntimeTrackingResult processGlobalLocateLineExtraction(const camera::CameraFrame &frame);
    bool makeObservation(
        qint64 frameId,
        digital_twin::DigitalTwinVisionLaserObservation *observation);
    RuntimeTrackingOptions m_options;
    vision::WeldProjectionContext m_projectionContext;
    robot_comm::RobotState m_robotState;
    SimulatedVisionCallbacks m_callbacks;
    std::optional<digital_twin::DigitalTwinVisionLaserObservation>
        m_latestDigitalTwinObservation;
    bool m_requireSynchronizedObservation = false;
    RuntimeVisionCorrelation m_activeCorrelation;
    vision::CLegacyOnlineKcfDetector m_detector;
    bool m_trackingActive = false;
    int m_currentLocalizationSegmentIndex = -1;
};

/**
 * @brief 全局定位 OCCT/RobotSim 虚拟激光先验后台 worker。
 *
 * OpenCASCADE 几何求交和 RobotSim 通信都必须与 UI、逐帧 Vision worker
 * 隔离。requestId 用于丢弃停止、切换来源或重启流程后才返回的过期结果。
 */
class CGlobalLocatePriorWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CGlobalLocatePriorWorkerAdapter(QObject *parent = nullptr);
    /**
     * @brief 使用调用方提供的 RobotSim transport 创建 worker。
     *
     * 仅测试和受控集成路径使用该构造函数。transport 的所有权转移给
     * RobotSimDigitalTwinService，并在本 worker 所在线程中独占访问；生产代码
     * 继续使用默认构造函数创建真实 ZMQ transport。
     */
    explicit CGlobalLocatePriorWorkerAdapter(
        std::unique_ptr<digital_twin::IRobotSimTransport> transport,
        QObject *parent = nullptr);

    /**
     * @brief 将全局定位的虚拟到真实修正矩阵写回 RobotSim 工件。
     *
     * 本函数必须在该 worker 所属线程调用。修正矩阵位于工件父级坐标系，
     * 写回时左乘工件当前相对位姿，并通过绝对位姿回读确认 SET_POSE 生效。
     */
    digital_twin::DigitalTwinCommandResult applyGlobalLocalizationCorrection(
        const localization::RigidTransform &virtualToRealCorrection);

public slots:
    void generateVirtualPrior(
        quint64 requestId,
        const digital_twin::WeldmentVirtualLaserRequest &request);

    /**
     * @brief 更新 RobotSim 动态端口和当前场景对象 ID。
     */
    void configureRobotSim(
        const digital_twin::RobotSimEndpointConfig &endpoint,
        const digital_twin::RobotSimSceneConfig &scene);

    /**
     * @brief 连接手动调整的 RobotSim 场景，只轮询仿真激光线和焊缝点。
     *
     * 该模式不会读取真实机器人状态，也不会发送 SET_Q_VEC。
     */
    void connectRobotSimManual(
        const digital_twin::RobotSimEndpointConfig &endpoint,
        const digital_twin::RobotSimSceneConfig &scene);

    /// @brief 轮询一帧手动 RobotSim 场景参考，并按固定节拍继续调度。
    void pollRobotSimManualReference();

    /**
     * @brief 连接 RobotSim，同步当前关节值并获取首条仿真激光线。
     */
    void connectRobotSim(
        const digital_twin::RobotSimEndpointConfig &endpoint,
        const digital_twin::RobotSimSceneConfig &scene,
        const robot_comm::RobotState &initialState);

    /**
     * @brief 同步最新关节值并获取用于 UI 实时显示的仿真激光线。
     */
    void synchronizeRobotSimState(
        quint64 requestId,
        const robot_comm::RobotState &state);

    /**
     * @brief 在同一批次中同步关节值并获取指定帧的仿真观测。
     */
    void generateRobotSimVirtualPrior(
        quint64 requestId,
        qint64 frameId,
        const robot_comm::RobotState &state,
        int positioningSegmentIndex);

    /// @brief 断开 RobotSim 并重置逆解分支。
    void resetRobotSim();

signals:
    void virtualPriorReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void robotSimVirtualPriorReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void robotSimRealtimeVirtualPriorReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void robotSimRealtimeVirtualReferenceReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVirtualReferenceResult &reference);
    void robotSimConnectionStateChanged(
        bool connected,
        const QString &message);
    void robotSimManualConnectionStateChanged(
        bool connected,
        const QString &message);
    void robotSimManualVirtualReferenceReady(
        const digital_twin::DigitalTwinVirtualReferenceResult &reference);
    void robotSimStateSynchronized(
        quint64 requestId,
        bool succeeded,
        const QString &message);

private:
    enum class RobotSimConnectionMode {
        Disconnected,
        ManualSimulation,
        RealScene
    };

    bool buildRobotSimJointVectors(
        const robot_comm::RobotState &state,
        std::vector<double> &robotJointValuesRad,
        std::vector<double> &positionerJointValuesRad,
        QString &message);

    digital_twin::WeldmentVirtualLaserPriorGenerator m_generator;
    digital_twin::RobotSimDigitalTwinService m_robotSimService;
    digital_twin::CMotomanMh12InverseKinematics m_robotInverseKinematics;
    digital_twin::RobotSimEndpointConfig m_robotSimEndpoint;
    digital_twin::RobotSimSceneConfig m_robotSimScene;
    bool m_robotSimConfigured = false;
    bool m_robotSimConnected = false;
    RobotSimConnectionMode m_robotSimConnectionMode =
        RobotSimConnectionMode::Disconnected;
    quint64 m_robotSimManualPollGeneration = 0;
};

/**
 * @brief 独立全局定位工作线程的不可变输入快照。
 *
 * UI 线程完成采点和定位面结算后按值提交本结构。后台 worker 不再访问
 * UiController、QWidget 或仍可能变化的采点容器。
 */
struct StandaloneGlobalLocalizationRequest {
    qint64 frameId = -1;
    QString outputDirectory;
    QString weldmentPath;
    GlobalLocateVirtualPointCloudSource virtualPointCloudSource =
        GlobalLocateVirtualPointCloudSource::Weldment;
    std::vector<std::vector<localization::Point3D_mm>> realSegments;
    std::vector<std::vector<localization::Point3D_mm>> robotSimVirtualSegments;
    int trackingFrameCount = 0;
    int triggerRejectedFrameCount = 0;
    int invalidFrameCount = 0;
    int priorSuccessCount = 0;
    int priorFailureCount = 0;
    QString lastTrackingMessage;
    QString lastPriorMessage;
};

/**
 * @brief 独立全局定位工作线程返回给 UI 的值结果。
 *
 * 点云和矩阵已经在 worker 线程落盘；最终修正矩阵同时按值交给运行时写回
 * RobotSim，UI 只消费日志、流程更新和最终状态。
 */
struct StandaloneGlobalLocalizationResult {
    bool succeeded = false;
    QString message;
    QStringList infoMessages;
    QStringList warningMessages;
    /// @brief 分段累计 ICP 输出的最终虚拟到真实修正矩阵。
    localization::RigidTransform correctionTransform;
    bool hasSegmentedUpdate = false;
    localization::LocalizationWorkflowUpdate segmentedUpdate;
    bool hasWholeUpdate = false;
    localization::LocalizationWorkflowUpdate wholeUpdate;
};

/**
 * @brief 全局定位求解工作线程适配器。
 *
 * 本对象只能在所属 QThread 中调用。它独占定位服务，并串行执行 weldment
 * 解析、点云文件写入、分段/整体 ICP 和 RMSE，完成后仅发送值结果。
 */
class CGlobalLocalizationWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CGlobalLocalizationWorkerAdapter(QObject *parent = nullptr);

public slots:
    void apply(
        quint64 requestId,
        const StandaloneGlobalLocalizationRequest &request);

signals:
    void resultReady(
        quint64 requestId,
        const StandaloneGlobalLocalizationResult &result);

private:
    localization::LocalizationApplicationService m_localizationService;
};

/**
 * @brief 在线纠偏控制线程适配器。
 */
class COnlineCorrectionWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit COnlineCorrectionWorkerAdapter(QObject *parent = nullptr);

public slots:
    void configure(const RuntimeOnlineCorrectionConfiguration &configuration);
    /// @brief 配置在线控制器并返回轨迹与 RBCALIB 是否均可用于本轮启动。
    bool configureForStart(
        const RuntimeOnlineCorrectionConfiguration &configuration);
    void startInitialDetection();
    void updateRobotState(const robot_comm::RobotState &state);
    void handleRobotCommandReceipt(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &state);
    void appendTrackingResult(const RuntimeTrackingResult &result);
    void finishOnlineSegment();
    /**
     * @brief 因逐帧 RobotSim 同步连续失败而终止在线纠偏。
     *
     * 入口先复用 controller reset 清除检测样本和命令状态，再发布不携带
     * O/P 命令的 Error 快照。轨迹和标定配置仍由 controller 既有 reset
     * 语义保留，下一次用户启动可建立全新 runtime session。
     */
    void failSynchronization(const QString &message);
    void reset();

signals:
    void onlineCorrectionUpdated(const control::OnlineCorrectionUpdate &update);
    void robotCommandReady(const robot_comm::RobotCorrectionCommand &command);
    void realtimeCommandReady(
        const robot_comm::RobotRealtimeCorrectionCommand &command);
    void startTrackingRequested();
    void stopTrackingRequested();
    void statusChanged(const QString &message);

private:
    void publishUpdate(const control::OnlineCorrectionUpdate &update);
    /// @brief 为必须收到机器人握手的等待阶段设置一次性超时保护。
    void armPhaseTimeout(control::OnlineCorrectionPhase phase);
    void reportRecordingFailure(const QString &context);

    control::COnlineCorrectionController m_controller;
    robot_comm::RobotState m_robotState;
    RuntimeOnlineCorrectionConfiguration m_configuration;
    vision::PositionerCalibration m_positionerCalibration;
    COnlineCorrectionRunRecorder m_runRecorder;
    control::OnlineCorrectionPhase m_timeoutPhase =
        control::OnlineCorrectionPhase::Idle;
    quint64 m_phaseTimeoutGeneration = 0;
};

/**
 * @brief 有匹配离线纠偏控制线程适配器。
 *
 * 本 worker 将旧 OffLineCor 的按钮二态流程放到 runtime 线程中执行：
 * 开始检测时请求 Vision tracking，结束检测时执行 ICP 并通过 RobotComm 发送 O，
 * 最后等待控制柜 D.offline==5 确认。
 */
class COfflineCorrectionWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit COfflineCorrectionWorkerAdapter(QObject *parent = nullptr);

public slots:
    void configure(
        const std::string &archiveFilePath,
        const control::OfflineCorrectionOptions &options);
    void startDetection(int detType);
    void finishDetectionAndCompute();
    /**
     * @brief 输入机器人状态，监听本轮扫描运行状态和 OfflineRun 握手。
     *
     * 启动前先确认 sStart==0、B040==0；专用 JBI 只扫描焊缝，
     * 按 STARTDET/ENDDET 检测窗口收集数据，不依赖 posflag 或定位段次数。
     * 锁存本轮 B040 触发，等待 sStart==0 才结束检测。
     * detflag 不再作为离线纠偏总流程完成条件。
     */
    void updateRobotState(const robot_comm::RobotState &state);
    void appendTrackingResult(const RuntimeTrackingResult &result);
    /**
     * @brief 终止当前离线纠偏，并保留同步链失败原因供 UI 和日志诊断。
     *
     * 该入口只由 runtime 在连续三帧 RobotSim 同步失败后调用。它复用既有
     * OfflineCorrectionController 的 Error 语义，但不会生成或发送 O 命令。
     */
    void failSynchronization(const QString &message);
    void handleRobotCommandReceipt(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &state);
    void reset();

signals:
    void offlineCorrectionUpdated(const control::OfflineCorrectionUpdate &update);
    void robotCommandReady(const robot_comm::RobotCorrectionCommand &command);
    void startTrackingRequested();
    void stopTrackingRequested();
    /// @brief 请求 RobotComm 在旧 N/O/P 协议中回送 OfflineStartCount。
    void offlineStartCountChanged(int count);
    void statusChanged(const QString &message);

private:
    void publishUpdate(const control::OfflineCorrectionUpdate &update);
    /// @brief 完成控制柜确认，并覆盖保存最近一次成功离线纠偏的诊断文件。
    void completeSuccessfulCorrection();
    /// @brief 在纠偏 worker 线程原子覆盖本轮原始采集点；启动时允许空集以清除旧点。
    void saveDetectionPoints();

    control::COfflineCorrectionController m_controller;
    bool m_waitingForRobotAck = false;
    bool m_preflightReady = false;
    bool m_scanProgramStarted = false;
    bool m_scanProgramRunning = false;
    bool m_offlineRunReleased = false;
    bool m_offlineRunEndLatched = false;
    bool m_offlineRunAcknowledged = false;
    bool m_finishRequested = false;
    bool m_allTeachPointCommandsCompleted = false;
    int m_expectedTeachPointCount = 0;
    quint64 m_jobStopGraceGeneration = 0;
    quint64 m_robotAckGeneration = 0;
    robot_comm::RobotState m_lastRobotState;
    QString m_diagnosticOutputDirectory;
};

/**
 * @brief 示教轨迹保存线程适配器。
 *
 * 负责在 runtime 线程中按 RobotState::s_Bsavetraj 自动写入旧式临时轨迹文件，
 * 并在用户确认时复制为 `_W/_T` 示教轨迹。
 */
class CTeachTrajectoryRecorderWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CTeachTrajectoryRecorderWorkerAdapter(QObject *parent = nullptr);

public slots:
    void configureTempDirectory(const QString &tempDirectory);
    void startRecording();
    void updateRobotState(const robot_comm::RobotState &state);
    void saveCapturedTrajectory(
        const QString &outputDirectory,
        const QString &baseName);
    void reset();

signals:
    void teachTrajectoryRecorderUpdated(
        const control::TeachTrajectoryRecorderSnapshot &snapshot);
    void statusChanged(const QString &message);

private:
    void publishSnapshot(
        const control::TeachTrajectoryRecorderSnapshot &snapshot);
    bool shouldPublishRobotStateSnapshot(
        const control::TeachTrajectoryRecorderSnapshot &snapshot);

    control::CTeachTrajectoryRecorder m_recorder;
    long m_lastPublishedSaveSignal = 0;
    bool m_lastPublishedSegmentActive = false;
    int m_lastPublishedTrajectoryPointCount = 0;
    int m_lastPublishedTeachPointCount = 0;
    int m_lastPublishedAllPointCount = 0;
};

} // namespace i2r_rlc::ui

Q_DECLARE_METATYPE(i2r_rlc::control::TeachTrajectoryRecorderSnapshot)
Q_DECLARE_METATYPE(i2r_rlc::ui::StandaloneGlobalLocalizationRequest)
Q_DECLARE_METATYPE(i2r_rlc::ui::StandaloneGlobalLocalizationResult)

#endif // I2R_RLC_UI_RUNTIME_WORKER_ADAPTERS_H
