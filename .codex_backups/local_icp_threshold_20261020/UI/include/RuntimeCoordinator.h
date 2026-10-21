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
 * @file        RuntimeCoordinator.h
 * @brief       Qt 运行时线程编排器
 * @details     负责创建 Camera、Vision、Control 和 RobotComm worker 线程，并统一转发事件。
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
#ifndef I2R_RLC_UI_RUNTIME_COORDINATOR_H
#define I2R_RLC_UI_RUNTIME_COORDINATOR_H

#include <memory>
#include <optional>

#include <QObject>
#include <QElapsedTimer>
#include <QThread>

#include "OnlineLocalRegistrationWorker.h"
#include "RuntimeWorkerAdapters.h"

namespace i2r_rlc::ui {

/**
 * @brief UI 层运行时线程编排器。
 *
 * 本类不处理按钮语义和显示文本，只管理 QThread、worker 生命周期和跨线程事件路由。
 */
class CRuntimeCoordinator final : public QObject {
    Q_OBJECT

public:
    explicit CRuntimeCoordinator(QObject *parent = nullptr);
    explicit CRuntimeCoordinator(
        std::unique_ptr<robot_comm::IRobotCommandTransport> robotTransport,
        QObject *parent = nullptr);
    /**
     * @brief 使用可替换 RobotComm 和 RobotSim transport 创建运行时。
     *
     * 两个 transport 的所有权在 `start()` 时分别转移给对应 worker。该构造
     * 路径用于跨线程时序测试；生产默认构造仍创建现有真实 transport。
     */
    CRuntimeCoordinator(
        std::unique_ptr<robot_comm::IRobotCommandTransport> robotTransport,
        std::unique_ptr<digital_twin::IRobotSimTransport> robotSimTransport,
        QObject *parent = nullptr);
    ~CRuntimeCoordinator() override;

    CRuntimeCoordinator(const CRuntimeCoordinator &) = delete;
    CRuntimeCoordinator &operator=(const CRuntimeCoordinator &) = delete;

    /**
     * @brief 启动运行时 worker 线程。
     */
    void start();

    /**
     * @brief 停止运行时 worker 线程。
     */
    void stop();

    /**
     * @brief 设置仿真输入回调。
     */
    void setSimulatedCallbacks(const SimulatedVisionCallbacks &callbacks);

    /**
     * @brief 注入真实流式相机设备。
     *
     * 必须在 `start()` 前调用；设备随后由 Camera worker 线程独占。
     * 相机不会随运行时自动启动，需要由 `startCamera()` 显式触发。
     */
    bool setCameraDevice(
        std::unique_ptr<camera::ICameraDevice> device,
        int captureIntervalMs = 33);

    /**
     * @brief 显式启动真实相机采集。
     *
     * 运行时启动只创建 Camera worker，不主动连接厂商 SDK，避免程序启动阶段
     * 在未确认设备环境时进入连续采集。
     */
    void startCamera();

    /**
     * @brief 配置跟踪检测链。
     */
    void configureTracking(
        const RuntimeTrackingOptions &options,
        const vision::WeldProjectionContext &projectionContext);

    /**
     * @brief 配置在线轨迹、示教位姿和 RBCALIB 文件。
     */
    void configureOnlineCorrection(
        const RuntimeOnlineCorrectionConfiguration &configuration);

    /** @brief UI 线程热更新在线选项；关闭定位时立即撤销未执行的位姿写回。 */
    void updateOnlineProcessingOptions(bool localEnabled, const vision::KcfFusionOptions &fusion);
    /// @brief UI 线程读取当前定位开关，不代表现场配准一定可用。
    bool onlineLocalRegistrationEnabled() const;
    /// @brief UI 线程读取已归一化的在线融合配置。
    vision::KcfFusionOptions onlineFusionOptions() const;

    /**
     * @brief 配置有匹配离线纠偏存档。
     */
    void configureOfflineCorrectionArchive(
        const std::string &archiveFilePath,
        const control::OfflineCorrectionOptions &options = {});

    /**
     * @brief 重新尝试连接机器人控制柜。
     */
    void connectRobot();

    /**
     * @brief 设置真实相机曝光时间。
     *
     * @param exposureTimeUs 曝光时间，单位 us；请求会投递到 Camera worker 线程。
     */
    void setCameraExposureUs(int exposureTimeUs);

    /**
     * @brief 设置旧机器人协议上下文中的激光开关。
     *
     * 后续周期 N 以及业务 O/P 请求会携带最新 `s_laser` 值。
     */
    void setLaserEnabled(bool enabled);

    /**
     * @brief 请求机器人进入下一自动标定位置，发送旧 C 命令。
     */
    void sendCalibrationCommand(
        const robot_comm::RobotCalibrationCommand &command);

    /**
     * @brief 发送旧 O 示教点更新命令。
     *
     * 调用方必须已经生成完整的 RobotCorrectionCommand；本函数只负责
     * 投递到 RobotComm worker 线程。
     */
    void sendCorrectionCommand(
        const robot_comm::RobotCorrectionCommand &command);

    /**
     * @brief 启动控制柜已有 JBI 作业，发送旧 J 命令。
     */
    void startRobotJob(const std::string &jobName);

    /**
     * @brief 向运行时发布相机帧。
     */
    void publishCameraFrame(const camera::CameraFrame &frame);

    /**
     * @brief 向运行时发布机器人状态。
     */
    void publishRobotState(const robot_comm::RobotState &state);

    /**
     * @brief 向 Vision worker 按值发布 DigitalTwin 仿真激光观测。
     *
     * 该接口只做 queued 投递，不共享外部容器引用；Vision worker 在自身线程
     * 保存最近一份快照，并在 start/stop/reset 边界清空。
     */
    void publishDigitalTwinVisionObservation(
        const digital_twin::DigitalTwinVisionLaserObservation &observation);

    /**
     * @brief 在独立 DigitalTwin 线程生成一帧 weldment 虚拟激光先验。
     */
    void requestGlobalLocateVirtualPrior(
        quint64 requestId,
        const digital_twin::WeldmentVirtualLaserRequest &request);

    /// @brief 设置下一次全局定位使用的仿真激光来源，不改变 RobotSim 连接。
    void setGlobalLocateVirtualLaserSource(
        GlobalLocateVirtualLaserSource source);

    /// @brief 设置下一次全局定位配准使用的虚拟点云来源。
    void setGlobalLocateVirtualPointCloudSource(
        GlobalLocateVirtualPointCloudSource source);

    /**
     * @brief 连接手动 RobotSim 场景，仅持续查询仿真激光线和焊缝点。
     */
    void connectRobotSimManual(
        const digital_twin::RobotSimEndpointConfig &endpoint,
        const digital_twin::RobotSimSceneConfig &scene);

    /**
     * @brief 在 DigitalTwin 线程配置并实际确认 RobotSim 端点。
     */
    void connectRobotSim(
        const digital_twin::RobotSimEndpointConfig &endpoint,
        const digital_twin::RobotSimSceneConfig &scene);

    /**
     * @brief 断开 RobotSim，并清除待处理的关节同步和虚拟参考请求。
     *
     * 运行时线程保持启动，用户可继续使用常驻端口输入重新选择连接模式。
     */
    void resetRobotSimConnection();

    /**
     * @brief 启动在线跟踪检测和在线纠偏初始检测阶段。
     */
    void startOnlineTracking();

    /**
     * @brief 以在线专用事务启动 Vision、协议握手和控制柜 JBI。
     * @param configuration 本轮在线轨迹、标定和模式配置。
     * @param jobName 控制柜中已有的在线纠偏作业名称。
     *
     * 本入口只服务在线纠偏：先等待新鲜相机帧，再确认 Vision、RobotSim 和
     * tracker/laser N-D 事务，最后发送 J 并观察 sStart、JBI 行号和 traflag。
     * 离线纠偏仍使用 startOfflineCorrectionJob()，不经过本状态机。
     */
    void startOnlineCorrectionJob(
        const RuntimeOnlineCorrectionConfiguration &configuration,
        const std::string &jobName);

    /// @brief 取消尚未完成的在线启动事务，不影响离线纠偏会话。
    void cancelOnlineCorrectionStart();

    /**
     * @brief 启动仅供全局定位采点使用的视觉跟踪。
     *
     * 该模式只向 UI 和 RobotComm 转发检测结果，不会喂给在线/离线纠偏控制器。
     */
    void startGlobalLocalizationTracking();

    /**
     * @brief 使用显式先验来源启动本轮全局定位视觉跟踪。
     *
     * 该重载只影响当前跟踪会话，不修改单独定位页面保存的默认来源。
     */
    void startGlobalLocalizationTracking(
        const GlobalLocalizationTrackingSources& sources);

    /**
     * @brief 停止全局定位采点跟踪。
     */
    void stopGlobalLocalizationTracking();

    /**
     * @brief 停止接收新全局定位帧，并在已绑定状态的在途帧处理完成后发出 drained。
     */
    void finishGlobalLocalizationTracking();

    /**
     * @brief 在专用定位线程执行全局点云生成、配准、RMSE和结果保存。
     *
     * requestId 由 UI 流程生成，用于停止或重启后丢弃过期结果。
     */
    void applyStandaloneGlobalLocalization(
        quint64 requestId,
        StandaloneGlobalLocalizationRequest request);

    /**
     * @brief 停止在线跟踪检测。
     */
    void stopOnlineTracking();

    /**
     * @brief 通知当前在线分段结束。
     */
    void finishOnlineSegment();

    /**
     * @brief 按旧按钮二态开始离线检测。
     */
    void startOfflineCorrectionDetection();

    /**
     * @brief 完成 B040 预检和 Vision 就绪确认后启动离线纠偏 JBI。
     * @param jobName 控制柜中已有的离线纠偏作业名称。
     *
     * 本入口保证离线 worker 与 Vision 均已准备完成后才投递 J 命令，避免
     * 快速 JBI 在检测会话建立前越过 B040 或首个焊缝帧。
     */
    void startOfflineCorrectionJob(const std::string &jobName);

    /**
     * @brief 结束离线检测、执行 ICP 并发送 O 示教点命令。
     */
    void finishOfflineCorrectionDetectionAndCompute();

    /**
     * @brief 重置离线纠偏状态并停止视觉跟踪。
     */
    void resetOfflineCorrection();

    /**
     * @brief 配置示教轨迹临时记录目录。
     */
    void configureTeachTrajectoryTempDirectory(const QString &tempDirectory);

    /**
     * @brief 清空临时示教轨迹并开始监听 s_Bsavetraj。
     */
    void startTeachTrajectoryRecording();

    /**
     * @brief 将临时示教轨迹保存为 `<baseName>_W/_T.txt`。
     */
    void saveTeachTrajectory(
        const QString &outputDirectory,
        const QString &baseName);

    /**
     * @brief 重置示教轨迹记录器。
     */
    void resetTeachTrajectoryRecording();

signals:
    void frameReady(const camera::CameraFrame &frame);
    void trackingResultReady(const RuntimeTrackingResult &result);
    /**
     * @brief 发布在线/离线共用的逐帧同步状态，不复用全局定位专属信号。
     */
    void correctionSyncUpdated(const CorrectionSyncUpdate &update);
    void onlineCorrectionUpdated(const control::OnlineCorrectionUpdate &update);
    void offlineCorrectionUpdated(const control::OfflineCorrectionUpdate &update);
    /// @brief 发布当前发往旧协议的 OfflineStartCount，供离线状态面板显示。
    void offlineCorrectionHandshakeChanged(int offlineStartCount);
    void teachTrajectoryRecorderUpdated(
        const control::TeachTrajectoryRecorderSnapshot &snapshot);
    void robotStateReady(const robot_comm::RobotState &state);
    void robotDiagnosticsReady(
        const robot_comm::RobotCommunicationDiagnostics &diagnostics);
    void statusMessage(const QString &message);
    void globalLocateVirtualPriorReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void globalLocateFrameVirtualPriorReady(
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void robotSimRealtimeVirtualPriorReady(
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void robotSimRealtimeVirtualReferenceReady(
        const digital_twin::DigitalTwinVirtualReferenceResult &reference);
    void robotSimConnectionStateChanged(
        bool connected,
        const QString &message);
    void robotSimManualConnectionStateChanged(
        bool connected,
        const QString &message);
    void robotSimManualVirtualReferenceReady(
        const digital_twin::DigitalTwinVirtualReferenceResult &reference);
    void robotSimStateSynchronizationChanged(
        bool succeeded,
        const QString &message);
    void globalLocalizationTrackingDrained();
    void standaloneGlobalLocalizationReady(
        quint64 requestId,
        const StandaloneGlobalLocalizationResult &result);
    /// @brief 发布在线启动事务的可展示阶段说明。
    void onlineCorrectionStartProgress(const QString &message);
    /// @brief sStart、JBI 行号和 traflag 均已确认，在线控制器可以开始检测。
    void onlineCorrectionStartSucceeded();
    /// @brief 在线启动事务终止；message 是可直接展示的现场诊断原因。
    void onlineCorrectionStartFailed(const QString &message);

private:
    /// @brief 按值组装在线 Vision 参数；仅覆盖在线会话标定，不改其他跟踪用途。
    RuntimeTrackingOptions onlineTrackingOptions() const;
    enum class OnlineCorrectionStartPhase {
        Idle,
        WaitingForCameraFrame,
        WaitingForConfiguration,
        WaitingForVision,
        WaitingForProtocol,
        WaitingForJobReceipt,
        WaitingForJobMotion,
        WaitingForDetectionWindow
    };

    /**
     * @brief 相机帧进入 runtime 边界时形成的不可拆分按值快照。
     *
     * current 和 latest-pending 都保存完整结构，禁止只替换图像而沿用旧
     * RobotState、purpose 或 session。requestId 为 0 表示尚未派发 RobotSim。
     */
    struct FrozenVisionFrameContext {
        camera::CameraFrame frame;
        robot_comm::RobotState robotState;
        bool hasRobotState = false;
        qint64 robotStateReceivedMs = -1;
        RuntimeTrackingPurpose purpose = RuntimeTrackingPurpose::None;
        quint64 correctionSessionId = 0;
        /// @brief RobotSim 查询派发时的模型代次，隔离写回前后的局部表面。
        quint64 modelGeneration = 0;
        int localizationSegmentIndex = -1;
        quint64 requestId = 0;
        std::optional<digital_twin::DigitalTwinVisionLaserObservation>
            synchronizedObservation;
    };

    void registerMetaTypes();
    void connectWorkers();
    void handleCameraFrame(const camera::CameraFrame &frame);
    void handleRobotState(const robot_comm::RobotState &state);
    void beginOnlineCorrectionArm();
    void handleOnlineConfigurationReady(
        quint64 sessionId,
        bool succeeded);
    void handleOnlineVisionArmed(quint64 sessionId);
    void handleOnlineProtocolReady(quint64 sessionId, bool succeeded);
    void handleOnlineJobReceipt(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &state);
    void processOnlineCorrectionStartState(
        const robot_comm::RobotState &state);
    void armOnlineCorrectionStartTimeout(
        OnlineCorrectionStartPhase phase,
        int timeoutMs,
        const QString &message);
    void failOnlineCorrectionStart(const QString &message);
    void clearOnlineCorrectionStart();
    /**
     * @brief 判断普通 RobotSim 实时同步是否可运行。
     *
     * 正式纠偏会话独占逐帧同步通道，避免普通实时状态覆盖同帧参考。
     * 会话完成、失败或重置后，恢复普通实时同步。
     */
    [[nodiscard]] bool canRunRealtimeRobotSimSynchronization() const;
    void dispatchPendingRobotSimStateSynchronization();
    void handleRobotSimConnectionStateChanged(
        bool connected,
        const QString &message);
    void handleRobotSimStateSynchronized(
        quint64 requestId,
        bool succeeded,
        const QString &message);
    void handleRobotSimRealtimeVirtualPriorReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void handleRobotSimRealtimeVirtualReferenceReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVirtualReferenceResult &reference);
    void enqueueVisionFrame(const camera::CameraFrame &frame);
    void dispatchPendingVisionFrame();
    void resetVisionFrameDispatch();
    void handleOfflineVisionArmed(quint64 sessionId);
    /// @brief 回到 coordinator 线程确认激光事务；过期会话不得启动 JBI。
    void handleOfflineLaserReady(quint64 sessionId, bool succeeded);
    /// @brief 本轮进入扫描后关闭入帧和迟到结果，仅停止 KCF，不触发 ICP/O。
    void updateOfflineSeamWindow(const robot_comm::RobotState &state);
    void beginCorrectionSession(CorrectionPurpose purpose);
    void setCorrectionVisionTrackingActive(bool active);
    void invalidateCorrectionSession(
        const QString &message,
        bool resumeRealtimeSynchronization = true);
    void resumeRealtimeRobotSimSynchronization();
    void emitCorrectionSyncState(
        CorrectionSyncState state,
        qint64 frameId,
        const QString &message,
        const digital_twin::DigitalTwinVisionLaserObservation *observation = nullptr);
    /**
     * @brief 处理在线/离线共用的逐帧同步失败和固定三次停止策略。
     *
     * 调用方必须已确认 current 属于活动 correction session。函数先释放
     * current，再累计失败；达到阈值时先失效 session，随后才通知对应业务
     * worker 进入终态，确保任何迟到 RobotSim/Vision 结果不能进入 Control。
     */
    void failCorrectionFrameSynchronization(
        qint64 frameId,
        const QString &message);
    RuntimeVisionCorrelation makeVisionCorrelation(
        const FrozenVisionFrameContext &context) const;
    bool currentFrameMatches(
        const RuntimeVisionCorrelation &correlation) const;
    void completeGlobalLocalizationTrackingDrain();
    void updateGlobalLocalizationSegmentState(
        const robot_comm::RobotState &state);
    void handleRobotSimVirtualPriorReady(
        quint64 requestId,
        const digital_twin::DigitalTwinVisionLaserObservation &observation);
    void routeTrackingResult(const RuntimeTrackingResult &result);
    /// @brief 启动独立局部定位线程，读取有界采样/验收配置。
    void startLocalRegistrationWorker();
    /// @brief 撤销旧窗口、候选修正及网络写回代次；不停止机器人运动。
    void resetOnlineLocalRegistration();
    /// @brief 从已完成的同帧上下文低频投递表面采样，繁忙时跳过且不排队。
    void submitOnlineLocalRegistration(const FrozenVisionFrameContext &context);
    void handleOnlineLocalRegistrationResult(
        quint64 requestId,
        const COnlineLocalRegistrationWorker::Result &result);
    /// @brief 仅在主视觉和 RobotSim 查询空隙提交候选修正；写回期间保留最新相机帧。
    bool dispatchOnlineLocalRegistrationWriteback();
    void shutdownThread(QThread &thread);

    QThread m_localRegistrationThread;
    COnlineLocalRegistrationWorker *m_localRegistrationWorker = nullptr;
    COnlineLocalRegistrationWorker::Options m_localRegistrationOptions;
    /// @brief 启动参数无效时禁止通过界面开关绕过停用保护。
    bool m_localRegistrationConfigurationValid = true;
    std::optional<COnlineLocalRegistrationWorker::Result> m_pendingLocalCorrection;
    QElapsedTimer m_localRegistrationClock;
    qint64 m_lastLocalRegistrationSampleMs = -1;
    /// @brief 跨模型写回代次保留的配准冷却，防止清窗重置 worker 时钟后过频求解。
    qint64 m_lastLocalRegistrationAttemptMs = -1;
    qint64 m_localRegistrationInputMs = -1;
    quint64 m_localRegistrationGeneration = 0;
    quint64 m_localRegistrationRequestId = 0;
    bool m_localRegistrationBusy = false;
    bool m_localRegistrationWritebackBusy = false;
    /// @brief 配准异常或写回未确认时停用本轮自动更新，避免重复叠加不确定结果。
    bool m_localRegistrationFaulted = false;
    bool m_localRegistrationRobotStateFresh = false;

    QThread m_cameraThread;
    QThread m_robotThread;
    QThread m_visionThread;
    QThread m_globalLocatePriorThread;
    QThread m_globalLocalizationThread;
    QThread m_robotSimThread;
    QThread m_onlineCorrectionThread;
    QThread m_offlineCorrectionThread;
    QThread m_teachTrajectoryThread;

    CCameraWorkerAdapter *m_cameraWorker = nullptr;
    CRobotCommWorkerAdapter *m_robotWorker = nullptr;
    CVisionTrackingWorkerAdapter *m_visionWorker = nullptr;
    CGlobalLocatePriorWorkerAdapter *m_globalLocatePriorWorker = nullptr;
    CGlobalLocalizationWorkerAdapter *m_globalLocalizationWorker = nullptr;
    CGlobalLocatePriorWorkerAdapter *m_robotSimWorker = nullptr;
    COnlineCorrectionWorkerAdapter *m_onlineCorrectionWorker = nullptr;
    COfflineCorrectionWorkerAdapter *m_offlineCorrectionWorker = nullptr;
    CTeachTrajectoryRecorderWorkerAdapter *m_teachTrajectoryWorker = nullptr;
    SimulatedVisionCallbacks m_callbacks;
    std::unique_ptr<camera::ICameraDevice> m_cameraDevice;
    int m_cameraCaptureIntervalMs = 33;
    std::unique_ptr<robot_comm::IRobotCommandTransport> m_robotTransport;
    std::unique_ptr<digital_twin::IRobotSimTransport> m_robotSimTransport;
    qint64 m_latestRobotStateReceivedMs = -1;
    robot_comm::RobotState m_latestRobotState;
    bool m_hasLatestRobotState = false;
    bool m_hasLatestCameraFrame = false;
    qint64 m_lastCameraFrameReceivedMs = 0;
    std::optional<FrozenVisionFrameContext> m_pendingVisionFrame;
    std::optional<FrozenVisionFrameContext> m_currentVisionFrame;
    bool m_visionFrameInFlight = false;
    bool m_globalLocalizationDrainRequested = false;
    quint64 m_robotSimPriorNextRequestId = 0;
    quint64 m_robotSimPriorInFlightRequestId = 0;
    robot_comm::RobotState m_pendingRobotSimSyncState;
    bool m_hasPendingRobotSimSyncState = false;
    bool m_robotSimConnected = false;
    bool m_robotSimManualModeRequested = false;
    bool m_robotSimConnectionActiveOrPending = false;
    bool m_robotSimHasSynchronizationStatus = false;
    bool m_robotSimLastSynchronizationSucceeded = false;
    QString m_robotSimLastSynchronizationMessage;
    quint64 m_robotSimSyncNextRequestId = 0;
    quint64 m_robotSimSyncInFlightRequestId = 0;
    int m_globalLocalizationLastPosflag = 0;
    int m_globalLocalizationCurrentSegmentIndex = 0;
    int m_globalLocalizationSegmentSequence = 0;
    RuntimeTrackingOptions m_trackingOptions;
    /// @brief 与在线 Control 配置同源的标定快照，包含无效配置的拒绝语义。
    std::optional<vision::PositionerCalibration> m_onlinePositionerCalibration;
    vision::WeldProjectionContext m_trackingProjectionContext;
    RuntimeTrackingPurpose m_trackingPurpose = RuntimeTrackingPurpose::None;
    CorrectionRunSession m_correctionSession;
    OnlineCorrectionStartPhase m_onlineCorrectionStartPhase =
        OnlineCorrectionStartPhase::Idle;
    RuntimeOnlineCorrectionConfiguration
        m_pendingOnlineCorrectionConfiguration;
    std::string m_pendingOnlineCorrectionJobName;
    quint64 m_onlineCorrectionStartSessionId = 0;
    quint64 m_onlineCorrectionStartGeneration = 0;
    int m_onlineCorrectionStartBaselineJobLine = 0;
    std::string m_pendingOfflineCorrectionJobName;
    /// @brief 本轮已观察到 sStart=1、traflag=1；初始 traflag=0 不算扫描结束。
    bool m_offlineSeamWindowEntered = false;
    quint64 m_nextCorrectionSessionId = 0;
    GlobalLocateVirtualLaserSource m_globalLocateVirtualLaserSource =
        GlobalLocateVirtualLaserSource::Occt;
    GlobalLocateVirtualPointCloudSource m_globalLocateVirtualPointCloudSource =
        GlobalLocateVirtualPointCloudSource::Weldment;
    GlobalLocalizationTrackingSources m_activeGlobalLocalizationSources;
    digital_twin::RobotSimEndpointConfig m_globalLocateRobotSimEndpoint;
    digital_twin::RobotSimSceneConfig m_globalLocateRobotSimScene;
    bool m_running = false;
    bool m_stopping = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_RUNTIME_COORDINATOR_H
