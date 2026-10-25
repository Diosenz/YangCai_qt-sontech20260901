#include "RuntimeWorkerAdapters.h"
namespace i2r_rlc::ui {
class CVisionTrackingWorkerAdapter final : public QObject {
    Q_OBJECT

public:
    explicit CVisionTrackingWorkerAdapter(QObject *parent = nullptr);

    void setSimulatedCallbacks(const SimulatedVisionCallbacks &callbacks);

public slots:
    void configure(
        const RuntimeTrackingOptions &options,
        const vision::WeldProjectionContext &projectionContext);
    /// @brief 视觉线程热更新在线融合选项，保留当前 KCF 模板。
    void updateOnlineFusionOptions(const vision::KcfFusionOptions &options);
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
    COnlineVisionImageSampling m_diagnosticImageSampling;
    quint64 m_diagnosticImageSamplingSessionId = 0;
    /// @brief 仅诊断节流使用的本机单调时间；不参与帧/会话关联身份。
    QElapsedTimer m_diagnosticImageSamplingClock;
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
}
