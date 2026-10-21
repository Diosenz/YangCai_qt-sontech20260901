/** @file OnlineLocalRegistrationWorker.h
 * @brief 在线局部表面点云定位的独立线程边界。
 */
#ifndef I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_WORKER_H
#define I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_WORKER_H

#include <deque>
#include <vector>

#include <QElapsedTimer>
#include <QObject>

#include "LocalizationLocalCorrectionPipeline.h"
#include "LocalizationService.h"
#include "RuntimeTrackingTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief 有界采样、局部配准与验收；不访问设备或修改仿真场景。
 * @details 所有方法只能在所属 worker 线程串行调用。coordinator 只保留一个在途
 * 请求，并负责会话撤销、模型代次及写回。无 Q_OBJECT，通过排队 functor 调用。
 */
class COnlineLocalRegistrationWorker final : public QObject {
public:
    /** @brief 点数、频率及小幅修正验收参数；长度单位 mm，角度单位 degree。 */
    struct Options {
        bool enabled = true;
        int sampleIntervalMs = 200;
        int minimumMatchIntervalMs = 2000;
        int minimumFrames = 8;
        int maximumFrames = 16;
        int maximumPointsPerCloud = 512;
        int maximumIterations = 40;
        /// @brief 求解后超预算拒绝，不是对 PCL 的硬中断。
        int maximumSolveTimeMs = 300;
        double maximumTranslationMm = 2.0;
        double maximumRotationDeg = 1.0;
        double minimumImprovementRatio = 0.05;
        double maximumResidualMm = 2.0;
    };

    /** @brief 主纠偏链冻结的同帧输入；虚拟点以工件父级的变位机 TCP 系为参考。 */
    struct Input {
        quint64 sessionId = 0;
        quint64 generation = 0;
        camera::CameraFrame frame;
        robot_comm::RobotState robotState;
        digital_twin::DigitalTwinVisionLaserObservation observation;
        RuntimeTrackingOptions trackingOptions;
        vision::WeldProjectionContext projectionContext;
    };

    /** @brief 工件父级系下的小幅虚拟到真实修正；accepted 才允许提交场景写回。 */
    struct Result {
        quint64 sessionId = 0;
        quint64 generation = 0;
        qint64 frameId = -1;
        bool attempted = false;
        bool accepted = false;
        localization::RigidTransform correction;
        QString message;
        double beforeResidualMm = 0.0;
        double afterResidualMm = 0.0;
        qint64 elapsedMs = 0;
        qint64 solveElapsedMs = 0;
        int frameCount = 0;
        int pointCount = 0;
    };

    /** @brief 创建不拥有外部设备的 worker。 */
    explicit COnlineLocalRegistrationWorker(QObject *parent);
    /** @brief 更新参数并清空历史窗口。 */
    void configure(const Options &options);
    /** @brief 提取真实表面线并进入有界窗口；不阻塞主视觉线程。 */
    Result process(const Input &input);
    /**
     * @brief 累计同一工件父级参考系下的同帧点云，供生产提取和离线验证共用。
     * @details 生产输入均为变位机 TCP 系，不再叠加末节到 TCP 的偏移；
     * 不进行坐标猜测，窗口跨会话或模型代次自动失效。
     */
    Result processClouds(quint64 sessionId, quint64 generation, qint64 frameId,
        const std::vector<localization::Point3D_mm> &realPointsInParent_mm,
        const std::vector<localization::Point3D_mm> &virtualPointsInParent_mm);
    /** @brief 清窗及节流时钟；正在执行的 PCL 调用只能在返回后由上层丢弃。 */
    void reset();

private:
    struct CloudFrame {
        std::vector<localization::Point3D_mm> real;
        std::vector<localization::Point3D_mm> reference;
        qint64 capturedMs = 0;
    };
    Options m_options;
    quint64 m_sessionId = 0;
    quint64 m_generation = 0;
    qint64 m_lastFrameId = -1;
    qint64 m_lastSampleMs = -1;
    qint64 m_lastMatchMs = -1;
    QElapsedTimer m_clock;
    std::deque<CloudFrame> m_frames;
    localization::LocalizationService m_service;
    localization::LocalizationLocalCorrectionPipeline m_pipeline;
};

} // namespace i2r_rlc::ui
#endif // I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_WORKER_H
