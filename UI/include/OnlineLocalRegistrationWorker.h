/** @file OnlineLocalRegistrationWorker.h
 * @brief 在线局部表面点云定位的独立线程边界。
 */
#ifndef I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_WORKER_H
#define I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_WORKER_H

#include <deque>
#include <optional>
#include <vector>

#include <QElapsedTimer>
#include <QObject>
#include <QJsonObject>

#include "LocalizationLocalCorrectionPipeline.h"
#include "LocalizationService.h"
#include "LocalizationSurfaceValidation.h"
#include "OnlineLocalRegistrationStability.h"
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
        /// @brief 已有可信位姿后，允许用母材表面与留出验收跨越缺焊点段。
        bool surfaceAssistedUpdatesEnabled = true;
        /// @brief 可选焊点收益保护和历史监测；未经几何校核的历史不否决候选。
        bool stabilityProtectionEnabled = false;
        int sampleIntervalMs = 200;
        int minimumMatchIntervalMs = 2000;
        int minimumFrames = 8;
        int maximumFrames = 16;
        int maximumPointsPerCloud = 512;
        int maximumIterations = 40;
        /// @brief ICP 最近邻对应点的最大距离，单位 mm，范围 [0.1,10]。
        double maximumCorrespondenceDistanceMm = 10.0;
        /// @brief 求解后超预算拒绝，不是对 PCL 的硬中断。
        int maximumSolveTimeMs = 2500;
        /// @brief 单次修正的三轴合成平移上限，mm，范围 [0.01,30]；超过则拒绝。
        double maximumTranslationMm = 30.0;
        double maximumRotationDeg = 10.0;
        double minimumImprovementRatio = 0.05;
        /// @brief 全会话统一的平均最近邻残差上限，mm；仍须满足最低改善比例。
        double maximumResidualMm = 5.5;
        /// @brief 独立焊点仅作否决观测，并非真实值；误差或恶化超限均拒绝。
        double maximumIndependentErrorMm = 5.5;
        double maximumIndependentWorseningMm = 0.1;
        /// @brief 双向匹配覆盖及匹配区域分布门禁，不代表全部自由度可观测。
        double coverageDistanceMm = 3.0;
        double minimumBidirectionalCoverage = 0.7;
        double minimumMatchedSpreadRatio = 0.5;
        /// @brief 归档预算跨代次保留；达到上限暂停更新，禁止删除旧归档腾出空间。
        int maximumArchivedAttemptsPerSession = 128;
        int maximumRawPointsPerFrame = 32768;
    };

    /** @brief 同帧独立特征；缺失时仅已有可信位姿的母材表面支路可申请更新。 */
    struct IndependentObservation {
        bool valid = false;
        qint64 frameId = -1;
        localization::Point3D_mm realWeldPointInParent_mm;
        localization::Point3D_mm virtualWeldPointInParent_mm;
    };

    /** @brief 离线验证也可复用生产验收；旧五参数入口仅保留合成点云兼容。 */
    struct CloudValidationContext {
        IndependentObservation independentObservation;
        QString archiveDirectory;
        /// @brief 当前位姿下的历史近似对应快照，仅作未校核几何监测。
        std::vector<COnlineLocalRegistrationStability::Anchor> stabilityAnchors;
        bool requireIndependentObservation = true;
        /// @brief 表面验收通道仍需已有后验确认位姿；不用于绕过初始化。
        bool hasVerifiedWorkpiecePose = false;
        quint64 workpiecePoseRevision = 0;
        QImage sourceImage;
        /// @brief 同帧身份、姿态和采集来源；缺失项保持未知，不伪造为有效几何。
        QJsonObject evidenceContext;
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
        IndependentObservation independentObservation;
        /// @brief 本轮运行的局部配准归档目录；必须非空且可写，I/O 仅在 worker 执行。
        QString archiveDirectory;
        std::vector<COnlineLocalRegistrationStability::Anchor> stabilityAnchors;
        int workpieceId = 0;
        int parentId = 0;
        QString sceneName;
        std::optional<localization::RigidTransform> lastVerifiedWorkpiecePose;
        quint64 workpiecePoseRevision = 0;
        /// @brief 仅用于排除局部预焊凸起邻域，不作为真实焊点证据。
        double seamImageXPixel = -1.0;
    };

    /** @brief 工件父级系下的小幅虚拟到真实修正；accepted 才允许提交场景写回。 */
    struct Result {
        quint64 sessionId = 0;
        quint64 generation = 0;
        qint64 frameId = -1;
        bool attempted = false;
        bool accepted = false;
        localization::RigidTransform correction;
        /// @brief 最新同帧独立观测，同时供写回关键点位移限幅使用。
        IndependentObservation independentObservation;
        /// @brief true表示表面验收/新帧后验通道，不要求本窗缺点，也不晋升估计点。
        bool surfaceOnly = false;
        quint64 workpiecePoseRevision = 0;
        localization::surface_validation::Result surfaceValidation;
        /// @brief 末两帧对冻结训练面的配对可支持性；只作写前准入，非新帧证明。
        localization::surface_validation::PairedSurfaceMetrics surfaceVerificationPreflight;
        /// @brief 表面候选投影前的PCL矩阵，供归档复核，不可直接写回。
        localization::RigidTransform originalSurfaceCorrection;
        std::vector<localization::Point3D_mm> safetyKeypointsInParent_mm;
        /// @brief 写回前训练虚拟云，冻结为新帧配对局部面的来源；不含留出或新实测。
        std::vector<localization::Point3D_mm> surfaceVerificationReference_mm;
        QString message;
        double beforeResidualMm = 0.0;
        double afterResidualMm = 0.0;
        qint64 elapsedMs = 0;
        qint64 solveElapsedMs = 0;
        int frameCount = 0;
        int pointCount = 0;
        double beforeIndependentErrorMm = 0.0;
        double afterIndependentErrorMm = 0.0;
        double realCoverageRatio = 0.0;
        double virtualCoverageRatio = 0.0;
        double matchedSpreadRatio = 0.0;
        bool baseSafetyAccepted = false;
        bool stabilityEvaluated = false;
        COnlineLocalRegistrationStability::Evaluation stabilityEvaluation;
        bool archiveSucceeded = false;
        QString archivePath;
    };

    /** @brief 创建不拥有外部设备的 worker。 */
    explicit COnlineLocalRegistrationWorker(QObject *parent);
    /** @brief 更新参数并清空历史窗口。 */
    void configure(const Options &options);
    /** @brief 提取真实表面线并进入有界窗口；不阻塞主视觉线程。 */
    Result process(const Input &input);
    /** @brief 后台线程提取同帧母材并测量；不累计窗口、不求解或写回场景。 */
    struct SurfaceFrameResult {
        bool valid = false;
        qint64 frameId = -1;
        quint64 workpiecePoseRevision = 0;
        std::vector<localization::Point3D_mm> realPoints_mm;
        std::vector<localization::Point3D_mm> virtualPoints_mm;
        localization::surface_validation::Metrics metrics;
        localization::surface_validation::PairedSurfaceMetrics comparison;
        QString message;
        QJsonObject evidenceContext;
    };
    /** @brief 仅在本worker所属线程调用，供写回后新帧表面后验复用。 */
    SurfaceFrameResult measureSurface(const Input &input) const;
    /** @brief 在线新帧绝对切片检查及当前CAD支持的有限局部面对照；参考与候选冻结。 */
    SurfaceFrameResult measureSurfaceForVerification(const Input &input,
        const std::vector<localization::Point3D_mm> &frozenVirtual,
        const localization::RigidTransform &correction) const;
    /**
     * @brief 累计同一工件父级参考系下的同帧点云，供生产提取和离线验证共用。
     * @details 生产输入均为变位机 TCP 系，不再叠加末节到 TCP 的偏移；
     * 不进行坐标猜测，窗口跨会话或模型代次自动失效。
     */
    Result processClouds(quint64 sessionId, quint64 generation, qint64 frameId,
        const std::vector<localization::Point3D_mm> &realPointsInParent_mm,
        const std::vector<localization::Point3D_mm> &virtualPointsInParent_mm);
    /** @brief 生产安全入口；对应通道的独立证据、双向覆盖及归档均通过才允许 accepted。 */
    Result processClouds(quint64 sessionId, quint64 generation, qint64 frameId,
        const std::vector<localization::Point3D_mm> &realPointsInParent_mm,
        const std::vector<localization::Point3D_mm> &virtualPointsInParent_mm,
        const CloudValidationContext &validation);
    /** @brief 清窗及节流时钟；正在执行的 PCL 调用只能在返回后由上层丢弃。 */
    void reset();

private:
    struct CloudFrame {
        std::vector<localization::Point3D_mm> real;
        std::vector<localization::Point3D_mm> reference;
        std::vector<localization::Point3D_mm> rawReal;
        std::vector<localization::Point3D_mm> rawReference;
        IndependentObservation independentObservation;
        qint64 frameId = -1;
        qint64 capturedMs = 0;
        QJsonObject evidenceContext;
        QImage sourceImage;
        bool hasVerifiedWorkpiecePose = false;
        quint64 workpiecePoseRevision = 0;
    };
    /** @brief 完整帧前段训练、末两帧留出；本函数不把预测焊点纳入求解。 */
    Result processSurfaceWindow(const std::deque<CloudFrame> &frames,
        const CloudValidationContext &validation, Result result);
    /** @brief 只按尝试落盘；写入失败或预算耗尽必须使候选失效。 */
    bool archiveAttempt(const std::deque<CloudFrame> &frames,
        const localization::LocalCorrectionPipelineRequest &request,
        const std::vector<localization::Point3D_mm> &preprocessedRealPoints,
        const CloudValidationContext &validation, Result &result);
    Options m_options;
    quint64 m_sessionId = 0;
    quint64 m_generation = 0;
    qint64 m_lastFrameId = -1;
    qint64 m_lastSampleMs = -1;
    qint64 m_lastMatchMs = -1;
    QElapsedTimer m_clock;
    std::deque<CloudFrame> m_frames;
    quint64 m_archiveSessionId = 0;
    int m_archiveAttemptCount = 0;
    localization::LocalizationService m_service;
    localization::LocalizationLocalCorrectionPipeline m_pipeline;
};

} // namespace i2r_rlc::ui
#endif // I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_WORKER_H
