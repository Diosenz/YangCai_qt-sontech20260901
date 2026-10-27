/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        LegacyOnlineKcfDetector.h
 * @brief       旧在线 KCF 跟踪服务
 * @details     声明面向旧喷嘴跟踪流程的 KCF 在线检测封装。
 ******************************************************************************/
#ifndef I2R_RLC_VISION_TRACK_LEGACY_ONLINE_KCF_DETECTOR_H
#define I2R_RLC_VISION_TRACK_LEGACY_ONLINE_KCF_DETECTOR_H

#include <array>
#include <memory>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "DigitalTwinVisionTypes.h"
#include "feature/OnlineSeamRefiner.h"
#include "feature/PointCloudGapWidthDetector.h"
#include "feature/SimulationGuidedSeamEstimator.h"
#include "geometry/WeldGeometryTypes.h"
#include "track/KcfTracker.h"
#include "track/OnlineSeamContinuity.h"
#include "track/OnlineSeamEvidence.h"

namespace i2r_rlc::vision {

/**
 * @brief 旧在线 KCF 与 gap 检测参数。
 */
struct LegacyOnlineKcfOptions {
    KcfFusionOptions fusion;
    /// @brief 正式在线采用仿真模板初始化和当前图像相关观测；几何仅补充独立资格。
    bool enablePaperTracking = false;
    /// @brief 正式在线与生产策略回放共用；legacy 入口保持既有行为。
    bool enableEvidencePolicy = false;
    COnlineSeamEvidenceOptions evidenceOptions;
    COnlineSeamEvidenceContext evidenceContext;
    /// @brief 在线纠偏显式启用；仿真仅提示搜索，输出必须由同帧原图验收。
    bool enableImageMeasurementOutput = false;
    /// @brief 仅正式在线显式启用：原图重捕获分帧执行，默认保留初扫完整搜索。
    bool enableBoundedImageSearch = false;
    /// @brief 在线纠偏显式启用原图精定位；离线/legacy 回放保持既有行为。
    bool enableSeamRefinement = false;
    OnlineSeamRefinementOptions seamRefinement;
    /// @brief 在线运行时显式启用；辅助结果永不成为独立原图焊点。
    bool enableSimulationGuidance = false;
    /// @brief 仅正式在线纠偏启用原图质量降权与连续新观测重捕获；其他入口保持旧策略。
    bool enableOnlineObservationFusion = false;
    /// @brief 实际工件写回/回滚版本；不是每次定位采样的任务代次。
    std::uint64_t referenceRevision = 0;
    SimulationGuidedSeamOptions guidedOptions;
    /// @brief 同会话位姿切换的业务估计种子，不可作为独立原图实测。
    bool hasReferenceTransitionPoint = false;
    cv::Point2d referenceTransitionPoint{-1.0, -1.0};
    bool enableGapDetection = false;
    PointCloudGapWidthOptions gapWidthOptions;
};

/**
 * @brief 检测器阶段快照，随本帧结果按值传递，不拥有跟踪器状态。
 */
struct CLegacyOnlineKcfDiagnostic {
    /// @brief 原图搜索的曝光身份；重复/未知曝光不累计候选确认。
    std::int64_t exposureFrameId = -1;
    COnlineSeamEvidenceResult evidence;
    bool imageMeasurementOutputEnabled = false;
    bool imageObservationAccepted = false;
    bool imageSearchExpanded = false;
    bool boundedImageSearchEnabled = false;
    /// @brief 本次调用实际执行的原图 refiner 总次数，含局部与扩大搜索。
    int imageRefinementCallCount = 0;
    /// @brief 有界搜索实际检查的中心和几何状态；每帧最多两项，只读诊断。
    std::string imageSearchDiagnostic;
    bool imageSearchCyclePending = false;
    bool imageSearchCycleCompleted = false;
    /// @brief 当前原图搜索提案生成总耗时，单位 ms；不包含 refiner。
    double imageSearchProposalGenerationMs = 0.0;
    /// @brief 同帧 CAD 点实际存在且有限；没有参考时禁止以默认零坐标记录偏差。
    bool simulatedPointAvailable = false;
    /// @brief 仿真绝对位置可在原图取样；不代表位置与实际工件一致。
    bool simulationPositionInImage = false;
    /// @brief 局部仿真外观可单独使用，绝对位置失效不会自动清除此资格。
    bool simulationShapeAvailable = false;
    /// @brief 外观按已接纳图像点/初始化搜索锚点重定位；不是工件位姿校正。
    bool simulationTemplateRebased = false;
    cv::Point2d simulationTemplatePoint;
    /// @brief 本曝光的仿真位置由上一可靠新曝光的局部偏差支持，不代表绝对坐标准确。
    bool simulationPositionTrusted = false;
    /// @brief 已执行可信仿真先验与当前图像观测的二维 Kalman 更新。
    bool positionFusionApplied = false;
    cv::Point2d positionPriorPoint;
    cv::Point2d positionObservationPoint;
    cv::Point2d simulationLocalBias;
    double positionGainX = 0.0;
    double positionGainY = 0.0;
    bool onlineObservationFusionEnabled = false;
    bool attempted = false;
    bool templateReinitialized = false;
    bool gaborSucceeded = false;
    int framesSinceInitialize = 0;
    cv::Size imageSize;
    cv::Rect enhancementRoi;
    cv::Point2d templateAnchor;
    cv::Point2d simulatedPoint;
    cv::Point2d responsePoint;
    cv::Point2d beforeFusionPoint;
    /// @brief 仅供粗跟踪诊断；不作为精定位启用后的业务输出。
    cv::Point2d coarseFinalPoint;
    bool seamRefinementEnabled = false;
    bool refinementAttempted = false;
    /// @brief 时序门控前的原图候选；仅诊断，不能直接作为独立验收点。
    OnlineSeamRefinementResult rawRefinement;
    OnlineSeamRefinementResult refinement;
    /// @brief 原图局部窄峰及相关响应的质量分数，非标定概率。
    double coarseObservationQuality = 0.0;
    bool coarseObservationUsed = false;
    /// @brief 正式在线以真实图像历史重新搜索；不借用工件模型位姿作为运动。
    bool realSearchFallbackUsed = false;
    double refinementElapsedMs = 0.0;
    OnlineSeamContinuityResult continuity;
    SimulationGuidedSeamResult guided;
    /// @brief 实际业务输出像素；启用精定位时为精定位后按既有比例融合的点。
    cv::Point2d finalPoint{-1.0, -1.0};
    /// @brief 反向初扫可信图像响应的统计；仅控制模板更新，不等同失测判据。
    int normalResponseSamples = 0;
    double normalResponseMean = 0.0;
    double normalResponseStd = 0.0;
    bool responseStatisticsReady = false;
    bool responseNormal = true;
    /// @brief 中心遮挡时的当前原图双肩部匹配，仅为位置约束，不授予独立几何资格。
    bool shoulderObservationAccepted = false;
    bool shoulderEstimateUsed = false;
    double shoulderObservationQuality = 0.0;
    double centerObservationQuality = 0.0;
    /// @brief 实拍训练要求当前完整外观得到支持；统计不足时不把未校准PSR当正常证明。
    bool completeAppearanceSupported = false;
    CKcfUpdateDiagnostic tracker;
};

/** @brief 当帧定位支持与恢复结论；独立于可选诊断和几何资格。 */
struct OnlineSeamLocalizationEvidence {
    /// @brief 新曝光缺原图或有效空间投影；保留实际图像尝试，不冒充搜索失败。
    bool inputUnavailable = false;
    /// @brief 本曝光已实际执行目标定位；缺图或过期曝光不冒充失败搜索。
    bool attempted = false;
    bool currentImageSupported = false;
    bool imageConstrainedEstimate = false;
    /// @brief 当前图像相关/局部身份恢复已尝试，不声明全图搜索穷尽。
    bool recoveryAttempted = false;
    bool recoveryFailed = false;
};

/** @brief 单曝光生产定位输出；定位支持在关闭可选诊断时仍有效。 */
struct LegacyOnlineKcfResult {
    bool valid = false;
    OnlineSeamLocalizationEvidence localizationEvidence;
    CLegacyOnlineKcfDiagnostic diagnostic;
    /// @brief 同帧业务像素，单位 pixel；退化时由显式来源标记区分粗跟踪和仿真辅助估计。
    cv::Point2d weldPointPixel{-1.0, -1.0};
    WeldSpacePoint stationPoint;
    double gapWidthMm = 0.0;
    bool gapValid = false;
    double gapConfidence = 0.0;
    cv::Mat processedImage;
    cv::Rect trackedRoi;
    std::string message;
};

/**
 * @brief 旧 OpencvThread 活动路径中的标准 KCF/Gabor 在线检测器。
 *
 * 类内部持有 KCF 跨帧状态；runtime 与离线 replay 必须复用此实现。
 */
class CLegacyOnlineKcfDetector final {
public:
    CLegacyOnlineKcfDetector();

    void reset();

    /**
     * @brief 重建曝光/位置跟踪状态，保留焊前响应分布及其图像尺寸、焊缝来源。
     * @details 仅用于同一纠偏会话的阶段切换；调用方负责核对会话和参考版本。
     * 新会话必须 reset，后续原图尺寸或焊缝变化仍会撤销保留的统计。
     */
    void restartTracking();

    /** @brief 缺图/门禁断开连续重捕获与待训练帧；保留此前原图模板供后续搜索。 */
    void notifyImageObservationGap();

    LegacyOnlineKcfResult process(
        const cv::Mat &gray8,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const WeldProjectionContext &projectionContext,
        const LegacyOnlineKcfOptions &options);

private:
    /** @brief 用当前原图重建目标模型和取样锚点；只在视觉线程调用，不继承旧运动量。 */
    void initializePaperTracker(const cv::Mat &gray8, const cv::Point2d &point);
    /**
     * @brief 在当前曝光图像上跟踪接缝，不要求每帧具备凸折点双侧拟合。
     * @details 当前实拍建立目标外观，仿真只提供辅助模板；输出须通过当帧相关及条纹身份校核。
     * 仅视觉工作线程串行调用，更新模板和曝光历史；跟踪不刷新独立几何资格。
     */
    LegacyOnlineKcfResult processPaperTracking(
        const cv::Mat &gray8,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const WeldProjectionContext &projectionContext,
        const LegacyOnlineKcfOptions &options);
    /** @brief 原图测量策略；不使用 CAD 模板、坐标融合或失测补点。 */
    LegacyOnlineKcfResult processImageMeasurement(
        const cv::Mat &gray8,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const WeldProjectionContext &projectionContext,
        const LegacyOnlineKcfOptions &options);

    /**
     * @brief 正式在线分帧搜索；缓存仅保存提案，输出始终重新测量当前原图。
     * @details 每帧合计最多两次 refiner；可信 KCF 期间仍轮转扩大搜索及旧目标局部搜索。
     */
    OnlineSeamRefinementResult searchBoundedImageGeometry(
        const cv::Mat &gray8, const std::vector<cv::Point2d> &hints,
        const LegacyOnlineKcfOptions &options,
        CLegacyOnlineKcfDiagnostic &diagnostic, bool coarseAccepted);

    /**
     * @brief 将上一可靠曝光支持的局部仿真先验与当前观测融合；不授予图像或几何资格。
     * @details 在视觉线程调用，参考缺失/跳变/版本切换即废弃偏差；不更新工件位姿。
     */
    cv::Point2d fusePaperPosition(const cv::Point2d &observedPoint,
        bool reliableObservation, const LegacyOnlineKcfOptions &options,
        CLegacyOnlineKcfDiagnostic &diagnostic);

    /// @brief 仅供当前焊缝局部相邻曝光使用；不能当作永久虚实标定。
    struct PaperPositionState {
        std::unique_ptr<CFilterKalman> filter;
        cv::Point2d localBias;
        std::int64_t frameId = -1;
        std::uint64_t referenceRevision = 0;
        int seamIndex = -1;
        bool hasBias = false;
    };
    PaperPositionState m_paperPosition;
    std::unique_ptr<CKcfTracker> m_tracker;
    COnlineSeamContinuity m_continuity;
    COnlineSeamEvidence m_evidence;
    CSimulationGuidedSeamEstimator m_guidedEstimator;
    std::uint64_t m_guidanceReferenceRevision = 0;
    bool m_guidanceRevisionKnown = false;
    int m_continuitySeamIndex = -1;
    /// @brief 模板内焊缝位置除以模板宽高；重建模板时更新，跟踪期间保持不变。
    cv::Point2d m_templateWeldAnchor;
    /// @brief 模板所属原图尺寸；分辨率变化时清除旧坐标和滤波状态。
    cv::Size m_templateImageSize;
    /** @brief 不同模板数的响应分布分别校准，防止模型切换混合PSR尺度。 */
    struct PaperResponseStatistics {
        int samples = 0;
        double mean = 0.0;
        double secondMoment = 0.0;
    };
    std::array<PaperResponseStatistics, 4> m_paperResponseStatistics{};
    bool m_initialized = false;
    int m_framesSinceInitialize = 0;
    bool m_onlineObservationActive = false;
    /// @brief 最近图像约束位置与同来源位移，只用于局部搜索及候选消歧，不提供独立几何。
    cv::Point2d m_paperImagePoint{-1.0, -1.0};
    cv::Point2d m_paperImageVelocity;
    bool m_hasPaperImageMotion = false;
    /// @brief 运动差分两端必须属于同一种定位来源，几何校正不能充当目标运动。
    bool m_paperMotionHasGeometry = false;
    bool m_hasImageSearchPoint = false;
    cv::Point2d m_lastImageSearchPoint{-1.0, -1.0};
    /// @brief 新原图支系已确认而粗 KCF 尚未同位验收；期间保留检测但不融合旧粗点。
    bool m_coarseRecoveryPending = false;
    bool m_imageMeasurementActive = false;
    /// @brief 图像身份模板；论文路径可用初始化形状和完整实拍更新，旧测量路径仅由几何更新。
    cv::Mat m_imageIdentityPatch;
    /// @brief 远处原图身份重捕获候选；失测中断确认，不产生业务补点。
    cv::Point2d m_imageReacquisitionPoint{-1.0, -1.0};
    int m_imageReacquisitionFrames = 0;
    qint64 m_imageLastObservationFrameId = -1;
    struct BoundedImageSearchState {
        std::vector<cv::Point2d> proposals;
        std::size_t cursor = 0;
        int generationStage = 0;
        bool hasCandidateHint = false;
        bool candidateReady = false;
        bool candidateTurn = false;
        bool localPairTurn = false;
        int candidateConfirmations = 0;
        std::int64_t candidateFrameId = -1;
        cv::Point2d candidateHint{-1.0, -1.0};
    };
    BoundedImageSearchState m_boundedImageSearch;
    bool m_boundedImageSearchAmbiguous = false;
};

} // namespace i2r_rlc::vision

#endif
