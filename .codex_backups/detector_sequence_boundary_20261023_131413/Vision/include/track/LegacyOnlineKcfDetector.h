/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        LegacyOnlineKcfDetector.h
 * @brief       旧在线 KCF 跟踪服务
 * @details     声明面向旧喷嘴跟踪流程的 KCF 在线检测封装。
 ******************************************************************************/
#ifndef I2R_RLC_VISION_TRACK_LEGACY_ONLINE_KCF_DETECTOR_H
#define I2R_RLC_VISION_TRACK_LEGACY_ONLINE_KCF_DETECTOR_H

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

namespace i2r_rlc::vision {

/**
 * @brief 旧在线 KCF 与 gap 检测参数。
 */
struct LegacyOnlineKcfOptions {
    KcfFusionOptions fusion;
    /// @brief 在线纠偏显式启用原图精定位；离线/legacy 回放保持既有行为。
    bool enableSeamRefinement = false;
    OnlineSeamRefinementOptions seamRefinement;
    /// @brief 在线运行时显式启用；辅助结果永不成为独立原图焊点。
    bool enableSimulationGuidance = false;
    /// @brief 仅正式在线纠偏启用原图质量降权与连续新观测重捕获；其他入口保持旧策略。
    bool enableOnlineObservationFusion = false;
    /// @brief 相邻曝光间隔及原图恢复时间；调用方没有真实时间时 dt 保持 0。
    OnlineSeamRecoveryOptions recovery;
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
    CKcfUpdateDiagnostic tracker;
};

/**
 * @brief 旧在线 KCF 单帧输出。
 */
struct LegacyOnlineKcfResult {
    bool valid = false;
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

    LegacyOnlineKcfResult process(
        const cv::Mat &gray8,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const WeldProjectionContext &projectionContext,
        const LegacyOnlineKcfOptions &options);

private:
    std::unique_ptr<CKcfTracker> m_tracker;
    COnlineSeamContinuity m_continuity;
    CSimulationGuidedSeamEstimator m_guidedEstimator;
    std::uint64_t m_guidanceReferenceRevision = 0;
    bool m_guidanceRevisionKnown = false;
    int m_continuitySeamIndex = -1;
    /// @brief 模板内焊缝位置除以模板宽高；重建模板时更新，跟踪期间保持不变。
    cv::Point2d m_templateWeldAnchor;
    /// @brief 模板所属原图尺寸；分辨率变化时清除旧坐标和滤波状态。
    cv::Size m_templateImageSize;
    bool m_initialized = false;
    int m_framesSinceInitialize = 0;
    bool m_onlineObservationActive = false;
    bool m_hasImageSearchPoint = false;
    cv::Point2d m_lastImageSearchPoint{-1.0, -1.0};
    /// @brief 新原图支系已确认而粗 KCF 尚未同位验收；期间保留检测但不融合旧粗点。
    bool m_coarseRecoveryPending = false;
};

} // namespace i2r_rlc::vision

#endif
