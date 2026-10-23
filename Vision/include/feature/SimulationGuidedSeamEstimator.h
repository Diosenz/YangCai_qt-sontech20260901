/**
 * @file SimulationGuidedSeamEstimator.h
 * @brief 原图焊点暂不可辨认时，以近期实测锚和同帧仿真运动估计原接缝。
 */
#ifndef I2R_RLC_VISION_FEATURE_SIMULATION_GUIDED_SEAM_ESTIMATOR_H
#define I2R_RLC_VISION_FEATURE_SIMULATION_GUIDED_SEAM_ESTIMATOR_H

#include <cstdint>
#include <chrono>
#include <vector>

#include <opencv2/core.hpp>

#include "feature/OnlineSeamRefiner.h"

namespace i2r_rlc::vision {

/** @brief 仅为工程续接预算；像素/处理帧均不代表机器人安全距离。 */
struct SimulationGuidedSeamOptions {
    /// @brief 仅在线纠偏显式启用连续高质量原图重捕获；缺省沿用旧时序门控。
    bool enableRawReacquisition = false;
    /// @brief 正式在线使用图像历史与双侧肩部定位；CAD 位移不能冒充相机运动。
    bool enableImageContinuity = false;
    int maximumDegradedFrames = 180;
    int maximumPredictionTimeMs = 12000;
    double maximumReferenceTravelPixel = 180.0;
    double maximumLocalCorrectionPixel = 4.0;
    double maximumBranchDistancePixel = 7.0;
    double maximumAnchorInnovationPixel = 12.0;
    /// @brief 累计趋势诊断阈值，不作为连续原图观测的拒绝门限。
    double maximumAccumulatedAnchorCorrectionPixel = 16.0;
    int minimumShoulderColumns = 20;
};

/** @brief 模型辅助诊断；valid 也始终是 estimated，不是独立实测验收。 */
struct SimulationGuidedSeamResult {
    bool attempted = false;
    bool valid = false;
    bool estimated = true;
    bool hasTrustedAnchor = false;
    bool predictionBudgetValid = false;
    bool hasRawShoulderSupport = false;
    /// @brief 图像几何续接是否启用；肩部质量仅用于业务估计，不代表独立精定位。
    bool imageContinuityActive = false;
    double imageObservationQuality = 0.0;
    /// @brief 本帧没有沿 CAD 的绝对平移推进历史图像点。
    bool modelTranslationSuppressed = false;
    bool localGapCenterSupported = false;
    bool ambiguous = false;
    /// @brief 原图候选虽局部有效，但不满足同位姿版本的运动补偿时序约束。
    bool rawJumpRejected = false;
    /// @brief 旧锚拒绝后，连续新原图证据正在重建或已经重建当前实测锚。
    bool reacquisitionPending = false;
    bool reacquired = false;
    /// @brief 旧实测两侧分支仍有原图支撑，新亮分支不能凭连续出现替换它。
    bool reacquisitionBlockedByPreviousBranch = false;
    int reacquisitionFrames = 0;
    bool accumulatedCorrectionExceeded = false;
    int degradedFrames = 0;
    std::int64_t anchorAgeMs = 0;
    int leftSupportColumns = 0;
    int rightSupportColumns = 0;
    int ambiguousColumns = 0;
    double maximumBranchRmsPixel = 0.0;
    double referenceTravelPixel = 0.0;
    cv::Point2d prediction{-1.0, -1.0};
    cv::Point2d point{-1.0, -1.0};
};

/**
 * @brief 视觉线程独占的时序估计器，不访问设备、不更新工件位姿。
 * @details 只有原图 refinement.valid 能刷新锚；辅助输出永不反馈成新锚。
 * 同帧仿真推进补偿相机/机器人运动，原图多峰只校核局部支撑及有限新增修正。
 * 默认入口在位姿版本变化时清历史；正式在线保留相机图像历史，模型只提供初始方向。
 * 预算到期仍搜索原图；缺测仅有限外推近期图像运动，不无限追随 CAD 平移。
 */
class CSimulationGuidedSeamEstimator final {
public:
    /** @brief 清除实测锚、方向及退化计数；可在会话边界重复调用。 */
    void reset();

    /**
     * @brief 消费同帧原图与参考；referenceRevision 是实际工件位姿版本。
     * @details 点与折线均为原图像素。原图无效或无实测锚时不种模型假锚。
     * rawRefinement.valid 时仅更新历史，调用方继续使用原始精定位链。
     */
    SimulationGuidedSeamResult update(const cv::Mat &gray8,
        const std::vector<cv::Point2d> &simulatedPolyline,
        const cv::Point2d &simulatedSeam,
        const OnlineSeamRefinementResult &rawRefinement,
        std::uint64_t referenceRevision,
        const SimulationGuidedSeamOptions &options);

    /** @brief 位姿切换时可用重投影业务点续接；该种子始终不构成实测锚。 */
    SimulationGuidedSeamResult update(const cv::Mat &gray8,
        const std::vector<cv::Point2d> &simulatedPolyline,
        const cv::Point2d &simulatedSeam,
        const OnlineSeamRefinementResult &rawRefinement,
        std::uint64_t referenceRevision,
        const SimulationGuidedSeamOptions &options,
        const cv::Point2d &transitionTarget);

private:
    /** @brief 双侧历史分支约束的在线续接；不刷新辅助点的独立实测身份。 */
    SimulationGuidedSeamResult updateImageContinuity(const cv::Mat &gray8,
        const std::vector<cv::Point2d> &simulatedPolyline,
        const cv::Point2d &simulatedSeam,
        const OnlineSeamRefinementResult &rawRefinement,
        const SimulationGuidedSeamOptions &options,
        const cv::Point2d &transitionTarget);
    bool m_imageContinuityActive = false;
    bool m_hasImagePosition = false;
    cv::Point2d m_imagePoint;
    cv::Point2d m_imageVelocity;
    /// @brief 仅完整原图精定位刷新；肩部估计不能移动独立实测基准。
    cv::Point2d m_lastImageSupportedPoint;
    /// @brief 最近有图像支撑的显示点，仅限制随后纯外推，不具备实测身份。
    cv::Point2d m_predictionStartPoint;
    double m_imageAnchorLeftSlope = 0.0;
    double m_imageAnchorRightSlope = 0.0;
    double m_imageAnchorLeftHeightOffset = 0.0;
    double m_imageAnchorRightHeightOffset = 0.0;
    bool m_hasAnchor = false;
    bool m_hasDirections = false;
    cv::Size m_imageSize;
    std::uint64_t m_referenceRevision = 0;
    cv::Point2d m_referenceOffset;
    cv::Point2d m_versionAnchorOffset;
    cv::Point2d m_anchorSimulation;
    std::chrono::steady_clock::time_point m_anchorTime;
    double m_leftSlope = 0.0;
    double m_rightSlope = 0.0;
    double m_anchorLeftModelSlope = 0.0;
    double m_anchorRightModelSlope = 0.0;
    double m_leftHeightOffset = 0.0;
    double m_rightHeightOffset = 0.0;
    int m_degradedFrames = 0;
    cv::Point2d m_reacquisitionOffset;
    cv::Point2d m_reacquisitionVelocity;
    double m_reacquisitionLeftSlope = 0.0;
    double m_reacquisitionRightSlope = 0.0;
    double m_reacquisitionGapHeight = 0.0;
    int m_reacquisitionFrames = 0;
};

} // namespace i2r_rlc::vision
#endif // I2R_RLC_VISION_FEATURE_SIMULATION_GUIDED_SEAM_ESTIMATOR_H
