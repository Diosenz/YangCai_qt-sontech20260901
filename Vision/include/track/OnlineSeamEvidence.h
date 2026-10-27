/**
 * @file OnlineSeamEvidence.h
 * @brief 在线和回放共用的焊缝证据状态机。
 * @details 视觉 worker 独占；不访问设备；连续跟随资格与独立几何资格分别输出。
 */
#ifndef I2R_RLC_VISION_TRACK_ONLINE_SEAM_EVIDENCE_H
#define I2R_RLC_VISION_TRACK_ONLINE_SEAM_EVIDENCE_H

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "track/OnlineSeamContinuity.h"

namespace i2r_rlc::vision {

/** @brief 图像门限沿用回放候选；连续跟随显式启用，其他入口沿用时间/行程预算。 */
struct COnlineSeamEvidenceOptions {
    int anchorConfirmationFrames = 2;
    int recoveryConfirmationFrames = 2;
    /// @brief 测试阶段使用 24 px 图像残差走廊；不限制经参考补偿后的总扫描位移。
    int identityCorridorPixel = 24;
    double identityMinPatchScore = 0.55;
    double identityMinBranchScore = 0.40;
    double identityMinUniqueness = 0.02;
    double recoveryMaxStepPixel = 6.0;
    double maxPredictionMs = 0.0;
    double maxPredictionTravelMm = 0.0;
    bool enableMotherConstraints = false;
    /// @brief 仅正式在线启用：建锚后逐曝光续接供跟随，不授予独立几何资格。
    bool enableContinuousOutput = false;
    /// @brief 续接及恢复的每曝光像素步长上限；异常参考跳变另行拒绝。
    double maxContinuousStepPixel = 8.0;
};

/**
 * @brief 曝光关联的续接上下文；空值表示证据缺失，禁止用 TCP 净位移代替沿缝弧长。
 * @details progressReference 固定标识工件路径；previousReferencePixel 必须是
 *          旧模型版本在当前曝光的投影，不能传上一曝光的旧像素。
 */
struct COnlineSeamEvidenceContext {
    std::int64_t timestampMs = -1;
    std::optional<double> weldProgressMm;
    std::string progressReference;
    std::string progressProvenance;
    std::int64_t progressExposureFrameId = -1;
    std::optional<cv::Point2d> referencePixel;
    std::int64_t referenceExposureFrameId = -1;
    std::optional<cv::Point2d> previousReferencePixel;
    std::uint64_t previousReferenceRevision = 0;
    std::int64_t previousReferenceExposureFrameId = -1;
};

/** @brief 按值导出的最终资格；原始候选在 detector 的 rawRefinement 中单独保留。 */
struct COnlineSeamEvidenceResult {
    bool enabled = false;
    bool freshExposure = false;
    bool pixelAvailable = false;
    bool independentGeometry = false;
    bool controlEligible = false;
    bool anchorRefreshed = false;
    std::int64_t anchorFrameId = -1;
    int confirmationCount = 0;
    int motherSupportRank = 0;
    double identityScore = 0.0;
    double elapsedMs = -1.0;
    double travelMm = 0.0;
    cv::Point2d point{-1.0, -1.0};
    OnlineSeamPointSource source = OnlineSeamPointSource::Unavailable;
    std::string message;
    /// @brief 临时逐项门禁诊断；仅归档，不参与验收或状态更新。
    std::string debugDetail;
    /// @brief 当帧实际门限快照，用于离线回放，不作为配置入口。
    COnlineSeamEvidenceOptions debugOptions;
};

/**
 * @brief 可靠原图身份、母材约束、有限续接和恢复确认的单线程状态边界。
 * @details 原图候选及其空间投影先验收，返回 independentGeometry 后才允许
 *          detector 更新 KCF 模板。估计不能更新锚点、预算或独立 ICP 资格；正式在线可显式用于连续跟随。
 */
class COnlineSeamEvidence final {
public:
    COnlineSeamEvidence();
    ~COnlineSeamEvidence();
    /** @brief 会话/焊缝/分辨率变化时清空全部证据，不能沿用前一目标身份。 */
    void reset();
    /** @brief 缺图或投影门禁中断恢复计数，保留最后可靠锚点和既有预算。 */
    void notifyGap();
    /** @brief 在任何模板训练之前调用；geometryProjected 表示原始候选投影成功。 */
    COnlineSeamEvidenceResult evaluate(const cv::Mat &gray8, std::int64_t frameId,
        std::uint64_t revision, const COnlineSeamEvidenceContext &context,
        const OnlineSeamRefinementResult &geometry, bool geometryProjected,
        const cv::Point2d &coarse, bool coarseUsable,
        const COnlineSeamEvidenceOptions &options);

private:
    struct CState;
    std::unique_ptr<CState> m_state;
};

} // namespace i2r_rlc::vision
#endif // I2R_RLC_VISION_TRACK_ONLINE_SEAM_EVIDENCE_H
