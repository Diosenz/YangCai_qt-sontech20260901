/** @file OnlineSeamContinuity.h @brief 原图观测退化时的同帧仿真辅助连续目标。 */
#ifndef I2R_RLC_VISION_TRACK_ONLINE_SEAM_CONTINUITY_H
#define I2R_RLC_VISION_TRACK_ONLINE_SEAM_CONTINUITY_H
#include "feature/OnlineSeamRefiner.h"
#include "track/KcfTracker.h"
namespace i2r_rlc::vision {
/** @brief 来源是业务估计依据，不代表实测精度或机器人运动许可。 */
enum class OnlineSeamPointSource {
    Unavailable, Geometry, CorrectedKcf, SimulationPrediction,
    /// @brief 时序实测锚及原图肩部辅助估计；不具备独立焊点验收资格。
    GuidedEstimate
};
/** @brief 当前帧连续目标；独立定位验收只能使用另存的原图精定位点。 */
struct OnlineSeamContinuityResult {
    bool valid = false;
    cv::Point2d point{-1.0, -1.0};
    /// @brief 恢复限幅基点；正式在线沿图像历史，旧入口按同帧仿真运动推进。
    cv::Point2d motionPrediction{-1.0, -1.0};
    /// @brief 使用最近几何偏差校正后的粗观测；仅业务估计，不能作为独立原图焊点。
    cv::Point2d correctedCoarsePoint{-1.0, -1.0};
    OnlineSeamPointSource source = OnlineSeamPointSource::Unavailable;
    int degradedFrames = 0;
    bool transitionLimited = false;
    /// @brief 在线质量降权及过渡限幅后的原图观测系数；纯预测和旧入口为 0。
    double observationWeightX = 0.0;
    double observationWeightY = 0.0;
};
/**
 * @brief 视觉线程独占的连续估计器；不读设备、不产生控制指令。
 * 仅可靠几何更新粗点偏差；失去图像时以当前仿真运动推进最近目标，
 * 禁止把上一帧绝对像素固定在屏幕上。reset用于会话、尺寸及焊缝切换。
 */
class COnlineSeamContinuity final {
public:
    void reset();
    /** @brief 新原图支系确认后清除旧粗点偏差；保留业务位置与恢复限幅状态。 */
    void clearCoarseObservationBias();
    /** @brief 同步最终辅助输出供恢复限幅使用；不刷新实测偏移或粗点偏差。 */
    void synchronizeEstimate(const cv::Point2d &point,
        const cv::Point2d &simulated, int degradedFrames);
    /** @brief 输入同帧已验证参考、粗响应和几何；尺寸、坐标均为原图像素。 */
    OnlineSeamContinuityResult update(const cv::Size &size, const cv::Point2d &simulated,
        const cv::Point2d &coarse, bool coarseUsable, const CKcfUpdateDiagnostic &tracker,
        const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion);
    /**
     * @brief 正式在线纠偏按原图质量和运动创新降权粗观测，仍限制新增修正为每帧 2 px。
     * @details observationQuality 范围 [0,1]；粗点只能形成估计，不能刷新实测锚；CAD 平移不作为图像运动。
     * 旧 update 入口不使用该策略，避免改变离线与未启用的新行为。
     */
    OnlineSeamContinuityResult updateOnline(const cv::Size &size,
        const cv::Point2d &simulated, const cv::Point2d &coarse,
        double observationQuality, const CKcfUpdateDiagnostic &tracker,
        const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion);
    /** @brief 首次进入时可用重投影业务点限定过渡；已有图像历史不因 CAD 更新而清空。 */
    OnlineSeamContinuityResult updateOnline(const cv::Size &size,
        const cv::Point2d &simulated, const cv::Point2d &coarse,
        double observationQuality, const CKcfUpdateDiagnostic &tracker,
        const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion,
        const cv::Point2d &transitionTarget);
private:
    OnlineSeamContinuityResult updateImpl(const cv::Size &size,
        const cv::Point2d &simulated, const cv::Point2d &coarse, bool coarseUsable,
        const CKcfUpdateDiagnostic &tracker, const OnlineSeamRefinementResult &geometry,
        const KcfFusionOptions &fusion, bool qualityWeighted, double observationQuality);
    bool m_initialized = false;
    bool m_biasValid = false;
    KcfFusionOptions m_previousFusion;
    cv::Point2d m_bias;
    cv::Point2d m_referenceOffset;
    cv::Point2d m_previousPoint;
    cv::Point2d m_previousSimulation;
    int m_degradedFrames = 0;
    int m_recoveryFrames = 0;
};
}
#endif
