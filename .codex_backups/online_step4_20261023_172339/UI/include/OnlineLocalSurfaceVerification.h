/**
 * @file OnlineLocalSurfaceVerification.h
 * @brief 用新帧真实母材表面对局部工件写回进行连续后验检查。
 * @details 仅保存值对象和连续计数；不提取图像、不访问设备或修改场景。
 */
#ifndef I2R_RLC_UI_ONLINE_LOCAL_SURFACE_VERIFICATION_H
#define I2R_RLC_UI_ONLINE_LOCAL_SURFACE_VERIFICATION_H

#include <cmath>

#include <QtGlobal>

namespace i2r_rlc::ui {

/**
 * @brief coordinator 线程独占的单次表面写回后验检查器。
 * @details 写入前留出基线仅作事务准入；新帧恶化使用同帧真实点的配对局部面误差。
 * 调用方须先确认会话、帧号关联、曝光晚于写回完成及当前工件位姿版本。
 * 本类只检查指标及连续新帧，不负责曝光/位姿身份，也不负责五秒事务超时。
 * 新帧指标必须使用真实表面与当前仿真重新求交的参考，禁止把变换后的旧激光
 * 交点当作新场景参考，或把模型估计接缝当作真实表面证据。
 */
class COnlineLocalSurfaceVerification final {
public:
    enum class Decision { Waiting, Verified, Rollback };
    /** @brief 证据缺失不能计作有效观测证明候选恶化。 */
    enum class ObservationStatus { Unavailable, Good, Worsened };

    /**
     * @brief 同一评估口径下的双向表面指标；长度单位 mm，覆盖比例为 [0,1]。
     * @details 原图支撑、最少点数、区域身份及距离口径由表面评估器校核。
     * 两向完整均值仅用于各自绝对上限；跨时刻的切片均值不作恶化对照。
     * 两个残差分别保留，避免方向均值掩盖一侧变坏。
     */
    struct Metrics {
        double realToVirtualResidualMm = 0.0;
        double virtualToRealResidualMm = 0.0;
        double realCoverageRatio = 0.0;
        double virtualCoverageRatio = 0.0;
    };

    /** @brief 同一新实测点集对原位及候选局部面的误差；不使用跨帧切片均值。 */
    struct Comparison {
        bool valid = false;
        double previousResidualMm = 0.0;
        double currentResidualMm = 0.0;
        double supportedRatio = 0.0;
    };

    /** @brief 单次事务冻结阈值；不随观测或后续参数热调抬升。 */
    struct Options {
        int requiredGoodFrames = 3;
        int maximumBadFrames = 3;
        double maximumResidualMm = 5.5;
        double maximumWorseningMm = 0.1;
        double minimumBidirectionalCoverage = 0.7;
    };

    /**
     * @brief 冻结写入前候选留出验收基线；非法基线或配置直接请求恢复。
     * @details 基线须已满足绝对残差和覆盖门槛。本类不重新证明留出来源，
     * 调用方应在写入前拒绝无此证据的候选，不能等写入后补造基线。
     */
    COnlineLocalSurfaceVerification(
        const Metrics &frozenPreWriteHoldout, const Options &options)
        : m_baselineMetrics(frozenPreWriteHoldout), m_options(options)
    {
        m_baselineUsable = validOptions(options)
            && withinAbsoluteLimits(frozenPreWriteHoldout, options);
        if (!m_baselineUsable) {
            m_decision = Decision::Rollback;
        }
    }

    /**
     * @brief 接收已完成曝光/位姿版本门控的新帧，重复旧帧不产生新证据。
     * @details 原图/数值无效或配对支持不足只打断连续证据，不累计恶化帧。
     * 有效表面超出绝对残差/覆盖门槛，或有效配对证明恶化，才累计坏帧。
     * 证据不足不晋升位姿，事务仍由调用方的有界超时结束；终态不再采样。
     */
    Decision observe(qint64 frameId, bool valid, const Metrics &metrics,
        const Comparison &comparison)
    {
        m_lastObservationGood = false;
        m_lastObservationStatus = ObservationStatus::Unavailable;
        if (m_decision != Decision::Waiting || frameId <= m_lastFrameId) {
            return m_decision;
        }
        m_lastFrameId = frameId;
        m_currentMetrics = metrics;
        const bool observedSurface = valid && validMetrics(metrics);
        const bool absoluteGood = observedSurface && withinAbsoluteLimits(metrics, m_options);
        const bool pairedEvidence = comparison.valid && validRatio(comparison.supportedRatio)
            && comparison.supportedRatio >= m_options.minimumBidirectionalCoverage
            && std::isfinite(comparison.previousResidualMm)
            && std::isfinite(comparison.currentResidualMm)
            && comparison.previousResidualMm >= 0.0 && comparison.currentResidualMm >= 0.0;
        const bool pairedGood = pairedEvidence
            && comparison.currentResidualMm <= m_options.maximumResidualMm
            && comparison.currentResidualMm
                <= comparison.previousResidualMm + m_options.maximumWorseningMm;
        const bool good = absoluteGood && pairedGood;
        const bool worsened = observedSurface
            && (!absoluteGood || (pairedEvidence && !pairedGood));
        m_lastObservationGood = good;
        if (good) {
            m_lastObservationStatus = ObservationStatus::Good;
            ++m_goodFrames;
            m_badFrames = 0;
            if (m_goodFrames >= m_options.requiredGoodFrames) {
                m_decision = Decision::Verified;
            }
        } else if (worsened) {
            m_lastObservationStatus = ObservationStatus::Worsened;
            m_goodFrames = 0;
            ++m_badFrames;
            if (m_badFrames >= m_options.maximumBadFrames) {
                m_decision = Decision::Rollback;
            }
        } else {
            // 离开冻结面或遮挡不是候选变差的反证；仍不能承接前面的合格帧。
            m_goodFrames = 0;
            m_badFrames = 0;
            ++m_unavailableFrames;
        }
        return m_decision;
    }

    Decision decision() const { return m_decision; }
    bool baselineUsable() const { return m_baselineUsable; }
    bool lastObservationGood() const { return m_lastObservationGood; }
    ObservationStatus lastObservationStatus() const { return m_lastObservationStatus; }
    int unavailableFrameCount() const { return m_unavailableFrames; }
    const Metrics &baselineMetrics() const { return m_baselineMetrics; }
    const Metrics &currentMetrics() const { return m_currentMetrics; }
    int goodFrameCount() const { return m_goodFrames; }
    int badFrameCount() const { return m_badFrames; }

private:
    static bool validOptions(const Options &options)
    {
        return options.requiredGoodFrames > 0 && options.maximumBadFrames > 0
            && std::isfinite(options.maximumResidualMm)
            && options.maximumResidualMm > 0.0
            && std::isfinite(options.maximumWorseningMm)
            && options.maximumWorseningMm >= 0.0
            && std::isfinite(options.minimumBidirectionalCoverage)
            && options.minimumBidirectionalCoverage > 0.0
            && options.minimumBidirectionalCoverage <= 1.0;
    }

    static bool validRatio(double value)
    {
        return std::isfinite(value) && value >= 0.0 && value <= 1.0;
    }

    static bool validMetrics(const Metrics &metrics)
    {
        return std::isfinite(metrics.realToVirtualResidualMm)
            && metrics.realToVirtualResidualMm >= 0.0
            && std::isfinite(metrics.virtualToRealResidualMm)
            && metrics.virtualToRealResidualMm >= 0.0
            && validRatio(metrics.realCoverageRatio) && validRatio(metrics.virtualCoverageRatio);
    }

    static bool withinAbsoluteLimits(
        const Metrics &metrics, const Options &options)
    {
        return std::isfinite(metrics.realToVirtualResidualMm)
            && metrics.realToVirtualResidualMm >= 0.0
            && metrics.realToVirtualResidualMm <= options.maximumResidualMm
            && std::isfinite(metrics.virtualToRealResidualMm)
            && metrics.virtualToRealResidualMm >= 0.0
            && metrics.virtualToRealResidualMm <= options.maximumResidualMm
            && validRatio(metrics.realCoverageRatio)
            && metrics.realCoverageRatio >= options.minimumBidirectionalCoverage
            && validRatio(metrics.virtualCoverageRatio)
            && metrics.virtualCoverageRatio
                >= options.minimumBidirectionalCoverage;
    }

    Metrics m_baselineMetrics;
    Metrics m_currentMetrics;
    Options m_options;
    qint64 m_lastFrameId = -1;
    int m_goodFrames = 0;
    int m_badFrames = 0;
    int m_unavailableFrames = 0;
    ObservationStatus m_lastObservationStatus = ObservationStatus::Unavailable;
    Decision m_decision = Decision::Waiting;
    bool m_baselineUsable = false;
    bool m_lastObservationGood = false;
};

} // namespace i2r_rlc::ui
#endif // I2R_RLC_UI_ONLINE_LOCAL_SURFACE_VERIFICATION_H
