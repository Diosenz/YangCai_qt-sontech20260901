/** @file OnlineSeamContinuity.cpp @brief 分离精定位观测、粗跟踪与仿真预测。 */
#include "track/OnlineSeamContinuity.h"
#include <algorithm>
#include <cmath>
namespace i2r_rlc::vision {
namespace {
bool inside(const cv::Point2d &p, const cv::Size &s)
{
    return std::isfinite(p.x) && std::isfinite(p.y)
        && p.x >= 0.0 && p.y >= 0.0 && p.x < s.width && p.y < s.height;
}
cv::Point2d bounded(const cv::Point2d &d, double limit)
{
    const double length = cv::norm(d);
    return length > limit ? d * (limit / length) : d;
}
}
void COnlineSeamContinuity::reset()
{
    m_initialized = false;
    m_biasValid = false;
    m_bias = {};
    m_referenceOffset = {};
    m_previousPoint = {};
    m_previousSimulation = {};
    m_degradedFrames = 0;
    m_recoveryFrames = 0;
    resetRecoveryTime();
}
void COnlineSeamContinuity::resetRecoveryTime()
{
    m_previousTimedGeometryValid = false;
    m_previousTimedGeometry = {};
    m_timedRecoveryActive = false;
    m_recoveryElapsedSeconds = 0.0;
    m_recoveryDurationSeconds = 0.0;
}
void COnlineSeamContinuity::clearCoarseObservationBias()
{
    m_biasValid = false;
    m_bias = {};
}
void COnlineSeamContinuity::synchronizeEstimate(const cv::Point2d &point,
    const cv::Point2d &simulated, int degradedFrames)
{
    m_initialized = true;
    m_previousPoint = point;
    m_previousSimulation = simulated;
    m_degradedFrames = degradedFrames;
    if (degradedFrames > 0) resetRecoveryTime();
}
OnlineSeamContinuityResult COnlineSeamContinuity::update(
    const cv::Size &size, const cv::Point2d &simulated, const cv::Point2d &coarse,
    bool coarseUsable, const CKcfUpdateDiagnostic &tracker,
    const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion)
{
    return updateImpl(size, simulated, coarse, coarseUsable, tracker,
        geometry, fusion, false, 1.0, nullptr);
}
OnlineSeamContinuityResult COnlineSeamContinuity::updateOnline(
    const cv::Size &size, const cv::Point2d &simulated, const cv::Point2d &coarse,
    double observationQuality, const CKcfUpdateDiagnostic &tracker,
    const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion)
{
    const double quality = std::isfinite(observationQuality)
        ? std::clamp(observationQuality, 0.0, 1.0) : 0.0;
    return updateImpl(size, simulated, coarse, quality > 0.0, tracker,
        geometry, fusion, true, quality, nullptr);
}
OnlineSeamContinuityResult COnlineSeamContinuity::updateOnline(
    const cv::Size &size, const cv::Point2d &simulated, const cv::Point2d &coarse,
    double observationQuality, const CKcfUpdateDiagnostic &tracker,
    const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion,
    const cv::Point2d &transitionTarget)
{
    if (!m_initialized && inside(transitionTarget, size)) {
        // 新版本仅接续最终业务坐标，不能携带旧版本的观测身份与模板偏差。
        reset();
        m_previousFusion = fusion;
        synchronizeEstimate(transitionTarget, simulated, 0);
        m_recoveryFrames = 1;
    }
    return updateOnline(size, simulated, coarse, observationQuality,
        tracker, geometry, fusion);
}
OnlineSeamContinuityResult COnlineSeamContinuity::updateOnline(
    const cv::Size &size, const cv::Point2d &simulated, const cv::Point2d &coarse,
    double observationQuality, const CKcfUpdateDiagnostic &tracker,
    const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion,
    const cv::Point2d &transitionTarget, const OnlineSeamRecoveryOptions &recovery)
{
    if (!m_initialized && inside(transitionTarget, size)) {
        reset();
        m_previousFusion = fusion;
        synchronizeEstimate(transitionTarget, simulated, 0);
        m_recoveryFrames = 1;
    }
    const double quality = std::isfinite(observationQuality)
        ? std::clamp(observationQuality, 0.0, 1.0) : 0.0;
    return updateImpl(size, simulated, coarse, quality > 0.0, tracker,
        geometry, fusion, true, quality, &recovery);
}
OnlineSeamContinuityResult COnlineSeamContinuity::updateImpl(
    const cv::Size &size, const cv::Point2d &simulated, const cv::Point2d &coarse,
    bool coarseUsable, const CKcfUpdateDiagnostic &tracker,
    const OnlineSeamRefinementResult &geometry, const KcfFusionOptions &fusion,
    bool qualityWeighted, double observationQuality,
    const OnlineSeamRecoveryOptions *recovery)
{
    OnlineSeamContinuityResult result;
    if (!inside(simulated, size)) { reset(); return result; }
    if (m_initialized && (fusion.mode != m_previousFusion.mode
        || fusion.simulationWeight != m_previousFusion.simulationWeight)) {
        if (recovery && (m_degradedFrames > 0 || m_recoveryFrames > 0)) {
            // 用户改融合比例只改变恢复目标；不能清掉旧业务误差而瞬切，也不是原图运动。
            clearCoarseObservationBias();
        } else {
            reset();
        }
    }
    m_previousFusion = fusion;
    // 明显参考跳变不可携带旧工件/旧区域偏差；当前参考仍作为显式预测显示。
    if (!qualityWeighted && m_initialized
        && cv::norm(simulated - m_previousSimulation) > 25.0) reset();
    const cv::Point2d anchoredPrediction = qualityWeighted && m_initialized
        ? m_previousPoint : simulated + m_referenceOffset;
    // 在线的 CAD 平移可能来自 ICP 写回，不能直接当作真实激光线运动。
    // 当前原图肩部与近期图像运动由 guided 路径校核；旧入口保持仿真推进。
    cv::Point2d prediction = m_initialized
        ? (qualityWeighted ? m_previousPoint
            : simulated + m_previousPoint - m_previousSimulation) : simulated;
    const bool acceptedGeometry = geometry.valid && inside(geometry.point, size);
    if (!recovery || !acceptedGeometry) resetRecoveryTime();
    if (recovery && m_initialized && m_timedRecoveryActive
        && m_previousTimedGeometryValid && acceptedGeometry) {
        // 相邻已验收图像位移按当前融合比例推进；首个恢复跳变和 CAD 写回不能充当运动。
        // 使用 raw 增量而非融合目标增量，避免把参考位姿或增益变化伪装成真实图像运动。
        const auto motion = geometry.point - m_previousTimedGeometry;
        prediction.x += motion.x * (tracker.fusionApplied
            ? std::clamp(tracker.gainX, 0.0, 1.0) : 1.0);
        prediction.y += motion.y * (tracker.fusionApplied
            ? std::clamp(tracker.gainY, 0.0, 1.0) : 1.0);
    }
    result.motionPrediction = {
        std::clamp(prediction.x, 0.0, static_cast<double>(size.width - 1)),
        std::clamp(prediction.y, 0.0, static_cast<double>(size.height - 1))};
    coarseUsable = coarseUsable && inside(coarse, size);
    if (acceptedGeometry) {
        result.source = OnlineSeamPointSource::Geometry;
        result.point = geometry.point;
        if (tracker.fusionApplied) {
            result.point.x = simulated.x + std::clamp(tracker.gainX, 0.0, 1.0)
                * (geometry.point.x - simulated.x);
            result.point.y = simulated.y + std::clamp(tracker.gainY, 0.0, 1.0)
                * (geometry.point.y - simulated.y);
        }
        if (qualityWeighted) {
            result.observationWeightX = tracker.fusionApplied
                ? std::clamp(tracker.gainX, 0.0, 1.0) : 1.0;
            result.observationWeightY = tracker.fusionApplied
                ? std::clamp(tracker.gainY, 0.0, 1.0) : 1.0;
        }
        if (coarseUsable) {
            const auto observedBias = bounded(geometry.point - coarse, 40.0);
            m_bias = m_biasValid ? m_bias + bounded((observedBias - m_bias) * 0.2, 2.0)
                : observedBias;
            m_biasValid = true;
        }
        if (m_degradedFrames > 0) m_recoveryFrames = 5;
        m_degradedFrames = 0;
    } else {
        ++m_degradedFrames;
        const auto corrected = coarse + (m_biasValid ? m_bias : cv::Point2d());
        if (coarseUsable && inside(corrected, size)) { result.correctedCoarsePoint = corrected; }
        const double innovation = cv::norm(corrected - anchoredPrediction);
        // 响应强度不等于焊点可靠性；还需与同帧仿真推进的目标相容。
        if (coarseUsable && inside(corrected, size)
            && innovation <= (qualityWeighted ? 24.0 : 12.0)) {
            result.source = OnlineSeamPointSource::CorrectedKcf;
            double gainX = tracker.fusionApplied ? tracker.gainX : 0.35;
            double gainY = tracker.fusionApplied ? tracker.gainY : 0.35;
            if (fusion.mode == KcfFusionMode::KcfOnly) { gainX = gainY = 1.0; }
            else { gainX = std::clamp(gainX, 0.0, 0.35); gainY = std::clamp(gainY, 0.0, 0.35); }
            if (qualityWeighted) {
                // 质量低或创新大时增大等效观测噪声，而不是瞬间丢弃有图像支持的 KCF。
                // 仍以最近几何参考约束总偏移；估计输出不会刷新该参考。
                const double noiseScale = observationQuality
                    / (1.0 + innovation * innovation / (12.0 * 12.0));
                gainX *= noiseScale;
                gainY *= noiseScale;
                result.observationWeightX = gainX;
                result.observationWeightY = gainY;
                if (gainX <= 1e-9 && gainY <= 1e-9) {
                    result.source = OnlineSeamPointSource::SimulationPrediction;
                }
            }
            result.point = cv::Point2d(prediction.x + gainX * (corrected.x - prediction.x),
                prediction.y + gainY * (corrected.y - prediction.y));
        } else {
            result.source = OnlineSeamPointSource::SimulationPrediction;
            result.point = prediction;
        }
    }
    if (recovery && acceptedGeometry && m_initialized && m_recoveryFrames > 0) {
        if (!m_timedRecoveryActive) {
            m_timedRecoveryActive = true;
            m_recoveryElapsedSeconds = 0.0;
            m_recoveryDurationSeconds = std::isfinite(recovery->durationSeconds)
                && recovery->durationSeconds > 0.0 ? recovery->durationSeconds : 0.6;
        }
        const double previousPhase = std::clamp(
            m_recoveryElapsedSeconds / m_recoveryDurationSeconds, 0.0, 1.0);
        if (m_previousTimedGeometryValid && std::isfinite(recovery->dtSeconds)
            && recovery->dtSeconds > 0.0) {
            m_recoveryElapsedSeconds = std::min(m_recoveryDurationSeconds,
                m_recoveryElapsedSeconds + recovery->dtSeconds);
        }
        const double phase = std::clamp(
            m_recoveryElapsedSeconds / m_recoveryDurationSeconds, 0.0, 1.0);
        // 三次平滑阶跃让历史误差按实际曝光时间归零，起止修正速度均为零。
        // 对剩余误差按比例收敛，使当前可信图像运动与历史恢复误差彼此独立。
        const auto remaining = [](double value) {
            return (1.0 - value) * (1.0 - value) * (1.0 + 2.0 * value);
        };
        const double scale = previousPhase < 1.0
            ? remaining(phase) / remaining(previousPhase) : 0.0;
        const auto target = result.point;
        result.point = target + (prediction - target) * scale;
        result.recoveryResidualPixel = cv::norm(result.point - target);
        result.transitionLimited = result.recoveryResidualPixel > 1e-7;
        result.observationWeightX *= 1.0 - scale;
        result.observationWeightY *= 1.0 - scale;
        result.recoveryElapsedSeconds = m_recoveryElapsedSeconds;
        result.recoveryDurationSeconds = m_recoveryDurationSeconds;
        result.recoveryActive = result.transitionLimited && phase < 1.0;
        m_timedRecoveryActive = result.recoveryActive;
        m_recoveryFrames = result.recoveryActive ? 1 : 0;
    } else if (m_initialized && (m_degradedFrames > 0 || m_recoveryFrames > 0)) {
        // 退化辅助及未传曝光时间的旧入口保留原有 2 px 修正边界。
        const auto delta = result.point - prediction;
        result.transitionLimited = cv::norm(delta) > 2.0;
        if (result.transitionLimited) {
            const double scale = 2.0 / cv::norm(delta);
            result.observationWeightX *= scale;
            result.observationWeightY *= scale;
        }
        result.point = prediction + bounded(delta, 2.0);
        if (m_recoveryFrames > 0) m_recoveryFrames = result.transitionLimited ? 1 : 0;
    }
    if (recovery && acceptedGeometry) {
        m_previousTimedGeometry = geometry.point;
        m_previousTimedGeometryValid = true;
    }
    result.point.x = std::clamp(result.point.x, 0.0, static_cast<double>(size.width - 1));
    result.point.y = std::clamp(result.point.y, 0.0, static_cast<double>(size.height - 1));
    if (result.source == OnlineSeamPointSource::Geometry) {
        m_referenceOffset = result.point - simulated;
    }
    result.valid = inside(result.point, size);
    result.degradedFrames = m_degradedFrames;
    m_initialized = result.valid;
    m_previousPoint = result.point;
    m_previousSimulation = simulated;
    return result;
}
}
