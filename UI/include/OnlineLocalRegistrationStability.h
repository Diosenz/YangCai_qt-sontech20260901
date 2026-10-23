/**
 * @file OnlineLocalRegistrationStability.h
 * @brief 在线 ICP 独立收益保护和未经几何验证的历史区域监测。
 */
#ifndef I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_STABILITY_H
#define I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_STABILITY_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <QString>
#include <QtGlobal>

#include "LocalizationTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief 保存连续采样的轻量区域观测，不访问设备、文件或 QObject。
 * @details 状态由 coordinator 线程独占；evaluate 可使用输入值快照在配准线程调用。
 * 历史虚拟点只随后验及归档成功的修正推进，不重算旧激光平面交点。
 * 坐标传递不证明实测/虚拟交点属于同一材料点；缺少 CAD 身份和独立留出验证，
 * 历史始终仅作监测，不参与候选否决，也不因样本数或跨度自动晋升为成熟证据。
 */
class COnlineLocalRegistrationStability final {
public:
    using Point = localization::Point3D_mm;

    /** @brief 同帧父级 TCP 系独立焊点观测；长度单位 mm，不使用融合输出。 */
    struct Observation {
        Point realPoint;
        Point virtualPoint;
        qint64 frameId = -1;
    };

    /** @brief referenceErrorMm 固定为建立时误差，禁止随每次允许恶化重新抬高。 */
    struct Anchor {
        Observation observation;
        double referenceErrorMm = 0.0;
        /// @brief 同区域采样计数，含相关帧，不能解释为独立验证数量。
        int sampleCount = 1;
        qint64 lastFrameId = -1;
        /// @brief 最近一帧在采样当时的误差；不随位姿推进重写。
        double lastObservedErrorMm = 0.0;
    };

    /** @brief 稳定保护的可归档诊断；所有残差、收益及上限均为 mm。 */
    struct Evaluation {
        bool accepted = false;
        QString reason;
        double beforeMeanMm = 0.0;
        double afterMeanMm = 0.0;
        double requiredGainMm = 0.0;
        double maximumHistoricalErrorMm = 0.0;
        int historicalCount = 0;
        double historyBeforeMeanMm = 0.0;
        double historyAfterMeanMm = 0.0;
        double historyBeforeMaxMm = 0.0;
        double historyAfterMaxMm = 0.0;
        /// @brief 相对本次前值或固定基线恶化、或超过绝对上限的区域数，仅作冲突告警。
        int worsenedRegionCount = 0;
        double spatialSpanMm = 0.0;
        int regionCount = 0;
        int invalidHistoricalRegionCount = 0;
        QString evidenceStatus = QStringLiteral("collecting");
        bool historyEnforced = false;
    };

    /**
     * @brief 同批当前观测保留逐帧 0.1 mm 噪声容差，并有至少 max(0.1 mm,5%) 的平均收益。
     * @details 当前表面和焊点均值都不超过 2 mm 时，最低绝对收益提高为 0.2 mm。
     * 历史相对建立误差、本次前误差恶化 0.1 mm 或超过误差上限时仅标记冲突。
     * 历史异常或无效区域不会改变当前收益结论；缺少几何验证时绝不实施历史硬门。
     * 常量为首版收益门槛与监测告警值，不比较不同窗口的历史最低表面残差。
     */
    static Evaluation evaluate(const std::vector<Observation> &current,
        const std::vector<Anchor> &anchors, const localization::RigidTransform &correction,
        double beforeSurfaceResidualMm, double maximumErrorMm)
    {
        Evaluation result;
        result.historicalCount = static_cast<int>(anchors.size());
        if (current.empty() || !validTransform(correction)
            || !std::isfinite(beforeSurfaceResidualMm) || beforeSurfaceResidualMm < 0.0
            || !std::isfinite(maximumErrorMm) || maximumErrorMm <= 0.0) {
            result.reason = QStringLiteral("稳定更新保护：缺少当前观测或输入参数无效。");
            return result;
        }
        bool currentWorsened = false;
        bool currentExceeded = false;
        for (const auto &observation : current) {
            if (!validObservation(observation)) {
                result.reason = QStringLiteral("稳定更新保护：当前独立观测无效。");
                return result;
            }
            const double before = distance(observation.realPoint, observation.virtualPoint);
            const double after = distance(observation.realPoint,
                transform(observation.virtualPoint, correction));
            if (!std::isfinite(before) || !std::isfinite(after)) {
                result.reason = QStringLiteral("稳定更新保护：当前误差非有限值。");
                return result;
            }
            result.beforeMeanMm += before / static_cast<double>(current.size());
            result.afterMeanMm += after / static_cast<double>(current.size());
            currentWorsened = currentWorsened
                || after > before + MAXIMUM_CURRENT_WORSENING_MM + NUMERICAL_TOLERANCE_MM;
            currentExceeded = currentExceeded || after > maximumErrorMm;
        }
        if (!std::isfinite(result.beforeMeanMm) || !std::isfinite(result.afterMeanMm)) {
            result.reason = QStringLiteral("稳定更新保护：当前平均误差非有限值。");
            return result;
        }
        result.requiredGainMm = std::max(MINIMUM_GAIN_MM, result.beforeMeanMm * MINIMUM_GAIN_RATIO);
        if (result.beforeMeanMm <= STABLE_ERROR_MM && beforeSurfaceResidualMm <= STABLE_ERROR_MM) {
            result.requiredGainMm = std::max(result.requiredGainMm, STABLE_MINIMUM_GAIN_MM);
        }
        std::vector<Point> historicalRealPoints;
        for (const auto &anchor : anchors) {
            if (!validObservation(anchor.observation) || !std::isfinite(anchor.referenceErrorMm)
                || anchor.referenceErrorMm < 0.0 || anchor.sampleCount < 1
                || !std::isfinite(anchor.lastObservedErrorMm) || anchor.lastObservedErrorMm < 0.0) {
                ++result.invalidHistoricalRegionCount;
                continue;
            }
            const double before = distance(anchor.observation.realPoint, anchor.observation.virtualPoint);
            const double after = distance(anchor.observation.realPoint,
                transform(anchor.observation.virtualPoint, correction));
            if (!std::isfinite(before) || !std::isfinite(after)) {
                ++result.invalidHistoricalRegionCount;
                continue;
            }
            ++result.regionCount;
            result.historyBeforeMeanMm += (before - result.historyBeforeMeanMm) / result.regionCount;
            result.historyAfterMeanMm += (after - result.historyAfterMeanMm) / result.regionCount;
            result.historyBeforeMaxMm = std::max(result.historyBeforeMaxMm, before);
            result.historyAfterMaxMm = std::max(result.historyAfterMaxMm, after);
            for (const auto &point : historicalRealPoints) {
                const double span = distance(point, anchor.observation.realPoint);
                if (std::isfinite(span)) { result.spatialSpanMm = std::max(result.spatialSpanMm, span); }
            }
            historicalRealPoints.push_back(anchor.observation.realPoint);
            if (after > std::min(anchor.referenceErrorMm + MAXIMUM_HISTORICAL_WORSENING_MM,
                    maximumErrorMm) + NUMERICAL_TOLERANCE_MM
                || after > before + MAXIMUM_HISTORICAL_WORSENING_MM + NUMERICAL_TOLERANCE_MM) {
                ++result.worsenedRegionCount;
            }
        }
        result.maximumHistoricalErrorMm = result.historyAfterMaxMm;
        if (!anchors.empty()) {
            result.evidenceStatus = result.worsenedRegionCount > 0
                ? QStringLiteral("conflicting") : QStringLiteral("unverified_geometry");
        }
        if (currentWorsened || currentExceeded) {
            result.reason = QStringLiteral("稳定更新保护：当前独立焊点恶化或超限。");
        } else if (result.beforeMeanMm - result.afterMeanMm + NUMERICAL_TOLERANCE_MM
            < result.requiredGainMm) {
            result.reason = QStringLiteral("稳定更新保护：独立焊点实际收益不足。");
        } else {
            result.accepted = true;
            result.reason = QStringLiteral("稳定更新保护：当前独立焊点收益通过。");
        }
        result.reason += QStringLiteral(" 历史仅监测：%1，有效区域=%2，冲突区域=%3，无效区域=%4，不参与否决。")
            .arg(result.evidenceStatus).arg(result.regionCount).arg(result.worsenedRegionCount)
            .arg(result.invalidHistoricalRegionCount);
        return result;
    }

    /** @brief 返回同一当前位姿下的历史值对象；跨线程调用方必须复制后使用。 */
    const std::vector<Anchor> &anchors() const { return m_anchors; }

    /** @brief 会话、观测连续性、场景或参数失效时清空，不执行任何场景操作。 */
    void clear()
    {
        m_anchors.clear();
        m_lastObservedFrameId = -1;
    }

    /**
     * @brief 在当前已确认场景位姿下独立采样，与候选求解及写回成功无关。
     * @details 同区域仅更新统计，不替换首份近似对应或抬高建立基线。
     * 重复/旧帧、非法样本或容量满且不属于已有区域时返回 false；不清空已有历史。
     * 事务后验期间不得提前输入新位姿观测，应在 promote 推进旧坐标后采入新帧。
     */
    bool observe(const Observation &observation)
    {
        if (!validObservation(observation) || observation.frameId <= m_lastObservedFrameId) { return false; }
        const double error = distance(observation.realPoint, observation.virtualPoint);
        if (!std::isfinite(error)) { return false; }
        m_lastObservedFrameId = observation.frameId;
        for (auto &anchor : m_anchors) {
            if (distance(anchor.observation.realPoint, observation.realPoint) < MINIMUM_ANCHOR_SPACING_MM) {
                if (anchor.sampleCount < std::numeric_limits<int>::max()) { ++anchor.sampleCount; }
                anchor.lastFrameId = observation.frameId;
                anchor.lastObservedErrorMm = error;
                return true;
            }
        }
        if (m_anchors.size() >= MAXIMUM_ANCHORS) { return false; }
        m_anchors.push_back({observation, error, 1, observation.frameId, error});
        return true;
    }

    /**
     * @brief 仅在后验事务晋升且归档成功后调用，先推进旧参考点，再采入合格新帧。
     * @details verifiedGoodFrames 已位于修正后的父级系，只接收最终连续合格帧。
     * 新实测点距既有点不足 10 mm 时仅更新该区统计；最多保留 64 区，不淘汰早期观测。
     * 通过 observe 统一去重，不把已采过的帧重复计为新证据；空新帧允许只推进位姿。
     * 非有限值或非刚体修正使历史失效并返回 false，不部分推进历史。
     */
    bool promote(const localization::RigidTransform &correction,
        const std::vector<Observation> &verifiedGoodFrames)
    {
        if (!validTransform(correction)) {
            clear();
            return false;
        }
        for (const auto &observation : verifiedGoodFrames) {
            if (!validObservation(observation)
                || !std::isfinite(distance(observation.realPoint, observation.virtualPoint))) {
                clear();
                return false;
            }
        }
        auto advanced = m_anchors;
        for (auto &anchor : advanced) {
            anchor.observation.virtualPoint = transform(anchor.observation.virtualPoint, correction);
            if (!validObservation(anchor.observation)
                || !std::isfinite(distance(anchor.observation.realPoint, anchor.observation.virtualPoint))) {
                clear();
                return false;
            }
        }
        m_anchors = std::move(advanced);
        for (const auto &observation : verifiedGoodFrames) { observe(observation); }
        return true;
    }

private:
    static constexpr double NUMERICAL_TOLERANCE_MM = 1e-6;
    static constexpr double MINIMUM_GAIN_MM = 0.1;
    static constexpr double MINIMUM_GAIN_RATIO = 0.05;
    static constexpr double STABLE_ERROR_MM = 2.0;
    static constexpr double STABLE_MINIMUM_GAIN_MM = 0.2;
    static constexpr double MAXIMUM_CURRENT_WORSENING_MM = 0.1;
    static constexpr double MAXIMUM_HISTORICAL_WORSENING_MM = 0.1;
    static constexpr double MINIMUM_ANCHOR_SPACING_MM = 10.0;
    static constexpr std::size_t MAXIMUM_ANCHORS = 64;

    static bool finitePoint(const Point &point)
    {
        return std::isfinite(point.x_mm) && std::isfinite(point.y_mm) && std::isfinite(point.z_mm);
    }

    static bool validObservation(const Observation &observation)
    {
        return observation.frameId >= 0 && finitePoint(observation.realPoint)
            && finitePoint(observation.virtualPoint);
    }

    /** @brief 防止非法矩阵破坏跨次传递的材料坐标；允许现有 ICP 的微小数值误差。 */
    static bool validTransform(const localization::RigidTransform &correction)
    {
        const auto &m = correction.rowMajor;
        if (!std::all_of(m.begin(), m.end(), [](double value) { return std::isfinite(value); })
            || std::abs(m[12]) > 1e-6 || std::abs(m[13]) > 1e-6 || std::abs(m[14]) > 1e-6
            || std::abs(m[15] - 1.0) > 1e-6) { return false; }
        for (int row = 0; row < 3; ++row) {
            for (int other = 0; other < 3; ++other) {
                double dot = 0.0;
                for (int column = 0; column < 3; ++column) { dot += m[row * 4 + column] * m[other * 4 + column]; }
                if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1e-4) { return false; }
            }
        }
        const double determinant = m[0] * (m[5] * m[10] - m[6] * m[9])
            - m[1] * (m[4] * m[10] - m[6] * m[8]) + m[2] * (m[4] * m[9] - m[5] * m[8]);
        return std::abs(determinant - 1.0) <= 1e-4;
    }

    static Point transform(const Point &point, const localization::RigidTransform &correction)
    {
        const auto &m = correction.rowMajor;
        return {m[0] * point.x_mm + m[1] * point.y_mm + m[2] * point.z_mm + m[3],
            m[4] * point.x_mm + m[5] * point.y_mm + m[6] * point.z_mm + m[7],
            m[8] * point.x_mm + m[9] * point.y_mm + m[10] * point.z_mm + m[11]};
    }

    static double distance(const Point &a, const Point &b)
    {
        return std::hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm, a.z_mm - b.z_mm);
    }

    std::vector<Anchor> m_anchors;
    qint64 m_lastObservedFrameId = -1;
};

} // namespace i2r_rlc::ui
#endif // I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_STABILITY_H
