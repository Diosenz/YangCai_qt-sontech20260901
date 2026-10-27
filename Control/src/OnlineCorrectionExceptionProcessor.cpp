/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionExceptionProcessor.cpp
 * @brief       在线检测段异常处理实现
 * @details     沿用拷贝工程的逆变换匹配方向，适配逐帧累计、边界释放及单次前向输出。
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Geometry>

#include "geometry/IcpRegistrationMatcher.h"
#include "OnlineCorrectionExceptionProcessor.h"

namespace i2r_rlc::control {
namespace {

constexpr std::size_t MIN_ICP_POINTS = 5U;
constexpr std::size_t MAX_WINDOW_POINTS = 512U;
constexpr double MIN_WINDOW_SPAN_MM = 2.0;

bool isValid(const OnlineCorrectionPoint &point)
{
    return point.valid && std::isfinite(point.xMm)
        && std::isfinite(point.yMm) && std::isfinite(point.zMm);
}

double squaredDistance(const OnlineCorrectionPoint &lhs, const OnlineCorrectionPoint &rhs)
{
    const double dx = lhs.xMm - rhs.xMm;
    const double dy = lhs.yMm - rhs.yMm;
    const double dz = lhs.zMm - rhs.zMm;
    return dx * dx + dy * dy + dz * dz;
}

struct PathProjection {
    OnlineCorrectionPoint point;
    double phase = 0.0;
    double arcLengthMm = 0.0;
};

/** @brief 模型刚体变换前后弧长相同，用它限定短批次的前向搜索范围。 */
std::vector<double> pathArcLengths(const std::vector<OnlineCorrectionPoint> &path)
{
    std::vector<double> lengths(path.size(), 0.0);
    for (std::size_t index = 1U; index < path.size(); ++index) {
        lengths[index] = lengths[index - 1U];
        if (isValid(path[index - 1U]) && isValid(path[index])) {
            lengths[index] += std::sqrt(squaredDistance(path[index - 1U], path[index]));
        }
    }
    return lengths;
}

/** @brief 将同一参考内的连续相位换算为累计弧长，供跨批前进范围比较。 */
double arcLengthAtPhase(const std::vector<double> &lengths, double phase)
{
    const auto index = static_cast<std::size_t>(phase);
    if (index + 1U >= lengths.size()) {
        return lengths.back();
    }
    return lengths[index] + (phase - static_cast<double>(index))
        * (lengths[index + 1U] - lengths[index]);
}

/**
 * @brief 在有序折线上投影，并用连续相位限制搜索起点。
 * @details 使用精确投影端点，避免最近顶点截取把输出扩展到尚未检测的前方。
 */
std::optional<PathProjection> projectToPath(
    const std::vector<OnlineCorrectionPoint> &path,
    const std::vector<double> &lengths, const OnlineCorrectionPoint &point,
    double minimumPhase, double maximumArcLengthMm)
{
    std::optional<PathProjection> projection;
    double nearest = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index + 1U < path.size(); ++index) {
        if (static_cast<double>(index + 1U) < minimumPhase
            || lengths[index] > maximumArcLengthMm
            || !isValid(path[index]) || !isValid(path[index + 1U])) {
            continue;
        }
        const auto &begin = path[index];
        const auto &end = path[index + 1U];
        const double dx = end.xMm - begin.xMm;
        const double dy = end.yMm - begin.yMm;
        const double dz = end.zMm - begin.zMm;
        const double lengthSquared = dx * dx + dy * dy + dz * dz;
        if (lengthSquared <= 1.0e-12) {
            continue;
        }
        const double lower = std::max(0.0, minimumPhase - static_cast<double>(index));
        const double length = std::sqrt(lengthSquared);
        const double upper = std::min(1.0, (maximumArcLengthMm - lengths[index]) / length);
        if (upper < lower) {
            continue;
        }
        const double fraction = std::clamp(
            ((point.xMm - begin.xMm) * dx + (point.yMm - begin.yMm) * dy
                + (point.zMm - begin.zMm) * dz) / lengthSquared, lower, upper);
        OnlineCorrectionPoint candidate{ begin.xMm + fraction * dx,
            begin.yMm + fraction * dy, begin.zMm + fraction * dz, true };
        const double distance = squaredDistance(candidate, point);
        if (distance < nearest) {
            nearest = distance;
            projection = PathProjection{ candidate, static_cast<double>(index) + fraction,
                lengths[index] + fraction * length };
        }
    }
    return projection;
}

vision::WeldSpacePoint spacePoint(const OnlineCorrectionPoint &point)
{
    return { point.xMm, point.yMm, point.zMm, isValid(point) };
}

/** @brief 沿用拷贝工程每三点抽样并补末点，门槛检查必须在抽样之后。 */
std::vector<vision::WeldSpacePoint> sampleWindow(
    const std::vector<OnlineCorrectionDetectionSample> &window)
{
    std::vector<vision::WeldSpacePoint> points;
    for (std::size_t index = 0; index < window.size(); index += 3U) {
        points.push_back(spacePoint(window[index].point));
    }
    if (!window.empty() && (window.size() - 1U) % 3U != 0U) {
        points.push_back(spacePoint(window.back().point));
    }
    return points;
}

/** @brief 独立计算变换前 RMS，用于拒绝非有限或比初值更差的求解结果。 */
double rawRms(
    const std::vector<vision::WeldSpacePoint> &model,
    const std::vector<vision::WeldSpacePoint> &detected)
{
    double sum = 0.0;
    for (const auto &point : detected) {
        double best = std::numeric_limits<double>::max();
        for (const auto &target : model) {
            const double dx = point.xMm - target.xMm;
            const double dy = point.yMm - target.yMm;
            const double dz = point.zMm - target.zMm;
            best = std::min(best, dx * dx + dy * dy + dz * dz);
        }
        sum += best;
    }
    return std::sqrt(sum / static_cast<double>(detected.size()));
}

void appendResult(OnlineCorrectionExceptionResult &result,
                  OnlineCorrectionExceptionResult part)
{
    result.batches.insert(result.batches.end(), part.batches.begin(), part.batches.end());
    result.icpAttempted = result.icpAttempted || part.icpAttempted;
    result.icpApplied = result.icpApplied || part.icpApplied;
    if (part.icpApplied) {
        result.rmsErrorMm = part.rmsErrorMm;
    }
    result.samples.insert(result.samples.end(), part.samples.begin(), part.samples.end());
    result.bufferedSampleCount = part.bufferedSampleCount;
    if (!part.message.empty()) {
        if (!result.message.empty()) {
            result.message += "; ";
        }
        result.message += part.message;
    }
}

} // namespace

OnlineCorrectionExceptionResult COnlineCorrectionExceptionProcessor::processPending(
    const OnlineCorrectionPlannedTrajectory &trajectory, bool flush)
{
    OnlineCorrectionExceptionResult result;
    result.valid = true;
    if (m_pendingSamples.empty()) {
        if (flush) {
            m_normalSamples.clear();
        }
        return result;
    }

    std::vector<OnlineCorrectionDetectionSample> window = m_normalSamples;
    window.insert(window.end(), m_pendingSamples.begin(), m_pendingSamples.end());
    // RobotSim 同帧观测可以没有 seam 元数据；此时用完整当前段，不能锁死第一几何段。
    const bool knownBoundary = m_lastSimulatedSeamIndex >= 0 || m_lastChangePoint.has_value();
    const auto &model = trajectory.pathSegments.empty() || !knownBoundary
        ? trajectory.segmentPathPoints
        : trajectory.pathSegments[std::min(m_curveFlag, trajectory.pathSegments.size() - 1U)];
    std::vector<vision::WeldSpacePoint> modelPoints;
    for (const auto &point : model) {
        if (isValid(point)) {
            modelPoints.push_back(spacePoint(point));
        }
    }
    const auto detected = sampleWindow(window);
    const bool enoughNewPoints = m_normalSamples.empty() || m_pendingSamples.size() >= 3U;
    const bool ready = modelPoints.size() >= MIN_ICP_POINTS
        && detected.size() >= MIN_ICP_POINTS
        && squaredDistance(window.front().point, window.back().point)
            >= MIN_WINDOW_SPAN_MM * MIN_WINDOW_SPAN_MM
        && (flush || squaredDistance(m_pendingSamples.front().point, m_pendingSamples.back().point)
            >= MIN_WINDOW_SPAN_MM * MIN_WINDOW_SPAN_MM)
        && (enoughNewPoints || flush);

    OnlineCorrectionIcpBatchDiagnostic diagnostic;
    diagnostic.flush = flush;
    if (ready) {
        vision::IcpRegistrationOptions options;
        options.dimension = 3;
        options.maxIterations = m_curveFlag == 0U ? 60 : 50;
        options.minDelta = 5.0e-4;
        // 与拷贝单段活动分支一致：检测 -> 示教求解，取逆移动完整示教段。
        // 不启用旧 keepRemoveFailPoint，其实现把坐标为 0 的合法点当作删除标记。
        result.icpAttempted = true;
        const auto match = vision::CIcpRegistrationMatcher::alignPointToPoint(
            modelPoints, detected, options);
        std::array<double, 16> matrix;
        for (int row = 0; row < 4; ++row) {
            for (int column = 0; column < 4; ++column) {
                matrix[static_cast<std::size_t>(row * 4 + column)] =
                    match.transformMatrix(row, column);
            }
        }
        diagnostic.detectionToModel = matrix;
        diagnostic.rmsErrorMm = match.rmsErrorMm;
        const Eigen::Matrix3d rotation = match.transformMatrix.topLeftCorner<3, 3>();
        const bool usable = match.valid && match.transformMatrix.allFinite()
            && std::isfinite(match.rmsErrorMm)
            && (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1.0e-5)
            && std::abs(rotation.determinant() - 1.0) < 1.0e-5
            && match.rmsErrorMm <= rawRms(modelPoints, detected) + 1.0e-4;
        if (usable) {
            const Eigen::Matrix4d inverse = match.transformMatrix.inverse();
            std::vector<OnlineCorrectionPoint> transformed;
            for (const auto &point : model) {
                const Eigen::Vector4d target = inverse
                    * Eigen::Vector4d(point.xMm, point.yMm, point.zMm, 1.0);
                transformed.push_back({ target.x(), target.y(), target.z(),
                                        isValid(point) && target.allFinite() });
            }
            const auto lengths = pathArcLengths(model);
            double maximumStepMm = 0.0;
            for (std::size_t index = 1U; index < lengths.size(); ++index) {
                maximumStepMm = std::max(maximumStepMm, lengths[index] - lengths[index - 1U]);
            }
            double pendingLengthMm = 0.0;
            for (std::size_t index = 1U; index < m_pendingSamples.size(); ++index) {
                pendingLengthMm += std::sqrt(squaredDistance(
                    m_pendingSamples[index - 1U].point, m_pendingSamples[index].point));
            }
            const double bridgeMm = m_lastReleasedRawPoint.has_value()
                ? std::sqrt(squaredDistance(*m_lastReleasedRawPoint, m_pendingSamples.front().point))
                : 0.0;
            const double minimumPhase = m_lastReleasedPhase.value_or(0.0);
            // 两个采样间距仅容纳离散模型和跨批空档，不是配准精度阈值。
            // 上界从已释放相位计算，阻止近闭环上的短检测批次跳到远端或重放整圈。
            const double forwardBudgetMm = pendingLengthMm + bridgeMm + 2.0 * maximumStepMm;
            // 先在剩余模型中定位本批，不把受限窗口端点硬当成最近点。
            // 若新变换造成模型相位大幅滑移，保留原始批并可靠重定位游标，避免永久停滞。
            const auto first = projectToPath(transformed, lengths,
                m_pendingSamples.front().point, minimumPhase, lengths.back());
            const auto last = first.has_value()
                ? projectToPath(transformed, lengths, m_pendingSamples.back().point,
                    first->phase, std::min(lengths.back(), first->arcLengthMm + forwardBudgetMm))
                : std::nullopt;
            // 离散模型间距和拟合残差只用来拒绝与本批不相容的截段，不声明实际精度。
            const double endpointToleranceMm = std::max(1.0e-4,
                maximumStepMm + 3.0 * match.rmsErrorMm);
            const bool endpointsCompatible = first.has_value() && last.has_value()
                && squaredDistance(first->point, m_pendingSamples.front().point)
                    <= endpointToleranceMm * endpointToleranceMm
                && squaredDistance(last->point, m_pendingSamples.back().point)
                    <= endpointToleranceMm * endpointToleranceMm;
            const bool forwardRangeCompatible = !m_lastReleasedPhase.has_value()
                || (last.has_value() && last->arcLengthMm
                    <= arcLengthAtPhase(lengths, minimumPhase) + forwardBudgetMm + 1.0e-9);
            // 只输出本批端点投影之间的折线，历史只拟合，不扩格也不当作闭环。
            if (endpointsCompatible && last->phase > first->phase + 1.0e-9
                && !forwardRangeCompatible) {
                m_lastReleasedPhase = last->phase;
                result.message = "online ICP phase shift; raw pending samples retained and cursor reanchored";
            } else if (endpointsCompatible && last->phase > first->phase + 1.0e-9) {
                std::vector<OnlineCorrectionPoint> batchPath;
                if (!m_lastReleasedPhase.has_value() || first->phase > minimumPhase + 1.0e-9) {
                    batchPath.push_back(first->point);
                }
                for (std::size_t index = 0; index < transformed.size(); ++index) {
                    const double phase = static_cast<double>(index);
                    if (phase > first->phase + 1.0e-9 && phase < last->phase - 1.0e-9) {
                        batchPath.push_back(transformed[index]);
                    }
                }
                batchPath.push_back(last->point);
                for (const auto &point : batchPath) {
                    if (!isValid(point)) {
                        result.samples.clear();
                        break;
                    }
                    auto sample = m_pendingSamples.front();
                    double nearest = std::numeric_limits<double>::max();
                    for (const auto &raw : m_pendingSamples) {
                        const double distance = squaredDistance(point, raw.point);
                        if (distance < nearest) {
                            nearest = distance;
                            sample = raw;
                        }
                    }
                    sample.point = point;
                    // 模型重采样不与某次曝光一一对应，不能把最近原始样本当来源。
                    sample.sourceExposure.reset();
                    sample.scanLineChangePoint.reset();
                    result.samples.push_back(sample);
                }
                result.icpApplied = !result.samples.empty();
                result.rmsErrorMm = match.rmsErrorMm;
                if (result.icpApplied) {
                    m_lastReleasedPhase = last->phase;
                }
            }
        }
        if (result.message.empty()) {
            result.message = result.icpApplied ? "online ICP applied"
                : "online ICP rejected; raw pending samples retained";
        }
    }

    const bool release = ready || flush || window.size() >= MAX_WINDOW_POINTS
        || modelPoints.size() < MIN_ICP_POINTS;
    if (!release) {
        result.bufferedSampleCount = m_pendingSamples.size();
        result.message = "online ICP waiting for five sampled points, 2 mm span and new batch";
        return result;
    }
    if (!result.icpApplied) {
        result.samples = m_pendingSamples;
        if (result.message.empty()) {
            result.message = modelPoints.size() < MIN_ICP_POINTS
                ? "online ICP reference insufficient; raw samples retained"
                : (flush ? "online ICP tail insufficient; raw samples retained"
                         : "online ICP window limit; raw samples retained");
        }
    }
    diagnostic.attempted = result.icpAttempted;
    diagnostic.applied = result.icpApplied;
    diagnostic.pendingSamples = m_pendingSamples;
    diagnostic.releasedSamples = result.samples;
    diagnostic.modelPoints = model;
    diagnostic.message = result.message;
    for (const auto &point : detected) {
        diagnostic.fitPoints.push_back({point.xMm, point.yMm, point.zMm, true});
    }
    result.batches.push_back(std::move(diagnostic));
    m_lastReleasedRawPoint = m_pendingSamples.back().point;
    m_pendingSamples.clear();
    if (flush) {
        m_normalSamples.clear();
    } else {
        // 拷贝工程保留批次后 80%；当前每帧只有一点，必须保留完整累计窗口的后 80%。
        const std::size_t retainedBegin = std::max(window.size() / 5U,
            window.size() > MAX_WINDOW_POINTS ? window.size() - MAX_WINDOW_POINTS : 0U);
        m_normalSamples.assign(window.begin() + static_cast<std::ptrdiff_t>(retainedBegin), window.end());
    }
    return result;
}

OnlineCorrectionExceptionResult COnlineCorrectionExceptionProcessor::process(
    const std::vector<OnlineCorrectionDetectionSample> &newSamples,
    const OnlineCorrectionPlannedTrajectory &trajectory, bool flush)
{
    OnlineCorrectionExceptionResult result;
    result.valid = true;
    for (const auto &sample : newSamples) {
        if (!isValid(sample.point)) {
            continue;
        }
        std::vector<OnlineCorrectionDetectionSample> bypassSamples;
        if (sample.estimated) {
            // 预测边界直接释放未完成的实测窗口；flush 可能触发 ICP，不能用于隔离。
            bypassSamples.swap(m_pendingSamples);
            m_normalSamples.clear();
        }
        const bool seamEdge = sample.simulatedSeamIndex >= 0
            && m_lastSimulatedSeamIndex >= 0
            && sample.simulatedSeamIndex != m_lastSimulatedSeamIndex;
        const bool changePointEdge = sample.scanLineChangePoint.has_value()
            && isValid(*sample.scanLineChangePoint)
            && (!m_lastChangePoint.has_value()
                || squaredDistance(*sample.scanLineChangePoint, *m_lastChangePoint) > 1.0e-12);
        const bool referenceBecameKnown = m_lastSimulatedSeamIndex < 0
            && !m_lastChangePoint.has_value()
            && (sample.simulatedSeamIndex >= 0
                || (sample.scanLineChangePoint.has_value() && isValid(*sample.scanLineChangePoint)));
        if (seamEdge || changePointEdge || referenceBecameKnown) {
            // 两侧分别处理。旧侧不足时原样释放，新侧继续累计，避免一侧成功吞掉另一侧。
            appendResult(result, processPending(trajectory, true));
            if ((seamEdge || changePointEdge) && !referenceBecameKnown
                && m_curveFlag + 1U < trajectory.pathSegments.size()) {
                ++m_curveFlag;
            }
            m_lastReleasedPhase.reset();
            m_lastReleasedRawPoint.reset();
            result.segmentChanged = seamEdge || (changePointEdge && !referenceBecameKnown);
        }
        if (sample.simulatedSeamIndex >= 0) {
            m_lastSimulatedSeamIndex = sample.simulatedSeamIndex;
        }
        if (sample.scanLineChangePoint.has_value() && isValid(*sample.scanLineChangePoint)) {
            m_lastChangePoint = sample.scanLineChangePoint;
        }
        if (sample.estimated) {
            bypassSamples.push_back(sample);
            OnlineCorrectionExceptionResult bypass;
            bypass.valid = true;
            bypass.samples = std::move(bypassSamples);
            bypass.message = "estimated seam continuation bypasses ICP; measured window isolated";
            OnlineCorrectionIcpBatchDiagnostic diagnostic;
            diagnostic.flush = true;
            diagnostic.pendingSamples = bypass.samples;
            diagnostic.releasedSamples = bypass.samples;
            diagnostic.message = bypass.message;
            bypass.batches.push_back(std::move(diagnostic));
            appendResult(result, std::move(bypass));
            // 保留单调相位约束，但推进跨批距离锚点，预测不进入拟合历史。
            m_lastReleasedRawPoint = sample.point;
            continue;
        }
        m_pendingSamples.push_back(sample);
        // 限制坏输入/静止相机窗口的内存，不让未满足条件的样本无限增长。
        if (m_pendingSamples.size() + m_normalSamples.size() >= MAX_WINDOW_POINTS) {
            appendResult(result, processPending(trajectory, false));
        }
    }
    appendResult(result, processPending(trajectory, flush));
    return result;
}

OnlineCorrectionExceptionResult COnlineCorrectionExceptionProcessor::process(
    const std::vector<OnlineCorrectionDetectionSample> &newSamples,
    const OnlineCorrectionPlannedTrajectory &trajectory)
{
    return process(newSamples, trajectory, false);
}

OnlineCorrectionExceptionResult COnlineCorrectionExceptionProcessor::process(
    const std::vector<OnlineCorrectionDetectionSample> &newSamples,
    const std::vector<OnlineCorrectionPoint> &referenceSegment)
{
    OnlineCorrectionPlannedTrajectory trajectory;
    trajectory.segmentPathPoints = referenceSegment;
    return process(newSamples, trajectory, false);
}

void COnlineCorrectionExceptionProcessor::reset()
{
    m_normalSamples.clear();
    m_pendingSamples.clear();
    m_lastSimulatedSeamIndex = -1;
    m_curveFlag = 0U;
    m_lastChangePoint.reset();
    m_lastReleasedPhase.reset();
    m_lastReleasedRawPoint.reset();
}

} // namespace i2r_rlc::control
