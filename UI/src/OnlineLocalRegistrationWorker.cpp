/** @file OnlineLocalRegistrationWorker.cpp
 * @brief 在线局部表面点云采样、配准和小幅修正验收。
 */
#include "OnlineLocalRegistrationWorker.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

#include <Eigen/Eigenvalues>

#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QUuid>

#include "LocalizationPipelineMath.h"
#include "RuntimeWorkerAdapters.h"

namespace i2r_rlc::ui {
namespace {

using Point = localization::Point3D_mm;

bool finitePoint(const Point &point)
{
    return std::isfinite(point.x_mm) && std::isfinite(point.y_mm)
        && std::isfinite(point.z_mm);
}

// 等距取样不依赖点云库；每帧及整个窗口均有独立上限。
std::vector<Point> boundedPoints(const std::vector<Point> &points, int limit)
{
    std::vector<Point> finite;
    finite.reserve(points.size());
    for (const auto &point : points) {
        if (finitePoint(point)) { finite.push_back(point); }
    }
    const std::size_t maximum = static_cast<std::size_t>(limit);
    if (finite.size() <= maximum) { return finite; }
    std::vector<Point> sampled;
    sampled.reserve(maximum);
    for (std::size_t index = 0; index < maximum; ++index) {
        sampled.push_back(finite[index * (finite.size() - 1U) / (maximum - 1U)]);
    }
    return sampled;
}

bool observableSurface(const std::vector<Point> &points)
{
    if (points.size() < 10U) { return false; }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &p : points) { mean += Eigen::Vector3d(p.x_mm, p.y_mm, p.z_mm); }
    mean /= static_cast<double>(points.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &p : points) {
        const Eigen::Vector3d delta = Eigen::Vector3d(p.x_mm, p.y_mm, p.z_mm) - mean;
        covariance += delta * delta.transpose();
    }
    covariance /= static_cast<double>(points.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) { return false; }
    const auto values = solver.eigenvalues();
    // 这里只能排除近共线/单点；平面通过不代表切向平移或旋转具有真实约束。
    return values.allFinite() && values[1] >= 0.01
        && values[1] >= values[2] * 1e-4;
}

double pointDistance(const Point &lhs, const Point &rhs)
{
    return Eigen::Vector3d(lhs.x_mm - rhs.x_mm, lhs.y_mm - rhs.y_mm,
        lhs.z_mm - rhs.z_mm).norm();
}

/** @brief 以完整云主方向衡量匹配区域跨度，防止只有一小片区域支撑大幅移动。 */
double matchedSpread(const std::vector<Point> &all, const std::vector<Point> &matched)
{
    if (all.size() < 3U || matched.size() < 3U) { return 0.0; }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &p : all) { mean += Eigen::Vector3d(p.x_mm, p.y_mm, p.z_mm); }
    mean /= static_cast<double>(all.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &p : all) {
        const Eigen::Vector3d delta = Eigen::Vector3d(p.x_mm, p.y_mm, p.z_mm) - mean;
        covariance += delta * delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) { return 0.0; }
    double ratio = 1.0;
    for (int axis = 1; axis <= 2; ++axis) {
        const Eigen::Vector3d direction = solver.eigenvectors().col(axis);
        const auto span = [&direction](const std::vector<Point> &points) {
            double low = std::numeric_limits<double>::infinity();
            double high = -std::numeric_limits<double>::infinity();
            for (const auto &p : points) {
                const double value = direction.dot(Eigen::Vector3d(p.x_mm, p.y_mm, p.z_mm));
                low = std::min(low, value);
                high = std::max(high, value);
            }
            return high - low;
        };
        const double fullSpan = span(all);
        if (fullSpan <= 1e-6) { return 0.0; }
        ratio = std::min(ratio, span(matched) / fullSpan);
    }
    return ratio;
}

/** @brief 双向最近邻分别统计，避免一小片真实点被大型虚拟面单向包含。 */
double coveredPoints(const std::vector<Point> &source, const std::vector<Point> &target,
    double distanceMm, std::vector<Point> &matched)
{
    for (const auto &point : source) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &other : target) {
            nearest = std::min(nearest, pointDistance(point, other));
        }
        if (nearest <= distanceMm) { matched.push_back(point); }
    }
    return source.empty() ? 0.0 : static_cast<double>(matched.size()) / source.size();
}

QJsonArray pointJson(const Point &point)
{
    // 非有限原始值保留为文本，避免 JSON 的 null 混淆缺失数据与原始坏点。
    QJsonArray values;
    for (double value : {point.x_mm, point.y_mm, point.z_mm}) {
        values.append(std::isfinite(value) ? QJsonValue(value)
            : QJsonValue(QString::number(value)));
    }
    return values;
}

QJsonArray cloudJson(const std::vector<Point> &points)
{
    QJsonArray values;
    for (const auto &point : points) { values.append(pointJson(point)); }
    return values;
}

/** @brief 最终候选的诊断最近邻；不声称是 PCL 迭代内部使用的对应关系。 */
QJsonArray diagnosticCorrespondences(const std::vector<Point> &source,
    const std::vector<Point> &target, double coverageDistanceMm)
{
    QJsonArray values;
    for (std::size_t sourceIndex = 0; sourceIndex < source.size(); ++sourceIndex) {
        double nearest = std::numeric_limits<double>::infinity();
        int targetIndex = -1;
        for (std::size_t index = 0; index < target.size(); ++index) {
            const double distance = pointDistance(source[sourceIndex], target[index]);
            if (distance < nearest) {
                nearest = distance;
                targetIndex = static_cast<int>(index);
            }
        }
        values.append(QJsonObject{
            {QStringLiteral("source_index"), static_cast<int>(sourceIndex)},
            {QStringLiteral("target_index"), targetIndex},
            {QStringLiteral("distance_mm"), std::isfinite(nearest)
                ? QJsonValue(nearest) : QJsonValue(QStringLiteral("nonfinite"))},
            {QStringLiteral("inside_coverage"), nearest <= coverageDistanceMm}});
    }
    return values;
}

QJsonObject independentJson(
    const COnlineLocalRegistrationWorker::IndependentObservation &observation)
{
    return {{QStringLiteral("valid"), observation.valid},
        {QStringLiteral("frame_id"), QString::number(observation.frameId)},
        {QStringLiteral("real_weld_parent_mm"), pointJson(observation.realWeldPointInParent_mm)},
        {QStringLiteral("virtual_weld_parent_mm"), pointJson(observation.virtualWeldPointInParent_mm)}};
}

} // namespace

COnlineLocalRegistrationWorker::COnlineLocalRegistrationWorker(QObject *parent)
    : QObject(parent), m_pipeline(m_service)
{
    m_clock.start();
}

void COnlineLocalRegistrationWorker::configure(const Options &options)
{
    m_options = options;
    m_options.sampleIntervalMs = std::clamp(options.sampleIntervalMs, 0, 60000);
    m_options.minimumMatchIntervalMs = std::clamp(options.minimumMatchIntervalMs, 0, 60000);
    m_options.minimumFrames = std::clamp(options.minimumFrames, 1, 64);
    m_options.maximumFrames = std::clamp(options.maximumFrames, m_options.minimumFrames, 64);
    m_options.maximumPointsPerCloud = std::clamp(options.maximumPointsPerCloud, 10, 2048);
    m_options.maximumIterations = std::clamp(options.maximumIterations, 1, 100);
    m_options.maximumCorrespondenceDistanceMm = std::isfinite(options.maximumCorrespondenceDistanceMm)
        ? std::clamp(options.maximumCorrespondenceDistanceMm, 0.1, 10.0) : 10.0;
    m_options.maximumTranslationMm = std::isfinite(options.maximumTranslationMm)
        ? std::clamp(options.maximumTranslationMm, 0.01, 30.0) : 30.0;
    m_options.maximumSolveTimeMs = std::clamp(options.maximumSolveTimeMs, 1, 2500);
    const auto normalized = [](double value, double low, double high, double fallback) {
        return std::isfinite(value) ? std::clamp(value, low, high) : fallback;
    };
    m_options.maximumIndependentErrorMm = normalized(options.maximumIndependentErrorMm, 0.01, 10.0, 5.5);
    m_options.maximumIndependentWorseningMm = normalized(options.maximumIndependentWorseningMm, 0.0, 1.0, 0.1);
    m_options.coverageDistanceMm = normalized(options.coverageDistanceMm, 0.01, 10.0, 3.0);
    m_options.minimumBidirectionalCoverage = normalized(options.minimumBidirectionalCoverage, 0.1, 1.0, 0.7);
    m_options.minimumMatchedSpreadRatio = normalized(options.minimumMatchedSpreadRatio, 0.1, 1.0, 0.5);
    m_options.maximumArchivedAttemptsPerSession = std::clamp(options.maximumArchivedAttemptsPerSession, 1, 1024);
    m_options.maximumRawPointsPerFrame = std::clamp(options.maximumRawPointsPerFrame, 10, 65536);
    reset();
}

void COnlineLocalRegistrationWorker::reset()
{
    m_frames.clear();
    m_sessionId = 0;
    m_generation = 0;
    m_lastFrameId = -1;
    m_lastSampleMs = -1;
    m_lastMatchMs = -1;
    m_clock.restart();
}

COnlineLocalRegistrationWorker::Result COnlineLocalRegistrationWorker::process(const Input &input)
{
    QElapsedTimer timer;
    timer.start();
    Result result;
    result.sessionId = input.sessionId;
    result.generation = input.generation;
    result.frameId = input.frame.frameId;
    if (!m_options.enabled || input.sessionId == 0
        || !isUsableCorrectionObservationForFrame(input.frame.frameId, input.observation)
        || !input.trackingOptions.positionerCalibration.valid) {
        result.message = QStringLiteral("局部定位输入缺少同帧参考或有效变位机标定。");
        return result;
    }
    if (!input.independentObservation.valid
        || input.independentObservation.frameId != input.frame.frameId
        || !finitePoint(input.independentObservation.realWeldPointInParent_mm)
        || !finitePoint(input.independentObservation.virtualWeldPointInParent_mm)
        || input.archiveDirectory.trimmed().isEmpty()) {
        // 独立检测缺失时不积累旧窗；不能把仿真融合点当作独立真实点替代。
        reset();
        result.message = QStringLiteral("独立焊点观测或局部配准归档目录无效，暂停更新并清窗。");
        return result;
    }

    try {
        RuntimeTrackingOptions options = input.trackingOptions;
        options.globalLocateLineExtraction = true;
        options.requireDetectionWindow = false;
        // 在线会话窗口由 coordinator 门控；保留播放/保持/启动真实状态门禁。
        options.testStart = 0;
        options.detType = 2;
        options.visionDiagnostics.enabled = false;
        CVisionTrackingWorkerAdapter extractor;
        RuntimeTrackingResult extracted;
        QObject::connect(&extractor, &CVisionTrackingWorkerAdapter::trackingResultReady,
            &extractor, [&extracted](const RuntimeTrackingResult &value) { extracted = value; },
            Qt::DirectConnection);
        extractor.configure(options, input.projectionContext);
        extractor.startTracking();
        RuntimeVisionCorrelation correlation;
        correlation.frameId = input.frame.frameId;
        auto observation = input.observation;
        observation.positioningSegmentIndex = -1;
        extractor.submitFrameWithRobotStateObservationAndCorrelation(
            input.frame, input.robotState, -1, observation, correlation);
        if (!extracted.valid || !extracted.triggerAccepted) {
            result.message = QStringLiteral("局部真实表面提取跳过：%1").arg(extracted.message);
            result.elapsedMs = timer.elapsed();
            return result;
        }
        std::vector<Point> realPoints;
        realPoints.reserve(extracted.linePoints.size());
        // 提取结果已在变位机 TCP 系；工件父级 1063 本身包含末节到 TCP 的偏移。
        // 与该父级参考下的仿真点直接匹配，不能再加 toolTcpInLastLinkMm。
        for (const auto &p : extracted.linePoints) {
            if (p.valid) {
                realPoints.push_back({p.xMm, p.yMm, p.zMm});
            }
        }
        std::vector<Point> virtualPoints;
        virtualPoints.reserve(observation.laserLinePoints_mm.size());
        for (const auto &p : observation.laserLinePoints_mm) {
            virtualPoints.push_back({p.x_mm, p.y_mm, p.z_mm});
        }
        CloudValidationContext validation;
        validation.independentObservation = input.independentObservation;
        validation.archiveDirectory = input.archiveDirectory;
        result = processClouds(input.sessionId, input.generation, input.frame.frameId,
            realPoints, virtualPoints, validation);
        result.elapsedMs = timer.elapsed();
        return result;
    } catch (const std::exception &error) {
        result.message = QStringLiteral("局部表面提取异常：%1")
            .arg(QString::fromUtf8(error.what()));
        result.elapsedMs = timer.elapsed();
        return result;
    }
}

COnlineLocalRegistrationWorker::Result COnlineLocalRegistrationWorker::processClouds(
    quint64 sessionId, quint64 generation, qint64 frameId,
    const std::vector<Point> &realPointsInParent_mm,
    const std::vector<Point> &virtualPointsInParent_mm)
{
    CloudValidationContext validation;
    validation.requireIndependentObservation = false;
    return processClouds(sessionId, generation, frameId,
        realPointsInParent_mm, virtualPointsInParent_mm, validation);
}

COnlineLocalRegistrationWorker::Result COnlineLocalRegistrationWorker::processClouds(
    quint64 sessionId, quint64 generation, qint64 frameId,
    const std::vector<Point> &realPointsInParent_mm,
    const std::vector<Point> &virtualPointsInParent_mm,
    const CloudValidationContext &validation)
{
    QElapsedTimer timer;
    timer.start();
    Result result;
    result.sessionId = sessionId;
    result.generation = generation;
    result.frameId = frameId;
    result.independentObservation = validation.independentObservation;
    if (!m_options.enabled || sessionId == 0) { return result; }
    if (validation.requireIndependentObservation
        && (!validation.independentObservation.valid
            || validation.independentObservation.frameId != frameId
            || !finitePoint(validation.independentObservation.realWeldPointInParent_mm)
            || !finitePoint(validation.independentObservation.virtualWeldPointInParent_mm)
            || validation.archiveDirectory.trimmed().isEmpty())) {
        reset();
        result.message = QStringLiteral("独立观测不完整、不同帧或归档目录缺失，拒绝并清窗。");
        return result;
    }
    if (realPointsInParent_mm.size() > static_cast<std::size_t>(m_options.maximumRawPointsPerFrame)
        || virtualPointsInParent_mm.size() > static_cast<std::size_t>(m_options.maximumRawPointsPerFrame)) {
        reset();
        result.message = QStringLiteral("原始局部点云超出归档预算，拒绝更新并清窗。");
        return result;
    }
    if (sessionId != m_sessionId || generation != m_generation) {
        reset();
        m_sessionId = sessionId;
        m_generation = generation;
    }
    const qint64 now = m_clock.elapsed();
    if (frameId <= m_lastFrameId
        || (m_lastSampleMs >= 0 && now - m_lastSampleMs < m_options.sampleIntervalMs)) {
        return result;
    }
    m_lastFrameId = frameId;
    CloudFrame frame;
    frame.frameId = frameId;
    frame.rawReal = realPointsInParent_mm;
    frame.rawReference = virtualPointsInParent_mm;
    frame.independentObservation = validation.independentObservation;
    frame.real = boundedPoints(realPointsInParent_mm, m_options.maximumPointsPerCloud);
    frame.reference = boundedPoints(virtualPointsInParent_mm, m_options.maximumPointsPerCloud);
    frame.capturedMs = now;
    if (frame.real.size() < 10U || frame.reference.size() < 10U) {
        result.message = QStringLiteral("局部定位有效表面点不足。");
        return result;
    }
    m_lastSampleMs = now;
    m_frames.push_back(std::move(frame));
    const qint64 maximumAgeMs = std::max<qint64>(5000,
        static_cast<qint64>(m_options.minimumFrames) * m_options.sampleIntervalMs * 4);
    while (m_frames.size() > static_cast<std::size_t>(m_options.maximumFrames)
        || (!m_frames.empty() && now - m_frames.front().capturedMs > maximumAgeMs)) {
        m_frames.pop_front();
    }
    result.frameCount = static_cast<int>(m_frames.size());
    result.pointCount = static_cast<int>(std::max(
        m_frames.back().real.size(), m_frames.back().reference.size()));
    if (result.frameCount < m_options.minimumFrames
        || (m_lastMatchMs >= 0 && now - m_lastMatchMs < m_options.minimumMatchIntervalMs)) {
        return result;
    }
    result.attempted = true;
    m_lastMatchMs = now;
    localization::LocalCorrectionPipelineRequest request;
    request.frameId = frameId;
    request.windowFrameCount = result.frameCount;
    request.minimumWindowFrameCount = m_options.minimumFrames;
    request.dofMask = {true, true, true, false, false, true};
    request.parameters.maximumIterations = m_options.maximumIterations;
    request.parameters.maxCorrespondenceDistance_mm = m_options.maximumCorrespondenceDistanceMm;
    request.parameters.enableSorFilter = true;
    for (const auto &item : m_frames) {
        request.realWindowPoints_mm.insert(request.realWindowPoints_mm.end(), item.real.begin(), item.real.end());
        request.virtualWindowPoints_mm.insert(request.virtualWindowPoints_mm.end(), item.reference.begin(), item.reference.end());
    }
    // 每次求解后重新累计；保留本次有界原始窗口供归档，绝不重复作为下一次输入。
    std::deque<CloudFrame> attemptedFrames;
    attemptedFrames.swap(m_frames);
    request.realWindowPoints_mm = boundedPoints(request.realWindowPoints_mm, m_options.maximumPointsPerCloud);
    request.virtualWindowPoints_mm = boundedPoints(request.virtualWindowPoints_mm, m_options.maximumPointsPerCloud);
    result.pointCount = static_cast<int>(std::max(request.realWindowPoints_mm.size(), request.virtualWindowPoints_mm.size()));
    if (!observableSurface(request.realWindowPoints_mm)
        || !observableSurface(request.virtualWindowPoints_mm)) {
        result.message = QStringLiteral("局部点云近共线或覆盖不足，拒绝工件修正。");
        archiveAttempt(attemptedFrames, request, {}, validation, result);
        result.elapsedMs = timer.elapsed();
        return result;
    }
    QElapsedTimer solveTimer;
    solveTimer.start();
    std::vector<Point> preprocessedRealPoints;
    try {
        const auto solved = m_pipeline.run(request);
        preprocessedRealPoints = solved.icpResult.preprocessedRealPoints_mm;
        result.solveElapsedMs = solveTimer.elapsed();
        result.beforeResidualMm = solved.beforeResidual_mm;
        result.afterResidualMm = solved.afterResidual_mm;
        result.correction = solved.virtualToRealCorrectionTransform;
        const auto &matrix = result.correction.rowMajor;
        const bool finite = std::all_of(matrix.begin(), matrix.end(),
            [](double value) { return std::isfinite(value); });
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << matrix[0], matrix[1], matrix[2],
            matrix[4], matrix[5], matrix[6], matrix[8], matrix[9], matrix[10];
        const bool rigid = finite
            && std::abs(matrix[12]) <= 1e-6 && std::abs(matrix[13]) <= 1e-6
            && std::abs(matrix[14]) <= 1e-6 && std::abs(matrix[15] - 1.0) <= 1e-6
            && (rotationMatrix.transpose() * rotationMatrix - Eigen::Matrix3d::Identity()).norm() <= 1e-4
            && std::abs(rotationMatrix.determinant() - 1.0) <= 1e-4;
        const double translation = std::sqrt(matrix[3] * matrix[3]
            + matrix[7] * matrix[7] + matrix[11] * matrix[11]);
        const double rotation = std::acos(std::clamp(
            (matrix[0] + matrix[5] + matrix[10] - 1.0) * 0.5, -1.0, 1.0))
            * 180.0 / 3.14159265358979323846;
        result.accepted = solved.status == localization::LocalizationStatus::Succeeded
            && solved.icpResult.hasPoseCorrection && rigid
            && std::isfinite(result.beforeResidualMm) && std::isfinite(result.afterResidualMm)
            && result.beforeResidualMm > 1e-6
            && result.afterResidualMm <= m_options.maximumResidualMm
            && result.afterResidualMm <= result.beforeResidualMm * (1.0 - m_options.minimumImprovementRatio)
            && translation <= m_options.maximumTranslationMm
            && rotation <= m_options.maximumRotationDeg
            && result.solveElapsedMs <= m_options.maximumSolveTimeMs;
        result.message = result.accepted
            ? QStringLiteral("局部表面配准通过小幅修正验收。")
            : QStringLiteral("局部配准未通过质量、幅度或耗时验收：%1；前=%2，后=%3 mm，平移=%4 mm，旋转=%5 度，求解=%6 ms。")
                .arg(solved.message).arg(result.beforeResidualMm).arg(result.afterResidualMm)
                .arg(translation).arg(rotation).arg(result.solveElapsedMs);
        if (solved.status == localization::LocalizationStatus::Succeeded
            && solved.hasCorrectedVirtualPoints && finite) {
            std::vector<Point> matchedReal;
            std::vector<Point> matchedVirtual;
            result.realCoverageRatio = coveredPoints(request.realWindowPoints_mm,
                solved.correctedVirtualWindowPoints_mm, m_options.coverageDistanceMm, matchedReal);
            result.virtualCoverageRatio = coveredPoints(solved.correctedVirtualWindowPoints_mm,
                request.realWindowPoints_mm, m_options.coverageDistanceMm, matchedVirtual);
            result.matchedSpreadRatio = std::min(
                matchedSpread(request.realWindowPoints_mm, matchedReal),
                matchedSpread(solved.correctedVirtualWindowPoints_mm, matchedVirtual));
            if (result.realCoverageRatio < m_options.minimumBidirectionalCoverage
                || result.virtualCoverageRatio < m_options.minimumBidirectionalCoverage
                || result.matchedSpreadRatio < m_options.minimumMatchedSpreadRatio
                || !observableSurface(matchedReal) || !observableSurface(matchedVirtual)) {
                result.accepted = false;
                result.message = QStringLiteral("双向覆盖或匹配区域不足，拒绝更新：真实覆盖=%1，虚拟覆盖=%2，匹配跨度=%3。")
                    .arg(result.realCoverageRatio).arg(result.virtualCoverageRatio)
                    .arg(result.matchedSpreadRatio);
            }
        }
        if (validation.requireIndependentObservation) {
            // KCF 与表面提取是不同观测，但也可能丢失或误跟踪，只能用于否决候选。
            // 逐帧检查，禁止均值掩盖个别明显恶化；不参与 ICP 求解。
            for (const auto &item : attemptedFrames) {
                const auto &independent = item.independentObservation;
                const auto corrected = localization::pipeline_math::transformPoints(
                    {independent.virtualWeldPointInParent_mm}, result.correction);
                const double before = pointDistance(independent.realWeldPointInParent_mm,
                    independent.virtualWeldPointInParent_mm);
                const double after = pointDistance(independent.realWeldPointInParent_mm,
                    corrected.front());
                result.beforeIndependentErrorMm = std::max(result.beforeIndependentErrorMm, before);
                result.afterIndependentErrorMm = std::max(result.afterIndependentErrorMm, after);
                if (!independent.valid || !std::isfinite(after)
                    || after > m_options.maximumIndependentErrorMm
                    || after > before + m_options.maximumIndependentWorseningMm) {
                    result.accepted = false;
                    result.message = QStringLiteral("独立焊点误差恶化或超限，拒绝更新：frame=%1，前=%2 mm，后=%3 mm。")
                        .arg(item.frameId).arg(before).arg(after);
                }
            }
        }
    } catch (const std::exception &error) {
        result.accepted = false;
        result.solveElapsedMs = solveTimer.elapsed();
        result.message = QStringLiteral("局部配准异常：%1").arg(QString::fromUtf8(error.what()));
    }
    archiveAttempt(attemptedFrames, request, preprocessedRealPoints, validation, result);
    result.elapsedMs = timer.elapsed();
    return result;
}

bool COnlineLocalRegistrationWorker::archiveAttempt(const std::deque<CloudFrame> &frames,
    const localization::LocalCorrectionPipelineRequest &request,
    const std::vector<Point> &preprocessedRealPoints,
    const CloudValidationContext &validation, Result &result)
{
    // 合成兼容入口不写盘；生产入口及显式提供目录的离线验证都必须成功归档。
    if (!validation.requireIndependentObservation && validation.archiveDirectory.isEmpty()) {
        return true;
    }
    if (m_archiveSessionId != result.sessionId) {
        m_archiveSessionId = result.sessionId;
        m_archiveAttemptCount = 0;
    }
    const auto fail = [&result](const QString &reason) {
        result.accepted = false;
        result.archiveSucceeded = false;
        result.message += QStringLiteral(" 局部配准归档失败，拒绝更新：%1").arg(reason);
        return false;
    };
    if (m_archiveAttemptCount >= m_options.maximumArchivedAttemptsPerSession) {
        return fail(QStringLiteral("本会话归档预算已用尽，需新会话后继续。"));
    }
    ++m_archiveAttemptCount;
    QDir directory(validation.archiveDirectory);
    if (!directory.exists() && !directory.mkpath(QStringLiteral("."))) {
        return fail(QStringLiteral("无法创建归档目录。"));
    }
    QJsonArray rawFrames;
    for (const auto &frame : frames) {
        rawFrames.append(QJsonObject{
            {QStringLiteral("frame_id"), QString::number(frame.frameId)},
            {QStringLiteral("captured_ms"), QString::number(frame.capturedMs)},
            {QStringLiteral("raw_real_parent_mm"), cloudJson(frame.rawReal)},
            {QStringLiteral("raw_virtual_parent_mm"), cloudJson(frame.rawReference)},
            {QStringLiteral("independent"), independentJson(frame.independentObservation)}});
    }
    QJsonArray matrix;
    for (double value : result.correction.rowMajor) { matrix.append(value); }
    const auto correctedVirtual = localization::pipeline_math::transformPoints(
        request.virtualWindowPoints_mm, result.correction);
    QJsonObject record{
        {QStringLiteral("schema_version"), 1},
        {QStringLiteral("session_id"), QString::number(result.sessionId)},
        {QStringLiteral("generation"), QString::number(result.generation)},
        {QStringLiteral("frame_id"), QString::number(result.frameId)},
        {QStringLiteral("coordinate_frame"), QStringLiteral("workpiece_parent_tcp_mm")},
        {QStringLiteral("dof_xyz_rx_ry_rz"), QJsonArray{true, true, true, false, false, true}},
        {QStringLiteral("raw_frames"), rawFrames},
        {QStringLiteral("input_real_window_parent_mm"), cloudJson(request.realWindowPoints_mm)},
        {QStringLiteral("solve_real_parent_mm"), cloudJson(preprocessedRealPoints)},
        {QStringLiteral("solve_virtual_parent_mm"), cloudJson(request.virtualWindowPoints_mm)},
        {QStringLiteral("corrected_virtual_parent_mm"), cloudJson(correctedVirtual)},
        {QStringLiteral("correspondence_semantics"),
            QStringLiteral("final_candidate_diagnostic_nearest_neighbors_not_pcl_iteration_correspondences")},
        {QStringLiteral("diagnostic_real_to_corrected_virtual"), diagnosticCorrespondences(
            request.realWindowPoints_mm, correctedVirtual, m_options.coverageDistanceMm)},
        {QStringLiteral("diagnostic_corrected_virtual_to_real"), diagnosticCorrespondences(
            correctedVirtual, request.realWindowPoints_mm, m_options.coverageDistanceMm)},
        {QStringLiteral("virtual_to_real_row_major"), matrix},
        {QStringLiteral("before_residual_mm"), result.beforeResidualMm},
        {QStringLiteral("after_residual_mm"), result.afterResidualMm},
        {QStringLiteral("before_independent_error_mm"), result.beforeIndependentErrorMm},
        {QStringLiteral("after_independent_error_mm"), result.afterIndependentErrorMm},
        {QStringLiteral("real_coverage_ratio"), result.realCoverageRatio},
        {QStringLiteral("virtual_coverage_ratio"), result.virtualCoverageRatio},
        {QStringLiteral("matched_spread_ratio"), result.matchedSpreadRatio},
        {QStringLiteral("accepted_before_archive"), result.accepted},
        {QStringLiteral("reason"), result.message},
        {QStringLiteral("solve_elapsed_ms"), QString::number(result.solveElapsedMs)},
        {QStringLiteral("icp_parameters"), QJsonObject{
            {QStringLiteral("maximum_iterations"), request.parameters.maximumIterations},
            {QStringLiteral("transformation_epsilon"), request.parameters.transformationEpsilon},
            {QStringLiteral("enable_sor_filter"), request.parameters.enableSorFilter},
            {QStringLiteral("sor_mean_k"), request.parameters.sorMeanK},
            {QStringLiteral("sor_stddev_multiplier"), request.parameters.sorStddevMulThresh},
            {QStringLiteral("downsample_target_points"), request.parameters.downsampleTargetPointCount}}},
        {QStringLiteral("options"), QJsonObject{
            {QStringLiteral("coverage_distance_mm"), m_options.coverageDistanceMm},
            {QStringLiteral("minimum_bidirectional_coverage"), m_options.minimumBidirectionalCoverage},
            {QStringLiteral("minimum_matched_spread_ratio"), m_options.minimumMatchedSpreadRatio},
            {QStringLiteral("maximum_independent_error_mm"), m_options.maximumIndependentErrorMm},
            {QStringLiteral("maximum_independent_worsening_mm"), m_options.maximumIndependentWorseningMm},
            {QStringLiteral("maximum_correspondence_distance_mm"), m_options.maximumCorrespondenceDistanceMm},
            {QStringLiteral("maximum_translation_mm"), m_options.maximumTranslationMm},
            {QStringLiteral("maximum_rotation_deg"), m_options.maximumRotationDeg},
            {QStringLiteral("maximum_residual_mm"), m_options.maximumResidualMm},
            {QStringLiteral("minimum_improvement_ratio"), m_options.minimumImprovementRatio}}}};
    result.archivePath = directory.absoluteFilePath(QStringLiteral("local_icp_%1_%2_%3.json")
        .arg(result.sessionId).arg(result.frameId)
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
    QSaveFile file(result.archivePath);
    file.setDirectWriteFallback(false);
    if (!file.open(QIODevice::WriteOnly)) { return fail(file.errorString()); }
    const QByteArray bytes = QJsonDocument(record).toJson(QJsonDocument::Compact);
    if (file.write(bytes) != bytes.size() || !file.commit()) { return fail(file.errorString()); }
    result.archiveSucceeded = true;
    return true;
}

} // namespace i2r_rlc::ui
