/** @file OnlineLocalRegistrationWorker.cpp
 * @brief 在线局部表面点云采样、配准和小幅修正验收。
 */
#include "OnlineLocalRegistrationWorker.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include <Eigen/Eigenvalues>

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
    // 平面可参与 XYZ+Rz 配准，近共线/单点窗口不能约束工件位姿。
    return values.allFinite() && values[1] >= 0.01
        && values[1] >= values[2] * 1e-4;
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
    m_options.maximumSolveTimeMs = std::clamp(options.maximumSolveTimeMs, 1, 2000);
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
        const auto &offset = options.positionerCalibration.toolTcpInLastLinkMm;
        // 提取结果为工装 TCP 系；RobotSim 线以末节为参考，只有平移差。
        for (const auto &p : extracted.linePoints) {
            if (p.valid) {
                realPoints.push_back({p.xMm + offset.x(), p.yMm + offset.y(), p.zMm + offset.z()});
            }
        }
        std::vector<Point> virtualPoints;
        virtualPoints.reserve(observation.laserLinePoints_mm.size());
        for (const auto &p : observation.laserLinePoints_mm) {
            virtualPoints.push_back({p.x_mm, p.y_mm, p.z_mm});
        }
        result = processClouds(input.sessionId, input.generation, input.frame.frameId,
            realPoints, virtualPoints);
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
    const std::vector<Point> &realPointsLastLink_mm,
    const std::vector<Point> &virtualPointsLastLink_mm)
{
    QElapsedTimer timer;
    timer.start();
    Result result;
    result.sessionId = sessionId;
    result.generation = generation;
    result.frameId = frameId;
    if (!m_options.enabled || sessionId == 0) { return result; }
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
    frame.real = boundedPoints(realPointsLastLink_mm, m_options.maximumPointsPerCloud);
    frame.reference = boundedPoints(virtualPointsLastLink_mm, m_options.maximumPointsPerCloud);
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
    request.parameters.maxCorrespondenceDistance_mm = 5.0;
    request.parameters.enableSorFilter = true;
    for (const auto &item : m_frames) {
        request.realWindowPoints_mm.insert(request.realWindowPoints_mm.end(), item.real.begin(), item.real.end());
        request.virtualWindowPoints_mm.insert(request.virtualWindowPoints_mm.end(), item.reference.begin(), item.reference.end());
    }
    // 每次求解后重新累计；旧窗口不能成为工件增量修正的第二次输入。
    m_frames.clear();
    request.realWindowPoints_mm = boundedPoints(request.realWindowPoints_mm, m_options.maximumPointsPerCloud);
    request.virtualWindowPoints_mm = boundedPoints(request.virtualWindowPoints_mm, m_options.maximumPointsPerCloud);
    result.pointCount = static_cast<int>(std::max(request.realWindowPoints_mm.size(), request.virtualWindowPoints_mm.size()));
    if (!observableSurface(request.realWindowPoints_mm)
        || !observableSurface(request.virtualWindowPoints_mm)) {
        result.message = QStringLiteral("局部点云近共线或覆盖不足，拒绝工件修正。");
        result.elapsedMs = timer.elapsed();
        return result;
    }
    QElapsedTimer solveTimer;
    solveTimer.start();
    try {
        const auto solved = m_pipeline.run(request);
        result.solveElapsedMs = solveTimer.elapsed();
        result.beforeResidualMm = solved.beforeResidual_mm;
        result.afterResidualMm = solved.afterResidual_mm;
        result.correction = solved.virtualToRealCorrectionTransform;
        const auto &matrix = result.correction.rowMajor;
        const bool finite = std::all_of(matrix.begin(), matrix.end(),
            [](double value) { return std::isfinite(value); });
        const double translation = std::sqrt(matrix[3] * matrix[3]
            + matrix[7] * matrix[7] + matrix[11] * matrix[11]);
        const double rotation = std::acos(std::clamp(
            (matrix[0] + matrix[5] + matrix[10] - 1.0) * 0.5, -1.0, 1.0))
            * 180.0 / 3.14159265358979323846;
        result.accepted = solved.status == localization::LocalizationStatus::Succeeded
            && solved.icpResult.hasPoseCorrection && finite
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
    } catch (const std::exception &error) {
        result.solveElapsedMs = solveTimer.elapsed();
        result.message = QStringLiteral("局部配准异常：%1").arg(QString::fromUtf8(error.what()));
    }
    result.elapsedMs = timer.elapsed();
    return result;
}

} // namespace i2r_rlc::ui
