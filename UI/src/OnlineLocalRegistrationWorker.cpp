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
#include <QBuffer>
#include <QCryptographicHash>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QUuid>

#include "LocalizationPipelineMath.h"
#include "RuntimeWorkerAdapters.h"
#include "geometry/WeldImageToSpaceProjector.h"
#include "geometry/WeldProjectionContextBuilder.h"

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

bool usableIndependent(const COnlineLocalRegistrationWorker::IndependentObservation &observation,
    qint64 frameId)
{
    return observation.valid && observation.frameId == frameId
        && finitePoint(observation.realWeldPointInParent_mm)
        && finitePoint(observation.virtualWeldPointInParent_mm);
}

QJsonObject surfaceMetricsJson(const localization::surface_validation::Metrics &metrics)
{
    return {{QStringLiteral("valid"), metrics.valid},
        {QStringLiteral("real_to_virtual_mean_mm"), metrics.realToVirtualMeanMm},
        {QStringLiteral("virtual_to_real_mean_mm"), metrics.virtualToRealMeanMm},
        {QStringLiteral("robust_symmetric_residual_mm"), metrics.robustSymmetricResidualMm},
        {QStringLiteral("real_coverage_ratio"), metrics.realCoverageRatio},
        {QStringLiteral("virtual_coverage_ratio"), metrics.virtualCoverageRatio}};
}

localization::surface_validation::Options surfaceOptions(
    const COnlineLocalRegistrationWorker::Options &options)
{
    localization::surface_validation::Options result;
    result.coverageDistanceMm = options.coverageDistanceMm;
    result.minimumBidirectionalCoverage = options.minimumBidirectionalCoverage;
    result.maximumResidualMm = options.maximumResidualMm;
    result.minimumImprovementRatio = options.minimumImprovementRatio;
    result.maximumHoldoutWorseningMm = options.maximumIndependentWorseningMm;
    return result;
}

/** @brief 原图窄亮带支持，只排除缺峰和宽饱和反光，不宣称识别所有反射。 */
bool rawSurfaceSupport(const QImage &gray, double pixelX, double pixelY,
    bool &widePeak)
{
    widePeak = false;
    if (!std::isfinite(pixelX) || !std::isfinite(pixelY) || gray.isNull()
        || pixelX < 0.0 || pixelX >= gray.width()
        || pixelY < 16.0 || pixelY >= gray.height() - 16.0) { return false; }
    const int x = static_cast<int>(std::lround(pixelX));
    const int centerY = static_cast<int>(std::lround(pixelY));
    if (gray.isNull() || x < 0 || x >= gray.width() || centerY < 16
        || centerY + 16 >= gray.height()) { return false; }
    int background = 255;
    int peak = -1;
    int peakY = centerY;
    for (int y = centerY - 16; y <= centerY + 16; ++y) {
        const int value = gray.constScanLine(y)[x];
        background = std::min(background, value);
        if (std::abs(y - centerY) <= 3 && value > peak) {
            peak = value;
            peakY = y;
        }
    }
    if (peak - background < 40) { return false; }
    int leftBackground = 255;
    int rightBackground = 255;
    for (int y = centerY - 16; y <= peakY - 5; ++y) {
        leftBackground = std::min(leftBackground, static_cast<int>(gray.constScanLine(y)[x]));
    }
    for (int y = peakY + 5; y <= centerY + 16; ++y) {
        rightBackground = std::min(rightBackground, static_cast<int>(gray.constScanLine(y)[x]));
    }
    if (peak - leftBackground < 40 || peak - rightBackground < 40) { return false; }
    const double threshold = background + 0.5 * (peak - background);
    int width = 1;
    for (int y = peakY - 1; y >= centerY - 16
        && gray.constScanLine(y)[x] > threshold; --y) { ++width; }
    for (int y = peakY + 1; y <= centerY + 16
        && gray.constScanLine(y)[x] > threshold; ++y) { ++width; }
    widePeak = width > 12;
    return !widePeak;
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
    m_surfaceRejectionStreak = 0;
    m_surfaceRecoveryWindowFrames = 0;
    m_surfaceRecoveryPoseRevision = 0;
    m_clock.restart();
}

COnlineLocalRegistrationWorker::SurfaceFrameResult
COnlineLocalRegistrationWorker::measureSurface(const Input &input) const
{
    SurfaceFrameResult result;
    result.frameId = input.frame.frameId;
    result.workpiecePoseRevision = input.workpiecePoseRevision;
    if (!isUsableCorrectionObservationForFrame(input.frame.frameId, input.observation)
        || !input.trackingOptions.positionerCalibration.valid) {
        result.message = QStringLiteral("表面缺少同帧参考或有效变位机标定。");
        return result;
    }
    try {
        RuntimeTrackingOptions options = input.trackingOptions;
        options.globalLocateLineExtraction = true;
        options.requireDetectionWindow = false;
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
        correlation.workpiecePoseRevision = input.workpiecePoseRevision;
        auto observation = input.observation;
        observation.positioningSegmentIndex = -1;
        extractor.submitFrameWithRobotStateObservationAndCorrelation(
            input.frame, input.robotState, -1, observation, correlation);
        if (!extracted.valid || !extracted.triggerAccepted) {
            result.message = QStringLiteral("真实母材表面提取跳过：%1").arg(extracted.message);
            return result;
        }
        // 预焊凸起附近不参与整体位姿拟合。中心来自业务参考，只作排除掩膜，
        // 绝不把该估计点写进真实表面或独立焊点。
        const double seamX = input.seamImageXPixel >= 0.0
            ? input.seamImageXPixel : input.observation.weldPointImage_px.x_px;
        if (m_options.surfaceAssistedUpdatesEnabled && !std::isfinite(seamX)) {
            result.message = QStringLiteral("母材排除掩膜缺少有限同帧参考位置。");
            return result;
        }
        if (!m_options.surfaceAssistedUpdatesEnabled) {
            // 显式关闭新支路时保留旧点云来源、采样和独立焊点验收行为。
            for (const auto &point : extracted.linePoints) {
                if (point.valid) {
                    result.realPoints_mm.push_back({point.xMm, point.yMm, point.zMm});
                }
            }
        }
        auto projectionContext = input.projectionContext;
        const auto built = vision::CWeldProjectionContextBuilder::build(
            options.sensorCalibration, options.positionerCalibration, input.robotState);
        if (!built.valid) {
            result.message = QStringLiteral("母材同帧投影上下文无效：%1")
                .arg(QString::fromStdString(built.message));
            return result;
        }
        projectionContext = built.context;
        const QImage rawImage = (input.frame.image.isNull()
            ? input.frame.previewImage : input.frame.image)
            .convertToFormat(QImage::Format_Grayscale8);
        int rawCandidates = 0;
        int rawSupported = 0;
        int rawWideRejected = 0;
        int rawOtherRejected = 0;
        QJsonArray supportedPixels;
        // 不按压缩后的三维点索引反查像素：单点投影失败会使两列索引错位。
        // 每个被采样的原图像素独立应用与提取器相同的冻结姿态/标定链。
        for (std::size_t index = 0; index < extracted.realLaserLineImagePoints_px.size(); index += 5U) {
            if (!m_options.surfaceAssistedUpdatesEnabled) { break; }
            const auto &pixel = extracted.realLaserLineImagePoints_px[index];
            if (std::abs(pixel.x_px - seamX) <= 24.0) { continue; }
            ++rawCandidates;
            bool widePeak = false;
            if (!rawSurfaceSupport(rawImage, pixel.x_px, pixel.y_px, widePeak)) {
                if (widePeak) { ++rawWideRejected; }
                else { ++rawOtherRejected; }
                continue;
            }
            ++rawSupported;
            const auto projected = vision::CWeldImageToSpaceProjector::projectCvPoint(
                cv::Point2d(pixel.x_px, pixel.y_px), projectionContext,
                vision::WeldProjectionTargetFrame::PositionerTcp);
            if (projected.valid && projected.spacePoint.valid) {
                const auto &point = projected.spacePoint;
                result.realPoints_mm.push_back({point.xMm, point.yMm, point.zMm});
                supportedPixels.append(QJsonArray{pixel.x_px, pixel.y_px});
            }
        }
        const auto &virtualPixels = input.observation.laserLineImagePoints_px;
        for (std::size_t i = 0; i < input.observation.laserLinePoints_mm.size(); ++i) {
            if (m_options.surfaceAssistedUpdatesEnabled
                && virtualPixels.size() == input.observation.laserLinePoints_mm.size()
                && std::abs(virtualPixels[i].x_px - input.observation.weldPointImage_px.x_px) <= 24.0) {
                continue;
            }
            const auto &point = input.observation.laserLinePoints_mm[i];
            result.virtualPoints_mm.push_back({point.x_mm, point.y_mm, point.z_mm});
        }
        result.realPoints_mm = boundedPoints(result.realPoints_mm, m_options.maximumPointsPerCloud);
        result.virtualPoints_mm = boundedPoints(result.virtualPoints_mm, m_options.maximumPointsPerCloud);
        const auto metricsOptions = surfaceOptions(m_options);
        result.metrics = localization::surface_validation::measure(
            result.realPoints_mm, result.virtualPoints_mm, metricsOptions);
        result.valid = m_options.surfaceAssistedUpdatesEnabled
            ? result.metrics.valid && rawSupported >= 24 && rawCandidates > 0
                && static_cast<double>(rawSupported) / rawCandidates >= 0.6
            : result.realPoints_mm.size() >= 10U && result.virtualPoints_mm.size() >= 10U;
        QJsonArray virtualImage;
        for (const auto &point : virtualPixels) { virtualImage.append(QJsonArray{point.x_px, point.y_px}); }
        const auto &state = input.robotState;
        std::vector<Point> originalVirtual;
        originalVirtual.reserve(input.observation.laserLinePoints_mm.size());
        for (const auto &point : input.observation.laserLinePoints_mm) {
            originalVirtual.push_back({point.x_mm, point.y_mm, point.z_mm});
        }
        QJsonArray verifiedPose;
        if (input.lastVerifiedWorkpiecePose.has_value()) {
            for (double value : input.lastVerifiedWorkpiecePose->rowMajor) { verifiedPose.append(value); }
        }
        result.evidenceContext = QJsonObject{
            {QStringLiteral("capture_time_ms"), input.frame.timestamp.isValid()
                ? QString::number(input.frame.timestamp.toMSecsSinceEpoch()) : QString()},
            {QStringLiteral("workpiece_pose_revision"), QString::number(input.workpiecePoseRevision)},
            {QStringLiteral("workpiece_id"), input.workpieceId},
            {QStringLiteral("parent_id"), input.parentId},
            {QStringLiteral("reference_object_id"), input.observation.referenceObjectId},
            {QStringLiteral("scene_name"), input.sceneName},
            {QStringLiteral("seam_index"), input.observation.seamIndex},
            {QStringLiteral("geometry_correspondence_verified"), false},
            {QStringLiteral("last_verified_parent_from_workpiece"), verifiedPose},
            {QStringLiteral("virtual_laser_image_px"), virtualImage},
            {QStringLiteral("virtual_laser_parent_mm"), cloudJson(originalVirtual)},
            {QStringLiteral("raw_supported_surface_image_px"), supportedPixels},
            {QStringLiteral("raw_surface_candidate_count"), rawCandidates},
            {QStringLiteral("raw_surface_supported_count"), rawSupported},
            {QStringLiteral("raw_surface_wide_peak_rejected_count"), rawWideRejected},
            {QStringLiteral("raw_surface_other_rejected_count"), rawOtherRejected},
            {QStringLiteral("raw_surface_minimum_contrast_gray"), 40},
            {QStringLiteral("raw_surface_maximum_peak_width_px"), 12},
            {QStringLiteral("raw_surface_minimum_support_ratio"), 0.6},
            {QStringLiteral("raw_surface_support_gate_enabled"), m_options.surfaceAssistedUpdatesEnabled},
            {QStringLiteral("surface_seam_exclusion_half_width_px"),
                m_options.surfaceAssistedUpdatesEnabled ? 24 : 0},
            {QStringLiteral("real_seam_exclusion_center_x_px"), seamX},
            {QStringLiteral("virtual_pixel_point_pairing_available"),
                virtualPixels.size() == input.observation.laserLinePoints_mm.size()},
            {QStringLiteral("robot_pose_mm_deg"), QJsonArray{
                state.curx_r / 1000.0, state.cury_r / 1000.0, state.curz_r / 1000.0,
                state.curRx_r / 10000.0, state.curRy_r / 10000.0, state.curRz_r / 10000.0,
                state.curS1 / 10000.0, state.curS2 / 10000.0}}};
        result.message = result.valid ? QStringLiteral("同帧真实母材表面已提取。")
            : QStringLiteral("母材表面不足，不使用估计焊点替代。");
    } catch (const std::exception &error) {
        result.message = QStringLiteral("母材表面提取异常：%1").arg(QString::fromUtf8(error.what()));
    }
    return result;
}

COnlineLocalRegistrationWorker::SurfaceFrameResult
COnlineLocalRegistrationWorker::measureSurfaceForVerification(const Input &input,
    const std::vector<Point> &frozenVirtual,
    const localization::RigidTransform &correction) const
{
    auto result = measureSurface(input);
    if (result.valid) {
        auto options = surfaceOptions(m_options);
        // 只作用于在线后验；当前模型须先支持有限续展，真实覆盖/绝对残差不放宽。
        options.maximumModelContinuationMm = std::min(10.0, m_options.maximumCorrespondenceDistanceMm);
        options.maximumModelPlaneDeviationMm = std::max(1e-6, std::min(0.05,
            m_options.maximumIndependentWorseningMm * 0.5));
        result.comparison = localization::surface_validation::compareFreshSurfaceWithCurrentReference(
            result.realPoints_mm, result.virtualPoints_mm, frozenVirtual, correction, options);
    }
    return result;
}

COnlineLocalRegistrationWorker::Result COnlineLocalRegistrationWorker::process(const Input &input)
{
    QElapsedTimer timer;
    timer.start();
    Result result;
    result.sessionId = input.sessionId;
    result.generation = input.generation;
    result.frameId = input.frame.frameId;
    if (!m_options.enabled || input.sessionId == 0) { return result; }
    const auto surface = measureSurface(input);
    if (!surface.valid) {
        reset();
        result.message = surface.message;
        result.elapsedMs = timer.elapsed();
        return result;
    }
    CloudValidationContext validation;
    validation.independentObservation = input.independentObservation;
    validation.archiveDirectory = input.archiveDirectory;
    validation.stabilityAnchors = input.stabilityAnchors;
    validation.hasVerifiedWorkpiecePose = input.lastVerifiedWorkpiecePose.has_value();
    validation.workpiecePoseRevision = input.workpiecePoseRevision;
    validation.sourceImage = input.frame.image.isNull() ? input.frame.previewImage : input.frame.image;
    validation.evidenceContext = surface.evidenceContext;
    result = processClouds(input.sessionId, input.generation, input.frame.frameId,
        surface.realPoints_mm, surface.virtualPoints_mm, validation);
    result.elapsedMs = timer.elapsed();
    return result;
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
    result.workpiecePoseRevision = validation.workpiecePoseRevision;
    if (!m_options.enabled || sessionId == 0) { return result; }
    const bool independentUsable = usableIndependent(validation.independentObservation, frameId);
    const bool surfaceEligible = m_options.surfaceAssistedUpdatesEnabled
        && validation.hasVerifiedWorkpiecePose && validation.workpiecePoseRevision > 0;
    if (validation.requireIndependentObservation
        && ((!independentUsable && !surfaceEligible)
            || (validation.independentObservation.valid && !independentUsable)
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
    if (surfaceEligible && m_surfaceRecoveryPoseRevision != validation.workpiecePoseRevision) {
        m_frames.clear();
        m_surfaceRejectionStreak = 0;
        m_surfaceRecoveryWindowFrames = 0;
        m_surfaceRecoveryPoseRevision = validation.workpiecePoseRevision;
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
    frame.evidenceContext = validation.evidenceContext;
    frame.hasVerifiedWorkpiecePose = validation.hasVerifiedWorkpiecePose;
    frame.workpiecePoseRevision = validation.workpiecePoseRevision;
    // 每次尝试只留第一张原图；图像共享只读，后续帧不累计额外图像预算。
    if (m_frames.empty()) { frame.sourceImage = validation.sourceImage; }
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
    const bool useSurfaceWindow = validation.requireIndependentObservation
        && m_options.surfaceAssistedUpdatesEnabled
        && (surfaceEligible || std::any_of(m_frames.begin(), m_frames.end(), [](const CloudFrame &item) {
            return item.hasVerifiedWorkpiecePose;
        }));
    const int maximumFrames = useSurfaceWindow
        ? std::max(8, m_options.maximumFrames) : m_options.maximumFrames;
    while (m_frames.size() > static_cast<std::size_t>(maximumFrames)
        || (!m_frames.empty() && now - m_frames.front().capturedMs > maximumAgeMs)) {
        m_frames.pop_front();
    }
    result.frameCount = static_cast<int>(m_frames.size());
    result.pointCount = static_cast<int>(std::max(
        m_frames.back().real.size(), m_frames.back().reference.size()));
    // 建立可信位姿后统一走表面验收；不能因恰好有焊点而重新耦合到焊点后验。
    const bool needsSurfaceWindow = useSurfaceWindow;
    const int minimumFrames = needsSurfaceWindow
        ? std::max(std::max(8, m_options.minimumFrames), m_surfaceRecoveryWindowFrames)
        : m_options.minimumFrames;
    if (result.frameCount < minimumFrames
        || (m_lastMatchMs >= 0 && now - m_lastMatchMs < m_options.minimumMatchIntervalMs)) {
        return result;
    }
    result.attempted = true;
    m_lastMatchMs = now;
    if (needsSurfaceWindow) {
        std::deque<CloudFrame> attemptedFrames;
        attemptedFrames.swap(m_frames);
        result.precedingSurfaceRejections = m_surfaceRejectionStreak;
        result = processSurfaceWindow(attemptedFrames, validation, result);
        if (result.accepted) {
            m_surfaceRejectionStreak = 0;
            m_surfaceRecoveryWindowFrames = 0;
        } else {
            ++m_surfaceRejectionStreak;
            // 本次旧窗已经移出；退化时增加下一批全新曝光的跨度，不重算同一坏窗。
            if (m_options.surfaceRecoveryEnabled && !result.surfaceValidation.hasObservableCorrection) {
                m_surfaceRecoveryWindowFrames = std::min(maximumFrames, minimumFrames + 4);
            }
        }
        result.elapsedMs = timer.elapsed();
        return result;
    }
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
    result.baseSafetyAccepted = result.accepted;
    if (m_options.stabilityProtectionEnabled && result.accepted) {
        std::vector<COnlineLocalRegistrationStability::Observation> current;
        for (const auto &item : attemptedFrames) {
            const auto &observation = item.independentObservation;
            current.push_back({observation.realWeldPointInParent_mm,
                observation.virtualWeldPointInParent_mm, observation.frameId});
        }
        result.stabilityEvaluated = true;
        result.stabilityEvaluation = COnlineLocalRegistrationStability::evaluate(
            current, validation.stabilityAnchors, result.correction,
            result.beforeResidualMm, m_options.maximumIndependentErrorMm);
        result.accepted = result.stabilityEvaluation.accepted;
        result.message = QStringLiteral("%1 焊点均值 %2→%3 mm，要求收益 %4 mm；历史 %5 区，预测最大误差 %6 mm（未写入），冲突 %7 区，跨度 %8 mm，历史几何未校核。")
            .arg(result.stabilityEvaluation.reason)
            .arg(result.stabilityEvaluation.beforeMeanMm, 0, 'f', 4)
            .arg(result.stabilityEvaluation.afterMeanMm, 0, 'f', 4)
            .arg(result.stabilityEvaluation.requiredGainMm, 0, 'f', 4)
            .arg(result.stabilityEvaluation.historicalCount)
            .arg(result.stabilityEvaluation.maximumHistoricalErrorMm, 0, 'f', 4)
            .arg(result.stabilityEvaluation.worsenedRegionCount)
            .arg(result.stabilityEvaluation.spatialSpanMm, 0, 'f', 1);
    }
    archiveAttempt(attemptedFrames, request, preprocessedRealPoints, validation, result);
    result.elapsedMs = timer.elapsed();
    return result;
}

COnlineLocalRegistrationWorker::Result
COnlineLocalRegistrationWorker::processSurfaceWindow(const std::deque<CloudFrame> &frames,
    const CloudValidationContext &validation, Result result)
{
    result.surfaceOnly = true;
    result.workpiecePoseRevision = validation.workpiecePoseRevision;
    localization::LocalCorrectionPipelineRequest request;
    request.frameId = result.frameId;
    request.dofMask = {true, true, true, false, false, true};
    request.parameters.maximumIterations = m_options.maximumIterations;
    request.parameters.maxCorrespondenceDistance_mm = m_options.maximumCorrespondenceDistanceMm;
    request.parameters.enableSorFilter = true;
    const auto reject = [&](const QString &message) {
        result.accepted = false;
        result.message = message;
        archiveAttempt(frames, request, {}, validation, result);
        return result;
    };
    if (frames.size() < 8U || !m_options.surfaceAssistedUpdatesEnabled
        || validation.workpiecePoseRevision == 0
        || !std::all_of(frames.begin(), frames.end(), [&validation](const CloudFrame &frame) {
            return frame.hasVerifiedWorkpiecePose
                && frame.workpiecePoseRevision == validation.workpiecePoseRevision;
        })) {
        return reject(QStringLiteral("表面窗要求已有可信位姿、同位姿修订及至少8个完整帧。"));
    }
    localization::surface_validation::Request surfaceRequest;
    surfaceRequest.options = surfaceOptions(m_options);
    surfaceRequest.options.allowTrainingCoverageRecovery = m_options.surfaceRecoveryEnabled
        && result.precedingSurfaceRejections >= 2;
    const std::size_t trainingCount = frames.size() - 2U;
    for (std::size_t index = 0; index < frames.size(); ++index) {
        auto &real = index < trainingCount ? request.realWindowPoints_mm
            : surfaceRequest.holdoutReal_mm;
        auto &reference = index < trainingCount ? request.virtualWindowPoints_mm
            : surfaceRequest.holdoutVirtual_mm;
        real.insert(real.end(), frames[index].real.begin(), frames[index].real.end());
        reference.insert(reference.end(), frames[index].reference.begin(), frames[index].reference.end());
    }
    request.realWindowPoints_mm = boundedPoints(request.realWindowPoints_mm, m_options.maximumPointsPerCloud);
    request.virtualWindowPoints_mm = boundedPoints(request.virtualWindowPoints_mm, m_options.maximumPointsPerCloud);
    surfaceRequest.trainingReal_mm = request.realWindowPoints_mm;
    surfaceRequest.trainingVirtual_mm = request.virtualWindowPoints_mm;
    surfaceRequest.holdoutReal_mm = boundedPoints(surfaceRequest.holdoutReal_mm, m_options.maximumPointsPerCloud);
    surfaceRequest.holdoutVirtual_mm = boundedPoints(surfaceRequest.holdoutVirtual_mm, m_options.maximumPointsPerCloud);
    // 只用训练云提供平移初值，避免旧LM在纯轴向、重复采样时停在单位阵。
    // 初值不参与验收；最终候选仍需可观测投影、原始云收益及留出/新帧检查。
    const auto center = [](const std::vector<Point> &cloud) {
        Eigen::Vector3d value = Eigen::Vector3d::Zero();
        for (const auto &point : cloud) {
            value += Eigen::Vector3d(point.x_mm, point.y_mm, point.z_mm);
        }
        return (value / static_cast<double>(cloud.size())).eval();
    };
    const Eigen::Vector3d realToVirtualSeed = center(request.virtualWindowPoints_mm)
        - center(request.realWindowPoints_mm);
    request.initialGuess.rowMajor[3] = realToVirtualSeed.x();
    request.initialGuess.rowMajor[7] = realToVirtualSeed.y();
    request.initialGuess.rowMajor[11] = realToVirtualSeed.z();
    request.windowFrameCount = static_cast<int>(trainingCount);
    request.minimumWindowFrameCount = request.windowFrameCount;
    result.pointCount = static_cast<int>(std::max(request.realWindowPoints_mm.size(),
        request.virtualWindowPoints_mm.size()));
    QElapsedTimer solveTimer;
    solveTimer.start();
    std::vector<Point> preprocessedRealPoints;
    try {
        // 求解只读前N-2帧；末两帧不传入PCL或模式投影的法向估计。
        const auto solved = m_pipeline.run(request);
        preprocessedRealPoints = solved.icpResult.preprocessedRealPoints_mm;
        result.correction = solved.virtualToRealCorrectionTransform;
        if (solved.status != localization::LocalizationStatus::Succeeded
            || !solved.icpResult.hasPoseCorrection) {
            result.solveElapsedMs = solveTimer.elapsed();
            result.message = QStringLiteral("母材训练配准未求得候选：%1").arg(solved.message);
            archiveAttempt(frames, request, preprocessedRealPoints, validation, result);
            return result;
        }
        surfaceRequest.candidateVirtualToReal = result.correction;
        result.originalSurfaceCorrection = result.correction;
        result.surfaceValidation = localization::surface_validation::evaluate(surfaceRequest);
        if (result.surfaceValidation.hasObservableCorrection) {
            result.correction = result.surfaceValidation.observableVirtualToReal;
        }
        const auto &checked = result.surfaceValidation;
        result.beforeResidualMm = checked.trainingBefore.robustSymmetricResidualMm;
        result.afterResidualMm = checked.trainingAfter.robustSymmetricResidualMm;
        result.realCoverageRatio = std::min(checked.trainingAfter.realCoverageRatio,
            checked.holdoutAfter.realCoverageRatio);
        result.virtualCoverageRatio = std::min(checked.trainingAfter.virtualCoverageRatio,
            checked.holdoutAfter.virtualCoverageRatio);
        const auto corrected = localization::pipeline_math::transformPoints(
            request.virtualWindowPoints_mm, result.correction);
        std::vector<Point> matchedReal;
        std::vector<Point> matchedVirtual;
        coveredPoints(request.realWindowPoints_mm, corrected, m_options.coverageDistanceMm, matchedReal);
        coveredPoints(corrected, request.realWindowPoints_mm, m_options.coverageDistanceMm, matchedVirtual);
        result.matchedSpreadRatio = std::min(matchedSpread(request.realWindowPoints_mm, matchedReal),
            matchedSpread(corrected, matchedVirtual));
        const auto fullResidualWithinBudget = [this](const localization::surface_validation::Metrics &metrics) {
            return metrics.valid && metrics.realToVirtualMeanMm <= m_options.maximumResidualMm
                && metrics.virtualToRealMeanMm <= m_options.maximumResidualMm;
        };
        const auto &matrix = result.correction.rowMajor;
        const double translation = std::sqrt(matrix[3] * matrix[3]
            + matrix[7] * matrix[7] + matrix[11] * matrix[11]);
        const double rotation = std::abs(std::atan2(matrix[4], matrix[0]))
            * 180.0 / 3.14159265358979323846;
        result.solveElapsedMs = solveTimer.elapsed();
        const double translationLimit = checked.trainingCoverageRecoveryUsed
            ? std::min(10.0, m_options.maximumTranslationMm) : m_options.maximumTranslationMm;
        const double rotationLimit = checked.trainingCoverageRecoveryUsed
            ? std::min(3.0, m_options.maximumRotationDeg) : m_options.maximumRotationDeg;
        result.accepted = checked.accepted
            && fullResidualWithinBudget(checked.trainingAfter)
            && fullResidualWithinBudget(checked.holdoutAfter)
            && result.matchedSpreadRatio >= m_options.minimumMatchedSpreadRatio
            && translation <= translationLimit
            && rotation <= rotationLimit
            && result.solveElapsedMs <= m_options.maximumSolveTimeMs;
        // 仅作当前冻结模型参考的位移预算，不宣称这些激光交点具有固定材料身份。
        result.safetyKeypointsInParent_mm = request.virtualWindowPoints_mm;
        result.surfaceVerificationReference_mm = request.virtualWindowPoints_mm;
        auto preflightOptions = surfaceOptions(m_options);
        preflightOptions.maximumModelContinuationMm = std::min(10.0, m_options.maximumCorrespondenceDistanceMm);
        preflightOptions.maximumModelPlaneDeviationMm = std::max(1e-6, std::min(0.05,
            m_options.maximumIndependentWorseningMm * 0.5));
        const auto movedHoldoutReference = localization::pipeline_math::transformPoints(
            surfaceRequest.holdoutVirtual_mm, result.correction);
        // 这里只验证末两帧有冻结面支持；刚体变换旧切片不是写后CAD重新求交真值。
        result.surfaceVerificationPreflight = localization::surface_validation::compareFreshSurfaceWithCurrentReference(
            surfaceRequest.holdoutReal_mm, movedHoldoutReference,
            request.virtualWindowPoints_mm, result.correction, preflightOptions);
        result.solveElapsedMs = solveTimer.elapsed();
        result.accepted = result.accepted && result.surfaceVerificationPreflight.valid
            && result.solveElapsedMs <= m_options.maximumSolveTimeMs;
        result.safetyKeypointsInParent_mm.insert(result.safetyKeypointsInParent_mm.end(),
            surfaceRequest.holdoutVirtual_mm.begin(), surfaceRequest.holdoutVirtual_mm.end());
        result.message = result.accepted
            ? QStringLiteral("母材表面可观测投影及完整帧留出通过；不使用估计焊点自证，待新帧表面后验。")
            : QStringLiteral("母材表面投影、留出、完整两向残差、跨度或幅度/耗时未通过：%1")
                .arg(checked.message);
        if (!result.surfaceVerificationPreflight.valid) {
            result.message += QStringLiteral(" 写前冻结面配对支持不足，取消本次写回（状态=%1，实测支持=%2，模型支持=%3）。")
                .arg(static_cast<int>(result.surfaceVerificationPreflight.status))
                .arg(result.surfaceVerificationPreflight.supportedRatio, 0, 'f', 3)
                .arg(result.surfaceVerificationPreflight.currentModelSupportedRatio, 0, 'f', 3);
        }
        const auto recordRejection = [&result](bool failed, const QString &reason) {
            if (failed) { result.rejectionReasons.append(reason); }
        };
        recordRejection(!checked.accepted, QStringLiteral("表面验收：%1").arg(checked.message));
        recordRejection(!fullResidualWithinBudget(checked.trainingAfter)
                || !fullResidualWithinBudget(checked.holdoutAfter),
            QStringLiteral("完整两向平均残差超限。"));
        recordRejection(result.matchedSpreadRatio < m_options.minimumMatchedSpreadRatio,
            QStringLiteral("匹配区域跨度不足。"));
        recordRejection(translation > translationLimit,
            QStringLiteral("单步平移 %1 mm 超过本候选预算 %2 mm。").arg(translation).arg(translationLimit));
        recordRejection(rotation > rotationLimit,
            QStringLiteral("单步旋转 %1 度超过本候选预算 %2 度。").arg(rotation).arg(rotationLimit));
        recordRejection(result.solveElapsedMs > m_options.maximumSolveTimeMs,
            QStringLiteral("求解及验收耗时超预算。"));
        recordRejection(!result.surfaceVerificationPreflight.valid,
            QStringLiteral("写前冻结面配对不足：状态=%1，实测支持=%2，模型支持=%3。")
                .arg(static_cast<int>(result.surfaceVerificationPreflight.status))
                .arg(result.surfaceVerificationPreflight.supportedRatio)
                .arg(result.surfaceVerificationPreflight.currentModelSupportedRatio));
        if (!result.rejectionReasons.isEmpty()) {
            result.message = result.rejectionReasons.join(QStringLiteral("；"));
        } else if (checked.trainingCoverageRecoveryUsed) {
            result.message = QStringLiteral("连续拒绝后受限恢复候选通过；留出门限保持原值，待写后新帧验证。");
        }
        int independentVetoCount = 0;
        // 缺失观测允许表面支路工作，但不能因此丢弃同窗其他真实焊点的反证。
        // 始终检查最终投影候选，不用PCL原矩阵或估计焊点替代这些原图点。
        for (const auto &frame : frames) {
            const auto &independent = frame.independentObservation;
            if (!usableIndependent(independent, frame.frameId)) { continue; }
            const auto correctedWeld = localization::pipeline_math::transformPoints(
                {independent.virtualWeldPointInParent_mm}, result.correction);
            const double before = pointDistance(independent.realWeldPointInParent_mm,
                independent.virtualWeldPointInParent_mm);
            const double after = pointDistance(independent.realWeldPointInParent_mm,
                correctedWeld.front());
            result.beforeIndependentErrorMm = std::max(result.beforeIndependentErrorMm, before);
            result.afterIndependentErrorMm = std::max(result.afterIndependentErrorMm, after);
            if (!std::isfinite(after) || after > m_options.maximumIndependentErrorMm
                || after > before + m_options.maximumIndependentWorseningMm) {
                result.accepted = false;
                ++independentVetoCount;
                if (independentVetoCount == 1) {
                    const auto reason = QStringLiteral("表面窗已有真实焊点否决最终投影候选：frame=%1，前=%2 mm，后=%3 mm。")
                        .arg(frame.frameId).arg(before).arg(after);
                    result.rejectionReasons.append(reason);
                    result.message += QStringLiteral(" ") + reason;
                }
            }
        }
        result.baseSafetyAccepted = result.accepted;
        if (m_options.stabilityProtectionEnabled && result.accepted) {
            std::vector<COnlineLocalRegistrationStability::Observation> current;
            for (const auto &frame : frames) {
                const auto &independent = frame.independentObservation;
                if (usableIndependent(independent, frame.frameId)) {
                    current.push_back({independent.realWeldPointInParent_mm,
                        independent.virtualWeldPointInParent_mm, frame.frameId});
                }
            }
            if (!current.empty()) {
                result.stabilityEvaluated = true;
                result.stabilityEvaluation = COnlineLocalRegistrationStability::evaluate(
                    current, validation.stabilityAnchors, result.correction,
                    result.beforeResidualMm, m_options.maximumIndependentErrorMm);
                result.accepted = result.stabilityEvaluation.accepted;
                result.message += QStringLiteral(" 有效实测焊点子集收益：%1。")
                    .arg(result.stabilityEvaluation.reason);
            } else {
                result.message += QStringLiteral(" 本窗无独立焊点，未执行焊点收益评估；仅表面训练/留出提供验收。 ");
            }
        }
    } catch (const std::exception &error) {
        result.accepted = false;
        result.solveElapsedMs = solveTimer.elapsed();
        result.message = QStringLiteral("母材表面训练/验收异常：%1").arg(QString::fromUtf8(error.what()));
    }
    archiveAttempt(frames, request, preprocessedRealPoints, validation, result);
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
    result.archivePath = directory.absoluteFilePath(QStringLiteral("local_icp_%1_%2_%3.json")
        .arg(result.sessionId).arg(result.frameId)
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
    QJsonObject sourceImageRecord{
        {QStringLiteral("saved"), false},
        {QStringLiteral("policy"), QStringLiteral("at_most_one_original_image_per_attempt")}};
    QImage sourceImage = validation.sourceImage;
    qint64 sourceFrameId = result.frameId;
    for (const auto &frame : frames) {
        if (!frame.sourceImage.isNull()) {
            sourceImage = frame.sourceImage;
            sourceFrameId = frame.frameId;
            break;
        }
    }
    if (!sourceImage.isNull()) {
        if (static_cast<quint64>(sourceImage.width()) * sourceImage.height() > 16U * 1024U * 1024U) {
            return fail(QStringLiteral("原图尺寸超出单次证据预算。"));
        }
        QByteArray pngBytes;
        QBuffer buffer(&pngBytes);
        if (!buffer.open(QIODevice::WriteOnly) || !sourceImage.save(&buffer, "PNG")) {
            return fail(QStringLiteral("无法编码原图证据。"));
        }
        const QString imagePath = result.archivePath + QStringLiteral(".source.png");
        QSaveFile imageFile(imagePath);
        imageFile.setDirectWriteFallback(false);
        if (!imageFile.open(QIODevice::WriteOnly)
            || imageFile.write(pngBytes) != pngBytes.size() || !imageFile.commit()) {
            return fail(QStringLiteral("原图证据写入失败：%1").arg(imageFile.errorString()));
        }
        sourceImageRecord = QJsonObject{
            {QStringLiteral("saved"), true},
            {QStringLiteral("frame_id"), QString::number(sourceFrameId)},
            {QStringLiteral("file"), QFileInfo(imagePath).fileName()},
            {QStringLiteral("encoded_bytes"), pngBytes.size()},
            {QStringLiteral("sha256"), QString::fromLatin1(
                QCryptographicHash::hash(pngBytes, QCryptographicHash::Sha256).toHex())},
            {QStringLiteral("policy"), QStringLiteral("at_most_one_original_image_per_attempt")}};
    }
    QJsonArray rawFrames;
    for (const auto &frame : frames) {
        rawFrames.append(QJsonObject{
            {QStringLiteral("frame_id"), QString::number(frame.frameId)},
            {QStringLiteral("captured_ms"), QString::number(frame.capturedMs)},
            {QStringLiteral("raw_real_parent_mm"), cloudJson(frame.rawReal)},
            {QStringLiteral("raw_virtual_parent_mm"), cloudJson(frame.rawReference)},
            {QStringLiteral("independent"), independentJson(frame.independentObservation)},
            {QStringLiteral("evidence_context"), frame.evidenceContext}});
    }
    QJsonArray matrix;
    for (double value : result.correction.rowMajor) { matrix.append(value); }
    QJsonArray initialGuessMatrix;
    for (double value : request.initialGuess.rowMajor) { initialGuessMatrix.append(value); }
    QJsonArray originalSurfaceMatrix;
    for (double value : result.originalSurfaceCorrection.rowMajor) { originalSurfaceMatrix.append(value); }
    QJsonArray trainingFrameIds;
    QJsonArray holdoutFrameIds;
    std::vector<Point> holdoutReal;
    std::vector<Point> holdoutVirtual;
    if (result.surfaceOnly && frames.size() >= 2U) {
        for (std::size_t index = 0; index < frames.size(); ++index) {
            if (index + 2U < frames.size()) {
                trainingFrameIds.append(QString::number(frames[index].frameId));
            } else {
                holdoutFrameIds.append(QString::number(frames[index].frameId));
                holdoutReal.insert(holdoutReal.end(), frames[index].real.begin(), frames[index].real.end());
                holdoutVirtual.insert(holdoutVirtual.end(), frames[index].reference.begin(), frames[index].reference.end());
            }
        }
        holdoutReal = boundedPoints(holdoutReal, m_options.maximumPointsPerCloud);
        holdoutVirtual = boundedPoints(holdoutVirtual, m_options.maximumPointsPerCloud);
    }
    QJsonArray modeStrengths;
    for (double value : result.surfaceValidation.relativeModeStrengths) { modeStrengths.append(value); }
    const auto correctedVirtual = localization::pipeline_math::transformPoints(
        request.virtualWindowPoints_mm, result.correction);
    QJsonArray stabilityAnchors;
    for (const auto &anchor : validation.stabilityAnchors) {
        stabilityAnchors.append(QJsonObject{
            {QStringLiteral("frame_id"), QString::number(anchor.observation.frameId)},
            {QStringLiteral("real_parent_mm"), pointJson(anchor.observation.realPoint)},
            {QStringLiteral("material_point_current_parent_mm"), pointJson(anchor.observation.virtualPoint)},
            {QStringLiteral("reference_error_mm"), anchor.referenceErrorMm},
            {QStringLiteral("sample_count"), anchor.sampleCount},
            {QStringLiteral("last_frame_id"), QString::number(anchor.lastFrameId)},
            {QStringLiteral("last_observed_error_mm"), anchor.lastObservedErrorMm},
            {QStringLiteral("geometry_correspondence_verified"), false}});
    }
    QJsonObject record{
        {QStringLiteral("schema_version"), 4},
        {QStringLiteral("history_policy_version"), 2},
        {QStringLiteral("base_safety_accepted"), result.baseSafetyAccepted},
        {QStringLiteral("preceding_surface_rejections"), result.precedingSurfaceRejections},
        {QStringLiteral("surface_recovery_enabled"), m_options.surfaceRecoveryEnabled},
        {QStringLiteral("training_coverage_recovery_used"), result.surfaceValidation.trainingCoverageRecoveryUsed},
        {QStringLiteral("rejection_reasons"), QJsonArray::fromStringList(result.rejectionReasons)},
        {QStringLiteral("session_id"), QString::number(result.sessionId)},
        {QStringLiteral("generation"), QString::number(result.generation)},
        {QStringLiteral("frame_id"), QString::number(result.frameId)},
        {QStringLiteral("workpiece_pose_revision"), QString::number(result.workpiecePoseRevision)},
        {QStringLiteral("training_real_to_virtual_initial_guess_row_major"), initialGuessMatrix},
        {QStringLiteral("source_image"), sourceImageRecord},
        {QStringLiteral("surface_only"), result.surfaceOnly},
        {QStringLiteral("surface_postverification_preflight"), QJsonObject{
            {QStringLiteral("valid"), result.surfaceVerificationPreflight.valid},
            {QStringLiteral("status"), static_cast<int>(result.surfaceVerificationPreflight.status)},
            {QStringLiteral("frozen_patch_count"), result.surfaceVerificationPreflight.frozenPatchCount},
            {QStringLiteral("current_model_supported_count"), result.surfaceVerificationPreflight.currentModelSupportedPointCount},
            {QStringLiteral("current_model_supported_ratio"), result.surfaceVerificationPreflight.currentModelSupportedRatio},
            {QStringLiteral("real_supported_count"), result.surfaceVerificationPreflight.supportedPointCount},
            {QStringLiteral("real_supported_ratio"), result.surfaceVerificationPreflight.supportedRatio},
            {QStringLiteral("maximum_model_continuation_mm"), std::min(10.0, m_options.maximumCorrespondenceDistanceMm)},
            {QStringLiteral("maximum_model_plane_deviation_mm"), std::max(1e-6, std::min(0.05,
                m_options.maximumIndependentWorseningMm * 0.5))},
            {QStringLiteral("semantics"), QStringLiteral("frozen_holdout_supportability_not_fresh_cad_rerender")}}},
        {QStringLiteral("surface_validation"), QJsonObject{
            {QStringLiteral("accepted"), result.surfaceValidation.accepted},
            {QStringLiteral("projected"), result.surfaceValidation.projected},
            {QStringLiteral("observable_rank"), result.surfaceValidation.observableRank},
            {QStringLiteral("relative_mode_strengths"), modeStrengths},
            {QStringLiteral("mode_strength_semantics"), QStringLiteral("per_cloud_sorted_spectrum_minimum_not_common_eigenmode_spectrum")},
            {QStringLiteral("pcl_candidate_virtual_to_real_row_major"), originalSurfaceMatrix},
            {QStringLiteral("training_frame_ids"), trainingFrameIds},
            {QStringLiteral("holdout_frame_ids"), holdoutFrameIds},
            {QStringLiteral("training_before"), surfaceMetricsJson(result.surfaceValidation.trainingBefore)},
            {QStringLiteral("training_after"), surfaceMetricsJson(result.surfaceValidation.trainingAfter)},
            {QStringLiteral("holdout_before"), surfaceMetricsJson(result.surfaceValidation.holdoutBefore)},
            {QStringLiteral("holdout_after"), surfaceMetricsJson(result.surfaceValidation.holdoutAfter)},
            {QStringLiteral("holdout_real_parent_mm"), cloudJson(holdoutReal)},
            {QStringLiteral("holdout_virtual_parent_mm"), cloudJson(holdoutVirtual)},
            {QStringLiteral("safety_reference_keypoints_parent_mm"), cloudJson(result.safetyKeypointsInParent_mm)},
            {QStringLiteral("safety_keypoint_semantics"), QStringLiteral("frozen_model_reference_displacement_budget_not_verified_material_identity")},
            {QStringLiteral("reference_semantics"), QStringLiteral("rigidly_transformed_frozen_laser_slice_approximation_not_cad_rerender")},
            {QStringLiteral("message"), result.surfaceValidation.message}}},
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
        {QStringLiteral("stability_protection"), QJsonObject{
            {QStringLiteral("enabled"), m_options.stabilityProtectionEnabled},
            {QStringLiteral("evaluated"), result.stabilityEvaluated},
            {QStringLiteral("accepted"), result.stabilityEvaluation.accepted},
            {QStringLiteral("reason"), result.stabilityEvaluation.reason},
            {QStringLiteral("before_mean_weld_error_mm"), result.stabilityEvaluation.beforeMeanMm},
            {QStringLiteral("after_mean_weld_error_mm"), result.stabilityEvaluation.afterMeanMm},
            {QStringLiteral("required_gain_mm"), result.stabilityEvaluation.requiredGainMm},
            {QStringLiteral("maximum_historical_error_mm"), result.stabilityEvaluation.maximumHistoricalErrorMm},
            {QStringLiteral("history_enforced"), result.stabilityEvaluation.historyEnforced},
            {QStringLiteral("region_count"), result.stabilityEvaluation.regionCount},
            {QStringLiteral("correspondence_semantics"), QStringLiteral("unverified_laser_seam_intersection_pairs_not_material_identity")},
            {QStringLiteral("evidence_status"), result.stabilityEvaluation.evidenceStatus},
            {QStringLiteral("historical_before_mean_mm"), result.stabilityEvaluation.historyBeforeMeanMm},
            {QStringLiteral("historical_after_mean_mm"), result.stabilityEvaluation.historyAfterMeanMm},
            {QStringLiteral("historical_before_max_mm"), result.stabilityEvaluation.historyBeforeMaxMm},
            {QStringLiteral("historical_after_max_mm"), result.stabilityEvaluation.historyAfterMaxMm},
            {QStringLiteral("conflicting_region_count"), result.stabilityEvaluation.worsenedRegionCount},
            {QStringLiteral("invalid_region_count"), result.stabilityEvaluation.invalidHistoricalRegionCount},
            {QStringLiteral("spatial_span_mm"), result.stabilityEvaluation.spatialSpanMm},
            {QStringLiteral("history_capacity_reached"), validation.stabilityAnchors.size() >= 64U},
            {QStringLiteral("anchors"), stabilityAnchors}}},
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
    QSaveFile file(result.archivePath);
    file.setDirectWriteFallback(false);
    if (!file.open(QIODevice::WriteOnly)) { return fail(file.errorString()); }
    const QByteArray bytes = QJsonDocument(record).toJson(QJsonDocument::Compact);
    if (file.write(bytes) != bytes.size() || !file.commit()) { return fail(file.errorString()); }
    result.archiveSucceeded = true;
    return true;
}

} // namespace i2r_rlc::ui
