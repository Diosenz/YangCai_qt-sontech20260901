/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        LegacyOnlineKcfDetector.cpp
 * @brief       旧在线 KCF 跟踪服务实现
 * @details     连接在线 KCF 跟踪器和旧流程需要的检测结果字段。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <exception>

#include <opencv2/imgproc.hpp>

#include "geometry/WeldImageToSpaceProjector.h"
#include "preprocess/ImageDenoise.h"
#include "stripe/DpStripeCenterlineDetector.h"
#include "track/LegacyOnlineKcfDetector.h"

namespace i2r_rlc::vision {
namespace {

constexpr int kRoiWidthPixel = 250;
constexpr int kRoiHeightPixel = 200;
constexpr int kGapRoiSizePixel = 400;
constexpr int kTemplateLineWidthPixel = 10;
constexpr int kReinitializeIntervalFrame = 50;

MyLaserImagePoint toMyLaserImagePoint(
    const digital_twin::DigitalTwinImagePoint2D_px &point)
{
    MyLaserImagePoint output;
    output.px = point.x_px;
    output.py = point.y_px;
    return output;
}

std::vector<MyLaserImagePoint> toMyLaserImagePoints(
    const std::vector<digital_twin::DigitalTwinImagePoint2D_px> &points)
{
    std::vector<MyLaserImagePoint> output;
    output.reserve(points.size());
    for (const auto &point : points) {
        output.push_back(toMyLaserImagePoint(point));
    }
    return output;
}

cv::Rect clampRoi(const cv::Rect &roi, const cv::Size &size)
{
    if (size.width <= 0 || size.height <= 0) {
        return {};
    }
    const int width = std::min(roi.width, size.width);
    const int height = std::min(roi.height, size.height);
    return cv::Rect(
        std::max(0, std::min(roi.x, size.width - width)),
        std::max(0, std::min(roi.y, size.height - height)),
        width,
        height);
}

cv::Rect centeredRoi(const cv::Point2d &point, int width, int height, const cv::Size &size)
{
    return clampRoi(
        cv::Rect(
            static_cast<int>(std::lround(point.x)) - width / 2,
            static_cast<int>(std::lround(point.y)) - height / 2,
            width,
            height),
        size);
}

cv::Mat simulatedTemplate(
    const cv::Size &size,
    const std::vector<MyLaserImagePoint> &points)
{
    cv::Mat output = cv::Mat::zeros(size, CV_8UC3);
    for (std::size_t index = 1; index < points.size(); ++index) {
        cv::line(
            output,
            cv::Point(
                static_cast<int>(std::lround(points[index - 1].px)),
                static_cast<int>(std::lround(points[index - 1].py))),
            cv::Point(
                static_cast<int>(std::lround(points[index].px)),
                static_cast<int>(std::lround(points[index].py))),
            cv::Scalar(255, 255, 255),
            kTemplateLineWidthPixel,
            cv::LINE_AA);
    }
    return output;
}

/**
 * @brief 按模板内归一化锚点恢复原图焊缝位置，允许 ROI 部分位于图像外。
 */
cv::Point2d weldPointFromRoi(
    const cv::Rect2f &roi, const cv::Point2d &anchor)
{
    return cv::Point2d(
        roi.x + anchor.x * roi.width,
        roi.y + anchor.y * roi.height);
}

cv::Mat enhancedImage(
    const cv::Mat &gray8,
    const cv::Rect &roi,
    const std::vector<MyLaserImagePoint> &simulatedLaserPoints,
    const MyLaserImagePoint &simulatedWeldPoint,
    bool *succeeded)
{
    *succeeded = false;
    cv::Mat output = gray8.clone();
    if (roi.empty()) {
        return output;
    }
    VisionPreprocessStatus status;
    cv::Mat enhanced = CImageDenoise::gaborDenoise(
        gray8(roi).clone(),
        simulatedLaserPoints,
        simulatedWeldPoint,
        &status);
    if (enhanced.empty() || status != VisionPreprocessStatus::Succeeded) {
        return output;
    }
    cv::threshold(enhanced, enhanced, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // 处理图与原图共用坐标，禁止把增强像素平移后再用原图标定投影。
    enhanced.copyTo(output(roi));
    *succeeded = true;
    return output;
}

} // namespace

CLegacyOnlineKcfDetector::CLegacyOnlineKcfDetector()
{
    reset();
}

void CLegacyOnlineKcfDetector::reset()
{
    m_tracker.reset();
    m_templateWeldAnchor = {};
    m_templateImageSize = {};
    m_initialized = false;
    m_framesSinceInitialize = 0;
}

LegacyOnlineKcfResult CLegacyOnlineKcfDetector::process(
    const cv::Mat &gray8,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const WeldProjectionContext &projectionContext,
    const LegacyOnlineKcfOptions &options)
{
    LegacyOnlineKcfResult result;
    result.diagnostic.imageSize = gray8.size();
    if (gray8.empty() || gray8.type() != CV_8UC1) {
        result.message = "input frame is not CV_8UC1";
        return result;
    }
    if (!observation.valid
        || observation.laserLineImagePoints_px.size() < 2U
        || !observation.hasWeldPointImage_px) {
        result.message = "simulated observation is invalid";
        return result;
    }
    if (!projectionContext.valid) {
        result.message = "projection context is invalid";
        return result;
    }

    const auto &weldPoint = observation.weldPointImage_px;
    if (!std::isfinite(weldPoint.x_px) || !std::isfinite(weldPoint.y_px)
        || weldPoint.x_px < 0.0 || weldPoint.x_px >= gray8.cols
        || weldPoint.y_px < 0.0 || weldPoint.y_px >= gray8.rows) {
        result.message = "simulated weld point is outside the source image";
        return result;
    }
    if (m_initialized && m_templateImageSize != gray8.size()) {
        reset();
    }

    if (!m_tracker) {
        m_tracker = std::make_unique<CKcfTracker>(
            true, true, false, false, KcfFeatureMode::Standard, 2);
    }

    const std::vector<MyLaserImagePoint> simulatedLaserPoints =
        toMyLaserImagePoints(observation.laserLineImagePoints_px);
    const MyLaserImagePoint simulatedWeldPoint =
        toMyLaserImagePoint(observation.weldPointImage_px);
    const cv::Point2d simulatedPoint(
        simulatedWeldPoint.px,
        simulatedWeldPoint.py);
    const cv::Rect simulationRoi = centeredRoi(
        simulatedPoint, kRoiWidthPixel, kRoiHeightPixel, gray8.size());
    if (simulationRoi.empty()) {
        result.message = "simulated ROI is empty";
        return result;
    }

    const cv::Rect2f previous = m_tracker->m_roi;
    const bool wasInitialized = m_initialized;
    if (!m_initialized || m_framesSinceInitialize > kReinitializeIntervalFrame) {
        result.diagnostic.templateReinitialized = true;
        const cv::Point2d previousWeldPoint =
            weldPointFromRoi(previous, m_templateWeldAnchor);
        m_tracker->init(
            simulationRoi,
            simulatedTemplate(gray8.size(), simulatedLaserPoints));
        // 边缘窗口已被裁到原图内，焊缝不再位于窗口中心。
        // 以 tracker 实际初始化后的 ROI 为准，保存焊缝的模板内坐标。
        m_templateWeldAnchor = cv::Point2d(
            (simulatedPoint.x - m_tracker->m_roi.x) / m_tracker->m_roi.width,
            (simulatedPoint.y - m_tracker->m_roi.y) / m_tracker->m_roi.height);
        m_templateImageSize = gray8.size();
        m_initialized = true;
        m_framesSinceInitialize = 0;
        if (wasInitialized) {
            // 重建后锚点可能改变，保留上次焊缝位置而非旧窗口左上角。
            m_tracker->m_roi.x = static_cast<float>(previousWeldPoint.x
                - m_templateWeldAnchor.x * m_tracker->m_roi.width);
            m_tracker->m_roi.y = static_cast<float>(previousWeldPoint.y
                - m_templateWeldAnchor.y * m_tracker->m_roi.height);
        }
    }

    // Gabor 只处理图内交集；不为方便裁图而移动 KCF 的坐标原点。
    const cv::Rect currentRoi = cv::Rect(
            static_cast<int>(std::lround(m_tracker->m_roi.x)),
            static_cast<int>(std::lround(m_tracker->m_roi.y)),
            static_cast<int>(std::lround(m_tracker->m_roi.width)),
            static_cast<int>(std::lround(m_tracker->m_roi.height)))
        & cv::Rect(0, 0, gray8.cols, gray8.rows);
    bool gaborSucceeded = false;
    result.processedImage = enhancedImage(
        gray8, currentRoi, simulatedLaserPoints, simulatedWeldPoint, &gaborSucceeded);
    m_tracker->m_Kalman_status_x =
        simulatedWeldPoint.px - m_tracker->m_roi.width * m_templateWeldAnchor.x;
    m_tracker->m_Kalman_status_y =
        simulatedWeldPoint.py - m_tracker->m_roi.height * m_templateWeldAnchor.y;

    result.diagnostic.attempted = true;
    result.diagnostic.gaborSucceeded = gaborSucceeded;
    result.diagnostic.enhancementRoi = currentRoi;
    result.diagnostic.templateAnchor = m_templateWeldAnchor;
    result.diagnostic.simulatedPoint = simulatedPoint;
    result.diagnostic.framesSinceInitialize = m_framesSinceInitialize;
    try {
        result.trackedRoi = m_tracker->update(result.processedImage);
        ++m_framesSinceInitialize;
    } catch (const std::exception &exception) {
        result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
        result.message = std::string("KCF exception: ") + exception.what();
        return result;
    }

    result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
    result.diagnostic.responsePoint = weldPointFromRoi(
        result.diagnostic.tracker.responseRoi, m_templateWeldAnchor);
    result.diagnostic.beforeFusionPoint = weldPointFromRoi(
        result.diagnostic.tracker.beforeFusionRoi, m_templateWeldAnchor);
    result.weldPointPixel = weldPointFromRoi(m_tracker->m_roi, m_templateWeldAnchor);
    result.diagnostic.finalPoint = result.weldPointPixel;
    if (!std::isfinite(result.weldPointPixel.x)
        || !std::isfinite(result.weldPointPixel.y)
        || result.weldPointPixel.x < 0.0 || result.weldPointPixel.x >= gray8.cols
        || result.weldPointPixel.y < 0.0 || result.weldPointPixel.y >= gray8.rows) {
        result.message = "tracked weld anchor is outside the source image";
        reset();
        return result;
    }
    const auto projection = CWeldImageToSpaceProjector::projectCvPoint(
        result.weldPointPixel,
        projectionContext,
        WeldProjectionTargetFrame::PositionerTcp);
    if (!projection.valid || !projection.spacePoint.valid) {
        result.message = "KCF succeeded but station projection failed: " + projection.message;
        return result;
    }
    result.stationPoint = projection.spacePoint;
    result.valid = true;

    if (options.enableGapDetection) {
        DpStripeDetectionOptions centerlineOptions;
        centerlineOptions.roi = centeredRoi(
            result.weldPointPixel,
            kGapRoiSizePixel,
            kGapRoiSizePixel,
            gray8.size());
        centerlineOptions.seamXPixel =
            static_cast<int>(std::lround(result.weldPointPixel.x))
            - centerlineOptions.roi.x;
        const auto centerline =
            CDpStripeCenterlineDetector::detectCenterline(gray8, centerlineOptions);
        WeldPointCloud cloud;
        cloud.frameId = "positioner_tcp";
        if (centerline.valid) {
            std::vector<WeldImagePoint> imagePoints;
            imagePoints.reserve(centerline.points.size());
            for (const auto &point : centerline.points) {
                imagePoints.push_back(WeldImagePoint{
                    point.xPixel, point.yPixel, point.valid});
            }
            cloud.points = CWeldImageToSpaceProjector::projectImagePoints(
                imagePoints,
                projectionContext,
                WeldProjectionTargetFrame::PositionerTcp);
            cloud.valid = std::any_of(
                cloud.points.begin(), cloud.points.end(),
                [](const WeldSpacePoint &point) { return point.valid; });
        }
        const auto gap = CPointCloudGapWidthDetector::detect(
            cloud, result.stationPoint, options.gapWidthOptions);
        result.gapValid = gap.valid;
        result.gapWidthMm = gap.valid ? gap.gapWidthMm : 0.0;
        result.gapConfidence = gap.valid ? gap.confidence : 0.0;
        result.message = gap.valid
            ? "KCF/Gabor and gap detection succeeded"
            : "KCF/Gabor succeeded; gap degraded to 0 mm: " + gap.message;
    } else {
        result.message = gaborSucceeded
            ? "KCF/Gabor detection succeeded"
            : "KCF succeeded; Gabor enhancement unavailable";
    }
    return result;
}

} // namespace i2r_rlc::vision
