/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        LegacyOnlineKcfDetector.cpp
 * @brief       旧在线 KCF 跟踪服务实现
 * @details     连接在线 KCF 跟踪器和旧流程需要的检测结果字段。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <chrono>
#include <exception>
#include <limits>

#include <opencv2/imgproc.hpp>

#include "feature/ActiveContourWeldFeatureDetector.h"
#include "geometry/WeldImageToSpaceProjector.h"
#include "preprocess/ImageDenoise.h"
#include "stripe/DpStripeCenterlineDetector.h"
#include "track/LegacyOnlineKcfDetector.h"

namespace i2r_rlc::vision {
namespace {

/** @brief 阶段切换先结算旧区间；外部快照在返回值拷贝前完成，覆盖提前退出。 */
class CDetectorStageClock {
public:
    CDetectorStageClock(bool enabled, COnlineKcfPerformanceDiagnostic &performance)
        : m_enabled(enabled), m_performance(performance),
          m_stage(&performance.inputBookkeepingMs)
    {
        if (m_enabled) {
            m_performance.available = true;
            m_started = std::chrono::steady_clock::now();
            m_previous = m_started;
        }
    }

    ~CDetectorStageClock()
    {
        if (m_enabled) {
            const auto finished = std::chrono::steady_clock::now();
            accumulate(finished);
            m_performance.totalMs = std::chrono::duration<double, std::milli>(
                finished - m_started).count();
        }
    }

    /** @brief 同一阶段可重复进入，初始化或恢复中的响应不会重复计入校核阶段。 */
    void enter(double &stage)
    {
        if (m_enabled) {
            accumulate(std::chrono::steady_clock::now());
            m_stage = &stage;
        }
    }

private:
    void accumulate(std::chrono::steady_clock::time_point now)
    {
        *m_stage += std::chrono::duration<double, std::milli>(now - m_previous).count();
        m_previous = now;
    }

    bool m_enabled;
    COnlineKcfPerformanceDiagnostic &m_performance;
    double *m_stage;
    std::chrono::steady_clock::time_point m_started;
    std::chrono::steady_clock::time_point m_previous;
};

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

/**
 * @brief 检查粗 KCF 附近原图的唯一窄峰支撑，避免增强相关峰在遮挡时自证。
 * @details 只产生业务估计权重；局部支撑不证明接缝材料身份，不能作为独立焊点。
 * 大窗口含竞争条纹时仍可保留局部唯一且时序相容的低权重粗观测。
 */
double coarseObservationQuality(const cv::Mat &gray8, const cv::Point2d &coarse,
    const CKcfUpdateDiagnostic &tracker, const OnlineSeamRefinementResult &raw)
{
    if (raw.status == OnlineSeamRefinementStatus::InvalidInput
        || !std::isfinite(raw.ambiguousColumnRatio)
        || !std::isfinite(coarse.x) || !std::isfinite(coarse.y)
        || coarse.x < 0.0 || coarse.x >= gray8.cols
        || coarse.y < 0.0 || coarse.y >= gray8.rows) { return 0.0; }
    const int centerX = static_cast<int>(std::lround(coarse.x));
    const int centerY = static_cast<int>(std::lround(coarse.y));
    const int xBegin = std::max(0, centerX - 16);
    const int xEnd = std::min(gray8.cols - 1, centerX + 16);
    const int yBegin = std::max(1, centerY - 24);
    const int yEnd = std::min(gray8.rows - 2, centerY + 24);
    if (xEnd - xBegin < 16 || yEnd - yBegin < 16) { return 0.0; }
    int supported = 0;
    int ambiguous = 0;
    const auto smooth = [&gray8](int x, int y) {
        return (gray8.at<unsigned char>(y - 1, x)
            + gray8.at<unsigned char>(y, x)
            + gray8.at<unsigned char>(y + 1, x)) / 3.0;
    };
    for (int x = xBegin; x <= xEnd; ++x) {
        double background = 255.0;
        double peak = 0.0;
        int peakY = yBegin;
        for (int y = yBegin; y <= yEnd; ++y) {
            background = std::min(background,
                static_cast<double>(gray8.at<unsigned char>(y, x)));
            const double value = smooth(x, y);
            if (value > peak) { peak = value; peakY = y; }
        }
        if (peak - background < 40.0) { continue; }
        const double halfHeight = background + (peak - background) * 0.5;
        int top = peakY;
        int bottom = peakY;
        while (top > yBegin && gray8.at<unsigned char>(top - 1, x) > halfHeight) { --top; }
        while (bottom < yEnd && gray8.at<unsigned char>(bottom + 1, x) > halfHeight) { ++bottom; }
        if (bottom - top + 1 > 12) { continue; }
        bool competing = false;
        for (int y = yBegin + 1; y < yEnd; ++y) {
            if (std::abs(y - peakY) < 8) { continue; }
            const double value = smooth(x, y);
            if (value - background >= (peak - background) * 0.8
                && value >= smooth(x, y - 1) && value >= smooth(x, y + 1)) {
                competing = true;
                break;
            }
        }
        if (competing) { ++ambiguous; }
        else { ++supported; }
    }
    const double columns = xEnd - xBegin + 1.0;
    if (supported < 16 || supported / columns < 0.6
        || ambiguous / columns > 0.2) { return 0.0; }
    const double responseQuality = std::clamp((tracker.responsePeak - 0.35) / 0.45, 0.0, 1.0);
    const double ambiguityPenalty = 1.0 / (1.0 + 3.0 * raw.ambiguousColumnRatio);
    return std::clamp(responseQuality * supported / columns * ambiguityPenalty, 0.0, 1.0);
}

bool insideImage(const cv::Mat &gray8, const cv::Point2d &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y)
        && point.x >= 0.0 && point.x < gray8.cols
        && point.y >= 0.0 && point.y < gray8.rows;
}

/** @brief 仿真局部外观及其取样坐标；渲染锚点不是当前图像测得的位置。 */
struct PaperSimulationAppearance {
    cv::Mat image;
    cv::Rect roi;
    cv::Point2d anchor;
};

/**
 * @brief 将仿真局部形状平移到指定取样锚点，不要求仿真绝对焊点落在实拍图内。
 * @details 只改变外观采样坐标；不创建图像观测、独立几何或永久虚实位姿修正。
 * 原始仿真位置保留在诊断中，业务输出仍须由后续当前图像相关及身份验收产生。
 */
PaperSimulationAppearance makePaperSimulationAppearance(
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const cv::Size &size, const cv::Point2d &anchor)
{
    PaperSimulationAppearance appearance;
    const cv::Point2d origin(observation.weldPointImage_px.x_px,
        observation.weldPointImage_px.y_px);
    if (!observation.valid || !observation.hasWeldPointImage_px
        || !std::isfinite(origin.x) || !std::isfinite(origin.y)
        || observation.laserLineImagePoints_px.size() < 2
        || !std::isfinite(anchor.x) || !std::isfinite(anchor.y)) { return appearance; }
    cv::Mat image = cv::Mat::zeros(size, CV_8UC1);
    int segmentCount = 0;
    const auto &points = observation.laserLineImagePoints_px;
    const double coordinateLimit = std::numeric_limits<int>::max() / 4.0;
    for (std::size_t index = 1; index < points.size(); ++index) {
        const cv::Point2d first(points[index - 1].x_px - origin.x + anchor.x,
            points[index - 1].y_px - origin.y + anchor.y);
        const cv::Point2d second(points[index].x_px - origin.x + anchor.x,
            points[index].y_px - origin.y + anchor.y);
        if (!std::isfinite(first.x) || !std::isfinite(first.y)
            || !std::isfinite(second.x) || !std::isfinite(second.y)
            || std::abs(first.x) > coordinateLimit || std::abs(first.y) > coordinateLimit
            || std::abs(second.x) > coordinateLimit || std::abs(second.y) > coordinateLimit) {
            continue;
        }
        cv::line(image, cv::Point(static_cast<int>(std::lround(first.x)),
            static_cast<int>(std::lround(first.y))),
            cv::Point(static_cast<int>(std::lround(second.x)),
                static_cast<int>(std::lround(second.y))), cv::Scalar(220), 2, cv::LINE_AA);
        ++segmentCount;
    }
    if (segmentCount == 0 || cv::countNonZero(image) == 0) { return appearance; }
    cv::GaussianBlur(image, image, cv::Size(5, 5), 0.8);
    image += cv::Scalar(15);
    appearance.image = image;
    appearance.roi = centeredRoi(anchor, kRoiWidthPixel, kRoiHeightPixel, size);
    appearance.anchor = anchor;
    return appearance;
}

/** @brief 固定原图支撑范围用于身份比较，不从反光增强图或 CAD 学习模板。 */
cv::Mat imageIdentityPatch(const cv::Mat &gray8, const cv::Point2d &point)
{
    constexpr int patchSize = 81;
    if (!insideImage(gray8, point) || point.x < patchSize / 2
        || point.y < patchSize / 2 || point.x + patchSize / 2 >= gray8.cols
        || point.y + patchSize / 2 >= gray8.rows) {
        return {};
    }
    cv::Mat patch;
    cv::getRectSubPix(gray8, {patchSize, patchSize}, point, patch);
    return patch;
}

/**
 * @brief 初始化时在整幅原图搜索仿真局部外观的平移提案，避免把 CAD 坐标当位置真值。
 * @details 只提供后续 MKCF 搜索/取样锚点，不创建观测或控制资格；
 * 提案仍须通过当前相关响应、原图身份及投影验收，不能写回工件位姿。
 */
cv::Point2d findPaperSimulationSearchAnchor(const cv::Mat &gray8,
    const PaperSimulationAppearance &appearance)
{
    const auto shape = imageIdentityPatch(appearance.image, appearance.anchor);
    if (shape.empty() || shape.cols > gray8.cols || shape.rows > gray8.rows) {
        return {-1.0, -1.0};
    }
    cv::Mat response;
    cv::matchTemplate(gray8, shape, response, cv::TM_CCOEFF_NORMED);
    double maximum = 0.0;
    cv::Point location;
    cv::minMaxLoc(response, nullptr, &maximum, nullptr, &location);
    // 相关峰只提供取样位置，不以未经标定的虚实灰度分数撤销有效仿真模板。
    if (!std::isfinite(maximum)) { return {-1.0, -1.0}; }
    return {location.x + (shape.cols - 1) * 0.5, location.y + (shape.rows - 1) * 0.5};
}

/**
 * @brief 复用已迁移主动轮廓提供无仿真外观时的初始化提案，不带历史备用。
 * @details 新局部状态只消费当前原图；交点仅初始化跟踪器，
 * 凹角提案在纵向反射原图中复验窄条纹与稳定转折，再映射回原坐标。
 * 该确认与后续 MKCF 响应、身份及投影验收分离，不授予独立几何资格。
 */
cv::Point2d findPaperImageBootstrapPoint(const cv::Mat &gray8,
    const cv::Point2d &appearanceProposal,
    bool confirmReflectedProposal,
    const OnlineSeamRefinementOptions &refinementOptions,
    CLegacyOnlineKcfDiagnostic &diagnostic, cv::Point2d *confirmedPoint)
{
    ActiveContourDetectionState state;
    const ActiveContourDetectionOptions options;
    const auto candidate = CActiveContourWeldFeatureDetector::detect(
        gray8, ActiveContourSeamMode::TJoint, options, &state);
    const cv::Point2d contourPoint(candidate.weldPoint.xPixel, candidate.weldPoint.yPixel);
    const bool contourValid = candidate.valid && !candidate.usedHistoryFallback
        && insideImage(gray8, contourPoint) && candidate.fittedLines.size() >= 2;
    const auto point = contourValid ? contourPoint : appearanceProposal;
    if (!insideImage(gray8, point)) { return {-1.0, -1.0}; }
    const double leftSlope = contourValid ? candidate.fittedLines.front().slope : 0.0;
    const double rightSlope = contourValid ? candidate.fittedLines.back().slope : 0.0;
    diagnostic.imageSearchDiagnostic += "bootstrap=" + std::to_string(point.x) + ","
        + std::to_string(point.y) + ";slopes=" + std::to_string(leftSlope) + ","
        + std::to_string(rightSlope) + "|";
    if (confirmReflectedProposal
        && (!contourValid || (leftSlope < -0.05 && rightSlope > 0.05))) {
        cv::Mat reflected;
        cv::flip(gray8, reflected, 0);
        const auto raw = COnlineSeamRefiner::refine(reflected,
            {point.x, gray8.rows - 1.0 - point.y}, refinementOptions);
        ++diagnostic.imageRefinementCallCount;
        diagnostic.imageSearchDiagnostic += "reflected_status="
            + std::to_string(static_cast<int>(raw.status)) + "|";
        if (raw.valid) {
            const cv::Point2d measured(raw.point.x, gray8.rows - 1.0 - raw.point.y);
            *confirmedPoint = cv::norm(measured - point)
                    <= refinementOptions.maxPositionUncertaintyPixel ? point : measured;
        }
    }
    return point;
}

/**
 * @brief KCF 响应以外的原图身份校核：相关形状和可见窄条纹必须同时保留。
 * @details 参照来自当前实拍初始化或已接纳的实拍更新；本校核不授予独立几何资格。
 * 缺少参照、空白、宽饱和及大范围竞争反光均不接纳。
 */
double imageIdentityQuality(const cv::Mat &gray8, const cv::Point2d &point,
    const cv::Mat &identity)
{
    const auto patch = imageIdentityPatch(gray8, point);
    if (patch.empty() || identity.empty() || patch.size() != identity.size()) {
        return 0.0;
    }
    double minimum = 0.0, maximum = 0.0;
    double anchorMinimum = 0.0, anchorMaximum = 0.0;
    cv::minMaxLoc(patch, &minimum, &maximum);
    cv::minMaxLoc(identity, &anchorMinimum, &anchorMaximum);
    if (maximum - minimum < 40.0 || anchorMaximum - anchorMinimum < 40.0) {
        return 0.0;
    }
    cv::Mat currentMask, anchorMask;
    cv::threshold(patch, currentMask, minimum + 0.4 * (maximum - minimum), 255,
        cv::THRESH_BINARY);
    cv::threshold(identity, anchorMask,
        anchorMinimum + 0.4 * (anchorMaximum - anchorMinimum), 255, cv::THRESH_BINARY);
    const int support = cv::countNonZero(anchorMask);
    const int currentSupport = cv::countNonZero(currentMask);
    if (support < 32 || currentSupport < support * 0.5
        || currentSupport > support * 2.0) {
        return 0.0;
    }
    cv::Mat shared;
    cv::bitwise_and(anchorMask, currentMask, shared);
    const double overlap = static_cast<double>(cv::countNonZero(shared)) / support;
    if (overlap < 0.6) {
        return 0.0;
    }
    cv::Mat response;
    cv::matchTemplate(patch, identity, response, cv::TM_CCOEFF_NORMED);
    const double correlation = response.at<float>(0, 0);
    return std::isfinite(correlation) && correlation >= 0.7
        ? std::clamp(correlation * overlap, 0.0, 1.0) : 0.0;
}

/** @brief 模板中心不参与双肩部位置约束；中心支持单独决定是否仍为完整当前观测。 */
struct ShoulderMatch {
    bool valid = false;
    cv::Point2d point{-1.0, -1.0};
    double quality = 0.0;
    double centerQuality = 0.0;
};

/**
 * @brief 在相关响应附近，用既有实拍身份模板的两外侧匹配当前母材条纹。
 * @details 只校正 MKCF 的局部图像平移，中心预焊形变不参与位置匹配；
 * 两侧与中心分别核验。无条纹、单侧或宽反光不产生有效位置，不使用仿真绝对点。
 */
ShoulderMatch matchImageShoulders(const cv::Mat &gray8, const cv::Point2d &coarse,
    const cv::Point2d &imagePrediction, const cv::Mat &identity, bool stripeSupportOnly)
{
    ShoulderMatch result;
    if (identity.empty() || identity.type() != CV_8UC1
        || !insideImage(gray8, coarse)) { return result; }
    const int halfWidth = identity.cols / 2;
    const int halfHeight = identity.rows / 2;
    constexpr int searchRadius = 12;
    const cv::Rect search(static_cast<int>(std::lround(coarse.x)) - halfWidth - searchRadius,
        static_cast<int>(std::lround(coarse.y)) - halfHeight - searchRadius,
        identity.cols + searchRadius * 2, identity.rows + searchRadius * 2);
    if ((search & cv::Rect(0, 0, gray8.cols, gray8.rows)) != search) { return result; }
    cv::Mat source, reference;
    gray8(search).convertTo(source, CV_32F);
    identity.convertTo(reference, CV_32F);
    double minimum = 0.0, maximum = 0.0;
    cv::minMaxLoc(source, &minimum, &maximum);
    if (maximum - minimum < 40.0) { return result; }
    source -= minimum;
    cv::minMaxLoc(reference, &minimum, &maximum);
    if (maximum - minimum < 40.0) { return result; }
    reference -= minimum;
    // 匹配母材条纹本身的支持，额外亮线的矩形区域能量不得压低真实支系。
    // 完整外观/防污染仍单独检查新增亮区；此掩码只负责当前位置的双侧约束。
    cv::Mat stripeSupport;
    cv::threshold(reference, stripeSupport, 0.4 * (maximum - minimum), 1.0, cv::THRESH_BINARY);
    cv::Mat shoulders = cv::Mat::zeros(identity.size(), CV_32F);
    const int shoulderWidth = std::max(1, identity.cols / 4);
    shoulders(cv::Rect(0, 0, shoulderWidth, identity.rows)).setTo(1.0F);
    shoulders(cv::Rect(identity.cols - shoulderWidth, 0, shoulderWidth, identity.rows)).setTo(1.0F);
    if (stripeSupportOnly) { cv::multiply(shoulders, stripeSupport, shoulders); }
    cv::Mat response;
    cv::matchTemplate(source, reference, response, cv::TM_CCORR_NORMED, shoulders);
    // 多条亮线会使矩形归一化响应的几个峰近乎等价；亮度不决定目标身份。
    // 仅在本帧双肩均获支持的局部峰中，按最近可靠图像运动消歧。
    // 预测只排序候选，不作为位置输出，也不借用 CAD 平移。
    cv::Mat candidates = response.clone();
    // 只对原响应中的真实局部峰排序；逐次挖去峰邻域后产生的斜坡边界
    // 不是新候选，否则偏移的旧估计会抢占已恢复的清晰匹配峰。
    cv::Mat localMaximum, nonPeak;
    cv::dilate(response, localMaximum, cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));
    cv::compare(response, localMaximum, nonPeak, cv::CMP_LT);
    candidates.setTo(-1.0F, nonPeak);
    cv::Point peak;
    double bestInnovation = std::numeric_limits<double>::max();
    double left = 0.0, right = 0.0, centerQuality = 0.0;
    bool found = false;
    for (int candidate = 0; candidate < 24; ++candidate) {
        cv::Point location;
        double score = 0.0;
        cv::minMaxLoc(candidates, nullptr, &score, nullptr, &location);
        if (!std::isfinite(score) || score < 0.7) { break; }
        const cv::Rect exclusion = cv::Rect(location.x - 3, location.y - 3, 7, 7)
            & cv::Rect(0, 0, response.cols, response.rows);
        candidates(exclusion).setTo(-1.0F);
        const cv::Mat matched = source(cv::Rect(location.x, location.y, identity.cols, identity.rows));
        const auto maskedQuality = [&](const cv::Rect &region) {
            cv::Mat mask = cv::Mat::zeros(identity.size(), CV_32F);
            mask(region).setTo(1.0F);
            if (stripeSupportOnly) { cv::multiply(mask, stripeSupport, mask); }
            cv::Mat local;
            cv::matchTemplate(matched, reference, local, cv::TM_CCORR_NORMED, mask);
            const double value = local.at<float>(0, 0);
            return std::isfinite(value) ? value : 0.0;
        };
        const double candidateLeft = maskedQuality({0, 0, shoulderWidth, identity.rows});
        const double candidateRight = maskedQuality(
            {identity.cols - shoulderWidth, 0, shoulderWidth, identity.rows});
        if (candidateLeft < 0.7 || candidateRight < 0.7) { continue; }
        const cv::Point2d point(search.x + location.x + halfWidth,
            search.y + location.y + halfHeight);
        const double innovation = cv::norm(point - imagePrediction);
        if (innovation >= bestInnovation) { continue; }
        found = true;
        bestInnovation = innovation;
        peak = location;
        left = candidateLeft;
        right = candidateRight;
        // 中心资格只检查锚点附近；把两侧母材也计入中心会掩盖实际缺失。
        const int centerWidth = shoulderWidth;
        centerQuality = maskedQuality({(identity.cols - centerWidth) / 2, 0,
            centerWidth, identity.rows});
    }
    if (!found) { return result; }
    // 身份块会递推更新，整数峰每帧的同向量化误差会被反复学入模板。
    // 沿用相关滤波的三点抛物线细化，不将历史平滑或仿真位置写回当前观测。
    const auto peakOffset = [](float previous, float center, float next) {
        const double curvature = 2.0 * center - previous - next;
        return curvature > 1e-9 ? std::clamp(
            0.5 * (next - previous) / curvature, -0.5, 0.5) : 0.0;
    };
    double offsetX = 0.0, offsetY = 0.0;
    if (peak.x > 0 && peak.x + 1 < response.cols) {
        offsetX = peakOffset(response.at<float>(peak.y, peak.x - 1),
            response.at<float>(peak.y, peak.x), response.at<float>(peak.y, peak.x + 1));
    }
    if (peak.y > 0 && peak.y + 1 < response.rows) {
        offsetY = peakOffset(response.at<float>(peak.y - 1, peak.x),
            response.at<float>(peak.y, peak.x), response.at<float>(peak.y + 1, peak.x));
    }
    result.valid = true;
    result.point = cv::Point2d(search.x + peak.x + halfWidth + offsetX,
        search.y + peak.y + halfHeight + offsetY);
    result.quality = std::min(left, right);
    result.centerQuality = centerQuality;
    return result;
}

/**
 * @brief 完整实拍训练比位置续接更严格：拒绝模板窄条纹之外的新增亮支系。
 * @details 沿用身份块的0.4动态阈值，1px容差只吸收亚像素采样与离散化误差；
 * 当前匹配不支持的亮区不能写入可信模板，位置输出仍独立使用肩部约束。
 */
bool completeImageAppearance(const cv::Mat &gray8, const cv::Point2d &point,
    const cv::Mat &identity)
{
    const auto current = imageIdentityPatch(gray8, point);
    if (current.empty() || identity.empty() || current.size() != identity.size()) { return false; }
    double minimum = 0.0, maximum = 0.0;
    cv::minMaxLoc(current, &minimum, &maximum);
    if (maximum - minimum < 40.0) { return false; }
    cv::Mat currentMask, referenceMask;
    cv::threshold(current, currentMask, minimum + 0.4 * (maximum - minimum), 255, cv::THRESH_BINARY);
    cv::minMaxLoc(identity, &minimum, &maximum);
    if (maximum - minimum < 40.0) { return false; }
    cv::threshold(identity, referenceMask, minimum + 0.4 * (maximum - minimum), 255, cv::THRESH_BINARY);
    const int support = cv::countNonZero(referenceMask);
    if (support < 32) { return false; }
    cv::Mat tolerantReference, tolerantCurrent;
    const auto kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::dilate(referenceMask, tolerantReference, kernel);
    cv::dilate(currentMask, tolerantCurrent, kernel);
    cv::Mat unsupported, missing;
    cv::bitwise_not(tolerantReference, unsupported);
    cv::bitwise_and(currentMask, unsupported, unsupported);
    cv::bitwise_not(tolerantCurrent, missing);
    cv::bitwise_and(referenceMask, missing, missing);
    return cv::countNonZero(unsupported) <= support * 0.1
        && cv::countNonZero(missing) <= support * 0.1;
}

/**
 * @brief 核对原身份条纹是否仍可见，允许额外反光条纹，但不授予完整实拍训练资格。
 * @details 与完整外观使用同一亮度支持及 1px 离散容差；缺失的目标支系不能靠相关高分补足。
 */
bool imageIdentityStripeSupported(const cv::Mat &gray8, const cv::Point2d &point,
    const cv::Mat &identity)
{
    const auto current = imageIdentityPatch(gray8, point);
    if (current.empty() || identity.empty() || current.size() != identity.size()) { return false; }
    double minimum = 0.0, maximum = 0.0;
    cv::minMaxLoc(current, &minimum, &maximum);
    if (maximum - minimum < 40.0) { return false; }
    cv::Mat currentMask, referenceMask;
    cv::threshold(current, currentMask, minimum + 0.4 * (maximum - minimum),
        255, cv::THRESH_BINARY);
    cv::minMaxLoc(identity, &minimum, &maximum);
    if (maximum - minimum < 40.0) { return false; }
    cv::threshold(identity, referenceMask, minimum + 0.4 * (maximum - minimum),
        255, cv::THRESH_BINARY);
    const int support = cv::countNonZero(referenceMask);
    if (support < 32) { return false; }
    cv::Mat tolerantCurrent, missing;
    cv::dilate(currentMask, tolerantCurrent,
        cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::bitwise_not(tolerantCurrent, missing);
    cv::bitwise_and(referenceMask, missing, missing);
    return cv::countNonZero(missing) <= support * 0.1;
}

struct ImageSearchResult {
    OnlineSeamRefinementResult geometry;
    bool ambiguous = false;
    int refinementCallCount = 0;
    double proposalGenerationMs = 0.0;
};

/**
 * @brief 搜索中心仅产生提案；每个输出必须由既有原图双侧几何独立定位。
 * @details 保留原 refiner 的相机截面语义，不根据 CAD 方向改变几何判据。
 * 多个分离的合法接缝候选不能以 CAD 距离或亮度决定身份。
 */
ImageSearchResult searchImageGeometry(const cv::Mat &gray8,
    const std::vector<cv::Point2d> &hints, bool expanded,
    const OnlineSeamRefinementOptions &options)
{
    ImageSearchResult result;
    result.geometry.status = OnlineSeamRefinementStatus::InsufficientSupport;
    std::vector<cv::Point2d> proposals = hints;
    if (expanded) {
        const auto proposalStarted = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(gray8, corners, 32, 0.02, 24.0, cv::noArray(), 9);
        for (const auto &corner : corners) {
            proposals.push_back(corner);
        }
        // 对比度差异只影响提案，不改变最终测量原图及几何门禁。
        // 二值条纹提案避免远端饱和反光独占角点预算，弱但清晰的真实折点仍可被搜索。
        cv::Mat proposalImage;
        cv::GaussianBlur(gray8, proposalImage, {3, 3}, 0.8);
        cv::threshold(proposalImage, proposalImage, 0.0, 255.0,
            cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::goodFeaturesToTrack(proposalImage, corners, 32, 0.02, 24.0, cv::noArray(), 9);
        for (const auto &corner : corners) {
            proposals.push_back(corner);
        }
        proposals.push_back({0.5 * gray8.cols, 0.5 * gray8.rows});
        result.proposalGenerationMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - proposalStarted).count();
    }
    std::vector<cv::Point2d> searched;
    for (const auto &proposal : proposals) {
        if (!insideImage(gray8, proposal)) {
            continue;
        }
        if (std::any_of(searched.begin(), searched.end(), [&proposal](const auto &point) {
            return cv::norm(point - proposal) < 12.0;
        })) {
            continue;
        }
        searched.push_back(proposal);
        ++result.refinementCallCount;
        const auto candidate = COnlineSeamRefiner::refine(gray8, proposal, options);
        if (!candidate.valid) {
            // 失测也保留原图拒绝证据；不得将有歧义的图像统一写成非法输入。
            if (!result.geometry.valid
                && ((candidate.status == OnlineSeamRefinementStatus::AmbiguousStripe
                        && candidate.ambiguousColumnRatio > result.geometry.ambiguousColumnRatio)
                    || (result.geometry.status != OnlineSeamRefinementStatus::AmbiguousStripe
                        && candidate.supportRatio > result.geometry.supportRatio))) {
                result.geometry = candidate;
            }
            continue;
        }
        if (result.geometry.valid && cv::norm(candidate.point - result.geometry.point)
            > std::max(3.0, options.maxPositionUncertaintyPixel)) {
            result.ambiguous = true;
            result.geometry.valid = false;
            result.geometry.status = OnlineSeamRefinementStatus::AmbiguousStripe;
            return result;
        }
        if (!result.geometry.valid || candidate.confidence > result.geometry.confidence) {
            result.geometry = candidate;
        }
    }
    return result;
}

} // namespace

CLegacyOnlineKcfDetector::CLegacyOnlineKcfDetector()
{
    reset();
}

void CLegacyOnlineKcfDetector::reset()
{
    m_paperPosition = {};
    m_tracker.reset();
    m_paperImagePoint = {-1.0, -1.0};
    m_paperImageVelocity = {};
    m_hasPaperImageMotion = false;
    m_paperResponseStatistics = {};
    m_paperMotionHasGeometry = false;
    m_continuity.reset();
    m_evidence.reset();
    m_guidedEstimator.reset();
    m_guidanceRevisionKnown = false;
    m_continuitySeamIndex = -1;
    m_templateWeldAnchor = {};
    m_templateImageSize = {};
    m_initialized = false;
    m_framesSinceInitialize = 0;
    m_onlineObservationActive = false;
    m_hasImageSearchPoint = false;
    m_lastImageSearchPoint = {-1.0, -1.0};
    m_coarseRecoveryPending = false;
    m_imageMeasurementActive = false;
    m_imageIdentityPatch.release();
    m_imageReacquisitionPoint = {-1.0, -1.0};
    m_imageReacquisitionFrames = 0;
    m_imageLastObservationFrameId = -1;
    m_boundedImageSearch = {};
    m_boundedImageSearchAmbiguous = false;
}

void CLegacyOnlineKcfDetector::restartTracking()
{
    const auto statistics = m_paperResponseStatistics;
    const auto imageSize = m_templateImageSize;
    const auto seamIndex = m_continuitySeamIndex;
    reset();
    m_paperResponseStatistics = statistics;
    m_templateImageSize = imageSize;
    m_continuitySeamIndex = seamIndex;
}

void CLegacyOnlineKcfDetector::notifyImageObservationGap()
{
    m_paperImageVelocity = {};
    m_hasPaperImageMotion = false;
    m_paperPosition = {};
    m_evidence.notifyGap();
    m_imageReacquisitionPoint = {-1.0, -1.0};
    m_imageReacquisitionFrames = 0;
    m_boundedImageSearch = {};
    m_boundedImageSearchAmbiguous = false;
    if (m_tracker) {
        m_tracker->commitOnlineObservationTraining(false);
    }
}

OnlineSeamRefinementResult CLegacyOnlineKcfDetector::searchBoundedImageGeometry(
    const cv::Mat &gray8, const std::vector<cv::Point2d> &hints,
    const LegacyOnlineKcfOptions &options,
    CLegacyOnlineKcfDiagnostic &diagnostic, bool coarseAccepted)
{
    OnlineSeamRefinementResult missing;
    missing.status = OnlineSeamRefinementStatus::InsufficientSupport;
    m_boundedImageSearchAmbiguous = false;
    const auto rejectCandidate = [this]() {
        // 失效的是当前候选，不是尚未检查的搜索提案。保留游标可避免运动/
        // 短暂遮挡使整轮反复从头开始；缓存仍只是提示，测量必须复验新原图。
        m_boundedImageSearch.hasCandidateHint = false;
        m_boundedImageSearch.candidateReady = false;
        m_boundedImageSearch.candidateHint = {-1.0, -1.0};
        m_boundedImageSearch.candidateConfirmations = 0;
        m_boundedImageSearch.candidateFrameId = -1;
    };
    const auto refine = [&](const cv::Point2d &point) {
        if (!insideImage(gray8, point) || diagnostic.imageRefinementCallCount >= 2) {
            return missing;
        }
        ++diagnostic.imageRefinementCallCount;
        const auto candidate = COnlineSeamRefiner::refine(gray8, point, options.seamRefinement);
        diagnostic.imageSearchDiagnostic += "center=" + std::to_string(point.x) + ","
            + std::to_string(point.y) + ";status=" + std::to_string(static_cast<int>(candidate.status))
            + ";support=" + std::to_string(candidate.supportRatio)
            + ";rms=" + std::to_string(candidate.fitRmsPixel) + "|";
        return candidate;
    };
    const auto rememberCandidate = [&](const OnlineSeamRefinementResult &candidate) {
        const bool consistent = m_boundedImageSearch.hasCandidateHint
            && cv::norm(candidate.point - m_boundedImageSearch.candidateHint)
                <= std::max(3.0, options.seamRefinement.maxPositionUncertaintyPixel);
        if (diagnostic.exposureFrameId >= 0
            && diagnostic.exposureFrameId > m_boundedImageSearch.candidateFrameId) {
            m_boundedImageSearch.candidateConfirmations = consistent
                ? m_boundedImageSearch.candidateConfirmations + 1 : 1;
            m_boundedImageSearch.candidateFrameId = diagnostic.exposureFrameId;
        }
        m_boundedImageSearch.candidateHint = candidate.point;
        m_boundedImageSearch.hasCandidateHint = true;
    };
    // 原目标每隔一帧仍有局部几何机会；其余帧复验候选并推进竞争检查。
    // 旧 KCF 已失效时直接逐帧复验候选，避免三新曝光确认被无关旧 hint 打断。
    const bool candidateTurn = m_boundedImageSearch.hasCandidateHint
        && (!coarseAccepted || m_boundedImageSearch.candidateTurn);
    m_boundedImageSearch.candidateTurn = !m_boundedImageSearch.candidateTurn;
    OnlineSeamRefinementResult current = missing;
    if (candidateTurn) {
        current = refine(m_boundedImageSearch.candidateHint);
        if (!current.valid) {
            rejectCandidate();
        } else {
            rememberCandidate(current);
        }
    } else if (!hints.empty()) {
        current = refine(hints.front());
        const bool hasSeparateHistoricalHint = hints.size() > 1
            && cv::norm(hints[0] - hints[1]) >= 12.0;
        if (hasSeparateHistoricalHint
            && (current.valid || m_boundedImageSearch.localPairTurn)) {
            const auto historical = refine(hints[1]);
            if (current.valid && historical.valid
                && cv::norm(current.point - historical.point)
                    > std::max(3.0, options.seamRefinement.maxPositionUncertaintyPixel)) {
                current.valid = false;
                current.status = OnlineSeamRefinementStatus::AmbiguousStripe;
                m_boundedImageSearchAmbiguous = true;
                m_boundedImageSearch = {};
                return current;
            }
            if (historical.valid) { current = historical; }
        }
        // 两个局部 hint 都退化时轮转保留一个调用给扩大搜索，不能占满后永久饿死。
        m_boundedImageSearch.localPairTurn = !m_boundedImageSearch.localPairTurn;
        if (current.valid) {
            m_boundedImageSearch = {};
            return current;
        }
    } else if (diagnostic.simulatedPointAvailable) {
        // 尚无图像锚时，每个新曝光都给同帧参考附近一次原图检验机会。
        // 某一帧条纹暂态失败不能把该位置排到整轮角点游标之后；另一次
        // 调用仍用于扩大搜索，参考坐标本身不会成为测量输出。
        current = refine(diagnostic.simulatedPoint);
        if (current.valid) { rememberCandidate(current); }
    }
    diagnostic.imageSearchExpanded = true;
    if (m_boundedImageSearch.candidateReady) {
        if (!candidateTurn) {
            if (diagnostic.imageRefinementCallCount >= 2) { return current; }
            current = refine(m_boundedImageSearch.candidateHint);
        }
        if (current.valid) {
            m_boundedImageSearch.candidateHint = current.point;
            return current;
        } else {
            rejectCandidate();
        }
    }
    diagnostic.imageSearchCyclePending = true;
    const auto appendProposal = [&](const cv::Point2d &point) {
        if (insideImage(gray8, point)
            && !std::any_of(m_boundedImageSearch.proposals.begin(),
                m_boundedImageSearch.proposals.end(), [&point](const auto &other) {
                    return cv::norm(other - point) < 12.0;
                })) {
            m_boundedImageSearch.proposals.push_back(point);
        }
    };
    if (m_boundedImageSearch.generationStage < 2) {
        // 两路全图角点生成分开到新帧，避免两次生成与整组拟合叠在一帧。
        const auto started = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> corners;
        if (m_boundedImageSearch.generationStage == 0) {
            for (const auto &point : hints) { appendProposal(point); }
            if (diagnostic.simulatedPointAvailable) {
                appendProposal(diagnostic.simulatedPoint);
            }
            cv::goodFeaturesToTrack(gray8, corners, 32, 0.02, 24.0, cv::noArray(), 9);
        } else {
            cv::Mat proposalImage;
            cv::GaussianBlur(gray8, proposalImage, {3, 3}, 0.8);
            cv::threshold(proposalImage, proposalImage, 0.0, 255.0,
                cv::THRESH_BINARY | cv::THRESH_OTSU);
            cv::goodFeaturesToTrack(proposalImage, corners, 32, 0.02, 24.0, cv::noArray(), 9);
        }
        // 优先检查同帧搜索提案附近的角点；顺序不授予候选测量资格。
        const cv::Point2d preferred = !hints.empty() ? hints.front() : diagnostic.simulatedPoint;
        if (!hints.empty() || diagnostic.simulatedPointAvailable) {
            std::stable_sort(corners.begin(), corners.end(), [&preferred](const auto &left, const auto &right) {
                return cv::norm(cv::Point2d(left) - preferred) < cv::norm(cv::Point2d(right) - preferred);
            });
        }
        for (const auto &point : corners) { appendProposal(point); }
        if (m_boundedImageSearch.generationStage == 1) {
            appendProposal({0.5 * gray8.cols, 0.5 * gray8.rows});
        }
        ++m_boundedImageSearch.generationStage;
        diagnostic.imageSearchProposalGenerationMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        return current;
    }
    if (m_boundedImageSearch.hasCandidateHint && !candidateTurn) {
        // 这是旧目标局部搜索帧；剩余预算只复验候选，不用旧候选结果比较当前图像。
        if (diagnostic.imageRefinementCallCount >= 2) { return current; }
        const auto candidate = refine(m_boundedImageSearch.candidateHint);
        if (candidate.valid) {
            rememberCandidate(candidate);
            if (m_boundedImageSearch.candidateConfirmations >= 2) {
                m_boundedImageSearch.candidateReady = true;
                diagnostic.imageSearchCyclePending = false;
                return candidate;
            }
            return current;
        } else {
            rejectCandidate();
        }
    }
    while (m_boundedImageSearch.cursor < m_boundedImageSearch.proposals.size()
        && diagnostic.imageRefinementCallCount < 2) {
        const auto proposal = m_boundedImageSearch.proposals[m_boundedImageSearch.cursor++];
        const auto candidate = refine(proposal);
        if (!candidate.valid) {
            if (!current.valid && candidate.status == OnlineSeamRefinementStatus::AmbiguousStripe) {
                current = candidate;
            }
            continue;
        }
        if (m_boundedImageSearch.hasCandidateHint && current.valid
            && cv::norm(candidate.point - current.point)
                > std::max(3.0, options.seamRefinement.maxPositionUncertaintyPixel)) {
            // 同帧两候选才可比较；不能拿不同曝光保存的几何当成竞争或测量证据。
            current.valid = false;
            current.status = OnlineSeamRefinementStatus::AmbiguousStripe;
            m_boundedImageSearchAmbiguous = true;
            m_boundedImageSearch = {};
            diagnostic.imageSearchCyclePending = false;
            return current;
        }
        current = candidate;
        rememberCandidate(candidate);
        // 首次命中后，下一帧先复验 primary，再用剩余调用检查下一个提案。
        break;
    }
    if (m_boundedImageSearch.cursor >= m_boundedImageSearch.proposals.size()) {
        diagnostic.imageSearchCycleCompleted = true;
        diagnostic.imageSearchCyclePending = false;
        if (m_boundedImageSearch.hasCandidateHint && current.valid) {
            m_boundedImageSearch.candidateReady = true;
            return current;
        }
        m_boundedImageSearch = {};
    }
    // 连续新曝光复验稳定的候选可交给证据层做身份确认；不必耗尽整幅图的
    // 所有无关角点。当前帧仍最多两次拟合，已发现的同帧竞争分支仍会拒绝。
    if (current.valid && m_boundedImageSearch.candidateConfirmations >= 2) {
        m_boundedImageSearch.candidateReady = true;
        diagnostic.imageSearchCyclePending = false;
        return current;
    }
    // 单曝光候选只作搜索提示，不能跳过逐曝光确认。
    if (current.valid) {
        current.valid = false;
        current.status = OnlineSeamRefinementStatus::UnstablePosition;
    }
    return current;
}

LegacyOnlineKcfResult CLegacyOnlineKcfDetector::processImageMeasurement(
    const cv::Mat &gray8,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const WeldProjectionContext &projectionContext,
    const LegacyOnlineKcfOptions &options)
{
    LegacyOnlineKcfResult result;
    result.diagnostic.imageMeasurementOutputEnabled = true;
    result.diagnostic.exposureFrameId = observation.frameId;
    result.diagnostic.boundedImageSearchEnabled = options.enableBoundedImageSearch;
    result.diagnostic.seamRefinementEnabled = true;
    result.diagnostic.imageSize = gray8.size();
    if (gray8.empty() || gray8.type() != CV_8UC1 || !projectionContext.valid) {
        notifyImageObservationGap();
        result.message = "原图或投影上下文无效，不生成测量点。";
        return result;
    }
    if ((m_templateImageSize.area() > 0 && m_templateImageSize != gray8.size())
        || (observation.valid && observation.seamIndex >= 0
            && observation.seamIndex != m_continuitySeamIndex)) {
        reset();
        m_imageMeasurementActive = true;
        m_continuitySeamIndex = observation.seamIndex;
    }
    m_templateImageSize = gray8.size();
    const bool freshImageFrame = observation.frameId >= 0
        && (m_imageLastObservationFrameId < 0
            || observation.frameId > m_imageLastObservationFrameId);
    if (!freshImageFrame) {
        notifyImageObservationGap();
    }
    m_imageLastObservationFrameId = std::max(m_imageLastObservationFrameId,
        observation.frameId);
    result.processedImage = gray8.clone();
    result.diagnostic.attempted = true;
    result.diagnostic.simulatedPoint = {observation.weldPointImage_px.x_px,
        observation.weldPointImage_px.y_px};
    result.diagnostic.simulatedPointAvailable = observation.valid
        && observation.hasWeldPointImage_px
        && std::isfinite(result.diagnostic.simulatedPoint.x)
        && std::isfinite(result.diagnostic.simulatedPoint.y);
    cv::Point2d coarse(-1.0, -1.0);
    bool coarseUsable = false;
    if (m_initialized && m_tracker) {
        try {
            result.trackedRoi = m_tracker->update(gray8);
            ++m_framesSinceInitialize;
            result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
            coarse = weldPointFromRoi(result.diagnostic.tracker.beforeFusionRoi,
                m_templateWeldAnchor);
            const auto &tracking = result.diagnostic.tracker;
            coarseUsable = tracking.completed && tracking.hasResponse
                && !tracking.correlationJumpRejected && insideImage(gray8, coarse)
                && std::isfinite(tracking.responsePeak) && tracking.responsePeak >= 0.45;
        } catch (const std::exception &) {
            m_tracker->commitOnlineObservationTraining(false);
        }
    }
    result.diagnostic.beforeFusionPoint = coarse;
    result.diagnostic.responsePoint = coarse;
    std::vector<cv::Point2d> hints;
    if (coarseUsable) { hints.push_back(coarse); }
    if (m_hasImageSearchPoint) { hints.push_back(m_lastImageSearchPoint); }
    ImageSearchResult search;
    double coarseIdentityQuality = options.enableBoundedImageSearch && coarseUsable
        ? imageIdentityQuality(gray8, coarse, m_imageIdentityPatch) : 0.0;
    const auto started = std::chrono::steady_clock::now();
    if (options.enableBoundedImageSearch) {
        search.geometry = searchBoundedImageGeometry(gray8, hints, options,
            result.diagnostic, coarseIdentityQuality > 0.0);
        search.ambiguous = m_boundedImageSearchAmbiguous;
        result.diagnostic.imageSearchDiagnostic += "cursor=" + std::to_string(m_boundedImageSearch.cursor)
            + ";proposals=" + std::to_string(m_boundedImageSearch.proposals.size())
            + ";confirmations=" + std::to_string(m_boundedImageSearch.candidateConfirmations);
    } else {
        m_boundedImageSearch = {};
        search = searchImageGeometry(gray8, hints, false, options.seamRefinement);
        int refinementCallCount = search.refinementCallCount;
        if (!search.geometry.valid && !search.ambiguous) {
            result.diagnostic.imageSearchExpanded = true;
            // 仿真仅是可退出的搜索提案；无效、越界、错误方向均不阻断原图搜索。
            if (result.diagnostic.simulatedPointAvailable) {
                hints.push_back(result.diagnostic.simulatedPoint);
            }
            search = searchImageGeometry(gray8, hints, true, options.seamRefinement);
            refinementCallCount += search.refinementCallCount;
            result.diagnostic.imageSearchCycleCompleted = true;
        }
        result.diagnostic.imageRefinementCallCount = refinementCallCount;
        result.diagnostic.imageSearchProposalGenerationMs = search.proposalGenerationMs;
    }
    result.diagnostic.refinementAttempted = true;
    result.diagnostic.refinementElapsedMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (!options.enableBoundedImageSearch && coarseUsable) {
        coarseIdentityQuality = imageIdentityQuality(gray8, coarse, m_imageIdentityPatch);
    }
    result.diagnostic.rawRefinement = search.geometry;
    result.diagnostic.refinement = search.geometry;
    // 局部尺度仅决定是否需要重新确认，不限制总行程。
    // 远候选必须匹配既有独立原图身份，并由连续新图像确认，不能重建模板后自证。
    const double associationRadius = static_cast<double>(std::min(
        options.seamRefinement.searchHalfWidthPixel,
        options.seamRefinement.searchHalfHeightPixel));
    if (search.geometry.valid && m_hasImageSearchPoint
        && cv::norm(search.geometry.point - m_lastImageSearchPoint) > associationRadius) {
        const double identity = imageIdentityQuality(gray8,
            search.geometry.point, m_imageIdentityPatch);
        // 旧目标仍有当前图像身份支撑时，远处相似折点不能通过累计帧数取代它。
        // 只有旧分支也不可验收时，才允许远候选开始独立重捕获确认。
        if (identity > 0.0 && freshImageFrame && coarseIdentityQuality <= 0.0) {
            m_imageReacquisitionFrames = m_imageReacquisitionFrames > 0
                    && cv::norm(search.geometry.point - m_imageReacquisitionPoint)
                        <= associationRadius
                ? m_imageReacquisitionFrames + 1 : 1;
            m_imageReacquisitionPoint = search.geometry.point;
        } else {
            m_imageReacquisitionFrames = 0;
        }
        if (m_imageReacquisitionFrames < 3) {
            result.diagnostic.refinement.valid = false;
            result.diagnostic.refinement.status = OnlineSeamRefinementStatus::UnstablePosition;
        }
    } else {
        m_imageReacquisitionFrames = 0;
    }
    bool geometryAccepted = result.diagnostic.refinement.valid;
    if (options.enableEvidencePolicy) {
        const auto projected = geometryAccepted
            ? CWeldImageToSpaceProjector::projectCvPoint(search.geometry.point,
                projectionContext, WeldProjectionTargetFrame::PositionerTcp)
            : WeldProjectionResult{};
        auto &evidence = result.diagnostic.evidence;
        evidence = m_evidence.evaluate(gray8, observation.frameId, options.referenceRevision,
            options.evidenceContext, result.diagnostic.refinement,
            projected.valid && projected.spacePoint.valid, coarse, coarseUsable,
            options.evidenceOptions);
        geometryAccepted = evidence.independentGeometry;
        if (!geometryAccepted) {
            result.diagnostic.refinement.valid = false;
            result.diagnostic.continuity.valid = evidence.pixelAvailable;
            result.diagnostic.continuity.point = evidence.point;
            result.diagnostic.continuity.source = evidence.source;
            result.weldPointPixel = evidence.point;
            result.diagnostic.finalPoint = evidence.point;
            result.message = evidence.message;
            // 拒绝训练后还原搜索位置，避免临时 KCF 响应逐帧污染可靠基点。
            if (m_tracker) {
                m_tracker->commitOnlineObservationTraining(false);
                if (m_hasImageSearchPoint) {
                    m_tracker->m_roi.x = static_cast<float>(m_lastImageSearchPoint.x
                        - m_tracker->m_roi.width * m_templateWeldAnchor.x);
                    m_tracker->m_roi.y = static_cast<float>(m_lastImageSearchPoint.y
                        - m_tracker->m_roi.height * m_templateWeldAnchor.y);
                }
                result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
            }
            if (options.evidenceOptions.enableContinuousOutput && evidence.controlEligible) {
                const auto continuation = CWeldImageToSpaceProjector::projectCvPoint(
                    evidence.point, projectionContext, WeldProjectionTargetFrame::PositionerTcp);
                result.stationPoint = continuation.spacePoint;
                result.valid = continuation.valid && continuation.spacePoint.valid;
                if (!result.valid) { evidence.controlEligible = false; }
            }
            // 连续业务点可进入跟随，但原图几何和模板训练资格仍保持独立。
            return result;
        }
    }
    if (geometryAccepted) {
        m_imageReacquisitionFrames = 0;
        m_boundedImageSearch = {};
        result.weldPointPixel = search.geometry.point;
        result.diagnostic.continuity.source = OnlineSeamPointSource::Geometry;
    } else if (!search.ambiguous && coarseUsable) {
        result.diagnostic.coarseObservationQuality = coarseIdentityQuality;
        if (result.diagnostic.coarseObservationQuality > 0.0) {
            result.weldPointPixel = coarse;
            result.diagnostic.coarseObservationUsed = true;
            result.diagnostic.continuity.source = OnlineSeamPointSource::CorrectedKcf;
        }
    }
    result.diagnostic.imageObservationAccepted = geometryAccepted
        || result.diagnostic.coarseObservationUsed;
    if (!result.diagnostic.imageObservationAccepted) {
        if (m_tracker) {
            m_tracker->commitOnlineObservationTraining(false);
            if (m_hasImageSearchPoint) {
                m_tracker->m_roi.x = static_cast<float>(m_lastImageSearchPoint.x
                    - m_tracker->m_roi.width * m_templateWeldAnchor.x);
                m_tracker->m_roi.y = static_cast<float>(m_lastImageSearchPoint.y
                    - m_tracker->m_roi.height * m_templateWeldAnchor.y);
            }
        }
        result.message = search.ambiguous
            ? "原图存在多个合法接缝候选，身份未确认，不生成点。"
            : "同帧原图未通过接缝或 KCF 身份验收，不生成点。";
        return result;
    }
    result.diagnostic.finalPoint = result.weldPointPixel;
    result.diagnostic.coarseFinalPoint = result.weldPointPixel;
    result.diagnostic.continuity.valid = true;
    result.diagnostic.continuity.point = result.weldPointPixel;
    result.diagnostic.continuity.observationWeightX = 1.0;
    result.diagnostic.continuity.observationWeightY = 1.0;
    const auto projection = CWeldImageToSpaceProjector::projectCvPoint(
        result.weldPointPixel, projectionContext, WeldProjectionTargetFrame::PositionerTcp);
    if (!projection.valid || !projection.spacePoint.valid) {
        if (m_tracker) { m_tracker->commitOnlineObservationTraining(false); }
        result.message = "原图测量已验收，但空间投影失败：" + projection.message;
        return result;
    }
    result.stationPoint = projection.spacePoint;
    result.valid = true;
    // 初始化和重捕获模板都只能来自当前已接纳的真实接缝；CAD 写回不改变此状态。
    if (geometryAccepted) {
        const bool aligned = m_initialized && coarseUsable
            && cv::norm(coarse - result.weldPointPixel) <= 3.0;
        if (!aligned) {
            m_tracker = std::make_unique<CKcfTracker>(
                true, true, false, false, KcfFeatureMode::Standard, 2);
            KcfFusionOptions imageFusion;
            imageFusion.mode = KcfFusionMode::KcfOnly;
            m_tracker->setFusionOptions(imageFusion);
            m_tracker->setOnlineObservationMode(true);
            m_tracker->init(centeredRoi(result.weldPointPixel,
                kRoiWidthPixel, kRoiHeightPixel, gray8.size()), gray8);
            m_templateWeldAnchor = {(result.weldPointPixel.x - m_tracker->m_roi.x)
                    / m_tracker->m_roi.width,
                (result.weldPointPixel.y - m_tracker->m_roi.y) / m_tracker->m_roi.height};
            m_templateImageSize = gray8.size();
            m_initialized = true;
            m_framesSinceInitialize = 0;
            result.diagnostic.templateReinitialized = true;
        } else {
            m_tracker->commitOnlineObservationTraining(true);
        }
        m_imageIdentityPatch = imageIdentityPatch(gray8, result.weldPointPixel);
    } else {
        // 退化 KCF 可续测，不能训练自身或更新独立几何身份参照。
        m_tracker->commitOnlineObservationTraining(false);
    }
    m_lastImageSearchPoint = result.weldPointPixel;
    m_hasImageSearchPoint = true;
    result.trackedRoi = clampRoi(cv::Rect(m_tracker->m_roi), gray8.size());
    result.diagnostic.templateAnchor = m_templateWeldAnchor;
    result.diagnostic.framesSinceInitialize = m_framesSinceInitialize;
    result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
    result.message = options.enableEvidencePolicy ? result.diagnostic.evidence.message : (geometryAccepted
        ? "原图接缝精定位测量；仿真仅辅助搜索。"
        : "当前原图 KCF 身份及质量验收通过，非独立精定位。");
    if (options.enableGapDetection) {
        DpStripeDetectionOptions centerlineOptions;
        centerlineOptions.roi = centeredRoi(result.weldPointPixel,
            kGapRoiSizePixel, kGapRoiSizePixel, gray8.size());
        centerlineOptions.seamXPixel = static_cast<int>(std::lround(result.weldPointPixel.x))
            - centerlineOptions.roi.x;
        const auto centerline = CDpStripeCenterlineDetector::detectCenterline(
            gray8, centerlineOptions);
        WeldPointCloud cloud;
        cloud.frameId = "positioner_tcp";
        if (centerline.valid) {
            std::vector<WeldImagePoint> points;
            for (const auto &point : centerline.points) {
                points.push_back({point.xPixel, point.yPixel, point.valid});
            }
            cloud.points = CWeldImageToSpaceProjector::projectImagePoints(points,
                projectionContext, WeldProjectionTargetFrame::PositionerTcp);
            cloud.valid = std::any_of(cloud.points.begin(), cloud.points.end(),
                [](const auto &point) { return point.valid; });
        }
        const auto gap = CPointCloudGapWidthDetector::detect(
            cloud, result.stationPoint, options.gapWidthOptions);
        result.gapValid = gap.valid;
        result.gapWidthMm = gap.valid ? gap.gapWidthMm : 0.0;
        result.gapConfidence = gap.valid ? gap.confidence : 0.0;
    }
    return result;
}

cv::Point2d CLegacyOnlineKcfDetector::fusePaperPosition(
    const cv::Point2d &observedPoint, bool reliableObservation,
    const LegacyOnlineKcfOptions &options, CLegacyOnlineKcfDiagnostic &diagnostic)
{
    diagnostic.positionObservationPoint = observedPoint;
    const bool consecutiveReference = m_paperPosition.hasBias
        && m_paperPosition.frameId < diagnostic.exposureFrameId
        && m_paperPosition.referenceRevision == options.referenceRevision
        && m_paperPosition.seamIndex == m_continuitySeamIndex;
    if (!diagnostic.simulatedPointAvailable) {
        m_paperPosition = {};
        return observedPoint;
    }
    if (!consecutiveReference) { m_paperPosition = {}; }
    cv::Point2d result = observedPoint;
    if (reliableObservation && m_paperPosition.hasBias) {
        const cv::Point2d prior = diagnostic.simulatedPoint + m_paperPosition.localBias;
        // 既有局部像素不确定度只撤销先验；再大图像创新也不拒绝可靠观测。
        const double uncertainty = options.seamRefinement.maxPositionUncertaintyPixel;
        if (std::isfinite(uncertainty) && uncertainty > 0.0
            && cv::norm(observedPoint - prior) <= uncertainty) {
            auto &filter = *m_paperPosition.filter;
            double priorState[2] = {prior.x, prior.y};
            double measurement[2] = {observedPoint.x, observedPoint.y};
            double control[2] = {0.0, 0.0};
            filter.m_X = RMatrix::Matrix(2, 1, priorState);
            filter.m_Y = RMatrix::Matrix(2, 1, measurement);
            filter.m_u = RMatrix::Matrix(2, 1, control);
            const double quality = std::clamp(diagnostic.coarseObservationQuality, 0.0, 1.0);
            filter.m_cR = filter.m_cRMaximum
                - quality * (filter.m_cRMaximum - filter.m_cRMinimum);
            filter.Kalfilter();
            const cv::Point2d fused(filter.m_X.val[0][0], filter.m_X.val[1][0]);
            if (std::isfinite(fused.x) && std::isfinite(fused.y)) {
                result = fused;
                diagnostic.simulationPositionTrusted = true;
                diagnostic.positionFusionApplied = true;
                diagnostic.positionPriorPoint = prior;
                diagnostic.simulationLocalBias = m_paperPosition.localBias;
                diagnostic.positionGainX = filter.m_K.val[0][0];
                diagnostic.positionGainY = filter.m_K.val[1][1];
            }
        } else {
            m_paperPosition = {};
        }
    }
    if (reliableObservation) {
        if (!m_paperPosition.filter) {
            m_paperPosition.filter = std::make_unique<CFilterKalman>(2);
            m_paperPosition.filter->m_A.eye();
            m_paperPosition.filter->m_B.eye();
            m_paperPosition.filter->m_H.eye();
        }
        // 用原始已验收观测刷新相邻曝光偏差，不反馈融合点、不固化旧偏差。
        m_paperPosition.localBias = observedPoint - diagnostic.simulatedPoint;
        m_paperPosition.frameId = diagnostic.exposureFrameId;
        m_paperPosition.referenceRevision = options.referenceRevision;
        m_paperPosition.seamIndex = m_continuitySeamIndex;
        m_paperPosition.hasBias = true;
    } else {
        m_paperPosition = {};
    }
    return result;
}

void CLegacyOnlineKcfDetector::initializePaperTracker(
    const cv::Mat &gray8, const cv::Point2d &point)
{
    const auto roi = centeredRoi(point, kRoiWidthPixel, kRoiHeightPixel, gray8.size());
    m_tracker = std::make_unique<CKcfTracker>(
        true, true, false, false, KcfFeatureMode::Mkcf, 3);
    KcfFusionOptions imageFusion;
    imageFusion.mode = KcfFusionMode::KcfOnly;
    m_tracker->setFusionOptions(imageFusion);
    m_tracker->setOnlineObservationMode(true);
    m_tracker->init(roi, gray8);
    m_templateWeldAnchor = {(point.x - roi.x) / roi.width, (point.y - roi.y) / roi.height};
    m_imageIdentityPatch = imageIdentityPatch(gray8, point);
    m_paperImageVelocity = {};
    m_hasPaperImageMotion = false;
    m_paperPosition = {};
    m_initialized = true;
}

LegacyOnlineKcfResult CLegacyOnlineKcfDetector::processPaperTracking(
    const cv::Mat &gray8,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const WeldProjectionContext &projectionContext,
    const LegacyOnlineKcfOptions &options,
    COnlineKcfPerformanceDiagnostic &performance)
{
    CDetectorStageClock stageClock(options.enablePerformanceDiagnostics, performance);
    LegacyOnlineKcfResult result;
    auto &diagnostic = result.diagnostic;
    diagnostic.imageMeasurementOutputEnabled = true;
    diagnostic.exposureFrameId = observation.frameId;
    diagnostic.imageSize = gray8.size();
    // 曝光身份先于参考元数据：旧缝号或旧图像尺寸不能清空单调性保护。
    if (observation.frameId < 0 || observation.frameId <= m_imageLastObservationFrameId) {
        result.message = "重复或倒序曝光不刷新跟踪观测。";
        return result;
    }
    if (gray8.empty() || gray8.type() != CV_8UC1 || !projectionContext.valid) {
        notifyImageObservationGap();
        result.message = "当前原图或曝光投影上下文无效。";
        // 消费本次新曝光身份，避免同一缺失输入重复产生恢复失败证据。
        m_imageLastObservationFrameId = observation.frameId;
        result.localizationEvidence.inputUnavailable = true;
        return result;
    }
    if ((m_templateImageSize.area() > 0 && m_templateImageSize != gray8.size())
        || (observation.valid && observation.seamIndex >= 0
            && observation.seamIndex != m_continuitySeamIndex)) {
        reset();
    }
    m_imageMeasurementActive = true;
    m_templateImageSize = gray8.size();
    // 暂缺仿真参考不是切换焊缝；保留最近已知身份以便下一有效参考恢复。
    if (observation.valid && observation.seamIndex >= 0) {
        m_continuitySeamIndex = observation.seamIndex;
    }
    m_imageLastObservationFrameId = observation.frameId;
    result.processedImage = gray8.clone();
    diagnostic.attempted = true;
    result.localizationEvidence.attempted = true;
    diagnostic.simulatedPoint = {observation.weldPointImage_px.x_px,
        observation.weldPointImage_px.y_px};
    diagnostic.simulatedPointAvailable = observation.valid && observation.hasWeldPointImage_px
        && std::isfinite(diagnostic.simulatedPoint.x) && std::isfinite(diagnostic.simulatedPoint.y);
    diagnostic.simulationPositionInImage = diagnostic.simulatedPointAvailable
        && insideImage(gray8, diagnostic.simulatedPoint);
    // 已接纳图像点只提供本次外观采样锚点；初次无有效绝对位置时从图像中心搜索。
    // 渲染点不作为实测输出，且不会写回工件仿真位姿。
    const cv::Point2d renderAnchor = m_hasImageSearchPoint ? m_lastImageSearchPoint
        : (diagnostic.simulationPositionInImage ? diagnostic.simulatedPoint
            : cv::Point2d(gray8.cols * 0.5, gray8.rows * 0.5));
    stageClock.enter(performance.simulationMs);
    auto appearance = makePaperSimulationAppearance(observation, gray8.size(), renderAnchor);
    if (!m_initialized && !appearance.image.empty()) {
        const auto imageSearchAnchor = findPaperSimulationSearchAnchor(gray8, appearance);
        if (!insideImage(gray8, imageSearchAnchor)) {
            appearance = {};
        } else if (cv::norm(imageSearchAnchor - appearance.anchor) > 1e-6) {
            appearance = makePaperSimulationAppearance(observation, gray8.size(), imageSearchAnchor);
        }
    }
    OnlineSeamRefinementResult bootstrapGeometry;
    if (!m_initialized) {
        stageClock.enter(performance.initializationMs);
        cv::Point2d confirmedBootstrap(-1.0, -1.0);
        const auto imageBootstrap = findPaperImageBootstrapPoint(gray8,
            appearance.image.empty() ? cv::Point2d(-1.0, -1.0) : appearance.anchor,
            !options.enableBoundedImageSearch || m_boundedImageSearch.generationStage == 0,
            options.seamRefinement, diagnostic, &confirmedBootstrap);
        std::vector<cv::Point2d> hints{imageBootstrap};
        if (diagnostic.simulationPositionInImage) { hints.push_back(diagnostic.simulatedPoint); }
        if (!insideImage(gray8, confirmedBootstrap) && options.enableBoundedImageSearch) {
            bootstrapGeometry = searchBoundedImageGeometry(
                gray8, hints, options, diagnostic, false);
        } else if (!insideImage(gray8, confirmedBootstrap)) {
            auto searched = searchImageGeometry(gray8, hints, false, options.seamRefinement);
            diagnostic.imageRefinementCallCount += searched.refinementCallCount;
            if (!searched.geometry.valid) {
                searched = searchImageGeometry(gray8, hints, true, options.seamRefinement);
                diagnostic.imageRefinementCallCount += searched.refinementCallCount;
                diagnostic.imageSearchExpanded = true;
            }
            bootstrapGeometry = searched.geometry;
        }
        if (!bootstrapGeometry.valid && !insideImage(gray8, confirmedBootstrap)) {
            result.localizationEvidence.recoveryAttempted = true;
            result.localizationEvidence.recoveryFailed = !diagnostic.imageSearchCyclePending;
            result.message = "初始化提案尚未通过当前图像确认，继续搜索目标焊缝。";
            return result;
        }
        const auto bootstrapPoint = bootstrapGeometry.valid
            ? bootstrapGeometry.point : confirmedBootstrap;
        if (!insideImage(gray8, bootstrapPoint)
            || imageIdentityQuality(gray8, bootstrapPoint,
                imageIdentityPatch(gray8, bootstrapPoint)) <= 0.0) {
            result.localizationEvidence.recoveryAttempted = true;
            result.localizationEvidence.recoveryFailed = true;
            result.message = "尚无可确认目标的初始化外观，继续等待当前图像证据。";
            return result;
        }
        initializePaperTracker(gray8, bootstrapPoint);
        stageClock.enter(performance.simulationMs);
        if (!appearance.image.empty()) {
            appearance = makePaperSimulationAppearance(observation, gray8.size(), bootstrapPoint);
        }
        diagnostic.templateReinitialized = true;
    }
    const cv::Mat &simulated = appearance.image;
    const cv::Rect &simulatedRoi = appearance.roi;
    diagnostic.simulationShapeAvailable = !simulated.empty();
    diagnostic.simulationTemplatePoint = appearance.anchor;
    diagnostic.simulationTemplateRebased = diagnostic.simulationShapeAvailable
        && cv::norm(appearance.anchor - diagnostic.simulatedPoint) > 1e-6;
    stageClock.enter(performance.simulationMs);
    if (!simulated.empty()) { m_tracker->setSimulatedTemplate(simulated, simulatedRoi); }
    else { m_tracker->clearSimulatedTemplate(); }
    stageClock.enter(performance.trackingMs);
    result.trackedRoi = m_tracker->update(gray8);
    stageClock.enter(performance.validationProjectionMs);
    diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
    auto coarse = weldPointFromRoi(diagnostic.tracker.beforeFusionRoi, m_templateWeldAnchor);
    diagnostic.beforeFusionPoint = coarse;
    diagnostic.responsePoint = coarse;
    diagnostic.coarseObservationQuality = imageIdentityQuality(gray8, coarse, m_imageIdentityPatch);
    bool observed = diagnostic.tracker.completed && diagnostic.tracker.hasResponse
        && !diagnostic.tracker.correlationJumpRejected && insideImage(gray8, coarse)
        && std::isfinite(diagnostic.tracker.responsePeak)
        && diagnostic.tracker.responsePeak > 0.1 && diagnostic.coarseObservationQuality > 0.0;
    const auto refreshResponseStatistics = [this, &diagnostic]() {
        const auto &statistics = m_paperResponseStatistics[
            std::clamp(diagnostic.tracker.responseTemplateCount, 0, 3)];
        diagnostic.normalResponseSamples = statistics.samples;
        diagnostic.normalResponseMean = statistics.mean;
        diagnostic.normalResponseStd = statistics.samples > 1
            ? std::sqrt(statistics.secondMoment / (statistics.samples - 1)) : 0.0;
        // 少量初扫响应尚不能校准成熟模型分布；30复用已有工程的最低样本数，
        // 只计焊前已校核图像，不能收集任意前30在线帧。
        diagnostic.responseStatisticsReady = statistics.samples >= 30;
        const double psr = diagnostic.tracker.responsePsr;
        const double spread = std::max(1e-6, diagnostic.normalResponseStd);
        diagnostic.responseNormal = !diagnostic.responseStatisticsReady
            || (std::isfinite(psr) && psr >= statistics.mean - 3.0 * spread
                && psr <= statistics.mean + 3.0 * spread);
    };
    refreshResponseStatistics();
    if (bootstrapGeometry.valid) {
        diagnostic.rawRefinement = bootstrapGeometry;
    } else if (insideImage(gray8, coarse)
        && (!options.enableBoundedImageSearch || diagnostic.imageRefinementCallCount < 2)) {
        diagnostic.rawRefinement = COnlineSeamRefiner::refine(gray8, coarse, options.seamRefinement);
        ++diagnostic.imageRefinementCallCount;
    }
    diagnostic.refinementAttempted = diagnostic.imageRefinementCallCount > 0;
    if (diagnostic.rawRefinement.valid) {
        coarse = diagnostic.rawRefinement.point;
        observed = true;
    }
    bool partialObservation = false;
    bool contradictoryLocalCorrection = false;
    const bool coarseAppearanceComplete = completeImageAppearance(gray8, coarse, m_imageIdentityPatch);
    if (!diagnostic.rawRefinement.valid && m_hasImageSearchPoint
        && (!observed || !diagnostic.responseStatisticsReady
            || !diagnostic.responseNormal || !coarseAppearanceComplete)
        && diagnostic.tracker.completed && diagnostic.tracker.hasResponse
        && !diagnostic.tracker.correlationJumpRejected) {
        // 完整响应优先；反光破坏外观时才复用相邻同来源运动，不能沿用几何校正差分。
        const bool hasAppearanceMotion = m_hasPaperImageMotion && !m_paperMotionHasGeometry;
        const cv::Point2d imagePrediction = hasAppearanceMotion
            ? m_paperImagePoint + m_paperImageVelocity : coarse;
        const cv::Point2d shoulderSearchCenter = !coarseAppearanceComplete && hasAppearanceMotion
            ? imagePrediction : coarse;
        auto shoulder = matchImageShoulders(gray8, shoulderSearchCenter,
            imagePrediction, m_imageIdentityPatch, false);
        if (!shoulder.valid || !completeImageAppearance(gray8, shoulder.point, m_imageIdentityPatch)) {
            // 完整单条纹保留矩形亚像素峰；竞争外观则仅按原身份条纹支持匹配。
            shoulder = matchImageShoulders(gray8, shoulderSearchCenter,
                imagePrediction, m_imageIdentityPatch, true);
        }
        diagnostic.shoulderObservationAccepted = shoulder.valid;
        diagnostic.shoulderObservationQuality = shoulder.quality;
        diagnostic.centerObservationQuality = shoulder.centerQuality;
        if (!shoulder.valid && !coarseAppearanceComplete) { observed = false; }
        if (shoulder.valid) {
            coarse = shoulder.point;
            contradictoryLocalCorrection =
                cv::norm(shoulder.point - diagnostic.responsePoint)
                    > options.seamRefinement.maxPositionUncertaintyPixel
                && !imageIdentityStripeSupported(gray8, shoulder.point, m_imageIdentityPatch);
            partialObservation = shoulder.centerQuality < 0.7 || contradictoryLocalCorrection;
            diagnostic.shoulderEstimateUsed = partialObservation;
            observed = true;
            diagnostic.coarseObservationQuality = shoulder.quality;
            cv::Rect2f correctedRoi = m_tracker->m_roi;
            correctedRoi.x = static_cast<float>(coarse.x - correctedRoi.width * m_templateWeldAnchor.x);
            correctedRoi.y = static_cast<float>(coarse.y - correctedRoi.height * m_templateWeldAnchor.y);
            m_tracker->correctOnlineObservationRoi(correctedRoi);
        }
    }
    // 局部校正与当前响应冲突时，用原图另起提案；历史/参考只核对身份，不替代测量。
    // 首轮之后让出分帧搜索预算，避免每帧重做提案而饿死恢复游标。
    if ((!observed || contradictoryLocalCorrection) && diagnostic.imageRefinementCallCount < 2
        && m_boundedImageSearch.generationStage == 0) {
        cv::Point2d confirmedRecovery(-1.0, -1.0);
        const auto proposal = findPaperImageBootstrapPoint(gray8, coarse, true,
            options.seamRefinement, diagnostic, &confirmedRecovery);
        OnlineSeamRefinementResult recoveryGeometry;
        if (!insideImage(gray8, confirmedRecovery) && insideImage(gray8, proposal)
            && diagnostic.imageRefinementCallCount < 2) {
            recoveryGeometry = COnlineSeamRefiner::refine(
                gray8, proposal, options.seamRefinement);
            ++diagnostic.imageRefinementCallCount;
        }
        const auto recoveredPoint = recoveryGeometry.valid
            ? recoveryGeometry.point : confirmedRecovery;
        if (insideImage(gray8, recoveredPoint)) {
            const double uncertainty = options.seamRefinement.maxPositionUncertaintyPixel;
            const bool localRecovery = !m_hasImageSearchPoint
                || cv::norm(recoveredPoint - m_lastImageSearchPoint) <= uncertainty;
            const bool alignedReferenceRecovery = m_paperPosition.hasBias
                && m_paperPosition.referenceRevision == options.referenceRevision
                && diagnostic.simulatedPointAvailable
                && cv::norm(recoveredPoint - diagnostic.simulatedPoint - m_paperPosition.localBias)
                    <= uncertainty;
            const auto identity = matchImageShoulders(
                gray8, recoveredPoint, recoveredPoint, m_imageIdentityPatch, false);
            if ((localRecovery || alignedReferenceRecovery) && identity.valid
                && cv::norm(identity.point - recoveredPoint)
                    <= uncertainty) {
                coarse = recoveredPoint;
                diagnostic.rawRefinement = recoveryGeometry;
                diagnostic.realSearchFallbackUsed = true;
                stageClock.enter(performance.initializationMs);
                initializePaperTracker(gray8, coarse);
                stageClock.enter(performance.simulationMs);
                if (!appearance.image.empty()) {
                    appearance = makePaperSimulationAppearance(observation, gray8.size(), coarse);
                    m_tracker->setSimulatedTemplate(appearance.image, appearance.roi);
                    diagnostic.simulationTemplatePoint = appearance.anchor;
                }
                stageClock.enter(performance.trackingMs);
                result.trackedRoi = m_tracker->update(gray8);
                stageClock.enter(performance.validationProjectionMs);
                diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
                refreshResponseStatistics();
                diagnostic.templateReinitialized = true;
                observed = true;
                partialObservation = false;
                contradictoryLocalCorrection = false;
                m_boundedImageSearch = {};
            }
        }
    }
    if (!observed || contradictoryLocalCorrection) {
        std::vector<cv::Point2d> hints;
        if (insideImage(gray8, coarse)) { hints.push_back(coarse); }
        if (m_hasImageSearchPoint) { hints.push_back(m_lastImageSearchPoint); }
        const auto recovered = searchBoundedImageGeometry(gray8, hints, options, diagnostic, false);
        result.localizationEvidence.recoveryAttempted = true;
        const double uncertainty = options.seamRefinement.maxPositionUncertaintyPixel;
        const bool localRecovery = !m_hasImageSearchPoint
            || cv::norm(recovered.point - m_lastImageSearchPoint) <= uncertainty;
        const bool alignedReferenceRecovery = m_paperPosition.hasBias
            && m_paperPosition.referenceRevision == options.referenceRevision
            && diagnostic.simulatedPointAvailable
            && cv::norm(recovered.point - diagnostic.simulatedPoint - m_paperPosition.localBias)
                <= uncertainty;
        if (recovered.valid && (localRecovery || alignedReferenceRecovery)) {
            coarse = recovered.point;
            diagnostic.rawRefinement = recovered;
            diagnostic.realSearchFallbackUsed = true;
            stageClock.enter(performance.initializationMs);
            initializePaperTracker(gray8, coarse);
            stageClock.enter(performance.simulationMs);
            if (!simulated.empty()) { m_tracker->setSimulatedTemplate(simulated, simulatedRoi); }
            stageClock.enter(performance.trackingMs);
            result.trackedRoi = m_tracker->update(gray8);
            stageClock.enter(performance.validationProjectionMs);
            diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
            refreshResponseStatistics();
            diagnostic.templateReinitialized = true;
            observed = true;
            partialObservation = false;
            contradictoryLocalCorrection = false;
        }
    }
    if (!observed) {
        // 被拒绝的响应不能把下帧搜索窗口继续拖向干扰分支。
        result.localizationEvidence.recoveryAttempted = true;
        result.localizationEvidence.recoveryFailed = true;
        if (m_hasImageSearchPoint) {
            cv::Rect2f previousRoi = m_tracker->m_roi;
            previousRoi.x = static_cast<float>(m_lastImageSearchPoint.x
                - previousRoi.width * m_templateWeldAnchor.x);
            previousRoi.y = static_cast<float>(m_lastImageSearchPoint.y
                - previousRoi.height * m_templateWeldAnchor.y);
            m_tracker->correctOnlineObservationRoi(previousRoi);
        }
        stageClock.enter(performance.trainingMs);
        m_tracker->commitOnlineSimulationTraining();
        stageClock.enter(performance.inputBookkeepingMs);
        diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
        m_evidence.notifyGap();
        m_paperPosition = {};
        m_paperImageVelocity = {};
        m_hasPaperImageMotion = false;
        result.message = "当前图像相关观测尚未确认目标。";
        return result;
    }
    diagnostic.refinement = diagnostic.rawRefinement;
    const auto independent = diagnostic.refinement.valid
        ? CWeldImageToSpaceProjector::projectCvPoint(diagnostic.refinement.point,
            projectionContext, WeldProjectionTargetFrame::PositionerTcp)
        : WeldProjectionResult{};
    diagnostic.evidence = m_evidence.evaluate(gray8, observation.frameId, options.referenceRevision,
        options.evidenceContext, diagnostic.refinement,
        independent.valid && independent.spacePoint.valid, coarse, true, options.evidenceOptions);
    const bool geometryAccepted = diagnostic.evidence.independentGeometry;
    const cv::Point2d currentObservation = diagnostic.refinement.valid ? diagnostic.refinement.point : coarse;
    diagnostic.coarseFinalPoint = coarse;
    const auto observationProjection = CWeldImageToSpaceProjector::projectCvPoint(currentObservation,
        projectionContext, WeldProjectionTargetFrame::PositionerTcp);
    result.weldPointPixel = fusePaperPosition(currentObservation,
        observationProjection.valid && observationProjection.spacePoint.valid && (geometryAccepted || !partialObservation), options, diagnostic);
    diagnostic.continuity.source = geometryAccepted ? OnlineSeamPointSource::Geometry
        : (partialObservation ? OnlineSeamPointSource::GuidedEstimate
            : OnlineSeamPointSource::CorrectedKcf);
    diagnostic.continuity.point = result.weldPointPixel;
    diagnostic.continuity.valid = true;
    diagnostic.imageObservationAccepted = geometryAccepted || !partialObservation;
    result.localizationEvidence.currentImageSupported = diagnostic.imageObservationAccepted;
    result.localizationEvidence.imageConstrainedEstimate = partialObservation
        && !contradictoryLocalCorrection;
    result.localizationEvidence.recoveryAttempted |= diagnostic.shoulderObservationAccepted;
    result.localizationEvidence.recoveryFailed = contradictoryLocalCorrection
        && !diagnostic.imageSearchCyclePending;
    diagnostic.coarseObservationUsed = !geometryAccepted;
    diagnostic.finalPoint = result.weldPointPixel;
    diagnostic.evidence.pixelAvailable = true;
    diagnostic.evidence.controlEligible = true;
    diagnostic.evidence.point = result.weldPointPixel;
    diagnostic.evidence.source = diagnostic.continuity.source;
    const auto projection = CWeldImageToSpaceProjector::projectCvPoint(result.weldPointPixel,
        projectionContext, WeldProjectionTargetFrame::PositionerTcp);
    result.stationPoint = projection.spacePoint;
    result.valid = projection.valid && projection.spacePoint.valid;
    // 图像定位可以成功但射线/激光面无法形成空间点；保留图像证据，撤销空间控制资格。
    result.localizationEvidence.inputUnavailable = !result.valid;
    diagnostic.evidence.controlEligible = result.valid && !contradictoryLocalCorrection;
    diagnostic.completeAppearanceSupported = !partialObservation
        && (diagnostic.refinement.valid || !m_hasImageSearchPoint
            || completeImageAppearance(gray8, currentObservation, m_imageIdentityPatch));
    // 成熟统计沿用论文双侧3sigma；未成熟时明确依靠完整当前图像校核，
    // 不是把小样本PSR分布视为已校准，也不把异常在线帧加入正常分布。
    const bool updateRealTemplate = result.valid && !partialObservation
        && (diagnostic.responseStatisticsReady ? diagnostic.responseNormal
            : diagnostic.completeAppearanceSupported);
    // 业务位置、训练ROI和身份取样保持同一锚点；仿真替代训练也不丢失原图校正。
    cv::Rect2f correctedRoi = m_tracker->m_roi;
    correctedRoi.x = static_cast<float>(currentObservation.x
        - correctedRoi.width * m_templateWeldAnchor.x);
    correctedRoi.y = static_cast<float>(currentObservation.y
        - correctedRoi.height * m_templateWeldAnchor.y);
    m_tracker->correctOnlineObservationRoi(correctedRoi);
    stageClock.enter(performance.trainingMs);
    if (updateRealTemplate) {
        m_tracker->commitOnlineObservationTraining(true);
    } else {
        m_tracker->commitOnlineSimulationTraining();
    }
    stageClock.enter(performance.inputBookkeepingMs);
    // 焊前可信观测可校准实际单/三模板分布，缺仿真不能令统计永久停在零。
    auto &responseStatistics = m_paperResponseStatistics[
        std::clamp(diagnostic.tracker.responseTemplateCount, 0, 3)];
    const double psr = diagnostic.tracker.responsePsr;
    if (updateRealTemplate && !options.enableBoundedImageSearch
        && std::isfinite(psr) && psr > 0.0) {
        ++responseStatistics.samples;
        const double delta = psr - responseStatistics.mean;
        responseStatistics.mean += delta / responseStatistics.samples;
        responseStatistics.secondMoment += delta * (psr - responseStatistics.mean);
    }
    diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
    if (result.valid && !contradictoryLocalCorrection) {
        if (updateRealTemplate) {
            m_imageIdentityPatch = imageIdentityPatch(gray8, currentObservation);
        }
        if (m_hasPaperImageMotion
            && m_paperMotionHasGeometry == diagnostic.refinement.valid) {
            m_paperImageVelocity = currentObservation - m_paperImagePoint;
        } else {
            m_paperImageVelocity = {};
        }
        m_paperImagePoint = currentObservation;
        m_hasPaperImageMotion = true;
        m_paperMotionHasGeometry = diagnostic.refinement.valid;
        m_lastImageSearchPoint = result.weldPointPixel;
        m_hasImageSearchPoint = true;
    }
    result.message = !result.valid ? "当前图像定位可用，但空间投影失败：" + projection.message
        : (contradictoryLocalCorrection ? "局部校正未获目标条纹支持，仅显示估计，不提供控制点。"
            : (geometryAccepted ? "当前图像跟踪，附独立几何确认。"
                : "当前图像相关跟踪，非独立几何测量。"));
    return result;
}

LegacyOnlineKcfResult CLegacyOnlineKcfDetector::process(
    const cv::Mat &gray8,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const WeldProjectionContext &projectionContext,
    const LegacyOnlineKcfOptions &options)
{
    if (options.enablePaperTracking) {
        COnlineKcfPerformanceDiagnostic performance;
        auto result = processPaperTracking(
            gray8, observation, projectionContext, options, performance);
        result.diagnostic.performance = performance;
        return result;
    }
    if (options.enableImageMeasurementOutput) {
        if (!m_imageMeasurementActive) {
            reset();
            m_imageMeasurementActive = true;
        }
        return processImageMeasurement(gray8, observation, projectionContext, options);
    }
    if (m_imageMeasurementActive) {
        reset();
    }
    LegacyOnlineKcfResult result;
    result.diagnostic.onlineObservationFusionEnabled = options.enableOnlineObservationFusion;
    result.diagnostic.imageSize = gray8.size();
    result.diagnostic.seamRefinementEnabled = options.enableSeamRefinement;
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

    m_tracker->setFusionOptions(options.fusion);
    m_tracker->setOnlineObservationMode(options.enableOnlineObservationFusion);
    if (options.enableOnlineObservationFusion && !m_onlineObservationActive
        && m_hasImageSearchPoint && m_initialized) {
        // 初始扫描进入正式在线时以已看到的真实接缝接续搜索，而非上一融合 ROI。
        m_tracker->m_roi.x = static_cast<float>(m_lastImageSearchPoint.x
            - m_tracker->m_roi.width * m_templateWeldAnchor.x);
        m_tracker->m_roi.y = static_cast<float>(m_lastImageSearchPoint.y
            - m_tracker->m_roi.height * m_templateWeldAnchor.y);
    }
    m_onlineObservationActive = options.enableOnlineObservationFusion;
    if (!options.enableOnlineObservationFusion) { m_coarseRecoveryPending = false; }

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

    if (observation.seamIndex != m_continuitySeamIndex) {
        m_continuity.reset();
        m_guidedEstimator.reset();
        m_continuitySeamIndex = observation.seamIndex;
        m_hasImageSearchPoint = false;
        m_coarseRecoveryPending = false;
    }
    if (options.enableSimulationGuidance
        && (!m_guidanceRevisionKnown
            || m_guidanceReferenceRevision != options.referenceRevision)) {
        if (!options.enableOnlineObservationFusion) {
            // 旧入口保留模型版本边界；正式在线的真实图像模板不因 CAD 写回而失效。
            m_continuity.reset();
            m_guidedEstimator.reset();
            m_initialized = false;
            m_framesSinceInitialize = 0;
        }
        m_guidanceReferenceRevision = options.referenceRevision;
        m_guidanceRevisionKnown = true;
    }
    const cv::Rect2f previous = m_tracker->m_roi;
    const bool wasInitialized = m_initialized;
    if (!m_initialized || (!options.enableOnlineObservationFusion
            && m_framesSinceInitialize > kReinitializeIntervalFrame)) {
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
        m_coarseRecoveryPending = false;
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
    result.diagnostic.simulatedPointAvailable = true;
    result.diagnostic.gaborSucceeded = gaborSucceeded;
    result.diagnostic.enhancementRoi = currentRoi;
    result.diagnostic.templateAnchor = m_templateWeldAnchor;
    result.diagnostic.simulatedPoint = simulatedPoint;
    result.diagnostic.framesSinceInitialize = m_framesSinceInitialize;
    bool trackingException = false;
    try {
        result.trackedRoi = m_tracker->update(result.processedImage);
        ++m_framesSinceInitialize;
    } catch (const std::exception &exception) {
        result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
        trackingException = true;
        result.message = std::string("KCF exception: ") + exception.what();
        if (!options.enableSeamRefinement) return result;
    }

    result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
    result.diagnostic.responsePoint = weldPointFromRoi(
        result.diagnostic.tracker.responseRoi, m_templateWeldAnchor);
    result.diagnostic.beforeFusionPoint = weldPointFromRoi(
        result.diagnostic.tracker.beforeFusionRoi, m_templateWeldAnchor);
    result.diagnostic.coarseFinalPoint = weldPointFromRoi(
        options.enableOnlineObservationFusion ? result.diagnostic.tracker.finalRoi : m_tracker->m_roi,
        m_templateWeldAnchor);
    result.weldPointPixel = result.diagnostic.coarseFinalPoint;
    if (options.enableSeamRefinement) {
        const auto &tracking = result.diagnostic.tracker;
        double rawMinimum = 0.0;
        double rawMaximum = 0.0;
        cv::minMaxLoc(gray8(currentRoi), &rawMinimum, &rawMaximum);
        // 增强后的相关峰不能证明原图存在条纹；完全遮挡/饱和时禁用粗观测。
        const bool rawContrastUsable = rawMaximum - rawMinimum >= 40.0;
        const bool coarseUsable = !trackingException && rawContrastUsable && tracking.completed && tracking.hasResponse
            && !tracking.correlationJumpRejected
            && (!tracking.fusionJumpRejected || options.enableOnlineObservationFusion)
            && std::isfinite(tracking.responsePeak) && tracking.responsePeak >= 0.45;
        if (!trackingException && tracking.completed && tracking.hasResponse
            && !tracking.correlationJumpRejected
            && (!tracking.fusionJumpRejected || options.enableOnlineObservationFusion)) {
            result.diagnostic.refinementAttempted = true;
            const auto refinementStarted = std::chrono::steady_clock::now();
            result.diagnostic.refinement = COnlineSeamRefiner::refine(
                gray8, result.diagnostic.beforeFusionPoint, options.seamRefinement);
            result.diagnostic.refinementElapsedMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - refinementStarted).count();
        }
        if (options.enableOnlineObservationFusion && m_hasImageSearchPoint
            && cv::norm(result.diagnostic.beforeFusionPoint - m_lastImageSearchPoint) > 8.0
            && (!result.diagnostic.refinement.valid
                || cv::norm(result.diagnostic.refinement.point - m_lastImageSearchPoint) > 12.0
                || cv::norm(result.diagnostic.beforeFusionPoint - m_lastImageSearchPoint) > 12.0)) {
            // 反光可能将粗窗口带离真实接缝；备用搜索围绕真实历史，绝不围绕仿真点。
            const auto started = std::chrono::steady_clock::now();
            const auto historical = COnlineSeamRefiner::refine(
                gray8, m_lastImageSearchPoint, options.seamRefinement);
            result.diagnostic.refinementAttempted = true;
            result.diagnostic.refinementElapsedMs += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            // 有效的新原图证据由辅助器统一做跨帧及旧分支竞争校核，
            // 不能仅因历史更近而在进入验收前抹去它。备用只补足本帧缺测。
            if (historical.valid && !result.diagnostic.refinement.valid) {
                result.diagnostic.refinement = historical;
                result.diagnostic.realSearchFallbackUsed = true;
            }
        }
        result.diagnostic.rawRefinement = result.diagnostic.refinement;
        if (options.enableOnlineObservationFusion && coarseUsable) {
            result.diagnostic.coarseObservationQuality = coarseObservationQuality(gray8,
                result.diagnostic.beforeFusionPoint, tracking, result.diagnostic.rawRefinement);
        }
        if (options.enableSimulationGuidance) {
            std::vector<cv::Point2d> referencePolyline;
            referencePolyline.reserve(simulatedLaserPoints.size());
            for (const auto &point : simulatedLaserPoints) {
                referencePolyline.push_back({point.px, point.py});
            }
            auto guidedOptions = options.guidedOptions;
            guidedOptions.enableRawReacquisition = options.enableOnlineObservationFusion;
            guidedOptions.enableImageContinuity = options.enableOnlineObservationFusion;
            result.diagnostic.guided = m_guidedEstimator.update(gray8,
                referencePolyline, simulatedPoint, result.diagnostic.refinement,
                options.referenceRevision, guidedOptions,
                options.hasReferenceTransitionPoint
                    ? options.referenceTransitionPoint : cv::Point2d(-1.0, -1.0));
            if (result.diagnostic.guided.rawJumpRejected) {
                // 模型补偿后的突跳或原图分支不一致同样否决假阳性，禁止其种新独立锚。
                result.diagnostic.refinement.valid = false;
                result.diagnostic.refinement.status = OnlineSeamRefinementStatus::UnstablePosition;
            }
        }
        if (options.enableOnlineObservationFusion && coarseUsable
            && result.diagnostic.guided.hasRawShoulderSupport
            && result.diagnostic.guided.imageObservationQuality > 0.0
            && cv::norm(result.diagnostic.beforeFusionPoint - result.diagnostic.guided.point) <= 8.0) {
            // 全窗口多峰不等于真实 KCF 无用；与持续可见的母材肩部一致时按图像质量降权。
            const double responseQuality = std::clamp((tracking.responsePeak - 0.35) / 0.45, 0.0, 1.0);
            result.diagnostic.coarseObservationQuality = std::max(
                result.diagnostic.coarseObservationQuality,
                responseQuality * result.diagnostic.guided.imageObservationQuality);
        }
        if (options.enableOnlineObservationFusion && result.diagnostic.guided.reacquired
            && result.diagnostic.refinement.valid) {
            m_coarseRecoveryPending = true;
            m_continuity.clearCoarseObservationBias();
        }
        if (options.enableOnlineObservationFusion && m_coarseRecoveryPending) {
            // 恢复帧的粗 ROI 仍可能在旧支系；新图像点与 KCF 同位验收前，
            // 不重学刚清掉的大偏差，也不让旧模板响应参与退化业务融合。
            result.diagnostic.coarseObservationQuality = 0.0;
        }
        result.diagnostic.continuity = options.enableOnlineObservationFusion
            ? m_continuity.updateOnline(gray8.size(), simulatedPoint,
                result.diagnostic.beforeFusionPoint, result.diagnostic.coarseObservationQuality,
                tracking, result.diagnostic.refinement, options.fusion,
                options.hasReferenceTransitionPoint
                    ? options.referenceTransitionPoint : cv::Point2d(-1.0, -1.0))
            : m_continuity.update(gray8.size(), simulatedPoint,
                result.diagnostic.beforeFusionPoint, coarseUsable, tracking,
                result.diagnostic.refinement, options.fusion);
        result.diagnostic.coarseObservationUsed = options.enableOnlineObservationFusion
            && result.diagnostic.continuity.source == OnlineSeamPointSource::CorrectedKcf;
        if (result.diagnostic.coarseObservationUsed
            && cv::norm(result.diagnostic.continuity.correctedCoarsePoint
                - (result.diagnostic.guided.hasRawShoulderSupport
                    ? result.diagnostic.guided.point : result.diagnostic.guided.prediction)) > 8.0) {
            // 创新锚必须来自独立图像历史；没有肩部时也不能以逐帧业务结果放宽慢漂移。
            result.diagnostic.coarseObservationUsed = false;
        }
        if (options.enableSimulationGuidance) {
            if (options.enableOnlineObservationFusion && !result.diagnostic.refinement.valid) {
                const auto &guided = result.diagnostic.guided;
                if (!guided.valid) {
                    result.message = "图像连续估计坐标无效。";
                    return result;
                }
                const bool shoulders = guided.hasRawShoulderSupport
                    && guided.imageObservationQuality > 0.0;
                cv::Point2d businessPoint = guided.point;
                double weightX = 0.0;
                double weightY = 0.0;
                if (result.diagnostic.coarseObservationUsed) {
                    weightX = std::clamp(result.diagnostic.continuity.observationWeightX, 0.0, 0.35);
                    weightY = std::clamp(result.diagnostic.continuity.observationWeightY, 0.0, 0.35);
                    if (shoulders) {
                        // 两侧真实母材拟合是主基准，KCF 只补充一致的有界图像修正。
                        const auto delta = result.diagnostic.continuity.correctedCoarsePoint - guided.point;
                        businessPoint.x += weightX * delta.x;
                        businessPoint.y += weightY * delta.y;
                    } else {
                        businessPoint = result.diagnostic.continuity.point;
                    }
                }
                const auto delta = businessPoint - result.diagnostic.continuity.motionPrediction;
                const double distance = cv::norm(delta);
                if (distance > 2.0) {
                    const double scale = 2.0 / distance;
                    businessPoint = result.diagnostic.continuity.motionPrediction + delta * scale;
                    weightX *= scale;
                    weightY *= scale;
                    result.diagnostic.continuity.transitionLimited = true;
                }
                result.diagnostic.continuity.point = businessPoint;
                result.diagnostic.continuity.valid = true;
                result.diagnostic.continuity.degradedFrames = guided.degradedFrames;
                result.diagnostic.continuity.observationWeightX = weightX;
                result.diagnostic.continuity.observationWeightY = weightY;
                result.diagnostic.continuity.source = result.diagnostic.coarseObservationUsed
                    ? OnlineSeamPointSource::CorrectedKcf
                    : (shoulders ? OnlineSeamPointSource::GuidedEstimate
                        : OnlineSeamPointSource::SimulationPrediction);
                m_continuity.synchronizeEstimate(businessPoint, simulatedPoint, guided.degradedFrames);
            }
            // 有原图局部支撑的低权重 KCF 已经通过连续器的创新/限幅检查，
            // 不再因精定位退化被辅助预测无条件覆盖；其来源仍是 estimated。
            else if (!result.diagnostic.refinement.valid && !result.diagnostic.coarseObservationUsed) {
                const auto &guided = result.diagnostic.guided;
                // 参考或估计坐标无效才中断；无锚及预算耗尽由来源与质量标记表达。
                if (!guided.valid) {
                    result.diagnostic.continuity.valid = false;
                    result.diagnostic.continuity.source = OnlineSeamPointSource::Unavailable;
                    result.message = "仿真辅助续接坐标无效。";
                    return result;
                }
                cv::Point2d businessPoint = guided.point;
                if (tracking.fusionApplied && guided.hasTrustedAnchor) {
                    // 无锚种子已经是业务目标，不能再次按原图点融合。
                    // 辅助器保留原图锚坐标；业务输出仍遵守用户选择的同帧融合比例。
                    // 不把此融合点写回 raw refinement 或辅助器历史。
                    businessPoint.x = simulatedPoint.x
                        + std::clamp(tracking.gainX, 0.0, 1.0)
                            * (guided.point.x - simulatedPoint.x);
                    businessPoint.y = simulatedPoint.y
                        + std::clamp(tracking.gainY, 0.0, 1.0)
                            * (guided.point.y - simulatedPoint.y);
                }
                if (!options.hasReferenceTransitionPoint) {
                    // 与连续器共用上帧业务目标的运动预测，避免两次限幅叠加。
                    const auto delta = businessPoint - result.diagnostic.continuity.motionPrediction;
                    const double distance = cv::norm(delta);
                    if (distance > 2.0) {
                        businessPoint = result.diagnostic.continuity.motionPrediction + delta * (2.0 / distance);
                        result.diagnostic.continuity.transitionLimited = true;
                    }
                }
                result.diagnostic.continuity.point = businessPoint;
                result.diagnostic.continuity.valid = true;
                result.diagnostic.continuity.degradedFrames = guided.degradedFrames;
                result.diagnostic.continuity.observationWeightX = 0.0;
                result.diagnostic.continuity.observationWeightY = 0.0;
                result.diagnostic.continuity.source = guided.hasRawShoulderSupport
                    ? OnlineSeamPointSource::GuidedEstimate
                    : OnlineSeamPointSource::SimulationPrediction;
                m_continuity.synchronizeEstimate(businessPoint,
                    simulatedPoint, guided.degradedFrames);
            }
        } else {
            // 配置关闭后不携带辅助锚跨越未知图像区间。
            m_guidedEstimator.reset();
            m_guidanceRevisionKnown = false;
        }
        result.weldPointPixel = result.diagnostic.continuity.point;
        if (!result.diagnostic.continuity.valid) {
            result.message = "接缝连续估计不可用：缺少有效同帧参考。";
            return result;
        }
    }
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
    if (result.diagnostic.refinement.valid) {
        m_lastImageSearchPoint = result.diagnostic.refinement.point;
        m_hasImageSearchPoint = true;
    } else if (options.enableOnlineObservationFusion
        && result.diagnostic.guided.hasRawShoulderSupport
        && result.diagnostic.guided.imageObservationQuality > 0.0) {
        m_lastImageSearchPoint = result.diagnostic.guided.point;
        m_hasImageSearchPoint = true;
    }
    if (options.enableOnlineObservationFusion) {
        const auto &tracking = result.diagnostic.tracker;
        // 精定位回到真实接缝不代表原始相关 ROI 正确；仅提交同一图像位置的特征。
        const bool trainAccepted = result.diagnostic.refinement.valid
            && !result.diagnostic.realSearchFallbackUsed
            && cv::norm(result.diagnostic.beforeFusionPoint
                - result.diagnostic.refinement.point) <= 8.0
            && !result.diagnostic.guided.rawJumpRejected
            && !result.diagnostic.guided.reacquired
            && tracking.completed && tracking.hasResponse && !tracking.correlationJumpRejected
            && result.diagnostic.rawRefinement.ambiguousColumnRatio <= 0.05;
        m_tracker->commitOnlineObservationTraining(trainAccepted);
        if (trainAccepted) { m_coarseRecoveryPending = false; }
        if (result.diagnostic.guided.reacquired) {
            // 新原图支系已连续自证；旧相关 ROI 的特征不可回灌，
            // 下一帧在真实点重新搜索并验收训练，避免旧滤波历史把恢复点拉回。
            m_tracker->initKF();
        }
        result.diagnostic.tracker = m_tracker->lastUpdateDiagnostic();
        if (m_hasImageSearchPoint && (!trainAccepted || result.diagnostic.realSearchFallbackUsed)) {
            // 拒绝反光帧不训练；下一帧搜索接续真实激光/肩部，不能追着仿真或坏峰走。
            m_tracker->m_roi.x = static_cast<float>(m_lastImageSearchPoint.x
                - m_tracker->m_roi.width * m_templateWeldAnchor.x);
            m_tracker->m_roi.y = static_cast<float>(m_lastImageSearchPoint.y
                - m_tracker->m_roi.height * m_templateWeldAnchor.y);
        }
    }

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
    if (options.enableSeamRefinement && result.valid) {
        switch (result.diagnostic.continuity.source) {
        case OnlineSeamPointSource::Geometry:
            result.message += "; 原图接缝精定位（间隙中心）。";
            break;
        case OnlineSeamPointSource::CorrectedKcf:
            result.message += options.enableOnlineObservationFusion
                ? "; 原图受干扰：真实 KCF 与图像连续估计，非独立精定位。"
                : "; 原图受干扰：KCF/仿真辅助连续估计，非独立精定位。";
            break;
        case OnlineSeamPointSource::GuidedEstimate:
            result.message += "; 时序锚与原图母材肩部辅助估计，非独立实测焊点。";
            break;
        default:
            result.message += options.enableOnlineObservationFusion
                ? "; 图像观测不足：有界图像历史预测，非实测焊点。"
                : "; 图像观测不足：仿真辅助预测，非实测焊点。";
            break;
        }
    }
    return result;
}

} // namespace i2r_rlc::vision
