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
 * @brief KCF 响应以外的原图身份校核：相关形状和可见窄条纹必须同时保留。
 * @details 身份参照只能来自独立几何；缺少参照、空白、宽饱和及大范围竞争反光均不接纳。
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
    m_tracker.reset();
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

void CLegacyOnlineKcfDetector::notifyImageObservationGap()
{
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
    };
    const auto refine = [&](const cv::Point2d &point) {
        if (!insideImage(gray8, point) || diagnostic.imageRefinementCallCount >= 2) {
            return missing;
        }
        ++diagnostic.imageRefinementCallCount;
        return COnlineSeamRefiner::refine(gray8, point, options.seamRefinement);
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
            m_boundedImageSearch.candidateHint = current.point;
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
            m_boundedImageSearch.candidateHint = candidate.point;
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
        m_boundedImageSearch.candidateHint = candidate.point;
        m_boundedImageSearch.hasCandidateHint = true;
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
    // 轮次未完的候选仅作下一帧搜索中心，不能提前成为原图测量或独立 ICP。
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
            // 估计仅保留像素，不投影为实测空间点，不进入 Control/独立 ICP。
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

LegacyOnlineKcfResult CLegacyOnlineKcfDetector::process(
    const cv::Mat &gray8,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const WeldProjectionContext &projectionContext,
    const LegacyOnlineKcfOptions &options)
{
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
