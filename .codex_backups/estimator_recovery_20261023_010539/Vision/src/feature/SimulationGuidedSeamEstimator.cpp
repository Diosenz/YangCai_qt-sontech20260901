/**
 * @file SimulationGuidedSeamEstimator.cpp
 * @brief 先补偿仿真运动，再用原图支撑校核有界接缝续接，禁止估计自我累积。
 */
#include "feature/SimulationGuidedSeamEstimator.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace i2r_rlc::vision {
namespace {

bool inside(const cv::Point2d &point, const cv::Size &size)
{
    return std::isfinite(point.x) && std::isfinite(point.y)
        && point.x >= 0.0 && point.x < size.width
        && point.y >= 0.0 && point.y < size.height;
}

struct Peak {
    double y = 0.0;
    double contrast = 0.0;
};

struct Branch {
    bool valid = false;
    int ambiguousColumns = 0;
    int count = 0;
    double slope = 0.0;
    double intercept = 0.0;
    double rms = 0.0;
};

double smooth(const cv::Mat &image, int x, int y)
{
    return (image.at<unsigned char>(y - 1, x)
        + image.at<unsigned char>(y, x)
        + image.at<unsigned char>(y + 1, x)) / 3.0;
}

/** @brief 保留竞争窄峰；亮度优势不能将仿真形状直接提升为真实条纹身份。 */
std::vector<Peak> findPeaks(const cv::Mat &image, int x,
    double expectedY, double radius)
{
    const int begin = std::max(2, static_cast<int>(std::floor(expectedY - radius)));
    const int end = std::min(image.rows - 3,
        static_cast<int>(std::ceil(expectedY + radius)));
    if (begin >= end) { return {}; }
    double background = 255.0;
    for (int y = begin; y <= end; ++y) {
        background = std::min(background,
            static_cast<double>(image.at<unsigned char>(y, x)));
    }
    std::vector<Peak> peaks;
    for (int y = begin; y <= end; ++y) {
        const double value = smooth(image, x, y);
        if (value - background < 40.0
            || value < smooth(image, x, y - 1)
            || value <= smooth(image, x, y + 1)) { continue; }
        const double threshold = background + (value - background) * 0.5;
        int left = y;
        int right = y;
        while (left > begin && image.at<unsigned char>(left - 1, x) > threshold) { --left; }
        while (right < end && image.at<unsigned char>(right + 1, x) > threshold) { ++right; }
        if (right - left + 1 > 12) { continue; }
        double weight = 0.0;
        double weightedY = 0.0;
        for (int row = std::max(0, y - 3);
             row <= std::min(image.rows - 1, y + 3); ++row) {
            const double sample = std::max(0.0,
                image.at<unsigned char>(row, x) - background);
            weight += sample;
            weightedY += row * sample;
        }
        if (weight > 0.0) { peaks.push_back({weightedY / weight, value - background}); }
    }
    return peaks;
}

/**
 * @brief 围绕历史方向收集真实窄峰；竞争峰列弃用，不按 CAD 更相似者强行选峰。
 * 输出端部在当前预测 X 处，允许分别保留左右台阶并采用 GapCenter 语义。
 */
Branch fitBranch(const cv::Mat &image, const cv::Point2d &prediction,
    int side, int firstOffset, int lastOffset, double slope,
    double heightOffset, double searchRadius, int minimumColumns,
    bool followHistory)
{
    Branch result;
    std::vector<cv::Point2d> samples;
    for (int offset = firstOffset; offset <= lastOffset; ++offset) {
        const int x = static_cast<int>(std::lround(prediction.x)) + side * offset;
        if (x < 0 || x >= image.cols) { continue; }
        const double expected = prediction.y + heightOffset + slope * (x - prediction.x);
        auto peaks = findPeaks(image, x, expected, searchRadius);
        if (peaks.empty()) { continue; }
        std::sort(peaks.begin(), peaks.end(), [expected, followHistory](const Peak &lhs, const Peak &rhs) {
            return followHistory ? std::abs(lhs.y - expected) < std::abs(rhs.y - expected)
                : lhs.contrast > rhs.contrast;
        });
        bool ambiguous = false;
        for (std::size_t i = 1; i < peaks.size(); ++i) {
            const bool competing = followHistory
                ? std::abs(std::abs(peaks[i].y - expected)
                    - std::abs(peaks.front().y - expected)) < 1.5
                : peaks[i].contrast >= peaks.front().contrast * 0.8;
            if (competing && std::abs(peaks[i].y - peaks.front().y) >= 7.0) {
                ambiguous = true;
            }
        }
        if (ambiguous) { ++result.ambiguousColumns; continue; }
        if (std::abs(peaks.front().y - expected) > searchRadius) { continue; }
        samples.push_back({x - prediction.x, peaks.front().y});
    }
    result.count = static_cast<int>(samples.size());
    if (result.count < minimumColumns) { return result; }
    double fittedSlope = slope;
    double fittedIntercept = prediction.y + heightOffset;
    for (int iteration = 0; iteration < 4; ++iteration) {
        double sw = 0.0;
        double sx = 0.0;
        double sy = 0.0;
        double sxx = 0.0;
        double sxy = 0.0;
        for (const auto &sample : samples) {
            const double error = sample.y - (fittedIntercept + fittedSlope * sample.x);
            const double weight = iteration == 0 ? 1.0
                : std::min(1.0, 1.5 / std::max(std::abs(error), 1e-9));
            sw += weight;
            sx += weight * sample.x;
            sy += weight * sample.y;
            sxx += weight * sample.x * sample.x;
            sxy += weight * sample.x * sample.y;
        }
        const double determinant = sw * sxx - sx * sx;
        if (determinant <= 1e-9) { return result; }
        fittedSlope = (sw * sxy - sx * sy) / determinant;
        fittedIntercept = (sy - fittedSlope * sx) / sw;
    }
    double errorSquared = 0.0;
    for (const auto &sample : samples) {
        const double error = sample.y - fittedIntercept - fittedSlope * sample.x;
        errorSquared += error * error;
    }
    result.slope = fittedSlope;
    result.intercept = fittedIntercept;
    result.rms = std::sqrt(errorSquared / samples.size());
    result.valid = std::isfinite(result.intercept) && result.rms <= 1.8
        && std::abs(result.slope - slope) <= 0.25;
    return result;
}

/** @brief 旧入口保持按亮度取峰；在线历史入口显式采用最近连续分支。 */
Branch fitBranch(const cv::Mat &image, const cv::Point2d &prediction,
    int side, int firstOffset, int lastOffset, double slope,
    double heightOffset, double searchRadius, int minimumColumns)
{
    return fitBranch(image, prediction, side, firstOffset, lastOffset, slope,
        heightOffset, searchRadius, minimumColumns, false);
}

/**
 * @brief 只在当前双肩部已成立时，沿其窄峰恢复可见折点；结果仍是带历史先验的估计。
 * @details 多条纹原图不能整体通过独立精定位时，保留距当前母材线最近的原始窄峰，
 * 再检查中心的真实局部支撑。预焊点凸起不在母材带内，不能靠遮挡外推伪造可见折点。
 */
OnlineSeamRefinementResult refineSupportedCenter(const cv::Mat &gray8,
    const cv::Point2d &prediction, const Branch &left, const Branch &right,
    double radius)
{
    cv::Mat supported = cv::Mat::zeros(gray8.size(), CV_8UC1);
    const int begin = std::max(0, static_cast<int>(std::lround(prediction.x)) - 100);
    const int end = std::min(gray8.cols - 1,
        static_cast<int>(std::lround(prediction.x)) + 100);
    for (int x = begin; x <= end; ++x) {
        const double dx = x - prediction.x;
        const double leftY = left.intercept + left.slope * dx;
        const double rightY = right.intercept + right.slope * dx;
        // 中心可能平滑移动：在有限转折区保留两侧线的并集，不能在旧 X
        // 硬切左右掩膜，否则会人为制造恰好位于旧 X 的台阶。
        const bool allowLeft = dx <= 12.0;
        const bool allowRight = dx >= -12.0;
        auto peaks = findPeaks(gray8, x, allowLeft ? leftY : rightY, radius);
        if (allowLeft && allowRight) {
            const auto rightPeaks = findPeaks(gray8, x, rightY, radius);
            for (const auto &peak : rightPeaks) {
                if (std::none_of(peaks.begin(), peaks.end(), [&peak](const Peak &old) {
                    return std::abs(old.y - peak.y) < 1.0;
                })) { peaks.push_back(peak); }
            }
        }
        if (peaks.empty()) { continue; }
        const auto distance = [allowLeft, allowRight, leftY, rightY](const Peak &peak) {
            return std::min(allowLeft ? std::abs(peak.y - leftY) : 1e9,
                allowRight ? std::abs(peak.y - rightY) : 1e9);
        };
        const auto nearest = std::min_element(peaks.begin(), peaks.end(),
            [&distance](const Peak &a, const Peak &b) { return distance(a) < distance(b); });
        if (distance(*nearest) > radius) { continue; }
        const int centerY = static_cast<int>(std::lround(nearest->y));
        for (int y = std::max(0, centerY - 4);
             y <= std::min(gray8.rows - 1, centerY + 4); ++y) {
            supported.at<unsigned char>(y, x) = gray8.at<unsigned char>(y, x);
        }
    }
    OnlineSeamRefinementOptions options;
    options.heightPolicy = OnlineSeamHeightPolicy::GapCenter;
    auto result = COnlineSeamRefiner::refine(supported, prediction, options);
    const double dx = result.point.x - prediction.x;
    if (result.valid && (std::abs(left.intercept + left.slope * dx - result.leftEndpoint.y) > 2.0
        || std::abs(right.intercept + right.slope * dx - result.rightEndpoint.y) > 2.0)) {
        result.valid = false;
    }
    return result;
}

double referenceSlope(const std::vector<cv::Point2d> &polyline,
    const cv::Point2d &seam, int side, double fallback)
{
    double bestDistance = 1e30;
    double bestSlope = fallback;
    for (std::size_t i = 1; i < polyline.size(); ++i) {
        const auto &a = polyline[i - 1];
        const auto &b = polyline[i];
        const double dx = b.x - a.x;
        const double center = (a.x + b.x) * 0.5 - seam.x;
        if (side * center <= 0.0 || std::abs(dx) < 2.0) { continue; }
        const double slope = (b.y - a.y) / dx;
        if (std::isfinite(slope) && std::abs(slope) <= 2.0
            && std::abs(center) < bestDistance) {
            bestDistance = std::abs(center);
            bestSlope = slope;
        }
    }
    return bestSlope;
}

/**
 * @brief 以新候选本身的原图支撑确认重捕获资格，不靠它与仿真点接近来确认。
 * @details 该门槛比普通原图候选更严格；明显多峰、宽峰和单侧支撑不能换掉旧锚。
 */
bool reacquisitionSupported(const cv::Mat &gray8,
    const OnlineSeamRefinementResult &raw,
    const std::vector<cv::Point2d> &polyline, const cv::Point2d &simulated)
{
    if (!std::isfinite(raw.supportRatio) || !std::isfinite(raw.fitRmsPixel)
        || !std::isfinite(raw.xUncertaintyPixel) || !std::isfinite(raw.ambiguousColumnRatio)
        || !std::isfinite(raw.confidence) || !std::isfinite(raw.leftEndpoint.y)
        || !std::isfinite(raw.rightEndpoint.y)
        || raw.supportRatio < 0.85 || raw.fitRmsPixel > 1.0
        || raw.xUncertaintyPixel > 3.0 || raw.ambiguousColumnRatio > 0.05
        || raw.confidence < 0.35 || raw.leftSupportColumns < 25
        || raw.rightSupportColumns < 25) { return false; }
    const auto left = fitBranch(gray8, raw.point, -1, 25, 70,
        referenceSlope(polyline, simulated, -1, 0.3),
        raw.leftEndpoint.y - raw.point.y, 20.0, 25);
    const auto right = fitBranch(gray8, raw.point, 1, 25, 70,
        referenceSlope(polyline, simulated, 1, -0.3),
        raw.rightEndpoint.y - raw.point.y, 20.0, 25);
    return left.valid && right.valid && left.rms <= 1.0 && right.rms <= 1.0
        && left.ambiguousColumns == 0 && right.ambiguousColumns == 0
        && std::abs(left.intercept - raw.leftEndpoint.y) <= 1.5
        && std::abs(right.intercept - raw.rightEndpoint.y) <= 1.5;
}

/**
 * @brief 独立检查旧实测分支是否仍存在，不以更亮新峰将旧峰解释成已消失。
 * @details 选择最接近历史预测的原图窄峰，再检查整侧的支撑、端部和拟合残差；
 * 历史预测只用于识别竞争证据，不据此生成新的实测点或放宽候选准入。
 */
bool historicalBranchPresent(const cv::Mat &gray8, const cv::Point2d &prediction,
    int side, double slope, double heightOffset, double searchRadius, int minimumColumns)
{
    std::vector<cv::Point2d> samples;
    for (int offset = 25; offset <= 70; ++offset) {
        const int x = static_cast<int>(std::lround(prediction.x)) + side * offset;
        if (x < 0 || x >= gray8.cols) { continue; }
        const double dx = x - prediction.x;
        const double expected = prediction.y + heightOffset + slope * dx;
        const auto peaks = findPeaks(gray8, x, expected, searchRadius);
        if (peaks.empty()) { continue; }
        const auto nearest = std::min_element(peaks.begin(), peaks.end(),
            [expected](const Peak &left, const Peak &right) {
                return std::abs(left.y - expected) < std::abs(right.y - expected);
            });
        if (std::abs(nearest->y - expected) <= searchRadius) {
            samples.push_back({dx, nearest->y});
        }
    }
    if (samples.size() < static_cast<std::size_t>(minimumColumns)) { return false; }
    double sx = 0.0;
    double sy = 0.0;
    double sxx = 0.0;
    double sxy = 0.0;
    for (const auto &sample : samples) {
        sx += sample.x;
        sy += sample.y;
        sxx += sample.x * sample.x;
        sxy += sample.x * sample.y;
    }
    const double count = static_cast<double>(samples.size());
    const double determinant = count * sxx - sx * sx;
    if (determinant <= 1e-9) { return false; }
    const double fittedSlope = (count * sxy - sx * sy) / determinant;
    const double fittedIntercept = (sy - fittedSlope * sx) / count;
    double errorSquared = 0.0;
    for (const auto &sample : samples) {
        const double error = sample.y - fittedIntercept - fittedSlope * sample.x;
        errorSquared += error * error;
    }
    return std::isfinite(fittedIntercept) && std::abs(fittedSlope - slope) <= 0.25
        && std::abs(fittedIntercept - prediction.y - heightOffset) <= 3.0
        && std::sqrt(errorSquared / count) <= 1.8;
}

} // namespace

void CSimulationGuidedSeamEstimator::reset()
{
    m_imageContinuityActive = false;
    m_hasImagePosition = false;
    m_imagePoint = {};
    m_imageVelocity = {};
    m_lastImageSupportedPoint = {};
    m_hasAnchor = false;
    m_hasDirections = false;
    m_referenceOffset = {};
    m_versionAnchorOffset = {};
    m_anchorSimulation = {};
    m_degradedFrames = 0;
    m_reacquisitionOffset = {};
    m_reacquisitionGapHeight = 0.0;
    m_reacquisitionFrames = 0;
}

SimulationGuidedSeamResult CSimulationGuidedSeamEstimator::update(
    const cv::Mat &gray8, const std::vector<cv::Point2d> &simulatedPolyline,
    const cv::Point2d &simulatedSeam,
    const OnlineSeamRefinementResult &rawRefinement,
    std::uint64_t referenceRevision, const SimulationGuidedSeamOptions &options)
{
    return update(gray8, simulatedPolyline, simulatedSeam, rawRefinement,
        referenceRevision, options, {-1.0, -1.0});
}

SimulationGuidedSeamResult CSimulationGuidedSeamEstimator::update(
    const cv::Mat &gray8, const std::vector<cv::Point2d> &simulatedPolyline,
    const cv::Point2d &simulatedSeam,
    const OnlineSeamRefinementResult &rawRefinement,
    std::uint64_t referenceRevision, const SimulationGuidedSeamOptions &options,
    const cv::Point2d &transitionTarget)
{
    SimulationGuidedSeamResult result;
    result.attempted = true;
    if (gray8.empty() || gray8.type() != CV_8UC1
        || !inside(simulatedSeam, gray8.size())
        || options.maximumDegradedFrames < 1
        || options.maximumPredictionTimeMs < 1
        || !std::isfinite(options.maximumReferenceTravelPixel)
        || options.maximumReferenceTravelPixel <= 0.0
        || !std::isfinite(options.maximumLocalCorrectionPixel)
        || options.maximumLocalCorrectionPixel <= 0.0
        || !std::isfinite(options.maximumBranchDistancePixel)
        || options.maximumBranchDistancePixel < 1.0
        || options.maximumBranchDistancePixel > 30.0
        || !std::isfinite(options.maximumAnchorInnovationPixel)
        || options.maximumAnchorInnovationPixel <= 0.0
        || !std::isfinite(options.maximumAccumulatedAnchorCorrectionPixel)
        || options.maximumAccumulatedAnchorCorrectionPixel <= 0.0
        || options.minimumShoulderColumns < 8
        || options.minimumShoulderColumns > 46) {
        reset();
        return result;
    }
    if (options.enableImageContinuity) {
        if (!m_imageContinuityActive || gray8.size() != m_imageSize) {
            reset();
            m_imageContinuityActive = true;
            m_imageSize = gray8.size();
        }
        // CAD 写回既不是新的相机图像，也不是新的测量，不清锚、不续期预算。
        m_referenceRevision = referenceRevision;
        return updateImageContinuity(gray8, simulatedPolyline, simulatedSeam,
            rawRefinement, options, transitionTarget);
    }
    if (m_imageContinuityActive) { reset(); }
    if (gray8.size() != m_imageSize || referenceRevision != m_referenceRevision) {
        reset();
        m_imageSize = gray8.size();
        m_referenceRevision = referenceRevision;
    }
    if (!m_hasAnchor && inside(transitionTarget, gray8.size())) {
        // 仅保留业务坐标残差；不继承旧版本的测量身份、方向或预算。
        m_referenceOffset = transitionTarget - simulatedSeam;
    }
    const bool rawUsable = rawRefinement.valid && inside(rawRefinement.point, gray8.size());
    if (rawUsable && m_hasAnchor) {
        const auto offset = rawRefinement.point - simulatedSeam;
        result.accumulatedCorrectionExceeded = cv::norm(offset - m_versionAnchorOffset)
            > options.maximumAccumulatedAnchorCorrectionPixel;
        result.rawJumpRejected = cv::norm(offset - m_referenceOffset)
            > options.maximumAnchorInnovationPixel;
        if (!result.rawJumpRejected && m_hasDirections) {
            // 校核上一可信分支的当前原图支撑，不围绕候选另找一条更像 CAD 的线。
            const cv::Point2d prediction = simulatedSeam + m_referenceOffset;
            const double leftDirection = m_leftSlope + std::clamp(
                referenceSlope(simulatedPolyline, simulatedSeam, -1, m_anchorLeftModelSlope)
                    - m_anchorLeftModelSlope, -0.25, 0.25);
            const double rightDirection = m_rightSlope + std::clamp(
                referenceSlope(simulatedPolyline, simulatedSeam, 1, m_anchorRightModelSlope)
                    - m_anchorRightModelSlope, -0.25, 0.25);
            const auto left = fitBranch(gray8, prediction, -1, 25, 70,
                leftDirection, m_leftHeightOffset, options.maximumBranchDistancePixel,
                options.minimumShoulderColumns);
            const auto right = fitBranch(gray8, prediction, 1, 25, 70,
                rightDirection, m_rightHeightOffset, options.maximumBranchDistancePixel,
                options.minimumShoulderColumns);
            const double dx = rawRefinement.point.x - prediction.x;
            result.rawJumpRejected = !left.valid || !right.valid
                || std::abs(left.intercept + left.slope * dx - rawRefinement.leftEndpoint.y) > 3.0
                || std::abs(right.intercept + right.slope * dx - rawRefinement.rightEndpoint.y) > 3.0;
        }
    }
    const bool withinReacquisitionRange = rawUsable
        && cv::norm(rawRefinement.point - simulatedSeam - m_referenceOffset)
            <= options.maximumAnchorInnovationPixel * 2.0;
    if (options.enableRawReacquisition && result.rawJumpRejected
        && withinReacquisitionRange && m_hasDirections) {
        const cv::Point2d prediction = simulatedSeam + m_referenceOffset;
        const double leftDirection = m_leftSlope + std::clamp(
            referenceSlope(simulatedPolyline, simulatedSeam, -1, m_anchorLeftModelSlope)
                - m_anchorLeftModelSlope, -0.25, 0.25);
        const double rightDirection = m_rightSlope + std::clamp(
            referenceSlope(simulatedPolyline, simulatedSeam, 1, m_anchorRightModelSlope)
                - m_anchorRightModelSlope, -0.25, 0.25);
        result.reacquisitionBlockedByPreviousBranch = historicalBranchPresent(gray8,
            prediction, -1, leftDirection, m_leftHeightOffset,
            options.maximumBranchDistancePixel, options.minimumShoulderColumns)
            && historicalBranchPresent(gray8, prediction, 1, rightDirection, m_rightHeightOffset,
                options.maximumBranchDistancePixel, options.minimumShoulderColumns);
    }
    if (options.enableRawReacquisition && result.rawJumpRejected
        && withinReacquisitionRange && !result.reacquisitionBlockedByPreviousBranch
        && reacquisitionSupported(gray8, rawRefinement, simulatedPolyline, simulatedSeam)) {
        const auto offset = rawRefinement.point - simulatedSeam;
        const double gapHeight = rawRefinement.rightEndpoint.y - rawRefinement.leftEndpoint.y;
        const bool consecutive = m_reacquisitionFrames > 0
            && cv::norm(offset - m_reacquisitionOffset) <= 2.0
            && std::abs(gapHeight - m_reacquisitionGapHeight) <= 2.0;
        m_reacquisitionFrames = consecutive ? m_reacquisitionFrames + 1 : 1;
        m_reacquisitionOffset = offset;
        m_reacquisitionGapHeight = gapHeight;
        result.reacquisitionFrames = m_reacquisitionFrames;
        result.reacquisitionPending = m_reacquisitionFrames < 3;
        if (!result.reacquisitionPending) {
            // 三份连续高质量原图证据替换过旧的分支约束；本帧仍用原始候选，
            // 不把此前的粗 KCF、仿真或肩部估计变成独立实测。
            result.rawJumpRejected = false;
            result.reacquired = true;
        }
    } else {
        m_reacquisitionFrames = 0;
    }
    if (rawUsable && !result.rawJumpRejected) {
        if (!m_hasAnchor) {
            m_versionAnchorOffset = rawRefinement.point - simulatedSeam;
        }
        m_hasAnchor = true;
        m_referenceOffset = rawRefinement.point - simulatedSeam;
        m_anchorSimulation = simulatedSeam;
        m_anchorTime = std::chrono::steady_clock::now();
        m_degradedFrames = 0;
        m_leftHeightOffset = rawRefinement.leftEndpoint.y - rawRefinement.point.y;
        m_rightHeightOffset = rawRefinement.rightEndpoint.y - rawRefinement.point.y;
        const double leftPrior = referenceSlope(simulatedPolyline, simulatedSeam, -1, 0.3);
        const double rightPrior = referenceSlope(simulatedPolyline, simulatedSeam, 1, -0.3);
        m_anchorLeftModelSlope = leftPrior;
        m_anchorRightModelSlope = rightPrior;
        const auto left = fitBranch(gray8, rawRefinement.point, -1, 25, 70,
            leftPrior, m_leftHeightOffset, 20.0, options.minimumShoulderColumns);
        const auto right = fitBranch(gray8, rawRefinement.point, 1, 25, 70,
            rightPrior, m_rightHeightOffset, 20.0, options.minimumShoulderColumns);
        m_hasDirections = left.valid && right.valid;
        m_leftSlope = left.slope;
        m_rightSlope = right.slope;
        result.hasTrustedAnchor = true;
        result.predictionBudgetValid = true;
        m_reacquisitionFrames = 0;
        return result;
    }
    result.hasTrustedAnchor = m_hasAnchor;
    if (!m_hasAnchor) {
        result.prediction = simulatedSeam + m_referenceOffset;
        result.point = inside(result.prediction, gray8.size())
            ? result.prediction : simulatedSeam;
        result.valid = true;
        return result;
    }
    ++m_degradedFrames;
    result.degradedFrames = m_degradedFrames;
    result.anchorAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - m_anchorTime).count();
    result.referenceTravelPixel = cv::norm(simulatedSeam - m_anchorSimulation);
    result.predictionBudgetValid = m_degradedFrames <= options.maximumDegradedFrames
        && result.anchorAgeMs <= options.maximumPredictionTimeMs
        && result.referenceTravelPixel <= options.maximumReferenceTravelPixel;
    result.prediction = simulatedSeam + m_referenceOffset;
    result.point = result.prediction;
    // 当前预测始终由最后实测锚重新计算；估计修正不更新偏移，慢漂移不能累积。
    if (m_hasDirections && result.predictionBudgetValid) {
        // 仿真只提供方向变化；绝对方向仍以最近原图拟合为基准，避免 CAD
        // 更相似的较弱反光取代正确时序分支。突变只降低支撑，不生成新实测。
        const double leftDirection = m_leftSlope + std::clamp(
            referenceSlope(simulatedPolyline, simulatedSeam, -1, m_anchorLeftModelSlope)
                - m_anchorLeftModelSlope, -0.25, 0.25);
        const double rightDirection = m_rightSlope + std::clamp(
            referenceSlope(simulatedPolyline, simulatedSeam, 1, m_anchorRightModelSlope)
                - m_anchorRightModelSlope, -0.25, 0.25);
        const auto left = fitBranch(gray8, result.prediction, -1, 25, 70,
            leftDirection, m_leftHeightOffset, options.maximumBranchDistancePixel,
            options.minimumShoulderColumns);
        const auto right = fitBranch(gray8, result.prediction, 1, 25, 70,
            rightDirection, m_rightHeightOffset, options.maximumBranchDistancePixel,
            options.minimumShoulderColumns);
        result.leftSupportColumns = left.count;
        result.rightSupportColumns = right.count;
        result.ambiguousColumns = left.ambiguousColumns + right.ambiguousColumns;
        result.ambiguous = result.ambiguousColumns > 0;
        result.maximumBranchRmsPixel = std::max(left.rms, right.rms);
        result.hasRawShoulderSupport = left.valid && right.valid;
        if (result.hasRawShoulderSupport) {
            const auto innerLeft = fitBranch(gray8, result.prediction, -1, 5, 20,
                left.slope, left.intercept - result.prediction.y,
                options.maximumBranchDistancePixel, 8);
            const auto innerRight = fitBranch(gray8, result.prediction, 1, 5, 20,
                right.slope, right.intercept - result.prediction.y,
                options.maximumBranchDistancePixel, 8);
            result.localGapCenterSupported = innerLeft.valid && innerRight.valid;
            const double height = result.localGapCenterSupported
                ? (innerLeft.intercept + innerRight.intercept) * 0.5
                : (left.intercept + right.intercept) * 0.5;
            result.point.y += std::clamp(height - result.prediction.y,
                -options.maximumLocalCorrectionPixel,
                options.maximumLocalCorrectionPixel);
        }
    }
    if (!inside(result.point, gray8.size())) {
        // 历史残差离开视野时只能保留同帧参考估计，不能将边界截断伪装成测量。
        result.point = simulatedSeam;
        result.hasRawShoulderSupport = false;
        result.predictionBudgetValid = false;
    }
    result.valid = true;
    return result;
}

SimulationGuidedSeamResult CSimulationGuidedSeamEstimator::updateImageContinuity(
    const cv::Mat &gray8, const std::vector<cv::Point2d> &simulatedPolyline,
    const cv::Point2d &simulatedSeam,
    const OnlineSeamRefinementResult &rawRefinement,
    const SimulationGuidedSeamOptions &options, const cv::Point2d &transitionTarget)
{
    SimulationGuidedSeamResult result;
    result.attempted = true;
    result.imageContinuityActive = true;
    result.modelTranslationSuppressed = m_hasImagePosition;
    const bool hadImageAnchor = m_hasAnchor;
    const auto now = std::chrono::steady_clock::now();
    if (!m_hasImagePosition) {
        m_imagePoint = inside(transitionTarget, gray8.size()) ? transitionTarget : simulatedSeam;
        m_lastImageSupportedPoint = m_imagePoint;
        m_hasImagePosition = true;
        m_anchorTime = now;
        m_anchorSimulation = simulatedSeam;
    }
    // 只外推已被真实图像支持的近期运动；失去支撑时速度衰减且总外推有界。
    result.prediction = m_imagePoint + m_imageVelocity;
    if (!inside(result.prediction, gray8.size())) { result.prediction = m_imagePoint; }
    result.point = result.prediction;
    Branch left;
    Branch right;
    if (m_hasAnchor && m_hasDirections) {
        left = fitBranch(gray8, result.prediction, -1, 35, 90,
            m_leftSlope, m_leftHeightOffset, options.maximumBranchDistancePixel,
            options.minimumShoulderColumns, true);
        right = fitBranch(gray8, result.prediction, 1, 35, 90,
            m_rightSlope, m_rightHeightOffset, options.maximumBranchDistancePixel,
            options.minimumShoulderColumns, true);
        const double denominator = left.slope - right.slope;
        if (left.valid && right.valid && std::abs(denominator) >= 0.15
            && std::abs(left.slope - m_leftSlope) <= 0.12
            && std::abs(right.slope - m_rightSlope) <= 0.12) {
            // 预焊点遮住折点时，按历史间隙高度解两母材线的相对平移；
            // 不将凸起中心或 CAD 的绝对位置当成焊缝。左右端部可保留真实台阶。
            const double gap = m_rightHeightOffset - m_leftHeightOffset;
            const double dx = (right.intercept - left.intercept - gap) / denominator;
            const cv::Point2d supported(result.prediction.x + dx,
                0.5 * (left.intercept + right.intercept
                    + (left.slope + right.slope) * dx));
            result.hasRawShoulderSupport = inside(supported, gray8.size())
                && cv::norm(supported - result.prediction)
                    <= options.maximumAnchorInnovationPixel;
            if (result.hasRawShoulderSupport) {
                result.point = supported;
                const auto center = refineSupportedCenter(gray8, result.prediction,
                    left, right, options.maximumBranchDistancePixel);
                result.localGapCenterSupported = center.valid
                    && cv::norm(center.point - result.prediction)
                        <= options.maximumAnchorInnovationPixel;
                if (result.localGapCenterSupported) {
                    result.point = center.point;
                    // 当前中心确有窄峰支撑才更新加工间隙的时序形状；
                    // 此带历史筛选的点不输出为独立 refinement，也不刷新其年龄。
                    m_leftHeightOffset = center.leftEndpoint.y - center.point.y;
                    m_rightHeightOffset = center.rightEndpoint.y - center.point.y;
                }
                const double support = std::min(left.count, right.count) / 56.0;
                result.imageObservationQuality = std::clamp(support
                    / (1.0 + std::max(left.rms, right.rms)), 0.0, 1.0);
            }
        }
        result.leftSupportColumns = left.count;
        result.rightSupportColumns = right.count;
        result.ambiguousColumns = left.ambiguousColumns + right.ambiguousColumns;
        result.ambiguous = result.ambiguousColumns > 0;
        result.maximumBranchRmsPixel = std::max(left.rms, right.rms);
    }
    const bool rawUsable = rawRefinement.valid && inside(rawRefinement.point, gray8.size());
    if (rawUsable && m_hasAnchor) {
        const double rawDx = rawRefinement.point.x - result.prediction.x;
        // 中心仍可见时，以当前两侧原图线核对实测端部，允许真实间隙连续变化；
        // 历史 gap 只用于遮挡外推，不能反过来否决当前清晰的 GapCenter 实测。
        result.rawJumpRejected = result.hasRawShoulderSupport
            ? cv::norm(rawRefinement.point - result.prediction)
                    > options.maximumAnchorInnovationPixel
                || std::abs(left.intercept + left.slope * rawDx
                    - rawRefinement.leftEndpoint.y) > 3.0
                || std::abs(right.intercept + right.slope * rawDx
                    - rawRefinement.rightEndpoint.y) > 3.0
            : cv::norm(rawRefinement.point - result.prediction)
                > options.maximumLocalCorrectionPixel;
        // 新的清晰接缝必须连续自证；旧分支还在时，更亮的反光不能替换它。
        if (result.rawJumpRejected && options.enableRawReacquisition
            && cv::norm(rawRefinement.point - result.prediction)
                <= options.maximumAnchorInnovationPixel * 2.0) {
            result.reacquisitionBlockedByPreviousBranch = historicalBranchPresent(gray8,
                result.prediction, -1, m_leftSlope, m_leftHeightOffset,
                options.maximumBranchDistancePixel, options.minimumShoulderColumns)
                && historicalBranchPresent(gray8, result.prediction, 1, m_rightSlope,
                    m_rightHeightOffset, options.maximumBranchDistancePixel,
                    options.minimumShoulderColumns);
            if (!result.reacquisitionBlockedByPreviousBranch
                && reacquisitionSupported(gray8, rawRefinement, simulatedPolyline, simulatedSeam)) {
                const bool consecutive = m_reacquisitionFrames > 0
                    && cv::norm(rawRefinement.point - m_reacquisitionOffset) <= 2.0;
                m_reacquisitionFrames = consecutive ? m_reacquisitionFrames + 1 : 1;
                m_reacquisitionOffset = rawRefinement.point;
                result.reacquisitionFrames = m_reacquisitionFrames;
                result.reacquisitionPending = m_reacquisitionFrames < 3;
                if (!result.reacquisitionPending) {
                    result.rawJumpRejected = false;
                    result.reacquired = true;
                }
            } else { m_reacquisitionFrames = 0; }
        } else { m_reacquisitionFrames = 0; }
    } else { m_reacquisitionFrames = 0; }
    if (rawUsable && !result.rawJumpRejected) {
        const bool hasRawDirections = rawRefinement.hasBranchDirections
            && std::isfinite(rawRefinement.leftSlope) && std::isfinite(rawRefinement.rightSlope);
        // 已验收精定位拥有本帧真实方向，不能再靠 CAD 方向从反光中重选一次。
        const double leftPrior = hasRawDirections ? rawRefinement.leftSlope
            : (m_hasDirections ? m_leftSlope
                : referenceSlope(simulatedPolyline, simulatedSeam, -1, 0.3));
        const double rightPrior = hasRawDirections ? rawRefinement.rightSlope
            : (m_hasDirections ? m_rightSlope
                : referenceSlope(simulatedPolyline, simulatedSeam, 1, -0.3));
        m_leftHeightOffset = rawRefinement.leftEndpoint.y - rawRefinement.point.y;
        m_rightHeightOffset = rawRefinement.rightEndpoint.y - rawRefinement.point.y;
        const double initialDirectionRadius = m_hasDirections ? 7.0 : 20.0;
        left = fitBranch(gray8, rawRefinement.point, -1, 35, 90, leftPrior,
            m_leftHeightOffset, initialDirectionRadius, options.minimumShoulderColumns, true);
        right = fitBranch(gray8, rawRefinement.point, 1, 35, 90, rightPrior,
            m_rightHeightOffset, initialDirectionRadius, options.minimumShoulderColumns, true);
        m_hasDirections = hasRawDirections || (left.valid && right.valid);
        if (m_hasDirections) {
            m_leftSlope = hasRawDirections ? rawRefinement.leftSlope : left.slope;
            m_rightSlope = hasRawDirections ? rawRefinement.rightSlope : right.slope;
        }
        m_hasAnchor = true;
        m_anchorSimulation = simulatedSeam;
        m_anchorTime = now;
        m_degradedFrames = 0;
        result.point = rawRefinement.point;
        result.imageObservationQuality = 1.0;
        result.predictionBudgetValid = true;
        result.hasTrustedAnchor = true;
        result.localGapCenterSupported = true;
        m_reacquisitionFrames = 0;
    } else {
        ++m_degradedFrames;
        result.anchorAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - m_anchorTime).count();
        result.referenceTravelPixel = cv::norm(simulatedSeam - m_anchorSimulation);
        result.predictionBudgetValid = m_hasAnchor
            && m_degradedFrames <= options.maximumDegradedFrames
            && result.anchorAgeMs <= options.maximumPredictionTimeMs
            && result.referenceTravelPixel <= options.maximumReferenceTravelPixel;
        result.hasTrustedAnchor = m_hasAnchor;
        result.degradedFrames = m_degradedFrames;
        if (result.hasRawShoulderSupport) {
            // 续接方向可随真实图像小幅变化；不把肩部估计升级为独立焊点或重置实测年龄。
            m_leftSlope += 0.2 * (left.slope - m_leftSlope);
            m_rightSlope += 0.2 * (right.slope - m_rightSlope);
        }
    }
    if ((rawUsable && !result.rawJumpRejected) || result.hasRawShoulderSupport) {
        const auto motion = result.point - m_imagePoint;
        const double length = cv::norm(motion);
        const auto limitedMotion = length > 2.0 ? motion * (2.0 / length) : motion;
        m_imageVelocity = !hadImageAnchor || result.reacquired ? cv::Point2d()
            : 0.5 * m_imageVelocity + 0.5 * limitedMotion;
        m_lastImageSupportedPoint = result.point;
    } else {
        m_imageVelocity *= result.predictionBudgetValid ? 0.7 : 0.0;
        result.point = m_imagePoint + m_imageVelocity;
        const auto travel = result.point - m_lastImageSupportedPoint;
        const double distance = cv::norm(travel);
        if (distance > options.maximumLocalCorrectionPixel * 2.0) {
            result.point = m_lastImageSupportedPoint
                + travel * (options.maximumLocalCorrectionPixel * 2.0 / distance);
            m_imageVelocity = {};
        }
    }
    if (!inside(result.point, gray8.size())) { result.point = m_imagePoint; }
    m_imagePoint = result.point;
    result.valid = inside(result.point, gray8.size());
    return result;
}

} // namespace i2r_rlc::vision
