/**
 * @file OnlineSeamRefiner.cpp
 * @brief 原图列峰、稳健分段和可见支撑联合校核在线焊缝位置。
 */
#include "feature/OnlineSeamRefiner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "stripe/SubpixelCenterlineTools.h"

namespace i2r_rlc::vision {
namespace {

struct StripeSample {
    double x = 0.0;
    double y = 0.0;
    bool ambiguous = false;
};

struct LineFit {
    bool valid = false;
    double slope = 0.0;
    double intercept = 0.0;
};

struct SplitFit {
    bool valid = false;
    double x = 0.0;
    double score = std::numeric_limits<double>::max();
    double rms = 0.0;
    double supportRatio = 0.0;
    int leftSupport = 0;
    int rightSupport = 0;
    LineFit left;
    LineFit right;
};

double smoothColumn(const cv::Mat &gray8, int x, int y)
{
    return (gray8.at<unsigned char>(y - 1, x)
        + gray8.at<unsigned char>(y, x)
        + gray8.at<unsigned char>(y + 1, x)) / 3.0;
}

/** @brief 测量同一亮带半高宽，不把平台肩部当作独立窄条纹。 */
int peakWidth(const cv::Mat &gray8, int x, int peakY,
              int yBegin, int yEnd, double threshold)
{
    int width = 1;
    for (int y = peakY - 1; y >= yBegin
         && gray8.at<unsigned char>(y, x) > threshold; --y)
    {
        ++width;
    }
    for (int y = peakY + 1; y < yEnd
         && gray8.at<unsigned char>(y, x) > threshold; ++y)
    {
        ++width;
    }
    return width;
}

/**
 * @brief 检查独立窄次峰；峰间谷必须下降，防止同一条纹的起伏双计。
 */
bool hasCompetingPeak(const cv::Mat &gray8, int x, int peakY,
                      int yBegin, int yEnd, double background, double peak,
                      const OnlineSeamRefinementOptions &options)
{
    for (int y = yBegin + 2; y + 2 < yEnd; ++y)
    {
        if (std::abs(y - peakY) < options.minPeakSeparationPixel)
        {
            continue;
        }
        const double secondary = smoothColumn(gray8, x, y);
        if (secondary - background < options.minContrastGrayValue
            || secondary - background < options.secondaryPeakRatio * (peak - background)
            || secondary < smoothColumn(gray8, x, y - 1)
            || secondary < smoothColumn(gray8, x, y + 1))
        {
            continue;
        }
        if (peakWidth(gray8, x, y, yBegin, yEnd,
            background + 0.5 * (secondary - background)) > 12)
        {
            continue;
        }
        double valley = peak;
        for (int row = std::min(y, peakY) + 1;
             row < std::max(y, peakY); ++row)
        {
            valley = std::min(valley, smoothColumn(gray8, x, row));
        }
        if (valley < background + 0.65 * (std::min(peak, secondary) - background))
        {
            return true;
        }
    }
    return false;
}

/** @brief 拒绝过宽亮带，避免把饱和反光区域的边界当成激光中心。 */
bool extractColumn(const cv::Mat &gray8, int x, int yBegin, int yEnd,
                   const OnlineSeamRefinementOptions &options, StripeSample *sample)
{
    int peakY = yBegin;
    double peak = -1.0;
    int background = 255;
    for (int y = yBegin; y < yEnd; ++y)
    {
        const int value = gray8.at<unsigned char>(y, x);
        background = std::min(background, value);
        if (y > yBegin && y + 1 < yEnd)
        {
            const double smooth = smoothColumn(gray8, x, y);
            if (smooth > peak)
            {
                peak = smooth;
                peakY = y;
            }
        }
    }
    if (peak - background < options.minContrastGrayValue
        || peakY - yBegin < 5 || yEnd - peakY <= 5)
    {
        return false;
    }

    const double threshold = background + 0.5 * (peak - background);
    const int width = peakWidth(gray8, x, peakY, yBegin, yEnd, threshold);
    if (width > 12)
    {
        return false;
    }

    const auto centroid = CSubpixelCenterlineTools::calculateGrayCentroid(
        gray8, x, peakY, 3, background, nullptr);
    if (!centroid.valid)
    {
        return false;
    }
    sample->x = static_cast<double>(x);
    sample->y = centroid.centerYPixel;
    sample->ambiguous = hasCompetingPeak(gray8, x, peakY,
        yBegin, yEnd, background, peak, options);
    return true;
}

/** @brief Huber 权重抑制散斑离群点，坐标在分割点附近中心化以防病态。 */
LineFit fitLine(const std::vector<StripeSample> &samples,
                std::size_t begin, std::size_t end, double splitX)
{
    LineFit result;
    for (int iteration = 0; iteration < 5; ++iteration)
    {
        double weightSum = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXX = 0.0;
        double sumXY = 0.0;
        for (std::size_t i = begin; i < end; ++i)
        {
            const double x = samples[i].x - splitX;
            const double residual = samples[i].y
                - (result.intercept + result.slope * x);
            const double weight = iteration == 0 ? 1.0
                : std::min(1.0, 2.0 / std::max(std::fabs(residual), 1e-9));
            weightSum += weight;
            sumX += weight * x;
            sumY += weight * samples[i].y;
            sumXX += weight * x * x;
            sumXY += weight * x * samples[i].y;
        }
        const double determinant = weightSum * sumXX - sumX * sumX;
        if (determinant <= 1e-9)
        {
            return LineFit();
        }
        result.slope = (weightSum * sumXY - sumX * sumY) / determinant;
        result.intercept = (sumY - result.slope * sumX) / weightSum;
        result.valid = true;
    }
    return result;
}

/** @brief 分割两侧独立拟合，允许台阶而不强迫两线相交。 */
SplitFit evaluateSplit(const std::vector<StripeSample> &samples,
                       std::size_t split, double splitX)
{
    SplitFit result;
    result.x = splitX;
    result.left = fitLine(samples, 0, split, splitX);
    result.right = fitLine(samples, split, samples.size(), splitX);
    if (!result.left.valid || !result.right.valid
        || result.left.slope <= 0.05 || result.right.slope >= -0.05)
    {
        return result;
    }

    double score = 0.0;
    double squaredError = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        const auto &line = i < split ? result.left : result.right;
        const double residual = samples[i].y
            - line.intercept - line.slope * (samples[i].x - splitX);
        const double absolute = std::fabs(residual);
        score += absolute <= 2.0 ? residual * residual : 4.0 * absolute - 4.0;
        if (absolute <= 3.0)
        {
            squaredError += residual * residual;
            if (i < split)
            {
                ++result.leftSupport;
            }
            else
            {
                ++result.rightSupport;
            }
        }
    }
    const int support = result.leftSupport + result.rightSupport;
    result.score = score / static_cast<double>(samples.size());
    result.supportRatio = static_cast<double>(support)
        / static_cast<double>(samples.size());
    result.rms = support > 0 ? std::sqrt(squaredError / support) : 0.0;
    result.valid = true;
    return result;
}

/**
 * @brief 搜索所有有效分割，并保留近优解宽度；宽平台表示 X 不可辨识。
 */
SplitFit findSplit(const std::vector<StripeSample> &samples,
                   const OnlineSeamRefinementOptions &options,
                   double *uncertainty)
{
    SplitFit best;
    const auto minimum = static_cast<std::size_t>(options.minSideSupportColumns);
    if (samples.size() < 2 * minimum)
    {
        return best;
    }
    std::vector<SplitFit> candidates;
    candidates.reserve(samples.size());
    for (std::size_t split = minimum; split + minimum <= samples.size(); ++split)
    {
        const double x = 0.5 * (samples[split - 1].x + samples[split].x);
        auto fit = evaluateSplit(samples, split, x);
        if (fit.valid)
        {
            candidates.push_back(fit);
            if (!best.valid || fit.score < best.score)
            {
                best = fit;
            }
        }
    }
    double minX = best.x;
    double maxX = best.x;
    for (const auto &fit : candidates)
    {
        if (fit.score <= best.score + 0.01)
        {
            minX = std::min(minX, fit.x);
            maxX = std::max(maxX, fit.x);
        }
    }
    *uncertainty = maxX - minX;
    return best;
}

bool validOptions(const OnlineSeamRefinementOptions &options)
{
    return options.searchHalfWidthPixel >= 40
        && options.searchHalfWidthPixel <= 1000
        && options.searchHalfHeightPixel >= 10
        && options.searchHalfHeightPixel <= 1000
        && options.minSideSupportColumns >= 10
        && options.minSideSupportColumns < options.searchHalfWidthPixel
        && std::isfinite(options.minContrastGrayValue)
        && options.minContrastGrayValue > 0.0
        && options.minContrastGrayValue <= 255.0
        && std::isfinite(options.minSupportRatio)
        && options.minSupportRatio > 0.0 && options.minSupportRatio <= 1.0
        && std::isfinite(options.maxFitRmsPixel) && options.maxFitRmsPixel > 0.0
        && std::isfinite(options.maxPositionUncertaintyPixel)
        && options.maxPositionUncertaintyPixel > 0.0
        && std::isfinite(options.maxHeightJumpPixel)
        && options.maxHeightJumpPixel >= 0.0
        && std::isfinite(options.maxGapHeightPixel)
        && options.maxGapHeightPixel > 0.0
        && options.minPeakSeparationPixel >= 6
        && options.minPeakSeparationPixel <= options.searchHalfHeightPixel
        && std::isfinite(options.secondaryPeakRatio)
        && options.secondaryPeakRatio > 0.0 && options.secondaryPeakRatio <= 1.0
        && std::isfinite(options.maxAmbiguousColumnRatio)
        && options.maxAmbiguousColumnRatio >= 0.0
        && options.maxAmbiguousColumnRatio < 1.0
        && (options.heightPolicy == OnlineSeamHeightPolicy::RequireContinuous
            || options.heightPolicy == OnlineSeamHeightPolicy::GapCenter);
}

} // namespace

OnlineSeamRefinementResult COnlineSeamRefiner::refine(
    const cv::Mat &gray8,
    const cv::Point2d &coarsePoint,
    const OnlineSeamRefinementOptions &options)
{
    OnlineSeamRefinementResult result;
    if (gray8.empty() || gray8.type() != CV_8UC1
        || !std::isfinite(coarsePoint.x) || !std::isfinite(coarsePoint.y)
        || coarsePoint.x < 0.0 || coarsePoint.x >= gray8.cols
        || coarsePoint.y < 0.0 || coarsePoint.y >= gray8.rows
        || !validOptions(options))
    {
        return result;
    }

    const int centerX = cvRound(coarsePoint.x);
    const int centerY = cvRound(coarsePoint.y);
    const int xBegin = std::max(0, centerX - options.searchHalfWidthPixel);
    const int xEnd = std::min(gray8.cols, centerX + options.searchHalfWidthPixel + 1);
    const int yBegin = std::max(0, centerY - options.searchHalfHeightPixel);
    const int yEnd = std::min(gray8.rows, centerY + options.searchHalfHeightPixel + 1);
    std::vector<StripeSample> samples;
    samples.reserve(static_cast<std::size_t>(xEnd - xBegin));
    for (int x = xBegin; x < xEnd; ++x)
    {
        StripeSample sample;
        if (extractColumn(gray8, x, yBegin, yEnd, options, &sample))
        {
            samples.push_back(sample);
        }
    }
    const auto ambiguous = std::count_if(samples.begin(), samples.end(),
        [](const StripeSample &sample) { return sample.ambiguous; });
    result.ambiguousColumnRatio = samples.empty() ? 0.0
        : static_cast<double>(ambiguous) / static_cast<double>(samples.size());
    if (result.ambiguousColumnRatio > options.maxAmbiguousColumnRatio)
    {
        result.status = OnlineSeamRefinementStatus::AmbiguousStripe;
        return result;
    }
    result.status = OnlineSeamRefinementStatus::InsufficientSupport;
    if (samples.size() < static_cast<std::size_t>(2 * options.minSideSupportColumns)
        || static_cast<double>(samples.size()) / (xEnd - xBegin)
            < options.minSupportRatio)
    {
        return result;
    }

    const auto fit = findSplit(samples, options, &result.xUncertaintyPixel);
    result.status = OnlineSeamRefinementStatus::InconsistentGeometry;
    if (!fit.valid)
    {
        return result;
    }
    result.leftEndpoint = cv::Point2d(fit.x, fit.left.intercept);
    result.rightEndpoint = cv::Point2d(fit.x, fit.right.intercept);
    result.heightJumpPixel = fit.right.intercept - fit.left.intercept;
    result.leftSupportColumns = fit.leftSupport;
    result.rightSupportColumns = fit.rightSupport;
    result.fitRmsPixel = fit.rms;
    result.supportRatio = fit.supportRatio * static_cast<double>(samples.size())
        / (xEnd - xBegin);
    if (fit.supportRatio < options.minSupportRatio
        || fit.rms > options.maxFitRmsPixel
        || fit.leftSupport < options.minSideSupportColumns
        || fit.rightSupport < options.minSideSupportColumns)
    {
        return result;
    }

    // 另一搜索宽度必须给出一致的 X，避免窗口内反光或曲面弯曲主导分割。
    std::vector<StripeSample> innerSamples;
    const double innerRadius = options.searchHalfWidthPixel * 0.85;
    for (const auto &sample : samples)
    {
        if (std::fabs(sample.x - coarsePoint.x) <= innerRadius)
        {
            innerSamples.push_back(sample);
        }
    }
    double innerUncertainty = 0.0;
    const auto inner = findSplit(innerSamples, options, &innerUncertainty);
    result.status = OnlineSeamRefinementStatus::UnstablePosition;
    if (!inner.valid || std::fabs(inner.x - fit.x) > 3.0
        || result.xUncertaintyPixel > options.maxPositionUncertaintyPixel
        || innerUncertainty > options.maxPositionUncertaintyPixel
        || fit.x - xBegin < options.minSideSupportColumns
        || xEnd - fit.x < options.minSideSupportColumns)
    {
        return result;
    }

    // 焊点必须有紧邻原图条纹支撑；不允许跨越缺线区外推产生二维目标。
    const StripeSample *nearest = nullptr;
    double distance = std::numeric_limits<double>::max();
    double leftDistance = distance;
    double rightDistance = distance;
    int localLeftSupport = 0;
    int localRightSupport = 0;
    for (const auto &sample : samples)
    {
        const auto &line = sample.x < fit.x ? fit.left : fit.right;
        if (std::fabs(sample.y - line.intercept
            - line.slope * (sample.x - fit.x)) > 3.0)
        {
            continue;
        }
        const double dx = std::fabs(sample.x - fit.x);
        if (sample.x < fit.x)
        {
            leftDistance = std::min(leftDistance, dx);
            if (dx <= 8.0)
            {
                ++localLeftSupport;
            }
        }
        else
        {
            rightDistance = std::min(rightDistance, dx);
            if (dx <= 8.0)
            {
                ++localRightSupport;
            }
        }
        if (dx < distance)
        {
            nearest = &sample;
            distance = dx;
        }
    }
    if (nearest == nullptr || leftDistance > 3.0 || rightDistance > 3.0
        || localLeftSupport < 3 || localRightSupport < 3)
    {
        result.status = OnlineSeamRefinementStatus::InsufficientSupport;
        return result;
    }
    result.horizontalValid = true;
    result.confidence = result.supportRatio
        / (1.0 + fit.rms + 0.1 * result.xUncertaintyPixel);
    result.status = OnlineSeamRefinementStatus::AmbiguousHeight;
    if (options.heightPolicy == OnlineSeamHeightPolicy::GapCenter)
    {
        const double supportedGapLimit = std::min(options.maxGapHeightPixel,
            options.searchHalfHeightPixel * 0.25);
        if (std::fabs(result.heightJumpPixel) > supportedGapLimit)
        {
            return result;
        }
        // 业务已明确接缝中部语义；连续与台阶统一用相同表达式，
        // 不在 maxHeightJumpPixel 门槛处切换不同的纵向目标。
        result.point = cv::Point2d(fit.x,
            0.5 * (fit.left.intercept + fit.right.intercept));
    }
    else if (std::fabs(result.heightJumpPixel) > options.maxHeightJumpPixel)
    {
        return result;
    }
    else
    {
        // RequireContinuous 保持实测中心行，不假定台阶中部语义。
        result.point = cv::Point2d(fit.x, nearest->y);
    }
    result.valid = true;
    result.hasBranchDirections = true;
    result.leftSlope = fit.left.slope;
    result.rightSlope = fit.right.slope;
    result.status = OnlineSeamRefinementStatus::Succeeded;
    return result;
}

} // namespace i2r_rlc::vision
