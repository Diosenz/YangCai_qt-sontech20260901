/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsStartPointImageDetector.cpp
 * @brief     IBVS 起始点图像识别链路实现。
 * @details   本文件实现模板对齐、边缘提取、焊缝候选整理和起始点计算链路。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsStartPointImageDetector.h"
#include "IbvsImageProjectionTools.h"
#include "IbvsTemplateAlignmentTools.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double DEG_TO_RAD = CV_PI / 180.0;

struct SubpixelEdgeCandidate
{
    cv::Point2f point;
    double confidence = 0.0;
    bool valid = false;
};

struct SubpixelEdgeSet
{
    std::vector<cv::Point2f> points;
    double rmsePx = std::numeric_limits<double>::infinity();
    double confidenceMean = 0.0;
};

std::string makeFailure(
    IbvsStartPointImageResult& result,
    IbvsStatus status,
    const std::string& message)
{
    result.error = { status, message };
    result.failureReason = message;
    return message;
}

cv::Point toCvDrawPoint(const PixelPoint& point)
{
    return cv::Point(
        static_cast<int>(std::lround(point.u)),
        static_cast<int>(std::lround(point.v)));
}

cv::Mat extractEdges(const cv::Mat& bgrImage)
{
    cv::Mat gray;
    if (bgrImage.channels() == 3)
    {
        cv::cvtColor(bgrImage, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = bgrImage.clone();
    }

    cv::Mat enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.5, cv::Size(4, 4));
    clahe->apply(gray, enhanced);

    cv::Mat blurred;
    cv::bilateralFilter(enhanced, blurred, 9, 75.0, 75.0);

    cv::Mat sobelX;
    cv::Mat sobelY;
    cv::Sobel(blurred, sobelX, CV_16S, 1, 0, 3);
    cv::Sobel(blurred, sobelY, CV_16S, 0, 1, 3);

    cv::Mat absX;
    cv::Mat absY;
    cv::convertScaleAbs(sobelX, absX);
    cv::convertScaleAbs(sobelY, absY);

    cv::Mat sobelEnhanced;
    cv::addWeighted(absX, 0.99, absY, 0.6, 0.0, sobelEnhanced);

    cv::Mat edges;
    cv::Canny(sobelEnhanced, edges, 50.0, 160.0);

    if (bgrImage.channels() == 3)
    {
        cv::Mat hsv;
        cv::cvtColor(bgrImage, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(
            hsv,
            cv::Scalar(0, 15, 0),
            cv::Scalar(180, 255, 245),
            mask);
        cv::morphologyEx(
            mask,
            mask,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::bitwise_and(edges, mask, edges);
    }

    cv::morphologyEx(
        edges,
        edges,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::dilate(
        edges,
        edges,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
    return edges;
}

cv::Mat makeDistanceMap(const cv::Mat& edges)
{
    cv::Mat inverted;
    cv::bitwise_not(edges, inverted);

    cv::Mat distanceMap;
    cv::distanceTransform(
        inverted,
        distanceMap,
        cv::DIST_L2,
        cv::DIST_MASK_PRECISE);
    return distanceMap;
}

void drawPolyline(
    cv::Mat& image,
    const std::vector<cv::Point2f>& points,
    const cv::Scalar& color,
    int thickness)
{
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        cv::line(
            image,
            cv::Point(cvRound(points[i - 1].x), cvRound(points[i - 1].y)),
            cv::Point(cvRound(points[i].x), cvRound(points[i].y)),
            color,
            thickness,
            cv::LINE_AA);
    }
}

cv::Mat drawTemplateOverlay(
    const cv::Mat& bgrImage,
    const std::vector<std::vector<cv::Point2f>>& edges,
    const cv::Scalar& color)
{
    cv::Mat visual = bgrImage.clone();
    for (const auto& edge : edges)
    {
        drawPolyline(visual, edge, color, 2);
    }
    return visual;
}

cv::Mat drawContoursOverlay(const cv::Mat& bgrImage, const cv::Mat& edges)
{
    cv::Mat visual = bgrImage.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        edges.clone(),
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_NONE);
    cv::drawContours(visual, contours, -1, cv::Scalar(0, 255, 0), 1);
    return visual;
}

double sampleDistance(const cv::Mat& distanceMap, const cv::Point2f& point)
{
    if (point.x < 0.0f
        || point.y < 0.0f
        || point.x >= static_cast<float>(distanceMap.cols - 1)
        || point.y >= static_cast<float>(distanceMap.rows - 1))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const int x0 = static_cast<int>(point.x);
    const int y0 = static_cast<int>(point.y);
    const double dx = point.x - x0;
    const double dy = point.y - y0;
    const double v00 = distanceMap.at<float>(y0, x0);
    const double v01 = distanceMap.at<float>(y0, x0 + 1);
    const double v10 = distanceMap.at<float>(y0 + 1, x0);
    const double v11 = distanceMap.at<float>(y0 + 1, x0 + 1);
    return (1.0 - dx) * (1.0 - dy) * v00
        + dx * (1.0 - dy) * v01
        + (1.0 - dx) * dy * v10
        + dx * dy * v11;
}

double bilinearSample64(const cv::Mat& image, double x, double y)
{
    if (x < 0.0
        || y < 0.0
        || x >= image.cols - 1
        || y >= image.rows - 1)
    {
        return 0.0;
    }

    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const double dx = x - x0;
    const double dy = y - y0;
    const double v00 = image.at<double>(y0, x0);
    const double v01 = image.at<double>(y0, x0 + 1);
    const double v10 = image.at<double>(y0 + 1, x0);
    const double v11 = image.at<double>(y0 + 1, x0 + 1);
    return (1.0 - dx) * (1.0 - dy) * v00
        + dx * (1.0 - dy) * v01
        + (1.0 - dx) * dy * v10
        + dx * dy * v11;
}

cv::Point2f tangentAt(
    const std::vector<cv::Point2f>& priorPoints,
    std::size_t index)
{
    if (priorPoints.size() < 2)
    {
        return cv::Point2f(1.0f, 0.0f);
    }

    const std::size_t previous = index == 0 ? index : index - 1;
    const std::size_t next =
        std::min(index + 1, priorPoints.size() - 1);
    cv::Point2f tangent = priorPoints[next] - priorPoints[previous];
    const double norm = cv::norm(tangent);
    if (norm < 1e-6)
    {
        return cv::Point2f(1.0f, 0.0f);
    }
    return tangent * static_cast<float>(1.0 / norm);
}

double percentile95(std::vector<double> values)
{
    if (values.empty())
    {
        return 1.0;
    }

    std::sort(values.begin(), values.end());
    std::size_t index = static_cast<std::size_t>(values.size() * 0.95);
    if (index >= values.size())
    {
        index = values.size() - 1;
    }
    return std::max(1e-6, values[index]);
}

SubpixelEdgeSet robustLineFilter(
    const std::vector<SubpixelEdgeCandidate>& candidates,
    double inlierThresholdPx)
{
    SubpixelEdgeSet filtered;
    std::vector<PixelPoint> validPoints;
    std::vector<double> validConfidences;
    for (const SubpixelEdgeCandidate& candidate : candidates)
    {
        if (candidate.valid
            && std::isfinite(candidate.point.x)
            && std::isfinite(candidate.point.y))
        {
            validPoints.push_back({ candidate.point.x, candidate.point.y });
            validConfidences.push_back(candidate.confidence);
        }
    }

    if (validPoints.size() < 2)
    {
        return filtered;
    }

    std::vector<PixelPoint> inliers;
    std::vector<double> inlierConfidences;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < validPoints.size(); ++i)
    {
        for (std::size_t j = i + 1; j < validPoints.size(); ++j)
        {
            const std::vector<PixelPoint> sample = { validPoints[i], validPoints[j] };
            const IbvsLineModel line = IbvsStartPointDetector::fitLineTls(sample);
            if (!line.valid)
            {
                continue;
            }

            std::vector<PixelPoint> currentInliers;
            std::vector<double> currentConfidences;
            double confidenceSum = 0.0;
            double sumSquared = 0.0;
            for (std::size_t k = 0; k < validPoints.size(); ++k)
            {
                const double distance = std::abs(
                    line.a * validPoints[k].u
                    + line.b * validPoints[k].v
                    + line.c);
                if (distance <= inlierThresholdPx)
                {
                    currentInliers.push_back(validPoints[k]);
                    currentConfidences.push_back(validConfidences[k]);
                    confidenceSum += validConfidences[k];
                    sumSquared += distance * distance;
                }
            }

            if (currentInliers.empty())
            {
                continue;
            }
            const double rmse =
                std::sqrt(sumSquared / static_cast<double>(currentInliers.size()));
            const double score =
                static_cast<double>(currentInliers.size())
                + 0.5 * confidenceSum
                - 0.2 * rmse;
            if (score > bestScore)
            {
                bestScore = score;
                inliers = currentInliers;
                inlierConfidences = currentConfidences;
            }
        }
    }

    IbvsLineModel line;
    if (inliers.size() >= 2)
    {
        line = IbvsStartPointDetector::fitLineTls(inliers);
    }
    else
    {
        inliers = validPoints;
        inlierConfidences = validConfidences;
        line = IbvsStartPointDetector::fitLineTls(inliers);
    }
    if (!line.valid)
    {
        return filtered;
    }

    double sumSquared = 0.0;
    double confidenceSum = 0.0;
    for (std::size_t i = 0; i < inliers.size(); ++i)
    {
        const double distance =
            line.a * inliers[i].u + line.b * inliers[i].v + line.c;
        sumSquared += distance * distance;
        confidenceSum += inlierConfidences[i];
        filtered.points.emplace_back(
            static_cast<float>(inliers[i].u),
            static_cast<float>(inliers[i].v));
    }

    filtered.rmsePx = std::sqrt(sumSquared / static_cast<double>(inliers.size()));
    filtered.confidenceMean = confidenceSum / static_cast<double>(inliers.size());
    return filtered;
}

std::vector<cv::Point2f> extractDistanceFallbackPoints(
    const cv::Mat& distanceMap,
    const std::vector<cv::Point2f>& priorPoints,
    double halfWidthPx,
    double maxEdgeDistancePx)
{
    std::vector<cv::Point2f> extracted;
    if (priorPoints.size() < 2)
    {
        return extracted;
    }

    for (std::size_t i = 0; i < priorPoints.size(); i += 2)
    {
        const cv::Point2f previous = i == 0 ? priorPoints[i] : priorPoints[i - 1];
        const cv::Point2f next =
            (i + 1 < priorPoints.size()) ? priorPoints[i + 1] : priorPoints[i];
        cv::Point2f tangent = next - previous;
        const double tangentNorm = cv::norm(tangent);
        if (tangentNorm < 1e-6)
        {
            continue;
        }
        tangent *= static_cast<float>(1.0 / tangentNorm);
        const cv::Point2f normal(-tangent.y, tangent.x);

        double bestDistance = std::numeric_limits<double>::infinity();
        cv::Point2f bestPoint;
        for (double offset = -halfWidthPx; offset <= halfWidthPx + 1e-9; offset += 1.0)
        {
            const cv::Point2f candidate = priorPoints[i]
                + normal * static_cast<float>(offset);
            const double distance = sampleDistance(distanceMap, candidate);
            if (std::isfinite(distance) && distance < bestDistance)
            {
                bestDistance = distance;
                bestPoint = candidate;
            }
        }

        if (bestDistance <= maxEdgeDistancePx)
        {
            extracted.push_back(bestPoint);
        }
    }
    return extracted;
}

std::vector<cv::Point2f> extractSubpixelEdgePoints(
    const cv::Mat& bgrImage,
    const cv::Mat& distanceMap,
    const std::vector<cv::Point2f>& priorPoints,
    double halfWidthPx,
    double maxEdgeDistancePx,
    bool preserveCurve)
{
    if (priorPoints.size() < 2 || bgrImage.empty())
    {
        return {};
    }

    cv::Mat gray;
    if (bgrImage.channels() == 3)
    {
        cv::cvtColor(bgrImage, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = bgrImage.clone();
    }

    cv::Mat smooth;
    cv::GaussianBlur(gray, smooth, cv::Size(), 1.0);

    cv::Mat ix;
    cv::Mat iy;
    cv::Sobel(smooth, ix, CV_64F, 1, 0, 3);
    cv::Sobel(smooth, iy, CV_64F, 0, 1, 3);

    const double deltaRange = std::min(10.0, std::max(3.0, halfWidthPx));
    const double deltaStep = 1.0;
    const double ridgeShoulderDistance = 2.0;
    const int peakSearchRadius = 3;
    const int stride = 2;

    std::vector<SubpixelEdgeCandidate> stepCandidates;
    std::vector<SubpixelEdgeCandidate> ridgeCandidates;
    std::vector<double> stepPeaks;
    std::vector<double> stepSharpnesses;
    std::vector<double> ridgeCurvatures;

    for (std::size_t i = 0; i < priorPoints.size(); i += stride)
    {
        const cv::Point2f center = priorPoints[i];
        const cv::Point2f tangent = tangentAt(priorPoints, i);
        const cv::Point2f normal(-tangent.y, tangent.x);

        double bestStepScore = -1.0;
        double bestStepDelta = 0.0;
        double bestStepSharpness = 0.0;
        for (double delta = -deltaRange; delta <= deltaRange + 1e-9; delta += deltaStep)
        {
            const double x = center.x + delta * normal.x;
            const double y = center.y + delta * normal.y;
            const double gn =
                bilinearSample64(ix, x, y) * normal.x
                + bilinearSample64(iy, x, y) * normal.y;
            const double score = std::abs(gn);
            if (score > bestStepScore)
            {
                double left = 0.0;
                double right = 0.0;
                if (std::abs(delta) + deltaStep <= deltaRange)
                {
                    const double lx = center.x + (delta - deltaStep) * normal.x;
                    const double ly = center.y + (delta - deltaStep) * normal.y;
                    const double rx = center.x + (delta + deltaStep) * normal.x;
                    const double ry = center.y + (delta + deltaStep) * normal.y;
                    left = std::abs(
                        bilinearSample64(ix, lx, ly) * normal.x
                        + bilinearSample64(iy, lx, ly) * normal.y);
                    right = std::abs(
                        bilinearSample64(ix, rx, ry) * normal.x
                        + bilinearSample64(iy, rx, ry) * normal.y);
                }
                bestStepScore = score;
                bestStepDelta = delta;
                bestStepSharpness = std::max(0.0, 2.0 * score - left - right);
            }
        }

        SubpixelEdgeCandidate stepCandidate;
        stepCandidate.point = cv::Point2f(
            static_cast<float>(center.x + bestStepDelta * normal.x),
            static_cast<float>(center.y + bestStepDelta * normal.y));
        stepCandidate.valid = bestStepScore > 0.0;
        stepCandidate.confidence = bestStepScore;
        stepCandidates.push_back(stepCandidate);
        stepPeaks.push_back(bestStepScore);
        stepSharpnesses.push_back(bestStepSharpness);

        SubpixelEdgeCandidate ridgeCandidate;
        double bestRidgeCurvature = -1.0;
        double bestRidgeDelta = 0.0;
        int zeroCrossingCount = 0;
        double previousGn = 0.0;
        bool hasPrevious = false;
        for (double delta = -peakSearchRadius;
             delta <= peakSearchRadius + 1e-9;
             delta += deltaStep)
        {
            const double x = center.x + delta * normal.x;
            const double y = center.y + delta * normal.y;
            const double gn =
                bilinearSample64(ix, x, y) * normal.x
                + bilinearSample64(iy, x, y) * normal.y;
            if (hasPrevious && previousGn * gn < 0.0)
            {
                ++zeroCrossingCount;
                const double xPlus = x + ridgeShoulderDistance * normal.x;
                const double yPlus = y + ridgeShoulderDistance * normal.y;
                const double xMinus = x - ridgeShoulderDistance * normal.x;
                const double yMinus = y - ridgeShoulderDistance * normal.y;
                const double shoulder =
                    std::abs(
                        bilinearSample64(ix, xPlus, yPlus) * normal.x
                        + bilinearSample64(iy, xPlus, yPlus) * normal.y)
                    + std::abs(
                        bilinearSample64(ix, xMinus, yMinus) * normal.x
                        + bilinearSample64(iy, xMinus, yMinus) * normal.y);
                if (shoulder > bestRidgeCurvature)
                {
                    bestRidgeCurvature = shoulder;
                    bestRidgeDelta = delta - 0.5 * deltaStep;
                }
            }
            previousGn = gn;
            hasPrevious = true;
        }

        ridgeCandidate.point = cv::Point2f(
            static_cast<float>(center.x + bestRidgeDelta * normal.x),
            static_cast<float>(center.y + bestRidgeDelta * normal.y));
        ridgeCandidate.valid = bestRidgeCurvature > 0.0;
        ridgeCandidate.confidence =
            ridgeCandidate.valid
                ? bestRidgeCurvature / static_cast<double>(std::max(1, zeroCrossingCount))
                : 0.0;
        ridgeCandidates.push_back(ridgeCandidate);
        ridgeCurvatures.push_back(std::max(0.0, bestRidgeCurvature));
    }

    const double p95StepPeak = percentile95(stepPeaks);
    const double p95StepSharpness = percentile95(stepSharpnesses);
    const double p95RidgeCurvature = percentile95(ridgeCurvatures);

    for (std::size_t i = 0; i < stepCandidates.size(); ++i)
    {
        const double peakScore = std::min(1.0, stepPeaks[i] / p95StepPeak);
        const double sharpnessScore =
            std::min(1.0, stepSharpnesses[i] / p95StepSharpness);
        stepCandidates[i].confidence = 0.65 * peakScore + 0.35 * sharpnessScore;
        stepCandidates[i].valid =
            stepCandidates[i].valid && stepCandidates[i].confidence >= 0.15;
    }
    for (std::size_t i = 0; i < ridgeCandidates.size(); ++i)
    {
        ridgeCandidates[i].confidence =
            std::min(1.0, ridgeCurvatures[i] / p95RidgeCurvature);
        ridgeCandidates[i].valid =
            ridgeCandidates[i].valid && ridgeCandidates[i].confidence >= 0.15;
    }

    if (preserveCurve)
    {
        std::vector<cv::Point2f> selectedPoints;
        selectedPoints.reserve(stepCandidates.size());
        for (std::size_t i = 0; i < stepCandidates.size(); ++i)
        {
            const SubpixelEdgeCandidate& step = stepCandidates[i];
            const SubpixelEdgeCandidate& ridge = ridgeCandidates[i];
            const SubpixelEdgeCandidate& selected =
                ridge.valid && ridge.confidence > step.confidence ? ridge : step;
            if (selected.valid)
            {
                selectedPoints.push_back(selected.point);
            }
        }
        if (selectedPoints.size() >= 2)
        {
            return selectedPoints;
        }
        return extractDistanceFallbackPoints(
            distanceMap,
            priorPoints,
            halfWidthPx,
            maxEdgeDistancePx);
    }

    const double inlierThreshold = std::max(0.75, maxEdgeDistancePx);
    const SubpixelEdgeSet stepSet =
        robustLineFilter(stepCandidates, inlierThreshold);
    const SubpixelEdgeSet ridgeSet =
        robustLineFilter(ridgeCandidates, inlierThreshold);

    auto scoreSet = [](const SubpixelEdgeSet& set) -> double
    {
        if (set.points.size() < 2)
        {
            return -std::numeric_limits<double>::infinity();
        }
        return set.confidenceMean
            + 0.02 * static_cast<double>(set.points.size())
            - 0.08 * set.rmsePx;
    };

    const SubpixelEdgeSet& selected =
        scoreSet(ridgeSet) > scoreSet(stepSet) ? ridgeSet : stepSet;
    if (selected.points.size() >= 2)
    {
        return selected.points;
    }

    return extractDistanceFallbackPoints(
        distanceMap,
        priorPoints,
        halfWidthPx,
        maxEdgeDistancePx);
}

std::vector<cv::Point2f> makeCurveExtractArc(const std::vector<cv::Point2f>& priorPoints)
{
    if (priorPoints.size() < 5)
    {
        return priorPoints;
    }

    cv::RotatedRect ellipse;
    try
    {
        ellipse = cv::fitEllipse(priorPoints);
    }
    catch (const cv::Exception&)
    {
        return priorPoints;
    }

    cv::Point2f center = ellipse.center;
    float major = ellipse.size.width / 2.0f;
    float minor = ellipse.size.height / 2.0f;
    float angleRad = ellipse.angle * static_cast<float>(DEG_TO_RAD);
    if (major < minor)
    {
        std::swap(major, minor);
        angleRad += static_cast<float>(CV_PI / 2.0);
    }
    if (major <= 0.0f || minor <= 0.0f)
    {
        return priorPoints;
    }

    auto ellipsePoint = [&](float theta) -> cv::Point2f
    {
        const float cosPhi = std::cos(angleRad);
        const float sinPhi = std::sin(angleRad);
        const float xLocal = major * std::cos(theta);
        const float yLocal = minor * std::sin(theta);
        return cv::Point2f(
            center.x + xLocal * cosPhi - yLocal * sinPhi,
            center.y + xLocal * sinPhi + yLocal * cosPhi);
    };

    const cv::Point2f m1 = ellipsePoint(static_cast<float>(CV_PI / 2.0));
    const cv::Point2f m2 = ellipsePoint(static_cast<float>(3.0 * CV_PI / 2.0));
    const float thetaA = m1.y > m2.y ? 0.0f : static_cast<float>(CV_PI);
    const float thetaB =
        m1.y > m2.y ? static_cast<float>(CV_PI) : static_cast<float>(2.0 * CV_PI);
    const float thetaStart = thetaA;
    const float thetaEnd = thetaA + 0.85f * (thetaB - thetaA);
    const int sampleCount =
        std::max(24, static_cast<int>(std::ceil(priorPoints.size() * 0.85)));

    std::vector<cv::Point2f> arc;
    arc.reserve(static_cast<std::size_t>(sampleCount));
    for (int i = 0; i < sampleCount; ++i)
    {
        const float t = sampleCount == 1 ? 0.0f
            : static_cast<float>(i) / static_cast<float>(sampleCount - 1);
        arc.push_back(ellipsePoint(thetaStart + t * (thetaEnd - thetaStart)));
    }
    return arc;
}

double lineFitRmse(
    const std::vector<PixelPoint>& points,
    const IbvsLineModel& line)
{
    if (points.empty() || !line.valid)
    {
        return std::numeric_limits<double>::infinity();
    }

    double sumSquared = 0.0;
    for (const PixelPoint& point : points)
    {
        const double distance = line.a * point.u + line.b * point.v + line.c;
        sumSquared += distance * distance;
    }
    return std::sqrt(sumSquared / static_cast<double>(points.size()));
}

PixelPoint curveFixedArcStartPoint(
    const std::vector<PixelPoint>& points,
    double fraction)
{
    if (points.empty())
    {
        return {};
    }

    const double clamped = std::max(0.0, std::min(1.0, fraction));
    const std::size_t index = static_cast<std::size_t>(
        std::lround(clamped * static_cast<double>(points.size() - 1)));
    return points[index];
}

cv::Mat drawWeldOverlay(
    const cv::Mat& bgrImage,
    const std::vector<PixelPoint>& weldPoints,
    const std::vector<PixelPoint>& secondEdgePoints,
    const PixelPoint* startPoint,
    const PixelPoint* endPoint)
{
    cv::Mat visual = bgrImage.clone();
    for (const PixelPoint& point : weldPoints)
    {
        cv::circle(visual, toCvDrawPoint(point), 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }
    for (const PixelPoint& point : secondEdgePoints)
    {
        cv::circle(visual, toCvDrawPoint(point), 2, cv::Scalar(255, 0, 0), cv::FILLED);
    }
    if (startPoint != nullptr && startPoint->isFinite())
    {
        cv::circle(visual, toCvDrawPoint(*startPoint), 7, cv::Scalar(0, 255, 0), 2);
        cv::putText(
            visual,
            "Start",
            toCvDrawPoint(*startPoint) + cv::Point(10, -10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA);
    }
    if (endPoint != nullptr && endPoint->isFinite())
    {
        cv::circle(visual, toCvDrawPoint(*endPoint), 6, cv::Scalar(0, 255, 255), 2);
    }
    return visual;
}

bool isPointInImage(const PixelPoint& point, const cv::Size& imageSize)
{
    return point.isFinite()
        && point.u >= 0.0
        && point.v >= 0.0
        && point.u < imageSize.width
        && point.v < imageSize.height;
}

} // namespace

IbvsStartPointImageResult IbvsStartPointImageDetector::detect(
    const cv::Mat& bgrImage,
    const std::vector<std::vector<cv::Point3f>>& curveTemplateEdges,
    const std::vector<std::vector<cv::Point3f>>& weldSampleEdges,
    const IbvsStartPointImageDetectorConfig& config) const
{
    IbvsStartPointImageResult result;
    if (bgrImage.empty())
    {
        makeFailure(result, IbvsStatus::EMPTY_INPUT, "start point image is empty");
        return result;
    }
    if (curveTemplateEdges.empty())
    {
        makeFailure(result, IbvsStatus::EMPTY_INPUT, "curve template edges are empty");
        return result;
    }
    if (weldSampleEdges.empty())
    {
        makeFailure(result, IbvsStatus::EMPTY_INPUT, "weld sample edges are empty");
        return result;
    }
    if (!config.intrinsics.isValid())
    {
        makeFailure(result, IbvsStatus::INVALID_ARGUMENT, "camera intrinsics are invalid");
        return result;
    }

    const cv::Mat cameraMatrix =
        IbvsImageProjectionTools::makeCameraMatrix(config.intrinsics);
    const auto curveInitialEdges =
        IbvsImageProjectionTools::projectEdges(
            curveTemplateEdges,
            config.rvec0,
            config.tvec0,
            cameraMatrix);
    const auto curveInitialFlat =
        IbvsImageProjectionTools::flattenEdges(curveInitialEdges);
    result.curveInitialTemplatePoints =
        IbvsImageProjectionTools::toPixelPoints(curveInitialFlat);
    result.curveInitialTemplateOverlay =
        drawTemplateOverlay(bgrImage, curveInitialEdges, cv::Scalar(255, 255, 0));

    result.edges = extractEdges(bgrImage);
    result.edgeExtractionSucceeded = cv::countNonZero(result.edges) > 0;
    result.contoursOverlay = drawContoursOverlay(bgrImage, result.edges);
    if (!result.edgeExtractionSucceeded)
    {
        makeFailure(result, IbvsStatus::RUNTIME_ERROR, "edge extraction produced no edge pixels");
        return result;
    }

    const IbvsTemplateAlignmentResult alignment =
        IbvsTemplateAlignmentTools::alignTemplateToEdges(
            curveTemplateEdges,
            result.edges,
            cameraMatrix,
            config.rvec0,
            config.tvec0,
            config.alignment);
    result.bestDeltaXMm = alignment.best.dxMm;
    result.bestDeltaYMm = alignment.best.dyMm;
    result.bestDeltaPsiDeg = alignment.best.dpsiDeg;
    result.templateCostPx = alignment.best.cost;
    result.secondBestTemplateCostPx = alignment.secondBest.cost;
    result.templateCostMarginPx = alignment.secondBest.cost - alignment.best.cost;
    result.templateInlierRatio = alignment.inlierRatio;
    result.rvecOpt = alignment.rvecOpt;
    result.tvecOpt = alignment.tvecOpt;

    const auto curveAlignedEdges =
        IbvsImageProjectionTools::projectEdges(
            curveTemplateEdges,
            alignment.rvecOpt,
            alignment.tvecOpt,
            cameraMatrix);
    const auto curveAlignedFlat =
        IbvsImageProjectionTools::flattenEdges(curveAlignedEdges);
    result.curveAlignedTemplatePoints =
        IbvsImageProjectionTools::toPixelPoints(curveAlignedFlat);
    result.curveAlignedTemplateOverlay =
        drawTemplateOverlay(bgrImage, curveAlignedEdges, cv::Scalar(0, 255, 255));

    const auto insideCount = std::count_if(
        curveAlignedFlat.begin(),
        curveAlignedFlat.end(),
        [&bgrImage](const cv::Point2f& point)
        {
            return ibvsIsInsideImage(point, bgrImage.size());
        });
    result.templateAlignmentSucceeded =
        std::isfinite(result.templateCostPx)
        && result.templateCostPx < config.alignment.chamferTauPx
        && result.templateInlierRatio > 0.3
        && insideCount >= 2;
    if (!result.templateAlignmentSucceeded)
    {
        makeFailure(
            result,
            IbvsStatus::DEGENERATE_GEOMETRY,
            "curve template alignment failed quality checks");
        return result;
    }

    const auto weldPriorEdges =
        IbvsImageProjectionTools::projectEdges(
            weldSampleEdges,
            result.rvecOpt,
            result.tvecOpt,
            cameraMatrix);
    const std::vector<cv::Point2f> weldPrior =
        weldPriorEdges.empty() ? std::vector<cv::Point2f>() : weldPriorEdges.front();
    const std::vector<cv::Point2f> secondEdgePrior =
        weldPriorEdges.size() >= 2 ? weldPriorEdges[1] : std::vector<cv::Point2f>();
    result.weldPriorPoints = IbvsImageProjectionTools::toPixelPoints(weldPrior);
    result.secondEdgePriorPoints =
        IbvsImageProjectionTools::toPixelPoints(secondEdgePrior);
    result.weldPriorOverlay =
        drawWeldOverlay(bgrImage, result.weldPriorPoints, result.secondEdgePriorPoints, nullptr, nullptr);

    const cv::Mat distanceMap = makeDistanceMap(result.edges);
    const double searchHalfWidth =
        config.shape == IbvsStartPointWorkpieceShape::Curve
            ? config.curveEdgeSearchHalfWidthPx
            : config.straightEdgeSearchHalfWidthPx;
    const std::vector<cv::Point2f> extractionPrior =
        config.shape == IbvsStartPointWorkpieceShape::Curve
            ? makeCurveExtractArc(weldPrior)
            : weldPrior;
    const std::vector<cv::Point2f> weldCvPoints = extractSubpixelEdgePoints(
        bgrImage,
        distanceMap,
        extractionPrior,
        searchHalfWidth,
        config.maxEdgeDistancePx,
        config.shape == IbvsStartPointWorkpieceShape::Curve);
    result.weldPoints = IbvsImageProjectionTools::toPixelPoints(weldCvPoints);

    if (result.weldPoints.size() < static_cast<std::size_t>(config.minWeldPointCount))
    {
        result.weldCandidatesOverlay =
            drawWeldOverlay(bgrImage, result.weldPoints, result.secondEdgePoints, nullptr, nullptr);
        result.weldSubpixelOverlay = result.weldCandidatesOverlay.clone();
        makeFailure(result, IbvsStatus::NOT_ENOUGH_SAMPLES, "not enough weld edge points extracted");
        return result;
    }

    result.weldExtractionSucceeded = true;
    result.weldLine = IbvsStartPointDetector::fitLineTls(result.weldPoints);
    result.weldFitErrorPx = lineFitRmse(result.weldPoints, result.weldLine);

    if (config.shape == IbvsStartPointWorkpieceShape::Straight)
    {
        const std::vector<cv::Point2f> secondCvPoints = extractSubpixelEdgePoints(
            bgrImage,
            distanceMap,
            secondEdgePrior,
            searchHalfWidth,
            config.maxEdgeDistancePx,
            false);
        result.secondEdgePoints =
            IbvsImageProjectionTools::toPixelPoints(secondCvPoints);
        result.secondEdgeLine =
            IbvsStartPointDetector::fitLineTls(result.secondEdgePoints);

        if (config.strategy == IbvsStartPointStrategy::Intersection
            && result.secondEdgeLine.valid)
        {
            const auto start = IbvsStartPointDetector::computeIntersectionStartPoint(
                result.weldLine,
                result.secondEdgeLine);
            if (start.error.isOk() && isPointInImage(start.startPoint, bgrImage.size()))
            {
                result.startPoint = start.startPoint;
                result.endPoint = start.endPoint;
                result.startPointSucceeded = true;
            }
        }

        if (!result.startPointSucceeded)
        {
            const auto endpoint =
                IbvsStartPointDetector::computeEndpointStartPoint(result.weldPoints, true);
            if (endpoint.error.isOk())
            {
                result.startPoint = endpoint.startPoint;
                result.endPoint = endpoint.endPoint;
                result.startPointSucceeded =
                    isPointInImage(result.startPoint, bgrImage.size());
                if (config.strategy == IbvsStartPointStrategy::Intersection)
                {
                    result.failureReason = "intersection unavailable; endpoint candidate used";
                }
            }
        }
    }
    else
    {
        result.startPoint =
            curveFixedArcStartPoint(result.weldPoints, config.curveStartArcFraction);
        result.endPoint = result.startPoint;
        result.startPointSucceeded =
            isPointInImage(result.startPoint, bgrImage.size());
    }

    result.weldCandidatesOverlay =
        drawWeldOverlay(bgrImage, result.weldPriorPoints, result.secondEdgePriorPoints, nullptr, nullptr);
    result.weldSubpixelOverlay =
        drawWeldOverlay(bgrImage, result.weldPoints, result.secondEdgePoints, nullptr, nullptr);
    result.startPointOverlay = drawWeldOverlay(
        bgrImage,
        result.weldPoints,
        result.secondEdgePoints,
        &result.startPoint,
        &result.endPoint);

    if (!result.startPointSucceeded)
    {
        makeFailure(result, IbvsStatus::RUNTIME_ERROR, "start point calculation failed");
        return result;
    }

    result.success = result.edgeExtractionSucceeded
        && result.templateAlignmentSucceeded
        && result.weldExtractionSucceeded
        && result.startPointSucceeded;
    result.error = { IbvsStatus::OK, std::string() };
    if (result.failureReason.empty())
    {
        result.failureReason = "ok";
    }
    return result;
}

} // namespace i2r_rlc::ibvs
