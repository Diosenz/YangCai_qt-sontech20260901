/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsColorBlobTools.cpp
 * @brief     IBVS 内部颜色连通域特征提取工具实现。
 * @details   本文件实现 HSV 颜色掩膜、形态学预处理、连通域候选提取和评分。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "IbvsColorBlobTools.h"

namespace i2r_rlc::ibvs {

namespace {

double clamp01(double value)
{
    if (value < 0.0)
    {
        return 0.0;
    }
    if (value > 1.0)
    {
        return 1.0;
    }
    return value;
}

double blobScore(
    const IbvsColorBlobExtractionConfig& config,
    double area,
    double aspectRatio,
    double circularity,
    double meanSaturation,
    double meanValue)
{
    const double areaScore = clamp01(area / 40.0);
    const double aspectScore = clamp01(
        (config.maxAspectRatio - aspectRatio)
        / std::max(1e-6, config.maxAspectRatio - 1.0));
    const double circularityScore = clamp01(
        (circularity - config.minCircularity)
        / std::max(1e-6, 1.0 - config.minCircularity));
    const double saturationScore = clamp01(
        (meanSaturation - config.minMeanSaturation)
        / std::max(1e-6, 255.0 - config.minMeanSaturation));
    const double valueScore = clamp01(
        (meanValue - config.minMeanValue)
        / std::max(1e-6, 255.0 - config.minMeanValue));
    return 0.20 * areaScore
        + 0.20 * aspectScore
        + 0.25 * circularityScore
        + 0.20 * saturationScore
        + 0.15 * valueScore;
}

} // namespace

cv::Mat IbvsColorBlobTools::applyGammaCorrection(
    const cv::Mat& image,
    double gamma)
{
    cv::Mat imageFloat;
    cv::Mat imageGamma;
    image.convertTo(imageFloat, CV_32F, 1.0 / 255.0);
    cv::pow(imageFloat, gamma, imageFloat);
    imageFloat *= 255.0;
    imageFloat.convertTo(imageGamma, CV_8UC3);
    return imageGamma;
}

cv::Mat IbvsColorBlobTools::extractColorMask(
    const cv::Mat& bgrImage,
    const IbvsColorBlobExtractionConfig& config,
    cv::Mat* hsvImage)
{
    cv::Mat image = bgrImage.clone();
    if (config.applyGamma)
    {
        image = applyGammaCorrection(image, config.gamma);
    }
    if (config.applyBilateral)
    {
        cv::Mat smoothed;
        cv::bilateralFilter(image, smoothed, 9, 75, 75);
        image = smoothed;
    }

    cv::Mat hsvLocal;
    cv::cvtColor(image, hsvLocal, cv::COLOR_BGR2HSV);
    if (hsvImage != nullptr)
    {
        *hsvImage = hsvLocal.clone();
    }

    cv::Mat colorMask = cv::Mat::zeros(hsvLocal.size(), CV_8UC1);
    for (const IbvsColorRange& range : config.hsvRanges)
    {
        cv::Mat rangeMask;
        cv::inRange(hsvLocal, range.lower, range.upper, rangeMask);
        cv::bitwise_or(colorMask, rangeMask, colorMask);
    }

    const int kernelSize = std::max(1, config.morphKernelSize);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    return colorMask;
}

std::vector<IbvsColorBlobCandidate> IbvsColorBlobTools::extractCandidates(
    const cv::Mat& mask,
    const cv::Mat& hsvImage,
    const IbvsColorBlobExtractionConfig& config)
{
    std::vector<IbvsColorBlobCandidate> candidates;
    if (mask.empty() || hsvImage.empty())
    {
        return candidates;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount =
        cv::connectedComponentsWithStats(mask, labels, stats, centroids);
    candidates.reserve(static_cast<std::size_t>(std::max(0, labelCount - 1)));

    for (int label = 1; label < labelCount; ++label)
    {
        const double area = static_cast<double>(
            stats.at<int>(label, cv::CC_STAT_AREA));
        if (area < config.minBlobAreaPx)
        {
            continue;
        }

        const cv::Mat componentMask = labels == label;
        std::vector<cv::Point> points;
        cv::findNonZero(componentMask, points);
        if (points.size() < 3)
        {
            continue;
        }

        const cv::RotatedRect rotatedRect = cv::minAreaRect(points);
        const double width = rotatedRect.size.width;
        const double height = rotatedRect.size.height;
        if (width <= 1e-6 || height <= 1e-6)
        {
            continue;
        }
        if (width > config.maxBlobSizePx || height > config.maxBlobSizePx)
        {
            continue;
        }

        const double aspectRatio =
            std::max(width, height) / std::max(1e-6, std::min(width, height));
        if (aspectRatio > config.maxAspectRatio)
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(
            componentMask.clone(),
            contours,
            cv::RETR_EXTERNAL,
            cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            continue;
        }

        const auto bestContour = std::max_element(
            contours.begin(),
            contours.end(),
            [](const std::vector<cv::Point>& left,
               const std::vector<cv::Point>& right)
            {
                return std::abs(cv::contourArea(left))
                    < std::abs(cv::contourArea(right));
            });
        const double contourArea = std::abs(cv::contourArea(*bestContour));
        const double perimeter = cv::arcLength(*bestContour, true);
        const double circularity = perimeter > 1e-6
            ? 4.0 * CV_PI * contourArea / (perimeter * perimeter)
            : 0.0;
        if (circularity < config.minCircularity)
        {
            continue;
        }

        const cv::Scalar hsvMean = cv::mean(hsvImage, componentMask);
        const double meanSaturation = hsvMean[1];
        const double meanValue = hsvMean[2];
        if (meanSaturation < config.minMeanSaturation
            || meanValue < config.minMeanValue)
        {
            continue;
        }

        IbvsColorBlobCandidate candidate;
        candidate.centroid = {
            centroids.at<double>(label, 0),
            centroids.at<double>(label, 1)
        };
        candidate.areaPx = area;
        candidate.aspectRatio = aspectRatio;
        candidate.circularity = circularity;
        candidate.meanSaturation = meanSaturation;
        candidate.meanValue = meanValue;
        candidate.score = blobScore(
            config,
            area,
            aspectRatio,
            circularity,
            meanSaturation,
            meanValue);
        candidates.push_back(candidate);
    }

    return candidates;
}

} // namespace i2r_rlc::ibvs
