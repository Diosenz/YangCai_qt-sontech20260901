/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsRedFeatureDetector.cpp
 * @brief     IBVS 焊枪红色双特征点检测实现。
 * @details   本文件实现红色连通域候选转换、候选筛选和最佳双点匹配。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>

#include "ibvs/IbvsRedFeatureDetector.h"
#include "IbvsColorBlobTools.h"

namespace i2r_rlc::ibvs {

namespace {

double pointDistance(const PixelPoint& first, const PixelPoint& second)
{
    const double du = first.u - second.u;
    const double dv = first.v - second.v;
    return std::sqrt(du * du + dv * dv);
}

IbvsColorBlobExtractionConfig makeRedBlobConfig(
    const IbvsRedFeatureDetectorConfig& config)
{
    IbvsColorBlobExtractionConfig blobConfig;
    blobConfig.applyGamma = config.applyGamma;
    blobConfig.applyBilateral = config.applyBilateral;
    blobConfig.gamma = config.gamma;
    blobConfig.morphKernelSize = config.morphKernelSize;
    blobConfig.minBlobAreaPx = config.minBlobAreaPx;
    blobConfig.maxBlobSizePx = config.maxBlobSizePx;
    blobConfig.maxAspectRatio = config.maxAspectRatio;
    blobConfig.minCircularity = config.minCircularity;
    blobConfig.minMeanSaturation = config.minMeanSaturation;
    blobConfig.minMeanValue = config.minMeanValue;
    blobConfig.hsvRanges = {
        { cv::Scalar(0, 30, 10), cv::Scalar(15, 255, 255) },
        { cv::Scalar(160, 30, 10), cv::Scalar(180, 255, 255) }
    };
    return blobConfig;
}

std::vector<IbvsRedBlobCandidate> toRedBlobCandidates(
    const std::vector<IbvsColorBlobCandidate>& candidates)
{
    std::vector<IbvsRedBlobCandidate> converted;
    converted.reserve(candidates.size());
    for (const IbvsColorBlobCandidate& candidate : candidates)
    {
        converted.push_back({
            candidate.centroid,
            candidate.areaPx,
            candidate.aspectRatio,
            candidate.circularity,
            candidate.meanSaturation,
            candidate.meanValue,
            candidate.score
        });
    }
    return converted;
}

} // namespace

IbvsRedFeatureDetector::IbvsRedFeatureDetector() = default;

IbvsRedFeatureDetector::IbvsRedFeatureDetector(
    const IbvsRedFeatureDetectorConfig& config)
    : m_config(config)
{
}

IbvsRedFeatureDetectionResult IbvsRedFeatureDetector::detect(
    const cv::Mat& bgrImage) const
{
    IbvsRedFeatureDetectionResult result;
    if (bgrImage.empty())
    {
        result.error = { IbvsStatus::EMPTY_INPUT, "red feature image is empty" };
        return result;
    }

    cv::Mat hsvImage;
    const IbvsColorBlobExtractionConfig blobConfig = makeRedBlobConfig(m_config);
    const cv::Mat mask =
        IbvsColorBlobTools::extractColorMask(bgrImage, blobConfig, &hsvImage);
    result = selectBestPair(
        toRedBlobCandidates(
            IbvsColorBlobTools::extractCandidates(mask, hsvImage, blobConfig)));
    return result;
}

IbvsRedFeatureDetectionResult IbvsRedFeatureDetector::selectBestPair(
    const std::vector<IbvsRedBlobCandidate>& candidates) const
{
    IbvsRedFeatureDetectionResult result;
    result.candidates = candidates;
    if (candidates.size() < 2)
    {
        result.error = { IbvsStatus::NOT_ENOUGH_SAMPLES, "no red blob pair found" };
        return result;
    }

    double bestScore = -1.0;
    PixelFeaturePair bestPair;
    double bestDistance = 0.0;
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
        for (std::size_t j = i + 1; j < candidates.size(); ++j)
        {
            PixelPoint left = candidates[i].centroid;
            PixelPoint right = candidates[j].centroid;
            if (left.u > right.u)
            {
                std::swap(left, right);
            }

            const double distance = pointDistance(left, right);
            if (distance < m_config.minPairDistancePx
                || distance > m_config.maxPairDistancePx)
            {
                continue;
            }

            const double pairScore =
                0.5 * (candidates[i].score + candidates[j].score);
            if (pairScore > bestScore)
            {
                bestScore = pairScore;
                bestPair = { left, right };
                bestDistance = distance;
            }
        }
    }

    if (bestScore < 0.0)
    {
        result.error = {
            IbvsStatus::DEGENERATE_GEOMETRY,
            "red blob candidates do not satisfy pair geometry"
        };
        return result;
    }

    result.error = { IbvsStatus::OK, std::string() };
    result.accepted = true;
    result.featurePoints = bestPair;
    result.distancePx = bestDistance;
    return result;
}

} // namespace i2r_rlc::ibvs
