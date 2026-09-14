/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsTemplateAlignmentTools.cpp
 * @brief     IBVS 内部模板投影对齐实现。
 * @details   本文件实现多层 chamfer 搜索、候选排序和自适应局部精搜。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

#include <opencv2/imgproc.hpp>

#ifdef I2R_RLC_IBVS_USE_OPENMP
#include <omp.h>
#endif

#include "IbvsImageProjectionTools.h"
#include "IbvsTemplateAlignmentTools.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double LARGE_COST = 1e9;

double chamferCost(
    const std::vector<cv::Point2f>& points,
    const cv::Mat& distanceMap,
    double tauPx,
    double bestCost)
{
    if (points.empty())
    {
        return LARGE_COST;
    }

    const bool useEarlyAbandon = bestCost >= 0.0;
    const double abandonThreshold =
        useEarlyAbandon ? bestCost * static_cast<double>(points.size())
                        : std::numeric_limits<double>::max();
    double totalCost = 0.0;
    int validCount = 0;
    const float tauF = static_cast<float>(tauPx);

    for (const cv::Point2f& point : points)
    {
        const int x = static_cast<int>(point.x + 0.5f);
        const int y = static_cast<int>(point.y + 0.5f);
        if (static_cast<unsigned>(x) < static_cast<unsigned>(distanceMap.cols)
            && static_cast<unsigned>(y) < static_cast<unsigned>(distanceMap.rows))
        {
            const float distance = distanceMap.ptr<float>(y)[x];
            totalCost += distance < tauF ? distance : tauF;
        }
        else
        {
            totalCost += tauF;
        }
        ++validCount;

        if (useEarlyAbandon && totalCost > abandonThreshold)
        {
            return LARGE_COST;
        }
    }

    return validCount > 0 ? totalCost / static_cast<double>(validCount)
                          : LARGE_COST;
}

double computeInlierRatio(
    const std::vector<cv::Point2f>& points,
    const cv::Mat& distanceMap,
    double epsilonPx)
{
    if (points.empty())
    {
        return 0.0;
    }

    int inlierCount = 0;
    const float epsilonF = static_cast<float>(epsilonPx);
    for (const cv::Point2f& point : points)
    {
        const int x = static_cast<int>(point.x + 0.5f);
        const int y = static_cast<int>(point.y + 0.5f);
        if (static_cast<unsigned>(x) < static_cast<unsigned>(distanceMap.cols)
            && static_cast<unsigned>(y) < static_cast<unsigned>(distanceMap.rows)
            && distanceMap.ptr<float>(y)[x] < epsilonF)
        {
            ++inlierCount;
        }
    }
    return static_cast<double>(inlierCount) / static_cast<double>(points.size());
}

void keepBestCandidates(std::vector<IbvsTemplateAlignmentCandidate>* candidates, int topN)
{
    if (candidates == nullptr)
    {
        return;
    }

    const auto costLess =
        [](const IbvsTemplateAlignmentCandidate& left,
           const IbvsTemplateAlignmentCandidate& right)
        {
            return left.cost < right.cost;
        };
    if (topN > 0 && candidates->size() > static_cast<std::size_t>(topN))
    {
        const auto nth =
            candidates->begin() + static_cast<std::ptrdiff_t>(topN);
        std::nth_element(candidates->begin(), nth, candidates->end(), costLess);
        candidates->resize(static_cast<std::size_t>(topN));
    }
    std::sort(candidates->begin(), candidates->end(), costLess);
}

double candidateStdDev(
    const std::vector<IbvsTemplateAlignmentCandidate>& candidates,
    double IbvsTemplateAlignmentCandidate::*member)
{
    if (candidates.empty())
    {
        return 0.0;
    }

    double mean = 0.0;
    for (const IbvsTemplateAlignmentCandidate& candidate : candidates)
    {
        mean += candidate.*member;
    }
    mean /= static_cast<double>(candidates.size());

    double variance = 0.0;
    for (const IbvsTemplateAlignmentCandidate& candidate : candidates)
    {
        const double diff = candidate.*member - mean;
        variance += diff * diff;
    }
    variance /= static_cast<double>(candidates.size());
    return std::sqrt(variance);
}

double adaptiveSearchRange(
    const std::vector<IbvsTemplateAlignmentCandidate>& candidates,
    double IbvsTemplateAlignmentCandidate::*member,
    double nominalRange,
    double minimumRange)
{
    if (candidates.size() < 2)
    {
        return nominalRange;
    }

    const double lowerBound = std::min(nominalRange, minimumRange);
    return std::max(
        lowerBound,
        std::min(3.0 * candidateStdDev(candidates, member), nominalRange));
}

std::int64_t quantizedKey(
    double dx,
    double dy,
    double dpsi,
    double stepX,
    double stepY,
    double stepPsi)
{
    const auto ix = static_cast<std::int64_t>(std::llround(dx / stepX)) + 20000;
    const auto iy = static_cast<std::int64_t>(std::llround(dy / stepY)) + 20000;
    const auto ipsi =
        static_cast<std::int64_t>(std::llround(dpsi / stepPsi)) + 20000;
    return (ix << 42) ^ (iy << 21) ^ ipsi;
}

IbvsTemplateAlignmentCandidate evaluateCandidate(
    const IbvsProjectedTemplateModel& templateModel,
    const cv::Mat& cameraMatrix,
    const cv::Vec3d& rvec0,
    const cv::Vec3d& tvec0,
    const cv::Mat& distanceMap,
    double scale,
    double sampleIntervalPx,
    double tauPx,
    double dxMm,
    double dyMm,
    double dpsiDeg,
    double bestCost)
{
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    IbvsImageProjectionTools::computePerturbedPose(
        rvec0,
        tvec0,
        dxMm,
        dyMm,
        dpsiDeg,
        &rvec,
        &tvec);

    IbvsTemplateAlignmentCandidate candidate;
    candidate.dxMm = dxMm;
    candidate.dyMm = dyMm;
    candidate.dpsiDeg = dpsiDeg;
    candidate.cost = chamferCost(
        IbvsImageProjectionTools::projectAndSampleEdges(
            templateModel,
            rvec,
            tvec,
            cameraMatrix,
            scale,
            sampleIntervalPx),
        distanceMap,
        tauPx,
        bestCost);
    return candidate;
}

std::vector<IbvsTemplateAlignmentCandidate> evaluateGrid(
    const IbvsProjectedTemplateModel& templateModel,
    const cv::Mat& cameraMatrix,
    const cv::Vec3d& rvec0,
    const cv::Vec3d& tvec0,
    const cv::Mat& distanceMap,
    double scale,
    double sampleIntervalPx,
    double tauPx,
    double dxMin,
    double dxMax,
    double dxStep,
    double dyMin,
    double dyMax,
    double dyStep,
    double psiMin,
    double psiMax,
    double psiStep,
    int topN)
{
    std::vector<IbvsTemplateAlignmentCandidate> candidates;
    const int countX = static_cast<int>((dxMax - dxMin) / dxStep) + 1;
    const int countY = static_cast<int>((dyMax - dyMin) / dyStep) + 1;
    const int countPsi = static_cast<int>((psiMax - psiMin) / psiStep) + 1;
    const int totalCount = countX * countY * countPsi;
    candidates.resize(static_cast<std::size_t>(totalCount));

#ifdef I2R_RLC_IBVS_USE_OPENMP
#pragma omp parallel for schedule(dynamic, 100)
#endif
    for (int index = 0; index < totalCount; ++index)
    {
        int temp = index;
        const int psiIndex = temp % countPsi;
        temp /= countPsi;
        const int yIndex = temp % countY;
        const int xIndex = temp / countY;

        const double dx = dxMin + static_cast<double>(xIndex) * dxStep;
        const double dy = dyMin + static_cast<double>(yIndex) * dyStep;
        const double dpsi = psiMin + static_cast<double>(psiIndex) * psiStep;
        candidates[static_cast<std::size_t>(index)] = evaluateCandidate(
            templateModel,
            cameraMatrix,
            rvec0,
            tvec0,
            distanceMap,
            scale,
            sampleIntervalPx,
            tauPx,
            dx,
            dy,
            dpsi,
            -1.0);
    }

    keepBestCandidates(&candidates, topN);
    return candidates;
}

std::vector<IbvsTemplateAlignmentCandidate> evaluateAroundSeeds(
    const std::vector<IbvsTemplateAlignmentCandidate>& seeds,
    int seedCount,
    const IbvsProjectedTemplateModel& templateModel,
    const cv::Mat& cameraMatrix,
    const cv::Vec3d& rvec0,
    const cv::Vec3d& tvec0,
    const cv::Mat& distanceMap,
    double scale,
    double sampleIntervalPx,
    double tauPx,
    double dxRange,
    double dxStep,
    double dyRange,
    double dyStep,
    double psiRange,
    double psiStep,
    int topN,
    bool useEarlyAbandon)
{
    struct CandidateParameters
    {
        double dxMm = 0.0;
        double dyMm = 0.0;
        double dpsiDeg = 0.0;
    };

    std::vector<CandidateParameters> parameters;
    std::unordered_set<std::int64_t> evaluated;
    const int seedsToUse =
        std::min(seedCount, static_cast<int>(seeds.size()));

    for (int seedIndex = 0; seedIndex < seedsToUse; ++seedIndex)
    {
        const IbvsTemplateAlignmentCandidate& seed = seeds[seedIndex];
        for (double dx = seed.dxMm - dxRange;
             dx <= seed.dxMm + dxRange + 1e-9;
             dx += dxStep)
        {
            for (double dy = seed.dyMm - dyRange;
                 dy <= seed.dyMm + dyRange + 1e-9;
                 dy += dyStep)
            {
                for (double dpsi = seed.dpsiDeg - psiRange;
                     dpsi <= seed.dpsiDeg + psiRange + 1e-9;
                     dpsi += psiStep)
                {
                    const std::int64_t key =
                        quantizedKey(dx, dy, dpsi, dxStep, dyStep, psiStep);
                    if (!evaluated.insert(key).second)
                    {
                        continue;
                    }

                    parameters.push_back({ dx, dy, dpsi });
                }
            }
        }
    }

    std::vector<IbvsTemplateAlignmentCandidate> candidates(parameters.size());
#ifdef I2R_RLC_IBVS_USE_OPENMP
    if (useEarlyAbandon)
#endif
    {
        double bestCost = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < parameters.size(); ++i)
        {
            const CandidateParameters& parameter = parameters[i];
            candidates[i] = evaluateCandidate(
                templateModel,
                cameraMatrix,
                rvec0,
                tvec0,
                distanceMap,
                scale,
                sampleIntervalPx,
                tauPx,
                parameter.dxMm,
                parameter.dyMm,
                parameter.dpsiDeg,
                useEarlyAbandon ? bestCost : -1.0);
            if (candidates[i].cost < bestCost)
            {
                bestCost = candidates[i].cost;
            }
        }
    }
#ifdef I2R_RLC_IBVS_USE_OPENMP
    else
    {
#pragma omp parallel for schedule(dynamic, 50)
        for (int i = 0; i < static_cast<int>(parameters.size()); ++i)
        {
            const CandidateParameters& parameter =
                parameters[static_cast<std::size_t>(i)];
            candidates[static_cast<std::size_t>(i)] = evaluateCandidate(
                templateModel,
                cameraMatrix,
                rvec0,
                tvec0,
                distanceMap,
                scale,
                sampleIntervalPx,
                tauPx,
                parameter.dxMm,
                parameter.dyMm,
                parameter.dpsiDeg,
                -1.0);
        }
    }
#endif

    keepBestCandidates(&candidates, topN);
    return candidates;
}

} // namespace

IbvsTemplateAlignmentResult IbvsTemplateAlignmentTools::alignTemplateToEdges(
    const std::vector<std::vector<cv::Point3f>>& templateEdges,
    const cv::Mat& edges,
    const cv::Mat& cameraMatrix,
    const cv::Vec3d& rvec0,
    const cv::Vec3d& tvec0,
    const IbvsTemplateAlignmentParameters& parameters)
{
    const IbvsProjectedTemplateModel templateModel =
        IbvsImageProjectionTools::makeProjectedTemplateModel(templateEdges);
    std::vector<cv::Mat> edgePyramid(3);
    std::vector<cv::Mat> distancePyramid(3);
    const std::vector<double> scaleFactors = { 0.25, 0.5, 1.0 };

    cv::resize(edges, edgePyramid[0], cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
    cv::resize(edges, edgePyramid[1], cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    edgePyramid[2] = edges.clone();

    for (int level = 0; level < 3; ++level)
    {
        cv::Mat inverted;
        cv::bitwise_not(edgePyramid[level], inverted);
        cv::distanceTransform(
            inverted,
            distancePyramid[level],
            cv::DIST_L2,
            cv::DIST_MASK_PRECISE);
    }

    std::vector<IbvsTemplateAlignmentCandidate> level0 = evaluateGrid(
        templateModel,
        cameraMatrix,
        rvec0,
        tvec0,
        distancePyramid[0],
        scaleFactors[0],
        parameters.templateSampleIntervalPx,
        parameters.chamferTauPx * scaleFactors[0],
        parameters.level0DeltaXMinMm,
        parameters.level0DeltaXMaxMm,
        parameters.level0DeltaXStepMm,
        parameters.level0DeltaYMinMm,
        parameters.level0DeltaYMaxMm,
        parameters.level0DeltaYStepMm,
        parameters.level0DeltaPsiMinDeg,
        parameters.level0DeltaPsiMaxDeg,
        parameters.level0DeltaPsiStepDeg,
        parameters.level0TopN);

    const double level1DeltaXRange = adaptiveSearchRange(
        level0,
        &IbvsTemplateAlignmentCandidate::dxMm,
        parameters.level1DeltaXRangeMm,
        parameters.level0DeltaXStepMm * 3.0);
    const double level1DeltaYRange = adaptiveSearchRange(
        level0,
        &IbvsTemplateAlignmentCandidate::dyMm,
        parameters.level1DeltaYRangeMm,
        parameters.level0DeltaYStepMm * 3.0);
    const double level1DeltaPsiRange = adaptiveSearchRange(
        level0,
        &IbvsTemplateAlignmentCandidate::dpsiDeg,
        parameters.level1DeltaPsiRangeDeg,
        parameters.level0DeltaPsiStepDeg * 3.0);

    std::vector<IbvsTemplateAlignmentCandidate> level1 = evaluateAroundSeeds(
        level0,
        parameters.level1SeedCount,
        templateModel,
        cameraMatrix,
        rvec0,
        tvec0,
        distancePyramid[1],
        scaleFactors[1],
        parameters.templateSampleIntervalPx,
        parameters.chamferTauPx * scaleFactors[1],
        level1DeltaXRange,
        parameters.level1DeltaXStepMm,
        level1DeltaYRange,
        parameters.level1DeltaYStepMm,
        level1DeltaPsiRange,
        parameters.level1DeltaPsiStepDeg,
        parameters.level1TopN,
        false);

    const double level2DeltaXRange = adaptiveSearchRange(
        level1,
        &IbvsTemplateAlignmentCandidate::dxMm,
        parameters.level2DeltaXRangeMm,
        parameters.level1DeltaXStepMm * 3.0);
    const double level2DeltaYRange = adaptiveSearchRange(
        level1,
        &IbvsTemplateAlignmentCandidate::dyMm,
        parameters.level2DeltaYRangeMm,
        parameters.level1DeltaYStepMm * 3.0);
    const double level2DeltaPsiRange = adaptiveSearchRange(
        level1,
        &IbvsTemplateAlignmentCandidate::dpsiDeg,
        parameters.level2DeltaPsiRangeDeg,
        parameters.level1DeltaPsiStepDeg * 3.0);

    std::vector<IbvsTemplateAlignmentCandidate> level2 = evaluateAroundSeeds(
        level1,
        parameters.level2SeedCount,
        templateModel,
        cameraMatrix,
        rvec0,
        tvec0,
        distancePyramid[2],
        scaleFactors[2],
        parameters.templateSampleIntervalPx,
        parameters.chamferTauPx,
        level2DeltaXRange,
        parameters.level2DeltaXStepMm,
        level2DeltaYRange,
        parameters.level2DeltaYStepMm,
        level2DeltaPsiRange,
        parameters.level2DeltaPsiStepDeg,
        2,
        true);

    IbvsTemplateAlignmentResult result;
    if (level2.empty())
    {
        return result;
    }

    keepBestCandidates(&level2, 2);
    result.best = level2.front();
    result.secondBest =
        level2.size() >= 2 ? level2[1] : IbvsTemplateAlignmentCandidate{};
    IbvsImageProjectionTools::computePerturbedPose(
        rvec0,
        tvec0,
        result.best.dxMm,
        result.best.dyMm,
        result.best.dpsiDeg,
        &result.rvecOpt,
        &result.tvecOpt);

    const std::vector<cv::Point2f> finalSampledPoints =
        IbvsImageProjectionTools::projectAndSampleEdges(
            templateModel,
            result.rvecOpt,
            result.tvecOpt,
            cameraMatrix,
            1.0,
            parameters.templateSampleIntervalPx);
    result.inlierRatio =
        computeInlierRatio(finalSampledPoints, distancePyramid[2], parameters.inlierEpsilonPx);
    return result;
}

} // namespace i2r_rlc::ibvs
