/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionExceptionProcessor.cpp
 * @brief       在线检测段异常处理实现
 * @details     复刻 OnlineExceptionHandling_beta 活动分支的单段/两段 ICP 语义。
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#include "geometry/IcpRegistrationMatcher.h"
#include "OnlineCorrectionExceptionProcessor.h"

namespace i2r_rlc::control {
namespace {

bool isValid(const OnlineCorrectionPoint &point)
{
    return point.valid
        && std::isfinite(point.xMm)
        && std::isfinite(point.yMm)
        && std::isfinite(point.zMm);
}

Eigen::Vector3d vectorOf(const OnlineCorrectionPoint &point)
{
    return { point.xMm, point.yMm, point.zMm };
}

vision::WeldSpacePoint toSpacePoint(const OnlineCorrectionPoint &point)
{
    return { point.xMm, point.yMm, point.zMm, isValid(point) };
}

OnlineCorrectionPoint fromVector(const Eigen::Vector3d &point)
{
    return { point.x(), point.y(), point.z(), point.allFinite() };
}

OnlineCorrectionPoint transformPoint(
    const OnlineCorrectionPoint &point,
    const Eigen::Matrix4d &transform)
{
    const Eigen::Vector4d source(point.xMm, point.yMm, point.zMm, 1.0);
    const Eigen::Vector4d target = transform * source;
    return { target.x(), target.y(), target.z(), target.allFinite() };
}

double squaredDistance(const OnlineCorrectionPoint &lhs, const OnlineCorrectionPoint &rhs)
{
    return (vectorOf(lhs) - vectorOf(rhs)).squaredNorm();
}

std::vector<vision::WeldSpacePoint> samplePoints(
    const std::vector<OnlineCorrectionPoint> &points,
    int interval)
{
    std::vector<vision::WeldSpacePoint> sampled;
    interval = std::max(1, interval);
    for (std::size_t index = 0; index < points.size(); index += static_cast<std::size_t>(interval)) {
        if (isValid(points[index])) {
            sampled.push_back(toSpacePoint(points[index]));
        }
    }
    return sampled;
}

std::vector<vision::WeldSpacePoint> sampleSamples(
    const std::vector<OnlineCorrectionDetectionSample> &samples,
    int begin,
    int end,
    int interval)
{
    std::vector<vision::WeldSpacePoint> sampled;
    interval = std::max(1, interval);
    begin = std::clamp(begin, 0, static_cast<int>(samples.size()));
    end = std::clamp(end, begin, static_cast<int>(samples.size()));
    for (int index = begin; index < end; index += interval) {
        if (isValid(samples[static_cast<std::size_t>(index)].point)) {
            sampled.push_back(toSpacePoint(samples[static_cast<std::size_t>(index)].point));
        }
    }
    return sampled;
}

std::vector<OnlineCorrectionPoint> pointsFromSamples(
    const std::vector<OnlineCorrectionDetectionSample> &samples)
{
    std::vector<OnlineCorrectionPoint> points;
    points.reserve(samples.size());
    for (const auto &sample : samples) {
        if (isValid(sample.point)) {
            points.push_back(sample.point);
        }
    }
    return points;
}

int nearestIndex(
    const std::vector<OnlineCorrectionPoint> &points,
    const OnlineCorrectionPoint &reference,
    int begin,
    int end)
{
    return COnlineCorrectionTrajectoryPlanner::findNearestPointIndex(
        begin, end, points, reference);
}

std::vector<OnlineCorrectionPoint> rangeWithClosedLoop(
    const std::vector<OnlineCorrectionPoint> &points,
    int firstIndex,
    int lastIndex)
{
    std::vector<OnlineCorrectionPoint> result;
    if (points.empty() || firstIndex < 0 || lastIndex < 0) {
        return result;
    }
    firstIndex = std::clamp(firstIndex, 0, static_cast<int>(points.size()) - 1);
    lastIndex = std::clamp(lastIndex, 0, static_cast<int>(points.size()) - 1);
    if (lastIndex >= firstIndex) {
        result.insert(result.end(), points.begin() + firstIndex, points.begin() + lastIndex + 1);
        return result;
    }
    result.insert(result.end(), points.begin() + firstIndex, points.end());
    result.insert(result.end(), points.begin(), points.begin() + lastIndex + 1);
    return result;
}

void mapGapsToProcessedPts(
    std::vector<OnlineCorrectionDetectionSample> *processed,
    const std::vector<OnlineCorrectionDetectionSample> &detected)
{
    if (processed == nullptr || detected.empty()) {
        return;
    }
    for (auto &sample : *processed) {
        double best = std::numeric_limits<double>::max();
        std::optional<float> gap;
        for (const auto &candidate : detected) {
            if (!candidate.gapWidthMm.has_value()) {
                continue;
            }
            const double distance = squaredDistance(sample.point, candidate.point);
            if (distance < best) {
                best = distance;
                gap = candidate.gapWidthMm;
            }
        }
        sample.gapWidthMm = gap;
    }
}

std::vector<OnlineCorrectionDetectionSample> makeSamples(
    const std::vector<OnlineCorrectionPoint> &points,
    int seamIndex)
{
    std::vector<OnlineCorrectionDetectionSample> samples;
    samples.reserve(points.size());
    for (const auto &point : points) {
        if (!isValid(point)) {
            continue;
        }
        OnlineCorrectionDetectionSample sample;
        sample.point = point;
        sample.simulatedSeamIndex = seamIndex;
        samples.push_back(sample);
    }
    return samples;
}

std::vector<OnlineCorrectionPoint> transformModelToDetected(
    const std::vector<OnlineCorrectionPoint> &model,
    const std::vector<OnlineCorrectionDetectionSample> &detected,
    int detectedBegin,
    int detectedEnd,
    int detectedInterval,
    bool singleMatch)
{
    vision::IcpRegistrationOptions options;
    options.dimension = 3;
    options.maxIterations = singleMatch ? 60 : 50;
    options.minDelta = 5.0e-4;

    const auto modelSampled = samplePoints(model, 1);
    const auto detectedSampled = sampleSamples(
        detected, detectedBegin, detectedEnd, detectedInterval);
    if (modelSampled.size() < 3U || detectedSampled.size() < 3U) {
        return {};
    }

    if (singleMatch) {
        const auto match = vision::CIcpRegistrationMatcher::alignPointToPoint(
            modelSampled, detectedSampled, options);
        if (!match.valid) {
            return {};
        }
        const Eigen::Matrix4d inverse = match.transformMatrix.inverse();
        std::vector<OnlineCorrectionPoint> transformed;
        transformed.reserve(model.size());
        for (const auto &point : model) {
            transformed.push_back(transformPoint(point, inverse));
        }
        return transformed;
    }

    const auto match = vision::CIcpRegistrationMatcher::alignPointToPoint(
        detectedSampled, modelSampled, options);
    if (!match.valid) {
        return {};
    }
    std::vector<OnlineCorrectionPoint> transformed;
    transformed.reserve(model.size());
    for (const auto &point : model) {
        transformed.push_back(transformPoint(point, match.transformMatrix));
    }
    return transformed;
}

void applyTranslationContinuity(
    std::vector<OnlineCorrectionPoint> *points,
    const OnlineCorrectionPoint &lastPoint)
{
    if (points == nullptr || points->empty() || !isValid(lastPoint)) {
        return;
    }
    const Eigen::Vector3d shift = vectorOf(lastPoint) - vectorOf(points->front());
    for (auto &point : *points) {
        point = fromVector(vectorOf(point) + shift);
    }
}

void applyRotationContinuity(
    std::vector<OnlineCorrectionPoint> *points,
    const OnlineCorrectionPoint &lastPoint)
{
    if (points == nullptr || points->size() < 2U || !isValid(lastPoint)) {
        return;
    }
    const Eigen::Vector3d source = vectorOf((*points)[1]) - vectorOf(points->front());
    const Eigen::Vector3d target = vectorOf(points->front()) - vectorOf(lastPoint);
    if (source.norm() <= 1.0e-9 || target.norm() <= 1.0e-9) {
        return;
    }
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond::FromTwoVectors(source.normalized(), target.normalized());
    const Eigen::Vector3d origin = vectorOf(points->front());
    for (auto &point : *points) {
        point = fromVector(origin + rotation * (vectorOf(point) - origin));
    }
}

} // namespace

OnlineCorrectionExceptionResult COnlineCorrectionExceptionProcessor::process(
    const std::vector<OnlineCorrectionDetectionSample> &newSamples,
    const OnlineCorrectionPlannedTrajectory &trajectory)
{
    OnlineCorrectionExceptionResult result;
    std::vector<OnlineCorrectionDetectionSample> accepted;
    accepted.reserve(newSamples.size());
    for (const auto &sample : newSamples) {
        if (isValid(sample.point)) {
            accepted.push_back(sample);
        }
    }
    if (accepted.empty()) {
        result.message = "online detection samples contain no valid point";
        return result;
    }

    const int seamIndex = accepted.back().simulatedSeamIndex;
    bool seamEdge = m_lastSimulatedSeamIndex >= 0 && seamIndex >= 0
        && seamIndex != m_lastSimulatedSeamIndex;
    if (seamIndex >= 0) {
        m_lastSimulatedSeamIndex = seamIndex;
    }

    std::vector<OnlineCorrectionDetectionSample> normalUpdate = m_normalSamples;
    normalUpdate.insert(normalUpdate.end(), accepted.begin(), accepted.end());

    int curveIndex = -1;
    for (const auto &sample : accepted) {
        if (sample.scanLineChangePoint.has_value()
            && isValid(*sample.scanLineChangePoint)) {
            curveIndex = nearestIndex(
                pointsFromSamples(normalUpdate),
                *sample.scanLineChangePoint,
                0,
                static_cast<int>(normalUpdate.size()));
            break;
        }
    }
    if (curveIndex < 0 && seamEdge) {
        curveIndex = static_cast<int>(m_normalSamples.size());
    }
    result.segmentChanged = curveIndex >= 0;
    if (result.segmentChanged) {
        ++m_curveFlag;
    }

    const std::vector<std::vector<OnlineCorrectionPoint>> fallbackSegments{
        trajectory.segmentPathPoints
    };
    const auto &segments =
        trajectory.pathSegments.empty() ? fallbackSegments : trajectory.pathSegments;
    const int currentSegmentIndex = std::clamp(m_curveFlag,0, static_cast<int>(segments.size()) - 1);
    const auto &currentTeachSegment = segments[static_cast<std::size_t>(currentSegmentIndex)];

    std::vector<OnlineCorrectionDetectionSample> processed;
    if (curveIndex > 0 && m_curveFlag > 0
        && static_cast<std::size_t>(currentSegmentIndex) < segments.size()) {
        const int previousSegmentIndex = std::clamp(
            currentSegmentIndex - 1,
            0,
            static_cast<int>(segments.size()) - 1);
        const auto &previousTeachSegment = segments[static_cast<std::size_t>(previousSegmentIndex)];

        const auto previousMatched = transformModelToDetected(
            previousTeachSegment, normalUpdate, 0, curveIndex, 2, false);
        const auto nextMatched = transformModelToDetected(
            currentTeachSegment,
            normalUpdate,
            curveIndex,
            static_cast<int>(normalUpdate.size()),
            3,
            false);

        if (!previousMatched.empty()) {
            const auto detPoints = pointsFromSamples(normalUpdate);
            const int first = nearestIndex(
                previousMatched,
                detPoints.front(),
                0,
                static_cast<int>(previousMatched.size()));
            const int last = nearestIndex(
                previousMatched,
                normalUpdate[static_cast<std::size_t>(curveIndex - 1)].point,
                0,
                static_cast<int>(previousMatched.size()));
            const auto previousRange = rangeWithClosedLoop(previousMatched, first, last);
            auto previousSamples = makeSamples(previousRange, seamIndex);
            processed.insert(processed.end(), previousSamples.begin(), previousSamples.end());
        }
        if (!nextMatched.empty()) {
            const int first = nearestIndex(
                nextMatched,
                normalUpdate[static_cast<std::size_t>(curveIndex)].point,
                0,
                static_cast<int>(nextMatched.size()));
            const int last = nearestIndex(
                nextMatched,
                normalUpdate.back().point,
                0,
                static_cast<int>(nextMatched.size()));
            const auto nextRange = rangeWithClosedLoop(nextMatched, first, last);
            auto nextSamples = makeSamples(nextRange, seamIndex);
            processed.insert(processed.end(), nextSamples.begin(), nextSamples.end());
        }
        if (isValid(m_lastMatchPoint)) {
            std::vector<OnlineCorrectionPoint> points = pointsFromSamples(processed);
            applyTranslationContinuity(&points, m_lastMatchPoint);
            processed = makeSamples(points, seamIndex);
        }
    } else if (!currentTeachSegment.empty()) {
        const auto matched = transformModelToDetected(
            currentTeachSegment,
            normalUpdate,
            0,
            static_cast<int>(normalUpdate.size()),
            3,
            true);
        if (!matched.empty()) {
            const int first = nearestIndex(
                matched,
                normalUpdate.front().point,
                0,
                static_cast<int>(matched.size()));
            const int last = nearestIndex(
                matched,
                normalUpdate.back().point,
                0,
                static_cast<int>(matched.size()));
            std::vector<OnlineCorrectionPoint> range = rangeWithClosedLoop(matched, first, last);
            if (m_matchTime != 0 && isValid(m_lastMatchPoint)) {
                applyTranslationContinuity(&range, m_lastMatchPoint);
                applyRotationContinuity(&range, m_lastMatchPoint);
            }
            processed = makeSamples(range, seamIndex);
        }
    }

    if (processed.empty()) {
        processed = accepted;
        result.message = result.segmentChanged
            ? "ICP failed; raw detection samples were kept"
            : "online detection segment accepted";
    } else {
        mapGapsToProcessedPts(&processed, accepted);
        result.message = result.segmentChanged
            ? " two/one segment ICP processed"
            : "single ICP processed";
    }

    if (!processed.empty()) {
        m_lastMatchPoint = processed.back().point;
        ++m_matchTime;
    }

    m_normalSamples.clear();
    const std::size_t retainedBegin = accepted.size() / 5U;
    m_normalSamples.insert(
        m_normalSamples.end(),
        accepted.begin() + static_cast<std::ptrdiff_t>(retainedBegin),
        accepted.end());
    if (!normalUpdate.empty()) {
        m_preDetLstPtToAllTpId = m_segDetLstPtToAllTpId;
        const auto allPoints = trajectory.allMatchedPathPoints.empty()
            ? trajectory.allPathPoints
            : trajectory.allMatchedPathPoints;
        if (!allPoints.empty()) {
            m_segDetFstPtToAllTpId = nearestIndex(
                allPoints,
                normalUpdate.front().point,
                0,
                static_cast<int>(allPoints.size()));
            m_segDetLstPtToAllTpId = nearestIndex(
                allPoints,
                normalUpdate.back().point,
                0,
                static_cast<int>(allPoints.size()));
        }
    }

    result.samples = std::move(processed);
    result.valid = !result.samples.empty();
    return result;
}

OnlineCorrectionExceptionResult COnlineCorrectionExceptionProcessor::process(
    const std::vector<OnlineCorrectionDetectionSample> &newSamples,
    const std::vector<OnlineCorrectionPoint> &referenceSegment)
{
    OnlineCorrectionPlannedTrajectory trajectory;
    trajectory.valid = !referenceSegment.empty();
    trajectory.allPathPoints = referenceSegment;
    trajectory.segmentPathPoints = referenceSegment;
    trajectory.allMatchedPathPoints = referenceSegment;
    trajectory.pathSegments.push_back(referenceSegment);
    return process(newSamples, trajectory);
}

void COnlineCorrectionExceptionProcessor::reset()
{
    m_normalSamples.clear();
    m_lastSimulatedSeamIndex = -1;
    m_curveFlag = 0;
    m_matchTime = 0;
    m_segDetFstPtToAllTpId = 0;
    m_segDetLstPtToAllTpId = 0;
    m_preDetLstPtToAllTpId = 0;
    m_lastMatchPoint = {};
}

} // namespace i2r_rlc::control
