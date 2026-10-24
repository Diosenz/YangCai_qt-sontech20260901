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
 * @file        OnlineCorrectionPathFollower.cpp
 * @brief       在线纠偏实时路径跟随实现
 * @details     按 CorrectTaughtPathO 的目标点选择、纠偏量计算和控制输出三段迁移。
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Core>

#include "OnlineCorrectionPathFollower.h"

namespace i2r_rlc::control {
namespace {

Eigen::Vector3d vectorOf(const OnlineCorrectionPoint &point)
{
    return { point.xMm, point.yMm, point.zMm };
}

OnlineCorrectionPoint makePoint(const Eigen::Vector3d &point)
{
    return { point.x(), point.y(), point.z(), point.allFinite() };
}

bool isValid(const OnlineCorrectionPoint &point)
{
    return point.valid
        && std::isfinite(point.xMm)
        && std::isfinite(point.yMm)
        && std::isfinite(point.zMm);
}

vision::PositionerPose robotPose(const robot_comm::RobotState &state)
{
    return {
        static_cast<double>(state.curx_r) / 1000.0,
        static_cast<double>(state.cury_r) / 1000.0,
        static_cast<double>(state.curz_r) / 1000.0,
        static_cast<double>(state.curRx_r) / 10000.0,
        static_cast<double>(state.curRy_r) / 10000.0,
        static_cast<double>(state.curRz_r) / 10000.0,
        true
    };
}

double cosineBetween(const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs)
{
    const double denom = lhs.norm() * rhs.norm();
    if (denom <= 1.0e-9) {
        return 1.0;
    }
    return lhs.dot(rhs) / denom;
}

Eigen::Vector3d projectOnLine(
    const Eigen::Vector3d &point,
    const Eigen::Vector3d &lineStart,
    const Eigen::Vector3d &lineEnd)
{
    const Eigen::Vector3d direction = lineEnd - lineStart;
    const double lengthSquared = direction.squaredNorm();
    if (lengthSquared <= 1.0e-12) {
        return lineStart;
    }
    const double t = direction.dot(point - lineStart) / lengthSquared;
    return lineStart + t * direction;
}

struct TaughtDirectionResult {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    int index = -1;
};

TaughtDirectionResult taughtDirectionAt(
    const std::vector<OnlineCorrectionPoint> &teachTrajectory,
    const OnlineCorrectionPoint &current,
    int *cursor)
{
    if (cursor == nullptr || teachTrajectory.size() < 2U) {
        return {};
    }
    const int pointCount = static_cast<int>(teachTrajectory.size());
    const int searchBegin = std::clamp(*cursor, 0, pointCount - 1);
    const int searchEnd = std::min(pointCount, searchBegin + 100);
    int nearest = searchBegin;
    double best = std::numeric_limits<double>::max();
    for (int index = searchBegin; index < searchEnd; ++index) {
        if (!isValid(teachTrajectory[static_cast<std::size_t>(index)])) {
            continue;
        }
        const double distance =
            (vectorOf(teachTrajectory[static_cast<std::size_t>(index)]) - vectorOf(current))
                .squaredNorm();
        if (distance < best) {
            best = distance;
            nearest = index;
        }
    }
    *cursor = nearest;
    const int next = std::min(nearest + 3, pointCount - 1);
    const int previous = std::max(nearest - 3, 0);
    Eigen::Vector3d direction =
        vectorOf(teachTrajectory[static_cast<std::size_t>(next)])
        - vectorOf(teachTrajectory[static_cast<std::size_t>(previous)]);
    if (direction.norm() <= 1.0e-9 && nearest + 1 < static_cast<int>(teachTrajectory.size())) {
        direction =
            vectorOf(teachTrajectory[static_cast<std::size_t>(nearest + 1)])
            - vectorOf(teachTrajectory[static_cast<std::size_t>(nearest)]);
    }
    return {
        direction.norm() <= 1.0e-9 ? Eigen::Vector3d::Zero() : direction.normalized(),
        nearest};
}

float gapAtCurrentPose(
    const std::vector<OnlineCorrectionDetectionSample> &samples,
    const OnlineCorrectionPoint &current)
{
    if (samples.size() < 2U) {
        return samples.empty() ? 0.0F : samples.front().gapWidthMm.value_or(0.0F);
    }

    const Eigen::Vector3d currentVector = vectorOf(current);
    double bestDistance = std::numeric_limits<double>::max();
    float bestGap = 0.0F;
    for (std::size_t index = 0; index + 1 < samples.size(); ++index) {
        if (!samples[index].gapWidthMm.has_value()) {
            continue;
        }
        const Eigen::Vector3d a = vectorOf(samples[index].point);
        const Eigen::Vector3d b = vectorOf(samples[index + 1U].point);
        const Eigen::Vector3d ab = b - a;
        if (ab.squaredNorm() <= 1.0e-12) {
            continue;
        }
        const double t = std::clamp(ab.dot(currentVector - a) / ab.squaredNorm(), 0.0, 1.0);
        const Eigen::Vector3d foot = a + t * ab;
        const double distance = (currentVector - foot).squaredNorm();
        if (distance < bestDistance) {
            bestDistance = distance;
            const float gap0 = samples[index].gapWidthMm.value_or(0.0F);
            const float gap1 = samples[index + 1U].gapWidthMm.value_or(gap0);
            bestGap = static_cast<float>((1.0 - t) * gap0 + t * gap1);
        }
    }
    return bestGap;
}

float normalizeGap(float gap)
{
    return std::abs(gap - 1.0F) <= std::abs(gap - 2.0F) ? 1.0F : 2.0F;
}

long quantizeSpeed(long value)
{
    value = std::clamp(value, 1000L, 10000L);
    return ((value + 500L) / 1000L) * 1000L;
}

long clampJump(long value, long previous, const OnlineCorrectionOptions &options)
{
    if ((previous != 0 && std::abs(value - previous) > options.maxCorrectionJumpUm)
        || std::abs(value) > options.maxCorrectionIncrementUm) {
        return previous;
    }
    return value;
}

double historySum(const std::array<double, 10> &history, int count)
{
    return std::accumulate(history.begin(), history.begin() + count, 0.0);
}

void pushHistory(std::array<double, 10> *history, int count, double value)
{
    if (history == nullptr) {
        return;
    }
    if (count < 10) {
        (*history)[count] = value;
        return;
    }
    std::move(history->begin() + 1, history->end(), history->begin());
    history->back() = value;
}

double clampFuzzyInput(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

} // namespace

OnlineCorrectionFollowResult COnlineCorrectionPathFollower::update(
    const robot_comm::RobotState &robotState,
    const OnlineCorrectionOptions &options,
    const vision::PositionerCalibration &calibration,
    const std::vector<OnlineCorrectionPoint> &teachTrajectory,
    std::vector<OnlineCorrectionDetectionSample> *detectionQueue,
    bool finishing)
{
    OnlineCorrectionFollowResult result;
    if (detectionQueue == nullptr || detectionQueue->size() <= 1U) {
        result.completed = finishing;
        result.message = finishing ? "online segment completed" : "waiting for online detection points";
        return result;
    }
    if (!calibration.valid || teachTrajectory.size() < 2U) {
        result.message = "online correction geometry is not configured";
        return result;
    }

    const double s1Deg = static_cast<double>(robotState.curS1) / 10000.0;
    const double s2Deg = static_cast<double>(robotState.curS2) / 10000.0;
    const auto stationCurrent =
        vision::CPositionerKinematics::robotPoseToStationPose(
            robotPose(robotState), s1Deg, s2Deg, calibration);
    if (!stationCurrent.valid) {
        result.message = "robot pose to positioner pose conversion failed";
        return result;
    }
    const OnlineCorrectionPoint current{
        stationCurrent.xMm,
        stationCurrent.yMm,
        stationCurrent.zMm,
        true
    };

    while (detectionQueue->size() > 1U) {
        const OnlineCorrectionPoint firstPoint = detectionQueue->front().point;
        const OnlineCorrectionPoint thirdPoint =
            (*detectionQueue)[std::min<std::size_t>(3U, detectionQueue->size() - 1U)].point;
        const Eigen::Vector3d first = vectorOf(firstPoint);
        const Eigen::Vector3d third = vectorOf(thirdPoint);
        const Eigen::Vector3d currentFromFirst = vectorOf(current) - first;
        const Eigen::Vector3d direction = third - first;
        const double angle = cosineBetween(currentFromFirst, direction);
        if ((vectorOf(current) - first).norm() <= options.passedPointDistanceMm
            || angle >= -0.1) {
            detectionQueue->erase(detectionQueue->begin());
            continue;
        }
        break;
    }
    if (detectionQueue->size() <= 1U) {
        result.completed = finishing;
        result.message = finishing ? "online segment completed" : "detection points were passed";
        return result;
    }

    const OnlineCorrectionPoint firstPoint = detectionQueue->front().point;
    if (m_lastTarget.valid
        && vectorOf(m_lastTarget).isApprox(vectorOf(firstPoint), 1.0e-9)) {
        ++m_sameTargetCount;
    } else {
        m_sameTargetCount = 1;
    }
    m_lastTarget = firstPoint;
    if (m_sameTargetCount > 10) {
        detectionQueue->erase(detectionQueue->begin());
        m_sameTargetCount = 0;
        result.message = "repeated online target point removed";
        return result;
    }

    const OnlineCorrectionPoint thirdPoint =
        (*detectionQueue)[std::min<std::size_t>(3U, detectionQueue->size() - 1U)].point;
    Eigen::Vector3d detectionDirection = vectorOf(thirdPoint) - vectorOf(firstPoint);
    if (detectionDirection.norm() <= 1.0e-9) {
        result.message = "online detection direction is degenerate";
        return result;
    }
    detectionDirection.normalize();

    if (m_weldDirectionHistoryCount < 10) {
        m_weldDirectionXHistory[m_weldDirectionHistoryCount] = detectionDirection.x();
        m_weldDirectionYHistory[m_weldDirectionHistoryCount] = detectionDirection.y();
        m_weldDirectionZHistory[m_weldDirectionHistoryCount] = detectionDirection.z();
        ++m_weldDirectionHistoryCount;
    } else {
        pushHistory(&m_weldDirectionXHistory, 10, detectionDirection.x());
        pushHistory(&m_weldDirectionYHistory, 10, detectionDirection.y());
        pushHistory(&m_weldDirectionZHistory, 10, detectionDirection.z());
    }

    const Eigen::Vector3d footOnDetection =
        projectOnLine(vectorOf(current), vectorOf(firstPoint), vectorOf(thirdPoint));
    const TaughtDirectionResult taught = taughtDirectionAt(
        teachTrajectory, current, &m_teachPointCursor);
    const Eigen::Vector3d &taughtDirection = taught.direction;
    result.taughtPointIndex = taught.index;
    const Eigen::Vector3d predictedTeach =
        vectorOf(current) + taughtDirection * options.stepTcpVelocityMmPerSecond
            * options.controlCycleSecond;
    const Eigen::Vector3d footOnFutureDetection =
        projectOnLine(predictedTeach, vectorOf(firstPoint), vectorOf(thirdPoint));

    vision::PositionerPose stationFoot = stationCurrent;
    stationFoot.xMm = footOnDetection.x();
    stationFoot.yMm = footOnDetection.y();
    stationFoot.zMm = footOnDetection.z();
    vision::PositionerPose stationPredictedTeach = stationCurrent;
    stationPredictedTeach.xMm = predictedTeach.x();
    stationPredictedTeach.yMm = predictedTeach.y();
    stationPredictedTeach.zMm = predictedTeach.z();
    vision::PositionerPose stationFutureFoot = stationCurrent;
    stationFutureFoot.xMm = footOnFutureDetection.x();
    stationFutureFoot.yMm = footOnFutureDetection.y();
    stationFutureFoot.zMm = footOnFutureDetection.z();

    const auto rp1 = vision::CPositionerKinematics::stationPoseToRobotPose(
        stationFoot, s1Deg, s2Deg, calibration);
    const auto rp2 = vision::CPositionerKinematics::stationPoseToRobotPose(
        stationCurrent, s1Deg, s2Deg, calibration);
    const auto rp3 = vision::CPositionerKinematics::stationPoseToRobotPose(
        stationPredictedTeach, s1Deg, s2Deg, calibration);
    const auto rp4 = vision::CPositionerKinematics::stationPoseToRobotPose(
        stationFutureFoot, s1Deg, s2Deg, calibration);
    if (!rp1.valid || !rp2.valid || !rp3.valid || !rp4.valid) {
        result.message = "positioner pose conversion failed";
        return result;
    }

    Eigen::Vector3d oriToDetected(
        rp1.xMm - rp2.xMm,
        rp1.yMm - rp2.yMm,
        rp1.zMm - rp2.zMm);
    const double d11 = oriToDetected.norm();
    Eigen::Vector3d correctionDirection = Eigen::Vector3d::Zero();
    if (d11 > 1.0e-9) {
        correctionDirection = oriToDetected / d11;
    }

    constexpr double legacyCorrectionVelocityMmPerSecond = 0.1;
    Eigen::Vector3d baseStep =
        correctionDirection * legacyCorrectionVelocityMmPerSecond
        * options.controlCycleSecond * 1000.0;
    baseStep.z() *= 0.5;

    if (m_historyCount < 10) {
        pushHistory(&m_errorDistanceHistory, m_historyCount, d11);
        pushHistory(&m_directionXHistory, m_historyCount, correctionDirection.x());
        pushHistory(&m_directionYHistory, m_historyCount, correctionDirection.y());
        pushHistory(&m_directionZHistory, m_historyCount, correctionDirection.z());
        ++m_historyCount;
    } else {
        pushHistory(&m_errorDistanceHistory, 10, d11);
        pushHistory(&m_directionXHistory, 10, correctionDirection.x());
        pushHistory(&m_directionYHistory, 10, correctionDirection.y());
        pushHistory(&m_directionZHistory, 10, correctionDirection.z());
    }

    if (m_controlCount == 0) {
        m_lastErrorDistanceMm = d11;
        m_lastBaseDxUm = baseStep.x();
        m_lastBaseDyUm = baseStep.y();
        m_lastBaseDzUm = baseStep.z();
    }

    Eigen::Vector3d output = Eigen::Vector3d::Zero();
    switch (options.controlMethod) {
    case OnlineCorrectionControlMethod::LegacyPidThreshold: {
        if (d11 <= 5000.0) {
            constexpr double kp = 3.0;
            constexpr double ki = 0.2;
            constexpr double kd = 7.0;
            output.x() = kp * baseStep.x()
                + ki * historySum(m_directionXHistory, m_historyCount)
                + kd * (baseStep.x() - m_lastBaseDxUm);
            output.y() = kp * baseStep.y()
                + ki * historySum(m_directionYHistory, m_historyCount)
                + kd * (baseStep.y() - m_lastBaseDyUm);
            output.z() = kp * baseStep.z()
                + ki * historySum(m_directionZHistory, m_historyCount)
                + kd * (baseStep.z() - m_lastBaseDzUm);
        }
        break;
    }
    case OnlineCorrectionControlMethod::LegacyFuzzyAdaptivePid: {
        const double fuzzyError = clampFuzzyInput(d11);
        const double fuzzyDelta = clampFuzzyInput(std::abs(d11) - std::abs(m_lastErrorDistanceMm));
        // 当前新工程没有旧 fisMatrixFile.txt 运行时依赖；默认 FIS 增量为 0，
        // 保留输入归一化和参数应用位置，后续可直接替换为表驱动 FIS。
        const double deltaKp = 0.0 * fuzzyError;
        const double deltaKi = 0.0 * fuzzyDelta;
        const double deltaKd = 0.0;
        const double kp = 5.0 + deltaKp;
        const double ki = 0.7 - deltaKi;
        const double kd = 10.0 - deltaKd;
        output.x() = kp * baseStep.x()
            + ki * historySum(m_directionXHistory, m_historyCount)
            + kd * (baseStep.x() - m_lastBaseDxUm);
        output.y() = kp * baseStep.y()
            + ki * historySum(m_directionYHistory, m_historyCount)
            + kd * (baseStep.y() - m_lastBaseDyUm);
        output.z() = kp * baseStep.z()
            + ki * historySum(m_directionZHistory, m_historyCount)
            + kd * (baseStep.z() - m_lastBaseDzUm);
        break;
    }
    case OnlineCorrectionControlMethod::LegacyReservedNoop:
        output.setZero();
        break;
    }

    robot_comm::RobotRealtimeCorrectionCommand command;
    command.dxUm = clampJump(static_cast<long>(std::llround(output.x())), m_lastDxUm, options);
    command.dyUm = clampJump(static_cast<long>(std::llround(output.y())), m_lastDyUm, options);
    command.dzUm = clampJump(static_cast<long>(std::llround(output.z())), m_lastDzUm, options);
    command.speedOverride = 10000;
    if (options.enableGapDetection) {
        const float gap = normalizeGap(gapAtCurrentPose(*detectionQueue, current));
        command.speedOverride = quantizeSpeed(
            static_cast<long>(((300.0F - gap * 60.0F) / 300.0F) * 10000.0F));
    }
    command.speedMode = robotState.cur_interpolation == 2 ? 1 : 0;
    command.valid = true;
    command.message = "realtime online correction generated";

    m_lastErrorDistanceMm = d11;
    m_lastBaseDxUm = baseStep.x();
    m_lastBaseDyUm = baseStep.y();
    m_lastBaseDzUm = baseStep.z();
    m_lastDxUm = command.dxUm;
    m_lastDyUm = command.dyUm;
    m_lastDzUm = command.dzUm;
    ++m_controlCount;

    result.diagnostic.valid = true;
    result.diagnostic.current = current;
    result.diagnostic.first = firstPoint;
    result.diagnostic.directionPoint = thirdPoint;
    result.diagnostic.foot = {footOnDetection.x(), footOnDetection.y(), footOnDetection.z(), true};
    result.diagnostic.futureFoot = {
        footOnFutureDetection.x(), footOnFutureDetection.y(), footOnFutureDetection.z(), true};
    result.diagnostic.s1Deg = s1Deg;
    result.diagnostic.s2Deg = s2Deg;
    result.valid = true;
    result.command = command;
    result.message = command.message;
    return result;
}

void COnlineCorrectionPathFollower::reset()
{
    m_errorDistanceHistory.fill(0.0);
    m_directionXHistory.fill(0.0);
    m_directionYHistory.fill(0.0);
    m_directionZHistory.fill(0.0);
    m_weldDirectionXHistory.fill(0.0);
    m_weldDirectionYHistory.fill(0.0);
    m_weldDirectionZHistory.fill(0.0);
    m_historyCount = 0;
    m_weldDirectionHistoryCount = 0;
    m_controlCount = 0;
    m_sameTargetCount = 0;
    m_lastTarget = {};
    m_lastErrorDistanceMm = 0.0;
    m_lastBaseDxUm = 0.0;
    m_lastBaseDyUm = 0.0;
    m_lastBaseDzUm = 0.0;
    m_lastDxUm = 0;
    m_lastDyUm = 0;
    m_lastDzUm = 0;
    m_teachPointCursor = -1;
}

} // namespace i2r_rlc::control
