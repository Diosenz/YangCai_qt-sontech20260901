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
 * @file        OnlineCorrectionController.cpp
 * @brief       在线纠偏状态机实现
 * @details     串行处理视觉样本、机器人状态和命令回执，生成 O/P 协议 DTO。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "OnlineCorrectionController.h"

namespace i2r_rlc::control {

namespace {

/** @brief 控制几何要求标定矩阵和 TCP 偏移均为有限值。 */
bool usableCalibration(const vision::PositionerCalibration &calibration)
{
    return calibration.valid
        && calibration.stationBaseInRobotBase.matrix().allFinite()
        && calibration.toolTcpInLastLinkMm.allFinite();
}

bool finitePoint(const OnlineCorrectionPoint &point)
{
    return point.valid && std::isfinite(point.xMm)
        && std::isfinite(point.yMm) && std::isfinite(point.zMm);
}

/** @brief 将起点规划的同一刚体变换应用到在线参考的所有几何子段。 */
OnlineCorrectionPlannedTrajectory correctedOnlineTrajectory(
    const OnlineCorrectionPlannedTrajectory &trajectory,
    const OnlineCorrectionOptions &options,
    const std::vector<OnlineCorrectionPoint> &correctedPath)
{
    auto result = trajectory;
    if (correctedPath.empty() || trajectory.segmentPathPoints.empty()) {
        return result;
    }
    const auto globalPoint = [&options](const OnlineCorrectionPoint &point) {
        if (options.startMode != OnlineCorrectionStartMode::GlobalCorrection) {
            return point;
        }
        const auto &matrix = options.globalCorrectionRowMajor;
        return OnlineCorrectionPoint{
            matrix[0] * point.xMm + matrix[1] * point.yMm + matrix[2] * point.zMm + matrix[3],
            matrix[4] * point.xMm + matrix[5] * point.yMm + matrix[6] * point.zMm + matrix[7],
            matrix[8] * point.xMm + matrix[9] * point.yMm + matrix[10] * point.zMm + matrix[11],
            point.valid };
    };
    const auto first = globalPoint(trajectory.segmentPathPoints.front());
    const auto transform = [&globalPoint, &first, &correctedPath](OnlineCorrectionPoint &point) {
        point = globalPoint(point);
        point.xMm += correctedPath.front().xMm - first.xMm;
        point.yMm += correctedPath.front().yMm - first.yMm;
        point.zMm += correctedPath.front().zMm - first.zMm;
    };
    result.allMatchedPathPoints = trajectory.allPathPoints;
    for (auto &point : result.allMatchedPathPoints) {
        transform(point);
    }
    for (auto &segment : result.pathSegments) {
        for (auto &point : segment) {
            transform(point);
        }
    }
    result.segmentPathPoints = correctedPath;
    return result;
}

/** @brief 原始点旁路不调用异常处理器，也不持有其窗口。 */
OnlineCorrectionExceptionResult processOnlineSamples(
    COnlineCorrectionExceptionProcessor &processor,
    const OnlineCorrectionOptions &options,
    const OnlineCorrectionPlannedTrajectory &trajectory,
    const std::vector<OnlineCorrectionDetectionSample> &samples, bool flush)
{
    if (options.enableOnlineIcp) {
        return processor.process(samples, trajectory, flush);
    }
    OnlineCorrectionExceptionResult result;
    result.valid = true;
    result.samples = samples;
    result.message = "online ICP disabled; raw detection samples used";
    return result;
}

void updateIcpStatus(OnlineCorrectionSessionState &state, bool enabled,
                     const OnlineCorrectionExceptionResult &result)
{
    state.onlineIcp.enabled = enabled;
    state.onlineIcp.attempted = result.icpAttempted;
    state.onlineIcp.applied = result.icpApplied;
    state.onlineIcp.bufferedSampleCount = result.bufferedSampleCount;
    state.onlineIcp.rmsErrorMm = result.rmsErrorMm;
    state.onlineIcp.message = result.message;
}

} // namespace

void COnlineCorrectionController::configure(
    const OnlineCorrectionOptions &options,
    const OnlineCorrectionPlannedTrajectory &trajectory,
    const vision::PositionerCalibration &positionerCalibration)
{
    m_options = options;
    m_trajectory = trajectory;
    m_onlineTrajectory = trajectory;
    m_positionerCalibration = positionerCalibration;
    m_state = OnlineCorrectionSessionState{};
    m_state.startMode = options.startMode;
    m_state.onlineIcp.enabled = options.enableOnlineIcp;
    m_state.segmentIndex = trajectory.segmentIndex;
    m_state.startTeachIndex = trajectory.startTeachIndex;
    m_state.endTeachIndex = trajectory.endTeachIndex;
    m_state.onlineTeachTrajectoryInitialized = trajectory.valid;
    m_state.positionerCalibrationInitialized = usableCalibration(positionerCalibration);
    m_exceptionProcessor.reset();
    m_pathFollower.reset();

    const bool configured = trajectory.valid && m_state.positionerCalibrationInitialized;
    setPhase(
        OnlineCorrectionPhase::Idle,
        configured ? "online correction configured"
                   : "trajectory or RBCALIB calibration is invalid");
}

void COnlineCorrectionController::configure(
    const OnlineCorrectionOptions &options,
    const OnlineCorrectionPlannedTrajectory &trajectory)
{
    configure(options, trajectory, vision::PositionerCalibration{});
}

OnlineCorrectionUpdate COnlineCorrectionController::startInitialDetection()
{
    if (!m_state.onlineTeachTrajectoryInitialized
        || !m_state.positionerCalibrationInitialized) {
        setPhase(OnlineCorrectionPhase::Error, "online trajectory and RBCALIB must be configured");
        return makeUpdate(false, m_state.message);
    }

    m_state.initialDetectionSamples.clear();
    m_onlineTrajectory = m_trajectory;
    m_exceptionProcessor.reset();
    m_pathFollower.reset();
    m_state.onlineIcp = OnlineCorrectionIcpStatus{};
    m_state.onlineIcp.enabled = m_options.enableOnlineIcp;
    m_state.onlineDetectionSamples.clear();
    m_state.correctedSegmentPath.clear();
    m_state.initialPathMatch = OnlineCorrectionInitialPathMatch{};
    m_state.initialDetectionFinished = false;
    m_state.reverseCompletionArmed =
        m_robotState.onlineReverseCompleteFlag == 0;
    m_state.teachPointSendIndex = 0;
    m_state.initialTeachPointCount = 0;
    m_state.trackerActive = true;
    setPhase(OnlineCorrectionPhase::InitialDetectionRunning, "initial detection started");
    OnlineCorrectionUpdate update = makeUpdate(true, m_state.message);
    update.startTracking = true;
    return update;
}

OnlineCorrectionUpdate COnlineCorrectionController::appendDetectionEvent(
    const OnlineCorrectionDetectionEvent &event)
{
    const bool initialPhase =
        m_state.phase == OnlineCorrectionPhase::InitialDetectionRunning
        || m_state.phase == OnlineCorrectionPhase::WaitingForReverseCompletion;
    const bool onlinePhase =
        m_state.phase == OnlineCorrectionPhase::OnlineRunning;
    if (!initialPhase && !onlinePhase) {
        return makeUpdate(false, "detection event is not accepted in current phase");
    }

    std::vector<OnlineCorrectionDetectionSample> accepted;
    for (const auto &sample : event.samples) {
        if (!finitePoint(sample.point)) {
            continue;
        }
        if (m_options.enableGapDetection && !sample.gapWidthMm.has_value()) {
            setPhase(OnlineCorrectionPhase::Error, "gap width is required when gap detection is enabled");
            OnlineCorrectionUpdate update = makeUpdate(false, m_state.message);
            update.stopTracking = true;
            return update;
        }

        OnlineCorrectionDetectionSample normalized = sample;
        if (!initialPhase) {
            // 在线异常处理、参考轨迹和机器人反馈均使用末节系。入口只转换
            // 新样本一次；初始缓存保留 TCP 点，由起点规划器完成其转换。
            normalized.point.xMm += m_positionerCalibration.toolTcpInLastLinkMm.x();
            normalized.point.yMm += m_positionerCalibration.toolTcpInLastLinkMm.y();
            normalized.point.zMm += m_positionerCalibration.toolTcpInLastLinkMm.z();
            if (normalized.scanLineChangePoint.has_value()) {
                normalized.scanLineChangePoint->xMm += m_positionerCalibration.toolTcpInLastLinkMm.x();
                normalized.scanLineChangePoint->yMm += m_positionerCalibration.toolTcpInLastLinkMm.y();
                normalized.scanLineChangePoint->zMm += m_positionerCalibration.toolTcpInLastLinkMm.z();
            }
            if (!finitePoint(normalized.point)) {
                continue;
            }
        }
        if (!m_options.enableGapDetection) {
            normalized.gapWidthMm = std::nullopt;
        }
        accepted.push_back(normalized);
    }

    if (initialPhase) {
        m_state.initialDetectionSamples.insert(
            m_state.initialDetectionSamples.end(), accepted.begin(), accepted.end());
        if (event.detectionFinished) {
            m_state.initialDetectionFinished = true;
            setPhase(
                OnlineCorrectionPhase::WaitingForReverseCompletion,
                "initial detection finished; waiting for reverse completion");
        }
        return makeUpdate(true, event.message.empty() ? m_state.message : event.message);
    }

    const auto processed = processOnlineSamples(
        m_exceptionProcessor, m_options, m_onlineTrajectory, accepted, event.detectionFinished);
    updateIcpStatus(m_state, m_options.enableOnlineIcp, processed);
    if (!processed.valid) {
        return makeUpdate(false, processed.message);
    }
    m_state.onlineDetectionSamples.insert(
        m_state.onlineDetectionSamples.end(),
        processed.samples.begin(),
        processed.samples.end());
    if (!processed.samples.empty()) {
        m_state.simulatedSeamIndex = processed.samples.back().simulatedSeamIndex;
    }

    const auto follow = m_pathFollower.update(
        m_robotState,
        m_options,
        m_positionerCalibration,
        m_onlineTrajectory.segmentPathPoints,
        &m_state.onlineDetectionSamples,
        false);

    OnlineCorrectionUpdate update = makeUpdate(follow.valid, follow.message);
    update.onlineIcpUpdated = true;
    if (!follow.command.valid && processed.valid) {
        update.succeeded = true;
        update.message = processed.message + "; " + follow.message;
    }
    update.realtimeCommand = follow.command;
    update.taughtPointIndex = follow.taughtPointIndex;
    if (event.detectionFinished) {
        setPhase(OnlineCorrectionPhase::SegmentFinishing, "online segment is finishing");
        update = makeUpdate(true, m_state.message);
        update.onlineIcpUpdated = true;
        update.realtimeCommand = follow.command;
        update.realtimeCommand.valid = true;
        update.realtimeCommand.onlineEndFlag = 1;
        update.realtimeCommand.message = "final online correction command";
    }
    return update;
}

OnlineCorrectionUpdate COnlineCorrectionController::onDetectionFinished(
    const robot_comm::RobotState &robotState)
{
    m_robotState = robotState;
    m_state.initialDetectionFinished = true;
    if (m_options.startMode != OnlineCorrectionStartMode::IBVSStart
        && m_state.initialDetectionSamples.empty()) {
        return makeUpdate(false, "initial detection points are empty");
    }
    if (m_options.startMode != OnlineCorrectionStartMode::IBVSStart
        && robotState.onlineReverseCompleteFlag == 0) {
        m_state.reverseCompletionArmed = true;
        setPhase(OnlineCorrectionPhase::WaitingForReverseCompletion,
                 "waiting for reverse detection completion");
        return makeUpdate(false, m_state.message);
    }

    m_state.reverseCompletionArmed = false;
    setPhase(OnlineCorrectionPhase::PreparingInitialTrajectory,
             "preparing initial correction trajectory");
    auto planResult = COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        m_trajectory,
        m_options,
        m_state,
        robotState,
        m_positionerCalibration);
    if (!planResult.valid) {
        setPhase(OnlineCorrectionPhase::Error, planResult.message);
        OnlineCorrectionUpdate update = makeUpdate(false, planResult.message);
        update.stopTracking = true;
        return update;
    }

    m_state.correctedSegmentPath = planResult.correctedSegmentPath;
    m_onlineTrajectory = correctedOnlineTrajectory(
        m_trajectory, m_options, planResult.correctedSegmentPath);
    m_state.initialPathMatch = planResult.initialPathMatch;
    m_state.teachPointSendIndex = 0;
    m_state.initialTeachPointCount = planResult.command.pointCount;
    m_state.trackerActive = false;
    setPhase(OnlineCorrectionPhase::SendingInitialTeachPoints,
             "initial teach points are ready to send");
    OnlineCorrectionUpdate update = makeUpdate(true, planResult.message);
    update.command = std::move(planResult.command);
    update.stopTracking = true;
    return update;
}

OnlineCorrectionUpdate COnlineCorrectionController::onRobotStateChanged(
    const robot_comm::RobotState &robotState)
{
    const bool robotProgramStopped =
        m_robotState.sStart != 0 && robotState.sStart == 0;
    m_robotState = robotState;
    if (robotState.sIsAlarm != 0 || robotState.sHold != 0) {
        setPhase(OnlineCorrectionPhase::Error, "robot alarm or hold is active");
        OnlineCorrectionUpdate update = makeUpdate(false, m_state.message);
        update.stopTracking = true;
        return update;
    }
    const bool sessionIsActive =
        m_state.phase != OnlineCorrectionPhase::Idle
        && m_state.phase != OnlineCorrectionPhase::Completed
        && m_state.phase != OnlineCorrectionPhase::Error;
    const bool finalSegmentIsEnding =
        m_state.pendingSegmentEndKind
            == OnlineCorrectionSegmentEndKind::Final
        && (m_state.phase == OnlineCorrectionPhase::SegmentFinishing
            || m_state.phase
                == OnlineCorrectionPhase::WaitingForDetectionReset);
    const bool robotProgramStopIsExpected =
        m_state.phase == OnlineCorrectionPhase::WaitingForRobotProgramStop
        || finalSegmentIsEnding;
    if (robotProgramStopped && sessionIsActive
        && !robotProgramStopIsExpected) {
        setPhase(
            OnlineCorrectionPhase::Error,
            "robot program stopped before online correction completed");
        OnlineCorrectionUpdate update = makeUpdate(false, m_state.message);
        update.stopTracking = true;
        return update;
    }
    if (robotState.onlineReverseCompleteFlag == 0) {
        m_state.reverseCompletionArmed = true;
    }
    if (m_state.phase == OnlineCorrectionPhase::WaitingForReverseCompletion
        && m_state.reverseCompletionArmed
        && robotState.onlineReverseCompleteFlag != 0) {
        return onDetectionFinished(robotState);
    }
    if (m_state.phase == OnlineCorrectionPhase::InitialDetectionRunning
        && m_state.reverseCompletionArmed
        && robotState.onlineReverseCompleteFlag != 0) {
        return onDetectionFinished(robotState);
    }
    if (m_state.phase == OnlineCorrectionPhase::WaitingInitialTeachPointAck
        && m_state.initialTeachPointCount > 0
        && m_state.teachPointSendIndex >= m_state.initialTeachPointCount
        && robotState.offline == 5) {
        m_state.trackerActive = false;
        setPhase(OnlineCorrectionPhase::WaitingForInitialApproach,
                 "teach points confirmed; waiting for robot to reach P001");
        return makeUpdate(true, m_state.message);
    }
    // 当前在线 JBI 在 MOVL P001 后和 SMOVC 结束后都会调用 ENDDET。
    // 只有先观察到 STARTCOR，后续 ENDDET 才具有“在线段结束”语义。
    const bool onlineStartObserved =
        robotState.corflag == 1 && robotState.detflag == 1;
    const bool preOnlineDetectionEndObserved =
        (robotState.corflag == 2 && robotState.detflag == 2)
        || (robotState.corflag == 3 && robotState.detflag == 3);
    if (m_state.phase == OnlineCorrectionPhase::WaitingForInitialApproach) {
        if (preOnlineDetectionEndObserved) {
            setPhase(OnlineCorrectionPhase::WaitingForOnlineStart,
                     "P001 approach completed; waiting for STARTCOR");
            return makeUpdate(true, m_state.message);
        }
    }
    if (m_state.phase == OnlineCorrectionPhase::WaitingForOnlineStart
        && onlineStartObserved) {
        m_state.trackerActive = true;
        setPhase(OnlineCorrectionPhase::OnlineRunning,
                 "STARTCOR observed; online correction started");
        OnlineCorrectionUpdate update = makeUpdate(true, m_state.message);
        update.startTracking = true;
        return update;
    }
    if (m_state.phase == OnlineCorrectionPhase::OnlineRunning
        && !m_state.segmentEndLatched) {
        if (robotState.corflag == 2 && robotState.detflag == 2) {
            m_state.segmentEndLatched = true;
            return finishOnlineSegment(
                OnlineCorrectionSegmentEndKind::Intermediate);
        }
        if (robotState.corflag == 3 && robotState.detflag == 3) {
            m_state.segmentEndLatched = true;
            return finishOnlineSegment(
                OnlineCorrectionSegmentEndKind::Final);
        }
    }
    if (m_state.phase == OnlineCorrectionPhase::WaitingForDetectionReset
        && robotState.detflag == 0) {
        m_state.segmentEndLatched = false;
        m_state.trackerActive = false;
        if (m_state.pendingSegmentEndKind
            == OnlineCorrectionSegmentEndKind::Intermediate) {
            ++m_state.segmentIndex;
            m_state.pendingSegmentEndKind.reset();
            setPhase(OnlineCorrectionPhase::Idle, "loading next online segment");
            OnlineCorrectionUpdate update = makeUpdate(true, m_state.message);
            update.requestNextSegment = true;
            return update;
        }
        m_state.pendingSegmentEndKind.reset();
        // 最终 ENDDET 只关闭在线检测窗口；退出段仍可能继续运动。保持会话和
        // 实际轨迹记录，直到控制柜明确报告主程序停止。
        if (robotState.sStart == 0) {
            setPhase(OnlineCorrectionPhase::Completed,
                     "online correction completed after robot program stopped");
            return makeUpdate(true, m_state.message);
        }
        setPhase(OnlineCorrectionPhase::WaitingForRobotProgramStop,
                 "online correction ended; waiting for robot program to stop");
        return makeUpdate(true, m_state.message);
    }
    if (m_state.phase == OnlineCorrectionPhase::WaitingForRobotProgramStop
        && robotState.sStart == 0) {
        setPhase(OnlineCorrectionPhase::Completed,
                 "online correction completed after robot program stopped");
        return makeUpdate(true, m_state.message);
    }
    return makeUpdate(true, "robot state accepted");
}

OnlineCorrectionUpdate COnlineCorrectionController::onRobotCommandReceipt(
    const robot_comm::RobotCommandReceipt &receipt,
    const robot_comm::RobotState &robotState)
{
    m_robotState = robotState;
    if (receipt.type != robot_comm::RobotCommandType::TeachPointUpdate
        && receipt.type != robot_comm::RobotCommandType::RealtimeCorrection) {
        return makeUpdate(true, "robot status receipt ignored");
    }
    if (!receipt.sent || !receipt.responseReceived
        || receipt.error != robot_comm::RobotCommandError::None) {
        const std::string message =
            receipt.message.empty() ? "robot command failed" : receipt.message;
        setPhase(OnlineCorrectionPhase::Error, message);
        OnlineCorrectionUpdate update = makeUpdate(false, m_state.message);
        update.stopTracking = true;
        return update;
    }

    if (receipt.type == robot_comm::RobotCommandType::TeachPointUpdate) {
        m_state.teachPointSendIndex = std::max(
            m_state.teachPointSendIndex, receipt.pointIndex + 1);
        setPhase(OnlineCorrectionPhase::WaitingInitialTeachPointAck,
                 "waiting for teach point update acknowledgement");
        const bool finalTeachPointReceived =
            m_state.initialTeachPointCount > 0
            && receipt.pointIndex + 1 >= m_state.initialTeachPointCount;
        if (finalTeachPointReceived && receipt.completed && robotState.offline == 5) {
            m_state.trackerActive = false;
            setPhase(OnlineCorrectionPhase::WaitingForInitialApproach,
                     "teach points confirmed; waiting for robot to reach P001");
            return makeUpdate(true, m_state.message);
        }
    }
    if (receipt.type == robot_comm::RobotCommandType::RealtimeCorrection
        && m_state.phase == OnlineCorrectionPhase::SegmentFinishing) {
        m_state.trackerActive = false;
        setPhase(
            OnlineCorrectionPhase::WaitingForDetectionReset,
            "segment end acknowledged; waiting for detflag reset");
        OnlineCorrectionUpdate update = makeUpdate(true, m_state.message);
        update.stopTracking = true;
        return update;
    }
    return makeUpdate(true, receipt.message);
}

OnlineCorrectionUpdate COnlineCorrectionController::finishOnlineSegment(
    OnlineCorrectionSegmentEndKind kind)
{
    if (m_state.phase != OnlineCorrectionPhase::OnlineRunning) {
        return makeUpdate(false, "online correction is not running");
    }
    m_state.pendingSegmentEndKind = kind;
    const auto tail = processOnlineSamples(
        m_exceptionProcessor, m_options, m_onlineTrajectory, {}, true);
    updateIcpStatus(m_state, m_options.enableOnlineIcp, tail);
    m_state.onlineDetectionSamples.insert(
        m_state.onlineDetectionSamples.end(), tail.samples.begin(), tail.samples.end());
    m_state.segmentEndLatched = true;
    m_state.trackerActive = false;
    setPhase(OnlineCorrectionPhase::SegmentFinishing, "online segment finish requested");
    OnlineCorrectionUpdate update = makeUpdate(true, m_state.message);
    update.onlineIcpUpdated = true;
    update.realtimeCommand.valid = true;
    update.realtimeCommand.speedOverride = 10000;
    update.realtimeCommand.onlineEndFlag = 1;
    update.realtimeCommand.message =
        kind == OnlineCorrectionSegmentEndKind::Intermediate
        ? "intermediate segment final correction command"
        : "final online correction command";
    // ENDDET 已关闭本段检测窗口，立即停止 Vision，避免迟到帧生成普通 P 并
    // 覆盖已排队的 Online_endflag=1 收尾命令。
    update.stopTracking = true;
    return update;
}

OnlineCorrectionUpdate COnlineCorrectionController::finishOnlineSegment()
{
    return finishOnlineSegment(OnlineCorrectionSegmentEndKind::Final);
}

OnlineCorrectionUpdate COnlineCorrectionController::fail(
    const std::string &message)
{
    m_state.trackerActive = false;
    setPhase(OnlineCorrectionPhase::Error, message);
    OnlineCorrectionUpdate update = makeUpdate(false, m_state.message);
    update.stopTracking = true;
    return update;
}

OnlineCorrectionUpdate COnlineCorrectionController::reset()
{
    const bool trajectoryInitialized = m_trajectory.valid;
    const bool calibrationInitialized = usableCalibration(m_positionerCalibration);
    m_state = OnlineCorrectionSessionState{};
    m_state.startMode = m_options.startMode;
    m_state.onlineIcp.enabled = m_options.enableOnlineIcp;
    m_onlineTrajectory = m_trajectory;
    m_state.onlineTeachTrajectoryInitialized = trajectoryInitialized;
    m_state.positionerCalibrationInitialized = calibrationInitialized;
    m_exceptionProcessor.reset();
    m_pathFollower.reset();
    setPhase(OnlineCorrectionPhase::Idle, "online correction reset");
    return makeUpdate(true, m_state.message);
}

const OnlineCorrectionSessionState &COnlineCorrectionController::state() const
{
    return m_state;
}

OnlineCorrectionUpdate COnlineCorrectionController::makeUpdate(
    bool succeeded,
    const std::string &message) const
{
    OnlineCorrectionUpdate update;
    update.succeeded = succeeded;
    update.phase = m_state.phase;
    update.state = m_state;
    update.message = message;
    return update;
}

void COnlineCorrectionController::setPhase(
    OnlineCorrectionPhase phase,
    const std::string &message)
{
    m_state.phase = phase;
    m_state.message = message;
}

} // namespace i2r_rlc::control
