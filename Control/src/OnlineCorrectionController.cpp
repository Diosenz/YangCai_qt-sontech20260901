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
#include <limits>
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

struct ReferenceProjection {
    bool valid = false;
    int segmentIndex = 0;
    OnlineCorrectionPoint point;
    double distanceSquared = std::numeric_limits<double>::max();
};

/**
 * @brief 从既有路径进度向相邻段递进求投影，避免在交叉/近邻支系全局跳转。
 * @details 可沿任意轴向及前后相邻段移动；只按几何下降选择，不设置固定窗口或行程上限。
 */
ReferenceProjection projectToControlReference(
    const std::vector<OnlineCorrectionPoint> &path,
    const OnlineCorrectionPoint &point,
    const OnlineCorrectionPoint &offset,
    int previousSegment)
{
    ReferenceProjection result;
    if (path.size() < 2U || !finitePoint(point) || !finitePoint(offset)) { return result; }
    const int segmentCount = static_cast<int>(path.size()) - 1;
    const Eigen::Vector3d current(point.xMm, point.yMm, point.zMm);
    const Eigen::Vector3d translation(offset.xMm, offset.yMm, offset.zMm);
    const auto evaluate = [&](int index) {
        ReferenceProjection candidate;
        if (index < 0 || index >= segmentCount
            || !finitePoint(path[index]) || !finitePoint(path[index + 1])) { return candidate; }
        const Eigen::Vector3d first = Eigen::Vector3d(
            path[index].xMm, path[index].yMm, path[index].zMm) + translation;
        const Eigen::Vector3d direction(path[index + 1].xMm - path[index].xMm,
            path[index + 1].yMm - path[index].yMm, path[index + 1].zMm - path[index].zMm);
        const double lengthSquared = direction.squaredNorm();
        if (lengthSquared <= 1.0e-12) { return candidate; }
        const double fraction = std::clamp(direction.dot(current - first) / lengthSquared, 0.0, 1.0);
        const Eigen::Vector3d projected = first + fraction * direction;
        candidate.valid = projected.allFinite();
        candidate.segmentIndex = index;
        candidate.point = {projected.x(), projected.y(), projected.z(), candidate.valid};
        candidate.distanceSquared = (current - projected).squaredNorm();
        return candidate;
    };
    result = evaluate(std::clamp(previousSegment, 0, segmentCount - 1));
    if (!result.valid) {
        // 重复顶点可以跳过，但不能借此绕过一个有效的拓扑相邻段。
        for (int radius = 1; radius < segmentCount && !result.valid; ++radius) {
            const auto left = evaluate(previousSegment - radius);
            const auto right = evaluate(previousSegment + radius);
            if (left.valid) { result = left; }
            if (right.valid && (!result.valid || right.distanceSquared < result.distanceSquared)) { result = right; }
        }
    }
    if (!result.valid) { return result; }
    for (int iteration = 0; iteration < segmentCount; ++iteration) {
        auto best = result;
        for (const int direction : {-1, 1}) {
            for (int index = result.segmentIndex + direction;
                 index >= 0 && index < segmentCount; index += direction) {
                const auto candidate = evaluate(index);
                if (!candidate.valid) { continue; }
                if (candidate.distanceSquared < best.distanceSquared - 1.0e-12) { best = candidate; }
                break;
            }
        }
        if (best.segmentIndex == result.segmentIndex) { break; }
        result = best;
    }
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
        auto result = processor.process(samples, trajectory, flush);
        if (options.preserveMeasuredReference) {
            result.samples.clear();
            result.icpApplied = false;
            for (auto &batch : result.batches) {
                if (batch.applied) { batch.modelProposalSamples = batch.releasedSamples; }
                batch.releasedSamples = batch.pendingSamples;
                batch.applied = false;
                batch.message = "measured-reference policy: raw samples released, model proposal diagnostic only; candidate: "
                    + batch.message;
                result.samples.insert(result.samples.end(),
                    batch.releasedSamples.begin(), batch.releasedSamples.end());
            }
            result.message = "measured-reference policy preserves measured positions; candidate: "
                + result.message;
        }
        return result;
    }
    OnlineCorrectionExceptionResult result;
    result.valid = true;
    result.samples = samples;
    result.message = "online ICP disabled; raw detection samples used";
    if (!samples.empty()) {
        OnlineCorrectionIcpBatchDiagnostic diagnostic;
        diagnostic.enabled = false;
        diagnostic.flush = flush;
        diagnostic.pendingSamples = samples;
        diagnostic.releasedSamples = samples;
        diagnostic.message = result.message;
        result.batches.push_back(std::move(diagnostic));
    }
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
    m_predictionReferenceReady = false;
    m_predictionOffset = {};
    m_predictionGapWidthMm.reset();
    m_predictionSegmentIndex = 0;
    m_targetGeneration = 0;
    m_hasControlRobotState = false;
    m_state.controlTarget = {};

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
    m_predictionReferenceReady = false;
    m_predictionOffset = {};
    m_predictionGapWidthMm.reset();
    m_predictionSegmentIndex = 0;
    m_targetGeneration = 0;
    m_hasControlRobotState = false;
    m_state.controlTarget = {};
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
        if (!finitePoint(sample.point)
            || (sample.estimated && (initialPhase || m_options.preserveMeasuredReference))) {
            // 初始 O 点和实测优先策略均只接纳独立实测；legacy 估计行为另行保留。
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

    if (accepted.empty() && !event.detectionFinished) {
        // 拒绝的原图不触发控制历史；缺测续行由有效机器人反馈重算。
        return makeUpdate(true, "no accepted image measurement");
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
    // 传统模式等待匹配验收；实测优先模式只用实际释放的独立实测更新参考。
    if (!m_options.enableOnlineIcp || m_options.preserveMeasuredReference) {
        updatePredictionAnchor(processed.samples);
    } else {
        for (const auto &batch : processed.batches) {
            if (batch.applied) { updatePredictionAnchor(batch.releasedSamples); }
        }
    }
    if (!processed.samples.empty()) {
        m_state.simulatedSeamIndex = processed.samples.back().simulatedSeamIndex;
    }

    const auto follow = followCurrentTarget();

    OnlineCorrectionUpdate update = makeUpdate(follow.valid, follow.message);
    update.onlineIcpUpdated = true;
    update.onlineIcpBatches = processed.batches;
    update.followDiagnostic = follow.diagnostic;
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
        update.onlineIcpBatches = processed.batches;
        update.followDiagnostic = follow.diagnostic;
        update.taughtPointIndex = follow.taughtPointIndex;
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
    m_predictionReferenceReady = m_onlineTrajectory.segmentPathPoints.size() >= 2U;
    m_predictionOffset = {0.0, 0.0, 0.0, true};
    m_predictionSegmentIndex = 0;
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
    // P 回包也携带 RobotState；相同控制位姿不能触发 P→D→P 的静止自激循环。
    // 精确变化检测没有附加运动阈值，新视觉仍可通过 appendDetectionEvent 更新目标。
    const bool controlFeedbackChanged = !m_hasControlRobotState
        || m_lastControlRobotState.curx_r != robotState.curx_r
        || m_lastControlRobotState.cury_r != robotState.cury_r || m_lastControlRobotState.curz_r != robotState.curz_r
        || m_lastControlRobotState.curRx_r != robotState.curRx_r || m_lastControlRobotState.curRy_r != robotState.curRy_r
        || m_lastControlRobotState.curRz_r != robotState.curRz_r || m_lastControlRobotState.curS1 != robotState.curS1
        || m_lastControlRobotState.curS2 != robotState.curS2
        || m_lastControlRobotState.cur_interpolation != robotState.cur_interpolation;
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
        if (!onlineStartObserved) {
            // 新增反馈驱动入口只服务 STARTCOR；不在窗口外消费仍排队的实测点。
            m_state.controlTarget.valid = false;
            m_state.controlTarget.source = OnlineCorrectionTargetSource::Unavailable;
            return makeUpdate(true, "waiting for STARTCOR state");
        }
        if (!controlFeedbackChanged && m_state.controlTarget.valid) {
            return makeUpdate(true, "robot state accepted");
        }
        const auto follow = followCurrentTarget();
        OnlineCorrectionUpdate update = makeUpdate(true,
            follow.valid ? follow.message : "robot state accepted");
        if (follow.valid) {
            update.followDiagnostic = follow.diagnostic;
            update.taughtPointIndex = follow.taughtPointIndex;
            update.realtimeCommand = follow.command;
        }
        return update;
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
    update.onlineIcpBatches = tail.batches;
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
    m_predictionReferenceReady = false;
    m_predictionOffset = {};
    m_predictionGapWidthMm.reset();
    m_predictionSegmentIndex = 0;
    m_targetGeneration = 0;
    m_hasControlRobotState = false;
    m_state.controlTarget = {};
    setPhase(OnlineCorrectionPhase::Idle, "online correction reset");
    return makeUpdate(true, m_state.message);
}

const OnlineCorrectionSessionState &COnlineCorrectionController::state() const
{
    return m_state;
}

void COnlineCorrectionController::recordControlTarget(
    const OnlineCorrectionFollowResult &follow, OnlineCorrectionTargetSource source)
{
    if (!follow.valid || !follow.command.valid || !follow.diagnostic.valid
        || !finitePoint(follow.diagnostic.futureFoot)) { return; }
    OnlineCorrectionTargetSnapshot target;
    target.valid = true;
    target.source = source;
    target.generation = ++m_targetGeneration;
    target.targetPointInLastLink_mm = follow.diagnostic.futureFoot;
    target.geometry = follow.diagnostic;
    target.taughtPointIndex = follow.taughtPointIndex;
    m_state.controlTarget = target;
    m_lastControlRobotState = m_robotState;
    m_hasControlRobotState = true;
}

void COnlineCorrectionController::updatePredictionAnchor(
    const std::vector<OnlineCorrectionDetectionSample> &samples)
{
    if (!m_predictionReferenceReady) { return; }
    for (const auto &sample : samples) {
        if (sample.estimated || !finitePoint(sample.point)) { continue; }
        const auto projection = projectToControlReference(m_onlineTrajectory.segmentPathPoints,
            sample.point, OnlineCorrectionPoint{0.0, 0.0, 0.0, true}, m_predictionSegmentIndex);
        if (!projection.valid) { continue; }
        m_predictionOffset = {sample.point.xMm - projection.point.xMm,
            sample.point.yMm - projection.point.yMm, sample.point.zMm - projection.point.zMm, true};
        m_predictionSegmentIndex = projection.segmentIndex;
        m_predictionGapWidthMm = sample.gapWidthMm;
    }
}

OnlineCorrectionFollowResult COnlineCorrectionController::followCurrentTarget()
{
    auto follow = m_pathFollower.update(m_robotState, m_options, m_positionerCalibration,
        m_onlineTrajectory.segmentPathPoints, &m_state.onlineDetectionSamples, false);
    if (follow.valid) {
        recordControlTarget(follow, OnlineCorrectionTargetSource::Measured);
        return follow;
    }
    const bool mayPredict = m_state.phase == OnlineCorrectionPhase::OnlineRunning
        && m_state.trackerActive && !m_state.segmentEndLatched
        && m_robotState.sStart != 0 && m_robotState.sHold == 0 && m_robotState.sIsAlarm == 0
        && m_robotState.corflag == 1 && m_robotState.detflag == 1
        && m_predictionReferenceReady;
    if (!mayPredict) {
        m_state.controlTarget.valid = false;
        m_state.controlTarget.source = OnlineCorrectionTargetSource::Unavailable;
        return follow;
    }
    const auto current = vision::CPositionerKinematics::robotPoseToStationPose(
        vision::PositionerPose{static_cast<double>(m_robotState.curx_r) / 1000.0,
            static_cast<double>(m_robotState.cury_r) / 1000.0,
            static_cast<double>(m_robotState.curz_r) / 1000.0,
            static_cast<double>(m_robotState.curRx_r) / 10000.0,
            static_cast<double>(m_robotState.curRy_r) / 10000.0,
            static_cast<double>(m_robotState.curRz_r) / 10000.0, true},
        static_cast<double>(m_robotState.curS1) / 10000.0,
        static_cast<double>(m_robotState.curS2) / 10000.0, m_positionerCalibration);
    const auto projection = projectToControlReference(m_onlineTrajectory.segmentPathPoints,
        OnlineCorrectionPoint{current.xMm, current.yMm, current.zMm, current.valid},
        m_predictionOffset, m_predictionSegmentIndex);
    if (!projection.valid) {
        m_state.controlTarget.valid = false;
        m_state.controlTarget.source = OnlineCorrectionTargetSource::Unavailable;
        return follow;
    }
    m_predictionSegmentIndex = projection.segmentIndex;
    const auto shifted = [this](OnlineCorrectionPoint point) {
        point.xMm += m_predictionOffset.xMm;
        point.yMm += m_predictionOffset.yMm;
        point.zMm += m_predictionOffset.zMm;
        return point;
    };
    const auto index = static_cast<std::size_t>(projection.segmentIndex);
    const OnlineCorrectionFollowReference reference{
        shifted(m_onlineTrajectory.segmentPathPoints[index]),
        shifted(m_onlineTrajectory.segmentPathPoints[index + 1U]), m_predictionGapWidthMm};
    follow = m_pathFollower.updateReference(m_robotState, m_options, m_positionerCalibration,
        m_onlineTrajectory.segmentPathPoints, reference);
    if (follow.valid) {
        follow.message = "predicted control reference recomputed from current robot feedback";
        follow.command.message = follow.message;
        recordControlTarget(follow, OnlineCorrectionTargetSource::PredictedReference);
    } else {
        m_state.controlTarget.valid = false;
        m_state.controlTarget.source = OnlineCorrectionTargetSource::Unavailable;
    }
    return follow;
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
    if (phase != OnlineCorrectionPhase::OnlineRunning) {
        // 阶段退出立即撤销当前目标资格，迟到反馈不得复活旧 P 几何。
        m_state.controlTarget.valid = false;
        m_state.controlTarget.source = OnlineCorrectionTargetSource::Unavailable;
    }
    m_state.phase = phase;
    m_state.message = message;
}

} // namespace i2r_rlc::control
