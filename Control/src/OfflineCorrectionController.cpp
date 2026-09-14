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
 * @file        OfflineCorrectionController.cpp
 * @brief       离线纠偏控制器实现
 * @details     将旧按钮和计时器流程拆为显式状态转换，供 UI 和测试调用。
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
#include "OfflineCorrectionController.h"

#include <utility>

namespace i2r_rlc::control {

void COfflineCorrectionController::configure(const OfflineCorrectionOptions &options)
{
    m_options = options;
    m_archive = OfflineCorrectionArchive{};
    m_lastCommand = robot_comm::RobotCorrectionCommand{};
    m_lastDiagnostics = OfflineCorrectionDiagnosticData{};
    m_state = OfflineCorrectionSessionState{};
    setPhase(OfflineCorrectionPhase::Idle, "offline correction configured");
}

OfflineCorrectionUpdate COfflineCorrectionController::selectArchiveFile(
    const std::string &archiveFilePath)
{
    m_archive = COfflineCorrectionTrajectoryMatcher::loadArchive(archiveFilePath);
    m_lastCommand = robot_comm::RobotCorrectionCommand{};
    m_lastDiagnostics = OfflineCorrectionDiagnosticData{};
    m_state = OfflineCorrectionSessionState{};
    m_state.archiveFilePath = archiveFilePath;
    if (!m_archive.valid) {
        return makeError(m_archive.message);
    }

    m_state.modelFilePath = m_archive.modelFilePath;
    m_state.teachFilePath = m_archive.teachFilePath;
    setPhase(OfflineCorrectionPhase::ArchiveSelected, m_archive.message);
    return makeUpdate(true, m_archive.message);
}

OfflineCorrectionUpdate COfflineCorrectionController::startDetection()
{
    if (!m_archive.valid) {
        return makeError("请先选择离线纠偏存档文件。");
    }

    m_state.detectionSamples.clear();
    m_state.report = OfflineCorrectionMatchReport{};
    m_lastCommand = robot_comm::RobotCorrectionCommand{};
    m_lastDiagnostics = OfflineCorrectionDiagnosticData{};
    setPhase(OfflineCorrectionPhase::WaitingForDetection, "offline correction waiting for detection samples");
    return makeUpdate(true, "离线纠偏等待检测点输入。");
}

OfflineCorrectionUpdate COfflineCorrectionController::appendDetectionEvent(
    const OfflineCorrectionDetectionEvent &event)
{
    if (m_state.phase != OfflineCorrectionPhase::WaitingForDetection
        && m_state.phase != OfflineCorrectionPhase::Detecting) {
        return makeError("离线纠偏当前阶段不接收检测点。");
    }

    int acceptedCount = 0;
    for (const auto &sample : event.samples) {
        if (!sample.point.valid) {
            continue;
        }
        m_state.detectionSamples.push_back(sample);
        ++acceptedCount;
    }

    if (m_state.detectionSamples.empty()) {
        setPhase(OfflineCorrectionPhase::WaitingForDetection, "waiting for valid offline detection samples");
        return makeUpdate(false, "离线纠偏检测事件未包含有效空间点。");
    }

    setPhase(OfflineCorrectionPhase::Detecting, "offline correction detection samples accepted");
    return makeUpdate(
        true,
        "离线纠偏本帧接收=" + std::to_string(acceptedCount)
            + "，累计=" + std::to_string(m_state.detectionSamples.size()) + "。");
}

OfflineCorrectionUpdate COfflineCorrectionController::finishDetection()
{
    if (m_state.detectionSamples.empty()) {
        return makeError("离线纠偏检测点为空，无法结束检测。");
    }

    setPhase(OfflineCorrectionPhase::ReadyToCompute, "offline correction detection finished");
    return makeUpdate(
        true,
        "离线纠偏焊缝采集完成，累计="
            + std::to_string(m_state.detectionSamples.size()) + "，开始点清洗和 ICP。");
}

OfflineCorrectionUpdate COfflineCorrectionController::computeCorrection()
{
    if (!m_archive.valid) {
        return makeError("离线纠偏存档无效，无法计算。");
    }
    if (m_state.detectionSamples.empty()) {
        return makeError("离线纠偏检测点为空，无法计算。");
    }

    setPhase(OfflineCorrectionPhase::Computing, "offline correction computing");
    auto matchResult = COfflineCorrectionTrajectoryMatcher::match(
        m_archive,
        m_state.detectionSamples,
        m_options);
    if (!matchResult.valid) {
        return makeError(matchResult.message);
    }

    m_state.report = matchResult.report;
    m_lastCommand = matchResult.robotCommand;
    m_lastDiagnostics = std::move(matchResult.diagnostics);
    setPhase(OfflineCorrectionPhase::ReadyToSend, matchResult.message);
    return makeUpdate(true, matchResult.message);
}

OfflineCorrectionUpdate COfflineCorrectionController::markCommandSent()
{
    if (!m_lastCommand.valid) {
        return makeError("离线纠偏没有可发送的机器人命令。");
    }

    setPhase(OfflineCorrectionPhase::Completed, "offline correction command marked as sent");
    return makeUpdate(true, "离线纠偏示教点更新命令已确认。");
}

OfflineCorrectionUpdate COfflineCorrectionController::markCommandDispatched()
{
    if (!m_lastCommand.valid) {
        return makeError("离线纠偏没有可发送的机器人命令。");
    }

    setPhase(OfflineCorrectionPhase::WaitingForRobotAck,
             "offline correction waiting for D.offline==5");
    return makeUpdate(true, "离线纠偏 O 命令已发送，等待 D.offline==5。");
}

OfflineCorrectionUpdate COfflineCorrectionController::fail(const std::string &message)
{
    return makeError(message);
}

OfflineCorrectionUpdate COfflineCorrectionController::reset()
{
    m_archive = OfflineCorrectionArchive{};
    m_lastCommand = robot_comm::RobotCorrectionCommand{};
    m_lastDiagnostics = OfflineCorrectionDiagnosticData{};
    m_state = OfflineCorrectionSessionState{};
    setPhase(OfflineCorrectionPhase::Idle, "offline correction reset");
    return makeUpdate(true, "离线纠偏已重置。");
}

const OfflineCorrectionSessionState &COfflineCorrectionController::state() const
{
    return m_state;
}

const OfflineCorrectionDiagnosticData &COfflineCorrectionController::diagnostics() const
{
    return m_lastDiagnostics;
}

OfflineCorrectionUpdate COfflineCorrectionController::makeUpdate(
    const bool succeeded,
    const std::string &message) const
{
    OfflineCorrectionUpdate update;
    update.succeeded = succeeded;
    update.phase = m_state.phase;
    update.state = m_state;
    update.robotCommand = m_lastCommand;
    update.message = message;
    return update;
}

OfflineCorrectionUpdate COfflineCorrectionController::makeError(const std::string &message)
{
    m_lastCommand = robot_comm::RobotCorrectionCommand{};
    m_lastDiagnostics = OfflineCorrectionDiagnosticData{};
    setPhase(OfflineCorrectionPhase::Error, message);
    return makeUpdate(false, message);
}

void COfflineCorrectionController::setPhase(
    const OfflineCorrectionPhase phase,
    const std::string &message)
{
    m_state.phase = phase;
    m_state.message = message;
}

} // namespace i2r_rlc::control
