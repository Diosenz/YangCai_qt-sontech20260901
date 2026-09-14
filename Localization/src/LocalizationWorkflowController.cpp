/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationWorkflowController.cpp
 * @brief     定位应用层流程状态控制器实现。
 * @details   本文件实现 Localization 模块内局部修正和全局定位的流程状态转换。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 16 日
 */

#include "LocalizationWorkflowController.h"

namespace i2r_rlc::localization {

QString localizationWorkflowStateToText(const LocalizationWorkflowState state)
{
    switch (state) {
    case LocalizationWorkflowState::Idle:
        return QStringLiteral("空闲");
    case LocalizationWorkflowState::CameraStateAvailable:
        return QStringLiteral("相机状态可用");
    case LocalizationWorkflowState::RobotStateAvailable:
        return QStringLiteral("机器人状态可用");
    case LocalizationWorkflowState::MissingVirtualReference:
        return QStringLiteral("等待虚拟参考");
    case LocalizationWorkflowState::LocalCorrectionRunning:
        return QStringLiteral("局部修正运行中");
    case LocalizationWorkflowState::GlobalIcpRunning:
        return QStringLiteral("全局 ICP 运行中");
    case LocalizationWorkflowState::Succeeded:
        return QStringLiteral("成功");
    case LocalizationWorkflowState::Failed:
        return QStringLiteral("失败");
    case LocalizationWorkflowState::Stopping:
        return QStringLiteral("停止中");
    }

    return QStringLiteral("未知");
}

QString localizationWorkflowStateToKey(const LocalizationWorkflowState state)
{
    switch (state) {
    case LocalizationWorkflowState::Idle:
        return QStringLiteral("idle");
    case LocalizationWorkflowState::CameraStateAvailable:
        return QStringLiteral("camera_state_available");
    case LocalizationWorkflowState::RobotStateAvailable:
        return QStringLiteral("robot_state_available");
    case LocalizationWorkflowState::MissingVirtualReference:
        return QStringLiteral("missing_virtual_reference");
    case LocalizationWorkflowState::LocalCorrectionRunning:
        return QStringLiteral("local_correction_running");
    case LocalizationWorkflowState::GlobalIcpRunning:
        return QStringLiteral("global_icp_running");
    case LocalizationWorkflowState::Succeeded:
        return QStringLiteral("succeeded");
    case LocalizationWorkflowState::Failed:
        return QStringLiteral("failed");
    case LocalizationWorkflowState::Stopping:
        return QStringLiteral("stopping");
    }

    return QStringLiteral("unknown");
}

LocalizationWorkflowState LocalizationWorkflowController::state() const
{
    return m_state;
}

LocalizationWorkflowUpdate LocalizationWorkflowController::markCameraStateAvailable(
    const qint64 frameId)
{
    m_lastFrameId = frameId;
    return makeUpdate(
        LocalizationWorkflowState::CameraStateAvailable,
        LocalizationMode::Local,
        frameId,
        true,
        QStringLiteral("定位流程：相机状态可用。"));
}

LocalizationWorkflowUpdate LocalizationWorkflowController::markRobotStateAvailable()
{
    return makeUpdate(
        LocalizationWorkflowState::RobotStateAvailable,
        LocalizationMode::Global,
        m_lastFrameId,
        true,
        QStringLiteral("定位流程：机器人状态可用。"));
}

LocalizationWorkflowUpdate LocalizationWorkflowController::startLocalCorrection(
    const qint64 frameId,
    const IcpDofMask dofMask)
{
    m_lastFrameId = frameId;
    m_lastDofMask = dofMask;
    return makeUpdate(
        LocalizationWorkflowState::LocalCorrectionRunning,
        LocalizationMode::Local,
        frameId,
        true,
        QStringLiteral("定位流程：局部修正开始。"));
}

LocalizationWorkflowUpdate LocalizationWorkflowController::finishLocalCorrection(
    const LocalCorrectionPipelineResult& result)
{
    const bool succeeded = result.status == LocalizationStatus::Succeeded;
    LocalizationWorkflowUpdate update = makeUpdate(
        succeeded ? LocalizationWorkflowState::Succeeded : LocalizationWorkflowState::Failed,
        LocalizationMode::Local,
        result.icpResult.frameId,
        succeeded,
        succeeded ? QStringLiteral("定位流程：局部修正完成。")
                  : QStringLiteral("定位流程：局部修正失败。"),
        succeeded ? QString() : result.message);
    update.hasResidual = true;
    update.beforeResidual_mm = result.beforeResidual_mm;
    update.afterResidual_mm = result.afterResidual_mm;
    update.segmentCount = 1;
    update.dofMask = m_lastDofMask;
    return update;
}

LocalizationWorkflowUpdate LocalizationWorkflowController::startGlobalIcp(
    const IcpDofMask dofMask,
    const int segmentCount)
{
    m_lastDofMask = dofMask;
    LocalizationWorkflowUpdate update = makeUpdate(
        LocalizationWorkflowState::GlobalIcpRunning,
        LocalizationMode::Global,
        m_lastFrameId,
        true,
        QStringLiteral("定位流程：全局定位开始。"));
    update.segmentCount = segmentCount;
    return update;
}

LocalizationWorkflowUpdate LocalizationWorkflowController::finishGlobalIcp(
    const GlobalIcpPipelineResult& result)
{
    const bool succeeded = result.status == LocalizationStatus::Succeeded;
    const int segmentCount = static_cast<int>(result.segmentResults.size());
    LocalizationWorkflowUpdate update = makeUpdate(
        succeeded ? LocalizationWorkflowState::Succeeded : LocalizationWorkflowState::Failed,
        LocalizationMode::Global,
        m_lastFrameId,
        succeeded,
        succeeded ? QStringLiteral("定位流程：全局定位完成。")
                  : QStringLiteral("定位流程：全局定位失败。"),
        succeeded ? QString() : result.message);
    update.hasResidual = result.hasCorrectedVirtualPoints || segmentCount > 0;
    update.beforeResidual_mm = result.beforeResidual_mm;
    update.afterResidual_mm = result.afterResidual_mm;
    update.segmentCount = segmentCount;
    update.dofMask = m_lastDofMask;
    return update;
}

LocalizationWorkflowUpdate LocalizationWorkflowController::markMissingVirtualReference(
    const QString& message)
{
    return makeUpdate(
        LocalizationWorkflowState::MissingVirtualReference,
        LocalizationMode::Global,
        m_lastFrameId,
        false,
        QStringLiteral("定位流程：缺少虚拟参考数据。"),
        message);
}

LocalizationWorkflowUpdate LocalizationWorkflowController::stop()
{
    makeUpdate(
        LocalizationWorkflowState::Stopping,
        LocalizationMode::Global,
        m_lastFrameId,
        true,
        QStringLiteral("定位流程正在停止。"));
    return clear();
}

LocalizationWorkflowUpdate LocalizationWorkflowController::clear()
{
    m_lastFrameId = -1;
    return makeUpdate(
        LocalizationWorkflowState::Idle,
        LocalizationMode::Global,
        m_lastFrameId,
        true,
        QStringLiteral("定位流程状态已清除。"));
}

LocalizationWorkflowUpdate LocalizationWorkflowController::makeUpdate(
    const LocalizationWorkflowState state,
    const LocalizationMode mode,
    const qint64 frameId,
    const bool succeeded,
    const QString& message,
    const QString& errorMessage)
{
    m_state = state;

    LocalizationWorkflowUpdate update;
    update.state = state;
    update.mode = mode;
    update.frameId = frameId;
    update.succeeded = succeeded;
    update.message = message;
    update.errorMessage = errorMessage;
    update.dofMask = m_lastDofMask;
    return update;
}

} // namespace i2r_rlc::localization
