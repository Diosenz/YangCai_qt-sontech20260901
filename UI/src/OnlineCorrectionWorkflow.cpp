/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionWorkflow.cpp
 * @brief       在线纠偏启动计划与阶段状态机实现
 ******************************************************************************/
#include "OnlineCorrectionWorkflow.h"

namespace i2r_rlc::ui {

bool COnlineCorrectionWorkflow::begin(
    control::OnlineCorrectionStartMode mode,
    const QString& globalLocateJbi,
    const QString& onlineCorrectionJbi,
    QString* errorMessage)
{
    if (m_state != OnlineCorrectionWorkflowState::Idle) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("在线纠偏流程尚未复位。");
        }
        return false;
    }

    const QString normalizedOnlineJbi = onlineCorrectionJbi.trimmed();
    if (normalizedOnlineJbi.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("当前起始模式未配置在线 JBI。");
        }
        return false;
    }

    bool requiresGlobalLocalization = false;
    switch (mode) {
    case control::OnlineCorrectionStartMode::InitType:
    case control::OnlineCorrectionStartMode::IBVSStart:
        break;
    case control::OnlineCorrectionStartMode::GlobalCorrection:
        requiresGlobalLocalization = true;
        break;
    default:
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("不支持的在线纠偏起始模式。");
        }
        return false;
    }
    const QString normalizedGlobalLocateJbi = globalLocateJbi.trimmed();
    if (requiresGlobalLocalization && normalizedGlobalLocateJbi.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("当前工件未配置全局定位 JBI。");
        }
        return false;
    }

    m_plan.mode = mode;
    m_plan.requiresGlobalLocalization = requiresGlobalLocalization;
    m_plan.globalLocateJbi = normalizedGlobalLocateJbi;
    m_plan.onlineCorrectionJbi = normalizedOnlineJbi;
    m_state = requiresGlobalLocalization
        ? OnlineCorrectionWorkflowState::WaitingGlobalLocateStartGate
        : OnlineCorrectionWorkflowState::WaitingOnlineStartGate;
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

void COnlineCorrectionWorkflow::reset()
{
    m_state = OnlineCorrectionWorkflowState::Idle;
    m_plan = {};
}

void COnlineCorrectionWorkflow::fail()
{
    m_state = OnlineCorrectionWorkflowState::Failed;
}

OnlineCorrectionWorkflowState COnlineCorrectionWorkflow::state() const
{
    return m_state;
}

const OnlineCorrectionStartPlan& COnlineCorrectionWorkflow::plan() const
{
    return m_plan;
}

bool COnlineCorrectionWorkflow::acceptsGlobalLocalizationFrame() const
{
    return m_state == OnlineCorrectionWorkflowState::GlobalLocateJobRunning
        || m_state == OnlineCorrectionWorkflowState::DrainingGlobalLocalization;
}

bool COnlineCorrectionWorkflow::markGlobalLocateStarted()
{
    return transition(
        OnlineCorrectionWorkflowState::WaitingGlobalLocateStartGate,
        OnlineCorrectionWorkflowState::GlobalLocateJobRunning);
}

bool COnlineCorrectionWorkflow::markGlobalLocateJobCompleted()
{
    return transition(
        OnlineCorrectionWorkflowState::GlobalLocateJobRunning,
        OnlineCorrectionWorkflowState::DrainingGlobalLocalization);
}

bool COnlineCorrectionWorkflow::markGlobalLocalizationDrained()
{
    return transition(
        OnlineCorrectionWorkflowState::DrainingGlobalLocalization,
        OnlineCorrectionWorkflowState::ApplyingGlobalLocalization);
}

bool COnlineCorrectionWorkflow::markGlobalLocalizationApplied()
{
    return transition(
        OnlineCorrectionWorkflowState::ApplyingGlobalLocalization,
        OnlineCorrectionWorkflowState::WaitingOnlineStartGate);
}

bool COnlineCorrectionWorkflow::markOnlineCorrectionStarted()
{
    return transition(
        OnlineCorrectionWorkflowState::WaitingOnlineStartGate,
        OnlineCorrectionWorkflowState::OnlineCorrectionRunning);
}

bool COnlineCorrectionWorkflow::markOnlineCorrectionCompleted()
{
    return transition(
        OnlineCorrectionWorkflowState::OnlineCorrectionRunning,
        OnlineCorrectionWorkflowState::Completed);
}

bool COnlineCorrectionWorkflow::transition(
    OnlineCorrectionWorkflowState expected,
    OnlineCorrectionWorkflowState next)
{
    if (m_state != expected) {
        return false;
    }
    m_state = next;
    return true;
}

} // namespace i2r_rlc::ui
