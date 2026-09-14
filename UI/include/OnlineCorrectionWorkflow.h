/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionWorkflow.h
 * @brief       在线纠偏启动计划与阶段状态机
 ******************************************************************************/
#ifndef I2R_RLC_UI_ONLINE_CORRECTION_WORKFLOW_H
#define I2R_RLC_UI_ONLINE_CORRECTION_WORKFLOW_H

#include <QString>

#include "OnlineCorrectionTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief 在线纠偏跨 JBI 阶段状态。
 *
 * 状态机只表达流程约束，不访问 QWidget、设备或运行时线程。UiController 负责
 * 把状态转换映射为机器人门禁、视觉跟踪和界面更新。
 */
enum class OnlineCorrectionWorkflowState {
    Idle,
    WaitingGlobalLocateStartGate,
    GlobalLocateJobRunning,
    DrainingGlobalLocalization,
    ApplyingGlobalLocalization,
    WaitingOnlineStartGate,
    OnlineCorrectionRunning,
    Failed,
    Completed
};

/**
 * @brief 一次在线纠偏启动请求解析出的不可变计划。
 */
struct OnlineCorrectionStartPlan {
    control::OnlineCorrectionStartMode mode =
        control::OnlineCorrectionStartMode::InitType;
    bool requiresGlobalLocalization = false;
    QString globalLocateJbi;
    QString onlineCorrectionJbi;
};

/**
 * @brief 在线纠偏工作流状态机。
 *
 * 三种模式的阶段差异集中在本类：反向扫描和 IBVS 直接等待在线 JBI；仅
 * GlobalCorrection 先执行全局定位、排空在途视觉帧并应用修正，再进入在线 JBI。
 */
class COnlineCorrectionWorkflow final {
public:
    /**
     * @brief 创建并启动一轮工作流计划。
     * @param mode 用户选择的在线纠偏起始模式。
     * @param globalLocateJbi 全局定位 JBI；仅 GlobalCorrection 必填。
     * @param onlineCorrectionJbi 对应模式的在线 JBI，所有模式必填。
     * @param errorMessage 失败时写入可展示的原因。
     */
    bool begin(
        control::OnlineCorrectionStartMode mode,
        const QString& globalLocateJbi,
        const QString& onlineCorrectionJbi,
        QString* errorMessage);

    /// @brief 清除本轮计划并回到空闲状态。
    void reset();
    /// @brief 将当前流程置为失败；保留计划供诊断显示。
    void fail();
    /// @brief 返回当前工作流状态。
    OnlineCorrectionWorkflowState state() const;
    /// @brief 返回本轮冻结的启动计划。
    const OnlineCorrectionStartPlan& plan() const;
    /// @brief 当前阶段是否仍可接收全局定位真实/虚拟帧。
    bool acceptsGlobalLocalizationFrame() const;

    /// @brief 门禁通过并启动全局定位 JBI。
    bool markGlobalLocateStarted();
    /// @brief 全局定位 JBI 已结束，进入在途帧排空阶段。
    bool markGlobalLocateJobCompleted();
    /// @brief 在途视觉帧已排空，可以执行全局定位。
    bool markGlobalLocalizationDrained();
    /// @brief 修正矩阵已应用，进入在线 JBI 门禁阶段。
    bool markGlobalLocalizationApplied();
    /// @brief 在线 JBI 已启动。
    bool markOnlineCorrectionStarted();
    /// @brief 在线纠偏已完成。
    bool markOnlineCorrectionCompleted();

private:
    bool transition(
        OnlineCorrectionWorkflowState expected,
        OnlineCorrectionWorkflowState next);

    OnlineCorrectionWorkflowState m_state =
        OnlineCorrectionWorkflowState::Idle;
    OnlineCorrectionStartPlan m_plan;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_CORRECTION_WORKFLOW_H
