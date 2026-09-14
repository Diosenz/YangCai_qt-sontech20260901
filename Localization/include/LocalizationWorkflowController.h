/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationWorkflowController.h
 * @brief     定位应用层流程状态控制器。
 * @details   本文件定义归属 Localization 模块的定位流程状态机，供 UI 或 worker
 *            编排局部修正和全局定位使用。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 16 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_WORKFLOW_CONTROLLER_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_WORKFLOW_CONTROLLER_H

#include <QString>

#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 定位流程状态。
 *
 * 该状态只描述 Localization 模块内的应用层编排进展，不代表 RobotSim、
 * 真实机器人或 UI widget 的生命周期。
 */
enum class LocalizationWorkflowState {
    Idle,
    CameraStateAvailable,
    RobotStateAvailable,
    MissingVirtualReference,
    LocalCorrectionRunning,
    GlobalIcpRunning,
    Succeeded,
    Failed,
    Stopping
};

/**
 * @brief 定位流程状态更新。
 */
struct LocalizationWorkflowUpdate {
    LocalizationWorkflowState state = LocalizationWorkflowState::Idle;
    LocalizationMode mode = LocalizationMode::Global;
    qint64 frameId = -1;
    bool succeeded = true;
    QString message;
    QString errorMessage;
    int segmentCount = 0;
    IcpDofMask dofMask;
    bool hasResidual = false;
    double beforeResidual_mm = 0.0;
    double afterResidual_mm = 0.0;
};

/**
 * @brief 将定位流程状态转换为 UI 可显示文本。
 */
QString localizationWorkflowStateToText(LocalizationWorkflowState state);

/**
 * @brief 将定位流程状态转换为稳定 key。
 */
QString localizationWorkflowStateToKey(LocalizationWorkflowState state);

/**
 * @brief Localization 模块内的应用层流程状态机。
 *
 * 本类只维护定位流程状态和状态转换消息，不访问 QWidget、不依赖 Control/Vision、
 * 不连接 RobotSim，也不发送真实机器人命令。
 */
class LocalizationWorkflowController final {
public:
    /// @brief 当前定位流程状态。
    LocalizationWorkflowState state() const;

    /**
     * @brief 标记相机状态可用于定位流程。
     */
    LocalizationWorkflowUpdate markCameraStateAvailable(qint64 frameId);

    /**
     * @brief 标记机器人状态可用于定位流程。
     */
    LocalizationWorkflowUpdate markRobotStateAvailable();

    /**
     * @brief 标记局部修正流程开始。
     */
    LocalizationWorkflowUpdate startLocalCorrection(qint64 frameId, IcpDofMask dofMask);

    /**
     * @brief 根据局部修正结果更新流程状态。
     */
    LocalizationWorkflowUpdate finishLocalCorrection(const LocalCorrectionPipelineResult& result);

    /**
     * @brief 标记全局定位流程开始。
     */
    LocalizationWorkflowUpdate startGlobalIcp(IcpDofMask dofMask, int segmentCount);

    /**
     * @brief 根据全局 ICP 结果更新流程状态。
     */
    LocalizationWorkflowUpdate finishGlobalIcp(const GlobalIcpPipelineResult& result);

    /**
     * @brief 标记虚拟参考缺失。
     */
    LocalizationWorkflowUpdate markMissingVirtualReference(const QString& message);

    /**
     * @brief 停止当前定位流程。
     */
    LocalizationWorkflowUpdate stop();

    /**
     * @brief 清除定位流程状态。
     */
    LocalizationWorkflowUpdate clear();

private:
    LocalizationWorkflowUpdate makeUpdate(
        LocalizationWorkflowState state,
        LocalizationMode mode,
        qint64 frameId,
        bool succeeded,
        const QString& message,
        const QString& errorMessage = {});

    LocalizationWorkflowState m_state = LocalizationWorkflowState::Idle;
    qint64 m_lastFrameId = -1;
    IcpDofMask m_lastDofMask;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_WORKFLOW_CONTROLLER_H
