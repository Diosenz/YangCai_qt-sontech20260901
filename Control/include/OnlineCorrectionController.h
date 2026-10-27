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
 * @file        OnlineCorrectionController.h
 * @brief       在线纠偏状态机
 * @details     用显式输入事件替代旧 OnlineCor 的 MFC Timer 和按钮回调，负责在线纠偏流程编排。
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
#ifndef I2R_RLC_CONTROL_ONLINE_CORRECTION_CONTROLLER_H
#define I2R_RLC_CONTROL_ONLINE_CORRECTION_CONTROLLER_H

#include <string>

#include "OnlineCorrectionExceptionProcessor.h"
#include "OnlineCorrectionPathFollower.h"
#include "OnlineCorrectionTrajectoryPlanner.h"

namespace i2r_rlc::control {

/**
 * @brief 在线纠偏流程控制器。
 *
 * 控制器不访问相机、UI 或机器人 socket；视觉检测结果和机器人状态由调用方显式传入。
 */
class COnlineCorrectionController final {
public:
    COnlineCorrectionController() = default;

    /**
     * @brief 配置在线纠偏状态机。
     * @param options 在线纠偏模式、gap 检测和全局修正矩阵。
     * @param trajectory 已加载和分段的示教轨迹，调用后按值保存。
     * @note 本类不持有 UI、相机或 socket 资源，可在控制线程同步调用。
     */
    void configure(
        const OnlineCorrectionOptions &options,
        const OnlineCorrectionPlannedTrajectory &trajectory,
        const vision::PositionerCalibration &positionerCalibration);

    /**
     * @brief 兼容只配置轨迹的调用；启动前仍需提供有效 RBCALIB 标定。
     */
    void configure(
        const OnlineCorrectionOptions &options,
        const OnlineCorrectionPlannedTrajectory &trajectory);

    /**
     * @brief 启动初始视觉检测阶段。
     * @return 状态更新结果，trackerActive 会被置为 true。
     */
    OnlineCorrectionUpdate startInitialDetection();

    /**
     * @brief 追加一批视觉检测事件。
     * @param event 视觉检测样本和阶段完成标志，点坐标单位为 mm。
     * @return 状态更新结果；gap 模式下缺少 gapWidthMm 会进入 Error。
     */
    OnlineCorrectionUpdate appendDetectionEvent(const OnlineCorrectionDetectionEvent &event);

    /**
     * @brief 通知初始检测结束并尝试规划初始纠偏命令。
     * @param robotState 当前机器人状态，IBVSStart 使用当前 TCP 作为起点。
     * @return 规划结果和可选机器人命令。
     */
    OnlineCorrectionUpdate onDetectionFinished(const robot_comm::RobotState &robotState);

    /**
     * @brief 输入机器人状态变化，推进反扫、P001、STARTCOR 和程序停止门禁。
     * @param robotState 当前机器人状态。
     * @return 状态更新结果；仅确认 STARTCOR 后才请求启动在线 Vision tracking。
     */
    OnlineCorrectionUpdate onRobotStateChanged(const robot_comm::RobotState &robotState);

    /**
     * @brief 输入机器人命令发送及回包结果。
     */
    OnlineCorrectionUpdate onRobotCommandReceipt(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &robotState);

    /**
     * @brief 通知在线分段结束。
     */
    OnlineCorrectionUpdate finishOnlineSegment();
    OnlineCorrectionUpdate finishOnlineSegment(
        OnlineCorrectionSegmentEndKind kind);

    /**
     * @brief 以不可继续的错误结束当前会话，不生成任何机器人命令。
     * @param message 供 UI、运行记录和现场诊断显示的错误原因。
     * @return Error 状态更新，并请求停止 Vision tracking。
     */
    OnlineCorrectionUpdate fail(const std::string &message);

    /**
     * @brief 重置会话状态，保留 configure 设置的选项。
     * @return 重置后的状态快照。
     */
    OnlineCorrectionUpdate reset();

    /**
     * @brief 获取当前在线纠偏状态。
     * @return 控制器内部状态的只读引用，调用方不得超过控制器生命周期保存。
     */
    const OnlineCorrectionSessionState &state() const;

private:
    /// @brief 在控制线程按当前空间进度检查可靠前视末端；不按帧数或超时删点。
    bool reliableTrajectoryExhausted();
    /// @brief 只在当前图像恢复失败且可靠空间范围耗尽时锁存作业停止。
    bool shouldStopForLocalizationLoss();
    bool m_localizationUnsupported = false;
    OnlineCorrectionPoint m_lastReliablePoint;
    /// @brief 独立维护可靠末端与机器人进度，避免回折支系切平面造成误停。
    int m_reliableEndSegmentIndex = 0;
    /// @brief 仅实测扩展的有序路径进度，不被较近的新曝光缩短。
    double m_reliableEndProgress = -1.0;
    int m_lossRobotSegmentIndex = 0;
    /**
     * @brief 基于当前状态生成统一更新 DTO。
     * @param succeeded 当前事件是否成功。
     * @param message 事件说明或错误消息。
     * @return 在线纠偏状态更新。
     */
    OnlineCorrectionUpdate makeUpdate(bool succeeded, const std::string &message) const;

    /**
     * @brief 设置状态机阶段和说明文本。
     * @param phase 新阶段。
     * @param message 状态说明。
     */
    void setPhase(OnlineCorrectionPhase phase, const std::string &message);

    /// @brief 先使用已释放实测队列；无可用几何时以本会话校正参考重算控制目标。
    OnlineCorrectionFollowResult followCurrentTarget();
    /// @brief 仅独立图像几何可更新参考偏差；estimated 不反向训练预测锚。
    void updatePredictionAnchor(const std::vector<OnlineCorrectionDetectionSample> &samples);
    /// @brief 原子保存实际 P 使用的几何及单会话计算序号，不生成相机测量身份。
    void recordControlTarget(const OnlineCorrectionFollowResult &follow, OnlineCorrectionTargetSource source);
    OnlineCorrectionOptions m_options;
    OnlineCorrectionPlannedTrajectory m_trajectory;
    /// @brief 本会话起点修正后的参考，只供在线 ICP 和跟随器使用。
    OnlineCorrectionPlannedTrajectory m_onlineTrajectory;
    vision::PositionerCalibration m_positionerCalibration;
    OnlineCorrectionSessionState m_state;
    robot_comm::RobotState m_robotState;
    COnlineCorrectionExceptionProcessor m_exceptionProcessor;
    COnlineCorrectionPathFollower m_pathFollower;
    bool m_predictionReferenceReady = false;
    OnlineCorrectionPoint m_predictionOffset;
    std::optional<float> m_predictionGapWidthMm;
    int m_predictionSegmentIndex = 0;
    std::uint64_t m_targetGeneration = 0;
    /// @brief 上次实际计算 P 所用反馈，独立于后续命令回包更新的 m_robotState。
    robot_comm::RobotState m_lastControlRobotState;
    bool m_hasControlRobotState = false;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_CONTROLLER_H
