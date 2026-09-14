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
 * @file        OfflineCorrectionController.h
 * @brief       离线纠偏控制器
 * @details     以现代事件状态机承接旧 OffLineCor 有匹配离线纠偏主链路。
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
#ifndef I2R_RLC_CONTROL_OFFLINE_CORRECTION_CONTROLLER_H
#define I2R_RLC_CONTROL_OFFLINE_CORRECTION_CONTROLLER_H

#include <string>

#include "OfflineCorrectionTrajectoryMatcher.h"

namespace i2r_rlc::control {

/**
 * @brief 离线纠偏有匹配流程控制器。
 *
 * 控制器只保存状态并生成 RobotComm DTO，不直接访问相机、视觉线程或 socket。
 */
class COfflineCorrectionController final {
public:
    /**
     * @brief 配置离线纠偏状态机并清空当前会话。
     * @param options ICP、检测点裁剪和平滑参数。
     */
    void configure(const OfflineCorrectionOptions &options);

    /**
     * @brief 选择离线纠偏存档文件并解析配套 _W/_T 文件。
     * @param archiveFilePath 用户选择的存档路径，可为 _W.txt 或 _T.txt。
     * @return 状态更新结果，失败时进入 Error。
     */
    OfflineCorrectionUpdate selectArchiveFile(const std::string &archiveFilePath);

    /**
     * @brief 开始接收离线检测点。
     * @return 状态更新结果，要求已成功选择存档。
     */
    OfflineCorrectionUpdate startDetection();

    /**
     * @brief 追加一批离线检测点。
     * @param event 检测事件，空间点单位为 mm。
     * @return 状态更新结果。
     */
    OfflineCorrectionUpdate appendDetectionEvent(const OfflineCorrectionDetectionEvent &event);

    /**
     * @brief 结束检测点接收，进入可计算阶段。
     * @return 状态更新结果。
     */
    OfflineCorrectionUpdate finishDetection();

    /**
     * @brief 执行离线 ICP 匹配并生成机器人示教点更新命令。
     * @return 匹配报告和可选机器人命令。
     */
    OfflineCorrectionUpdate computeCorrection();

    /**
     * @brief 标记最近一次机器人命令已提交给 RobotComm，等待 D.offline==5。
     * @return 状态更新结果。
     */
    OfflineCorrectionUpdate markCommandDispatched();

    /**
     * @brief 标记最近一次机器人命令已由控制柜确认。
     * @return 状态更新结果。
     */
    OfflineCorrectionUpdate markCommandSent();

    /**
     * @brief 将当前会话置为 Error，用于 runtime worker 报告通信或配置错误。
     * @param message 错误消息。
     * @return 失败状态更新。
     */
    OfflineCorrectionUpdate fail(const std::string &message);

    /**
     * @brief 重置当前会话和缓存命令。
     * @return 重置后的状态快照。
     */
    OfflineCorrectionUpdate reset();

    /**
     * @brief 获取当前离线纠偏状态。
     * @return 控制器内部状态的只读引用，调用方不得超过控制器生命周期保存。
     */
    const OfflineCorrectionSessionState &state() const;

    /**
     * @brief 获取最近一次成功计算、尚待或已经得到机器人确认的诊断快照。
     * @return 控制器内部快照的只读引用，调用方不得超过控制器生命周期保存。
     */
    const OfflineCorrectionDiagnosticData &diagnostics() const;

private:
    /**
     * @brief 基于当前状态和最近命令生成统一更新 DTO。
     * @param succeeded 当前事件是否成功。
     * @param message 事件说明或错误消息。
     * @return 离线纠偏状态更新。
     */
    OfflineCorrectionUpdate makeUpdate(bool succeeded, const std::string &message) const;

    /**
     * @brief 清空最近命令、进入 Error 阶段并生成失败更新。
     * @param message 错误消息。
     * @return 失败状态更新。
     */
    OfflineCorrectionUpdate makeError(const std::string &message);

    /**
     * @brief 设置状态机阶段和说明文本。
     * @param phase 新阶段。
     * @param message 状态说明。
     */
    void setPhase(OfflineCorrectionPhase phase, const std::string &message);

    OfflineCorrectionOptions m_options;
    OfflineCorrectionArchive m_archive;
    OfflineCorrectionSessionState m_state;
    robot_comm::RobotCorrectionCommand m_lastCommand;
    OfflineCorrectionDiagnosticData m_lastDiagnostics;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_OFFLINE_CORRECTION_CONTROLLER_H
