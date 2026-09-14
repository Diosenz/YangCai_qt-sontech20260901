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
 * @file        OfflineCorrectionTypes.h
 * @brief       离线纠偏状态机数据结构
 * @details     定义有匹配离线纠偏流程的输入事件、状态、结果和机器人命令输出。
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
#ifndef I2R_RLC_CONTROL_OFFLINE_CORRECTION_TYPES_H
#define I2R_RLC_CONTROL_OFFLINE_CORRECTION_TYPES_H

#include <cstdint>
#include <string>
#include <vector>

#include "geometry/IcpTypes.h"
#include "geometry/WeldGeometryTypes.h"
#include "OnlineCorrectionCommand.h"

namespace i2r_rlc::control {

/**
 * @brief 离线纠偏阶段。
 */
enum class OfflineCorrectionPhase {
    /// @brief 空闲，尚未选择存档。
    Idle,
    /// @brief 已选择并解析离线纠偏存档。
    ArchiveSelected,
    /// @brief 等待视觉检测点输入。
    WaitingForDetection,
    /// @brief 正在接收检测点。
    Detecting,
    /// @brief 检测完成，可执行匹配计算。
    ReadyToCompute,
    /// @brief ICP 匹配和命令生成中。
    Computing,
    /// @brief 已生成机器人示教点更新命令。
    ReadyToSend,
    /// @brief 已发送 O 示教点更新命令，等待控制柜 D.offline==5 确认。
    WaitingForRobotAck,
    /// @brief 命令已由控制柜确认，流程完成。
    Completed,
    /// @brief 流程出现错误。
    Error
};

/**
 * @brief 离线纠偏配置。
 */
struct OfflineCorrectionOptions {
    /// @brief ICP 最大迭代次数。
    int maxIcpIterations = 200;

    /// @brief ICP 收敛阈值，单位 mm。
    double minIcpDeltaMm = 1.0e-4;

    /// @brief 检测点头部丢弃比例，旧工程当前运行路径为 5%。
    double discardLeadingDetectionRatio = 0.05;

    /// @brief 简单平滑窗口半径，单位为点数。
    int smoothingRadius = 1;

    /// @brief 相邻检测点的最小轴向变化量，单位 mm；兼容旧工程 200 um 采样门限。
    double minimumSpatialStepMm = 0.2;

    /// @brief 图像域密度过滤的最小邻域点数，旧工程固定为 4。
    int imageClusterMinimumPoints = 4;

    /**
     * @brief 最近一次成功离线纠偏的诊断输出目录。
     *
     * 目录为空时不保存。文件只在控制柜以 D.offline==5 确认全部 O 点后覆盖，
     * 计算或通信失败不会破坏上一份成功记录。
     */
    std::string diagnosticOutputDirectory;
};

/**
 * @brief 单个离线检测空间点样本。
 */
struct OfflineCorrectionDetectionSample {
    /// @brief 检测空间点，单位 mm。
    i2r_rlc::vision::WeldSpacePoint point;
    /// @brief 检测点图像横坐标，单位 pixel。
    double imageXPixel = 0.0;
    /// @brief 检测点图像纵坐标，单位 pixel。
    double imageYPixel = 0.0;
    /// @brief true 表示图像坐标可用于旧式图像域密度过滤。
    bool imagePointValid = false;
    /// @brief 产生该样本的相机帧号，仅用于诊断和顺序追踪。
    std::int64_t frameId = -1;
};

/**
 * @brief 离线检测事件。
 */
struct OfflineCorrectionDetectionEvent {
    /// @brief 当前批次检测点，按轨迹顺序排列。
    std::vector<OfflineCorrectionDetectionSample> samples;

    /// @brief 事件说明。
    std::string message;
};

/**
 * @brief 离线纠偏匹配报告。
 */
struct OfflineCorrectionMatchReport {
    /// @brief true 表示匹配报告可用于展示和发送。
    bool valid = false;
    /// @brief 存档模型轨迹点数量，单位为点。
    int modelPointCount = 0;
    /// @brief 参与 ICP 的检测点数量，单位为点。
    int detectionPointCount = 0;
    /// @brief 生成命令的示教点数量，单位为点。
    int teachPointCount = 0;
    /// @brief ICP 平均误差，单位为 mm。
    double meanErrorMm = 0.0;
    /// @brief ICP RMS 误差，单位为 mm。
    double rmsErrorMm = 0.0;
    /// @brief 匹配置信度，范围为 [0,1]。
    double confidence = 0.0;
    /// @brief 匹配说明或错误消息。
    std::string message;
};

/**
 * @brief 离线纠偏会话状态。
 */
struct OfflineCorrectionSessionState {
    /// @brief 当前状态机阶段。
    OfflineCorrectionPhase phase = OfflineCorrectionPhase::Idle;
    /// @brief 用户选择的 _W.txt 或 _T.txt 存档路径。
    std::string archiveFilePath;
    /// @brief 解析得到的模型轨迹文件路径。
    std::string modelFilePath;
    /// @brief 解析得到的示教点文件路径。
    std::string teachFilePath;
    /// @brief 已接收的检测点样本，坐标单位为 mm。
    std::vector<OfflineCorrectionDetectionSample> detectionSamples;
    /// @brief 最近一次匹配报告。
    OfflineCorrectionMatchReport report;
    /// @brief 最近一次状态说明或错误消息。
    std::string message;
};

/**
 * @brief 离线纠偏单次状态更新。
 */
struct OfflineCorrectionUpdate {
    /// @brief 当前事件处理是否成功。
    bool succeeded = true;
    /// @brief 事件处理后的状态机阶段。
    OfflineCorrectionPhase phase = OfflineCorrectionPhase::Idle;
    /// @brief 事件处理后的完整会话快照。
    OfflineCorrectionSessionState state;
    /// @brief 可选机器人示教点更新命令，仅 valid 为 true 时发送。
    robot_comm::RobotCorrectionCommand robotCommand;
    /// @brief 事件处理说明或错误消息。
    std::string message;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_OFFLINE_CORRECTION_TYPES_H
