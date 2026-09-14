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
 * @file        OnlineCorrectionTypes.h
 * @brief       在线纠偏流程数据结构
 * @details     定义旧 OnlineCor change 主链路在新工程中的状态、输入事件和输出结果。
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
#ifndef I2R_RLC_CONTROL_ONLINE_CORRECTION_TYPES_H
#define I2R_RLC_CONTROL_ONLINE_CORRECTION_TYPES_H

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "geometry/PositionerKinematics.h"
#include "OnlineCorrectionCommand.h"
#include "RobotState.h"

namespace i2r_rlc::control {

/**
 * @brief 在线纠偏起点引导方式。
 *
 * 对应旧 `COnlineCor::correctStartPtType`，后续 UI 可直接映射为下拉选项。
 */
enum class OnlineCorrectionStartMode {
    /// @brief 由视觉初始检测点对齐示教轨迹起点。
    InitType,
    /// @brief 对示教轨迹应用全局刚性修正矩阵。
    GlobalCorrection,
    /// @brief 以当前机器人位姿作为 IBVS 起始点。
    IBVSStart
};

/**
 * @brief 在线纠偏状态机阶段。
 */
enum class OnlineCorrectionPhase {
    /// @brief 空闲，尚未开始纠偏。
    Idle,
    /// @brief 初始视觉检测正在运行。
    InitialDetectionRunning,
    /// @brief 等待机器人反向检测完成信号。
    WaitingForReverseCompletion,
    /// @brief 正在计算初始轨迹修正。
    PreparingInitialTrajectory,
    /// @brief 首批示教点正在按顺序发送。
    SendingInitialTeachPoints,
    /// @brief 等待控制柜以 offline==5 确认示教点更新完成。
    WaitingInitialTeachPointAck,
    /// @brief 在线纠偏运行中。
    OnlineRunning,
    /// @brief 当前在线分段正在收尾。
    SegmentFinishing,
    /// @brief 结束 P 已确认，等待机器人 detflag 复位到 0。
    WaitingForDetectionReset,
    /// @brief 全流程完成。
    Completed,
    /// @brief 流程出现错误。
    Error
};

/**
 * @brief 在线分段结束类型，对应旧 corflag 2/3。
 */
enum class OnlineCorrectionSegmentEndKind {
    Intermediate,
    Final
};

/**
 * @brief 旧 CorrectTaughtPathO 的控制方法枚举。
 *
 * 数值保持旧工程 `ControlMethod` 语义：0 为当前活动 PID 阈值分支，
 * 1 为模糊自适应 PID，2 为保留空控制分支。
 */
enum class OnlineCorrectionControlMethod {
    LegacyPidThreshold = 0,
    LegacyFuzzyAdaptivePid = 1,
    LegacyReservedNoop = 2
};

/**
 * @brief 空间点，单位 mm。
 */
struct OnlineCorrectionPoint {
    /// @brief X 坐标，单位为 mm。
    double xMm = 0.0;
    /// @brief Y 坐标，单位为 mm。
    double yMm = 0.0;
    /// @brief Z 坐标，单位为 mm。
    double zMm = 0.0;
    /// @brief 坐标是否有效。
    bool valid = false;
};

/**
 * @brief 位姿，位置单位 mm，姿态单位 deg。
 */
struct OnlineCorrectionPose {
    /// @brief X 坐标，单位为 mm。
    double xMm = 0.0;
    /// @brief Y 坐标，单位为 mm。
    double yMm = 0.0;
    /// @brief Z 坐标，单位为 mm。
    double zMm = 0.0;
    /// @brief RX 姿态角，单位为 deg。
    double rxDeg = 0.0;
    /// @brief RY 姿态角，单位为 deg。
    double ryDeg = 0.0;
    /// @brief RZ 姿态角，单位为 deg。
    double rzDeg = 0.0;
    /// @brief 位姿是否有效。
    bool valid = false;
};

/**
 * @brief 变位机轴角，单位 deg。
 */
struct OnlineCorrectionStationAxis {
    /// @brief 变位机第一轴角度，单位为 deg。
    double s1Deg = 0.0;
    /// @brief 变位机第二轴角度，单位为 deg。
    double s2Deg = 0.0;
    /// @brief 轴角是否有效。
    bool valid = false;
};

/**
 * @brief 单个在线检测样本。
 *
 * 点与间隙宽度放在同一结构中，保证启用间隙检测时两者按样本一一对应。
 */
struct OnlineCorrectionDetectionSample {
    /// @brief 视觉重建后的空间点，单位为 mm。
    OnlineCorrectionPoint point;
    /// @brief 可选 gap 宽度，单位为 mm，仅 enableGapDetection 时强制要求。
    std::optional<float> gapWidthMm;
    /// @brief 仿真扫描焊缝编号，来自旧 getSeamIndex()；无仿真时为 -1。
    int simulatedSeamIndex = -1;
    /// @brief 旧 sim_scan_line_change_point 等价输入，单位 mm；无效时不触发曲线切换。
    std::optional<OnlineCorrectionPoint> scanLineChangePoint;
};

/**
 * @brief 一帧或一批视觉检测事件。
 *
 * 该事件承接旧 `D_initdata` 的填充语义，调用方负责把视觉点重建到空间坐标。
 */
struct OnlineCorrectionDetectionEvent {
    /// @brief 当前批次视觉样本，调用方按检测顺序填充。
    std::vector<OnlineCorrectionDetectionSample> samples;
    /// @brief true 表示初始检测阶段已经结束。
    bool detectionFinished = false;
    /// @brief 事件说明或诊断消息。
    std::string message;
};

/**
 * @brief 在线纠偏配置。
 */
struct OnlineCorrectionOptions {
    /// @brief 在线纠偏起点模式，兼容旧 correctStartPtType。
    OnlineCorrectionStartMode startMode = OnlineCorrectionStartMode::InitType;
    /// @brief true 表示初始检测样本必须携带 gap 宽度。
    bool enableGapDetection = false;
    /// @brief 机器人 TCP 速度，单位 mm/s，用于保持旧纠偏周期语义。
    double stepTcpVelocityMmPerSecond = 11.0;
    /// @brief 控制周期，单位 s，旧活动路径使用 0.0043。
    double controlCycleSecond = 0.0043;
    /// @brief 检测点越过判定距离，单位 mm。
    double passedPointDistanceMm = 0.8;
    /// @brief CorrectTaughtPathO 控制方法，默认保持旧活动分支 ControlMethod==0。
    OnlineCorrectionControlMethod controlMethod =
        OnlineCorrectionControlMethod::LegacyPidThreshold;
    /// @brief 实时增量绝对值限制，单位 um。
    long maxCorrectionIncrementUm = 80;
    /// @brief 相邻实时增量允许跳变，单位 um。
    long maxCorrectionJumpUm = 60;
    /// @brief 行主序 4x4 全局修正矩阵，长度固定为 16，坐标单位为 mm。
    std::array<double, 16> globalCorrectionRowMajor = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
};

/**
 * @brief 在线纠偏控制器持有的会话状态。
 */
struct OnlineCorrectionSessionState {
    /// @brief 当前状态机阶段。
    OnlineCorrectionPhase phase = OnlineCorrectionPhase::Idle;
    /// @brief 当前起点模式。
    OnlineCorrectionStartMode startMode = OnlineCorrectionStartMode::InitType;
    /// @brief 当前轨迹分段索引。
    int segmentIndex = 0;
    /// @brief 当前分段起始示教点索引。
    int startTeachIndex = 0;
    /// @brief 当前分段结束示教点索引。
    int endTeachIndex = 0;
    /// @brief true 表示视觉跟踪/检测应保持运行。
    bool trackerActive = false;
    /// @brief true 表示初始检测已经结束。
    bool initialDetectionFinished = false;
    /**
     * @brief true 表示已观察到 onlineReverseCompleteFlag==0，可接受下一次上升为 1。
     *
     * 防止上一轮遗留的完成电平在新一轮初始检测启动后被误判为新事件。
     */
    bool reverseCompletionArmed = false;
    /// @brief true 表示示教轨迹已成功加载并分段。
    bool onlineTeachTrajectoryInitialized = false;
    /// @brief RBCALIB 变位机标定是否有效。
    bool positionerCalibrationInitialized = false;
    /// @brief 初始检测样本缓存，由控制器按事件累积。
    std::vector<OnlineCorrectionDetectionSample> initialDetectionSamples;
    /// @brief 在线运行阶段尚未消费的检测样本。
    std::vector<OnlineCorrectionDetectionSample> onlineDetectionSamples;
    /// @brief 初始修正后的当前分段轨迹，单位 mm。
    std::vector<OnlineCorrectionPoint> correctedSegmentPath;
    /// @brief 已发送的首批示教点数量。
    int teachPointSendIndex = 0;
    /// @brief 首批示教点总数，用于只接受最终 O 命令的 offline==5 确认。
    int initialTeachPointCount = 0;
    /// @brief 最近一次 getSeamIndex()。
    int simulatedSeamIndex = -1;
    /// @brief 当前锁存的分段结束类型。
    std::optional<OnlineCorrectionSegmentEndKind> pendingSegmentEndKind;
    /// @brief true 表示当前 corflag/detflag 结束边沿已经处理。
    bool segmentEndLatched = false;
    /// @brief 最近一次状态说明或错误消息。
    std::string message;
};

/**
 * @brief 在线纠偏控制器单次状态更新结果。
 */
struct OnlineCorrectionUpdate {
    /// @brief 当前事件处理是否成功。
    bool succeeded = true;
    /// @brief 事件处理后的状态机阶段。
    OnlineCorrectionPhase phase = OnlineCorrectionPhase::Idle;
    /// @brief 事件处理后的完整会话快照。
    OnlineCorrectionSessionState state;
    /// @brief 可选机器人纠偏命令，仅 valid 为 true 时发送。
    robot_comm::RobotCorrectionCommand command;
    /// @brief 可选实时 P 命令，仅 valid 时发送。
    robot_comm::RobotRealtimeCorrectionCommand realtimeCommand;
    /// @brief true 表示 runtime 应启动 Vision tracking。
    bool startTracking = false;
    /// @brief true 表示 runtime 应停止 Vision tracking。
    bool stopTracking = false;
    /// @brief true 表示 runtime 应加载下一分段并自动重新开始初始检测。
    bool requestNextSegment = false;
    /// @brief 事件处理说明或错误消息。
    std::string message;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_TYPES_H
