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
 * @file        OnlineCorrectionCommand.h
 * @brief       在线纠偏机器人命令数据结构
 * @details     定义 Control 模块生成、RobotComm 模块负责发送的在线纠偏命令 DTO。
 *              第一版只表达命令内容，不在该文件中绑定真实机器人协议。
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
#ifndef I2R_RLC_ROBOT_COMM_ONLINE_CORRECTION_COMMAND_H
#define I2R_RLC_ROBOT_COMM_ONLINE_CORRECTION_COMMAND_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "RobotState.h"

namespace i2r_rlc::robot_comm {

/**
 * @brief 待写入机器人示教点的位置与姿态。
 *
 * 该结构保留旧工程发送给机器人端的整数单位：位置为 um，姿态为 deg * 10000。
 */
struct TeachPointUpdateCommand {
    /// @brief X 坐标，单位 um。
    long xUm = 0;

    /// @brief Y 坐标，单位 um。
    long yUm = 0;

    /// @brief Z 坐标，单位 um。
    long zUm = 0;

    /// @brief RX 姿态，单位 deg * 10000。
    long rxDeg10000 = 0;

    /// @brief RY 姿态，单位 deg * 10000。
    long ryDeg10000 = 0;

    /// @brief RZ 姿态，单位 deg * 10000。
    long rzDeg10000 = 0;

    /// @brief true 表示该示教点可发送。
    bool valid = false;
};

/**
 * @brief 待写入变位机示教点的轴角。
 *
 * 轴角单位沿用旧工程与机器人端约定：deg * 10000。
 */
struct StationAxisUpdateCommand {
    /// @brief 变位机 1 轴角度，单位 deg * 10000。
    long s1Deg10000 = 0;

    /// @brief 变位机 2 轴角度，单位 deg * 10000。
    long s2Deg10000 = 0;

    /// @brief true 表示该变位机点可发送。
    bool valid = false;
};

/**
 * @brief 一批在线纠偏示教点更新命令。
 *
 * `teachPoints` 与 `stationAxes` 按索引对应；当 `stationAxes` 为空时表示调用方
 * 当前不需要更新变位机轴角。
 */
struct RobotCorrectionCommand {
    /// @brief true 表示需要执行示教点更新。
    bool requestTeachPointUpdate = false;

    /// @brief 旧工程 `PvarNum` 语义：本批需要更新的点数。
    int pointCount = 0;

    /// @brief 机器人示教点更新列表。
    std::vector<TeachPointUpdateCommand> teachPoints;

    /// @brief 变位机轴角更新列表。
    std::vector<StationAxisUpdateCommand> stationAxes;

    /// @brief 旧工程 `s_offline` 的命令语义，第一版只作为 DTO 字段保留。
    int offlineSignal = 0;

    /// @brief 状态说明，用于 dry-run 日志或测试。
    std::string message;

    /// @brief true 表示命令内容完整。
    bool valid = false;
};

/**
 * @brief 旧机器人控制柜协议的公共状态字段。
 *
 * 字段顺序与旧 `RobotCtrl.cpp` 中 N/O/P 消息保持一致。`onlineEndFlag`
 * 只属于出站协议，不写入 RobotState。
 */
struct RobotProtocolContext {
    int robotEnabled = 0;
    int trackerEnabled = 0;
    int laserEnabled = 0;
    int filterEnabled = 0;
    int offlineSignal = 0;
    int offlineStartCount = 0;
    int holdRequested = 0;
    int firstDetectionSucceeded = 0;
    int onlineEndFlag = 0;
};

/** @brief 上位机本地命令来源；不进入P/D线协议，不代表控制柜接纳或执行确认。 */
struct RobotLocalCommandTrace {
    std::uint64_t sessionId = 0;
    std::uint64_t commandId = 0;
    std::optional<std::uint64_t> targetGeneration;
};

/**
 * @brief 在线实时纠偏增量命令。
 *
 * 位置增量单位为 um，姿态增量单位为 deg * 10000，速度倍率范围为
 * 1000~10000。
 */
struct RobotRealtimeCorrectionCommand {
    /// @brief 由生成该P的控制会话赋值；未知调用方保持缺失。
    std::optional<RobotLocalCommandTrace> localTrace;
    long dxUm = 0;
    long dyUm = 0;
    long dzUm = 0;
    long drxDeg10000 = 0;
    long dryDeg10000 = 0;
    long drzDeg10000 = 0;
    long speedOverride = 10000;
    int speedMode = 0;
    /// @brief 旧协议 Online_endflag；仅属于本次出站 P 命令。
    int onlineEndFlag = 0;
    bool valid = false;
    std::string message;
};

/**
 * @brief 机器人命令类型。
 */
enum class RobotCommandType {
    StatusPoll,
    TeachPointUpdate,
    RealtimeCorrection,
    CalibrationAdvance,
    JobStart
};

enum class RobotCommandError {
    None,
    NotConnected,
    ConnectFailed,
    SendFailed,
    ReceiveFailed,
    EmptyResponse,
    InvalidResponse,
    TransportError,
    /// @brief 本地作业保持已锁存，命令未交给传输层。
    OperationStopped
};

/**
 * @brief 单次机器人协议请求的处理结果。
 */
struct RobotCommandReceipt {
    /// @brief 实际选中并交给传输层的P来源，不从最新命令或回包内容反推。
    std::optional<RobotLocalCommandTrace> localTrace;
    /// @brief RobotComm本地P事务序号；0表示未知，绝非机器人执行序号。
    std::uint64_t localTransactionId = 0;
    RobotCommandType type = RobotCommandType::StatusPoll;
    bool sent = false;
    bool responseReceived = false;
    bool completed = false;
    int pointIndex = -1;
    std::string request;
    std::string response;
    RobotCommandError error = RobotCommandError::None;
    std::string message;
};

/** @brief 作业停止的控制器保持状态；不代表机械零速或焊机停弧。 */
enum class RobotOperationStopStatus {
    NotRequested,
    Pending,
    HoldConfirmed,
    Unconfirmed
};

/** @brief 跨线程发布的停止快照；停弧接口尚未核实，不能从 HOLD 推导弧状态。 */
struct RobotOperationStopState {
    RobotOperationStopStatus status = RobotOperationStopStatus::NotRequested;
    std::uint64_t requestGeneration = 0;
    bool holdRequestLatched = false;
    bool arcStopVerified = false;
    RobotCommandError error = RobotCommandError::None;
    std::string message;
};

/**
 * @brief RobotComm 现场诊断快照。
 */
struct RobotCommunicationDiagnostics {
    std::string endpoint;
    bool connected = false;
    /// @brief true 表示至少成功解析过一次完整 D 回包，状态字段可信。
    bool validDReceived = false;
    /// @brief 最近一次协议事务是否完成发送、接收并解析成功。
    bool lastTransactionSucceeded = false;
    /// @brief 连续失败次数，用于现场判断连接是否在抖动或已断开。
    int consecutiveFailureCount = 0;
    RobotState state;
    int laserEnabled = 0;
    int filterEnabled = 0;
    RobotCommandType lastCommandType = RobotCommandType::StatusPoll;
    std::string lastRequest;
    std::string lastResponse;
    std::string lastErrorText;
    std::string latestStatusRequest;
    std::string latestTeachPointRequest;
    std::string latestRealtimeRequest;
    std::string latestCalibrationRequest;
    std::string latestJobRequest;
    std::string latestResponse;
    std::string errorMessage;
    long long timestampMs = 0;
    unsigned long long transactionCount = 0;
};

} // namespace i2r_rlc::robot_comm

#endif // I2R_RLC_ROBOT_COMM_ONLINE_CORRECTION_COMMAND_H
