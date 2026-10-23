/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OnlineCorrectionRunRecorder.h
 * @brief       在线纠偏运行数据记录器
 * @details     为每次在线纠偏会话保存初始修正、实时修正和机器人实际路径。
 ******************************************************************************/
#ifndef I2R_RLC_UI_ONLINE_CORRECTION_RUN_RECORDER_H
#define I2R_RLC_UI_ONLINE_CORRECTION_RUN_RECORDER_H

#include <array>
#include <memory>

#include <QByteArray>
#include <QString>
#include <QStringList>

#include "RuntimeTrackingTypes.h"
#include "TrackingDiagnosticImageWriter.h"

namespace i2r_rlc::ui {

/**
 * @brief 在线纠偏会话级文本记录器。
 *
 * 本类仅由在线纠偏 worker 线程调用。每次写入均立即打开、追加并关闭文件，
 * 确保算法失败或用户中止时，已产生的数据仍保留在本轮会话目录中。
 */
class COnlineCorrectionRunRecorder final {
public:
    /**
     * @brief 创建新的在线纠偏记录目录和带表头的文本文件。
     * @param outputRootDirectory `output/online_correction` 根目录。
     * @param configuration 本轮在线纠偏配置。
     * @param errorMessage 失败时接收文件系统错误。
     * @return 全部记录文件创建成功时返回 true。
     */
    bool beginSession(
        const QString &outputRootDirectory,
        const RuntimeOnlineCorrectionConfiguration &configuration,
        QString *errorMessage);

    /// @brief 记录一帧机器人 D 状态，形成实际运行路径采样。
    bool recordRobotState(
        const robot_comm::RobotState &state,
        control::OnlineCorrectionPhase phase);

    /// @brief 记录在线视觉输入，包括无效结果和其错误说明。
    bool recordTrackingResult(
        const RuntimeTrackingResult &result,
        control::OnlineCorrectionPhase phase);

    /// @brief 记录初始 O 点或实时 P 修正命令。
    bool recordUpdate(
        const control::OnlineCorrectionUpdate &update,
        const vision::PositionerCalibration &positionerCalibration);

    /// @brief 记录 O/P 命令的实际发送和回包结果。
    bool recordCommandReceipt(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &state,
        control::OnlineCorrectionPhase phase);

    /**
     * @brief 写入本轮最终状态并关闭会话。
     * @param resultText completed、error、reset 或 restarted。
     * @param message 最终状态说明。
     */
    bool finishSession(const QString &resultText, const QString &message);

    /// @brief true 表示当前存在尚未结束的记录会话。
    bool isActive() const;

    /// @brief 当前会话输出目录；未开始时为空。
    QString sessionDirectory() const;

    /// @brief 最近一次文件操作错误。
    QString lastError() const;

private:
    /// @brief 保存本帧各阶段快照；只读诊断，失败不改变控制状态。
    bool recordVisionDiagnostic(
        const RuntimeTrackingResult &result, control::OnlineCorrectionPhase phase);
    COnlineVisionDiagnosticOptions m_visionDiagnosticOptions;
    std::unique_ptr<CTrackingDiagnosticImageWriter> m_imageWriter;
    quint64 m_visionDiagnosticCount = 0;
    quint64 m_imageSubmissionCount = 0;
    std::array<int, 5> m_imageBucketSubmissionCounts{};
    bool m_imageErrorReported = false;

    /// @brief 批次点列合并写入，避免逐点开关文件；仅由在线 worker 调用。
    bool recordIcpDiagnostics(const control::OnlineCorrectionUpdate &update);
    bool appendData(const QString &fileName, const QByteArray &data);
    bool createSessionFile(const QString &fileName, const QString &header);
    bool appendFields(const QString &fileName, const QStringList &fields);
    bool appendSummary(const QString &key, const QString &value);
    void setFileError(const QString &operation, const QString &path);

    QString m_sessionDirectory;
    QString m_lastError;
    qint64 m_startedAtMs = 0;
    quint64 m_robotStateCount = 0;
    quint64 m_trackingResultCount = 0;
    quint64 m_correctedPathPointCount = 0;
    quint64 m_initialCommandCount = 0;
    quint64 m_realtimeCommandCount = 0;
    quint64 m_receiptCount = 0;
    quint64 m_updateCount = 0;
    quint64 m_icpBatchCount = 0;
    bool m_active = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_CORRECTION_RUN_RECORDER_H
