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
#include <deque>
#include <memory>

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <QStringList>

#include "RuntimeTrackingTypes.h"
#include "OnlineCorrectionTextWriter.h"
#include "TrackingDiagnosticImageWriter.h"

namespace i2r_rlc::ui {

/**
 * @brief 在线纠偏会话级文本记录器。
 *
 * 本类仅由在线纠偏 worker 线程调用；记录时间在入队前冻结，文件追加在后台执行。
 * 每会话最多8 MiB/4096块未完成文本，队列满或写失败明确报告不完整，不阻塞控制。
 * 正常会话结束排空并回收线程；进程崩溃可能丢失尚未落盘的有界队列。
 */
class COnlineCorrectionRunRecorder final {
public:
    /**
     * @brief 文件系统追加边界；成功须包含刷新检查，失败须填写错误说明。
     * @details 运行期在后台调用；终态排空后在所有者线程写最终摘要。
     * 回调由记录器持有，不能访问 widgets 或要求固定线程绑定资源。
     */
    using TextAppendFunction = COnlineCorrectionTextWriter::AppendFunction;
    COnlineCorrectionRunRecorder();
    explicit COnlineCorrectionRunRecorder(TextAppendFunction append);
    ~COnlineCorrectionRunRecorder();
    /// @brief 离线读取活动记录前显式排空；不得在在线控制周期调用。
    bool flush();

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

    /// @brief 同时冻结结果到达控制线程时的参考；该快照不是曝光时刻的参考。
    bool recordTrackingResult(const RuntimeTrackingResult &result,
        control::OnlineCorrectionPhase phase,
        const control::OnlineCorrectionTargetSnapshot &target);

    /// @brief 记录初始 O 点或实时 P 修正命令。
    bool recordUpdate(
        const control::OnlineCorrectionUpdate &update,
        const vision::PositionerCalibration &positionerCalibration);

    /// @brief 记录 O/P 命令的实际发送和回包结果。
    bool recordCommandReceipt(
        const robot_comm::RobotCommandReceipt &receipt,
        const robot_comm::RobotState &state,
        control::OnlineCorrectionPhase phase);

    /// @brief 按原运行会话身份记录 HOLD 进度，不将保持反馈写成停弧确认。
    bool recordOperationStop(
        quint64 sessionId,
        const robot_comm::RobotOperationStopState &state);

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
    /** @brief 临时诊断环；每阶段首拒绝及首像素丢失各有窗口，最多六帧、16 MiB。 */
    struct CLossFrame {
        qint64 frameId = -1;
        QString stem;
        QImage source;
        QByteArray context;
    };
    /// @brief 只归档到达顺序，不补造缺失曝光；后台编码不阻塞控制计算。
    bool recordLossWindow(const RuntimeTrackingResult &result,
        control::OnlineCorrectionPhase phase,
        const control::OnlineCorrectionTargetSnapshot &target);
    bool submitLossFrame(const CLossFrame &frame, qint64 triggerFrame, const QString &role);
    /// @brief 独立连续采集预算；从初始化开始，不以控制拒绝作为触发条件。
    bool recordReplayFrame(const CLossFrame &frame, QJsonObject context);
    std::unique_ptr<CTrackingDiagnosticImageWriter> m_replayImageWriter;
    quint64 m_replaySequence = 0;
    std::unique_ptr<CTrackingDiagnosticImageWriter> m_lossImageWriter;
    std::deque<CLossFrame> m_lossHistory;
    std::array<bool, 2> m_lossTriggered{};
    std::array<bool, 2> m_lossUnavailableTriggered{};
    std::array<bool, 2> m_lossSeenValid{};
    int m_lossStage = -1;
    int m_lossRejected = 0;
    int m_lossPostRemaining = 0;
    qint64 m_lossTriggerFrame = -1;
    qint64 m_lossLastFrame = -1;
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
    /// @brief 同一 worker 到达顺序；只归档，不推测未传播的曝光与控制因果关系。
    bool appendSessionEvent(QJsonObject event);
    /// @brief 归档实际释放批次、消费几何及待发命令，和实际回执分别记录。
    bool recordSessionUpdate(const control::OnlineCorrectionUpdate &update);
    void setFileError(const QString &operation, const QString &path);

    TextAppendFunction m_textAppend;
    std::unique_ptr<COnlineCorrectionTextWriter> m_textWriter;
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
    quint64 m_sessionEventCount = 0;
    quint64 m_icpBatchCount = 0;
    bool m_active = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_CORRECTION_RUN_RECORDER_H
