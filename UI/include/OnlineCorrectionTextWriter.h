/**
 * @file OnlineCorrectionTextWriter.h
 * @brief 在线会话文本的有界后台写入与排空接口。
 */
#ifndef I2R_RLC_UI_ONLINE_CORRECTION_TEXT_WRITER_H
#define I2R_RLC_UI_ONLINE_CORRECTION_TEXT_WRITER_H

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include <QByteArray>
#include <QString>

namespace i2r_rlc::ui {

/**
 * @brief recorder 独占的文本写入线程，不持有业务对象或 widgets。
 * @details 调用者串行提交已冻结的文件路径和字节，后台保持入队顺序。
 * 队列字节/条数包含正在写入的一条；满队列或写失败锁存错误，后续提交立即失败。
 * 文件回调不在队列锁内执行。stop 排空已接纳数据后 join，不使用 detached 线程。
 */
class COnlineCorrectionTextWriter final {
public:
    /// @brief 文件系统追加边界；成功须包含刷新检查，不得回调控制对象或 widgets。
    using AppendFunction = std::function<bool(const QString &, const QByteArray &, QString *)>;
    COnlineCorrectionTextWriter(AppendFunction append, qsizetype maxPendingBytes,
        std::size_t maxPendingChunks);
    ~COnlineCorrectionTextWriter();
    /// @brief 在会话准备阶段启动；线程创建失败由调用者处理，不能重复启动。
    void start();
    /// @brief 非阻塞提交值快照；true 只表示接纳，不表示已经落盘。
    bool submit(const QString &path, const QByteArray &data);
    /// @brief 仅离线导出/会话结束等待已接纳写入；禁止在在线控制周期调用。
    bool flush();
    /// @brief 停止接纳并排空；由唯一所有者调用，幂等，可能等待慢盘完成。
    void stop();
    /// @brief 返回已锁存的首个错误，不因后续成功写入而清除。
    QString lastError() const;
    /// @brief 未接纳的数据块数，用于明确声明会话记录不完整。
    quint64 rejectedChunks() const;

private:
    struct CChunk {
        QString path;
        QByteArray data;
    };
    void run();

    AppendFunction m_append;
    qsizetype m_maxPendingBytes;
    std::size_t m_maxPendingChunks;
    qsizetype m_pendingBytes = 0;
    std::size_t m_pendingChunks = 0;
    quint64 m_rejectedChunks = 0;
    bool m_stopping = false;
    bool m_started = false;
    QString m_lastError;
    mutable std::mutex m_mutex;
    std::condition_variable m_condition;
    std::deque<CChunk> m_pending;
    std::thread m_thread;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_CORRECTION_TEXT_WRITER_H
