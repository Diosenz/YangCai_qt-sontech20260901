/**
 * @file TrackingDiagnosticImageWriter.h
 * @brief 有界图像诊断写入器，隔离 PNG 编码与在线控制线程。
 */
#ifndef I2R_RLC_UI_TRACKING_DIAGNOSTIC_IMAGE_WRITER_H
#define I2R_RLC_UI_TRACKING_DIAGNOSTIC_IMAGE_WRITER_H

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include <QImage>
#include <QString>

namespace i2r_rlc::ui {

/**
 * @brief 每会话拥有一个可回收后台线程；调用者仅在在线 worker 中提交。
 *
 * 最多两对待写图；满队列立即跳过。stop 幂等并排空后 join，无 detached 线程。
 * 写入结果保存在独立 manifest，绝不删除旧图。该类不可复制。
 */
class CTrackingDiagnosticImageWriter final {
public:
    CTrackingDiagnosticImageWriter(const QString &directory, qint64 maxBytes);
    ~CTrackingDiagnosticImageWriter();
    /// @brief 非阻塞入队；返回 queued、queue_full 或 stopped。
    QString submit(const QString &stem, const QImage &source, const QImage &enhanced);
    /// @brief 排空有界队列并回收线程；仅在会话结束或重配时调用。
    void stop();
    /// @brief 返回后台编码/写入错误；预算和队列跳过不视为错误。
    QString lastError();

private:
    struct CImagePair {
        QString stem;
        QImage source;
        QImage enhanced;
    };
    void run();
    void writePair(const CImagePair &pair);
    void recordOutcome(const QString &stem, const QString &status, qint64 bytes);

    QString m_lastError;
    QString m_directory;
    qint64 m_maxBytes = 0;
    qint64 m_writtenBytes = 0;
    bool m_stopping = false;
    std::mutex m_mutex;
    std::condition_variable m_condition;
    std::deque<CImagePair> m_pending;
    std::thread m_thread;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_TRACKING_DIAGNOSTIC_IMAGE_WRITER_H
