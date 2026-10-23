/**
 * @file TrackingDiagnosticImageWriter.cpp
 * @brief 后台无损图像采样及实际保存结果记录。
 */
#include "TrackingDiagnosticImageWriter.h"

#include <exception>
#include <utility>

#include <QBuffer>
#include <QDir>
#include <QFile>
#include <QStorageInfo>

namespace i2r_rlc::ui {

CTrackingDiagnosticImageWriter::CTrackingDiagnosticImageWriter(
    const QString &directory, qint64 maxBytes)
    : m_directory(directory), m_maxBytes(maxBytes)
{
    m_thread = std::thread([this]() { run(); });
}

CTrackingDiagnosticImageWriter::~CTrackingDiagnosticImageWriter()
{
    stop();
}

QString CTrackingDiagnosticImageWriter::submit(
    const QString &stem, const QImage &source, const QImage &enhanced,
    int bucket, qint64 bucketByteBudget)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_stopping) {
        return QStringLiteral("stopped");
    }
    if (m_pending.size() >= 2U) {
        return QStringLiteral("queue_full");
    }
    if (bucket < 0 || bucket >= 5 || bucketByteBudget <= 0) {
        return QStringLiteral("invalid_bucket");
    }
    m_pending.push_back({stem, source, enhanced, bucket, bucketByteBudget});
    m_condition.notify_one();
    return QStringLiteral("queued");
}

void CTrackingDiagnosticImageWriter::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopping = true;
    }
    m_condition.notify_one();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

QString CTrackingDiagnosticImageWriter::lastError()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastError;
}

void CTrackingDiagnosticImageWriter::run()
{
    for (;;) {
        CImagePair pair;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_condition.wait(lock, [this]() { return m_stopping || !m_pending.empty(); });
            if (m_pending.empty()) {
                return;
            }
            pair = std::move(m_pending.front());
            m_pending.pop_front();
        }
        try {
            writePair(pair);
        } catch (const std::exception &) {
            recordOutcome(pair.stem, QStringLiteral("worker_exception"), 0);
        }
    }
}

void CTrackingDiagnosticImageWriter::recordOutcome(
    const QString &stem, const QString &status, qint64 bytes)
{
    QFile file(QDir(m_directory).filePath(QStringLiteral("image_manifest.tsv")));
    const QByteArray row = QStringLiteral("%1\t%2\t%3\n")
        .arg(stem, status).arg(bytes).toUtf8();
    const bool manifestWritten = file.open(QIODevice::WriteOnly | QIODevice::Append)
        && file.write(row) == row.size() && file.flush();
    if (!manifestWritten || status.endsWith(QStringLiteral("failed"))
        || status == QStringLiteral("worker_exception")) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_lastError = QStringLiteral("诊断图像 %1：%2%3")
            .arg(stem, status, manifestWritten ? QString()
                 : QStringLiteral("；manifest 写入失败"));
    }
}

void CTrackingDiagnosticImageWriter::writePair(const CImagePair &pair)
{
    // PNG 编码和磁盘查询只发生在本线程；预算按实际编码字节计算。
    QStorageInfo storage(m_directory);
    const qint64 reserve = 512LL * 1024 * 1024;
    if (!storage.isValid() || !storage.isReady()
        || storage.bytesAvailable() < reserve || m_writtenBytes >= m_maxBytes
        || m_bucketWrittenBytes[pair.bucket] >= pair.bucketByteBudget) {
        recordOutcome(pair.stem, QStringLiteral("budget_or_disk_limit"), 0);
        return;
    }
    QByteArray sourceData;
    QByteArray enhancedData;
    QBuffer sourceBuffer(&sourceData);
    QBuffer enhancedBuffer(&enhancedData);
    sourceBuffer.open(QIODevice::WriteOnly);
    enhancedBuffer.open(QIODevice::WriteOnly);
    if (!pair.source.save(&sourceBuffer, "PNG")
        || !pair.enhanced.save(&enhancedBuffer, "PNG")) {
        recordOutcome(pair.stem, QStringLiteral("encode_failed"), 0);
        return;
    }
    const qint64 bytes = sourceData.size() + enhancedData.size();
    storage.refresh();
    if (!storage.isValid() || !storage.isReady()
        || bytes > m_maxBytes - m_writtenBytes
        || bytes > pair.bucketByteBudget - m_bucketWrittenBytes[pair.bucket]
        || bytes > storage.bytesAvailable() - reserve) {
        recordOutcome(pair.stem, QStringLiteral("budget_or_disk_limit"), 0);
        return;
    }
    const auto write = [this, &pair](const QString &name, const QByteArray &data) {
        QFile file(QDir(m_directory).filePath(name));
        if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly)) {
            return false;
        }
        const qint64 written = file.write(data);
        // 失败后的部分文件也占用预算，保留用于核查，不回收或覆盖。
        if (written > 0) {
            m_writtenBytes += written;
            m_bucketWrittenBytes[pair.bucket] += written;
        }
        return written == data.size() && file.flush();
    };
    const bool sourceWritten = write(pair.stem + QStringLiteral("_source.png"), sourceData);
    const bool enhancedWritten = sourceWritten
        && write(pair.stem + QStringLiteral("_enhanced.png"), enhancedData);
    recordOutcome(pair.stem, enhancedWritten ? QStringLiteral("saved")
                                             : QStringLiteral("write_failed"), bytes);
}

} // namespace i2r_rlc::ui
