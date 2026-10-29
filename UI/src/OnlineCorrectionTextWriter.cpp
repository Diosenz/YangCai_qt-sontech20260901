/**
 * @file OnlineCorrectionTextWriter.cpp
 * @brief 慢文件系统与在线控制隔离；时间和来源由提交方冻结。
 */
#include "OnlineCorrectionTextWriter.h"

#include <exception>
#include <utility>

namespace i2r_rlc::ui {

COnlineCorrectionTextWriter::COnlineCorrectionTextWriter(AppendFunction append,
    qsizetype maxPendingBytes, std::size_t maxPendingChunks)
    : m_append(std::move(append)), m_maxPendingBytes(maxPendingBytes),
      m_maxPendingChunks(maxPendingChunks)
{
}

COnlineCorrectionTextWriter::~COnlineCorrectionTextWriter()
{
    stop();
}

void COnlineCorrectionTextWriter::start()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_started || m_stopping) { return; }
    m_thread = std::thread(&COnlineCorrectionTextWriter::run, this);
    m_started = true;
}

bool COnlineCorrectionTextWriter::submit(const QString &path, const QByteArray &data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_started || m_stopping || !m_lastError.isEmpty()) {
        ++m_rejectedChunks;
        return false;
    }
    if (data.size() > m_maxPendingBytes - m_pendingBytes
        || m_pendingChunks >= m_maxPendingChunks) {
        m_lastError = QStringLiteral("在线文本记录队列已满；记录不完整，控制继续运行。");
        ++m_rejectedChunks;
        return false;
    }
    m_pending.push_back({path, data});
    m_pendingBytes += data.size();
    ++m_pendingChunks;
    m_condition.notify_one();
    return true;
}

bool COnlineCorrectionTextWriter::flush()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_condition.wait(lock, [this]() { return m_pendingChunks == 0; });
    return m_lastError.isEmpty();
}

void COnlineCorrectionTextWriter::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopping = true;
    }
    m_condition.notify_all();
    if (m_thread.joinable()) { m_thread.join(); }
}

QString COnlineCorrectionTextWriter::lastError() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastError;
}

quint64 COnlineCorrectionTextWriter::rejectedChunks() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rejectedChunks;
}

void COnlineCorrectionTextWriter::run()
{
    for (;;) {
        CChunk chunk;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_condition.wait(lock, [this]() { return m_stopping || !m_pending.empty(); });
            if (m_pending.empty()) { return; }
            chunk = std::move(m_pending.front());
            m_pending.pop_front();
        }
        QString error;
        bool written = false;
        // 磁盘及外部回调可以阻塞，队列锁必须已释放，生产者仍能提交或报告满队列。
        try {
            written = m_append(chunk.path, chunk.data, &error);
        } catch (const std::exception &exception) {
            error = QString::fromUtf8(exception.what());
        } catch (...) {
            error = QStringLiteral("未知文件写入异常。");
        }
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!written && m_lastError.isEmpty()) {
                m_lastError = QStringLiteral("在线文本写入失败：%1；%2")
                    .arg(chunk.path, error);
            }
            m_pendingBytes -= chunk.data.size();
            --m_pendingChunks;
        }
        m_condition.notify_all();
    }
}

} // namespace i2r_rlc::ui
