/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        Logger.cpp
 * @brief       UI 日志格式化实现
 * @details     生成带时间戳和等级的文本，并追加到有限大小的滚动日志文件。
 ******************************************************************************/
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QStandardPaths>

#include "Logger.h"

namespace i2r_rlc::ui {

namespace {

constexpr qint64 MAXIMUM_LOG_FILE_SIZE_BYTES = 5LL * 1024LL * 1024LL;
constexpr int RETAINED_LOG_FILE_COUNT = 3;

QString formatLine(const QString& level, const QString& message)
{
    return QStringLiteral("[%1] [%2] %3")
        .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs), level, message);
}

QString resolveLogFilePath()
{
    QString root = QStandardPaths::writableLocation(
        QStandardPaths::AppLocalDataLocation);
    if (root.isEmpty()) {
        root = QCoreApplication::applicationDirPath();
    }
    const QString directory = QDir(root).filePath(QStringLiteral("logs"));
    QDir().mkpath(directory);
    return QDir(directory).filePath(QStringLiteral("i2r_rlc.log"));
}

void rotateLogFiles(const QString& path)
{
    QFile::remove(QStringLiteral("%1.%2").arg(path).arg(RETAINED_LOG_FILE_COUNT));
    for (int index = RETAINED_LOG_FILE_COUNT - 1; index >= 1; --index) {
        const QString source = QStringLiteral("%1.%2").arg(path).arg(index);
        const QString target = QStringLiteral("%1.%2").arg(path).arg(index + 1);
        if (QFileInfo::exists(source)) {
            QFile::rename(source, target);
        }
    }
    if (QFileInfo::exists(path)) {
        QFile::rename(path, QStringLiteral("%1.1").arg(path));
    }
}

QString formatAndPersist(const QString& level, const QString& message)
{
    static QMutex mutex;
    const QString line = formatLine(level, message);
    const QByteArray bytes = line.toUtf8() + '\n';
    const QString path = resolveLogFilePath();
    QMutexLocker locker(&mutex);
    const QFileInfo fileInfo(path);
    if (fileInfo.exists()
        && fileInfo.size() + bytes.size() > MAXIMUM_LOG_FILE_SIZE_BYTES) {
        rotateLogFiles(path);
    }

    QFile file(path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
        file.write(bytes);
    }
    return line;
}

} // namespace

QString Logger::info(const QString& message)
{
    return formatAndPersist(QStringLiteral("INFO"), message);
}

QString Logger::warning(const QString& message)
{
    return formatAndPersist(QStringLiteral("WARN"), message);
}

QString Logger::logFilePath()
{
    return resolveLogFilePath();
}

} // namespace i2r_rlc::ui
