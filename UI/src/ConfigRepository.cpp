/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        ConfigRepository.cpp
 * @brief       运行时配置仓库实现
 * @details     读取传感器标定、变位机标定和工件轨迹配置。
 ******************************************************************************/
#include <utility>

#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>
#include <QTextStream>
#include <QtGlobal>
#if QT_VERSION_MAJOR >= 6
#include <QStringConverter>
#endif

#include "ConfigRepository.h"

namespace i2r_rlc::ui {

ConfigRepository::ConfigRepository(AppPaths paths)
    : m_paths(std::move(paths))
{
}

QString ConfigRepository::runtimeRootText() const
{
    return m_paths.runtimeRootDir().absolutePath();
}

QString ConfigRepository::calibrationSearchText() const
{
    return QStringLiteral("SENSORPARA.txt: %1 | legacy: %2 | RBCALIB.DAT: %3 | CalibPose.txt: %4")
        .arg(
            m_paths.sensorCalibrationFilePath(),
            m_paths.legacySensorCalibrationFilePath(),
            m_paths.rbcalibFilePath(),
            m_paths.calibPoseTemplateFilePath());
}

bool ConfigRepository::ensureRuntimeDirectories(QString* errorMessage) const
{
    return m_paths.ensureRuntimeDirectories(errorMessage);
}

QString ConfigRepository::findSensorCalibrationFile() const
{
    const QString primaryPath = m_paths.sensorCalibrationFilePath();
    if (QFileInfo::exists(primaryPath)) {
        return primaryPath;
    }

    const QString legacyPath = m_paths.legacySensorCalibrationFilePath();
    if (QFileInfo::exists(legacyPath)) {
        return legacyPath;
    }

    return {};
}

SensorCalibrationParseResult ConfigRepository::loadSensorCalibration() const
{
    const QString filePath = findSensorCalibrationFile();
    if (filePath.isEmpty()) {
        return {false, {}, QStringLiteral("未找到 SENSORPARA.txt。已查找：%1；%2")
                             .arg(m_paths.sensorCalibrationFilePath(), m_paths.legacySensorCalibrationFilePath())};
    }

    return m_sensorCalibrationParser.parseFile(filePath);
}

vision::PositionerCalibration ConfigRepository::loadPositionerCalibration() const
{
    return vision::CPositionerCalibrationParser::parseFile(
        m_paths.rbcalibFilePath().toStdString());
}

WorkpieceProfileLoadResult ConfigRepository::loadWorkpieceProfiles() const
{
    WorkpieceProfileLoadResult result;
    result.filePath = m_paths.workpieceSelectionFilePath();

    QFile file(result.filePath);
    if (!file.exists()) {
        result.message = QStringLiteral("未找到工件选择配置文件：%1").arg(result.filePath);
        return result;
    }
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        result.message = QStringLiteral("无法打开工件选择配置文件：%1").arg(file.errorString());
        return result;
    }

    static const QRegularExpression idPattern(QStringLiteral("^[A-Za-z0-9_-]+$"));
    static const QRegularExpression commandNamePattern(QStringLiteral("^[A-Za-z0-9_-]+$"));
    QSet<QString> seenIds;
    QTextStream stream(&file);
#if QT_VERSION_MAJOR >= 6
    stream.setEncoding(QStringConverter::Utf8);
#else
    stream.setCodec("UTF-8");
#endif

    int lineNumber = 0;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString rawLine = stream.readLine();
        const QString line = rawLine.trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }

        const QStringList fields = rawLine.split(QLatin1Char('|'));
        if (fields.size() != 8 && fields.size() != 10) {
            result.message = QStringLiteral(
                                 "工件配置第 %1 行字段数量错误，应为 8 个或 10 个字段。")
                                 .arg(lineNumber);
            return result;
        }

        WorkpieceProfile profile;
        profile.id = fields.at(0).trimmed();
        profile.displayName = fields.at(1).trimmed();
        profile.weldmentModel = fields.at(2).trimmed();
        profile.offlineLocateJbi = fields.at(3).trimmed();
        profile.onlineJbi = fields.at(4).trimmed();
        if (fields.size() == 10) {
            profile.onlineGlobalCorrectionJbi = fields.at(5).trimmed();
            profile.onlineIbvsStartJbi = fields.at(6).trimmed();
            profile.offlineCorrectionJbi = fields.at(7).trimmed();
            profile.onlineTeachBaseName = fields.at(8).trimmed();
            profile.offlineCorrectionTeachBaseName = fields.at(9).trimmed();
        } else {
            profile.onlineGlobalCorrectionJbi = profile.onlineJbi;
            profile.onlineIbvsStartJbi = profile.onlineJbi;
            profile.offlineCorrectionJbi = fields.at(5).trimmed();
            profile.onlineTeachBaseName = fields.at(6).trimmed();
            profile.offlineCorrectionTeachBaseName = fields.at(7).trimmed();
        }

        if (profile.id.isEmpty() || profile.displayName.isEmpty()
            || profile.weldmentModel.isEmpty() || profile.offlineLocateJbi.isEmpty()
            || profile.onlineJbi.isEmpty()
            || profile.onlineGlobalCorrectionJbi.isEmpty()
            || profile.onlineIbvsStartJbi.isEmpty()
            || profile.offlineCorrectionJbi.isEmpty()
            || profile.onlineTeachBaseName.isEmpty()
            || profile.offlineCorrectionTeachBaseName.isEmpty()) {
            result.message = QStringLiteral("工件配置第 %1 行存在空字段。").arg(lineNumber);
            return result;
        }
        if (!idPattern.match(profile.id).hasMatch()) {
            result.message = QStringLiteral("工件配置第 %1 行 id 非法：%2")
                                 .arg(lineNumber)
                                 .arg(profile.id);
            return result;
        }
        if (seenIds.contains(profile.id)) {
            result.message = QStringLiteral("工件配置第 %1 行 id 重复：%2")
                                 .arg(lineNumber)
                                 .arg(profile.id);
            return result;
        }
        const QStringList jobNames = {
            profile.offlineLocateJbi,
            profile.onlineJbi,
            profile.onlineGlobalCorrectionJbi,
            profile.onlineIbvsStartJbi,
            profile.offlineCorrectionJbi
        };
        for (const QString& jobName : jobNames) {
            if (!commandNamePattern.match(jobName).hasMatch()) {
                result.message = QStringLiteral(
                    "工件配置第 %1 行 JBI 名非法：%2。JBI 名不能包含空白或逗号。")
                                     .arg(lineNumber)
                                     .arg(jobName);
                return result;
            }
        }
        const QStringList teachBaseNames = {
            profile.onlineTeachBaseName,
            profile.offlineCorrectionTeachBaseName
        };
        for (const QString& teachBaseName : teachBaseNames) {
            if (teachBaseName.endsWith(QStringLiteral("_W"), Qt::CaseInsensitive)
                || teachBaseName.endsWith(QStringLiteral("_T"), Qt::CaseInsensitive)
                || teachBaseName.endsWith(QStringLiteral("_W.txt"), Qt::CaseInsensitive)
                || teachBaseName.endsWith(QStringLiteral("_T.txt"), Qt::CaseInsensitive)) {
                result.message = QStringLiteral(
                    "工件配置第 %1 行示教轨迹名只能写基名，不要写 _W/_T 后缀。")
                                     .arg(lineNumber);
                return result;
            }
        }

        seenIds.insert(profile.id);
        result.profiles.push_back(profile);
    }

    if (result.profiles.isEmpty()) {
        result.message = QStringLiteral("工件选择配置为空：%1").arg(result.filePath);
        return result;
    }

    result.ok = true;
    result.message = QStringLiteral("工件选择配置读取完成，共 %1 个工件。")
                         .arg(result.profiles.size());
    return result;
}

bool ConfigRepository::loadOnlineIcpEnabled(
    bool &enabled, QString *errorMessage) const
{
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    const QString filePath = m_paths.workflowsDir().filePath(
        QStringLiteral("online_correction.txt"));
    QFile file(filePath);
    if (!file.exists()) {
        return true;
    }
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法打开在线纠偏配置：%1；%2")
                                .arg(filePath, file.errorString());
        }
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION_MAJOR >= 6
    stream.setEncoding(QStringConverter::Utf8);
#else
    stream.setCodec("UTF-8");
#endif
    bool hasSwitch = false;
    bool parsedEnabled = enabled;
    int lineNumber = 0;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }
        const int separator = line.indexOf(QLatin1Char('='));
        const QString key = line.left(separator).trimmed();
        const QString value = separator >= 0
            ? line.mid(separator + 1).trimmed()
            : QString();
        if (separator < 0 || key != QStringLiteral("enable_online_icp")
            || hasSwitch
            || (value != QStringLiteral("true")
                && value != QStringLiteral("false"))) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "在线纠偏配置第 %1 行无效：%2；仅允许一条 enable_online_icp=true 或 false。")
                                    .arg(lineNumber)
                                    .arg(filePath);
            }
            return false;
        }
        hasSwitch = true;
        parsedEnabled = value == QStringLiteral("true");
    }
    if (!hasSwitch || stream.status() != QTextStream::Ok) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "在线纠偏配置缺少有效 enable_online_icp 开关或读取失败：%1")
                                .arg(filePath);
        }
        return false;
    }

    enabled = parsedEnabled;
    return true;
}

bool ConfigRepository::loadOnlineVisionDiagnostics(
    COnlineVisionDiagnosticOptions &options, QString *errorMessage) const
{
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    QFile file(m_paths.workflowsDir().filePath(
        QStringLiteral("online_vision_diagnostics.txt")));
    if (!file.exists()) {
        return true;
    }
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法读取在线视觉诊断配置：%1").arg(file.fileName());
        }
        return false;
    }
    auto parsed = options;
    QSet<QString> keys;
    const QString content = QString::fromUtf8(file.readAll());
    bool valid = file.error() == QFile::NoError;
    int lineNumber = 0;
    for (const QString &rawLine : content.split(QLatin1Char('\n'))) {
        ++lineNumber;
        QString line = rawLine.trimmed();
        if (line.startsWith(QChar(0xfeff))) {
            line.remove(0, 1);
        }
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }
        const int separator = line.indexOf(QLatin1Char('='));
        const QString key = line.left(separator).trimmed();
        const QString value = line.mid(separator + 1).trimmed();
        valid = valid && separator > 0 && !keys.contains(key);
        keys.insert(key);
        if (key == QStringLiteral("enabled") || key == QStringLiteral("images_enabled")
            || key == QStringLiteral("replay_capture_enabled")) {
            valid = valid && (value == QStringLiteral("true") || value == QStringLiteral("false"));
            if (key == QStringLiteral("enabled")) {
                parsed.enabled = value == QStringLiteral("true");
            } else if (key == QStringLiteral("images_enabled")) {
                parsed.imagesEnabled = value == QStringLiteral("true");
            } else {
                parsed.replayCaptureEnabled = value == QStringLiteral("true");
            }
        } else {
            bool converted = false;
            const qint64 number = value.toLongLong(&converted);
            valid = valid && converted;
            if (key == QStringLiteral("image_sample_every_n")) {
                valid = valid && number >= 1 && number <= 1000000;
                if (valid) {
                    parsed.imageSampleEveryN = static_cast<int>(number);
                }
            } else if (key == QStringLiteral("max_image_pairs")) {
                valid = valid && number >= 0 && number <= 10000;
                if (valid) {
                    parsed.maxImagePairs = static_cast<int>(number);
                }
            } else if (key == QStringLiteral("replay_max_frames")) {
                valid = valid && number >= 1 && number <= 10000;
                if (valid) { parsed.replayMaxFrames = static_cast<int>(number); }
            } else if (key == QStringLiteral("replay_max_bytes")) {
                valid = valid && number >= 1 && number <= 1024LL * 1024 * 1024;
                if (valid) { parsed.replayMaxBytes = number; }
            } else if (key == QStringLiteral("max_image_bytes")) {
                valid = valid && number >= 0 && number <= 1024LL * 1024 * 1024;
                if (valid) {
                    parsed.maxImageBytes = number;
                }
            } else {
                valid = false;
            }
        }
        if (!valid) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("在线视觉诊断配置第 %1 行无效：%2")
                    .arg(lineNumber).arg(file.fileName());
            }
            return false;
        }
    }
    if (!valid) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("在线视觉诊断配置读取失败：%1").arg(file.fileName());
        }
        return false;
    }
    options = parsed;
    return true;
}

QString ConfigRepository::workpieceOnlineWorkPathFile(const WorkpieceProfile& profile) const
{
    return m_paths.workpieceTrajectoryDir().filePath(
        QStringLiteral("%1_W.txt").arg(profile.onlineTeachBaseName));
}

QString ConfigRepository::workpieceOnlineTeachPoseFile(const WorkpieceProfile& profile) const
{
    return m_paths.workpieceTrajectoryDir().filePath(
        QStringLiteral("%1_T.txt").arg(profile.onlineTeachBaseName));
}

QString ConfigRepository::workpieceOfflineCorrectionWorkPathFile(
    const WorkpieceProfile& profile) const
{
    return m_paths.workpieceTrajectoryDir().filePath(
        QStringLiteral("%1_W.txt").arg(profile.offlineCorrectionTeachBaseName));
}

QString ConfigRepository::workpieceOfflineCorrectionTeachPoseFile(
    const WorkpieceProfile& profile) const
{
    return m_paths.workpieceTrajectoryDir().filePath(
        QStringLiteral("%1_T.txt").arg(profile.offlineCorrectionTeachBaseName));
}

const AppPaths& ConfigRepository::paths() const
{
    return m_paths;
}

} // namespace i2r_rlc::ui
