/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        SensorCalibrationParser.cpp
 * @brief       传感器标定文件解析实现
 * @details     兼容 legacy SENSORPARA.txt 字段名和单位格式。
 ******************************************************************************/
#include <algorithm>

#include <QFile>
#include <QLocale>
#include <QMap>
#include <QRegularExpression>
#include <QStringList>

#include "SensorCalibrationParser.h"

namespace i2r_rlc::ui {

namespace {

const QStringList kRequiredFields = {
    QStringLiteral("Calib_Method"),
    QStringLiteral("f/m"),
    QStringLiteral("Kappa"),
    QStringLiteral("sx/m"),
    QStringLiteral("sy/m"),
    QStringLiteral("cx"),
    QStringLiteral("cy"),
    QStringLiteral("a"),
    QStringLiteral("b"),
    QStringLiteral("c"),
    QStringLiteral("pose_CinT[0]/m"),
    QStringLiteral("pose_CinT[1]/m"),
    QStringLiteral("pose_CinT[2]/m"),
    QStringLiteral("pose_CinT[3]/deg"),
    QStringLiteral("pose_CinT[4]/deg"),
    QStringLiteral("pose_CinT[5]/deg"),
    QStringLiteral("pose_CinT[6]")
};

bool isDateLine(const QString& line)
{
    static const QRegularExpression datePattern(
        QStringLiteral("^(Mon|Tue|Wed|Thu|Fri|Sat|Sun)\\s+"
                       "(Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec)\\s+"));
    return datePattern.match(line).hasMatch();
}

bool shouldIgnoreLine(const QString& line)
{
    return line.isEmpty()
        || line.startsWith(QStringLiteral("More Info"), Qt::CaseInsensitive)
        || isDateLine(line);
}

QString matchedFieldName(const QString& line)
{
    QStringList fields = kRequiredFields;
    std::sort(fields.begin(), fields.end(), [](const QString& left, const QString& right) {
        return left.size() > right.size();
    });

    for (const auto& field : fields) {
        if (line == field || line.startsWith(field + QLatin1Char(' ')) || line.startsWith(field + QLatin1Char('\t'))) {
            return field;
        }
    }

    return {};
}

bool parseDouble(const QString& fieldName, const QString& rawValue, double* value, QString* errorMessage)
{
    bool ok = false;
    const double parsed = QLocale::c().toDouble(rawValue.trimmed(), &ok);
    if (!ok) {
        *errorMessage = QStringLiteral("字段 `%1` 的数值格式无效：`%2`").arg(fieldName, rawValue);
        return false;
    }

    *value = parsed;
    return true;
}

} // namespace

SensorCalibrationParseResult SensorCalibrationParser::parseFile(const QString& filePath) const
{
    QFile file(filePath);
    if (!file.exists()) {
        return {false, {}, QStringLiteral("标定文件不存在：%1").arg(filePath)};
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {false, {}, QStringLiteral("无法打开标定文件：%1，原因：%2").arg(filePath, file.errorString())};
    }

    return parseText(QString::fromUtf8(file.readAll()), filePath);
}

SensorCalibrationParseResult SensorCalibrationParser::parseText(const QString& content, const QString& sourceFile) const
{
    i2r_rlc::localization::SensorCalibrationData data;
    data.sourceFile = sourceFile;

    QMap<QString, QString> values;
    const auto lines = content.split(QRegularExpression(QStringLiteral("\\r?\\n")));
    for (const auto& originalLine : lines) {
        QString line = originalLine;
        line.remove(QChar(0xFEFF));
        line = line.trimmed();
        if (shouldIgnoreLine(line)) {
            continue;
        }

        const QString field = matchedFieldName(line);
        if (field.isEmpty()) {
            continue;
        }

        const QString rawValue = line.mid(field.size()).trimmed();
        if (rawValue.isEmpty()) {
            return {false, data, QStringLiteral("字段 `%1` 缺少值，文件：%2").arg(field, sourceFile)};
        }

        values.insert(field, rawValue);
    }

    QStringList missingFields;
    for (const auto& field : kRequiredFields) {
        if (!values.contains(field)) {
            missingFields.append(field);
        }
    }

    if (!missingFields.isEmpty()) {
        return {false, data, QStringLiteral("SENSORPARA.txt 缺少关键字段：%1，文件：%2")
                                 .arg(missingFields.join(QStringLiteral(", ")), sourceFile)};
    }

    data.calibMethod = values.value(QStringLiteral("Calib_Method"));

    QString errorMessage;
    auto setDouble = [&](const QString& fieldName, double* target) {
        return parseDouble(fieldName, values.value(fieldName), target, &errorMessage);
    };

    if (!setDouble(QStringLiteral("f/m"), &data.focalLengthM)
        || !setDouble(QStringLiteral("Kappa"), &data.kappa)
        || !setDouble(QStringLiteral("sx/m"), &data.sxM)
        || !setDouble(QStringLiteral("sy/m"), &data.syM)
        || !setDouble(QStringLiteral("cx"), &data.cxPixel)
        || !setDouble(QStringLiteral("cy"), &data.cyPixel)
        || !setDouble(QStringLiteral("a"), &data.laserPlaneA)
        || !setDouble(QStringLiteral("b"), &data.laserPlaneB)
        || !setDouble(QStringLiteral("c"), &data.laserPlaneC)
        || !setDouble(QStringLiteral("pose_CinT[0]/m"), &data.poseCinTXM)
        || !setDouble(QStringLiteral("pose_CinT[1]/m"), &data.poseCinTYM)
        || !setDouble(QStringLiteral("pose_CinT[2]/m"), &data.poseCinTZM)
        || !setDouble(QStringLiteral("pose_CinT[3]/deg"), &data.poseCinTRxDeg)
        || !setDouble(QStringLiteral("pose_CinT[4]/deg"), &data.poseCinTRyDeg)
        || !setDouble(QStringLiteral("pose_CinT[5]/deg"), &data.poseCinTRzDeg)
        || !setDouble(QStringLiteral("pose_CinT[6]"), &data.poseCinTReserved)) {
        return {false, data, QStringLiteral("%1，文件：%2").arg(errorMessage, sourceFile)};
    }

    return {true, data, {}};
}

} // namespace i2r_rlc::ui
