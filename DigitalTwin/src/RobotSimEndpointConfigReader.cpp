/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimEndpointConfigReader.cpp
 * @brief     RobotSim endpoint 配置读取实现。
 * @details   本文件实现 host、port 和超时参数的 key-value 文本解析，并返回
 *            结构化加载结果。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include <QFile>
#include <QTextStream>

#include "RobotSimEndpointConfigReader.h"

namespace i2r_rlc::digital_twin {

namespace {

bool parsePositiveInt(const QString& value, int* output)
{
    bool ok = false;
    const int parsed = value.toInt(&ok);
    if (!ok || parsed <= 0) {
        return false;
    }

    *output = parsed;
    return true;
}

bool parseNonNegativeInt(const QString& value, int* output)
{
    bool ok = false;
    const int parsed = value.toInt(&ok);
    if (!ok || parsed < 0) {
        return false;
    }

    *output = parsed;
    return true;
}

bool parsePort(const QString& value, quint16* output)
{
    bool ok = false;
    const uint parsed = value.toUInt(&ok);
    if (!ok || parsed == 0 || parsed > 65535U) {
        return false;
    }

    *output = static_cast<quint16>(parsed);
    return true;
}

} // namespace

RobotSimEndpointConfigLoadResult RobotSimEndpointConfigReader::readFile(
    const QString& filePath) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {
            false,
            {},
            QStringLiteral("无法打开 RobotSim endpoint 配置：%1").arg(filePath)
        };
    }

    RobotSimEndpointConfig config;
    QTextStream stream(&file);
    int lineNumber = 0;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }

        const int separator = line.indexOf(QLatin1Char('='));
        if (separator <= 0) {
            return {
                false,
                {},
                QStringLiteral("RobotSim endpoint 配置第 %1 行缺少 '='。").arg(lineNumber)
            };
        }

        const QString key = line.left(separator).trimmed();
        const QString value = line.mid(separator + 1).trimmed();
        if (key == QStringLiteral("host")) {
            config.host = value;
        } else if (key == QStringLiteral("port")) {
            if (!parsePort(value, &config.port)) {
                return {
                    false,
                    {},
                    QStringLiteral("RobotSim endpoint 配置第 %1 行 port 无效。").arg(lineNumber)
                };
            }
        } else if (key == QStringLiteral("connect_timeout_ms")) {
            if (!parsePositiveInt(value, &config.connectTimeoutMs)) {
                return {
                    false,
                    {},
                    QStringLiteral("RobotSim endpoint 配置第 %1 行 connect_timeout_ms 无效。").arg(lineNumber)
                };
            }
        } else if (key == QStringLiteral("request_timeout_ms")) {
            if (!parsePositiveInt(value, &config.requestTimeoutMs)) {
                return {
                    false,
                    {},
                    QStringLiteral("RobotSim endpoint 配置第 %1 行 request_timeout_ms 无效。").arg(lineNumber)
                };
            }
        } else if (key == QStringLiteral("reference_object_id")) {
            if (!parseNonNegativeInt(value, &config.referenceObjectId)) {
                return {
                    false,
                    {},
                    QStringLiteral("RobotSim endpoint 配置第 %1 行 reference_object_id 无效。").arg(lineNumber)
                };
            }
        }
    }

    if (config.host.trimmed().isEmpty() || config.port == 0) {
        return {
            false,
            {},
            QStringLiteral("RobotSim endpoint 配置必须包含有效 host 和 port。")
        };
    }

    return {
        true,
        config,
        QStringLiteral("RobotSim endpoint 配置读取成功。")
    };
}

} // namespace i2r_rlc::digital_twin
