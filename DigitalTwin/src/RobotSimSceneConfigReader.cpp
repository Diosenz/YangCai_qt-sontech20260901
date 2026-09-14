/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimSceneConfigReader.cpp
 * @brief     RobotSim 场景对象 ID 配置读取实现。
 */

#include "RobotSimSceneConfigReader.h"

#include <QFile>
#include <QTextStream>

namespace i2r_rlc::digital_twin {
namespace {

bool parsePositiveId(const QString &text, int *value)
{
    if (value == nullptr) {
        return false;
    }
    bool ok = false;
    const int parsed = text.trimmed().toInt(&ok);
    if (!ok || parsed <= 0) {
        return false;
    }
    *value = parsed;
    return true;
}

} // namespace

RobotSimSceneConfigLoadResult CRobotSimSceneConfigReader::load(
    const QString &filePath)
{
    RobotSimSceneConfigLoadResult result;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        result.message = QStringLiteral("无法打开 RobotSim 场景配置：%1")
                             .arg(filePath);
        return result;
    }

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
            result.message = QStringLiteral(
                "RobotSim 场景配置第 %1 行缺少 '='。")
                                 .arg(lineNumber);
            return result;
        }

        const QString key = line.left(separator).trimmed();
        const QString value = line.mid(separator + 1).trimmed();
        if (key == QStringLiteral("scene_name")) {
            result.config.sceneName = value;
        }
        else if (key == QStringLiteral("robot_id")) {
            if (!parsePositiveId(value, &result.config.robotId)) {
                result.message = QStringLiteral(
                    "RobotSim 场景配置第 %1 行 robot_id 无效。")
                                     .arg(lineNumber);
                return result;
            }
        }
        else if (key == QStringLiteral("positioner_id")) {
            if (!parsePositiveId(value, &result.config.positionerId)) {
                result.message = QStringLiteral(
                    "RobotSim 场景配置第 %1 行 positioner_id 无效。")
                                     .arg(lineNumber);
                return result;
            }
        }
        else if (key == QStringLiteral("sensor_id")) {
            if (!parsePositiveId(value, &result.config.sensorId)) {
                result.message = QStringLiteral(
                    "RobotSim 场景配置第 %1 行 sensor_id 无效。")
                                     .arg(lineNumber);
                return result;
            }
        }
        else if (key == QStringLiteral("workpiece_id")) {
            if (!parsePositiveId(value, &result.config.workpieceId)) {
                result.message = QStringLiteral(
                    "RobotSim 场景配置第 %1 行 workpiece_id 无效。")
                                     .arg(lineNumber);
                return result;
            }
        }
        else if (key == QStringLiteral("workpiece_parent_id")) {
            if (!parsePositiveId(value, &result.config.workpieceParentId)) {
                result.message = QStringLiteral(
                    "RobotSim 场景配置第 %1 行 workpiece_parent_id 无效。")
                                     .arg(lineNumber);
                return result;
            }
        }
    }

    if (result.config.robotId <= 0
        || result.config.positionerId <= 0
        || result.config.sensorId <= 0) {
        result.message = QStringLiteral(
            "RobotSim 场景配置必须包含有效 robot_id、positioner_id 和 sensor_id。");
        return result;
    }

    result.succeeded = true;
    result.message = QStringLiteral("RobotSim 场景配置读取成功。");
    return result;
}

} // namespace i2r_rlc::digital_twin
