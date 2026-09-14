/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        SensorCalibrationParser.h
 * @brief       传感器标定文件解析器
 * @details     解析 legacy SENSORPARA.txt，并输出新工程定位/投影可复用的数据。
 ******************************************************************************/
#pragma once

#include <QString>

#include "CalibrationData.h"

namespace i2r_rlc::ui {

/**
 * @brief SENSORPARA.txt 解析结果。
 *
 * 解析失败时 `ok` 为 false，`errorMessage` 包含结构化错误说明；调用方不应
 * 在解析器内弹窗，而应通过日志或 UI 状态显示错误。
 */
struct SensorCalibrationParseResult {
    /// @brief 是否解析成功。
    bool ok = false;

    /// @brief 解析成功时的传感器标定数据。
    i2r_rlc::localization::SensorCalibrationData data;

    /// @brief 解析失败或警告时的说明文本。
    QString errorMessage;
};

/**
 * @brief legacy SENSORPARA.txt 解析器。
 *
 * 支持字段名中包含 `/m`、`/deg`、`[0]` 等字符；会忽略空行、`More Info`
 * 行、日期行和未知行。关键字段缺失或数值格式错误会返回失败结果。
 */
class SensorCalibrationParser {
public:
    /**
     * @brief 从文件解析 SENSORPARA.txt。
     */
    SensorCalibrationParseResult parseFile(const QString& filePath) const;

    /**
     * @brief 从文本内容解析 SENSORPARA.txt。
     *
     * @param content 文件内容。
     * @param sourceFile 数据来源文件路径，用于写入结果和错误信息。
     */
    SensorCalibrationParseResult parseText(const QString& content, const QString& sourceFile = {}) const;
};

} // namespace i2r_rlc::ui
