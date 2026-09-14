/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        Logger.h
 * @brief       UI 日志格式化工具
 * @details     提供带时间戳的日志格式化，并写入有限大小的滚动日志文件。
 ******************************************************************************/
#pragma once

#include <QString>

namespace i2r_rlc::ui {

/**
 * @brief 最小日志格式化工具。
 *
 * UI 线程调用 info()/warning() 时同步追加日志；单文件达到 5 MiB 后滚动，
 * 最多保留当前文件和三个历史文件。写文件失败不阻塞现场业务。
 */
class Logger {
public:
    /// @brief 格式化一条普通日志。
    static QString info(const QString& message);

    /// @brief 格式化一条警告日志。
    static QString warning(const QString& message);

    /// @brief 返回当前滚动日志文件的绝对路径。
    static QString logFilePath();
};

} // namespace i2r_rlc::ui
