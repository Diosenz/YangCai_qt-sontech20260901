/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        AppError.h
 * @brief       应用错误值对象
 * @details     定义 UI 层可展示、可记录的跨模块错误信息。
 ******************************************************************************/
#pragma once

#include <QString>

namespace i2r_rlc::ui {

/**
 * @brief 应用错误信息。
 *
 * 用于跨模块传递可显示、可记录的错误，不要求 UI 了解具体后端实现。
 */
struct AppError {
    /// @brief 错误来源模块。
    QString source;

    /// @brief 面向用户或日志的错误说明。
    QString message;

    /// @brief 是否属于安全相关错误。
    bool safetyCritical = false;
};

} // namespace i2r_rlc::ui
