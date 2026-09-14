/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CalibrationGuideOverlay.h
 * @brief       标定采集引导叠加层
 * @details     声明自动标定预览红框的绘制结果和纯函数接口。
 ******************************************************************************/
#ifndef I2R_RLC_UI_CALIBRATION_GUIDE_OVERLAY_H
#define I2R_RLC_UI_CALIBRATION_GUIDE_OVERLAY_H

#include <QImage>

#include "CalibrationCaptureTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief 自动标定红框绘制结果。
 */
struct CalibrationGuideOverlayResult {
    /// @brief 绘制后的图像；输入无效时保持空图。
    QImage image;
    /// @brief true 表示已绘制红色提示框。
    bool drawn = false;
    /// @brief true 表示旧固定红框超出当前图像并被裁剪。
    bool clipped = false;
};

/**
 * @brief 在 UI 图像上绘制旧工程自动标定红色提示框。
 *
 * 红框仅用于显示引导，不应回写到标定采集保存的原始图像。
 */
CalibrationGuideOverlayResult renderCalibrationGuideOverlay(
    const QImage &source,
    const control::CalibrationCaptureUpdate &update);

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_CALIBRATION_GUIDE_OVERLAY_H
