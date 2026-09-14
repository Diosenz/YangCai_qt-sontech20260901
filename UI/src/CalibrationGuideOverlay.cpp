/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CalibrationGuideOverlay.cpp
 * @brief       标定采集引导叠加层实现
 * @details     在预览图上绘制旧工程自动标定红框，不污染原始采集图像。
 ******************************************************************************/
#include <algorithm>

#include <QColor>
#include <QPainter>
#include <QPen>
#include <QPoint>
#include <QRect>

#include "CalibrationGuideOverlay.h"

namespace i2r_rlc::ui {

CalibrationGuideOverlayResult renderCalibrationGuideOverlay(
    const QImage &source,
    const control::CalibrationCaptureUpdate &update)
{
    CalibrationGuideOverlayResult result;
    result.image = source;
    if (!update.automaticGuideOverlayActive || source.isNull()
        || source.width() <= 0 || source.height() <= 0) {
        return result;
    }

    const int requestedLeft =
        std::min(update.automaticGuideLeftPixel, update.automaticGuideRightPixel);
    const int requestedRight =
        std::max(update.automaticGuideLeftPixel, update.automaticGuideRightPixel);
    const int requestedTop =
        std::min(update.automaticGuideTopPixel, update.automaticGuideBottomPixel);
    const int requestedBottom =
        std::max(update.automaticGuideTopPixel, update.automaticGuideBottomPixel);

    const int left = std::clamp(requestedLeft, 0, source.width() - 1);
    const int right = std::clamp(requestedRight, 0, source.width() - 1);
    const int top = std::clamp(requestedTop, 0, source.height() - 1);
    const int bottom = std::clamp(requestedBottom, 0, source.height() - 1);
    result.clipped = left != requestedLeft || right != requestedRight
        || top != requestedTop || bottom != requestedBottom;

    if (right < left || bottom < top) {
        return result;
    }

    if (result.image.format() == QImage::Format_Grayscale8
        || result.image.format() == QImage::Format_Indexed8
        || result.image.format() == QImage::Format_Mono
        || result.image.format() == QImage::Format_MonoLSB) {
        result.image = result.image.convertToFormat(QImage::Format_RGB32);
    }

    QPainter painter(&result.image);
    QPen pen(QColor(255, 0, 0));
    pen.setWidth(std::max(1, update.automaticGuideLineWidthPixel));
    painter.setPen(pen);
    painter.drawRect(QRect(QPoint(left, top), QPoint(right, bottom)));

    result.drawn = true;
    return result;
}

} // namespace i2r_rlc::ui
