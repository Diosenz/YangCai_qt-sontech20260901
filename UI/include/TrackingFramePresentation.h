/**
 * @file TrackingFramePresentation.h
 * @brief 检测图像与全部叠加元素的一次性显示数据。
 *
 * 由 Controller 从同帧结果组装，View 只负责绘制。所有坐标均为原图像素，
 * 图像按值共享只读存储；不得把该快照的点叠加到后续相机帧。
 */
#ifndef I2R_RLC_UI_TRACKING_FRAME_PRESENTATION_H
#define I2R_RLC_UI_TRACKING_FRAME_PRESENTATION_H

#include <QImage>
#include <QPointF>
#include <QString>
#include <QVector>

namespace i2r_rlc::ui {

/// @brief 与具体跟踪算法无关的完整显示快照，仅在 UI 线程组装和消费。
struct TrackingFramePresentation {
    QImage sourceImage;
    QVector<QPointF> virtualLaserPoints;
    QPointF virtualWeldPoint;
    bool virtualWeldPointVisible = false;
    QPointF detectedWeldPoint;
    bool detectedWeldPointVisible = false;
    /// @brief 黄色表示估计/控制参考显示；绿色保留给有效测量。
    bool detectedWeldPointEstimated = false;
    QString caption;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_TRACKING_FRAME_PRESENTATION_H
