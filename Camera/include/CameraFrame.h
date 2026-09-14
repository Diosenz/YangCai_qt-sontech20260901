/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CameraFrame.h
 * @brief       相机帧数据类型
 * @details     定义相机角色和跨 UI/Camera 边界传递的帧数据。
 ******************************************************************************/
#ifndef I2R_RLC_CAMERA_CAMERA_FRAME_H
#define I2R_RLC_CAMERA_CAMERA_FRAME_H

#include <QDateTime>
#include <QImage>
#include <QString>

namespace i2r_rlc::camera {

/**
 * @brief 相机角色标识。
 *
 * 用于区分焊缝跟踪、IBVS 和离线回放链路。真实相机 SDK 后续只在
 * camera 模块内部适配，不向 UI 暴露设备对象。
 */
enum class CameraId {
    OfflineReplay,
    VimbaTracking,
    BaslerIbvs
};

/**
 * @brief 相机帧数据。
 *
 * Camera 模块对外使用 `QImage` 承载离线与真实相机图像。OpenCV 和厂商 SDK
 * 只允许停留在 Camera 模块内部适配层，UI 仍只接收安全的显示数据。
 */
struct CameraFrame {
    /// @brief 相机角色。
    CameraId cameraId = CameraId::OfflineReplay;

    /// @brief 帧 ID，从 0 开始递增，单位为帧。
    qint64 frameId = 0;

    /// @brief 兼容旧骨架命名的帧序号，从 0 开始递增，单位为帧。
    qint64 frameNumber = 0;

    /// @brief 采集或读取时间戳，使用本机时间。
    QDateTime timestamp = QDateTime::currentDateTime();

    /// @brief 离线图像文件来源路径；真实相机帧可为空。
    QString sourcePath;

    /// @brief 图像数据，当前使用 Qt `QImage`，像素单位由图像自身描述。
    QImage image;

    /// @brief 兼容旧骨架命名的显示图像；与 `image` 保持同一内容。
    QImage previewImage;

    /// @brief 诊断说明，例如 offline/mock 来源。
    QString description;
};

} // namespace i2r_rlc::camera

#endif // I2R_RLC_CAMERA_CAMERA_FRAME_H
