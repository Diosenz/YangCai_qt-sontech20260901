/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OfflineImageCameraDevice.h
 * @brief       离线图像相机接口
 * @details     声明按目录回放图像的相机设备实现。
 ******************************************************************************/
#ifndef I2R_RLC_CAMERA_OFFLINE_IMAGE_CAMERA_DEVICE_H
#define I2R_RLC_CAMERA_OFFLINE_IMAGE_CAMERA_DEVICE_H

#include <QFileInfoList>
#include <QString>
#include <QStringList>

#include "ICameraDevice.h"

namespace i2r_rlc::camera {

/**
 * @brief 离线图像读取结果。
 *
 * 读取失败时 `ok` 为 false，`errorMessage` 给出结构化原因；调用方不应弹窗，
 * 应通过日志或状态栏显示错误。
 */
struct OfflineImageFrameResult {
    /// @brief 是否成功读取图像帧。
    bool ok = false;

    /// @brief 成功时的相机帧。
    CameraFrame frame;

    /// @brief 失败原因。
    QString errorMessage;
};

/**
 * @brief 离线图像相机实现。
 *
 * 从用户指定目录按文件名稳定读取 png、jpg、jpeg、bmp 图像。该类不访问旧工程，
 * 不接真实相机 SDK，也不依赖 Qt Widgets。
 */
class OfflineImageCameraDevice final : public ICameraDevice {
public:
    OfflineImageCameraDevice() = default;

    /**
     * @brief 设置离线图像目录。
     *
     * @param directoryPath 离线图像目录路径。
     * @param errorMessage 失败时写入结构化错误。
     */
    bool setImageDirectory(const QString& directoryPath, QString* errorMessage = nullptr);

    /// @brief 是否还有下一帧。
    bool hasNextFrame() const;

    /**
     * @brief 读取下一帧离线图像。
     * @return 图像帧读取结果；序列耗尽或解码失败时 ok 为 false。
     */
    OfflineImageFrameResult nextFrame();

    /// @brief 重置离线序列读取位置。
    void reset();

    /// @brief 当前图像目录，未配置时为空字符串。
    QString imageDirectory() const;

    /// @brief 已加载的离线图像数量，单位为张。
    int imageCount() const;

    /// @brief 启动离线相机；不打开真实设备，仅更新状态文本。
    void start() override;

    /// @brief 停止离线相机；不清空已加载的图像列表。
    void stop() override;

    /// @brief 最近一次相机状态文本。
    QString stateText() const override;

private:
    QStringList supportedNameFilters() const;

    QString m_imageDirectory;
    QFileInfoList m_imageFiles;
    int m_nextIndex = 0;
    qint64 m_nextFrameNumber = 0;
    QString m_stateText = QStringLiteral("Offline camera 尚未启动");
};

} // namespace i2r_rlc::camera

#endif // I2R_RLC_CAMERA_OFFLINE_IMAGE_CAMERA_DEVICE_H
