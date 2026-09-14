/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OfflineImageCameraDevice.cpp
 * @brief       离线图像相机实现
 * @details     按文件名顺序读取图像目录，用于无真实相机时的回放和测试。
 ******************************************************************************/
#include <QDir>
#include <QImage>

#include "OfflineImageCameraDevice.h"

namespace i2r_rlc::camera {

bool OfflineImageCameraDevice::setImageDirectory(const QString& directoryPath, QString* errorMessage)
{
    QDir directory(directoryPath);
    if (!directory.exists()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("离线图像目录不存在：%1").arg(directoryPath);
        }
        return false;
    }

    m_imageFiles = directory.entryInfoList(
        supportedNameFilters(),
        QDir::Files | QDir::Readable,
        QDir::Name | QDir::IgnoreCase);

    if (m_imageFiles.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("离线图像目录中没有支持的图片：%1").arg(directoryPath);
        }
        return false;
    }

    m_imageDirectory = directory.absolutePath();
    reset();
    m_stateText = QStringLiteral("已加载离线图像目录：%1，共 %2 张").arg(m_imageDirectory).arg(m_imageFiles.size());
    return true;
}

bool OfflineImageCameraDevice::hasNextFrame() const
{
    return m_nextIndex >= 0 && m_nextIndex < m_imageFiles.size();
}

OfflineImageFrameResult OfflineImageCameraDevice::nextFrame()
{
    if (m_imageFiles.isEmpty()) {
        return {false, {}, QStringLiteral("尚未加载离线图像目录")};
    }

    if (!hasNextFrame()) {
        return {false, {}, QStringLiteral("离线图像序列已到末尾")};
    }

    const QFileInfo fileInfo = m_imageFiles.at(m_nextIndex++);
    QImage image(fileInfo.absoluteFilePath());
    if (image.isNull()) {
        return {false, {}, QStringLiteral("无法读取离线图像：%1").arg(fileInfo.absoluteFilePath())};
    }

    CameraFrame frame;
    frame.cameraId = CameraId::OfflineReplay;
    frame.frameId = m_nextFrameNumber;
    frame.frameNumber = m_nextFrameNumber++;
    frame.timestamp = QDateTime::currentDateTime();
    frame.sourcePath = fileInfo.absoluteFilePath();
    frame.image = image;
    frame.previewImage = image;
    frame.description = QStringLiteral("离线图像帧：%1").arg(fileInfo.fileName());
    return {true, frame, {}};
}

void OfflineImageCameraDevice::reset()
{
    m_nextIndex = 0;
    m_nextFrameNumber = 0;
    m_stateText = QStringLiteral("离线图像序列已重置");
}

QString OfflineImageCameraDevice::imageDirectory() const
{
    return m_imageDirectory;
}

int OfflineImageCameraDevice::imageCount() const
{
    return m_imageFiles.size();
}

void OfflineImageCameraDevice::start()
{
    m_stateText = QStringLiteral("Offline camera 已启动，等待读取离线帧");
}

void OfflineImageCameraDevice::stop()
{
    m_stateText = QStringLiteral("Offline camera 已停止");
}

QString OfflineImageCameraDevice::stateText() const
{
    return m_stateText;
}

QStringList OfflineImageCameraDevice::supportedNameFilters() const
{
    return {
        QStringLiteral("*.png"),
        QStringLiteral("*.jpg"),
        QStringLiteral("*.jpeg"),
        QStringLiteral("*.bmp"),
        QStringLiteral("*.PNG"),
        QStringLiteral("*.JPG"),
        QStringLiteral("*.JPEG"),
        QStringLiteral("*.BMP")
    };
}

} // namespace i2r_rlc::camera
