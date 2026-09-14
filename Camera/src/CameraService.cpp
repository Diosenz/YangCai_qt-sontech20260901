/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CameraService.cpp
 * @brief       相机服务门面实现
 * @details     维护当前相机后端指针和离线回放状态文本。
 ******************************************************************************/
#include "CameraService.h"

namespace i2r_rlc::camera {

void CameraService::setDevice(ICameraDevice* device)
{
    m_device = device;
    m_offlineDevice = dynamic_cast<OfflineImageCameraDevice*>(device);
}

bool CameraService::loadOfflineImageDirectory(const QString& directoryPath, QString* errorMessage)
{
    if (!m_offlineDevice) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("未配置离线图像相机后端");
        }
        return false;
    }

    return m_offlineDevice->setImageDirectory(directoryPath, errorMessage);
}

OfflineImageFrameResult CameraService::nextOfflineFrame()
{
    if (!m_offlineDevice) {
        return {false, {}, QStringLiteral("未配置离线图像相机后端")};
    }

    const auto result = m_offlineDevice->nextFrame();
    return result;
}

void CameraService::resetOfflineSequence()
{
    if (!m_offlineDevice) {
        m_stateText = QStringLiteral("未配置离线图像相机后端");
        return;
    }

    m_offlineDevice->reset();
    m_stateText = m_offlineDevice->stateText();
}

void CameraService::start()
{
    if (!m_device) {
        m_stateText = QStringLiteral("未配置相机设备");
        return;
    }

    m_device->start();
    if (m_offlineDevice) {
        m_stateText = m_offlineDevice->stateText();
    }
}

void CameraService::stop()
{
    if (!m_device) {
        return;
    }

    m_device->stop();
    if (m_offlineDevice) {
        m_stateText = m_offlineDevice->stateText();
    }
}

QString CameraService::stateText() const
{
    return m_stateText;
}

} // namespace i2r_rlc::camera
