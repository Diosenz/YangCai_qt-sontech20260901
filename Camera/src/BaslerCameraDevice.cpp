/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        BaslerCameraDevice.cpp
 * @brief       Basler/Pylon 相机设备实现
 * @details     封装 CommCamera Basler 后端，并将 OpenCV Mat 转为 UI 安全的 QImage。
 ******************************************************************************/
#include <memory>

#include <opencv2/core/mat.hpp>
#include <QDateTime>
#include <QImage>

#include "BaslerCameraDevice.h"
#include "CameraBasler.h"

namespace i2r_rlc::camera {

namespace {

/**
 * @brief 将 CommCamera 返回的 OpenCV 图像复制为 Qt 图像。
 * @param image SDK 取到的 Mat，生命周期由相机后端持有。
 * @return 独立拥有像素数据的 QImage；不支持的格式返回空图。
 */
QImage matToImage(const cv::Mat& image)
{
    if (image.empty()) {
        return {};
    }

    if (image.type() == CV_8UC1) {
        return QImage(
            image.data,
            image.cols,
            image.rows,
            static_cast<int>(image.step),
            QImage::Format_Grayscale8).copy();
    }

    if (image.type() == CV_8UC3) {
        return QImage(
            image.data,
            image.cols,
            image.rows,
            static_cast<int>(image.step),
            QImage::Format_RGB888).rgbSwapped();
    }

    if (image.type() == CV_8UC4) {
        return QImage(
            image.data,
            image.cols,
            image.rows,
            static_cast<int>(image.step),
            QImage::Format_ARGB32).copy();
    }

    return {};
}

} // namespace

/**
 * @brief Basler 设备私有状态，隔离 Pylon/CommCamera 头和连续采集游标。
 */
struct BaslerCameraDevice::Impl {
    /// @brief CommCamera Basler 后端实例，只在 Camera 模块内部可见。
    std::unique_ptr<i2r_comm_camera::CameraBasler> camera;
    /// @brief 最近一次连接、采集或停止状态。
    QString stateText = QStringLiteral("Basler/Pylon camera 尚未启动");
    /// @brief 对外输出的递增帧号，单位为帧。
    qint64 nextFrameNumber = 0;
};

BaslerCameraDevice::BaslerCameraDevice() :
    m_impl(std::make_unique<Impl>())
{
}

BaslerCameraDevice::~BaslerCameraDevice()
{
    stop();
}

void BaslerCameraDevice::start()
{
    if (!m_impl->camera) {
        m_impl->camera = std::make_unique<i2r_comm_camera::CameraBasler>();
    }

    auto connectResult = m_impl->camera->connect();
    if (!connectResult) {
        m_impl->stateText = QStringLiteral("Basler/Pylon camera 连接失败");
        return;
    }

    auto shootResult = m_impl->camera->shootContinuousStart();
    if (!shootResult) {
        m_impl->camera->disconnect();
        m_impl->stateText = QStringLiteral("Basler/Pylon camera 连续采集启动失败");
        return;
    }

    m_impl->stateText = QStringLiteral("Basler/Pylon camera 正在采集");
}

void BaslerCameraDevice::stop()
{
    if (!m_impl || !m_impl->camera) {
        return;
    }

    m_impl->camera->disconnect();
    m_impl->camera.reset();
    m_impl->stateText = QStringLiteral("Basler/Pylon camera 已停止");
}

QString BaslerCameraDevice::stateText() const
{
    return m_impl->stateText;
}

bool BaslerCameraDevice::setExposureTimeUs(double exposureTimeUs, QString* errorMessage)
{
    if (!m_impl->camera) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Basler/Pylon camera 未连接，无法设置曝光");
        }
        return false;
    }

    const auto result = m_impl->camera->setExposureTime(exposureTimeUs);
    if (!result) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Basler/Pylon camera 设置曝光失败");
        }
        return false;
    }

    m_impl->stateText =
        QStringLiteral("Basler/Pylon camera 曝光已设置为 %1 us").arg(exposureTimeUs, 0, 'f', 0);
    return true;
}

bool BaslerCameraDevice::getExposureTimeUs(double* exposureTimeUs, QString* errorMessage) const
{
    if (exposureTimeUs == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("曝光时间输出指针为空");
        }
        return false;
    }
    if (!m_impl->camera) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Basler/Pylon camera 未连接，无法读取曝光");
        }
        return false;
    }

    const auto result = m_impl->camera->getExposureTime();
    if (!result) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Basler/Pylon camera 读取曝光失败");
        }
        return false;
    }

    *exposureTimeUs = result.value();
    return true;
}

CameraFrame BaslerCameraDevice::latestFrame() const
{
    CameraFrame frame;
    frame.cameraId = CameraId::BaslerIbvs;
    frame.timestamp = QDateTime::currentDateTime();
    frame.description = m_impl->stateText;

    if (!m_impl->camera) {
        return frame;
    }

    const auto imageResult = m_impl->camera->getImage();
    if (!imageResult) {
        return frame;
    }

    frame.frameId = m_impl->nextFrameNumber;
    frame.frameNumber = m_impl->nextFrameNumber;
    ++m_impl->nextFrameNumber;
    frame.image = matToImage(imageResult.value());
    frame.previewImage = frame.image;
    return frame;
}

} // namespace i2r_rlc::camera
