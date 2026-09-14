/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        ICameraDevice.h
 * @brief       相机设备抽象接口
 * @details     约束相机启动、停止、取帧和曝光读写的模块边界。
 ******************************************************************************/
#ifndef I2R_RLC_CAMERA_I_CAMERA_DEVICE_H
#define I2R_RLC_CAMERA_I_CAMERA_DEVICE_H

#include <QString>

#include "CameraFrame.h"

namespace i2r_rlc::camera {

/**
 * @brief 相机设备接口。
 *
 * UI 不直接依赖本接口。后续真实后端应放在 camera 模块中，例如
 * `VimbaTrackingCameraBackend` 和 `BaslerIbvsCameraBackend`。
 */
class ICameraDevice {
public:
    virtual ~ICameraDevice() = default;

    /**
     * @brief 启动采集。
     * @note 实现应避免阻塞 UI 线程；真实相机可在内部启动 SDK 取流。
     */
    virtual void start() = 0;

    /**
     * @brief 停止采集。
     * @note stop 应可重复调用，并负责释放实现内部的取流状态。
     */
    virtual void stop() = 0;

    /**
     * @brief 获取流式相机缓存中的最新帧。
     *
     * 离线设备或尚未准备好的设备可返回空帧。真实相机实现不得在此函数中
     * 长时间阻塞；厂商 SDK 的取流线程应先把图像写入内部缓存。
     *
     * @return 最近一帧图像；没有可用图像时返回默认 CameraFrame。
     */
    virtual CameraFrame latestFrame() const
    {
        return {};
    }

    /// @brief 返回最近一次设备状态，供 Camera worker 和 UI 诊断使用。
    virtual QString stateText() const
    {
        return {};
    }

    /**
     * @brief 设置相机曝光时间。
     *
     * @param exposureTimeUs 曝光时间，单位 us。
     * @param errorMessage 失败原因输出；调用方可为空。
     * @return 成功写入真实相机时返回 true。默认实现表示当前相机后端不支持曝光设置。
     */
    virtual bool setExposureTimeUs(double exposureTimeUs, QString* errorMessage = nullptr)
    {
        Q_UNUSED(exposureTimeUs);
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("当前相机后端不支持曝光设置");
        }
        return false;
    }

    /**
     * @brief 读取当前相机曝光时间。
     *
     * @param exposureTimeUs 曝光时间输出，单位 us。
     * @param errorMessage 失败原因输出；调用方可为空。
     * @return 成功从真实相机读取曝光时返回 true。默认实现表示当前相机后端不支持读取曝光。
     */
    virtual bool getExposureTimeUs(double* exposureTimeUs, QString* errorMessage = nullptr) const
    {
        Q_UNUSED(exposureTimeUs);
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("当前相机后端不支持读取曝光");
        }
        return false;
    }
};

} // namespace i2r_rlc::camera

#endif // I2R_RLC_CAMERA_I_CAMERA_DEVICE_H
