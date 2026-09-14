/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        AlliedCameraDevice.h
 * @brief       Allied/Vimba X 相机设备接口
 * @details     声明 Allied 相机适配器，隐藏厂商 SDK 和 OpenCV 细节。
 ******************************************************************************/
#ifndef I2R_RLC_CAMERA_ALLIED_CAMERA_DEVICE_H
#define I2R_RLC_CAMERA_ALLIED_CAMERA_DEVICE_H

#include <memory>

#include <QString>

#include "CameraFrame.h"
#include "ICameraDevice.h"

namespace i2r_rlc::camera {

/**
 * @brief Allied/Vimba X 相机设备适配器。
 *
 * 该类是当前 Camera 模块对 CommCamera Allied 后端的薄封装。公共头不暴露
 * Vimba X、OpenCV 或 CommCamera 头文件，避免 UI 和普通 Camera 调用方依赖厂商 SDK。
 */
class AlliedCameraDevice final : public ICameraDevice {
public:
    AlliedCameraDevice();
    ~AlliedCameraDevice() override;

    AlliedCameraDevice(const AlliedCameraDevice&) = delete;
    AlliedCameraDevice(AlliedCameraDevice&&) = delete;
    AlliedCameraDevice& operator=(const AlliedCameraDevice&) = delete;
    AlliedCameraDevice& operator=(AlliedCameraDevice&&) = delete;

    void start() override;
    void stop() override;

    /// @brief 最近一次适配器状态文本。
    QString stateText() const override;

    /**
     * @brief 设置 Allied/Vimba X 相机曝光时间。
     *
     * @param exposureTimeUs 曝光时间，单位 us。
     * @param errorMessage 失败原因输出；调用方可为空。
     * @return 厂商 SDK 接受曝光参数时返回 true。
     */
    bool setExposureTimeUs(double exposureTimeUs, QString* errorMessage = nullptr) override;

    /**
     * @brief 读取 Allied/Vimba X 相机当前曝光时间。
     *
     * @param exposureTimeUs 曝光时间输出，单位 us。
     * @param errorMessage 失败原因输出；调用方可为空。
     * @return 成功从厂商 SDK 读取曝光时返回 true。
     */
    bool getExposureTimeUs(double* exposureTimeUs, QString* errorMessage = nullptr) const override;

    /**
     * @brief 获取连续采集缓存中的最近一帧。
     *
     * 未启动或暂时没有图像时返回空图像帧，调用方应通过 `image.isNull()` 判断。
     * @return 最近一帧相机图像和帧号。
     */
    CameraFrame latestFrame() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace i2r_rlc::camera

#endif // I2R_RLC_CAMERA_ALLIED_CAMERA_DEVICE_H
