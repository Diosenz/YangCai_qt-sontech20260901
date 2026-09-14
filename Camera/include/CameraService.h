/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CameraService.h
 * @brief       相机服务门面
 * @details     隔离 UI 控制层与离线、真实相机后端的生命周期边界。
 ******************************************************************************/
#ifndef I2R_RLC_CAMERA_CAMERA_SERVICE_H
#define I2R_RLC_CAMERA_CAMERA_SERVICE_H

#include <QString>

#include "ICameraDevice.h"
#include "OfflineImageCameraDevice.h"

namespace i2r_rlc::camera {

/**
 * @brief 相机服务门面。
 *
 * 负责隔离 UI/controller 与具体相机后端。当前同步封装离线图像后端；
 * 后续可在不改变 UI 的情况下迁移到 `CameraWorker` 和 `QThread`。
 */
class CameraService final {
public:
    /**
     * @brief 设置当前相机设备；服务不拥有设备生命周期。
     * @param device 相机设备指针，调用方负责保证其生命周期覆盖服务使用期。
     */
    void setDevice(ICameraDevice* device);

    /**
     * @brief 加载离线图像目录。
     * @param directoryPath 图像目录路径。
     * @param errorMessage 可选错误说明输出。
     * @return 目录可读且已建立离线图像序列时返回 true。
     */
    bool loadOfflineImageDirectory(const QString& directoryPath, QString* errorMessage = nullptr);

    /**
     * @brief 读取下一帧离线图像。
     * @return 读取结果；无离线设备、文件结束或解码失败时 ok 为 false。
     */
    OfflineImageFrameResult nextOfflineFrame();

    /**
     * @brief 重置离线图像序列。
     */
    void resetOfflineSequence();

    /**
     * @brief 启动当前相机设备。
     * @note 当前实现为同步调用；真实设备的长时取流应放在 worker 线程中。
     */
    void start();

    /**
     * @brief 停止当前相机设备。
     * @note 不释放由调用方拥有的设备对象。
     */
    void stop();

    /// @brief 最近一次相机状态文本，供 UI 状态栏和日志展示。
    QString stateText() const;

private:
    ICameraDevice* m_device = nullptr;
    OfflineImageCameraDevice* m_offlineDevice = nullptr;
    QString m_stateText;
};

} // namespace i2r_rlc::camera

#endif // I2R_RLC_CAMERA_CAMERA_SERVICE_H
