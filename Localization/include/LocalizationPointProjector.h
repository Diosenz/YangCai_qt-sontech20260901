/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationPointProjector.h
 * @brief     图像点到空间点云的投影工具。
 * @details   本文件定义定位模块使用的离线图像点投影请求和工具入口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_POINT_PROJECTOR_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_POINT_PROJECTOR_H

#include <vector>

#include "CalibrationData.h"
#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 图像像素点。
 */
struct ImagePoint_px {
    double x_px = 0.0;
    double y_px = 0.0;
};

/**
 * @brief 图像点投影请求。
 *
 * `toolPoseInRobotBase` 表达线结构光工具坐标系在机器人基坐标系下的位姿。
 * `outputFromRobotBase` 可把结果继续转换到变位机 TCP 等工作坐标系，默认恒等。
 */
struct ImageProjectionRequest {
    std::vector<ImagePoint_px> imagePoints_px;
    SensorCalibrationData calibration;
    Pose6D toolPoseInRobotBase;
    RigidTransform outputFromRobotBase;
};

/**
 * @brief 图像线点到空间点云的离线投影工具。
 *
 * 该工具按显式标定参数和工具位姿计算 mm 点云，不依赖第三方视觉运行时或全局状态。
 * 当前姿态矩阵按 `Rz * Ry * Rx` 组合。
 */
class LocalizationPointProjector {
public:
    /**
     * @brief 将图像点投影为 mm 点云。
     */
    static std::vector<Point3D_mm> projectImagePoints(const ImageProjectionRequest& request);
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_POINT_PROJECTOR_H
