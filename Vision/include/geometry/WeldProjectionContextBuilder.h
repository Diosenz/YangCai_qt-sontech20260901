/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        WeldProjectionContextBuilder.h
 * @brief       焊缝投影上下文构造器
 * @details     声明由 SENSORPARA、RBCALIB 和机器人状态生成投影上下文的接口。
 ******************************************************************************/
#ifndef I2R_RLC_VISION_GEOMETRY_WELD_PROJECTION_CONTEXT_BUILDER_H
#define I2R_RLC_VISION_GEOMETRY_WELD_PROJECTION_CONTEXT_BUILDER_H

#include <string>

#include "geometry/PositionerKinematics.h"
#include "geometry/WeldGeometryTypes.h"
#include "RobotState.h"

namespace i2r_rlc::vision {

/**
 * @brief SENSORPARA.txt 中参与旧 ConvertImgPt2SP 的标定字段。
 */
struct WeldSensorCalibration {
    /// @brief 相机焦距，单位为 m。
    double focalLengthM = 0.0;
    /// @brief 旧 SENSORPARA 畸变系数 kappa，沿用原文件单位。
    double kappa = 0.0;
    /// @brief 像元 X 尺寸，单位为 m/pixel。
    double sxM = 0.0;
    /// @brief 像元 Y 尺寸，单位为 m/pixel。
    double syM = 0.0;
    /// @brief 主点 X 坐标，单位为 pixel。
    double cxPixel = 0.0;
    /// @brief 主点 Y 坐标，单位为 pixel。
    double cyPixel = 0.0;
    /// @brief 激光平面 A 系数，沿用 SENSORPARA 中的相机坐标系表达。
    double laserPlaneA = 0.0;
    /// @brief 激光平面 B 系数，沿用 SENSORPARA 中的相机坐标系表达。
    double laserPlaneB = 0.0;
    /// @brief 激光平面 C 系数，沿用 SENSORPARA 中的相机坐标系表达。
    double laserPlaneC = 0.0;
    /// @brief 相机相对工具坐标系 X 平移，单位为 m。
    double cameraXInToolM = 0.0;
    /// @brief 相机相对工具坐标系 Y 平移，单位为 m。
    double cameraYInToolM = 0.0;
    /// @brief 相机相对工具坐标系 Z 平移，单位为 m。
    double cameraZInToolM = 0.0;
    /// @brief 相机相对工具坐标系 RX 角，单位为 deg。
    double cameraRxInToolDeg = 0.0;
    /// @brief 相机相对工具坐标系 RY 角，单位为 deg。
    double cameraRyInToolDeg = 0.0;
    /// @brief 相机相对工具坐标系 RZ 角，单位为 deg。
    double cameraRzInToolDeg = 0.0;
    /// @brief true 表示 SENSORPARA 关键字段已经完整解析。
    bool valid = false;
};

/**
 * @brief 投影上下文构造结果。
 */
struct WeldProjectionContextBuildResult {
    /// @brief true 表示投影上下文可用于图像点到空间点转换。
    bool valid = false;
    /// @brief 构造得到的投影上下文，坐标单位统一为 mm。
    WeldProjectionContext context;
    /// @brief 构造说明或错误消息。
    std::string message;
};

/**
 * @brief 使用 SENSORPARA、实时机器人状态和 RBCALIB 构造逐帧投影上下文。
 */
class CWeldProjectionContextBuilder final {
public:
    /**
     * @brief 构造投影到当前变位机坐标系所需的完整变换链。
     *
     * 相机姿态保持旧 HALCON `gba` 的 Rx*Ry*Rz 顺序，机器人工具姿态保持
     * `abg` 的 Rz*Ry*Rx 顺序；机器人位置输入为 um，姿态和轴角输入为
     * deg*10000。
     *
     * @param sensorCalibration SENSORPARA 解析结果。
     * @param positionerCalibration RBCALIB 解析结果。
     * @param robotState 当前机器人状态。
     * @return 完整投影上下文；输入标定无效时 valid 为 false。
     */
    static WeldProjectionContextBuildResult build(
        const WeldSensorCalibration &sensorCalibration,
        const PositionerCalibration &positionerCalibration,
        const robot_comm::RobotState &robotState);
};

} // namespace i2r_rlc::vision

#endif
