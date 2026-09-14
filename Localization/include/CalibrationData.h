/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      CalibrationData.h
 * @brief     定位模块传感器标定数据结构。
 * @details   本文件定义图像点投影和局部定位共享的传感器标定值对象。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_CALIBRATION_DATA_H
#define I2R_RLC_LOCALIZATION_CALIBRATION_DATA_H

#include <QString>

namespace i2r_rlc::localization {

/**
 * @brief SENSORPARA.txt 对应的传感器标定数据。
 *
 * 该结构是系统标定、图像点到空间点转换、局部定位、视觉仿真和纠偏计算的
 * 共享输入模型。字段名和注释显式保留单位，避免 m、deg、pixel 混用。
 */
struct SensorCalibrationData {
    /// @brief 数据来源文件路径。
    QString sourceFile;

    /// @brief `Calib_Method`，标定方法标识，取值含义待确认。
    QString calibMethod;

    /// @brief `f/m`，焦距，单位 m。
    double focalLengthM = 0.0;

    /// @brief `Kappa`，畸变或标定系数，单位/语义待确认。
    double kappa = 0.0;

    /// @brief `sx/m`，X 方向像素尺度，单位 m。
    double sxM = 0.0;

    /// @brief `sy/m`，Y 方向像素尺度，单位 m。
    double syM = 0.0;

    /// @brief `cx`，主点 X 坐标，单位 pixel。
    double cxPixel = 0.0;

    /// @brief `cy`，主点 Y 坐标，单位 pixel。
    double cyPixel = 0.0;

    /// @brief `a`，激光平面或标定系数，坐标系待确认。
    double laserPlaneA = 0.0;

    /// @brief `b`，激光平面或标定系数，坐标系待确认。
    double laserPlaneB = 0.0;

    /// @brief `c`，激光平面或标定系数，坐标系待确认。
    double laserPlaneC = 0.0;

    /// @brief `pose_CinT[0]/m`，相机在工具坐标系下的 X 位置，单位 m。
    double poseCinTXM = 0.0;

    /// @brief `pose_CinT[1]/m`，相机在工具坐标系下的 Y 位置，单位 m。
    double poseCinTYM = 0.0;

    /// @brief `pose_CinT[2]/m`，相机在工具坐标系下的 Z 位置，单位 m。
    double poseCinTZM = 0.0;

    /// @brief `pose_CinT[3]/deg`，相机在工具坐标系下的第 1 个旋转角，单位 deg，轴序待确认。
    double poseCinTRxDeg = 0.0;

    /// @brief `pose_CinT[4]/deg`，相机在工具坐标系下的第 2 个旋转角，单位 deg，轴序待确认。
    double poseCinTRyDeg = 0.0;

    /// @brief `pose_CinT[5]/deg`，相机在工具坐标系下的第 3 个旋转角，单位 deg，轴序待确认。
    double poseCinTRzDeg = 0.0;

    /// @brief `pose_CinT[6]`，标定文件保留字段，语义待确认。
    double poseCinTReserved = 0.0;
};

/**
 * @brief 标定数据兼容别名。
 */
using CalibrationData = SensorCalibrationData;

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_CALIBRATION_DATA_H
