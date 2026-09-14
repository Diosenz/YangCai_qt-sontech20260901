/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      WeldmentVirtualLaserPriorGenerator.h
 * @brief     基于 weldment 模型的虚拟激光先验生成器。
 * @details   根据 .weldment 工件模型、当前机器人/变位机位姿和传感器标定，
 *            按 SWv1 的扇形激光射线算法与完整工件求最近交点，再投影得到
 *            仿真激光线二维点与仿真焊缝点二维点。该先验用于替代 RobotSim
 *            观测，使全局定位的真实激光线提取/跟踪在无外部仿真时也可运行。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 8 月 8 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_WELDMENT_VIRTUAL_LASER_PRIOR_GENERATOR_H
#define I2R_RLC_DIGITAL_TWIN_WELDMENT_VIRTUAL_LASER_PRIOR_GENERATOR_H

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "DigitalTwinVisionTypes.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief 生成仿真激光先验所需的传感器标定，字段与 SENSORPARA.txt 对齐。
 */
struct WeldmentVirtualLaserCalibration {
    /// @brief 相机焦距，单位 m。
    double focalLengthM = 0.0;
    /// @brief 径向畸变系数 kappa。
    double kappa = 0.0;
    /// @brief 像元 X 尺寸，单位 m/pixel。
    double sxM = 0.0;
    /// @brief 像元 Y 尺寸，单位 m/pixel。
    double syM = 0.0;
    /// @brief 主点 X 坐标，单位 pixel。
    double cxPixel = 0.0;
    /// @brief 主点 Y 坐标，单位 pixel。
    double cyPixel = 0.0;
    /// @brief 激光平面系数 a（相机坐标系表达）。
    double laserPlaneA = 0.0;
    /// @brief 激光平面系数 b。
    double laserPlaneB = 0.0;
    /// @brief 激光平面系数 c。
    double laserPlaneC = 0.0;
    /// @brief 相机相对工具坐标系平移 X，单位 m。
    double cameraXInToolM = 0.0;
    /// @brief 相机相对工具坐标系平移 Y，单位 m。
    double cameraYInToolM = 0.0;
    /// @brief 相机相对工具坐标系平移 Z，单位 m。
    double cameraZInToolM = 0.0;
    /// @brief 相机相对工具坐标系旋转 RX，单位 deg。
    double cameraRxInToolDeg = 0.0;
    /// @brief 相机相对工具坐标系旋转 RY，单位 deg。
    double cameraRyInToolDeg = 0.0;
    /// @brief 相机相对工具坐标系旋转 RZ，单位 deg。
    double cameraRzInToolDeg = 0.0;
    /// @brief true 表示标定字段完整可用。
    bool valid = false;
};

/**
 * @brief 生成仿真激光先验的一次请求。
 */
struct WeldmentVirtualLaserRequest {
    /// @brief .weldment 文件路径。
    std::string weldmentPath;

    /// @brief 工具（传感器）在机器人基坐标系下的位置，单位 mm。
    double toolXmm = 0.0;
    double toolYmm = 0.0;
    double toolZmm = 0.0;
    /// @brief 工具在机器人基坐标系下的姿态，单位 deg。
    double toolRxDeg = 0.0;
    double toolRyDeg = 0.0;
    double toolRzDeg = 0.0;

    /// @brief 变位机两轴角度，单位 deg。
    double stationAxis1Deg = 0.0;
    double stationAxis2Deg = 0.0;

    /// @brief 传感器标定。
    WeldmentVirtualLaserCalibration calibration;

    /// @brief RBCALIB.DAT 解析得到的变位机基坐标系在机器人基坐标系下的位姿，平移单位 mm。
    Eigen::Affine3d stationBaseInRobotBase = Eigen::Affine3d::Identity();

    /// @brief 变位机末节坐标系到定位工装 TCP 的平移，单位 mm。
    Eigen::Vector3d positionerToolTcpInLastLinkMm = Eigen::Vector3d::Zero();

    /// @brief 当前 STARTPOSSEG 对应的定位面序号，从 1 开始；0 表示仅预热/整模检查。
    int positioningSegmentIndex = 0;

    /// @brief 相机图像宽，单位 pixel；用于裁剪投影点。
    int imageWidth = 1292;
    /// @brief 相机图像高，单位 pixel。
    int imageHeight = 964;

    /// @brief 激光扇形起始角，单位 deg；与 SWv1 的 SwLSLSensor 默认值一致。
    double planeAngleStartDeg = -15.0;
    /// @brief 激光扇形终止角（不包含），单位 deg。
    double planeAngleEndDeg = 15.0;
    /// @brief 相邻激光射线角度间隔，单位 deg。
    double planeAngleStepDeg = 0.075;
    /// @brief 激光射线最大求交距离，单位 mm。
    double maxRayDistanceMm = 400.0;
};

/**
 * @brief 基于 OpenCASCADE 的 weldment 虚拟激光先验生成器。
 *
 * 本类只负责从模型和位姿计算仿真先验，不访问真实相机、机器人或 RobotSim。
 */
class WeldmentVirtualLaserPriorGenerator final {
public:
    WeldmentVirtualLaserPriorGenerator();
    ~WeldmentVirtualLaserPriorGenerator();

    WeldmentVirtualLaserPriorGenerator(const WeldmentVirtualLaserPriorGenerator&) = delete;
    WeldmentVirtualLaserPriorGenerator& operator=(const WeldmentVirtualLaserPriorGenerator&) = delete;

    /**
     * @brief 生成一帧仿真激光观测先验。
     *
     * @return 有效观测至少包含 2 个激光线二维点与 1 个焊缝二维点；
     *         模型/标定缺失或求交失败时返回无效观测。
     */
    DigitalTwinVisionLaserObservation generate(
        const WeldmentVirtualLaserRequest& request) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_WELDMENT_VIRTUAL_LASER_PRIOR_GENERATOR_H
