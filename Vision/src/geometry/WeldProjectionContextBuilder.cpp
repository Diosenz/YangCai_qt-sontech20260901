/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        WeldProjectionContextBuilder.cpp
 * @brief       焊缝投影上下文构造实现
 * @details     按旧工程姿态顺序构造相机、工具和变位机坐标变换链。
 ******************************************************************************/
#include <cmath>

#include <Eigen/Geometry>

#include "geometry/WeldProjectionContextBuilder.h"

namespace i2r_rlc::vision {
namespace {

/// @brief 角度换算使用的圆周率常量。
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 将角度转换为弧度。
 * @param valueDeg 角度，单位为 deg。
 * @return 弧度值。
 */
double toRad(double valueDeg)
{
    return valueDeg * kPi / 180.0;
}

/**
 * @brief 将 Eigen 仿射变换转为 Vision 模块统一刚体变换。
 * @param transform Eigen 4x4 仿射变换，平移单位为 mm。
 * @return WeldRigidTransform，矩阵有限时 valid 为 true。
 */
WeldRigidTransform makeRigidTransform(const Eigen::Affine3d &transform)
{
    WeldRigidTransform result;
    result.rotation = transform.rotation();
    result.translationMm = transform.translation();
    result.valid = transform.matrix().allFinite();
    return result;
}

/**
 * @brief 检查 SENSORPARA 关键字段是否有限且尺度可用。
 * @param value SENSORPARA 解析结果。
 * @return 标定有效、焦距和像元尺寸非零时返回 true。
 */
bool finiteSensorCalibration(const WeldSensorCalibration &value)
{
    const double fields[] = {
        value.focalLengthM, value.kappa, value.sxM, value.syM,
        value.cxPixel, value.cyPixel,
        value.laserPlaneA, value.laserPlaneB, value.laserPlaneC,
        value.cameraXInToolM, value.cameraYInToolM, value.cameraZInToolM,
        value.cameraRxInToolDeg, value.cameraRyInToolDeg, value.cameraRzInToolDeg
    };
    for (double field : fields) {
        if (!std::isfinite(field)) {
            return false;
        }
    }
    return value.valid && std::abs(value.focalLengthM) > 1e-12
        && std::abs(value.sxM) > 1e-12 && std::abs(value.syM) > 1e-12;
}

} // namespace

WeldProjectionContextBuildResult CWeldProjectionContextBuilder::build(
    const WeldSensorCalibration &sensor,
    const PositionerCalibration &positioner,
    const robot_comm::RobotState &robotState)
{
    WeldProjectionContextBuildResult result;
    if (!finiteSensorCalibration(sensor)) {
        result.message = "SENSORPARA calibration is invalid";
        return result;
    }
    if (!positioner.valid) {
        result.message = "RBCALIB calibration is invalid";
        return result;
    }

    WeldProjectionContext context;
    context.calibration.focalLengthM = sensor.focalLengthM;
    context.calibration.kappa = sensor.kappa;
    context.calibration.sxM = sensor.sxM;
    context.calibration.syM = sensor.syM;
    context.calibration.intrinsics.fxPixel = sensor.focalLengthM / sensor.sxM;
    context.calibration.intrinsics.fyPixel = sensor.focalLengthM / sensor.syM;
    context.calibration.intrinsics.cxPixel = sensor.cxPixel;
    context.calibration.intrinsics.cyPixel = sensor.cyPixel;
    context.calibration.laserPlaneA = sensor.laserPlaneA;
    context.calibration.laserPlaneB = sensor.laserPlaneB;
    context.calibration.laserPlaneC = sensor.laserPlaneC;

    const Eigen::Matrix3d cameraRotation =
        (Eigen::AngleAxisd(toRad(sensor.cameraRxInToolDeg), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(toRad(sensor.cameraRyInToolDeg), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(toRad(sensor.cameraRzInToolDeg), Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    const Eigen::Affine3d cameraInTool =
        Eigen::Translation3d(
            sensor.cameraXInToolM * 1000.0,
            sensor.cameraYInToolM * 1000.0,
            sensor.cameraZInToolM * 1000.0)
        * cameraRotation;
    context.calibration.cameraInTool = makeRigidTransform(cameraInTool);
    context.calibration.valid = context.calibration.cameraInTool.valid;

    const Eigen::Matrix3d toolRotation =
        (Eigen::AngleAxisd(
             toRad(static_cast<double>(robotState.curRz_r) / 10000.0),
             Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(
             toRad(static_cast<double>(robotState.curRy_r) / 10000.0),
             Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(
             toRad(static_cast<double>(robotState.curRx_r) / 10000.0),
             Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Affine3d toolInRobot =
        Eigen::Translation3d(
            static_cast<double>(robotState.curx_r) / 1000.0,
            static_cast<double>(robotState.cury_r) / 1000.0,
            static_cast<double>(robotState.curz_r) / 1000.0)
        * toolRotation;
    context.toolInRobotBase = makeRigidTransform(toolInRobot);

    const double s1Deg = static_cast<double>(robotState.curS1) / 10000.0;
    const double s2Deg = static_cast<double>(robotState.curS2) / 10000.0;
    const Eigen::Matrix3d currentInStationRotation =
        (Eigen::AngleAxisd(toRad(-s2Deg), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(toRad(-s1Deg), Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    const Eigen::Affine3d robotBaseToCurrentStation =
        Eigen::Translation3d(
            -positioner.toolTcpInLastLinkMm.x(),
            -positioner.toolTcpInLastLinkMm.y(),
            -positioner.toolTcpInLastLinkMm.z())
        * Eigen::Affine3d(currentInStationRotation)
        * positioner.stationBaseInRobotBase;
    context.robotBaseToPositionerTcp = makeRigidTransform(robotBaseToCurrentStation);

    context.valid = context.calibration.valid
        && context.toolInRobotBase.valid
        && context.robotBaseToPositionerTcp.valid;
    result.valid = context.valid;
    result.context = context;
    result.message = context.valid
        ? "projection context built"
        : "projection transform contains invalid values";
    return result;
}

} // namespace i2r_rlc::vision
