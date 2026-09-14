/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationPointProjector.cpp
 * @brief     图像点到空间点云投影实现。
 * @details   本文件实现标定参数、工具位姿和图像点到 mm 点云的转换。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "LocalizationPointProjector.h"

namespace i2r_rlc::localization {

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kMetersToMillimeters = 1000.0;

Eigen::Matrix4d toMatrix(const RigidTransform& transform)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) = transform.rowMajor[row * 4 + column];
        }
    }
    return matrix;
}

Eigen::Matrix4d poseToMatrix(
    const double x_mm,
    const double y_mm,
    const double z_mm,
    const double rx_deg,
    const double ry_deg,
    const double rz_deg)
{
    const Eigen::AngleAxisd rx(rx_deg * kDegreesToRadians, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(ry_deg * kDegreesToRadians, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(rz_deg * kDegreesToRadians, Eigen::Vector3d::UnitZ());

    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
    matrix(0, 3) = x_mm;
    matrix(1, 3) = y_mm;
    matrix(2, 3) = z_mm;
    return matrix;
}

Eigen::Matrix4d poseToMatrix(const Pose6D& pose)
{
    return poseToMatrix(
        pose.x_mm,
        pose.y_mm,
        pose.z_mm,
        pose.rx_deg,
        pose.ry_deg,
        pose.rz_deg);
}

Eigen::Matrix4d cameraInToolMatrix(const SensorCalibrationData& calibration)
{
    return poseToMatrix(
        calibration.poseCinTXM * kMetersToMillimeters,
        calibration.poseCinTYM * kMetersToMillimeters,
        calibration.poseCinTZM * kMetersToMillimeters,
        calibration.poseCinTRxDeg,
        calibration.poseCinTRyDeg,
        calibration.poseCinTRzDeg);
}

} // namespace

std::vector<Point3D_mm> LocalizationPointProjector::projectImagePoints(
    const ImageProjectionRequest& request)
{
    const auto& calibration = request.calibration;
    const Eigen::Matrix4d baseFromTool = poseToMatrix(request.toolPoseInRobotBase);
    const Eigen::Matrix4d toolFromCamera = cameraInToolMatrix(calibration);
    const Eigen::Matrix4d outputFromBase = toMatrix(request.outputFromRobotBase);
    const Eigen::Matrix4d outputFromCamera = outputFromBase * baseFromTool * toolFromCamera;

    std::vector<Point3D_mm> points;
    points.reserve(request.imagePoints_px.size());

    for (const ImagePoint_px& imagePoint : request.imagePoints_px) {
        double u_m = calibration.sxM * (imagePoint.x_px - calibration.cxPixel);
        double v_m = calibration.syM * (imagePoint.y_px - calibration.cyPixel);

        const double distortionScale =
            1.0 / (1.0 + calibration.kappa * (u_m * u_m + v_m * v_m));
        u_m *= distortionScale;
        v_m *= distortionScale;

        const double denominator =
            calibration.laserPlaneA * u_m
            + calibration.laserPlaneB * v_m
            + calibration.laserPlaneC * calibration.focalLengthM;
        if (std::abs(denominator) <= 1e-12) {
            continue;
        }

        const double scale = -1.0 / denominator;
        const Eigen::Vector4d pointCamera_mm(
            scale * u_m * kMetersToMillimeters,
            scale * v_m * kMetersToMillimeters,
            scale * calibration.focalLengthM * kMetersToMillimeters,
            1.0);
        const Eigen::Vector4d pointOutput_mm = outputFromCamera * pointCamera_mm;

        points.push_back(Point3D_mm{
            pointOutput_mm(0),
            pointOutput_mm(1),
            pointOutput_mm(2)
        });
    }

    return points;
}

} // namespace i2r_rlc::localization
