/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        PositionerKinematics.cpp
 * @brief       变位机标定解析与坐标变换实现
 * @details     保持旧工程旋转顺序、平移方向和欧拉角范围修正。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <cmath>
#include <fstream>
#include <sstream>

#include "geometry/PositionerKinematics.h"

namespace i2r_rlc::vision {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSwv1PositionerToolTcpZMm = 231.0;

double toRad(double degree)
{
    return degree * kPi / 180.0;
}

double toDeg(double radian)
{
    return radian * 180.0 / kPi;
}

} // namespace

PositionerCalibration CPositionerCalibrationParser::parseFile(const std::string &path)
{
    PositionerCalibration result;
    std::ifstream input(path);
    if (!input) {
        result.message = "RBCALIB file cannot be opened";
        return result;
    }

    std::string line;
    bool found = false;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string marker;
        std::string values;
        stream >> marker >> values;
        if (marker != "////SRANG") {
            continue;
        }

        std::istringstream valueStream(values);
        std::string value;
        std::size_t index = 0;
        while (std::getline(valueStream, value, ',') && index < result.stationPoseMmDeg.size()) {
            try {
                result.stationPoseMmDeg[index++] = std::stod(value);
            } catch (...) {
                result.message = "RBCALIB SRANG contains invalid number";
                return result;
            }
        }
        found = index == result.stationPoseMmDeg.size();
        break;
    }

    if (!found) {
        result.message = "RBCALIB SRANG record is missing";
        return result;
    }

    const auto &pose = result.stationPoseMmDeg;
    const Eigen::Translation3d translation(-pose[0], -pose[1], -pose[2]);
    const Eigen::AngleAxisd rotationX(toRad(-pose[3]), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd rotationY(toRad(-pose[4]), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rotationZ(toRad(-pose[5]), Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d rotation =
        (rotationX * rotationY * rotationZ).toRotationMatrix();

    // 保持旧 ReadStationPosition 的 Rot * Translation 组合顺序。
    result.stationBaseInRobotBase = Eigen::Affine3d(rotation) * translation;
    // SWv1 initialVirtualInformation() 固定安装
    // positiontool20211124-jiyan.tool；其 TCP origin 为 0 0 0.231 m。
    // RBCALIB 只描述变位机基座安装位姿，因此这里补齐旧虚拟机器人末端链。
    result.toolTcpInLastLinkMm = Eigen::Vector3d(
        0.0,
        0.0,
        kSwv1PositionerToolTcpZMm);
    result.valid = result.stationBaseInRobotBase.matrix().allFinite();
    result.message = result.valid ? "RBCALIB loaded" : "RBCALIB transform is invalid";
    return result;
}

PositionerPose CPositionerKinematics::stationPoseToRobotPose(
    const PositionerPose &stationPose,
    double stationAxis1Deg,
    double stationAxis2Deg,
    const PositionerCalibration &calibration)
{
    PositionerPose result;
    if (!stationPose.valid || !calibration.valid) {
        return result;
    }

    const Eigen::Affine3d robotBaseInStationBase =
        calibration.stationBaseInRobotBase.inverse();
    const Eigen::Matrix3d stationRotation =
        (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(toRad(stationAxis1Deg), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(toRad(stationAxis2Deg), Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    const Eigen::Affine3d stationBaseInCurrent =
        Eigen::Translation3d(0.0, 0.0, 0.0) * stationRotation;

    const Eigen::Matrix3d poseRotation =
        (Eigen::AngleAxisd(toRad(stationPose.rzDeg), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(toRad(stationPose.ryDeg), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(toRad(stationPose.rxDeg), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Affine3d poseInCurrent =
        Eigen::Translation3d(stationPose.xMm, stationPose.yMm, stationPose.zMm)
        * poseRotation;
    const Eigen::Affine3d poseInRobot =
        robotBaseInStationBase * stationBaseInCurrent * poseInCurrent;

    Eigen::Vector3d eulerZyx = poseInRobot.rotation().eulerAngles(2, 1, 0);
    if (std::abs(eulerZyx[1]) > kPi / 2.0) {
        eulerZyx[1] = eulerZyx[1] > kPi / 2.0
            ? kPi - eulerZyx[1]
            : -(eulerZyx[1] + kPi);
        eulerZyx[0] += eulerZyx[0] < 0.0 ? kPi : -kPi;
        eulerZyx[2] += eulerZyx[2] < 0.0 ? kPi : -kPi;
    }

    result.xMm = poseInRobot.translation().x();
    result.yMm = poseInRobot.translation().y();
    result.zMm = poseInRobot.translation().z();
    result.rxDeg = toDeg(eulerZyx[2]);
    result.ryDeg = toDeg(eulerZyx[1]);
    result.rzDeg = toDeg(eulerZyx[0]);
    result.valid = poseInRobot.matrix().allFinite();
    return result;
}

PositionerPose CPositionerKinematics::robotPoseToStationPose(
    const PositionerPose &robotPose,
    double stationAxis1Deg,
    double stationAxis2Deg,
    const PositionerCalibration &calibration)
{
    PositionerPose result;
    if (!robotPose.valid || !calibration.valid) {
        return result;
    }

    const Eigen::Matrix3d currentInStationBaseRotation =
        (Eigen::AngleAxisd(toRad(-stationAxis2Deg), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(toRad(-stationAxis1Deg), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Affine3d currentInStationBase(currentInStationBaseRotation);
    const Eigen::Matrix3d robotPoseRotation =
        (Eigen::AngleAxisd(toRad(robotPose.rzDeg), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(toRad(robotPose.ryDeg), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(toRad(robotPose.rxDeg), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Affine3d poseInRobot =
        Eigen::Translation3d(robotPose.xMm, robotPose.yMm, robotPose.zMm)
        * robotPoseRotation;
    const Eigen::Affine3d poseInCurrent =
        currentInStationBase
        * calibration.stationBaseInRobotBase
        * poseInRobot;

    const Eigen::Vector3d eulerZyx = poseInCurrent.rotation().eulerAngles(2, 1, 0);
    result.xMm = poseInCurrent.translation().x();
    result.yMm = poseInCurrent.translation().y();
    result.zMm = poseInCurrent.translation().z();
    result.rxDeg = toDeg(eulerZyx[2]);
    result.ryDeg = toDeg(eulerZyx[1]);
    result.rzDeg = toDeg(eulerZyx[0]);
    result.valid = poseInCurrent.matrix().allFinite();
    return result;
}

} // namespace i2r_rlc::vision
