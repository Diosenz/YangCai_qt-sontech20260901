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
 * @file        PositionerKinematics.h
 * @brief       变位机标定解析与坐标变换
 * @details     使用 Eigen 复刻旧 ReadStationPosition 和 StationPos2RobotPos。
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
#ifndef I2R_RLC_VISION_GEOMETRY_POSITIONER_KINEMATICS_H
#define I2R_RLC_VISION_GEOMETRY_POSITIONER_KINEMATICS_H

#include <array>
#include <string>

#include <Eigen/Geometry>

namespace i2r_rlc::vision {

/**
 * @brief RBCALIB.DAT 中的变位机安装标定。
 */
struct PositionerCalibration {
    std::array<double, 6> stationPoseMmDeg{};
    Eigen::Affine3d stationBaseInRobotBase = Eigen::Affine3d::Identity();
    /// @brief 变位机末节坐标系到定位工装 TCP 的平移，单位 mm。
    ///
    /// SWv1 全局定位使用 positiontool20211124-jiyan.tool，其 TCP 为
    /// (0, 0, 231) mm。真实点云和 weldment 模型必须同时落在这个 TCP 系下。
    Eigen::Vector3d toolTcpInLastLinkMm = Eigen::Vector3d::Zero();
    bool valid = false;
    std::string message;
};

/**
 * @brief 位置单位 mm、姿态单位 deg 的六维位姿。
 */
struct PositionerPose {
    double xMm = 0.0;
    double yMm = 0.0;
    double zMm = 0.0;
    double rxDeg = 0.0;
    double ryDeg = 0.0;
    double rzDeg = 0.0;
    bool valid = false;
};

/**
 * @brief RBCALIB.DAT 解析器。
 */
class CPositionerCalibrationParser final {
public:
    /// @brief 从 RBCALIB.DAT 的 ////SRANG 记录解析变位机标定。
    static PositionerCalibration parseFile(const std::string &path);
};

/**
 * @brief 变位机动态坐标系到机器人基坐标系的位姿变换。
 */
class CPositionerKinematics final {
public:
    /**
     * @brief 将机器人基坐标系位姿转换到当前变位机动坐标系。
     */
    static PositionerPose robotPoseToStationPose(
        const PositionerPose &robotPose,
        double stationAxis1Deg,
        double stationAxis2Deg,
        const PositionerCalibration &calibration);

    /**
     * @brief 将当前变位机动坐标系位姿转换到机器人基坐标系。
     */
    static PositionerPose stationPoseToRobotPose(
        const PositionerPose &stationPose,
        double stationAxis1Deg,
        double stationAxis2Deg,
        const PositionerCalibration &calibration);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_GEOMETRY_POSITIONER_KINEMATICS_H
