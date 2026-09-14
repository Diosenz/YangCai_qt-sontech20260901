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
 * @file        WeldGeometryTypes.h
 * @brief       焊接几何数据类型
 * @details     提供焊接视觉中的二维几何、投影和线段计算能力，输入输出统一使用像素或空间坐标结构。
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
#ifndef I2R_RLC_VISION_GEOMETRY_WELD_GEOMETRY_TYPES_H
#define I2R_RLC_VISION_GEOMETRY_WELD_GEOMETRY_TYPES_H
#include <string>
#include <vector>

#include <Eigen/Core>

namespace i2r_rlc::vision {

/**
 * @brief 图像像素点，单位 pixel。
 */
struct WeldImagePoint {
    /// @brief 横向像素坐标，单位 pixel。
    double xPixel = 0.0;

    /// @brief 纵向像素坐标，单位 pixel。
    double yPixel = 0.0;

    /// @brief true 表示该像素点可参与几何投影。
    bool valid = false;
};

/**
 * @brief 焊接观测空间点，单位 mm。
 */
struct WeldSpacePoint {
    /// @brief X 坐标，单位 mm。
    double xMm = 0.0;

    /// @brief Y 坐标，单位 mm。
    double yMm = 0.0;

    /// @brief Z 坐标，单位 mm。
    double zMm = 0.0;

    /// @brief true 表示该空间点有效。
    bool valid = false;
};

/**
 * @brief 焊接观测点云的轻量容器。
 *
 * 当前公共接口不暴露 PCL 类型；若下游内部需要 PCL 滤波或 ICP，可在
 * `vision/geometry` 内部转换。
 */
struct WeldPointCloud {
    /// @brief 点云中的 3D 点，单位 mm。
    std::vector<WeldSpacePoint> points;

    /// @brief 坐标系名称，例如 camera、sensor 或 weld_local。
    std::string frameId;

    /// @brief true 表示点云至少包含一个有效点。
    bool valid = false;
};

/**
 * @brief 针孔相机内参，单位 pixel。
 */
struct WeldCameraIntrinsics {
    /// @brief X 方向焦距，单位 pixel。
    double fxPixel = 0.0;

    /// @brief Y 方向焦距，单位 pixel。
    double fyPixel = 0.0;

    /// @brief 主点 X 坐标，单位 pixel。
    double cxPixel = 0.0;

    /// @brief 主点 Y 坐标，单位 pixel。
    double cyPixel = 0.0;
};

/**
 * @brief 刚体变换，平移单位 mm。
 */
struct WeldRigidTransform {
    /// @brief 旋转矩阵。
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    /// @brief 平移向量，单位 mm。
    Eigen::Vector3d translationMm = Eigen::Vector3d::Zero();

    /// @brief true 表示该变换可用。
    bool valid = true;
};

/**
 * @brief 图像点投影的目标坐标系。
 */
enum class WeldProjectionTargetFrame {
    Camera,
    Tool,
    RobotBase,
    PositionerTcp
};

/**
 * @brief 观测平面模型，方程为 ax + by + cz + d = 0，坐标单位 mm。
 */
struct WeldPlaneModel {
    /// @brief 平面法向 X 分量。
    double normalX = 0.0;

    /// @brief 平面法向 Y 分量。
    double normalY = 0.0;

    /// @brief 平面法向 Z 分量。
    double normalZ = 1.0;

    /// @brief 平面常数项，单位 mm。
    double offsetMm = 0.0;
};

/**
 * @brief 图像点到空间点重建所需的标定参数。
 *
 * 该结构对应旧 `SENSORPARA_4` 中真正参与投影计算的字段，统一使用显式单位，
 * 便于在 `vision` 内部用 Eigen 完成相机射线、激光平面和刚体链式变换。
 */
struct WeldProjectionCalibration {
    /// @brief 主点和像素焦距参数。
    WeldCameraIntrinsics intrinsics;

    /// @brief legacy `f`，焦距，单位 m。
    double focalLengthM = 0.0;

    /// @brief legacy `Kappa`，径向修正系数。
    double kappa = 0.0;

    /// @brief legacy `sx`，像素 x 方向尺度，单位 m。
    double sxM = 0.0;

    /// @brief legacy `sy`，像素 y 方向尺度，单位 m。
    double syM = 0.0;

    /// @brief 激光平面系数 `a`。
    double laserPlaneA = 0.0;

    /// @brief 激光平面系数 `b`。
    double laserPlaneB = 0.0;

    /// @brief 激光平面系数 `c`。
    double laserPlaneC = 0.0;

    /// @brief 相机坐标系到工具坐标系的刚体变换，平移单位 mm。
    WeldRigidTransform cameraInTool;

    /// @brief true 表示该标定可用于投影。
    bool valid = false;
};

/**
 * @brief 一次投影请求所需的全部上下文。
 */
struct WeldProjectionContext {
    /// @brief 投影标定参数。
    WeldProjectionCalibration calibration;

    /// @brief 工具坐标系到机器人基坐标系的刚体变换，平移单位 mm。
    WeldRigidTransform toolInRobotBase;

    /// @brief 机器人基坐标系到变位机/TCP 坐标系的刚体变换，平移单位 mm。
    WeldRigidTransform robotBaseToPositionerTcp;

    /// @brief true 表示该上下文整体可用。
    bool valid = false;
};

/**
 * @brief 图像点到空间点重建结果。
 */
struct WeldProjectionResult {
    /// @brief true 表示投影成功。
    bool valid = false;

    /// @brief 投影得到的空间点，单位 mm。
    WeldSpacePoint spacePoint;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 焊接几何匹配结果，供下游 ICP / 轨迹匹配复用。
 */
struct WeldGeometryMatchResult {
    /// @brief true 表示匹配有效。
    bool valid = false;

    /// @brief 匹配得到的刚体变换。
    WeldRigidTransform transform;

    /// @brief 平均残差，单位 mm。
    double meanErrorMm = 0.0;

    /// @brief 最大残差，单位 mm。
    double maxErrorMm = 0.0;

    /// @brief 内点比例，范围 0~1。
    double inlierRatio = 0.0;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_GEOMETRY_WELD_GEOMETRY_TYPES_H
