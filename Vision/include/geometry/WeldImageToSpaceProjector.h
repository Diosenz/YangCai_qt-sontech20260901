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
 * @file        WeldImageToSpaceProjector.h
 * @brief       图像点到空间点投影接口
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
#ifndef I2R_RLC_VISION_GEOMETRY_WELD_IMAGE_TO_SPACE_PROJECTOR_H
#define I2R_RLC_VISION_GEOMETRY_WELD_IMAGE_TO_SPACE_PROJECTOR_H

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "geometry/WeldGeometryTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 图像点到焊接观测空间点的最小重建工具。
 *
 * 使用 legacy division 畸变、相机射线与显式激光平面求交，并提供已知空间点的
 * 逆相机投影。使用 Eigen 完成刚体链路，不访问真实设备或全局标定状态。
 */
class CWeldImageToSpaceProjector {
public:
    /**
     * @brief 按完整投影链路将图像点转换为空间点。
     * @param imagePoint 输入图像点，坐标单位 pixel。
     * @param context 投影上下文，包含相机内参、观测平面和外参。
     * @param targetFrame 输出目标坐标系。
     * @return 投影结果；失败时 `valid=false` 并给出 message。
     */
    static WeldProjectionResult projectImagePoint(
        const WeldImagePoint &imagePoint,
        const WeldProjectionContext &context,
        WeldProjectionTargetFrame targetFrame =
            WeldProjectionTargetFrame::RobotBase);

    /**
     * @brief 将已知空间点沿完整相机链路反投影到像素，失败时 valid=false。
     * @param spacePoint sourceFrame 中的空间点，单位 mm。
     * @param context 当前曝光帧冻结的标定与机器人位姿。
     * @param sourceFrame 输入空间点坐标系。
     * @return legacy division 畸变模型主分支的像素；不保证点落在当前激光截面上。
     *
     * 用于业务目标的跨位姿过渡，不产生独立实测或当前截面交点。
     */
    static WeldImagePoint projectSpacePointToImage(
        const WeldSpacePoint &spacePoint,
        const WeldProjectionContext &context,
        WeldProjectionTargetFrame sourceFrame);

    /**
     * @brief 按完整投影链路将 OpenCV 图像点转换为空间点。
     * @param imagePoint OpenCV 像素点，坐标单位 pixel。
     * @param context 投影上下文，包含相机内参、观测平面和外参。
     * @param targetFrame 输出目标坐标系。
     * @return 投影结果；失败时 `valid=false` 并给出 message。
     */
    static WeldProjectionResult projectCvPoint(
        const cv::Point2d &imagePoint,
        const WeldProjectionContext &context,
        WeldProjectionTargetFrame targetFrame =
            WeldProjectionTargetFrame::RobotBase);

    /**
     * @brief 按完整投影链路批量转换图像点。
     * @param imagePoints 输入图像点集合，坐标单位 pixel。
     * @param context 投影上下文，包含相机内参、观测平面和外参。
     * @param targetFrame 输出目标坐标系。
     * @return 空间点集合，单点失败时对应元素 `valid=false`。
     */
    static std::vector<WeldSpacePoint> projectImagePoints(
        const std::vector<WeldImagePoint> &imagePoints,
        const WeldProjectionContext &context,
        WeldProjectionTargetFrame targetFrame =
            WeldProjectionTargetFrame::RobotBase);

    /**
     * @brief 将图像点投影到指定观测平面。
     * @param imagePoint 输入图像点，坐标单位 pixel。
     * @param intrinsics 相机内参。
     * @param plane 目标平面模型，空间单位为 mm。
     * @return 相机坐标系下的平面交点。
     */
    static WeldProjectionResult projectImagePointToPlane(
        const WeldImagePoint &imagePoint,
        const WeldCameraIntrinsics &intrinsics,
        const WeldPlaneModel &plane);

    /**
     * @brief 将 OpenCV 像素点投影到指定观测平面。
     * @param imagePoint OpenCV 像素点，坐标单位 pixel。
     * @param intrinsics 相机内参。
     * @param plane 目标平面模型，空间单位为 mm。
     * @return 相机坐标系下的平面交点。
     */
    static WeldProjectionResult projectCvPointToPlane(
        const cv::Point2d &imagePoint,
        const WeldCameraIntrinsics &intrinsics,
        const WeldPlaneModel &plane);

    /**
     * @brief 将 feature 层候选点投影到指定观测平面。
     * @param featurePoint feature 层候选点，坐标单位 pixel。
     * @param intrinsics 相机内参。
     * @param plane 目标平面模型，空间单位为 mm。
     * @return 相机坐标系下的平面交点。
     */
    static WeldProjectionResult projectFeaturePointToPlane(
        const WeldFeaturePoint &featurePoint,
        const WeldCameraIntrinsics &intrinsics,
        const WeldPlaneModel &plane);

    /**
     * @brief 对空间点应用刚体变换。
     * @param spacePoint 输入空间点，坐标单位 mm。
     * @param transform 刚体变换，平移单位 mm。
     * @return 变换后的空间点。
     */
    static WeldProjectionResult transformSpacePoint(
        const WeldSpacePoint &spacePoint,
        const WeldRigidTransform &transform);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_GEOMETRY_WELD_IMAGE_TO_SPACE_PROJECTOR_H
