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
 * @file        WeldImageToSpaceProjector.cpp
 * @brief       图像点到空间点投影实现
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
#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "geometry/WeldImageToSpaceProjector.h"

namespace i2r_rlc::vision {

namespace {

/// @brief 投影计算中的零值保护阈值。
constexpr double kEpsilon = 1.0e-9;

/**
 * @brief 判断标量是否为有限数。
 * @param value 输入标量。
 * @return true 表示不是 NaN/Inf。
 */
bool isFinite(double value)
{
    return std::isfinite(value);
}

/**
 * @brief 校验图像点是否可投影。
 * @param point 输入图像点，坐标单位 pixel。
 * @return true 表示点有效且坐标有限。
 */
bool isValidImagePoint(const WeldImagePoint &point)
{
    return point.valid && isFinite(point.xPixel) && isFinite(point.yPixel);
}

/**
 * @brief 校验完整针孔相机内参。
 * @param intrinsics 相机内参，焦距单位 pixel。
 * @return true 表示主点和焦距有限且焦距非零。
 */
bool isValidIntrinsics(const WeldCameraIntrinsics &intrinsics)
{
    return isFinite(intrinsics.fxPixel)
        && isFinite(intrinsics.fyPixel)
        && isFinite(intrinsics.cxPixel)
        && isFinite(intrinsics.cyPixel)
        && std::fabs(intrinsics.fxPixel) > kEpsilon
        && std::fabs(intrinsics.fyPixel) > kEpsilon;
}

/**
 * @brief 校验旧标定投影所需的主点内参。
 * @param intrinsics 相机内参。
 * @return true 表示主点坐标有限。
 */
bool isValidProjectionIntrinsics(const WeldCameraIntrinsics &intrinsics)
{
    return isFinite(intrinsics.cxPixel)
        && isFinite(intrinsics.cyPixel);
}

/**
 * @brief 读取平面法向量。
 * @param plane 平面模型。
 * @return 平面法向量。
 */
Eigen::Vector3d planeNormal(const WeldPlaneModel &plane)
{
    return Eigen::Vector3d(
        plane.normalX,
        plane.normalY,
        plane.normalZ);
}

/**
 * @brief 校验观测平面模型。
 * @param plane 平面模型，空间单位为 mm。
 * @return true 表示法向、偏移有限且法向非零。
 */
bool isValidPlane(const WeldPlaneModel &plane)
{
    return isFinite(plane.normalX)
        && isFinite(plane.normalY)
        && isFinite(plane.normalZ)
        && isFinite(plane.offsetMm)
        && planeNormal(plane).norm() > kEpsilon;
}

/**
 * @brief 校验刚体变换。
 * @param transform 刚体变换，平移单位 mm。
 * @return true 表示变换标记有效且矩阵元素有限。
 */
bool isValidRigidTransform(const WeldRigidTransform &transform)
{
    return transform.valid
        && transform.rotation.allFinite()
        && transform.translationMm.allFinite();
}

bool isValidCalibration(const WeldProjectionCalibration &calibration)
{
    return calibration.valid
        && isValidProjectionIntrinsics(calibration.intrinsics)
        && isFinite(calibration.focalLengthM)
        && std::fabs(calibration.focalLengthM) > kEpsilon
        && isFinite(calibration.kappa)
        && isFinite(calibration.sxM)
        && isFinite(calibration.syM)
        && std::fabs(calibration.sxM) > kEpsilon
        && std::fabs(calibration.syM) > kEpsilon
        && isFinite(calibration.laserPlaneA)
        && isFinite(calibration.laserPlaneB)
        && isFinite(calibration.laserPlaneC);
}

bool isValidContext(const WeldProjectionContext &context)
{
    return context.valid && isValidCalibration(context.calibration);
}

WeldProjectionResult invalidResult(const std::string &message)
{
    WeldProjectionResult result;
    result.message = message;
    return result;
}

WeldSpacePoint makeSpacePoint(const Eigen::Vector3d &pointMm)
{
    WeldSpacePoint point;
    point.xMm = pointMm.x();
    point.yMm = pointMm.y();
    point.zMm = pointMm.z();
    point.valid = true;
    return point;
}

WeldSpacePoint makeInvalidSpacePoint(void)
{
    return WeldSpacePoint();
}

WeldProjectionResult makeSuccessResult(
    const Eigen::Vector3d &pointMm,
    const std::string &message)
{
    WeldProjectionResult result;
    result.valid = true;
    result.spacePoint = makeSpacePoint(pointMm);
    result.confidence = 1.0;
    result.message = message;
    return result;
}

WeldCameraIntrinsics makeLegacyProjectionIntrinsics(
    const WeldProjectionCalibration &calibration)
{
    WeldCameraIntrinsics intrinsics;
    intrinsics.fxPixel = calibration.focalLengthM / calibration.sxM;
    intrinsics.fyPixel = calibration.focalLengthM / calibration.syM;
    intrinsics.cxPixel = calibration.intrinsics.cxPixel;
    intrinsics.cyPixel = calibration.intrinsics.cyPixel;
    return intrinsics;
}

WeldPlaneModel makeLegacyLaserPlane(const WeldProjectionCalibration &calibration)
{
    WeldPlaneModel plane;
    plane.normalX = calibration.laserPlaneA;
    plane.normalY = calibration.laserPlaneB;
    plane.normalZ = calibration.laserPlaneC;
    plane.offsetMm = 1000.0;
    return plane;
}

WeldImagePoint makeLegacyUndistortedPoint(
    const WeldImagePoint &imagePoint,
    const WeldProjectionCalibration &calibration)
{
    const double deltaXPixel =
        imagePoint.xPixel - calibration.intrinsics.cxPixel;
    const double deltaYPixel =
        imagePoint.yPixel - calibration.intrinsics.cyPixel;
    const double uM = calibration.sxM * deltaXPixel;
    const double vM = calibration.syM * deltaYPixel;
    const double radialTerm = uM * uM + vM * vM;
    const double scale =
        1.0 / (1.0 + calibration.kappa * radialTerm);

    WeldImagePoint correctedPoint = imagePoint;
    correctedPoint.xPixel =
        calibration.intrinsics.cxPixel + scale * deltaXPixel;
    correctedPoint.yPixel =
        calibration.intrinsics.cyPixel + scale * deltaYPixel;
    correctedPoint.valid = true;
    return correctedPoint;
}

WeldProjectionResult transformResultToTarget(
    const WeldProjectionResult &input,
    const WeldRigidTransform &transform,
    const std::string &message)
{
    if (!input.valid)
    {
        return input;
    }

    WeldProjectionResult result =
        CWeldImageToSpaceProjector::transformSpacePoint(
            input.spacePoint,
            transform);
    if (result.valid)
    {
        result.message = message;
    }
    return result;
}

} // namespace

WeldProjectionResult CWeldImageToSpaceProjector::projectImagePoint(
    const WeldImagePoint &imagePoint,
    const WeldProjectionContext &context,
    WeldProjectionTargetFrame targetFrame)
{
    if (!isValidImagePoint(imagePoint))
    {
        return invalidResult("invalid image point");
    }

    if (!isValidContext(context))
    {
        return invalidResult("invalid projection context");
    }

    const WeldProjectionCalibration &calibration = context.calibration;
    const WeldImagePoint correctedPoint =
        makeLegacyUndistortedPoint(imagePoint, calibration);
    WeldProjectionResult result = projectImagePointToPlane(
        correctedPoint,
        makeLegacyProjectionIntrinsics(calibration),
        makeLegacyLaserPlane(calibration));
    if (!result.valid)
    {
        return result;
    }

    result.message = "image point projected to camera frame";
    if (targetFrame == WeldProjectionTargetFrame::Camera)
    {
        return result;
    }

    if (!isValidRigidTransform(calibration.cameraInTool))
    {
        return invalidResult("missing camera to tool transform");
    }

    result = transformResultToTarget(
        result,
        calibration.cameraInTool,
        "image point projected to tool frame");
    if (!result.valid || targetFrame == WeldProjectionTargetFrame::Tool)
    {
        return result;
    }

    if (!isValidRigidTransform(context.toolInRobotBase))
    {
        return invalidResult("missing tool to robot base transform");
    }

    result = transformResultToTarget(
        result,
        context.toolInRobotBase,
        "image point projected to robot base frame");
    if (!result.valid || targetFrame == WeldProjectionTargetFrame::RobotBase)
    {
        return result;
    }

    if (!isValidRigidTransform(context.robotBaseToPositionerTcp))
    {
        return invalidResult("missing robot base to positioner tcp transform");
    }

    return transformResultToTarget(
        result,
        context.robotBaseToPositionerTcp,
        "image point projected to positioner tcp frame");
}

WeldProjectionResult CWeldImageToSpaceProjector::projectCvPoint(
    const cv::Point2d &imagePoint,
    const WeldProjectionContext &context,
    WeldProjectionTargetFrame targetFrame)
{
    WeldImagePoint point;
    point.xPixel = imagePoint.x;
    point.yPixel = imagePoint.y;
    point.valid = true;
    return projectImagePoint(point, context, targetFrame);
}

std::vector<WeldSpacePoint> CWeldImageToSpaceProjector::projectImagePoints(
    const std::vector<WeldImagePoint> &imagePoints,
    const WeldProjectionContext &context,
    WeldProjectionTargetFrame targetFrame)
{
    std::vector<WeldSpacePoint> output;
    output.reserve(imagePoints.size());

    for (const WeldImagePoint &imagePoint : imagePoints)
    {
        const WeldProjectionResult result =
            projectImagePoint(imagePoint, context, targetFrame);
        output.push_back(result.valid ? result.spacePoint : makeInvalidSpacePoint());
    }

    return output;
}

WeldProjectionResult CWeldImageToSpaceProjector::projectImagePointToPlane(
    const WeldImagePoint &imagePoint,
    const WeldCameraIntrinsics &intrinsics,
    const WeldPlaneModel &plane)
{
    if (!isValidImagePoint(imagePoint))
    {
        return invalidResult("invalid image point");
    }

    if (!isValidIntrinsics(intrinsics))
    {
        return invalidResult("invalid camera intrinsics");
    }

    if (!isValidPlane(plane))
    {
        return invalidResult("invalid plane model");
    }

    const Eigen::Vector3d rayDirection(
        (imagePoint.xPixel - intrinsics.cxPixel) / intrinsics.fxPixel,
        (imagePoint.yPixel - intrinsics.cyPixel) / intrinsics.fyPixel,
        1.0);
    const Eigen::Vector3d normal = planeNormal(plane);
    const double denominator = normal.dot(rayDirection);
    if (std::fabs(denominator) <= kEpsilon)
    {
        return invalidResult("ray is parallel to plane");
    }

    const double distanceAlongRay = -plane.offsetMm / denominator;
    if (!isFinite(distanceAlongRay) || distanceAlongRay <= 0.0)
    {
        return invalidResult("intersection is behind camera");
    }

    const Eigen::Vector3d pointMm = distanceAlongRay * rayDirection;

    WeldProjectionResult result;
    result.valid = true;
    result.spacePoint = makeSpacePoint(pointMm);
    result.confidence = 1.0;
    result.message = "image point projected to plane";
    return result;
}

WeldProjectionResult CWeldImageToSpaceProjector::projectCvPointToPlane(
    const cv::Point2d &imagePoint,
    const WeldCameraIntrinsics &intrinsics,
    const WeldPlaneModel &plane)
{
    WeldImagePoint point;
    point.xPixel = imagePoint.x;
    point.yPixel = imagePoint.y;
    point.valid = true;
    return projectImagePointToPlane(point, intrinsics, plane);
}

WeldProjectionResult CWeldImageToSpaceProjector::projectFeaturePointToPlane(
    const WeldFeaturePoint &featurePoint,
    const WeldCameraIntrinsics &intrinsics,
    const WeldPlaneModel &plane)
{
    WeldImagePoint point;
    point.xPixel = featurePoint.xPixel;
    point.yPixel = featurePoint.yPixel;
    point.valid = featurePoint.valid;

    WeldProjectionResult result = projectImagePointToPlane(
        point,
        intrinsics,
        plane);
    if (result.valid)
    {
        result.confidence = std::max(0.0, std::min(1.0, featurePoint.confidence));
    }
    return result;
}

WeldProjectionResult CWeldImageToSpaceProjector::transformSpacePoint(
    const WeldSpacePoint &spacePoint,
    const WeldRigidTransform &transform)
{
    if (!spacePoint.valid
        || !isFinite(spacePoint.xMm)
        || !isFinite(spacePoint.yMm)
        || !isFinite(spacePoint.zMm))
    {
        return invalidResult("invalid space point");
    }

    if (!isValidRigidTransform(transform))
    {
        return invalidResult("invalid rigid transform");
    }

    const Eigen::Vector3d inputPoint(
        spacePoint.xMm,
        spacePoint.yMm,
        spacePoint.zMm);
    const Eigen::Vector3d outputPoint =
        transform.rotation * inputPoint + transform.translationMm;

    return makeSuccessResult(outputPoint, "space point transformed");
}

} // namespace i2r_rlc::vision
