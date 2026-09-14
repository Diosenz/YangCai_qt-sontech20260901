/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_to_space.cpp
 * @brief       vision image to space 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "geometry/WeldImageToSpaceProjector.h"
#include "stripe/DpStripeCenterlineDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

i2r_rlc::vision::WeldCameraIntrinsics makePlaneIntrinsics(void)
{
    i2r_rlc::vision::WeldCameraIntrinsics intrinsics;
    intrinsics.fxPixel = 100.0;
    intrinsics.fyPixel = 100.0;
    intrinsics.cxPixel = 50.0;
    intrinsics.cyPixel = 40.0;
    return intrinsics;
}

i2r_rlc::vision::WeldPlaneModel makeZPlane(double zMm)
{
    i2r_rlc::vision::WeldPlaneModel plane;
    plane.normalX = 0.0;
    plane.normalY = 0.0;
    plane.normalZ = 1.0;
    plane.offsetMm = -zMm;
    return plane;
}

i2r_rlc::vision::WeldProjectionCalibration makeProjectionCalibration(void)
{
    i2r_rlc::vision::WeldProjectionCalibration calibration;
    calibration.intrinsics.cxPixel = 50.0;
    calibration.intrinsics.cyPixel = 40.0;
    calibration.focalLengthM = 0.01;
    calibration.kappa = 0.0;
    calibration.sxM = 0.0001;
    calibration.syM = 0.0001;
    calibration.laserPlaneA = 0.0;
    calibration.laserPlaneB = 0.0;
    calibration.laserPlaneC = -1.0;
    calibration.cameraInTool.translationMm = Eigen::Vector3d(10.0, 20.0, 30.0);
    calibration.valid = true;
    return calibration;
}

i2r_rlc::vision::WeldProjectionContext makeProjectionContext(void)
{
    i2r_rlc::vision::WeldProjectionContext context;
    context.calibration = makeProjectionCalibration();
    context.toolInRobotBase.translationMm = Eigen::Vector3d(100.0, 200.0, 300.0);
    context.robotBaseToPositionerTcp.translationMm =
        Eigen::Vector3d(-5.0, 6.0, 7.0);
    context.valid = true;
    return context;
}

cv::Mat makeProjectionStripeImage(void)
{
    cv::Mat image(70, 100, CV_8UC1, cv::Scalar(4));
    for (int xPixel = 10; xPixel < 90; ++xPixel) {
        cv::line(
            image,
            cv::Point(xPixel, 34),
            cv::Point(xPixel, 36),
            cv::Scalar(230),
            1);
    }
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0.5);
    return image;
}

Eigen::Vector3d computeLegacyCameraPointMm(
    double xPixel,
    double yPixel,
    const i2r_rlc::vision::WeldProjectionCalibration &calibration)
{
    const double uM =
        calibration.sxM * (xPixel - calibration.intrinsics.cxPixel);
    const double vM =
        calibration.syM * (yPixel - calibration.intrinsics.cyPixel);
    const double scale =
        1.0 / (1.0 + calibration.kappa * (uM * uM + vM * vM));
    const double correctedUM = scale * uM;
    const double correctedVM = scale * vM;
    const double g =
        -1.0 / (calibration.laserPlaneA * correctedUM
            + calibration.laserPlaneB * correctedVM
            + calibration.laserPlaneC * calibration.focalLengthM);

    return Eigen::Vector3d(
        correctedUM * g * 1000.0,
        correctedVM * g * 1000.0,
        calibration.focalLengthM * g * 1000.0);
}

} // namespace

int main()
{
    using i2r_rlc::vision::CWeldImageToSpaceProjector;
    using i2r_rlc::vision::WeldFeaturePoint;
    using i2r_rlc::vision::WeldImagePoint;
    using i2r_rlc::vision::WeldProjectionTargetFrame;
    using i2r_rlc::vision::WeldRigidTransform;

    const auto intrinsics = makePlaneIntrinsics();
    const auto plane = makeZPlane(1000.0);

    WeldImagePoint centerPoint;
    centerPoint.xPixel = 50.0;
    centerPoint.yPixel = 40.0;
    centerPoint.valid = true;
    const auto centerResult =
        CWeldImageToSpaceProjector::projectImagePointToPlane(
            centerPoint,
            intrinsics,
            plane);
    if (!expectTrue(centerResult.valid)
        || !expectNear(centerResult.spacePoint.xMm, 0.0, 1.0e-9)
        || !expectNear(centerResult.spacePoint.yMm, 0.0, 1.0e-9)
        || !expectNear(centerResult.spacePoint.zMm, 1000.0, 1.0e-9))
    {
        return 1;
    }

    const auto offsetResult =
        CWeldImageToSpaceProjector::projectCvPointToPlane(
            cv::Point2d(60.0, 30.0),
            intrinsics,
            plane);
    if (!expectTrue(offsetResult.valid)
        || !expectNear(offsetResult.spacePoint.xMm, 100.0, 1.0e-9)
        || !expectNear(offsetResult.spacePoint.yMm, -100.0, 1.0e-9)
        || !expectNear(offsetResult.spacePoint.zMm, 1000.0, 1.0e-9))
    {
        return 2;
    }

    WeldFeaturePoint featurePoint;
    featurePoint.xPixel = 55.0;
    featurePoint.yPixel = 45.0;
    featurePoint.confidence = 0.72;
    featurePoint.valid = true;
    const auto featureResult =
        CWeldImageToSpaceProjector::projectFeaturePointToPlane(
            featurePoint,
            intrinsics,
            plane);
    if (!expectTrue(featureResult.valid)
        || !expectNear(featureResult.spacePoint.xMm, 50.0, 1.0e-9)
        || !expectNear(featureResult.spacePoint.yMm, 50.0, 1.0e-9)
        || !expectNear(featureResult.confidence, 0.72, 1.0e-9))
    {
        return 3;
    }

    auto invalidIntrinsics = intrinsics;
    invalidIntrinsics.fxPixel = 0.0;
    if (!expectTrue(!CWeldImageToSpaceProjector::projectImagePointToPlane(
                        centerPoint,
                        invalidIntrinsics,
                        plane)
                         .valid))
    {
        return 4;
    }

    i2r_rlc::vision::WeldPlaneModel invalidPlane;
    invalidPlane.normalX = 0.0;
    invalidPlane.normalY = 0.0;
    invalidPlane.normalZ = 0.0;
    if (!expectTrue(!CWeldImageToSpaceProjector::projectImagePointToPlane(
                        centerPoint,
                        intrinsics,
                        invalidPlane)
                         .valid))
    {
        return 5;
    }

    i2r_rlc::vision::WeldPlaneModel parallelPlane;
    parallelPlane.normalX = 1.0;
    parallelPlane.normalY = 0.0;
    parallelPlane.normalZ = 0.0;
    parallelPlane.offsetMm = -10.0;
    if (!expectTrue(!CWeldImageToSpaceProjector::projectImagePointToPlane(
                        centerPoint,
                        intrinsics,
                        parallelPlane)
                         .valid))
    {
        return 6;
    }

    const auto behindResult =
        CWeldImageToSpaceProjector::projectImagePointToPlane(
            centerPoint,
            intrinsics,
            makeZPlane(-100.0));
    if (!expectTrue(!behindResult.valid))
    {
        return 7;
    }

    WeldRigidTransform transform;
    transform.translationMm = Eigen::Vector3d(10.0, 20.0, -30.0);
    const auto transformed =
        CWeldImageToSpaceProjector::transformSpacePoint(
            centerResult.spacePoint,
            transform);
    if (!expectTrue(transformed.valid)
        || !expectNear(transformed.spacePoint.xMm, 10.0, 1.0e-9)
        || !expectNear(transformed.spacePoint.yMm, 20.0, 1.0e-9)
        || !expectNear(transformed.spacePoint.zMm, 970.0, 1.0e-9))
    {
        return 8;
    }

    const auto context = makeProjectionContext();
    const Eigen::Vector3d expectedCameraPoint =
        computeLegacyCameraPointMm(60.0, 30.0, context.calibration);
    const auto projectedCameraPoint = CWeldImageToSpaceProjector::projectImagePoint(
        centerPoint,
        context,
        WeldProjectionTargetFrame::Camera);
    if (!expectTrue(projectedCameraPoint.valid)
        || !expectNear(projectedCameraPoint.spacePoint.xMm, 0.0, 1.0e-9)
        || !expectNear(projectedCameraPoint.spacePoint.yMm, 0.0, 1.0e-9)
        || !expectNear(projectedCameraPoint.spacePoint.zMm, 1000.0, 1.0e-9))
    {
        return 9;
    }

    const auto projectedOffsetCamera =
        CWeldImageToSpaceProjector::projectCvPoint(
            cv::Point2d(60.0, 30.0),
            context,
            WeldProjectionTargetFrame::Camera);
    if (!expectTrue(projectedOffsetCamera.valid)
        || !expectNear(
            projectedOffsetCamera.spacePoint.xMm,
            expectedCameraPoint.x(),
            1.0e-9)
        || !expectNear(
            projectedOffsetCamera.spacePoint.yMm,
            expectedCameraPoint.y(),
            1.0e-9)
        || !expectNear(
            projectedOffsetCamera.spacePoint.zMm,
            expectedCameraPoint.z(),
            1.0e-9))
    {
        return 10;
    }

    const auto projectedToolPoint = CWeldImageToSpaceProjector::projectImagePoint(
        centerPoint,
        context,
        WeldProjectionTargetFrame::Tool);
    if (!expectTrue(projectedToolPoint.valid)
        || !expectNear(projectedToolPoint.spacePoint.xMm, 10.0, 1.0e-9)
        || !expectNear(projectedToolPoint.spacePoint.yMm, 20.0, 1.0e-9)
        || !expectNear(projectedToolPoint.spacePoint.zMm, 1030.0, 1.0e-9))
    {
        return 11;
    }

    const auto projectedBasePoint = CWeldImageToSpaceProjector::projectImagePoint(
        centerPoint,
        context,
        WeldProjectionTargetFrame::RobotBase);
    if (!expectTrue(projectedBasePoint.valid)
        || !expectNear(projectedBasePoint.spacePoint.xMm, 110.0, 1.0e-9)
        || !expectNear(projectedBasePoint.spacePoint.yMm, 220.0, 1.0e-9)
        || !expectNear(projectedBasePoint.spacePoint.zMm, 1330.0, 1.0e-9))
    {
        return 12;
    }

    const auto projectedTcpPoint = CWeldImageToSpaceProjector::projectImagePoint(
        centerPoint,
        context,
        WeldProjectionTargetFrame::PositionerTcp);
    if (!expectTrue(projectedTcpPoint.valid)
        || !expectNear(projectedTcpPoint.spacePoint.xMm, 105.0, 1.0e-9)
        || !expectNear(projectedTcpPoint.spacePoint.yMm, 226.0, 1.0e-9)
        || !expectNear(projectedTcpPoint.spacePoint.zMm, 1337.0, 1.0e-9))
    {
        return 13;
    }

    std::vector<WeldImagePoint> batchPoints;
    batchPoints.push_back(centerPoint);
    batchPoints.push_back(WeldImagePoint {60.0, 30.0, true});
    batchPoints.push_back(WeldImagePoint {0.0, 0.0, false});
    const std::vector<i2r_rlc::vision::WeldSpacePoint> batchResult =
        CWeldImageToSpaceProjector::projectImagePoints(
            batchPoints,
            context,
            WeldProjectionTargetFrame::RobotBase);
    if (!expectTrue(batchResult.size() == batchPoints.size())
        || !batchResult[0].valid
        || !batchResult[1].valid
        || batchResult[2].valid)
    {
        return 14;
    }

    const auto singleSecondPoint = CWeldImageToSpaceProjector::projectImagePoint(
        batchPoints[1],
        context,
        WeldProjectionTargetFrame::RobotBase);
    if (!expectTrue(singleSecondPoint.valid)
        || !expectNear(batchResult[1].xMm, singleSecondPoint.spacePoint.xMm, 1.0e-9)
        || !expectNear(batchResult[1].yMm, singleSecondPoint.spacePoint.yMm, 1.0e-9)
        || !expectNear(batchResult[1].zMm, singleSecondPoint.spacePoint.zMm, 1.0e-9))
    {
        return 15;
    }

    auto missingBaseContext = context;
    missingBaseContext.toolInRobotBase.valid = false;
    if (!expectTrue(!CWeldImageToSpaceProjector::projectImagePoint(
                        centerPoint,
                        missingBaseContext,
                        WeldProjectionTargetFrame::RobotBase)
                         .valid))
    {
        return 16;
    }

    auto missingTcpContext = context;
    missingTcpContext.robotBaseToPositionerTcp.valid = false;
    if (!expectTrue(!CWeldImageToSpaceProjector::projectImagePoint(
                        centerPoint,
                        missingTcpContext,
                        WeldProjectionTargetFrame::PositionerTcp)
                         .valid))
    {
        return 17;
    }

    auto distortedContext = context;
    distortedContext.calibration.kappa = 1000.0;
    const auto distortedResult = CWeldImageToSpaceProjector::projectCvPoint(
        cv::Point2d(70.0, 45.0),
        distortedContext,
        WeldProjectionTargetFrame::Camera);
    const Eigen::Vector3d distortedExpected = computeLegacyCameraPointMm(
        70.0,
        45.0,
        distortedContext.calibration);
    if (!expectTrue(distortedResult.valid)
        || !expectNear(distortedResult.spacePoint.xMm, distortedExpected.x(), 1.0e-9)
        || !expectNear(distortedResult.spacePoint.yMm, distortedExpected.y(), 1.0e-9)
        || !expectNear(distortedResult.spacePoint.zMm, distortedExpected.z(), 1.0e-9))
    {
        return 18;
    }

    i2r_rlc::vision::DpStripeDetectionOptions dpOptions;
    dpOptions.minValidPointCount = 50;
    dpOptions.useDirectionPrior = false;
    dpOptions.maxJumpPixel = 5.0;
    const auto stripeResult =
        i2r_rlc::vision::CDpStripeCenterlineDetector::detect(
            makeProjectionStripeImage(),
            dpOptions);
    if (!expectTrue(stripeResult.valid)) {
        return 19;
    }

    std::vector<WeldImagePoint> stripeImagePoints;
    stripeImagePoints.reserve(stripeResult.centerlinePoints.size());
    for (const auto &point : stripeResult.centerlinePoints) {
        stripeImagePoints.push_back(WeldImagePoint {point.x, point.y, true});
    }
    const auto stripeSpacePoints =
        CWeldImageToSpaceProjector::projectImagePoints(
            stripeImagePoints,
            context,
            WeldProjectionTargetFrame::RobotBase);
    if (!expectTrue(stripeSpacePoints.size() == stripeImagePoints.size())
        || !expectTrue(!stripeSpacePoints.empty())
        || !expectTrue(stripeSpacePoints.front().valid)
        || !expectTrue(stripeSpacePoints.back().valid))
    {
        return 20;
    }

    std::printf("PASS: vision image to space tests\n");
    return 0;
}
