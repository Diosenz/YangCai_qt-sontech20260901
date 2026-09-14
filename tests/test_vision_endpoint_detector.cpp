/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_endpoint_detector.cpp
 * @brief       vision endpoint detector 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include "feature/WeldEndpointDetector.h"
#include "feature/WeldFeatureDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

cv::Mat makeImage(void)
{
    cv::Mat image(70, 100, CV_8UC1, cv::Scalar(40));
    for (int xPixel = 10; xPixel <= 80; ++xPixel)
    {
        image.at<unsigned char>(30, xPixel) = 220;
        image.at<unsigned char>(31, xPixel) = 210;
    }
    image.at<unsigned char>(8, 8) = 255;
    image.at<unsigned char>(65, 90) = 255;
    return image;
}

i2r_rlc::vision::WeldFeatureDetectionOptions makeOptions(void)
{
    i2r_rlc::vision::WeldFeatureDetectionOptions options;
    options.minConfidence = 0.10;
    options.minEndpointPointCount = 2;
    options.minEndpointDistanceFromWeldPointPixel = 3.0;
    return options;
}

i2r_rlc::vision::LaserCenterlinePoint makeCenterlinePoint(
    double xPixel,
    double yPixel)
{
    i2r_rlc::vision::LaserCenterlinePoint point;
    point.xPixel = xPixel;
    point.yPixel = yPixel;
    point.response = 220.0;
    point.localContrast = 160.0;
    point.confidence = 0.90;
    point.valid = true;
    return point;
}

i2r_rlc::vision::LaserCenterlineResult makeCenterline(
    const std::vector<cv::Point2d> &points)
{
    i2r_rlc::vision::LaserCenterlineResult centerline;
    centerline.valid = !points.empty();
    centerline.meanConfidence = 0.90;
    centerline.validColumnRatio = 1.0;
    for (const cv::Point2d &point : points)
    {
        centerline.points.push_back(makeCenterlinePoint(point.x, point.y));
    }
    return centerline;
}

i2r_rlc::vision::WeldFeatureResult makeFeatureResult(void)
{
    i2r_rlc::vision::WeldFeatureResult feature;
    feature.valid = true;
    feature.confidence = 0.75;
    feature.gapFeature.valid = true;
    feature.gapFeature.weldPoint.valid = true;
    feature.gapFeature.weldPoint.xPixel = 45.0;
    feature.gapFeature.weldPoint.yPixel = 30.0;
    feature.gapFeature.weldPoint.confidence = 0.80;
    feature.gapFeature.weldPoint.label = "weld_point";
    feature.gapFeature.confidence = 0.75;
    return feature;
}

std::vector<cv::Point2d> makeHorizontalPoints(void)
{
    std::vector<cv::Point2d> points;
    for (int xPixel = 10; xPixel <= 80; xPixel += 10)
    {
        points.push_back(cv::Point2d(
            static_cast<double>(xPixel),
            30.0));
    }
    return points;
}

std::vector<cv::Point2d> makeTiltedPoints(void)
{
    std::vector<cv::Point2d> points;
    for (int index = 0; index < 8; ++index)
    {
        points.push_back(cv::Point2d(
            12.0 + 8.0 * static_cast<double>(index),
            20.0 + 1.5 * static_cast<double>(index)));
    }
    return points;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CWeldEndpointDetector;
    using i2r_rlc::vision::CWeldFeatureDetector;

    const cv::Mat image = makeImage();
    const auto options = makeOptions();
    const auto feature = makeFeatureResult();

    const auto horizontalResult = CWeldEndpointDetector::detect(
        image,
        makeCenterline(makeHorizontalPoints()),
        feature,
        options);
    if (!expectTrue(horizontalResult.valid)
        || !expectTrue(horizontalResult.startPoint.valid)
        || !expectTrue(horizontalResult.endPoint.valid)
        || !expectNear(horizontalResult.startPoint.xPixel, 10.0, 0.001)
        || !expectNear(horizontalResult.endPoint.xPixel, 80.0, 0.001)
        || !expectTrue(horizontalResult.startPoint.xPixel
                      < horizontalResult.endPoint.xPixel)
        || !expectTrue(horizontalResult.confidence >= 0.0)
        || !expectTrue(horizontalResult.confidence <= 1.0))
    {
        return 1;
    }

    const auto tiltedResult = CWeldEndpointDetector::detect(
        image,
        makeCenterline(makeTiltedPoints()),
        feature,
        options);
    if (!expectTrue(tiltedResult.valid)
        || !expectNear(tiltedResult.startPoint.xPixel, 12.0, 0.001)
        || !expectNear(tiltedResult.endPoint.xPixel, 68.0, 0.001)
        || !expectTrue(tiltedResult.startPoint.yPixel
                      < tiltedResult.endPoint.yPixel))
    {
        return 2;
    }

    const auto insufficientResult = CWeldEndpointDetector::detect(
        image,
        makeCenterline(std::vector<cv::Point2d>{
            cv::Point2d(30.0, 30.0)}),
        feature,
        options);
    if (!expectTrue(!insufficientResult.valid)
        || !expectTrue(insufficientResult.message
                      == "not enough centerline points"))
    {
        return 3;
    }

    const auto noiseStableResult = CWeldEndpointDetector::detect(
        image,
        makeCenterline(makeHorizontalPoints()),
        i2r_rlc::vision::WeldFeatureResult(),
        options);
    if (!expectTrue(noiseStableResult.valid)
        || !expectNear(noiseStableResult.startPoint.xPixel, 10.0, 0.001)
        || !expectNear(noiseStableResult.endPoint.xPixel, 80.0, 0.001))
    {
        return 4;
    }

    auto conflictFeature = feature;
    conflictFeature.gapFeature.weldPoint.xPixel = 10.0;
    conflictFeature.gapFeature.weldPoint.yPixel = 30.0;
    const auto conflictResult = CWeldEndpointDetector::detect(
        image,
        makeCenterline(makeHorizontalPoints()),
        conflictFeature,
        options);
    if (!expectTrue(!conflictResult.valid)
        || !expectTrue(conflictResult.message
                      == "endpoint conflicts with weld point"))
    {
        return 5;
    }

    const auto wrapperResult = CWeldFeatureDetector::detect(
        image,
        makeCenterline(makeHorizontalPoints()),
        options);
    if (!expectTrue(wrapperResult.valid)
        || !expectTrue(wrapperResult.endpointResult.valid)
        || !expectTrue(wrapperResult.endpointResult.startPoint.xPixel
                      < wrapperResult.endpointResult.endPoint.xPixel))
    {
        return 6;
    }

    const auto colorResult = CWeldEndpointDetector::detect(
        cv::Mat::zeros(10, 10, CV_8UC3),
        makeCenterline(makeHorizontalPoints()),
        feature,
        options);
    if (!expectTrue(!colorResult.valid)
        || !expectTrue(colorResult.message == "input must be CV_8UC1"))
    {
        return 7;
    }

    std::printf("PASS: vision endpoint detector tests\n");
    return 0;
}
