/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_skeleton_feature.cpp
 * @brief       vision skeleton feature 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "stripe/WeldSkeletonFeatureTools.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

int fail(int code)
{
    std::printf("FAIL: vision skeleton feature tests code=%d\n", code);
    return code;
}

cv::Mat makeHorizontalStripe(void)
{
    cv::Mat image(40, 80, CV_8UC1, cv::Scalar(0));
    cv::rectangle(image, cv::Rect(10, 18, 55, 5), cv::Scalar(255), cv::FILLED);
    return image;
}

cv::Mat makeTShape(void)
{
    cv::Mat image(70, 70, CV_8UC1, cv::Scalar(0));
    cv::rectangle(image, cv::Rect(12, 28, 45, 5), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(32, 28, 5, 28), cv::Scalar(255), cv::FILLED);
    return image;
}

cv::Mat makeBrokenStripe(void)
{
    cv::Mat image(40, 80, CV_8UC1, cv::Scalar(0));
    cv::rectangle(image, cv::Rect(8, 18, 20, 5), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(42, 18, 22, 5), cv::Scalar(255), cv::FILLED);
    return image;
}

cv::Mat makeNoiseOnly(void)
{
    cv::Mat image(40, 40, CV_8UC1, cv::Scalar(0));
    image.at<unsigned char>(5, 5) = 255;
    image.at<unsigned char>(20, 18) = 255;
    return image;
}

cv::Mat makeLargeLaserRegion(void)
{
    cv::Mat image(90, 120, CV_8UC1, cv::Scalar(0));
    cv::rectangle(image, cv::Rect(20, 36, 68, 12), cv::Scalar(255), cv::FILLED);
    cv::rectangle(image, cv::Rect(53, 24, 11, 42), cv::Scalar(255), cv::FILLED);
    return image;
}

i2r_rlc::vision::WeldSkeletonExtractionOptions makeOptions(void)
{
    i2r_rlc::vision::WeldSkeletonExtractionOptions options;
    options.inputAlreadyBinary = true;
    options.minComponentArea = 3;
    options.maxIterations = 80;
    options.minConfidence = 0.10;
    return options;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CWeldSkeletonFeatureTools;
    using i2r_rlc::vision::LaserSkeletonExtractionOptions;

    const auto options = makeOptions();

    const auto emptyResult = CWeldSkeletonFeatureTools::extract(
        cv::Mat(),
        options);
    if (!expectTrue(!emptyResult.valid)
        || !expectTrue(emptyResult.message == "empty input"))
    {
        return fail(1);
    }

    const auto colorResult = CWeldSkeletonFeatureTools::extract(
        cv::Mat::zeros(20, 20, CV_8UC3),
        options);
    if (!expectTrue(!colorResult.valid)
        || !expectTrue(colorResult.message == "input must be CV_8UC1"))
    {
        return fail(2);
    }

    const auto horizontalResult = CWeldSkeletonFeatureTools::extract(
        makeHorizontalStripe(),
        options);
    if (!expectTrue(horizontalResult.valid)
        || !expectTrue(horizontalResult.connectedComponentCount == 1)
        || !expectTrue(horizontalResult.endpointCount == 2)
        || !expectTrue(horizontalResult.branchPointCount == 0)
        || !expectTrue(horizontalResult.skeletonMask.type() == CV_8UC1)
        || !expectTrue(horizontalResult.skeletonPoints.size() >= 45)
        || !expectTrue(horizontalResult.validPixelRatio > 0.0)
        || !expectTrue(horizontalResult.validPixelRatio < 1.0)
        || !expectTrue(horizontalResult.confidence >= 0.0)
        || !expectTrue(horizontalResult.confidence <= 1.0))
    {
        return fail(3);
    }

    const auto tShapeResult = CWeldSkeletonFeatureTools::extract(
        makeTShape(),
        options);
    if (!expectTrue(tShapeResult.valid)
        || !expectTrue(tShapeResult.connectedComponentCount == 1)
        || !expectTrue(tShapeResult.endpointCount >= 3)
        || !expectTrue(tShapeResult.branchPointCount >= 1))
    {
        return fail(4);
    }

    const auto brokenResult = CWeldSkeletonFeatureTools::extract(
        makeBrokenStripe(),
        options);
    if (!expectTrue(!brokenResult.valid)
        || !expectTrue(brokenResult.connectedComponentCount == 2)
        || !expectTrue(brokenResult.message == "skeleton is broken"))
    {
        return fail(5);
    }

    const auto noiseResult = CWeldSkeletonFeatureTools::extract(
        makeNoiseOnly(),
        options);
    if (!expectTrue(!noiseResult.valid)
        || !expectTrue(noiseResult.skeletonPoints.empty()))
    {
        return fail(6);
    }

    const cv::Mat skeletonMask = CWeldSkeletonFeatureTools::thinBinaryMask(
        makeHorizontalStripe(),
        options.maxIterations);
    const auto points =
        CWeldSkeletonFeatureTools::collectSkeletonPoints(skeletonMask);
    if (!expectTrue(!skeletonMask.empty())
        || !expectTrue(!points.empty()))
    {
        return fail(7);
    }

    LaserSkeletonExtractionOptions laserOptions;
    laserOptions.thresholdValue = 128;
    laserOptions.minComponentArea = 3;
    laserOptions.closingRadiusPixel = 0;
    laserOptions.minCenterPointCount = 5;
    laserOptions.enableHoughFallback = false;
    const auto laserSkeletonResult =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(
            makeTShape(),
            laserOptions);
    if (!expectTrue(laserSkeletonResult.valid)
        || !expectTrue(!laserSkeletonResult.skeletonMask.empty())
        || !expectTrue(!laserSkeletonResult.refinedCenterPoints.empty())
        || !expectTrue(laserSkeletonResult.branchPointCount >= 1)
        || !expectTrue(laserSkeletonResult.skeletonPoints.size()
            > laserSkeletonResult.refinedCenterPoints.size()))
    {
        return fail(8);
    }

    const auto largeRegionResult =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(
            makeLargeLaserRegion(),
            laserOptions);
    if (!expectTrue(largeRegionResult.valid)
        || !expectTrue(cv::countNonZero(largeRegionResult.skeletonMask)
            == static_cast<int>(largeRegionResult.skeletonPoints.size()))
        || !expectTrue(largeRegionResult.skeletonPoints.size()
            > largeRegionResult.refinedCenterPoints.size()))
    {
        return fail(9);
    }

    std::printf("PASS: vision skeleton feature tests\n");
    return 0;
}
