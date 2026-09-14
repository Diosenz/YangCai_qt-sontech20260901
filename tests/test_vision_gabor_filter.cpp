/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_gabor_filter.cpp
 * @brief       vision gabor filter 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "preprocess/ImageDenoise.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

std::vector<i2r_rlc::vision::MyLaserImagePoint> makeHorizontalSimPoints(void)
{
    std::vector<i2r_rlc::vision::MyLaserImagePoint> points;
    points.reserve(21);
    for (int i = 0; i <= 20; ++i)
    {
        i2r_rlc::vision::MyLaserImagePoint point;
        point.px = static_cast<double>(i);
        point.py = 10.0;
        points.push_back(point);
    }
    return points;
}

bool hasOutputShape(const cv::Mat &image, int rows, int cols)
{
    return !image.empty()
        && image.type() == CV_8UC1
        && image.rows == rows
        && image.cols == cols;
}

bool fileExists(const char *path)
{
    FILE *file = std::fopen(path, "rb");
    if (file == NULL)
    {
        return false;
    }

    std::fclose(file);
    return true;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CImageDenoise;
    using i2r_rlc::vision::GaborFilterOptions;
    using i2r_rlc::vision::MyLaserImagePoint;
    using i2r_rlc::vision::VisionPreprocessStatus;

    const GaborFilterOptions defaultOptions;
    CImageDenoise::setGaborFilterOptions(defaultOptions);
    const GaborFilterOptions currentOptions =
        CImageDenoise::gaborFilterOptions();
    if (!expectTrue(currentOptions.kernelSizePixel == 21)
        || !expectNear(currentOptions.posSigma, 4.0, 1.0e-12)
        || !expectNear(currentOptions.posLm, 90.0, 1.0e-12)
        || !expectNear(currentOptions.posPsiDeg, 90.0, 1.0e-12)
        || !expectTrue(currentOptions.neighborWindow == 10))
    {
        return 1;
    }

    GaborFilterOptions changedOptions = defaultOptions;
    changedOptions.kernelSizePixel = 23;
    changedOptions.posSigma = 5.0;
    changedOptions.posLm = 80.0;
    changedOptions.posPsiDeg = 45.0;
    changedOptions.neighborWindow = 6;
    CImageDenoise::setGaborFilterOptions(changedOptions);
    const GaborFilterOptions modifiedOptions =
        CImageDenoise::gaborFilterOptions();
    if (!expectTrue(modifiedOptions.kernelSizePixel == 23)
        || !expectNear(modifiedOptions.posSigma, 5.0, 1.0e-12)
        || !expectNear(modifiedOptions.posLm, 80.0, 1.0e-12)
        || !expectNear(modifiedOptions.posPsiDeg, 45.0, 1.0e-12)
        || !expectTrue(modifiedOptions.neighborWindow == 6))
    {
        CImageDenoise::setGaborFilterOptions(defaultOptions);
        return 2;
    }
    CImageDenoise::setGaborFilterOptions(defaultOptions);

    const auto simPoints = makeHorizontalSimPoints();
    MyLaserImagePoint weldPoint;
    weldPoint.px = 10.0;
    weldPoint.py = 10.0;
    const auto angleResult =
        CImageDenoise::estimateGaborFilterAngle(simPoints, weldPoint);
    if (!expectTrue(angleResult.valid)
        || !expectTrue(angleResult.weldPointIndex == 10)
        || !expectNear(angleResult.filterAngleDeg, 90.0, 1.0e-9))
    {
        return 3;
    }

    cv::Mat inputImage = cv::Mat::zeros(32, 32, CV_8UC1);
    cv::line(inputImage, cv::Point(0, 16), cv::Point(31, 16), cv::Scalar(180), 2);
    const cv::Mat originalImage = inputImage.clone();
    const bool hadGaborPng = fileExists("gabor.png");
    const bool hadGabor255Png = fileExists("gabor255.png");
    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const cv::Mat outputImage = CImageDenoise::gaborDenoise(
        inputImage,
        simPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(outputImage, 32, 32))
        || !expectTrue(cv::countNonZero(inputImage != originalImage) == 0))
    {
        return 4;
    }

    const cv::Mat emptyOutput = CImageDenoise::gaborDenoise(
        cv::Mat(),
        simPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::EmptyInput)
        || !expectTrue(emptyOutput.empty()))
    {
        return 5;
    }

    cv::Mat colorInput(16, 16, CV_8UC3, cv::Scalar(0, 0, 0));
    const cv::Mat invalidGrayOutput = CImageDenoise::gaborDenoise(
        colorInput,
        simPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InvalidGrayImage)
        || !expectTrue(invalidGrayOutput.empty()))
    {
        return 6;
    }

    const std::vector<MyLaserImagePoint> emptyPoints;
    const cv::Mat emptyPointsOutput = CImageDenoise::gaborDenoise(
        inputImage,
        emptyPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InsufficientPoints)
        || !expectTrue(emptyPointsOutput.empty()))
    {
        return 7;
    }

    std::vector<MyLaserImagePoint> fewPoints;
    fewPoints.push_back(weldPoint);
    const cv::Mat fewPointsOutput = CImageDenoise::gaborDenoise(
        inputImage,
        fewPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InsufficientPoints)
        || !expectTrue(fewPointsOutput.empty()))
    {
        return 8;
    }

    MyLaserImagePoint leftEdgeWeldPoint = simPoints.front();
    const cv::Mat noLeftOutput = CImageDenoise::gaborDenoise(
        inputImage,
        simPoints,
        leftEdgeWeldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InsufficientPoints)
        || !expectTrue(noLeftOutput.empty()))
    {
        return 9;
    }

    MyLaserImagePoint rightEdgeWeldPoint = simPoints.back();
    const cv::Mat noRightOutput = CImageDenoise::gaborDenoise(
        inputImage,
        simPoints,
        rightEdgeWeldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InsufficientPoints)
        || !expectTrue(noRightOutput.empty()))
    {
        return 10;
    }

    GaborFilterOptions invalidOptions = defaultOptions;
    invalidOptions.kernelSizePixel = 20;
    CImageDenoise::setGaborFilterOptions(invalidOptions);
    const cv::Mat invalidKernelOutput = CImageDenoise::gaborDenoise(
        inputImage,
        simPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InvalidKernel)
        || !expectTrue(invalidKernelOutput.empty()))
    {
        CImageDenoise::setGaborFilterOptions(defaultOptions);
        return 11;
    }
    CImageDenoise::setGaborFilterOptions(defaultOptions);

    GaborFilterOptions degenerateOptions = defaultOptions;
    degenerateOptions.denominatorEpsilon = 100.0;
    CImageDenoise::setGaborFilterOptions(degenerateOptions);
    const cv::Mat degenerateOutput = CImageDenoise::gaborDenoise(
        inputImage,
        simPoints,
        weldPoint,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::DegenerateGeometry)
        || !expectTrue(degenerateOutput.empty()))
    {
        CImageDenoise::setGaborFilterOptions(defaultOptions);
        return 12;
    }
    CImageDenoise::setGaborFilterOptions(defaultOptions);

    if (fileExists("gabor.png") != hadGaborPng
        || fileExists("gabor255.png") != hadGabor255Png)
    {
        return 13;
    }

    std::printf("PASS: vision gabor filter tests\n");
    return 0;
}
