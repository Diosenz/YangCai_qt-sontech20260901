/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_active_contour_feature.cpp
 * @brief       vision active contour feature 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "feature/ActiveContourWeldFeatureDetector.h"
#include "stripe/ActiveContourStripeExtractor.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

i2r_rlc::vision::ActiveContourDetectionOptions makeOptions(void)
{
    i2r_rlc::vision::ActiveContourDetectionOptions options;
    options.laserWidthThresholdGrayValue = 180;
    options.laserWidthMaxPixel = 12;
    options.initialThresholdGrayValue = 120;
    options.initialClosingRadiusPixel = 1;
    options.initialControlPointStepPixel = 4;
    options.interpolationSamplesPerSegment = 4;
    options.snakeIterationCount = 20;
    options.snakeMinGrayValue = 180;
    options.firstFrameRoiHalfWidthPixel = 80;
    options.tJointRoiHalfWidthPixel = 80;
    options.buttRoiHalfWidthPixel = 80;
    options.lapRoiHalfWidthPixel = 80;
    options.tJointSplitDistancePixel = 2.0;
    options.buttSplitDistancePixel = 2.0;
    options.lapSplitDistancePixel = 2.0;
    options.ransacIterations = 40;
    options.tJointRansacDistancePixel = 1.5;
    options.buttRansacDistancePixel = 1.5;
    options.lapRansacDistancePixel = 1.5;
    options.ransacMinInlierRatio = 0.55;
    options.gapBrightThresholdGrayValue = 180;
    options.buttGapEndpointMarginPixel = 5;
    options.buttGapScanPaddingPixel = 2;
    options.maxTJointJumpPixel = 100.0;
    options.maxLapJumpPixel = 15.0;
    options.lapJumpFilterStartFrame = 2;
    options.minConfidence = 0.10;
    return options;
}

cv::Mat makeWidthImage(void)
{
    cv::Mat image(40, 60, CV_8UC1, cv::Scalar(0));
    for (int xPixel = 0; xPixel < image.cols; ++xPixel)
    {
        for (int yPixel = 18; yPixel <= 22; ++yPixel)
        {
            image.at<unsigned char>(yPixel, xPixel) = 255;
        }
    }
    return image;
}

cv::Mat makeHorizontalLineImage(void)
{
    cv::Mat image(50, 80, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 25), cv::Point(74, 25), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeLongHorizontalLineImage(void)
{
    cv::Mat image(50, 100, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 25), cv::Point(94, 25), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeBranchedAndFloatingInitialImage(void)
{
    cv::Mat image(70, 120, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 35), cv::Point(114, 35), cv::Scalar(255), 3);
    cv::line(image, cv::Point(58, 10), cv::Point(58, 35), cv::Scalar(255), 3);
    cv::line(image, cv::Point(25, 20), cv::Point(90, 20), cv::Scalar(170), 3);
    return image;
}

cv::Mat makeButtImage(void)
{
    cv::Mat image(60, 90, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 30), cv::Point(32, 30), cv::Scalar(255), 3);
    cv::line(image, cv::Point(48, 30), cv::Point(84, 30), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeTJointImage(void)
{
    cv::Mat image(70, 90, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 20), cv::Point(45, 45), cv::Scalar(255), 3);
    cv::line(image, cv::Point(45, 45), cv::Point(84, 18), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeDualTJointImage(void)
{
    cv::Mat image(80, 140, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(4, 22), cv::Point(35, 48), cv::Scalar(255), 3);
    cv::line(image, cv::Point(35, 48), cv::Point(66, 22), cv::Scalar(255), 3);
    cv::line(image, cv::Point(72, 24), cv::Point(108, 58), cv::Scalar(255), 3);
    cv::line(image, cv::Point(108, 58), cv::Point(136, 24), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeAnchoredButtImage(void)
{
    cv::Mat image(70, 130, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(8, 42), cv::Point(45, 28), cv::Scalar(255), 3);
    cv::line(image, cv::Point(57, 28), cv::Point(83, 28), cv::Scalar(255), 3);
    cv::line(image, cv::Point(95, 28), cv::Point(122, 42), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeLapImage(void)
{
    cv::Mat image(80, 90, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 24), cv::Point(42, 55), cv::Scalar(255), 3);
    cv::line(image, cv::Point(42, 55), cv::Point(84, 30), cv::Scalar(255), 3);
    return image;
}

i2r_rlc::vision::ActiveContourDetectionState makeInitializedState(
    const cv::Mat &image,
    const i2r_rlc::vision::ActiveContourDetectionOptions &options)
{
    i2r_rlc::vision::ActiveContourDetectionState state;
    state.initialized = true;
    state.frameCount = 6;
    state.laserWidthProfile =
        i2r_rlc::vision::CActiveContourStripeExtractor::estimateLaserWidth(
            image,
            options);
    state.activeCurve =
        i2r_rlc::vision::CActiveContourStripeExtractor::initializeCurve(
            image,
            options);
    return state;
}

void seedHistory(i2r_rlc::vision::ActiveContourDetectionState *state)
{
    for (int index = 0; index < 6; ++index)
    {
        i2r_rlc::vision::WeldFeaturePoint point;
        point.xPixel = 20.0 + static_cast<double>(index);
        point.yPixel = 30.0;
        point.confidence = 0.80;
        point.valid = true;
        point.label = "active_contour_seed_history";
        state->recentWeldPoints.push_back(point);
        state->lastWeldPoint = point;
    }
}

} // namespace

int main()
{
    using i2r_rlc::vision::ActiveContourDetectionState;
    using i2r_rlc::vision::ActiveContourSeamMode;
    using i2r_rlc::vision::CActiveContourStripeExtractor;
    using i2r_rlc::vision::CActiveContourWeldFeatureDetector;
    using i2r_rlc::vision::WeldSeamType;

    const auto options = makeOptions();

    const auto emptyWidth =
        CActiveContourStripeExtractor::estimateLaserWidth(
            cv::Mat(),
            options);
    if (!expectTrue(!emptyWidth.valid))
    {
        return 1;
    }

    const auto widthProfile =
        CActiveContourStripeExtractor::estimateLaserWidth(
            makeWidthImage(),
            options);
    if (!expectTrue(widthProfile.valid)
        || !expectTrue(widthProfile.widthByRowPixel.size() == 40U)
        || !expectTrue(widthProfile.safeWidthByRowPixel.size() == 40U)
        || !expectTrue(widthProfile.widthByRowPixel[20] >= 4)
        || !expectTrue(widthProfile.widthByRowPixel[20] <= 6)
        || !expectTrue(widthProfile.safeWidthByRowPixel[20]
                      >= widthProfile.widthByRowPixel[20]))
    {
        return 2;
    }

    const cv::Mat horizontalImage = makeHorizontalLineImage();
    const auto initialCurve =
        CActiveContourStripeExtractor::initializeCurve(
            horizontalImage,
            options);
    if (!expectTrue(initialCurve.valid)
        || !expectTrue(initialCurve.controlPoints.size() >= 5U)
        || !expectTrue(initialCurve.curvePoints.size()
                      > initialCurve.controlPoints.size()))
    {
        return 3;
    }

    auto legacyInitOptions = options;
    legacyInitOptions.initialClosingRadiusPixel = 4;
    const auto branchedInitialCurve =
        CActiveContourStripeExtractor::initializeCurve(
            makeBranchedAndFloatingInitialImage(),
            legacyInitOptions);
    double branchedRowSum = 0.0;
    for (const cv::Point2d &point : branchedInitialCurve.controlPoints)
    {
        branchedRowSum += point.y;
    }
    const double branchedMeanRow = branchedInitialCurve.controlPoints.empty()
        ? 0.0
        : branchedRowSum
            / static_cast<double>(branchedInitialCurve.controlPoints.size());
    if (!expectTrue(branchedInitialCurve.valid)
        || !expectTrue(branchedInitialCurve.controlPoints.size() >= 8U)
        || !expectNear(branchedMeanRow, 35.0, 4.0))
    {
        std::printf(
            "branched initial curve failed: valid=%d points=%zu meanRow=%.3f message=%s\n",
            branchedInitialCurve.valid ? 1 : 0,
            branchedInitialCurve.controlPoints.size(),
            branchedMeanRow,
            branchedInitialCurve.message.c_str());
        return 14;
    }

    auto offsetCurve = initialCurve;
    for (cv::Point2d &point : offsetCurve.controlPoints)
    {
        point.y -= 2.0;
    }
    const auto horizontalWidthProfile =
        CActiveContourStripeExtractor::estimateLaserWidth(
            horizontalImage,
            options);
    const auto iteratedCurve =
        CActiveContourStripeExtractor::iterateCurve(
            horizontalImage,
            horizontalWidthProfile,
            offsetCurve,
            options);
    if (!expectTrue(iteratedCurve.valid)
        || !expectNear(iteratedCurve.controlPoints.front().y, 25.0, 2.0))
    {
        return 4;
    }

    auto handleOptions = options;
    handleOptions.controlPointDragSearchHalfHeightPixel = 12;
    handleOptions.controlPointGrowSampleStepPixel = 5;
    handleOptions.controlPointMaxGapPixel = 30;
    const cv::Mat longHorizontalImage = makeLongHorizontalLineImage();
    const auto longWidthProfile =
        CActiveContourStripeExtractor::estimateLaserWidth(
            longHorizontalImage,
            handleOptions);
    i2r_rlc::vision::ActiveContourCurve sparseOfflineCurve;
    sparseOfflineCurve.valid = true;
    sparseOfflineCurve.controlPoints.push_back(cv::Point2d(20.0, 18.0));
    sparseOfflineCurve.controlPoints.push_back(cv::Point2d(50.0, 18.0));
    sparseOfflineCurve.controlPoints.push_back(cv::Point2d(80.0, 18.0));
    sparseOfflineCurve.curvePoints = sparseOfflineCurve.controlPoints;
    const auto handledCurve = CActiveContourStripeExtractor::iterateCurve(
        longHorizontalImage,
        longWidthProfile,
        sparseOfflineCurve,
        handleOptions);
    if (!expectTrue(handledCurve.valid)
        || !expectTrue(
            handledCurve.controlPoints.size()
            > sparseOfflineCurve.controlPoints.size())
        || !expectTrue(handledCurve.controlPoints.front().x < 20.0)
        || !expectTrue(handledCurve.controlPoints.back().x > 80.0)
        || !expectNear(handledCurve.controlPoints.front().y, 25.0, 3.0))
    {
        return 13;
    }

    ActiveContourDetectionState tState;
    const auto tResult = CActiveContourWeldFeatureDetector::detect(
        makeTJointImage(),
        ActiveContourSeamMode::TJoint,
        options,
        &tState);
    if (!expectTrue(tResult.valid)
        || !expectTrue(tResult.seamType == WeldSeamType::TJoint)
        || !expectTrue(tResult.weldPoint.valid)
        || !expectNear(tResult.weldPoint.xPixel, 45.0, 5.0)
        || !expectNear(tResult.weldPoint.yPixel, 45.0, 5.0)
        || !expectTrue(tResult.slopeParameters.size() == 4U))
    {
        return 5;
    }

    auto dualTOptions = options;
    dualTOptions.firstFrameRoiHalfWidthPixel = 28;
    dualTOptions.tJointRoiHalfWidthPixel = 28;
    ActiveContourDetectionState dualTState;
    dualTState.lastWeldPoint.valid = true;
    dualTState.lastWeldPoint.xPixel = 105.0;
    dualTState.lastWeldPoint.yPixel = 58.0;
    const auto dualTResult = CActiveContourWeldFeatureDetector::detect(
        makeDualTJointImage(),
        ActiveContourSeamMode::TJoint,
        dualTOptions,
        &dualTState);
    if (!expectTrue(dualTResult.valid)
        || !expectNear(dualTResult.weldPoint.xPixel, 108.0, 7.0)
        || !expectNear(dualTResult.weldPoint.yPixel, 58.0, 7.0))
    {
        std::printf(
            "dual T result failed: valid=%d weld=(%.3f, %.3f) message=%s\n",
            dualTResult.valid ? 1 : 0,
            dualTResult.weldPoint.xPixel,
            dualTResult.weldPoint.yPixel,
            dualTResult.message.c_str());
        return 15;
    }

    ActiveContourDetectionState buttState;
    const auto buttResult = CActiveContourWeldFeatureDetector::detect(
        makeButtImage(),
        ActiveContourSeamMode::Butt,
        options,
        &buttState);
    if (!expectTrue(buttResult.valid)
        || !expectTrue(buttResult.seamType == WeldSeamType::Butt)
        || !expectTrue(buttResult.leftGapPoint.valid)
        || !expectTrue(buttResult.rightGapPoint.valid)
        || !expectNear(buttResult.weldPoint.xPixel, 40.0, 8.0)
        || !expectNear(buttResult.weldPoint.yPixel, 30.0, 4.0))
    {
        return 6;
    }

    auto anchoredButtOptions = options;
    anchoredButtOptions.firstFrameRoiHalfWidthPixel = 65;
    anchoredButtOptions.buttRoiHalfWidthPixel = 65;
    anchoredButtOptions.buttSplitDistancePixel = 2.0;
    anchoredButtOptions.buttRansacDistancePixel = 2.0;
    anchoredButtOptions.buttGapEndpointMarginPixel = 35;
    anchoredButtOptions.buttGapScanPaddingPixel = 10;
    ActiveContourDetectionState anchoredButtState;
    const auto anchoredButtResult = CActiveContourWeldFeatureDetector::detect(
        makeAnchoredButtImage(),
        ActiveContourSeamMode::Butt,
        anchoredButtOptions,
        &anchoredButtState);
    if (!expectTrue(anchoredButtResult.valid)
        || !expectTrue(anchoredButtResult.leftGapPoint.valid)
        || !expectTrue(anchoredButtResult.rightGapPoint.valid)
        || !expectTrue(anchoredButtResult.fittedLines.size() >= 3U)
        || !expectNear(anchoredButtResult.leftGapPoint.xPixel, 45.0, 8.0)
        || !expectNear(anchoredButtResult.rightGapPoint.xPixel, 95.0, 8.0))
    {
        return 16;
    }

    ActiveContourDetectionState lapState;
    const auto lapResult = CActiveContourWeldFeatureDetector::detect(
        makeLapImage(),
        ActiveContourSeamMode::Lap,
        options,
        &lapState);
    if (!expectTrue(lapResult.valid)
        || !expectTrue(lapResult.seamType == WeldSeamType::Lap)
        || !expectNear(lapResult.weldPoint.xPixel, 42.0, 6.0)
        || !expectNear(lapResult.weldPoint.yPixel, 55.0, 6.0)
        || !expectTrue(lapResult.slopeParameters.size() == 2U))
    {
        return 7;
    }

    cv::Mat brightButt = makeButtImage();
    cv::line(brightButt, cv::Point(35, 30), cv::Point(45, 30), cv::Scalar(255), 3);
    auto fallbackState = makeInitializedState(makeButtImage(), options);
    seedHistory(&fallbackState);
    const auto fallbackResult = CActiveContourWeldFeatureDetector::detect(
        brightButt,
        ActiveContourSeamMode::Butt,
        options,
        &fallbackState);
    if (!expectTrue(fallbackResult.valid)
        || !expectTrue(fallbackResult.usedHistoryFallback)
        || !expectNear(fallbackResult.weldPoint.xPixel, 26.0, 1.5))
    {
        return 8;
    }

    std::printf("PASS: vision active contour feature tests\n");
    return 0;
}
