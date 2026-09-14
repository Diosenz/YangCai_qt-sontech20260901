/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_gap_feature_detector.cpp
 * @brief       vision gap feature detector 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include "feature/GapFeatureDetector.h"
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

cv::Mat makeGapImage(void)
{
    cv::Mat image(50, 70, CV_8UC1, cv::Scalar(40));
    for (int yPixel = 18; yPixel <= 32; ++yPixel)
    {
        for (int xPixel = 0; xPixel < image.cols; ++xPixel)
        {
            if (xPixel < 20 || xPixel > 40)
            {
                image.at<unsigned char>(yPixel, xPixel) = 220;
            }
        }
    }
    return image;
}

cv::Mat makeGapImageWithCorruptedCenterRow(void)
{
    cv::Mat image = makeGapImage();
    for (int xPixel = 0; xPixel < image.cols; ++xPixel)
    {
        image.at<unsigned char>(25, xPixel) = 40;
    }
    return image;
}

cv::Mat makeGapImageWithDistractorEdges(void)
{
    cv::Mat image = makeGapImage();
    for (int yPixel = 18; yPixel <= 32; ++yPixel)
    {
        image.at<unsigned char>(yPixel, 8) = 255;
        image.at<unsigned char>(yPixel, 9) = 30;
        image.at<unsigned char>(yPixel, 58) = 30;
        image.at<unsigned char>(yPixel, 59) = 255;
    }
    return image;
}

cv::Mat makeLowContrastGapImage(void)
{
    cv::Mat image(50, 70, CV_8UC1, cv::Scalar(90));
    for (int yPixel = 18; yPixel <= 32; ++yPixel)
    {
        for (int xPixel = 20; xPixel <= 40; ++xPixel)
        {
            image.at<unsigned char>(yPixel, xPixel) = 70;
        }
    }
    return image;
}

i2r_rlc::vision::WeldFeatureDetectionOptions makeOptions(void)
{
    i2r_rlc::vision::WeldFeatureDetectionOptions options;
    options.searchHalfWindowY = 5;
    options.minEdgeContrast = 80;
    options.minGapWidthPixel = 12;
    options.maxGapWidthPixel = 30;
    options.minConfidence = 0.10;
    options.useCenterlinePrior = true;
    options.edgeSearchHalfWidthPixel = 40;
    return options;
}

i2r_rlc::vision::LaserCenterlineResult makeCenterline(
    double xPixel,
    double yPixel)
{
    i2r_rlc::vision::LaserCenterlineResult centerline;
    centerline.valid = true;

    i2r_rlc::vision::LaserCenterlinePoint point;
    point.xPixel = xPixel;
    point.yPixel = yPixel;
    point.response = 220.0;
    point.localContrast = 180.0;
    point.confidence = 0.90;
    point.valid = true;
    centerline.points.push_back(point);
    centerline.meanConfidence = point.confidence;
    centerline.validColumnRatio = 1.0;
    return centerline;
}

int countValidPoints(
    const i2r_rlc::vision::GapFeatureResult &result)
{
    int count = 0;
    if (result.weldPoint.valid)
    {
        ++count;
    }
    if (result.edgePair.left.valid)
    {
        ++count;
    }
    if (result.edgePair.right.valid)
    {
        ++count;
    }
    return count;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CGapFeatureDetector;
    using i2r_rlc::vision::CWeldFeatureDetector;

    const auto options = makeOptions();

    const auto emptyResult = CGapFeatureDetector::detectGapFromCenterline(
        cv::Mat(),
        makeCenterline(30.0, 25.0),
        options);
    if (!expectTrue(!emptyResult.valid)
        || !expectTrue(emptyResult.message == "empty input"))
    {
        return 1;
    }

    const cv::Mat colorImage = cv::Mat::zeros(20, 30, CV_8UC3);
    const auto colorResult = CGapFeatureDetector::detectGapFromCenterline(
        colorImage,
        makeCenterline(15.0, 10.0),
        options);
    if (!expectTrue(!colorResult.valid)
        || !expectTrue(colorResult.message == "input must be CV_8UC1"))
    {
        return 2;
    }

    i2r_rlc::vision::LaserCenterlineResult emptyCenterline;
    const auto noCenterlineResult =
        CGapFeatureDetector::detectGapFromCenterline(
            makeGapImage(),
            emptyCenterline,
            options);
    if (!expectTrue(!noCenterlineResult.valid)
        || !expectTrue(noCenterlineResult.message == "empty centerline"))
    {
        return 3;
    }

    const cv::Mat gapImage = makeGapImage();
    const auto edgePair = CGapFeatureDetector::detectEdgesOnRow(
        gapImage,
        30.0,
        25.0,
        options);
    if (!expectTrue(edgePair.valid)
        || !expectTrue(edgePair.left.valid)
        || !expectTrue(edgePair.right.valid)
        || !expectTrue(edgePair.left.xPixel < edgePair.right.xPixel)
        || !expectTrue(edgePair.confidence > 0.0))
    {
        return 4;
    }

    if (!expectNear(edgePair.widthPixel, 21.0, 3.0))
    {
        return 5;
    }

    const auto gapResult = CGapFeatureDetector::detectGapFromCenterline(
        gapImage,
        makeCenterline(30.0, 25.0),
        options);
    if (!expectTrue(gapResult.valid)
        || !expectTrue(gapResult.gapWidthPixel >= 18.0)
        || !expectTrue(gapResult.gapWidthPixel <= 24.0)
        || !expectTrue(gapResult.confidence >= 0.0)
        || !expectTrue(gapResult.confidence <= 1.0)
        || !expectTrue(countValidPoints(gapResult) == 3)
        || !expectTrue(gapResult.leftCandidateCount > 0)
        || !expectTrue(gapResult.rightCandidateCount > 0)
        || !expectTrue(gapResult.selectedRowPixel >= 0)
        || !expectTrue(gapResult.edgeContrastGrayValue >= 80.0)
        || !expectTrue(gapResult.failureReason.empty()))
    {
        return 6;
    }

    const double midpoint =
        0.5 * (gapResult.edgePair.left.xPixel
               + gapResult.edgePair.right.xPixel);
    if (!expectNear(gapResult.weldPoint.xPixel, midpoint, 0.001)
        || !expectNear(gapResult.weldPoint.yPixel,
                       gapResult.edgePair.left.yPixel,
                       0.001))
    {
        return 7;
    }

    i2r_rlc::vision::WeldFeatureDetectionOptions narrowOptions = options;
    narrowOptions.minGapWidthPixel = 24;
    narrowOptions.maxGapWidthPixel = 30;
    const auto narrowResult = CGapFeatureDetector::detectGapFromCenterline(
        gapImage,
        makeCenterline(30.0, 25.0),
        narrowOptions);
    if (!expectTrue(!narrowResult.valid))
    {
        return 8;
    }
    if (!expectTrue(narrowResult.failureReason == "gap width is out of range"))
    {
        return 13;
    }

    i2r_rlc::vision::WeldFeatureDetectionOptions wideOptions = options;
    wideOptions.minGapWidthPixel = 4;
    wideOptions.maxGapWidthPixel = 12;
    const auto wideResult = CGapFeatureDetector::detectGapFromCenterline(
        gapImage,
        makeCenterline(30.0, 25.0),
        wideOptions);
    if (!expectTrue(!wideResult.valid))
    {
        return 9;
    }
    if (!expectTrue(wideResult.failureReason == "gap width is out of range"))
    {
        return 14;
    }

    const auto shiftedCenterlineResult =
        CGapFeatureDetector::detectGapFromCenterline(
            gapImage,
            makeCenterline(30.0, 21.0),
            options);
    if (!expectTrue(shiftedCenterlineResult.valid)
        || !expectTrue(shiftedCenterlineResult.gapWidthPixel >= 18.0)
        || !expectTrue(shiftedCenterlineResult.gapWidthPixel <= 24.0))
    {
        return 10;
    }

    const auto contrast = CGapFeatureDetector::calculateEdgeContrast(
        gapImage,
        static_cast<int>(edgePair.left.xPixel),
        static_cast<int>(edgePair.left.yPixel),
        1);
    if (!expectTrue(contrast >= 80.0))
    {
        return 11;
    }

    const auto weldFeature = CWeldFeatureDetector::detect(
        gapImage,
        makeCenterline(30.0, 25.0),
        options);
    if (!expectTrue(weldFeature.valid)
        || !expectTrue(weldFeature.gapFeature.valid))
    {
        return 12;
    }

    const auto corruptedCenterRowResult =
        CGapFeatureDetector::detectGapFromCenterline(
            makeGapImageWithCorruptedCenterRow(),
            makeCenterline(30.0, 25.0),
            options);
    if (!expectTrue(corruptedCenterRowResult.valid)
        || !expectTrue(corruptedCenterRowResult.selectedRowPixel != 25)
        || !expectTrue(corruptedCenterRowResult.gapWidthPixel >= 18.0)
        || !expectTrue(corruptedCenterRowResult.gapWidthPixel <= 24.0))
    {
        return 15;
    }

    const auto distractorResult = CGapFeatureDetector::detectGapFromCenterline(
        makeGapImageWithDistractorEdges(),
        makeCenterline(30.0, 25.0),
        options);
    if (!expectTrue(distractorResult.valid)
        || !expectTrue(distractorResult.edgePair.left.xPixel >= 18.0)
        || !expectTrue(distractorResult.edgePair.left.xPixel <= 22.0)
        || !expectTrue(distractorResult.edgePair.right.xPixel >= 39.0)
        || !expectTrue(distractorResult.edgePair.right.xPixel <= 42.0))
    {
        return 16;
    }

    std::vector<i2r_rlc::vision::GapEdgeCandidate> leftCandidates;
    std::vector<i2r_rlc::vision::GapEdgeCandidate> rightCandidates;
    CGapFeatureDetector::collectEdgeCandidates(
        gapImage,
        30.0,
        25.0,
        options,
        &leftCandidates,
        &rightCandidates);
    if (!expectTrue(!leftCandidates.empty())
        || !expectTrue(!rightCandidates.empty()))
    {
        return 17;
    }

    const auto lowContrastResult = CGapFeatureDetector::detectGapFromCenterline(
        makeLowContrastGapImage(),
        makeCenterline(30.0, 25.0),
        options);
    if (!expectTrue(!lowContrastResult.valid)
        || !expectTrue(lowContrastResult.failureReason
                       == "edge candidate not found"))
    {
        return 18;
    }

    std::printf("PASS: vision gap feature detector tests\n");
    return 0;
}
