/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_groove_edge_detector.cpp
 * @brief       vision groove edge detector 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include "feature/GrooveEdgeDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

i2r_rlc::vision::WeldFeatureDetectionOptions makeOptions(void)
{
    i2r_rlc::vision::WeldFeatureDetectionOptions options;
    options.searchHalfWindowY = 1;
    options.minEdgeContrast = 80;
    options.minGapWidthPixel = 8;
    options.maxGapWidthPixel = 70;
    options.minConfidence = 0.10;
    options.edgeSearchHalfWidthPixel = 60;
    return options;
}

i2r_rlc::vision::LaserCenterlineResult makeCenterline(
    int beginY,
    int endY,
    double centerXPixel)
{
    i2r_rlc::vision::LaserCenterlineResult centerline;
    centerline.valid = true;
    for (int yPixel = beginY; yPixel <= endY; yPixel += 4)
    {
        i2r_rlc::vision::LaserCenterlinePoint point;
        point.xPixel = centerXPixel;
        point.yPixel = static_cast<double>(yPixel);
        point.response = 220.0;
        point.localContrast = 180.0;
        point.confidence = 0.90;
        point.valid = true;
        centerline.points.push_back(point);
    }
    centerline.meanConfidence = 0.90;
    centerline.validColumnRatio = 1.0;
    return centerline;
}

cv::Mat makeVGrooveImage(void)
{
    cv::Mat image(70, 90, CV_8UC1, cv::Scalar(40));
    for (int yPixel = 12; yPixel <= 56; ++yPixel)
    {
        const double dy = static_cast<double>(yPixel - 12);
        const int leftX = static_cast<int>(20.0 + 0.25 * dy);
        const int rightX = static_cast<int>(70.0 - 0.25 * dy);
        for (int xPixel = 0; xPixel < image.cols; ++xPixel)
        {
            if (xPixel < leftX || xPixel > rightX)
            {
                image.at<unsigned char>(yPixel, xPixel) = 220;
            }
        }
    }
    return image;
}

cv::Mat makeParallelGapImage(void)
{
    cv::Mat image(70, 90, CV_8UC1, cv::Scalar(40));
    for (int yPixel = 12; yPixel <= 56; ++yPixel)
    {
        for (int xPixel = 0; xPixel < image.cols; ++xPixel)
        {
            if (xPixel < 25 || xPixel > 58)
            {
                image.at<unsigned char>(yPixel, xPixel) = 220;
            }
        }
    }
    return image;
}

cv::Mat makeTooNarrowGrooveImage(void)
{
    cv::Mat image(50, 70, CV_8UC1, cv::Scalar(40));
    for (int yPixel = 12; yPixel <= 36; ++yPixel)
    {
        for (int xPixel = 0; xPixel < image.cols; ++xPixel)
        {
            if (xPixel < 31 || xPixel > 34)
            {
                image.at<unsigned char>(yPixel, xPixel) = 220;
            }
        }
    }
    return image;
}

cv::Mat makeSingleSideImage(void)
{
    cv::Mat image(50, 70, CV_8UC1, cv::Scalar(40));
    for (int yPixel = 12; yPixel <= 36; ++yPixel)
    {
        for (int xPixel = 0; xPixel < 25; ++xPixel)
        {
            image.at<unsigned char>(yPixel, xPixel) = 220;
        }
    }
    return image;
}

cv::Mat makeNoisyVGrooveImage(void)
{
    cv::Mat image = makeVGrooveImage();
    for (int yPixel = 18; yPixel <= 48; yPixel += 6)
    {
        image.at<unsigned char>(yPixel, 45) = 255;
        image.at<unsigned char>(yPixel, 46) = 255;
    }
    return image;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CGrooveEdgeDetector;

    const auto options = makeOptions();

    const auto emptyResult = CGrooveEdgeDetector::detect(
        cv::Mat(),
        makeCenterline(12, 56, 45.0),
        options);
    if (!expectTrue(!emptyResult.valid)
        || !expectTrue(emptyResult.message == "empty input"))
    {
        return 1;
    }

    const auto colorResult = CGrooveEdgeDetector::detect(
        cv::Mat::zeros(30, 40, CV_8UC3),
        makeCenterline(12, 24, 20.0),
        options);
    if (!expectTrue(!colorResult.valid)
        || !expectTrue(colorResult.message == "input must be CV_8UC1"))
    {
        return 2;
    }

    i2r_rlc::vision::LaserCenterlineResult emptyCenterline;
    const auto noCenterlineResult = CGrooveEdgeDetector::detect(
        makeVGrooveImage(),
        emptyCenterline,
        options);
    if (!expectTrue(!noCenterlineResult.valid)
        || !expectTrue(noCenterlineResult.message == "empty centerline"))
    {
        return 3;
    }

    const auto vGrooveResult = CGrooveEdgeDetector::detect(
        makeVGrooveImage(),
        makeCenterline(12, 56, 45.0),
        options);
    if (!expectTrue(vGrooveResult.valid)
        || !expectTrue(vGrooveResult.leftEdge.xPixel
                       < vGrooveResult.rightEdge.xPixel)
        || !expectTrue(vGrooveResult.profileLabel == "v_groove")
        || !expectTrue(vGrooveResult.edgePairCount >= 6)
        || !expectTrue(vGrooveResult.lineAngleDegree >= 12.0)
        || !expectNear(vGrooveResult.centerPoint.xPixel, 45.0, 2.0)
        || !expectTrue(vGrooveResult.confidence >= 0.0)
        || !expectTrue(vGrooveResult.confidence <= 1.0))
    {
        return 4;
    }

    const auto parallelResult = CGrooveEdgeDetector::detect(
        makeParallelGapImage(),
        makeCenterline(12, 56, 41.5),
        options);
    if (!expectTrue(parallelResult.valid)
        || !expectTrue(parallelResult.profileLabel == "parallel_gap")
        || !expectTrue(parallelResult.confidence
                       < vGrooveResult.confidence)
        || !expectTrue(parallelResult.message == "parallel gap edge detected"))
    {
        return 5;
    }

    const auto narrowResult = CGrooveEdgeDetector::detect(
        makeTooNarrowGrooveImage(),
        makeCenterline(12, 36, 32.5),
        options);
    if (!expectTrue(!narrowResult.valid))
    {
        return 6;
    }

    const auto singleSideResult = CGrooveEdgeDetector::detect(
        makeSingleSideImage(),
        makeCenterline(12, 36, 35.0),
        options);
    if (!expectTrue(!singleSideResult.valid)
        || !expectTrue(singleSideResult.message
                       == "not enough groove edge pairs"))
    {
        return 7;
    }

    const auto noisyResult = CGrooveEdgeDetector::detect(
        makeNoisyVGrooveImage(),
        makeCenterline(12, 56, 45.0),
        options);
    if (!expectTrue(noisyResult.valid)
        || !expectTrue(noisyResult.profileLabel == "v_groove")
        || !expectNear(noisyResult.centerPoint.xPixel, 45.0, 3.0))
    {
        return 8;
    }

    std::printf("PASS: vision groove edge detector tests\n");
    return 0;
}
