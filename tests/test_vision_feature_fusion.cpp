/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_feature_fusion.cpp
 * @brief       vision feature fusion 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "feature/WeldFeatureDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool containsText(const std::string &text, const std::string &pattern)
{
    return text.find(pattern) != std::string::npos;
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

i2r_rlc::vision::LaserCenterlinePoint makeCenterlinePoint(
    double xPixel,
    double yPixel)
{
    i2r_rlc::vision::LaserCenterlinePoint point;
    point.xPixel = xPixel;
    point.yPixel = yPixel;
    point.response = 220.0;
    point.localContrast = 180.0;
    point.confidence = 0.95;
    point.valid = true;
    return point;
}

i2r_rlc::vision::LaserCenterlineResult makeCenterline(
    const std::vector<cv::Point2d> &points)
{
    i2r_rlc::vision::LaserCenterlineResult centerline;
    centerline.valid = !points.empty();
    centerline.meanConfidence = 0.95;
    centerline.validColumnRatio = points.empty() ? 0.0 : 1.0;
    for (const cv::Point2d &point : points)
    {
        centerline.points.push_back(makeCenterlinePoint(point.x, point.y));
    }
    return centerline;
}

i2r_rlc::vision::LaserCenterlineResult makeVerticalCenterline(
    int beginY,
    int endY,
    double centerXPixel)
{
    std::vector<cv::Point2d> points;
    for (int yPixel = beginY; yPixel <= endY; yPixel += 4)
    {
        points.push_back(cv::Point2d(
            centerXPixel,
            static_cast<double>(yPixel)));
    }
    return makeCenterline(points);
}

cv::Mat makeBlankImage(void)
{
    return cv::Mat(80, 80, CV_8UC1, cv::Scalar(40));
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

} // namespace

int main()
{
    using i2r_rlc::vision::CWeldFeatureDetector;

    const auto options = makeOptions();

    const auto buttResult = CWeldFeatureDetector::detect(
        makeParallelGapImage(),
        makeVerticalCenterline(12, 56, 41.5),
        options);
    if (!expectTrue(buttResult.valid)
        || !expectTrue(buttResult.gapFeature.valid)
        || !expectTrue(buttResult.grooveFeature.valid)
        || !expectTrue(buttResult.grooveFeature.profileLabel
                      == "parallel_gap")
        || !expectTrue(containsText(buttResult.message, "groove")))
    {
        return 1;
    }

    const auto endpointOnlyResult = CWeldFeatureDetector::detect(
        makeBlankImage(),
        makeVerticalCenterline(12, 56, 40.0),
        options);
    if (!expectTrue(endpointOnlyResult.valid)
        || !expectTrue(endpointOnlyResult.endpointResult.valid)
        || !expectTrue(!endpointOnlyResult.grooveFeature.valid))
    {
        return 2;
    }

    const auto grooveResult = CWeldFeatureDetector::detect(
        makeVGrooveImage(),
        makeVerticalCenterline(12, 56, 45.0),
        options);
    if (!expectTrue(grooveResult.valid)
        || !expectTrue(grooveResult.grooveFeature.valid)
        || !expectTrue(grooveResult.grooveFeature.profileLabel == "v_groove")
        || !expectTrue(containsText(grooveResult.message, "groove")))
    {
        return 3;
    }

    i2r_rlc::vision::LaserCenterlineResult emptyCenterline;
    const auto invalidResult = CWeldFeatureDetector::detect(
        makeBlankImage(),
        emptyCenterline,
        options);
    if (!expectTrue(!invalidResult.valid))
    {
        return 4;
    }

    std::printf("PASS: vision feature fusion tests\n");
    return 0;
}
