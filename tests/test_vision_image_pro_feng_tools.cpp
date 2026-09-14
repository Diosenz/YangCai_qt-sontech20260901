/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_pro_feng_tools.cpp
 * @brief       vision image pro feng tools 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "feature/SimulatedLaserWeldDetector.h"
#include "preprocess/DirectionalMorphologyFilter.h"
#include "stripe/WeldSkeletonFeatureTools.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

int fail(int code)
{
    std::printf("FAIL: vision image pro feng tools tests code=%d\n", code);
    return code;
}

using DigitalTwinImagePoint2D_px =
    i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px;
using DigitalTwinVisionLaserObservation =
    i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation;

DigitalTwinImagePoint2D_px makeImagePoint(double xPixel, double yPixel)
{
    DigitalTwinImagePoint2D_px point;
    point.x_px = xPixel;
    point.y_px = yPixel;
    return point;
}

std::vector<cv::Point2d> makeSimulatedLaser(void)
{
    std::vector<cv::Point2d> points;
    points.push_back(cv::Point2d(20.0, 30.0));
    points.push_back(cv::Point2d(35.0, 38.0));
    points.push_back(cv::Point2d(50.0, 48.0));
    points.push_back(cv::Point2d(65.0, 42.0));
    points.push_back(cv::Point2d(80.0, 36.0));
    points.push_back(cv::Point2d(95.0, 30.0));
    return points;
}

std::vector<DigitalTwinImagePoint2D_px> makeSimulatedObservationLaser(void)
{
    const std::vector<cv::Point2d> points = makeSimulatedLaser();
    std::vector<DigitalTwinImagePoint2D_px> output;
    output.reserve(points.size());
    for (const cv::Point2d &point : points)
    {
        output.push_back(makeImagePoint(point.x, point.y));
    }
    return output;
}

cv::Mat makeLaserImage(const std::vector<cv::Point2d> &points, double yOffset)
{
    cv::Mat image(90, 120, CV_8UC1, cv::Scalar(0));
    for (int index = 1; index < static_cast<int>(points.size()); ++index)
    {
        const cv::Point start(
            static_cast<int>(std::round(points[index - 1].x)),
            static_cast<int>(std::round(points[index - 1].y + yOffset)));
        const cv::Point end(
            static_cast<int>(std::round(points[index].x)),
            static_cast<int>(std::round(points[index].y + yOffset)));
        cv::line(image, start, end, cv::Scalar(255), 5, cv::LINE_8);
    }
    return image;
}

cv::Mat makeLaserImage(
    const cv::Size &size,
    const std::vector<cv::Point2d> &points)
{
    cv::Mat image(size, CV_8UC1, cv::Scalar(0));
    for (int index = 1; index < static_cast<int>(points.size()); ++index)
    {
        cv::line(
            image,
            cv::Point(
                static_cast<int>(std::round(points[index - 1].x)),
                static_cast<int>(std::round(points[index - 1].y))),
            cv::Point(
                static_cast<int>(std::round(points[index].x)),
                static_cast<int>(std::round(points[index].y))),
            cv::Scalar(255),
            5,
            cv::LINE_8);
    }
    return image;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CDirectionalMorphologyFilter;
    using i2r_rlc::vision::DirectionalMorphologyOptions;
    using i2r_rlc::vision::SimulatedLaserMatchingOptions;
    using i2r_rlc::vision::CSimulatedLaserRegistrationMatcher;
    using i2r_rlc::vision::CSimulatedLaserReplayParser;
    using i2r_rlc::vision::CSimulatedLaserTemplateMatcher;
    using i2r_rlc::vision::LaserSkeletonExtractionOptions;
    using i2r_rlc::vision::CWeldSkeletonFeatureTools;

    const auto parseResult = CSimulatedLaserReplayParser::parse(
        "0 10 10 20 20 10\n1 11 11 21 21 11\n",
        "10 20\n11 21\n");
    if (!expectTrue(parseResult.valid)
        || !expectTrue(parseResult.observations.size() == 2U)
        || !expectTrue(parseResult.observations[0].laserLineImagePoints_px.size() == 3U)
        || !expectTrue(parseResult.observations[1].hasWeldPointImage_px))
    {
        return fail(1);
    }

    const auto blockParseResult = CSimulatedLaserReplayParser::parse(
        "$ 7\n"
        "RobotPose: 1 2 3 4 5 6\n"
        "WeldImgPnt: 50 48\n"
        "OriLaserImgPnts: 20 30 35 38 50 48 65 42 80 36 95 30\n"
        "#\n",
        "");
    if (!expectTrue(blockParseResult.valid)
        || !expectTrue(blockParseResult.observations.size() == 1U)
        || !expectTrue(blockParseResult.observations[0].frameId == 7)
        || !expectTrue(blockParseResult.observations[0].laserLineImagePoints_px.size()
            == 6U)
        || !expectTrue(blockParseResult.observations[0].hasWeldPointImage_px)
        || !expectTrue(blockParseResult.observations[0].valid))
    {
        return fail(2);
    }

    const cv::Mat kernel =
        CDirectionalMorphologyFilter::createLineKernel(45.0, 15);
    if (!expectTrue(!kernel.empty())
        || !expectTrue(kernel.type() == CV_8UC1)
        || !expectTrue(cv::countNonZero(kernel) > 0))
    {
        return fail(3);
    }

    const std::vector<cv::Point2d> laserPoints = makeSimulatedLaser();
    DirectionalMorphologyOptions morphologyOptions;
    morphologyOptions.leftLengthPixel = 15;
    morphologyOptions.rightLengthPixel = 15;
    const auto morphologyResult =
        CDirectionalMorphologyFilter::enhance(
            makeLaserImage(laserPoints, 0.0),
            laserPoints,
            morphologyOptions);
    if (!expectTrue(morphologyResult.valid)
        || !expectTrue(morphologyResult.enhancedImage.type() == CV_8UC1)
        || !expectTrue(cv::countNonZero(morphologyResult.enhancedImage) > 0)
        || !expectTrue(morphologyResult.splitPointIndex > 0))
    {
        return fail(4);
    }

    LaserSkeletonExtractionOptions skeletonOptions;
    skeletonOptions.thresholdValue = 80;
    skeletonOptions.minCenterPointCount = 5;
    skeletonOptions.enableOrderedPathExtraction = true;
    const auto skeletonResult =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(
            makeLaserImage(laserPoints, 3.0),
            skeletonOptions);
    if (!expectTrue(skeletonResult.valid)
        || !expectTrue(skeletonResult.skeletonPoints.size() >= 20U)
        || !expectTrue(skeletonResult.refinedCenterPoints.size() >= 20U)
        || !expectTrue(skeletonResult.orderedPathPoints.size() >= 20U))
    {
        return fail(5);
    }

    cv::Mat twoBranchImage(100, 130, CV_8UC1, cv::Scalar(0));
    cv::line(twoBranchImage, cv::Point(10, 22), cv::Point(115, 42),
        cv::Scalar(255), 3, cv::LINE_8);
    cv::line(twoBranchImage, cv::Point(10, 78), cv::Point(115, 58),
        cv::Scalar(255), 3, cv::LINE_8);
    LaserSkeletonExtractionOptions twoBranchOptions;
    twoBranchOptions.thresholdValue = 80;
    twoBranchOptions.minCenterPointCount = 5;
    twoBranchOptions.enableLogSupportedComponentFilter = false;
    twoBranchOptions.enableOrderedPathExtraction = true;
    const auto twoBranchResult =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(
            twoBranchImage,
            twoBranchOptions);
    double orderedRowSum = 0.0;
    for (const cv::Point2d &point : twoBranchResult.orderedPathPoints)
    {
        orderedRowSum += point.y;
    }
    const double orderedMeanRow = orderedRowSum
        / static_cast<double>(std::max<std::size_t>(
            1U,
            twoBranchResult.orderedPathPoints.size()));
    if (!expectTrue(twoBranchResult.valid)
        || !expectTrue(twoBranchResult.orderedPathPoints.size() >= 80U)
        || !expectTrue(std::abs(orderedMeanRow - 50.0) > 8.0))
    {
        std::printf(
            "ordered path points=%zu mean=%f\n",
            twoBranchResult.orderedPathPoints.size(),
            orderedMeanRow);
        return fail(55);
    }

    LaserSkeletonExtractionOptions houghOptions;
    houghOptions.thresholdValue = 80;
    houghOptions.minComponentArea = 0;
    houghOptions.minCenterPointCount = 5;
    houghOptions.enableHoughFallback = true;
    houghOptions.houghThreshold = 5;
    houghOptions.houghMinLineLengthPixel = 8.0;
    houghOptions.houghMaxLineGapPixel = 4.0;
    cv::Mat houghImage(90, 120, CV_8UC1, cv::Scalar(0));
    cv::line(houghImage, cv::Point(15, 25), cv::Point(50, 45),
        cv::Scalar(255), 1, cv::LINE_8);
    cv::line(houghImage, cv::Point(62, 45), cv::Point(100, 25),
        cv::Scalar(255), 1, cv::LINE_8);
    const auto houghResult =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(
            houghImage,
            houghOptions);
    if (!expectTrue(houghResult.valid)
        || !expectTrue(houghResult.message.find("hough fallback")
            != std::string::npos)
        || !expectTrue(houghResult.skeletonPoints.size() >= 20U)
        || !expectTrue(houghResult.refinedCenterPoints.size() >= 20U))
    {
        std::printf("Hough result valid=%d points=%zu message=%s\n",
            houghResult.valid ? 1 : 0,
            houghResult.skeletonPoints.size(),
            houghResult.message.c_str());
        return fail(6);
    }

    DigitalTwinVisionLaserObservation observation;
    observation.frameId = 0;
    observation.laserLineImagePoints_px = makeSimulatedObservationLaser();
    observation.weldPointImage_px = makeImagePoint(50.0, 48.0);
    observation.hasWeldPointImage_px = true;
    observation.valid = true;

    SimulatedLaserMatchingOptions matchOptions;
    matchOptions.morphologyOptions.leftLengthPixel = 15;
    matchOptions.morphologyOptions.rightLengthPixel = 15;
    matchOptions.skeletonThresholdValue = 25;
    matchOptions.logSearchMinGrayValue = 80;
    matchOptions.minConfidence = 0.05;
    const auto matchResult = CSimulatedLaserRegistrationMatcher::match(
        makeLaserImage(laserPoints, 3.0),
        observation,
        matchOptions);
    if (!expectTrue(matchResult.valid)
        || !expectTrue(!matchResult.matchedLaserPoints.empty())
        || !expectTrue(matchResult.coarseValid)
        || !expectTrue(!matchResult.coarseMatchedLaserPoints.empty())
        || !expectTrue(!matchResult.fittedLines.empty())
        || !expectTrue(!matchResult.skeletonPoints.empty())
        || !expectTrue(std::abs(matchResult.weldPoint.y - 51.0) < 8.0)
        || !expectTrue(matchResult.meanResidualPixel < 15.0))
    {
        std::printf(
            "match valid=%d coarse=%d skeleton=%zu lines=%zu weld=(%f,%f) mean=%f message=%s\n",
            matchResult.valid ? 1 : 0,
            matchResult.coarseValid ? 1 : 0,
            matchResult.skeletonPoints.size(),
            matchResult.fittedLines.size(),
            matchResult.weldPoint.x,
            matchResult.weldPoint.y,
            matchResult.meanResidualPixel,
            matchResult.message.c_str());
        return fail(7);
    }

    DigitalTwinVisionLaserObservation shapeObservation;
    shapeObservation.frameId = 0;
    shapeObservation.laserLineImagePoints_px = {
        makeImagePoint(35.0, 50.0),
        makeImagePoint(43.0, 54.0),
        makeImagePoint(50.0, 58.0),
        makeImagePoint(57.0, 54.0),
        makeImagePoint(65.0, 50.0)};
    shapeObservation.weldPointImage_px = makeImagePoint(50.0, 58.0);
    shapeObservation.hasWeldPointImage_px = true;
    shapeObservation.valid = true;
    const std::vector<cv::Point2d> shapePoints{
        cv::Point2d(35.0, 50.0),
        cv::Point2d(43.0, 54.0),
        cv::Point2d(50.0, 58.0),
        cv::Point2d(57.0, 54.0),
        cv::Point2d(65.0, 50.0)};

    SimulatedLaserMatchingOptions shapeOptions;
    shapeOptions.shapeBasedTemplateSizePixel = 31;
    shapeOptions.shapeBasedRoiHalfSizePixel = 35;
    shapeOptions.shapeBasedMinSimilarity = 40.0;
    shapeOptions.templateAngleRangeDegree = 0.0;
    shapeOptions.templateAngleStepDegree = 1.0;
    shapeOptions.shapeBasedPyramidLevels = {4};
    const auto shapeTemplateResult = CSimulatedLaserTemplateMatcher::match(
        makeLaserImage(cv::Size(100, 90), shapePoints),
        shapeObservation,
        shapeOptions);
    if (!expectTrue(shapeTemplateResult.valid)
        || !expectTrue(shapeTemplateResult.templateMethod
            == "shape_based_matching")
        || !expectTrue(!shapeTemplateResult.usedChamferFallback)
        || !expectTrue(!shapeTemplateResult.matchedLaserPoints.empty())
        || !expectTrue(std::abs(shapeTemplateResult.weldPoint.y - 58.0)
            < 6.0))
    {
        std::printf("shape result valid=%d method=%s score=%f point=(%f,%f) message=%s\n",
            shapeTemplateResult.valid ? 1 : 0,
            shapeTemplateResult.templateMethod.c_str(),
            shapeTemplateResult.templateScore,
            shapeTemplateResult.weldPoint.x,
            shapeTemplateResult.weldPoint.y,
            shapeTemplateResult.message.c_str());
        return fail(8);
    }

    SimulatedLaserMatchingOptions fallbackOptions = shapeOptions;
    fallbackOptions.shapeBasedMinSimilarity = 101.0;
    fallbackOptions.templateSearchRadiusPixel = 8;
    fallbackOptions.templateSearchStepPixel = 1;
    fallbackOptions.templateMinScore = 0.30;
    const auto fallbackResult = CSimulatedLaserTemplateMatcher::match(
        makeLaserImage(cv::Size(100, 90), shapePoints),
        shapeObservation,
        fallbackOptions);
    if (!expectTrue(fallbackResult.valid)
        || !expectTrue(fallbackResult.templateMethod == "chamfer_fallback")
        || !expectTrue(fallbackResult.usedChamferFallback)
        || !expectTrue(fallbackResult.message.find("shape_based_matching failed")
            != std::string::npos))
    {
        return fail(9);
    }

    SimulatedLaserMatchingOptions templateOptions;
    templateOptions.enableShapeBasedTemplateMatching = false;
    templateOptions.logSearchMinGrayValue = 80;
    templateOptions.templateSearchRadiusPixel = 8;
    templateOptions.templateSearchStepPixel = 1;
    templateOptions.templateAngleRangeDegree = 2.0;
    templateOptions.templateAngleStepDegree = 1.0;
    templateOptions.templateMinScore = 0.30;
    const auto templateResult = CSimulatedLaserTemplateMatcher::match(
        makeLaserImage(laserPoints, 4.0),
        observation,
        templateOptions);
    if (!expectTrue(templateResult.valid)
        || !expectTrue(templateResult.usedTemplateFallback)
        || !expectTrue(templateResult.templateMethod == "chamfer")
        || !expectTrue(!templateResult.usedChamferFallback)
        || !expectTrue(templateResult.templateScore >= 0.30)
        || !expectTrue(std::abs(templateResult.weldPoint.y - 52.0) < 5.0))
    {
        return fail(10);
    }

    std::printf("PASS: vision image pro feng tools tests\n");
    return 0;
}


