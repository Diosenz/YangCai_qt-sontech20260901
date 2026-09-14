/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_pro_share_tools.cpp
 * @brief       vision image pro share tools 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include "feature/ContourControlPointTools.h"
#include "geometry/LineGeometryTools.h"
#include "stripe/LaserLineResponseTools.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

i2r_rlc::vision::WeldFeaturePoint makePoint(double xPixel, double yPixel)
{
    i2r_rlc::vision::WeldFeaturePoint point;
    point.xPixel = xPixel;
    point.yPixel = yPixel;
    point.confidence = 1.0;
    point.valid = true;
    point.label = "image_pro_share_test_point";
    return point;
}

std::vector<i2r_rlc::vision::WeldFeaturePoint> makePolyline(void)
{
    std::vector<i2r_rlc::vision::WeldFeaturePoint> points;
    for (int index = 0; index <= 5; ++index)
    {
        points.push_back(makePoint(index, index));
    }
    for (int index = 6; index <= 10; ++index)
    {
        points.push_back(makePoint(index, 10 - index));
    }
    for (int index = 11; index <= 15; ++index)
    {
        points.push_back(makePoint(index, index - 10));
    }
    return points;
}

cv::Mat makeLogImage(void)
{
    cv::Mat image(30, 20, CV_8UC1, cv::Scalar(0));
    image.at<unsigned char>(9, 5) = 255;
    image.at<unsigned char>(10, 5) = 255;
    image.at<unsigned char>(14, 5) = 255;
    image.at<unsigned char>(15, 5) = 255;
    image.at<unsigned char>(14, 6) = 255;
    image.at<unsigned char>(15, 6) = 255;
    return image;
}

cv::Mat makeHorizontalLaserImage(void)
{
    cv::Mat image(30, 30, CV_8UC1, cv::Scalar(0));
    for (int x = 3; x <= 25; ++x)
    {
        image.at<unsigned char>(15, x) = 255;
    }
    return image;
}

bool hasControlPointNearX(
    const std::vector<cv::Point2d> &points,
    double xPixel,
    double tolerance)
{
    for (const cv::Point2d &point : points)
    {
        if (std::fabs(point.x - xPixel) <= tolerance)
        {
            return true;
        }
    }
    return false;
}

} // namespace

int main()
{
    using i2r_rlc::vision::ContourControlPointOptions;
    using i2r_rlc::vision::CContourControlPointTools;
    using i2r_rlc::vision::ControlPointGrowDirection;
    using i2r_rlc::vision::CLaserLineResponseTools;
    using i2r_rlc::vision::LaserLogPeakOptions;
    using i2r_rlc::vision::CLineGeometryTools;
    using i2r_rlc::vision::VisionPreprocessStatus;

    const std::vector<int> laserWidthByRow(30, 2);
    const cv::Mat logImage = makeLogImage();
    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const double response = CLaserLineResponseTools::calculateLogResponseAt(
        logImage,
        laserWidthByRow,
        5,
        10,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectNear(response, 255.0, 0.001))
    {
        return 1;
    }

    const auto centerMask = CLaserLineResponseTools::buildEfficientMask(0, 0);
    const auto leftMask = CLaserLineResponseTools::buildEfficientMask(-1, 0);
    if (!expectTrue(centerMask.valid)
        || !expectTrue(centerMask.offsets.size() == 9)
        || !expectTrue(leftMask.valid)
        || !expectTrue(leftMask.offsets.size() == 3))
    {
        return 2;
    }

    LaserLogPeakOptions peakOptions;
    peakOptions.rowMinPixel = 6;
    peakOptions.rowMaxPixel = 18;
    peakOptions.columnStepPixel = 2;
    const auto peakResult = CLaserLineResponseTools::detectLogPeaks(
        logImage,
        laserWidthByRow,
        peakOptions,
        &status);
    if (!expectTrue(peakResult.valid)
        || !expectTrue(peakResult.maxRowByColumn[5] == -1)
        || !expectTrue(peakResult.maxRowByColumn[6] > 0)
        || !expectTrue(peakResult.maxRowByColumn[7] == -1))
    {
        return 3;
    }

    cv::Mat centerImage(30, 12, CV_8UC1, cv::Scalar(0));
    centerImage.at<unsigned char>(14, 4) = 255;
    centerImage.at<unsigned char>(15, 4) = 255;
    centerImage.at<unsigned char>(16, 4) = 255;
    std::vector<int> peakRows(12, -1);
    peakRows[4] = 15;
    const auto centerResult = CLaserLineResponseTools::refineCenterOfMass(
        centerImage,
        peakRows,
        std::vector<int>(30, 3),
        5,
        &status);
    if (!expectTrue(centerResult.valid)
        || !expectNear(centerResult.refinedRowByColumn[4], 15.0, 0.25))
    {
        return 4;
    }

    const auto draggedPoint = CLaserLineResponseTools::dragPointToLaserLine(
        logImage,
        laserWidthByRow,
        cv::Point2i(5, 10),
        6,
        100,
        &status);
    if (!expectTrue(draggedPoint.valid)
        || !expectTrue(draggedPoint.point.y >= 10.0))
    {
        return 5;
    }

    ContourControlPointOptions contourOptions;
    contourOptions.growSampleStepPixel = 5;
    contourOptions.dragSearchHalfHeightPixel = 6;
    const cv::Mat laserImage = makeHorizontalLaserImage();
    const auto leftGrow = CContourControlPointTools::growLaserControlPoints(
        laserImage,
        cv::Point2i(15, 15),
        ControlPointGrowDirection::Left,
        contourOptions,
        &status);
    const auto rightGrow = CContourControlPointTools::growLaserControlPoints(
        laserImage,
        cv::Point2i(15, 15),
        ControlPointGrowDirection::Right,
        contourOptions,
        &status);
    if (!expectTrue(leftGrow.valid)
        || !expectTrue(rightGrow.valid)
        || !expectTrue(leftGrow.points.front().x < 15.0)
        || !expectTrue(rightGrow.points.back().x > 15.0))
    {
        return 6;
    }

    std::vector<cv::Point2d> controlPoints;
    controlPoints.push_back(cv::Point2d(7.0, 13.0));
    controlPoints.push_back(cv::Point2d(15.0, 15.0));
    controlPoints.push_back(cv::Point2d(22.0, 15.0));
    const auto adjustedCurve =
        CContourControlPointTools::adjustOfflineControlPoints(
            laserImage,
            std::vector<int>(30, 2),
            controlPoints,
            contourOptions,
            &status);
    if (!expectTrue(adjustedCurve.valid)
        || !expectTrue(adjustedCurve.controlPoints.size() >= 3)
        || !expectTrue(adjustedCurve.curvePoints.size()
            > adjustedCurve.controlPoints.size())
        || !expectNear(
            adjustedCurve.curvePoints.front().x,
            adjustedCurve.controlPoints.front().x,
            0.001)
        || !expectNear(
            adjustedCurve.curvePoints.back().x,
            adjustedCurve.controlPoints.back().x,
            0.001))
    {
        return 7;
    }

    std::vector<cv::Point2d> failingDragPoints;
    failingDragPoints.push_back(cv::Point2d(7.0, 15.0));
    failingDragPoints.push_back(cv::Point2d(15.0, 15.0));
    failingDragPoints.push_back(cv::Point2d(28.0, 5.0));
    const auto deletedOfflinePointCurve =
        CContourControlPointTools::adjustOfflineControlPoints(
            laserImage,
            std::vector<int>(30, 2),
            failingDragPoints,
            contourOptions,
            &status);
    if (!expectTrue(deletedOfflinePointCurve.valid)
        || !expectTrue(!hasControlPointNearX(
            deletedOfflinePointCurve.controlPoints,
            28.0,
            0.25)))
    {
        return 13;
    }

    cv::Mat curveScanImage(50, 90, CV_8UC1, cv::Scalar(0));
    curveScanImage.at<unsigned char>(20, 10) = 255;
    curveScanImage.at<unsigned char>(20, 70) = 255;
    curveScanImage.at<unsigned char>(22, 40) = 255;
    ContourControlPointOptions curveScanOptions = contourOptions;
    curveScanOptions.dragSearchHalfHeightPixel = 6;
    curveScanOptions.interpolationSamplesPerSegment = 60;
    curveScanOptions.curveScanMaxPointCount = 1000;
    const auto curveScanResult =
        CContourControlPointTools::adjustOfflineControlPoints(
            curveScanImage,
            std::vector<int>(50, 3),
            std::vector<cv::Point2d>{
                cv::Point2d(10.0, 20.0),
                cv::Point2d(70.0, 20.0)},
            curveScanOptions,
            &status);
    if (!expectTrue(curveScanResult.valid)
        || !expectTrue(hasControlPointNearX(
            curveScanResult.controlPoints,
            40.0,
            0.25)))
    {
        return 14;
    }

    cv::Mat gapImage(50, 90, CV_8UC1, cv::Scalar(0));
    gapImage.at<unsigned char>(20, 10) = 255;
    gapImage.at<unsigned char>(20, 70) = 255;
    ContourControlPointOptions gapOptions = contourOptions;
    gapOptions.interpolationSamplesPerSegment = 60;
    gapOptions.maxControlPointGapPixel = 30;
    const auto gapInsertedCurve =
        CContourControlPointTools::adjustOfflineControlPoints(
            gapImage,
            std::vector<int>(50, 3),
            std::vector<cv::Point2d>{
                cv::Point2d(10.0, 20.0),
                cv::Point2d(70.0, 20.0)},
            gapOptions,
            &status);
    if (!expectTrue(gapInsertedCurve.valid)
        || !expectTrue(hasControlPointNearX(
            gapInsertedCurve.controlPoints,
            40.0,
            1.0)))
    {
        return 15;
    }

    const auto linearCurve = CContourControlPointTools::interpolateControlCurve(
        std::vector<cv::Point2d>{
            cv::Point2d(2.0, 10.0),
            cv::Point2d(12.0, 20.0)},
        5,
        &status);
    if (!expectTrue(linearCurve.valid)
        || !expectTrue(linearCurve.curvePoints.size() == 6)
        || !expectNear(linearCurve.curvePoints.front().x, 2.0, 0.001)
        || !expectNear(linearCurve.curvePoints.back().x, 12.0, 0.001))
    {
        return 10;
    }

    const auto repeatedPointCurve =
        CContourControlPointTools::interpolateControlCurve(
            std::vector<cv::Point2d>{
                cv::Point2d(1.0, 3.0),
                cv::Point2d(1.0, 3.0),
                cv::Point2d(6.0, 8.0),
                cv::Point2d(12.0, 6.0)},
            4,
            &status);
    if (!expectTrue(repeatedPointCurve.valid)
        || !expectTrue(repeatedPointCurve.controlPoints.size() == 3)
        || !expectTrue(repeatedPointCurve.curvePoints.size()
            > repeatedPointCurve.controlPoints.size()))
    {
        return 11;
    }

    const auto insufficientCurve =
        CContourControlPointTools::interpolateControlCurve(
            std::vector<cv::Point2d>{cv::Point2d(1.0, 3.0)},
            4,
            &status);
    if (!expectTrue(!insufficientCurve.valid)
        || !expectTrue(status == VisionPreprocessStatus::InsufficientPoints))
    {
        return 12;
    }

    const auto splitResult =
        CLineGeometryTools::splitPolylineByLegacyRule(makePolyline(), 1.0);
    const auto segmentLines = CLineGeometryTools::fitSegmentsRansac(
        makePolyline(),
        splitResult.splitIndices,
        40,
        0.75,
        0.50);
    if (!expectTrue(splitResult.valid)
        || !expectTrue(splitResult.splitIndices.size() >= 3)
        || !expectTrue(segmentLines.valid)
        || !expectTrue(segmentLines.lines.size() >= 2))
    {
        return 8;
    }

    const auto intersection =
        CLineGeometryTools::intersectLinesNearReferenceX(
            segmentLines.lines,
            7.0);
    if (!expectTrue(intersection.valid)
        || !expectTrue(intersection.point.x >= 4.0)
        || !expectTrue(intersection.point.x <= 12.0))
    {
        return 9;
    }

    std::printf("PASS: vision image pro share tools tests\n");
    return 0;
}
