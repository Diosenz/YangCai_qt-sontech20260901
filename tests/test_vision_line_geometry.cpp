/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_line_geometry.cpp
 * @brief       vision line geometry 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include "geometry/LineGeometryTools.h"

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
    point.label = "line_test_point";
    return point;
}

std::vector<i2r_rlc::vision::WeldFeaturePoint> makeStraightLine(void)
{
    std::vector<i2r_rlc::vision::WeldFeaturePoint> points;
    for (int index = 0; index < 12; ++index)
    {
        const double x = static_cast<double>(index);
        points.push_back(makePoint(x, 2.0 * x + 1.0));
    }
    return points;
}

std::vector<i2r_rlc::vision::WeldFeaturePoint> makeBrokenLine(void)
{
    std::vector<i2r_rlc::vision::WeldFeaturePoint> points;
    for (int index = 0; index <= 6; ++index)
    {
        points.push_back(makePoint(
            static_cast<double>(index),
            static_cast<double>(index)));
    }
    for (int index = 7; index <= 12; ++index)
    {
        points.push_back(makePoint(
            static_cast<double>(index),
            6.0 - 0.8 * static_cast<double>(index - 6)));
    }
    return points;
}

std::vector<i2r_rlc::vision::WeldFeaturePoint> makeLineWithOutliers(void)
{
    std::vector<i2r_rlc::vision::WeldFeaturePoint> points;
    for (int index = 0; index < 20; ++index)
    {
        const double x = static_cast<double>(index);
        points.push_back(makePoint(x, x + 3.0));
    }
    points.push_back(makePoint(4.0, 40.0));
    points.push_back(makePoint(15.0, -20.0));
    return points;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CLineGeometryTools;

    const auto straightPoints = makeStraightLine();
    const auto straightSplit =
        CLineGeometryTools::splitPolylineByDistance(straightPoints, 0.5);
    if (!expectTrue(straightSplit.valid)
        || !expectTrue(straightSplit.splitIndices.size() == 2)
        || !expectTrue(straightSplit.splitIndices.front() == 0)
        || !expectTrue(straightSplit.splitIndices.back() == 11))
    {
        return 1;
    }

    const auto leastSquaresLine =
        CLineGeometryTools::fitLineLeastSquares(straightPoints);
    if (!expectTrue(leastSquaresLine.valid)
        || !expectTrue(!leastSquaresLine.vertical)
        || !expectNear(leastSquaresLine.slope, 2.0, 0.001)
        || !expectNear(leastSquaresLine.intercept, 1.0, 0.001))
    {
        return 2;
    }

    const auto brokenSplit =
        CLineGeometryTools::splitPolylineByDistance(makeBrokenLine(), 1.0);
    if (!expectTrue(brokenSplit.valid)
        || !expectTrue(brokenSplit.splitIndices.size() >= 3)
        || !expectTrue(brokenSplit.maxDistancePixel > 1.0))
    {
        return 3;
    }

    const auto ransacLine = CLineGeometryTools::fitLineRansac(
        makeLineWithOutliers(),
        40,
        0.75,
        0.70);
    if (!expectTrue(ransacLine.valid)
        || !expectTrue(!ransacLine.vertical)
        || !expectNear(ransacLine.slope, 1.0, 0.08)
        || !expectNear(ransacLine.intercept, 3.0, 0.50))
    {
        return 4;
    }

    const auto lineA = CLineGeometryTools::fitLineLeastSquares(
        std::vector<i2r_rlc::vision::WeldFeaturePoint>{
            makePoint(0.0, 0.0),
            makePoint(10.0, 10.0)});
    const auto lineB = CLineGeometryTools::fitLineLeastSquares(
        std::vector<i2r_rlc::vision::WeldFeaturePoint>{
            makePoint(0.0, 10.0),
            makePoint(10.0, 0.0)});
    const auto intersection = CLineGeometryTools::intersectLines(lineA, lineB);
    if (!expectTrue(intersection.valid)
        || !expectNear(intersection.point.x, 5.0, 0.001)
        || !expectNear(intersection.point.y, 5.0, 0.001)
        || !expectNear(intersection.angleDegree, 90.0, 0.001))
    {
        return 5;
    }

    const auto parallelLineA = CLineGeometryTools::fitLineLeastSquares(
        std::vector<i2r_rlc::vision::WeldFeaturePoint>{
            makePoint(0.0, 1.0),
            makePoint(10.0, 11.0)});
    const auto parallelLineB = CLineGeometryTools::fitLineLeastSquares(
        std::vector<i2r_rlc::vision::WeldFeaturePoint>{
            makePoint(0.0, 3.0),
            makePoint(10.0, 13.0)});
    const auto parallelIntersection =
        CLineGeometryTools::intersectLines(parallelLineA, parallelLineB);
    if (!expectTrue(!parallelIntersection.valid)
        || !expectTrue(parallelIntersection.message == "parallel lines"))
    {
        return 6;
    }

    const auto invalidLine = CLineGeometryTools::fitLineLeastSquares(
        std::vector<i2r_rlc::vision::WeldFeaturePoint>{makePoint(1.0, 1.0)});
    if (!expectTrue(!invalidLine.valid)
        || !expectTrue(invalidLine.message == "not enough valid points"))
    {
        return 7;
    }

    std::printf("PASS: vision line geometry tests\n");
    return 0;
}
