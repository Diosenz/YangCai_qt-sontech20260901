/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsStartPointDetector.cpp
 * @brief     IBVS 焊缝起始点几何计算实现。
 * @details   本文件实现 TLS 直线拟合、端点法、交点法和曲线最近点起点计算。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

#include "ibvs/IbvsStartPointDetector.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double MIN_LINE_NORM = 1e-12;

double squaredDistance(const PixelPoint& first, const PixelPoint& second)
{
    const double du = first.u - second.u;
    const double dv = first.v - second.v;
    return du * du + dv * dv;
}

} // namespace

IbvsLineModel IbvsStartPointDetector::fitLineTls(
    const std::vector<PixelPoint>& points)
{
    IbvsLineModel line;
    if (points.size() < 2)
    {
        return line;
    }

    double meanU = 0.0;
    double meanV = 0.0;
    for (const PixelPoint& point : points)
    {
        if (!point.isFinite())
        {
            return line;
        }
        meanU += point.u;
        meanV += point.v;
    }
    meanU /= static_cast<double>(points.size());
    meanV /= static_cast<double>(points.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const PixelPoint& point : points)
    {
        const Eigen::Vector2d centered(point.u - meanU, point.v - meanV);
        covariance += centered * centered.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covariance);
    if (eig.info() != Eigen::Success)
    {
        return line;
    }

    const Eigen::Vector2d normal = eig.eigenvectors().col(0);
    const double norm = normal.norm();
    if (norm < MIN_LINE_NORM)
    {
        return line;
    }

    line.a = normal.x() / norm;
    line.b = normal.y() / norm;
    line.c = -(line.a * meanU + line.b * meanV);
    line.valid = true;
    return line;
}

IbvsStartPointResult IbvsStartPointDetector::computeEndpointStartPoint(
    const std::vector<PixelPoint>& weldSeamPoints,
    bool useLeftSide)
{
    IbvsStartPointResult result;
    if (weldSeamPoints.size() < 2)
    {
        result.error = {
            IbvsStatus::NOT_ENOUGH_SAMPLES,
            "endpoint start point requires at least two weld seam points"
        };
        return result;
    }

    result.line = fitLineTls(weldSeamPoints);
    if (!result.line.valid)
    {
        result.error = {
            IbvsStatus::DEGENERATE_GEOMETRY,
            "failed to fit weld seam line"
        };
        return result;
    }

    double meanU = 0.0;
    double meanV = 0.0;
    for (const PixelPoint& point : weldSeamPoints)
    {
        meanU += point.u;
        meanV += point.v;
    }
    meanU /= static_cast<double>(weldSeamPoints.size());
    meanV /= static_cast<double>(weldSeamPoints.size());

    const double tangentU = -result.line.b;
    const double tangentV = result.line.a;
    double minProjection = std::numeric_limits<double>::max();
    double maxProjection = std::numeric_limits<double>::lowest();
    PixelPoint minPoint;
    PixelPoint maxPoint;

    for (const PixelPoint& point : weldSeamPoints)
    {
        const double projection =
            (point.u - meanU) * tangentU + (point.v - meanV) * tangentV;
        if (projection < minProjection)
        {
            minProjection = projection;
            minPoint = point;
        }
        if (projection > maxProjection)
        {
            maxProjection = projection;
            maxPoint = point;
        }
    }

    const bool minIsLeft = minPoint.u <= maxPoint.u;
    if ((useLeftSide && minIsLeft) || (!useLeftSide && !minIsLeft))
    {
        result.startPoint = minPoint;
        result.endPoint = maxPoint;
    }
    else
    {
        result.startPoint = maxPoint;
        result.endPoint = minPoint;
    }

    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

IbvsStartPointResult IbvsStartPointDetector::computeIntersectionStartPoint(
    const IbvsLineModel& weldLine,
    const IbvsLineModel& secondEdgeLine)
{
    IbvsStartPointResult result;
    if (!weldLine.valid || !secondEdgeLine.valid)
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "intersection start point requires two valid lines"
        };
        return result;
    }

    const double det = weldLine.a * secondEdgeLine.b
        - secondEdgeLine.a * weldLine.b;
    if (std::abs(det) < MIN_LINE_NORM)
    {
        result.error = {
            IbvsStatus::DEGENERATE_GEOMETRY,
            "weld seam line and second edge are parallel"
        };
        return result;
    }

    result.startPoint = {
        (weldLine.b * secondEdgeLine.c - secondEdgeLine.b * weldLine.c) / det,
        (secondEdgeLine.a * weldLine.c - weldLine.a * secondEdgeLine.c) / det
    };
    result.endPoint = result.startPoint;
    result.line = weldLine;
    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

IbvsStartPointResult IbvsStartPointDetector::computeCurveNearestStartPoint(
    const std::vector<PixelPoint>& curveWeldSeamPoints,
    const PixelPoint& torchEndPoint)
{
    IbvsStartPointResult result;
    if (curveWeldSeamPoints.empty())
    {
        result.error = {
            IbvsStatus::EMPTY_INPUT,
            "curve start point requires weld seam curve points"
        };
        return result;
    }
    if (!torchEndPoint.isFinite())
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "torch end point is invalid"
        };
        return result;
    }

    double bestDistance = std::numeric_limits<double>::max();
    for (const PixelPoint& point : curveWeldSeamPoints)
    {
        if (!point.isFinite())
        {
            continue;
        }

        const double distance = squaredDistance(point, torchEndPoint);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            result.startPoint = point;
        }
    }

    if (bestDistance == std::numeric_limits<double>::max())
    {
        result.error = {
            IbvsStatus::DEGENERATE_GEOMETRY,
            "curve points do not contain a finite candidate"
        };
        return result;
    }

    result.endPoint = result.startPoint;
    result.line = fitLineTls(curveWeldSeamPoints);
    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

} // namespace i2r_rlc::ibvs
