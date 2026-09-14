/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsImageProjectionTools.cpp
 * @brief     IBVS 内部 3D 模板投影与像素采样实现。
 * @details   本文件实现相机矩阵构造、扰动位姿计算、模板边投影和采样转换。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>

#include "IbvsImageProjectionTools.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double DEG_TO_RAD = CV_PI / 180.0;

cv::Mat toColumnMat(const cv::Vec3d& value)
{
    return (cv::Mat_<double>(3, 1) << value[0], value[1], value[2]);
}

cv::Vec3d toVec3d(const cv::Mat& value)
{
    return cv::Vec3d(
        value.at<double>(0, 0),
        value.at<double>(1, 0),
        value.at<double>(2, 0));
}

cv::Vec3d normalizedVec3(const cv::Mat& value)
{
    return cv::Vec3d(
        value.at<double>(0, 0),
        value.at<double>(1, 0),
        value.at<double>(2, 0));
}

} // namespace

cv::Mat IbvsImageProjectionTools::makeCameraMatrix(
    const IbvsCameraIntrinsics& intrinsics)
{
    cv::Mat camera = cv::Mat::eye(3, 3, CV_64F);
    camera.at<double>(0, 0) = intrinsics.fx;
    camera.at<double>(1, 1) = intrinsics.fy;
    camera.at<double>(0, 2) = intrinsics.cx;
    camera.at<double>(1, 2) = intrinsics.cy;
    return camera;
}

void IbvsImageProjectionTools::computePerturbedPose(
    const cv::Vec3d& rvec0,
    const cv::Vec3d& tvec0,
    double dxMm,
    double dyMm,
    double dpsiDeg,
    cv::Vec3d* rvecOut,
    cv::Vec3d* tvecOut)
{
    cv::Mat r0;
    cv::Rodrigues(toColumnMat(rvec0), r0);

    const double angle = dpsiDeg * DEG_TO_RAD;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const cv::Mat rz = (cv::Mat_<double>(3, 3) <<
        c, -s, 0.0,
        s,  c, 0.0,
        0.0, 0.0, 1.0);
    const cv::Mat tDelta = (cv::Mat_<double>(3, 1) << dxMm, dyMm, 0.0);

    const cv::Mat rOut = r0 * rz;
    const cv::Mat tOut = r0 * tDelta + toColumnMat(tvec0);

    cv::Mat rvecMat;
    cv::Rodrigues(rOut, rvecMat);
    if (rvecOut != nullptr)
    {
        *rvecOut = normalizedVec3(rvecMat);
    }
    if (tvecOut != nullptr)
    {
        *tvecOut = toVec3d(tOut);
    }
}

std::vector<std::vector<cv::Point2f>> IbvsImageProjectionTools::projectEdges(
    const std::vector<std::vector<cv::Point3f>>& edges3d,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat& cameraMatrix)
{
    std::vector<std::vector<cv::Point2f>> projectedEdges;
    projectedEdges.reserve(edges3d.size());
    const cv::Mat distCoeffs;

    for (const auto& edge3d : edges3d)
    {
        std::vector<cv::Point2f> edge2d;
        if (!edge3d.empty())
        {
            cv::projectPoints(edge3d, rvec, tvec, cameraMatrix, distCoeffs, edge2d);
        }
        projectedEdges.push_back(edge2d);
    }
    return projectedEdges;
}

std::vector<cv::Point2f> IbvsImageProjectionTools::flattenEdges(
    const std::vector<std::vector<cv::Point2f>>& edges)
{
    std::vector<cv::Point2f> points;
    for (const auto& edge : edges)
    {
        points.insert(points.end(), edge.begin(), edge.end());
    }
    return points;
}

std::vector<PixelPoint> IbvsImageProjectionTools::toPixelPoints(
    const std::vector<cv::Point2f>& points)
{
    std::vector<PixelPoint> converted;
    converted.reserve(points.size());
    for (const cv::Point2f& point : points)
    {
        converted.push_back(ibvsToPixelPoint(point));
    }
    return converted;
}

IbvsProjectedTemplateModel IbvsImageProjectionTools::makeProjectedTemplateModel(
    const std::vector<std::vector<cv::Point3f>>& edges3d)
{
    IbvsProjectedTemplateModel model;
    for (const auto& edge : edges3d)
    {
        if (edge.empty())
        {
            continue;
        }

        const int start = static_cast<int>(model.points3d.size());
        model.points3d.insert(model.points3d.end(), edge.begin(), edge.end());
        const int end = static_cast<int>(model.points3d.size());
        model.edgeRanges.emplace_back(start, end);
    }
    return model;
}

std::vector<cv::Point2f> IbvsImageProjectionTools::sampleProjectedEdges(
    const std::vector<std::vector<cv::Point2f>>& projectedEdges,
    double scale,
    double sampleIntervalPx)
{
    std::vector<cv::Point2f> sampledPoints;
    const float scaleF = static_cast<float>(scale);
    const float interval =
        static_cast<float>(std::max(1e-6, sampleIntervalPx * scale));

    for (const auto& edge : projectedEdges)
    {
        if (edge.empty())
        {
            continue;
        }

        std::vector<cv::Point2f> scaledEdge;
        scaledEdge.reserve(edge.size());
        for (const cv::Point2f& point : edge)
        {
            scaledEdge.emplace_back(point.x * scaleF, point.y * scaleF);
        }

        sampledPoints.push_back(scaledEdge.front());
        float accuLength = 0.0f;
        float nextSampleAt = interval;
        for (std::size_t i = 1; i < scaledEdge.size(); ++i)
        {
            const cv::Point2f segment = scaledEdge[i] - scaledEdge[i - 1];
            const float segmentLength = std::sqrt(
                segment.x * segment.x + segment.y * segment.y);
            while (accuLength + segmentLength >= nextSampleAt
                   && segmentLength > 1e-6f)
            {
                const float t = (nextSampleAt - accuLength) / segmentLength;
                if (t >= 0.0f && t <= 1.0f)
                {
                    sampledPoints.push_back(scaledEdge[i - 1] + t * segment);
                }
                nextSampleAt += interval;
            }
            accuLength += segmentLength;
        }
    }
    return sampledPoints;
}

std::vector<cv::Point2f> IbvsImageProjectionTools::projectAndSampleEdges(
    const std::vector<std::vector<cv::Point3f>>& edges3d,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat& cameraMatrix,
    double scale,
    double sampleIntervalPx)
{
    return sampleProjectedEdges(
        projectEdges(edges3d, rvec, tvec, cameraMatrix),
        scale,
        sampleIntervalPx);
}

std::vector<cv::Point2f> IbvsImageProjectionTools::projectAndSampleEdges(
    const IbvsProjectedTemplateModel& model,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat& cameraMatrix,
    double scale,
    double sampleIntervalPx)
{
    if (model.points3d.empty())
    {
        return {};
    }

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(model.points3d, rvec, tvec, cameraMatrix, cv::Mat(), projectedPoints);

    const float scaleF = static_cast<float>(scale);
    for (cv::Point2f& point : projectedPoints)
    {
        point.x *= scaleF;
        point.y *= scaleF;
    }

    const float interval =
        static_cast<float>(std::max(1e-6, sampleIntervalPx * scale));
    std::vector<cv::Point2f> sampledPoints;

    for (const auto& range : model.edgeRanges)
    {
        if (range.second <= range.first)
        {
            continue;
        }

        sampledPoints.push_back(projectedPoints[range.first]);
        float accuLength = 0.0f;
        float nextSampleAt = interval;
        for (int i = range.first + 1; i < range.second; ++i)
        {
            const cv::Point2f segment =
                projectedPoints[i] - projectedPoints[i - 1];
            const float segmentLength = std::sqrt(
                segment.x * segment.x + segment.y * segment.y);
            while (accuLength + segmentLength >= nextSampleAt
                   && segmentLength > 1e-6f)
            {
                const float t = (nextSampleAt - accuLength) / segmentLength;
                if (t >= 0.0f && t <= 1.0f)
                {
                    sampledPoints.push_back(projectedPoints[i - 1] + t * segment);
                }
                nextSampleAt += interval;
            }
            accuLength += segmentLength;
        }
    }

    return sampledPoints;
}

PixelPoint ibvsToPixelPoint(const cv::Point2f& point)
{
    return { point.x, point.y };
}

bool ibvsIsInsideImage(const cv::Point2f& point, const cv::Size& imageSize)
{
    return point.x >= 0.0f
        && point.y >= 0.0f
        && point.x < static_cast<float>(imageSize.width)
        && point.y < static_cast<float>(imageSize.height);
}

} // namespace i2r_rlc::ibvs
