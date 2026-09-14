/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      test_ibvs_start_point_offline.cpp
 * @brief     IBVS 起始点纯算法的离线诊断和可视化测试。
 * @details   测试覆盖端点、直线交点和曲线最近点三类入口；输入为可控
 *            离线点集，不迁入 legacy CAD/PnP/模板对齐完整图像流程。
 * @author    I2R Laboratory
 * @version   1.0
 */

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsStartPointDetector.h"

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

namespace {

using namespace i2r_rlc::ibvs;

std::string makeTimestamp(void)
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(now);
    std::tm localTime;
    localtime_s(&localTime, &timeValue);

    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::filesystem::path makeArtifactDir(void)
{
    const std::filesystem::path root =
        std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "output"
        / "ibvs_offline_validation"
        / "i2r_rlc_ibvs_start_point_offline_tests"
        / makeTimestamp();
    std::filesystem::create_directories(root);
    return root;
}

bool expectTrue(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectNear(
    double actual,
    double expected,
    double tolerance,
    const char* message)
{
    if (std::abs(actual - expected) > tolerance)
    {
        std::cerr << "FAIL: " << message
                  << ", actual=" << actual
                  << ", expected=" << expected << '\n';
        return false;
    }
    return true;
}

void drawPoint(
    cv::Mat& image,
    const PixelPoint& point,
    const cv::Scalar& color,
    int radius)
{
    cv::circle(
        image,
        cv::Point(
            static_cast<int>(std::lround(point.u)),
            static_cast<int>(std::lround(point.v))),
        radius,
        color,
        cv::FILLED,
        cv::LINE_AA);
}

void drawLineModel(
    cv::Mat& image,
    const IbvsLineModel& line,
    const cv::Scalar& color)
{
    if (!line.valid || std::abs(line.b) < 1e-9)
    {
        return;
    }

    const double y0 = -line.c / line.b;
    const double y1 = -(line.a * (image.cols - 1) + line.c) / line.b;
    cv::line(
        image,
        cv::Point(0, static_cast<int>(std::lround(y0))),
        cv::Point(image.cols - 1, static_cast<int>(std::lround(y1))),
        color,
        2,
        cv::LINE_AA);
}

void writePointSetCsv(
    const std::filesystem::path& path,
    const std::vector<PixelPoint>& points)
{
    std::ofstream file(path);
    file << "u,v\n";
    for (const PixelPoint& point : points)
    {
        file << point.u << ',' << point.v << '\n';
    }
}

void drawScenario(
    const std::filesystem::path& path,
    const std::vector<PixelPoint>& points,
    const IbvsStartPointResult& result,
    const PixelPoint* referencePoint)
{
    cv::Mat image(420, 620, CV_8UC3, cv::Scalar(35, 35, 35));
    for (const PixelPoint& point : points)
    {
        drawPoint(image, point, cv::Scalar(180, 180, 180), 3);
    }

    drawLineModel(image, result.line, cv::Scalar(0, 255, 255));
    if (referencePoint != nullptr)
    {
        drawPoint(image, *referencePoint, cv::Scalar(255, 0, 255), 6);
    }
    if (result.error.isOk())
    {
        drawPoint(image, result.startPoint, cv::Scalar(0, 0, 255), 8);
        drawPoint(image, result.endPoint, cv::Scalar(0, 255, 0), 7);
    }

    cv::imwrite(path.string(), image);
}

} // namespace

/**
 * @brief 起始点纯算法离线测试入口。
 */
int main(void)
{
    const std::filesystem::path artifactDir = makeArtifactDir();
    bool ok = true;

    const std::vector<PixelPoint> linePoints{
        { 80.0, 260.0 },
        { 160.0, 240.0 },
        { 240.0, 220.0 },
        { 320.0, 200.0 },
        { 400.0, 180.0 },
        { 480.0, 160.0 }
    };
    const auto endpoint =
        IbvsStartPointDetector::computeEndpointStartPoint(linePoints, true);
    ok &= expectTrue(endpoint.error.isOk(), "endpoint start point should succeed");
    ok &= expectNear(endpoint.startPoint.u, 80.0, 1e-6, "left endpoint u");
    ok &= expectNear(endpoint.endPoint.u, 480.0, 1e-6, "right endpoint u");
    drawScenario(artifactDir / "endpoint_start_point.jpg", linePoints, endpoint, nullptr);
    writePointSetCsv(artifactDir / "endpoint_points.csv", linePoints);

    const IbvsLineModel secondEdge{ 1.0, 0.0, -320.0, true };
    const auto intersection =
        IbvsStartPointDetector::computeIntersectionStartPoint(
            endpoint.line,
            secondEdge);
    ok &= expectTrue(
        intersection.error.isOk(),
        "intersection start point should succeed");
    ok &= expectTrue(
        intersection.startPoint.isFinite(),
        "intersection start point should be finite");
    drawScenario(
        artifactDir / "intersection_start_point.jpg",
        linePoints,
        intersection,
        nullptr);

    const std::vector<PixelPoint> curvePoints{
        { 100.0, 340.0 },
        { 160.0, 300.0 },
        { 230.0, 270.0 },
        { 310.0, 260.0 },
        { 390.0, 275.0 },
        { 470.0, 310.0 }
    };
    const PixelPoint torchEndPoint{ 380.0, 280.0 };
    const auto curveNearest =
        IbvsStartPointDetector::computeCurveNearestStartPoint(
            curvePoints,
            torchEndPoint);
    ok &= expectTrue(
        curveNearest.error.isOk(),
        "curve nearest start point should succeed");
    ok &= expectNear(curveNearest.startPoint.u, 390.0, 1e-6, "curve nearest u");
    ok &= expectNear(curveNearest.startPoint.v, 275.0, 1e-6, "curve nearest v");
    drawScenario(
        artifactDir / "curve_nearest_start_point.jpg",
        curvePoints,
        curveNearest,
        &torchEndPoint);
    writePointSetCsv(artifactDir / "curve_points.csv", curvePoints);

    std::ofstream metrics(artifactDir / "metrics.csv");
    metrics << "scenario,accepted,start_u,start_v,end_u,end_v,line_a,line_b,line_c\n";
    metrics << "endpoint," << (endpoint.error.isOk() ? 1 : 0) << ','
            << endpoint.startPoint.u << ',' << endpoint.startPoint.v << ','
            << endpoint.endPoint.u << ',' << endpoint.endPoint.v << ','
            << endpoint.line.a << ',' << endpoint.line.b << ','
            << endpoint.line.c << '\n';
    metrics << "intersection," << (intersection.error.isOk() ? 1 : 0) << ','
            << intersection.startPoint.u << ','
            << intersection.startPoint.v << ','
            << intersection.endPoint.u << ','
            << intersection.endPoint.v << ','
            << intersection.line.a << ',' << intersection.line.b << ','
            << intersection.line.c << '\n';
    metrics << "curve_nearest," << (curveNearest.error.isOk() ? 1 : 0) << ','
            << curveNearest.startPoint.u << ','
            << curveNearest.startPoint.v << ','
            << curveNearest.endPoint.u << ','
            << curveNearest.endPoint.v << ','
            << curveNearest.line.a << ',' << curveNearest.line.b << ','
            << curveNearest.line.c << '\n';

    std::ofstream summary(artifactDir / "summary.txt");
    summary << "endpointAccepted=" << (endpoint.error.isOk() ? 1 : 0) << '\n'
            << "intersectionAccepted=" << (intersection.error.isOk() ? 1 : 0) << '\n'
            << "curveNearestAccepted=" << (curveNearest.error.isOk() ? 1 : 0) << '\n'
            << "artifactDir=" << artifactDir.string() << '\n';

    if (!ok)
    {
        return 1;
    }

    std::cout << "INFO: start point artifacts=" << artifactDir.string() << '\n';
    std::cout << "PASS: IBVS start point offline tests\n";
    return 0;
}
