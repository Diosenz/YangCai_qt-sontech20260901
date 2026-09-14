/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsLaserLineObservation.cpp
 * @brief     IBVS 模块激光中心线观测实现。
 * @details   本文件实现 IBVS 自有的激光线 ROI 构造、方向滤波和中心线提取流程；
 *            输入输出均使用 IBVS 模块值对象，不依赖 Localization 类型。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include <algorithm>
#include <cmath>

#ifdef I2R_RLC_ENABLE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

#include "ibvs/IbvsLaserLineObservation.h"

namespace i2r_rlc::ibvs {

#ifdef I2R_RLC_ENABLE_OPENCV
namespace {

constexpr double kPi = 3.14159265358979323846;

cv::Mat toGray8(const cv::Mat& image)
{
    if (image.empty())
    {
        return {};
    }
    if (image.channels() == 1)
    {
        if (image.type() == CV_8UC1)
        {
            return image.clone();
        }
        cv::Mat converted;
        image.convertTo(converted, CV_8UC1);
        return converted;
    }

    cv::Mat gray;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    if (image.channels() == 4)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return gray;
    }

    return {};
}

bool pointInsideImage(const PixelPoint& point, int width, int height)
{
    return point.u >= 0.0
        && point.v >= 0.0
        && point.u < static_cast<double>(width)
        && point.v < static_cast<double>(height);
}

cv::Rect makeRoi(const cv::Mat& gray, const IbvsLaserPrior& prior)
{
    if (prior.hasVirtualWeldPoint
        && pointInsideImage(prior.virtualWeldPoint, gray.cols, gray.rows))
    {
        const int expand = std::max(1, prior.roiExpandPx);
        const int centerX = static_cast<int>(std::lround(prior.virtualWeldPoint.u));
        const int centerY = static_cast<int>(std::lround(prior.virtualWeldPoint.v));
        const int left = std::max(0, centerX - expand);
        const int right = std::min(gray.cols - 1, centerX + expand);
        const int top = std::max(
            0,
            centerY - static_cast<int>(std::lround(
                static_cast<double>(expand) * prior.roiTopScale)));
        const int bottom = std::min(
            gray.rows - 1,
            centerY + static_cast<int>(std::lround(
                static_cast<double>(expand) * prior.roiBottomScale)));
        return cv::Rect(left, top, right - left + 1, bottom - top + 1);
    }

    if (!prior.virtualLaserLine.empty())
    {
        double minX = static_cast<double>(gray.cols - 1);
        double maxX = 0.0;
        double minY = static_cast<double>(gray.rows - 1);
        double maxY = 0.0;
        int validCount = 0;
        for (const PixelPoint& point : prior.virtualLaserLine)
        {
            if (!pointInsideImage(point, gray.cols, gray.rows))
            {
                continue;
            }
            minX = std::min(minX, point.u);
            maxX = std::max(maxX, point.u);
            minY = std::min(minY, point.v);
            maxY = std::max(maxY, point.v);
            ++validCount;
        }
        if (validCount > 0)
        {
            const int expand = std::max(1, prior.roiExpandPx / 4);
            const int left = std::max(0, static_cast<int>(std::floor(minX)) - expand);
            const int right = std::min(
                gray.cols - 1,
                static_cast<int>(std::ceil(maxX)) + expand);
            const int top = std::max(0, static_cast<int>(std::floor(minY)) - expand);
            const int bottom = std::min(
                gray.rows - 1,
                static_cast<int>(std::ceil(maxY)) + expand);
            return cv::Rect(left, top, right - left + 1, bottom - top + 1);
        }
    }

    return cv::Rect(0, 0, gray.cols, gray.rows);
}

std::vector<PixelPoint> filterPriorToRoi(
    const std::vector<PixelPoint>& points,
    const cv::Rect& roi)
{
    std::vector<PixelPoint> filtered;
    for (const PixelPoint& point : points)
    {
        if (point.u >= static_cast<double>(roi.x)
            && point.u < static_cast<double>(roi.x + roi.width)
            && point.v >= static_cast<double>(roi.y)
            && point.v < static_cast<double>(roi.y + roi.height))
        {
            filtered.push_back(point);
        }
    }
    return filtered;
}

double estimateGaborTheta(const std::vector<PixelPoint>& points)
{
    if (points.size() < 2U)
    {
        return 0.0;
    }

    double sumDx = 0.0;
    double sumDy = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const double dx = points[i].u - points[i - 1U].u;
        const double dy = points[i].v - points[i - 1U].v;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-9)
        {
            continue;
        }
        sumDx += dx / length;
        sumDy += dy / length;
    }
    if (std::hypot(sumDx, sumDy) <= 1e-9)
    {
        return 0.0;
    }

    return (kPi / 2.0) - std::atan2(sumDy, sumDx);
}

cv::Mat applyGabor(
    const cv::Mat& roiGray,
    const std::vector<PixelPoint>& priorLine,
    const IbvsLaserExtractionOptions& options)
{
    if (!options.enableGabor || priorLine.size() < 2U)
    {
        return roiGray.clone();
    }

    const int kernelSize = std::max(3, options.gaborKernelSize | 1);
    const cv::Mat kernel = cv::getGaborKernel(
        cv::Size(kernelSize, kernelSize),
        options.gaborSigma,
        estimateGaborTheta(priorLine),
        options.gaborLambda,
        options.gaborGamma,
        options.gaborPsi,
        CV_32F);
    cv::Mat roiFloat;
    roiGray.convertTo(roiFloat, CV_32F);
    cv::Mat filtered;
    cv::filter2D(roiFloat, filtered, CV_32F, kernel);
    cv::Mat normalized;
    cv::normalize(filtered, normalized, 0.0, 255.0, cv::NORM_MINMAX);
    cv::Mat gray8;
    normalized.convertTo(gray8, CV_8UC1);
    return gray8;
}

std::vector<PixelPoint> extractCenterline(
    const cv::Mat& roiGray,
    const cv::Rect& roi,
    const IbvsLaserExtractionOptions& options)
{
    cv::Mat smooth;
    const int gaussianKernel = std::max(3, options.gaussianKernelSize | 1);
    cv::GaussianBlur(
        roiGray,
        smooth,
        cv::Size(gaussianKernel, gaussianKernel),
        options.gaussianSigma,
        options.gaussianSigma);

    cv::Mat grayFloat;
    smooth.convertTo(grayFloat, CV_32F);
    cv::Mat dx;
    cv::Mat dy;
    cv::Mat dxx;
    cv::Mat dxy;
    cv::Mat dyy;
    cv::Sobel(grayFloat, dx, CV_32F, 1, 0, 3);
    cv::Sobel(grayFloat, dy, CV_32F, 0, 1, 3);
    cv::Sobel(grayFloat, dxx, CV_32F, 2, 0, 3);
    cv::Sobel(grayFloat, dxy, CV_32F, 1, 1, 3);
    cv::Sobel(grayFloat, dyy, CV_32F, 0, 2, 3);

    std::vector<PixelPoint> centerline;
    for (int y = 1; y < roiGray.rows - 1; ++y)
    {
        for (int x = 1; x < roiGray.cols - 1; ++x)
        {
            if (roiGray.at<uchar>(y, x) <= options.intensityThreshold)
            {
                continue;
            }

            const double hxx = dxx.at<float>(y, x);
            const double hxy = dxy.at<float>(y, x);
            const double hyy = dyy.at<float>(y, x);
            const double trace = hxx + hyy;
            const double delta = std::sqrt(
                std::max(0.0, (hxx - hyy) * (hxx - hyy) + 4.0 * hxy * hxy));
            const double lambda1 = 0.5 * (trace + delta);
            const double lambda2 = 0.5 * (trace - delta);
            const double lambda = std::abs(lambda1) > std::abs(lambda2)
                ? lambda1
                : lambda2;
            double nx = hxy;
            double ny = lambda - hxx;
            if (std::hypot(nx, ny) <= 1e-9)
            {
                nx = lambda - hyy;
                ny = hxy;
            }
            const double norm = std::hypot(nx, ny);
            if (norm <= 1e-9)
            {
                continue;
            }
            nx /= norm;
            ny /= norm;

            const double denominator =
                hxx * nx * nx + 2.0 * hxy * nx * ny + hyy * ny * ny;
            if (std::abs(denominator) <= 1e-9)
            {
                continue;
            }
            const double t =
                -(dx.at<float>(y, x) * nx + dy.at<float>(y, x) * ny)
                / denominator;
            if (std::abs(t * nx) <= 0.5 && std::abs(t * ny) <= 0.5)
            {
                centerline.push_back({
                    static_cast<double>(roi.x) + static_cast<double>(x) + t * nx,
                    static_cast<double>(roi.y) + static_cast<double>(y) + t * ny
                });
            }
        }
    }
    return centerline;
}

} // namespace

IbvsLaserObservationResult IbvsLaserLineExtractor::extractObservation(
    const cv::Mat& image,
    const IbvsLaserPrior& prior,
    const IbvsLaserExtractionOptions& options)
{
    IbvsLaserObservationResult result;
    result.observation.frameIndex = prior.frameIndex;

    const cv::Mat gray = toGray8(image);
    if (gray.empty())
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "IBVS laser image is empty or unsupported"
        };
        result.message = result.error.message;
        return result;
    }

    const cv::Rect roi = makeRoi(gray, prior);
    if (roi.width <= 2 || roi.height <= 2)
    {
        result.error = { IbvsStatus::INVALID_ARGUMENT, "IBVS laser ROI is invalid" };
        result.message = result.error.message;
        return result;
    }
    result.roi = { roi.x, roi.y, roi.width, roi.height };

    const cv::Mat roiGray = gray(roi).clone();
    const std::vector<PixelPoint> roiPrior =
        filterPriorToRoi(prior.virtualLaserLine, roi);
    const cv::Mat filtered = applyGabor(roiGray, roiPrior, options);
    result.centerline = extractCenterline(filtered, roi, options);
    if (result.centerline.empty())
    {
        result.error = { IbvsStatus::EMPTY_INPUT, "IBVS laser centerline is empty" };
        result.message = result.error.message;
        return result;
    }

    result.error = { IbvsStatus::OK, std::string() };
    result.centerlineQuality = std::min(
        1.0,
        static_cast<double>(result.centerline.size())
            / static_cast<double>(std::max(10, roi.width)));
    result.observation.hasStartPoint = true;
    result.observation.startPoint = result.centerline.front();
    result.observation.hasWeldPoint = true;
    result.observation.weldPoint =
        result.centerline[result.centerline.size() / 2U];
    result.observation.confidence = result.centerlineQuality;
    result.observation.centerlineQuality = result.centerlineQuality;
    result.message = "IBVS laser centerline extracted";
    return result;
}
#endif

} // namespace i2r_rlc::ibvs
