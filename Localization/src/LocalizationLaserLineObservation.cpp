/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationLaserLineObservation.cpp
 * @brief     定位模块激光中心线与点云窗口实现。
 * @details   本文件实现 Localization 自有的激光线 ROI 构造、方向滤波、中心线提取
 *            和局部点云滑窗累积流程；输入输出均使用定位模块值对象。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include <algorithm>
#include <cmath>

#ifdef I2R_RLC_ENABLE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

#include "LocalizationLaserLineObservation.h"

namespace i2r_rlc::localization {

namespace {

constexpr double kPi = 3.14159265358979323846;

int positiveStride(int stride)
{
    return std::max(1, stride);
}

std::vector<ImagePoint_px> toProjectionPoints(
    const std::vector<LocalizationLaserImagePoint_px>& points,
    int stride)
{
    const int step = positiveStride(stride);
    std::vector<ImagePoint_px> projectionPoints;
    projectionPoints.reserve(points.size() / static_cast<std::size_t>(step) + 1U);
    for (std::size_t i = 0; i < points.size(); i += static_cast<std::size_t>(step))
    {
        projectionPoints.push_back({ points[i].x_px, points[i].y_px });
    }
    return projectionPoints;
}

template <typename PointT>
std::vector<PointT> sampledPoints(const std::vector<PointT>& points, int stride)
{
    const int step = positiveStride(stride);
    std::vector<PointT> sampled;
    sampled.reserve(points.size() / static_cast<std::size_t>(step) + 1U);
    for (std::size_t i = 0; i < points.size(); i += static_cast<std::size_t>(step))
    {
        sampled.push_back(points[i]);
    }
    return sampled;
}

#ifdef I2R_RLC_ENABLE_OPENCV
bool isPointInsideImage(
    const LocalizationLaserImagePoint_px& point,
    int width,
    int height)
{
    return point.x_px >= 0.0
        && point.y_px >= 0.0
        && point.x_px < static_cast<double>(width)
        && point.y_px < static_cast<double>(height);
}

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

cv::Rect makeRoi(
    const cv::Mat& gray,
    const LocalizationLaserPrior& prior)
{
    if (gray.empty())
    {
        return {};
    }

    if (prior.hasVirtualWeldPoint
        && isPointInsideImage(
            prior.virtualWeldPoint_px,
            gray.cols,
            gray.rows))
    {
        const int expand = std::max(1, prior.roiExpand_px);
        const int centerX = static_cast<int>(std::lround(
            prior.virtualWeldPoint_px.x_px));
        const int centerY = static_cast<int>(std::lround(
            prior.virtualWeldPoint_px.y_px));
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

    if (!prior.virtualLaserLine_px.empty())
    {
        double minX = static_cast<double>(gray.cols - 1);
        double maxX = 0.0;
        double minY = static_cast<double>(gray.rows - 1);
        double maxY = 0.0;
        int validCount = 0;
        for (const auto& point : prior.virtualLaserLine_px)
        {
            if (!isPointInsideImage(point, gray.cols, gray.rows))
            {
                continue;
            }
            minX = std::min(minX, point.x_px);
            maxX = std::max(maxX, point.x_px);
            minY = std::min(minY, point.y_px);
            maxY = std::max(maxY, point.y_px);
            ++validCount;
        }
        if (validCount > 0)
        {
            const int expand = std::max(1, prior.roiExpand_px / 4);
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

double estimateGaborTheta(const std::vector<LocalizationLaserImagePoint_px>& points)
{
    if (points.size() < 2U)
    {
        return 0.0;
    }

    double sumDx = 0.0;
    double sumDy = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const double dx = points[i].x_px - points[i - 1U].x_px;
        const double dy = points[i].y_px - points[i - 1U].y_px;
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

    const double lineAngle = std::atan2(sumDy, sumDx);
    return (kPi / 2.0) - lineAngle;
}

cv::Mat applyGabor(
    const cv::Mat& roiGray,
    const std::vector<LocalizationLaserImagePoint_px>& priorLine,
    const LocalizationCenterlineExtractionOptions& options)
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

std::vector<LocalizationLaserImagePoint_px> filterVirtualLineToRoi(
    const LocalizationLaserPrior& prior,
    const cv::Rect& roi)
{
    std::vector<LocalizationLaserImagePoint_px> filtered;
    for (const auto& point : prior.virtualLaserLine_px)
    {
        if (point.x_px >= static_cast<double>(roi.x)
            && point.x_px < static_cast<double>(roi.x + roi.width)
            && point.y_px >= static_cast<double>(roi.y)
            && point.y_px < static_cast<double>(roi.y + roi.height))
        {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<LocalizationLaserImagePoint_px> extractStegerCenterline(
    const cv::Mat& roiGray,
    const cv::Rect& roi,
    qint64 frameId,
    const LocalizationCenterlineExtractionOptions& options)
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

    std::vector<LocalizationLaserImagePoint_px> points;
    points.reserve(static_cast<std::size_t>(roiGray.cols * roiGray.rows / 16));
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
                points.push_back({
                    static_cast<double>(roi.x) + static_cast<double>(x) + t * nx,
                    static_cast<double>(roi.y) + static_cast<double>(y) + t * ny,
                    frameId,
                    1.0
                });
            }
        }
    }

    return points;
}
#endif

} // namespace

#ifdef I2R_RLC_ENABLE_OPENCV
LocalizationCenterlineExtractionResult LocalizationLaserLineExtractor::extractCenterline(
    const cv::Mat& image,
    const LocalizationLaserPrior& prior,
    const LocalizationCenterlineExtractionOptions& options)
{
    LocalizationCenterlineExtractionResult result;
    result.frameId = prior.frameId;

    const cv::Mat gray = toGray8(image);
    if (gray.empty())
    {
        result.failureReason = QStringLiteral("定位激光图为空或通道格式不支持。");
        return result;
    }

    const cv::Rect roi = makeRoi(gray, prior);
    if (roi.width <= 2 || roi.height <= 2)
    {
        result.failureReason = QStringLiteral("定位激光 ROI 无效。");
        return result;
    }

    result.roiX_px = roi.x;
    result.roiY_px = roi.y;
    result.roiWidth_px = roi.width;
    result.roiHeight_px = roi.height;
    result.roiVirtualLaserLine_px = filterVirtualLineToRoi(prior, roi);

    const cv::Mat roiGray = gray(roi).clone();
    const cv::Mat filtered = applyGabor(roiGray, result.roiVirtualLaserLine_px, options);
    result.centerline_px = extractStegerCenterline(
        filtered,
        roi,
        prior.frameId,
        options);
    if (result.centerline_px.empty())
    {
        result.failureReason = QStringLiteral("定位激光中心线提取为空。");
        return result;
    }

    result.succeeded = true;
    result.quality = std::min(
        1.0,
        static_cast<double>(result.centerline_px.size())
            / static_cast<double>(std::max(10, roi.width)));
    return result;
}
#endif

LocalizationPointCloudWindowSnapshot LocalizationLaserPointCloudWindow::appendFrame(
    const LocalizationPointCloudFrameRequest& request)
{
    if (!request.centerline.succeeded || request.centerline.centerline_px.empty())
    {
        LocalizationPointCloudWindowSnapshot current = snapshot();
        current.available = false;
        current.message = request.centerline.failureReason.isEmpty()
            ? QStringLiteral("定位中心线无效，未追加点云窗口。")
            : request.centerline.failureReason;
        return current;
    }

    ImageProjectionRequest projection = request.realProjection;
    projection.imagePoints_px = toProjectionPoints(
        request.centerline.centerline_px,
        request.options.realPointStride);

    FrameEntry frame;
    frame.frameId = request.frameId >= 0
        ? request.frameId
        : request.centerline.frameId;
    frame.realPoints_mm = LocalizationPointProjector::projectImagePoints(projection);
    frame.virtualPoints_mm = sampledPoints(
        request.virtualFramePoints_mm,
        request.options.virtualPointStride);

    m_frames.push_back(frame);
    const int maxFrames = std::max(1, request.options.slidingWindowFrameCount);
    while (m_frames.size() > static_cast<std::size_t>(maxFrames))
    {
        m_frames.pop_front();
    }
    if (request.options.maxPointCount > 0)
    {
        auto pointCount = [this]() {
            std::size_t count = 0;
            for (const FrameEntry& item : m_frames)
            {
                count += item.realPoints_mm.size();
                count += item.virtualPoints_mm.size();
            }
            return count;
        };
        while (!m_frames.empty()
            && pointCount() > static_cast<std::size_t>(request.options.maxPointCount))
        {
            m_frames.pop_front();
        }
    }

    LocalizationPointCloudWindowSnapshot current = snapshot();
    current.message = QStringLiteral("定位局部点云窗口已更新。");
    return current;
}

LocalizationPointCloudWindowSnapshot LocalizationLaserPointCloudWindow::snapshot() const
{
    LocalizationPointCloudWindowSnapshot result;
    result.windowFrameCount = static_cast<int>(m_frames.size());

    for (const FrameEntry& frame : m_frames)
    {
        result.realFramePointCounts.push_back(
            static_cast<int>(frame.realPoints_mm.size()));
        result.virtualFramePointCounts.push_back(
            static_cast<int>(frame.virtualPoints_mm.size()));
        result.realWindowPoints_mm.insert(
            result.realWindowPoints_mm.end(),
            frame.realPoints_mm.begin(),
            frame.realPoints_mm.end());
        result.virtualWindowPoints_mm.insert(
            result.virtualWindowPoints_mm.end(),
            frame.virtualPoints_mm.begin(),
            frame.virtualPoints_mm.end());
    }

    result.available = !result.realWindowPoints_mm.empty()
        && !result.virtualWindowPoints_mm.empty();
    result.message = result.available
        ? QStringLiteral("定位局部点云窗口可用。")
        : QStringLiteral("定位局部点云窗口尚无足够点。");
    return result;
}

void LocalizationLaserPointCloudWindow::clear()
{
    m_frames.clear();
}

} // namespace i2r_rlc::localization
