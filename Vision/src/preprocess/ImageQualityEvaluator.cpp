/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        ImageQualityEvaluator.cpp
 * @brief       图像质量评价实现
 * @details     提供灰度图像校验、增强、形态学和质量评价能力，供条纹、特征和管线模块复用。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "preprocess/ImageQualityEvaluator.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 判断图像是否为质量评价支持的灰度图。
 * @param gray8 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 */
bool isSupportedGray(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 根据选项选择评价 ROI。
 * @param gray8 输入 `CV_8UC1` 灰度图。
 * @param options ROI 选项。
 * @return ROI 视图；ROI 非法时返回空 Mat。
 */
cv::Mat selectRoi(const cv::Mat &gray8, const ImageQualityOptions &options)
{
    if (!options.useRoi)
    {
        return gray8;
    }
    const cv::Rect roi = options.roi & cv::Rect(0, 0, gray8.cols, gray8.rows);
    if (roi.empty())
    {
        return cv::Mat();
    }
    return gray8(roi);
}

/**
 * @brief 计算灰度标准差。
 * @param gray8 输入 `CV_8UC1` 灰度图或 ROI。
 * @return 标准差，单位 gray value。
 */
double calculateDeviation(const cv::Mat &gray8)
{
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray8, mean, stddev);
    return stddev[0];
}

double calculateLaplace(const cv::Mat &gray8)
{
    cv::Mat laplace;
    cv::Laplacian(gray8, laplace, CV_64F, 3);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplace, mean, stddev);
    return stddev[0] * stddev[0];
}

double calculateEnergy(const cv::Mat &gray8)
{
    double energy = 0.0;
    for (int y = 0; y + 1 < gray8.rows; ++y)
    {
        for (int x = 0; x + 1 < gray8.cols; ++x)
        {
            const double dx = static_cast<double>(gray8.at<unsigned char>(y, x + 1))
                - static_cast<double>(gray8.at<unsigned char>(y, x));
            const double dy = static_cast<double>(gray8.at<unsigned char>(y + 1, x))
                - static_cast<double>(gray8.at<unsigned char>(y, x));
            energy += dx * dx + dy * dy;
        }
    }
    return energy / static_cast<double>(std::max(1, gray8.rows * gray8.cols));
}

double calculateBrenner(const cv::Mat &gray8)
{
    double brenner = 0.0;
    for (int y = 0; y < gray8.rows; ++y)
    {
        for (int x = 0; x + 2 < gray8.cols; ++x)
        {
            const double diff =
                static_cast<double>(gray8.at<unsigned char>(y, x + 2))
                - static_cast<double>(gray8.at<unsigned char>(y, x));
            brenner += diff * diff;
        }
    }
    return brenner / static_cast<double>(std::max(1, gray8.rows * gray8.cols));
}

double calculateTenegrad(const cv::Mat &gray8)
{
    cv::Mat gradX;
    cv::Mat gradY;
    cv::Sobel(gray8, gradX, CV_64F, 1, 0, 3);
    cv::Sobel(gray8, gradY, CV_64F, 0, 1, 3);
    double sum = 0.0;
    for (int y = 0; y < gray8.rows; ++y)
    {
        const double *gx = gradX.ptr<double>(y);
        const double *gy = gradY.ptr<double>(y);
        for (int x = 0; x < gray8.cols; ++x)
        {
            sum += gx[x] * gx[x] + gy[x] * gy[x];
        }
    }
    return sum / static_cast<double>(std::max(1, gray8.rows * gray8.cols));
}

} // namespace

ImageQualityResult CImageQualityEvaluator::evaluate(
    const cv::Mat &gray8,
    ImageQualityMetric metric,
    const ImageQualityOptions &options)
{
    ImageQualityResult result;
    result.metric = metric;
    if (!isSupportedGray(gray8))
    {
        result.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return result;
    }

    const cv::Mat roiImage = selectRoi(gray8, options);
    if (roiImage.empty())
    {
        result.message = "invalid image quality roi";
        return result;
    }

    switch (metric)
    {
    case ImageQualityMetric::Deviation:
        result.value = calculateDeviation(roiImage);
        break;
    case ImageQualityMetric::Laplace:
        result.value = calculateLaplace(roiImage);
        break;
    case ImageQualityMetric::Energy:
        result.value = calculateEnergy(roiImage);
        break;
    case ImageQualityMetric::Brenner:
        result.value = calculateBrenner(roiImage);
        break;
    case ImageQualityMetric::Tenegrad:
        result.value = calculateTenegrad(roiImage);
        break;
    }

    result.valid = true;
    result.message = "image quality metric evaluated";
    return result;
}

std::map<ImageQualityMetric, ImageQualityResult> CImageQualityEvaluator::evaluateAll(
    const cv::Mat &gray8,
    const ImageQualityOptions &options)
{
    std::map<ImageQualityMetric, ImageQualityResult> results;
    const ImageQualityMetric metrics[] = {
        ImageQualityMetric::Deviation,
        ImageQualityMetric::Laplace,
        ImageQualityMetric::Energy,
        ImageQualityMetric::Brenner,
        ImageQualityMetric::Tenegrad};
    for (ImageQualityMetric metric : metrics)
    {
        results.emplace(metric, evaluate(gray8, metric, options));
    }
    return results;
}

std::string CImageQualityEvaluator::metricToKey(ImageQualityMetric metric)
{
    switch (metric)
    {
    case ImageQualityMetric::Deviation:
        return "Deviation";
    case ImageQualityMetric::Laplace:
        return "Laplace";
    case ImageQualityMetric::Energy:
        return "Energy";
    case ImageQualityMetric::Brenner:
        return "Brenner";
    case ImageQualityMetric::Tenegrad:
        return "Tenegrad";
    }
    return "Unknown";
}

} // namespace i2r_rlc::vision
