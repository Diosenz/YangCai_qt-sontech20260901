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
 * @file        DirectionalMorphologyFilter.cpp
 * @brief       方向形态学增强实现
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
#include <limits>

#include <opencv2/imgproc.hpp>

#include "preprocess/DirectionalMorphologyFilter.h"

namespace i2r_rlc::vision {

namespace {

/// @brief 圆周率常量，用于角度和弧度转换。
constexpr double PI_VALUE = 3.14159265358979323846;

/**
 * @brief 判断图像是否为方向形态学增强支持的灰度格式。
 * @param image 输入图像。
 * @return 图像非空且类型为 CV_8UC1 时返回 true。
 */
bool isValidGrayImage(const cv::Mat &image)
{
    return !image.empty() && image.type() == CV_8UC1;
}

/**
 * @brief 将结构元素尺寸修正为正奇数。
 * @param sizePixel 请求尺寸，单位为 pixel。
 * @return 不小于 1 的奇数尺寸。
 */
int makeOddKernelSize(int sizePixel)
{
    int size = std::max(1, sizePixel);
    if ((size % 2) == 0)
    {
        ++size;
    }
    return size;
}

/**
 * @brief 计算点到线段所在直线的垂直距离。
 * @param point 待测点，单位为 pixel。
 * @param start 直线起点，单位为 pixel。
 * @param end 直线终点，单位为 pixel。
 * @return 点到直线距离，单位为 pixel。
 */
double distanceToLine(
    const cv::Point2d &point,
    const cv::Point2d &start,
    const cv::Point2d &end)
{
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double denominator = std::sqrt(dx * dx + dy * dy);
    if (denominator <= std::numeric_limits<double>::epsilon())
    {
        return 0.0;
    }

    return std::abs(dy * point.x - dx * point.y + end.x * start.y
        - end.y * start.x) / denominator;
}

/**
 * @brief 根据源线段方向换算 OpenCV 线形 kernel 的角度。
 * @param start 源线起点，单位为 pixel。
 * @param end 源线终点，单位为 pixel。
 * @return kernel 角度，单位为 degree。
 */
double sourceLineKernelAngleDegree(
    const cv::Point2d &start,
    const cv::Point2d &end)
{
    return std::atan2(end.x - start.x, start.y - end.y)
        * 180.0 / PI_VALUE;
}

/**
 * @brief 将任意浮点响应图归一化为 8 位灰度图。
 * @param source 输入响应图。
 * @return CV_8UC1 归一化图像；常量图返回全零图。
 */
cv::Mat normalizeToByte(const cv::Mat &source)
{
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(source, &minValue, &maxValue);
    if (maxValue - minValue <= std::numeric_limits<double>::epsilon())
    {
        return cv::Mat::zeros(source.size(), CV_8UC1);
    }

    cv::Mat normalized;
    source.convertTo(
        normalized,
        CV_8UC1,
        255.0 / (maxValue - minValue),
        -255.0 * minValue / (maxValue - minValue));
    return normalized;
}

} // namespace

cv::Mat CDirectionalMorphologyFilter::createLineKernel(
    double angleDegree,
    int lengthPixel)
{
    // 1. 结构元素尺寸：根据方向角估计覆盖宽高，至少保留 1 pixel。
    if (lengthPixel <= 0)
    {
        return cv::Mat();
    }

    const double angleRad = angleDegree * PI_VALUE / 180.0;
    const double cosValue = std::cos(angleRad);
    const double sinValue = std::sin(angleRad);
    const int width = std::max(
        1,
        static_cast<int>(std::round(std::abs(lengthPixel * cosValue))));
    const int height = std::max(
        1,
        static_cast<int>(std::round(std::abs(lengthPixel * sinValue))));

    cv::Mat kernel = cv::Mat::zeros(height, width, CV_8UC1);
    // 2. 画线形 kernel：用 OpenCV 二值 Mat 替代 HALCON GenRegionLine。
    if (cosValue <= 0.0)
    {
        cv::line(
            kernel,
            cv::Point(0, height - 1),
            cv::Point(width - 1, 0),
            cv::Scalar(255),
            1,
            cv::LINE_8);
    }
    else
    {
        cv::line(
            kernel,
            cv::Point(width - 1, height - 1),
            cv::Point(0, 0),
            cv::Scalar(255),
            1,
            cv::LINE_8);
    }

    return kernel;
}

DirectionalMorphologyResult
CDirectionalMorphologyFilter::buildFromSimulatedLaser(
    const std::vector<cv::Point2d> &laserPoints,
    const DirectionalMorphologyOptions &options)
{
    DirectionalMorphologyResult result;
    // 1. 输入校验：仿真线点数不足时无法判断 T 型左右方向。
    if (static_cast<int>(laserPoints.size()) < options.minLaserPointCount)
    {
        result.message = "not enough simulated laser points";
        return result;
    }

    // 2. 分割点选择：沿用旧工程“点到首尾线最大距离”的弯折点语义。
    int splitIndex = 0;
    double maxDistance = -1.0;
    const cv::Point2d &start = laserPoints.front();
    const cv::Point2d &end = laserPoints.back();
    for (int index = 0; index < static_cast<int>(laserPoints.size()); ++index)
    {
        const double distance =
            distanceToLine(laserPoints[index], start, end);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            splitIndex = index;
        }
    }

    if (splitIndex <= 0
        || splitIndex >= static_cast<int>(laserPoints.size()) - 1)
    {
        splitIndex = static_cast<int>(laserPoints.size()) / 2;
    }

    // 3. 结构元素生成：左右两侧分别按仿真线方向构造线形腐蚀核。
    const double leftAngle = sourceLineKernelAngleDegree(
        laserPoints.front(),
        laserPoints[splitIndex]);
    const double rightAngle = sourceLineKernelAngleDegree(
        laserPoints[splitIndex + 1],
        laserPoints.back());

    result.leftKernel = createLineKernel(leftAngle, options.leftLengthPixel);
    result.rightKernel = createLineKernel(rightAngle, options.rightLengthPixel);
    if (result.leftKernel.empty() || result.rightKernel.empty())
    {
        result.message = "failed to create morphology kernels";
        return result;
    }

    result.splitPoint = laserPoints[splitIndex];
    result.splitPointIndex = splitIndex;
    result.valid = true;
    result.message = "directional kernels created";
    return result;
}

DirectionalMorphologyResult CDirectionalMorphologyFilter::enhance(
    const cv::Mat &grayImage,
    const std::vector<cv::Point2d> &laserPoints,
    const DirectionalMorphologyOptions &options)
{
    // 1. 根据仿真线生成左右结构元素和分割列。
    DirectionalMorphologyResult result =
        buildFromSimulatedLaser(laserPoints, options);
    if (!result.valid)
    {
        return result;
    }

    if (!isValidGrayImage(grayImage))
    {
        result.valid = false;
        result.enhancedImage.release();
        result.message = grayImage.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }

    // 2. 左右方向腐蚀：不同区域使用不同方向的线形 kernel。
    cv::Mat leftEroded;
    cv::Mat rightEroded;
    cv::erode(grayImage, leftEroded, result.leftKernel);
    cv::erode(grayImage, rightEroded, result.rightKernel);

    const int splitColumn = std::max(
        0,
        std::min(grayImage.cols - 1, static_cast<int>(
            std::round(result.splitPoint.x))));
    cv::Mat mergedErosion(grayImage.size(), CV_8UC1);
    // 3. 非均匀合成：分割列左侧取左腐蚀，右侧取右腐蚀。
    for (int row = 0; row < grayImage.rows; ++row)
    {
        unsigned char *mergedPtr = mergedErosion.ptr<unsigned char>(row);
        const unsigned char *leftPtr = leftEroded.ptr<unsigned char>(row);
        const unsigned char *rightPtr = rightEroded.ptr<unsigned char>(row);
        for (int col = 0; col < grayImage.cols; ++col)
        {
            mergedPtr[col] = col < splitColumn ? leftPtr[col] : rightPtr[col];
        }
    }

    // 4. 差分与归一化：突出比方向腐蚀结果更亮的激光纹理。
    cv::Mat source32;
    cv::Mat erosion32;
    grayImage.convertTo(source32, CV_32F);
    mergedErosion.convertTo(erosion32, CV_32F);
    cv::Mat difference = source32 - erosion32;
    result.enhancedImage = normalizeToByte(difference);

    // 5. 均值滤波：对应旧 MeanImage 平滑，降低骨架前的局部噪声。
    const int blurSize = makeOddKernelSize(options.blurSizePixel);
    if (blurSize >= 3)
    {
        cv::blur(
            result.enhancedImage,
            result.enhancedImage,
            cv::Size(blurSize, blurSize));
    }

    result.valid = true;
    result.message = "directional morphology enhanced";
    return result;
}

} // namespace i2r_rlc::vision
