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
 * @file        ImageDenoise.cpp
 * @brief       图像去噪与增强实现
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

#include "preprocess/ImageDenoise.h"

namespace i2r_rlc::vision {

namespace {

constexpr double kPi = 3.14159265358979323846;

void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

bool validateGrayImage(const cv::Mat &grayImage, VisionPreprocessStatus *status)
{
    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return false;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return false;
    }

    return true;
}

bool isPositiveOdd(int value)
{
    return value > 0 && (value % 2) == 1;
}

bool validateOddKernel(int kernelSizePixel, VisionPreprocessStatus *status)
{
    if (!isPositiveOdd(kernelSizePixel))
    {
        setStatus(status, VisionPreprocessStatus::InvalidKernel);
        return false;
    }

    return true;
}

bool validatePositive(double value, VisionPreprocessStatus *status)
{
    if (value <= 0.0)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return false;
    }

    return true;
}

bool validateGaborOptions(const GaborDenoiseOptions &options,
                          VisionPreprocessStatus *status)
{
    if (!isPositiveOdd(options.kernelWidthPixel)
        || !isPositiveOdd(options.kernelHeightPixel))
    {
        setStatus(status, VisionPreprocessStatus::InvalidKernel);
        return false;
    }

    return validatePositive(options.sigma, status)
        && validatePositive(options.lambdaPixel, status)
        && validatePositive(options.gamma, status);
}

GaborFilterOptions &mutableGaborFilterOptions()
{
    // 旧函数参数原本是函数内固定值。这里集中保存，满足测试和下游调参需要。
    static GaborFilterOptions options;
    return options;
}

bool validateGaborFilterOptions(const GaborFilterOptions &options,
                                VisionPreprocessStatus *status)
{
    // 旧 Process_P 中 lambda 不是直接使用 pos_lm，而是先做百分值转换。
    const double lambda = 0.5 + options.posLm / 100.0;
    if (!isPositiveOdd(options.kernelSizePixel))
    {
        setStatus(status, VisionPreprocessStatus::InvalidKernel);
        return false;
    }

    if (options.posSigma <= 0.0
        || lambda <= 0.0
        || options.neighborWindow <= 0
        || options.denominatorEpsilon <= 0.0)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return false;
    }

    return true;
}

VisionPreprocessStatus statusFromAngleFailure(
    const GaborFilterAngleResult &angleResult)
{
    if (angleResult.message == "degenerate gabor angle geometry")
    {
        return VisionPreprocessStatus::DegenerateGeometry;
    }

    return VisionPreprocessStatus::InsufficientPoints;
}

} // namespace

cv::Mat CImageDenoise::medianDenoise(const cv::Mat &grayImage,
                                    int kernelSizePixel,
                                    VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || !validateOddKernel(kernelSizePixel, status))
    {
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::medianBlur(grayImage, outputImage, kernelSizePixel);
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

cv::Mat CImageDenoise::gaussianDenoise(const cv::Mat &grayImage,
                                      int kernelSizePixel,
                                      double sigma,
                                      VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || !validateOddKernel(kernelSizePixel, status)
        || !validatePositive(sigma, status))
    {
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::GaussianBlur(grayImage,
                     outputImage,
                     cv::Size(kernelSizePixel, kernelSizePixel),
                     sigma);
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

cv::Mat CImageDenoise::bilateralDenoise(const cv::Mat &grayImage,
                                       int diameterPixel,
                                       double sigmaColor,
                                       double sigmaSpace,
                                       VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || diameterPixel <= 0
        || !validatePositive(sigmaColor, status)
        || !validatePositive(sigmaSpace, status))
    {
        if (diameterPixel <= 0)
        {
            setStatus(status, VisionPreprocessStatus::InvalidKernel);
        }
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::bilateralFilter(grayImage,
                        outputImage,
                        diameterPixel,
                        sigmaColor,
                        sigmaSpace);
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

cv::Mat CImageDenoise::gaborDenoise(const cv::Mat &grayImage,
                                   const GaborDenoiseOptions &options,
                                   VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || !validateGaborOptions(options, status))
    {
        return cv::Mat();
    }

    const cv::Mat kernel = cv::getGaborKernel(
        cv::Size(options.kernelWidthPixel, options.kernelHeightPixel),
        options.sigma,
        options.thetaRad,
        options.lambdaPixel,
        options.gamma,
        options.psiRad,
        CV_32F);

    cv::Mat outputImage;
    cv::filter2D(grayImage, outputImage, -1, kernel);
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

GaborFilterOptions CImageDenoise::gaborFilterOptions()
{
    return mutableGaborFilterOptions();
}

void CImageDenoise::setGaborFilterOptions(const GaborFilterOptions &options)
{
    mutableGaborFilterOptions() = options;
}

GaborFilterAngleResult CImageDenoise::estimateGaborFilterAngle(
    const std::vector<MyLaserImagePoint> &simPoints,
    const MyLaserImagePoint &weldPoint)
{
    GaborFilterAngleResult result;
    const GaborFilterOptions options = gaborFilterOptions();
    if (simPoints.empty())
    {
        result.message = "empty simulated points";
        return result;
    }

    // 只按 X 方向距离寻找焊点索引，不使用二维欧氏距离。
    double minXDistance = std::numeric_limits<double>::max();
    int weldPointIndex = -1;
    for (int i = 0; i < static_cast<int>(simPoints.size()); ++i)
    {
        const double xDistance = std::fabs(simPoints[i].px - weldPoint.px);
        if (xDistance < minXDistance)
        {
            minXDistance = xDistance;
            weldPointIndex = i;
        }
    }

    if (weldPointIndex < 0)
    {
        result.message = "weld point index not found";
        return result;
    }

    // 左侧和右侧各取 neighborWindow 内的平均点，用于估计焊缝局部夹角。
    double leftPointX = 0.0;
    double leftPointY = 0.0;
    int pointCount = 0;
    const int leftBegin = std::max(0, weldPointIndex - options.neighborWindow);
    for (int i = leftBegin; i < weldPointIndex; ++i)
    {
        leftPointX += simPoints[i].px;
        leftPointY += simPoints[i].py;
        ++pointCount;
    }

    if (pointCount == 0)
    {
        result.weldPointIndex = weldPointIndex;
        result.message = "missing left neighbor points";
        return result;
    }

    leftPointX /= static_cast<double>(pointCount);
    leftPointY /= static_cast<double>(pointCount);

    double rightPointX = 0.0;
    double rightPointY = 0.0;
    pointCount = 0;
    const int rightBegin = std::min(
        static_cast<int>(simPoints.size()) - 1,
        weldPointIndex + options.neighborWindow);
    for (int i = rightBegin; i > weldPointIndex; --i)
    {
        rightPointX += simPoints[i].px;
        rightPointY += simPoints[i].py;
        ++pointCount;
    }

    if (pointCount == 0)
    {
        result.weldPointIndex = weldPointIndex;
        result.message = "missing right neighbor points";
        return result;
    }

    rightPointX /= static_cast<double>(pointCount);
    rightPointY /= static_cast<double>(pointCount);

    // 旧公式只在左右点与焊点完全重合时失败；单独的垂直方向会通过
    // `atan(dy / dx)` 进入无穷比值路径。
    const double leftDx = weldPoint.px - leftPointX;
    const double leftDy = weldPoint.py - leftPointY;
    const double rightDx = rightPointX - weldPoint.px;
    const double rightDy = weldPoint.py - rightPointY;
    if ((std::fabs(leftDx) <= options.denominatorEpsilon
            && std::fabs(leftDy) <= options.denominatorEpsilon)
        || (std::fabs(rightDx) <= options.denominatorEpsilon
            && std::fabs(rightDy) <= options.denominatorEpsilon))
    {
        result.weldPointIndex = weldPointIndex;
        result.message = "degenerate gabor angle geometry";
        return result;
    }

    // 与旧 GaborFilter 保持一致：直接使用 atan(dy / dx)，不使用 atan2。
    const double leftAngle =
        std::atan(leftDy / leftDx) / kPi * 180.0;
    const double rightAngle =
        std::atan(rightDy / rightDx) / kPi * 180.0;

    //  FilterAngle = LeftAngle + (180 - RightAngle - LeftAngle) / 2 。
    result.valid = true;
    result.filterAngleDeg =
        leftAngle + (180.0 - rightAngle - leftAngle) / 2.0;
    result.weldPointIndex = weldPointIndex;
    result.message = "gabor filter angle estimated";
    return result;
}

cv::Mat CImageDenoise::makeGaborFilterKernel(
    double filterAngleDeg,
    const GaborFilterOptions &options,
    VisionPreprocessStatus *status)
{
    if (!validateGaborFilterOptions(options, status))
    {
        return cv::Mat();
    }

    // 下列变量名按旧 mkKernel 语义展开。
    const int kernelSize = options.kernelSizePixel;
    const int halfKernelSize = (kernelSize - 1) / 2;
    const double theta = filterAngleDeg * kPi / 180.0;
    const double psi = options.posPsiDeg * kPi / 180.0;
    const double step = 2.0 / static_cast<double>(kernelSize - 1);
    const double lambda = 0.5 + options.posLm / 100.0;
    const double sigma =
        options.posSigma / static_cast<double>(kernelSize);

    cv::Mat kernel(kernelSize, kernelSize, CV_32F);
    for (int y = -halfKernelSize; y <= halfKernelSize; ++y)
    {
        for (int x = -halfKernelSize; x <= halfKernelSize; ++x)
        {
            // 先把 kernel 坐标旋转到滤波方向坐标系，再计算高斯包络和余弦波。
            const double xTheta =
                x * step * std::cos(theta) + y * step * std::sin(theta);
            const double yTheta =
                -x * step * std::sin(theta) + y * step * std::cos(theta);
            const double envelope = std::exp(
                -0.5 * (xTheta * xTheta + yTheta * yTheta)
                / (sigma * sigma));
            const double wave =
                std::cos(2.0 * kPi * xTheta / lambda + psi);
            kernel.at<float>(halfKernelSize + y, halfKernelSize + x) =
                static_cast<float>(envelope * wave);
        }
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return kernel;
}

cv::Mat CImageDenoise::gaborDenoise(
    const cv::Mat &grayImage,
    const std::vector<MyLaserImagePoint> &simPoints,
    const MyLaserImagePoint &weldPoint,
    VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status))
    {
        return cv::Mat();
    }

    // 该重载使用旧工程固定参数路径；显式参数版本仍保留为通用 Gabor 工具。
    const GaborFilterOptions options = gaborFilterOptions();
    if (!validateGaborFilterOptions(options, status))
    {
        return cv::Mat();
    }

    const GaborFilterAngleResult angleResult =
        estimateGaborFilterAngle(simPoints, weldPoint);
    if (!angleResult.valid)
    {
        setStatus(status, statusFromAngleFailure(angleResult));
        return cv::Mat();
    }

    const cv::Mat kernel = makeGaborFilterKernel(
        angleResult.filterAngleDeg,
        options,
        status);
    if (kernel.empty())
    {
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::filter2D(grayImage, outputImage, CV_8U, kernel);
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

} // namespace i2r_rlc::vision

