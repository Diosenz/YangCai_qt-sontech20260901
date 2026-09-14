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
 * @file        ImagePreprocessor.cpp
 * @brief       图像预处理实现
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
#include <opencv2/imgproc.hpp>

#include "preprocess/ImagePreprocessor.h"

namespace i2r_rlc::vision {

namespace {

void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

bool isSingleChannelNormalizable(int type)
{
    return type == CV_8UC1
        || type == CV_16UC1
        || type == CV_32FC1;
}

} // namespace

bool hasPositiveArea(const VisionImageRoi &roi)
{
    return roi.widthPixel > 0 && roi.heightPixel > 0;
}

cv::Rect toCvRect(const VisionImageRoi &roi)
{
    return cv::Rect(roi.xPixel, roi.yPixel, roi.widthPixel, roi.heightPixel);
}

bool CImagePreprocessor::isSupportedInput(const cv::Mat &image)
{
    if (image.empty())
    {
        return false;
    }

    const int type = image.type();
    return type == CV_8UC1
        || type == CV_8UC3
        || type == CV_8UC4
        || type == CV_16UC1
        || type == CV_32FC1;
}

bool CImagePreprocessor::isValidRoi(const cv::Mat &image,
                                   const VisionImageRoi &roi)
{
    if (image.empty() || !hasPositiveArea(roi))
    {
        return false;
    }

    if (roi.xPixel < 0 || roi.yPixel < 0)
    {
        return false;
    }

    if (roi.xPixel > image.cols || roi.yPixel > image.rows)
    {
        return false;
    }

    return roi.widthPixel <= image.cols - roi.xPixel
        && roi.heightPixel <= image.rows - roi.yPixel;
}

cv::Mat CImagePreprocessor::cropRoi(const cv::Mat &image,
                                   const VisionImageRoi &roi,
                                   VisionPreprocessStatus *status)
{
    if (image.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return cv::Mat();
    }

    if (!isValidRoi(image, roi))
    {
        setStatus(status, VisionPreprocessStatus::InvalidRoi);
        return cv::Mat();
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return image(toCvRect(roi)).clone();
}

cv::Mat CImagePreprocessor::toGray8(const cv::Mat &image,
                                   VisionPreprocessStatus *status)
{
    if (image.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return cv::Mat();
    }

    if (!isSupportedInput(image))
    {
        setStatus(status, VisionPreprocessStatus::UnsupportedImageType);
        return cv::Mat();
    }

    if (image.type() == CV_8UC3)
    {
        cv::Mat grayImage;
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
        setStatus(status, VisionPreprocessStatus::Succeeded);
        return grayImage;
    }

    if (image.type() == CV_8UC4)
    {
        cv::Mat grayImage;
        cv::cvtColor(image, grayImage, cv::COLOR_BGRA2GRAY);
        setStatus(status, VisionPreprocessStatus::Succeeded);
        return grayImage;
    }

    return normalizeToGray8(image, status);
}

cv::Mat CImagePreprocessor::normalizeToGray8(const cv::Mat &image,
                                            VisionPreprocessStatus *status)
{
    if (image.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return cv::Mat();
    }

    if (!isSingleChannelNormalizable(image.type()))
    {
        setStatus(status, VisionPreprocessStatus::UnsupportedImageType);
        return cv::Mat();
    }

    if (image.type() == CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::Succeeded);
        return image.clone();
    }

    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(image, &minValue, &maxValue);

    cv::Mat grayImage;
    if (minValue == maxValue)
    {
        grayImage = cv::Mat::zeros(image.size(), CV_8UC1);
    }
    else
    {
        image.convertTo(grayImage,
                        CV_8UC1,
                        255.0 / (maxValue - minValue),
                        -minValue * 255.0 / (maxValue - minValue));
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return grayImage;
}

VisionGrayStatistics CImagePreprocessor::calculateGrayStatistics(
    const cv::Mat &grayImage,
    VisionPreprocessStatus *status)
{
    VisionGrayStatistics statistics;

    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return statistics;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return statistics;
    }

    cv::Scalar meanValue;
    cv::Scalar stddevValue;
    cv::meanStdDev(grayImage, meanValue, stddevValue);

    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(grayImage, &minValue, &maxValue);

    statistics.pixelCount = grayImage.rows * grayImage.cols;
    statistics.meanGrayValue = meanValue[0];
    statistics.minGrayValue = minValue;
    statistics.maxGrayValue = maxValue;

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return statistics;
}

} // namespace i2r_rlc::vision
