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
 * @file        ImageMorphologyUtils.cpp
 * @brief       图像形态学基础工具
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

#include "preprocess/ImageMorphologyUtils.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 写入形态学处理状态，允许调用方忽略状态输出。
 * @param status 可选输出状态指针，生命周期由调用方持有。
 * @param value 本次校验或处理状态。
 */
void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

/**
 * @brief 校验输入图像是否为非空 8 位单通道灰度图。
 * @param grayImage 输入图像。
 * @param status 可选输出状态指针。
 * @return 图像满足形态学工具输入约束时返回 true。
 */
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

/**
 * @brief 校验矩形结构元素尺寸。
 * @param kernelWidthPixel 结构元素宽度，单位为 pixel。
 * @param kernelHeightPixel 结构元素高度，单位为 pixel。
 * @param status 可选输出状态指针。
 * @return 宽高均为正数时返回 true。
 */
bool validateKernel(int kernelWidthPixel,
                    int kernelHeightPixel,
                    VisionPreprocessStatus *status)
{
    if (kernelWidthPixel <= 0 || kernelHeightPixel <= 0)
    {
        setStatus(status, VisionPreprocessStatus::InvalidKernel);
        return false;
    }

    return true;
}

/**
 * @brief 创建 OpenCV 矩形形态学结构元素。
 * @param kernelWidthPixel 结构元素宽度，单位为 pixel。
 * @param kernelHeightPixel 结构元素高度，单位为 pixel。
 * @return CV_8UC1 矩形 kernel。
 */
cv::Mat makeRectKernel(int kernelWidthPixel, int kernelHeightPixel)
{
    return cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernelWidthPixel, kernelHeightPixel));
}

/**
 * @brief 执行通用 morphologyEx 操作并统一状态处理。
 * @param grayImage 输入灰度图，要求 CV_8UC1。
 * @param kernelWidthPixel 结构元素宽度，单位为 pixel。
 * @param kernelHeightPixel 结构元素高度，单位为 pixel。
 * @param operation OpenCV 形态学操作枚举值。
 * @param status 可选输出状态指针。
 * @return 处理后的灰度图；输入非法时返回空 Mat。
 */
cv::Mat applyMorphology(const cv::Mat &grayImage,
                        int kernelWidthPixel,
                        int kernelHeightPixel,
                        int operation,
                        VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || !validateKernel(kernelWidthPixel, kernelHeightPixel, status))
    {
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::morphologyEx(grayImage,
                     outputImage,
                     operation,
                     makeRectKernel(kernelWidthPixel, kernelHeightPixel));
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

} // namespace

cv::Mat CImageMorphologyUtils::morphologyOpen(
    const cv::Mat &grayImage,
    int kernelWidthPixel,
    int kernelHeightPixel,
    VisionPreprocessStatus *status)
{
    return applyMorphology(grayImage,
                           kernelWidthPixel,
                           kernelHeightPixel,
                           cv::MORPH_OPEN,
                           status);
}

cv::Mat CImageMorphologyUtils::morphologyClose(
    const cv::Mat &grayImage,
    int kernelWidthPixel,
    int kernelHeightPixel,
    VisionPreprocessStatus *status)
{
    return applyMorphology(grayImage,
                           kernelWidthPixel,
                           kernelHeightPixel,
                           cv::MORPH_CLOSE,
                           status);
}

cv::Mat CImageMorphologyUtils::erodeGray(const cv::Mat &grayImage,
                                        int kernelWidthPixel,
                                        int kernelHeightPixel,
                                        VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || !validateKernel(kernelWidthPixel, kernelHeightPixel, status))
    {
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::erode(grayImage,
              outputImage,
              makeRectKernel(kernelWidthPixel, kernelHeightPixel));
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

cv::Mat CImageMorphologyUtils::dilateGray(const cv::Mat &grayImage,
                                         int kernelWidthPixel,
                                         int kernelHeightPixel,
                                         VisionPreprocessStatus *status)
{
    if (!validateGrayImage(grayImage, status)
        || !validateKernel(kernelWidthPixel, kernelHeightPixel, status))
    {
        return cv::Mat();
    }

    cv::Mat outputImage;
    cv::dilate(grayImage,
               outputImage,
               makeRectKernel(kernelWidthPixel, kernelHeightPixel));
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

cv::Mat CImageMorphologyUtils::removeSmallComponents(
    const cv::Mat &binaryImage,
    int minAreaPixel,
    VisionPreprocessStatus *status)
{
    if (!validateGrayImage(binaryImage, status))
    {
        return cv::Mat();
    }

    if (minAreaPixel <= 0)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return cv::Mat();
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(
        binaryImage,
        labels,
        stats,
        centroids,
        8,
        CV_32S);

    cv::Mat outputImage = cv::Mat::zeros(binaryImage.size(), CV_8UC1);
    for (int label = 1; label < labelCount; ++label)
    {
        const int areaPixel = stats.at<int>(label, cv::CC_STAT_AREA);
        if (areaPixel >= minAreaPixel)
        {
            outputImage.setTo(255, labels == label);
        }
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return outputImage;
}

} // namespace i2r_rlc::vision
