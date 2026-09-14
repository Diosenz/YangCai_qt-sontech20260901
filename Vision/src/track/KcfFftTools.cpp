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
 * @file        KcfFftTools.cpp
 * @brief       KCF 频域运算工具实现
 * @details     使用 OpenCV DFT 承接旧 KCF 的频域复数运算。
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
#include <vector>

#include <opencv2/imgproc.hpp>

#include "track/KcfFftTools.h"

namespace i2r_rlc::vision::KcfFftTools {

cv::Mat fftd(const cv::Mat &image, bool backwards)
{
    cv::Mat floatImage;
    image.convertTo(floatImage, CV_32F);
    cv::Mat output;
    const int flags = backwards ? (cv::DFT_INVERSE | cv::DFT_SCALE) : 0;
    if (floatImage.channels() == 1)
    {
        cv::dft(floatImage, output, flags | cv::DFT_COMPLEX_OUTPUT);
    }
    else
    {
        cv::dft(floatImage, output, flags);
    }
    return output;
}

cv::Mat real(const cv::Mat &image)
{
    if (image.channels() == 1)
    {
        return image.clone();
    }
    std::vector<cv::Mat> planes;
    cv::split(image, planes);
    return planes[0];
}

cv::Mat imag(const cv::Mat &image)
{
    if (image.channels() == 1)
    {
        return cv::Mat::zeros(image.size(), CV_32F);
    }
    std::vector<cv::Mat> planes;
    cv::split(image, planes);
    return planes[1];
}

cv::Mat complexMultiplication(const cv::Mat &left, const cv::Mat &right)
{
    cv::Mat output;
    cv::mulSpectrums(left, right, output, 0, false);
    return output;
}

cv::Mat complexDivision(const cv::Mat &left, const cv::Mat &right)
{
    std::vector<cv::Mat> leftPlanes;
    std::vector<cv::Mat> rightPlanes;
    cv::split(left, leftPlanes);
    cv::split(right, rightPlanes);

    cv::Mat denominator =
        rightPlanes[0].mul(rightPlanes[0]) + rightPlanes[1].mul(rightPlanes[1]);
    denominator += 1e-7F;

    std::vector<cv::Mat> outputPlanes(2);
    outputPlanes[0] =
        (leftPlanes[0].mul(rightPlanes[0]) + leftPlanes[1].mul(rightPlanes[1]))
        / denominator;
    outputPlanes[1] =
        (leftPlanes[1].mul(rightPlanes[0]) - leftPlanes[0].mul(rightPlanes[1]))
        / denominator;

    cv::Mat output;
    cv::merge(outputPlanes, output);
    return output;
}

cv::Mat complexConj(const cv::Mat &value)
{
    if (value.channels() == 1)
    {
        return value.clone();
    }
    std::vector<cv::Mat> planes;
    cv::split(value, planes);
    planes[1] = -planes[1];
    cv::Mat output;
    cv::merge(planes, output);
    return output;
}

void rearrange(cv::Mat &image)
{
    const int centerX = image.cols / 2;
    const int centerY = image.rows / 2;
    cv::Mat q0(image, cv::Rect(0, 0, centerX, centerY));
    cv::Mat q1(image, cv::Rect(centerX, 0, centerX, centerY));
    cv::Mat q2(image, cv::Rect(0, centerY, centerX, centerY));
    cv::Mat q3(image, cv::Rect(centerX, centerY, centerX, centerY));

    cv::Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}

} // namespace i2r_rlc::vision::KcfFftTools
