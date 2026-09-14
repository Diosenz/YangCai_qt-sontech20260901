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
 * @file        KcfRectTools.cpp
 * @brief       KCF 矩形与子窗口工具实现
 * @details     使用 OpenCV ROI 与 copyMakeBorder 承接旧 KCF 子窗口截取语义。
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

#include <opencv2/imgproc.hpp>

#include "track/KcfRectTools.h"

namespace i2r_rlc::vision::KcfRectTools {

cv::Rect clampRect(const cv::Rect &rect, int width, int height)
{
    const int x = std::max(0, std::min(rect.x, width - 1));
    const int y = std::max(0, std::min(rect.y, height - 1));
    const int right = std::max(x + 1, std::min(rect.x + rect.width, width));
    const int bottom = std::max(y + 1, std::min(rect.y + rect.height, height));
    return cv::Rect(x, y, right - x, bottom - y);
}

cv::Mat subwindow(const cv::Mat &image, const cv::Rect &window, int borderType)
{
    const int left = std::max(0, -window.x);
    const int top = std::max(0, -window.y);
    const int right = std::max(0, window.x + window.width - image.cols);
    const int bottom = std::max(0, window.y + window.height - image.rows);

    cv::Mat bordered;
    cv::copyMakeBorder(image, bordered, top, bottom, left, right, borderType);
    const cv::Rect shifted(window.x + left, window.y + top, window.width, window.height);
    return bordered(shifted).clone();
}

cv::Mat getGrayImage(const cv::Mat &image)
{
    cv::Mat gray;
    if (image.channels() == 1)
    {
        gray = image.clone();
    }
    else
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    gray.convertTo(gray, CV_32F, 1.0 / 255.0);
    return gray;
}

} // namespace i2r_rlc::vision::KcfRectTools
