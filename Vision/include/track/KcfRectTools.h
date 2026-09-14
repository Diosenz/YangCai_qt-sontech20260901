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
 * @file        KcfRectTools.h
 * @brief       KCF 矩形与子窗口工具
 * @details     提供 `recttools.hpp` 对应的 ROI 限幅、边界补偿和灰度转换能力。
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
#ifndef I2R_RLC_VISION_TRACK_KCF_RECT_TOOLS_H
#define I2R_RLC_VISION_TRACK_KCF_RECT_TOOLS_H

#include <opencv2/core.hpp>

namespace i2r_rlc::vision::KcfRectTools {

/**
 * @brief 从图像中截取 KCF 搜索窗口。
 * @param image 输入图像，坐标单位为 pixel。
 * @param window 期望截取的窗口，允许部分越界。
 * @param borderType OpenCV 边界补偿方式，默认复制边界像素。
 * @return 与 `window.size()` 一致的子图；越界区域按 `borderType` 补齐。
 */
cv::Mat subwindow(
    const cv::Mat &image,
    const cv::Rect &window,
    int borderType = cv::BORDER_REPLICATE);

/**
 * @brief 将输入图像转换为 KCF 使用的灰度图。
 * @param image 输入灰度或 BGR/BGRA 图像。
 * @return 单通道灰度图；灰度输入会返回拷贝。
 */
cv::Mat getGrayImage(const cv::Mat &image);

/**
 * @brief 将 ROI 限制在图像边界内。
 * @param rect 待限幅矩形，坐标单位为 pixel。
 * @param width 图像宽度，单位 pixel。
 * @param height 图像高度，单位 pixel。
 * @return 与图像区域相交后的矩形；无交集时返回空矩形。
 */
cv::Rect clampRect(const cv::Rect &rect, int width, int height);

} // namespace i2r_rlc::vision::KcfRectTools

#endif // I2R_RLC_VISION_TRACK_KCF_RECT_TOOLS_H
