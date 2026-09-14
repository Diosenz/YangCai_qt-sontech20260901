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
 * @file        KcfFftTools.h
 * @brief       KCF 频域运算工具
 * @details     提供 OpenCV DFT、复数乘除和频谱重排能力。
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
#ifndef I2R_RLC_VISION_TRACK_KCF_FFT_TOOLS_H
#define I2R_RLC_VISION_TRACK_KCF_FFT_TOOLS_H

#include <opencv2/core.hpp>

namespace i2r_rlc::vision::KcfFftTools {

/**
 * @brief 对输入图像执行二维 DFT 或反 DFT。
 * @param image 输入单通道实数图或双通道复数图，像素值类型会转换为 `CV_32F`。
 * @param backwards 为 `true` 时执行带缩放的反变换。
 * @return 频域或空域结果；正变换的单通道输入返回 `CV_32FC2`。
 */
cv::Mat fftd(const cv::Mat &image, bool backwards = false);

/**
 * @brief 提取复数矩阵的实部。
 * @param image 单通道实数矩阵或双通道复数矩阵。
 * @return 与输入同尺寸的单通道实部矩阵。
 */
cv::Mat real(const cv::Mat &image);

/**
 * @brief 提取复数矩阵的虚部。
 * @param image 单通道实数矩阵或双通道复数矩阵。
 * @return 与输入同尺寸的单通道虚部矩阵；单通道输入返回零矩阵。
 */
cv::Mat imag(const cv::Mat &image);

/**
 * @brief 执行逐频点复数乘法。
 * @param left 左操作数，通常为 `CV_32FC2` 频域响应。
 * @param right 右操作数，尺寸和通道数需与 `left` 一致。
 * @return `left * right` 的复数结果。
 */
cv::Mat complexMultiplication(const cv::Mat &left, const cv::Mat &right);

/**
 * @brief 执行逐频点复数除法。
 * @param left 分子，通常为 KCF 的频域模板或响应。
 * @param right 分母，尺寸和通道数需与 `left` 一致。
 * @return `left / right` 的复数结果；内部使用小量避免除零。
 */
cv::Mat complexDivision(const cv::Mat &left, const cv::Mat &right);

/**
 * @brief 计算复数矩阵共轭。
 * @param value 单通道实数矩阵或双通道复数矩阵。
 * @return 与输入同尺寸的共轭矩阵。
 */
cv::Mat complexConj(const cv::Mat &value);

/**
 * @brief 将频谱象限交换到以零频为中心的显示布局。
 * @param image 输入输出矩阵，宽高应为偶数；函数会就地交换四个象限。
 */
void rearrange(cv::Mat &image);

} // namespace i2r_rlc::vision::KcfFftTools

#endif // I2R_RLC_VISION_TRACK_KCF_FFT_TOOLS_H
