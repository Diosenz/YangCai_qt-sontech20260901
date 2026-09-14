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
 * @file        StegerStripeDetector.h
 * @brief       Steger 风格条纹检测接口
 * @details     提供激光条纹、骨架、中心线和主动轮廓曲线提取能力，不承担焊缝语义分类。
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
#ifndef I2R_RLC_VISION_STRIPE_STEGER_STRIPE_DETECTOR_H
#define I2R_RLC_VISION_STRIPE_STEGER_STRIPE_DETECTOR_H

#include <opencv2/core.hpp>

#include "stripe/StripeDetectionTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief Steger-like 亚像素条纹中心线检测器。
 *
 * 旧工程依赖 HALCON `lines_gauss`。新实现使用 OpenCV 高斯平滑、
 * Hessian 二阶响应和一维抛物线亚像素修正，目标是承接 type=2 的工程
 * 入口，不声明与 HALCON `lines_gauss` 分数或像素级完全一致。
 */
class CStegerStripeDetector final {
public:
    /**
     * @brief 从 `CV_8UC1` 灰度图提取 Steger-like 中心线。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 条纹检测选项，ROI 和距离单位均为 pixel。
     * @param state 可选跨帧状态；函数会更新 ROI/质量等状态，不持有该指针。
     * @return Steger-like 中心线检测结果。
     */
    static StripeCenterlineResult detect(
        const cv::Mat &gray8,
        const StripeDetectionOptions &options,
        StripeDetectionState *state);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_STEGER_STRIPE_DETECTOR_H

