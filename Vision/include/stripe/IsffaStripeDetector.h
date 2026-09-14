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
 * @file        IsffaStripeDetector.h
 * @brief       ISFFA 激光条纹检测接口
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
#ifndef I2R_RLC_VISION_STRIPE_ISFFA_STRIPE_DETECTOR_H
#define I2R_RLC_VISION_STRIPE_ISFFA_STRIPE_DETECTOR_H

#include <opencv2/core.hpp>

#include "stripe/StripeDetectionTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief ISFFA 条纹中心线检测器。
 *
 * 对应旧 `LOGandISFFA -> ISFFA -> ISFFALink/FindROI` 路径。该类先提取
 * LOG/灰度中心线，再做分段拟合、短缺口连接和质量评价。
 */
class CIsffaStripeDetector final {
public:
    /**
     * @brief 从 `CV_8UC1` 灰度图提取 ISFFA 候选中心线。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 条纹检测选项，ROI 和距离单位均为 pixel。
     * @param state 可选跨帧状态；函数会更新 ROI/质量等状态，不持有该指针。
     * @return ISFFA 中心线检测结果。
     */
    static StripeCenterlineResult detect(
        const cv::Mat &gray8,
        const StripeDetectionOptions &options,
        StripeDetectionState *state);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_ISFFA_STRIPE_DETECTOR_H

