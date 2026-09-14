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
 * @file        SamStripeDetector.h
 * @brief       SAM 激光条纹检测接口
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
#ifndef I2R_RLC_VISION_STRIPE_SAM_STRIPE_DETECTOR_H
#define I2R_RLC_VISION_STRIPE_SAM_STRIPE_DETECTOR_H

#include <opencv2/core.hpp>

#include "stripe/StripeDetectionTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief SAM 条纹中心线检测器。
 *
 * 对应旧 `LOGandSAM -> SAM -> SAMSplit/SAMMerge/SAMRemoveNFill` 路径。
 * 新实现复用 LOG/CoM、连续性分段和短缺口补线，不保留旧裸数组和全局状态。
 */
class CSamStripeDetector final {
public:
    /**
     * @brief 从 `CV_8UC1` 灰度图提取 SAM 候选中心线。
     */
    static StripeCenterlineResult detect(
        const cv::Mat &gray8,
        const StripeDetectionOptions &options,
        StripeDetectionState *state);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_SAM_STRIPE_DETECTOR_H

