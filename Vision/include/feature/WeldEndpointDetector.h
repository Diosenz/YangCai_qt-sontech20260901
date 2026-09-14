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
 * @file        WeldEndpointDetector.h
 * @brief       焊缝端点检测接口
 * @details     提供 T 型、对接、搭接、gap、端点和仿真激光定位等焊缝语义检测能力。
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
#ifndef I2R_RLC_VISION_FEATURE_WELD_ENDPOINT_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_WELD_ENDPOINT_DETECTOR_H

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "stripe/LaserCenterlineTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 焊缝起点/终点最小候选检测器。
 *
 * 本类只从中心线有效点列和当前 feature 结果中提取 2D 起止点候选。
 * 
 */
class CWeldEndpointDetector final {
public:
    /**
     * @brief 根据灰度图、中心线和已有焊接特征结果估计起点/终点候选。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param centerline 激光中心线结果，坐标单位 pixel。
     * @param featureResult 已有焊缝特征结果。
     * @param options 焊缝特征检测参数。
     * @return 起点/终点候选结果。
     */
    static WeldEndpointResult detect(
        const cv::Mat &gray8,
        const LaserCenterlineResult &centerline,
        const WeldFeatureResult &featureResult,
        const WeldFeatureDetectionOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_WELD_ENDPOINT_DETECTOR_H

