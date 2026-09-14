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
 * @file        GrooveEdgeDetector.h
 * @brief       坡口边缘特征检测接口
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
#ifndef I2R_RLC_VISION_FEATURE_GROOVE_EDGE_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_GROOVE_EDGE_DETECTOR_H

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "stripe/LaserCenterlineTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 坡口/复杂间隙边缘最小候选检测器。
 *
 * 本类迁移旧 ImagePro / GapDetector 中沿焊缝附近搜索左右 gap 边缘、
 * 结合点列线拟合判断坡口形态的低阶语义。本接口处理 CV_8UC1 图像、
 * 激光中心线和 2D 像素点列。
 */
class CGrooveEdgeDetector final {
public:
    /**
     * @brief 基于中心线多点采样检测坡口/复杂间隙左右边缘。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param centerline 激光中心线结果，坐标单位 pixel。
     * @param options 焊缝特征检测参数。
     * @return 坡口/复杂间隙边缘检测结果。
     */
    static GrooveEdgeResult detect(
        const cv::Mat &gray8,
        const LaserCenterlineResult &centerline,
        const WeldFeatureDetectionOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_GROOVE_EDGE_DETECTOR_H

