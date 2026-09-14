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
 * @file        WeldFeatureDetector.h
 * @brief       焊缝特征聚合检测接口
 * @details     聚合 gap、端点和坡口边缘候选；T/Butt/Lap 语义由主动轮廓检测器承接。
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
#ifndef I2R_RLC_VISION_FEATURE_WELD_FEATURE_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_WELD_FEATURE_DETECTOR_H

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "stripe/LaserCenterlineTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 焊接特征检测薄封装。
 *
 * 当前组合 CGapFeatureDetector、CWeldEndpointDetector 和
 * CGrooveEdgeDetector，输出 gap/edge/weld point、起止点和坡口辅助候选。
 */
class CWeldFeatureDetector final {
public:
    /**
     * @brief 基于灰度图和中心线结果检测当前阶段支持的焊接特征。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param centerline 激光中心线结果，坐标单位 pixel。
     * @param options 焊缝特征检测参数。
     * @return 聚合后的焊缝特征结果。
     */
    static WeldFeatureResult detect(
        const cv::Mat &gray8,
        const LaserCenterlineResult &centerline,
        const WeldFeatureDetectionOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_WELD_FEATURE_DETECTOR_H

