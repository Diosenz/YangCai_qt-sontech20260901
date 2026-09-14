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
 * @file        WeldFeatureDetector.cpp
 * @brief       焊缝特征聚合检测实现
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
#include <algorithm>

#include "feature/GapFeatureDetector.h"
#include "feature/GrooveEdgeDetector.h"
#include "feature/WeldEndpointDetector.h"
#include "feature/WeldFeatureDetector.h"

namespace i2r_rlc::vision {

WeldFeatureResult CWeldFeatureDetector::detect(
    const cv::Mat &gray8,
    const LaserCenterlineResult &centerline,
    const WeldFeatureDetectionOptions &options)
{
    WeldFeatureResult result;
    result.gapFeature = CGapFeatureDetector::detectGapFromCenterline(
        gray8,
        centerline,
        options);

    result.grooveFeature = CGrooveEdgeDetector::detect(
        gray8,
        centerline,
        options);

    result.endpointResult = CWeldEndpointDetector::detect(
        gray8,
        centerline,
        result,
        options);

    result.valid = result.gapFeature.valid
        || result.endpointResult.valid
        || result.grooveFeature.valid;
    result.confidence = std::max(
        result.gapFeature.confidence,
        std::max(result.endpointResult.confidence,
                 result.grooveFeature.confidence));
    if (!result.valid)
    {
        result.message = result.gapFeature.message.empty()
            ? result.endpointResult.message
            : result.gapFeature.message;
        return result;
    }

    result.message = result.endpointResult.valid
        ? (result.grooveFeature.valid
               ? "weld feature, endpoint candidates and groove support detected"
               : "weld feature and endpoint candidates detected")
        : (result.grooveFeature.valid
               ? (result.gapFeature.valid
                      ? "weld feature and groove support detected"
                      : "groove support detected")
               : "weld feature detected");
    return result;
}

} // namespace i2r_rlc::vision
