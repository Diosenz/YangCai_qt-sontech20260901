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
 * @file        WeldStartPointTypes.h
 * @brief       焊缝起点检测数据类型
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
#ifndef I2R_RLC_VISION_FEATURE_WELD_START_POINT_TYPES_H
#define I2R_RLC_VISION_FEATURE_WELD_START_POINT_TYPES_H

#include <deque>
#include <string>

#include "feature/ActiveContourWeldFeatureDetector.h"
#include "feature/WeldFeatureTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 起点/终点检测方法。
 */
enum class WeldStartPointMethod {
    /// @brief 对应旧 `StartPtDet_Area` 的区域变化入口。
    AreaChange = 0,

    /// @brief 对应旧 `StartPtDet` 的曲线极值/线段距离入口。
    CurveDistance = 1
};

/**
 * @brief 起点/终点检测参数。
 */
struct WeldStartPointOptions {
    /// @brief 当前焊缝模式。
    ActiveContourSeamMode seamMode = ActiveContourSeamMode::Automatic;

    /// @brief 端点检测方法。
    WeldStartPointMethod method = WeldStartPointMethod::AreaChange;

    /// @brief 主动轮廓参数。
    ActiveContourDetectionOptions activeContourOptions;

    /// @brief 端点结果最低置信度。
    double minConfidence = 0.10;

    /// @brief 端点跳变过滤阈值，单位 pixel。
    double maxEndpointJumpPixel = 80.0;

    /// @brief 历史备用长度，单位 frame。
    int historySize = 6;
};

/**
 * @brief 起点/终点检测跨帧状态。
 */
struct WeldStartPointState {
    /// @brief 主动轮廓跨帧状态。
    ActiveContourDetectionState activeContourState;

    /// @brief 最近一次 gap 情况，替代旧全局 `gap_situ`。
    int gapSituation = 0;

    /// @brief 最近若干有效端点结果。
    std::deque<WeldEndpointResult> recentEndpointResults;
};

/**
 * @brief 起点/终点检测结果。
 */
struct WeldStartPointResult {
    /// @brief true 表示端点结果有效。
    bool valid = false;

    /// @brief 焊缝类型。
    WeldSeamType seamType = WeldSeamType::Unknown;

    /// @brief 起点候选。
    WeldFeaturePoint startPoint;

    /// @brief 终点候选。
    WeldFeaturePoint endPoint;

    /// @brief 主焊缝点。
    WeldFeaturePoint weldPoint;

    /// @brief 对接 gap 左端点。
    WeldFeaturePoint leftGapPoint;

    /// @brief 对接 gap 右端点。
    WeldFeaturePoint rightGapPoint;

    /// @brief 主动轮廓结果，供 replay/调试观察。
    ActiveContourDetectionResult activeContourResult;

    /// @brief 端点检测置信度。
    double confidence = 0.0;

    /// @brief 状态说明。
    std::string message;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_WELD_START_POINT_TYPES_H

