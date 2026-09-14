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
 * @file        StripeDetectionTypes.h
 * @brief       条纹检测数据类型
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
#ifndef I2R_RLC_VISION_STRIPE_STRIPE_DETECTION_TYPES_H
#define I2R_RLC_VISION_STRIPE_STRIPE_DETECTION_TYPES_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "geometry/LineGeometryTools.h"

namespace i2r_rlc::vision {

/**
 * @brief 激光条纹中心线分段。
 *
 * 坐标单位均为 pixel。该结构承接旧 `SplitLines` 的现代语义：点列中一段
 * 连续或可拟合的中心线区间，以及该区间的鲁棒拟合直线。
 */
struct StripeSegment {
    /// @brief 段在完整中心线点列中的起始索引，包含。
    int startIndex = 0;

    /// @brief 段在完整中心线点列中的结束索引，包含。
    int endIndex = 0;

    /// @brief 该段中心线点，单位 pixel。
    std::vector<cv::Point2d> points;

    /// @brief 该段拟合线。
    WeldLineSegment fittedLine;

    /// @brief 段置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示该段可用于下游 feature 或 pipeline。
    bool valid = false;
};

/**
 * @brief SAM/ISFFA/Steger 共用检测参数。
 *
 * 本结构只处理 `CV_8UC1` 灰度图。所有距离单位为 pixel；灰度阈值范围
 * 为 0~255；ratio/confidence 范围为 0~1。
 */
struct StripeDetectionOptions {
    /// @brief 参与条纹候选的最低灰度。
    int thresholdGrayValue = 180;

    /// @brief LOG/CoM 使用的默认激光宽度。
    int defaultLaserWidthPixel = 6;

    /// @brief LOG 峰值列采样步长，对应旧 `eff`。
    int logColumnStepPixel = 1;

    /// @brief LOG 峰值响应阈值。
    double logThreshold = -3000.0;

    /// @brief 列中心线断裂允许的最大列间距。
    int maxColumnGapPixel = 8;

    /// @brief 相邻中心线点允许的最大行跳变。
    double maxYJumpPixel = 12.0;

    /// @brief 小于该点数的分段会被视为无效。
    int minSegmentPointCount = 6;

    /// @brief 整体最少中心线点数量。
    int minTotalPointCount = 12;

    /// @brief 分段拟合时触发 split_line 的最大点线距离。
    double splitDistancePixel = 5.0;

    /// @brief 缺失段线性补点允许的最大列距。
    int repairMaxGapPixel = 18;

    /// @brief 中心线平滑半径；0 表示不平滑。
    int smoothWindowRadius = 1;

    /// @brief Steger-like 路径高斯平滑 sigma。
    double stegerSigma = 1.2;

    /// @brief Steger-like 路径最低二阶响应。
    double stegerMinRidgeResponse = 10.0;

    /// @brief 是否限制 ROI。
    bool useRoi = false;

    /// @brief 可选 ROI，单位 pixel。
    cv::Rect roi;
};

/**
 * @brief 条纹检测跨帧状态。
 *
 * 旧工程使用全局点列和失败计数。新工程由调用方持有本结构，下游若需要
 * 多帧补线或跳变过滤，可在此扩展。
 */
struct StripeDetectionState {
    /// @brief 最近一次有效中心线，单位 pixel。
    std::vector<cv::Point2d> lastValidCenterlinePoints;

    /// @brief 连续失败帧数。
    int consecutiveFailureCount = 0;
};

/**
 * @brief SAM/ISFFA/Steger 统一输出。
 */
struct StripeCenterlineResult {
    /// @brief true 表示中心线结果可用于下游焊缝语义。
    bool valid = false;

    /// @brief 中心线点，单位 pixel，按 x 升序。
    std::vector<cv::Point2d> centerlinePoints;

    /// @brief 由修复逻辑插入的点，单位 pixel。
    std::vector<cv::Point2d> repairedPoints;

    /// @brief 中心线分段结果。
    std::vector<StripeSegment> segments;

    /// @brief 每列响应，通常来自 LOG、Steger 或灰度峰值。
    std::vector<double> responseByColumn;

    /// @brief 中心线质量分数，范围 0~1。
    double qualityScore = 0.0;

    /// @brief 处理耗时，单位 ms。
    double processingTimeMs = 0.0;

    /// @brief 状态说明。
    std::string message;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_STRIPE_DETECTION_TYPES_H

