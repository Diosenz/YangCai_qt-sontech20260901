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
 * @file        DpStripeCenterlineDetector.h
 * @brief       动态规划激光中心线检测接口
 * @details     承接旧 GapDetector 的二阶 DP 条纹提取流程，只输出图像坐标
 *              中心线，不承担三维重建或焊缝宽度语义。
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
#ifndef I2R_RLC_VISION_STRIPE_DP_STRIPE_CENTERLINE_DETECTOR_H
#define I2R_RLC_VISION_STRIPE_DP_STRIPE_CENTERLINE_DETECTOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "stripe/LaserCenterlineTypes.h"
#include "stripe/StripeDetectionTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 二阶 DP 中心线检测参数。
 *
 * 所有坐标和距离单位均为 pixel，灰度范围为 0~255。方向先验按每列
 * y 方向增量表达，数组长度应与 ROI 宽度一致。
 */
struct DpStripeDetectionOptions {
    /// @brief 检测 ROI；宽高小于等于 0 时使用整图。
    cv::Rect roi;

    /// @brief ROI 内参与检测的最小 y；小于 0 时从 0 开始。
    int yMinPixel = -1;

    /// @brief ROI 内参与检测的最大 y；小于 0 时到 ROI 底部。
    int yMaxPixel = -1;

    /// @brief 预滤波高斯 sigma。
    double blurSigma = 1.2;

    /// @brief 列自适应阈值系数，阈值为 alpha * 列分位灰度。
    double adaptiveThresholdRatio = 0.45;

    /// @brief 峰值显著性最小绝对值。
    double minProminenceGray = 6.0;

    /// @brief 峰值显著性相对列分位灰度的比例。
    double prominenceRatio = 0.02;

    /// @brief 峰值显著性的绝对下限。
    double prominenceAbsMinGray = 1.5;

    /// @brief 每列保留的最多真实候选数量。
    int maxCandidatesPerColumn = 6;

    /// @brief 峰值强度一元项权重。
    double intensityWeight = 2.0;

    /// @brief 一阶斜率平滑项权重。
    double slopeWeight = 0.25;

    /// @brief 二阶曲率项权重。
    double curvatureWeight = 0.15;

    /// @brief 相邻列允许的最大 y 跳变。
    double maxJumpPixel = 20.0;

    /// @brief 使用虚拟候选表示缺失列时的代价。
    double missingPenalty = 1.2;

    /// @brief 是否使用方向先验。
    bool useDirectionPrior = true;

    /// @brief 方向先验权重。
    double directionPriorWeight = 2.3;

    /// @brief 方向先验鲁棒截断阈值。
    double directionDeltaPixel = 16.0;

    /// @brief ROI 坐标中的焊缝列，用于放松附近曲率。
    int seamXPixel = -1;

    /// @brief 焊缝列附近放松曲率的半宽。
    int seamBandPixel = 25;

    /// @brief 每列期望斜率，单位 pixel/column。
    std::vector<double> slopePriorByColumn;

    /// @brief 每列方向先验权重；为空时按 1 处理。
    std::vector<double> priorWeightByColumn;

    /// @brief 是否根据方向先验中的拐点放松曲率。
    bool relaxCurvatureByPrior = true;

    /// @brief 判断方向先验拐点的斜率变化阈值。
    double cornerSlopeThreshold = 1.5;

    /// @brief 拐点处曲率代价缩放系数。
    double cornerCurvatureScale = 0.2;

    /// @brief Y 方向候选抑制最小间距。
    double candidateMinSeparationPixel = 5.0;

    /// @brief 输出有效中心线所需的最少点数。
    int minValidPointCount = 12;
};

/**
 * @brief 二阶动态规划激光中心线检测器。
 */
class CDpStripeCenterlineDetector final {
public:
    /**
     * @brief 从 CV_8UC1 灰度图检测 DP 中心线。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 二阶 DP 检测参数，ROI 和距离单位均为 pixel。
     * @return 条纹中心线检测结果；失败时 `valid=false` 并给出 message。
     */
    static StripeCenterlineResult detect(
        const cv::Mat &gray8,
        const DpStripeDetectionOptions &options);

    /**
     * @brief 从 CV_8UC1 灰度图检测正式 gap 链路中心线。
     *
     * 该接口承接旧 GapDetector::extractLaserCenterline，返回 gap、
     * 端点和坡口特征检测共享的中心线结果结构。
     *
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 二阶 DP 检测参数，ROI 和距离单位均为 pixel。
     * @return LaserCenterlineResult 兼容结果，坐标单位为 pixel。
     */
    static LaserCenterlineResult detectCenterline(
        const cv::Mat &gray8,
        const DpStripeDetectionOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_DP_STRIPE_CENTERLINE_DETECTOR_H
