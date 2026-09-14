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
 * @file        GapFeatureDetector.h
 * @brief       焊缝间隙特征检测接口
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
#ifndef I2R_RLC_VISION_FEATURE_GAP_FEATURE_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_GAP_FEATURE_DETECTOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "stripe/LaserCenterlineTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 基于中心线先验的最小 gap/左右边缘检测器。
 *
 * 本类只做 CV_8UC1 图像上的局部截面边缘候选。
 */
class CGapFeatureDetector final {
public:
    /**
     * @brief 在指定中心点附近的局部行窗口内检测左右边缘。
     */
    static WeldEdgePair detectEdgesOnRow(
        const cv::Mat &gray8,
        double centerXPixel,
        double centerYPixel,
        const WeldFeatureDetectionOptions &options);

    /**
     * @brief 收集指定中心点附近的左右边缘候选。
     *
     * 候选来自局部多行窗口，左边缘优先匹配亮到暗的变化，右边缘优先
     * 匹配暗到亮的变化；当方向判断不足时保留强对比度候选作为备用。
     */
    static void collectEdgeCandidates(
        const cv::Mat &gray8,
        double centerXPixel,
        double centerYPixel,
        const WeldFeatureDetectionOptions &options,
        std::vector<GapEdgeCandidate> *leftCandidates,
        std::vector<GapEdgeCandidate> *rightCandidates);

    /**
     * @brief 使用中心线结果估计 gap 宽度和焊缝点候选。
     */
    static GapFeatureResult detectGapFromCenterline(
        const cv::Mat &gray8,
        const LaserCenterlineResult &centerline,
        const WeldFeatureDetectionOptions &options);

    /**
     * @brief 根据左右边缘中点估计当前阶段的焊缝点候选。
     */
    static WeldFeaturePoint estimateWeldPointFromEdges(
        const WeldEdgePair &edgePair,
        const WeldFeatureDetectionOptions &options);

    /**
     * @brief 计算局部行截面上的一维边缘对比度。
     */
    static double calculateEdgeContrast(
        const cv::Mat &gray8,
        int xPixel,
        int yPixel,
        int searchHalfWindowY);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_GAP_FEATURE_DETECTOR_H

