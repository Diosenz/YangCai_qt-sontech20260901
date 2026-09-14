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
 * @file        StripeLineRepairTools.h
 * @brief       条纹中心线修复工具
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
#ifndef I2R_RLC_VISION_STRIPE_STRIPE_LINE_REPAIR_TOOLS_H
#define I2R_RLC_VISION_STRIPE_STRIPE_LINE_REPAIR_TOOLS_H

#include <opencv2/core.hpp>

#include "stripe/StripeDetectionTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 条纹中心线修复和分段工具。
 *
 * 该类承接旧 `SAMSplit/SAMMerge/SAMRemoveNFill/ISFFALink/FindROI`
 * 的可复用低阶语义。它只操作点列和 `CV_8UC1` 图像，不访问旧全局状态。
 */
class CStripeLineRepairTools final {
public:
    /**
     * @brief 按列灰度重心提取中心点。
     *
     * 这是 LOG 候选不足时的通用备用，也用于 SAM/ISFFA 早期路径的初始
     * `pMaxRowXLD` 构造。输出点按 x 升序。
     *
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 条纹检测选项，ROI 和距离单位均为 pixel。
     * @param responseByColumn 可选输出每列响应；函数不持有该指针。
     * @return 中心点列，坐标单位 pixel。
     */
    static std::vector<cv::Point2d> extractThresholdColumnCenters(
        const cv::Mat &gray8,
        const StripeDetectionOptions &options,
        std::vector<double> *responseByColumn);

    /**
     * @brief 对中心线做简单滑动平均平滑。
     * @param points 输入中心线点列，坐标单位 pixel。
     * @param windowRadius 滑动窗口半径，单位 point。
     * @return 平滑后的中心线点列。
     */
    static std::vector<cv::Point2d> smoothCenterline(
        const std::vector<cv::Point2d> &points,
        int windowRadius);

    /**
     * @brief 旧 `SAMSplit` 等价：按列断裂和行跳变切分点列。
     * @param points 输入中心线点列，坐标单位 pixel。
     * @param options 条纹检测选项，连续性阈值单位为 pixel。
     * @return 分段结果，段内点仍保持原坐标单位。
     */
    static std::vector<StripeSegment> splitByContinuity(
        const std::vector<cv::Point2d> &points,
        const StripeDetectionOptions &options);

    /**
     * @brief 旧 `SAMRemoveNFill/ISFFALink` 等价：线性补齐短缺失段。
     * @param points 输入中心线点列，坐标单位 pixel。
     * @param options 条纹检测选项。
     * @param insertedPoints 可选输出新插入点；函数不持有该指针。
     * @return 修复后的中心线点列。
     */
    static std::vector<cv::Point2d> repairLinearGaps(
        const std::vector<cv::Point2d> &points,
        const StripeDetectionOptions &options,
        std::vector<cv::Point2d> *insertedPoints);

    /**
     * @brief 用 split_line + RANSAC 补充分段拟合线。
     * @param points 输入中心线点列，坐标单位 pixel。
     * @param options 条纹检测选项。
     * @return 带拟合线和质量信息的分段集合。
     */
    static std::vector<StripeSegment> fitSegments(
        const std::vector<cv::Point2d> &points,
        const StripeDetectionOptions &options);

    /**
     * @brief 旧 `FindROI` 等价：围绕有效中心线估计下一阶段 ROI。
     * @param points 输入中心线点列，坐标单位 pixel。
     * @param imageSize 图像尺寸，单位 pixel。
     * @param paddingPixel ROI 外扩距离，单位 pixel。
     * @return 限制在图像范围内的 ROI。
     */
    static cv::Rect estimateCenterlineRoi(
        const std::vector<cv::Point2d> &points,
        const cv::Size &imageSize,
        int paddingPixel);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_STRIPE_LINE_REPAIR_TOOLS_H

