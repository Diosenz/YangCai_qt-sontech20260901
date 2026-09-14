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
 * @file        SubpixelCenterlineTools.h
 * @brief       亚像素中心线计算工具
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
#ifndef I2R_RLC_VISION_STRIPE_SUBPIXEL_CENTERLINE_TOOLS_H
#define I2R_RLC_VISION_STRIPE_SUBPIXEL_CENTERLINE_TOOLS_H

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"
#include "stripe/LaserStripeExtractor.h"

namespace i2r_rlc::vision {

/**
 * @brief 三点抛物线亚像素拟合结果。
 *
 * `offsetPixel` 是相对整数峰值位置的偏移，范围约束为 [-0.5, 0.5]。
 */
struct SubpixelParabolaResult {
    /// @brief true 表示成功得到稳定的亚像素偏移。
    bool valid = false;

    /// @brief 相对整数峰值的偏移，单位 pixel。
    double offsetPixel = 0.0;

    /// @brief 亚像素位置，单位 pixel。
    double positionPixel = 0.0;

    /// @brief 曲率强度，用于辅助质量评价。
    double curvature = 0.0;

    /// @brief 由峰值显著性归一化得到的置信度，范围 0~1。
    double confidenceRatio = 0.0;
};

/**
 * @brief 灰度重心结果。
 *
 * 本结构只描述单列局部窗口内的中心点，不代表完整中心线。
 */
struct GrayCentroidResult {
    /// @brief true 表示窗口内有效权重大于 0。
    bool valid = false;

    /// @brief 重心行坐标，单位 pixel。
    double centerYPixel = 0.0;

    /// @brief 参与重心计算的总权重。
    double totalWeight = 0.0;

    /// @brief 重心偏移相对窗口半宽的归一化置信度，范围 0~1。
    double confidenceRatio = 0.0;
};

/**
 * @brief 局部中心点质量评价。
 *
 * 质量只来自局部灰度、对比度和重心稳定性，不包含 DP 连续性。
 */
struct LocalCenterlineQuality {
    /// @brief true 表示局部窗口和候选点有效。
    bool valid = false;

    /// @brief 候选列坐标，单位 pixel。
    int xPixel = 0;

    /// @brief 候选中心行坐标，单位 pixel。
    double centerYPixel = 0.0;

    /// @brief 候选峰值灰度，范围 0~255。
    double peakGrayValue = 0.0;

    /// @brief 局部背景灰度估计，范围 0~255。
    double backgroundGrayValue = 0.0;

    /// @brief 峰值与背景的灰度差，范围 0~255。
    double contrastGrayValue = 0.0;

    /// @brief 对比度比例，范围 0~1。
    double contrastRatio = 0.0;

    /// @brief 综合质量分数，范围 0~1。
    double qualityRatio = 0.0;
};

/**
 * @brief 中心线亚像素和局部质量基础工具。
 *
 * 本类只提供单点/单列数学工具，不执行完整中心线提取、不做二阶 DP，
 * 也不包含 Hessian/Steger 完整算法。
 */
class CSubpixelCenterlineTools final {
public:
    /**
     * @brief 使用三点抛物线估计峰值亚像素位置。
     * @param leftValue 左邻采样值，通常为灰度或响应值。
     * @param centerValue 中心采样值，通常为整数峰值。
     * @param rightValue 右邻采样值，通常为灰度或响应值。
     * @param centerPositionPixel 中心采样点坐标，单位 pixel。
     * @return 亚像素拟合结果；三点退化时 `valid=false`。
     */
    static SubpixelParabolaResult fitParabola(
        double leftValue,
        double centerValue,
        double rightValue,
        double centerPositionPixel);

    /**
     * @brief 使用 CLaserStripeExtractor 的整数峰候选做抛物线细化。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param peak 单列整数峰候选，坐标单位为 pixel。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 亚像素峰值结果。
     */
    static SubpixelParabolaResult refinePeakWithParabola(
        const cv::Mat &grayImage,
        const LaserStripeColumnPeak &peak,
        VisionPreprocessStatus *status);

    /**
     * @brief 计算单列灰度分位数，作为局部尺度估计。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param xPixel 列坐标，单位 pixel。
     * @param yMinPixel 搜索起始行，单位 pixel。
     * @param yMaxPixel 搜索结束行，单位 pixel。
     * @param quantileRatio 分位比例，范围 0~1。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 分位灰度值，范围 0~255；失败时返回 0。
     */
    static int robustColumnScale(const cv::Mat &grayImage,
                                 int xPixel,
                                 int yMinPixel,
                                 int yMaxPixel,
                                 double quantileRatio,
                                 VisionPreprocessStatus *status);

    /**
     * @brief 在单列局部窗口内计算灰度重心。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param xPixel 列坐标，单位 pixel。
     * @param centerYPixel 中心行坐标，单位 pixel。
     * @param halfWindowPixel 窗口半高，单位 pixel。
     * @param backgroundGrayValue 背景灰度估计，范围 0~255。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 灰度重心结果。
     */
    static GrayCentroidResult calculateGrayCentroid(
        const cv::Mat &grayImage,
        int xPixel,
        int centerYPixel,
        int halfWindowPixel,
        int backgroundGrayValue,
        VisionPreprocessStatus *status);

    /**
     * @brief 估计单列候选点的局部对比度。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param xPixel 列坐标，单位 pixel。
     * @param centerYPixel 中心行坐标，单位 pixel。
     * @param innerHalfWindowPixel 内层峰值窗口半高，单位 pixel。
     * @param outerHalfWindowPixel 外层背景窗口半高，单位 pixel。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 峰值与背景灰度差，范围约为 0~255。
     */
    static double calculateLocalContrast(const cv::Mat &grayImage,
                                         int xPixel,
                                         int centerYPixel,
                                         int innerHalfWindowPixel,
                                         int outerHalfWindowPixel,
                                         VisionPreprocessStatus *status);

    /**
     * @brief 对单个整数峰候选计算局部质量分数。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param peak 单列整数峰候选。
     * @param centroidHalfWindowPixel 重心窗口半高，单位 pixel。
     * @param backgroundHalfWindowPixel 背景估计窗口半高，单位 pixel。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 局部中心点质量评价。
     */
    static LocalCenterlineQuality evaluateLocalQuality(
        const cv::Mat &grayImage,
        const LaserStripeColumnPeak &peak,
        int centroidHalfWindowPixel,
        int backgroundHalfWindowPixel,
        VisionPreprocessStatus *status);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_SUBPIXEL_CENTERLINE_TOOLS_H

