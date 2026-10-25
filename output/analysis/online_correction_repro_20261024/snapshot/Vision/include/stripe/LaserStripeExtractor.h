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
 * @file        LaserStripeExtractor.h
 * @brief       激光条纹提取接口
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
#ifndef I2R_RLC_VISION_STRIPE_LASER_STRIPE_EXTRACTOR_H
#define I2R_RLC_VISION_STRIPE_LASER_STRIPE_EXTRACTOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 激光条纹阈值和 ROI 估计参数。
 *
 * 参数只作用于调用方传入的 CV_8UC1 图像，单位均为 pixel 或 0~1 ratio。
 * 本结构不包含中心线、亚像素、DP 或焊缝语义。
 */
struct LaserStripeThresholdOptions {
    /// @brief 固定灰度阈值，范围 0~255。
    int thresholdGrayValue = 180;

    /// @brief 是否使用 `maxGray * relativeThresholdRatio` 提高实际阈值。
    bool useRelativeThreshold = true;

    /// @brief 相对最大灰度阈值比例，范围 0~1。
    double relativeThresholdRatio = 0.65;

    /// @brief 判定有效列所需的最少前景像素数量，单位 pixel。
    int minForegroundPixelsPerColumn = 2;

    /// @brief 候选 ROI 外扩像素，单位 pixel。
    int roiPaddingPixel = 1;

    /// @brief 每列保留的局部峰数量上限。
    int maxPeaksPerColumn = 3;
};

/**
 * @brief 单列内的整数像素局部峰候选。
 *
 * 本结构只表达基础峰值候选，不做亚像素插值，也不代表最终中心线。
 */
struct LaserStripeColumnPeak {
    /// @brief 峰值所在列坐标，单位 pixel。
    int xPixel = 0;

    /// @brief 峰值所在行坐标，单位 pixel。
    int yPixel = 0;

    /// @brief 峰值灰度，范围 0~255。
    int grayValue = 0;

    /// @brief 峰值相对 255 的置信比例，范围 0~1。
    double scoreRatio = 0.0;
};

/**
 * @brief 激光条纹单列统计。
 *
 * `firstYPixel` 到 `lastYPixel` 描述阈值 mask 中该列前景包围范围；
 * `peakCandidates` 仅用于下游中心线阶段的候选输入。
 */
struct LaserStripeColumnProfile {
    /// @brief 列坐标，单位 pixel。
    int xPixel = 0;

    /// @brief 该列是否包含有效条纹前景。
    bool hasStripe = false;

    /// @brief 该列第一个前景像素行坐标，单位 pixel。
    int firstYPixel = -1;

    /// @brief 该列最后一个前景像素行坐标，单位 pixel。
    int lastYPixel = -1;

    /// @brief 该列前景像素数量，单位 pixel。
    int foregroundPixelCount = 0;

    /// @brief 该列条纹包围宽度，单位 pixel。
    int stripeWidthPixel = 0;

    /// @brief 该列前景像素比例，范围 0~1。
    double foregroundRatio = 0.0;

    /// @brief 该列局部峰候选。
    std::vector<LaserStripeColumnPeak> peakCandidates;
};

/**
 * @brief 基于阈值 mask 的候选 ROI 和宽度估计。
 */
struct LaserStripeRoiEstimate {
    /// @brief 候选条纹 ROI，单位 pixel。
    VisionImageRoi roi;

    /// @brief 有效列数量。
    int validColumnCount = 0;

    /// @brief 有效列比例，范围 0~1。
    double validColumnRatio = 0.0;

    /// @brief 平均条纹包围宽度，单位 pixel。
    double meanStripeWidthPixel = 0.0;

    /// @brief 最小条纹包围宽度，单位 pixel。
    int minStripeWidthPixel = 0;

    /// @brief 最大条纹包围宽度，单位 pixel。
    int maxStripeWidthPixel = 0;

    /// @brief 基于有效列比例的基础置信度，范围 0~1。
    double confidenceRatio = 0.0;
};

/**
 * @brief 激光条纹基础提取结果。
 */
struct LaserStripeExtractionResult {
    /// @brief 二值 mask，CV_8UC1，前景值为 255。
    cv::Mat mask;

    /// @brief 每列条纹统计。
    std::vector<LaserStripeColumnProfile> columnProfiles;

    /// @brief 候选 ROI 与宽度统计。
    LaserStripeRoiEstimate roiEstimate;

    /// @brief 全图前景像素数量。
    int foregroundPixelCount = 0;

    /// @brief 全图前景像素比例，范围 0~1。
    double foregroundRatio = 0.0;
};

/**
 * @brief 激光条纹基础提取工具。
 *
 * 本类只做阈值 mask、有效像素比例、候选 ROI 和列宽度统计。
 * 不包含中心线、亚像素、二阶 DP、GapDetector 完整迁移或焊缝特征判断。
 */
class CLaserStripeExtractor final {
public:
    /**
     * @brief 根据灰度图生成激光条纹二值 mask。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param options 阈值和 ROI 估计参数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return `CV_8UC1` 二值 mask，前景为 255；失败时返回空 Mat。
     */
    static cv::Mat createMask(const cv::Mat &grayImage,
                              const LaserStripeThresholdOptions &options,
                              VisionPreprocessStatus *status);

    /**
     * @brief 提取指定列的整数像素局部峰候选。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param xPixel 列坐标，单位 pixel。
     * @param options 阈值和峰值筛选参数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 按峰值强度排序的局部峰候选。
     */
    static std::vector<LaserStripeColumnPeak> extractColumnPeaks(
        const cv::Mat &grayImage,
        int xPixel,
        const LaserStripeThresholdOptions &options,
        VisionPreprocessStatus *status);

    /**
     * @brief 根据灰度图和 mask 计算每列条纹统计。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param mask 输入 `CV_8UC1` 二值 mask，非零为前景。
     * @param options 阈值和列统计参数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 每列条纹统计，长度通常等于图像宽度。
     */
    static std::vector<LaserStripeColumnProfile> calculateColumnProfiles(
        const cv::Mat &grayImage,
        const cv::Mat &mask,
        const LaserStripeThresholdOptions &options,
        VisionPreprocessStatus *status);

    /**
     * @brief 根据列统计估计候选 ROI 和条纹宽度。
     * @param columnProfiles 每列条纹统计。
     * @param imageWidthPixel 图像宽度，单位 pixel。
     * @param imageHeightPixel 图像高度，单位 pixel。
     * @param options ROI 外扩和有效列参数。
     * @return ROI 与宽度估计结果。
     */
    static LaserStripeRoiEstimate estimateRoi(
        const std::vector<LaserStripeColumnProfile> &columnProfiles,
        int imageWidthPixel,
        int imageHeightPixel,
        const LaserStripeThresholdOptions &options);

    /**
     * @brief 一次性执行 mask、列统计、候选 ROI 与有效像素比例计算。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param options 阈值和 ROI 估计参数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 激光条纹基础提取结果。
     */
    static LaserStripeExtractionResult extract(
        const cv::Mat &grayImage,
        const LaserStripeThresholdOptions &options,
        VisionPreprocessStatus *status);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_LASER_STRIPE_EXTRACTOR_H

