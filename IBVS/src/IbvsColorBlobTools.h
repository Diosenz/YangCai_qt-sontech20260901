/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsColorBlobTools.h
 * @brief     IBVS 内部颜色连通域特征提取工具。
 * @details   本文件只表达通用颜色分割、连通域候选提取和评分，不绑定焊枪、
 *            工件或焊缝语义。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_SRC_IBVS_COLOR_BLOB_TOOLS_H
#define I2R_RLC_IBVS_SRC_IBVS_COLOR_BLOB_TOOLS_H

#include <vector>

#include <opencv2/core.hpp>

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief HSV 颜色阈值范围。
 */
struct IbvsColorRange
{
    cv::Scalar lower;
    cv::Scalar upper;
};

/**
 * @brief 颜色连通域候选提取参数。
 */
struct IbvsColorBlobExtractionConfig
{
    bool applyGamma = true;
    bool applyBilateral = true;
    double gamma = 1.8;
    int morphKernelSize = 3;
    double minBlobAreaPx = 6.0;
    double maxBlobSizePx = 70.0;
    double maxAspectRatio = 2.0;
    double minCircularity = 0.45;
    double minMeanSaturation = 60.0;
    double minMeanValue = 20.0;
    std::vector<IbvsColorRange> hsvRanges;
};

/**
 * @brief 颜色连通域候选及其评分特征。
 */
struct IbvsColorBlobCandidate
{
    PixelPoint centroid;
    double areaPx = 0.0;
    double aspectRatio = 0.0;
    double circularity = 0.0;
    double meanSaturation = 0.0;
    double meanValue = 0.0;
    double score = 0.0;
};

/**
 * @brief IBVS 内部颜色连通域提取工具。
 */
class IbvsColorBlobTools final
{
public:
    /**
     * @brief 对 BGR 图像执行 gamma 校正。
     */
    static cv::Mat applyGammaCorrection(
        const cv::Mat& image,
        double gamma);

    /**
     * @brief 根据 HSV 阈值和形态学参数提取颜色掩膜。
     */
    static cv::Mat extractColorMask(
        const cv::Mat& bgrImage,
        const IbvsColorBlobExtractionConfig& config,
        cv::Mat* hsvImage);

    /**
     * @brief 从二值掩膜和 HSV 图像中提取连通域候选。
     */
    static std::vector<IbvsColorBlobCandidate> extractCandidates(
        const cv::Mat& mask,
        const cv::Mat& hsvImage,
        const IbvsColorBlobExtractionConfig& config);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_SRC_IBVS_COLOR_BLOB_TOOLS_H
