/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsRedFeatureDetector.h
 * @brief     焊枪红色双特征点检测。
 * @details   本文件定义红色特征点候选提取、筛选和双点匹配接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_RED_FEATURE_DETECTOR_H
#define I2R_RLC_IBVS_IBVS_RED_FEATURE_DETECTOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 红色连通域候选及其评分特征。
 */
struct IbvsRedBlobCandidate
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
 * @brief 红色双特征点检测结果。
 */
struct IbvsRedFeatureDetectionResult
{
    IbvsError error;
    bool accepted = false;
    PixelFeaturePair featurePoints;
    double distancePx = 0.0;
    std::vector<IbvsRedBlobCandidate> candidates;
};

/**
 * @brief 红色双特征点检测参数。
 */
struct IbvsRedFeatureDetectorConfig
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
    double minPairDistancePx = 15.0;
    double maxPairDistancePx = 60.0;
};

/**
 * @brief 基于 HSV 红色分割和连通域筛选的双特征点检测器。
 */
class IbvsRedFeatureDetector
{
public:
    IbvsRedFeatureDetector();
    explicit IbvsRedFeatureDetector(const IbvsRedFeatureDetectorConfig& config);

    /**
     * @brief 从 BGR 图像中检测焊枪红色双特征点。
     */
    IbvsRedFeatureDetectionResult detect(const cv::Mat& bgrImage) const;

private:
    IbvsRedFeatureDetectionResult selectBestPair(
        const std::vector<IbvsRedBlobCandidate>& candidates) const;

    IbvsRedFeatureDetectorConfig m_config;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_RED_FEATURE_DETECTOR_H
