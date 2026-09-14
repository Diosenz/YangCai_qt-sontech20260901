/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsTemplateAlignmentTools.h
 * @brief     IBVS 内部模板投影对齐工具。
 * @details   本文件只处理 3D 模板到 2D 边缘图的 chamfer 对齐搜索，不绑定
 *            工件起始点、焊缝提取或可视化输出。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_SRC_IBVS_TEMPLATE_ALIGNMENT_TOOLS_H
#define I2R_RLC_IBVS_SRC_IBVS_TEMPLATE_ALIGNMENT_TOOLS_H

#include <vector>

#include <opencv2/core.hpp>

#include "ibvs/IbvsStartPointImageDetector.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 模板对齐单个候选位姿及其 chamfer 代价。
 */
struct IbvsTemplateAlignmentCandidate
{
    double dxMm = 0.0;
    double dyMm = 0.0;
    double dpsiDeg = 0.0;
    double cost = 1e9;
};

/**
 * @brief 模板对齐结果和可信度诊断。
 */
struct IbvsTemplateAlignmentResult
{
    IbvsTemplateAlignmentCandidate best;
    IbvsTemplateAlignmentCandidate secondBest;
    cv::Vec3d rvecOpt = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d tvecOpt = cv::Vec3d(0.0, 0.0, 0.0);
    double inlierRatio = 0.0;
};

/**
 * @brief IBVS 内部模板投影对齐工具。
 */
class IbvsTemplateAlignmentTools final
{
public:
    /**
     * @brief 将 3D 模板边投影到当前边缘图并搜索最优扰动位姿。
     */
    static IbvsTemplateAlignmentResult alignTemplateToEdges(
        const std::vector<std::vector<cv::Point3f>>& templateEdges,
        const cv::Mat& edges,
        const cv::Mat& cameraMatrix,
        const cv::Vec3d& rvec0,
        const cv::Vec3d& tvec0,
        const IbvsTemplateAlignmentParameters& parameters);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_SRC_IBVS_TEMPLATE_ALIGNMENT_TOOLS_H
