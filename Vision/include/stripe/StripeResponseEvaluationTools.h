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
 * @file        StripeResponseEvaluationTools.h
 * @brief       条纹响应评价工具
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
#ifndef I2R_RLC_VISION_STRIPE_STRIPE_RESPONSE_EVALUATION_TOOLS_H
#define I2R_RLC_VISION_STRIPE_STRIPE_RESPONSE_EVALUATION_TOOLS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geometry/LineGeometryTools.h"

namespace i2r_rlc::vision {

/**
 * @brief 单列 LOG 搜索结果。
 */
struct StripeSingleColumnLogResult {
    /// @brief true 表示搜索得到有效行。
    bool valid = false;

    /// @brief 最优行坐标，单位 pixel。
    int rowPixel = -1;

    /// @brief 最大 LOG 响应。
    double response = 0.0;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 条纹响应质量评价工具。
 *
 * 旧 `SingleCol_LOG` 与 `RDA` 被拆到这里：前者用于对接端点等局部列搜索，
 * 后者用于把点到拟合线的残差归一化为 0~1 质量分数。
 */
class CStripeResponseEvaluationTools final {
public:
    /**
     * @brief 旧 `SingleCol_LOG` 等价：在单列行范围内找最大 LOG 响应。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param xPixel 列坐标，单位 pixel。
     * @param rowStartPixel 搜索起始行，单位 pixel。
     * @param rowEndPixel 搜索结束行，单位 pixel。
     * @param laserWidthPixel LOG 近似尺度，单位 pixel。
     * @return 单列 LOG 峰值搜索结果。
     */
    static StripeSingleColumnLogResult findSingleColumnLogPeak(
        const cv::Mat &gray8,
        int xPixel,
        int rowStartPixel,
        int rowEndPixel,
        int laserWidthPixel);

    /**
     * @brief 旧 `RDA` 等价：用点到拟合线残差评价中心线质量。
     * @param points 中心线点列，坐标单位 pixel。
     * @param line 拟合线段，坐标单位 pixel。
     * @param maxResidualPixel 残差归一化上限，单位 pixel。
     * @return 0~1 质量分数，1 表示点列贴近拟合线。
     */
    static double calculateResidualDistributionAgreement(
        const std::vector<cv::Point2d> &points,
        const WeldLineSegment &line,
        double maxResidualPixel);

    /**
     * @brief 将每列响应均值归一化为 0~1。
     * @param responseByColumn 每列响应值。
     * @param normalizationValue 归一化参考值，必须大于 0。
     * @return 0~1 平均响应分数。
     */
    static double calculateMeanResponseScore(
        const std::vector<double> &responseByColumn,
        double normalizationValue);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_STRIPE_RESPONSE_EVALUATION_TOOLS_H

