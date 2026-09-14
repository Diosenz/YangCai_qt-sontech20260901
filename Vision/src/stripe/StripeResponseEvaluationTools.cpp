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
 * @file        StripeResponseEvaluationTools.cpp
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
#include <algorithm>
#include <cmath>

#include "stripe/LaserLineResponseTools.h"
#include "stripe/StripeResponseEvaluationTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 计算点到拟合线的距离。
 * @param point 输入点，坐标单位 pixel。
 * @param line 拟合线段，坐标单位 pixel。
 * @return 点线距离，单位 pixel；无效线段返回 0。
 */
double pointLineDistance(const cv::Point2d &point, const WeldLineSegment &line)
{
    if (!line.valid)
    {
        return 0.0;
    }
    if (line.vertical)
    {
        return std::abs(point.x - line.intercept);
    }
    const double numerator = std::abs(
        line.slope * point.x - point.y + line.intercept);
    const double denominator = std::sqrt(line.slope * line.slope + 1.0);
    return denominator > 0.0 ? numerator / denominator : 0.0;
}

/**
 * @brief 将质量比例裁剪到 0~1。
 * @param value 输入比例。
 * @return 裁剪后的比例。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

} // namespace

StripeSingleColumnLogResult CStripeResponseEvaluationTools::findSingleColumnLogPeak(
    const cv::Mat &gray8,
    int xPixel,
    int rowStartPixel,
    int rowEndPixel,
    int laserWidthPixel)
{
    StripeSingleColumnLogResult result;
    if (gray8.empty() || gray8.type() != CV_8UC1)
    {
        result.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return result;
    }
    if (xPixel < 0 || xPixel >= gray8.cols || laserWidthPixel <= 0)
    {
        result.message = "invalid single column log options";
        return result;
    }

    const int begin = std::max(0, std::min(rowStartPixel, rowEndPixel));
    const int end = std::min(gray8.rows - 1, std::max(rowStartPixel, rowEndPixel));
    std::vector<int> widthByRow(gray8.rows, std::max(1, laserWidthPixel));

    double bestResponse = -1.0e30;
    int bestRow = -1;
    for (int y = begin; y <= end; ++y)
    {
        VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
        const double response = CLaserLineResponseTools::calculateLogResponseAt(
            gray8,
            widthByRow,
            xPixel,
            y,
            &status);
        if (status == VisionPreprocessStatus::Succeeded
            && response > bestResponse)
        {
            bestResponse = response;
            bestRow = y;
        }
    }

    result.valid = bestRow >= 0;
    result.rowPixel = bestRow;
    result.response = bestResponse;
    result.message = result.valid
        ? "single column log peak found"
        : "single column log found no peak";
    return result;
}

double CStripeResponseEvaluationTools::calculateResidualDistributionAgreement(
    const std::vector<cv::Point2d> &points,
    const WeldLineSegment &line,
    double maxResidualPixel)
{
    if (points.empty() || !line.valid || maxResidualPixel <= 0.0)
    {
        return 0.0;
    }

    double residualSum = 0.0;
    for (const cv::Point2d &point : points)
    {
        residualSum += std::min(
            maxResidualPixel,
            pointLineDistance(point, line));
    }
    const double meanResidual =
        residualSum / static_cast<double>(points.size());
    return clampRatio(1.0 - meanResidual / maxResidualPixel);
}

double CStripeResponseEvaluationTools::calculateMeanResponseScore(
    const std::vector<double> &responseByColumn,
    double normalizationValue)
{
    if (responseByColumn.empty() || normalizationValue <= 0.0)
    {
        return 0.0;
    }

    double sum = 0.0;
    int count = 0;
    for (double response : responseByColumn)
    {
        if (response <= 0.0)
        {
            continue;
        }
        sum += std::min(normalizationValue, response);
        ++count;
    }
    if (count == 0)
    {
        return 0.0;
    }
    return clampRatio((sum / static_cast<double>(count)) / normalizationValue);
}

} // namespace i2r_rlc::vision
