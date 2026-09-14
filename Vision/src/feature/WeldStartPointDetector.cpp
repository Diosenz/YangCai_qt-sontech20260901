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
 * @file        WeldStartPointDetector.cpp
 * @brief       焊缝起点检测实现
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
#include <algorithm>
#include <cmath>

#include "feature/WeldStartPointDetector.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 将比例或置信度夹紧到 [0,1]。
 * @param value 原始数值。
 * @return 归一化后的比例。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 判断起点检测输入图像是否为有效灰度图。
 * @param gray8 输入图像。
 * @return 图像非空且类型为 CV_8UC1 时返回 true。
 */
bool isSupportedGray(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 构造焊缝起点/端点候选。
 * @param point 输入点，坐标单位为 pixel。
 * @param label 输出特征标签。
 * @param confidence 原始置信度。
 * @return 标记为有效的焊缝特征点。
 */
WeldFeaturePoint makePoint(
    const cv::Point2d &point,
    const std::string &label,
    double confidence)
{
    WeldFeaturePoint featurePoint;
    featurePoint.xPixel = point.x;
    featurePoint.yPixel = point.y;
    featurePoint.confidence = clampRatio(confidence);
    featurePoint.valid = true;
    featurePoint.label = label;
    return featurePoint;
}

/**
 * @brief 计算两个焊缝特征点之间的欧氏距离。
 * @param lhs 第一个点，坐标单位为 pixel。
 * @param rhs 第二个点，坐标单位为 pixel。
 * @return 两点距离，单位为 pixel。
 */
double distancePixel(const WeldFeaturePoint &lhs, const WeldFeaturePoint &rhs)
{
    const double dx = lhs.xPixel - rhs.xPixel;
    const double dy = lhs.yPixel - rhs.yPixel;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 判断跨帧状态中是否已有可用于回退的端点历史。
 * @param state 可选跨帧状态对象。
 * @return 存在历史端点时返回 true。
 */
bool hasStableHistory(const WeldStartPointState *state)
{
    return state != nullptr && !state->recentEndpointResults.empty();
}

/**
 * @brief 用两个特征点组装端点检测结果。
 * @param startPoint 起点候选。
 * @param endPoint 终点候选。
 * @param confidence 端点对整体置信度。
 * @return 端点结果，只有两点有效且置信度大于 0 时 valid。
 */
WeldEndpointResult makeEndpointPair(
    const WeldFeaturePoint &startPoint,
    const WeldFeaturePoint &endPoint,
    double confidence)
{
    WeldEndpointResult endpoint;
    endpoint.startPoint = startPoint;
    endpoint.endPoint = endPoint;
    endpoint.confidence = clampRatio(confidence);
    endpoint.valid = startPoint.valid && endPoint.valid
        && endpoint.confidence > 0.0;
    endpoint.message = endpoint.valid
        ? "endpoint pair detected"
        : "endpoint pair invalid";
    return endpoint;
}

/**
 * @brief 将有效端点写入有限长度历史队列。
 * @param endpoint 当前端点结果。
 * @param historySize 最大历史长度。
 * @param state 可选跨帧状态对象，生命周期由调用方持有。
 */
void pushHistory(
    const WeldEndpointResult &endpoint,
    int historySize,
    WeldStartPointState *state)
{
    if (state == nullptr || !endpoint.valid)
    {
        return;
    }
    state->recentEndpointResults.push_back(endpoint);
    const int limit = std::max(1, historySize);
    while (static_cast<int>(state->recentEndpointResults.size()) > limit)
    {
        state->recentEndpointResults.pop_front();
    }
}

/**
 * @brief 从历史端点中构造回退结果。
 * @param state 跨帧状态对象。
 * @param confidence 回退结果置信度。
 * @return 最近一次有效端点；无历史时返回 invalid 结果。
 */
WeldEndpointResult fallbackFromHistory(
    const WeldStartPointState *state,
    double confidence)
{
    if (!hasStableHistory(state))
    {
        return WeldEndpointResult();
    }
    WeldEndpointResult endpoint = state->recentEndpointResults.back();
    endpoint.confidence = clampRatio(confidence);
    endpoint.message = "endpoint fallback from history";
    endpoint.valid = endpoint.confidence > 0.0;
    return endpoint;
}

/**
 * @brief 使用历史端点过滤单帧异常跳变。
 * @param current 当前端点结果。
 * @param options 跳变阈值配置，单位为 pixel。
 * @param state 跨帧状态对象。
 * @return 通过过滤的当前结果或降置信度历史回退结果。
 */
WeldEndpointResult filterJump(
    const WeldEndpointResult &current,
    const WeldStartPointOptions &options,
    const WeldStartPointState *state)
{
    if (!current.valid || !hasStableHistory(state))
    {
        return current;
    }

    const WeldEndpointResult &last = state->recentEndpointResults.back();
    const double startJump = distancePixel(current.startPoint, last.startPoint);
    const double endJump = distancePixel(current.endPoint, last.endPoint);
    if (startJump <= options.maxEndpointJumpPixel
        && endJump <= options.maxEndpointJumpPixel)
    {
        return current;
    }

    WeldEndpointResult fallback = last;
    fallback.confidence = clampRatio(last.confidence * 0.80);
    fallback.message = "endpoint jump filtered by history";
    return fallback;
}

/**
 * @brief 从主动轮廓结果推导起止端点。
 * @param activeResult 主动轮廓焊缝检测结果。
 * @param options 起点检测配置。
 * @return 主动轮廓端点候选；曲线不足时返回 invalid 结果。
 */
WeldEndpointResult deriveEndpointFromActiveContour(
    const ActiveContourDetectionResult &activeResult,
    const WeldStartPointOptions &options)
{
    if (!activeResult.valid && activeResult.activeCurve.curvePoints.size() < 2U)
    {
        return WeldEndpointResult();
    }

    // 1. 对接：优先使用已迁移 gap 扫描端点，承接 ButtEndPtsFinding 的
    // gap_situ 主语义；缺少 gap 时再退回曲线首尾。
    if (activeResult.seamType == WeldSeamType::Butt
        && activeResult.leftGapPoint.valid
        && activeResult.rightGapPoint.valid)
    {
        return makeEndpointPair(
            activeResult.leftGapPoint,
            activeResult.rightGapPoint,
            std::max(0.20, activeResult.confidence));
    }

    const std::vector<cv::Point2d> &curve = activeResult.activeCurve.curvePoints;
    if (curve.size() < 2U)
    {
        return WeldEndpointResult();
    }

    WeldFeaturePoint startPoint = makePoint(
        curve.front(),
        options.method == WeldStartPointMethod::AreaChange
            ? "area_start_endpoint"
            : "curve_start_endpoint",
        std::max(0.20, activeResult.confidence));
    WeldFeaturePoint endPoint = makePoint(
        curve.back(),
        options.method == WeldStartPointMethod::AreaChange
            ? "area_end_endpoint"
            : "curve_end_endpoint",
        std::max(0.20, activeResult.confidence));

    // 2. 搭接：旧实现会在特征点中找最大 row 候选。主动轮廓已输出该语义点；
    // 端点仍保持曲线首尾，weldPoint 由结果字段承载。
    if (activeResult.seamType == WeldSeamType::Lap
        && !activeResult.featurePoints.empty())
    {
        const auto maxRowIt = std::max_element(
            activeResult.featurePoints.begin(),
            activeResult.featurePoints.end(),
            [](const WeldFeaturePoint &lhs, const WeldFeaturePoint &rhs) {
                return lhs.yPixel < rhs.yPixel;
            });
        if (maxRowIt != activeResult.featurePoints.end()
            && maxRowIt->valid)
        {
            startPoint.confidence =
                std::max(startPoint.confidence, maxRowIt->confidence * 0.8);
            endPoint.confidence =
                std::max(endPoint.confidence, maxRowIt->confidence * 0.8);
        }
    }

    return makeEndpointPair(
        startPoint,
        endPoint,
        std::max(startPoint.confidence, endPoint.confidence));
}

} // namespace

WeldStartPointResult CWeldStartPointDetector::detect(
    const cv::Mat &gray8,
    const WeldStartPointOptions &options,
    WeldStartPointState *state)
{
    WeldStartPointResult result;
    if (!isSupportedGray(gray8))
    {
        result.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return result;
    }
    if (state == nullptr)
    {
        result.message = "start point state is null";
        return result;
    }

    // 1. 调主动轮廓：旧端点入口依赖 NURBS/BSnake 曲线和 gap/线交点结果，
    // 新实现复用已迁移的主动轮廓 runtime 行为。
    const ActiveContourDetectionResult activeResult =
        CActiveContourWeldFeatureDetector::detect(
            gray8,
            options.seamMode,
            options.activeContourOptions,
            &state->activeContourState);
    result.activeContourResult = activeResult;
    result.seamType = activeResult.seamType;
    result.weldPoint = activeResult.weldPoint;
    result.leftGapPoint = activeResult.leftGapPoint;
    result.rightGapPoint = activeResult.rightGapPoint;
    state->gapSituation =
        activeResult.leftGapPoint.valid && activeResult.rightGapPoint.valid ? 1 : 0;

    // 2. 由曲线/gap/特征点生成端点；跳变时用显式历史替代旧全局点队列。
    WeldEndpointResult endpoint =
        deriveEndpointFromActiveContour(activeResult, options);
    endpoint = filterJump(endpoint, options, state);
    if (!endpoint.valid)
    {
        endpoint = fallbackFromHistory(state, 0.20);
    }

    result.startPoint = endpoint.startPoint;
    result.endPoint = endpoint.endPoint;
    result.confidence = clampRatio(
        0.65 * endpoint.confidence + 0.35 * activeResult.confidence);
    result.valid = endpoint.valid && result.confidence >= options.minConfidence;
    result.message = result.valid
        ? "weld start/end point detected"
        : "weld start/end point detection failed: " + activeResult.message;

    if (result.valid)
    {
        pushHistory(endpoint, options.historySize, state);
    }
    return result;
}

} // namespace i2r_rlc::vision
