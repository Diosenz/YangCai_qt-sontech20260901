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
 * @file        WeldFeatureTypes.h
 * @brief       焊缝特征数据类型
 * @details     提供 gap、端点、坡口和主动轮廓焊缝语义检测共享的数据结构。
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
#ifndef I2R_RLC_VISION_FEATURE_WELD_FEATURE_TYPES_H
#define I2R_RLC_VISION_FEATURE_WELD_FEATURE_TYPES_H

#include <string>

namespace i2r_rlc::vision {

/**
 * @brief 当前 vision/feature 支持的焊缝语义类型。
 *
 *  Unknown、Butt、TJoint 与 Lap。
 */
enum class WeldSeamType {
    Unknown = 0,
    Butt = 1,
    TJoint = 2,
    Lap = 3
};

/**
 * @brief 焊接视觉特征点。
 *
 * 坐标单位为 pixel，confidence 按 0~1 表达。
 */
struct WeldFeaturePoint {
    /// @brief X 坐标，单位 pixel。
    double xPixel = 0.0;

    /// @brief Y 坐标，单位 pixel。
    double yPixel = 0.0;

    /// @brief 点级置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示该点可用于下游焊接特征计算。
    bool valid = false;

    /// @brief 特征标签，例如 left_edge、right_edge、weld_point。
    std::string label;
};

/**
 * @brief gap 左右边缘的中间候选。
 *
 * 坐标单位为 pixel，
 * contrastGrayValue 为 0~255 灰度差，confidence 按 0~1 表达。
 */
struct GapEdgeCandidate {
    /// @brief 候选 X 坐标，单位 pixel。
    double xPixel = 0.0;

    /// @brief 候选 Y 坐标，单位 pixel。
    double yPixel = 0.0;

    /// @brief 候选边缘对比度，范围 0~255。
    double contrastGrayValue = 0.0;

    /// @brief 候选置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示该候选可参与左右边缘配对。
    bool valid = false;

    /// @brief 候选标签，例如 left_edge_candidate、right_edge_candidate。
    std::string label;
};

/**
 * @brief 一组左右边缘候选。
 *
 * widthPixel 为左右边缘的像素距离，用于当前 V2-A2 最小 gap 候选。
 */
struct WeldEdgePair {
    /// @brief 左侧边缘候选点。
    WeldFeaturePoint left;

    /// @brief 右侧边缘候选点。
    WeldFeaturePoint right;

    /// @brief 左右边缘宽度，单位 pixel。
    double widthPixel = 0.0;

    /// @brief true 表示左右边缘顺序和宽度均满足当前选项。
    bool valid = false;

    /// @brief 边缘对置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 间隙/坡口基础候选结果。
 */
struct GapFeatureResult {
    /// @brief true 表示得到有效左右边缘和焊缝点候选。
    bool valid = false;

    /// @brief 基于左右边缘中点估计的焊缝点。
    WeldFeaturePoint weldPoint;

    /// @brief 左右边缘候选。
    WeldEdgePair edgePair;

    /// @brief 间隙宽度，单位 pixel。
    double gapWidthPixel = 0.0;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 左侧边缘候选数量。
    int leftCandidateCount = 0;

    /// @brief 右侧边缘候选数量。
    int rightCandidateCount = 0;

    /// @brief 选中边缘所在行，单位 pixel；无有效结果时为 -1。
    int selectedRowPixel = -1;

    /// @brief 选中左右边缘的平均对比度，范围 0~255。
    double edgeContrastGrayValue = 0.0;

    /// @brief 机器可读失败原因或状态码，不使用平台相关字符串类型。
    std::string failureReason;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 焊缝起点/终点候选结果。
 *
 * 本结构只表达图像坐标系下的 2D endpoint 候选。
 */
struct WeldEndpointResult {
    /// @brief true 表示起点和终点候选均可用于下游特征计算。
    bool valid = false;

    /// @brief 起点候选，单位 pixel。
    WeldFeaturePoint startPoint;

    /// @brief 终点候选，单位 pixel。
    WeldFeaturePoint endPoint;

    /// @brief 起止点整体置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 坡口/复杂间隙左右边缘候选结果。
 *
 * 本结构只表达 2D 图像坐标中的坡口边缘候选。。
 */
struct GrooveEdgeResult {
    /// @brief true 表示得到可用于下游坡口语义的左右边缘候选。
    bool valid = false;

    /// @brief 左侧坡口/间隙边缘代表点，单位 pixel。
    WeldFeaturePoint leftEdge;

    /// @brief 右侧坡口/间隙边缘代表点，单位 pixel。
    WeldFeaturePoint rightEdge;

    /// @brief 左右边缘中点，单位 pixel。
    WeldFeaturePoint centerPoint;

    /// @brief 左右边缘平均宽度，单位 pixel。
    double widthPixel = 0.0;

    /// @brief 左右边缘平均对比度，范围 0~255。
    double edgeContrastGrayValue = 0.0;

    /// @brief 参与结果选择的有效边缘对数量。
    int edgePairCount = 0;

    /// @brief 左右拟合线夹角，单位 degree；平行退化时为 0。
    double lineAngleDegree = 0.0;

    /// @brief 当前最小版本的轮廓标签，例如 v_groove 或 parallel_gap。
    std::string profileLabel;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 焊接特征检测统一结果。
 */
struct WeldFeatureResult {
    /// @brief true 表示至少得到当前阶段支持的有效特征。
    bool valid = false;

    /// @brief 当前阶段支持的 gap/edge/weld point 候选。
    GapFeatureResult gapFeature;

    /// @brief 当前阶段支持的起点/终点候选。
    WeldEndpointResult endpointResult;

    /// @brief 当前阶段支持的坡口/复杂间隙边缘辅助候选。
    GrooveEdgeResult grooveFeature;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 焊接特征检测参数。
 */
struct WeldFeatureDetectionOptions {
    /// @brief 中心线行附近的搜索半窗口，单位 pixel。
    int searchHalfWindowY = 4;

    /// @brief 认为是边缘的最小一维灰度梯度，范围 0~255。
    int minEdgeContrast = 40;

    /// @brief 最大间隙宽度，单位 pixel。
    int maxGapWidthPixel = 80;

    /// @brief 最小间隙宽度，单位 pixel。
    int minGapWidthPixel = 4;

    /// @brief 低于该置信度时输出 invalid，范围 0~1。
    double minConfidence = 0.10;

    /// @brief 是否要求使用中心线作为横向位置先验。
    bool useCenterlinePrior = true;

    /// @brief 中心线两侧最大搜索半宽，单位 pixel；小于等于 0 时搜索整行。
    int edgeSearchHalfWidthPixel = 120;

    /// @brief endpoint 检测所需的最少有效中心线点数。
    int minEndpointPointCount = 2;

    /// @brief endpoint 与焊缝点之间建议保持的最小距离，单位 pixel。
    double minEndpointDistanceFromWeldPointPixel = 3.0;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_WELD_FEATURE_TYPES_H

