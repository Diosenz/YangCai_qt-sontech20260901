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
 * @file        LaserCenterlineTypes.h
 * @brief       激光中心线公共数据结构
 * @details     定义中心线检测参数、点和结果结构，供 DP 条纹检测、gap
 *              特征检测和 replay 评估共享。
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
#ifndef I2R_RLC_VISION_STRIPE_LASER_CENTERLINE_TYPES_H
#define I2R_RLC_VISION_STRIPE_LASER_CENTERLINE_TYPES_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief 激光中心线点。
 */
struct LaserCenterlinePoint {
    /// @brief 列坐标，单位 pixel。
    double xPixel = 0.0;

    /// @brief 行坐标，单位 pixel。
    double yPixel = 0.0;

    /// @brief 原始峰值响应，范围按检测器内部响应定义。
    double response = 0.0;

    /// @brief 局部对比度，范围按检测器内部响应定义。
    double localContrast = 0.0;

    /// @brief 点级置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示该点可参与中心线输出统计。
    bool valid = false;
};

/**
 * @brief 激光中心线检测结果。
 */
struct LaserCenterlineResult {
    /// @brief true 表示至少得到一条有效中心线。
    bool valid = false;

    /// @brief 中心线点列，按 x 从小到大排序。
    std::vector<LaserCenterlinePoint> points;

    /// @brief 实际检测 ROI，单位 pixel。
    cv::Rect roi;

    /// @brief 有效列比例，范围 0~1。
    double validColumnRatio = 0.0;

    /// @brief 断裂比例，范围 0~1。
    double brokenStripeRatio = 0.0;

    /// @brief 有效点平均置信度，范围 0~1。
    double meanConfidence = 0.0;

    /// @brief 处理耗时，单位 ms。
    double processingTimeMs = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_LASER_CENTERLINE_TYPES_H
