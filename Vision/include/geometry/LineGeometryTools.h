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
 * @file        LineGeometryTools.h
 * @brief       线段几何计算工具
 * @details     提供焊接视觉中的二维几何、投影和线段计算能力，输入输出统一使用像素或空间坐标结构。
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
#ifndef I2R_RLC_VISION_GEOMETRY_LINE_GEOMETRY_TOOLS_H
#define I2R_RLC_VISION_GEOMETRY_LINE_GEOMETRY_TOOLS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 焊接特征点列拟合得到的二维线段。
 *
 * 坐标单位为 pixel。非竖直线使用 `y = slope * x + intercept`；
 * 竖直线 `vertical=true` 时，`intercept` 表示常量 x 坐标。
 * 该结构替代旧 `Ransac_line` / `returnLines` 中与 HALCON XLD 拟合相关
 * 的线段表达。
 */
struct WeldLineSegment {
    /// @brief 线段起点，单位 pixel。
    cv::Point2d start;

    /// @brief 线段终点，单位 pixel。
    cv::Point2d end;

    /// @brief 斜率；竖直线时为 0。
    double slope = 0.0;

    /// @brief 非竖直线为 y 截距；竖直线为常量 x。
    double intercept = 0.0;

    /// @brief 线段长度，单位 pixel。
    double lengthPixel = 0.0;

    /// @brief 拟合置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示线段拟合可用于下游特征语义。
    bool valid = false;

    /// @brief true 表示线段近似竖直。
    bool vertical = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 点列递归分段结果。
 *
 * 对应旧 `split_line` 的 `split_place` 输出。`splitIndices` 包含首尾点
 * 索引，可直接作为下游逐段 RANSAC 拟合的切分边界。
 */
struct WeldLineSplitResult {
    /// @brief 分段边界索引，包含首尾点索引。
    std::vector<int> splitIndices;

    /// @brief 递归过程中观察到的最大点线距离，单位 pixel。
    double maxDistancePixel = 0.0;

    /// @brief true 表示至少形成一个有效线段区间。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 两条线的交点结果。
 *
 * 对应旧 `line_interfast` 的交点输出。新实现显式携带夹角和状态，
 * 便于调用方区分平行、近似平行或样本不足等退化情况。
 */
struct WeldLineIntersectionResult {
    /// @brief 交点，单位 pixel。
    cv::Point2d point;

    /// @brief 两线夹角，单位 degree。
    double angleDegree = 0.0;

    /// @brief true 表示两条线存在稳定交点。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 分段拟合得到的线段集合。
 * 新结构统一保存每段线的起止点、斜率、截距、长度和置信度。
 */
struct WeldLineSegmentsResult {
    /// @brief 按点列顺序输出的有效线段。
    std::vector<WeldLineSegment> lines;

    /// @brief true 表示至少拟合出一条有效线段。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 线段分裂、RANSAC 拟合和交点计算工具。
 *
 * 本类迁移旧 `ImageProShare::split_line/ransac_line/line_interfast`
 * 的低阶几何语义，只处理点列和数值结构
 */
class CLineGeometryTools final {
public:
    /**
     * @brief 按点到首尾直线最大距离递归拆分折线。
     *
     * @param points 输入点列，坐标单位 pixel，需按曲线顺序排列。
     * @param maxDistancePixel 最大允许点线距离，单位 pixel。
     * @return 分段索引和最大偏离距离；点数不足时 `valid=false`。
     */
    static WeldLineSplitResult splitPolylineByDistance(
        const std::vector<WeldFeaturePoint> &points,
        double maxDistancePixel);

    /**
     * @brief 按最大点线距离递归拆分折线。
     *
     * 该接口是旧 `split_line` 来源语义在新工程中的通用命名入口：
     * 用首尾点连线寻找最大偏离点，超过阈值时递归切分。实现仍是
     * OpenCV/C++ 数值逻辑，不依赖 HALCON XLD。
     *
     * @param points 输入点列，坐标单位 pixel，需按曲线顺序排列。
     * @param maxDistancePixel 触发递归切分的最大偏离距离，单位 pixel。
     * @return 分段边界索引。
     */
    static WeldLineSplitResult splitPolylineByMaxDeviation(
        const std::vector<WeldFeaturePoint> &points,
        double maxDistancePixel);

    /**
     * @brief 按旧 `split_line` 分段规则拆分折线。
     *
     * 该接口保留旧函数“首尾直线最大距离递归分段”的调用语义，
     * 输出分段索引，便于下游按旧 `split_place` 方式拟合多段线。
     *
     * @param points 输入点列，坐标单位 pixel，需按曲线顺序排列。
     * @param maxDistancePixel 触发递归切分的最大偏离距离，单位 pixel。
     * @return 旧 `split_place` 等价分段索引。
     */
    static WeldLineSplitResult splitPolylineByLegacyRule(
        const std::vector<WeldFeaturePoint> &points,
        double maxDistancePixel);

    /**
     * @brief 使用确定性最小二乘拟合二维线段。
     *
     * @param points 输入点列，坐标单位 pixel。
     * @return 拟合线段；点数不足或退化时 `valid=false`。
     */
    static WeldLineSegment fitLineLeastSquares(
        const std::vector<WeldFeaturePoint> &points);

    /**
     * @brief 使用确定性采样的 RANSAC 拟合二维线段。
     *
     * 对应旧 `ransac_demo` 的鲁棒拟合语义，但不使用随机全局状态。
     *
     * @param points 输入点列，坐标单位 pixel。
     * @param iterations 确定性采样迭代次数。
     * @param distanceThresholdPixel 内点距离阈值，单位 pixel。
     * @param minInlierRatio 最小内点比例，范围 0~1。
     * @return 鲁棒拟合线段；内点不足时 `valid=false`。
     */
    static WeldLineSegment fitLineRansac(
        const std::vector<WeldFeaturePoint> &points,
        int iterations,
        double distanceThresholdPixel,
        double minInlierRatio);

    /**
     * @brief 对分段点列执行旧 `ransac_line` 风格的逐段拟合。
     *
     * @param points 输入点列，坐标单位 pixel。
     * @param splitIndices `splitPolylineByMaxDeviation` 输出的分段边界。
     * @param iterations 每段 RANSAC 迭代次数。
     * @param distanceThresholdPixel 内点距离阈值，单位 pixel。
     * @param minInlierRatio 最小内点比例，范围 0~1。
     * @return 多段线拟合结果；没有有效线段时 `valid=false`。
     */
    static WeldLineSegmentsResult fitSegmentsRansac(
        const std::vector<WeldFeaturePoint> &points,
        const std::vector<int> &splitIndices,
        int iterations,
        double distanceThresholdPixel,
        double minInlierRatio);

    /**
     * @brief 计算两条非平行线的交点。
     *
     * @param lineA 第一条有效线段。
     * @param lineB 第二条有效线段。
     * @return 交点和夹角；平行或输入无效时 `valid=false`。
     */
    static WeldLineIntersectionResult intersectLines(
        const WeldLineSegment &lineA,
        const WeldLineSegment &lineB);

    /**
     * @brief 按旧 `line_interfast` 语义选择接近参考列的两段线求交点。
     *
     * 旧函数会根据 `maxplace` 在多段线中选择参考列左右最合适的线段。
     * 新实现以 `referenceXPixel` 表达该参考列，优先选择跨越或最接近
     * 参考列的左右线段，再调用 `intersectLines`。
     *
     * @param lines 已拟合的线段集合，坐标单位 pixel。
     * @param referenceXPixel 参考列坐标，单位 pixel。
     * @return 参考列附近两段线的交点。
     */
    static WeldLineIntersectionResult intersectLinesNearReferenceX(
        const std::vector<WeldLineSegment> &lines,
        double referenceXPixel);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_GEOMETRY_LINE_GEOMETRY_TOOLS_H

