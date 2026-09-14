/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsStartPointDetector.h
 * @brief     焊缝起始点几何计算。
 * @details   本文件根据焊缝点、辅助边线或曲线点计算 IBVS 所需的起止点。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_START_POINT_DETECTOR_H
#define I2R_RLC_IBVS_IBVS_START_POINT_DETECTOR_H

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 2D 直线模型，形式为 a*x + b*y + c = 0。
 */
struct IbvsLineModel
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    bool valid = false;
};

/**
 * @brief 焊缝起始点计算结果。
 */
struct IbvsStartPointResult
{
    IbvsError error;
    PixelPoint startPoint;
    PixelPoint endPoint;
    IbvsLineModel line;
};

/**
 * @brief 焊缝起始点几何计算入口。
 */
class IbvsStartPointDetector
{
public:
    /**
     * @brief 对已提取焊缝点做 TLS 直线拟合。
     */
    static IbvsLineModel fitLineTls(const std::vector<PixelPoint>& points);

    /**
     * @brief 端点法：沿焊缝切向投影找两端，再按图像 x 方向选起点。
     */
    static IbvsStartPointResult computeEndpointStartPoint(
        const std::vector<PixelPoint>& weldSeamPoints,
        bool useLeftSide);

    /**
     * @brief 计算两条 2D 直线交点。
     */
    static IbvsStartPointResult computeIntersectionStartPoint(
        const IbvsLineModel& weldLine,
        const IbvsLineModel& secondEdgeLine);

    /**
     * @brief 从曲线焊缝点中选择靠近焊枪末端的起始点。
     */
    static IbvsStartPointResult computeCurveNearestStartPoint(
        const std::vector<PixelPoint>& curveWeldSeamPoints,
        const PixelPoint& torchEndPoint);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_START_POINT_DETECTOR_H
