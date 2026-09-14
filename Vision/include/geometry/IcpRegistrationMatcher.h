/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        IcpRegistrationMatcher.h
 * @brief       libicp 配准封装接口
 * @details     对旧 libicp 的 point-to-point、point-to-plane、affine、BDD
 *              和 scale 配准能力做现代 C++ 封装。输入输出使用 Vision 几何类型，
 *              不向外暴露旧矩阵和 kd-tree 实现。
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
#ifndef I2R_RLC_VISION_GEOMETRY_ICP_REGISTRATION_MATCHER_H
#define I2R_RLC_VISION_GEOMETRY_ICP_REGISTRATION_MATCHER_H

#include <vector>

#include "geometry/IcpTypes.h"
#include "geometry/WeldGeometryTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief ICP 轨迹/点云配准器。
 *
 * libicp 内部公式为 `target = R * source + t`。因此本接口统一使用
 * `targetPoints` 表示模型点集，`sourcePoints` 表示待配准点集。
 */
class CIcpRegistrationMatcher {
public:
    /**
     * @brief 点到点 ICP。
     * @param targetPoints 目标/模型点集，坐标单位为 mm，仅使用 `valid=true` 的点。
     * @param sourcePoints 待配准点集，坐标单位为 mm，仅使用 `valid=true` 的点。
     * @param options ICP 维度、迭代次数、收敛阈值和内点距离等选项。
     * @return 配准结果；失败时 `success=false` 且 `message` 给出原因。
     */
    static IcpRegistrationResult alignPointToPoint(
        const std::vector<WeldSpacePoint> &targetPoints,
        const std::vector<WeldSpacePoint> &sourcePoints,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 点到面 ICP。
     * @param targetPoints 目标/模型点集，坐标单位为 mm，并用于估计局部法向。
     * @param sourcePoints 待配准点集，坐标单位为 mm。
     * @param options ICP 选项，其中 normalNeighborCount/normalFlatness 控制法向估计。
     * @return 配准结果；失败时 `success=false` 且 `message` 给出原因。
     */
    static IcpRegistrationResult alignPointToPlane(
        const std::vector<WeldSpacePoint> &targetPoints,
        const std::vector<WeldSpacePoint> &sourcePoints,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 仿射 ICP。
     * @param targetPoints 目标/模型点集，坐标单位为 mm。
     * @param sourcePoints 待配准点集，坐标单位为 mm。
     * @param options ICP 选项；旧 libicp affine 分支仅支持二维输入。
     * @return 配准结果，包含仿射矩阵和残差统计。
     */
    static IcpRegistrationResult alignAffine(
        const std::vector<WeldSpacePoint> &targetPoints,
        const std::vector<WeldSpacePoint> &sourcePoints,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 双向距离约束仿射 ICP。
     * @param targetPoints 目标/模型点集，坐标单位为 mm。
     * @param sourcePoints 待配准点集，坐标单位为 mm。
     * @param options ICP 选项；旧 libicp affineBDD 分支仅支持二维输入。
     * @return 配准结果，包含双向距离残差统计。
     */
    static IcpRegistrationResult alignAffineBidirectionalDistance(
        const std::vector<WeldSpacePoint> &targetPoints,
        const std::vector<WeldSpacePoint> &sourcePoints,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 双向距离约束刚体 ICP。
     * @param targetPoints 目标/模型点集，坐标单位为 mm。
     * @param sourcePoints 待配准点集，坐标单位为 mm。
     * @param options ICP 选项，inlierDistanceMm 用于内点筛选。
     * @return 配准结果，包含刚体矩阵和残差统计。
     */
    static IcpRegistrationResult alignBidirectionalDistance(
        const std::vector<WeldSpacePoint> &targetPoints,
        const std::vector<WeldSpacePoint> &sourcePoints,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 带尺度估计的点到点 ICP。
     * @param targetPoints 目标/模型点集，坐标单位为 mm。
     * @param sourcePoints 待配准点集，坐标单位为 mm。
     * @param options ICP 选项；scale 会写入结果的 `scale` 字段。
     * @return 配准结果，包含旋转、平移和尺度估计。
     */
    static IcpRegistrationResult alignWithScale(
        const std::vector<WeldSpacePoint> &targetPoints,
        const std::vector<WeldSpacePoint> &sourcePoints,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 使用 WeldPointCloud 作为输入的点到点 ICP。
     * @param targetPointCloud 目标/模型点云，内部点坐标单位为 mm。
     * @param sourcePointCloud 待配准点云，内部点坐标单位为 mm。
     * @param options ICP 维度、迭代次数、收敛阈值和内点距离等选项。
     * @return 配准结果；失败时 `success=false` 且 `message` 给出原因。
     */
    static IcpRegistrationResult alignPointToPoint(
        const WeldPointCloud &targetPointCloud,
        const WeldPointCloud &sourcePointCloud,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 使用 WeldPointCloud 作为输入的点到面 ICP。
     * @param targetPointCloud 目标/模型点云，内部点坐标单位为 mm。
     * @param sourcePointCloud 待配准点云，内部点坐标单位为 mm。
     * @param options ICP 选项，其中 normalNeighborCount/normalFlatness 控制法向估计。
     * @return 配准结果；失败时 `success=false` 且 `message` 给出原因。
     */
    static IcpRegistrationResult alignPointToPlane(
        const WeldPointCloud &targetPointCloud,
        const WeldPointCloud &sourcePointCloud,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 使用 WeldPointCloud 作为输入的仿射 ICP。
     * @param targetPointCloud 目标/模型点云，内部点坐标单位为 mm。
     * @param sourcePointCloud 待配准点云，内部点坐标单位为 mm。
     * @param options ICP 选项；旧 libicp affine 分支仅支持二维输入。
     * @return 配准结果，包含仿射矩阵和残差统计。
     */
    static IcpRegistrationResult alignAffine(
        const WeldPointCloud &targetPointCloud,
        const WeldPointCloud &sourcePointCloud,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 使用 WeldPointCloud 作为输入的双向距离约束仿射 ICP。
     * @param targetPointCloud 目标/模型点云，内部点坐标单位为 mm。
     * @param sourcePointCloud 待配准点云，内部点坐标单位为 mm。
     * @param options ICP 选项；旧 libicp affineBDD 分支仅支持二维输入。
     * @return 配准结果，包含双向距离残差统计。
     */
    static IcpRegistrationResult alignAffineBidirectionalDistance(
        const WeldPointCloud &targetPointCloud,
        const WeldPointCloud &sourcePointCloud,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 使用 WeldPointCloud 作为输入的双向距离约束刚体 ICP。
     * @param targetPointCloud 目标/模型点云，内部点坐标单位为 mm。
     * @param sourcePointCloud 待配准点云，内部点坐标单位为 mm。
     * @param options ICP 选项，inlierDistanceMm 用于内点筛选。
     * @return 配准结果，包含刚体矩阵和残差统计。
     */
    static IcpRegistrationResult alignBidirectionalDistance(
        const WeldPointCloud &targetPointCloud,
        const WeldPointCloud &sourcePointCloud,
        const IcpRegistrationOptions &options = {});

    /**
     * @brief 使用 WeldPointCloud 作为输入的带尺度估计 ICP。
     * @param targetPointCloud 目标/模型点云，内部点坐标单位为 mm。
     * @param sourcePointCloud 待配准点云，内部点坐标单位为 mm。
     * @param options ICP 选项；scale 会写入结果的 `scale` 字段。
     * @return 配准结果，包含旋转、平移和尺度估计。
     */
    static IcpRegistrationResult alignWithScale(
        const WeldPointCloud &targetPointCloud,
        const WeldPointCloud &sourcePointCloud,
        const IcpRegistrationOptions &options = {});
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_GEOMETRY_ICP_REGISTRATION_MATCHER_H
