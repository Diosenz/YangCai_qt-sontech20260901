/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      WeldmentVirtualPointCloudGenerator.h
 * @brief     weldment 工件模型虚拟参考点云生成器。
 * @details   根据 .weldment 文件（STEP 几何 + 尾部 [weldment_parameters] 配置）
 *            对定位基准面进行 UV 随机采样，生成变位机 TCP 坐标系下的 mm 虚拟点云。
 *            采样与坐标变换语义对齐旧 SWv1 工程 stepICPforOffline() 与
 *            ComTest/virPoints.cpp 的离线验证实现。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 8 月 8 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_WELDMENT_VIRTUAL_POINT_CLOUD_GENERATOR_H
#define I2R_RLC_DIGITAL_TWIN_WELDMENT_VIRTUAL_POINT_CLOUD_GENERATOR_H

#include <cstddef>
#include <string>
#include <vector>

#include "LocalizationTypes.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief weldment 模型采样结果。
 */
struct WeldmentVirtualPointCloudResult {
    /// @brief 是否成功生成。
    bool ok = false;
    /// @brief 失败原因或诊断说明。
    std::string message;
    /// @brief 全部定位基准面合并后的虚拟点云，单位 mm，坐标系为变位机 TCP。
    std::vector<localization::Point3D_mm> points_mm;
    /// @brief 每个定位段采样点数，用于诊断。
    /// @brief 每个定位段独立的虚拟点云，段序与 position_face_dof_list 一致，单位 mm。
    std::vector<std::vector<localization::Point3D_mm>> segmentPoints_mm;
    std::vector<std::size_t> segmentPointCounts;
    /// @brief 每个定位段对应的 ICP 自由度掩码（由段配置 x/y/z/rx/ry/rz 转换）。
    std::vector<localization::IcpDofMask> segmentDofMasks;
    /// @brief 独立定位基准面数量。
    std::size_t referenceFaceCount = 0;
};

/**
 * @brief 基于 OpenCASCADE 的 weldment 虚拟点云生成器。
 *
 * 本类只负责从模型文件生成参考点云，不执行 ICP，不访问真实机器人或相机。
 */
class WeldmentVirtualPointCloudGenerator final {
public:
    WeldmentVirtualPointCloudGenerator() = default;
    ~WeldmentVirtualPointCloudGenerator() = default;

    WeldmentVirtualPointCloudGenerator(const WeldmentVirtualPointCloudGenerator&) = delete;
    WeldmentVirtualPointCloudGenerator& operator=(const WeldmentVirtualPointCloudGenerator&) = delete;

    /**
     * @brief 生成虚拟参考点云。
     *
     * @param weldmentPath .weldment 文件路径，支持相对与绝对路径。
     * @param sampleRate  每平方毫米采样点数，默认 0.03，与原工程一致。
     * @return 采样结果；文件缺失、模型解析失败或没有定位面时 ok=false。
     */
    WeldmentVirtualPointCloudResult generate(
        const std::string& weldmentPath,
        double sampleRate = 0.03) const;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_WELDMENT_VIRTUAL_POINT_CLOUD_GENERATOR_H
