/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      WeldmentLocalizationVirtualReferenceProvider.h
 * @brief     基于 weldment 模型的 Localization 虚拟参考提供器。
 * @details   本类把 .weldment 工件模型采样得到的虚拟点云包装为
 *            Localization 可消费的 LocalizationVirtualReferenceSnapshot，
 *            供全局 ICP 定位使用，替代 RobotSim 在线参考的离线路径。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 8 月 8 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_WELDMENT_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H
#define I2R_RLC_DIGITAL_TWIN_WELDMENT_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H

#include <string>

#include "ILocalizationVirtualReferenceProvider.h"
#include "WeldmentVirtualPointCloudGenerator.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief 面向 Localization 的 weldment 虚拟参考适配器。
 *
 * 该提供器使用 WeldmentVirtualPointCloudGenerator 从 .weldment 文件生成
 * 定位基准面虚拟点云，并填充到快照的 workpiecePointCloud_mm 字段。
 */
class WeldmentLocalizationVirtualReferenceProvider final
    : public localization::ILocalizationVirtualReferenceProvider {
public:
    /**
     * @brief 构造提供器。
     * @param weldmentPath .weldment 文件路径。
     * @param sampleRate   每平方毫米采样点数，默认 0.03。
     */
    explicit WeldmentLocalizationVirtualReferenceProvider(
        std::string weldmentPath,
        double sampleRate = 0.03);

    /**
     * @brief 请求一帧定位可消费的虚拟参考快照。
     */
    localization::LocalizationVirtualReferenceSnapshot requestSnapshot(
        const localization::LocalizationVirtualReferenceRequest& request) override;

private:
    std::string m_weldmentPath;
    WeldmentVirtualPointCloudGenerator m_generator;
    double m_sampleRate = 0.03;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_WELDMENT_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H
