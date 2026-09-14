/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      ILocalizationVirtualReferenceProvider.h
 * @brief     定位模块虚拟参考数据提供接口。
 * @details   本文件定义定位流程从仿真或离线数据源获取虚拟点云和虚拟激光线的边界。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_I_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H
#define I2R_RLC_LOCALIZATION_I_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H

#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 虚拟参考数据提供接口。
 */
class ILocalizationVirtualReferenceProvider {
public:
    virtual ~ILocalizationVirtualReferenceProvider() = default;

    /**
     * @brief 请求一帧定位可消费的虚拟参考快照。
     */
    virtual LocalizationVirtualReferenceSnapshot requestSnapshot(
        const LocalizationVirtualReferenceRequest& request) = 0;
};

/**
 * @brief 空虚拟参考数据提供器。
 */
class NullLocalizationVirtualReferenceProvider final
    : public ILocalizationVirtualReferenceProvider {
public:
    /**
     * @brief 返回不可用快照，用于未接入仿真服务时的显式失败路径。
     */
    LocalizationVirtualReferenceSnapshot requestSnapshot(
        const LocalizationVirtualReferenceRequest& request) override;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_I_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H
