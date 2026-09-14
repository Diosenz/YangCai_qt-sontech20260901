/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      DigitalTwinLocalizationVirtualReferenceProvider.h
 * @brief     Localization 虚拟参考适配器。
 * @details   本文件定义 DigitalTwin 到 Localization 虚拟参考快照的转换边界，
 *            只搬运值对象，不执行定位求解或替代数据生成。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_DIGITAL_TWIN_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H
#define I2R_RLC_DIGITAL_TWIN_DIGITAL_TWIN_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H

#include "IDigitalTwinService.h"
#include "ILocalizationVirtualReferenceProvider.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief 面向 Localization 的数字孪生虚拟参考适配器。
 *
 * 本类只做 DigitalTwin facade 到 Localization 快照的值对象转换。服务返回
 * 不可用虚拟参考时，会透传 unavailable 结果，不生成替代数据。
 */
class DigitalTwinLocalizationVirtualReferenceProvider final
    : public localization::ILocalizationVirtualReferenceProvider {
public:
    explicit DigitalTwinLocalizationVirtualReferenceProvider(IDigitalTwinService& service);

    localization::LocalizationVirtualReferenceSnapshot requestSnapshot(
        const localization::LocalizationVirtualReferenceRequest& request) override;

private:
    IDigitalTwinService& m_service;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_DIGITAL_TWIN_LOCALIZATION_VIRTUAL_REFERENCE_PROVIDER_H
