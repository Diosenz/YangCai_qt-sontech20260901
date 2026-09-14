/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullLocalizationService.h
 * @brief     定位空服务实现。
 * @details   本文件定义定位服务不可用时使用的显式空实现。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_NULL_LOCALIZATION_SERVICE_H
#define I2R_RLC_LOCALIZATION_NULL_LOCALIZATION_SERVICE_H

#include "ILocalizationService.h"

namespace i2r_rlc::localization {

/**
 * @brief 定位空实现。
 *
 * 该实现用于定位服务不可用时保持接口可调用，并返回结构化不可用状态。
 */
class NullLocalizationService final : public ILocalizationService {
public:
    LocalizationStartResult start() override;
    LocalizationResult runLocalization(const LocalizationRequest& request) override;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_NULL_LOCALIZATION_SERVICE_H
