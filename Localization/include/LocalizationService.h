/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationService.h
 * @brief     定位服务实现入口。
 * @details   本文件声明基于受限 ICP 的全局定位和局部修正服务。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_SERVICE_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_SERVICE_H

#include "ILocalizationService.h"

namespace i2r_rlc::localization {

/**
 * @brief 定位模块真实服务实现。
 *
 * 本实现提供全局受限 ICP 和局部滑窗 ICP 修正。公共接口只接收
 * localization 值对象，PCL 点云和自由度 warp 均隔离在实现文件中。
 */
class LocalizationService final : public ILocalizationService {
public:
    LocalizationStartResult start() override;
    LocalizationResult runLocalization(const LocalizationRequest& request) override;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_SERVICE_H
