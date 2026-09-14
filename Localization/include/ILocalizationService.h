/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      ILocalizationService.h
 * @brief     定位服务抽象接口。
 * @details   本文件定义全局定位和局部修正共用的定位服务入口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_I_LOCALIZATION_SERVICE_H
#define I2R_RLC_LOCALIZATION_I_LOCALIZATION_SERVICE_H

#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 定位服务接口。
 *
 * 该接口承载全局定位、局部定位和 PCL 点云配准算法边界。公共接口只使用
 * localization 模块的公开数据结构不暴露 PCL 类型，避免点云库依赖扩散到 UI、control 或 digital_twin。
 */
class ILocalizationService {
public:
    virtual ~ILocalizationService() = default;

    /**
     * @brief 启动或声明服务状态。
     */
    virtual LocalizationStartResult start() = 0;

    /**
     * @brief 执行一次定位请求。
     */
    virtual LocalizationResult runLocalization(const LocalizationRequest& request) = 0;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_I_LOCALIZATION_SERVICE_H
