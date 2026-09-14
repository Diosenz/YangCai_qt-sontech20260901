/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullLocalizationService.cpp
 * @brief     定位空服务实现。
 * @details   本文件实现定位服务不可用时的显式空实现行为。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include "NullLocalizationService.h"

namespace i2r_rlc::localization {

LocalizationStartResult NullLocalizationService::start()
{
    return {
        LocalizationServiceState::Unavailable,
        QStringLiteral("LocalizationService 不可用：未启用定位算法服务。")
    };
}

LocalizationResult NullLocalizationService::runLocalization(const LocalizationRequest& request)
{
    LocalizationResult result;
    result.mode = request.mode;
    result.frameId = request.frameId;
    result.status = LocalizationStatus::AlgorithmNotImplemented;
    result.realPointCount = request.realPointCount;
    result.virtualPointCount = request.virtualPointCount;
    result.hasPoseCorrection = false;
    result.message = QStringLiteral("定位服务不可用：未执行全局或局部定位算法。");

    return result;
}

} // namespace i2r_rlc::localization
