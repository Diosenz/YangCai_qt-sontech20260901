/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullLocalizationVirtualReferenceProvider.cpp
 * @brief     空虚拟参考数据提供器实现。
 * @details   本文件提供未接入仿真服务时的显式不可用返回。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include "ILocalizationVirtualReferenceProvider.h"

namespace i2r_rlc::localization {

LocalizationVirtualReferenceSnapshot
NullLocalizationVirtualReferenceProvider::requestSnapshot(
    const LocalizationVirtualReferenceRequest& request)
{
    LocalizationVirtualReferenceSnapshot snapshot;
    snapshot.available = false;
    snapshot.referenceId = request.referenceId;
    snapshot.referencePose = request.workpiecePose;
    snapshot.message = QStringLiteral("虚拟参考数据源未接入。");
    return snapshot;
}

} // namespace i2r_rlc::localization
