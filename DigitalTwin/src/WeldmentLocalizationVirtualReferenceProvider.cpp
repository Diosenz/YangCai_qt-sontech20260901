/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      WeldmentLocalizationVirtualReferenceProvider.cpp
 * @brief     基于 weldment 模型的 Localization 虚拟参考提供器实现。
 * @details   调用 WeldmentVirtualPointCloudGenerator 生成虚拟点云并填充快照。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 8 月 8 日
 */

#include "WeldmentLocalizationVirtualReferenceProvider.h"

#include <QString>

namespace i2r_rlc::digital_twin {

WeldmentLocalizationVirtualReferenceProvider::WeldmentLocalizationVirtualReferenceProvider(
    std::string weldmentPath,
    double sampleRate)
    : m_weldmentPath(std::move(weldmentPath))
    , m_sampleRate(sampleRate)
{
}

localization::LocalizationVirtualReferenceSnapshot
WeldmentLocalizationVirtualReferenceProvider::requestSnapshot(
    const localization::LocalizationVirtualReferenceRequest& request)
{
    localization::LocalizationVirtualReferenceSnapshot snapshot;
    snapshot.available = false;
    snapshot.referenceId = request.referenceId;
    snapshot.referencePose = request.workpiecePose;

    if (m_weldmentPath.empty()) {
        snapshot.message = QStringLiteral("weldment 模型路径为空。");
        return snapshot;
    }

    const WeldmentVirtualPointCloudResult cloudResult =
        m_generator.generate(m_weldmentPath, m_sampleRate);
    if (!cloudResult.ok || cloudResult.points_mm.empty()) {
        snapshot.message = QString::fromStdString(cloudResult.message);
        return snapshot;
    }

    snapshot.available = true;
    snapshot.workpiecePointCloud_mm = cloudResult.points_mm;
    snapshot.message = QString::fromStdString(cloudResult.message);
    return snapshot;
}

} // namespace i2r_rlc::digital_twin
