/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationLocalCorrectionPipeline.cpp
 * @brief     局部定位流程封装实现。
 * @details   本文件实现局部窗口点云修正、残差统计和诊断聚合。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <algorithm>

#include "LocalizationLocalCorrectionPipeline.h"
#include "LocalizationPipelineMath.h"

namespace i2r_rlc::localization {

namespace {

LocalCorrectionPipelineResult failResult(const QString& message)
{
    LocalCorrectionPipelineResult result;
    result.status = LocalizationStatus::Failed;
    result.message = message;
    return result;
}

} // namespace

LocalizationLocalCorrectionPipeline::LocalizationLocalCorrectionPipeline(
    ILocalizationService& localizationService)
    : m_localizationService(localizationService)
{
}

LocalCorrectionPipelineResult LocalizationLocalCorrectionPipeline::run(
    const LocalCorrectionPipelineRequest& request)
{
    const int minFrameCount = std::max(1, request.minimumWindowFrameCount);
    if (request.windowFrameCount > 0 && request.windowFrameCount < minFrameCount) {
        return failResult(
            QStringLiteral("局部定位窗口帧数不足：当前 %1，最少需要 %2。")
                .arg(request.windowFrameCount)
                .arg(minFrameCount));
    }

    if (request.realWindowPoints_mm.empty() || request.virtualWindowPoints_mm.empty()) {
        return failResult(QStringLiteral("局部定位点云为空。"));
    }

    LocalizationRequest localizationRequest;
    localizationRequest.mode = LocalizationMode::Local;
    localizationRequest.frameId = request.frameId;
    localizationRequest.realPointCount = static_cast<int>(request.realWindowPoints_mm.size());
    localizationRequest.virtualPointCount = static_cast<int>(request.virtualWindowPoints_mm.size());
    localizationRequest.localCorrection.realWindowPoints_mm = request.realWindowPoints_mm;
    localizationRequest.localCorrection.referenceWindowPoints_mm = request.virtualWindowPoints_mm;
    localizationRequest.localCorrection.dofMask = request.dofMask;
    localizationRequest.localCorrection.parameters = request.parameters;
    localizationRequest.localCorrection.currentWorkpiecePose = request.currentWorkpiecePose;
    localizationRequest.localCorrection.initialGuess = request.initialGuess;
    localizationRequest.localCorrection.windowFrameCount = request.windowFrameCount;

    LocalCorrectionPipelineResult result;
    result.beforeResidual_mm = pipeline_math::meanNearestDistance(
        request.realWindowPoints_mm,
        request.virtualWindowPoints_mm);
    result.icpResult = m_localizationService.runLocalization(localizationRequest);
    result.status = result.icpResult.status;

    if (result.icpResult.status != LocalizationStatus::Succeeded
        || !result.icpResult.hasPoseCorrection) {
        result.message = result.icpResult.message;
        return result;
    }

    result.virtualToRealCorrectionTransform =
        pipeline_math::inverseRigidTransform(result.icpResult.correctionTransform);
    result.correctedVirtualWindowPoints_mm = pipeline_math::transformPoints(
        request.virtualWindowPoints_mm,
        result.virtualToRealCorrectionTransform);
    result.hasCorrectedVirtualPoints = true;
    result.afterResidual_mm = pipeline_math::meanNearestDistance(
        request.realWindowPoints_mm,
        result.correctedVirtualWindowPoints_mm);
    result.message = QStringLiteral("局部定位离线流程完成。");
    return result;
}

} // namespace i2r_rlc::localization
