/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationGlobalIcpPipeline.cpp
 * @brief     全局 ICP 定位流程封装实现。
 * @details   本文件实现单段和分段全局 ICP 编排、变换组合和诊断聚合。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <QDebug>

#include "LocalizationGlobalIcpPipeline.h"
#include "LocalizationPipelineMath.h"

namespace i2r_rlc::localization {

namespace {

struct PointCloudSummary {
    Point3D_mm minimum;
    Point3D_mm maximum;
    Point3D_mm center;
};

PointCloudSummary summarizePointCloud(const std::vector<Point3D_mm>& points)
{
    PointCloudSummary summary;
    if (points.empty()) {
        return summary;
    }

    summary.minimum = points.front();
    summary.maximum = points.front();
    for (const Point3D_mm& point : points) {
        summary.minimum.x_mm = std::min(summary.minimum.x_mm, point.x_mm);
        summary.minimum.y_mm = std::min(summary.minimum.y_mm, point.y_mm);
        summary.minimum.z_mm = std::min(summary.minimum.z_mm, point.z_mm);
        summary.maximum.x_mm = std::max(summary.maximum.x_mm, point.x_mm);
        summary.maximum.y_mm = std::max(summary.maximum.y_mm, point.y_mm);
        summary.maximum.z_mm = std::max(summary.maximum.z_mm, point.z_mm);
        summary.center.x_mm += point.x_mm;
        summary.center.y_mm += point.y_mm;
        summary.center.z_mm += point.z_mm;
    }
    const double count = static_cast<double>(points.size());
    summary.center.x_mm /= count;
    summary.center.y_mm /= count;
    summary.center.z_mm /= count;
    return summary;
}

GlobalIcpPipelineResult failResult(const QString& message)
{
    GlobalIcpPipelineResult result;
    result.status = LocalizationStatus::Failed;
    result.message = message;
    return result;
}

std::vector<Point3D_mm> referencePointsFromRequest(
    const GlobalIcpPipelineRequest& request)
{
    if (!request.virtualReferencePoints_mm.empty()) {
        return request.virtualReferencePoints_mm;
    }
    if (request.virtualReferenceSnapshot.available
        && !request.virtualReferenceSnapshot.workpiecePointCloud_mm.empty()) {
        return request.virtualReferenceSnapshot.workpiecePointCloud_mm;
    }
    if (request.virtualReferenceSnapshot.available
        && !request.virtualReferenceSnapshot.laserLinePoints_mm.empty()) {
        return request.virtualReferenceSnapshot.laserLinePoints_mm;
    }

    return {};
}

std::vector<Point3D_mm> referencePointsFromSegments(
    const std::vector<GlobalIcpSegmentRequest>& segments)
{
    std::vector<Point3D_mm> points;
    for (const GlobalIcpSegmentRequest& segment : segments) {
        points.insert(
            points.end(),
            segment.virtualReferencePoints_mm.begin(),
            segment.virtualReferencePoints_mm.end());
    }
    return points;
}

std::vector<Point3D_mm> realPointsFromSegments(
    const std::vector<GlobalIcpSegmentRequest>& segments)
{
    std::vector<Point3D_mm> points;
    for (const GlobalIcpSegmentRequest& segment : segments) {
        points.insert(
            points.end(),
            segment.realPoints_mm.begin(),
            segment.realPoints_mm.end());
    }
    return points;
}

GlobalIcpSegmentRequest makeDefaultSegment(
    const GlobalIcpPipelineRequest& request,
    const std::vector<Point3D_mm>& referencePoints)
{
    GlobalIcpSegmentRequest segment;
    segment.segmentId = request.requestId.isEmpty()
        ? QStringLiteral("global")
        : request.requestId;
    segment.realPoints_mm = request.realScanPoints_mm;
    segment.virtualReferencePoints_mm = referencePoints;
    segment.dofMask = request.defaultDofMask;
    segment.parameters = request.defaultParameters;
    segment.initialGuess = request.initialGuess;
    return segment;
}

LocalizationRequest makeLocalizationRequest(
    const GlobalIcpPipelineRequest& request,
    const GlobalIcpSegmentRequest& segment,
    const std::vector<Point3D_mm>& referencePoints,
    const RigidTransform& initialGuess)
{
    LocalizationRequest localizationRequest;
    localizationRequest.mode = LocalizationMode::Global;
    localizationRequest.frameId = request.frameId;
    localizationRequest.realPointCount = static_cast<int>(segment.realPoints_mm.size());
    localizationRequest.virtualPointCount = static_cast<int>(referencePoints.size());
    localizationRequest.message = segment.segmentId;
    localizationRequest.globalIcp.realPoints_mm = segment.realPoints_mm;
    localizationRequest.globalIcp.referencePoints_mm = referencePoints;
    localizationRequest.globalIcp.dofMask = segment.dofMask;
    localizationRequest.globalIcp.parameters = segment.parameters;
    localizationRequest.globalIcp.initialGuess = initialGuess;
    return localizationRequest;
}

GlobalIcpSegmentResult makeFailedSegmentResult(
    const QString& segmentId,
    const QString& message)
{
    GlobalIcpSegmentResult result;
    result.segmentId = segmentId;
    result.icpResult.status = LocalizationStatus::Failed;
    result.message = message;
    return result;
}

double aggregateSegmentRmse(
    const std::vector<GlobalIcpSegmentRequest>& segments,
    const RigidTransform& realToVirtualTransform)
{
    double totalSquaredError_mm2 = 0.0;
    std::size_t totalPointCount = 0;
    for (const GlobalIcpSegmentRequest& segment : segments) {
        if (segment.realPoints_mm.empty()
            || segment.virtualReferencePoints_mm.empty()) {
            continue;
        }

        const std::vector<Point3D_mm> transformedRealPoints =
            pipeline_math::transformPoints(
                segment.realPoints_mm,
                realToVirtualTransform);
        const double rmse_mm = pipeline_math::nearestNeighborRmse(
            transformedRealPoints,
            segment.virtualReferencePoints_mm);
        const std::size_t pointCount = transformedRealPoints.size();
        totalSquaredError_mm2 += rmse_mm * rmse_mm
            * static_cast<double>(pointCount);
        totalPointCount += pointCount;
    }

    if (totalPointCount == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(
        totalSquaredError_mm2 / static_cast<double>(totalPointCount));
}

double aggregateSegmentUnweightedRmse(
    const std::vector<GlobalIcpSegmentRequest>& segments,
    const RigidTransform& realToVirtualTransform)
{
    double totalRmse_mm = 0.0;
    std::size_t validSegmentCount = 0;
    for (const GlobalIcpSegmentRequest& segment : segments) {
        if (segment.realPoints_mm.empty()
            || segment.virtualReferencePoints_mm.empty()) {
            continue;
        }

        const std::vector<Point3D_mm> transformedRealPoints =
            pipeline_math::transformPoints(
                segment.realPoints_mm,
                realToVirtualTransform);
        const double rmse_mm = pipeline_math::nearestNeighborRmse(
            transformedRealPoints,
            segment.virtualReferencePoints_mm);
        if (!std::isfinite(rmse_mm)) {
            continue;
        }
        totalRmse_mm += rmse_mm;
        ++validSegmentCount;
    }

    if (validSegmentCount == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return totalRmse_mm / static_cast<double>(validSegmentCount);
}

} // namespace

LocalizationGlobalIcpPipeline::LocalizationGlobalIcpPipeline(
    ILocalizationService& localizationService)
    : m_localizationService(localizationService)
{
}

GlobalIcpPipelineResult LocalizationGlobalIcpPipeline::run(
    const GlobalIcpPipelineRequest& request)
{
    const bool hasExplicitSegments = !request.segments.empty();
    std::vector<GlobalIcpSegmentRequest> segments = request.segments;
    std::vector<Point3D_mm> defaultReferencePoints =
        referencePointsFromRequest(request);
    if (defaultReferencePoints.empty() && !segments.empty()) {
        // 分段运行时每段已经携带自己的定位面点云，不应再强制要求额外的默认参考。
        defaultReferencePoints = referencePointsFromSegments(segments);
    }
    if (defaultReferencePoints.empty()) {
        return failResult(QStringLiteral("全局 ICP 缺少虚拟参考点云。"));
    }

    if (segments.empty()) {
        if (request.realScanPoints_mm.empty()) {
            return failResult(QStringLiteral("全局 ICP 缺少真实扫描点云。"));
        }
        segments.push_back(makeDefaultSegment(request, defaultReferencePoints));
    }

    GlobalIcpPipelineResult result;
    result.status = LocalizationStatus::Succeeded;
    result.combinedVirtualToRealCorrectionTransform = RigidTransform{};
    std::vector<Point3D_mm> allRealPoints = request.realScanPoints_mm;
    if (allRealPoints.empty()) {
        allRealPoints = realPointsFromSegments(segments);
    }
    ///////不空行/////////////////////////
    std::vector<GlobalIcpSegmentRequest> metricSegments = segments;

    // 与 SWv1 stepICPforOffline() 一致：上一段输出的完整 real->virtual
    // 变换直接作为下一段 icp.align() 的初值，而不是组合各段独立增量。
    RigidTransform accumulatedRealToVirtualTransform = request.initialGuess;
    qInfo().noquote()
        << QStringLiteral("[全局定位][ICP] 开始分段配准：segments=%1, real=%2, virtual=%3")
               .arg(static_cast<int>(segments.size()))
               .arg(static_cast<int>(allRealPoints.size()))
               .arg(static_cast<int>(defaultReferencePoints.size()));

    for (std::size_t index = 0; index < segments.size(); ++index) {
        const GlobalIcpSegmentRequest& segment = segments[index];
        const QString segmentId = segment.segmentId.isEmpty()
            ? QStringLiteral("segment_%1").arg(static_cast<int>(index + 1))
            : segment.segmentId;

        if (segment.realPoints_mm.empty()) {
            result.status = LocalizationStatus::Failed;
            result.message = QStringLiteral("全局 ICP 分段 %1 缺少真实点云。").arg(segmentId);
            result.segmentResults.push_back(makeFailedSegmentResult(segmentId, result.message));
            return result;
        }

        const std::vector<Point3D_mm>& segmentReferencePoints =
            (!hasExplicitSegments && segment.virtualReferencePoints_mm.empty())
                ? defaultReferencePoints
                : segment.virtualReferencePoints_mm;
        if (segmentReferencePoints.empty()) {
            result.status = LocalizationStatus::Failed;
            result.message = QStringLiteral("全局 ICP 分段 %1 缺少虚拟参考点云。").arg(segmentId);
            result.segmentResults.push_back(makeFailedSegmentResult(segmentId, result.message));
            return result;
        }

        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP] 开始 %1：real=%2, virtual=%3, dof=[%4,%5,%6,%7,%8,%9]")
                   .arg(segmentId)
                   .arg(static_cast<int>(segment.realPoints_mm.size()))
                   .arg(static_cast<int>(segmentReferencePoints.size()))
                   .arg(segment.dofMask.allowX)
                   .arg(segment.dofMask.allowY)
                   .arg(segment.dofMask.allowZ)
                   .arg(segment.dofMask.allowRx)
                   .arg(segment.dofMask.allowRy)
                   .arg(segment.dofMask.allowRz);
        // 评估与 ICP 使用完全相同的累计初值。第二段这里已经先应用第一段完整
        // real->virtual 结果，禁止的自由度只限制后续增量，不会清除已有修正。
        const std::vector<Point3D_mm> initialRawRealPoints =
            pipeline_math::transformPoints(
                segment.realPoints_mm,
                accumulatedRealToVirtualTransform);
        const PointCloudSummary rawRealSummary =
            summarizePointCloud(segment.realPoints_mm);
        const PointCloudSummary initialRealSummary =
            summarizePointCloud(initialRawRealPoints);
        const PointCloudSummary virtualSummary =
            summarizePointCloud(segmentReferencePoints);
        const double centerDx =
            initialRealSummary.center.x_mm - virtualSummary.center.x_mm;
        const double centerDy =
            initialRealSummary.center.y_mm - virtualSummary.center.y_mm;
        const double centerDz =
            initialRealSummary.center.z_mm - virtualSummary.center.z_mm;
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP] %1 累计初值检查: rawRealCenter=(%2,%3,%4), initialRealCenter=(%5,%6,%7),virtualCenter=(%8,%9,%10),initialCenterDelta=(%11,%12,%13),centerGap=%14 mm")
                   .arg(segmentId)
                   .arg(rawRealSummary.center.x_mm, 0, 'f', 3) // raw X
                   .arg(rawRealSummary.center.y_mm, 0, 'f', 3) // raw Y
                   .arg(rawRealSummary.center.z_mm, 0, 'f', 3) // raw Z
                   .arg(initialRealSummary.center.x_mm, 0, 'f', 3)
                   .arg(initialRealSummary.center.y_mm, 0, 'f', 3)
                   .arg(initialRealSummary.center.z_mm, 0, 'f', 3)
                   .arg(virtualSummary.center.x_mm, 0, 'f', 3)
                   .arg(virtualSummary.center.y_mm, 0, 'f', 3)
                   .arg(virtualSummary.center.z_mm, 0, 'f', 3)
                   .arg(centerDx, 0, 'f', 3)
                   .arg(centerDy, 0, 'f', 3)
                   .arg(centerDz, 0, 'f', 3)
                   .arg(std::sqrt(
                            centerDx * centerDx
                            + centerDy * centerDy
                            + centerDz * centerDz),
                        0,
                        'f',
                        3);
        const LocalizationRequest localizationRequest = makeLocalizationRequest(
            request,
            segment,
            segmentReferencePoints,
            accumulatedRealToVirtualTransform);
        GlobalIcpSegmentResult segmentResult;
        segmentResult.segmentId = segmentId;
        segmentResult.icpResult = m_localizationService.runLocalization(localizationRequest);
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP] %1 预处理：rawReal=%2, filteredReal=%3, virtual=%4, maxCorrespondence=%5")
                   .arg(segmentId)
                   .arg(static_cast<int>(segment.realPoints_mm.size()))
                   .arg(segmentResult.icpResult.diagnostics.filteredSourcePointCount)
                   .arg(static_cast<int>(segmentReferencePoints.size()))
                   .arg(segment.parameters.maxCorrespondenceDistance_mm > 0.0
                            ? QString::number(
                                  segment.parameters.maxCorrespondenceDistance_mm,
                                  'f',
                                  3)
                            : QStringLiteral("PCL default"));

        if (segmentResult.icpResult.status != LocalizationStatus::Succeeded
            || !segmentResult.icpResult.hasPoseCorrection) {
            result.status = LocalizationStatus::Failed;
            segmentResult.message = segmentResult.icpResult.message;
            result.message = QStringLiteral("全局 ICP 分段 %1 失败：%2")
                                 .arg(segmentId, segmentResult.message);
            result.segmentResults.push_back(segmentResult);
            qWarning().noquote() << QStringLiteral("[全局定位][ICP] %1").arg(result.message);
            return result;
        }

        // 质量评估使用 ICP 实际消费的 FPS+SOR 点云，避免被已剔除的扫描离群点
        // 主导；服务未返回预处理点云时才退回原始点云。
        const std::vector<Point3D_mm>& metricRealPoints =
            segmentResult.icpResult.preprocessedRealPoints_mm.empty()
            ? segment.realPoints_mm
            : segmentResult.icpResult.preprocessedRealPoints_mm;
        metricSegments[index].realPoints_mm = metricRealPoints;
        const std::vector<Point3D_mm> initialMetricRealPoints =
            pipeline_math::transformPoints(
                metricRealPoints,
                accumulatedRealToVirtualTransform);
        segmentResult.beforeResidual_mm = pipeline_math::nearestNeighborRmse(
            initialMetricRealPoints,
            segmentReferencePoints);

        accumulatedRealToVirtualTransform =
            segmentResult.icpResult.correctionTransform;
        segmentResult.virtualToRealCorrectionTransform =
            pipeline_math::inverseRigidTransform(accumulatedRealToVirtualTransform);
        segmentResult.correctedVirtualReferencePoints_mm =
            pipeline_math::transformPoints(
                segmentReferencePoints,
                segmentResult.virtualToRealCorrectionTransform);
        segmentResult.hasCorrectedVirtualPoints = true;
        const std::vector<Point3D_mm> finalRealPoints =
            pipeline_math::transformPoints(
                metricRealPoints,
                accumulatedRealToVirtualTransform);
        segmentResult.afterResidual_mm = pipeline_math::nearestNeighborRmse(
            finalRealPoints,
            segmentReferencePoints);
        segmentResult.message = QStringLiteral("全局 ICP 分段完成。");

        result.combinedVirtualToRealCorrectionTransform =
            segmentResult.virtualToRealCorrectionTransform;
        result.segmentResults.push_back(segmentResult);
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP] 完成 %1：RMSE=%2 -> %3 mm,累计 real->virtual 平移=(%4,%5,%6) mm")
                   .arg(segmentId)
                   .arg(segmentResult.beforeResidual_mm, 0, 'f', 6)
                   .arg(segmentResult.afterResidual_mm, 0, 'f', 6)
                   .arg(accumulatedRealToVirtualTransform.rowMajor[3], 0, 'f', 6)
                   .arg(accumulatedRealToVirtualTransform.rowMajor[7], 0, 'f', 6)
                   .arg(accumulatedRealToVirtualTransform.rowMajor[11], 0, 'f', 6);
        if (segmentResult.icpResult.diagnostics.refinementAttempted) {
            const LocalizationDiagnostics& diagnostics =
                segmentResult.icpResult.diagnostics;
            const QString fineRmseText = diagnostics.refinementConverged
                ? QString::number(
                    std::sqrt(std::max(
                        0.0,
                        diagnostics.refinementFitnessScore_mm2)),
                    'f',
                    6) + QStringLiteral(" mm")
                : QStringLiteral("未收敛");
            qInfo().noquote()
                << QStringLiteral("[全局定位][ICP] %1 精配准: threshold=%2 mm, coarseRMSE=%3，fineRMSE=%4 mm, accepted=%5")
                .arg(segmentId)
                .arg(
                    diagnostics.refinementMaxCorrespondenceDistance_mm,
                    0,
                    'f',
                    3)
                .arg(
                    std::sqrt(std::max(
                        0.0,
                        diagnostics.coarseFitnessScore_mm2)),
                    0,
                    'f',
                    6)
                .arg(fineRmseText)
                .arg(diagnostics.refinementAccepted);
        }
    }

    result.correctedVirtualReferencePoints_mm = pipeline_math::transformPoints(
        defaultReferencePoints,
        result.combinedVirtualToRealCorrectionTransform);
    result.hasCorrectedVirtualPoints = true;
    result.beforeResidual_mm = aggregateSegmentRmse(
        metricSegments,
        request.initialGuess);
    result.afterResidual_mm = aggregateSegmentRmse(
        metricSegments,
        accumulatedRealToVirtualTransform);
    const bool hasSingleMetricSegment = metricSegments.size() == 1;
    if (hasSingleMetricSegment) {
        // 单段时加权和不加权公式都等于该段自身 RMSE，避免重复执行
        // 一次开销较大的最近邻遍历，同时保留结果结构的兼容字段。
        result.beforeUnweightedResidual_mm = result.beforeResidual_mm;
        result.afterUnweightedResidual_mm = result.afterResidual_mm;
    } else {
        result.beforeUnweightedResidual_mm = aggregateSegmentUnweightedRmse(
            metricSegments,
            request.initialGuess);
        result.afterUnweightedResidual_mm = aggregateSegmentUnweightedRmse(
            metricSegments,
            accumulatedRealToVirtualTransform);
    }
    for (std::size_t index = 0; index < metricSegments.size(); ++index) {
        const GlobalIcpSegmentRequest& segment = metricSegments[index];
        const std::vector<Point3D_mm> finalRealPoints =
            pipeline_math::transformPoints(
                segment.realPoints_mm,
                accumulatedRealToVirtualTransform);
        const double finalRmse_mm = pipeline_math::nearestNeighborRmse(
            finalRealPoints,
            segment.virtualReferencePoints_mm);
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP] 最终统一变换复核 %1: RMSE=%2 mm")
            .arg(segment.segmentId.isEmpty()
                ? QStringLiteral("segment_%1")
                .arg(static_cast<int>(index + 1))
                : segment.segmentId)
            .arg(finalRmse_mm, 0, 'f', 6);
    }
    result.message = QStringLiteral("全局 ICP 流程完成。");
    if (hasSingleMetricSegment) {
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP汇总][RMSE] %1 -> %2 mm")
                   .arg(result.beforeResidual_mm, 0, 'f', 6)
                   .arg(result.afterResidual_mm, 0, 'f', 6);
    } else {
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP汇总][加权RMSE] %1 -> %2 mm")
                   .arg(result.beforeResidual_mm, 0, 'f', 6)
                   .arg(result.afterResidual_mm, 0, 'f', 6);
        qInfo().noquote()
            << QStringLiteral("[全局定位][ICP汇总][不加权RMSE] %1 -> %2 mm")
                   .arg(result.beforeUnweightedResidual_mm, 0, 'f', 6)
                   .arg(result.afterUnweightedResidual_mm, 0, 'f', 6);
    }
    return result;
}

} // namespace i2r_rlc::localization
