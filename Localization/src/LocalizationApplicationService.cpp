/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationApplicationService.cpp
 * @brief     定位 UI 应用层门面实现。
 * @details   本文件实现局部修正、全局定位和定位流程状态聚合。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 16 日
 */

#include <algorithm>

#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include "ILocalizationVirtualReferenceProvider.h"
#include "LocalizationApplicationService.h"

namespace i2r_rlc::localization {

namespace {

struct ReplayCase {
    QString directoryPath;
    QString realBeforePath;
    QString virtualBeforePath;
};

std::vector<Point3D_mm> readPointCloudFile(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }

    QTextStream stream(&file);
    std::vector<Point3D_mm> points;
    while (!stream.atEnd()) {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double z_mm = 0.0;
        stream >> x_mm >> y_mm >> z_mm;
        if (stream.status() != QTextStream::Ok) {
            break;
        }
        points.push_back({x_mm, y_mm, z_mm});
    }
    return points;
}

std::vector<ReplayCase> findReplayCases(const QString& rootPath)
{
    if (rootPath.isEmpty() || !QFileInfo::exists(rootPath)) {
        return {};
    }

    std::vector<ReplayCase> cases;
    QDirIterator iterator(rootPath, QDir::Files, QDirIterator::Subdirectories);
    while (iterator.hasNext()) {
        const QString path = iterator.next();
        const QFileInfo fileInfo(path);
        if (fileInfo.fileName() != QStringLiteral("realPointCloudFile_firstForICP.txt")) {
            continue;
        }

        const QDir directory = fileInfo.dir();
        ReplayCase replayCase;
        replayCase.directoryPath = directory.absolutePath();
        replayCase.realBeforePath = path;
        replayCase.virtualBeforePath =
            directory.filePath(QStringLiteral("virtualPointCloudFile_firstForICP.txt"));

        if (QFileInfo::exists(replayCase.virtualBeforePath)) {
            cases.push_back(replayCase);
        }
    }

    std::sort(cases.begin(), cases.end(), [](const ReplayCase& left, const ReplayCase& right) {
        return left.directoryPath < right.directoryPath;
    });
    return cases;
}

std::vector<Point3D_mm> makeDryRunRealCloud()
{
    return {
        {0.0, 0.0, 0.0},
        {20.0, 3.0, 1.0},
        {41.0, 6.0, 3.0},
        {5.0, 18.0, 8.0},
        {28.0, 24.0, 4.0},
        {50.0, 32.0, 7.0},
        {9.0, 43.0, 2.0},
        {30.0, 47.0, 9.0},
        {55.0, 53.0, 5.0},
        {75.0, 11.0, 11.0},
        {82.0, 38.0, 6.0},
        {96.0, 59.0, 13.0}
    };
}

} // namespace

LocalizationApplicationService::LocalizationApplicationService()
    : m_localCorrectionPipeline(m_localizationService)
    , m_globalIcpPipeline(m_localizationService)
{
}

LocalizationStartResult LocalizationApplicationService::start()
{
    return m_localizationService.start();
}

LocalizationApplicationResult LocalizationApplicationService::runLocalCorrectionReplay(
    const LocalizationLocalReplayRequest& request)
{
    auto cases = findReplayCases(request.pointCloudRootPath);
    if (cases.empty()) {
        return failApplicationResult(
            LocalizationMode::Local,
            QStringLiteral("未找到局部修正点云：%1").arg(request.pointCloudRootPath));
    }

    const ReplayCase& replayCase = cases.front();
    const std::vector<Point3D_mm> realBefore = readPointCloudFile(replayCase.realBeforePath);
    const std::vector<Point3D_mm> virtualBefore = readPointCloudFile(replayCase.virtualBeforePath);
    if (realBefore.empty() || virtualBefore.empty()) {
        return failApplicationResult(
            LocalizationMode::Local,
            QStringLiteral("局部修正点云为空：%1").arg(replayCase.directoryPath));
    }

    LocalCorrectionPipelineRequest pipelineRequest;
    pipelineRequest.frameId = 0;
    pipelineRequest.realWindowPoints_mm = realBefore;
    pipelineRequest.virtualWindowPoints_mm = virtualBefore;
    pipelineRequest.windowFrameCount = request.minimumWindowFrameCount;
    pipelineRequest.minimumWindowFrameCount = request.minimumWindowFrameCount;
    pipelineRequest.dofMask = {true, true, true, false, false, true};
    pipelineRequest.parameters.enableSorFilter = false;
    pipelineRequest.parameters.preprocessingProfile =
        IcpPreprocessingProfile::CompatibilityFirstLocalIcp;
    pipelineRequest.parameters.maxCorrespondenceDistance_mm = 0.0;
    pipelineRequest.parameters.maximumIterations = 0;

    m_workflowController.startLocalCorrection(
        pipelineRequest.frameId,
        pipelineRequest.dofMask);

    LocalizationApplicationResult result;
    result.localCorrectionResult = m_localCorrectionPipeline.run(pipelineRequest);
    result.hasLocalCorrectionResult = true;
    result.replayDirectoryPath = replayCase.directoryPath;
    result.update = m_workflowController.finishLocalCorrection(result.localCorrectionResult);
    return result;
}

LocalizationApplicationResult LocalizationApplicationService::runGlobalIcpDryRun(
    const LocalizationGlobalDryRunRequest& request)
{
    m_workflowController.startGlobalIcp(request.dofMask, 1);

    NullLocalizationVirtualReferenceProvider provider;
    LocalizationVirtualReferenceRequest referenceRequest;
    referenceRequest.frameId = request.frameId;
    referenceRequest.referenceId = QStringLiteral("ui-global-icp-dry-run");

    GlobalIcpPipelineRequest pipelineRequest;
    pipelineRequest.frameId = request.frameId;
    pipelineRequest.realScanPoints_mm = makeDryRunRealCloud();
    pipelineRequest.virtualReferenceSnapshot = provider.requestSnapshot(referenceRequest);
    pipelineRequest.defaultDofMask = request.dofMask;
    pipelineRequest.defaultParameters.enableSorFilter = false;

    LocalizationApplicationResult result;
    result.globalIcpResult = m_globalIcpPipeline.run(pipelineRequest);
    result.hasGlobalIcpResult = true;

    if (result.globalIcpResult.status == LocalizationStatus::Failed
        && result.globalIcpResult.message.contains(QStringLiteral("虚拟参考点云"))) {
        result.update = m_workflowController.markMissingVirtualReference(
            QStringLiteral("全局定位需要 DigitalTwin/RobotSim 提供虚拟参考数据；当前未获得可用数据。"));
        result.update.segmentCount = 1;
        result.update.dofMask = request.dofMask;
        return result;
    }

    result.update = m_workflowController.finishGlobalIcp(result.globalIcpResult);
    return result;
}

LocalizationApplicationResult LocalizationApplicationService::runGlobalIcpDryRun(
    const LocalizationGlobalDryRunRequest& request,
    const LocalizationVirtualReferenceSnapshot& virtualReferenceSnapshot)
{
    m_workflowController.startGlobalIcp(request.dofMask, 1);

    GlobalIcpPipelineRequest pipelineRequest;
    pipelineRequest.frameId = request.frameId;
    pipelineRequest.realScanPoints_mm = makeDryRunRealCloud();
    pipelineRequest.virtualReferenceSnapshot = virtualReferenceSnapshot;
    pipelineRequest.defaultDofMask = request.dofMask;
    pipelineRequest.defaultParameters.enableSorFilter = false;

    LocalizationApplicationResult result;
    result.globalIcpResult = m_globalIcpPipeline.run(pipelineRequest);
    result.hasGlobalIcpResult = true;

    if (result.globalIcpResult.status == LocalizationStatus::Failed
        && result.globalIcpResult.message.contains(QStringLiteral("虚拟参考点云"))) {
        result.update = m_workflowController.markMissingVirtualReference(
            QStringLiteral("全局定位未获得可用 RobotSim 虚拟参考数据。"));
        result.update.segmentCount = 1;
        result.update.dofMask = request.dofMask;
        return result;
    }

    result.update = m_workflowController.finishGlobalIcp(result.globalIcpResult);
    return result;
}

LocalizationApplicationResult LocalizationApplicationService::runGlobalIcpRuntime(
    const LocalizationGlobalRuntimeRequest& request)
{
    m_workflowController.startGlobalIcp(request.dofMask, 1);

    GlobalIcpPipelineRequest pipelineRequest;
    pipelineRequest.frameId = request.frameId;
    pipelineRequest.requestId = QStringLiteral("online-auto-global-localization");
    pipelineRequest.realScanPoints_mm = request.realScanPoints_mm;
    pipelineRequest.virtualReferenceSnapshot = request.virtualReferenceSnapshot;
    pipelineRequest.defaultDofMask = request.dofMask;
    pipelineRequest.defaultParameters.enableSorFilter = false;

    LocalizationApplicationResult result;
    result.globalIcpResult = m_globalIcpPipeline.run(pipelineRequest);
    result.hasGlobalIcpResult = true;

    if (result.globalIcpResult.status == LocalizationStatus::Failed
        && result.globalIcpResult.message.contains(QStringLiteral("真实扫描点云"))) {
        result.update = m_workflowController.finishGlobalIcp(result.globalIcpResult);
        return result;
    }
    if (result.globalIcpResult.status == LocalizationStatus::Failed
        && result.globalIcpResult.message.contains(QStringLiteral("虚拟参考点云"))) {
        result.update = m_workflowController.markMissingVirtualReference(
            QStringLiteral("全局定位需要真实扫描点和可用虚拟参考；当前未获得可用虚拟参考点云。"));
        result.update.segmentCount = 1;
        result.update.dofMask = request.dofMask;
        return result;
    }

    result.update = m_workflowController.finishGlobalIcp(result.globalIcpResult);
    return result;
}

LocalizationApplicationResult LocalizationApplicationService::runGlobalIcpRuntimeSegmented(
    const LocalizationGlobalRuntimeSegmentedRequest& request)
{
    const int segmentCount =
        request.segments.empty() ? 1 : static_cast<int>(request.segments.size());
    m_workflowController.startGlobalIcp(request.defaultDofMask, segmentCount);

    GlobalIcpPipelineRequest pipelineRequest;
    pipelineRequest.frameId = request.frameId;
    pipelineRequest.requestId = QStringLiteral("standalone-global-localization");
    pipelineRequest.virtualReferenceSnapshot = request.virtualReferenceSnapshot;
    pipelineRequest.defaultDofMask = request.defaultDofMask;
    pipelineRequest.defaultParameters = request.defaultParameters;
    pipelineRequest.initialGuess = request.initialGuess;
    pipelineRequest.segments = request.segments;

    LocalizationApplicationResult result;
    result.globalIcpResult = m_globalIcpPipeline.run(pipelineRequest);
    result.hasGlobalIcpResult = true;

    if (result.globalIcpResult.status == LocalizationStatus::Failed
        && result.globalIcpResult.message.contains(QStringLiteral("真实扫描点云"))) {
        result.update = m_workflowController.finishGlobalIcp(result.globalIcpResult);
        return result;
    }
    if (result.globalIcpResult.status == LocalizationStatus::Failed
        && result.globalIcpResult.message.contains(QStringLiteral("虚拟参考点云"))) {
        result.update = m_workflowController.markMissingVirtualReference(
            QStringLiteral("全局定位需要真实扫描点和可用虚拟参考；当前未获得可用虚拟参考点云。"));
        result.update.segmentCount = segmentCount;
        result.update.dofMask = request.defaultDofMask;
        return result;
    }

    result.update = m_workflowController.finishGlobalIcp(result.globalIcpResult);
    return result;
}

LocalizationWorkflowUpdate LocalizationApplicationService::clear()
{
    return m_workflowController.clear();
}

LocalizationWorkflowState LocalizationApplicationService::workflowState() const
{
    return m_workflowController.state();
}

LocalizationApplicationResult LocalizationApplicationService::failApplicationResult(
    const LocalizationMode mode,
    const QString& message)
{
    LocalizationApplicationResult result;
    result.update.state = LocalizationWorkflowState::Failed;
    result.update.mode = mode;
    result.update.succeeded = false;
    result.update.message = QStringLiteral("定位流程失败。");
    result.update.errorMessage = message;
    return result;
}

} // namespace i2r_rlc::localization
