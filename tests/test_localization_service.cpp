/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_localization_service.cpp
 * @brief       localization service 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "ILocalizationVirtualReferenceProvider.h"
#include "LocalizationApplicationService.h"
#include "LocalizationGlobalIcpPipeline.h"
#include "LocalizationLocalCorrectionPipeline.h"
#include "LocalizationPointProjector.h"
#include "LocalizationService.h"
#include "NullLocalizationService.h"

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool expectNear(const double actual, const double expected, const double tolerance, const char* message)
{
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << ", actual=" << actual << ", expected=" << expected << '\n';
        return false;
    }

    return true;
}

class RecordingLocalizationService final
    : public i2r_rlc::localization::ILocalizationService {
public:
    i2r_rlc::localization::LocalizationStartResult start() override
    {
        return {};
    }

    i2r_rlc::localization::LocalizationResult runLocalization(
        const i2r_rlc::localization::LocalizationRequest& request) override
    {
        requests.push_back(request);

        i2r_rlc::localization::LocalizationResult result;
        result.status = i2r_rlc::localization::LocalizationStatus::Succeeded;
        result.hasPoseCorrection = true;
        result.correctionTransform.rowMajor[3] = requests.size() == 1U ? 3.0 : 7.0;
        result.message = QStringLiteral("recorded global ICP request");
        return result;
    }

    std::vector<i2r_rlc::localization::LocalizationRequest> requests;
};

std::filesystem::path projectSourceDir()
{
#ifdef I2R_RLC_PROJECT_SOURCE_DIR
    return std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR);
#else
    return std::filesystem::current_path();
#endif
}

i2r_rlc::localization::LocalizationRequest makeRequest(
    const i2r_rlc::localization::LocalizationMode mode,
    const qint64 frameId,
    const int realPointCount,
    const int virtualPointCount)
{
    i2r_rlc::localization::LocalizationRequest request;
    request.mode = mode;
    request.frameId = frameId;
    request.realPointCount = realPointCount;
    request.virtualPointCount = virtualPointCount;
    request.message = QStringLiteral("test request");
    return request;
}

std::vector<i2r_rlc::localization::Point3D_mm> makeReferenceCloud()
{
    using i2r_rlc::localization::Point3D_mm;

    return {
        {0.0, 0.0, 0.0},
        {20.0, 2.0, 1.0},
        {42.0, 5.0, 3.0},
        {4.0, 19.0, 8.0},
        {27.0, 25.0, 4.0},
        {49.0, 31.0, 7.0},
        {8.0, 43.0, 2.0},
        {31.0, 46.0, 9.0},
        {55.0, 52.0, 5.0},
        {75.0, 10.0, 11.0},
        {82.0, 37.0, 6.0},
        {96.0, 58.0, 13.0}
    };
}

std::vector<i2r_rlc::localization::Point3D_mm> makeApplicationDryRunCloud()
{
    using i2r_rlc::localization::Point3D_mm;

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

std::vector<i2r_rlc::localization::Point3D_mm> translatedCloud(
    const std::vector<i2r_rlc::localization::Point3D_mm>& points,
    const double dx_mm,
    const double dy_mm,
    const double dz_mm)
{
    auto translated = points;
    for (auto& point : translated) {
        point.x_mm += dx_mm;
        point.y_mm += dy_mm;
        point.z_mm += dz_mm;
    }

    return translated;
}

std::vector<i2r_rlc::localization::Point3D_mm> transformedCloudXyzRz(
    const std::vector<i2r_rlc::localization::Point3D_mm>& points,
    const double dx_mm,
    const double dy_mm,
    const double dz_mm,
    const double rz_deg)
{
    constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;
    const double angle_rad = rz_deg * degreesToRadians;
    const double cosAngle = std::cos(angle_rad);
    const double sinAngle = std::sin(angle_rad);

    std::vector<i2r_rlc::localization::Point3D_mm> transformed;
    transformed.reserve(points.size());
    for (const auto& point : points) {
        transformed.push_back({
            cosAngle * point.x_mm - sinAngle * point.y_mm + dx_mm,
            sinAngle * point.x_mm + cosAngle * point.y_mm + dy_mm,
            point.z_mm + dz_mm
        });
    }

    return transformed;
}

std::vector<i2r_rlc::localization::Point3D_mm> readPointCloudFile(
    const std::filesystem::path& path)
{
    using i2r_rlc::localization::Point3D_mm;

    std::ifstream stream(path);
    std::vector<Point3D_mm> points;

    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    while (stream >> x_mm >> y_mm >> z_mm) {
        points.push_back({x_mm, y_mm, z_mm});
    }

    return points;
}

double squaredDistance(
    const i2r_rlc::localization::Point3D_mm& first,
    const i2r_rlc::localization::Point3D_mm& second)
{
    const double dx_mm = first.x_mm - second.x_mm;
    const double dy_mm = first.y_mm - second.y_mm;
    const double dz_mm = first.z_mm - second.z_mm;
    return dx_mm * dx_mm + dy_mm * dy_mm + dz_mm * dz_mm;
}

double meanNearestDistance(
    const std::vector<i2r_rlc::localization::Point3D_mm>& source,
    const std::vector<i2r_rlc::localization::Point3D_mm>& target)
{
    if (source.empty() || target.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double totalDistance_mm = 0.0;
    for (const auto& sourcePoint : source) {
        double bestDistanceSquared_mm2 = std::numeric_limits<double>::max();
        for (const auto& targetPoint : target) {
            bestDistanceSquared_mm2 = std::min(
                bestDistanceSquared_mm2,
                squaredDistance(sourcePoint, targetPoint));
        }
        totalDistance_mm += std::sqrt(bestDistanceSquared_mm2);
    }

    return totalDistance_mm / static_cast<double>(source.size());
}

double meanPointwiseDistance(
    const std::vector<i2r_rlc::localization::Point3D_mm>& first,
    const std::vector<i2r_rlc::localization::Point3D_mm>& second)
{
    const auto pointCount = std::min(first.size(), second.size());
    if (pointCount == 0) {
        return std::numeric_limits<double>::infinity();
    }

    double totalDistance_mm = 0.0;
    for (std::size_t index = 0; index < pointCount; ++index) {
        totalDistance_mm += std::sqrt(squaredDistance(first[index], second[index]));
    }

    return totalDistance_mm / static_cast<double>(pointCount);
}

struct GlobalIcpReplayCase {
    std::filesystem::path directory;
    std::filesystem::path realBeforePath;
    std::filesystem::path virtualBeforePath;
    std::filesystem::path virtualAfterPath;
};

std::vector<GlobalIcpReplayCase> findGlobalIcpReplayCases()
{
    const std::filesystem::path root =
        projectSourceDir() / "data" / "IBVS" / "pointcloud";
    std::error_code error;
    if (!std::filesystem::exists(root, error)) {
        return {};
    }

    std::vector<GlobalIcpReplayCase> cases;
    for (std::filesystem::recursive_directory_iterator it(root, error), end;
         !error && it != end;
         it.increment(error)) {
        if (!it->is_regular_file(error)
            || it->path().filename() != "realPointCloudFile_firstForICP.txt") {
            continue;
        }

        const std::filesystem::path directory = it->path().parent_path();
        GlobalIcpReplayCase replayCase;
        replayCase.directory = directory;
        replayCase.realBeforePath = it->path();
        replayCase.virtualBeforePath = directory / "virtualPointCloudFile_firstForICP.txt";
        replayCase.virtualAfterPath = directory / "virtualPointCloudFile_firstAfterICP.txt";

        if (std::filesystem::exists(replayCase.virtualBeforePath, error)
            && std::filesystem::exists(replayCase.virtualAfterPath, error)) {
            cases.push_back(replayCase);
        }
    }

    return cases;
}

bool testStartReportsUnavailable()
{
    i2r_rlc::localization::NullLocalizationService service;

    const auto result = service.start();

    bool ok = true;
    ok &= expectTrue(result.state == i2r_rlc::localization::LocalizationServiceState::Unavailable,
                     "start should return unavailable service state");
    ok &= expectTrue(result.message.contains(QStringLiteral("LocalizationService"))
                         && result.message.contains(QStringLiteral("不可用")),
                     "start should return unavailable log message");
    return ok;
}

bool testRealServiceStartReportsIdle()
{
    i2r_rlc::localization::LocalizationService service;

    const auto result = service.start();

    bool ok = true;
    ok &= expectTrue(result.state == i2r_rlc::localization::LocalizationServiceState::Idle,
                     "real service start should return idle state");
    ok &= expectTrue(result.message.contains(QStringLiteral("ready")),
                     "real service start should describe ready state");
    return ok;
}

bool testNoOpResult(const i2r_rlc::localization::LocalizationMode mode)
{
    i2r_rlc::localization::NullLocalizationService service;
    const auto request = makeRequest(mode, mode == i2r_rlc::localization::LocalizationMode::Global ? 21 : 34, 120, 95);

    const auto captured = service.runLocalization(request);

    bool ok = true;
    ok &= expectTrue(captured.status == i2r_rlc::localization::LocalizationStatus::AlgorithmNotImplemented,
                     "no-op result should report AlgorithmNotImplemented");
    ok &= expectTrue(captured.mode == request.mode, "result should preserve request mode");
    ok &= expectTrue(captured.frameId == request.frameId, "result should preserve frameId");
    ok &= expectTrue(captured.realPointCount == request.realPointCount, "result should preserve real point count");
    ok &= expectTrue(captured.virtualPointCount == request.virtualPointCount, "result should preserve virtual point count");
    ok &= expectTrue(!captured.hasPoseCorrection, "no-op result should not contain pose correction");
    ok &= expectTrue(!captured.message.isEmpty(), "no-op result should carry a message");
    return ok;
}

bool testGlobalIcpRecoversTranslation()
{
    using namespace i2r_rlc::localization;

    LocalizationService service;
    LocalizationRequest request;
    request.mode = LocalizationMode::Global;
    request.frameId = 1001;
    request.globalIcp.referencePoints_mm = makeReferenceCloud();
    request.globalIcp.realPoints_mm = translatedCloud(request.globalIcp.referencePoints_mm, 5.0, -3.0, 2.0);
    request.globalIcp.parameters.enableSorFilter = false;
    request.globalIcp.parameters.maxCorrespondenceDistance_mm = 30.0;
    request.globalIcp.parameters.maximumIterations = 80;

    const auto result = service.runLocalization(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Succeeded, "global ICP should succeed");
    ok &= expectTrue(result.hasPoseCorrection, "global ICP should contain pose correction");
    ok &= expectNear(result.correctionX_mm, -5.0, 0.7, "global ICP x correction");
    ok &= expectNear(result.correctionY_mm, 3.0, 0.7, "global ICP y correction");
    ok &= expectNear(result.correctionZ_mm, -2.0, 0.7, "global ICP z correction");
    ok &= expectTrue(result.diagnostics.converged, "global ICP diagnostics should report convergence");
    return ok;
}

bool testLocalIcpUsesWindowPoints()
{
    using namespace i2r_rlc::localization;

    LocalizationService service;
    LocalizationRequest request;
    request.mode = LocalizationMode::Local;
    request.frameId = 2002;
    request.localCorrection.referenceWindowPoints_mm = makeReferenceCloud();
    request.localCorrection.realWindowPoints_mm =
        translatedCloud(request.localCorrection.referenceWindowPoints_mm, -4.0, 6.0, 0.0);
    request.localCorrection.windowFrameCount = 20;
    request.localCorrection.parameters.enableSorFilter = false;
    request.localCorrection.parameters.maxCorrespondenceDistance_mm = 30.0;
    request.localCorrection.parameters.maximumIterations = 80;

    const auto result = service.runLocalization(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Succeeded, "local ICP should succeed");
    ok &= expectNear(result.correctionX_mm, 4.0, 0.8, "local ICP x correction");
    ok &= expectNear(result.correctionY_mm, -6.0, 0.8, "local ICP y correction");
    ok &= expectNear(result.correctionZ_mm, 0.0, 0.8, "local ICP z correction");
    return ok;
}

bool testLocalCorrectionPipelineRejectsShortWindow()
{
    using namespace i2r_rlc::localization;

    LocalizationService service;
    LocalizationLocalCorrectionPipeline pipeline(service);

    LocalCorrectionPipelineRequest request;
    request.realWindowPoints_mm = makeReferenceCloud();
    request.virtualWindowPoints_mm = makeReferenceCloud();
    request.windowFrameCount = 3;
    request.minimumWindowFrameCount = 20;

    const auto result = pipeline.run(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Failed, "local pipeline should reject short windows");
    ok &= expectTrue(!result.hasCorrectedVirtualPoints, "short window should not produce corrected points");
    ok &= expectTrue(result.message.contains(QStringLiteral("窗口帧数不足")),
                     "short window failure should explain frame count");
    return ok;
}

bool testGlobalIcpPipelineSingleSegment()
{
    using namespace i2r_rlc::localization;

    const auto reference = makeReferenceCloud();
    const auto real = transformedCloudXyzRz(reference, 5.0, -3.0, 2.0, 4.0);

    LocalizationService service;
    LocalizationGlobalIcpPipeline pipeline(service);
    GlobalIcpPipelineRequest request;
    request.frameId = 3003;
    request.realScanPoints_mm = real;
    request.virtualReferencePoints_mm = reference;
    request.defaultParameters.enableSorFilter = false;
    request.defaultParameters.maxCorrespondenceDistance_mm = 80.0;
    request.defaultParameters.maximumIterations = 120;

    const auto result = pipeline.run(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Succeeded, "global pipeline should succeed");
    ok &= expectTrue(result.hasCorrectedVirtualPoints, "global pipeline should correct virtual points");
    ok &= expectTrue(result.segmentResults.size() == 1, "global pipeline should create one segment result");
    ok &= expectTrue(result.afterResidual_mm < result.beforeResidual_mm * 0.5,
                     "global pipeline should reduce residual");
    if (!result.segmentResults.empty()) {
        const auto& segment = result.segmentResults.front();
        ok &= expectNear(segment.icpResult.correctionRx_deg, 0.0, 1e-5, "global pipeline should lock rx");
        ok &= expectNear(segment.icpResult.correctionRy_deg, 0.0, 1e-5, "global pipeline should lock ry");
    }
    return ok;
}

bool testGlobalIcpPipelineSegmentDofDiagnostics()
{
    using namespace i2r_rlc::localization;

    const auto reference = makeReferenceCloud();
    const std::vector<Point3D_mm> firstReference(reference.begin(), reference.begin() + 6);
    const std::vector<Point3D_mm> secondReference(reference.begin() + 6, reference.end());

    GlobalIcpSegmentRequest firstSegment;
    firstSegment.segmentId = QStringLiteral("x_only");
    firstSegment.realPoints_mm = translatedCloud(firstReference, 3.0, 0.0, 0.0);
    firstSegment.virtualReferencePoints_mm = firstReference;
    firstSegment.dofMask = {true, false, false, false, false, false};
    firstSegment.parameters.enableSorFilter = false;
    firstSegment.parameters.maxCorrespondenceDistance_mm = 30.0;
    firstSegment.parameters.minPointCount = 3;

    GlobalIcpSegmentRequest secondSegment;
    secondSegment.segmentId = QStringLiteral("y_only");
    // 两个定位面属于同一刚体：第二面继承第一面的 X 偏移，并额外提供 Y 约束。
    secondSegment.realPoints_mm = translatedCloud(secondReference, 3.0, -4.0, 0.0);
    secondSegment.virtualReferencePoints_mm = secondReference;
    secondSegment.dofMask = {false, true, false, false, false, false};
    secondSegment.parameters.enableSorFilter = false;
    secondSegment.parameters.maxCorrespondenceDistance_mm = 30.0;
    secondSegment.parameters.minPointCount = 3;

    LocalizationService service;
    LocalizationGlobalIcpPipeline pipeline(service);
    GlobalIcpPipelineRequest request;
    request.virtualReferencePoints_mm = reference;
    request.segments = {firstSegment, secondSegment};

    const auto result = pipeline.run(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Succeeded, "segmented global pipeline should succeed");
    ok &= expectTrue(result.segmentResults.size() == 2, "segmented global pipeline should report two segments");
    if (result.segmentResults.size() == 2) {
        ok &= expectTrue(result.segmentResults[0].segmentId == QStringLiteral("x_only"),
                         "first segment id should be preserved");
        ok &= expectTrue(result.segmentResults[1].segmentId == QStringLiteral("y_only"),
                         "second segment id should be preserved");
        ok &= expectNear(result.segmentResults[0].icpResult.correctionY_mm, 0.0, 1e-5,
                         "x-only segment should lock y");
        ok &= expectNear(result.segmentResults[1].icpResult.correctionX_mm, -3.0, 1e-4,
            "y-only segment should preserve x correction inherited from segment");
    }
    return ok;
}

bool testGlobalIcpPipelineConsumesVirtualSnapshot()
{
    using namespace i2r_rlc::localization;

    const auto reference = makeReferenceCloud();
    LocalizationVirtualReferenceSnapshot snapshot;
    snapshot.available = true;
    snapshot.referenceId = QStringLiteral("fixture");
    snapshot.workpiecePointCloud_mm = reference;

    LocalizationService service;
    LocalizationGlobalIcpPipeline pipeline(service);
    GlobalIcpPipelineRequest request;
    request.realScanPoints_mm = translatedCloud(reference, 2.0, 0.0, 0.0);
    request.virtualReferenceSnapshot = snapshot;
    request.defaultParameters.enableSorFilter = false;
    request.defaultParameters.maxCorrespondenceDistance_mm = 30.0;

    const auto result = pipeline.run(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Succeeded,
                     "global pipeline should consume virtual reference snapshot");
    ok &= expectTrue(result.hasCorrectedVirtualPoints,
                     "snapshot-backed global pipeline should correct virtual points");
    return ok;
}

bool testGlobalIcpPipelineRejectsMissingVirtualReference()
{
    using namespace i2r_rlc::localization;

    NullLocalizationVirtualReferenceProvider provider;
    LocalizationVirtualReferenceRequest referenceRequest;
    referenceRequest.referenceId = QStringLiteral("not-connected");

    LocalizationService service;
    LocalizationGlobalIcpPipeline pipeline(service);
    GlobalIcpPipelineRequest request;
    request.realScanPoints_mm = makeReferenceCloud();
    request.virtualReferenceSnapshot = provider.requestSnapshot(referenceRequest);

    const auto result = pipeline.run(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Failed,
                     "global pipeline should reject missing virtual reference");
    ok &= expectTrue(!result.hasCorrectedVirtualPoints,
                     "missing virtual reference should not produce corrected points");
    ok &= expectTrue(result.message.contains(QStringLiteral("虚拟参考点云")),
                     "missing virtual reference failure should explain reference cloud");
    return ok;
}

bool testLocalIcpRealPointCloudReplay()
{
    using namespace i2r_rlc::localization;

    const auto replayCases = findGlobalIcpReplayCases();
    if (replayCases.empty()) {
        std::cout << "SKIP: localization local ICP pointcloud fixtures not found\n";
        return true;
    }

    bool ok = true;
    int caseIndex = 0;
    for (const auto& replayCase : replayCases) {
        ++caseIndex;

        const auto realBefore = readPointCloudFile(replayCase.realBeforePath);
        const auto virtualBefore = readPointCloudFile(replayCase.virtualBeforePath);
        const auto legacyVirtualAfter = readPointCloudFile(replayCase.virtualAfterPath);

        ok &= expectTrue(!realBefore.empty(), "real replay cloud should not be empty");
        ok &= expectTrue(!virtualBefore.empty(), "virtual replay cloud should not be empty");
        ok &= expectTrue(!legacyVirtualAfter.empty(), "legacy after replay cloud should not be empty");
        if (realBefore.empty() || virtualBefore.empty() || legacyVirtualAfter.empty()) {
            continue;
        }

        const double beforeMean_mm = meanNearestDistance(realBefore, virtualBefore);
        const double legacyAfterMean_mm = meanNearestDistance(realBefore, legacyVirtualAfter);
        ok &= expectTrue(
            legacyAfterMean_mm < beforeMean_mm * 0.75,
            "legacy ICP fixture should improve nearest-neighbor residual");

        LocalizationService service;
        LocalizationLocalCorrectionPipeline pipeline(service);
        LocalCorrectionPipelineRequest request;
        request.frameId = caseIndex;
        request.realWindowPoints_mm = realBefore;
        request.virtualWindowPoints_mm = virtualBefore;
        request.windowFrameCount = 20;
        request.minimumWindowFrameCount = 20;
        request.dofMask = {true, true, true, false, false, true};
        request.parameters.enableSorFilter = false;
        request.parameters.preprocessingProfile = IcpPreprocessingProfile::CompatibilityFirstLocalIcp;
        request.parameters.maxCorrespondenceDistance_mm = 0.0;
        request.parameters.maximumIterations = 0;

        const auto result = pipeline.run(request);
        ok &= expectTrue(result.status == LocalizationStatus::Succeeded, "real replay local pipeline should succeed");
        ok &= expectTrue(result.hasCorrectedVirtualPoints, "real replay local pipeline should correct virtual points");
        ok &= expectTrue(result.icpResult.hasPoseCorrection, "real replay local ICP should return correction");
        ok &= expectNear(result.icpResult.correctionRx_deg, 0.0, 1e-5, "real replay should lock rx");
        ok &= expectNear(result.icpResult.correctionRy_deg, 0.0, 1e-5, "real replay should lock ry");

        if (result.status != LocalizationStatus::Succeeded || !result.hasCorrectedVirtualPoints) {
            continue;
        }

        const double legacyPointwiseMean_mm =
            meanPointwiseDistance(result.correctedVirtualWindowPoints_mm, legacyVirtualAfter);
        ok &= expectTrue(
            result.afterResidual_mm < beforeMean_mm * 0.90,
            "current local ICP replay should improve nearest-neighbor residual");

        std::cout << "INFO: local ICP replay case " << caseIndex
                  << ", beforeMean_mm=" << result.beforeResidual_mm
                  << ", legacyAfterMean_mm=" << legacyAfterMean_mm
                  << ", currentAfterMean_mm=" << result.afterResidual_mm
                  << ", legacyPointwiseMean_mm=" << legacyPointwiseMean_mm
                  << ", correction=(" << result.icpResult.correctionX_mm
                  << ", " << result.icpResult.correctionY_mm
                  << ", " << result.icpResult.correctionZ_mm
                  << ", rz=" << result.icpResult.correctionRz_deg << "deg)\n";
    }

    return ok;
}

bool testIcpRejectsInsufficientPoints()
{
    using namespace i2r_rlc::localization;

    LocalizationService service;
    LocalizationRequest request;
    request.mode = LocalizationMode::Global;
    request.globalIcp.realPoints_mm = {{0.0, 0.0, 0.0}};
    request.globalIcp.referencePoints_mm = {{1.0, 1.0, 1.0}};

    const auto result = service.runLocalization(request);

    bool ok = true;
    ok &= expectTrue(result.status == LocalizationStatus::Failed, "ICP should reject insufficient points");
    ok &= expectTrue(!result.hasPoseCorrection, "failed ICP should not contain pose correction");
    ok &= expectTrue(result.message.contains(QStringLiteral("点云不足")), "failure should explain point count");
    return ok;
}

bool testImagePointProjection()
{
    using namespace i2r_rlc::localization;

    SensorCalibrationData calibration;
    calibration.focalLengthM = 1.0;
    calibration.kappa = 0.0;
    calibration.sxM = 1.0;
    calibration.syM = 1.0;
    calibration.cxPixel = 10.0;
    calibration.cyPixel = 20.0;
    calibration.laserPlaneA = 0.0;
    calibration.laserPlaneB = 0.0;
    calibration.laserPlaneC = -1.0;

    ImageProjectionRequest request;
    request.calibration = calibration;
    request.toolPoseInRobotBase = {10.0, 20.0, 30.0, 0.0, 0.0, 0.0};
    request.imagePoints_px = {{10.0, 20.0}};

    const auto points = LocalizationPointProjector::projectImagePoints(request);

    bool ok = true;
    ok &= expectTrue(points.size() == 1, "projection should produce one point");
    if (!points.empty()) {
        ok &= expectNear(points.front().x_mm, 10.0, 1e-9, "projected x");
        ok &= expectNear(points.front().y_mm, 20.0, 1e-9, "projected y");
        ok &= expectNear(points.front().z_mm, 1030.0, 1e-9, "projected z");
    }
    return ok;
}

bool testLocalizationWorkflowInitialAndClearState()
{
    using namespace i2r_rlc::localization;

    LocalizationWorkflowController controller;

    bool ok = true;
    ok &= expectTrue(controller.state() == LocalizationWorkflowState::Idle,
                     "localization workflow initial state should be idle");

    const auto running = controller.startGlobalIcp(IcpDofMask{}, 1);
    ok &= expectTrue(running.state == LocalizationWorkflowState::GlobalIcpRunning,
                     "localization workflow should enter global ICP running state");

    const auto cleared = controller.clear();
    ok &= expectTrue(cleared.state == LocalizationWorkflowState::Idle,
                     "localization workflow clear should return idle update");
    ok &= expectTrue(controller.state() == LocalizationWorkflowState::Idle,
                     "localization workflow clear should reset controller state");
    return ok;
}

bool testLocalizationApplicationLocalReplay()
{
    using namespace i2r_rlc::localization;

    const auto replayCases = findGlobalIcpReplayCases();
    if (replayCases.empty()) {
        std::cout << "SKIP: localization application replay fixtures not found\n";
        return true;
    }

    LocalizationApplicationService service;
    LocalizationLocalReplayRequest request;
    request.pointCloudRootPath =
        QString::fromStdWString((projectSourceDir() / "data" / "IBVS" / "pointcloud").wstring());

    const auto result = service.runLocalCorrectionReplay(request);

    bool ok = true;
    ok &= expectTrue(result.hasLocalCorrectionResult,
                     "localization application local replay should contain local result");
    ok &= expectTrue(result.update.state == LocalizationWorkflowState::Succeeded,
                     "localization application local replay should succeed");
    ok &= expectTrue(result.update.hasResidual,
                     "localization application local replay should report residual");
    ok &= expectTrue(result.update.afterResidual_mm < result.update.beforeResidual_mm,
                     "localization application local replay should reduce residual");
    ok &= expectNear(result.update.dofMask.allowRx ? 1.0 : 0.0, 0.0, 1e-9,
                     "localization application local replay should lock rx");
    ok &= expectNear(result.update.dofMask.allowRy ? 1.0 : 0.0, 0.0, 1e-9,
                     "localization application local replay should lock ry");
    return ok;
}

bool testLocalizationApplicationGlobalDryRunMissingVirtualReference()
{
    using namespace i2r_rlc::localization;

    LocalizationApplicationService service;
    LocalizationGlobalDryRunRequest request;

    const auto result = service.runGlobalIcpDryRun(request);

    bool ok = true;
    ok &= expectTrue(result.hasGlobalIcpResult,
                     "global dry-run should contain global ICP result");
    ok &= expectTrue(result.update.state == LocalizationWorkflowState::MissingVirtualReference,
                     "global dry-run should report missing virtual reference");
    ok &= expectTrue(!result.update.succeeded,
                     "global dry-run missing virtual reference should be unsuccessful");
    ok &= expectTrue(result.update.errorMessage.contains(QStringLiteral("DigitalTwin"))
                         || result.update.errorMessage.contains(QStringLiteral("RobotSim")),
                     "global dry-run should explain DigitalTwin/RobotSim dependency");
    return ok;
}

bool testLocalizationApplicationGlobalDryRunUsesRobotSimLaserLineSnapshot()
{
    using namespace i2r_rlc::localization;

    LocalizationApplicationService service;
    LocalizationGlobalDryRunRequest request;
    request.frameId = 42;

    LocalizationVirtualReferenceSnapshot snapshot;
    snapshot.available = true;
    snapshot.referenceId = QStringLiteral("robotsim-sensor-1013");
    snapshot.laserLinePoints_mm = makeApplicationDryRunCloud();
    snapshot.laserLineImagePoints_px = {
        {812.0, 904.0},
        {823.0, 874.0}
    };
    snapshot.message = QStringLiteral("RobotSim fake laser line snapshot");

    const auto result = service.runGlobalIcpDryRun(request, snapshot);

    bool ok = true;
    ok &= expectTrue(result.hasGlobalIcpResult,
                     "global dry-run with RobotSim snapshot should contain global ICP result");
    ok &= expectTrue(result.update.state == LocalizationWorkflowState::Succeeded,
                     "global dry-run should consume RobotSim laser line snapshot");
    ok &= expectTrue(result.update.succeeded,
                     "global dry-run with RobotSim snapshot should succeed");
    ok &= expectTrue(result.globalIcpResult.hasCorrectedVirtualPoints,
                     "global dry-run should output corrected virtual points");
    return ok;
}

bool testGlobalIcpPipelineUsesSegmentReferencesAndPreviousResultAsInitialGuess()
{
    using namespace i2r_rlc::localization;

    const auto reference = makeReferenceCloud();
    GlobalIcpSegmentRequest firstSegment;
    firstSegment.segmentId = QStringLiteral("first");
    firstSegment.realPoints_mm.assign(reference.begin(), reference.begin() + 6);
    firstSegment.virtualReferencePoints_mm = firstSegment.realPoints_mm;

    GlobalIcpSegmentRequest secondSegment;
    secondSegment.segmentId = QStringLiteral("second");
    secondSegment.realPoints_mm.assign(reference.begin() + 6, reference.end());
    secondSegment.virtualReferencePoints_mm = secondSegment.realPoints_mm;

    RecordingLocalizationService service;
    LocalizationGlobalIcpPipeline pipeline(service);
    GlobalIcpPipelineRequest request;
    request.segments = {firstSegment, secondSegment};

    const auto result = pipeline.run(request);

    bool ok = true;
    ok &= expectTrue(
        result.status == LocalizationStatus::Succeeded,
        "segmented global pipeline should use per-segment references without a default cloud");
    ok &= expectTrue(service.requests.size() == 2U, "segmented global pipeline should run both segments");
    if (service.requests.size() == 2U) {
        ok &= expectNear(
            service.requests[0].globalIcp.initialGuess.rowMajor[3],
            0.0,
            1e-9,
            "first segment should use the pipeline initial guess");
        ok &= expectNear(
            service.requests[1].globalIcp.initialGuess.rowMajor[3],
            3.0,
            1e-9,
            "second segment should use the previous complete ICP result as initial guess");
    }
    ok &= expectNear(
        result.combinedVirtualToRealCorrectionTransform.rowMajor[3],
        -7.0,
        1e-9,
        "final virtual-to-real correction should invert the final accumulated result");
    return ok;
}

bool testLocalizationApplicationGlobalRuntimeUsesProvidedRealCloud()
{
    using namespace i2r_rlc::localization;

    LocalizationApplicationService service;
    LocalizationGlobalRuntimeRequest request;
    const auto reference = makeReferenceCloud();
    request.realScanPoints_mm = translatedCloud(reference, 2.0, 0.0, 0.0);
    request.virtualReferenceSnapshot.available = true;
    request.virtualReferenceSnapshot.referenceId = QStringLiteral("runtime-test");
    request.virtualReferenceSnapshot.workpiecePointCloud_mm = reference;

    const auto result = service.runGlobalIcpRuntime(request);

    bool ok = true;
    ok &= expectTrue(result.hasGlobalIcpResult,
                     "runtime global ICP should return pipeline result");
    ok &= expectTrue(result.update.state == LocalizationWorkflowState::Succeeded,
                     "runtime global ICP should succeed with provided real cloud");
    ok &= expectTrue(result.globalIcpResult.hasCorrectedVirtualPoints,
                     "runtime global ICP should correct virtual points");
    ok &= expectTrue(result.globalIcpResult.afterResidual_mm
                         < result.globalIcpResult.beforeResidual_mm * 0.5,
                     "runtime global ICP should reduce residual");
    return ok;
}

bool testLocalizationApplicationGlobalRuntimeRejectsMissingRealCloud()
{
    using namespace i2r_rlc::localization;

    LocalizationApplicationService service;
    LocalizationGlobalRuntimeRequest request;
    request.virtualReferenceSnapshot.available = true;
    request.virtualReferenceSnapshot.referenceId = QStringLiteral("runtime-test");
    request.virtualReferenceSnapshot.workpiecePointCloud_mm = makeReferenceCloud();

    const auto result = service.runGlobalIcpRuntime(request);

    bool ok = true;
    ok &= expectTrue(result.hasGlobalIcpResult,
                     "runtime global ICP missing real cloud should return pipeline result");
    ok &= expectTrue(result.globalIcpResult.status == LocalizationStatus::Failed,
                     "runtime global ICP must fail without real scan points");
    ok &= expectTrue(result.update.state == LocalizationWorkflowState::Failed,
                     "runtime global ICP missing real cloud should not use dry-run fallback");
    ok &= expectTrue(result.globalIcpResult.message.contains(QStringLiteral("真实扫描点云")),
                     "runtime global ICP should explain missing real scan points");
    return ok;
}

bool testLocalizationWorkflowDefaultDof()
{
    using namespace i2r_rlc::localization;

    const IcpDofMask mask;

    bool ok = true;
    ok &= expectTrue(mask.allowX, "default DOF should allow x");
    ok &= expectTrue(mask.allowY, "default DOF should allow y");
    ok &= expectTrue(mask.allowZ, "default DOF should allow z");
    ok &= expectTrue(!mask.allowRx, "default DOF should lock rx");
    ok &= expectTrue(!mask.allowRy, "default DOF should lock ry");
    ok &= expectTrue(mask.allowRz, "default DOF should allow rz");
    return ok;
}

} // namespace

/**
 * @brief 定位服务骨架测试入口。
 *
 * 覆盖 no-op 边界、受限 ICP 基础行为和真实点云回放夹具。
 */
int main()
{
    bool ok = true;
    ok &= testStartReportsUnavailable();
    ok &= testNoOpResult(i2r_rlc::localization::LocalizationMode::Global);
    ok &= testNoOpResult(i2r_rlc::localization::LocalizationMode::Local);
    ok &= testRealServiceStartReportsIdle();
    ok &= testGlobalIcpRecoversTranslation();
    ok &= testLocalIcpUsesWindowPoints();
    ok &= testLocalCorrectionPipelineRejectsShortWindow();
    ok &= testGlobalIcpPipelineSingleSegment();
    ok &= testGlobalIcpPipelineSegmentDofDiagnostics();
    ok &= testGlobalIcpPipelineUsesSegmentReferencesAndPreviousResultAsInitialGuess();
    ok &= testGlobalIcpPipelineConsumesVirtualSnapshot();
    ok &= testGlobalIcpPipelineRejectsMissingVirtualReference();
    ok &= testLocalIcpRealPointCloudReplay();
    ok &= testIcpRejectsInsufficientPoints();
    ok &= testImagePointProjection();
    ok &= testLocalizationWorkflowInitialAndClearState();
    ok &= testLocalizationApplicationLocalReplay();
    ok &= testLocalizationApplicationGlobalDryRunMissingVirtualReference();
    ok &= testLocalizationApplicationGlobalDryRunUsesRobotSimLaserLineSnapshot();
    ok &= testLocalizationApplicationGlobalRuntimeUsesProvidedRealCloud();
    ok &= testLocalizationApplicationGlobalRuntimeRejectsMissingRealCloud();
    ok &= testLocalizationWorkflowDefaultDof();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: localization service tests\n";
    return 0;
}
