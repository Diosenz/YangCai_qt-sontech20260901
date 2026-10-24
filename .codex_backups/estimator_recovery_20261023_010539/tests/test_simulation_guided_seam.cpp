/**
 * @file test_simulation_guided_seam.cpp
 * @brief 独立生成时序场景验证反光、预焊点续接、预算及工件位姿版本。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "feature/SimulationGuidedSeamEstimator.h"
#include "track/LegacyOnlineKcfDetector.h"

namespace {
namespace v = i2r_rlc::vision;

cv::Mat makeImage(const cv::Point2d &center, double gap,
    bool tack, bool realVisible, bool reflection)
{
    cv::Mat image(480, 640, CV_8UC1);
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            const double dx = x - center.x;
            const double stripeY = center.y + (dx < 0.0 ? 0.45 * dx : -0.45 * dx)
                + (dx < 0.0 ? -gap * 0.5 : gap * 0.5)
                - (tack && std::abs(dx) <= 28.0 ? 24.0 : 0.0);
            const double distance = y - stripeY;
            double signal = realVisible
                ? 215.0 * std::exp(-distance * distance / (2.0 * 1.3 * 1.3)) : 0.0;
            if (reflection) {
                // 更弱反光比真实 +/-0.45 斜率更像名义 +/-0.35/0.30 模型。
                const double reflectionDx = x - (center.x - 18.0);
                const double reflectionY = center.y + 26.0
                    + (reflectionDx < 0.0 ? 0.35 : -0.30) * reflectionDx;
                const double reflectionDistance = y - reflectionY;
                signal = std::max(signal, 180.0 * std::exp(
                    -reflectionDistance * reflectionDistance / (2.0 * 1.3 * 1.3)));
            }
            image.at<unsigned char>(y, x) = cv::saturate_cast<unsigned char>(14.0 + signal);
        }
    }
    return image;
}

std::vector<cv::Point2d> reference(const cv::Point2d &seam)
{
    return {{seam.x - 100.0, seam.y - 35.0}, seam,
        {seam.x + 100.0, seam.y - 30.0}};
}

v::OnlineSeamRefinementResult rawPoint(const cv::Point2d &point, double gap)
{
    v::OnlineSeamRefinementResult raw;
    raw.valid = true;
    raw.horizontalValid = true;
    raw.point = point;
    raw.leftEndpoint = {point.x, point.y - gap * 0.5};
    raw.rightEndpoint = {point.x, point.y + gap * 0.5};
    raw.status = v::OnlineSeamRefinementStatus::Succeeded;
    return raw;
}

v::OnlineSeamRefinementResult supportedRawPoint(const cv::Point2d &point, double gap)
{
    auto raw = rawPoint(point, gap);
    raw.supportRatio = 1.0;
    raw.fitRmsPixel = 0.1;
    raw.confidence = 0.9;
    raw.leftSupportColumns = raw.rightSupportColumns = 60;
    return raw;
}

} // namespace

int main()
{
    bool passed = true;
    const auto check = [&passed](bool condition, const char *message) {
        if (!condition) { std::cerr << "FAIL: " << message << '\n'; passed = false; }
    };
    const cv::Point2d actual(320.0, 230.0);
    const cv::Point2d simulated(302.0, 236.0);
    const auto clear = makeImage(actual, -8.0, false, true, false);
    const auto glare = makeImage(actual, -8.0, false, true, true);
    const auto tack = makeImage(actual, -8.0, true, true, true);
    const auto soleReflection = makeImage(actual, -8.0, false, false, true);
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(14));
    v::SimulationGuidedSeamOptions options;
    v::CSimulationGuidedSeamEstimator estimator;
    const v::OnlineSeamRefinementResult missing;
    auto result = estimator.update(glare, reference(simulated), simulated, missing, 1, options);
    check(result.valid && result.estimated && !result.hasTrustedAnchor,
        "without a measured anchor model fallback remains estimated");
    result = estimator.update(soleReflection, reference(simulated), simulated, missing, 1, options);
    check(result.valid && result.estimated && !result.hasTrustedAnchor,
        "sole model-like reflection cannot bootstrap a fake measured anchor");
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 1, options);
    // 从正确实测点开始，反光即使被上游局部几何错误标为 valid 也不能绕过时序门禁。
    result = estimator.update(glare, reference(simulated), simulated,
        rawPoint({actual.x - 18.0, actual.y + 26.0}, 0.0), 1, options);
    check(result.rawJumpRejected && result.valid && result.estimated
        && cv::norm(result.point - actual) <= options.maximumLocalCorrectionPixel + 0.01,
        "valid-looking 31.6px reflection jump is rejected after motion compensation");
    for (int frame = 0; frame < 60; ++frame) {
        result = estimator.update(tack, reference(simulated), simulated, missing, 1, options);
        check(result.valid && result.estimated && result.hasRawShoulderSupport
            && !result.localGapCenterSupported && cv::norm(result.point - actual) < 1.0,
            "tack keeps original GapCenter from visible shoulders without inventing a measured joint");
    }
    result = estimator.update(soleReflection, reference(simulated), simulated, missing, 1, options);
    check(result.valid && result.estimated && !result.hasRawShoulderSupport
        && cv::norm(result.point - actual) < 0.01,
        "only reflection remaining uses bounded history prediction, not reflection measurement");
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 1, options);
    const cv::Point2d movement(60.0, -20.0);
    const auto moved = makeImage(actual + movement, -8.0, false, true, false);
    result = estimator.update(moved, reference(simulated + movement), simulated + movement,
        missing, 1, options);
    check(result.valid && cv::norm(result.point - actual - movement) < 1.0,
        "large simulated camera motion is allowed when compensated correction remains small");
    result = estimator.update(moved, reference(simulated + movement), simulated + movement,
        rawPoint(actual + movement, -8.0), 1, options);
    check(!result.rawJumpRejected && result.hasTrustedAnchor,
        "large raw pixel motion with matching reference motion refreshes the correct anchor");
    result = estimator.update(moved, reference(simulated + movement), simulated + movement,
        missing, 2, options);
    check(result.valid && result.estimated && !result.hasTrustedAnchor
        && cv::norm(result.point - simulated - movement) < 0.01,
        "workpiece pose revision clears old measured residual while preserving model fallback");
    const cv::Point2d transitionTarget = actual + movement;
    result = estimator.update(blank, reference(simulated + movement), simulated + movement,
        missing, 21, options, transitionTarget);
    check(result.valid && result.estimated && !result.hasTrustedAnchor
        && cv::norm(result.point - transitionTarget) < 0.01,
        "reprojected revision transition preserves the business point without creating a measured anchor");
    const cv::Point2d nextMotion(2.0, -1.0);
    result = estimator.update(blank, reference(simulated + movement + nextMotion),
        simulated + movement + nextMotion, missing, 21, options);
    check(result.valid && result.estimated && !result.hasTrustedAnchor
        && cv::norm(result.point - transitionTarget - nextMotion) < 0.01,
        "unmeasured transition target follows subsequent same-version reference motion");
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 22, options);
    const cv::Point2d edgeSimulation(630.0, simulated.y);
    result = estimator.update(blank, reference(edgeSimulation), edgeSimulation,
        missing, 22, options);
    check(result.valid && result.estimated && !result.hasRawShoulderSupport
        && !result.predictionBudgetValid && result.point.x >= 0.0 && result.point.x < blank.cols
        && result.point.y >= 0.0 && result.point.y < blank.rows,
        "off-image historical offset falls back to an in-image estimated business target");
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 3, options);
    options.maximumDegradedFrames = 3;
    for (int frame = 0; frame < 3; ++frame) {
        result = estimator.update(blank, reference(simulated), simulated, missing, 3, options);
        check(result.valid && result.predictionBudgetValid, "prediction remains available inside explicit budget");
    }
    result = estimator.update(blank, reference(simulated), simulated, missing, 3, options);
    check(result.valid && result.estimated && !result.predictionBudgetValid,
        "expired frame budget degrades quality without interrupting the business target");
    options.maximumDegradedFrames = 180;
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 4, options);
    bool slowDriftRejected = false;
    for (int frame = 1; frame <= 120; ++frame) {
        const auto drifting = rawPoint(actual + cv::Point2d(0.0, frame * 0.25), -8.0);
        result = estimator.update(clear, reference(simulated), simulated, drifting, 4, options);
        slowDriftRejected = slowDriftRejected || result.rawJumpRejected;
        if (result.valid) {
            check(cv::norm(result.point - actual)
                    <= options.maximumAccumulatedAnchorCorrectionPixel
                        + options.maximumLocalCorrectionPixel + 0.01,
                "small per-frame false candidates cannot accumulate unbounded drift");
        }
    }
    check(slowDriftRejected, "raw candidates drifting away from the fixed image are rejected");
    // 事故反例：真实原图与候选一起连续移动，不能因累计离首锚超过 16 px 锁死。
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 6, options);
    cv::Point2d continuousActual = actual;
    for (int frame = 1; frame <= 120; ++frame) {
        continuousActual = actual + cv::Point2d(0.0, frame * 0.25);
        const auto continuousImage = makeImage(continuousActual, -8.0, false, true, false);
        result = estimator.update(continuousImage, reference(simulated), simulated,
            rawPoint(continuousActual, -8.0), 6, options);
        check(!result.rawJumpRejected && result.hasTrustedAnchor,
            "supported small frame motion can accumulate beyond the first-anchor offset");
    }
    result = estimator.update(blank, reference(simulated), simulated, missing, 6, options);
    check(result.valid && cv::norm(result.point - continuousActual) < 0.01,
        "fallback advances from the latest accepted continuous measurement");
    estimator.reset();
    options.maximumPredictionTimeMs = 20;
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 5, options);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    result = estimator.update(blank, reference(simulated), simulated, missing, 5, options);
    check(result.valid && result.estimated && !result.predictionBudgetValid,
        "elapsed time expires quality budget while preserving a marked business estimate");
    // 现场原图接缝已清晰恢复，但旧锚窄分支长期否决；需连续新证据而非单帧换锚。
    options.maximumPredictionTimeMs = 12000;
    options.enableRawReacquisition = true;
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 8, options);
    const cv::Point2d shiftedActual = actual + cv::Point2d(18.0, 0.0);
    const auto shiftedImage = makeImage(shiftedActual, -8.0, false, true, false);
    for (int frame = 1; frame <= 3; ++frame) {
        result = estimator.update(shiftedImage, reference(simulated), simulated,
            supportedRawPoint(shiftedActual, -8.0), 8, options);
        check(result.reacquisitionFrames == frame
            && result.reacquisitionPending == (frame < 3)
            && result.rawJumpRejected == (frame < 3)
            && result.reacquired == (frame == 3),
            "only three consecutive high-quality new raw frames can replace a stale anchor");
    }
    result = estimator.update(blank, reference(simulated), simulated, missing, 8, options);
    check(result.valid && cv::norm(result.point - shiftedActual) < 0.01,
        "reacquisition prediction uses the latest measured anchor rather than the stale branch");
    // 审查反例：旧真实 V 仍清晰，18px 内更亮反光能独立通过原图精定位。
    // 次峰低于0.8亮度比不代表旧分支消失，不能只凭新候选连续性替换实测锚。
    cv::Mat competingImage = clear.clone();
    for (int y = 0; y < competingImage.rows; ++y) {
        for (int x = 0; x < competingImage.cols; ++x) {
            const double previousStripe = 14.0
                + (competingImage.at<unsigned char>(y, x) - 14.0) * 0.55;
            competingImage.at<unsigned char>(y, x) = std::max(
                cv::saturate_cast<unsigned char>(previousStripe),
                shiftedImage.at<unsigned char>(y, x));
        }
    }
    v::OnlineSeamRefinementOptions competingOptions;
    competingOptions.heightPolicy = v::OnlineSeamHeightPolicy::GapCenter;
    const auto competingRaw = v::COnlineSeamRefiner::refine(
        competingImage, shiftedActual, competingOptions);
    check(competingRaw.valid && cv::norm(competingRaw.point - shiftedActual) < 2.0,
        "nearby brighter competing V produces a reachable high-quality raw candidate");
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 12, options);
    for (int frame = 0; frame < 3; ++frame) {
        result = estimator.update(competingImage, reference(simulated), simulated,
            competingRaw, 12, options);
        check(result.rawJumpRejected && !result.reacquired,
            "visible historical seam blocks reacquisition by a brighter eighteen-pixel reflection");
    }
    result = estimator.update(blank, reference(simulated), simulated, missing, 12, options);
    check(result.valid && cv::norm(result.point - actual) < 0.01,
        "nearby competing reflection never replaces the original measured anchor");
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 9, options);
    estimator.update(shiftedImage, reference(simulated), simulated,
        supportedRawPoint(shiftedActual, -8.0), 9, options);
    estimator.update(blank, reference(simulated), simulated, missing, 9, options);
    result = estimator.update(shiftedImage, reference(simulated), simulated,
        supportedRawPoint(shiftedActual, -8.0), 9, options);
    check(result.reacquisitionFrames == 1 && result.rawJumpRejected,
        "missing original-image evidence breaks the reacquisition sequence");
    for (int frame = 0; frame < 5; ++frame) {
        result = estimator.update(glare, reference(simulated), simulated,
            supportedRawPoint({actual.x - 18.0, actual.y + 26.0}, 0.0), 9, options);
        check(result.rawJumpRejected && !result.reacquired,
            "a persistent distant model-like reflection cannot replace the measured anchor");
    }
    options.enableRawReacquisition = false;
    estimator.reset();
    estimator.update(clear, reference(simulated), simulated, rawPoint(actual, -8.0), 10, options);
    for (int frame = 0; frame < 4; ++frame) {
        result = estimator.update(shiftedImage, reference(simulated), simulated,
            supportedRawPoint(shiftedActual, -8.0), 10, options);
        check(result.rawJumpRejected && !result.reacquired && !result.reacquisitionPending,
            "default guided entry preserves the original rejection behavior");
    }
    // 验证生产接线：空图可提供显式仿真估计，但不能产生原图实测。
    v::CLegacyOnlineKcfDetector detector;
    i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation observation;
    observation.valid = true;
    observation.hasWeldPointImage_px = true;
    observation.weldPointImage_px = {simulated.x, simulated.y};
    for (const auto &point : reference(simulated)) {
        observation.laserLineImagePoints_px.push_back({point.x, point.y});
    }
    v::LegacyOnlineKcfOptions detectorOptions;
    detectorOptions.enableSeamRefinement = true;
    detectorOptions.enableSimulationGuidance = true;
    detectorOptions.referenceRevision = 7;
    v::WeldProjectionContext projectionContext;
    projectionContext.valid = true;
    projectionContext.calibration.valid = true;
    projectionContext.calibration.focalLengthM = 0.01;
    projectionContext.calibration.sxM = 0.00001;
    projectionContext.calibration.syM = 0.00001;
    projectionContext.calibration.intrinsics.cxPixel = 320.0;
    projectionContext.calibration.intrinsics.cyPixel = 240.0;
    projectionContext.calibration.laserPlaneC = -1.0;
    const auto detected = detector.process(blank, observation,
        projectionContext, detectorOptions);
    check(detected.valid && detected.stationPoint.valid && detected.diagnostic.guided.attempted
        && !detected.diagnostic.guided.hasTrustedAnchor
        && !detected.diagnostic.refinement.valid
        && detected.diagnostic.continuity.source == v::OnlineSeamPointSource::SimulationPrediction,
        "production detector emits a projected model estimate without inventing raw measurement");
    // 保持正常 KCF 原图输入，仅由已存在的高度策略令精定位降级，验证最终选择生产分支。
    v::CLegacyOnlineKcfDetector onlineDetector;
    detectorOptions.enableOnlineObservationFusion = true;
    detectorOptions.referenceRevision = 11;
    observation.weldPointImage_px = {actual.x, actual.y};
    observation.laserLineImagePoints_px.clear();
    for (const auto &point : reference(actual)) {
        observation.laserLineImagePoints_px.push_back({point.x, point.y});
    }
    bool usedRawKcf = false;
    for (int frame = 0; frame < 8; ++frame) {
        const auto degraded = onlineDetector.process(clear, observation,
            projectionContext, detectorOptions);
        check(degraded.valid && degraded.diagnostic.attempted,
            "online geometry degradation keeps detecting KCF and displaying a projected business point");
        if (degraded.diagnostic.coarseObservationUsed) {
            usedRawKcf = true;
            check(!degraded.diagnostic.refinement.valid
                && degraded.diagnostic.coarseObservationQuality > 0.0
                && degraded.diagnostic.continuity.source == v::OnlineSeamPointSource::CorrectedKcf,
                "supported coarse observation survives the guided branch and remains non-independent");
        }
    }
    check(usedRawKcf, "production online fusion uses supported real KCF during nonfatal geometry degradation");
    const auto occluded = onlineDetector.process(blank, observation, projectionContext, detectorOptions);
    check(occluded.valid && !occluded.diagnostic.coarseObservationUsed
        && occluded.diagnostic.coarseObservationQuality == 0.0
        && !occluded.diagnostic.refinement.valid
        && occluded.diagnostic.continuity.source == v::OnlineSeamPointSource::SimulationPrediction,
        "fully occluded production frame predicts visibly without using fake KCF evidence");
    if (passed) {
        std::cout << "PASS: temporal anchor, valid-looking reflection, tack GapCenter, pose version, motion compensation, drift and budgets\n";
    }
    return passed ? 0 : 1;
}
