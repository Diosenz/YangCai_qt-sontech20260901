/**
 * @file test_online_image_measurement.cpp
 * @brief 原图测量输出对错误 CAD 提示、缺测和多方向运动的行为契约。
 */
#include <cmath>
#include <cstdio>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "track/LegacyOnlineKcfDetector.h"

namespace v = i2r_rlc::vision;
namespace dt = i2r_rlc::digital_twin;
namespace {
cv::Mat scene(const cv::Point2d &point, double slope)
{
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(14));
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            const double dx = x - point.x, dy = y - point.y;
            const double stripe = -slope * std::abs(dx);
            image.at<unsigned char>(y, x) = cv::saturate_cast<unsigned char>(
                14 + 210 * std::exp(-std::pow(dy - stripe, 2) / 3.38));
        }
    }
    return image;
}
dt::DigitalTwinVisionLaserObservation observation(const cv::Point2d &point)
{
    dt::DigitalTwinVisionLaserObservation result;
    result.valid = true;
    result.seamIndex = 0;
    result.hasWeldPointImage_px = true;
    result.weldPointImage_px = {point.x, point.y};
    result.laserLineImagePoints_px = {{point.x - 100, point.y + 45},
        {point.x, point.y}, {point.x + 100, point.y + 45}};
    return result;
}
dt::DigitalTwinVisionLaserObservation observationAtFrame(const cv::Point2d &point,
    qint64 frameId)
{
    auto result = observation(point);
    result.frameId = frameId;
    return result;
}
v::WeldProjectionContext context()
{
    v::WeldProjectionContext result;
    result.valid = true;
    result.calibration.valid = true;
    result.calibration.focalLengthM = 0.01;
    result.calibration.sxM = result.calibration.syM = 0.00001;
    result.calibration.intrinsics.fxPixel = result.calibration.intrinsics.fyPixel = 1000;
    result.calibration.intrinsics.cxPixel = 320;
    result.calibration.intrinsics.cyPixel = 240;
    result.calibration.laserPlaneC = -1;
    return result;
}
}
int main()
{
    bool ok = true;
    const auto check = [&ok](bool value, const char *message) {
        if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); ok = false; }
    };
    v::LegacyOnlineKcfOptions options;
    options.enableImageMeasurementOutput = true;
    options.enableSeamRefinement = true;
    options.enableSimulationGuidance = true;
    options.enableOnlineObservationFusion = true;
    options.seamRefinement.heightPolicy = v::OnlineSeamHeightPolicy::GapCenter;
    const auto projection = context();
    const cv::Point2d actual(360, 260);
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(14));
    v::CLegacyOnlineKcfDetector missing;
    for (int i = 0; i < 4; ++i) {
        const auto result = missing.process(blank, observation({260.0 + 20 * i, 200}),
            projection, options);
        check(!result.valid && !result.diagnostic.imageObservationAccepted,
            "unanchored blank frames never manufacture a CAD measurement");
    }
    for (int mode = 0; mode < 3; ++mode) {
        v::CLegacyOnlineKcfDetector detector;
        options.fusion.mode = static_cast<v::KcfFusionMode>(mode);
        options.fusion.simulationWeight = 1;
        for (int frame = 0; frame < 5; ++frame) {
            options.referenceRevision = frame;
            const auto result = detector.process(scene(actual, 0.45),
                observation({80.0 + frame * 30, 100}), projection, options);
            check(result.valid && result.diagnostic.imageObservationAccepted
                && cv::norm(result.weldPointPixel - actual) < 3,
                "wrong CAD position/direction, fusion and revision cannot move image measurement");
        }
        const auto lost = detector.process(blank, observation(actual), projection, options);
        check(!lost.valid, "anchored full image loss does not append predicted points");
        const auto recovered = detector.process(scene(actual, 0.45), observation({-900, 900}),
            projection, options);
        check(recovered.valid && cv::norm(recovered.weldPointPixel - actual) < 3,
            "out of image CAD cannot block clear image recovery");
    }
    for (int trajectory = 0; trajectory < 4; ++trajectory) {
        v::CLegacyOnlineKcfDetector detector;
        for (int frame = 0; frame < 8; ++frame) {
            const cv::Point2d target = actual + (trajectory == 0
                ? cv::Point2d(frame * 1.5, 0)
                : (trajectory == 1 ? cv::Point2d(-frame * 1.5, frame * 0.6)
                    : cv::Point2d(-frame * 1.5, 5 * std::sin(frame * 0.4))));
            const auto result = detector.process(scene(target, 0.3 + trajectory * 0.1),
                observation({80, 80}), projection, options);
            check(result.valid && cv::norm(result.weldPointPixel - target) < 3,
                "straight, diagonal and curved motion with supported stripe slope changes remain measurable");
        }
    }
    v::CLegacyOnlineKcfDetector unidentifiable;
    cv::Mat straight(480, 640, CV_8UC1, cv::Scalar(14));
    cv::line(straight, {0, 240}, {639, 240}, cv::Scalar(220), 3);
    check(!unidentifiable.process(straight, observation({320, 240}), projection,
        options).valid, "a single bright stripe cannot establish weld identity");
    v::CLegacyOnlineKcfDetector validWithoutCad;
    dt::DigitalTwinVisionLaserObservation unavailable;
    const auto noCad = validWithoutCad.process(scene(actual, 0.45), unavailable,
        projection, options);
    check(noCad.valid && noCad.diagnostic.refinement.valid,
        "clear raw image can initialize without CAD observation, weld point or polyline");
    auto invalidProjection = projection;
    invalidProjection.valid = false;
    check(!validWithoutCad.process(scene(actual, 0.45), observation(actual),
        invalidProjection, options).valid, "invalid projection never emits a valid station point");
    v::CLegacyOnlineKcfDetector fallback;
    fallback.process(scene(actual, 0.45), observation(actual), projection, options);
    auto centerHidden = scene(actual, 0.45);
    centerHidden.colRange(static_cast<int>(actual.x) - 8,
        static_cast<int>(actual.x) + 9).setTo(14);
    const auto continued = fallback.process(centerHidden, observation({80, 80}),
        projection, options);
    check(continued.valid && !continued.diagnostic.refinement.valid
        && continued.diagnostic.coarseObservationUsed
        && continued.diagnostic.imageObservationAccepted
        && !continued.diagnostic.tracker.trainingCommitted
        && cv::norm(continued.weldPointPixel - actual) < 3,
        "visible historical feature permits quality KCF through a hidden center without training or ICP identity");
    v::CLegacyOnlineKcfDetector competition;
    competition.process(scene({180, 260}, 0.45),
        observationAtFrame({180, 260}, 50), projection, options);
    auto visibleHistorical = scene({180, 260}, 0.45);
    visibleHistorical.colRange(172, 189).setTo(14);
    cv::Mat distantCompetitor;
    cv::max(visibleHistorical, scene({500, 260}, 0.45), distantCompetitor);
    for (int frame = 0; frame < 4; ++frame) {
        const auto kept = competition.process(distantCompetitor,
            observationAtFrame({500, 260}, 51 + frame), projection, options);
        check(kept.valid && kept.diagnostic.coarseObservationUsed
            && !kept.diagnostic.refinement.valid
            && cv::norm(kept.weldPointPixel - cv::Point2d(180, 260)) < 3,
            "visible accepted historical KCF cannot be replaced by a distant similar corner after three frames");
    }
    v::CLegacyOnlineKcfDetector noJump;
    noJump.process(scene({180, 260}, 0.45), observation({180, 260}), projection, options);
    const auto unrelated = noJump.process(scene({500, 260}, 0.45),
        observation({500, 260}), projection, options);
    check(!unrelated.valid && !unrelated.diagnostic.refinement.valid,
        "occluded target cannot be replaced or retrained by a distant single valid reflection corner");
    v::CLegacyOnlineKcfDetector reacquisition;
    reacquisition.process(scene({180, 260}, 0.45), observation({180, 260}),
        projection, options);
    check(!reacquisition.process(blank, observation({180, 260}), projection, options).valid,
        "full loss interrupts measurement instead of extending a historical point");
    for (int frame = 0; frame < 3; ++frame) {
        const auto reacquired = reacquisition.process(scene({500, 260}, 0.45),
            observationAtFrame({80, 80}, frame + 20), projection, options);
        check(frame < 2 ? !reacquired.valid
                : reacquired.valid && reacquired.diagnostic.refinement.valid
                    && cv::norm(reacquired.weldPointPixel - cv::Point2d(500, 260)) < 3,
            "matching independent image identity can reacquire beyond old ROI after consecutive raw confirmation");
    }
    for (int interruption = 0; interruption < 6; ++interruption) {
        v::CLegacyOnlineKcfDetector confirmation;
        confirmation.process(scene({180, 260}, 0.45),
            observationAtFrame({180, 260}, 10), projection, options);
        confirmation.process(scene({500, 260}, 0.45),
            observationAtFrame({80, 80}, 11), projection, options);
        if (interruption == 2) {
            confirmation.notifyImageObservationGap();
        } else if (interruption == 3) {
            confirmation.process(cv::Mat(), observationAtFrame({80, 80}, 12),
                projection, options);
        } else if (interruption == 4) {
            confirmation.process(scene({500, 260}, 0.45),
                observationAtFrame({80, 80}, 12), invalidProjection, options);
        }
        const qint64 firstId = interruption == 0 ? 11
            : (interruption == 1 ? 10 : (interruption == 5 ? -1 : 13));
        const qint64 secondId = interruption == 0 ? 11
            : (interruption == 1 ? 11 : (interruption == 5 ? -1 : 14));
        confirmation.process(scene({500, 260}, 0.45),
            observationAtFrame({80, 80}, firstId), projection, options);
        const auto interrupted = confirmation.process(scene({500, 260}, 0.45),
            observationAtFrame({80, 80}, secondId), projection, options);
        check(!interrupted.valid && !interrupted.diagnostic.refinement.valid,
            "repeated, reversed, unknown or interrupted exposures cannot complete consecutive reacquisition");
    }
    v::CLegacyOnlineKcfDetector saturation;
    saturation.process(scene(actual, 0.45), observation(actual), projection, options);
    check(!saturation.process(cv::Mat(480, 640, CV_8UC1, cv::Scalar(255)),
        observation(actual), projection, options).valid,
        "a saturated frame cannot provide quality KCF or an image measurement");
    // 正式在线工作量合同以真实 refiner 调用数验证，不以机器负载相关的墙钟阈值代替。
    auto boundedOptions = options;
    boundedOptions.enableBoundedImageSearch = true;
    v::CLegacyOnlineKcfDetector boundedFallback;
    boundedFallback.process(scene(actual, 0.45), observationAtFrame(actual, 200),
        projection, options);
    bool boundedKcfValid = true;
    bool boundedKcfWork = true;
    bool boundedCycleCompleted = false;
    for (int frame = 0; frame < 96 && !boundedCycleCompleted; ++frame) {
        const auto measured = boundedFallback.process(centerHidden,
            observationAtFrame({80, 80}, 201 + frame), projection, boundedOptions);
        boundedKcfValid = boundedKcfValid && measured.valid
            && measured.diagnostic.coarseObservationUsed
            && !measured.diagnostic.refinement.valid
            && !measured.diagnostic.tracker.trainingCommitted
            && cv::norm(measured.weldPointPixel - actual) < 3;
        boundedKcfWork = boundedKcfWork
            && measured.diagnostic.imageRefinementCallCount <= 2;
        boundedCycleCompleted = measured.diagnostic.imageSearchCycleCompleted;
    }
    check(boundedKcfValid, "trusted current-image KCF remains usable during bounded reacquisition");
    check(boundedKcfWork,
        "formal online KCF never waits for a whole unbounded set of raw refinements");
    check(boundedCycleCompleted,
        "trusted KCF cannot indefinitely starve expanded image reacquisition");
    const auto boundedGeometryRestored = boundedFallback.process(scene(actual, 0.45),
        observationAtFrame({80, 80}, 350), projection, boundedOptions);
    check(boundedGeometryRestored.valid && boundedGeometryRestored.diagnostic.refinement.valid,
        "a restored local raw feature regains independent geometry while bounded search is enabled");
    v::CLegacyOnlineKcfDetector boundedReacquisition;
    boundedReacquisition.process(scene({180, 260}, 0.45),
        observationAtFrame({180, 260}, 400), projection, options);
    bool boundedRecovered = false;
    bool boundedRecoveryWork = true;
    for (int frame = 0; frame < 160 && !boundedRecovered; ++frame) {
        const auto measured = boundedReacquisition.process(scene({500, 260}, 0.45),
            observationAtFrame({80, 80}, 401 + frame), projection, boundedOptions);
        boundedRecoveryWork = boundedRecoveryWork
            && measured.diagnostic.imageRefinementCallCount <= 2;
        if (measured.valid) {
            boundedRecovered = frame >= 2 && measured.diagnostic.refinement.valid
                && cv::norm(measured.weldPointPixel - cv::Point2d(500, 260)) < 3;
            break;
        }
    }
    check(boundedRecoveryWork, "lost image identity uses at most two current raw refinements per frame");
    check(boundedRecovered, "bounded search eventually reacquires a moved image target beyond the old ROI");
    v::CLegacyOnlineKcfDetector boundedCompetition;
    boundedCompetition.process(scene({180, 260}, 0.45),
        observationAtFrame({180, 260}, 600), projection, options);
    bool boundedCompetitionValid = true;
    bool boundedCompetitionWork = true;
    for (int frame = 0; frame < 8; ++frame) {
        const auto measured = boundedCompetition.process(distantCompetitor,
            observationAtFrame({500, 260}, 601 + frame), projection, boundedOptions);
        boundedCompetitionValid = boundedCompetitionValid && measured.valid
            && measured.diagnostic.coarseObservationUsed
            && !measured.diagnostic.refinement.valid
            && cv::norm(measured.weldPointPixel - cv::Point2d(180, 260)) < 3;
        boundedCompetitionWork = boundedCompetitionWork
            && measured.diagnostic.imageRefinementCallCount <= 2;
    }
    check(boundedCompetitionValid, "bounded search preserves a visible old image identity against distant competition");
    check(boundedCompetitionWork, "competition checking cannot restore whole-frame unbounded refinement work");
    v::CLegacyOnlineKcfDetector interruptedSearch;
    bool pendingCandidateFound = false;
    qint64 searchFrameId = 700;
    for (int frame = 0; frame < 80 && !pendingCandidateFound; ++frame) {
        const auto measured = interruptedSearch.process(scene({500, 260}, 0.45),
            observationAtFrame({80, 80}, searchFrameId++), projection, boundedOptions);
        pendingCandidateFound = measured.diagnostic.imageSearchCyclePending
            && measured.diagnostic.rawRefinement.point.x > 0.0;
    }
    check(pendingCandidateFound, "expanded search has a current candidate before interruption");
    const auto candidateLost = interruptedSearch.process(blank,
        observationAtFrame({80, 80}, searchFrameId++), projection, boundedOptions);
    const auto resumedSearch = interruptedSearch.process(scene({500, 260}, 0.45),
        observationAtFrame({80, 80}, searchFrameId++), projection, boundedOptions);
    check(!candidateLost.valid && !candidateLost.diagnostic.imageObservationAccepted,
        "an interrupted candidate never survives as a measurement on a blank image");
    check(resumedSearch.diagnostic.imageSearchProposalGenerationMs == 0.0
            && resumedSearch.diagnostic.imageRefinementCallCount <= 2,
        "one rejected candidate keeps untested proposal progress instead of restarting full-image search");
    v::CLegacyOnlineKcfDetector staleHint;
    staleHint.process(scene({180, 260}, 0.45), observationAtFrame({180, 260}, 800),
        projection, options);
    staleHint.process(scene({500, 260}, 0.45), observationAtFrame({80, 80}, 801),
        projection, boundedOptions);
    bool staleRejected = true;
    for (int frame = 0; frame < 8; ++frame) {
        const auto measured = staleHint.process(blank, observationAtFrame({80, 80}, 802 + frame),
            projection, boundedOptions);
        staleRejected = staleRejected && !measured.valid
            && !measured.diagnostic.imageObservationAccepted
            && !measured.diagnostic.refinement.valid;
    }
    check(staleRejected, "cached search proposals never become measurements after the current raw image disappears");
    v::CLegacyOnlineKcfDetector initialCompatibility;
    const auto initial = initialCompatibility.process(scene(actual, 0.45),
        observationAtFrame({80, 80}, 1000), projection, options);
    check(!options.enableBoundedImageSearch && initial.valid
        && initial.diagnostic.refinement.valid && initial.diagnostic.imageSearchCycleCompleted,
        "default cold initial scan retains its existing complete raw geometry search");
    // 现场图仅验证原图几何链的可重现性，既有日志候选不是物理焊缝真值。
    const std::string fieldRoot = std::string(I2R_RLC_PROJECT_SOURCE_DIR)
        + "/tests/fixtures/online_image_measurement/";
    const char *fieldNames[] = {"frame_1120_source.png", "frame_2148_source.png"};
    const cv::Point2d imageReferences[] = {{490, 670}, {405, 610}};
    cv::Mat fieldImages[2];
    for (int i = 0; i < 2; ++i) {
        const auto image = cv::imread(fieldRoot + fieldNames[i], cv::IMREAD_GRAYSCALE);
        fieldImages[i] = image;
        check(!image.empty(), "field raw frame exists for current local regression");
        if (!image.empty()) {
            const auto reference = v::COnlineSeamRefiner::refine(image,
                imageReferences[i], options.seamRefinement);
            std::printf("field local geometry status=%d valid=%d support=%.6f rms=%.6f uncertainty=%.3f ambiguity=%.6f\n",
                static_cast<int>(reference.status), reference.valid, reference.supportRatio,
                reference.fitRmsPixel, reference.xUncertaintyPixel, reference.ambiguousColumnRatio);
            v::CLegacyOnlineKcfDetector field;
            const auto result = field.process(image, observation({319, 615}), projection, options);
            std::printf("field %s valid=%d point=[%.3f,%.3f] refinement_ms=%.3f message=%s\n",
                fieldNames[i], result.valid, result.weldPointPixel.x, result.weldPointPixel.y,
                result.diagnostic.refinementElapsedMs, result.message.c_str());
            if (i == 0) {
                check(result.valid && result.diagnostic.refinement.valid
                    && cv::norm(result.weldPointPixel - imageReferences[i]) < 8,
                    "clear field raw corner is recovered despite displaced CAD");
            } else {
                // 当前原图 .3265 的竞争列比例超过原有 .2 门，不存在已标注物理真值。
                // 此前必须 fresh geometry valid 的测试假设被原 refiner 的实际证据推翻。
                check(!reference.valid
                    && reference.status == v::OnlineSeamRefinementStatus::AmbiguousStripe
                    && !result.valid && !result.diagnostic.imageObservationAccepted
                    && !result.diagnostic.refinement.valid
                    && result.diagnostic.rawRefinement.status
                        == v::OnlineSeamRefinementStatus::AmbiguousStripe,
                    "ambiguous unanchored field image cannot manufacture a CAD measurement or independent geometry");
            }
        }
    }
    if (!fieldImages[0].empty() && !fieldImages[1].empty()) {
        v::CLegacyOnlineKcfDetector warm;
        warm.process(fieldImages[0], observationAtFrame({319, 615}, 100), projection, options);
        const auto result = warm.process(fieldImages[1], observationAtFrame({319, 615}, 101),
            projection, options);
        std::printf("field warm replay valid=%d geometry=%d kcf=%d point=[%.3f,%.3f] quality=%.6f response=%.6f refinement_ms=%.3f\n",
            result.valid, result.diagnostic.refinement.valid, result.diagnostic.coarseObservationUsed,
            result.weldPointPixel.x, result.weldPointPixel.y, result.diagnostic.coarseObservationQuality,
            result.diagnostic.tracker.responsePeak, result.diagnostic.refinementElapsedMs);
    }
    std::printf("%s: online image measurement\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
