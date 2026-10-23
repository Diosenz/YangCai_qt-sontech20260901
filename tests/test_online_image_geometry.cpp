/**
 * @file test_online_image_geometry.cpp
 * @brief 真实图像主导的在线几何续接：CAD 漂移、反光、预焊点和现场原图反例。
 */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "feature/SimulationGuidedSeamEstimator.h"

namespace v = i2r_rlc::vision;
namespace {
cv::Mat scene(const cv::Point2d &center, double gap, bool tack, bool reflection)
{
    cv::Mat image(480, 640, CV_8UC1);
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            const double dx = x - center.x;
            const double stripe = center.y + (dx < 0.0 ? 0.45 : -0.45) * dx
                + (dx < 0.0 ? -gap * 0.5 : gap * 0.5)
                - (tack && std::abs(dx) <= 28.0 ? 24.0 : 0.0);
            double value = 160.0 * std::exp(-std::pow(y - stripe, 2.0) / 3.38);
            if (reflection) {
                const double fakeDx = x - center.x - 18.0;
                const double fake = center.y + (fakeDx < 0 ? 0.45 : -0.45) * fakeDx;
                value = std::max(value, 235.0 * std::exp(-std::pow(y - fake, 2.0) / 3.38));
            }
            image.at<unsigned char>(y, x) = cv::saturate_cast<unsigned char>(14.0 + value);
        }
    }
    return image;
}
std::vector<cv::Point2d> model(const cv::Point2d &center)
{
    return {{center.x - 100.0, center.y - 45.0}, center,
        {center.x + 100.0, center.y - 45.0}};
}
v::OnlineSeamRefinementResult refine(const cv::Mat &image, const cv::Point2d &center)
{
    v::OnlineSeamRefinementOptions options;
    options.heightPolicy = v::OnlineSeamHeightPolicy::GapCenter;
    return v::COnlineSeamRefiner::refine(image, center, options);
}
}
int main()
{
    bool ok = true;
    const auto check = [&ok](bool value, const char *message) {
        if (!value) { std::cerr << "FAIL: " << message << '\n'; ok = false; }
    };
    const cv::Point2d actual(320.0, 230.0);
    const cv::Point2d simulated(302.0, 236.0);
    const auto clear = scene(actual, -8.0, false, false);
    const auto raw = refine(clear, actual);
    check(raw.valid, "initial synthetic anchor is independently refined from raw pixels");
    v::SimulationGuidedSeamOptions options;
    options.enableImageContinuity = true;
    options.enableRawReacquisition = true;
    options.maximumDegradedFrames = 3;
    v::CSimulationGuidedSeamEstimator estimator;
    const v::OnlineSeamRefinementResult missing;
    estimator.update(clear, model(simulated), simulated, raw, 1, options);
    const auto reflected = scene(actual, -8.0, true, true);
    for (int frame = 1; frame <= 240; ++frame) {
        const auto wrongModel = simulated + cv::Point2d(-0.15 * frame, 0.04 * frame);
        const auto result = estimator.update(reflected, model(wrongModel), wrongModel,
            missing, 1 + frame / 30, options);
        check(result.valid && result.hasTrustedAnchor && result.hasRawShoulderSupport
            && result.imageObservationQuality > 0.0
            && cv::norm(result.point - raw.point) < 2.0,
            "visible historical shoulders hold seam through tack, brighter reflection and CAD drift");
        if (frame > 3) {
            check(!result.predictionBudgetValid && result.degradedFrames == frame,
                "budget expiry continues searching actual shoulders and CAD revision does not renew anchor");
        }
    }
    estimator.reset();
    estimator.update(clear, model(simulated), simulated, raw, 10, options);
    const cv::Mat blank(clear.size(), CV_8UC1, cv::Scalar(14));
    for (int frame = 1; frame <= 240; ++frame) {
        const auto wrongModel = simulated + cv::Point2d(-0.3 * frame, 0.1 * frame);
        const auto result = estimator.update(blank, model(wrongModel), wrongModel,
            missing, 10 + frame / 20, options);
        check(result.valid && !result.hasRawShoulderSupport
            && cv::norm(result.point - raw.point) < 2.0,
            "prolonged full occlusion never copies accumulated model translation");
    }
    estimator.reset();
    estimator.update(clear, model(simulated), simulated, raw, 30, options);
    for (int frame = 1; frame <= 160; ++frame) {
        const auto moved = actual + cv::Point2d(0.3 * frame, -0.1 * frame);
        const auto image = scene(moved, -8.0, true, false);
        const auto result = estimator.update(image, model(simulated), simulated, missing, 30, options);
        check(result.hasRawShoulderSupport && cv::norm(result.point - moved) < 2.0,
            "two shoulders measure real image motion in both axes despite stationary CAD and hidden center");
    }
    // 真实肩部运动可超过旧 16/24 px 门槛；缺肩后不能跳回很久以前的实测点。
    const auto lastVisible = actual + cv::Point2d(48.0, -16.0);
    for (int frame = 0; frame < 180; ++frame) {
        const auto result = estimator.update(blank, model(simulated), simulated,
            missing, 30, options);
        check(result.valid && !result.hasRawShoulderSupport
            && cv::norm(result.point - lastVisible) <= 8.5,
            "loss of shoulders extrapolates locally without returning to stale independent anchor");
    }
    // 路径形状仅在测试中生成；生产算法不接收直线/圆弧类别或半径。
    for (int trajectory = 0; trajectory < 3; ++trajectory) {
        estimator.reset();
        estimator.update(clear, model(simulated), simulated, raw, 35, options);
        for (int frame = 1; frame <= 160; ++frame) {
            const double step = static_cast<double>(frame);
            const cv::Point2d displacement = trajectory == 0
                ? cv::Point2d(0.32 * step, 0.0)
                : (trajectory == 1 ? cv::Point2d(-0.23 * step, 0.18 * step)
                    : cv::Point2d(0.28 * step, 9.0 * std::sin(step / 42.0)));
            const auto moved = actual + displacement;
            const auto image = scene(moved, -8.0, true, false);
            const auto result = estimator.update(image, model(simulated), simulated,
                missing, 35 + frame / 30, options);
            check(result.valid && result.hasRawShoulderSupport
                && result.estimated && (frame <= 3 || !result.predictionBudgetValid)
                && cv::norm(result.point - moved) < 2.0,
                "straight diagonal and curved image paths remain trackable beyond old anchor gates");
        }
    }
    estimator.reset();
    estimator.update(clear, model(simulated), simulated, raw, 40, options);
    const auto newGapImage = scene(actual, -3.0, false, false);
    const auto newGap = refine(newGapImage, actual);
    check(newGap.valid, "gap-change counterexample uses actual raw refinement");
    for (int frame = 0; frame < 10; ++frame) {
        const auto result = estimator.update(newGapImage, model(simulated), simulated,
            newGap, 40, options);
        check(!result.rawJumpRejected && result.hasTrustedAnchor
            && cv::norm(result.point - newGap.point) < 0.1,
            "visible real gap can change without historical extrapolation vetoing current measurement");
    }
    estimator.reset();
    auto noAnchor = estimator.update(blank, model(simulated), simulated, missing, 50, options);
    const auto seed = noAnchor.point;
    noAnchor = estimator.update(blank, model(simulated + cv::Point2d(60, 20)),
        simulated + cv::Point2d(60, 20), missing, 51, options);
    check(noAnchor.valid && !noAnchor.hasTrustedAnchor && noAnchor.degradedFrames == 2
        && cv::norm(noAnchor.point - seed) < 0.1,
        "unmeasured startup seed is not an immortal moving CAD anchor");
    // 现场原图：这里验证几何子链，不冒充缺失中间帧的完整机器人重放。
    const std::string fixture = std::string(I2R_RLC_PROJECT_SOURCE_DIR)
        + "/tests/fixtures/online_image_geometry/";
    const auto anchorImage = cv::imread(fixture + "frame_14591.png", cv::IMREAD_GRAYSCALE);
    const auto targetImage = cv::imread(fixture + "frame_15248.png", cv::IMREAD_GRAYSCALE);
    check(!anchorImage.empty() && !targetImage.empty(), "archived field pixels are available");
    if (!anchorImage.empty() && !targetImage.empty()) {
        const auto anchor = refine(anchorImage, {435.5, 608.562989});
        check(anchor.valid && anchor.hasBranchDirections
            && cv::norm(anchor.point - cv::Point2d(435.5, 608.562989)) < 1.0,
            "field anchor and branch directions are reproduced by production raw refiner");
        const std::vector<cv::Point2d> anchorModel = {
            {20, 471},
            {49, 483},
            {78, 495},
            {107, 505},
            {136, 515},
            {165, 527},
            {194, 538},
            {223, 549},
            {253, 560},
            {282, 571},
            {312, 583},
            {341, 594},
            {371, 605},
            {397, 609},
            {401, 608},
            {432, 600},
            {462, 592},
            {493, 584},
            {523, 575},
            {553, 567},
            {584, 559},
            {614, 551},
            {644, 543},
            {674, 534},
            {704, 542},
            {734, 553},
            {764, 565},
            {794, 574},
            {824, 582},
            {854, 590},
            {884, 598},
            {915, 606},
            {942, 461},
            {972, 453},
            {1001, 445},
            {1030, 437},
            {1060, 429},
            {1089, 424},
            {1119, 439},
            {1149, 454},
            {1180, 469},
            {1210, 484},
            {1241, 499},
            {1272, 515},
        };
        estimator.reset();
        estimator.update(anchorImage, anchorModel, {397, 609}, anchor, 25, options);
        const auto result = estimator.update(targetImage, model({367, 619}), {367, 619},
            missing, 26, options);
        std::cout << "field shoulder=" << result.hasRawShoulderSupport
            << " point=" << result.point << " quality=" << result.imageObservationQuality
            << " rms=" << result.maximumBranchRmsPixel
            << " supports=" << result.leftSupportColumns << "," << result.rightSupportColumns
            << " anchorSlopes=" << anchor.leftSlope << "," << anchor.rightSlope << '\n';
        check(result.hasTrustedAnchor && result.hasRawShoulderSupport
            && result.localGapCenterSupported && result.estimated && !missing.valid
            && result.imageObservationQuality > 0.0
            && cv::norm(result.point - cv::Point2d(425, 607)) < 6.0,
            "field frame15248 remains on visible V instead of model-led old output395.84,626.80");
    }
    if (ok) { std::cout << "PASS: image-led online geometry and archived reflection counterexample\n"; }
    return ok ? 0 : 1;
}
