/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_algorithm_pipeline.cpp
 * @brief       vision algorithm pipeline 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "pipeline/WeldVisionAlgorithmPipeline.h"

namespace {

bool expectTrue(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

cv::Mat makeStripeImage(void)
{
    cv::Mat image(80, 120, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(8, 34), cv::Point(112, 40), cv::Scalar(255), 3);
    return image;
}

cv::Mat makeTJointImage(void)
{
    cv::Mat image(70, 90, CV_8UC1, cv::Scalar(0));
    cv::line(image, cv::Point(5, 20), cv::Point(45, 45), cv::Scalar(255), 3);
    cv::line(image, cv::Point(45, 45), cv::Point(84, 18), cv::Scalar(255), 3);
    return image;
}

bool testLegacyTypeMapping()
{
    using i2r_rlc::vision::ActiveContourSeamMode;
    using i2r_rlc::vision::WeldVisionEndpointMethod;
    using i2r_rlc::vision::WeldVisionTask;
    using i2r_rlc::vision::WeldVisionTaskKind;

    const WeldVisionTask snakeTask =
        WeldVisionTask::fromLegacyImageProType(3);
    const WeldVisionTask tEndpointTask =
        WeldVisionTask::fromLegacyImageProType(
            4,
            ActiveContourSeamMode::TJoint,
            0);
    const WeldVisionTask buttEndpointTask =
        WeldVisionTask::fromLegacyImageProType(
            4,
            ActiveContourSeamMode::Butt,
            1);
    const WeldVisionTask samTask =
        WeldVisionTask::fromLegacyImageProType(0);
    const WeldVisionTask isffaTask =
        WeldVisionTask::fromLegacyImageProType(1);
    const WeldVisionTask stegerTask =
        WeldVisionTask::fromLegacyImageProType(2);

    bool ok = true;
    ok &= expectTrue(snakeTask.kind == WeldVisionTaskKind::ActiveContour,
                     "legacy type 3 should map to active contour");
    ok &= expectTrue(snakeTask.activeContourMode == ActiveContourSeamMode::Automatic,
                     "legacy type 3 should default to automatic seam mode");
    ok &= expectTrue(tEndpointTask.kind == WeldVisionTaskKind::Endpoint,
                     "legacy type 4 should map to endpoint");
    ok &= expectTrue(tEndpointTask.activeContourMode == ActiveContourSeamMode::TJoint,
                     "explicit T joint mode should be preserved");
    ok &= expectTrue(tEndpointTask.endpointMethod == WeldVisionEndpointMethod::AreaChange,
                     "endpoint method 0 should map to area change");
    ok &= expectTrue(buttEndpointTask.activeContourMode == ActiveContourSeamMode::Butt,
                     "explicit butt mode should be preserved");
    ok &= expectTrue(buttEndpointTask.endpointMethod == WeldVisionEndpointMethod::CurveDistance,
                     "non-zero endpoint method should map to curve distance");
    ok &= expectTrue(samTask.kind == WeldVisionTaskKind::SamCandidate,
                     "legacy type 0 should map to SAM candidate");
    ok &= expectTrue(isffaTask.kind == WeldVisionTaskKind::IsffaCandidate,
                     "legacy type 1 should map to ISFFA candidate");
    ok &= expectTrue(stegerTask.kind == WeldVisionTaskKind::StegerCandidate,
                     "legacy type 2 should map to Steger candidate");
    return ok;
}

bool testLegacyStripeTasksRun()
{
    using i2r_rlc::vision::CWeldVisionAlgorithmPipeline;
    using i2r_rlc::vision::WeldVisionFrameInput;
    using i2r_rlc::vision::WeldVisionPipelineOptions;
    using i2r_rlc::vision::WeldVisionPipelineState;
    using i2r_rlc::vision::WeldVisionPipelineStatus;
    using i2r_rlc::vision::WeldVisionTask;

    WeldVisionFrameInput input;
    input.gray8 = makeStripeImage();

    bool ok = true;
    for (int legacyType : {0, 1, 2})
    {
        WeldVisionPipelineOptions options;
        options.task = WeldVisionTask::fromLegacyImageProType(legacyType);
        options.stripeOptions.thresholdGrayValue = 80;
        options.stripeOptions.minTotalPointCount = 20;
        options.stripeOptions.minSegmentPointCount = 5;
        options.stripeOptions.stegerMinRidgeResponse = 1.0;

        WeldVisionPipelineState state;
        const auto result = CWeldVisionAlgorithmPipeline::process(
            input,
            options,
            &state);

        ok &= expectTrue(result.valid,
                         "legacy stripe task should produce a valid candidate");
        ok &= expectTrue(result.status == WeldVisionPipelineStatus::Succeeded,
                         "legacy stripe task should succeed");
        ok &= expectTrue(!result.curveOrCenterlinePoints.empty(),
                         "legacy stripe task should output centerline points");
        ok &= expectTrue(state.processedFrameCount == 1,
                         "pipeline should count processed stripe frames");
    }
    return ok;
}

bool testEndpointTaskUsesStartPointDetector()
{
    using i2r_rlc::vision::CWeldVisionAlgorithmPipeline;
    using i2r_rlc::vision::WeldVisionFrameInput;
    using i2r_rlc::vision::WeldVisionPipelineOptions;
    using i2r_rlc::vision::WeldVisionPipelineState;
    using i2r_rlc::vision::WeldVisionPipelineStatus;
    using i2r_rlc::vision::WeldVisionTask;

    WeldVisionFrameInput input;
    input.gray8 = makeTJointImage();

    WeldVisionPipelineOptions options;
    options.task = WeldVisionTask::fromLegacyImageProType(
        4,
        i2r_rlc::vision::ActiveContourSeamMode::TJoint,
        0);
    options.activeContourOptions.initialThresholdGrayValue = 80;
    options.activeContourOptions.minConfidence = 0.05;
    options.startPointOptions.minConfidence = 0.05;

    WeldVisionPipelineState state;
    const auto result = CWeldVisionAlgorithmPipeline::process(
        input,
        options,
        &state);

    bool ok = true;
    ok &= expectTrue(result.valid, "endpoint task should produce start/end points");
    ok &= expectTrue(result.status == WeldVisionPipelineStatus::Succeeded,
                     "endpoint task should succeed");
    ok &= expectTrue(result.startPoint.valid && result.endPoint.valid,
                     "endpoint task should output both endpoints");
    ok &= expectTrue(state.processedFrameCount == 1,
                     "pipeline should count processed endpoint frame");
    return ok;
}

bool testInvalidInputIsRejectedBeforeAlgorithm()
{
    using i2r_rlc::vision::CWeldVisionAlgorithmPipeline;
    using i2r_rlc::vision::WeldVisionFrameInput;
    using i2r_rlc::vision::WeldVisionPipelineOptions;
    using i2r_rlc::vision::WeldVisionPipelineState;
    using i2r_rlc::vision::WeldVisionPipelineStatus;

    WeldVisionFrameInput input;
    input.gray8 = cv::Mat::zeros(4, 4, CV_8UC1);

    WeldVisionPipelineOptions options;
    WeldVisionPipelineState state;
    const auto result = CWeldVisionAlgorithmPipeline::process(
        input,
        options,
        &state);

    bool ok = true;
    ok &= expectTrue(!result.valid, "too-small input should be invalid");
    ok &= expectTrue(result.status == WeldVisionPipelineStatus::InvalidInput,
                     "too-small input should be rejected as invalid input");
    ok &= expectTrue(state.processedFrameCount == 0,
                     "invalid input should not update processed frame count");
    return ok;
}

} // namespace

/**
 * @brief SWv1 ImagePro 兼容路由和现代算法管线轻量测试入口。
 */
int main()
{
    bool ok = true;
    ok &= testLegacyTypeMapping();
    ok &= testLegacyStripeTasksRun();
    ok &= testEndpointTaskUsesStartPointDetector();
    ok &= testInvalidInputIsRejectedBeforeAlgorithm();

    if (!ok)
    {
        return 1;
    }

    std::cout << "PASS: vision algorithm pipeline tests\n";
    return 0;
}
