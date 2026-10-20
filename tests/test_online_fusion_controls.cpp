/**
 * @file test_online_fusion_controls.cpp
 * @brief 使用真实 KCF 更新验证在线融合策略与热切换，不依赖设备或仿真连接。
 */
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include "track/FilterKalman.h"

#define private public
#include "track/KcfTracker.h"
#undef private

namespace {

using i2r_rlc::vision::CKcfTracker;
using i2r_rlc::vision::KcfFusionMode;
using i2r_rlc::vision::KcfFusionOptions;

void require(bool condition, const char *message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

cv::Mat makeFrame(int dx, int dy)
{
    cv::Mat frame(160, 200, CV_8UC1, cv::Scalar(15));
    cv::Mat texture(28, 32, CV_8UC1);
    cv::RNG random(71653);
    random.fill(texture, cv::RNG::UNIFORM, 40, 240);
    texture.copyTo(frame(cv::Rect(70 + dx, 60 + dy, 32, 28)));
    return frame;
}

void initialize(CKcfTracker &tracker)
{
    tracker.init(cv::Rect(68, 58, 36, 32), makeFrame(0, 0));
}

void setPrior(CKcfTracker &tracker, double x, double y)
{
    tracker.m_Kalman_status_x = x;
    tracker.m_Kalman_status_y = y;
}

void testDefaultAutomatic()
{
    CKcfTracker implicit(false, true, false, false);
    CKcfTracker explicitMode(false, true, false, false);
    initialize(implicit);
    initialize(explicitMode);
    explicitMode.setFusionOptions(KcfFusionOptions{});
    for (int index = 0; index < 4; ++index)
    {
        // 每帧提交相同配置，必须保留原有 Kalman 协方差及首帧播种语义。
        explicitMode.setFusionOptions(KcfFusionOptions{});
        setPrior(implicit, 70.0 + index, 60.0 + index);
        setPrior(explicitMode, 70.0 + index, 60.0 + index);
        implicit.update(makeFrame(index, index));
        explicitMode.update(makeFrame(index, index));
        const auto &actual = explicitMode.lastUpdateDiagnostic();
        require(cv::norm(implicit.m_roi.tl() - explicitMode.m_roi.tl()) < 1e-6,
            "default mode changed legacy automatic result");
        require(actual.kalmanSeeded == (index == 0),
            "unchanged frame options reset Kalman history");
        require(actual.fusionApplied == (index > 0),
            "automatic fusion no longer preserves initial seeding");
    }
}

void testKcfOnly()
{
    CKcfTracker tracker(false, true, false, false);
    initialize(tracker);
    tracker.setFusionOptions({KcfFusionMode::KcfOnly, 0.5});
    setPrior(tracker, 120.0, 110.0);
    tracker.update(makeFrame(2, 1));
    const auto &actual = tracker.lastUpdateDiagnostic();
    require(actual.completed && actual.hasResponse && !actual.fusionApplied
        && !actual.kalmanSeeded && !tracker.m_Kal_init,
        "KCF-only mode used Kalman or simulation");
    require(actual.finalRoi == actual.beforeFusionRoi,
        "KCF-only output differs from raw guarded KCF");
}

void testManualWeights()
{
    for (const double weight : {0.0, 0.25, 0.5, 1.0})
    {
        CKcfTracker tracker(false, true, false, false);
        initialize(tracker);
        tracker.setFusionOptions({KcfFusionMode::ManualWeight, weight});
        setPrior(tracker, 74.0, 62.0);
        tracker.update(makeFrame(2, 1));
        const auto actual = tracker.lastUpdateDiagnostic();
        require(actual.completed && actual.fusionApplied && !actual.kalmanSeeded
            && !tracker.m_Kal_init && !actual.fusionJumpRejected,
            "manual weighting was filtered or rejected unexpectedly");
        require(std::abs(actual.gainX - (1.0 - weight)) < 1e-12
            && std::abs(actual.gainY - actual.gainX) < 1e-12,
            "manual diagnostic does not report KCF contribution");
        require(std::abs(actual.finalRoi.x
                - (weight * 74.0 + (1.0 - weight) * actual.beforeFusionRoi.x)) < 1e-4
            && std::abs(actual.finalRoi.y
                - (weight * 62.0 + (1.0 - weight) * actual.beforeFusionRoi.y)) < 1e-4,
            "manual simulation/KCF position ratio is incorrect");
        tracker.setFusionOptions({KcfFusionMode::ManualWeight, weight});
        setPrior(tracker, 74.0, 62.0);
        tracker.update(makeFrame(3, 2));
        require(tracker.lastUpdateDiagnostic().fusionApplied
            && !tracker.lastUpdateDiagnostic().kalmanSeeded,
            "manual mode lost hot-update continuity");
    }
}

void testSwitchWithoutTemplateReset()
{
    CKcfTracker tracker(false, true, false, false);
    initialize(tracker);
    tracker.update(makeFrame(1, 1));
    require(tracker.m_Kal_init, "automatic mode was not initialized");
    tracker.setSimulatedTemplate(makeFrame(0, 0), cv::Rect(68, 58, 36, 32));
    const auto *templateStorage = tracker.m_tmpl.data;
    const cv::Mat templateBefore = tracker.m_tmpl.clone();
    const auto roiBefore = tracker.m_roi;
    const int countBefore = tracker.m_imageCount;
    tracker.setFusionOptions({KcfFusionMode::ManualWeight, 0.2});
    require(!tracker.m_Kal_init && tracker.m_tmpl.data == templateStorage
        && cv::norm(templateBefore, tracker.m_tmpl, cv::NORM_INF) < 1e-12
        && tracker.m_roi == roiBefore && tracker.m_imageCount == countBefore
        && tracker.hasSimulatedTemplate(),
        "mode switch reset KCF templates or ROI");
    setPrior(tracker, 72.0, 62.0);
    tracker.update(makeFrame(2, 1));
    tracker.setFusionOptions({KcfFusionMode::ManualWeight, 0.8});
    require(!tracker.m_Kal_init, "manual ratio change retained Kalman state");
    tracker.setFusionOptions(KcfFusionOptions{});
    setPrior(tracker, 72.0, 62.0);
    tracker.update(makeFrame(3, 1));
    require(tracker.lastUpdateDiagnostic().kalmanSeeded,
        "switching back to automatic did not reseed Kalman");
    tracker.setFusionOptions(KcfFusionOptions{});
    setPrior(tracker, 73.0, 62.0);
    tracker.update(makeFrame(4, 1));
    require(!tracker.lastUpdateDiagnostic().kalmanSeeded
        && tracker.lastUpdateDiagnostic().fusionApplied,
        "automatic mode did not resume after reseeding");
}

void testManualZeroCoordinate()
{
    CKcfTracker tracker(false, true, false, false);
    const cv::Mat frame = makeFrame(-66, -56);
    tracker.init(cv::Rect(2, 2, 36, 32), frame);
    tracker.setFusionOptions({KcfFusionMode::ManualWeight, 1.0});
    setPrior(tracker, 0.0, 0.0);
    tracker.update(frame);
    const auto &actual = tracker.lastUpdateDiagnostic();
    require(actual.fusionApplied && !actual.fusionJumpRejected
        && std::abs(actual.finalRoi.x) < 1e-6 && std::abs(actual.finalRoi.y) < 1e-6,
        "valid zero simulation coordinate was ignored");
}

void testInvalidOptionsAndJumpGuard()
{
    CKcfTracker tracker(false, true, false, false);
    initialize(tracker);
    tracker.setFusionOptions({static_cast<KcfFusionMode>(999),
        std::numeric_limits<double>::quiet_NaN()});
    tracker.update(makeFrame(1, 1));
    require(tracker.lastUpdateDiagnostic().fusionMode == KcfFusionMode::AutomaticKalman
        && std::abs(tracker.lastUpdateDiagnostic().simulationWeight - 0.5) < 1e-12,
        "invalid options were not normalized");
    tracker.setFusionOptions({KcfFusionMode::ManualWeight, -3.0});
    setPrior(tracker, 74.0, 62.0);
    tracker.update(makeFrame(2, 1));
    require(tracker.lastUpdateDiagnostic().finalRoi
            == tracker.lastUpdateDiagnostic().beforeFusionRoi,
        "negative simulation weight was not clamped to zero");
    const auto previous = tracker.m_roi;
    tracker.setFusionOptions({KcfFusionMode::ManualWeight, 4.0});
    setPrior(tracker, 140.0, 120.0);
    tracker.update(makeFrame(3, 1));
    require(tracker.lastUpdateDiagnostic().fusionJumpRejected && tracker.m_roi == previous,
        "manual weight bypassed original fusion jump guard");
}

} // namespace

int main()
{
    try
    {
        testDefaultAutomatic();
        testKcfOnly();
        testManualWeights();
        testSwitchWithoutTemplateReset();
        testManualZeroCoordinate();
        testInvalidOptionsAndJumpGuard();
        std::puts("online fusion controls passed");
        return 0;
    }
    catch (const std::exception &exception)
    {
        std::fprintf(stderr, "%s\n", exception.what());
        return 1;
    }
}
