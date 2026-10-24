/** @file test_online_seam_continuity.cpp @brief 连续退化、仿真运动及恢复反例。 */
#include <cmath>
#include <iostream>
#include <limits>
#include "track/OnlineSeamContinuity.h"
namespace v = i2r_rlc::vision;
int main()
{
    bool ok = true;
    const auto check = [&ok](bool value, const char *message) {
        if (!value) { std::cerr << "FAIL: " << message << '\n'; ok = false; }
    };
    v::COnlineSeamContinuity continuity;
    v::OnlineSeamRefinementResult geometry;
    geometry.valid = true; geometry.point = {150, 110};
    v::CKcfUpdateDiagnostic tracker;
    tracker.fusionApplied = true; tracker.gainX = tracker.gainY = 0.9;
    v::KcfFusionOptions options;
    const cv::Size size(640, 480);
    auto first = continuity.update(size, {160,110}, {162,110}, true, tracker, geometry, options);
    check(first.valid && std::abs(first.point.x - 151) < 1e-9, "clear frame preserves existing fusion");
    geometry.valid = false;
    // 连续400帧遮挡，参考每帧移动0.1px；目标必须同步移动，不能屏幕锁点。
    for (int i=1; i<=400; ++i) {
        auto r = continuity.update(size, {160+i*.1,110}, {-1,-1}, false, tracker, geometry, options);
        check(r.valid && r.source == v::OnlineSeamPointSource::SimulationPrediction
            && r.degradedFrames == i && std::abs(r.point.x - (151+i*.1)) < 1e-7,
            "prolonged smoke uses current reference with frozen learned correction");
    }
    // 强反光给出有响应却偏离的粗点，不能把它当成新真实观测。
    auto glare = continuity.update(size, {200,110}, {300,110}, true, tracker, geometry, options);
    check(glare.source == v::OnlineSeamPointSource::SimulationPrediction
        && std::abs(glare.point.x-191)<1e-7, "strong incompatible coarse response rejected");
    auto coarse = continuity.update(size,{200,110},{202,110},true,tracker,geometry,options);
    check(coarse.valid && coarse.source == v::OnlineSeamPointSource::CorrectedKcf
        && coarse.point.x < 192, "coarse fallback preserves learned right-bias correction");
    // 逐帧假峰不能用上一次降级结果不断抬高允许偏差。
    for(int i=0;i<100;++i) {
        auto r=continuity.update(size,{200,110},{202+i*.5,110},true,tracker,geometry,options);
        check(r.valid && r.point.x<=203.0001, "degraded drift bounded by last geometric reference");
    }
    geometry.valid=true;geometry.point={180,110};
    auto recovery=continuity.update(size,{200,110},{192,110},true,tracker,geometry,options);
    check(recovery.valid && recovery.source==v::OnlineSeamPointSource::Geometry
        && recovery.transitionLimited, "reacquisition limits transition instead of jumping");
    auto previousRecovery = recovery.point;
    for (int i = 0; i < 20; ++i) {
        const auto step = continuity.update(size, {200,110}, {192,110}, true, tracker, geometry, options);
        check(step.valid && cv::norm(step.point - previousRecovery) <= 2.000001,
            "recovery remains bounded beyond five frames until converged");
        previousRecovery = step.point;
    }
    check(std::abs(previousRecovery.x - 182.0) < 1e-7, "recovery reaches the measured fused target");
    continuity.reset();geometry.valid=false;
    auto reset=continuity.update(size,{250,120},{-1,-1},false,tracker,geometry,options);
    check(reset.valid && reset.point==cv::Point2d(250,120) && reset.degradedFrames==1,
        "new session cannot inherit previous bias");
    continuity.reset();
    auto startup=continuity.update(size,{250,120},{400,120},true,tracker,geometry,options);
    check(startup.source==v::OnlineSeamPointSource::SimulationPrediction
        && startup.point==cv::Point2d(250,120), "first-frame glare cannot seed persistent false offset");
    auto missing=continuity.update(size,{-1,120},{250,120},true,tracker,geometry,options);
    check(!missing.valid, "missing reference is explicit invalid input, never fabricated point");
    // 现场旧锚/分支不一致将正常 KCF 整点替换为预测；新入口仅降低其权重。
    v::COnlineSeamContinuity highQuality;
    v::COnlineSeamContinuity lowQuality;
    v::COnlineSeamContinuity legacy;
    geometry.valid = true;
    geometry.point = {250, 120};
    highQuality.updateOnline(size, {250,120}, {250,120}, 1.0, tracker, geometry, options);
    lowQuality.updateOnline(size, {250,120}, {250,120}, 1.0, tracker, geometry, options);
    legacy.update(size, {250,120}, {250,120}, true, tracker, geometry, options);
    geometry.valid = false;
    const auto high = highQuality.updateOnline(size, {250,120}, {268,120}, 1.0,
        tracker, geometry, options);
    const auto low = lowQuality.updateOnline(size, {250,120}, {268,120}, 0.1,
        tracker, geometry, options);
    check(high.valid && low.valid && high.source == v::OnlineSeamPointSource::CorrectedKcf
        && low.source == v::OnlineSeamPointSource::CorrectedKcf
        && high.point.x > low.point.x && low.point.x > 250.0
        && high.observationWeightX > low.observationWeightX
        && low.observationWeightX > 0.0
        && high.point.x <= 252.0,
        "online quality weakens a supported KCF observation without replacing it by pure prediction");
    const auto unchanged = legacy.update(size, {250,120}, {268,120}, true,
        tracker, geometry, options);
    check(unchanged.source == v::OnlineSeamPointSource::SimulationPrediction,
        "legacy entry keeps its original twelve-pixel rejection boundary");
    const auto smoke = highQuality.updateOnline(size, {251,120}, {269,120}, 0.0,
        tracker, geometry, options);
    check(smoke.valid && smoke.source == v::OnlineSeamPointSource::SimulationPrediction
        && smoke.observationWeightX == 0.0 && smoke.observationWeightY == 0.0
        && std::abs(smoke.point.x - high.point.x) < 1e-7,
        "zero image quality preserves image history without copying CAD translation");
    const auto farPeak = highQuality.updateOnline(size, {251,120}, {300,120}, 1.0,
        tracker, geometry, options);
    check(farPeak.valid && farPeak.source == v::OnlineSeamPointSource::SimulationPrediction,
        "quality weighting still rejects an unrelated distant response");
    v::COnlineSeamContinuity revised;
    geometry.valid = true;
    geometry.point = {270, 120};
    const auto revisedPoint = revised.updateOnline(size, {250,120}, {270,120}, 1.0,
        tracker, geometry, options, {260,120});
    check(revisedPoint.valid && revisedPoint.transitionLimited
        && cv::norm(revisedPoint.point - cv::Point2d(260,120)) <= 2.000001,
        "revision reprojects the last business target and limits the first recovered raw frame");
    // 新原图支系确认只能清粗点偏差，不能清业务位置来绕过恢复限幅。
    v::COnlineSeamContinuity newBranch;
    v::CKcfUpdateDiagnostic rawTracker;
    v::KcfFusionOptions rawFusion;
    rawFusion.mode = v::KcfFusionMode::KcfOnly;
    geometry.valid = true;
    geometry.point = {250, 120};
    newBranch.updateOnline(size, {250,120}, {238,120}, 1.0,
        rawTracker, geometry, rawFusion);
    geometry.valid = false;
    newBranch.updateOnline(size, {250,120}, {238,120}, 1.0,
        rawTracker, geometry, rawFusion);
    newBranch.clearCoarseObservationBias();
    geometry.valid = true;
    geometry.point = {300,120};
    auto newTarget = cv::Point2d(250,120);
    for (int frame = 0; frame < 30; ++frame) {
        const auto step = newBranch.updateOnline(size, {250,120}, {238,120}, 0.0,
            rawTracker, geometry, rawFusion);
        check(step.valid && step.source == v::OnlineSeamPointSource::Geometry
            && cv::norm(step.point - newTarget) <= 2.000001,
            "clearing old coarse bias preserves bounded business recovery");
        newTarget = step.point;
    }
    check(cv::norm(newTarget - geometry.point) < 1e-7,
        "business recovery reaches the newly confirmed image branch");
    geometry.valid = false;
    const auto newCoarse = newBranch.updateOnline(size, {250,120}, {300,120}, 1.0,
        rawTracker, geometry, rawFusion);
    check(newCoarse.valid && newCoarse.source == v::OnlineSeamPointSource::CorrectedKcf
        && cv::norm(newCoarse.correctedCoarsePoint - cv::Point2d(300,120)) < 1e-7
        && cv::norm(newCoarse.point - newTarget) < 1e-7,
        "newly verified KCF does not inherit or relearn old branch bias");
    if(ok) std::cout<<"PASS: continuous targets, 400-frame occlusion, glare, bounded drift, recovery, reset\n";
    return ok?0:1;
}
