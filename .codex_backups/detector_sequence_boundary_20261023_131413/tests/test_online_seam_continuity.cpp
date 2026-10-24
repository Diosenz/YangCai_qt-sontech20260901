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
    // 同一已验收原图序列按不同曝光间隔处理，恢复时间和真实运动响应应一致。
    const auto timedRecovery = [&](double dt, const cv::Point2d &velocity,
        bool curvedMotion) {
        v::COnlineSeamContinuity timed;
        v::OnlineSeamRefinementResult accepted;
        accepted.valid = true;
        accepted.point = {300, 120};
        v::OnlineSeamRecoveryOptions recoveryOptions;
        recoveryOptions.dtSeconds = dt;
        recoveryOptions.durationSeconds = 0.6;
        timed.updateOnline(size, {300,120}, {-1,-1}, 0.0, rawTracker,
            accepted, rawFusion, {-1,-1}, recoveryOptions);
        accepted.valid = false;
        timed.updateOnline(size, {300,120}, {-1,-1}, 0.0, rawTracker,
            accepted, rawFusion, {-1,-1}, recoveryOptions);
        accepted.valid = true;
        accepted.point = {230,120};
        recoveryOptions.dtSeconds = 0.0;
        auto step = timed.updateOnline(size, {300,120}, {-1,-1}, 0.0,
            rawTracker, accepted, rawFusion, {-1,-1}, recoveryOptions);
        check(step.valid && step.transitionLimited
            && cv::norm(step.point - cv::Point2d(300,120)) < 1e-7,
            "first reacquisition starts recovery without misreading the old error as image motion");
        recoveryOptions.dtSeconds = dt;
        cv::Point2d midpoint;
        const int frameCount = static_cast<int>(std::llround(0.6 / dt));
        for (int frame = 1; frame <= frameCount; ++frame) {
            const double time = frame * dt;
            const cv::Point2d motion = curvedMotion
                ? cv::Point2d(20.0 * std::sin(4.0 * time),
                    10.0 * (1.0 - std::cos(4.0 * time))) : velocity * time;
            accepted.point = cv::Point2d(230,120) + motion;
            step = timed.updateOnline(size, {300,120}, {-1,-1}, 0.0,
                rawTracker, accepted, rawFusion, {-1,-1}, recoveryOptions);
            check(step.valid && step.source == v::OnlineSeamPointSource::Geometry,
                "timed recovery keeps a continuous valid geometry-supported target");
            if (frame * 2 == frameCount) {
                midpoint = step.point - motion;
            }
        }
        check(cv::norm(step.point - accepted.point) < 0.5 && !step.transitionLimited,
            "confirmed geometry clears a seventy-pixel history error within the configured recovery time");
        return midpoint;
    };
    const auto fastStatic = timedRecovery(0.03, {0,0}, false);
    const auto slowStatic = timedRecovery(0.06, {0,0}, false);
    const auto positiveMotion = timedRecovery(0.03, {40,-20}, false);
    const auto negativeMotion = timedRecovery(0.06, {-40,20}, false);
    const auto turningMotion = timedRecovery(0.03, {0,0}, true);
    check(cv::norm(fastStatic - slowStatic) < 1e-6,
        "equal elapsed time gives equal history-error recovery across processing frame rates");
    check(cv::norm(positiveMotion - fastStatic) < 1e-6
        && cv::norm(negativeMotion - fastStatic) < 1e-6
        && cv::norm(turningMotion - fastStatic) < 1e-6,
        "supported straight reverse and turning image motion stays separate from recovery error");
    // 恢复中断后重新使用最近业务估计种恢复误差，不跨断测累计时间或运动。
    v::COnlineSeamContinuity interrupted;
    v::OnlineSeamRecoveryOptions timedOptions;
    timedOptions.dtSeconds = 0.03;
    geometry.valid = true;
    geometry.point = {300,120};
    interrupted.updateOnline(size, {300,120}, {-1,-1}, 0.0, rawTracker,
        geometry, rawFusion, {-1,-1}, timedOptions);
    geometry.valid = false;
    interrupted.updateOnline(size, {300,120}, {-1,-1}, 0.0, rawTracker,
        geometry, rawFusion, {-1,-1}, timedOptions);
    geometry.valid = true;
    geometry.point = {230,120};
    auto interruptedStep = interrupted.updateOnline(size, {300,120}, {-1,-1}, 0.0,
        rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    for (int frame = 0; frame < 5; ++frame) {
        interruptedStep = interrupted.updateOnline(size, {300,120}, {-1,-1}, 0.0,
            rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    }
    const auto beforeBadTime = interruptedStep;
    for (const double invalidTime : {0.0, -0.1,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
        timedOptions.dtSeconds = invalidTime;
        geometry.point += cv::Point2d(4.0, -3.0);
        interruptedStep = interrupted.updateOnline(size, {310,120}, {-1,-1}, 0.0,
            rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
        check(interruptedStep.recoveryActive
            && std::abs(interruptedStep.recoveryElapsedSeconds
                - beforeBadTime.recoveryElapsedSeconds) < 1e-9
            && cv::norm(interruptedStep.point - beforeBadTime.point) < 1e-7,
            "nonpositive and nonfinite exposure intervals cannot advance recovery or import raw and CAD motion");
    }
    // 无效曝光仍可验收当前原图；下一有效相邻曝光只使用重建后基准的增量。
    timedOptions.dtSeconds = 0.03;
    geometry.point += cv::Point2d(2.0, 1.0);
    interruptedStep = interrupted.updateOnline(size, {310,120}, {-1,-1}, 0.0,
        rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    check(interruptedStep.recoveryActive
        && std::abs(interruptedStep.recoveryElapsedSeconds
            - beforeBadTime.recoveryElapsedSeconds - 0.03) < 1e-9
        && cv::norm(interruptedStep.motionPrediction
            - beforeBadTime.point - cv::Point2d(2.0,1.0)) < 1e-7,
        "the next valid exposure uses only motion since the newly accepted local raw baseline");
    geometry.valid = false;
    timedOptions.dtSeconds = 10.0;
    interruptedStep = interrupted.updateOnline(size, {310,120}, {-1,-1}, 0.0,
        rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    const auto interruptionPoint = interruptedStep.point;
    check(!interruptedStep.recoveryActive
        && interruptedStep.recoveryElapsedSeconds == 0.0,
        "invalid raw clears the recovery clock rather than spending a missing-observation interval");
    geometry.valid = true;
    geometry.point = {210,125};
    interruptedStep = interrupted.updateOnline(size, {310,120}, {-1,-1}, 0.0,
        rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    check(interruptedStep.recoveryActive
        && interruptedStep.recoveryElapsedSeconds == 0.0
        && cv::norm(interruptedStep.point - interruptionPoint) < 1e-7,
        "first raw after an interruption restarts time and does not treat the unobserved jump as motion");
    timedOptions.dtSeconds = 0.1;
    for (int frame = 0; frame < 6; ++frame) {
        interruptedStep = interrupted.updateOnline(size, {310,120}, {-1,-1}, 0.0,
            rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    }
    check(!interruptedStep.recoveryActive
        && cv::norm(interruptedStep.point - geometry.point) < 0.5,
        "a fresh uninterrupted recovery reaches the changed raw observation within its duration");
    interrupted.reset();
    geometry.point = {400,200};
    const auto newSession = interrupted.updateOnline(size, {300,120}, {-1,-1}, 0.0,
        rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    check(!newSession.recoveryActive && newSession.recoveryElapsedSeconds == 0.0
        && cv::norm(newSession.point - geometry.point) < 1e-7,
        "session reset cannot inherit a prior recovery clock or raw-motion delta");
    // 恢复前或恢复中切换融合，改变目标但不能通过reset绕过恢复连续性。
    v::COnlineSeamContinuity changingFusion;
    auto blendOptions = options;
    auto blendTracker = tracker;
    geometry.point = {300,120};
    changingFusion.updateOnline(size, {300,120}, {-1,-1}, 0.0, blendTracker,
        geometry, blendOptions, {-1,-1}, timedOptions);
    geometry.valid = false;
    changingFusion.updateOnline(size, {300,120}, {-1,-1}, 0.0, blendTracker,
        geometry, blendOptions, {-1,-1}, timedOptions);
    geometry.valid = true;
    geometry.point = {230,120};
    auto changedFusion = changingFusion.updateOnline(size, {300,120}, {-1,-1}, 0.0,
        rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    check(changedFusion.recoveryActive
        && cv::norm(changedFusion.point - cv::Point2d(300,120)) < 1e-7,
        "switching fusion after lost raw preserves the business recovery seed");
    for (int frame = 0; frame < 3; ++frame) {
        changedFusion = changingFusion.updateOnline(size, {300,120}, {-1,-1}, 0.0,
            rawTracker, geometry, rawFusion, {-1,-1}, timedOptions);
    }
    const auto beforeFusionChange = changedFusion;
    timedOptions.dtSeconds = 0.0;
    changedFusion = changingFusion.updateOnline(size, {320,120}, {-1,-1}, 0.0,
        blendTracker, geometry, blendOptions, {-1,-1}, timedOptions);
    check(changedFusion.recoveryActive
        && cv::norm(changedFusion.point - beforeFusionChange.point) < 1e-7
        && std::abs(changedFusion.recoveryElapsedSeconds
            - beforeFusionChange.recoveryElapsedSeconds) < 1e-9,
        "fusion and CAD changes cannot masquerade as image motion or erase active recovery");
    v::COnlineSeamContinuity normalFusionChange;
    normalFusionChange.updateOnline(size, {300,120}, {-1,-1}, 0.0, rawTracker,
        geometry, rawFusion, {-1,-1}, timedOptions);
    geometry.point = {260,120};
    const auto ordinaryChange = normalFusionChange.updateOnline(size, {300,120}, {-1,-1},
        0.0, blendTracker, geometry, blendOptions, {-1,-1}, timedOptions);
    check(!ordinaryChange.recoveryActive
        && std::abs(ordinaryChange.point.x - 264.0) < 1e-7,
        "normal nondegraded fusion changes retain the established immediate reset contract");
    if(ok) std::cout<<"PASS: continuous targets, 400-frame occlusion, glare, bounded drift, recovery, reset\n";
    return ok?0:1;
}
