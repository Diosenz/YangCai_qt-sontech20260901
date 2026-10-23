/**
 * @file test_online_local_surface_verification.cpp
 * @brief 表面后验的双向指标、冻结基线与连续新帧回归；不访问设备。
 */
#include <cmath>
#include <cstdio>
#include <limits>

#include "OnlineLocalSurfaceVerification.h"
#include "OnlineLocalRegistrationRecovery.h"

namespace {
using Verification = i2r_rlc::ui::COnlineLocalSurfaceVerification;
using Decision = Verification::Decision;
using Metrics = Verification::Metrics;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

Metrics baseline()
{
    return {1.0, 1.2, 0.90, 0.85};
}

// 帧计数、绝对门禁及配置专项使用固定合格的合成配对证据。
Decision observe(Verification &verification, qint64 frameId, bool valid, const Metrics &metrics)
{
    return verification.observe(frameId, valid, metrics, {true, 0.8, 0.6, 1.0});
}

bool testNewFrameStreakAndTerminalState()
{
    Verification verification(baseline(), Verification::Options{});
    const Metrics clear{0.8, 0.9, 0.95, 0.90};
    bool ok = expect(verification.baselineUsable(), "valid holdout baseline accepted");
    ok &= expect(observe(verification, 10, true, clear) == Decision::Waiting
        && verification.lastObservationGood() && verification.goodFrameCount() == 1,
        "first surface-only new frame must wait");
    ok &= expect(observe(verification, 10, false, {}) == Decision::Waiting
        && !verification.lastObservationGood() && verification.goodFrameCount() == 1
        && verification.badFrameCount() == 0,
        "duplicate invalid frame must not reset or advance evidence");
    ok &= expect(observe(verification, 9, true, clear) == Decision::Waiting
        && !verification.lastObservationGood() && verification.goodFrameCount() == 1,
        "older good frame cannot supply an extra sample");
    observe(verification, 11, true, clear);
    ok &= expect(observe(verification, 12, true, clear) == Decision::Verified
        && verification.goodFrameCount() == 3,
        "three distinct consecutive surface frames verify without a weld point");
    ok &= expect(observe(verification, 13, false, {}) == Decision::Verified
        && !verification.lastObservationGood()
        && verification.currentMetrics().realToVirtualResidualMm == clear.realToVirtualResidualMm,
        "verified decision is terminal and cannot add historical evidence");
    return ok;
}

bool testBadStreakAndCoverage()
{
    Verification verification(baseline(), Verification::Options{});
    const Metrics clear{0.8, 0.9, 0.95, 0.90};
    observe(verification, 1, true, clear);
    observe(verification, 2, true, clear);
    bool ok = expect(observe(verification, 3, false, clear) == Decision::Waiting
        && verification.goodFrameCount() == 0 && verification.badFrameCount() == 0
        && verification.lastObservationStatus() == Verification::ObservationStatus::Unavailable,
        "missing real surface breaks good streak without manufacturing worsening evidence");
    observe(verification, 4, true, clear);
    ok &= expect(verification.badFrameCount() == 0 && verification.goodFrameCount() == 1,
        "a new good surface resets consecutive bad evidence");
    const Metrics poorRealCoverage{0.8, 0.9, 0.69, 0.90};
    const Metrics poorVirtualCoverage{0.8, 0.9, 0.95, 0.69};
    observe(verification, 5, true, poorRealCoverage);
    observe(verification, 6, true, poorVirtualCoverage);
    ok &= expect(observe(verification, 6, false, {}) == Decision::Waiting
        && verification.badFrameCount() == 2,
        "duplicate bad frame cannot cause rollback");
    ok &= expect(observe(verification, 7, true, poorRealCoverage) == Decision::Rollback
        && verification.badFrameCount() == 3,
        "three bad distinct frames request rollback");
    ok &= expect(observe(verification, 8, true, clear) == Decision::Rollback,
        "rollback remains terminal after later clear surfaces");
    return ok;
}

bool testFrozenDirectionalBaseline()
{
    Verification::Options options;
    options.requiredGoodFrames = 1;
    options.maximumBadFrames = 1;
    Verification tolerated(baseline(), options);
    bool ok = expect(tolerated.observe(1, true, {1.8, 1.9, 0.7, 0.7}, {true, 0.8, 0.85, 1.0})
        == Decision::Verified,
        "paired worsening within tolerance passes regardless of old slice sampling");
    Verification asymmetric(baseline(), options);
    ok &= expect(observe(asymmetric, 1, true, {0.1, 5.51, 1.0, 1.0})
        == Decision::Rollback,
        "one direction cannot hide the other direction exceeding absolute limits");
    Verification::Options streakOptions;
    Verification creeping(baseline(), streakOptions);
    creeping.observe(1, true, baseline(), {true, 0.8, 0.85, 1.0});
    creeping.observe(2, true, baseline(), {true, 0.8, 0.89, 1.0});
    ok &= expect(creeping.observe(3, true, baseline(), {true, 0.8, 0.95, 1.0})
        == Decision::Waiting && !creeping.lastObservationGood()
        && creeping.goodFrameCount() == 0,
        "previously tolerated new-frame errors cannot relax paired worsening tolerance");
    ok &= expect(creeping.baselineMetrics().realToVirtualResidualMm == 1.0
        && creeping.baselineMetrics().virtualToRealResidualMm == 1.2,
        "frozen holdout baseline remains immutable");
    Verification absolute(baseline(), options);
    ok &= expect(observe(absolute, 1, true, {5.6, 1.0, 1.0, 1.0})
        == Decision::Rollback, "absolute residual cap still applies");
    return ok;
}

bool testInvalidEvidenceAndConfiguration()
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    Verification::Options options;
    options.maximumBadFrames = 1;
    bool ok = true;
    const Metrics invalidMetrics[] = {
        {nan, 1.0, 0.9, 0.9}, {1.0, inf, 0.9, 0.9},
        {-0.1, 1.0, 0.9, 0.9}, {1.0, -0.1, 0.9, 0.9},
        {1.0, 1.0, nan, 0.9}, {1.0, 1.0, 0.9, inf},
        {1.0, 1.0, 1.01, 0.9}, {1.0, 1.0, 0.9, -0.1}
    };
    for (const auto &metrics : invalidMetrics) {
        Verification invalid(baseline(), options);
        ok &= expect(observe(invalid, 1, true, metrics) == Decision::Waiting
            && invalid.badFrameCount() == 0 && invalid.unavailableFrameCount() == 1,
            "invalid or nonfinite metrics are missing evidence, never a fabricated worsening");
    }
    Verification missingBaseline({}, Verification::Options{});
    ok &= expect(!missingBaseline.baselineUsable()
        && missingBaseline.decision() == Decision::Rollback,
        "default metrics do not manufacture a valid pre-write holdout baseline");
    Verification badBaseline({nan, 1.0, 0.9, 0.9}, Verification::Options{});
    ok &= expect(!badBaseline.baselineUsable()
        && observe(badBaseline, 1, true, baseline()) == Decision::Rollback,
        "new frames cannot repair an invalid frozen transaction baseline");
    options.requiredGoodFrames = 0;
    Verification zeroFrames(baseline(), options);
    ok &= expect(zeroFrames.decision() == Decision::Rollback,
        "zero required good frames is invalid configuration");
    options = Verification::Options{};
    options.maximumBadFrames = 0;
    Verification zeroBadFrames(baseline(), options);
    ok &= expect(zeroBadFrames.decision() == Decision::Rollback,
        "zero maximum bad frames is invalid configuration");
    options = Verification::Options{};
    options.minimumBidirectionalCoverage = nan;
    Verification invalidCoverage(baseline(), options);
    ok &= expect(invalidCoverage.decision() == Decision::Rollback,
        "nonfinite threshold cannot verify surfaces");
    return ok;
}
bool testMovingSliceAndSameFrameCounterfactual()
{
    const Metrics frozen{1.3061662919, 0.4767739274, 0.9449, 1.0};
    Verification moving(frozen, Verification::Options{});
    const Metrics fresh[] = {
        {1.4962744289, 0.4475627589, 0.8928, 1.0},
        {1.9867825967, 0.6365605719, 0.8384, 0.9534},
        {1.7706495806, 0.6137708139, 0.8482, 0.9534}
    };
    // 现场切片指标只复现旧误拒触发；配对指标为合成，不能声称现场位姿正确。
    for (int index = 0; index < 3; ++index) {
        moving.observe(2980 + index, true, fresh[index], {true, 0.8, 0.6, 0.9});
    }
    bool ok = expect(moving.decision() == Decision::Verified,
        "new slice statistics may change while same-frame paired surfaces improve");
    Verification wrong(frozen, Verification::Options{});
    for (int index = 0; index < 3; ++index) {
        wrong.observe(10 + index, true, fresh[index], {true, 0.4, 0.6, 0.9});
    }
    ok &= expect(wrong.decision() == Decision::Rollback,
        "genuine same-frame worsening still rolls back despite absolute slice limits passing");
    Verification unsupported(frozen, Verification::Options{});
    for (int index = 0; index < 3; ++index) {
        unsupported.observe(20 + index, true, fresh[index], {false, 0.8, 0.6, 0.9});
    }
    ok &= expect(unsupported.decision() == Decision::Waiting
        && unsupported.badFrameCount() == 0 && unsupported.goodFrameCount() == 0
        && unsupported.unavailableFrameCount() == 3,
        "missing frozen support remains pending without being mistaken for three worsening frames");
    for (int index = 0; index < 3; ++index) {
        unsupported.observe(30 + index, true, fresh[index], {true, 0.8, 0.6, 0.9});
    }
    ok &= expect(unsupported.decision() == Decision::Verified,
        "new current-CAD supported evidence can verify a previously unsupported pending candidate");
    return ok;
}

bool testRecoveryRequiresConfirmationCooldownAndNewExposure()
{
    using Recovery = i2r_rlc::ui::COnlineLocalRegistrationRecovery;
    Recovery recovery;
    const Recovery::Outcome confirmed{true, true, true, true, false, false};
    bool ok = expect(recovery.complete(confirmed, 1000, 50000, 3008, 2000),
        "readback and archived rollback may recover sampling without a new session");
    ok &= expect(!recovery.allowsSample(2999, 50001, 3013),
        "new exposure cannot skip recovery cooldown");
    ok &= expect(!recovery.allowsSample(3000, 50000, 3013)
        && !recovery.allowsSample(3000, 50001, 3008),
        "buffered pre-restore exposure or old frame cannot rebuild a new window");
    ok &= expect(recovery.allowsSample(3000, 50001, 3013),
        "cooldown plus fresh exposure and newer frame permit rebuilding evidence");
    auto failed = confirmed;
    failed.commandSucceeded = false;
    ok &= expect(!recovery.complete(failed, 4000, 60000, 3100, 2000)
        && !recovery.allowsSample(100000, 99999, 99999),
        "unknown rollback must remain paused even after unlimited cooldown");
    failed = confirmed;
    failed.archived = false;
    ok &= expect(!recovery.complete(failed, 4000, 60000, 3100, 2000),
        "unarchived recovery cannot reopen automatic updates");
    failed = confirmed;
    failed.hasPendingPose = true;
    ok &= expect(!recovery.complete(failed, 4000, 60000, 3100, 2000),
        "pending worker transaction cannot be treated as confirmed recovery");
    recovery.reset();
    ok &= expect(recovery.allowsSample(0, 0, 1), "explicit new session resets prior recovery boundary");
    return ok;
}

/** @brief 不回退结案必须保持冷却和新曝光边界，不能伪造恢复尝试解锁。 */
bool testRetainedPoseSamplingBoundary()
{
    using Recovery = i2r_rlc::ui::COnlineLocalRegistrationRecovery;
    Recovery boundary;
    const Recovery::RetainedOutcome retained{true, true, true, false};
    bool ok = expect(boundary.completeRetained(retained, 1000, 50000, 100, 2000),
        "read-only confirmed retention can end pending without a restore operation");
    ok &= expect(!boundary.allowsSample(2999, 50001, 101)
        && !boundary.allowsSample(3000, 50000, 101)
        && !boundary.allowsSample(3000, 50001, 100),
        "retained pose still requires cooldown, a new exposure and a higher frame id");
    ok &= expect(boundary.allowsSample(3000, 50001, 101),
        "fresh post-retention frame resumes ICP without changing workpiece pose");
    for (const auto outcome : {Recovery::RetainedOutcome{false, true, true, false},
             Recovery::RetainedOutcome{true, false, true, false},
             Recovery::RetainedOutcome{true, true, false, false},
             Recovery::RetainedOutcome{true, true, true, true}}) {
        Recovery blocked;
        ok &= expect(!blocked.completeRetained(outcome, 1000, 50000, 100, 2000)
            && !blocked.allowsSample(99999, 99999, 99999),
            "unknown confirmation, archive failure or pending pose cannot resume retained sampling");
    }
    return ok;
}

} // namespace

int main()
{
    if (!testRetainedPoseSamplingBoundary()) { return 1; }
    bool ok = testNewFrameStreakAndTerminalState();
    ok &= testBadStreakAndCoverage();
    ok &= testFrozenDirectionalBaseline();
    ok &= testInvalidEvidenceAndConfiguration();
    ok &= testMovingSliceAndSameFrameCounterfactual();
    ok &= testRecoveryRequiresConfirmationCooldownAndNewExposure();
    if (ok) { std::puts("PASS: online local surface verification tests"); }
    return ok ? 0 : 1;
}
