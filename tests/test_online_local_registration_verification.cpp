/**
 * @file test_online_local_registration_verification.cpp
 * @brief 独立新帧对局部写回的连续后验检查与逆变换对照回归。
 */
#include <cmath>
#include <cstdio>
#include <limits>

#include "OnlineLocalRegistrationVerification.h"

namespace {
namespace ui = i2r_rlc::ui;
namespace loc = i2r_rlc::localization;
using Verification = ui::COnlineLocalRegistrationVerification;
using Decision = Verification::Decision;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

loc::RigidTransform translation()
{
    loc::RigidTransform correction;
    correction.rowMajor[3] = 3.0;
    return correction;
}

bool testConsecutiveGoodFramesAndDuplicates()
{
    Verification verification(translation(), Verification::Options{});
    bool ok = expect(verification.observe(10, true, {13.0, 2.0, 0.0}, {13.0, 2.0, 0.0})
        == Decision::Waiting, "first good frame must wait");
    ok &= expect(verification.observe(10, false, {}, {}) == Decision::Waiting
        && verification.observe(9, false, {}, {}) == Decision::Waiting,
        "duplicate and older invalid frames must not count as new evidence");
    ok &= expect(verification.observe(11, true, {103.0, 4.0, 0.0}, {103.0, 4.0, 0.0})
        == Decision::Waiting, "second distinct good frame must still wait");
    ok &= expect(verification.observe(11, true, {}, {}) == Decision::Waiting,
        "duplicate good frame must not complete verification");
    ok &= expect(verification.observe(12, true, {-47.0, 7.0, 0.0}, {-47.0, 7.0, 0.0})
        == Decision::Verified, "three distinct consecutive good frames must verify");
    ok &= expect(std::abs(verification.previousErrorMm() - 3.0) < 1e-9
        && verification.currentErrorMm() < 1e-9,
        "previous pose comparison must invert correction on the current frame point");
    ok &= expect(verification.observe(13, false, {}, {}) == Decision::Verified,
        "verified decision must remain terminal");
    return ok;
}

bool testBadFramesAndConsecutiveReset()
{
    Verification verification(translation(), Verification::Options{});
    verification.observe(1, true, {3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    verification.observe(2, true, {3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    bool ok = expect(verification.observe(3, false, {}, {}) == Decision::Waiting,
        "an invalid new frame must reset consecutive good evidence");
    ok &= expect(verification.observe(4, true, {3.0, 0.0, 0.0}, {3.0, 0.0, 0.0})
        == Decision::Waiting, "a good frame after invalid evidence must start a new streak");
    verification.observe(5, false, {}, {});
    ok &= expect(verification.observe(6, true, {0.0, 0.0, 0.0}, {3.0, 0.0, 0.0})
        == Decision::Waiting,
        "a finite point worsened by the correction must count as bad evidence");
    ok &= expect(verification.observe(6, false, {}, {}) == Decision::Waiting,
        "duplicate bad frame must not trigger rollback");
    ok &= expect(verification.observe(7, false, {}, {}) == Decision::Rollback,
        "three consecutive invalid or worsened distinct frames must request rollback");
    ok &= expect(verification.observe(8, true, {3.0, 0.0, 0.0}, {3.0, 0.0, 0.0})
        == Decision::Rollback, "rollback decision must remain terminal");
    return ok;
}

bool testRotationAndNonfiniteEvidence()
{
    auto correction = translation();
    correction.rowMajor = {0.0, -1.0, 0.0, 10.0,
        1.0, 0.0, 0.0, 20.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Verification::Options options;
    options.maximumBadFrames = 1;
    options.requiredGoodFrames = 1;
    Verification movedAway(correction, options);
    bool ok = expect(movedAway.observe(1, true, {100.0, 0.0, 0.0}, {10.0, 120.0, 0.0})
        == Decision::Rollback && movedAway.previousErrorMm() < 1e-9
        && movedAway.currentErrorMm() > 100.0,
        "rotation of a distant weld point must reject even with a finite correction");
    Verification movedCloser(correction, options);
    ok &= expect(movedCloser.observe(1, true, {10.0, 120.0, 0.0}, {10.0, 120.0, 0.0})
        == Decision::Verified && movedCloser.previousErrorMm() > 100.0,
        "inverse rotation and translation must recover the same-frame previous virtual point");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    Verification invalidReal(translation(), options);
    ok &= expect(invalidReal.observe(1, true, {nan, 0.0, 0.0}, {}) == Decision::Rollback,
        "nonfinite real observation must reject regardless of valid flag");
    Verification invalidVirtual(translation(), options);
    ok &= expect(invalidVirtual.observe(1, true, {}, {0.0, infinity, 0.0}) == Decision::Rollback,
        "nonfinite virtual observation must reject regardless of valid flag");
    correction.rowMajor[0] = nan;
    Verification invalidCorrection(correction, options);
    ok &= expect(invalidCorrection.observe(1, true, {}, {}) == Decision::Rollback,
        "nonfinite inverse correction must not produce verified evidence");
    return ok;
}
} // namespace

int main()
{
    bool ok = testConsecutiveGoodFramesAndDuplicates();
    ok &= testBadFramesAndConsecutiveReset();
    ok &= testRotationAndNonfiniteEvidence();
    if (ok) { std::puts("PASS: online local registration verification tests"); }
    return ok ? 0 : 1;
}
