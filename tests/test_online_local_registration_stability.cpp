/**
 * @file test_online_local_registration_stability.cpp
 * @brief 验证稳定更新收益和未经几何验证的历史监测；不访问设备。
 */
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "OnlineLocalRegistrationStability.h"

namespace {
using Stability = i2r_rlc::ui::COnlineLocalRegistrationStability;
using Transform = i2r_rlc::localization::RigidTransform;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

Transform translation(double x)
{
    Transform correction;
    correction.rowMajor[3] = x;
    return correction;
}

Stability::Observation observation(double realX, double virtualX, qint64 frameId)
{
    return {{realX, 0.0, 0.0}, {virtualX, 0.0, 0.0}, frameId};
}

bool testCurrentGainAndWorsening()
{
    const auto low = observation(1.0, 0.0, 1);
    bool ok = expect(!Stability::evaluate({low}, {}, translation(-0.02), 1.1, 5.5).accepted,
        "low surface residual must not justify worsening the independent observation");
    ok &= expect(!Stability::evaluate({low}, {}, translation(0.15), 1.1, 5.5).accepted,
        "already aligned surface and seam require at least 0.2 mm seam gain");
    const auto normal = Stability::evaluate({low}, {}, translation(0.3), 1.1, 5.5);
    ok &= expect(normal.accepted && std::abs(normal.requiredGainMm - 0.2) < 1e-9,
        "clear independent gain may update an already aligned region");
    ok &= expect(!Stability::evaluate({observation(1.0, 0.0, 2), observation(-1.0, 0.0, 3)},
        {}, translation(0.5), 3.0, 5.5).accepted,
        "aggregate gain must never hide worsening above the existing per-frame tolerance");
    ok &= expect(Stability::evaluate({observation(3.0, 0.0, 7), observation(3.0, 0.0, 8),
        observation(0.225, 0.0, 9)}, {}, translation(0.5), 3.0, 5.5).accepted,
        "strong mean gain must tolerate one frame worsening 0.05 mm within existing measurement noise");
    ok &= expect(!Stability::evaluate({observation(4.0, 0.0, 4)}, {}, translation(0.15),
        3.0, 5.5).accepted, "relative independent gain must reach five percent");
    ok &= expect(Stability::evaluate({observation(3.0, 0.0, 5)}, {}, translation(0.5),
        3.2, 5.5).accepted, "new region above a previous low residual may still improve");
    ok &= expect(!Stability::evaluate({observation(0.6, 0.0, 6)}, {}, translation(0.1),
        3.0, 0.4).accepted, "configured independent ceiling must remain authoritative");
    return ok;
}

bool testUnverifiedHistoryCannotVeto()
{
    const std::vector<Stability::Anchor> early{
        {observation(0.0, 0.0, 1), 0.0}, {observation(10.3, 10.3, 2), 0.0}};
    const auto earlyConflict = Stability::evaluate({observation(3.0, 0.0, 3)}, early,
        translation(0.5), 3.0, 5.5);
    Stability history;
    bool ok = expect(earlyConflict.accepted && earlyConflict.regionCount == 2
        && earlyConflict.worsenedRegionCount == 2 && !earlyConflict.historyEnforced,
        "two nearby early observations must not lock later clearly useful candidates");
    ok &= expect(history.promote(translation(0.0),
        {{{100.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 10}}), "collect remote approximate correspondence");
    Transform rotation;
    const double angle = 0.01;
    rotation.rowMajor[0] = std::cos(angle);
    rotation.rowMajor[1] = -std::sin(angle);
    rotation.rowMajor[4] = std::sin(angle);
    rotation.rowMajor[5] = std::cos(angle);
    rotation.rowMajor[3] = 0.3;
    const auto rotated = Stability::evaluate({observation(0.6, 0.0, 11)}, history.anchors(),
        rotation, 3.0, 5.5);
    ok &= expect(rotated.accepted && rotated.maximumHistoricalErrorMm > 0.9
        && !rotated.historyEnforced && rotated.worsenedRegionCount == 1
        && rotated.evidenceStatus == QStringLiteral("conflicting"),
        "unverified remote correspondence may warn about a rotation but cannot veto current gain");

    history.clear();
    ok &= expect(history.promote(translation(0.0), {observation(0.0, 0.0, 20)}),
        "collect an initially zero-error approximate pair");
    const auto first = Stability::evaluate({observation(1.0, 0.0, 21)}, history.anchors(),
        translation(0.06), 3.0, 5.5);
    ok &= expect(!first.accepted, "small current gain remains insufficient independently of history");
    ok &= expect(history.promote(translation(0.06), {observation(20.0, 20.0, 22)}),
        "verified transform advances existing material points");
    ok &= expect(std::abs(history.anchors().front().observation.virtualPoint.x_mm - 0.06) < 1e-9
        && history.anchors().front().referenceErrorMm == 0.0,
        "advancing coordinates must never rebase the original historical error");
    const auto cumulative = Stability::evaluate({observation(1.0, 0.0, 23)}, history.anchors(),
        translation(0.1), 3.0, 5.5);
    ok &= expect(cumulative.accepted && cumulative.maximumHistoricalErrorMm > 0.11
        && cumulative.evidenceStatus == QStringLiteral("conflicting") && !cumulative.historyEnforced,
        "fixed historical baseline diagnoses cumulative drift without claiming material truth");
    const std::vector<Stability::Anchor> invalidated{{observation(0.0, 0.2, 24), 0.5}};
    const auto loose = Stability::evaluate({observation(1.0, 0.0, 25)}, invalidated,
        translation(0.3), 3.0, 5.5);
    ok &= expect(loose.accepted && loose.worsenedRegionCount == 1,
        "loose original ceiling must not hide a step worsening diagnostic");
    return ok;
}

bool testSpatialBudgetAndPromotion()
{
    Stability history;
    bool ok = expect(history.promote(translation(0.0),
        {observation(0.0, 0.0, 30), observation(9.0, 9.0, 31), observation(10.0, 10.0, 32)}),
        "finite verified observations may seed history");
    ok &= expect(history.anchors().size() == 2, "nearby observations must not dominate history");
    std::vector<Stability::Observation> many;
    for (int i = 2; i < 80; ++i) { many.push_back(observation(i * 10.0, i * 10.0, 40 + i)); }
    ok &= expect(history.promote(translation(0.0), many) && history.anchors().size() == 64,
        "history must stay within its fixed spatial budget");
    const auto first = history.anchors().front();
    ok &= expect(history.promote(translation(1.0), {observation(1000.0, 1000.0, 150)})
        && history.anchors().size() == 64
        && history.anchors().front().observation.frameId == first.observation.frameId
        && std::abs(history.anchors().front().observation.virtualPoint.x_mm - 1.0) < 1e-9,
        "full history retains early evidence while advancing its coordinates");
    history.clear();
    ok &= expect(history.anchors().empty(), "invalidated continuity must explicitly clear history");
    return ok;
}

/** @brief 无候选或候选拒绝时照常采样；区域计数不等同于独立证据或成熟状态。 */
bool testCollectionIndependentOfWriteback()
{
    Stability history;
    bool ok = expect(history.observe(observation(0.0, 0.0, 1))
        && history.observe(observation(5.0, 4.5, 2)), "sampling must work before any successful writeback");
    ok &= expect(history.anchors().size() == 1 && history.anchors().front().sampleCount == 2
        && history.anchors().front().lastFrameId == 2
        && std::abs(history.anchors().front().lastObservedErrorMm - 0.5) < 1e-9
        && history.anchors().front().referenceErrorMm == 0.0,
        "regional repeat sampling records observations without inflating the initial baseline");
    const auto rejected = Stability::evaluate({observation(1.0, 0.0, 3)}, history.anchors(),
        translation(0.05), 1.0, 5.5);
    ok &= expect(!rejected.accepted && history.observe(observation(20.0, 19.8, 3))
        && history.anchors().size() == 2, "rejected current gain must not prevent independent history collection");
    const auto snapshot = history.anchors();
    ok &= expect(!history.observe(observation(20.0, 20.0, 3))
        && !history.observe(observation(30.0, 30.0, 2))
        && history.anchors().size() == snapshot.size()
        && history.anchors().back().sampleCount == snapshot.back().sampleCount,
        "duplicate and old frames must neither seed nor inflate regional statistics");
    ok &= expect(history.promote(translation(0.3), {observation(20.0, 19.8, 3)})
        && history.anchors().size() == 2 && history.anchors().back().sampleCount == 1
        && std::abs(history.anchors().front().observation.virtualPoint.x_mm - 0.3) < 1e-9,
        "pose promotion advances old references without reseeding an already collected frame");
    history.clear();
    ok &= expect(history.observe(observation(0.0, 0.0, 1)), "continuity reset also resets frame deduplication");
    return ok;
}

bool testRegionalMetricsAndNoAutomaticMaturity()
{
    Stability history;
    history.observe(observation(0.0, 0.0, 1));
    for (qint64 frame = 2; frame <= 100; ++frame) { history.observe(observation(0.0, 0.0, frame)); }
    history.observe(observation(20.0, 19.0, 101));
    const auto metrics = Stability::evaluate({observation(1.0, 0.0, 102)}, history.anchors(),
        translation(0.3), 3.0, 5.5);
    bool ok = expect(metrics.accepted && metrics.regionCount == 2
        && std::abs(metrics.historyBeforeMeanMm - 0.5) < 1e-9
        && std::abs(metrics.historyAfterMeanMm - 0.5) < 1e-9
        && std::abs(metrics.historyBeforeMaxMm - 1.0) < 1e-9
        && std::abs(metrics.historyAfterMaxMm - 0.7) < 1e-9
        && std::abs(metrics.spatialSpanMm - 20.0) < 1e-9,
        "historical metrics must weight spatial regions equally rather than repeated frame counts");
    const auto zeroMove = Stability::evaluate({observation(1.0, 0.0, 103)}, history.anchors(),
        translation(0.0), 3.0, 5.5);
    ok &= expect(zeroMove.evidenceStatus == QStringLiteral("unverified_geometry")
        && !zeroMove.historyEnforced && zeroMove.reason.contains(QStringLiteral("收益不足")),
        "many repeat observations cannot manufacture geometric maturity or hide current gain failure");
    for (int i = 2; i < 70; ++i) { history.observe(observation(i * 20.0, i * 20.0, 110 + i)); }
    const auto many = Stability::evaluate({observation(1.0, 0.0, 200)}, history.anchors(),
        translation(0.0), 3.0, 5.5);
    ok &= expect(many.regionCount == 64 && many.spatialSpanMm > 1000.0
        && many.evidenceStatus == QStringLiteral("unverified_geometry") && !many.historyEnforced,
        "wide coverage and maximum capacity alone must never imply mature geometry");
    return ok;
}

bool testInvalidInputsFailClosed()
{
    const auto good = observation(1.0, 0.0, 200);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    bool ok = expect(!Stability::evaluate({}, {}, translation(0.5), 3.0, 5.5).accepted,
        "missing current observations must reject stable updates");
    auto bad = good;
    bad.realPoint.x_mm = nan;
    ok &= expect(!Stability::evaluate({bad}, {}, translation(0.5), 3.0, 5.5).accepted,
        "nonfinite observations must reject stable updates");
    ok &= expect(!Stability::evaluate({good}, {}, translation(nan), 3.0, 5.5).accepted
        && !Stability::evaluate({good}, {}, translation(0.5), nan, 5.5).accepted
        && !Stability::evaluate({good}, {}, translation(0.5), 3.0, nan).accepted,
        "nonfinite transform or thresholds must reject stable updates");
    const auto invalidHistory = Stability::evaluate({good}, {{good, nan}}, translation(0.5), 3.0, 5.5);
    ok &= expect(invalidHistory.accepted && invalidHistory.invalidHistoricalRegionCount == 1
        && !invalidHistory.historyEnforced,
        "invalid historical monitoring must not override a valid current gain decision");
    const auto invalidBoth = Stability::evaluate({bad}, {{good, nan}}, translation(0.5), 3.0, 5.5);
    ok &= expect(!invalidBoth.accepted && invalidBoth.reason.contains(QStringLiteral("当前")),
        "current input failure must remain the primary rejection reason");
    Stability history;
    history.promote(translation(0.0), {good});
    ok &= expect(!history.observe(bad) && history.anchors().size() == 1,
        "invalid standalone samples must preserve prior finite history");
    ok &= expect(!history.promote(translation(0.0), {bad}) && history.anchors().empty(),
        "invalid new promotion must clear old continuity evidence");
    history.promote(translation(0.0), {good});
    ok &= expect(!history.promote(translation(nan), {good}) && history.anchors().empty(),
        "invalid coordinate advance must clear old continuity evidence");
    return ok;
}
} // namespace

int main()
{
    bool ok = testCurrentGainAndWorsening();
    ok &= testUnverifiedHistoryCannotVeto();
    ok &= testSpatialBudgetAndPromotion();
    ok &= testCollectionIndependentOfWriteback();
    ok &= testRegionalMetricsAndNoAutomaticMaturity();
    ok &= testInvalidInputsFailClosed();
    if (ok) { std::puts("PASS: online local registration stability tests"); }
    return ok ? 0 : 1;
}
