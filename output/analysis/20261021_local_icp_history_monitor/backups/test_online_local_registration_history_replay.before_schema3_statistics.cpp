/**
 * @file test_online_local_registration_history_replay.cpp
 * @brief 默认使用合成观测回归历史监视语义，可选只读重放现场候选 JSON。
 * @details 仅调用生产收益验收，不重跑 ICP、不发送写回，不推演拒绝后的闭环轨迹。
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QStringList>
#include <QVariant>

#include "OnlineLocalRegistrationStability.h"

namespace {
using Stability = i2r_rlc::ui::COnlineLocalRegistrationStability;
using Transform = i2r_rlc::localization::RigidTransform;
using Point = Stability::Point;

struct ReplayCandidate {
    qint64 frameId = -1;
    bool baseSafetyAccepted = false;
    bool baseSafetyInferred = false;
    bool originallyAccepted = false;
    double beforeSurfaceMm = 0.0;
    double maximumErrorMm = 0.0;
    Transform correction;
    std::vector<Stability::Observation> current;
    std::vector<Stability::Anchor> history;
};

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

bool readPoint(const QJsonValue &value, Point &point)
{
    const auto array = value.toArray();
    if (array.size() != 3) { return false; }
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    point = {array.at(0).toDouble(invalid), array.at(1).toDouble(invalid),
        array.at(2).toDouble(invalid)};
    return std::isfinite(point.x_mm) && std::isfinite(point.y_mm) && std::isfinite(point.z_mm);
}

qint64 readFrameId(const QJsonValue &value)
{
    bool valid = false;
    const qint64 frameId = value.toVariant().toLongLong(&valid);
    return valid ? frameId : -1;
}

/** @brief 旧 schema 的 evaluated 仅在原 worker 安全门禁已通过后才可能为 true。 */
bool readCandidate(const QJsonObject &object, ReplayCandidate &candidate)
{
    const int schema = object.value(QStringLiteral("schema_version")).toInt();
    const auto stability = object.value(QStringLiteral("stability_protection")).toObject();
    if ((schema != 2 && schema != 3) || stability.isEmpty()) { return false; }
    candidate.frameId = readFrameId(object.value(QStringLiteral("frame_id")));
    candidate.originallyAccepted = object.value(QStringLiteral("accepted_before_archive")).toBool();
    if (object.contains(QStringLiteral("base_safety_accepted"))) {
        if (!object.value(QStringLiteral("base_safety_accepted")).isBool()) { return false; }
        candidate.baseSafetyAccepted = object.value(QStringLiteral("base_safety_accepted")).toBool();
    } else if (schema == 2 && stability.value(QStringLiteral("enabled")).toBool()) {
        // 旧归档缺 pipeline 状态和完整时限，不从残差猜测未通过的原安全条件。
        candidate.baseSafetyInferred = true;
        candidate.baseSafetyAccepted = stability.value(QStringLiteral("evaluated")).toBool()
            || candidate.originallyAccepted;
    } else {
        return false;
    }
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    candidate.beforeSurfaceMm = object.value(QStringLiteral("before_residual_mm")).toDouble(invalid);
    candidate.maximumErrorMm = object.value(QStringLiteral("options")).toObject()
        .value(QStringLiteral("maximum_independent_error_mm")).toDouble(invalid);
    const auto matrix = object.value(QStringLiteral("virtual_to_real_row_major")).toArray();
    if (matrix.size() != 16 || candidate.frameId < 0) { return false; }
    for (int i = 0; i < matrix.size(); ++i) {
        candidate.correction.rowMajor[static_cast<std::size_t>(i)] = matrix.at(i).toDouble(invalid);
    }
    for (const auto &frameValue : object.value(QStringLiteral("raw_frames")).toArray()) {
        const auto independent = frameValue.toObject().value(QStringLiteral("independent")).toObject();
        Stability::Observation observation;
        observation.frameId = readFrameId(independent.value(QStringLiteral("frame_id")));
        if (!readPoint(independent.value(QStringLiteral("real_weld_parent_mm")), observation.realPoint)
            || !readPoint(independent.value(QStringLiteral("virtual_weld_parent_mm")), observation.virtualPoint)) {
            return false;
        }
        if (!independent.value(QStringLiteral("valid")).toBool()) { observation.frameId = -1; }
        candidate.current.push_back(observation);
    }
    for (const auto &anchorValue : stability.value(QStringLiteral("anchors")).toArray()) {
        const auto anchorObject = anchorValue.toObject();
        Stability::Anchor anchor;
        anchor.observation.frameId = readFrameId(anchorObject.value(QStringLiteral("frame_id")));
        if (!readPoint(anchorObject.value(QStringLiteral("real_parent_mm")), anchor.observation.realPoint)
            || !readPoint(anchorObject.value(QStringLiteral("material_point_current_parent_mm")),
                anchor.observation.virtualPoint)) {
            return false;
        }
        anchor.referenceErrorMm = anchorObject.value(QStringLiteral("reference_error_mm")).toDouble(invalid);
        candidate.history.push_back(anchor);
    }
    return !candidate.current.empty();
}

Point transformPoint(const Point &point, const Transform &correction)
{
    const auto &m = correction.rowMajor;
    return {m[0] * point.x_mm + m[1] * point.y_mm + m[2] * point.z_mm + m[3],
        m[4] * point.x_mm + m[5] * point.y_mm + m[6] * point.z_mm + m[7],
        m[8] * point.x_mm + m[9] * point.y_mm + m[10] * point.z_mm + m[11]};
}

double distance(const Point &first, const Point &second)
{
    return std::hypot(first.x_mm - second.x_mm, first.y_mm - second.y_mm,
        first.z_mm - second.z_mm);
}

/** @brief 只重建旧历史否决的诊断标记；此结果绝不参与新候选接受决策。 */
bool oldHistoryWouldWarn(const ReplayCandidate &candidate)
{
    for (const auto &anchor : candidate.history) {
        const double before = distance(anchor.observation.realPoint, anchor.observation.virtualPoint);
        const double after = distance(anchor.observation.realPoint,
            transformPoint(anchor.observation.virtualPoint, candidate.correction));
        if (after > std::min(anchor.referenceErrorMm + 0.1, candidate.maximumErrorMm) + 1e-6
            || after > before + 0.1 + 1e-6) { return true; }
    }
    return false;
}

bool testUnverifiedHistoryCannotOverrideCurrentSafety()
{
    ReplayCandidate candidate;
    candidate.frameId = 100;
    candidate.baseSafetyAccepted = true;
    candidate.beforeSurfaceMm = 3.0;
    candidate.maximumErrorMm = 5.5;
    candidate.correction.rowMajor[3] = 0.5;
    candidate.current = {{{3.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 100}};
    candidate.history = {{{{100.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 10}, 0.0},
        {{{110.0, 0.0, 0.0}, {110.0, 0.0, 0.0}, 11}, 0.0}};
    const auto result = Stability::evaluate(candidate.current, candidate.history,
        candidate.correction, candidate.beforeSurfaceMm, candidate.maximumErrorMm);
    bool ok = expect(oldHistoryWouldWarn(candidate) && result.accepted && !result.historyEnforced,
        "two nearby unverified regions must warn without freezing clear current benefit");
    candidate.correction.rowMajor[3] = 0.02;
    const auto lowGain = Stability::evaluate(candidate.current, candidate.history,
        candidate.correction, candidate.beforeSurfaceMm, candidate.maximumErrorMm);
    ok &= expect(!lowGain.accepted, "diagnostic-only history must preserve the minimum current gain");
    candidate.correction.rowMajor[3] = -0.5;
    const auto worsening = Stability::evaluate(candidate.current, candidate.history,
        candidate.correction, candidate.beforeSurfaceMm, candidate.maximumErrorMm);
    ok &= expect(!worsening.accepted, "diagnostic-only history must preserve current per-frame safety");
    candidate.baseSafetyAccepted = false;
    ok &= expect(!(candidate.baseSafetyAccepted && result.accepted),
        "clear current benefit must never bypass a recorded base safety rejection");
    return ok;
}

bool testSchemaPolicyIsolation()
{
    const auto makeRecord = [](int schema) {
        return QJsonObject{{QStringLiteral("schema_version"), schema},
            {QStringLiteral("frame_id"), QStringLiteral("1")},
            {QStringLiteral("accepted_before_archive"), false},
            {QStringLiteral("before_residual_mm"), 3.0},
            {QStringLiteral("options"), QJsonObject{{QStringLiteral("maximum_independent_error_mm"), 5.5}}},
            {QStringLiteral("virtual_to_real_row_major"), QJsonArray{
                1, 0, 0, 0.5, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}},
            {QStringLiteral("raw_frames"), QJsonArray{QJsonObject{
                {QStringLiteral("independent"), QJsonObject{
                    {QStringLiteral("valid"), true}, {QStringLiteral("frame_id"), QStringLiteral("1")},
                    {QStringLiteral("real_weld_parent_mm"), QJsonArray{3, 0, 0}},
                    {QStringLiteral("virtual_weld_parent_mm"), QJsonArray{0, 0, 0}}}}}}},
            {QStringLiteral("stability_protection"), QJsonObject{
                {QStringLiteral("enabled"), true}, {QStringLiteral("evaluated"), true}}}};
    };
    ReplayCandidate oldCandidate;
    bool ok = expect(readCandidate(makeRecord(2), oldCandidate)
        && oldCandidate.baseSafetyAccepted && oldCandidate.baseSafetyInferred,
        "legacy evaluated flag may establish base safety only for schema two");
    auto newRecord = makeRecord(3);
    ReplayCandidate missingExplicitSafety;
    ok &= expect(!readCandidate(newRecord, missingExplicitSafety),
        "new policy archives must not silently reuse legacy base-safety inference");
    newRecord.insert(QStringLiteral("base_safety_accepted"), false);
    ReplayCandidate explicitRejection;
    ok &= expect(readCandidate(newRecord, explicitRejection) && !explicitRejection.baseSafetyAccepted
        && !explicitRejection.baseSafetyInferred,
        "explicit base safety rejection remains authoritative despite evaluated diagnostics");
    return ok;
}

/** @brief 每个候选沿用其原始窗口；未发送事务的累计门禁和后验效果不在离线证据内。 */
bool replayArchive(const QString &directory)
{
    const QDir archive(directory);
    if (!archive.exists()) { return expect(false, "archive directory does not exist"); }
    int total = 0;
    int baseRejected = 0;
    int currentRejected = 0;
    int originalRejected = 0;
    int historyWarnings = 0;
    int accepted = 0;
    int inferred = 0;
    bool ok = true;
    for (const auto &name : archive.entryList({QStringLiteral("local_icp_*.json")}, QDir::Files, QDir::Name)) {
        if (name.contains(QStringLiteral(".writeback.")) || name.contains(QStringLiteral(".verified."))
            || name.contains(QStringLiteral(".rollback."))) { continue; }
        QFile file(archive.filePath(name));
        if (!file.open(QIODevice::ReadOnly)) { return expect(false, "cannot read archived candidate"); }
        QJsonParseError error;
        const auto document = QJsonDocument::fromJson(file.readAll(), &error);
        ReplayCandidate candidate;
        if (error.error != QJsonParseError::NoError || !document.isObject()
            || !readCandidate(document.object(), candidate)) {
            std::fprintf(stderr, "FAIL: unsupported or malformed candidate: %s\n", qPrintable(name));
            return false;
        }
        const auto current = Stability::evaluate(candidate.current, {}, candidate.correction,
            candidate.beforeSurfaceMm, candidate.maximumErrorMm);
        const auto monitored = Stability::evaluate(candidate.current, candidate.history, candidate.correction,
            candidate.beforeSurfaceMm, candidate.maximumErrorMm);
        const bool warning = oldHistoryWouldWarn(candidate);
        const bool decision = candidate.baseSafetyAccepted && monitored.accepted;
        ok &= expect(!monitored.historyEnforced && monitored.accepted == current.accepted,
            "unverified archived history must not change the production current-gain decision");
        ++total;
        baseRejected += !candidate.baseSafetyAccepted;
        currentRejected += candidate.baseSafetyAccepted && !current.accepted;
        originalRejected += !candidate.originallyAccepted;
        historyWarnings += candidate.baseSafetyAccepted && warning;
        accepted += decision;
        inferred += candidate.baseSafetyInferred;
        std::printf("frame=%lld base=%d inferred=%d current=%d histwarn=%d original=%d new=%d "
            "mean=%.6f->%.6f required=%.6f history_enforced=%d evidence=%s\n",
            static_cast<long long>(candidate.frameId), candidate.baseSafetyAccepted,
            candidate.baseSafetyInferred, current.accepted, warning, candidate.originallyAccepted,
            decision, monitored.beforeMeanMm, monitored.afterMeanMm, monitored.requiredGainMm,
            monitored.historyEnforced, qPrintable(monitored.evidenceStatus));
    }
    ok &= expect(total > 0, "archive directory must contain candidate records");
    std::printf("REPLAY candidates=%d base_safety_rejected=%d current_gain_rejected=%d "
        "historical_warnings=%d original_policy_rejected=%d new_policy_accepted=%d "
        "legacy_base_inference=%d\n", total, baseRejected, currentRejected, historyWarnings,
        originalRejected, accepted, inferred);
    std::puts("Frozen-window replay only; no ICP rerun, cumulative writeback safety or closed-loop prediction.");
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    const QCoreApplication application(argc, argv);
    bool ok = testUnverifiedHistoryCannotOverrideCurrentSafety();
    ok &= testSchemaPolicyIsolation();
    const auto arguments = application.arguments();
    if (arguments.size() == 3 && arguments.at(1) == QStringLiteral("--archive-dir")) {
        ok &= replayArchive(arguments.at(2));
    } else if (arguments.size() != 1) {
        std::fprintf(stderr, "Usage: %s [--archive-dir DIRECTORY]\n", argv[0]);
        return 2;
    }
    if (ok) { std::puts("PASS: online local registration history replay tests"); }
    return ok ? 0 : 1;
}
