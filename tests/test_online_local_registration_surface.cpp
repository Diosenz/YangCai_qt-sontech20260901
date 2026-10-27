/**
 * @file test_online_local_registration_surface.cpp
 * @brief 已验证位姿后的母材训练/完整帧留出及schema4证据回归。
 * @details 注入独立定义的合成母材云，不等同于原图提取或现场闭环验证。
 * 临时证据目录保留，不删除文件，不访问机器人/相机/RobotSim。
 */
#include <cmath>
#include <cstdio>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>

#include "OnlineLocalRegistrationWorker.h"

namespace {
using Worker = i2r_rlc::ui::COnlineLocalRegistrationWorker;
using Point = i2r_rlc::localization::Point3D_mm;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

/** @brief 不同帧只改变面内采样位置，真实母材始终相对参考抬高给定Z。 */
std::vector<Point> surface(int frame, double offsetZ)
{
    std::vector<Point> cloud;
    const double phase = (frame % 4) * 0.4;
    for (int first = -6; first <= 6; ++first) {
        for (int second = -6; second <= 6; ++second) {
            const double a = first * 2.0 + phase;
            const double b = second * 2.0 - phase;
            cloud.push_back({a, b, offsetZ});
            cloud.push_back({30.0, a, 30.0 + b + offsetZ});
            cloud.push_back({a, 30.0, 65.0 + b + offsetZ});
        }
    }
    return cloud;
}

Worker::Options options()
{
    Worker::Options result;
    result.sampleIntervalMs = 0;
    result.minimumMatchIntervalMs = 0;
    result.minimumFrames = 8;
    result.maximumFrames = 16;
    result.maximumPointsPerCloud = 512;
    result.maximumSolveTimeMs = 2500;
    result.coverageDistanceMm = 1.0;
    result.maximumResidualMm = 2.0;
    return result;
}

Worker::CloudValidationContext evidence(const QString &directory, int frame,
    bool verified, quint64 revision)
{
    Worker::CloudValidationContext result;
    result.archiveDirectory = directory;
    result.hasVerifiedWorkpiecePose = verified;
    result.workpiecePoseRevision = revision;
    result.sourceImage = QImage(4, 4, QImage::Format_Grayscale8);
    result.sourceImage.fill(frame);
    result.evidenceContext = QJsonObject{
        {QStringLiteral("synthetic_surface_only_fixture"), true},
        {QStringLiteral("capture_time_ms"), QString::number(10000 + frame)},
        {QStringLiteral("workpiece_pose_revision"), QString::number(revision)}};
    return result;
}

bool firstUpdateNeedsReliablePoint(const QString &directory)
{
    Worker worker(nullptr);
    worker.configure(options());
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        result = worker.processClouds(1, 1, frame, surface(frame, 0.25), surface(frame, 0.0),
            evidence(directory, frame, false, 1));
    }
    return expect(!result.attempted && !result.accepted,
        "missing seam point cannot initialize surface-only scene updates");
}

bool completeFramesAndEvidence(const QString &directory)
{
    Worker worker(nullptr);
    worker.configure(options());
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        result = worker.processClouds(2, 3, frame, surface(frame, 0.25), surface(frame, 0.0),
            evidence(directory, frame, true, 7));
        if (frame < 8 && !expect(!result.attempted,
            "surface-only fit waits for eight complete frames")) { return false; }
    }
    bool ok = expect(result.attempted && result.surfaceOnly && result.archiveSucceeded,
        "verified surface-only window attempts and archives without a fabricated weld point");
    if (!result.accepted) {
        std::fprintf(stderr, "surface candidate rejected: %s; before=%.6f after=%.6f rank=%d solve=%lldms archive=%s\n",
            result.message.toUtf8().constData(), result.beforeResidualMm, result.afterResidualMm,
            result.surfaceValidation.observableRank, static_cast<long long>(result.solveElapsedMs),
            result.archivePath.toUtf8().constData());
    }
    ok &= expect(result.accepted && result.surfaceValidation.accepted
        && result.surfaceValidation.observableRank >= 1
        && !result.safetyKeypointsInParent_mm.empty(),
        "known small surface shift passes PCL, projected train/holdout and safety references");
    QFile file(result.archivePath);
    ok &= expect(file.open(QIODevice::ReadOnly), "surface archive readable");
    if (!file.isOpen()) { return false; }
    const auto record = QJsonDocument::fromJson(file.readAll()).object();
    const auto frozen = record.value(QStringLiteral("solve_virtual_parent_mm")).toArray();
    ok &= expect(!result.surfaceVerificationReference_mm.empty()
        && frozen.size() == static_cast<int>(result.surfaceVerificationReference_mm.size()),
        "fresh-frame local surface reference is the archived training virtual cloud");
    for (int index = 0; index < frozen.size(); ++index) {
        const auto point = frozen[index].toArray();
        const auto &actual = result.surfaceVerificationReference_mm[static_cast<std::size_t>(index)];
        ok &= expect(std::abs(point[0].toDouble() - actual.x_mm) < 1e-8
            && std::abs(point[1].toDouble() - actual.y_mm) < 1e-8
            && std::abs(point[2].toDouble() - actual.z_mm) < 1e-8,
            "comparison reference cannot silently include heldout or fresh real samples");
    }
    const auto seed = record.value(QStringLiteral("training_real_to_virtual_initial_guess_row_major")).toArray();
    ok &= expect(seed.size() == 16 && std::abs(seed[11].toDouble() + 0.25) < 1e-6,
        "training-only real-to-virtual seed preserves direction and is recorded separately from correction");
    const auto preflight = record.value(QStringLiteral("surface_postverification_preflight")).toObject();
    ok &= expect(result.surfaceVerificationPreflight.valid
        && preflight.value(QStringLiteral("valid")).toBool()
        && preflight.value(QStringLiteral("real_supported_ratio")).toDouble() >= 0.7,
        "accepted surface writeback has immutable holdout supportability before touching the scene");
    const auto validation = record.value(QStringLiteral("surface_validation")).toObject();
    const auto train = validation.value(QStringLiteral("training_frame_ids")).toArray();
    const auto holdout = validation.value(QStringLiteral("holdout_frame_ids")).toArray();
    ok &= expect(record.value(QStringLiteral("schema_version")).toInt() == 4
        && train.size() == 6 && train.first().toString() == QStringLiteral("1")
        && train.last().toString() == QStringLiteral("6")
        && holdout.size() == 2 && holdout.first().toString() == QStringLiteral("7")
        && holdout.last().toString() == QStringLiteral("8"),
        "schema4 records ordered disjoint training6 and holdout2 frame identities");
    const auto raw = record.value(QStringLiteral("raw_frames")).toArray();
    ok &= expect(raw.size() == 8
        && !raw.first().toObject().value(QStringLiteral("independent")).toObject()
            .value(QStringLiteral("valid")).toBool(),
        "missing independent point remains missing in raw evidence");
    const auto image = record.value(QStringLiteral("source_image")).toObject();
    ok &= expect(image.value(QStringLiteral("saved")).toBool()
        && image.value(QStringLiteral("frame_id")).toString() == QStringLiteral("1")
        && !image.value(QStringLiteral("sha256")).toString().isEmpty(),
        "one original fixture image is bound to its first-frame identity");
    const QDir archiveDirectory(QFileInfo(result.archivePath).absolutePath());
    const auto images = archiveDirectory.entryList({QStringLiteral("*.source.png")}, QDir::Files);
    ok &= expect(images.size() == 1, "one attempt saves at most one source image");
    return ok;
}

bool heldoutAndRevisionMustReject(const QString &directory)
{
    Worker worker(nullptr);
    worker.configure(options());
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        const double offset = frame <= 6 ? 0.25 : -0.25;
        result = worker.processClouds(4, 5, frame, surface(frame, offset), surface(frame, 0.0),
            evidence(directory, frame, true, 9));
    }
    bool ok = expect(result.attempted && result.surfaceOnly && !result.accepted
        && result.archiveSucceeded,
        "training surface gain cannot hide opposite heldout surface movement");
    worker.reset();
    for (int frame = 1; frame <= 8; ++frame) {
        result = worker.processClouds(5, 6, frame, surface(frame, 0.25), surface(frame, 0.0),
            evidence(directory, frame, true, frame == 8 ? 11 : 10));
    }
    ok &= expect(!result.attempted && !result.accepted && result.frameCount == 1,
        "pose revision switch clears old samples before a new surface training window");
    for (int frame = 9; frame <= 15; ++frame) {
        result = worker.processClouds(5, 6, frame, surface(frame, 0.25), surface(frame, 0.0),
            evidence(directory, frame, true, 11));
    }
    ok &= expect(result.attempted && result.surfaceOnly && result.frameCount == 8,
        "new pose revision resumes solving only after eight complete new exposures");
    return ok;
}

bool mixedWindowRetainsMeasuredWeldVeto(const QString &directory)
{
    Worker worker(nullptr);
    worker.configure(options());
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        auto context = evidence(directory, frame, true, 12);
        if (frame > 1) {
            // 只有首帧缺点，其余七帧实测明确反对将焊点上移0.25mm。
            context.independentObservation.valid = true;
            context.independentObservation.frameId = frame;
            context.independentObservation.realWeldPointInParent_mm = {0.0, 0.0, 0.0};
            context.independentObservation.virtualWeldPointInParent_mm = {0.0, 0.0, 0.0};
        }
        result = worker.processClouds(6, 7, frame,
            surface(frame, 0.25), surface(frame, 0.0), context);
    }
    return expect(result.surfaceOnly && result.surfaceValidation.accepted
        && result.archiveSucceeded && !result.accepted && !result.baseSafetyAccepted
        && result.afterIndependentErrorMm > result.beforeIndependentErrorMm + 0.1,
        "one missing frame cannot bypass seven measured weld counterexamples");
}

bool verifiedGoodPointsStillUseSurfaceChannel(const QString &directory)
{
    Worker worker(nullptr);
    auto configured = options();
    configured.stabilityProtectionEnabled = true;
    worker.configure(configured);
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        auto context = evidence(directory, frame, true, 14);
        context.independentObservation.valid = true;
        context.independentObservation.frameId = frame;
        context.independentObservation.realWeldPointInParent_mm = {0.0, 0.0, 0.25};
        context.independentObservation.virtualWeldPointInParent_mm = {0.0, 0.0, 0.0};
        result = worker.processClouds(7, 8, frame,
            surface(frame, 0.25), surface(frame, 0.0), context);
    }
    return expect(result.surfaceOnly && result.surfaceValidation.accepted
        && result.stabilityEvaluated && result.stabilityEvaluation.accepted && result.accepted,
        "verified windows with all measured points still use surface holdout/postverification channel");
}

bool measuredSubsetStillRequiresStabilityGain(const QString &directory)
{
    Worker worker(nullptr);
    auto configured = options();
    configured.stabilityProtectionEnabled = true;
    worker.configure(configured);
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        auto context = evidence(directory, frame, true, 15);
        if (frame == 8) {
            context.independentObservation.valid = true;
            context.independentObservation.frameId = frame;
            // 单帧未恶化但改善约0.006mm，达不到现有稳定保护要求的0.2mm收益。
            context.independentObservation.realWeldPointInParent_mm = {1.0, 0.0, 0.15};
            context.independentObservation.virtualWeldPointInParent_mm = {0.0, 0.0, 0.0};
        }
        result = worker.processClouds(8, 9, frame,
            surface(frame, 0.25), surface(frame, 0.0), context);
    }
    return expect(result.surfaceOnly && result.surfaceValidation.accepted
        && result.baseSafetyAccepted && result.stabilityEvaluated
        && !result.stabilityEvaluation.accepted && !result.accepted,
        "available measured subset retains stability gain gate despite other missing points");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    if (!expect(directory.isValid(), "evidence directory created")) { return 1; }
    const QString base = directory.path();
    const bool ok = firstUpdateNeedsReliablePoint(base + QStringLiteral("/initial"))
        && completeFramesAndEvidence(base + QStringLiteral("/complete"))
        && heldoutAndRevisionMustReject(base + QStringLiteral("/reject"))
        && mixedWindowRetainsMeasuredWeldVeto(base + QStringLiteral("/mixed"))
        && verifiedGoodPointsStillUseSurfaceChannel(base + QStringLiteral("/goodpoints"))
        && measuredSubsetStillRequiresStabilityGain(base + QStringLiteral("/subsetgain"));
    if (ok) { std::printf("PASS: production surface-only training/holdout and schema4 evidence\n"); }
    return ok ? 0 : 1;
}
