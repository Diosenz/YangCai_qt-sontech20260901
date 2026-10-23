/**
 * @file test_online_seam_geometry_worker.cpp
 * @brief 独立 CAD 与母材窗口验证生产 worker 的候选重求交及原有门禁。
 * @details 仅用倾斜直边与三正交面，保留临时证据，不访问设备或修改 RobotSim。
 */
#include <cmath>
#include <cstdio>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QTemporaryDir>

#include "OnlineLocalRegistrationWorker.h"

namespace {
using Worker = i2r_rlc::ui::COnlineLocalRegistrationWorker;
using Point = i2r_rlc::localization::Point3D_mm;
using Binding = i2r_rlc::digital_twin::OnlineSeamSceneBinding;
using GeometryRequest = i2r_rlc::digital_twin::OnlineSeamGeometryRequest;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

bool sameCoordinate(double first, double second)
{
    return std::abs(first - second) < 1.0e-3;
}

/** @brief 每帧改变面内采样，三非平行面避免仅靠一个平面的滑动歧义。 */
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

Worker::Options options(bool stability)
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
    result.stabilityProtectionEnabled = stability;
    return result;
}

/** @brief 独立定义直边与实际对象绑定；短边用来验证候选无交点必须否决。 */
bool writeFixture(const QString &directory, double halfLength)
{
    if (!QDir().mkpath(directory)) { return false; }
    const auto edge = BRepBuilderAPI_MakeEdge(
        gp_Pnt(-halfLength, 0.0, -halfLength),
        gp_Pnt(halfLength, 0.0, halfLength)).Edge();
    STEPControl_Writer writer;
    const QByteArray path = QFile::encodeName(directory + QStringLiteral("/seam.step"));
    if (writer.Transfer(edge, STEPControl_AsIs) != IFSelect_RetDone
        || writer.Write(path.constData()) != IFSelect_RetDone) { return false; }
    QFile scene(directory + QStringLiteral("/scene.xml"));
    if (!scene.open(QIODevice::WriteOnly)) { return false; }
    const QByteArray xml = "<root><scene><OBJ_SHAPE><base_object>"
        "<id>8</id><parent_id>3</parent_id></base_object><shape><model>"
        "<path>seam.step</path></model></shape></OBJ_SHAPE>"
        "<SENSOR_LSL><base_object><id>13</id></base_object><lsl_camera>"
        "<workpiece>8</workpiece><weld_seam>1</weld_seam>"
        "</lsl_camera></SENSOR_LSL></scene></root>";
    return scene.write(xml) == xml.size();
}

Worker::CloudValidationContext context(const QString &directory, int frame,
    bool validBinding, const Point &realWeld)
{
    Worker::CloudValidationContext result;
    result.archiveDirectory = directory + QStringLiteral("/evidence");
    result.hasVerifiedWorkpiecePose = true;
    result.workpiecePoseRevision = 7;
    result.independentObservation.valid = true;
    result.independentObservation.frameId = frame;
    result.independentObservation.virtualWeldPointInParent_mm = {0.0, 0.0, 0.0};
    result.independentObservation.realWeldPointInParent_mm = realWeld;
    result.candidateGeometryBinding = Binding{
        directory + QStringLiteral("/scene.xml"), 8, validBinding ? 3 : 99, 13};
    GeometryRequest geometry;
    geometry.parentLaserPlane = {0.0, 0.0, 1.0, 0.0};
    geometry.currentVirtualWeld = {0.0, 0.0, 0.0};
    geometry.realWeld = realWeld;
    geometry.referenceToleranceMm = 0.01;
    result.candidateGeometry = geometry;
    result.evidenceContext = QJsonObject{
        {QStringLiteral("synthetic_geometry_worker_fixture"), true},
        {QStringLiteral("capture_time_ms"), QString::number(10000 + frame)},
        {QStringLiteral("workpiece_pose_revision"), QStringLiteral("7")}};
    return result;
}

Worker::Result runWindow(const QString &directory, bool validBinding,
    bool stability, bool badHoldout, const Point &realWeld)
{
    Worker worker(nullptr);
    worker.configure(options(stability));
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        const double offsetZ = badHoldout && frame > 6 ? -0.25 : 0.25;
        result = worker.processClouds(1, 1, frame, surface(frame, offsetZ),
            surface(frame, 0.0), context(directory, frame, validBinding, realWeld));
    }
    if (!result.accepted) {
        std::fprintf(stderr, "worker result: %s; surfaceAccepted=%d before=%.6f "
            "after=%.6f correctionZ=%.6f evidence=%d\n",
            result.message.toUtf8().constData(), result.surfaceValidation.accepted,
            result.beforeIndependentErrorMm, result.afterIndependentErrorMm,
            result.correction.rowMajor[11], result.candidateGeometryEvidence.size());
    }
    return result;
}

/** @brief 固定曝光截点改善而旧点刚体误差恶化，证明直线也需要重新求交。 */
bool straightSeamAccepted(const QString &directory, bool stability)
{
    if (!expect(writeFixture(directory, 20.0), "write straight worker fixture")) { return false; }
    const auto result = runWindow(directory, true, stability, false, {-0.25, 0.0, 0.0});
    bool ok = expect(result.attempted && result.surfaceOnly
        && result.surfaceValidation.accepted && result.archiveSucceeded && result.accepted,
        "production surface window accepts independently reintersected straight seam");
    ok &= expect(sameCoordinate(result.correction.rowMajor[11], 0.25)
        && sameCoordinate(result.beforeIndependentErrorMm, 0.25)
        && sameCoordinate(result.afterIndependentErrorMm, 0.0),
        "worker compares fixed-plane CAD intersection instead of moving old laser point");
    ok &= expect(result.candidateGeometryEvidence.size() == 8,
        "all measured frames retain candidate geometry evidence");
    for (const auto &value : result.candidateGeometryEvidence) {
        const auto record = value.toObject();
        const auto after = record.value(QStringLiteral("after_intersection_parent_mm")).toArray();
        ok &= expect(record.value(QStringLiteral("geometry_verified")).toBool()
            && record.value(QStringLiteral("reference_verified")).toBool()
            && record.value(QStringLiteral("evaluation_kind")).toString()
                == QStringLiteral("fixed_exposure_plane_cad_reintersection")
            && after.size() == 3 && sameCoordinate(after[0].toDouble(), -0.25)
            && sameCoordinate(after[2].toDouble(), 0.0),
            "same-exposure CAD evidence stays on laser plane");
    }
    if (stability) {
        ok &= expect(result.stabilityEvaluated && result.stabilityEvaluation.accepted
            && sameCoordinate(result.stabilityEvaluation.beforeMeanMm, 0.25)
            && sameCoordinate(result.stabilityEvaluation.afterMeanMm, 0.0),
            "existing stability gain consumes actual candidate intersection");
    }
    return ok;
}

/** @brief 每个有限候选保留完整八帧；缩步按固定顺序评价而非丢掉失败证据。 */
bool validStepEvidence(const QJsonArray &records)
{
    if (records.size() < 8 || records.size() > 24 || records.size() % 8 != 0) {
        return false;
    }
    const double scales[] = {1.0, 0.5, 0.25};
    for (int index = 0; index < records.size(); ++index) {
        const auto record = records[index].toObject();
        if (!sameCoordinate(record.value(QStringLiteral("candidate_step_scale")).toDouble(),
                scales[index / 8])
            || record.value(QStringLiteral("frame_id")).toString()
                != QString::number(index % 8 + 1)) {
            return false;
        }
    }
    return true;
}

bool invalidBindingRetainsLegacyVeto(const QString &directory)
{
    if (!expect(writeFixture(directory, 20.0), "write wrong-binding fixture")) { return false; }
    const auto result = runWindow(directory, false, true, false, {-0.25, 0.0, 0.0});
    bool ok = expect(result.surfaceValidation.accepted && result.archiveSucceeded
        && !result.accepted
        && result.afterIndependentErrorMm >= result.beforeIndependentErrorMm,
        "unverified CAD binding retains independent weld veto");
    ok &= expect(validStepEvidence(result.candidateGeometryEvidence),
        "failed identity is recorded for all measured frames");
    for (const auto &value : result.candidateGeometryEvidence) {
        const auto record = value.toObject();
        ok &= expect(!record.value(QStringLiteral("reference_verified")).toBool()
            && record.value(QStringLiteral("evaluation_kind")).toString()
                == QStringLiteral("legacy_point_veto_fallback"),
            "identity failure explicitly retains old-point gate");
    }
    return ok;
}

bool missingCandidateCannotFallback(const QString &directory)
{
    if (!expect(writeFixture(directory, 0.0001), "write short-seam fixture")) { return false; }
    // 刚体移动旧点会完美命中此实测点，但有限 CAD 边已离开曝光平面。
    const auto result = runWindow(directory, true, false, false, {0.0, 0.0, 0.25});
    bool ok = expect(result.surfaceValidation.accepted && result.archiveSucceeded
        && !result.baseSafetyAccepted && !result.accepted,
        "verified current seam with missing candidate intersection rejects");
    ok &= expect(validStepEvidence(result.candidateGeometryEvidence),
        "missing intersection recorded for all measured frames");
    for (const auto &value : result.candidateGeometryEvidence) {
        const auto record = value.toObject();
        ok &= expect(record.value(QStringLiteral("reference_verified")).toBool()
            && !record.value(QStringLiteral("geometry_verified")).toBool()
            && record.value(QStringLiteral("evaluation_kind")).toString()
                == QStringLiteral("candidate_geometry_rejected")
            && !record.value(QStringLiteral("accepted")).toBool()
            && record.value(QStringLiteral("before_intersection_parent_mm")).isNull()
            && record.value(QStringLiteral("after_intersection_parent_mm")).isNull()
            && record.value(QStringLiteral("fallback_before_point_parent_mm")).isArray()
            && record.value(QStringLiteral("fallback_after_point_parent_mm")).isArray()
            && record.value(QStringLiteral("error_point_source")).toString()
                == QStringLiteral("legacy_transform_diagnostic_only_rejected"),
            "missing candidate never uses favorable rigid-point fallback");
    }
    return ok;
}

/** @brief 有限直边允许合法缩步：全步离开激光面，半步求交仍须通过全部门禁。 */
bool fullRejectedThenHalfAccepted(const QString &directory)
{
    if (!expect(writeFixture(directory, 0.2), "write legal half-step fixture")) { return false; }
    Worker worker(nullptr);
    auto configured = options(false);
    // 降低本例采样量，保留三面与八帧验证，在生产 2500 ms 预算内隔离缩步逻辑。
    configured.maximumPointsPerCloud = 128;
    worker.configure(configured);
    Worker::Result result;
    for (int frame = 1; frame <= 8; ++frame) {
        result = worker.processClouds(1, 1, frame, surface(frame, 0.25), surface(frame, 0.0),
            context(directory, frame, true, {-0.125, 0.0, 0.0}));
    }
    bool ok = expect(result.attempted && result.surfaceOnly && result.accepted
        && result.surfaceValidation.accepted && result.baseSafetyAccepted && result.archiveSucceeded
        && result.solveElapsedMs <= 2500
        && sameCoordinate(result.candidateStepScale, 0.5)
        && sameCoordinate(result.correction.rowMajor[11], 0.125)
        && sameCoordinate(result.beforeIndependentErrorMm, 0.125)
        && sameCoordinate(result.afterIndependentErrorMm, 0.0),
        "missing full-step intersection selects legal half-step after all production gates");
    ok &= expect(result.candidateGeometryEvidence.size() == 16
        && validStepEvidence(result.candidateGeometryEvidence),
        "half-step keeps rejected full-step and accepted half-step eight-frame evidence");
    for (int index = 0; index < result.candidateGeometryEvidence.size(); ++index) {
        const auto record = result.candidateGeometryEvidence[index].toObject();
        const bool fullStep = index < 8;
        const auto after = record.value(QStringLiteral("after_intersection_parent_mm"));
        ok &= expect(record.value(QStringLiteral("reference_verified")).toBool()
            && record.value(QStringLiteral("geometry_verified")).toBool() == !fullStep
            && record.value(QStringLiteral("accepted")).toBool() == !fullStep
            && (fullStep ? after.isNull()
                : (after.isArray() && after.toArray().size() == 3
                    && sameCoordinate(after.toArray()[0].toDouble(), -0.125)
                    && sameCoordinate(after.toArray()[2].toDouble(), 0.0))),
            "only reintersected half-step has a valid candidate intersection");
    }
    ok &= expect(result.candidateStepEvidence.size() == 2,
        "half-step acceptance keeps both candidate-level gate results");
    if (result.candidateStepEvidence.size() == 2) {
        const auto full = result.candidateStepEvidence[0].toObject();
        const auto half = result.candidateStepEvidence[1].toObject();
        ok &= expect(sameCoordinate(full.value(QStringLiteral("step_scale")).toDouble(), 1.0)
            && full.value(QStringLiteral("surface_accepted")).toBool()
            && full.value(QStringLiteral("within_time_budget")).toBool()
            && full.value(QStringLiteral("elapsed_ms")).toDouble() <= 2500.0
            && !full.value(QStringLiteral("base_safety_accepted")).toBool()
            && !full.value(QStringLiteral("accepted")).toBool()
            && sameCoordinate(half.value(QStringLiteral("step_scale")).toDouble(), 0.5)
            && half.value(QStringLiteral("surface_accepted")).toBool()
            && half.value(QStringLiteral("base_safety_accepted")).toBool()
            && half.value(QStringLiteral("within_time_budget")).toBool()
            && half.value(QStringLiteral("elapsed_ms")).toDouble() <= 2500.0
            && half.value(QStringLiteral("accepted")).toBool(),
            "legal half-step satisfies surface safety and shared time budget");
    }
    if (!ok) {
        std::fprintf(stderr, "half-step result: %s; accepted=%d step=%.6f evidence=%d\n",
            result.message.toUtf8().constData(), result.accepted, result.candidateStepScale,
            result.candidateGeometryEvidence.size());
    }
    return ok;
}

bool seamCannotOverrideFailedHoldout(const QString &directory)
{
    if (!expect(writeFixture(directory, 20.0), "write failed-holdout fixture")) { return false; }
    const auto result = runWindow(directory, true, true, true, {-0.25, 0.0, 0.0});
    return expect(result.attempted && result.surfaceOnly && result.archiveSucceeded
        && !result.surfaceValidation.accepted && !result.baseSafetyAccepted && !result.accepted,
        "favorable seam cannot override rejected heldout mother-material surface");
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    if (!expect(directory.isValid(), "retained fixture directory created")) { return 1; }
    const QString base = directory.path();
    bool ok = straightSeamAccepted(base + QStringLiteral("/straight"), false);
    ok = straightSeamAccepted(base + QStringLiteral("/stability"), true) && ok;
    ok = invalidBindingRetainsLegacyVeto(base + QStringLiteral("/identity")) && ok;
    ok = missingCandidateCannotFallback(base + QStringLiteral("/missing")) && ok;
    ok = fullRejectedThenHalfAccepted(base + QStringLiteral("/halfstep")) && ok;
    ok = seamCannotOverrideFailedHoldout(base + QStringLiteral("/holdout")) && ok;
    std::printf("worker geometry fixtures retained: %s\n", base.toUtf8().constData());
    if (ok) { std::puts("PASS: production worker CAD reintersection and retained safety gates"); }
    return ok ? 0 : 1;
}