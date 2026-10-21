/**
 * @file test_local_registration_writeback_safety.cpp
 * @brief 用受控内存 transport 验证累计预算、独立验收与受保护恢复；不访问设备。
 */
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>

#include <QCoreApplication>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>

#include "RuntimeWorkerAdapters.h"

namespace {
namespace ui = i2r_rlc::ui;
namespace twin = i2r_rlc::digital_twin;
namespace loc = i2r_rlc::localization;

bool expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
    }
    return condition;
}

/** @brief 所有调用均在测试线程；回调在父级查询返回前模拟跨线程代次撤销。 */
struct MemoryScene {
    int writes = 0;
    bool failReadback = false;
    int failReadbackFromWrite = 1;
    QJsonArray pose{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::function<void()> beforeParentRead;
};

/** @brief 只解析本测试用到的位姿协议，其他初始化查询返回固定虚拟参考。 */
class CMemoryPoseTransport final : public twin::IRobotSimTransport {
public:
    explicit CMemoryPoseTransport(const std::shared_ptr<MemoryScene> &scene)
        : m_scene(scene)
    {
    }

    twin::RobotSimTransportResult connect(const twin::RobotSimEndpointConfig &) override
    {
        m_connected = true;
        return {true, {}, QStringLiteral("memory connected"), 0};
    }

    void disconnect() override { m_connected = false; }
    bool isConnected() const override { return m_connected; }

    twin::RobotSimTransportResult send(const QJsonDocument &document, bool expectResponse) override
    {
        const auto operation = document.array();
        const QString type = operation.at(1).toString();
        const auto parameters = operation.at(2).toArray();
        if (type == QStringLiteral("GET_ABS_POSE")) {
            const int objectId = parameters.at(0).toInt();
            if (objectId == 200 && m_scene->beforeParentRead) {
                m_scene->beforeParentRead();
            }
            if (objectId == 300 && m_scene->failReadback
                && m_scene->writes >= m_scene->failReadbackFromWrite) {
                return {false, {}, QStringLiteral("memory readback failed"), 0};
            }
            const auto pose = objectId == 200
                ? QJsonArray{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} : m_scene->pose;
            return {true, QString::fromUtf8(QJsonDocument(pose).toJson(QJsonDocument::Compact)),
                QStringLiteral("memory pose"), 0};
        }
        if (type == QStringLiteral("SET_POSE")) {
            ++m_scene->writes;
            m_scene->pose = parameters.at(1).toArray();
            return {true, {}, QStringLiteral("memory set"), 0};
        }
        return {true, expectResponse
            ? QStringLiteral("[[[[[1,2,3],[100,120]],[[4,5,6],[220,120]]]],[[160,120]]]")
            : QString(), QStringLiteral("memory reference"), 0};
    }

private:
    std::shared_ptr<MemoryScene> m_scene;
    bool m_connected = false;
};

/** @brief worker 与内存 transport 同线程，无事件循环等待或真实 socket。 */
struct SafetyFixture {
    std::shared_ptr<MemoryScene> scene = std::make_shared<MemoryScene>();
    ui::CGlobalLocatePriorWorkerAdapter worker{std::make_unique<CMemoryPoseTransport>(scene)};
    twin::RobotSimEndpointConfig endpoint;
    twin::RobotSimSceneConfig config;
    i2r_rlc::robot_comm::RobotState state;

    SafetyFixture()
    {
        endpoint.port = 40000;
        endpoint.referenceObjectId = 200;
        config.robotId = 1001;
        config.positionerId = 1002;
        config.sensorId = 1013;
        config.workpieceId = 300;
        config.workpieceParentId = 200;
        state.curx_r = 1389425;
        state.cury_r = -4882;
        state.curz_r = 688864;
        state.curRx_r = -1791754;
        state.curRy_r = -551313;
        state.curRz_r = -2478;
        worker.connectRobotSim(endpoint, config, state);
        worker.configureLocalRegistrationSafety({});
        worker.setLocalRegistrationEpoch(7);
        refreshDeadline();
        worker.setLocalRegistrationKeypointsInParent({{20.0, 0.0, 0.0}, {120.0, 0.0, 0.0}});
    }

    void refreshDeadline()
    {
        worker.setLocalRegistrationStateDeadline(QDateTime::currentMSecsSinceEpoch() + 10000);
    }

    twin::DigitalTwinCommandResult applyTranslation(double distance)
    {
        loc::RigidTransform correction;
        correction.rowMajor[3] = distance;
        return worker.applyLocalRegistrationCorrection(7, correction);
    }
};

bool testPendingBlocksAndGeometryPromotion()
{
    SafetyFixture fixture;
    bool ok = expect(fixture.applyTranslation(5.0).succeeded, "first safe write must succeed");
    auto state = fixture.worker.localRegistrationSafetySnapshot();
    ok &= expect(state.hasPendingPose && state.pendingPoseConfirmed
        && std::abs(state.verifiedParentFromWorkpiece.rowMajor[3] - 20.0) < 1e-6,
        "readback must preserve previous verified pose and retain pending geometry validation");
    ok &= expect(!fixture.applyTranslation(1.0).succeeded && fixture.scene->writes == 1,
        "pending geometry validation must block consecutive writes");
    ok &= expect(fixture.worker.markLocalRegistrationVerified(7).succeeded,
        "explicit independent verification may promote pending pose");
    state = fixture.worker.localRegistrationSafetySnapshot();
    ok &= expect(!state.hasPendingPose
        && std::abs(state.verifiedParentFromWorkpiece.rowMajor[3] - 25.0) < 1e-6,
        "promotion must preserve confirmed pose as next rollback target");
    return ok;
}

bool testCumulativeAndRotatingKeypointBudgets()
{
    SafetyFixture fixture;
    bool ok = expect(fixture.applyTranslation(16.0).succeeded, "first cumulative step allowed");
    ok &= expect(fixture.worker.markLocalRegistrationVerified(7).succeeded, "first step verified");
    ok &= expect(!fixture.applyTranslation(16.0).succeeded && fixture.scene->writes == 1,
        "two individually allowed increments must not exceed session translation budget");
    SafetyFixture rotation;
    rotation.worker.setLocalRegistrationKeypointsInParent({{1020.0, 0.0, 0.0}});
    loc::RigidTransform correction;
    const double angle = 0.05;
    correction.rowMajor[0] = std::cos(angle);
    correction.rowMajor[1] = -std::sin(angle);
    correction.rowMajor[4] = std::sin(angle);
    correction.rowMajor[5] = std::cos(angle);
    ok &= expect(!rotation.worker.applyLocalRegistrationCorrection(7, correction).succeeded
        && rotation.scene->writes == 0,
        "small rotation must reject large material keypoint displacement");
    SafetyFixture cumulativeRotation;
    const double rotationStep = 6.0 * 3.14159265358979323846 / 180.0;
    correction.rowMajor[0] = std::cos(rotationStep);
    correction.rowMajor[1] = -std::sin(rotationStep);
    correction.rowMajor[4] = std::sin(rotationStep);
    correction.rowMajor[5] = std::cos(rotationStep);
    correction.rowMajor[3] = 20.0 * (1.0 - std::cos(rotationStep));
    correction.rowMajor[7] = -20.0 * std::sin(rotationStep);
    ok &= expect(cumulativeRotation.worker.applyLocalRegistrationCorrection(7, correction).succeeded,
        "first rotation around object origin must fit rotation and keypoint budgets");
    ok &= expect(cumulativeRotation.worker.markLocalRegistrationVerified(7).succeeded,
        "first rotation may be explicitly verified");
    ok &= expect(!cumulativeRotation.worker.applyLocalRegistrationCorrection(7, correction).succeeded
        && cumulativeRotation.scene->writes == 1,
        "two small rotations must still obey initial-session rotation budget");
    return ok;
}

bool testRestoreLastVerifiedPose()
{
    SafetyFixture fixture;
    bool ok = expect(fixture.applyTranslation(5.0).succeeded, "first candidate written");
    ok &= expect(fixture.worker.markLocalRegistrationVerified(7).succeeded, "first candidate verified");
    ok &= expect(fixture.applyTranslation(3.0).succeeded, "second candidate written");
    ok &= expect(fixture.worker.rollbackLocalRegistrationCorrection(7).succeeded,
        "failed new geometry may explicitly restore prior verified pose");
    ok &= expect(std::abs(fixture.scene->pose.at(0).toDouble() - 25.0) < 1e-6
        && fixture.scene->writes == 3, "rollback must restore last verified pose rather than session origin");
    ok &= expect(!fixture.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && fixture.scene->writes == 3, "successful rollback cannot be repeated");
    return ok;
}

bool testExternalChangesAndCancellation()
{
    SafetyFixture external;
    bool ok = expect(external.applyTranslation(5.0).succeeded, "external-change fixture written");
    external.scene->pose[0] = 99.0;
    ok &= expect(!external.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && external.scene->writes == 1, "external object edit must block rollback overwrite");
    ok &= expect(!external.worker.markLocalRegistrationVerified(7).succeeded,
        "external object edit must also block verified promotion");
    SafetyFixture stopped;
    ok &= expect(stopped.applyTranslation(5.0).succeeded, "stop fixture written");
    stopped.worker.setLocalRegistrationEpoch(0);
    ok &= expect(stopped.scene->writes == 1
        && !stopped.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && stopped.scene->writes == 1, "stop only invalidates token and never sends restore command");
    SafetyFixture duringRead;
    ok &= expect(duringRead.applyTranslation(5.0).succeeded, "read-cancel fixture written");
    duringRead.scene->beforeParentRead = [&duringRead]() { duringRead.worker.setLocalRegistrationEpoch(0); };
    ok &= expect(!duringRead.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && duringRead.scene->writes == 1, "cancellation during parent GET must block rollback SET");
    SafetyFixture expired;
    ok &= expect(expired.applyTranslation(5.0).succeeded, "expired fixture written");
    expired.worker.setLocalRegistrationStateDeadline(QDateTime::currentMSecsSinceEpoch() - 1);
    ok &= expect(!expired.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && expired.scene->writes == 1, "expired robot state must block rollback SET");
    return ok;
}

bool testSceneIdentityAndUnknownWrite()
{
    SafetyFixture changed;
    bool ok = expect(changed.applyTranslation(5.0).succeeded, "scene-change fixture written");
    changed.config.workpieceId = 301;
    changed.worker.configureRobotSim(changed.endpoint, changed.config);
    ok &= expect(!changed.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && changed.scene->writes == 1, "scene/object change must invalidate rollback transaction");
    SafetyFixture unknown;
    unknown.scene->failReadback = true;
    ok &= expect(!unknown.applyTranslation(5.0).succeeded,
        "readback failure must not claim confirmed command");
    const auto pending = unknown.worker.localRegistrationSafetySnapshot();
    ok &= expect(pending.hasPendingPose && !pending.pendingPoseConfirmed
        && unknown.worker.localRegistrationWriteAttempted(),
        "unknown write must retain target, previous verified pose, and attempted marker");
    ok &= expect(!unknown.worker.markLocalRegistrationVerified(7).succeeded,
        "unknown command must never be promoted as geometrically verified");
    unknown.scene->failReadback = false;
    ok &= expect(unknown.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && unknown.scene->writes == 2,
        "unknown write may restore only after current pose matches recorded target");
    SafetyFixture failedRestore;
    ok &= expect(failedRestore.applyTranslation(5.0).succeeded, "failed-restore fixture written");
    failedRestore.scene->failReadback = true;
    failedRestore.scene->failReadbackFromWrite = 2;
    ok &= expect(!failedRestore.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && failedRestore.scene->writes == 2,
        "restore SET followed by unknown readback must preserve attempted outcome");
    failedRestore.scene->failReadback = false;
    ok &= expect(!failedRestore.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && failedRestore.scene->writes == 2,
        "unknown restore result must forbid automatic retry");
    SafetyFixture missingPoints;
    missingPoints.worker.setLocalRegistrationKeypointsInParent({});
    ok &= expect(!missingPoints.applyTranslation(1.0).succeeded && missingPoints.scene->writes == 0,
        "missing independent keypoints must reject protected writes");
    return ok;
}

/** @brief 同对象、同父级、同位姿且连接有效，仅场景修订变化也必须撤销旧事务。 */
bool testSceneRevisionAloneRejectsPendingTransaction()
{
    SafetyFixture fixture;
    bool ok = expect(fixture.applyTranslation(5.0).succeeded,
        "revision-only fixture must hold a confirmed pending pose");
    const auto original = fixture.worker.localRegistrationSafetySnapshot();
    const auto pendingPose = fixture.scene->pose;
    bool connected = false;
    const auto connection = QObject::connect(&fixture.worker,
        &ui::CGlobalLocatePriorWorkerAdapter::robotSimConnectionStateChanged,
        [&connected](bool isConnected, const QString &) { connected = isConnected; });
    // 不 configure safety，不改 endpoint、任何 ID 或内存位姿，仅重新建立真实场景连接。
    fixture.worker.connectRobotSim(fixture.endpoint, fixture.config, fixture.state);
    QObject::disconnect(connection);
    fixture.worker.setLocalRegistrationEpoch(7);
    fixture.refreshDeadline();
    const auto retained = fixture.worker.localRegistrationSafetySnapshot();
    ok &= expect(connected && fixture.scene->pose == pendingPose && fixture.scene->writes == 1,
        "reconnect must be live while preserving exact pending pose and write count");
    ok &= expect(retained.hasPendingPose && retained.pendingPoseConfirmed
        && retained.pendingEpoch == 7 && retained.sceneRevision == original.sceneRevision
        && retained.workpieceId == fixture.config.workpieceId
        && retained.parentId == fixture.config.workpieceParentId,
        "old pending snapshot and IDs must survive reconnect unchanged");
    ok &= expect(!fixture.worker.markLocalRegistrationVerified(7).succeeded,
        "scene revision alone must prevent old pending pose promotion");
    ok &= expect(!fixture.worker.rollbackLocalRegistrationCorrection(7).succeeded
        && fixture.scene->writes == 1 && fixture.scene->pose == pendingPose,
        "scene revision alone must prevent rollback despite live connection and identical IDs/pose");
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    bool ok = testPendingBlocksAndGeometryPromotion();
    ok &= testCumulativeAndRotatingKeypointBudgets();
    ok &= testRestoreLastVerifiedPose();
    ok &= testExternalChangesAndCancellation();
    ok &= testSceneIdentityAndUnknownWrite();
    ok &= testSceneRevisionAloneRejectsPendingTransaction();
    if (ok) {
        std::puts("PASS: local registration writeback safety tests");
    }
    return ok ? 0 : 1;
}
