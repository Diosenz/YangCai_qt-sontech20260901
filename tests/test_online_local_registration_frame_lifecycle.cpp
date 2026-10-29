/**
 * @file test_online_local_registration_frame_lifecycle.cpp
 * @brief 通过真实协调器写回、路由和结案路径验证慢帧与真实失联的事务边界。
 * @details 定位器候选采用固定已验收结果，RobotSim 使用内存协议；不启动设备或生产线程。
 */
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QThread>

#include "RuntimeCoordinator.h"

namespace {
namespace twin = i2r_rlc::digital_twin;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

struct MemoryScene {
    QJsonArray pose{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int writes = 0;
    int poseReads = 0;
    bool failWriteReadback = false;
};

/** @brief 仅实现测试所需的场景协议；所有收发均在当前测试线程完成。 */
class CMemoryPoseTransport final : public twin::IRobotSimTransport {
public:
    explicit CMemoryPoseTransport(std::shared_ptr<MemoryScene> scene)
        : m_scene(std::move(scene))
    {
    }

    twin::RobotSimTransportResult connect(const twin::RobotSimEndpointConfig &) override
    {
        m_connected = true;
        return {true, {}, QStringLiteral("memory connected"), 0};
    }

    void disconnect() override { m_connected = false; }
    bool isConnected() const override { return m_connected; }

    twin::RobotSimTransportResult send(const QJsonDocument &document, bool response) override
    {
        const auto operation = document.array();
        const auto type = operation.at(1).toString();
        const auto parameters = operation.at(2).toArray();
        if (type == QStringLiteral("GET_ABS_POSE")) {
            ++m_scene->poseReads;
            const int object = parameters.at(0).toInt();
            if (object == 300 && m_scene->failWriteReadback && m_scene->writes > 0) {
                return {false, {}, QStringLiteral("unknown write readback"), 0};
            }
            const auto pose = object == 200
                ? QJsonArray{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} : m_scene->pose;
            return {true, QString::fromUtf8(QJsonDocument(pose).toJson(QJsonDocument::Compact)),
                QStringLiteral("memory pose"), 0};
        }
        if (type == QStringLiteral("SET_POSE")) {
            ++m_scene->writes;
            m_scene->pose = parameters.at(1).toArray();
            return {true, {}, QStringLiteral("memory set"), 0};
        }
        return {true, response
            ? QStringLiteral("[[[[[1,2,3],[100,120]],[[4,5,6],[220,120]]]],[[160,120]]]")
            : QString(), QStringLiteral("memory reference"), 0};
    }

private:
    std::shared_ptr<MemoryScene> m_scene;
    bool m_connected = false;
};

QJsonObject readObject(const QString &path)
{
    QFile file(path);
    return file.open(QIODevice::ReadOnly)
        ? QJsonDocument::fromJson(file.readAll()).object() : QJsonObject{};
}
} // namespace

namespace i2r_rlc::ui {
/** @brief 独立测试目标复用既有友元名，不增加生产公共测试接口。 */
struct CRuntimeCoordinatorHistoryTestAccess {
    struct Fixture {
        CRuntimeCoordinator runtime;
        std::shared_ptr<MemoryScene> scene = std::make_shared<MemoryScene>();
        CGlobalLocatePriorWorkerAdapter twinWorker{std::make_unique<CMemoryPoseTransport>(scene)};
        COnlineLocalRegistrationWorker surfaceWorker{nullptr};
        QTemporaryDir directory;
        int candidateCount = 0;
        qint64 frameId = 100;
        QString candidatePath;

        Fixture()
        {
            directory.setAutoRemove(false);
            digital_twin::RobotSimEndpointConfig endpoint;
            endpoint.port = 40000;
            endpoint.referenceObjectId = 200;
            digital_twin::RobotSimSceneConfig config;
            config.robotId = 1001;
            config.positionerId = 1002;
            config.sensorId = 1013;
            config.workpieceId = 300;
            config.workpieceParentId = 200;
            robot_comm::RobotState state;
            state.sStart = 1;
            state.sMode = 2;
            state.traflag = state.corflag = state.detflag = 1;
            twinWorker.connectRobotSim(endpoint, config, state);
            twinWorker.configureLocalRegistrationSafety({});
            twinWorker.setLocalRegistrationEpoch(7);
            runtime.m_running = true;
            runtime.m_localRegistrationClock.start();
            runtime.m_localRegistrationOptions.enabled = true;
            runtime.m_localRegistrationOptions.sampleIntervalMs = 0;
            runtime.m_localRegistrationOptions.minimumMatchIntervalMs = 2000;
            runtime.m_trackingPurpose = RuntimeTrackingPurpose::OnlineCorrection;
            runtime.m_correctionSession.active = true;
            runtime.m_correctionSession.purpose = CorrectionPurpose::OnlineCorrection;
            runtime.m_correctionSession.onlineControlWindowActive = true;
            runtime.m_correctionSession.sessionId = 44;
            runtime.m_localRegistrationGeneration = 7;
            runtime.m_robotSimWorker = &twinWorker;
            runtime.m_robotSimConnected = true;
            runtime.m_globalLocateRobotSimScene = config;
            runtime.m_latestRobotState = state;
            runtime.m_hasLatestRobotState = true;
            runtime.m_localRegistrationRobotStateFresh = true;
            runtime.m_localRegistrationArchiveDirectory = directory.path();
            vision::PositionerCalibration calibration;
            calibration.valid = true;
            runtime.m_onlinePositionerCalibration = calibration;
            surfaceWorker.configure(runtime.m_localRegistrationOptions);
            refreshCommunication();
        }

        ~Fixture()
        {
            runtime.m_running = false;
            runtime.m_robotSimWorker = nullptr;
            runtime.m_localRegistrationWorker = nullptr;
            runtime.m_localVerification.reset();
        }

        void refreshCommunication()
        {
            runtime.m_latestRobotStateReceivedMs = QDateTime::currentMSecsSinceEpoch();
        }

        bool waitFor(const std::function<bool()> &condition, int maximumMs)
        {
            QElapsedTimer timer;
            timer.start();
            while (!condition() && timer.elapsed() < maximumMs) {
                refreshCommunication();
                QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
                QThread::msleep(1);
            }
            return condition();
        }

        /** @brief 固定候选替代PCL求解；真实执行协调器写回、GET/SET和后验创建。 */
        bool beginCandidate(bool surface)
        {
            refreshCommunication();
            COnlineLocalRegistrationWorker::Result result;
            result.sessionId = runtime.m_correctionSession.sessionId;
            result.generation = runtime.m_localRegistrationGeneration;
            result.frameId = ++frameId;
            result.workpiecePoseRevision = runtime.m_localWorkpiecePoseRevision;
            result.attempted = result.accepted = result.archiveSucceeded = true;
            result.correction.rowMajor[3] = 1.0;
            result.independentObservation.valid = true;
            result.independentObservation.frameId = result.frameId;
            result.independentObservation.virtualWeldPointInParent_mm = {20.0, 0.0, 0.0};
            result.independentObservation.realWeldPointInParent_mm = {21.0, 0.0, 0.0};
            candidatePath = QDir(directory.path()).filePath(
                QStringLiteral("candidate_%1.json").arg(++candidateCount));
            result.archivePath = candidatePath;
            result.surfaceOnly = surface;
            if (surface) {
                result.surfaceValidation.accepted = true;
                auto &metrics = result.surfaceValidation.holdoutAfter;
                metrics.valid = true;
                metrics.realToVirtualMeanMm = metrics.virtualToRealMeanMm = 0.2;
                metrics.realCoverageRatio = metrics.virtualCoverageRatio = 1.0;
                result.surfaceVerificationPreflight.valid = true;
                result.safetyKeypointsInParent_mm = {{20.0, 0.0, 0.0}};
                for (int i = 0; i < 24; ++i) {
                    result.surfaceVerificationReference_mm.push_back({static_cast<double>(i), 0.0, 0.0});
                }
            }
            runtime.m_localRegistrationInputMs = runtime.m_localRegistrationClock.elapsed();
            runtime.m_localRegistrationBusy = true;
            const auto request = ++runtime.m_localRegistrationRequestId;
            runtime.handleOnlineLocalRegistrationResult(request, result);
            return waitFor([this]() { return !runtime.m_localRegistrationWritebackBusy; }, 1000)
                && runtime.m_localVerification.has_value();
        }

        /** @brief 经生产route依次执行后验和采样；慢帧仍保留正常视觉路由身份。 */
        void route(bool slow, bool latestStale = false, bool oldEpoch = false)
        {
            if (slow && runtime.m_localVerification.has_value()) {
                const auto completed = runtime.m_localVerification->completedAtMs;
                // 曝光时状态只早70ms；结果晚580ms返回，期间仍持续收到新通信。
                waitFor([completed]() {
                    return QDateTime::currentMSecsSinceEpoch() - completed > 700;
                }, 1000);
            }
            refreshCommunication();
            const auto now = QDateTime::currentMSecsSinceEpoch();
            CRuntimeCoordinator::FrozenVisionFrameContext context;
            context.frame.frameId = ++frameId;
            context.frame.timestamp = QDateTime::fromMSecsSinceEpoch(now - (slow ? 580 : 0));
            context.frame.image = QImage(64, 64, QImage::Format_Grayscale8);
            context.frame.image.fill(15);
            context.robotState = runtime.m_latestRobotState;
            context.hasRobotState = true;
            context.robotStateReceivedMs = now - (slow ? 650 : 0);
            context.purpose = RuntimeTrackingPurpose::OnlineCorrection;
            context.correctionSessionId = runtime.m_correctionSession.sessionId;
            context.onlineControlWindowActive = true;
            context.modelGeneration = runtime.m_localRegistrationGeneration - (oldEpoch ? 1 : 0);
            context.workpiecePoseRevision = runtime.m_localWorkpiecePoseRevision;
            digital_twin::DigitalTwinVisionLaserObservation observation;
            observation.valid = observation.hasWeldPoint_mm = true;
            observation.frameId = context.frame.frameId;
            observation.referenceObjectId = 200;
            observation.weldPoint_mm = {21.0, 0.0, 0.0};
            for (int i = 0; i < 24; ++i) {
                observation.laserLinePoints_mm.push_back({static_cast<double>(i), 0.0, 0.0});
            }
            // 表面采样要求空间线和同帧像素线都有效；夹具必须满足生产合同。
            observation.laserLineImagePoints_px = {{5.0, 10.0}, {28.0, 10.0}};
            context.synchronizedObservation = observation;
            RuntimeTrackingResult result;
            result.valid = result.triggerAccepted = true;
            result.frameId = context.frame.frameId;
            result.sample.point = result.independentWeldPointInParent_mm = {21.0, 0.0, 0.0, true};
            result.pointSource = vision::OnlineSeamPointSource::Geometry;
            result.correlation = runtime.makeVisionCorrelation(context);
            runtime.m_currentVisionFrame = context;
            runtime.m_visionFrameInFlight = true;
            if (latestStale) { runtime.m_latestRobotStateReceivedMs = now - 650; }
            runtime.routeTrackingResult(result);
        }

        bool pending() const
        {
            return runtime.m_localVerification.has_value() && !runtime.m_localRegistrationFaulted;
        }

        bool completeWithFreshFrames()
        {
            if (!runtime.m_localVerification.has_value()) { return false; }
            const auto completed = runtime.m_localVerification->completedAtMs;
            waitFor([completed]() { return QDateTime::currentMSecsSinceEpoch() > completed; }, 100);
            for (int i = 0; i < 3; ++i) { route(false); }
            return waitFor([this]() { return !runtime.m_localVerification.has_value(); }, 1000)
                && !runtime.m_localRegistrationFaulted;
        }

        bool nextWindowCanWrite()
        {
            const auto previousWrites = scene->writes;
            const auto attempted = runtime.m_lastLocalRegistrationAttemptMs;
            waitFor([this, attempted]() {
                return runtime.m_localRegistrationClock.elapsed() - attempted
                    >= runtime.m_localRegistrationOptions.minimumMatchIntervalMs;
            }, 2500);
            runtime.m_localRegistrationWorker = &surfaceWorker;
            route(false);
            const bool samplingStarted = runtime.m_localRegistrationBusy;
            waitFor([this]() { return !runtime.m_localRegistrationBusy; }, 1000);
            runtime.m_localRegistrationWorker = nullptr;
            const bool written = beginCandidate(false);
            return samplingStarted && written && scene->writes == previousWrites + 1;
        }
    };

    static bool testOrdinarySlowFrame()
    {
        Fixture fixture;
        if (!expect(fixture.beginCandidate(false), "ordinary candidate enters a real confirmed pending write")) {
            return false;
        }
        fixture.waitFor([&fixture]() {
            return QDateTime::currentMSecsSinceEpoch() > fixture.runtime.m_localVerification->completedAtMs;
        }, 100);
        fixture.route(false);
        fixture.route(false);
        const auto completedAt = fixture.runtime.m_localVerification->completedAtMs;
        fixture.route(true);
        bool ok = expect(fixture.pending(), "one slow frame with live communication must not cancel the ordinary transaction");
        ok &= expect(!QFile::exists(fixture.candidatePath + QStringLiteral(".verified.json")),
            "slow frame cannot supply the third independent good observation");
        if (!fixture.pending()) { return false; }
        ok &= expect(fixture.runtime.m_localVerification->completedAtMs == completedAt,
            "skipping a slow frame cannot restart the original verification timeout");
        fixture.route(false);
        fixture.route(false);
        ok &= expect(fixture.pending()
                && fixture.runtime.m_localVerification->check->decision()
                    == COnlineLocalRegistrationVerification::Decision::Waiting,
            "slow frame breaks the prior two-good-frame sequence without counting as worsening");
        fixture.route(false);
        ok &= expect(fixture.waitFor([&fixture]() { return !fixture.runtime.m_localVerification.has_value(); }, 1000),
            "three new fresh frames finish ordinary verification through the real callback");
        const auto record = readObject(fixture.candidatePath + QStringLiteral(".verified.json"));
        ok &= expect(record.value(QStringLiteral("succeeded")).toBool()
                && !fixture.twinWorker.localRegistrationSafetySnapshot().hasPendingPose,
            "verified archive and worker transaction both confirm completion");
        ok &= expect(fixture.nextWindowCanWrite(),
            "next fresh sampling window can reach another SET after ordinary verification");
        return ok;
    }

    static bool testSurfaceTimeoutRetention()
    {
        Fixture fixture;
        if (!expect(fixture.beginCandidate(false) && fixture.completeWithFreshFrames(),
                "surface test establishes its geometric baseline through an ordinary write and verification")
            || !expect(fixture.beginCandidate(true), "surface candidate enters confirmed pending write")) {
            return false;
        }
        const auto writes = fixture.scene->writes;
        const auto reads = fixture.scene->poseReads;
        const auto completedAt = fixture.runtime.m_localVerification->completedAtMs;
        fixture.route(true);
        bool ok = expect(fixture.pending(), "slow surface frame must preserve pending and allow bounded retention");
        if (!fixture.pending()) { return false; }
        ok &= expect(fixture.runtime.m_localVerification->completedAtMs == completedAt,
            "surface slow-frame skip retains the original five-second deadline");
        ok &= expect(fixture.waitFor([&fixture]() { return !fixture.runtime.m_localVerification.has_value(); }, 6500),
            "actual five-second timer completes surface retention without another valid surface frame");
        const auto retained = readObject(fixture.candidatePath + QStringLiteral(".retained.json"));
        ok &= expect(retained.value(QStringLiteral("succeeded")).toBool()
                && retained.value(QStringLiteral("retained_without_geometric_verification")).toBool()
                && !retained.value(QStringLiteral("rollback_attempted")).toBool()
                && fixture.scene->writes == writes && fixture.scene->poseReads > reads,
            "timeout retention performs GET and archive, without SET rollback or geometry promotion");
        ok &= expect(!fixture.runtime.m_localRegistrationFaulted && fixture.nextWindowCanWrite(),
            "retained transaction permits a new-exposure sampling window and a later legal SET");
        return ok;
    }

    /** @brief 开关只影响追加更新；通过内存场景实际 GET/SET 核对已写位姿持续保留。 */
    static bool testToggleRetainsWrittenScene()
    {
        Fixture fixture;
        const auto fusion = fixture.runtime.onlineFusionOptions();
        fixture.runtime.updateOnlineProcessingOptions(false, fusion);
        QCoreApplication::processEvents();
        bool ok = expect(fixture.scene->writes == 0
                && std::abs(fixture.scene->pose.at(0).toDouble() - 20.0) < 1e-6,
            "disabling ICP before a write preserves the initial workpiece scene");
        fixture.runtime.updateOnlineProcessingOptions(true, fusion);
        if (!expect(fixture.beginCandidate(false) && fixture.completeWithFreshFrames(),
                "enabled ICP writes and verifies a correction through the real coordinator")) {
            return false;
        }
        fixture.runtime.updateOnlineProcessingOptions(false, fusion);
        QCoreApplication::processEvents();
        ok &= expect(fixture.scene->writes == 1
                && std::abs(fixture.scene->pose.at(0).toDouble() - 21.0) < 1e-6,
            "disabling ICP retains the updated workpiece instead of silently restoring its initial pose");
        fixture.runtime.updateOnlineProcessingOptions(true, fusion);
        ok &= expect(fixture.beginCandidate(false) && fixture.scene->writes == 2
                && std::abs(fixture.scene->pose.at(0).toDouble() - 22.0) < 1e-6,
            "reenabled ICP applies the next correction to the retained current scene");
        fixture.runtime.updateOnlineProcessingOptions(false, fusion);
        QCoreApplication::processEvents();
        ok &= expect(fixture.scene->writes == 2
                && std::abs(fixture.scene->pose.at(0).toDouble() - 22.0) < 1e-6,
            "disabling during postverification does not issue an implicit rollback SET");
        return ok;
    }

    static bool testSafetyPrecedenceAndOldEpoch()
    {
        bool ok = true;
        for (int state = 0; state < 3; ++state) {
            Fixture fixture;
            if (!fixture.beginCandidate(false)) { return expect(false, "unsafe-state fixture creates pending"); }
            if (state == 1) { fixture.runtime.m_latestRobotState.sHold = 1; }
            if (state == 2) { fixture.runtime.m_latestRobotState.sStart = 0; }
            fixture.route(true, state == 0);
            ok &= expect(fixture.runtime.m_localRegistrationFaulted && !fixture.runtime.m_localVerification.has_value()
                    && fixture.scene->writes == 1,
                "latest real staleness, hold or stop retains existing cancellation ahead of slow-frame skipping");
        }
        Fixture oldEpoch;
        if (!oldEpoch.beginCandidate(false)) { return expect(false, "old-epoch fixture creates current pending"); }
        const auto generation = oldEpoch.runtime.m_localRegistrationGeneration;
        oldEpoch.route(true, false, true);
        ok &= expect(oldEpoch.pending() && oldEpoch.runtime.m_localRegistrationGeneration == generation,
            "late old-epoch slow result cannot cancel or replace the current transaction");
        Fixture unknown;
        unknown.scene->failWriteReadback = true;
        unknown.beginCandidate(false);
        const auto snapshot = unknown.twinWorker.localRegistrationSafetySnapshot();
        ok &= expect(unknown.runtime.m_localRegistrationFaulted && snapshot.hasPendingPose
                && !snapshot.pendingPoseConfirmed && unknown.scene->writes == 1,
            "unknown SET readback remains faulted rather than being reclassified as a slow frame");
        unknown.route(true);
        ok &= expect(unknown.runtime.m_localRegistrationFaulted && unknown.scene->writes == 1,
            "later slow input cannot clear an unknown-write fault");
        return ok;
    }
};
} // namespace i2r_rlc::ui

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    using Access = i2r_rlc::ui::CRuntimeCoordinatorHistoryTestAccess;
    const bool ordinary = Access::testOrdinarySlowFrame();
    const bool surface = Access::testSurfaceTimeoutRetention();
    const bool safety = Access::testSafetyPrecedenceAndOldEpoch();
    const bool toggle = Access::testToggleRetainsWrittenScene();
    return ordinary && surface && safety && toggle ? 0 : 1;
}
