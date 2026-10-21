/**
 * @file test_online_local_registration.cpp
 * @brief 在线局部点云配准及 RobotSim 位姿写回的边界回归。
 * @details 使用内存点云和受控 transport，验证小幅修正方向、限幅、取消与回读。
 */
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include <QCoreApplication>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSettings>
#include <QThread>

#include "OnlineLocalRegistrationWorker.h"
#include "AppPaths.h"
#include "RuntimeCoordinator.h"
#include "RuntimeWorkerAdapters.h"

namespace {
namespace ui = i2r_rlc::ui;
namespace twin = i2r_rlc::digital_twin;
namespace loc = i2r_rlc::localization;

bool expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool near(double actual, double expected)
{
    return std::abs(actual - expected) < 1e-3;
}

/** @brief 从实际运行目录检查配置及启动路径，防止配置语法错误令功能静默停用。 */
bool testRuntimeStartupConfiguration()
{
    QSettings settings(ui::AppPaths::fromDefaultLocation().digitalTwinConfigDir()
        .filePath(QStringLiteral("online_local_registration.ini")), QSettings::IniFormat);
    const int interval = settings.value(QStringLiteral("sample_interval_ms"), 0).toInt();
    bool ok = expect(interval >= 100 && settings.status() == QSettings::NoError,
        "production local registration INI must parse without disabling the feature");
    ui::CRuntimeCoordinator runtime;
    bool enabledReported = false;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::statusMessage,
        [&enabledReported](const QString &message) {
            enabledReported |= message.startsWith(QStringLiteral("在线局部定位：启用"));
        });
    // start 仅创建线程，不启动相机、不连接机器人或仿真场景。
    runtime.start();
    runtime.stop();
    return expect(enabledReported, "production runtime must create and enable its local worker") && ok;
}

/** @brief 无设备启动后热切换策略；参数更新与停止须保持线程生命周期安全。 */
bool testRuntimeHotProcessingOptions()
{
    using i2r_rlc::vision::KcfFusionMode;
    using i2r_rlc::vision::KcfFusionOptions;
    QSettings settings(ui::AppPaths::fromDefaultLocation().digitalTwinConfigDir()
        .filePath(QStringLiteral("online_local_registration.ini")), QSettings::IniFormat);
    const bool configuredEnabled = settings.value(QStringLiteral("enabled"), true).toBool();
    const double configuredDistance = settings.value(
        QStringLiteral("max_correspondence_distance_mm"), 10.0).toDouble();
    const double configuredTranslation = settings.value(
        QStringLiteral("maximum_translation_mm"), 30.0).toDouble();
    ui::CRuntimeCoordinator runtime;
    runtime.start();
    bool ok = expect(runtime.onlineLocalRegistrationEnabled() == configuredEnabled,
        "runtime getter must reflect production INI enabled state after start");
    ok &= expect(near(runtime.onlineLocalIcpDistance(), configuredDistance),
        "runtime ICP correspondence distance must initially reflect INI or default 10 mm");
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), configuredTranslation),
        "runtime maximum translation must reflect INI or default 30 mm");
    ok &= expect(runtime.onlineFusionOptions().mode == KcfFusionMode::AutomaticKalman,
        "runtime must initially retain automatic Kalman fusion");

    runtime.updateOnlineProcessingOptions(false, {KcfFusionMode::ManualWeight, 0.3});
    auto actual = runtime.onlineFusionOptions();
    ok &= expect(!runtime.onlineLocalRegistrationEnabled()
        && actual.mode == KcfFusionMode::ManualWeight && near(actual.simulationWeight, 0.3),
        "running runtime must accept ICP disable and manual fusion together");
    runtime.setOnlineLocalIcpTranslationLimit(7.5);
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 7.5)
        && near(runtime.onlineLocalIcpDistance(), configuredDistance)
        && !runtime.onlineLocalRegistrationEnabled()
        && runtime.onlineFusionOptions().mode == KcfFusionMode::ManualWeight
        && near(runtime.onlineFusionOptions().simulationWeight, 0.3),
        "translation adjustment must preserve correspondence range, disabled ICP and fusion");
    runtime.setOnlineLocalIcpTranslationLimit(30.0);
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 30.0),
        "running runtime must accept 30 mm translation limit");
    runtime.setOnlineLocalIcpTranslationLimit(100.0);
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 30.0),
        "excessive finite translation limit must clamp to 30 mm");
    runtime.setOnlineLocalIcpTranslationLimit(-1.0);
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 0.01),
        "negative finite translation limit must clamp to 0.01 mm");
    runtime.setOnlineLocalIcpTranslationLimit(std::numeric_limits<double>::quiet_NaN());
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 30.0),
        "NaN translation limit must fall back to 30 mm");
    runtime.setOnlineLocalIcpTranslationLimit(std::numeric_limits<double>::infinity());
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 30.0),
        "infinite translation limit must fall back to 30 mm");
    runtime.setOnlineLocalIcpDistance(7.5);
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 7.5)
        && !runtime.onlineLocalRegistrationEnabled()
        && runtime.onlineFusionOptions().mode == KcfFusionMode::ManualWeight
        && near(runtime.onlineFusionOptions().simulationWeight, 0.3),
        "changing match distance must preserve disabled ICP and manual fusion");
    runtime.setOnlineLocalIcpDistance(10.0);
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 10.0),
        "running runtime must allow the 10 mm upper correspondence limit");
    runtime.setOnlineLocalIcpDistance(100.0);
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 10.0),
        "finite correspondence distance above limit must clamp to 10 mm");
    runtime.setOnlineLocalIcpDistance(-1.0);
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 0.1),
        "finite negative correspondence distance must clamp to 0.1 mm");
    runtime.setOnlineLocalIcpDistance(std::numeric_limits<double>::quiet_NaN());
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 10.0),
        "NaN correspondence distance must fall back to 10 mm");
    runtime.setOnlineLocalIcpDistance(std::numeric_limits<double>::infinity());
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 10.0),
        "infinite correspondence distance must fall back to 10 mm");
    QCoreApplication::processEvents();
    runtime.updateOnlineProcessingOptions(true, {KcfFusionMode::KcfOnly, 0.3});
    ok &= expect(runtime.onlineLocalRegistrationEnabled()
        && runtime.onlineFusionOptions().mode == KcfFusionMode::KcfOnly,
        "running runtime must reenable ICP independently of KCF-only mode");
    ok &= expect(near(runtime.onlineLocalIcpDistance(), 10.0),
        "ICP enable and fusion changes must not overwrite correspondence distance");
    ok &= expect(near(runtime.onlineLocalIcpTranslationLimit(), 30.0),
        "correspondence, enable and fusion updates must not overwrite translation limit");

    runtime.updateOnlineProcessingOptions(true,
        {static_cast<KcfFusionMode>(999), std::numeric_limits<double>::quiet_NaN()});
    actual = runtime.onlineFusionOptions();
    ok &= expect(actual.mode == KcfFusionMode::AutomaticKalman
        && near(actual.simulationWeight, 0.5),
        "runtime must normalize invalid fusion mode and NaN weight");
    runtime.updateOnlineProcessingOptions(true, {KcfFusionMode::ManualWeight, -2.0});
    ok &= expect(near(runtime.onlineFusionOptions().simulationWeight, 0.0),
        "runtime must clamp finite negative simulation weight");
    runtime.updateOnlineProcessingOptions(true, {KcfFusionMode::ManualWeight, 2.0});
    ok &= expect(near(runtime.onlineFusionOptions().simulationWeight, 1.0),
        "runtime must clamp finite simulation weight above one");
    runtime.updateOnlineProcessingOptions(true,
        {KcfFusionMode::ManualWeight, std::numeric_limits<double>::infinity()});
    ok &= expect(near(runtime.onlineFusionOptions().simulationWeight, 0.5),
        "runtime must normalize infinite simulation weight");

    // 连续投递以及重复相同配置后立即停止，覆盖尚未排空的配置队列。
    for (int index = 0; index < 8; ++index) {
        const KcfFusionOptions fusion{KcfFusionMode::ManualWeight, index / 10.0};
        runtime.updateOnlineProcessingOptions(index % 2 == 0, fusion);
        runtime.updateOnlineProcessingOptions(index % 2 == 0, fusion);
        runtime.setOnlineLocalIcpDistance(5.0 + index * 0.5);
        runtime.setOnlineLocalIcpDistance(5.0 + index * 0.5);
        runtime.setOnlineLocalIcpTranslationLimit(5.0 + index * 0.5);
        runtime.setOnlineLocalIcpTranslationLimit(5.0 + index * 0.5);
    }
    runtime.stop();
    runtime.stop();
    QCoreApplication::processEvents();
    return ok;
}

/** @brief 非对称曲面避免平面切向及直线绕轴歧义，单位 mm。 */
std::vector<loc::Point3D_mm> makeCloud(double offsetX)
{
    std::vector<loc::Point3D_mm> points;
    for (int x = 0; x < 15; ++x) {
        for (int y = 0; y < 11; ++y) {
            points.push_back({x * 1.3 + offsetX, y * 1.7,
                0.025 * x * x + 0.013 * y * y + 0.01 * x * y});
        }
    }
    return points;
}

struct PoseRecord {
    std::mutex mutex;
    std::condition_variable condition;
    int writes = 0;
    bool mismatchReadback = false;
    bool failReadback = false;
    bool lastWriteAttempted = false;
    int parentReadDelayMs = 0;
    bool blockParentRead = false;
    bool parentReadEntered = false;
    bool releaseParentRead = false;
    QJsonArray lastSet;
    QJsonArray worldPose{100.0, 20.0, 0.0, 0.0, 0.0, 90.0};
};

/** @brief 保留父级 90 度旋转，用不同坐标系验证相对位姿左乘方向。 */
class CPoseTransport final : public twin::IRobotSimTransport {
public:
    explicit CPoseTransport(std::shared_ptr<PoseRecord> record)
        : m_record(std::move(record))
    {
    }

    twin::RobotSimTransportResult connect(
        const twin::RobotSimEndpointConfig &) override
    {
        m_connected = true;
        return {true, {}, QStringLiteral("test connected"), 0};
    }

    void disconnect() override { m_connected = false; }
    bool isConnected() const override { return m_connected; }

    twin::RobotSimTransportResult send(
        const QJsonDocument &document, bool expectResponse) override
    {
        const QJsonArray operation = document.array();
        const QString type = operation.at(1).toString();
        if (type == QStringLiteral("GET_ABS_POSE")) {
            const int objectId = operation.at(2).toArray().at(0).toInt();
            std::unique_lock<std::mutex> lock(m_record->mutex);
            if (objectId == 200 && m_record->blockParentRead) {
                m_record->parentReadEntered = true;
                m_record->condition.notify_all();
                m_record->condition.wait_for(lock, std::chrono::seconds(5), [this]() {
                    return m_record->releaseParentRead;
                });
            }
            if (objectId == 200 && m_record->parentReadDelayMs > 0) {
                QThread::msleep(static_cast<unsigned long>(m_record->parentReadDelayMs));
            }
            if (objectId == 300 && m_record->writes > 0 && m_record->failReadback) {
                return {false, {}, QStringLiteral("test readback failure"), 0};
            }
            const QJsonArray pose = objectId == 200
                ? QJsonArray{100.0, 0.0, 0.0, 0.0, 0.0, 90.0}
                : m_record->worldPose;
            return {true, QString::fromUtf8(QJsonDocument(pose).toJson(
                QJsonDocument::Compact)), QStringLiteral("test pose"), 0};
        }
        if (type == QStringLiteral("SET_POSE")) {
            std::lock_guard<std::mutex> lock(m_record->mutex);
            ++m_record->writes;
            m_record->lastSet = operation.at(2).toArray();
            const QJsonArray relative = m_record->lastSet.at(1).toArray();
            if (!m_record->mismatchReadback) {
                // worldFromParent * parentFromWorkpiece，父级旋转为 Rz(90)。
                m_record->worldPose = QJsonArray{
                    100.0 - relative.at(1).toDouble(), relative.at(0).toDouble(),
                    relative.at(2).toDouble(), relative.at(3).toDouble(),
                    relative.at(4).toDouble(), 90.0 + relative.at(5).toDouble()};
            }
            return {true, {}, QStringLiteral("test set"), 0};
        }
        return {true, expectResponse
            ? QStringLiteral("[[[[[1,2,3],[100,120]],[[4,5,6],[220,120]]]],[[160,120]]]")
            : QString(), QStringLiteral("test reference"), 0};
    }

private:
    std::shared_ptr<PoseRecord> m_record;
    bool m_connected = false;
};

i2r_rlc::robot_comm::RobotState reachableState()
{
    i2r_rlc::robot_comm::RobotState state;
    state.curx_r = 1389425;
    state.cury_r = -4882;
    state.curz_r = 688864;
    state.curRx_r = -1791754;
    state.curRy_r = -551313;
    state.curRz_r = -2478;
    return state;
}

/** @brief 所有 socket/位姿调用在专用线程中串行执行，epoch 设置器跨线程取消。 */
struct PoseFixture {
    std::shared_ptr<PoseRecord> record = std::make_shared<PoseRecord>();
    QThread thread;
    ui::CGlobalLocatePriorWorkerAdapter *worker;

    explicit PoseFixture(int referenceId)
        : worker(new ui::CGlobalLocatePriorWorkerAdapter(
              std::make_unique<CPoseTransport>(record)))
    {
        worker->moveToThread(&thread);
        QObject::connect(&thread, &QThread::finished, worker, &QObject::deleteLater);
        thread.start();
        QMetaObject::invokeMethod(worker, [this, referenceId]() {
            twin::RobotSimEndpointConfig endpoint;
            endpoint.port = 40000;
            endpoint.referenceObjectId = referenceId;
            twin::RobotSimSceneConfig scene;
            scene.robotId = 1001;
            scene.positionerId = 1002;
            scene.sensorId = 1013;
            scene.workpieceId = 300;
            scene.workpieceParentId = 200;
            worker->connectRobotSim(endpoint, scene, reachableState());
        }, Qt::BlockingQueuedConnection);
        worker->setLocalRegistrationEpoch(7);
    }

    ~PoseFixture()
    {
        {
            std::lock_guard<std::mutex> lock(record->mutex);
            record->releaseParentRead = true;
            record->condition.notify_all();
        }
        thread.quit();
        thread.wait();
    }

    std::future<twin::DigitalTwinCommandResult> apply(quint64 epoch)
    {
        loc::RigidTransform correction;
        correction.rowMajor[3] = 0.5;
        return applyTransform(epoch, correction);
    }

    std::future<twin::DigitalTwinCommandResult> applyTransform(
        quint64 epoch, const loc::RigidTransform &correction)
    {
        auto completion = std::make_shared<std::promise<twin::DigitalTwinCommandResult>>();
        auto future = completion->get_future();
        QMetaObject::invokeMethod(worker, [this, epoch, correction, completion]() {
            const auto result = worker->applyLocalRegistrationCorrection(epoch, correction);
            // 必须紧随 apply 在 worker 线程读取，不能从测试主线程读取普通成员。
            const bool writeAttempted = worker->localRegistrationWriteAttempted();
            {
                std::lock_guard<std::mutex> lock(record->mutex);
                record->lastWriteAttempted = writeAttempted;
            }
            completion->set_value(result);
        }, Qt::QueuedConnection);
        return future;
    }
};

bool testWriteDirectionAndReadback()
{
    PoseFixture fixture(200);
    const auto result = fixture.apply(7).get();
    std::lock_guard<std::mutex> lock(fixture.record->mutex);
    const QJsonArray parameters = fixture.record->lastSet;
    bool ok = expect(result.succeeded, "small local correction must pass readback");
    ok &= expect(fixture.record->writes == 1, "one accepted correction writes exactly once");
    ok &= expect(fixture.record->lastWriteAttempted,
        "confirmed SET_POSE must expose attempted-write outcome");
    if (parameters.size() < 4) {
        return expect(false, "SET_POSE must contain relative/children flags");
    }
    const auto pose = parameters.at(1).toArray();
    ok &= expect(near(pose.at(0).toDouble(), 20.5)
        && near(pose.at(1).toDouble(), 0.0), "correction left-multiplies parent-relative pose");
    ok &= expect(parameters.at(2).toBool() && parameters.at(3).toBool(),
        "SET_POSE must use relative coordinates and update children");
    ok &= expect(near(fixture.record->worldPose.at(1).toDouble(), 20.5),
        "parent X correction becomes world Y correction");
    return ok;
}

bool testWriteRejection()
{
    bool ok = true;
    {
        PoseFixture fixture(200);
        fixture.worker->setLocalRegistrationStateDeadline(QDateTime::currentMSecsSinceEpoch() - 1);
        ok &= expect(!fixture.apply(7).get().succeeded, "expired robot snapshot must reject writeback");
        ok &= expect(fixture.record->writes == 0 && !fixture.record->lastWriteAttempted,
            "expired snapshot must not send or claim SET_POSE");
    }
    {
        PoseFixture fixture(201);
        ok &= expect(!fixture.apply(7).get().succeeded, "wrong reference frame must reject writeback");
        ok &= expect(fixture.record->writes == 0 && !fixture.record->lastWriteAttempted,
            "frame mismatch must not send or claim SET_POSE");
    }
    {
        PoseFixture fixture(200);
        fixture.record->mismatchReadback = true;
        ok &= expect(!fixture.apply(7).get().succeeded, "readback mismatch must be reported as failure");
        ok &= expect(fixture.record->lastWriteAttempted,
            "readback mismatch must retain attempted-write outcome");
    }
    {
        PoseFixture fixture(200);
        fixture.worker->setLocalRegistrationEpoch(8);
        ok &= expect(!fixture.apply(7).get().succeeded, "already stale epoch must reject writeback");
        ok &= expect(fixture.record->writes == 0 && !fixture.record->lastWriteAttempted,
            "stale epoch must not send or claim SET_POSE");
    }
    return ok;
}

bool testCancelDuringRead()
{
    PoseFixture fixture(200);
    {
        std::lock_guard<std::mutex> lock(fixture.record->mutex);
        fixture.record->blockParentRead = true;
    }
    auto future = fixture.apply(7);
    bool entered = false;
    {
        std::unique_lock<std::mutex> lock(fixture.record->mutex);
        entered = fixture.record->condition.wait_for(lock, std::chrono::seconds(3), [&]() {
            return fixture.record->parentReadEntered;
        });
    }
    fixture.worker->setLocalRegistrationEpoch(8);
    {
        std::lock_guard<std::mutex> lock(fixture.record->mutex);
        fixture.record->releaseParentRead = true;
        fixture.record->condition.notify_all();
    }
    const auto result = future.get();
    bool ok = expect(entered, "test must reach blocked pose query");
    ok &= expect(!result.succeeded, "epoch changed during GET must cancel writeback");
    ok &= expect(fixture.record->writes == 0 && !fixture.record->lastWriteAttempted,
        "canceled read must never send or claim SET_POSE");
    return ok;
}

/** @brief 写入标记区分确定未发送与发送后未确认，并在下一次调用开始清除。 */
bool testWriteAttemptOutcomeReset()
{
    bool ok = true;
    {
        PoseFixture fixture(200);
        ok &= expect(fixture.apply(7).get().succeeded && fixture.record->lastWriteAttempted,
            "normal local write must mark SET_POSE attempted");
        fixture.worker->setLocalRegistrationEpoch(8);
        ok &= expect(!fixture.apply(7).get().succeeded && !fixture.record->lastWriteAttempted,
            "next canceled apply must clear previous attempted-write outcome");
        ok &= expect(fixture.record->writes == 1,
            "canceled apply after success must not repeat prior write");
    }
    {
        PoseFixture fixture(200);
        fixture.record->failReadback = true;
        ok &= expect(!fixture.apply(7).get().succeeded && fixture.record->lastWriteAttempted
            && fixture.record->writes == 1,
            "SET_POSE followed by failed GET must remain marked as attempted");
        fixture.worker->setLocalRegistrationEpoch(8);
        ok &= expect(!fixture.apply(7).get().succeeded && !fixture.record->lastWriteAttempted,
            "failed readback outcome must not leak into the next canceled call");
    }
    {
        PoseFixture fixture(200);
        fixture.record->parentReadDelayMs = 180;
        ok &= expect(!fixture.apply(7).get().succeeded && !fixture.record->lastWriteAttempted
            && fixture.record->writes == 0,
            "pose read beyond 150 ms must not claim a SET_POSE attempt");
    }
    return ok;
}

ui::COnlineLocalRegistrationWorker::Options cloudOptions()
{
    ui::COnlineLocalRegistrationWorker::Options options;
    options.minimumFrames = 1;
    options.sampleIntervalMs = 0;
    options.minimumMatchIntervalMs = 0;
    // 正确性测试不依赖调试/CI 机器的求解速度，生产默认预算为 2500 ms。
    options.maximumSolveTimeMs = 2000;
    return options;
}

/** @brief 开关重配丢弃半窗，禁止恢复后将关闭前的数据拼成配准结果。 */
bool testWorkerHotEnableIsolation()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    auto options = cloudOptions();
    options.minimumFrames = 2;
    worker.configure(options);
    const auto real = makeCloud(0.25);
    const auto reference = makeCloud(0.0);
    bool ok = expect(!worker.processClouds(41, 3, 1, real, reference).attempted,
        "first sample must leave an incomplete window before disable");
    options.enabled = false;
    worker.configure(options);
    const auto disabled = worker.processClouds(41, 3, 2, real, reference);
    ok &= expect(!disabled.attempted && !disabled.accepted && disabled.frameCount == 0,
        "disabled local worker must neither accumulate nor solve");
    options.enabled = true;
    worker.configure(options);
    const auto resumed = worker.processClouds(41, 3, 3, real, reference);
    ok &= expect(!resumed.attempted && resumed.frameCount == 1,
        "reenabled worker must begin a new window");
    ok &= expect(worker.processClouds(41, 3, 4, real, reference).attempted,
        "reenabled worker must resume solving after a fresh complete window");
    return ok;
}

bool testSmallCloudCorrection()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    worker.configure(cloudOptions());
    const auto result = worker.processClouds(1, 1, 1, makeCloud(0.25), makeCloud(0.0));
    std::cout << "PERF: synthetic surface points=" << result.pointCount
        << " solve_ms=" << result.solveElapsedMs << " total_ms=" << result.elapsedMs << '\n';
    bool ok = expect(result.attempted && result.accepted,
        "nondegenerate small local cloud shift must be accepted");
    ok &= expect(result.afterResidualMm < result.beforeResidualMm * 0.5,
        "accepted surface alignment must improve nearest residual");
    ok &= expect(std::abs(result.correction.rowMajor[3] - 0.25) < 0.08,
        "output virtual-to-real correction must have positive X direction");
    return ok;
}

bool testWindowAndSampleBounds()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    auto options = cloudOptions();
    options.minimumFrames = 3;
    options.maximumFrames = 3;
    options.maximumPointsPerCloud = 64;
    worker.configure(options);
    const auto real = makeCloud(0.25);
    const auto reference = makeCloud(0.0);
    bool ok = expect(!worker.processClouds(1, 1, 1, real, reference).attempted,
        "one frame must not satisfy three-frame window");
    ok &= expect(!worker.processClouds(1, 1, 2, real, reference).attempted,
        "two frames must not satisfy three-frame window");
    const auto result = worker.processClouds(1, 1, 3, real, reference);
    ok &= expect(result.attempted && result.frameCount == 3,
        "third distinct frame must trigger bounded solve");
    ok &= expect(result.pointCount <= 64,
        "largest cloud must remain bounded by configured per-cloud limit");
    ok &= expect(!worker.processClouds(1, 1, 4, real, reference).attempted,
        "completed window must not be reused for another correction");
    worker.reset();
    options.sampleIntervalMs = 60000;
    worker.configure(options);
    worker.processClouds(1, 1, 10, real, reference);
    const auto throttled = worker.processClouds(1, 1, 11, real, reference);
    ok &= expect(!throttled.attempted && throttled.frameCount <= 1,
        "rapid samples must not fill the local registration window");
    options.minimumFrames = 1;
    options.sampleIntervalMs = 0;
    options.minimumMatchIntervalMs = 60000;
    worker.configure(options);
    ok &= expect(worker.processClouds(1, 1, 20, real, reference).attempted,
        "first eligible window may solve immediately");
    ok &= expect(!worker.processClouds(1, 1, 21, real, reference).attempted,
        "solve cadence must prevent back-to-back ICP requests");
    return ok;
}

bool testGenerationIsolationAndDegeneracy()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    auto options = cloudOptions();
    options.minimumFrames = 2;
    worker.configure(options);
    const auto real = makeCloud(0.25);
    const auto reference = makeCloud(0.0);
    worker.processClouds(1, 1, 1, real, reference);
    bool ok = expect(!worker.processClouds(1, 2, 2, real, reference).attempted,
        "model generation change must discard old reference cloud");
    ok &= expect(!worker.processClouds(2, 2, 3, real, reference).attempted,
        "session change must discard old real cloud");
    options.minimumFrames = 1;
    worker.configure(options);
    std::vector<loc::Point3D_mm> line;
    for (int index = 0; index < 80; ++index) {
        line.push_back({index * 0.5, 0.0, 0.0});
    }
    const auto degenerate = worker.processClouds(3, 1, 1, line, line);
    ok &= expect(!degenerate.accepted, "collinear line must never move workpiece");
    worker.reset();
    const auto empty = worker.processClouds(3, 1, 2, {}, reference);
    ok &= expect(!empty.accepted, "empty real cloud must never move workpiece");
    return ok;
}

bool testCorrectionLimit()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    auto options = cloudOptions();
    options.maximumTranslationMm = 0.05;
    worker.configure(options);
    const auto result = worker.processClouds(1, 1, 1, makeCloud(0.25), makeCloud(0.0));
    return expect(result.attempted && !result.accepted,
        "improved ICP beyond configured small-motion bound must be rejected");
}

/** @brief 可手算图像入口：相机与末节重合，线在末节 z=1000、父级 TCP z=769 mm。 */
ui::COnlineLocalRegistrationWorker::Input makeSurfaceInput(qint64 frameId, int stripeY)
{
    ui::COnlineLocalRegistrationWorker::Input input;
    input.sessionId = 41;
    input.generation = 3;
    input.frame.frameId = frameId;
    input.frame.image = QImage(320, 240, QImage::Format_Grayscale8);
    input.frame.image.fill(0);
    for (int y = stripeY - 3; y <= stripeY + 3; ++y) {
        uchar *row = input.frame.image.scanLine(y);
        for (int x = 60; x <= 260; ++x) {
            // 对称亮度峰使 Steger 中心位于给定的整数行。
            row[x] = static_cast<uchar>(255 - 35 * std::abs(y - stripeY));
        }
    }
    input.robotState.sMode = 2;
    input.robotState.sStart = 1;
    input.robotState.sHold = 0;
    input.robotState.traflag = 0;
    auto &sensor = input.trackingOptions.sensorCalibration;
    sensor.valid = true;
    sensor.focalLengthM = 0.01;
    sensor.sxM = 0.000005;
    sensor.syM = 0.000005;
    sensor.cxPixel = 160.0;
    sensor.cyPixel = 120.0;
    sensor.laserPlaneC = -1.0;
    auto &positioner = input.trackingOptions.positionerCalibration;
    positioner.valid = true;
    positioner.toolTcpInLastLinkMm = Eigen::Vector3d(0.0, 0.0, 231.0);
    input.trackingOptions.requireDetectionWindow = false;
    input.observation.valid = true;
    input.observation.frameId = frameId;
    input.observation.referenceObjectId = 200;
    input.observation.hasWeldPointImage_px = true;
    input.observation.weldPointImage_px = {160.0, static_cast<double>(stripeY)};
    // 生产走廊过滤拒绝跨度超过 100 pixel 的参考相邻点；仿真线应保持采样连续。
    for (int x = 60; x <= 260; x += 10) {
        input.observation.laserLineImagePoints_px.push_back({
            static_cast<double>(x), static_cast<double>(stripeY)});
    }
    for (int x = 60; x <= 260; ++x) {
        input.observation.laserLinePoints_mm.push_back({
            0.5 * static_cast<double>(x - 160), 0.5 * static_cast<double>(stripeY - 120), 768.75});
    }
    return input;
}

bool testImageExtractionInParentTcpFrame()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    auto options = cloudOptions();
    options.minimumFrames = 2;
    worker.configure(options);
    const auto first = worker.process(makeSurfaceInput(1, 100));
    bool ok = expect(!first.attempted && first.frameCount == 1 && first.pointCount > 0,
        "online traflag=0 image must extract surface points into first window frame");
    const auto second = worker.process(makeSurfaceInput(2, 140));
    if (!second.accepted) {
        std::cerr << "image local registration: " << second.message.toStdString() << '\n';
    }
    ok &= expect(second.attempted && second.accepted,
        "two separated extracted laser lines must register as a local surface");
    ok &= expect(std::abs(second.correction.rowMajor[11] - 0.25) < 0.08,
        "real TCP z=769 must align virtual parent TCP z=768.75 by +0.25 without adding 231");
    // Gabor/Steger 后的条纹不等于理想几何线；初值应在既有匹配范围内，
    // 修正后应贴合。不能把理想 Z 差 0.25 当成整云最近邻初始残差。
    ok &= expect(second.beforeResidualMm > 0.1 && second.beforeResidualMm < 5.0
        && second.afterResidualMm < 0.1,
        "same-parent image cloud must enter existing ICP capture range and align below 0.1 mm");
    ok &= expect(second.afterResidualMm < second.beforeResidualMm,
        "image-to-scene local alignment must improve surface residual");
    std::cout << "PERF: synthetic image extraction_ms=" << first.elapsedMs
        << " extraction_and_match_ms=" << second.elapsedMs
        << " solve_ms=" << second.solveElapsedMs
        << " before_mm=" << second.beforeResidualMm
        << " after_mm=" << second.afterResidualMm << '\n';
    if (second.accepted) {
        PoseFixture fixture(200);
        const auto written = fixture.applyTransform(7, second.correction).get();
        ok &= expect(written.succeeded && fixture.record->writes == 1,
            "real image to local ICP to RobotSim must commit exactly once and verify");
        ok &= expect(std::abs(fixture.record->worldPose.at(2).toDouble() - 0.25) < 0.08,
            "image-derived normal correction must reach simulated workpiece world pose");
    }
    return ok;
}

/** @brief 非零 XYZ TCP 安装偏移只由投影器处理，局部入口不能再次应用。 */
bool testImageExtractionWithDifferentTcpOffsets()
{
    bool ok = true;
    double zeroOffsetResidualMm = 0.0;
    const std::array<Eigen::Vector3d, 3> offsets{
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(12.0, -7.0, 231.0),
        Eigen::Vector3d(0.0, 0.0, 150.0)};
    for (const auto &offset : offsets) {
        ui::COnlineLocalRegistrationWorker worker(nullptr);
        auto options = cloudOptions();
        options.minimumFrames = 2;
        worker.configure(options);
        const auto makeInput = [&offset](qint64 frameId, int stripeY) {
            auto input = makeSurfaceInput(frameId, stripeY);
            input.trackingOptions.positionerCalibration.toolTcpInLastLinkMm = offset;
            for (auto &point : input.observation.laserLinePoints_mm) {
                point.x_mm -= offset.x();
                point.y_mm -= offset.y();
                point.z_mm = 999.75 - offset.z();
            }
            return input;
        };
        worker.process(makeInput(1, 100));
        const auto result = worker.process(makeInput(2, 140));
        std::cout << "TCP fixture offset=" << offset.transpose()
            << " before_mm=" << result.beforeResidualMm
            << " after_mm=" << result.afterResidualMm
            << " accepted=" << result.accepted << '\n';
        if (offset.isZero()) {
            zeroOffsetResidualMm = result.beforeResidualMm;
        }
        ok &= expect(result.accepted && result.afterResidualMm < 0.1
            && std::abs(result.beforeResidualMm - zeroOffsetResidualMm) < 1e-3
            && std::abs(result.correction.rowMajor[11] - 0.25) < 0.08,
            "consistent TCP offsets must retain the zero-offset residual and the same small correction");
    }
    return ok;
}

/** @brief 捕获距离只决定对应点搜索，放宽到 10 mm 不能放宽 2 mm 工件移动限幅。 */
bool testLocalCorrespondenceDistanceAndMotionLimit()
{
    bool ok = expect(near(ui::COnlineLocalRegistrationWorker::Options{}
        .maximumCorrespondenceDistanceMm, 10.0),
        "local ICP default correspondence distance must be 10 mm");
    const auto runWithDistance = [](double distance) {
        ui::COnlineLocalRegistrationWorker worker(nullptr);
        auto options = cloudOptions();
        options.minimumFrames = 2;
        options.maximumCorrespondenceDistanceMm = distance;
        options.maximumTranslationMm = 2.0;
        worker.configure(options);
        const auto shiftedInput = [](qint64 frameId, int stripeY) {
            auto input = makeSurfaceInput(frameId, stripeY);
            for (auto &point : input.observation.laserLinePoints_mm) {
                point.z_mm -= 6.0;
            }
            return input;
        };
        worker.process(shiftedInput(1, 100));
        return worker.process(shiftedInput(2, 140));
    };
    const auto narrow = runWithDistance(5.0);
    const auto wide = runWithDistance(10.0);
    std::cout << "ICP match distance fixture: narrow_before=" << narrow.beforeResidualMm
        << " wide_before=" << wide.beforeResidualMm
        << " wide_after=" << wide.afterResidualMm
        << " wide_correction_z=" << wide.correction.rowMajor[11] << '\n';
    ok &= expect(narrow.attempted && !narrow.accepted && narrow.beforeResidualMm > 5.0
        && std::abs(narrow.correction.rowMajor[11]) < 0.1,
        "5 mm capture range must not solve a surface shifted by 6.25 mm");
    ok &= expect(wide.attempted && !wide.accepted && wide.afterResidualMm < 0.1
        && std::abs(wide.correction.rowMajor[11] - 6.25) < 0.08,
        "10 mm capture must solve 6.25 mm shift but still reject it under 2 mm motion bound");
    const auto clamped = runWithDistance(100.0);
    ok &= expect(clamped.attempted && !clamped.accepted && clamped.afterResidualMm < 0.1
        && std::abs(clamped.correction.rowMajor[11] - 6.25) < 0.08,
        "worker must clamp excessive correspondence distance while retaining motion safety");
    const auto fallback = runWithDistance(std::numeric_limits<double>::quiet_NaN());
    ok &= expect(fallback.attempted && !fallback.accepted
        && fallback.afterResidualMm < 0.1
        && std::abs(fallback.correction.rowMajor[11] - 6.25) < 0.08,
        "worker NaN distance must fall back to 10 mm while retaining the 2 mm motion bound");
    return ok;
}

/** @brief 匹配范围及平移限幅独立生效，放宽位移仍需通过残差质量验收。 */
bool testAdjustableTranslationLimitAndQuality()
{
    bool ok = expect(near(ui::COnlineLocalRegistrationWorker::Options{}.maximumTranslationMm, 30.0),
        "default local translation limit must be 30 mm");
    const auto runWithOptions = [](const ui::COnlineLocalRegistrationWorker::Options &options) {
        ui::COnlineLocalRegistrationWorker worker(nullptr);
        worker.configure(options);
        const auto shiftedInput = [](qint64 frameId, int stripeY) {
            auto input = makeSurfaceInput(frameId, stripeY);
            for (auto &point : input.observation.laserLinePoints_mm) {
                point.z_mm -= 6.0;
            }
            return input;
        };
        worker.process(shiftedInput(1, 100));
        return worker.process(shiftedInput(2, 140));
    };
    auto options = cloudOptions();
    options.minimumFrames = 2;
    options.maximumCorrespondenceDistanceMm = 10.0;
    options.maximumTranslationMm = 5.0;
    const auto restricted = runWithOptions(options);
    options.maximumTranslationMm = 10.0;
    const auto allowed = runWithOptions(options);
    ok &= expect(restricted.attempted && !restricted.accepted
        && restricted.afterResidualMm < 0.1
        && std::abs(restricted.correction.rowMajor[11] - 6.25) < 0.08,
        "5 mm translation limit must reject a solved 6.25 mm correction");
    ok &= expect(allowed.attempted && allowed.accepted && allowed.afterResidualMm < 0.1
        && allowed.afterResidualMm <= 2.0
        && allowed.afterResidualMm <= allowed.beforeResidualMm * 0.95
        && std::abs(allowed.correction.rowMajor[11] - 6.25) < 0.08,
        "10 mm translation limit may accept 6.25 mm only after existing quality gates pass");
    options.maximumTranslationMm = 100.0;
    ok &= expect(runWithOptions(options).accepted,
        "worker must normalize finite translation limit above 30 mm");
    options.maximumTranslationMm = std::numeric_limits<double>::quiet_NaN();
    ok &= expect(runWithOptions(options).accepted,
        "worker NaN translation limit must fall back to 30 mm");
    options.maximumTranslationMm = std::numeric_limits<double>::infinity();
    ok &= expect(runWithOptions(options).accepted,
        "worker infinite translation limit must fall back to 30 mm");
    options.maximumTranslationMm = 10.0;
    options.maximumResidualMm = 0.01;
    const auto strictResidual = runWithOptions(options);
    ok &= expect(strictResidual.attempted && !strictResidual.accepted
        && strictResidual.afterResidualMm > options.maximumResidualMm
        && strictResidual.afterResidualMm < 0.1,
        "a relaxed translation limit must not bypass stricter residual acceptance");

    ui::COnlineLocalRegistrationWorker worker(nullptr);
    options = cloudOptions();
    options.maximumCorrespondenceDistanceMm = 10.0;
    options.maximumTranslationMm = 10.0;
    worker.configure(options);
    const auto reference = makeCloud(0.0);
    auto noisy = reference;
    for (std::size_t index = 0; index < noisy.size(); ++index) {
        noisy[index].z_mm += index % 2 == 0 ? 0.25 : -0.25;
    }
    const auto unimproved = worker.processClouds(41, 3, 1, noisy, reference);
    ok &= expect(unimproved.attempted && !unimproved.accepted
        && unimproved.afterResidualMm > 0.0 && unimproved.afterResidualMm <= 2.0
        && unimproved.afterResidualMm > unimproved.beforeResidualMm * 0.95,
        "10 mm movement limit must still reject less than 5 percent residual improvement");
    return ok;
}

bool testImageInputFaults()
{
    ui::COnlineLocalRegistrationWorker worker(nullptr);
    auto options = cloudOptions();
    options.minimumFrames = 2;
    worker.configure(options);
    auto input = makeSurfaceInput(1, 100);
    input.robotState.sHold = 1;
    const auto held = worker.process(input);
    bool ok = expect(!held.attempted && !held.accepted && held.frameCount == 0,
        "robot hold must reject real surface extraction");
    input = makeSurfaceInput(2, 100);
    input.observation.frameId = 999;
    const auto mismatch = worker.process(input);
    ok &= expect(!mismatch.attempted && mismatch.frameCount == 0,
        "wrong-frame simulation must not enter local window");
    input = makeSurfaceInput(3, 100);
    input.frame.image = {};
    const auto empty = worker.process(input);
    ok &= expect(!empty.attempted && empty.frameCount == 0,
        "empty real image must not enter local window");
    input = makeSurfaceInput(4, 100);
    input.frame.image.fill(0);
    const auto blank = worker.process(input);
    ok &= expect(!blank.attempted && blank.frameCount == 0,
        "no detected laser stripe must not enter local window");
    return ok;
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    bool ok = testRuntimeStartupConfiguration();
    ok &= testRuntimeHotProcessingOptions();
    ok &= testWorkerHotEnableIsolation();
    ok &= testWriteDirectionAndReadback();
    ok &= testWriteRejection();
    ok &= testCancelDuringRead();
    ok &= testWriteAttemptOutcomeReset();
    ok &= testSmallCloudCorrection();
    ok &= testWindowAndSampleBounds();
    ok &= testGenerationIsolationAndDegeneracy();
    ok &= testCorrectionLimit();
    ok &= testImageExtractionInParentTcpFrame();
    ok &= testImageExtractionWithDifferentTcpOffsets();
    ok &= testLocalCorrespondenceDistanceAndMotionLimit();
    ok &= testAdjustableTranslationLimitAndQuality();
    ok &= testImageInputFaults();
    if (ok) {
        std::cout << "PASS: online local registration tests\n";
    }
    return ok ? 0 : 1;
}
