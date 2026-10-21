/**
 * @file test_online_local_registration.cpp
 * @brief 在线局部点云配准及 RobotSim 位姿写回的边界回归。
 * @details 使用内存点云和受控 transport，验证小幅修正方向、限幅、取消与回读。
 */
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iostream>
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
            completion->set_value(worker->applyLocalRegistrationCorrection(epoch, correction));
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
        ok &= expect(fixture.record->writes == 0, "expired snapshot must not send SET_POSE");
    }
    {
        PoseFixture fixture(201);
        ok &= expect(!fixture.apply(7).get().succeeded, "wrong reference frame must reject writeback");
        ok &= expect(fixture.record->writes == 0, "frame mismatch must not send SET_POSE");
    }
    {
        PoseFixture fixture(200);
        fixture.record->mismatchReadback = true;
        ok &= expect(!fixture.apply(7).get().succeeded, "readback mismatch must be reported as failure");
    }
    {
        PoseFixture fixture(200);
        fixture.worker->setLocalRegistrationEpoch(8);
        ok &= expect(!fixture.apply(7).get().succeeded, "already stale epoch must reject writeback");
        ok &= expect(fixture.record->writes == 0, "stale epoch must not send SET_POSE");
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
    ok &= expect(fixture.record->writes == 0, "canceled read must never send SET_POSE");
    return ok;
}

ui::COnlineLocalRegistrationWorker::Options cloudOptions()
{
    ui::COnlineLocalRegistrationWorker::Options options;
    options.minimumFrames = 1;
    options.sampleIntervalMs = 0;
    options.minimumMatchIntervalMs = 0;
    // 正确性测试不依赖调试/CI 机器的求解速度，生产默认预算仍为 300 ms。
    options.maximumSolveTimeMs = 2000;
    return options;
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

/** @brief 用可手算标定构造在线图像入口；原始真实线位于末节 z=1000 mm。 */
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
            0.5 * static_cast<double>(x - 160), 0.5 * static_cast<double>(stripeY - 120), 999.75});
    }
    return input;
}

bool testImageExtractionAndTcpConversion()
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
        "real TCP z=769 plus 231 offset must align virtual last-link z=999.75 by +0.25");
    ok &= expect(second.afterResidualMm < second.beforeResidualMm,
        "image-to-scene local alignment must improve surface residual");
    std::cout << "PERF: synthetic image extraction_ms=" << first.elapsedMs
        << " extraction_and_match_ms=" << second.elapsedMs
        << " solve_ms=" << second.solveElapsedMs << '\n';
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
    ok &= testWriteDirectionAndReadback();
    ok &= testWriteRejection();
    ok &= testCancelDuringRead();
    ok &= testSmallCloudCorrection();
    ok &= testWindowAndSampleBounds();
    ok &= testGenerationIsolationAndDegeneracy();
    ok &= testCorrectionLimit();
    ok &= testImageExtractionAndTcpConversion();
    ok &= testImageInputFaults();
    if (ok) {
        std::cout << "PASS: online local registration tests\n";
    }
    return ok ? 0 : 1;
}
