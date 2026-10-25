/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_correction_robot_sim_frame_sync.cpp
 * @brief       纠偏逐帧 RobotSim 同步专项测试入口
 * @details     `--offline` 与 `--online` 共用同一脚本化 transport 和时序夹具，
 *              验证单 current、单 latest-pending、三键关联、连续失败阈值以及
 *              停止/重启迟到结果隔离，避免为两种 purpose 复制测试状态机。
 ******************************************************************************/
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string_view>
#include <vector>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QJsonDocument>
#include <QThread>

#include "RuntimeCoordinator.h"

namespace {

using namespace std::chrono_literals;
namespace digital_twin = i2r_rlc::digital_twin;
namespace robot_comm = i2r_rlc::robot_comm;
namespace ui = i2r_rlc::ui;

enum class TestMode {
    Offline,
    Online
};

ui::CorrectionPurpose correctionPurpose(TestMode mode)
{
    return mode == TestMode::Online
        ? ui::CorrectionPurpose::OnlineCorrection
        : ui::CorrectionPurpose::OfflineCorrection;
}

bool expectTrue(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

template<typename Predicate>
bool waitUntil(Predicate predicate, int timeoutMs = 3000)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(2);
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    return predicate();
}

robot_comm::RobotState makeReachableState()
{
    robot_comm::RobotState state;
    state.traflag = 1;
    state.sMode = 2;
    state.sServoPower = 1;
    state.sHold = 0;
    state.sStart = 1;
    // 该 TCP 对应测试覆盖过的 MH12 可逆解位姿。协议单位分别为 um 和
    // 0.0001 degree，保证测试失败来自同步链而不是逆解前置条件。
    state.curx_r = 1389425;
    state.cury_r = -4882;
    state.curz_r = 688864;
    state.curRx_r = -1791754;
    state.curRy_r = -551313;
    state.curRz_r = -2478;
    return state;
}

i2r_rlc::camera::CameraFrame makeFrame(qint64 frameId)
{
    i2r_rlc::camera::CameraFrame frame;
    frame.frameId = frameId;
    frame.image = QImage(320, 240, QImage::Format_Grayscale8);
    // 在线结果须有真实接缝几何；同步测试不再用没有折点的水平亮带。
    for (int y = 0; y < frame.image.height(); ++y) {
        uchar *row = frame.image.scanLine(y);
        for (int x = 0; x < frame.image.width(); ++x) {
            const double dx = x - 160.0;
            const double stripeY = 120.0 + (dx < 0.0 ? 0.28 : -0.32) * dx;
            const double dy = y - stripeY;
            row[x] = static_cast<uchar>(15.0 + 220.0 * std::exp(-dy * dy / 2.88));
        }
    }
    frame.previewImage = frame.image;
    return frame;
}

i2r_rlc::camera::CameraFrame makeBlankFrame(qint64 frameId)
{
    auto frame = makeFrame(frameId);
    frame.image.fill(0);
    frame.previewImage = frame.image;
    return frame;
}

i2r_rlc::vision::WeldProjectionContext makeProjectionContext()
{
    i2r_rlc::vision::WeldProjectionContext context;
    context.valid = true;
    context.calibration.valid = true;
    context.calibration.focalLengthM = 0.01;
    context.calibration.sxM = 0.00001;
    context.calibration.syM = 0.00001;
    context.calibration.intrinsics.cxPixel = 160.0;
    context.calibration.intrinsics.cyPixel = 120.0;
    context.calibration.laserPlaneC = -1.0;
    return context;
}

std::filesystem::path createOfflineArchive()
{
    const auto directory = std::filesystem::temp_directory_path()
        / ("i2r_rlc_frame_sync_"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    std::ofstream model(directory / "case_W.txt");
    for (int index = 0; index < 24; ++index) {
        model << index * 4.0 << ' ' << index * 0.25 << ' '
              << index * 0.1 << '\n';
    }
    std::ofstream teach(directory / "case_T.txt");
    for (int index = 0; index < 5; ++index) {
        teach << index * 16.0 << ' ' << index << ' ' << index * 0.5
              << " 1 2 3 10 20\n";
    }
    return directory / "case_W.txt";
}

ui::RuntimeOnlineCorrectionConfiguration createOnlineConfiguration()
{
    const auto directory = std::filesystem::temp_directory_path()
        / ("i2r_rlc_online_frame_sync_"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    const auto workPath = directory / "case_W.txt";
    const auto teachPath = directory / "case_T.txt";
    const auto rbcalibPath = directory / "RBCALIB.DAT";
    std::ofstream(workPath) << "0 0 0\n100 0 0\n";
    std::ofstream(teachPath)
        << "0 0 0 0 0 0 0 0\n100 0 0 0 0 0 0 0\n";
    std::ofstream(rbcalibPath) << "////SRANG 0,0,0,0,0,0\n";

    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.trajectoryRequest.workPathFile = workPath.string();
    configuration.trajectoryRequest.teachPoseFile = teachPath.string();
    configuration.rbcalibFile = rbcalibPath.string();
    return configuration;
}

struct RobotRecord {
    std::mutex mutex;
    std::vector<robot_comm::RobotCommandType> commands;
};

class CFakeRobotTransport final : public robot_comm::IRobotCommandTransport {
public:
    explicit CFakeRobotTransport(std::shared_ptr<RobotRecord> record)
        : m_record(std::move(record))
    {
    }

    bool connect(std::string_view, std::uint16_t) override
    {
        m_connected = true;
        return true;
    }
    void disconnect() override { m_connected = false; }
    bool isConnected() const override { return m_connected; }

    robot_comm::RobotCommandReceipt transact(
        robot_comm::RobotCommandType type,
        const std::string &request,
        int pointIndex,
        robot_comm::RobotState *state) override
    {
        Q_UNUSED(pointIndex);
        {
            std::lock_guard<std::mutex> lock(m_record->mutex);
            m_record->commands.push_back(type);
        }
        robot_comm::RobotCommandReceipt receipt;
        receipt.type = type;
        receipt.sent = true;
        receipt.responseReceived = true;
        receipt.completed = true;
        receipt.pointIndex = pointIndex;
        receipt.message = "fake robot response";
        if (state != nullptr) {
            *state = makeReachableState();
            if (type == robot_comm::RobotCommandType::TeachPointUpdate) {
                // 和生产O编码共用批次数量语义，中间点回4，末点才回5。
                const auto fields = QString::fromStdString(request).split(',');
                const int pointCount = fields.size() > 8 ? fields[8].toInt() : 0;
                state->offline = pointIndex + 1 == pointCount ? 5 : 4;
                receipt.completed = state->offline == 5;
            }
        }
        return receipt;
    }

private:
    std::shared_ptr<RobotRecord> m_record;
    bool m_connected = false;
};

enum class ScriptedReply {
    Valid,
    Empty,
    Invalid,
    TransportFailure
};

/**
 * @brief 可阻塞且可脚本化的 RobotSim transport。
 *
 * 所有控制字段受 mutex 保护；worker 线程只在 expectResponse 请求上计数和
 * 阻塞。测试线程因此可以稳定构造 frame 100 在途、101..105 覆盖 pending，
 * 而不会依赖机器速度或定时器碰运气。
 */
struct ScriptedRobotSimState {
    std::mutex mutex;
    std::condition_variable condition;
    std::vector<ScriptedReply> replies;
    int queryCount = 0;
    int activeQueries = 0;
    int maximumActiveQueries = 0;
    bool blockNextQuery = false;
    bool blockedQueryEntered = false;
    bool releaseBlockedQuery = false;
};

class CScriptedRobotSimTransport final
    : public digital_twin::IRobotSimTransport {
public:
    explicit CScriptedRobotSimTransport(
        std::shared_ptr<ScriptedRobotSimState> state)
        : m_state(std::move(state))
    {
    }

    digital_twin::RobotSimTransportResult connect(
        const digital_twin::RobotSimEndpointConfig &endpoint) override
    {
        m_connected = endpoint.port > 0;
        return {m_connected, {}, QStringLiteral("scripted connect"), 0};
    }
    void disconnect() override { m_connected = false; }
    bool isConnected() const override { return m_connected; }

    digital_twin::RobotSimTransportResult send(
        const QJsonDocument &request,
        bool expectResponse) override
    {
        Q_UNUSED(request);
        if (!expectResponse) {
            return {true, {}, QStringLiteral("scripted send"), 0};
        }

        ScriptedReply reply = ScriptedReply::Valid;
        {
            std::unique_lock<std::mutex> lock(m_state->mutex);
            ++m_state->queryCount;
            ++m_state->activeQueries;
            m_state->maximumActiveQueries = std::max(
                m_state->maximumActiveQueries,
                m_state->activeQueries);
            if (m_state->blockNextQuery) {
                m_state->blockNextQuery = false;
                m_state->blockedQueryEntered = true;
                m_state->condition.notify_all();
                m_state->condition.wait_for(
                    lock,
                    3s,
                    [this]() { return m_state->releaseBlockedQuery; });
                m_state->releaseBlockedQuery = false;
                m_state->blockedQueryEntered = false;
            }
            if (!m_state->replies.empty()) {
                reply = m_state->replies.front();
                m_state->replies.erase(m_state->replies.begin());
            }
            --m_state->activeQueries;
        }
        if (reply == ScriptedReply::Empty) {
            // JSON 合法但两个批次均为 null：服务应保持 supported，观测无效。
            // 这样可验证可恢复的空响应，而不会把 transport 永久断开。
            return {true, QStringLiteral("[null,null]"),
                    QStringLiteral("scripted empty"), 0};
        }
        if (reply == ScriptedReply::Invalid) {
            // 批次结构合法、内容为空数组，覆盖“格式存在但观测不可用”分支。
            return {true, QStringLiteral("[[],[]]"),
                    QStringLiteral("scripted invalid"), 0};
        }
        if (reply == ScriptedReply::TransportFailure) {
            return {false, {}, QStringLiteral("scripted transport failure"), 0};
        }
        return {
            true,
            QStringLiteral(
                "[[[[[1,2,3],[60,92]],[[2,3,4],[160,120]],[[4,5,6],[260,88]]]],[[160,120]]]") ,
            QStringLiteral("scripted valid"),
            0};
    }

private:
    std::shared_ptr<ScriptedRobotSimState> m_state;
    bool m_connected = false;
};

struct BlockingSynchronizedVisionState {
    std::mutex mutex;
    std::condition_variable condition;
    int callCount = 0;
    bool firstCallEntered = false;
    bool releaseFirstCall = false;
};

ui::SimulatedVisionCallbacks makeBlockingSynchronizedVisionCallbacks(
    const std::shared_ptr<BlockingSynchronizedVisionState> &state)
{
    ui::SimulatedVisionCallbacks callbacks;
    callbacks.beforeSynchronizedFrame = [state](qint64) {
        std::unique_lock<std::mutex> lock(state->mutex);
        ++state->callCount;
        if (state->callCount != 1) {
            return;
        }
        state->firstCallEntered = true;
        state->condition.notify_all();
        state->condition.wait_for(
            lock,
            3s,
            [state]() { return state->releaseFirstCall; });
    };
    return callbacks;
}

bool waitForBlockedVision(
    const std::shared_ptr<BlockingSynchronizedVisionState> &state)
{
    return waitUntil([state]() {
        std::lock_guard<std::mutex> lock(state->mutex);
        return state->firstCallEntered;
    });
}

void releaseBlockedVision(
    const std::shared_ptr<BlockingSynchronizedVisionState> &state)
{
    std::lock_guard<std::mutex> lock(state->mutex);
    state->releaseFirstCall = true;
    state->condition.notify_all();
}

/**
 * @brief 在线/离线共享的运行时专项夹具。
 *
 * mode 只决定正式会话的配置、启动和重置入口；RobotSim 脚本、冻结帧、
 * worker 线程及观测记录完全共用。这保证两种 purpose 接受同一契约验证，
 * 同时避免测试代码复制出一套可能偏离生产共享状态机的“镜像实现”。
 */
struct CorrectionFixture {
    explicit CorrectionFixture(TestMode requestedMode)
        : mode(requestedMode)
    {
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::correctionSyncUpdated,
            [&updates = syncUpdates](const ui::CorrectionSyncUpdate &update) {
                updates.push_back(update);
            });
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::trackingResultReady,
            [&results = trackingResults](const ui::RuntimeTrackingResult &result) {
                results.push_back(result);
            });
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::offlineCorrectionUpdated,
            [&updates = offlineUpdates](
                const i2r_rlc::control::OfflineCorrectionUpdate &update) {
                updates.push_back(update);
            });
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::onlineCorrectionUpdated,
            [&updates = onlineUpdates](
                const i2r_rlc::control::OnlineCorrectionUpdate &update) {
                updates.push_back(update);
            });
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::robotSimConnectionStateChanged,
            [this](bool connected, const QString &) {
                robotSimConnected = connected;
            });
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::robotSimStateSynchronizationChanged,
            [this](bool succeeded, const QString &) {
                robotSimSynchronized = succeeded;
            });
        QObject::connect(
            &runtime,
            &ui::CRuntimeCoordinator::statusMessage,
            [this](const QString &message) {
                if (message == QStringLiteral("vision tracking started")) {
                    ++visionStartCount;
                }
            });
    }

    TestMode mode;
    std::shared_ptr<RobotRecord> robot = std::make_shared<RobotRecord>();
    std::shared_ptr<ScriptedRobotSimState> robotSim =
        std::make_shared<ScriptedRobotSimState>();
    ui::CRuntimeCoordinator runtime{
        std::make_unique<CFakeRobotTransport>(robot),
        std::make_unique<CScriptedRobotSimTransport>(robotSim)};
    std::vector<ui::CorrectionSyncUpdate> syncUpdates;
    std::vector<ui::RuntimeTrackingResult> trackingResults;
    std::vector<i2r_rlc::control::OfflineCorrectionUpdate> offlineUpdates;
    std::vector<i2r_rlc::control::OnlineCorrectionUpdate> onlineUpdates;
    std::string offlineArchivePath;
    bool robotSimConnected = false;
    bool robotSimSynchronized = false;
    int visionStartCount = 0;
    int expectedVisionStartCount = 0;

    void start(
        bool publishState,
        bool connectRobotSim,
        bool connectRobotForCommands = false)
    {
        runtime.start();
        ui::RuntimeTrackingOptions trackingOptions;
        // 正式在线会注入本会话RBCALIB，必须同时提供完整的SENSORPARA；
        // 单独传旧projectionContext不能替代动态标定构建所需的输入。
        trackingOptions.sensorCalibration.valid = true;
        trackingOptions.sensorCalibration.focalLengthM = 0.01;
        trackingOptions.sensorCalibration.sxM = 0.00001;
        trackingOptions.sensorCalibration.syM = 0.00001;
        trackingOptions.sensorCalibration.cxPixel = 160.0;
        trackingOptions.sensorCalibration.cyPixel = 120.0;
        trackingOptions.sensorCalibration.laserPlaneC = -1.0;
        trackingOptions.sensorCalibration.cameraZInToolM = -1.0;
        trackingOptions.positionerCalibration.valid = true;
        runtime.configureTracking(trackingOptions, makeProjectionContext());
        if (mode == TestMode::Online) {
            runtime.configureOnlineCorrection(createOnlineConfiguration());
            if (connectRobotForCommands) {
                runtime.connectRobot();
            }
        } else {
            offlineArchivePath = createOfflineArchive().string();
            runtime.configureOfflineCorrectionArchive(offlineArchivePath);
        }
        if (publishState) {
            runtime.publishRobotState(makeReachableState());
        }
        if (connectRobotSim) {
            digital_twin::RobotSimEndpointConfig endpoint;
            endpoint.port = 40000;
            digital_twin::RobotSimSceneConfig scene;
            scene.sceneName = QStringLiteral("correction-frame-sync-test");
            scene.robotId = 1001;
            scene.positionerId = 1002;
            scene.sensorId = 1013;
            runtime.connectRobotSim(endpoint, scene);
            waitUntil([this]() {
                return robotSimConnected && robotSimSynchronized;
            }, 4000);
        }
        startCorrection();
        waitForDetection();
    }

    void startCorrection()
    {
        expectedVisionStartCount = visionStartCount + 1;
        if (mode == TestMode::Online) {
            runtime.startOnlineTracking();
        } else {
            runtime.startOfflineCorrectionDetection();
            // Offline 必须先经过真实停止状态预检，WaitingForDetection 本身
            // 不代表 Vision 已启动；状态按序排在 startDetection 之后。
            auto stopped = makeReachableState();
            stopped.sStart = 0;
            stopped.traflag = 0;
            stopped.OfflineRun = 0;
            runtime.publishRobotState(stopped);
        }
    }

    void resetCorrection()
    {
        if (mode == TestMode::Online) {
            runtime.stopOnlineTracking();
        } else {
            runtime.resetOfflineCorrection();
        }
    }

    void restartCorrection()
    {
        resetCorrection();
        if (mode == TestMode::Offline) {
            // Offline controller 的 reset 按既有语义释放 archive；新会话必须
            // 显式重新注入相同存档，不能依赖已清理的 worker 内部缓存。
            runtime.configureOfflineCorrectionArchive(offlineArchivePath);
        }
        startCorrection();
    }

    bool waitForDetection()
    {
        const bool ready = waitUntil([this]() {
            if (mode == TestMode::Online) {
                return visionStartCount >= expectedVisionStartCount && std::any_of(
                    onlineUpdates.begin(),
                    onlineUpdates.end(),
                    [](const i2r_rlc::control::OnlineCorrectionUpdate &update) {
                        return update.succeeded
                            && update.phase
                                == i2r_rlc::control::OnlineCorrectionPhase::InitialDetectionRunning;
                    });
            }
            return visionStartCount >= expectedVisionStartCount && std::any_of(
                offlineUpdates.begin(),
                offlineUpdates.end(),
                [](const i2r_rlc::control::OfflineCorrectionUpdate &update) {
                    return update.succeeded
                        && update.phase
                            == i2r_rlc::control::OfflineCorrectionPhase::WaitingForDetection;
                });
        });
        // correction worker 先发布业务阶段，再发 startTrackingRequested。
        // 再泵一次事件队列，确保 coordinator 已切到当前 session 后才发帧。
        QElapsedTimer settleTimer;
        settleTimer.start();
        while (settleTimer.elapsed() < 80) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            QThread::msleep(2);
        }
        if (ready && mode == TestMode::Offline) {
            // 只有真实 Vision startTracking 已完成，才开放扫描窗口和测试帧。
            runtime.publishRobotState(makeReachableState());
        }
        return ready;
    }

    void stop()
    {
        releaseBlockedQuery();
        runtime.stop();
    }

    int queryCount() const
    {
        std::lock_guard<std::mutex> lock(robotSim->mutex);
        return robotSim->queryCount;
    }

    void script(std::initializer_list<ScriptedReply> replies)
    {
        std::lock_guard<std::mutex> lock(robotSim->mutex);
        robotSim->replies.insert(
            robotSim->replies.end(), replies.begin(), replies.end());
    }

    void blockNextQuery()
    {
        std::lock_guard<std::mutex> lock(robotSim->mutex);
        robotSim->blockNextQuery = true;
        robotSim->releaseBlockedQuery = false;
    }

    bool waitForBlockedQuery()
    {
        // 相机 worker 先把 frameReady 排回主线程；主线程等待阻塞点时必须继续
        // 泵 Qt 事件，否则会把尚未进入 coordinator 的帧误判成“没有查询”。
        return waitUntil([this]() {
            std::lock_guard<std::mutex> lock(robotSim->mutex);
            return robotSim->blockedQueryEntered;
        });
    }

    void releaseBlockedQuery()
    {
        std::lock_guard<std::mutex> lock(robotSim->mutex);
        robotSim->releaseBlockedQuery = true;
        robotSim->condition.notify_all();
    }
};

bool testNoRobotStateDoesNotQuery(TestMode mode)
{
    if (mode == TestMode::Offline) {
        // Offline 公开入口必须先用 RobotState 完成预检，无法同时构造
        // “从未有状态且已经 armed”。缺状态门由 Online 共用 dispatch 验证。
        std::printf("NOT APPLICABLE: offline no-state after public preflight is unreachable; shared dispatch is covered by --online\n");
        return true;
    }
    CorrectionFixture fixture(mode);
    fixture.start(false, false);
    const int before = fixture.queryCount();
    fixture.runtime.publishCameraFrame(makeFrame(10));
    const bool failed = waitUntil([&fixture]() {
        return std::any_of(
            fixture.syncUpdates.begin(),
            fixture.syncUpdates.end(),
            [](const ui::CorrectionSyncUpdate &update) {
                return update.state == ui::CorrectionSyncState::Failed
                    && update.message.contains(QStringLiteral("RobotState"));
            });
    });
    const int after = fixture.queryCount();
    fixture.stop();
    return expectTrue(failed, "missing RobotState should emit an explicit failure")
        && expectTrue(after == before,
                      "missing RobotState must not query RobotSim");
}

bool testLatestPendingAndSuccessfulCorrelation(TestMode mode)
{
    CorrectionFixture fixture(mode);
    fixture.start(true, true);
    fixture.blockNextQuery();
    const int before = fixture.queryCount();
    fixture.runtime.publishCameraFrame(makeFrame(100));
    bool ok = expectTrue(
        fixture.waitForBlockedQuery(),
        "frame 100 RobotSim query should enter the deterministic block");
    for (qint64 frameId = 101; frameId <= 105; ++frameId) {
        fixture.runtime.publishCameraFrame(makeFrame(frameId));
    }
    fixture.releaseBlockedQuery();
    ok &= expectTrue(
        waitUntil([&fixture]() { return fixture.trackingResults.size() >= 2U; }, 5000),
        "blocked current and latest pending frames should both reach Vision");

    std::vector<qint64> frames;
    for (const auto &result : fixture.trackingResults) {
        frames.push_back(result.frameId);
    }
    ok &= expectTrue(
        frames.size() == 2U && frames[0] == 100 && frames[1] == 105,
        "while frame 100 is current, frames 101..105 must collapse to frame 105");
    ok &= expectTrue(
        fixture.queryCount() - before == 2,
        "only current and latest-pending frames should issue per-frame queries");
    ok &= expectTrue(
        fixture.robotSim->maximumActiveQueries == 1,
        "RobotSim worker must never execute more than one current query");
    for (const auto &result : fixture.trackingResults) {
        ok &= expectTrue(
            result.correlation.sessionId != 0
                && result.correlation.purpose
                    == correctionPurpose(mode)
                && result.correlation.frameId == result.frameId,
            "Vision result must retain the exact session/purpose/frame key");
    }
    fixture.stop();
    return ok;
}

bool testFailuresResetThenThirdFailureStops(TestMode mode)
{
    if (mode == TestMode::Online) {
        CorrectionFixture fixture(mode);
        fixture.start(true, true);
        fixture.script({ScriptedReply::Empty, ScriptedReply::Invalid,
            ScriptedReply::Valid, ScriptedReply::Empty, ScriptedReply::Invalid, ScriptedReply::Empty});
        bool ok = true;
        for (qint64 frameId = 200; frameId <= 205; ++frameId) {
            fixture.runtime.publishCameraFrame(makeFrame(frameId));
            ok &= expectTrue(waitUntil([&fixture, frameId]() {
                return std::any_of(fixture.trackingResults.begin(), fixture.trackingResults.end(),
                    [frameId](const ui::RuntimeTrackingResult &result) { return result.frameId == frameId; });
            }), "online optional-reference failure still reaches current-image Vision");
        }
        ok &= expectTrue(std::none_of(fixture.syncUpdates.begin(), fixture.syncUpdates.end(),
            [](const ui::CorrectionSyncUpdate &update) {
                return update.state == ui::CorrectionSyncState::StoppedAfterFailures;
            }), "missing optional CAD alone cannot stop independent online image detection");
        for (const auto &result : fixture.trackingResults) {
            ok &= expectTrue(result.evidencePolicyEnabled && result.frameContext.has_value()
                && result.frameContext->observation.frameId == result.frameId,
                "fallback retains same-exposure identity and never reuses previous observation");
        }
        fixture.stop();
        return ok;
    }
    CorrectionFixture fixture(mode);
    fixture.start(true, true);
    fixture.script({
        ScriptedReply::Empty,
        ScriptedReply::Invalid,
        ScriptedReply::Valid,
        ScriptedReply::Empty,
        ScriptedReply::Invalid,
        ScriptedReply::Empty});
    for (qint64 frameId = 200; frameId <= 205; ++frameId) {
        fixture.runtime.publishCameraFrame(makeFrame(frameId));
        waitUntil([&fixture, frameId]() {
            return std::any_of(
                fixture.syncUpdates.begin(),
                fixture.syncUpdates.end(),
                [frameId](const ui::CorrectionSyncUpdate &update) {
                    return update.frameId == frameId
                        && (update.state == ui::CorrectionSyncState::Failed
                            || update.state
                                == ui::CorrectionSyncState::VisionConsumed);
                });
        }, 4000);
    }

    bool sawResetSuccess = false;
    bool sawTerminal = false;
    for (const auto &update : fixture.syncUpdates) {
        if (update.frameId == 202
            && update.state == ui::CorrectionSyncState::ObservationReady
            && update.consecutiveSyncFailures == 0) {
            sawResetSuccess = true;
        }
        if (update.state == ui::CorrectionSyncState::StoppedAfterFailures
            && update.consecutiveSyncFailures == 3
            && update.message.contains(QStringLiteral("连续 3 帧"))) {
            sawTerminal = true;
        }
    }
    const int queriesAtStop = fixture.queryCount();
    fixture.runtime.publishCameraFrame(makeFrame(206));
    QThread::msleep(80);
    QCoreApplication::processEvents(QEventLoop::AllEvents, 30);

    int correctionCommandCount = 0;
    {
        std::lock_guard<std::mutex> lock(fixture.robot->mutex);
        correctionCommandCount = static_cast<int>(std::count_if(
            fixture.robot->commands.begin(),
            fixture.robot->commands.end(),
            [](robot_comm::RobotCommandType type) {
                return type == robot_comm::RobotCommandType::TeachPointUpdate
                    || type
                        == robot_comm::RobotCommandType::RealtimeCorrection;
            }));
    }
    bool ok = expectTrue(
        sawResetSuccess,
        "a valid observation after one/two failures must clear the counter before Vision")
        && expectTrue(sawTerminal,
                      "the third consecutive failure must publish terminal wording")
        && expectTrue(fixture.queryCount() == queriesAtStop,
                      "post-stop frames must not issue RobotSim queries")
        && expectTrue(correctionCommandCount == 0,
                      "failure cleanup and late results must not generate O/P commands");
    fixture.stop();
    return ok;
}

bool testTransportFailureAndWrongFrameDoNotReuseObservation(TestMode mode)
{
    digital_twin::DigitalTwinVisionLaserObservation wrongFrame;
    wrongFrame.valid = true;
    wrongFrame.frameId = 701;
    bool ok = expectTrue(
        !ui::isUsableCorrectionObservationForFrame(700, wrongFrame),
        "a valid observation with the wrong frameId must still be rejected");

    CorrectionFixture fixture(mode);
    fixture.start(true, true);
    fixture.script({ScriptedReply::TransportFailure});
    const int queriesBefore = fixture.queryCount();
    fixture.runtime.publishCameraFrame(makeFrame(700));
    ok &= expectTrue(
        waitUntil([&fixture]() {
            return std::any_of(
                fixture.syncUpdates.begin(),
                fixture.syncUpdates.end(),
                [](const ui::CorrectionSyncUpdate &update) {
                    return update.frameId == 700
                        && update.state == ui::CorrectionSyncState::Failed;
                });
        }),
        "transport failure should produce one explicit synchronization failure");
    ok &= expectTrue(
        fixture.queryCount() == queriesBefore + 1,
        "transport failure should consume exactly the current frame query");
    if (mode == TestMode::Online) {
        ok &= expectTrue(waitUntil([&fixture]() { return !fixture.trackingResults.empty(); }),
            "online transport failure still submits frozen original image");
        if (!fixture.trackingResults.empty()) {
            const auto &result = fixture.trackingResults.back();
            ok &= expectTrue(result.frameContext.has_value()
                && !result.frameContext->observation.valid
                && result.frameContext->observation.frameId == 700,
                "online fallback carries invalid current reference rather than an old observation");
        }
    } else {
        ok &= expectTrue(fixture.trackingResults.empty(),
            "offline transport failure still requires synchronized reference");
    }
    fixture.stop();
    return ok;
}

bool testStopAndRestartRejectsLateQueryAndVision(TestMode mode)
{
    CorrectionFixture fixture(mode);
    fixture.start(true, true);
    fixture.blockNextQuery();
    fixture.runtime.publishCameraFrame(makeFrame(300));
    bool ok = expectTrue(
        fixture.waitForBlockedQuery(),
        "old-session query should be blocked before stop");
    fixture.restartCorrection();
    ok &= expectTrue(fixture.waitForDetection(),
        "restarted correction must complete its actual Vision arming");
    fixture.runtime.publishCameraFrame(makeFrame(400));
    fixture.releaseBlockedQuery();
    ok &= expectTrue(
        waitUntil([&fixture]() {
            return std::any_of(
                fixture.trackingResults.begin(),
                fixture.trackingResults.end(),
                [](const ui::RuntimeTrackingResult &result) {
                    return result.frameId == 400;
                });
        }, 5000),
        "restarted session should route its own frame after old query returns");
    ok &= expectTrue(
        std::none_of(
            fixture.trackingResults.begin(),
            fixture.trackingResults.end(),
            [](const ui::RuntimeTrackingResult &result) {
                return result.frameId == 300;
            }),
        "late RobotSim/Vision result from the stopped session must not route");
    fixture.stop();
    return ok;
}

bool testStopAndRestartRejectsLateVisionResult(TestMode mode)
{
    CorrectionFixture fixture(mode);
    auto visionState = std::make_shared<BlockingSynchronizedVisionState>();
    fixture.runtime.setSimulatedCallbacks(
        makeBlockingSynchronizedVisionCallbacks(visionState));
    fixture.start(true, true);
    fixture.runtime.publishCameraFrame(makeFrame(500));
    bool ok = expectTrue(
        waitForBlockedVision(visionState),
        "old-session Vision should enter the deterministic synchronization hook");

    fixture.restartCorrection();
    // 旧 Vision 调用阻塞其线程；先解除旧调用，再等新会话真正 armed。
    releaseBlockedVision(visionState);
    ok &= expectTrue(fixture.waitForDetection(),
        "restarted correction must arm after the old Vision call returns");
    fixture.runtime.publishCameraFrame(makeFrame(600));
    ok &= expectTrue(
        waitUntil([&fixture]() {
            return std::any_of(
                fixture.trackingResults.begin(),
                fixture.trackingResults.end(),
                [](const ui::RuntimeTrackingResult &result) {
                    return result.frameId == 600;
                });
        }, 5000),
        "new session should route after the blocked old Vision call returns");
    ok &= expectTrue(
        std::none_of(
            fixture.trackingResults.begin(),
            fixture.trackingResults.end(),
            [](const ui::RuntimeTrackingResult &result) {
                return result.frameId == 500;
            }),
        "late Vision result from the stopped session must not enter Control");
    releaseBlockedVision(visionState);
    fixture.stop();
    return ok;
}

bool testCorrectionPausesAndResumesOrdinaryRealtimeSync(TestMode mode)
{
    CorrectionFixture fixture(mode);
    fixture.start(true, true);
    const int before = fixture.queryCount();
    for (int index = 0; index < 4; ++index) {
        auto state = makeReachableState();
        state.curS1 = index * 100;
        fixture.runtime.publishRobotState(state);
    }
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < 100) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(2);
    }
    bool ok = expectTrue(
        fixture.queryCount() == before,
        "active correction must pause ordinary realtime RobotSim queries");
    fixture.resetCorrection();
    ok &= expectTrue(
        waitUntil([&fixture, before]() {
            return fixture.queryCount() > before;
        }),
        "correction stop should resume realtime synchronization from the latest RobotState");
    fixture.stop();
    return ok;
}

bool testOnlineNoTargetIsSynchronizationSuccess()
{
    CorrectionFixture fixture(TestMode::Online);
    fixture.start(true, true);
    auto noTargetState = makeReachableState();
    // 初始扫描仍需STARTDETPRG窗口；无目标由空白图表示，不伪造窗口关闭。
    noTargetState.traflag = 1;
    fixture.runtime.publishRobotState(noTargetState);
    fixture.runtime.publishCameraFrame(makeBlankFrame(800));
    const bool consumed = waitUntil([&fixture]() {
        return std::any_of(
            fixture.syncUpdates.begin(),
            fixture.syncUpdates.end(),
            [](const ui::CorrectionSyncUpdate &update) {
                return update.frameId == 800
                    && update.state == ui::CorrectionSyncState::VisionConsumed
                    && update.consecutiveSyncFailures == 0;
            });
    });
    const bool noTargetResult = std::any_of(
        fixture.trackingResults.begin(),
        fixture.trackingResults.end(),
        [](const ui::RuntimeTrackingResult &result) {
            return result.frameId == 800 && !result.valid
                && result.imageMeasurementOutputEnabled && !result.imageObservationAccepted
                && !result.sample.point.valid
                && !result.independentWeldPointInParent_mm.valid
                && !result.detectedRobotBasePoint.valid && !result.detectedStationBasePoint.valid
                && result.pointSource == i2r_rlc::vision::OnlineSeamPointSource::Unavailable;
        });
    fixture.stop();
    return expectTrue(
               consumed,
               "online no-target must consume a synchronized frame without counting a failure")
        && expectTrue(
               noTargetResult,
               "online blank frame succeeds in synchronization but cannot manufacture a CAD or historical measurement");
}

bool testOnlineInternalPauseKeepsSession()
{
    CorrectionFixture fixture(TestMode::Online);
    fixture.start(true, true, true);
    fixture.runtime.publishCameraFrame(makeFrame(810));
    bool ok = expectTrue(
        waitUntil([&fixture]() { return !fixture.trackingResults.empty(); }),
        "online initial detection should consume its synchronized frame");
    fixture.runtime.publishCameraFrame(makeFrame(811));
    ok &= expectTrue(waitUntil([&fixture]() { return fixture.trackingResults.size() >= 2U; }),
        "second fresh exposure confirms initial geometry");
    if (fixture.trackingResults.size() >= 2U) {
        const auto &initial = fixture.trackingResults.back();
        ok &= expectTrue(initial.valid && initial.sample.point.valid,
            "online pause fixture must provide a valid initial weld point");
        if (!initial.valid || !initial.sample.point.valid) {
            std::cerr << "pause fixture initial frame=" << initial.frameId
                      << " trigger=" << initial.triggerAccepted
                      << " valid=" << initial.valid << " message="
                      << initial.message.toStdString() << '\n';
        }
    }

    auto reverseComplete = makeReachableState();
    reverseComplete.onlineReverseCompleteFlag = 1;
    fixture.runtime.publishRobotState(reverseComplete);
    ok &= expectTrue(waitUntil([&fixture]() {
        return std::any_of(fixture.onlineUpdates.begin(), fixture.onlineUpdates.end(),
            [](const i2r_rlc::control::OnlineCorrectionUpdate &update) {
                return update.phase
                    == i2r_rlc::control::OnlineCorrectionPhase::WaitingForInitialApproach;
            });
    }, 5000), "online O acknowledgement must wait for the P001 approach");
    auto approachComplete = makeReachableState();
    approachComplete.corflag = 2;
    approachComplete.detflag = 2;
    fixture.runtime.publishRobotState(approachComplete);
    ok &= expectTrue(waitUntil([&fixture]() {
        return std::any_of(fixture.onlineUpdates.begin(), fixture.onlineUpdates.end(),
            [](const i2r_rlc::control::OnlineCorrectionUpdate &update) {
                return update.phase
                    == i2r_rlc::control::OnlineCorrectionPhase::WaitingForOnlineStart;
            });
    }), "P001 ENDDET must precede the online STARTCOR window");
    auto onlineStart = makeReachableState();
    onlineStart.corflag = 1;
    onlineStart.detflag = 1;
    fixture.runtime.publishRobotState(onlineStart);
    const bool resumed = waitUntil([&fixture]() {
            return std::any_of(
                fixture.onlineUpdates.begin(),
                fixture.onlineUpdates.end(),
                [](const i2r_rlc::control::OnlineCorrectionUpdate &update) {
                    return update.phase
                        == i2r_rlc::control::OnlineCorrectionPhase::OnlineRunning;
                });
        }, 5000);
    ok &= expectTrue(
        resumed,
        "online O acknowledgement and STARTCOR should resume Vision in the same run");
    if (!resumed) {
        for (const auto &update : fixture.onlineUpdates) {
            std::cerr << "pause fixture phase=" << static_cast<int>(update.phase)
                      << " success=" << update.succeeded << " message="
                      << update.message << '\n';
        }
        std::lock_guard<std::mutex> lock(fixture.robot->mutex);
        for (const auto command : fixture.robot->commands) {
            std::cerr << "pause fixture command=" << static_cast<int>(command) << '\n';
        }
    }

    fixture.runtime.publishCameraFrame(makeFrame(812));
    ok &= expectTrue(
        waitUntil([&fixture]() { return fixture.trackingResults.size() >= 3U; }),
        "online Vision should consume a frame after its internal pause");
    if (fixture.trackingResults.size() >= 2U) {
        ok &= expectTrue(
            fixture.trackingResults[0].correlation.sessionId
                == fixture.trackingResults[1].correlation.sessionId,
            "online internal pause/resume must retain the correction sessionId");
    }
    fixture.stop();
    return ok;
}

} // namespace

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    TestMode mode = TestMode::Offline;
#if defined(I2R_RLC_FRAME_SYNC_DEFAULT_ONLINE)
    mode = TestMode::Online;
#endif
    bool explicitOffline = false;
    bool explicitOnline = false;
    for (int index = 1; index < argc; ++index) {
        explicitOffline |= std::string_view(argv[index]) == "--offline";
        explicitOnline |= std::string_view(argv[index]) == "--online";
    }
    if (explicitOffline && explicitOnline) {
        std::cerr << "FAIL: choose exactly one of --offline or --online\n";
        return 2;
    }
    if (explicitOnline) {
        mode = TestMode::Online;
    } else if (explicitOffline) {
        mode = TestMode::Offline;
    }

    bool ok = true;
    ok &= testNoRobotStateDoesNotQuery(mode);
    ok &= testLatestPendingAndSuccessfulCorrelation(mode);
    ok &= testFailuresResetThenThirdFailureStops(mode);
    ok &= testTransportFailureAndWrongFrameDoNotReuseObservation(mode);
    ok &= testStopAndRestartRejectsLateQueryAndVision(mode);
    ok &= testStopAndRestartRejectsLateVisionResult(mode);
    ok &= testCorrectionPausesAndResumesOrdinaryRealtimeSync(mode);
    if (mode == TestMode::Online) {
        ok &= testOnlineNoTargetIsSynchronizationSuccess();
        ok &= testOnlineInternalPauseKeepsSession();
    }
    if (!ok) {
        return 1;
    }

    std::cout
        << "PASS: "
        << (mode == TestMode::Online ? "online" : "offline")
        << " correction RobotSim frame sync tests; "
        << "frame mismatch=0, max current=1, max pending=1, "
        << "post-stop query=0, old-session route=0, stale observation reuse=0\n";
    return 0;
}
