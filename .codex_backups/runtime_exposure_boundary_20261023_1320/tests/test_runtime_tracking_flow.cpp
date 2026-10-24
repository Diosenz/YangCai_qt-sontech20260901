/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_runtime_tracking_flow.cpp
 * @brief       runtime tracking flow 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string_view>
#include <string>
#include <vector>

#include <QColor>
#include <QCoreApplication>
#include <QDateTime>
#include <QEventLoop>
#include <QJsonDocument>
#include <QTimer>

#include <Eigen/Geometry>

#include "geometry/WeldImageToSpaceProjector.h"
#include "track/LegacyOnlineKcfDetector.h"

#include "CalibrationGuideOverlay.h"
#include "RuntimeCoordinator.h"
#include "RuntimeWorkerAdapters.h"

namespace {

bool expectTrue(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

/** @brief 非零畸变和完整非单位外参验证，防止过渡目标在位姿切换时被错投。 */
bool testSpacePointInverseProjection()
{
    using namespace i2r_rlc::vision;
    WeldProjectionContext context;
    context.valid = true;
    auto &calibration = context.calibration;
    calibration.valid = true;
    calibration.focalLengthM = 0.012;
    calibration.sxM = 0.000007;
    calibration.syM = 0.000009;
    calibration.kappa = 1200.0;
    calibration.intrinsics.cxPixel = 640.0;
    calibration.intrinsics.cyPixel = 512.0;
    calibration.laserPlaneA = 0.02;
    calibration.laserPlaneB = -0.03;
    calibration.laserPlaneC = -1.0;
    calibration.cameraInTool.rotation = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    calibration.cameraInTool.translationMm = {30.0, -20.0, 10.0};
    context.toolInRobotBase.rotation = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    context.toolInRobotBase.translationMm = {100.0, 200.0, -40.0};
    context.robotBaseToPositionerTcp.rotation = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    context.robotBaseToPositionerTcp.translationMm = {-400.0, 90.0, 20.0};
    bool ok = true;
    const WeldImagePoint pixel{820.0, 430.0, true};
    for (const auto frame : {WeldProjectionTargetFrame::Camera, WeldProjectionTargetFrame::Tool,
             WeldProjectionTargetFrame::RobotBase, WeldProjectionTargetFrame::PositionerTcp}) {
        const auto space = CWeldImageToSpaceProjector::projectImagePoint(pixel, context, frame);
        const auto recovered = CWeldImageToSpaceProjector::projectSpacePointToImage(space.spacePoint, context, frame);
        ok &= expectTrue(space.valid && recovered.valid
                && std::hypot(recovered.xPixel - pixel.xPixel, recovered.yPixel - pixel.yPixel) < 1.0e-6,
            "division distortion inverse and full rigid chain preserve pixel roundtrip in every frame");
    }
    ok &= expectTrue(!CWeldImageToSpaceProjector::projectSpacePointToImage(
            {0.0, 0.0, -1000.0, true}, context, WeldProjectionTargetFrame::Camera).valid,
        "inverse projection rejects a point behind the camera");
    calibration.kappa = 1.0e9;
    ok &= expectTrue(!CWeldImageToSpaceProjector::projectSpacePointToImage(
            {1000.0, 1000.0, 1000.0, true}, context, WeldProjectionTargetFrame::Camera).valid,
        "division inverse rejects a radial point without a real principal-branch solution");
    return ok;
}

i2r_rlc::camera::CameraFrame makeFrame()
{
    i2r_rlc::camera::CameraFrame frame;
    frame.frameId = 1;
    frame.image = QImage(320, 240, QImage::Format_Grayscale8);
    frame.image.fill(0);
    for (int y = 116; y <= 124; ++y) {
        uchar *row = frame.image.scanLine(y);
        std::fill(row + 60, row + 260, static_cast<uchar>(255));
    }
    frame.previewImage = frame.image;
    return frame;
}

/** @brief 真实中心坐标已知的接缝夹具，用于正式在线几何校核。 */
i2r_rlc::camera::CameraFrame makeOnlineSeamFrame()
{
    auto frame = makeFrame();
    for (int y = 0; y < frame.image.height(); ++y) {
        auto *row = frame.image.scanLine(y);
        for (int x = 0; x < frame.image.width(); ++x) {
            const double dx = x - 160.0;
            const double centerY = 120.0 + (dx < 0.0 ? 0.28 : -0.32) * dx;
            const double dy = y - centerY;
            row[x] = static_cast<uchar>(15.0 + 220.0 * std::exp(-dy * dy / 2.88));
        }
    }
    frame.previewImage = frame.image;
    return frame;
}

i2r_rlc::robot_comm::RobotState makeReadyRobotState()
{
    i2r_rlc::robot_comm::RobotState state;
    state.traflag = 1;
    state.sMode = 2;
    state.sHold = 0;
    state.sStart = 1;
    return state;
}

i2r_rlc::robot_comm::RobotState makeRobotSimReachableState()
{
    auto state = makeReadyRobotState();
    // 此 TCP 由 MH12 内部关节 [0, pi/2, 0, 0, 0, 0] 经固定工具
    // 标定正向构造，确保解析逆解存在。协议单位分别为 um 和 0.0001 deg。
    state.curx_r = 1389425;
    state.cury_r = -4882;
    state.curz_r = 688864;
    state.curRx_r = -1791754;
    state.curRy_r = -551313;
    state.curRz_r = -2478;
    return state;
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

struct FakeTransportState {
    std::vector<i2r_rlc::robot_comm::RobotCommandType> types;
    std::vector<std::string> requests;
    std::vector<int> teachPointIndices;
    std::vector<int> teachPointOfflineValues;
};

class CFakeRobotTransport final
    : public i2r_rlc::robot_comm::IRobotCommandTransport {
public:
    explicit CFakeRobotTransport(
        std::shared_ptr<FakeTransportState> state =
            std::make_shared<FakeTransportState>())
        : m_record(std::move(state))
    {
    }

    bool connect(std::string_view, std::uint16_t) override
    {
        m_connected = true;
        return true;
    }

    void disconnect() override
    {
        m_connected = false;
    }

    bool isConnected() const override
    {
        return m_connected;
    }

    i2r_rlc::robot_comm::RobotCommandReceipt transact(
        i2r_rlc::robot_comm::RobotCommandType type,
        const std::string &request,
        int pointIndex,
        i2r_rlc::robot_comm::RobotState *state) override
    {
        i2r_rlc::robot_comm::RobotCommandReceipt receipt;
        receipt.type = type;
        receipt.sent = true;
        receipt.responseReceived = true;
        receipt.completed = true;
        receipt.pointIndex = pointIndex;
        receipt.request = request;
        receipt.response =
            "D 1,2,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
        receipt.message = "fake robot response";
        m_record->types.push_back(type);
        m_record->requests.push_back(request);
        if (state != nullptr) {
            const auto before = *state;
            *state = makeReadyRobotState();
            if (type == i2r_rlc::robot_comm::RobotCommandType::TeachPointUpdate) {
                const auto fields = QString::fromStdString(request).split(',');
                const int pointCount = fields.size() > 8 ? fields[8].toInt() : 0;
                state->offline = pointIndex + 1 == pointCount ? 5 : 4;
                receipt.completed = state->offline == 5;
                m_record->teachPointIndices.push_back(pointIndex);
                m_record->teachPointOfflineValues.push_back(state->offline);
            } else if (type == i2r_rlc::robot_comm::RobotCommandType::RealtimeCorrection) {
                // P 回包不代替控制柜 ENDDET/程序停止边沿，由闭环夹具显式投递。
                *state = before;
            }
        }
        return receipt;
    }

private:
    bool m_connected = false;
    std::shared_ptr<FakeTransportState> m_record;
};

/**
 * @brief 跨线程记录 RobotSim transport 是否由 runtime 的注入路径实际使用。
 *
 * 仅保存原子计数，不把 worker 线程中的 QJsonDocument 引用泄漏回测试线程。
 */
struct FakeRobotSimTransportState {
    std::atomic<int> connectCount{0};
    std::atomic<int> sendCount{0};
    bool centeredObservation = false;
};

class CFakeRobotSimTransport final
    : public i2r_rlc::digital_twin::IRobotSimTransport {
public:
    explicit CFakeRobotSimTransport(
        std::shared_ptr<FakeRobotSimTransportState> state)
        : m_state(std::move(state))
    {
    }

    i2r_rlc::digital_twin::RobotSimTransportResult connect(
        const i2r_rlc::digital_twin::RobotSimEndpointConfig &endpoint) override
    {
        ++m_state->connectCount;
        m_connected = endpoint.port > 0;
        return {
            m_connected,
            {},
            m_connected
                ? QStringLiteral("fake RobotSim connected")
                : QStringLiteral("fake RobotSim endpoint invalid"),
            0
        };
    }

    void disconnect() override
    {
        m_connected = false;
    }

    bool isConnected() const override
    {
        return m_connected;
    }

    i2r_rlc::digital_twin::RobotSimTransportResult send(
        const QJsonDocument &,
        bool expectResponse) override
    {
        ++m_state->sendCount;
        return {
            true,
            expectResponse
                ? (m_state->centeredObservation
                    ? QStringLiteral("[[[[[1,2,3],[60,92]],[[2,3,4],[160,120]],[[4,5,6],[260,88]]]],[[160,120]]]")
                    : QStringLiteral(
                    "[[[[[1,2,3],[10,20]],[[4,5,6],[11,21]]]],[[12,22]]]")
                  )
                : QString(),
            QStringLiteral("fake RobotSim send ok"),
            0
        };
    }

private:
    std::shared_ptr<FakeRobotSimTransportState> m_state;
    bool m_connected = false;
};

/**
 * @brief 控制首个 Vision 回调的完成时机，用于稳定构造在途帧和待处理帧。
 */
struct BlockingVisionCallbackState {
    std::mutex mutex;
    std::condition_variable condition;
    int callCount = 0;
    bool firstCallEntered = false;
    bool releaseFirstCall = false;
};

i2r_rlc::ui::SimulatedVisionCallbacks makeBlockingVisionCallbacks(
    const std::shared_ptr<BlockingVisionCallbackState> &state)
{
    i2r_rlc::ui::SimulatedVisionCallbacks callbacks;
    const auto blockFirstCall = [state]() {
        std::unique_lock<std::mutex> lock(state->mutex);
        ++state->callCount;
        if (state->callCount == 1) {
            state->firstCallEntered = true;
            state->condition.notify_all();
            state->condition.wait_for(
                lock,
                std::chrono::seconds(3),
                [state]() { return state->releaseFirstCall; });
        }
    };
    // 正式纠偏走 synchronized observation 钩子；legacy/global 测试仍可能走
    // generate_simu_laser1。两者共用一个计数器，确保只阻塞首个 Vision 调用。
    callbacks.beforeSynchronizedFrame =
        [blockFirstCall](qint64) { blockFirstCall(); };
    callbacks.generate_simu_laser1 = [blockFirstCall]() {
        blockFirstCall();
        return false;
    };
    return callbacks;
}

void releaseFirstVisionCall(
    const std::shared_ptr<BlockingVisionCallbackState> &state)
{
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->releaseFirstCall = true;
    }
    state->condition.notify_all();
}

std::filesystem::path makeOnlineConfigurationFiles(
    i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration *configuration)
{
    const auto tempDir =
        std::filesystem::temp_directory_path() / "i2r_rlc_runtime_tracking_flow";
    std::filesystem::create_directories(tempDir);
    const auto workPath = tempDir / "case_W.txt";
    const auto teachPath = tempDir / "case_T.txt";
    const auto rbcalibPath = tempDir / "RBCALIB.DAT";
    std::ofstream(workPath) << "0 0 0\n100 0 0\n";
    std::ofstream(teachPath)
        << "0 0 0 0 0 0 0 0\n100 0 0 0 0 0 0 0\n";
    std::ofstream(rbcalibPath) << "////SRANG 0,0,0,0,0,0\n";

    configuration->trajectoryRequest.workPathFile = workPath.string();
    configuration->trajectoryRequest.teachPoseFile = teachPath.string();
    configuration->rbcalibFile = rbcalibPath.string();
    return tempDir;
}

bool connectCorrectionRobotSim(
    i2r_rlc::ui::CRuntimeCoordinator &runtime,
    QEventLoop &loop)
{
    bool synchronized = false;
    const QMetaObject::Connection connection = QObject::connect(
        &runtime,
        &i2r_rlc::ui::CRuntimeCoordinator::robotSimStateSynchronizationChanged,
        &loop,
        [&synchronized, &loop](bool succeeded, const QString &) {
            if (succeeded) {
                synchronized = true;
                loop.quit();
            }
        });
    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    i2r_rlc::digital_twin::RobotSimSceneConfig scene;
    scene.sceneName = QStringLiteral("runtime-correction-frame-sync");
    scene.robotId = 1001;
    scene.positionerId = 1002;
    scene.sensorId = 1013;
    runtime.connectRobotSim(endpoint, scene);
    QTimer::singleShot(2000, &loop, &QEventLoop::quit);
    loop.exec();
    QObject::disconnect(connection);
    return synchronized;
}

bool testInjectedRobotSimTransportIsUsed(QCoreApplication &app)
{
    Q_UNUSED(app);
    QEventLoop loop;
    auto robotSimState = std::make_shared<FakeRobotSimTransportState>();
    i2r_rlc::ui::CRuntimeCoordinator runtime(
        std::make_unique<CFakeRobotTransport>(),
        std::make_unique<CFakeRobotSimTransport>(robotSimState));
    bool connected = false;
    QObject::connect(
        &runtime,
        &i2r_rlc::ui::CRuntimeCoordinator::robotSimManualConnectionStateChanged,
        &loop,
        [&connected, &loop](bool value, const QString &) {
            connected = value;
            loop.quit();
        });

    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    i2r_rlc::digital_twin::RobotSimSceneConfig scene;
    scene.sceneName = QStringLiteral("runtime-test-scene");
    scene.sensorId = 1013;

    runtime.start();
    runtime.connectRobotSimManual(endpoint, scene);
    QTimer::singleShot(2000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();

    bool ok = true;
    ok &= expectTrue(connected,
                     "injected RobotSim transport should establish manual test connection");
    ok &= expectTrue(
        robotSimState->connectCount.load() == 1,
        "runtime should transfer the injected RobotSim transport to its worker");
    ok &= expectTrue(
        robotSimState->sendCount.load() >= 1,
        "injected RobotSim transport should receive the virtual-reference request");
    return ok;
}

bool testCorrectionSessionPausesAndResumesRealtimeRobotSim(
    QCoreApplication &app)
{
    Q_UNUSED(app);
    QEventLoop loop;
    auto robotSimState = std::make_shared<FakeRobotSimTransportState>();
    i2r_rlc::ui::CRuntimeCoordinator runtime(
        std::make_unique<CFakeRobotTransport>(),
        std::make_unique<CFakeRobotSimTransport>(robotSimState));
    i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);

    bool connected = false;
    bool correctionStarted = false;
    int sendsBeforeCorrection = -1;
    int sendsDuringCorrection = -1;
    int sendsAfterStop = -1;
    QObject::connect(
        &runtime,
        &i2r_rlc::ui::CRuntimeCoordinator::robotSimConnectionStateChanged,
        &loop,
        [&connected](bool value, const QString &) {
            connected = value;
        });
    QObject::connect(
        &runtime,
        &i2r_rlc::ui::CRuntimeCoordinator::robotSimStateSynchronizationChanged,
        &loop,
        [&](bool succeeded, const QString &) {
            if (!succeeded || correctionStarted) {
                return;
            }
            correctionStarted = true;
            sendsBeforeCorrection = robotSimState->sendCount.load();
            runtime.startOnlineTracking();
            for (int index = 0; index < 4; ++index) {
                auto state = makeRobotSimReachableState();
                state.curS1 = index * 100;
                runtime.publishRobotState(state);
            }
            QTimer::singleShot(350, &loop, [&]() {
                sendsDuringCorrection = robotSimState->sendCount.load();
                runtime.stopOnlineTracking();
            });
            QTimer::singleShot(800, &loop, [&]() {
                sendsAfterStop = robotSimState->sendCount.load();
                loop.quit();
            });
        });

    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    i2r_rlc::digital_twin::RobotSimSceneConfig scene;
    scene.sceneName = QStringLiteral("runtime-test-scene");
    scene.robotId = 1001;
    scene.positionerId = 1002;
    scene.sensorId = 1013;

    runtime.start();
    runtime.configureOnlineCorrection(configuration);
    runtime.publishRobotState(makeRobotSimReachableState());
    runtime.connectRobotSim(endpoint, scene);
    QTimer::singleShot(4000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();

    bool ok = true;
    ok &= expectTrue(connected && correctionStarted,
                     "real-scene fake RobotSim should reach realtime synchronization");
    ok &= expectTrue(
        sendsBeforeCorrection >= 2,
        "connection and initial realtime synchronization should use fake transport");
    ok &= expectTrue(
        sendsDuringCorrection == sendsBeforeCorrection,
        "active correction session should pause ordinary realtime RobotSim sends");
    ok &= expectTrue(
        sendsAfterStop > sendsDuringCorrection,
        "correction stop should resume RobotSim from the latest cached RobotState");
    return ok;
}

bool testCorrectionTrackingKeepsOnlyLatestPendingFrame(QCoreApplication &app)
{
    Q_UNUSED(app);
    using i2r_rlc::ui::CRuntimeCoordinator;
    using i2r_rlc::ui::RuntimeTrackingResult;

    QEventLoop loop;
    auto callbackState = std::make_shared<BlockingVisionCallbackState>();
    auto robotSimState = std::make_shared<FakeRobotSimTransportState>();
    CRuntimeCoordinator runtime(
        std::make_unique<CFakeRobotTransport>(),
        std::make_unique<CFakeRobotSimTransport>(robotSimState));
    runtime.setSimulatedCallbacks(makeBlockingVisionCallbacks(callbackState));

    std::vector<RuntimeTrackingResult> results;
    QObject::connect(
        &runtime,
        &CRuntimeCoordinator::trackingResultReady,
        &loop,
        [&results, &loop](const RuntimeTrackingResult &result) {
            results.push_back(result);
            if (results.size() == 2U) {
                loop.quit();
            }
        });

    i2r_rlc::ui::RuntimeTrackingOptions options;
    i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.configureOnlineCorrection(configuration);
    runtime.publishRobotState(makeRobotSimReachableState());
    const bool robotSimSynchronized = connectCorrectionRobotSim(runtime, loop);
    runtime.startOnlineTracking();

    QTimer::singleShot(100, &loop, [&runtime]() {
        auto frame = makeFrame();
        frame.frameId = 100;
        runtime.publishCameraFrame(frame);
    });
    QTimer::singleShot(250, &loop, [&runtime]() {
        for (qint64 frameId = 101; frameId <= 105; ++frameId) {
            auto frame = makeFrame();
            frame.frameId = frameId;
            runtime.publishCameraFrame(frame);
        }
    });
    QTimer::singleShot(350, &loop, [callbackState]() {
        releaseFirstVisionCall(callbackState);
    });
    QTimer::singleShot(3500, &loop, &QEventLoop::quit);
    loop.exec();
    releaseFirstVisionCall(callbackState);
    runtime.stop();

    bool firstCallEntered = false;
    {
        std::lock_guard<std::mutex> lock(callbackState->mutex);
        firstCallEntered = callbackState->firstCallEntered;
    }
    bool ok = true;
    ok &= expectTrue(
        robotSimSynchronized,
        "correction backpressure test requires real-scene RobotSim synchronization");
    ok &= expectTrue(firstCallEntered,
                     "test should hold the first Vision frame in flight");
    ok &= expectTrue(
        results.size() == 2U,
        "correction flow should keep one current and one latest pending frame");
    if (results.size() == 2U) {
        ok &= expectTrue(
            results[0].frameId == 100 && results[1].frameId == 105,
            "frames 101-104 should be replaced by latest pending frame 105");
        ok &= expectTrue(
            results[0].correlation.sessionId != 0
                && results[0].correlation.sessionId
                    == results[1].correlation.sessionId,
            "both processed frames should retain the same nonzero correction session");
        ok &= expectTrue(
            results[0].correlation.purpose
                    == i2r_rlc::ui::CorrectionPurpose::OnlineCorrection
                && results[1].correlation.purpose
                    == i2r_rlc::ui::CorrectionPurpose::OnlineCorrection
                && results[0].correlation.frameId == results[0].frameId
                && results[1].correlation.frameId == results[1].frameId,
            "Vision correlation should preserve purpose and each frozen frameId");
        for (const auto &result : results) {
            ok &= expectTrue(result.diagnostic.has_value() && result.frameContext.has_value(),
                             "online results must retain frozen diagnostic inputs");
            if (result.diagnostic.has_value() && result.frameContext.has_value()) {
                ok &= expectTrue(result.diagnostic->cameraTimestampMs
                                     == result.frameContext->sourceFrame.timestamp.toMSecsSinceEpoch()
                                 && result.diagnostic->robotStateReceivedMs > 0,
                                 "diagnostic timestamps must belong to the displayed frozen frame");
                ok &= expectTrue(result.diagnostic->sourceImage.isNull()
                                 && result.diagnostic->enhancedImage.isNull(),
                                 "diagnostic image payload must not be sent to the display consumer");
            }
        }
    }
    return ok;
}

bool testLatestRealtimeCommandWins(QCoreApplication &app)
{
    auto record = std::make_shared<FakeTransportState>();
    i2r_rlc::ui::CRobotCommWorkerAdapter robot(
        std::make_unique<CFakeRobotTransport>(record));
    i2r_rlc::robot_comm::RobotCommunicationDiagnostics diagnostics;
    QObject::connect(
        &robot,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::diagnosticsReady,
        &app,
        [&diagnostics](
            const i2r_rlc::robot_comm::RobotCommunicationDiagnostics &value) {
            diagnostics = value;
        });
    robot.connectRobot();

    i2r_rlc::robot_comm::RobotRealtimeCorrectionCommand command;
    command.valid = true;
    command.dxUm = 111;
    robot.sendRealtimeCorrectionCommand(command);
    command.dxUm = 222;
    robot.sendRealtimeCorrectionCommand(command);
    command.dxUm = 333;
    robot.sendRealtimeCorrectionCommand(command);
    app.processEvents(QEventLoop::AllEvents, 100);
    robot.disconnectRobot();

    int realtimeCount = 0;
    std::string realtimeRequest;
    for (std::size_t index = 0; index < record->types.size(); ++index) {
        if (record->types[index]
            == i2r_rlc::robot_comm::RobotCommandType::RealtimeCorrection) {
            ++realtimeCount;
            realtimeRequest = record->requests[index];
        }
    }

    bool ok = true;
    ok &= expectTrue(realtimeCount == 1,
                     "queued realtime corrections should coalesce to one P request");
    ok &= expectTrue(
        realtimeRequest.find(",333,0,0,0,0,0,10000,0") != std::string::npos,
        "coalesced P request should contain the latest correction");
    ok &= expectTrue(
        diagnostics.latestRealtimeRequest == realtimeRequest
            && !diagnostics.latestResponse.empty()
            && diagnostics.transactionCount >= 2,
        "diagnostics should retain actual P/D text and transaction count");
    return ok;
}

bool testWorkerClosedLoop(QCoreApplication &app)
{
    using i2r_rlc::control::OnlineCorrectionPhase;
    using i2r_rlc::robot_comm::RobotCommandType;
    auto record = std::make_shared<FakeTransportState>();
    i2r_rlc::ui::CRobotCommWorkerAdapter robot(
        std::make_unique<CFakeRobotTransport>(record));
    i2r_rlc::ui::COnlineCorrectionWorkerAdapter control;
    i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    {
        // 保留真实异常处理器；参考和输入都须跨过其五个下采样点初始化门槛。
        std::ofstream workPath(configuration.trajectoryRequest.workPathFile);
        for (int index = 0; index <= 10; ++index) {
            workPath << index * 10 << " 0 0\n";
        }
    }

    std::vector<OnlineCorrectionPhase> phases;
    bool attemptedOnlineIcp = false;
    QObject::connect(
        &control,
        &i2r_rlc::ui::COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
        &app,
        [&phases, &attemptedOnlineIcp](const i2r_rlc::control::OnlineCorrectionUpdate &update) {
            phases.push_back(update.phase);
            for (const auto &batch : update.onlineIcpBatches) {
                attemptedOnlineIcp |= batch.attempted;
            }
        });
    QObject::connect(
        &control,
        &i2r_rlc::ui::COnlineCorrectionWorkerAdapter::robotCommandReady,
        &robot,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::sendCorrectionCommand);
    QObject::connect(
        &control,
        &i2r_rlc::ui::COnlineCorrectionWorkerAdapter::realtimeCommandReady,
        &robot,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::sendRealtimeCorrectionCommand);
    QObject::connect(
        &robot,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::robotStateReady,
        &control,
        &i2r_rlc::ui::COnlineCorrectionWorkerAdapter::updateRobotState);
    QObject::connect(
        &robot,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::commandReceiptReady,
        &control,
        &i2r_rlc::ui::COnlineCorrectionWorkerAdapter::handleRobotCommandReceipt);

    robot.connectRobot();
    control.configure(configuration);
    control.startInitialDetection();
    i2r_rlc::ui::RuntimeTrackingResult initial;
    initial.valid = true;
    initial.sample.point = {10.0, 0.0, 0.0, true};
    initial.message = QStringLiteral("initial detection");
    control.appendTrackingResult(initial);
    auto state = makeReadyRobotState();
    state.onlineReverseCompleteFlag = 1;
    robot.updateState(state);

    bool ok = expectTrue(!phases.empty()
            && phases.back() == OnlineCorrectionPhase::WaitingForInitialApproach,
        "O=4/5 acknowledgements must wait for the robot P001 approach");
    state.corflag = 2;
    state.detflag = 2;
    robot.updateState(state);
    ok &= expectTrue(phases.back() == OnlineCorrectionPhase::WaitingForOnlineStart,
        "P001 ENDDET must wait for the subsequent STARTCOR");
    state.corflag = 1;
    state.detflag = 1;
    robot.updateState(state);
    ok &= expectTrue(phases.back() == OnlineCorrectionPhase::OnlineRunning,
        "STARTCOR after P001 ENDDET must enter OnlineRunning");

    for (int index = 0; index < 16; ++index) {
        i2r_rlc::ui::RuntimeTrackingResult online;
        online.valid = true;
        online.sample.point = {10.0 + static_cast<double>(index) * 5.0, 1.0, 0.0, true};
        online.simulatedSeamIndex = 0;
        online.message = QStringLiteral("online detection");
        control.appendTrackingResult(online);
    }
    app.processEvents(QEventLoop::AllEvents, 100);
    // 最终 ENDDET 关闭检测窗口，P 回包仍不能代替 detflag 清零和主程序停止。
    state.corflag = 3;
    state.detflag = 3;
    robot.updateState(state);
    app.processEvents(QEventLoop::AllEvents, 100);
    ok &= expectTrue(phases.back() != OnlineCorrectionPhase::Completed,
        "final P acknowledgement must keep the session alive before robot program stop");
    state.corflag = 0;
    state.detflag = 0;
    robot.updateState(state);
    ok &= expectTrue(phases.back() == OnlineCorrectionPhase::WaitingForRobotProgramStop,
        "final detection reset must wait for the running main program to stop");
    state.sStart = 0;
    robot.updateState(state);
    robot.disconnectRobot();

    int teachPointCount = 0;
    int realtimeCount = 0;
    bool ordinaryPFound = false;
    bool finalPFound = false;
    std::vector<qint64> teachXValues;
    for (std::size_t index = 0; index < record->types.size(); ++index) {
        const auto fields = QString::fromStdString(record->requests[index]).split(',');
        if (record->types[index] == RobotCommandType::TeachPointUpdate) {
            ++teachPointCount;
            if (fields.size() > 9) { teachXValues.push_back(fields[9].toLongLong()); }
        }
        if (record->types[index] == RobotCommandType::RealtimeCorrection && fields.size() > 8) {
            ++realtimeCount;
            ordinaryPFound |= fields[8] == QStringLiteral("0");
            finalPFound |= fields[8] == QStringLiteral("1")
                && record->requests[index].rfind("P 1,0,1,0,0,0,0,0,1,", 0) == 0;
        }
    }
    ok &= expectTrue(teachPointCount == 2
            && record->teachPointIndices == std::vector<int>{0, 1}
            && record->teachPointOfflineValues == std::vector<int>{4, 5}
            && teachXValues == std::vector<qint64>{10000, 110000},
        "closed loop must send P001/P002 in index order with the expected O coordinates and 4/5 receipts");
    ok &= expectTrue(attemptedOnlineIcp,
        "online samples must cross the real exception processor initialization threshold");
    ok &= expectTrue(realtimeCount >= 2 && ordinaryPFound && finalPFound,
        "closed loop must send both an ordinary P and a final P with Online_endflag=1");
    ok &= expectTrue(!phases.empty() && phases.back() == OnlineCorrectionPhase::Completed
            && std::find(phases.begin(), phases.end(), OnlineCorrectionPhase::Error) == phases.end(),
        "ENDDET, final P, detection reset and main program stop must complete without error");
    return ok;
}

bool testStoppedOnlineTrackingDropsLateResults(QCoreApplication &app)
{
    Q_UNUSED(app);
    using i2r_rlc::ui::CRuntimeCoordinator;
    using i2r_rlc::ui::RuntimeTrackingOptions;
    using i2r_rlc::ui::RuntimeTrackingResult;

    QEventLoop loop;
    auto callbackState = std::make_shared<BlockingVisionCallbackState>();
    auto robotSimState = std::make_shared<FakeRobotSimTransportState>();
    CRuntimeCoordinator runtime(
        std::make_unique<CFakeRobotTransport>(),
        std::make_unique<CFakeRobotSimTransport>(robotSimState));
    runtime.setSimulatedCallbacks(makeBlockingVisionCallbacks(callbackState));
    std::vector<RuntimeTrackingResult> results;
    std::vector<quint64> startedSessionIds;
    QObject::connect(
        &runtime,
        &CRuntimeCoordinator::trackingResultReady,
        &loop,
        [&results, &loop](const RuntimeTrackingResult &result) {
            results.push_back(result);
            loop.quit();
        });
    QObject::connect(
        &runtime,
        &CRuntimeCoordinator::correctionSyncUpdated,
        &loop,
        [&startedSessionIds](const i2r_rlc::ui::CorrectionSyncUpdate &update) {
            if (update.state == i2r_rlc::ui::CorrectionSyncState::Idle
                && update.sessionId != 0
                && update.purpose == i2r_rlc::ui::CorrectionPurpose::OnlineCorrection
                && std::find(startedSessionIds.begin(), startedSessionIds.end(), update.sessionId)
                    == startedSessionIds.end()) {
                // 会话建立和同会话窗口启动都可能发 Idle，不得把事件数当会话数。
                startedSessionIds.push_back(update.sessionId);
            }
        });

    RuntimeTrackingOptions options;
    i2r_rlc::ui::RuntimeOnlineCorrectionConfiguration onlineConfiguration;
    makeOnlineConfigurationFiles(&onlineConfiguration);

    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.configureOnlineCorrection(onlineConfiguration);
    runtime.publishRobotState(makeRobotSimReachableState());
    const bool robotSimSynchronized = connectCorrectionRobotSim(runtime, loop);
    runtime.startOnlineTracking();

    QTimer::singleShot(100, &loop, [&runtime]() {
        auto frame = makeFrame();
        frame.frameId = 100;
        runtime.publishCameraFrame(frame);
    });
    QTimer::singleShot(250, &loop, [&runtime]() {
        runtime.stopOnlineTracking();
        runtime.startOnlineTracking();
    });
    QTimer::singleShot(450, &loop, [&runtime]() {
        auto frame = makeFrame();
        frame.frameId = 200;
        runtime.publishCameraFrame(frame);
    });
    QTimer::singleShot(600, &loop, [callbackState]() {
        releaseFirstVisionCall(callbackState);
    });
    QTimer::singleShot(4000, &loop, &QEventLoop::quit);
    loop.exec();
    releaseFirstVisionCall(callbackState);
    runtime.stop();

    bool ok = true;
    ok &= expectTrue(
        robotSimSynchronized,
        "late-result test requires real-scene RobotSim synchronization");
    ok &= expectTrue(
        startedSessionIds.size() == 2U
            && startedSessionIds[0] != 0
            && startedSessionIds[1] > startedSessionIds[0],
        "stop and restart should create a newer nonzero correction session");
    ok &= expectTrue(
        results.size() == 1U && results.front().frameId == 200,
        "late result from stopped session must not release or route over new current frame");
    if (results.size() == 1U && startedSessionIds.size() >= 2U) {
        ok &= expectTrue(
            results.front().correlation.sessionId == startedSessionIds[1]
                && results.front().correlation.frameId == 200,
            "new current frame should retain the restarted session correlation");
    }
    return ok;
}

bool testGlobalLocalizationTrackingDoesNotFeedOnlineController(QCoreApplication &app)
{
    using i2r_rlc::control::TrackingVirtualStatus;
    using i2r_rlc::ui::CRuntimeCoordinator;
    using i2r_rlc::ui::RuntimeTrackingOptions;

    CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>());
    int trackingResultCount = 0;
    int onlineDetectionEventCount = 0;
    i2r_rlc::ui::RuntimeTrackingResult globalLocateResult;
    QObject::connect(
        &runtime,
        &CRuntimeCoordinator::trackingResultReady,
        &app,
        [&trackingResultCount, &globalLocateResult](
            const i2r_rlc::ui::RuntimeTrackingResult& result) {
            ++trackingResultCount;
            globalLocateResult = result;
            QCoreApplication::quit();
        });
    QObject::connect(
        &runtime,
        &CRuntimeCoordinator::onlineCorrectionUpdated,
        &app,
        [&onlineDetectionEventCount](const i2r_rlc::control::OnlineCorrectionUpdate &update) {
            if (update.message.find("detection event") != std::string::npos
                || !update.state.initialDetectionSamples.empty()
                || !update.state.onlineDetectionSamples.empty()) {
                ++onlineDetectionEventCount;
            }
        });

    RuntimeTrackingOptions options;
    options.detType = 2;
    options.virtualStatus = TrackingVirtualStatus::OnlineRecord;

    runtime.start();
    runtime.configureTracking(options, i2r_rlc::vision::WeldProjectionContext{});
    runtime.publishRobotState(makeReadyRobotState());
    runtime.startGlobalLocalizationTracking();
    QTimer::singleShot(100, &app, [&runtime]() {
        runtime.publishCameraFrame(makeFrame());
    });
    QTimer::singleShot(900, &app, &QCoreApplication::quit);
    app.exec();
    runtime.stop();

    bool ok = true;
    ok &= expectTrue(
        trackingResultCount == 1,
        "global localization tracking should emit tracking result to UI");
    ok &= expectTrue(
        onlineDetectionEventCount == 0,
        "global localization tracking must not feed online correction controller");
    ok &= expectTrue(
        globalLocateResult.message.contains(
            QStringLiteral("virtual laser prior is required")),
        "global localization must reject real-image extraction without the OCCT virtual prior");
    return ok;
}

bool testGlobalLocalizationFramesKeepCaptureTimeRobotSegment(QCoreApplication &app)
{
    using i2r_rlc::ui::CRuntimeCoordinator;
    using i2r_rlc::ui::RuntimeTrackingOptions;
    using i2r_rlc::ui::RuntimeTrackingResult;

    CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>());
    std::vector<RuntimeTrackingResult> results;
    QObject::connect(
        &runtime,
        &CRuntimeCoordinator::trackingResultReady,
        &app,
        [&](const RuntimeTrackingResult& result) {
            results.push_back(result);
            if (results.size() == 1U) {
                auto firstSegmentGapState = makeReadyRobotState();
                firstSegmentGapState.traflag = 0;
                firstSegmentGapState.posflag = 1;
                runtime.publishRobotState(firstSegmentGapState);
                auto firstSegmentGapFrame = makeFrame();
                firstSegmentGapFrame.frameId = 2;
                runtime.publishCameraFrame(firstSegmentGapFrame);
            } else if (results.size() == 2U) {
                auto firstSegmentNextWindowState = makeReadyRobotState();
                firstSegmentNextWindowState.posflag = 1;
                runtime.publishRobotState(firstSegmentNextWindowState);
                auto firstSegmentNextWindowFrame = makeFrame();
                firstSegmentNextWindowFrame.frameId = 3;
                runtime.publishCameraFrame(firstSegmentNextWindowFrame);
            } else if (results.size() == 3U) {
                auto gapState = makeReadyRobotState();
                gapState.traflag = 0;
                gapState.posflag = 0;
                runtime.publishRobotState(gapState);
                auto gapFrame = makeFrame();
                gapFrame.frameId = 4;
                runtime.publishCameraFrame(gapFrame);
            } else if (results.size() == 4U) {
                auto secondSegmentState = makeReadyRobotState();
                secondSegmentState.posflag = 1;
                runtime.publishRobotState(secondSegmentState);
                auto secondSegmentFrame = makeFrame();
                secondSegmentFrame.frameId = 5;
                runtime.publishCameraFrame(secondSegmentFrame);
            } else if (results.size() == 5U) {
                QCoreApplication::quit();
            }
        });

    RuntimeTrackingOptions options;
    options.detType = 2;
    runtime.start();
    runtime.configureTracking(options, i2r_rlc::vision::WeldProjectionContext{});
    runtime.startGlobalLocalizationTracking();
    auto firstSegmentState = makeReadyRobotState();
    firstSegmentState.posflag = 1;
    runtime.publishRobotState(firstSegmentState);
    auto firstSegmentFrame = makeFrame();
    firstSegmentFrame.frameId = 1;
    runtime.publishCameraFrame(firstSegmentFrame);
    QTimer::singleShot(3000, &app, &QCoreApplication::quit);
    app.exec();
    runtime.stop();

    bool ok = true;
    ok &= expectTrue(results.size() == 5U,
                     "global localization should process positioning and detection-window transitions");
    if (results.size() == 5U) {
        ok &= expectTrue(
            results[0].localizationSegmentIndex == 1
                && results[0].traflag == 1,
            "first scan frame must retain positioning segment 1 and traflag=1");
        ok &= expectTrue(
            results[1].localizationSegmentIndex == 1
                && results[1].traflag == 0
                && !results[1].triggerAccepted,
            "ENDDETPRG must close collection without ending positioning segment 1");
        ok &= expectTrue(
            results[2].localizationSegmentIndex == 1
                && results[2].traflag == 1,
            "the next STARTDETPRG must remain in positioning segment 1");
        ok &= expectTrue(
            results[3].localizationSegmentIndex == 0
                && results[3].traflag == 0
                && !results[3].triggerAccepted,
            "ENDPOSSEG motion frame must remain outside every positioning segment");
        ok &= expectTrue(
            results[4].localizationSegmentIndex == 2
                && results[4].traflag == 1,
            "second STARTPOSSEG must produce positioning segment 2");
    }
    return ok;
}

bool isRedPixel(const QImage &image, int x, int y)
{
    const QColor color(image.pixel(x, y));
    return color.red() > 200 && color.green() < 80 && color.blue() < 80;
}

bool testCalibrationGuideOverlayDrawsRedBox()
{
    i2r_rlc::control::CalibrationCaptureUpdate update;
    update.automaticGuideOverlayActive = true;
    QImage image(1000, 800, QImage::Format_RGB32);
    image.fill(QColor(0, 0, 0));

    const auto result =
        i2r_rlc::ui::renderCalibrationGuideOverlay(image, update);

    bool ok = true;
    ok &= expectTrue(result.drawn, "automatic calibration overlay should be drawn");
    ok &= expectTrue(!result.clipped, "default legacy red box should fit 1000x800 image");
    ok &= expectTrue(isRedPixel(result.image, 422, 272),
                     "legacy red box should draw red top-left corner");
    ok &= expectTrue(isRedPixel(result.image, 860, 692),
                     "legacy red box should draw red bottom-right corner");
    return ok;
}

bool testCalibrationGuideOverlayInactiveKeepsImage()
{
    i2r_rlc::control::CalibrationCaptureUpdate update;
    update.automaticGuideOverlayActive = false;
    QImage image(1000, 800, QImage::Format_RGB32);
    image.fill(QColor(0, 0, 0));

    const auto result =
        i2r_rlc::ui::renderCalibrationGuideOverlay(image, update);

    bool ok = true;
    ok &= expectTrue(!result.drawn,
                     "inactive automatic calibration overlay should not draw");
    ok &= expectTrue(!isRedPixel(result.image, 422, 272),
                     "inactive overlay should keep source pixels unchanged");
    return ok;
}

bool testCalibrationGuideOverlayClipsSmallImage()
{
    i2r_rlc::control::CalibrationCaptureUpdate update;
    update.automaticGuideOverlayActive = true;
    QImage image(500, 400, QImage::Format_RGB32);
    image.fill(QColor(0, 0, 0));

    const auto result =
        i2r_rlc::ui::renderCalibrationGuideOverlay(image, update);

    bool ok = true;
    ok &= expectTrue(result.drawn, "clipped automatic calibration overlay should draw");
    ok &= expectTrue(result.clipped, "small image should report red box clipping");
    ok &= expectTrue(isRedPixel(result.image, 422, 272),
                     "clipped red box should preserve visible top-left edge");
    ok &= expectTrue(isRedPixel(result.image, 499, 399),
                     "clipped red box should draw on image boundary");
    return ok;
}

/**
 * @brief 正式纠偏使用配置时的标定；磁盘改动仅在下一次显式配置生效。
 * @details 从 Vision 实际输出及其图像坐标手算空间点，验证 runtime 装配而非字段相等。
 */
bool testOnlineCalibrationSnapshot(QCoreApplication &app)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    auto simulation = std::make_shared<FakeRobotSimTransportState>();
    simulation->centeredObservation = true;
    ui::CRuntimeCoordinator runtime(
        std::make_unique<CFakeRobotTransport>(),
        std::make_unique<CFakeRobotSimTransport>(simulation));
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    ui::RuntimeTrackingOptions options;
    options.sensorCalibration.valid = true;
    options.sensorCalibration.focalLengthM = 0.01;
    options.sensorCalibration.sxM = options.sensorCalibration.syM = 0.00001;
    options.sensorCalibration.cxPixel = 160;
    options.sensorCalibration.cyPixel = 120;
    options.sensorCalibration.laserPlaneC = -1;
    options.sensorCalibration.cameraZInToolM = -1;
    options.positionerCalibration.valid = true; // 故意保留初始化时的另一份标定。
    auto state = makeRobotSimReachableState();
    QEventLoop loop;
    std::vector<ui::RuntimeTrackingResult> results;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::trackingResultReady, &loop,
                     [&](const ui::RuntimeTrackingResult &result) {
                         results.push_back(result);
                         loop.quit();
                     });
    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.publishRobotState(state);
    bool ok = connectCorrectionRobotSim(runtime, loop);
    for (int session = 0; session < 2; ++session) {
        const double installX = session == 0 ? 100 : 200;
        std::ofstream(configuration.rbcalibFile)
            << "////SRANG " << installX << ",20,30,0,0,0\n";
        runtime.configureOnlineCorrection(configuration);
        // 会话已配置之后文件改为第三版本，两端必须继续使用刚选定的快照。
        std::ofstream(configuration.rbcalibFile) << "////SRANG 900,20,30,0,0,0\n";
        runtime.startOnlineTracking();
        QTimer::singleShot(100, &loop, [&]() {
            auto frame = makeOnlineSeamFrame();
            frame.frameId = 900 + session;
            runtime.publishCameraFrame(frame);
        });
        QTimer timeout;
        timeout.setSingleShot(true);
        QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
        timeout.start(4000);
        loop.exec();
        if (!results.empty() && !results.back().valid) {
            std::cerr << "snapshot Vision: " << results.back().message.toStdString() << '\n';
        }
        ok &= expectTrue(results.size() == static_cast<std::size_t>(session + 1)
                             && results.back().valid, "snapshot test gets a valid synchronized Vision result");
        if (results.size() == static_cast<std::size_t>(session + 1) && results.back().valid) {
            const auto &result = results.back();
            const double rx = state.curRx_r * (3.14159265358979323846 / 1800000);
            const double ry = state.curRy_r * (3.14159265358979323846 / 1800000);
            const double rz = state.curRz_r * (3.14159265358979323846 / 1800000);
            const double x = result.imageXPixel - 160;
            const double y = result.imageYPixel - 120;
            const double expectedX = state.curx_r / 1000.0 - installX
                + std::cos(rz) * std::cos(ry) * x
                + (std::cos(rz) * std::sin(ry) * std::sin(rx) - std::sin(rz) * std::cos(rx)) * y;
            ok &= expectTrue(std::abs(result.sample.point.xMm - expectedX) < 1.0e-6,
                             "Vision uses the selected session calibration A/B, not startup or changed file");
        }
        runtime.stopOnlineTracking();
        QEventLoop drain;
        QTimer::singleShot(100, &drain, &QEventLoop::quit);
        drain.exec();
    }
    runtime.stop();
    return ok;
}

/** @brief 真实 Vision worker 重配保留会话采样；相机时间跃迁不提前释放异常配额。 */
bool testVisionImageSamplingSessionAndMonotonicClock(QCoreApplication &app)
{
    using namespace i2r_rlc;
    ui::CVisionTrackingWorkerAdapter worker;
    ui::RuntimeTrackingOptions options;
    options.detType = 2;
    options.virtualStatus = control::TrackingVirtualStatus::OnlineRecord;
    options.fusion.mode = vision::KcfFusionMode::KcfOnly;
    options.visionDiagnostics.imageSampleEveryN = 1;
    std::vector<ui::RuntimeTrackingResult> results;
    QObject::connect(&worker, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app, [&results](const ui::RuntimeTrackingResult &result) { results.push_back(result); });
    worker.configure(options, makeProjectionContext());
    worker.startTracking();
    qint64 frameId = 0;
    bool ok = true;
    const auto submit = [&](quint64 sessionId, bool onlineWindow, double simulatedX, qint64 cameraMs) {
        auto frame = makeFrame();
        frame.frameId = ++frameId;
            frame.timestamp = QDateTime::fromMSecsSinceEpoch(1700000000000LL + frameId * 100);
        frame.timestamp = QDateTime::fromMSecsSinceEpoch(cameraMs);
        digital_twin::DigitalTwinVisionLaserObservation observation;
        observation.valid = true;
        observation.frameId = frame.frameId;
        observation.hasWeldPointImage_px = true;
        observation.weldPointImage_px = {simulatedX, 120.0};
        observation.laserLineImagePoints_px = {{80.0, 120.0}, {240.0, 120.0}};
        ui::RuntimeVisionCorrelation correlation;
        correlation.sessionId = sessionId;
        correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
        correlation.frameId = frame.frameId;
        correlation.onlineControlWindowActive = onlineWindow;
        const auto count = results.size();
        worker.submitFrameWithRobotStateObservationAndCorrelation(
            frame, makeReadyRobotState(), -1, observation, correlation);
        ok &= expectTrue(results.size() == count + 1 && results.back().diagnostic.has_value()
                && results.back().diagnostic->algorithm.attempted,
            "sampling regression must execute a real KCF frame with diagnostic evidence");
        return results.size() == count + 1 ? results.back() : ui::RuntimeTrackingResult{};
    };
    const auto bucket = [](const ui::RuntimeTrackingResult &result) {
        return result.diagnostic.has_value() ? result.diagnostic->imageSampleBucket : -2;
    };
    const auto isAnomaly = [](const ui::RuntimeTrackingResult &result) {
        if (!result.diagnostic.has_value()) { return false; }
        const auto &raw = result.diagnostic->algorithm;
        return ui::COnlineVisionImageSampling::isAnomaly(raw.beforeFusionPoint.x,
            raw.beforeFusionPoint.y, raw.simulatedPoint.x, raw.simulatedPoint.y,
            raw.attempted && raw.tracker.completed && raw.tracker.hasResponse,
            raw.tracker.correlationJumpRejected || raw.tracker.fusionJumpRejected);
    };
    for (int index = 0; index < 3; ++index) {
        ok &= expectTrue(bucket(submit(100, false, 160.0, 1000 + index)) == 0,
            "startup must expose exactly its three initial samples");
    }
    worker.stopTracking();
    worker.configure(options, makeProjectionContext());
    worker.startTracking();
    const auto resumedStartup = submit(100, false, 160.0, 1000000000000LL);
    ok &= expectTrue(bucket(resumedStartup) == -1
            && resumedStartup.diagnostic.has_value()
            && resumedStartup.diagnostic->sourceImage.isNull(),
        "same-session configure and pause/resume must retain exhausted startup quota");

    const auto initial = submit(200, true, 160.0, 0);
    ok &= expectTrue(bucket(initial) == 1, "new session must reset the regular image sampling schedule");
    const auto firstAnomaly = submit(200, true, 220.0, 1000);
    ok &= expectTrue(isAnomaly(firstAnomaly) && bucket(firstAnomaly) == 4,
        "raw KCF deviation fixture must trigger the first immediate anomaly");
    const auto forwardClock = submit(200, true, 220.0, 1000000000000LL);
    ok &= expectTrue(isAnomaly(forwardClock) && bucket(forwardClock) != 4,
        "camera clock forward jump must not release the worker monotonic anomaly cooldown");
    const auto backwardClock = submit(200, true, 220.0, 0);
    ok &= expectTrue(isAnomaly(backwardClock) && bucket(backwardClock) != 4,
        "camera clock reset must not reset the worker anomaly cooldown");

    worker.stopTracking();
    worker.configure(options, makeProjectionContext());
    worker.startTracking();
    submit(200, true, 160.0, 0);
    const auto resumedAnomaly = submit(200, true, 220.0, 2000000000000LL);
    ok &= expectTrue(isAnomaly(resumedAnomaly) && bucket(resumedAnomaly) != 4,
        "same-session configure must preserve the monotonic origin and anomaly cooldown");
    const auto nextSession = submit(201, true, 220.0, 2000000000001LL);
    ok &= expectTrue(isAnomaly(nextSession) && bucket(nextSession) == 4,
        "actual session change must release the new session first-anomaly sample");
    const auto newStartup = submit(300, false, 160.0, 0);
    ok &= expectTrue(bucket(newStartup) == 0,
        "actual session change must restore the new startup image quota");
    worker.stopTracking();
    return ok;
}
/** @brief 精定位纠正粗 KCF 锚偏差时，不能把错误粗 ROI 特征提交给模板。 */
bool testOnlineTrainingRequiresConsistentRawRoi()
{
    using namespace i2r_rlc;
    auto frame = makeOnlineSeamFrame();
    const cv::Mat image(frame.image.height(), frame.image.width(), CV_8UC1,
        frame.image.bits(), frame.image.bytesPerLine());
    digital_twin::DigitalTwinVisionLaserObservation observation;
    observation.valid = true;
    observation.hasWeldPointImage_px = true;
    // 模板激光与真实原图重合，但模型焊点锚错误偏右 20 px；精定位仍应恢复真实拐点。
    observation.weldPointImage_px = {180.0, 120.0};
    observation.laserLineImagePoints_px = {{30.0, 83.6}, {160.0, 120.0}, {300.0, 75.2}};
    vision::LegacyOnlineKcfOptions options;
    options.enableSeamRefinement = true;
    options.enableSimulationGuidance = true;
    options.enableOnlineObservationFusion = true;
    options.fusion.mode = vision::KcfFusionMode::KcfOnly;
    vision::CLegacyOnlineKcfDetector detector;
    const auto result = detector.process(image, observation, makeProjectionContext(), options);
    const auto &diagnostic = result.diagnostic;
    bool ok = expectTrue(result.valid && diagnostic.refinement.valid
            && std::abs(diagnostic.refinement.point.x - 160.0) < 1.5
            && cv::norm(diagnostic.beforeFusionPoint - diagnostic.refinement.point) > 8.0,
        "raw ROI mismatch fixture must preserve accepted true geometry despite a wrong model weld anchor");
    ok &= expectTrue(diagnostic.tracker.completed && diagnostic.tracker.hasResponse
            && diagnostic.tracker.trainingDeferred && !diagnostic.tracker.trainingCommitted,
        "accepted original-image refinement cannot train a mismatched coarse KCF patch");
    if (!ok) {
        std::cerr << "training admission: valid=" << result.valid
            << " raw=" << diagnostic.beforeFusionPoint
            << " geometry=" << diagnostic.refinement.point
            << " refinement=" << diagnostic.refinement.valid
            << " deferred=" << diagnostic.tracker.trainingDeferred
            << " committed=" << diagnostic.tracker.trainingCommitted << '\n';
    }
    return ok;
}

/** @brief 正式在线的独立投影必须使用同帧精定位，不受归档与业务融合影响。 */
bool testOnlineRefinedProjectionAndRecovery(QCoreApplication &app)
{
    using namespace i2r_rlc;
    bool ok = true;
    for (const bool diagnostics : {false, true}) {
        ui::CVisionTrackingWorkerAdapter worker;
        ui::RuntimeTrackingOptions options;
        options.detType = 2;
        options.virtualStatus = control::TrackingVirtualStatus::OnlineRecord;
        options.fusion.mode = vision::KcfFusionMode::ManualWeight;
        options.fusion.simulationWeight = 1.0;
        options.visionDiagnostics.enabled = diagnostics;
        options.sensorCalibration.valid = true;
        options.sensorCalibration.focalLengthM = 0.01;
        options.sensorCalibration.sxM = options.sensorCalibration.syM = 0.00001;
        options.sensorCalibration.cxPixel = 160.0;
        options.sensorCalibration.cyPixel = 120.0;
        options.sensorCalibration.laserPlaneC = -1.0;
        options.positionerCalibration.valid = true;
        std::vector<ui::RuntimeTrackingResult> results;
        QObject::connect(&worker, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
            &app, [&](const ui::RuntimeTrackingResult &result) { results.push_back(result); });
        worker.configure(options, makeProjectionContext());
        worker.startTracking();
        qint64 frameId = 0;
        quint64 poseRevision = 1;
        bool onlineWindow = true;
        double simulatedX = 180.0;
        bool tackVisible = false;
        const auto submit = [&](bool blank) {
            auto frame = makeOnlineSeamFrame();
            frame.frameId = ++frameId;
            frame.timestamp = QDateTime::fromMSecsSinceEpoch(1700000000000LL + frameId * 100);
            if (tackVisible) {
                for (int y = 0; y < frame.image.height(); ++y) {
                    auto *row = frame.image.scanLine(y);
                    for (int x = 0; x < frame.image.width(); ++x) {
                        const double dx = x - 160.0;
                        const double stripeY = 120.0 + (dx < 0.0 ? 0.28 : -0.32) * dx
                            - (std::abs(dx) <= 28.0 ? 22.0 : 0.0);
                        const double dy = y - stripeY;
                        row[x] = static_cast<uchar>(15.0 + 220.0 * std::exp(-dy * dy / 2.88));
                    }
                }
                frame.previewImage = frame.image;
            }
            if (blank) {
                frame.image.fill(0);
                frame.previewImage = frame.image;
            }
            digital_twin::DigitalTwinVisionLaserObservation observation;
            observation.valid = true;
            observation.frameId = frame.frameId;
            observation.hasWeldPointImage_px = true;
            observation.weldPointImage_px = {simulatedX, 120.0};
            const double modelShift = simulatedX - 180.0;
            observation.laserLineImagePoints_px = {
                {30.0 + modelShift, 78.0}, {simulatedX, 120.0}, {300.0 + modelShift, 81.6}};
            ui::RuntimeVisionCorrelation correlation;
            correlation.sessionId = diagnostics ? 710 : 711;
            correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
            correlation.onlineControlWindowActive = onlineWindow;
            correlation.frameId = frame.frameId;
            correlation.workpiecePoseRevision = poseRevision;
            worker.submitFrameWithRobotStateObservationAndCorrelation(
                frame, makeReadyRobotState(), -1, observation, correlation);
            return results.empty() ? ui::RuntimeTrackingResult{} : results.back();
        };
        const auto first = submit(false);
        if (!first.valid) {
            std::cerr << "refinement integration: " << first.message.toStdString() << '\n';
        }
        ok &= expectTrue(first.valid && first.sample.point.valid
                && first.independentWeldPointInParent_mm.valid && !first.sample.estimated,
            "online geometry must project with diagnostics both enabled and disabled");
        ok &= expectTrue(first.diagnostic.has_value() == diagnostics,
            "geometry execution must not force diagnostic payload on");
        if (first.valid) {
            ok &= expectTrue(std::abs(first.imageXPixel - 180.0) < 1e-6,
                "manual full simulation weight preserves final business fusion");
            ok &= expectTrue(std::abs(first.independentWeldPointInParent_mm.xMm) < 1.5
                    && std::abs(first.independentWeldPointInParent_mm.yMm) < 1.5,
                "ICP independent projection uses known real seam, not fused prior at X180");
            ok &= expectTrue(std::abs(first.sample.point.xMm - 20.0) < 1e-6,
                "business projection corresponds to final displayed pixel");
            if (first.diagnostic.has_value()) {
                const auto &algorithm = first.diagnostic->algorithm;
                ok &= expectTrue(algorithm.onlineObservationFusionEnabled
                        && algorithm.rawRefinement.valid && algorithm.refinement.valid
                        && std::abs(algorithm.rawRefinement.point.x
                            - algorithm.refinement.point.x) < 1.0e-9
                        && std::abs(algorithm.refinement.point.x - 160.0) <= 1.5
                        && std::abs(algorithm.finalPoint.x - first.imageXPixel) < 1e-9,
                    "diagnostics distinguish original geometry and final fused output");
            }
        }
        const auto predicted = submit(true);
        ok &= expectTrue(predicted.valid && predicted.sample.point.valid
                && predicted.detectedRobotBasePoint.valid
                && predicted.detectedStationBasePoint.valid
                && !predicted.independentWeldPointInParent_mm.valid
                && predicted.sample.estimated
                && predicted.pointSource == vision::OnlineSeamPointSource::SimulationPrediction
                && std::hypot(predicted.imageXPixel - first.imageXPixel,
                    predicted.imageYPixel - first.imageYPixel) <= 2.000001,
            "blank frame keeps a bounded image-history estimate without inventing independent geometry");
        if (predicted.diagnostic.has_value()) {
            const auto &algorithm = predicted.diagnostic->algorithm;
            ok &= expectTrue(algorithm.onlineObservationFusionEnabled
                    && algorithm.attempted && algorithm.tracker.completed
                    && !algorithm.coarseObservationUsed
                    && std::abs(algorithm.continuity.observationWeightX) < 1.0e-12
                    && std::abs(algorithm.continuity.observationWeightY) < 1.0e-12,
                "online blank-frame prediction still runs KCF without fabricating an observation weight");
        }
        vision::KcfFusionOptions realOnly;
        realOnly.mode = vision::KcfFusionMode::KcfOnly;
        worker.updateOnlineFusionOptions(realOnly);
        const auto recovered = submit(false);
        ok &= expectTrue(recovered.valid && recovered.independentWeldPointInParent_mm.valid
                && recovered.sample.estimated,
            "first restored raw geometry stays independent while business recovery is explicitly estimated");
        for (int index = 0; index < 7; ++index) { submit(false); }
        const auto unfused = submit(false);
        ok &= expectTrue(!unfused.sample.estimated,
            "timestamp based finite recovery ends estimated business identity after its duration");
        ok &= expectTrue(unfused.valid && std::abs(unfused.imageXPixel - 160.0) <= 1.5,
            "real-only mode outputs geometry instead of the biased simulated prior");
        // 此标定把像素(x,y)投影为(x-160,y-120,1000) mm；独立点可作
        // 未融合几何观测，非端点权重可识别权重方向反转和重复融合。
        vision::KcfFusionOptions manual;
        manual.mode = vision::KcfFusionMode::ManualWeight;
        manual.simulationWeight = 0.1;
        worker.updateOnlineFusionOptions(manual);
        const auto mixed = submit(false);
        ok &= expectTrue(mixed.valid && mixed.independentWeldPointInParent_mm.valid,
            "non-endpoint manual fusion retains a valid independent observation");
        if (mixed.valid && mixed.independentWeldPointInParent_mm.valid) {
            const auto &independent = mixed.independentWeldPointInParent_mm;
            const double expectedX = 2.0 + 0.9 * independent.xMm;
            const double expectedY = 0.9 * independent.yMm;
            ok &= expectTrue(std::abs(mixed.imageXPixel - 160.0 - expectedX) < 1e-6
                    && std::abs(mixed.imageYPixel - 120.0 - expectedY) < 1e-6
                    && std::abs(mixed.sample.point.xMm - expectedX) < 1e-6
                    && std::abs(mixed.sample.point.yMm - expectedY) < 1e-6
                    && std::abs(mixed.sample.point.zMm - independent.zMm) < 1e-6,
                "manual 10 percent prior and 90 percent geometry fuse once before projection");
        }

        // 切换模式重置二维Kalman：首帧只种子，第二帧首次更新。
        // A/H/B均为单位矩阵、P初值为零、Q=.03I，因此K=.03/(.03+R)。
        vision::KcfFusionOptions automatic;
        automatic.mode = vision::KcfFusionMode::AutomaticKalman;
        worker.updateOnlineFusionOptions(automatic);
        const auto seeded = submit(false);
        const auto automaticSecond = submit(false);
        ok &= expectTrue(seeded.valid && automaticSecond.valid
                && automaticSecond.independentWeldPointInParent_mm.valid,
            "automatic seeded and second frames preserve valid geometry");
        if (diagnostics && seeded.diagnostic.has_value()
            && automaticSecond.diagnostic.has_value() && automaticSecond.valid) {
            const auto &seedTracker = seeded.diagnostic->algorithm.tracker;
            const auto &tracker = automaticSecond.diagnostic->algorithm.tracker;
            const auto &independent = automaticSecond.independentWeldPointInParent_mm;
            const double expectedGain = 0.03 / (0.03 + tracker.observationNoise);
            const double expectedX = 20.0 + expectedGain * (independent.xMm - 20.0);
            const double expectedY = expectedGain * independent.yMm;
            ok &= expectTrue(seedTracker.kalmanSeeded && !seedTracker.fusionApplied
                    && !tracker.kalmanSeeded && tracker.fusionApplied
                    && std::isfinite(expectedGain) && expectedGain > 0.0
                    && expectedGain < 1.0
                    && std::abs(tracker.gainX - expectedGain) < 1e-9
                    && std::abs(tracker.gainY - expectedGain) < 1e-9,
                "automatic second-frame gain follows initial covariance and observation noise");
            ok &= expectTrue(std::abs(automaticSecond.imageXPixel - 160.0 - expectedX) < 1e-6
                    && std::abs(automaticSecond.imageYPixel - 120.0 - expectedY) < 1e-6
                    && std::abs(automaticSecond.sample.point.xMm - expectedX) < 1e-6
                    && std::abs(automaticSecond.sample.point.yMm - expectedY) < 1e-6
                    && std::abs(automaticSecond.sample.point.zMm - independent.zMm) < 1e-6,
                "automatic second frame fuses raw geometry once and projects that displayed point");
        }
        ++poseRevision;
        const auto afterPoseChange = submit(true);
        ok &= expectTrue(afterPoseChange.valid && afterPoseChange.sample.estimated
                && !afterPoseChange.independentWeldPointInParent_mm.valid
                && afterPoseChange.correlation.workpiecePoseRevision == poseRevision
                && std::hypot(afterPoseChange.imageXPixel - automaticSecond.imageXPixel,
                    afterPoseChange.imageYPixel - automaticSecond.imageYPixel) <= 2.000001,
            "CAD pose revision preserves bounded real-image continuity without inventing independent geometry");
        if (afterPoseChange.diagnostic.has_value()) {
            ok &= expectTrue(afterPoseChange.diagnostic->algorithm.guided.attempted
                    && afterPoseChange.diagnostic->algorithm.guided.hasTrustedAnchor
                    && !afterPoseChange.diagnostic->algorithm.templateReinitialized,
                "formal online CAD revision preserves real guidance and KCF template history");
        }
        const auto newPoseGeometry = submit(false);
        if (!newPoseGeometry.valid || !newPoseGeometry.independentWeldPointInParent_mm.valid) {
            std::cerr << "pose recovery: valid=" << newPoseGeometry.valid
                << " independent=" << newPoseGeometry.independentWeldPointInParent_mm.valid
                << " message=" << newPoseGeometry.message.toStdString() << '\n';
            if (newPoseGeometry.diagnostic.has_value()) {
                const auto &raw = newPoseGeometry.diagnostic->algorithm;
                std::cerr << "geometry=" << raw.refinement.valid
                    << " status=" << static_cast<int>(raw.refinement.status)
                    << " hasResponse=" << raw.tracker.hasResponse
                    << " completed=" << raw.tracker.completed
                    << " correlationRejected=" << raw.tracker.correlationJumpRejected
                    << " fusionRejected=" << raw.tracker.fusionJumpRejected
                    << " beforeFusion=" << raw.beforeFusionPoint
                    << " peak=" << raw.tracker.responsePeak << '\n';
            }
        }
        ok &= expectTrue(newPoseGeometry.valid
                && newPoseGeometry.independentWeldPointInParent_mm.valid
                && newPoseGeometry.correlation.workpiecePoseRevision == poseRevision,
            "fresh supported geometry remains independently usable after a CAD-only pose revision");
        // 事故链：真实条纹不动、预焊点遮中心、CAD平滑漂移及反复版本切换。
        // 图像两肩仍可见时，控制业务点不能把模型位移当作真实激光运动。
        tackVisible = true;
        for (int frame = 0; frame < 36; ++frame) {
            simulatedX = 180.0 + frame * 2.0;
            if (frame % 12 == 0) { ++poseRevision; }
            const auto assisted = submit(false);
            ok &= expectTrue(assisted.valid && assisted.sample.estimated
                    && !assisted.independentWeldPointInParent_mm.valid
                    && std::hypot(assisted.imageXPixel - 160.0,
                        assisted.imageYPixel - 120.0) < 6.0,
                "visible real shoulders cross a tack without following smooth CAD drift or inventing raw seam points");
            if (assisted.diagnostic.has_value()) {
                const auto &algorithm = assisted.diagnostic->algorithm;
                ok &= expectTrue(algorithm.guided.hasRawShoulderSupport
                        && algorithm.guided.imageObservationQuality > 0.0
                        && algorithm.tracker.onlineObservationMode
                        && !algorithm.tracker.trainingCommitted,
                    "tack branch uses actual shoulder evidence and never trains its rejected central KCF patch");
            }
        }
        tackVisible = false;
        simulatedX = 290.0;
        ++poseRevision;
        const auto inaccurateModel = submit(false);
        ok &= expectTrue(inaccurateModel.valid
                && inaccurateModel.independentWeldPointInParent_mm.valid
                && std::abs(inaccurateModel.independentWeldPointInParent_mm.xMm) < 1.5,
            "far simulated prior and new CAD version cannot reject clear independent original-image geometry");
        simulatedX = 180.0;
        worker.stopTracking();
        worker.startTracking();
        const auto restartedWithoutAnchor = submit(true);
        ok &= expectTrue(restartedWithoutAnchor.valid && restartedWithoutAnchor.sample.estimated
                && !restartedWithoutAnchor.independentWeldPointInParent_mm.valid
                && std::abs(restartedWithoutAnchor.imageXPixel - 180.0) < 1.0e-6,
            "restart clears old transition seed and a valid current simulation supplies a marked target");
        onlineWindow = false;
        const auto initialStage = submit(false);
        ok &= expectTrue(initialStage.valid
                && (!initialStage.diagnostic.has_value()
                    || !initialStage.diagnostic->algorithm.onlineObservationFusionEnabled),
            "initial reverse-scan stage keeps the new online-only observation policy disabled");
        worker.stopTracking();
    }
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);

    if (argc > 1 && std::string_view(argv[1]) == "--seam-refinement-contract") {
        return testSpacePointInverseProjection() && testOnlineTrainingRequiresConsistentRawRoi()
            && testOnlineRefinedProjectionAndRecovery(app) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--worker-closed-loop-contract") {
        return testWorkerClosedLoop(app) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--session-restart-contract") {
        return testStoppedOnlineTrackingDropsLateResults(app) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--image-sampling-contract") {
        return testVisionImageSamplingSessionAndMonotonicClock(app) ? 0 : 1;
    }

    if (argc > 1 && std::string_view(argv[1]) == "--calibration-contract") {
        return testOnlineCalibrationSnapshot(app) ? 0 : 1;
    }

    using i2r_rlc::control::TrackingVirtualStatus;
    using i2r_rlc::ui::CRuntimeCoordinator;
    using i2r_rlc::ui::RuntimeTrackingOptions;
    using i2r_rlc::ui::RuntimeTrackingResult;

    // 下列三组断言验证 Vision adapter 自身的 fallback/优先级，不再借用
    // 正式在线纠偏启动链。正式纠偏从本特性起必须先完成逐帧 RobotSim 查询，
    // 其 runtime 行为由后续 correction 专项和会话保护测试覆盖。
    i2r_rlc::ui::CVisionTrackingWorkerAdapter defaultVision;
    bool receivedTrackingResult = false;
    RuntimeTrackingResult lastResult;

    QObject::connect(
        &defaultVision,
        &i2r_rlc::ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app,
        [&](const RuntimeTrackingResult &result) {
            receivedTrackingResult = true;
            lastResult = result;
        });

    RuntimeTrackingOptions options;
    options.detType = 2;
    options.virtualStatus = TrackingVirtualStatus::OnlineRecord;
    defaultVision.configure(
        options,
        i2r_rlc::vision::WeldProjectionContext{});
    defaultVision.updateRobotState(makeReadyRobotState());
    defaultVision.startTracking();
    defaultVision.submitFrame(makeFrame());
    defaultVision.stopTracking();

    bool ok = true;
    ok &= expectTrue(receivedTrackingResult, "runtime should emit a tracking result");
    ok &= expectTrue(lastResult.triggerAccepted, "ready robot state should accept trigger");
    ok &= expectTrue(!lastResult.valid, "default simulated callbacks should produce invalid result");

    std::vector<std::string> callbackOrder;
    i2r_rlc::ui::CVisionTrackingWorkerAdapter simulatedVision;
    i2r_rlc::ui::SimulatedVisionCallbacks callbacks;
    callbacks.toNextWeldSeam = [&callbackOrder]() {
        callbackOrder.push_back("toNextWeldSeam");
    };
    callbacks.generate_simu_laser1 = [&callbackOrder]() {
        callbackOrder.push_back("generate_simu_laser1");
        return true;
    };
    callbacks.getSeamPoint2d = [&callbackOrder](
                                       i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px
                                           *point) {
        callbackOrder.push_back("getSeamPoint2d");
        point->x_px = 160.0;
        point->y_px = 120.0;
        return true;
    };
    callbacks.getCurrentVirtualLine = [&callbackOrder]() {
        callbackOrder.push_back("getCurrentVirtualLine");
        return std::vector<i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px>{
            { 80.0, 120.0 },
            { 240.0, 120.0 }
        };
    };
    callbacks.getSeamIndex = [&callbackOrder]() {
        callbackOrder.push_back("getSeamIndex");
        return 7;
    };
    callbacks.getScanLineChangePoint = [&callbackOrder]() {
        callbackOrder.push_back("getScanLineChangePoint");
        return std::optional<i2r_rlc::digital_twin::DigitalTwinPoint3D_mm>{};
    };
    simulatedVision.setSimulatedCallbacks(callbacks);

    bool simulatedResultReceived = false;
    RuntimeTrackingResult simulatedResult;
    QObject::connect(
        &simulatedVision,
        &i2r_rlc::ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app,
        [&](const RuntimeTrackingResult &result) {
            simulatedResultReceived = true;
            simulatedResult = result;
        });
    simulatedVision.configure(options, makeProjectionContext());
    simulatedVision.updateRobotState(makeReadyRobotState());
    simulatedVision.startTracking();
    simulatedVision.submitFrame(makeFrame());
    simulatedVision.stopTracking();

    const std::vector<std::string> expectedOrder{
        "toNextWeldSeam",
        "generate_simu_laser1",
        "getSeamPoint2d",
        "getCurrentVirtualLine",
        "getSeamIndex",
        "getScanLineChangePoint"
    };
    ok &= expectTrue(simulatedResultReceived,
                     "simulated runtime should emit a tracking result");
    ok &= expectTrue(callbackOrder == expectedOrder,
                     "legacy simulated callbacks must keep exact call order");
    if (!simulatedResult.valid || simulatedResult.overlayImage.isNull()) {
        std::cerr << "simulated KCF result: "
                  << simulatedResult.message.toStdString()
                  << ", overlayNull=" << simulatedResult.overlayImage.isNull()
                  << '\n';
    }
    ok &= expectTrue(
        simulatedResult.valid && !simulatedResult.overlayImage.isNull(),
        "valid KCF result should provide a UI overlay image");
    if (simulatedResult.valid && !simulatedResult.overlayImage.isNull()) {
        const int x = std::clamp(
            static_cast<int>(std::lround(simulatedResult.imageXPixel)),
            0,
            simulatedResult.overlayImage.width() - 1);
        const int y = std::clamp(
            static_cast<int>(std::lround(simulatedResult.imageYPixel)),
            0,
            simulatedResult.overlayImage.height() - 1);
        const QColor marker(simulatedResult.overlayImage.pixel(x, y));
        ok &= expectTrue(
            marker.green() > 200 && marker.red() < 80 && marker.blue() < 80,
            "KCF overlay should draw a green cross at the detected point");
    }

    std::vector<std::string> digitalTwinFallbackCallbacks;
    i2r_rlc::ui::CVisionTrackingWorkerAdapter digitalTwinVision;
    i2r_rlc::ui::SimulatedVisionCallbacks fallbackCallbacks;
    fallbackCallbacks.toNextWeldSeam = [&digitalTwinFallbackCallbacks]() {
        digitalTwinFallbackCallbacks.push_back("toNextWeldSeam");
    };
    fallbackCallbacks.generate_simu_laser1 = [&digitalTwinFallbackCallbacks]() {
        digitalTwinFallbackCallbacks.push_back("generate_simu_laser1");
        return false;
    };
    digitalTwinVision.setSimulatedCallbacks(fallbackCallbacks);
    bool digitalTwinResultReceived = false;
    RuntimeTrackingResult digitalTwinResult;
    QObject::connect(
        &digitalTwinVision,
        &i2r_rlc::ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app,
        [&](const RuntimeTrackingResult &result) {
            digitalTwinResultReceived = true;
            digitalTwinResult = result;
        });
    digitalTwinVision.configure(options, makeProjectionContext());
    digitalTwinVision.updateRobotState(makeReadyRobotState());
    digitalTwinVision.startTracking();
    i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation observation;
    observation.valid = true;
    observation.frameId = 1;
    observation.hasWeldPointImage_px = true;
    observation.weldPointImage_px = {160.0, 120.0};
    observation.laserLineImagePoints_px = {
        {80.0, 120.0},
        {240.0, 120.0}
    };
    observation.message = QStringLiteral("test DigitalTwin observation");
    digitalTwinVision.updateDigitalTwinVisionObservation(observation);
    digitalTwinVision.submitFrame(makeFrame());
    digitalTwinVision.stopTracking();

    ok &= expectTrue(digitalTwinResultReceived,
                     "DigitalTwin runtime should emit a tracking result");
    ok &= expectTrue(digitalTwinResult.valid,
                     "DigitalTwin observation should drive a valid KCF result");
    ok &= expectTrue(digitalTwinFallbackCallbacks.empty(),
                     "DigitalTwin observation must be preferred over legacy callbacks");

    ok &= testOnlineTrainingRequiresConsistentRawRoi();
    ok &= testOnlineRefinedProjectionAndRecovery(app);
    ok &= testVisionImageSamplingSessionAndMonotonicClock(app);
    ok &= testInjectedRobotSimTransportIsUsed(app);
    ok &= testOnlineCalibrationSnapshot(app);
    ok &= testCorrectionSessionPausesAndResumesRealtimeRobotSim(app);
    ok &= testCorrectionTrackingKeepsOnlyLatestPendingFrame(app);
    ok &= testLatestRealtimeCommandWins(app);
    ok &= testWorkerClosedLoop(app);
    ok &= testStoppedOnlineTrackingDropsLateResults(app);
    ok &= testGlobalLocalizationTrackingDoesNotFeedOnlineController(app);
    ok &= testGlobalLocalizationFramesKeepCaptureTimeRobotSegment(app);
    ok &= testCalibrationGuideOverlayDrawsRedBox();
    ok &= testCalibrationGuideOverlayInactiveKeepsImage();
    ok &= testCalibrationGuideOverlayClipsSmallImage();

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: runtime tracking flow tests\n";
    return 0;
}
