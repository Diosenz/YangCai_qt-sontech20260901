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
#include <limits>
#include <memory>
#include <mutex>
#include <string_view>
#include <string>
#include <vector>

#include <QColor>
#include <QCoreApplication>
#include <QDateTime>
#include <QEventLoop>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <QUuid>

#include <Eigen/Geometry>

#include "geometry/WeldImageToSpaceProjector.h"
#include "track/LegacyOnlineKcfDetector.h"

#include "CalibrationGuideOverlay.h"
#include "RobotProtocolCodec.h"
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
    std::atomic<int> reverseCompleteReply{-1};
    bool blockFirstTeachPoint = false;
    std::atomic<bool> firstTeachPointEntered{false};
    std::mutex teachPointMutex;
    std::condition_variable teachPointCondition;
    bool releaseFirstTeachPoint = false;
    std::vector<i2r_rlc::robot_comm::RobotCommandType> types;
    std::vector<std::string> requests;
    std::vector<int> teachPointIndices;
    std::vector<int> teachPointOfflineValues;
    std::optional<i2r_rlc::robot_comm::RobotState> replayPollState;
    /// @brief 跨线程会话测试只用原子字段推进外部设备的反馈阶段和 TCP。
    std::atomic<int> replayCorrectionFlags{-1};
    std::atomic<long> replayTcpXUm{std::numeric_limits<long>::min()};
    /// @brief 外部设备替身的原子反馈控制，不向生产代码注入确认结论。
    std::atomic<int> operationHoldReply{-1};
    std::atomic<i2r_rlc::robot_comm::RobotCommandError> operationError{
        i2r_rlc::robot_comm::RobotCommandError::None};
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
        if (m_record->operationError.load()
            != i2r_rlc::robot_comm::RobotCommandError::None) {
            receipt.responseReceived = false;
            receipt.completed = false;
            receipt.error = m_record->operationError.load();
            receipt.response.clear();
            receipt.message = "simulated external transport failure";
            return receipt;
        }
        if (type == i2r_rlc::robot_comm::RobotCommandType::TeachPointUpdate
            && pointIndex == 0 && m_record->blockFirstTeachPoint) {
            std::unique_lock<std::mutex> lock(m_record->teachPointMutex);
            m_record->firstTeachPointEntered = true;
            m_record->teachPointCondition.wait_for(lock, std::chrono::seconds(3),
                [this]() { return m_record->releaseFirstTeachPoint; });
        }
        if (state != nullptr) {
            const auto before = *state;
            *state = m_record->replayPollState.value_or(makeReadyRobotState());
            if (m_record->replayCorrectionFlags.load() >= 0) {
                state->corflag = state->detflag = m_record->replayCorrectionFlags.load();
            }
            if (m_record->replayTcpXUm.load() != std::numeric_limits<long>::min()) {
                state->curx_r = m_record->replayTcpXUm.load();
            }
            if (m_record->reverseCompleteReply.load() >= 0) {
                state->onlineReverseCompleteFlag = m_record->reverseCompleteReply.load();
            }
            if (m_record->operationHoldReply.load() >= 0) {
                state->sStart = 1;
                state->sHold = m_record->operationHoldReply.load();
                auto fields = QString::fromStdString(receipt.response).mid(2).split(',');
                fields[4] = QString::number(state->sStart);
                fields[5] = QString::number(state->sHold);
                receipt.response = "D " + fields.join(',').toStdString();
            }
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
    // 同一个event loop会在正式会话中再次exec；设置阶段的超时不能泄漏到下一阶段。
    QTimer setupTimeout;
    setupTimeout.setSingleShot(true);
    QObject::connect(&setupTimeout, &QTimer::timeout, &loop, &QEventLoop::quit);
    setupTimeout.start(2000);
    loop.exec();
    setupTimeout.stop();
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

/** @brief 作业停止必须经实际公共协议请求保持，关闭跟踪不能冒充停止设备。 */
bool testOnlineOperationStopIsolatesMotion(QCoreApplication &app)
{
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    ui::CRobotCommWorkerAdapter robot(std::make_unique<CFakeRobotTransport>(record));
    robot.connectRobot();
    robot_comm::RobotRealtimeCorrectionCommand realtime;
    realtime.valid = true;
    realtime.dxUm = 111;
    robot.sendRealtimeCorrectionCommand(realtime);
    const auto stopBoundary = record->requests.size();
    robot.requestControlledHold();
    realtime.dxUm = 222;
    robot.sendRealtimeCorrectionCommand(realtime);

    robot_comm::RobotCorrectionCommand correction;
    correction.valid = true;
    correction.requestTeachPointUpdate = true;
    correction.pointCount = 1;
    correction.teachPoints.resize(1);
    correction.teachPoints[0].valid = true;
    correction.stationAxes.resize(1);
    correction.stationAxes[0].valid = true;
    robot.sendCorrectionCommand(correction);
    robot_comm::RobotJobStartCommand start;
    start.valid = true;
    start.requestStart = true;
    start.jobName = "OFFLINE_HOLD_CONTRACT";
    robot.sendJobStartCommand(start);
    robot.setTrackingRequested(true);
    robot.disconnectRobot();
    robot.connectRobot();
    app.processEvents(QEventLoop::AllEvents, 100);
    robot.disconnectRobot();

    bool noMotionAfterStop = true;
    bool holdRemainsLatched = true;
    int pollCount = 0;
    for (auto index = stopBoundary; index < record->requests.size(); ++index) {
        noMotionAfterStop &= record->types[index] == robot_comm::RobotCommandType::StatusPoll;
        if (record->types[index] != robot_comm::RobotCommandType::StatusPoll) { continue; }
        ++pollCount;
        const auto fields = QString::fromStdString(record->requests[index]).mid(2).split(',');
        holdRemainsLatched &= fields.size() >= 7 && fields[6] == QStringLiteral("1");
    }
    return expectTrue(noMotionAfterStop && holdRemainsLatched && pollCount >= 2,
        "operation stop cancels pending P and rejects late P/O/J; tracking and reconnect keep HOLD latched");
}

bool testOnlineOperationStopRequestsHold(QCoreApplication &app)
{
    Q_UNUSED(app);
    auto record = std::make_shared<FakeTransportState>();
    i2r_rlc::ui::CRuntimeCoordinator runtime(
        std::make_unique<CFakeRobotTransport>(record));
    QEventLoop loop;
    bool requested = false;
    QObject::connect(&runtime,
        &i2r_rlc::ui::CRuntimeCoordinator::robotStateReady,
        &loop, [&](const i2r_rlc::robot_comm::RobotState &) {
            if (!requested) {
                requested = true;
                runtime.requestOnlineOperationStop();
                QTimer::singleShot(250, &loop, &QEventLoop::quit);
            }
        });
    runtime.start();
    runtime.connectRobot();
    QTimer::singleShot(3000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();

    bool holdRequested = false;
    for (const auto &request : record->requests) {
        if (request.rfind("N ", 0) != 0) { continue; }
        const auto fields = QString::fromStdString(request).mid(2).split(',');
        holdRequested |= fields.size() >= 7 && fields[6] == QStringLiteral("1");
    }
    return expectTrue(requested && holdRequested,
        "online operation stop sends the existing HOLD field instead of only disabling tracking");
}

/** @brief 请求成功仍须等待新 D 保持反馈，缓存状态不能提前确认。 */
bool testOnlineOperationStopWaitsForFreshHold(QCoreApplication &app)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    record->operationHoldReply = 0;
    ui::CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>(record));
    QEventLoop loop;
    bool requested = false;
    bool sawPending = false;
    bool prematureConfirmation = false;
    bool confirmed = false;
    bool arcUnverified = true;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineOperationStopUpdated,
        &loop, [&](const robot_comm::RobotOperationStopState &state) {
            sawPending |= state.status == robot_comm::RobotOperationStopStatus::Pending;
            arcUnverified &= !state.arcStopVerified;
            if (state.status == robot_comm::RobotOperationStopStatus::HoldConfirmed) {
                prematureConfirmation |= record->operationHoldReply.load() != 1;
                confirmed = true;
                loop.quit();
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::robotStateReady,
        &loop, [&](const robot_comm::RobotState &) {
            if (requested) { return; }
            requested = true;
            runtime.requestOnlineOperationStop();
            QTimer::singleShot(25, &loop, [&runtime]() {
                auto cached = makeReadyRobotState();
                cached.sHold = 1;
                runtime.publishRobotState(cached);
            });
            QTimer::singleShot(150, &loop, [record]() { record->operationHoldReply = 1; });
        });
    runtime.start();
    runtime.connectRobot();
    QTimer::singleShot(3000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();
    return expectTrue(sawPending && confirmed && !prematureConfirmation && arcUnverified,
        "stop stays pending despite a successful transaction or cached HOLD, then confirms only a fresh D HOLD without claiming arc-off");
}

bool testOnlineOperationStopReportsUnconfirmed(QCoreApplication &app)
{
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    record->operationHoldReply = 1;
    ui::CRobotCommWorkerAdapter robot(std::make_unique<CFakeRobotTransport>(record));
    robot_comm::RobotOperationStopState latest;
    QObject::connect(&robot, &ui::CRobotCommWorkerAdapter::operationStopUpdated,
        &app, [&](const robot_comm::RobotOperationStopState &state) { latest = state; });
    robot.connectRobot();
    robot.requestControlledHold();
    bool ok = expectTrue(latest.status == robot_comm::RobotOperationStopStatus::HoldConfirmed,
        "precondition: a fresh transport reply confirms HOLD");
    for (const auto failure : {robot_comm::RobotCommandError::ReceiveFailed,
                              robot_comm::RobotCommandError::InvalidResponse}) {
        record->operationError = failure;
        robot.pollRobotState();
        ok &= expectTrue(latest.status == robot_comm::RobotOperationStopStatus::Unconfirmed
                && latest.error == failure && latest.holdRequestLatched && !latest.arcStopVerified,
            "timeout or invalid D invalidates stale HOLD confirmation without releasing request");
        record->operationError = robot_comm::RobotCommandError::None;
        robot.pollRobotState();
        ok &= expectTrue(latest.status == robot_comm::RobotOperationStopStatus::HoldConfirmed,
            "fresh valid D can confirm the still-latched HOLD again");
    }
    robot.disconnectRobot();
    return expectTrue(latest.status == robot_comm::RobotOperationStopStatus::Unconfirmed
            && latest.error == robot_comm::RobotCommandError::NotConnected
            && latest.holdRequestLatched && !latest.arcStopVerified,
        "disconnect reports HOLD unconfirmed instead of retaining old acknowledgement") && ok;
}

bool testOnlineOperationStopKeepsSessionReport(QCoreApplication &app)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    const QString root = QDir::temp().filePath(
        QStringLiteral("i2r_stop_session_%1").arg(QUuid::createUuid().toString(QUuid::Id128)));
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    configuration.diagnosticOutputDirectory = root.toStdString();
    configuration.visionDiagnostics.enabled = true;
    auto record = std::make_shared<FakeTransportState>();
    record->operationHoldReply = 0;
    ui::CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>(record));
    QEventLoop loop;
    bool stopRequested = false;
    quint64 originalSessionId = 0;
    bool holdConfirmed = false;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineCorrectionUpdated,
        &loop, [&](const control::OnlineCorrectionUpdate &update) {
            if (!stopRequested
                && update.phase == control::OnlineCorrectionPhase::InitialDetectionRunning) {
                originalSessionId = update.state.controlTarget.sessionId;
                stopRequested = true;
                runtime.requestOnlineOperationStop();
                // 收起面板等普通检测清理不能关闭仍在等待设备确认的报告。
                QTimer::singleShot(25, &loop, [&runtime]() { runtime.stopOnlineTracking(); });
                QTimer::singleShot(150, &loop, [record]() { record->operationHoldReply = 1; });
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineOperationStopUpdated,
        &loop, [&](const robot_comm::RobotOperationStopState &state) {
            if (state.status == robot_comm::RobotOperationStopStatus::HoldConfirmed) {
                holdConfirmed = true;
                QTimer::singleShot(50, &loop, &QEventLoop::quit);
            }
        });
    runtime.start();
    runtime.configureOnlineCorrection(configuration);
    runtime.connectRobot();
    runtime.startOnlineTracking();
    QTimer::singleShot(3000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();
    bool ok = expectTrue(stopRequested && holdConfirmed,
        "runtime session reaches explicit stop and fresh HOLD acknowledgement");
    const auto directories = QDir(root).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    if (!expectTrue(directories.size() == 1, "operation stop preserves one session directory")) { return false; }
    QFile report(QDir(QDir(root).filePath(directories.first())).filePath(
        QStringLiteral("unified_session.jsonl")));
    if (!expectTrue(report.open(QIODevice::ReadOnly), "stop session publishes unified report")) { return false; }
    bool requestedInReport = false;
    bool holdReceiptInReport = false;
    bool holdResponseInReport = false;
    bool confirmedInReport = false;
    bool prematureEnd = false;
    for (const auto &line : report.readAll().split('\n')) {
        if (line.isEmpty()) { continue; }
        const auto event = QJsonDocument::fromJson(line).object();
        const auto type = event.value(QStringLiteral("event")).toString();
        if (type == QStringLiteral("operation_stop")) {
            ok &= expectTrue(originalSessionId != 0
                    && event.value(QStringLiteral("session_id")).toString() == QString::number(originalSessionId)
                    && !event.value(QStringLiteral("arc_stop_verified")).toBool(),
                "stop events retain original session identity and arc remains unverified");
            requestedInReport |= event.value(QStringLiteral("status")).toString() == QStringLiteral("pending");
            if (event.value(QStringLiteral("status")).toString() == QStringLiteral("hold_confirmed")) {
                confirmedInReport = requestedInReport && holdReceiptInReport && holdResponseInReport;
            }
        } else if (type == QStringLiteral("receipt") && requestedInReport) {
            const auto request = event.value(QStringLiteral("request")).toString();
            const auto fields = request.mid(2).split(',');
            holdReceiptInReport |= request.startsWith(QStringLiteral("N "))
                && fields.size() >= 7 && fields[6] == QStringLiteral("1")
                && event.value(QStringLiteral("response_received")).toBool();
            robot_comm::RobotState response;
            holdResponseInReport |= holdReceiptInReport
                && robot_comm::CRobotProtocolCodec::decodeRobotState(
                    event.value(QStringLiteral("response")).toString().toStdString(), &response)
                && response.sHold == 1;
        } else if (type == QStringLiteral("session_end")) {
            prematureEnd |= !confirmedInReport;
        }
    }
    return expectTrue(requestedInReport && holdReceiptInReport && confirmedInReport && !prematureEnd,
        "same-session report keeps pending request, actual N/D receipt and HOLD feedback before closing") && ok;
}

/** @brief HOLD 后的检测与完整作业启动均须由人工处理，不能由新调用解除保持。 */
bool testOnlineOperationStopRejectsRestart(QCoreApplication &app)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    record->operationHoldReply = 1;
    ui::CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>(record));
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    QEventLoop loop;
    bool requested = false;
    bool restartAttempted = false;
    bool detectionRestarted = false;
    int holdRefusals = 0;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::robotStateReady,
        &loop, [&](const robot_comm::RobotState &) {
            if (!requested) {
                requested = true;
                runtime.requestOnlineOperationStop();
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineOperationStopUpdated,
        &loop, [&](const robot_comm::RobotOperationStopState &state) {
            if (state.status != robot_comm::RobotOperationStopStatus::HoldConfirmed
                || restartAttempted) { return; }
            restartAttempted = true;
            runtime.startOnlineTracking();
            runtime.startOnlineCorrectionJob(configuration, "MANUAL_RESTART_REQUIRED");
            QTimer::singleShot(150, &loop, &QEventLoop::quit);
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineCorrectionStartFailed,
        &loop, [&](const QString &message) {
            holdRefusals += message.contains(QStringLiteral("HOLD")) ? 1 : 0;
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineCorrectionUpdated,
        &loop, [&](const control::OnlineCorrectionUpdate &update) {
            detectionRestarted |= restartAttempted
                && update.phase == control::OnlineCorrectionPhase::InitialDetectionRunning;
        });
    runtime.start();
    runtime.configureOnlineCorrection(configuration);
    runtime.connectRobot();
    QTimer::singleShot(3000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();
    return expectTrue(restartAttempted && !detectionRestarted && holdRefusals == 2,
        "latched operation HOLD rejects both tracking and job restart with an explicit manual-recovery reason");
}

/** @brief 从完整 runtime 连续曝光建立初始 O，在首笔真实事务在途时请求作业停止。 */
bool testOnlineOperationStopInterruptsInFlightBatch(QCoreApplication &app)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    record->blockFirstTeachPoint = true;
    record->operationHoldReply = 0;
    record->reverseCompleteReply = 0;
    auto feedback = makeReadyRobotState();
    feedback.corflag = feedback.detflag = 1;
    record->replayPollState = feedback;
    auto simulation = std::make_shared<FakeRobotSimTransportState>();
    simulation->centeredObservation = true;
    ui::CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>(record),
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
    options.sensorCalibration.cameraXInToolM = 0.01;
    options.positionerCalibration.valid = true;
    QEventLoop loop;
    bool reverseCompleted = false;
    bool stopEntered = false;
    bool confirmed = false;
    bool incompletePointObserved = false;
    int validExposures = 0;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::trackingResultReady, &loop,
        [&](const ui::RuntimeTrackingResult &result) {
            if (result.valid && !result.sample.estimated) { ++validExposures; }
            if (validExposures >= 2 && !reverseCompleted) {
                reverseCompleted = true;
                record->reverseCompleteReply = 1;
                auto state = feedback;
                state.onlineReverseCompleteFlag = 1;
                runtime.publishRobotState(state);
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineOperationStopUpdated, &loop,
        [&](const robot_comm::RobotOperationStopState &state) {
            if (state.status == robot_comm::RobotOperationStopStatus::HoldConfirmed) {
                confirmed = true;
                QTimer::singleShot(20, &loop, &QEventLoop::quit);
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::robotStateReady, &loop,
        [&](const robot_comm::RobotState &state) { incompletePointObserved |= state.offline == 4; });
    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.configureOnlineCorrection(configuration);
    runtime.publishRobotState(makeRobotSimReachableState());
    const bool synchronized = connectCorrectionRobotSim(runtime, loop);
    runtime.publishRobotState(feedback);
    runtime.connectRobot();
    runtime.startOnlineTracking();
    QTimer exposureTimer;
    qint64 frameId = 8000;
    QObject::connect(&exposureTimer, &QTimer::timeout, &loop, [&]() {
        if (reverseCompleted) { return; }
        auto frame = makeOnlineSeamFrame();
        frame.frameId = ++frameId;
        frame.timestamp = QDateTime::currentDateTime();
        runtime.publishCameraFrame(frame);
    });
    exposureTimer.start(100);
    QTimer stopTimer;
    QObject::connect(&stopTimer, &QTimer::timeout, &loop, [&]() {
        if (!stopEntered && record->firstTeachPointEntered.load()) {
            stopEntered = true;
            runtime.requestOnlineOperationStop();
            record->operationHoldReply = 1;
            {
                std::lock_guard<std::mutex> lock(record->teachPointMutex);
                record->releaseFirstTeachPoint = true;
            }
            record->teachPointCondition.notify_all();
        }
    });
    stopTimer.start(10);
    QTimer::singleShot(6000, &loop, &QEventLoop::quit);
    loop.exec();
    exposureTimer.stop();
    stopTimer.stop();
    {
        std::lock_guard<std::mutex> lock(record->teachPointMutex);
        record->releaseFirstTeachPoint = true;
    }
    record->teachPointCondition.notify_all();
    runtime.stop();
    const auto pointCount = std::count(record->types.begin(), record->types.end(),
        robot_comm::RobotCommandType::TeachPointUpdate);
    bool holdRequest = false;
    for (std::size_t i = 0; i < record->types.size(); ++i) {
        if (record->types[i] == robot_comm::RobotCommandType::StatusPoll) {
            const auto fields = QString::fromStdString(record->requests[i]).mid(2).split(',');
            holdRequest |= fields.size() >= 7 && fields[6] == QStringLiteral("1");
        }
    }
    bool ok = expectTrue(synchronized && reverseCompleted && stopEntered && confirmed,
        "complete runtime exposure/control chain reaches blocked first O and explicit confirmed HOLD");
    ok &= expectTrue(incompletePointObserved && !record->teachPointOfflineValues.empty()
            && record->teachPointOfflineValues.front() == 4,
        "already transmitted first O retains its incomplete batch feedback without forged completion");
    return expectTrue(pointCount == 1 && holdRequest,
        "stop entering during first O prevents every later batch point and proceeds with N HOLD") && ok;
}

/** @brief UI 在公开结果回调内普通停止并重新启动时，旧点不得进入新控制会话。 */
bool testReentrantOnlineRestartDropsOldTrackingResult(QCoreApplication &app, bool provideNewExposures)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    auto feedback = makeReadyRobotState();
    feedback.corflag = feedback.detflag = 1;
    record->replayPollState = feedback;
    record->reverseCompleteReply = 0;
    auto simulation = std::make_shared<FakeRobotSimTransportState>();
    simulation->centeredObservation = true;
    ui::CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>(record),
        std::make_unique<CFakeRobotSimTransport>(simulation));
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    configuration.visionDiagnostics.enabled = true;
    configuration.visionDiagnostics.imagesEnabled = false;
    ui::RuntimeTrackingOptions options;
    options.sensorCalibration.valid = true;
    options.sensorCalibration.focalLengthM = 0.01;
    options.sensorCalibration.sxM = options.sensorCalibration.syM = 0.00001;
    options.sensorCalibration.cxPixel = 160;
    options.sensorCalibration.cyPixel = 120;
    options.sensorCalibration.laserPlaneC = -1;
    options.sensorCalibration.cameraZInToolM = -1;
    options.sensorCalibration.cameraXInToolM = 0.01;
    options.positionerCalibration.valid = true;
    options.visionDiagnostics.enabled = true;
    QEventLoop loop;
    std::vector<quint64> sessions;
    bool restarted = false;
    bool reverseCompleted = false;
    bool holdRequested = false;
    bool newExposuresEnabled = false;
    bool newInitialApproach = false;
    bool oldInitialApproach = false;
    bool freshModel = false;
    int newReliableExposures = 0;
    qint64 stoppedFrameId = -1;
    int newSubmitted = 0;
    int newCameraFrames = 0;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::frameReady, &loop,
        [&](const camera::CameraFrame &frame) {
            if (newExposuresEnabled) { ++newCameraFrames; }
            if (provideNewExposures) {
                std::cerr << "restart-camera=" << frame.frameId
                    << " newEnabled=" << newExposuresEnabled << std::endl;
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::statusMessage, &loop,
        [&](const QString &message) {
            if (provideNewExposures) {
                std::cerr << "restart-status=" << message.toStdString() << std::endl;
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::correctionSyncUpdated, &loop,
        [&](const ui::CorrectionSyncUpdate &update) {
            if (provideNewExposures) {
                std::cerr << "restart-sync=" << static_cast<int>(update.state)
                    << " session=" << update.sessionId << " frame=" << update.frameId
                    << " message=" << update.message.toStdString() << std::endl;
            }
            if (update.purpose == ui::CorrectionPurpose::OnlineCorrection
                && update.state == ui::CorrectionSyncState::Idle && update.sessionId != 0
                && std::find(sessions.begin(), sessions.end(), update.sessionId) == sessions.end()) {
                sessions.push_back(update.sessionId);
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineOperationStopUpdated, &loop,
        [&](const robot_comm::RobotOperationStopState &state) { holdRequested |= state.holdRequestLatched; });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineCorrectionUpdated, &loop,
        [&](const control::OnlineCorrectionUpdate &update) {
            if (provideNewExposures) {
                std::cerr << "restart-phase=" << static_cast<int>(update.phase)
                    << " newEnabled=" << newExposuresEnabled << " reliable=" << newReliableExposures
                    << " message=" << update.message << std::endl;
            }
            if (!restarted || update.phase != control::OnlineCorrectionPhase::WaitingForInitialApproach) { return; }
            oldInitialApproach |= !newExposuresEnabled;
            if (newExposuresEnabled && newReliableExposures >= 2) {
                newInitialApproach = true;
                QTimer::singleShot(20, &loop, &QEventLoop::quit);
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::trackingResultReady, &loop,
        [&](const ui::RuntimeTrackingResult &result) {
            if (provideNewExposures) {
                std::cerr << "restart-frame=" << result.frameId << " valid=" << result.valid
                    << " estimated=" << result.sample.estimated << " session=" << result.correlation.sessionId
                    << " newEnabled=" << newExposuresEnabled << " diagnostic=" << result.diagnostic.has_value()
                    << " reinit=" << (result.diagnostic.has_value() && result.diagnostic->algorithm.templateReinitialized)
                    << " message=" << result.message.toStdString() << std::endl;
            }
            if (!result.valid || result.sample.estimated) { return; }
            if (restarted) {
                if (!newExposuresEnabled || result.frameId <= stoppedFrameId
                    || sessions.size() != 2 || result.correlation.sessionId != sessions.back()) { return; }
                ++newReliableExposures;
                if (newReliableExposures == 1) {
                    freshModel = result.diagnostic.has_value()
                        && result.diagnostic->algorithm.templateReinitialized;
                }
                if (newReliableExposures == 2) {
                    record->reverseCompleteReply = 1;
                    auto state = feedback;
                    state.onlineReverseCompleteFlag = 1;
                    runtime.publishRobotState(state);
                }
                return;
            }
            // 公开UI回调是合法重入入口；不触碰worker内部或测试专用生产入口。
            stoppedFrameId = result.frameId;
            restarted = true;
            runtime.stopOnlineTracking();
            runtime.startOnlineTracking();
            QTimer::singleShot(100, &loop, [&]() {
                reverseCompleted = true;
                record->reverseCompleteReply = 1;
                auto state = feedback;
                state.onlineReverseCompleteFlag = 1;
                runtime.publishRobotState(state);
                if (!provideNewExposures) {
                    QTimer::singleShot(300, &loop, &QEventLoop::quit);
                    return;
                }
                QTimer::singleShot(300, &loop, [&]() {
                    // 空窗已经证明旧结果不能形成O；现在只通过公开入口供给新曝光。
                    record->reverseCompleteReply = 0;
                    auto state = feedback;
                    state.onlineReverseCompleteFlag = 0;
                    runtime.publishRobotState(state);
                    newExposuresEnabled = true;
                });
            });
        });
    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.configureOnlineCorrection(configuration);
    runtime.publishRobotState(makeRobotSimReachableState());
    const bool synchronized = connectCorrectionRobotSim(runtime, loop);
    runtime.publishRobotState(feedback);
    runtime.connectRobot();
    runtime.startOnlineTracking();
    QTimer exposureTimer;
    qint64 frameId = 12000;
    QObject::connect(&exposureTimer, &QTimer::timeout, &loop, [&]() {
        if (restarted && !newExposuresEnabled) { return; }
        auto frame = makeOnlineSeamFrame();
        frame.frameId = ++frameId;
        frame.timestamp = QDateTime::currentDateTime();
        if (newExposuresEnabled) { ++newSubmitted; }
        runtime.publishCameraFrame(frame);
    });
    exposureTimer.start(100);
    QTimer::singleShot(10000, &loop, &QEventLoop::quit);
    loop.exec();
    exposureTimer.stop();
    runtime.stop();
    bool ok = expectTrue(synchronized && restarted && reverseCompleted
            && sessions.size() == 2 && sessions[1] > sessions[0] && !holdRequested,
        "complete runtime image chain performs explicit ordinary restart into a new isolated session");
    const auto oldPointCommands = std::count(record->types.begin(), record->types.end(),
        robot_comm::RobotCommandType::TeachPointUpdate);
    if (provideNewExposures) {
        std::cerr << "restart-contract oldApproach=" << oldInitialApproach
            << " freshModel=" << freshModel << " newReliable=" << newReliableExposures
            << " newApproach=" << newInitialApproach << " O=" << oldPointCommands << std::endl;
        std::cerr << "restart-input submitted=" << newSubmitted
            << " delivered=" << newCameraFrames << std::endl;
        ok &= expectTrue(!oldInitialApproach && freshModel && newReliableExposures >= 2
                && newInitialApproach && oldPointCommands > 0,
            "explicit new session builds a fresh image model and acknowledges O only after new reliable exposures");
        return ok;
    }
    return expectTrue(oldPointCommands == 0,
        "reentrant restart without any new exposure cannot produce O from the old session result") && ok;
}

bool testLatestRealtimeCommandWins(QCoreApplication &app)
{
    using namespace i2r_rlc;
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
    std::vector<robot_comm::RobotCommandReceipt> realtimeReceipts;
    QObject::connect(&robot, &ui::CRobotCommWorkerAdapter::commandReceiptReady, &app,
        [&realtimeReceipts](const robot_comm::RobotCommandReceipt &receipt,
            const robot_comm::RobotState &) {
            if (receipt.type == robot_comm::RobotCommandType::RealtimeCorrection) {
                realtimeReceipts.push_back(receipt);
            }
        });

    i2r_rlc::robot_comm::RobotRealtimeCorrectionCommand command;
    command.valid = true;
    command.dxUm = 111;
    command.localTrace = robot_comm::RobotLocalCommandTrace{902, 11, 71};
    robot.sendRealtimeCorrectionCommand(command);
    command.dxUm = 222;
    command.localTrace = robot_comm::RobotLocalCommandTrace{902, 12, 72};
    robot.sendRealtimeCorrectionCommand(command);
    command.dxUm = 333;
    command.localTrace = robot_comm::RobotLocalCommandTrace{902, 13, 73};
    robot.sendRealtimeCorrectionCommand(command);
    app.processEvents(QEventLoop::AllEvents, 100);
    // 相同线协议增量仍是新生成命令，不能按request文本误配旧来源。
    command.localTrace = robot_comm::RobotLocalCommandTrace{902, 14, 74};
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
    ok &= expectTrue(realtimeCount == 2,
                     "queued corrections coalesce while a later identical increment is a distinct P");
    ok &= expectTrue(realtimeReceipts.size() == 2
        && realtimeReceipts[0].localTrace.has_value() && realtimeReceipts[1].localTrace.has_value()
        && realtimeReceipts[0].localTrace->commandId == 13
        && realtimeReceipts[0].localTrace->sessionId == 902
        && realtimeReceipts[0].localTrace->targetGeneration == 73
        && realtimeReceipts[1].localTrace->commandId == 14
        && realtimeReceipts[1].localTrace->targetGeneration == 74
        && realtimeReceipts[0].localTransactionId > 0
        && realtimeReceipts[1].localTransactionId > realtimeReceipts[0].localTransactionId,
        "actual selected P retains 333 identity and identical increments keep distinct local transactions");
    ui::COnlineCorrectionWorkerAdapter control;
    control.setControlSessionId(903);
    int updates = 0;
    QObject::connect(&control, &ui::COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
        &app, [&updates](const control::OnlineCorrectionUpdate &) { ++updates; });
    if (!realtimeReceipts.empty()) {
        control.handleRobotCommandReceipt(realtimeReceipts.front(), {});
    }
    ok &= expectTrue(updates == 0, "a known old-session P receipt cannot update the new control session");
    robot_comm::RobotCommandReceipt legacyReceipt;
    legacyReceipt.type = robot_comm::RobotCommandType::RealtimeCorrection;
    legacyReceipt.sent = legacyReceipt.responseReceived = true;
    control.handleRobotCommandReceipt(legacyReceipt, {});
    ok &= expectTrue(updates == 1, "unattributed legacy receipts remain compatible without invented trace");
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

/**
 * @brief 有序合成曝光经生产检测、投影、完整控制队列及模拟通信生成关联记录。
 * @details 真值不注入检测器；不实例化工件配准 worker，不连接真实设备。
 *          未传播的释放点曝光身份必须标缺，不能套用最近输入的帧号。
 */
/** @brief 正常凹形接缝应通过生产会话定位，不依赖仅支持凸折点的完整几何。 */
bool testLiutaoNormalVisibleSeam(QCoreApplication &app, bool competingStripe)
{
    using namespace i2r_rlc;
    ui::CVisionTrackingWorkerAdapter worker;
    ui::RuntimeTrackingOptions options;
    options.virtualStatus = control::TrackingVirtualStatus::OnlineRecord;
    options.visionDiagnostics.enabled = true;
    options.visionDiagnostics.imagesEnabled = false;
    options.sensorCalibration.valid = true;
    options.sensorCalibration.focalLengthM = 0.01;
    options.sensorCalibration.sxM = options.sensorCalibration.syM = 0.00001;
    options.sensorCalibration.cxPixel = 160.0;
    options.sensorCalibration.cyPixel = 120.0;
    options.sensorCalibration.laserPlaneC = -1.0;
    options.sensorCalibration.cameraZInToolM = -1.0;
    options.sensorCalibration.cameraXInToolM = 0.01;
    options.positionerCalibration.valid = true;
    std::vector<ui::RuntimeTrackingResult> results;
    QObject::connect(&worker, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app, [&](const ui::RuntimeTrackingResult &result) { results.push_back(result); });
    worker.configure(options, makeProjectionContext());
    worker.startTracking();
    bool ok = true;
    int reverseValid = 0;
    int onlineValid = 0;
    for (int index = 0; index < 16; ++index) {
        const double seamX = 156.0 + index * (competingStripe ? 1.5 : 0.5);
        const double seamY = 118.0 + index * 0.2;
        auto frame = makeFrame();
        frame.frameId = 3000 + index;
        frame.timestamp = QDateTime::fromMSecsSinceEpoch(1700001000000LL + index * 100);
        for (int y = 0; y < frame.image.height(); ++y) {
            auto *row = frame.image.scanLine(y);
            for (int x = 0; x < frame.image.width(); ++x) {
                const double dx = x - seamX;
                const double stripeY = seamY + (dx < 0.0 ? -0.28 : 0.32) * dx;
                const double dy = y - stripeY;
                double intensity = 220.0 * std::exp(-dy * dy / 2.88);
                if (competingStripe && index >= 4) {
                    // 同形弱反射保留在旧位置，主条纹仍清晰可见且随曝光连续运动。
                    const double oldDx = x - 156.0;
                    const double oldDy = y - (118.0 + (oldDx < 0.0 ? -0.28 : 0.32) * oldDx);
                    intensity = std::max(intensity, 95.0 * std::exp(-oldDy * oldDy / 2.88));
                }
                row[x] = static_cast<uchar>(15.0 + intensity);
            }
        }
        frame.previewImage = frame.image;
        digital_twin::DigitalTwinVisionLaserObservation observation;
        observation.valid = true;
        observation.frameId = frame.frameId;
        observation.seamIndex = 0;
        observation.hasWeldPointImage_px = true;
        observation.weldPointImage_px = {seamX, seamY};
        observation.laserLineImagePoints_px = {
            {30.0, seamY - 0.28 * (30.0 - seamX)}, {seamX, seamY},
            {300.0, seamY + 0.32 * (300.0 - seamX)}};
        ui::RuntimeVisionCorrelation correlation;
        correlation.sessionId = 903;
        correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
        correlation.frameId = frame.frameId;
        correlation.onlineControlWindowActive = index >= 6;
        correlation.workpiecePoseRevision = 18;
        auto state = makeReadyRobotState();
        state.curx_r = index * 1000;
        const auto previousCount = results.size();
        worker.submitFrameWithRobotStateObservationAndCorrelation(
            frame, state, -1, observation, correlation);
        app.processEvents(QEventLoop::AllEvents, 10);
        if (!expectTrue(results.size() == previousCount + 1,
                "each normal seam exposure produces a public runtime result")) {
            ok = false;
            continue;
        }
        const auto &result = results.back();
        if (result.diagnostic.has_value()) {
            const auto &algorithm = result.diagnostic->algorithm;
            const auto &tracker = algorithm.tracker;
            std::cerr << "Liutao trace frame=" << frame.frameId
                << " competing=" << competingStripe << " valid=" << result.valid
                << " expectedPixel=" << seamX << ',' << seamY
                << " coarse=" << algorithm.beforeFusionPoint.x << ',' << algorithm.beforeFusionPoint.y
                << " raw=" << algorithm.rawRefinement.point.x << ',' << algorithm.rawRefinement.point.y
                << " rawValid=" << algorithm.rawRefinement.valid
                << " final=" << result.imageXPixel << ',' << result.imageYPixel
                << " responseRoi=" << tracker.responseRoi.x << ',' << tracker.responseRoi.y
                << " beforeRoi=" << tracker.beforeFusionRoi.x << ',' << tracker.beforeFusionRoi.y
                << " finalRoi=" << tracker.finalRoi.x << ',' << tracker.finalRoi.y
                << " peak=" << tracker.responsePeak
                << " deferred=" << tracker.trainingDeferred
                << " trained=" << tracker.trainingCommitted
                << " trainingRejected=" << tracker.trainingRejected << '\n';
        }
        if (result.valid) {
            if (index < 6) { ++reverseValid; }
            else { ++onlineValid; }
        }
        if (index < 2) { continue; }
        // 此标定下激光平面每像素 1 mm，相机在 TCP 前方 10 mm；真值仅用于评分。
        const double expectedX = 10.0 + index + seamX - 160.0;
        const double expectedY = seamY - 120.0;
        const bool accurate = result.valid && result.diagnostic.has_value()
            && std::hypot(result.sample.point.xMm - expectedX,
                result.sample.point.yMm - expectedY) < 4.0;
        if (!accurate) {
            std::cerr << "Liutao normal seam frame=" << frame.frameId
                << " valid=" << result.valid << " expected=" << expectedX << ',' << expectedY
                << " actual=" << result.sample.point.xMm << ',' << result.sample.point.yMm << '\n';
        }
        ok &= expectTrue(accurate,
            "visible concave seam remains accurately tracked before and during online correction");
    }
    worker.stopTracking();
    ok &= expectTrue(reverseValid >= 4 && onlineValid == 10,
        "normal seam initializes during reverse scan and continues through the online window");
    return ok;
}

/** @brief 原图经正式 runtime 线程路由触发严重失测 HOLD，防止 Error 先失效会话吞掉请求。 */
bool testSevereLossThroughRuntime(QCoreApplication &app)
{
    Q_UNUSED(app);
    using namespace i2r_rlc;
    auto record = std::make_shared<FakeTransportState>();
    record->operationHoldReply = 0;
    record->reverseCompleteReply = 0;
    record->replayCorrectionFlags = 0;
    record->replayTcpXUm = 0;
    auto simulation = std::make_shared<FakeRobotSimTransportState>();
    simulation->centeredObservation = true;
    ui::CRuntimeCoordinator runtime(std::make_unique<CFakeRobotTransport>(record),
        std::make_unique<CFakeRobotSimTransport>(simulation));
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    makeOnlineConfigurationFiles(&configuration);
    const QString root = QDir::temp().filePath(QStringLiteral("i2r_runtime_loss_%1")
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
    configuration.diagnosticOutputDirectory = root.toStdString();
    configuration.visionDiagnostics.enabled = true;
    configuration.options.enableOnlineIcp = false;
    ui::RuntimeTrackingOptions options;
    options.sensorCalibration.valid = true;
    options.sensorCalibration.focalLengthM = 0.01;
    options.sensorCalibration.sxM = options.sensorCalibration.syM = 0.00001;
    options.sensorCalibration.cxPixel = 160;
    options.sensorCalibration.cyPixel = 120;
    options.sensorCalibration.laserPlaneC = -1;
    options.sensorCalibration.cameraZInToolM = -1;
    options.sensorCalibration.cameraXInToolM = 0.01;
    options.positionerCalibration.valid = true;
    QEventLoop loop;
    bool online = false, finishingInitial = false, advanced = false;
    bool requested = false, pending = false, confirmed = false, premature = false;
    bool requestUsedCurrentSession = false;
    quint64 sessionId = 0;
    int initialResults = 0, lostResults = 0, stopUpdates = 0;
    qint64 frameId = 0;
    const auto publishState = [&]() {
        auto state = makeReadyRobotState();
        state.curx_r = record->replayTcpXUm.load();
        state.corflag = state.detflag = record->replayCorrectionFlags.load();
        state.onlineReverseCompleteFlag = record->reverseCompleteReply.load();
        runtime.publishRobotState(state);
    };
    const auto submit = [&]() {
        if (requested || finishingInitial) { return; }
        auto frame = makeOnlineSeamFrame();
        frame.frameId = ++frameId;
        frame.timestamp = QDateTime::currentDateTime();
        if (online) { frame.image.fill(15); frame.previewImage = frame.image; }
        runtime.publishCameraFrame(frame);
    };
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::trackingResultReady, &loop,
        [&](const ui::RuntimeTrackingResult &result) {
            if (!online) {
                if (result.valid && !result.sample.estimated) { ++initialResults; }
                if (initialResults >= 4) {
                    finishingInitial = true;
                    record->reverseCompleteReply = 1;
                    publishState();
                } else { QTimer::singleShot(20, &loop, submit); }
            } else {
                ++lostResults;
                premature |= requested && !advanced;
                if (lostResults == 1) {
                    // 首次定位失败仍有初扫末端；等待队列事件处理后才推进真实设备。
                    QTimer::singleShot(50, &loop, [&]() {
                        premature |= requested;
                        advanced = true;
                        record->replayTcpXUm = 20000;
                        publishState();
                        submit();
                    });
                } else { QTimer::singleShot(20, &loop, submit); }
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineCorrectionUpdated, &loop,
        [&](const control::OnlineCorrectionUpdate &update) {
            if (update.phase == control::OnlineCorrectionPhase::InitialDetectionRunning) {
                sessionId = update.state.controlTarget.sessionId;
            }
            if (update.requestOperationStop) {
                ++stopUpdates;
                requested = true;
                premature |= !advanced;
                requestUsedCurrentSession = sessionId != 0
                    && sessionId == update.state.controlTarget.sessionId;
            }
            if (update.phase == control::OnlineCorrectionPhase::WaitingForInitialApproach) {
                record->replayCorrectionFlags = 2;
                publishState();
            }
            if (update.phase == control::OnlineCorrectionPhase::WaitingForOnlineStart) {
                // 按公开状态完成握手，不用短定时脉冲假设生产线程已处理 ENDDET。
                record->replayCorrectionFlags = 1;
                publishState();
            }
            if (!online && update.phase == control::OnlineCorrectionPhase::OnlineRunning) {
                online = true;
                finishingInitial = false;
                QTimer::singleShot(50, &loop, submit);
            }
        });
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::onlineOperationStopUpdated, &loop,
        [&](const robot_comm::RobotOperationStopState &state) {
            pending |= state.status == robot_comm::RobotOperationStopStatus::Pending;
            if (pending && state.status == robot_comm::RobotOperationStopStatus::Pending) {
                record->operationHoldReply = 1;
            }
            if (state.status == robot_comm::RobotOperationStopStatus::HoldConfirmed) {
                confirmed = !state.arcStopVerified;
                QTimer::singleShot(100, &loop, &QEventLoop::quit);
            }
        });
    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.configureOnlineCorrection(configuration);
    runtime.publishRobotState(makeRobotSimReachableState());
    const bool synchronized = connectCorrectionRobotSim(runtime, loop);
    publishState();
    runtime.connectRobot();
    runtime.startOnlineTracking();
    QTimer::singleShot(100, &loop, submit);
    QTimer::singleShot(15000, &loop, &QEventLoop::quit);
    loop.exec();
    runtime.stop();
    bool holdOnWire = false;
    for (std::size_t index = 0; index < record->requests.size(); ++index) {
        if (record->types[index] != robot_comm::RobotCommandType::StatusPoll) { continue; }
        const auto fields = QString::fromStdString(record->requests[index]).mid(2).split(',');
        holdOnWire |= fields.size() >= 7 && fields[6] == QStringLiteral("1");
    }
    return expectTrue(synchronized && initialResults >= 4 && online && lostResults > 0,
        "runtime loss fixture reaches online control through production raw-image detection and O receipts")
        && expectTrue(requested && stopUpdates == 1 && requestUsedCurrentSession && !premature,
            "current runtime session routes exactly one severe-loss stop after reliable spatial coverage expires")
        && expectTrue(pending && confirmed && holdOnWire,
            "runtime Error routing preserves actual N HOLD request and fresh device feedback");
}

bool testNormalSessionReplay(QCoreApplication &app, bool concaveMoving, bool simulationGap, int priorScenario,
    bool interference, int lifecycleScenario)
{
    const bool reflectionWithoutTack = QCoreApplication::arguments().contains(
        QStringLiteral("--without-tack"));
    const bool biasedEstimateRecovery = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-biased-estimate-recovery-session-contract"));
    const bool initialSevereLoss = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-initial-severe-loss-session-contract"));
    const bool unavailableImageLoss = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-unavailable-image-loss-session-contract"));
    const bool unavailableProjectionLoss = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-unavailable-projection-loss-session-contract"));
    const bool lateProjectionLoss = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-late-projection-loss-session-contract"));
    const bool severeLoss = initialSevereLoss || unavailableImageLoss || unavailableProjectionLoss || lateProjectionLoss
        || QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-severe-loss-session-contract"));
    const qint64 lossStartsAt = initialSevereLoss ? 5 : 17;
    const bool strongerReflection = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-strong-reflection-session-contract"));
    const bool equalReflection = strongerReflection || QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-equal-reflection-session-contract"));
    const double realStripeIntensity = strongerReflection ? 180.0 : 220.0;
    const bool matureStatistics = QCoreApplication::arguments().contains(
        QStringLiteral("--liutao-mature-normal-statistics-session-contract"));
    const int preweldExposureCount = matureStatistics ? 32 : 4;
    const std::size_t expectedExposureCount = static_cast<std::size_t>(preweldExposureCount + 24);
    const bool lifecycleReplay = lifecycleScenario == 1;
    const bool invalidReferenceGap = lifecycleScenario == 2;
    const bool referenceRevisionSwitch = lifecycleScenario == 3;
    const bool stationaryQueueWindow = lifecycleScenario == 4;
    using namespace i2r_rlc;
    const QString root = QDir::temp().filePath(QStringLiteral("i2r_rlc_normal_replay_%1")
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
    QDir().mkpath(root);
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.diagnosticOutputDirectory = root.toStdString();
    configuration.visionDiagnostics.enabled = true;
    configuration.visionDiagnostics.imagesEnabled = false;
    configuration.options.enableOnlineIcp = true;
    configuration.options.preserveMeasuredReference = true;
    configuration.rbcalibFile = (root + QStringLiteral("/RBCALIB.DAT")).toStdString();
    configuration.trajectoryRequest.workPathFile = (root + QStringLiteral("/case_W.txt")).toStdString();
    configuration.trajectoryRequest.teachPoseFile = (root + QStringLiteral("/case_T.txt")).toStdString();
    std::ofstream(configuration.rbcalibFile) << "////SRANG 0,0,0,0,0,0\n";
    std::ofstream(configuration.trajectoryRequest.teachPoseFile)
        << "0 0 0 0 0 0 0 0\n100 0 0 0 0 0 0 0\n";
    {
        std::ofstream path(configuration.trajectoryRequest.workPathFile);
        for (int index = 0; index <= 10; ++index) { path << index * 10 << " 0 0\n"; }
    }
    auto record = std::make_shared<FakeTransportState>();
    ui::CRobotCommWorkerAdapter robot(std::make_unique<CFakeRobotTransport>(record));
    ui::COnlineCorrectionWorkerAdapter control;
    ui::CVisionTrackingWorkerAdapter vision;
    ui::RuntimeTrackingOptions options;
    options.virtualStatus = control::TrackingVirtualStatus::OnlineRecord;
    options.visionDiagnostics = configuration.visionDiagnostics;
    options.sensorCalibration.valid = true;
    options.sensorCalibration.focalLengthM = 0.01;
    options.sensorCalibration.sxM = options.sensorCalibration.syM = 0.00001;
    options.sensorCalibration.cxPixel = 160.0;
    options.sensorCalibration.cyPixel = 120.0;
    options.sensorCalibration.laserPlaneC = -1.0;
    // 激光平面距离抵消相机 Z 偏置，中心曝光位于 TCP 前方 10 mm。
    options.sensorCalibration.cameraZInToolM = -1.0;
    options.sensorCalibration.cameraXInToolM = 0.01;
    options.positionerCalibration.valid = true;
    std::vector<ui::RuntimeTrackingResult> observations;
    std::vector<qint64> inputFrameIds;
    std::vector<quint64> inputReferenceRevisions;
    bool replayingExposure = false;
    std::optional<ui::RuntimeTrackingResult> replayResult;
    camera::CameraFrame savedFrame;
    robot_comm::RobotState savedState;
    digital_twin::DigitalTwinVisionLaserObservation savedObservation;
    ui::RuntimeVisionCorrelation savedCorrelation;
    std::vector<cv::Point2d> expectedSpacePoints;
    std::vector<cv::Point2d> expectedPixelPoints;
    std::vector<qint64> scoredFrameIds;
    std::vector<control::OnlineCorrectionUpdate> updates;
    qint64 frameId = 0;
    bool lossStopRequested = false;
    bool prematureLossStop = false;
    int lossStopRequests = 0;
    robot_comm::RobotOperationStopState lossStopState;
    QObject::connect(&control, &ui::COnlineCorrectionWorkerAdapter::operationStopRequested,
        &app, [&](quint64 sessionId, const QString &) {
            ++lossStopRequests;
            prematureLossStop |= sessionId != 902 || frameId < lossStartsAt;
            lossStopRequested = true;
            control.stopForOperation();
            robot.requestControlledHold();
        });
    QObject::connect(&robot, &ui::CRobotCommWorkerAdapter::operationStopUpdated,
        &app, [&](const robot_comm::RobotOperationStopState &value) {
            lossStopState = value;
            control.recordOperationStop(value);
        });
    QObject::connect(&vision, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app, [&](const ui::RuntimeTrackingResult &result) {
            if (replayingExposure) { replayResult = result; }
            else { observations.push_back(result); }
        });
    QObject::connect(&vision, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &control, &ui::COnlineCorrectionWorkerAdapter::appendTrackingResult);
    QObject::connect(&control, &ui::COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
        &app, [&](const control::OnlineCorrectionUpdate &update) { updates.push_back(update); });
    QObject::connect(&control, &ui::COnlineCorrectionWorkerAdapter::robotCommandReady,
        &robot, &ui::CRobotCommWorkerAdapter::sendCorrectionCommand);
    QObject::connect(&control, &ui::COnlineCorrectionWorkerAdapter::realtimeCommandReady,
        &robot, &ui::CRobotCommWorkerAdapter::sendRealtimeCorrectionCommand);
    QObject::connect(&robot, &ui::CRobotCommWorkerAdapter::robotStateReady,
        &control, &ui::COnlineCorrectionWorkerAdapter::updateRobotState);
    QObject::connect(&robot, &ui::CRobotCommWorkerAdapter::commandReceiptReady,
        &control, &ui::COnlineCorrectionWorkerAdapter::handleRobotCommandReceipt);
    robot.connectRobot();
    control.setControlSessionId(902);
    control.configure(configuration);
    control.startInitialDetection();
    vision.configure(options, makeProjectionContext());
    vision.startTracking();
    auto state = makeReadyRobotState();
    record->replayPollState = state;
    const auto submit = [&](bool online) {
        const auto exposureStarted = std::chrono::steady_clock::now();
        auto frame = makeOnlineSeamFrame();
        // 模拟背压保留的新曝光：ID 单调递增但不连续，原图输入仍完整。
        frameId += priorScenario == 6 ? 7 : 1;
        frame.frameId = frameId;
        frame.timestamp = QDateTime::fromMSecsSinceEpoch(1700000000000LL + frameId * 100);
        digital_twin::DigitalTwinVisionLaserObservation observation;
        observation.valid = true;
        observation.frameId = frame.frameId;
        observation.seamIndex = 0;
        observation.hasWeldPointImage_px = true;
        observation.weldPointImage_px = {160.0, 120.0};
        observation.laserLineImagePoints_px = {{30.0, 83.6}, {160.0, 120.0}, {300.0, 75.2}};
        double seamX = 160.0;
        double seamY = 120.0;
        if (concaveMoving) {
            seamX = 156.0 + (frameId - 1) * (interference ? 1.5 : 0.25);
            seamY = 118.0 + (frameId - 1) * 0.1;
            for (int y = 0; y < frame.image.height(); ++y) {
                auto *row = frame.image.scanLine(y);
                for (int x = 0; x < frame.image.width(); ++x) {
                    // 预焊表面使可见肩部出现4px横向偏差，评分仍以被遮住的母材接缝为准。
                    const double visibleShift = biasedEstimateRecovery && frameId >= 14 && frameId <= 17 ? 4.0 : 0.0;
                    const double dx = x - seamX - visibleShift;
                    const double dy = y - seamY - (dx < 0.0 ? -0.28 : 0.32) * dx;
                    double intensity = realStripeIntensity * std::exp(-dy * dy / 2.88);
                    if (interference && frameId >= 9 && frameId <= 20) {
                        // 竞争条纹只改变当前原图；同一焊缝与仿真参考不切换。
                        const double ghostDx = x - 156.0;
                        const double ghostDy = y - 118.0
                            - (ghostDx < 0.0 ? -0.28 : 0.32) * ghostDx;
                        // 仅把旧位置竞争条纹提升至与主条纹等亮；真值、运动和评分不变。
                        const double reflectionIntensity = equalReflection ? 220.0 : 95.0;
                        intensity = std::max(intensity,
                            reflectionIntensity * std::exp(-ghostDy * ghostDy / 2.88));
                    }
                    if (interference && !reflectionWithoutTack
                        && frameId >= 14 && frameId <= 17 && std::abs(dx) < 16.0) {
                        // 预焊凸起局部遮住母材接缝，两侧可见条纹仍保留。
                        const double surfaceDy = dy + 9.0 * (1.0 - std::abs(dx) / 16.0);
                        intensity = realStripeIntensity * std::exp(-surfaceDy * surfaceDy / 2.88);
                    }
                    row[x] = static_cast<uchar>(15.0 + intensity);
                }
            }
            frame.previewImage = frame.image;
            observation.weldPointImage_px = {seamX, seamY};
            observation.laserLineImagePoints_px = {
                {30.0, seamY - 0.28 * (30.0 - seamX)}, {seamX, seamY},
                {300.0, seamY + 0.32 * (300.0 - seamX)}};
        }
        if (priorScenario == 1) {
            // 装夹位置明显偏离仿真：仿真焊点落在图外，局部斜率/形状仍有意义。
            // 仅变换外部仿真输入，原图和独立评分真值不变。
            observation.weldPointImage_px.x_px += 220.0;
            observation.weldPointImage_px.y_px -= 12.0;
            for (auto &point : observation.laserLineImagePoints_px) {
                point.x_px += 220.0;
                point.y_px -= 12.0;
            }
        }
        if (priorScenario == 2) {
            // 仿真位置仍在图内但与工件摆放相差明显；有效局部形状不能固定拉偏实测。
            observation.weldPointImage_px.x_px += 48.0;
            observation.weldPointImage_px.y_px += 18.0;
            for (auto &point : observation.laserLineImagePoints_px) {
                point.x_px += 48.0;
                point.y_px += 18.0;
            }
        }
        if (priorScenario == 3) {
            // 仅仿真方向错误：保持点坐标，局部条纹旋转25度；原图和真值不变。
            const double cosine = std::cos(25.0 * 3.14159265358979323846 / 180.0);
            const double sine = std::sin(25.0 * 3.14159265358979323846 / 180.0);
            for (auto &point : observation.laserLineImagePoints_px) {
                const double dx = point.x_px - observation.weldPointImage_px.x_px;
                const double dy = point.y_px - observation.weldPointImage_px.y_px;
                point.x_px = observation.weldPointImage_px.x_px + cosine * dx - sine * dy;
                point.y_px = observation.weldPointImage_px.y_px + sine * dx + cosine * dy;
            }
        }
        if (priorScenario == 5 || priorScenario == 7) {
            // 中途只改变仿真位置，旧局部偏差必须失效，原图与独立真值不变。
            const double shiftX = frameId < 14 ? 48.0 : -72.0;
            const double shiftY = frameId < 14 ? 18.0 : 32.0;
            observation.weldPointImage_px.x_px += shiftX;
            observation.weldPointImage_px.y_px += shiftY;
            for (auto &point : observation.laserLineImagePoints_px) {
                point.x_px += shiftX;
                point.y_px += shiftY;
            }
        }
        if (priorScenario == 4 && frameId <= 2) {
            // 初始两次同帧仿真外观缺失，清晰原图与曝光姿态仍完整；不丢弃评分帧。
            observation.hasWeldPointImage_px = false;
            observation.laserLineImagePoints_px.clear();
        }
        if (simulationGap && frameId >= 10 && frameId <= 11) {
            // 同一焊缝的仿真外观暂缺，不伪造参考切换；位置真值仍只进入评分器。
            observation.hasWeldPointImage_px = false;
            observation.laserLineImagePoints_px.clear();
        }
        if (invalidReferenceGap && frameId >= 10 && frameId <= 11) {
            // 外部仿真暂时失败，原图、曝光姿态与参考版本仍连续。
            observation = digital_twin::makeInvalidVisionLaserObservation(
                frame.frameId, "temporary simulation failure");
        }
        if (referenceRevisionSwitch && frameId >= 12) {
            // 仅外部CAD版本及绝对位置跳变；当前原图和真实曝光运动保持连续。
            const double shiftX = frameId < 21 ? 48.0 : -35.0;
            const double shiftY = frameId < 21 ? 18.0 : -15.0;
            observation.weldPointImage_px.x_px += shiftX;
            observation.weldPointImage_px.y_px += shiftY;
            for (auto &point : observation.laserLineImagePoints_px) {
                point.x_px += shiftX;
                point.y_px += shiftY;
            }
        }
        // 独立已知夹具位置仅进入评分器；曝光姿态仍由真实 worker 构建投影。
        if ((unavailableProjectionLoss || lateProjectionLoss) && frameId == 27) {
            options.sensorCalibration.focalLengthM = 0.01;
            options.sensorCalibration.laserPlaneC = -1.0;
            vision.configure(options, makeProjectionContext());
        }
        if (severeLoss && frameId >= lossStartsAt && frameId <= 26) {
            if (!lateProjectionLoss) { frame.image.fill(15); }
            frame.previewImage = frame.image;
            if (unavailableImageLoss) { frame.image = {}; frame.previewImage = {}; }
            if (unavailableProjectionLoss && frameId == lossStartsAt) {
                options.sensorCalibration.focalLengthM = 0.0;
                vision.configure(options, makeProjectionContext());
            }
            if (lateProjectionLoss && frameId == lossStartsAt) {
                // 上下文仍有效、原图清晰，但激光面交点位于相机后方，最终空间投影失败。
                options.sensorCalibration.laserPlaneC = 1.0;
                vision.configure(options, makeProjectionContext());
            }
            if (!lateProjectionLoss) {
                observation.hasWeldPointImage_px = false;
                observation.laserLineImagePoints_px.clear();
            }
        }
        expectedSpacePoints.emplace_back(state.curx_r / 1000.0 + 10.0 + seamX - 160.0,
            seamY - 120.0);
        expectedPixelPoints.emplace_back(seamX, seamY);
        scoredFrameIds.push_back(frame.frameId);
        ui::RuntimeVisionCorrelation correlation;
        correlation.sessionId = 902;
        correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
        correlation.frameId = frame.frameId;
        correlation.onlineControlWindowActive = online;
        correlation.workpiecePoseRevision = referenceRevisionSwitch
            ? (frameId < 12 ? 17 : (frameId < 21 ? 18 : 19)) : 17;
        if ((lifecycleReplay && frameId == 10) || matureStatistics) {
            savedFrame = frame;
            savedState = state;
            savedObservation = observation;
            savedCorrelation = correlation;
        }
        if (lifecycleReplay && frameId == 12) {
            // 旧曝光仍是原图/原位姿/原参考版本；变化的过期缝号不得重置模型。
            auto expiredObservation = savedObservation;
            expiredObservation.seamIndex = 1;
            replayingExposure = true;
            inputFrameIds.push_back(savedFrame.frameId);
            inputReferenceRevisions.push_back(savedCorrelation.workpiecePoseRevision);
            vision.submitFrameWithRobotStateObservationAndCorrelation(
                savedFrame, savedState, -1, expiredObservation, savedCorrelation);
            replayingExposure = false;
        }
        inputFrameIds.push_back(frame.frameId);
        inputReferenceRevisions.push_back(correlation.workpiecePoseRevision);
        vision.submitFrameWithRobotStateObservationAndCorrelation(frame, state, -1, observation, correlation);
        if ((priorScenario != 0 || interference || referenceRevisionSwitch)
            && !observations.empty() && observations.back().diagnostic.has_value()) {
            const auto &result = observations.back();
            const auto &tracker = result.diagnostic->algorithm.tracker;
            const double elapsedMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - exposureStarted).count();
            std::cerr << "prior-frame=" << frame.frameId << " valid=" << result.valid
                << " algorithm=" << result.activeAlgorithm.toStdString()
                << " point=" << result.imageXPixel << ',' << result.imageYPixel
                << " templates=" << tracker.responseTemplateCount
                << " train=" << tracker.trainingCommitted << '/' << tracker.trainingRejected
                << " weights=" << tracker.responseHogWeight << ',' << tracker.responseGrayWeight
                << " psr=" << tracker.responsePsr
                << " responseNormal=" << result.diagnostic->algorithm.responseNormal
                << " normalSamples=" << result.diagnostic->algorithm.normalResponseSamples
                << " normalMean=" << result.diagnostic->algorithm.normalResponseMean
                << " normalStd=" << result.diagnostic->algorithm.normalResponseStd
                << " identity=" << result.diagnostic->algorithm.coarseObservationQuality
                << " coarse=" << result.diagnostic->algorithm.beforeFusionPoint.x
                << ',' << result.diagnostic->algorithm.beforeFusionPoint.y
                << " simulationTraining=" << tracker.trainingFromSimulation
                << " shoulder=" << result.diagnostic->algorithm.shoulderObservationAccepted
                << '/' << result.diagnostic->algorithm.shoulderObservationQuality
                << " center=" << result.diagnostic->algorithm.centerObservationQuality
                << " estimated=" << result.sample.estimated
                << " fullAppearance=" << result.diagnostic->algorithm.completeAppearanceSupported
                << " elapsed_ms=" << elapsedMs << std::endl;
        }
    };
    for (int index = 0; index < preweldExposureCount; ++index) { submit(false); }
    state.onlineReverseCompleteFlag = 1;
    record->replayPollState = state;
    robot.updateState(state);
    bool ok = expectTrue(!updates.empty()
        && updates.back().phase == control::OnlineCorrectionPhase::WaitingForInitialApproach,
        "raw production detections must prepare and acknowledge initial O trajectory");
    state.corflag = state.detflag = 2;
    record->replayPollState = state;
    robot.updateState(state);
    state.corflag = state.detflag = 1;
    record->replayPollState = state;
    robot.updateState(state);
    std::optional<control::OnlineCorrectionPoint> stationaryFront;
    std::size_t stationaryQueueSize = 0;
    bool stationaryReferencePreserved = true;
    if (matureStatistics) {
        vision.stopTracking();
        options.requireDetectionWindow = false;
        vision.configure(options, makeProjectionContext());
        vision.startTracking();
    }
    for (int index = 0; index < 24; ++index) {
        // 外部设备保持同一位置，仍提供不同的新曝光；队列只能按运动进度消费。
        const int motionIndex = stationaryQueueWindow && index >= 9 && index <= 21 ? 8 : index;
        state.curx_r = motionIndex * 5000;
        record->replayPollState = state;
        robot.updateState(state);
        submit(true);
        app.processEvents(QEventLoop::AllEvents, 10);
        if (severeLoss && frameId == lossStartsAt) {
            ok &= expectTrue(!lossStopRequested,
                "lost image support does not stop while a reliable forward trajectory remains");
        }
        if (stationaryQueueWindow && index >= 10 && index <= 21 && !updates.empty()) {
            const auto &queue = updates.back().state.onlineDetectionSamples;
            if (index == 10 && queue.size() > 1) {
                stationaryFront = queue.front().point;
                stationaryQueueSize = queue.size();
            } else if (stationaryFront.has_value()) {
                stationaryReferencePreserved &= queue.size() >= stationaryQueueSize
                    && !queue.empty()
                    && std::abs(queue.front().point.xMm - stationaryFront->xMm) < 1.0e-9
                    && std::abs(queue.front().point.yMm - stationaryFront->yMm) < 1.0e-9
                    && std::abs(queue.front().point.zMm - stationaryFront->zMm) < 1.0e-9;
            }
        }
    }
    if (stationaryQueueWindow) {
        ok &= expectTrue(stationaryFront.has_value() && stationaryReferencePreserved,
            "fresh exposures at a stationary robot cannot consume an unpassed forward reference");
    }
    // 真值只存在于测试夹具：保留每次输入，包括无输出/估计/遮挡，不以成功子集冒充全程。
    QJsonArray scoredExposures;
    const auto scoreRange = [&](std::size_t begin, std::size_t end) {
        int validCount = 0, estimateCount = 0, missingCount = 0;
        double totalSpaceError = 0.0, maxSpaceError = 0.0;
        double totalPixelError = 0.0, maxPixelError = 0.0;
        for (std::size_t index = begin; index < end; ++index) {
            if (index >= observations.size() || !observations[index].valid
                || !observations[index].sample.point.valid) { ++missingCount; continue; }
            const auto &result = observations[index];
            const double spaceError = std::hypot(result.sample.point.xMm - expectedSpacePoints[index].x,
                result.sample.point.yMm - expectedSpacePoints[index].y, result.sample.point.zMm);
            const double pixelError = std::hypot(result.imageXPixel - expectedPixelPoints[index].x,
                result.imageYPixel - expectedPixelPoints[index].y);
            if (!std::isfinite(spaceError) || !std::isfinite(pixelError)) { ++missingCount; continue; }
            ++validCount;
            estimateCount += result.sample.estimated ? 1 : 0;
            totalSpaceError += spaceError;
            maxSpaceError = std::max(maxSpaceError, spaceError);
            totalPixelError += pixelError;
            maxPixelError = std::max(maxPixelError, pixelError);
        }
        return QJsonObject{{QStringLiteral("input_count"), static_cast<int>(end - begin)},
            {QStringLiteral("valid_count"), validCount}, {QStringLiteral("missing_count"), missingCount},
            {QStringLiteral("estimate_count"), estimateCount},
            {QStringLiteral("mean_detection_mm"), validCount ? QJsonValue(totalSpaceError / validCount) : QJsonValue()},
            {QStringLiteral("max_detection_mm"), validCount ? QJsonValue(maxSpaceError) : QJsonValue()},
            {QStringLiteral("mean_pixel_error"), validCount ? QJsonValue(totalPixelError / validCount) : QJsonValue()},
            {QStringLiteral("max_pixel_error"), validCount ? QJsonValue(maxPixelError) : QJsonValue()}};
    };
    for (std::size_t index = 0; index < expectedSpacePoints.size(); ++index) {
        const bool present = index < observations.size();
        const auto &truth = expectedSpacePoints[index];
        QJsonObject sample{{QStringLiteral("frame_id"), scoredFrameIds[index]},
            {QStringLiteral("truth_space_mm"), QJsonArray{truth.x, truth.y, 0.0}},
            {QStringLiteral("truth_pixel"), QJsonArray{expectedPixelPoints[index].x, expectedPixelPoints[index].y}},
            {QStringLiteral("valid"), present && observations[index].valid},
            {QStringLiteral("estimated"), present && observations[index].sample.estimated}};
        if (present && observations[index].valid) {
            const auto &point = observations[index].sample.point;
            sample.insert(QStringLiteral("output_space_mm"), QJsonArray{point.xMm, point.yMm, point.zMm});
            sample.insert(QStringLiteral("output_pixel"), QJsonArray{observations[index].imageXPixel, observations[index].imageYPixel});
        }
        scoredExposures.append(sample);
    }
    const QJsonObject score{{QStringLiteral("evidence_kind"), QStringLiteral("synthetic_raw_image_production_session")},
        {QStringLiteral("scenario"), QCoreApplication::arguments().value(1)},
        {QStringLiteral("all_exposures"), scoreRange(0, expectedSpacePoints.size())},
        {QStringLiteral("preweld_tack"), interference && !reflectionWithoutTack ? scoreRange(13, 17) : QJsonObject{}},
        {QStringLiteral("operation_stop_requests"), lossStopRequests},
        {QStringLiteral("false_stop"), !severeLoss && lossStopRequested},
        {QStringLiteral("missed_stop"), severeLoss && !lossStopRequested},
        {QStringLiteral("actual_tcp_accuracy"), QJsonValue()},
        {QStringLiteral("field_accuracy_verified"), false},
        {QStringLiteral("exposures"), scoredExposures}};
    std::cout << "SESSION_SCORE " << QJsonDocument(score).toJson(QJsonDocument::Compact).constData() << '\n';
    if (severeLoss) {
        ok &= expectTrue(lossStopRequested && lossStopRequests == 1 && !prematureLossStop,
            "failed image recovery and spatially exhausted reliable trajectory request one operation HOLD");
        bool holdOnWire = false;
        for (std::size_t index = 0; index < record->requests.size(); ++index) {
            if (record->types[index] != robot_comm::RobotCommandType::StatusPoll) { continue; }
            const auto fields = QString::fromStdString(record->requests[index]).mid(2).split(',');
            holdOnWire |= fields.size() >= 7 && fields[6] == QStringLiteral("1");
        }
        ok &= expectTrue(holdOnWire && lossStopState.holdRequestLatched
                && lossStopState.status == robot_comm::RobotOperationStopStatus::Pending,
            "severe loss sends actual N HOLD and waits for fresh device confirmation");
        record->operationHoldReply = 1;
        robot.pollRobotState();
        ok &= expectTrue(lossStopState.status == robot_comm::RobotOperationStopStatus::HoldConfirmed
                && !lossStopState.arcStopVerified,
            "fresh device HOLD confirms motion holding without claiming arc stop");
        ok &= expectTrue(observations.size() == expectedExposureCount && observations.back().valid
                && lossStopRequests == 1,
            "new clear images can recover detection without automatically resuming the stopped operation");
        const auto directories = QDir(root).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        if (!expectTrue(directories.size() == 1, "loss run retains one recorder session")) { return false; }
        const QString session = QDir(root).filePath(directories.first());
        QFile report(QDir(session).filePath(QStringLiteral("unified_session.jsonl")));
        if (!expectTrue(report.open(QIODevice::ReadOnly), "loss run retains its unified report")) { return false; }
        bool requested = false, confirmed = false, ended = false, recordedLoss = false;
        int recordedObservations = 0;
        for (const auto &line : report.readAll().split('\n')) {
            if (line.isEmpty()) { continue; }
            const auto event = QJsonDocument::fromJson(line).object();
            const auto type = event.value(QStringLiteral("event")).toString();
            if (type == QStringLiteral("observation")) {
                ++recordedObservations;
                recordedLoss |= !event.value(QStringLiteral("valid")).toBool();
                const auto exposure = event.value(QStringLiteral("frame_id")).toString().toLongLong();
                if ((unavailableImageLoss || unavailableProjectionLoss)
                    && exposure >= lossStartsAt && exposure <= 26) {
                    const auto evidence = event.value(QStringLiteral("localization_evidence")).toObject();
                    ok &= expectTrue(evidence.value(QStringLiteral("input_unavailable")).toBool()
                        && !evidence.value(QStringLiteral("attempted")).toBool()
                        && !evidence.value(QStringLiteral("recovery_attempted")).toBool(),
                        "missing inputs remain distinct from attempted image recovery in the session report");
                }
                if (lateProjectionLoss && exposure >= lossStartsAt && exposure <= 26) {
                    const auto evidence = event.value(QStringLiteral("localization_evidence")).toObject();
                    ok &= expectTrue(evidence.value(QStringLiteral("input_unavailable")).toBool()
                        && evidence.value(QStringLiteral("attempted")).toBool()
                        && evidence.value(QStringLiteral("current_image_supported")).toBool()
                        && !event.value(QStringLiteral("control_eligible")).toBool(),
                        "failed space projection preserves actual image support but cannot authorize control");
                }
            } else if (type == QStringLiteral("operation_stop")) {
                const auto status = event.value(QStringLiteral("status")).toString();
                requested |= status == QStringLiteral("pending");
                confirmed |= requested && !ended && status == QStringLiteral("hold_confirmed");
                ok &= expectTrue(event.value(QStringLiteral("session_id")).toString() == QStringLiteral("902")
                    && !event.value(QStringLiteral("arc_stop_verified")).toBool(),
                    "loss HOLD report retains session identity and does not claim arc stop");
            } else if (type == QStringLiteral("session_end")) {
                ended = true;
                ok &= expectTrue(confirmed, "loss report closes only after fresh HOLD confirmation");
            }
        }
        ok &= expectTrue(recordedObservations >= lossStartsAt && recordedLoss && requested && confirmed && ended,
            "unified loss report preserves failed observations and pending-to-confirmed stop sequence");
        std::cout << "Severe loss synthetic session report: " << session.toStdString() << std::endl;
        vision.stopTracking();
        return ok;
    }
    if (lifecycleReplay) {
        ok &= expectTrue(replayResult.has_value() && !replayResult->valid
                && !replayResult->imageObservationAccepted
                && replayResult->diagnostic.has_value()
                && !replayResult->diagnostic->algorithm.templateReinitialized
                && !replayResult->diagnostic->algorithm.tracker.trainingCommitted,
            "expired exposure with changed seam metadata cannot reinitialize, train or create a new control observation");
    }
    if (invalidReferenceGap) {
        const bool continuityPreserved = observations.size() == expectedExposureCount
            && observations.at(11).diagnostic.has_value()
            && !observations.at(11).diagnostic->algorithm.templateReinitialized;
        ok &= expectTrue(continuityPreserved,
            "returning simulation reference does not discard the established image target");
    }
    if (referenceRevisionSwitch) {
        bool sameImageTarget = observations.size() == expectedExposureCount;
        for (const std::size_t index : {std::size_t{11}, std::size_t{20}}) {
            sameImageTarget &= observations.size() > index
                && observations.at(index).valid && observations.at(index).imageObservationAccepted
                && !observations.at(index).sample.estimated
                && observations.at(index).diagnostic.has_value()
                && !observations.at(index).diagnostic->algorithm.templateReinitialized;
        }
        ok &= expectTrue(sameImageTarget,
            "CAD revision switches preserve the current image target without a template restart");
    }
    ok &= expectTrue(!lossStopRequested && lossStopRequests == 0,
        "normal, misaligned and image-constrained interference sessions must not request operation HOLD");
    std::size_t releasedCount = 0;
    bool consumedReference = false;
    bool attemptedTrajectoryIcp = false;
    for (const auto &update : updates) {
        for (const auto &batch : update.onlineIcpBatches) {
            releasedCount += batch.releasedSamples.size();
            attemptedTrajectoryIcp |= batch.attempted;
        }
        consumedReference |= update.followDiagnostic.valid && update.realtimeCommand.valid;
    }
    ok &= expectTrue(observations.size() == expectedExposureCount && observations.back().valid
        && observations.back().diagnostic.has_value() && observations.back().sample.point.xMm > 120.0,
        "each exposure reaches production detection and pose-dependent space projection");
    ok &= expectTrue(releasedCount > 5 && consumedReference && attemptedTrajectoryIcp,
        "production processing releases and consumes the full forward queue, not a local snapshot");
    if (concaveMoving) {
        double totalError = 0.0;
        double maxError = 0.0;
        bool allCurrentImageObservations = observations.size() == expectedSpacePoints.size();
        bool truthfulAlgorithm = observations.size() >= 2;
        bool actualModelLifecycle = true;
        bool bothFeaturesParticipated = false;
        double hogMinimum = std::numeric_limits<double>::max();
        double hogMaximum = 0.0;
        double grayMinimum = std::numeric_limits<double>::max();
        double grayMaximum = 0.0;
        for (std::size_t index = 0; index < observations.size(); ++index) {
            const auto &observation = observations[index];
            allCurrentImageObservations &= observation.valid && observation.controlEligible
                && observation.imageObservationAccepted && !observation.sample.estimated
                && !observation.independentWeldPointInParent_mm.valid;
            const bool singleTemplateResponse = index == 0
                || (matureStatistics && index == static_cast<std::size_t>(preweldExposureCount))
                || ((simulationGap || invalidReferenceGap) && (index == 10 || index == 11));
            const int expectedTemplates = singleTemplateResponse ? 1 : 3;
            if (priorScenario == 0) {
                truthfulAlgorithm &= observation.activeAlgorithm
                    == QStringLiteral("liutao_mkcf%1_hog_gray_observation").arg(expectedTemplates);
            } else {
                truthfulAlgorithm &= observation.activeAlgorithm.startsWith(QStringLiteral("liutao_mkcf"))
                    && observation.activeAlgorithm.endsWith(QStringLiteral("_hog_gray_observation"));
            }
            if (observation.diagnostic.has_value()) {
                const auto &model = observation.diagnostic->algorithm.tracker;
                actualModelLifecycle &= (priorScenario != 0 || model.responseTemplateCount == expectedTemplates)
                    && model.responseTemplateCount >= 1 && model.responseTemplateCount <= 3
                    && model.trainingCommitted && !model.trainingRejected
                    && std::isfinite(model.responseHogWeight) && model.responseHogWeight >= 0.0
                    && std::isfinite(model.responseGrayWeight) && model.responseGrayWeight >= 0.0;
                bothFeaturesParticipated |= model.responseHogWeight > 0.0
                    && model.responseGrayWeight > 0.0;
                hogMinimum = std::min(hogMinimum, model.responseHogWeight);
                hogMaximum = std::max(hogMaximum, model.responseHogWeight);
                grayMinimum = std::min(grayMinimum, model.responseGrayWeight);
                grayMaximum = std::max(grayMaximum, model.responseGrayWeight);
            } else { actualModelLifecycle = false; }
            if (priorScenario != 0 && observation.diagnostic.has_value()) {
                const auto &fusion = observation.diagnostic->algorithm;
                const bool hasPreviousReference = index > (priorScenario == 4 ? 2U : 0U);
                const bool changedPrior = (priorScenario == 5 || priorScenario == 7) && index == 13;
                const bool partialOrFirstRecovery = priorScenario == 7 && index >= 13 && index <= 17;
                ok &= expectTrue(fusion.positionFusionApplied
                        == (hasPreviousReference && !changedPrior && !partialOrFirstRecovery)
                    && fusion.simulationPositionTrusted == fusion.positionFusionApplied,
                    "trusted simulation prior uses an earlier exposure and expires on local bias jump");
                if (fusion.positionFusionApplied) {
                    const cv::Point2d expectedFusion(
                        fusion.positionPriorPoint.x + fusion.positionGainX
                            * (fusion.positionObservationPoint.x - fusion.positionPriorPoint.x),
                        fusion.positionPriorPoint.y + fusion.positionGainY
                            * (fusion.positionObservationPoint.y - fusion.positionPriorPoint.y));
                    ok &= expectTrue(std::isfinite(fusion.positionGainX) && fusion.positionGainX > 0.0 && fusion.positionGainX <= 1.0
                        && std::isfinite(fusion.positionGainY) && fusion.positionGainY > 0.0 && fusion.positionGainY <= 1.0
                        && cv::norm(fusion.finalPoint - expectedFusion) < 1e-6,
                        "paper fusion diagnostics explain the finite Kalman innovation update");
                }
            }
            const auto &expected = expectedSpacePoints[index];
            const double error = std::hypot(observation.sample.point.xMm - expected.x,
                observation.sample.point.yMm - expected.y);
            totalError += error;
            maxError = std::max(maxError, error);
        }
        ok &= expectTrue(interference || allCurrentImageObservations,
            "current-image MKCF observations feed control while independent geometry remains unavailable");
        ok &= expectTrue(interference || truthfulAlgorithm,
            "full session reports the actual three-template dual-kernel model used for each response");
        ok &= expectTrue(interference || (actualModelLifecycle && bothFeaturesParticipated),
            "actual trained response models preserve 1/3-template lifecycle and learned HOG/gray contributions");
        std::cout << "MKCF response evidence: simulation_gap=" << simulationGap
            << " hog_range=" << hogMinimum << ',' << hogMaximum
            << " gray_range=" << grayMinimum << ',' << grayMaximum << '\n';
        ok &= expectTrue(!observations.empty() && totalError / observations.size() <= 2.0
                && maxError <= 5.0,
            "every normal moving exposure is scored against independent fixture truth without discarding failures");
    }
    if (interference) {
        if (biasedEstimateRecovery && observations.size() == expectedExposureCount) {
            double maximumEstimateBias = 0.0;
            for (std::size_t index = 13; index <= 16; ++index) {
                maximumEstimateBias = std::max(maximumEstimateBias,
                    std::abs(observations[index].imageXPixel - expectedPixelPoints[index].x));
            }
            ok &= expectTrue(maximumEstimateBias >= 2.0,
                "recovery fixture actually creates a visible localization bias during occlusion");
            ok &= expectTrue(observations[17].valid && observations[17].imageObservationAccepted
                    && !observations[17].sample.estimated
                    && std::hypot(observations[17].imageXPixel - expectedPixelPoints[17].x,
                        observations[17].imageYPixel - expectedPixelPoints[17].y) <= 1.5,
                "first visible same-target observation corrects the wrong estimate without display smoothing delay");
        }
        for (std::size_t index = 0; index < observations.size(); ++index) {
            const auto &result = observations[index];
            const auto &expected = expectedSpacePoints[index];
            std::cerr << "Interference full session frame=" << index + 1
                << " valid=" << result.valid << " estimated=" << result.sample.estimated
                << " expected=" << expected.x << ',' << expected.y
                << " actual=" << result.sample.point.xMm << ',' << result.sample.point.yMm << '\n';
        }
        bool truthfulTackEstimates = observations.size() == expectedExposureCount;
        bool currentImageRecovered = observations.size() == expectedExposureCount;
        bool noAbnormalImageTraining = observations.size() == expectedExposureCount;
        if (observations.size() == expectedExposureCount) {
            for (std::size_t index = 8; index <= 19; ++index) {
                const auto &result = observations[index];
                noAbnormalImageTraining &= result.diagnostic.has_value()
                    && result.diagnostic->algorithm.tracker.trainingFromSimulation;
            }
            for (std::size_t index = 13; !reflectionWithoutTack && index <= 16; ++index) {
                const auto &result = observations[index];
                truthfulTackEstimates &= result.valid && result.controlEligible
                    && result.sample.estimated && !result.imageObservationAccepted
                    && result.pointSource == vision::OnlineSeamPointSource::GuidedEstimate
                    && !result.independentWeldPointInParent_mm.valid
                    && result.diagnostic.has_value()
                    && !result.diagnostic->algorithm.evidence.independentGeometry;
            }
            // Frame 21 is the first clear exposure after interference; do not only check the end.
            for (std::size_t index = 20; index < observations.size(); ++index) {
                const auto &result = observations[index];
                currentImageRecovered &= result.valid && result.imageObservationAccepted
                    && !result.sample.estimated;
            }
        }
        ok &= expectTrue(truthfulTackEstimates,
            "preweld shoulder constraints remain estimates without independent geometry");
        ok &= expectTrue(noAbnormalImageTraining,
            "reflection and preweld appearances never update trusted real templates");
        ok &= expectTrue(currentImageRecovered,
            "first clear exposure and subsequent frames recover current-image observation");
        ok &= expectTrue(observations.size() == expectedExposureCount
            && observations.back().valid && observations.back().imageObservationAccepted
            && !observations.back().sample.estimated,
            "same-session tracking returns to current-image observation after preweld and reflection");
    }
    if (QCoreApplication::arguments().contains(QStringLiteral("--liutao-normal-training-source-contract"))) {
        bool everyNormalOnlineFrameTrainsRealImage = observations.size() == expectedExposureCount;
        for (std::size_t index = static_cast<std::size_t>(preweldExposureCount); index < observations.size(); ++index) {
            const auto &result = observations[index];
            everyNormalOnlineFrameTrainsRealImage &= result.valid && result.imageObservationAccepted
                && result.diagnostic.has_value();
            if (result.diagnostic.has_value()) {
                const auto &tracker = result.diagnostic->algorithm.tracker;
                everyNormalOnlineFrameTrainsRealImage &= tracker.trainingCommitted
                    && !tracker.trainingRejected && !tracker.trainingFromSimulation;
                std::cerr << "Normal training frame=" << index + 1
                    << " committed=" << tracker.trainingCommitted
                    << " simulation=" << tracker.trainingFromSimulation
                    << " psr=" << tracker.responsePsr
                    << " normal=" << result.diagnostic->algorithm.responseNormal << std::endl;
                if (tracker.trainingFromSimulation) {
                    std::cerr << "Clear appearance rejected point=" << result.imageXPixel << ','
                        << result.imageYPixel << " shoulder="
                        << result.diagnostic->algorithm.shoulderObservationQuality << " center="
                        << result.diagnostic->algorithm.centerObservationQuality << '\n';
                }
            }
        }
        ok &= expectTrue(everyNormalOnlineFrameTrainsRealImage,
            "every clear normal online exposure updates real templates without false simulation replacement");
    }
    if (matureStatistics) {
        bool matureAndFrozen = observations.size() == expectedExposureCount;
        bool trainingSourcesMatchResponse = true;
        bool realLearningObserved = false;
        bool awaitingRealLearningRecovery = false;
        int preweldSamples = -1;
        for (std::size_t index = static_cast<std::size_t>(preweldExposureCount);
             index < observations.size(); ++index) {
            const auto &result = observations[index];
            matureAndFrozen &= result.diagnostic.has_value();
            if (result.diagnostic.has_value()) {
                const auto &algorithm = result.diagnostic->algorithm;
                const auto &tracker = algorithm.tracker;
                trainingSourcesMatchResponse &= result.valid && result.imageObservationAccepted
                    && !result.sample.estimated && tracker.trainingCommitted && !tracker.trainingRejected
                    && tracker.trainingFromSimulation != algorithm.responseNormal;
                if (tracker.trainingFromSimulation) {
                    awaitingRealLearningRecovery = true;
                } else {
                    realLearningObserved = true;
                    awaitingRealLearningRecovery = false;
                }
                if (tracker.responseTemplateCount == 3) {
                    if (preweldSamples < 0) {
                        preweldSamples = algorithm.normalResponseSamples;
                    }
                    matureAndFrozen &= algorithm.responseStatisticsReady
                        && algorithm.normalResponseSamples >= 30
                        && algorithm.normalResponseSamples == preweldSamples
                        && std::isfinite(algorithm.normalResponseMean)
                        && std::isfinite(algorithm.normalResponseStd)
                        && algorithm.normalResponseStd > 0.0;
                } else {
                    matureAndFrozen &= !algorithm.responseStatisticsReady;
                }
                std::cerr << "Mature statistics frame=" << index + 1
                    << " ready=" << algorithm.responseStatisticsReady
                    << " samples=" << algorithm.normalResponseSamples
                    << " mean=" << algorithm.normalResponseMean
                    << " std=" << algorithm.normalResponseStd
                    << " psr=" << algorithm.tracker.responsePsr
                    << " normal=" << algorithm.responseNormal << std::endl;
            }
        }
        ok &= expectTrue(matureAndFrozen && preweldSamples >= 30,
            "at least thirty accepted preweld normal responses calibrate a frozen distribution before online welding");
        ok &= expectTrue(trainingSourcesMatchResponse && realLearningObserved
                && !awaitingRealLearningRecovery,
            "mature statistics preserve current observations, use truthful training sources and recover real learning after isolated outliers");
        QObject::disconnect(&vision, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
            &control, &ui::COnlineCorrectionWorkerAdapter::appendTrackingResult);
        vision.stopTracking();
        vision.configure(options, makeProjectionContext());
        vision.startTracking();
        savedFrame.frameId = ++frameId;
        savedFrame.timestamp = savedFrame.timestamp.addMSecs(100);
        savedObservation.frameId = savedFrame.frameId;
        savedCorrelation.frameId = savedFrame.frameId;
        savedCorrelation.sessionId = 903;
        replayingExposure = true;
        vision.submitFrameWithRobotStateObservationAndCorrelation(savedFrame, savedState,
            -1, savedObservation, savedCorrelation);
        replayingExposure = false;
        ok &= expectTrue(replayResult.has_value() && replayResult->diagnostic.has_value()
                && replayResult->diagnostic->algorithm.normalResponseSamples == 0
                && !replayResult->diagnostic->algorithm.responseStatisticsReady,
            "a new correction session cannot inherit the preceding weld response calibration");
    }
    state.corflag = state.detflag = 3;
    record->replayPollState = state;
    robot.updateState(state);
    app.processEvents(QEventLoop::AllEvents, 10);
    state.corflag = state.detflag = 0;
    record->replayPollState = state;
    robot.updateState(state);
    state.sStart = 0;
    record->replayPollState = state;
    robot.updateState(state);
    vision.stopTracking();
    robot.disconnectRobot();
    ok &= expectTrue(!updates.empty() && updates.back().phase == control::OnlineCorrectionPhase::Completed,
        "final P receipt, detection reset and robot program stop complete the replay session");
    const auto directories = QDir(root).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    if (!expectTrue(directories.size() == 1, "replay creates one recorder session")) { return false; }
    const QString session = QDir(root).filePath(directories.first());
    QFile report(QDir(session).filePath(QStringLiteral("unified_session.jsonl")));
    if (!expectTrue(report.open(QIODevice::ReadOnly),
        "session publishes unified observation/release/command/receipt report")) { return false; }
    int observed = 0;
    int releases = 0;
    int commands = 0;
    int receipts = 0;
    QJsonObject releasedByExposure;
    QJsonObject targetsByGeneration;
    QJsonObject commandsByIdentity;
    std::vector<std::string> reportedRequests;
    qint64 sequence = 0;
    for (const auto &line : report.readAll().split('\n')) {
        if (line.isEmpty()) { continue; }
        const auto event = QJsonDocument::fromJson(line).object();
        ok &= expectTrue(event.value(QStringLiteral("sequence")).toVariant().toLongLong() == ++sequence,
            "unified report preserves recorder arrival order without inventing causal attribution");
        const auto type = event.value(QStringLiteral("event")).toString();
        if (type == QStringLiteral("observation")) {
            ++observed;
            ok &= expectTrue(event.value(QStringLiteral("session_id")).toString() == QStringLiteral("902")
                && event.value(QStringLiteral("frame_id")).toString() == QString::number(inputFrameIds.at(observed - 1))
                && event.value(QStringLiteral("reference_revision")).toString()
                    == QString::number(inputReferenceRevisions.at(observed - 1))
                && event.value(QStringLiteral("camera_timestamp_ms")).toString()
                    == QString::number(1700000000000LL + inputFrameIds.at(observed - 1) * 100)
                && !event.value(QStringLiteral("active_algorithm")).toString().isEmpty()
                && event.value(QStringLiteral("active_algorithm")).toString() != QStringLiteral("unavailable"),
                "observation records exposure/session/reference identity and actual algorithm");
            if (priorScenario != 0 && observed <= static_cast<int>(observations.size())) {
                const auto &reported = observations[static_cast<std::size_t>(observed - 1)];
                const auto prior = event.value(QStringLiteral("simulation_prior")).toObject();
                if (!reported.diagnostic.has_value()) {
                    ok &= expectTrue(false, "prior report has corresponding algorithm diagnostic");
                    continue;
                }
                const auto &algorithm = reported.diagnostic->algorithm;
                const auto source = prior.value(QStringLiteral("source_point_px"));
                const auto sampled = prior.value(QStringLiteral("template_point_px"));
                const auto samePoint = [](const QJsonValue &value, const cv::Point2d &point) {
                    const auto pair = value.toArray();
                    return pair.size() == 2 && std::abs(pair[0].toDouble() - point.x) < 1e-6
                        && std::abs(pair[1].toDouble() - point.y) < 1e-6;
                };
                ok &= expectTrue(!prior.isEmpty()
                    && prior.value(QStringLiteral("position_in_image")).toBool()
                        == algorithm.simulationPositionInImage
                    && prior.value(QStringLiteral("shape_available")).toBool()
                        == algorithm.simulationShapeAvailable
                    && prior.value(QStringLiteral("template_rebased")).toBool()
                        == algorithm.simulationTemplateRebased
                    && (algorithm.simulatedPointAvailable ? samePoint(source, algorithm.simulatedPoint)
                        : source.isNull())
                    && (algorithm.simulationShapeAvailable ? samePoint(sampled, algorithm.simulationTemplatePoint)
                        : sampled.isNull()),
                    "report links raw simulation position, qualified shape and actual sampling anchor to the same exposure");
                const auto fusion = event.value(QStringLiteral("position_fusion")).toObject();
                const auto &actual = reported.diagnostic->algorithm;
                ok &= expectTrue(!fusion.isEmpty()
                    && fusion.value(QStringLiteral("applied")).toBool() == actual.positionFusionApplied
                    && fusion.value(QStringLiteral("trusted_prior")).toBool() == actual.simulationPositionTrusted
                    && (actual.imageObservationAccepted
                        ? samePoint(fusion.value(QStringLiteral("observation_px")), actual.positionObservationPoint)
                        : fusion.value(QStringLiteral("observation_px")).isNull())
                    && (actual.positionFusionApplied
                        ? samePoint(fusion.value(QStringLiteral("prior_px")), actual.positionPriorPoint)
                            && samePoint(fusion.value(QStringLiteral("local_bias_px")), actual.simulationLocalBias)
                            && samePoint(fusion.value(QStringLiteral("gain")),
                                {actual.positionGainX, actual.positionGainY})
                        : fusion.value(QStringLiteral("prior_px")).isNull()
                            && fusion.value(QStringLiteral("local_bias_px")).isNull()
                            && fusion.value(QStringLiteral("gain")).isNull()),
                    "session report preserves actual image measurement, trusted simulation prior and applied fusion separately");

            }
        } else if (type == QStringLiteral("release")) {
            ++releases;
            const auto source = event.value(QStringLiteral("source_exposure")).toObject();
            const qint64 exposure = source.value(QStringLiteral("frame_id")).toString().toLongLong();
            const auto found = std::find(inputFrameIds.begin(), inputFrameIds.end(), exposure);
            const auto index = static_cast<std::size_t>(std::distance(inputFrameIds.begin(), found));
            ok &= expectTrue(found != inputFrameIds.end()
                && source.value(QStringLiteral("session_id")).toString() == QStringLiteral("902")
                && source.value(QStringLiteral("reference_revision")).toString().toULongLong()
                    == inputReferenceRevisions.at(index),
                "released samples retain their original exposure and reference identity through buffering");
            if (found != inputFrameIds.end()) {
                const auto space = event.value(QStringLiteral("space_last_link_mm")).toArray();
                const auto measured = std::find_if(observations.begin(), observations.end(),
                    [exposure](const ui::RuntimeTrackingResult &value) { return value.frameId == exposure; });
                ok &= expectTrue(space.size() == 3 && measured != observations.end() && measured->valid
                    && std::abs(space[0].toDouble() - measured->sample.point.xMm) < 1e-6
                    && std::abs(space[1].toDouble() - measured->sample.point.yMm) < 1e-6
                    // RBCALIB解析使用既有固定231mm工具偏置；观测为TCP系，释放为末节系。
                    && std::abs(space[2].toDouble() - measured->sample.point.zMm - 231.0) < 1e-6,
                    "release origin identifies the actual measurement rather than the latest arrival");
                releasedByExposure.insert(QString::number(exposure), space);
            }
        } else if (type == QStringLiteral("control_target")) {
            const auto first = event.value(QStringLiteral("source_exposure"));
            const auto direction = event.value(QStringLiteral("direction_source_exposure"));
            if (event.value(QStringLiteral("target_source")).toInt()
                == static_cast<int>(control::OnlineCorrectionTargetSource::Measured)) {
                const auto firstFrame = first.toObject().value(QStringLiteral("frame_id")).toString();
                const auto directionFrame = direction.toObject().value(QStringLiteral("frame_id")).toString();
                ok &= expectTrue(releasedByExposure.contains(firstFrame)
                    && releasedByExposure.contains(directionFrame)
                    && releasedByExposure.value(firstFrame) == event.value(QStringLiteral("line_first_mm"))
                    && releasedByExposure.value(directionFrame)
                        == event.value(QStringLiteral("line_direction_point_mm")),
                    "control target names the actual buffered line endpoints consumed by the follower");
            } else {
                ok &= expectTrue(first.isNull() && direction.isNull(),
                    "predicted control geometry cannot claim a current exposure origin");
            }
            targetsByGeneration.insert(event.value(QStringLiteral("target_generation")).toString(), event);
        } else if (type == QStringLiteral("command")) {
            ++commands;
            if (event.value(QStringLiteral("type")).toString() == QStringLiteral("realtime")) {
                const auto trace = event.value(QStringLiteral("local_trace")).toObject();
                const auto commandId = trace.value(QStringLiteral("command_id")).toString();
                ok &= expectTrue(trace.value(QStringLiteral("session_id")).toString() == QStringLiteral("902")
                    && commandId.toULongLong() > 0 && !commandsByIdentity.contains(commandId)
                    && trace.value(QStringLiteral("target_generation"))
                        == event.value(QStringLiteral("target_generation")),
                    "generated P carries a unique local command identity in the producing session");
                commandsByIdentity.insert(commandId, event);
            }
            if (event.value(QStringLiteral("type")).toString() == QStringLiteral("realtime")
                && !event.value(QStringLiteral("target_generation")).isNull()) {
                ok &= expectTrue(targetsByGeneration.contains(
                    event.value(QStringLiteral("target_generation")).toString()),
                    "generated realtime command references its recorded control target");
            }
        }
        else if (type == QStringLiteral("receipt")) {
            ++receipts;
            if (event.value(QStringLiteral("type")).toInt()
                == static_cast<int>(robot_comm::RobotCommandType::RealtimeCorrection)) {
                const auto trace = event.value(QStringLiteral("local_trace")).toObject();
                const auto id = trace.value(QStringLiteral("command_id")).toString();
                const auto generated = commandsByIdentity.value(id).toObject();
                ok &= expectTrue(trace.value(QStringLiteral("session_id")).toString() == QStringLiteral("902")
                    && !generated.isEmpty() && generated.value(QStringLiteral("local_trace"))
                        == event.value(QStringLiteral("local_trace"))
                    && event.value(QStringLiteral("local_transaction_id")).toString().toULongLong() > 0
                    && event.value(QStringLiteral("execution_semantics")).toString()
                        == QStringLiteral("unknown_execution")
                    && event.value(QStringLiteral("delivery")).toString() == QStringLiteral("response_received"),
                    "P receipt identifies the command actually selected for sending without claiming device execution");
            }
            reportedRequests.push_back(event.value(QStringLiteral("request")).toString().toStdString());
            ok &= expectTrue(event.value(QStringLiteral("sent")).toBool()
                && event.value(QStringLiteral("response_received")).toBool(),
                "reported actual commands include successful transport and device reply evidence");
        }
    }
    std::vector<std::string> transportedRequests;
    for (std::size_t index = 0; index < record->types.size(); ++index) {
        if (record->types[index] == robot_comm::RobotCommandType::TeachPointUpdate
            || record->types[index] == robot_comm::RobotCommandType::RealtimeCorrection) {
            transportedRequests.push_back(record->requests[index]);
        }
    }
    ok &= expectTrue(reportedRequests == transportedRequests,
        "reported receipt requests match all actual O/P transport requests in order");
    ok &= expectTrue(observed == static_cast<int>(inputFrameIds.size()) && releases > 5 && commands > 0 && receipts > 0,
        "report retains every exposure including rejects, release targets and command/receipt events");
    std::cout << "Normal synthetic session report: " << session.toStdString()
        << "; workpiece ICP absent/disabled; trajectory ICP enabled; no field TCP precision claim\n";
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
    std::size_t expectedResultCount = 0;
    QObject::connect(&runtime, &ui::CRuntimeCoordinator::trackingResultReady, &loop,
                     [&](const ui::RuntimeTrackingResult &result) {
                         results.push_back(result);
                         if (results.size() >= expectedResultCount && result.valid) { loop.quit(); }
                     });
    runtime.start();
    runtime.configureTracking(options, makeProjectionContext());
    runtime.publishRobotState(state);
    bool ok = connectCorrectionRobotSim(runtime, loop);
    for (int session = 0; session < 2; ++session) {
        expectedResultCount = static_cast<std::size_t>((session + 1) * 2);
        const double installX = session == 0 ? 100 : 200;
        std::ofstream(configuration.rbcalibFile)
            << "////SRANG " << installX << ",20,30,0,0,0\n";
        runtime.configureOnlineCorrection(configuration);
        // 会话已配置之后文件改为第三版本，两端必须继续使用刚选定的快照。
        std::ofstream(configuration.rbcalibFile) << "////SRANG 900,20,30,0,0,0\n";
        runtime.startOnlineTracking();
        for (int exposure = 0; exposure < 2; ++exposure) {
            QTimer::singleShot(100 + exposure * 100, &loop, [&, exposure]() {
                auto frame = makeOnlineSeamFrame();
                frame.frameId = 900 + session * 2 + exposure;
                runtime.publishCameraFrame(frame);
            });
        }
        QTimer timeout;
        timeout.setSingleShot(true);
        QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
        timeout.start(4000);
        loop.exec();
        if (!results.empty() && !results.back().valid) {
            std::cerr << "snapshot Vision: " << results.back().message.toStdString() << '\n';
        }
        ok &= expectTrue(results.size() == static_cast<std::size_t>((session + 1) * 2)
                             && results.back().valid, "snapshot test gets a valid synchronized Vision result");
        if (results.size() == static_cast<std::size_t>((session + 1) * 2) && results.back().valid) {
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
        // 采样时钟测试先由可验收原图建立 KCF，不能再借 CAD 模板制造目标。
        auto frame = makeOnlineSeamFrame();
        frame.frameId = ++frameId;
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
            raw.simulatedPointAvailable && raw.attempted
                && raw.tracker.completed && raw.tracker.hasResponse,
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
    const auto resumedConfirmed = submit(100, false, 160.0, 1000000000001LL);
    ok &= expectTrue(resumedConfirmed.valid && bucket(resumedConfirmed) == -1,
        "second fresh resumed exposure confirms identity without resetting sampling quota");

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
    auto resumedIdentity = submit(200, true, 160.0, 0);
    for (int attempt = 0; attempt < 160 && !resumedIdentity.imageObservationAccepted; ++attempt) {
        resumedIdentity = submit(200, true, 160.0, attempt + 1);
    }
    ok &= expectTrue(resumedIdentity.valid && resumedIdentity.imageObservationAccepted,
        "resumed sampling fixture establishes a fresh raw identity before testing anomaly cooldown");
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

/** @brief 实际 worker 两阶段均以当前图像定点，错误 CAD 与缺图不能新增测量。 */
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
        ui::COnlineCorrectionWorkerAdapter initialControl;
        ui::RuntimeOnlineCorrectionConfiguration initialConfiguration;
        makeOnlineConfigurationFiles(&initialConfiguration);
        std::optional<control::OnlineCorrectionUpdate> initialUpdate;
        QObject::connect(&initialControl, &ui::COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
            &app, [&](const control::OnlineCorrectionUpdate &update) { initialUpdate = update; });
        const auto initialConnection = QObject::connect(&worker,
            &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
            &initialControl, &ui::COnlineCorrectionWorkerAdapter::appendTrackingResult);
        initialControl.configure(initialConfiguration);
        initialControl.startInitialDetection();
        worker.configure(options, makeProjectionContext());
        worker.startTracking();
        qint64 frameId = 0;
        quint64 poseRevision = 1;
        bool onlineWindow = false;
        double simulatedX = 180.0;
        double imageShiftX = 0.0;
        bool hasCadImageHint = true;
        bool hideCenter = false;
        bool symmetricCrossSection = false;
        bool emptyImage = false;
        const auto submit = [&](bool blank) {
            auto frame = makeOnlineSeamFrame();
            frame.frameId = ++frameId;
            frame.timestamp = QDateTime::fromMSecsSinceEpoch(1700000000000LL + frameId * 100);
            if (imageShiftX != 0.0) {
                QImage shifted(frame.image.size(), QImage::Format_Grayscale8);
                shifted.fill(15);
                for (int y = 0; y < shifted.height(); ++y) {
                    for (int x = 0; x < shifted.width(); ++x) {
                        const int sourceX = x - static_cast<int>(imageShiftX);
                        if (sourceX >= 0 && sourceX < frame.image.width()) {
                            shifted.scanLine(y)[x] = frame.image.constScanLine(y)[sourceX];
                        }
                    }
                }
                frame.image = shifted;
            }
            if (symmetricCrossSection) {
                for (int y = 0; y < frame.image.height(); ++y) {
                    for (int x = 0; x < frame.image.width(); ++x) {
                        const double stripeY = 120.0 - 0.45 * std::abs(x - 160.0 - imageShiftX);
                        const double dy = y - stripeY;
                        frame.image.scanLine(y)[x] = static_cast<uchar>(
                            14.0 + 210.0 * std::exp(-dy * dy / 3.38));
                    }
                }
            }
            if (hideCenter) {
                const int centerX = 160 + static_cast<int>(imageShiftX);
                for (int y = 0; y < frame.image.height(); ++y) {
                    for (int x = std::max(0, centerX - 8);
                         x < std::min(frame.image.width(), centerX + 9); ++x) {
                        frame.image.scanLine(y)[x] = 14;
                    }
                }
            }
            if (blank) { frame.image.fill(0); }
            frame.previewImage = frame.image;
            if (emptyImage) { frame.image = {}; frame.previewImage = {}; }
            digital_twin::DigitalTwinVisionLaserObservation observation;
            observation.valid = true;
            observation.frameId = frame.frameId;
            observation.hasWeldPointImage_px = true;
            observation.weldPointImage_px = {simulatedX, 120.0};
            const double modelShift = simulatedX - 180.0;
            observation.laserLineImagePoints_px = {
                {30.0 + modelShift, 78.0}, {simulatedX, 120.0}, {300.0 + modelShift, 81.6}};
            if (!hasCadImageHint) {
                observation.hasWeldPointImage_px = false;
                observation.laserLineImagePoints_px.clear();
            }
            ui::RuntimeVisionCorrelation correlation;
            correlation.sessionId = diagnostics ? 710 : 711;
            correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
            correlation.onlineControlWindowActive = onlineWindow;
            correlation.frameId = frame.frameId;
            correlation.workpiecePoseRevision = poseRevision;
            worker.submitFrameWithRobotStateObservationAndCorrelation(
                frame, makeReadyRobotState(), -1, observation, correlation);
            const auto result = results.empty() ? ui::RuntimeTrackingResult{} : results.back();
            if (onlineWindow && result.diagnostic.has_value()) {
                const auto &algorithm = result.diagnostic->algorithm;
                if (algorithm.attempted) {
                    ok &= expectTrue(algorithm.imageRefinementCallCount <= 2,
                        "paper tracking cannot trigger an unbounded sequence of geometric refinements");
                } else {
                    ok &= expectTrue(algorithm.imageRefinementCallCount == 0,
                        "preflight rejection cannot perform any image refinement work");
                }
            }
            return result;
        };
        // 论文路径允许当前相关观测，不再等待旧几何主线固定次数确认。
        const auto awaitIndependentMeasurement = [&]() {
            ui::RuntimeTrackingResult result;
            for (int attempt = 0; attempt < 8; ++attempt) {
                result = submit(false);
                if (result.valid && result.imageObservationAccepted) {
                    break;
                }
                ok &= expectTrue(!result.independentWeldPointInParent_mm.valid,
                    "unfinished bounded search cannot publish an independent ICP sample");
            }
            return result;
        };
        double lastMeasuredX = 160.0;
        const auto expectMeasured = [&](const ui::RuntimeTrackingResult &result,
                                        double expectedX, const char *message) {
            lastMeasuredX = expectedX;
            ok &= expectTrue(result.valid && result.sample.point.valid
                    && result.imageMeasurementOutputEnabled && result.imageObservationAccepted
                    && !result.sample.estimated
                    && std::abs(result.imageXPixel - expectedX) <= 1.5
                    && std::abs(result.sample.point.xMm - (expectedX - 160.0)) <= 1.5,
                message);
            ok &= expectTrue(result.diagnostic.has_value() == diagnostics,
                "measurement provenance must survive disabled diagnostic payload");
            if (result.diagnostic.has_value()) {
                const auto &algorithm = result.diagnostic->algorithm;
                ok &= expectTrue(algorithm.imageMeasurementOutputEnabled
                        && algorithm.imageObservationAccepted
                        && std::abs(algorithm.finalPoint.x - result.imageXPixel) < 1e-9,
                    "diagnostic identity and displayed measurement must agree");
            }
        };
        const auto expectMissing = [&](const ui::RuntimeTrackingResult &result,
                                       const char *message) {
            if (onlineWindow && result.kcfPixelAvailable) {
                ok &= expectTrue(result.valid && result.sample.point.valid && result.controlEligible
                        && result.sample.estimated && !result.imageObservationAccepted
                        && !result.independentWeldPointInParent_mm.valid
                        && std::abs(result.imageXPixel - lastMeasuredX) <= 8.0,
                    message);
                return;
            }
            ok &= expectTrue(!result.valid && !result.sample.point.valid
                    && !result.detectedRobotBasePoint.valid
                    && !result.detectedStationBasePoint.valid
                    && !result.independentWeldPointInParent_mm.valid
                    && result.imageMeasurementOutputEnabled && !result.imageObservationAccepted,
                message);
        };
        expectMissing(submit(true), "initial reverse scan cannot create an initial point from CAD alone");
        expectMeasured(submit(false), 160.0, "a supported first image can establish paper tracking without a fixed frame delay");
        const auto first = submit(false);
        expectMeasured(first, 160.0, "reverse scan must use original-image geometry despite full CAD weight");
        ok &= expectTrue(first.independentWeldPointInParent_mm.valid,
            "accepted reverse-scan geometry remains independently projected");
        imageShiftX = 3.0;
        const auto lastInitial = submit(false);
        expectMeasured(lastInitial, 163.0, "later reverse-scan image supplies a fresh measured starting sample");
        expectMissing(submit(true), "reverse-scan blank tail cannot append a newer simulated starting point");
        ok &= expectTrue(initialUpdate.has_value()
                && initialUpdate->state.initialDetectionSamples.size() == 3
                && std::abs(initialUpdate->state.initialDetectionSamples.back().point.xMm - 3.0) <= 1.5,
            "real control input retains last accepted reverse-scan image point and appends nothing for blank tail");
        QObject::disconnect(initialConnection);
        initialControl.reset();
        // 实际反扫完成会停 Vision，STARTCOR 再 configure/start；不假设模板跨阶段保留。
        worker.stopTracking();
        worker.configure(options, makeProjectionContext());
        worker.startTracking();
        onlineWindow = true;
        simulatedX = 250.0;
        const auto firstOnline = awaitIndependentMeasurement();
        expectMeasured(firstOnline, 163.0, "online entry retains image measurement while CAD remains misaligned");
        auto independentlyConfirmed = firstOnline;
        for (int attempt = 0; attempt < 8 && !independentlyConfirmed.independentWeldPointInParent_mm.valid; ++attempt) {
            independentlyConfirmed = submit(false);
        }
        ok &= expectTrue(independentlyConfirmed.independentWeldPointInParent_mm.valid,
            "repeated clear online geometry retains separately confirmed independent ICP identity");
        for (const auto mode : {vision::KcfFusionMode::KcfOnly,
                               vision::KcfFusionMode::ManualWeight,
                               vision::KcfFusionMode::AutomaticKalman}) {
            vision::KcfFusionOptions fusion;
            fusion.mode = mode;
            fusion.simulationWeight = 0.1;
            worker.updateOnlineFusionOptions(fusion);
            expectMeasured(submit(false), 163.0,
                "legacy fusion mode controls cannot pull accepted online measurement toward wrong CAD");
        }
        ++poseRevision;
        simulatedX = 290.0;
        expectMissing(submit(true), "CAD update on a missing frame preserves a stable estimated KCF without making a measurement");
        expectMeasured(submit(false), 163.0, "recovered current-image identity is not held behind a fixed confirmation delay");
        expectMeasured(submit(false), 163.0,
            "new CAD revision cannot displace a fresh confirmed image measurement");
        worker.stopTracking();
        worker.startTracking();
        expectMissing(submit(true), "restart without a current image cannot seed business output from simulation");
        expectMeasured(awaitIndependentMeasurement(), 163.0,
            "restart with distant CAD must recover from actual image evidence");
        // 同帧运输仍成功，仅仿真图像提示缺失；不能由此剥夺原图检测资格。
        worker.stopTracking();
        worker.startTracking();
        hasCadImageHint = false;
        const auto noHint = awaitIndependentMeasurement();
        expectMeasured(noHint, 163.0,
            "same-frame transport without CAD image hints still allows original-image recovery");
        const auto secondNoHint = submit(false);
        expectMeasured(secondNoHint, 163.0, "missing CAD image hint cannot contaminate the next image measurement");
        if (secondNoHint.diagnostic.has_value()) {
            ok &= expectTrue(!secondNoHint.diagnostic->algorithm.simulatedPointAvailable
                    && secondNoHint.diagnostic->imageSampleBucket != 4,
                "missing CAD hint cannot manufacture a CAD deviation anomaly against default coordinates");
        }
        worker.stopTracking();
        worker.startTracking();
        hasCadImageHint = true;
        imageShiftX = 0.0;
        simulatedX = 180.0;
        onlineWindow = false;
        symmetricCrossSection = true;
        expectMeasured(awaitIndependentMeasurement(), 160.0, "clear new exposures confirm KCF identity for stage admission test");
        hideCenter = true;
        const auto initialKcf = submit(false);
        ok &= expectTrue(!initialKcf.independentWeldPointInParent_mm.valid
                && (!initialKcf.valid || initialKcf.sample.estimated),
            "a hidden-center estimate cannot masquerade as an initial independent measurement");
        hideCenter = false;
        worker.stopTracking();
        worker.configure(options, makeProjectionContext());
        worker.startTracking();
        onlineWindow = true;
        expectMeasured(awaitIndependentMeasurement(), 160.0,
            "formal online KCF fallback starts from its own freshly accepted raw-image identity");
        hideCenter = true;
        const auto onlineKcf = submit(false);
        if (!onlineKcf.sample.estimated) {
            std::cerr << "Hidden-center source: valid=" << onlineKcf.valid
                << " observation=" << onlineKcf.imageObservationAccepted
                << " geometry=" << onlineKcf.independentWeldPointInParent_mm.valid
                << " message=" << onlineKcf.message.toStdString() << '\n';
        }
        ok &= expectTrue(onlineKcf.valid && onlineKcf.controlEligible && onlineKcf.sample.estimated
                && !onlineKcf.independentWeldPointInParent_mm.valid,
            "continuous online KCF enters following while staying outside independent ICP");
        hideCenter = false;
        expectMeasured(submit(false), 160.0, "clear recovered same-target evidence immediately replaces the estimate");
        expectMeasured(submit(false), 160.0, "new exposures confirm original geometry after missing center");
        imageShiftX = 100.0;
        auto pendingFar = submit(false);
        expectMissing(pendingFar, "far raw image needs consecutive independent identity confirmations");
        // 论文局部身份恢复不声称完成旧几何搜索的远处提案确认；仍保留远点隔离断言。
        emptyImage = true;
        expectMissing(submit(false), "invalid image input interrupts pending image identity confirmations");
        emptyImage = false;
        int confirmedFreshCandidates = 0;
        const auto observeFarConfirmation = [&](const ui::RuntimeTrackingResult &result) {
            if (!diagnostics) { return; }
            const auto &raw = result.diagnostic->algorithm.rawRefinement;
            confirmedFreshCandidates = raw.valid ? confirmedFreshCandidates + 1 : 0;
            if (confirmedFreshCandidates < 3) {
                expectMissing(result,
                    "post-gap far candidate needs three consecutive new image confirmations after search completes");
            }
        };
        const auto firstFarAfterGap = submit(false);
        observeFarConfirmation(firstFarAfterGap);
        expectMissing(firstFarAfterGap, "first far image after invalid input cannot inherit prior confirmation");
        const auto secondFarAfterGap = submit(false);
        observeFarConfirmation(secondFarAfterGap);
        expectMissing(secondFarAfterGap, "second far image after invalid input is still unconfirmed");
        for (int attempt = 0; attempt < 20; ++attempt) {
            expectMissing(submit(false), "far candidate cannot automatically replace historical target identity");
        }
        imageShiftX = 0.0;
        expectMeasured(awaitIndependentMeasurement(), 160.0,
            "original target can be recovered after rejecting distant candidates");
        worker.stopTracking();
        options.sensorCalibration.focalLengthM = 0.0;
        worker.configure(options, makeProjectionContext());
        worker.startTracking();
        expectMissing(submit(false),
            "invalid same-exposure projection context cannot publish any 3D measurement");
        worker.stopTracking();
    }
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);

    if (argc > 1 && std::string_view(argv[1]) == "--session-new-exposure-contract") {
        return testReentrantOnlineRestartDropsOldTrackingResult(app, true) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--session-lifecycle-contract") {
        return testReentrantOnlineRestartDropsOldTrackingResult(app, false) ? 0 : 1;
    }

    if (argc > 1 && std::string_view(argv[1]) == "--controlled-stop-contract") {
        return testOnlineOperationStopRequestsHold(app)
            && testOnlineOperationStopWaitsForFreshHold(app)
            && testOnlineOperationStopIsolatesMotion(app)
            && testOnlineOperationStopReportsUnconfirmed(app)
            && testOnlineOperationStopKeepsSessionReport(app)
            && testOnlineOperationStopRejectsRestart(app)
            && testOnlineOperationStopInterruptsInFlightBatch(app) ? 0 : 1;
    }

    if (argc > 1 && std::string_view(argv[1]) == "--liutao-normal-visible-contract") {
        return testLiutaoNormalVisibleSeam(app, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-distinctive-stripe-contract") {
        return testLiutaoNormalVisibleSeam(app, true) ? 0 : 1;
    }

    if (argc > 1 && std::string_view(argv[1]) == "--liutao-strong-reflection-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, true, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-equal-reflection-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, true, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-mature-normal-statistics-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-interference-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, true, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-severe-loss-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-initial-severe-loss-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-runtime-severe-loss-contract") {
        return testSevereLossThroughRuntime(app) ? 0 : 1;
    }
    if (argc > 1 && (std::string_view(argv[1]) == "--liutao-unavailable-image-loss-session-contract"
            || std::string_view(argv[1]) == "--liutao-unavailable-projection-loss-session-contract"
            || std::string_view(argv[1]) == "--liutao-late-projection-loss-session-contract")) {
        return testNormalSessionReplay(app, true, false, 0, false, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-biased-estimate-recovery-session-contract") {
        return testNormalSessionReplay(app, true, false, 7, true, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-misalignment-recovery-session-contract") {
        return testNormalSessionReplay(app, true, false, 7, true, 0) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-stationary-queue-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, 4) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-reference-revision-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, 3) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-invalid-reference-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, 2) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-repeated-exposure-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, true) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--normal-session-replay-contract") {
        return testNormalSessionReplay(app, false, false, 0, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-normal-session-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-normal-training-source-contract") {
        return testNormalSessionReplay(app, true, false, 0, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-model-lifecycle-contract") {
        return testNormalSessionReplay(app, true, true, 0, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-prior-misalignment-contract") {
        return testNormalSessionReplay(app, true, false, 1, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-prior-in-image-bias-contract") {
        return testNormalSessionReplay(app, true, false, 2, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-prior-direction-bias-contract") {
        return testNormalSessionReplay(app, true, false, 3, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-initial-reference-gap-contract") {
        return testNormalSessionReplay(app, true, false, 4, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-local-prior-change-contract") {
        return testNormalSessionReplay(app, true, false, 5, false, false) ? 0 : 1;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--liutao-skipped-exposure-prior-contract") {
        return testNormalSessionReplay(app, true, false, 6, false, false) ? 0 : 1;
    }

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
