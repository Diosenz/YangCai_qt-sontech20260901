/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_offline_runtime_worker_flow.cpp
 * @brief       offline runtime worker flow 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <QCoreApplication>
#include <QEventLoop>

#include "RuntimeWorkerAdapters.h"

namespace {

namespace camera = i2r_rlc::camera;
namespace control = i2r_rlc::control;
namespace robot = i2r_rlc::robot_comm;
namespace ui = i2r_rlc::ui;
namespace vision = i2r_rlc::vision;

void require(bool condition, const char *message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class FakeTeachPointTransport final : public robot::IRobotCommandTransport {
public:
    bool connect(std::string_view, std::uint16_t) override
    {
        connected = true;
        return true;
    }

    void disconnect() override
    {
        connected = false;
    }

    bool isConnected() const override
    {
        return connected;
    }

    robot::RobotCommandReceipt transact(
        robot::RobotCommandType type,
        const std::string &request,
        int pointIndex,
        robot::RobotState *state) override
    {
        requests.push_back(request);
        robot::RobotCommandReceipt receipt;
        receipt.type = type;
        receipt.sent = true;
        receipt.responseReceived = true;
        receipt.completed = true;
        receipt.pointIndex = pointIndex;
        receipt.request = request;
        receipt.response = "D 0,0,0,0,5,0,0,0,0,0";
        receipt.message = "fake D.offline==5";
        if (state != nullptr) {
            state->offline = 5;
        }
        return receipt;
    }

    bool connected = false;
    std::vector<std::string> requests;
};

std::filesystem::path makeTempCaseDirectory()
{
    const auto root = std::filesystem::temp_directory_path()
        / "i2r_rlc_offline_runtime_worker_flow_tests";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    return root;
}

vision::WeldSpacePoint makePoint(double x, double y, double z)
{
    return vision::WeldSpacePoint{ x, y, z, true };
}

std::vector<vision::WeldSpacePoint> makeModelPoints()
{
    std::vector<vision::WeldSpacePoint> points;
    for (int i = 0; i < 24; ++i) {
        const double t = static_cast<double>(i);
        points.push_back(makePoint(
            t * 4.0,
            std::sin(t * 0.3) * 6.0,
            std::cos(t * 0.2) * 2.0));
    }
    return points;
}

void writeArchive(
    const std::filesystem::path &directory,
    const std::vector<vision::WeldSpacePoint> &modelPoints)
{
    std::ofstream model(directory / "case_W.txt");
    for (const auto &point : modelPoints) {
        model << point.xMm << ' ' << point.yMm << ' ' << point.zMm << '\n';
    }

    std::ofstream teach(directory / "case_T.txt");
    for (int i = 0; i < 5; ++i) {
        const auto &point = modelPoints[static_cast<std::size_t>(i * 4)];
        teach << point.xMm << ' ' << point.yMm << ' ' << point.zMm << ' '
              << 1.0 + i << ' ' << 2.0 + i << ' ' << 3.0 + i << ' '
              << 10.0 + i << ' ' << 20.0 + i << '\n';
    }
}

ui::RuntimeTrackingResult makeTrackingResult(const vision::WeldSpacePoint &point)
{
    ui::RuntimeTrackingResult result;
    result.valid = true;
    result.triggerAccepted = true;
    result.traflag = 1;
    result.posflag = 0;
    result.sample.point.xMm = point.xMm + 0.4;
    result.sample.point.yMm = point.yMm - 0.25;
    result.sample.point.zMm = point.zMm + 0.15;
    result.sample.point.valid = true;
    result.message = QStringLiteral("fake offline KCF sample");
    return result;
}

void processQueuedEvents()
{
    for (int i = 0; i < 8; ++i) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }
}

void testOfflineWorkerSendsTeachPointsAndCompletes()
{
    const auto directory = makeTempCaseDirectory();
    const auto modelPoints = makeModelPoints();
    writeArchive(directory, modelPoints);

    auto transport = std::make_unique<FakeTeachPointTransport>();
    auto *transportPtr = transport.get();
    ui::CRobotCommWorkerAdapter robotWorker(std::move(transport));
    ui::COfflineCorrectionWorkerAdapter offlineWorker;

    std::vector<control::OfflineCorrectionPhase> phases;
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::offlineCorrectionUpdated,
        [&](const control::OfflineCorrectionUpdate &update) {
            phases.push_back(update.phase);
        });
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::robotCommandReady,
        &robotWorker,
        &ui::CRobotCommWorkerAdapter::sendCorrectionCommand,
        Qt::QueuedConnection);
    QObject::connect(
        &robotWorker,
        &ui::CRobotCommWorkerAdapter::commandReceiptReady,
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::handleRobotCommandReceipt,
        Qt::QueuedConnection);

    robotWorker.connectRobot();
    offlineWorker.configure(
        (directory / "case_W.txt").string(),
        control::OfflineCorrectionOptions{});
    offlineWorker.startDetection(2);
    robot::RobotState state;
    offlineWorker.updateRobotState(state);
    state.sStart = 1;
    state.traflag = 1;
    offlineWorker.updateRobotState(state);
    for (const auto &point : modelPoints) {
        offlineWorker.appendTrackingResult(makeTrackingResult(point));
    }
    state.sStart = 0;
    state.traflag = 0;
    offlineWorker.updateRobotState(state);
    offlineWorker.finishDetectionAndCompute();
    processQueuedEvents();

    require(transportPtr->connected, "fake transport should connect");
    require(!transportPtr->requests.empty(), "offline worker should send O requests");
    const bool hasTeachPointRequest = std::any_of(
        transportPtr->requests.begin(),
        transportPtr->requests.end(),
        [](const std::string &request) {
            return request.rfind("O ", 0) == 0;
        });
    require(hasTeachPointRequest,
            "offline worker should send old O request");
    require(std::find(phases.begin(), phases.end(),
                      control::OfflineCorrectionPhase::WaitingForRobotAck)
                != phases.end(),
            "offline worker should wait for robot ack");
    require(std::find(phases.begin(), phases.end(),
                      control::OfflineCorrectionPhase::Completed)
                != phases.end(),
            "offline worker should complete after D.offline==5");
}

void testOfflineWorkerWaitsForB040AndProgramStop()
{
    const auto directory = makeTempCaseDirectory();
    const auto modelPoints = makeModelPoints();
    writeArchive(directory, modelPoints);

    auto transport = std::make_unique<FakeTeachPointTransport>();
    auto *transportPtr = transport.get();
    ui::CRobotCommWorkerAdapter robotWorker(std::move(transport));
    ui::COfflineCorrectionWorkerAdapter offlineWorker;

    std::vector<control::OfflineCorrectionPhase> phases;
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::offlineCorrectionUpdated,
        [&](const control::OfflineCorrectionUpdate &update) {
            phases.push_back(update.phase);
        });
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::robotCommandReady,
        &robotWorker,
        &ui::CRobotCommWorkerAdapter::sendCorrectionCommand,
        Qt::QueuedConnection);
    QObject::connect(
        &robotWorker,
        &ui::CRobotCommWorkerAdapter::commandReceiptReady,
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::handleRobotCommandReceipt,
        Qt::QueuedConnection);

    robotWorker.connectRobot();
    offlineWorker.configure(
        (directory / "case_W.txt").string(),
        control::OfflineCorrectionOptions{});
    offlineWorker.startDetection(2);
    robot::RobotState state;
    offlineWorker.updateRobotState(state);
    state.sStart = 1;
    state.traflag = 1;
    offlineWorker.updateRobotState(state);
    for (const auto &point : modelPoints) {
        offlineWorker.appendTrackingResult(makeTrackingResult(point));
    }

    state.detflag = 3;
    state.traflag = 0;
    offlineWorker.updateRobotState(state);
    processQueuedEvents();
    require(transportPtr->requests.empty(),
            "ENDDET alone must not finish offline correction");
    state.OfflineRun = 1;
    offlineWorker.updateRobotState(state);
    processQueuedEvents();
    require(transportPtr->requests.empty(),
            "B040 must not compute while the scan JBI is running");
    state.sStart = 0;
    offlineWorker.updateRobotState(state);
    processQueuedEvents();

    require(!transportPtr->requests.empty(),
            "B040 and stopped JBI should finish detection and send O");
    const auto requestCountAfterFirstEnd = transportPtr->requests.size();
    offlineWorker.updateRobotState(state);
    processQueuedEvents();
    require(transportPtr->requests.size() == requestCountAfterFirstEnd,
            "latched B040 should not repeat offline finish/send");
    require(std::find(phases.begin(), phases.end(),
                      control::OfflineCorrectionPhase::WaitingForRobotAck)
                != phases.end(),
            "auto finish should wait for robot ack");
    require(std::find(phases.begin(), phases.end(),
                      control::OfflineCorrectionPhase::Completed)
                != phases.end(),
            "auto finish should complete after D.offline==5");
}

/**
 * @brief 专用扫描 JBI 不依赖定位段；关闭检测窗口不采点，再开启后继续采点。
 */
void testOfflineWorkerCollectsSeamsWithoutPositionSegments()
{
    const auto directory = makeTempCaseDirectory();
    const auto modelPoints = makeModelPoints();
    writeArchive(directory, modelPoints);
    ui::COfflineCorrectionWorkerAdapter offlineWorker;
    control::OfflineCorrectionUpdate lastUpdate;
    std::vector<int> acknowledgements;
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::offlineCorrectionUpdated,
        [&](const control::OfflineCorrectionUpdate &update) {
            lastUpdate = update;
        });
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::offlineStartCountChanged,
        [&](int count) { acknowledgements.push_back(count); });
    offlineWorker.configure(
        (directory / "case_W.txt").string(),
        control::OfflineCorrectionOptions{});
    offlineWorker.startDetection(2);
    auto result = makeTrackingResult(modelPoints.front());
    offlineWorker.appendTrackingResult(result);
    require(lastUpdate.state.detectionSamples.empty(),
            "samples before current scan start must be ignored");

    robot::RobotState state;
    state.OfflineRun = 1;
    offlineWorker.updateRobotState(state);
    require(!acknowledgements.empty() && acknowledgements.back() == 1,
            "stale B040 must be acknowledged before starting");
    state.OfflineRun = 0;
    offlineWorker.updateRobotState(state);
    state.sStart = 1;
    state.traflag = 1;
    offlineWorker.updateRobotState(state);
    offlineWorker.appendTrackingResult(result);
    require(lastUpdate.state.detectionSamples.size() == 1,
            "posflag=0 must allow seam samples");

    result.traflag = 0;
    offlineWorker.appendTrackingResult(result);
    require(lastUpdate.state.detectionSamples.size() == 1,
            "samples outside STARTDET must be ignored");

    // 即使仍保留定位段指令，第二次定位段也不能再被当作侧面而永久关闭。
    for (int posflag : { 1, 0, 1 }) {
        state.posflag = posflag;
        offlineWorker.updateRobotState(state);
        result.traflag = 1;
        result.posflag = posflag;
        offlineWorker.appendTrackingResult(result);
    }
    require(lastUpdate.state.detectionSamples.size() == 4,
            "all seam detection windows must remain eligible");
    result.valid = false;
    offlineWorker.appendTrackingResult(result);
    require(lastUpdate.state.detectionSamples.size() == 4,
            "invalid KCF results must still be rejected");
    offlineWorker.reset();
    offlineWorker.appendTrackingResult(makeTrackingResult(modelPoints.front()));
    require(lastUpdate.state.detectionSamples.empty(),
            "reset must isolate the previous scan");
}

void testUnsupportedDetTypeFailsBeforeTracking()
{
    ui::COfflineCorrectionWorkerAdapter offlineWorker;
    control::OfflineCorrectionUpdate lastUpdate;
    QObject::connect(
        &offlineWorker,
        &ui::COfflineCorrectionWorkerAdapter::offlineCorrectionUpdated,
        [&](const control::OfflineCorrectionUpdate &update) {
            lastUpdate = update;
        });

    offlineWorker.startDetection(1);
    require(!lastUpdate.succeeded, "det_type!=2 should fail");
    require(lastUpdate.phase == control::OfflineCorrectionPhase::Error,
            "det_type!=2 should enter Error");
    require(lastUpdate.message.find("det_type=2") != std::string::npos,
            "det_type error should be explicit");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    try {
        testOfflineWorkerSendsTeachPointsAndCompletes();
        testOfflineWorkerWaitsForB040AndProgramStop();
        testOfflineWorkerCollectsSeamsWithoutPositionSegments();
        testUnsupportedDetTypeFailsBeforeTracking();
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }

    std::cout << "PASS: offline runtime worker flow tests\n";
    return 0;
}
