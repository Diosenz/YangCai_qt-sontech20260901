/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_robot_legacy_commands.cpp
 * @brief       robot legacy commands 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>
#include <functional>
#include <memory>
#include <string_view>
#include <string>
#include <vector>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QObject>

#include "RuntimeWorkerAdapters.h"

namespace {

struct RecordedTransaction {
    i2r_rlc::robot_comm::RobotCommandType type;
    std::string request;
};

class CFakeTransport final
    : public i2r_rlc::robot_comm::IRobotCommandTransport {
public:
    bool connect(std::string_view, std::uint16_t) override
    {
        ++connectCallCount;
        m_connected = true;
        return true;
    }

    void disconnect() override
    {
        ++disconnectCallCount;
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
        transactions.push_back({type, request});
        i2r_rlc::robot_comm::RobotCommandReceipt receipt;
        receipt.type = type;
        receipt.request = request;
        receipt.pointIndex = pointIndex;
        if (!m_connected) {
            receipt.error =
                i2r_rlc::robot_comm::RobotCommandError::NotConnected;
            receipt.message = "socket is not connected";
            return receipt;
        }
        if (disconnectOnNextTransaction) {
            disconnectOnNextTransaction = false;
            m_connected = false;
            receipt.error =
                i2r_rlc::robot_comm::RobotCommandError::ReceiveFailed;
            receipt.message = "socket is not connected";
            return receipt;
        }
        receipt.sent = true;
        receipt.responseReceived = true;
        receipt.completed = true;
        receipt.response =
            "D 1,2,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,"
            "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
        if (state != nullptr) {
            state->sStart = 0;
        }
        return receipt;
    }

    std::vector<RecordedTransaction> transactions;
    int connectCallCount = 0;
    int disconnectCallCount = 0;
    bool disconnectOnNextTransaction = false;

private:
    bool m_connected = false;
};

bool expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

void processEventsFor(int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        QThread::msleep(5);
    }
}

bool spinUntil(
    const std::function<bool()> &predicate,
    int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        if (predicate()) {
            return true;
        }
        QThread::msleep(5);
    }
    QCoreApplication::processEvents();
    return predicate();
}

bool runAutoReconnectAfterTransportDropTest()
{
    auto transport = std::make_unique<CFakeTransport>();
    CFakeTransport *record = transport.get();
    i2r_rlc::ui::CRobotCommWorkerAdapter worker(std::move(transport));
    i2r_rlc::robot_comm::RobotCommunicationDiagnostics diagnostics;
    QObject::connect(
        &worker,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::diagnosticsReady,
        &worker,
        [&diagnostics](
            const i2r_rlc::robot_comm::RobotCommunicationDiagnostics& value) {
            diagnostics = value;
        });

    worker.connectRobot();
    bool ok = true;
    ok &= expect(record->connectCallCount == 1,
                 "initial connect should open transport once");
    ok &= expect(diagnostics.validDReceived,
                 "initial connect should poll a valid D response");

    record->disconnectOnNextTransaction = true;
    worker.pollRobotState();
    ok &= expect(!record->isConnected(),
                 "scripted receive failure should drop fake transport");
    ok &= expect(!diagnostics.validDReceived,
                 "transport drop should invalidate trusted D state");

    ok &= expect(spinUntil(
                     [&diagnostics, record]() {
                         return record->connectCallCount >= 2
                             && diagnostics.validDReceived;
                     },
                     1500),
                 "worker should reconnect automatically and resume N/D polling");
    ok &= expect(record->connectCallCount >= 2,
                 "automatic reconnect should call transport connect again");

    worker.disconnectRobot();
    return ok;
}

bool runExplicitDisconnectDoesNotReconnectTest()
{
    auto transport = std::make_unique<CFakeTransport>();
    CFakeTransport *record = transport.get();
    i2r_rlc::ui::CRobotCommWorkerAdapter worker(std::move(transport));
    worker.connectRobot();
    const int connectCallsBeforeDisconnect = record->connectCallCount;
    worker.disconnectRobot();
    processEventsFor(300);
    return expect(record->connectCallCount == connectCallsBeforeDisconnect,
                  "explicit disconnect must not schedule automatic reconnect");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    auto transport = std::make_unique<CFakeTransport>();
    CFakeTransport *record = transport.get();
    i2r_rlc::ui::CRobotCommWorkerAdapter worker(std::move(transport));
    i2r_rlc::robot_comm::RobotCommunicationDiagnostics diagnostics;
    QObject::connect(
        &worker,
        &i2r_rlc::ui::CRobotCommWorkerAdapter::diagnosticsReady,
        &app,
        [&diagnostics](const i2r_rlc::robot_comm::RobotCommunicationDiagnostics& value) {
            diagnostics = value;
        });
    worker.connectRobot();
    worker.setLaserEnabled(false);
    worker.pollRobotState();

    i2r_rlc::robot_comm::RobotCalibrationCommand calibration;
    calibration.requestNextCalibrationPosition = true;
    calibration.calibResetSignal = 1;
    calibration.capturedSampleCount = 3;
    calibration.valid = true;
    worker.sendCalibrationCommand(calibration);

    i2r_rlc::robot_comm::RobotState stopped;
    stopped.sStart = 0;
    worker.updateState(stopped);
    i2r_rlc::robot_comm::RobotJobStartCommand job;
    job.jobName = "ONLINE01";
    job.requestStart = true;
    job.valid = true;
    worker.sendJobStartCommand(job);
    const auto diagnosticsBeforeDisconnect = diagnostics;
    worker.disconnectRobot();

    bool cFound = false;
    bool jFound = false;
    bool laserOffNFound = false;
    int statusPollCount = 0;
    int laserOffStatusPollCount = 0;
    for (const auto &transaction : record->transactions) {
        if (transaction.type
            == i2r_rlc::robot_comm::RobotCommandType::StatusPoll) {
            ++statusPollCount;
            if (transaction.request == "N 1,0,0,0,0,0,0,0,0") {
                ++laserOffStatusPollCount;
            }
        }
        laserOffNFound = laserOffNFound
            || (transaction.type
                    == i2r_rlc::robot_comm::RobotCommandType::StatusPoll
                && transaction.request == "N 1,0,0,0,0,0,0,0,0");
        cFound = cFound
            || (transaction.type
                    == i2r_rlc::robot_comm::RobotCommandType::CalibrationAdvance
                && transaction.request == "C 1,0,0,0,0,0,0,0");
        jFound = jFound
            || (transaction.type
                    == i2r_rlc::robot_comm::RobotCommandType::JobStart
                && transaction.request == "J 1,0,0,0,0,0,0,0,ONLINE01");
    }

    if (!expect(laserOffNFound, "worker must use latest laser flag in N request")
        || !expect(statusPollCount >= 3,
                   "connect, laser toggle, and explicit poll should each send N")
        || !expect(laserOffStatusPollCount >= 2,
                   "laser toggle should immediately send an N with s_laser=0")
        || !expect(diagnosticsBeforeDisconnect.validDReceived,
                   "diagnostics should mark valid D after successful poll")
        || !expect(diagnosticsBeforeDisconnect.lastTransactionSucceeded,
                   "diagnostics should record successful last transaction")
        || !expect(!diagnosticsBeforeDisconnect.lastRequest.empty()
                       && !diagnosticsBeforeDisconnect.lastResponse.empty(),
                   "diagnostics should expose latest request and D response")
        || !expect(cFound, "worker must transact exact legacy C request")
        || !expect(jFound, "worker must transact exact legacy J request")) {
        return 1;
    }
    if (!runAutoReconnectAfterTransportDropTest()
        || !runExplicitDisconnectDoesNotReconnectTest()) {
        return 1;
    }
    std::cout << "PASS: legacy J/C robot command tests\n";
    return 0;
}
