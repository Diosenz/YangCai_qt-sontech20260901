/**
 * @file test_online_correction_initialization.cpp
 * @brief 验证在线启动前的 N 初始化事务及中断后的重新初始化。
 * @details 使用注入传输检查真实 worker 的出站请求和失败处理，不连接控制柜。
 *          本测试不证明控制柜实际清除了 B005，也不证明 MOVL 的目标取值。
 */
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <QCoreApplication>

#include "RuntimeWorkerAdapters.h"

namespace {

namespace robot = i2r_rlc::robot_comm;
namespace ui = i2r_rlc::ui;

const std::string kInitializationRequest = "N 1,1,1,0,0,0,0,0,1";

bool expectTrue(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

/**
 * @brief 提供可控状态/失败回执，只记录出站请求，不模拟机器人运动。
 */
class CRecordingTransport final : public robot::IRobotCommandTransport {
public:
    bool connect(std::string_view robotIp, std::uint16_t robotPort) override
    {
        connected = true;
        return true;
    }

    void disconnect() override { connected = false; }
    bool isConnected() const override { return connected; }

    robot::RobotCommandReceipt transact(
        robot::RobotCommandType type,
        const std::string &request,
        int pointIndex,
        robot::RobotState *state) override
    {
        requests.push_back(request);
        robot::RobotCommandReceipt receipt;
        receipt.type = type;
        receipt.pointIndex = pointIndex;
        receipt.request = request;
        receipt.sent = true;
        if (failAll || (failInitialization && request == kInitializationRequest)) {
            receipt.error = robot::RobotCommandError::InvalidResponse;
            receipt.message = "injected invalid D response";
            return receipt;
        }
        receipt.responseReceived = true;
        receipt.completed = true;
        if (state != nullptr) {
            *state = responseState;
        }
        return receipt;
    }

    bool connected = true;
    bool failAll = false;
    bool failInitialization = false;
    robot::RobotState responseState;
    std::vector<std::string> requests;
};

bool testInitializationAndRestart()
{
    auto transport = std::make_unique<CRecordingTransport>();
    auto *record = transport.get();
    ui::CRobotCommWorkerAdapter worker(std::move(transport));
    bool ok = expectTrue(worker.ensureOnlineCorrectionReady(),
                         "successful online preflight should be ready");
    ok &= expectTrue(std::count(record->requests.begin(), record->requests.end(),
                                kInitializationRequest) == 1,
                     "new cycle must send N.init=1 before job start");

    record->requests.clear();
    worker.pollRobotState();
    worker.ensureOfflineLaserEnabled();
    ok &= expectTrue(record->requests.size() == 2
                         && std::all_of(record->requests.begin(), record->requests.end(),
                             [](const std::string &request) {
                                 return request == "N 1,1,1,0,0,0,0,0,0";
                             }),
                     "ordinary polling and offline laser check must not initialize");

    // O 批次已完成，但接近 P001 时停止，没有执行 JBI 的 ENDDET。
    record->responseState.offline = 5;
    worker.updateState(record->responseState);
    worker.setTrackingRequested(false);
    record->requests.clear();
    ok &= expectTrue(worker.ensureOnlineCorrectionReady(),
                     "stopped completed-batch cycle may restart");
    ok &= expectTrue(std::count(record->requests.begin(), record->requests.end(),
                                kInitializationRequest) == 1,
                     "restart after interrupted approach must initialize again");
    return ok;
}

bool testInitializationFailure()
{
    auto transport = std::make_unique<CRecordingTransport>();
    auto *record = transport.get();
    ui::CRobotCommWorkerAdapter worker(std::move(transport));
    worker.pollRobotState(); // 已有成功诊断不能掩盖本次初始化失败。
    record->failInitialization = true;
    return expectTrue(!worker.ensureOnlineCorrectionReady(),
                      "failed initialization must not report online readiness");
}

bool testBlockedStates()
{
    bool ok = true;
    for (const bool freshResponse : {false, true}) {
        for (const bool unfinishedBatch : {false, true}) {
            auto transport = std::make_unique<CRecordingTransport>();
            auto *record = transport.get();
            ui::CRobotCommWorkerAdapter worker(std::move(transport));
            robot::RobotState blocked;
            blocked.sStart = unfinishedBatch ? 0 : 1;
            blocked.offline = unfinishedBatch ? 4 : 0;
            if (freshResponse) {
                record->responseState = blocked;
            } else {
                worker.updateState(blocked);
            }
            ok &= expectTrue(!worker.ensureOnlineCorrectionReady(),
                             "running job or unfinished batch must block readiness");
            ok &= expectTrue(std::count(record->requests.begin(), record->requests.end(),
                                        kInitializationRequest) == 0,
                             "blocked state must not send a destructive init pulse");
        }
    }
    for (const bool disconnected : {false, true}) {
        auto transport = std::make_unique<CRecordingTransport>();
        auto *record = transport.get();
        record->connected = !disconnected;
        record->failAll = !disconnected;
        ui::CRobotCommWorkerAdapter worker(std::move(transport));
        ok &= expectTrue(!worker.ensureOnlineCorrectionReady(),
                         "missing transport or valid D must block readiness");
        ok &= expectTrue(std::count(record->requests.begin(), record->requests.end(),
                                    kInitializationRequest) == 0,
                         "initialization requires a successful current status check");
    }
    return ok;
}

} // namespace

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    bool ok = testInitializationAndRestart();
    ok &= testInitializationFailure();
    ok &= testBlockedStates();
    if (!ok) {
        return 1;
    }
    std::cout << "PASS: online correction initialization boundary tests\n";
    return 0;
}
