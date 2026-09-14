/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_robot_comm_client_smoke.cpp
 * @brief       robot comm client smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <atomic>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <future>
#include <zmq.hpp>

#include "Client.h"
#include "ClientQueue.h"
#include "Config.h"

namespace {

constexpr const char* kRobotDResponse =
    "D 1,2,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,"
    "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";

std::uint16_t parsePort(const std::string& endpoint)
{
    const auto pos = endpoint.rfind(':');
    if (pos == std::string::npos || pos + 1 >= endpoint.size()) {
        return 0;
    }
    return static_cast<std::uint16_t>(
        std::stoi(endpoint.substr(pos + 1)));
}

bool runStreamEchoServer(
    std::promise<std::uint16_t> portPromise,
    std::atomic_bool& stopRequested)
{
    try {
        zmq::context_t context(1);
        zmq::socket_t server(context, zmq::socket_type::stream);
        server.set(zmq::sockopt::rcvtimeo, 200);
        server.set(zmq::sockopt::sndtimeo, 200);
        server.set(zmq::sockopt::linger, 0);
        server.bind("tcp://127.0.0.1:*");
        const std::string endpoint =
            server.get(zmq::sockopt::last_endpoint);
        portPromise.set_value(parsePort(endpoint));

        int replies = 0;
        while (!stopRequested.load() && replies < 2) {
            zmq::message_t identity;
            const auto identityReceived =
                server.recv(identity, zmq::recv_flags::none);
            if (!identityReceived.has_value()) {
                continue;
            }

            zmq::message_t payload;
            const auto payloadReceived =
                server.recv(payload, zmq::recv_flags::none);
            if (!payloadReceived.has_value()) {
                continue;
            }
            if (payload.empty()) {
                continue;
            }

            zmq::message_t replyIdentity(identity.data(), identity.size());
            zmq::message_t reply(kRobotDResponse, std::strlen(kRobotDResponse));
            if (!server.send(replyIdentity, zmq::send_flags::sndmore)
                || !server.send(reply, zmq::send_flags::none)) {
                return false;
            }
            ++replies;
        }
        return replies == 2;
    } catch (...) {
        try {
            portPromise.set_value(0);
        } catch (...) {
        }
        return false;
    }
}

} // namespace

int main()
{
    const bool defaultConfigLooksValid =
        i2r_rlc::robot_comm::config::ROBOT_IP[0] != '\0';

    if (!defaultConfigLooksValid) {
        std::cerr << "FAIL: RobotComm default endpoint is invalid\n";
        return 1;
    }

    if (i2r_rlc::robot_comm::Client::instance().isConnected()) {
        std::cerr << "FAIL: RobotComm client should not be connected before connect()\n";
        return 1;
    }

    std::atomic_bool stopRequested = false;
    std::promise<std::uint16_t> portPromise;
    auto portFuture = portPromise.get_future();
    auto serverResult = std::async(
        std::launch::async,
        runStreamEchoServer,
        std::move(portPromise),
        std::ref(stopRequested));
    const std::uint16_t port = portFuture.get();
    if (port == 0) {
        std::cerr << "FAIL: local STREAM test server did not bind\n";
        stopRequested.store(true);
        return 1;
    }

    auto& client = i2r_rlc::robot_comm::Client::instance();
    if (!client.connect("127.0.0.1", port)) {
        std::cerr << "FAIL: RobotComm client should connect to local STREAM server\n";
        stopRequested.store(true);
        return 1;
    }

    const auto first =
        client.sendAndReceiveDetailed("N 1,0,1,0,0,0,0,0,0");
    const auto second =
        client.sendAndReceiveDetailed("N 1,0,0,0,0,0,0,0,0");
    client.disconnect();
    stopRequested.store(true);

    if (!first || !second) {
        std::cerr << "FAIL: RobotComm client should support consecutive "
                     "transactions on one STREAM connection\n";
        return 1;
    }
    if (!serverResult.get()) {
        std::cerr << "FAIL: local STREAM server did not receive two requests\n";
        return 1;
    }

    std::cout << "PASS: RobotComm client smoke test\n";
    return 0;
}
