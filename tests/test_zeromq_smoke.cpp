/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_zeromq_smoke.cpp
 * @brief       zeromq smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>
#include <string>
#include <vector>

#include <zmq_addon.hpp>
#include <zmq.hpp>

namespace {

void expect(bool condition, const char* message, int& failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool waitForInput(zmq::socket_t& socket, long timeoutMs)
{
    zmq::pollitem_t items[] = {
        { socket, 0, ZMQ_POLLIN, 0 }
    };

    return zmq::poll(items, 1, timeoutMs) > 0 && (items[0].revents & ZMQ_POLLIN);
}

std::vector<zmq::message_t> makeRobotSimStyleRequest()
{
    const std::string requestJson =
        R"({"op_mode":"BASE_OBJECT_VALUE_RET","op_type":"GET_NAME","param":[]})";

    std::vector<zmq::message_t> requestFrames;
    requestFrames.emplace_back();
    requestFrames.emplace_back(requestJson.data(), requestJson.size());
    return requestFrames;
}

std::string messageToString(const zmq::message_t& message)
{
    return message.to_string();
}

} // namespace

int main()
{
    int failures = 0;

    zmq::context_t context(1);
    zmq::socket_t router(context, zmq::socket_type::router);
    zmq::socket_t dealer(context, zmq::socket_type::dealer);

    router.set(zmq::sockopt::linger, 0);
    dealer.set(zmq::sockopt::linger, 0);

    router.bind("tcp://127.0.0.1:*");
    const std::string endpoint = router.get(zmq::sockopt::last_endpoint);
    dealer.connect(endpoint);

    auto requestFrames = makeRobotSimStyleRequest();
    expect(zmq::send_multipart(dealer, requestFrames, zmq::send_flags::none).has_value(), "DEALER should send multipart request", failures);

    expect(waitForInput(router, 2000), "ROUTER should receive request", failures);

    std::vector<zmq::message_t> receivedFrames;
    expect(zmq::recv_multipart(router, std::back_inserter(receivedFrames), zmq::recv_flags::none).has_value(), "ROUTER should receive multipart frames", failures);
    expect(receivedFrames.size() == 3, "ROUTER should receive client id, empty delimiter, and JSON payload", failures);

    if (receivedFrames.size() == 3) {
        expect(!receivedFrames[0].empty(), "client id frame should not be empty", failures);
        expect(receivedFrames[1].empty(), "delimiter frame should be empty", failures);
        expect(messageToString(receivedFrames[2]).find(R"("op_mode")") != std::string::npos, "payload should contain op_mode", failures);

        const std::string responseJson = R"({"ok":true})";
        std::vector<zmq::message_t> responseFrames;
        responseFrames.emplace_back(receivedFrames[0].data(), receivedFrames[0].size());
        responseFrames.emplace_back();
        responseFrames.emplace_back(responseJson.data(), responseJson.size());

        expect(zmq::send_multipart(router, responseFrames, zmq::send_flags::none).has_value(), "ROUTER should send multipart response", failures);
    }

    expect(waitForInput(dealer, 2000), "DEALER should receive response", failures);

    std::vector<zmq::message_t> dealerResponseFrames;
    expect(zmq::recv_multipart(dealer, std::back_inserter(dealerResponseFrames), zmq::recv_flags::none).has_value(), "DEALER should receive response frames", failures);
    expect(dealerResponseFrames.size() == 2, "DEALER should receive delimiter and JSON payload", failures);

    if (dealerResponseFrames.size() == 2) {
        expect(dealerResponseFrames[0].empty(), "response delimiter frame should be empty", failures);
        expect(messageToString(dealerResponseFrames[1]) == R"({"ok":true})", "response payload should match", failures);
    }

    dealer.close();
    router.close();
    context.close();

    return failures == 0 ? 0 : 1;
}
