/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimProbe.cpp
 * @brief     RobotSim ZeroMQ 手工联调探针。
 * @details   本工具用于向外部 RobotSim 进程发送任意 JSON 请求，便于黑盒验证
 *            当前 exe 支持的通信协议；它不是自动测试，不依赖 CI 环境启动 RobotSim。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 25 日
 */

#include <iostream>

#include <QJsonDocument>
#include <QString>

#include "RobotSimZmqTransport.h"

namespace {

void printUsage(const char* executableName)
{
    std::cerr
        << "Usage: " << executableName
        << " <port> <json> [expect_response:0|1]\n";
}

bool parseBoolFlag(const char* text)
{
    const QString value = QString::fromLocal8Bit(text).trimmed().toLower();
    return value != QStringLiteral("0")
        && value != QStringLiteral("false")
        && value != QStringLiteral("no");
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 3 || argc > 4) {
        printUsage(argv[0]);
        return 2;
    }

    bool ok = false;
    const quint16 port = QString::fromLocal8Bit(argv[1]).toUShort(&ok);
    if (!ok || port == 0) {
        std::cerr << "Invalid port: " << argv[1] << '\n';
        return 2;
    }

    QJsonParseError parseError;
    const QJsonDocument request = QJsonDocument::fromJson(
        QString::fromLocal8Bit(argv[2]).toUtf8(),
        &parseError);
    if (parseError.error != QJsonParseError::NoError || request.isNull()) {
        std::cerr
            << "Invalid JSON: "
            << parseError.errorString().toStdString()
            << '\n';
        return 2;
    }

    const bool expectResponse = argc < 4 || parseBoolFlag(argv[3]);

    i2r_rlc::digital_twin::RobotSimZmqTransport transport;
    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.host = QStringLiteral("127.0.0.1");
    endpoint.port = port;
    endpoint.requestTimeoutMs = 3000;

    const auto connectResult = transport.connect(endpoint);
    std::cout << connectResult.message.toStdString() << '\n';
    if (!connectResult.succeeded) {
        return 1;
    }

    const auto sendResult = transport.send(request, expectResponse);
    std::cout << sendResult.message.toStdString() << '\n';
    if (!sendResult.payload.isEmpty()) {
        std::cout << sendResult.payload.toStdString() << '\n';
    }

    return sendResult.succeeded ? 0 : 1;
}
