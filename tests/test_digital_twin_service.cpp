/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_digital_twin_service.cpp
 * @brief       digital twin service 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>
#include <memory>
#include <queue>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>

#include "DigitalTwinLocalizationVirtualReferenceProvider.h"
#include "DigitalTwinVisionTypes.h"
#include "IRobotSimTransport.h"
#include "NullDigitalTwinService.h"
#include "RobotSimDigitalTwinService.h"
#include "RobotSimEndpointConfigReader.h"

namespace {

class FakeRobotSimTransport final : public i2r_rlc::digital_twin::IRobotSimTransport {
public:
    i2r_rlc::digital_twin::RobotSimTransportResult connect(
        const i2r_rlc::digital_twin::RobotSimEndpointConfig& endpoint) override
    {
        lastEndpoint = endpoint;
        connected = endpoint.port > 0;
        return {
            connected,
            {},
            connected ? QStringLiteral("fake connected") : QStringLiteral("fake invalid endpoint"),
            0
        };
    }

    void disconnect() override
    {
        connected = false;
    }

    bool isConnected() const override
    {
        return connected;
    }

    i2r_rlc::digital_twin::RobotSimTransportResult send(
        const QJsonDocument& request,
        bool expectResponse) override
    {
        lastDocument = request;
        requests.push_back(request);
        lastExpectResponse = expectResponse;
        sendCount += 1;
        if (!queuedResults.empty()) {
            const i2r_rlc::digital_twin::RobotSimTransportResult result =
                queuedResults.front();
            queuedResults.pop();
            return result;
        }
        return {
            nextSendSucceeded,
            expectResponse ? nextPayload : QString(),
            nextSendSucceeded ? QStringLiteral("fake send ok") : QStringLiteral("fake send failed"),
            3
        };
    }

    bool connected = false;
    bool nextSendSucceeded = true;
    bool lastExpectResponse = false;
    int sendCount = 0;
    QString nextPayload = QStringLiteral("[1,2,3,4,5,6]");
    std::queue<i2r_rlc::digital_twin::RobotSimTransportResult> queuedResults;
    std::vector<QJsonDocument> requests;
    QJsonDocument lastDocument;
    i2r_rlc::digital_twin::RobotSimEndpointConfig lastEndpoint;
};

i2r_rlc::digital_twin::RobotSimTransportResult fakePayload(const QString& payload)
{
    return {
        true,
        payload,
        QStringLiteral("fake send ok"),
        3
    };
}

void expect(bool condition, const char* message, int& failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void verifyEndpointConfigReader(int& failures)
{
#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

    const QString filePath = QDir(QStringLiteral(I2R_RLC_PROJECT_SOURCE_DIR)).filePath(
        QStringLiteral("tests/fixtures/digital_twin/robotsim_endpoint_valid.txt"));

    const i2r_rlc::digital_twin::RobotSimEndpointConfigReader reader;
    const auto result = reader.readFile(filePath);
    expect(result.loaded, "endpoint config should load", failures);
    expect(result.config.host == QStringLiteral("127.0.0.1"), "endpoint host should match", failures);
    expect(result.config.port == 45678, "endpoint port should match", failures);
    expect(result.config.requestTimeoutMs == 321, "endpoint request timeout should match", failures);
}

void verifyRobotSimServiceProtocol(int& failures)
{
    auto fakeTransport = std::make_unique<FakeRobotSimTransport>();
    FakeRobotSimTransport* fakeTransportPtr = fakeTransport.get();
    i2r_rlc::digital_twin::RobotSimDigitalTwinService service(std::move(fakeTransport));

    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    const auto startResult = service.start(endpoint);
    expect(startResult.state == i2r_rlc::digital_twin::DigitalTwinServiceState::Idle,
        "service should enter Idle after fake connect",
        failures);

    const auto jointsResult = service.setRobotJoints(1002, { 1.0, 2.0, 3.0 });
    expect(jointsResult.succeeded, "setRobotJoints should succeed with fake transport", failures);
    QJsonArray lastOperation = fakeTransportPtr->lastDocument.array();
    expect(lastOperation.at(0).toString() == QStringLiteral("ROBOT_VOID"),
        "setRobotJoints op_mode should match",
        failures);
    expect(lastOperation.at(1).toString() == QStringLiteral("SET_Q_VEC"),
        "setRobotJoints op_type should match",
        failures);
    expect(!fakeTransportPtr->lastExpectResponse,
        "setRobotJoints should not wait for response",
        failures);

    const QJsonArray jointParams = lastOperation.at(2).toArray();
    expect(jointParams.size() == 2, "setRobotJoints param size should match", failures);
    expect(jointParams.at(0).toInt() == 1002, "setRobotJoints robot id should match", failures);
    expect(jointParams.at(1).toArray().size() == 3,
        "setRobotJoints joint vector should match",
        failures);

    i2r_rlc::digital_twin::DigitalTwinPose6D pose;
    pose.x_mm = 10.0;
    pose.y_mm = 20.0;
    pose.z_mm = 30.0;
    pose.rx_deg = 1.0;
    pose.ry_deg = 2.0;
    pose.rz_deg = 3.0;

    const auto tcpResult = service.setRobotTcpPose(1002, pose);
    expect(tcpResult.succeeded, "setRobotTcpPose should succeed", failures);
    lastOperation = fakeTransportPtr->lastDocument.array();
    expect(lastOperation.at(1).toString() == QStringLiteral("SET_POSE_FRAME_TCP"),
        "setRobotTcpPose op_type should match",
        failures);

    const auto objectPose = service.getObjectPose(1004);
    expect(objectPose.available, "getObjectPose should parse fake pose response", failures);
    expect(fakeTransportPtr->lastExpectResponse,
        "getObjectPose should wait for response",
        failures);
    expect(objectPose.pose.x_mm == 1.0 && objectPose.pose.rz_deg == 6.0,
        "getObjectPose parsed pose should match",
        failures);
    lastOperation = fakeTransportPtr->lastDocument.array();
    expect(lastOperation.at(1).toString() == QStringLiteral("GET_ABS_POSE"),
        "getObjectPose op_type should match",
        failures);

    const auto setObjectResult = service.setObjectPose(1004, pose, false, true);
    expect(setObjectResult.succeeded, "setObjectPose should succeed", failures);
    lastOperation = fakeTransportPtr->lastDocument.array();
    expect(lastOperation.at(0).toString() == QStringLiteral("BASE_OBJECT_VOID"),
        "setObjectPose op_mode should match",
        failures);
    expect(lastOperation.at(1).toString() == QStringLiteral("SET_POSE"),
        "setObjectPose op_type should match",
        failures);

    i2r_rlc::digital_twin::DigitalTwinWorkpieceSelection selection;
    selection.workpieceId = QStringLiteral("chufengkouxiao");
    selection.robotSimModelId = QStringLiteral("chufengkouxiao");
    selection.robotSimModelPath = QStringLiteral("models/weldment/chufengkouxiao");
    const auto selectionResult = service.selectWorkpiece(selection);
    expect(selectionResult.succeeded, "selectWorkpiece should succeed", failures);
    lastOperation = fakeTransportPtr->lastDocument.array();
    expect(lastOperation.at(0).toString() == QStringLiteral("DIGITAL_TWIN_VOID"),
        "selectWorkpiece op_mode should match",
        failures);
    expect(lastOperation.at(1).toString() == QStringLiteral("SET_WORKPIECE_MODEL"),
        "selectWorkpiece op_type should match",
        failures);

    fakeTransportPtr->requests.clear();
    fakeTransportPtr->queuedResults.push(fakePayload(
        QStringLiteral("[[[[[1,2,3],[10,20]],[[4,5,6],[11,21]]]],[[12,22]]]")));
    i2r_rlc::digital_twin::DigitalTwinVirtualReferenceRequest virtualRequest;
    virtualRequest.frameId = 12;
    virtualRequest.workpieceId = QStringLiteral("chufengkouxiao");
    virtualRequest.sensorId = 1013;
    const auto virtualResult = service.requestVirtualReference(virtualRequest);
    expect(virtualResult.available, "virtual reference should parse", failures);
    expect(virtualResult.laserLineImagePoints_px.size() == 2U,
        "virtual reference should parse 2D laser line",
        failures);
    expect(virtualResult.laserLinePoints_mm.size() == 2U,
        "virtual reference should parse 3D laser line",
        failures);
    expect(virtualResult.laserLinePoints_mm.front().x_mm == 1.0
            && virtualResult.laserLineImagePoints_px.front().x_px == 10.0,
        "virtual reference should preserve paired 3D and 2D laser points",
        failures);
    expect(virtualResult.hasSeamPointImage_px,
        "virtual reference should parse seam image point",
        failures);
    expect(fakeTransportPtr->requests.size() == 1U,
        "virtual reference should send one SensorLSL batch query",
        failures);
    const QJsonArray batchRequest = fakeTransportPtr->requests.at(0).array();
    expect(batchRequest.size() == 2,
        "virtual reference batch should contain two operations",
        failures);
    expect(batchRequest.at(0).toArray().at(1).toString()
            == QStringLiteral("GET_LSL_PHOTO"),
        "virtual reference 2D op_type should match",
        failures);
    expect(batchRequest.at(1).toArray().at(1).toString()
            == QStringLiteral("GET_WELD_SEAM_PHOTO"),
        "virtual reference seam op_type should match",
        failures);
    const QJsonArray sensorParams =
        batchRequest.at(0).toArray().at(2).toArray();
    expect(batchRequest.at(0).toArray().at(0).toString()
            == QStringLiteral("SENSOR_LSL_VALUE_RET"),
        "virtual reference op_mode should match",
        failures);
    expect(sensorParams.size() == 1 && sensorParams.at(0).toInt() == 1013,
        "virtual reference sensor id should match",
        failures);
}

void verifyVirtualReferenceHandlesBadSensorPayload(int& failures)
{
    auto fakeTransport = std::make_unique<FakeRobotSimTransport>();
    FakeRobotSimTransport* fakeTransportPtr = fakeTransport.get();
    fakeTransport->queuedResults.push(fakePayload(QStringLiteral("not-json")));
    i2r_rlc::digital_twin::RobotSimDigitalTwinService service(std::move(fakeTransport));
    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    service.start(endpoint);

    i2r_rlc::digital_twin::DigitalTwinVirtualReferenceRequest request;
    const auto result = service.requestVirtualReference(request);
    expect(!result.available,
        "invalid batch virtual reference should not be available",
        failures);
    expect(result.message.contains(QStringLiteral("SENSOR_LSL_VALUE_RET batch")),
        "invalid batch virtual reference should report batch parse failure",
        failures);
    expect(fakeTransportPtr->requests.size() == 1U,
        "invalid batch virtual reference should send one query",
        failures);
}

void verifyVirtualReferenceHandlesNullSensorPayload(int& failures)
{
    auto fakeTransport = std::make_unique<FakeRobotSimTransport>();
    fakeTransport->queuedResults.push(fakePayload(QStringLiteral("[null,null]")));
    i2r_rlc::digital_twin::RobotSimDigitalTwinService service(std::move(fakeTransport));
    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    service.start(endpoint);

    i2r_rlc::digital_twin::DigitalTwinVirtualReferenceRequest request;
    const auto result = service.requestVirtualReference(request);
    expect(!result.available,
        "null virtual reference should not be available",
        failures);
    expect(result.supported,
        "null virtual reference should keep protocol supported after transport success",
        failures);
    expect(result.message.contains(QStringLiteral("响应为 null")),
        "null virtual reference should report null batch items",
        failures);
}

void verifyVirtualReferenceBoundary(int& failures)
{
    i2r_rlc::digital_twin::NullDigitalTwinService nullService;
    i2r_rlc::digital_twin::DigitalTwinLocalizationVirtualReferenceProvider provider(nullService);

    i2r_rlc::localization::LocalizationVirtualReferenceRequest request;
    request.referenceId = QStringLiteral("fixture");
    request.needWorkpiecePointCloud = true;

    const auto snapshot = provider.requestSnapshot(request);
    expect(!snapshot.available,
        "virtual reference provider should report unavailable without RobotSim API",
        failures);
    expect(snapshot.referenceId == QStringLiteral("fixture"),
        "virtual reference provider should preserve reference id",
        failures);
}

void verifyVirtualReferenceProviderWithFakeRobotSim(int& failures)
{
    auto fakeTransport = std::make_unique<FakeRobotSimTransport>();
    fakeTransport->queuedResults.push(fakePayload(
        QStringLiteral("[[[[[1,0,0],[30,40]]]],[[30,40]]]")));
    i2r_rlc::digital_twin::RobotSimDigitalTwinService service(std::move(fakeTransport));
    i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
    endpoint.port = 40000;
    service.start(endpoint);

    i2r_rlc::digital_twin::DigitalTwinLocalizationVirtualReferenceProvider provider(service);
    i2r_rlc::localization::LocalizationVirtualReferenceRequest request;
    request.frameId = 21;
    request.referenceId = QStringLiteral("fixture");
    request.needWorkpiecePointCloud = true;
    request.needLaserLinePoints = true;

    const auto snapshot = provider.requestSnapshot(request);
    expect(snapshot.available,
        "provider should expose fake RobotSim virtual reference",
        failures);
    expect(snapshot.laserLineImagePoints_px.size() == 1U,
        "provider should convert 2D laser line points",
        failures);
    expect(snapshot.laserLinePoints_mm.size() == 1U,
        "provider should convert 3D laser line points",
        failures);
}

void verifyVisionObservationConversion(int& failures)
{
    i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult reference;
    reference.available = true;
    reference.frameId = 42;
    reference.hasSeamPointImage_px = true;
    reference.seamPointImage_px = {160.0, 120.0};
    reference.laserLineImagePoints_px = {
        {80.0, 120.0},
        {240.0, 120.0}
    };

    const auto observation =
        i2r_rlc::digital_twin::makeVisionLaserObservation(reference);
    expect(observation.valid,
        "vision observation conversion should succeed with line and seam point",
        failures);
    expect(observation.frameId == 42,
        "vision observation should preserve frame id",
        failures);
    expect(observation.seamIndex == -1,
        "vision observation should keep unknown seam index as -1",
        failures);
    expect(!observation.scanLineChangePoint_mm.has_value(),
        "vision observation should not fabricate scan line change point",
        failures);

    reference.laserLineImagePoints_px = {{80.0, 120.0}};
    const auto tooFewLaserPoints =
        i2r_rlc::digital_twin::makeVisionLaserObservation(reference);
    expect(!tooFewLaserPoints.valid,
        "vision observation conversion should reject short laser line",
        failures);

    reference.laserLineImagePoints_px = {{80.0, 120.0}, {240.0, 120.0}};
    reference.hasSeamPointImage_px = false;
    const auto missingSeamPoint =
        i2r_rlc::digital_twin::makeVisionLaserObservation(reference);
    expect(!missingSeamPoint.valid,
        "vision observation conversion should reject missing seam point",
        failures);

    reference.available = false;
    reference.frameId = 77;
    reference.message = QStringLiteral("RobotSim query timeout: request 77");
    const auto failedQuery =
        i2r_rlc::digital_twin::makeVisionLaserObservation(reference);
    expect(!failedQuery.valid,
        "failed virtual reference should remain invalid",
        failures);
    expect(failedQuery.frameId == 77,
        "failed vision observation should preserve frame id",
        failures);
    expect(failedQuery.message == reference.message,
        "failed vision observation should preserve original error",
        failures);

    reference.available = true;
    reference.frameId = 88;
    reference.message = QStringLiteral("RobotSim returned an empty laser line");
    reference.laserLineImagePoints_px.clear();
    const auto emptyObservation =
        i2r_rlc::digital_twin::makeVisionLaserObservation(reference);
    expect(!emptyObservation.valid,
        "empty virtual reference should remain invalid",
        failures);
    expect(emptyObservation.frameId == 88,
        "empty vision observation should preserve frame id",
        failures);
    expect(emptyObservation.message == reference.message,
        "empty vision observation should preserve original error",
        failures);
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);

    int failures = 0;

    verifyEndpointConfigReader(failures);
    verifyRobotSimServiceProtocol(failures);
    verifyVirtualReferenceHandlesBadSensorPayload(failures);
    verifyVirtualReferenceHandlesNullSensorPayload(failures);
    verifyVirtualReferenceBoundary(failures);
    verifyVirtualReferenceProviderWithFakeRobotSim(failures);
    verifyVisionObservationConversion(failures);

    return failures == 0 ? 0 : 1;
}
