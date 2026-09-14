/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimDigitalTwinService.cpp
 * @brief     RobotSim 数字孪生服务实现。
 * @details   本文件实现 RobotSim JSON 命令构造、响应解析和服务状态同步；
 *            定位求解与视觉伺服控制不在本服务中执行。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include <nlohmann/json.hpp>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QStringList>

#include "RobotSimDigitalTwinService.h"
#include "RobotSimZmqTransport.h"

namespace i2r_rlc::digital_twin {

namespace {

constexpr std::size_t MOTOMAN_MH12_DOF = 6U;
constexpr std::size_t MA2P_300D_DOF = 2U;

QJsonArray poseToJsonArray(const DigitalTwinPose6D& pose)
{
    return {
        pose.x_mm,
        pose.y_mm,
        pose.z_mm,
        pose.rx_deg,
        pose.ry_deg,
        pose.rz_deg
    };
}

QJsonArray makeRobotSimOperation(
    const QString& operationMode,
    const QString& operationType,
    const QJsonArray& parameters)
{
    return { operationMode, operationType, parameters };
}

nlohmann::json makeSetJointsOperation(
    int objectId,
    const std::vector<double>& jointValues)
{
    if (jointValues.size() == MOTOMAN_MH12_DOF) {
        nlohmann::json reqJson = {
            "ROBOT_VOID",
            "SET_Q_VEC",
            {
                objectId,
                {
                    jointValues[0],
                    jointValues[1],
                    jointValues[2],
                    jointValues[3],
                    jointValues[4],
                    jointValues[5]
                }
            }
        };
        return reqJson;
    }
    if (jointValues.size() == MA2P_300D_DOF) {
        nlohmann::json reqJson = {
            "ROBOT_VOID",
            "SET_Q_VEC",
            { objectId, { jointValues[0], jointValues[1] } }
        };
        return reqJson;
    }

    nlohmann::json jointVector = nlohmann::json::array();
    for (double jointValue : jointValues) {
        jointVector.push_back(jointValue);
    }

    nlohmann::json reqJson = {
        "ROBOT_VOID",
        "SET_Q_VEC",
        { objectId, jointVector }
    };
    return reqJson;
}

bool hasExpectedJointVector(
    const nlohmann::json& operation,
    std::size_t expectedSize)
{
    return operation.is_array()
        && operation.size() == 3U
        && operation.at(2).is_array()
        && operation.at(2).size() == 2U
        && operation.at(2).at(1).is_array()
        && operation.at(2).at(1).size() == expectedSize;
}

bool serializedJointVectorHasExpectedSize(
    const QByteArray& payload,
    std::size_t expectedSize)
{
    const nlohmann::json parsed = nlohmann::json::parse(
        payload.constData(),
        payload.constData() + payload.size(),
        nullptr,
        false);
    return !parsed.is_discarded()
        && hasExpectedJointVector(parsed, expectedSize);
}

bool serializedBatchHasExpectedJointVectors(
    const QByteArray& payload,
    const DigitalTwinVirtualReferenceRequest& request)
{
    const nlohmann::json parsed = nlohmann::json::parse(
        payload.constData(),
        payload.constData() + payload.size(),
        nullptr,
        false);
    if (parsed.is_discarded() || !parsed.is_array()) {
        return false;
    }

    std::size_t operationIndex = 0U;
    if (!request.robotJointValuesRad.empty()) {
        if (operationIndex >= parsed.size()
            || !hasExpectedJointVector(
                parsed.at(operationIndex),
                request.robotJointValuesRad.size())) {
            return false;
        }
        ++operationIndex;
    }
    if (!request.positionerJointValuesRad.empty()) {
        if (operationIndex >= parsed.size()
            || !hasExpectedJointVector(
                parsed.at(operationIndex),
                request.positionerJointValuesRad.size())) {
            return false;
        }
    }
    return true;
}

nlohmann::json makeSensorOperation(
    const char* operationType,
    int sensorId,
    int referenceObjectId)
{
    nlohmann::json parameters = {sensorId};
    if (referenceObjectId > 0) {
        parameters.push_back(referenceObjectId);
    }
    nlohmann::json reqJson = {
        "SENSOR_LSL_VALUE_RET",
        operationType,
        std::move(parameters)
    };
    return reqJson;
}

QByteArray toUtf8Json(const nlohmann::json& reqJson)
{
    return QByteArray::fromStdString(reqJson.dump());
}

QByteArray sensorLslVirtualReferenceBatchPayload(
    const DigitalTwinVirtualReferenceRequest& request)
{
    nlohmann::json reqJson = nlohmann::json::array();
    if (!request.robotJointValuesRad.empty()) {
        reqJson.push_back(makeSetJointsOperation(
            request.robotId,
            request.robotJointValuesRad));
    }
    if (!request.positionerJointValuesRad.empty()) {
        reqJson.push_back(makeSetJointsOperation(
            request.positionerId,
            request.positionerJointValuesRad));
    }
    reqJson.push_back(makeSensorOperation(
        "GET_LSL_PHOTO",
        request.sensorId,
        request.referenceObjectId));
    if (request.needWeldSeamPoint) {
        reqJson.push_back(makeSensorOperation(
            "GET_WELD_SEAM_PHOTO",
            request.sensorId,
            request.referenceObjectId));
    }
    return toUtf8Json(reqJson);
}

bool isNumericArray(const QJsonArray& array, int expectedSize)
{
    if (array.size() != expectedSize) {
        return false;
    }

    for (int i = 0; i < expectedSize; ++i) {
        if (!array.at(i).isDouble()) {
            return false;
        }
    }
    return true;
}

bool readImagePoint2D(
    const QJsonValue& value,
    DigitalTwinImagePoint2D_px& point)
{
    if (!value.isArray()) {
        return false;
    }

    const QJsonArray array = value.toArray();
    if (!isNumericArray(array, 2)) {
        return false;
    }

    point = {
        array.at(0).toDouble(),
        array.at(1).toDouble()
    };
    return true;
}

bool readPoint3D(
    const QJsonValue& value,
    DigitalTwinPoint3D_mm& point)
{
    if (!value.isArray()) {
        return false;
    }

    const QJsonArray array = value.toArray();
    if (!isNumericArray(array, 3)) {
        return false;
    }

    point = {
        array.at(0).toDouble(),
        array.at(1).toDouble(),
        array.at(2).toDouble()
    };
    return true;
}

bool readPhotoPoint(
    const QJsonValue &value,
    DigitalTwinPoint3D_mm *worldPoint,
    DigitalTwinImagePoint2D_px *imagePoint)
{
    if (worldPoint == nullptr || imagePoint == nullptr || !value.isArray()) {
        return false;
    }
    const QJsonArray pointEntry = value.toArray();
    return pointEntry.size() == 2
        && readPoint3D(pointEntry.at(0), *worldPoint)
        && readImagePoint2D(pointEntry.at(1), *imagePoint);
}

int parseFirstLaserPhotoLine(
    const QJsonValue &value,
    std::vector<DigitalTwinPoint3D_mm>& worldPoints,
    std::vector<DigitalTwinImagePoint2D_px>& imagePoints)
{
    if (!value.isArray()) {
        return 0;
    }

    const QJsonArray laserLines = value.toArray();
    if (laserLines.isEmpty() || !laserLines.at(0).isArray()) {
        return laserLines.size();
    }
    const QJsonArray firstLine = laserLines.at(0).toArray();
    worldPoints.reserve(static_cast<std::size_t>(firstLine.size()));
    imagePoints.reserve(static_cast<std::size_t>(firstLine.size()));
    for (const QJsonValue &entry : firstLine) {
        DigitalTwinPoint3D_mm worldPoint;
        DigitalTwinImagePoint2D_px imagePoint;
        if (readPhotoPoint(entry, &worldPoint, &imagePoint)) {
            worldPoints.push_back(worldPoint);
            imagePoints.push_back(imagePoint);
        }
    }
    return laserLines.size();
}

bool parseFirstSeamPoint(
    const QJsonValue &value,
    DigitalTwinPoint3D_mm *worldPoint,
    DigitalTwinImagePoint2D_px *imagePoint,
    int *pointCount)
{
    if (worldPoint == nullptr || imagePoint == nullptr || !value.isArray()) {
        return false;
    }
    const QJsonArray seamPoints = value.toArray();
    if (pointCount != nullptr) {
        *pointCount = seamPoints.size();
    }
    if (seamPoints.isEmpty()) {
        return false;
    }

    // RobotSim 现场版本存在两种兼容响应：完整项为
    // [[X,Y,Z],[x,y]]，早期版本只返回 [x,y]。后者没有三维焊缝点，
    // 但二维点仍是 Vision 同帧约束所需的有效数据，不能因缺少可选三维值丢弃。
    if (readPhotoPoint(seamPoints.at(0), worldPoint, imagePoint)) {
        return true;
    }
    *worldPoint = {};
    return readImagePoint2D(seamPoints.at(0), *imagePoint);
}

void insertSeamPointIntoLaserLine(
    DigitalTwinVirtualReferenceResult& result)
{
    if (!result.hasSeamPointImage_px
        || !result.hasSeamPoint_mm
        || result.laserLineImagePoints_px.empty()) {
        return;
    }

    // 早期 RobotSim 只返回独立的二维焊缝点。此时没有三维点可证明它与
    // 当前激光线属于同一个空间交点，因此保持激光线原始点数，并通过
    // seamPointImage_px 单独交给 Vision；只有完整 3D+2D 响应才执行旧插入适配。

    constexpr double SAME_POINT_DISTANCE_SQUARED_PX2 = 0.25;
    const auto squaredDistance = [](const DigitalTwinImagePoint2D_px& first,
                                    const DigitalTwinImagePoint2D_px& second) {
        const double dx = first.x_px - second.x_px;
        const double dy = first.y_px - second.y_px;
        return dx * dx + dy * dy;
    };
    for (const auto& point : result.laserLineImagePoints_px) {
        if (squaredDistance(point, result.seamPointImage_px)
            <= SAME_POINT_DISTANCE_SQUARED_PX2) {
            return;
        }
    }

    const std::size_t originalPointCount =
        result.laserLineImagePoints_px.size();
    std::size_t insertionIndex = originalPointCount;
    double bestDistanceSquared_px2 = -1.0;
    for (std::size_t index = 0; index + 1 < originalPointCount; ++index) {
        const auto& start = result.laserLineImagePoints_px[index];
        const auto& end = result.laserLineImagePoints_px[index + 1];
        const double dx = end.x_px - start.x_px;
        const double dy = end.y_px - start.y_px;
        const double segmentLengthSquared_px2 = dx * dx + dy * dy;
        double ratio = 0.0;
        if (segmentLengthSquared_px2 > 0.0) {
            ratio = ((result.seamPointImage_px.x_px - start.x_px) * dx
                     + (result.seamPointImage_px.y_px - start.y_px) * dy)
                / segmentLengthSquared_px2;
            ratio = std::clamp(ratio, 0.0, 1.0);
        }
        const DigitalTwinImagePoint2D_px projection = {
            start.x_px + ratio * dx,
            start.y_px + ratio * dy};
        const double distanceSquared_px2 =
            squaredDistance(projection, result.seamPointImage_px);
        if (bestDistanceSquared_px2 < 0.0
            || distanceSquared_px2 < bestDistanceSquared_px2) {
            bestDistanceSquared_px2 = distanceSquared_px2;
            insertionIndex = index + 1;
        }
    }

    result.laserLineImagePoints_px.insert(
        result.laserLineImagePoints_px.begin()
            + static_cast<std::ptrdiff_t>(insertionIndex),
        result.seamPointImage_px);
}

bool validJointRequest(
    int objectId,
    const std::vector<double> &jointValues)
{
    return jointValues.empty()
        || (objectId > 0
            && std::all_of(
                jointValues.begin(),
                jointValues.end(),
                [](double value) { return std::isfinite(value); }));
}

bool parseJsonPayload(
    const QString& payload,
    const QString& operationType,
    QJsonDocument& document,
    QString& message)
{
    if (payload.trimmed() == QStringLiteral("null")) {
        message = QStringLiteral("%1 响应为 null。").arg(operationType);
        return false;
    }

    QJsonParseError parseError;
    document = QJsonDocument::fromJson(payload.toUtf8(), &parseError);
    if (parseError.error == QJsonParseError::NoError) {
        if (!document.isNull()) {
            return true;
        }

        message = QStringLiteral("%1 响应为 null。").arg(operationType);
        return false;
    }

    message = QStringLiteral("%1 响应不是合法 JSON：%2")
                  .arg(operationType, parseError.errorString());
    return false;
}

QJsonValue documentRootValue(const QJsonDocument& document)
{
    if (document.isArray()) {
        return document.array();
    }

    if (document.isObject()) {
        return document.object();
    }

    return {};
}

QJsonValue batchItemOrNull(
    const QJsonValue& batchRoot,
    int index)
{
    if (!batchRoot.isArray()) {
        return {};
    }

    const QJsonArray array = batchRoot.toArray();
    if (array.size() <= index) {
        return {};
    }
    return array.at(index);
}

void appendMessage(QStringList& messages, const QString& message)
{
    if (!message.trimmed().isEmpty()) {
        messages.push_back(message);
    }
}

DigitalTwinPoseResult parsePoseResult(const QString& payload)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(payload.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        return {
            false,
            {},
            QStringLiteral("RobotSim 位姿响应不是合法 JSON：%1").arg(parseError.errorString())
        };
    }

    if (document.isNull() || document.isEmpty() || document.isObject()) {
        return {
            false,
            {},
            QStringLiteral("RobotSim 位姿响应为空或格式不支持。")
        };
    }

    const QJsonArray poseArray = document.array();
    if (poseArray.size() < 6) {
        return {
            false,
            {},
            QStringLiteral("RobotSim 位姿响应字段不足。")
        };
    }

    DigitalTwinPose6D pose;
    pose.x_mm = poseArray.at(0).toDouble();
    pose.y_mm = poseArray.at(1).toDouble();
    pose.z_mm = poseArray.at(2).toDouble();
    pose.rx_deg = poseArray.at(3).toDouble();
    pose.ry_deg = poseArray.at(4).toDouble();
    pose.rz_deg = poseArray.at(5).toDouble();
    return {
        true,
        pose,
        QStringLiteral("RobotSim 位姿查询成功。")
    };
}

} // namespace

RobotSimDigitalTwinService::RobotSimDigitalTwinService()
    : RobotSimDigitalTwinService(std::make_unique<RobotSimZmqTransport>())
{
}

RobotSimDigitalTwinService::RobotSimDigitalTwinService(
    std::unique_ptr<IRobotSimTransport> transport)
    : m_transport(std::move(transport))
{
}

RobotSimDigitalTwinService::~RobotSimDigitalTwinService()
{
    stop();
}

DigitalTwinStartResult RobotSimDigitalTwinService::start()
{
    return start(m_status.endpoint);
}

DigitalTwinStartResult RobotSimDigitalTwinService::start(
    const RobotSimEndpointConfig& endpoint)
{
    if (!m_transport) {
        m_status.state = DigitalTwinServiceState::Error;
        m_status.lastError = QStringLiteral("RobotSim transport 未创建。");
        return { m_status.state, m_status.lastError };
    }

    m_status.endpoint = endpoint;
    const RobotSimTransportResult result = m_transport->connect(endpoint);
    updateFromTransportResult(result, DigitalTwinServiceState::Idle);

    return {
        m_status.state,
        result.message
    };
}

void RobotSimDigitalTwinService::stop()
{
    if (m_transport) {
        m_transport->disconnect();
    }

    m_status.state = DigitalTwinServiceState::Unavailable;
}

DigitalTwinStatusSnapshot RobotSimDigitalTwinService::status() const
{
    return m_status;
}

DigitalTwinCommandResult RobotSimDigitalTwinService::setRobotJoints(
    int robotId,
    const std::vector<double>& jointValues)
{
    if (robotId <= 0
        || jointValues.empty()
        || !validJointRequest(robotId, jointValues)) {
        return {
            false,
            QStringLiteral("RobotSim SET_Q_VEC 的对象 ID 或关节向量无效。")
        };
    }

    const nlohmann::json reqJson = makeSetJointsOperation(robotId, jointValues);
    if (!hasExpectedJointVector(reqJson, jointValues.size())) {
        return {
            false,
            QStringLiteral(
                "RobotSim SET_Q_VEC 序列化失败：对象 %1 的 %2 项关节值未形成 JSON 数组。")
                .arg(robotId)
                .arg(static_cast<qulonglong>(jointValues.size()))
        };
    }
    const QByteArray payload = toUtf8Json(reqJson);
    if (!serializedJointVectorHasExpectedSize(payload, jointValues.size())) {
        return {
            false,
            QStringLiteral(
                "RobotSim SET_Q_VEC 序列化复核失败：对象 %1 的关节数组长度不是 %2。")
                .arg(robotId)
                .arg(static_cast<qulonglong>(jointValues.size()))
        };
    }
    return sendCommandRaw(payload);
}

DigitalTwinCommandResult RobotSimDigitalTwinService::synchronizeJointVectors(
    int robotId,
    const std::vector<double>& robotJointValues,
    int positionerId,
    const std::vector<double>& positionerJointValues)
{
    if (robotJointValues.size() != MOTOMAN_MH12_DOF
        || positionerJointValues.size() != MA2P_300D_DOF
        || !validJointRequest(robotId, robotJointValues)
        || !validJointRequest(positionerId, positionerJointValues)) {
        return {
            false,
            QStringLiteral(
                "RobotSim 关节同步参数无效：MH12 需要 6 项，当前 %1 项；"
                "MA2P-300D 需要 2 项，当前 %2 项。")
                .arg(static_cast<qulonglong>(robotJointValues.size()))
                .arg(static_cast<qulonglong>(positionerJointValues.size()))
        };
    }

    const DigitalTwinCommandResult robotResult =
        setRobotJoints(robotId, robotJointValues);
    if (!robotResult.succeeded) {
        return robotResult;
    }
    return setRobotJoints(positionerId, positionerJointValues);
}

DigitalTwinCommandResult RobotSimDigitalTwinService::setRobotTcpPose(
    int robotId,
    const DigitalTwinPose6D& pose)
{
    QJsonArray parameters;
    parameters.push_back(robotId);
    parameters.push_back(poseToJsonArray(pose));

    return sendCommand(QJsonDocument(makeRobotSimOperation(
        QStringLiteral("ROBOT_VOID"),
        QStringLiteral("SET_POSE_FRAME_TCP"),
        parameters)));
}

DigitalTwinPoseResult RobotSimDigitalTwinService::getObjectPose(int objectId)
{
    QJsonArray parameters;
    parameters.push_back(objectId);

    const RobotSimTransportResult result = sendQuery(QJsonDocument(makeRobotSimOperation(
        QStringLiteral("BASE_OBJECT_VALUE_RET"),
        QStringLiteral("GET_ABS_POSE"),
        parameters)));
    if (!result.succeeded) {
        return {
            false,
            {},
            result.message
        };
    }

    DigitalTwinPoseResult poseResult = parsePoseResult(result.payload);
    if (!poseResult.available) {
        m_status.state = DigitalTwinServiceState::Error;
        m_status.lastError = poseResult.message;
    }
    return poseResult;
}

DigitalTwinCommandResult RobotSimDigitalTwinService::setObjectPose(
    int objectId,
    const DigitalTwinPose6D& pose,
    bool isRelative,
    bool updateChildren)
{
    QJsonArray parameters;
    parameters.push_back(objectId);
    parameters.push_back(poseToJsonArray(pose));
    parameters.push_back(isRelative);
    parameters.push_back(updateChildren);

    return sendCommand(QJsonDocument(makeRobotSimOperation(
        QStringLiteral("BASE_OBJECT_VOID"),
        QStringLiteral("SET_POSE"),
        parameters)));
}

DigitalTwinCommandResult RobotSimDigitalTwinService::selectWorkpiece(
    const DigitalTwinWorkpieceSelection& selection)
{
    QJsonArray parameters;
    parameters.push_back(selection.workpieceId);
    parameters.push_back(selection.robotSimModelId);
    parameters.push_back(selection.robotSimModelPath);

    return sendCommand(QJsonDocument(makeRobotSimOperation(
        QStringLiteral("DIGITAL_TWIN_VOID"),
        QStringLiteral("SET_WORKPIECE_MODEL"),
        parameters)));
}

DigitalTwinVirtualReferenceResult RobotSimDigitalTwinService::requestVirtualReference(
    const DigitalTwinVirtualReferenceRequest& request)
{
    DigitalTwinVirtualReferenceRequest effectiveRequest = request;
    if (effectiveRequest.referenceObjectId <= 0) {
        effectiveRequest.referenceObjectId =
            m_status.endpoint.referenceObjectId;
    }

    DigitalTwinVirtualReferenceResult result;
    result.available = false;
    result.supported = true;
    result.frameId = request.frameId;
    result.workpieceId = request.workpieceId;
    result.referencePose = request.workpiecePose;
    result.referenceObjectId = effectiveRequest.referenceObjectId > 0
        ? effectiveRequest.referenceObjectId
        : 0;

    if (request.sensorId <= 0
        || !validJointRequest(request.robotId, request.robotJointValuesRad)
        || !validJointRequest(
            request.positionerId,
            request.positionerJointValuesRad)
        || (!request.robotJointValuesRad.empty()
            && request.robotJointValuesRad.size() != MOTOMAN_MH12_DOF)
        || (!request.positionerJointValuesRad.empty()
            && request.positionerJointValuesRad.size() != MA2P_300D_DOF)) {
        result.supported = false;
        result.message = QStringLiteral(
            "RobotSim 虚拟参考请求的对象 ID 或关节值无效："
            "MH12=%1 项，MA2P-300D=%2 项。")
            .arg(static_cast<qulonglong>(
                request.robotJointValuesRad.size()))
            .arg(static_cast<qulonglong>(
                request.positionerJointValuesRad.size()));
        return result;
    }

    const QByteArray payload =
        sensorLslVirtualReferenceBatchPayload(effectiveRequest);
    if (!serializedBatchHasExpectedJointVectors(payload, effectiveRequest)) {
        result.supported = false;
        result.message = QStringLiteral(
            "RobotSim 批量请求序列化复核失败：MH12 或 MA2P-300D 关节数组长度错误。");
        return result;
    }

    const RobotSimTransportResult transportResult = sendQueryRaw(payload);
    if (!transportResult.succeeded) {
        result.supported = false;
        result.message = transportResult.message;
        m_status.state = DigitalTwinServiceState::Error;
        m_status.lastError = result.message;
        return result;
    }

    QJsonDocument document;
    QString parseMessage;
    if (!parseJsonPayload(
            transportResult.payload,
            QStringLiteral("SENSOR_LSL_VALUE_RET batch"),
            document,
            parseMessage)) {
        result.supported = false;
        result.message = parseMessage;
        m_status.state = DigitalTwinServiceState::Error;
        m_status.lastError = result.message;
        return result;
    }

    QStringList messages;
    const QJsonValue rootValue = documentRootValue(document);
    const QJsonValue lslPhotoValue = batchItemOrNull(rootValue, 0);

    if (request.needLaserLineImagePoints || request.needLaserLinePoints) {
        std::vector<DigitalTwinPoint3D_mm> laserLinePoints;
        std::vector<DigitalTwinImagePoint2D_px> laserLineImagePoints;
        const int laserLineCount = parseFirstLaserPhotoLine(
            lslPhotoValue,
            laserLinePoints,
            laserLineImagePoints);

        if (request.needLaserLinePoints) {
            result.laserLinePoints_mm = std::move(laserLinePoints);
        }
        if (request.needLaserLineImagePoints) {
            result.laserLineImagePoints_px = std::move(laserLineImagePoints);
        }
        if (lslPhotoValue.isNull() || lslPhotoValue.isUndefined()) {
            appendMessage(messages, QStringLiteral("GET_LSL_PHOTO 响应为 null。"));
        } else if (result.laserLineImagePoints_px.empty()
                   && result.laserLinePoints_mm.empty()) {
            appendMessage(messages, QStringLiteral("GET_LSL_PHOTO 响应为空数组。"));
        } else if (laserLineCount > 1) {
            appendMessage(
                messages,
                QStringLiteral(
                    "GET_LSL_PHOTO 返回 %1 条线结构光，按当前场景只使用第一条。")
                    .arg(laserLineCount));
        }
    }

    if (request.needWeldSeamPoint) {
        const QJsonValue weldSeamPhotoValue = batchItemOrNull(rootValue, 1);
        int seamPointCount = 0;
        if (parseFirstSeamPoint(
                weldSeamPhotoValue,
                &result.seamPoint_mm,
                &result.seamPointImage_px,
                &seamPointCount)) {
            result.hasSeamPointImage_px = true;
            const QJsonArray seamPoints = weldSeamPhotoValue.toArray();
            result.hasSeamPoint_mm = !seamPoints.isEmpty()
                && seamPoints.at(0).isArray()
                && seamPoints.at(0).toArray().size() == 2
                && seamPoints.at(0).toArray().at(0).isArray();
            if (seamPointCount > 1) {
                appendMessage(
                    messages,
                    QStringLiteral(
                        "GET_WELD_SEAM_PHOTO 返回 %1 个焊缝点，使用第一条结构光的焊缝点。")
                        .arg(seamPointCount));
            }
        } else if (weldSeamPhotoValue.isNull()
                   || weldSeamPhotoValue.isUndefined()) {
            appendMessage(
                messages,
                QStringLiteral("GET_WELD_SEAM_PHOTO 响应为 null。"));
        } else {
            appendMessage(
                messages,
                QStringLiteral(
                    "GET_WELD_SEAM_PHOTO 响应为空数组或格式不符合 "
                    "[[[X,Y,Z],[x,y]], ...]。"));
        }
    }

    // RobotSim 的焊缝点理论上属于当前结构光线。按最近线段插入可保持
    // 折线顺序，避免简单追加后从线尾跨接到焊缝点。
    insertSeamPointIntoLaserLine(result);

    result.supported = true;
    result.available = !result.laserLineImagePoints_px.empty()
        || !result.laserLinePoints_mm.empty()
        || !result.localPointCloud_mm.empty()
        || result.hasSeamPoint_mm
        || result.hasSeamPointImage_px;
    if (messages.isEmpty()) {
        result.message = result.available
            ? QStringLiteral("RobotSim 线结构光虚拟参考查询成功。")
            : QStringLiteral("RobotSim 响应成功但未包含可用虚拟参考数据。");
    } else {
        result.message = messages.join(QStringLiteral("；"));
    }
    if (!result.available) {
        m_status.state = result.supported
            ? DigitalTwinServiceState::Idle
            : DigitalTwinServiceState::Error;
        m_status.lastError = result.message;
    }
    return result;
}

DigitalTwinCommandResult RobotSimDigitalTwinService::sendCommand(
    const QJsonDocument& request)
{
    if (!m_transport || !m_transport->isConnected()) {
        m_status.state = DigitalTwinServiceState::Unavailable;
        m_status.lastError = QStringLiteral("RobotSim transport 未连接。");
        return { false, m_status.lastError };
    }

    const RobotSimTransportResult result = m_transport->send(request, false);
    updateFromTransportResult(result, DigitalTwinServiceState::Running);
    return {
        result.succeeded,
        result.message
    };
}

DigitalTwinCommandResult RobotSimDigitalTwinService::sendCommandRaw(
    const QByteArray& payload)
{
    if (!m_transport || !m_transport->isConnected()) {
        m_status.state = DigitalTwinServiceState::Unavailable;
        m_status.lastError = QStringLiteral("RobotSim transport 未连接。");
        return { false, m_status.lastError };
    }

    const RobotSimTransportResult result = m_transport->sendRaw(payload, false);
    updateFromTransportResult(result, DigitalTwinServiceState::Running);
    return {
        result.succeeded,
        result.message
    };
}

RobotSimTransportResult RobotSimDigitalTwinService::sendQuery(
    const QJsonDocument& request)
{
    if (!m_transport || !m_transport->isConnected()) {
        m_status.state = DigitalTwinServiceState::Unavailable;
        m_status.lastError = QStringLiteral("RobotSim transport 未连接。");
        return {
            false,
            {},
            m_status.lastError,
            -1
        };
    }

    const RobotSimTransportResult result = m_transport->send(request, true);
    updateFromTransportResult(result, DigitalTwinServiceState::Running);
    return result;
}

RobotSimTransportResult RobotSimDigitalTwinService::sendQueryRaw(
    const QByteArray& payload)
{
    if (!m_transport || !m_transport->isConnected()) {
        m_status.state = DigitalTwinServiceState::Unavailable;
        m_status.lastError = QStringLiteral("RobotSim transport 未连接。");
        return {
            false,
            {},
            m_status.lastError,
            -1
        };
    }

    const RobotSimTransportResult result = m_transport->sendRaw(payload, true);
    updateFromTransportResult(result, DigitalTwinServiceState::Running);
    return result;
}

void RobotSimDigitalTwinService::updateFromTransportResult(
    const RobotSimTransportResult& result,
    DigitalTwinServiceState successState)
{
    m_status.lastResponseTimeMs = result.elapsedMs;
    if (result.succeeded) {
        m_status.state = successState;
        m_status.lastError.clear();
    } else {
        m_status.state = DigitalTwinServiceState::Error;
        m_status.lastError = result.message;
    }
}

} // namespace i2r_rlc::digital_twin
