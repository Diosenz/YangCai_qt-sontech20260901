/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        RuntimeWorkerAdapters.cpp
 * @brief       运行时线程 worker 适配器实现
 * @details     将旧 OpencvThread 拆分为相机帧、机器人状态、视觉跟踪和在线纠偏事件。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef I2R_RLC_HAS_OPENCV
#include <opencv2/imgproc.hpp>
#endif

#include <QByteArray>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QSaveFile>
#include <QTextStream>
#include <QTimer>

#include "Config.h"
#include "preprocess/ImageDenoise.h"
#include "stripe/StegerStripeDetector.h"
#include "stripe/StripeDetectionTypes.h"
#include "RuntimeWorkerAdapters.h"

namespace i2r_rlc::ui {

namespace {

constexpr int kRobotPollIntervalMs = 100;
constexpr int kRobotReconnectBaseDelayMs = 100;
constexpr int kRobotReconnectMaxDelayMs = 5000;
constexpr int kRobotReconnectFailureThreshold = 3;
constexpr int kRobotSimManualPollIntervalMs = 100;
constexpr int kOfflinePreflightTimeoutMs = 3000;
constexpr int kOfflineB040GraceMs = 2000;
constexpr int kOfflineRobotAckTimeoutMs = 10000;
constexpr int kOnlineInitialDetectionTimeoutMs = 120000;
constexpr int kOnlineTeachPointTimeoutMs = 15000;
constexpr int kOnlineInitialApproachTimeoutMs = 90000;
constexpr int kOnlineStartTimeoutMs = 10000;
constexpr int kOnlineFinishHandshakeTimeoutMs = 10000;
constexpr int kOnlineProgramStopTimeoutMs = 120000;

Eigen::Matrix4d robotSimPoseToMatrix(
    const digital_twin::DigitalTwinPose6D &pose)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const Eigen::AngleAxisd rx(
        pose.rx_deg * kDegToRad,
        Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(
        pose.ry_deg * kDegToRad,
        Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(
        pose.rz_deg * kDegToRad,
        Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    // RobotSim 世界固定轴 X-Y-Z 欧拉角对应旋转矩阵 Rz*Ry*Rx。
    matrix.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
    matrix.block<3, 1>(0, 3) =
        Eigen::Vector3d(pose.x_mm, pose.y_mm, pose.z_mm);
    return matrix;
}

Eigen::Matrix4d rigidTransformToMatrix(
    const localization::RigidTransform &transform)
{
    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) =
                transform.rowMajor[static_cast<std::size_t>(row * 4 + column)];
        }
    }
    return matrix;
}

/** @brief 将内部矩阵复制为可跨线程归档的值对象，不暴露 Eigen 类型。 */
localization::RigidTransform matrixToRigidTransform(const Eigen::Matrix4d &matrix)
{
    localization::RigidTransform transform;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            transform.rowMajor[static_cast<std::size_t>(row * 4 + column)] = matrix(row, column);
        }
    }
    return transform;
}

/** @brief 事务一致性比较；容差只覆盖位姿协议的浮点序列化误差。 */
bool localRegistrationPosesMatch(const Eigen::Matrix4d &actual, const Eigen::Matrix4d &expected)
{
    return actual.allFinite() && expected.allFinite()
        && (actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm() < 1e-3
        && (actual.block<3, 3>(0, 0) - expected.block<3, 3>(0, 0)).norm() < 1e-5;
}

digital_twin::DigitalTwinPose6D matrixToRobotSimPose(
    const Eigen::Matrix4d &matrix)
{
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
    double rxRad = 0.0;
    double ryRad = 0.0;
    double rzRad = 0.0;
    const double sy = std::clamp(-rotation(2, 0), -1.0, 1.0);
    if (std::abs(sy) < 1.0 - std::numeric_limits<double>::epsilon()) {
        ryRad = std::asin(sy);
        rxRad = std::atan2(rotation(2, 1), rotation(2, 2));
        rzRad = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        ryRad = sy > 0.0
            ? 3.14159265358979323846 / 2.0
            : -3.14159265358979323846 / 2.0;
        rzRad = std::atan2(-rotation(0, 1), rotation(1, 1));
    }
    return {
        matrix(0, 3),
        matrix(1, 3),
        matrix(2, 3),
        rxRad * kRadToDeg,
        ryRad * kRadToDeg,
        rzRad * kRadToDeg
    };
}

bool writeGlobalLocatePlyPointCloudFile(
    const QString &path,
    const std::vector<localization::Point3D_mm> &points,
    QString *errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    QTextStream stream(&file);
    stream << "ply\n";
    stream << "format ascii 1.0\n";
    stream << "comment coordinate_frame positioner_tcp\n";
    stream << "comment units millimeter\n";
    stream << "element vertex " << static_cast<qulonglong>(points.size()) << '\n';
    stream << "property double x\n";
    stream << "property double y\n";
    stream << "property double z\n";
    stream << "end_header\n";
    for (const auto &point : points) {
        stream << point.x_mm << ' ' << point.y_mm << ' ' << point.z_mm << '\n';
    }
    return true;
}

bool writeGlobalLocateTextFile(
    const QString &path,
    const QString &text,
    QString *errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    QTextStream stream(&file);
    stream << text;
    return true;
}

/**
 * @brief 原子覆盖单个离线纠偏诊断文本，避免中断时留下半个文件。
 */
bool writeOfflineCorrectionTextFile(
    const QString &path,
    const QString &text,
    QString *errorMessage)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    const QByteArray bytes = text.toUtf8();
    if (file.write(bytes) != bytes.size() || !file.commit()) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }
    return true;
}

/**
 * @brief 将毫米点集编码为无表头的三列空格分隔文本。
 */
QString offlineCorrectionPointsText(
    const std::vector<vision::WeldSpacePoint> &points)
{
    QString text;
    QTextStream stream(&text);
    stream.setRealNumberNotation(QTextStream::SmartNotation);
    stream.setRealNumberPrecision(17);
    for (const auto &point : points) {
        stream << point.xMm << ' ' << point.yMm << ' ' << point.zMm << '\n';
    }
    return text;
}

/**
 * @brief 覆盖保存最近一次已由控制柜确认的离线纠偏诊断快照。
 */
bool saveOfflineCorrectionDiagnostics(
    const QString &directoryPath,
    const control::OfflineCorrectionDiagnosticData &diagnostics,
    const control::OfflineCorrectionMatchReport &report,
    QString *errorMessage)
{
    if (directoryPath.trimmed().isEmpty() || !diagnostics.valid) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("诊断输出目录为空或诊断快照无效");
        }
        return false;
    }
    if (!QDir().mkpath(directoryPath)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法创建目录：%1").arg(directoryPath);
        }
        return false;
    }

    const QDir outputDirectory(directoryPath);
    QString transformText;
    QTextStream transformStream(&transformText);
    transformStream.setRealNumberNotation(QTextStream::SmartNotation);
    transformStream.setRealNumberPrecision(17);
    for (int index = 0; index < 16; ++index) {
        transformStream << diagnostics.modelToDetectionTransformRowMajor[
            static_cast<std::size_t>(index)];
        transformStream << (index % 4 == 3 ? '\n' : ' ');
    }

    QString reportText;
    QTextStream reportStream(&reportText);
    reportStream.setRealNumberNotation(QTextStream::SmartNotation);
    reportStream.setRealNumberPrecision(17);
    reportStream << "status=completed\n";
    reportStream << "saved_at_local="
                 << QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
                 << '\n';
    reportStream << "completion_condition=all_O_points_and_D.offline_5\n";
    reportStream << "transform_direction=model_to_detection\n";
    reportStream << "point_units=mm\n";
    reportStream << "point_file_columns=x_mm y_mm z_mm\n";
    reportStream << "detection_point_file=icp_detection_points.txt\n";
    reportStream << "model_file="
                 << QString::fromStdString(diagnostics.modelFilePath) << '\n';
    reportStream << "teach_file="
                 << QString::fromStdString(diagnostics.teachFilePath) << '\n';
    reportStream << "model_point_count=" << report.modelPointCount << '\n';
    reportStream << "detection_point_count=" << report.detectionPointCount << '\n';
    reportStream << "teach_point_count=" << report.teachPointCount << '\n';
    reportStream << "mean_error_mm=" << report.meanErrorMm << '\n';
    reportStream << "rms_error_mm=" << report.rmsErrorMm << '\n';
    reportStream << "confidence=" << report.confidence << '\n';

    const std::array<std::pair<QString, QString>, 5> files{{
        {QStringLiteral("icp_detection_points.txt"),
         offlineCorrectionPointsText(diagnostics.detectionPoints)},
        {QStringLiteral("icp_transform.txt"), transformText},
        {QStringLiteral("transformed_model_points.txt"),
         offlineCorrectionPointsText(diagnostics.transformedModelPoints)},
        {QStringLiteral("transformed_teach_points.txt"),
         offlineCorrectionPointsText(diagnostics.transformedTeachPoints)},
        {QStringLiteral("match_report.txt"), reportText}
    }};
    for (const auto &file : files) {
        const QString path = outputDirectory.filePath(file.first);
        if (!writeOfflineCorrectionTextFile(path, file.second, errorMessage)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("%1：%2").arg(path, *errorMessage);
            }
            return false;
        }
    }
    return true;
}

cv::Mat cloneContiguousMat(const QImage &image, int cvType)
{
    const cv::Mat wrapped(
        image.height(),
        image.width(),
        cvType,
        const_cast<uchar *>(image.constBits()),
        static_cast<size_t>(image.bytesPerLine()));
    return wrapped.clone();
}

cv::Mat qImageToGray8(const QImage &image)
{
    if (image.isNull()) {
        return {};
    }
    cv::Mat cvImage;
    switch (image.format()) {
    case QImage::Format_Grayscale8:
        return cloneContiguousMat(image, CV_8UC1);
    case QImage::Format_RGB888: {
        cv::Mat rgb = cloneContiguousMat(image, CV_8UC3);
        cv::cvtColor(rgb, cvImage, cv::COLOR_RGB2GRAY);
        return cvImage;
    }
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    case QImage::Format_BGR888: {
        cv::Mat bgr = cloneContiguousMat(image, CV_8UC3);
        cv::cvtColor(bgr, cvImage, cv::COLOR_BGR2GRAY);
        return cvImage;
    }
#endif
    default: {
        const QImage converted = image.convertToFormat(QImage::Format_Grayscale8);
        return converted.isNull() ? cv::Mat{} : cloneContiguousMat(converted, CV_8UC1);
    }
    }
}

QImage makeTrackingOverlay(
    const cv::Mat &processedImage,
    const cv::Point2d &weldPointPixel,
    bool valid)
{
    if (processedImage.empty()) {
        return {};
    }

    cv::Mat bgr;
    if (processedImage.type() == CV_8UC1) {
        cv::cvtColor(processedImage, bgr, cv::COLOR_GRAY2BGR);
    } else if (processedImage.type() == CV_8UC3) {
        bgr = processedImage.clone();
    } else {
        return {};
    }

    if (valid
        && std::isfinite(weldPointPixel.x)
        && std::isfinite(weldPointPixel.y)) {
        constexpr int kCrossHalfSizePixel = 15;
        constexpr int kCrossThicknessPixel = 2;
        const cv::Point center(
            static_cast<int>(std::lround(weldPointPixel.x)),
            static_cast<int>(std::lround(weldPointPixel.y)));
        cv::line(
            bgr,
            cv::Point(center.x - kCrossHalfSizePixel, center.y),
            cv::Point(center.x + kCrossHalfSizePixel, center.y),
            cv::Scalar(0, 255, 0),
            kCrossThicknessPixel,
            cv::LINE_AA);
        cv::line(
            bgr,
            cv::Point(center.x, center.y - kCrossHalfSizePixel),
            cv::Point(center.x, center.y + kCrossHalfSizePixel),
            cv::Scalar(0, 255, 0),
            kCrossThicknessPixel,
            cv::LINE_AA);
    }

    QImage image(
        bgr.data,
        bgr.cols,
        bgr.rows,
        static_cast<int>(bgr.step),
        QImage::Format_RGB888);
    return image.rgbSwapped().copy();
}

bool constrainCenterlineToVirtualLaserCorridor(
    vision::StripeCenterlineResult *centerline,
    const std::vector<vision::MyLaserImagePoint> &simPoints,
    int corridorRadiusPixel)
{
    if (centerline == nullptr
        || simPoints.size() < 2U
        || corridorRadiusPixel <= 0) {
        return false;
    }

    const double maximumSquaredDistance =
        static_cast<double>(corridorRadiusPixel * corridorRadiusPixel);
    std::vector<cv::Point2d> filteredPoints;
    filteredPoints.reserve(centerline->centerlinePoints.size());
    for (const cv::Point2d &point : centerline->centerlinePoints) {
        double minimumSquaredDistance = std::numeric_limits<double>::max();
        for (std::size_t index = 1; index < simPoints.size(); ++index) {
            const auto &start = simPoints[index - 1];
            const auto &end = simPoints[index];
            const double segmentX = end.px - start.px;
            const double segmentY = end.py - start.py;
            const double segmentSquaredLength =
                segmentX * segmentX + segmentY * segmentY;
            if (segmentSquaredLength <= 0.0
                || segmentSquaredLength > 100.0 * 100.0) {
                continue;
            }
            const double projection = std::clamp(
                ((point.x - start.px) * segmentX
                 + (point.y - start.py) * segmentY)
                    / segmentSquaredLength,
                0.0,
                1.0);
            const double deltaX =
                point.x - (start.px + projection * segmentX);
            const double deltaY =
                point.y - (start.py + projection * segmentY);
            minimumSquaredDistance = std::min(
                minimumSquaredDistance,
                deltaX * deltaX + deltaY * deltaY);
        }
        if (minimumSquaredDistance <= maximumSquaredDistance) {
            filteredPoints.push_back(point);
        }
    }

    constexpr std::size_t MINIMUM_CENTERLINE_POINT_COUNT = 12U;
    centerline->centerlinePoints = std::move(filteredPoints);
    centerline->valid = centerline->valid
        && centerline->centerlinePoints.size()
            >= MINIMUM_CENTERLINE_POINT_COUNT;
    return centerline->valid;
}

} // namespace

CCameraWorkerAdapter::CCameraWorkerAdapter(QObject *parent)
    : QObject(parent)
{
}

CCameraWorkerAdapter::CCameraWorkerAdapter(
    std::unique_ptr<camera::ICameraDevice> device,
    int captureIntervalMs,
    QObject *parent)
    : QObject(parent)
    , m_device(std::move(device))
    , m_captureIntervalMs(std::max(1, captureIntervalMs))
{
}

void CCameraWorkerAdapter::start()
{
    if (!m_device) {
        emit statusChanged(QStringLiteral("Camera worker 已启动，等待外部发布图像帧"));
        return;
    }

    m_device->start();
    if (m_captureTimer == nullptr) {
        m_captureTimer = new QTimer(this);
        m_captureTimer->setTimerType(Qt::PreciseTimer);
        connect(
            m_captureTimer,
            &QTimer::timeout,
            this,
            &CCameraWorkerAdapter::captureLatestFrame);
    }
    m_captureTimer->start(m_captureIntervalMs);
    emit statusChanged(
        m_device->stateText().isEmpty()
            ? QStringLiteral("实时相机采集已启动")
            : m_device->stateText());
    captureLatestFrame();
}

void CCameraWorkerAdapter::stop()
{
    if (m_captureTimer != nullptr) {
        m_captureTimer->stop();
    }
    if (m_device) {
        m_device->stop();
        emit statusChanged(
            m_device->stateText().isEmpty()
                ? QStringLiteral("实时相机采集已停止")
                : m_device->stateText());
        return;
    }
    emit statusChanged(QStringLiteral("Camera worker 已停止"));
}

void CCameraWorkerAdapter::publishFrame(const camera::CameraFrame &frame)
{
    emit frameReady(frame);
}

void CCameraWorkerAdapter::captureLatestFrame()
{
    if (!m_device) {
        return;
    }
    camera::CameraFrame frame = m_device->latestFrame();
    if (frame.image.isNull() && frame.previewImage.isNull()) {
        return;
    }
    emit frameReady(frame);
}

void CCameraWorkerAdapter::setExposureTimeUs(int exposureTimeUs)
{
    if (exposureTimeUs <= 0) {
        emit statusChanged(QStringLiteral("相机曝光设置失败：曝光时间必须大于 0 us"));
        return;
    }
    if (!m_device) {
        emit statusChanged(QStringLiteral("相机曝光设置失败：未连接真实相机"));
        return;
    }

    QString errorMessage;
    if (!m_device->setExposureTimeUs(static_cast<double>(exposureTimeUs), &errorMessage)) {
        emit statusChanged(QStringLiteral("相机曝光设置失败：%1").arg(errorMessage));
        return;
    }

    emit statusChanged(QStringLiteral("相机曝光已设置为 %1 us").arg(exposureTimeUs));
}

CRobotCommWorkerAdapter::CRobotCommWorkerAdapter(QObject *parent)
    : CRobotCommWorkerAdapter(
          std::make_unique<robot_comm::CClientRobotCommandTransport>(),
          parent)
{
}

CRobotCommWorkerAdapter::CRobotCommWorkerAdapter(
    std::unique_ptr<robot_comm::IRobotCommandTransport> transport,
    QObject *parent)
    : QObject(parent),
      m_transport(std::move(transport))
{
    if (!m_transport) {
        m_transport =
            std::make_unique<robot_comm::CClientRobotCommandTransport>();
    }
    m_diagnostics.endpoint = std::string(robot_comm::config::ROBOT_IP)
        + ":" + std::to_string(robot_comm::config::ROBOT_PORT);
    m_diagnostics.laserEnabled = m_laserEnabled;
    m_diagnostics.filterEnabled = m_filterEnabled;
}

void CRobotCommWorkerAdapter::connectRobot()
{
    m_keepConnectionRequested = true;
    if (m_reconnectTimer != nullptr) {
        m_reconnectTimer->stop();
    }
    m_reconnectAttemptCount = 0;
    if (m_transport->isConnected()) {
        ensurePollTimer();
        if (!m_pollTimer->isActive()) {
            m_pollTimer->start();
        }
        emit statusChanged(QStringLiteral("RobotComm 已连接"));
        pollRobotState();
        return;
    }
    if (!m_transport->connect(
            robot_comm::config::ROBOT_IP,
            robot_comm::config::ROBOT_PORT)) {
        emit statusChanged(QStringLiteral("RobotComm 连接失败"));
        m_diagnostics.connected = false;
        m_diagnostics.validDReceived = false;
        m_diagnostics.lastTransactionSucceeded = false;
        ++m_diagnostics.consecutiveFailureCount;
        m_diagnostics.errorMessage = "connection failed";
        m_diagnostics.lastErrorText = m_diagnostics.errorMessage;
        m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
        emit diagnosticsReady(m_diagnostics);
        scheduleReconnectAfterTransportFailure();
        return;
    }
    ensurePollTimer();
    m_pollTimer->start();
    m_reconnectAttemptCount = 0;
    m_diagnostics.connected = true;
    m_diagnostics.validDReceived = false;
    m_diagnostics.lastTransactionSucceeded = false;
    m_diagnostics.lastErrorText.clear();
    m_diagnostics.errorMessage.clear();
    m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
    emit diagnosticsReady(m_diagnostics);
    emit statusChanged(QStringLiteral("RobotComm 已连接"));
    pollRobotState();
}

void CRobotCommWorkerAdapter::disconnectRobot()
{
    m_keepConnectionRequested = false;
    if (m_pollTimer != nullptr) {
        m_pollTimer->stop();
    }
    if (m_reconnectTimer != nullptr) {
        m_reconnectTimer->stop();
    }
    m_reconnectAttemptCount = 0;
    m_transport->disconnect();
    m_diagnostics.connected = false;
    m_diagnostics.validDReceived = false;
    m_diagnostics.lastTransactionSucceeded = false;
    m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
    emit diagnosticsReady(m_diagnostics);
    emit statusChanged(QStringLiteral("RobotComm 已断开"));
}

robot_comm::RobotProtocolContext CRobotCommWorkerAdapter::protocolContext() const
{
    robot_comm::RobotProtocolContext context;
    context.robotEnabled = 1;
    context.trackerEnabled = m_trackerEnabled;
    context.laserEnabled = m_laserEnabled;
    context.filterEnabled = m_filterEnabled;
    context.offlineSignal = m_offlineSignal;
    context.offlineStartCount = m_offlineStartCount;
    context.holdRequested = 0;
    context.firstDetectionSucceeded = m_firstDetectionSucceeded;
    return context;
}

void CRobotCommWorkerAdapter::ensurePollTimer()
{
    if (m_pollTimer != nullptr) {
        return;
    }
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(kRobotPollIntervalMs);
    connect(m_pollTimer, &QTimer::timeout,
            this, &CRobotCommWorkerAdapter::pollRobotState);
}

void CRobotCommWorkerAdapter::ensureReconnectTimer()
{
    if (m_reconnectTimer != nullptr) {
        return;
    }
    m_reconnectTimer = new QTimer(this);
    m_reconnectTimer->setSingleShot(true);
    connect(m_reconnectTimer, &QTimer::timeout,
            this, &CRobotCommWorkerAdapter::attemptReconnect);
}

void CRobotCommWorkerAdapter::pollRobotState()
{
    if (!m_transport->isConnected()) {
        stopPollingAfterTransportFailure();
        return;
    }
    robot_comm::RobotProtocolContext context = protocolContext();
    context.onlineEndFlag = 0;
    const std::string request = robot_comm::CRobotProtocolCodec::encodeStatusPoll(context);
    const auto receipt = m_transport->transact(
        robot_comm::RobotCommandType::StatusPoll, request, -1, &m_state);
    emit commandReceiptReady(receipt, m_state);
    recordDiagnostics(receipt);
    if (receipt.sent
        && receipt.responseReceived
        && receipt.error == robot_comm::RobotCommandError::None) {
        emit robotStateReady(m_state);
    }
}

void CRobotCommWorkerAdapter::updateState(const robot_comm::RobotState &state)
{
    m_state = state;
    emit robotStateReady(state);
}

bool CRobotCommWorkerAdapter::ensureOfflineLaserEnabled()
{
    if (m_laserEnabled == 0) {
        setLaserEnabled(true);
    } else {
        // 默认 ON 可能尚未下发，不能只凭本地开关跳过启动前的通信确认。
        if (!m_transport->isConnected()) {
            return false;
        }
        pollRobotState();
    }
    return m_transport->isConnected()
        && m_diagnostics.lastTransactionSucceeded
        && m_diagnostics.laserEnabled == 1;
}

bool CRobotCommWorkerAdapter::ensureOnlineCorrectionReady()
{
    if (m_state.offline == 4 || m_offlineSignal == 4) {
        emit statusChanged(QStringLiteral(
            "RobotComm 在线预检失败：机器人端仍有未完成的 O 示教点批次（offline=4）"));
        return false;
    }
    if (m_state.sStart != 0) {
        emit statusChanged(QStringLiteral(
            "RobotComm 在线预检失败：机器人程序正在运行，不能初始化 P001 就绪状态"));
        return false;
    }

    m_trackerEnabled = 1;
    m_laserEnabled = 1;
    m_firstDetectionSucceeded = 0;
    if (!m_transport->isConnected()) {
        return false;
    }

    emit statusChanged(QStringLiteral(
        "RobotComm 在线预检：发送 tracker=1、laser=1 的 N 并等待有效 D 回包"));
    pollRobotState();
    if (!m_transport->isConnected()
        || !m_diagnostics.lastTransactionSucceeded
        || m_state.sStart != 0 || m_state.offline == 4) {
        return false;
    }

    // 中断接近 P001 会跳过 JBI 的 ENDDET。只在下一次作业启动前请求
    // 清除残留 B005，普通 N 轮询不能反复清除本轮 O 写入的就绪标志。
    const std::string request =
        robot_comm::CRobotProtocolCodec::encodeOnlineCorrectionInitialization(
            protocolContext());
    emit statusChanged(QStringLiteral(
        "RobotComm 在线初始化：发送 N.init=1 请求清除上轮 P001 就绪标志"));
    const auto receipt = m_transport->transact(
        robot_comm::RobotCommandType::StatusPoll, request, -1, &m_state);
    emit commandReceiptReady(receipt, m_state);
    recordDiagnostics(receipt);
    const bool succeeded = receipt.sent && receipt.responseReceived
        && receipt.error == robot_comm::RobotCommandError::None;
    if (succeeded) {
        emit robotStateReady(m_state);
    }
    return succeeded && m_transport->isConnected()
        && m_state.sStart == 0 && m_state.offline != 4;
}

void CRobotCommWorkerAdapter::setLaserEnabled(bool enabled)
{
    m_laserEnabled = enabled ? 1 : 0;
    if (m_transport->isConnected()) {
        emit statusChanged(
            enabled
                ? QStringLiteral("RobotComm 激光指令已置为 ON，立即发送 N 确认")
                : QStringLiteral("RobotComm 激光指令已置为 OFF，立即发送 N 确认"));
        pollRobotState();
        if (!m_diagnostics.lastTransactionSucceeded) {
            emit statusChanged(QStringLiteral("RobotComm 激光指令 N 发送失败：%1")
                                   .arg(QString::fromStdString(m_diagnostics.lastErrorText)));
        }
        return;
    }

    m_diagnostics.laserEnabled = m_laserEnabled;
    m_diagnostics.filterEnabled = m_filterEnabled;
    m_diagnostics.state = m_state;
    m_diagnostics.connected = false;
    m_diagnostics.validDReceived = false;
    m_diagnostics.lastTransactionSucceeded = false;
    ++m_diagnostics.consecutiveFailureCount;
    m_diagnostics.lastErrorText = "robot is not connected";
    m_diagnostics.errorMessage = m_diagnostics.lastErrorText;
    m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
    emit diagnosticsReady(m_diagnostics);
    emit statusChanged(
        enabled
            ? QStringLiteral("RobotComm 激光指令已置为 ON，未连接时将等待后续 N/O/P 携带")
            : QStringLiteral("RobotComm 激光指令已置为 OFF，未连接时将等待后续 N/O/P 携带"));
    scheduleReconnectAfterTransportFailure();
}

void CRobotCommWorkerAdapter::setTrackingRequested(bool enabled)
{
    m_trackerEnabled = enabled ? 1 : 0;
    if (!enabled) {
        m_firstDetectionSucceeded = 0;
    }
}

void CRobotCommWorkerAdapter::setOfflineStartCount(const int count)
{
    m_offlineStartCount = count != 0 ? 1 : 0;
}

void CRobotCommWorkerAdapter::updateDetectionStatus(
    const RuntimeTrackingResult &result)
{
    if (result.valid) {
        m_firstDetectionSucceeded = 1;
    }
}

void CRobotCommWorkerAdapter::sendCorrectionCommand(
    const robot_comm::RobotCorrectionCommand &command)
{
    const bool payloadIsComplete =
        command.pointCount > 0
        && command.pointCount
            == static_cast<int>(command.teachPoints.size())
        && command.pointCount
            == static_cast<int>(command.stationAxes.size())
        && std::all_of(
            command.teachPoints.cbegin(),
            command.teachPoints.cend(),
            [](const robot_comm::TeachPointUpdateCommand &point) {
                return point.valid;
            })
        && std::all_of(
            command.stationAxes.cbegin(),
            command.stationAxes.cend(),
            [](const robot_comm::StationAxisUpdateCommand &axis) {
                return axis.valid;
            });
    if (!command.valid || !command.requestTeachPointUpdate
        || !payloadIsComplete
        || !m_transport->isConnected()) {
        emit statusChanged(QStringLiteral("RobotComm 示教点命令无效或未连接"));
        robot_comm::RobotCommandReceipt receipt;
        receipt.type = robot_comm::RobotCommandType::TeachPointUpdate;
        receipt.pointIndex = 0;
        receipt.error = m_transport->isConnected()
            ? robot_comm::RobotCommandError::InvalidResponse
            : robot_comm::RobotCommandError::NotConnected;
        receipt.message = "RobotComm teach point batch invalid or disconnected";
        emit commandReceiptReady(receipt, m_state);
        return;
    }

    robot_comm::RobotProtocolContext context = protocolContext();
    context.offlineSignal = command.offlineSignal;
    m_offlineSignal = command.offlineSignal;
    for (int index = 0; index < command.pointCount; ++index) {
        const std::string request = robot_comm::CRobotProtocolCodec::encodeTeachPointUpdate(
            context,
            command.pointCount,
            command.teachPoints[index],
            command.stationAxes[index]);
        auto receipt = m_transport->transact(
            robot_comm::RobotCommandType::TeachPointUpdate,
            request,
            index,
            &m_state);
        const bool transactionSucceeded = receipt.sent
            && receipt.responseReceived
            && receipt.error == robot_comm::RobotCommandError::None;
        const bool finalPoint = index + 1 == command.pointCount;
        const int expectedOffline = finalPoint ? 5 : 4;
        if (transactionSucceeded && m_state.offline != expectedOffline) {
            receipt.completed = false;
            receipt.error = robot_comm::RobotCommandError::InvalidResponse;
            receipt.message = QStringLiteral(
                "O 示教点批次失步：第 %1/%2 点期望 D.offline=%3，实际为 %4；已停止发送后续点。")
                                  .arg(index + 1)
                                  .arg(command.pointCount)
                                  .arg(expectedOffline)
                                  .arg(m_state.offline)
                                  .toStdString();
        }
        emit commandReceiptReady(receipt, m_state);
        recordDiagnostics(receipt);
        if (receipt.sent
            && receipt.responseReceived
            && receipt.error == robot_comm::RobotCommandError::None) {
            emit robotStateReady(m_state);
        }
        if (!receipt.sent
            || !receipt.responseReceived
            || receipt.error != robot_comm::RobotCommandError::None) {
            break;
        }
    }
    if (m_state.offline == 5) {
        m_offlineSignal = 0;
    }
}

void CRobotCommWorkerAdapter::sendRealtimeCorrectionCommand(
    const robot_comm::RobotRealtimeCorrectionCommand &command)
{
    if (!command.valid) {
        return;
    }
    m_latestRealtimeCommand = command;
    if (m_realtimeFlushScheduled) {
        return;
    }
    m_realtimeFlushScheduled = true;
    QTimer::singleShot(
        0,
        this,
        [this]() {
            flushRealtimeCorrectionCommand();
        });
}

void CRobotCommWorkerAdapter::flushRealtimeCorrectionCommand()
{
    m_realtimeFlushScheduled = false;
    const robot_comm::RobotRealtimeCorrectionCommand command =
        m_latestRealtimeCommand;
    m_latestRealtimeCommand = {};
    if (!command.valid || !m_transport->isConnected()) {
        return;
    }

    robot_comm::RobotProtocolContext context = protocolContext();
    context.offlineSignal = 0;
    context.onlineEndFlag = command.onlineEndFlag;
    const std::string request = robot_comm::CRobotProtocolCodec::encodeRealtimeCorrection(
        context, command);
    const auto receipt = m_transport->transact(
        robot_comm::RobotCommandType::RealtimeCorrection,
        request,
        -1,
        &m_state);
    emit commandReceiptReady(receipt, m_state);
    recordDiagnostics(receipt);
    if (receipt.sent
        && receipt.responseReceived
        && receipt.error == robot_comm::RobotCommandError::None) {
        emit robotStateReady(m_state);
    }
}

void CRobotCommWorkerAdapter::sendCalibrationCommand(
    const robot_comm::RobotCalibrationCommand &command)
{
    if (!command.valid
        || !command.requestNextCalibrationPosition
        || command.calibResetSignal != 1
        || !m_transport->isConnected()) {
        emit statusChanged(QStringLiteral("RobotComm 标定 C 命令无效或未连接"));
        return;
    }

    const std::string request =
        robot_comm::CRobotProtocolCodec::encodeCalibrationAdvance(protocolContext());
    const auto receipt = m_transport->transact(
        robot_comm::RobotCommandType::CalibrationAdvance,
        request,
        command.capturedSampleCount,
        &m_state);
    emit commandReceiptReady(receipt, m_state);
    recordDiagnostics(receipt);
    if (receipt.sent
        && receipt.responseReceived
        && receipt.error == robot_comm::RobotCommandError::None) {
        emit robotStateReady(m_state);
    }
}

void CRobotCommWorkerAdapter::sendJobStartCommand(
    const robot_comm::RobotJobStartCommand &command)
{
    if (!command.valid || !command.requestStart || !m_transport->isConnected()) {
        emit statusChanged(QStringLiteral("RobotComm 作业 J 命令无效或未连接"));
        return;
    }
    if (m_state.sStart != 0) {
        emit statusChanged(QStringLiteral("RobotComm 拒绝 J 命令：机器人作业正在运行"));
        return;
    }

    const std::string request =
        robot_comm::CRobotProtocolCodec::encodeJobStart(
            protocolContext(), command.jobName);
    if (request.empty()) {
        emit statusChanged(QStringLiteral("RobotComm 拒绝 J 命令：作业名为空或包含空白/逗号"));
        return;
    }
    const auto receipt = m_transport->transact(
        robot_comm::RobotCommandType::JobStart,
        request,
        -1,
        &m_state);
    emit commandReceiptReady(receipt, m_state);
    recordDiagnostics(receipt);
    if (receipt.sent
        && receipt.responseReceived
        && receipt.error == robot_comm::RobotCommandError::None) {
        emit robotStateReady(m_state);
    }
}

bool CRobotCommWorkerAdapter::shouldReconnectForFailure(
    const robot_comm::RobotCommandReceipt &receipt) const
{
    if (receipt.error == robot_comm::RobotCommandError::None) {
        return false;
    }
    if (!m_transport->isConnected()
        || receipt.error == robot_comm::RobotCommandError::NotConnected
        || receipt.error == robot_comm::RobotCommandError::ConnectFailed) {
        return true;
    }
    if (receipt.error == robot_comm::RobotCommandError::InvalidResponse) {
        return false;
    }
    return m_diagnostics.consecutiveFailureCount
        >= kRobotReconnectFailureThreshold;
}

void CRobotCommWorkerAdapter::stopPollingAfterTransportFailure(bool emitSnapshot)
{
    if (m_pollTimer != nullptr) {
        m_pollTimer->stop();
    }
    m_diagnostics.connected = false;
    m_diagnostics.validDReceived = false;
    m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
    scheduleReconnectAfterTransportFailure();
    if (emitSnapshot) {
        emit diagnosticsReady(m_diagnostics);
    }
}

void CRobotCommWorkerAdapter::scheduleReconnectAfterTransportFailure()
{
    if (!m_keepConnectionRequested) {
        return;
    }
    ensureReconnectTimer();
    if (m_reconnectTimer->isActive()) {
        return;
    }
    const int cappedAttempt = std::min(m_reconnectAttemptCount, 6);
    const int delayMs = std::min(
        kRobotReconnectMaxDelayMs,
        kRobotReconnectBaseDelayMs << cappedAttempt);
    m_reconnectTimer->start(delayMs);
}

void CRobotCommWorkerAdapter::attemptReconnect()
{
    if (!m_keepConnectionRequested) {
        return;
    }
    if (m_transport->isConnected()) {
        ensurePollTimer();
        m_pollTimer->start();
        m_reconnectAttemptCount = 0;
        emit statusChanged(QStringLiteral("RobotComm 连接已恢复，重新发送 N 轮询"));
        pollRobotState();
        return;
    }

    ++m_reconnectAttemptCount;
    emit statusChanged(QStringLiteral("RobotComm 自动重连第 %1 次").arg(m_reconnectAttemptCount));
    if (!m_transport->connect(
            robot_comm::config::ROBOT_IP,
            robot_comm::config::ROBOT_PORT)) {
        m_diagnostics.connected = false;
        m_diagnostics.validDReceived = false;
        m_diagnostics.lastTransactionSucceeded = false;
        ++m_diagnostics.consecutiveFailureCount;
        m_diagnostics.lastErrorText = "auto reconnect failed";
        m_diagnostics.errorMessage = m_diagnostics.lastErrorText;
        m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
        emit diagnosticsReady(m_diagnostics);
        scheduleReconnectAfterTransportFailure();
        return;
    }

    ensurePollTimer();
    m_pollTimer->start();
    m_reconnectAttemptCount = 0;
    m_diagnostics.connected = true;
    m_diagnostics.validDReceived = false;
    m_diagnostics.lastTransactionSucceeded = false;
    m_diagnostics.lastErrorText.clear();
    m_diagnostics.errorMessage.clear();
    m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
    emit diagnosticsReady(m_diagnostics);
    emit statusChanged(QStringLiteral("RobotComm 自动重连成功，重新发送 N 轮询"));
    pollRobotState();
}

void CRobotCommWorkerAdapter::recordDiagnostics(
    const robot_comm::RobotCommandReceipt &receipt)
{
    ++m_diagnostics.transactionCount;
    const bool transactionSucceeded =
        receipt.sent
        && receipt.responseReceived
        && receipt.error == robot_comm::RobotCommandError::None;
    m_diagnostics.connected = m_transport->isConnected();
    if (transactionSucceeded) {
        m_diagnostics.state = m_state;
        m_diagnostics.validDReceived = true;
        m_diagnostics.consecutiveFailureCount = 0;
    } else {
        ++m_diagnostics.consecutiveFailureCount;
    }
    m_diagnostics.lastTransactionSucceeded = transactionSucceeded;
    m_diagnostics.laserEnabled = m_laserEnabled;
    m_diagnostics.filterEnabled = m_filterEnabled;
    m_diagnostics.lastCommandType = receipt.type;
    m_diagnostics.timestampMs = QDateTime::currentMSecsSinceEpoch();
    m_diagnostics.latestResponse = receipt.response;
    m_diagnostics.lastRequest = receipt.request;
    m_diagnostics.lastResponse = receipt.response;
    m_diagnostics.lastErrorText =
        transactionSucceeded
        ? std::string()
        : (receipt.message.empty() ? std::string("robot transaction failed") : receipt.message);
    m_diagnostics.errorMessage =
        transactionSucceeded
        ? std::string()
        : m_diagnostics.lastErrorText;
    if (!transactionSucceeded && shouldReconnectForFailure(receipt)) {
        stopPollingAfterTransportFailure(false);
        m_diagnostics.connected = false;
        m_diagnostics.validDReceived = false;
    }
    switch (receipt.type) {
    case robot_comm::RobotCommandType::StatusPoll:
        m_diagnostics.latestStatusRequest = receipt.request;
        break;
    case robot_comm::RobotCommandType::TeachPointUpdate:
        m_diagnostics.latestTeachPointRequest = receipt.request;
        break;
    case robot_comm::RobotCommandType::RealtimeCorrection:
        m_diagnostics.latestRealtimeRequest = receipt.request;
        break;
    case robot_comm::RobotCommandType::CalibrationAdvance:
        m_diagnostics.latestCalibrationRequest = receipt.request;
        break;
    case robot_comm::RobotCommandType::JobStart:
        m_diagnostics.latestJobRequest = receipt.request;
        break;
    }
    emit diagnosticsReady(m_diagnostics);
}

CVisionTrackingWorkerAdapter::CVisionTrackingWorkerAdapter(QObject *parent)
    : QObject(parent)
{
}

void CVisionTrackingWorkerAdapter::setSimulatedCallbacks(
    const SimulatedVisionCallbacks &callbacks)
{
    m_callbacks = callbacks;
}

void CVisionTrackingWorkerAdapter::configure(
    const RuntimeTrackingOptions &options,
    const vision::WeldProjectionContext &projectionContext)
{
    m_diagnosticFrameCount = 0;
    m_diagnosticImageCount = 0;
    m_options = options;
    m_options.method = RuntimeTrackingMethod::Kcf;
    m_projectionContext = projectionContext;
    m_detector.reset();
    emit statusChanged(QStringLiteral("vision tracking configured"));
}

void CVisionTrackingWorkerAdapter::updateOnlineFusionOptions(
    const vision::KcfFusionOptions &options)
{
    m_options.fusion = options;
}

void CVisionTrackingWorkerAdapter::updateRobotState(const robot_comm::RobotState &state)
{
    m_robotState = state;
}

void CVisionTrackingWorkerAdapter::updateDigitalTwinVisionObservation(
    const digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    if (!observation.valid) {
        // SWv1 的虚拟传感器会保留上一帧成功生成的激光线。单次 OCCT 求交
        // 失败不能清空该参考，否则检测窗口内会退化成无先验的整图识别。
        return;
    }

    m_latestDigitalTwinObservation = observation;
}

void CVisionTrackingWorkerAdapter::startTracking()
{
    m_trackingActive = true;
    // 全局定位会在机器人启动门禁期间预热 OCCT 模型和虚拟激光先验；启动
    // 图像跟踪时必须保留这份先验。其他跟踪模式仍从空观测开始。
    if (!m_options.globalLocateLineExtraction) {
        m_latestDigitalTwinObservation.reset();
    }
    emit statusChanged(QStringLiteral("vision tracking started"));
}

void CVisionTrackingWorkerAdapter::stopTracking()
{
    m_trackingActive = false;
    m_currentLocalizationSegmentIndex = -1;
    m_detector.reset();
    m_latestDigitalTwinObservation.reset();
    m_requireSynchronizedObservation = false;
    m_activeCorrelation = {};
    emit statusChanged(QStringLiteral("vision tracking stopped"));
}

void CVisionTrackingWorkerAdapter::submitFrame(const camera::CameraFrame &frame)
{
    if (!m_trackingActive) {
        return;
    }

    control::TrackingTriggerInput triggerInput;
    triggerInput.testStart = m_options.testStart;
    triggerInput.robotState = m_robotState;
    triggerInput.virtualStatus = m_options.virtualStatus;
    triggerInput.requireDetectionWindow = m_options.requireDetectionWindow;
    const control::TrackingTriggerDecision decision =
        control::CTrackingTriggerPolicy::evaluate(triggerInput);
    if (!decision.shouldStartTracking) {
        RuntimeTrackingResult result;
        result.frameId = frame.frameId;
        result.posflag = m_robotState.posflag;
        result.traflag = m_robotState.traflag;
        result.robotJobLine = m_robotState.usJobLine;
        result.localizationSegmentIndex = m_currentLocalizationSegmentIndex;
        result.triggerAccepted = false;
        result.correlation = m_activeCorrelation;
        result.message = QString::fromStdString(decision.reasonKey + ": " + decision.message);
        emit trackingResultReady(result);
        return;
    }

    RuntimeTrackingResult result;
    if (m_options.detType != 2) {
        result.frameId = frame.frameId;
        result.triggerAccepted = true;
        result.valid = false;
        result.message = QStringLiteral("unsupported det_type=%1").arg(m_options.detType);
    } else {
        if (m_options.globalLocateLineExtraction) {
            result = processGlobalLocateLineExtraction(frame);
        } else {
            result = processDetType2(frame);
        }
        result.triggerAccepted = true;
    }
    result.posflag = m_robotState.posflag;
    result.traflag = m_robotState.traflag;
    result.robotJobLine = m_robotState.usJobLine;
    result.localizationSegmentIndex = m_currentLocalizationSegmentIndex;
    result.correlation = m_activeCorrelation;
    if (m_activeCorrelation.purpose == CorrectionPurpose::OnlineCorrection
        && m_options.visionDiagnostics.enabled) {
        if (!result.diagnostic.has_value()) {
            result.diagnostic.emplace();
            result.diagnostic->robotState = m_robotState;
            result.diagnostic->cameraTimestampMs = frame.timestamp.isValid()
                ? frame.timestamp.toMSecsSinceEpoch() : -1;
        }
        result.diagnostic->processingFinishedMs = QDateTime::currentMSecsSinceEpoch();
    }
    emit trackingResultReady(result);
}

void CVisionTrackingWorkerAdapter::submitFrameWithRobotState(
    const camera::CameraFrame &frame,
    const robot_comm::RobotState &state,
    int localizationSegmentIndex)
{
    submitFrameWithRobotStateAndCorrelation(
        frame,
        state,
        localizationSegmentIndex,
        RuntimeVisionCorrelation{});
}

void CVisionTrackingWorkerAdapter::submitFrameWithRobotStateAndCorrelation(
    const camera::CameraFrame &frame,
    const robot_comm::RobotState &state,
    int localizationSegmentIndex,
    const RuntimeVisionCorrelation &correlation)
{
    // frame、state 和定位面序号由 RuntimeCoordinator 在相机帧到达时按值绑定。
    // correlation 同样只在本次调用期间有效，不能被后续会话覆盖。
    m_robotState = state;
    m_currentLocalizationSegmentIndex = localizationSegmentIndex;
    m_activeCorrelation = correlation;
    submitFrame(frame);
    m_activeCorrelation = {};
}

void CVisionTrackingWorkerAdapter::submitFrameWithRobotStateAndObservation(
    const camera::CameraFrame &frame,
    const robot_comm::RobotState &state,
    int localizationSegmentIndex,
    const digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    submitFrameWithRobotStateObservationAndCorrelation(
        frame,
        state,
        localizationSegmentIndex,
        observation,
        RuntimeVisionCorrelation{});
}

void CVisionTrackingWorkerAdapter::submitFrameWithRobotStateObservationAndCorrelation(
    const camera::CameraFrame &frame,
    const robot_comm::RobotState &state,
    int localizationSegmentIndex,
    const digital_twin::DigitalTwinVisionLaserObservation &observation,
    const RuntimeVisionCorrelation &correlation)
{
    m_callbacks.beforeSynchronizedFrame(frame.frameId);
    // RobotSim 观测与相机帧在 coordinator 中以同一份 RobotState
    // 绑定。无效响应必须清除旧先验，不允许用上一位姿误导本帧。
    m_latestDigitalTwinObservation = observation;
    m_requireSynchronizedObservation = true;
    submitFrameWithRobotStateAndCorrelation(
        frame,
        state,
        localizationSegmentIndex,
        correlation);
    m_requireSynchronizedObservation = false;
}

RuntimeTrackingResult CVisionTrackingWorkerAdapter::processDetType2(
    const camera::CameraFrame &frame)
{
    RuntimeTrackingResult result;
    result.frameId = frame.frameId;
    result.posflag = m_robotState.posflag;
    const bool diagnosticsEnabled = m_options.visionDiagnostics.enabled
        && m_activeCorrelation.purpose == CorrectionPurpose::OnlineCorrection;
    if (diagnosticsEnabled) {
        result.diagnostic.emplace();
        result.diagnostic->robotState = m_robotState;
        result.diagnostic->cameraTimestampMs = frame.timestamp.isValid()
            ? frame.timestamp.toMSecsSinceEpoch() : -1;
        result.diagnostic->processingStartedMs = QDateTime::currentMSecsSinceEpoch();
    }

    const QImage image = frame.image.isNull() ? frame.previewImage : frame.image;
    const cv::Mat gray8 = qImageToGray8(image);
    if (gray8.empty()) {
        result.message = QStringLiteral("invalid frame image");
        return result;
    }

    digital_twin::DigitalTwinVisionLaserObservation observation;
    if (!makeObservation(frame.frameId, &observation)) {
        result.message = QStringLiteral("simulated observation unavailable");
        return result;
    }
    result.simulatedSeamIndex = observation.seamIndex;
    if (observation.scanLineChangePoint_mm.has_value()) {
        const auto &point = observation.scanLineChangePoint_mm.value();
        result.scanLineChangePoint = control::OnlineCorrectionPoint{
            point.x_mm,
            point.y_mm,
            point.z_mm,
            true};
    }

    vision::WeldProjectionContext projectionContext = m_projectionContext;
    if (m_options.sensorCalibration.valid
        || m_options.positionerCalibration.valid) {
        const auto contextResult = vision::CWeldProjectionContextBuilder::build(
            m_options.sensorCalibration,
            m_options.positionerCalibration,
            m_robotState);
        if (!contextResult.valid) {
            result.message = QStringLiteral("projection context rejected: %1")
                                 .arg(QString::fromStdString(contextResult.message));
            return result;
        }
        projectionContext = contextResult.context;
    }

    vision::LegacyOnlineKcfOptions detectorOptions;
    detectorOptions.enableGapDetection = m_options.enableGapDetection;
    if (m_activeCorrelation.purpose == CorrectionPurpose::OnlineCorrection) {
        detectorOptions.fusion = m_options.fusion;
    }
    const auto detected = m_detector.process(
        gray8, observation, projectionContext, detectorOptions);
    if (diagnosticsEnabled) {
        result.diagnostic->algorithm = detected.diagnostic;
        const auto &policy = m_options.visionDiagnostics;
        const bool sampleImage = policy.imagesEnabled && policy.maxImageBytes > 0
            && policy.imageSampleEveryN > 0 && policy.maxImagePairs > 0
            && m_diagnosticImageCount < static_cast<quint64>(policy.maxImagePairs)
            && (m_diagnosticFrameCount % static_cast<quint64>(policy.imageSampleEveryN)) == 0;
        ++m_diagnosticFrameCount;
        if (sampleImage && !detected.processedImage.empty()) {
            // Mat 数据属于检测器输出；copy 后才可跨线程交给后台写入器。
            result.diagnostic->sourceImage = image;
            result.diagnostic->enhancedImage = QImage(
                detected.processedImage.data, detected.processedImage.cols,
                detected.processedImage.rows, static_cast<int>(detected.processedImage.step),
                QImage::Format_Grayscale8).copy();
            ++m_diagnosticImageCount;
        }
    }
    result.imageXPixel = detected.weldPointPixel.x;
    result.imageYPixel = detected.weldPointPixel.y;
    result.sample.point = control::OnlineCorrectionPoint{
        detected.stationPoint.xMm,
        detected.stationPoint.yMm,
        detected.stationPoint.zMm,
        detected.stationPoint.valid};
    result.sample.simulatedSeamIndex = result.simulatedSeamIndex;
    if (result.scanLineChangePoint.has_value()) {
        result.sample.scanLineChangePoint = result.scanLineChangePoint;
    }
    if (m_options.enableGapDetection) {
        result.sample.gapWidthMm = static_cast<float>(detected.gapWidthMm);
    }
    result.valid = detected.valid;
    // 独立验收只能使用融合前的真实 KCF 观测；诊断归档开关不影响安全数据。
    const auto &rawDiagnostic = detected.diagnostic;
    const auto &rawPixel = rawDiagnostic.beforeFusionPoint;
    const auto &rawTracker = rawDiagnostic.tracker;
    if (detected.valid && rawDiagnostic.attempted && rawTracker.hasResponse
        && rawTracker.completed && !rawTracker.correlationJumpRejected
        && !rawTracker.fusionJumpRejected
        && std::isfinite(rawPixel.x) && std::isfinite(rawPixel.y)
        && rawPixel.x >= 0.0 && rawPixel.x < gray8.cols
        && rawPixel.y >= 0.0 && rawPixel.y < gray8.rows) {
        const auto independentProjection = vision::CWeldImageToSpaceProjector::projectCvPoint(
            rawPixel, projectionContext, vision::WeldProjectionTargetFrame::PositionerTcp);
        const auto &independent = independentProjection.spacePoint;
        if (independentProjection.valid && independent.valid
            && std::isfinite(independent.xMm) && std::isfinite(independent.yMm)
            && std::isfinite(independent.zMm)) {
            result.independentWeldPointInParent_mm = control::OnlineCorrectionPoint{
                independent.xMm, independent.yMm, independent.zMm, true};
        }
    }
    if (detected.valid) {
        // 复用检测当帧的完整投影链，禁止使用 UI 收到结果时的最新机器人位姿。
        const auto robotProjection = vision::CWeldImageToSpaceProjector::projectCvPoint(
            detected.weldPointPixel, projectionContext,
            vision::WeldProjectionTargetFrame::RobotBase);
        const auto& point = robotProjection.spacePoint;
        if (robotProjection.valid && point.valid) {
            const Eigen::Vector3d robotPoint(point.xMm, point.yMm, point.zMm);
            result.detectedRobotBasePoint = control::OnlineCorrectionPoint{
                point.xMm, point.yMm, point.zMm, robotPoint.allFinite()};
            if (m_options.positionerCalibration.valid) {
                const Eigen::Vector3d stationPoint =
                    m_options.positionerCalibration.stationBaseInRobotBase * robotPoint;
                result.detectedStationBasePoint = control::OnlineCorrectionPoint{
                    stationPoint.x(), stationPoint.y(), stationPoint.z(),
                    stationPoint.allFinite()};
            }
        }
    }
    const bool correctionTracking =
        m_activeCorrelation.purpose == CorrectionPurpose::OnlineCorrection
        || m_activeCorrelation.purpose == CorrectionPurpose::OfflineCorrection;
    if (!correctionTracking) {
        result.overlayImage = makeTrackingOverlay(
            detected.processedImage,
            detected.weldPointPixel,
            detected.valid);
    }
    result.message = observation.message.isEmpty()
        ? QString::fromStdString(detected.message)
        : QStringLiteral("%1; %2")
              .arg(QString::fromStdString(detected.message), observation.message);
    return result;
}

RuntimeTrackingResult CVisionTrackingWorkerAdapter::processGlobalLocateLineExtraction(
    const camera::CameraFrame &frame)
{
    RuntimeTrackingResult result;
    result.frameId = frame.frameId;
    result.posflag = m_robotState.posflag;

    const QImage image = frame.image.isNull() ? frame.previewImage : frame.image;
    const cv::Mat gray8 = qImageToGray8(image);
    if (gray8.empty()) {
        result.message = QStringLiteral("invalid frame image");
        return result;
    }

    digital_twin::DigitalTwinVisionLaserObservation observation;
    const bool hasVirtualPrior =
        makeObservation(frame.frameId, &observation)
        && observation.laserLineImagePoints_px.size() >= 2U
        && (observation.positioningSegmentIndex < 0
            || observation.positioningSegmentIndex
                == m_currentLocalizationSegmentIndex);
    if (!hasVirtualPrior) {
        result.message = observation.valid
            ? QStringLiteral(
                  "SWv1 virtual laser prior segment mismatch: prior=%1, frame=%2")
                  .arg(observation.positioningSegmentIndex)
                  .arg(m_currentLocalizationSegmentIndex)
            : QStringLiteral(
                  "SWv1-compatible virtual laser prior is required; real line extraction skipped");
        return result;
    }
    result.simulatedSeamIndex = observation.seamIndex;

    vision::WeldProjectionContext projectionContext = m_projectionContext;
    if (m_options.sensorCalibration.valid
        || m_options.positionerCalibration.valid) {
        const auto contextResult = vision::CWeldProjectionContextBuilder::build(
            m_options.sensorCalibration,
            m_options.positionerCalibration,
            m_robotState);
        if (!contextResult.valid) {
            result.message = QStringLiteral("projection context rejected: %1")
                                 .arg(QString::fromStdString(contextResult.message));
            return result;
        }
        projectionContext = contextResult.context;
    }

    // 1. 与 SWv1 一致，使用当前来源的仿真激光线确定真实线搜索区域。
    std::vector<vision::MyLaserImagePoint> simPoints;
    simPoints.reserve(observation.laserLineImagePoints_px.size());
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    for (const auto &point : observation.laserLineImagePoints_px) {
        simPoints.push_back({point.x_px, point.y_px});
        minX = std::min(minX, point.x_px);
        maxX = std::max(maxX, point.x_px);
        minY = std::min(minY, point.y_px);
        maxY = std::max(maxY, point.y_px);
    }
    // SWv1 面扫描路径固定向虚拟线包围盒外扩 100 pixel。
    constexpr int kRoiExpandPixel = 100;
    cv::Rect roi(
        static_cast<int>(std::floor(minX)) - kRoiExpandPixel,
        static_cast<int>(std::floor(minY)) - kRoiExpandPixel,
        static_cast<int>(std::ceil(maxX - minX)) + 2 * kRoiExpandPixel,
        static_cast<int>(std::ceil(maxY - minY)) + 2 * kRoiExpandPixel);
    roi &= cv::Rect(0, 0, gray8.cols, gray8.rows);
    if (roi.empty()) {
        result.message = QStringLiteral("simulated laser ROI is empty");
        return result;
    }

    // 2. 仿真激光线引导的 Gabor 方向性滤波（对应 SWv1 GaborFilter）。
    const auto &gaborReferencePoint = observation.hasWeldPointImage_px
        ? observation.weldPointImage_px
        : observation.laserLineImagePoints_px[
              observation.laserLineImagePoints_px.size() / 2U];
    // RobotSim/OCCT 未返回焊缝点时，使用虚拟线中点提供 Gabor 的线上方向参考。
    // 该点不作为真实焊缝点，最终定位仍使用提取出的整条真实中心线三维点。
    const vision::MyLaserImagePoint weldSimPoint{
        gaborReferencePoint.x_px,
        gaborReferencePoint.y_px};
    vision::VisionPreprocessStatus gaborStatus =
        vision::VisionPreprocessStatus::Succeeded;
    cv::Mat gaborImg = vision::CImageDenoise::gaborDenoise(
        gray8, simPoints, weldSimPoint, &gaborStatus);
    const bool usedRawImageFallback = gaborImg.empty();
    const cv::Mat& primaryStripeImage = usedRawImageFallback ? gray8 : gaborImg;
    // RobotSim/OCCT 已给出当前位姿下的像素线。Steger 在原图上计算以避免
    // 掩膜边界产生假响应，随后只保留仿真线 60 pixel 走廊内的亚像素点。
    constexpr int kVirtualLineCorridorRadiusPixel = 80;

    // 3. Steger 亚像素中心线提取整条真实激光线（对应 SWv1 getCurrentCenterLine）。
    vision::StripeDetectionOptions stripeOptions;
    stripeOptions.useRoi = true;
    stripeOptions.roi = roi;
    vision::StripeDetectionState stripeState;
    vision::StripeCenterlineResult centerline =
        vision::CStegerStripeDetector::detect(
            primaryStripeImage, stripeOptions, &stripeState);
    constrainCenterlineToVirtualLaserCorridor(
        &centerline,
        simPoints,
        kVirtualLineCorridorRadiusPixel);
    QString extractionMode;
    extractionMode = usedRawImageFallback
        ? QStringLiteral("raw-prior-filtered")
        : QStringLiteral("gabor-prior-filtered");
    if (!centerline.valid) {
        // 现场线激光曝光随材质变化明显；默认 180 灰度失败后，在同一仿真 ROI
        // 内以测试覆盖的 80 灰度阈值重试，不扩大到整图以限制误检。
        stripeOptions.thresholdGrayValue = 80;
        centerline = vision::CStegerStripeDetector::detect(
            primaryStripeImage, stripeOptions, &stripeState);
        constrainCenterlineToVirtualLaserCorridor(
            &centerline,
            simPoints,
            kVirtualLineCorridorRadiusPixel);
        extractionMode += QStringLiteral("-threshold80");
    }
    if (!centerline.valid && !usedRawImageFallback) {
        centerline = vision::CStegerStripeDetector::detect(
            gray8, stripeOptions, &stripeState);
        constrainCenterlineToVirtualLaserCorridor(
            &centerline,
            simPoints,
            kVirtualLineCorridorRadiusPixel);
        extractionMode = QStringLiteral("raw-prior-filtered-threshold80");
    }
    if (!centerline.valid || centerline.centerlinePoints.empty()) {
        result.message = QStringLiteral(
            "steger centerline extraction failed: mode=%1, roi=(%2,%3,%4,%5), threshold=%6, candidates=%7, detail=%8")
                             .arg(extractionMode)
                             .arg(roi.x)
                             .arg(roi.y)
                             .arg(roi.width)
                             .arg(roi.height)
                             .arg(stripeOptions.thresholdGrayValue)
                             .arg(static_cast<int>(centerline.centerlinePoints.size()))
                             .arg(QString::fromStdString(centerline.message));
        return result;
    }

    // 4. 逐像素投影到变位机 TCP 系，得到整条真实激光线 3D 点。
    std::vector<control::OnlineCorrectionPoint> linePoints;
    linePoints.reserve(centerline.centerlinePoints.size());
    QString firstProjectionFailure;
    // SWv1 updateRealPointCloud(curPose, 5) 每隔 5 个中心线像素重建一个点。
    // 这一步先抑制同帧高度相关的密集点，再由 ICP 前的 FPS/SOR 做空间清理。
    constexpr std::size_t kSwv1GlobalLineSampleStep = 5U;
    for (std::size_t pointIndex = 0;
         pointIndex < centerline.centerlinePoints.size();
         pointIndex += kSwv1GlobalLineSampleStep) {
        const cv::Point2d& pixel = centerline.centerlinePoints[pointIndex];
        const auto projection = vision::CWeldImageToSpaceProjector::projectCvPoint(
            pixel,
            projectionContext,
            vision::WeldProjectionTargetFrame::PositionerTcp);
        if (projection.valid && projection.spacePoint.valid) {
            linePoints.push_back({
                projection.spacePoint.xMm,
                projection.spacePoint.yMm,
                projection.spacePoint.zMm,
                true});
        } else if (firstProjectionFailure.isEmpty()) {
            firstProjectionFailure = QString::fromStdString(projection.message);
        }
    }
    if (linePoints.empty()) {
        result.message = QStringLiteral(
            "projection failed for all centerline points: pixels=%1, mode=%2, firstError=%3")
                             .arg(static_cast<int>(centerline.centerlinePoints.size()))
                             .arg(extractionMode)
                             .arg(firstProjectionFailure);
        return result;
    }

    result.realLaserLineImagePoints_px.reserve(
        centerline.centerlinePoints.size());
    for (const cv::Point2d& point : centerline.centerlinePoints) {
        result.realLaserLineImagePoints_px.push_back({ point.x, point.y });
    }
    result.linePoints = std::move(linePoints);
    result.imageXPixel = centerline.centerlinePoints.front().x;
    result.imageYPixel = centerline.centerlinePoints.front().y;
    result.sample.point = result.linePoints.front();
    result.valid = true;
    result.message = QStringLiteral(
        "global locate line extraction: pixels=%1, points3d=%2, mode=%3, posflag=%4")
                         .arg(static_cast<int>(centerline.centerlinePoints.size()))
                         .arg(static_cast<int>(result.linePoints.size()))
                         .arg(extractionMode)
                         .arg(result.posflag);
    return result;
}

bool CVisionTrackingWorkerAdapter::makeObservation(
    qint64 frameId,
    digital_twin::DigitalTwinVisionLaserObservation *observation)
{
    if (observation == nullptr) {
        return false;
    }

    if (m_requireSynchronizedObservation) {
        if (m_latestDigitalTwinObservation.has_value()) {
            *observation = *m_latestDigitalTwinObservation;
        }
        return m_latestDigitalTwinObservation.has_value()
            && m_latestDigitalTwinObservation->valid
            && m_latestDigitalTwinObservation->frameId == frameId;
    }

    if (m_latestDigitalTwinObservation.has_value()
        && m_latestDigitalTwinObservation->valid) {
        *observation = *m_latestDigitalTwinObservation;
        if (observation->frameId == frameId) {
            return true;
        }
        const QString fallbackMessage = QStringLiteral(
            "DigitalTwin observation frameId=%1 used for camera frameId=%2")
                                            .arg(observation->frameId)
                                            .arg(frameId);
        observation->message = observation->message.isEmpty()
            ? fallbackMessage
            : QStringLiteral("%1; %2").arg(observation->message, fallbackMessage);
        return true;
    }

    m_callbacks.toNextWeldSeam();
    if (!m_callbacks.generate_simu_laser1()) {
        return false;
    }
    digital_twin::DigitalTwinImagePoint2D_px point;
    if (!m_callbacks.getSeamPoint2d(&point)) {
        return false;
    }
    std::vector<digital_twin::DigitalTwinImagePoint2D_px> line =
        m_callbacks.getCurrentVirtualLine();
    if (line.size() < 2U) {
        return false;
    }
    observation->valid = true;
    observation->frameId = frameId;
    observation->laserLineImagePoints_px = std::move(line);
    observation->hasWeldPointImage_px = true;
    observation->weldPointImage_px = point;
    observation->seamIndex = m_callbacks.getSeamIndex();
    observation->scanLineChangePoint_mm = m_callbacks.getScanLineChangePoint();
    observation->message = QStringLiteral("legacy simulated callbacks observation");
    return true;
}

CGlobalLocatePriorWorkerAdapter::CGlobalLocatePriorWorkerAdapter(QObject *parent)
    : QObject(parent)
{
}

CGlobalLocatePriorWorkerAdapter::CGlobalLocatePriorWorkerAdapter(
    std::unique_ptr<digital_twin::IRobotSimTransport> transport,
    QObject *parent)
    : QObject(parent)
    , m_robotSimService(std::move(transport))
{
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::applyGlobalLocalizationCorrection(
    const localization::RigidTransform &virtualToRealCorrection)
{
    return applyWorkpieceCorrection(virtualToRealCorrection, 0);
}

void CGlobalLocatePriorWorkerAdapter::setLocalRegistrationEpoch(quint64 epoch) noexcept
{
    m_localRegistrationEpoch.store(epoch);
}

void CGlobalLocatePriorWorkerAdapter::setLocalRegistrationStateDeadline(qint64 deadlineMs) noexcept
{
    m_localRegistrationStateDeadlineMs.store(deadlineMs);
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::applyLocalRegistrationCorrection(
    quint64 epoch,
    const localization::RigidTransform &virtualToRealCorrection)
{
    m_localRegistrationWriteAttempted = false;
    const qint64 deadlineMs = m_localRegistrationStateDeadlineMs.load();
    if (epoch == 0 || epoch != m_localRegistrationEpoch.load()
        || (deadlineMs > 0 && QDateTime::currentMSecsSinceEpoch() > deadlineMs)) {
        return {false, QStringLiteral("局部定位代次已失效，未写回工件。")};
    }
    return applyWorkpieceCorrection(virtualToRealCorrection, epoch);
}

bool CGlobalLocatePriorWorkerAdapter::localRegistrationWriteAttempted() const
{
    return m_localRegistrationWriteAttempted;
}

void CGlobalLocatePriorWorkerAdapter::configureLocalRegistrationSafety(
    const LocalRegistrationSafetyOptions &options)
{
    m_localRegistrationSafetyOptions = options;
    m_localRegistrationSafety = {};
    m_localRegistrationKeypoints.clear();
    m_localRegistrationSafetyConfigured = true;
}

void CGlobalLocatePriorWorkerAdapter::setLocalRegistrationKeypointsInParent(
    const std::vector<localization::Point3D_mm> &points)
{
    m_localRegistrationKeypoints = points;
}

CGlobalLocatePriorWorkerAdapter::LocalRegistrationSafetySnapshot
CGlobalLocatePriorWorkerAdapter::localRegistrationSafetySnapshot() const
{
    return m_localRegistrationSafety;
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::markLocalRegistrationVerified(quint64 epoch)
{
    return verifyOrRestoreLocalRegistration(epoch, false);
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::rollbackLocalRegistrationCorrection(quint64 epoch)
{
    return verifyOrRestoreLocalRegistration(epoch, true);
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::verifyOrRestoreLocalRegistration(quint64 epoch, bool restore)
{
    m_localRegistrationWriteAttempted = false;
    QElapsedTimer timer;
    timer.start();
    auto &transaction = m_localRegistrationSafety;
    const auto eligible = [this, epoch, &transaction, &timer]() {
        const qint64 deadline = m_localRegistrationStateDeadlineMs.load();
        return epoch != 0 && epoch == m_localRegistrationEpoch.load()
            && epoch == transaction.pendingEpoch && transaction.hasPendingPose
            && transaction.hasVerifiedPose && transaction.sceneRevision == m_robotSimSceneRevision
            && transaction.workpieceId == m_robotSimScene.workpieceId
            && transaction.parentId == m_robotSimScene.workpieceParentId
            && m_robotSimEndpoint.referenceObjectId == transaction.parentId
            && m_robotSimConnected && m_robotSimConnectionMode == RobotSimConnectionMode::RealScene
            && timer.elapsed() <= 150
            && deadline > 0 && QDateTime::currentMSecsSinceEpoch() <= deadline;
    };
    if (!eligible() || transaction.rollbackAttempted
        || (!restore && !transaction.pendingPoseConfirmed)) {
        return {false, QStringLiteral("局部定位事务、场景、代次或机器人状态已失效，未恢复/晋升位姿。")};
    }
    const auto workpiece = m_robotSimService.getObjectPose(transaction.workpieceId);
    if (!workpiece.available || !eligible()) {
        return {false, QStringLiteral("局部定位事务工件读取失败或已撤销，未恢复/晋升位姿。")};
    }
    const auto parent = m_robotSimService.getObjectPose(transaction.parentId);
    if (!parent.available || !eligible()) {
        return {false, QStringLiteral("局部定位事务父级读取失败或已撤销，未恢复/晋升位姿。")};
    }
    const Eigen::Matrix4d worldFromParent = robotSimPoseToMatrix(parent.pose);
    const Eigen::Matrix4d actual = worldFromParent.inverse() * robotSimPoseToMatrix(workpiece.pose);
    if (!localRegistrationPosesMatch(actual,
            rigidTransformToMatrix(transaction.pendingParentFromWorkpiece))) {
        return {false, QStringLiteral("工件位姿已被外部修改，拒绝恢复/晋升局部定位事务。")};
    }
    if (!restore) {
        transaction.verifiedParentFromWorkpiece = transaction.pendingParentFromWorkpiece;
        transaction.verifiedByIndependentObservation = true;
        transaction.hasPendingPose = false;
        transaction.pendingPoseConfirmed = false;
        transaction.message = QStringLiteral("独立新帧验证后，工件位姿已晋升为可信位姿。");
        return {true, transaction.message};
    }
    // 恢复是显式事务，绝不由 stop、超时或取消回调隐式触发。
    if (!eligible()) {
        return {false, QStringLiteral("恢复发送前事务已撤销，未发送 SET_POSE。")};
    }
    m_localRegistrationWriteAttempted = true;
    transaction.rollbackAttempted = true;
    const Eigen::Matrix4d verified = rigidTransformToMatrix(transaction.verifiedParentFromWorkpiece);
    const auto sent = m_robotSimService.setObjectPose(transaction.workpieceId,
        matrixToRobotSimPose(verified), true, true);
    if (!sent.succeeded) {
        return {false, QStringLiteral("恢复位姿发送未确认：%1").arg(sent.message)};
    }
    const auto readback = m_robotSimService.getObjectPose(transaction.workpieceId);
    if (!readback.available || !localRegistrationPosesMatch(
            robotSimPoseToMatrix(readback.pose), worldFromParent * verified)) {
        return {false, QStringLiteral("恢复已发送，但可信位姿回读未确认，禁止重复恢复。")};
    }
    transaction.hasPendingPose = false;
    transaction.pendingPoseConfirmed = false;
    transaction.message = QStringLiteral("已恢复并回读确认上一可信工件位姿，自动更新仍应暂停。");
    return {true, transaction.message};
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::checkLocalRegistrationCandidate(
    const localization::RigidTransform &current,
    const localization::RigidTransform &candidate)
{
    auto &transaction = m_localRegistrationSafety;
    const auto &options = m_localRegistrationSafetyOptions;
    if (transaction.hasPendingPose) {
        return {false, QStringLiteral("上一局部修正尚未几何验证，禁止连续累积写回。")};
    }
    if (!std::isfinite(options.maximumCumulativeTranslationMm)
        || options.maximumCumulativeTranslationMm <= 0.0
        || !std::isfinite(options.maximumCumulativeRotationDeg)
        || options.maximumCumulativeRotationDeg <= 0.0
        || !std::isfinite(options.maximumKeypointDisplacementMm)
        || options.maximumKeypointDisplacementMm <= 0.0
        || m_localRegistrationKeypoints.empty()) {
        return {false, QStringLiteral("累计安全预算或独立关键点无效，拒绝局部写回。")};
    }
    const Eigen::Matrix4d currentMatrix = rigidTransformToMatrix(current);
    const Eigen::Matrix4d candidateMatrix = rigidTransformToMatrix(candidate);
    const Eigen::Matrix3d rotation = candidateMatrix.block<3, 3>(0, 0);
    if (!currentMatrix.allFinite() || !candidateMatrix.allFinite()
        || (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5
        || std::abs(rotation.determinant() - 1.0) > 1e-5
        || (candidateMatrix.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-8) {
        return {false, QStringLiteral("候选位姿不是有限刚体变换，拒绝局部写回。")};
    }
    if (!transaction.hasBaseline) {
        // 初始现场位姿是会话恢复基线；此处不表示它已经通过新采样几何验收。
        transaction.hasBaseline = true;
        transaction.hasVerifiedPose = true;
        transaction.sceneRevision = m_robotSimSceneRevision;
        transaction.workpieceId = m_robotSimScene.workpieceId;
        transaction.parentId = m_robotSimScene.workpieceParentId;
        transaction.baselineParentFromWorkpiece = current;
        transaction.verifiedParentFromWorkpiece = current;
    }
    if (transaction.sceneRevision != m_robotSimSceneRevision
        || transaction.workpieceId != m_robotSimScene.workpieceId
        || transaction.parentId != m_robotSimScene.workpieceParentId
        || !localRegistrationPosesMatch(currentMatrix,
            rigidTransformToMatrix(transaction.verifiedParentFromWorkpiece))) {
        return {false, QStringLiteral("场景身份或工件位姿已变更，必须显式建立新会话，拒绝累积写回。")};
    }
    const Eigen::Matrix4d baseline = rigidTransformToMatrix(transaction.baselineParentFromWorkpiece);
    const Eigen::Matrix4d cumulative = candidateMatrix * baseline.inverse();
    transaction.cumulativeTranslationMm =
        (candidateMatrix.block<3, 1>(0, 3) - baseline.block<3, 1>(0, 3)).norm();
    transaction.cumulativeRotationDeg = std::acos(std::clamp(
        (cumulative.block<3, 3>(0, 0).trace() - 1.0) * 0.5, -1.0, 1.0))
        * 180.0 / 3.14159265358979323846;
    transaction.maximumKeypointDisplacementMm = 0.0;
    for (const auto &point : m_localRegistrationKeypoints) {
        const Eigen::Vector4d parentPoint(point.x_mm, point.y_mm, point.z_mm, 1.0);
        if (!parentPoint.allFinite()) {
            return {false, QStringLiteral("独立关键点包含非法数值，拒绝局部写回。")};
        }
        const Eigen::Vector4d materialPoint = currentMatrix.inverse() * parentPoint;
        const Eigen::Vector4d targetPoint = candidateMatrix * materialPoint;
        const double step = (targetPoint - parentPoint).head<3>().norm();
        const double total = (targetPoint - baseline * materialPoint).head<3>().norm();
        transaction.maximumKeypointDisplacementMm = std::max(
            transaction.maximumKeypointDisplacementMm, std::max(step, total));
    }
    if (transaction.cumulativeTranslationMm > options.maximumCumulativeTranslationMm
        || transaction.cumulativeRotationDeg > options.maximumCumulativeRotationDeg
        || transaction.maximumKeypointDisplacementMm > options.maximumKeypointDisplacementMm) {
        return {false, QStringLiteral("累计位姿或关键点位移超过会话预算，拒绝局部写回。")};
    }
    return {true, QStringLiteral("累计位姿和独立关键点预算通过。")};
}

digital_twin::DigitalTwinCommandResult
CGlobalLocatePriorWorkerAdapter::applyWorkpieceCorrection(
    const localization::RigidTransform &virtualToRealCorrection,
    quint64 epoch)
{
    QElapsedTimer writebackTimer;
    writebackTimer.start();
    if (!m_robotSimConnected
        || m_robotSimConnectionMode != RobotSimConnectionMode::RealScene) {
        return {
            false,
            QStringLiteral("RobotSim 真实场景未连接，无法写回全局定位结果。")
        };
    }
    if (m_robotSimScene.workpieceId <= 0
        || m_robotSimScene.workpieceParentId <= 0) {
        return {
            false,
            QStringLiteral("RobotSim 工件或工件父级对象 ID 未配置。")
        };
    }
    if (m_robotSimEndpoint.referenceObjectId
        != m_robotSimScene.workpieceParentId) {
        return {
            false,
            QStringLiteral(
                "RobotSim 点云参考对象与工件父级不一致，无法安全写回定位矩阵。")
        };
    }

    const auto workpiecePose =
        m_robotSimService.getObjectPose(m_robotSimScene.workpieceId);
    if (!workpiecePose.available) {
        return {false, QStringLiteral("读取 RobotSim 工件位姿失败：%1").arg(workpiecePose.message)};
    }
    if (epoch != 0 && (epoch != m_localRegistrationEpoch.load()
        || writebackTimer.elapsed() > 150
        || (m_localRegistrationStateDeadlineMs.load() > 0
            && QDateTime::currentMSecsSinceEpoch() > m_localRegistrationStateDeadlineMs.load()))) {
        return {false, QStringLiteral("局部定位读取已取消或超过150 ms，未继续查询和写回。")};
    }
    const auto parentPose =
        m_robotSimService.getObjectPose(m_robotSimScene.workpieceParentId);
    if (!workpiecePose.available || !parentPose.available) {
        return {
            false,
            QStringLiteral("读取 RobotSim 工件/父级绝对位姿失败：%1；%2")
                .arg(workpiecePose.message, parentPose.message)
        };
    }

    const Eigen::Matrix4d worldFromParent =
        robotSimPoseToMatrix(parentPose.pose);
    const Eigen::Matrix4d worldFromWorkpiece =
        robotSimPoseToMatrix(workpiecePose.pose);
    const Eigen::Matrix4d parentFromWorkpiece =
        worldFromParent.inverse() * worldFromWorkpiece;
    // 修正需与工件相对位姿使用同一父级参考系；局部修正已在父级 TCP 系，
    // 直接左乘，不再叠加或共轭末节到 TCP 的 231 mm 偏移。
    const Eigen::Matrix4d correctedParentFromWorkpiece =
        rigidTransformToMatrix(virtualToRealCorrection)
        * parentFromWorkpiece;
    if (!correctedParentFromWorkpiece.allFinite()) {
        return {
            false,
            QStringLiteral("全局定位修正后的 RobotSim 工件位姿包含非法数值。")
        };
    }

    const bool protectLocalWrite = epoch != 0 && m_localRegistrationSafetyConfigured
        && m_localRegistrationSafetyOptions.enabled;
    if (protectLocalWrite) {
        if (m_localRegistrationStateDeadlineMs.load() <= 0) {
            return {false, QStringLiteral("局部定位安全写回缺少机器人状态截止时间，拒绝发送。")};
        }
        const auto safety = checkLocalRegistrationCandidate(
            matrixToRigidTransform(parentFromWorkpiece),
            matrixToRigidTransform(correctedParentFromWorkpiece));
        m_localRegistrationSafety.message = safety.message;
        if (!safety.succeeded) {
            return safety;
        }
    }

    // 网络读取期间可能已停止或切换焊缝；不可将旧模型的修正应用到新模型。
    if (epoch != 0 && (epoch != m_localRegistrationEpoch.load()
        || writebackTimer.elapsed() > 150
        || (m_localRegistrationStateDeadlineMs.load() > 0
            && QDateTime::currentMSecsSinceEpoch() > m_localRegistrationStateDeadlineMs.load()))) {
        return {false, QStringLiteral("局部定位读取期间已取消，未发送 SET_POSE。")};
    }
    if (epoch != 0) {
        m_localRegistrationWriteAttempted = true;
    }
    if (protectLocalWrite) {
        // 在发送前保留目标；SET_POSE/回读异常时，也不可遗失待验证事务。
        m_localRegistrationSafety.beforeParentFromWorkpiece = matrixToRigidTransform(parentFromWorkpiece);
        m_localRegistrationSafety.pendingParentFromWorkpiece = matrixToRigidTransform(correctedParentFromWorkpiece);
        m_localRegistrationSafety.hasPendingPose = true;
        m_localRegistrationSafety.pendingPoseConfirmed = false;
        m_localRegistrationSafety.rollbackAttempted = false;
        m_localRegistrationSafety.pendingEpoch = epoch;
    }
    const auto commandResult = m_robotSimService.setObjectPose(
        m_robotSimScene.workpieceId,
        matrixToRobotSimPose(correctedParentFromWorkpiece),
        true,
        true);
    if (!commandResult.succeeded) {
        return {
            false,
            QStringLiteral("全局定位写回 RobotSim 失败：%1")
                .arg(commandResult.message)
        };
    }

    // SET_POSE 是无响应命令，通过绝对位姿查询确认场景已采用目标位姿。
    const auto verifiedPose =
        m_robotSimService.getObjectPose(m_robotSimScene.workpieceId);
    if (!verifiedPose.available) {
        return {
            false,
            QStringLiteral("SET_POSE 已发送，但工件位姿回读失败：%1")
                .arg(verifiedPose.message)
        };
    }
    const Eigen::Matrix4d expectedWorldPose =
        worldFromParent * correctedParentFromWorkpiece;
    const Eigen::Matrix4d actualWorldPose =
        robotSimPoseToMatrix(verifiedPose.pose);
    const bool poseConfirmed = actualWorldPose.allFinite()
        && (actualWorldPose.block<3, 1>(0, 3)
            - expectedWorldPose.block<3, 1>(0, 3)).norm() < 1e-3
        && (actualWorldPose.block<3, 3>(0, 0)
            - expectedWorldPose.block<3, 3>(0, 0)).norm() < 1e-5;
    if (protectLocalWrite) {
        m_localRegistrationSafety.pendingPoseConfirmed = poseConfirmed;
    }
    return {
        poseConfirmed,
        poseConfirmed
            ? QStringLiteral(
                  "定位矩阵已写回并确认 RobotSim 工件 %1。")
                  .arg(m_robotSimScene.workpieceId)
            : QStringLiteral("SET_POSE 已发送，但回读位姿与目标不一致。")
    };
}

void CGlobalLocatePriorWorkerAdapter::generateVirtualPrior(
    quint64 requestId,
    const digital_twin::WeldmentVirtualLaserRequest &request)
{
    emit virtualPriorReady(requestId, m_generator.generate(request));
}

void CGlobalLocatePriorWorkerAdapter::configureRobotSim(
    const digital_twin::RobotSimEndpointConfig &endpoint,
    const digital_twin::RobotSimSceneConfig &scene)
{
    ++m_robotSimSceneRevision;
    ++m_robotSimManualPollGeneration;
    m_robotSimService.stop();
    m_robotInverseKinematics.reset();
    m_robotSimEndpoint = endpoint;
    m_robotSimScene = scene;
    m_robotSimConnected = false;
    m_robotSimConnectionMode = RobotSimConnectionMode::Disconnected;
    m_robotSimConfigured = endpoint.port > 0
        && !endpoint.host.trimmed().isEmpty()
        && scene.robotId > 0
        && scene.positionerId > 0
        && scene.sensorId > 0;
}

void CGlobalLocatePriorWorkerAdapter::connectRobotSimManual(
    const digital_twin::RobotSimEndpointConfig &endpoint,
    const digital_twin::RobotSimSceneConfig &scene)
{
    configureRobotSim(endpoint, scene);
    if (endpoint.port == 0
        || endpoint.host.trimmed().isEmpty()
        || scene.sensorId <= 0) {
        emit robotSimManualConnectionStateChanged(
            false,
            QStringLiteral("RobotSim endpoint 或线结构光传感器 ID 未配置。"));
        return;
    }

    const auto startResult = m_robotSimService.start(m_robotSimEndpoint);
    if (startResult.state != digital_twin::DigitalTwinServiceState::Idle) {
        emit robotSimManualConnectionStateChanged(false, startResult.message);
        return;
    }

    m_robotSimConnectionMode = RobotSimConnectionMode::ManualSimulation;
    pollRobotSimManualReference();
}

void CGlobalLocatePriorWorkerAdapter::pollRobotSimManualReference()
{
    if (m_robotSimConnectionMode
        != RobotSimConnectionMode::ManualSimulation) {
        return;
    }

    digital_twin::DigitalTwinVirtualReferenceRequest request;
    request.workpieceId = m_robotSimScene.sceneName;
    request.sensorId = m_robotSimScene.sensorId;
    request.needLocalPointCloud = false;
    request.needWeldSeamPoint = true;

    const auto reference = m_robotSimService.requestVirtualReference(request);
    if (!reference.supported) {
        m_robotSimService.stop();
        m_robotSimConnected = false;
        m_robotSimConnectionMode = RobotSimConnectionMode::Disconnected;
        emit robotSimManualConnectionStateChanged(false, reference.message);
        return;
    }

    if (!m_robotSimConnected) {
        m_robotSimConnected = true;
        emit robotSimManualConnectionStateChanged(
            true,
            QStringLiteral(
                "RobotSim 手动仿真场景通信成功；未发送 SET_Q_VEC，"
                "正在持续请求仿真激光线和焊缝点。"));
    }
    emit robotSimManualVirtualReferenceReady(reference);

    const quint64 generation = m_robotSimManualPollGeneration;
    QTimer::singleShot(
        kRobotSimManualPollIntervalMs,
        this,
        [this, generation]() {
            if (generation == m_robotSimManualPollGeneration) {
                pollRobotSimManualReference();
            }
        });
}

void CGlobalLocatePriorWorkerAdapter::connectRobotSim(
    const digital_twin::RobotSimEndpointConfig &endpoint,
    const digital_twin::RobotSimSceneConfig &scene,
    const robot_comm::RobotState &initialState)
{
    configureRobotSim(endpoint, scene);
    if (!m_robotSimConfigured) {
        emit robotSimConnectionStateChanged(
            false,
            QStringLiteral("RobotSim endpoint 或场景对象 ID 未配置。"));
        return;
    }

    std::vector<double> robotJointValuesRad;
    std::vector<double> positionerJointValuesRad;
    QString jointMessage;
    if (!buildRobotSimJointVectors(
            initialState,
            robotJointValuesRad,
            positionerJointValuesRad,
            jointMessage)) {
        emit robotSimConnectionStateChanged(
            false,
            QStringLiteral("RobotSim 首次位姿同步失败：%1").arg(jointMessage));
        return;
    }

    const auto startResult = m_robotSimService.start(m_robotSimEndpoint);
    if (startResult.state != digital_twin::DigitalTwinServiceState::Idle) {
        emit robotSimConnectionStateChanged(false, startResult.message);
        return;
    }

    digital_twin::DigitalTwinVirtualReferenceRequest request;
    request.workpieceId = m_robotSimScene.sceneName;
    request.robotId = m_robotSimScene.robotId;
    request.positionerId = m_robotSimScene.positionerId;
    request.sensorId = m_robotSimScene.sensorId;
    request.robotJointValuesRad = std::move(robotJointValuesRad);
    request.positionerJointValuesRad = std::move(positionerJointValuesRad);
    request.needLaserLinePoints = true;
    request.needLocalPointCloud = false;
    request.needWeldSeamPoint = true;

    const auto reference = m_robotSimService.requestVirtualReference(request);
    if (!reference.supported) {
        m_robotSimService.stop();
        m_robotSimConnectionMode = RobotSimConnectionMode::Disconnected;
        emit robotSimConnectionStateChanged(
            false,
            reference.message);
        return;
    }

    m_robotSimConnected = true;
    m_robotSimConnectionMode = RobotSimConnectionMode::RealScene;
    emit robotSimConnectionStateChanged(
        true,
        QStringLiteral(
            "RobotSim 通信与激光响应已确认；已发送机器人和变位机关节值。"
            "SET_Q_VEC 按协议无响应，客户端无法确认 RobotSim 模型是否接受。"));
    emit robotSimRealtimeVirtualReferenceReady(0, reference);
    emit robotSimRealtimeVirtualPriorReady(
        0,
        digital_twin::makeVisionLaserObservation(reference));
}

void CGlobalLocatePriorWorkerAdapter::synchronizeRobotSimState(
    quint64 requestId,
    const robot_comm::RobotState &state)
{
    if (!m_robotSimConnected
        || m_robotSimConnectionMode != RobotSimConnectionMode::RealScene) {
        emit robotSimStateSynchronized(
            requestId,
            false,
            QStringLiteral("RobotSim 尚未确认连接。"));
        return;
    }

    std::vector<double> robotJointValuesRad;
    std::vector<double> positionerJointValuesRad;
    QString message;
    if (!buildRobotSimJointVectors(
            state,
            robotJointValuesRad,
            positionerJointValuesRad,
            message)) {
        emit robotSimStateSynchronized(requestId, false, message);
        return;
    }

    digital_twin::DigitalTwinVirtualReferenceRequest request;
    request.workpieceId = m_robotSimScene.sceneName;
    request.robotId = m_robotSimScene.robotId;
    request.positionerId = m_robotSimScene.positionerId;
    request.sensorId = m_robotSimScene.sensorId;
    request.robotJointValuesRad = std::move(robotJointValuesRad);
    request.positionerJointValuesRad = std::move(positionerJointValuesRad);
    request.needLaserLinePoints = true;
    request.needLocalPointCloud = false;
    request.needWeldSeamPoint = true;

    const auto reference = m_robotSimService.requestVirtualReference(request);
    const auto observation =
        digital_twin::makeVisionLaserObservation(reference);
    if (!reference.supported) {
        m_robotSimService.stop();
        m_robotSimConnected = false;
        m_robotSimConnectionMode = RobotSimConnectionMode::Disconnected;
        emit robotSimConnectionStateChanged(false, reference.message);
    }
    emit robotSimRealtimeVirtualReferenceReady(requestId, reference);
    emit robotSimRealtimeVirtualPriorReady(requestId, observation);
    emit robotSimStateSynchronized(
        requestId,
        reference.supported,
        reference.message);
}

void CGlobalLocatePriorWorkerAdapter::generateRobotSimVirtualPrior(
    quint64 requestId,
    qint64 frameId,
    const robot_comm::RobotState &state,
    int positioningSegmentIndex)
{
    if (!m_robotSimConfigured) {
        emit robotSimVirtualPriorReady(
            requestId,
            digital_twin::makeInvalidVisionLaserObservation(
                frameId,
                QStringLiteral("RobotSim endpoint or scene IDs are not configured")));
        return;
    }

    if (!m_robotSimConnected
        || m_robotSimConnectionMode != RobotSimConnectionMode::RealScene) {
        emit robotSimVirtualPriorReady(
            requestId,
            digital_twin::makeInvalidVisionLaserObservation(
                frameId,
                QStringLiteral("RobotSim 尚未通过定位界面的确定按钮连接。")));
        return;
    }

    std::vector<double> robotJointValuesRad;
    std::vector<double> positionerJointValuesRad;
    QString jointMessage;
    if (!buildRobotSimJointVectors(
            state,
            robotJointValuesRad,
            positionerJointValuesRad,
            jointMessage)) {
        emit robotSimVirtualPriorReady(
            requestId,
            digital_twin::makeInvalidVisionLaserObservation(
                frameId,
                jointMessage));
        return;
    }

    digital_twin::DigitalTwinVirtualReferenceRequest request;
    request.frameId = frameId;
    request.workpieceId = m_robotSimScene.sceneName;
    request.robotId = m_robotSimScene.robotId;
    request.positionerId = m_robotSimScene.positionerId;
    request.sensorId = m_robotSimScene.sensorId;
    request.robotJointValuesRad = std::move(robotJointValuesRad);
    request.positionerJointValuesRad = std::move(positionerJointValuesRad);
    request.needLaserLinePoints = true;
    request.needLocalPointCloud = false;
    request.needWeldSeamPoint = true;

    const auto reference = m_robotSimService.requestVirtualReference(request);
    auto observation = digital_twin::makeVisionLaserObservation(reference);
    // 即使二维先验无效，也保留 RobotSim 已返回的三维线点用于配准点云采集。
    observation.laserLinePoints_mm = reference.laserLinePoints_mm;
    observation.positioningSegmentIndex = positioningSegmentIndex;
    if (!reference.supported) {
        m_robotSimService.stop();
        m_robotSimConnected = false;
        m_robotSimConnectionMode = RobotSimConnectionMode::Disconnected;
        emit robotSimConnectionStateChanged(false, reference.message);
    }
    emit robotSimVirtualPriorReady(requestId, observation);
}

bool CGlobalLocatePriorWorkerAdapter::buildRobotSimJointVectors(
    const robot_comm::RobotState &state,
    std::vector<double> &robotJointValuesRad,
    std::vector<double> &positionerJointValuesRad,
    QString &message)
{
    digital_twin::MotomanMh12TcpPose tcpPose;
    tcpPose.xMm = static_cast<double>(state.curx_r) / 1000.0;
    tcpPose.yMm = static_cast<double>(state.cury_r) / 1000.0;
    tcpPose.zMm = static_cast<double>(state.curz_r) / 1000.0;
    tcpPose.rxDeg = static_cast<double>(state.curRx_r) / 10000.0;
    tcpPose.ryDeg = static_cast<double>(state.curRy_r) / 10000.0;
    tcpPose.rzDeg = static_cast<double>(state.curRz_r) / 10000.0;
    const auto inverseResult = m_robotInverseKinematics.solve(tcpPose);
    if (!inverseResult.succeeded) {
        message = QString::fromStdString(inverseResult.message);
        return false;
    }

    robotJointValuesRad.assign(
        inverseResult.jointAnglesRad.begin(),
        inverseResult.jointAnglesRad.end());
    constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    // SWv1 vsUpdateRobot/getFirstFrameLaserLine 均以 [curS1, curS2]
    // 顺序、同号写入 MA2P_300D 的两个主动转动关节。
    positionerJointValuesRad = {
        static_cast<double>(state.curS1) / 10000.0 * DEG_TO_RAD,
        static_cast<double>(state.curS2) / 10000.0 * DEG_TO_RAD};
    message.clear();
    return true;
}

void CGlobalLocatePriorWorkerAdapter::resetRobotSim()
{
    ++m_robotSimManualPollGeneration;
    m_robotSimService.stop();
    m_robotInverseKinematics.reset();
    m_robotSimConnected = false;
    m_robotSimConnectionMode = RobotSimConnectionMode::Disconnected;
}

CGlobalLocalizationWorkerAdapter::CGlobalLocalizationWorkerAdapter(
    QObject *parent)
    : QObject(parent)
{
}

void CGlobalLocalizationWorkerAdapter::apply(
    quint64 requestId,
    const StandaloneGlobalLocalizationRequest &request)
{
    StandaloneGlobalLocalizationResult result;
    const QDir outputDir(request.outputDirectory);
    auto fail = [&result](const QString &message) {
        result.succeeded = false;
        result.message = message;
    };
    auto savePointCloud = [&result, &outputDir](
                              const QString &fileName,
                              const std::vector<localization::Point3D_mm> &points,
                              const QString &description) {
        const QString filePath = outputDir.filePath(fileName);
        QString errorMessage;
        if (!writeGlobalLocatePlyPointCloudFile(
                filePath,
                points,
                &errorMessage)) {
            result.warningMessages.push_back(
                QStringLiteral("全局定位输出失败：%1，error=%2")
                    .arg(filePath, errorMessage));
            return false;
        }
        result.infoMessages.push_back(
            QStringLiteral("全局定位输出：%1，points=%2，file=%3")
                .arg(description)
                .arg(static_cast<int>(points.size()))
                .arg(filePath));
        return true;
    };

    std::vector<localization::Point3D_mm> allRealPoints;
    for (const auto &segment : request.realSegments) {
        allRealPoints.insert(
            allRealPoints.end(),
            segment.begin(),
            segment.end());
    }
    savePointCloud(
        QStringLiteral("global_locate_real.ply"),
        allRealPoints,
        QStringLiteral("全部真实定位面点云"));
    for (std::size_t index = 0; index < request.realSegments.size(); ++index) {
        savePointCloud(
            QStringLiteral("global_locate_real_segment_%1.ply")
                .arg(static_cast<int>(index + 1)),
            request.realSegments[index],
            QStringLiteral("真实定位面 %1")
                .arg(static_cast<int>(index + 1)));
    }

    result.infoMessages.push_back(
        QStringLiteral("全局定位步骤 6/7：开始解析 weldment 定位段和自由度，file=%1")
            .arg(request.weldmentPath));
    digital_twin::WeldmentVirtualPointCloudGenerator generator;
    const auto cloud = generator.generate(request.weldmentPath.toStdString());
    if (!cloud.ok || cloud.points_mm.empty()) {
        fail(QStringLiteral("全局定位失败：weldment 虚拟点云生成失败：%1")
                 .arg(QString::fromStdString(cloud.message)));
        emit resultReady(requestId, result);
        return;
    }
    if (allRealPoints.empty()) {
        fail(QStringLiteral(
                 "全局定位失败：虚拟参考配置已解析，但扫描未采集到有效三维点。trackingFrames=%1，triggerRejected=%2，invalid=%3，priorSuccess=%4，priorFailure=%5，lastTracking=%6，lastPrior=%7。请重点检查 JBI 的 STARTDETPRG/STARTPOSSEG、Steger 提取和标定投影日志。")
                 .arg(request.trackingFrameCount)
                 .arg(request.triggerRejectedFrameCount)
                 .arg(request.invalidFrameCount)
                 .arg(request.priorSuccessCount)
                 .arg(request.priorFailureCount)
                 .arg(request.lastTrackingMessage)
                 .arg(request.lastPriorMessage));
        emit resultReady(requestId, result);
        return;
    }

    const bool useRobotSimRegistrationPointCloud =
        request.virtualPointCloudSource
        == GlobalLocateVirtualPointCloudSource::RobotSimLaserScan;
    const auto &virtualSegments = useRobotSimRegistrationPointCloud
        ? request.robotSimVirtualSegments
        : cloud.segmentPoints_mm;
    const QString virtualPointCloudSourceTag =
        useRobotSimRegistrationPointCloud
        ? QStringLiteral("robotsim_scan")
        : QStringLiteral("weldment");
    const QString virtualPointCloudSourceText =
        useRobotSimRegistrationPointCloud
        ? QStringLiteral("RobotSim 仿真激光扫描")
        : QStringLiteral("weldment 模型采样");

    if (request.realSegments.size() != virtualSegments.size()) {
        fail(QStringLiteral("全局定位失败：真实定位面数量 %1 与 %2 虚拟定位面数量 %3 不一致。请检查 JBI 中 STARTPOSSEG/ENDPOSSEG 的数量和顺序以及 RobotSim 每帧三维激光响应。")
                 .arg(static_cast<int>(request.realSegments.size()))
                 .arg(virtualPointCloudSourceText)
                 .arg(static_cast<int>(virtualSegments.size())));
        emit resultReady(requestId, result);
        return;
    }
    if (request.realSegments.size() != cloud.segmentDofMasks.size()) {
        fail(QStringLiteral("全局定位失败：真实定位面数量 %1 与 weldment 自由度配置数量 %2 不一致。")
                 .arg(static_cast<int>(request.realSegments.size()))
                 .arg(static_cast<int>(cloud.segmentDofMasks.size())));
        emit resultReady(requestId, result);
        return;
    }

    std::vector<localization::Point3D_mm> allVirtualPoints;
    for (std::size_t index = 0; index < request.realSegments.size(); ++index) {
        if (request.realSegments[index].empty()) {
            fail(QStringLiteral("全局定位失败：已检测到定位面 %1 的 STARTPOSSEG/ENDPOSSEG，但该面没有采集到有效三维点。请检查该面 STARTDETPRG 窗口内的虚拟激光参考线、ROI 和 Steger 日志。")
                     .arg(static_cast<int>(index + 1)));
            emit resultReady(requestId, result);
            return;
        }
        if (virtualSegments[index].empty()) {
            fail(QStringLiteral("全局定位失败：%1 在定位面 %2 没有累计到有效虚拟三维点。")
                     .arg(virtualPointCloudSourceText)
                     .arg(static_cast<int>(index + 1)));
            emit resultReady(requestId, result);
            return;
        }
        allVirtualPoints.insert(
            allVirtualPoints.end(),
            virtualSegments[index].begin(),
            virtualSegments[index].end());
    }

    result.infoMessages.push_back(
        QStringLiteral("全局定位步骤 6/7：配准点云来源=%1，全部 %2 点，%3 个定位面。")
            .arg(virtualPointCloudSourceText)
            .arg(static_cast<int>(allVirtualPoints.size()))
            .arg(static_cast<int>(virtualSegments.size())));
    savePointCloud(
        QStringLiteral("global_locate_virtual_%1_reference.ply")
            .arg(virtualPointCloudSourceTag),
        allVirtualPoints,
        QStringLiteral("%1全部虚拟参考点云")
            .arg(virtualPointCloudSourceText));
    for (std::size_t index = 0; index < virtualSegments.size(); ++index) {
        savePointCloud(
            QStringLiteral("global_locate_virtual_%1_segment_%2.ply")
                .arg(virtualPointCloudSourceTag)
                .arg(static_cast<int>(index + 1)),
            virtualSegments[index],
            QStringLiteral("%1虚拟定位面 %2")
                .arg(virtualPointCloudSourceText)
                .arg(static_cast<int>(index + 1)));
    }

    localization::LocalizationGlobalRuntimeSegmentedRequest segmentedRequest;
    segmentedRequest.frameId = request.frameId;
    segmentedRequest.defaultDofMask = {true, true, true, false, false, true};
    segmentedRequest.defaultParameters.preprocessingProfile =
        localization::IcpPreprocessingProfile::CompatibilitySwv1GlobalIcp;
    segmentedRequest.defaultParameters.enableSorFilter = true;
    segmentedRequest.defaultParameters.sorMeanK = 5;
    segmentedRequest.defaultParameters.sorStddevMulThresh = 2.0;
    segmentedRequest.defaultParameters.maxCorrespondenceDistance_mm = 0.0;
    segmentedRequest.defaultParameters.maximumIterations = 10000;
    segmentedRequest.defaultParameters.transformationEpsilon = 1e-7;
    segmentedRequest.virtualReferenceSnapshot.available = true;
    segmentedRequest.virtualReferenceSnapshot.referenceId =
        virtualPointCloudSourceText;
    segmentedRequest.virtualReferenceSnapshot.workpiecePointCloud_mm =
        allVirtualPoints;

    for (std::size_t index = 0; index < virtualSegments.size(); ++index) {
        localization::GlobalIcpSegmentRequest segment;
        segment.segmentId = QStringLiteral("segment_%1")
                                .arg(static_cast<int>(index + 1));
        segment.realPoints_mm = request.realSegments[index];
        segment.virtualReferencePoints_mm = virtualSegments[index];
        segment.dofMask = cloud.segmentDofMasks[index];
        segment.parameters = segmentedRequest.defaultParameters;
        segmentedRequest.segments.push_back(std::move(segment));
    }

    auto wholeRequest = segmentedRequest;
    wholeRequest.segments.clear();
    localization::GlobalIcpSegmentRequest wholeSegment;
    wholeSegment.segmentId = QStringLiteral("whole_cloud");
    wholeSegment.realPoints_mm = allRealPoints;
    wholeSegment.virtualReferencePoints_mm = allVirtualPoints;
    wholeSegment.dofMask = wholeRequest.defaultDofMask;
    wholeSegment.parameters = wholeRequest.defaultParameters;
    wholeRequest.segments.push_back(std::move(wholeSegment));

    result.infoMessages.push_back(
        QStringLiteral("全局定位步骤 7/7：[分段配准] 点云来源=%1，开始执行 %2 个定位面的受限 ICP；每面使用对应真实点云和对应虚拟定位面，上一面完整结果作为下一面初值。真实点预处理=SWv1 FPS(1/5)+SOR(MeanK=5,Stddev=2)，粗配准最大对应距离=PCL默认，粗配准完成后使用基于RMSE的自适应对应距离执行精配准。")
            .arg(virtualPointCloudSourceText)
            .arg(static_cast<int>(segmentedRequest.segments.size())));
    const auto segmentedApplicationResult =
        m_localizationService.runGlobalIcpRuntimeSegmented(segmentedRequest);
    result.hasSegmentedUpdate = true;
    result.segmentedUpdate = segmentedApplicationResult.update;

    result.infoMessages.push_back(
        QStringLiteral("全局定位步骤 7/7：[整体配准] 点云来源=%1，开始把全部真实扫描点与全部虚拟定位点作为两个完整点云执行一次独立 ICP。")
            .arg(virtualPointCloudSourceText));
    const auto wholeApplicationResult =
        m_localizationService.runGlobalIcpRuntimeSegmented(wholeRequest);
    result.hasWholeUpdate = true;
    result.wholeUpdate = wholeApplicationResult.update;

    std::vector<localization::Point3D_mm> allSegmentedPreprocessedRealPoints;
    if (segmentedApplicationResult.hasGlobalIcpResult) {
        const auto &segmentResults =
            segmentedApplicationResult.globalIcpResult.segmentResults;
        for (std::size_t index = 0; index < segmentResults.size(); ++index) {
            const auto &preprocessed =
                segmentResults[index].icpResult.preprocessedRealPoints_mm;
            allSegmentedPreprocessedRealPoints.insert(
                allSegmentedPreprocessedRealPoints.end(),
                preprocessed.begin(),
                preprocessed.end());
            if (!preprocessed.empty()) {
                savePointCloud(
                    QStringLiteral("global_locate_segmented_real_segment_%1_preprocessed.ply")
                        .arg(static_cast<int>(index + 1)),
                    preprocessed,
                    QStringLiteral("[分段配准] 真实定位面 %1 ICP预处理后点云")
                        .arg(static_cast<int>(index + 1)));
            }
        }
    }
    if (!allSegmentedPreprocessedRealPoints.empty()) {
        savePointCloud(
            QStringLiteral("global_locate_segmented_real_preprocessed.ply"),
            allSegmentedPreprocessedRealPoints,
            QStringLiteral("[分段配准] 全部真实定位面 ICP预处理后点云"));
    }

    if (wholeApplicationResult.hasGlobalIcpResult
        && !wholeApplicationResult.globalIcpResult.segmentResults.empty()) {
        const auto &wholePreprocessed =
            wholeApplicationResult.globalIcpResult.segmentResults.front()
                .icpResult.preprocessedRealPoints_mm;
        if (!wholePreprocessed.empty()) {
            savePointCloud(
                QStringLiteral("global_locate_whole_real_preprocessed.ply"),
                wholePreprocessed,
                QStringLiteral("[整体配准] 全部真实扫描点 ICP预处理后点云"));
        }
    }

    const auto isSucceeded = [](const auto &applicationResult) {
        return applicationResult.hasGlobalIcpResult
            && applicationResult.globalIcpResult.status
                == localization::LocalizationStatus::Succeeded;
    };
    const bool segmentedSucceeded = isSucceeded(segmentedApplicationResult);
    const bool wholeSucceeded = isSucceeded(wholeApplicationResult);
    if (!segmentedSucceeded || !wholeSucceeded) {
        QString failureDetails;
        if (!segmentedSucceeded) {
            failureDetails += QStringLiteral("分段配准失败：%1")
                .arg(segmentedApplicationResult.hasGlobalIcpResult
                    ? segmentedApplicationResult.globalIcpResult.message
                    : QStringLiteral("未返回全局定位结果。"));
        }
        if (!wholeSucceeded) {
            if (!failureDetails.isEmpty()) {
                failureDetails += QStringLiteral("；");
            }
            failureDetails += QStringLiteral("整体配准失败：%1")
                .arg(wholeApplicationResult.hasGlobalIcpResult
                    ? wholeApplicationResult.globalIcpResult.message
                    : QStringLiteral("未返回全局定位结果。"));
        }
        fail(QStringLiteral("全局定位失败：%1").arg(failureDetails));
        emit resultReady(requestId, result);
        return;
    }

    const auto &segmentedResult = segmentedApplicationResult.globalIcpResult;
    const auto &wholeResult = wholeApplicationResult.globalIcpResult;
    result.correctionTransform =
        segmentedResult.combinedVirtualToRealCorrectionTransform;
    savePointCloud(
        QStringLiteral("global_locate_segmented_%1_virtual_corrected.ply")
            .arg(virtualPointCloudSourceTag),
        segmentedResult.correctedVirtualReferencePoints_mm,
        QStringLiteral("[分段配准][%1] 最终修正后的虚拟点云")
            .arg(virtualPointCloudSourceText));
    savePointCloud(
        QStringLiteral("global_locate_whole_%1_virtual_corrected.ply")
            .arg(virtualPointCloudSourceTag),
        wholeResult.correctedVirtualReferencePoints_mm,
        QStringLiteral("[整体配准][%1] 最终修正后的虚拟点云")
            .arg(virtualPointCloudSourceText));

    auto saveCorrectionTransform = [&result, &outputDir](
                                       const QString &fileName,
                                       const localization::RigidTransform &transform,
                                       const QString &description) {
        QString transformText;
        QTextStream transformStream(&transformText);
        for (int index = 0; index < 16; ++index) {
            transformStream
                << transform.rowMajor[static_cast<std::size_t>(index)];
            if (index % 4 == 3) {
                transformStream << '\n';
            } else {
                transformStream << ' ';
            }
        }
        const QString transformPath = outputDir.filePath(fileName);
        QString errorMessage;
        if (writeGlobalLocateTextFile(
                transformPath,
                transformText,
                &errorMessage)) {
            result.infoMessages.push_back(
                QStringLiteral("全局定位输出：%1，file=%2")
                    .arg(description, transformPath));
        } else {
            result.warningMessages.push_back(
                QStringLiteral("全局定位修正矩阵写入失败：%1，error=%2")
                    .arg(transformPath, errorMessage));
        }
    };
    saveCorrectionTransform(
        QStringLiteral("global_locate_segmented_%1_correction_transform.txt")
            .arg(virtualPointCloudSourceTag),
        segmentedResult.combinedVirtualToRealCorrectionTransform,
        QStringLiteral("[分段配准][%1] 最终修正矩阵")
            .arg(virtualPointCloudSourceText));
    saveCorrectionTransform(
        QStringLiteral("global_locate_whole_%1_correction_transform.txt")
            .arg(virtualPointCloudSourceTag),
        wholeResult.combinedVirtualToRealCorrectionTransform,
        QStringLiteral("[整体配准][%1] 最终修正矩阵")
            .arg(virtualPointCloudSourceText));

    result.infoMessages.push_back(
        QStringLiteral("[全局定位][分段配准][加权RMSE] 配准前=%1 mm，配准后=%2 mm")
            .arg(segmentedResult.beforeResidual_mm, 0, 'f', 6)
            .arg(segmentedResult.afterResidual_mm, 0, 'f', 6));
    result.infoMessages.push_back(
        QStringLiteral("[全局定位][分段配准][普通不加权RMSE] 配准前=%1 mm，配准后=%2 mm")
            .arg(segmentedResult.beforeUnweightedResidual_mm, 0, 'f', 6)
            .arg(segmentedResult.afterUnweightedResidual_mm, 0, 'f', 6));
    result.infoMessages.push_back(
        QStringLiteral("[全局定位][整体配准][RMSE] 配准前=%1 mm，配准后=%2 mm")
            .arg(wholeResult.beforeResidual_mm, 0, 'f', 6)
            .arg(wholeResult.afterResidual_mm, 0, 'f', 6));

    result.succeeded = true;
    result.message =
        QStringLiteral("全局定位完成：点云来源=%1，%2 个定位面的分段配准和全部点云的整体一次配准均已完成；全部结果位于 %3。")
            .arg(virtualPointCloudSourceText)
            .arg(static_cast<int>(segmentedRequest.segments.size()))
            .arg(outputDir.absolutePath());
    result.infoMessages.push_back(result.message);
    emit resultReady(requestId, result);
}

COnlineCorrectionWorkerAdapter::COnlineCorrectionWorkerAdapter(QObject *parent)
    : QObject(parent)
{
}

void COnlineCorrectionWorkerAdapter::configure(
    const RuntimeOnlineCorrectionConfiguration &configuration)
{
    ++m_phaseTimeoutGeneration;
    m_timeoutPhase = control::OnlineCorrectionPhase::Idle;
    m_configuration = configuration;
    const auto trajectory =
        control::COnlineCorrectionTrajectoryPlanner::loadTrajectory(
            configuration.trajectoryRequest);
    if (!m_configuration.positionerCalibrationSnapshot.has_value()) {
        m_configuration.positionerCalibrationSnapshot =
            vision::CPositionerCalibrationParser::parseFile(configuration.rbcalibFile);
    }
    const auto &calibration = *m_configuration.positionerCalibrationSnapshot;
    m_positionerCalibration = calibration;
    m_controller.configure(configuration.options, trajectory, calibration);
    emit statusChanged(QString::fromStdString(m_controller.state().message));
}

bool COnlineCorrectionWorkerAdapter::configureForStart(
    const RuntimeOnlineCorrectionConfiguration &configuration)
{
    if (m_runRecorder.isActive()) {
        m_runRecorder.finishSession(
            QStringLiteral("restarted"),
            QStringLiteral("新的在线纠偏启动请求已替代上一轮会话。"));
    }

    configure(configuration);
    const control::OnlineCorrectionSessionState &state = m_controller.state();
    const bool configured = state.phase == control::OnlineCorrectionPhase::Idle
        && state.onlineTeachTrajectoryInitialized
        && state.positionerCalibrationInitialized;

    const QString recordingRoot =
        QString::fromStdString(m_configuration.diagnosticOutputDirectory);
    if (recordingRoot.trimmed().isEmpty()) {
        return configured;
    }

    QString recordingError;
    if (!m_runRecorder.beginSession(
            recordingRoot,
            m_configuration,
            &recordingError)) {
        emit statusChanged(QStringLiteral("在线纠偏记录初始化失败：%1")
                               .arg(recordingError));
        return false;
    }
    emit statusChanged(QStringLiteral("在线纠偏运行记录：%1")
                           .arg(m_runRecorder.sessionDirectory()));

    if (!configured) {
        m_runRecorder.finishSession(
            QStringLiteral("error"),
            QString::fromStdString(state.message));
    }
    return configured;
}

void COnlineCorrectionWorkerAdapter::startInitialDetection()
{
    const QString recordingRoot =
        QString::fromStdString(m_configuration.diagnosticOutputDirectory);
    if (!m_runRecorder.isActive() && !recordingRoot.trimmed().isEmpty()) {
        QString recordingError;
        if (!m_runRecorder.beginSession(
                recordingRoot,
                m_configuration,
                &recordingError)) {
            failSynchronization(QStringLiteral("在线纠偏记录初始化失败：%1")
                                    .arg(recordingError));
            return;
        }
        emit statusChanged(QStringLiteral("在线纠偏运行记录：%1")
                               .arg(m_runRecorder.sessionDirectory()));
    }
    publishUpdate(m_controller.startInitialDetection());
}

void COnlineCorrectionWorkerAdapter::updateRobotState(const robot_comm::RobotState &state)
{
    m_robotState = state;
    if (!m_runRecorder.recordRobotState(state, m_controller.state().phase)) {
        reportRecordingFailure(QStringLiteral("机器人实际路径"));
    }
    publishUpdate(m_controller.onRobotStateChanged(state));
}

void COnlineCorrectionWorkerAdapter::handleRobotCommandReceipt(
    const robot_comm::RobotCommandReceipt &receipt,
    const robot_comm::RobotState &state)
{
    if (receipt.type != robot_comm::RobotCommandType::TeachPointUpdate
        && receipt.type != robot_comm::RobotCommandType::RealtimeCorrection) {
        return;
    }
    if (!m_runRecorder.recordCommandReceipt(
            receipt, state, m_controller.state().phase)) {
        reportRecordingFailure(QStringLiteral("机器人命令回执"));
    }
    publishUpdate(m_controller.onRobotCommandReceipt(receipt, state));
}

void COnlineCorrectionWorkerAdapter::appendTrackingResult(
    const RuntimeTrackingResult &result)
{
    if (!m_runRecorder.recordTrackingResult(result, m_controller.state().phase)) {
        reportRecordingFailure(QStringLiteral("视觉与实时修正输入"));
    }
    if (!result.valid || !result.sample.point.valid) {
        emit statusChanged(QStringLiteral("tracking result ignored: %1").arg(result.message));
        return;
    }

    control::OnlineCorrectionDetectionEvent event;
    control::OnlineCorrectionDetectionSample sample = result.sample;
    sample.simulatedSeamIndex = result.simulatedSeamIndex;
    if (result.scanLineChangePoint.has_value()) {
        sample.scanLineChangePoint = result.scanLineChangePoint;
    }
    event.samples.push_back(sample);
    // 初始检测结束由机器人 D 回包中的 onlineReverseCompleteFlag 决定：
    // 反向扫描走完或 IBVS 到达起始点时置 1，Vision 不自行结束该阶段。
    event.detectionFinished = false;
    event.message = result.message.toStdString();
    const control::OnlineCorrectionUpdate update = m_controller.appendDetectionEvent(event);
    publishUpdate(update);
}

void COnlineCorrectionWorkerAdapter::finishOnlineSegment()
{
    publishUpdate(m_controller.finishOnlineSegment());
}

void COnlineCorrectionWorkerAdapter::failSynchronization(const QString &message)
{
    publishUpdate(m_controller.fail(message.toStdString()));
}

void COnlineCorrectionWorkerAdapter::reset()
{
    if (!m_runRecorder.finishSession(
            QStringLiteral("reset"),
            QStringLiteral("在线纠偏在终态前被复位。"))) {
        reportRecordingFailure(QStringLiteral("会话复位状态"));
    }
    publishUpdate(m_controller.reset());
}

void COnlineCorrectionWorkerAdapter::publishUpdate(
    const control::OnlineCorrectionUpdate &update)
{
    if (update.phase != m_timeoutPhase) {
        m_timeoutPhase = update.phase;
        armPhaseTimeout(update.phase);
    }
    if (!m_runRecorder.recordUpdate(update, m_positionerCalibration)) {
        reportRecordingFailure(QStringLiteral("在线纠偏计算结果"));
    }
    emit onlineCorrectionUpdated(update);
    if (update.command.valid) {
        emit robotCommandReady(update.command);
    }
    if (update.realtimeCommand.valid) {
        emit realtimeCommandReady(update.realtimeCommand);
    }
    if (update.startTracking) {
        emit startTrackingRequested();
    }
    if (update.stopTracking) {
        emit stopTrackingRequested();
    }
    if (update.requestNextSegment) {
        RuntimeOnlineCorrectionConfiguration next = m_configuration;
        next.trajectoryRequest.segmentIndex = update.state.segmentIndex;
        next.trajectoryRequest.startTeachIndex = -1;
        next.trajectoryRequest.endTeachIndex = -1;
        configure(next);
        publishUpdate(m_controller.startInitialDetection());
    }

    if (update.phase == control::OnlineCorrectionPhase::Completed
        || update.phase == control::OnlineCorrectionPhase::Error) {
        const QString resultText =
            update.phase == control::OnlineCorrectionPhase::Completed
                ? QStringLiteral("completed")
                : QStringLiteral("error");
        if (!m_runRecorder.finishSession(
                resultText,
                QString::fromStdString(update.message))) {
            reportRecordingFailure(QStringLiteral("会话最终状态"));
        }
    }
}

void COnlineCorrectionWorkerAdapter::armPhaseTimeout(
    const control::OnlineCorrectionPhase phase)
{
    ++m_phaseTimeoutGeneration;
    int timeoutMs = 0;
    QString waitDescription;
    switch (phase) {
    case control::OnlineCorrectionPhase::InitialDetectionRunning:
    case control::OnlineCorrectionPhase::WaitingForReverseCompletion:
        timeoutMs = kOnlineInitialDetectionTimeoutMs;
        waitDescription = QStringLiteral("反向检测完成信号");
        break;
    case control::OnlineCorrectionPhase::SendingInitialTeachPoints:
    case control::OnlineCorrectionPhase::WaitingInitialTeachPointAck:
        timeoutMs = kOnlineTeachPointTimeoutMs;
        waitDescription = QStringLiteral("O(P001)写入确认");
        break;
    case control::OnlineCorrectionPhase::WaitingForInitialApproach:
        timeoutMs = kOnlineInitialApproachTimeoutMs;
        waitDescription = QStringLiteral("P001后的ENDDETPRG");
        break;
    case control::OnlineCorrectionPhase::WaitingForOnlineStart:
        timeoutMs = kOnlineStartTimeoutMs;
        waitDescription = QStringLiteral("STARTCOR");
        break;
    case control::OnlineCorrectionPhase::SegmentFinishing:
    case control::OnlineCorrectionPhase::WaitingForDetectionReset:
        timeoutMs = kOnlineFinishHandshakeTimeoutMs;
        waitDescription = QStringLiteral("在线段结束握手");
        break;
    case control::OnlineCorrectionPhase::WaitingForRobotProgramStop:
        timeoutMs = kOnlineProgramStopTimeoutMs;
        waitDescription = QStringLiteral("机器人主程序停止");
        break;
    default:
        return;
    }

    const quint64 generation = m_phaseTimeoutGeneration;
    QTimer::singleShot(
        timeoutMs,
        this,
        [this, generation, phase, timeoutMs, waitDescription]() {
            if (generation != m_phaseTimeoutGeneration
                || m_controller.state().phase != phase) {
                return;
            }
            const QString message = QStringLiteral(
                "在线纠偏超时：等待%1超过%2秒。"
                "当前sStart=%3, corflag=%4, detflag=%5, offline=%6, JBI行=%7。")
                                        .arg(waitDescription)
                                        .arg(timeoutMs / 1000)
                                        .arg(m_robotState.sStart)
                                        .arg(m_robotState.corflag)
                                        .arg(m_robotState.detflag)
                                        .arg(m_robotState.offline)
                                        .arg(m_robotState.usJobLine);
            failSynchronization(message);
        });
}

void COnlineCorrectionWorkerAdapter::reportRecordingFailure(
    const QString &context)
{
    emit statusChanged(QStringLiteral("在线纠偏记录失败（%1）：%2")
                           .arg(context, m_runRecorder.lastError()));
}

COfflineCorrectionWorkerAdapter::COfflineCorrectionWorkerAdapter(QObject *parent)
    : QObject(parent)
{
    m_controller.configure(control::OfflineCorrectionOptions{});
}

void COfflineCorrectionWorkerAdapter::configure(
    const std::string &archiveFilePath,
    const control::OfflineCorrectionOptions &options)
{
    m_waitingForRobotAck = false;
    m_preflightReady = false;
    m_scanProgramStarted = false;
    m_scanProgramRunning = false;
    m_offlineRunReleased = false;
    m_offlineRunEndLatched = false;
    m_offlineRunAcknowledged = false;
    m_finishRequested = false;
    m_allTeachPointCommandsCompleted = false;
    m_expectedTeachPointCount = 0;
    ++m_jobStopGraceGeneration;
    ++m_robotAckGeneration;
    m_lastRobotState = {};
    m_diagnosticOutputDirectory =
        QString::fromStdString(options.diagnosticOutputDirectory);
    emit offlineStartCountChanged(0);
    m_controller.configure(options);
    publishUpdate(m_controller.selectArchiveFile(archiveFilePath));
}

void COfflineCorrectionWorkerAdapter::startDetection(const int detType)
{
    if (detType != 2) {
        publishUpdate(m_controller.fail(
            "offline correction currently supports scan/KCF det_type=2 only"));
        return;
    }

    const auto update = m_controller.startDetection();
    if (update.succeeded) {
        saveDetectionPoints();
    }
    // 清除上一轮扫描、OfflineRun 和 O 命令状态。先从后续 D 帧确认
    // sStart==0、B040==0，再启动 Vision；J 命令只能在 Vision 回报就绪后发送。
    // posflag 可能停留在上一轮定位段，不能把它作为启动阻断条件。
    m_waitingForRobotAck = false;
    m_preflightReady = false;
    m_scanProgramStarted = false;
    m_scanProgramRunning = false;
    m_offlineRunReleased = false;
    m_offlineRunEndLatched = false;
    m_offlineRunAcknowledged = false;
    m_finishRequested = false;
    m_allTeachPointCommandsCompleted = false;
    m_expectedTeachPointCount = 0;
    ++m_jobStopGraceGeneration;
    ++m_robotAckGeneration;
    emit offlineStartCountChanged(0);
    publishUpdate(update);
    if (update.succeeded) {
        emit statusChanged(QStringLiteral(
            "offline correction preflight: waiting for sStart=0 and B040=0"));
    }
}

void COfflineCorrectionWorkerAdapter::finishDetectionAndCompute()
{
    const auto phase = m_controller.state().phase;
    if (m_waitingForRobotAck
        || phase == control::OfflineCorrectionPhase::ReadyToSend
        || phase == control::OfflineCorrectionPhase::WaitingForRobotAck) {
        emit statusChanged(QStringLiteral("offline correction finish ignored: command is already being sent or waiting for D acknowledgement"));
        return;
    }

    if (!m_scanProgramStarted) {
        emit statusChanged(QStringLiteral(
            "offline correction finish ignored: current scan JBI has not entered sStart=1"));
        return;
    }
    if (m_scanProgramRunning) {
        m_finishRequested = true;
        emit statusChanged(QStringLiteral(
            "offline correction finish deferred: waiting for current scan JBI to stop"));
        return;
    }

    m_finishRequested = false;
    m_offlineRunEndLatched = true;
    emit stopTrackingRequested();

    const auto finishUpdate = m_controller.finishDetection();
    publishUpdate(finishUpdate);
    if (!finishUpdate.succeeded) {
        return;
    }

    const auto computeUpdate = m_controller.computeCorrection();
    if (computeUpdate.succeeded && computeUpdate.robotCommand.valid) {
        m_waitingForRobotAck = true;
        m_allTeachPointCommandsCompleted = false;
        m_expectedTeachPointCount = computeUpdate.robotCommand.pointCount;
    }
    publishUpdate(computeUpdate);
    if (computeUpdate.succeeded && computeUpdate.robotCommand.valid
        && m_waitingForRobotAck
        && m_controller.state().phase == control::OfflineCorrectionPhase::ReadyToSend) {
        publishUpdate(m_controller.markCommandDispatched());
    }
}

void COfflineCorrectionWorkerAdapter::updateRobotState(
    const robot_comm::RobotState &state)
{
    m_lastRobotState = state;
    if (m_offlineRunAcknowledged && state.OfflineRun == 0) {
        m_offlineRunAcknowledged = false;
        emit offlineStartCountChanged(0);
    }

    if (m_waitingForRobotAck
        && m_allTeachPointCommandsCompleted
        && state.offline == 5) {
        m_waitingForRobotAck = false;
        m_allTeachPointCommandsCompleted = false;
        m_expectedTeachPointCount = 0;
        ++m_robotAckGeneration;
        completeSuccessfulCorrection();
        return;
    }

    const auto phase = m_controller.state().phase;
    if (phase != control::OfflineCorrectionPhase::WaitingForDetection
        && phase != control::OfflineCorrectionPhase::Detecting) {
        return;
    }

    if (!m_preflightReady) {
        if (state.sStart != 0) {
            emit offlineStartCountChanged(0);
            emit stopTrackingRequested();
            const QString message = QStringLiteral(
                "离线纠偏预检失败：准备检测期间机器人已运行，"
                "sStart=%1, B040=%2, JBI行=%3。")
                                        .arg(state.sStart)
                                        .arg(state.OfflineRun)
                                        .arg(state.usJobLine);
            publishUpdate(m_controller.fail(message.toStdString()));
            return;
        }
        if (state.OfflineRun != 0) {
            if (!m_offlineRunAcknowledged) {
                m_offlineRunAcknowledged = true;
                emit offlineStartCountChanged(1);
                emit statusChanged(QStringLiteral(
                    "offline correction preflight: clearing stale B040=1"));
                const quint64 generation = ++m_jobStopGraceGeneration;
                QTimer::singleShot(
                    kOfflinePreflightTimeoutMs,
                    this,
                    [this, generation]() {
                        const auto currentPhase = m_controller.state().phase;
                        if (generation != m_jobStopGraceGeneration
                            || m_preflightReady
                            || m_lastRobotState.OfflineRun == 0
                            || (currentPhase
                                    != control::OfflineCorrectionPhase::WaitingForDetection
                                && currentPhase
                                    != control::OfflineCorrectionPhase::Detecting)) {
                            return;
                        }

                        m_offlineRunAcknowledged = false;
                        emit offlineStartCountChanged(0);
                        emit stopTrackingRequested();
                        const QString message = QStringLiteral(
                            "离线纠偏预检失败：已请求复位B040，但3秒内仍未变为0。"
                            "当前sStart=%1, B040=%2, JBI行=%3。")
                                                    .arg(m_lastRobotState.sStart)
                                                    .arg(m_lastRobotState.OfflineRun)
                                                    .arg(m_lastRobotState.usJobLine);
                        publishUpdate(m_controller.fail(message.toStdString()));
                    });
            }
            return;
        }

        m_preflightReady = true;
        m_offlineRunReleased = true;
        ++m_jobStopGraceGeneration;
        emit statusChanged(QStringLiteral(
            "offline correction preflight passed: sStart=0, B040=0; arming vision"));
        emit startTrackingRequested();
    }

    if (state.sStart == 1) {
        if (!m_scanProgramStarted) {
            emit statusChanged(QStringLiteral(
                "offline correction scan JBI entered sStart=1"));
        }
        m_scanProgramStarted = true;
        m_scanProgramRunning = true;
    } else if (m_scanProgramStarted && m_scanProgramRunning) {
        m_scanProgramRunning = false;
        if (!m_finishRequested && !m_offlineRunEndLatched) {
            const quint64 generation = ++m_jobStopGraceGeneration;
            QTimer::singleShot(
                kOfflineB040GraceMs,
                this,
                [this, generation]() {
                    const auto currentPhase = m_controller.state().phase;
                    if (generation != m_jobStopGraceGeneration
                        || m_finishRequested
                        || m_offlineRunEndLatched
                        || m_scanProgramRunning
                        || (currentPhase
                                != control::OfflineCorrectionPhase::WaitingForDetection
                            && currentPhase
                                != control::OfflineCorrectionPhase::Detecting)) {
                        return;
                    }

                    emit stopTrackingRequested();
                    const QString message = QStringLiteral(
                        "JBI已结束，但2秒内未检测到B040完成信号："
                        "sStart=%1, traflag=%2, posflag=%3, B040=%4, JBI行=%5。")
                                                .arg(m_lastRobotState.sStart)
                                                .arg(m_lastRobotState.traflag)
                                                .arg(m_lastRobotState.posflag)
                                                .arg(m_lastRobotState.OfflineRun)
                                                .arg(m_lastRobotState.usJobLine);
                    publishUpdate(m_controller.fail(message.toStdString()));
                });
        }
    }

    if (!m_scanProgramStarted) {
        return;
    }

    if (state.OfflineRun == 1
               && m_offlineRunReleased
               && !m_offlineRunEndLatched) {
        m_offlineRunEndLatched = true;
        m_offlineRunAcknowledged = true;
        m_finishRequested = true;
        ++m_jobStopGraceGeneration;
        emit offlineStartCountChanged(1);
        emit statusChanged(QStringLiteral(
            "offline correction received current OfflineRun trigger; waiting for scan JBI to stop"));
    }

    if (m_finishRequested && !m_scanProgramRunning) {
        finishDetectionAndCompute();
    }
}

void COfflineCorrectionWorkerAdapter::appendTrackingResult(
    const RuntimeTrackingResult &result)
{
    const auto phase = m_controller.state().phase;
    if (phase != control::OfflineCorrectionPhase::WaitingForDetection
        && phase != control::OfflineCorrectionPhase::Detecting) {
        return;
    }
    // 专用 JBI 只扫描焊缝，按结果所属帧的检测窗口接收，不依赖定位段编号。
    if (!m_preflightReady
        || !m_scanProgramStarted
        || result.traflag != 1) {
        return;
    }
    if (!result.valid || !result.sample.point.valid) {
        emit statusChanged(QStringLiteral("offline tracking result ignored: %1")
                               .arg(result.message));
        return;
    }

    control::OfflineCorrectionDetectionSample sample;
    sample.point.xMm = result.sample.point.xMm;
    sample.point.yMm = result.sample.point.yMm;
    sample.point.zMm = result.sample.point.zMm;
    sample.point.valid = result.sample.point.valid;
    sample.imageXPixel = result.imageXPixel;
    sample.imageYPixel = result.imageYPixel;
    sample.imagePointValid = std::isfinite(result.imageXPixel)
        && std::isfinite(result.imageYPixel);
    sample.frameId = result.frameId;

    control::OfflineCorrectionDetectionEvent event;
    event.samples.push_back(sample);
    event.message = result.message.toStdString();
    const auto update = m_controller.appendDetectionEvent(event);
    if (update.succeeded) {
        saveDetectionPoints();
    }
    publishUpdate(update);
}

void COfflineCorrectionWorkerAdapter::failSynchronization(const QString &message)
{
    // 同步失败属于检测链的终止错误，而不是一次可发送的纠偏计算结果。
    // 先关闭本 worker 的等待状态，再通知 Vision/RobotComm 停止，最后发布
    // controller Error；COfflineCorrectionController::fail 不会携带 O 命令。
    m_waitingForRobotAck = false;
    m_scanProgramRunning = false;
    m_offlineRunEndLatched = true;
    m_offlineRunAcknowledged = false;
    m_finishRequested = false;
    m_allTeachPointCommandsCompleted = false;
    m_expectedTeachPointCount = 0;
    ++m_jobStopGraceGeneration;
    ++m_robotAckGeneration;
    emit offlineStartCountChanged(0);
    emit stopTrackingRequested();
    publishUpdate(m_controller.fail(message.toStdString()));
}

void COfflineCorrectionWorkerAdapter::handleRobotCommandReceipt(
    const robot_comm::RobotCommandReceipt &receipt,
    const robot_comm::RobotState &state)
{
    m_lastRobotState = state;
    if (receipt.type != robot_comm::RobotCommandType::TeachPointUpdate
        || !m_waitingForRobotAck
        || receipt.pointIndex < 0
        || receipt.pointIndex >= m_expectedTeachPointCount) {
        return;
    }

    if (receipt.error != robot_comm::RobotCommandError::None
        || !receipt.sent
        || !receipt.responseReceived) {
        m_waitingForRobotAck = false;
        m_allTeachPointCommandsCompleted = false;
        m_expectedTeachPointCount = 0;
        ++m_robotAckGeneration;
        publishUpdate(m_controller.fail(
            "offline correction robot command failed: " + receipt.message));
        return;
    }

    emit statusChanged(QStringLiteral("offline correction O progress: %1/%2")
                           .arg(receipt.pointIndex + 1)
                           .arg(m_expectedTeachPointCount));

    const bool finalPointReceipt = receipt.pointIndex == m_expectedTeachPointCount - 1;
    if (!finalPointReceipt) {
        return;
    }

    m_allTeachPointCommandsCompleted = true;
    if (state.offline == 5) {
        m_waitingForRobotAck = false;
        m_allTeachPointCommandsCompleted = false;
        m_expectedTeachPointCount = 0;
        ++m_robotAckGeneration;
        completeSuccessfulCorrection();
        return;
    }

    emit statusChanged(QStringLiteral(
        "offline correction all O points sent; waiting for D.offline=5"));
    const quint64 generation = ++m_robotAckGeneration;
    QTimer::singleShot(
        kOfflineRobotAckTimeoutMs,
        this,
        [this, generation]() {
            if (generation != m_robotAckGeneration
                || !m_waitingForRobotAck
                || !m_allTeachPointCommandsCompleted) {
                return;
            }

            m_waitingForRobotAck = false;
            m_allTeachPointCommandsCompleted = false;
            m_expectedTeachPointCount = 0;
            const QString message = QStringLiteral(
                "O点已全部发送，但10秒内未收到D.offline=5："
                "当前offline=%1, B040=%2, JBI行=%3。")
                                        .arg(m_lastRobotState.offline)
                                        .arg(m_lastRobotState.OfflineRun)
                                        .arg(m_lastRobotState.usJobLine);
            publishUpdate(m_controller.fail(message.toStdString()));
        });
}

void COfflineCorrectionWorkerAdapter::reset()
{
    m_waitingForRobotAck = false;
    m_preflightReady = false;
    m_scanProgramStarted = false;
    m_scanProgramRunning = false;
    m_offlineRunReleased = false;
    m_offlineRunEndLatched = false;
    m_offlineRunAcknowledged = false;
    m_finishRequested = false;
    m_allTeachPointCommandsCompleted = false;
    m_expectedTeachPointCount = 0;
    ++m_jobStopGraceGeneration;
    ++m_robotAckGeneration;
    m_lastRobotState = {};
    emit offlineStartCountChanged(0);
    emit stopTrackingRequested();
    publishUpdate(m_controller.reset());
}

void COfflineCorrectionWorkerAdapter::saveDetectionPoints()
{
    if (m_diagnosticOutputDirectory.trimmed().isEmpty()) {
        return;
    }
    // 直接保存采集样本，使同步/匹配失败和用户 reset 不会丢失本轮已检测的点。
    std::vector<vision::WeldSpacePoint> points;
    const auto &samples = m_controller.state().detectionSamples;
    points.reserve(samples.size());
    for (const auto &sample : samples) {
        points.push_back(sample.point);
    }

    QString errorMessage;
    if (!QDir().mkpath(m_diagnosticOutputDirectory)) {
        errorMessage = QStringLiteral("无法创建诊断输出目录：%1")
                           .arg(m_diagnosticOutputDirectory);
    } else if (writeOfflineCorrectionTextFile(
                   QDir(m_diagnosticOutputDirectory).filePath(
                       QStringLiteral("detection_points.txt")),
                   offlineCorrectionPointsText(points),
                   &errorMessage)) {
        return;
    }
    emit statusChanged(QStringLiteral("离线检测点保存失败：%1")
                           .arg(errorMessage));
}

void COfflineCorrectionWorkerAdapter::completeSuccessfulCorrection()
{
    const auto update = m_controller.markCommandSent();
    QString errorMessage;
    if (update.succeeded
        && saveOfflineCorrectionDiagnostics(
            m_diagnosticOutputDirectory,
            m_controller.diagnostics(),
            m_controller.state().report,
            &errorMessage)) {
        emit statusChanged(QStringLiteral(
            "离线纠偏诊断已覆盖保存：%1")
                               .arg(m_diagnosticOutputDirectory));
    } else if (update.succeeded) {
        emit statusChanged(QStringLiteral(
            "离线纠偏已完成，但诊断保存失败：%1")
                               .arg(errorMessage));
    }
    publishUpdate(update);
}

void COfflineCorrectionWorkerAdapter::publishUpdate(
    const control::OfflineCorrectionUpdate &update)
{
    emit offlineCorrectionUpdated(update);
    emit statusChanged(QString::fromStdString(update.message));
    if (update.phase == control::OfflineCorrectionPhase::ReadyToSend
        && update.robotCommand.valid) {
        emit robotCommandReady(update.robotCommand);
    }
}

CTeachTrajectoryRecorderWorkerAdapter::CTeachTrajectoryRecorderWorkerAdapter(
    QObject *parent)
    : QObject(parent)
{
}

void CTeachTrajectoryRecorderWorkerAdapter::configureTempDirectory(
    const QString &tempDirectory)
{
    publishSnapshot(
        m_recorder.configureTempDirectory(tempDirectory.toStdString()));
}

void CTeachTrajectoryRecorderWorkerAdapter::startRecording()
{
    publishSnapshot(m_recorder.startRecording());
}

void CTeachTrajectoryRecorderWorkerAdapter::updateRobotState(
    const robot_comm::RobotState &state)
{
    const auto snapshot = m_recorder.onRobotStateChanged(state);
    if (shouldPublishRobotStateSnapshot(snapshot)) {
        publishSnapshot(snapshot);
    }
}

void CTeachTrajectoryRecorderWorkerAdapter::saveCapturedTrajectory(
    const QString &outputDirectory,
    const QString &baseName)
{
    control::TeachTrajectorySaveRequest request;
    request.outputDirectory = outputDirectory.toStdString();
    request.baseName = baseName.toStdString();
    publishSnapshot(m_recorder.saveCapturedTrajectory(request));
}

void CTeachTrajectoryRecorderWorkerAdapter::reset()
{
    publishSnapshot(m_recorder.reset());
}

void CTeachTrajectoryRecorderWorkerAdapter::publishSnapshot(
    const control::TeachTrajectoryRecorderSnapshot &snapshot)
{
    m_lastPublishedSaveSignal = snapshot.latestSaveSignal;
    m_lastPublishedSegmentActive = snapshot.segmentActive;
    m_lastPublishedTrajectoryPointCount = snapshot.trajectoryPointCount;
    m_lastPublishedTeachPointCount = snapshot.teachPointCount;
    m_lastPublishedAllPointCount = snapshot.allPointCount;
    emit teachTrajectoryRecorderUpdated(snapshot);
    emit statusChanged(QString::fromStdString(snapshot.message));
}

bool CTeachTrajectoryRecorderWorkerAdapter::shouldPublishRobotStateSnapshot(
    const control::TeachTrajectoryRecorderSnapshot &snapshot)
{
    return !snapshot.succeeded
        || snapshot.latestSaveSignal != m_lastPublishedSaveSignal
        || snapshot.segmentActive != m_lastPublishedSegmentActive
        || snapshot.trajectoryPointCount != m_lastPublishedTrajectoryPointCount
        || snapshot.teachPointCount != m_lastPublishedTeachPointCount
        || snapshot.allPointCount != m_lastPublishedAllPointCount;
}

} // namespace i2r_rlc::ui
