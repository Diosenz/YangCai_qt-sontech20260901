/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        UiController.cpp
 * @brief       UI 控制器实现
 * @details     编排 UI 事件与相机、控制、视觉、定位、数字孪生服务之间的数据流。
 ******************************************************************************/
#include <algorithm>
#include <utility>

#ifdef I2R_RLC_HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QImage>
#include <QStringList>
#include <QTimer>

#include "AppPaths.h"
#include "CalibrationGuideOverlay.h"
#include "CorrectionRobotStartGate.h"
#include "DigitalTwinVisionTypes.h"
#include "Logger.h"
#include "MainWidget.h"
#include "RobotSimEndpointConfigReader.h"
#include "RobotSimSceneConfigReader.h"
#include "TrackingFramePresentation.h"
#include "UiController.h"
#include "ibvs/IbvsTextConfigReader.h"

#ifdef I2R_RLC_HAS_OPENCV
#include "pipeline/WeldVisionAlgorithmPipeline.h"
#endif

namespace i2r_rlc::ui {

namespace {

constexpr int AUTOMATIC_CALIBRATION_LASER_CAPTURE_DELAY_MS = 2800;

enum class UiDetectionStatus {
    NotRun,
    Succeeded,
    NoWeldFound,
    LowConfidence,
    InvalidInput,
    AlgorithmUnavailable,
    Error
};

struct UiPixelPoint {
    double xPixel = 0.0;
    double yPixel = 0.0;
    bool isValid = false;
};

struct UiCenterline {
    std::vector<UiPixelPoint> points;
    bool isValid = false;
};

struct UiDetectionResult {
    qint64 frameId = -1;
    QString sourcePath;
    UiDetectionStatus status = UiDetectionStatus::NotRun;
    QString message;
    UiPixelPoint weldPointPixel;
    UiCenterline centerline;
    double confidence = 0.0;
    double processingTimeMs = 0.0;
};

struct UiFrameProcessingResult {
    qint64 frameId = -1;
    QString sourcePath;
    int imageWidthPixel = 0;
    int imageHeightPixel = 0;
    bool processSucceeded = false;
    double processingTimeMs = 0.0;
    bool preprocessingSucceeded = false;
    int imageChannels = 0;
    double meanGrayValue = 0.0;
    double minGrayValue = 0.0;
    double maxGrayValue = 0.0;
    QString message;
    UiDetectionResult weldDetection;
};

QString detectionStatusToKey(UiDetectionStatus status)
{
    switch (status) {
    case UiDetectionStatus::NotRun:
        return QStringLiteral("NotRun");
    case UiDetectionStatus::Succeeded:
        return QStringLiteral("Succeeded");
    case UiDetectionStatus::NoWeldFound:
        return QStringLiteral("NoWeldFound");
    case UiDetectionStatus::LowConfidence:
        return QStringLiteral("LowConfidence");
    case UiDetectionStatus::InvalidInput:
        return QStringLiteral("InvalidInput");
    case UiDetectionStatus::AlgorithmUnavailable:
        return QStringLiteral("AlgorithmUnavailable");
    case UiDetectionStatus::Error:
        return QStringLiteral("Error");
    }
    return QStringLiteral("Unknown");
}

QString localizationModeToText(const i2r_rlc::localization::LocalizationMode mode)
{
    switch (mode) {
    case i2r_rlc::localization::LocalizationMode::Global:
        return QStringLiteral("全局 ICP");
    case i2r_rlc::localization::LocalizationMode::Local:
        return QStringLiteral("局部修正");
    }
    return QStringLiteral("未知");
}

QString dofMaskToText(const i2r_rlc::localization::IcpDofMask& mask)
{
    QStringList enabled;
    if (mask.allowX) {
        enabled.push_back(QStringLiteral("x"));
    }
    if (mask.allowY) {
        enabled.push_back(QStringLiteral("y"));
    }
    if (mask.allowZ) {
        enabled.push_back(QStringLiteral("z"));
    }
    if (mask.allowRx) {
        enabled.push_back(QStringLiteral("rx"));
    }
    if (mask.allowRy) {
        enabled.push_back(QStringLiteral("ry"));
    }
    if (mask.allowRz) {
        enabled.push_back(QStringLiteral("rz"));
    }
    return enabled.isEmpty() ? QStringLiteral("<none>") : enabled.join(QStringLiteral("/"));
}

UiDetectionResult makeDetectionResult(
    const i2r_rlc::camera::CameraFrame& frame,
    UiDetectionStatus status,
    const QString& message)
{
    UiDetectionResult result;
    result.frameId = frame.frameId;
    result.sourcePath = frame.sourcePath;
    result.status = status;
    result.message = message;
    return result;
}

#ifdef I2R_RLC_HAS_OPENCV
void setError(QString* errorMessage, const QString& text)
{
    if (errorMessage) {
        *errorMessage = text;
    }
}

cv::Mat cloneContiguousMat(const QImage& image, int cvType)
{
    const cv::Mat wrapped(image.height(),
                          image.width(),
                          cvType,
                          const_cast<uchar*>(image.constBits()),
                          static_cast<size_t>(image.bytesPerLine()));
    return wrapped.clone();
}

cv::Mat qImageToCvMat(const QImage& image, QString* errorMessage)
{
    if (image.isNull()) {
        setError(errorMessage, QStringLiteral("QImage 为空，无法转换为 cv::Mat。"));
        return {};
    }

    switch (image.format()) {
    case QImage::Format_Grayscale8:
        return cloneContiguousMat(image, CV_8UC1);
    case QImage::Format_RGB888: {
        const cv::Mat rgb = cloneContiguousMat(image, CV_8UC3);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    case QImage::Format_BGR888:
        return cloneContiguousMat(image, CV_8UC3);
#endif
    case QImage::Format_RGBA8888: {
        const cv::Mat rgba = cloneContiguousMat(image, CV_8UC4);
        cv::Mat bgra;
        cv::cvtColor(rgba, bgra, cv::COLOR_RGBA2BGRA);
        return bgra;
    }
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
    case QImage::Format_RGB32: {
        const QImage converted = image.convertToFormat(QImage::Format_RGBA8888);
        return qImageToCvMat(converted, errorMessage);
    }
    default: {
        const QImage converted = image.convertToFormat(QImage::Format_RGBA8888);
        if (converted.isNull()) {
            setError(errorMessage, QStringLiteral("QImage 格式转换失败。"));
            return {};
        }
        return qImageToCvMat(converted, errorMessage);
    }
    }
}

cv::Mat toGray8(const cv::Mat& image, QString* errorMessage)
{
    if (image.empty()) {
        setError(errorMessage, QStringLiteral("cv::Mat 为空，无法灰度化。"));
        return {};
    }
    if (image.channels() == 1) {
        return image.clone();
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        return gray;
    }

    setError(errorMessage, QStringLiteral("不支持的 OpenCV 图像通道数：%1。").arg(image.channels()));
    return {};
}

UiDetectionStatus mapPipelineStatus(
    i2r_rlc::vision::WeldVisionPipelineStatus status)
{
    using i2r_rlc::vision::WeldVisionPipelineStatus;
    switch (status) {
    case WeldVisionPipelineStatus::Succeeded:
        return UiDetectionStatus::Succeeded;
    case WeldVisionPipelineStatus::NoWeldFound:
        return UiDetectionStatus::NoWeldFound;
    case WeldVisionPipelineStatus::LowConfidence:
        return UiDetectionStatus::LowConfidence;
    case WeldVisionPipelineStatus::InvalidInput:
        return UiDetectionStatus::InvalidInput;
    case WeldVisionPipelineStatus::UnsupportedTask:
        return UiDetectionStatus::AlgorithmUnavailable;
    case WeldVisionPipelineStatus::Error:
        return UiDetectionStatus::Error;
    case WeldVisionPipelineStatus::NotRun:
        return UiDetectionStatus::NotRun;
    }
    return UiDetectionStatus::Error;
}

UiPixelPoint toPixelPoint(const i2r_rlc::vision::WeldFeaturePoint& point)
{
    UiPixelPoint pixelPoint;
    pixelPoint.xPixel = point.xPixel;
    pixelPoint.yPixel = point.yPixel;
    pixelPoint.isValid = point.valid;
    return pixelPoint;
}

UiCenterline toCenterline(const std::vector<cv::Point2d>& points)
{
    UiCenterline centerline;
    centerline.points.reserve(points.size());
    for (const cv::Point2d& point : points) {
        UiPixelPoint pixelPoint;
        pixelPoint.xPixel = point.x;
        pixelPoint.yPixel = point.y;
        pixelPoint.isValid = true;
        centerline.points.push_back(pixelPoint);
    }
    centerline.isValid = !centerline.points.empty();
    return centerline;
}
#endif

QString onlineCorrectionPhaseToText(i2r_rlc::control::OnlineCorrectionPhase phase)
{
    switch (phase) {
    case i2r_rlc::control::OnlineCorrectionPhase::Idle:
        return QStringLiteral("空闲");
    case i2r_rlc::control::OnlineCorrectionPhase::InitialDetectionRunning:
        return QStringLiteral("初始检测中");
    case i2r_rlc::control::OnlineCorrectionPhase::WaitingForReverseCompletion:
        return QStringLiteral("等待反向检测完成");
    case i2r_rlc::control::OnlineCorrectionPhase::PreparingInitialTrajectory:
        return QStringLiteral("正在规划初始轨迹");
    case i2r_rlc::control::OnlineCorrectionPhase::SendingInitialTeachPoints:
        return QStringLiteral("正在发送示教点");
    case i2r_rlc::control::OnlineCorrectionPhase::WaitingInitialTeachPointAck:
        return QStringLiteral("等待示教点确认");
    case i2r_rlc::control::OnlineCorrectionPhase::WaitingForInitialApproach:
        return QStringLiteral("正在前往动态起点P001");
    case i2r_rlc::control::OnlineCorrectionPhase::WaitingForOnlineStart:
        return QStringLiteral("已到达P001，等待STARTCOR");
    case i2r_rlc::control::OnlineCorrectionPhase::OnlineRunning:
        return QStringLiteral("KCF在线纠偏中");
    case i2r_rlc::control::OnlineCorrectionPhase::SegmentFinishing:
        return QStringLiteral("当前分段收尾中");
    case i2r_rlc::control::OnlineCorrectionPhase::WaitingForDetectionReset:
        return QStringLiteral("等待检测复位");
    case i2r_rlc::control::OnlineCorrectionPhase::WaitingForRobotProgramStop:
        return QStringLiteral("纠偏结束，等待机器人程序停止");
    case i2r_rlc::control::OnlineCorrectionPhase::Completed:
        return QStringLiteral("已完成");
    case i2r_rlc::control::OnlineCorrectionPhase::Error:
        return QStringLiteral("错误");
    }

    return QStringLiteral("未知");
}

QString onlineCorrectionStartModeToText(
    i2r_rlc::control::OnlineCorrectionStartMode mode)
{
    switch (mode) {
    case i2r_rlc::control::OnlineCorrectionStartMode::InitType:
        return QStringLiteral("反向扫描");
    case i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection:
        return QStringLiteral("全局定位修正后反向扫描");
    case i2r_rlc::control::OnlineCorrectionStartMode::IBVSStart:
        return QStringLiteral("IBVS");
    }
    return QStringLiteral("反向扫描");
}

QString robotModeToLegacyText(int mode)
{
    switch (mode) {
    case 1:
        return QStringLiteral("TEACH");
    case 2:
        return QStringLiteral("PLAY");
    case 3:
        return QStringLiteral("REMOTE");
    default:
        return QStringLiteral("UNKNOWN");
    }
}

QString servoToLegacyText(int servoPower)
{
    return servoPower == 1
        ? QStringLiteral("SERVO ON")
        : QStringLiteral("SERVO OFF");
}

QString runToLegacyText(int start)
{
    return start == 1
        ? QStringLiteral("ROBOT EXE!")
        : QStringLiteral("ROBOTS STOP!");
}

QString correctionToLegacyText(int corflag)
{
    return corflag == 1
        ? QStringLiteral("WELDING")
        : QStringLiteral("WAIT");
}

QString robotDeviceText(
    const i2r_rlc::robot_comm::RobotCommunicationDiagnostics& diagnostics)
{
    return diagnostics.validDReceived
        ? QStringLiteral("机器人已连接")
        : QStringLiteral("机器人未连接");
}

QString robotDeviceText(bool validDReceived)
{
    return validDReceived
        ? QStringLiteral("机器人已连接")
        : QStringLiteral("机器人未连接");
}

bool autoDeviceStartDisabled()
{
    return qEnvironmentVariableIsSet("I2R_RLC_DISABLE_AUTO_DEVICE_START");
}

QString legacyTeachTrajectorySaveFailurePrompt(const QString& message)
{
    if (message.contains(QStringLiteral("已存在"))
        || message.contains(QStringLiteral("导入失败"))) {
        return QStringLiteral("文件已经存在，导入失败!");
    }
    if (message.contains(QStringLiteral("为空"))
        || message.contains(QStringLiteral("请先运行"))
        || message.contains(QStringLiteral("示教程序"))) {
        return QStringLiteral("请运行示教程序。");
    }
    return message.isEmpty()
        ? QStringLiteral("保存示教轨迹失败。")
        : message;
}

QString legacyOfflineFailurePrompt(const QString& message)
{
    if (message.contains(QStringLiteral("检测"))) {
        return QStringLiteral("检测错误，请重新检测！");
    }
    return QStringLiteral("数据错误，请检查相关设置是否正确并重新检测!");
}

i2r_rlc::control::OnlineCorrectionStartMode onlineCorrectionStartModeFromIndex(
    int modeIndex,
    bool* valid)
{
    if (valid) {
        *valid = true;
    }
    switch (modeIndex) {
    case 0:
        return i2r_rlc::control::OnlineCorrectionStartMode::InitType;
    case 1:
        return i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection;
    case 2:
        return i2r_rlc::control::OnlineCorrectionStartMode::IBVSStart;
    default:
        if (valid) {
            *valid = false;
        }
        return i2r_rlc::control::OnlineCorrectionStartMode::InitType;
    }
}

QString onlineJbiForStartMode(
    const i2r_rlc::ui::WorkpieceProfile& profile,
    i2r_rlc::control::OnlineCorrectionStartMode startMode)
{
    switch (startMode) {
    case i2r_rlc::control::OnlineCorrectionStartMode::InitType:
        return profile.onlineJbi;
    case i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection:
        return profile.onlineGlobalCorrectionJbi;
    case i2r_rlc::control::OnlineCorrectionStartMode::IBVSStart:
        return profile.onlineIbvsStartJbi;
    }
    return profile.onlineJbi;
}

QString offlineCorrectionPhaseToText(i2r_rlc::control::OfflineCorrectionPhase phase)
{
    switch (phase) {
    case i2r_rlc::control::OfflineCorrectionPhase::Idle:
        return QStringLiteral("空闲");
    case i2r_rlc::control::OfflineCorrectionPhase::ArchiveSelected:
        return QStringLiteral("存档已选择");
    case i2r_rlc::control::OfflineCorrectionPhase::WaitingForDetection:
        return QStringLiteral("等待检测输入");
    case i2r_rlc::control::OfflineCorrectionPhase::Detecting:
        return QStringLiteral("检测点接收中");
    case i2r_rlc::control::OfflineCorrectionPhase::ReadyToCompute:
        return QStringLiteral("等待计算");
    case i2r_rlc::control::OfflineCorrectionPhase::Computing:
        return QStringLiteral("计算中");
    case i2r_rlc::control::OfflineCorrectionPhase::ReadyToSend:
        return QStringLiteral("待发送");
    case i2r_rlc::control::OfflineCorrectionPhase::WaitingForRobotAck:
        return QStringLiteral("等待机器人确认");
    case i2r_rlc::control::OfflineCorrectionPhase::Completed:
        return QStringLiteral("已完成");
    case i2r_rlc::control::OfflineCorrectionPhase::Error:
        return QStringLiteral("错误");
    }
    return QStringLiteral("未知");
}

QString correctionSyncStateToText(CorrectionSyncState state)
{
    switch (state) {
    case CorrectionSyncState::Idle:
        return QStringLiteral("等待相机帧");
    case CorrectionSyncState::Querying:
        return QStringLiteral("查询 RobotSim");
    case CorrectionSyncState::ObservationReady:
        return QStringLiteral("同帧观测已就绪");
    case CorrectionSyncState::VisionConsumed:
        return QStringLiteral("Vision 已确认消费");
    case CorrectionSyncState::Failed:
        return QStringLiteral("同步失败");
    case CorrectionSyncState::Cleared:
        return QStringLiteral("已清理");
    case CorrectionSyncState::StoppedAfterFailures:
        return QStringLiteral("连续 3 帧失败，已停止");
    case CorrectionSyncState::OfflineSeamWindowClosed:
        return QStringLiteral("焊缝采集已关闭，侧面扫描中");
    case CorrectionSyncState::TrackingPaused:
        return QStringLiteral("检测窗口已暂停");
    }
    return QStringLiteral("未知");
}

QString calibrationCapturePhaseToText(i2r_rlc::control::CalibrationCapturePhase phase)
{
    switch (phase) {
    case i2r_rlc::control::CalibrationCapturePhase::Idle:
        return QStringLiteral("空闲");
    case i2r_rlc::control::CalibrationCapturePhase::ManualCapturingBoard:
        return QStringLiteral("人工采集标定板");
    case i2r_rlc::control::CalibrationCapturePhase::ManualCapturingLaser:
        return QStringLiteral("人工采集激光图");
    case i2r_rlc::control::CalibrationCapturePhase::AutomaticGuidePreview:
        return QStringLiteral("自动标定红框确认");
    case i2r_rlc::control::CalibrationCapturePhase::AutomaticWaitingRobot:
        return QStringLiteral("自动采集等待机器人");
    case i2r_rlc::control::CalibrationCapturePhase::AutomaticCapturingBoard:
        return QStringLiteral("自动采集标定板");
    case i2r_rlc::control::CalibrationCapturePhase::AutomaticCapturingLaser:
        return QStringLiteral("自动采集激光图");
    case i2r_rlc::control::CalibrationCapturePhase::Completed:
        return QStringLiteral("已完成");
    case i2r_rlc::control::CalibrationCapturePhase::Error:
        return QStringLiteral("错误");
    }

    return QStringLiteral("未知");
}

QString findRepoConfigFile(const QString& fileName)
{
    QStringList roots;
    roots << QDir::currentPath();
    roots << QCoreApplication::applicationDirPath();

    QDir current(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 6; ++i) {
        current.cdUp();
        roots << current.absolutePath();
    }

    for (const QString& root : roots) {
        const QString candidate = QDir(root).absoluteFilePath(
            QStringLiteral("config/IBVS/%1").arg(fileName));
        if (QFileInfo::exists(candidate)) {
            return candidate;
        }
    }

    return QDir::current().absoluteFilePath(
        QStringLiteral("config/IBVS/%1").arg(fileName));
}

QString findRepoRelativeFile(const QString& relativePath)
{
    QStringList roots;
    roots << QDir::currentPath();
    roots << QCoreApplication::applicationDirPath();

    QDir current(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 6; ++i) {
        current.cdUp();
        roots << current.absolutePath();
    }

    for (const QString& root : roots) {
        const QString candidate = QDir(root).absoluteFilePath(relativePath);
        if (QFileInfo::exists(candidate)) {
            return candidate;
        }
    }

    return QDir::current().absoluteFilePath(relativePath);
}

QString ibvsServiceStateToText(i2r_rlc::ibvs::IbvsServiceState state)
{
    using i2r_rlc::ibvs::IbvsServiceState;
    switch (state) {
    case IbvsServiceState::Unavailable:
        return QStringLiteral("不可用");
    case IbvsServiceState::Idle:
        return QStringLiteral("空闲");
    case IbvsServiceState::Configured:
        return QStringLiteral("已配置");
    case IbvsServiceState::DryRunReady:
        return QStringLiteral("就绪");
    case IbvsServiceState::RunningDryRun:
        return QStringLiteral("运行中");
    case IbvsServiceState::Running:
        return QStringLiteral("运行中");
    case IbvsServiceState::Error:
        return QStringLiteral("错误");
    }

    return QStringLiteral("未知");
}

i2r_rlc::ibvs::IbvsOnlineFrameInput makeSimulatedIbvsFrame(qint64 frameId)
{
    using namespace i2r_rlc::ibvs;

    const double offset = static_cast<double>(std::max<qint64>(0, frameId)) * 0.1;
    IbvsTorchObservation torch;
    torch.featurePoints = { { 40.0 + offset, 20.0 }, { 16.0 + offset, 20.0 } };
    torch.confidence = 0.9;
    torch.frameIndex = static_cast<int>(frameId);
    torch.depthM = -1.0;

    IbvsLvsObservation lvs;
    lvs.startPoint = { 10.0 + offset, 10.0 };
    lvs.weldPoint = { 11.0 + offset, 12.0 };
    lvs.hasStartPoint = true;
    lvs.hasWeldPoint = true;
    lvs.confidence = 0.9;
    lvs.centerlineQuality = 0.8;
    lvs.frameIndex = static_cast<int>(frameId);

    IbvsLvsConstraint lvsConstraint;
    lvsConstraint.enabled = true;
    lvsConstraint.jacobian =
        IbvsUnifiedVelocitySolver::computeLvsJacobian(Eigen::Vector3d::UnitX());
    lvsConstraint.errorM << 0.0, 0.003, -0.002;
    lvsConstraint.lambdaD = 1.0;
    lvsConstraint.alphaLvs = 1.0;
    lvsConstraint.omegaLvs = 1.0;
    lvsConstraint.betaLvs = 1.0;

    IbvsOnlineFrameInput input;
    input.frameIndex = static_cast<int>(frameId);
    input.currentFeature = torch.featurePoints.toVector();
    input.fallbackDesiredFeature = IbvsFeatureVector::Zero();
    input.cameraJacobian << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 1.0, 1.0;
    input.torchObservation = torch;
    input.lvsObservation = lvs;
    input.lvsConstraint = lvsConstraint;
    return input;
}

UiFrameProcessingResult processVisionFrame(
    const i2r_rlc::camera::CameraFrame& frame
#ifdef I2R_RLC_HAS_OPENCV
    ,
    i2r_rlc::vision::WeldVisionPipelineOptions& options,
    i2r_rlc::vision::WeldVisionPipelineState& state
#endif
)
{
    QElapsedTimer timer;
    timer.start();

    UiFrameProcessingResult result;
    result.frameId = frame.frameId;
    result.sourcePath = frame.sourcePath;

    const QImage image = frame.image.isNull() ? frame.previewImage : frame.image;
    if (image.isNull()) {
        result.processSucceeded = false;
        result.preprocessingSucceeded = false;
        result.message = QStringLiteral("视觉输入帧无效：图像为空。");
        result.weldDetection = makeDetectionResult(
            frame,
            UiDetectionStatus::InvalidInput,
            QStringLiteral("焊缝检测输入无效：图像为空。"));
        result.processingTimeMs = static_cast<double>(timer.nsecsElapsed()) / 1000000.0;
        return result;
    }

    result.imageWidthPixel = image.width();
    result.imageHeightPixel = image.height();

#ifdef I2R_RLC_HAS_OPENCV
    QString errorMessage;
    const cv::Mat cvImage = qImageToCvMat(image, &errorMessage);
    if (cvImage.empty()) {
        result.processSucceeded = false;
        result.preprocessingSucceeded = false;
        result.message = QStringLiteral("OpenCV 预处理失败：%1").arg(errorMessage);
        result.weldDetection = makeDetectionResult(
            frame,
            UiDetectionStatus::InvalidInput,
            QStringLiteral("焊缝检测输入无效：OpenCV 预处理失败。"));
        result.processingTimeMs = static_cast<double>(timer.nsecsElapsed()) / 1000000.0;
        return result;
    }

    result.imageChannels = cvImage.channels();
    const cv::Mat gray8 = toGray8(cvImage, &errorMessage);
    if (gray8.empty()) {
        result.processSucceeded = false;
        result.preprocessingSucceeded = false;
        result.message = QStringLiteral("OpenCV 灰度化失败：%1").arg(errorMessage);
        result.weldDetection = makeDetectionResult(
            frame,
            UiDetectionStatus::InvalidInput,
            QStringLiteral("焊缝检测输入无效：OpenCV 灰度化失败。"));
        result.processingTimeMs = static_cast<double>(timer.nsecsElapsed()) / 1000000.0;
        return result;
    }

    cv::Scalar meanValue;
    cv::Scalar stddevValue;
    cv::meanStdDev(gray8, meanValue, stddevValue);
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(gray8, &minValue, &maxValue);
    result.meanGrayValue = meanValue[0];
    result.minGrayValue = minValue;
    result.maxGrayValue = maxValue;

    i2r_rlc::vision::WeldVisionFrameInput input;
    input.gray8 = gray8;
    input.frameId = frame.frameId;
    input.sourcePath = frame.sourcePath.toStdString();

    const i2r_rlc::vision::WeldVisionPipelineResult algorithmResult =
        i2r_rlc::vision::CWeldVisionAlgorithmPipeline::process(input, options, &state);

    result.weldDetection.frameId = frame.frameId;
    result.weldDetection.sourcePath = frame.sourcePath;
    result.weldDetection.status = mapPipelineStatus(algorithmResult.status);
    result.weldDetection.message = QStringLiteral("%1：%2")
                                       .arg(QString::fromStdString(
                                           i2r_rlc::vision::weldVisionPipelineStatusToKey(
                                               algorithmResult.status)))
                                       .arg(QString::fromStdString(algorithmResult.message));
    result.weldDetection.weldPointPixel = toPixelPoint(algorithmResult.weldPoint);
    result.weldDetection.centerline = toCenterline(algorithmResult.curveOrCenterlinePoints);
    result.weldDetection.confidence = algorithmResult.confidence;
    result.weldDetection.processingTimeMs = algorithmResult.processingTimeMs;

    result.preprocessingSucceeded = true;
    result.processSucceeded = true;
    result.message = QStringLiteral("OpenCV 预处理完成：mean=%1, min=%2, max=%3；算法管线：%4")
                         .arg(result.meanGrayValue, 0, 'f', 3)
                         .arg(result.minGrayValue, 0, 'f', 0)
                         .arg(result.maxGrayValue, 0, 'f', 0)
                         .arg(result.weldDetection.message);
#else
    result.preprocessingSucceeded = false;
    result.processSucceeded = false;
    result.weldDetection = makeDetectionResult(
            frame,
            UiDetectionStatus::AlgorithmUnavailable,
            QStringLiteral("OpenCV 未启用，无法执行焊缝检测算法。"));
    result.message = QStringLiteral("OpenCV 未启用，视觉算法管线不可用。");
#endif

    result.processingTimeMs = static_cast<double>(timer.nsecsElapsed()) / 1000000.0;
    result.message += QStringLiteral("；焊缝检测：%1。")
                          .arg(detectionStatusToKey(result.weldDetection.status));
    return result;
}

} // namespace

UiController::UiController(MainWidget& view, QObject* parent)
    : QObject(parent)
    , m_view(view)
    , m_configRepository(AppPaths::fromDefaultLocation())
{
    m_automaticCalibrationLaserTimer = new QTimer(this);
    m_automaticCalibrationLaserTimer->setSingleShot(true);
    connect(m_automaticCalibrationLaserTimer, &QTimer::timeout,
            this, &UiController::capturePendingAutomaticCalibrationLaserImage);
    m_cameraService.setDevice(&m_offlineCamera);
    bindView();
}

UiController::~UiController()
{
    shutdown();
}

bool UiController::setLiveCameraDevice(
    std::unique_ptr<i2r_rlc::camera::ICameraDevice> device,
    int captureIntervalMs)
{
    const bool configured = m_runtimeCoordinator.setCameraDevice(
        std::move(device), captureIntervalMs);
    m_liveCameraDeviceConfigured = configured;
    return configured;
}

void UiController::initialize()
{
    appendInfo(QStringLiteral("i2r_rlc 最小骨架已启动。"));
    appendInfo(QStringLiteral("持久化滚动日志：%1")
                   .arg(Logger::logFilePath()));
    appendInfo(QStringLiteral("运行时根目录：%1").arg(m_configRepository.runtimeRootText()));
    appendInfo(m_configRepository.calibrationSearchText());
    m_runtimeCoordinator.start();
    QString directoryError;
    if (!m_configRepository.ensureRuntimeDirectories(&directoryError)) {
        appendWarning(QStringLiteral("运行时目录检查失败：%1").arg(directoryError));
    }
    const QString teachTrajectoryTempDir =
        m_configRepository.paths().teachTrajectoryCaptureDir().absolutePath();
    const QString teachTrajectoryOutputDir =
        m_configRepository.paths().workpieceTrajectoryDir().absolutePath();
    m_view.setTeachTrajectoryOutputDirectory(teachTrajectoryOutputDir);
    m_runtimeCoordinator.configureTeachTrajectoryTempDirectory(teachTrajectoryTempDir);
    m_view.setTeachTrajectoryStateText(QStringLiteral("未开始"));
    m_view.setExposureControlValue(m_cameraExposureUs);
    m_view.setLaserControlChecked(m_laserEnabled);
    m_view.setLegacyStatusPanel(
        QStringLiteral("UNKNOWN"),
        QStringLiteral("SERVO OFF"),
        QStringLiteral("ROBOTS STOP!"),
        QStringLiteral("WAIT"),
        m_laserEnabled,
        m_filterEnabled,
        robotDeviceText(m_robotDeviceConnected),
        QStringLiteral("--"),
        QString::number(m_cameraExposureUs));
    loadWorkpieceProfiles();

    const auto calibrationResult = m_configRepository.loadSensorCalibration();
    const auto positionerCalibration =
        m_configRepository.loadPositionerCalibration();
    i2r_rlc::ui::RuntimeTrackingOptions trackingOptions;
    trackingOptions.detType = 2;
    trackingOptions.virtualStatus =
        i2r_rlc::control::TrackingVirtualStatus::OnlineRecord;
    trackingOptions.method = i2r_rlc::ui::RuntimeTrackingMethod::Kcf;
    if (calibrationResult.ok) {
        const auto& calibration = calibrationResult.data;
        trackingOptions.sensorCalibration = i2r_rlc::vision::WeldSensorCalibration{
            calibration.focalLengthM,
            calibration.kappa,
            calibration.sxM,
            calibration.syM,
            calibration.cxPixel,
            calibration.cyPixel,
            calibration.laserPlaneA,
            calibration.laserPlaneB,
            calibration.laserPlaneC,
            calibration.poseCinTXM,
            calibration.poseCinTYM,
            calibration.poseCinTZM,
            calibration.poseCinTRxDeg,
            calibration.poseCinTRyDeg,
            calibration.poseCinTRzDeg,
            true};
        appendInfo(QStringLiteral("已加载传感器标定文件：file=%1，f=%2 m，cx=%3，cy=%4，laser=(%5,%6,%7)")
                       .arg(m_configRepository.paths().sensorCalibrationFilePath())
                       .arg(calibration.focalLengthM, 0, 'f', 6)
                       .arg(calibration.cxPixel, 0, 'f', 6)
                       .arg(calibration.cyPixel, 0, 'f', 6)
                       .arg(calibration.laserPlaneA, 0, 'f', 6)
                       .arg(calibration.laserPlaneB, 0, 'f', 6)
                       .arg(calibration.laserPlaneC, 0, 'f', 6));
    } else {
        appendWarning(QStringLiteral("传感器标定文件加载失败：%1").arg(calibrationResult.errorMessage));
    }
    trackingOptions.positionerCalibration = positionerCalibration;
    if (calibrationResult.ok) {
        const auto& c = calibrationResult.data;
        m_sensorCalibration = i2r_rlc::vision::WeldSensorCalibration{
            c.focalLengthM,
            c.kappa,
            c.sxM,
            c.syM,
            c.cxPixel,
            c.cyPixel,
            c.laserPlaneA,
            c.laserPlaneB,
            c.laserPlaneC,
            c.poseCinTXM,
            c.poseCinTYM,
            c.poseCinTZM,
            c.poseCinTRxDeg,
            c.poseCinTRyDeg,
            c.poseCinTRzDeg,
            true};
    }
    m_positionerCalibration = positionerCalibration;
    if (!positionerCalibration.valid) {
        appendWarning(QStringLiteral("RBCALIB.DAT 加载失败：%1")
                          .arg(QString::fromStdString(positionerCalibration.message)));
    } else {
        appendInfo(QStringLiteral("已自动加载 RBCALIB.DAT：%1")
                       .arg(m_configRepository.paths().rbcalibFilePath()));
    }
    m_runtimeCoordinator.configureTracking(
        trackingOptions,
        i2r_rlc::vision::WeldProjectionContext{});

    const bool autoStartDevices = !autoDeviceStartDisabled();
    if (m_liveCameraDeviceConfigured) {
        appendInfo(QStringLiteral(
            "实时相机采集与 UI/KCF 显示链路已启用；已注入真实流式相机设备。"
            "RobotSim 仿真回调仍需现场适配。"));
        if (autoStartDevices) {
            appendInfo(QStringLiteral("启动应用：自动启动实时相机采集。"));
            m_runtimeCoordinator.startCamera();
            m_view.setVisionStateText(QStringLiteral("视觉：实时相机启动中"));
        } else {
            appendInfo(QStringLiteral("自动设备启动已禁用：跳过实时相机自动启动。"));
            m_view.setVisionStateText(QStringLiteral("视觉：自动启动已禁用"));
        }
    } else {
        appendWarning(QStringLiteral(
            "实时相机采集与 UI/KCF 显示链路已就绪；当前启动未注入 Vimba/Basler 设备。"
            "RobotSim 仿真回调仍需现场适配。"));
        m_view.setVisionStateText(QStringLiteral("视觉：未注入实时相机设备"));
    }

    if (autoStartDevices) {
        appendInfo(QStringLiteral("启动应用：自动连接机器人控制柜。"));
        m_robotConnectionPromptPending = false;
        m_runtimeCoordinator.connectRobot();
    } else {
        appendInfo(QStringLiteral("自动设备启动已禁用：跳过机器人自动连接。"));
    }

    i2r_rlc::control::CalibrationCaptureOptions calibrationOptions;
    calibrationOptions.outputRootDirectory =
        m_configRepository.paths().runtimeRootDir()
            .filePath(QStringLiteral("Data"))
            .toStdString();
    m_calibrationCaptureController.configure(calibrationOptions);
    m_view.setCalibrationStateText(QStringLiteral("标定：空闲"));

    appendInfo(m_localizationService.start().message);
    appendInfo(m_digitalTwinService.start().message);
    appendInfo(QString::fromStdString(m_ibvsService.start().message));
    m_view.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：%1").arg(
        onlineCorrectionPhaseToText(i2r_rlc::control::OnlineCorrectionPhase::Idle)));
    m_onlineCorrectionP001Presentation.clear();
    m_view.setOnlineCorrectionP001Text(
        m_onlineCorrectionP001Presentation.text());
    m_view.setIbvsStateText(QStringLiteral("视觉伺服：%1\n%2")
                                .arg(ibvsServiceStateToText(m_ibvsService.state()))
                                .arg(QString::fromStdString(
                                    m_ibvsService.lastDiagnostics().message)));
    m_view.setLocalizationStateText(QStringLiteral("定位：%1\n模式：--\n自由度：x/y/z/rz\n等待离线回放或虚拟参考")
                                        .arg(i2r_rlc::localization::localizationWorkflowStateToText(
                                            i2r_rlc::localization::LocalizationWorkflowState::Idle)));
    m_runtimeCoordinator.setGlobalLocateVirtualLaserSource(
        GlobalLocateVirtualLaserSource::Occt);
    m_runtimeCoordinator.setGlobalLocateVirtualPointCloudSource(
        GlobalLocateVirtualPointCloudSource::Weldment);
    m_view.setGlobalLocateRobotSimConnectionText(
        QStringLiteral("OCCT 激光 + weldment 点云：无需连接 RobotSim"));
    m_view.setDigitalTwinStateText(QStringLiteral("仿真：未连接"));
}

void UiController::shutdown()
{
    cancelPendingAutomaticCalibrationLaserCapture();
    cancelPendingCorrectionRobotStart();
    cancelOnlineAutoCorrectionWorkflow();
    cancelStandaloneGlobalLocate();
    disconnect(&m_runtimeCoordinator, nullptr, this, nullptr);
    m_runtimeCoordinator.stop();
}

void UiController::bindView()
{
    connect(&m_view, &MainWidget::cameraExposureRequested,
            this, &UiController::onCameraExposureRequested);
    connect(&m_view, &MainWidget::laserEnabledChanged,
            this, &UiController::onLaserEnabledChanged);
    connect(&m_view, &MainWidget::workpieceSelected,
            this, &UiController::onWorkpieceSelected);
    connect(&m_view, &MainWidget::onlineCorrectionStartModeChanged,
            this, &UiController::onOnlineCorrectionStartModeChanged);
    connect(&m_view, &MainWidget::onlineCorrectionConfigurationRequested,
            this, &UiController::onOnlineCorrectionConfigurationRequested);
    connect(&m_view, &MainWidget::onlineCorrectionConfigurationSelected,
            this, &UiController::onOnlineCorrectionConfigurationSelected);
    connect(&m_view, &MainWidget::startWeldTrackingRequested,
            this, &UiController::onStartWeldTrackingRequested);
    connect(&m_view, &MainWidget::startManualCalibrationCaptureRequested,
            this, &UiController::onStartManualCalibrationCaptureRequested);
    connect(&m_view, &MainWidget::captureCalibrationBoardRequested,
            this, &UiController::onCaptureCalibrationBoardRequested);
    connect(&m_view, &MainWidget::deleteLastCalibrationSampleRequested,
            this, &UiController::onDeleteLastCalibrationSampleRequested);
    connect(&m_view, &MainWidget::captureCalibrationLaserRequested,
            this, &UiController::onCaptureCalibrationLaserRequested);
    connect(&m_view, &MainWidget::previewAutomaticCalibrationCaptureRequested,
            this, &UiController::onPreviewAutomaticCalibrationCaptureRequested);
    connect(&m_view, &MainWidget::startAutomaticCalibrationCaptureRequested,
            this, &UiController::onStartAutomaticCalibrationCaptureRequested);
    connect(&m_view, &MainWidget::finishCalibrationCaptureRequested,
            this, &UiController::onFinishCalibrationCaptureRequested);
    connect(&m_view, &MainWidget::calibrationModeSelectionReturned,
            this, &UiController::onCalibrationModeSelectionReturned);
    connect(&m_view, &MainWidget::calibrationFeaturePanelClosed,
            this, &UiController::onCalibrationFeaturePanelClosed);
    connect(&m_view, &MainWidget::offlineCorrectionConfigurationRequested,
            this, &UiController::onOfflineCorrectionConfigurationRequested);
    connect(&m_view, &MainWidget::offlineCorrectionArchiveSelected,
            this, &UiController::onOfflineCorrectionArchiveSelected);
    connect(&m_view, &MainWidget::startOfflineCorrectionRequested,
            this, &UiController::onStartOfflineCorrectionRequested);
    connect(&m_view, &MainWidget::offlineCorrectionFeaturePanelClosed,
            this, &UiController::onOfflineCorrectionFeaturePanelClosed);
    connect(&m_view, &MainWidget::onlineCorrectionFeaturePanelClosed,
            this, &UiController::onOnlineCorrectionFeaturePanelClosed);
    connect(&m_view, &MainWidget::startTeachTrajectoryRecordingRequested,
            this, &UiController::onStartTeachTrajectoryRecordingRequested);
    connect(&m_view, &MainWidget::saveTeachTrajectoryRequested,
            this, &UiController::onSaveTeachTrajectoryRequested);
    connect(&m_view, &MainWidget::teachTrajectoryFeaturePanelClosed,
            this, &UiController::onTeachTrajectoryFeaturePanelClosed);
    connect(&m_view, &MainWidget::loadIbvsConfigRequested,
            this, &UiController::onLoadIbvsConfigRequested);
    connect(&m_view, &MainWidget::startIbvsDryRunRequested,
            this, &UiController::onStartIbvsDryRunRequested);
    connect(&m_view, &MainWidget::stepIbvsDryRunRequested,
            this, &UiController::onStepIbvsDryRunRequested);
    connect(&m_view, &MainWidget::stopIbvsDryRunRequested,
            this, &UiController::onStopIbvsDryRunRequested);
    connect(&m_view, &MainWidget::runLocalCorrectionReplayRequested,
            this, &UiController::onRunLocalCorrectionReplayRequested);
    connect(&m_view, &MainWidget::globalLocateRequested,
            this, &UiController::onGlobalLocateRequested);
    connect(
        &m_view,
        &MainWidget::globalLocateVirtualLaserConfigurationChanged,
        this,
        &UiController::onGlobalLocateVirtualLaserConfigurationChanged);
    connect(&m_view, &MainWidget::clearLocalizationResultRequested,
            this, &UiController::onClearLocalizationResultRequested);
    connect(&m_view, &MainWidget::robotSimVirtualReferenceRequested,
            this, &UiController::onRobotSimVirtualReferenceRequested);
    connect(&m_view, &MainWidget::robotSimRealSceneConnectionRequested,
            this, &UiController::onRobotSimRealSceneConnectionRequested);
    connect(&m_view, &MainWidget::stopRequested, this, &UiController::onStopRequested);

    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::statusMessage,
            this, &UiController::appendInfo, Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::onlineCorrectionStartProgress,
        this,
        [this](const QString &message) {
            if (m_onlineAutoCorrectionWorkflow.state()
                != OnlineCorrectionWorkflowState::WaitingOnlineStartGate) {
                return;
            }
            m_view.setOnlineCorrectionStateText(
                QStringLiteral("在线纠偏：启动事务进行中\n%1")
                    .arg(message));
        },
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::onlineCorrectionStartSucceeded,
        this,
        [this]() {
            if (!m_onlineAutoCorrectionWorkflow.markOnlineCorrectionStarted()) {
                failOnlineAutoCorrectionWorkflow(QStringLiteral(
                    "在线纠偏内部状态错误：启动事务完成后无法进入运行态。"));
                return;
            }
            m_view.setOnlineCorrectionStateText(QStringLiteral(
                "在线纠偏：启动事务完成\n已进入 STARTDETPRG，正在启动初始检测。"));
        },
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::onlineCorrectionStartFailed,
        this,
        [this](const QString &message) {
            if (m_onlineAutoCorrectionWorkflow.state()
                == OnlineCorrectionWorkflowState::WaitingOnlineStartGate) {
                failOnlineAutoCorrectionWorkflow(message);
            }
        },
        Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::frameReady,
            this, &UiController::handleRuntimeFrameReady, Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::trackingResultReady,
            this, &UiController::showRuntimeTrackingResult, Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::globalLocateVirtualPriorReady,
        this,
        &UiController::handleGlobalLocateVirtualPriorResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::globalLocateFrameVirtualPriorReady,
        this,
        &UiController::handleGlobalLocateFrameVirtualPriorResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::globalLocateFrameVirtualPriorReady,
        this,
        &UiController::handleOnlineAutoGlobalLocateVirtualPriorResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::robotSimRealtimeVirtualPriorReady,
        this,
        &UiController::handleRobotSimRealtimeVirtualPriorResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::robotSimRealtimeVirtualReferenceReady,
        this,
        &UiController::handleRobotSimRealtimeVirtualReferenceResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::robotSimManualConnectionStateChanged,
        this,
        &UiController::handleRobotSimManualConnectionStateChanged,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::robotSimManualVirtualReferenceReady,
        this,
        &UiController::handleRobotSimManualVirtualReferenceResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::globalLocalizationTrackingDrained,
        this,
        &UiController::completeStandaloneGlobalLocateVisionDrain,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::globalLocalizationTrackingDrained,
        this,
        &UiController::completeOnlineAutoCorrectionGlobalLocateVisionDrain,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::standaloneGlobalLocalizationReady,
        this,
        &UiController::handleStandaloneGlobalLocalizationResult,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::robotSimConnectionStateChanged,
        this,
        &UiController::handleRobotSimConnectionStateChanged,
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::robotSimStateSynchronizationChanged,
        this,
        &UiController::handleRobotSimStateSynchronizationChanged,
        Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::onlineCorrectionUpdated,
            this, &UiController::showOnlineCorrectionUpdate, Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::offlineCorrectionUpdated,
            this, &UiController::showOfflineCorrectionUpdate, Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::offlineCorrectionHandshakeChanged,
        this,
        [this](int offlineStartCount) {
            m_offlineStartCount = offlineStartCount;
            renderOfflineCorrectionState();
        },
        Qt::QueuedConnection);
    connect(
        &m_runtimeCoordinator,
        &CRuntimeCoordinator::correctionSyncUpdated,
        this,
        &UiController::handleCorrectionSyncUpdate,
        Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::teachTrajectoryRecorderUpdated,
            this, &UiController::showTeachTrajectoryRecorderUpdate, Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::robotStateReady,
            this, [this](const i2r_rlc::robot_comm::RobotState& state) {
                m_robotState = state;
                if (m_hasOfflineCorrectionUpdate) {
                    renderOfflineCorrectionState();
                }
                processAutomaticCalibrationRobotState();
                evaluatePendingCorrectionRobotStart();
                processOnlineAutoCorrectionRobotState();
                processStandaloneGlobalLocateRobotState();
            },
            Qt::QueuedConnection);
    connect(&m_runtimeCoordinator, &CRuntimeCoordinator::robotDiagnosticsReady,
            this, &UiController::showRobotDiagnostics, Qt::QueuedConnection);
}

void UiController::showRobotDiagnostics(
    const i2r_rlc::robot_comm::RobotCommunicationDiagnostics& diagnostics)
{
    const auto& state = diagnostics.validDReceived ? diagnostics.state : m_robotState;
    if (diagnostics.validDReceived) {
        m_robotState = diagnostics.state;
    }
    m_robotDeviceConnected = diagnostics.validDReceived;
    m_laserEnabled = diagnostics.laserEnabled != 0;
    m_filterEnabled = diagnostics.filterEnabled != 0;
    m_view.setLaserControlChecked(m_laserEnabled);
    m_view.setLegacyStatusPanel(
        robotModeToLegacyText(state.sMode),
        servoToLegacyText(state.sServoPower),
        runToLegacyText(state.sStart),
        correctionToLegacyText(state.corflag),
        m_laserEnabled,
        m_filterEnabled,
        robotDeviceText(diagnostics),
        QStringLiteral("--"),
        QString::number(m_cameraExposureUs));

    if (m_robotConnectionPromptPending) {
        if (diagnostics.validDReceived) {
            m_robotConnectionPromptPending = false;
            showLegacyInfoPrompt(
                QStringLiteral("提示!"),
                QStringLiteral("机器人连接成功，已收到有效 D 回包，状态监控可信。"));
        } else if (!diagnostics.connected || !diagnostics.lastErrorText.empty()) {
            m_robotConnectionPromptPending = false;
            m_lastPromptedRobotFailureTransactionCount = diagnostics.transactionCount;
            showLegacyWarningPrompt(
                QStringLiteral("提示!"),
                QStringLiteral("机器人连接/状态回包异常：%1")
                    .arg(diagnostics.lastErrorText.empty()
                             ? QStringLiteral("未收到有效 D 回包")
                             : QString::fromStdString(diagnostics.lastErrorText)));
        }
    }

    if (m_laserCommandPromptPending
        && (diagnostics.lastCommandType
                == i2r_rlc::robot_comm::RobotCommandType::StatusPoll
            || !diagnostics.connected
            || !diagnostics.lastErrorText.empty())) {
        m_laserCommandPromptPending = false;
        if (!diagnostics.lastTransactionSucceeded) {
            m_lastPromptedRobotFailureTransactionCount = diagnostics.transactionCount;
            appendWarning(
                QStringLiteral("激光指令 N 发送失败：%1；已保留当前激光期望状态，重新连接后会继续随 N/O/P 携带。")
                    .arg(diagnostics.lastErrorText.empty()
                             ? QStringLiteral("未收到有效 D 回包")
                             : QString::fromStdString(diagnostics.lastErrorText)));
        }
    }

    const bool detailed =
        diagnostics.transactionCount != m_lastDetailedTransactionCount
        && (diagnostics.lastCommandType
                != i2r_rlc::robot_comm::RobotCommandType::StatusPoll
            || !diagnostics.errorMessage.empty());
    if (detailed) {
        appendInfo(QStringLiteral(
                       "RobotComm 请求：%1\nD 回包：%2%3\n状态：sStart=%4, sMode=%5, sServoPower=%6, s_Bsavetraj=%7")
                       .arg(
                           QString::fromStdString(diagnostics.lastRequest),
                           QString::fromStdString(diagnostics.latestResponse),
                           diagnostics.errorMessage.empty()
                               ? QString()
                               : QStringLiteral("\n错误：%1").arg(
                                     QString::fromStdString(diagnostics.errorMessage)))
                       .arg(state.sStart)
                       .arg(state.sMode)
                       .arg(state.sServoPower)
                       .arg(state.s_Bsavetraj));
        m_lastDetailedTransactionCount = diagnostics.transactionCount;
        const bool statusPollFailure =
            diagnostics.lastCommandType
            == i2r_rlc::robot_comm::RobotCommandType::StatusPoll;
        if (!statusPollFailure
            && !diagnostics.errorMessage.empty()
            && diagnostics.transactionCount
                   != m_lastPromptedRobotFailureTransactionCount
            && diagnostics.consecutiveFailureCount == 1) {
            m_lastPromptedRobotFailureTransactionCount = diagnostics.transactionCount;
            showLegacyWarningPrompt(
                QStringLiteral("提示!"),
                QStringLiteral("RobotComm 通信异常：%1")
                    .arg(QString::fromStdString(diagnostics.errorMessage)));
        }
    } else if (diagnostics.timestampMs - m_lastRobotPollSummaryMs >= 1000) {
        appendInfo(QStringLiteral(
                       "RobotComm N/D 摘要 #%1：%2 -> %3；sStart=%4, sMode=%5, sServoPower=%6, s_Bsavetraj=%7")
                       .arg(diagnostics.transactionCount)
                       .arg(QString::fromStdString(diagnostics.latestStatusRequest))
                       .arg(QString::fromStdString(diagnostics.latestResponse))
                       .arg(state.sStart)
                       .arg(state.sMode)
                       .arg(state.sServoPower)
                       .arg(state.s_Bsavetraj));
        m_lastRobotPollSummaryMs = diagnostics.timestampMs;
    }

    evaluatePendingCorrectionRobotStart();
    processOnlineAutoCorrectionRobotState();
    processStandaloneGlobalLocateRobotState();
}

void UiController::onCameraExposureRequested(int exposureUs)
{
    if (exposureUs <= 0) {
        appendWarning(QStringLiteral("曝光设置失败：曝光时间必须大于 0 us。"));
        return;
    }

    m_cameraExposureUs = exposureUs;
    m_view.setExposureControlValue(m_cameraExposureUs);
    m_runtimeCoordinator.setCameraExposureUs(exposureUs);
    m_view.setLegacyStatusPanel(
        robotModeToLegacyText(m_robotState.sMode),
        servoToLegacyText(m_robotState.sServoPower),
        runToLegacyText(m_robotState.sStart),
        correctionToLegacyText(m_robotState.corflag),
        m_laserEnabled,
        m_filterEnabled,
        robotDeviceText(m_robotDeviceConnected),
        QStringLiteral("--"),
        QString::number(m_cameraExposureUs));
    appendInfo(QStringLiteral("UI 请求设置相机曝光：%1 us").arg(exposureUs));
}

void UiController::onLaserEnabledChanged(bool enabled)
{
    m_laserEnabled = enabled;
    m_view.setLaserControlChecked(enabled);
    m_laserCommandPromptPending = true;
    m_runtimeCoordinator.setLaserEnabled(enabled);
    m_view.setLegacyStatusPanel(
        robotModeToLegacyText(m_robotState.sMode),
        servoToLegacyText(m_robotState.sServoPower),
        runToLegacyText(m_robotState.sStart),
        correctionToLegacyText(m_robotState.corflag),
        m_laserEnabled,
        m_filterEnabled,
        robotDeviceText(m_robotDeviceConnected),
        QStringLiteral("--"),
        QString::number(m_cameraExposureUs));
    appendInfo(enabled
                   ? QStringLiteral("UI 请求激光指令 ON，立即 N 发送 s_laser=1。")
                   : QStringLiteral("UI 请求激光指令 OFF，立即 N 发送 s_laser=0。"));
}

void UiController::onWorkpieceSelected(int profileIndex, const QString& profileId)
{
    if (profileIndex < 0 || profileIndex >= m_workpieceProfiles.size()) {
        appendWarning(QStringLiteral("工件选择失败：索引无效 %1。").arg(profileIndex));
        m_view.setCurrentWorkpieceText(QStringLiteral("工件选择失败：索引无效"));
        return;
    }

    const WorkpieceProfile& profile = m_workpieceProfiles.at(profileIndex);
    if (profile.id != profileId) {
        appendWarning(QStringLiteral("工件选择失败：UI id=%1 与配置 id=%2 不一致。")
                          .arg(profileId, profile.id));
        m_view.setCurrentWorkpieceText(QStringLiteral("工件选择失败：配置不一致"));
        return;
    }

    // 等待请求绑定旧工件；切换工件时必须撤销，不能随后启动旧 JBI。
    cancelPendingCorrectionRobotStart();
    cancelOnlineAutoCorrectionWorkflow();
    m_selectedWorkpieceIndex = profileIndex;
    m_onlineCorrectionConfigured = false;
    m_offlineCorrectionConfigured = false;
    m_offlineCorrectionPhase = i2r_rlc::control::OfflineCorrectionPhase::Idle;
    m_offlineCompletionPromptShown = false;
    m_offlineSeamWindowClosed = false;
    m_hasOfflineCorrectionUpdate = false;
    m_offlineStartCount = 0;
    m_runtimeCoordinator.resetOfflineCorrection();

    const QString onlineWorkPathFile = m_configRepository.workpieceOnlineWorkPathFile(profile);
    const QString onlineTeachPoseFile = m_configRepository.workpieceOnlineTeachPoseFile(profile);
    const QString offlineWorkPathFile =
        m_configRepository.workpieceOfflineCorrectionWorkPathFile(profile);
    const QString offlineTeachPoseFile =
        m_configRepository.workpieceOfflineCorrectionTeachPoseFile(profile);
    m_view.setCurrentWorkpieceText(QStringLiteral("已选择：%1").arg(profile.displayName));
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("当前工件：%1\n正在自动加载在线纠偏配置").arg(profile.displayName));
    m_view.setOfflineCorrectionStateText(
        QStringLiteral("当前工件：%1\n正在自动加载离线纠偏配置").arg(profile.displayName));
    appendInfo(QStringLiteral(
                   "已选择工件：%1\n在线_W=%2\n在线_T=%3\n离线纠偏_W=%4\n离线纠偏_T=%5\n全局定位JBI=%6\n在线JBI(InitType)=%7\n在线JBI(GlobalCorrection)=%8\n在线JBI(IBVSStart)=%9\n离线纠偏JBI=%10")
                   .arg(
                       profile.displayName,
                       onlineWorkPathFile,
                       onlineTeachPoseFile,
                       offlineWorkPathFile,
                       offlineTeachPoseFile,
                       profile.offlineLocateJbi,
                       profile.onlineJbi,
                       profile.onlineGlobalCorrectionJbi,
                       profile.onlineIbvsStartJbi,
                       profile.offlineCorrectionJbi));
    configureSelectedWorkpieceCorrection();
}

void UiController::onOnlineCorrectionStartModeChanged(int modeIndex)
{
    bool valid = true;
    m_onlineCorrectionStartMode =
        onlineCorrectionStartModeFromIndex(modeIndex, &valid);

    if (!valid) {
        appendWarning(QStringLiteral(
            "在线纠偏起始点定位方式索引无效：%1，已回退为 InitType。")
                          .arg(modeIndex));
    }

    const QString modeText =
        onlineCorrectionStartModeToText(m_onlineCorrectionStartMode);
    appendInfo(QStringLiteral("在线纠偏起始点定位方式已选择：%1").arg(modeText));

    if (selectedWorkpieceProfile() != nullptr) {
        configureSelectedWorkpieceOnlineCorrection();
    }
}

void UiController::onOnlineCorrectionConfigurationRequested()
{
    configureSelectedWorkpieceOnlineCorrection();
}

void UiController::onOnlineCorrectionConfigurationSelected(
    const QString& workPathFile,
    const QString& teachPoseFile)
{
    const QFileInfo workPathInfo(workPathFile);
    const QFileInfo teachPoseInfo(teachPoseFile);
    if (!workPathInfo.isFile()
        || !workPathInfo.fileName().endsWith(QStringLiteral("_W.txt"), Qt::CaseInsensitive)) {
        m_onlineCorrectionConfigured = false;
        appendWarning(QStringLiteral("在线纠偏配置失败：工作轨迹必须是存在的 *_W.txt 文件。"));
        showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("未选择示教文件或错误示教文件!"));
        return;
    }
    if (!teachPoseInfo.isFile()
        || !teachPoseInfo.fileName().endsWith(QStringLiteral("_T.txt"), Qt::CaseInsensitive)) {
        m_onlineCorrectionConfigured = false;
        appendWarning(QStringLiteral("在线纠偏配置失败：示教位姿必须是存在的 *_T.txt 文件。"));
        showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("未选择示教文件或错误示教文件!"));
        return;
    }
    RuntimeOnlineCorrectionConfiguration configuration;
    configuration.trajectoryRequest.workPathFile = workPathInfo.absoluteFilePath().toStdString();
    configuration.trajectoryRequest.teachPoseFile = teachPoseInfo.absoluteFilePath().toStdString();
    configuration.rbcalibFile =
        m_configRepository.paths().rbcalibFilePath().toStdString();
    configuration.diagnosticOutputDirectory =
        m_configRepository.paths().outputDir()
            .filePath(QStringLiteral("online_correction"))
            .toStdString();
    configuration.options.startMode = m_onlineCorrectionStartMode;

    const auto trajectory =
        i2r_rlc::control::COnlineCorrectionTrajectoryPlanner::loadTrajectory(
            configuration.trajectoryRequest);
    if (!trajectory.valid) {
        m_onlineCorrectionConfigured = false;
        appendWarning(QStringLiteral("在线纠偏配置失败：%1")
                          .arg(QString::fromStdString(trajectory.message)));
        showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("未选择示教文件或错误示教文件!"));
        return;
    }
    const auto calibration =
        i2r_rlc::vision::CPositionerCalibrationParser::parseFile(
            configuration.rbcalibFile);
    if (!calibration.valid) {
        m_onlineCorrectionConfigured = false;
        appendWarning(QStringLiteral("在线纠偏配置失败：%1")
                          .arg(QString::fromStdString(calibration.message)));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("在线纠偏配置失败：RBCALIB.DAT 缺失或无效。"));
        return;
    }

    m_positionerCalibration = calibration;
    m_onlineCorrectionConfiguration = configuration;
    m_runtimeCoordinator.configureOnlineCorrection(m_onlineCorrectionConfiguration);
    m_onlineCorrectionConfigured = true;

    const QString message = QStringLiteral(
        "在线纠偏配置已提交：\n起始点=%1\n_W=%2\n_T=%3\nRBCALIB=%4")
                                .arg(
                                    onlineCorrectionStartModeToText(m_onlineCorrectionStartMode),
                                    workPathInfo.absoluteFilePath(),
                                    teachPoseInfo.absoluteFilePath(),
                                    m_configRepository.paths().rbcalibFilePath());
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：配置已提交\n起始点：%1")
            .arg(onlineCorrectionStartModeToText(m_onlineCorrectionStartMode)));
    appendInfo(message);
}

void UiController::onStartWeldTrackingRequested()
{
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::OnlineGlobalLocate
        || m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::OnlineCorrection) {
        cancelOnlineAutoCorrectionWorkflow();
        m_view.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：已取消等待启动"));
        appendInfo(QStringLiteral("在线纠偏：已取消等待，不会自动启动 JBI。"));
        return;
    }
    if (m_onlineAutoCorrectionWorkflow.state()
        != OnlineCorrectionWorkflowState::Idle) {
        appendWarning(QStringLiteral("在线纠偏流程正在执行，请使用停止按钮结束，不能重复启动。"));
        return;
    }
    if (!m_onlineCorrectionConfigured && selectedWorkpieceProfile() != nullptr) {
        configureSelectedWorkpieceOnlineCorrection();
    }
    if (!m_onlineCorrectionConfigured) {
        const QString message = QStringLiteral(
            "在线纠偏启动失败：尚未配置 *_W.txt、*_T.txt 和 RBCALIB.DAT。");
        m_view.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：配置缺失\n%1").arg(message));
        appendWarning(message);
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return;
    }

    appendInfo(QStringLiteral("UI 请求启动在线纠偏初始检测。"));
    const WorkpieceProfile* profile = selectedWorkpieceProfile();
    if (profile == nullptr) {
        appendWarning(QStringLiteral("在线纠偏启动失败：尚未选择工件，无法确定在线 JBI。"));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("未选择示教文件或错误示教文件!"));
        return;
    }

    const QString onlineJbi =
        onlineJbiForStartMode(*profile, m_onlineCorrectionStartMode);
    if (onlineJbi.isEmpty()) {
        appendWarning(QStringLiteral("在线纠偏启动失败：当前起始点方式未配置在线 JBI。"));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("未选择示教文件或错误示教文件!"));
        return;
    }
    const bool requiresGlobalLocalization =
        m_onlineCorrectionStartMode
        == i2r_rlc::control::OnlineCorrectionStartMode::GlobalCorrection;
    const QString globalLocateJbi = requiresGlobalLocalization
        ? profile->offlineLocateJbi.trimmed()
        : QString();
    if (requiresGlobalLocalization && globalLocateJbi.isEmpty()) {
        const QString message = QStringLiteral(
            "在线纠偏启动失败：当前工件未配置全局定位 JBI。");
        appendWarning(message);
        m_view.setOnlineCorrectionStateText(
            QStringLiteral("在线纠偏：启动失败\n%1").arg(message));
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return;
    }
    if (requiresGlobalLocalization
        && (!m_robotSimConnected || m_robotSimManualModeRequested)) {
        const QString message = QStringLiteral(
            "在线纠偏启动失败：全局定位修正模式需要先连接 RobotSim 真实场景。");
        appendWarning(message);
        m_view.setOnlineCorrectionStateText(
            QStringLiteral("在线纠偏：等待 RobotSim\n%1").arg(message));
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return;
    }

    cancelOnlineAutoCorrectionWorkflow();
    cancelStandaloneGlobalLocate();
    QString planError;
    if (!m_onlineAutoCorrectionWorkflow.begin(
            m_onlineCorrectionStartMode,
            globalLocateJbi,
            onlineJbi,
            &planError)) {
        const QString message =
            QStringLiteral("在线纠偏启动失败：%1").arg(planError);
        appendWarning(message);
        m_view.setOnlineCorrectionStateText(
            QStringLiteral("在线纠偏：启动失败\n%1").arg(message));
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return;
    }
    m_onlineCorrectionP001Presentation.clear();
    m_view.setOnlineCorrectionP001Text(
        m_onlineCorrectionP001Presentation.text());
    m_onlineAutoGlobalLocalizationScanPoints.clear();
    m_onlineAutoGlobalLocalizationVirtualPoints.clear();
    m_onlineAutoCorrectionRunConfiguration =
        m_onlineCorrectionConfiguration;
    if (!m_onlineAutoCorrectionWorkflow.plan().requiresGlobalLocalization) {
        queueOnlineAutoCorrectionStart();
        return;
    }
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：等待全局定位启动\n全局定位JBI：%1\n在线JBI：%2")
            .arg(globalLocateJbi, onlineJbi));
    appendInfo(QStringLiteral("在线纠偏一键全自动：等待机器人状态满足后启动全局定位 JBI：%1；第二段在线 JBI：%2（起始点：%3）")
                   .arg(globalLocateJbi,
                        onlineJbi,
                        onlineCorrectionStartModeToText(m_onlineCorrectionStartMode)));
    beginCorrectionRobotStartGate(
        CorrectionRobotStartKind::OnlineGlobalLocate,
        globalLocateJbi);
}

void UiController::onStartManualCalibrationCaptureRequested()
{
    appendInfo(QStringLiteral("UI 请求启动人工标定采集。"));
    cancelPendingAutomaticCalibrationLaserCapture();
    showLegacyInfoPrompt(
        QStringLiteral("提示!"),
        QStringLiteral("保存19张标定板图像和4张激光图像及19个位置信息，后4张标定板图需与激光图一一对应。"));
    showCalibrationCaptureUpdate(m_calibrationCaptureController.startManualCapture());
}

void UiController::onCaptureCalibrationBoardRequested()
{
#ifdef I2R_RLC_HAS_OPENCV
    if (!confirmLegacyPrompt(QStringLiteral("提示!"), QStringLiteral("确定添加？"))) {
        appendInfo(QStringLiteral("用户取消采集标定板图像。"));
        return;
    }
    if (!m_hasLatestFrame) {
        appendWarning(QStringLiteral("标定板采集失败：尚未读取图像帧。"));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("标定板采集失败：尚未读取图像帧。"));
        return;
    }

    QString errorMessage;
    const QImage image = m_latestFrame.image.isNull() ? m_latestFrame.previewImage : m_latestFrame.image;
    const cv::Mat cvImage = qImageToCvMat(image, &errorMessage);
    if (cvImage.empty()) {
        appendWarning(QStringLiteral("标定板采集失败：%1").arg(errorMessage));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("标定板采集失败：%1").arg(errorMessage));
        return;
    }

    showCalibrationCaptureUpdate(
        m_calibrationCaptureController.captureBoardSample(cvImage, m_robotState));
#else
    appendWarning(QStringLiteral("标定板采集失败：OpenCV 未启用。"));
    showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("标定板采集失败：OpenCV 未启用。"));
#endif
}

void UiController::onDeleteLastCalibrationSampleRequested()
{
    if (!confirmLegacyPrompt(QStringLiteral("提示!"), QStringLiteral("确定删除？"))) {
        appendInfo(QStringLiteral("用户取消删除上一组标定样本。"));
        return;
    }
    appendInfo(QStringLiteral("UI 请求删除上一组标定样本。"));
    showCalibrationCaptureUpdate(m_calibrationCaptureController.deleteLastSample());
}

void UiController::onCaptureCalibrationLaserRequested()
{
#ifdef I2R_RLC_HAS_OPENCV
    showLegacyInfoPrompt(QStringLiteral("提示!"), QStringLiteral("Get the laser Picture!"));
    if (!m_hasLatestFrame) {
        appendWarning(QStringLiteral("激光图采集失败：尚未读取图像帧。"));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("激光图采集失败：尚未读取图像帧。"));
        return;
    }

    QString errorMessage;
    const QImage image = m_latestFrame.image.isNull() ? m_latestFrame.previewImage : m_latestFrame.image;
    const cv::Mat cvImage = qImageToCvMat(image, &errorMessage);
    if (cvImage.empty()) {
        appendWarning(QStringLiteral("激光图采集失败：%1").arg(errorMessage));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("激光图采集失败：%1").arg(errorMessage));
        return;
    }

    showCalibrationCaptureUpdate(
        m_calibrationCaptureController.captureLaserImage(cvImage, m_robotState));
#else
    appendWarning(QStringLiteral("激光图采集失败：OpenCV 未启用。"));
    showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("激光图采集失败：OpenCV 未启用。"));
#endif
}

void UiController::onStartAutomaticCalibrationCaptureRequested()
{
    appendInfo(QStringLiteral("UI 确认自动标定红框，启动自动标定采集。"));
    if (!dispatchAutomaticCalibrationCorrectedPoses()) {
        return;
    }
    showLegacyInfoPrompt(QStringLiteral("提示!"), QStringLiteral("请运行标定程序。"));
    cancelPendingAutomaticCalibrationLaserCapture();
    m_lastCalibrationBcalib = 0;
    showCalibrationCaptureUpdate(m_calibrationCaptureController.startAutomaticCapture());
}

void UiController::onPreviewAutomaticCalibrationCaptureRequested()
{
    appendInfo(QStringLiteral("UI 请求自动标定红框预览，请将首张标定板图像移动至红框内。"));
    if (!dispatchAutomaticCalibrationTemplatePoses()) {
        return;
    }
    cancelPendingAutomaticCalibrationLaserCapture();
    showLegacyInfoPrompt(
        QStringLiteral("提示!"),
        QStringLiteral("请将首张标定板图像移动至红色框中，完毕后点击确认按钮!"));
    m_lastCalibrationBcalib = 0;
    showCalibrationCaptureUpdate(m_calibrationCaptureController.previewAutomaticCapture());
}

void UiController::onFinishCalibrationCaptureRequested()
{
    appendInfo(QStringLiteral("UI 请求结束标定采集。"));
    cancelPendingAutomaticCalibrationLaserCapture();
    m_lastCalibrationBcalib = 0;
    showCalibrationCaptureUpdate(m_calibrationCaptureController.finishCapture());
}

void UiController::onCalibrationModeSelectionReturned()
{
    appendInfo(QStringLiteral("系统标定返回模式选择，清理当前子模式状态。"));
    cancelPendingAutomaticCalibrationLaserCapture();
    m_lastCalibrationBcalib = 0;
    showCalibrationCaptureUpdate(m_calibrationCaptureController.reset());
    if (m_hasLatestFrame && !m_trackingOverlayActive) {
        m_view.showFrame(m_latestFrame);
    }
}

void UiController::onCalibrationFeaturePanelClosed()
{
    appendInfo(QStringLiteral("系统标定面板已收起，释放标定采集状态。"));
    cancelPendingAutomaticCalibrationLaserCapture();
    m_lastCalibrationBcalib = 0;
    showCalibrationCaptureUpdate(m_calibrationCaptureController.reset());
    if (m_hasLatestFrame && !m_trackingOverlayActive) {
        m_view.showFrame(m_latestFrame);
    }
}

bool UiController::dispatchAutomaticCalibrationTemplatePoses()
{
    if (!m_robotDeviceConnected) {
        const QString message =
            QStringLiteral("自动标定位姿下发失败：机器人未连接或尚未收到有效 D 回包。");
        appendWarning(message);
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return false;
    }

    const QString templatePath =
        m_configRepository.paths().calibPoseTemplateFilePath();
    const auto result =
        m_automaticCalibrationPoseController.makeTemplateCommand(
            templatePath.toStdString());
    if (!result.succeeded || !result.robotCommand.valid) {
        const QString message =
            QStringLiteral("自动标定模板位姿下发失败：%1")
                .arg(QString::fromStdString(result.message));
        appendWarning(message);
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return false;
    }

    appendInfo(QStringLiteral("自动标定模板位姿已投递：points=%1, file=%2")
                   .arg(result.robotCommand.pointCount)
                   .arg(templatePath));
    m_runtimeCoordinator.sendCorrectionCommand(result.robotCommand);
    return true;
}

bool UiController::dispatchAutomaticCalibrationCorrectedPoses()
{
    if (!m_robotDeviceConnected) {
        const QString message =
            QStringLiteral("自动标定位姿修正失败：机器人未连接或尚未收到有效 D 回包。");
        appendWarning(message);
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return false;
    }

    const QString templatePath =
        m_configRepository.paths().calibPoseTemplateFilePath();
    const QString correctedPath =
        m_configRepository.paths().calibPoseCorrectedFilePath();
    const auto result =
        m_automaticCalibrationPoseController.makeCorrectedCommand(
            templatePath.toStdString(),
            correctedPath.toStdString(),
            m_robotState);
    if (!result.succeeded || !result.robotCommand.valid) {
        const QString message =
            QStringLiteral("自动标定位姿修正失败：%1")
                .arg(QString::fromStdString(result.message));
        appendWarning(message);
        showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        return false;
    }

    appendInfo(QStringLiteral("自动标定修正位姿已生成并投递：points=%1, output=%2")
                   .arg(result.robotCommand.pointCount)
                   .arg(correctedPath));
    m_runtimeCoordinator.sendCorrectionCommand(result.robotCommand);
    return true;
}

void UiController::onOfflineCorrectionConfigurationRequested()
{
    configureSelectedWorkpieceOfflineCorrection();
}

void UiController::onOfflineCorrectionArchiveSelected(const QString& archiveFilePath)
{
    appendInfo(QStringLiteral("UI 选择离线纠偏存档：%1").arg(archiveFilePath));
    i2r_rlc::control::OfflineCorrectionOptions options;
    options.diagnosticOutputDirectory =
        m_configRepository.paths().outputDir()
            .filePath(QStringLiteral("offline_correction"))
            .toStdString();
    m_runtimeCoordinator.configureOfflineCorrectionArchive(
        archiveFilePath.toStdString(),
        options);
}

void UiController::onStartOfflineCorrectionRequested()
{
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::Offline) {
        cancelPendingCorrectionRobotStart();
        m_view.setOfflineCorrectionStateText(QStringLiteral("离线纠偏：已取消等待启动"));
        appendInfo(QStringLiteral("离线纠偏：已取消等待，不会自动启动 JBI。"));
        return;
    }
    if (!m_offlineCorrectionConfigured && selectedWorkpieceProfile() != nullptr) {
        configureSelectedWorkpieceOfflineCorrection();
    }
    if (!m_offlineCorrectionConfigured) {
        appendWarning(QStringLiteral("离线纠偏启动失败：请先在右侧“当前工件”中点击“选择工件”。"));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("离线纠偏启动失败：请先选择工件并加载示教文件。"));
        return;
    }
    switch (m_offlineCorrectionPhase) {
    case i2r_rlc::control::OfflineCorrectionPhase::WaitingForDetection:
    case i2r_rlc::control::OfflineCorrectionPhase::Detecting:
        appendInfo(QStringLiteral("UI 请求结束离线检测并执行 ICP/O 命令。"));
        m_runtimeCoordinator.finishOfflineCorrectionDetectionAndCompute();
        break;
    case i2r_rlc::control::OfflineCorrectionPhase::ReadyToSend:
    case i2r_rlc::control::OfflineCorrectionPhase::WaitingForRobotAck:
        appendWarning(QStringLiteral("离线纠偏正在等待机器人 D.offline==5 回包，请勿重复点击。"));
        break;
    default:
        const WorkpieceProfile* profile = selectedWorkpieceProfile();
        if (profile == nullptr) {
            appendWarning(QStringLiteral("离线纠偏启动失败：尚未选择工件，无法确定离线纠偏 JBI。"));
            showLegacyWarningPrompt(
                QStringLiteral("提示!"),
                QStringLiteral("离线纠偏启动失败：请先选择工件并加载示教文件。"));
            return;
        }

        const QString offlineJbi = profile->offlineCorrectionJbi.trimmed();
        if (offlineJbi.isEmpty()) {
            appendWarning(QStringLiteral("离线纠偏启动失败：当前工件未配置离线纠偏 JBI。"));
            showLegacyWarningPrompt(
                QStringLiteral("提示!"),
                QStringLiteral("未选择示教文件或错误示教文件!"));
            return;
        }

        m_offlineCompletionPromptShown = false;
        appendInfo(QStringLiteral("离线纠偏等待机器人状态满足后启动 JBI：%1")
                       .arg(offlineJbi));
        beginCorrectionRobotStartGate(CorrectionRobotStartKind::Offline, offlineJbi);
        break;
    }
}

void UiController::onOfflineCorrectionFeaturePanelClosed()
{
    appendInfo(QStringLiteral("离线纠偏面板已收起，重置离线纠偏状态并停止对应视觉跟踪。"));
    cancelPendingCorrectionRobotStart();
    m_offlineCorrectionPhase = i2r_rlc::control::OfflineCorrectionPhase::Idle;
    m_offlineCorrectionConfigured = false;
    m_offlineCompletionPromptShown = false;
    m_offlineSeamWindowClosed = false;
    m_hasOfflineCorrectionUpdate = false;
    m_offlineStartCount = 0;
    m_trackingOverlayActive = false;
    m_runtimeCoordinator.resetOfflineCorrection();
    m_view.setOfflineCorrectionStateText(QStringLiteral("离线纠偏：空闲"));
    if (m_hasLatestFrame) {
        m_view.showFrame(m_latestFrame);
    }
}

void UiController::onOnlineCorrectionFeaturePanelClosed()
{
    appendInfo(QStringLiteral("在线纠偏面板已收起，停止在线跟踪并清除当前在线配置。"));
    cancelOnlineAutoCorrectionWorkflow();
    cancelStandaloneGlobalLocate();
    cancelPendingCorrectionRobotStart();
    m_onlineCorrectionConfigured = false;
    m_trackingOverlayActive = false;
    m_runtimeCoordinator.stopOnlineTracking();
    m_view.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：空闲"));
    m_onlineCorrectionP001Presentation.clear();
    m_view.setOnlineCorrectionP001Text(
        m_onlineCorrectionP001Presentation.text());
    m_view.setVisionStateText(QStringLiteral("视觉：空闲"));
    if (m_hasLatestFrame) {
        m_view.showFrame(m_latestFrame);
    }
}

void UiController::onStartTeachTrajectoryRecordingRequested()
{
    appendInfo(QStringLiteral("UI 请求开始记录示教轨迹。"));
    showLegacyInfoPrompt(
        QStringLiteral("提示!"),
        QStringLiteral("请运行示教程序。"));
    m_runtimeCoordinator.startTeachTrajectoryRecording();
}

void UiController::onSaveTeachTrajectoryRequested(
    const QString& outputDirectory,
    const QString& baseName)
{
    const QString trimmedDirectory = outputDirectory.trimmed();
    const QString trimmedBaseName = baseName.trimmed();
    if (trimmedDirectory.isEmpty()) {
        appendWarning(QStringLiteral("保存示教轨迹失败：请选择输出目录。"));
        showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("请选择输出目录。"));
        return;
    }
    if (trimmedBaseName.isEmpty()) {
        appendWarning(QStringLiteral("保存示教轨迹失败：请输入示教轨迹名称。"));
        showLegacyWarningPrompt(QStringLiteral("提示!"), QStringLiteral("请先输入保存的工件名字"));
        return;
    }
    appendInfo(QStringLiteral("UI 请求保存示教轨迹：%1/%2_W.txt, %1/%2_T.txt")
                   .arg(trimmedDirectory, trimmedBaseName));
    m_teachTrajectorySavePromptPending = true;
    m_runtimeCoordinator.saveTeachTrajectory(trimmedDirectory, trimmedBaseName);
}

void UiController::onTeachTrajectoryFeaturePanelClosed()
{
    appendInfo(QStringLiteral("保存示教轨迹面板已收起，重置示教轨迹记录器。"));
    m_teachTrajectorySavePromptPending = false;
    m_view.setTeachTrajectoryStateText(QStringLiteral("未开始"));
    m_runtimeCoordinator.resetTeachTrajectoryRecording();
}

void UiController::onLoadIbvsConfigRequested()
{
    i2r_rlc::ibvs::IbvsServiceConfig config;
    config.runtimeDefaultsPath =
        findRepoConfigFile(QStringLiteral("ibvs_runtime_defaults.txt")).toStdString();
    config.workpieceProfilesPath =
        findRepoRelativeFile(QStringLiteral("config/workpieces/workpiece_profiles.txt"))
            .toStdString();
    const QString legacyProfilesPath =
        findRepoConfigFile(QStringLiteral("workpiece_profiles.txt"));
    i2r_rlc::ibvs::IbvsWorkpieceProfilesResult profileResult =
        i2r_rlc::ibvs::IbvsTextConfigReader::readWorkpieceProfiles(
            config.workpieceProfilesPath);
    if (!profileResult.error.isOk()) {
        config.workpieceProfilesPath = legacyProfilesPath.toStdString();
        profileResult = i2r_rlc::ibvs::IbvsTextConfigReader::readWorkpieceProfiles(
            config.workpieceProfilesPath);
    }
    if (profileResult.error.isOk()
        && !profileResult.defaultWorkpieceId.empty()) {
        config.workpieceId = profileResult.defaultWorkpieceId;
    }
    config.workpieceType = 4;

    const i2r_rlc::ibvs::IbvsStartResult result =
        m_ibvsService.configure(config);
    const QString message = QString::fromStdString(result.message);
    m_view.setIbvsStateText(QStringLiteral("视觉伺服：%1\n%2")
                                .arg(ibvsServiceStateToText(result.state), message));
    if (result.state == i2r_rlc::ibvs::IbvsServiceState::Error) {
        appendWarning(QStringLiteral("视觉伺服参数加载失败：%1").arg(message));
        return;
    }

    appendInfo(QStringLiteral("视觉伺服参数加载完成：%1").arg(message));
}

void UiController::onStartIbvsDryRunRequested()
{
    const i2r_rlc::ibvs::IbvsStartResult result = m_ibvsService.startDryRun();
    const QString message = QString::fromStdString(result.message);
    m_view.setIbvsStateText(QStringLiteral("视觉伺服：%1\n%2")
                                .arg(ibvsServiceStateToText(result.state), message));
    if (result.state == i2r_rlc::ibvs::IbvsServiceState::Error) {
        appendWarning(QStringLiteral("视觉伺服启动失败：%1").arg(message));
        return;
    }

    appendInfo(QStringLiteral("视觉伺服状态：%1").arg(message));
    if (m_hasRobotSimReference && m_lastRobotSimReference.hasIbvsLaserPrior) {
        appendInfo(QStringLiteral("IBVS dry-run 将优先使用 RobotSim 二维激光线先验：点数=%1，焊缝点=%2。")
                       .arg(static_cast<int>(
                           m_lastRobotSimReference.ibvsLaserPrior.virtualLaserLine.size()))
                       .arg(m_lastRobotSimReference.ibvsLaserPrior.hasVirtualWeldPoint
                                ? QStringLiteral("有")
                                : QStringLiteral("无")));
    } else {
        appendWarning(QStringLiteral("IBVS dry-run 未使用 RobotSim 二维先验：尚未查询到可用仿真数据。"));
    }
}

void UiController::onStepIbvsDryRunRequested()
{
    const qint64 frameId = m_lastFrameId >= 0 ? m_lastFrameId : 0;
    i2r_rlc::ibvs::IbvsOnlineFrameInput input = makeSimulatedIbvsFrame(frameId);
    if (m_hasRobotSimReference
        && m_lastRobotSimReference.ibvsLvsObservation.has_value()) {
        input.lvsObservation = m_lastRobotSimReference.ibvsLvsObservation;
        appendInfo(QStringLiteral("IBVS 单步使用 RobotSim LVS 先验：二维线点=%1。")
                       .arg(static_cast<int>(
                           m_lastRobotSimReference.ibvsLaserPrior.virtualLaserLine.size())));
    } else {
        appendWarning(QStringLiteral("IBVS 单步未使用 RobotSim LVS 先验：使用默认 dry-run 输入。"));
    }

    const i2r_rlc::ibvs::IbvsOnlineStepResult result =
        m_ibvsService.processFrame(input);
    showIbvsStepResult(result);
}

void UiController::onStopIbvsDryRunRequested()
{
    const i2r_rlc::ibvs::IbvsStartResult result = m_ibvsService.stop();
    const QString message = QString::fromStdString(result.message);
    m_view.setIbvsStateText(QStringLiteral("视觉伺服：%1\n%2")
                                .arg(ibvsServiceStateToText(result.state), message));
    appendInfo(QStringLiteral("视觉伺服停止：%1").arg(message));
}

void UiController::onRunLocalCorrectionReplayRequested()
{
    appendInfo(QStringLiteral("UI 请求执行局部修正。"));

    i2r_rlc::localization::LocalizationLocalReplayRequest request;
    request.pointCloudRootPath =
        m_configRepository.paths().runtimeRootDir().filePath(QStringLiteral("data/IBVS/pointcloud"));

    const auto result = m_localizationService.runLocalCorrectionReplay(request);
    showLocalizationUpdate(result.update);
    if (!result.replayDirectoryPath.isEmpty()) {
        appendInfo(QStringLiteral("局部修正数据目录：%1").arg(result.replayDirectoryPath));
    }
}

void UiController::onGlobalLocateVirtualLaserConfigurationChanged()
{
    if (m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Idle
        && m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Failed) {
        m_view.setGlobalLocateRobotSimConnectionText(
            QStringLiteral("当前全局定位仍使用启动时锁定的仿真激光和配准点云来源"));
        return;
    }

    const bool useRobotSimLaser =
        m_view.globalLocateVirtualLaserSourceIndex() == 1;
    const bool useRobotSimPointCloud =
        m_view.globalLocateVirtualPointCloudSourceIndex() == 1;
    m_globalLocateVirtualLaserSource = useRobotSimLaser
        ? GlobalLocateVirtualLaserSource::RobotSim
        : GlobalLocateVirtualLaserSource::Occt;
    m_globalLocateVirtualPointCloudSource = useRobotSimPointCloud
        ? GlobalLocateVirtualPointCloudSource::RobotSimLaserScan
        : GlobalLocateVirtualPointCloudSource::Weldment;
    m_runtimeCoordinator.setGlobalLocateVirtualLaserSource(
        m_globalLocateVirtualLaserSource);
    m_runtimeCoordinator.setGlobalLocateVirtualPointCloudSource(
        m_globalLocateVirtualPointCloudSource);
    if (!useRobotSimLaser) {
        m_view.setVirtualWeldPointOverlay({}, false);
    }

    if (!useRobotSimLaser && !useRobotSimPointCloud) {
        m_globalLocateVirtualLaserConfigurationConfirmed = true;
        m_view.setGlobalLocateRobotSimConnectionText(
            m_robotSimConnected
                ? QStringLiteral(
                      "OCCT 激光 + weldment 点云；RobotSim 端口 %1 仍保持实时位姿同步")
                      .arg(m_globalLocateRobotSimEndpoint.port)
                : QStringLiteral("OCCT 激光 + weldment 点云：无需连接 RobotSim"));
        return;
    }

    const int port = m_view.globalLocateRobotSimPort();
    const bool matchesConnectedRealScene =
        m_robotSimConnected
        && !m_robotSimManualModeRequested
        && port != 0
        && port == static_cast<int>(m_globalLocateRobotSimEndpoint.port);
    m_globalLocateVirtualLaserConfigurationConfirmed =
        matchesConnectedRealScene;
    const QString robotSimUsage = useRobotSimLaser && useRobotSimPointCloud
        ? QStringLiteral("仿真激光和配准点云")
        : useRobotSimLaser
            ? QStringLiteral("仿真激光")
            : QStringLiteral("配准点云");
    m_view.setGlobalLocateRobotSimConnectionText(
        matchesConnectedRealScene
            ? QStringLiteral("RobotSim：真实场景已连接，可提供%1")
                  .arg(robotSimUsage)
            : m_robotSimManualConnected
                ? QStringLiteral(
                      "RobotSim：当前为仿真场景连接；%1需要连接真实场景")
                      .arg(robotSimUsage)
                : QStringLiteral(
                      "RobotSim：%1需要先在左侧常驻连接区连接真实场景")
                      .arg(robotSimUsage));
}

void UiController::handleRobotSimConnectionStateChanged(
    bool connected,
    const QString& message)
{
    if (m_robotSimManualModeRequested
        || (!m_robotSimConnectionPending && !m_robotSimConnected)) {
        return;
    }
    m_robotSimManualConnectionPending = false;
    m_robotSimManualConnected = false;
    m_robotSimConnectionPending = false;
    m_robotSimConnected = connected;
    const bool useRobotSim =
        m_view.globalLocateVirtualLaserSourceIndex() == 1
        || m_view.globalLocateVirtualPointCloudSourceIndex() == 1;
    const bool matchesCurrentConfiguration =
        useRobotSim
        && m_view.globalLocateRobotSimPort()
            == static_cast<int>(m_globalLocateRobotSimEndpoint.port);
    m_globalLocateVirtualLaserConfigurationConfirmed =
        useRobotSim ? connected && matchesCurrentConfiguration : true;

    if (connected) {
        m_view.setGlobalLocateRobotSimConnectionText(
            !useRobotSim
                ? QStringLiteral(
                      "OCCT 已选择；RobotSim 端口 %1 通信正常并刷新仿真激光线")
                      .arg(m_globalLocateRobotSimEndpoint.port)
                : matchesCurrentConfiguration
                    ? QStringLiteral(
                          "RobotSim：通信成功，已收到激光响应（端口 %1）\n"
                          "SET_Q_VEC 无响应，执行结果未确认")
                          .arg(m_globalLocateRobotSimEndpoint.port)
                    : QStringLiteral(
                          "RobotSim 端口 %1 通信正常；常驻端口输入已改变")
                          .arg(m_globalLocateRobotSimEndpoint.port));
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：通信成功\n端口：%1\n关节设置：无响应可核验")
                .arg(m_globalLocateRobotSimEndpoint.port));
        appendInfo(QStringLiteral("RobotSim 通信成功：%1").arg(message));
    } else {
        if (useRobotSim) {
            m_view.setVirtualLaserOverlay({});
            m_view.setVirtualWeldPointOverlay({}, false);
        }
        m_view.setGlobalLocateRobotSimConnectionText(
            useRobotSim
                ? QStringLiteral("RobotSim：连接失败（端口 %1）\n%2")
                      .arg(m_globalLocateRobotSimEndpoint.port)
                      .arg(message)
                : QStringLiteral(
                      "OCCT 已选择；RobotSim 实时仿真激光同步已断开：%1")
                      .arg(message));
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：连接失败\n%1").arg(message));
        appendWarning(QStringLiteral("RobotSim 连接失败：%1").arg(message));
    }
}

void UiController::handleRobotSimManualConnectionStateChanged(
    bool connected,
    const QString& message)
{
    if (!m_robotSimManualModeRequested) {
        return;
    }
    if (!m_robotSimManualConnectionPending
        && !m_robotSimManualConnected) {
        return;
    }

    m_robotSimManualConnectionPending = false;
    m_robotSimManualConnected = connected;
    m_robotSimConnected = false;
    if (m_view.globalLocateVirtualLaserSourceIndex() == 1
        || m_view.globalLocateVirtualPointCloudSourceIndex() == 1) {
        m_globalLocateVirtualLaserConfigurationConfirmed = false;
    }

    if (connected) {
        m_view.setDigitalTwinStateText(
            QStringLiteral(
                "仿真：手动场景通信成功\n端口：%1\n"
                "未发送 SET_Q_VEC，正在刷新激光线和焊缝点")
                .arg(m_robotSimManualPort));
        appendInfo(QStringLiteral("RobotSim 手动仿真场景连接成功：%1")
                       .arg(message));
        return;
    }

    m_hasRobotSimReference = false;
    m_lastRobotSimReference = RobotSimVirtualReferenceCache();
    m_view.setVirtualLaserOverlay({});
    m_view.setVirtualWeldPointOverlay({}, false);
    m_view.setDigitalTwinStateText(
        QStringLiteral("仿真：手动场景连接失败\n%1").arg(message));
    appendWarning(QStringLiteral("RobotSim 手动仿真场景连接失败：%1")
                      .arg(message));
}

void UiController::handleRobotSimStateSynchronizationChanged(
    bool succeeded,
    const QString& message)
{
    if (!m_robotSimConnected) {
        return;
    }

    const bool useRobotSim =
        m_view.globalLocateVirtualLaserSourceIndex() == 1
        || m_view.globalLocateVirtualPointCloudSourceIndex() == 1;
    if (succeeded) {
        m_view.setGlobalLocateRobotSimConnectionText(
            useRobotSim
                ? QStringLiteral(
                      "RobotSim：通信正常（端口 %1），持续发送关节并刷新仿真激光线\n"
                      "SET_Q_VEC 执行结果未确认")
                      .arg(m_globalLocateRobotSimEndpoint.port)
                : QStringLiteral(
                      "OCCT 已选择；RobotSim 端口 %1 仍在实时刷新仿真激光线")
                      .arg(m_globalLocateRobotSimEndpoint.port));
        appendInfo(QStringLiteral(
            "RobotSim 已开始持续发送关节值并刷新仿真激光线；"
            "SET_Q_VEC 无响应，模型是否接受需由 RobotSim 端确认。"));
        return;
    }

    m_view.setGlobalLocateRobotSimConnectionText(
        useRobotSim
            ? QStringLiteral("RobotSim：已连接，但位姿/仿真激光同步失败\n%1")
                  .arg(message)
            : QStringLiteral("OCCT 已选择；RobotSim 位姿/仿真激光同步失败：%1")
                  .arg(message));
    appendWarning(
        QStringLiteral("RobotSim 位姿/仿真激光同步失败：%1").arg(message));
}

void UiController::onGlobalLocateRequested()
{
    appendGlobalLocateInfo(QStringLiteral("全局定位步骤 1/7：收到 UI 启动请求。"));

    const bool useRobotSimLaser =
        m_view.globalLocateVirtualLaserSourceIndex() == 1;
    const bool useRobotSimPointCloud =
        m_view.globalLocateVirtualPointCloudSourceIndex() == 1;
    if (useRobotSimLaser || useRobotSimPointCloud) {
        const int port = m_view.globalLocateRobotSimPort();
        if (!m_globalLocateVirtualLaserConfigurationConfirmed
            || !m_robotSimConnected
            || port == 0
            || port != static_cast<int>(
                m_globalLocateRobotSimEndpoint.port)) {
            failStandaloneGlobalLocate(
                QStringLiteral(
                    "全局定位失败：请先在常驻 RobotSim 连接区使用当前端口连接真实场景。"));
            return;
        }
    }

    const WorkpieceProfile* profile = selectedWorkpieceProfile();
    if (profile == nullptr) {
        failStandaloneGlobalLocate(QStringLiteral("全局定位失败：尚未选择工件。"));
        return;
    }
    const QString jbi = profile->offlineLocateJbi.trimmed();
    if (jbi.isEmpty()) {
        failStandaloneGlobalLocate(QStringLiteral("全局定位失败：当前工件未配置全局定位 JBI。"));
        return;
    }
    const QString weldmentPath = resolveWeldmentPath(*profile);
    if (weldmentPath.isEmpty() || !QFileInfo::exists(weldmentPath)) {
        failStandaloneGlobalLocate(
            QStringLiteral("全局定位失败：weldment 模型文件不存在：%1").arg(weldmentPath));
        return;
    }

    cancelStandaloneGlobalLocate();
    m_globalLocateVirtualLaserSource = useRobotSimLaser
        ? GlobalLocateVirtualLaserSource::RobotSim
        : GlobalLocateVirtualLaserSource::Occt;
    m_globalLocateVirtualPointCloudSource = useRobotSimPointCloud
        ? GlobalLocateVirtualPointCloudSource::RobotSimLaserScan
        : GlobalLocateVirtualPointCloudSource::Weldment;
    m_runtimeCoordinator.setGlobalLocateVirtualLaserSource(
        m_globalLocateVirtualLaserSource);
    m_runtimeCoordinator.setGlobalLocateVirtualPointCloudSource(
        m_globalLocateVirtualPointCloudSource);
    m_view.setGlobalLocateVirtualLaserSourceLocked(true);
    const QDir outputRoot = m_configRepository.paths().outputPointCloudsDir();
    const QString runDirectoryName = QStringLiteral("global_locate_%1")
                                         .arg(QDateTime::currentDateTime().toString(
                                             QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    if (!outputRoot.mkpath(runDirectoryName)) {
        failStandaloneGlobalLocate(
            QStringLiteral("全局定位失败：无法创建运行输出目录：%1")
                .arg(outputRoot.filePath(runDirectoryName)));
        return;
    }
    m_standaloneGlobalLocateOutputDirectory =
        outputRoot.filePath(runDirectoryName);
    m_standaloneGlobalLocateJbi = jbi;
    m_standaloneGlobalLocateWeldmentPath = weldmentPath;
    m_standaloneGlobalLocateState =
        StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate;
    m_view.setLocalizationStateText(
        QStringLiteral("定位：等待全局定位启动\nJBI：%1\nweldment：%2")
            .arg(jbi, QFileInfo(weldmentPath).fileName()));
    appendGlobalLocateInfo(
        QStringLiteral("全局定位步骤 2/7：配置检查完成，等待机器人启动门禁。JBI=%1，weldment=%2，SENSORPARA=%3，RBCALIB=%4，输出目录=%5")
            .arg(jbi,
                 weldmentPath,
                 m_configRepository.paths().sensorCalibrationFilePath(),
                 m_configRepository.paths().rbcalibFilePath(),
                 m_standaloneGlobalLocateOutputDirectory));
    appendGlobalLocateInfo(
        QStringLiteral("全局定位步骤 2/7：已加载 SWv1 坐标链标定，cameraInTool=(%1,%2,%3,%4,%5,%6)，positionerToolTcp=(%7,%8,%9) mm。")
            .arg(m_sensorCalibration.cameraXInToolM * 1000.0, 0, 'f', 6)
            .arg(m_sensorCalibration.cameraYInToolM * 1000.0, 0, 'f', 6)
            .arg(m_sensorCalibration.cameraZInToolM * 1000.0, 0, 'f', 6)
            .arg(m_sensorCalibration.cameraRxInToolDeg, 0, 'f', 6)
            .arg(m_sensorCalibration.cameraRyInToolDeg, 0, 'f', 6)
            .arg(m_sensorCalibration.cameraRzInToolDeg, 0, 'f', 6)
            .arg(m_positionerCalibration.toolTcpInLastLinkMm.x(), 0, 'f', 3)
            .arg(m_positionerCalibration.toolTcpInLastLinkMm.y(), 0, 'f', 3)
            .arg(m_positionerCalibration.toolTcpInLastLinkMm.z(), 0, 'f', 3));
    if (m_globalLocateVirtualLaserSource
        == GlobalLocateVirtualLaserSource::Occt) {
        // 门禁等待时间用于提前读取/缓存 STEP，避免 JBI 进入首个
        // 检测窗口后才开始耗时的 OCCT 模型加载。
        appendGlobalLocateInfo(
            QStringLiteral("全局定位步骤 2/7：开始在后台预热 OCCT weldment 模型和 SWv1 虚拟激光传感器。"));
        startGlobalLocatePriorTimer();
    }
    else {
        appendGlobalLocateInfo(
            QStringLiteral(
                "全局定位步骤 2/7：使用 RobotSim 逐帧同步参考，端口=%1，scene=%2，robot=%3，positioner=%4，sensor=%5。")
                .arg(m_globalLocateRobotSimEndpoint.port)
                .arg(m_globalLocateRobotSimScene.sceneName)
                .arg(m_globalLocateRobotSimScene.robotId)
                .arg(m_globalLocateRobotSimScene.positionerId)
                .arg(m_globalLocateRobotSimScene.sensorId));
    }
    beginStandaloneGlobalLocate();
}

void UiController::beginStandaloneGlobalLocate()
{
    if (m_standaloneGlobalLocateState
        != StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate) {
        return;
    }
    beginCorrectionRobotStartGate(
        CorrectionRobotStartKind::GlobalLocateStandalone,
        m_standaloneGlobalLocateJbi);
}

void UiController::cancelStandaloneGlobalLocate()
{
    m_standaloneGlobalLocateApplicationPendingRequestId = 0;
    if (m_pendingCorrectionRobotStartKind
        == CorrectionRobotStartKind::GlobalLocateStandalone) {
        cancelPendingCorrectionRobotStart();
    }
    m_runtimeCoordinator.stopGlobalLocalizationTracking();
    stopGlobalLocatePriorTimer();
    m_view.setVirtualLaserOverlay({});
    m_view.setVirtualWeldPointOverlay({}, false);
    m_view.setRealLaserOverlay({});
    m_standaloneGlobalLocateJobMonitor.reset();
    m_standaloneGlobalLocateState = StandaloneGlobalLocateWorkflowState::Idle;
    m_view.setGlobalLocateVirtualLaserSourceLocked(false);
    onGlobalLocateVirtualLaserConfigurationChanged();
    m_standaloneGlobalLocateJbi.clear();
    m_standaloneGlobalLocateWeldmentPath.clear();
    m_standaloneGlobalLocateOutputDirectory.clear();
    m_standaloneGlobalLocateSegments.clear();
    m_standaloneGlobalLocateRobotSimVirtualSegments.clear();
    m_standaloneGlobalLocateCurrentSegment.clear();
    m_standaloneGlobalLocateActiveSegmentIndex = 0;
    m_standaloneGlobalLocateLastDetectionWindowState = -1;
    resetStandaloneGlobalLocateDiagnostics();
}

void UiController::processStandaloneGlobalLocateRobotState()
{
    if (m_standaloneGlobalLocateState
        != StandaloneGlobalLocateWorkflowState::JobRunning) {
        return;
    }

    const int currentPosflag = m_robotState.posflag == 0 ? 0 : 1;
    if (currentPosflag != m_standaloneGlobalLocateLastRobotPosflag) {
        m_standaloneGlobalLocateLastRobotPosflag = currentPosflag;
        if (currentPosflag == 1) {
            ++m_standaloneGlobalLocateObservedFaceCount;
            appendGlobalLocateInfo(
                QStringLiteral("全局定位 JBI 定位面 %1 开始：检测到 STARTPOSSEG，JBI行=%2；本面内可包含多次 STARTDETPRG。")
                    .arg(m_standaloneGlobalLocateObservedFaceCount)
                    .arg(m_robotState.usJobLine));
        } else {
            appendGlobalLocateInfo(
                QStringLiteral("全局定位 JBI 定位面 %1 结束：检测到 ENDPOSSEG，JBI行=%2。")
                    .arg(m_standaloneGlobalLocateObservedFaceCount)
                    .arg(m_robotState.usJobLine));
        }
    }
    if (!m_standaloneGlobalLocateJobMonitor.update(m_robotState)) {
        return;
    }

    stopGlobalLocatePriorTimer();
    m_view.setVirtualLaserOverlay({});
    m_view.setVirtualWeldPointOverlay({}, false);
    m_view.setRealLaserOverlay({});
    m_standaloneGlobalLocateState =
        StandaloneGlobalLocateWorkflowState::DrainingVision;
    appendGlobalLocateInfo(
        QStringLiteral("全局定位步骤 5/7：JBI 已结束，停止接收新图像，等待最后的在途视觉帧处理完成。"));
    m_runtimeCoordinator.finishGlobalLocalizationTracking();
}

void UiController::completeStandaloneGlobalLocateVisionDrain()
{
    if (m_standaloneGlobalLocateState
        != StandaloneGlobalLocateWorkflowState::DrainingVision) {
        return;
    }
    m_standaloneGlobalLocateState =
        StandaloneGlobalLocateWorkflowState::Applying;
    int totalRealPointCount = 0;
    for (const auto& segment : m_standaloneGlobalLocateSegments) {
        totalRealPointCount += static_cast<int>(segment.size());
    }
    totalRealPointCount +=
        static_cast<int>(m_standaloneGlobalLocateCurrentSegment.size());
    appendGlobalLocateInfo(
        QStringLiteral("全局定位步骤 5/7：在途视觉帧已处理完成，开始保存真实点云并生成虚拟点云。有效帧=%1，拒绝帧=%2，无效帧=%3，已采集真实点=%4")
            .arg(m_standaloneGlobalLocateValidFrameCount)
            .arg(m_standaloneGlobalLocateTriggerRejectedFrameCount)
            .arg(m_standaloneGlobalLocateInvalidFrameCount)
            .arg(totalRealPointCount));
    m_view.setLocalizationStateText(
        QStringLiteral("定位：正在执行全局定位\n已采集点：%1")
            .arg(static_cast<int>(totalRealPointCount)));
    applyStandaloneGlobalLocalization();
}

void UiController::applyStandaloneGlobalLocalization()
{
    if (m_standaloneGlobalLocateState
        != StandaloneGlobalLocateWorkflowState::Applying) {
        return;
    }

    // 若 JBI 结束时最后一个定位面仍有已处理结果，结算该定位面。
    if (m_standaloneGlobalLocateActiveSegmentIndex > 0) {
        finishStandaloneGlobalLocateCurrentSegment();
    }
    // 定位面数量以 RobotComm 直接观察到的 STARTPOSSEG 边沿为准。即使某一面
    // 没有任何有效 Steger 点，也保留一个空槽，避免误报成“JBI 只有一个面”。
    while (static_cast<int>(m_standaloneGlobalLocateSegments.size())
           < m_standaloneGlobalLocateObservedFaceCount) {
        m_standaloneGlobalLocateSegments.emplace_back();
    }

    StandaloneGlobalLocalizationRequest request;
    request.frameId = m_lastFrameId;
    request.outputDirectory = m_standaloneGlobalLocateOutputDirectory;
    request.weldmentPath = m_standaloneGlobalLocateWeldmentPath;
    request.virtualPointCloudSource = m_globalLocateVirtualPointCloudSource;
    request.realSegments = std::move(m_standaloneGlobalLocateSegments);
    request.robotSimVirtualSegments =
        std::move(m_standaloneGlobalLocateRobotSimVirtualSegments);
    request.trackingFrameCount = m_standaloneGlobalLocateTrackingFrameCount;
    request.triggerRejectedFrameCount =
        m_standaloneGlobalLocateTriggerRejectedFrameCount;
    request.invalidFrameCount = m_standaloneGlobalLocateInvalidFrameCount;
    request.priorSuccessCount = m_standaloneGlobalLocatePriorSuccessCount;
    request.priorFailureCount = m_standaloneGlobalLocatePriorFailureCount;
    request.lastTrackingMessage = m_standaloneGlobalLocateLastTrackingMessage;
    request.lastPriorMessage = m_standaloneGlobalLocateLastPriorMessage;

    m_standaloneGlobalLocateApplicationPendingRequestId =
        ++m_standaloneGlobalLocateApplicationNextRequestId;
    appendGlobalLocateInfo(
        QStringLiteral(
            "全局定位步骤 6/7：已把点云快照提交到定位工作线程，"
            "界面可继续响应操作。request=%1")
            .arg(m_standaloneGlobalLocateApplicationPendingRequestId));
    m_view.setLocalizationStateText(
        QStringLiteral("定位：后台执行分段/整体配准\n界面可继续操作"));
    m_runtimeCoordinator.applyStandaloneGlobalLocalization(
        m_standaloneGlobalLocateApplicationPendingRequestId,
        std::move(request));
}

void UiController::handleStandaloneGlobalLocalizationResult(
    quint64 requestId,
    const StandaloneGlobalLocalizationResult &result)
{
    if (requestId != m_standaloneGlobalLocateApplicationPendingRequestId
        || m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Applying) {
        return;
    }
    m_standaloneGlobalLocateApplicationPendingRequestId = 0;

    for (const QString &message : result.infoMessages) {
        appendGlobalLocateInfo(message);
    }
    for (const QString &message : result.warningMessages) {
        appendGlobalLocateWarning(message);
    }
    if (result.hasSegmentedUpdate) {
        showLocalizationUpdate(result.segmentedUpdate);
    }
    if (result.hasWholeUpdate) {
        showLocalizationUpdate(result.wholeUpdate);
    }

    if (!result.succeeded) {
        failStandaloneGlobalLocate(
            result.message.isEmpty()
                ? QStringLiteral("全局定位失败：定位工作线程未返回错误原因。")
                : result.message);
        return;
    }

    m_view.setRealLaserOverlay({});
    m_standaloneGlobalLocateState = StandaloneGlobalLocateWorkflowState::Idle;
    m_view.setGlobalLocateVirtualLaserSourceLocked(false);
    onGlobalLocateVirtualLaserConfigurationChanged();
}

void UiController::finishStandaloneGlobalLocateCurrentSegment()
{
    if (m_standaloneGlobalLocateActiveSegmentIndex <= 0) {
        return;
    }
    const int segmentIndex = m_standaloneGlobalLocateActiveSegmentIndex;
    while (static_cast<int>(m_standaloneGlobalLocateSegments.size())
           < segmentIndex - 1) {
        m_standaloneGlobalLocateSegments.emplace_back();
    }
    if (m_standaloneGlobalLocateCurrentSegment.empty()) {
        appendGlobalLocateWarning(
            QStringLiteral("全局定位：定位面 %1 已结束，但本面没有有效三维点。")
                .arg(segmentIndex));
        m_standaloneGlobalLocateSegments.emplace_back();
        m_standaloneGlobalLocateActiveSegmentIndex = 0;
        return;
    }
    const int pointCount =
        static_cast<int>(m_standaloneGlobalLocateCurrentSegment.size());
    m_standaloneGlobalLocateSegments.push_back(
        std::move(m_standaloneGlobalLocateCurrentSegment));
    m_standaloneGlobalLocateCurrentSegment.clear();
    m_standaloneGlobalLocateActiveSegmentIndex = 0;
    appendGlobalLocateInfo(
        QStringLiteral("全局定位步骤 4/7：定位面 %1 扫描结束，累计 %2 个三维点。")
            .arg(segmentIndex)
            .arg(pointCount));
}

void UiController::collectStandaloneGlobalLocateRobotSimVirtualPoints(
    int segmentIndex,
    const std::vector<i2r_rlc::digital_twin::DigitalTwinPoint3D_mm>& points)
{
    if (m_globalLocateVirtualPointCloudSource
            != GlobalLocateVirtualPointCloudSource::RobotSimLaserScan
        || (m_standaloneGlobalLocateState
                != StandaloneGlobalLocateWorkflowState::JobRunning
            && m_standaloneGlobalLocateState
                != StandaloneGlobalLocateWorkflowState::DrainingVision)
        || segmentIndex <= 0
        || points.empty()) {
        return;
    }

    while (static_cast<int>(
               m_standaloneGlobalLocateRobotSimVirtualSegments.size())
           < segmentIndex) {
        m_standaloneGlobalLocateRobotSimVirtualSegments.emplace_back();
    }

    auto& segmentPoints =
        m_standaloneGlobalLocateRobotSimVirtualSegments[
            static_cast<std::size_t>(segmentIndex - 1)];
    segmentPoints.reserve(segmentPoints.size() + points.size());
    for (const auto& point : points) {
        segmentPoints.push_back({point.x_mm, point.y_mm, point.z_mm});
    }
}

void UiController::collectStandaloneGlobalLocateScanPoint(
    const RuntimeTrackingResult& result)
{
    if (m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::JobRunning
        && m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::DrainingVision) {
        return;
    }

    // RuntimeCoordinator 在相机帧到达时冻结机器人状态，并为每次
    // STARTPOSSEG 分配递增面序号。即使 Vision 队列跳过了中间 posflag=0
    // 图像，第二个定位面仍会携带 segmentIndex=2，不会并入定位面 1。
    int segmentIndex = result.localizationSegmentIndex;
    if (segmentIndex < 0) {
        segmentIndex = result.posflag == 0
            ? 0
            : (m_standaloneGlobalLocateActiveSegmentIndex > 0
                   ? m_standaloneGlobalLocateActiveSegmentIndex
                   : static_cast<int>(m_standaloneGlobalLocateSegments.size()) + 1);
    }
    if (segmentIndex != m_standaloneGlobalLocateActiveSegmentIndex) {
        if (m_standaloneGlobalLocateActiveSegmentIndex > 0) {
            finishStandaloneGlobalLocateCurrentSegment();
        }
        if (segmentIndex > 0) {
            m_standaloneGlobalLocateActiveSegmentIndex = segmentIndex;
            m_standaloneGlobalLocateCurrentSegment.clear();
            appendGlobalLocateInfo(
                QStringLiteral("全局定位步骤 4/7：开始采集定位面 %1，frame=%2，JBI行=%3，traflag=%4，posflag=%5。")
                    .arg(segmentIndex)
                    .arg(result.frameId)
                    .arg(result.robotJobLine)
                    .arg(result.traflag)
                    .arg(result.posflag));
        }
    }

    auto updateCollectionStateText = [this]() {
        int totalPointCount = 0;
        for (const auto& segment : m_standaloneGlobalLocateSegments) {
            totalPointCount += static_cast<int>(segment.size());
        }
        totalPointCount +=
            static_cast<int>(m_standaloneGlobalLocateCurrentSegment.size());
        const QString currentFaceText =
            m_standaloneGlobalLocateActiveSegmentIndex > 0
            ? QString::number(m_standaloneGlobalLocateActiveSegmentIndex)
            : QStringLiteral("定位面外");
        m_view.setLocalizationStateText(
            QStringLiteral("定位：全局定位程序运行中\n全局定位JBI：%1\n已采集点：%2\n当前定位面：%3")
                .arg(m_standaloneGlobalLocateJbi)
                .arg(totalPointCount)
                .arg(currentFaceText));
        return totalPointCount;
    };

    if (!result.valid) {
        // 定位面状态独立于 Steger 是否提取成功；即使当前帧无有效点，UI 也必须
        // 立即显示第二个 STARTPOSSEG，而不能一直停留在“定位面 1”。
        updateCollectionStateText();
        return;
    }

    // 整条激光线重建点优先；否则回退在线纠偏式单点。
    std::vector<i2r_rlc::localization::Point3D_mm> framePoints;
    if (!result.linePoints.empty()) {
        framePoints.reserve(result.linePoints.size());
        for (const auto& point : result.linePoints) {
            if (point.valid) {
                framePoints.push_back({
                    point.xMm,
                    point.yMm,
                    point.zMm});
            }
        }
    } else if (result.sample.point.valid) {
        framePoints.push_back({
            result.sample.point.xMm,
            result.sample.point.yMm,
            result.sample.point.zMm});
    }
    if (framePoints.empty()) {
        updateCollectionStateText();
        return;
    }

    if (segmentIndex > 0 && result.traflag == 1) {
        m_standaloneGlobalLocateCurrentSegment.insert(
            m_standaloneGlobalLocateCurrentSegment.end(),
            framePoints.begin(),
            framePoints.end());
    } else {
        // SWv1 只在定位面内的检测窗口采点；窗口外点不能并入任一定位面。
        m_standaloneGlobalLocateDiscardedPointCount +=
            static_cast<int>(framePoints.size());
    }

    const int totalPointCount = updateCollectionStateText();
    if (m_standaloneGlobalLocateValidFrameCount == 1
        || (m_standaloneGlobalLocateValidFrameCount % 30) == 0) {
        appendGlobalLocateInfo(
            QStringLiteral("全局定位采集进度：validFrames=%1，collectedPoints=%2，discardedOutsideWindow=%3，定位面=%4，traflag=%5，posflag=%6")
                .arg(m_standaloneGlobalLocateValidFrameCount)
                .arg(totalPointCount)
                .arg(m_standaloneGlobalLocateDiscardedPointCount)
                .arg(m_standaloneGlobalLocateActiveSegmentIndex)
                .arg(result.traflag)
                .arg(result.posflag));
    }
}

void UiController::failStandaloneGlobalLocate(const QString& message)
{
    m_standaloneGlobalLocateApplicationPendingRequestId = 0;
    m_standaloneGlobalLocateState = StandaloneGlobalLocateWorkflowState::Failed;
    m_view.setGlobalLocateVirtualLaserSourceLocked(false);
    onGlobalLocateVirtualLaserConfigurationChanged();
    m_view.setVirtualLaserOverlay({});
    m_view.setVirtualWeldPointOverlay({}, false);
    m_view.setRealLaserOverlay({});
    m_standaloneGlobalLocateJobMonitor.reset();
    m_runtimeCoordinator.stopGlobalLocalizationTracking();
    stopGlobalLocatePriorTimer();
    if (m_pendingCorrectionRobotStartKind
        == CorrectionRobotStartKind::GlobalLocateStandalone) {
        cancelPendingCorrectionRobotStart();
    }
    appendGlobalLocateWarning(message);
    m_view.setLocalizationStateText(QStringLiteral("定位：失败\n%1").arg(message));
    showLegacyWarningPrompt(QStringLiteral("提示!"), message);
    m_standaloneGlobalLocateSegments.clear();
    m_standaloneGlobalLocateRobotSimVirtualSegments.clear();
    m_standaloneGlobalLocateCurrentSegment.clear();
    m_standaloneGlobalLocateActiveSegmentIndex = 0;
}

void UiController::appendGlobalLocateInfo(const QString& message)
{
    appendInfo(message);
    qInfo().noquote() << Logger::info(message);
}

void UiController::appendGlobalLocateWarning(const QString& message)
{
    appendWarning(message);
    qWarning().noquote() << Logger::warning(message);
}

void UiController::resetStandaloneGlobalLocateDiagnostics()
{
    m_standaloneGlobalLocateLastRobotPosflag = 0;
    m_standaloneGlobalLocateObservedFaceCount = 0;
    m_standaloneGlobalLocateTrackingFrameCount = 0;
    m_standaloneGlobalLocateTriggerRejectedFrameCount = 0;
    m_standaloneGlobalLocateInvalidFrameCount = 0;
    m_standaloneGlobalLocateValidFrameCount = 0;
    m_standaloneGlobalLocateDiscardedPointCount = 0;
    m_standaloneGlobalLocateLastDetectionWindowState = -1;
    m_standaloneGlobalLocatePriorSuccessCount = 0;
    m_standaloneGlobalLocatePriorFailureCount = 0;
    m_standaloneGlobalLocateLastPriorSegmentIndex = 0;
    m_standaloneGlobalLocatePriorReady = false;
    m_standaloneGlobalLocateLastTrackingMessage.clear();
    m_standaloneGlobalLocateLastPriorMessage.clear();
}

void UiController::startGlobalLocatePriorTimer()
{
    if (m_globalLocateVirtualLaserSource
        != GlobalLocateVirtualLaserSource::Occt) {
        return;
    }
    stopGlobalLocatePriorTimer();
    m_globalLocatePriorTimer = new QTimer(this);
    // 先验计算由独立线程执行。定时器只在上一请求结束后投递下一次，避免
    // OpenCASCADE 求交慢于相机帧率时形成无界请求队列。
    m_globalLocatePriorTimer->setInterval(100);
    connect(
        m_globalLocatePriorTimer,
        &QTimer::timeout,
        this,
        &UiController::publishGlobalLocateVirtualPrior);
    m_globalLocatePriorTimer->start();
    publishGlobalLocateVirtualPrior();
}

void UiController::stopGlobalLocatePriorTimer()
{
    m_globalLocatePriorRequestInFlight = false;
    m_globalLocatePriorPendingRequestId = 0;
    if (m_globalLocatePriorTimer == nullptr) {
        return;
    }
    m_globalLocatePriorTimer->stop();
    m_globalLocatePriorTimer->deleteLater();
    m_globalLocatePriorTimer = nullptr;
}

void UiController::publishGlobalLocateVirtualPrior()
{
    if (m_globalLocateVirtualLaserSource
        != GlobalLocateVirtualLaserSource::Occt) {
        return;
    }
    if ((m_standaloneGlobalLocateState
             != StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate
         && m_standaloneGlobalLocateState
             != StandaloneGlobalLocateWorkflowState::JobRunning)
        || m_globalLocatePriorRequestInFlight) {
        return;
    }

    const bool prewarming =
        m_standaloneGlobalLocateState
        == StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate;
    // SWv1 只在 STARTPOSSEG 内且 traflag==1 的 STARTDETPRG 窗口生成并消费
    // 当前定位面的虚拟激光线。定位面外求交既无意义，也会产生大量误报警。
    if (!prewarming
        && (m_robotState.posflag == 0
            || m_robotState.traflag != 1
            || m_standaloneGlobalLocateObservedFaceCount <= 0)) {
        return;
    }

    QString unavailableReason;
    if (!m_sensorCalibration.valid) {
        unavailableReason = QStringLiteral("SENSORPARA 传感器标定无效");
    } else if (!m_positionerCalibration.valid) {
        unavailableReason = QStringLiteral("RBCALIB 变位机标定无效");
    }
    if (!unavailableReason.isEmpty()) {
        ++m_standaloneGlobalLocatePriorFailureCount;
        if (unavailableReason != m_standaloneGlobalLocateLastPriorMessage) {
            m_standaloneGlobalLocateLastPriorMessage = unavailableReason;
            appendGlobalLocateWarning(
                QStringLiteral("全局定位步骤 3/7：虚拟激光先验不可用，reason=%1")
                    .arg(unavailableReason));
        }
        return;
    }

    i2r_rlc::digital_twin::WeldmentVirtualLaserRequest request;
    request.weldmentPath = m_standaloneGlobalLocateWeldmentPath.toStdString();
    request.toolXmm = static_cast<double>(m_robotState.curx_r) / 1000.0;
    request.toolYmm = static_cast<double>(m_robotState.cury_r) / 1000.0;
    request.toolZmm = static_cast<double>(m_robotState.curz_r) / 1000.0;
    request.toolRxDeg = static_cast<double>(m_robotState.curRx_r) / 10000.0;
    request.toolRyDeg = static_cast<double>(m_robotState.curRy_r) / 10000.0;
    request.toolRzDeg = static_cast<double>(m_robotState.curRz_r) / 10000.0;
    request.stationAxis1Deg = static_cast<double>(m_robotState.curS1) / 10000.0;
    request.stationAxis2Deg = static_cast<double>(m_robotState.curS2) / 10000.0;
    request.positioningSegmentIndex = prewarming
        ? 0
        : m_standaloneGlobalLocateObservedFaceCount;

    request.calibration.focalLengthM = m_sensorCalibration.focalLengthM;
    request.calibration.kappa = m_sensorCalibration.kappa;
    request.calibration.sxM = m_sensorCalibration.sxM;
    request.calibration.syM = m_sensorCalibration.syM;
    request.calibration.cxPixel = m_sensorCalibration.cxPixel;
    request.calibration.cyPixel = m_sensorCalibration.cyPixel;
    request.calibration.laserPlaneA = m_sensorCalibration.laserPlaneA;
    request.calibration.laserPlaneB = m_sensorCalibration.laserPlaneB;
    request.calibration.laserPlaneC = m_sensorCalibration.laserPlaneC;
    request.calibration.cameraXInToolM = m_sensorCalibration.cameraXInToolM;
    request.calibration.cameraYInToolM = m_sensorCalibration.cameraYInToolM;
    request.calibration.cameraZInToolM = m_sensorCalibration.cameraZInToolM;
    request.calibration.cameraRxInToolDeg = m_sensorCalibration.cameraRxInToolDeg;
    request.calibration.cameraRyInToolDeg = m_sensorCalibration.cameraRyInToolDeg;
    request.calibration.cameraRzInToolDeg = m_sensorCalibration.cameraRzInToolDeg;
    request.calibration.valid = m_sensorCalibration.valid;
    request.stationBaseInRobotBase = m_positionerCalibration.stationBaseInRobotBase;
    request.positionerToolTcpInLastLinkMm =
        m_positionerCalibration.toolTcpInLastLinkMm;

    if (!m_latestFrame.image.isNull()) {
        request.imageWidth = m_latestFrame.image.width();
        request.imageHeight = m_latestFrame.image.height();
    } else if (!m_latestFrame.previewImage.isNull()) {
        request.imageWidth = m_latestFrame.previewImage.width();
        request.imageHeight = m_latestFrame.previewImage.height();
    }

    m_globalLocatePriorRequestInFlight = true;
    m_globalLocatePriorPendingRequestId = ++m_globalLocatePriorNextRequestId;
    m_globalLocatePriorPendingRequest = request;
    m_runtimeCoordinator.requestGlobalLocateVirtualPrior(
        m_globalLocatePriorPendingRequestId,
        request);
}

void UiController::handleGlobalLocateVirtualPriorResult(
    quint64 requestId,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation& observation)
{
    if (requestId != m_globalLocatePriorPendingRequestId) {
        return;
    }
    const auto completedRequest = m_globalLocatePriorPendingRequest;
    m_globalLocatePriorRequestInFlight = false;
    m_globalLocatePriorPendingRequestId = 0;
    if (m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate
        && m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::JobRunning) {
        return;
    }
    const bool prewarming =
        m_standaloneGlobalLocateState
        == StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate;

    if (!prewarming
        && (m_robotState.traflag != 1
            || m_robotState.posflag == 0
            || completedRequest.positioningSegmentIndex
                != m_standaloneGlobalLocateObservedFaceCount)) {
        return;
    }

    if (observation.valid) {
        if (!prewarming) {
            ++m_standaloneGlobalLocatePriorSuccessCount;
        }
        m_standaloneGlobalLocateLastPriorMessage = observation.message;
        if (!prewarming
            && observation.positioningSegmentIndex
                != m_standaloneGlobalLocateLastPriorSegmentIndex) {
            m_standaloneGlobalLocateLastPriorSegmentIndex =
                observation.positioningSegmentIndex;
            appendGlobalLocateInfo(
                QStringLiteral("全局定位步骤 3/7：定位面 %1 的 OCCT 虚拟激光参考线生成成功并发布到图像，points=%2。%3")
                    .arg(observation.positioningSegmentIndex)
                    .arg(static_cast<int>(
                        observation.laserLineImagePoints_px.size()))
                    .arg(observation.message));
        }
        if (!m_standaloneGlobalLocatePriorReady) {
            m_standaloneGlobalLocatePriorReady = true;
            appendGlobalLocateInfo(
                (prewarming
                     ? QStringLiteral("全局定位步骤 2/7：OCCT 模型预热完成，虚拟激光先验已就绪。%1")
                     : QStringLiteral("全局定位步骤 3/7：虚拟激光先验首次生成成功。%1"))
                    .arg(observation.message));
        }
        QVector<QPointF> overlayPoints;
        overlayPoints.reserve(
            static_cast<int>(observation.laserLineImagePoints_px.size()));
        for (const auto& point : observation.laserLineImagePoints_px) {
            overlayPoints.push_back(QPointF(point.x_px, point.y_px));
        }
        m_view.setVirtualLaserOverlay(overlayPoints);
        m_view.setVirtualWeldPointOverlay({}, false);
        m_runtimeCoordinator.publishDigitalTwinVisionObservation(observation);
        return;
    }

    // 门禁姿态尚未进入工件视野时没有射线交点是正常现象；模型已在专用
    // worker 中缓存，JBI 启动后会继续以实时位姿求交，不污染运行期失败统计。
    if (prewarming) {
        return;
    }
    ++m_standaloneGlobalLocatePriorFailureCount;
    if (observation.message != m_standaloneGlobalLocateLastPriorMessage) {
        m_standaloneGlobalLocateLastPriorMessage = observation.message;
        appendGlobalLocateWarning(
            QStringLiteral("全局定位步骤 3/7：虚拟激光先验生成失败，定位面=%1，reason=%2，tool=(%3,%4,%5,%6,%7,%8)，station=(%9,%10)，positionerToolTcp=(%11,%12,%13) mm，image=%14x%15")
                .arg(completedRequest.positioningSegmentIndex)
                .arg(observation.message)
                .arg(completedRequest.toolXmm, 0, 'f', 3)
                .arg(completedRequest.toolYmm, 0, 'f', 3)
                .arg(completedRequest.toolZmm, 0, 'f', 3)
                .arg(completedRequest.toolRxDeg, 0, 'f', 3)
                .arg(completedRequest.toolRyDeg, 0, 'f', 3)
                .arg(completedRequest.toolRzDeg, 0, 'f', 3)
                .arg(completedRequest.stationAxis1Deg, 0, 'f', 3)
                .arg(completedRequest.stationAxis2Deg, 0, 'f', 3)
                .arg(completedRequest.positionerToolTcpInLastLinkMm.x(), 0, 'f', 3)
                .arg(completedRequest.positionerToolTcpInLastLinkMm.y(), 0, 'f', 3)
                .arg(completedRequest.positionerToolTcpInLastLinkMm.z(), 0, 'f', 3)
                .arg(completedRequest.imageWidth)
                .arg(completedRequest.imageHeight));
    }
}

void UiController::handleGlobalLocateFrameVirtualPriorResult(
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation& observation)
{
    if (m_standaloneGlobalLocateState
                != StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate
            && m_standaloneGlobalLocateState
                != StandaloneGlobalLocateWorkflowState::JobRunning
            && m_standaloneGlobalLocateState
                != StandaloneGlobalLocateWorkflowState::DrainingVision) {
        return;
    }

    if (m_globalLocateVirtualPointCloudSource
        == GlobalLocateVirtualPointCloudSource::RobotSimLaserScan) {
        collectStandaloneGlobalLocateRobotSimVirtualPoints(
            observation.positioningSegmentIndex,
            observation.laserLinePoints_mm);
    }

    if (m_globalLocateVirtualLaserSource
        != GlobalLocateVirtualLaserSource::RobotSim) {
        return;
    }

    if (!observation.valid) {
        m_view.setVirtualLaserOverlay({});
        m_view.setVirtualWeldPointOverlay({}, false);
        ++m_standaloneGlobalLocatePriorFailureCount;
        if (observation.message != m_standaloneGlobalLocateLastPriorMessage) {
            m_standaloneGlobalLocateLastPriorMessage = observation.message;
            appendGlobalLocateWarning(
                QStringLiteral(
                    "全局定位 RobotSim 帧同步失败：frame=%1，定位面=%2，reason=%3")
                    .arg(observation.frameId)
                    .arg(observation.positioningSegmentIndex)
                    .arg(observation.message));
        }
        return;
    }

    ++m_standaloneGlobalLocatePriorSuccessCount;
    m_standaloneGlobalLocateLastPriorMessage = observation.message;
    QVector<QPointF> overlayPoints;
    overlayPoints.reserve(
        static_cast<int>(observation.laserLineImagePoints_px.size()));
    for (const auto &point : observation.laserLineImagePoints_px) {
        overlayPoints.push_back(QPointF(point.x_px, point.y_px));
    }
    m_view.setVirtualLaserOverlay(overlayPoints);
    m_view.setVirtualWeldPointOverlay(
        QPointF(
            observation.weldPointImage_px.x_px,
            observation.weldPointImage_px.y_px),
        observation.hasWeldPointImage_px);

    if (!m_standaloneGlobalLocatePriorReady) {
        m_standaloneGlobalLocatePriorReady = true;
        appendGlobalLocateInfo(
            QStringLiteral(
                "全局定位步骤 3/7：RobotSim 逐帧先验已就绪，frame=%1，points=%2。")
                .arg(observation.frameId)
                .arg(static_cast<int>(
                    observation.laserLineImagePoints_px.size())));
    }
}

void UiController::handleOnlineAutoGlobalLocateVirtualPriorResult(
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation& observation)
{
    if (!m_onlineAutoCorrectionWorkflow.acceptsGlobalLocalizationFrame()
        || observation.laserLinePoints_mm.empty()) {
        return;
    }

    m_onlineAutoGlobalLocalizationVirtualPoints.reserve(
        m_onlineAutoGlobalLocalizationVirtualPoints.size()
        + observation.laserLinePoints_mm.size());
    for (const auto& point : observation.laserLinePoints_mm) {
        m_onlineAutoGlobalLocalizationVirtualPoints.push_back({
            point.x_mm, point.y_mm, point.z_mm});
    }
}

void UiController::handleRobotSimRealtimeVirtualPriorResult(
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation& observation)
{
    // 纠偏显示由完整检测帧独占，拒绝其他通道迟到的参考覆盖。
    if (m_onlineCorrectionSyncState.acceptingFrames
        || m_offlineCorrectionSyncState.acceptingFrames) {
        return;
    }
    const bool occtGlobalLocateIsRunning =
        m_globalLocateVirtualLaserSource
            == GlobalLocateVirtualLaserSource::Occt
        && (m_standaloneGlobalLocateState
                == StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate
            || m_standaloneGlobalLocateState
                == StandaloneGlobalLocateWorkflowState::JobRunning
            || m_standaloneGlobalLocateState
                == StandaloneGlobalLocateWorkflowState::DrainingVision);
    if (occtGlobalLocateIsRunning) {
        return;
    }
    if (!observation.valid) {
        m_view.setVirtualLaserOverlay({});
        m_view.setVirtualWeldPointOverlay({}, false);
        return;
    }

    QVector<QPointF> overlayPoints;
    overlayPoints.reserve(
        static_cast<int>(observation.laserLineImagePoints_px.size()));
    for (const auto &point : observation.laserLineImagePoints_px) {
        overlayPoints.push_back(QPointF(point.x_px, point.y_px));
    }
    m_view.setVirtualLaserOverlay(overlayPoints);
    m_view.setVirtualWeldPointOverlay(
        QPointF(
            observation.weldPointImage_px.x_px,
            observation.weldPointImage_px.y_px),
        observation.hasWeldPointImage_px);
}

void UiController::handleRobotSimRealtimeVirtualReferenceResult(
    const i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult& reference)
{
    if (m_onlineCorrectionSyncState.acceptingFrames
        || m_offlineCorrectionSyncState.acceptingFrames) {
        return;
    }
    if (!m_robotSimConnected || m_robotSimManualModeRequested) {
        return;
    }

    m_lastRobotSimReference = makeRobotSimVirtualReferenceCache(
        reference,
        m_lastFrameId,
        m_globalLocateRobotSimScene.sensorId);
    m_hasRobotSimReference = m_lastRobotSimReference.available;

    QVector<QPointF> overlayPoints;
    overlayPoints.reserve(
        static_cast<int>(reference.laserLineImagePoints_px.size()));
    for (const auto& point : reference.laserLineImagePoints_px) {
        overlayPoints.push_back(QPointF(point.x_px, point.y_px));
    }
    const bool occtGlobalLocateIsRunning =
        m_globalLocateVirtualLaserSource
            == GlobalLocateVirtualLaserSource::Occt
        && (m_standaloneGlobalLocateState
                == StandaloneGlobalLocateWorkflowState::WaitingRobotStartGate
            || m_standaloneGlobalLocateState
                == StandaloneGlobalLocateWorkflowState::JobRunning
            || m_standaloneGlobalLocateState
                == StandaloneGlobalLocateWorkflowState::DrainingVision);
    if (!occtGlobalLocateIsRunning) {
        m_view.setVirtualLaserOverlay(overlayPoints);
        m_view.setVirtualWeldPointOverlay(
            QPointF(
                reference.seamPointImage_px.x_px,
                reference.seamPointImage_px.y_px),
            reference.hasSeamPointImage_px);
    }

    QString stateText = QStringLiteral(
        "仿真：真实场景通信成功\n端口：%1\n"
        "已发送 SET_Q_VEC\n激光线：%2 图像点 / %3 三维点")
                            .arg(m_globalLocateRobotSimEndpoint.port)
                            .arg(static_cast<int>(
                                reference.laserLineImagePoints_px.size()))
                            .arg(static_cast<int>(
                                reference.laserLinePoints_mm.size()));
    if (reference.hasSeamPointImage_px) {
        stateText += QStringLiteral("\n焊缝点二维：(%1, %2) pixel")
                         .arg(reference.seamPointImage_px.x_px, 0, 'f', 0)
                         .arg(reference.seamPointImage_px.y_px, 0, 'f', 0);
    } else {
        stateText += QStringLiteral("\n焊缝点二维：RobotSim 未返回");
    }
    m_view.setDigitalTwinStateText(stateText);
}

void UiController::handleRobotSimManualVirtualReferenceResult(
    const i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult& reference)
{
    if (m_onlineCorrectionSyncState.acceptingFrames
        || m_offlineCorrectionSyncState.acceptingFrames) {
        return;
    }
    if (!m_robotSimManualConnected) {
        return;
    }

    m_lastRobotSimReference = makeRobotSimVirtualReferenceCache(
        reference,
        m_lastFrameId,
        m_robotSimManualSensorId);
    m_hasRobotSimReference = m_lastRobotSimReference.available;

    QVector<QPointF> overlayPoints;
    overlayPoints.reserve(
        static_cast<int>(reference.laserLineImagePoints_px.size()));
    for (const auto& point : reference.laserLineImagePoints_px) {
        overlayPoints.push_back(QPointF(point.x_px, point.y_px));
    }
    m_view.setVirtualLaserOverlay(overlayPoints);
    m_view.setVirtualWeldPointOverlay(
        QPointF(
            reference.seamPointImage_px.x_px,
            reference.seamPointImage_px.y_px),
        reference.hasSeamPointImage_px);

    const auto observation =
        i2r_rlc::digital_twin::makeVisionLaserObservation(reference);
    m_runtimeCoordinator.publishDigitalTwinVisionObservation(observation);

    QString stateText = QStringLiteral(
        "仿真：手动场景通信成功\n端口：%1\n"
        "未发送 SET_Q_VEC\n激光线：%2 图像点 / %3 三维点")
                            .arg(m_robotSimManualPort)
                            .arg(static_cast<int>(
                                reference.laserLineImagePoints_px.size()))
                            .arg(static_cast<int>(
                                reference.laserLinePoints_mm.size()));
    if (reference.hasSeamPointImage_px) {
        stateText += QStringLiteral("\n焊缝点二维：(%1, %2) pixel")
                         .arg(reference.seamPointImage_px.x_px, 0, 'f', 0)
                         .arg(reference.seamPointImage_px.y_px, 0, 'f', 0);
    } else {
        stateText += QStringLiteral("\n焊缝点二维：RobotSim 未返回");
    }
    m_view.setDigitalTwinStateText(stateText);
}

QString UiController::resolveWeldmentPath(const WorkpieceProfile& profile) const
{
    const QString modelPath = profile.weldmentModel.trimmed();
    if (modelPath.isEmpty()) {
        return {};
    }
    const QFileInfo info(modelPath);
    if (info.isAbsolute()) {
        return modelPath;
    }
    return m_configRepository.paths().runtimeRootDir().filePath(modelPath);
}

void UiController::onClearLocalizationResultRequested()
{
    appendInfo(QStringLiteral("UI 请求清除定位状态。"));
    showLocalizationUpdate(m_localizationService.clear());
}

void UiController::onRobotSimVirtualReferenceRequested(int port)
{
    if (m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Idle
        && m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Failed) {
        appendWarning(QStringLiteral(
            "全局定位正在运行，不能切换 RobotSim 连接场景。"));
        return;
    }
    if (port <= 0 || port > 65535) {
        m_view.setDigitalTwinStateText(QStringLiteral("仿真：端口无效"));
        appendWarning(QStringLiteral("RobotSim 联调失败：端口号无效。"));
        return;
    }
    if (m_robotSimManualConnectionPending) {
        appendWarning(QStringLiteral("RobotSim 手动场景正在连接，请等待本次结果。"));
        return;
    }

    const QString sceneConfigPath =
        m_configRepository.paths().runtimeRootDir().filePath(
            QStringLiteral("config/digital_twin/robotsim_scene.txt"));
    const auto sceneResult =
        i2r_rlc::digital_twin::CRobotSimSceneConfigReader::load(
            sceneConfigPath);
    if (!sceneResult.succeeded) {
        m_view.setDigitalTwinStateText(QStringLiteral("仿真：场景配置无效"));
        appendWarning(sceneResult.message);
        return;
    }

    const QString endpointConfigPath =
        m_configRepository.paths().runtimeRootDir().filePath(
            QStringLiteral("config/digital_twin/robotsim_endpoint.txt"));
    i2r_rlc::digital_twin::RobotSimEndpointConfigReader endpointReader;
    const auto endpointResult = endpointReader.readFile(endpointConfigPath);
    if (!endpointResult.loaded) {
        m_view.setDigitalTwinStateText(QStringLiteral("仿真：端点配置无效"));
        appendWarning(endpointResult.message);
        return;
    }

    auto endpoint = endpointResult.config;
    endpoint.port = static_cast<quint16>(port);

    m_robotSimManualConnectionPending = true;
    m_robotSimManualConnected = false;
    m_robotSimManualModeRequested = true;
    m_robotSimManualPort = endpoint.port;
    m_robotSimManualSensorId = sceneResult.config.sensorId;
    m_robotSimConnected = false;
    m_robotSimConnectionPending = false;
    m_globalLocateVirtualLaserConfigurationConfirmed =
        m_view.globalLocateVirtualLaserSourceIndex() != 1;
    m_view.setDigitalTwinStateText(
        QStringLiteral(
            "仿真：正在连接手动场景\n端口：%1\n"
            "不会发送真实机器人/变位机关节值")
            .arg(port));
    if (m_view.globalLocateVirtualLaserSourceIndex() == 1) {
        m_view.setGlobalLocateRobotSimConnectionText(
            QStringLiteral(
                "RobotSim：手动仿真联调已接管连接；"
                "全局定位前需重新连接真实场景"));
    }
    appendInfo(QStringLiteral(
        "正在连接 RobotSim 手动仿真场景：端口=%1，sensor=%2，"
        "三维参考对象=%3；"
        "仅持续请求激光线和焊缝点，不发送 SET_Q_VEC。")
                   .arg(port)
                   .arg(sceneResult.config.sensorId)
                   .arg(endpoint.referenceObjectId > 0
                       ? QString::number(endpoint.referenceObjectId)
                       : QStringLiteral("世界坐标")));
    m_runtimeCoordinator.connectRobotSimManual(endpoint, sceneResult.config);
}

void UiController::onRobotSimRealSceneConnectionRequested(int port)
{
    if (m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Idle
        && m_standaloneGlobalLocateState
            != StandaloneGlobalLocateWorkflowState::Failed) {
        appendWarning(QStringLiteral(
            "全局定位正在运行，不能切换 RobotSim 连接场景。"));
        return;
    }
    if (port <= 0 || port > 65535) {
        m_view.setDigitalTwinStateText(QStringLiteral("仿真：端口无效"));
        appendWarning(QStringLiteral("RobotSim 真实场景连接失败：端口号无效。"));
        return;
    }
    if (m_robotSimConnectionPending) {
        appendWarning(QStringLiteral("RobotSim 真实场景正在连接，请等待本次结果。"));
        return;
    }

    const QString sceneConfigPath =
        m_configRepository.paths().runtimeRootDir().filePath(
            QStringLiteral("config/digital_twin/robotsim_scene.txt"));
    const auto sceneResult =
        i2r_rlc::digital_twin::CRobotSimSceneConfigReader::load(
            sceneConfigPath);
    if (!sceneResult.succeeded) {
        m_view.setDigitalTwinStateText(QStringLiteral("仿真：场景配置无效"));
        appendWarning(sceneResult.message);
        return;
    }

    const QString endpointConfigPath =
        m_configRepository.paths().runtimeRootDir().filePath(
            QStringLiteral("config/digital_twin/robotsim_endpoint.txt"));
    i2r_rlc::digital_twin::RobotSimEndpointConfigReader endpointReader;
    const auto endpointResult = endpointReader.readFile(endpointConfigPath);
    if (!endpointResult.loaded) {
        m_view.setDigitalTwinStateText(QStringLiteral("仿真：端点配置无效"));
        appendWarning(endpointResult.message);
        return;
    }

    auto endpoint = endpointResult.config;
    endpoint.port = static_cast<quint16>(port);

    m_robotSimManualConnectionPending = false;
    m_robotSimManualConnected = false;
    m_robotSimManualModeRequested = false;
    m_globalLocateRobotSimEndpoint = endpoint;
    m_globalLocateRobotSimScene = sceneResult.config;
    m_globalLocateVirtualLaserConfigurationConfirmed =
        m_view.globalLocateVirtualLaserSourceIndex() != 1;
    m_robotSimConnected = false;
    m_robotSimConnectionPending = true;
    m_view.setDigitalTwinStateText(
        QStringLiteral(
            "仿真：正在连接真实场景\n端口：%1\n"
            "将持续同步真实机器人和变位机关节值")
            .arg(port));
    if (m_view.globalLocateVirtualLaserSourceIndex() == 1) {
        m_view.setGlobalLocateRobotSimConnectionText(
            QStringLiteral("RobotSim：正在连接真实场景（端口 %1）...")
                .arg(port));
    }
    appendInfo(QStringLiteral(
        "正在连接 RobotSim 真实场景：端口=%1，robot=%2，"
        "positioner=%3，sensor=%4，三维参考对象=%5。")
                   .arg(port)
                   .arg(sceneResult.config.robotId)
                   .arg(sceneResult.config.positionerId)
                   .arg(sceneResult.config.sensorId)
                   .arg(endpoint.referenceObjectId > 0
                       ? QString::number(endpoint.referenceObjectId)
                       : QStringLiteral("世界坐标")));
    m_runtimeCoordinator.connectRobotSim(endpoint, sceneResult.config);
}

void UiController::onStopRequested()
{
    appendInfo(QStringLiteral("UI 请求停止。"));
    cancelOnlineAutoCorrectionWorkflow();
    cancelStandaloneGlobalLocate();
    cancelPendingCorrectionRobotStart();
    m_trackingOverlayActive = false;
    m_calibrationGuideInfoEmitted = false;
    m_calibrationGuideClipWarningEmitted = false;
    m_offlineCompletionPromptShown = false;
    m_offlineSeamWindowClosed = false;
    m_hasOfflineCorrectionUpdate = false;
    m_offlineStartCount = 0;
    m_teachTrajectorySavePromptPending = false;
    m_offlineCorrectionPhase =
        i2r_rlc::control::OfflineCorrectionPhase::Idle;
    showCalibrationCaptureUpdate(m_calibrationCaptureController.reset());
    m_runtimeCoordinator.stopOnlineTracking();
    m_runtimeCoordinator.resetOfflineCorrection();
    m_runtimeCoordinator.resetTeachTrajectoryRecording();
    m_runtimeCoordinator.resetRobotSimConnection();
    m_cameraService.stop();
    appendInfo(m_cameraService.stateText());
    onStopIbvsDryRunRequested();
    showRobotState(RobotStateModel{});

    m_robotSimManualConnectionPending = false;
    m_robotSimManualConnected = false;
    m_robotSimManualModeRequested = false;
    m_robotSimConnectionPending = false;
    m_robotSimConnected = false;
    m_globalLocateVirtualLaserSource =
        GlobalLocateVirtualLaserSource::Occt;
    m_globalLocateVirtualPointCloudSource =
        GlobalLocateVirtualPointCloudSource::Weldment;
    m_globalLocateVirtualLaserConfigurationConfirmed = true;
    m_hasRobotSimReference = false;
    m_lastRobotSimReference = RobotSimVirtualReferenceCache();
    m_runtimeCoordinator.setGlobalLocateVirtualLaserSource(
        GlobalLocateVirtualLaserSource::Occt);
    m_runtimeCoordinator.setGlobalLocateVirtualPointCloudSource(
        GlobalLocateVirtualPointCloudSource::Weldment);

    m_view.resetFeaturePanelsAfterStop();
    m_view.setOfflineCorrectionStateText(QStringLiteral("离线纠偏：空闲"));
    m_view.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：空闲"));
    m_onlineCorrectionP001Presentation.clear();
    m_view.setOnlineCorrectionP001Text(
        m_onlineCorrectionP001Presentation.text());
    m_view.setTeachTrajectoryStateText(QStringLiteral("未开始"));
    showLocalizationUpdate(m_localizationService.clear());
    m_view.setGlobalLocateRobotSimConnectionText(
        QStringLiteral("OCCT 激光 + weldment 点云：无需连接 RobotSim"));
    m_view.setDigitalTwinStateText(
        QStringLiteral(
            "仿真：未连接\n请选择连接仿真场景或连接真实场景"));
    if (m_hasLatestFrame) {
        m_view.showFrame(m_latestFrame);
    }
}

void UiController::handleFrameReady(const i2r_rlc::camera::CameraFrame& frame)
{
    m_latestFrame = frame;
    m_hasLatestFrame = true;
    m_lastFrameId = frame.frameId;
    showFrameWithCalibrationGuide(frame);
    m_runtimeCoordinator.publishCameraFrame(frame);
    appendInfo(QStringLiteral("已读取离线图像帧：%1，frameId=%2")
                   .arg(frame.sourcePath)
                   .arg(frame.frameId));

#ifdef I2R_RLC_HAS_OPENCV
    const auto result = processVisionFrame(frame, m_visionOptions, m_visionState);
#else
    const auto result = processVisionFrame(frame);
#endif
    QString visionSummary = QStringLiteral("视觉：%1\n帧：%2\n图像：%3 x %4 pixel")
                                .arg(result.processSucceeded ? QStringLiteral("处理成功") : QStringLiteral("处理失败"))
                                .arg(result.frameId)
                                .arg(result.imageWidthPixel)
                                .arg(result.imageHeightPixel);
    if (result.preprocessingSucceeded) {
        visionSummary += QStringLiteral("\nmean=%1, min=%2, max=%3")
                             .arg(result.meanGrayValue, 0, 'f', 3)
                             .arg(result.minGrayValue, 0, 'f', 0)
                             .arg(result.maxGrayValue, 0, 'f', 0);
    }
    visionSummary += QStringLiteral("\n焊缝检测：%1\nconfidence=%2")
                         .arg(detectionStatusToKey(result.weldDetection.status))
                         .arg(result.weldDetection.confidence, 0, 'f', 3);
    m_view.setVisionStateText(visionSummary);

    const QString logText = QStringLiteral("%1 frameId=%2, image=%3x%4 pixel, channels=%5, mean=%6, min=%7, max=%8, weld=%9, confidence=%10, source=%11, time=%12 ms")
                                .arg(result.message)
                                .arg(result.frameId)
                                .arg(result.imageWidthPixel)
                                .arg(result.imageHeightPixel)
                                .arg(result.imageChannels)
                                .arg(result.meanGrayValue, 0, 'f', 3)
                                .arg(result.minGrayValue, 0, 'f', 0)
                                .arg(result.maxGrayValue, 0, 'f', 0)
                                .arg(detectionStatusToKey(result.weldDetection.status))
                                .arg(result.weldDetection.confidence, 0, 'f', 3)
                                .arg(result.sourcePath.isEmpty() ? QStringLiteral("<empty>") : result.sourcePath)
                                .arg(result.processingTimeMs, 0, 'f', 3);
    if (result.processSucceeded) {
        appendInfo(logText);
    } else {
        appendWarning(logText);
    }

    appendInfo(QStringLiteral("焊缝检测：%1，%2")
                   .arg(detectionStatusToKey(result.weldDetection.status),
                        result.weldDetection.message));

}

void UiController::processAutomaticCalibrationRobotState()
{
#ifdef I2R_RLC_HAS_OPENCV
    const auto phase = m_calibrationCaptureController.state().phase;
    const bool automaticCaptureActive =
        phase == i2r_rlc::control::CalibrationCapturePhase::AutomaticWaitingRobot
        || phase == i2r_rlc::control::CalibrationCapturePhase::AutomaticCapturingBoard
        || phase == i2r_rlc::control::CalibrationCapturePhase::AutomaticCapturingLaser;
    if (!automaticCaptureActive) {
        m_lastCalibrationBcalib = m_robotState.s_Bcalib;
        return;
    }

    const bool risingEdge =
        m_lastCalibrationBcalib == 0 && m_robotState.s_Bcalib != 0;
    const bool releaseEdge =
        m_lastCalibrationBcalib != 0 && m_robotState.s_Bcalib == 0;
    m_lastCalibrationBcalib = m_robotState.s_Bcalib;
    if (!risingEdge && !releaseEdge) {
        return;
    }
    if (releaseEdge) {
        showCalibrationCaptureUpdate(
            m_calibrationCaptureController.onRobotStateChanged(cv::Mat{}, m_robotState));
        return;
    }
    if (!m_hasLatestFrame) {
        appendWarning(QStringLiteral("自动标定采集等待图像帧：已收到 s_Bcalib 上升沿，但当前无可用图像。"));
        return;
    }

    QString errorMessage;
    const QImage image = m_latestFrame.image.isNull()
        ? m_latestFrame.previewImage
        : m_latestFrame.image;
    const cv::Mat cvImage = qImageToCvMat(image, &errorMessage);
    if (cvImage.empty()) {
        appendWarning(QStringLiteral("自动标定采集失败：%1").arg(errorMessage));
        return;
    }

    showCalibrationCaptureUpdate(
        m_calibrationCaptureController.onRobotStateChanged(cvImage, m_robotState));
#endif
}

void UiController::capturePendingAutomaticCalibrationLaserImage()
{
#ifdef I2R_RLC_HAS_OPENCV
    const auto& state = m_calibrationCaptureController.state();
    if (state.mode != i2r_rlc::control::CalibrationCaptureMode::Automatic
        || state.phase
            != i2r_rlc::control::CalibrationCapturePhase::AutomaticCapturingLaser) {
        return;
    }
    if (!m_hasLatestFrame) {
        appendWarning(QStringLiteral("自动标定激光图采集失败：尚未读取图像帧。"));
        return;
    }

    QString errorMessage;
    const QImage image = m_latestFrame.image.isNull()
        ? m_latestFrame.previewImage
        : m_latestFrame.image;
    const cv::Mat cvImage = qImageToCvMat(image, &errorMessage);
    if (cvImage.empty()) {
        appendWarning(QStringLiteral("自动标定激光图采集失败：%1").arg(errorMessage));
        return;
    }

    appendInfo(QStringLiteral("自动标定延时采集同位置激光图。"));
    showCalibrationCaptureUpdate(
        m_calibrationCaptureController.captureLaserImage(cvImage, m_robotState));
#endif
}

void UiController::scheduleAutomaticCalibrationLaserCaptureIfNeeded(
    const i2r_rlc::control::CalibrationCaptureUpdate& update)
{
#ifdef I2R_RLC_HAS_OPENCV
    const bool shouldSchedule =
        update.succeeded
        && update.state.mode == i2r_rlc::control::CalibrationCaptureMode::Automatic
        && update.phase
            == i2r_rlc::control::CalibrationCapturePhase::AutomaticCapturingLaser;
    if (!shouldSchedule) {
        cancelPendingAutomaticCalibrationLaserCapture();
        return;
    }
    if (m_automaticCalibrationLaserTimer == nullptr
        || m_automaticCalibrationLaserTimer->isActive()) {
        return;
    }
    m_automaticCalibrationLaserTimer->start(
        AUTOMATIC_CALIBRATION_LASER_CAPTURE_DELAY_MS);
    appendInfo(QStringLiteral("自动标定将在 %1 ms 后采集同位置激光图。")
                   .arg(AUTOMATIC_CALIBRATION_LASER_CAPTURE_DELAY_MS));
#else
    Q_UNUSED(update);
#endif
}

void UiController::cancelPendingAutomaticCalibrationLaserCapture()
{
    if (m_automaticCalibrationLaserTimer != nullptr
        && m_automaticCalibrationLaserTimer->isActive()) {
        m_automaticCalibrationLaserTimer->stop();
        appendInfo(QStringLiteral("已取消待执行的自动标定激光图采集。"));
    }
}

void UiController::applyCalibrationCaptureHardwareRequest(
    const i2r_rlc::control::CalibrationCaptureUpdate& update)
{
    if (update.requestedExposureUs <= 0) {
        return;
    }

    appendInfo(QStringLiteral("标定采集请求：exposure=%1 us, laser=%2")
                   .arg(update.requestedExposureUs)
                   .arg(update.requestLaserOn ? QStringLiteral("on") : QStringLiteral("off")));

    bool statusPanelNeedsUpdate = false;
    if (m_cameraExposureUs != update.requestedExposureUs) {
        m_cameraExposureUs = update.requestedExposureUs;
        m_view.setExposureControlValue(m_cameraExposureUs);
        m_runtimeCoordinator.setCameraExposureUs(m_cameraExposureUs);
        statusPanelNeedsUpdate = true;
        appendInfo(QStringLiteral("标定采集已设置相机曝光：%1 us。")
                       .arg(m_cameraExposureUs));
    }

    if (m_laserEnabled != update.requestLaserOn) {
        m_laserEnabled = update.requestLaserOn;
        m_view.setLaserControlChecked(m_laserEnabled);
        m_laserCommandPromptPending = true;
        m_runtimeCoordinator.setLaserEnabled(m_laserEnabled);
        statusPanelNeedsUpdate = true;
        appendInfo(m_laserEnabled
                       ? QStringLiteral("标定采集已请求打开激光。")
                       : QStringLiteral("标定采集已请求关闭激光。"));
    }

    if (statusPanelNeedsUpdate) {
        m_view.setLegacyStatusPanel(
            robotModeToLegacyText(m_robotState.sMode),
            servoToLegacyText(m_robotState.sServoPower),
            runToLegacyText(m_robotState.sStart),
            correctionToLegacyText(m_robotState.corflag),
            m_laserEnabled,
            m_filterEnabled,
            robotDeviceText(m_robotDeviceConnected),
            QStringLiteral("--"),
            QString::number(m_cameraExposureUs));
    }
}

void UiController::handleRuntimeFrameReady(
    const i2r_rlc::camera::CameraFrame& frame)
{
    m_latestFrame = frame;
    m_hasLatestFrame = true;
    m_lastFrameId = frame.frameId;
    if (!m_trackingOverlayActive) {
        showFrameWithCalibrationGuide(frame);
    }
}

void UiController::showFrameWithCalibrationGuide(
    const i2r_rlc::camera::CameraFrame& frame)
{
    if (!m_lastCalibrationCaptureUpdate.automaticGuideOverlayActive) {
        m_calibrationGuideClipWarningEmitted = false;
        m_view.showFrame(frame);
        return;
    }

    const QImage source = frame.image.isNull() ? frame.previewImage : frame.image;
    const auto overlay =
        renderCalibrationGuideOverlay(source, m_lastCalibrationCaptureUpdate);
    if (!overlay.drawn) {
        m_view.showFrame(frame);
        return;
    }

    i2r_rlc::camera::CameraFrame displayFrame = frame;
    displayFrame.image = overlay.image;
    displayFrame.previewImage = overlay.image;
    displayFrame.description =
        QStringLiteral("自动标定红框：标定板应进入红框区域");
    m_view.showFrame(displayFrame);

    if (overlay.clipped && !m_calibrationGuideClipWarningEmitted) {
        appendWarning(QStringLiteral(
            "自动标定红框超出当前图像范围，已按图像尺寸裁剪；请确认相机分辨率是否与旧工程一致。"));
        m_calibrationGuideClipWarningEmitted = true;
    }
}

void UiController::showCalibrationCaptureUpdate(
    const i2r_rlc::control::CalibrationCaptureUpdate& update)
{
    m_lastCalibrationCaptureUpdate = update;
    const auto& state = update.state;
    const QString phaseText = calibrationCapturePhaseToText(update.phase);
    const QString text =
        update.phase == i2r_rlc::control::CalibrationCapturePhase::Idle
            ? QStringLiteral("标定：空闲")
            : QStringLiteral("标定：%1\nboard=%2/%3, laser=%4/%5")
                  .arg(phaseText)
                  .arg(state.boardImageCount)
                  .arg(update.requiredBoardImageCount)
                  .arg(state.laserImageCount)
                  .arg(update.requiredLaserImageCount);
    m_view.setCalibrationStateText(text);

    const QString message = QStringLiteral("标定采集：%1；%2")
                                .arg(phaseText, QString::fromStdString(update.message));
    if (update.succeeded) {
        appendInfo(message);
    } else {
        appendWarning(message);
        if (update.phase == i2r_rlc::control::CalibrationCapturePhase::Error) {
            showLegacyWarningPrompt(
                QStringLiteral("提示!"),
                QString::fromStdString(update.message));
        }
    }
    if (update.robotCommand.valid) {
        appendInfo(QStringLiteral("标定采集发送 C 命令：s_Calibreset=%1, count=%2")
                       .arg(update.robotCommand.calibResetSignal)
                       .arg(update.robotCommand.capturedSampleCount));
        m_runtimeCoordinator.sendCalibrationCommand(update.robotCommand);
    }
    applyCalibrationCaptureHardwareRequest(update);
    scheduleAutomaticCalibrationLaserCaptureIfNeeded(update);

    if (update.automaticGuideOverlayActive && !m_calibrationGuideInfoEmitted) {
        appendInfo(QStringLiteral("自动标定红框为标定板应进入区域。"));
        m_calibrationGuideInfoEmitted = true;
    }
    if (!update.automaticGuideOverlayActive) {
        m_calibrationGuideInfoEmitted = false;
        m_calibrationGuideClipWarningEmitted = false;
        m_lastCalibrationBcalib = 0;
    }
    if (m_hasLatestFrame && !m_trackingOverlayActive) {
        showFrameWithCalibrationGuide(m_latestFrame);
    }
}

void UiController::showOfflineCorrectionUpdate(
    const i2r_rlc::control::OfflineCorrectionUpdate& update)
{
    m_offlineCorrectionPhase = update.phase;
    m_lastOfflineCorrectionUpdate = update;
    m_hasOfflineCorrectionUpdate = true;
    renderOfflineCorrectionState();

    const QString message = QStringLiteral("离线纠偏：%1；%2")
                                .arg(offlineCorrectionPhaseToText(update.phase),
                                     QString::fromStdString(update.message));
    if (update.succeeded) {
        appendInfo(message);
    } else {
        appendWarning(message);
    }
    if (update.robotCommand.valid) {
        appendInfo(QStringLiteral("离线纠偏 O 命令已交给 RobotComm：s_offline=%1, PvarNum=%2")
                       .arg(update.robotCommand.offlineSignal)
                       .arg(update.robotCommand.pointCount));
    }
    if (!update.succeeded
        && update.phase == i2r_rlc::control::OfflineCorrectionPhase::Error) {
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            legacyOfflineFailurePrompt(QString::fromStdString(update.message)));
    }
    if (update.succeeded
        && update.phase == i2r_rlc::control::OfflineCorrectionPhase::Completed
        && !m_offlineCompletionPromptShown) {
        showLegacyInfoPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("离线纠偏已完成，请运行焊接程序"));
        m_offlineCompletionPromptShown = true;
    }
}

void UiController::renderOfflineCorrectionState()
{
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::Offline) {
        renderPendingCorrectionRobotStart();
        return;
    }
    if (!m_hasOfflineCorrectionUpdate) {
        return;
    }

    const auto& update = m_lastOfflineCorrectionUpdate;
    const auto& state = update.state;
    QString text = QStringLiteral("离线纠偏：%1\n%2")
                       .arg(offlineCorrectionPhaseToText(update.phase),
                            QString::fromStdString(update.message));
    if (state.report.valid) {
        text += QStringLiteral(
                    "\nICP点数：模型=%1，检测=%2，示教=%3"
                    "\nICP误差：mean=%4 mm，RMS=%5 mm"
                    "\nICP置信度：%6")
                    .arg(state.report.modelPointCount)
                    .arg(state.report.detectionPointCount)
                    .arg(state.report.teachPointCount)
                    .arg(state.report.meanErrorMm, 0, 'f', 3)
                    .arg(state.report.rmsErrorMm, 0, 'f', 3)
                    .arg(state.report.confidence, 0, 'f', 3);
    } else if (!state.detectionSamples.empty()) {
        text += QStringLiteral("\n已累计检测点：%1")
                    .arg(static_cast<int>(state.detectionSamples.size()));
    }
    if (!state.archiveFilePath.empty()) {
        text += QStringLiteral("\n存档：%1")
                    .arg(QFileInfo(QString::fromStdString(state.archiveFilePath)).fileName());
    }
    text += QStringLiteral(
                "\n机器人：sStart=%1，traflag=%2，posflag=%3"
                "\n流程：B040=%4，应答=%5，offline=%6，JBI=%7"
                "\n焊缝采集：%8")
                .arg(m_robotState.sStart)
                .arg(m_robotState.traflag)
                .arg(m_robotState.posflag)
                .arg(m_robotState.OfflineRun)
                .arg(m_offlineStartCount)
                .arg(m_robotState.offline)
                .arg(m_robotState.usJobLine)
                .arg(m_offlineSeamWindowClosed
                         ? QStringLiteral("已锁存关闭（侧面帧不参与ICP）")
                         : QStringLiteral("等待/进行中"));
    m_view.setOfflineCorrectionStateText(text);
}

void UiController::showOnlineCorrectionUpdate(const i2r_rlc::control::OnlineCorrectionUpdate& update)
{
    m_onlineCorrectionP001Presentation.update(update, m_positionerCalibration);
    m_view.setOnlineCorrectionP001Text(
        m_onlineCorrectionP001Presentation.text());
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::OnlineGlobalLocate
        || m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::OnlineCorrection) {
        renderPendingCorrectionRobotStart();
        return;
    }
    const QString message = QString::fromStdString(update.message);
    m_view.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：%1\n%2")
                                    .arg(onlineCorrectionPhaseToText(update.phase),
                                         message));
    if (update.succeeded) {
        appendInfo(message);
        if (m_onlineAutoCorrectionWorkflow.state()
                == OnlineCorrectionWorkflowState::OnlineCorrectionRunning
            && update.phase == i2r_rlc::control::OnlineCorrectionPhase::Completed
            && m_onlineAutoCorrectionWorkflow.markOnlineCorrectionCompleted()) {
            appendInfo(QStringLiteral("在线纠偏一键全自动流程完成。"));
        }
    } else {
        appendWarning(message);
        if (update.phase == i2r_rlc::control::OnlineCorrectionPhase::Error) {
            if (m_onlineAutoCorrectionWorkflow.state()
                == OnlineCorrectionWorkflowState::OnlineCorrectionRunning) {
                m_onlineAutoCorrectionWorkflow.fail();
            }
            showLegacyWarningPrompt(QStringLiteral("提示!"), message);
        }
    }
}

void UiController::showTeachTrajectoryRecorderUpdate(
    const i2r_rlc::control::TeachTrajectoryRecorderSnapshot& snapshot)
{
    const QString shortText = QStringLiteral("%1：W=%2，T=%3")
                                  .arg(snapshot.recordingEnabled
                                           ? QStringLiteral("记录中")
                                           : QStringLiteral("未开始"))
                                  .arg(snapshot.trajectoryPointCount)
                                  .arg(snapshot.teachPointCount);
    m_view.setTeachTrajectoryStateText(shortText);

    const QString message = QStringLiteral("保存示教轨迹：%1")
                                .arg(QString::fromStdString(snapshot.message));
    if (snapshot.succeeded) {
        appendInfo(message);
    } else {
        appendWarning(message);
    }
    if (m_teachTrajectorySavePromptPending) {
        if (snapshot.succeeded
            && (!snapshot.outputWorkPathFile.empty()
                || !snapshot.outputTeachPoseFile.empty())) {
            showLegacyInfoPrompt(
                QStringLiteral("提示!"),
                QStringLiteral("保存示教轨迹成功\n%1\n%2")
                    .arg(QString::fromStdString(snapshot.outputWorkPathFile),
                         QString::fromStdString(snapshot.outputTeachPoseFile)));
            m_teachTrajectorySavePromptPending = false;
        } else if (!snapshot.succeeded) {
            showLegacyWarningPrompt(
                QStringLiteral("提示!"),
                legacyTeachTrajectorySaveFailurePrompt(
                    QString::fromStdString(snapshot.message)));
            m_teachTrajectorySavePromptPending = false;
        }
    }
}

void UiController::showRuntimeTrackingResult(const i2r_rlc::ui::RuntimeTrackingResult& result)
{
    const bool standaloneGlobalLocateRunning =
        m_standaloneGlobalLocateState
        == StandaloneGlobalLocateWorkflowState::JobRunning
        || m_standaloneGlobalLocateState
        == StandaloneGlobalLocateWorkflowState::DrainingVision;
    CorrectionSyncPresentationState* correctionState = nullptr;
    if (result.correlation.purpose == CorrectionPurpose::OnlineCorrection) {
        correctionState = &m_onlineCorrectionSyncState;
    } else if (result.correlation.purpose == CorrectionPurpose::OfflineCorrection) {
        correctionState = &m_offlineCorrectionSyncState;
    }
    if (correctionState != nullptr) {
        if (!correctionState->acceptingFrames
            || result.correlation.sessionId != correctionState->sessionId
            || result.correlation.frameId != result.frameId
            || result.frameId != correctionState->lastConsumedFrameId
            || !presentCorrectionFrame(result)) {
            return;
        }
        ++correctionState->trackingResultCount;
    }
    const bool shouldRefreshCorrectionText = correctionState == nullptr
        || correctionState->trackingResultCount == 1
        || (correctionState->trackingResultCount % 5) == 0;
    const bool shouldLogCorrectionResult = correctionState == nullptr
        || correctionState->trackingResultCount == 1
        || (correctionState->trackingResultCount % 15) == 0;

    if (standaloneGlobalLocateRunning) {
        QVector<QPointF> realLaserOverlayPoints;
        if (result.triggerAccepted && result.valid) {
            realLaserOverlayPoints.reserve(static_cast<int>(
                result.realLaserLineImagePoints_px.size()));
            for (const auto& point : result.realLaserLineImagePoints_px) {
                realLaserOverlayPoints.push_back(
                    QPointF(point.x_px, point.y_px));
            }
        }
        m_view.setRealLaserOverlay(realLaserOverlayPoints);

        ++m_standaloneGlobalLocateTrackingFrameCount;
        const QString previousMessage = m_standaloneGlobalLocateLastTrackingMessage;
        m_standaloneGlobalLocateLastTrackingMessage = result.message;
        const int detectionWindowState = result.triggerAccepted ? 1 : 0;
        if (detectionWindowState
            != m_standaloneGlobalLocateLastDetectionWindowState) {
            m_standaloneGlobalLocateLastDetectionWindowState = detectionWindowState;
            appendGlobalLocateInfo(
                detectionWindowState == 1
                    ? QStringLiteral("全局定位检测窗口开始：traflag=1（STARTDETPRG），定位面=%1，JBI行=%2，开始从相机图像提取真实激光线。")
                          .arg(result.localizationSegmentIndex)
                          .arg(result.robotJobLine)
                    : QStringLiteral("全局定位检测窗口关闭：traflag=0（ENDDETPRG 或尚未 STARTDETPRG），定位面=%1，JBI行=%2，不采图。")
                          .arg(result.localizationSegmentIndex)
                          .arg(result.robotJobLine));
        }
        if (!result.triggerAccepted) {
            ++m_standaloneGlobalLocateTriggerRejectedFrameCount;
            const bool normalDetectionWindowClosed =
                result.message.startsWith(QStringLiteral("traflag_not_ready"));
            if (!normalDetectionWindowClosed && result.message != previousMessage) {
                appendGlobalLocateWarning(
                    QStringLiteral("全局定位图像帧未触发：frame=%1，reason=%2")
                        .arg(result.frameId)
                        .arg(result.message));
            }
        } else if (!result.valid) {
            ++m_standaloneGlobalLocateInvalidFrameCount;
            if (result.message != previousMessage) {
                appendGlobalLocateWarning(
                    QStringLiteral("全局定位图像处理无效：frame=%1，stage=%2")
                        .arg(result.frameId)
                        .arg(result.message));
            }
        } else {
            ++m_standaloneGlobalLocateValidFrameCount;
        }
        collectStandaloneGlobalLocateScanPoint(result);
    }

    if (!result.triggerAccepted) {
        if (!standaloneGlobalLocateRunning && shouldLogCorrectionResult) {
            appendInfo(QStringLiteral("视觉跟踪未触发：frame=%1, %2")
                           .arg(result.frameId)
                           .arg(result.message));
        }
        return;
    }

    const QString status = result.valid ? QStringLiteral("有效") : QStringLiteral("无效");
    QString text = QStringLiteral("视觉跟踪：%1\nframe=%2, image=(%3,%4)\n%5")
                       .arg(status)
                       .arg(result.frameId)
                       .arg(result.imageXPixel, 0, 'f', 2)
                       .arg(result.imageYPixel, 0, 'f', 2)
                       .arg(result.message);
    if (result.sample.point.valid) {
        text += QStringLiteral("\nspace=(%1,%2,%3) mm")
                    .arg(result.sample.point.xMm, 0, 'f', 3)
                    .arg(result.sample.point.yMm, 0, 'f', 3)
                    .arg(result.sample.point.zMm, 0, 'f', 3);
    }
    collectOnlineAutoCorrectionScanPoint(result);
    if (!standaloneGlobalLocateRunning) {
        collectStandaloneGlobalLocateScanPoint(result);
    }

    // 全局定位期间相机帧仍由 frameReady 连续显示。这里只按固定节拍更新文字，
    // 并禁止逐帧追加日志/重复缩放图像，避免数千条事件拖慢 UI event loop。
    if (standaloneGlobalLocateRunning) {
        m_trackingOverlayActive = false;
        if (m_standaloneGlobalLocateTrackingFrameCount == 1
            || (m_standaloneGlobalLocateTrackingFrameCount % 15) == 0) {
            m_view.setVisionStateText(text);
        }
        return;
    }

    if (shouldRefreshCorrectionText) {
        m_view.setVisionStateText(text);
    }

    if (correctionState != nullptr) {
        // 已通过完整快照显示。相机新帧仍被接收，但不能覆盖带检测点的旧帧。
        m_trackingOverlayActive = true;
    } else if (!result.overlayImage.isNull()) {
        i2r_rlc::camera::CameraFrame overlayFrame;
        overlayFrame.cameraId = m_latestFrame.cameraId;
        overlayFrame.frameId = result.frameId;
        overlayFrame.frameNumber = result.frameId;
        overlayFrame.timestamp = m_latestFrame.timestamp;
        overlayFrame.sourcePath = m_latestFrame.sourcePath;
        overlayFrame.image = result.overlayImage;
        overlayFrame.previewImage = result.overlayImage;
        overlayFrame.description = QStringLiteral("KCF 检测结果：绿色十字为焊缝点");
        m_view.showFrame(overlayFrame);
        m_trackingOverlayActive = true;
    } else {
        m_trackingOverlayActive = false;
    }

    if (result.valid && shouldLogCorrectionResult) {
        appendInfo(QStringLiteral("视觉跟踪输出：frame=%1, image=(%2,%3)")
                       .arg(result.frameId)
                       .arg(result.imageXPixel, 0, 'f', 2)
                       .arg(result.imageYPixel, 0, 'f', 2));
    } else if (!result.valid && shouldLogCorrectionResult) {
        appendWarning(QStringLiteral("视觉跟踪无效：frame=%1, %2")
                          .arg(result.frameId)
                          .arg(result.message));
    }
}

bool UiController::presentCorrectionFrame(const RuntimeTrackingResult& result)
{
    if (!result.frameContext.has_value()) {
        return false;
    }
    const auto &context = result.frameContext.value();
    const auto &observation = context.observation;
    const auto &frame = context.sourceFrame;
    if (frame.frameId != result.frameId
        || !isUsableCorrectionObservationForFrame(result.frameId, observation)) {
        return false;
    }

    TrackingFramePresentation presentation;
    presentation.sourceImage = frame.image.isNull() ? frame.previewImage : frame.image;
    if (presentation.sourceImage.isNull()) {
        return false;
    }
    presentation.virtualLaserPoints.reserve(
        static_cast<int>(observation.laserLineImagePoints_px.size()));
    for (const auto &point : observation.laserLineImagePoints_px) {
        presentation.virtualLaserPoints.push_back(QPointF(point.x_px, point.y_px));
    }
    presentation.virtualWeldPoint = QPointF(
        observation.weldPointImage_px.x_px, observation.weldPointImage_px.y_px);
    presentation.virtualWeldPointVisible = observation.hasWeldPointImage_px;
    presentation.detectedWeldPoint = QPointF(result.imageXPixel, result.imageYPixel);
    presentation.detectedWeldPointVisible = result.triggerAccepted && result.valid;
    presentation.caption = QStringLiteral("检测帧 %1｜原图与参考、检测点同帧｜%2")
        .arg(result.frameId)
        .arg(presentation.detectedWeldPointVisible
            ? QStringLiteral("绿色：检测点") : QStringLiteral("本帧检测无效"));
    m_view.showTrackingFrame(presentation);
    m_trackingOverlayActive = true;

    const QString stateText = QStringLiteral(
        "仿真：与检测图同帧\n端口：%1\n"
        "显示帧：%2\n激光线：%3 图像点 / %4 三维点\n"
        "焊缝点二维：(%5, %6) pixel")
        .arg(m_globalLocateRobotSimEndpoint.port)
        .arg(result.frameId)
        .arg(static_cast<int>(observation.laserLineImagePoints_px.size()))
        .arg(static_cast<int>(observation.laserLinePoints_mm.size()))
        .arg(observation.weldPointImage_px.x_px, 0, 'f', 0)
        .arg(observation.weldPointImage_px.y_px, 0, 'f', 0);
    m_view.setDigitalTwinStateText(stateText);
    return true;
}

void UiController::clearCorrectionFrameDisplay()
{
    m_trackingOverlayActive = false;
    m_view.setVirtualReferenceOverlay({}, {}, false);
    m_view.setDetectedWeldPointOverlay({}, false);
    if (m_hasLatestFrame) {
        m_view.showFrame(m_latestFrame);
    }
}

void UiController::showRobotState(const RobotStateModel& state)
{
    Q_UNUSED(state);
    m_view.setLegacyStatusPanel(
        robotModeToLegacyText(m_robotState.sMode),
        servoToLegacyText(m_robotState.sServoPower),
        runToLegacyText(m_robotState.sStart),
        correctionToLegacyText(m_robotState.corflag),
        m_laserEnabled,
        m_filterEnabled,
        robotDeviceText(m_robotDeviceConnected),
        QStringLiteral("--"),
        QString::number(m_cameraExposureUs));
}

void UiController::showIbvsStepResult(
    const i2r_rlc::ibvs::IbvsOnlineStepResult& result)
{
    const QString statusText = QStringLiteral(
        "视觉伺服：%1\n帧：%2\n速度：[%3, %4, %5] m/s\n误差范数：%6 px\n行数：%7，限幅：%8\nLVS先验/约束：%9/%10\n命令：%11")
        .arg(ibvsServiceStateToText(result.state))
        .arg(result.frameIndex)
        .arg(result.velocityMps(0), 0, 'f', 6)
        .arg(result.velocityMps(1), 0, 'f', 6)
        .arg(result.velocityMps(2), 0, 'f', 6)
        .arg(result.featureErrorPx.norm(), 0, 'f', 3)
        .arg(result.rowCount)
        .arg(result.velocityLimited ? QStringLiteral("是") : QStringLiteral("否"))
        .arg(result.usedLvsPrior ? QStringLiteral("是") : QStringLiteral("否"))
        .arg(result.usedLvsConstraint ? QStringLiteral("是") : QStringLiteral("否"))
        .arg(QString::fromStdString(result.dryRunCommand));
    m_view.setIbvsStateText(statusText);

    const QString logText = QStringLiteral(
        "视觉伺服 frame=%1, ok=%2, velocity=[%3,%4,%5] m/s, limited=%6, rows=%7, command=%8, message=%9")
        .arg(result.frameIndex)
        .arg(result.error.isOk() ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(result.velocityMps(0), 0, 'f', 9)
        .arg(result.velocityMps(1), 0, 'f', 9)
        .arg(result.velocityMps(2), 0, 'f', 9)
        .arg(result.velocityLimited ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(result.rowCount)
        .arg(QString::fromStdString(result.dryRunCommand))
        .arg(QString::fromStdString(result.message));
    if (result.error.isOk()) {
        appendInfo(logText);
    } else {
        appendWarning(QStringLiteral("%1, error=%2")
                          .arg(logText, QString::fromStdString(result.error.message)));
    }
}

void UiController::showLocalizationUpdate(
    const i2r_rlc::localization::LocalizationWorkflowUpdate& update)
{
    QString statusText = QStringLiteral("定位：%1\n模式：%2\n自由度：%3\n段数：%4")
                             .arg(i2r_rlc::localization::localizationWorkflowStateToText(update.state))
                             .arg(localizationModeToText(update.mode))
                             .arg(dofMaskToText(update.dofMask))
                             .arg(update.segmentCount);
    if (update.hasResidual) {
        statusText += (update.mode == i2r_rlc::localization::LocalizationMode::Global
            ? QStringLiteral("\nRMSE: %1 -> %2 mm")
            : QStringLiteral("\n残差: %1 -> %2 mm"))
            .arg(update.beforeResidual_mm, 0, 'f', 6)
            .arg(update.afterResidual_mm, 0, 'f', 6);
    }
    statusText += QStringLiteral("\n%1").arg(
        update.succeeded || update.errorMessage.isEmpty()
            ? update.message
            : update.errorMessage);
    m_view.setLocalizationStateText(statusText);

    const QString logText = QStringLiteral("定位流程：state=%1, mode=%2, dof=%3, segments=%4, message=%5")
                                .arg(i2r_rlc::localization::localizationWorkflowStateToKey(update.state))
                                .arg(localizationModeToText(update.mode))
                                .arg(dofMaskToText(update.dofMask))
                                .arg(update.segmentCount)
                                .arg(update.succeeded || update.errorMessage.isEmpty()
                                         ? update.message
                                         : update.errorMessage);
    if (update.succeeded) {
        appendInfo(logText);
    } else {
        appendWarning(logText);
    }
}

void UiController::loadWorkpieceProfiles()
{
    const WorkpieceProfileLoadResult result =
        m_configRepository.loadWorkpieceProfiles();
    if (!result.ok) {
        m_workpieceProfiles.clear();
        m_selectedWorkpieceIndex = -1;
        m_view.setWorkpieceChoices({}, {});
        m_view.setCurrentWorkpieceText(QStringLiteral("工件配置加载失败"));
        appendWarning(result.message);
        return;
    }

    m_workpieceProfiles = result.profiles;
    QStringList displayNames;
    QStringList profileIds;
    displayNames.reserve(m_workpieceProfiles.size());
    profileIds.reserve(m_workpieceProfiles.size());
    for (const WorkpieceProfile& profile : m_workpieceProfiles) {
        displayNames.push_back(profile.displayName);
        profileIds.push_back(profile.id);
    }
    m_view.setWorkpieceChoices(displayNames, profileIds);
    m_view.setCurrentWorkpieceText(QStringLiteral("请选择工件"));
    appendInfo(QStringLiteral("%1：%2")
                   .arg(result.message, result.filePath));
}

const WorkpieceProfile* UiController::selectedWorkpieceProfile() const
{
    if (m_selectedWorkpieceIndex < 0
        || m_selectedWorkpieceIndex >= m_workpieceProfiles.size()) {
        return nullptr;
    }
    return &m_workpieceProfiles.at(m_selectedWorkpieceIndex);
}

bool UiController::validateSelectedOnlineTrajectory(
    QString* workPathFile,
    QString* teachPoseFile)
{
    const WorkpieceProfile* profile = selectedWorkpieceProfile();
    if (profile == nullptr) {
        appendWarning(QStringLiteral("尚未选择工件，请先在右侧“当前工件”中点击“选择工件”。"));
        return false;
    }

    const QString workPath = m_configRepository.workpieceOnlineWorkPathFile(*profile);
    const QString teachPath = m_configRepository.workpieceOnlineTeachPoseFile(*profile);
    const QFileInfo workPathInfo(workPath);
    const QFileInfo teachPoseInfo(teachPath);
    if (!workPathInfo.isFile() || !teachPoseInfo.isFile()) {
        appendWarning(QStringLiteral(
                          "当前工件“%1”的在线纠偏轨迹文件缺失，不能启动在线纠偏：\n_W=%2\n_T=%3\n请将对应文件放入 %4。")
                          .arg(
                              profile->displayName,
                              workPath,
                              teachPath,
                              m_configRepository.paths().workpieceTrajectoryDir().absolutePath()));
        return false;
    }

    if (workPathFile) {
        *workPathFile = workPathInfo.absoluteFilePath();
    }
    if (teachPoseFile) {
        *teachPoseFile = teachPoseInfo.absoluteFilePath();
    }
    return true;
}

bool UiController::validateSelectedOfflineCorrectionTrajectory(
    QString* workPathFile,
    QString* teachPoseFile)
{
    const WorkpieceProfile* profile = selectedWorkpieceProfile();
    if (profile == nullptr) {
        appendWarning(QStringLiteral("尚未选择工件，请先在右侧“当前工件”中点击“选择工件”。"));
        return false;
    }

    const QString workPath = m_configRepository.workpieceOfflineCorrectionWorkPathFile(*profile);
    const QString teachPath =
        m_configRepository.workpieceOfflineCorrectionTeachPoseFile(*profile);
    const QFileInfo workPathInfo(workPath);
    const QFileInfo teachPoseInfo(teachPath);
    if (!workPathInfo.isFile() || !teachPoseInfo.isFile()) {
        appendWarning(QStringLiteral(
                          "当前工件“%1”的离线纠偏轨迹文件缺失，不能启动离线纠偏：\n_W=%2\n_T=%3\n请将对应文件放入 %4。")
                          .arg(
                              profile->displayName,
                              workPath,
                              teachPath,
                              m_configRepository.paths().workpieceTrajectoryDir().absolutePath()));
        return false;
    }

    if (workPathFile) {
        *workPathFile = workPathInfo.absoluteFilePath();
    }
    if (teachPoseFile) {
        *teachPoseFile = teachPoseInfo.absoluteFilePath();
    }
    return true;
}

void UiController::configureSelectedWorkpieceCorrection()
{
    configureSelectedWorkpieceOnlineCorrection();
    configureSelectedWorkpieceOfflineCorrection();
    if (const WorkpieceProfile* profile = selectedWorkpieceProfile()) {
        appendInfo(QStringLiteral("当前工件已分别自动提交在线/离线纠偏配置：%1")
                       .arg(profile->displayName));
    }
}

void UiController::configureSelectedWorkpieceOnlineCorrection()
{
    QString workPathFile;
    QString teachPoseFile;
    if (!validateSelectedOnlineTrajectory(&workPathFile, &teachPoseFile)) {
        m_onlineCorrectionConfigured = false;
        return;
    }

    onOnlineCorrectionConfigurationSelected(workPathFile, teachPoseFile);
}

void UiController::configureSelectedWorkpieceOfflineCorrection()
{
    QString workPathFile;
    QString teachPoseFile;
    if (!validateSelectedOfflineCorrectionTrajectory(&workPathFile, &teachPoseFile)) {
        m_offlineCorrectionConfigured = false;
        return;
    }
    Q_UNUSED(teachPoseFile);

    onOfflineCorrectionArchiveSelected(workPathFile);
    m_offlineCorrectionConfigured = true;
}

void UiController::beginCorrectionRobotStartGate(
    CorrectionRobotStartKind kind,
    const QString& jobName)
{
    const QString trimmedJobName = jobName.trimmed();
    if (trimmedJobName.isEmpty()) {
        appendWarning(QStringLiteral("纠偏启动失败：机器人 JBI 名称为空。"));
        showLegacyWarningPrompt(
            QStringLiteral("提示!"),
            QStringLiteral("未选择示教文件或错误示教文件!"));
        return;
    }

    if (m_pendingCorrectionRobotStartKind != CorrectionRobotStartKind::None) {
        showLegacyWarningPrompt(QStringLiteral("已有启动请求"),
            QStringLiteral("请先取消当前等待启动的流程，或点击停止，再启动其他流程。"));
        return;
    }

    m_pendingCorrectionRobotStartKind = kind;
    m_pendingCorrectionRobotJobName = trimmedJobName;
    m_correctionRobotStartWaitText.clear();
    evaluatePendingCorrectionRobotStart();
    // 仅响应本次启动点击弹窗；状态回包只刷新面板，避免反复打断操作。
    if (m_pendingCorrectionRobotStartKind != CorrectionRobotStartKind::None
        && !m_correctionRobotStartWaitText.isEmpty()) {
        showLegacyWarningPrompt(QStringLiteral("启动条件未满足"),
            QStringLiteral("机器人程序尚未启动，请完成以下操作：\n\n%1\n\n"
                           "条件满足后将自动启动。若不继续，请点击“取消等待启动”或“停止”。")
                .arg(m_correctionRobotStartWaitText));
    }
}

void UiController::cancelPendingCorrectionRobotStart()
{
    m_view.setCorrectionStartWaiting(true, false);
    m_view.setCorrectionStartWaiting(false, false);
    m_pendingCorrectionRobotStartKind = CorrectionRobotStartKind::None;
    m_pendingCorrectionRobotJobName.clear();
    m_correctionRobotStartWaitText.clear();
}

void UiController::handleCorrectionSyncUpdate(
    const CorrectionSyncUpdate& update)
{
    CorrectionSyncPresentationState* state = nullptr;
    if (update.purpose == CorrectionPurpose::OnlineCorrection) {
        state = &m_onlineCorrectionSyncState;
    } else if (update.purpose == CorrectionPurpose::OfflineCorrection) {
        state = &m_offlineCorrectionSyncState;
    } else {
        // None/全局定位/普通实时/手动 RobotSim 都不具备纠偏三键，必须忽略。
        return;
    }

    const bool mayEstablishSession = update.state == CorrectionSyncState::Idle
        || update.state == CorrectionSyncState::Querying
        || update.state == CorrectionSyncState::ObservationReady;
    const bool replacementSession = mayEstablishSession
        && update.sessionId != 0
        && update.sessionId != state->sessionId;
    if (replacementSession) {
        state->sessionId = update.sessionId;
        state->purpose = update.purpose;
        state->pendingFrameId = -1;
        state->lastConsumedFrameId = -1;
        state->trackingResultCount = 0;
        state->acceptingFrames = true;
        state->lastError.clear();
    }
    if (update.sessionId == 0 || update.sessionId != state->sessionId) {
        // 迟到的 consumed/failure/clear 不能建立或替换会话，更不能清除新会话 pending。
        return;
    }

    switch (update.state) {
    case CorrectionSyncState::Idle:
        state->acceptingFrames = true;
        clearCorrectionFrameDisplay();
        if (update.purpose != CorrectionPurpose::OfflineCorrection) {
            m_trackingOverlayActive = true;
            m_view.setImagePlaceholderText(QStringLiteral("等待首个完整检测帧"));
        }
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：等待纠偏检测窗口\n端口：%1\n尚无本轮同帧参考")
                .arg(m_globalLocateRobotSimEndpoint.port));
        break;
    case CorrectionSyncState::Querying:
        break;
    case CorrectionSyncState::ObservationReady: {
        if (update.sessionId != 0
            && update.sessionId == state->sessionId
            && update.purpose == state->purpose
            && update.frameId >= 0
            && update.hasObservation
            && isUsableCorrectionObservationForFrame(
                update.frameId, update.observation)) {
            state->pendingFrameId = update.frameId;
        } else {
            return;
        }

        // 只更新同步进度。仿真参考必须等对应检测结果到达后，与原图一起显示。
        break;
    }
    case CorrectionSyncState::VisionConsumed:
        if (update.sessionId == state->sessionId
            && update.purpose == state->purpose
            && update.frameId == state->pendingFrameId) {
            state->lastConsumedFrameId = update.frameId;
            state->pendingFrameId = -1;
            state->lastError.clear();
        } else {
            return;
        }
        break;
    case CorrectionSyncState::Failed:
        state->pendingFrameId = -1;
        state->lastError = update.message;
        clearCorrectionFrameDisplay();
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：纠偏帧 %1 同步失败\n本帧参考不可用\n%2")
                .arg(update.frameId)
                .arg(update.message));
        break;
    case CorrectionSyncState::Cleared:
        state->pendingFrameId = -1;
        state->acceptingFrames = false;
        clearCorrectionFrameDisplay();
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：纠偏同帧参考已清除\n%1").arg(update.message));
        break;
    case CorrectionSyncState::StoppedAfterFailures:
        state->pendingFrameId = -1;
        state->lastError = update.message;
        state->acceptingFrames = false;
        clearCorrectionFrameDisplay();
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：连续同步失败，纠偏已停止\n%1")
                .arg(update.message));
        break;
    case CorrectionSyncState::OfflineSeamWindowClosed:
        state->pendingFrameId = -1;
        m_offlineSeamWindowClosed = true;
        state->acceptingFrames = false;
        clearCorrectionFrameDisplay();
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：焊缝采集窗口已关闭\n已停止纠偏逐帧查询"));
        renderOfflineCorrectionState();
        break;
    case CorrectionSyncState::TrackingPaused:
        state->pendingFrameId = -1;
        state->acceptingFrames = false;
        clearCorrectionFrameDisplay();
        m_view.setDigitalTwinStateText(
            QStringLiteral("仿真：纠偏检测窗口已暂停\n%1")
                .arg(update.message));
        break;
    }
    state->stateText = correctionSyncStateToText(update.state);
    renderCorrectionSyncState(*state);
}

void UiController::renderCorrectionSyncState(
    const CorrectionSyncPresentationState& state)
{
    const QString stateText = QStringLiteral("逐帧同步：%1")
        .arg(state.stateText);
    const QString frameText = state.lastConsumedFrameId >= 0
        ? QStringLiteral("最近确认帧：%1").arg(state.lastConsumedFrameId)
        : QStringLiteral("最近确认帧：无");
    const QString errorText = state.lastError.isEmpty()
        ? QStringLiteral("最近同步错误：无")
        : QStringLiteral("最近同步错误：%1").arg(state.lastError);
    if (state.purpose == CorrectionPurpose::OnlineCorrection) {
        m_view.setOnlineCorrectionSyncDetails(
            stateText, frameText, errorText);
    } else if (state.purpose == CorrectionPurpose::OfflineCorrection) {
        m_view.setOfflineCorrectionSyncDetails(
            stateText, frameText, errorText);
    }
}

void UiController::evaluatePendingCorrectionRobotStart()
{
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::None) {
        return;
    }

    const auto decision = CCorrectionRobotStartGate::evaluate(
        m_robotDeviceConnected,
        m_robotState);
    const bool formalCorrection = m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::Offline
        || m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::OnlineCorrection;
    CorrectionRobotSimStartState simState;
    simState.realSceneConnected = m_robotSimConnected;
    simState.realSceneConnectionPending = m_robotSimConnectionPending;
    simState.manualModeRequested = m_robotSimManualModeRequested;
    simState.manualConnectionPending = m_robotSimManualConnectionPending;
    simState.manualConnected = m_robotSimManualConnected;
    const auto simDecision = CCorrectionRobotStartGate::evaluateRobotSim(formalCorrection, simState);
    if (decision.ready && simDecision.ready) {
        const CorrectionRobotStartKind kind = m_pendingCorrectionRobotStartKind;
        const QString jobName = m_pendingCorrectionRobotJobName;
        cancelPendingCorrectionRobotStart();
        executePendingCorrectionRobotStart(kind, jobName);
        return;
    }

    QStringList reasons;
    if (!m_robotDeviceConnected) {
        reasons << QStringLiteral("等待机器人连接及有效状态回包");
    } else {
        if (m_robotState.sStart != 0) {
            reasons << QStringLiteral("请先停止机器人当前程序");
        }
        if (m_robotState.sMode != 2) {
            reasons << QStringLiteral("请将示教器模式开关拨到 PLAY（再现）模式");
        }
        if (m_robotState.sServoPower != 1) {
            reasons << QStringLiteral("请在示教器打开伺服（当前 SERVO OFF）");
        }
    }
    if (!simDecision.ready) {
        reasons << QStringLiteral("请连接 RobotSim 真实场景，退出手动仿真模式");
    }
    const QString waitText = reasons.join(QStringLiteral("\n"));
    if (waitText != m_correctionRobotStartWaitText) {
        m_correctionRobotStartWaitText = waitText;
        appendWarning(QStringLiteral("等待启动 %1：%2")
            .arg(m_pendingCorrectionRobotJobName, waitText));
    }
    renderPendingCorrectionRobotStart();
}

void UiController::renderPendingCorrectionRobotStart()
{
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::None) {
        return;
    }
    const QString text = QStringLiteral("等待启动：%1\n%2\n条件满足后自动启动；可取消等待或点击停止。")
        .arg(m_pendingCorrectionRobotJobName, m_correctionRobotStartWaitText);
    if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::Offline) {
        m_view.setCorrectionStartWaiting(true, true);
        m_view.setOfflineCorrectionStateText(text);
    } else if (m_pendingCorrectionRobotStartKind == CorrectionRobotStartKind::GlobalLocateStandalone) {
        m_view.setLocalizationStateText(text);
    } else {
        m_view.setCorrectionStartWaiting(false, true);
        m_view.setOnlineCorrectionStateText(text);
    }
}

void UiController::executePendingCorrectionRobotStart(
    CorrectionRobotStartKind kind,
    const QString& jobName)
{
    switch (kind) {
    case CorrectionRobotStartKind::OnlineGlobalLocate: {
        if (!m_onlineAutoCorrectionWorkflow.markGlobalLocateStarted()) {
            failOnlineAutoCorrectionWorkflow(
                QStringLiteral("在线纠偏内部状态错误：不能启动全局定位 JBI。"));
            break;
        }
        appendInfo(QStringLiteral("机器人状态满足在线全自动第一段启动条件，发送全局定位 JBI：%1")
                       .arg(jobName));
        m_onlineAutoGlobalLocalizationScanPoints.clear();
        m_onlineAutoGlobalLocalizationVirtualPoints.clear();
        m_onlineAutoGlobalLocateJobMonitor.arm();
        GlobalLocalizationTrackingSources sources;
        sources.laserSource = GlobalLocateVirtualLaserSource::RobotSim;
        sources.pointCloudSource =
            GlobalLocateVirtualPointCloudSource::RobotSimLaserScan;
        m_runtimeCoordinator.startGlobalLocalizationTracking(sources);
        m_runtimeCoordinator.startRobotJob(jobName.toStdString());
        m_view.setOnlineCorrectionStateText(
            QStringLiteral("在线纠偏：全局定位程序运行中\n全局定位JBI：%1\n已采集点：0")
                .arg(jobName));
        break;
    }
    case CorrectionRobotStartKind::GlobalLocateStandalone:
        appendGlobalLocateInfo(
            QStringLiteral("全局定位步骤 3/7：机器人门禁通过，发送 J 命令启动扫描程序：%1")
                .arg(jobName));
        m_trackingOverlayActive = false;
        m_runtimeCoordinator.startRobotJob(jobName.toStdString());
        m_standaloneGlobalLocateState =
            StandaloneGlobalLocateWorkflowState::JobRunning;
        m_standaloneGlobalLocateSegments.clear();
        m_standaloneGlobalLocateRobotSimVirtualSegments.clear();
        m_standaloneGlobalLocateCurrentSegment.clear();
        m_standaloneGlobalLocateActiveSegmentIndex = 0;
        m_standaloneGlobalLocateJobMonitor.arm();
        m_runtimeCoordinator.startGlobalLocalizationTracking();
        if (m_globalLocateVirtualLaserSource
            == GlobalLocateVirtualLaserSource::Occt) {
            if (m_globalLocatePriorTimer == nullptr) {
                startGlobalLocatePriorTimer();
            } else {
                publishGlobalLocateVirtualPrior();
            }
        }
        m_view.setLocalizationStateText(
            QStringLiteral("定位：全局定位程序运行中\n全局定位JBI：%1\n已采集点：0")
                .arg(jobName));
        break;
    case CorrectionRobotStartKind::OnlineCorrection: {
        const QString stageText =
            m_onlineAutoCorrectionWorkflow.plan().requiresGlobalLocalization
            ? QStringLiteral("全局定位后的在线阶段")
            : QStringLiteral("直接在线阶段");
        appendInfo(QStringLiteral("机器人状态满足%1启动条件，进入在线启动事务：%2")
                       .arg(stageText, jobName));
        m_trackingOverlayActive = false;
        m_runtimeCoordinator.startOnlineCorrectionJob(
            m_onlineAutoCorrectionRunConfiguration,
            jobName.toStdString());
        break;
    }
    case CorrectionRobotStartKind::Offline:
        appendInfo(QStringLiteral("机器人状态满足离线纠偏启动条件，开始 B040 预检并准备 Vision：%1")
                       .arg(jobName));
        m_offlineSeamWindowClosed = false;
        m_offlineStartCount = 0;
        m_trackingOverlayActive = false;
        m_view.setDetectedWeldPointOverlay({}, false);
        m_runtimeCoordinator.startOfflineCorrectionJob(jobName.toStdString());
        break;
    case CorrectionRobotStartKind::None:
        break;
    }
}

void UiController::cancelOnlineAutoCorrectionWorkflow()
{
    m_runtimeCoordinator.cancelOnlineCorrectionStart();
    if (m_pendingCorrectionRobotStartKind
            == CorrectionRobotStartKind::OnlineGlobalLocate
        || m_pendingCorrectionRobotStartKind
            == CorrectionRobotStartKind::OnlineCorrection) {
        cancelPendingCorrectionRobotStart();
    }
    m_runtimeCoordinator.stopGlobalLocalizationTracking();
    m_onlineAutoGlobalLocateJobMonitor.reset();
    m_onlineAutoCorrectionWorkflow.reset();
    m_onlineAutoGlobalLocalizationScanPoints.clear();
    m_onlineAutoGlobalLocalizationVirtualPoints.clear();
    m_onlineAutoCorrectionRunConfiguration = {};
}

void UiController::processOnlineAutoCorrectionRobotState()
{
    if (m_onlineAutoCorrectionWorkflow.state()
        != OnlineCorrectionWorkflowState::GlobalLocateJobRunning) {
        return;
    }

    if (!m_onlineAutoGlobalLocateJobMonitor.update(m_robotState)) {
        return;
    }

    if (!m_onlineAutoCorrectionWorkflow.markGlobalLocateJobCompleted()) {
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏内部状态错误：全局定位 JBI 完成事件无效。"));
        return;
    }
    appendInfo(QStringLiteral("在线纠偏：全局定位 JBI 已结束，正在排空最后一帧视觉结果。当前真实点：%1")
                   .arg(static_cast<int>(
                       m_onlineAutoGlobalLocalizationScanPoints.size())));
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：正在锁存全局定位末帧\n已采集点：%1")
            .arg(static_cast<int>(
                m_onlineAutoGlobalLocalizationScanPoints.size())));
    m_runtimeCoordinator.finishGlobalLocalizationTracking();
}

void UiController::completeOnlineAutoCorrectionGlobalLocateVisionDrain()
{
    if (!m_onlineAutoCorrectionWorkflow.markGlobalLocalizationDrained()) {
        return;
    }

    appendInfo(
        QStringLiteral("在线纠偏：在途视觉帧已排空，开始全局 ICP。真实点=%1，RobotSim 虚拟点=%2")
            .arg(static_cast<int>(
                m_onlineAutoGlobalLocalizationScanPoints.size()))
            .arg(static_cast<int>(
                m_onlineAutoGlobalLocalizationVirtualPoints.size())));
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：正在执行全局定位\n真实点：%1，虚拟点：%2")
            .arg(static_cast<int>(
                m_onlineAutoGlobalLocalizationScanPoints.size()))
            .arg(static_cast<int>(
                m_onlineAutoGlobalLocalizationVirtualPoints.size())));
    applyOnlineAutoCorrectionGlobalLocalization();
}

void UiController::applyOnlineAutoCorrectionGlobalLocalization()
{
    if (m_onlineAutoCorrectionWorkflow.state()
        != OnlineCorrectionWorkflowState::ApplyingGlobalLocalization) {
        return;
    }
    if (m_onlineAutoGlobalLocalizationScanPoints.empty()) {
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏全局定位失败：第一段运行未采集到有效三维点。"));
        return;
    }
    if (m_onlineAutoGlobalLocalizationVirtualPoints.empty()) {
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏全局定位失败：本轮未采集到 RobotSim 虚拟激光线三维点。"));
        return;
    }

    i2r_rlc::localization::LocalizationGlobalRuntimeRequest request;
    request.frameId = m_lastFrameId;
    request.realScanPoints_mm = m_onlineAutoGlobalLocalizationScanPoints;
    request.virtualReferenceSnapshot.available = true;
    request.virtualReferenceSnapshot.referenceId =
        m_globalLocateRobotSimScene.sceneName;
    request.virtualReferenceSnapshot.laserLinePoints_mm =
        m_onlineAutoGlobalLocalizationVirtualPoints;
    request.virtualReferenceSnapshot.message =
        QStringLiteral("在线全局定位期间逐帧锁存的 RobotSim 激光线点云。");
    const auto result = m_localizationService.runGlobalIcpRuntime(request);
    showLocalizationUpdate(result.update);

    if (!result.hasGlobalIcpResult
        || result.globalIcpResult.status
            != i2r_rlc::localization::LocalizationStatus::Succeeded) {
        const QString message =
            result.hasGlobalIcpResult
                ? result.globalIcpResult.message
                : QStringLiteral("未返回全局定位结果。");
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏全局定位失败：%1").arg(message));
        return;
    }

    m_onlineAutoCorrectionRunConfiguration.options.globalCorrectionRowMajor =
        result.globalIcpResult.combinedVirtualToRealCorrectionTransform.rowMajor;
    m_runtimeCoordinator.configureOnlineCorrection(
        m_onlineAutoCorrectionRunConfiguration);
    appendInfo(QStringLiteral("在线纠偏一键全自动：全局定位完成，已把修正矩阵写入在线纠偏配置。"));

    if (!m_onlineAutoCorrectionWorkflow.markGlobalLocalizationApplied()) {
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏内部状态错误：不能进入在线 JBI 阶段。"));
        return;
    }
    queueOnlineAutoCorrectionStart();
}

void UiController::failOnlineAutoCorrectionWorkflow(const QString& message)
{
    m_onlineAutoCorrectionWorkflow.fail();
    m_onlineAutoGlobalLocateJobMonitor.reset();
    m_runtimeCoordinator.stopGlobalLocalizationTracking();
    if (m_pendingCorrectionRobotStartKind
            == CorrectionRobotStartKind::OnlineGlobalLocate
        || m_pendingCorrectionRobotStartKind
            == CorrectionRobotStartKind::OnlineCorrection) {
        cancelPendingCorrectionRobotStart();
    }
    appendWarning(message);
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：启动失败\n%1").arg(message));
    showLegacyWarningPrompt(QStringLiteral("提示!"), message);
}

bool UiController::queueOnlineAutoCorrectionStart()
{
    if (!m_onlineCorrectionConfigured) {
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏启动失败：尚未配置 *_W.txt、*_T.txt 和 RBCALIB.DAT。"));
        return false;
    }
    const OnlineCorrectionStartPlan& plan =
        m_onlineAutoCorrectionWorkflow.plan();
    if (plan.onlineCorrectionJbi.isEmpty()) {
        failOnlineAutoCorrectionWorkflow(
            QStringLiteral("在线纠偏启动失败：当前模式的在线 JBI 为空。"));
        return false;
    }

    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：等待在线纠偏启动\n在线JBI：%1")
            .arg(plan.onlineCorrectionJbi));
    const QString stageText = plan.requiresGlobalLocalization
        ? QStringLiteral("全局定位完成后的在线阶段")
        : onlineCorrectionStartModeToText(plan.mode);
    appendInfo(QStringLiteral("在线纠偏：%1，等待机器人状态满足后启动在线 JBI：%2")
                   .arg(stageText, plan.onlineCorrectionJbi));
    beginCorrectionRobotStartGate(
        CorrectionRobotStartKind::OnlineCorrection,
        plan.onlineCorrectionJbi);
    return true;
}

void UiController::collectOnlineAutoCorrectionScanPoint(
    const RuntimeTrackingResult& result)
{
    if (!m_onlineAutoCorrectionWorkflow.acceptsGlobalLocalizationFrame()) {
        return;
    }
    if (!result.valid) {
        return;
    }

    int acceptedPointCount = 0;
    for (const auto& point : result.linePoints) {
        if (!point.valid) {
            continue;
        }
        m_onlineAutoGlobalLocalizationScanPoints.push_back({
            point.xMm, point.yMm, point.zMm});
        ++acceptedPointCount;
    }
    if (acceptedPointCount == 0 && result.sample.point.valid) {
        m_onlineAutoGlobalLocalizationScanPoints.push_back({
            result.sample.point.xMm,
            result.sample.point.yMm,
            result.sample.point.zMm});
        acceptedPointCount = 1;
    }
    if (acceptedPointCount == 0) {
        return;
    }
    m_view.setOnlineCorrectionStateText(
        QStringLiteral("在线纠偏：全局定位程序运行中\n全局定位JBI：%1\n已采集点：%2")
            .arg(m_onlineAutoCorrectionWorkflow.plan().globalLocateJbi)
            .arg(static_cast<int>(
                m_onlineAutoGlobalLocalizationScanPoints.size())));
}

void UiController::showLegacyInfoPrompt(const QString& title, const QString& message)
{
    m_view.showLegacyInfoPrompt(title, message);
}

void UiController::showLegacyWarningPrompt(const QString& title, const QString& message)
{
    m_view.showLegacyWarningPrompt(title, message);
}

bool UiController::confirmLegacyPrompt(const QString& title, const QString& message)
{
    return m_view.confirmLegacyPrompt(title, message);
}

void UiController::appendInfo(const QString& text)
{
    m_view.appendLog(Logger::info(text));
}

void UiController::appendWarning(const QString& text)
{
    m_view.appendLog(Logger::warning(text));
}

} // namespace i2r_rlc::ui
