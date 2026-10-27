/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        MainWidget.cpp
 * @brief       主界面 QWidget 实现
 * @details     构建工业视觉运行界面并把用户操作转换为信号。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QFontDatabase>
#include <QGridLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPen>
#include <QMessageBox>
#include <QMetaObject>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QStringList>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include "MainWidget.h"
#include "TrackingFramePresentation.h"

namespace i2r_rlc::ui {

namespace {

/** @brief 仅在实时原图实际 paintEvent 返回后报告新曝光；重绘同帧不累计，指标不参与业务。 */
class CMeasuredImageLabel final : public QLabel {
public:
    CMeasuredImageLabel(QWidget *parent, std::function<void(qint64, qint64, qint64)> onPainted)
        : QLabel(parent), m_onPainted(std::move(onPainted))
    {
        setProperty("previewFrameId", -1LL);
        setProperty("lastPaintedFrameId", -1LL);
        setProperty("paintedFrameCount", 0LL);
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QLabel::paintEvent(event);
        const auto frameId = property("previewFrameId").toLongLong();
        if (pixmap().isNull() || frameId < 0
            || frameId <= property("lastPaintedFrameId").toLongLong()) { return; }
        const auto exposureMs = property("exposureTimestampMs").toLongLong();
        const auto finishedMs = QDateTime::currentMSecsSinceEpoch();
        setProperty("lastPaintedFrameId", frameId);
        setProperty("paintedFrameCount", property("paintedFrameCount").toLongLong() + 1);
        m_onPainted(frameId, exposureMs, finishedMs);
    }

private:
    std::function<void(qint64, qint64, qint64)> m_onPainted;
};

constexpr int kMainWindowWidth = 1340;
constexpr int kMainWindowHeight = 930;
constexpr int kOuterMargin = 10;
constexpr int kPanelSpacing = 8;
constexpr int kImageCanvasWidth = 775;
constexpr int kImageCanvasHeight = 578;
constexpr int kImageViewWidth = kImageCanvasWidth + 16;
constexpr int kImageViewHeight = kImageCanvasHeight + 28;
constexpr int kRightPanelWidth = 520;
constexpr int kStatusPanelHeight = 176;
constexpr int kDeviceControlPanelHeight = 88;
constexpr int kWorkpiecePanelHeight = 86;
constexpr int kFunctionPanelHeight =
    kImageViewHeight - kStatusPanelHeight - kDeviceControlPanelHeight
    - kWorkpiecePanelHeight - 3 * kPanelSpacing;
constexpr int kEntryPanelWidth = kMainWindowWidth - 2 * kOuterMargin;
constexpr int kEntryPanelHeight = 78;
constexpr int kLogPanelWidth = kMainWindowWidth - 2 * kOuterMargin;
constexpr int kLogPanelHeight = 210;

void setFixedPanelSize(QWidget* widget, int width, int height)
{
    widget->setMinimumSize(width, height);
    widget->setMaximumSize(width, height);
}

QPushButton* makeButton(
    const QString& text,
    QWidget* parent,
    const char* objectName,
    int width = 0)
{
    auto* button = new QPushButton(text, parent);
    button->setObjectName(QString::fromLatin1(objectName));
    if (width > 0) {
        button->setFixedWidth(width);
    }
    return button;
}

QLabel* makeHintLabel(const QString& text, QWidget* parent)
{
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    return label;
}

// 固定诊断区几何，长错误和坐标全文仍可通过提示查看，避免刷新挤动滚动视口。
void stabilizeDiagnosticLabel(QLabel* label, int lineCount)
{
    label->setTextFormat(Qt::PlainText);
    label->setFixedHeight(label->fontMetrics().lineSpacing() * lineCount + 4);
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    label->setToolTip(label->text());
}

void updateDiagnosticLabel(QLabel* label, const QString& text)
{
    if (label != nullptr && label->text() != text) {
        label->setText(text);
        label->setToolTip(text);
    }
}

QLabel* makeStatusTile(const QString& text, QWidget* parent, const char* objectName)
{
    auto* label = new QLabel(text, parent);
    label->setObjectName(QString::fromLatin1(objectName));
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumHeight(26);
    label->setStyleSheet(QStringLiteral(
        "QLabel { border-radius: 4px; padding: 3px; background: #5f6368; color: white; }"));
    return label;
}

void setStatusTileStyle(QLabel* label, const QString& text, const QString& color)
{
    if (label == nullptr) {
        return;
    }
    label->setText(text);
    label->setStyleSheet(QStringLiteral(
        "QLabel { border-radius: 4px; padding: 3px; background: %1; color: white; font-weight: 600; }")
                             .arg(color));
}

QString legacyStatusColor(const QString& text)
{
    if (text.contains(QStringLiteral("ON"))
        || text.contains(QStringLiteral("EXE"))
        || text.contains(QStringLiteral("WELDING"))
        || text.contains(QStringLiteral("PLAY"))) {
        return QStringLiteral("#188038");
    }
    if (text.contains(QStringLiteral("REMOTE"))) {
        return QStringLiteral("#1967d2");
    }
    if (text.contains(QStringLiteral("OFF"))
        || text.contains(QStringLiteral("STOP"))
        || text.contains(QStringLiteral("WAIT"))
        || text.contains(QStringLiteral("TEACH"))) {
        return QStringLiteral("#f9ab00");
    }
    return QStringLiteral("#5f6368");
}

QString findUiResourceFile(const QString& relativePath)
{
    const QStringList startDirectories{
        QCoreApplication::applicationDirPath(),
        QDir::currentPath()
    };

    for (const QString& startDirectory : startDirectories) {
        QDir directory(startDirectory);
        for (int depth = 0; depth < 8; ++depth) {
            const QStringList candidates{
                directory.absoluteFilePath(QStringLiteral("UI/resources/%1").arg(relativePath)),
                directory.absoluteFilePath(QStringLiteral("resources/%1").arg(relativePath))
            };
            for (const QString& candidate : candidates) {
                if (QFileInfo::exists(candidate) && QFileInfo(candidate).isFile()) {
                    return candidate;
                }
            }
            if (!directory.cdUp()) {
                break;
            }
        }
    }
    return {};
}

bool legacyPromptsSuppressed()
{
    return qEnvironmentVariableIsSet("I2R_RLC_SUPPRESS_LEGACY_PROMPTS")
        || QGuiApplication::platformName().compare(
               QStringLiteral("offscreen"),
               Qt::CaseInsensitive) == 0;
}

} // namespace

MainWidget::MainWidget(QWidget* parent)
    : QWidget(parent)
{
    setWindowTitle(tr("i2r_rlc 机器人定位与纠偏系统"));
    setFixedSize(kMainWindowWidth, kMainWindowHeight);

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(
        kOuterMargin, kOuterMargin, kOuterMargin, kOuterMargin);
    rootLayout->setSpacing(kPanelSpacing);

    auto* topLayout = new QHBoxLayout();
    topLayout->setSpacing(kPanelSpacing);
    rootLayout->addLayout(topLayout);

    auto* imageGroup = new QGroupBox(tr("图像显示"), this);
    setFixedPanelSize(imageGroup, kImageViewWidth, kImageViewHeight);
    auto* imageLayout = new QVBoxLayout(imageGroup);
    imageLayout->setContentsMargins(8, 20, 8, 8);
    m_imagePlaceholder = new CMeasuredImageLabel(imageGroup,
        [this](qint64 frameId, qint64 exposureMs, qint64 paintedAtMs) {
            emit liveFramePainted(frameId, exposureMs, paintedAtMs);
        });
    m_imagePlaceholder->setObjectName(QStringLiteral("liveImageCanvas"));
    m_imagePlaceholder->setText(tr("实时相机尚未启动"));
    m_imagePlaceholder->setAlignment(Qt::AlignCenter);
    m_imagePlaceholder->setFixedSize(kImageCanvasWidth, kImageCanvasHeight);
    m_imagePlaceholder->setStyleSheet(QStringLiteral(
        "QLabel { border: 1px solid #777; background: #101010; color: #f1f3f4; }"));
    imageLayout->addWidget(m_imagePlaceholder);
    m_livePreviewImage = new CMeasuredImageLabel(m_imagePlaceholder,
        [this](qint64 frameId, qint64 exposureMs, qint64 paintedAtMs) {
            emit liveFramePainted(frameId, exposureMs, paintedAtMs);
        });
    m_livePreviewImage->setObjectName(QStringLiteral("livePreviewImage"));
    m_livePreviewImage->setAlignment(Qt::AlignCenter);
    m_livePreviewImage->setGeometry(kImageCanvasWidth - 308, 8, 300, 224);
    m_livePreviewImage->setStyleSheet(QStringLiteral(
        "QLabel { border: 1px solid #777; background: #101010; }"));
    m_livePreviewImage->setToolTip(tr("实时相机原图"));
    m_livePreviewImage->hide();
    topLayout->addWidget(imageGroup);

    auto* rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(kPanelSpacing);
    topLayout->addLayout(rightLayout);

    auto* statusGroup = new QGroupBox(tr("状态监控"), this);
    statusGroup->setObjectName(QStringLiteral("legacyStatusGroup"));
    setFixedPanelSize(statusGroup, kRightPanelWidth, kStatusPanelHeight);
    auto* statusLayout = new QGridLayout(statusGroup);
    statusLayout->setContentsMargins(8, 20, 8, 8);
    statusLayout->setHorizontalSpacing(6);
    statusLayout->setVerticalSpacing(5);
    m_modeStatusLabel = makeStatusTile(tr("UNKNOWN"), statusGroup, "modeStatusLabel");
    m_servoStatusLabel = makeStatusTile(tr("SERVO OFF"), statusGroup, "servoStatusLabel");
    m_runStatusLabel = makeStatusTile(tr("ROBOTS STOP!"), statusGroup, "runStatusLabel");
    m_weldStatusLabel = makeStatusTile(tr("WAIT"), statusGroup, "weldStatusLabel");
    m_laserStatusLabel = makeStatusTile(tr("激光指令 OFF"), statusGroup, "laserStatusLabel");
    m_filterStatusLabel = makeStatusTile(tr("滤光片 OFF"), statusGroup, "filterStatusLabel");
    m_deviceStatusLabel = makeHintLabel(tr("机器人：--"), statusGroup);
    m_deviceStatusLabel->setObjectName(QStringLiteral("deviceStatusLabel"));
    m_exposureStatusLabel = makeHintLabel(tr("曝光：-- us"), statusGroup);
    m_exposureStatusLabel->setObjectName(QStringLiteral("exposureStatusLabel"));
    statusLayout->addWidget(m_modeStatusLabel, 0, 0);
    statusLayout->addWidget(m_servoStatusLabel, 0, 1);
    statusLayout->addWidget(m_runStatusLabel, 1, 0);
    statusLayout->addWidget(m_weldStatusLabel, 1, 1);
    statusLayout->addWidget(m_laserStatusLabel, 2, 0);
    statusLayout->addWidget(m_filterStatusLabel, 2, 1);
    statusLayout->addWidget(m_deviceStatusLabel, 3, 0, 1, 2);
    statusLayout->addWidget(m_exposureStatusLabel, 4, 0, 1, 2);
    rightLayout->addWidget(statusGroup);

    auto* deviceControlGroup = new QGroupBox(tr("设备控制"), this);
    deviceControlGroup->setObjectName(QStringLiteral("deviceControlGroup"));
    setFixedPanelSize(deviceControlGroup, kRightPanelWidth, kDeviceControlPanelHeight);
    auto* deviceControlLayout = new QGridLayout(deviceControlGroup);
    deviceControlLayout->setContentsMargins(8, 20, 8, 6);
    deviceControlLayout->setHorizontalSpacing(6);
    deviceControlLayout->setVerticalSpacing(6);
    deviceControlLayout->addWidget(makeHintLabel(tr("曝光(us)："), deviceControlGroup), 0, 0);
    m_exposureControlEdit = new QLineEdit(deviceControlGroup);
    m_exposureControlEdit->setObjectName(QStringLiteral("exposureControlEdit"));
    m_exposureControlEdit->setValidator(new QIntValidator(1, 1000000, m_exposureControlEdit));
    m_exposureControlEdit->setText(QStringLiteral("1500"));
    m_setExposureButton =
        makeButton(tr("设置曝光"), deviceControlGroup, "setExposureButton", 82);
    m_laserControlButton =
        makeButton(tr("激光指令 ON"), deviceControlGroup, "laserControlButton", 96);
    m_laserControlButton->setCheckable(true);
    m_laserControlButton->setChecked(true);
    deviceControlLayout->addWidget(m_exposureControlEdit, 0, 1);
    deviceControlLayout->addWidget(m_setExposureButton, 1, 0);
    deviceControlLayout->addWidget(m_laserControlButton, 1, 1);
    rightLayout->addWidget(deviceControlGroup);

    auto* workpieceGroup = new QGroupBox(tr("当前工件"), this);
    workpieceGroup->setObjectName(QStringLiteral("workpieceGroup"));
    setFixedPanelSize(workpieceGroup, kRightPanelWidth, kWorkpiecePanelHeight);
    auto* workpieceLayout = new QGridLayout(workpieceGroup);
    workpieceLayout->setContentsMargins(8, 20, 8, 6);
    workpieceLayout->setHorizontalSpacing(6);
    workpieceLayout->setVerticalSpacing(4);
    m_workpieceComboBox = new QComboBox(workpieceGroup);
    m_workpieceComboBox->setObjectName(QStringLiteral("workpieceComboBox"));
    m_selectWorkpieceButton =
        makeButton(tr("选择工件"), workpieceGroup, "selectWorkpieceButton", 82);
    m_currentWorkpieceLabel = makeHintLabel(tr("尚未选择工件"), workpieceGroup);
    m_currentWorkpieceLabel->setObjectName(QStringLiteral("currentWorkpieceLabel"));
    workpieceLayout->addWidget(m_workpieceComboBox, 0, 0);
    workpieceLayout->addWidget(m_selectWorkpieceButton, 0, 1);
    workpieceLayout->addWidget(m_currentWorkpieceLabel, 1, 0, 1, 2);
    rightLayout->addWidget(workpieceGroup);

    auto* functionGroup = new QGroupBox(tr("功能面板"), this);
    setFixedPanelSize(functionGroup, kRightPanelWidth, kFunctionPanelHeight);
    auto* functionLayout = new QHBoxLayout(functionGroup);
    functionLayout->setContentsMargins(8, 20, 8, 8);

    auto* robotSimConnectionGroup =
        new QGroupBox(tr("RobotSim 连接"), functionGroup);
    auto* robotSimLayout = new QGridLayout(robotSimConnectionGroup);
    robotSimLayout->setContentsMargins(8, 20, 8, 8);
    robotSimLayout->setHorizontalSpacing(6);
    robotSimLayout->setVerticalSpacing(6);
    auto* robotSimPortLabel = makeHintLabel(tr("端口"), robotSimConnectionGroup);
    m_robotSimPortLineEdit = new QLineEdit(robotSimConnectionGroup);
    m_robotSimPortLineEdit->setObjectName(QStringLiteral("robotSimPortLineEdit"));
    m_robotSimPortLineEdit->setPlaceholderText(tr("例如 61541"));
    m_robotSimPortLineEdit->setValidator(
        new QIntValidator(1, 65535, m_robotSimPortLineEdit));
    m_robotSimQueryButton = makeButton(
        tr("连接仿真场景"),
        robotSimConnectionGroup,
        "robotSimQueryButton");
    m_robotSimRealSceneButton = makeButton(
        tr("连接真实场景"),
        robotSimConnectionGroup,
        "robotSimRealSceneButton");
    m_digitalTwinStateLabel =
        makeHintLabel(tr("仿真：未连接"), robotSimConnectionGroup);
    m_digitalTwinStateLabel->setObjectName(QStringLiteral("digitalTwinStateLabel"));
    stabilizeDiagnosticLabel(m_digitalTwinStateLabel, 3);
    robotSimLayout->addWidget(robotSimPortLabel, 0, 0);
    robotSimLayout->addWidget(m_robotSimPortLineEdit, 0, 1);
    robotSimLayout->addWidget(m_robotSimQueryButton, 1, 0);
    robotSimLayout->addWidget(m_robotSimRealSceneButton, 1, 1);
    robotSimLayout->addWidget(m_digitalTwinStateLabel, 2, 0, 1, 2);
    robotSimLayout->setRowStretch(3, 1);
    functionLayout->addWidget(robotSimConnectionGroup, 1);

    m_featureStack = new QStackedWidget(functionGroup);
    m_featureStack->setObjectName(QStringLiteral("featureStack"));
    functionLayout->addWidget(m_featureStack, 1);
    rightLayout->addWidget(functionGroup);

    auto* defaultPage = new QWidget(m_featureStack);
    defaultPage->setObjectName(QStringLiteral("defaultFeaturePage"));
    auto* defaultLayout = new QVBoxLayout(defaultPage);
    defaultLayout->addWidget(makeHintLabel(
        tr("RobotSim 连接区始终可用。请点击底部大按钮打开对应功能面板。"),
        defaultPage));
    m_visionStateLabel = makeHintLabel(tr("视觉：等待实时相机帧"), defaultPage);
    m_visionStateLabel->setObjectName(QStringLiteral("visionStateLabel"));
    defaultLayout->addWidget(m_visionStateLabel);
    defaultLayout->addStretch();
    m_featureStack->addWidget(defaultPage);

    auto* calibrationPage = new QWidget(m_featureStack);
    calibrationPage->setObjectName(QStringLiteral("calibrationPage"));
    auto* calibrationLayout = new QVBoxLayout(calibrationPage);
    calibrationLayout->setContentsMargins(0, 0, 0, 0);
    calibrationLayout->setSpacing(6);
    m_calibrationPlacementImageLabel = new QLabel(calibrationPage);
    m_calibrationPlacementImageLabel->setObjectName(
        QStringLiteral("calibrationPlacementImageLabel"));
    m_calibrationPlacementImageLabel->setAlignment(Qt::AlignCenter);
    m_calibrationPlacementImageLabel->setFixedSize(220, 62);
    m_calibrationPlacementImageLabel->setStyleSheet(QStringLiteral(
        "QLabel { border: 1px solid #b0b0b0; background: #202124; color: #f1f3f4; }"));
    updateCalibrationPlacementImage();
    calibrationLayout->addWidget(m_calibrationPlacementImageLabel, 0, Qt::AlignHCenter);
    m_calibrationStack = new QStackedWidget(calibrationPage);
    m_calibrationStack->setObjectName(QStringLiteral("calibrationStack"));
    calibrationLayout->addWidget(m_calibrationStack);

    auto* calibrationSelectPage = new QWidget(m_calibrationStack);
    calibrationSelectPage->setObjectName(QStringLiteral("calibrationSelectPage"));
    auto* calibrationSelectLayout = new QVBoxLayout(calibrationSelectPage);
    calibrationSelectLayout->setContentsMargins(0, 0, 0, 0);
    calibrationSelectLayout->addWidget(makeHintLabel(
        tr("系统标定：请选择人工标定或自动标定。"), calibrationSelectPage));
    m_calibrationStateLabel = makeHintLabel(tr("标定：空闲"), calibrationSelectPage);
    m_calibrationStateLabel->setObjectName(QStringLiteral("calibrationStateLabel"));
    calibrationSelectLayout->addWidget(m_calibrationStateLabel);
    m_calibrationManualModeButton =
        makeButton(tr("人工标定"), calibrationSelectPage, "calibrationManualModeButton", 100);
    m_calibrationAutomaticModeButton =
        makeButton(tr("自动标定"), calibrationSelectPage, "calibrationAutomaticModeButton", 100);
    calibrationSelectLayout->addWidget(m_calibrationManualModeButton);
    calibrationSelectLayout->addWidget(m_calibrationAutomaticModeButton);
    calibrationSelectLayout->addStretch();
    m_calibrationStack->addWidget(calibrationSelectPage);

    auto* calibrationManualPage = new QWidget(m_calibrationStack);
    calibrationManualPage->setObjectName(QStringLiteral("calibrationManualPage"));
    auto* calibrationManualLayout = new QGridLayout(calibrationManualPage);
    calibrationManualLayout->setContentsMargins(0, 0, 0, 0);
    calibrationManualLayout->setHorizontalSpacing(4);
    calibrationManualLayout->setVerticalSpacing(4);
    calibrationManualLayout->addWidget(makeHintLabel(
        tr("需要采集 19 张标定板和 4 张激光图，后 4 张需同位置配对。"),
        calibrationManualPage), 0, 0, 1, 2);
    m_captureCalibrationBoardButton =
        makeButton(tr("采集标定板"), calibrationManualPage, "captureCalibrationBoardButton", 104);
    m_deleteLastCalibrationButton =
        makeButton(tr("删除上一点"), calibrationManualPage, "deleteLastCalibrationButton", 104);
    m_captureCalibrationLaserButton =
        makeButton(tr("采集激光图"), calibrationManualPage, "captureCalibrationLaserButton", 104);
    m_finishCalibrationButton =
        makeButton(tr("结束标定"), calibrationManualPage, "finishCalibrationButton", 104);
    auto* backFromManualButton =
        makeButton(tr("返回选择"), calibrationManualPage, "calibrationManualBackButton", 104);
    calibrationManualLayout->addWidget(m_captureCalibrationBoardButton, 1, 0);
    calibrationManualLayout->addWidget(m_deleteLastCalibrationButton, 1, 1);
    calibrationManualLayout->addWidget(m_captureCalibrationLaserButton, 2, 0);
    calibrationManualLayout->addWidget(m_finishCalibrationButton, 2, 1);
    calibrationManualLayout->addWidget(backFromManualButton, 3, 0);
    calibrationManualLayout->setRowStretch(5, 1);
    m_calibrationStack->addWidget(calibrationManualPage);

    auto* calibrationAutomaticPage = new QWidget(m_calibrationStack);
    calibrationAutomaticPage->setObjectName(QStringLiteral("calibrationAutomaticPage"));
    auto* calibrationAutomaticLayout = new QVBoxLayout(calibrationAutomaticPage);
    calibrationAutomaticLayout->setContentsMargins(0, 0, 0, 0);
    calibrationAutomaticLayout->setSpacing(6);
    calibrationAutomaticLayout->addWidget(makeHintLabel(
        tr("自动标定：请将首张标定板图像移动至红色框内，确认后再运行机器人标定程序。"),
        calibrationAutomaticPage));
    m_confirmAutomaticCalibrationButton =
        makeButton(tr("确认"), calibrationAutomaticPage, "confirmAutomaticCalibrationButton", 104);
    auto* finishAutomaticButton =
        makeButton(tr("结束标定"), calibrationAutomaticPage, "finishAutomaticCalibrationButton", 104);
    auto* backFromAutomaticButton =
        makeButton(tr("返回选择"), calibrationAutomaticPage, "calibrationAutomaticBackButton", 104);
    calibrationAutomaticLayout->addWidget(m_confirmAutomaticCalibrationButton);
    calibrationAutomaticLayout->addWidget(finishAutomaticButton);
    calibrationAutomaticLayout->addWidget(backFromAutomaticButton);
    calibrationAutomaticLayout->addStretch();
    m_calibrationStack->addWidget(calibrationAutomaticPage);
    m_featureStack->addWidget(calibrationPage);

    auto* offlineCorrectionPage = new QWidget(m_featureStack);
    offlineCorrectionPage->setObjectName(QStringLiteral("offlineCorrectionPage"));
    auto* offlineLayout = new QVBoxLayout(offlineCorrectionPage);
    offlineLayout->setContentsMargins(0, 0, 0, 0);
    offlineLayout->setSpacing(6);
    offlineLayout->addWidget(makeHintLabel(
        tr("离线纠偏：选择工件后自动加载示教轨迹"),
        offlineCorrectionPage));

    auto* offlineDetailsScrollArea = new QScrollArea(offlineCorrectionPage);
    offlineDetailsScrollArea->setObjectName(QStringLiteral("offlineCorrectionDetailsScrollArea"));
    offlineDetailsScrollArea->setWidgetResizable(true);
    offlineDetailsScrollArea->setFrameShape(QFrame::NoFrame);
    offlineDetailsScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    offlineDetailsScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    auto* offlineDetailsWidget = new QWidget(offlineDetailsScrollArea);
    auto* offlineDetailsLayout = new QVBoxLayout(offlineDetailsWidget);
    offlineDetailsLayout->setContentsMargins(0, 0, 4, 0);
    offlineDetailsLayout->setSpacing(4);
    m_offlineCorrectionStateLabel = makeHintLabel(tr("离线纠偏：空闲"), offlineDetailsWidget);
    m_offlineCorrectionStateLabel->setObjectName(QStringLiteral("offlineCorrectionStateLabel"));
    offlineDetailsLayout->addWidget(m_offlineCorrectionStateLabel);
    m_offlineCorrectionSyncStateLabel = makeHintLabel(
        tr("逐帧同步：空闲"), offlineDetailsWidget);
    m_offlineCorrectionSyncStateLabel->setObjectName(
        QStringLiteral("offlineCorrectionSyncStateLabel"));
    m_offlineCorrectionConsumedFrameLabel = makeHintLabel(
        tr("最近确认帧：无"), offlineDetailsWidget);
    m_offlineCorrectionConsumedFrameLabel->setObjectName(
        QStringLiteral("offlineCorrectionConsumedFrameLabel"));
    m_offlineCorrectionSyncErrorLabel = makeHintLabel(
        tr("最近同步错误：无"), offlineDetailsWidget);
    m_offlineCorrectionSyncErrorLabel->setObjectName(
        QStringLiteral("offlineCorrectionSyncErrorLabel"));
    offlineDetailsLayout->addWidget(m_offlineCorrectionSyncStateLabel);
    offlineDetailsLayout->addWidget(m_offlineCorrectionConsumedFrameLabel);
    offlineDetailsLayout->addWidget(m_offlineCorrectionSyncErrorLabel);
    offlineDetailsLayout->addStretch();
    offlineDetailsScrollArea->setWidget(offlineDetailsWidget);
    offlineLayout->addWidget(offlineDetailsScrollArea, 1);
    m_startOfflineCorrectionButton =
        makeButton(tr("启动/结束离线纠偏"), offlineCorrectionPage, "startOfflineCorrectionButton", 150);
    offlineLayout->addWidget(m_startOfflineCorrectionButton);
    m_featureStack->addWidget(offlineCorrectionPage);

    auto* onlineCorrectionPage = new QWidget(m_featureStack);
    onlineCorrectionPage->setObjectName(QStringLiteral("onlineCorrectionPage"));
    auto* onlineLayout = new QVBoxLayout(onlineCorrectionPage);
    onlineLayout->setContentsMargins(0, 0, 0, 0);
    onlineLayout->setSpacing(6);
    onlineLayout->addWidget(makeHintLabel(
        tr("在线纠偏：选择工件后自动加载示教轨迹，再启动在线纠偏。"),
        onlineCorrectionPage));

    auto* onlineDetailsScrollArea = new QScrollArea(onlineCorrectionPage);
    onlineDetailsScrollArea->setObjectName(QStringLiteral("onlineCorrectionDetailsScrollArea"));
    onlineDetailsScrollArea->setWidgetResizable(true);
    onlineDetailsScrollArea->setFrameShape(QFrame::NoFrame);
    onlineDetailsScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    onlineDetailsScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    auto* onlineDetailsWidget = new QWidget(onlineDetailsScrollArea);
    auto* onlineDetailsLayout = new QVBoxLayout(onlineDetailsWidget);
    onlineDetailsLayout->setContentsMargins(0, 0, 4, 0);
    onlineDetailsLayout->setSpacing(4);
    m_onlineLocalIcpCheckBox = new QCheckBox(tr("开启在线 ICP 局部定位"), onlineDetailsWidget);
    m_onlineLocalIcpCheckBox->setObjectName(QStringLiteral("onlineLocalIcpCheckBox"));
    m_onlineLocalIcpCheckBox->setChecked(true);
    m_onlineLocalIcpCheckBox->setToolTip(tr("可在运行中调整；关闭后停止新的局部配准和位姿写回。"));
    onlineDetailsLayout->addWidget(m_onlineLocalIcpCheckBox);
    m_onlineLocalIcpStabilityCheckBox = new QCheckBox(tr("在线 ICP 稳定更新保护"), onlineDetailsWidget);
    m_onlineLocalIcpStabilityCheckBox->setObjectName(QStringLiteral("onlineLocalIcpStabilityCheckBox"));
    m_onlineLocalIcpStabilityCheckBox->setChecked(false);
    m_onlineLocalIcpStabilityCheckBox->setToolTip(tr(
        "可在启动前设置或运行中切换；启用后要求焊点明确改善，并监测历史区域。"
        "历史几何未校核，仅作诊断，不作硬否决；不比较不同窗口的历史最低表面残差。"));
    onlineDetailsLayout->addWidget(m_onlineLocalIcpStabilityCheckBox);
    m_onlineLocalIcpDistanceRow = new QWidget(onlineDetailsWidget);
    m_onlineLocalIcpDistanceRow->setObjectName(QStringLiteral("onlineLocalIcpDistanceRow"));
    auto* localIcpDistanceLayout = new QHBoxLayout(m_onlineLocalIcpDistanceRow);
    localIcpDistanceLayout->setContentsMargins(0, 0, 0, 0);
    localIcpDistanceLayout->setSpacing(4);
    localIcpDistanceLayout->addWidget(new QLabel(tr("最大对应距离(mm)"), m_onlineLocalIcpDistanceRow));
    m_onlineLocalIcpDistanceSpinBox = new QDoubleSpinBox(m_onlineLocalIcpDistanceRow);
    m_onlineLocalIcpDistanceSpinBox->setObjectName(QStringLiteral("onlineLocalIcpDistanceSpinBox"));
    m_onlineLocalIcpDistanceSpinBox->setRange(0.1, 10.0);
    m_onlineLocalIcpDistanceSpinBox->setDecimals(1);
    m_onlineLocalIcpDistanceSpinBox->setSingleStep(0.5);
    m_onlineLocalIcpDistanceSpinBox->setValue(10.0);
    m_onlineLocalIcpDistanceSpinBox->setKeyboardTracking(false);
    m_onlineLocalIcpDistanceSpinBox->setToolTip(tr(
        "默认 10 mm，上限 10 mm：ICP 搜索对应点的最大距离，不是最终残差验收阈值；可在运行中调整。"));
    localIcpDistanceLayout->addWidget(m_onlineLocalIcpDistanceSpinBox);
    onlineDetailsLayout->addWidget(m_onlineLocalIcpDistanceRow);
    m_onlineLocalIcpTranslationRow = new QWidget(onlineDetailsWidget);
    m_onlineLocalIcpTranslationRow->setObjectName(QStringLiteral("onlineLocalIcpTranslationRow"));
    auto* localIcpTranslationLayout = new QHBoxLayout(m_onlineLocalIcpTranslationRow);
    localIcpTranslationLayout->setContentsMargins(0, 0, 0, 0);
    localIcpTranslationLayout->setSpacing(4);
    localIcpTranslationLayout->addWidget(new QLabel(tr("最大平移修正(mm)"), m_onlineLocalIcpTranslationRow));
    m_onlineLocalIcpTranslationSpinBox = new QDoubleSpinBox(m_onlineLocalIcpTranslationRow);
    m_onlineLocalIcpTranslationSpinBox->setObjectName(QStringLiteral("onlineLocalIcpTranslationSpinBox"));
    m_onlineLocalIcpTranslationSpinBox->setRange(0.01, 30.0);
    m_onlineLocalIcpTranslationSpinBox->setDecimals(2);
    m_onlineLocalIcpTranslationSpinBox->setSingleStep(0.5);
    m_onlineLocalIcpTranslationSpinBox->setValue(30.0);
    m_onlineLocalIcpTranslationSpinBox->setKeyboardTracking(false);
    m_onlineLocalIcpTranslationSpinBox->setToolTip(tr(
        "默认 30 mm，上限 30 mm：一次修正三轴合成平移长度的上限，超过时整次拒绝，不截断修正；不是对应点搜索距离。可在运行中调整。"));
    localIcpTranslationLayout->addWidget(m_onlineLocalIcpTranslationSpinBox);
    onlineDetailsLayout->addWidget(m_onlineLocalIcpTranslationRow);
    m_onlineFusionModeComboBox = new QComboBox(onlineDetailsWidget);
    m_onlineFusionModeComboBox->setObjectName(QStringLiteral("onlineFusionModeComboBox"));
    m_onlineFusionModeComboBox->addItems({tr("自动卡尔曼"), tr("手动比例"), tr("仅真实检测（关闭融合）")});
    m_onlineFusionModeComboBox->setToolTip(tr(
        "自动卡尔曼：按响应自适应融合仿真与原图精定位；手动比例：按下方权重融合；仅真实检测：输出原图接缝精定位点。"));
    onlineDetailsLayout->addWidget(m_onlineFusionModeComboBox);
    auto* fusionWeights = new QWidget(onlineDetailsWidget);
    auto* fusionWeightLayout = new QGridLayout(fusionWeights);
    fusionWeightLayout->setContentsMargins(0, 0, 0, 0);
    fusionWeightLayout->setHorizontalSpacing(3);
    m_onlineSimulationWeightSpinBox = new QDoubleSpinBox(fusionWeights);
    m_onlineSimulationWeightSpinBox->setObjectName(QStringLiteral("onlineSimulationWeightSpinBox"));
    m_onlineKcfWeightSpinBox = new QDoubleSpinBox(fusionWeights);
    m_onlineKcfWeightSpinBox->setObjectName(QStringLiteral("onlineKcfWeightSpinBox"));
    for (auto* spinBox : {m_onlineSimulationWeightSpinBox, m_onlineKcfWeightSpinBox}) {
        spinBox->setRange(0.0, 100.0);
        spinBox->setDecimals(1);
        spinBox->setSingleStep(1.0);
        spinBox->setSuffix(QStringLiteral(" %"));
        spinBox->setKeyboardTracking(false);
        spinBox->setEnabled(false);
        spinBox->setValue(50.0);
        spinBox->setToolTip(tr("仅手动比例模式可编辑；仿真与真实检测百分比之和为 100%。"));
    }
    fusionWeightLayout->addWidget(new QLabel(tr("仿真"), fusionWeights), 0, 0);
    fusionWeightLayout->addWidget(m_onlineSimulationWeightSpinBox, 0, 1);
    fusionWeightLayout->addWidget(new QLabel(tr("真实检测"), fusionWeights), 0, 2);
    fusionWeightLayout->addWidget(m_onlineKcfWeightSpinBox, 0, 3);
    onlineDetailsLayout->addWidget(fusionWeights);
    auto* coordinates = new QWidget(onlineDetailsWidget);
    coordinates->setObjectName(QStringLiteral("onlineCorrectionCoordinates"));
    auto* coordinateLayout = new QGridLayout(coordinates);
    coordinateLayout->setContentsMargins(0, 0, 0, 0);
    coordinateLayout->setHorizontalSpacing(3);
    coordinateLayout->setVerticalSpacing(2);
    const QStringList axes{tr("X"), tr("Y"), tr("Z")};
    const QStringList titles{
        tr("TCP · 机器人基（mm）"), tr("TCP · 变位机基（mm）"),
        tr("检测焊缝 · 机器人基（mm）"), tr("检测焊缝 · 变位机基（mm）")};
    for (int axis = 0; axis < 3; ++axis) {
        auto* label = new QLabel(axes.at(axis), coordinates);
        label->setAlignment(Qt::AlignCenter);
        coordinateLayout->addWidget(label, 0, axis);
        coordinateLayout->setColumnStretch(axis, 1);
    }
    for (int row = 0; row < 4; ++row) {
        auto* title = new QLabel(titles.at(row), coordinates);
        title->setToolTip(tr("变位机基指 RBCALIB 固定基座，不随外轴转动。"));
        coordinateLayout->addWidget(title, row * 2 + 1, 0, 1, 3);
        for (int axis = 0; axis < 3; ++axis) {
            auto* edit = new QLineEdit(QStringLiteral("--"), coordinates);
            edit->setObjectName(QStringLiteral("onlineCoordinate%1%2")
                                    .arg(row).arg(axes.at(axis)));
            edit->setReadOnly(true);
            edit->setAlignment(Qt::AlignRight);
            edit->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            edit->setFixedHeight(22);
            edit->setMinimumWidth(0);
            // 数字的位数和正负号不能改变列宽或触发布局重新分配。
            edit->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            m_onlineCoordinateEdits[row][axis] = edit;
            coordinateLayout->addWidget(edit, row * 2 + 2, axis);
        }
    }
    onlineDetailsLayout->addWidget(coordinates);
    m_onlineCoordinateStatusLabel = new QLabel(
        tr("TCP：未连接\nKCF：等待有效检测"), onlineDetailsWidget);
    m_onlineCoordinateStatusLabel->setObjectName(
        QStringLiteral("onlineCoordinateStatusLabel"));
    stabilizeDiagnosticLabel(m_onlineCoordinateStatusLabel, 2);
    onlineDetailsLayout->addWidget(m_onlineCoordinateStatusLabel);
    onlineDetailsLayout->addWidget(makeHintLabel(
        tr("起始点定位方式："), onlineDetailsWidget));
    m_onlineCorrectionStateLabel = makeHintLabel(tr("在线纠偏：空闲"), onlineDetailsWidget);
    m_onlineCorrectionStateLabel->setObjectName(QStringLiteral("onlineCorrectionStateLabel"));
    m_onlineCorrectionP001Label = makeHintLabel(tr("P001：未计算"), onlineDetailsWidget);
    m_onlineCorrectionP001Label->setObjectName(QStringLiteral("onlineCorrectionP001Label"));
    stabilizeDiagnosticLabel(m_onlineCorrectionStateLabel, 3);
    stabilizeDiagnosticLabel(m_onlineCorrectionP001Label, 8);
    m_onlineStartModeComboBox = new QComboBox(onlineDetailsWidget);
    m_onlineStartModeComboBox->setObjectName(QStringLiteral("onlineStartModeComboBox"));
    m_onlineStartModeComboBox->addItem(tr("反向扫描"));
    m_onlineStartModeComboBox->addItem(tr("全局定位修正后反向扫描"));
    m_onlineStartModeComboBox->addItem(tr("IBVS"));
    m_onlineStartModeComboBox->setCurrentIndex(0);
    m_startTrackingButton =
        makeButton(tr("启动在线纠偏"), onlineCorrectionPage, "startTrackingButton", 130);
    onlineDetailsLayout->addWidget(m_onlineCorrectionStateLabel);
    onlineDetailsLayout->addWidget(m_onlineCorrectionP001Label);
    m_onlineCorrectionSyncStateLabel = makeHintLabel(
        tr("逐帧同步：空闲"), onlineDetailsWidget);
    m_onlineCorrectionSyncStateLabel->setObjectName(
        QStringLiteral("onlineCorrectionSyncStateLabel"));
    m_onlineCorrectionConsumedFrameLabel = makeHintLabel(
        tr("最近确认帧：无"), onlineDetailsWidget);
    m_onlineCorrectionConsumedFrameLabel->setObjectName(
        QStringLiteral("onlineCorrectionConsumedFrameLabel"));
    m_onlineCorrectionSyncErrorLabel = makeHintLabel(
        tr("最近同步错误：无"), onlineDetailsWidget);
    m_onlineCorrectionSyncErrorLabel->setObjectName(
        QStringLiteral("onlineCorrectionSyncErrorLabel"));
    stabilizeDiagnosticLabel(m_onlineCorrectionSyncStateLabel, 1);
    stabilizeDiagnosticLabel(m_onlineCorrectionConsumedFrameLabel, 1);
    stabilizeDiagnosticLabel(m_onlineCorrectionSyncErrorLabel, 2);
    onlineDetailsLayout->addWidget(m_onlineCorrectionSyncStateLabel);
    onlineDetailsLayout->addWidget(m_onlineCorrectionConsumedFrameLabel);
    onlineDetailsLayout->addWidget(m_onlineCorrectionSyncErrorLabel);
    onlineDetailsLayout->addWidget(m_onlineStartModeComboBox);
    onlineDetailsLayout->addStretch();
    onlineDetailsScrollArea->setWidget(onlineDetailsWidget);
    onlineLayout->addWidget(onlineDetailsScrollArea, 1);
    onlineLayout->addWidget(m_startTrackingButton);
    m_featureStack->addWidget(onlineCorrectionPage);

    auto* ibvsPage = new QWidget(m_featureStack);
    ibvsPage->setObjectName(QStringLiteral("ibvsPage"));
    auto* ibvsLayout = new QGridLayout(ibvsPage);
    ibvsLayout->setContentsMargins(0, 0, 0, 0);
    ibvsLayout->setHorizontalSpacing(6);
    ibvsLayout->setVerticalSpacing(6);
    m_loadIbvsConfigButton = makeButton(tr("视觉伺服参数"), ibvsPage, "loadIbvsConfigButton", 104);
    m_startIbvsDryRunButton = makeButton(tr("启动视觉伺服"), ibvsPage, "startIbvsDryRunButton", 104);
    m_stepIbvsDryRunButton = makeButton(tr("视觉伺服单步"), ibvsPage, "stepIbvsDryRunButton", 104);
    m_stopIbvsDryRunButton = makeButton(tr("停止视觉伺服"), ibvsPage, "stopIbvsDryRunButton", 104);
    m_stepIbvsDryRunButton->setVisible(false);
    m_ibvsStateLabel = makeHintLabel(tr("视觉伺服：未配置"), ibvsPage);
    m_ibvsStateLabel->setObjectName(QStringLiteral("ibvsStateLabel"));
    ibvsLayout->addWidget(m_loadIbvsConfigButton, 0, 0);
    ibvsLayout->addWidget(m_startIbvsDryRunButton, 0, 1);
    ibvsLayout->addWidget(m_stepIbvsDryRunButton, 1, 0);
    ibvsLayout->addWidget(m_stopIbvsDryRunButton, 1, 1);
    ibvsLayout->addWidget(m_ibvsStateLabel, 2, 0, 1, 2);
    ibvsLayout->setRowStretch(3, 1);
    m_featureStack->addWidget(ibvsPage);

    auto* localizationPage = new QWidget(m_featureStack);
    localizationPage->setObjectName(QStringLiteral("localizationPage"));
    auto* localizationLayout = new QGridLayout(localizationPage);
    localizationLayout->setContentsMargins(0, 0, 0, 0);
    localizationLayout->setHorizontalSpacing(6);
    localizationLayout->setVerticalSpacing(6);
    m_localReplayButton = makeButton(tr("局部修正"), localizationPage, "localReplayButton", 110);
    m_globalIcpDryRunButton = makeButton(tr("全局定位"), localizationPage, "globalIcpDryRunButton", 120);
    m_clearLocalizationButton = makeButton(tr("清除定位状态"), localizationPage, "clearLocalizationButton", 110);
    auto* virtualLaserSourceLabel =
        makeHintLabel(tr("仿真激光来源"), localizationPage);
    m_globalLocateVirtualLaserSourceComboBox = new QComboBox(localizationPage);
    m_globalLocateVirtualLaserSourceComboBox->setObjectName(
        QStringLiteral("globalLocateVirtualLaserSourceComboBox"));
    m_globalLocateVirtualLaserSourceComboBox->addItem(tr("OCCT 本地模型"));
    m_globalLocateVirtualLaserSourceComboBox->addItem(tr("RobotSim 当前场景"));
    m_globalLocateVirtualLaserSourceComboBox->setToolTip(
        tr("选择 RobotSim 前，请先在左侧常驻连接区连接真实场景。"));
    auto* virtualPointCloudSourceLabel =
        makeHintLabel(tr("配准点云来源"), localizationPage);
    m_globalLocateVirtualPointCloudSourceComboBox =
        new QComboBox(localizationPage);
    m_globalLocateVirtualPointCloudSourceComboBox->setObjectName(
        QStringLiteral("globalLocateVirtualPointCloudSourceComboBox"));
    m_globalLocateVirtualPointCloudSourceComboBox->addItem(
        tr("weldment 模型采样"));
    m_globalLocateVirtualPointCloudSourceComboBox->addItem(
        tr("RobotSim 激光扫描"));
    m_globalLocateVirtualPointCloudSourceComboBox->setToolTip(
        tr("RobotSim 激光扫描点云需要先连接真实场景，并在扫描期间逐定位段累计三维激光点。"));

    auto* localizationDetailsScrollArea = new QScrollArea(localizationPage);
    localizationDetailsScrollArea->setObjectName(
        QStringLiteral("localizationDetailsScrollArea"));
    localizationDetailsScrollArea->setWidgetResizable(true);
    localizationDetailsScrollArea->setFrameShape(QFrame::NoFrame);
    localizationDetailsScrollArea->setHorizontalScrollBarPolicy(
        Qt::ScrollBarAlwaysOff);
    localizationDetailsScrollArea->setVerticalScrollBarPolicy(
        Qt::ScrollBarAsNeeded);

    auto* localizationDetailsWidget =
        new QWidget(localizationDetailsScrollArea);
    auto* localizationDetailsLayout =
        new QVBoxLayout(localizationDetailsWidget);
    localizationDetailsLayout->setContentsMargins(0, 0, 4, 0);
    localizationDetailsLayout->setSpacing(4);
    m_globalLocateRobotSimConnectionLabel =
        makeHintLabel(
            tr("OCCT 激光 + weldment 点云：无需连接"),
            localizationDetailsWidget);
    m_globalLocateRobotSimConnectionLabel->setObjectName(
        QStringLiteral("globalLocateRobotSimConnectionLabel"));
    m_localReplayButton->setVisible(false);
    m_localizationStateLabel =
        makeHintLabel(tr("定位：空闲"), localizationDetailsWidget);
    m_localizationStateLabel->setObjectName(QStringLiteral("localizationStateLabel"));
    localizationDetailsLayout->addWidget(m_globalLocateRobotSimConnectionLabel);
    localizationDetailsLayout->addWidget(m_localizationStateLabel);
    localizationDetailsLayout->addStretch();
    localizationDetailsScrollArea->setWidget(localizationDetailsWidget);
    localizationLayout->addWidget(m_globalIcpDryRunButton, 0, 0);
    localizationLayout->addWidget(m_clearLocalizationButton, 0, 1);
    localizationLayout->addWidget(virtualLaserSourceLabel, 1, 0);
    localizationLayout->addWidget(
        m_globalLocateVirtualLaserSourceComboBox, 1, 1);
    localizationLayout->addWidget(virtualPointCloudSourceLabel, 2, 0);
    localizationLayout->addWidget(
        m_globalLocateVirtualPointCloudSourceComboBox, 2, 1);
    localizationLayout->addWidget(
        localizationDetailsScrollArea, 3, 0, 1, 2);
    localizationLayout->setRowStretch(3, 1);
    m_featureStack->addWidget(localizationPage);

    auto* teachTrajectoryPage = new QWidget(m_featureStack);
    teachTrajectoryPage->setObjectName(QStringLiteral("teachTrajectoryPage"));
    auto* teachTrajectoryLayout = new QVBoxLayout(teachTrajectoryPage);
    teachTrajectoryLayout->setContentsMargins(0, 0, 0, 0);
    teachTrajectoryLayout->setSpacing(6);
    teachTrajectoryLayout->addWidget(makeHintLabel(
        tr("先选择输出目录和名称。点击开始后运行机器人程序。最后保存到文件"),
        teachTrajectoryPage));

    auto* teachOutputRow = new QHBoxLayout();
    teachOutputRow->setSpacing(6);
    auto* teachOutputLabel = makeHintLabel(tr("输出目录"), teachTrajectoryPage);
    teachOutputLabel->setFixedWidth(56);
    teachOutputRow->addWidget(teachOutputLabel);
    m_teachTrajectoryOutputDirectoryEdit = new QLineEdit(teachTrajectoryPage);
    m_teachTrajectoryOutputDirectoryEdit->setObjectName(
        QStringLiteral("teachTrajectoryOutputDirectoryEdit"));
    m_teachTrajectoryOutputDirectoryEdit->setMinimumWidth(112);
    m_chooseTeachTrajectoryOutputDirectoryButton =
        makeButton(tr("选择目录"), teachTrajectoryPage, "chooseTeachTrajectoryOutputDirectoryButton", 82);
    teachOutputRow->addWidget(m_teachTrajectoryOutputDirectoryEdit, 1);
    teachOutputRow->addWidget(m_chooseTeachTrajectoryOutputDirectoryButton);
    teachTrajectoryLayout->addLayout(teachOutputRow);

    auto* teachNameRow = new QHBoxLayout();
    teachNameRow->setSpacing(6);
    auto* teachNameLabel = makeHintLabel(tr("轨迹名称"), teachTrajectoryPage);
    teachNameLabel->setFixedWidth(56);
    teachNameRow->addWidget(teachNameLabel);
    m_teachTrajectoryBaseNameEdit = new QLineEdit(teachTrajectoryPage);
    m_teachTrajectoryBaseNameEdit->setObjectName(QStringLiteral("teachTrajectoryBaseNameEdit"));
    m_teachTrajectoryBaseNameEdit->setPlaceholderText(tr("例如 20260625TEST，不填 _W/_T"));
    m_teachTrajectoryBaseNameEdit->setMinimumWidth(112);
    teachNameRow->addWidget(m_teachTrajectoryBaseNameEdit, 1);
    teachTrajectoryLayout->addLayout(teachNameRow);

    auto* teachButtonRow = new QHBoxLayout();
    teachButtonRow->setSpacing(6);
    m_startTeachTrajectoryRecordingButton =
        makeButton(tr("开始"), teachTrajectoryPage, "startTeachTrajectoryRecordingButton", 112);
    m_saveTeachTrajectoryButton =
        makeButton(tr("保存到文件"), teachTrajectoryPage, "saveTeachTrajectoryButton", 112);
    teachButtonRow->addWidget(m_startTeachTrajectoryRecordingButton);
    teachButtonRow->addWidget(m_saveTeachTrajectoryButton);
    teachTrajectoryLayout->addLayout(teachButtonRow);

    auto* teachTrajectoryStateLabel =
        makeHintLabel(tr("未开始"), teachTrajectoryPage);
    teachTrajectoryStateLabel->setObjectName(QStringLiteral("teachTrajectoryStateLabel"));
    teachTrajectoryStateLabel->setVisible(false);
    teachTrajectoryLayout->addWidget(teachTrajectoryStateLabel);
    teachTrajectoryLayout->addStretch();
    m_featureStack->addWidget(teachTrajectoryPage);

    auto* entryGroup = new QGroupBox(tr("主功能入口"), this);
    setFixedPanelSize(entryGroup, kEntryPanelWidth, kEntryPanelHeight);
    auto* entryLayout = new QHBoxLayout(entryGroup);
    entryLayout->setContentsMargins(10, 20, 10, 8);
    entryLayout->setSpacing(10);
    m_entryCalibrationButton =
        makeButton(tr("系统\n标定"), entryGroup, "entryCalibrationButton", 112);
    m_entryOfflineCorrectionButton =
        makeButton(tr("离线\n纠偏"), entryGroup, "entryOfflineCorrectionButton", 112);
    m_entryOnlineCorrectionButton =
        makeButton(tr("在线\n纠偏"), entryGroup, "entryOnlineCorrectionButton", 112);
    m_entryTeachTrajectoryButton =
        makeButton(tr("保存\n示教轨迹"), entryGroup, "entryTeachTrajectoryButton", 112);
    m_entryIbvsButton =
        makeButton(tr("视觉伺服"), entryGroup, "entryIbvsButton", 96);
    m_entryLocalizationButton =
        makeButton(tr("定位"), entryGroup, "entryLocalizationButton", 70);
    m_stopButton = makeButton(tr("停止"), entryGroup, "stopButton", 80);
    m_entryCalibrationButton->setMinimumHeight(42);
    m_entryOfflineCorrectionButton->setMinimumHeight(42);
    m_entryOnlineCorrectionButton->setMinimumHeight(42);
    m_entryTeachTrajectoryButton->setMinimumHeight(42);
    m_entryCalibrationButton->setCheckable(true);
    m_entryOfflineCorrectionButton->setCheckable(true);
    m_entryOnlineCorrectionButton->setCheckable(true);
    m_entryTeachTrajectoryButton->setCheckable(true);
    entryLayout->addWidget(m_entryCalibrationButton);
    entryLayout->addWidget(m_entryOfflineCorrectionButton);
    entryLayout->addWidget(m_entryOnlineCorrectionButton);
    entryLayout->addWidget(m_entryTeachTrajectoryButton);
    entryLayout->addWidget(m_entryIbvsButton);
    entryLayout->addWidget(m_entryLocalizationButton);
    entryLayout->addStretch();
    entryLayout->addWidget(m_stopButton);
    rootLayout->addWidget(entryGroup);

    auto* logGroup = new QGroupBox(tr("日志"), this);
    setFixedPanelSize(logGroup, kLogPanelWidth, kLogPanelHeight);
    auto* logLayout = new QVBoxLayout(logGroup);
    logLayout->setContentsMargins(8, 20, 8, 8);
    m_logView = new QPlainTextEdit(logGroup);
    m_logView->setObjectName(QStringLiteral("logView"));
    m_logView->setReadOnly(true);
    m_logView->setUndoRedoEnabled(false);
    m_logView->setMaximumBlockCount(200);
    m_logView->setStyleSheet(QStringLiteral(
        "QPlainTextEdit { background: #202124; color: #e8eaed; padding: 4px; }"));
    m_logView->setFixedSize(kLogPanelWidth - 16, kLogPanelHeight - 28);
    logLayout->addWidget(m_logView);
    rootLayout->addWidget(logGroup);

    connect(m_entryCalibrationButton, &QPushButton::clicked,
            this, [this]() {
                const bool opening = activeExclusiveFeaturePanelIndex() != 1;
                toggleExclusiveFeaturePanel(1);
                if (opening && activeExclusiveFeaturePanelIndex() == 1) {
                    showLegacyInfoPrompt(
                        tr("提示!"),
                        tr("标定板放置位置参见帮助!请设定标定模式及标定方法!"));
                }
            });
    connect(m_entryOfflineCorrectionButton, &QPushButton::clicked,
            this, [this]() { toggleExclusiveFeaturePanel(2); });
    connect(m_entryOnlineCorrectionButton, &QPushButton::clicked,
            this, [this]() { toggleExclusiveFeaturePanel(3); });
    connect(m_entryTeachTrajectoryButton, &QPushButton::clicked,
            this, [this]() { toggleExclusiveFeaturePanel(6); });
    connect(m_entryIbvsButton, &QPushButton::clicked,
            this, [this]() {
                clearExclusiveFeaturePanel();
                showFeaturePanel(4);
            });
    connect(m_entryLocalizationButton, &QPushButton::clicked,
            this, [this]() {
                clearExclusiveFeaturePanel();
                showFeaturePanel(5);
            });
    connect(m_setExposureButton, &QPushButton::clicked,
            this, &MainWidget::emitCameraExposureRequested);
    connect(m_laserControlButton, &QPushButton::toggled,
            this, [this](bool enabled) {
                if (m_laserControlButton != nullptr) {
                    m_laserControlButton->setText(
                        enabled ? tr("激光指令 ON") : tr("激光指令 OFF"));
                }
                emit laserEnabledChanged(enabled);
            });
    connect(m_selectWorkpieceButton, &QPushButton::clicked,
            this, &MainWidget::emitCurrentWorkpieceSelected);
    connect(m_startTrackingButton, &QPushButton::clicked,
            this, &MainWidget::startWeldTrackingRequested);
    connect(m_onlineStartModeComboBox,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            &MainWidget::onlineCorrectionStartModeChanged);
    const auto emitProcessingOptions = [this]() {
        emit onlineProcessingOptionsChanged(m_onlineLocalIcpCheckBox->isChecked(),
            m_onlineFusionModeComboBox->currentIndex(),
            m_onlineSimulationWeightSpinBox->value() / 100.0);
    };
    connect(m_onlineLocalIcpCheckBox, &QCheckBox::toggled,
            this, [this, emitProcessingOptions](bool enabled) {
                m_onlineLocalIcpStabilityCheckBox->setVisible(enabled);
                m_onlineLocalIcpDistanceRow->setVisible(enabled);
                m_onlineLocalIcpTranslationRow->setVisible(enabled);
                emitProcessingOptions();
            });
    connect(m_onlineLocalIcpStabilityCheckBox, &QCheckBox::toggled,
            this, &MainWidget::onlineLocalIcpStabilityChanged);
    connect(m_onlineLocalIcpDistanceSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &MainWidget::onlineLocalIcpDistanceChanged);
    connect(m_onlineLocalIcpTranslationSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &MainWidget::onlineLocalIcpTranslationLimitChanged);
    connect(m_onlineFusionModeComboBox, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this, emitProcessingOptions](int index) {
                m_onlineSimulationWeightSpinBox->setEnabled(index == 1);
                m_onlineKcfWeightSpinBox->setEnabled(index == 1);
                emitProcessingOptions();
            });
    connect(m_onlineSimulationWeightSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this, emitProcessingOptions](double percentage) {
                const QSignalBlocker blocker(m_onlineKcfWeightSpinBox);
                m_onlineKcfWeightSpinBox->setValue(100.0 - percentage);
                emitProcessingOptions();
            });
    connect(m_onlineKcfWeightSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this, emitProcessingOptions](double percentage) {
                const QSignalBlocker blocker(m_onlineSimulationWeightSpinBox);
                m_onlineSimulationWeightSpinBox->setValue(100.0 - percentage);
                emitProcessingOptions();
            });

    connect(m_calibrationManualModeButton, &QPushButton::clicked,
            this, [this]() {
                showCalibrationModePanel(1);
                emit startManualCalibrationCaptureRequested();
            });
    connect(m_calibrationAutomaticModeButton, &QPushButton::clicked,
            this, [this]() {
                showCalibrationModePanel(2);
                emit previewAutomaticCalibrationCaptureRequested();
            });
    connect(m_captureCalibrationBoardButton, &QPushButton::clicked,
            this, &MainWidget::captureCalibrationBoardRequested);
    connect(m_deleteLastCalibrationButton, &QPushButton::clicked,
            this, &MainWidget::deleteLastCalibrationSampleRequested);
    connect(m_captureCalibrationLaserButton, &QPushButton::clicked,
            this, &MainWidget::captureCalibrationLaserRequested);
    connect(m_confirmAutomaticCalibrationButton, &QPushButton::clicked,
            this, &MainWidget::startAutomaticCalibrationCaptureRequested);
    connect(m_finishCalibrationButton, &QPushButton::clicked,
            this, &MainWidget::finishCalibrationCaptureRequested);
    connect(finishAutomaticButton, &QPushButton::clicked,
            this, &MainWidget::finishCalibrationCaptureRequested);
    connect(backFromManualButton, &QPushButton::clicked,
            this, [this]() {
                showCalibrationModePanel(0);
                setCalibrationStateText(tr("标定：空闲"));
                emit calibrationModeSelectionReturned();
            });
    connect(backFromAutomaticButton, &QPushButton::clicked,
            this, [this]() {
                showCalibrationModePanel(0);
                setCalibrationStateText(tr("标定：空闲"));
                emit calibrationModeSelectionReturned();
            });

    connect(m_startOfflineCorrectionButton, &QPushButton::clicked,
            this, &MainWidget::startOfflineCorrectionRequested);
    connect(m_chooseTeachTrajectoryOutputDirectoryButton, &QPushButton::clicked,
            this, &MainWidget::chooseTeachTrajectoryOutputDirectory);
    connect(m_startTeachTrajectoryRecordingButton, &QPushButton::clicked,
            this, &MainWidget::startTeachTrajectoryRecordingRequested);
    connect(m_saveTeachTrajectoryButton, &QPushButton::clicked,
            this, [this]() {
                emit saveTeachTrajectoryRequested(
                    m_teachTrajectoryOutputDirectoryEdit
                        ? m_teachTrajectoryOutputDirectoryEdit->text().trimmed()
                        : QString(),
                    m_teachTrajectoryBaseNameEdit
                        ? m_teachTrajectoryBaseNameEdit->text().trimmed()
                        : QString());
            });
    connect(m_loadIbvsConfigButton, &QPushButton::clicked,
            this, &MainWidget::loadIbvsConfigRequested);
    connect(m_startIbvsDryRunButton, &QPushButton::clicked,
            this, &MainWidget::startIbvsDryRunRequested);
    connect(m_stepIbvsDryRunButton, &QPushButton::clicked,
            this, &MainWidget::stepIbvsDryRunRequested);
    connect(m_stopIbvsDryRunButton, &QPushButton::clicked,
            this, &MainWidget::stopIbvsDryRunRequested);
    connect(m_localReplayButton, &QPushButton::clicked,
            this, &MainWidget::runLocalCorrectionReplayRequested);
    connect(m_globalIcpDryRunButton, &QPushButton::clicked,
            this, &MainWidget::globalLocateRequested);
    connect(
        m_globalLocateVirtualLaserSourceComboBox,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            emit globalLocateVirtualLaserConfigurationChanged();
        });
    connect(
        m_globalLocateVirtualPointCloudSourceComboBox,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            emit globalLocateVirtualLaserConfigurationChanged();
        });
    connect(
        m_robotSimPortLineEdit,
        &QLineEdit::textChanged,
        this,
        [this]() {
            emit globalLocateVirtualLaserConfigurationChanged();
        });
    connect(m_clearLocalizationButton, &QPushButton::clicked,
            this, &MainWidget::clearLocalizationResultRequested);
    connect(m_robotSimQueryButton, &QPushButton::clicked,
            this, &MainWidget::emitRobotSimVirtualReferenceRequested);
    connect(m_robotSimRealSceneButton, &QPushButton::clicked,
            this, &MainWidget::emitRobotSimRealSceneConnectionRequested);
    connect(m_stopButton, &QPushButton::clicked,
            this, [this]() {
                emit stopRequested();
            });

    updateExclusiveEntryButtons();
    setLegacyStatusPanel(
        tr("UNKNOWN"),
        tr("SERVO OFF"),
        tr("ROBOTS STOP!"),
        tr("WAIT"),
        true,
        false,
        tr("机器人未连接"),
        tr("--"),
        tr("1500"));
}

void MainWidget::setWorkpieceChoices(
    const QStringList& displayNames,
    const QStringList& profileIds)
{
    if (!m_workpieceComboBox) {
        return;
    }

    m_workpieceComboBox->clear();
    const int count = std::min(displayNames.size(), profileIds.size());
    for (int index = 0; index < count; ++index) {
        if (profileIds.at(index) != QStringLiteral("chufengkou_zheng20")) {
            continue;
        }
        m_workpieceComboBox->addItem(displayNames.at(index), profileIds.at(index));
    }
    if (m_workpieceComboBox->count() > 0) {
        m_workpieceComboBox->setCurrentIndex(0);
        m_selectWorkpieceButton->setEnabled(true);
    } else if (m_selectWorkpieceButton) {
        m_selectWorkpieceButton->setEnabled(false);
    }
}

void MainWidget::setCurrentWorkpieceText(const QString& text)
{
    if (m_currentWorkpieceLabel) {
        m_currentWorkpieceLabel->setText(text);
    }
}

void MainWidget::setTeachTrajectoryOutputDirectory(const QString& directoryPath)
{
    if (m_teachTrajectoryOutputDirectoryEdit) {
        m_teachTrajectoryOutputDirectoryEdit->setText(directoryPath);
    }
}

void MainWidget::setTeachTrajectoryStateText(const QString& text)
{
    if (QCoreApplication::instance() != nullptr
        && QThread::currentThread() != QCoreApplication::instance()->thread()) {
        return;
    }
    if (QCoreApplication::closingDown()) {
        return;
    }

    if (auto* label = findChild<QLabel*>(QStringLiteral("teachTrajectoryStateLabel"))) {
        label->setText(text);
    }
}

void MainWidget::showFrame(const i2r_rlc::camera::CameraFrame& frame)
{
    if (m_trackingDisplayActive) {
        m_virtualLaserOverlayPoints.clear();
        m_virtualWeldPointOverlayVisible = false;
        m_detectedWeldPointOverlayVisible = false;
        m_realLaserOverlayPoints.clear();
    }
    m_trackingDisplayActive = false;
    m_livePreviewImage->hide();
    m_trackingFrameCaption.clear();
    m_liveFrameId = frame.frameId;
    m_liveExposureMs = frame.timestamp.isValid() ? frame.timestamp.toMSecsSinceEpoch() : -1;
    m_imagePlaceholder->setProperty("previewFrameId", m_liveFrameId);
    m_imagePlaceholder->setProperty("exposureTimestampMs", m_liveExposureMs);
    m_currentImage = frame.image.isNull() ? frame.previewImage : frame.image;
    m_liveImage = m_currentImage;
    if (m_currentImage.isNull()) {
        setImagePlaceholderText(tr("未加载图像"));
        return;
    }

    updateDisplayedImage();
    updateImageAttribution();
}

void MainWidget::showLivePreview(const i2r_rlc::camera::CameraFrame& frame)
{
    const auto source = frame.image.isNull() ? frame.previewImage : frame.image;
    if (source.isNull()) { return; }
    m_liveFrameId = frame.frameId;
    m_liveExposureMs = frame.timestamp.isValid() ? frame.timestamp.toMSecsSinceEpoch() : -1;
    m_liveImage = source;
    auto *target = m_trackingDisplayActive ? m_livePreviewImage : m_imagePlaceholder;
    target->setPixmap(QPixmap::fromImage(source).scaled(
        target->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    target->setProperty("previewFrameId", m_liveFrameId);
    target->setProperty("exposureTimestampMs", m_liveExposureMs);
    if (m_trackingDisplayActive) { m_livePreviewImage->show(); }
}

void MainWidget::updateImageAttribution()
{
    m_imagePlaceholder->setProperty("detectionFrameId", m_detectionFrameId);
    // 曝光身份保留在属性与日志，来源/控制资格保留悬浮说明，不覆盖检测画面。
    m_imagePlaceholder->setToolTip(m_trackingFrameCaption);
}

void MainWidget::setVirtualLaserOverlay(const QVector<QPointF>& imagePoints)
{
    m_virtualLaserOverlayPoints = imagePoints;
    if (!m_currentImage.isNull()) {
        updateDisplayedImage();
    }
}

void MainWidget::setVirtualWeldPointOverlay(
    const QPointF& imagePoint,
    bool visible)
{
    m_virtualWeldPointOverlay = imagePoint;
    m_virtualWeldPointOverlayVisible = visible;
    if (!m_currentImage.isNull()) {
        updateDisplayedImage();
    }
}

void MainWidget::setVirtualReferenceOverlay(
    const QVector<QPointF>& laserLineImagePoints,
    const QPointF& weldPointImage,
    bool weldPointVisible)
{
    m_virtualLaserOverlayPoints = laserLineImagePoints;
    m_virtualWeldPointOverlay = weldPointImage;
    m_virtualWeldPointOverlayVisible = weldPointVisible;
    if (!m_currentImage.isNull()) {
        updateDisplayedImage();
    }
}

void MainWidget::setDetectedWeldPointOverlay(
    const QPointF& imagePoint,
    bool visible)
{
    m_detectedWeldPointOverlay = imagePoint;
    m_detectedWeldPointOverlayVisible = visible;
    m_detectedWeldPointOverlayEstimated = false;
    if (!m_currentImage.isNull()) {
        updateDisplayedImage();
    }
}

void MainWidget::setRealLaserOverlay(const QVector<QPointF>& imagePoints)
{
    m_realLaserOverlayPoints = imagePoints;
}

void MainWidget::showTrackingFrame(const TrackingFramePresentation& presentation)
{
    m_trackingDisplayActive = true;
    m_imagePlaceholder->setProperty("previewFrameId", -1LL);
    camera::CameraFrame liveFrame;
    const bool hasNewerLiveFrame = m_liveFrameId >= presentation.frameId && !m_liveImage.isNull();
    liveFrame.frameId = hasNewerLiveFrame ? m_liveFrameId : presentation.frameId;
    liveFrame.image = hasNewerLiveFrame ? m_liveImage : presentation.sourceImage;
    const qint64 exposureMs = hasNewerLiveFrame ? m_liveExposureMs : presentation.exposureTimestampMs;
    liveFrame.timestamp = {};
    if (exposureMs >= 0) { liveFrame.timestamp = QDateTime::fromMSecsSinceEpoch(exposureMs); }
    if (m_livePreviewImage->property("previewFrameId").toLongLong() != liveFrame.frameId
        || m_livePreviewImage->pixmap().isNull()) {
        showLivePreview(liveFrame);
    }
    m_imagePlaceholder->setProperty("exposureTimestampMs", presentation.exposureTimestampMs);
    m_detectionFrameId = presentation.frameId;
    m_detectionExposureMs = presentation.exposureTimestampMs;
    m_currentImage = presentation.sourceImage;
    m_virtualLaserOverlayPoints = presentation.virtualLaserPoints;
    m_virtualWeldPointOverlay = presentation.virtualWeldPoint;
    m_virtualWeldPointOverlayVisible = presentation.virtualWeldPointVisible;
    m_detectedWeldPointOverlay = presentation.detectedWeldPoint;
    m_detectedWeldPointOverlayVisible = presentation.detectedWeldPointVisible;
    m_detectedWeldPointOverlayEstimated = presentation.detectedWeldPointEstimated;
    m_realLaserOverlayPoints.clear();
    m_trackingFrameCaption = presentation.caption;
    updateDisplayedImage();
    updateImageAttribution();
    m_livePreviewImage->show();
    m_livePreviewImage->raise();
}

void MainWidget::setImagePlaceholderText(const QString& text)
{
    m_livePreviewImage->hide();
    m_trackingDisplayActive = false;
    m_detectionFrameId = -1;
    m_detectionExposureMs = -1;
    m_liveFrameId = -1;
    m_liveExposureMs = -1;
    m_imagePlaceholder->setProperty("previewFrameId", -1LL);
    m_imagePlaceholder->setProperty("lastPaintedFrameId", -1LL);
    m_imagePlaceholder->setProperty("paintedFrameCount", 0LL);
    m_livePreviewImage->setProperty("previewFrameId", -1LL);
    m_livePreviewImage->setProperty("lastPaintedFrameId", -1LL);
    m_livePreviewImage->setProperty("paintedFrameCount", 0LL);
    m_imagePlaceholder->setToolTip({});
    m_currentImage = {};
    m_liveImage = {};
    m_trackingFrameCaption.clear();
    m_imagePlaceholder->setPixmap(QPixmap());
    m_imagePlaceholder->setText(text);
}

void MainWidget::setRobotStateText(const QString& text)
{
    if (m_robotStateLabel != nullptr) {
        m_robotStateLabel->setText(text);
    } else {
        appendLog(text);
    }
}

void MainWidget::setLegacyStatusPanel(
    const QString& modeText,
    const QString& servoText,
    const QString& runText,
    const QString& weldText,
    bool laserEnabled,
    bool filterEnabled,
    const QString& deviceText,
    const QString& fpsText,
    const QString& exposureText)
{
    Q_UNUSED(fpsText);
    setStatusTileStyle(m_modeStatusLabel, modeText, legacyStatusColor(modeText));
    setStatusTileStyle(m_servoStatusLabel, servoText, legacyStatusColor(servoText));
    setStatusTileStyle(m_runStatusLabel, runText, legacyStatusColor(runText));
    setStatusTileStyle(m_weldStatusLabel, weldText, legacyStatusColor(weldText));
    setStatusTileStyle(
        m_laserStatusLabel,
        laserEnabled ? tr("激光指令 ON") : tr("激光指令 OFF"),
        laserEnabled ? QStringLiteral("#188038") : QStringLiteral("#f9ab00"));
    setStatusTileStyle(
        m_filterStatusLabel,
        filterEnabled ? tr("滤光片 ON") : tr("滤光片 OFF"),
        filterEnabled ? QStringLiteral("#188038") : QStringLiteral("#f9ab00"));
    if (m_deviceStatusLabel != nullptr) {
        m_deviceStatusLabel->setText(tr("机器人：%1").arg(deviceText.isEmpty() ? tr("--") : deviceText));
    }
    if (m_exposureStatusLabel != nullptr) {
        m_exposureStatusLabel->setText(
            tr("曝光：%1 us").arg(exposureText.isEmpty() ? tr("--") : exposureText));
    }
}

void MainWidget::setLaserControlChecked(bool enabled)
{
    if (m_laserControlButton == nullptr) {
        return;
    }
    const QSignalBlocker blocker(m_laserControlButton);
    m_laserControlButton->setChecked(enabled);
    m_laserControlButton->setText(enabled ? tr("激光指令 ON") : tr("激光指令 OFF"));
}

void MainWidget::setExposureControlValue(int exposureUs)
{
    if (m_exposureControlEdit != nullptr && exposureUs > 0) {
        m_exposureControlEdit->setText(QString::number(exposureUs));
    }
}

void MainWidget::setCalibrationStateText(const QString& text)
{
    if (m_calibrationStateLabel != nullptr) {
        m_calibrationStateLabel->setText(text);
    }
}

void MainWidget::setOfflineCorrectionStateText(const QString& text)
{
    if (m_offlineCorrectionStateLabel != nullptr) {
        m_offlineCorrectionStateLabel->setText(text);
    }
}

void MainWidget::setOnlineCorrectionStateText(const QString& text)
{
    updateDiagnosticLabel(m_onlineCorrectionStateLabel, text);
}

void MainWidget::setOnlineProcessingOptions(
    bool localEnabled, int fusionMode, double simulationWeight)
{
    const QSignalBlocker localBlocker(m_onlineLocalIcpCheckBox);
    const QSignalBlocker modeBlocker(m_onlineFusionModeComboBox);
    const QSignalBlocker simulationBlocker(m_onlineSimulationWeightSpinBox);
    const QSignalBlocker kcfBlocker(m_onlineKcfWeightSpinBox);
    const int mode = fusionMode >= 0 && fusionMode <= 2 ? fusionMode : 0;
    const double weight = std::isfinite(simulationWeight)
        ? std::clamp(simulationWeight, 0.0, 1.0) : 0.5;
    m_onlineLocalIcpCheckBox->setChecked(localEnabled);
    m_onlineLocalIcpStabilityCheckBox->setVisible(localEnabled);
    m_onlineLocalIcpDistanceRow->setVisible(localEnabled);
    m_onlineLocalIcpTranslationRow->setVisible(localEnabled);
    m_onlineFusionModeComboBox->setCurrentIndex(mode);
    m_onlineSimulationWeightSpinBox->setValue(weight * 100.0);
    m_onlineKcfWeightSpinBox->setValue(100.0 - m_onlineSimulationWeightSpinBox->value());
    m_onlineSimulationWeightSpinBox->setEnabled(mode == 1);
    m_onlineKcfWeightSpinBox->setEnabled(mode == 1);
}

void MainWidget::setOnlineLocalIcpDistance(double distanceMm)
{
    const QSignalBlocker blocker(m_onlineLocalIcpDistanceSpinBox);
    m_onlineLocalIcpDistanceSpinBox->setValue(std::isfinite(distanceMm)
        ? std::clamp(distanceMm, 0.1, 10.0) : 10.0);
}

void MainWidget::setOnlineLocalIcpTranslationLimit(double translationMm)
{
    const QSignalBlocker blocker(m_onlineLocalIcpTranslationSpinBox);
    m_onlineLocalIcpTranslationSpinBox->setValue(std::isfinite(translationMm)
        ? std::clamp(translationMm, 0.01, 30.0) : 30.0);
}

void MainWidget::setOnlineLocalIcpStabilityEnabled(bool enabled)
{
    const QSignalBlocker blocker(m_onlineLocalIcpStabilityCheckBox);
    m_onlineLocalIcpStabilityCheckBox->setChecked(enabled);
}

void MainWidget::setOnlineCorrectionP001Text(const QString& text)
{
    updateDiagnosticLabel(m_onlineCorrectionP001Label, text);
}

void MainWidget::setOnlineCorrectionCoordinates(
    const std::array<QStringList, 4>& rows, const QString& status)
{
    for (int row = 0; row < 4; ++row) {
        for (int axis = 0; axis < 3; ++axis) {
            auto* edit = m_onlineCoordinateEdits[row][axis];
            const QString value = rows[row].value(axis, QStringLiteral("--"));
            if (edit != nullptr && edit->text() != value) {
                edit->setText(value);
            }
        }
    }
    if (m_onlineCoordinateStatusLabel != nullptr
        && m_onlineCoordinateStatusLabel->text() != status) {
        m_onlineCoordinateStatusLabel->setText(status);
        m_onlineCoordinateStatusLabel->setToolTip(status);
    }
}

void MainWidget::setOfflineCorrectionSyncDetails(
    const QString& stateText,
    const QString& consumedFrameText,
    const QString& errorText)
{
    if (m_offlineCorrectionSyncStateLabel != nullptr) {
        m_offlineCorrectionSyncStateLabel->setText(stateText);
    }
    if (m_offlineCorrectionConsumedFrameLabel != nullptr) {
        m_offlineCorrectionConsumedFrameLabel->setText(consumedFrameText);
    }
    if (m_offlineCorrectionSyncErrorLabel != nullptr) {
        m_offlineCorrectionSyncErrorLabel->setText(errorText);
    }
}

void MainWidget::setOnlineCorrectionSyncDetails(
    const QString& stateText,
    const QString& consumedFrameText,
    const QString& errorText)
{
    updateDiagnosticLabel(m_onlineCorrectionSyncStateLabel, stateText);
    updateDiagnosticLabel(m_onlineCorrectionConsumedFrameLabel, consumedFrameText);
    updateDiagnosticLabel(m_onlineCorrectionSyncErrorLabel, errorText);
}

void MainWidget::setVisionStateText(const QString& text)
{
    if (m_visionStateLabel != nullptr) {
        m_visionStateLabel->setText(text);
    }
}

void MainWidget::setIbvsStateText(const QString& text)
{
    m_ibvsStateLabel->setText(text);
}

void MainWidget::setLocalizationStateText(const QString& text)
{
    m_localizationStateLabel->setText(text);
}

void MainWidget::setGlobalLocateRobotSimConnectionText(const QString& text)
{
    if (m_globalLocateRobotSimConnectionLabel != nullptr) {
        m_globalLocateRobotSimConnectionLabel->setText(text);
    }
}

void MainWidget::setGlobalLocateVirtualLaserSourceLocked(bool locked)
{
    if (m_globalLocateVirtualLaserSourceComboBox == nullptr) {
        return;
    }

    m_globalLocateVirtualLaserSourceComboBox->setEnabled(!locked);
    if (m_globalLocateVirtualPointCloudSourceComboBox != nullptr) {
        m_globalLocateVirtualPointCloudSourceComboBox->setEnabled(!locked);
    }
    if (m_globalIcpDryRunButton != nullptr) {
        m_globalIcpDryRunButton->setEnabled(!locked);
    }
    m_globalLocateVirtualLaserSourceComboBox->setToolTip(
        locked
            ? tr("本次全局定位已锁定仿真激光和配准点云来源；流程结束后可为下一次重新选择。")
            : tr("选择 RobotSim 前，请先在左侧常驻连接区连接真实场景。"));
    if (m_globalLocateVirtualPointCloudSourceComboBox != nullptr) {
        m_globalLocateVirtualPointCloudSourceComboBox->setToolTip(
            locked
                ? tr("本次全局定位已锁定配准点云来源。")
                : tr("RobotSim 激光扫描点云需要先连接真实场景。"));
    }
}

void MainWidget::resetFeaturePanelsAfterStop()
{
    showCalibrationModePanel(0);

    if (m_onlineStartModeComboBox != nullptr) {
        const QSignalBlocker blocker(m_onlineStartModeComboBox);
        m_onlineStartModeComboBox->setCurrentIndex(0);
    }
    if (m_globalLocateVirtualLaserSourceComboBox != nullptr) {
        const QSignalBlocker blocker(m_globalLocateVirtualLaserSourceComboBox);
        m_globalLocateVirtualLaserSourceComboBox->setCurrentIndex(0);
    }
    if (m_globalLocateVirtualPointCloudSourceComboBox != nullptr) {
        const QSignalBlocker blocker(
            m_globalLocateVirtualPointCloudSourceComboBox);
        m_globalLocateVirtualPointCloudSourceComboBox->setCurrentIndex(0);
    }
    if (m_teachTrajectoryBaseNameEdit != nullptr) {
        m_teachTrajectoryBaseNameEdit->clear();
    }

    setGlobalLocateVirtualLaserSourceLocked(false);
    m_virtualLaserOverlayPoints.clear();
    m_virtualWeldPointOverlay = {};
    m_virtualWeldPointOverlayVisible = false;
    m_detectedWeldPointOverlay = {};
    m_detectedWeldPointOverlayVisible = false;
    m_realLaserOverlayPoints.clear();
    if (!m_currentImage.isNull()) {
        updateDisplayedImage();
    }
    updateExclusiveEntryButtons();
}

void MainWidget::setDigitalTwinStateText(const QString& text)
{
    updateDiagnosticLabel(m_digitalTwinStateLabel, text);
}

int MainWidget::globalLocateVirtualLaserSourceIndex() const
{
    return m_globalLocateVirtualLaserSourceComboBox != nullptr
        ? m_globalLocateVirtualLaserSourceComboBox->currentIndex()
        : 0;
}

int MainWidget::globalLocateVirtualPointCloudSourceIndex() const
{
    return m_globalLocateVirtualPointCloudSourceComboBox != nullptr
        ? m_globalLocateVirtualPointCloudSourceComboBox->currentIndex()
        : 0;
}

int MainWidget::globalLocateRobotSimPort() const
{
    return robotSimPort();
}

int MainWidget::robotSimPort() const
{
    if (m_robotSimPortLineEdit == nullptr) {
        return 0;
    }
    bool ok = false;
    const int port = m_robotSimPortLineEdit->text().trimmed().toInt(&ok);
    return ok && port > 0 && port <= 65535 ? port : 0;
}

void MainWidget::appendLog(const QString& text)
{
    if (QThread::currentThread() != thread()) {
        return;
    }
    if (QCoreApplication::closingDown()) {
        return;
    }

    if (!m_logView.isNull()) {
        m_logView->appendPlainText(text);
    }
}

void MainWidget::showLegacyInfoPrompt(const QString& title, const QString& message)
{
    if (QThread::currentThread() != thread()
        || QCoreApplication::closingDown()
        || legacyPromptsSuppressed()
        || m_legacyPromptActive) {
        return;
    }
    m_legacyPromptActive = true;
    QMessageBox::information(this, title, message, QMessageBox::Ok);
    m_legacyPromptActive = false;
}

void MainWidget::showLegacyWarningPrompt(const QString& title, const QString& message)
{
    if (QThread::currentThread() != thread()
        || QCoreApplication::closingDown()
        || legacyPromptsSuppressed()
        || m_legacyPromptActive) {
        return;
    }
    m_legacyPromptActive = true;
    QMessageBox::warning(this, title, message, QMessageBox::Ok);
    m_legacyPromptActive = false;
}

bool MainWidget::confirmLegacyPrompt(const QString& title, const QString& message)
{
    if (QThread::currentThread() != thread()
        || QCoreApplication::closingDown()
        || legacyPromptsSuppressed()
        || m_legacyPromptActive) {
        return true;
    }
    m_legacyPromptActive = true;
    const bool confirmed = QMessageBox::question(
               this,
               title,
               message,
               QMessageBox::Ok | QMessageBox::Cancel,
               QMessageBox::Ok) == QMessageBox::Ok;
    m_legacyPromptActive = false;
    return confirmed;
}

bool MainWidget::isFeaturePageActive(const QString& pageObjectName) const
{
    if (QThread::currentThread() != thread()) {
        return false;
    }
    if (QCoreApplication::closingDown() || m_featureStack.isNull()) {
        return false;
    }

    QWidget* currentPage = m_featureStack->currentWidget();
    return currentPage != nullptr && currentPage->objectName() == pageObjectName;
}

void MainWidget::emitCameraExposureRequested()
{
    if (m_exposureControlEdit == nullptr) {
        return;
    }
    bool ok = false;
    const int exposureUs = m_exposureControlEdit->text().trimmed().toInt(&ok);
    if (!ok || exposureUs <= 0) {
        appendLog(tr("曝光设置无效：请输入大于 0 的 us 整数"));
        return;
    }
    emit cameraExposureRequested(exposureUs);
}

void MainWidget::chooseTeachTrajectoryOutputDirectory()
{
    const QString currentDirectory =
        m_teachTrajectoryOutputDirectoryEdit
        ? m_teachTrajectoryOutputDirectoryEdit->text().trimmed()
        : QString();
    const QString directoryPath = QFileDialog::getExistingDirectory(
        this,
        tr("选择示教轨迹输出目录"),
        currentDirectory);

    if (!directoryPath.isEmpty() && m_teachTrajectoryOutputDirectoryEdit) {
        m_teachTrajectoryOutputDirectoryEdit->setText(directoryPath);
    }
}

void MainWidget::chooseOfflineCorrectionArchive()
{
    emit offlineCorrectionConfigurationRequested();
}

void MainWidget::chooseOnlineCorrectionConfiguration()
{
    emit onlineCorrectionConfigurationRequested();
}

void MainWidget::emitCurrentWorkpieceSelected()
{
    if (!m_workpieceComboBox || m_workpieceComboBox->currentIndex() < 0) {
        return;
    }
    emit workpieceSelected(
        m_workpieceComboBox->currentIndex(),
        m_workpieceComboBox->currentData().toString());
}

void MainWidget::emitRobotSimVirtualReferenceRequested()
{
    if (m_robotSimPortLineEdit == nullptr) {
        return;
    }

    bool ok = false;
    const int port = m_robotSimPortLineEdit->text().trimmed().toInt(&ok);
    if (!ok || port <= 0 || port > 65535) {
        setDigitalTwinStateText(tr("仿真：端口无效"));
        appendLog(tr("RobotSim 联调失败：请输入 1 到 65535 的端口号。"));
        return;
    }

    emit robotSimVirtualReferenceRequested(port);
}

void MainWidget::emitRobotSimRealSceneConnectionRequested()
{
    const int port = robotSimPort();
    if (port == 0) {
        setDigitalTwinStateText(tr("仿真：端口无效"));
        appendLog(tr("RobotSim 真实场景连接失败：请输入 1 到 65535 的端口号。"));
        return;
    }

    emit robotSimRealSceneConnectionRequested(port);
}

void MainWidget::showFeaturePanel(int index)
{
    if (QThread::currentThread() != thread()) {
        return;
    }
    if (QCoreApplication::closingDown() || m_featureStack.isNull()) {
        return;
    }

    QStackedWidget* stack = m_featureStack.data();
    if (stack == nullptr || index < 0 || index >= stack->count()) {
        return;
    }
    if (stack->currentIndex() == index) {
        updateExclusiveEntryButtons();
        return;
    }
    QWidget* targetPage = stack->widget(index);
    if (targetPage == nullptr) {
        return;
    }

    const QSignalBlocker blocker(stack);
    stack->setUpdatesEnabled(false);
    stack->setCurrentWidget(targetPage);
    stack->setUpdatesEnabled(true);
    updateExclusiveEntryButtons();
    if (stack->currentWidget() != targetPage) {
        appendLog(tr("功能面板切换失败：%1").arg(targetPage->objectName()));
    } else {
        appendLog(tr("功能面板已切换：%1").arg(targetPage->objectName()));
    }
}

void MainWidget::showCalibrationModePanel(int index)
{
    if (!m_calibrationStack || index < 0 || index >= m_calibrationStack->count()) {
        return;
    }
    m_calibrationStack->setCurrentIndex(index);
}

void MainWidget::toggleExclusiveFeaturePanel(int index)
{
    if (index != 1 && index != 2 && index != 3 && index != 6) {
        return;
    }

    if (activeExclusiveFeaturePanelIndex() == index) {
        clearExclusiveFeaturePanel();
        return;
    }

    if (activeExclusiveFeaturePanelIndex() != 0) {
        updateExclusiveEntryButtons();
        return;
    }

    showFeaturePanel(index);
    if (index == 1) {
        showCalibrationModePanel(0);
    }
    updateExclusiveEntryButtons();
}

void MainWidget::clearExclusiveFeaturePanel()
{
    const int closedIndex = activeExclusiveFeaturePanelIndex();
    if (closedIndex == 0) {
        return;
    }

    showFeaturePanel(0);
    if (activeExclusiveFeaturePanelIndex() != 0) {
        return;
    }
    if (closedIndex == 1) {
        showCalibrationModePanel(0);
    }
    updateExclusiveEntryButtons();
    emitExclusiveFeatureClosed(closedIndex);
}

int MainWidget::activeExclusiveFeaturePanelIndex() const
{
    const int index = m_featureStack.isNull() ? 0 : m_featureStack->currentIndex();
    return index == 1 || index == 2 || index == 3 || index == 6 ? index : 0;
}

void MainWidget::setCorrectionStartWaiting(bool offline, bool waiting)
{
    QPushButton* button = offline ? m_startOfflineCorrectionButton : m_startTrackingButton;
    button->setText(waiting ? tr("取消等待启动")
        : (offline ? tr("启动/结束离线纠偏") : tr("启动在线纠偏")));
}

void MainWidget::updateExclusiveEntryButtons()
{
    if (!m_entryCalibrationButton
        || !m_entryOfflineCorrectionButton
        || !m_entryOnlineCorrectionButton
        || !m_entryTeachTrajectoryButton) {
        return;
    }

    const int activeIndex = activeExclusiveFeaturePanelIndex();
    const bool hasActive = activeIndex != 0;
    m_entryCalibrationButton->setChecked(activeIndex == 1);
    m_entryOfflineCorrectionButton->setChecked(activeIndex == 2);
    m_entryOnlineCorrectionButton->setChecked(activeIndex == 3);
    m_entryTeachTrajectoryButton->setChecked(activeIndex == 6);

    m_entryCalibrationButton->setEnabled(
        !hasActive || activeIndex == 1);
    m_entryOfflineCorrectionButton->setEnabled(
        !hasActive || activeIndex == 2);
    m_entryOnlineCorrectionButton->setEnabled(
        !hasActive || activeIndex == 3);
    m_entryTeachTrajectoryButton->setEnabled(
        !hasActive || activeIndex == 6);
}

void MainWidget::emitExclusiveFeatureClosed(int index)
{
    switch (index) {
    case 1:
        emit calibrationFeaturePanelClosed();
        break;
    case 2:
        emit offlineCorrectionFeaturePanelClosed();
        break;
    case 3:
        emit onlineCorrectionFeaturePanelClosed();
        break;
    case 6:
        emit teachTrajectoryFeaturePanelClosed();
        break;
    default:
        break;
    }
}

void MainWidget::updateDisplayedImage()
{
    if (m_currentImage.isNull() || !m_imagePlaceholder) {
        return;
    }

    auto *target = m_imagePlaceholder;
    QPixmap pixmap = QPixmap::fromImage(m_currentImage)
                         .scaled(
                             target->size(),
                             Qt::KeepAspectRatio,
                             Qt::SmoothTransformation);
    if ((m_virtualLaserOverlayPoints.size() >= 2
        || m_virtualWeldPointOverlayVisible
        || m_detectedWeldPointOverlayVisible
        || m_realLaserOverlayPoints.size() >= 2
        || !m_trackingFrameCaption.isEmpty())
        && m_currentImage.width() > 0
        && m_currentImage.height() > 0){
        const double scaleX = static_cast<double>(pixmap.width())
            / static_cast<double>(m_currentImage.width());
        const double scaleY = static_cast<double>(pixmap.height())
            / static_cast<double>(m_currentImage.height());
        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);//开始

        if (!m_trackingDisplayActive && !m_trackingFrameCaption.isEmpty()) {
            painter.setPen(QPen(Qt::white, 1.0));
            painter.drawText(QPointF(10.0, 40.0), m_trackingFrameCaption);
        }

        if (m_virtualLaserOverlayPoints.size() >= 2) {
            painter.setPen(QPen(QColor(255, 215, 0), 2.0));
            for (int index = 1;
                index < m_virtualLaserOverlayPoints.size();
                ++index) {
                const QPointF previous = m_virtualLaserOverlayPoints[index - 1];
                const QPointF current = m_virtualLaserOverlayPoints[index];
                const double dx = current.x() - previous.x();
                const double dy = current.y() - previous.y();

                if (dx * dx + dy * dy > 100.0 * 100.0) {
                    continue;
                }

                painter.drawLine(
                    QPointF(previous.x() * scaleX, previous.y() * scaleY),
                    QPointF(current.x() * scaleX, current.y() * scaleY));
            }

            painter.setPen(QPen(QColor(255, 215, 0), 1.0));
            if (!m_trackingDisplayActive) { painter.drawText(
                QPointF(10.0, 20.0),
                tr("黄色：仿真激光参考线")); }
        }

        if (m_virtualWeldPointOverlayVisible) {

            const QPointF displayedPoint(
                m_virtualWeldPointOverlay.x() * scaleX,
                m_virtualWeldPointOverlay.y() * scaleY);
            painter.setPen(QPen(QColor(255, 64, 64), 2.0));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(displayedPoint, 6.0, 6.0);
            painter.drawLine(
                displayedPoint + QPointF(-9.0, 0.0),
                displayedPoint + QPointF(9.0, 0.0));
            painter.drawLine(
                displayedPoint + QPointF(0.0, -9.0),
                displayedPoint + QPointF(0.0, 9.0));
            painter.setPen(QPen(QColor(255, 64, 64), 1.0));
            if (!m_trackingDisplayActive) { painter.drawText(
                QPointF(10.0, 60.0),
                tr("红色：RobotSim仿真焊缝点")); }
        }
        if (m_detectedWeldPointOverlayVisible) {
            const QPointF displayedPoint(
                m_detectedWeldPointOverlay.x() * scaleX,
                m_detectedWeldPointOverlay.y() * scaleY);
            // KCF 业务点统一绿色；实测/续接来源由文字和日志表达。
            painter.setPen(QPen(QColor(0, 255, 0), 2.0));
            painter.drawLine(
                displayedPoint + QPointF(-15.0, 0.0),
                displayedPoint + QPointF(15.0, 0.0));
            painter.drawLine(
                displayedPoint + QPointF(0.0, -15.0),
                displayedPoint + QPointF(0.0, 15.0));
        }
        if (m_realLaserOverlayPoints.size() >= 2) {
            painter.setPen(QPen(QColor(0, 255, 255), 2.0));
            for (int index = 1;
                index < m_realLaserOverlayPoints.size();
                ++index) {
                const QPointF previous = m_realLaserOverlayPoints[index - 1];
                const QPointF current = m_realLaserOverlayPoints[index];
                const double dx = current.x() - previous.x();
                const double dy = current.y() - previous.y();
                // Steger 点按列排序；较大列间隔表示真实中心线断口。
                if (dx * dx + dy * dy > 25.0 * 25.0) {
                    continue;
                }
                painter.drawLine(
                    QPointF(previous.x() * scaleX, previous.y() * scaleY),
                    QPointF(current.x() * scaleX, current.y() * scaleY));
            }
            painter.setPen(QPen(QColor(0, 255, 255), 1.0));
            painter.drawText(
                QPointF(10.0, 40.0),
                tr("青色：Steger真实激光中心线"));
        }
    }
    target->setText({});
    target->setPixmap(pixmap);
}

void MainWidget::updateCalibrationPlacementImage()
{
    if (m_calibrationPlacementImageLabel == nullptr) {
        return;
    }

    const QString imagePath =
        findUiResourceFile(QStringLiteral("calibration/Calibplate.bmp"));
    const QPixmap sourcePixmap(imagePath);
    if (sourcePixmap.isNull()) {
        m_calibrationPlacementImageLabel->setPixmap({});
        m_calibrationPlacementImageLabel->setText(tr("标定板放置示意图缺失"));
        m_calibrationPlacementImageLabel->setProperty(
            "calibrationPlacementImageLoaded",
            false);
        return;
    }

    m_calibrationPlacementImageLabel->setText({});
    m_calibrationPlacementImageLabel->setPixmap(
        sourcePixmap.scaled(
            m_calibrationPlacementImageLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
    m_calibrationPlacementImageLabel->setProperty(
        "calibrationPlacementImageLoaded",
        true);
}

} // namespace i2r_rlc::ui
