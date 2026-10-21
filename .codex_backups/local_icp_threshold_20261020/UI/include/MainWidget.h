/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        MainWidget.h
 * @brief       主界面 QWidget
 * @details     声明顶层界面组件、用户意图信号和状态刷新接口。
 ******************************************************************************/
#ifndef I2R_RLC_UI_MAIN_WIDGET_H
#define I2R_RLC_UI_MAIN_WIDGET_H

#include <array>

#include <QImage>
#include <QPointF>
#include <QPointer>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include "CameraFrame.h"

class QLabel;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QStackedWidget;

namespace i2r_rlc::ui {

struct TrackingFramePresentation;

/**
 * @brief 应用顶层 QWidget。
 *
 * 本类只负责固定界面组合、状态显示和发出用户意图，不直接操作相机、
 * 机器人、算法或配置文件。
 */
class MainWidget final : public QWidget {
    Q_OBJECT

public:
    explicit MainWidget(QWidget* parent = nullptr);

signals:
    /// @brief 用户请求设置相机曝光时间，单位 us。
    void cameraExposureRequested(int exposureUs);

    /// @brief 用户请求切换旧协议上下文中的激光开关。
    void laserEnabledChanged(bool enabled);

    /// @brief 用户请求启动焊缝跟踪主流程。
    void startWeldTrackingRequested();

    /**
     * @brief 用户选择了在线纠偏所需的轨迹与变位机标定文件。
     */
    void onlineCorrectionConfigurationSelected(
        const QString& workPathFile,
        const QString& teachPoseFile);

    /// @brief 用户请求使用当前工件提交在线纠偏配置。
    void onlineCorrectionConfigurationRequested();

    /**
     * @brief 用户切换在线纠偏起始点定位方式。
     *
     * 索引顺序与 UI 下拉框保持一致：0=InitType，1=GlobalCorrection，2=IBVSStart。
     */
    void onlineCorrectionStartModeChanged(int modeIndex);

    /// @brief 用户热调整在线处理；融合模式为自动/手动/仅 KCF，仿真权重范围 [0, 1]。
    void onlineProcessingOptionsChanged(bool localEnabled, int fusionMode, double simulationWeight);

    /**
     * @brief 用户确认选择当前工件。
     *
     * `profileId` 来自 `config/workpieces/workpiece_selection.txt`，UI 只透传
     * 标识，不解析工件轨迹或 JBI 配置。
     */
    void workpieceSelected(int profileIndex, const QString& profileId);

    /// @brief 用户请求启动人工标定采集。
    void startManualCalibrationCaptureRequested();

    /// @brief 用户请求进入自动标定红框预览。
    void previewAutomaticCalibrationCaptureRequested();

    /// @brief 用户请求采集当前标定板图像。
    void captureCalibrationBoardRequested();

    /// @brief 用户请求删除上一组标定采集样本。
    void deleteLastCalibrationSampleRequested();

    /// @brief 用户请求采集当前激光标定图像。
    void captureCalibrationLaserRequested();

    /// @brief 用户请求启动自动标定采集。
    void startAutomaticCalibrationCaptureRequested();

    /// @brief 用户请求结束标定采集。
    void finishCalibrationCaptureRequested();

    /// @brief 用户从人工/自动标定子页返回模式选择页。
    void calibrationModeSelectionReturned();

    /// @brief 用户收起系统标定主功能面板。
    void calibrationFeaturePanelClosed();

    /// @brief 用户选择了离线纠偏存档文件。
    void offlineCorrectionArchiveSelected(const QString& archiveFilePath);

    /// @brief 用户请求使用当前工件提交离线纠偏配置。
    void offlineCorrectionConfigurationRequested();

    /// @brief 用户请求启动有匹配离线纠偏。
    void startOfflineCorrectionRequested();

    /// @brief 用户收起离线纠偏主功能面板。
    void offlineCorrectionFeaturePanelClosed();

    /// @brief 用户收起在线纠偏主功能面板。
    void onlineCorrectionFeaturePanelClosed();

    /// @brief 用户请求清空临时文件并开始保存示教轨迹记录。
    void startTeachTrajectoryRecordingRequested();

    /// @brief 用户请求保存临时示教轨迹为 `_W/_T` 文件。
    void saveTeachTrajectoryRequested(
        const QString& outputDirectory,
        const QString& baseName);

    /// @brief 用户收起保存示教轨迹主功能面板。
    void teachTrajectoryFeaturePanelClosed();

    /// @brief 用户请求停止当前流程。
    void stopRequested();

    /// @brief 用户请求加载视觉伺服参数。
    void loadIbvsConfigRequested();

    /// @brief 用户请求启动视觉伺服。
    void startIbvsDryRunRequested();

    /// @brief 用户请求执行视觉伺服内部单步；正式界面默认隐藏。
    void stepIbvsDryRunRequested();

    /// @brief 用户请求停止视觉伺服。
    void stopIbvsDryRunRequested();

    /// @brief 用户请求执行局部修正内部入口；正式界面默认隐藏。
    void runLocalCorrectionReplayRequested();

    /// @brief 用户请求执行全局定位。
    void globalLocateRequested();

    /// @brief 用户修改了全局定位参考来源或常驻 RobotSim 端口。
    void globalLocateVirtualLaserConfigurationChanged();

    /// @brief 用户请求清除定位状态。
    void clearLocalizationResultRequested();

    /// @brief 用户请求连接 RobotSim 手动仿真场景，不发送真实关节值。
    void robotSimVirtualReferenceRequested(int port);

    /// @brief 用户请求连接 RobotSim 并持续同步真实机器人和变位机关节值。
    void robotSimRealSceneConnectionRequested(int port);

public slots:
    /**
     * @brief 设置旧式“当前工件”下拉框选项。
     */
    void setWorkpieceChoices(
        const QStringList& displayNames,
        const QStringList& profileIds);

    /// @brief 更新当前工件摘要文本。
    void setCurrentWorkpieceText(const QString& text);

    /// @brief 设置保存示教轨迹默认输出目录。
    void setTeachTrajectoryOutputDirectory(const QString& directoryPath);

    /// @brief 更新保存示教轨迹状态摘要。
    void setTeachTrajectoryStateText(const QString& text);

    /// @brief 在固定图像区域内按比例显示相机帧。
    void showFrame(const i2r_rlc::camera::CameraFrame& frame);

    /// @brief 设置叠加在实时相机图像上的仿真激光参考线。
    void setVirtualLaserOverlay(const QVector<QPointF>& imagePoints);

    /// @brief 设置或清除叠加在实时图像上的 RobotSim 仿真焊缝点。
    void setVirtualWeldPointOverlay(const QPointF& imagePoint, bool visible);

    /**
     * @brief 原子更新 RobotSim 仿真激光线和仿真焊缝点叠加。
     *
     * 两类叠加状态写入完成后只刷新一次图像，避免纠偏逐帧更新时显示中间态。
     */
    void setVirtualReferenceOverlay(
        const QVector<QPointF>& laserLineImagePoints,
        const QPointF& weldPointImage,
        bool weldPointVisible);

    /**
     * @brief 设置或清除 KCF 从真实图像检测到的绿色焊缝点。
     *
     * 用于单独清除叠加；纠偏显示必须使用 showTrackingFrame 一次提交同帧数据。
     */
    void setDetectedWeldPointOverlay(const QPointF& imagePoint, bool visible);

    /// @brief UI 线程一次提交原图和全部检测叠加，仅重绘一次，避免跨帧混合。
    void showTrackingFrame(const TrackingFramePresentation& presentation);

    /// @brief 设置叠加在实时相机图像上的 Steger 真实激光中心线。
    void setRealLaserOverlay(const QVector<QPointF>& imagePoints);

    /// @brief 更新图像占位区文本。
    void setImagePlaceholderText(const QString& text);

    /// @brief 更新机器人状态文本。
    void setRobotStateText(const QString& text);

    /**
     * @brief 更新旧工程右上角等价状态监控块。
     *
     * 文本对应旧 IDC_ST1/ST2/ST4/ST5，设备字段单位分别为设备状态、fps 和 us。
     */
    void setLegacyStatusPanel(
        const QString& modeText,
        const QString& servoText,
        const QString& runText,
        const QString& weldText,
        bool laserEnabled,
        bool filterEnabled,
        const QString& deviceText,
        const QString& fpsText,
        const QString& exposureText);

    /// @brief 同步设备控制区的激光按钮状态，避免信号回环。
    void setLaserControlChecked(bool enabled);

    /// @brief 更新设备控制区的曝光输入值，单位 us。
    void setExposureControlValue(int exposureUs);

    /// @brief 更新系统标定功能面板摘要。
    void setCalibrationStateText(const QString& text);

    /// @brief 更新离线纠偏功能面板摘要。
    void setOfflineCorrectionStateText(const QString& text);
    /// @brief 显示可取消的启动等待；仅更改按钮反馈，不改变机器人执行条件。
    void setCorrectionStartWaiting(bool offline, bool waiting);

    /// @brief 更新流程状态文本。
    void setOnlineCorrectionStateText(const QString& text);
    /// @brief 在 UI 线程同步在线处理选项，不发出用户变更信号。
    void setOnlineProcessingOptions(bool localEnabled, int fusionMode, double simulationWeight);
    /// @brief 更新在线纠偏最终 P001 的独立诊断文本。
    void setOnlineCorrectionP001Text(const QString& text);
    /**
     * @brief 在 UI 线程更新固定 XYZ 数值框；四行依次为 TCP 机器人基/变位机基、
     * KCF 机器人基/变位机基，单位 mm。状态说明保留值是否仍有效。
     */
    void setOnlineCorrectionCoordinates(
        const std::array<QStringList, 4>& rows, const QString& status);

    /**
     * @brief 更新离线纠偏逐帧同步的独立状态、最近确认帧和最近错误。
     */
    void setOfflineCorrectionSyncDetails(
        const QString& stateText,
        const QString& consumedFrameText,
        const QString& errorText);

    /**
     * @brief 更新在线纠偏逐帧同步的独立状态、最近确认帧和最近错误。
     */
    void setOnlineCorrectionSyncDetails(
        const QString& stateText,
        const QString& consumedFrameText,
        const QString& errorText);

    /// @brief 更新视觉流水线状态文本。
    void setVisionStateText(const QString& text);

    /// @brief 更新视觉伺服状态文本。
    void setIbvsStateText(const QString& text);

    /// @brief 更新定位状态文本。
    void setLocalizationStateText(const QString& text);

    /// @brief 更新定位页 RobotSim 连接状态文本。
    void setGlobalLocateRobotSimConnectionText(const QString& text);

    /// @brief 锁定或解锁本次全局定位使用的仿真激光和配准点云来源。
    void setGlobalLocateVirtualLaserSourceLocked(bool locked);

    /**
     * @brief 停止流程后复位当前功能页控件，但保留当前页和 RobotSim 端口文本。
     *
     * 该操作清除图像叠加、恢复功能页首次打开时的选择状态，不发出新的业务请求。
     */
    void resetFeaturePanelsAfterStop();

    /// @brief 更新 RobotSim 联调状态文本。
    void setDigitalTwinStateText(const QString& text);

    /// @brief 返回全局定位仿真激光来源下拉框索引；0=OCCT，1=RobotSim。
    int globalLocateVirtualLaserSourceIndex() const;

    /// @brief 返回配准虚拟点云来源索引；0=weldment，1=RobotSim 激光扫描。
    int globalLocateVirtualPointCloudSourceIndex() const;

    /// @brief 返回常驻 RobotSim 连接区输入的当前端口；无效时为 0。
    int globalLocateRobotSimPort() const;

    /// @brief 返回用户输入的 RobotSim 当前端口；无效时为 0。
    int robotSimPort() const;

    /// @brief 追加日志文本。
    void appendLog(const QString& text);

    /// @brief 显示旧工程风格的信息提示框。
    void showLegacyInfoPrompt(const QString& title, const QString& message);

    /// @brief 显示旧工程风格的警告提示框。
    void showLegacyWarningPrompt(const QString& title, const QString& message);

    /**
     * @brief 显示旧工程风格的确认提示框。
     * @return 用户点击确认时返回 true；测试/offscreen 环境默认返回 true。
     */
    bool confirmLegacyPrompt(const QString& title, const QString& message);

    /// @brief 判断指定 objectName 的功能页是否为当前右下角面板。
    bool isFeaturePageActive(const QString& pageObjectName) const;

private:
    void emitCameraExposureRequested();
    void chooseTeachTrajectoryOutputDirectory();
    void chooseOfflineCorrectionArchive();
    void chooseOnlineCorrectionConfiguration();
    void emitCurrentWorkpieceSelected();
    void emitRobotSimVirtualReferenceRequested();
    void emitRobotSimRealSceneConnectionRequested();
    void showFeaturePanel(int index);
    void showCalibrationModePanel(int index);
    void toggleExclusiveFeaturePanel(int index);
    void clearExclusiveFeaturePanel();
    /// @brief 从实际显示页取得互斥入口，不维护第二份页面状态。
    int activeExclusiveFeaturePanelIndex() const;
    void updateExclusiveEntryButtons();
    void emitExclusiveFeatureClosed(int index);
    void updateDisplayedImage();
    void updateCalibrationPlacementImage();

    QLabel* m_imagePlaceholder = nullptr;
    QLabel* m_robotStateLabel = nullptr;
    QLabel* m_modeStatusLabel = nullptr;
    QLabel* m_servoStatusLabel = nullptr;
    QLabel* m_runStatusLabel = nullptr;
    QLabel* m_weldStatusLabel = nullptr;
    QLabel* m_laserStatusLabel = nullptr;
    QLabel* m_filterStatusLabel = nullptr;
    QLabel* m_deviceStatusLabel = nullptr;
    QLabel* m_exposureStatusLabel = nullptr;
    QLineEdit* m_exposureControlEdit = nullptr;
    QPushButton* m_setExposureButton = nullptr;
    QPushButton* m_laserControlButton = nullptr;
    QLabel* m_calibrationStateLabel = nullptr;
    QLabel* m_offlineCorrectionStateLabel = nullptr;
    QLabel* m_onlineCorrectionStateLabel = nullptr;
    QCheckBox* m_onlineLocalIcpCheckBox = nullptr;
    QComboBox* m_onlineFusionModeComboBox = nullptr;
    QDoubleSpinBox* m_onlineSimulationWeightSpinBox = nullptr;
    QDoubleSpinBox* m_onlineKcfWeightSpinBox = nullptr;
    QLabel* m_onlineCorrectionP001Label = nullptr;
    std::array<std::array<QLineEdit*, 3>, 4> m_onlineCoordinateEdits{};
    QLabel* m_onlineCoordinateStatusLabel = nullptr;
    QLabel* m_offlineCorrectionSyncStateLabel = nullptr;
    QLabel* m_offlineCorrectionConsumedFrameLabel = nullptr;
    QLabel* m_offlineCorrectionSyncErrorLabel = nullptr;
    QLabel* m_onlineCorrectionSyncStateLabel = nullptr;
    QLabel* m_onlineCorrectionConsumedFrameLabel = nullptr;
    QLabel* m_onlineCorrectionSyncErrorLabel = nullptr;
    QLabel* m_visionStateLabel = nullptr;
    QLabel* m_ibvsStateLabel = nullptr;
    QLabel* m_localizationStateLabel = nullptr;
    QLabel* m_digitalTwinStateLabel = nullptr;
    QComboBox* m_workpieceComboBox = nullptr;
    QPushButton* m_selectWorkpieceButton = nullptr;
    QLabel* m_currentWorkpieceLabel = nullptr;
    QComboBox* m_onlineStartModeComboBox = nullptr;
    QPushButton* m_selectOnlineCorrectionConfigButton = nullptr;
    QPushButton* m_startTrackingButton = nullptr;
    QPushButton* m_startManualCalibrationButton = nullptr;
    QPushButton* m_captureCalibrationBoardButton = nullptr;
    QPushButton* m_deleteLastCalibrationButton = nullptr;
    QPushButton* m_captureCalibrationLaserButton = nullptr;
    QPushButton* m_startAutomaticCalibrationButton = nullptr;
    QPushButton* m_finishCalibrationButton = nullptr;
    QPushButton* m_selectOfflineCorrectionArchiveButton = nullptr;
    QPushButton* m_startOfflineCorrectionButton = nullptr;
    QPushButton* m_loadIbvsConfigButton = nullptr;
    QPushButton* m_startIbvsDryRunButton = nullptr;
    QPushButton* m_stepIbvsDryRunButton = nullptr;
    QPushButton* m_stopIbvsDryRunButton = nullptr;
    QPushButton* m_localReplayButton = nullptr;
    QPushButton* m_globalIcpDryRunButton = nullptr;
    QPushButton* m_clearLocalizationButton = nullptr;
    QComboBox* m_globalLocateVirtualLaserSourceComboBox = nullptr;
    QComboBox* m_globalLocateVirtualPointCloudSourceComboBox = nullptr;
    QLabel* m_globalLocateRobotSimConnectionLabel = nullptr;
    QLineEdit* m_robotSimPortLineEdit = nullptr;
    QPushButton* m_robotSimQueryButton = nullptr;
    QPushButton* m_robotSimRealSceneButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_entryCalibrationButton = nullptr;
    QPushButton* m_entryOfflineCorrectionButton = nullptr;
    QPushButton* m_entryOnlineCorrectionButton = nullptr;
    QPushButton* m_entryTeachTrajectoryButton = nullptr;
    QPushButton* m_entryIbvsButton = nullptr;
    QPushButton* m_entryLocalizationButton = nullptr;
    QPushButton* m_calibrationManualModeButton = nullptr;
    QPushButton* m_calibrationAutomaticModeButton = nullptr;
    QPushButton* m_confirmAutomaticCalibrationButton = nullptr;
    QLabel* m_calibrationPlacementImageLabel = nullptr;
    QLineEdit* m_teachTrajectoryOutputDirectoryEdit = nullptr;
    QLineEdit* m_teachTrajectoryBaseNameEdit = nullptr;
    QPushButton* m_chooseTeachTrajectoryOutputDirectoryButton = nullptr;
    QPushButton* m_startTeachTrajectoryRecordingButton = nullptr;
    QPushButton* m_saveTeachTrajectoryButton = nullptr;
    QPointer<QStackedWidget> m_featureStack;
    QStackedWidget* m_calibrationStack = nullptr;
    QPointer<QPlainTextEdit> m_logView;
    QStringList m_logLines;
    QImage m_currentImage;
    QVector<QPointF> m_virtualLaserOverlayPoints;
    QPointF m_virtualWeldPointOverlay;
    bool m_virtualWeldPointOverlayVisible = false;
    QPointF m_detectedWeldPointOverlay;
    bool m_detectedWeldPointOverlayVisible = false;
    QString m_trackingFrameCaption;
    QVector<QPointF> m_realLaserOverlayPoints;
    bool m_legacyPromptActive = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_MAIN_WIDGET_H
