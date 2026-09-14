/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_main_widget_panels.cpp
 * @brief       main widget panels 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdlib>
#include <iostream>

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QStringList>

#include "MainWidget.h"
#include "UiController.h"

namespace {

bool expectTrue(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

QPushButton* findButton(i2r_rlc::ui::MainWidget &widget, const char *name)
{
    return widget.findChild<QPushButton*>(QString::fromLatin1(name));
}

QComboBox* findComboBox(i2r_rlc::ui::MainWidget &widget, const char *name)
{
    return widget.findChild<QComboBox*>(QString::fromLatin1(name));
}

QLineEdit* findLineEdit(i2r_rlc::ui::MainWidget &widget, const char *name)
{
    return widget.findChild<QLineEdit*>(QString::fromLatin1(name));
}

QLabel* findLabel(i2r_rlc::ui::MainWidget &widget, const char *name)
{
    return widget.findChild<QLabel*>(QString::fromLatin1(name));
}

bool currentPageHasName(QStackedWidget *stack, const char *name)
{
    return stack != nullptr && stack->currentWidget() != nullptr
        && stack->currentWidget()->objectName() == QString::fromLatin1(name);
}

bool testLegacyStatusAndDeviceControlPanel()
{
    i2r_rlc::ui::MainWidget widget;
    auto *statusGroup = widget.findChild<QGroupBox*>(QStringLiteral("legacyStatusGroup"));
    auto *deviceControlGroup = widget.findChild<QGroupBox*>(QStringLiteral("deviceControlGroup"));
    auto *exposureEdit = widget.findChild<QLineEdit*>(QStringLiteral("exposureControlEdit"));
    auto *setExposureButton = findButton(widget, "setExposureButton");
    auto *laserButton = findButton(widget, "laserControlButton");

    int exposureUs = 0;
    int exposureSignalCount = 0;
    bool lastLaserEnabled = true;
    int laserSignalCount = 0;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::cameraExposureRequested,
        [&exposureUs, &exposureSignalCount](int value) {
            exposureUs = value;
            ++exposureSignalCount;
        });
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::laserEnabledChanged,
        [&lastLaserEnabled, &laserSignalCount](bool enabled) {
            lastLaserEnabled = enabled;
            ++laserSignalCount;
        });

    widget.setLegacyStatusPanel(
        QStringLiteral("PLAY"),
        QStringLiteral("SERVO ON"),
        QStringLiteral("ROBOT EXE!"),
        QStringLiteral("WELDING"),
        true,
        false,
        QStringLiteral("机器人已连接"),
        QStringLiteral("30.00"),
        QStringLiteral("2000"));

    bool ok = true;
    ok &= expectTrue(statusGroup != nullptr,
                     "legacy status group should exist");
    ok &= expectTrue(deviceControlGroup != nullptr,
                     "device control group should exist");
    ok &= expectTrue(findLabel(widget, "onlineCorrectionStateLabel") != nullptr
                         && findLabel(widget, "onlineCorrectionStateLabel")->parent()
                                != statusGroup,
                     "online correction status label should not be inside status monitor");
    ok &= expectTrue(findLabel(widget, "visionStateLabel") != nullptr
                         && findLabel(widget, "visionStateLabel")->parent()
                                != statusGroup,
                     "vision status label should not be inside status monitor");
    ok &= expectTrue(findLabel(widget, "modeStatusLabel")->text() == QStringLiteral("PLAY"),
                     "mode tile should show legacy mode text");
    ok &= expectTrue(findLabel(widget, "servoStatusLabel")->text() == QStringLiteral("SERVO ON"),
                     "servo tile should show legacy servo text");
    ok &= expectTrue(findLabel(widget, "laserStatusLabel")->text().contains(QStringLiteral("ON")),
                     "laser status tile should reflect enabled state");
    ok &= expectTrue(findLabel(widget, "filterStatusLabel")->text().contains(QStringLiteral("OFF")),
                     "filter status tile should reflect disabled state");
    ok &= expectTrue(findLabel(widget, "fpsStatusLabel") == nullptr,
                     "legacy status monitor should not show FPS");
    ok &= expectTrue(findLabel(widget, "exposureStatusLabel")->text().contains(QStringLiteral("2000")),
                     "exposure status label should be updateable");
    ok &= expectTrue(findLabel(widget, "deviceStatusLabel")->text()
                         .contains(QStringLiteral("机器人："))
                         && findLabel(widget, "deviceStatusLabel")->text()
                         .contains(QStringLiteral("机器人已连接")),
                     "robot status should occupy the device row with robot wording");
    bool hasFilterNoControlLabel = false;
    for (auto *label : widget.findChildren<QLabel*>()) {
        hasFilterNoControlLabel |= label->text().contains(QStringLiteral("滤光片：不控制"));
    }
    ok &= expectTrue(!hasFilterNoControlLabel,
                     "device control panel should not show the filter no-control hint");

    widget.setExposureControlValue(4321);
    setExposureButton->click();
    ok &= expectTrue(exposureUs == 4321 && exposureSignalCount == 1,
                     "set exposure button should emit exposure request");

    laserButton->click();
    ok &= expectTrue(!lastLaserEnabled && laserSignalCount == 1,
                     "laser control button should emit disabled state");
    widget.setLaserControlChecked(true);
    ok &= expectTrue(laserSignalCount == 1,
                     "programmatic laser sync should not emit toggled signal");
    return ok;
}

bool testFeaturePanelSwitching()
{
    i2r_rlc::ui::MainWidget widget;
    widget.show();
    QCoreApplication::processEvents();
    auto *featureStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("featureStack"));
    auto *calibrationButton = findButton(widget, "entryCalibrationButton");
    auto *offlineButton = findButton(widget, "entryOfflineCorrectionButton");
    auto *onlineButton = findButton(widget, "entryOnlineCorrectionButton");
    auto *teachButton = findButton(widget, "entryTeachTrajectoryButton");

    int calibrationClosedCount = 0;
    int offlineClosedCount = 0;
    int onlineClosedCount = 0;
    int teachClosedCount = 0;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::calibrationFeaturePanelClosed,
        [&calibrationClosedCount]() { ++calibrationClosedCount; });
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::offlineCorrectionFeaturePanelClosed,
        [&offlineClosedCount]() { ++offlineClosedCount; });
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::onlineCorrectionFeaturePanelClosed,
        [&onlineClosedCount]() { ++onlineClosedCount; });
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::teachTrajectoryFeaturePanelClosed,
        [&teachClosedCount]() { ++teachClosedCount; });

    bool ok = true;
    ok &= expectTrue(featureStack != nullptr, "feature stack should exist");
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "feature stack should start on default page");
    ok &= expectTrue(widget.findChild<QWidget*>(QStringLiteral("cameraRobotPage")) == nullptr,
                     "camera/robot feature page should not exist");
    ok &= expectTrue(findButton(widget, "entryCameraRobotButton") == nullptr,
                     "camera/robot entry button should not exist");
    ok &= expectTrue(findButton(widget, "selectOfflineDirectoryButton") == nullptr
                         && findButton(widget, "nextFrameButton") == nullptr
                         && findButton(widget, "resetOfflineSequenceButton") == nullptr
                         && findButton(widget, "startCameraButton") == nullptr
                         && findButton(widget, "connectRobotButton") == nullptr,
                     "offline image and manual connect buttons should not be exposed in main UI");

    calibrationButton->click();
    ok &= expectTrue(currentPageHasName(featureStack, "calibrationPage"),
                     "calibration entry should show calibration page");
    ok &= expectTrue(calibrationButton->isChecked(),
                     "calibration entry should be checked while active");
    ok &= expectTrue(!offlineButton->isEnabled() && !onlineButton->isEnabled()
                         && !teachButton->isEnabled(),
                     "offline, online and teach entries should be locked while calibration is active");

    offlineButton->click();
    ok &= expectTrue(currentPageHasName(featureStack, "calibrationPage"),
                     "disabled offline entry should not switch away from calibration");

    calibrationButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "clicking active calibration entry again should clear the panel");
    ok &= expectTrue(calibrationClosedCount == 1,
                     "closing calibration panel should emit release signal");
    ok &= expectTrue(offlineButton->isEnabled() && onlineButton->isEnabled()
                         && teachButton->isEnabled(),
                     "offline, online and teach entries should unlock after calibration closes");

    offlineButton->click();
    ok &= expectTrue(currentPageHasName(featureStack, "offlineCorrectionPage"),
                     "offline entry should show offline correction page");
    ok &= expectTrue(!calibrationButton->isEnabled() && !onlineButton->isEnabled()
                         && !teachButton->isEnabled(),
                     "calibration, online and teach entries should be locked while offline correction is active");

    offlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "clicking active offline entry again should clear the panel");
    ok &= expectTrue(offlineClosedCount == 1,
                     "closing offline panel should emit release signal");

    onlineButton->click();
    ok &= expectTrue(currentPageHasName(featureStack, "onlineCorrectionPage"),
                     "online entry should show online correction page");
    ok &= expectTrue(!calibrationButton->isEnabled() && !offlineButton->isEnabled()
                         && !teachButton->isEnabled(),
                     "calibration, offline and teach entries should be locked while online correction is active");

    onlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "clicking active online entry again should clear the panel");
    ok &= expectTrue(onlineClosedCount == 1,
                     "closing online panel should emit release signal");

    teachButton->click();
    ok &= expectTrue(currentPageHasName(featureStack, "teachTrajectoryPage"),
                     "teach trajectory entry should show save teach trajectory page");
    ok &= expectTrue(!calibrationButton->isEnabled()
                         && !offlineButton->isEnabled()
                         && !onlineButton->isEnabled(),
                     "calibration/offline/online entries should be locked while teach trajectory is active");

    teachButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "clicking active teach trajectory entry again should clear the panel");
    ok &= expectTrue(teachClosedCount == 1,
                     "closing teach trajectory panel should emit release signal");

    findButton(widget, "entryIbvsButton")->click();
    ok &= expectTrue(currentPageHasName(featureStack, "ibvsPage"),
                     "visual servo entry should show visual servo page");

    findButton(widget, "entryLocalizationButton")->click();
    ok &= expectTrue(currentPageHasName(featureStack, "localizationPage"),
                     "localization entry should show localization page");

    ok &= expectTrue(
        findButton(widget, "entryRobotSimButton") == nullptr,
        "RobotSim should remain in the persistent connection area, not a feature page");

    return ok;
}

bool testAutomaticCalibrationConfirmationGate()
{
    i2r_rlc::ui::MainWidget widget;
    auto *featureStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("featureStack"));
    auto *calibrationStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("calibrationStack"));

    int previewCount = 0;
    int startCount = 0;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::previewAutomaticCalibrationCaptureRequested,
        [&previewCount]() { ++previewCount; });
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::startAutomaticCalibrationCaptureRequested,
        [&startCount]() { ++startCount; });

    bool ok = true;
    findButton(widget, "entryCalibrationButton")->click();
    ok &= expectTrue(currentPageHasName(featureStack, "calibrationPage"),
                     "calibration entry should show calibration page");
    ok &= expectTrue(currentPageHasName(calibrationStack, "calibrationSelectPage"),
                     "calibration page should start from mode selection");

    findButton(widget, "calibrationAutomaticModeButton")->click();
    ok &= expectTrue(previewCount == 1,
                     "automatic mode should request red-box preview");
    ok &= expectTrue(startCount == 0,
                     "automatic mode should not start capture before confirmation");
    ok &= expectTrue(currentPageHasName(calibrationStack, "calibrationAutomaticPage"),
                     "automatic mode should show confirmation page");

    findButton(widget, "confirmAutomaticCalibrationButton")->click();
    ok &= expectTrue(startCount == 1,
                     "confirm should start automatic capture");
    return ok;
}

bool testFeaturePanelSwitchingWithUiController()
{
    i2r_rlc::ui::MainWidget widget;
    i2r_rlc::ui::UiController controller(widget);
    controller.initialize();
    widget.show();
    QCoreApplication::processEvents();

    auto *featureStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("featureStack"));
    auto *calibrationButton = findButton(widget, "entryCalibrationButton");
    auto *offlineButton = findButton(widget, "entryOfflineCorrectionButton");
    auto *onlineButton = findButton(widget, "entryOnlineCorrectionButton");
    auto *teachButton = findButton(widget, "entryTeachTrajectoryButton");

    bool ok = true;
    ok &= expectTrue(featureStack != nullptr,
                     "feature stack should exist with UI controller");
    ok &= expectTrue(calibrationButton != nullptr
                         && offlineButton != nullptr
                         && onlineButton != nullptr
                         && teachButton != nullptr,
                     "exclusive feature buttons should exist with UI controller");

    calibrationButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "calibrationPage"),
                     "controller-bound calibration click should show panel");
    calibrationButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "controller-bound calibration second click should clear panel");

    offlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "offlineCorrectionPage"),
                     "controller-bound offline click should show panel");
    offlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "controller-bound offline second click should clear panel");

    onlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "onlineCorrectionPage"),
                     "controller-bound online click should show panel");
    onlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "controller-bound online second click should clear panel");

    teachButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "teachTrajectoryPage"),
                     "controller-bound teach trajectory click should show panel");
    teachButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "defaultFeaturePage"),
                     "controller-bound teach trajectory second click should clear panel");

    calibrationButton->click();
    QCoreApplication::processEvents();
    findButton(widget, "entryIbvsButton")->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "ibvsPage"),
                     "visual servo entry should safely close calibration panel first");

    offlineButton->click();
    QCoreApplication::processEvents();
    findButton(widget, "entryLocalizationButton")->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(currentPageHasName(featureStack, "localizationPage"),
                     "localization entry should safely close offline panel first");

    onlineButton->click();
    QCoreApplication::processEvents();
    ok &= expectTrue(
        findButton(widget, "entryRobotSimButton") == nullptr
            && currentPageHasName(featureStack, "onlineCorrectionPage"),
        "persistent RobotSim connection area must not replace the active correction page");
    onlineButton->click();
    QCoreApplication::processEvents();

    controller.shutdown();
    QCoreApplication::processEvents();
    return ok;
}

bool testManualCalibrationPlacementImage()
{
    i2r_rlc::ui::MainWidget widget;
    auto *featureStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("featureStack"));
    auto *calibrationStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("calibrationStack"));

    bool ok = true;
    findButton(widget, "entryCalibrationButton")->click();
    ok &= expectTrue(currentPageHasName(featureStack, "calibrationPage"),
                     "calibration entry should show calibration page");

    auto *placementImageLabel = findLabel(widget, "calibrationPlacementImageLabel");
    ok &= expectTrue(placementImageLabel != nullptr,
                     "calibration page should contain persistent placement image label");
    ok &= expectTrue(placementImageLabel != nullptr
                         && placementImageLabel
                                ->property("calibrationPlacementImageLoaded")
                                .toBool(),
                     "calibration placement image should load immediately after opening page");

    findButton(widget, "calibrationManualModeButton")->click();
    ok &= expectTrue(currentPageHasName(calibrationStack, "calibrationManualPage"),
                     "manual calibration should show manual page");
    ok &= expectTrue(placementImageLabel != nullptr && !placementImageLabel->isHidden(),
                     "manual calibration should keep placement image visible");

    widget.setCalibrationStateText(
        QStringLiteral("标定：手动采集中\nboard=1/19, laser=0/4\n输出：Data/CALID/4 TWO_STEP"));
    findButton(widget, "calibrationManualBackButton")->click();
    auto *calibrationStateLabel = findLabel(widget, "calibrationStateLabel");
    ok &= expectTrue(currentPageHasName(calibrationStack, "calibrationSelectPage"),
                     "manual calibration back button should return to mode selection");
    ok &= expectTrue(calibrationStateLabel != nullptr
                         && !calibrationStateLabel->text().contains(QStringLiteral("输出："))
                         && !calibrationStateLabel->text().contains(QStringLiteral("CALID"))
                         && !calibrationStateLabel->text().contains(QStringLiteral("TWO_STEP")),
                     "manual calibration back button should clear output directory residue");

    findButton(widget, "calibrationAutomaticModeButton")->click();
    ok &= expectTrue(currentPageHasName(calibrationStack, "calibrationAutomaticPage"),
                     "automatic calibration should show automatic page");
    ok &= expectTrue(placementImageLabel != nullptr
                         && placementImageLabel
                                ->property("calibrationPlacementImageLoaded")
                                .toBool(),
                     "automatic calibration should keep placement image loaded");
    widget.setCalibrationStateText(
        QStringLiteral("标定：自动红框预览\nboard=0/19, laser=0/4\n输出：Data/CALID/4 TWO_STEP"));
    findButton(widget, "calibrationAutomaticBackButton")->click();
    ok &= expectTrue(currentPageHasName(calibrationStack, "calibrationSelectPage"),
                     "automatic calibration back button should return to mode selection");
    ok &= expectTrue(calibrationStateLabel != nullptr
                         && !calibrationStateLabel->text().contains(QStringLiteral("输出："))
                         && !calibrationStateLabel->text().contains(QStringLiteral("CALID"))
                         && !calibrationStateLabel->text().contains(QStringLiteral("TWO_STEP")),
                     "automatic calibration back button should clear output directory residue");
    ok &= expectTrue(placementImageLabel != nullptr && !placementImageLabel->isHidden(),
                     "calibration placement image should remain visible after returning to selection");
    return ok;
}

bool testOnlineCorrectionStartModeComboBox()
{
    i2r_rlc::ui::MainWidget widget;
    auto *comboBox = findComboBox(widget, "onlineStartModeComboBox");

    int lastModeIndex = -1;
    int changeCount = 0;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::onlineCorrectionStartModeChanged,
        [&lastModeIndex, &changeCount](int modeIndex) {
            lastModeIndex = modeIndex;
            ++changeCount;
        });

    bool ok = true;
    ok &= expectTrue(comboBox != nullptr,
                     "online start mode combo box should exist");
    ok &= expectTrue(comboBox != nullptr && comboBox->count() == 3,
                     "online start mode combo box should expose three modes");
    ok &= expectTrue(comboBox != nullptr && comboBox->currentIndex() == 0,
                     "online start mode combo box should default to InitType");

    comboBox->setCurrentIndex(1);
    ok &= expectTrue(lastModeIndex == 1,
                     "selecting GlobalCorrection should emit index 1");

    comboBox->setCurrentIndex(2);
    ok &= expectTrue(lastModeIndex == 2,
                     "selecting IBVSStart should emit index 2");
    ok &= expectTrue(changeCount == 2,
                     "online start mode combo box should emit once per change");
    return ok;
}

bool testWorkpieceSelectorPanel()
{
    i2r_rlc::ui::MainWidget widget;
    auto *workpieceGroup = widget.findChild<QGroupBox*>(QStringLiteral("workpieceGroup"));
    auto *comboBox = findComboBox(widget, "workpieceComboBox");
    auto *selectButton = findButton(widget, "selectWorkpieceButton");
    auto *label = widget.findChild<QLabel*>(QStringLiteral("currentWorkpieceLabel"));

    int selectedIndex = -1;
    QString selectedId;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::workpieceSelected,
        [&selectedIndex, &selectedId](int index, const QString& id) {
            selectedIndex = index;
            selectedId = id;
        });

    widget.setWorkpieceChoices(
        QStringList{
            QStringLiteral("喷嘴"),
            QStringLiteral("出风口小（正20mm）"),
            QStringLiteral("中集平板工件")
        },
        QStringList{
            QStringLiteral("penzui"),
            QStringLiteral("chufengkou_zheng20"),
            QStringLiteral("zhongji_plane")
        });
    widget.setCurrentWorkpieceText(QStringLiteral("已选择：出风口小（正20mm）"));

    bool ok = true;
    ok &= expectTrue(workpieceGroup != nullptr,
                     "workpiece selector group should exist as a persistent right-side panel");
    ok &= expectTrue(comboBox != nullptr && comboBox->count() == 1,
                     "workpiece combo box should only show the selected outlet profile");
    ok &= expectTrue(comboBox != nullptr && comboBox->currentIndex() == 0,
                     "workpiece combo box should default to first profile");
    ok &= expectTrue(comboBox != nullptr
                         && comboBox->itemText(0) == QStringLiteral("出风口小（正20mm）"),
                     "workpiece combo box should show outlet small normal 20mm");
    ok &= expectTrue(comboBox != nullptr
                         && comboBox->itemData(0).toString() == QStringLiteral("chufengkou_zheng20"),
                     "workpiece combo box should keep outlet small normal 20mm id");
    ok &= expectTrue(label != nullptr && label->text().contains(QStringLiteral("出风口小")),
                     "current workpiece label should be updateable");

    selectButton->click();
    ok &= expectTrue(selectedIndex == 0,
                     "select workpiece button should emit selected combo index");
    ok &= expectTrue(selectedId == QStringLiteral("chufengkou_zheng20"),
                     "select workpiece button should emit selected profile id");
    return ok;
}

bool testIbvsAndLocalizationPanelsUseTwinButtonDesign()
{
    i2r_rlc::ui::MainWidget widget;
    auto *featureStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("featureStack"));

    findButton(widget, "entryIbvsButton")->click();

    bool ok = true;
    ok &= expectTrue(findButton(widget, "entryIbvsButton")->text()
                         == QStringLiteral("视觉伺服"),
                     "visual servo entry should be separated from localization");
    ok &= expectTrue(findButton(widget, "entryLocalizationButton")->text()
                         == QStringLiteral("定位"),
                     "localization entry should be separated from visual servo");
    ok &= expectTrue(currentPageHasName(featureStack, "ibvsPage"),
                     "visual servo entry should show visual servo page");
    ok &= expectTrue(findButton(widget, "loadIbvsConfigButton")->text()
                         == QStringLiteral("视觉伺服参数"),
                     "visual servo config button should use Twin text");
    ok &= expectTrue(findButton(widget, "startIbvsDryRunButton")->text()
                         == QStringLiteral("启动视觉伺服"),
                     "visual servo start button should use Twin text");
    ok &= expectTrue(findButton(widget, "stopIbvsDryRunButton")->text()
                         == QStringLiteral("停止视觉伺服"),
                     "visual servo stop button should use Twin text");
    ok &= expectTrue(findButton(widget, "stepIbvsDryRunButton")->isHidden(),
                     "visual servo internal step button should be hidden by default");

    findButton(widget, "entryLocalizationButton")->click();
    ok &= expectTrue(currentPageHasName(featureStack, "localizationPage"),
                     "localization entry should show localization page");
    ok &= expectTrue(findButton(widget, "globalIcpDryRunButton")->text()
                         == QStringLiteral("全局定位"),
                     "global localization button should use Twin text");
    ok &= expectTrue(findButton(widget, "clearLocalizationButton")->text()
                         == QStringLiteral("清除定位状态"),
                     "clear localization button should use Twin text");
    ok &= expectTrue(findButton(widget, "localReplayButton")->isHidden(),
                     "local replay internal button should be hidden by default");
    return ok;
}

bool testTeachTrajectoryPanelSignals()
{
    i2r_rlc::ui::MainWidget widget;
    auto *featureStack = widget.findChild<QStackedWidget*>(
        QStringLiteral("featureStack"));
    auto *outputEdit = widget.findChild<QLineEdit*>(
        QStringLiteral("teachTrajectoryOutputDirectoryEdit"));
    auto *baseNameEdit = widget.findChild<QLineEdit*>(
        QStringLiteral("teachTrajectoryBaseNameEdit"));
    auto *stateLabel = widget.findChild<QLabel*>(
        QStringLiteral("teachTrajectoryStateLabel"));
    auto *startButton = findButton(widget, "startTeachTrajectoryRecordingButton");
    auto *saveButton = findButton(widget, "saveTeachTrajectoryButton");

    int startCount = 0;
    QString savedDirectory;
    QString savedBaseName;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::startTeachTrajectoryRecordingRequested,
        [&startCount]() { ++startCount; });
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::saveTeachTrajectoryRequested,
        [&savedDirectory, &savedBaseName](const QString& directory, const QString& baseName) {
            savedDirectory = directory;
            savedBaseName = baseName;
        });

    bool ok = true;
    findButton(widget, "entryTeachTrajectoryButton")->click();
    ok &= expectTrue(currentPageHasName(featureStack, "teachTrajectoryPage"),
                     "teach trajectory entry should show panel");
    ok &= expectTrue(outputEdit != nullptr && baseNameEdit != nullptr,
                     "teach trajectory panel should expose output dir and base name fields");
    ok &= expectTrue(startButton != nullptr
                         && startButton->text() == QStringLiteral("开始"),
                     "teach trajectory start button should use compact text");
    ok &= expectTrue(saveButton != nullptr
                         && saveButton->text() == QStringLiteral("保存到文件"),
                     "teach trajectory save button should use compact text");
    bool hasShortHint = false;
    bool hasLegacyLongHint = false;
    for (auto *label : widget.findChildren<QLabel*>()) {
        hasShortHint |= label->text().contains(
            QStringLiteral("先选择输出目录和名称。点击开始后运行机器人程序。最后保存到文件"));
        hasLegacyLongHint |= label->text().contains(QStringLiteral("机器人程序 savetraj 信号会自动记录临时轨迹"));
        hasLegacyLongHint |= label->text().contains(QStringLiteral("先清空临时轨迹"));
    }
    ok &= expectTrue(hasShortHint,
                     "teach trajectory panel should show compact operation hint");
    ok &= expectTrue(!hasLegacyLongHint,
                     "teach trajectory panel should not show the old long hint");
    ok &= expectTrue(stateLabel != nullptr && stateLabel->isHidden(),
                     "teach trajectory detailed state label should be hidden from the compact panel");
    widget.setTeachTrajectoryOutputDirectory(QStringLiteral("C:/tmp/workpiece"));
    widget.setTeachTrajectoryStateText(QStringLiteral("保存示教轨迹：测试状态"));
    ok &= expectTrue(outputEdit != nullptr && outputEdit->text() == QStringLiteral("C:/tmp/workpiece"),
                     "teach trajectory output directory should be settable");
    ok &= expectTrue(stateLabel != nullptr
                         && stateLabel->text().contains(QStringLiteral("测试状态")),
                     "teach trajectory hidden state label should remain updateable for compatibility");

    startButton->click();
    ok &= expectTrue(startCount == 1,
                     "start recording button should emit start request");
    baseNameEdit->setText(QStringLiteral("TESTTRAJ"));
    saveButton->click();
    ok &= expectTrue(savedDirectory == QStringLiteral("C:/tmp/workpiece"),
                     "save signal should include output directory");
    ok &= expectTrue(savedBaseName == QStringLiteral("TESTTRAJ"),
                     "save signal should include base name");
    return ok;
}

bool testRobotSimJointTestPanel()
{
    i2r_rlc::ui::MainWidget widget;
    auto *entryButton = findButton(widget, "entryRobotSimButton");
    auto *portEdit = findLineEdit(widget, "robotSimPortLineEdit");
    auto *queryButton = findButton(widget, "robotSimQueryButton");
    auto *stateLabel = findLabel(widget, "digitalTwinStateLabel");

    int requestedPort = 0;
    int requestCount = 0;
    QObject::connect(
        &widget,
        &i2r_rlc::ui::MainWidget::robotSimVirtualReferenceRequested,
        [&requestedPort, &requestCount](int port) {
            requestedPort = port;
            ++requestCount;
        });

    bool ok = true;
    ok &= expectTrue(entryButton == nullptr,
                     "persistent RobotSim connection should not require an entry button");
    ok &= expectTrue(portEdit != nullptr,
                     "robot sim port input should exist");
    ok &= expectTrue(queryButton != nullptr,
                     "robot sim query button should exist");
    ok &= expectTrue(stateLabel != nullptr,
                     "robot sim state label should exist");
    if (portEdit != nullptr && queryButton != nullptr) {
        portEdit->setText(QStringLiteral("61541"));
        queryButton->click();
    }
    ok &= expectTrue(requestCount == 1,
                     "robot sim query should emit one request");
    ok &= expectTrue(requestedPort == 61541,
                     "robot sim query should emit the entered port");
    return ok;
}

bool testAppendLogSmoke()
{
    i2r_rlc::ui::MainWidget widget;
    widget.appendLog(QStringLiteral("log smoke test"));
    QCoreApplication::processEvents();

    return expectTrue(true, "appendLog smoke test should not crash");
}

bool testCorrectionSyncThreeKeyPresentation()
{
    using i2r_rlc::ui::CorrectionPurpose;
    using i2r_rlc::ui::CorrectionSyncState;
    using i2r_rlc::ui::CorrectionSyncUpdate;

    i2r_rlc::ui::MainWidget widget;
    i2r_rlc::ui::UiController controller(widget);
    auto *offlineState = findLabel(widget, "offlineCorrectionSyncStateLabel");
    auto *offlineFrame = findLabel(widget, "offlineCorrectionConsumedFrameLabel");
    auto *offlineError = findLabel(widget, "offlineCorrectionSyncErrorLabel");
    auto *onlineFrame = findLabel(widget, "onlineCorrectionConsumedFrameLabel");

    const auto apply = [&controller](const CorrectionSyncUpdate &update) {
        return QMetaObject::invokeMethod(
            &controller,
            "handleCorrectionSyncUpdate",
            Qt::DirectConnection,
            Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, update));
    };
    const auto observationReady = [](quint64 sessionId, qint64 frameId) {
        CorrectionSyncUpdate update;
        update.state = CorrectionSyncState::ObservationReady;
        update.purpose = CorrectionPurpose::OfflineCorrection;
        update.sessionId = sessionId;
        update.frameId = frameId;
        update.hasObservation = true;
        update.observation.valid = true;
        update.observation.frameId = frameId;
        return update;
    };
    const auto consumed = [](quint64 sessionId, qint64 frameId) {
        CorrectionSyncUpdate update;
        update.state = CorrectionSyncState::VisionConsumed;
        update.purpose = CorrectionPurpose::OfflineCorrection;
        update.sessionId = sessionId;
        update.frameId = frameId;
        return update;
    };

    bool ok = true;
    ok &= expectTrue(
        offlineState != nullptr && offlineFrame != nullptr
            && offlineError != nullptr && onlineFrame != nullptr,
        "online/offline pages should expose independent sync, frame and error labels");
    if (offlineState == nullptr || offlineFrame == nullptr
        || offlineError == nullptr || onlineFrame == nullptr) {
        controller.shutdown();
        return false;
    }
    ok &= expectTrue(apply(observationReady(1, 10)),
                     "correction sync slot should be invokable through Qt metadata");
    apply(consumed(2, 10));
    apply(consumed(1, 11));
    ok &= expectTrue(
        offlineFrame->text().contains(QStringLiteral("无")),
        "wrong session or frame must not confirm a pending observation");
    apply(consumed(1, 10));
    ok &= expectTrue(
        offlineFrame->text().contains(QStringLiteral("10")),
        "an exact session/purpose/frame match should confirm consumption");

    apply(observationReady(1, 11));
    CorrectionSyncUpdate failed;
    failed.state = CorrectionSyncState::Failed;
    failed.purpose = CorrectionPurpose::OfflineCorrection;
    failed.sessionId = 1;
    failed.frameId = 11;
    failed.message = QStringLiteral("同步失败样例");
    apply(failed);
    apply(consumed(1, 11));
    ok &= expectTrue(
        offlineFrame->text().contains(QStringLiteral("10"))
            && offlineError->text().contains(QStringLiteral("同步失败样例")),
        "failure should clear pending without replacing the last confirmed frame");

    apply(observationReady(1, 12));
    CorrectionSyncUpdate replacement;
    replacement.state = CorrectionSyncState::Idle;
    replacement.purpose = CorrectionPurpose::OfflineCorrection;
    replacement.sessionId = 2;
    apply(replacement);
    apply(consumed(1, 12));
    ok &= expectTrue(
        offlineFrame->text().contains(QStringLiteral("无")),
        "replacement session should clear prior pending and confirmed display state");

    apply(observationReady(2, 13));
    CorrectionSyncUpdate cleared;
    cleared.state = CorrectionSyncState::Cleared;
    cleared.purpose = CorrectionPurpose::OfflineCorrection;
    cleared.sessionId = 2;
    apply(cleared);
    apply(consumed(2, 13));
    ok &= expectTrue(
        offlineFrame->text().contains(QStringLiteral("无")),
        "stop/clear should prevent a late Vision result from confirming the frame");

    apply(observationReady(3, 14));
    apply(consumed(3, 14));
    CorrectionSyncUpdate terminal;
    terminal.state = CorrectionSyncState::StoppedAfterFailures;
    terminal.purpose = CorrectionPurpose::OfflineCorrection;
    terminal.sessionId = 3;
    terminal.frameId = 17;
    terminal.consecutiveSyncFailures = 3;
    terminal.message = QStringLiteral("RobotSim 连续 3 帧同步失败，离线纠偏已完整停止");
    apply(terminal);
    ok &= expectTrue(
        offlineState->text().contains(QStringLiteral("连续 3 帧失败"))
            && offlineError->text().contains(QStringLiteral("完整停止")),
        "three consecutive failures should use the explicit terminal wording");

    CorrectionSyncUpdate onlineReady = observationReady(20, 21);
    onlineReady.purpose = CorrectionPurpose::OnlineCorrection;
    CorrectionSyncUpdate onlineConsumed = consumed(20, 21);
    onlineConsumed.purpose = CorrectionPurpose::OnlineCorrection;
    apply(onlineReady);
    apply(onlineConsumed);
    ok &= expectTrue(
        onlineFrame->text().contains(QStringLiteral("21"))
            && offlineFrame->text().contains(QStringLiteral("14")),
        "online and offline recently consumed frames should remain independent");

    CorrectionSyncUpdate unrelated;
    unrelated.state = CorrectionSyncState::ObservationReady;
    unrelated.purpose = CorrectionPurpose::None;
    unrelated.sessionId = 99;
    unrelated.frameId = 99;
    unrelated.hasObservation = true;
    unrelated.observation.valid = true;
    unrelated.observation.frameId = 99;
    apply(unrelated);
    ok &= expectTrue(
        onlineFrame->text().contains(QStringLiteral("21"))
            && offlineFrame->text().contains(QStringLiteral("14")),
        "global/realtime/manual updates without correction purpose cannot populate pending");
    controller.shutdown();
    return ok;
}

} // namespace

int main(int argc, char **argv)
{
#ifdef _WIN32
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        _putenv_s("QT_QPA_PLATFORM", "offscreen");
    }
    _putenv_s("I2R_RLC_DISABLE_AUTO_DEVICE_START", "1");
#else
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        setenv("QT_QPA_PLATFORM", "offscreen", 1);
    }
    setenv("I2R_RLC_DISABLE_AUTO_DEVICE_START", "1", 1);
#endif
    QApplication app(argc, argv);
    bool ok = true;
    ok &= testLegacyStatusAndDeviceControlPanel();
    ok &= testFeaturePanelSwitching();
    ok &= testFeaturePanelSwitchingWithUiController();
    ok &= testAutomaticCalibrationConfirmationGate();
    ok &= testManualCalibrationPlacementImage();
    ok &= testOnlineCorrectionStartModeComboBox();
    ok &= testWorkpieceSelectorPanel();
    ok &= testIbvsAndLocalizationPanelsUseTwinButtonDesign();
    ok &= testTeachTrajectoryPanelSignals();
    ok &= testRobotSimJointTestPanel();
    ok &= testCorrectionSyncThreeKeyPresentation();
    ok &= testAppendLogSmoke();
    if (!ok) {
        return 1;
    }
    std::cout << "PASS: main widget panel tests\n";
    return 0;
}
