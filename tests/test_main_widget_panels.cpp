/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_main_widget_panels.cpp
 * @brief       main widget panels 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QStackedWidget>
#include <QStringList>

#include "MainWidget.h"
#include "OnlineCorrectionTypes.h"
#include "UiController.h"
#include "TrackingFramePresentation.h"

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
    ok &= expectTrue(findLabel(widget, "onlineCorrectionP001Label") != nullptr
                         && findLabel(widget, "onlineCorrectionP001Label")->parent()
                                != statusGroup,
                     "online correction P001 label should be in its feature panel");
    ok &= expectTrue(
        findLabel(widget, "onlineCorrectionP001Label")->text()
            == QStringLiteral("P001：未计算"),
        "online correction P001 must not show zero coordinates before generation");
    widget.setOnlineCorrectionP001Text(QStringLiteral("P001 test text"));
    ok &= expectTrue(
        findLabel(widget, "onlineCorrectionP001Label")->text()
            == QStringLiteral("P001 test text"),
        "online correction P001 label should expose its view update interface");
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

bool testOnlineProcessingOptionsRemainEditable()
{
    i2r_rlc::ui::MainWidget widget;
    auto* localIcp = widget.findChild<QCheckBox*>(QStringLiteral("onlineLocalIcpCheckBox"));
    auto* mode = findComboBox(widget, "onlineFusionModeComboBox");
    auto* simulation = widget.findChild<QDoubleSpinBox*>(QStringLiteral("onlineSimulationWeightSpinBox"));
    auto* kcf = widget.findChild<QDoubleSpinBox*>(QStringLiteral("onlineKcfWeightSpinBox"));
    if (localIcp == nullptr || mode == nullptr || simulation == nullptr || kcf == nullptr) {
        return expectTrue(false, "online processing controls should exist");
    }
    bool ok = true;
    int count = 0;
    bool lastLocalEnabled = false;
    int lastMode = -1;
    double lastSimulationWeight = -1.0;
    QObject::connect(&widget, &i2r_rlc::ui::MainWidget::onlineProcessingOptionsChanged,
        [&count, &lastLocalEnabled, &lastMode, &lastSimulationWeight](bool enabled, int selectedMode, double weight) {
            ++count;
            lastLocalEnabled = enabled;
            lastMode = selectedMode;
            lastSimulationWeight = weight;
        });
    widget.setOnlineProcessingOptions(true, 0, 0.4);
    ok &= expectTrue(count == 0 && !simulation->isEnabled() && !kcf->isEnabled(),
                     "automatic mode synchronization should block feedback and disable manual weights");
    mode->setCurrentIndex(1);
    ok &= expectTrue(count == 1 && lastLocalEnabled && lastMode == 1
                         && simulation->isEnabled() && kcf->isEnabled(),
                     "manual mode should immediately publish and enable both weights");
    simulation->setValue(25.0);
    ok &= expectTrue(count == 2 && std::abs(lastSimulationWeight - 0.25) < 1e-9
                         && kcf->value() == 75.0,
                     "simulation weight should complement KCF with one user update");
    kcf->setValue(35.0);
    ok &= expectTrue(count == 3 && std::abs(lastSimulationWeight - 0.65) < 1e-9
                         && simulation->value() == 65.0,
                     "KCF weight should complement simulation with one user update");
    widget.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：运行中"));
    localIcp->setChecked(false);
    mode->setCurrentIndex(2);
    ok &= expectTrue(count == 5 && !lastLocalEnabled && lastMode == 2
                         && localIcp->isEnabled() && mode->isEnabled()
                         && !simulation->isEnabled() && !kcf->isEnabled()
                         && !simulation->isHidden() && !kcf->isHidden(),
                     "running status must not lock or hide processing options");
    widget.setOnlineProcessingOptions(true, 1, 2.0);
    ok &= expectTrue(count == 5 && simulation->value() == 100.0 && kcf->value() == 0.0,
                     "programmatic weight synchronization should clamp values without feedback");
    return ok;
}

bool testOnlineDiagnosticsKeepStableGeometry()
{
    i2r_rlc::ui::MainWidget widget;
    auto* entry = findButton(widget, "entryOnlineCorrectionButton");
    auto* scroll = widget.findChild<QScrollArea*>(QStringLiteral("onlineCorrectionDetailsScrollArea"));
    auto* p001 = findLabel(widget, "onlineCorrectionP001Label");
    auto* error = findLabel(widget, "onlineCorrectionSyncErrorLabel");
    if (entry == nullptr || scroll == nullptr || p001 == nullptr || error == nullptr) {
        return expectTrue(false, "online diagnostics should expose stable scroll content");
    }
    widget.show();
    entry->click();
    QCoreApplication::processEvents();
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum() / 2);
    const int scrollValue = scroll->verticalScrollBar()->value();
    const int scrollMaximum = scroll->verticalScrollBar()->maximum();
    const QSize viewportSize = scroll->viewport()->size();
    const QRect p001Geometry = p001->geometry();
    const QString longError = QStringLiteral("最近同步错误：连接超时，未完成同帧参考查询，请检查通信状态。").repeated(12);
    bool ok = true;
    for (int index = 0; index < 6; ++index) {
        const bool detailed = (index % 2) == 0;
        widget.setOnlineCorrectionStateText(detailed
            ? QStringLiteral("在线纠偏：运行中\nICP 配准已采用，等待后续检测与机器人状态。\n详细诊断信息。").repeated(4)
            : QStringLiteral("在线纠偏：运行中"));
        widget.setOnlineCorrectionP001Text(detailed
            ? QStringLiteral("P001：X = 123.456，Y = -789.123，Z = 456.789\nRX = 123.45，RY = 67.89，RZ = -12.34\n").repeated(4)
            : QStringLiteral("P001：未计算"));
        widget.setOnlineCorrectionSyncDetails(QStringLiteral("逐帧同步：运行中"),
            QStringLiteral("最近确认帧：1234567"), detailed ? longError : QStringLiteral("最近同步错误：无"));
        widget.setDigitalTwinStateText(detailed
            ? QStringLiteral("仿真：与检测图同帧\n端口：61541\n显示帧：1234567\n激光线：512 图像点 / 512 三维点\n焊缝点二维：(1234, 5678) pixel")
            : QStringLiteral("仿真：未连接"));
        QCoreApplication::processEvents();
        ok &= expectTrue(scroll->viewport()->size() == viewportSize
                             && scroll->verticalScrollBar()->maximum() == scrollMaximum
                             && scroll->verticalScrollBar()->value() == scrollValue
                             && p001->geometry() == p001Geometry,
                         "changing online diagnostic lengths must not move content or its scroll position");
        if (detailed) {
            ok &= expectTrue(error->text() == longError && error->toolTip() == longError,
                             "fixed diagnostic geometry should retain full error text and tooltip");
        }
    }
    return ok;
}

bool testOnlineLocalIcpDistanceControl()
{
    i2r_rlc::ui::MainWidget widget;
    auto* localIcp = widget.findChild<QCheckBox*>(QStringLiteral("onlineLocalIcpCheckBox"));
    auto* row = widget.findChild<QWidget*>(QStringLiteral("onlineLocalIcpDistanceRow"));
    auto* distance = widget.findChild<QDoubleSpinBox*>(QStringLiteral("onlineLocalIcpDistanceSpinBox"));
    if (localIcp == nullptr || row == nullptr || distance == nullptr) {
        return expectTrue(false, "online local ICP distance controls should exist");
    }
    bool ok = expectTrue(localIcp->isChecked() && !row->isHidden()
                             && distance->value() == 10.0
                             && distance->minimum() == 0.1 && distance->maximum() == 10.0,
                         "enabled local ICP should expose a default 10 mm correspondence distance capped at 10 mm");
    ok &= expectTrue(distance->toolTip().contains(QStringLiteral("不是最终残差验收阈值")),
                     "distance help must distinguish correspondence selection from residual acceptance");
    int distanceSignalCount = 0;
    double lastDistance = 0.0;
    QObject::connect(&widget, &i2r_rlc::ui::MainWidget::onlineLocalIcpDistanceChanged,
        [&distanceSignalCount, &lastDistance](double value) {
            ++distanceSignalCount;
            lastDistance = value;
        });
    localIcp->setChecked(false);
    ok &= expectTrue(row->isHidden() && distanceSignalCount == 0,
                     "turning local ICP off should hide its distance without modifying it");
    localIcp->setChecked(true);
    widget.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：运行中"));
    distance->setValue(6.5);
    ok &= expectTrue(!row->isHidden() && distance->isEnabled()
                         && distanceSignalCount == 1 && lastDistance == 6.5,
                     "distance must remain editable during online correction with one update per value change");
    widget.setOnlineLocalIcpDistance(20.0);
    ok &= expectTrue(distance->value() == 10.0 && distanceSignalCount == 1,
                     "programmatic distance update should clamp to 10 mm without signal feedback");
    widget.setOnlineProcessingOptions(false, 1, 0.5);
    ok &= expectTrue(row->isHidden() && distance->value() == 10.0,
                     "programmatic local ICP disable should hide the row and retain its setting");
    widget.setOnlineProcessingOptions(true, 1, 0.5);
    ok &= expectTrue(!row->isHidden() && distance->isEnabled() && distanceSignalCount == 1,
                     "programmatic re-enable should restore the distance row without user feedback");
    return ok;
}

bool testOnlineLocalIcpTranslationLimitControl()
{
    i2r_rlc::ui::MainWidget widget;
    auto* localIcp = widget.findChild<QCheckBox*>(QStringLiteral("onlineLocalIcpCheckBox"));
    auto* row = widget.findChild<QWidget*>(QStringLiteral("onlineLocalIcpTranslationRow"));
    auto* translation = widget.findChild<QDoubleSpinBox*>(QStringLiteral("onlineLocalIcpTranslationSpinBox"));
    auto* distance = widget.findChild<QDoubleSpinBox*>(QStringLiteral("onlineLocalIcpDistanceSpinBox"));
    if (localIcp == nullptr || row == nullptr || translation == nullptr || distance == nullptr) {
        return expectTrue(false, "online local ICP translation limit controls should exist");
    }
    bool ok = expectTrue(localIcp->isChecked() && !row->isHidden()
                             && translation->value() == 30.0 && translation->minimum() == 0.01
                             && translation->maximum() == 30.0 && translation->decimals() == 2,
                         "local ICP translation limit should default to 30 mm with a 30 mm maximum");
    ok &= expectTrue(translation->toolTip().contains(QStringLiteral("三轴合成平移长度"))
                         && translation->toolTip().contains(QStringLiteral("整次拒绝"))
                         && translation->toolTip().contains(QStringLiteral("不截断")),
                     "translation limit help should describe whole-correction rejection and its vector length");
    int signalCount = 0;
    double lastValue = 0.0;
    QObject::connect(&widget, &i2r_rlc::ui::MainWidget::onlineLocalIcpTranslationLimitChanged,
        [&signalCount, &lastValue](double value) {
            ++signalCount;
            lastValue = value;
        });
    localIcp->setChecked(false);
    ok &= expectTrue(row->isHidden() && signalCount == 0,
                     "disabling ICP should hide the translation limit without changing its value");
    localIcp->setChecked(true);
    widget.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：运行中"));
    translation->setValue(6.5);
    ok &= expectTrue(!row->isHidden() && translation->isEnabled()
                         && signalCount == 1 && lastValue == 6.5 && distance->value() == 10.0,
                     "translation limit must support one hot-update independently of correspondence distance");
    distance->setValue(3.0);
    ok &= expectTrue(translation->value() == 6.5 && signalCount == 1,
                     "correspondence distance adjustment must not modify the translation limit");
    widget.setOnlineLocalIcpTranslationLimit(100.0);
    ok &= expectTrue(translation->value() == 30.0 && signalCount == 1,
                     "programmatic translation limit should clamp without feedback");
    widget.setOnlineLocalIcpTranslationLimit(std::numeric_limits<double>::quiet_NaN());
    ok &= expectTrue(translation->value() == 30.0 && signalCount == 1,
                     "nonfinite translation limit should restore 30 mm without feedback");
    widget.setOnlineProcessingOptions(false, 1, 0.5);
    ok &= expectTrue(row->isHidden() && translation->value() == 30.0,
                     "programmatic disable should hide the translation row and retain its setting");
    widget.setOnlineProcessingOptions(true, 1, 0.5);
    ok &= expectTrue(!row->isHidden() && translation->isEnabled() && signalCount == 1,
                     "programmatic enable should restore the translation row without feedback");
    return ok;
}

bool testOnlineLocalIcpStabilityControl()
{
    i2r_rlc::ui::MainWidget widget;
    auto* localIcp = widget.findChild<QCheckBox*>(QStringLiteral("onlineLocalIcpCheckBox"));
    auto* stability = widget.findChild<QCheckBox*>(QStringLiteral("onlineLocalIcpStabilityCheckBox"));
    if (localIcp == nullptr || stability == nullptr) {
        return expectTrue(false, "online local ICP stability control should exist");
    }
    bool ok = expectTrue(!stability->isChecked() && !stability->isHidden()
                             && stability->isEnabled(),
                         "stability protection should default off and be editable before start");
    ok &= expectTrue(stability->text() == QStringLiteral("在线 ICP 稳定更新保护")
                         && stability->toolTip().contains(QStringLiteral("焊点明确改善"))
                         && stability->toolTip().contains(QStringLiteral("监测历史区域"))
                         && stability->toolTip().contains(QStringLiteral("仅作诊断"))
                         && stability->toolTip().contains(QStringLiteral("不比较不同窗口")),
                     "stability help should explain independent benefit and historical region protection");
    int stabilitySignalCount = 0;
    int processingSignalCount = 0;
    bool lastEnabled = false;
    QObject::connect(&widget, &i2r_rlc::ui::MainWidget::onlineLocalIcpStabilityChanged,
        [&stabilitySignalCount, &lastEnabled](bool enabled) {
            ++stabilitySignalCount;
            lastEnabled = enabled;
        });
    QObject::connect(&widget, &i2r_rlc::ui::MainWidget::onlineProcessingOptionsChanged,
        [&processingSignalCount](bool, int, double) {
            ++processingSignalCount;
        });
    stability->setChecked(true);
    ok &= expectTrue(stabilitySignalCount == 1 && lastEnabled && processingSignalCount == 0,
                     "stability toggle should publish exactly one independent user update");
    localIcp->setChecked(false);
    ok &= expectTrue(stability->isHidden() && stability->isChecked()
                         && stabilitySignalCount == 1 && processingSignalCount == 1,
                     "ICP disable should hide stability protection and retain its choice");
    localIcp->setChecked(true);
    widget.setOnlineCorrectionStateText(QStringLiteral("在线纠偏：运行中"));
    stability->setChecked(false);
    ok &= expectTrue(!stability->isHidden() && stability->isEnabled()
                         && stabilitySignalCount == 2 && !lastEnabled && processingSignalCount == 2,
                     "stability protection should remain independently editable during correction");
    widget.setOnlineLocalIcpStabilityEnabled(true);
    widget.setOnlineProcessingOptions(false, 1, 0.5);
    ok &= expectTrue(stability->isHidden() && stability->isChecked()
                         && stabilitySignalCount == 2 && processingSignalCount == 2,
                     "programmatic synchronization should retain protection without user feedback");
    widget.setOnlineProcessingOptions(true, 1, 0.5);
    ok &= expectTrue(!stability->isHidden() && stability->isChecked()
                         && stabilitySignalCount == 2 && processingSignalCount == 2,
                     "programmatic ICP enable should restore the retained protection choice");
    widget.setOnlineLocalIcpStabilityEnabled(false);
    ok &= expectTrue(!stability->isChecked() && stabilitySignalCount == 2,
                     "programmatic stability disable should block user feedback");
    return ok;
}

bool testOnlineBusinessAndSyncTextRemainStable()
{
    using i2r_rlc::control::OnlineCorrectionPhase;
    using i2r_rlc::control::OnlineCorrectionUpdate;
    using i2r_rlc::ui::CorrectionPurpose;
    using i2r_rlc::ui::CorrectionSyncState;
    using i2r_rlc::ui::CorrectionSyncUpdate;
    i2r_rlc::ui::MainWidget widget;
    i2r_rlc::ui::UiController controller(widget);
    auto* business = findLabel(widget, "onlineCorrectionStateLabel");
    auto* sync = findLabel(widget, "onlineCorrectionSyncStateLabel");
    auto* error = findLabel(widget, "onlineCorrectionSyncErrorLabel");
    if (business == nullptr || sync == nullptr || error == nullptr) {
        controller.shutdown();
        return expectTrue(false, "online business and synchronization labels should exist");
    }
    const auto applyBusiness = [&controller](const OnlineCorrectionUpdate& update) {
        return QMetaObject::invokeMethod(&controller, "showOnlineCorrectionUpdate",
            Qt::DirectConnection, Q_ARG(i2r_rlc::control::OnlineCorrectionUpdate, update));
    };
    const auto applySync = [&controller](const CorrectionSyncUpdate& update) {
        return QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate",
            Qt::DirectConnection, Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, update));
    };
    OnlineCorrectionUpdate update;
    update.succeeded = true;
    update.phase = OnlineCorrectionPhase::OnlineRunning;
    update.message = "online business detail retained";
    bool ok = expectTrue(applyBusiness(update), "online update must be callable through Qt metadata");
    const QString businessText = business->text();
    ok &= expectTrue(businessText.contains(QStringLiteral("online business detail retained")),
                     "online business result should be displayed immediately");
    update.message = "robot state accepted";
    ok &= expectTrue(applyBusiness(update) && business->text() == businessText,
                     "robot polling heartbeat must not overwrite online business details");
    update.phase = OnlineCorrectionPhase::SegmentFinishing;
    update.message = "online segment finishing";
    ok &= expectTrue(applyBusiness(update)
                         && business->text().contains(QStringLiteral("online segment finishing")),
                     "real phase transition should update immediately after a heartbeat");

    CorrectionSyncUpdate syncUpdate;
    syncUpdate.purpose = CorrectionPurpose::OnlineCorrection;
    syncUpdate.sessionId = 31;
    syncUpdate.frameId = 41;
    syncUpdate.state = CorrectionSyncState::Querying;
    ok &= expectTrue(applySync(syncUpdate)
                         && sync->text() == QStringLiteral("逐帧同步：运行中"),
                     "online query stage should display a stable running status");
    syncUpdate.state = CorrectionSyncState::ObservationReady;
    syncUpdate.hasObservation = true;
    syncUpdate.observation.valid = true;
    syncUpdate.observation.frameId = syncUpdate.frameId;
    ok &= expectTrue(applySync(syncUpdate)
                         && sync->text() == QStringLiteral("逐帧同步：运行中"),
                     "online observation-ready stage should retain the running status");
    syncUpdate.state = CorrectionSyncState::VisionConsumed;
    ok &= expectTrue(applySync(syncUpdate)
                         && sync->text() == QStringLiteral("逐帧同步：运行中"),
                     "online consumed stage should retain the running status");
    syncUpdate.state = CorrectionSyncState::Failed;
    syncUpdate.message = QStringLiteral("专项测试同步失败");
    ok &= expectTrue(applySync(syncUpdate)
                         && sync->text().contains(QStringLiteral("同步失败"))
                         && error->text().contains(syncUpdate.message),
                     "synchronization errors should replace the stable status immediately");
    controller.shutdown();
    return ok;
}

/** @brief 复用现有停止信号，区分清理、保持待确认、设备反馈及通信失败。 */
bool testOnlineOperationStopPresentation()
{
    using namespace i2r_rlc;
    ui::MainWidget widget;
    ui::UiController controller(widget);
    controller.initialize();
    auto *business = findLabel(widget, "onlineCorrectionStateLabel");
    if (business == nullptr) { controller.shutdown(); return false; }
    bool ok = expectTrue(QMetaObject::invokeMethod(&widget, "stopRequested", Qt::DirectConnection)
            && !business->text().contains(QStringLiteral("HOLD")),
        "stop without an online operation performs ordinary cleanup without requesting HOLD");
    ui::CorrectionSyncUpdate sync;
    sync.purpose = ui::CorrectionPurpose::OnlineCorrection;
    sync.sessionId = 71;
    sync.state = ui::CorrectionSyncState::Idle;
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate",
            Qt::DirectConnection, Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, sync)),
        "online session presentation is established through its existing public Qt boundary");
    ok &= expectTrue(QMetaObject::invokeMethod(&widget, "stopRequested", Qt::DirectConnection)
            && business->text().contains(QStringLiteral("已请求"))
            && business->text().contains(QStringLiteral("等待"))
            && business->text().contains(QStringLiteral("停弧未验证")),
        "existing Stop signal enters pending HOLD without claiming completed welding stop");
    robot_comm::RobotOperationStopState stop;
    stop.holdRequestLatched = true;
    stop.requestGeneration = 1;
    stop.status = robot_comm::RobotOperationStopStatus::HoldConfirmed;
    const auto applyStop = [&controller](const robot_comm::RobotOperationStopState &state) {
        return QMetaObject::invokeMethod(&controller, "showOnlineOperationStop",
            Qt::DirectConnection, Q_ARG(i2r_rlc::robot_comm::RobotOperationStopState, state));
    };
    ok &= expectTrue(applyStop(stop)
            && business->text().contains(QStringLiteral("HOLD 已反馈"))
            && business->text().contains(QStringLiteral("停弧未验证"))
            && business->text().contains(QStringLiteral("零速未验证"))
            && business->text().contains(QStringLiteral("人工")),
        "fresh HOLD feedback is presented separately from arc-off and mechanical-zero-speed confirmation");
    const auto confirmedText = business->text();
    control::OnlineCorrectionUpdate late;
    late.succeeded = true;
    late.phase = control::OnlineCorrectionPhase::Idle;
    late.message = "late cleared control state";
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "showOnlineCorrectionUpdate",
            Qt::DirectConnection, Q_ARG(i2r_rlc::control::OnlineCorrectionUpdate, late))
            && business->text() == confirmedText,
        "late ordinary control reset cannot erase operation HOLD presentation");
    ok &= expectTrue(QMetaObject::invokeMethod(&widget, "onlineCorrectionFeaturePanelClosed",
            Qt::DirectConnection) && business->text() == confirmedText,
        "closing the panel cannot erase the latched HOLD status");
    stop.status = robot_comm::RobotOperationStopStatus::Unconfirmed;
    stop.error = robot_comm::RobotCommandError::ReceiveFailed;
    stop.message = "simulated feedback lost";
    ok &= expectTrue(applyStop(stop)
            && business->text().contains(QStringLiteral("无法确认"))
            && !business->text().contains(QStringLiteral("HOLD 已反馈"))
            && business->text().contains(QStringLiteral("停弧未验证")),
        "feedback failure removes stale confirmation and preserves the unverified arc boundary");
    controller.shutdown();
    return ok;
}

/** @brief 已结束会话残留身份不能令普通停止误发机器人 HOLD。 */
bool testOnlineCompletedSessionStopIsOrdinary()
{
    using namespace i2r_rlc;
    bool ok = true;
    for (int scenario = 0; scenario < 4; ++scenario) {
        ui::MainWidget widget;
        ui::UiController controller(widget);
        controller.initialize();
        auto *business = findLabel(widget, "onlineCorrectionStateLabel");
        ui::CorrectionSyncUpdate sync;
        sync.purpose = ui::CorrectionPurpose::OnlineCorrection;
        sync.sessionId = 720 + scenario;
        sync.state = ui::CorrectionSyncState::Idle;
        ok &= QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate",
            Qt::DirectConnection, Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, sync));
        if (scenario < 2) {
            control::OnlineCorrectionUpdate terminal;
            terminal.succeeded = scenario == 0;
            terminal.phase = scenario == 0 ? control::OnlineCorrectionPhase::Completed
                                          : control::OnlineCorrectionPhase::Error;
            terminal.message = "terminal operation";
            ok &= QMetaObject::invokeMethod(&controller, "showOnlineCorrectionUpdate",
                Qt::DirectConnection, Q_ARG(i2r_rlc::control::OnlineCorrectionUpdate, terminal));
        } else {
            sync.state = scenario == 2 ? ui::CorrectionSyncState::Cleared
                                      : ui::CorrectionSyncState::StoppedAfterFailures;
            ok &= QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate",
                Qt::DirectConnection, Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, sync));
        }
        ok &= expectTrue(QMetaObject::invokeMethod(&widget, "stopRequested", Qt::DirectConnection)
                && business != nullptr && !business->text().contains(QStringLiteral("HOLD")),
            "completed, failed or terminated session identity cannot turn ordinary Stop into HOLD");
        controller.shutdown();
    }
    return ok;
}

bool testLivePreviewContinuesWhileDetectionSnapshotIsDisplayed()
{
    using namespace i2r_rlc;
    ui::MainWidget widget;
    ui::UiController controller(widget);
    const auto root = QDir::current();
    const QString directory = QStringLiteral("tests/fixtures/online_tracking_20261027/");
    const QImage detected(root.filePath(directory + QStringLiteral("20261027_122726_191_3937.png")));
    const QImage preview(root.filePath(directory + QStringLiteral("20261027_122726_191_3969.png")));
    bool ok = expectTrue(!detected.isNull() && !preview.isNull(), "captured UI replay images exist");
    ui::CorrectionSyncUpdate ready;
    ready.state = ui::CorrectionSyncState::Idle;
    ready.purpose = ui::CorrectionPurpose::OnlineCorrection;
    ready.sessionId = 1;
    ready.frameId = 3937;
    ready.observation.frameId = 3937;
    QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, ready));
    ready.state = ui::CorrectionSyncState::ImageOnlyReady;
    QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, ready));
    ready.state = ui::CorrectionSyncState::VisionConsumed;
    QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, ready));
    ui::RuntimeTrackingResult result;
    result.frameId = 3937;
    result.triggerAccepted = result.valid = true;
    result.evidencePolicyEnabled = result.kcfPixelAvailable = true;
    result.pointSource = vision::OnlineSeamPointSource::Geometry;
    result.imageXPixel = 550;
    result.imageYPixel = 585;
    result.correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
    result.correlation.sessionId = 1;
    result.correlation.frameId = 3937;
    result.frameContext.emplace();
    result.frameContext->sourceFrame.frameId = 3937;
    result.frameContext->sourceFrame.image = detected;
    result.frameContext->observation = ready.observation;
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "showRuntimeTrackingResult", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::RuntimeTrackingResult, result)), "runtime result boundary invoked");
    auto *live = findLabel(widget, "runtimePreviewImage");
    auto *mainImage = live == nullptr ? nullptr : qobject_cast<QLabel *>(live->parentWidget());
    const QImage snapshot = mainImage == nullptr ? QImage() : mainImage->pixmap().toImage();
    camera::CameraFrame frame;
    frame.frameId = 3969;
    frame.image = preview;
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "handleRuntimeFrameReady", Qt::DirectConnection,
        Q_ARG(i2r_rlc::camera::CameraFrame, frame)), "runtime preview boundary invoked");
    auto *caption = findLabel(widget, "runtimePreviewCaption");
    auto *consumed = findLabel(widget, "onlineCorrectionConsumedFrameLabel");
    if (live != nullptr && live->isHidden()) {
        std::cerr << "preview diagnostic: consumed="
            << (consumed == nullptr ? "missing" : consumed->text().toStdString())
            << " caption=" << (caption == nullptr ? "missing" : caption->text().toStdString())
            << '\n';
    }
    ok &= expectTrue(live != nullptr && !live->isHidden() && !live->pixmap().isNull(),
                     "fresh camera image remains visible during delayed detection");
    if (live != nullptr) {
        ok &= expectTrue(live->property("previewFrameId").toLongLong() == 3969
            && live->property("detectionFrameId").toLongLong() == 3937,
            "preview and detection retain distinct exposure identities");
        const auto shown = live->pixmap().toImage();
        const auto expected = preview.scaled(shown.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        ok &= expectTrue(shown.pixelColor(shown.width() / 2, shown.height() / 2)
            == expected.pixelColor(expected.width() / 2, expected.height() / 2),
            "new preview pixels are not old snapshot pixels or stale overlay");
        ok &= expectTrue(!snapshot.isNull() && mainImage->pixmap().toImage() == snapshot,
            "live camera frames cannot replace the exposure supporting main-image overlays");
    }
    ready.frameId = 3969;
    ready.observation.frameId = 3969;
    ready.state = ui::CorrectionSyncState::ImageOnlyReady;
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, ready)), "next exposure ready boundary invoked");
    ready.state = ui::CorrectionSyncState::VisionConsumed;
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "handleCorrectionSyncUpdate", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::CorrectionSyncUpdate, ready)), "next exposure consumption boundary invoked");
    result.frameId = 3969;
    result.correlation.frameId = 3969;
    result.frameContext->sourceFrame.frameId = 3969;
    result.frameContext->sourceFrame.image = preview;
    result.frameContext->observation.frameId = 3969;
    ok &= expectTrue(QMetaObject::invokeMethod(&controller, "showRuntimeTrackingResult", Qt::DirectConnection,
        Q_ARG(i2r_rlc::ui::RuntimeTrackingResult, result)), "next runtime result boundary invoked");
    ok &= expectTrue(live != nullptr && live->property("detectionFrameId").toLongLong() == 3969
        && caption != nullptr && caption->text().contains(QStringLiteral("检测快照 3969")),
        "a new detection updates preview attribution even without another camera callback");
    const auto snapshotPath = qEnvironmentVariable("I2R_UI_REPLAY_SNAPSHOT");
    if (!snapshotPath.isEmpty() && mainImage != nullptr) {
        ok &= expectTrue(mainImage->grab().save(snapshotPath), "offline UI snapshot saved");
    }
    widget.showFrame(frame);
    ok &= expectTrue(live != nullptr && live->isHidden() && caption->isHidden(),
        "returning to camera-only display clears delayed detection preview state");
    controller.shutdown();
    return ok;
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
    if (app.arguments().contains(QStringLiteral("--live-preview-contract"))) {
        return testLivePreviewContinuesWhileDetectionSnapshotIsDisplayed() ? 0 : 1;
    }
    if (app.arguments().contains(QStringLiteral("--operation-stop-ui"))) {
        return testOnlineOperationStopPresentation()
            && testOnlineCompletedSessionStopIsOrdinary() ? 0 : 1;
    }
    bool ok = true;
    ok &= testLegacyStatusAndDeviceControlPanel();
    ok &= testFeaturePanelSwitching();
    ok &= testFeaturePanelSwitchingWithUiController();
    ok &= testAutomaticCalibrationConfirmationGate();
    ok &= testManualCalibrationPlacementImage();
    ok &= testOnlineCorrectionStartModeComboBox();
    ok &= testOnlineProcessingOptionsRemainEditable();
    ok &= testOnlineDiagnosticsKeepStableGeometry();
    ok &= testOnlineLocalIcpDistanceControl();
    ok &= testOnlineLocalIcpTranslationLimitControl();
    ok &= testOnlineLocalIcpStabilityControl();
    ok &= testOnlineBusinessAndSyncTextRemainStable();
    ok &= testOnlineOperationStopPresentation();
    ok &= testOnlineCompletedSessionStopIsOrdinary();
    ok &= testWorkpieceSelectorPanel();
    ok &= testIbvsAndLocalizationPanelsUseTwinButtonDesign();
    ok &= testTeachTrajectoryPanelSignals();
    ok &= testRobotSimJointTestPanel();
    ok &= testCorrectionSyncThreeKeyPresentation();
    ok &= testLivePreviewContinuesWhileDetectionSnapshotIsDisplayed();
    ok &= testAppendLogSmoke();
    if (!ok) {
        return 1;
    }
    std::cout << "PASS: main widget panel tests\n";
    return 0;
}
