/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        main.cpp
 * @brief       Qt Widgets 应用入口
 * @details     创建 QApplication、主界面和 UI 控制器，并负责安全关闭。
 ******************************************************************************/
#include <memory>

#include <QApplication>

#ifdef I2R_RLC_ENABLE_CAMERA_ALLIED
#include "AlliedCameraDevice.h"
#endif

#include "MainWidget.h"
#include "UiController.h"

/**
 * @brief i2r_rlc Qt Widgets 应用入口。
 *
 * main 只负责创建 UI 和 UI Controller。具体算法、设备访问
 * 和业务函数放在各自模块中，避免回到旧 robotleading 的单体窗口实现方式。
 */
int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    i2r_rlc::ui::MainWidget mainWidget;
    i2r_rlc::ui::UiController controller(mainWidget);

#ifdef I2R_RLC_ENABLE_CAMERA_ALLIED
    controller.setLiveCameraDevice(
        std::make_unique<i2r_rlc::camera::AlliedCameraDevice>(),
        33);
#endif

    controller.initialize();
    QObject::connect(
        &application,
        &QApplication::aboutToQuit,
        &controller,
        &i2r_rlc::ui::UiController::shutdown);
    mainWidget.show();

    const int exitCode = QApplication::exec();
    controller.shutdown();
    return exitCode;
}
