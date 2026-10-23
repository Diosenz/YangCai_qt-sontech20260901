find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

add_library(i2r_rlc_ui STATIC
    src/AppPaths.cpp
    src/ConfigRepository.cpp
    src/CalibrationGuideOverlay.cpp
    src/CorrectionRobotJobMonitor.cpp
    src/CorrectionRobotStartGate.cpp
    src/Logger.cpp
    src/MainWidget.cpp
    src/OnlineCorrectionP001Presentation.cpp
    src/TrackingDiagnosticImageWriter.cpp
    src/OnlineCorrectionRunRecorder.cpp
    src/OnlineCorrectionWorkflow.cpp
    src/RobotSimVirtualReferenceCache.cpp
    src/RuntimeCoordinator.cpp
    src/RuntimeLocalRegistration.cpp
    src/OnlineLocalRegistrationWorker.cpp
    src/RuntimeWorkerAdapters.cpp
    src/SensorCalibrationParser.cpp
    src/UiController.cpp
    include/AppError.h
    include/AppPaths.h
    include/ConfigRepository.h
    include/CalibrationGuideOverlay.h
    include/CorrectionRobotJobMonitor.h
    include/CorrectionRobotStartGate.h
    include/Logger.h
    include/MainWidget.h
    include/OnlineCorrectionP001Presentation.h
    include/OnlineVisionDiagnosticOptions.h
    include/TrackingDiagnosticImageWriter.h
    include/OnlineCorrectionRunRecorder.h
    include/OnlineCorrectionWorkflow.h
    include/RobotSimVirtualReferenceCache.h
    include/RuntimeCoordinator.h
    include/OnlineLocalRegistrationStability.h
    include/OnlineLocalRegistrationWorker.h
    include/OnlineLocalRegistrationRecovery.h
    include/OnlineLocalSurfaceVerification.h
    include/RuntimeTrackingTypes.h
    include/TrackingFramePresentation.h
    include/RuntimeWorkerAdapters.h
    include/SensorCalibrationParser.h
    include/UiController.h
)

target_include_directories(i2r_rlc_ui
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(i2r_rlc_ui
    PUBLIC
        i2r_rlc_camera
        i2r_rlc_control
        i2r_rlc_digital_twin
        i2r_rlc_ibvs
        i2r_rlc_localization
        i2r_rlc_vision
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
)

i2r_rlc_apply_common_options(i2r_rlc_ui)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_add_executable(i2r_rlc
        MANUAL_FINALIZATION
        src/main.cpp
    )
else()
    add_executable(i2r_rlc
        src/main.cpp
    )
endif()

target_link_libraries(i2r_rlc
    PRIVATE
        i2r_rlc_ui
)

if(TARGET i2r_rlc_camera_vendor)
    target_link_libraries(i2r_rlc
        PRIVATE
            i2r_rlc_camera_vendor
    )
endif()

i2r_rlc_apply_common_options(i2r_rlc)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(i2r_rlc)
endif()
