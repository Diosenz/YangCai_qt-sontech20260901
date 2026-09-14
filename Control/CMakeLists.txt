add_library(i2r_rlc_control STATIC
    src/AutomaticCalibrationPoseController.cpp
    src/CalibrationCaptureController.cpp
    src/OfflineCorrectionController.cpp
    src/OfflineCorrectionTrajectoryMatcher.cpp
    src/OnlineCorrectionController.cpp
    src/OnlineCorrectionExceptionProcessor.cpp
    src/OnlineCorrectionPathFollower.cpp
    src/OnlineCorrectionTrajectoryPlanner.cpp
    src/TeachTrajectoryRecorder.cpp
    src/TrackingTriggerPolicy.cpp
    include/AutomaticCalibrationPoseController.h
    include/CalibrationCaptureController.h
    include/CalibrationCaptureTypes.h
    include/OfflineCorrectionController.h
    include/OfflineCorrectionTrajectoryMatcher.h
    include/OfflineCorrectionTypes.h
    include/OnlineCorrectionController.h
    include/OnlineCorrectionExceptionProcessor.h
    include/OnlineCorrectionPathFollower.h
    include/OnlineCorrectionTrajectoryPlanner.h
    include/OnlineCorrectionTypes.h
    include/TeachTrajectoryRecorder.h
    include/TrackingTriggerPolicy.h
)

target_include_directories(i2r_rlc_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(i2r_rlc_control
    PUBLIC
        i2r_rlc_robot_comm
        i2r_rlc_vision
)

if(I2R_RLC_ENABLE_OPENCV)
    target_include_directories(i2r_rlc_control
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(i2r_rlc_control
        PUBLIC
            ${OpenCV_LIBS}
    )
    target_compile_definitions(i2r_rlc_control
        PUBLIC
            I2R_RLC_HAS_OPENCV
    )
endif()

i2r_rlc_apply_common_options(i2r_rlc_control)
