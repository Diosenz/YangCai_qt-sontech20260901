find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(Eigen3 CONFIG REQUIRED)
if(I2R_RLC_ENABLE_OPENCV)
    if(NOT OpenCV_DIR AND DEFINED ENV{OpenCV_DIR})
        set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV package directory")
    elseif(NOT OpenCV_DIR AND DEFINED ENV{OPENCV_DIR})
        set(OpenCV_DIR "$ENV{OPENCV_DIR}" CACHE PATH "OpenCV package directory")
    endif()

    find_package(OpenCV QUIET COMPONENTS core imgproc)
    if(NOT OpenCV_FOUND)
        find_package(OpenCV QUIET)
    endif()

    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR
            "OpenCV was not found. Pass -DOpenCV_DIR=<path> or disable it with "
            "-DI2R_RLC_ENABLE_OPENCV=OFF."
        )
    endif()

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.0)
        message(FATAL_ERROR
            "OpenCV ${OpenCV_VERSION} was found, but this migration stage requires "
            "OpenCV 3.4.x for legacy algorithm compatibility."
        )
    endif()
endif()
find_package(PCL CONFIG REQUIRED COMPONENTS
    common
    io
    kdtree
    octree
    search
    filters
    sample_consensus
    features
    registration
    segmentation
    surface
    keypoints
)

separate_arguments(I2R_RLC_PCL_DEFINITIONS NATIVE_COMMAND "${PCL_DEFINITIONS}")

add_library(i2r_rlc_localization STATIC
    src/LocalizationApplicationService.cpp
    src/LocalizationGlobalIcpPipeline.cpp
    src/LocalizationLocalCorrectionPipeline.cpp
    src/LocalizationPipelineMath.cpp
    src/LocalizationLaserLineObservation.cpp
    src/LocalizationPointProjector.cpp
    src/LocalizationService.cpp
    src/LocalizationWorkflowController.cpp
    src/NullLocalizationVirtualReferenceProvider.cpp
    src/NullLocalizationService.cpp
    include/CalibrationData.h
    include/ILocalizationService.h
    include/ILocalizationVirtualReferenceProvider.h
    include/LocalizationApplicationService.h
    include/LocalizationGlobalIcpPipeline.h
    include/LocalizationLaserLineObservation.h
    include/LocalizationLocalCorrectionPipeline.h
    include/LocalizationPipelineMath.h
    include/LocalizationPointProjector.h
    include/LocalizationService.h
    include/LocalizationTypes.h
    include/LocalizationWorkflowController.h
    include/NullLocalizationService.h
)

set_target_properties(i2r_rlc_localization
    PROPERTIES
        AUTOMOC OFF
)

target_include_directories(i2r_rlc_localization
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(i2r_rlc_localization
    PUBLIC
        Eigen3::Eigen
        Qt${QT_VERSION_MAJOR}::Core
    PRIVATE
        ${PCL_LIBRARIES}
)

if(I2R_RLC_ENABLE_OPENCV)
    target_include_directories(i2r_rlc_localization
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(i2r_rlc_localization
        PRIVATE
            ${OpenCV_LIBS}
    )
    target_compile_definitions(i2r_rlc_localization
        PUBLIC
            I2R_RLC_ENABLE_OPENCV=1
    )
endif()

target_compile_options(i2r_rlc_localization
    PRIVATE
        ${I2R_RLC_PCL_DEFINITIONS}
)

i2r_rlc_apply_common_options(i2r_rlc_localization)
