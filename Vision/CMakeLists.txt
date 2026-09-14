find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui)
find_package(Eigen3 CONFIG REQUIRED)
find_package(tinysplinecxx CONFIG REQUIRED)
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

if(I2R_RLC_ENABLE_OPENCV)
    if(NOT OpenCV_DIR AND DEFINED ENV{OpenCV_DIR})
        set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV package directory")
    elseif(NOT OpenCV_DIR AND DEFINED ENV{OPENCV_DIR})
        set(OpenCV_DIR "$ENV{OPENCV_DIR}" CACHE PATH "OpenCV package directory")
    endif()

    find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
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

    message(STATUS "Vision OpenCV enabled: ${OpenCV_VERSION}")
    message(STATUS "Vision OpenCV_DIR: ${OpenCV_DIR}")
endif()

set(I2R_RLC_VISION_SOURCES
    src/geometry/IcpRegistrationMatcher.cpp
    src/geometry/PositionerKinematics.cpp
    src/geometry/WeldProjectionContextBuilder.cpp
    third_party/libicp/icp.cpp
    third_party/libicp/icpPointToPlane.cpp
    third_party/libicp/icpPointToPoint.cpp
    third_party/libicp/kdtree.cpp
    third_party/libicp/matrix.cpp
    include/geometry/IcpRegistrationMatcher.h
    include/geometry/IcpTypes.h
    include/geometry/PositionerKinematics.h
    include/geometry/WeldProjectionContextBuilder.h
)

if(I2R_RLC_ENABLE_OPENCV)
    add_library(i2r_rlc_shape_based_matching STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/shape_based_matching/line2Dup.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/shape_based_matching/line2Dup.h
    )
    target_include_directories(i2r_rlc_shape_based_matching
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/shape_based_matching
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/shape_based_matching/MIPP
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(i2r_rlc_shape_based_matching
        PUBLIC
            ${OpenCV_LIBS}
    )
    i2r_rlc_apply_common_options(i2r_rlc_shape_based_matching)

    list(APPEND I2R_RLC_VISION_SOURCES
        src/feature/ActiveContourWeldFeatureDetector.cpp
        src/feature/ContourControlPointTools.cpp
        src/feature/GapFeatureDetector.cpp
        src/feature/GrooveEdgeDetector.cpp
        src/feature/PointCloudGapWidthDetector.cpp
        src/feature/WeldEndpointDetector.cpp
        src/feature/WeldFeatureDetector.cpp
        src/feature/WeldStartPointDetector.cpp
        src/geometry/LineGeometryTools.cpp
        src/geometry/WeldImageToSpaceProjector.cpp
        src/feature/SimulatedLaserWeldDetector.cpp
        src/pipeline/WeldVisionAlgorithmPipeline.cpp
        src/preprocess/ImageDenoise.cpp
        src/preprocess/DirectionalMorphologyFilter.cpp
        src/preprocess/ImageQualityEvaluator.cpp
        src/preprocess/ImageMorphologyUtils.cpp
        src/preprocess/ImagePreprocessor.cpp
        src/stripe/ActiveContourStripeExtractor.cpp
        src/stripe/DpStripeCenterlineDetector.cpp
        src/stripe/IsffaStripeDetector.cpp
        src/stripe/LaserStripeExtractor.cpp
        src/stripe/LaserLineResponseTools.cpp
        src/stripe/SamStripeDetector.cpp
        src/stripe/StegerStripeDetector.cpp
        src/stripe/StripeLineRepairTools.cpp
        src/stripe/StripeResponseEvaluationTools.cpp
        src/stripe/SubpixelCenterlineTools.cpp
        src/stripe/WeldSkeletonFeatureTools.cpp
        src/track/FilterKalman.cpp
        src/track/KcfTracker.cpp
        src/track/LegacyOnlineKcfDetector.cpp
        src/track/KcfFftTools.cpp
        src/track/KcfFhog.cpp
        src/track/KcfRectTools.cpp
        include/common/VisionImageTypes.h
        include/feature/ActiveContourWeldFeatureDetector.h
        include/feature/ContourControlPointTools.h
        include/feature/GapFeatureDetector.h
        include/feature/GrooveEdgeDetector.h
        include/feature/PointCloudGapWidthDetector.h
        include/feature/WeldEndpointDetector.h
        include/feature/WeldFeatureDetector.h
        include/feature/WeldStartPointDetector.h
        include/feature/WeldStartPointTypes.h
        include/feature/WeldFeatureTypes.h
        include/geometry/LineGeometryTools.h
        include/geometry/WeldGeometryTypes.h
        include/geometry/WeldImageToSpaceProjector.h
        include/feature/SimulatedLaserWeldDetector.h
        include/pipeline/WeldVisionAlgorithmPipeline.h
        include/preprocess/ImageDenoise.h
        include/preprocess/DirectionalMorphologyFilter.h
        include/preprocess/GaborFilterTypes.h
        include/preprocess/ImageQualityEvaluator.h
        include/preprocess/ImageMorphologyUtils.h
        include/preprocess/ImagePreprocessor.h
        include/stripe/ActiveContourStripeExtractor.h
        include/stripe/DpStripeCenterlineDetector.h
        include/stripe/IsffaStripeDetector.h
        include/stripe/LaserCenterlineTypes.h
        include/stripe/LaserLineResponseTools.h
        include/stripe/LaserStripeExtractor.h
        include/stripe/SamStripeDetector.h
        include/stripe/StegerStripeDetector.h
        include/stripe/StripeDetectionTypes.h
        include/stripe/StripeLineRepairTools.h
        include/stripe/StripeResponseEvaluationTools.h
        include/stripe/SubpixelCenterlineTools.h
        include/stripe/WeldSkeletonFeatureTools.h
        include/track/FilterKalman.h
        include/track/KcfTracker.h
        include/track/LegacyOnlineKcfDetector.h
        include/track/Tracker.h
        include/track/KcfFftTools.h
        include/track/KcfFhog.h
        include/track/KcfLabData.h
        include/track/KcfRectTools.h
    )
endif()

add_library(i2r_rlc_vision STATIC
    ${I2R_RLC_VISION_SOURCES}
)

target_include_directories(i2r_rlc_vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(i2r_rlc_vision
    PUBLIC
        Eigen3::Eigen
        i2r_rlc_digital_twin_types
        i2r_rlc_robot_comm
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        tinysplinecxx::tinysplinecxx
)

target_include_directories(i2r_rlc_vision
    PRIVATE
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(i2r_rlc_vision
    PRIVATE
        ${PCL_LIBRARIES}
)
target_compile_options(i2r_rlc_vision
    PRIVATE
        ${I2R_RLC_PCL_DEFINITIONS}
)

if(I2R_RLC_ENABLE_OPENCV)
    target_compile_definitions(i2r_rlc_vision
        PUBLIC
            I2R_RLC_HAS_OPENCV
    )
    target_include_directories(i2r_rlc_vision
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(i2r_rlc_vision
        PUBLIC
            ${OpenCV_LIBS}
        PRIVATE
            i2r_rlc_shape_based_matching
    )
endif()

i2r_rlc_apply_common_options(i2r_rlc_vision)

