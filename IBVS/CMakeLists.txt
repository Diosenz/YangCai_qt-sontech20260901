find_package(Eigen3 CONFIG REQUIRED)

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

    message(STATUS "IBVS OpenCV enabled: ${OpenCV_VERSION}")
    message(STATUS "IBVS OpenCV_DIR: ${OpenCV_DIR}")
endif()

if(I2R_RLC_ENABLE_ONNX_RUNTIME)
    find_package(onnxruntime CONFIG REQUIRED)
endif()

set(I2R_RLC_IBVS_SOURCES
    src/NullIbvsService.cpp
    src/IbvsFeatureGeometry.cpp
    src/IbvsJacobianEstimator.cpp
    src/IbvsOfflineDataReader.cpp
    src/IbvsOnlineService.cpp
    src/IbvsSensorFusion.cpp
    src/IbvsStartPointDetector.cpp
    src/IbvsTextConfigReader.cpp
    src/IbvsUnifiedVelocitySolver.cpp
    src/IbvsVelocitySolver.cpp
)

set(I2R_RLC_IBVS_HEADERS
    include/ibvs/IIbvsService.h
    include/ibvs/IbvsConfig.h
    include/ibvs/IbvsFeatureGeometry.h
    include/ibvs/IbvsJacobianEstimator.h
    include/ibvs/IbvsOfflineDataReader.h
    include/ibvs/IbvsOnlineService.h
    include/ibvs/IbvsSensorFusion.h
    include/ibvs/IbvsStartPointDetector.h
    include/ibvs/IbvsTextConfigReader.h
    include/ibvs/IbvsTypes.h
    include/ibvs/IbvsUnifiedVelocitySolver.h
    include/ibvs/IbvsVelocitySolver.h
    include/ibvs/NullIbvsService.h
)

if(I2R_RLC_ENABLE_OPENCV)
    list(APPEND I2R_RLC_IBVS_SOURCES
        src/IbvsColorBlobTools.cpp
        src/IbvsImageProjectionTools.cpp
        src/IbvsLaserLineObservation.cpp
        src/IbvsRedFeatureDetector.cpp
        src/IbvsStartPointImageDetector.cpp
        src/IbvsTemplateAlignmentTools.cpp
    )
    list(APPEND I2R_RLC_IBVS_HEADERS
        include/ibvs/IbvsLaserLineObservation.h
        include/ibvs/IbvsRedFeatureDetector.h
        include/ibvs/IbvsStartPointImageDetector.h
        src/IbvsColorBlobTools.h
        src/IbvsImageProjectionTools.h
        src/IbvsTemplateAlignmentTools.h
    )

    find_package(OpenMP QUIET)
endif()

if(I2R_RLC_ENABLE_ONNX_RUNTIME)
    if(NOT I2R_RLC_ENABLE_OPENCV)
        message(FATAL_ERROR
            "I2R_RLC_ENABLE_ONNX_RUNTIME requires I2R_RLC_ENABLE_OPENCV=ON."
        )
    endif()

    list(APPEND I2R_RLC_IBVS_SOURCES
        src/IbvsYoloDetector.cpp
    )
    list(APPEND I2R_RLC_IBVS_HEADERS
        include/ibvs/IbvsYoloDetector.h
    )
endif()

add_library(i2r_rlc_ibvs STATIC
    ${I2R_RLC_IBVS_SOURCES}
    ${I2R_RLC_IBVS_HEADERS}
)

set_target_properties(i2r_rlc_ibvs
    PROPERTIES
        AUTOMOC OFF
)

target_include_directories(i2r_rlc_ibvs
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(i2r_rlc_ibvs
    PUBLIC
        Eigen3::Eigen
)

if(I2R_RLC_ENABLE_OPENCV)
    target_include_directories(i2r_rlc_ibvs
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(i2r_rlc_ibvs
        PRIVATE
            ${OpenCV_LIBS}
    )
    target_compile_definitions(i2r_rlc_ibvs
        PUBLIC
            I2R_RLC_ENABLE_OPENCV=1
    )

    if(OpenMP_CXX_FOUND)
        target_link_libraries(i2r_rlc_ibvs
            PRIVATE
                OpenMP::OpenMP_CXX
        )
        target_compile_definitions(i2r_rlc_ibvs
            PRIVATE
                I2R_RLC_IBVS_USE_OPENMP=1
        )
    endif()
endif()

if(I2R_RLC_ENABLE_ONNX_RUNTIME)
    target_link_libraries(i2r_rlc_ibvs
        PRIVATE
            onnxruntime::onnxruntime
    )
    target_compile_definitions(i2r_rlc_ibvs
        PUBLIC
            I2R_RLC_ENABLE_ONNX_RUNTIME=1
    )
endif()

i2r_rlc_apply_common_options(i2r_rlc_ibvs)
