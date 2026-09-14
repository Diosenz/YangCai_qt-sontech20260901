cmake_minimum_required(VERSION 3.20)

project(i2r_rlc
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

option(I2R_RLC_ENABLE_OPENCV "Enable OpenCV-based vision utilities" ON)
option(I2R_RLC_ENABLE_ONNX_RUNTIME "Enable ONNX Runtime based IBVS YOLO detector" OFF)
option(I2R_RLC_ENABLE_TESTS "Build i2r_rlc tests" ON)
option(I2R_RLC_ENABLE_SHAPE_MATCHING_EXPERIMENT
    "Build isolated penzui HALCON shape/skeleton comparison experiment targets" OFF)

function(i2r_rlc_apply_common_options target_name)
    target_compile_options(${target_name}
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/EHsc>
            $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
            $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
            $<$<CXX_COMPILER_ID:MSVC>:/external:anglebrackets>
            $<$<CXX_COMPILER_ID:MSVC>:/external:W0>
            $<$<CXX_COMPILER_ID:MSVC>:/analyze:external->
    )
endfunction()

add_subdirectory(Camera)
add_subdirectory(Localization)
add_subdirectory(DigitalTwin)
add_subdirectory(Vision)
add_subdirectory(RobotComm)
add_subdirectory(Control)
add_subdirectory(IBVS)
add_subdirectory(UI)

if(I2R_RLC_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
