cmake_minimum_required(VERSION 3.20)

project(i2r_rlc
    VERSION 0.1.0
    LANGUAGES CXX
)

# MSVC 的 /showIncludes 文本必须与 Ninja 的解析前缀一致。只设置配置进程
# 的环境不足以约束后续 VS/Developer Shell 构建；launcher 固定每次编译的语言。
# project() 之后覆盖前缀，同时修复已有 build tree 缓存的其他语言探测结果。
if(MSVC AND CMAKE_GENERATOR MATCHES "Ninja")
    set(CMAKE_CXX_COMPILER_LAUNCHER
        "${CMAKE_COMMAND}" "-E" "env" "VSLANG=1033" ${CMAKE_CXX_COMPILER_LAUNCHER})
    set(CMAKE_CL_SHOWINCLUDES_PREFIX "Note: including file: ")
    set(CMAKE_CXX_CL_SHOWINCLUDES_PREFIX "Note: including file: ")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

option(I2R_RLC_ENABLE_OPENCV "Enable OpenCV-based vision utilities" ON)
option(I2R_RLC_ENABLE_ONNX_RUNTIME "Enable ONNX Runtime based IBVS YOLO detector" OFF)
option(I2R_RLC_ENABLE_TESTS "Build i2r_rlc tests" ON)
option(I2R_RLC_ENABLE_KCF_DIAGNOSTICS "Build offline KCF replay programs (no devices)" OFF)
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

if(I2R_RLC_ENABLE_KCF_DIAGNOSTICS AND NOT I2R_RLC_ENABLE_TESTS)
    message(FATAL_ERROR "KCF diagnostics require I2R_RLC_ENABLE_TESTS=ON")
endif()

if(I2R_RLC_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
