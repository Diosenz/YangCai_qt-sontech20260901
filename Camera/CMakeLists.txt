find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui)

option(I2R_RLC_ENABLE_VENDOR_CAMERAS
    "Enable vendor camera SDK backends in the Camera module"
    OFF
)
option(I2R_RLC_ENABLE_CAMERA_ALLIED "Enable Allied/Vimba X camera backend" OFF)
option(I2R_RLC_ENABLE_CAMERA_BASLER "Enable Basler/Pylon camera backend" OFF)

set(I2R_RLC_CAMERA_BUILD_ALLIED ${I2R_RLC_ENABLE_CAMERA_ALLIED})
set(I2R_RLC_CAMERA_BUILD_BASLER ${I2R_RLC_ENABLE_CAMERA_BASLER})
if(I2R_RLC_ENABLE_VENDOR_CAMERAS
    AND NOT I2R_RLC_ENABLE_CAMERA_ALLIED
    AND NOT I2R_RLC_ENABLE_CAMERA_BASLER)
    set(I2R_RLC_CAMERA_BUILD_ALLIED ON)
    set(I2R_RLC_CAMERA_BUILD_BASLER ON)
endif()
set(I2R_RLC_CAMERA_BUILD_VENDOR_BACKENDS OFF)
if(I2R_RLC_CAMERA_BUILD_ALLIED OR I2R_RLC_CAMERA_BUILD_BASLER)
    set(I2R_RLC_CAMERA_BUILD_VENDOR_BACKENDS ON)
endif()

add_library(i2r_rlc_camera STATIC
    src/CameraService.cpp
    src/OfflineImageCameraDevice.cpp
    include/AlliedCameraDevice.h
    include/BaslerCameraDevice.h
    include/CameraFrame.h
    include/CameraService.h
    include/ICameraDevice.h
    include/OfflineImageCameraDevice.h
)

target_include_directories(i2r_rlc_camera
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(i2r_rlc_camera
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
)

i2r_rlc_apply_common_options(i2r_rlc_camera)

if(I2R_RLC_CAMERA_BUILD_VENDOR_BACKENDS)
    if(NOT I2R_RLC_ENABLE_OPENCV)
        message(FATAL_ERROR
            "Vendor camera backends require I2R_RLC_ENABLE_OPENCV=ON."
        )
    endif()

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
            "OpenCV was not found. Vendor camera backends reuse the project "
            "OpenCV 3.4.x dependency; pass -DOpenCV_DIR=<path> or disable "
            "vendor camera backends."
        )
    endif()

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.0)
        message(FATAL_ERROR
            "OpenCV ${OpenCV_VERSION} was found, but vendor camera backends "
            "must follow the project OpenCV 3.4.x compatibility policy."
        )
    endif()

    find_package(fmt CONFIG REQUIRED)
    find_package(magic_enum CONFIG REQUIRED)

    set(i2r_rlc_camera_vendor_sources
        vendor/CommCamera/include/CameraCRTP.h
        vendor/CommCamera/include/Config.h
    )
    set(i2r_rlc_camera_vendor_libraries
        fmt::fmt
        magic_enum::magic_enum
        ${OpenCV_LIBS}
    )
    set(i2r_rlc_camera_vendor_include_dirs
        ${OpenCV_INCLUDE_DIRS}
    )
    set(i2r_rlc_camera_vendor_definitions)
    set(i2r_rlc_camera_vendor_private_definitions)
    set(i2r_rlc_camera_vendor_runtime_dirs)
    set(i2r_rlc_camera_vendor_environment_modifications)

    if(I2R_RLC_CAMERA_BUILD_ALLIED)
        if(DEFINED ENV{VIMBA_X_HOME})
            list(APPEND CMAKE_PREFIX_PATH "$ENV{VIMBA_X_HOME}/api")
            if(EXISTS "$ENV{VIMBA_X_HOME}/api/bin")
                list(APPEND i2r_rlc_camera_vendor_runtime_dirs
                    "$ENV{VIMBA_X_HOME}/api/bin"
                )
            endif()
        endif()
        if(EXISTS "C:/Program Files/Allied Vision/Vimba X/api")
            list(APPEND CMAKE_PREFIX_PATH
                "C:/Program Files/Allied Vision/Vimba X/api"
            )
        endif()
        if(EXISTS "C:/Program Files/Allied Vision/Vimba X/api/bin")
            list(APPEND i2r_rlc_camera_vendor_runtime_dirs
                "C:/Program Files/Allied Vision/Vimba X/api/bin"
            )
        endif()
        find_package(Vmb REQUIRED COMPONENTS CPP NAMES Vmb VmbC VmbCPP)
        list(APPEND i2r_rlc_camera_vendor_sources
            vendor/CommCamera/include/CameraAllied.h
            vendor/CommCamera/src/CameraAllied.cpp
            src/AlliedCameraDevice.cpp
        )
        list(APPEND i2r_rlc_camera_vendor_libraries Vmb::CPP)
        list(APPEND i2r_rlc_camera_vendor_definitions
            I2R_RLC_ENABLE_CAMERA_ALLIED=1
        )
    endif()

    if(I2R_RLC_CAMERA_BUILD_BASLER)
        set(i2r_rlc_pylon_cmake_dir)
        set(i2r_rlc_pylon_runtime_x64_dir)
        if(DEFINED ENV{PYLON_ROOT}
            AND EXISTS "$ENV{PYLON_ROOT}/Development/CMake")
            set(i2r_rlc_pylon_cmake_dir
                "$ENV{PYLON_ROOT}/Development/CMake"
            )
        elseif(DEFINED ENV{BASLER_PYLON_ROOT}
            AND EXISTS "$ENV{BASLER_PYLON_ROOT}/Development/CMake")
            set(i2r_rlc_pylon_cmake_dir
                "$ENV{BASLER_PYLON_ROOT}/Development/CMake"
            )
        elseif(EXISTS "C:/Program Files/Basler/pylon/Development/CMake")
            set(i2r_rlc_pylon_cmake_dir
                "C:/Program Files/Basler/pylon/Development/CMake"
            )
        endif()
        if(i2r_rlc_pylon_cmake_dir)
            list(APPEND CMAKE_PREFIX_PATH "${i2r_rlc_pylon_cmake_dir}")
            if(NOT pylon_DIR)
                set(pylon_DIR "${i2r_rlc_pylon_cmake_dir}"
                    CACHE PATH "Basler pylon CMake package directory"
                )
            endif()
        endif()
        if(DEFINED ENV{PYLON_ROOT}
            AND EXISTS "$ENV{PYLON_ROOT}/Runtime/x64")
            set(i2r_rlc_pylon_runtime_x64_dir
                "$ENV{PYLON_ROOT}/Runtime/x64"
            )
        elseif(DEFINED ENV{BASLER_PYLON_ROOT}
            AND EXISTS "$ENV{BASLER_PYLON_ROOT}/Runtime/x64")
            set(i2r_rlc_pylon_runtime_x64_dir
                "$ENV{BASLER_PYLON_ROOT}/Runtime/x64"
            )
        elseif(EXISTS "C:/Program Files/Basler/pylon/Runtime/x64")
            set(i2r_rlc_pylon_runtime_x64_dir
                "C:/Program Files/Basler/pylon/Runtime/x64"
            )
        endif()
        if(i2r_rlc_pylon_runtime_x64_dir)
            list(APPEND i2r_rlc_camera_vendor_runtime_dirs
                "${i2r_rlc_pylon_runtime_x64_dir}"
            )
            list(APPEND i2r_rlc_camera_vendor_environment_modifications
                "GENICAM_GENTL64_PATH=set:${i2r_rlc_pylon_runtime_x64_dir}"
            )
        endif()
        find_package(pylon REQUIRED)
        list(APPEND i2r_rlc_camera_vendor_sources
            vendor/CommCamera/include/CameraBasler.h
            vendor/CommCamera/src/CameraBasler.cpp
            src/BaslerCameraDevice.cpp
        )
        list(APPEND i2r_rlc_camera_vendor_libraries pylon::pylon)
        list(APPEND i2r_rlc_camera_vendor_definitions
            I2R_RLC_ENABLE_CAMERA_BASLER=1
        )
        list(APPEND i2r_rlc_camera_vendor_private_definitions
            PYLON_NO_AUTO_IMPLIB
            GENICAM_NO_AUTO_IMPLIB
        )
    endif()

    add_library(i2r_rlc_camera_vendor STATIC
        ${i2r_rlc_camera_vendor_sources}
    )
    set_target_properties(i2r_rlc_camera_vendor
        PROPERTIES
            AUTOMOC OFF
    )
    if(i2r_rlc_camera_vendor_runtime_dirs)
        list(REMOVE_DUPLICATES i2r_rlc_camera_vendor_runtime_dirs)
        set_target_properties(i2r_rlc_camera_vendor
            PROPERTIES
                I2R_RLC_RUNTIME_DIRS "${i2r_rlc_camera_vendor_runtime_dirs}"
        )
    endif()
    if(i2r_rlc_camera_vendor_environment_modifications)
        set_target_properties(i2r_rlc_camera_vendor
            PROPERTIES
                I2R_RLC_ENVIRONMENT_MODIFICATION
                "${i2r_rlc_camera_vendor_environment_modifications}"
        )
    endif()
    target_compile_features(i2r_rlc_camera_vendor
        PRIVATE
            cxx_std_23
    )
    target_include_directories(i2r_rlc_camera_vendor
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/vendor/CommCamera/compat
            ${CMAKE_CURRENT_SOURCE_DIR}/vendor/CommCamera/include
            ${i2r_rlc_camera_vendor_include_dirs}
    )
    target_compile_definitions(i2r_rlc_camera_vendor
        PUBLIC
            ${i2r_rlc_camera_vendor_definitions}
        PRIVATE
            ${i2r_rlc_camera_vendor_private_definitions}
    )
    target_link_libraries(i2r_rlc_camera_vendor
        PUBLIC
            i2r_rlc_camera
        PRIVATE
            ${i2r_rlc_camera_vendor_libraries}
    )
    i2r_rlc_apply_common_options(i2r_rlc_camera_vendor)
endif()
