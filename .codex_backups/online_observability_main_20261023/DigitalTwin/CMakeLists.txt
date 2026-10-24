find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(cppzmq CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCASCADE REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_library(i2r_rlc_digital_twin_types INTERFACE)
target_include_directories(i2r_rlc_digital_twin_types
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(i2r_rlc_digital_twin_types
    INTERFACE
        Qt${QT_VERSION_MAJOR}::Core
)

add_library(i2r_rlc_digital_twin STATIC
    src/DigitalTwinLocalizationVirtualReferenceProvider.cpp
    src/WeldmentVirtualPointCloudGenerator.cpp
    src/WeldmentVirtualLaserPriorGenerator.cpp
    src/WeldmentLocalizationVirtualReferenceProvider.cpp
    src/NullDigitalTwinService.cpp
    src/RobotSimDigitalTwinService.cpp
    src/RobotSimEndpointConfigReader.cpp
    src/RobotSimSceneConfigReader.cpp
    src/RobotSimZmqTransport.cpp
    src/MotomanMh12InverseKinematics.cpp
    include/DigitalTwinVisionTypes.h
    include/DigitalTwinLocalizationVirtualReferenceProvider.h
    include/WeldmentVirtualPointCloudGenerator.h
    include/WeldmentVirtualLaserPriorGenerator.h
    include/WeldmentLocalizationVirtualReferenceProvider.h
    include/IDigitalTwinService.h
    include/IRobotSimTransport.h
    include/NullDigitalTwinService.h
    include/RobotSimDigitalTwinService.h
    include/RobotSimEndpointConfigReader.h
    include/RobotSimSceneConfigReader.h
    include/RobotSimZmqTransport.h
    include/MotomanMh12InverseKinematics.h
)

set_target_properties(i2r_rlc_digital_twin
    PROPERTIES
        AUTOMOC OFF
)

target_include_directories(i2r_rlc_digital_twin
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${OpenCASCADE_INCLUDE_DIRS}
)

target_link_libraries(i2r_rlc_digital_twin
    PUBLIC
        i2r_rlc_digital_twin_types
        i2r_rlc_localization
        Qt${QT_VERSION_MAJOR}::Core
    PRIVATE
        cppzmq
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        ${OpenCASCADE_LIBRARIES}
)

i2r_rlc_apply_common_options(i2r_rlc_digital_twin)
