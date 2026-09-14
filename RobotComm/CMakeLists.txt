find_package(cppzmq CONFIG REQUIRED)
find_package(concurrentqueue CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

add_library(i2r_rlc_robot_comm STATIC
    src/Client.cpp
    src/ClientQueue.cpp
    src/RobotCommandTransport.cpp
    src/RobotProtocolCodec.cpp
    include/CalibrationCommand.h
    include/JobCommand.h
    include/Client.h
    include/ClientQueue.h
    include/Config.h
    include/OnlineCorrectionCommand.h
    include/RobotCommandTransport.h
    include/RobotProtocolCodec.h
    include/RobotState.h
    include/RobotStateSnap.h
)

set_target_properties(i2r_rlc_robot_comm
    PROPERTIES
        AUTOMOC OFF
)

target_compile_features(i2r_rlc_robot_comm
    PUBLIC
        cxx_std_23
)

target_include_directories(i2r_rlc_robot_comm
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(i2r_rlc_robot_comm
    PUBLIC
        concurrentqueue::concurrentqueue
        cppzmq
        cppzmq-static
        fmt::fmt
    PRIVATE
        magic_enum::magic_enum
)

i2r_rlc_apply_common_options(i2r_rlc_robot_comm)
