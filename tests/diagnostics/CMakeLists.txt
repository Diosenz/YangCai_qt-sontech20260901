# 离线工具只编译实际使用的算法源，不链接设备服务或启动生产流程。
if(NOT I2R_RLC_ENABLE_OPENCV)
    message(FATAL_ERROR "KCF diagnostics require I2R_RLC_ENABLE_OPENCV=ON")
endif()
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

set(replay_dir "${CMAKE_CURRENT_SOURCE_DIR}/online_detector_replay")
set(correction_dir "${CMAKE_CURRENT_SOURCE_DIR}/online_correction_repro")
set(vision_dir "${PROJECT_SOURCE_DIR}/Vision")
set(replay_output "${CMAKE_CURRENT_BINARY_DIR}/native/$<CONFIG>")
function(i2r_rlc_add_replay target_name output_name)
    add_executable(${target_name} ${ARGN})
    set_target_properties(${target_name} PROPERTIES AUTOMOC OFF OUTPUT_NAME "${output_name}"
        RUNTIME_OUTPUT_DIRECTORY "${replay_output}")
    foreach(module IN ITEMS Vision DigitalTwin Control RobotComm Localization UI)
        target_include_directories(${target_name} PRIVATE "${PROJECT_SOURCE_DIR}/${module}/include")
    endforeach()
    target_include_directories(${target_name} PRIVATE "${vision_dir}/third_party")
    target_link_libraries(${target_name} PRIVATE Qt${QT_VERSION_MAJOR}::Core Eigen3::Eigen)
    i2r_rlc_apply_common_options(${target_name})
endfunction()

set(detector_sources)
foreach(name IN ITEMS
    track/LegacyOnlineKcfDetector track/KcfTracker track/KcfFhog track/KcfFftTools
    track/KcfRectTools track/FilterKalman track/OnlineSeamContinuity track/OnlineSeamEvidence
    feature/OnlineSeamRefiner feature/SimulationGuidedSeamEstimator
    geometry/WeldImageToSpaceProjector preprocess/ImageDenoise
    stripe/DpStripeCenterlineDetector stripe/SubpixelCenterlineTools)
    list(APPEND detector_sources "${vision_dir}/src/${name}.cpp")
endforeach()
i2r_rlc_add_replay(i2r_rlc_kcf_detector_replay DetectorReplay
    "${replay_dir}/DetectorReplay.cpp" "${replay_dir}/GapDisabledAdapter.cpp"
    "${vision_dir}/third_party/libicp/matrix.cpp" ${detector_sources})
i2r_rlc_add_replay(i2r_rlc_kcf_identity_evidence IdentityEvidence "${replay_dir}/IdentityEvidence.cpp")
i2r_rlc_add_replay(i2r_rlc_kcf_mother_observation MotherObservation "${replay_dir}/MotherObservation.cpp")
i2r_rlc_add_replay(i2r_rlc_kcf_refiner_replay RefinerReplay
    "${correction_dir}/RefinerReplay.cpp" "${vision_dir}/src/feature/OnlineSeamRefiner.cpp"
    "${vision_dir}/src/stripe/SubpixelCenterlineTools.cpp")
i2r_rlc_add_replay(i2r_rlc_kcf_surface_replay SurfaceReplay
    "${correction_dir}/SurfaceReplay.cpp"
    "${PROJECT_SOURCE_DIR}/Localization/src/LocalizationSurfaceValidation.cpp")
set(icp_sources)
foreach(name IN ITEMS icp icpPointToPoint icpPointToPlane kdtree matrix)
    list(APPEND icp_sources "${vision_dir}/third_party/libicp/${name}.cpp")
endforeach()
i2r_rlc_add_replay(i2r_rlc_kcf_control_replay ControlReplay
    "${replay_dir}/ControlReplay.cpp"
    "${PROJECT_SOURCE_DIR}/Control/src/OnlineCorrectionPathFollower.cpp"
    "${PROJECT_SOURCE_DIR}/Control/src/OnlineCorrectionExceptionProcessor.cpp"
    "${vision_dir}/src/geometry/PositionerKinematics.cpp"
    "${vision_dir}/src/geometry/IcpRegistrationMatcher.cpp" ${icp_sources})
set(image_targets i2r_rlc_kcf_detector_replay i2r_rlc_kcf_identity_evidence
    i2r_rlc_kcf_mother_observation i2r_rlc_kcf_refiner_replay)
foreach(target_name IN LISTS image_targets)
    target_include_directories(${target_name} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBS})
endforeach()
set(replay_targets ${image_targets} i2r_rlc_kcf_surface_replay i2r_rlc_kcf_control_replay)

# 构建聚合目标后写出程序/源码哈希和运行库目录；Python 不自动触发生成。
set(source_files)
set(binary_entries)
set(runtime_entries "\"$<TARGET_FILE_DIR:Qt${QT_VERSION_MAJOR}::Core>\"")
foreach(target_name IN LISTS replay_targets)
    get_target_property(target_sources ${target_name} SOURCES)
    list(APPEND source_files ${target_sources})
    get_target_property(output_name ${target_name} OUTPUT_NAME)
    list(APPEND binary_entries "\"${output_name}\":\"$<TARGET_FILE_NAME:${target_name}>\"")
endforeach()
foreach(module IN ITEMS Vision DigitalTwin Control RobotComm Localization UI)
    file(GLOB_RECURSE module_headers CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/${module}/include/*.h")
    list(APPEND source_files ${module_headers})
endforeach()
file(GLOB_RECURSE icp_headers CONFIGURE_DEPENDS "${vision_dir}/third_party/libicp/*.h")
list(APPEND source_files ${icp_headers})
list(REMOVE_DUPLICATES source_files)
list(SORT source_files)
set(source_entries)
foreach(source_file IN LISTS source_files)
    file(RELATIVE_PATH relative_source "${PROJECT_SOURCE_DIR}" "${source_file}")
    list(APPEND source_entries "\"${relative_source}\"")
endforeach()
foreach(library IN LISTS OpenCV_LIBS)
    if(TARGET ${library})
        list(APPEND runtime_entries "\"$<TARGET_FILE_DIR:${library}>\"")
    endif()
endforeach()
if(OpenCV_DIR)
    get_filename_component(opencv_vc_dir "${OpenCV_DIR}/.." ABSOLUTE)
    if(EXISTS "${opencv_vc_dir}/bin")
        list(APPEND runtime_entries "\"${opencv_vc_dir}/bin\"")
    endif()
endif()
list(REMOVE_DUPLICATES runtime_entries)
list(JOIN source_entries "," source_json)
list(JOIN binary_entries "," binary_json)
list(JOIN runtime_entries "," runtime_json)
file(GENERATE OUTPUT "${replay_output}/replay-inputs.json" CONTENT
    "{\"source_root\":\"${PROJECT_SOURCE_DIR}\",\"source_files\":[${source_json}],\"binaries\":{${binary_json}},\"runtime_dirs\":[${runtime_json}],\"dependency_versions\":{\"Qt\":\"${Qt${QT_VERSION_MAJOR}Core_VERSION}\",\"OpenCV\":\"${OpenCV_VERSION}\",\"compiler\":\"${CMAKE_CXX_COMPILER_ID}\",\"compiler_version\":\"${CMAKE_CXX_COMPILER_VERSION}\",\"configuration\":\"$<CONFIG>\"}}")
add_custom_target(i2r_rlc_kcf_diagnostics ALL
    COMMAND "${CMAKE_COMMAND}" "-DREPLAY_DIRECTORY=${replay_output}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/WriteReplayManifest.cmake"
    DEPENDS ${replay_targets} VERBATIM)

find_package(Python3 3.10 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME i2r_rlc_kcf_replay_regression
        COMMAND "${Python3_EXECUTABLE}" -X utf8 -m unittest discover -s "${replay_dir}" -p "test_*.py")
    add_test(NAME i2r_rlc_kcf_causal_probes
        COMMAND "${Python3_EXECUTABLE}" -X utf8 "${correction_dir}/verify_replays.py")
    set_tests_properties(i2r_rlc_kcf_replay_regression i2r_rlc_kcf_causal_probes PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        ENVIRONMENT "I2R_KCF_REPLAY_BIN_DIR=${replay_output};PYTHONUTF8=1"
        LABELS "offline;kcf")
else()
    message(WARNING "Python3 not found: native programs build, but JSONL regression requires Python 3.10+")
endif()
