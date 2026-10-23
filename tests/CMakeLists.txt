find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(cppzmq CONFIG REQUIRED)
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
endif()

if(I2R_RLC_ENABLE_ONNX_RUNTIME)
    find_package(onnxruntime CONFIG REQUIRED)
endif()

function(i2r_rlc_add_test target_name)
    add_executable(${target_name}
        ${ARGN}
    )
    set_target_properties(${target_name}
        PROPERTIES
            AUTOMOC OFF
    )
    i2r_rlc_apply_common_options(${target_name})
    add_test(
        NAME ${target_name}
        COMMAND ${target_name}
    )
    i2r_rlc_apply_opencv_test_runtime_path(${target_name})
endfunction()

function(i2r_rlc_apply_opencv_test_runtime_path target_name)
    if(NOT WIN32)
        return()
    endif()

    set(runtime_path_updates)
    foreach(prefix_path IN LISTS CMAKE_PREFIX_PATH)
        if(EXISTS "${prefix_path}/bin")
            list(APPEND runtime_path_updates
                "PATH=path_list_prepend:${prefix_path}/bin"
            )
        endif()
    endforeach()

    if(DEFINED OpenCV_DIR)
        get_filename_component(opencv_vc_dir "${OpenCV_DIR}/.." ABSOLUTE)
        if(EXISTS "${opencv_vc_dir}/bin")
            list(APPEND runtime_path_updates
                "PATH=path_list_prepend:${opencv_vc_dir}/bin"
            )
        endif()
    endif()

    if(DEFINED VCPKG_TARGET_TRIPLET)
        set(vcpkg_runtime_dir
            "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/bin"
        )
        if(EXISTS "${vcpkg_runtime_dir}")
            list(APPEND runtime_path_updates
                "PATH=path_list_prepend:${vcpkg_runtime_dir}"
            )
        endif()
    endif()

    if(runtime_path_updates)
        set_tests_properties(${target_name}
            PROPERTIES
                ENVIRONMENT_MODIFICATION "${runtime_path_updates}"
        )
    endif()
endfunction()

function(i2r_rlc_apply_target_runtime_path test_name runtime_target)
    if(NOT WIN32)
        return()
    endif()

    get_target_property(runtime_dirs ${runtime_target} I2R_RLC_RUNTIME_DIRS)
    if(NOT runtime_dirs)
        return()
    endif()

    get_property(runtime_path_updates
        TEST ${test_name}
        PROPERTY ENVIRONMENT_MODIFICATION
    )
    foreach(runtime_dir IN LISTS runtime_dirs)
        if(EXISTS "${runtime_dir}")
            list(APPEND runtime_path_updates
                "PATH=path_list_prepend:${runtime_dir}"
            )
        endif()
    endforeach()
    get_target_property(target_environment_modifications
        ${runtime_target}
        I2R_RLC_ENVIRONMENT_MODIFICATION
    )
    if(target_environment_modifications)
        list(APPEND runtime_path_updates
            ${target_environment_modifications}
        )
    endif()

    if(runtime_path_updates)
        set_tests_properties(${test_name}
            PROPERTIES
                ENVIRONMENT_MODIFICATION "${runtime_path_updates}"
        )
    endif()
endfunction()

i2r_rlc_add_test(i2r_rlc_config_tests
    test_sensor_calibration_parser.cpp
)
target_link_libraries(i2r_rlc_config_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_offline_camera_tests
    test_offline_image_camera_device.cpp
)
target_link_libraries(i2r_rlc_offline_camera_tests
    PRIVATE
        i2r_rlc_camera
)

i2r_rlc_add_test(i2r_rlc_online_correction_tests
    test_online_correction_controller.cpp
)
target_link_libraries(i2r_rlc_online_correction_tests
    PRIVATE
        i2r_rlc_control
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_online_correction_tests)

add_test(
    NAME i2r_rlc_online_correction_p001_contract_tests
    COMMAND i2r_rlc_online_correction_tests --p001-contract
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_online_correction_p001_contract_tests)

i2r_rlc_add_test(i2r_rlc_online_correction_coordinate_contract_tests
    test_online_correction_coordinate_contract.cpp
)
target_link_libraries(i2r_rlc_online_correction_coordinate_contract_tests
    PRIVATE i2r_rlc_ui
)

add_test(NAME i2r_rlc_online_calibration_contract_tests
    COMMAND i2r_rlc_runtime_tracking_flow_tests --calibration-contract
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_online_calibration_contract_tests)

i2r_rlc_add_test(i2r_rlc_online_correction_p001_presentation_tests
    test_online_correction_p001_presentation.cpp
)
target_link_libraries(i2r_rlc_online_correction_p001_presentation_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_correction_run_recorder_tests
    test_online_correction_run_recorder.cpp
)
target_link_libraries(i2r_rlc_online_correction_run_recorder_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_correction_initialization_tests
    test_online_correction_initialization.cpp
)
target_link_libraries(i2r_rlc_online_correction_initialization_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_correction_regression_tests
    test_online_correction_regression.cpp
)
target_compile_definitions(i2r_rlc_online_correction_regression_tests
    PRIVATE
        I2R_RLC_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(i2r_rlc_online_correction_regression_tests
    PRIVATE
        i2r_rlc_control
)

i2r_rlc_add_test(i2r_rlc_tracking_trigger_policy_tests
    test_tracking_trigger_policy.cpp
)
target_link_libraries(i2r_rlc_tracking_trigger_policy_tests
    PRIVATE
        i2r_rlc_control
)

i2r_rlc_add_test(i2r_rlc_automatic_calibration_pose_tests
    test_automatic_calibration_pose.cpp
)
target_link_libraries(i2r_rlc_automatic_calibration_pose_tests
    PRIVATE
        i2r_rlc_control
)

i2r_rlc_add_test(i2r_rlc_correction_robot_start_gate_tests
    test_correction_robot_start_gate.cpp
)
target_link_libraries(i2r_rlc_correction_robot_start_gate_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_teach_trajectory_recorder_tests
    test_teach_trajectory_recorder.cpp
)
target_link_libraries(i2r_rlc_teach_trajectory_recorder_tests
    PRIVATE
        i2r_rlc_control
)

i2r_rlc_add_test(i2r_rlc_runtime_tracking_flow_tests
    test_runtime_tracking_flow.cpp
)
target_link_libraries(i2r_rlc_runtime_tracking_flow_tests
    PRIVATE
        i2r_rlc_ui
)

add_test(NAME i2r_rlc_online_seam_refinement_contract_tests
    COMMAND i2r_rlc_runtime_tracking_flow_tests --seam-refinement-contract)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_online_seam_refinement_contract_tests)

i2r_rlc_add_test(i2r_rlc_online_fusion_controls_tests
    test_online_fusion_controls.cpp
)
target_link_libraries(i2r_rlc_online_fusion_controls_tests PRIVATE i2r_rlc_vision)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_online_fusion_controls_tests)

i2r_rlc_add_test(i2r_rlc_online_local_registration_tests
    test_online_local_registration.cpp
    test_online_local_registration_history_collection.cpp
)
target_link_libraries(i2r_rlc_online_local_registration_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_local_registration_frame_lifecycle_tests
    test_online_local_registration_frame_lifecycle.cpp)
target_link_libraries(i2r_rlc_online_local_registration_frame_lifecycle_tests PRIVATE i2r_rlc_ui)

i2r_rlc_add_test(i2r_rlc_online_control_target_presentation_tests
    test_online_control_target_presentation.cpp)
target_link_libraries(i2r_rlc_online_control_target_presentation_tests PRIVATE i2r_rlc_ui)

i2r_rlc_add_test(i2r_rlc_online_local_registration_surface_tests
    test_online_local_registration_surface.cpp
)
target_link_libraries(i2r_rlc_online_local_registration_surface_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_local_registration_safety_tests
    test_online_local_registration_safety.cpp
)
target_link_libraries(i2r_rlc_online_local_registration_safety_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_local_registration_stability_tests
    test_online_local_registration_stability.cpp
)
target_link_libraries(i2r_rlc_online_local_registration_stability_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_local_registration_history_replay_tests
    test_online_local_registration_history_replay.cpp
)
target_link_libraries(i2r_rlc_online_local_registration_history_replay_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_local_registration_verification_tests
    test_online_local_registration_verification.cpp
)
target_link_libraries(i2r_rlc_online_local_registration_verification_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_online_local_surface_verification_tests
    test_online_local_surface_verification.cpp
)
target_include_directories(i2r_rlc_online_local_surface_verification_tests
    PRIVATE ${CMAKE_SOURCE_DIR}/UI/include
)
target_link_libraries(i2r_rlc_online_local_surface_verification_tests
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
)

i2r_rlc_add_test(i2r_rlc_local_registration_writeback_safety_tests
    test_local_registration_writeback_safety.cpp
)
target_link_libraries(i2r_rlc_local_registration_writeback_safety_tests
    PRIVATE i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_runtime_live_camera_flow_tests
    test_runtime_live_camera_flow.cpp
)
target_link_libraries(i2r_rlc_runtime_live_camera_flow_tests
    PRIVATE
        i2r_rlc_ui
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_runtime_tracking_flow_tests)

i2r_rlc_add_test(i2r_rlc_correction_robot_sim_offline_frame_sync_tests
    test_correction_robot_sim_frame_sync.cpp
)
target_link_libraries(i2r_rlc_correction_robot_sim_offline_frame_sync_tests
    PRIVATE
        i2r_rlc_ui
)
i2r_rlc_apply_opencv_test_runtime_path(
    i2r_rlc_correction_robot_sim_offline_frame_sync_tests
)

# 在线/离线目标编译同一份契约夹具，避免两套测试逻辑随生产共享状态机漂移。
# CTest 依赖显式保证在线结果只有在离线门禁先通过后才具备验收意义。
i2r_rlc_add_test(i2r_rlc_correction_robot_sim_online_frame_sync_tests
    test_correction_robot_sim_frame_sync.cpp
)
target_compile_definitions(
    i2r_rlc_correction_robot_sim_online_frame_sync_tests
    PRIVATE
        I2R_RLC_FRAME_SYNC_DEFAULT_ONLINE=1
)
target_link_libraries(i2r_rlc_correction_robot_sim_online_frame_sync_tests
    PRIVATE
        i2r_rlc_ui
)
i2r_rlc_apply_opencv_test_runtime_path(
    i2r_rlc_correction_robot_sim_online_frame_sync_tests
)
set_tests_properties(
    i2r_rlc_correction_robot_sim_online_frame_sync_tests
    PROPERTIES
        DEPENDS i2r_rlc_correction_robot_sim_offline_frame_sync_tests
)

i2r_rlc_add_test(i2r_rlc_main_widget_panel_tests
    test_main_widget_panels.cpp
)
target_link_libraries(i2r_rlc_main_widget_panel_tests
    PRIVATE
        i2r_rlc_ui
)

if(TARGET i2r_rlc_camera_vendor)
    i2r_rlc_add_test(i2r_rlc_camera_vendor_smoke_tests
        test_camera_vendor_smoke.cpp
    )
    target_link_libraries(i2r_rlc_camera_vendor_smoke_tests
        PRIVATE
            i2r_rlc_camera_vendor
    )
    i2r_rlc_apply_target_runtime_path(
        i2r_rlc_camera_vendor_smoke_tests
        i2r_rlc_camera_vendor
    )
endif()

i2r_rlc_add_test(i2r_rlc_offline_correction_tests
    test_offline_correction.cpp
)
target_link_libraries(i2r_rlc_offline_correction_tests
    PRIVATE
        i2r_rlc_control
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_offline_correction_tests)

i2r_rlc_add_test(i2r_rlc_offline_runtime_worker_flow_tests
    test_offline_runtime_worker_flow.cpp
)
target_link_libraries(i2r_rlc_offline_runtime_worker_flow_tests
    PRIVATE
        i2r_rlc_ui
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_offline_runtime_worker_flow_tests)

i2r_rlc_add_test(i2r_rlc_calibration_capture_tests
    test_calibration_capture.cpp
)
target_link_libraries(i2r_rlc_calibration_capture_tests
    PRIVATE
        i2r_rlc_control
)
i2r_rlc_apply_opencv_test_runtime_path(i2r_rlc_calibration_capture_tests)

i2r_rlc_add_test(i2r_rlc_localization_tests
    test_localization_service.cpp
)
target_link_libraries(i2r_rlc_localization_tests
    PRIVATE
        i2r_rlc_localization
)
target_compile_definitions(i2r_rlc_localization_tests
    PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

i2r_rlc_add_test(i2r_rlc_localization_surface_validation_tests
    test_localization_surface_validation.cpp
)
target_link_libraries(i2r_rlc_localization_surface_validation_tests
    PRIVATE i2r_rlc_localization
)

i2r_rlc_add_test(i2r_rlc_ibvs_core_tests
    test_ibvs_core.cpp
)
target_link_libraries(i2r_rlc_ibvs_core_tests
    PRIVATE
        i2r_rlc_ibvs
)

i2r_rlc_add_test(i2r_rlc_ibvs_offline_replay_tests
    test_ibvs_offline_replay.cpp
)
target_link_libraries(i2r_rlc_ibvs_offline_replay_tests
    PRIVATE
        i2r_rlc_ibvs
)

i2r_rlc_add_test(i2r_rlc_ibvs_online_service_tests
    test_ibvs_online_service.cpp
)
target_link_libraries(i2r_rlc_ibvs_online_service_tests
    PRIVATE
        i2r_rlc_ibvs
)
target_compile_definitions(i2r_rlc_ibvs_online_service_tests
    PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

i2r_rlc_add_test(i2r_rlc_ibvs_sensor_fusion_tests
    test_ibvs_sensor_fusion.cpp
)
target_link_libraries(i2r_rlc_ibvs_sensor_fusion_tests
    PRIVATE
        i2r_rlc_ibvs
)

i2r_rlc_add_test(i2r_rlc_ibvs_text_config_reader_tests
    test_ibvs_text_config_reader.cpp
)
target_link_libraries(i2r_rlc_ibvs_text_config_reader_tests
    PRIVATE
        i2r_rlc_ibvs
)
target_compile_definitions(i2r_rlc_ibvs_text_config_reader_tests
    PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

i2r_rlc_add_test(i2r_rlc_ibvs_unified_velocity_solver_tests
    test_ibvs_unified_velocity_solver.cpp
)
target_link_libraries(i2r_rlc_ibvs_unified_velocity_solver_tests
    PRIVATE
        i2r_rlc_ibvs
)

if(I2R_RLC_ENABLE_OPENCV)
    i2r_rlc_add_test(i2r_rlc_opencv_smoke_tests
        test_opencv_smoke.cpp
    )
    target_include_directories(i2r_rlc_opencv_smoke_tests
        PRIVATE
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(i2r_rlc_opencv_smoke_tests
        PRIVATE
            ${OpenCV_LIBS}
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_opencv_tests
        test_ibvs_opencv.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_opencv_tests
        PRIVATE
            i2r_rlc_ibvs
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_image_tools_tests
        test_ibvs_image_tools.cpp
    )
    target_include_directories(i2r_rlc_ibvs_image_tools_tests
        PRIVATE
            ${CMAKE_SOURCE_DIR}/IBVS/src
    )
    target_link_libraries(i2r_rlc_ibvs_image_tools_tests
        PRIVATE
            i2r_rlc_ibvs
    )

    i2r_rlc_add_test(i2r_rlc_localization_laser_line_observation_tests
        test_localization_laser_line_observation.cpp
    )
    target_link_libraries(i2r_rlc_localization_laser_line_observation_tests
        PRIVATE
            i2r_rlc_localization
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_laser_line_observation_tests
        test_ibvs_laser_line_observation.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_laser_line_observation_tests
        PRIVATE
            i2r_rlc_ibvs
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_torch_feature_crops_tests
        test_ibvs_torch_feature_crops.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_torch_feature_crops_tests
        PRIVATE
            i2r_rlc_ibvs
    )
    target_compile_definitions(i2r_rlc_ibvs_torch_feature_crops_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_start_point_offline_tests
        test_ibvs_start_point_offline.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_start_point_offline_tests
        PRIVATE
            i2r_rlc_ibvs
    )
    target_compile_definitions(i2r_rlc_ibvs_start_point_offline_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_start_point_image_chain_tests
        test_ibvs_start_point_image_chain.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_start_point_image_chain_tests
        PRIVATE
            i2r_rlc_ibvs
    )
    target_compile_definitions(i2r_rlc_ibvs_start_point_image_chain_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_velocity_diagnostics_tests
        test_ibvs_velocity_diagnostics.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_velocity_diagnostics_tests
        PRIVATE
            i2r_rlc_ibvs
    )
    target_compile_definitions(i2r_rlc_ibvs_velocity_diagnostics_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_preprocessor_tests
        test_vision_image_preprocessor.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_preprocessor_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_denoise_tests
        test_vision_image_denoise.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_denoise_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_gabor_filter_tests
        test_vision_gabor_filter.cpp
    )
    target_link_libraries(i2r_rlc_vision_gabor_filter_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_laser_stripe_extractor_tests
        test_vision_laser_stripe_extractor.cpp
    )
    target_link_libraries(i2r_rlc_vision_laser_stripe_extractor_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_subpixel_centerline_tests
        test_vision_subpixel_centerline_tools.cpp
    )
    target_link_libraries(i2r_rlc_vision_subpixel_centerline_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_centerline_detector_tests
        test_vision_centerline_detector.cpp
    )
    target_link_libraries(i2r_rlc_vision_centerline_detector_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_centerline_replay_tests
        test_vision_centerline_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_centerline_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_centerline_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_vision_active_contour_stripe_zhixian_replay_tests
        test_vision_active_contour_stripe_zhixian_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_active_contour_stripe_zhixian_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_active_contour_stripe_zhixian_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_active_contour_stripe_zhixian_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_dp_stripe_detector_tests
        test_vision_dp_stripe_detector.cpp
    )
    target_link_libraries(i2r_rlc_vision_dp_stripe_detector_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_dp_stripe_detector_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_gap_feature_detector_tests
        test_vision_gap_feature_detector.cpp
    )
    target_link_libraries(i2r_rlc_vision_gap_feature_detector_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_algorithm_pipeline_tests
        test_vision_algorithm_pipeline.cpp
    )
    target_link_libraries(i2r_rlc_vision_algorithm_pipeline_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_algorithm_pipeline_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_stripe_sam_tests
        test_vision_stripe_sam.cpp
    )
    target_link_libraries(i2r_rlc_vision_stripe_sam_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_stripe_sam_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_stripe_isffa_tests
        test_vision_stripe_isffa.cpp
    )
    target_link_libraries(i2r_rlc_vision_stripe_isffa_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_stripe_isffa_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_stripe_steger_tests
        test_vision_stripe_steger.cpp
    )
    target_link_libraries(i2r_rlc_vision_stripe_steger_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_stripe_steger_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_start_point_tests
        test_vision_start_point.cpp
    )
    target_link_libraries(i2r_rlc_vision_start_point_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_start_point_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_quality_tests
        test_vision_image_quality.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_quality_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_image_quality_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_sw_image_pro_replay_tests
        test_vision_sw_image_pro_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_sw_image_pro_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_sw_image_pro_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_sw_image_pro_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_gap_feature_replay_tests
        test_vision_gap_feature_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_gap_feature_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_gap_feature_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_vision_gap_width_replay_tests
        test_vision_gap_width_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_gap_width_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_gap_width_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_gap_width_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_line_geometry_tests
        test_vision_line_geometry.cpp
    )
    target_link_libraries(i2r_rlc_vision_line_geometry_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_endpoint_detector_tests
        test_vision_endpoint_detector.cpp
    )
    target_link_libraries(i2r_rlc_vision_endpoint_detector_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_skeleton_feature_tests
        test_vision_skeleton_feature.cpp
    )
    target_link_libraries(i2r_rlc_vision_skeleton_feature_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_skeleton_feature_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_groove_edge_detector_tests
        test_vision_groove_edge_detector.cpp
    )
    target_link_libraries(i2r_rlc_vision_groove_edge_detector_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_feature_fusion_tests
        test_vision_feature_fusion.cpp
    )
    target_link_libraries(i2r_rlc_vision_feature_fusion_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_to_space_tests
        test_vision_image_to_space.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_to_space_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_image_to_space_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_penzui_station_projection_tests
        test_vision_penzui_station_projection.cpp
    )
    target_link_libraries(i2r_rlc_vision_penzui_station_projection_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_penzui_station_projection_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )

    i2r_rlc_add_test(i2r_rlc_vision_point_cloud_gap_width_tests
        test_vision_point_cloud_gap_width.cpp
    )
    target_link_libraries(i2r_rlc_vision_point_cloud_gap_width_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_vision_libicp_tests
        test_vision_libicp.cpp
    )
    target_link_libraries(i2r_rlc_vision_libicp_tests
        PRIVATE
            i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_online_seam_continuity_replay_tests
        test_online_seam_continuity_replay.cpp)
    target_link_libraries(i2r_rlc_online_seam_continuity_replay_tests PRIVATE i2r_rlc_vision)
    target_compile_definitions(i2r_rlc_online_seam_continuity_replay_tests PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    set_tests_properties(i2r_rlc_online_seam_continuity_replay_tests PROPERTIES SKIP_RETURN_CODE 77)

    i2r_rlc_add_test(i2r_rlc_online_seam_continuity_tests
        test_online_seam_continuity.cpp)
    target_link_libraries(i2r_rlc_online_seam_continuity_tests PRIVATE i2r_rlc_vision)

    i2r_rlc_add_test(i2r_rlc_online_image_measurement_tests
        test_online_image_measurement.cpp)
    target_link_libraries(i2r_rlc_online_image_measurement_tests PRIVATE i2r_rlc_vision)
    target_compile_definitions(i2r_rlc_online_image_measurement_tests PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

    i2r_rlc_add_test(i2r_rlc_online_image_geometry_tests
        test_online_image_geometry.cpp)
    target_link_libraries(i2r_rlc_online_image_geometry_tests PRIVATE i2r_rlc_vision)
    target_compile_definitions(i2r_rlc_online_image_geometry_tests PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

    i2r_rlc_add_test(i2r_rlc_simulation_guided_seam_tests
        test_simulation_guided_seam.cpp
    )
    target_link_libraries(i2r_rlc_simulation_guided_seam_tests
        PRIVATE i2r_rlc_vision
    )

    i2r_rlc_add_test(i2r_rlc_online_seam_refiner_tests
        test_online_seam_refiner.cpp
    )
    target_link_libraries(i2r_rlc_online_seam_refiner_tests PRIVATE i2r_rlc_vision)
    target_compile_definitions(i2r_rlc_online_seam_refiner_tests PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

    i2r_rlc_add_test(i2r_rlc_online_seam_refiner_replay_tests
        test_online_seam_refiner_replay.cpp
    )
    target_link_libraries(i2r_rlc_online_seam_refiner_replay_tests PRIVATE i2r_rlc_vision)
    target_compile_definitions(i2r_rlc_online_seam_refiner_replay_tests PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    set_tests_properties(i2r_rlc_online_seam_refiner_replay_tests PROPERTIES SKIP_RETURN_CODE 77)

    i2r_rlc_add_test(i2r_rlc_vision_kcf_tracker_tests
        test_vision_kcf_tracker.cpp
    )
    target_link_libraries(i2r_rlc_vision_kcf_tracker_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_kcf_tracker_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_kcf_penzui_replay_tests
        test_vision_kcf_penzui_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_kcf_penzui_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_kcf_penzui_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_kcf_penzui_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_legacy_kcf_service_replay_tests
        test_vision_legacy_kcf_service_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_legacy_kcf_service_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_legacy_kcf_service_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_legacy_kcf_service_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_pro_share_tools_tests
        test_vision_image_pro_share_tools.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_pro_share_tools_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_image_pro_share_tools_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_pro_share_replay_tests
        test_vision_image_pro_share_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_pro_share_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_image_pro_share_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_image_pro_share_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_pro_feng_tools_tests
        test_vision_image_pro_feng_tools.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_pro_feng_tools_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_image_pro_feng_tools_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_image_pro_feng_replay_tests
        test_vision_image_pro_feng_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_image_pro_feng_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_image_pro_feng_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_image_pro_feng_replay_tests
    )

    if(I2R_RLC_ENABLE_SHAPE_MATCHING_EXPERIMENT)
        i2r_rlc_add_test(i2r_rlc_vision_shape_matching_experiment_tests
            test_vision_shape_matching_experiment.cpp
        )
        target_link_libraries(i2r_rlc_vision_shape_matching_experiment_tests
            PRIVATE
                i2r_rlc_vision
        )
        target_compile_definitions(i2r_rlc_vision_shape_matching_experiment_tests
            PRIVATE
                I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        )
        i2r_rlc_apply_opencv_test_runtime_path(
            i2r_rlc_vision_shape_matching_experiment_tests
        )

        i2r_rlc_add_test(i2r_rlc_vision_skeleton_halcon_diff_experiment_tests
            test_vision_skeleton_halcon_diff_experiment.cpp
        )
        target_link_libraries(i2r_rlc_vision_skeleton_halcon_diff_experiment_tests
            PRIVATE
                i2r_rlc_vision
        )
        target_compile_definitions(i2r_rlc_vision_skeleton_halcon_diff_experiment_tests
            PRIVATE
                I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        )
        i2r_rlc_apply_opencv_test_runtime_path(
            i2r_rlc_vision_skeleton_halcon_diff_experiment_tests
        )
    endif()

    i2r_rlc_add_test(i2r_rlc_vision_active_contour_replay_tests
        test_vision_active_contour_replay.cpp
    )
    target_link_libraries(i2r_rlc_vision_active_contour_replay_tests
        PRIVATE
            i2r_rlc_vision
    )
    target_compile_definitions(i2r_rlc_vision_active_contour_replay_tests
        PRIVATE
            I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_active_contour_replay_tests
    )

    i2r_rlc_add_test(i2r_rlc_vision_active_contour_feature_tests
        test_vision_active_contour_feature.cpp
    )
    target_link_libraries(i2r_rlc_vision_active_contour_feature_tests
        PRIVATE
            i2r_rlc_vision
    )
    i2r_rlc_apply_opencv_test_runtime_path(
        i2r_rlc_vision_active_contour_feature_tests
    )
endif()

if(I2R_RLC_ENABLE_ONNX_RUNTIME)
    i2r_rlc_add_test(i2r_rlc_onnxruntime_smoke_tests
        test_onnxruntime_smoke.cpp
    )
    target_link_libraries(i2r_rlc_onnxruntime_smoke_tests
        PRIVATE
            onnxruntime::onnxruntime
    )

    i2r_rlc_add_test(i2r_rlc_ibvs_yolo_smoke_tests
        test_ibvs_yolo_smoke.cpp
    )
    target_link_libraries(i2r_rlc_ibvs_yolo_smoke_tests
        PRIVATE
            i2r_rlc_ibvs
    )
    target_compile_definitions(i2r_rlc_ibvs_yolo_smoke_tests
        PRIVATE
            I2R_RLC_IBVS_YOLO_MODEL_PATH="C:/Feature_detection/onnx_workpiece/best.onnx"
    )

    if(I2R_RLC_ENABLE_OPENCV)
        i2r_rlc_add_test(i2r_rlc_ibvs_image_chain_tests
            test_ibvs_image_chain.cpp
        )
        target_link_libraries(i2r_rlc_ibvs_image_chain_tests
            PRIVATE
                i2r_rlc_ibvs
        )
        target_compile_definitions(i2r_rlc_ibvs_image_chain_tests
            PRIVATE
                I2R_RLC_IBVS_YOLO_MODEL_PATH="C:/Feature_detection/onnx_workpiece/best.onnx"
                I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        )
    endif()
endif()

function(i2r_rlc_add_pcl_smoke_test target_name source_file)
    i2r_rlc_add_test(${target_name}
        ${source_file}
    )
    target_include_directories(${target_name}
        PRIVATE
            ${PCL_INCLUDE_DIRS}
    )
    target_link_libraries(${target_name}
        PRIVATE
            ${PCL_LIBRARIES}
    )
    target_compile_options(${target_name}
        PRIVATE
            ${I2R_RLC_PCL_DEFINITIONS}
    )
endfunction()

i2r_rlc_add_pcl_smoke_test(i2r_rlc_pcl_core_io_filter_tests test_pcl_smoke.cpp)
i2r_rlc_add_pcl_smoke_test(i2r_rlc_pcl_features_segmentation_tests test_pcl_features_segmentation.cpp)
i2r_rlc_add_pcl_smoke_test(i2r_rlc_pcl_registration_tests test_pcl_registration.cpp)
i2r_rlc_add_pcl_smoke_test(i2r_rlc_pcl_surface_keypoints_tests test_pcl_surface_keypoints.cpp)
i2r_rlc_add_pcl_smoke_test(i2r_rlc_pcl_localization_icp_compat_tests test_pcl_localization_icp_compat.cpp)

i2r_rlc_add_test(i2r_rlc_zeromq_smoke_tests
    test_zeromq_smoke.cpp
)
target_link_libraries(i2r_rlc_zeromq_smoke_tests
    PRIVATE
        cppzmq
)

i2r_rlc_add_test(i2r_rlc_robot_comm_client_tests
    test_robot_comm_client_smoke.cpp
)
target_link_libraries(i2r_rlc_robot_comm_client_tests
    PRIVATE
        i2r_rlc_robot_comm
)

i2r_rlc_add_test(i2r_rlc_robot_protocol_codec_tests
    test_robot_protocol_codec.cpp
)
target_link_libraries(i2r_rlc_robot_protocol_codec_tests
    PRIVATE
        i2r_rlc_robot_comm
)

i2r_rlc_add_test(i2r_rlc_robot_legacy_commands_tests
    test_robot_legacy_commands.cpp
)
target_link_libraries(i2r_rlc_robot_legacy_commands_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_robot_sim_virtual_reference_cache_tests
    test_robot_sim_virtual_reference_cache.cpp
)
target_link_libraries(i2r_rlc_robot_sim_virtual_reference_cache_tests
    PRIVATE
        i2r_rlc_ui
)

i2r_rlc_add_test(i2r_rlc_mh12_inverse_kinematics_tests
    test_mh12_inverse_kinematics.cpp
    ${CMAKE_SOURCE_DIR}/DigitalTwin/src/MotomanMh12InverseKinematics.cpp
)
target_include_directories(i2r_rlc_mh12_inverse_kinematics_tests
    PRIVATE ${CMAKE_SOURCE_DIR}/DigitalTwin/include
)
target_link_libraries(i2r_rlc_mh12_inverse_kinematics_tests PRIVATE Eigen3::Eigen)

i2r_rlc_add_test(i2r_rlc_digital_twin_tests
    test_digital_twin_service.cpp
)
target_link_libraries(i2r_rlc_digital_twin_tests
    PRIVATE
        i2r_rlc_digital_twin
)
target_compile_definitions(i2r_rlc_digital_twin_tests
    PRIVATE
        I2R_RLC_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_executable(i2r_rlc_robotsim_probe
    RobotSimProbe.cpp
)
set_target_properties(i2r_rlc_robotsim_probe
    PROPERTIES
        AUTOMOC OFF
)
i2r_rlc_apply_common_options(i2r_rlc_robotsim_probe)
target_link_libraries(i2r_rlc_robotsim_probe
    PRIVATE
        i2r_rlc_digital_twin
)
