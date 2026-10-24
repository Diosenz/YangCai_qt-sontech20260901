@echo off
call D:\vs2022\VC\Auxiliary\Build\vcvars64.bat >nul
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\cmake.exe --build out\build\qt-debug --parallel 4 --target i2r_rlc_correction_robot_sim_online_frame_sync_tests i2r_rlc_correction_robot_sim_offline_frame_sync_tests i2r_rlc_online_correction_run_recorder_tests
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\ctest.exe --test-dir out\build\qt-debug --output-on-failure -R "^i2r_rlc_(online_seam_refiner_tests|online_seam_refiner_replay_tests|online_seam_refinement_contract_tests|online_calibration_contract_tests|runtime_tracking_flow_tests|online_correction_run_recorder_tests|main_widget_panel_tests|online_local_registration_tests|online_local_registration_safety_tests|online_local_registration_verification_tests|online_fusion_controls_tests|correction_robot_sim_online_frame_sync_tests|correction_robot_sim_offline_frame_sync_tests)$"
exit /b %errorlevel%