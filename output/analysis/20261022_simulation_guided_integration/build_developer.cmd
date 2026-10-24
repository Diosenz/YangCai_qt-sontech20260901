@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_simulation_guided_integration\developer_environment.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
set "PATH=D:\cmake\bin;D:\QT\6.7.3\msvc2022_64\bin;E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;%PATH%"
"D:\cmake\bin\cmake.exe" --build "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --parallel 4 --target i2r_rlc i2r_rlc_simulation_guided_seam_tests i2r_rlc_localization_surface_validation_tests i2r_rlc_online_local_surface_verification_tests i2r_rlc_online_local_registration_surface_tests i2r_rlc_online_local_registration_tests i2r_rlc_online_local_registration_safety_tests i2r_rlc_online_local_registration_stability_tests i2r_rlc_online_local_registration_history_replay_tests i2r_rlc_online_local_registration_verification_tests i2r_rlc_local_registration_writeback_safety_tests i2r_rlc_online_correction_run_recorder_tests i2r_rlc_runtime_tracking_flow_tests i2r_rlc_correction_robot_sim_online_frame_sync_tests i2r_rlc_correction_robot_sim_offline_frame_sync_tests i2r_rlc_online_seam_refiner_tests i2r_rlc_online_seam_continuity_tests > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_simulation_guided_integration\build.log" 2>&1
exit /b %errorlevel%

