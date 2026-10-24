@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_raw_reacquisition_repair\developer_environment_green.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
cd /d "H:\YangCai_Code\YangCai_qt-sontech20260901"
"D:\cmake\bin\cmake.exe" --build "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --parallel 4 --target i2r_rlc i2r_rlc_online_image_geometry_tests i2r_rlc_online_seam_continuity_tests i2r_rlc_simulation_guided_seam_tests i2r_rlc_online_seam_refiner_tests i2r_rlc_runtime_tracking_flow_tests i2r_rlc_online_fusion_controls_tests i2r_rlc_online_correction_run_recorder_tests i2r_rlc_correction_robot_sim_online_frame_sync_tests i2r_rlc_online_correction_tests i2r_rlc_local_registration_writeback_safety_tests i2r_rlc_online_local_surface_verification_tests > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_raw_reacquisition_repair\build_green.log" 2>&1
set "reacquire_build_result=%errorlevel%"
> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_raw_reacquisition_repair\build_green.exitcode.txt" echo %reacquire_build_result%
exit /b %reacquire_build_result%
