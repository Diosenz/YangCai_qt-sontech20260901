@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\test_environment_2.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
cd /d "H:\YangCai_Code\YangCai_qt-sontech20260901"
set "PATH=D:\QT\6.7.3\msvc2022_64\bin;E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;H:\YangCai_Code\YangCai_qt-sontech20260901\vcpkg_installed\x64-windows\debug\bin;%PATH%"
"D:\cmake\bin\ctest.exe" --test-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --output-on-failure --timeout 60 --parallel 2 -R "^i2r_rlc_(online_seam_continuity|simulation_guided_seam|online_seam_refinement_contract|correction_robot_sim_online_frame_sync|online_correction_run_recorder|online_fusion_controls)_tests$" > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\tests_2.log" 2>&1
set "repair_test_exit=%errorlevel%"
> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\tests_2.exitcode.txt" echo %repair_test_exit%
"H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug\tests\i2r_rlc_localization_surface_validation_tests.exe" --replay-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\output\online_local_registration\20261022_073051_527_session_1" > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\native_surface_replay_2.jsonl" 2>&1
set "repair_replay_exit=%errorlevel%"
> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\native_surface_replay_2.exitcode.txt" echo %repair_replay_exit%
if not %repair_test_exit%==0 exit /b %repair_test_exit%
exit /b %repair_replay_exit%
