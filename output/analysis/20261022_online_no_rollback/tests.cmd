@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_no_rollback\test_environment.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
cd /d "H:\YangCai_Code\YangCai_qt-sontech20260901"
"D:\cmake\bin\ctest.exe" --test-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --output-on-failure --timeout 60 --parallel 2 -R "^i2r_rlc_(local_registration_writeback_safety|online_local_surface_verification|online_local_registration_surface|online_seam_refinement_contract|correction_robot_sim_online_frame_sync)_tests$" > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_no_rollback\tests.log" 2>&1
set "no_rollback_test_exit=%errorlevel%"
> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_no_rollback\tests.exitcode.txt" echo %no_rollback_test_exit%
exit /b %no_rollback_test_exit%
