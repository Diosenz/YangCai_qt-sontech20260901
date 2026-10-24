@echo off
call D:\vs2022\Common7\Tools\VsDevCmd.bat -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%
cmake --build out\build\qt-debug --target i2r_rlc_online_control_target_presentation_tests --config Debug --parallel 3 > output\analysis\kcf_display_retest_20261023\green_build.log 2>&1
exit /b %errorlevel%
