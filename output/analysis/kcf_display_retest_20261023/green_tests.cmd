@echo off
call D:\vs2022\Common7\Tools\VsDevCmd.bat -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%
ctest --test-dir out/build/qt-debug -C Debug --output-on-failure --timeout 60 -R "^i2r_rlc_online_control_target_presentation_tests$" > output/analysis/kcf_display_retest_20261023/green_tests.log 2>&1
exit /b %errorlevel%
