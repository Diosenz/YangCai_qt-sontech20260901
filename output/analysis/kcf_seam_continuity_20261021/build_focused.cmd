@echo off
call D:\vs2022\VC\Auxiliary\Build\vcvars64.bat
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\cmake.exe --build out\build\qt-debug --parallel 4 --target i2r_rlc_online_correction_tests i2r_rlc_runtime_tracking_flow_tests i2r_rlc_online_seam_continuity_tests
exit /b %errorlevel%