@echo off
call D:\vs2022\VC\Auxiliary\Build\vcvars64.bat >nul
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\cmake.exe --build out\build\qt-debug --parallel 4 --target i2r_rlc_correction_robot_sim_online_frame_sync_tests
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\ctest.exe --test-dir out\build\qt-debug --output-on-failure -R "^i2r_rlc_correction_robot_sim_online_frame_sync_tests$"
exit /b %errorlevel%