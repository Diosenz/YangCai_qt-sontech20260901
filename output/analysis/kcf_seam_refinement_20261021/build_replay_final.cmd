@echo off
call D:\vs2022\VC\Auxiliary\Build\vcvars64.bat >nul
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\cmake.exe --build out\build\qt-debug --parallel 4 --target i2r_rlc_online_seam_refiner_replay_tests
exit /b %errorlevel%