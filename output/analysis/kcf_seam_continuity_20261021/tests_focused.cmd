@echo off
call D:\vs2022\VC\Auxiliary\Build\vcvars64.bat >nul
if errorlevel 1 exit /b %errorlevel%
D:\cmake\bin\ctest.exe --test-dir out\build\qt-debug --output-on-failure -R "^i2r_rlc_(online_correction_tests|runtime_tracking_flow_tests|online_seam_refinement_contract_tests|online_seam_continuity_tests)$"
exit /b %errorlevel%