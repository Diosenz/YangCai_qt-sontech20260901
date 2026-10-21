@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -no_logo -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%
"D:\cmake\bin\cmake.exe" --build "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --target i2r_rlc_mh12_inverse_kinematics_tests i2r_rlc_digital_twin_tests --parallel 2
if errorlevel 1 exit /b %errorlevel%
"D:\cmake\bin\ctest.exe" --test-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" -R "^(i2r_rlc_mh12_inverse_kinematics_tests|i2r_rlc_digital_twin_tests)$" --output-on-failure --timeout 30
exit /b %errorlevel%
