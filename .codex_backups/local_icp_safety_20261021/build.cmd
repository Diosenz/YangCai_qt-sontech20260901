@echo off
call "D:\vs2022\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 exit /b 1
set "PATH=D:\cmake\bin;D:\QT\6.7.3\msvc2022_64\bin;E:\Libs\OPENCV3\opencv\build\x64\vc15\bin;%PATH%"
"D:\cmake\bin\cmake.exe" --build "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --target i2r_rlc i2r_rlc_online_local_registration_tests i2r_rlc_online_local_registration_safety_tests i2r_rlc_online_local_registration_verification_tests i2r_rlc_local_registration_writeback_safety_tests --parallel 2
exit /b %errorlevel%