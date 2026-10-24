@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\reacquisition_red_environment.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
cd /d "H:\YangCai_Code\YangCai_qt-sontech20260901"
"D:\cmake\bin\cmake.exe" --build "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --parallel 2 --target i2r_rlc_simulation_guided_seam_tests > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\reacquisition_red_build.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
"D:\cmake\bin\ctest.exe" --test-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --output-on-failure --timeout 60 -R "^i2r_rlc_simulation_guided_seam_tests$" > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\reacquisition_red_test.log" 2>&1
set "repair_red_exit=%errorlevel%"
> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_online_correction_repair\reacquisition_red.exitcode.txt" echo %repair_red_exit%
exit /b %repair_red_exit%
