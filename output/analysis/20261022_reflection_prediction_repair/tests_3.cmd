@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_reflection_prediction_repair\test_environment_3.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
cd /d "H:\YangCai_Code\YangCai_qt-sontech20260901"
"D:\cmake\bin\ctest.exe" --test-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --output-on-failure -R "^i2r_rlc_online_image_geometry_tests$" > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_reflection_prediction_repair\tests_3.log" 2>&1
set "reflection_test_result=%errorlevel%"
> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\20261022_reflection_prediction_repair\tests_3.exitcode.txt" echo %reflection_test_result%
exit /b %reflection_test_result%
