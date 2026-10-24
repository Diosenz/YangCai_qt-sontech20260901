@echo off
call D:\vs2022\Common7\Tools\VsDevCmd.bat -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%
cd /d "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug"
cl @"H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\kcf_display_retest_20261023\probe_compile.rsp" > "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\kcf_display_retest_20261023\probe_build.log" 2>&1
if errorlevel 1 exit /b %errorlevel%
link @"H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\kcf_display_retest_20261023\probe_link.rsp" >> "H:\YangCai_Code\YangCai_qt-sontech20260901\output\analysis\kcf_display_retest_20261023\probe_build.log" 2>&1
exit /b %errorlevel%
