@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -no_logo -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%
set "QT_QPA_PLATFORM=offscreen"
"D:\cmake\bin\ctest.exe" --test-dir "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" -R "^i2r_rlc_(online_local_registration_stability|online_local_registration_history_replay|online_local_registration_safety|online_local_registration_verification|online_local_registration|main_widget_panel|local_registration_writeback_safety|online_correction_run_recorder|online_correction|online_fusion_controls|offline_correction|localization|digital_twin|runtime_tracking_flow)_tests$" --output-on-failure --timeout 90
exit /b %errorlevel%
