@echo off
call "D:\vs2022\Common7\Tools\VsDevCmd.bat" -no_logo -arch=x64 -host_arch=x64
if errorlevel 1 exit /b %errorlevel%
"D:\cmake\bin\cmake.exe" --build "H:\YangCai_Code\YangCai_qt-sontech20260901\out\build\qt-debug" --target i2r_rlc i2r_rlc_online_local_registration_stability_tests i2r_rlc_online_local_registration_safety_tests i2r_rlc_online_local_registration_verification_tests i2r_rlc_online_local_registration_tests i2r_rlc_main_widget_panel_tests i2r_rlc_local_registration_writeback_safety_tests i2r_rlc_online_correction_tests i2r_rlc_online_fusion_controls_tests i2r_rlc_offline_correction_tests i2r_rlc_localization_tests i2r_rlc_digital_twin_tests --parallel 2
exit /b %errorlevel%
