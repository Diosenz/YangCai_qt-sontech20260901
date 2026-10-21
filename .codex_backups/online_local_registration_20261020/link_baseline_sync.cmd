@echo off
call "D:\vs2022\VC\Auxiliary\Build\vcvars64.bat"
link.exe @"H:\YangCai_Code\YangCai_qt-sontech20260901\.codex_backups\online_local_registration_20261020\baseline_offline.rsp"
if errorlevel 1 exit /b 1
link.exe @"H:\YangCai_Code\YangCai_qt-sontech20260901\.codex_backups\online_local_registration_20261020\baseline_online.rsp"
if errorlevel 1 exit /b 1
