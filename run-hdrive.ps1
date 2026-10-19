$ErrorActionPreference = "Stop"

$qtRoot = "D:\QT\6.7.3\msvc2022_64"

$env:Path = "$qtRoot\bin;$env:Path"
$env:QT_PLUGIN_PATH = "$qtRoot\plugins"

$exe = (Resolve-Path "$PSScriptRoot\out\build\qt-debug\UI\i2r_rlc.exe").Path

Write-Host "Starting:"
Write-Host $exe

Start-Process `
    -FilePath $exe `
    -WorkingDirectory (Split-Path $exe)
