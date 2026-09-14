param(
    [string]$QtPrefixPath = "",
    [string]$OpenCV_DIR = "",
    [string]$BuildDir = "build",
    [string]$Configuration = "Debug",
    [switch]$RunBuildCheck,
    [switch]$DisableOpenCV
)

$ErrorActionPreference = "Stop"

$script:PassCount = 0
$script:WarnCount = 0
$script:FailCount = 0
$script:Suggestions = New-Object System.Collections.Generic.List[string]

function Write-Check {
    param(
        [ValidateSet("PASS", "WARN", "FAIL")]
        [string]$Status,
        [string]$Name,
        [string]$DetectedValue,
        [string]$Suggestion = ""
    )

    $line = "[{0}] {1}: {2}" -f $Status, $Name, $DetectedValue
    switch ($Status) {
        "PASS" {
            $script:PassCount++
            Write-Host $line -ForegroundColor Green
        }
        "WARN" {
            $script:WarnCount++
            Write-Host $line -ForegroundColor Yellow
        }
        "FAIL" {
            $script:FailCount++
            Write-Host $line -ForegroundColor Red
        }
    }

    if (-not [string]::IsNullOrWhiteSpace($Suggestion)) {
        $script:Suggestions.Add(("{0}: {1}" -f $Name, $Suggestion)) | Out-Null
    }
}

function Test-CommandExists {
    param(
        [string]$CommandName,
        [bool]$Required
    )

    $command = Get-Command $CommandName -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        Write-Check -Status "PASS" -Name $CommandName -DetectedValue $command.Source
        return $true
    }

    if ($Required) {
        Write-Check -Status "FAIL" -Name $CommandName -DetectedValue "not found in PATH" -Suggestion "Install it or run this script from Developer PowerShell for VS2022 with the tool in PATH."
    } else {
        Write-Check -Status "WARN" -Name $CommandName -DetectedValue "not found in PATH" -Suggestion "Optional tool. Install or add it to PATH if you want this workflow."
    }

    return $false
}

function Test-FileContains {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Name,
        [string]$Suggestion
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        Write-Check -Status "FAIL" -Name $Name -DetectedValue "CMakeLists.txt not found" -Suggestion "Run this script from newSontech or restore CMakeLists.txt."
        return $false
    }

    $content = Get-Content -Raw -LiteralPath $Path
    if ($content -match $Pattern) {
        Write-Check -Status "PASS" -Name $Name -DetectedValue "present"
        return $true
    }

    Write-Check -Status "FAIL" -Name $Name -DetectedValue "missing" -Suggestion $Suggestion
    return $false
}

function Test-PathEntryInProcessPath {
    param(
        [string]$PathEntry
    )

    if ([string]::IsNullOrWhiteSpace($PathEntry)) {
        return $false
    }

    $expectedPath = [System.IO.Path]::GetFullPath($PathEntry).TrimEnd('\')
    foreach ($pathPart in ($env:PATH -split ";")) {
        if ([string]::IsNullOrWhiteSpace($pathPart)) {
            continue
        }

        try {
            $candidatePath = [System.IO.Path]::GetFullPath($pathPart).TrimEnd('\')
            if ([string]::Equals($expectedPath, $candidatePath, [System.StringComparison]::OrdinalIgnoreCase)) {
                return $true
            }
        } catch {
            continue
        }
    }

    return $false
}

function Invoke-LoggedCommand {
    param(
        [string]$Name,
        [string[]]$CommandLine
    )

    Write-Host ""
    Write-Host ("Running {0}: {1}" -f $Name, ($CommandLine -join " ")) -ForegroundColor Cyan
    & $CommandLine[0] @($CommandLine[1..($CommandLine.Length - 1)])
    $exitCode = $LASTEXITCODE

    if ($exitCode -eq 0) {
        Write-Check -Status "PASS" -Name $Name -DetectedValue "exit code 0"
        return $true
    }

    Write-Check -Status "FAIL" -Name $Name -DetectedValue ("exit code {0}" -f $exitCode) -Suggestion "Read the CMake output above and fix the first configure/build error."
    return $false
}

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$cmakeListsPath = Join-Path $projectRoot "CMakeLists.txt"

$openCvEnabled = -not $DisableOpenCV
if ([string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
    if (-not [string]::IsNullOrWhiteSpace($env:OPENCV_DIR)) {
        $OpenCV_DIR = $env:OPENCV_DIR
    } elseif (-not [string]::IsNullOrWhiteSpace($env:OpenCV_DIR)) {
        $OpenCV_DIR = $env:OpenCV_DIR
    }
}

$openCvBin = ""
if (-not [string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
    $openCvBin = Join-Path (Split-Path -Parent $OpenCV_DIR) "bin"
}

Write-Host "newSontech environment check" -ForegroundColor Cyan
Write-Host ("Project root: {0}" -f $projectRoot)
Write-Host ("OpenCV:      {0}" -f ($(if ($openCvEnabled) { "ON" } else { "OFF" })))
Write-Host ("OpenCV_DIR:  {0}" -f ($(if ([string]::IsNullOrWhiteSpace($OpenCV_DIR)) { "<not set>" } else { $OpenCV_DIR })))
Write-Host ("OpenCV bin:  {0}" -f ($(if ([string]::IsNullOrWhiteSpace($openCvBin)) { "<not set>" } else { $openCvBin })))
Write-Host ""

$cmakeOk = Test-CommandExists -CommandName "cmake" -Required $true
$clOk = Test-CommandExists -CommandName "cl" -Required $true
$msbuildOk = Test-CommandExists -CommandName "msbuild" -Required $true
$ninjaOk = Test-CommandExists -CommandName "ninja" -Required $false
$gitOk = Test-CommandExists -CommandName "git" -Required $false

Write-Host ""
Write-Host "Qt compile-time checks" -ForegroundColor Cyan

$qtPrefixOk = $false
if ([string]::IsNullOrWhiteSpace($QtPrefixPath)) {
    Write-Check -Status "WARN" -Name "QtPrefixPath" -DetectedValue "not provided" -Suggestion "Pass -QtPrefixPath `"C:\Qt\6.x.x\msvc2022_64`" or set CMAKE_PREFIX_PATH when configuring."
} elseif (Test-Path -LiteralPath $QtPrefixPath -PathType Container) {
    $qtPrefixOk = $true
    Write-Check -Status "PASS" -Name "QtPrefixPath" -DetectedValue $QtPrefixPath
} else {
    Write-Check -Status "FAIL" -Name "QtPrefixPath" -DetectedValue $QtPrefixPath -Suggestion "Install Qt MSVC 2022 64-bit or pass the correct Qt prefix directory."
}

if ($qtPrefixOk) {
    $qt6Config = Join-Path $QtPrefixPath "lib\cmake\Qt6\Qt6Config.cmake"
    $qtCmake = Join-Path $QtPrefixPath "bin\qt-cmake.bat"
    $windeployqt = Join-Path $QtPrefixPath "bin\windeployqt.exe"
    $qtBin = Join-Path $QtPrefixPath "bin"

    if (Test-Path -LiteralPath $qt6Config -PathType Leaf) {
        Write-Check -Status "PASS" -Name "Qt6Config.cmake" -DetectedValue $qt6Config
    } else {
        Write-Check -Status "FAIL" -Name "Qt6Config.cmake" -DetectedValue "missing" -Suggestion "Verify that the Qt prefix is a Qt 6 MSVC kit directory."
    }

    if (Test-Path -LiteralPath $qtCmake -PathType Leaf) {
        Write-Check -Status "PASS" -Name "qt-cmake.bat" -DetectedValue $qtCmake
    } else {
        Write-Check -Status "WARN" -Name "qt-cmake.bat" -DetectedValue "missing" -Suggestion "Use regular cmake with -DCMAKE_PREFIX_PATH if qt-cmake.bat is unavailable."
    }

    if (Test-Path -LiteralPath $windeployqt -PathType Leaf) {
        Write-Check -Status "PASS" -Name "windeployqt.exe" -DetectedValue $windeployqt
    } else {
        Write-Check -Status "WARN" -Name "windeployqt.exe" -DetectedValue "missing" -Suggestion "Install Qt deployment tools or verify the Qt bin directory."
    }

    Write-Host ""
    Write-Host "Qt runtime DLL checks" -ForegroundColor Cyan

    if (Test-Path -LiteralPath $qtBin -PathType Container) {
        Write-Check -Status "PASS" -Name "Qt bin directory" -DetectedValue $qtBin

        $qtDebugWidgets = Join-Path $qtBin "Qt6Widgetsd.dll"
        $qtReleaseWidgets = Join-Path $qtBin "Qt6Widgets.dll"
        if ((Test-Path -LiteralPath $qtDebugWidgets -PathType Leaf) -or (Test-Path -LiteralPath $qtReleaseWidgets -PathType Leaf)) {
            Write-Check -Status "PASS" -Name "Qt Widgets runtime DLL" -DetectedValue "Qt6Widgetsd.dll or Qt6Widgets.dll found"
        } else {
            Write-Check -Status "FAIL" -Name "Qt Widgets runtime DLL" -DetectedValue "missing" -Suggestion "Verify C:\Qt\6.7.3\msvc2022_64\bin and the selected Qt kit."
        }

        if (Test-PathEntryInProcessPath -PathEntry $qtBin) {
            Write-Check -Status "PASS" -Name "Qt runtime PATH" -DetectedValue $qtBin
        } else {
            Write-Check -Status "WARN" -Name "Qt runtime PATH" -DetectedValue "Qt bin is not in current PATH" -Suggestion "Use CMakePresets.json environment or build_vs2022_qt.ps1 so newSontech.exe can find Qt6Widgetsd.dll."
        }
    } else {
        Write-Check -Status "FAIL" -Name "Qt bin directory" -DetectedValue "missing" -Suggestion "Verify the Qt prefix path."
    }
}

Write-Host ""
Write-Host "OpenCV compile-time checks" -ForegroundColor Cyan

if (-not $openCvEnabled) {
    Write-Check -Status "WARN" -Name "OpenCV" -DetectedValue "disabled by -DisableOpenCV" -Suggestion "Use this only when OpenCV is not needed for the current build."
} elseif ([string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
    Write-Check -Status "WARN" -Name "OpenCV_DIR" -DetectedValue "not configured" -Suggestion "Set OPENCV_DIR or OpenCV_DIR to E:\Libs\OPENCV3\opencv\build\x64\vc15\lib, pass -OpenCV_DIR, or use -DisableOpenCV."
} else {
    $openCvConfig = Join-Path $OpenCV_DIR "OpenCVConfig.cmake"
    if (Test-Path -LiteralPath $OpenCV_DIR -PathType Container) {
        Write-Check -Status "PASS" -Name "OpenCV_DIR" -DetectedValue $OpenCV_DIR
    } else {
        Write-Check -Status "WARN" -Name "OpenCV_DIR" -DetectedValue $OpenCV_DIR -Suggestion "Verify the OpenCV 3.4.x package directory."
    }

    if (Test-Path -LiteralPath $openCvConfig -PathType Leaf) {
        Write-Check -Status "PASS" -Name "OpenCVConfig.cmake" -DetectedValue $openCvConfig
    } else {
        Write-Check -Status "WARN" -Name "OpenCVConfig.cmake" -DetectedValue "missing" -Suggestion "OpenCV_DIR should point to a directory containing OpenCVConfig.cmake, for example E:\Libs\OPENCV3\opencv\build\x64\vc15\lib."
    }

    Write-Host ""
    Write-Host "OpenCV runtime DLL checks" -ForegroundColor Cyan

    if (Test-Path -LiteralPath $openCvBin -PathType Container) {
        Write-Check -Status "PASS" -Name "OpenCV bin directory" -DetectedValue $openCvBin

        $openCvDlls = Get-ChildItem -LiteralPath $openCvBin -Filter "opencv_world34*.dll" -File -ErrorAction SilentlyContinue
        if ($openCvDlls.Count -gt 0) {
            Write-Check -Status "PASS" -Name "OpenCV runtime DLL" -DetectedValue (($openCvDlls | Select-Object -ExpandProperty Name) -join ", ")
        } else {
            Write-Check -Status "WARN" -Name "OpenCV runtime DLL" -DetectedValue "opencv_world34*.dll not found" -Suggestion "Verify E:\Libs\OPENCV3\opencv\build\x64\vc15\bin or rebuild OpenCV 3.4.x for VS2022."
        }

        if (Test-PathEntryInProcessPath -PathEntry $openCvBin) {
            Write-Check -Status "PASS" -Name "OpenCV runtime PATH" -DetectedValue $openCvBin
        } else {
            Write-Check -Status "WARN" -Name "OpenCV runtime PATH" -DetectedValue "OpenCV bin is not in current PATH" -Suggestion "Use CMakePresets.json environment or build_vs2022_qt.ps1 so OpenCV smoke tests can find opencv_world34*.dll."
        }
    } else {
        Write-Check -Status "WARN" -Name "OpenCV bin directory" -DetectedValue "missing" -Suggestion "At runtime, add E:\Libs\OPENCV3\opencv\build\x64\vc15\bin to PATH if OpenCV DLLs are missing."
    }
}

Write-Host ""
Write-Host "CMake project checks" -ForegroundColor Cyan

if (Test-Path -LiteralPath $cmakeListsPath -PathType Leaf) {
    Write-Check -Status "PASS" -Name "CMakeLists.txt" -DetectedValue $cmakeListsPath
} else {
    Write-Check -Status "FAIL" -Name "CMakeLists.txt" -DetectedValue "missing" -Suggestion "Restore newSontech/CMakeLists.txt."
}

$null = Test-FileContains -Path $cmakeListsPath -Pattern "Widgets" -Name "Qt Widgets in CMake" -Suggestion "Add Qt Widgets to find_package and target_link_libraries."
$null = Test-FileContains -Path $cmakeListsPath -Pattern "/utf-8" -Name "MSVC /utf-8 option" -Suggestion "Add target_compile_options(<target> PRIVATE `$<`$<CXX_COMPILER_ID:MSVC>:/utf-8>)."
$null = Test-FileContains -Path $cmakeListsPath -Pattern "CMAKE_AUTOMOC|AUTOMOC" -Name "AUTOMOC" -Suggestion "Enable CMAKE_AUTOMOC for Qt QObject classes."
$null = Test-FileContains -Path $cmakeListsPath -Pattern "NEWSONTECH_ENABLE_OPENCV" -Name "OpenCV option" -Suggestion "Add NEWSONTECH_ENABLE_OPENCV and the OpenCV smoke test target."

if ($RunBuildCheck) {
    Write-Host ""
    Write-Host "Build checks" -ForegroundColor Cyan

    if (-not $cmakeOk) {
        Write-Check -Status "FAIL" -Name "CMake configure" -DetectedValue "skipped because cmake is missing" -Suggestion "Install CMake or add it to PATH."
    } elseif ([string]::IsNullOrWhiteSpace($QtPrefixPath)) {
        Write-Check -Status "FAIL" -Name "CMake configure" -DetectedValue "skipped because QtPrefixPath is missing" -Suggestion "Run with -QtPrefixPath `"C:\Qt\6.x.x\msvc2022_64`"."
    } else {
        Push-Location $projectRoot
        try {
            $configureCommand = @(
                "cmake",
                "-S",
                ".",
                "-B",
                $BuildDir,
                "-DCMAKE_PREFIX_PATH=$QtPrefixPath",
                "-DNEWSONTECH_ENABLE_OPENCV=$(if ($openCvEnabled) { 'ON' } else { 'OFF' })"
            )

            if ($openCvEnabled -and -not [string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
                $configureCommand += "-DOpenCV_DIR=$OpenCV_DIR"
            }

            $configureOk = Invoke-LoggedCommand -Name "CMake configure" -CommandLine $configureCommand
            if ($configureOk) {
                Invoke-LoggedCommand -Name "CMake build" -CommandLine @("cmake", "--build", $BuildDir, "--config", $Configuration)
            } else {
                Write-Check -Status "FAIL" -Name "CMake build" -DetectedValue "skipped because configure failed" -Suggestion "Fix configure errors first."
            }
        } finally {
            Pop-Location
        }
    }
} else {
    Write-Check -Status "WARN" -Name "CMake configure/build" -DetectedValue "not requested" -Suggestion "Add -RunBuildCheck after cmake and Qt are available."
}

Write-Host ""
Write-Host "Summary" -ForegroundColor Cyan
Write-Host ("PASS: {0}  WARN: {1}  FAIL: {2}" -f $script:PassCount, $script:WarnCount, $script:FailCount)

Write-Host ""
Write-Host "Suggested actions" -ForegroundColor Cyan
if ($script:Suggestions.Count -eq 0) {
    Write-Host "- Environment checks passed without additional suggestions."
} else {
    $script:Suggestions | Select-Object -Unique | ForEach-Object {
        Write-Host ("- {0}" -f $_)
    }
}

if ($script:FailCount -gt 0) {
    exit 1
}

exit 0
