param(
    [string]$QtPrefixPath = "",
    [string]$OpenCV_DIR = "",
    [string]$VcpkgRoot = "",
    [string]$BuildDir = "out/build/codex-debug",
    [string]$Configuration = "Debug",
    [switch]$Clean,
    [switch]$RunAfterBuild,
    [switch]$DisableOpenCV,
    [switch]$EnablePCL
)

$ErrorActionPreference = "Stop"

$script:PassCount = 0
$script:FailCount = 0
$script:Suggestions = New-Object System.Collections.Generic.List[string]

function Write-Result {
    param(
        [ValidateSet("PASS", "FAIL", "INFO")]
        [string]$Status,
        [string]$Name,
        [string]$Message,
        [string]$Suggestion = ""
    )

    $line = "[{0}] {1}: {2}" -f $Status, $Name, $Message
    switch ($Status) {
        "PASS" {
            $script:PassCount++
            Write-Host $line -ForegroundColor Green
        }
        "FAIL" {
            $script:FailCount++
            Write-Host $line -ForegroundColor Red
        }
        default {
            Write-Host $line -ForegroundColor Cyan
        }
    }

    if (-not [string]::IsNullOrWhiteSpace($Suggestion)) {
        $script:Suggestions.Add(("{0}: {1}" -f $Name, $Suggestion)) | Out-Null
    }
}

function Resolve-InProjectPath {
    param(
        [string]$ProjectRoot,
        [string]$Path
    )

    if ([System.IO.Path]::IsPathRooted($Path)) {
        return [System.IO.Path]::GetFullPath($Path)
    }

    return [System.IO.Path]::GetFullPath((Join-Path $ProjectRoot $Path))
}

function Test-PathInsideProject {
    param(
        [string]$ProjectRoot,
        [string]$CandidatePath
    )

    $projectFullPath = [System.IO.Path]::GetFullPath($ProjectRoot).TrimEnd('\') + '\'
    $candidateFullPath = [System.IO.Path]::GetFullPath($CandidatePath).TrimEnd('\') + '\'
    return $candidateFullPath.StartsWith($projectFullPath, [System.StringComparison]::OrdinalIgnoreCase)
}

function Find-VsWhere {
    $candidatePaths = @(
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe",
        "$env:ProgramFiles\Microsoft Visual Studio\Installer\vswhere.exe"
    )

    foreach ($candidatePath in $candidatePaths) {
        if (-not [string]::IsNullOrWhiteSpace($candidatePath) -and (Test-Path -LiteralPath $candidatePath -PathType Leaf)) {
            return $candidatePath
        }
    }

    $command = Get-Command "vswhere.exe" -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    return ""
}

function Find-VsDevCmd {
    $vsWhere = Find-VsWhere
    if (-not [string]::IsNullOrWhiteSpace($vsWhere)) {
        $installationPath = & $vsWhere -latest -version "[17.0,18.0)" -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($installationPath)) {
            $candidate = Join-Path $installationPath "Common7\Tools\VsDevCmd.bat"
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                Write-Result -Status "PASS" -Name "vswhere" -Message $vsWhere
                return $candidate
            }
        }
    }

    $knownRoots = @(
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Enterprise",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Professional",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Community",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\BuildTools"
    )

    foreach ($knownRoot in $knownRoots) {
        if ([string]::IsNullOrWhiteSpace($knownRoot)) {
            continue
        }

        $candidate = Join-Path $knownRoot "Common7\Tools\VsDevCmd.bat"
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            Write-Result -Status "PASS" -Name "VS2022 fallback" -Message $candidate
            return $candidate
        }
    }

    Write-Result -Status "FAIL" -Name "VS2022" -Message "VsDevCmd.bat not found" -Suggestion "Install VS2022 with the Desktop development with C++ workload, or run from Developer PowerShell for VS2022."
    return ""
}

function Add-PathEntry {
    param(
        [string]$PathEntry,
        [string]$Name
    )

    if ([string]::IsNullOrWhiteSpace($PathEntry) -or -not (Test-Path -LiteralPath $PathEntry -PathType Container)) {
        return
    }

    $pathParts = $env:PATH -split ";"
    if ($pathParts -notcontains $PathEntry) {
        $env:PATH = "$PathEntry;$env:PATH"
    }

    Write-Result -Status "PASS" -Name $Name -Message ("added to PATH: {0}" -f $PathEntry)
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

function Ensure-RuntimeDllPath {
    param(
        [string]$BinDir,
        [string]$Name
    )

    if ([string]::IsNullOrWhiteSpace($BinDir) -or -not (Test-Path -LiteralPath $BinDir -PathType Container)) {
        Write-Result -Status "FAIL" -Name $Name -Message "runtime bin directory missing" -Suggestion "Verify the installed runtime path before running newSontech.exe."
        return
    }

    if (Test-PathEntryInProcessPath -PathEntry $BinDir) {
        Write-Result -Status "PASS" -Name $Name -Message ("already in PATH: {0}" -f $BinDir)
    } else {
        $env:PATH = "$BinDir;$env:PATH"
        Write-Result -Status "PASS" -Name $Name -Message ("prepended to PATH: {0}" -f $BinDir)
    }
}

function Test-RuntimeDlls {
    param(
        [string]$QtBin,
        [bool]$OpenCvEnabled,
        [string]$OpenCvBin
    )

    if (Test-Path -LiteralPath $QtBin -PathType Container) {
        $qtDebugWidgets = Join-Path $QtBin "Qt6Widgetsd.dll"
        $qtReleaseWidgets = Join-Path $QtBin "Qt6Widgets.dll"
        if ((Test-Path -LiteralPath $qtDebugWidgets -PathType Leaf) -or (Test-Path -LiteralPath $qtReleaseWidgets -PathType Leaf)) {
            Write-Result -Status "PASS" -Name "Qt Widgets DLL" -Message ("found in {0}" -f $QtBin)
        } else {
            Write-Result -Status "FAIL" -Name "Qt Widgets DLL" -Message "Qt6Widgetsd.dll or Qt6Widgets.dll not found" -Suggestion "Verify that Qt 6.7.3 MSVC 2022 64-bit is installed completely."
        }
    }

    if ($OpenCvEnabled -and -not [string]::IsNullOrWhiteSpace($OpenCvBin) -and (Test-Path -LiteralPath $OpenCvBin -PathType Container)) {
        $openCvDlls = Get-ChildItem -LiteralPath $OpenCvBin -Filter "opencv_world34*.dll" -File -ErrorAction SilentlyContinue
        if ($openCvDlls.Count -gt 0) {
            Write-Result -Status "PASS" -Name "OpenCV DLL" -Message (($openCvDlls | Select-Object -ExpandProperty Name) -join ", ")
        } else {
            Write-Result -Status "INFO" -Name "OpenCV DLL" -Message "opencv_world34*.dll not found" -Suggestion "Verify E:\Libs\OPENCV3\opencv\build\x64\vc15\bin or rebuild OpenCV 3.4.x for VS2022."
        }
    }
}

function Add-VsBundledToolPaths {
    param(
        [string]$VsDevCmdPath
    )

    $vsToolsDir = Split-Path -Parent $VsDevCmdPath
    $common7Dir = Split-Path -Parent $vsToolsDir
    $vsRoot = Split-Path -Parent $common7Dir

    Add-PathEntry -PathEntry (Join-Path $vsRoot "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin") -Name "VS CMake"
    Add-PathEntry -PathEntry (Join-Path $vsRoot "Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja") -Name "VS Ninja"

    $msvcToolsRoot = Join-Path $vsRoot "VC\Tools\MSVC"
    if (Test-Path -LiteralPath $msvcToolsRoot -PathType Container) {
        $latestMsvcDir = Get-ChildItem -LiteralPath $msvcToolsRoot -Directory |
            Sort-Object -Property Name -Descending |
            Select-Object -First 1

        if ($null -ne $latestMsvcDir) {
            Add-PathEntry -PathEntry (Join-Path $latestMsvcDir.FullName "bin\Hostx64\x64") -Name "MSVC cl"
        }
    }
}

function Import-VsDevEnvironment {
    param(
        [string]$VsDevCmdPath
    )

    Write-Result -Status "INFO" -Name "VsDevCmd" -Message $VsDevCmdPath
    $command = "`"$VsDevCmdPath`" -arch=x64 -host_arch=x64 >nul && set"
    $environmentLines = & cmd.exe /s /c $command
    if ($LASTEXITCODE -ne 0) {
        Write-Result -Status "FAIL" -Name "VsDevCmd" -Message ("exit code {0}" -f $LASTEXITCODE) -Suggestion "Open Developer PowerShell for VS2022 and retry, or repair the VS2022 C++ workload."
        return $false
    }

    foreach ($line in $environmentLines) {
        $separatorIndex = $line.IndexOf("=")
        if ($separatorIndex -le 0) {
            continue
        }

        $name = $line.Substring(0, $separatorIndex)
        $value = $line.Substring($separatorIndex + 1)
        [System.Environment]::SetEnvironmentVariable($name, $value, "Process")
    }

    Write-Result -Status "PASS" -Name "VS2022 environment" -Message "loaded for x64"
    return $true
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
        Write-Result -Status "PASS" -Name $Name -Message "exit code 0"
        return $true
    }

    Write-Result -Status "FAIL" -Name $Name -Message ("exit code {0}" -f $exitCode) -Suggestion "Read the first error above and fix the missing tool, Qt path, or source error."
    return $false
}

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildFullPath = Resolve-InProjectPath -ProjectRoot $projectRoot -Path $BuildDir

if ([string]::IsNullOrWhiteSpace($QtPrefixPath)) {
    if (-not [string]::IsNullOrWhiteSpace($env:QT_MSVC2022_64)) {
        $QtPrefixPath = $env:QT_MSVC2022_64
    } elseif (Test-Path -LiteralPath "D:\QT\6.7.3\msvc2022_64" -PathType Container) {
        $QtPrefixPath = "D:\QT\6.7.3\msvc2022_64"
    } else {
        $QtPrefixPath = "C:\Qt\6.7.3\msvc2022_64"
    }
}

$openCvEnabled = -not $DisableOpenCV
if ([string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
    if (-not [string]::IsNullOrWhiteSpace($env:OPENCV_DIR)) {
        $OpenCV_DIR = $env:OPENCV_DIR
    } elseif (-not [string]::IsNullOrWhiteSpace($env:OpenCV_DIR)) {
        $OpenCV_DIR = $env:OpenCV_DIR
    } elseif (Test-Path -LiteralPath "E:\Libs\OPENCV3\opencv\build\x64\vc15\lib\OpenCVConfig.cmake" -PathType Leaf) {
        $OpenCV_DIR = "E:\Libs\OPENCV3\opencv\build\x64\vc15\lib"
    }
}

$openCvBin = ""
if (-not [string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
    $openCvBin = Join-Path (Split-Path -Parent $OpenCV_DIR) "bin"
}

if ([string]::IsNullOrWhiteSpace($VcpkgRoot)) {
    if (-not [string]::IsNullOrWhiteSpace($env:VCPKG_ROOT)) {
        $VcpkgRoot = $env:VCPKG_ROOT
    } elseif (Test-Path -LiteralPath "C:\vcpkg\scripts\buildsystems\vcpkg.cmake" -PathType Leaf) {
        $VcpkgRoot = "C:\vcpkg"
    }
}

$vcpkgToolchain = ""
if (-not [string]::IsNullOrWhiteSpace($VcpkgRoot)) {
    $vcpkgToolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"
}

Remove-Item Env:VCPKG_FORCE_SYSTEM_BINARIES -ErrorAction SilentlyContinue

Write-Host "newSontech VS2022 + Qt build" -ForegroundColor Cyan
Write-Host ("Project root: {0}" -f $projectRoot)
Write-Host ("Build dir:    {0}" -f $buildFullPath)
Write-Host ("Qt prefix:    {0}" -f $QtPrefixPath)
Write-Host ("OpenCV_DIR:   {0}" -f ($(if ([string]::IsNullOrWhiteSpace($OpenCV_DIR)) { "<not set>" } else { $OpenCV_DIR })))
Write-Host ("OpenCV bin:   {0}" -f ($(if ([string]::IsNullOrWhiteSpace($openCvBin)) { "<not set>" } else { $openCvBin })))
Write-Host ("OpenCV:       {0}" -f ($(if ($openCvEnabled) { "ON" } else { "OFF" })))
Write-Host "PCL:          REQUIRED (vcpkg official, default features off)"
Write-Host ("VCPKG_ROOT:   {0}" -f ($(if ([string]::IsNullOrWhiteSpace($VcpkgRoot)) { "<not set>" } else { $VcpkgRoot })))
Write-Host ("vcpkg tools:  vcpkg-managed binaries allowed")
Write-Host ("Config:       {0}" -f $Configuration)
Write-Host ""

if (-not (Test-Path -LiteralPath (Join-Path $projectRoot "CMakeLists.txt") -PathType Leaf)) {
    Write-Result -Status "FAIL" -Name "CMakeLists.txt" -Message "missing" -Suggestion "Run this script from the newSontech tree or restore CMakeLists.txt."
}

if (-not (Test-Path -LiteralPath $QtPrefixPath -PathType Container)) {
    Write-Result -Status "FAIL" -Name "QtPrefixPath" -Message $QtPrefixPath -Suggestion "Install Qt MSVC 2022 64-bit or pass -QtPrefixPath."
} else {
    Write-Result -Status "PASS" -Name "QtPrefixPath" -Message $QtPrefixPath
}

$qt6Config = Join-Path $QtPrefixPath "lib\cmake\Qt6\Qt6Config.cmake"
$qtBin = Join-Path $QtPrefixPath "bin"
$windeployqt = Join-Path $qtBin "windeployqt.exe"

if (Test-Path -LiteralPath $qt6Config -PathType Leaf) {
    Write-Result -Status "PASS" -Name "Qt6Config.cmake" -Message $qt6Config
} else {
    Write-Result -Status "FAIL" -Name "Qt6Config.cmake" -Message "missing" -Suggestion "Use a Qt 6 MSVC kit directory such as C:\Qt\6.7.3\msvc2022_64."
}

if (Test-Path -LiteralPath $qtBin -PathType Container) {
    Ensure-RuntimeDllPath -BinDir $qtBin -Name "Qt runtime PATH"
} else {
    Write-Result -Status "FAIL" -Name "Qt bin" -Message "missing" -Suggestion "Verify the Qt prefix path."
}

if ([string]::IsNullOrWhiteSpace($vcpkgToolchain)) {
    Write-Result -Status "FAIL" -Name "vcpkg toolchain" -Message "VCPKG_ROOT is not set" -Suggestion "Set VCPKG_ROOT or pass -VcpkgRoot so Eigen3 can be resolved from vcpkg manifest mode."
} elseif (Test-Path -LiteralPath $vcpkgToolchain -PathType Leaf) {
    Write-Result -Status "PASS" -Name "vcpkg toolchain" -Message $vcpkgToolchain
} else {
    Write-Result -Status "FAIL" -Name "vcpkg toolchain" -Message $vcpkgToolchain -Suggestion "Verify VCPKG_ROOT points to a vcpkg checkout containing scripts\buildsystems\vcpkg.cmake."
}

$sevenZipCommand = Get-Command "7z.exe" -ErrorAction SilentlyContinue
if ($null -ne $sevenZipCommand) {
    Write-Result -Status "PASS" -Name "7-Zip" -Message $sevenZipCommand.Source
} else {
    Write-Result -Status "INFO" -Name "7-Zip" -Message "7z.exe not found in PATH" -Suggestion "vcpkg can download and use its managed 7zip tool because VCPKG_FORCE_SYSTEM_BINARIES is not set."
}

if ($openCvEnabled) {
    if ([string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
        Write-Result -Status "INFO" -Name "OpenCV_DIR" -Message "not set" -Suggestion "Set OPENCV_DIR or OpenCV_DIR to E:\Libs\OPENCV3\opencv\build\x64\vc15\lib, pass -OpenCV_DIR, or use -DisableOpenCV."
    } else {
        $openCvConfig = Join-Path $OpenCV_DIR "OpenCVConfig.cmake"
        if (Test-Path -LiteralPath $openCvConfig -PathType Leaf) {
            Write-Result -Status "PASS" -Name "OpenCVConfig.cmake" -Message $openCvConfig
        } else {
            Write-Result -Status "FAIL" -Name "OpenCVConfig.cmake" -Message "missing" -Suggestion "OpenCV_DIR should point to a directory containing OpenCVConfig.cmake, for example E:\Libs\OPENCV3\opencv\build\x64\vc15\lib."
        }

        if (Test-Path -LiteralPath $openCvBin -PathType Container) {
            Ensure-RuntimeDllPath -BinDir $openCvBin -Name "OpenCV runtime PATH"
        } else {
            Write-Result -Status "INFO" -Name "OpenCV bin" -Message "not found" -Suggestion "If OpenCV DLLs are missing at runtime, add E:\Libs\OPENCV3\opencv\build\x64\vc15\bin to PATH."
        }
    }
} else {
    Write-Result -Status "INFO" -Name "OpenCV" -Message "disabled by -DisableOpenCV"
}

$vsDevCmd = Find-VsDevCmd
if (-not [string]::IsNullOrWhiteSpace($vsDevCmd)) {
    $null = Import-VsDevEnvironment -VsDevCmdPath $vsDevCmd
    Add-VsBundledToolPaths -VsDevCmdPath $vsDevCmd
    Ensure-RuntimeDllPath -BinDir $qtBin -Name "Qt runtime PATH after VsDevCmd"
    if ($openCvEnabled -and -not [string]::IsNullOrWhiteSpace($openCvBin)) {
        Ensure-RuntimeDllPath -BinDir $openCvBin -Name "OpenCV runtime PATH after VsDevCmd"
    }
    Test-RuntimeDlls -QtBin $qtBin -OpenCvEnabled $openCvEnabled -OpenCvBin $openCvBin
}

foreach ($requiredCommand in @("cmake", "ninja", "cl")) {
    $command = Get-Command $requiredCommand -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        Write-Result -Status "PASS" -Name $requiredCommand -Message $command.Source
    } else {
        Write-Result -Status "FAIL" -Name $requiredCommand -Message "not found in PATH" -Suggestion "Install the tool or verify that VsDevCmd and CMake/Ninja are available."
    }
}

if ($script:FailCount -gt 0) {
    Write-Host ""
    Write-Host "Build skipped because required environment checks failed." -ForegroundColor Red
} else {
    if ($Clean) {
        if (-not (Test-PathInsideProject -ProjectRoot $projectRoot -CandidatePath $buildFullPath)) {
            Write-Result -Status "FAIL" -Name "Clean" -Message "build dir is outside newSontech" -Suggestion "Use a build directory under newSontech."
        } elseif (Test-Path -LiteralPath $buildFullPath) {
            Remove-Item -LiteralPath $buildFullPath -Recurse -Force
            Write-Result -Status "PASS" -Name "Clean" -Message $buildFullPath
        } else {
            Write-Result -Status "INFO" -Name "Clean" -Message "build dir does not exist"
        }
    }

    if ($script:FailCount -eq 0) {
        Push-Location $projectRoot
        try {
            $configureCommand = @(
                "cmake",
                "-S",
                ".",
                "-B",
                $buildFullPath,
                "-G",
                "Ninja",
                "-DCMAKE_PREFIX_PATH=$QtPrefixPath",
                "-DCMAKE_TOOLCHAIN_FILE=$vcpkgToolchain",
                "-DVCPKG_TARGET_TRIPLET=x64-windows",
                "-DVCPKG_INSTALL_OPTIONS=--clean-buildtrees-after-build;--clean-packages-after-build",
                "-DCMAKE_BUILD_TYPE=$Configuration",
                "-DI2R_RLC_ENABLE_OPENCV=$(if ($openCvEnabled) { 'ON' } else { 'OFF' })"
            )

            $env:X_VCPKG_ASSET_SOURCES = "clear;x-script,curl.exe --ssl-no-revoke --retry 8 --retry-all-errors --retry-delay 5 -L {url} --output {dst}"

            if ($openCvEnabled -and -not [string]::IsNullOrWhiteSpace($OpenCV_DIR)) {
                $configureCommand += "-DOpenCV_DIR=$OpenCV_DIR"
            }

            $configureOk = Invoke-LoggedCommand -Name "CMake configure" -CommandLine $configureCommand

            if ($configureOk) {
                $buildOk = Invoke-LoggedCommand -Name "CMake build" -CommandLine @(
                    "cmake",
                    "--build",
                    $buildFullPath,
                    "--config",
                    $Configuration
                )

                if ($buildOk -and $RunAfterBuild) {
                    Ensure-RuntimeDllPath -BinDir $qtBin -Name "Qt runtime PATH before run"
                    if ($openCvEnabled -and -not [string]::IsNullOrWhiteSpace($openCvBin)) {
                        Ensure-RuntimeDllPath -BinDir $openCvBin -Name "OpenCV runtime PATH before run"
                    }
                    Test-RuntimeDlls -QtBin $qtBin -OpenCvEnabled $openCvEnabled -OpenCvBin $openCvBin
                    Write-Host ("Runtime PATH prefix: {0}" -f ((($env:PATH -split ";") | Select-Object -First 5) -join ";")) -ForegroundColor Cyan

                    $exePath = Join-Path $buildFullPath "newSontech.exe"
                    if (-not (Test-Path -LiteralPath $exePath -PathType Leaf)) {
                        $exePath = Join-Path (Join-Path $buildFullPath $Configuration) "newSontech.exe"
                    }

                    if (Test-Path -LiteralPath $exePath -PathType Leaf) {
                        Write-Host ""
                        Write-Host ("Running newSontech: {0}" -f $exePath) -ForegroundColor Cyan
                        & $exePath
                        if ($LASTEXITCODE -eq 0) {
                            Write-Result -Status "PASS" -Name "Run newSontech" -Message "exit code 0"
                        } else {
                            Write-Result -Status "FAIL" -Name "Run newSontech" -Message ("exit code {0}" -f $LASTEXITCODE) -Suggestion "If Qt or OpenCV DLLs are missing, verify that Qt bin and OpenCV bin appear in the runtime PATH prefix printed above."
                        }
                    } else {
                        Write-Result -Status "FAIL" -Name "Run newSontech" -Message "executable not found" -Suggestion "Check the build output directory."
                    }
                }
            }
        } finally {
            Pop-Location
        }
    }
}

Write-Host ""
Write-Host "Summary" -ForegroundColor Cyan
Write-Host ("PASS: {0}  FAIL: {1}" -f $script:PassCount, $script:FailCount)

Write-Host ""
Write-Host "Suggested actions" -ForegroundColor Cyan
if ($script:Suggestions.Count -eq 0) {
    Write-Host "- No additional action is required."
} else {
    $script:Suggestions | Select-Object -Unique | ForEach-Object {
        Write-Host ("- {0}" -f $_)
    }
}

if ($script:FailCount -gt 0) {
    if (Test-Path -LiteralPath $windeployqt -PathType Leaf) {
        Write-Host ("- Qt deployment helper is available at: {0}" -f $windeployqt)
    }
    exit 1
}
