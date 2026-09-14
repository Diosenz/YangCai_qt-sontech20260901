param(
    [Parameter(Mandatory = $true)]
    [string]$Preset,

    [string]$Target = "",
    [string]$CTestRegex = "",
    [string]$Configuration = "Debug",
    [string]$BuildDir = "",
    [string]$LogDir = "logs/build-watchdog",
    [string]$VsDevCmdPath = "",
    [string[]]$RuntimePathEntries = @("C:\Qt\6.7.3\msvc2022_64\bin"),
    [int]$TotalTimeoutSeconds = 180,
    [int]$NoOutputTimeoutSeconds = 90,
    [switch]$SkipBuild,
    [switch]$RunCTest
)

$ErrorActionPreference = "Stop"

function Find-VsDevCmd {
    if (-not [string]::IsNullOrWhiteSpace($VsDevCmdPath)) {
        if (Test-Path -LiteralPath $VsDevCmdPath -PathType Leaf) {
            return $VsDevCmdPath
        }
        throw "VsDevCmdPath does not exist: $VsDevCmdPath"
    }

    $knownRoots = @(
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Enterprise",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Professional",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Community",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\BuildTools"
    )

    foreach ($root in $knownRoots) {
        if ([string]::IsNullOrWhiteSpace($root)) {
            continue
        }

        $candidate = Join-Path $root "Common7\Tools\VsDevCmd.bat"
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }

    throw "VsDevCmd.bat was not found."
}

function Get-BuildDirFromPreset {
    param([string]$PresetName)

    switch ($PresetName) {
        "qt-debug-no-opencv" { return "out\build\qt-debug-no-opencv" }
        "qt-debug-opencv" { return "out\build\qt-debug-opencv" }
        "qt-debug" { return "out\build\qt-debug" }
        default { return "" }
    }
}

function Stop-ProcessTree {
    param([int]$ProcessId)

    & taskkill.exe /PID $ProcessId /T /F | Out-Host
}

function Invoke-WatchdogCommand {
    param(
        [string]$Name,
        [string]$CommandLine,
        [string]$VsDevCmd,
        [string]$LogRoot
    )

    New-Item -ItemType Directory -Force -Path $LogRoot | Out-Null
    $safeName = ($Name -replace '[^A-Za-z0-9_.-]', '_')
    $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $stdoutPath = Join-Path $LogRoot "$timestamp-$safeName.out.log"
    $stderrPath = Join-Path $LogRoot "$timestamp-$safeName.err.log"

    $runtimePrefix = ""
    foreach ($entry in $RuntimePathEntries) {
        if (-not [string]::IsNullOrWhiteSpace($entry)) {
            $runtimePrefix += "$entry;"
        }
    }

    $wrappedCommand =
        "call `"$VsDevCmd`" -arch=x64 -host_arch=x64 >nul && set PATH=$runtimePrefix%PATH% && $CommandLine"

    Write-Host ""
    Write-Host "[$Name] $CommandLine"
    Write-Host "stdout: $stdoutPath"
    Write-Host "stderr: $stderrPath"

    $process = Start-Process `
        -FilePath "cmd.exe" `
        -ArgumentList @("/d", "/s", "/c", $wrappedCommand) `
        -RedirectStandardOutput $stdoutPath `
        -RedirectStandardError $stderrPath `
        -NoNewWindow `
        -PassThru

    $startTime = Get-Date
    $lastOutputTime = $startTime
    $lastStdoutLength = 0L
    $lastStderrLength = 0L

    while (-not $process.HasExited) {
        Start-Sleep -Seconds 2
        $process.Refresh()

        $stdoutLength = 0L
        $stderrLength = 0L
        if (Test-Path -LiteralPath $stdoutPath -PathType Leaf) {
            $stdoutLength = (Get-Item -LiteralPath $stdoutPath).Length
        }
        if (Test-Path -LiteralPath $stderrPath -PathType Leaf) {
            $stderrLength = (Get-Item -LiteralPath $stderrPath).Length
        }

        if ($stdoutLength -ne $lastStdoutLength -or $stderrLength -ne $lastStderrLength) {
            $lastOutputTime = Get-Date
            $lastStdoutLength = $stdoutLength
            $lastStderrLength = $stderrLength
        }

        $elapsed = ((Get-Date) - $startTime).TotalSeconds
        $silent = ((Get-Date) - $lastOutputTime).TotalSeconds
        if ($elapsed -gt $TotalTimeoutSeconds -or $silent -gt $NoOutputTimeoutSeconds) {
            Write-Host "[$Name] timeout: elapsed=$([int]$elapsed)s silent=$([int]$silent)s"
            Stop-ProcessTree -ProcessId $process.Id
            throw "$Name timed out. See $stdoutPath and $stderrPath"
        }
    }

    $exitCode = $process.ExitCode
    if ($exitCode -ne 0) {
        Write-Host "[$Name] failed with exit code $exitCode"
        if (Test-Path -LiteralPath $stderrPath -PathType Leaf) {
            Get-Content -LiteralPath $stderrPath -Tail 80 | Out-Host
        }
        if (Test-Path -LiteralPath $stdoutPath -PathType Leaf) {
            Get-Content -LiteralPath $stdoutPath -Tail 80 | Out-Host
        }
        throw "$Name failed with exit code $exitCode"
    }

    Write-Host "[$Name] passed"
}

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Push-Location $projectRoot
try {
    $vsDevCmd = Find-VsDevCmd
    if ([string]::IsNullOrWhiteSpace($BuildDir)) {
        $BuildDir = Get-BuildDirFromPreset -PresetName $Preset
    }

    if (-not $SkipBuild) {
        $buildCommand = "cmake --build --preset $Preset --verbose"
        if (-not [string]::IsNullOrWhiteSpace($Target)) {
            $buildCommand = "cmake --build --preset $Preset --target $Target --verbose"
        }
        Invoke-WatchdogCommand `
            -Name "build-$Preset-$Target" `
            -CommandLine $buildCommand `
            -VsDevCmd $vsDevCmd `
            -LogRoot $LogDir
    }

    if ($RunCTest) {
        if ([string]::IsNullOrWhiteSpace($BuildDir)) {
            throw "BuildDir is required for ctest when preset is unknown."
        }

        $ctestCommand = "ctest --test-dir `"$BuildDir`" -C $Configuration --output-on-failure"
        if (-not [string]::IsNullOrWhiteSpace($CTestRegex)) {
            $ctestCommand += " -R $CTestRegex"
        }

        Invoke-WatchdogCommand `
            -Name "ctest-$Preset-$CTestRegex" `
            -CommandLine $ctestCommand `
            -VsDevCmd $vsDevCmd `
            -LogRoot $LogDir
    }
} finally {
    Pop-Location
}
