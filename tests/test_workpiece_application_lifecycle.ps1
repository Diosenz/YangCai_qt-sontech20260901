param(
    [Parameter(Mandatory = $true)]
    [string]$Executable
)

# 对最终应用做入口级验证；仅使用 UI Automation Invoke，不调用鼠标。
# 复用生产代码已有的禁止设备自动启动开关，不连接机器人或启动相机。
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
$taskReferences = @('UIAutomationClient', 'UIAutomationTypes')
if ($PSVersionTable.PSEdition -eq 'Core') {
    $taskReferences += Join-Path $PSHOME 'ref/System.Threading.Thread.dll'
}
Add-Type -ReferencedAssemblies $taskReferences -TypeDefinition @'
using System;
using System.Threading;
using System.Windows.Automation;
public sealed class WorkpieceTestInvoke {
    public volatile bool Completed;
    public Exception Error;
    public void Start(AutomationElement button) {
        var thread = new Thread(() => {
            try { ((InvokePattern)button.GetCurrentPattern(InvokePattern.Pattern)).Invoke(); }
            catch (Exception error) { Error = error; }
            finally { Completed = true; }
        });
        thread.IsBackground = true;
        thread.Start();
    }
}
'@

$taskExe = (Resolve-Path -LiteralPath $Executable).Path
$taskLog = Join-Path $env:LOCALAPPDATA 'i2r_rlc/logs/i2r_rlc.log'
$taskLogStart = if (Test-Path -LiteralPath $taskLog) {
    [IO.File]::ReadAllText($taskLog).Length
} else { 0 }
$taskProcess = [Diagnostics.Process]::new()
$taskProcess.StartInfo = [Diagnostics.ProcessStartInfo]::new($taskExe)
$taskProcess.StartInfo.UseShellExecute = $false
# UI Automation 需要可访问的实际窗口；隐藏启动会使 Qt 顶层窗口不进入 UIA 树。
$taskProcess.StartInfo.WindowStyle = [Diagnostics.ProcessWindowStyle]::Normal
$taskProcess.StartInfo.WorkingDirectory = Split-Path $taskExe
$taskProcess.StartInfo.EnvironmentVariables['I2R_RLC_DISABLE_AUTO_DEVICE_START'] = '1'
$taskProcess.StartInfo.EnvironmentVariables['QT_ACCESSIBILITY'] = '1'

function Read-ThisRunLog {
    if (-not (Test-Path -LiteralPath $taskLog)) { return '' }
    # 必须允许应用同时追加，否则轮询读取可能令生产日志 open() 失败。
    $stream = [IO.FileStream]::new($taskLog, [IO.FileMode]::Open,
        [IO.FileAccess]::Read, [IO.FileShare]::ReadWrite)
    $reader = [IO.StreamReader]::new($stream)
    try { $text = $reader.ReadToEnd() } finally { $reader.Dispose() }
    if ($text.Length -lt $taskLogStart) { return $text }
    return $text.Substring($taskLogStart)
}

function Assert-ApplicationAlive {
    $taskProcess.Refresh()
    if ($taskProcess.HasExited) {
        throw "application exited early, code=$($taskProcess.ExitCode)"
    }
    $condition = [System.Windows.Automation.PropertyCondition]::new(
        [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $taskProcess.Id)
    $windows = [System.Windows.Automation.AutomationElement]::RootElement.FindAll(
        [System.Windows.Automation.TreeScope]::Children, $condition)
    foreach ($window in $windows) {
        if ($window.Current.Name -match 'Microsoft Visual C\+\+|Runtime Library|提示!') {
            throw "application dialog: $($window.Current.Name)"
        }
    }
    return $windows
}

try {
    [void]$taskProcess.Start()
    $taskButton = $null
    $taskDeadline = [DateTime]::UtcNow.AddSeconds(20)
    while ([DateTime]::UtcNow -lt $taskDeadline -and $null -eq $taskButton) {
        foreach ($window in (Assert-ApplicationAlive)) {
            $taskButton = $window.FindFirst(
                [System.Windows.Automation.TreeScope]::Descendants,
                [System.Windows.Automation.PropertyCondition]::new(
                    [System.Windows.Automation.AutomationElement]::NameProperty, '选择工件'))
            if ($null -ne $taskButton) { break }
        }
        if ($null -eq $taskButton) { Start-Sleep -Milliseconds 100 }
    }
    if ($null -eq $taskButton) {
        foreach ($window in (Assert-ApplicationAlive)) {
            Write-Output "Window: $($window.Current.Name)"
            $elements = $window.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                [System.Windows.Automation.Condition]::TrueCondition)
            foreach ($element in $elements) {
                Write-Output "  $($element.Current.ControlType.ProgrammaticName): $($element.Current.Name)"
            }
        }
        throw 'workpiece button unavailable before timeout'
    }
    if ((Read-ThisRunLog) -notmatch '跳过机器人自动连接') {
        throw 'device isolation was not confirmed in application log'
    }
    for ($selection = 1; $selection -le 2; ++$selection) {
        $invoke = [WorkpieceTestInvoke]::new()
        $invoke.Start($taskButton)
        $taskDeadline = [DateTime]::UtcNow.AddSeconds(15)
        $ready = $false
        while ([DateTime]::UtcNow -lt $taskDeadline) {
            [void](Assert-ApplicationAlive)
            if ($invoke.Error) { throw $invoke.Error }
            $runLog = Read-ThisRunLog
            $count = [regex]::Matches($runLog, '当前工件已分别自动提交在线/离线纠偏配置').Count
            $onlineCount = [regex]::Matches($runLog, '在线纠偏配置已提交').Count
            $offlineCount = [regex]::Matches($runLog, '离线纠偏：存档已选择；离线纠偏存档读取完成').Count
            if ($invoke.Completed -and $count -ge $selection -and
                $onlineCount -ge $selection -and $offlineCount -ge $selection -and
                $runLog.Contains('online correction configured')) {
                $ready = $true
                break
            }
            Start-Sleep -Milliseconds 100
        }
        if (-not $ready) { throw "selection $selection did not complete configuration" }
        Write-Output "PASS T3: selection $selection submitted online configuration and loaded offline archive"
        Start-Sleep -Milliseconds 500
    }
    if (-not $taskProcess.CloseMainWindow()) { throw 'normal application close was unavailable' }
    if (-not $taskProcess.WaitForExit(10000)) { throw 'normal shutdown timed out' }
    if ($taskProcess.ExitCode -ne 0) { throw "shutdown exit code=$($taskProcess.ExitCode)" }
    Write-Output 'PASS T3: final application startup, repeated selection and normal shutdown'
} catch {
    Write-Output "FAIL T3: $_"
    exit 1
} finally {
    # 仅回收本测试启动的进程；不操作用户已有程序，也不删除崩溃转储。
    if ($taskProcess.Id -and -not $taskProcess.HasExited) {
        $taskProcess.Kill()
        [void]$taskProcess.WaitForExit(5000)
    }
    $taskProcess.Dispose()
}
