# ======================================================================
# automation/start_loop.ps1 -- start automation/loop_runner.py DETACHED (Task Scheduler, S4U)
# ======================================================================
# Adapted from GradThesis/tools/start_loop.ps1 v0.3.3 (measured on this desktop):
#   * the runner is handed to the Task Scheduler service, so an ssh / VS Code Remote drop cannot kill it;
#   * S4U (no console -> no console control event), pythonw (no console at all);
#   * restart on failure 3x / 5 min, AtStartup trigger so a reboot resumes it;
#   * the principal comes from WindowsIdentity (this is a WORKGROUP box);
#   * refuses to start a second runner beside a live one.
# Usage (repo root):
#   powershell -NoProfile -ExecutionPolicy Bypass -File automation\start_loop.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File automation\start_loop.ps1 -Extra "--dry-run"
# Status / stop from any later session:
#   python automation\loop_runner.py --status
#   python automation\loop_runner.py --stop "why"
param(
    [string]$Extra = "",
    [switch]$Interactive,
    [switch]$Force
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$dir  = Join-Path $root "automation\state"
$log  = Join-Path $dir "runner_stdout.log"
$err  = Join-Path $dir "runner_stdout.err"
$cmdf = Join-Path $dir "_detached.cmd"
New-Item -ItemType Directory -Force $dir | Out-Null

$alive = Get-CimInstance Win32_Process -Filter "(Name='python.exe' OR Name='pythonw.exe')" |
         Where-Object { $_.CommandLine -like "*automation*loop_runner.py*" }
if ($alive -and -not $Force) {
    Write-Output ("REFUSED: loop_runner.py already running (pid " + ($alive.ProcessId -join ", ") + ").")
    exit 2
}

$py = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $py) { Write-Output "FAILED: python is not on PATH"; exit 3 }
$pyw = Join-Path (Split-Path -Parent $py) "pythonw.exe"
if (Test-Path $pyw) { $py = $pyw }

$argv = @("automation\loop_runner.py")
if ($Extra -ne "") { $argv += ($Extra -split " ") }

$task = "DABThesisLoop"
$body = @()
$body += '@echo off'
$body += 'cd /d "' + $root + '"'
$body += 'echo ==== detached start %DATE% %TIME% (task ' + $task + ') >> "' + $log + '"'
$body += '"' + $py + '" ' + (($argv | ForEach-Object { '"' + $_ + '"' }) -join ' ') + ' >> "' + $log + '" 2>> "' + $err + '"'
# keep the runner's exit code: ending on `echo` reported success and disabled restart-on-failure (10-21)
$body += 'set RC=%ERRORLEVEL%'
$body += 'echo ==== detached exit %RC% %DATE% %TIME% >> "' + $log + '"'
$body += 'exit /b %RC%'
Set-Content -Path $cmdf -Value $body -Encoding ASCII

$me = [Security.Principal.WindowsIdentity]::GetCurrent().Name
$logon = "S4U"
if ($Interactive) { $logon = "Interactive" }
$action = New-ScheduledTaskAction -Execute "cmd.exe" -Argument ('/c "' + $cmdf + '"') -WorkingDirectory $root
$princ  = New-ScheduledTaskPrincipal -UserId $me -LogonType $logon -RunLevel Limited
$set    = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
            -StartWhenAvailable -MultipleInstances IgnoreNew -ExecutionTimeLimit ([TimeSpan]::Zero) `
            -RestartCount 3 -RestartInterval (New-TimeSpan -Minutes 5)
$boot = New-ScheduledTaskTrigger -AtStartup
$boot.Delay = "PT2M"
try {
    Register-ScheduledTask -TaskName $task -Action $action -Principal $princ -Settings $set -Trigger $boot -Force `
        -Description "DAB thesis loop_runner.py, detached so it survives an ssh / VS Code disconnect." | Out-Null
} catch {
    if ($logon -eq "S4U") {
        Write-Output ("S4U registration failed (" + $_.Exception.Message + ") -- falling back to Interactive")
        $logon = "Interactive"
        $princ = New-ScheduledTaskPrincipal -UserId $me -LogonType Interactive -RunLevel Limited
        Register-ScheduledTask -TaskName $task -Action $action -Principal $princ -Settings $set -Force `
            -Description "DAB thesis loop_runner.py (Interactive fallback)." | Out-Null
    } else { throw }
}
Start-ScheduledTask -TaskName $task

$seen = $null
for ($i = 0; $i -lt 30; $i++) {
    Start-Sleep -Milliseconds 500
    $now = Get-CimInstance Win32_Process -Filter "(Name='python.exe' OR Name='pythonw.exe')" |
           Where-Object { $_.CommandLine -like "*automation*loop_runner.py*" }
    if ($now) { $seen = ($now.ProcessId -join ", "); break }
}
Write-Output ("task     " + $task + "  (" + $logon + ", user " + $me + ", restart 3x/5min, AtStartup)")
Write-Output ("stdout   " + $log)
if ($seen) {
    Write-Output ("RUNNING  pid " + $seen + " -- owned by the Task Scheduler; the ssh session can close.")
} else {
    Write-Output "NOT SEEN within 15 s -- check automation\state\runner_stdout.err and runner_stdout.log"
}
Write-Output "status:  python automation\loop_runner.py --status"
Write-Output "stop:    python automation\loop_runner.py --stop ""why"""
