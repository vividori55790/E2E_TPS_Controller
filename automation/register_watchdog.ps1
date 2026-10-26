# ======================================================================
# automation/register_watchdog.ps1 -- register DABThesisLoopWatchdog (every 10 min + at startup, S4U)
# and rewrite the runner's .cmd wrapper so it returns the runner's real exit code.
# Usage (repo root):  powershell -NoProfile -ExecutionPolicy Bypass -File automation\register_watchdog.ps1
# Remove:             Unregister-ScheduledTask -TaskName DABThesisLoopWatchdog -Confirm:$false
# ======================================================================
$ErrorActionPreference = "Stop"
$root  = Split-Path -Parent $PSScriptRoot
$state = Join-Path $root "automation\state"
$wd    = Join-Path $root "automation\watchdog.ps1"
$log   = Join-Path $state "runner_stdout.log"
$err   = Join-Path $state "runner_stdout.err"
$cmdf  = Join-Path $state "_detached.cmd"

# (1) the wrapper: keep the runner's exit code (it is read at the NEXT start of DABThesisLoop)
$py = (Get-Command python -ErrorAction SilentlyContinue).Source
$pyw = Join-Path (Split-Path -Parent $py) "pythonw.exe"
if (Test-Path $pyw) { $py = $pyw }
# NOTE: one `$body +=` per line. Inside @( 'a' + $x, ... ) the comma binds tighter than +, which split every path
# onto its own line and produced a wrapper that hung (2026-10-24 19:44).
$body = @()
$body += '@echo off'
$body += ('cd /d "' + $root + '"')
$body += ('echo ==== detached start %DATE% %TIME% (task DABThesisLoop) >> "' + $log + '"')
$body += ('"' + $py + '" "automation\loop_runner.py" >> "' + $log + '" 2>> "' + $err + '"')
$body += 'set RC=%ERRORLEVEL%'
$body += ('echo ==== detached exit %RC% %DATE% %TIME% >> "' + $log + '"')
$body += 'exit /b %RC%'
Set-Content -Path $cmdf -Value $body -Encoding ASCII
Write-Output ("wrapper  " + $cmdf + " (returns the runner's exit code)")

# (2) the watchdog task
$me = [Security.Principal.WindowsIdentity]::GetCurrent().Name
$action = New-ScheduledTaskAction -Execute "powershell.exe" `
            -Argument ('-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "' + $wd + '"') -WorkingDirectory $root
$princ  = New-ScheduledTaskPrincipal -UserId $me -LogonType S4U -RunLevel Limited
$set    = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable `
            -MultipleInstances IgnoreNew -ExecutionTimeLimit (New-TimeSpan -Minutes 5)
$every  = New-ScheduledTaskTrigger -Once -At ((Get-Date).AddMinutes(2)) `
            -RepetitionInterval (New-TimeSpan -Minutes 10) -RepetitionDuration (New-TimeSpan -Days 3650)
$boot   = New-ScheduledTaskTrigger -AtStartup
$boot.Delay = "PT5M"
Register-ScheduledTask -TaskName "DABThesisLoopWatchdog" -Action $action -Principal $princ -Settings $set `
    -Trigger @($every, $boot) -Force -Description "Relaunches DABThesisLoop when loop_runner.py is not alive." | Out-Null
$t = Get-ScheduledTask -TaskName "DABThesisLoopWatchdog"
Write-Output ("watchdog " + $t.TaskName + " state=" + $t.State + " (S4U, every 10 min + 5 min after boot)")
Write-Output ("log      " + (Join-Path $state "watchdog.log"))
