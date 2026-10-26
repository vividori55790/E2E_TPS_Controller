# ======================================================================
# automation/watchdog.ps1 -- relaunch DABThesisLoop when loop_runner.py is not alive
# ======================================================================
# Why: on 2026-10-21 17:54 the runner died of one exception and stayed dead for three days. The task's
# restart-on-failure never fired, because the .cmd wrapper ended with an `echo` and reported success.
# A reboot (AtStartup) happened to bring it back on 10-24. This watchdog runs every 10 minutes:
#   STOP file present       -> do nothing (a deliberate stop, or the agent reported goal_met)
#   runner process alive    -> do nothing
#   otherwise               -> Start-ScheduledTask DABThesisLoop (and log it)
# Registered by automation/register_watchdog.ps1.
$ErrorActionPreference = "Continue"
$root  = Split-Path -Parent $PSScriptRoot
$state = Join-Path $root "automation\state"
$wlog  = Join-Path $state "watchdog.log"
$stop  = Join-Path $state "STOP"
function Say($m) { Add-Content -Path $wlog -Value ("[" + (Get-Date -Format "yyyy-MM-dd HH:mm:ss") + "] " + $m) -Encoding UTF8 }

$restart = Join-Path $state "RESTART"
$alive = Get-CimInstance Win32_Process -Filter "(Name='python.exe' OR Name='pythonw.exe')" |
         Where-Object { $_.CommandLine -like "*automation*loop_runner.py*" -and
                        $_.CommandLine -notmatch "--(status|stop|usage|dry-run|once|clear-stop)" }
if ($alive) { exit 0 }
if (Test-Path $stop) {
    # STOP + RESTART = "stop the old runner after its turn, then start the new code" (an old runner knows only STOP)
    if (Test-Path $restart) {
        Remove-Item $stop, $restart -Force -ErrorAction SilentlyContinue
        Say "STOP+RESTART and no runner alive -- cleared both, relaunching with the current code"
    } else { exit 0 }
}

$hb = Join-Path $state "heartbeat.txt"
$age = "none"
if (Test-Path $hb) { $age = [int]((Get-Date) - (Get-Item $hb).LastWriteTime).TotalMinutes }
try {
    Start-ScheduledTask -TaskName "DABThesisLoop"
    Say ("runner not alive (heartbeat age " + $age + " min) -- started DABThesisLoop")
} catch {
    Say ("runner not alive, and starting DABThesisLoop failed: " + $_.Exception.Message)
}
