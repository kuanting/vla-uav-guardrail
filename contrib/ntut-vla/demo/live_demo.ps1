# live_demo.ps1 - one-key live demo driver (A: shield off, B: on, C: dynamic)
# Usage:  powershell -ExecutionPolicy Bypass -File demo\live_demo.ps1 [-Stage A|B|C|ALL]
param([string]$Stage = "ALL")

$ROOT = "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
$PY   = "C:\Users\natha\.conda\envs\airsim\python.exe"
$SIM  = "D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe"
$CMD  = "fly to (40, 40) at 6 m/s altitude 20"
$POL  = "$ROOT\policies\urban_demo_policy.yaml"

function Start-Sim {
    Write-Host ">> restarting simulator..." -ForegroundColor Cyan
    Get-Process -Name AirSimNH -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep 5
    Start-Process $SIM -ArgumentList "-ResX=1280","-ResY=720","-windowed"
    foreach ($i in 1..40) {
        Start-Sleep 3
        if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) {
            Write-Host ">> sim ready" -ForegroundColor Green; return $true
        }
    }
    Write-Host "!! sim did not come up - check the AirSimNH window" -ForegroundColor Red
    return $false
}

function Run-Stage([string]$name, [string[]]$extra, [string]$tag, [string]$story) {
    Write-Host ""
    Write-Host "=====================================================" -ForegroundColor Cyan
    Write-Host " STAGE $name  -  $story" -ForegroundColor Cyan
    Write-Host "=====================================================" -ForegroundColor Cyan
    Read-Host "Press ENTER to start stage $name"
    if (-not (Start-Sim)) { return }
    & $PY "$ROOT\demo\run_demo.py" --policy $POL --command $CMD --tag $tag @extra
    $plot = "$ROOT\demo\out\$tag\trajectory.png"
    if (Test-Path $plot) { Invoke-Item $plot }   # pops the trajectory on screen
}

Set-Location $ROOT
if ($Stage -in @("A","ALL")) { Run-Stage "A" @("--shield","off") "live_A" "Shield OFF - watch it violate the no-fly zone" }
if ($Stage -in @("B","ALL")) { Run-Stage "B" @("--shield","on")  "live_B" "Shield ON - same VLA, now guarded" }
if ($Stage -in @("C","ALL")) { Run-Stage "C" @("--shield","on","--dynamic") "live_C" "Dynamic NFZ appears mid-flight at t=8s" }

Write-Host ""
Write-Host "demo complete - artifacts in demo\out\live_*" -ForegroundColor Green
