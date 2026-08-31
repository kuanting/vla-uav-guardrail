# demo_latest.ps1 - showcase the LATEST demo (trained policy + gate-2 model).
# Three stages tell the whole story; the script restarts the sim between each.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File demo_latest.ps1            # all 3 stages
#   powershell -ExecutionPolicy Bypass -File demo_latest.ps1 -Stage 3   # just the model
#   .\demo_latest.ps1 -World mountains                                   # different world
param(
    [ValidateSet("1","2","3","ALL")] [string]$Stage = "ALL",
    [ValidateSet("nh","blocks","mountains")] [string]$World = "nh"
)

$ROOT = "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
$PY   = "C:\Users\natha\.conda\envs\airsim\python.exe"
$POL  = "$ROOT\policies\urban_demo_policy.yaml"
$CMD  = "fly to (40, 40) at 6 m/s altitude 20"
$SIMS = @{ nh="D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe";
           blocks="D:\AirSim\Blocks\WindowsNoEditor\Blocks.exe";
           mountains="D:\AirSim\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe" }
$PROCS = @{ nh="AirSimNH"; blocks="Blocks"; mountains="LandscapeMountains" }

function Restart-Sim {
    Write-Host ">> restarting simulator ($World)..." -ForegroundColor Cyan
    Get-Process -Name $PROCS.Values -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep 5
    Start-Process $SIMS[$World] -ArgumentList "-ResX=1280","-ResY=720","-windowed"
    foreach ($i in 1..40) {
        Start-Sleep 3
        if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) {
            Write-Host ">> sim ready" -ForegroundColor Green; return $true }
    }
    Write-Host "!! sim did not open - check the window" -ForegroundColor Red; return $false
}

function Run-Stage($num, $title, $args, $tag) {
    Write-Host ""
    Write-Host "===================================================================" -ForegroundColor Yellow
    Write-Host " STAGE $num - $title" -ForegroundColor Yellow
    Write-Host "===================================================================" -ForegroundColor Yellow
    Read-Host "Press ENTER to run stage $num"
    if (-not (Restart-Sim)) { return }
    & $PY "$ROOT\demo\run_demo.py" --policy $POL --command $CMD --tag $tag @args
    $plot = "$ROOT\demo\out\$tag\trajectory.png"
    if (Test-Path $plot) { Invoke-Item $plot }
}

Set-Location $ROOT
Write-Host "LATEST DEMO - the trained flight policy vs the old stub" -ForegroundColor White

# STAGE 1: the villain - no guardrail, watch it violate
if ($Stage -in @("1","ALL")) {
    Run-Stage 1 "No guardrail (villain): the pilot cuts through the no-fly zone" `
        @("--shield","off","--vla","stub") "latest_1_off" }

# STAGE 2: the old way - reckless stub, guardrail catches ~200 times
if ($Stage -in @("2","ALL")) {
    Run-Stage 2 "Old stub + guardrail: safe, but the shield works hard (~200 saves)" `
        @("--shield","on","--vla","stub","--dynamic") "latest_2_stub" }

# STAGE 3: the new way - trained model flies itself, shield barely touches it
if ($Stage -in @("3","ALL")) {
    Run-Stage 3 "NEW trained policy (--vla v3): the model behaves; near-zero saves" `
        @("--shield","on","--vla","v3","--dynamic") "latest_3_model" }

Write-Host ""
Write-Host "Done. Compare the three plots + reports in demo\out\latest_*" -ForegroundColor Green
Write-Host "Story: villain violates -> guardrail saves the dumb stub -> the trained model needs almost no saving." -ForegroundColor Green
