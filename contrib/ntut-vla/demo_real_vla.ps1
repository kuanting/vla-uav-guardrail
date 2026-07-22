# demo_real_vla.ps1 - fly the REAL OpenVLA-7B through the guardrail in AirSim.
#
# Usage:
#   .\demo_real_vla.ps1                                        # default instruction
#   .\demo_real_vla.ps1 -Instruction "fly toward the buildings"
#   .\demo_real_vla.ps1 -World blocks                          # nh | blocks | mountains
#
# (If PowerShell blocks the script:  powershell -ExecutionPolicy Bypass -File demo_real_vla.ps1)
param(
    [string]$Instruction = "fly forward and avoid restricted areas",
    [ValidateSet("nh","blocks","mountains")] [string]$World = "nh"
)

$ROOT = "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
$PY   = "C:\Users\natha\.conda\envs\vla-real\python.exe"   # NOTE: vla-real env, NOT airsim
$SIMS = @{ nh="D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe";
           blocks="D:\AirSim\Blocks\WindowsNoEditor\Blocks.exe";
           mountains="D:\AirSim\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe" }

Write-Host "REAL VLA DEMO - OpenVLA-7B (4-bit) in the guardrail slot" -ForegroundColor White
Write-Host "  model : D:\models\openvla-7b  (loads in 2-5 min, ~4.7 GB VRAM)" -ForegroundColor Gray
Write-Host "  task  : $Instruction" -ForegroundColor Gray

# 1. fresh simulator (drone must start from spawn)
Write-Host ">> restarting simulator ($World)..." -ForegroundColor Cyan
Get-Process -Name AirSimNH,Blocks,LandscapeMountains -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep 5
Start-Process $SIMS[$World] -ArgumentList "-ResX=1280","-ResY=720","-windowed"
$ready = $false
foreach ($i in 1..40) {
    Start-Sleep 3
    if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) {
        Write-Host ">> sim ready" -ForegroundColor Green; $ready = $true; break }
}
if (-not $ready) { Write-Host "!! sim did not open - check the window" -ForegroundColor Red; exit 1 }

# 2. run the real VLA through the guardrail
Set-Location $ROOT
& $PY "$ROOT\demo\real_vla_demo.py" --instruction $Instruction --tag real_vla

Write-Host ""
Write-Host "done - audit log in demo\out\real_vla\audit.jsonl" -ForegroundColor Green
