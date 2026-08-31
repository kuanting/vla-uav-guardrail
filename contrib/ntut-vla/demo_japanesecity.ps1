# demo_japanesecity.ps1 - ONE-CLICK demo: fine-tuned VLA + guardrail on Project
# AirSim's urban maps (PASBlocks repo content). Starts the sim, waits, flies,
# pops the trajectory plot.
#
# Usage:
#   .\demo_japanesecity.ps1                              # JapaneseCity day (default)
#   .\demo_japanesecity.ps1 -Map night                   # JapaneseCity night
#   .\demo_japanesecity.ps1 -Target "60,20"              # choose the destination
#   .\demo_japanesecity.ps1 -Route "40,40; 70,0; 0,-30"  # multi-waypoint patrol
#   .\demo_japanesecity.ps1 -Map airport|military|blocks # other worlds
# (or just double-click run_japanesecity.bat)
param(
    [ValidateSet("day","night","airport","military","blocks")] [string]$Map = "day",
    [string]$Target = "",     # single destination "x,y" (default: 40,40)
    [string]$Route  = ""      # waypoints "x1,y1; x2,y2; ..." (wins over -Target)
)

$ROOT    = "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
$PY      = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$UE      = "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe"
$UPROJ   = "$ROOT\PASBlocks\Blocks.uproject"
$ADAPTER = "D:/models/aerialvla-lora/aero_vla"      # ORIGINAL, not our fine-tune
$MAPS = @{ day      = "/Game/JapaneseCity/Maps/Demo_day"
           night    = "/Game/JapaneseCity/Maps/Demo_night"
           airport  = "/Game/Airport/Maps/demo"
           military = "/Game/MilitaryAirport/Maps/Map_Airbase_Demo"
           blocks   = "" }
$TAG = "oneclick_$Map"

Write-Host "ONE-CLICK DEMO - fine-tuned AerialVLA + guardrail on Project AirSim" -ForegroundColor White
Write-Host "  map     : $Map  $($MAPS[$Map])" -ForegroundColor Gray
Write-Host "  model   : $ADAPTER" -ForegroundColor Gray

# 1. fresh sim server
Write-Host ">> starting Project AirSim server (map load ~2 min, be patient)..." -ForegroundColor Cyan
Get-Process -Name UnrealEditor,AirSimNH -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep 3
$ueArgs = @("`"$UPROJ`"")
if ($MAPS[$Map]) { $ueArgs += $MAPS[$Map] }
$ueArgs += @("-game","-windowed","-ResX=1280","-ResY=720")
Start-Process $UE -ArgumentList $ueArgs

$ready = $false
foreach ($i in 1..60) {
    Start-Sleep 6
    $p = Get-Process -Name "UnrealEditor" -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $p) { Write-Host "!! sim died - check UE install / map name" -ForegroundColor Red; exit 1 }
    $ports = Get-NetTCPConnection -State Listen -OwningProcess $p.Id -ErrorAction SilentlyContinue |
             Select-Object -ExpandProperty LocalPort
    if ($ports -contains 8989) { $ready = $true; break }
}
if (-not $ready) { Write-Host "!! server port 8989 never appeared" -ForegroundColor Red; exit 1 }
Write-Host ">> sim ready - loading the 7B model (~30 s) and flying..." -ForegroundColor Green

# 2. fly the fine-tuned model through the guardrail
Set-Location $ROOT
$flyArgs = @("$ROOT\demo\aerialvla_pas_demo.py", "--best", "--adapter", $ADAPTER,
             "--tag", $TAG, "--map-label", $Map)
if ($Route)      { $flyArgs += @("--route", $Route) }
elseif ($Target) { $flyArgs += @("--command", "fly to ($Target) at 6 m/s altitude 20") }
& $PY @flyArgs

# 3. show the result
$plot = "$ROOT\demo\out\$TAG\trajectory.png"
if (Test-Path $plot) { Invoke-Item $plot }
Write-Host ""
Write-Host "done - artifacts in demo\out\$TAG  (sim window left open for a look)" -ForegroundColor Green
Read-Host "ENTER to close"
