<#
.SYNOPSIS
    Runs the whole "follow the car" experiment end to end.

.DESCRIPTION
    Starts the simulator, spawns and photographs the car, flies the mission twice
    (once with no direction hint, once with the coordinate-derived hint the model
    was trained on), and scores both.

    Expected result, which is the finding rather than a fault: with no hint the
    drone does not move at all. With the hint it moves but ends further from the
    car than a drone that never took off.

.PARAMETER SkipSim
    Use a simulator that is already running instead of restarting it.

.PARAMETER ProbeOnly
    Spawn the car, photograph it, and stop. No VLA is loaded.

.PARAMETER Seconds
    Flight duration per run. Default 120.

.EXAMPLE
    .\run_follow_car.ps1
    .\run_follow_car.ps1 -ProbeOnly
    .\run_follow_car.ps1 -SkipSim -Seconds 90
#>
[CmdletBinding()]
param(
    [switch]$SkipSim,
    [switch]$ProbeOnly,
    [int]$Seconds = 120,
    [string]$Adapter = "D:/models/aerialvla-lora/aero_vla",
    [string]$Object  = "orange car"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$UE   = "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
$Map  = "/Game/JapaneseCity/Maps/Demo_day"

function Say($msg) { Write-Host "==> $msg" -ForegroundColor Cyan }
function Warn($msg) { Write-Host "!!  $msg" -ForegroundColor Yellow }

foreach ($p in @($Py, $UE, $Proj)) {
    if (-not (Test-Path $p)) { throw "not found: $p" }
}
Set-Location $Root

function Test-SimUp {
    (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded
}

function Stop-OurSim {
    # Match on the .uproject, NOT on the process name. Killing every
    # UnrealEditor takes unrelated projects - and unsaved work - with it.
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

function Start-Sim {
    Stop-OurSim
    Start-Sleep -Seconds 4
    # Launched from PowerShell on purpose: Git Bash rewrites the /Game/... map
    # argument into a Windows path, the map is not found, and the engine crashes
    # on the fallback map.
    Start-Process -FilePath $UE -ArgumentList `
        "`"$Proj`"", $Map, '-game', '-windowed', '-ResX=1280', '-ResY=720' | Out-Null
    Say "simulator starting, waiting for port 8989 ..."
    for ($i = 0; $i -lt 90; $i++) {
        Start-Sleep -Seconds 5
        if (Test-SimUp) { Start-Sleep -Seconds 6; Say "simulator ready"; return }
    }
    throw "simulator did not open port 8989 within ~7.5 minutes"
}

if (-not $SkipSim) { Start-Sim }
elseif (-not (Test-SimUp)) { throw "-SkipSim given but nothing is listening on 8989" }

# ---------------------------------------------------------------- 1. the car --
Say "spawning the car and photographing it"
& $Py "experiments\probe_moving_car.py"
Say "OPEN THESE and check the car is an orange box on a road:"
Write-Host "    experiments\out\car_probe\rng_22_front.png"
Write-Host "    experiments\out\car_probe\zoom_22.png"
if ($ProbeOnly) { Say "probe only - stopping here"; exit 0 }

# ------------------------------------------------- 2/3. the two flights --
$common = @(
    "--follow-car",
    "--object", $Object,
    "--adapter", $Adapter,
    "--policy", "policies\follow_car.yaml",
    "--cruise-alt", "9",
    "--start-beta-deg", "0",
    "--max-s", "$Seconds"
)

Say "flight 1/2: camera and language only (--hint-mode none). Expect NO motion."
if (-not $SkipSim) { Start-Sim }
& $Py "demo\semantic_seek.py" @common "--hint-mode" "none"  "--tag" "follow_none"

Say "flight 2/2: with the coordinate-derived hint (--hint-mode truth). Expect motion, but not following."
if (-not $SkipSim) { Start-Sim }
& $Py "demo\semantic_seek.py" @common "--hint-mode" "truth" "--tag" "follow_truth"

# ------------------------------------------------------------- 4. scoring --
Say "scoring both flights"
& $Py "experiments\analyze_semantic_ab.py" "--conditions" "experiments\conditions_follow_car.yaml"

Say "what the model actually emitted with no hint:"
& $Py -c "import json; r=[json.loads(l) for l in open('demo/out/follow_none/inference.jsonl',encoding='utf-8')]; print('  prompt:', repr(r[0]['prompt'])); print('  bins  :', [x['bins'] for x in r])"

Say "separation to the car (the following metric):"
& $Py -c @"
import csv
rows = list(csv.DictReader(open('experiments/out/comparison_table.csv', encoding='utf-8')))
cols = ['tag','sep_min_m','sep_mean_m','sep_end_m','frac_within_30m','n_both','viol_any_s']
print('  ' + ' | '.join(f'{c:>16}' for c in cols))
for r in rows:
    if r['tag'].startswith('follow'):
        print('  ' + ' | '.join(f'{r[c]:>16}' for c in cols))
print()
print('  sep_min is NOT success: a drone that never moves still scores ~12 m')
print('  because the car drives past it. Read sep_mean and frac_within_30m.')
"@

Stop-OurSim
Say "done. Table: experiments\out\comparison_table.md"
