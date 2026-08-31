<#
.SYNOPSIS
    Drone follows a named moving object using vision and language, through the guardrail.

.DESCRIPTION
    Starts the simulator, spawns a car that drives down a real street, and flies a
    mission whose only steering input is "find the thing I named, in the camera".
    No target coordinates reach the controller.

    Records two views for the demo - the drone's own camera with the detection
    box and telemetry drawn on it, and a third-person chase view - and stitches
    them into one side-by-side video.

    Measured 2026-08-11, all three demos:
      DEMO 1  tracking          100% of the flight within 30 m, mean 13.4 m
      DEMO 2  + 3 distractors   100% within 30 m, mean 15.2 m
      DEMO 3  no-fly zone       34.9% within 30 m by design, 0 Shield overrides
    NFZ time and altitude escape are 0.0 s on every flight ever recorded.

    Takes about 18 minutes: three flights, each preceded by a simulator restart,
    then three videos. Add -Controls for two more flights.

.PARAMETER Object
    What to follow, in words. Be specific: "a car" alone makes it chase city
    clutter. The colour matters more than the noun: a unique colour is what
    turns "some car-like thing" into "THAT car".

    Default "a yellow car", which is the glTF taxi. Measured at the same pose in
    the same run, it beats the mesh subject it replaced on both halves of the
    test - detector score 0.108 against 0.047, colour gate 0.317 against 0.119 -
    and it comes with three more genuinely different-coloured vehicles, which
    the packaged path could not produce at all.

    If the models are not installed the subject falls back to the orange buggy;
    pass -Object "an orange car" then. You will not have to guess, because the
    flight prints a loud warning when the query and the subject disagree.
    Avoid "a blue car" - the asphalt here reads blue above the saturation floor.

.PARAMETER Route
    Route the subject drives. "turn" (default) goes up the street and turns left
    at the intersection onto the cross street. "straight" is the baseline every
    measured result used; quote separation figures from that one, because a
    turning route is not comparable with them.

.PARAMETER RecordHz
    Save one recorded frame every N control ticks. 1 (default) gives a real-time
    video. The previous value of 3 recorded the whole flight but played it back
    at 3x speed, which is why a 58 s flight came out as a 20 s clip.

.PARAMETER SimWidth
    Width of the simulator window, default 960.

    Lowered from 1280 on measurement. This window is a SECOND continuous render,
    on top of the Chase and FrontCamera captures, and it feeds nothing: the demo
    video is built from the Chase camera's captured frames, not from the window.
    It was costing GPU that OWL-ViT needs. In flight, inference measured 286 ms
    median against 34-56 ms on an idle GPU, which is the whole of the detector
    rate: 1 / 0.286 = 3.5 Hz, and det_hz read 3.66.

    Dropping the Chase capture from 1280x720 to 960x540 moved det_hz only
    2.94 -> 3.66, so the Chase camera was not the main thief. The window is the
    remaining one.

    Pass -SimWidth 1920 -SimHeight 1080 to watch it full size, and do not
    compare the resulting numbers with the measured runs.

    If the window opens BLANK rather than small, that was a different bug -
    two depth captures were streaming at once - and it is fixed. See
    docs/FINDING-glb-vehicles-aug15.md.

.PARAMETER SimHeight
    Height of the simulator window, default 720.

.PARAMETER Controls
    Also fly the two control conditions (wrong colour word, and no car present).

.EXAMPLE
    .\run_follow_vlm.ps1
    .\run_follow_vlm.ps1 -Controls
    .\run_follow_vlm.ps1 -SkipSim
#>
[CmdletBinding()]
param(
    [string]$Object = "a yellow car",
    [double]$CarSpeed = 2.5,
    [ValidateSet("straight","turn")]
    [string]$Route  = "turn",
    [double]$RecordHz = 20,
    [int]$RecordHeight = 540,
    [int]$SimWidth  = 960,
    [int]$SimHeight = 540,
    [switch]$SkipSim,
    [switch]$Controls,
    [switch]$NoVideo
)
# Flight durations are per-demo and set at the call sites below: the tracking run
# is sized to the car's 48 s route plus its two stops, the fenced run to the
# point where the car has escaped. A single -Seconds knob would make one of them
# wrong.

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$UE   = "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
$Map  = "/Game/JapaneseCity/Maps/Demo_day"

function Say($m) { Write-Host "==> $m" -ForegroundColor Cyan }
foreach ($p in @($Py, $UE, $Proj)) { if (-not (Test-Path $p)) { throw "not found: $p" } }
Set-Location $Root

function Test-SimUp { (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded }

function Stop-OurSim {
    # Matched on the .uproject, not the process name: killing every UnrealEditor
    # would take unrelated projects, and unsaved work, with it.
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

function Start-Sim {
    Stop-OurSim
    Start-Sleep -Seconds 4
    # PowerShell, not Git Bash: bash rewrites the /Game/... map argument into a
    # Windows path, the map is not found, and the engine crashes on the fallback.
    Start-Process -FilePath $UE -ArgumentList "`"$Proj`"", $Map, '-game', '-windowed', "-ResX=$SimWidth", "-ResY=$SimHeight" | Out-Null
    Say "simulator starting ..."
    for ($i = 0; $i -lt 90; $i++) {
        Start-Sleep -Seconds 5
        if (Test-SimUp) { Start-Sleep -Seconds 8; Say "simulator ready"; return }
    }
    throw "simulator did not open port 8989"
}

function Fly($tag, $obj, $policy, $secs, $stopS, $traffic, [switch]$NoCar, [switch]$Record) {
    # The simulator is restarted before every flight. Measured repeatedly: it
    # refuses the next connection after a flight disconnects, so reusing it
    # silently costs a run.
    if (-not $SkipSim) { Start-Sim }
    $a = @("demo\follow_vlm.py", "--object", $obj, "--tag", $tag,
           "--max-s", "$secs", "--det-thresh", "0.008",
           "--car-speed", "$CarSpeed", "--car-stop-s", "$stopS",
           "--policy", $policy, "--route", $Route, "--want-width", "0.16",
           "--record-hz", "$RecordHz", "--record-height", "$RecordHeight")
    if ($traffic -gt 0) { $a += @("--traffic", "$traffic", "--traffic-mode", "demo", "--lock-target") }
    if ($NoCar)  { $a += "--no-car" }
    if ($Record) { $a += "--save-view" }
    & $Py @a
}

# Fly() starts the simulator itself before each flight, so do not start one here
# as well - that was an extra two-minute restart before the first run.
if ($SkipSim -and -not (Test-SimUp)) { throw "-SkipSim given but nothing on 8989" }

Say "DEMO 1: follow `"$Object`" - no fence, pure tracking"
Say "        measured 100% of the flight within 30 m, on two separate flights"
Fly "demo_follow" $Object "policies\follow_car.yaml" 70 8 0 -Record

# --want-width is COUPLED to --cruise-alt. It is an angular stand-off, so the
# same value is a much larger ground distance from higher up. 0.10 suits the 9 m
# cruise these demos fly. follow_car_gap.yaml forces 13 m and needs about 0.20,
# or the aircraft holds a 30 m stand-off and the "within 30 m" metric reads that
# as failure - it scored 0.26 for that reason alone, and 0.759 once corrected.
# See docs/FINDING-gapfence-was-never-the-fence.md.
Say "DEMO 2: three MORE vehicles on the street, each looking different"
Say "        target jumping fell from 14.0% of detections to 0.4%"
Fly "demo_traffic" $Object "policies\follow_car.yaml" 70 8 3 -Record

Say "DEMO 3: same mission with a no-fly zone across the route"
Say "        the car drives through it, the drone must not"
Fly "demo_nfz" $Object "policies\follow_car_nfz.yaml" 70 8 0 -Record

if ($Controls) {
    Say "CONTROL 1: same car, WRONG colour word - should NOT follow"
    # "a red car", not "a blue car": the asphalt reads blue above the saturation
    # floor, so a blue query can score on the road itself and the control looks
    # weaker than the colour gate really is. Red has no such background overlap.
    Fly "demo_wrongcolour" "a red car" "policies\follow_car.yaml" 70 6 0
    Say "CONTROL 2: right words, NO car in the scene"
    Fly "demo_nocar" $Object "policies\follow_car.yaml" 70 6 0 -NoCar
}

if (-not $NoVideo) {
    Say "building the side-by-side demo videos"
    # --height is passed explicitly and must match -RecordHeight. The recorder
    # already wrote frames at that height; letting the builder default to 720
    # would upscale a 540-line panel and soften it for nothing.
    & $Py "tools\make_demo_video.py" "--tag" "demo_follow"  "--height" "$RecordHeight"
    & $Py "tools\make_demo_video.py" "--tag" "demo_traffic" "--height" "$RecordHeight"
    & $Py "tools\make_demo_video.py" "--tag" "demo_nfz"     "--height" "$RecordHeight"
}

Say "summary"
& $Py -c @"
import json, pathlib
tags = ['demo_follow', 'demo_traffic', 'demo_nfz',
        'demo_wrongcolour', 'demo_nocar']
# frac_ticks_seen sits next to det_hit_rate deliberately. hit rate is
# seen/(seen+missed) over the inferences that RAN, so a stalled detector
# reads 1.000 on a flight that spent 76% of its time scanning. The
# fraction of TICKS the target was actually held is the honest column.
# kpi_grade and p0_violation_escape_rate are the grant's own vocabulary. The
# escape rate is the hard KPI (target 0); kpi_grade says whether this run is
# even allowed to be quoted as a contractual number.
cols = ['tag','object','kpi_grade','p0_violation_escape_rate','det_hit_rate','frac_ticks_seen','det_hz','sep_mean_m','frac_within_30m',
        'interventions','nfz_s','alt_violation_s','frac_absent']
print('  ' + ' | '.join(f'{c:>16}' for c in cols))
for t in tags:
    f = pathlib.Path('demo/out') / t / 'metrics.json'
    if not f.exists():
        continue
    m = json.loads(f.read_text(encoding='utf-8'))
    print('  ' + ' | '.join(f'{str(m.get(c)):>16}' for c in cols))
print()
print('  sep_mean and frac_within_30m are the following metrics.')
print('  sep_min alone is not: a drone that never moves still records ~12 m,')
print('  because the car drives past it.')
"@

Stop-OurSim
Say "done."
Write-Host "    videos  : demo\out\demo_follow\demo_follow_demo.mp4   (tracking, car stops twice)"
Write-Host "              demo\out\demo_traffic\demo_traffic_demo.mp4 (picks one of four identical cars)"
Write-Host "              demo\out\demo_nfz\demo_nfz_demo.mp4         (guardrail holds it outside)"
Write-Host "    metrics : demo\out\<tag>\metrics.json"
