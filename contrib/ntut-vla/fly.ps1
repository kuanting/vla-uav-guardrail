# fly.ps1 - one command: launch AirSim world + run the VLA drone mission.
#
# Examples:
#   .\fly.ps1                                   # urban world, shield ON
#   .\fly.ps1 -Shield off                       # watch it violate
#   .\fly.ps1 -Dynamic                          # NFZ appears mid-flight
#   .\fly.ps1 -World mountains                  # mountain world
#   .\fly.ps1 -Command "fly to (10, 30) at 5 m/s altitude 18"
#   .\fly.ps1 -KeepSim                          # reuse running sim (faster, but
#                                               # drone starts where it landed)
param(
    [ValidateSet("nh", "blocks", "mountains", "zhangjiajie")]
    [string]$World = "nh",
    [ValidateSet("on", "off")]
    [string]$Shield = "on",
    [switch]$Dynamic,
    [string]$Command = "",
    [string]$Policy = "",
    [string]$Tag = "",
    [switch]$KeepSim
)

$ROOT = "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
$PY   = "C:\Users\natha\.conda\envs\airsim\python.exe"

$SIMS = @{
    nh          = "D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe"
    blocks      = "D:\AirSim\Blocks\WindowsNoEditor\Blocks.exe"
    mountains   = "D:\AirSim\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe"
    zhangjiajie = "D:\AirSim\ZhangJiajie\WindowsNoEditor\ZhangJiajie.exe"
}
$PROCS = @{
    nh = "AirSimNH"; blocks = "Blocks"
    mountains = "LandscapeMountains"; zhangjiajie = "ZhangJiajie"
}

# world-appropriate defaults: urban policy over the neighborhood, generic
# sim policy (10-20 m band) elsewhere
if (-not $Policy) {
    $Policy = if ($World -eq "nh") { "$ROOT\policies\urban_demo_policy.yaml" }
              else                 { "$ROOT\policies\sim_demo_policy.yaml" }
}
if (-not $Command) {
    $Command = if ($World -eq "nh") { "fly to (40, 40) at 6 m/s altitude 20" }
               else                 { "fly to the northeast pad at 6 m/s" }
}
if (-not $Tag) {
    $Tag = "fly_{0}_{1}{2}" -f $World, $Shield, $(if ($Dynamic) { "_dynamic" } else { "" })
}

# ---- 1. simulator up ----
$running = Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue
if ($KeepSim -and $running) {
    Write-Host ">> reusing running simulator (-KeepSim)" -ForegroundColor Yellow
} else {
    Write-Host ">> starting $World world..." -ForegroundColor Cyan
    # kill ANY AirSim world so worlds never fight over the RPC port
    Get-Process -Name $PROCS.Values -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep 5
    Start-Process $SIMS[$World] -ArgumentList "-ResX=1280", "-ResY=720", "-windowed"
    $ok = $false
    foreach ($i in 1..40) {
        Start-Sleep 3
        if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) {
            $ok = $true; Write-Host ">> sim ready ($($i*3)s)" -ForegroundColor Green; break
        }
    }
    if (-not $ok) { Write-Host "!! sim never opened port 41451 - check the window" -ForegroundColor Red; exit 1 }
}

# ---- 2. mission ----
Write-Host ">> mission: '$Command'  shield=$Shield dynamic=$($Dynamic.IsPresent)" -ForegroundColor Cyan
$args2 = @("$ROOT\demo\run_demo.py", "--shield", $Shield, "--policy", $Policy,
           "--command", $Command, "--tag", $Tag)
if ($Dynamic) { $args2 += "--dynamic" }
& $PY @args2
if ($LASTEXITCODE -ne 0) { Write-Host "!! mission script failed" -ForegroundColor Red; exit 1 }

# ---- 3. show result ----
$plot = "$ROOT\demo\out\$Tag\trajectory.png"
if (Test-Path $plot) { Invoke-Item $plot }
Write-Host ">> artifacts: demo\out\$Tag\  (trajectory.png, report.md, audit.jsonl, frames\)" -ForegroundColor Green
