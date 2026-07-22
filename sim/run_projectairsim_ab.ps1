# Project AirSim A/B demo — launches the UE5 world, runs shield off then on.
# Run from PowerShell:
#     .\sim\run_projectairsim_ab.ps1
# Or from the repo:  powershell -ExecutionPolicy Bypass -File sim\run_projectairsim_ab.ps1
# Assumes the Project AirSim Neighborhood world is installed at the path below.

$ErrorActionPreference = "Stop"
$Root = Resolve-Path "$PSScriptRoot\.."
$PasPy = "$env:USERPROFILE\.conda\envs\pas\python.exe"
$WorldExe = "D:\ProjectAirSim\Neighborhood\Neighborhood-Windows-UE5.2-PAS_v0.2.0\AirSimNH\Binaries\Win64\AirSimNH-Win64-Shipping.exe"
$WorldDir = "D:\ProjectAirSim\Neighborhood\Neighborhood-Windows-UE5.2-PAS_v0.2.0"
$StartedOurselves = $false

function Wait-Port($port) {
    for ($i = 0; $i -lt 60; $i++) {
        if ((Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue)) {
            return $true
        }
        Start-Sleep -Seconds 5
    }
    return $false
}

# Start the world if port 8990 isn't already listening.
if (-not (Get-NetTCPConnection -LocalPort 8990 -State Listen -ErrorAction SilentlyContinue)) {
    Write-Host "[airsim] starting Project AirSim Neighborhood world ..."
    Start-Process -FilePath $WorldExe -WorkingDirectory $WorldDir
    $StartedOurselves = $true
    if (-not (Wait-Port 8990)) {
        Write-Error "Project AirSim world did not open port 8990"; exit 1
    }
}
Write-Host "[airsim] world ready (port 8990 listening)"

Set-Location $Root
Write-Host "`n===== RUN A: shield OFF =====`n"
& $PasPy -m demo.projectairsim_demo --shield off --tag airsim_shield_off

Write-Host "`n===== RUN B: shield ON =====`n"
& $PasPy -m demo.projectairsim_demo --shield on --tag airsim_shield_on

Write-Host "`n===== done — see episodes\airsim_shield_{off,on}\ ====="
if ($StartedOurselves) {
    Write-Host "[airsim] stopping world"
    Get-Process -Name "AirSimNH-Win64-Shipping" -ErrorAction SilentlyContinue | Stop-Process -Force
}
