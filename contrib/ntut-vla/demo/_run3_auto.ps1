$ROOT="D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"; $PY="C:\Users\natha\.conda\envs\airsim\python.exe"
$POL="$ROOT\policies\urban_demo_policy.yaml"; $CMD="fly to (40, 40) at 6 m/s altitude 20"
$SIM="D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe"
function Sim { Get-Process -Name AirSimNH -ErrorAction SilentlyContinue | Stop-Process -Force; Start-Sleep 6
  Start-Process $SIM -ArgumentList "-ResX=1280","-ResY=720","-windowed"
  foreach($i in 1..40){Start-Sleep 3; if(Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue){return}} }
$runs=@(@("latest_1_off",@("--shield","off","--vla","stub")),
        @("latest_2_stub",@("--shield","on","--vla","stub","--dynamic")),
        @("latest_3_model",@("--shield","on","--vla","v3","--dynamic")))
foreach($r in $runs){ "==== $($r[0]) ===="; Sim; & $PY "$ROOT\demo\run_demo.py" --policy $POL --command $CMD --tag $r[0] @($r[1]) 2>&1 | Select-String "report" }
"ALL DONE"
