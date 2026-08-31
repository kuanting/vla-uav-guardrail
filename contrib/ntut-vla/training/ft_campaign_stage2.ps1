# ft_campaign_stage2.ps1 - sequential remainder of the fine-tuning campaign.
# Runs unattended; each step logs to training\campaign2.log.
$ROOT = "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
$PY   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$LOG  = "$ROOT\training\campaign2.log"

function Step($name, $cmdArgs) {
    Add-Content $LOG "=== $(Get-Date -Format HH:mm:ss) START $name ==="
    & $PY @cmdArgs *>> $LOG
    Add-Content $LOG "=== $(Get-Date -Format HH:mm:ss) END $name (exit $LASTEXITCODE) ==="
}

Set-Location $ROOT
Set-Content $LOG "campaign2 stage2 start $(Get-Date)"

# 1. deployment-config evals (goal assist + yaw damping) for baseline and run2
Step "eval baseline-deploy" @("training\eval_aerialvla.py", "--adapter", "D:/models/aerialvla-lora/aero_vla", "--name", "base_deploy", "--episodes", "5", "--goal-blend", "0.55", "--yaw-gain", "0.4")
Step "eval ft2-deploy" @("training\eval_aerialvla.py", "--adapter", "D:/models/aerialvla-ft/run2/epoch1", "--name", "ft2_deploy", "--episodes", "5", "--goal-blend", "0.55", "--yaw-gain", "0.4")

# 2. v3 data: expert decides at VLA latency (12 ticks), softer yaw
Get-Process -Name AirSimNH -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep 4
Start-Process "D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe" -ArgumentList "-ResX=960","-ResY=540","-windowed"
foreach ($i in 1..40) { Start-Sleep 3; if (Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) { break } }
Step "collect v3" @("training\collect_aerialvla_data.py", "--episodes", "60", "--seed", "21", "--out", "$ROOT\dataset\aerialvla_ft_v3", "--decide-every", "12", "--yaw-p", "0.7", "--yaw-max", "0.7")
Get-Process -Name AirSimNH -ErrorAction SilentlyContinue | Stop-Process -Force

# 3. retrain from ORIGINAL adapter on v3
Step "train run3" @("training\finetune_aerialvla.py", "--epochs", "2", "--lr", "2e-5", "--run", "run3", "--data", "$ROOT\dataset\aerialvla_ft_v3")

# 4. evals of run3: pure and deployment config
Step "eval run3-pure" @("training\eval_aerialvla.py", "--adapter", "D:/models/aerialvla-ft/run3/epoch1", "--name", "ft3_pure", "--episodes", "5")
Step "eval run3-deploy" @("training\eval_aerialvla.py", "--adapter", "D:/models/aerialvla-ft/run3/epoch1", "--name", "ft3_deploy", "--episodes", "5", "--goal-blend", "0.55", "--yaw-gain", "0.4")

Add-Content $LOG "=== CAMPAIGN STAGE2 COMPLETE $(Get-Date) ==="
