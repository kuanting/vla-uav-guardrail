# Models & Large Assets — download guide

Model weights, UE5 maps, and datasets are **intentionally NOT in git** (they are
gigabytes). This file tells you exactly where to get them and where to put them,
so the code in `contrib/ntut-vla/` runs without shipping the heavy files.

## Model weights

| Model | Source | Local path used by the code | Size |
|---|---|---|---|
| OpenVLA-7B (base) | HuggingFace `openvla/openvla-7b` | `D:\models\openvla-7b` | ~15 GB |
| AerialVLA UAV LoRA | HuggingFace `XuPeng23/AerialVLA` (`aero_vla/`) | `D:\models\aerialvla-lora\aero_vla` | ~0.4 GB |
| **Our QLoRA fine-tune** (production) | produced by `training/finetune_aerialvla.py` | `D:\models\aerialvla-ft\run2\epoch1` | ~0.4 GB (adapter) |
| Trained BC policy (state-based) | produced by `training/train_bc.py` | `models/vla_policy_v2.pt` / `.onnx` | ~12 MB |

Download the base + LoRA (Windows, symlink-free):

```powershell
$env:HF_HUB_DISABLE_SYMLINKS = "1"
huggingface-cli download openvla/openvla-7b   --local-dir D:\models\openvla-7b
huggingface-cli download XuPeng23/AerialVLA    --local-dir D:\models\aerialvla-lora
```

Reproduce our fine-tune (needs a collected dataset, see below):

```powershell
python training\collect_aerialvla_data.py --episodes 60 --out dataset\aerialvla_ft_v2
python training\finetune_aerialvla.py --epochs 2 --lr 2e-5 --run run2 --data dataset\aerialvla_ft_v2
```

Deploy config (measured): path efficiency **0.996**, shield workload **−60%**,
P0 violations **0**. See `docs/aerialvla-ft-report.md`.

## Simulator & maps (not in git)

| Asset | Where | Size |
|---|---|---|
| Project AirSim UE5 project (JapaneseCity, Airport, Military) | `PASBlocks/` | ~26 GB |
| Project AirSim Python client | `D:\ProjectAirSim\repo\client\python\projectairsim` | — |
| Classic AirSim worlds (AirSimNH, Blocks, …) | `D:\AirSim\` | 10s of GB |
| City occupancy maps (per world) | `demo/out/citymap/occ_<map>.npz` | small, but regenerable |

Rebuild an occupancy map for a world. **Preferred: ground-truth voxel grid**
(queries the sim geometry directly — 1:1 accurate, solid blocks + real streets):

```powershell
# start the sim on that world first, then:
python demo\build_voxel_map.py --out occ_day     # -> occ_day.npz (+ occ.npz)
```

Legacy camera survey (`survey_city.py`) also exists but is approximate (nadir
sampling leaves holes and can misregister); use `build_voxel_map.py` when the
world exposes `create_voxel_grid` (Project AirSim does).

## Datasets (not in git)

Self-collected flight datasets live under `dataset/` (~1 GB). Regenerate with
`training/collect_aerialvla_data.py`. The eval JSONs and logs that ARE in git
(`training/eval_*.json`, `research_log.md`, `ft_loss_log.csv`) capture the
results without the raw data.

## Runtime environment

conda env `vla-real`: torch 2.6.0+cu124, transformers 4.40.1, tokenizers 0.19.1,
timm 0.9.16, peft 0.11.1, accelerate, bitsandbytes 0.49, numpy 1.26.4,
projectairsim client. (numpy must stay <2 — opencv tends to pull it up; re-pin
after any pip install.)
