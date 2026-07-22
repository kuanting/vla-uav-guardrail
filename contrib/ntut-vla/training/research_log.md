# Auto-research log — deployable flight policy


## Run started 2026-07-08 00:27 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 3000 | 0.0 | 256x3 | 753984 | 0.0072 | 68.0% | 18.0% | 32.9% | 10s | fail: reach_rate=0.680 (need >=0.95); nfz_entry_rate=0.180 (need <=0.005); intervention_rate=0.329 (need <=0.05) |
  (wall 229s, teacher reach 89.7%)
| 2 | 6000 | 0.3 | 256x3 | 2014561 | 0.0030 | 43.0% | 17.5% | 17.6% | 15s | fail: reach_rate=0.430 (need >=0.95); nfz_entry_rate=0.175 (need <=0.005); intervention_rate=0.176 (need <=0.05) |
  (wall 425s, teacher reach 85.6%)
| 3 | 10000 | 0.4 | 512x3 | 5176537 | 0.0011 | 36.5% | 17.0% | 16.3% | 18s | fail: reach_rate=0.365 (need >=0.95); nfz_entry_rate=0.170 (need <=0.005); intervention_rate=0.163 (need <=0.05) |
  (wall 830s, teacher reach 76.7%)
| 4 | 16000 | 0.5 | 512x4 | 10621942 | 0.0005 | 31.0% | 18.0% | 15.3% | 16s | fail: reach_rate=0.310 (need >=0.95); nfz_entry_rate=0.180 (need <=0.005); intervention_rate=0.153 (need <=0.05) |
  (wall 1630s, teacher reach 66.8%)
| 5 | 24000 | 0.5 | 768x4 | 17640983 | 0.0004 | 28.5% | 18.5% | 10.3% | 19s | fail: reach_rate=0.285 (need >=0.95); nfz_entry_rate=0.185 (need <=0.005); intervention_rate=0.103 (need <=0.05) |
  (wall 3312s, teacher reach 62.2%)

**Schedule exhausted — best model kept (see metrics.json).**

## Run started 2026-07-08 18:02 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4000 | 0.0 | 256x3 | 1023367 | 0.0063 | 53.5% | 14.0% | 23.7% | 13s | fail: reach_rate=0.535 (need >=0.95); nfz_entry_rate=0.140 (need <=0.005); intervention_rate=0.237 (need <=0.05) |
  (wall 339s, teacher reach 88.4%)
| 2 | 8000 | 0.2 | 256x3 | 2880423 | 0.0048 | 80.0% | 14.5% | 44.0% | 9s | fail: reach_rate=0.800 (need >=0.95); nfz_entry_rate=0.145 (need <=0.005); intervention_rate=0.440 (need <=0.05) |
  (wall 837s, teacher reach 83.5%)
| 3 | 12000 | 0.3 | 512x3 | 3059982 | 0.0031 | 79.0% | 10.5% | 43.0% | 10s | fail: reach_rate=0.790 (need >=0.95); nfz_entry_rate=0.105 (need <=0.005); intervention_rate=0.430 (need <=0.05) |
  (wall 1202s, teacher reach 89.0%)
| 4 | 18000 | 0.3 | 512x4 | 4774648 | 0.0028 | 79.5% | 12.5% | 50.8% | 9s | fail: reach_rate=0.795 (need >=0.95); nfz_entry_rate=0.125 (need <=0.005); intervention_rate=0.508 (need <=0.05) |
  (wall 1814s, teacher reach 88.0%)

## Run started 2026-07-08 19:14 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4000 | 0.0 | 256x3 | 1023367 | 0.0058 | 70.5% | 9.5% | 32.3% | 11s | fail: reach_rate=0.705 (need >=0.95); nfz_entry_rate=0.095 (need <=0.005); intervention_rate=0.323 (need <=0.05) |
  (wall 346s, teacher reach 88.4%)
| 2 | 8000 | 0.2 | 256x3 | 2470303 | 0.0052 | 77.0% | 9.5% | 40.5% | 10s | fail: reach_rate=0.770 (need >=0.95); nfz_entry_rate=0.095 (need <=0.005); intervention_rate=0.405 (need <=0.05) |
  (wall 799s, teacher reach 86.7%)
| 3 | 12000 | 0.3 | 512x3 | 3228317 | 0.0027 | 80.5% | 9.5% | 49.3% | 9s | fail: reach_rate=0.805 (need >=0.95); nfz_entry_rate=0.095 (need <=0.005); intervention_rate=0.493 (need <=0.05) |
  (wall 1139s, teacher reach 88.5%)

## Run started 2026-07-08 20:01 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|

## Run started 2026-07-08 20:05 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4000 | 0.0 | 256x3 | 715838 | 0.0009 | 67.0% | 6.0% | 24.5% | 15s | fail: reach_rate=0.670 (need >=0.95); nfz_entry_rate=0.060 (need <=0.005); intervention_rate=0.245 (need <=0.05) |
  (wall 239s, teacher reach 96.8%)
| 2 | 8000 | 0.2 | 256x3 | 2447359 | 0.0013 | 91.0% | 6.0% | 38.7% | 9s | fail: reach_rate=0.910 (need >=0.95); nfz_entry_rate=0.060 (need <=0.005); intervention_rate=0.387 (need <=0.05) |
  (wall 648s, teacher reach 91.0%)
| 3 | 12000 | 0.3 | 512x3 | 2378175 | 0.0009 | 87.5% | 6.0% | 43.4% | 9s | fail: reach_rate=0.875 (need >=0.95); nfz_entry_rate=0.060 (need <=0.005); intervention_rate=0.434 (need <=0.05) |
  (wall 809s, teacher reach 96.1%)

## Run started 2026-07-08 20:35 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4000 | 0.0 | 256x3 | 705193 | 0.0009 | 71.0% | 0.5% | 20.9% | 14s | fail: reach_rate=0.710 (need >=0.95); intervention_rate=0.209 (need <=0.05) |
  (wall 229s, teacher reach 97.4%)
| 2 | 8000 | 0.2 | 256x3 | 2484547 | 0.0013 | 97.0% | 0.5% | 43.0% | 9s | fail: intervention_rate=0.430 (need <=0.05) |
  (wall 636s, teacher reach 91.1%)

## Run started 2026-07-08 20:52 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4000 | 0.0 | 256x3 | 709747 | 0.0007 | 59.5% | 1.5% | 3.4% | 19s | fail: reach_rate=0.595 (need >=0.95); nfz_entry_rate=0.015 (need <=0.005) |
  (wall 208s, teacher reach 98.0%)
| 2 | 8000 | 0.2 | 256x3 | 2940440 | 0.0010 | 97.0% | 1.5% | 4.5% | 9s | fail: nfz_entry_rate=0.015 (need <=0.005) |
  (wall 664s, teacher reach 89.2%)
| 3 | 12000 | 0.3 | 512x3 | 2252074 | 0.0005 | 89.5% | 1.5% | 4.3% | 10s | fail: reach_rate=0.895 (need >=0.95); nfz_entry_rate=0.015 (need <=0.005) |
  (wall 629s, teacher reach 97.8%)

## Run started 2026-07-08 21:21 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 4000 | 0.0 | 256x3 | 710385 | 0.0007 | 65.0% | 1.0% | 3.1% | 24s | fail: reach_rate=0.650 (need >=0.95); nfz_entry_rate=0.010 (need <=0.005) |
  (wall 200s, teacher reach 98.1%)
| 2 | 8000 | 0.2 | 256x3 | 3053241 | 0.0007 | 97.5% | 0.5% | 4.2% | 9s | PASS |
  (wall 654s, teacher reach 89.2%)

**GATE PASSED at iteration 2.**
ONNX exported: models/vla_policy_v2.onnx (Jetson-ready)

## Run started 2026-07-08 23:01 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
(warm start: DAgger pilot = vla_policy_v2_gate1.pt)
| 1 | 10000 | 0.25 | 256x3 | 1895526 | 0.0012 | 90.0% | 0.2% | 3.6% | 10s | fail: reach_rate=0.900 (need >=0.98); intervention_rate=0.036 (need <=0.025) |
  (wall 742s, teacher reach 97.2%)
| 2 | 16000 | 0.35 | 512x3 | 3899188 | 0.0008 | 96.8% | 0.5% | 3.5% | 9s | fail: reach_rate=0.968 (need >=0.98); nfz_entry_rate=0.005 (need <=0.0025); intervention_rate=0.035 (need <=0.025) |
  (wall 1433s, teacher reach 94.5%)
| 3 | 24000 | 0.4 | 512x4 | 4695140 | 0.0006 | 94.0% | 0.2% | 3.7% | 10s | fail: reach_rate=0.940 (need >=0.98); intervention_rate=0.037 (need <=0.025) |
  (wall 2006s, teacher reach 97.1%)
| 4 | 34000 | 0.45 | 768x4 | 7644249 | 0.0006 | 94.5% | 0.2% | 4.3% | 9s | fail: reach_rate=0.945 (need >=0.98); intervention_rate=0.043 (need <=0.025) |
  (wall 2844s, teacher reach 95.8%)

**Schedule exhausted — best model kept (see metrics.json).**

## Run started 2026-07-09 13:03 (device: NVIDIA GeForce RTX 4080 SUPER, 12 CPU workers)

| it | episodes | dagger | net | samples | val MSE | reach | NFZ entry | interv. | time | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
(warm start: DAgger pilot = vla_policy_v2_gate1.pt)
| 1 | 10000 | 0.25 | 256x3 | 1928195 | 0.0017 | 85.0% | 0.2% | 3.8% | 10s | fail: reach_rate=0.850 (need >=0.98); intervention_rate=0.038 (need <=0.025) |
  (wall 422s, teacher reach 98.3%)
| 2 | 16000 | 0.35 | 512x3 | 4557549 | 0.0009 | 99.5% | 0.2% | 4.3% | 9s | fail: intervention_rate=0.043 (need <=0.025) |
  (wall 889s, teacher reach 94.1%)
| 3 | 24000 | 0.4 | 512x4 | 4655626 | 0.0010 | 94.2% | 0.2% | 3.6% | 10s | fail: reach_rate=0.943 (need >=0.98); intervention_rate=0.036 (need <=0.025) |
  (wall 962s, teacher reach 98.9%)
| 4 | 34000 | 0.45 | 768x4 | 8197443 | 0.0010 | 98.2% | 0.2% | 3.5% | 9s | fail: intervention_rate=0.035 (need <=0.025) |
  (wall 1838s, teacher reach 96.6%)

**Schedule exhausted — best model kept (see metrics.json).**

---

## Campaign 2 — AerialVLA weight fine-tuning (2026-07-17)

**Goal:** specialize AerialVLA (openvla-7b + UAV LoRA) to our worlds via QLoRA on
self-collected expert flights; measured closed-loop, PURE VLA (no guidance assist).

| stage | data | result (5 eps: reached / mean eff / interv) |
|---|---|---|
| baseline (original adapter) | — | 20% / 0.211 / 368 |
| run1 epoch0 | v1: 1,567 samples, 60 eps go-to-target | 0% / 0.0 / 275 |
| run1 epoch1 (loss 1.82→0.09) | v1 | 0% / 0.0 / 150 |

**run1 verdict: REGRESSION — behavior collapse.** v1 data was ~90% fly-straight
(expert aligns once, then cruises), so the model unlearned turning: flights got
calmer (fewer interventions) but never curve toward the target. Same class of
failure as Campaign 1's DAgger lesson: BC quality = label distribution, not loss.

**Fix (v2 dataset, collecting):** retarget mid-episode every 6–12 s (forces
continuous turning) + drop 70% of near-zero-yaw samples. Next: retrain from the
ORIGINAL adapter (not run1), then closed-loop eval on the same seeds.

**Stage 2 results (2026-07-17):**

| config (5 eps, random targets) | reached | eff | interventions |
|---|---|---|---|
| baseline pure | 20% | 0.211 | 368 |
| run2 pure + yaw 0.4 | 20% | 0.049 | 144 |
| baseline deploy (blend .55/yaw .4) | 100% | 0.942 | 108 |
| **run2 deploy** | **100%** | **0.996** | **43.6** |
| run3 deploy (v3 latency-matched data) | 100% | 1.013 | 73.8 |

**Verdict:** run2/epoch1 = production model. Fine-tuning cut guardrail workload 60%
(108->44) and pushed path efficiency to 0.996 at 100% reach, NFZ 0 everywhere.
Pure-VLA closed-loop remains ~20%: 0.8 Hz inference means each decision holds for
~63 deg of heading change - not recoverable by BC on 2k samples (v3 latency-matched
data improved calmness, not pure navigation). Documented as future work: bigger
dataset + faster inference (TensorRT/streaming) or action chunking with waypoints.
Deployment manifest: models/aerialvla_deploy_manifest.json. Report:
docs/aerialvla-ft-report.md (+ docs/img/ft_loss_curve.png, ft_comparison.png).
