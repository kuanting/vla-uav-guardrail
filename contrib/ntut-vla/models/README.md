# Model — VLA Flight Policy (`vla_policy_v2`)

Ringkasan lengkap: apa modelnya, bagaimana dibuat, angka-angkanya, dan cara pakai.

---

## 1. Apa modelnya

**Policy penerbangan hasil training (neural network)** yang mengisi slot VLA di
pipeline Guardrail. Input = keadaan drone + geometri zona larang terbang
(26 fitur); output = aksi kecepatan `(vx, vy, vz)` @ 10 Hz. Model *belajar
sendiri* menghindari no-fly zone, berhenti di target, dan mematuhi batas
kecepatan/ketinggian — Shield hampir tidak pernah perlu turun tangan lagi.

Bukan VLA berbasis kamera (itu tangga berikutnya) — ini policy state+geometri
(VLA-lite). Slot dan metodologinya sama persis untuk upgrade ke versi kamera.

| File | Untuk apa |
|---|---|
| `models/vla_policy_v2.pt` | TorchScript — dipakai di sim/PC |
| `models/vla_policy_v2.onnx` | ONNX 558 KB — **siap deploy Jetson Orin** (onnxruntime) |
| `models/vla_policy_v2.metrics.json` | Sertifikat metrik + catatan deploy |
| `models/vla_policy_v2_gate1.*` | Backup model generasi pertama |

## 2. Angka final (evaluasi 400 skenario held-out)

| Metrik | Nilai | Artinya |
|---|---|---|
| Reach rate (TANPA shield) | **99.5%** | hampir selalu sampai target sendirian |
| Masuk NFZ (TANPA shield) | **0.25%** (1/400) | aturan sudah "terinternalisasi" |
| Intervensi Shield | **0.04%** | Shield praktis menganggur |
| Reach DENGAN shield | **100%** | dipakai sebagaimana mestinya: sempurna |

Validasi live AirSim (kota + zona dinamis muncul di detik-8): sampai target
18 detik, 0 pelanggaran, 0 rem darurat, KPI PASS.

## 3. Bagaimana model ini dibuat (pipeline auto-research, semuanya di `training/`)

```
scenarios.py       pabrik misi acak (dijamin winnable)
generate_v2.py     rollout offline paralel 12-core; guru = PLANNER (visibility
                   graph + Dijkstra, planner_expert.py) difilter Shield;
                   DAgger: model mengunjungi state, guru melabeli
auto_research.py   loop otomatis: generate → train (RTX 4080, TF32+AMP)
                   → evaluasi → gate → eskalasi/ekspor
evaluate_policy.py gerbang mutu (400 episode, shield OFF = ujian sejati)
dashboard.py       monitor live http://127.0.0.1:8085
research_log.md    buku catatan semua run
```

3 kampanye, 9 run. Temuan kunci (kronologis): label DAgger harus dari ahli;
fitur geometri penuh; ahli harus deterministik (label multimodal = racun MSE);
benchmark harus winnable; zona dinamis butuh jarak peringatan; murid == guru
(paritas) → ganti guru medan-potensial dengan planner (plafon 96.8→98.5%);
sisa intervensi 100% altitude → decoder sadar-ketinggian (4.3%→0.04%, tanpa
retraining).

## 4. Cara pakai

### A. Di demo AirSim (paling mudah)
```powershell
# 1. jalankan D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe, tunggu drone muncul
conda activate airsim
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
python demo\run_demo.py --shield on --vla v3 --policy policies\urban_demo_policy.yaml --command "fly to (40, 40) at 6 m/s altitude 20"
# --vla v3  = model ini;  --vla stub = pembanding lama;  tambah --dynamic untuk zona mendadak
```

### B. Dari Python langsung
```python
from guardrail import load_policy
from guardrail.compiler import ConstraintCompiler
from guardrail.vla_bc import BCVLAv3

policy  = load_policy("policies/urban_demo_policy.yaml")
mission = ConstraintCompiler(policy).parse_command("fly to (40, 40) at 6 m/s altitude 20")
vla     = BCVLAv3(mission, policy)          # memuat models/vla_policy_v2.pt

action = vla.act(state)                     # state -> Action4D (vx, vy, vz_up, yaw_rate)
# panggil vla.refresh_fences(policy) setelah hot-apply zona baru
```

### C. Deploy di drone (Jetson Orin) — ONNX
```python
import onnxruntime as ort
sess = ort.InferenceSession("vla_policy_v2.onnx")
y = sess.run(None, {"features": feats_26dim})[0]        # feats: training/features.py::encode_v3
vx, vy, vz = decode_action(*y[0], up=alt, alt_min=15, alt_max=25)   # WAJIB pakai decoder!
```
**Kontrak deploy:** (1) fitur harus dibuat dengan `encode_v3` (26-dim, sama
persis dengan training); (2) output WAJIB melalui `decode_action` dengan band
ketinggian (bounded action head = bagian dari model); (3) **Shield tetap
menyala** di deployment nyata — 0.25% sisa itulah alasan guardrail ada.

### D. Latih ulang / tingkatkan
```powershell
conda activate vla-drone
python training\dashboard.py        # terminal 1: monitor live :8085
python training\auto_research.py    # terminal 2: loop otomatis sampai gate
```

## 5. Batasan jujur
- Input = state + geometri zona (bukan kamera); frame lokal meter (WGS84 nanti).
- Zona = poligon persegi axis-aligned (sesuai generator skenario).
- Ada sim2sim gap: intervensi live AirSim > offline (tracking lag) — normal,
  terdokumentasi; validasi bertingkat offline→sim→nyata memang diperlukan.
