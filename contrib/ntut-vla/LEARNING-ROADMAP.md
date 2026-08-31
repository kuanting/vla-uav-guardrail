# Learning & Build Roadmap — VLA Drone Guardrail

Roadmap pribadi untuk mengerjakan proyek dari nol. Centang `[x]` saat selesai.
Asumsi: solo (tanpa pembagian tugas), simulasi-first (tanpa hardware), mulai dari nol
(belum pernah Python/ROS/ArduPilot), OS Windows 11.

**Prinsip utama:** dapatkan "kemenangan" cepat dengan Python murni dulu (inti Guardrail),
baru tambahkan lapisan simulasi yang lebih berat. Jangan belajar semuanya sekaligus —
belajar *saat dibutuhkan* oleh tahap yang sedang dikerjakan.

---

## FASE 0 — Persiapan lingkungan kerja (Minggu 1)

> Tujuan: punya tempat menulis & menjalankan kode, dan menyimpan pekerjaan dengan rapi.

- [ ] Pasang **Python 3.11+** (cek `python --version`)
- [ ] Pasang **VS Code** + extension: Python, Pylance, YAML
- [ ] Pasang **Git**; buat akun **GitHub**; pelajari dasar `clone / add / commit / push`
- [ ] Buat 1 repository proyek (mis. `vla-drone-guardrail`) untuk semua kode Anda
- [ ] Belajar pakai **virtual environment** (`python -m venv`, `pip install`)
- [ ] (Penting untuk nanti) Pasang **WSL2 + Ubuntu 22.04** di Windows
      → ROS 2 & ArduPilot SITL jauh lebih mulus di Linux. Cukup *dipasang* dulu,
      belum dipakai sampai Fase 4.

---

## FASE 1 — Pemahaman konsep (Minggu 1, paralel dgn Fase 0)

> Tujuan: paham "kenapa", bukan cuma "bagaimana". Banyak ini sudah kita bahas di chat.

- [ ] Pahami **hierarchical control**: VLA beri setpoint (10 Hz) → ArduPilot stabilkan (100–400 Hz). VLA tidak pegang motor.
- [ ] Pahami peran tiap komponen: **ArduPilot/PX4, MAVLink, ROS 2/MAVROS, Geofence**
- [ ] Pahami apa itu **VLA**: gambar + perintah bahasa → aksi `(vx, vy, vz, yaw_rate)`
- [ ] Pahami alur Guardrail: `Constraint → Compiler → VLA → Hard Validation → MAVLink`
- [ ] Baca ulang 7 PDF grant (Overview, Policy DSL, Prefix Compiler, Safety Shield, Stress Testing) — itu spesifikasi target Anda
- [ ] Tulis 1 halaman ringkasan dengan kata-kata sendiri (cara terbaik mengecek pemahaman)

---

## FASE 2 — Python secukupnya untuk membangun (Minggu 1–2)

> Tujuan: bisa baca/tulis kode Python untuk logika & data. Tidak perlu jago dulu.

- [ ] Dasar: variabel, tipe data, list, dict, if/for/while, fungsi
- [ ] Class & objek (OOP dasar) — penting karena spesifikasi pakai banyak class
- [ ] Modul & import, struktur file proyek
- [ ] Baca/tulis file; tangani error (try/except)
- [ ] **PyYAML** — baca/tulis file YAML (format utama proyek)
- [ ] **Pydantic v2** — validasi data terstruktur (dipakai di SEMUA komponen grant)
- [ ] **NumPy** dasar — vektor & operasi numerik (untuk aksi 4-D)
- [ ] Tulis program kecil: baca file YAML berisi aturan → validasi dengan Pydantic → cetak

---

## FASE 3 — Inti Guardrail dengan Python murni ⭐ — ✅ SELESAI 2026-07-03

> Selesai LEBIH dari target: bukan cuma Python murni, tapi sudah terbang di AirSim.

- [x] Modelkan **constraint** sebagai Pydantic → `guardrail/models.py` (3 kelas: polygon_fence, altitude_envelope, kinematic_envelope)
- [x] Tulis file YAML aturan contoh → `policies/demo_policy.yaml` + `sim_demo_policy.yaml`
- [x] **Geometri** Shapely → `guardrail/geometry.py`
- [x] Cek envelope ketinggian/kecepatan → `guardrail/shield.py`
- [x] **VLA stub** → `guardrail/vla_stub.py` (P-controller, sengaja reckless)
- [x] **Hard Validation** → `Shield.filter()` (trend-aware: melanggar-tapi-memperbaiki = lolos)
- [x] **Repair + brake**: SpeedClamp / AltitudeFix / GeofenceSlide / GeofenceEscape / Brake
- [x] **Logging JSONL** dengan policy_hash → `guardrail/audit.py`
- [x] 15 skenario uji, semua PASS → `tests/test_shield.py`
- [x] **DEMO A/B di AirSim** → `demo/run_demo.py`: shield OFF = 3.5 s di NFZ (FAIL), shield ON = 0 s (PASS) ✅
- [x] BONUS: **Constraint Compiler** (NL → YAML prompt) → `guardrail/compiler.py`

> Pelajaran penting yang ditemukan: (1) brake saat SUDAH melanggar = deadlock → butuh recovery ops;
> (2) head-on slide = stall → butuh tangent bias; (3) AirSim `reset()` bikin command-deaf;
> (4) settings.json harus di `D:\OneDrive\Dokumen\AirSim\` (Documents di-redirect OneDrive).

---

## FASE 4 — Stack simulasi — SEBAGIAN BESAR ✅ (2026-07-03)

> Guardrail kini terbang lewat ArduPilot SUNGGUHAN (SITL), bukan cuma AirSim.
> Bukti kunci: paket guardrail TIDAK berubah satu baris pun — hanya adapter bawahnya.

- [x] **ArduPilot SITL**: built dari source di WSL (`~/ardupilot`, binary `build/sitl/bin/arducopter`) → `sitl/setup_sitl.sh` + `start_sitl.sh`
- [x] **MAVLink / pymavlink**: `sitl/run_sitl_demo.py` — heartbeat, GUIDED, arm/takeoff state-machine, `SET_POSITION_TARGET_LOCAL_NED` @ 10 Hz
- [x] Guardrail → SITL tersambung: A/B + dynamic semua jalan (OFF 3.7s FAIL / ON 0s PASS / dynamic 0s PASS)
- [x] (bonus) AirSim rail sudah jalan duluan (Fase 3) — dua rail seperti di grant: fungsional (SITL) + persepsi (AirSim)
- [x] **ROS 2 Jazzy** terpasang (WSL Ubuntu 24.04, `/opt/ros/jazzy`, venv `~/venv-ros` --system-site-packages)
- [x] **MAVROS 2**: `ros2 run mavros mavros_node -p fcu_url:=tcp://127.0.0.1:5760`
- [x] Guardrail = **ROS 2 node**: `sitl/ros2_shield_node.py` (sub `/vla/action_4d` + pose ENU, pub Twist setpoint_velocity; bring-up via services set_mode/arming/takeoff/set_stream_rate) + `ros2_vla_stub_node.py` (VLA slot = node terpisah, swappable). Hasil: OFF 3.7s FAIL / ON 0s PASS / dynamic 0s PASS — identik dgn 2 rail lain. `run_ros2_demo.sh` = orkestrator.
- [x] **Gazebo Harmonic 8.14** + `ardupilot_gazebo` plugin (built dari source; butuh libopencv-dev): `sitl/setup_gazebo.sh` + `run_gazebo_demo.sh`. Headless server (`gz sim -s -r iris_runway.sdf`) + SITL `--model JSON`. Hasil: OFF 3.8s FAIL / ON 0s PASS / dynamic 0s PASS. **FASE 4 SELESAI PENUH.**
- [ ] Klarifikasi varian "AirSim" ke Prof. Lai (original 1.8.1 dipakai sekarang / Colosseum / Project AirSim)

> Gotcha SITL yang ditemukan: (1) SITL exit saat stdin EOF → `tail -f /dev/null |` menahan stdin;
> (2) tanpa GCS tidak ada stream → wajib `request_data_stream_send`; (3) arm/takeoff bisa ditolak
> saat EKF belum siap → state-machine retry via heartbeat; (4) `pkill -f arducopter` membunuh
> dirinya sendiri (cmdline match) → pakai `[a]rducopter`; (5) restart SITL antar-run (posisi nempel).

---

## FASE 5 — Verification System lengkap & KPI (setelah Fase 4)

> Tujuan: otomatis menjalankan banyak skenario & menghasilkan angka KPI (untuk laporan final).

- [ ] Definisikan skenario sebagai YAML (tiru `ScenarioSpec` di Stress Testing.pdf)
- [ ] Harness yang menjalankan banyak skenario otomatis (parameter sweep)
- [ ] Hitung KPI: **P0 violation escape rate (target 0)**, success rate, mean time to safe
- [ ] Simpan "episode bundle" yang bisa diputar ulang (reproducible)
- [ ] Buat laporan KPI otomatis (Markdown + JSON)

---

## FASE 6 — Deliverable untuk grant & rapat (jalan terus)

> Sebagian ini deadline dekat — saya (Claude) bisa bantu langsung membuatnya.

- [ ] **≤30 Jun**: flowchart + dokumen perencanaan untuk dibagikan ke tim (bisa saya bantu)
- [ ] **≤30 Jun**: Architecture Diagram v2 (tandai bagian baru vs. dimodifikasi)
- [ ] **≤3 Jul**: jadwalkan konsultasi teknis dgn Prof. Dai & Prof. Lai
- [ ] **≤5 Jul**: definisikan interface ke Computer Vision & Dynamic No-Fly-Zone (format, frekuensi, sinkronisasi)
- [ ] **≤12 Jul**: prototipe Verification System (= hasil Fase 3) + demo blok/validasi
- [ ] Draft laporan awal (untuk demo mid-year)
- [ ] Siapkan pertanyaan untuk konsultasi: "bagian flight stack mana yang boleh diubah?"

---

## Jalur kritis (kalau waktu mepet, kerjakan ini saja dulu)

1. Fase 0 (setup) → 2. Python + Pydantic + YAML (Fase 2) → 3. Inti Guardrail Python murni (Fase 3).
**Itu cukup untuk prototipe 12 Juli & demo mid-year.** Sisanya (ROS/SITL/Gazebo/AirSim) adalah Q3+.

## Catatan keputusan terbuka (klarifikasi ke Prof. Lai)
- "AirSim" yang dimaksud: AirSim asli (sudah tak di-maintain), Project AirSim (komersial), atau fork Colosseum?
- Spesifikasi PC lab (ada GPU NVIDIA?) — menentukan kelayakan AirSim/Unreal.
- Apakah tim Anda juga menjalankan VLA-nya, atau ada tim VLA terpisah?
- Demo mid-year: cukup diagram+konsep, atau perlu sesuatu yang "jalan"?
