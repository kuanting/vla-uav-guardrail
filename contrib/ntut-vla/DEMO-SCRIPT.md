# DEMO SCRIPT — Live Demonstration Guardrail VLA Drone
Durasi total: ±12–15 menit (slide 5 menit + live demo 7 menit + tanya jawab).
Semua perintah sudah dites. Fallback tersedia di tiap langkah.

---

## PERSIAPAN (H-1, wajib gladi resik sekali)

- [ ] Charge laptop / colok listrik (AirSim = GPU berat, jangan battery-saver)
- [ ] Tutup aplikasi berat lain (game, browser banyak tab)
- [ ] Font **Poppins** terpasang (untuk deck)
- [ ] Buka `docs/VLA-Guardrail-Progress-Jul2026.pptx` → cek video slide 8 bisa play
- [ ] Gladi resik: jalankan `demo\live_demo.ps1` sekali penuh (lihat bawah)
- [ ] Siapkan fallback: folder `demo/out/` berisi semua plot + video hasil run sebelumnya —
      kalau live demo gagal total, tunjukkan ini

## PERSIAPAN (30 menit sebelum demo)

- [ ] Jalankan `AirSimNH.exe` sekali → pastikan drone muncul, tutup lagi (warm-up shader cache)
- [ ] Buka 1 terminal PowerShell, siapkan:
  ```powershell
  conda activate airsim
  cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
  ```
- [ ] Set layar: proyektor duplicate (bukan extend) supaya audiens lihat sim window

---

## BABAK 1 — Slide (±5 menit)

Slide 1–5 dari deck. Poin per slide:
1. **Cover** — perkenalkan diri + konteks kerja sama dua tim
2. **Problem** — "VLA itu pintar tapi kadang salah; keselamatan tidak boleh bergantung pada model yang kadang salah"
3. **Approach** — tunjuk diagram: "kami TIDAK menyentuh flight stack; hanya menambah lapisan sebelum & sesudah VLA"
4. **What's built** — "semua sudah jalan, 16 unit test pass"
5. **Four rails** — "guardrail yang sama persis diuji di 4 lingkungan, hasil identik"

Lalu bilang: **"Daripada percaya slide, mari lihat langsung."** → Babak 2.

---

## BABAK 2 — LIVE DEMO (±7 menit)

Jalankan helper (otomatis start/restart sim + run + buka hasil):
```powershell
cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
powershell -ExecutionPolicy Bypass -File demo\live_demo.ps1
```
Helper berhenti di tiap tahap, menunggu Enter. Tiga tahap:

### Tahap A — Shield OFF (villain-nya) ±2 menit
Sambil drone terbang, katakan:
> "VLA diberi perintah: *fly to (40,40) at 6 m/s*. Jalur lurusnya menembus
> zona larang terbang di atas blok perumahan. TANPA guardrail — lihat, dia
> masuk begitu saja."

Saat plot muncul: tunjuk garis biru menembus kotak merah. **"3.4 detik di dalam
zona terlarang. KPI: FAIL."**

### Tahap B — Shield ON (hero-nya) ±2.5 menit
> "VLA yang sama, perintah yang sama, kenakalan yang sama. Sekarang guardrail aktif."

Saat drone menyusuri tepi zona: **"Perhatikan — dia tidak berhenti, tidak gagal
misi. Dia MENYUSURI tepi zona lalu lanjut ke target. ~200 intervensi, nol rem
darurat, nol detik di dalam zona."**

Saat mendarat pelan: "Pendaratan juga terkontrol, 0.7 m/s."

### Tahap C — Dynamic NFZ (wow moment) ±2.5 menit
> "Sekarang skenario paling menarik: di detik ke-8, GCS atau sistem computer
> vision tiba-tiba menutup area — persis di jalur drone."

Saat terminal print `dynamic NFZ applied -> generation 1`:
**"Aturan berubah SAAT TERBANG. Policy generation naik, hash berubah — semua
tercatat di audit log — dan drone langsung memutar. Tetap nol pelanggaran."**

### Penutup babak 2
Buka `demo/out/live_C/report.md` + `audit.jsonl` sebentar:
> "Setiap run menghasilkan bukti: plot, laporan KPI, dan audit log yang setiap
> barisnya membawa hash policy — bisa direplay dan diaudit."

---

## BABAK 3 — Tutup (±2 menit)

Kembali ke deck slide 9–10: temuan riset + next steps + Thank You.

---

## JIKA GAGAL (fallback, urutan)

1. **Sim crash / tidak connect** → helper punya retry; kalau tetap gagal:
   tutup semua `AirSimNH.exe` (Task Manager), jalankan helper lagi dari tahap yang gagal
   (`demo\live_demo.ps1 -Stage B`)
2. **AirSim rusak total** → pindah ke WSL satu perintah (autopilot asli, tanpa grafis):
   ```powershell
   wsl bash "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/sitl/run_ros2_demo.sh" on --dynamic
   ```
   Narasi: "ini malah lebih kuat — ArduPilot sungguhan, arsitektur ROS 2 final"
3. **Semua gagal / tidak ada waktu** → video slide 8 + plot di `demo/out/` —
   "ini rekaman run yang sama, live-nya bisa saya tunjukkan setelah sesi"

## Pertanyaan yang mungkin muncul (+jawaban singkat)

| Pertanyaan | Jawaban |
|---|---|
| "VLA-nya model apa?" | Stub controller (sengaja nakal) — slot-nya contract-locked, model asli (OpenVLA dll) tinggal drop-in tanpa ubah guardrail. Integrasi model asli = fase berikutnya. |
| "Kenapa drone goyang sedikit di tepi zona?" | Tarik-tambang VLA vs Shield 10×/detik — sudah diredam rate-limiter (4.8→0.5 m/s). Solusi fundamental = PathRepair, jadwal Q3. Justru jadi data metrik *mean repair magnitude*. |
| "Kalau drone menabrak gedung?" | Obstacle fisik = domain tim flight-stack (obstacle avoidance). Guardrail menangani aturan/policy. Pembagian sesuai kickoff. |
| "Bisa jalan di drone asli?" | Jalur sudah terbukti sampai ArduPilot asli (SITL) via MAVLink & ROS 2 — tinggal ganti SITL dengan firmware + companion computer. Desain grant: Jetson Orin. |
| "Aturannya siapa yang menulis?" | YAML, human-readable — operator/regulator bisa menulis tanpa coding. Ada validasi schema otomatis. |
