# Presentation Cheat-Sheet — VLA Guardrail (v1)

Companion to `docs/VLA-Guardrail-FineTuned-Jul2026.pptx`. Keep this open while
presenting. Three parts: **(A) per-slide talking points**, **(B) the 60-second
story**, **(C) Q&A bank** (technical + non-technical).

---

## A. Per-slide talking points (14 slides)

| # | Slide | Say this (1 line) |
|---|---|---|
| 1 | Cover | "A trained AI pilot that flies a real city — kept safe by a rule layer it can never override." |
| 2 | Pipeline | "Two new layers since last time: the pilot is now OUR fine-tuned model, and it has depth-camera eyes." |
| 3 | What's new | "Four upgrades: real 7B VLA, fine-tuning, urban maps, and the GUI + avoidance." |
| 4 | Model & inference | "7 billion parameters run on one 4080 in 5.6 GB, ~1 second per decision." |
| 5 | Fine-tuning method | "We built our own flight-data engine and trained with QLoRA — 53 minutes a run." |
| 6 | 3 runs, 2 lessons | "Run 1 got a lower loss and flew WORSE — we judge by flights, not loss." |
| 7 | Results chart | "Efficiency 0.94 → 0.996, and the safety layer had 60% less work to do." |
| 8 | Intervention ladder | "As the pilot improved, the guardrail's workload dropped 368 → 0. The rules never moved." |
| 9 | Urban flight | "Best flight: through a Japanese city, zero interventions, zero violations." |
| 10 | Depth avoidance | "The drone brakes and side-steps buildings its camera sees — before the shield." |
| 11 | GUI + one-click | "Click a target, draw a no-fly-zone, watch it fly. One batch file for the whole demo." |
| 12 | Latency budget | "On a Jetson it's ~2–4 s per decision; the architecture already absorbs that." |
| 13 | KPI + limits | "Zero critical violations everywhere. Here's what still doesn't work, honestly." |
| 14 | Conclusion | "The pilot got smart. The rules never moved." |

---

## B. The 60-second story (if you only get one minute)

> "A Vision-Language-Action model flies a drone by looking at a camera and
> reading an instruction — but it's a neural network, so it sometimes wants to
> do illegal things: enter a no-fly-zone, fly too fast, too high. Our
> **Guardrail** sits between the AI and the aircraft and validates *every single
> action* against hard rules. This period we did three things: (1) plugged in a
> **real 7-billion-parameter VLA** and **fine-tuned it ourselves** so it flies
> efficiently — path efficiency 0.996; (2) flew it in **real Japanese-city 3D
> maps** in Project AirSim; (3) added a **global planner + depth-camera
> avoidance** so it routes around buildings, with a **point-and-click GUI**. The
> key number never changed: **zero critical safety violations**, across 40+
> flights and five different simulators. The pilot got smart; the rules stayed
> exactly as strict."

---

## C. Q&A bank

### C1 — Non-technical / conceptual

**Q: What is a VLA in plain words?**
A drone brain that takes a photo + a sentence ("fly to the park, avoid the
school") and outputs how to move. "Vision-Language-Action."

**Q: Why not just trust the AI to be safe?**
Neural networks are probabilistic — occasionally wrong. Safety can't be a
best-effort of the model; it must be a hard external check. That's the Guardrail.

**Q: What's the single most important result?**
Zero P0 (critical) safety-rule escapes — no forbidden action ever reached the
aircraft — across every test, while the pilot got much better at its job.

**Q: Is this the same as autopilot / ArduPilot?**
No. ArduPilot keeps the drone stable and flies waypoints. Our layer decides
whether the AI's *intent* is legal before ArduPilot ever sees it. We never
modify the flight stack.

**Q: Could this fly a real drone?**
The architecture is deployment-ready for an NVIDIA Jetson; we've validated it in
simulation. Real-hardware validation is the next phase.

**Q: What was the hardest part?**
Making the AI both *capable* and *safe* at the same time — and proving safety
never degrades as capability grows. The intervention-ladder slide is that proof.

### C2 — Technical (methods)

**Q: Which model, and how big?**
`openvla-7b` (7B params) + AerialVLA's UAV LoRA + our QLoRA fine-tune. Runs
4-bit (NF4) in **5.6 GB VRAM** on an RTX 4080 SUPER.

**Q: Inference latency? How do you fly at 10 Hz then?**
**0.9–1.3 s per action.** The model runs in a **background thread** with its own
sim connection; the 10 Hz control loop always reads the latest action (action
holding). Blocking the loop was the original "freeze-then-jerk" bug — fixed.

**Q: How did you fine-tune, and on what data?**
QLoRA (LoRA rank 64 on LLM projections + full projector, lr 2e-5, 2 epochs,
~53 min/run). Data is **self-collected**: an expert planner flies randomized
missions in AirSim; we record front+down camera mosaics, the instruction, and
quantized action labels (99 bins: fwd 0–5 m/s, down ±5, yaw ±1.1 rad/s).
~5,500 samples across 3 datasets.

**Q: Why did run 1 fail with a lower loss?**
Its data was ~90% "fly straight," so the model **unlearned turning** — behavior
collapse. Lesson: for behavior cloning, **label distribution beats loss**. Run 2
used 48% turning samples and won. We always judge by closed-loop flight, not loss.

**Q: What exactly does the Safety Shield check?**
Every action, at every tick: no-fly-zone entry (with a 3-second lookahead
forecast), altitude band, speed/climb/yaw caps. It's trend-aware (an action
that's violating-but-correcting passes), repairs the action (speed clamp,
altitude fix, geofence slide/escape), re-checks P0, and brakes as last resort.

**Q: How do you avoid buildings?**
Two layers. **Global:** an A* planner over a surveyed **building occupancy grid**
routes waypoints around known buildings. **Reactive:** a front **depth camera**
brakes (<15 m) and side-steps + climbs (<8 m) around anything the map missed.
The Shield is still the final authority on both.

**Q: How is the building map built?**
A high-altitude lawnmower survey; the down depth camera gives roof height =
altitude − depth. Nadir-sampled at 3 m line spacing + morphological gap-fill →
an 80×80 grid at 2 m/cell. (Per-pixel footprint registration was tried but the
down-depth scale semantics smeared streets — nadir is the reliable primitive.)

**Q: What's the effective navigation rate limit?**
At 0.8 Hz, each VLA decision persists ~63° of heading change — that's why pure
VLA reaches only ~20% and we add mission-direction assist. Faster inference
(TensorRT-LLM) or waypoint-level chunking lifts this — v2 work.

**Q: How many simulators, and why so many?**
Five rails: classic AirSim (UE4), ArduPilot SITL, ROS 2 + MAVROS, Gazebo, and
Project AirSim (UE5). Same guardrail code in every one — proves the design is
adapter-isolated. KPI held (zero P0 escape) in all.

**Q: What happens if a target is unreachable (inside/behind a tall building)?**
The drone tries slow → steer → climb-to-ceiling; if the building is taller than
the altitude ceiling it **safely skips** the waypoint and continues. Rules beat
wishes — it never crashes or hangs.

### C3 — Likely challenge / "gotcha" questions

**Q: Isn't "path efficiency 0.996" just the planner, not the VLA?**
The deployed system is VLA + mission assist + planner together — that's the
product. Pure VLA is weaker (~20% reach); we report both honestly. The VLA
contributes the camera-and-language reactivity the planner alone can't.

**Q: Your occupancy map has holes — is the planner trustworthy?**
The planner handles *known* geometry; the depth-camera reactive layer + climb +
safe-skip cover the unknown. Defense in depth. And the KPI (zero violations)
never depends on the map — that's the Shield.

**Q: 1 second per action is slow for a drone.**
For high-level navigation decisions (which way to a target) it's fine — the 10 Hz
control loop and rate limiter smooth between decisions. For aggressive/agile
flight you'd want faster inference, which is a known optimization path (Jetson +
TensorRT), not a redesign.

**Q: Did you actually fly hardware?**
No — everything is simulation (five simulators). That's deliberate for a safety
layer: you validate the rule engine exhaustively in sim first. Hardware is next.

**Q: Isn't the guardrail just clamping — where's the research?**
The research is: (1) proving safety is *invariant* as capability grows
(intervention ladder), (2) the trend-aware forecast shield (not naive clamping),
(3) the auto-research + fine-tuning methodology, (4) adapter-isolation across 5
sim stacks. Clamping is the easy part; keeping it correct under a real 7B pilot
is the contribution.

---

## D. Numbers to have memorized

- **0** — P0 violation escapes (the KPI)
- **0.996** — deployed path efficiency (was 0.942)
- **−60%** — shield workload after fine-tuning (108 → 44)
- **5.6 GB** — VRAM, 4-bit 7B model on RTX 4080
- **0.9–1.3 s** — inference latency per action; **10 Hz** control loop
- **53 min** — one fine-tuning run; **~5,500** training samples
- **5** — simulators the same guardrail runs in
- **35→0** — interventions from stub to fine-tuned model in the city
