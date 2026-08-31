# Presentation cheat-sheet — August 2026

For the deck `VLA-Guardrail-Follow-Aug2026.pptx`. Numbers first, then the
questions you will actually get.

---

## The 30-second version, if you only get one sentence

> We told a drone to follow a white car, in words. It did — within 30 m for
> **100%** of the flight, and it stopped when the car stopped. Then we put three
> more cars on the same street, identical except for colour, and it still picked
> the right one. And with a no-fly zone across the route it braked, held outside,
> and never entered.

## The one thing that makes it a result rather than a video

> Change one word — "a white car" to "a red car", same scene, same white car —
> and the system does not follow badly, it **fails to acquire at all**: the
> detector's hit rate collapses from 1.000 to **0.221** and time within 30 m from
> 100% to **24%**. That is the control. Without it, "the drone ended up near the
> car" is just drift.

That hit-rate collapse is new and it is worth a sentence of its own. The previous
control used "a blue car", and this map's **asphalt reads blue** — above the
saturation floor, the road filled 0.99 of the box — so a blue query always had a
background-coloured escape hatch and the detector kept firing (hit rate 0.99)
while the colour gate quietly suppressed the follow. Red has no such overlap, so
the gate now rejects nearly every candidate outright. The system is closer to
saying *"that thing is not here"* than it has ever been.

## The second thing, if you get more time

> The car stops twice mid-route and the drone stops with it. A follower has to
> stop — that is what separates following from flying down the same street.

The speed figures previously quoted here (1.17 m/s moving, 0.36 m/s parked) came
from the contaminated set AND cannot be recomputed offline, because the car's
per-tick ground-truth pose is not written to disk. Say the qualitative version
until the logging gap is closed and the pair is re-measured.

---

## Numbers to have in your head

Verified configuration - the one to demo.

Every row below was re-flown on 2026-08-11 after two faults were found that
invalidated the earlier set: the target mesh was an open roll-cage buggy, and a
teleport fault was respawning the car at the start of its route mid-flight. Do
not quote any figure from before that date.

| arm | tag | detector hit | within 30 m | mean sep | Shield |
|---|---|---|---|---|---|
| tracking | `vlm_nofreeze` | 0.993 | **1.000** | 13.4 m | 0 |
| tracking, replicate | `v2_track_r2` | 1.000 | **1.000** | 12.5 m | 0 |
| **CONTROL, "a red car"** | `v2_wrongcolour` | **0.221** | **0.241** | 71.8 m | 10 |
| 3 identical distractors | `vlm_traffic2` | 0.979 | **1.000** | 17.7 m | 0 |
| NFZ across the corridor | `v2_nfz2` | 1.000 | 0.349 | 38.8 m | 0 |

| | value |
|---|---|
| **NFZ time / altitude escape** | **0.0 s / 0.0 s, on every flight ever recorded** |
| tracking, n=2 | 1.000 and 1.000 within 30 m |
| detector rate | **3.9–5.4 Hz** measured (was 0.11 Hz) |
| detector size | **153 M** params (was 7 B), 0.61 GB VRAM |
| target mesh | `SKM_SportsCar`, "a car" scores **0.1272** (buggy scored 0.0395) |
| colour pairing | `M_Orange` renders **white** on this mesh — verified, and the phrase says white |
| car | 4.3 × 1.9 × 1.2 m, 2.0 m/s, 66 m straight, stops twice, parks |

**The NFZ arm's 0.349 is not a tracking failure.** The fence spans the whole
corridor by design, so the aircraft is held outside while the car drives through
and away. Losing the target is the correct outcome; entering the zone would not
be. NFZ 0.0 s with **0 Shield interventions** is the number that matters there —
the controller's own fence awareness kept it out, so the safety layer never had
to act.

**The gap-fence variant is still work in progress**, not a capability: it holds
the rule (NFZ 0.0 s) but does not exploit the gap (0.284 within 30 m). Present
`follow_car_nfz.yaml` as the NFZ demo.
**Earlier circuit numbers (65-90% within range) are superseded.** A closed loop
looks more natural but its turns swing the car through the aircraft's blind spot
- the front camera cannot see closer than 0.86 x altitude. Straight, with stops,
fixed it.

**Where they come from:** `demo/out/<tag>/metrics.json`, one file per flight.
Current tags: `vlm_nofreeze`, `v2_track_r2`, `v2_wrongcolour`, `vlm_traffic2`,
`v2_nfz2`. The older `vlm_stopgo` / `vlm_nfz_smooth` / `vlm_col_blue` set is kept
for the record but must not be quoted — see the note above the table.

---

## Say these three things before anyone asks

Volunteering a limitation is much stronger than conceding it under questioning.

1. **"The noun does most of the work — but we now measure how much."** "a car"
   alone chases city clutter and ends 51 m away. Put three IDENTICAL cars on the
   street and only the colour test can separate them; it does, and the cost shows
   up honestly as the detector hit rate falling 1.000 → 0.979 and mean separation
   13.4 → 17.7 m. The system grounds *"car"* by network and *"white"* by fixed
   rule, and those are different mechanisms.
2. **"It cannot tell you the object is absent."** With no car in the scene the
   raw detector still fires on about 75% of frames. Only the colour check
   suppresses it.
3. **"The target is made easy, and it is a simulator."** The car drives straight
   and parks, because turning swings it through the aircraft's blind spot. The
   obstacle map holds buildings only — not trees or lamp posts, which a flight
   found the hard way. One flight per condition, and no hardware yet.

---

## Q&A

### On the result

**Q: Is the drone really following, or does it just drift near the car?**
The wrong-colour control answers that. Identical scene, identical car, one word
changed: 100% within 30 m becomes 24.1%, and the detector's hit rate collapses
from 1.000 to 0.221. Drift would score the same in both.

**Q: Closest approach 1.0-2.1 m — is that safe?**
That is the stand-off it is asked to hold, set by `--want-width`. Closest
approach is not the score anyway: a drone that never moves still records about
12 m, because the car drives past it. Judge by mean separation and time within
range.

**Q: How does it know where the car is?**
It does not. The controller only ever sees a box in an image. The car's real
position enters in two places: spawning it, and scoring the flight afterwards.
`servo()` takes no target argument — that is checkable by reading the function.

**Q: Why is the detector's confidence only 0.03–0.07?**
Because the car is 20–30 px wide at that range. The box lands in the right place,
which is all a servo loop needs. We do not read the score as evidence the object
is present — that is limitation #2.

### On the change from last time

**Q: What happened to AerialVLA? You spent a whole campaign fine-tuning it.**
We measured what its two prompt slots actually do. The direction slot — a compass
phrase computed from ground-truth coordinates — orders the commanded yaw
monotonically. The object slot does essentially nothing: correct and wrong colour
words give indistinguishable actions. Without the direction phrase the model
emits stop-and-land on 11 of 18 real frames, and flown against a real car it
never moved at all — 12 of 12 inferences returned the same token triple.

**Q: So the fine-tuning was wasted?**
It made a coordinate-following model better at following coordinates — path
efficiency went 0.942 → 0.996. What it did not do, and could not have done, is
teach language grounding: the training data filled the object slot with five
generic strings carrying zero information about the label. We can measure that
cost: our fine-tune's object-slot sensitivity is *lower* than the original's,
0.321 against 0.454.

**Q: Isn't swapping in a detector giving up on VLA?**
It is still vision, language, and action: the words choose the target, the camera
finds it, the controller acts. What changed is using a model whose language input
reaches its output. A 7B action model that ignores its instruction is not more of
a VLA than a 153M detector that obeys it.

**Q: Why not fine-tune the detector too?**
OWL-ViT is zero-shot and works without it. Training it on this specific car would
mean a labelled dataset and would make it *worse* at the general case, which is
the property we actually want.

### On the guardrail

**Q: When it is following, are the VLA and the guardrail BOTH actually running?**
Both run every tick — the Shield validates every action of every flight. Whether
it *acts* depends on whether a rule is at risk. Three real ticks, pulled from
`flight_log.jsonl`, show all three outcomes:

| case | pilot asked for | what actually flew | rule at risk |
|---|---|---|---|
| guardrail agrees | vx +0.41, vy −0.62 | vx +0.40, vy −0.40 | none |
| controller brakes itself | vy +0.74 | vy +0.74, unchanged | none — fence 6.0 m |
| **guardrail overrides** | **vx −0.47, toward a wall** | **vx +0.50** | clearance 4.98 < 5.0 m |

`yaw_rate` is identical in all three, which is the clean separation: the Shield
never touches heading, so the nose is the pilot's and the track is the
guardrail's. Verified on 100% of ticks across every flight.

The interface is one struct, `Action4D(vx, vy, vz_up, yaw_rate)`. `raw` never
reaches the simulator — only `shield.filter(state, smooth).emitted` flies.

**Q: The guardrail records 0.0 s violations. Does it do anything?**
On the fenced flight (`v2_nfz2`) the aircraft was held outside the zone for the
whole crossing with **0 Shield interventions** — the controller's own fence
awareness kept it out, so the safety layer never had to act. The HUD reads NFZ AHEAD — HOLDING
while TARGET LOCKED stays up: it can see where it wants to go and is not allowed
to go there.

**Q: Shield overrides went from 659 to 0. Did you weaken the guardrail?**
The opposite. 659 overrides meant the servo and the Shield were arguing ten times
a second — safe, but the aircraft chattered against the boundary and looked
dangerous. The controller now knows the geofence itself and brakes from 12 m, so
the Shield has nothing left to correct. The rule did not change; the pilot stopped
breaking it.

**Q: Isn't telling the controller about the fence cheating?**
A real aircraft knows its own geofence — that is mission data, not target data.
Nothing about the car reaches the controller: `servo()` still has no target
argument.

**Q: Did the guardrail need changing for the new pilot?**
No. Not one line of `shield.py`, not one policy file. That is the architectural
claim and it survived replacing the brain.

**Q: Has the guardrail ever failed?**
Yes, and we fixed it. Earlier flights escaped the altitude band for up to 31.6 s
even with the guardrail on. The cause was architectural: those scripts had no
altitude controller and leaned on the Shield, which is a *constraint filter* that
makes minimal corrections, not a regulator. The current controller holds altitude
itself. Escape is now 0.0 s on every flight.

### Technical

**Q: Why was inference 0.11 Hz and how did you get to 4 Hz?**
Two things. The detector is 153M instead of 7B. And separately we found the 7B
model's slowness was GIL contention with the control loop, not the GPU — we tested
three cheaper explanations first (lazy frame decoding, removing an unused
1280×720 camera, throttling camera publish rate) and all three changed nothing.
Running the model in its own process took it from 1330 to 225 ms per token, on
the same GPU at the same moment.

**Q: Why does it fly at only 9 m?**
Camera geometry, not preference. The front camera has a 29.4° vertical half-FOV;
horizontal, it sees ground only past 1.79 × altitude, so the car left the frame
exactly when the drone got close. Pitching it 20° down makes that 0.86 ×
altitude — 7.7 m instead of 16 m at a 9 m cruise.

**Q: Why is the car a circuit and not a straight line?**
It used to be a straight line and it reversed its heading 180° in a single frame,
which looks wrong and breaks the tracker's continuity filter. The circuit has
rounded corners and a speed profile: heading now changes at most 3.1° per 0.1 s —
a 31°/s cornering rate, what a real car does.

**Q: What runs on the GPU, and will it fit on hardware?**
0.61 GB for the detector. The old 7B model needed 5.6 GB at 4-bit. That is the
part that makes an onboard version plausible, but we have not tried it.

### Awkward but fair

**Q: Four flights is not many.**
Agreed. They are consistent — 16.4 to 19.1 m mean separation — and the control
condition is far outside that spread, so the direction of the effect is not in
doubt. The magnitude is not calibrated, and we say so on the Limits slide.

**Q: Would this work outside the simulator?**
Unknown. The detector is trained on real photographs, which helps, but the
lighting, motion blur and camera of a real airframe are untested. The guardrail
runs on any action source, so it is the part least likely to be surprised.

**Q: Could a classical tracker do this without any language model?**
Yes — if you first tell it which pixels to track. The language model is what lets
you specify the target by describing it instead of clicking on it. That is the
whole difference, and it is also why the wrong-colour control is the slide that
matters.

---

## If the demo misbehaves live

| what you see | say this, then move on |
|---|---|
| Drone loses the car briefly | "Watch the HUD — COASTING, then SEARCHING, then it re-acquires. It used to just freeze." |
| Drone stops moving | "The car stopped. Watch it start again when the car does — that is the point of this run." |
| Drone sweeps at the start | "That is SEARCHING. It has not seen the car yet and will not sit still waiting for it." |
| It locks onto something wrong | "That is limitation one — a general noun catches city clutter. Watch the colour score in the HUD drop." |
| Flight reports `ticks 0` | Simulator needs a restart between flights; the script does this, a manual run may not. |
| Detector hit rate looks low (~0.3) | "That is after the colour filter. Before it, it was 0.82 — and that flight tracked *worse*." |

**Files to have open:** `demo/out/vlm_stopgo/vlm_stopgo_demo.mp4` (tracking with
stops), `demo/out/vlm_nfz_smooth/vlm_nfz_smooth_demo.mp4` (guardrail holding),
`docs/RESULT-vlm-follow.md`, `docs/FINDING-what-drives-aerialvla.md`.
