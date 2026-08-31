# The colour gate was measuring brightness, not colour

**Date:** 2026-08-15
**Trigger:** "ketika berada di per 4 an atau intersection (sekitar detik ke 8
setelah object berhenti) object detection sempat kehilangan object padahal object
ada di depannya."

The dropout is real, it happened at the same place every flight, and the cause
was a single hard-coded comparison. Fixing it took the detector hit rate from
0.73 to **1.000**.

This file also records a wrong diagnosis I acted on, because it was wrong in an
instructive way.

---

## 1. What it was not

Three explanations were tested and killed:

* **not occlusion** — projecting the car's ground truth into the camera puts it
  at u ≈ 200 of 400, dead centre and in frame, for the whole episode;
* **not the camera's near blind spot** — the car never projects below the bottom
  of the frame, and the losses persist at 15–18 m where the 7.7 m limit is
  irrelevant;
* **not the detector failing** — replaying the saved frames, OWL-ViT scored the
  taxi **0.17–0.30**, the strongest scores of the flight, on every single frame
  it was "lost".

`experiments/why_the_lock_dropped.py` settles it in one line:

    colour-gate rejections: 25   score rejections: 0   would pass: 0

The detector found the car 25 times out of 25. The colour gate threw it away
every time.

## 2. The wrong diagnosis, and why it survived a first look

The first explanation was that the zebra crossing **diluted** the yellow
fraction: a box around a car standing on white stripes is mostly stripes, so the
fraction of the box that is yellow collapses. I measured 0.148–0.172 on plain
asphalt against 0.027–0.042 at the intersection and treated that as the cause.

It was an artefact. Those boxes were **projected by hand** from ground truth with
a guessed size; the real detector's boxes are placed differently, and the moment
I replayed the actual boxes the dilution story stopped fitting.

The tell was available and I missed it: dilution cannot be the whole story
because **white stripes do not remove yellow pixels**. They change what else is
in the box, not how much of the car is yellow. A number that moved 4x therefore
had to come from somewhere else.

## 3. What it actually was

Measured on the car's own pixels, in the two scheduled stops of one flight:

| | hue in range | of those, pass `s > 90` | resulting score |
|---|---|---|---|
| stop 1, in shade | 0.321 | **56.7%** | 0.182 — passes the 0.10 gate |
| stop 2, in sunlight | 0.213 | **8.1%** | **0.017 — rejected** |

Median saturation of the yellow pixels: **99 in shade, 69 in sun**.

HSV saturation is `chroma / brightness`. Sunlight raises the denominator, so the
brighter a surface is lit the more absolute colour it needs to clear a fixed `S`
floor. The threshold `s > 90` was tuned on a car in shade and rejected the *same
car* when the sun came out. The gate was, in effect, measuring how brightly the
target was lit.

**A depth mask would not have fixed this.** The car's own pixels were failing, so
isolating them more accurately changes nothing. That mask was built before the
real cause was known; it is kept because it is a strictly better measurement, but
it is **off by default** and has never been shown to help in flight.

## 4. The fix

Absolute chroma — `max(R,G,B) − min(R,G,B)` — in place of the saturation ratio.
It does not deflate under bright light, and for uint8 HSV it is exactly
`s·v/255`, so it costs no extra colour conversion.

| region | chroma > 40 | old `s > 90` |
|---|---|---|
| car, in shade | 0.285 | 0.182 |
| **car, in sunlight** | **0.174** | 0.017 |
| zebra crossing + road | 0.0004 | 0.0000 |
| plain road | 0.0000 | 0.0000 |

The gate still rejects the background completely. It just stops rejecting the
target for being well lit.

## 5. Measured in flight

Six flights, arms interleaved, `--colour-legacy-sat` restoring the old floor.

| arm | detector hit rate | false-ABSENT | mean separation | target switches | lost-lock episodes |
|---|---|---|---|---|---|
| **chroma (new)** | **1.000, 1.000, 1.000** | 0%, 2%, 0% | 13.0 m | 1, 1, 1 | **0** |
| saturation (old) | 0.735, 0.758, 0.696 | 8%, 9%, 7% | 14.6 m | 4, 4, 4 | 1 |

The arms do not overlap. The effect is 0.270 against a within-arm SD of 0.031 —
roughly 8.7 SD — and a hit rate of 1.000 is higher than any configuration
measured on this demo, the previous best being 0.798.

Guardrail unchanged: 0 interventions, NFZ 0.0 s, altitude 0.0 s on all six.

**A perfect hit rate is also what a broken gate looks like**, so discrimination
was checked rather than assumed:

* **target switches fell from 4 to 1** and the jump rate stayed at 0.0%. A gate
  that had stopped discriminating would wander more, not less.
* **Wrong-colour control.** Asked for "a green car", a colour no vehicle in the
  fleet has: verdict ABSENT on **54%** of ticks with the new gate against 52%
  with the old — unchanged — and it stayed 28 m from the taxi instead of locking
  it. The colour word still selects.
* That control run shows 56 Shield interventions, all `bld-clearance`, with
  NFZ 0.0 s and no braking: the aircraft wandered toward a building while
  chasing something that does not exist and the guardrail pushed it off. That is
  the guardrail working on a degenerate mission, not a regression.

## 6. What is guarded against a repeat

`tests/test_range_and_lock.py` gains a fixture that lights the same synthetic car
two ways, with the washout tuned to the saturation actually measured in flight
(median 69 against 99). One test asserts the car passes under both lightings;
another pins the mechanism by asserting that a saturation floor *would* have
failed on it. A future change back to `s > 90` fails in the suite instead of at
an intersection.

## Reproducing

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\why_the_lock_dropped.py demo\out\demo_follow --t0 27.5 --t1 36.0 --object "a yellow car"
C:\Users\natha\.conda\envs\vla-real\python.exe demo\follow_vlm.py --object "a yellow car" --tag ctrl --colour-legacy-sat ...
```

## 7. A metric that flattered a stalled flight

Found while checking the demo re-run rather than shipping it. One `demo_traffic`
flight reported `det_hit_rate 1.000` and was a bad flight: **29 inferences at
0.52 Hz**, with the target actually held on **13.6%** of ticks and 76% of the
flight spent scanning.

`det_hit_rate` is `seen / (seen + missed)` over the inferences that RAN. When the
detector stalls it approaches 1.000 by doing almost nothing, which is the
opposite of what the number is read as. It has appeared in every summary table in
this repo.

Two changes, neither of which touches the flight:

* the report prints a loud warning below 2 Hz, stating that the hit rate is not a
  tracking result and the flight is unrepresentative;
* the summary table now carries `frac_ticks_seen` and `det_hz` beside
  `det_hit_rate`.

The stall was not caused by the colour change - the six A/B flights on identical
code produced 234, 228 and 314 detections - and the re-run reproduced healthy
flights (3.85, 5.62 and 4.14 Hz). It is transient contention, and it now
announces itself instead of being averaged into a result.

## 8. Final demo set

| tag | hit rate | ticks tracking | det Hz | mean sep | within 30 m | guardrail |
|---|---|---|---|---|---|---|
| demo_follow | 1.000 | 1.000 | 3.85 | 13.8 m | 1.000 | 0 / 0.0 s / 0.0 s |
| demo_traffic | 1.000 | 0.998 | 5.62 | 13.1 m | 1.000 | 0 / 0.0 s / 0.0 s |
| demo_nfz | 1.000 | 1.000 | 4.14 | 38.9 m | 0.368 | 0 / 0.0 s / 0.0 s |

`demo_nfz` keeps its distance by design: the fence holds the aircraft outside
while the car drives on through the zone.
