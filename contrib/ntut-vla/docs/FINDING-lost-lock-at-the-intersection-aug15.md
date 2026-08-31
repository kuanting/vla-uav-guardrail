# The 360 at the intersection: what actually caused it

**Date:** 2026-08-15
**Trigger:** "ketika masuk di per 4 an atau intersection, object detection seperti
hilang ... sehingga drone membuat manuver 360 dan sampai menemukan kembali
objectnya."

The observation is correct. My first explanation was wrong, and the biggest
cause turned out to be a bug that had been quietly corrupting every flight
comparison in this repo — including numbers reported earlier the same day.

---

## 1. Not occlusion

The obvious reading is that a street tree hides the car at the intersection. It
is wrong, and the flight log says so without ambiguity.

`experiments/check_target_occlusion.py` projects the car's ground-truth position
into the camera model (400x225, 90 deg HFOV, pitched 20 deg down). At the moment
detection stops:

| | u (of 400) | range | presence |
|---|---|---|---|
| episode 1 | **200.9** | 21.1 m | PRESENT |
| episode 2 | ~200 | 22.0 m | PRESENT |

Dead centre, three times the 7.7 m near blind spot, and the system's own
presence monitor still says the object is there. Nothing was in front of it.

## 2. The spin is what turns a 2-second miss into a 16-second one

Traced tick by tick, episode 1:

| t | mode | car at pixel u | psi |
|---|---|---|---|
| 19.5 | track | 199 | 89.4 |
| 19.8-21.3 | coast | 199-201 | 89.0 |
| 21.6 | **search** | 201 | 88.8 |
| 22.8 | search | 247 | 74.6 |
| 24.0 | search | 337 | 51.0 |
| 24.3 | search | **361** (edge is 400) | 46.1 |

The detector misses a centred car for two seconds. Then `search` starts, and it
was implemented as:

```python
side = 1.0 if last_bearing >= 0 else -1.0
yaw_rate, fwd, bearing = side * args.search_rate, 0.0, 0.0
```

A **constant sign**. Despite the comment calling it a sweep, the nose rotates and
never comes back — and it rotates away from a bearing that was still correct. In
episode 2 the car went from u = 318 to u = **-425**, i.e. behind the aircraft.
Only then is it genuinely unfindable, so the search continues: 292 degrees,
16.3 seconds.

`fwd = 0.0` compounds it. The car pulled away 26.3 -> 35.5 m during one episode,
shrinking the target and making recovery harder the longer recovery took.

## 3. But the dominant cause was an uncontrolled takeoff heading

Found while trying to A/B the fix, and it invalidates more than it fixes.

The code meant to point the aircraft down the street before handing over called
`move_by_velocity_async(0, 0, 0, yaw_is_rate=False, yaw=psi0)`, which does not
turn the aircraft, and then continued regardless. Two runs of the *identical*
demo:

| run | start heading | bearing to car | relative | outcome |
|---|---|---|---|---|
| A | 60.3 deg | 83.2 deg | **+22.9 deg**, in frame | hit 0.740, sep 17.5 m, within-30 m 0.946 |
| B | 135.2 deg | 83.2 deg | **-52.1 deg**, outside the 45 deg half-FOV | hit 0.331, sep 54.8 m, within-30 m 0.033 |

Run B never saw the car for 11.5 s. By then it had driven 23 m away and the
flight never recovered.

**That effect is larger than any change being tested.** Every single-flight
comparison in this repo is suspect to the extent it was decided by this lottery,
and the traffic numbers in `RESULT-glb-fleet-flights-aug15.md` were quoted from
one flight per arm. The vehicle conclusion may still hold; the evidence offered
for it was weaker than it looked.

Fixed three ways:

* use `rotate_to_yaw_async`, the API built for it — verified to land at 88.3 deg
  against 90.0 asked, on every flight since;
* **print the achieved heading, and warn loudly** if it is more than 10 deg out.
  This failed silently for an unknown number of runs; it will not again;
* take the heading from a **scene constant** (the direction of the road) instead
  of `car.pos`. The old code aimed the initial attitude using target ground
  truth, which sits badly with this demo's standing claim that the only steering
  input is where the detector puts the box.

## 4. The search fix, measured honestly

Six flights, arms interleaved, heading confound removed.
`--search-legacy-spin` restores the old behaviour so the new one has to earn its
place.

| | hit rate | sep mean | within 30 m | max nose excursion | episode length |
|---|---|---|---|---|---|
| bounded sweep (new) | 0.755 | 15.3 m | 1.000 | **23 deg** | 6.7 s |
| constant spin (old) | 0.768 | 15.7 m | 1.000 | 44 deg | 2.1 s |

**On tracking quality this is a null result.** 0.755 against 0.768 is inside the
per-arm spread (0.736-0.768 and 0.749-0.798), and see 4b: the experiment cannot
resolve differences this small at all. Reported as a null rather than dressed up,
because the sweep was my idea and it did not pay.

What it does do is bound the nose to 23 degrees instead of 44, which is the
symptom that prompted the report, and it bounds it *by construction* — the sweep
integrates to zero net rotation, asserted in `tests/test_range_and_lock.py`. It
costs longer time in search (6.7 s against 2.1 s), which is a real cost.

And in none of the six flights did either arm produce anything like the 292
degree spin. **The catastrophic case was the heading bug, not the search
strategy.**

Guardrail across all six: 0 interventions, NFZ 0.0 s, altitude 0.0 s.

## 4b. The forward creep: also unproven

The A/B above changed only the sweep — the creep was on in BOTH arms, so it was
never isolated. Three more flights with `--search-creep 0`:

| arm (n=3 each) | hit rate | min-max | mean sep | within 30 m |
|---|---|---|---|---|
| sweep + creep (shipped) | 0.755 | 0.736-0.768 | 15.3 m | 1.000 |
| legacy spin + creep | 0.768 | 0.749-0.798 | 15.7 m | 1.000 |
| sweep, creep OFF | 0.738 | 0.724-0.753 | 16.4 m | 0.980 |

Pooled within-arm SD is **0.019**; the largest gap between any two arms is
**0.030**. And with n=3 against n=3 the smallest two-sided Mann-Whitney p
obtainable is **0.10** — no comparison in this experiment can reach p<0.05. It is
underpowered by construction. Resolving a 0.013 difference would need roughly 17
flights per arm.

So the honest statement is not "creep helps a little". It is: **nine flights
cannot separate these three configurations.**

There is one asymmetry worth recording. The creep exists to stop the target
pulling away during a long loss — the failure that cost 9 m of separation
(26.3 -> 35.5 m) in the broken-heading run. In all nine of these flights the
aircraft was closing on the car anyway and the longest loss was 8.6 s, so **the
conditions the creep addresses never occurred.** It is therefore unproven rather
than disproven, and is kept at 0.5 with `--search-creep 0` available.

## 4c. And the residual dropout was none of the above

Everything above concerns what the aircraft DOES once the lock is lost. It never
explained why the lock was lost on a car in plain view, and the answer turned out
to be a single hard-coded comparison in the colour gate: HSV saturation is chroma
divided by brightness, so the `s > 90` floor rejected the target for being
sunlit. Fixing it took the detector hit rate to 1.000 and removed the lost-lock
episodes entirely. Full account in
`FINDING-colour-gate-and-sunlight-aug15.md`.

That also **retires the search-behaviour work above as mostly moot**: with the
gate fixed there are no lost-lock episodes for the sweep to handle. The bounded
sweep is kept because it bounds the nose by construction, but its practical
effect on these flights is now zero, which is a fairer verdict than the null it
already earned.

## 5. The structural weakness underneath all of it

Measured over 3443 ticks of the six A/B flights, the angle between the camera's
line of sight and the car's own heading:

| | mean | median |
|---|---|---|
| while tracking | 2.9 deg | 2.4 deg |
| while lost | 1.3 deg | 0.8 deg |

Zero means looking at the car's back. **A follower sits at the detector's worst
aspect for the whole flight.** From `experiments/probe_glb_yaw.py`, on the same
car and camera:

| aspect | rear-on | front-on | broadside | broadside |
|---|---|---|---|---|
| OWL-ViT score | 0.071 | 0.066 | **0.146** | 0.126 |

Detection is worth roughly **twice as much side-on**. The follow controller
drives straight at the target, so it holds the one geometry where the detector is
weakest, and the transient dropouts happen where the aspect is most exactly
rear-on (1.3 deg against 2.9 deg).

That points at the most promising remaining improvement, which is not a detector
change at all: **trail at a lateral offset** rather than directly astern, so the
camera sees a flank. It is a change to the follow geometry and has not been
tested, so it is recorded here as a hypothesis with a measurement behind it, not
as a result.

## Reproducing

```powershell
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\analyse_lost_lock.py demo\out\demo_traffic
C:\Users\natha\.conda\envs\vla-real\python.exe experiments\check_target_occlusion.py demo\out\demo_traffic
```

## 6. The blank simulator window, which I twice said was fixed and was not

Separate fault, same day, worth recording because the wrong diagnosis was
asserted to the user twice.

**Symptom:** "hanya keluar window berukuran kecil dan tidak menampilkan
simulasi" - the simulator opens a small window showing near-white, no scene.

**What I claimed first:** two depth captures (`DEPTH_PLANAR` and
`DEPTH_PERSPECTIVE`) were both streaming, costing two extra render targets every
0.1 s. I removed the planar one, launched the simulator, screenshotted it, saw
the city, and reported it fixed.

**Why that was wrong:** the screenshot was of an IDLE simulator. The fault only
appears once a client loads the scene. Running the demo and screenshotting
mid-flight reproduced it immediately - a 400x225 window, blank white.

**The actual cause.** `World.switch_streaming_view()` is documented as switching
"the main view to the next available camera with streaming-enabled=true", so the
game window is BOUND to such a camera. This robot declares exactly two:

    FrontCamera  image-type 2 (DEPTH_PERSPECTIVE)  400x225  streaming: true
    Chase        image-type 0 (scene)              640x360  streaming: true

The first is the depth stream. The window therefore resizes itself to **400x225**
and displays a **depth image**, which is near-white because everything in the
scene is far away. Both halves of the symptom, exactly.

**The fix, and why the obvious one is wrong.** Disabling the depth stream would
"fix" the window and silently break the range signal: with
`streaming-enabled: false` the depth arrives as all zeros and the controller
falls back to the width servo without saying so (already recorded as load-bearing
in the robot config). So the view is ADVANCED instead - one
`switch_streaming_view()` after the scene loads moves the window onto the Chase
camera, which is the third-person view of the aircraft and the thing worth
watching. Exposed as `--view-switch`, default 1.

**The lesson, which cost two wrong claims:** a fault that only manifests under
load cannot be cleared by observing the system at rest. The verification has to
reproduce the user's conditions, not merely the user's component.

