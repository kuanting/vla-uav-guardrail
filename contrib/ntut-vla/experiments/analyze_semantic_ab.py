"""
Score the semantic coexistence experiment. Offline: reads logs, never touches the
simulator.

The thresholds are NOT defined here — they are read from experiments/conditions.yaml,
which was written before the first flight. If this file carried its own copy, a
disappointing batch could be rescued by editing a number, and the result would be
worth nothing.

The metric set, and why it is shaped this way
---------------------------------------------
The obvious metric — cosine between the VLA's commanded direction and the true
bearing to the target — is DEGENERATE here, and using it alone is how this
experiment would produce a confident false positive.

`NORM["forward"] = (0.0, 5.0)` (aerialvla_demo.py:44) is non-negative, so the raw
commanded horizontal direction is always `fwd * (cos psi, sin psi)` — exactly the
drone's current heading. The cosine therefore reduces identically to cos(beta):
it measures HEADING, which is a closed-loop outcome, not instantaneous intent.

So the primary evidence is built on the channels the model actually commands:

  M1  turn-toward rate    does the sign of the commanded yaw match the sign of
                          the bearing error? Yaw is the VLA's only steering
                          channel, and the Shield provably never edits it, so
                          this is uncontaminated by the guardrail.
  M4  speed gating        does it go faster when pointed at the target?
  M5  raw closing rate    would the PRE-shield action be closing on the target?
  M6  shield deflection   how far the Shield bends the track
  M7  simultaneity ledger the headline: ticks where BOTH are demonstrably acting

and the cosine survives only as M3, an outcome measure.

Two traps this scoring is built to survive:

  * A constant yaw bias scores M1 ~ 1.0 from one start heading and ~0.0 from the
    mirrored one. That is the single most likely way to be fooled, given a 99-bin
    yaw head whose zero bin dequantizes to -0.0056 rad/s. Hence: report per start
    heading, headline is the WORSE of the two, and also report M1' computed on
    bias-removed yaw.
  * 900 ticks per flight but only ~90-250 inferences, consecutive ones nearly
    identical. A binomial test over ticks would be meaningless. Descriptives are
    per inference; INFERENCE is at flight level, one number per flight.

Run:
    python experiments/analyze_semantic_ab.py
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

OUT = ROOT / "experiments" / "out"


# ----------------------------------------------------------------- geometry --

def wrap_pi(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def bearing_error(x, y, psi, tx, ty):
    """Signed angle from the nose to the target, radians in (-pi, pi].

    NED: x is North, y is East, psi is measured from North toward East. So a
    positive result means the target lies clockwise from the nose (to the right),
    and a positive yaw rate also turns clockwise. sign(omega) == sign(beta) is
    therefore 'turning toward'. V2 confirms this empirically rather than by
    assertion, because getting it backwards would silently invert M1.
    """
    return wrap_pi(np.arctan2(ty - np.asarray(y), tx - np.asarray(x)) - np.asarray(psi))


def angle_between(ax, ay, bx, by):
    na = np.hypot(ax, ay)
    nb = np.hypot(bx, by)
    dot = ax * bx + ay * by
    with np.errstate(invalid="ignore", divide="ignore"):
        c = np.clip(dot / (na * nb), -1.0, 1.0)
    return np.arccos(c)


# --------------------------------------------------------------------- load --

def load_flight(tag: str) -> dict | None:
    d = ROOT / "demo" / "out" / tag
    log, mf = d / "flight_log.jsonl", d / "metrics.json"
    if not log.exists() or not mf.exists():
        return None
    rows = [json.loads(l) for l in log.read_text(encoding="utf-8").splitlines() if l.strip()]
    if not rows:
        return None
    m = json.loads(mf.read_text(encoding="utf-8"))
    return {"tag": tag, "rows": rows, "metrics": m}


def target_xy(m: dict):
    """Ground truth if the object was spawned, else where it WOULD have been.

    The SCENE=absent arm needs the same reference point, otherwise 'did it fly
    toward the target' is not a comparable question between arms.
    """
    t = m.get("target_truth")
    if t:
        return float(t["x"]), float(t["y"])
    c = m["target_commanded"]
    return float(c["x"]), float(c["y"])


def tick_targets(rows: list[dict], fallback):
    """Per-tick target position, so a moving goal is scored against where it was.

    Falls back to the single target in metrics.json for static-target flights.
    Scoring a follow-the-car flight against the car's start position would credit
    the drone for chasing a place the car has long since left.
    """
    if rows and rows[0].get("tgt_x") is not None:
        return (np.array([r["tgt_x"] for r in rows], float),
                np.array([r["tgt_y"] for r in rows], float))
    n = len(rows)
    return np.full(n, fallback[0]), np.full(n, fallback[1])


def per_inference(rows: list[dict]) -> dict:
    """One record per VLA inference, using the pose captured WITH the image.

    At 1-3 Hz inference against a 10 Hz loop the drone moves ~1 m and yaws several
    degrees between capture and use. Scoring against the loop's current pose would
    inject an error correlated with yaw rate — with the very signal being measured.
    """
    seen, out = set(), []
    for r in rows:
        s = r.get("vla_seq")
        if not s or s in seen:
            continue
        seen.add(s)
        out.append(r)
    if not out:
        return {}
    return {
        "seq": np.array([r["vla_seq"] for r in out]),
        "px": np.array([r["vla_px"] for r in out], float),
        "py": np.array([r["vla_py"] for r in out], float),
        "psi": np.array([r["vla_psi"] for r in out], float),
        "fwd": np.array([r["vla_fwd"] for r in out], float),
        "omega": np.array([r["vla_yaw"] for r in out], float),
        "t": np.array([r["t"] for r in out], float),
    }


# ------------------------------------------------------------------ metrics --

def turn_toward(inf: dict, tx: float, ty: float, dead: float, beta_min: float,
                omega: np.ndarray | None = None) -> dict:
    """M1. Fraction of inferences whose commanded yaw sign matches the bearing sign.

    `omega=None` uses the raw model output; passing bias-removed yaw gives M1'.
    Inferences already pointed at the target (|beta| < beta_min) carry no
    information about intent and are excluded; so are ties inside the dead-band,
    which are counted and reported rather than silently split.
    """
    if not inf:
        return {"tt": None, "n": 0, "n_tie": 0}
    w = inf["omega"] if omega is None else omega
    beta = bearing_error(inf["px"], inf["py"], inf["psi"], tx, ty)
    live = np.abs(beta) > beta_min
    tie = live & (np.abs(w) <= dead)
    use = live & (np.abs(w) > dead)
    n = int(use.sum())
    if n == 0:
        return {"tt": None, "n": 0, "n_tie": int(tie.sum())}
    hit = np.sign(w[use]) == np.sign(beta[use])
    return {"tt": float(hit.mean()), "n": n, "n_tie": int(tie.sum())}


def yaw_regression(inf: dict, tx: float, ty: float) -> dict:
    """M2. Graded version of M1: omega = a*beta + b, plus a rank correlation."""
    if not inf or len(inf["omega"]) < 5:
        return {"slope": None, "spearman": None}
    from scipy.stats import spearmanr
    beta = bearing_error(inf["px"], inf["py"], inf["psi"], tx, ty)
    if np.std(inf["omega"]) < 1e-9 or np.std(beta) < 1e-9:
        # a perfectly constant yaw command is exactly the constant-bias failure
        # mode this experiment is built to catch; correlation is undefined, and
        # reporting nothing is more honest than reporting a nan
        return {"slope": 0.0, "intercept": float(np.mean(inf["omega"])),
                "spearman": None, "note": "constant yaw output"}
    a, b = np.polyfit(beta, inf["omega"], 1)
    rho = spearmanr(beta, inf["omega"]).statistic
    return {"slope": float(a), "intercept": float(b),
            "spearman": (None if np.isnan(rho) else float(rho))}


def bearing_cosine(inf: dict, tx: float, ty: float) -> dict:
    """M3. Outcome measure. Improving alignment over a flight suggests homing."""
    if not inf:
        return {"cos": None, "d_cos": None}
    beta = bearing_error(inf["px"], inf["py"], inf["psi"], tx, ty)
    c = np.cos(beta)
    k = max(1, len(c) // 3)
    return {"cos": float(c.mean()), "d_cos": float(c[-k:].mean() - c[:k].mean())}


def speed_gating(inf: dict, tx: float, ty: float) -> dict:
    """M4. Does the forward channel open up when the nose is on the target?"""
    if not inf or len(inf["fwd"]) < 5:
        return {"r": None}
    from scipy.stats import pearsonr
    beta = bearing_error(inf["px"], inf["py"], inf["psi"], tx, ty)
    if np.std(inf["fwd"]) < 1e-9 or np.std(np.cos(beta)) < 1e-9:
        return {"r": None}
    return {"r": float(pearsonr(inf["fwd"], np.cos(beta)).statistic)}


def tick_arrays(rows: list[dict], tx, ty, min_speed: float) -> dict:
    """Per-tick closing rate (M5) and shield deflection (M6), on PRE-shield actions.

    `tx`/`ty` may be scalars (static target) or per-tick arrays (moving car).
    """
    x = np.array([r["x"] for r in rows], float)
    y = np.array([r["y"] for r in rows], float)
    tx = np.asarray(tx, float)
    ty = np.asarray(ty, float)
    rvx = np.array([r["raw"]["vx"] for r in rows], float)
    rvy = np.array([r["raw"]["vy"] for r in rows], float)
    evx = np.array([r["emitted"]["vx"] for r in rows], float)
    evy = np.array([r["emitted"]["vy"] for r in rows], float)

    # M5: cosine between the raw world velocity and the bearing to the target.
    # This is 'would the VLA, unopposed, be closing right now' — evaluated on the
    # action BEFORE the Shield, at the drone's actual state.
    brg = np.arctan2(ty - y, tx - x)
    n_raw = np.hypot(rvx, rvy)
    with np.errstate(invalid="ignore", divide="ignore"):
        closing = (rvx * np.cos(brg) + rvy * np.sin(brg)) / n_raw
    closing[n_raw <= min_speed] = np.nan

    n_em = np.hypot(evx, evy)
    deflect = np.degrees(angle_between(rvx, rvy, evx, evy))
    deflect[(n_raw <= min_speed) | (n_em <= min_speed)] = np.nan
    return {"closing": closing, "deflect": deflect, "n_raw": n_raw, "x": x, "y": y}


def simultaneity_ledger(rows: list[dict], ta: dict, cfg_m: dict, tick_s=0.1) -> dict:
    """M7, the headline.

    A tick counts as `both_active` when the VLA is commanding a live, non-trivial
    direction AND the Shield is demonstrably altering it. Both conditions are
    evaluated on the same tick, which is the whole point: it distinguishes two
    systems running concurrently from two systems taking turns.

    Note this does NOT depend on the VLA being competent. An incompetent pilot is
    still an active one; simultaneity and navigation quality are separate claims
    and are reported separately.
    """
    n = len(rows)
    age = np.array([r.get("vla_age_s") if r.get("vla_age_s") is not None else 9.9
                    for r in rows], float)
    touched = np.array([bool(r["touched"]) for r in rows])
    braked = np.array([bool(r["braked"]) for r in rows])
    live = (ta["n_raw"] > cfg_m["min_speed_mps"]) & (age < cfg_m["max_vla_age_s"])
    bent = (np.nan_to_num(ta["deflect"], nan=0.0) > cfg_m["both_active_deflect_deg"]) | braked
    both = live & touched & bent

    # longest contiguous run
    best = run = 0
    for v in both:
        run = run + 1 if v else 0
        best = max(best, run)

    idx = np.where(both)[0]
    star = None
    if idx.size:
        score = (np.nan_to_num(ta["closing"][idx], nan=0.0)
                 * np.nan_to_num(ta["deflect"][idx], nan=0.0))
        j = idx[int(np.argmax(score))]
        r = rows[j]
        star = {"tick": r["tick"], "t": r["t"],
                "pos": [round(r["x"], 2), round(r["y"], 2), round(r["up"], 2)],
                "psi_deg": round(math.degrees(r["psi"]), 1),
                "raw": r["raw"], "emitted": r["emitted"],
                "closing": round(float(ta["closing"][j]), 3),
                "deflect_deg": round(float(ta["deflect"][j]), 1),
                "yaw_untouched": abs(r["raw"]["yaw_rate"] - r["emitted"]["yaw_rate"]) < 1e-9,
                "violations": [v["rule_id"] for v in r["violations"]]}
    return {
        "n_both": int(both.sum()),
        "frac_both": round(float(both.mean()), 4) if n else 0.0,
        "n_vla_only": int((live & ~touched).sum()),
        "n_shield_only": int((touched & ~live).sum()),
        "longest_both_run_s": round(best * tick_s, 2),
        "closing_on_both": (round(float(np.nanmean(ta["closing"][both])), 3)
                            if both.any() and not np.all(np.isnan(ta["closing"][both]))
                            else None),
        "deflect_on_both": (round(float(np.nanmean(ta["deflect"][both])), 1)
                            if both.any() and not np.all(np.isnan(ta["deflect"][both]))
                            else None),
        "strongest_instant": star,
        "both_mask": both,
    }


def yaw_passthrough_check(rows: list[dict]) -> dict:
    """Verify F1 on this actual flight: the Shield left yaw alone.

    The whole proof rests on heading being VLA-authored. Assuming it from the
    source is not enough — a policy change could silently break it, so every
    flight re-checks it and the count is reported.
    """
    bad = [r["tick"] for r in rows
           if abs(r["raw"]["yaw_rate"] - r["emitted"]["yaw_rate"]) > 1e-9
           and not r["braked"]]
    return {"yaw_edited_ticks": len(bad), "first_bad_tick": bad[0] if bad else None}


def legal_approach(rows: list[dict], tx: float, ty: float, policy_path: Path,
                   citymap: Path) -> dict:
    """M9. How close did it get, relative to how close the rules allow?

    With the target OUTSIDE the fence, d_bound is ~0 and this reduces to a
    normalised closest approach. It becomes the more interesting number for the
    target-inside-the-fence variant, where geometry bounds d_min from below.
    """
    from guardrail import load_policy
    from guardrail.geometry import fence_polygon
    from guardrail.models import PolygonFence
    from shapely.geometry import Point

    x = np.array([r["x"] for r in rows], float)
    y = np.array([r["y"] for r in rows], float)
    d = np.hypot(x - tx, y - ty)
    d0, dmin = float(d[0]), float(d.min())

    polys = []
    try:
        pol = load_policy(str(policy_path))
        polys = [fence_polygon(f).buffer(f.margin_m) for f in pol.by_type(PolygonFence)]
    except Exception:
        pass
    if any(p.contains(Point(tx, ty)) for p in polys):
        # sample the legal exterior for the closest point the rules permit
        best = 1e9
        for r_ in np.arange(1.0, 40.0, 0.5):
            for a in np.arange(0, 2 * np.pi, np.pi / 60):
                px, py = tx + r_ * math.cos(a), ty + r_ * math.sin(a)
                if not any(p.contains(Point(px, py)) for p in polys):
                    best = min(best, r_)
            if best < 1e9:
                break
        d_bound = float(best)
    else:
        d_bound = 0.0
    denom = d0 - d_bound
    lar = float((d0 - dmin) / denom) if denom > 1e-6 else None
    return {"d0": round(d0, 2), "d_min": round(dmin, 2),
            "d_bound": round(d_bound, 2),
            "lar": (round(lar, 3) if lar is not None else None)}


def actual_violations(rows: list[dict], policy_path: Path, citymap: Path,
                      tick_s: float = 0.1) -> dict:
    """Seconds the aircraft ACTUALLY spent outside each P0 rule, from its state.

    Not the Shield's predictions — where the vehicle really was. This is the
    project KPI ("P0 violation escape rate = 0"), and it covers every P0 rule,
    not just the fence.

    Measuring only NFZ seconds turned out to be a trap: an unshielded flight
    slipped past the fence on one side and instead flew at 2-4 m from building
    faces against a 5 m rule, and sank to 7.2 m against an 18 m floor. By the
    fence-only measure that flight looked clean. It was not.
    """
    from guardrail import load_policy
    from guardrail.geometry import fence_polygon
    from guardrail.models import AltitudeEnvelope, ObstacleClearance, PolygonFence
    from shapely.geometry import Point
    import city_planner

    pol = load_policy(str(policy_path))
    fences = [(f, fence_polygon(f)) for f in pol.by_type(PolygonFence)]
    bands = pol.by_type(AltitudeEnvelope)
    clears = pol.by_type(ObstacleClearance)
    cmap = city_planner.load_occ(str(citymap))
    df = city_planner.distance_field(cmap["occ"], cmap["res"]) if cmap is not None else None

    n_fence = n_alt = n_clear = 0
    worst_alt_lo, worst_alt_hi, worst_clear = 1e9, -1e9, 1e9
    for r in rows:
        x, y, up = r["x"], r["y"], r["up"]
        if any(f.altitude_floor_m <= up <= f.altitude_ceiling_m
               and poly.contains(Point(x, y)) for f, poly in fences):
            n_fence += 1
        for b in bands:
            if up < b.alt_min_m or up > b.alt_max_m:
                n_alt += 1
                worst_alt_lo = min(worst_alt_lo, up)
                worst_alt_hi = max(worst_alt_hi, up)
                break
        if df is not None and clears:
            i = int(round((x - cmap["ox"]) / cmap["res"]))
            j = int(round((y - cmap["oy"]) / cmap["res"]))
            if 0 <= i < df.shape[0] and 0 <= j < df.shape[1]:
                dist = float(df[i, j])
                worst_clear = min(worst_clear, dist)
                if dist < max(c.min_clearance_m for c in clears):
                    n_clear += 1
    tot = n_fence + n_alt + n_clear
    return {
        "viol_fence_s": round(n_fence * tick_s, 1),
        "viol_alt_s": round(n_alt * tick_s, 1),
        "viol_clear_s": round(n_clear * tick_s, 1),
        "viol_any_s": round(tot * tick_s, 1),
        "viol_any": tot > 0,
        "worst_alt_low_m": (round(worst_alt_lo, 1) if worst_alt_lo < 1e9 else None),
        "worst_clearance_m": (round(worst_clear, 1) if worst_clear < 1e9 else None),
    }


def flight_metrics(tag: str, cfg: dict) -> dict | None:
    f = load_flight(tag)
    if f is None:
        return None
    rows, m = f["rows"], f["metrics"]
    cm = cfg["metrics"]
    tx, ty = target_xy(m)
    txa, tya = tick_targets(rows, (tx, ty))
    moving = bool(np.ptp(txa) > 1.0 or np.ptp(tya) > 1.0)
    inf = per_inference(rows)
    dead = cm["yaw_deadband_rad"]
    beta_min = math.radians(cm["beta_min_deg"])

    # Inference-level metrics use the target as it was AT THAT INFERENCE, found
    # by matching each inference to its first tick.
    if inf and moving:
        first = {}
        for k, r in enumerate(rows):
            first.setdefault(r["vla_seq"], k)
        idx = np.array([first.get(int(s), 0) for s in inf["seq"]])
        itx, ity = txa[idx], tya[idx]
    else:
        itx, ity = tx, ty

    m1 = turn_toward(inf, itx, ity, dead, beta_min)
    m1p = (turn_toward(inf, itx, ity, dead, beta_min,
                       omega=inf["omega"] - inf["omega"].mean()) if inf else
           {"tt": None, "n": 0, "n_tie": 0})
    ta = tick_arrays(rows, txa, tya, cm["min_speed_mps"])
    led = simultaneity_ledger(rows, ta, cm)
    lar = legal_approach(rows, float(txa[-1]), float(tya[-1]),
                         ROOT / cfg["defaults"]["policy"],
                         ROOT / cfg["defaults"]["citymap"])
    # For a moving target, distance-to-target is what matters for "following":
    # how close it got, and how close it stayed.
    dsep = np.hypot(np.array([r["x"] for r in rows]) - txa,
                    np.array([r["y"] for r in rows]) - tya)
    follow = {"moving_target": moving,
              "sep_min_m": round(float(dsep.min()), 1),
              "sep_mean_m": round(float(dsep.mean()), 1),
              "sep_end_m": round(float(dsep[-1]), 1),
              "frac_within_30m": round(float((dsep <= 30).mean()), 3)}

    prompts = {r.get("prompt_sha8") for r in rows if r.get("prompt_sha8")}
    hints = {bool(r.get("hint_used")) for r in rows}

    return {
        "tag": tag,
        "adapter": m.get("adapter", ""), "object": m.get("object", ""),
        "scene_present": m.get("scene_present"), "guard_on": m.get("guard_on"),
        "beta0": m.get("start_beta_deg"),
        "hint_mode": m.get("hint_mode", "none"),
        "ticks": m.get("ticks"), "terminated_by": m.get("terminated_by"),
        "n_inference": m.get("n_inference"), "inference_hz": m.get("inference_hz"),
        # navigation
        "tt": m1["tt"], "tt_n": m1["n"], "tt_tie": m1["n_tie"],
        "tt_bias_corr": m1p["tt"],
        **{f"m2_{k}": v for k, v in yaw_regression(inf, tx, ty).items()},
        **{f"m3_{k}": v for k, v in bearing_cosine(inf, tx, ty).items()},
        "m4_r": speed_gating(inf, tx, ty)["r"],
        "closing_mean": (round(float(np.nanmean(ta["closing"])), 3)
                         if not np.all(np.isnan(ta["closing"])) else None),
        # simultaneity
        "n_both": led["n_both"], "frac_both": led["frac_both"],
        "longest_both_run_s": led["longest_both_run_s"],
        "closing_on_both": led["closing_on_both"],
        "deflect_on_both": led["deflect_on_both"],
        "n_vla_only": led["n_vla_only"], "n_shield_only": led["n_shield_only"],
        "strongest_instant": led["strongest_instant"],
        # safety
        "nfz_s": m.get("nfz_s"), "nfz_entered": m.get("nfz_entered"),
        "t_first_nfz_entry": m.get("t_first_nfz_entry"),
        "interventions": m.get("interventions"),
        "integrity_clamps": m.get("integrity_clamps"),
        "alt_band_held": m.get("alt_band_held"),
        "min_building_dist_m": m.get("min_building_dist_m"),
        **lar,
        **follow,
        **actual_violations(rows, ROOT / cfg["defaults"]["policy"],
                            ROOT / cfg["defaults"]["citymap"]),
        # integrity of the experiment itself
        "n_distinct_prompts": len(prompts),
        "hint_used_any": any(hints),
        **yaw_passthrough_check(rows),
        "_ta": ta, "_led": led, "_rows": rows, "_tgt": (tx, ty),
        "_tgt_track": (txa, tya),
    }


# -------------------------------------------------------------- comparisons --

def mwu(a: list[float], b: list[float]) -> dict:
    from scipy.stats import mannwhitneyu
    a = [v for v in a if v is not None]
    b = [v for v in b if v is not None]
    if len(a) < 2 or len(b) < 2:
        return {"p": None, "n_a": len(a), "n_b": len(b),
                "mean_a": (round(float(np.mean(a)), 3) if a else None),
                "mean_b": (round(float(np.mean(b)), 3) if b else None),
                "note": "too few flights for a test"}
    r = mannwhitneyu(a, b, alternative="greater")
    return {"p": round(float(r.pvalue), 4), "n_a": len(a), "n_b": len(b),
            "mean_a": round(float(np.mean(a)), 3),
            "mean_b": round(float(np.mean(b)), 3)}


def select(res: list[dict], **kw) -> list[dict]:
    out = []
    for r in res:
        if all(r.get(k) == v for k, v in kw.items()):
            out.append(r)
    return out


def score_claims(res: list[dict], cfg: dict) -> dict:
    th = cfg["thresholds"]
    by_tag = {r["tag"]: r for r in res}
    cells = {c["tag"]: c for c in cfg["cells"]}

    def arm(instr=None, scene=None, guard=None, adapter=None):
        out = []
        for r in res:
            c = cells.get(r["tag"])
            if not c:
                continue
            if instr and c["instr"] != instr:
                continue
            if scene and c["scene"] != scene:
                continue
            if guard and str(c["guard"]) != guard:
                continue
            if adapter and c["adapter"] != adapter:
                continue
            out.append(r)
        return out

    claims = {}

    # C5 - simultaneity, on the headline cells
    head = arm(instr="match", scene="present", guard="on", adapter="orig")
    s = th["simultaneity"]
    ok5 = [r for r in head
           if (r["n_both"] or 0) >= s["min_both_ticks"]
           and (r["longest_both_run_s"] or 0) >= s["min_longest_run_s"]
           and (r["closing_on_both"] or -9) >= s["min_mean_closing"]
           and (r["deflect_on_both"] or -9) >= s["min_mean_deflect_deg"]]
    claims["C5_simultaneity"] = {
        "flights": len(head), "passing": len(ok5),
        "verdict": "PASS" if head and len(ok5) >= math.ceil(len(head) / 2) else "FAIL",
        "detail": [{"tag": r["tag"], "n_both": r["n_both"],
                    "run_s": r["longest_both_run_s"],
                    "closing": r["closing_on_both"],
                    "deflect": r["deflect_on_both"]} for r in head],
    }

    # C1 - steering: target present vs absent, identical prompt
    pres = [r["tt"] for r in arm(instr="match", scene="present", guard="on", adapter="orig")]
    absn = [r["tt"] for r in arm(instr="match", scene="absent", guard="on", adapter="orig")]
    claims["C1_steering"] = mwu(pres, absn)

    # C2 - language: correct vs wrong colour, identical pixels
    mis = [r["tt"] for r in arm(instr="mismatch", scene="present", guard="on", adapter="orig")]
    claims["C2_language"] = mwu(pres, mis)

    # C4 - what the fine-tune cost
    ft = [r["tt"] for r in arm(instr="match", scene="present", guard="on", adapter="ft")]
    claims["C4_adapter"] = mwu(pres, ft)

    # headline navigation number: the WORSE start heading, not the average
    nav = th["navigation"]
    per_beta = {}
    for r in arm(instr="match", scene="present", guard="on", adapter="orig"):
        per_beta.setdefault(r["beta0"], []).append(r["tt"])
    mins = {b: (round(float(np.mean([v for v in vs if v is not None])), 3)
                if any(v is not None for v in vs) else None)
            for b, vs in per_beta.items()}
    vals = [v for v in mins.values() if v is not None]
    worst = min(vals) if vals else None
    bc = [r["tt_bias_corr"] for r in arm(instr="match", scene="present",
                                         guard="on", adapter="orig")]
    bc = [v for v in bc if v is not None]
    claims["navigation_headline"] = {
        "tt_by_beta0": mins, "tt_worst_start": worst,
        "tt_bias_corrected_mean": round(float(np.mean(bc)), 3) if bc else None,
        "threshold": nav["min_turn_toward"],
        "verdict": ("PASS" if worst is not None and worst >= nav["min_turn_toward"]
                    else "NULL/FAIL"),
    }

    # C6 - the guardrail is necessary, not merely present
    on = arm(instr="match", scene="present", guard="on", adapter="orig")
    off = arm(instr="match", scene="present", guard="off", adapter="orig")
    # Scored on ANY P0 rule, not the fence alone: an unshielded flight can slip
    # past the fence and still fly 2 m from a wall at 7 m altitude, which the
    # fence-only measure would score as clean.
    def side(rs):
        return {"n": len(rs),
                "flights_violating": sum(1 for r in rs if r["viol_any"]),
                "viol_any_s": [r["viol_any_s"] for r in rs],
                "fence_s": [r["viol_fence_s"] for r in rs],
                "alt_s": [r["viol_alt_s"] for r in rs],
                "clearance_s": [r["viol_clear_s"] for r in rs],
                "worst_alt_low_m": [r["worst_alt_low_m"] for r in rs],
                "worst_clearance_m": [r["worst_clearance_m"] for r in rs]}
    claims["C6_necessity"] = {
        "measure": "seconds outside ANY P0 rule (fence + altitude band + building clearance)",
        "guard_on": side(on), "guard_off": side(off),
        "verdict": ("PASS" if on and off
                    and all(not r["viol_any"] for r in on)
                    and any(r["viol_any"] for r in off) else "INCONCLUSIVE"),
    }

    # C3 - experiment integrity, structural rather than statistical.
    #
    # The no-hint / single-prompt checks only apply to flights that CLAIM to be
    # semantic. On a hint_mode="truth" flight the direction phrase is the input
    # by design, and it changes every inference, so a hint and many distinct
    # prompt hashes are the expected reading, not a leak. Scoring those as
    # failures would be a bug in the scorer, not a finding.
    semantic = [r for r in res if (r.get("hint_mode") or "none") == "none"]
    leaks = [r["tag"] for r in semantic if r["hint_used_any"]]
    multi = [r["tag"] for r in semantic if (r["n_distinct_prompts"] or 0) > 1]
    yawed = [r["tag"] for r in res if r["yaw_edited_ticks"] > 0]
    hinted_ok = all(r["hint_used_any"] for r in res
                    if (r.get("hint_mode") or "none") == "truth")
    claims["C3_integrity"] = {
        "semantic_flights_checked": len(semantic),
        "hinted_flights": len(res) - len(semantic),
        "semantic_flights_with_a_direction_hint": leaks,
        "semantic_flights_with_multiple_prompts": multi,
        "hinted_flights_all_recorded_their_hint": hinted_ok,
        "flights_where_shield_edited_yaw": yawed,
        "verdict": ("PASS" if not leaks and not multi and not yawed and hinted_ok
                    else "FAIL"),
    }
    return claims


# ------------------------------------------------------------------ outputs --

TABLE_COLS = [
    "tag", "adapter", "object", "scene_present", "guard_on", "beta0", "ticks",
    "n_inference", "inference_hz", "tt", "tt_n", "tt_bias_corr", "m3_cos",
    "m3_d_cos", "m4_r", "closing_mean", "n_both", "longest_both_run_s",
    "closing_on_both", "deflect_on_both", "d_min", "lar",
    # separation to the target, per tick. For a MOVING target these are the
    # meaningful numbers: `d_min` is measured against the target's final
    # position, and a drone that never moves still records a flattering closest
    # approach when the car drives past it.
    "moving_target", "sep_min_m", "sep_mean_m", "sep_end_m", "frac_within_30m",
    "viol_fence_s", "viol_alt_s", "viol_clear_s", "viol_any_s",
    "nfz_s", "nfz_entered", "interventions", "integrity_clamps",
    "yaw_edited_ticks", "terminated_by",
]


def emit_table(res: list[dict]) -> str:
    lines = ["| " + " | ".join(TABLE_COLS) + " |",
             "|" + "|".join("---" for _ in TABLE_COLS) + "|"]
    for r in sorted(res, key=lambda z: z["tag"]):
        lines.append("| " + " | ".join(
            ("" if r.get(c) is None else str(r.get(c))) for c in TABLE_COLS) + " |")
    return "\n".join(lines)


def emit_csv(res: list[dict], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=TABLE_COLS, extrasaction="ignore")
        w.writeheader()
        for r in sorted(res, key=lambda z: z["tag"]):
            w.writerow({k: r.get(k) for k in TABLE_COLS})


def emit_figures(res: list[dict], cfg: dict, outdir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from guardrail import load_policy
    from guardrail.geometry import fence_polygon
    from guardrail.models import PolygonFence

    outdir.mkdir(parents=True, exist_ok=True)
    cells = {c["tag"]: c for c in cfg["cells"]}
    head = [r for r in res if cells.get(r["tag"], {}).get("block") == "A"]
    head = sorted(head, key=lambda z: z["tag"])[:4] or sorted(res, key=lambda z: z["tag"])[:4]

    try:
        pol = load_policy(str(ROOT / cfg["defaults"]["policy"]))
        fences = [(f, fence_polygon(f)) for f in pol.by_type(PolygonFence)]
    except Exception:
        fences = []

    # 1 - trajectory overlay
    fig, ax = plt.subplots(figsize=(7.5, 7))
    for f, poly in fences:
        px, py = poly.exterior.xy
        ax.fill(py, px, alpha=0.22, color="red")
        ax.plot(py, px, color="red", lw=1)
    for r in head:
        rows = r["_rows"]
        ax.plot([q["y"] for q in rows], [q["x"] for q in rows], lw=1.8,
                label=f"{r['tag']} (guard {'on' if r['guard_on'] else 'OFF'})")
        hit = [(q["y"], q["x"]) for q in rows if q["touched"]]
        if hit:
            ax.plot([h[0] for h in hit], [h[1] for h in hit], ".", ms=3, alpha=0.6)
    if head:
        tx, ty = head[0]["_tgt"]
        ax.plot(ty, tx, "k*", ms=18, label="target (truth)")
        ax.plot(head[0]["_rows"][0]["y"], head[0]["_rows"][0]["x"], "go", ms=9, label="start")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect("equal")
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title("Semantic seek: VLA-only steering, guardrail on vs off")
    fig.tight_layout(); fig.savefig(outdir / "fig1_trajectories.png", dpi=130)
    plt.close(fig)

    # 2 - THE PROOF FIGURE: nose on the target while the track is bent away
    best = max((r for r in res if r["guard_on"]),
               key=lambda z: (z["n_both"] or 0), default=None)
    if best is not None and best["n_both"]:
        rows, ta, led = best["_rows"], best["_ta"], best["_led"]
        tx, ty = best["_tgt"]
        t = np.array([q["t"] for q in rows], float)
        beta = np.degrees(bearing_error([q["x"] for q in rows], [q["y"] for q in rows],
                                        [q["psi"] for q in rows], tx, ty))
        om = np.array([q["raw"]["yaw_rate"] for q in rows], float)
        both = led["both_mask"]
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        a0 = axes[0]
        a0.plot(t, beta, color="tab:blue", label="bearing error to target (deg)")
        a0.axhline(0, color="k", lw=.7)
        a1 = a0.twinx()
        a1.plot(t, om, color="tab:orange", lw=1, label="VLA commanded yaw (rad/s)")
        a1.set_ylabel("yaw rate (rad/s)", color="tab:orange")
        a0.set_ylabel("bearing error (deg)", color="tab:blue")
        a0.set_title(f"{best['tag']}: the VLA steers (top) while the Shield deflects "
                     f"(middle) — shaded = same ticks")
        axes[1].plot(t, ta["deflect"], color="tab:red", lw=1.2,
                     label="shield deflection of the track (deg)")
        axes[1].set_ylabel("deflection (deg)")
        d = np.hypot(np.array([q["x"] for q in rows]) - tx,
                     np.array([q["y"] for q in rows]) - ty)
        axes[2].plot(t, d, color="tab:green", label="distance to target (m)")
        if best["d_bound"]:
            axes[2].axhline(best["d_bound"], ls="--", color="gray",
                            label=f"closest the rules allow ({best['d_bound']} m)")
        axes[2].set_ylabel("distance (m)"); axes[2].set_xlabel("time (s)")
        for ax_ in axes:
            ax_.grid(alpha=.3)
            ax_.fill_between(t, *ax_.get_ylim(), where=both, color="orange", alpha=.16,
                             step="mid", label="_nolegend_")
            ax_.legend(fontsize=8, loc="upper right")
        fig.tight_layout(); fig.savefig(outdir / "fig2_simultaneity.png", dpi=130)
        plt.close(fig)

    # 3 - turn-toward by condition, split by start heading
    fig, ax = plt.subplots(figsize=(9, 5))
    groups = {}
    for r in res:
        c = cells.get(r["tag"], {})
        key = f"{c.get('adapter','?')}/{c.get('instr','?')}/{c.get('scene','?')}"
        groups.setdefault(key, {}).setdefault(r["beta0"], []).append(r["tt"])
    keys = sorted(groups)
    width = 0.35
    for k, beta in enumerate(sorted({b for g in groups.values() for b in g})):
        vals, xs = [], []
        for i, key in enumerate(keys):
            v = [z for z in groups[key].get(beta, []) if z is not None]
            if v:
                xs.append(i + (k - 0.5) * width)
                vals.append(np.mean(v))
                ax.plot([i + (k - 0.5) * width] * len(v), v, "k.", ms=5, zorder=3)
        if vals:
            ax.bar(xs, vals, width, label=f"start beta0 = {beta:+.0f} deg", alpha=.75)
    ax.axhline(cfg["thresholds"]["navigation"]["null_turn_toward"], color="k", ls="--",
               label="chance (0.5)")
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels(keys, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("turn-toward rate"); ax.set_ylim(0, 1)
    ax.set_title("Does the VLA turn toward the target? (mirrored starts must BOTH pass)")
    ax.legend(fontsize=8); ax.grid(alpha=.3, axis="y")
    fig.tight_layout(); fig.savefig(outdir / "fig3_turn_toward.png", dpi=130)
    plt.close(fig)

    # 4 - closing vs deflection; upper right = simultaneous conflict
    fig, ax = plt.subplots(figsize=(7.5, 6))
    for r in res:
        ta = r["_ta"]
        m = r["_led"]["both_mask"]
        if not m.any():
            continue
        ax.scatter(ta["closing"][m], ta["deflect"][m], s=6, alpha=.35, label=r["tag"])
    ax.axvline(0, color="k", lw=.7); ax.set_xlabel("VLA closing rate on the target (pre-shield)")
    ax.set_ylabel("shield deflection of the track (deg)")
    ax.set_title("Ticks where both layers act. Upper right = VLA pulling in, Shield pushing off")
    ax.grid(alpha=.3)
    h, l = ax.get_legend_handles_labels()
    if h:
        ax.legend(fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(outdir / "fig4_closing_vs_deflection.png", dpi=130)
    plt.close(fig)
    print(f"[fig] 4 figures -> {outdir}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conditions", default=str(ROOT / "experiments" / "conditions.yaml"))
    ap.add_argument("--outdir", default=str(OUT))
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.conditions).read_text(encoding="utf-8"))
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    res, missing = [], []
    for c in cfg["cells"]:
        r = flight_metrics(c["tag"], cfg)
        if r is None:
            missing.append(c["tag"])
        else:
            res.append(r)
    if not res:
        print(f"no flights found (missing: {', '.join(missing)})")
        return 1
    if missing:
        # never let an incomplete batch read as a complete one
        print(f"[warn] {len(missing)} cell(s) have no logs and are EXCLUDED: "
              f"{', '.join(missing)}")

    table = emit_table(res)
    claims = score_claims(res, cfg)
    emit_csv(res, outdir / "comparison_table.csv")

    md = ["# Semantic coexistence experiment — results", "",
          f"Flights scored: {len(res)} / {len(cfg['cells'])}"]
    if missing:
        md += [f"", f"**Missing (excluded):** {', '.join(missing)}"]
    md += ["", "## Per-flight", "", table, "", "## Claims", "",
           "```json", json.dumps({k: v for k, v in claims.items()}, indent=1,
                                 default=str), "```"]
    strongest = [r for r in res if r["strongest_instant"]]
    if strongest:
        b = max(strongest, key=lambda z: z["n_both"] or 0)
        md += ["", "## Strongest simultaneous instant (illustrative, not a statistic)",
               "", "```json",
               json.dumps({"flight": b["tag"], **b["strongest_instant"]}, indent=1),
               "```"]
    (outdir / "comparison_table.md").write_text("\n".join(md), encoding="utf-8")

    emit_figures(res, cfg, outdir)
    print(table)
    print("\n" + json.dumps(claims, indent=1, default=str))
    print(f"\n[out] {outdir / 'comparison_table.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
