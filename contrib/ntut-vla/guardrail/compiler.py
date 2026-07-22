"""
Constraint Compiler (mini) — the "before the VLA" half of the Guardrail.

Meeting architecture slot:

    User Command -> [Constraint Compiler] -> YAML Prompt -> VLA -> ...

Takes a natural-language command, resolves it into a structured Mission, and
renders a YAML prompt that bundles the mission WITH a summary of the active
policy constraints (the grant calls this the Constraint Summary Pack — CSP).

v0 parsing is deliberately simple: a named-place registry + coordinate regex.
A real deployment would use map data / an LLM here; the *interface* (text in,
validated Mission + prompt out) is what matters, and that stays stable.
"""
from __future__ import annotations

import re

import yaml
from pydantic import BaseModel

from .models import AltitudeEnvelope, KinematicEnvelope, Policy, PolygonFence

# Named places the operator may refer to (stand-in for a map service).
PLACES = {
    "northeast pad": (30.0, 30.0),
    "north pad": (35.0, 0.0),
    "east pad": (0.0, 35.0),
    "home": (0.0, 0.0),
}


class Mission(BaseModel):
    task_text: str                 # the operator's original words
    target_x: float
    target_y: float
    cruise_alt_m: float
    speed_pref_mps: float          # what the operator ASKED for (may be illegal!)


class ConstraintCompiler:
    def __init__(self, policy: Policy):
        self.policy = policy

    # ---------------- command -> Mission ---------------- #

    def parse_command(self, text: str, default_alt: float = 15.0,
                      default_speed: float = 6.0) -> Mission:
        """Resolve ambiguous human text into an exact, structured mission."""
        low = text.lower()

        target = None
        for name, xy in PLACES.items():
            if name in low:
                target = xy
                break
        m = re.search(r"\(?\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)?", low)
        if target is None and m:
            target = (float(m.group(1)), float(m.group(2)))
        if target is None:
            raise ValueError(f"cannot resolve a target from: {text!r} "
                             f"(known places: {list(PLACES)})")

        # Parse speed FIRST and strip it, so "6 m/s" can never be mistaken
        # for an altitude (bit us once: "... 6 m/s altitude 20" parsed alt=6).
        speed = default_speed
        m = re.search(r"(\d+(?:\.\d+)?)\s*m/s", low)
        if m:
            speed = float(m.group(1))
            low = low.replace(m.group(0), " ")

        alt = default_alt
        m = re.search(r"(?:alt|altitude|height|tinggi)\D{0,3}(\d+(?:\.\d+)?)|"
                      r"(\d+(?:\.\d+)?)\s*m\s+(?:alt|altitude|height)", low)
        if m:
            alt = float(m.group(1) or m.group(2))

        return Mission(task_text=text, target_x=target[0], target_y=target[1],
                       cruise_alt_m=alt, speed_pref_mps=speed)

    # ---------------- Mission + policy -> YAML prompt ---------------- #

    def build_prompt(self, mission: Mission) -> str:
        """Render the structured YAML prompt the VLA receives (CSP-lite).
        Templated, never free-form — same rule as the grant's NL adapter."""
        nfz = [
            {"id": f.id, "vertices": [{"x": v.x, "y": v.y} for v in f.vertices]}
            for f in self.policy.by_type(PolygonFence)
        ]
        alts = self.policy.by_type(AltitudeEnvelope)
        kins = self.policy.by_type(KinematicEnvelope)

        doc = {
            "mission": {
                "task": mission.task_text,
                "target": {"x": mission.target_x, "y": mission.target_y},
                "cruise_alt_m": mission.cruise_alt_m,
            },
            "constraints": {
                "policy_id": self.policy.policy_id,
                "policy_hash": self.policy.policy_hash,
                "no_fly_zones": nfz,
                "altitude_band_m": (
                    [alts[0].alt_min_m, alts[0].alt_max_m] if alts else None),
                "speed_max_mps": kins[0].speed_max_mps if kins else None,
            },
        }
        nl = []
        for f in self.policy.by_type(PolygonFence):
            nl.append(f"Never enter zone '{f.id}'.")
        if alts:
            nl.append(f"Stay between {alts[0].alt_min_m} m and {alts[0].alt_max_m} m altitude.")
        if kins:
            nl.append(f"Keep speed at or below {kins[0].speed_max_mps} m/s.")
        doc["natural_language_prompt"] = " ".join(nl)

        return yaml.safe_dump(doc, sort_keys=False, allow_unicode=True)
