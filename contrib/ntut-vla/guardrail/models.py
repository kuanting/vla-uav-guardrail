"""
Policy DSL (mini) — Pydantic models + YAML loader.

Scaled-down version of the grant's constraint taxonomy. Three constraint classes
for v0 (the grant has nine):

    polygon_fence       keep-OUT area (no-fly zone), 2-D polygon + altitude band
    altitude_envelope   min/max height above ground
    kinematic_envelope  speed / climb-rate / yaw-rate caps

Coordinates: local meters (x = North, y = East), altitude = meters above ground,
up positive. The real grant DSL uses WGS84 lat/lon; local meters keeps the
prototype simple and matches AirSim's local frame. Swap later = loader change only.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Annotated, Literal, Union

import yaml
from pydantic import BaseModel, Field, model_validator


# --------------------------------------------------------------------------- #
# Runtime data types (not part of the policy file)
# --------------------------------------------------------------------------- #

class Action4D(BaseModel):
    """The VLA's output — the ONLY thing the Shield accepts. Grant contract."""
    vx: float = 0.0        # m/s, +North
    vy: float = 0.0        # m/s, +East
    vz_up: float = 0.0     # m/s, +up  (NED conversion happens at the sim adapter)
    yaw_rate: float = 0.0  # deg/s, +clockwise seen from above


class State(BaseModel):
    """Minimal vehicle state the Shield needs at each tick."""
    x: float               # m North of start
    y: float               # m East of start
    up: float              # m above ground
    yaw_deg: float = 0.0


# --------------------------------------------------------------------------- #
# Constraint taxonomy (the policy file surface)
# --------------------------------------------------------------------------- #

class ConstraintBase(BaseModel):
    id: str
    constraint_type: Literal["hard", "soft"] = "hard"
    priority: Literal["P0", "P1", "P2"] = "P0"
    violation_action: Literal["repair", "brake"] = "repair"


class XY(BaseModel):
    x: float
    y: float


class PolygonFence(ConstraintBase):
    """Keep-OUT no-fly zone. Violated when a (predicted) position is inside."""
    type: Literal["polygon_fence"]
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0            # extra safety ring around the polygon

    @model_validator(mode="after")
    def _sane_band(self) -> "PolygonFence":
        if self.altitude_ceiling_m <= self.altitude_floor_m:
            raise ValueError(f"{self.id}: ceiling must be > floor")
        return self


class AltitudeEnvelope(ConstraintBase):
    """Stay between alt_min and alt_max (meters above ground)."""
    type: Literal["altitude_envelope"]
    alt_min_m: float
    alt_max_m: float

    @model_validator(mode="after")
    def _sane(self) -> "AltitudeEnvelope":
        if self.alt_max_m <= self.alt_min_m:
            raise ValueError(f"{self.id}: alt_max must be > alt_min")
        return self


class KinematicEnvelope(ConstraintBase):
    """Caps on how fast the vehicle may move/climb/turn."""
    type: Literal["kinematic_envelope"]
    speed_max_mps: float = Field(gt=0)          # horizontal speed cap
    climb_rate_max_mps: float = Field(gt=0)     # |vz| cap
    yaw_rate_max_dps: float = Field(gt=0)


Constraint = Annotated[
    Union[PolygonFence, AltitudeEnvelope, KinematicEnvelope],
    Field(discriminator="type"),
]


class Policy(BaseModel):
    """A validated policy bundle (the 'IR' in miniature)."""
    policy_id: str
    version: str = "0.1.0"
    generation: int = 0            # bumps on every mid-flight hot-apply (grant rule)
    constraints: list[Constraint]

    @property
    def policy_hash(self) -> str:
        """SHA-256 over the canonicalised policy — the reproducibility anchor.
        Every audit record carries this, mirroring the grant's policy_hash rule."""
        canon = json.dumps(self.model_dump(), sort_keys=True, separators=(",", ":"))
        return "sha256:" + hashlib.sha256(canon.encode()).hexdigest()[:16]

    def by_type(self, cls) -> list:
        return [c for c in self.constraints if isinstance(c, cls)]


def load_policy(path: str | Path) -> Policy:
    """YAML file -> validated Policy. Any schema error raises here, loudly,
    BEFORE flight — never mid-air."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return Policy.model_validate(raw)
