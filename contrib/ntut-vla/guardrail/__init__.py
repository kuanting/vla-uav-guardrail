"""
Guardrail — mini Policy DSL + Safety Shield (Fase 3 prototype).

Pipeline it implements (grant terminology in parentheses):

    YAML policy file  --load-->  Policy (Policy DSL / IR)
    raw 4-D action    --Shield.filter-->  safe 4-D action  (Safety Shield)
                                    |
                                    +--> JSONL audit log

Frames: local NED-ish, but "up" is kept POSITIVE in this package for sanity.
Conversion to AirSim/MAVLink z-down happens only at the flight-adapter edge,
mirroring the grant's rule that frame conversion lives in the MAVLink adapter.
"""
from .models import (
    Action4D,
    AltitudeEnvelope,
    KinematicEnvelope,
    ObstacleClearance,
    Policy,
    PolygonFence,
    State,
    load_policy,
)
from .shield import Shield, ShieldDecision
from .audit import AuditLogger

__all__ = [
    "Action4D",
    "AltitudeEnvelope",
    "KinematicEnvelope",
    "ObstacleClearance",
    "Policy",
    "PolygonFence",
    "State",
    "load_policy",
    "Shield",
    "ShieldDecision",
    "AuditLogger",
]
