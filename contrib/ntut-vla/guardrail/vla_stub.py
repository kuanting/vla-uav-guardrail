"""
Stub VLA — stands in the VLA Backend slot until a real model arrives.

Interface contract (the part that must NOT change when the real VLA lands):

    observation (State [+ camera frame])  ->  Action4D at 10 Hz

The stub is a naive proportional controller that flies straight at the target.
It is DELIBERATELY rule-ignorant and a bit reckless (asks for more speed than
the policy allows): its job is to behave like an untrusted model so the
Guardrail has something real to catch.
"""
from __future__ import annotations

from .compiler import Mission
from .models import Action4D, State


class StubVLA:
    def __init__(self, mission: Mission):
        self.mission = mission

    def act(self, state: State) -> Action4D:
        dx = self.mission.target_x - state.x
        dy = self.mission.target_y - state.y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 1e-6:
            return Action4D()
        speed = min(self.mission.speed_pref_mps, dist)   # slow near target
        vz = 0.8 * (self.mission.cruise_alt_m - state.up)  # hold cruise altitude
        return Action4D(vx=dx / dist * speed, vy=dy / dist * speed, vz_up=vz)
