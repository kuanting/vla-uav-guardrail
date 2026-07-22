"""
VLA backend adapters — the swappable slot.

The grant's core rule: the VLA is a *pluggable* backend. Everything downstream
(Compiler, Shield, MAVLink) only cares that whatever sits in the slot honours
ONE contract:

    observation  ->  Action4D(vx, vy, vz_up, yaw_rate)   at 10 Hz

That is the entire interface. "Plugging a model in" = writing a small adapter
class with a single `.act(state) -> Action4D` method that (1) gathers the
inputs the model wants, (2) runs the model, (3) maps its output onto the 4-D
body-frame contract. Nothing else in the stack changes.

This file gives the Protocol + three reference adapters at increasing realism:
    StubBackend      hand-written, no model            (already have: vla_stub)
    PolicyBackend    our trained state+geometry policy (already have: vla_bc)
    AeroVLABackend   a REAL camera+language VLA         (skeleton — fill in)
"""
from __future__ import annotations

from typing import Protocol

from .models import Action4D, State


# --------------------------------------------------------------------------- #
# The contract every backend must satisfy — this IS the slot.
# --------------------------------------------------------------------------- #
class VLABackend(Protocol):
    def act(self, state: State) -> Action4D:
        """One 10 Hz tick: observation -> 4-D body-frame action."""
        ...

    # optional: called after a mid-flight policy change (hot-apply)
    def refresh_fences(self, policy) -> None: ...


# --------------------------------------------------------------------------- #
# REAL VLA adapter skeleton — e.g. AeroVLA (OpenVLA-7B + LoRA) or CognitiveDrone.
#
# A real VLA takes a CAMERA IMAGE + a natural-language INSTRUCTION and returns
# an action. So the adapter's job at each tick:
#   1. grab the current camera frame (from the sim/flight adapter)
#   2. hold the instruction text (set once per mission)
#   3. run the model  ->  raw action (its own units/convention)
#   4. remap raw action onto OUR 4-D contract (vx, vy, vz_up, yaw_rate)
#
# The model is heavy (7B). Two realities to respect:
#   - Load ONCE (in __init__), never per tick.
#   - Inference may be slower than 10 Hz on a 4080/Orin. Mitigation: run the VLA
#     at a lower rate and hold its last action for the ticks in between (action
#     chunking), or quantize (4-bit). Flagged, not solved here.
# --------------------------------------------------------------------------- #
class AeroVLABackend:
    """Skeleton for a real camera+language VLA in the slot.

    Fill the three TODOs. Everything downstream (Shield, MAVLink, KPIs) is
    unchanged — that is the whole point of the swappable slot.
    """

    def __init__(self, instruction: str, get_frame, model_dir: str,
                 vla_hz: float = 3.0):
        """
        instruction : the mission text, e.g. "follow the blue car"
        get_frame   : callable() -> HxWx3 uint8 RGB (the sim/flight camera)
        model_dir   : path to the VLA weights (OpenVLA-7B base + AeroVLA LoRA)
        vla_hz      : how often to actually run the VLA (held between)
        """
        self.instruction = instruction
        self.get_frame = get_frame
        self._every = max(1, round(10.0 / vla_hz))   # run once per N ticks
        self._tick = 0
        self._last = Action4D()

        # TODO 1 — load the model ONCE. Example (AeroVLA / OpenVLA):
        #   import torch
        #   from transformers import AutoModelForVision2Seq, AutoProcessor
        #   self.proc  = AutoProcessor.from_pretrained(model_dir, trust_remote_code=True)
        #   self.model = AutoModelForVision2Seq.from_pretrained(
        #       model_dir, torch_dtype=torch.bfloat16, load_in_4bit=True,
        #       trust_remote_code=True).eval().to("cuda")
        self.model = None
        self.proc = None

    def act(self, state: State) -> Action4D:
        self._tick += 1
        # action chunking: only run the heavy model every N ticks, hold otherwise
        if self._tick % self._every != 1 and self._last is not None:
            return self._last

        frame = self.get_frame()                     # HxWx3 RGB uint8

        # TODO 2 — run the model. Example shape:
        #   inputs = self.proc(self.instruction, frame).to("cuda", torch.bfloat16)
        #   raw = self.model.predict_action(**inputs)   # model's own action vector
        raw = None

        # TODO 3 — map the model's raw action onto OUR 4-D body-frame contract.
        # Every VLA emits a slightly different action; adapt units + axes here.
        # (CognitiveDrone / AeroVLA already emit ~4-D velocity — near 1:1.)
        #   act = Action4D(vx=float(raw[0]), vy=float(raw[1]),
        #                  vz_up=float(raw[2]), yaw_rate=float(raw[3]))
        act = self._last if raw is None else raw     # placeholder until filled

        self._last = act
        return act

    def refresh_fences(self, policy) -> None:
        # a camera VLA doesn't read the policy geometry directly — the Shield
        # enforces it downstream. No-op unless you also feed zones into the prompt.
        pass


# --------------------------------------------------------------------------- #
# Factory: choose the slot occupant by name. Wire this into run_demo (--vla).
# --------------------------------------------------------------------------- #
def make_backend(name: str, mission, policy, **kw) -> VLABackend:
    if name == "stub":
        from .vla_stub import StubVLA
        return StubVLA(mission)
    if name == "v3":
        from .vla_bc import BCVLAv3
        return BCVLAv3(mission, policy)
    if name == "aerovla":
        # requires get_frame + model_dir passed in kw (only in a sim/flight loop)
        return AeroVLABackend(instruction=mission.task_text,
                              get_frame=kw["get_frame"],
                              model_dir=kw["model_dir"])
    raise ValueError(f"unknown VLA backend: {name!r}")
