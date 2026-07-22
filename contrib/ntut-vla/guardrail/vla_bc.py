"""
BCVLA — the first LEARNED model in the VLA slot.

Same interface as StubVLA (state -> Action4D @ 10 Hz), but the action comes
from a behavior-cloned neural network trained on shield-corrected rollouts
(training/generate_dataset.py + train_bc.py) instead of hand-written math.

The point to demo: this model has (approximately) learned the rules by
imitation — it avoids no-fly zones on its own, so the Shield intervenes far
less. Shield stays on regardless: learned behaviour is a statistical habit,
the guardrail is the hard boundary. Defense in depth.
"""
from __future__ import annotations

from pathlib import Path

import torch

from .compiler import Mission
from .models import Action4D, Policy, State

_DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "models" / "bc_policy.pt"
_V3_MODEL = Path(__file__).resolve().parents[1] / "models" / "vla_policy_v2.pt"


class BCVLAv3:
    """The auto-research graduate: gate-passing policy (reach 97.5%, raw NFZ
    entry 0.5%, shield interventions 4.2% on 200 held-out scenarios).
    v3 features (26-dim, full zone geometry + prev action) + bounded decoder.
    Trained to stop at the target (hover samples) — no handover needed."""

    def __init__(self, mission, policy, model_path: str | Path = _V3_MODEL):
        from training.features import decode_action, encode_v3, make_fences
        from guardrail.models import AltitudeEnvelope
        self._encode = encode_v3
        self._decode = decode_action
        self._make_fences = make_fences
        self.mission = mission
        self._fences = make_fences(policy)
        self._prev = (0.0, 0.0, 0.0)
        envs = policy.by_type(AltitudeEnvelope)
        self._alt_band = (envs[0].alt_min_m, envs[0].alt_max_m) if envs else (None, None)
        self.model = torch.jit.load(str(model_path))
        self.model.eval()

    def refresh_fences(self, policy) -> None:
        self._fences = self._make_fences(policy)

    def act(self, state: State) -> Action4D:
        x = self._encode(state, self.mission.target_x, self.mission.target_y,
                         self.mission.cruise_alt_m, self._fences, self._prev)
        with torch.no_grad():
            y = self.model(torch.tensor([x], dtype=torch.float32))[0]
        vx, vy, vz = self._decode(float(y[0]), float(y[1]), float(y[2]),
                                  up=state.up, alt_min=self._alt_band[0],
                                  alt_max=self._alt_band[1])
        self._prev = (vx, vy, vz)
        return Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=0.0)


class BCVLA:
    def __init__(self, mission: Mission, policy: Policy,
                 model_path: str | Path = _DEFAULT_MODEL):
        # feature encoder needs the fence geometry (the model "sees" zones
        # through the same 8-feature encoding it was trained on)
        from training.features import ACTION_SCALE, encode, make_fences
        self._encode = encode
        self._scale = ACTION_SCALE
        self.mission = mission
        self._fences = make_fences(policy)
        self.model = torch.jit.load(str(model_path))
        self.model.eval()

    def refresh_fences(self, policy: Policy) -> None:
        """Call after a hot-apply so the model sees the new zone too."""
        from training.features import make_fences
        self._fences = make_fences(policy)

    HANDOVER_M = 6.0    # within this range, precision P-control takes over

    def act(self, state: State) -> Action4D:
        dx = self.mission.target_x - state.x
        dy = self.mission.target_y - state.y
        dist = (dx * dx + dy * dy) ** 0.5

        # Hybrid handover: the learned policy handles transit (where the rules
        # live); classical P-control does the final approach (where BC data is
        # sparse and the model tends to orbit the target). Same pattern as
        # learned-cruise + classical-docking in real systems.
        if dist < self.HANDOVER_M:
            v = min(2.0, dist)
            return Action4D(vx=dx / max(dist, 1e-6) * v,
                            vy=dy / max(dist, 1e-6) * v,
                            vz_up=0.8 * (self.mission.cruise_alt_m - state.up))

        x = self._encode(state, self.mission.target_x, self.mission.target_y,
                         self.mission.cruise_alt_m, self._fences)
        with torch.no_grad():
            y = self.model(torch.tensor([x], dtype=torch.float32))[0]
        return Action4D(vx=float(y[0]) * self._scale,
                        vy=float(y[1]) * self._scale,
                        vz_up=float(y[2]) * self._scale,
                        yaw_rate=0.0)
