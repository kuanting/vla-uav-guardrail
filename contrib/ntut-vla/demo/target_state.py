"""Where the target IS, rather than where the last box put it.

WHY THIS EXISTS

The follow controller was proportional on the detector's instantaneous output,
and the detector is noisy. Measured over the demo flights:

  * the box WIDTH, which drove the forward servo, jitters p95 37.7% and max 71%
    between consecutive detections in traffic. Through
    `fwd = ((want_w - w_frac)/want_w) * speed_max` a 30% width jump is about
    5.6 m/s of commanded forward change;
  * the raw forward command therefore stepped up to **1.897 m/s in one 0.1 s
    tick**, and the slew limiter clipped it on **20% of ticks** - masking the
    noise while adding phase lag;
  * the detector runs at 3.8-5.6 Hz against a 10 Hz control loop, so **62% of
    ticks reused a stale box** and then jumped when a new one arrived.

Filtering the controller was the obvious answer and it has already been flown and
lost: docs/DESIGN-orbit-building-task.md records a low-pass plus slew on the
radial loop making radius hold clearly worse, 24.3 +- 10.1 m against
33.6 +- 25.2 m, because "the filter costs more phase than it buys in noise".

That document also names what was never tried, and this module is it:

    "a radial term driven by something other than instantaneous depth - for
     instance integrating the tangential motion to estimate the subject's
     position and servoing on that. That is a different design, not a further
     tuning pass."

So: estimate the target's position and velocity with a motion model, update it
when a detection arrives, and PREDICT between detections. The controller then
sees a smooth signal because the smoothness comes from the model rather than
from a filter bolted onto the error.

WHAT MAY AND MAY NOT GO IN

Only the detector's box, the depth camera, and the aircraft's own pose. No target
ground truth, ever. The follow demo's standing claim is that the only steering
input is where the detector puts the box, and this must not quietly become
untrue: `tgt_x`/`tgt_y` in the flight log are for SCORING and must never reach
this class.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


class TargetState:
    """Constant-velocity Kalman filter on the target's world position.

    State is `[x, y, vx, vy]` in world NED metres. Polar measurements are
    converted to Cartesian with a properly rotated covariance rather than
    linearised, which keeps the update exact for the measurement we actually
    have and avoids an EKF's Jacobian bookkeeping.
    """

    def __init__(self, accel_noise: float = 1.5, range_sigma_m: float = 1.5,
                 bearing_sigma_deg: float = 2.0, gate_sigma: float = 4.0,
                 max_coast_s: float = 3.0):
        # Process noise as an acceleration the target might apply between
        # updates. A car pulling away from a stop is around 1-2 m/s^2, and the
        # route's speed profile is built with lon_acc 1.2, so 1.5 covers it
        # without letting the estimate wander.
        self.q = float(accel_noise)
        # Depth arrives QUANTISED TO WHOLE METRES (see semantic_demo.get_depth;
        # the logged rng_m values are all integers or .5 from a median of an
        # even sample). A 1 m quantum is a +-0.5 m uniform error, sigma 0.29,
        # but the box the depth is sampled through wanders too, so 1.5 m is the
        # honest figure rather than the arithmetic one.
        self.r_sig = float(range_sigma_m)
        self.b_sig = math.radians(float(bearing_sigma_deg))
        self.gate = float(gate_sigma)
        self.max_coast_s = float(max_coast_s)

        self.x: Optional[np.ndarray] = None      # [x, y, vx, vy]
        self.P: Optional[np.ndarray] = None
        self.t_last_update: Optional[float] = None
        self.n_updates = 0
        self.n_rejected = 0

    # ------------------------------------------------------------------ core --

    def _measure(self, drone_x: float, drone_y: float, yaw: float,
                 bearing: float, rng: float):
        """Polar measurement about the aircraft -> Cartesian world position + R."""
        th = yaw + bearing
        c, s = math.cos(th), math.sin(th)
        z = np.array([drone_x + rng * c, drone_y + rng * s], float)
        # J maps (range, bearing) errors into (x, y) errors.
        J = np.array([[c, -rng * s], [s, rng * c]], float)
        R = J @ np.diag([self.r_sig ** 2, self.b_sig ** 2]) @ J.T
        return z, R

    def predict(self, t: float) -> None:
        """Advance the estimate to time `t`. Safe to call every control tick."""
        if self.x is None or self.t_last_update is None:
            return
        dt = t - getattr(self, "_t_state", self.t_last_update)
        if dt <= 0:
            return
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        # Continuous white-acceleration model discretised over dt.
        q = self.q ** 2
        d2, d3, d4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([
            [d4 / 4, 0, d3 / 2, 0],
            [0, d4 / 4, 0, d3 / 2],
            [d3 / 2, 0, d2, 0],
            [0, d3 / 2, 0, d2],
        ], float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self._t_state = t

    def update(self, t: float, drone_x: float, drone_y: float, yaw: float,
               bearing: float, rng: float) -> bool:
        """Fold in one detection. Returns False if the measurement was gated out.

        Gating is what stops a confident lock on the WRONG vehicle from being
        propagated as though it were the right one - the risk this design adds
        over forgetting everything between frames.
        """
        z, R = self._measure(drone_x, drone_y, yaw, bearing, rng)
        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0], float)
            self.P = np.diag([R[0, 0], R[1, 1], 4.0, 4.0])
            self.t_last_update = t
            self._t_state = t
            self.n_updates = 1
            return True

        self.predict(t)
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        # Squared Mahalanobis distance: how surprising is this measurement?
        d2 = float(y @ Sinv @ y)
        if self.n_updates >= 3 and d2 > self.gate ** 2:
            self.n_rejected += 1
            return False
        K = self.P @ H.T @ Sinv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.t_last_update = t
        self._t_state = t
        self.n_updates += 1
        return True

    # ------------------------------------------------------------- readout --

    def observe(self, t: float, drone_x: float, drone_y: float,
                yaw: float) -> Optional[Tuple[float, float]]:
        """(bearing rad, range m) from the estimate, or None if unusable.

        This is what the controller servos on. It is available on EVERY tick,
        including the 62% that used to reuse a stale box, because it comes from
        the prediction rather than from the last message.
        """
        if self.x is None or self.t_last_update is None:
            return None
        if t - self.t_last_update > self.max_coast_s:
            return None                     # too long unseen to be trusted
        self.predict(t)
        dx, dy = self.x[0] - drone_x, self.x[1] - drone_y
        rng = math.hypot(dx, dy)
        if rng < 0.5:
            return None
        bearing = math.atan2(dy, dx) - yaw
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        return bearing, rng

    def range_rate(self, drone_x: float, drone_y: float) -> float:
        """How fast the target is opening the range, m/s, from ITS motion alone.

        This is what makes an integral term unnecessary. A proportional
        stand-off loop against a moving target settles at a lag error - measured
        in flight, a 2 m/s car and a 0.25 gain held 23.4 m against a 15.8 m
        stand-off, exactly the (r - want)*gain = 2.0 fixed point. The textbook
        fix is an I term, which would wind up every time the Shield overrides
        the actuator. The estimator already knows the target's velocity, so feed
        it FORWARD instead: no memory, no windup, and it responds immediately
        when the car stops rather than after an integrator drains.
        """
        if self.x is None:
            return 0.0
        dx, dy = self.x[0] - drone_x, self.x[1] - drone_y
        r = math.hypot(dx, dy)
        if r < 1e-3:
            return 0.0
        return float((self.x[2] * dx + self.x[3] * dy) / r)

    def speed(self) -> float:
        """Estimated ground speed of the target, m/s."""
        return 0.0 if self.x is None else float(math.hypot(self.x[2], self.x[3]))

    def age_s(self, t: float) -> Optional[float]:
        return None if self.t_last_update is None else t - self.t_last_update

    def reset(self) -> None:
        self.x = None
        self.P = None
        self.t_last_update = None

    def summary(self) -> dict:
        return {"updates": self.n_updates, "gated_out": self.n_rejected,
                "speed_mps": round(self.speed(), 2)}


def want_range_from_width(want_w_frac: float, object_width_m: float = 4.0,
                          hfov_deg: float = 90.0) -> float:
    """The stand-off that a target apparent-width fraction was really asking for.

    Keeps `--want-width` meaningful after the servo stops using apparent width,
    so existing command lines and every measured flight stay comparable. At
    want_w_frac 0.16 and a 4 m car this is 15.8 m, which matches the 13-15 m
    mean separation the width servo actually held.
    """
    half = math.radians(hfov_deg / 2.0) * max(1e-3, want_w_frac)
    return (object_width_m / 2.0) / math.tan(half)
