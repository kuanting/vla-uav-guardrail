"""
Survey a Project AirSim city into a building occupancy map.

Flies a high-altitude lawnmower over the demo footprint, sampling the DOWN
depth camera's central window each tick. Building height at the drone's (x,y) =
altitude - down_depth; cells taller than BUILD_H are marked occupied.

Output: demo/out/citymap/occ.npz  (occ uint8 grid, res, origin_x, origin_y)
        demo/out/citymap/occ_preview.png

Contract (shared with demo/city_planner.py):
  grid coords: x = North, y = East. cell = round((v - origin)/res).
  origin_x = origin_y = -GRID_HALF ; size = 2*GRID_HALF/RES per side.
  occ[i, j] == 1  ->  building at  x = origin_x + i*res , y = origin_y + j*res.

Run (server up):  python demo/survey_city.py --alt 52
"""
from __future__ import annotations

import argparse
import asyncio
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"

GRID_HALF = 80.0     # world spans [-80, 80] m in x and y
RES = 2.0            # meters per cell
BUILD_H = 4.0        # roof height (m) above ground to count as a building


class DepthStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.depth = None

    def put(self, msg):
        try:
            if not msg or "data" not in msg or not len(msg["data"]):
                return
            data = msg["data"]
            raw = (np.array(data, dtype="B") if isinstance(data, list)
                   else np.frombuffer(data, dtype=np.uint8))
            h, w = msg["height"], msg["width"]
            if msg.get("encoding") == "16UC1" or raw.size == h * w * 2:
                arr = raw.view(np.uint16).reshape(h, w).astype(np.float32)
            else:
                arr = raw.view(np.float32).reshape(h, w).astype(np.float32)
        except Exception:
            return
        with self.lock:
            self.depth = arr

    def get(self):
        with self.lock:
            return self.depth


def to_cell(v):
    return int(round((v + GRID_HALF) / RES))


def quat_yaw(q: dict) -> float:
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt", type=float, default=52.0)
    ap.add_argument("--speed", type=float, default=6.0)
    ap.add_argument("--line-step", type=float, default=4.0, help="lawnmower spacing m")
    ap.add_argument("--half", type=float, default=70.0, help="survey half-extent m")
    args = ap.parse_args()

    from projectairsim import Drone, ProjectAirSimClient, World

    n = int(2 * GRID_HALF / RES)
    height = np.full((n, n), -1.0, dtype=np.float32)   # max roof height per cell
    store = DepthStore()

    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        drone = Drone(client, world, "Drone1")
        client.subscribe(drone.sensors["DownCamera"]["depth_camera"],
                         lambda _, msg: store.put(msg))
        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        # climb to survey altitude
        for _ in range(400):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if up >= args.alt - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.5, duration=0.2)
            await asyncio.sleep(0.08)
        print(f"[survey] at {args.alt} m — starting lawnmower")

        H = args.half
        n_lines = int(2 * H / args.line_step) + 1
        samples = 0
        for li in range(n_lines):
            x = -H + li * args.line_step
            y_from, y_to = (-H, H) if li % 2 == 0 else (H, -H)
            # go to line start
            await goto(drone, x, y_from, args.alt, store, height, samples_ref=None)
            # sweep the line, sampling continuously
            samples += await sweep(drone, x, y_from, y_to, args.alt, args.speed,
                                   store, height)
            print(f"[survey] line {li + 1}/{n_lines} x={x:.0f}  samples={samples}")
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    finally:
        client.disconnect()

    occ_raw = (height > BUILD_H).astype(np.uint8)
    occ = _close_gaps(occ_raw)              # fill cross-track stripe holes
    obs = height > -0.5
    print(f"[survey] observed cells: {int(obs.sum())} / {occ.size} "
          f"({100*obs.mean():.0f}%)  roof p50={np.median(height[obs]):.1f}m "
          f"p95={np.percentile(height[obs],95):.1f}m")
    print(f"[survey] building cells raw={int(occ_raw.sum())} closed={int(occ.sum())}")
    out = ROOT / "demo" / "out" / "citymap"
    out.mkdir(parents=True, exist_ok=True)
    np.savez(out / "occ.npz", occ=occ, height=height, res=RES,
             origin_x=-GRID_HALF, origin_y=-GRID_HALF, build_h=BUILD_H)
    print(f"[survey] occupied cells: {int(occ.sum())} / {occ.size}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        ext = [-GRID_HALF, GRID_HALF, -GRID_HALF, GRID_HALF]
        # imshow with y=East on x-axis, x=North on y-axis
        ax.imshow(np.where(height < 0, np.nan, height).T, origin="lower",
                  extent=ext, cmap="viridis", alpha=0.9)
        occ_disp = np.where(occ.T == 1, 1.0, np.nan)
        ax.imshow(occ_disp, origin="lower", extent=ext, cmap="autumn", alpha=0.4)
        ax.set_xlabel("North x (m)")
        ax.set_ylabel("East y (m)")
        ax.set_title(f"City occupancy — {int(occ.sum())} building cells (>{BUILD_H}m)")
        fig.tight_layout()
        fig.savefig(out / "occ_preview.png", dpi=130)
        print(f"[survey] preview -> {out / 'occ_preview.png'}")
    except Exception as e:
        print(f"[survey] preview skipped: {e}")


async def goto(drone, tx, ty, alt, store, height, samples_ref):
    for _ in range(600):
        kin = drone.get_ground_truth_kinematics()
        p = kin["pose"]["position"]
        x, y, up = p["x"], p["y"], -p["z"]
        if math.hypot(tx - x, ty - y) < 1.5:
            return
        d = math.hypot(tx - x, ty - y)
        vx, vy = (tx - x) / d * 6.0, (ty - y) / d * 6.0
        await drone.move_by_velocity_async(vx, vy, -0.8 * (alt - up), duration=0.3)
        await asyncio.sleep(0.08)


# --- per-pixel registration cache (built lazily once we know image size) ---
_PIX = {}


def _register(dm, cx, cy, up, yaw, height):
    """Project every (sub-sampled) down-depth pixel to a ground cell and record
    the roof height there. Fills the WHOLE camera footprint (~100 m at 52 m),
    not just the nadir cell — this is what closes the striping holes.

    Down camera: optical axis points straight down; image 'up' (-row) = body
    forward (+North at yaw 0), image 'right' (+col) = body right (+East at
    yaw 0). Pinhole fx = fy = (W/2)/tan(hfov/2), hfov = 90deg -> fx = W/2.
    depth is PERSPECTIVE (ray length); vertical drop = d / sqrt(1+nx^2+ny^2).
    """
    H_img, W_img = dm.shape
    key = (H_img, W_img)
    if key not in _PIX:
        fx = (W_img / 2.0) / math.tan(math.radians(90.0) / 2.0)   # = W/2
        # subsample the central 80% of the frame (edges unreliable), stride 3
        us = np.arange(int(W_img * 0.1), int(W_img * 0.9), 3)
        vs = np.arange(int(H_img * 0.1), int(H_img * 0.9), 3)
        uu, vv = np.meshgrid(us, vs)
        nx = (uu - W_img / 2.0) / fx        # right
        ny = (vv - H_img / 2.0) / fx        # down-in-image
        _PIX[key] = (uu.ravel(), vv.ravel(), nx.ravel(), ny.ravel())
    uu, vv, nx, ny = _PIX[key]
    d = dm[vv, uu].astype(np.float32)
    valid = (d > 0.5) & (d < 250.0)
    if not valid.any():
        return 0
    d, nxv, nyv = d[valid], nx[valid], ny[valid]
    # image-type 2 returns PLANAR depth (Z = vertical drop directly). Treating
    # it as planar is safe either way: ground -> roof~0 (free); a building roof
    # -> still > threshold. (Treating planar data as perspective over-marks the
    # frame edges, smearing streets into "buildings".)
    vert = d                                # vertical drop below camera (m)
    roof = up - vert                        # roof height above ground
    # horizontal ground offset from nadir = Z * tan(angle) ~= d * n (pinhole)
    off_e0 = nxv * d                         # camera-right  -> East at yaw 0
    off_n0 = -nyv * d                        # camera-up(-ny) -> North at yaw 0
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    off_n = off_n0 * cy_ - off_e0 * sy_      # rotate by yaw into world
    off_e = off_n0 * sy_ + off_e0 * cy_
    wx = cx + off_n                          # x = North
    wy = cy + off_e                          # y = East
    ii = np.round((wx + GRID_HALF) / RES).astype(int)
    jj = np.round((wy + GRID_HALF) / RES).astype(int)
    inb = (ii >= 0) & (ii < height.shape[0]) & (jj >= 0) & (jj < height.shape[1])
    ii, jj, roof = ii[inb], jj[inb], roof[inb]
    # keep the MAX roof per cell (buildings win over adjacent ground samples)
    np.maximum.at(height, (ii, jj), roof)
    return int(inb.sum())


async def sweep(drone, x, y0, y1, alt, speed, store, height):
    """NADIR-only sampling: mark the single cell directly below the drone from
    the down-depth center window. This is trig-free and always correct (roof =
    altitude - depth_below); density comes from FINE line spacing, and the
    remaining cross-track gaps are closed morphologically after the survey.
    Per-pixel footprint registration was tried but the down-depth semantics
    (scale/planar) smeared streets — nadir is the reliable primitive."""
    took = 0
    dirn = 1.0 if y1 > y0 else -1.0
    for _ in range(2000):
        kin = drone.get_ground_truth_kinematics()
        p = kin["pose"]["position"]
        cx, cy, up = p["x"], p["y"], -p["z"]
        if (dirn > 0 and cy >= y1) or (dirn < 0 and cy <= y1):
            break
        vy = dirn * speed
        await drone.move_by_velocity_async(0.0, vy, -0.8 * (alt - up), duration=0.3)
        dm = store.get()
        if dm is not None:
            h, w = dm.shape
            win = dm[h // 2 - 8:h // 2 + 8, w // 2 - 8:w // 2 + 8]
            win = win[(win > 0.5) & (win < 250)]
            if win.size:
                roof = up - float(np.median(win))
                i, j = to_cell(cx), to_cell(cy)
                if 0 <= i < height.shape[0] and 0 <= j < height.shape[1]:
                    if roof > height[i, j]:
                        height[i, j] = roof
                    took += 1
        await asyncio.sleep(0.06)
    return took


def _close_gaps(occ):
    """Morphological closing (dilate then erode, 3x3) to fill the cross-track
    stripe gaps inside contiguous building blobs without leaking into streets."""
    def dil(a):
        out = a.copy()
        for si in (-1, 0, 1):
            for sj in (-1, 0, 1):
                out[max(si,0):a.shape[0]+min(si,0),
                    max(sj,0):a.shape[1]+min(sj,0)] |= a[
                    max(-si,0):a.shape[0]+min(-si,0),
                    max(-sj,0):a.shape[1]+min(-sj,0)]
        return out
    def ero(a):
        inv = 1 - a
        return 1 - dil(inv)
    return ero(dil(occ.astype(np.uint8)))


if __name__ == "__main__":
    asyncio.run(main())
