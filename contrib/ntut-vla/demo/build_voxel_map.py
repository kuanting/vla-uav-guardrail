"""
Build an ACCURATE occupancy map from Project AirSim's ground-truth geometry.

Unlike the camera survey (survey_city.py, which had holes + registration error),
this asks the sim itself for a voxel grid via world.create_voxel_grid — so the
map is 1:1 with the actual buildings. Collapses the voxels inside the flight
altitude band to a 2-D occupancy grid in our planner format.

Output: demo/out/citymap/occ.npz  (occ uint8 NxN, res, origin_x, origin_y)
        demo/out/citymap/occ_preview.png
Then copy to occ_<map>.npz for the world you surveyed.

Run (server on the target world):  python demo/build_voxel_map.py --out occ_day
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_guardrail.jsonc"

GRID_HALF = 80.0     # world spans [-80, 80] m in x(North) and y(East)
RES = 2.0            # meters per cell (matches the planner contract)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="occ", help="output basename (occ_day, …)")
    ap.add_argument("--band-lo", type=float, default=15.0, help="flight band low (m AGL)")
    ap.add_argument("--band-hi", type=float, default=55.0, help="flight band high (m AGL)")
    args = ap.parse_args()

    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import Pose, Quaternion, Vector3

    client = ProjectAirSimClient()
    client.connect()
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)
        Drone(client, world, "Drone1")           # ensure the scene is live

        edge = 2 * GRID_HALF                       # 160 m cube
        nc = int(edge / RES)                       # 80 cells per axis
        center = Pose({
            "translation": Vector3({"x": 0.0, "y": 0.0, "z": 0.0}),
            "rotation": Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}),
        })
        print(f"[voxel] requesting {nc}x{nc}x{nc} grid @ {RES} m ...")
        flat = world.create_voxel_grid(center, int(edge), int(edge), int(edge), int(RES))
        arr = np.asarray(flat, dtype=bool)
        print(f"[voxel] got {arr.size} voxels, occupied {int(arr.sum())} "
              f"({100*arr.mean():.1f}%)")
        # flat index = x + nc*(z + nc*y)  ->  reshape [y][z][x]
        vox = arr.reshape(nc, nc, nc)              # [y_idx, z_idx, x_idx]

        # find the GROUND z-slice = the densest-occupied horizontal slice
        occ_per_z = vox.sum(axis=(0, 2))           # occupancy count per z index
        gz = int(np.argmax(occ_per_z))
        print(f"[voxel] ground z-index = {gz} (occ {int(occ_per_z[gz])}); "
              f"per-z occ head/tail: {occ_per_z[:6]} ... {occ_per_z[-6:]}")

        # Buildings sit ABOVE ground. We don't know the z sign (NED vs NEU), so
        # take the band on BOTH sides of the ground slice and OR them — the empty
        # (sky) side contributes nothing, so this is sign-agnostic and correct.
        lo_c = int(round(args.band_lo / RES))
        hi_c = int(round(args.band_hi / RES))
        band_mask = np.zeros((nc, nc), dtype=bool)  # [y][x]
        for a, b in ((gz + lo_c, gz + hi_c), (gz - hi_c, gz - lo_c)):
            a2, b2 = max(0, min(a, nc)), max(0, min(b + 1, nc))
            if b2 > a2:
                band_mask |= vox[:, a2:b2, :].any(axis=1)
        print(f"[voxel] flight-band = ground z {gz} +/- [{lo_c},{hi_c}] cells "
              f"(~{args.band_lo:.0f}-{args.band_hi:.0f} m AGL)")
        occ2d_yx = band_mask                        # [y][x]  building in band?

        # our planner grid is occ[i=North(x)][j=East(y)]; here axis0=y, axis2=x
        occ = occ2d_yx.T.astype(np.uint8)           # -> [x][y]
        print(f"[voxel] 2-D building cells in band: {int(occ.sum())} / {occ.size}")

        # sanity: the spawn (35 N, -20 E) sits on an open street -> must be FREE
        si = int(round((35.0 + GRID_HALF) / RES))
        sj = int(round((-20.0 + GRID_HALF) / RES))
        print(f"[voxel] spawn cell ({si},{sj}) occupied = {bool(occ[si, sj])} "
              f"(expect False)")
    finally:
        client.disconnect()

    out = ROOT / "demo" / "out" / "citymap"
    out.mkdir(parents=True, exist_ok=True)
    npz = out / f"{args.out}.npz"
    np.savez(npz, occ=occ, res=RES, origin_x=-GRID_HALF, origin_y=-GRID_HALF)
    # also write the generic name the GUI/flight default to
    np.savez(out / "occ.npz", occ=occ, res=RES, origin_x=-GRID_HALF, origin_y=-GRID_HALF)
    print(f"[voxel] saved {npz}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        ext = [-GRID_HALF, GRID_HALF, -GRID_HALF, GRID_HALF]
        disp = np.where(occ.T == 1, 1.0, np.nan)
        ax.imshow(disp, origin="lower", extent=ext, cmap="autumn")
        ax.plot(-20, 35, "g*", markersize=14, label="spawn")
        ax.set_xlabel("North x (m)"); ax.set_ylabel("East y (m)")
        ax.set_title(f"Ground-truth voxel occupancy — {int(occ.sum())} building cells")
        ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "occ_preview.png", dpi=130)
        print(f"[voxel] preview -> {out / 'occ_preview.png'}")
    except Exception as e:
        print(f"[voxel] preview skipped: {e}")


if __name__ == "__main__":
    main()
