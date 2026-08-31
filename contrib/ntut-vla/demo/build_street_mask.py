"""
Build a STREET mask, which is a different question from an obstacle map.

Why this exists
---------------
An occupancy map answers "can the drone be here at cruise altitude". Several
places in this repository also needed "is this on the road", and inferred it
from the obstacle map by treating free space as road. That worked only while the
obstacle map was built by collapsing voxels over 15-55 m AGL, because then it
held nothing but buildings and every road was free by construction.

Once the obstacle map was rebuilt over the band the aircraft actually flies in
(6-14 m), the inference broke in both directions at once:

  * A canopy OVER a road is occupied at 9 m and perfectly drivable underneath,
    so free-space-means-road called a real road un-drivable. This is what failed
    tests/test_city_traffic.py at (38.0, 22.1) - the car route crossing a tree.

  * Low structures a drone can legally overfly became free at 6-14 m when they
    had been blocked at 15-55 m, so free-space-means-road offered detours over
    rooftops. This is what failed the fence guard's road test.

Three bands, three different questions
--------------------------------------
    2-4 m     low clutter: kerbs, parked vehicles, walls, tree trunks
    6-14 m    what the aircraft can hit at a 9 m cruise  -> the obstacle map
    15-55 m   only things that tall  -> buildings

A cell is STREET when it is free of buildings AND free of low clutter. Buildings
are what makes this work: they are the only structures tall enough to appear in
the 15-55 m band, so that band is a building footprint mask rather than the
obstacle map it was being used as. Low clutter alone is not enough to identify a
building, because the voxel grid reports building interiors as empty at 2-4 m -
walls are surfaces, not solids - so a low-band test alone calls the inside of a
city block a road.

A cell is a CANOPY when it is street below and blocked at cruise: drivable by a
car, closed to the aircraft. There are 128 of them in this map, and they are
exactly the cells that made "free space means road" wrong.

Usage (with the three band maps already built by build_voxel_map.py):
    python demo/build_street_mask.py
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CITYMAP = ROOT / "demo" / "out" / "citymap"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--low", default="ground_2to4",
                    help="occupancy over ~2-4 m AGL (kerbs, walls, trunks)")
    ap.add_argument("--tall", default="occ_day_highband_15to55",
                    help="occupancy over ~15-55 m AGL; only buildings reach this")
    ap.add_argument("--cruise", default="occ_day_flightband_6to14",
                    help="occupancy over the flight band, for the canopy count")
    ap.add_argument("--out", default="street")
    args = ap.parse_args()

    def load(name):
        p = CITYMAP / f"{name}.npz"
        if not p.is_file():
            raise SystemExit(f"missing {p}. Build it with:\n"
                             f"    python demo/build_voxel_map.py --out {name} "
                             f"--band-lo <lo> --band-hi <hi>")
        return np.load(p)

    low, tall, cruise = load(args.low), load(args.tall), load(args.cruise)
    G, B, F = low["occ"], tall["occ"], cruise["occ"]
    if not (G.shape == B.shape == F.shape):
        raise SystemExit(f"grids disagree: {G.shape} {B.shape} {F.shape}")

    street = ((B == 0) & (G == 0)).astype(np.uint8)
    canopy = int((street.astype(bool) & (F == 1)).sum())

    out = CITYMAP / f"{args.out}.npz"
    np.savez_compressed(
        out, street=street, res=tall["res"],
        origin_x=tall["origin_x"], origin_y=tall["origin_y"])

    n = street.size
    print(f"[street] buildings (tall band)     : {int((B == 1).sum()):5d} / {n}")
    print(f"[street] low clutter               : {int((G == 1).sum()):5d} / {n}")
    print(f"[street] STREET (free of both)     : {int(street.sum()):5d} / {n} "
          f"({street.mean():.3f})")
    print(f"[street] canopy over street        : {canopy:5d}  "
          f"(drivable below, blocked at cruise)")
    print(f"[street] saved {out}")
    return 0


def load_street(path: str | Path = None) -> dict:
    """Read the mask back. `street[i, j] == 1` means the cell is on a road."""
    p = Path(path) if path else (CITYMAP / "street.npz")
    d = np.load(p)
    return {"street": d["street"], "res": float(d["res"]),
            "ox": float(d["origin_x"]), "oy": float(d["origin_y"])}


def is_street(mask: dict, x: float, y: float) -> bool:
    """Is the world point (x North, y East) on a road?

    Outside the grid returns False: an unmapped cell is not evidence of a road,
    and the alternative would silently approve every route that leaves the map.

    ROUND, not int(). Cell `i` is CENTRED at `origin + i*res` - the convention
    demo/city_planner.py documents at the top of the file and
    demo/build_voxel_map.py uses when it writes these grids. This mask is a
    cell-wise AND of those grids and inherits their origin unchanged, so it
    shares the convention.

    Truncating instead reads the mask shifted by up to half a cell, which is
    1.0 m at this map's 2.0 m resolution. Measured over 40 000 random points,
    the two forms disagreed about whether a point was on a road **9.4 % of the
    time**.
    """
    i = int(round((x - mask["ox"]) / mask["res"]))
    j = int(round((y - mask["oy"]) / mask["res"]))
    s = mask["street"]
    if not (0 <= i < s.shape[0] and 0 <= j < s.shape[1]):
        return False
    return bool(s[i, j])


if __name__ == "__main__":
    sys.exit(main())
