"""
Spawn, verify, recolour and remove the semantic target for the coexistence
experiment.

Why a spawned prop rather than one of the city's own buildings:

  1. The strongest control condition ("same scene, target absent") needs the
     scene to be identical except that the object is gone. `destroy_object` makes
     that trivial for a spawned prop and impossible for baked geometry.
  2. AerialVLA's object vocabulary is colour+noun street objects — all 94 entries
     in source/AeroVLA/data/meta/object_description.json look like "red car",
     "white truck", "black dog". Not one architectural phrase. A building target
     would confound "does it follow the language?" with "does it know the word?".
  3. Colour is the discriminative attribute, which enables the tightest possible
     language ablation: same noun, wrong colour, identical pixels.
  4. `get_object_pose` gives ground truth read back FROM the sim, so scoring never
     depends on what we believe we commanded.

Nothing in here is imported by the flight loop's steering path — the flight
script uses it only to place the object and to record its true pose.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


@dataclass
class TargetSpec:
    """One candidate way of putting a findable object into the world.

    `desc_match` is the phrase the VLA is given when it should find this object;
    `desc_mismatch` names a *different* object of the same kind, so the language
    ablation changes one word and nothing else.
    """

    key: str
    asset: str
    source: str = "registry"          # "registry" (packaged) | "glb" (client file)
    scale: List[float] = field(default_factory=lambda: [6.0, 6.0, 14.0])
    z_ned: float = -7.0               # NED: negative is up; centre of the prop
    material: Optional[str] = None
    desc_match: str = "red truck"
    desc_mismatch: str = "blue truck"
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)


# Ordered best-first. `pick_spec` walks this list against what the sim actually
# reports, so a missing asset degrades to the next candidate instead of crashing
# 70 minutes into a batch.
CANDIDATES: List[TargetSpec] = [
    # Verified working 2026-08-04: spawns, recolours, and reads back its pose
    # exactly. 12 m wide subtends ~15 deg at the 46 m start range, about 37 px of
    # the 224 px image — small, but unambiguous. 16 m tall keeps its top 2 m below
    # the 18 m altitude floor, so the drone cannot hit it.
    #
    # On the description pair: the sim ships no red material (only Orange, Blue,
    # Green, Yellow under Content/Geometry/Materials), so "red truck" was not an
    # option. The noun is "barrier" rather than "car" or "truck" because the
    # JapaneseCity streets contain REAL cars — a "red car" mismatch arm could be
    # satisfied by an actual red car, which would confound the language ablation.
    # "Orange StreetBarrier" is in the training vocabulary and no street barriers
    # of these colours exist in the scene.
    TargetSpec(
        key="cube",
        asset="1M_Cube",
        scale=[12.0, 12.0, 16.0],
        z_ned=-8.0,
        # Orange, not blue: a blue cube was probed at this exact pose and was
        # indistinguishable from the pale glass-and-concrete buildings behind it.
        # Orange is the only colour in this scene with no natural competitor.
        material="/Game/Geometry/Materials/M_Orange",
        desc_match="orange barrier", desc_mismatch="blue barrier",
    ),
    TargetSpec(
        key="pad",
        asset="BasicLandingPad",
        scale=[8.0, 8.0, 8.0],
        z_ned=-7.0,
        # stood on its edge so it reads as a tall billboard rather than a disc on
        # the ground — see the FOV note in pick_spec's docstring
        rpy=(0.0, math.pi / 2, 0.0),
        material="/ProjectAirSim/Weather/WeatherFX/Materials/M_Leaf_master",
        desc_match="red truck", desc_mismatch="blue truck",
    ),
    TargetSpec(
        key="airtaxi",
        asset="AirTaxi.glb",
        source="glb",
        scale=[4.0, 4.0, 4.0],
        z_ned=-9.0,
        material=None,
        desc_match="white aircraft", desc_mismatch="black aircraft",
    ),
]

GLB_DIR = r"D:\ProjectAirSim\repo\client\python\example_user_scripts\assets"


def discover_assets(world, regex: str = ".*") -> List[str]:
    """Ask the sim what it can actually spawn. Never guess this."""
    try:
        return list(world.list_assets(regex))
    except Exception as e:                          # older server, or RPC refused
        print(f"[target] list_assets failed ({type(e).__name__}: {e})")
        return []


def pick_spec(world, preferred: Optional[str] = None) -> TargetSpec:
    """Choose the first candidate whose asset the sim reports as available.

    A note on why the prop must be TALL: FrontCamera is mounted horizontal
    (rpy-deg "0 0 0") at 400x225 with a 90 deg horizontal FOV, so its vertical
    half-FOV is atan(tan(45deg) * 225/400) = 29.2 deg. A ground-level object only
    enters the front image beyond 1.79 x altitude. At a 22 m cruise that is 39 m,
    which is workable; at the old 45 m cruise it was 80 m, where the object is a
    handful of pixels. Height buys back the margin.
    """
    assets = discover_assets(world)
    have = {a.lower() for a in assets}
    if preferred:
        for c in CANDIDATES:
            if c.key == preferred or c.asset == preferred:
                return c
        return TargetSpec(key="custom", asset=preferred)
    for c in CANDIDATES:
        if c.source == "glb":
            return c                                # client-side file, always available
        if not have or c.asset.lower() in have:
            # `not have` = the server would not enumerate; try the best candidate
            # anyway and let spawn_target report the real failure.
            return c
    print(f"[target] none of {[c.asset for c in CANDIDATES]} in {len(assets)} assets; "
          f"falling back to {CANDIDATES[0].asset}")
    return CANDIDATES[0]


def spawn_target(world, spec: TargetSpec, x: float, y: float,
                 name: str = "SemTarget") -> Tuple[str, Tuple[float, float, float], dict]:
    """Place the target and read its pose back out of the sim.

    Returns (actual_name, (x, y, up), bbox). The pose returned is the one the sim
    reports, never the one we asked for — if the server clamps, snaps or renames
    anything, the scoring uses the truth rather than our intent.
    """
    from projectairsim.types import BoxAlignment, Pose, Quaternion, Vector3
    from projectairsim.utils import rpy_to_quaternion

    w, qx, qy, qz = rpy_to_quaternion(*spec.rpy)
    pose = Pose({
        "translation": Vector3({"x": float(x), "y": float(y), "z": float(spec.z_ned)}),
        "rotation": Quaternion({"w": w, "x": qx, "y": qy, "z": qz}),
        "frame_id": "DEFAULT_ID",
    })

    if spec.source == "glb":
        import os
        path = os.path.join(GLB_DIR, spec.asset)
        with open(path, "rb") as fh:
            blob = fh.read()
        actual = world.spawn_object_from_file(
            name, "gltf", blob, True, pose, spec.scale, False)
    else:
        actual = world.spawn_object(name, spec.asset, pose, spec.scale, False)
    print(f"[target] spawned {spec.asset!r} as {actual!r} at ({x:.1f}, {y:.1f})")

    if spec.material:
        try:
            world.set_object_material(actual, spec.material)
            print(f"[target] material -> {spec.material}")
        except Exception as e:
            # Not fatal: a wrong-coloured target still tests grounding, it just
            # weakens the colour ablation. V1's image check is what catches this.
            print(f"[target] set_object_material failed ({type(e).__name__}: {e}) "
                  f"— colour ablation will be weaker; check the probe image")

    truth = read_truth(world, actual)
    bbox = {}
    try:
        bbox = world.get_3d_bounding_box(actual, BoxAlignment.WORLD_AXIS)
    except Exception as e:
        print(f"[target] get_3d_bounding_box failed ({type(e).__name__}: {e})")
    print(f"[target] truth pose (N,E,up) = ({truth[0]:.2f}, {truth[1]:.2f}, {truth[2]:.2f})")
    return actual, truth, bbox


def read_truth(world, name: str) -> Tuple[float, float, float]:
    """Ground-truth (north, east, up) of a scene object, from the sim itself."""
    pose = world.get_object_pose(name)
    t = pose.translation
    # Pose.translation is a Vector3; tolerate both attribute and mapping styles
    # since the client wraps raw RPC dicts in places.
    try:
        px, py, pz = t.x, t.y, t.z
    except AttributeError:
        px, py, pz = t["x"], t["y"], t["z"]
    return float(px), float(py), float(-pz)


def destroy_target(world, name: str) -> None:
    try:
        world.destroy_object(name)
        print(f"[target] destroyed {name!r}")
    except Exception as e:
        print(f"[target] destroy failed ({type(e).__name__}: {e})")


def probe_view(get_front: Callable[[], object], out_png) -> bool:
    """Save one front-camera frame so a human can confirm the target is visible.

    This is the cheapest and most important check in the whole experiment: if the
    object is not plainly visible and plainly the stated colour in this image,
    every metric downstream is measuring nothing.
    """
    from pathlib import Path
    img = get_front()
    if img is None:
        print("[target] probe: no front frame yet")
        return False
    out_png = Path(out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_png)
    print(f"[target] probe image -> {out_png}   <-- OPEN THIS AND LOOK AT IT")
    return True


def bearing_to(x: float, y: float, target: Tuple[float, float]) -> float:
    """World bearing from (x, y) to the target, radians. Analysis-side only."""
    return math.atan2(target[1] - y, target[0] - x)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
