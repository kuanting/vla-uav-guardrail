"""Fleet geometry and update budget — everything about demo/city_traffic.py that
can be checked without a simulator.

Run either way:
    pytest tests/test_city_traffic.py -v
    python tests/test_city_traffic.py

What these guard against is a fleet that LOOKS like traffic in the config but
degenerates in the scene: vehicles stacked on each other, every car in the same
place at t=0, a distractor wearing the target's paint, or an RPC bill that eats
the control loop. None of those crash anything. They just quietly turn the
discrimination experiment into a coin flip, which is the worst failure mode
because it still produces a number.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import city_traffic as ct                                        # noqa: E402
from moving_car import CarSpec                                   # noqa: E402


class _FakeWorld:
    """MovingCar only touches the world on spawn/update/destroy, and Traffic
    construction does none of those, so geometry is testable with a stub."""

    def spawn_object(self, *a, **k):
        raise AssertionError("no test here should spawn")

    def set_object_pose(self, *a, **k):
        raise AssertionError("no test here should teleport")


def _traffic(**kw):
    return ct.Traffic(_FakeWorld(), fleet=ct.default_fleet(**kw))


def _fake_models(tmp: Path) -> Path:
    """A model directory with the right FILENAMES and no real geometry.

    Everything about fleet identity - which vehicle is the target, what colour
    word each answers to, that no two look alike - is decided from the file
    names, so it is testable without shipping a 6 MB asset pack into the repo.
    What this cannot check is that the models render, which is why
    experiments/probe_glb_traffic_fitness.py exists and runs in-sim.
    """
    tmp.mkdir(parents=True, exist_ok=True)
    for fname, _ in [ct.GLB_TARGET] + list(ct.GLB_PALETTE):
        (tmp / fname).write_bytes(b"glTF-not-really")
    return tmp


def _glb_traffic(tmp: Path, **kw):
    kw.setdefault("n_background", 3)
    return ct.Traffic(_FakeWorld(),
                      fleet=ct.default_fleet(models=_fake_models(tmp), **kw))


# ------------------------------------------------------------------ identity

def test_exactly_one_vehicle_is_the_target():
    for n in (1, 2, 3, 4):
        t = _traffic(n_background=n)
        targets = [v for v in t.fleet if v.is_target]
        assert len(targets) == 1, f"{len(targets)} targets with {n} distractors"


def test_every_vehicle_LOOKS_different_in_demo_mode():
    """The point of this rebuild. With every car looking the same the noun cannot
    separate them and neither can the colour gate, so the tracker has nothing to
    hold and wanders between them -- which is exactly what it was doing.

    The invariant is APPEARANCE, not material path: M_Orange renders white on the
    sports car and genuinely orange on the offroad body, so the same material on
    two meshes is two different-looking vehicles."""
    t = _traffic(n_background=3, mode="demo")
    looks = [(v.asset, v.material) for v in t.fleet]
    assert len(set(looks)) == len(looks), f"two vehicles look identical: {looks}"


def test_the_target_is_the_only_one_answering_the_demo_phrase():
    t = _traffic(n_background=3)
    target = t.fleet[0]
    assert target.is_target and target.colour_word == ct.TARGET_COLOUR_WORD
    for vs in t.fleet[1:]:
        assert vs.colour_word != target.colour_word, (
            f"{vs.name} shares the target's colour word"
        )


def test_experiment_mode_holds_the_mesh_constant():
    """The controlled version. Mixing meshes makes the scene readable but
    confounds shape with colour - a correct lock could be the paint or could be
    the silhouette. Experiment mode gives that up on purpose."""
    assets = {vs.car_spec().asset
              for vs in _traffic(n_background=3, mode="experiment").fleet}
    assert assets == {ct.TARGET_ASSET}, f"experiment mode mixed meshes: {assets}"


def test_demo_mode_deliberately_does_not():
    """Stated as a test so the trade is visible rather than implied."""
    assets = {vs.car_spec().asset for vs in _traffic(n_background=3, mode="demo").fleet}
    assert len(assets) > 1, "demo mode gained nothing over experiment mode"


# ------------------------------------------------------------- glTF vehicles

def test_the_fleet_is_unchanged_when_no_models_are_supplied():
    """Discovery is the CALLER's job. A fleet that silently changes shape
    depending on what is installed on the machine is unreproducible, and a
    result measured on one machine would not be a result at all."""
    for vs in _traffic(n_background=3, mode="demo").fleet:
        assert vs.glb_path is None, f"{vs.name} picked up a model unasked"


def test_glb_fleet_gives_every_vehicle_its_own_colour(tmp=Path("_t_glb_colour")):
    """The reason for the whole exercise. The packaged path can bind exactly one
    material, so every vehicle was one of two appearances and the colour gate had
    nothing to separate - which is why the tracker wandered."""
    import shutil
    try:
        fleet = _glb_traffic(tmp).fleet
        assert all(v.glb_path for v in fleet), "not every vehicle is a model"
        files = [Path(v.glb_path).name for v in fleet]
        assert len(set(files)) == len(files), f"duplicate models: {files}"
        words = [v.colour_word for v in fleet if v.colour_word]
        assert len(set(words)) == len(words), f"duplicate colour words: {words}"
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_glb_target_owns_its_colour_word(tmp=Path("_t_glb_target")):
    import shutil
    try:
        fleet = _glb_traffic(tmp).fleet
        tgt = fleet[0]
        assert tgt.is_target and tgt.colour_word == ct.GLB_TARGET[1]
        for vs in fleet[1:]:
            assert vs.colour_word != tgt.colour_word, (
                f"{vs.name} answers to the target's colour word")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_glb_experiment_mode_refuses_the_models(tmp=Path("_t_glb_exp")):
    """Mixing five silhouettes confounds shape with colour. Experiment mode is
    the arm quoted when the question is whether the COLOUR WORD does the work,
    so it must keep one mesh even when better-looking models are available."""
    import shutil
    try:
        fleet = _glb_traffic(tmp, mode="experiment").fleet
        assert all(v.glb_path is None for v in fleet), "experiment mode took models"
        assert {v.car_spec().asset for v in fleet} == {ct.TARGET_ASSET}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_glb_spec_drops_the_material(tmp=Path("_t_glb_mat")):
    """A model carries its colour inside the file. Applying M_Orange on top
    would repaint it and undo the only thing this change buys."""
    import shutil
    try:
        for vs in _glb_traffic(tmp).fleet:
            sp = vs.car_spec()
            assert sp.glb_path, f"{vs.name} lost its model"
            assert sp.materials == [], f"{vs.name} would be repainted"
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_glb_fleet_falls_back_when_the_directory_is_empty(tmp=Path("_t_glb_none")):
    import shutil
    try:
        tmp.mkdir(parents=True, exist_ok=True)
        fleet = ct.default_fleet(n_background=3, models=tmp)
        assert all(v.glb_path is None for v in fleet), "invented models from nothing"
        assert fleet[0].colour_word == ct.TARGET_COLOUR_WORD
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_glb_fleet_obeys_the_same_motion_limits(tmp=Path("_t_glb_motion")):
    """The routes are shared with the mesh fleet, so this SHOULD hold by
    construction -- which is exactly why it is asserted. The rough-motion and
    driving-through-each-other faults both came from a fleet whose geometry was
    assumed rather than checked."""
    import shutil
    try:
        t = _glb_traffic(tmp)
        for vs, car in zip(t.fleet, t.cars):
            hs = [car.pose_at(i * 0.1)[2] for i in range(1400)]
            worst = max(abs(math.degrees(math.atan2(math.sin(hs[i + 1] - hs[i]),
                                                    math.cos(hs[i + 1] - hs[i]))))
                        for i in range(len(hs) - 1)) * 10.0
            assert worst <= 35.0, f"{vs.name} turns at {worst:.0f} deg/s"
            step = max(math.dist(car.pose_at(i * 0.1)[:2],
                                 car.pose_at((i + 1) * 0.1)[:2])
                       for i in range(600))
            assert step <= 0.35, f"{vs.name} jumps {step:.2f} m per tick"
        worst_sep = min(t.min_separation(i * 0.5) for i in range(280))
        assert worst_sep >= 3.0, f"vehicles closed to {worst_sep:.1f} m"
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_glb_yaw_offset_is_applied_so_cars_do_not_drive_sideways():
    """Measured at 270 deg: glTF is Y-up/-Z-forward, the sim is Z-up NED with
    heading 0 = North. Getting this wrong is not subtle - the whole fleet drives
    broadside - but it is silent, because the pose call still succeeds."""
    sp = CarSpec()
    assert sp.glb_yaw_offset_deg == 270.0
    sp.glb_path = "x.glb"
    plain = CarSpec()
    assert plain.glb_path is None and plain.materials, (
        "the packaged path must keep its material")


# ------------------------------------------------------------------- motion

def test_the_heading_never_turns_faster_than_a_car_can():
    """The complaint that started this rebuild. Per-vehicle 1 m loops with a
    0.6 m corner radius turned the heading at 109 deg/s; a real car manages
    about 30."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        hs = [car.pose_at(i * 0.1)[2] for i in range(1400)]
        worst = max(abs(math.degrees(math.atan2(math.sin(hs[i + 1] - hs[i]),
                                                math.cos(hs[i + 1] - hs[i]))))
                    for i in range(len(hs) - 1)) * 10.0
        assert worst <= 35.0, f"{vs.name} turns at {worst:.0f} deg/s"


def test_the_step_between_ticks_stays_small():
    """Teleport steps are what the eye reads as stutter."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        step = max(math.dist(car.pose_at(i * 0.1)[:2], car.pose_at((i + 1) * 0.1)[:2])
                   for i in range(600))
        assert step <= 0.35, f"{vs.name} jumps {step:.2f} m per tick"


def test_distractors_share_one_speed_so_they_cannot_collide():
    """They share a circuit. Unequal speeds mean a faster car catches a slower
    one and drives through it -- measured at 0.0 m separation before this."""
    speeds = {round(v.speed_mps, 4) for v in _traffic(n_background=3).fleet[1:]}
    assert len(speeds) == 1, f"distractors have differing speeds: {speeds}"


def test_no_two_vehicles_ever_meet():
    t = _traffic(n_background=3)
    worst, worst_t = float("inf"), None
    for i in range(1400):
        d = t.min_separation(i * 0.1)
        if d < worst:
            worst, worst_t = d, i * 0.1
    assert worst > 3.0, f"vehicles came within {worst:.1f} m at t={worst_t:.1f}s"


def test_phases_are_spread_around_the_lap():
    phases = [v.phase_frac for v in _traffic(n_background=3).fleet]
    assert len(set(phases)) == len(phases), f"repeated phases: {phases}"


def test_every_distractor_is_still_moving_late_in_a_long_flight():
    """A parked distractor stops being a distractor exactly when the selection
    problem should be hardest."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        if vs.is_target:
            continue                     # the target parks; that is the demo
        assert car.pose_at(110.0)[:2] != car.pose_at(100.0)[:2], (
            f"{vs.name} is stationary scenery by t=100 s"
        )


# ------------------------------------------------------------------ geometry

def test_no_distractor_crosses_the_stretch_the_target_drives():
    """A distractor sweeping through x=38 between y=-8 and y=58 occludes the
    very vehicle the aircraft is trying to select. The circuit's cross-legs are
    pushed 8 m beyond each end of the target's route for exactly this reason."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        if vs.is_target:
            continue
        for i in range(2000):
            x, y = car.pose_at(i * 0.1)[:2]
            if ct.ROUTE_Y0 <= y <= ct.ROUTE_Y1:
                assert abs(x - ct.TARGET_LANE_X) >= 3.0, (
                    f"{vs.name} reached ({x:.1f},{y:.1f}), {abs(x - 38):.1f} m "
                    "from the target lane on the stretch the target drives"
                )


def test_a_distractor_never_comes_close_to_the_target_itself():
    t = _traffic(n_background=3)
    tgt = t.cars[0]
    worst = min(math.dist(tgt.pose_at(i * 0.1)[:2], car.pose_at(i * 0.1)[:2])
                for i in range(1400) for car in t.cars[1:])
    assert worst > 3.0, f"a distractor passed {worst:.1f} m from the target"


def test_everything_stays_inside_the_clearance_envelope():
    """The obstacle map holds buildings only -- a flight at x=48 hit street
    furniture it cannot see."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        xs = [car.pose_at(i * 0.5)[0] for i in range(400)]
        assert 32.0 <= min(xs) and max(xs) <= 46.0, (
            f"{vs.name} spans x {min(xs):.1f}..{max(xs):.1f}, outside [32,46]"
        )


def test_the_target_drives_a_straight_run_and_distractors_a_closed_lap():
    fleet = _traffic(n_background=3).fleet
    assert len(fleet[0].route()) == 2, "the target should not loop; it parks"
    for vs in fleet[1:]:
        assert vs.route() == ct.circuit(), f"{vs.name} is not on the shared circuit"


# --------------------------------------------------------------- rpc budget

def test_the_target_updates_every_tick():
    assert _traffic(n_background=3).fleet[0].update_every == 1


def test_the_rpc_bill_stays_sane():
    t = _traffic(n_background=3)
    assert t.rpc_per_second(10.0) <= 45.0, t.rpc_per_second(10.0)


def test_a_skipped_update_costs_no_accuracy():
    """pose_at is a pure function of time, so staggering must not drift."""
    t = _traffic(n_background=3, bg_every=3)
    car = t.cars[1]
    stepped = [car.pose_at(i * 0.1)[:2] for i in range(300)]
    sparse = [car.pose_at(i * 0.1)[:2] for i in range(0, 300, 3)]
    for k, q in enumerate(sparse):
        assert stepped[k * 3] == q, "pose_at is not time-pure"


def test_update_stagger_skips_the_rpc_but_keeps_truth_current():
    class _CountingWorld(_FakeWorld):
        def __init__(self):
            self.n = 0

        def set_object_pose(self, *a, **k):
            self.n += 1

    w = _CountingWorld()
    t = ct.Traffic(w, fleet=ct.default_fleet(n_background=3, bg_every=3))
    for c in t.cars:
        c.actual_name = c.name
    for tick in range(30):
        t.update(tick * 0.1, tick)
    assert t.n_skipped > 0, "nothing was staggered"
    assert 30 <= w.n <= 30 + 3 * 10
    for vs, car in zip(t.fleet, t.cars):
        want = car.pose_at(29 * 0.1)[:2]
        assert math.dist(car.pos, want) < 1e-6, f"{vs.name} truth went stale"


def test_summary_records_the_colour_ground_truth():
    """Without this an experiment cannot be checked after the fact."""
    t = _traffic(n_background=3)
    out = t.summary()
    assert out["n_vehicles"] == 4
    for v in out["vehicles"]:
        for k in ("name", "is_target", "material", "colour_word", "speed_mps", "asset"):
            assert k in v, f"summary missing {k}"


# --------------------------------------------------------------------- runner


# ------------------------------------------- the turning route (aug 2026)

def _street():
    """The street mask, or None when it is not on this machine.

    NOT the occupancy map. The two answer different questions, and this test
    used to ask the wrong one - see the docstring below.
    """
    f = ROOT / "demo" / "out" / "citymap" / "street.npz"
    if not f.is_file():
        return None
    import sys as _sys
    _sys.path.insert(0, str(ROOT / "demo"))
    from build_street_mask import load_street
    return load_street(f)


def test_the_turn_route_stays_on_the_road():
    """The subject may drive a corner now, but not through a building.

    This checked the route against the OCCUPANCY MAP, and passed for as long as
    that map was built by collapsing voxels over 15-55 m AGL: nothing but
    buildings was in it, so every road was free by construction and free space
    was a fair proxy for road.

    Rebuilt over the band the aircraft actually flies in, that proxy is wrong in
    the direction this test could not tolerate. The route crosses a canopy at
    (38.0, 22.1) - a tree over the road - which is occupied at a 9 m cruise and
    entirely drivable underneath. The map was right and the question was wrong:
    a car is not constrained by what blocks a drone at altitude.

    So the route is now checked against demo/out/citymap/street.npz, which is
    built from what a car cares about: free of buildings and free of low
    clutter, ignoring anything overhead.
    """
    from moving_car import MovingCar, TURN_ROUTE
    import sys as _sys
    _sys.path.insert(0, str(ROOT / "demo"))
    from build_street_mask import is_street
    mask = _street()
    if mask is None:
        return                                  # mask not present; nothing to check

    car = MovingCar(_FakeWorld(), speed_mps=2.0, route=TURN_ROUTE, corner_r=6.0,
                    one_shot=True, phase_s=0.0, stops=[(0.30, 6.0), (0.62, 6.0)])
    pts = [car.pose_at(i * 0.1) for i in range(int(car.lap_time / 0.1))]
    bad = [(round(x, 1), round(y, 1)) for x, y, _ in pts if not is_street(mask, x, y)]
    assert not bad, f"turn route leaves the road at {bad[:5]}"


def test_the_corner_is_gentle_enough_to_follow():
    """A tight corner is what made the old traffic look wrong at 109 deg/s, and
    a fast heading swing is also what throws the target out of a 45 deg
    half-FOV. A real car manages about 30 deg/s."""
    from moving_car import MovingCar, TURN_ROUTE
    car = MovingCar(_FakeWorld(), speed_mps=2.0, route=TURN_ROUTE, corner_r=6.0,
                    one_shot=True, phase_s=0.0)
    hs = [car.pose_at(i * 0.1)[2] for i in range(int(car.lap_time / 0.1))]
    worst = max(abs(math.degrees(math.atan2(math.sin(hs[i + 1] - hs[i]),
                                            math.cos(hs[i + 1] - hs[i]))))
                for i in range(len(hs) - 1)) * 10.0
    assert worst <= 30.0, f"corner turns the heading at {worst:.0f} deg/s"


def test_the_turn_route_actually_turns():
    """Guards against the route silently degenerating back to a straight line."""
    from moving_car import MovingCar, TURN_ROUTE, STRAIGHT_ROUTE
    assert len(TURN_ROUTE) == 3 and TURN_ROUTE != STRAIGHT_ROUTE
    car = MovingCar(_FakeWorld(), speed_mps=2.0, route=TURN_ROUTE, corner_r=6.0,
                    one_shot=True, phase_s=0.0)
    h0 = car.pose_at(0.5)[2]
    h1 = car.pose_at(car.lap_time - 0.5)[2]
    swept = abs(math.degrees(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))))
    assert 70.0 < swept < 110.0, f"expected a ~90 deg turn, swept {swept:.0f}"


def test_the_turn_leg_is_long_enough_to_be_worth_filming():
    """The leg was extended south because the first version had the car
    parked for a third of the clip. 90 m keeps it moving for about a minute
    at a natural 2.5 m/s."""
    from moving_car import MovingCar, TURN_ROUTE
    car = MovingCar(_FakeWorld(), speed_mps=2.5, route=TURN_ROUTE,
                    corner_r=6.0, one_shot=True, phase_s=0.0,
                    stops=[(0.30, 8.0), (0.62, 8.0)])
    assert car.path.total > 85.0, f"route only {car.path.total:.0f} m"
    assert car.lap_time > 50.0, f"only {car.lap_time:.0f} s of motion"



def test_the_fleet_target_can_take_either_route():
    """--route must reach the traffic target too, or the traffic demo keeps
    driving straight while the single-car demo corners."""
    straight = ct.default_fleet(n_background=3, route_name="straight")[0].route()
    turn = ct.default_fleet(n_background=3, route_name="turn")[0].route()
    assert len(straight) == 2 and len(turn) == 3, (straight, turn)
    assert turn[0] == straight[0], "both routes must start in the same place"


def test_distractors_keep_the_circuit_whatever_the_target_does():
    for rn in ("straight", "turn"):
        fleet = ct.default_fleet(n_background=3, route_name=rn)
        for vs in fleet[1:]:
            assert vs.route() == ct.circuit(), f"{vs.name} left the circuit"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
