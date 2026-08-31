"""city_planner.py -- pure-python city occupancy map + global planner.

SHARED CONTRACT (no sim, no torch; numpy only).

Coordinates: x = North, y = East (meters). World spans [-80, 80] in both.
Cell mapping:
    i = round((x - origin_x) / res)   # i indexes North (x)
    j = round((y - origin_y) / res)   # j indexes East  (y)
    world of a cell: x = origin_x + i*res, y = origin_y + j*res
Grids are (N, N) uint8 with 1 = blocked (building), 0 = free.

This module is imported by both the flight demo and the GUI. Keep the public
interface stable:

    load_occ(path) -> dict | None
    inflate(occ, res, clearance_m) -> uint8 NxN
    is_blocked(grid, res, ox, oy, x, y) -> bool
    nearest_free(grid, res, ox, oy, x, y, max_r_m=30) -> (x, y)
    plan(grid, res, ox, oy, start_xy, goal_xy) -> list[(x, y)] | None
    smooth_path(grid, res, ox, oy, pts, iters=3) -> list[(x, y)]

Added 2026-08 (backwards compatible -- every old call site keeps working):

    distance_field(grid, res) -> float NxN     # metres to nearest blocked cell
    clearance_penalty(dist_m, clearance_pref_m=6.0) -> float array in [0, 1]
    plan_theta(grid, res, ox, oy, start_xy, goal_xy,
               clearance_pref_m=6.0, w_clear=1.0, prune_tol=0.03)
    plan_astar(...)                            # the original 8-connected A*
    plan(..., mode="theta"|"astar")            # dispatch; "theta" is the default
    inflate(occ, res, clearance_m, shape="square"|"euclid")

`plan()` now defaults to Theta* (any-angle A* with line-of-sight parent
shortcutting) on a clearance-shaped cost, so existing call sites get
corridor-centred routes with no edit. Pass mode="astar" for the old planner
verbatim (A/B), and see plan_theta's docstring for the tuning knobs.

Measured on demo/out/citymap/occ_day.npz, 90 random start/goal pairs, 5 m
square inflation -- Theta* vs the old A*: +1.2 % path length, +50 % waypoints,
+12 % min clearance, +17 % 5th-percentile clearance, +9 % mean clearance,
~1.5x plan time (26 ms vs 17 ms mean). The any-angle part alone is a WASH on
this map (the old post-hoc LOS simplification already produced the same
geometry); the clearance cost is what changes the routes.

Two other fixes shipped with it:
  * smooth_path() validated the wrong chord and could smooth the flown route
    straight through a building (12 of 60 routes on the real map). Fixed.
  * inflate(shape="euclid") removes the square kernel's ~40 % diagonal
    over-inflation, which is the real cause of "why did it go the long way".
    Opt-in, because it is genuinely less conservative than the default.
"""

import math
import heapq

import numpy as np

DEFAULT_OCC = (
    "D:/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/demo/out/citymap/occ.npz"
)

_INF = float("inf")


# ---------------------------------------------------------------------------
# map loading
# ---------------------------------------------------------------------------
def load_occ(path=DEFAULT_OCC):
    """Load the occupancy map .npz. Returns a dict, or None if the file is
    missing.

    dict keys: occ (uint8 NxN), height (float32 NxN), res (float),
               ox (float), oy (float), N (int)
    """
    try:
        data = np.load(path, allow_pickle=False)
    except FileNotFoundError:
        return None

    occ = np.asarray(data["occ"]).astype(np.uint8)
    # 'height' is optional — camera-survey maps carry it, the ground-truth voxel
    # maps (build_voxel_map.py) do not. It is not needed for planning.
    if "height" in data.files:
        height = np.asarray(data["height"]).astype(np.float32)
    else:
        height = np.zeros_like(occ, dtype=np.float32)
    res = float(data["res"])
    ox = float(data["origin_x"])
    oy = float(data["origin_y"])
    N = int(occ.shape[0])
    return {"occ": occ, "height": height, "res": res, "ox": ox, "oy": oy, "N": N}


# ---------------------------------------------------------------------------
# inflation (binary dilation by a square structuring element, pure numpy)
# ---------------------------------------------------------------------------
def _shift(a, di, dj):
    """Return `a` shifted so that out[i, j] = a[i - di, j - dj] (zero fill)."""
    out = np.zeros_like(a)
    N, M = a.shape
    # destination window
    di0, di1 = max(0, di), min(N, N + di)
    dj0, dj1 = max(0, dj), min(M, M + dj)
    # matching source window
    si0, si1 = max(0, -di), min(N, N - di)
    sj0, sj1 = max(0, -dj), min(M, M - dj)
    if di0 < di1 and dj0 < dj1:
        out[di0:di1, dj0:dj1] = a[si0:si1, sj0:sj1]
    return out


def inflate(occ, res, clearance_m, shape="square"):
    """Dilate occupied cells by clearance_m. Returns a uint8 NxN grid.

    shape="square" (DEFAULT, unchanged behaviour) -- dilate by
        ceil(clearance_m / res) cells with a square structuring element. This
        is conservative but ANISOTROPIC: the corner of the kernel reaches
        sqrt(2) * r cells, so a 5 m clearance on a 2 m grid actually inflates
        buildings by 6 m along the axes and 8.5 m diagonally. That extra
        diagonal fat closes diagonal gaps between buildings and is a real
        source of "why did it take the long way round" detours.

    shape="euclid" -- block exactly the cells whose centre is within
        clearance_m of an occupied cell centre, using distance_field(). This is
        what "clearance_m" literally means, it is isotropic, and it reopens the
        diagonal gaps. It is genuinely LESS conservative than "square", so it
        is opt-in: callers that rely on the old safety margin keep it by doing
        nothing.
    """
    base = (np.asarray(occ) > 0).astype(np.uint8)
    if float(clearance_m) <= 0.0:
        return base

    if shape == "euclid":
        d = distance_field(base, res)
        return (d <= float(clearance_m) + 1e-9).astype(np.uint8)
    if shape != "square":
        raise ValueError("inflate(): shape must be 'square' or 'euclid'")

    r = int(math.ceil(float(clearance_m) / float(res)))
    if r <= 0:
        return base
    out = base.copy()
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di == 0 and dj == 0:
                continue
            out = np.maximum(out, _shift(base, di, dj))
    return out.astype(np.uint8)


# ---------------------------------------------------------------------------
# cell <-> world helpers
# ---------------------------------------------------------------------------
def _to_cell(res, ox, oy, x, y):
    return int(round((x - ox) / res)), int(round((y - oy) / res))


def _to_world(res, ox, oy, i, j):
    return (ox + i * res, oy + j * res)


def is_blocked(grid, res, ox, oy, x, y):
    """True if world point (x, y) falls on a blocked cell. Points outside the
    grid are treated as FREE (open sky beyond the map)."""
    grid = np.asarray(grid)
    N, M = grid.shape
    i, j = _to_cell(res, ox, oy, x, y)
    if i < 0 or i >= N or j < 0 or j >= M:
        return False
    return bool(grid[i, j] == 1)


def nearest_free(grid, res, ox, oy, x, y, max_r_m=30):
    """Snap (x, y) out of a building to the nearest free world point.

    Returns the input unchanged if it is already free (or off-grid), or if no
    free cell is found within max_r_m meters."""
    grid = np.asarray(grid)
    N, M = grid.shape
    i0, j0 = _to_cell(res, ox, oy, x, y)

    # off-grid == free; already-free == return input unchanged
    if i0 < 0 or i0 >= N or j0 < 0 or j0 >= M:
        return (float(x), float(y))
    if grid[i0, j0] == 0:
        return (float(x), float(y))

    R = int(math.ceil(float(max_r_m) / float(res)))
    for r in range(1, R + 1):
        best = None
        best_d = None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:  # ring only
                    continue
                i, j = i0 + di, j0 + dj
                if 0 <= i < N and 0 <= j < M and grid[i, j] == 0:
                    wx, wy = _to_world(res, ox, oy, i, j)
                    d = (wx - x) ** 2 + (wy - y) ** 2
                    if best_d is None or d < best_d:
                        best_d = d
                        best = (wx, wy)
        if best is not None:
            return (float(best[0]), float(best[1]))
    return (float(x), float(y))


# ---------------------------------------------------------------------------
# line-of-sight (Bresenham) over cell space
# ---------------------------------------------------------------------------
def _blocked_cell(grid, i, j):
    N, M = grid.shape
    if i < 0 or i >= N or j < 0 or j >= M:
        return False  # off-grid == free
    return grid[i, j] == 1


def _line_cells(i0, j0, i1, j1):
    """SUPERCOVER line from (i0, j0) to (i1, j1), inclusive.

    A thin Bresenham walk skips the two 'shoulder' cells a diagonal step grazes
    past, so a straight segment that clips a building CORNER would test clear.
    This visits every cell the continuous segment touches (incl. both shoulders
    on each diagonal step) — conservative: it never lets a corner-cutting
    segment pass the line-of-sight test. Extra shoulder cells only ever make LOS
    stricter (keep an extra waypoint), never looser.
    """
    cells = []
    di = abs(i1 - i0)
    dj = abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1
    err = di - dj
    i, j = i0, j0
    cells.append((i, j))
    while i != i1 or j != j1:
        e2 = 2 * err
        stepped_i = stepped_j = False
        if e2 > -dj:
            err -= dj
            i += si
            stepped_i = True
        if e2 < di:
            err += di
            j += sj
            stepped_j = True
        if stepped_i and stepped_j:            # diagonal: grab both shoulders
            cells.append((i - si, j))
            cells.append((i, j - sj))
        cells.append((i, j))
    return cells


def _los_clear(grid, c0, c1):
    """True if the straight cell line c0->c1 crosses no blocked cell."""
    for (i, j) in _line_cells(c0[0], c0[1], c1[0], c1[1]):
        if _blocked_cell(grid, i, j):
            return False
    return True


def _world_clear(grid, res, ox, oy, a, b):
    """True if the straight WORLD segment a->b crosses no blocked cell."""
    ai, aj = _to_cell(res, ox, oy, a[0], a[1])
    bi, bj = _to_cell(res, ox, oy, b[0], b[1])
    return _los_clear(grid, (ai, aj), (bi, bj))


def _los_clear_fast(bl, N, M, i0, j0, i1, j1):
    """Flat-list twin of `_los_clear` -- SAME supercover walk, inlined bounds
    checks, no tuple allocation. `bl` is grid.reshape(-1).tolist() (truthy =
    blocked). Off-grid counts as FREE, exactly like `_blocked_cell`.

    This must stay bit-for-bit equivalent to `_los_clear(grid, c0, c1)`; the
    self-test at the bottom cross-checks the two on random cell pairs.
    """
    if 0 <= i0 < N and 0 <= j0 < M and bl[i0 * M + j0]:
        return False
    di = i1 - i0
    dj = j1 - j0
    si = 1 if di > 0 else -1
    sj = 1 if dj > 0 else -1
    di = di if di >= 0 else -di
    dj = dj if dj >= 0 else -dj
    err = di - dj
    i, j = i0, j0
    while i != i1 or j != j1:
        e2 = 2 * err
        stepped_i = stepped_j = False
        if e2 > -dj:
            err -= dj
            i += si
            stepped_i = True
        if e2 < di:
            err += di
            j += sj
            stepped_j = True
        if stepped_i and stepped_j:            # diagonal: both shoulders too
            a, b = i - si, j
            if 0 <= a < N and 0 <= b < M and bl[a * M + b]:
                return False
            a, b = i, j - sj
            if 0 <= a < N and 0 <= b < M and bl[a * M + b]:
                return False
        if 0 <= i < N and 0 <= j < M and bl[i * M + j]:
            return False
    return True


# ---------------------------------------------------------------------------
# obstacle distance field (two-pass chamfer, pure numpy -- no scipy)
# ---------------------------------------------------------------------------
_S2 = math.sqrt(2.0)
_S5 = math.sqrt(5.0)


def distance_field(grid, res=1.0):
    """Metres from every cell to the nearest BLOCKED cell centre.

    Two-pass chamfer distance transform over a 5x5 neighbourhood with true
    Euclidean step weights (1, sqrt2, sqrt5). Max relative error vs. the exact
    Euclidean transform is ~2 %, which is far below the resolution of a 2 m
    grid -- and it needs nothing but numpy.

    Pass 1 sweeps top-left -> bottom-right, pass 2 bottom-right -> top-left.
    Inside a pass the rows above/below are applied vectorised (they are already
    final), then the single in-row neighbour is propagated with a running min;
    that decomposition is exactly equivalent to the classic scalar raster scan,
    because the only causal in-row mask entries are (0,-1) w=1 and (0,-2) w=2,
    and the latter is dominated by two w=1 steps.

    Blocked cells get 0.0. Off-grid is treated as FREE (open sky beyond the
    map), i.e. no phantom seeds on the border -- consistent with is_blocked().
    An all-free grid returns a large finite constant everywhere.
    """
    g = np.asarray(grid)
    N, M = g.shape
    big = float(2 * (N + M))                   # cells; unreachably far
    d = np.where(g > 0, 0.0, big).astype(np.float64)
    if not (g > 0).any():
        return np.full((N, M), big * float(res))

    # ---- pass 1: forward (rows top->bottom, cols left->right) --------------
    for i in range(N):
        row = d[i]
        if i >= 1:
            up = d[i - 1]
            np.minimum(row, up + 1.0, out=row)
            np.minimum(row[1:], up[:-1] + _S2, out=row[1:])
            np.minimum(row[:-1], up[1:] + _S2, out=row[:-1])
            if M > 2:
                np.minimum(row[2:], up[:-2] + _S5, out=row[2:])
                np.minimum(row[:-2], up[2:] + _S5, out=row[:-2])
        if i >= 2:
            up2 = d[i - 2]
            np.minimum(row[1:], up2[:-1] + _S5, out=row[1:])
            np.minimum(row[:-1], up2[1:] + _S5, out=row[:-1])
        for j in range(1, M):                  # in-row running min
            v = row[j - 1] + 1.0
            if v < row[j]:
                row[j] = v

    # ---- pass 2: backward (rows bottom->top, cols right->left) -------------
    for i in range(N - 1, -1, -1):
        row = d[i]
        if i <= N - 2:
            dn = d[i + 1]
            np.minimum(row, dn + 1.0, out=row)
            np.minimum(row[1:], dn[:-1] + _S2, out=row[1:])
            np.minimum(row[:-1], dn[1:] + _S2, out=row[:-1])
            if M > 2:
                np.minimum(row[2:], dn[:-2] + _S5, out=row[2:])
                np.minimum(row[:-2], dn[2:] + _S5, out=row[:-2])
        if i <= N - 3:
            dn2 = d[i + 2]
            np.minimum(row[1:], dn2[:-1] + _S5, out=row[1:])
            np.minimum(row[:-1], dn2[1:] + _S5, out=row[:-1])
        for j in range(M - 2, -1, -1):
            v = row[j + 1] + 1.0
            if v < row[j]:
                row[j] = v

    return d * float(res)


def clearance_penalty(dist_m, clearance_pref_m=6.0):
    """Dimensionless soft cost in [0, 1] from an obstacle-distance field.

    0 at (and beyond) clearance_pref_m, rising quadratically to 1 on the
    obstacle itself. Same idea as nav2's costmap inflation layer / the Smac
    planner's cost penalty, just a quadratic instead of an exponential so it
    reaches exactly 0 at the preferred clearance (an exponential never does,
    which makes the cost never stop mattering in wide-open space)."""
    p = float(clearance_pref_m)
    if p <= 0.0:
        return np.zeros_like(np.asarray(dist_m, dtype=np.float64))
    t = 1.0 - np.asarray(dist_m, dtype=np.float64) / p
    np.clip(t, 0.0, 1.0, out=t)
    return t * t


# Planning grids are rebuilt on every replan but change rarely; the chamfer
# sweep is the one non-trivial per-plan cost, so memoise it on grid content.
_PEN_CACHE = {}
_PEN_CACHE_MAX = 8


def _penalty_flat(grid, res, clearance_pref_m):
    """Cached flat python list of clearance_penalty() values (fast scalar
    lookup inside the search loop)."""
    g = np.asarray(grid)
    key = (g.shape, float(res), float(clearance_pref_m), hash(g.tobytes()))
    hit = _PEN_CACHE.get(key)
    if hit is not None:
        return hit
    pen = clearance_penalty(distance_field(g, res), clearance_pref_m)
    flat = pen.reshape(-1).tolist()
    if len(_PEN_CACHE) >= _PEN_CACHE_MAX:
        _PEN_CACHE.clear()
    _PEN_CACHE[key] = flat
    return flat


def smooth_path(grid, res, ox, oy, pts, iters=3):
    """Chaikin corner-cutting on a world polyline, keeping clearance.

    A corner is only rounded off when the chord that replaces it is
    collision-free on `grid`; otherwise the original vertex is kept. Endpoints
    are preserved.

    BUGFIX 2026-08. The previous version emitted the two cut points of each
    EDGE (both at 25 %/75 % along the SAME segment) and validated the chord
    between them -- but that chord lies on the original edge and was already
    clear, so the test never rejected anything. The chord it never checked is
    the one that actually cuts a corner: 75 %-along-edge-k to 25 %-along-edge-
    (k+1), straight across the vertex between them. On the real city map that
    let the smoothed route clip real buildings on 12 of 60 planned routes.

    Cutting is now expressed per INTERIOR VERTEX, which is the standard Chaikin
    formulation and makes the corner chord the thing being validated:

        p0 -> r1 q1 -> r2 q2 -> ... -> pn      r_k = 75 % along (p_{k-1}, p_k)
                                               q_k = 25 % along (p_k, p_{k+1})

    every other output segment is a sub-segment of an original edge, so if the
    input polyline was clear and each accepted r_k -> q_k chord is clear, the
    whole smoothed polyline is clear.
    """
    pts = [(float(x), float(y)) for x, y in pts]
    for _ in range(max(0, iters)):
        if len(pts) < 3:
            break
        out = [pts[0]]
        for k in range(1, len(pts) - 1):
            a, b, c = pts[k - 1], pts[k], pts[k + 1]
            # Try the full Chaikin cut first, then progressively shallower ones.
            # A fixed 25 % cut on a 30 m leg shaves 7.5 m off the corner, which
            # in a tight city always clips the inflated obstacle and gets
            # rejected -- so a fixed cut silently degrades to no smoothing at
            # all. Backing off lets every corner round by as much as its
            # clearance actually allows. f <= 0.25 on both sides keeps the two
            # cut points from crossing over each other.
            done = False
            for f in (0.25, 0.125, 0.0625):
                r = (b[0] + (a[0] - b[0]) * f, b[1] + (a[1] - b[1]) * f)
                q = (b[0] + (c[0] - b[0]) * f, b[1] + (c[1] - b[1]) * f)
                if (not _blocked_cell(grid, *_to_cell(res, ox, oy, *r))
                        and not _blocked_cell(grid, *_to_cell(res, ox, oy, *q))
                        and _world_clear(grid, res, ox, oy, r, q)):
                    out.extend([r, q])
                    done = True
                    break
            if not done:
                out.append(b)          # keep the original vertex (stay clear)
        out.append(pts[-1])
        # drop consecutive duplicates
        dedup = [out[0]]
        for p in out[1:]:
            if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 0.3:
                dedup.append(p)
        pts = dedup
    return pts


# ---------------------------------------------------------------------------
# global planner: 8-connected A* + line-of-sight simplification
# ---------------------------------------------------------------------------
_DIRS = [
    (-1, 0), (1, 0), (0, -1), (0, 1),
    (-1, -1), (-1, 1), (1, -1), (1, 1),
]


def plan_astar(grid, res, ox, oy, start_xy, goal_xy):
    """8-connected grid A* on `grid` (0 = free, 1 = blocked).

    Start/goal are snapped to nearest_free first. The result is a
    line-of-sight-simplified list of world waypoints
    [start_world, ..., goal_world], or None if the goal is unreachable.

    This is the ORIGINAL planner, kept verbatim as the A/B baseline. New code
    should call plan() (which defaults to Theta*).
    """
    grid = np.asarray(grid)
    N, M = grid.shape

    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])

    # all-free short-circuit
    if not grid.any():
        return [(sx, sy), (gx, gy)]

    # snap endpoints out of any building
    sx, sy = nearest_free(grid, res, ox, oy, sx, sy)
    gx, gy = nearest_free(grid, res, ox, oy, gx, gy)

    si, sj = _to_cell(res, ox, oy, sx, sy)
    gi, gj = _to_cell(res, ox, oy, gx, gy)
    # clamp into the grid (endpoints just off the edge round to the border)
    si = min(max(si, 0), N - 1)
    sj = min(max(sj, 0), M - 1)
    gi = min(max(gi, 0), N - 1)
    gj = min(max(gj, 0), M - 1)

    start = (si, sj)
    goal = (gi, gj)

    def heur(c):
        return math.hypot(c[0] - gi, c[1] - gj)

    open_heap = [(heur(start), 0.0, start)]
    gscore = {start: 0.0}
    came = {}
    found = False

    while open_heap:
        _, gc, cur = heapq.heappop(open_heap)
        if cur == goal:
            found = True
            break
        if gc > gscore.get(cur, _INF):
            continue
        ci, cj = cur
        for di, dj in _DIRS:
            ni, nj = ci + di, cj + dj
            if ni < 0 or ni >= N or nj < 0 or nj >= M:
                continue
            if grid[ni, nj] == 1:
                continue
            # no corner-cutting: diagonal needs both orthogonal cells free
            if di != 0 and dj != 0:
                if grid[ci + di, cj] == 1 or grid[ci, cj + dj] == 1:
                    continue
            ng = gc + math.hypot(di, dj)
            nb = (ni, nj)
            if ng < gscore.get(nb, _INF):
                gscore[nb] = ng
                came[nb] = cur
                heapq.heappush(open_heap, (ng + heur(nb), ng, nb))

    if not found:
        return None

    # reconstruct cell path start -> goal
    path = [goal]
    c = goal
    while c != start:
        c = came[c]
        path.append(c)
    path.reverse()

    # line-of-sight simplification: keep a waypoint only when the straight line
    # to the next-kept point would cross a blocked cell
    simp = [path[0]]
    anchor = 0
    for k in range(1, len(path) - 1):
        if not _los_clear(grid, path[anchor], path[k + 1]):
            simp.append(path[k])
            anchor = k
    if len(path) > 1:
        simp.append(path[-1])

    return [(_to_world(res, ox, oy, i, j)) for (i, j) in simp]


# ---------------------------------------------------------------------------
# global planner v2: Theta* (any-angle) on a clearance-shaped cost
# ---------------------------------------------------------------------------
def _snap_endpoints(grid, res, ox, oy, start_xy, goal_xy):
    """Shared with plan_astar: snap out of buildings, clamp into the grid."""
    N, M = grid.shape
    sx, sy = nearest_free(grid, res, ox, oy,
                          float(start_xy[0]), float(start_xy[1]))
    gx, gy = nearest_free(grid, res, ox, oy,
                          float(goal_xy[0]), float(goal_xy[1]))
    si, sj = _to_cell(res, ox, oy, sx, sy)
    gi, gj = _to_cell(res, ox, oy, gx, gy)
    si = min(max(si, 0), N - 1)
    sj = min(max(sj, 0), M - 1)
    gi = min(max(gi, 0), N - 1)
    gj = min(max(gj, 0), M - 1)
    return (si, sj), (gi, gj)


def plan_theta(grid, res, ox, oy, start_xy, goal_xy,
               clearance_pref_m=6.0, w_clear=1.0, prune_tol=0.03,
               cost_samples=32):
    """Theta* (any-angle A*) with an obstacle-distance cost term.

    Two changes vs. plan_astar():

    1. ANY-ANGLE. Every time a node is relaxed we test line-of-sight from the
       *grandparent* to the successor; if it is clear the successor inherits the
       grandparent as its parent. Headings are therefore continuous instead of
       being quantised to the 8 grid directions, and the shortcut is found
       DURING the search (so the search actually optimises the true straight
       line cost) rather than being bolted on afterwards like the old
       post-hoc LOS simplification.

    2. CORRIDOR CENTRING. Step cost is

           cost(a -> b) = |ab|_m * (1 + w_clear * mean_pen(a..b))

       where pen = clearance_penalty(distance_field(grid)) is 0 at/beyond
       clearance_pref_m metres of obstacle distance and rises to 1 on the
       obstacle. Without this, any-angle planning makes corner-scraping WORSE
       (the shortest line is always the one that grazes the corner). Scaling by
       segment length is essential here: Theta* edges have wildly different
       lengths, so a flat additive penalty would let a long shortcut through a
       tight gap pay the same as a 2 m step.

       Because cost >= euclidean length, the euclidean heuristic (in metres)
       stays admissible.

    The parent-shortcut test uses `_los_clear_fast`, the exact same SUPERCOVER
    walk as plan()'s simplification (both shoulders of every diagonal step are
    tested), so a segment can never squeeze through a building corner.

    Tuning:
        clearance_pref_m -- how far off obstacles we would LIKE to fly. Beyond
                            this the penalty is exactly 0, so open-space paths
                            are pure shortest-path. Set 0 to disable.
        w_clear          -- how much that preference is worth. 0 = plain
                            any-angle Theta*; 1.0 = flying flush against a
                            building costs 2x its length; 3+ makes the drone
                            take visible detours to stay mid-corridor.
        prune_tol        -- waypoint-count knob. The final prune may spend up to
                            prune_tol * (total path cost) buying FEWER
                            waypoints; each removal shortens the path (triangle
                            inequality) and pays for itself out of that budget
                            in lost clearance. 0 = keep every vertex the search
                            produced (chattiest, best centred).
        cost_samples     -- CAP on the samples used to integrate the penalty
                            along one edge. Sampling is one-per-cell up to this
                            cap, so short edges are exact and only very long
                            shortcuts get coarser. Too low and a long shortcut
                            averages a corner-scrape away to nothing.

    Returns world waypoints [start, ..., goal] like plan(), or None.
    """
    grid = np.asarray(grid)
    N, M = grid.shape

    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    if not grid.any():                              # all-free short-circuit
        return [(sx, sy), (gx, gy)]

    (si, sj), (gi, gj) = _snap_endpoints(grid, res, ox, oy,
                                         (sx, sy), (gx, gy))
    start = si * M + sj
    goal = gi * M + gj
    if start == goal:
        return [_to_world(res, ox, oy, si, sj)]

    bl = (grid == 1).reshape(-1).tolist()
    pen = _penalty_flat(grid, res, clearance_pref_m) if (
        w_clear > 0.0 and clearance_pref_m > 0.0) else None
    res_f = float(res)
    wc = float(w_clear)
    ns = max(2, int(cost_samples))

    def edge_cost(ai, aj, bi, bj):
        """Length in metres, inflated by the mean clearance penalty along it."""
        dii = bi - ai
        djj = bj - aj
        length = math.hypot(dii, djj) * res_f
        if pen is None or length <= 0.0:
            return length
        n = int(math.hypot(dii, djj)) + 1
        if n > ns:
            n = ns
        elif n < 2:
            n = 2
        acc = 0.0
        inv = 1.0 / (n - 1)
        for k in range(n):
            t = k * inv
            # both endpoints are in-grid, so the interpolant is >= 0 and
            # int(v + 0.5) is a plain round-half-up.
            ii = int(ai + dii * t + 0.5)
            jj = int(aj + djj * t + 0.5)
            if 0 <= ii < N and 0 <= jj < M:
                acc += pen[ii * M + jj]
            # off-grid == free == no penalty
        return length * (1.0 + wc * acc / n)

    # ---- Theta* ------------------------------------------------------------
    gscore = {start: 0.0}
    parent = {start: start}
    closed = set()
    h0 = math.hypot(si - gi, sj - gj) * res_f
    open_heap = [(h0, start)]
    found = False

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            found = True
            break
        closed.add(cur)

        ci, cj = divmod(cur, M)
        par = parent[cur]
        pi, pj = divmod(par, M)
        g_cur = gscore[cur]
        g_par = gscore[par]
        # path-1 edges are always a single grid step, so edge_cost() would
        # sample exactly its two endpoints -- inline that (it is ~half of all
        # cost evaluations and the hottest line in the search).
        pen_cur = pen[cur] if pen is not None else 0.0

        for di, dj in _DIRS:
            ni = ci + di
            nj = cj + dj
            if ni < 0 or ni >= N or nj < 0 or nj >= M:
                continue
            nb = ni * M + nj
            if bl[nb] or nb in closed:
                continue
            if di and dj:                       # no corner-cutting, as before
                if bl[ni * M + cj] or bl[ci * M + nj]:
                    continue

            # path 1: through the current node (always legal)
            step = (_S2 * res_f) if (di and dj) else res_f
            if pen is None:
                best = g_cur + step
            else:
                best = g_cur + step * (1.0 + wc * 0.5 * (pen_cur + pen[nb]))
            best_par = cur
            # path 2: straight from the grandparent, if visible. Ties go to
            # path 2 -> fewer, longer, straighter legs.
            if par != cur and _los_clear_fast(bl, N, M, pi, pj, ni, nj):
                alt = g_par + edge_cost(pi, pj, ni, nj)
                if alt <= best:
                    best = alt
                    best_par = par

            if best < gscore.get(nb, _INF):
                gscore[nb] = best
                parent[nb] = best_par
                heapq.heappush(
                    open_heap,
                    (best + math.hypot(ni - gi, nj - gj) * res_f, nb))

    if not found:
        return None

    # ---- reconstruct (parents are already any-angle) -----------------------
    cells = [goal]
    c = goal
    while c != start:
        c = parent[c]
        cells.append(c)
    cells.reverse()
    pts = [divmod(c, M) for c in cells]

    # ---- prune vertices that cost (almost) nothing to skip -----------------
    # NB: this is a COST-AWARE prune, not the greedy LOS shortcutting the old
    # planner used. A vertex is dropped only if the shortcut is line-of-sight
    # clear AND the clearance cost it adds fits in a global budget of
    # prune_tol * (total path cost). Removing a vertex always SHORTENS the path
    # (triangle inequality), so the budget is exactly "how much centring am I
    # willing to sell for a smoother, shorter, lower-waypoint-count route".
    # A plain `direct <= via` test keeps every point of a corridor-hugging arc
    # and doubles the waypoint count for ~0 benefit.
    budget = 0.0
    if prune_tol > 0.0 and len(pts) > 2:
        total = sum(edge_cost(a[0], a[1], b[0], b[1])
                    for a, b in zip(pts[:-1], pts[1:]))
        budget = float(prune_tol) * total
    changed = True
    while changed and len(pts) > 2:
        changed = False
        k = 1
        while k < len(pts) - 1:
            a, b, cpt = pts[k - 1], pts[k], pts[k + 1]
            if _los_clear_fast(bl, N, M, a[0], a[1], cpt[0], cpt[1]):
                direct = edge_cost(a[0], a[1], cpt[0], cpt[1])
                via = (edge_cost(a[0], a[1], b[0], b[1])
                       + edge_cost(b[0], b[1], cpt[0], cpt[1]))
                extra = direct - via
                if extra <= 1e-9:
                    del pts[k]
                    changed = True
                    continue
                # LOCAL test (extra is small next to the two legs it replaces)
                # AND a global budget, so one long path cannot spend its whole
                # allowance buying a single corner scrape.
                if extra <= prune_tol * via and extra <= budget:
                    budget -= extra
                    del pts[k]
                    changed = True
                    continue
            k += 1

    return [_to_world(res, ox, oy, i, j) for (i, j) in pts]


def plan(grid, res, ox, oy, start_xy, goal_xy, mode="theta",
         clearance_pref_m=6.0, w_clear=1.0, prune_tol=0.03):
    """Global 2-D planner. Returns world waypoints [start, ..., goal] or None.

    mode="theta" (default) -- any-angle Theta* + obstacle-distance cost.
    mode="astar"           -- the original 8-connected A* + LOS simplification.

    The signature is a superset of the old one, so every existing call site
    (`plan(grid, res, ox, oy, a, b)`) keeps working and silently gains the
    upgrade; pass mode="astar" to A/B against the previous behaviour.
    """
    if mode == "astar":
        return plan_astar(grid, res, ox, oy, start_xy, goal_xy)
    if mode != "theta":
        raise ValueError("plan(): mode must be 'theta' or 'astar', got %r"
                         % (mode,))
    return plan_theta(grid, res, ox, oy, start_xy, goal_xy,
                      clearance_pref_m=clearance_pref_m, w_clear=w_clear,
                      prune_tol=prune_tol)


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    N = 80
    res = 2.0
    ox = oy = -80.0

    # synthetic city: vertical wall at i in [30..34] for j in [0..60],
    # leaving a gap at j > 60.
    occ = np.zeros((N, N), dtype=np.uint8)
    occ[30:35, 0:61] = 1

    start = (-70.0, -70.0)   # cell (5, 5)   -- below-left of the wall
    goal = (40.0, -70.0)     # cell (60, 5)  -- far side of the wall

    # ---- shared helpers ----------------------------------------------------
    def path_len(p):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(p[:-1], p[1:]))

    def dense_samples(p, step=0.25):
        """Every sample point along the polyline, spaced <= `step` metres."""
        out = []
        for a, b in zip(p[:-1], p[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(2, int(L / step) + 1)
            for k in range(n):
                t = k / (n - 1)
                out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        return out

    df_selftest = distance_field(occ, res)

    def _clear_vals(p):
        vals = []
        for (x, y) in dense_samples(p):
            i, j = _to_cell(res, ox, oy, x, y)
            if 0 <= i < N and 0 <= j < N:
                vals.append(float(df_selftest[i, j]))
        return vals

    def min_clear(p):
        return min(_clear_vals(p))

    def mean_clear(p):
        v = _clear_vals(p)
        return sum(v) / len(v)

    def p5_clear(p):
        return float(np.percentile(_clear_vals(p), 5))

    def shaped_clear(p, pref=6.0):
        """Mean of min(clearance, pref) -- ignores open sky (where the planner
        is indifferent) and only scores the stretches the cost term acts on."""
        v = _clear_vals(p)
        return sum(min(c, pref) for c in v) / len(v)

    # ======================================================================
    # CASE 1 -- original A* case (mode="astar"): unchanged behaviour
    # ======================================================================
    path = plan(occ, res, ox, oy, start, goal, mode="astar")

    # (a) a path exists
    assert path is not None, "plan returned None"

    # (b) every consecutive segment is line-of-sight clear
    for a, b in zip(path[:-1], path[1:]):
        ca = _to_cell(res, ox, oy, a[0], a[1])
        cb = _to_cell(res, ox, oy, b[0], b[1])
        assert _los_clear(occ, ca, cb), f"segment not clear: {a} -> {b}"

    # (c) routes through the gap: some waypoint has j > 60  (i.e. y > 40)
    assert any(p[1] > 40.0 for p in path), "path does not route through the gap"

    print("astar waypoints:", [(round(p[0], 1), round(p[1], 1)) for p in path])

    # ======================================================================
    # CASE 2 -- distance_field vs. brute force (small random grid)
    # ======================================================================
    rng = np.random.default_rng(7)
    small = (rng.random((24, 24)) < 0.08).astype(np.uint8)
    df = distance_field(small, 1.0)
    bi, bj = np.nonzero(small)
    ii, jj = np.mgrid[0:24, 0:24]
    exact = np.sqrt(np.min((ii[..., None] - bi) ** 2
                           + (jj[..., None] - bj) ** 2, axis=-1)).astype(float)
    err = np.max(np.abs(df - exact) / np.maximum(exact, 1.0))
    assert err < 0.05, f"chamfer distance error too large: {err:.3f}"
    assert (df[small == 1] == 0).all(), "blocked cells must have distance 0"
    print(f"distance_field max rel. error vs exact EDT: {err * 100:.2f} %")

    # ======================================================================
    # CASE 3 -- _los_clear_fast is identical to _los_clear (never weaker)
    # ======================================================================
    bl_s = (small == 1).reshape(-1).tolist()
    for _ in range(4000):
        a = (int(rng.integers(0, 24)), int(rng.integers(0, 24)))
        b = (int(rng.integers(0, 24)), int(rng.integers(0, 24)))
        ref = _los_clear(small, a, b)
        fast = _los_clear_fast(bl_s, 24, 24, a[0], a[1], b[0], b[1])
        assert ref == fast, f"LOS mismatch {a}->{b}: ref={ref} fast={fast}"
    print("supercover LOS: fast == reference on 4000 random pairs")

    # ======================================================================
    # CASE 4 -- Theta* (the default mode)
    # ======================================================================
    tpath = plan(occ, res, ox, oy, start, goal)          # mode="theta"
    assert tpath is not None, "plan_theta returned None"

    # (a) segments are supercover-LOS clear under the ORIGINAL checker
    for a, b in zip(tpath[:-1], tpath[1:]):
        ca = _to_cell(res, ox, oy, a[0], a[1])
        cb = _to_cell(res, ox, oy, b[0], b[1])
        assert _los_clear(occ, ca, cb), f"theta segment not clear: {a} -> {b}"

    # (b) densely sampled, no point ever lands in a blocked cell
    for (x, y) in dense_samples(tpath):
        assert not is_blocked(occ, res, ox, oy, x, y), \
            f"theta path enters a blocked cell at ({x:.2f}, {y:.2f})"

    # (c) still routes through the gap
    assert any(p[1] > 40.0 for p in tpath), "theta path misses the gap"

    # (d) endpoints preserved
    assert math.hypot(tpath[0][0] - start[0], tpath[0][1] - start[1]) < res
    assert math.hypot(tpath[-1][0] - goal[0], tpath[-1][1] - goal[1]) < res

    # (e) clearance improves where it can. The ABSOLUTE min on this map is
    #     pinched at the wall tip (2 m; no route avoids it) and the plain mean
    #     is drowned by open sky, so score the two metrics that reflect the
    #     objective: the 5th-percentile clearance and the mean of
    #     min(clearance, clearance_pref).
    assert p5_clear(tpath) > p5_clear(path) + 1e-6, \
        (f"theta p5 clearance {p5_clear(tpath):.2f} m did not beat astar "
         f"{p5_clear(path):.2f} m")
    assert shaped_clear(tpath) > shaped_clear(path) + 1e-9, \
        (f"theta shaped clearance {shaped_clear(tpath):.3f} did not beat astar "
         f"{shaped_clear(path):.3f}")

    # (f) w_clear=0 is plain any-angle Theta*: never longer than the shaped
    #     path, and never longer than 8-connected A*.
    raw = plan_theta(occ, res, ox, oy, start, goal, w_clear=0.0)
    assert raw is not None
    assert path_len(raw) <= path_len(tpath) + 1e-6, \
        "w_clear=0 should be no longer than the clearance-shaped path"
    assert path_len(raw) <= path_len(path) + 1e-6, \
        "plain Theta* should be no longer than 8-connected A*"

    print("theta waypoints:", [(round(p[0], 1), round(p[1], 1)) for p in tpath])
    for tag, p in (("astar     ", path), ("theta w=0 ", raw),
                   ("theta dflt", tpath)):
        print(f"  {tag}: {len(p):2d} wp, {path_len(p):6.1f} m, clearance "
              f"min {min_clear(p):5.2f} / p5 {p5_clear(p):5.2f} / "
              f"mean {mean_clear(p):5.2f} m  (shaped {shaped_clear(p):.3f})")

    # ======================================================================
    # CASE 5 -- corridor centring: one block in open space, endpoints placed
    #           so the shortest route scrapes its corner.
    # ======================================================================
    blk = np.zeros((N, N), dtype=np.uint8)
    blk[34:46, 34:46] = 1
    bs = _to_world(res, ox, oy, 40, 6)
    bg = _to_world(res, ox, oy, 40, 73)
    df_blk = distance_field(blk, res)

    def bclear(p, reducer):
        vals = []
        for (x, y) in dense_samples(p):
            i, j = _to_cell(res, ox, oy, x, y)
            if 0 <= i < N and 0 <= j < N:
                vals.append(float(df_blk[i, j]))
        return reducer(vals)

    ba = plan(blk, res, ox, oy, bs, bg, mode="astar")
    bt = plan(blk, res, ox, oy, bs, bg)
    b0 = plan_theta(blk, res, ox, oy, bs, bg, w_clear=0.0)

    for p, tag in ((bt, "theta"), (b0, "theta w=0")):
        for (x, y) in dense_samples(p):
            assert not is_blocked(blk, res, ox, oy, x, y), \
                f"{tag} path enters the block at ({x:.2f}, {y:.2f})"

    # any-angle alone must not lengthen the route ...
    assert path_len(b0) <= path_len(ba) + 1e-6, \
        "plain Theta* is longer than A* on the single-block map"
    # ... and the clearance term must actually pull it off the corner.
    assert bclear(bt, min) > bclear(ba, min) + 1e-6, \
        (f"theta min clearance {bclear(bt, min):.2f} m did not beat astar "
         f"{bclear(ba, min):.2f} m on the single-block map")

    print("single-block map:")
    print(f"  astar     : {len(ba):2d} wp, {path_len(ba):6.1f} m, "
          f"min clearance {bclear(ba, min):5.2f} m")
    print(f"  theta w=0 : {len(b0):2d} wp, {path_len(b0):6.1f} m, "
          f"min clearance {bclear(b0, min):5.2f} m")
    print(f"  theta dflt: {len(bt):2d} wp, {path_len(bt):6.1f} m, "
          f"min clearance {bclear(bt, min):5.2f} m")

    # ======================================================================
    # CASE 6 -- smooth_path must not smooth its way into an obstacle
    # ======================================================================
    #  A hard L around the block: the corner chord of a naive Chaikin cut goes
    #  straight through it.
    corner = [_to_world(res, ox, oy, 30, 30),
              _to_world(res, ox, oy, 30, 50),
              _to_world(res, ox, oy, 50, 50)]
    sm = smooth_path(blk, res, ox, oy, corner, iters=3)
    assert math.hypot(sm[0][0] - corner[0][0], sm[0][1] - corner[0][1]) < 1e-9
    assert math.hypot(sm[-1][0] - corner[-1][0],
                      sm[-1][1] - corner[-1][1]) < 1e-9
    for (x, y) in dense_samples(sm):
        assert not is_blocked(blk, res, ox, oy, x, y), \
            f"smooth_path cut a corner into the block at ({x:.2f}, {y:.2f})"
    # and it must still smooth when there IS room (open space -> corner rounds)
    free_corner = [(-60.0, -60.0), (-60.0, 0.0), (0.0, 0.0)]
    sm2 = smooth_path(np.zeros((N, N), dtype=np.uint8), res, ox, oy,
                      free_corner, iters=2)
    assert len(sm2) > len(free_corner), "smooth_path did nothing in open space"
    print(f"smooth_path: {len(corner)} -> {len(sm)} pts around the block "
          f"(no incursion), {len(free_corner)} -> {len(sm2)} pts in open space")

    # ======================================================================
    # CASE 7 -- API contract: unreachable, degenerate, and inflate() shapes
    # ======================================================================
    walled = np.zeros((N, N), dtype=np.uint8)
    walled[30:35, :] = 1
    assert plan(walled, res, ox, oy, start, goal) is None, \
        "fully walled-off goal must return None"
    assert plan(walled, res, ox, oy, start, goal, mode="astar") is None

    # all-free grid: straight through, both modes
    empty = np.zeros((N, N), dtype=np.uint8)
    assert len(plan(empty, res, ox, oy, start, goal)) == 2
    # start == goal
    assert len(plan(occ, res, ox, oy, start, start)) == 1

    try:
        plan(occ, res, ox, oy, start, goal, mode="bogus")
    except ValueError:
        pass
    else:
        raise AssertionError("plan() accepted an unknown mode")

    sq = inflate(blk, res, 5.0)
    eu = inflate(blk, res, 5.0, shape="euclid")
    assert (eu <= sq).all(), "euclid inflation must be a subset of square"
    assert int(eu.sum()) < int(sq.sum()), \
        "euclid inflation should drop the diagonal over-inflation"
    assert (inflate(blk, res, 0.0) == blk).all()
    print(f"inflate 5 m on the block map: square {int(sq.sum())} cells, "
          f"euclid {int(eu.sum())} cells "
          f"(-{100 * (1 - eu.sum() / sq.sum()):.0f} % over-inflation)")

    print("PLANNER SELFTEST OK")
