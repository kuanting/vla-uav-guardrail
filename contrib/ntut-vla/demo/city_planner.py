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


def inflate(occ, res, clearance_m):
    """Dilate occupied cells by ceil(clearance_m / res) using a square
    structuring element. Returns a uint8 NxN grid (1 = blocked)."""
    base = (np.asarray(occ) > 0).astype(np.uint8)
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


# ---------------------------------------------------------------------------
# global planner: 8-connected A* + line-of-sight simplification
# ---------------------------------------------------------------------------
_DIRS = [
    (-1, 0), (1, 0), (0, -1), (0, 1),
    (-1, -1), (-1, 1), (1, -1), (1, 1),
]


def plan(grid, res, ox, oy, start_xy, goal_xy):
    """8-connected grid A* on `grid` (0 = free, 1 = blocked).

    Start/goal are snapped to nearest_free first. The result is a
    line-of-sight-simplified list of world waypoints
    [start_world, ..., goal_world], or None if the goal is unreachable.
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

    path = plan(occ, res, ox, oy, start, goal)

    # (a) a path exists
    assert path is not None, "plan returned None"

    # (b) every consecutive segment is line-of-sight clear
    for a, b in zip(path[:-1], path[1:]):
        ca = _to_cell(res, ox, oy, a[0], a[1])
        cb = _to_cell(res, ox, oy, b[0], b[1])
        assert _los_clear(occ, ca, cb), f"segment not clear: {a} -> {b}"

    # (c) routes through the gap: some waypoint has j > 60  (i.e. y > 40)
    assert any(p[1] > 40.0 for p in path), "path does not route through the gap"

    print("waypoints:", [(round(p[0], 1), round(p[1], 1)) for p in path])
    print("PLANNER SELFTEST OK")
