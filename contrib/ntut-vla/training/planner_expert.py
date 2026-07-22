"""
Planner expert — visibility-graph + Dijkstra teacher (campaign 3).

Why: the potential-field expert capped at 96.8% reach on the benchmark (local
minima in 2-zone mazes), and a student cannot exceed its teacher. This planner
is COMPLETE for our axis-aligned rectangular zones: if a path exists through
the inflated corner graph, Dijkstra finds it. Deterministic by construction.

Plan nodes: start, target, and the corners of every zone polygon inflated by
(margin + extra). Edge = segment that doesn't cut through any inflated zone.
The plan is cached and only recomputed when zones change (hot-apply) or every
REPLAN_TICKS as drift insurance. Per-tick output is a carrot-follower velocity,
same action interface as the old expert; the Shield still filters every label.
"""
from __future__ import annotations

import heapq
import math

from shapely.geometry import LineString, Point

from guardrail.models import Action4D, State

INFLATE_EXTRA = 1.5      # beyond the fence margin, path-planning clearance
REPLAN_TICKS = 15
CARROT_M = 4.0           # follow a point this far along the path
SPEED = 4.0


class PlannerExpert:
    def __init__(self, fences):
        self.refresh(fences)

    def refresh(self, fences) -> None:
        """(Re)build inflated obstacle sets + clear the cached plan.
        Three inflation levels: plan with the widest that still admits a path
        (progressive relaxation — 11/400 eval scenarios had a path at ~3 m
        clearance but not at 4 m; the shield still guards the actual margin)."""
        self._levels = []
        for extra in (INFLATE_EXTRA, 0.75, 0.25):
            polys = [poly.buffer(f.margin_m + extra, join_style=2)
                     for f, poly in fences]
            test = [p.buffer(-0.05) for p in polys]
            self._levels.append((polys, test))
        self._polys, self._test_polys = self._levels[0]
        self._plan = None
        self._age = 0

    # ---------------- graph planning ---------------- #

    def _clear(self, a, b) -> bool:
        seg = LineString([a, b])
        return not any(seg.intersects(p) for p in self._test_polys)

    def _dijkstra(self, sx, sy, tx, ty):
        nodes = [(sx, sy), (tx, ty)]
        for p in self._polys:
            nodes.extend(list(p.exterior.coords)[:-1])   # corner points
        n = len(nodes)
        # adjacency lazily via clearance checks
        dist = [math.inf] * n
        prev = [-1] * n
        dist[0] = 0.0
        pq = [(0.0, 0)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == 1:
                break
            for v in range(n):
                if v == u:
                    continue
                if not self._clear(nodes[u], nodes[v]):
                    continue
                nd = d + math.dist(nodes[u], nodes[v])
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if math.isinf(dist[1]):
            return None                                   # no path exists
        path = []
        u = 1
        while u != -1:
            path.append(nodes[u])
            u = prev[u]
        return list(reversed(path))                       # start ... target

    # ---------------- per-tick action ---------------- #

    def act(self, state: State, tx: float, ty: float, cruise: float) -> Action4D:
        self._age += 1
        # replan when stale, missing, or the start of the cached plan is far
        if (self._plan is None or self._age >= REPLAN_TICKS
                or (self._plan and math.dist((state.x, state.y),
                                             self._plan[0]) > 8.0)):
            self._plan = None
            for polys, test in self._levels:      # widest inflation first
                self._polys, self._test_polys = polys, test
                self._plan = self._dijkstra(state.x, state.y, tx, ty)
                if self._plan:
                    break
            self._age = 0
        if not self._plan:
            # no path (rare) — hold position; the rollout will be dropped as a
            # teacher dead-end anyway
            return Action4D(vz_up=0.8 * (cruise - state.up))

        # final approach: macro obstacles are behind us — aim straight at the
        # target (the carrot geometry caused rare 2-3 m stalls at the goal)
        target_d0 = math.dist((state.x, state.y), (tx, ty))
        if target_d0 < 6.0:
            v = min(SPEED, target_d0)
            return Action4D(vx=(tx - state.x) / max(target_d0, 1e-6) * v,
                            vy=(ty - state.y) / max(target_d0, 1e-6) * v,
                            vz_up=0.8 * (cruise - state.up))

        # carrot point: walk along the polyline CARROT_M ahead of the vehicle
        cx, cy = state.x, state.y
        remaining = CARROT_M
        carrot = self._plan[-1]
        pts = [(state.x, state.y)] + [p for p in self._plan[1:]]
        for a, b in zip(pts, pts[1:]):
            seg_len = math.dist(a, b)
            if seg_len >= remaining:
                t = remaining / max(seg_len, 1e-9)
                carrot = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                break
            remaining -= seg_len
        dx, dy = carrot[0] - cx, carrot[1] - cy
        d = math.hypot(dx, dy)
        target_d = math.dist((state.x, state.y), (tx, ty))
        v = min(SPEED, target_d)                          # slow near the goal
        if d < 1e-6:
            return Action4D(vz_up=0.8 * (cruise - state.up))
        return Action4D(vx=dx / d * v, vy=dy / d * v,
                        vz_up=0.8 * (cruise - state.up))
