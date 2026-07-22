"""
Mission Control GUI for the AerialVLA + guardrail Project AirSim demo.

Left: square map canvas, world frame x=North / y=East, range [-80, 80] m.
  - left-click        -> add numbered waypoint
  - right-button drag -> draw a rectangular no-fly zone (NFZ)
Right: sim launcher, flight launcher, live log, status bar.

Flight is delegated to demo/aerialvla_pas_demo.py (subprocess); its live.json
(written every 5 ticks) drives the moving drone dot + trail on the canvas.

Selftest:  python demo/gui_mission_control.py --selftest
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

ROOT = Path(__file__).resolve().parents[1]

# city occupancy map + global planner (shared with the flight demo). Keep the
# import soft so the GUI still runs if the module is missing.
sys.path.insert(0, str(ROOT / "demo"))
try:
    import city_planner
except Exception:
    city_planner = None

PYTHON = "C:/Users/natha/.conda/envs/vla-real/python.exe"
UNREAL = ("C:/Program Files/Epic Games/UE_5.7/Engine/Binaries/Win64/"
          "UnrealEditor.exe")
UPROJECT = ROOT / "PASBlocks" / "Blocks.uproject"
FLIGHT_SCRIPT = "demo/aerialvla_pas_demo.py"
ADAPTER = "D:/models/aerialvla-ft/run2/epoch1"
POLICY_REL = "policies/gui_policy.yaml"
BASE_POLICY = ROOT / "policies" / "urban_demo_policy.yaml"
TAG = "gui_flight"
LIVE_JSON = ROOT / "demo" / "out" / TAG / "live.json"
TRAJ_PNG = ROOT / "demo" / "out" / TAG / "trajectory.png"
SIM_PORT = 8989

MAPS = {
    "day": "/Game/JapaneseCity/Maps/Demo_day",
    "night": "/Game/JapaneseCity/Maps/Demo_night",
    "airport": "/Game/Airport/Maps/demo",
    "military": "/Game/MilitaryAirport/Maps/Map_Airbase_Demo",
    "blocks": None,
}

# occupancy map per world — the global planner ONLY applies where we surveyed
# that world's buildings. day/night are the same JapaneseCity buildings; airport/
# military/blocks have no survey yet -> planner off there (reactive only), so the
# GUI preview and the actual flight stay consistent (no phantom buildings).
CITYMAP_DIR = ROOT / "demo" / "out" / "citymap"


def occ_path_for(map_choice: str) -> Path:
    return CITYMAP_DIR / f"occ_{map_choice}.npz"

WORLD_MIN, WORLD_MAX = -80.0, 80.0
WORLD_SPAN = WORLD_MAX - WORLD_MIN
CANVAS = 640
SPAWN = (35.0, -20.0)          # (x=North, y=East) scene origin
TRAIL_LEN = 40
FONT = ("Segoe UI", 9)


# Fallback constraints if the base policy cannot be read (same structure as
# policies/urban_demo_policy.yaml).
FALLBACK_KEEP = [
    {"id": "alt-band-urban", "type": "altitude_envelope",
     "constraint_type": "hard", "priority": "P0", "violation_action": "repair",
     "alt_min_m": 15, "alt_max_m": 25},
    {"id": "kin-caps", "type": "kinematic_envelope",
     "constraint_type": "hard", "priority": "P1", "violation_action": "repair",
     "speed_max_mps": 4.0, "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0},
]


class MissionControl:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("Mission Control — AerialVLA Guardrail Demo")
        root.resizable(False, False)

        self.waypoints: list[tuple[float, float]] = []   # (x_north, y_east)
        self.nfzs: list[tuple[float, float, float, float]] = []  # xmin,xmax,ymin,ymax
        self._drag_start = None
        self._drag_rect_id = None

        # city map + planner state (populated by _load_map)
        self.occ = None                       # dict from city_planner.load_occ
        self._plan_grid = None                # inflated grid used for planning
        self._building_rects = None           # precomputed canvas rects
        self.planned: list[tuple[float, float]] = []   # global-planner polyline
        self.snapped: list[bool] = []         # per-waypoint: snapped out of bldg

        self.sim_proc: subprocess.Popen | None = None
        self.flight_proc: subprocess.Popen | None = None
        self.log_q: queue.Queue = queue.Queue()
        self.status_q: queue.Queue = queue.Queue()
        self.trail: list[tuple[float, float]] = []
        self._traj_win = None
        self._traj_photo = None    # keep PhotoImage reference alive

        self._build_ui()
        self._load_map()
        self._replan()
        self.redraw()
        self.root.after(100, self._poll_queues)
        self.root.after(200, self._poll_live)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=6)
        main.grid(sticky="nsew")

        self.canvas = tk.Canvas(main, width=CANVAS, height=CANVAS,
                                bg="#fafafa", highlightthickness=1,
                                highlightbackground="#999")
        self.canvas.grid(row=0, column=0, rowspan=2, padx=(0, 8))
        self.canvas.bind("<Button-1>", self._on_left_click)
        self.canvas.bind("<ButtonPress-3>", self._on_rdrag_start)
        self.canvas.bind("<B3-Motion>", self._on_rdrag_move)
        self.canvas.bind("<ButtonRelease-3>", self._on_rdrag_end)

        side = ttk.Frame(main)
        side.grid(row=0, column=1, sticky="new")

        r = 0
        ttk.Label(side, text="Map:", font=FONT).grid(row=r, column=0, sticky="w")
        self.map_var = tk.StringVar(value="day")
        map_cb = ttk.Combobox(side, textvariable=self.map_var, values=list(MAPS),
                              state="readonly", width=12, font=FONT)
        map_cb.grid(row=r, column=1, sticky="w", pady=2)
        # changing the world reloads that world's building map (or turns the
        # planner off if it wasn't surveyed) so preview matches the flight
        map_cb.bind("<<ComboboxSelected>>", lambda e: self.reload_map())
        r += 1

        ttk.Label(side, text="Altitude:", font=FONT).grid(row=r, column=0, sticky="w")
        self.alt_var = tk.StringVar(value="High 35-55m (over roofs)")
        ttk.Combobox(side, textvariable=self.alt_var,
                     values=["Low 15-25m (street canyon)",
                             "High 35-55m (over roofs)"],
                     state="readonly", width=22, font=FONT).grid(
            row=r, column=1, sticky="w", pady=2)
        r += 1

        self.sim_btn = ttk.Button(side, text="1. Start Sim",
                                  command=self.start_sim)
        self.sim_btn.grid(row=r, column=0, sticky="ew", pady=2)
        self.sim_status = ttk.Label(side, text="sim: not started", font=FONT,
                                    foreground="#555")
        self.sim_status.grid(row=r, column=1, sticky="w", padx=4)
        r += 1

        self.fly_btn = ttk.Button(side, text="2. Fly", command=self.fly,
                                  state="disabled")
        self.fly_btn.grid(row=r, column=0, sticky="ew", pady=2)
        self.abort_btn = ttk.Button(side, text="Abort", command=self.abort,
                                    state="disabled")
        self.abort_btn.grid(row=r, column=1, sticky="w", padx=4)
        r += 1

        self.keep_sim = tk.BooleanVar(value=True)
        ttk.Checkbutton(side, text="keep sim running",
                        variable=self.keep_sim).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=2)
        r += 1

        mapf = ttk.Frame(side)
        mapf.grid(row=r, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(mapf, text="Reload map",
                   command=self.reload_map).pack(side="left", padx=2)
        self.map_status = ttk.Label(mapf, text="city map: —", font=FONT,
                                    foreground="#555")
        self.map_status.pack(side="left", padx=6)
        r += 1

        edit = ttk.Frame(side)
        edit.grid(row=r, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Button(edit, text="Undo waypoint",
                   command=self.undo_waypoint).pack(side="left", padx=2)
        ttk.Button(edit, text="Undo NFZ",
                   command=self.undo_nfz).pack(side="left", padx=2)
        ttk.Button(edit, text="Clear all",
                   command=self.clear_all).pack(side="left", padx=2)
        r += 1

        ttk.Label(side, text="Log:", font=FONT).grid(row=r, column=0, sticky="w")
        r += 1
        logf = ttk.Frame(side)
        logf.grid(row=r, column=0, columnspan=2, sticky="nsew")
        self.log = tk.Text(logf, width=48, height=28, font=("Consolas", 8),
                           state="disabled", wrap="none", bg="#ffffff",
                           fg="#222222")
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self.status = ttk.Label(main, text="ready — left-click: waypoint, "
                                "right-drag: NFZ", font=FONT, relief="sunken",
                                anchor="w")
        self.status.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))

    # ------------------------------------------------- coordinate mapping
    def w2c(self, x_north: float, y_east: float) -> tuple[float, float]:
        sx = (y_east - WORLD_MIN) / WORLD_SPAN * CANVAS
        sy = CANVAS - (x_north - WORLD_MIN) / WORLD_SPAN * CANVAS
        return sx, sy

    def c2w(self, sx: float, sy: float) -> tuple[float, float]:
        y_east = sx / CANVAS * WORLD_SPAN + WORLD_MIN
        x_north = (CANVAS - sy) / CANVAS * WORLD_SPAN + WORLD_MIN
        return x_north, y_east

    # ---------------------------------------------------------- drawing
    def redraw(self):
        c = self.canvas
        c.delete("static")
        # grid every 20 m + labels
        for v in range(int(WORLD_MIN), int(WORLD_MAX) + 1, 20):
            sx, _ = self.w2c(0, v)
            _, sy = self.w2c(v, 0)
            c.create_line(sx, 0, sx, CANVAS, fill="#dddddd", tags="static")
            c.create_line(0, sy, CANVAS, sy, fill="#dddddd", tags="static")
            c.create_text(sx + 2, CANVAS - 8, text=str(v), anchor="w",
                          font=("Segoe UI", 7), fill="#888", tags="static")
            c.create_text(4, sy - 2, text=str(v), anchor="sw",
                          font=("Segoe UI", 7), fill="#888", tags="static")
        c.create_text(CANVAS - 6, CANVAS - 20, text="East (m) →", anchor="e",
                      font=FONT, fill="#666", tags="static")
        c.create_text(16, 10, text="↑ North (m)", anchor="w", font=FONT,
                      fill="#666", tags="static")

        # city buildings — one light-grey square per occupied cell, drawn UNDER
        # NFZ / waypoints / drone. Coords are precomputed once when the map loads.
        if self._building_rects:
            for (bx0, by0, bx1, by1) in self._building_rects:
                c.create_rectangle(bx0, by0, bx1, by1, fill="#c8ccd0",
                                   outline="", tags="static")

        # NFZ rectangles (red, 25 % via stipple)
        for xmin, xmax, ymin, ymax in self.nfzs:
            x0, y0 = self.w2c(xmax, ymin)
            x1, y1 = self.w2c(xmin, ymax)
            c.create_rectangle(x0, y0, x1, y1, fill="red", stipple="gray25",
                               outline="red", width=2, tags="static")

        # spawn marker
        sx, sy = self.w2c(*SPAWN)
        c.create_oval(sx - 9, sy - 9, sx + 9, sy + 9, outline="green",
                      width=2, tags="static")
        c.create_text(sx, sy, text="S", fill="green",
                      font=("Segoe UI", 10, "bold"), tags="static")

        # straight requested route (light blue, spawn -> waypoints in order)
        pts = [SPAWN] + self.waypoints
        for a, b in zip(pts, pts[1:]):
            c.create_line(*self.w2c(*a), *self.w2c(*b), fill="#7799cc",
                          dash=(4, 3), tags="static")

        # global-planner path routed around the buildings (dashed teal)
        if self.planned and len(self.planned) >= 2:
            flat = []
            for (px, py) in self.planned:
                flat.extend(self.w2c(px, py))
            c.create_line(*flat, fill="#0f9c9c", width=2, dash=(6, 4),
                          tags="static")

        # numbered waypoints (a snapped-out-of-building one gets a red ring)
        for i, (wx, wy) in enumerate(self.waypoints):
            sx, sy = self.w2c(wx, wy)
            snapped = i < len(self.snapped) and self.snapped[i]
            c.create_oval(sx - 7, sy - 7, sx + 7, sy + 7, fill="#ffd75e",
                          outline="#d02020" if snapped else "#b8860b",
                          width=2, tags="static")
            if snapped:
                c.create_oval(sx - 10, sy - 10, sx + 10, sy + 10,
                              outline="#d02020", dash=(2, 2), tags="static")
            c.create_text(sx, sy, text=str(i + 1), font=("Segoe UI", 8, "bold"),
                          tags="static")
        c.tag_raise("live")
        self._update_fly_state()

    def _draw_live(self, x, y, up, tick, avoid, touched):
        c = self.canvas
        c.delete("live")
        self.trail.append((x, y))
        self.trail = self.trail[-TRAIL_LEN:]
        n = len(self.trail)
        for i, (tx, ty) in enumerate(self.trail[:-1]):
            # fade: older points lighter (Tk has no alpha, fake it with shades)
            f = i / max(1, n - 1)
            shade = int(0xd0 - f * 0x90)
            color = f"#{shade:02x}{shade:02x}ff"
            sx, sy = self.w2c(tx, ty)
            r = 1.5 + 1.5 * f
            c.create_oval(sx - r, sy - r, sx + r, sy + r, fill=color,
                          outline="", tags="live")
        sx, sy = self.w2c(x, y)
        col = "orange" if touched else "#1f6fd0"
        c.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=col,
                      outline="black", tags="live")
        self.status.config(text=f"tick {tick} | pos ({x:.1f} N, {y:.1f} E) "
                           f"| alt {up:.1f} m | avoid: {avoid or '-'}")

    # ----------------------------------------------------- map handlers
    def _on_left_click(self, ev):
        x, y = self.c2w(ev.x, ev.y)
        snapped = False
        if self.occ is not None and city_planner is not None:
            try:
                grid = self.occ["occ"]
                res, ox, oy = self.occ["res"], self.occ["ox"], self.occ["oy"]
                if city_planner.is_blocked(grid, res, ox, oy, x, y):
                    self._log("[map] that point is inside a building — "
                              "snapping to nearest open spot")
                    x, y = city_planner.nearest_free(grid, res, ox, oy, x, y)
                    snapped = True
            except Exception as e:
                self._log(f"[map] snap check failed: {e}")
        self.waypoints.append((x, y))
        self.snapped.append(snapped)
        self._replan()
        self.redraw()

    def _on_rdrag_start(self, ev):
        self._drag_start = (ev.x, ev.y)

    def _on_rdrag_move(self, ev):
        if not self._drag_start:
            return
        if self._drag_rect_id:
            self.canvas.delete(self._drag_rect_id)
        x0, y0 = self._drag_start
        self._drag_rect_id = self.canvas.create_rectangle(
            x0, y0, ev.x, ev.y, outline="red", dash=(3, 2))

    def _on_rdrag_end(self, ev):
        if not self._drag_start:
            return
        if self._drag_rect_id:
            self.canvas.delete(self._drag_rect_id)
            self._drag_rect_id = None
        ax, ay = self.c2w(*self._drag_start)
        bx, by = self.c2w(ev.x, ev.y)
        self._drag_start = None
        xmin, xmax = sorted((ax, bx))
        ymin, ymax = sorted((ay, by))
        if xmax - xmin < 2.0 or ymax - ymin < 2.0:
            return                                  # ignore accidental clicks
        self.nfzs.append((xmin, xmax, ymin, ymax))
        self._replan()                              # re-route around the new NFZ
        self.redraw()

    def undo_waypoint(self):
        if self.waypoints:
            self.waypoints.pop()
            if self.snapped:
                self.snapped.pop()
            self._replan()
            self.redraw()

    def undo_nfz(self):
        if self.nfzs:
            self.nfzs.pop()
            self._replan()
            self.redraw()

    def clear_all(self):
        self.waypoints.clear()
        self.snapped.clear()
        self.nfzs.clear()
        self.planned = []
        self.trail.clear()
        self.canvas.delete("live")
        self.redraw()

    # ------------------------------------------------- city map + planner
    def _load_map(self):
        """(Re)load the city occupancy map. Sets self.occ (dict or None), the
        inflated planning grid, and the precomputed building rectangles. Safe to
        call with no map file present — everything no-ops to the empty state."""
        self.occ = None
        self._plan_grid = None
        self._building_rects = None
        if city_planner is None:
            if hasattr(self, "map_status"):
                self.map_status.config(text="city map: planner unavailable")
            return None
        choice = self.map_var.get()
        occ_file = occ_path_for(choice)
        try:
            m = city_planner.load_occ(str(occ_file))
        except Exception as e:
            self._log(f"[map] load error: {e}")
            m = None
        if m is None:
            self._log(f"[map] no occupancy map for '{choice}' "
                      f"({occ_file.name}) — planner OFF, reactive only")
            if hasattr(self, "map_status"):
                self.map_status.config(text=f"city map: none ({choice})")
            self._precompute_building_rects()   # clears stale buildings
            return None
        self.occ = m
        try:
            self._plan_grid = city_planner.inflate(m["occ"], m["res"], 6.0)
        except Exception as e:
            self._plan_grid = None
            self._log(f"[map] inflate failed: {e}")
        self._precompute_building_rects()
        try:
            n_cells = int((m["occ"] > 0).sum())
        except Exception:
            n_cells = 0
        self._log(f"[map] loaded {m['N']}x{m['N']} @ {m['res']:.1f} m/cell, "
                  f"{n_cells} building cells")
        if hasattr(self, "map_status"):
            self.map_status.config(text=f"city map: {m['N']}x{m['N']} "
                                        f"@ {m['res']:.0f}m")
        return m

    def reload_map(self):
        self._load_map()
        self._replan()
        self.redraw()

    def _precompute_building_rects(self):
        """Cache one canvas rectangle per occupied cell so redraw is cheap."""
        self._building_rects = None
        if self.occ is None:
            return
        occ = self.occ["occ"]
        res = self.occ["res"]
        ox, oy = self.occ["ox"], self.occ["oy"]
        N = self.occ["N"]
        half = res / 2.0
        rects = []
        for i in range(N):
            for j in range(N):
                if occ[i, j] == 0:
                    continue
                cx = ox + i * res          # world North of cell centre
                cy = oy + j * res          # world East  of cell centre
                ax, ay = self.w2c(cx - half, cy - half)
                bx, by = self.w2c(cx + half, cy + half)
                x0, x1 = (ax, bx) if ax <= bx else (bx, ax)
                y0, y1 = (ay, by) if ay <= by else (by, ay)
                rects.append((x0, y0, x1, y1))
        self._building_rects = rects

    def _replan(self):
        """Rebuild self.planned: an A* path from SPAWN through every waypoint,
        planned on the inflated grid (buildings + drawn NFZs, clearance 6 m).
        Routes GLOBALLY around a no-fly-zone, not just reactively. Never raises."""
        self.planned = []
        if city_planner is None or not self.waypoints:
            return
        if self.occ is None and not self.nfzs:
            return                         # nothing to plan around
        try:
            import numpy as np
            if self.occ is not None:
                res = self.occ["res"]
                ox, oy = self.occ["ox"], self.occ["oy"]
                base = self.occ["occ"].copy()
            else:                          # NFZ-only: default 80 m / 2 m grid
                res, ox, oy = 2.0, -80.0, -80.0
                n = int(-2 * ox / res)
                base = np.zeros((n, n), dtype="uint8")
            # stamp every drawn NFZ rectangle (axis-aligned) into the grid
            N = base.shape[0]
            for xmin, xmax, ymin, ymax in self.nfzs:
                i0 = max(0, int((xmin - ox) / res)); i1 = min(N, int((xmax - ox) / res) + 1)
                j0 = max(0, int((ymin - oy) / res)); j1 = min(N, int((ymax - oy) / res) + 1)
                base[i0:i1, j0:j1] = 1
            grid = city_planner.inflate(base, res, 6.0)
            pts = [SPAWN] + list(self.waypoints)
            full = []
            unreachable = False
            for a, b in zip(pts, pts[1:]):
                leg = city_planner.plan(grid, res, ox, oy, a, b)
                if not leg:
                    unreachable = True
                    continue
                full.extend(leg if not full else leg[1:])
            self.planned = full
            if unreachable:
                self._log("[map] planner: a leg was unreachable "
                          "(partial path shown)")
        except Exception as e:
            self.planned = []
            self._log(f"[map] planner error: {e}")

    def _update_fly_state(self):
        ok = bool(self.waypoints) and self.flight_proc is None
        self.fly_btn.config(state="normal" if ok else "disabled")

    # ----------------------------------------------------------- logging
    def _log(self, line: str):
        self.log.config(state="normal")
        self.log.insert("end", line.rstrip("\n") + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    # -------------------------------------------------------------- sim
    def start_sim(self):
        self.sim_btn.config(state="disabled")
        self.sim_status.config(text="sim: starting…", foreground="#b8860b")
        choice = self.map_var.get()
        threading.Thread(target=self._sim_worker, args=(choice,),
                         daemon=True).start()

    def _sim_worker(self, choice: str):
        # kill any previous sim instances
        for exe in ("UnrealEditor.exe", "AirSimNH.exe"):
            try:
                subprocess.run(["taskkill", "/F", "/IM", exe],
                               capture_output=True, timeout=15)
            except Exception:
                pass
        time.sleep(2.0)

        cmd = [UNREAL, str(UPROJECT)]
        map_path = MAPS.get(choice)
        if map_path:
            cmd.append(map_path)
        cmd += ["-game", "-windowed", "-ResX=1280", "-ResY=720"]
        try:
            self.sim_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.status_q.put(("sim", f"failed: {e}"))
            return
        self.status_q.put(("log", f"[sim] launched {choice} "
                           f"({map_path or 'default Blocks map'})"))

        # poll port 8989 until the sim RPC server is up (max 10 min)
        deadline = time.time() + 600
        while time.time() < deadline:
            if self.sim_proc.poll() is not None:
                self.status_q.put(("sim", "failed (sim exited)"))
                return
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(1.0)
                    if s.connect_ex(("127.0.0.1", SIM_PORT)) == 0:
                        self.status_q.put(("sim", "READY"))
                        return
            except Exception:
                pass
            time.sleep(2.0)
        self.status_q.put(("sim", "failed (timeout waiting for port 8989)"))

    # ------------------------------------------------------------ policy
    def _write_policy(self) -> Path:
        """Rewrite polygon_fence entries from drawn NFZs; keep the rest of the
        base policy (altitude + kinematic envelopes) unchanged."""
        keep = FALLBACK_KEEP
        version = "0.1.0"
        try:
            import yaml
            base = yaml.safe_load(BASE_POLICY.read_text(encoding="utf-8"))
            keep = [c for c in base.get("constraints", [])
                    if c.get("type") != "polygon_fence"]
            version = base.get("version", version)
        except Exception as e:
            self._log(f"[policy] base policy read failed ({e}) — "
                      "using built-in envelope constraints")

        fences = []
        for i, (xmin, xmax, ymin, ymax) in enumerate(self.nfzs):
            fences.append({
                "id": f"gui-nfz-{i + 1}",
                "type": "polygon_fence",
                "constraint_type": "hard",
                "priority": "P0",
                "violation_action": "repair",
                "vertices": [
                    {"x": round(xmin, 1), "y": round(ymin, 1)},
                    {"x": round(xmax, 1), "y": round(ymin, 1)},
                    {"x": round(xmax, 1), "y": round(ymax, 1)},
                    {"x": round(xmin, 1), "y": round(ymax, 1)},
                ],
                "altitude_floor_m": 0,
                "altitude_ceiling_m": 60,
                "margin_m": 1.0,
            })
        # altitude band from the GUI selector (High = fly over the city roofs)
        high = self.alt_var.get().startswith("High")
        amin, amax = (35, 55) if high else (15, 25)
        for c in keep:
            if c.get("type") == "altitude_envelope":
                c["alt_min_m"], c["alt_max_m"] = amin, amax
        doc = {"policy_id": "gui-mission", "version": version,
               "constraints": fences + keep}
        path = ROOT / POLICY_REL
        try:
            import yaml
            path.write_text(yaml.safe_dump(doc, sort_keys=False),
                            encoding="utf-8")
        except Exception as e:
            self._log(f"[policy] WRITE FAILED: {e}")
            raise
        return path

    # ------------------------------------------------------------ flight
    def fly(self):
        if not self.waypoints or self.flight_proc is not None:
            return
        try:
            self._write_policy()
        except Exception:
            return
        route = "; ".join(f"{x:.1f},{y:.1f}" for x, y in self.waypoints)
        cruise = 45 if self.alt_var.get().startswith("High") else 20
        wx, wy = self.waypoints[0]
        choice = self.map_var.get()
        # pass the world-specific occupancy map so the flight plans against the
        # SAME buildings the GUI previewed (missing file -> flight's load_occ
        # returns None -> planner off, matching the preview)
        citymap = occ_path_for(choice)
        cmd = [PYTHON, FLIGHT_SCRIPT, "--best",
               "--adapter", ADAPTER,
               "--route", route,
               "--command", f"fly to ({wx:.0f}, {wy:.0f}) at 6 m/s altitude {cruise}",
               "--policy", POLICY_REL,
               "--citymap", str(citymap),
               "--tag", TAG,
               "--map-label", choice]
        # stale live.json from a previous run would flash an old position
        try:
            LIVE_JSON.unlink(missing_ok=True)
        except Exception:
            pass
        self.trail.clear()
        self._log(f"[fly] route: {route}")
        self._log(f"[fly] {' '.join(cmd)}")
        try:
            self.flight_proc = subprocess.Popen(
                cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1)
        except Exception as e:
            self._log(f"[fly] LAUNCH FAILED: {e}")
            self.flight_proc = None
            return
        self.fly_btn.config(state="disabled")
        self.abort_btn.config(state="normal")
        threading.Thread(target=self._flight_reader, args=(self.flight_proc,),
                         daemon=True).start()

    def _flight_reader(self, proc: subprocess.Popen):
        try:
            for line in proc.stdout:
                self.log_q.put(line)
        except Exception:
            pass
        code = proc.wait()
        self.log_q.put(None)                       # sentinel: flight ended
        self.status_q.put(("log", f"[fly] flight process exited (code {code})"))

    def abort(self):
        if self.flight_proc is not None:
            try:
                self.flight_proc.terminate()
                self._log("[fly] abort requested — terminating flight")
            except Exception as e:
                self._log(f"[fly] abort failed: {e}")

    def _flight_ended(self):
        self.flight_proc = None
        self.abort_btn.config(state="disabled")
        self._update_fly_state()
        self.status.config(text="flight ended")
        if TRAJ_PNG.exists():
            self._show_trajectory()

    def _show_trajectory(self):
        try:
            from PIL import Image, ImageTk
            img = Image.open(TRAJ_PNG)
            scale = 800 / img.width
            img = img.resize((800, max(1, int(img.height * scale))))
            win = tk.Toplevel(self.root)
            win.title("Trajectory — gui_flight")
            self._traj_photo = ImageTk.PhotoImage(img)   # keep reference
            tk.Label(win, image=self._traj_photo).pack()
            self._traj_win = win
        except Exception as e:
            self._log(f"[plot] could not show trajectory.png: {e}")

    # ----------------------------------------------------------- polling
    def _poll_queues(self):
        try:
            while True:
                item = self.log_q.get_nowait()
                if item is None:
                    self._flight_ended()
                else:
                    self._log(item)
        except queue.Empty:
            pass
        try:
            while True:
                kind, msg = self.status_q.get_nowait()
                if kind == "sim":
                    color = {"READY": "green"}.get(msg, "#b00")
                    self.sim_status.config(text=f"sim: {msg}", foreground=color)
                    self.sim_btn.config(state="normal")
                else:
                    self._log(msg)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queues)

    def _poll_live(self):
        if self.flight_proc is not None:
            try:
                data = json.loads(LIVE_JSON.read_text(encoding="utf-8"))
                self._draw_live(data["x"], data["y"], data.get("up", 0.0),
                                data.get("tick", 0), data.get("avoid", ""),
                                data.get("touched", False))
            except Exception:
                pass                       # missing / mid-write / locked file
        self.root.after(200, self._poll_live)      # 5 Hz

    # ----------------------------------------------------------- closing
    def _on_close(self):
        if self.flight_proc is not None:
            try:
                self.flight_proc.terminate()
            except Exception:
                pass
        if self.sim_proc is not None and not self.keep_sim.get():
            try:
                self.sim_proc.terminate()
            except Exception:
                pass
        self.root.destroy()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true",
                    help="build UI, auto-close after 700 ms, print SELFTEST OK")
    args = ap.parse_args()

    root = tk.Tk()
    MissionControl(root)
    if args.selftest:
        root.after(700, root.destroy)
    root.mainloop()
    if args.selftest:
        print("SELFTEST OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
