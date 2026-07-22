"""
REST hot-apply endpoint — the external interface other teams asked for
(2026-07-07 meeting: "provide an API so other modules can create obstacles /
no-fly zones dynamically in the simulation").

Wraps a live Shield instance in a small FastAPI app:

    GET  /health          liveness + current policy hash/generation
    GET  /policy          full active policy (all constraints)
    POST /nfz             inject a dynamic no-fly zone mid-flight

POST /nfz body (matches the interface proposal in docs/architecture-v2.md §4):

    {
      "id": "nfz-landslide-007",
      "vertices": [{"x": 23, "y": 6}, {"x": 31, "y": 6}, {"x": 31, "y": 14}],
      "altitude_floor_m": 0,
      "altitude_ceiling_m": 100,
      "margin_m": 1.0
    }

Run alongside a mission (demo/run_demo.py --api) — the mission loop and the
API share the same Shield object; an accepted POST takes effect on the very
next 10 Hz tick, bumps the policy generation, and changes the policy hash.
"""
from __future__ import annotations

import threading

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from .models import PolygonFence, XY
from .shield import Shield


class NfzRequest(BaseModel):
    id: str
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0


def build_app(shield: Shield) -> FastAPI:
    app = FastAPI(title="Guardrail hot-apply API", version="0.1")
    lock = threading.Lock()

    @app.get("/health")
    def health():
        return {
            "status": "ok",
            "policy_id": shield.policy.policy_id,
            "policy_hash": shield.policy.policy_hash,
            "generation": shield.policy.generation,
        }

    @app.get("/policy")
    def policy():
        return shield.policy.model_dump()

    @app.post("/nfz")
    def add_nfz(req: NfzRequest):
        with lock:
            if any(c.id == req.id for c in shield.policy.constraints):
                raise HTTPException(409, f"constraint id {req.id!r} already exists")
            fence = PolygonFence(
                id=req.id, type="polygon_fence",
                vertices=req.vertices,
                altitude_floor_m=req.altitude_floor_m,
                altitude_ceiling_m=req.altitude_ceiling_m,
                margin_m=req.margin_m,
            )
            shield.hot_apply(fence)
        return {
            "applied": req.id,
            "generation": shield.policy.generation,
            "policy_hash": shield.policy.policy_hash,
        }

    return app


def serve_in_background(shield: Shield, host: str = "127.0.0.1",
                        port: int = 8071) -> threading.Thread:
    """Start uvicorn in a daemon thread so the 10 Hz mission loop stays the
    main thread. Returns the thread (daemon: dies with the mission)."""
    import uvicorn

    app = build_app(shield)
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True, name="guardrail-api")
    t.start()
    return t
