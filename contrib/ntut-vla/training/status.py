"""
Live-status writer — single JSON file the dashboard polls.

Every pipeline stage calls StatusWriter.update(...) with whatever changed;
the file is written atomically (tmp + replace) so the dashboard never reads
a half-written JSON.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

STATUS_PATH = Path(__file__).resolve().parent / "status.json"


class StatusWriter:
    def __init__(self, path: Path = STATUS_PATH):
        self.path = path
        self.state: dict = {
            "run_started": time.strftime("%Y-%m-%d %H:%M:%S"),
            "stage": "starting",
            "iteration": 0,
            "config": {},
            "gen": {},
            "train": {"loss_hist": []},
            "eval": {},
            "iterations": [],
            "done": False,
            "verdict": "",
        }
        self.flush()

    def update(self, **kw) -> None:
        for k, v in kw.items():
            if isinstance(v, dict) and isinstance(self.state.get(k), dict):
                self.state[k].update(v)
            else:
                self.state[k] = v
        self.flush()

    def flush(self) -> None:
        self.state["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        # Monitoring must NEVER kill the training run. On Windows, os.replace
        # can hit a transient PermissionError when the dashboard (or OneDrive
        # sync — this project lives in OneDrive) has the file open. Retry
        # briefly, then skip this update; the next one will land.
        for attempt in range(4):
            try:
                tmp = self.path.with_suffix(".tmp")
                tmp.write_text(json.dumps(self.state), encoding="utf-8")
                os.replace(tmp, self.path)
                return
            except (PermissionError, OSError):
                time.sleep(0.1 * (attempt + 1))
        # give up silently — a lost status tick is harmless
