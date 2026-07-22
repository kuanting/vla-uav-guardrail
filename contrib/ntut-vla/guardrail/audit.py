"""
Audit log — one JSONL record per Shield decision that touched the action.

Mirrors the grant's audit-log rule: every record carries policy_hash so any
KPI number can be traced back to the exact policy that produced it.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from .shield import ShieldDecision


class AuditLogger:
    def __init__(self, path: str | Path, policy_hash: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.policy_hash = policy_hash
        self._n = 0

    def log(self, tick: int, decision: ShieldDecision) -> None:
        """Write a record only when something happened (violation seen)."""
        if not decision.touched:
            return
        rec = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "policy_hash": self.policy_hash,
            "tick": tick,
            "raw_action": decision.raw.model_dump(),
            "violations": [v.model_dump() for v in decision.violations],
            "repairs": [r.model_dump() for r in decision.repairs],
            "emitted_action": decision.emitted.model_dump(),
            "braked": decision.braked,
        }
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
        self._n += 1

    @property
    def records_written(self) -> int:
        return self._n
