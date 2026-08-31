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
    def __init__(self, path: str | Path, policy_hash: "str | object"):
        """`policy_hash` may be a string OR the Policy itself.

        Pass the POLICY when the run can hot-apply a rule. A string is a
        snapshot taken at construction, and `Shield.hot_apply` mutates the live
        policy and bumps its generation, so every record written afterwards
        carried the hash of a policy that no longer applied - the opposite of
        what hot_apply's own docstring promises ("every artefact after this
        instant carries a different policy_hash - the audit trail shows exactly
        which rules were active when").

        Reproduced before the fix: a record whose violation was `nfz-hot`
        stamped with the hash of a policy that did not contain `nfz-hot`. That
        is precisely the traceability this module exists to provide, and it
        failed on the one feature built for an external team (`POST /nfz`).

        The string form still works, so the seven existing call sites are
        unaffected; they simply do not benefit.
        """
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._policy = None if isinstance(policy_hash, str) else policy_hash
        self._static_hash = policy_hash if isinstance(policy_hash, str) else None
        self._n = 0

    @property
    def policy_hash(self) -> str:
        """Read live when a policy was supplied, so a hot-applied rule shows up."""
        if self._policy is not None:
            return getattr(self._policy, "policy_hash", None) or ""
        return self._static_hash or ""

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
            # What is still wrong with the action that was FLOWN. Empty is the
            # good case; non-empty is a P0 escape, the grant's hard KPI.
            "emitted_violations": [v.model_dump()
                                   for v in decision.emitted_violations],
            "braked": decision.braked,
        }
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
        self._n += 1

    @property
    def records_written(self) -> int:
        return self._n
