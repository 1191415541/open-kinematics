"""Row-level deterministic checkpoint and resume support."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Checkpoint:
    """Checkpoint header and completed case IDs."""

    model_hash: str
    case_hash: str
    solver_hash: str
    completed_ids: tuple[str, ...]


class CheckpointStore:
    """Persist and validate a small JSON checkpoint file."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def save(self, checkpoint: Checkpoint) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "model_hash": checkpoint.model_hash,
            "case_hash": checkpoint.case_hash,
            "solver_hash": checkpoint.solver_hash,
            "completed_ids": sorted(set(checkpoint.completed_ids)),
        }
        self.path.write_text(
            json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8"
        )

    def load(self, *, model_hash: str, case_hash: str, solver_hash: str) -> Checkpoint:
        if not self.path.is_file():
            return Checkpoint(model_hash, case_hash, solver_hash, ())
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        for key, expected in (
            ("model_hash", model_hash),
            ("case_hash", case_hash),
            ("solver_hash", solver_hash),
        ):
            if payload.get(key) != expected:
                raise ValueError(f"checkpoint {key} does not match current run")
        return Checkpoint(
            model_hash, case_hash, solver_hash, tuple(payload.get("completed_ids", ()))
        )

    def add(
        self, case_id: str, *, model_hash: str, case_hash: str, solver_hash: str
    ) -> Checkpoint:
        current = self.load(
            model_hash=model_hash, case_hash=case_hash, solver_hash=solver_hash
        )
        updated = Checkpoint(
            model_hash,
            case_hash,
            solver_hash,
            tuple(sorted(set(current.completed_ids) | {case_id})),
        )
        self.save(updated)
        return updated
