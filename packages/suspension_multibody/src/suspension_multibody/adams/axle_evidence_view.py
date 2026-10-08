"""Frozen evidence projection from ResultEnvelope, without a simulation route."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from .. import __version__
from ..axle_dynamics.result import (
    ANTI_ROLL_OUTPUT_COLUMNS,
    BODY_STATE_COLUMNS,
    BUMP_STOP_OUTPUT_COLUMNS,
    BUSHING_OUTPUT_COLUMNS,
    CONSTRAINT_WRENCH_COLUMNS,
    DAMPER_OUTPUT_COLUMNS,
    DIAGNOSTIC_COLUMNS,
    ENERGY_COLUMNS,
    PERFORMANCE_COLUMNS,
    SPRING_OUTPUT_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
)
from ..axle_dynamics.schema import AxleDynamicsCase, AxleDynamicsModel
from ..io.results import canonical_hash
from ..report.metrics import compute_case_metrics
from ..results.envelope import ResultEnvelope

_DIAGNOSTIC_FIELDS = (
    "accepted", "internal_steps", "rejected_attempts", "newton_iterations",
    "minimum_accepted_step_s", "maximum_accepted_step_s", "last_accepted_step_s",
    "position_residual", "velocity_residual", "dynamics_residual", "active_contacts",
    "contact_events", "local_error_ratio", "energy_residual", "failure_code", "pinned_null_directions",
)


class FrozenAxleEvidence:
    """Project explicitly declared entity IDs into the frozen comparison columns."""

    def __init__(self, result: ResultEnvelope, model: AxleDynamicsModel) -> None:
        self.result = result
        self.times_s = result.times_s
        self.body_names = tuple(row.name for row in model.bodies)
        self.states = np.stack([result.body_state(row.name+"."+row.name) for row in model.bodies], axis=1)
        joints = [(row.name, row.body_b+"."+row.name) for row in model.joints]
        joints.extend((row.name, row.body+"."+row.name) for row in model.driven_coordinates)
        self.constraint_names = tuple(name for name, _ in joints)
        native_joints = tuple(row["name"] for row in (result.raw.model_document or {})["joints"])
        indices = [native_joints.index(entity) for _, entity in joints]
        self.constraint_wrench = result.raw.block("constraint_wrench")[:len(self.times_s), indices]
        for stem, rows, block, width in (
            ("spring", model.springs, "spring_output", 4), ("damper", model.dampers, "damper_output", 4),
            ("bump_stop", model.bump_stops, "bump_stop_output", 5), ("bushing", model.bushings, "bushing_output", 12),
            ("anti_roll_bar", model.anti_roll_bars, "anti_roll_output", 3), ("tire", model.tires, "tire_output", 41),
        ):
            setattr(self, stem+"_names", tuple(row.name for row in rows))
            values = [result.element_state(getattr(row, "body" if stem == "tire" else "body_b")+"."+row.name) for row in rows]
            setattr(self, block, np.stack(values, axis=1) if values else np.zeros((len(self.times_s), 0, width)))
        diagnostics = result.raw.block("diagnostics")[:len(self.times_s)]
        self.diagnostics = SimpleNamespace(**{name: diagnostics[:, column].astype(bool) if name == "accepted" else
            diagnostics[:, column].astype(int) if name in {"internal_steps", "rejected_attempts", "newton_iterations", "active_contacts", "contact_events", "failure_code", "pinned_null_directions"} else diagnostics[:, column]
            for column, name in enumerate(_DIAGNOSTIC_FIELDS)})
        self.energy = result.energy
        tire_ids = {row.body+"."+row.name: row.name for row in model.tires}
        self.contact_events = tuple(SimpleNamespace(time_s=event.time_s, tire=tire_ids[event.element_id],
            transition="enter" if event.transition > 0 else "exit") for event in result.contact_events)
        self.performance = dict(result.raw.performance or {})
        self.metrics = compute_case_metrics("axle_dynamic", result)
        if diagnostics.shape[1] != len(DIAGNOSTIC_COLUMNS):
            raise ValueError("native diagnostics differ from the frozen evidence layout")

    def body_state(self, name: str) -> np.ndarray:
        """Read a declared body from the evidence matrix."""
        return self.states[:, self.body_names.index(name)]

    def tire_state(self, name: str) -> np.ndarray:
        """Read the frozen tire columns."""
        return self.tire_output[:, self.tire_names.index(name)]

    def joint_wrench_on_body_b(self, name: str) -> np.ndarray:
        """Read the frozen marker-referenced joint wrench."""
        return self.constraint_wrench[:, self.constraint_names.index(name)]

    def spring_state(self, name: str) -> np.ndarray:
        """Read the conservative spring ledger."""
        return self.spring_output[:, self.spring_names.index(name)]

    def damper_state(self, name: str) -> np.ndarray:
        """Read the dissipative damper ledger."""
        return self.damper_output[:, self.damper_names.index(name)]

    def bump_stop_state(self, name: str) -> np.ndarray:
        """Read the unilateral stop ledger."""
        return self.bump_stop_output[:, self.bump_stop_names.index(name)]

    def bushing_state(self, name: str) -> np.ndarray:
        """Read the local bushing ledger."""
        return self.bushing_output[:, self.bushing_names.index(name)]

    def anti_roll_bar_state(self, name: str) -> np.ndarray:
        """Read the torsional bar ledger."""
        return self.anti_roll_output[:, self.anti_roll_bar_names.index(name)]

    def __getattr__(self, name: str) -> Any:
        raise AttributeError(name)


def write_frozen_artifact(
    result: FrozenAxleEvidence, destination: Path, *, model: AxleDynamicsModel, case: AxleDynamicsCase,
) -> Path:
    """Serialize the historical comparison columns from the unified result."""
    destination.mkdir(parents=True, exist_ok=True)
    arrays = {"times_s": result.times_s}
    for kind in ("body", "constraint", "spring", "damper", "bump_stop", "bushing", "anti_roll_bar", "tire"):
        arrays[kind+"_names"] = np.asarray(getattr(result, kind+"_names"))
    arrays.update({name: getattr(result, name) for name in ("states", "constraint_wrench", "spring_output",
        "damper_output", "bump_stop_output", "bushing_output", "anti_roll_output")})
    arrays["diagnostics"] = np.column_stack([getattr(result.diagnostics, name) for name in _DIAGNOSTIC_FIELDS])
    arrays.update({"tire_output": result.tire_output, "energy": result.energy,
        "contact_event_time_s": np.asarray([row.time_s for row in result.contact_events], dtype=np.float64),
        "contact_event_tire": np.asarray([row.tire for row in result.contact_events]),
        "contact_event_transition": np.asarray([row.transition for row in result.contact_events])})
    np.savez_compressed(destination / "arrays.npz", **arrays)
    model_payload, case_payload = model.model_dump(mode="json"), case.model_dump(mode="json")
    manifest = {"schema_version": 1, "artifact_type": "axle_dynamics_result", "status": result.result.status,
        "package_version": __version__, "arrays_file": "arrays.npz",
        "model": model_payload, "case": case_payload,
        "model_sha256": canonical_hash(model_payload), "case_sha256": canonical_hash(case_payload),
        "time_grid": {"count": len(result.times_s), "values": result.times_s.tolist()},
        "channels": {kind: list(getattr(result, kind+"_names")) for kind in
            ("body", "constraint", "spring", "damper", "bump_stop", "bushing", "anti_roll_bar", "tire")},
        "layouts": {name: list(columns) for name, columns in (
            ("body_state", BODY_STATE_COLUMNS), ("constraint_wrench", CONSTRAINT_WRENCH_COLUMNS),
            ("spring_output", SPRING_OUTPUT_COLUMNS), ("damper_output", DAMPER_OUTPUT_COLUMNS),
            ("bump_stop_output", BUMP_STOP_OUTPUT_COLUMNS), ("bushing_output", BUSHING_OUTPUT_COLUMNS),
            ("anti_roll_output", ANTI_ROLL_OUTPUT_COLUMNS), ("diagnostics", DIAGNOSTIC_COLUMNS),
            ("tire_output", TIRE_OUTPUT_COLUMNS), ("energy", ENERGY_COLUMNS), ("performance", PERFORMANCE_COLUMNS))}}
    target = destination / "manifest.json"
    target.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return target
