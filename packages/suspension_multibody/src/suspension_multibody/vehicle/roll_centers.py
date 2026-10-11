"""Roll-center reports over explicitly named native measurement declarations."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..characteristics.centers import (
    LEGACY_LATERAL_FORCE,
    legacy_roll_center_primitives,
    legacy_roll_center_report_values,
)
from ..results.envelope import ResultEnvelope

InstantCentreEngine = Callable[[ResultEnvelope, str], np.ndarray]


@dataclass(frozen=True)
class RollCenterResult:
    axle: str
    center: np.ndarray
    left_contact_patch: np.ndarray
    left_contact_patch_slope: float
    right_contact_patch: np.ndarray
    right_contact_patch_slope: float
    lateral_force: float
    roll_moment: float


def _legacy_readings(result: ResultEnvelope, declaration: Mapping[str, Any]) -> np.ndarray:
    """
    Extract the legacy per-sample readings for this envelope's own result.

    The assembly and the aggregation both live in ``characteristics.centers``;
    this function's whole job is gathering what the envelope already holds and
    handing it over as arrays.  Keeping it outside the envelope is the point of
    R3: the report used to reach into ``result._roll_center_primitives``, a
    private method, which meant the report's contract silently depended on the
    envelope's internals.
    """
    jacobians = np.asarray(result.raw.block("constraint_jacobian"), dtype=float)
    free = tuple(result.raw.metadata["free_bodies"])
    frames = {row["name"]: row for row in result.model.to_document()["frames"]}
    reference = result.frame_pose(declaration.get("reference", "world"))
    selected = {row["id"]: row for row in result.raw.metadata["constraints"]}
    indices = [
        index
        for name in declaration["constraints"]
        for index in range(
            selected[name]["row_offset"],
            selected[name]["row_offset"] + selected[name]["row_count"],
        )
    ]
    contacts = declaration["contact_frames"]
    drives = declaration["drive_frames"]
    bodies: list[str] = []
    for contact, drive in zip(contacts, drives):
        body = frames[drive]["body"]
        if frames[contact]["body"] != body or body not in free:
            raise ValueError("roll-center contact and drive must belong to the same free body")
        bodies.append(body)
    samples = jacobians.shape[0]
    states = np.stack(
        [np.asarray(result.body_state(body)[:samples]) for body in bodies], axis=1
    )
    return legacy_roll_center_primitives(
        jacobians=jacobians,
        selected_rows=np.asarray(indices, dtype=int),
        body_offsets=np.asarray([6 * free.index(body) for body in bodies], dtype=int),
        reference_rotations=np.asarray(reference[:samples, :3, :3], dtype=float),
        reference_origins=np.asarray(reference[:samples, :3, 3], dtype=float),
        body_positions=states[:, :, :3],
        drive_points=np.stack(
            [np.asarray(result.frame_pose(name)[:samples, :3, 3]) for name in drives], axis=1
        ),
        contact_points=np.stack(
            [np.asarray(result.frame_pose(name)[:samples, :3, 3]) for name in contacts], axis=1
        ),
        reference_body_offset=_held_reference_offset(declaration, frames, free),
    )


def _held_reference_offset(
    declaration: Mapping[str, Any],
    frames: Mapping[str, Any],
    free: tuple[str, ...],
) -> int | None:
    """Return the free-body offset the legacy drive pinned, or ``None`` for world."""
    reference_id = declaration.get("reference", "world")
    if reference_id == "world":
        return None
    body = frames[reference_id]["body"]
    if body not in free:
        return None
    return 6 * free.index(body)


def compute_vehicle_roll_centers(
    result: ResultEnvelope, *, measurements: Mapping[str, str], sample: int = 0,
    instant_center_engine: InstantCentreEngine | None = None,
) -> dict[str, RollCenterResult]:
    """Read SI contact slopes from native constraint derivatives, without assembly."""
    declarations = {row["name"]: row for row in result.model.to_document().get("measurements", ())}
    reports = {}
    for placement, name in measurements.items():
        declaration = declarations[name]
        if declaration["type"] != "roll_center":
            raise ValueError(f"measurement {name!r} is not a roll center")
        readings = _legacy_readings(result, declaration)[sample]
        if len(readings) != 2:
            raise ValueError("roll-center report requires exactly two contact frames")
        patches = readings[:, :3]
        slopes = readings[:, 3].copy()
        if instant_center_engine is not None:
            for index, frame in enumerate(declaration["contact_frames"]):
                instant = np.asarray(instant_center_engine(result, frame))
                run = patches[index, 1] - instant[0]
                if abs(run) <= 1e-12:
                    raise ValueError("instant center has no lateral lever arm")
                slopes[index] = (instant[1] - patches[index, 2]) / run
        # The replacement slope is written back into the reading column so the
        # report's aggregation stays the legacy definition's, evaluated on the
        # injected slopes -- the hook changes the input, not the arithmetic.
        adjusted = readings.copy()
        adjusted[:, 3] = slopes
        patch_rows, slope_rows, center, moment = legacy_roll_center_report_values(adjusted)
        reports[placement] = RollCenterResult(
            placement,
            center,
            patch_rows[0].copy(),
            float(slope_rows[0]),
            patch_rows[1].copy(),
            float(slope_rows[1]),
            LEGACY_LATERAL_FORCE,
            moment,
        )
    return reports


__all__ = ["RollCenterResult", "compute_vehicle_roll_centers"]
