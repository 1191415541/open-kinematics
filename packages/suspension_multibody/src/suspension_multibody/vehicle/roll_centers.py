"""Roll-center reports over explicitly named native measurement declarations."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass

import numpy as np

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
        readings = result._roll_center_primitives(declaration)[sample]
        if len(readings) != 2:
            raise ValueError("roll-center report requires exactly two contact frames")
        patches, slopes = readings[:, :3], readings[:, 3].copy()
        if instant_center_engine is not None:
            for index, frame in enumerate(declaration["contact_frames"]):
                instant = np.asarray(instant_center_engine(result, frame))
                run = patches[index, 1]-instant[0]
                if abs(run) <= 1e-12:
                    raise ValueError("instant center has no lateral lever arm")
                slopes[index] = (instant[1]-patches[index, 2])/run
        lateral_force = -2.
        roll_moment = float(patches[:, 1]@slopes)
        reports[placement] = RollCenterResult(placement,
            np.array([patches[:, 1].mean(), -roll_moment/lateral_force]),
            patches[0].copy(), float(slopes[0]), patches[1].copy(), float(slopes[1]),
            lateral_force, roll_moment)
    return reports


__all__ = ["RollCenterResult", "compute_vehicle_roll_centers"]
