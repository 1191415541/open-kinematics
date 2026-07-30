"""Deterministic K-reference cache for C-mode runs."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field

from ..model import FrontAxleAssembly
from .k_mode import KModeSolver, KState


@dataclass
class KReferenceCache:
    """In-memory cache keyed by model and drive settings."""

    entries: dict[str, KState] = field(default_factory=dict)

    @staticmethod
    def key(*, wheel_left: float, wheel_right: float, rack: float, drive: str) -> str:
        payload = json.dumps(
            {
                "wheel_left": wheel_left,
                "wheel_right": wheel_right,
                "rack": rack,
                "drive": drive,
            },
            sort_keys=True,
        ).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    def get_or_solve(
        self,
        assembly: FrontAxleAssembly,
        *,
        wheel_left: float = 0.0,
        wheel_right: float = 0.0,
        rack: float = 0.0,
        drive: str = "wheel_center",
        solver: KModeSolver | None = None,
    ) -> KState:
        key = self.key(
            wheel_left=wheel_left, wheel_right=wheel_right, rack=rack, drive=drive
        )
        if key not in self.entries:
            self.entries[key] = (solver or KModeSolver()).solve(
                assembly,
                wheel_travel_left=wheel_left,
                wheel_travel_right=wheel_right,
                rack_displacement=rack,
                drive=drive,  # type: ignore[arg-type]
                case_id=f"k-ref-{len(self.entries):04d}",
            )
        return self.entries[key]

    def store(self, state: KState, *, drive: str | None = None) -> None:
        """Insert an already solved reference without performing another solve."""
        self.entries[
            self.key(
                wheel_left=state.wheel_travel_left,
                wheel_right=state.wheel_travel_right,
                rack=state.rack_displacement,
                drive=drive or state.drive,
            )
        ] = state
