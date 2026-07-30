"""YAML loader for weakly coupled vehicle sweep inputs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict, field_validator


class ValueSweepSpec(BaseModel):
    """One scalar sweep dimension."""

    model_config = ConfigDict(frozen=True)

    start: float | None = None
    stop: float | None = None
    steps: int | None = None
    values: Sequence[float] | None = None

    @field_validator("steps")
    @classmethod
    def check_steps(cls, value: int | None) -> int | None:
        if value is not None and value < 1:
            raise ValueError("steps must be >= 1")
        return value

    def expand_values(self) -> list[float]:
        """Expand explicit values or start/stop/steps into floats."""
        if self.values is not None:
            return [float(value) for value in self.values]
        if self.start is None or self.stop is None or self.steps is None:
            raise ValueError("must specify either values or start, stop, and steps")
        return [
            float(value)
            for value in np.linspace(float(self.start), float(self.stop), self.steps)
        ]


class CoupledSweepFile(BaseModel):
    """Schema for a weakly coupled vehicle sweep file."""

    model_config = ConfigDict(frozen=True)

    version: int
    wheel_travel: ValueSweepSpec
    pitman_angle: ValueSweepSpec

    @field_validator("version")
    @classmethod
    def check_version(cls, value: int) -> int:
        if value != 1:
            raise ValueError(f"Unsupported coupled sweep version: {value}")
        return value


@dataclass(frozen=True)
class CoupledSweepConfig:
    """Expanded weakly coupled sweep values."""

    wheel_travel_values: list[float]
    pitman_angle_values: list[float]


def parse_coupled_sweep_file(path: Path) -> CoupledSweepConfig:
    """Parse a coupled sweep YAML file."""
    try:
        raw_data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"Coupled sweep file not found: {path}")
    except yaml.YAMLError as exc:
        raise ValueError(f"Error parsing coupled sweep YAML: {exc}") from exc

    if not isinstance(raw_data, dict):
        raise ValueError("Coupled sweep file must contain a YAML mapping")

    try:
        file_spec = CoupledSweepFile.model_validate(raw_data)
        return CoupledSweepConfig(
            wheel_travel_values=file_spec.wheel_travel.expand_values(),
            pitman_angle_values=file_spec.pitman_angle.expand_values(),
        )
    except Exception as exc:
        raise ValueError(f"Invalid coupled sweep specification: {exc}") from exc
