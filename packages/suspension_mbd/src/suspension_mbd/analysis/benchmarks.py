"""Fixed, reproducible K/C performance benchmark definitions."""

from __future__ import annotations

import os
import platform
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .. import __version__
from ..io import canonical_hash, write_bundle
from ..model import build_front_axle
from ..schema import (
    FrontAxleModel,
    Manifest,
    MassSpec,
    Provenance,
    ResultBundle,
    StateResult,
)
from .c_mode import CModeSolver, CState
from .k_mode import KState
from .sweeps import KGrid, run_c_grid, run_k_grid


@dataclass(frozen=True)
class PerformanceReport:
    """Summary emitted by one fixed benchmark run."""

    name: str
    state_count: int
    converged_count: int
    max_constraint_residual: float
    max_force_residual: float
    elapsed_seconds: float
    hardware: dict[str, str | int | None]
    output_dir: str | None = None

    @property
    def convergence_rate(self) -> float:
        """Return the fraction of states that converged."""
        return self.converged_count / self.state_count if self.state_count else 1.0


def benchmark_model() -> FrontAxleModel:
    """Return the non-proprietary fixed geometry used by both gates."""
    return FrontAxleModel(
        name="benchmark_front_double_wishbone",
        hardpoints={
            "uca_front": [-100, -500, 400],
            "uca_rear": [100, -500, 400],
            "uca_outer": [0, -700, 450],
            "lca_front": [-120, -500, 150],
            "lca_rear": [120, -500, 150],
            "lca_outer": [0, -700, 150],
            "tierod_inner": [100, -400, 250],
            "tierod_outer": [50, -700, 250],
            "wheel_center": [0, -700, 300],
            "rack_center": [0, 0, 250],
        },
        mass=MassSpec(sprung_mass=1000),
    )


def benchmark_grid() -> KGrid:
    """Return the fixed 10x10 work-point grid for k-100/c-6600."""
    return KGrid()


def run_k_100_benchmark(output_dir: str | Path | None = None) -> PerformanceReport:
    """Run the complete 100-state K benchmark and optionally write tables."""
    model = benchmark_model()
    assembly = build_front_axle(model, "K")
    started = time.perf_counter()
    states = run_k_grid(assembly, benchmark_grid())
    bundle = _bundle_from_k(model, states)
    if output_dir is not None:
        write_bundle(bundle, output_dir)
    return _report("k-100", states, started, output_dir)


def run_c_6600_benchmark(output_dir: str | Path | None = None) -> PerformanceReport:
    """Run 100 cached K references and six 11-level C paths per point."""
    model = benchmark_model()
    assembly = build_front_axle(model, "K")
    started = time.perf_counter()
    # This throughput benchmark exercises table generation and cache behavior;
    # it intentionally requests the nonphysical proxy and is not an accuracy gate.
    states = run_c_grid(
        assembly,
        benchmark_grid(),
        c_solver=CModeSolver(np.eye(6) * 1e-3),
    )
    bundle = _bundle_from_c(model, states)
    if output_dir is not None:
        write_bundle(bundle, output_dir)
    return _report("c-6600", states, started, output_dir)


def _hardware() -> dict[str, str | int | None]:
    return {
        "platform": platform.platform(),
        "processor": platform.processor() or None,
        "physical_or_logical_cpus": os.cpu_count(),
        "python": platform.python_version(),
    }


def _report(
    name: str,
    states: tuple[KState, ...] | tuple[CState, ...],
    started: float,
    output_dir: str | Path | None,
) -> PerformanceReport:
    k_states = [item for item in states if isinstance(item, KState)]
    converged = [item.equilibrium.converged for item in k_states]
    constraints = [float(item.equilibrium.constraint_residual) for item in k_states]
    forces = [float(item.equilibrium.force_residual) for item in k_states]
    if not k_states:
        converged = [True] * len(states)
    return PerformanceReport(
        name=name,
        state_count=len(states),
        converged_count=sum(converged),
        max_constraint_residual=max(constraints, default=0.0),
        max_force_residual=max(forces, default=0.0),
        elapsed_seconds=time.perf_counter() - started,
        hardware=_hardware(),
        output_dir=str(output_dir) if output_dir is not None else None,
    )


def _provenance(model: FrontAxleModel, case_name: str) -> Provenance:
    return Provenance(
        package_version=__version__,
        model_hash=canonical_hash(model.model_dump(mode="json")),
        case_hash=canonical_hash({"name": case_name, "schema_version": 1}),
    )


def _bundle_from_k(model: FrontAxleModel, states: tuple[KState, ...]) -> ResultBundle:
    rows = tuple(
        StateResult(
            state_id=item.case_id,
            mode="K",
            drives={
                "wheel_travel_left": item.wheel_travel_left,
                "wheel_travel_right": item.wheel_travel_right,
                "rack_displacement": item.rack_displacement,
            },
            metrics=item.metrics,
            tire_compression=item.tire_compression,
            constraint_residual=item.equilibrium.constraint_residual,
            force_residual=item.equilibrium.force_residual,
            moment_residual=item.equilibrium.moment_residual,
            converged=item.equilibrium.converged,
        )
        for item in states
    )
    return ResultBundle(
        manifest=Manifest(
            run_id=f"benchmark-k-{canonical_hash([row.state_id for row in rows])[:16]}",
            mode="K",
            state_count=len(rows),
            provenance=_provenance(model, "k-100"),
        ),
        states=rows,
    )


def _bundle_from_c(model: FrontAxleModel, states: tuple[CState, ...]) -> ResultBundle:
    rows = tuple(
        StateResult(
            state_id=item.case_id,
            mode="C",
            external_loads={"left": item.load_left, "right": item.load_right},
            metrics=item.c_minus_k,
            converged=True,
        )
        for item in states
    )
    return ResultBundle(
        manifest=Manifest(
            run_id=f"benchmark-c-{canonical_hash([row.state_id for row in rows])[:16]}",
            mode="C",
            state_count=len(rows),
            provenance=_provenance(model, "c-6600"),
        ),
        states=rows,
    )
