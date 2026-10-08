"""Explicit external-runner gates for axle and vehicle time histories."""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias, cast

from ..api import validate
from ..authoring import CaseDocument, migrate_v1_kc_case
from ..authoring.signals import motion, time_grid, wrenches_at_time
from ..report.kc_evidence import frame_fields
from ..results.envelope import ResultEnvelope
from ..schema import DynamicCaseSpec, UnitSystem
from ..schema.model import AxleDeclaration
from ..simulation.runner import run_compiled
from .probe import AdamsProfile
from .time_domain import (
    TimeHistory,
    TimeHistoryTolerance,
    compare_time_histories,
    read_time_history,
)

TimeDomainRunner: TypeAlias = Callable[[AdamsProfile, Path, Path], object]

AXLE_RESPONSE_CHANNELS = (
    "left_wheel_center_x",
    "left_wheel_center_y",
    "left_wheel_center_z",
    "left_camber_deg",
    "left_toe_deg",
    "right_wheel_center_x",
    "right_wheel_center_y",
    "right_wheel_center_z",
    "right_camber_deg",
    "right_toe_deg",
)


@dataclass(frozen=True)
class TimeDomainGateResult:
    """Result of an independently executed time-history acceptance gate."""

    ok: bool
    message: str
    output_path: str
    report: Mapping[str, object]


class AdamsTimeDomainAdapter:
    """Run an explicit external runner and compare it to a package history."""

    def __init__(self, profile: AdamsProfile, runner: TimeDomainRunner) -> None:
        self.profile = profile
        self.runner = runner

    def validate(
        self,
        *,
        analysis: str,
        model: AxleDeclaration,
        case: DynamicCaseSpec,
        reference: TimeHistory,
        tolerances: Mapping[str, TimeHistoryTolerance],
        output_dir: str | Path | None = None,
    ) -> TimeDomainGateResult:
        """Execute the runner without exposing package reference values to it."""
        destination = _destination(output_dir)
        request_path = destination / "adams_time_domain_request.json"
        request = {
            "contract": "adams-time-domain-runner-v1",
            "analysis": analysis,
            "profile": self.profile.name,
            "model": model.model_dump(mode="json"),
            "case": case.model_dump(mode="json"),
            "channels": sorted(reference.channels),
            "output_file": str(destination / "adams_time_history.json"),
        }
        request_path.write_text(
            json.dumps(request, indent=2, sort_keys=True), encoding="utf-8"
        )
        report_path = destination / "adams_time_domain_report.json"
        if not self.profile.available:
            report = {
                "contract": "adams-time-domain-gate-v1",
                "analysis": analysis,
                "passed": False,
                "error": self.profile.message,
                "runner_invoked": False,
            }
            report_path.write_text(
                json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
            )
            return TimeDomainGateResult(
                False, self.profile.message, str(report_path), report
            )

        try:
            source = self.runner(self.profile, request_path, destination)
            actual = _runner_history(source, destination)
            comparison = compare_time_histories(reference, actual, tolerances)
            report: dict[str, object] = {
                "contract": "adams-time-domain-gate-v1",
                "analysis": analysis,
                "passed": comparison["passed"],
                "runner_invoked": True,
                "request_path": str(request_path),
                "comparison": comparison,
            }
        except Exception as exc:
            report = {
                "contract": "adams-time-domain-gate-v1",
                "analysis": analysis,
                "passed": False,
                "runner_invoked": True,
                "error": str(exc),
            }
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
        )
        passed = bool(report["passed"])
        return TimeDomainGateResult(
            passed,
            (
                f"{analysis} Adams time-domain gate passed"
                if passed
                else f"{analysis} Adams time-domain gate failed"
            ),
            str(report_path),
            report,
        )


def _axle_time_history(model: AxleDeclaration, case: DynamicCaseSpec, channels: Sequence[str]) -> TimeHistory:
    """Sample independent equilibria through the ordinary document pipeline."""
    if case.solver.integrator != "quasi_static":
        raise ValueError("axle time-domain reference requires independent quasi-static equilibria")
    document, study = migrate_v1_kc_case(model, mode="K", name=case.name,
        wheel_values_mm=(0,), rack_values_mm=(0,), drive_mode="kinematics")
    wheel = next(entry for entry in document.entries if entry.functional_role == "wheel")
    owners = {row["name"]: entry.ref+"."+row["name"] for entry in document.entries
        for row in entry.subsystem.template.payload["bodies"]}
    times = time_grid(case)
    millimetres = 1 if case.units == UnitSystem.ENGINEERING else 1000
    values = {name: [] for name in channels}
    for time in times:
        payload = study.to_payload()
        axes = [{"coordinate": "kc_rig.sub.json.wheel_drive_"+side,
            "values_mm": [motion(case, target).value_at(time)*millimetres]}
            for side, target in (("L", "wheel_travel_left"), ("R", "wheel_travel_right"))]
        if "rack" in owners:
            axes.append({"coordinate": "kc_rig.sub.json.rack_drive", "values_mm": [motion(case, "rack").value_at(time)*millimetres]})
        payload["excitation"]["k"] = {"axes": axes, "drive": "wheel_center",
            "body_wrench": [{"body": owners[name], "wrench": [*wrench[:3], *(wrench[3:]*millimetres)]}
                for name, wrench in wrenches_at_time(case, time).items()]}
        result = run_compiled(validate(document, CaseDocument(payload))).result
        if not isinstance(result, ResultEnvelope):
            raise TypeError("document simulation must return ResultEnvelope")
        fields = {}
        for side in ("L", "R"):
            fields.update({name.removesuffix("_mm"): value for name, value in
                frame_fields(result.frame_pose(wheel.ref+".wheel_center_"+side)[-1], side=side).items()})
        for name in channels:
            values[name].append(fields[name])
    return TimeHistory(time=times, channels={name: tuple(samples) for name, samples in values.items()})


def validate_axle_time_domain(
    profile: AdamsProfile,
    model: AxleDeclaration,
    case: DynamicCaseSpec,
    *,
    runner: TimeDomainRunner,
    output_dir: str | Path | None = None,
    channels: Sequence[str] = AXLE_RESPONSE_CHANNELS,
) -> TimeDomainGateResult:
    """Validate an axle time trace, including its prescribed motions and wrenches."""
    if case.mode != "axle_dynamic":
        raise ValueError("axle Adams gate requires mode='axle_dynamic'")
    reference = _axle_time_history(model, case, channels)
    return AdamsTimeDomainAdapter(profile, runner).validate(
        analysis="axle_time_domain",
        model=model,
        case=case,
        reference=reference,
        tolerances={
            name: TimeHistoryTolerance(
                absolute=0.02 if name.endswith("_deg") else 0.1,
                peak_relative_percent=1.0,
                rms_relative_percent=1.0,
                phase_ms=10.0,
            )
            for name in channels
        },
        output_dir=output_dir,
    )


def command_time_domain_runner(command: str) -> TimeDomainRunner:
    """Wrap an explicit external Adams time-domain runner command."""
    arguments = shlex.split(command, posix=False)
    if not arguments:
        raise ValueError("time-domain runner command is empty")

    def run(_profile: AdamsProfile, request_path: Path, output_dir: Path) -> None:
        completed = subprocess.run(
            [*arguments, str(request_path), str(output_dir)],
            cwd=output_dir,
            env=os.environ.copy(),
            capture_output=True,
            text=True,
            # Decode explicitly; see `adams/adapter.py`.
            encoding="utf-8",
            errors="replace",
            timeout=600,
            check=False,
        )
        (output_dir / "adams_time_domain_runner.stdout.txt").write_text(
            completed.stdout or "", encoding="utf-8", errors="replace"
        )
        (output_dir / "adams_time_domain_runner.stderr.txt").write_text(
            completed.stderr or "", encoding="utf-8", errors="replace"
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"time-domain Adams runner exited with code {completed.returncode}"
            )

    return run


def _destination(output_dir: str | Path | None) -> Path:
    destination = (
        Path(output_dir)
        if output_dir is not None
        else Path(tempfile.mkdtemp(prefix="suspension_multibody_adams_td_"))
    )
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def _runner_history(source: object, destination: Path) -> TimeHistory:
    if isinstance(source, TimeHistory):
        return source
    if isinstance(source, Mapping):
        return TimeHistory.from_mapping(cast(Mapping[str, object], source))
    if isinstance(source, (str, Path)):
        return read_time_history(source)
    if source is not None:
        raise ValueError("time-domain runner returned an unsupported result")
    for candidate in (
        destination / "adams_time_history.json",
        destination / "adams_time_history.csv",
    ):
        if candidate.is_file():
            return read_time_history(candidate)
    raise ValueError("time-domain runner did not produce an Adams time history")
