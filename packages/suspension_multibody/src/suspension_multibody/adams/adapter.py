"""Unattended Adams/Car validation and numeric comparison adapter."""

from __future__ import annotations

import csv
import inspect
import json
import math
import os
import shlex
import subprocess
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias, cast

from .probe import AdamsProfile

NumericGroups: TypeAlias = Mapping[str, Mapping[str, float]]
Runner: TypeAlias = Callable[..., object] | str | Path | Sequence[str]

REQUIRED_GROUPS = ("K_geometry", "C_compliance", "static_load")
MIN_GROUP_FIELDS = {"K_geometry": 3, "C_compliance": 2, "static_load": 2}
REQUIRED_FIELDS = {
    "time",
    "lcam",
    "ltoe",
    "rcam",
    "rtoe",
    "lf_wc_rise",
    "rf_wc_rise",
    "lspring_force",
    "rspring_force",
    "lfcam",
    "rfcam",
    "steering_displacement",
}


@dataclass(frozen=True)
class Tolerance:
    """Absolute/relative comparison tolerance for one physical quantity kind."""

    absolute: float
    relative: float
    unit: str

    def limit(self, reference: float) -> float:
        return max(self.absolute, self.relative * abs(reference))


@dataclass(frozen=True)
class RunnerExecution:
    """Non-proprietary record of an external runner invocation."""

    configured: bool
    succeeded: bool
    command: tuple[str, ...] = ()
    returncode: int | None = None
    output_path: str | None = None
    error: str | None = None


@dataclass(frozen=True)
class SmokeResult:
    """Result returned by profile validation and numeric comparison."""

    ok: bool
    message: str
    profile: AdamsProfile | None = None
    output_path: str | None = None
    report: Mapping[str, object] | None = None


class AdamsBatchAdapter:
    """
    Run an Adams comparison through an explicit external runner contract.

    A runner receives a JSON request path and an output directory. It must write
    either ``adams_results.json`` or ``adams_results.csv`` in that directory, or
    return a path/mapping containing the numeric results. JSON results use the
    following non-proprietary shape::

        {"groups": {"K_geometry": {"camber_deg": 1.2},
                    "C_compliance": {"wheel_rate": 2.3},
                    "static_load": {"spring_force": 100.0}}}

    The same shape is accepted for the reference values. A callable runner may
    accept ``(profile, request_path, output_dir)``, ``(request_path,
    output_dir)``, ``(request_path)`` or no arguments. A command runner receives
    the request path and output directory as its final two arguments.
    """

    def __init__(
        self,
        profile: AdamsProfile,
        runner: Runner | None = None,
        *,
        tolerances: Mapping[str, Tolerance | Sequence[float]] | None = None,
        force_absolute_tolerance: float = 1.0,
        moment_absolute_tolerance: float = 10.0,
        compliance_absolute_tolerance: float = 1e-9,
    ) -> None:
        if force_absolute_tolerance < 0 or moment_absolute_tolerance < 0:
            raise ValueError(
                "force and moment absolute tolerances must be non-negative"
            )
        if compliance_absolute_tolerance < 0:
            raise ValueError("compliance absolute tolerance must be non-negative")
        defaults: dict[str, Tolerance] = {
            "position": Tolerance(0.1, 0.002, "mm"),
            "angle": Tolerance(0.02, 0.005, "deg"),
            "force": Tolerance(force_absolute_tolerance, 0.02, "N"),
            "moment": Tolerance(moment_absolute_tolerance, 0.02, "N*mm"),
            "compliance": Tolerance(compliance_absolute_tolerance, 0.02, "derived"),
        }
        if tolerances:
            for kind, value in tolerances.items():
                if isinstance(value, Tolerance):
                    defaults[kind] = value
                else:
                    values = tuple(value)
                    if len(values) != 2:
                        raise ValueError(
                            f"tolerance for {kind!r} must be (absolute, relative)"
                        )
                    defaults[kind] = Tolerance(
                        float(values[0]),
                        float(values[1]),
                        defaults.get(kind, Tolerance(0, 0, "")).unit,
                    )
        self.profile = profile
        self.runner = runner
        self.tolerances = defaults

    def smoke(self, output_dir: str | Path | None = None) -> SmokeResult:
        """Export discovered version and report-field metadata."""
        if not self.profile.available:
            return SmokeResult(False, self.profile.message, self.profile)
        destination = _destination(output_dir, "suspension_multibody_adams_smoke_")
        output = destination / "adams_smoke.json"
        payload = {
            "profile": self.profile.name,
            "version": self.profile.version,
            "template_id": self.profile.template_id,
            "subsystem_id": self.profile.subsystem_id,
            "export_fields": self.profile.export_fields,
            "execution": "version_probe_and_report_dictionary",
        }
        output.write_text(
            json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8"
        )
        return SmokeResult(
            True,
            f"Adams smoke mapping exported to {output}",
            self.profile,
            str(output),
            payload,
        )

    def full(
        self,
        output_dir: str | Path | None = None,
        *,
        reference: str | Path | Mapping[str, object] | None = None,
        runner: Runner | None = None,
    ) -> SmokeResult:
        """
        Run installation checks and compare all required numeric result groups.

        ``reference`` is deliberately explicit: accepting an installed profile
        without a numerical baseline would make the accuracy gate a false pass.
        The reference can be a JSON/CSV path or an in-memory mapping. When no
        runner is supplied, ``SUSPENSION_MULTIBODY_ADAMS_RUNNER`` is used as a command.
        """
        destination = _destination(output_dir, "suspension_multibody_adams_full_")
        required_fields = set(REQUIRED_FIELDS)
        export_fields = set(self.profile.export_fields)
        missing_fields = sorted(required_fields - export_fields)
        paths = {
            "executable": Path(self.profile.executable or ""),
            "database": Path(self.profile.database_path or ""),
            "template": Path(self.profile.database_path or "")
            / "templates.tbl"
            / str(self.profile.template_id or ""),
            "subsystem": Path(self.profile.database_path or "")
            / "subsystems.tbl"
            / str(self.profile.subsystem_id or ""),
            "report_dictionary": Path(self.profile.report_dictionary or ""),
        }
        missing_paths = sorted(
            name
            for name, path in paths.items()
            if not path.is_file() and name != "database"
        )
        if not paths["database"].is_dir():
            missing_paths.append("database")
        checks: dict[str, bool] = {
            "profile_available": self.profile.available,
            "version": self.profile.version == "2024.1",
            "license_probe": self.profile.license_probe == "passed",
            "required_report_fields": not missing_fields,
            "installed_assets": not missing_paths,
        }

        reference_groups: dict[str, dict[str, float]] = {}
        reference_error: str | None = None
        if reference is not None:
            try:
                reference_groups = _read_numeric_groups(reference)
            except (
                OSError,
                TypeError,
                ValueError,
                json.JSONDecodeError,
                csv.Error,
            ) as exc:
                reference_error = f"reference result is invalid: {exc}"
        else:
            reference_error = "numeric reference is required for the full gate"

        request_path = destination / "adams_run_request.json"
        request = {
            "profile": self.profile.name,
            "version": self.profile.version,
            "template_id": self.profile.template_id,
            "subsystem_id": self.profile.subsystem_id,
            "required_groups": list(REQUIRED_GROUPS),
            "required_report_fields": sorted(required_fields),
            "reference_groups": reference_groups,
            "output_file": str(destination / "adams_results.json"),
        }
        request_path.write_text(
            json.dumps(request, indent=2, sort_keys=True), encoding="utf-8"
        )

        execution = RunnerExecution(False, False, error="runner was not invoked")
        actual_groups: dict[str, dict[str, float]] = {}
        actual_error: str | None = None
        if all(checks.values()) and reference_error is None:
            execution, actual_source, actual_error = self._run_runner(
                destination, request_path, runner if runner is not None else self.runner
            )
            if actual_error is None and actual_source is not None:
                try:
                    actual_groups = _read_numeric_groups(actual_source)
                except (
                    OSError,
                    TypeError,
                    ValueError,
                    json.JSONDecodeError,
                    csv.Error,
                ) as exc:
                    actual_error = f"Adams numeric result is invalid: {exc}"

        comparisons, comparison_checks, comparison_error = _compare_groups(
            reference_groups,
            actual_groups,
            self.tolerances,
        )
        if reference_error:
            comparison_error = reference_error
        elif actual_error:
            comparison_error = actual_error
        checks.update(comparison_checks)
        checks["runner"] = execution.succeeded
        checks["numeric_results"] = not actual_error and bool(actual_groups)
        checks["comparison"] = not comparison_error and all(
            comparison_checks.get(group, False) for group in REQUIRED_GROUPS
        )

        report: dict[str, object] = {
            "profile": self.profile.name,
            "version": self.profile.version,
            "template_id": self.profile.template_id,
            "subsystem_id": self.profile.subsystem_id,
            "execution": "full_profile_and_equivalence_contract",
            "checks": checks,
            "missing_fields": missing_fields,
            "missing_paths": missing_paths,
            "runner": {
                "configured": execution.configured,
                "succeeded": execution.succeeded,
                "command": list(execution.command),
                "returncode": execution.returncode,
                "output_path": execution.output_path,
                "error": execution.error,
            },
            "comparison_contract": {
                "position_tolerance": "max(0.1 mm, 0.2%)",
                "angle_tolerance": "max(0.02 deg, 0.5%)",
                "force_tolerance": "max(frozen absolute tolerance, 2%)",
                "moment_tolerance": "max(frozen absolute tolerance, 2%)",
                "required_groups": list(REQUIRED_GROUPS),
                "tolerances": {
                    kind: {
                        "absolute": tolerance.absolute,
                        "relative": tolerance.relative,
                        "unit": tolerance.unit,
                    }
                    for kind, tolerance in self.tolerances.items()
                },
            },
            "comparisons": comparisons,
            "comparison_error": comparison_error,
            "export_fields_count": len(self.profile.export_fields),
        }
        output = destination / "adams_full_validation.json"
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
        )
        ok = all(checks.values())
        message = (
            f"Adams full validation report exported to {output}"
            if ok
            else (
                f"Adams full validation failed"
                f" ({comparison_error or 'installation/profile checks failed'});"
                f" report exported to {output}"
            )
        )
        return SmokeResult(ok, message, self.profile, str(output), report)

    def _run_runner(
        self,
        destination: Path,
        request_path: Path,
        runner: Runner | None,
    ) -> tuple[RunnerExecution, object | None, str | None]:
        configured: Runner | None = runner
        if configured is None:
            command = os.environ.get("SUSPENSION_MULTIBODY_ADAMS_RUNNER")
            configured = command if command else None
        if configured is None:
            execution = RunnerExecution(
                configured=False,
                succeeded=False,
                error="numeric runner is not configured",
            )
            return execution, None, execution.error
        try:
            if callable(configured):
                result = _invoke_callable(
                    configured, self.profile, request_path, destination
                )
                payload, path = _runner_result(result, destination)
                execution = RunnerExecution(
                    True, True, output_path=str(path) if path else None
                )
                return execution, payload if payload is not None else path, None

            command_args = _command_args(configured)
            command = tuple(command_args)
            completed = _run_command(command_args, request_path, destination)
            stdout = completed.stdout or ""
            stderr = completed.stderr or ""
            (destination / "adams_runner.stdout.txt").write_text(
                stdout, encoding="utf-8", errors="replace"
            )
            (destination / "adams_runner.stderr.txt").write_text(
                stderr, encoding="utf-8", errors="replace"
            )
            if completed.returncode != 0:
                error = f"Adams runner exited with code {completed.returncode}"
                return (
                    RunnerExecution(
                        True, False, command, completed.returncode, error=error
                    ),
                    None,
                    error,
                )
            payload, path = _runner_result(None, destination)
            execution = RunnerExecution(
                True, True, command, completed.returncode, str(path) if path else None
            )
            return execution, payload if payload is not None else path, None
        except Exception as exc:
            error = f"Adams runner failed: {exc}"
            return RunnerExecution(True, False, error=error), None, error


def _destination(output_dir: str | Path | None, prefix: str) -> Path:
    destination = (
        Path(output_dir)
        if output_dir is not None
        else Path(tempfile.mkdtemp(prefix=prefix))
    )
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def _invoke_callable(
    runner: Callable[..., object],
    profile: AdamsProfile,
    request_path: Path,
    destination: Path,
) -> object:
    """Support the documented runner signatures without requiring wrappers."""
    try:
        parameters = inspect.signature(runner).parameters.values()
        positional = [
            parameter
            for parameter in parameters
            if parameter.kind
            in (
                inspect.Parameter.POSITIONAL_ONLY,
                inspect.Parameter.POSITIONAL_OR_KEYWORD,
            )
        ]
        has_varargs = any(
            parameter.kind == inspect.Parameter.VAR_POSITIONAL
            for parameter in parameters
        )
    except (TypeError, ValueError):
        positional = []
        has_varargs = True
    if has_varargs or len(positional) >= 3:
        return runner(profile, request_path, destination)
    if len(positional) == 2:
        return runner(request_path, destination)
    if len(positional) == 1:
        return runner(request_path)
    return runner()


def _command_args(runner: str | Path | Sequence[str]) -> list[str]:
    if isinstance(runner, Path):
        return [str(runner)]
    if isinstance(runner, str):
        args = shlex.split(runner, posix=False)
    else:
        args = [str(value) for value in runner]
    if not args:
        raise ValueError("numeric runner command is empty")
    return args


def _run_command(
    command: Sequence[str], request_path: Path, destination: Path
) -> subprocess.CompletedProcess[str]:
    args = list(command) + [str(request_path), str(destination)]
    environment = os.environ.copy()
    environment.update(
        {
            "SUSPENSION_MULTIBODY_ADAMS_REQUEST": str(request_path),
            "SUSPENSION_MULTIBODY_ADAMS_OUTPUT": str(destination),
        }
    )
    if Path(command[0]).suffix.lower() in {".bat", ".cmd"}:
        # Windows resolves batch files through cmd; list2cmdline preserves quoted
        # Program Files paths without adding a second nested cmd invocation.
        command_line = subprocess.list2cmdline(args)
        return subprocess.run(
            command_line,
            cwd=destination,
            env=environment,
            capture_output=True,
            text=True,
            timeout=300,
            check=False,
            shell=True,
        )
    return subprocess.run(
        args,
        cwd=destination,
        env=environment,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )


def _runner_result(
    result: object, destination: Path
) -> tuple[object | None, Path | None]:
    if isinstance(result, Mapping):
        return result, None
    if isinstance(result, (str, Path)):
        path = Path(result)
        if not path.is_absolute():
            path = destination / path
        return None, path
    if isinstance(result, Sequence) and not isinstance(result, (str, bytes, bytearray)):
        values = list(result)
        if len(values) == 2 and isinstance(values[0], (str, Path)):
            path = Path(values[0])
            if not path.is_absolute():
                path = destination / path
            return values[1], path
    for candidate in (
        destination / "adams_results.json",
        destination / "adams_results.csv",
    ):
        if candidate.is_file():
            return None, candidate
    return None, None


def _read_numeric_groups(source: object) -> dict[str, dict[str, float]]:
    if isinstance(source, Mapping):
        payload: object = source
    else:
        path = Path(source)  # type: ignore[arg-type]
        if not path.is_file():
            raise FileNotFoundError(path)
        if path.suffix.lower() == ".csv":
            return _read_csv_groups(path)
        payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise TypeError("numeric result must be a JSON object")
    payload_mapping = cast(Mapping[str, object], payload)
    groups_obj = payload_mapping.get(
        "groups", payload_mapping.get("values", payload_mapping)
    )
    if not isinstance(groups_obj, Mapping):
        raise TypeError("numeric result groups must be an object")
    has_group_container = "groups" in payload_mapping or "values" in payload_mapping
    has_known_group = any(group in groups_obj for group in REQUIRED_GROUPS)
    if not has_group_container and not has_known_group:
        return _normalise_flat_values(cast(Mapping[str, object], groups_obj))
    groups: dict[str, dict[str, float]] = {}
    flat_values: dict[str, object] = {}
    for group_name, values in groups_obj.items():
        if (
            isinstance(group_name, str)
            and isinstance(values, (int, float))
            and not isinstance(values, bool)
        ):
            flat_values[group_name] = values
            continue
        if not isinstance(group_name, str) or not isinstance(values, Mapping):
            continue
        group: dict[str, float] = {}
        for field, value in values.items():
            number = _coerce_numeric(group_name, str(field), value)
            group[str(field)] = number
        groups[group_name] = group
    if flat_values:
        for group_name, values in _normalise_flat_values(flat_values).items():
            groups.setdefault(group_name, {}).update(values)
    return groups


def _read_csv_groups(path: Path) -> dict[str, dict[str, float]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = set(reader.fieldnames or ())
    if not rows:
        raise ValueError("numeric CSV result is empty")
    groups: dict[str, dict[str, float]] = {}
    if not {"group", "metric_group", "field", "metric"} & fieldnames:
        flat_values: dict[str, object] = {}
        for row in rows:
            for field, value in row.items():
                if field is None or value is None or not value.strip():
                    continue
                if field in flat_values:
                    raise ValueError(f"duplicate CSV result field {field}")
                flat_values[field] = value
        return _normalise_flat_values(flat_values)
    for row in rows:
        group = row.get("group") or row.get("metric_group")
        field = row.get("field") or row.get("metric")
        value = row.get("value") or row.get("actual")
        if not group or not field or value is None:
            raise ValueError("CSV result requires group, field and value columns")
        raw_value: object = value
        unit = row.get("unit")
        if unit:
            raw_value = {"value": value, "unit": unit}
        if field in groups.setdefault(group, {}):
            raise ValueError(f"duplicate CSV result field {group}.{field}")
        groups[group][field] = _coerce_numeric(group, field, raw_value)
    return groups


def _group_flat_values(values: Mapping[str, object]) -> dict[str, dict[str, object]]:
    groups: dict[str, dict[str, object]] = {}
    for field, value in values.items():
        name = field.lower()
        if any(token in name for token in ("compliance", "rate", "flexibility")):
            group = "C_compliance"
        elif any(
            token in name
            for token in (
                "camber",
                "toe",
                "caster",
                "wc_rise",
                "wheel_center",
                "steering_displacement",
                "steering_angle",
                "rack",
            )
        ):
            group = "K_geometry"
        elif any(
            token in name
            for token in ("force", "moment", "torque", "spring", "damper", "load")
        ):
            group = "static_load"
        else:
            continue
        groups.setdefault(group, {})[field] = value
    return groups


def _normalise_flat_values(values: Mapping[str, object]) -> dict[str, dict[str, float]]:
    return {
        group: {
            field: _coerce_numeric(group, field, value)
            for field, value in fields.items()
        }
        for group, fields in _group_flat_values(values).items()
    }


def _coerce_numeric(group: str, field: str, value: object) -> float:
    unit: str | None = None
    raw_value = value
    if isinstance(value, Mapping):
        datum = cast(Mapping[str, object], value)
        raw_value = datum.get("value")
        unit_value = datum.get("unit")
        if not isinstance(unit_value, str) or not unit_value.strip():
            raise ValueError(f"{group}.{field} requires a non-empty unit")
        unit = unit_value.strip()
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float, str)):
        raise TypeError(f"{group}.{field} must be numeric")
    try:
        number = float(raw_value)
    except ValueError as exc:
        raise TypeError(f"{group}.{field} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{group}.{field} must be finite")
    if unit is None:
        return number
    kind = _metric_kind(group, field)
    factor = _unit_factor(kind, unit)
    if factor is None:
        raise ValueError(f"unsupported unit {unit!r} for {group}.{field}")
    return number * factor


def _unit_factor(kind: str, unit: str) -> float | None:
    normalized = unit.strip().lower().replace(" ", "")
    if kind == "position":
        return {"mm": 1.0, "m": 1000.0}.get(normalized)
    if kind == "angle":
        return {"deg": 1.0, "degree": 1.0, "rad": 180.0 / math.pi}.get(normalized)
    if kind == "force":
        return {"n": 1.0, "kn": 1000.0}.get(normalized)
    if kind == "moment":
        return {"n*mm": 1.0, "nmm": 1.0, "n*m": 1000.0, "nm": 1000.0}.get(normalized)
    if kind == "compliance":
        return {"mm/n": 1.0, "m/n": 1000.0, "deg/n": 1.0, "rad/n": 180.0 / math.pi}.get(
            normalized
        )
    return None


def _metric_kind(group: str, field: str) -> str:
    name = field.lower()
    if group == "K_geometry":
        if any(token in name for token in ("camber", "toe", "caster", "angle")):
            return "angle"
        return "position"
    if group == "C_compliance":
        return "compliance"
    if any(token in name for token in ("moment", "torque", "mx", "my", "mz")):
        return "moment"
    return "force"


def _compare_groups(
    reference: Mapping[str, Mapping[str, float]],
    actual: Mapping[str, Mapping[str, float]],
    tolerances: Mapping[str, Tolerance],
) -> tuple[dict[str, object], dict[str, bool], str | None]:
    comparisons: dict[str, object] = {}
    checks: dict[str, bool] = {}
    errors: list[str] = []
    for group in REQUIRED_GROUPS:
        expected = reference.get(group)
        observed = actual.get(group)
        if expected is None:
            checks[group] = False
            errors.append(f"reference is missing group {group}")
            comparisons[group] = {"passed": False, "error": "missing reference group"}
            continue
        minimum_fields = MIN_GROUP_FIELDS[group]
        if len(expected) < minimum_fields:
            checks[group] = False
            error = f"reference group {group} has fewer than {minimum_fields} fields"
            errors.append(error)
            comparisons[group] = {"passed": False, "error": error}
            continue
        if observed is None:
            checks[group] = False
            errors.append(f"Adams result is missing group {group}")
            comparisons[group] = {"passed": False, "error": "missing Adams group"}
            continue
        fields: dict[str, object] = {}
        group_ok = True
        extra_fields = sorted(set(observed) - set(expected))
        if extra_fields:
            group_ok = False
            errors.append(f"Adams result has unknown fields in {group}: {extra_fields}")
        for field, expected_value in expected.items():
            if field not in observed:
                group_ok = False
                errors.append(f"Adams result is missing {group}.{field}")
                fields[field] = {"passed": False, "error": "missing field"}
                continue
            observed_value = observed[field]
            kind = _metric_kind(group, field)
            tolerance = tolerances[kind]
            absolute_error = abs(observed_value - expected_value)
            limit = tolerance.limit(expected_value)
            relative_error = (
                absolute_error / abs(expected_value)
                if expected_value != 0
                else (0.0 if absolute_error == 0 else math.inf)
            )
            passed = math.isfinite(absolute_error) and absolute_error <= limit
            group_ok &= passed
            if not passed:
                errors.append(
                    f"{group}.{field} error {absolute_error:g} exceeds {limit:g}"
                )
            fields[field] = {
                "reference": expected_value,
                "adams": observed_value,
                "absolute_error": absolute_error,
                "relative_error": relative_error,
                "tolerance": limit,
                "kind": kind,
                "unit": tolerance.unit,
                "passed": passed,
            }
        checks[group] = group_ok
        comparisons[group] = {
            "passed": checks[group],
            "fields": fields,
            "unknown_fields": extra_fields,
        }
    return comparisons, checks, "; ".join(errors) if errors else None
