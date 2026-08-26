"""Frozen dynamic-axle manifest and explicit channel role bindings."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import yaml
from pydantic import Field

from .. import __version__
from ..axle_dynamics.schema import (
    AxleDynamicsCase,
    AxleDynamicsModel,
    Vec3Tuple,
)
from ..io import canonical_hash
from ..schema.common import StrictModel

AXLE_MANIFEST_CONTRACT = "dynamic-axle-manifest-v1"
AXLE_MANIFEST_GENERATOR = "open-kinematics-dynamic-axle-v1"
FIXTURE_WRENCH_CONVENTION = (
    "fixture wrench on the fixed body is reconstructed from the common public-"
    "grid momentum balance: non-fixture external wrench minus the derivative of "
    "total moving-body linear and angular momentum about the fixture reference "
    "marker; redundant joint multipliers are not used for canonical channels"
)
_CHANNELS_PATH = Path(__file__).with_name("axle_channels.yaml")
_ACCEPTANCE_PATH = Path(__file__).with_name("axle_acceptance.yaml")


class AxleMarkerBinding(StrictModel):
    """One named body-local marker used by the frozen output formulas."""

    body: str = Field(min_length=1)
    point_local_m: Vec3Tuple


class AxleChannelBindings(StrictModel):
    """Explicit model roles; no role is inferred from entity names."""

    sprung_body: str = Field(min_length=1)
    fixture_reference_marker: AxleMarkerBinding
    left_wheel_center_marker: AxleMarkerBinding
    right_wheel_center_marker: AxleMarkerBinding
    left_wheel_spin_joint: str = Field(min_length=1)
    right_wheel_spin_joint: str = Field(min_length=1)
    left_spring: str = Field(min_length=1)
    right_spring: str = Field(min_length=1)
    left_damper: str = Field(min_length=1)
    right_damper: str = Field(min_length=1)
    left_tire: str = Field(min_length=1)
    right_tire: str = Field(min_length=1)


class DynamicAxleManifestSettings(StrictModel):
    """Non-physical metadata required to freeze a comparison manifest."""

    schema_version: int = 1
    role_bindings: AxleChannelBindings
    adams_solver: dict[str, object]
    execution_environment: dict[str, object]
    case_metadata: dict[str, object] = Field(default_factory=dict)


@dataclass(frozen=True)
class DynamicAxleManifest:
    """Canonical manifest content and its SHA-256 identity."""

    payload: Mapping[str, object]
    sha256: str

    def as_dict(self) -> dict[str, object]:
        return {**dict(self.payload), "manifest_sha256": self.sha256}

    @property
    def model(self) -> AxleDynamicsModel:
        return AxleDynamicsModel.model_validate(self.payload["model"])

    @property
    def case(self) -> AxleDynamicsCase:
        return AxleDynamicsCase.model_validate(self.payload["case"])

    @property
    def bindings(self) -> AxleChannelBindings:
        return AxleChannelBindings.model_validate(self.payload["role_bindings"])


def load_axle_channel_contract() -> dict[str, Any]:
    """Load the packaged, frozen channel definitions."""
    return _load_yaml_mapping(_CHANNELS_PATH)


def load_axle_acceptance_contract() -> dict[str, Any]:
    """Load the packaged, frozen acceptance definitions."""
    return _load_yaml_mapping(_ACCEPTANCE_PATH)


def load_dynamic_axle_manifest_settings(
    path: str | Path,
) -> DynamicAxleManifestSettings:
    """Load explicit role, Adams-solver, and execution settings."""
    source = Path(path)
    payload = (
        json.loads(source.read_text(encoding="utf-8"))
        if source.suffix.lower() == ".json"
        else yaml.safe_load(source.read_text(encoding="utf-8"))
    )
    if not isinstance(payload, Mapping):
        raise ValueError("dynamic axle manifest settings root must be an object")
    return DynamicAxleManifestSettings.model_validate(payload)


def validate_axle_channel_bindings(
    model: AxleDynamicsModel,
    bindings: AxleChannelBindings,
) -> None:
    """Validate every semantic role against the closed physical model."""
    bodies = {body.name: body for body in model.bodies}
    joints = {joint.name: joint for joint in model.joints}
    springs = {spring.name: spring for spring in model.springs}
    tires = {tire.name: tire for tire in model.tires}

    if bindings.sprung_body not in bodies:
        raise ValueError(f"unknown sprung_body {bindings.sprung_body!r}")
    if bodies[bindings.sprung_body].fixed:
        raise ValueError("sprung_body must be a moving rigid body")

    markers = {
        "fixture_reference_marker": bindings.fixture_reference_marker,
        "left_wheel_center_marker": bindings.left_wheel_center_marker,
        "right_wheel_center_marker": bindings.right_wheel_center_marker,
    }
    for role, marker in markers.items():
        if marker.body not in bodies:
            raise ValueError(f"{role} references unknown body {marker.body!r}")
        if any(not math.isfinite(value) for value in marker.point_local_m):
            raise ValueError(f"{role} point_local_m must be finite")
    fixture_body = bodies[bindings.fixture_reference_marker.body]
    if not fixture_body.fixed:
        raise ValueError("fixture_reference_marker must belong to a fixed body")

    left_wheel = bindings.left_wheel_center_marker.body
    right_wheel = bindings.right_wheel_center_marker.body
    if left_wheel == right_wheel:
        raise ValueError("left and right wheel-center markers require different bodies")
    if bodies[left_wheel].fixed or bodies[right_wheel].fixed:
        raise ValueError("wheel-center markers must belong to moving bodies")

    for side, tire_name, wheel_body in (
        ("left", bindings.left_tire, left_wheel),
        ("right", bindings.right_tire, right_wheel),
    ):
        tire = tires.get(tire_name)
        if tire is None:
            raise ValueError(f"{side}_tire references unknown tire {tire_name!r}")
        if tire.body != wheel_body:
            raise ValueError(
                f"{side}_tire body {tire.body!r} does not match wheel body "
                f"{wheel_body!r}"
            )

    if bindings.left_tire == bindings.right_tire:
        raise ValueError("left_tire and right_tire must be different")

    for side, joint_name, wheel_body in (
        ("left", bindings.left_wheel_spin_joint, left_wheel),
        ("right", bindings.right_wheel_spin_joint, right_wheel),
    ):
        joint = joints.get(joint_name)
        if joint is None:
            raise ValueError(
                f"{side}_wheel_spin_joint references unknown joint {joint_name!r}"
            )
        if joint.kind != "revolute":
            raise ValueError(f"{side}_wheel_spin_joint must be revolute")
        if wheel_body not in {joint.body_a, joint.body_b}:
            raise ValueError(
                f"{side}_wheel_spin_joint does not connect wheel body {wheel_body!r}"
            )

    if bindings.left_wheel_spin_joint == bindings.right_wheel_spin_joint:
        raise ValueError("left and right wheel spin joints must be different")

    for role, name in (
        ("left_spring", bindings.left_spring),
        ("right_spring", bindings.right_spring),
        ("left_damper", bindings.left_damper),
        ("right_damper", bindings.right_damper),
    ):
        if name not in springs:
            raise ValueError(f"{role} references unknown spring-damper {name!r}")
    if bindings.left_spring == bindings.right_spring:
        raise ValueError("left_spring and right_spring must be different")
    if bindings.left_damper == bindings.right_damper:
        raise ValueError("left_damper and right_damper must be different")
    # A corner is everything reachable from its wheel through joints without
    # passing through the sprung body or ground, so the test works for any
    # topology rather than relying on how the bodies happen to be named.
    def corner_bodies(wheel: str) -> set[str]:
        blocked = {bindings.sprung_body} | {
            body.name for body in model.bodies if body.fixed
        }
        reached = {wheel}
        frontier = [wheel]
        while frontier:
            current = frontier.pop()
            for joint in model.joints:
                pair = (joint.body_a, joint.body_b)
                if current not in pair:
                    continue
                other = pair[1] if pair[0] == current else pair[0]
                if other in blocked or other in reached:
                    continue
                reached.add(other)
                frontier.append(other)
        return reached

    left_corner = corner_bodies(left_wheel)
    right_corner = corner_bodies(right_wheel)

    for side, opposite, side_bodies, opposite_bodies, spring_name, damper_name in (
        (
            "left",
            "right",
            left_corner,
            right_corner,
            bindings.left_spring,
            bindings.left_damper,
        ),
        (
            "right",
            "left",
            right_corner,
            left_corner,
            bindings.right_spring,
            bindings.right_damper,
        ),
    ):
        for role, name in (("spring", spring_name), ("damper", damper_name)):
            element = springs[name]
            bodies_at_endpoints = {element.body_a, element.body_b}
            # The element must belong to this corner, but it need not reach the
            # wheel: a double-wishbone spring seats on the control arm, and only
            # a strut layout puts it on the upright itself.  Side membership is
            # therefore judged by the corner's own bodies, and the guard that
            # matters is that it does not cross to the other corner.
            if not (bodies_at_endpoints & side_bodies):
                raise ValueError(
                    f"{side}_{role} does not connect any body of the "
                    f"{side} corner"
                )
            if bodies_at_endpoints & opposite_bodies:
                raise ValueError(
                    f"{side}_{role} crosses to the {opposite} corner"
                )


def create_dynamic_axle_manifest(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    bindings: AxleChannelBindings,
    *,
    adams_solver: Mapping[str, object],
    execution_environment: Mapping[str, object],
    case_metadata: Mapping[str, object] | None = None,
    generator_version: str = AXLE_MANIFEST_GENERATOR,
) -> DynamicAxleManifest:
    """Create the sole immutable input shared by independent runners."""
    validate_axle_channel_bindings(model, bindings)
    acceptance = load_axle_acceptance_contract()
    channels = load_axle_channel_contract()
    _validate_public_grid(case, acceptance)
    _validate_case(case, acceptance, case_metadata or {})
    _require_keys(
        adams_solver,
        ("integrator", "error", "maximum_step_s"),
        "adams_solver",
    )
    _validate_adams_solver(adams_solver)
    _require_keys(
        execution_environment,
        (
            "cpu_model",
            "physical_core_count",
            "thread_count",
            "process_affinity",
        ),
        "execution_environment",
    )
    payload: dict[str, object] = {
        "contract": AXLE_MANIFEST_CONTRACT,
        "schema_version": 1,
        "generator_version": generator_version,
        "package_version": __version__,
        "model": model.model_dump(mode="json"),
        "case": case.model_dump(mode="json"),
        "role_bindings": bindings.model_dump(mode="json"),
        "case_metadata": dict(case_metadata or {}),
        "adams_solver": dict(adams_solver),
        "execution_environment": dict(execution_environment),
        "channels": channels,
        "acceptance": acceptance,
        "component_hashes": {
            "model_sha256": canonical_hash(model.model_dump(mode="json")),
            "case_sha256": canonical_hash(case.model_dump(mode="json")),
            "role_bindings_sha256": canonical_hash(
                bindings.model_dump(mode="json")
            ),
            "channels_sha256": canonical_hash(channels),
            "acceptance_sha256": canonical_hash(acceptance),
        },
    }
    return DynamicAxleManifest(payload=payload, sha256=canonical_hash(payload))


def write_dynamic_axle_manifest(
    manifest: DynamicAxleManifest,
    path: str | Path,
) -> Path:
    """Write a canonical, self-hashed dynamic axle manifest."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(manifest.as_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return destination


def read_dynamic_axle_manifest(path: str | Path) -> DynamicAxleManifest:
    """Read and fully validate an immutable dynamic axle manifest."""
    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("dynamic axle manifest root must be an object")
    if payload.get("contract") != AXLE_MANIFEST_CONTRACT:
        raise ValueError("unsupported dynamic axle manifest contract")
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported dynamic axle manifest schema_version")
    if payload.get("generator_version") != AXLE_MANIFEST_GENERATOR:
        raise ValueError("unsupported dynamic axle manifest generator_version")
    if not isinstance(payload.get("package_version"), str):
        raise ValueError("dynamic axle manifest package_version must be a string")
    expected_hash = payload.get("manifest_sha256")
    if not isinstance(expected_hash, str):
        raise ValueError("dynamic axle manifest has no manifest_sha256")
    content = {str(key): value for key, value in payload.items()}
    del content["manifest_sha256"]
    if canonical_hash(content) != expected_hash:
        raise ValueError("dynamic axle manifest hash does not match content")
    manifest = DynamicAxleManifest(payload=content, sha256=expected_hash)
    validate_axle_channel_bindings(manifest.model, manifest.bindings)
    acceptance = load_axle_acceptance_contract()
    channels = load_axle_channel_contract()
    if content.get("acceptance") != acceptance:
        raise ValueError("dynamic axle manifest acceptance contract is not frozen v1")
    if content.get("channels") != channels:
        raise ValueError("dynamic axle manifest channel contract is not frozen v1")
    component_hashes = content.get("component_hashes")
    if not isinstance(component_hashes, Mapping):
        raise ValueError("dynamic axle manifest component_hashes must be an object")
    expected_component_hashes = {
        "model_sha256": canonical_hash(manifest.model.model_dump(mode="json")),
        "case_sha256": canonical_hash(manifest.case.model_dump(mode="json")),
        "role_bindings_sha256": canonical_hash(
            manifest.bindings.model_dump(mode="json")
        ),
        "channels_sha256": canonical_hash(channels),
        "acceptance_sha256": canonical_hash(acceptance),
    }
    if dict(component_hashes) != expected_component_hashes:
        raise ValueError("dynamic axle manifest component hashes do not match content")
    adams_solver = content.get("adams_solver")
    if not isinstance(adams_solver, Mapping):
        raise ValueError("dynamic axle manifest adams_solver must be an object")
    _require_keys(
        adams_solver,
        ("integrator", "error", "maximum_step_s"),
        "adams_solver",
    )
    _validate_adams_solver(adams_solver)
    execution_environment = content.get("execution_environment")
    if not isinstance(execution_environment, Mapping):
        raise ValueError(
            "dynamic axle manifest execution_environment must be an object"
        )
    _require_keys(
        execution_environment,
        (
            "cpu_model",
            "physical_core_count",
            "thread_count",
            "process_affinity",
        ),
        "execution_environment",
    )
    _validate_public_grid(manifest.case, acceptance)
    metadata = content.get("case_metadata")
    if not isinstance(metadata, Mapping):
        raise ValueError("dynamic axle manifest case_metadata must be an object")
    _validate_case(manifest.case, acceptance, metadata)
    return manifest


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path.name} must contain an object")
    return cast(dict[str, Any], payload)


def _require_keys(
    value: Mapping[str, object],
    names: tuple[str, ...],
    label: str,
) -> None:
    missing = [name for name in names if name not in value]
    if missing:
        raise ValueError(f"{label} misses required fields: {missing}")


def _validate_adams_solver(value: Mapping[str, object]) -> None:
    """Freeze parameters whose Adams defaults would change the comparison."""
    integrator = str(value.get("integrator", "")).strip().lower()
    if integrator == "hht":
        if "alpha" not in value:
            raise ValueError(
                "adams_solver for HHT must declare alpha explicitly; relying on "
                "the Adams default is not an equivalent comparison condition"
            )
        try:
            alpha = float(value["alpha"])
        except (TypeError, ValueError) as exc:
            raise ValueError("adams_solver HHT alpha must be a finite number") from exc
        if not math.isfinite(alpha) or alpha < -0.333333 or alpha > 0.0:
            raise ValueError(
                "adams_solver HHT alpha must be within the Adams range [-1/3, 0]"
            )
    fixed_values = (
        value.get("fixed_iterations"),
        value.get("step_ratio"),
    )
    if any(item is not None for item in fixed_values):
        if any(item is None for item in fixed_values):
            raise ValueError(
                "adams_solver fixed_iterations and step_ratio must be declared together"
            )
        try:
            fixed_iterations = int(value["fixed_iterations"])
            step_ratio = int(value["step_ratio"])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "adams_solver fixed-step settings must be integers"
            ) from exc
        if not 1 <= fixed_iterations <= 10:
            raise ValueError("adams_solver fixed_iterations must be in [1, 10]")
        if step_ratio < 1:
            raise ValueError("adams_solver step_ratio must be positive")


def _validate_public_grid(
    case: AxleDynamicsCase,
    acceptance: Mapping[str, object],
) -> None:
    comparison = cast(Mapping[str, object], acceptance["comparison"])
    rate = float(comparison["public_sample_rate_hz"])
    expected_step = 1.0 / rate
    for index, (left, right) in enumerate(zip(case.times_s, case.times_s[1:])):
        step = right - left
        if not math.isclose(step, expected_step, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"case public grid step at {index} is {step}, expected "
                f"{expected_step}"
            )


def _validate_case(
    case: AxleDynamicsCase,
    acceptance: Mapping[str, object],
    metadata: Mapping[str, object],
) -> None:
    cases = {
        str(item["name"])
        for item in cast(list[dict[str, object]], acceptance["case_matrix"])
    }
    if case.name not in cases:
        raise ValueError(f"case {case.name!r} is outside the frozen case matrix")
    if case.name == "road_sine":
        frequency = metadata.get("harmonic_frequency_hz")
        if not isinstance(frequency, (int, float)) or float(frequency) <= 0.0:
            raise ValueError("road_sine requires positive harmonic_frequency_hz")
