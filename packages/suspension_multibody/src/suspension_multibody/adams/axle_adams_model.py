"""
Native Adams Solver dataset generated item-by-item from the frozen manifest.

The dynamic axle manifest is the sole shared input.  This module turns that
manifest into a raw Adams/Solver dataset built only from primitive statements
(``PART``, ``MARKER``, ``JOINT``, ``SFORCE``, ``VARIABLE``, ``DIFF``,
``GFORCE``) so that every physical term has a traceable counterpart in the
native C++ kernel.  Stock Adams/Car templates and black-box tire models are
never used.

Any manifest element without an exact Adams/Solver counterpart is refused with
:class:`AxleAdamsExpressibilityError`; the generator never emits an
approximation, because an approximated Adams model cannot support a 5%
correlation claim.

Statement syntax follows the Adams/Solver datasets shipped with Adams 2024.1
(``solver/samples``, ``aview/examples``, ``vibration/examples``): parts carry no
``QG`` so marker ``QP``/``REULER`` are global, expressions continue on lines
that start with a comma, and angles carry an explicit ``D`` suffix.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import numpy as np

from ..axle_dynamics.schema import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleHarmonicRoad,
    AxleJoint,
    AxleSpringDamper,
    AxleTire,
)
from ..io import canonical_hash
from .axle_contract import (
    FIXTURE_WRENCH_CONVENTION,
    DynamicAxleManifest,
)

AXLE_ADAMS_DATASET_CONTRACT = "dynamic-axle-adams-dataset-v1"
AXLE_ADAMS_GENERATOR = "open-kinematics-axle-adams-v1"

#: Piecewise-linear inputs are emitted as an exact ramp sum.  A frozen upper
#: bound keeps the generated expression parseable; a case that needs more
#: breakpoints is refused instead of resampled onto an inexact spline.
MAXIMUM_INPUT_BREAKPOINTS = 64

#: Relative tolerance used when collapsing collinear samples of a public-grid
#: input signal into exact piecewise-linear breakpoints.
_COLLINEAR_TOLERANCE = 1e-12

_IDENTITY_QUATERNION = (1.0, 0.0, 0.0, 0.0)
_JOINT_KEYWORD = {
    "spherical": "SPHERICAL",
    "revolute": "REVOLUTE",
    "prismatic": "TRANSLATIONAL",
    "fixed": "FIXED",
    "universal": "HOOKE",
    "cylindrical": "CYLINDRICAL",
}
# Adams writes an in-plane condition as a joint primitive, not a JOINT.
_JPRIM_KEYWORD = {"inplane": "INPLANE"}


class AxleAdamsExpressibilityError(ValueError):
    """Raised when a manifest has no exact native Adams/Solver counterpart."""

    def __init__(self, blockers: Sequence[str]) -> None:
        self.blockers = tuple(blockers)
        super().__init__(
            "manifest cannot be expressed as native Adams/Solver elements: "
            + "; ".join(self.blockers)
        )


@dataclass(frozen=True)
class AxleAdamsDataset:
    """One generated Adams/Solver dataset plus its traceability metadata."""

    stem: str
    manifest_sha256: str
    model_text: str
    command_text: str
    conventions: Mapping[str, object]
    entity_ids: Mapping[str, int]
    requests: tuple[Mapping[str, object], ...]

    def as_dict(self) -> dict[str, object]:
        content: dict[str, object] = {
            "contract": AXLE_ADAMS_DATASET_CONTRACT,
            "generator": AXLE_ADAMS_GENERATOR,
            "stem": self.stem,
            "manifest_sha256": self.manifest_sha256,
            "conventions": dict(self.conventions),
            "entity_ids": dict(self.entity_ids),
            "requests": [dict(request) for request in self.requests],
            "model_text_sha256": canonical_hash({"text": self.model_text}),
            "command_text_sha256": canonical_hash({"text": self.command_text}),
        }
        return {**content, "dataset_sha256": canonical_hash(content)}


def axle_adams_blockers(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
) -> tuple[str, ...]:
    """List every manifest feature without an exact Adams/Solver counterpart."""
    blockers: list[str] = []
    if case.solver.initialization_mode not in {
        "static_equilibrium",
        "provided_consistent_state",
    }:
        blockers.append(
            "solver.initialization_mode must be 'static_equilibrium' or "
            "'provided_consistent_state'"
        )
    for body in model.bodies:
        if _nonzero(body.linear_velocity_m_per_s) or _nonzero(
            body.angular_velocity_rad_per_s
        ):
            blockers.append(
                f"body {body.name!r} declares a nonzero initial velocity; Adams "
                "sets part initial velocities from the command file and its "
                "reference frame is outside this frozen mapping"
            )
    poses = {body.name: body.quaternion_body_to_world for body in model.bodies}
    for joint in model.joints:
        if joint.kind not in {"fixed", "prismatic"}:
            continue
        if not _quaternions_equal(poses[joint.body_a], poses[joint.body_b]):
            blockers.append(
                f"joint {joint.name!r} of kind {joint.kind!r} drives the two body "
                "frames to a common orientation, but the manifest assembles the "
                "bodies with different initial orientations"
            )
    for bushing in model.bushings:
        if _matrix_nonzero(bushing.stiffness) or _matrix_nonzero(
            bushing.damping
        ) or _nonzero(bushing.preload_in_frame_a_n_n_m):
            blockers.append(
                f"bushing {bushing.name!r} has nonzero force terms, but the "
                "primitive dataset has no exact BUSHING/FIELD emitter"
            )
        if not _quaternions_equal(
            bushing.reference_quaternion_a_to_b, _IDENTITY_QUATERNION
        ):
            blockers.append(
                f"bushing {bushing.name!r} declares a non-identity reference "
                "orientation, which an Adams FIELD cannot carry"
            )
        if _matrix_has_rotational_terms(bushing.stiffness) or (
            _matrix_has_rotational_terms(bushing.damping)
        ):
            blockers.append(
                f"bushing {bushing.name!r} has rotational stiffness or damping; "
                "the native rotation-vector measure and the Adams FIELD angular "
                "measure agree only to second order in the rotation angle"
            )
        if _nonzero(bushing.preload_in_frame_a_n_n_m):
            blockers.append(
                f"bushing {bushing.name!r} declares a preload, which an Adams "
                "FIELD does not carry"
            )
    for bar in model.anti_roll_bars:
        if bar.stiffness_n_m_per_rad != 0.0 or bar.damping_n_m_s_per_rad != 0.0:
            blockers.append(
                f"anti-roll bar {bar.name!r} applies its torque about an axis "
                "carried by body_a, while an Adams rotational SFORCE resolves "
                "its axis on the I marker; the two diverge under finite "
                "relative rotation"
            )
    blockers.extend(_input_blockers(model, case))
    return tuple(blockers)


def _input_blockers(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
) -> list[str]:
    blockers: list[str] = []
    unsupported_tire_models = sorted(
        {
            tire.model_kind
            for tire in model.tires
            if tire.model_kind != "native_brush"
        }
    )
    if unsupported_tire_models:
        blockers.append(
            "primitive Adams axle dataset implements only the explicit "
            "native_brush reference; PAC2002 must use the independent "
            "Adams/Car PAC2002 reference: "
            + ", ".join(unsupported_tire_models)
        )
    bodies = {body.name: body for body in model.bodies}
    # A declared harmonic is written as a closed-form SIN expression, so its
    # tire never needs breakpoints on either side.
    harmonic_tires = {road.tire for road in case.harmonic_roads}
    for label, signals in (
        ("road_height_m", case.road_height_m),
        ("road_velocity_m_per_s", case.road_velocity_m_per_s),
        ("wheel_torque_n_m", case.wheel_torque_n_m),
    ):
        for name, values in sorted(signals.items()):
            if name in harmonic_tires and label.startswith("road_"):
                continue
            count = len(_linear_breakpoints(case.times_s, values))
            if count > MAXIMUM_INPUT_BREAKPOINTS:
                blockers.append(
                    f"{label}[{name!r}] needs {count} piecewise-linear "
                    f"breakpoints, above the frozen limit of "
                    f"{MAXIMUM_INPUT_BREAKPOINTS}; Adams has no exact "
                    "piecewise-linear spline for this many knots"
                )
    for name, wrenches in sorted(case.body_wrench_n_n_m.items()):
        body = bodies.get(name)
        if body is None:
            blockers.append(f"body_wrench_n_n_m references unknown body {name!r}")
            continue
        if body.fixed and any(
            any(component != 0.0 for component in wrench) for wrench in wrenches
        ):
            blockers.append(
                f"body_wrench_n_n_m[{name!r}] applies a nonzero wrench to a fixed "
                "body, which the primitive dataset does not emit"
            )
        for component in range(6):
            values = tuple(wrench[component] for wrench in wrenches)
            count = len(_linear_breakpoints(case.times_s, values))
            if count > MAXIMUM_INPUT_BREAKPOINTS:
                blockers.append(
                    f"body_wrench_n_n_m[{name!r}] component {component} needs "
                    f"{count} piecewise-linear breakpoints, above the frozen "
                    f"limit of {MAXIMUM_INPUT_BREAKPOINTS}"
                )
    return blockers


def build_axle_adams_dataset(
    manifest: DynamicAxleManifest,
    *,
    stem: str = "dynamic_axle",
) -> AxleAdamsDataset:
    """Generate the raw Adams/Solver dataset for one frozen manifest."""
    blockers = axle_adams_blockers(manifest.model, manifest.case)
    if blockers:
        raise AxleAdamsExpressibilityError(blockers)
    return _DatasetBuilder(manifest, stem).build()


def write_axle_adams_dataset(
    dataset: AxleAdamsDataset,
    output_dir: str | Path,
) -> dict[str, Path]:
    """Write the dataset, command file, and traceability sidecar."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    model_path = destination / f"{dataset.stem}.adm"
    command_path = destination / f"{dataset.stem}.acf"
    sidecar_path = destination / f"{dataset.stem}_dataset.json"
    model_path.write_text(dataset.model_text, encoding="ascii")
    command_path.write_text(dataset.command_text, encoding="ascii")
    sidecar_path.write_text(
        json.dumps(dataset.as_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return {
        "model": model_path,
        "command": command_path,
        "sidecar": sidecar_path,
    }


class _DatasetBuilder:
    """Allocate deterministic Adams identifiers and emit primitive statements."""

    def __init__(self, manifest: DynamicAxleManifest, stem: str) -> None:
        self._manifest = manifest
        self._model = manifest.model
        self._case = manifest.case
        self._stem = stem
        self._lines: list[str] = []
        self._marker_id = 1
        self._part_id = 1
        self._variable_id = 0
        self._diff_id = 0
        self._gforce_id = 0
        self._request_id = 0
        self._entity_ids: dict[str, int] = {}
        self._part_of_body: dict[str, int] = {}
        self._cm_marker: dict[str, int] = {}
        self._input_variables: dict[str, int] = {}
        self._tire_variables: dict[str, dict[str, int]] = {}
        self._requests: list[dict[str, object]] = []

    # -- identifier helpers -------------------------------------------------

    def _next_marker(self) -> int:
        self._marker_id += 1
        return self._marker_id

    def _emit(self, line: str) -> None:
        self._lines.append(line)

    def _body(self, name: str) -> AxleBody:
        return next(body for body in self._model.bodies if body.name == name)

    def _marker(
        self,
        key: str,
        body_name: str,
        point_local: Sequence[float],
        *,
        z_local: Sequence[float] | None = None,
        x_local: Sequence[float] | None = None,
    ) -> int:
        """
        Create one global-coordinate marker on the part carrying a body.

        Adams parts are written without ``QG``, so at ``t = 0`` every part local
        reference frame coincides with ground and marker coordinates are global.
        Fixed fixture bodies never move, so their markers live on ``PART/1``.
        """
        body = self._body(body_name)
        rotation = _rotation_matrix(body.quaternion_body_to_world)
        origin = np.asarray(body.position_m, dtype=float) + rotation @ np.asarray(
            point_local, dtype=float
        )
        orientation = rotation @ _orthonormal_frame(z_local, x_local)
        part = 1 if body.fixed else self._part_of_body[body_name]
        identifier = self._next_marker()
        self._entity_ids[key] = identifier
        record = f"MARKER/{identifier}, PART = {part}"
        if not np.array_equal(origin, np.zeros(3)):
            record += (
                f", QP = {_number(origin[0])}, {_number(origin[1])}"
                f", {_number(origin[2])}"
            )
        if not np.array_equal(orientation, np.eye(3)):
            psi, theta, phi = _euler_313_degrees(orientation)
            record += (
                f", REULER = {_number(psi)}D, {_number(theta)}D"
                f", {_number(phi)}D"
            )
        self._emit(record)
        return identifier

    def _variable(self, key: str, expression: str, comment: str) -> int:
        self._variable_id += 1
        identifier = self._variable_id
        self._entity_ids[key] = identifier
        self._emit(f"! {comment}")
        self._emit(f"VARIABLE/{identifier}, IC = 0")
        for line in _continuation(expression, ", FUNCTION = "):
            self._emit(line)
        return identifier

    # -- dataset sections ---------------------------------------------------

    def build(self) -> AxleAdamsDataset:
        self._emit_header()
        self._emit_parts()
        self._emit_joints()
        self._emit_springs()
        self._emit_inputs()
        self._emit_tires()
        self._emit_gravity()
        self._emit_requests()
        self._emit("!")
        self._emit("OUTPUT/REQSAVE")
        self._emit("RESULTS/FORMATTED, XRF")
        self._emit("END")
        self._emit("")
        return AxleAdamsDataset(
            stem=self._stem,
            manifest_sha256=self._manifest.sha256,
            model_text="\n".join(self._lines),
            command_text=self._command_text(),
            conventions=_CONVENTIONS,
            entity_ids=dict(self._entity_ids),
            requests=tuple(self._requests),
        )

    def _emit_header(self) -> None:
        # Adams reads the first line of a dataset as a free-form title, not as
        # a statement, so it must come before the comments and must not look
        # like a keyword.
        self._emit(f" open-kinematics {self._stem}")
        self._emit(f"! contract: {AXLE_ADAMS_DATASET_CONTRACT}")
        self._emit(f"! generator: {AXLE_ADAMS_GENERATOR}")
        self._emit(f"! manifest_sha256: {self._manifest.sha256}")
        self._emit(f"! case: {self._case.name}")
        self._emit(
            "! every statement is generated from the shared manifest; no "
            "Adams/Car template and no black-box tire is used"
        )
        self._emit(
            "UNITS/FORCE = NEWTON, MASS = KILOGRAM, LENGTH = METER"
            ", TIME = SECOND"
        )

    def _emit_parts(self) -> None:
        self._emit("!")
        self._emit("! ground carries every fixed fixture body")
        self._emit("PART/1, GROUND")
        self._emit("MARKER/1, PART = 1")
        self._entity_ids["ground"] = 1
        for body in self._model.bodies:
            if body.fixed:
                continue
            self._part_id += 1
            part = self._part_id
            self._part_of_body[body.name] = part
            self._emit("!")
            self._emit(f"! body {body.name}")
            self._emit(
                f"PART/{part}, MASS = {_number(body.mass_kg)}"
                f", CM = {self._marker_id + 1}"
                f", IP = {_inertia_fields(body.inertia_kg_m2)}"
            )
            identifier = self._marker(
                f"body:{body.name}:cm",
                body.name,
                (0.0, 0.0, 0.0),
                z_local=(0.0, 0.0, 1.0),
                x_local=(1.0, 0.0, 0.0),
            )
            self._cm_marker[body.name] = identifier

    def _emit_joints(self) -> None:
        if not self._model.joints:
            return
        self._emit("!")
        self._emit("! ideal joints")
        for index, joint in enumerate(self._model.joints, start=1):
            i_marker, j_marker = self._joint_markers(joint)
            self._entity_ids[f"joint:{joint.name}"] = index
            if joint.kind in _JPRIM_KEYWORD:
                # Adams writes the primitive type after the markers, and joint
                # primitives carry their own id space separate from JOINT.
                self._emit(
                    f"JPRIM/{index}, I = {i_marker}, J = {j_marker}"
                    f", {_JPRIM_KEYWORD[joint.kind]}"
                )
            else:
                self._emit(
                    f"JOINT/{index}, {_JOINT_KEYWORD[joint.kind]}"
                    f", I = {i_marker}, J = {j_marker}"
                )

    def _joint_markers(self, joint: AxleJoint) -> tuple[int, int]:
        # Adams takes the joint direction from each marker's z axis, for the
        # revolute/translational axis, the two Hooke cross axes, the shared
        # cylindrical axis, and the in-plane normal alike.
        axial = joint.kind in {
            "revolute",
            "prismatic",
            "universal",
            "cylindrical",
            "inplane",
        }
        i_marker = self._marker(
            f"joint:{joint.name}:i",
            joint.body_b,
            joint.point_b_m,
            z_local=joint.axis_b if axial else None,
        )
        j_marker = self._marker(
            f"joint:{joint.name}:j",
            joint.body_a,
            joint.point_a_m,
            z_local=joint.axis_a if axial else None,
        )
        return i_marker, j_marker

    def _emit_springs(self) -> None:
        if not self._model.springs:
            return
        self._emit("!")
        self._emit("! axial spring, damper, and stop elements")
        for index, spring in enumerate(self._model.springs, start=1):
            i_marker = self._marker(
                f"spring:{spring.name}:i", spring.body_b, spring.point_b_m
            )
            j_marker = self._marker(
                f"spring:{spring.name}:j", spring.body_a, spring.point_a_m
            )
            self._entity_ids[f"spring:{spring.name}"] = index
            self._emit(f"! spring-damper {spring.name}")
            damper_curve_expression: str | None = None
            if spring.damper_curve_velocity_m_per_s:
                rate = f"VR({i_marker}, {j_marker})"
                curve_id = self._variable(
                    f"spring:{spring.name}:damper_curve",
                    _piecewise_linear_expression(
                        rate,
                        spring.damper_curve_velocity_m_per_s,
                        spring.damper_curve_force_n,
                    ),
                    f"piecewise-linear damper curve for {spring.name}",
                )
                damper_curve_expression = f"VARVAL({curve_id})"
            # Adams expects the force kind on its own continuation line; on the
            # SFORCE line itself it is read as a force component number.
            self._emit(f"SFORCE/{index}")
            self._emit(", TRANSLATIONAL")
            self._emit(f", I = {i_marker}")
            self._emit(f", J = {j_marker}")
            for line in _continuation(
                _spring_function(
                    spring,
                    i_marker,
                    j_marker,
                    damper_curve_expression=damper_curve_expression,
                ),
                ", FUNCTION = ",
            ):
                self._emit(line)

    def _emit_inputs(self) -> None:
        self._emit("!")
        self._emit("! prescribed inputs as exact piecewise-linear ramp sums")
        for tire in self._model.tires:
            for label, signals in (
                ("road_height", self._case.road_height_m),
                ("road_velocity", self._case.road_velocity_m_per_s),
                ("wheel_torque", self._case.wheel_torque_n_m),
            ):
                key = f"input:{label}:{tire.name}"
                harmonic = next(
                    (
                        road
                        for road in self._case.harmonic_roads
                        if road.tire == tire.name
                    ),
                    None,
                )
                if harmonic is not None and label in (
                    "road_height",
                    "road_velocity",
                ):
                    expression = _harmonic_expression(harmonic, label)
                else:
                    expression = _ramp_sum_expression(
                        self._case.times_s, signals.get(tire.name)
                    )
                identifier = self._variable(
                    key,
                    expression,
                    f"{label} of tire {tire.name}",
                )
                self._input_variables[f"{label}:{tire.name}"] = identifier
        for name, wrenches in sorted(self._case.body_wrench_n_n_m.items()):
            if self._body(name).fixed:
                continue
            components = [
                self._variable(
                    f"input:body_wrench:{name}:{component}",
                    _ramp_sum_expression(
                        self._case.times_s,
                        tuple(wrench[component] for wrench in wrenches),
                    ),
                    f"applied wrench component {component} on body {name}",
                )
                for component in range(6)
            ]
            float_marker = self._next_marker()
            self._entity_ids[f"input:body_wrench:{name}:jfloat"] = float_marker
            self._emit(f"MARKER/{float_marker}, PART = 1, FLOATING")
            self._gforce_id += 1
            self._entity_ids[f"input:body_wrench:{name}:gforce"] = self._gforce_id
            self._emit(
                f"GFORCE/{self._gforce_id}, I = {self._cm_marker[name]}"
                f", JFLOAT = {float_marker}, RM = 1"
            )
            for axis, identifier in zip(
                ("FX", "FY", "FZ", "TX", "TY", "TZ"), components
            ):
                self._emit(f", {axis} = VARVAL({identifier})")

    def _emit_tires(self) -> None:
        if not self._model.tires:
            return
        self._emit("!")
        self._emit(
            "! native_brush unilateral compliant contact; the friction-ellipse "
            "radial return is written out explicitly"
        )
        for tire in self._model.tires:
            self._emit_tire(tire)

    def _emit_tire(self, tire: AxleTire) -> None:
        spin = _unit(tire.spin_axis_local)
        forward = _unit(
            np.asarray(tire.forward_axis_local, dtype=float)
            - spin * float(np.dot(tire.forward_axis_local, spin))
        )
        centre_local = np.asarray(tire.center_local_m, dtype=float)
        self._emit("!")
        self._emit(f"! tire {tire.name} on body {tire.body}")
        centre = self._marker(
            f"tire:{tire.name}:centre", tire.body, tire.center_local_m
        )
        forward_marker = self._marker(
            f"tire:{tire.name}:forward",
            tire.body,
            tuple(centre_local + forward),
        )
        spin_marker = self._marker(
            f"tire:{tire.name}:spin", tire.body, tuple(centre_local + spin)
        )
        cm = self._cm_marker[tire.body]
        road_z = self._input_variables[f"road_height:{tire.name}"]
        road_v = self._input_variables[f"road_velocity:{tire.name}"]
        drive = self._input_variables[f"wheel_torque:{tire.name}"]
        radius = _number(tire.unloaded_radius_m)

        def variable(label: str, expression: str, comment: str) -> int:
            return self._variable(
                f"tire:{tire.name}:{label}",
                expression,
                f"{tire.name}: {comment}",
            )

        raw_x = f"DX({forward_marker}, {centre}, 1)"
        raw_y = f"DY({forward_marker}, {centre}, 1)"
        norm = variable(
            "forward_norm",
            f"SQRT(({raw_x})**2 + ({raw_y})**2)",
            "in-plane norm of the wheel forward axis",
        )
        forward_x = variable(
            "forward_x",
            f"({raw_x})/VARVAL({norm})",
            "road-tangent forward unit vector, x",
        )
        forward_y = variable(
            "forward_y",
            f"({raw_y})/VARVAL({norm})",
            "road-tangent forward unit vector, y",
        )
        penetration = variable(
            "penetration",
            f"{radius} + VARVAL({road_z}) - DZ({centre}, 1, 1)",
            "normal penetration",
        )
        penetration_rate = variable(
            "penetration_rate",
            f"VARVAL({road_v}) - VZ({centre}, 1, 1, 1)",
            "normal closing rate",
        )
        normal = variable(
            "normal_force",
            f"IF(VARVAL({penetration}): 0, 0"
            f", MAX(0, {_number(tire.vertical_stiffness_n_per_m)}"
            f"*VARVAL({penetration})"
            f" + {_number(tire.vertical_damping_n_s_per_m)}"
            f"*VARVAL({penetration_rate})))",
            "unilateral normal force",
        )
        patch_x = f"(VX({centre}, 1, 1, 1) - {radius}*WY({centre}, 1, 1))"
        patch_y = f"(VY({centre}, 1, 1, 1) + {radius}*WX({centre}, 1, 1))"
        slip_x = variable(
            "longitudinal_slip",
            f"{patch_x}*VARVAL({forward_x}) + {patch_y}*VARVAL({forward_y})",
            "longitudinal patch slip velocity",
        )
        slip_y = variable(
            "lateral_slip",
            f"-{patch_x}*VARVAL({forward_y}) + {patch_y}*VARVAL({forward_x})",
            "lateral patch slip velocity",
        )
        rolling = variable(
            "rolling_speed",
            f"ABS(VX({centre}, 1, 1, 1)*VARVAL({forward_x})"
            f" + VY({centre}, 1, 1, 1)*VARVAL({forward_y}))",
            "rolling speed driving the brush relaxation length",
        )
        brush_x = self._brush_state(
            tire,
            "brush_x",
            slip_x,
            normal,
            rolling,
            tire.longitudinal_relaxation_length_m,
            "longitudinal",
        )
        brush_y = self._brush_state(
            tire,
            "brush_y",
            slip_y,
            normal,
            rolling,
            tire.lateral_relaxation_length_m,
            "lateral",
        )
        utilisation = variable(
            "friction_utilization",
            f"IF(VARVAL({normal}): 0, 0, SQRT(("
            f"{_number(tire.longitudinal_brush_stiffness_n_per_m)}*DIF({brush_x})"
            f"/({_number(tire.longitudinal_friction_coefficient)}"
            f"*VARVAL({normal})))**2 + ("
            f"{_number(tire.lateral_brush_stiffness_n_per_m)}*DIF({brush_y})"
            f"/({_number(tire.lateral_friction_coefficient)}"
            f"*VARVAL({normal})))**2))",
            "trial friction-ellipse utilisation",
        )
        scale = variable(
            "return_scale",
            f"MAX(1, VARVAL({utilisation}))",
            "friction-ellipse radial-return scale",
        )
        force_x = variable(
            "longitudinal_force",
            f"IF(VARVAL({normal}): 0, 0"
            f", -{_number(tire.longitudinal_brush_stiffness_n_per_m)}"
            f"*DIF({brush_x})/VARVAL({scale}))",
            "longitudinal contact force",
        )
        force_y = variable(
            "lateral_force",
            f"IF(VARVAL({normal}): 0, 0"
            f", -{_number(tire.lateral_brush_stiffness_n_per_m)}"
            f"*DIF({brush_y})/VARVAL({scale}))",
            "lateral contact force",
        )
        world_x = variable(
            "force_world_x",
            f"VARVAL({force_x})*VARVAL({forward_x})"
            f" - VARVAL({force_y})*VARVAL({forward_y})",
            "contact force in ground, x",
        )
        world_y = variable(
            "force_world_y",
            f"VARVAL({force_x})*VARVAL({forward_y})"
            f" + VARVAL({force_y})*VARVAL({forward_x})",
            "contact force in ground, y",
        )
        arm_x = f"DX({centre}, {cm}, 1)"
        arm_y = f"DY({centre}, {cm}, 1)"
        arm_z = f"(DZ({centre}, {cm}, 1) - {radius})"
        spin_x = f"DX({spin_marker}, {centre}, 1)"
        spin_y = f"DY({spin_marker}, {centre}, 1)"
        spin_z = f"DZ({spin_marker}, {centre}, 1)"
        float_marker = self._next_marker()
        self._entity_ids[f"tire:{tire.name}:jfloat"] = float_marker
        self._emit(f"MARKER/{float_marker}, PART = 1, FLOATING")
        self._gforce_id += 1
        self._entity_ids[f"tire:{tire.name}:gforce"] = self._gforce_id
        self._emit(
            f"GFORCE/{self._gforce_id}, I = {cm}, JFLOAT = {float_marker}, RM = 1"
        )
        # Like REQUEST, a GFORCE spanning several attribute lines needs each
        # line but the last to end in a backslash, or Adams merges the
        # attributes into one expression.
        block = [
            f", FX = VARVAL({world_x})",
            f", FY = VARVAL({world_y})",
            f", FZ = VARVAL({normal})",
        ]
        for axis, expression in (
            (
                "TX",
                f"{arm_y}*VARVAL({normal}) - {arm_z}*VARVAL({world_y})"
                f" + {spin_x}*VARVAL({drive})",
            ),
            (
                "TY",
                f"{arm_z}*VARVAL({world_x}) - {arm_x}*VARVAL({normal})"
                f" + {spin_y}*VARVAL({drive})",
            ),
            (
                "TZ",
                f"{arm_x}*VARVAL({world_y}) - {arm_y}*VARVAL({world_x})"
                f" + {spin_z}*VARVAL({drive})",
            ),
        ):
            block.extend(_continuation(expression, f", {axis} = "))
        for line in block[:-1]:
            self._emit(line if line.endswith("\\") else line + " \\")
        self._emit(block[-1])
        self._tire_variables[tire.name] = {
            "penetration": penetration,
            "penetration_rate": penetration_rate,
            "normal_force": normal,
            "longitudinal_force": force_x,
            "lateral_force": force_y,
            "longitudinal_slip": slip_x,
            "lateral_slip": slip_y,
            "friction_utilization": utilisation,
            "brush_x": brush_x,
            "brush_y": brush_y,
        }

    def _brush_state(
        self,
        tire: AxleTire,
        label: str,
        slip: int,
        normal: int,
        rolling: int,
        relaxation_length_m: float,
        comment: str,
    ) -> int:
        self._diff_id += 1
        identifier = self._diff_id
        self._entity_ids[f"tire:{tire.name}:{label}"] = identifier
        detached = (
            f"-DIF({identifier})/{_number(tire.detached_relaxation_s)}"
        )
        attached = (
            f"VARVAL({slip}) - VARVAL({rolling})"
            f"/{_number(relaxation_length_m)}*DIF({identifier})"
        )
        self._emit(f"! {tire.name}: {comment} brush deflection state")
        self._emit(f"DIFF/{identifier}, IC = 0")
        for line in _continuation(
            f"IF(VARVAL({normal}): {detached}, {detached}, {attached})",
            ", FUNCTION = ",
        ):
            self._emit(line)
        return identifier

    def _emit_gravity(self) -> None:
        gravity = self._model.gravity_m_per_s2
        self._emit("!")
        self._emit(
            f"ACCGRAV/IGRAV = {_number(gravity[0])}"
            f", JGRAV = {_number(gravity[1])}"
            f", KGRAV = {_number(gravity[2])}"
        )

    def _emit_requests(self) -> None:
        self._emit("!")
        self._emit(
            "! raw state and element requests; the 33 frozen channels are not "
            "requested from Adams but evaluated in Python by the same exporter "
            "that serves the native run; canonical body states use CM variables"
        )
        for body in self._model.bodies:
            if body.fixed:
                continue
            cm = self._cm_marker[body.name]
            for component, expression in (
                ("X", f"DX({cm}, 1, 1)"),
                ("Y", f"DY({cm}, 1, 1)"),
                ("Z", f"DZ({cm}, 1, 1)"),
                ("PSI", f"PSI({cm}, 1)"),
                ("THETA", f"THETA({cm}, 1)"),
                ("PHI", f"PHI({cm}, 1)"),
                ("VX", f"VX({cm}, 1, 1, 1)"),
                ("VY", f"VY({cm}, 1, 1, 1)"),
                ("VZ", f"VZ({cm}, 1, 1, 1)"),
                ("WX", f"WX({cm}, 1, 1)"),
                ("WY", f"WY({cm}, 1, 1)"),
                ("WZ", f"WZ({cm}, 1, 1)"),
                ("ACCX", f"ACCX({cm}, 1, 1, 1)"),
                ("ACCY", f"ACCY({cm}, 1, 1, 1)"),
                ("ACCZ", f"ACCZ({cm}, 1, 1, 1)"),
                ("WDX", f"WDTX({cm}, 1, 1, 1)"),
                ("WDY", f"WDTY({cm}, 1, 1, 1)"),
                ("WDZ", f"WDTZ({cm}, 1, 1, 1)"),
            ):
                self._variable(
                    f"body:{body.name}:state:{component}",
                    expression,
                    f"CM state {component} of body {body.name}",
                )
            self._request(
                f"body:{body.name}:pose",
                (
                    (f"DX({cm}, 1, 1)", "position_x"),
                    (f"DY({cm}, 1, 1)", "position_y"),
                    (f"DZ({cm}, 1, 1)", "position_z"),
                    (f"PSI({cm}, 1)", "euler_psi"),
                    (f"THETA({cm}, 1)", "euler_theta"),
                    (f"PHI({cm}, 1)", "euler_phi"),
                ),
            )
            self._request(
                f"body:{body.name}:rate",
                (
                    (f"VX({cm}, 1, 1, 1)", "velocity_x"),
                    (f"VY({cm}, 1, 1, 1)", "velocity_y"),
                    (f"VZ({cm}, 1, 1, 1)", "velocity_z"),
                    (f"WX({cm}, 1, 1)", "omega_x"),
                    (f"WY({cm}, 1, 1)", "omega_y"),
                    (f"WZ({cm}, 1, 1)", "omega_z"),
                ),
            )
            self._request(
                f"body:{body.name}:acceleration",
                (
                    (f"ACCX({cm}, 1, 1, 1)", "acceleration_x"),
                    (f"ACCY({cm}, 1, 1, 1)", "acceleration_y"),
                    (f"ACCZ({cm}, 1, 1, 1)", "acceleration_z"),
                    (f"WDTX({cm}, 1, 1, 1)", "alpha_x"),
                    (f"WDTY({cm}, 1, 1, 1)", "alpha_y"),
                    (f"WDTZ({cm}, 1, 1, 1)", "alpha_z"),
                ),
            )
        for joint in self._model.joints:
            i_marker = self._entity_ids[f"joint:{joint.name}:i"]
            j_marker = self._entity_ids[f"joint:{joint.name}:j"]
            self._request(
                f"joint:{joint.name}:wrench",
                (
                    (f"FX({i_marker}, {j_marker}, 1)", "force_x"),
                    (f"FY({i_marker}, {j_marker}, 1)", "force_y"),
                    (f"FZ({i_marker}, {j_marker}, 1)", "force_z"),
                    # The three-argument form already reports the moment about
                    # the I marker, which is the reference the frozen channel
                    # contract names; the four-argument spelling is not valid
                    # Adams function syntax.
                    (f"TX({i_marker}, {j_marker}, 1)", "moment_x"),
                    (f"TY({i_marker}, {j_marker}, 1)", "moment_y"),
                    (f"TZ({i_marker}, {j_marker}, 1)", "moment_z"),
                ),
            )
        for spring in self._model.springs:
            i_marker = self._entity_ids[f"spring:{spring.name}:i"]
            j_marker = self._entity_ids[f"spring:{spring.name}:j"]
            identifier = self._entity_ids[f"spring:{spring.name}"]
            length = f"DM({i_marker}, {j_marker})"
            rate = f"VR({i_marker}, {j_marker})"
            self._request(
                f"spring:{spring.name}:output",
                (
                    (length, "length"),
                    (rate, "length_rate"),
                    (_spring_elastic_term(spring, length), "elastic_force"),
                    (_spring_damping_term(spring, rate), "damping_force"),
                    (
                        _spring_compression_stop_elastic(spring, length),
                        "compression_stop_elastic_force",
                    ),
                    (
                        _spring_rebound_stop_elastic(spring, length),
                        "rebound_stop_elastic_force",
                    ),
                    (
                        # SFORCE(id, jflag, comp, rm): jflag 0 reports at the
                        # I marker, comp 1 is the force magnitude, and rm 0
                        # expresses it in the global frame.  The two trailing
                        # slots are not marker ids.
                        f"SFORCE({identifier}, 0, 1, 0)",
                        "total_axial_force",
                    ),
                ),
            )
        for tire in self._model.tires:
            names = self._tire_variables[tire.name]
            self._request(
                f"tire:{tire.name}:normal",
                (
                    (f"IF(VARVAL({names['normal_force']}): 0, 0, 1)", "active"),
                    (f"-VARVAL({names['penetration']})", "gap"),
                    (f"MAX(0, VARVAL({names['penetration']}))", "penetration"),
                    (
                        f"-VARVAL({names['penetration_rate']})",
                        "normal_velocity",
                    ),
                    (f"VARVAL({names['normal_force']})", "normal_force"),
                    (
                        f"VARVAL({names['longitudinal_force']})",
                        "longitudinal_force",
                    ),
                ),
            )
            self._request(
                f"tire:{tire.name}:tangential",
                (
                    (f"VARVAL({names['lateral_force']})", "lateral_force"),
                    (
                        f"VARVAL({names['longitudinal_slip']})",
                        "longitudinal_slip_velocity",
                    ),
                    (
                        f"VARVAL({names['lateral_slip']})",
                        "lateral_slip_velocity",
                    ),
                    (
                        f"VARVAL({names['friction_utilization']})",
                        "friction_utilization",
                    ),
                    (f"DIF({names['brush_x']})", "brush_longitudinal"),
                    (f"DIF({names['brush_y']})", "brush_lateral"),
                ),
            )

    def _request(
        self,
        key: str,
        fields: Sequence[tuple[str, str]],
    ) -> None:
        if len(fields) > 7:
            raise RuntimeError("an Adams request carries at most seven fields")
        self._request_id += 1
        identifier = self._request_id
        results_name = key.replace(":", "_")
        self._entity_ids[f"request:{key}"] = identifier
        self._emit(f"REQUEST/{identifier}")
        self._emit(f", TITLE = {results_name}")
        self._emit(f", RESULTS_NAME = {results_name}")
        names = ", ".join(f'"{name}"' for name in ("time", *(n for _, n in fields)))
        self._emit(f", CNAMES = {names}")
        # Adams parses the F1..F8 slots in order and treats a missing F1 as the
        # start of one long expression, so the first column is written
        # explicitly.  It carries the sample time, which is what the empty
        # leading CNAME stood for.
        # Inside a REQUEST every attribute line but the last must end in a
        # backslash; without it Adams runs the F-slots together into a single
        # expression and reports a syntax error at the next slot name.
        block = [", F1 = TIME"]
        for offset, (expression, _) in enumerate(fields, start=2):
            block.extend(_continuation(expression, f", F{offset} = "))
        for line in block[:-1]:
            self._emit(line if line.endswith("\\") else line + " \\")
        self._emit(block[-1])
        self._requests.append(
            {
                "key": key,
                "id": identifier,
                "results_name": results_name,
                "components": [name for _, name in fields],
            }
        )

    def _command_text(self) -> str:
        adams_solver = cast(
            Mapping[str, object], self._manifest.payload["adams_solver"]
        )
        times = self._case.times_s
        integrator = str(adams_solver["integrator"]).lower()
        error = float(cast(float, adams_solver["error"]))
        maximum_step = float(cast(float, adams_solver["maximum_step_s"]))
        integrator_command = f"integrator/{integrator}"
        if integrator == "hht":
            alpha = float(cast(float, adams_solver["alpha"]))
            integrator_command += f", alpha = {_number(alpha)}"
        fixed_iterations = adams_solver.get("fixed_iterations")
        step_ratio = adams_solver.get("step_ratio")
        if fixed_iterations is not None or step_ratio is not None:
            if fixed_iterations is None or step_ratio is None:
                raise ValueError(
                    "fixed_iterations and step_ratio must be declared together"
                )
            integrator_command += (
                f", fixit = {int(fixed_iterations)}"
                f", hratio = {int(step_ratio)}"
            )
        # Adams reads the first two lines positionally: the dataset to load and
        # the prefix for its output files.  A provided consistent state is
        # already embedded in the shared PART poses, so it must skip sim/static
        # and start the transient from that state.
        lines = [
            f"{self._stem}.adm",
            self._stem,
            f"! generated by {AXLE_ADAMS_GENERATOR}",
            f"! manifest_sha256: {self._manifest.sha256}",
        ]
        if self._case.solver.initialization_mode == "static_equilibrium":
            lines.extend(("! independent static equilibrium before the transient", "sim/static"))
        else:
            lines.append("! provided consistent state from the shared manifest")
        lines.extend(
            (
                f"{integrator_command}, error = {_number(error)}"
                f", hmax = {_number(maximum_step)}",
                f"simulate/dynamic, end = {_number(times[-1])}"
                f", dtout = {_number(times[1] - times[0])}",
                "stop",
                "",
            )
        )
        return "\n".join(lines)


_CONVENTIONS: Mapping[str, object] = {
    "units": "SI; UNITS/FORCE=NEWTON, MASS=KILOGRAM, LENGTH=METER, TIME=SECOND",
    "dataset_angle_unit": "degree, written with an explicit D suffix",
    "fixed_bodies": "every fixed fixture body is carried by PART/1 GROUND at "
    "its frozen world pose, which is exact because a fixed body never moves",
    "part_reference_frame": "parts are written without QG, so every marker QP "
    "and REULER is global at t=0, matching Adams/View dataset exports; canonical "
    "body histories use explicit CM-marker VARIABLE expressions and do not use "
    "PART_XFORM",
    "part_pose_reconstruction": "canonical body pose, velocity, and acceleration "
    "are read from explicit CM-marker VARIABLE expressions in the ground frame; "
    "no PART_XFORM reconstruction is applied",
    "initial_state_canonicalization": "at the common t=0 sample, the canonical "
    "body quaternion is taken from the shared manifest to remove finite-precision "
    "Euler round-trip error; all later samples are read from Adams CM variables",
    "inertia": "IP = ixx, iyy, izz, ixy, ixz, iyz about the body-aligned centre "
    "of mass marker, with the Adams product convention "
    "J = [[ixx,-ixy,-ixz],[-ixy,iyy,-iyz],[-ixz,-iyz,izz]]",
    "spring_sign": "SFORCE I is the body_b attachment and J the body_a "
    "attachment; a positive function value separates I and J, matching the "
    "native force +f*e applied to body_b with e directed from A to B",
    "input_interpolation": "piecewise linear, emitted as an exact ramp sum over "
        "MAX(0, TIME - t_k), matching the native linear input interpolation",
    "damper_curve_interpolation": "piecewise linear with constant extrapolation "
    "beyond the measured velocity endpoints, matching the native curve evaluator",
    "channel_evaluation": "the 33 frozen channels are never requested from "
        "Adams; raw states and element outputs are requested and the same Python "
        "exporter evaluates the frozen formulas for both runners",
    "fixture_wrench_reconstruction": FIXTURE_WRENCH_CONVENTION,
    "validity_conditions": (
        "the native run must report a friction utilisation strictly below 1 for "
        "every tire and sample; the native kernel applies an elastoplastic "
        "return mapping to the stored brush state after each accepted step, "
        "which has no Adams/Solver counterpart and is a no-op only while the "
        "friction ellipse is not reached",
    ),
    "unverified_note": "this metadata is not a correlation claim; the run-specific "
    "equivalence_audit.json and dynamic_comparison.json are authoritative, with "
    "raw .adm/.acf/.msg/.res evidence retained beside them",
}


# -- expression helpers -----------------------------------------------------


def _spring_elastic_term(spring: AxleSpringDamper, length: str) -> str:
    return (
        f"{_number(spring.stiffness_n_per_m)}"
        f"*({_number(spring.free_length_m)} - {length})"
    )


def _spring_damping_term(
    spring: AxleSpringDamper,
    rate: str,
    *,
    damper_curve_expression: str | None = None,
) -> str:
    if damper_curve_expression is not None:
        return f"-({damper_curve_expression})"
    return (
        f"-IF({rate}: {_number(spring.compression_damping_n_s_per_m)}"
        f", {_number(spring.rebound_damping_n_s_per_m)}"
        f", {_number(spring.rebound_damping_n_s_per_m)})*{rate}"
    )


def _spring_compression_stop_elastic(
    spring: AxleSpringDamper, length: str
) -> str:
    if spring.minimum_length_m is None:
        return "0"
    minimum = _number(spring.minimum_length_m)
    return (
        f"IF({length} - {minimum}: "
        f"{_number(spring.compression_stop_stiffness_n_per_m)}"
        f"*({minimum} - {length}), 0, 0)"
    )


def _spring_rebound_stop_elastic(spring: AxleSpringDamper, length: str) -> str:
    if spring.maximum_length_m is None:
        return "0"
    maximum = _number(spring.maximum_length_m)
    return (
        f"IF({length} - {maximum}: 0, 0"
        f", -{_number(spring.rebound_stop_stiffness_n_per_m)}"
        f"*({length} - {maximum}))"
    )


def _spring_function(
    spring: AxleSpringDamper,
    i_marker: int,
    j_marker: int,
    *,
    damper_curve_expression: str | None = None,
) -> str:
    length = f"DM({i_marker}, {j_marker})"
    rate = f"VR({i_marker}, {j_marker})"
    terms = [
        _spring_elastic_term(spring, length),
        _spring_damping_term(
            spring,
            rate,
            damper_curve_expression=damper_curve_expression,
        ),
    ]
    if spring.minimum_length_m is not None:
        minimum = _number(spring.minimum_length_m)
        stop = (
            f"{_number(spring.compression_stop_stiffness_n_per_m)}"
            f"*({minimum} - {length})"
        )
        if spring.compression_stop_damping_n_s_per_m > 0.0:
            stop += (
                f" + IF({rate}: "
                f"-{_number(spring.compression_stop_damping_n_s_per_m)}*{rate}"
                ", 0, 0)"
            )
        terms.append(f"IF({length} - {minimum}: {stop}, 0, 0)")
    if spring.maximum_length_m is not None:
        maximum = _number(spring.maximum_length_m)
        stop = (
            f"-{_number(spring.rebound_stop_stiffness_n_per_m)}"
            f"*({length} - {maximum})"
        )
        if spring.rebound_stop_damping_n_s_per_m > 0.0:
            stop += (
                f" + IF({rate}: 0, 0"
                f", -{_number(spring.rebound_stop_damping_n_s_per_m)}*{rate})"
            )
        terms.append(f"IF({length} - {maximum}: 0, 0, {stop})")
    return " + ".join(f"({term})" for term in terms)


def _linear_breakpoints(
    times: Sequence[float],
    values: Sequence[float],
) -> tuple[int, ...]:
    """Return the indices that reproduce the samples by linear interpolation."""
    if len(times) != len(values):
        raise ValueError("input signal length does not match the public grid")
    if len(times) < 2:
        raise ValueError("input signal requires at least two samples")
    kept = [0]
    for index in range(1, len(times) - 1):
        previous = kept[-1]
        span = times[index + 1] - times[previous]
        weight = (times[index] - times[previous]) / span
        interpolated = (
            values[previous] * (1.0 - weight) + values[index + 1] * weight
        )
        scale = max(
            abs(values[previous]),
            abs(values[index]),
            abs(values[index + 1]),
            1.0,
        )
        if abs(interpolated - values[index]) > _COLLINEAR_TOLERANCE * scale:
            kept.append(index)
    kept.append(len(times) - 1)
    return tuple(kept)


def _ramp_sum_expression(
    times: Sequence[float],
    values: Sequence[float] | None,
) -> str:
    """Write an exact piecewise-linear signal as a sum of one-sided ramps."""
    if values is None:
        return "0"
    knots = [(times[index], values[index]) for index in _linear_breakpoints(times, values)]
    if all(value == knots[0][1] for _, value in knots):
        return _number(knots[0][1])
    terms = [_number(knots[0][1])]
    previous_slope = 0.0
    for (time_a, value_a), (time_b, value_b) in zip(knots, knots[1:]):
        slope = (value_b - value_a) / (time_b - time_a)
        if slope != previous_slope:
            terms.append(
                f"{_number(slope - previous_slope)}"
                f"*MAX(0, TIME - {_number(time_a)})"
            )
        previous_slope = slope
    if previous_slope != 0.0:
        terms.append(
            f"{_number(-previous_slope)}*MAX(0, TIME - {_number(knots[-1][0])})"
        )
    return " + ".join(terms)


def _continuation(
    expression: str,
    prefix: str,
    *,
    width: int = 76,
) -> tuple[str, ...]:
    """
    Split one Adams attribute over continued dataset lines.

    A wrapped line must not start with a comma: Adams reads a leading comma as
    the start of the next argument, so an expression broken that way is
    reported as "an argument keyword was expected here".  Continued lines are
    indented instead, and the caller appends the backslashes.
    """
    # Adams rejects a break inside a function expression: the continuation is
    # scanned for a statement keyword, so `DY(...)` on its own line is reported
    # as an unknown token.  One attribute therefore stays on one line, however
    # long, and only whole attributes are separated.
    del width
    return (prefix + expression,)


# -- numeric helpers --------------------------------------------------------


def _number(value: float) -> str:
    return f"{float(value):.12g}"


def _inertia_fields(inertia: Sequence[Sequence[float]]) -> str:
    matrix = np.asarray(inertia, dtype=float)
    diagonal = (matrix[0][0], matrix[1][1], matrix[2][2])
    products = (-matrix[0][1], -matrix[0][2], -matrix[1][2])
    values = diagonal if not any(products) else (*diagonal, *products)
    return ", ".join(_number(value) for value in values)


def _nonzero(values: Sequence[float]) -> bool:
    return any(value != 0.0 for value in values)


def _matrix_nonzero(matrix: Sequence[Sequence[float]]) -> bool:
    return bool(np.any(np.asarray(matrix, dtype=float) != 0.0))


def _piecewise_linear_expression(
    argument: str,
    x_values: Sequence[float],
    y_values: Sequence[float],
) -> str:
    """Emit the native curve as an exact sum of one-sided Adams ramps."""
    if len(x_values) != len(y_values) or len(x_values) < 2:
        raise ValueError("a piecewise-linear expression needs paired curve points")
    knots = _linear_breakpoints(x_values, y_values)
    terms = [_number(y_values[knots[0]])]
    previous_slope = 0.0
    for index_a, index_b in zip(knots, knots[1:]):
        slope = (y_values[index_b] - y_values[index_a]) / (
            x_values[index_b] - x_values[index_a]
        )
        delta_slope = slope - previous_slope
        if delta_slope != 0.0:
            sign = "+" if delta_slope > 0.0 else "-"
            terms.append(
                f"{sign} {_number(abs(delta_slope))}*MAX(0, "
                f"{_subtract_constant(argument, x_values[index_a])})"
            )
        previous_slope = slope
    if previous_slope != 0.0:
        sign = "+" if previous_slope < 0.0 else "-"
        terms.append(
            f"{sign} {_number(abs(previous_slope))}*MAX(0, "
            f"{_subtract_constant(argument, x_values[knots[-1]])})"
        )
    return " ".join(terms)


def _subtract_constant(argument: str, value: float) -> str:
    """Format a difference without generating Adams' ambiguous double minus."""
    if value < 0.0:
        return f"({argument}) + {_number(-value)}"
    return f"({argument}) - {_number(value)}"


def _quaternions_equal(
    left: Sequence[float],
    right: Sequence[float],
) -> bool:
    a = np.asarray(left, dtype=float)
    b = np.asarray(right, dtype=float)
    return bool(
        np.allclose(a, b, atol=1e-12, rtol=0.0)
        or np.allclose(a, -b, atol=1e-12, rtol=0.0)
    )


def _matrix_has_rotational_terms(matrix: Sequence[Sequence[float]]) -> bool:
    values = np.asarray(matrix, dtype=float)
    return bool(np.any(values[3:, :] != 0.0) or np.any(values[:, 3:] != 0.0))


def _unit(vector: Sequence[float]) -> np.ndarray:
    values = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("cannot normalise a zero vector")
    return values / norm


def _orthonormal_frame(
    z_local: Sequence[float] | None,
    x_local: Sequence[float] | None,
) -> np.ndarray:
    if z_local is None and x_local is None:
        return np.eye(3)
    z_axis = (
        _unit(z_local)
        if z_local is not None
        else np.asarray((0.0, 0.0, 1.0), dtype=float)
    )
    seed = (
        np.asarray(x_local, dtype=float)
        if x_local is not None
        else (
            np.asarray((1.0, 0.0, 0.0), dtype=float)
            if abs(float(z_axis[0])) < 0.8
            else np.asarray((0.0, 1.0, 0.0), dtype=float)
        )
    )
    x_axis = _unit(seed - z_axis * float(np.dot(seed, z_axis)))
    return np.column_stack((x_axis, np.cross(z_axis, x_axis), z_axis))


def _rotation_matrix(quaternion: Sequence[float]) -> np.ndarray:
    w, x, y, z = _unit(quaternion)
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _euler_313_degrees(rotation: np.ndarray) -> tuple[float, float, float]:
    """Return the body-fixed z-x-z Euler angles used by Adams ``REULER``."""
    matrix = np.asarray(rotation, dtype=float)
    cos_theta = min(1.0, max(-1.0, float(matrix[2][2])))
    theta = math.acos(cos_theta)
    if math.sqrt(max(0.0, 1.0 - cos_theta * cos_theta)) <= 1e-12:
        phi = 0.0
        psi = (
            math.atan2(float(matrix[1][0]), float(matrix[0][0]))
            if cos_theta > 0.0
            else math.atan2(float(matrix[0][1]), float(matrix[0][0]))
        )
    else:
        psi = math.atan2(float(matrix[0][2]), -float(matrix[1][2]))
        phi = math.atan2(float(matrix[2][0]), float(matrix[2][1]))
    return math.degrees(psi), math.degrees(theta), math.degrees(phi)


def _harmonic_expression(road: AxleHarmonicRoad, label: str) -> str:
    """
    Write the harmonic road as an Adams run-time expression.

    Both solvers then evaluate the same closed form, so the comparison never
    tests one side's interpolation against the other's.
    """
    rate = 2.0 * math.pi * road.frequency_hz
    angle = f"{_number(rate)}*TIME + {_number(road.phase_rad)}"
    if label == "road_height":
        return (
            f"{_number(road.offset_m)} + "
            f"{_number(road.amplitude_m)}*SIN({angle})"
        )
    return f"{_number(road.amplitude_m * rate)}*COS({angle})"
