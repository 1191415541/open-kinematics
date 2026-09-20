#!/usr/bin/env python
"""
Gate script: every case family, judged on its own.

The acceptance for the case layer is per family, not in aggregate: a run that
covers seven of the eight families has not covered the eighth, and a green
aggregate would hide exactly that.  Each family below states how it is judged
and against what reference; a family with no implementation is reported as
missing and fails the gate, because "not supported yet" is a status a reader
needs to see rather than a silence they have to discover.

Two references are in use and they mean different things:

* `kc_quasi_static` and `axle_dynamic` are judged against the *frozen Python
  snapshot* and the *ctypes entry point* respectively.  The first is a tolerance
  comparison, because the snapshot is a different solver; the second is exact,
  because the ctypes entry point is the same kernel reached a different way.
* A family that is declared in the contract but has no case-layer expansion is
  reported missing.

Run with `--allow-partial` while the migration is in flight: the report is the
same, but an unimplemented family is a warning instead of a failure.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any, Callable, cast

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
TEST_DATA = REPOSITORY_ROOT / "packages/suspension_multibody/tests/data/kc_baseline"

#: Every family the case contract declares, in the order the contract lists them.
FAMILIES = (
    "kc_quasi_static",
    "axle_dynamic",
    "vehicle_kc",
    "vehicle_dynamic",
    "handling",
    "ride_four_post",
    "ride_random_road",
    "comparison",
)

_DIAGNOSTIC_FIELDS = (
    "accepted",
    "internal_steps",
    "rejected_attempts",
    "newton_iterations",
    "minimum_accepted_step_s",
    "maximum_accepted_step_s",
    "last_accepted_step_s",
    "position_residual",
    "velocity_residual",
    "dynamics_residual",
    "active_contacts",
    "contact_events",
    "local_error_ratio",
    "energy_residual",
    "failure_code",
    "pinned_null_directions",
)


def _tolerance(field: str, reference: float, index: int | None = None) -> float:
    """Return the strict tolerance for one K or C field."""
    magnitude = abs(reference)
    if field.endswith("_mm"):
        return 0.1 + 0.002 * magnitude
    if field.endswith("_deg"):
        return 0.02 + 0.005 * magnitude
    if index is not None:
        return (1e-6 + 1e-4 * magnitude) if index < 3 else (1e-8 + 1e-4 * magnitude)
    return 1e-6 + 1e-4 * magnitude


def _load_acceptance():
    path = (
        REPOSITORY_ROOT
        / "packages/suspension_multibody/scripts/run_axle_dynamics_acceptance.py"
    )
    spec = importlib.util.spec_from_file_location("case_parity_acceptance", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_vehicle_fixture():
    """Load the native vehicle fixtures so both gates use one model."""
    path = (
        REPOSITORY_ROOT
        / "packages/suspension_multibody/tests/vehicle/test_native_vehicle.py"
    )
    spec = importlib.util.spec_from_file_location("case_parity_vehicle", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _k_grid_records(assembly) -> list[dict[str, object]]:
    """Solve the K grid through the unified simulation service."""
    from suspension_multibody.cases.kc_quasi_static import (
        NativeKcError,
        case_document,
        model_document,
    )
    from suspension_multibody.cases.kc_quasi_static.workflow import (
        DEFAULT_SETTINGS,
        DEFAULT_TIMES,
        _side_fields,
    )
    from suspension_multibody.simulation import SimulationRequest, run_request

    wheels = (-10.0, 0.0, 10.0)
    racks = (-5.0, 0.0, 5.0)
    model = model_document(assembly, name="native-k", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-k",
        wheel_values_mm=wheels,
        rack_values_mm=racks,
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=case,
        )
    ).raw
    left_states = run.body_state("upright_L")
    right_states = run.body_state("upright_R")
    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases):
        wheel = wheels[index // len(racks)]
        rack = racks[index % len(racks)]
        case_id = f"k-w{wheel:+.0f}-r{rack:+.0f}"
        # The case layer expands the grid in document order; checking the name
        # it reported turns a silent reordering into a failure.
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        record: dict[str, object] = {
            "case_id": case_id,
            "wheel_travel_mm": float(wheel),
            "rack_displacement_mm": float(rack),
        }
        record.update(
            _side_fields(assembly, "L", left_states[last, :3], left_states[last, 3:7])
        )
        record.update(
            _side_fields(assembly, "R", right_states[last, :3], right_states[last, 3:7])
        )
        records.append(record)
    return records


def _c_path_records(assembly, *, paths: tuple[str, ...]) -> list[dict[str, object]]:
    """Solve the C load paths through the unified simulation service."""
    from suspension_multibody.cases.kc_quasi_static import (
        AXIS_ORDER,
        MM,
        NativeKcError,
        case_document,
        model_document,
        quaternion_to_rotation,
    )
    from suspension_multibody.cases.kc_quasi_static.workflow import (
        DEFAULT_SETTINGS,
        DEFAULT_TIMES,
        SIDES,
        _assembling_pose,
        _side_fields,
        quaternion_conjugate,
        quaternion_multiply,
        wheel_center_world,
    )
    from suspension_multibody.core import quaternion_to_rotation_vector
    from suspension_multibody.simulation import SimulationRequest, run_request

    levels = 11
    maximum = 1.0
    model = model_document(assembly, name="native-c", drive_wheels=False)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-c",
        paths=tuple(paths),
        levels=levels,
        maximum=maximum,
        side_mode="single",
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=False,
    )
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=case,
        )
    ).raw
    left_states = run.body_state("upright_L")
    right_states = run.body_state("upright_R")
    # The C deformation is measured against the neutral K pose, which is the
    # assembling pose: at the design separation every driven target has zero
    # residual, so the reference is the model document's own initial state.
    reference = {side: _assembling_pose(model, side) for side in SIDES}
    # The K reference the C response is measured from.  The driven case's zero
    # target resolves to the separation the model was assembled with, so the
    # assembling pose *is* the K reference -- the same thing the Python solver's
    # `KReferenceCache` solves for, reached without solving it again.
    reference_metrics = {
        ("left" if side == "L" else "right"): _side_fields(
            assembly, side, reference[side][0], reference[side][1]
        )
        for side in SIDES
    }

    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases):
        axis = paths[index // levels]
        position_in_path = index % levels
        if position_in_path == 0:
            level = -maximum
        elif position_in_path == levels - 1:
            level = maximum
        else:
            level = -maximum + position_in_path * (2.0 * maximum / (levels - 1))
        case_id = f"c-{axis}-{level:+.2f}"
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        load = [0.0] * 6
        load[AXIS_ORDER.index(axis)] = float(level)
        record = {
            "case_id": case_id,
            "path": axis,
            "level": float(level),
            "side_mode": "single",
            "load_left": list(load),
            "load_right": [0.0] * 6,
        }
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        metrics: dict[str, dict[str, float]] = {}
        for side, key, states in (
            ("L", "deformation_left", left_states),
            ("R", "deformation_right", right_states),
        ):
            state = states[last]
            metrics["left" if side == "L" else "right"] = _side_fields(
                assembly, side, state[:3], state[3:7]
            )
            ref_position, ref_quaternion = reference[side]
            centre = wheel_center_world(assembly, side, state[:3], state[3:7])
            ref_centre = wheel_center_world(assembly, side, ref_position, ref_quaternion)
            relative = quaternion_multiply(
                quaternion_conjugate(np.asarray(ref_quaternion, dtype=float)),
                np.asarray(state[3:7], dtype=float),
            )
            rotation = quaternion_to_rotation(
                ref_quaternion
            ) @ quaternion_to_rotation_vector(relative)
            record[key] = [
                float(value)
                for value in np.concatenate(((centre - ref_centre) / MM, rotation))
            ]
        record["metrics"] = metrics
        record["c_minus_k"] = {
            key: float(value - reference_metrics[side][key])
            for side in ("left", "right")
            for key, value in metrics[side].items()
        }
        records.append(record)
    return records


def check_kc_quasi_static() -> tuple[bool, str]:
    """Compare the contract-path K grid and C load paths with the snapshot."""
    from suspension_multibody.analysis.benchmarks import benchmark_model
    from suspension_multibody.cases.kc_quasi_static import AXIS_ORDER
    from suspension_multibody.model import build_front_axle

    # Loaded by path rather than imported: the fixture is a test module, and a
    # module reached through `sys.path` is neither resolvable by a type checker
    # nor obviously a test dependency.  `_load_vehicle_fixture` does the same.
    fixture = (
        REPOSITORY_ROOT
        / "packages/suspension_multibody/tests/cases/kc_quasi_static/kc_fixtures.py"
    )
    spec = importlib.util.spec_from_file_location("kc_parity_fixture", fixture)
    assert spec is not None and spec.loader is not None
    fixture_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture_module)
    _compliant_model = getattr(fixture_module, "_compliant_model")  # noqa: SLF001
    worst = 0.0
    k_expected = {
        state["case_id"]: state
        for state in json.loads((TEST_DATA / "k_states.json").read_text(encoding="utf-8"))
    }
    k_produced = _k_grid_records(build_front_axle(benchmark_model(), "K"))
    if {state["case_id"] for state in k_produced} != set(k_expected):
        return False, "the K grid did not cover the frozen case set"
    for state in k_produced:
        reference = k_expected[state["case_id"]]
        for field, value in state.items():
            if field not in reference or not isinstance(value, float):
                continue
            ratio = abs(value - float(reference[field])) / _tolerance(
                field, float(reference[field])
            )
            worst = max(worst, ratio)

    c_expected = {
        state["case_id"]: state
        for state in json.loads((TEST_DATA / "c_states.json").read_text(encoding="utf-8"))
    }
    c_produced = _c_path_records(
        build_front_axle(_compliant_model(), "C"), paths=AXIS_ORDER
    )
    if {state["case_id"] for state in c_produced} != set(c_expected):
        return False, "the C load paths did not cover the frozen case set"
    for state in c_produced:
        reference = c_expected[state["case_id"]]
        for key in ("deformation_left", "deformation_right"):
            actual = np.asarray(state[key], dtype=float)
            wanted = np.asarray(reference[key], dtype=float)
            for index in range(6):
                ratio = abs(actual[index] - wanted[index]) / _tolerance(
                    key, wanted[index], index
                )
                worst = max(worst, ratio)
    passed = worst < 1.0
    return passed, f"worst error / tolerance {worst:.6g} over the frozen K/C snapshot"


#: The frozen axle-dynamics snapshot.  It was recorded from the flat ctypes route
#: (`axle_run`) just before the public `run_axle_dynamics` moved to the contract,
#: which is the only moment at which the independent implementation could still
#: be asked: the digests are of the float64 bytes of each array, so a match is
#: bit-identity and the arrays themselves do not have to be committed.  That
#: route is no longer exported by the kernel, which is exactly why the snapshot
#: was taken first -- the reference outlives the implementation it came from.
_AXLE_BASELINE = (
    REPOSITORY_ROOT
    / "packages/suspension_multibody/tests/data/axle_dynamics_baseline/sha256.json"
)

#: The arrays the snapshot covers, in the order the recorder wrote them.
_AXLE_LEDGERS = (
    "states",
    "constraint_wrench",
    "spring_output",
    "bushing_output",
    "anti_roll_output",
    "tire_output",
    "energy",
)


def _array_digest(array: np.ndarray) -> str:
    import hashlib

    return hashlib.sha256(
        np.ascontiguousarray(array, dtype=np.float64).tobytes()
    ).hexdigest()


def check_axle_dynamic() -> tuple[bool, str]:
    """Compare the axle family's public run with the frozen ctypes snapshot."""
    from suspension_multibody.axle_dynamics import run_axle_dynamics

    acceptance = _load_acceptance()
    expected = json.loads(_AXLE_BASELINE.read_text(encoding="utf-8"))["cases"]
    model = acceptance.build_axle_model()
    failures: list[str] = []
    for case_name in acceptance._CASE_DURATIONS:
        case = acceptance.build_case(case_name)
        result = run_axle_dynamics(model, case)
        for field in _AXLE_LEDGERS:
            produced = _array_digest(np.asarray(getattr(result, field)))
            if produced != expected[case_name][field]:
                failures.append(f"{case_name}: {field} differs from the snapshot")
    if failures:
        return False, "; ".join(failures)
    return True, f"{len(acceptance._CASE_DURATIONS)} cases, bit-identical to the frozen snapshot"


#: The frozen vehicle-dynamics snapshot, recorded from the ctypes vehicle entry
#: just before the public `run_vehicle_dynamics` moved to the contract.  Digests
#: of the float64 bytes: a match is bit-identity, and the arrays do not have to
#: be committed for that claim to be checkable.
_VEHICLE_BASELINE = (
    REPOSITORY_ROOT
    / "packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/sha256.json"
)

_VEHICLE_LEDGERS = (
    "states",
    "constraint_wrench",
    "spring_output",
    "bushing_output",
    "anti_roll_output",
    "tire_output",
    "energy",
)


def _vehicle_diagnostics_digest(diagnostics) -> str:
    """Hash every diagnostics column together with its type and shape."""
    import hashlib

    digest = hashlib.sha256()
    for field in _DIAGNOSTIC_FIELDS:
        array = np.ascontiguousarray(getattr(diagnostics, field))
        digest.update(field.encode("utf-8"))
        digest.update(str(array.dtype).encode("ascii"))
        digest.update(repr(tuple(int(extent) for extent in array.shape)).encode("ascii"))
        digest.update(array.tobytes())
    return digest.hexdigest()


def _vehicle_digests(result) -> dict[str, str]:
    """Return the stable digest set for one vehicle result."""
    digests = {
        field: _array_digest(np.asarray(getattr(result.axle, field)))
        for field in _VEHICLE_LEDGERS
    }
    steering = result.steering_output
    digests["steering_output"] = (
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        if steering is None
        else _array_digest(np.asarray(steering))
    )
    digests["diagnostics"] = _vehicle_diagnostics_digest(result.axle.diagnostics)
    return digests


def _vehicle_case_matrix(fixture):
    """Return the coverage matrix shared by the snapshot recorder and the live gate."""
    from suspension_multibody.schema import (
        Bushing6x6,
        Pose,
        RoadSurfaceSpec,
        TimeSignal,
        Vec3,
    )

    base = fixture._positioned_vehicle(fixture._vehicle())
    pac2002 = fixture._positioned_vehicle(fixture._pac2002_model(combined=True))
    pac2002_adams = fixture._positioned_vehicle(
        fixture._pac2002_model(combined=True, parameter_source="adams_builtin")
    )
    non_default = fixture._case(base).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": fixture._uniform_velocity_initial_states(base),
        }
    )
    table = ((0.0, 0.0), (0.01, 1_000.0), (0.02, 2_000.0))
    tabulated = pac2002.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_tables": {
                                    "deflection_load_curve": table
                                }
                            }
                        )
                    }
                )
                for wheel in pac2002.wheels
            )
        }
    )
    point = base.front_axle.hardpoints["UPPER_INBOARD_FRONT"]
    bushing = Bushing6x6(
        name="curve_bushing",
        body_a="chassis",
        body_b="upper_arm",
        pose_a=Pose(translation=point),
        pose_b=Pose(
            translation=Vec3(x=point.x, y=point.y, z=point.z + 50.0)
        ),
        stiffness=((0.0,) * 6,) * 6,
        damping=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        rotation_coordinates="cardan_xyz",
        force_curves=(
            (),
            (),
            ((-100.0, -100.0), (0.0, 0.0), (100.0, 100.0)),
            (),
            (),
            (),
        ),
    )
    bushings = base.model_copy(
        update={
            "front_axle": base.front_axle.model_copy(
                update={"bushings": (bushing,)}
            )
        }
    )
    return {
        "default": (base, fixture._case(base)),
        "braking": (base, fixture._case(base, brake=0.4)),
        "steering": (
            base,
            fixture._case(base, steering=TimeSignal(constant=0.05)),
        ),
        "pac2002": (pac2002, fixture._case(pac2002)),
        "pac2002 adams": (pac2002_adams, fixture._case(pac2002_adams)),
        "nondefault road and initial state": (base, non_default),
        "measured tire table": (tabulated, fixture._case(tabulated)),
        "bushing force curves": (bushings, fixture._case(bushings)),
    }


def check_vehicle_dynamic() -> tuple[bool, str]:
    """Compare the public contract route with the frozen ctypes snapshot."""
    from suspension_multibody.vehicle.service import run_vehicle_dynamics

    fixture = _load_vehicle_fixture()
    cases = _vehicle_case_matrix(fixture)
    expected = json.loads(_VEHICLE_BASELINE.read_text(encoding="utf-8"))["cases"]
    failures: list[str] = []
    for name, (model, case) in cases.items():
        produced = _vehicle_digests(run_vehicle_dynamics(model, case))
        for field, digest in expected[name].items():
            if produced.get(field) != digest:
                failures.append(f"{name}: {field} differs from the snapshot")
    if failures:
        return False, "; ".join(failures)
    return True, f"{len(cases)} cases, bit-identical to the frozen snapshot"


def check_ride_four_post() -> tuple[bool, str]:
    """
    Judge the four-post expansion against an independently sampled excitation.

    A family whose job is to expand a declaration can only be checked by
    expanding it twice: once in the kernel and once here, and running both.
    """
    from dataclasses import replace

    from suspension_contracts import pack_container

    from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
    from suspension_multibody.cases import (
        FourPostCorner,
        ride_four_post_case_document,
        ride_four_post_corner_signals,
        vehicle_dynamic_model_document,
    )
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
    from suspension_multibody.simulation import SimulationRequest, run_request

    fixture = _load_vehicle_fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    case = fixture._case(model)
    prepared = prepare_vehicle_run(model, case)
    corners = (
        FourPostCorner("front_left", amplitude_m=0.002, frequency_hz=8.0),
        FourPostCorner("front_right", amplitude_m=0.002, frequency_hz=8.0),
        FourPostCorner("rear_left", amplitude_m=0.0015, frequency_hz=6.0, phase_rad=0.3),
        FourPostCorner("rear_right", amplitude_m=0.0015, frequency_hz=6.0, phase_rad=-0.3),
    )
    times = tuple(float(value) for value in prepared.times)
    document = ride_four_post_case_document(
        name="ride-four-post", corners=corners, times_s=times,
        settings=AxleSolverSettings(),
    )
    model_document, model_blob = vehicle_dynamic_model_document(model, prepared)
    model_payload = pack_container(model_document, model_blob)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="ride_four_post",
            model=model_document,
            case=document,
            context={"model_payload": model_payload},
        )
    ).raw

    height, velocity = ride_four_post_corner_signals(corners, times)
    # The explicit reference has to run on the solver block the family document
    # declares, so it is prepared with that solver rather than the fixture
    # case's own settings.
    explicit = replace(
        prepared,
        road_height=height,
        road_velocity=velocity,
        solver=AxleSolverSettings(),
    )
    reference = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": explicit},
        )
    ).raw
    difference = float(
        np.abs(produced.block("body_state") - reference.block("body_state")).max()
    )
    if difference >= 1e-12:
        return False, f"the expansion differs from an explicit excitation by {difference:.3e}"
    return True, f"expansion matches an independently sampled excitation ({difference:.1e})"


def check_handling() -> tuple[bool, str]:
    """
    Judge the handling expansion against an independently expanded manoeuvre.

    The same shape of check the four-post family gets, for the same reason: the
    family expands a declaration, so the declaration is expanded twice and both
    runs are compared.  The scope is the open-loop manoeuvres; a closed-loop one
    is a driver model and is refused by name, which this check also confirms.
    """
    from dataclasses import replace

    from suspension_contracts import pack_container

    from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
    from suspension_multibody.cases import (
        SteeringShape,
        handling_case_document,
        handling_steering_signals,
        vehicle_dynamic_model_document,
    )
    from suspension_multibody.kernel import KernelContractError
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
    from suspension_multibody.schema import Vec3
    from suspension_multibody.simulation import SimulationRequest, run_request

    fixture = _load_vehicle_fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    base_case = fixture._case(model)
    # The shared fixture runs with gravity switched off, which leaves no load
    # path to anchor the vehicle; a manoeuvre happens on the ground.
    case = base_case.model_copy(
        update={
            "solver": base_case.solver.model_copy(
                update={
                    "gravity": Vec3(x=0.0, y=0.0, z=-9806.65),
                    "end_time": 0.004,
                    "step_size": 0.001,
                    "internal_step_size": 0.001,
                    "min_internal_step_size": 0.001,
                }
            )
        }
    )
    prepared = prepare_vehicle_run(model, case)
    times = tuple(float(value) for value in prepared.times)
    actuator = prepared.steering.names[0]
    model_document, model_blob = vehicle_dynamic_model_document(model, prepared)
    model_payload = pack_container(model_document, model_blob)

    worst = 0.0
    for shape in ("constant", "ramp", "step", "sine"):
        shapes = (
            SteeringShape(
                actuator, shape, amplitude=0.008, start_s=5e-4, rise_s=5e-4,
                frequency_hz=4.0,
            ),
        )
        document = handling_case_document(
            name=f"handling-{shape}", shapes=shapes, times_s=times,
            settings=AxleSolverSettings(),
        )
        produced = run_request(
            SimulationRequest(
                assembly="vehicle",
                family="handling",
                model=model_document,
                case=document,
                context={"model_payload": model_payload},
            )
        ).raw
        target, rate = handling_steering_signals(shapes, times)
        steering = replace(
            prepared.steering,
            target=np.asarray(target[actuator], dtype=float),
            target_rate=np.asarray(rate[actuator], dtype=float),
        )
        # The explicit reference has to run on the solver block the family
        # document declares, so it is prepared with that solver rather than the
        # fixture case's own settings.
        explicit = replace(
            prepared, steering=steering, solver=AxleSolverSettings()
        )
        reference = run_request(
            SimulationRequest(
                assembly="vehicle",
                family="vehicle_dynamic",
                model=model,
                case=case,
                context={"prepared": explicit},
            )
        ).raw
        difference = float(
            np.abs(produced.block("body_state") - reference.block("body_state")).max()
        )
        worst = max(worst, difference)

    # A closed-loop manoeuvre has to be refused rather than approximated.
    closed = handling_case_document(
        name="handling-closed",
        shapes=(SteeringShape(actuator, "ramp", amplitude=0.008, rise_s=5e-4),),
        times_s=times,
        settings=AxleSolverSettings(),
    )
    closed["handling"]["steering"][0]["shape"] = "iso_lane_change"
    try:
        run_request(
            SimulationRequest(
                assembly="vehicle",
                family="handling",
                model=model_document,
                case=closed,
                context={"model_payload": model_payload},
            )
        )
    except KernelContractError:
        pass
    else:
        return False, "a closed-loop manoeuvre was accepted instead of refused"
    if worst >= 1e-12:
        return False, f"the expansion differs from an explicit manoeuvre by {worst:.3e}"
    return True, f"4 open-loop shapes match an independent expansion ({worst:.1e}); closed-loop refused"


def check_ride_random_road() -> tuple[bool, str]:
    """
    Judge the random-road expansion against an independently expanded profile.

    The spatial-to-temporal conversion is the whole family, so it is performed
    twice -- once by the kernel, once here -- and the two runs are compared.
    """
    from dataclasses import replace

    from suspension_contracts import pack_container

    from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
    from suspension_multibody.cases import (
        RandomRoadWheel,
        RoadComponent,
        ride_random_road_case_document,
        ride_random_road_signals,
        vehicle_dynamic_model_document,
    )
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
    from suspension_multibody.simulation import SimulationRequest, run_request

    fixture = _load_vehicle_fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    case = fixture._case(model)
    prepared = prepare_vehicle_run(model, case)
    times = tuple(float(value) for value in prepared.times)
    profile = (
        RoadComponent(amplitude_m=0.004, wavelength_m=25.0),
        RoadComponent(amplitude_m=0.002, wavelength_m=8.0, phase_rad=0.7),
    )
    speed = 20.0
    wheels = []
    for wheel in model.wheels:
        delay = 0.0 if wheel.name.startswith("front") else 2.4 / speed
        wheels.append(
            RandomRoadWheel(
                wheel.name,
                tuple(
                    RoadComponent(
                        amplitude_m=component.amplitude_m,
                        wavelength_m=component.wavelength_m,
                        phase_rad=component.phase_rad
                        - 2.0 * np.pi * delay * speed / component.wavelength_m,
                    )
                    for component in profile
                ),
            )
        )
    wheels = tuple(wheels)
    document = ride_random_road_case_document(
        name="ride-random-road", wheels=wheels, speed_mps=speed, times_s=times,
        settings=AxleSolverSettings(),
    )
    model_document, model_blob = vehicle_dynamic_model_document(model, prepared)
    model_payload = pack_container(model_document, model_blob)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="ride_random_road",
            model=model_document,
            case=document,
            context={"model_payload": model_payload},
        )
    ).raw

    height, velocity = ride_random_road_signals(wheels, speed, times)
    # The explicit reference has to run on the solver block the family document
    # declares, so it is prepared with that solver rather than the fixture
    # case's own settings.
    explicit = replace(
        prepared,
        road_height=height,
        road_velocity=velocity,
        solver=AxleSolverSettings(),
    )
    reference = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": explicit},
        )
    ).raw
    difference = float(
        np.abs(produced.block("body_state") - reference.block("body_state")).max()
    )
    if difference >= 1e-12:
        return False, f"the expansion differs from an explicit profile by {difference:.3e}"
    return True, f"expansion matches an independently expanded profile ({difference:.1e})"


#: The compliant vehicle the K/C family needs.  A rigid vehicle refuses the
#: driven set -- its analytic Jacobian's pivot sits just under the rank
#: threshold while central differences put it just above -- so the gate builds
#: the same bushed model the family's own tests use.
_VEHICLE_KC_STIFFNESS = tuple(
    tuple(
        10_000.0 if row == column and row < 3
        else 10_000_000.0 if row == column
        else 0.0
        for column in range(6)
    )
    for row in range(6)
)

#: The ramp window and its sampling, for the same reason the family's tests
#: carry them: the raised-cosine ramp has to be resolved by the adaptive
#: stepper, and 2 samples over it is rejected at ``t = 0``.
_VEHICLE_KC_WINDOW_S = 2e-2
_VEHICLE_KC_SAMPLES = 21

#: The four driven wheel coordinates the sweep names, in grid order.
_VEHICLE_KC_WHEELS = (
    "front_upright_L",
    "front_upright_R",
    "rear_upright_L",
    "rear_upright_R",
)


def _bushed(axle):
    """Give one axle the four compliant inboard mounts the C mode expects."""
    from suspension_multibody.schema import Bushing6x6, Pose, Vec3

    def mount(name):
        point = axle.hardpoints[name]
        return Pose(
            translation=Vec3(x=float(point.x), y=float(point.y), z=float(point.z))
        )

    return axle.model_copy(
        update={
            "bushings": tuple(
                Bushing6x6(
                    name=f"{body}_{index}",
                    body_a="chassis",
                    body_b=body,
                    pose_a=mount(name),
                    pose_b=mount(name),
                    stiffness=_VEHICLE_KC_STIFFNESS,
                )
                for body, names in (
                    ("upper_arm", ("UPPER_INBOARD_FRONT", "UPPER_INBOARD_REAR")),
                    ("lower_arm", ("LOWER_INBOARD_FRONT", "LOWER_INBOARD_REAR")),
                )
                for index, name in enumerate(names)
            )
        }
    )


def check_vehicle_kc() -> tuple[bool, str]:
    """
    Judge the vehicle K/C sweep on its own terms.

    This family is a new capability, so it has no second implementation to be
    held against.  What it does have is an exact definition: a K sweep
    prescribes the driven coordinates, so every driven wheel must end at the
    travel it was given, and the grid the kernel reports must be the grid the
    document asked for.
    """
    from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
    from suspension_multibody.cases import (
        vehicle_kc_case_document,
        vehicle_kc_model_document,
    )
    from suspension_multibody.core.spatial import quaternion_to_matrix
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
    from suspension_multibody.simulation import SimulationRequest, run_request

    fixture = _load_vehicle_fixture()
    rigid = fixture._positioned_vehicle(fixture._vehicle())
    model = rigid.model_copy(
        update={
            "front_axle": _bushed(rigid.front_axle),
            "rear_axle": _bushed(rigid.rear_axle),
        }
    )
    base = prepare_vehicle_run(model, fixture._case(model))
    wheels = (0.0, 10.0)
    racks = (0.0,)
    sweep = vehicle_kc_case_document(
        name="vehicle-kc-gate",
        wheel_values_mm=wheels,
        rack_values_mm=racks,
        times_s=tuple(
            np.linspace(0.0, _VEHICLE_KC_WINDOW_S, _VEHICLE_KC_SAMPLES).tolist()
        ),
        settings=AxleSolverSettings(),
    )
    model_document_pair = vehicle_kc_model_document(model, base)
    produced = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_kc",
            model=model_document_pair,
            case=sweep,
            context={
                "model_document_pair": model_document_pair,
                "wheels": (),
                "vehicle_assembly": base.assembly,
            },
        )
    ).raw

    expected = [f"k-w{w:+.0f}-r{r:+.0f}" for w in wheels for r in racks]
    given = [str(entry["name"]) for entry in produced.cases]
    if given != expected:
        return False, f"the driven grid expanded to {given}, not {expected}"

    # prepare_vehicle_run keeps its assembly behind a field whose concrete
    # type the package does not export, so the point table is read reflectively.
    assembly = cast(Any, base.assembly)
    names = list(produced.document["manifest"]["bodies"])
    states = produced.block("body_state")
    last = {
        str(entry["name"]): int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        for entry in produced.cases
    }

    # A zero sweep is resolved to the separation the model was assembled with,
    # so the final state is the assembling pose.
    zero_row = states[last[expected[0]]]
    for body in base.native_model.bodies:
        found = zero_row[names.index(body.name)]
        drift = max(
            abs(float(found[axis]) - float(body.position_m[axis])) for axis in range(3)
        )
        if drift > 1e-9:
            return False, f"a zero sweep moved {body.name} by {drift:.3e} m"

    def driven(body: str, row) -> float:
        """Return the wheel-centre separation the kernel's driven row measures."""
        entry = row[names.index(body)]
        origin = row[names.index("chassis")]
        local = (
            np.asarray(assembly.points[(body, "wheel_center")], dtype=float)
            / 1e3
        )
        world = np.asarray(entry[:3], dtype=float) + quaternion_to_matrix(
            np.asarray(entry[3:7], dtype=float)
        ) @ local
        axis = quaternion_to_matrix(
            np.asarray(origin[3:7], dtype=float)
        ) @ np.array([0.0, 0.0, 1.0])
        return float(np.dot(world - np.asarray(origin[:3], dtype=float), axis))

    bump_row = states[last[expected[1]]]
    for body in _VEHICLE_KC_WHEELS:
        advance = driven(body, bump_row) - driven(body, zero_row)
        if abs(advance - 0.010) > 1e-6:
            return False, f"{body} advanced {advance * 1e3:.4f} mm, not 10 mm"

    return True, "grid matches an independent expansion; 10 mm reaches every wheel drive"

#: Entries the case contract declares as families but which are not solves.
#:
#: `comparison` is the one that matters: the architecture defines it as a
#: per-target gate (strict K/C against Adams, axle evidence equivalence, vehicle
#: correlation), and the reason it is a gate rather than a family is that its
#: whole job is to hold two artefacts against each other.  A kernel that ran it
#: would have to read a reference, which is exactly what the boundary forbids --
#: the kernel does not know Adams exists.  So this is reported as not applicable
#: rather than as work that is pending, and this list is the one place that says
#: which is which.
NOT_A_SOLVER_FAMILY: dict[str, str] = {
    "comparison": (
        "a per-target gate, not a solve: the kernel never reads a reference"
    ),
}

#: Families with no case-layer expansion yet.  The value is why, so the report
#: says what is missing rather than only that something is.
UNIMPLEMENTED: dict[str, str] = {}

CHECKS: dict[str, Callable[[], tuple[bool, str]]] = {
    "kc_quasi_static": check_kc_quasi_static,
    "axle_dynamic": check_axle_dynamic,
    "vehicle_dynamic": check_vehicle_dynamic,
    "ride_four_post": check_ride_four_post,
    "handling": check_handling,
    "ride_random_road": check_ride_random_road,
    "vehicle_kc": check_vehicle_kc,
}


def main(argv: list[str] | None = None) -> int:
    """Judge every declared family and report the verdict per family."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--family",
        action="append",
        choices=FAMILIES,
        help="judge only these families; defaults to all of them",
    )
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="an unimplemented family is a warning rather than a failure",
    )
    args = parser.parse_args(argv)

    selected = tuple(args.family) if args.family else FAMILIES
    failures: list[str] = []
    for family in selected:
        if family in NOT_A_SOLVER_FAMILY:
            print(f"  {family:18s} N/A       {NOT_A_SOLVER_FAMILY[family]}")
            continue
        check = CHECKS.get(family)
        if check is None:
            reason = UNIMPLEMENTED.get(family, "no implementation")
            print(f"  {family:18s} MISSING   {reason}")
            if not args.allow_partial:
                failures.append(family)
            continue
        passed, detail = check()
        print(f"  {family:18s} {'PASS' if passed else 'FAIL':8s}  {detail}")
        if not passed:
            failures.append(family)

    if failures:
        print(f"FAIL: {len(failures)} of {len(selected)} families not accepted: {failures}")
        return 1
    print(f"OK: {len(selected)} families accepted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
