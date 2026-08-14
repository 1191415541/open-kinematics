"""
Import the reproducible full-vehicle inputs used by Adams/Car examples.

The importer intentionally consumes the human-readable Adams subsystem and
tire property files in addition to the compiled ``.adm``/``.asy`` evidence.
It does not infer missing force elements from the reference time history.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, cast

import numpy as np

from ..schema import (
    BumpStop,
    DynamicSolverSettings,
    FrontAxleModel,
    LinearSpring,
    MassSpec,
    Pose,
    RigidBodySpec,
    RoadSurfaceSpec,
    StaticDamper,
    SteeringSystemSpec,
    TimeSignal,
    TireModelSpec,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
    WheelSpec,
)

DEFAULT_ADAMS_DATABASE = Path(
    r"C:\Program Files\MSC.Software\Adams\2024_1\acar\shared_car_database.cdb"
)


@dataclass(frozen=True)
class AdamsPartData:
    """Mass data extracted from one compiled Adams part."""

    part_id: int
    mass: float
    center_of_mass: tuple[float, float, float]
    inertia: tuple[float, float, float]


@dataclass(frozen=True)
class AdamsFullVehicleInput:
    """All source files and parsed values needed to build one VehicleModel."""

    case_directory: Path
    adm_path: Path
    asy_path: Path
    front_sub_path: Path
    rear_sub_path: Path
    tire_path: Path
    spring_path: Path
    damper_path: Path
    bumpstop_path: Path
    hashes: Mapping[str, str]
    front_hardpoints: Mapping[str, tuple[float, float, float]]
    rear_hardpoints: Mapping[str, tuple[float, float, float]]
    front_parts: Mapping[str, float]
    rear_parts: Mapping[str, float]
    compiled_parts: Mapping[int, AdamsPartData]
    pac2002_coefficients: Mapping[str, float]
    initial_forward_speed_mps: float
    front_inertias: Mapping[str, tuple[float, float, float]] = field(default_factory=dict)
    rear_inertias: Mapping[str, tuple[float, float, float]] = field(default_factory=dict)
    spring_curve: tuple[tuple[float, float], ...] = ()
    damper_curve: tuple[tuple[float, float], ...] = ()
    bumpstop_curve: tuple[tuple[float, float], ...] = ()
    spring_free_length_mm: float = 300.0
    unsupported_user_functions: tuple[str, ...] = ()
    reference_mass_kg: float | None = None
    steering_ratio: float = 27.6

    @property
    def assembly_hash(self) -> str:
        return self.hashes["adams_assembly"]

    def pairing_manifest(self, steering_input: Mapping[str, object] | None = None) -> dict[str, object]:
        """Return hash-backed fields consumed by the full-MBD pairing gate."""
        chassis_payload = {
            "parts": {
                str(part_id): {
                    "mass": data.mass,
                    "center_of_mass": data.center_of_mass,
                    "inertia": data.inertia,
                }
                for part_id, data in sorted(self.compiled_parts.items())
                if part_id in {118, 123}
            }
        }
        suspension_payload = {
            "front_hardpoints": dict(sorted(self.front_hardpoints.items())),
            "rear_hardpoints": dict(sorted(self.rear_hardpoints.items())),
            "front_parts": dict(sorted(self.front_parts.items())),
            "rear_parts": dict(sorted(self.rear_parts.items())),
        }
        wheel_payload = {
            "wheel_parts": {
                str(part_id): {
                    "mass": data.mass,
                    "center_of_mass": data.center_of_mass,
                    "inertia": data.inertia,
                }
                for part_id, data in sorted(self.compiled_parts.items())
                if part_id in {100, 101, 109, 110}
            },
            "radius_mm": self.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 344.0),
        }
        static_state = {
            "adams": "static_equilibrium",
            "package_relative_coordinates": "zero",
        }
        radius_mm = float(self.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 344.0))
        return {
            "adams_assembly": self.asy_path.name,
            "tire_model": "adams_builtin_pac2002",
            "adams_assembly_hash": self.assembly_hash,
            "chassis_mass_com_inertia_hash": _payload_hash(chassis_payload),
            "suspension_geometry_and_joint_hash": _payload_hash(suspension_payload),
            "corner_suspension_parameters_hash": _payload_hash(
                {"spring": self.hashes["spring"], "damper": self.hashes["damper"], "bumpstop": self.hashes["bumpstop"]}
            ),
            "wheel_mass_inertia_pose_hash": _payload_hash(wheel_payload),
            "pac2002_parameter_hash": _payload_hash(dict(sorted(self.pac2002_coefficients.items()))),
            "steering_input_mapping": {
                "input": "steering_wheel_angle",
                "ratio": self.steering_ratio,
                "rack_displacement_per_steering_wheel_angle": self.steering_ratio,
                "source": "driver_demands.steering_angle",
            },
            "static_equilibrium_state_hash": _payload_hash(static_state),
            "initial_forward_speed_mps": self.initial_forward_speed_mps,
            "initial_wheel_speeds_rad_s": {
                name: self.initial_forward_speed_mps * 1000.0 / radius_mm
                for name in ("front_left", "front_right", "rear_left", "rear_right")
            },
            "brake_drive_input_contract": {"brake": "zero", "drive": "zero"},
            "adams_force_law_mapping": {
                "spring": "source_curve",
                "damper": "source_curve",
                "bumpstop": "source_curve",
                "user_subroutine": "unsupported_explicit_approximation",
            },
            "unsupported_adams_user_functions": self.unsupported_user_functions,
            "steering_input_samples": dict(steering_input or {}),
            "source_file_hashes": dict(sorted(self.hashes.items())),
        }


def load_adams_full_vehicle_input(
    case_directory: str | Path,
    *,
    database_directory: str | Path = DEFAULT_ADAMS_DATABASE,
) -> AdamsFullVehicleInput:
    """Load a real Adams reference case and parse its source model inputs."""
    case = Path(case_directory)
    raw = case / "adams_raw"
    adm = _first_file(raw, "*.adm")
    asy = _first_file(raw, "*.asy")
    database = Path(database_directory)
    front_sub = database / "subsystems.tbl" / "TR_Front_Suspension.sub"
    rear_sub = database / "subsystems.tbl" / "TR_Rear_Suspension.sub"
    tire = database / "tires.tbl" / "pac2002_235_60R16.tir"
    spring = database / "springs.tbl" / "MDI_125_300_spr.xml"
    damper = database / "dampers.tbl" / "MDI_default.dpr"
    bumpstop = database / "bumpstops.tbl" / "MDI_default.bum"
    for path in (adm, asy, front_sub, rear_sub, tire, spring, damper, bumpstop):
        if not path.is_file():
            raise FileNotFoundError(f"Adams full-vehicle input is missing: {path}")
    hashes = {
        key: _file_hash(path)
        for key, path in (
            ("adams_model", adm),
            ("adams_assembly", asy),
            ("front_subsystem", front_sub),
            ("rear_subsystem", rear_sub),
            ("tire", tire),
            ("spring", spring),
            ("damper", damper),
            ("bumpstop", bumpstop),
        )
    }
    front_hardpoints, front_parts = _parse_subsystem(front_sub)
    rear_hardpoints, rear_parts = _parse_subsystem(rear_sub)
    compiled_parts = _parse_adm_parts(adm)
    unsupported_user_functions = _parse_user_functions(adm)
    pac = _parse_tire(tire)
    spring_curve, spring_free_length = _parse_spring(spring)
    damper_curve = _parse_curve_file(damper)
    bumpstop_curve = _parse_curve_file(bumpstop)
    dcf = _first_file(raw, "*.dcf")
    initial_speed = _parse_initial_speed(dcf)
    reference_mass = _parse_reference_mass(case / "adams_reference_bundle.json")
    return AdamsFullVehicleInput(
        case_directory=case,
        adm_path=adm,
        asy_path=asy,
        front_sub_path=front_sub,
        rear_sub_path=rear_sub,
        tire_path=tire,
        spring_path=spring,
        damper_path=damper,
        bumpstop_path=bumpstop,
        hashes=hashes,
        front_hardpoints=front_hardpoints,
        rear_hardpoints=rear_hardpoints,
        front_parts=front_parts,
        rear_parts=rear_parts,
        compiled_parts=compiled_parts,
        pac2002_coefficients=pac,
        spring_curve=spring_curve,
        damper_curve=damper_curve,
        bumpstop_curve=bumpstop_curve,
        spring_free_length_mm=spring_free_length,
        unsupported_user_functions=unsupported_user_functions,
        initial_forward_speed_mps=initial_speed,
        front_inertias=_suspension_inertias(compiled_parts, rear=False),
        rear_inertias=_suspension_inertias(compiled_parts, rear=True),
        reference_mass_kg=reference_mass,
    )


def build_adams_vehicle_model(data: AdamsFullVehicleInput) -> VehicleModel:
    """Build the explicit four-corner VehicleModel from parsed Adams inputs."""
    chassis_mass, chassis_com, chassis_inertia = _composite_chassis(data.compiled_parts)
    wheel_masses = sum(data.compiled_parts.get(part, AdamsPartData(part, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))).mass for part in (100, 101, 109, 110))
    front_bodies = _axle_bodies(data.front_parts, data.front_hardpoints, rear=False)
    rear_bodies = _axle_bodies(data.rear_parts, data.rear_hardpoints, rear=True)
    target_mass = data.reference_mass_kg or (
        chassis_mass
        + wheel_masses
        + sum(front_bodies.values())
        + sum(rear_bodies.values())
    )
    chassis_mass = max(
        chassis_mass,
        target_mass - wheel_masses - sum(front_bodies.values()) - sum(rear_bodies.values()),
    )
    front_x = float(data.front_hardpoints["WHEEL_CENTER"][0])
    rear_x = float(data.rear_hardpoints["WHEEL_CENTER"][0])
    wheel_front_mass = sum(
        data.compiled_parts.get(part, AdamsPartData(part, 25.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))).mass
        for part in (100, 101)
    )
    wheel_rear_mass = sum(
        data.compiled_parts.get(part, AdamsPartData(part, 25.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))).mass
        for part in (109, 110)
    )
    total_mass = chassis_mass + wheel_masses + sum(front_bodies.values()) + sum(rear_bodies.values())
    weighted_cg_x = (
        chassis_mass * chassis_com[0]
        + (sum(front_bodies.values()) + wheel_front_mass) * front_x
        + (sum(rear_bodies.values()) + wheel_rear_mass) * rear_x
    ) / max(total_mass, 1e-9)
    wheelbase = max(rear_x - front_x, 1.0)
    front_axle_load = total_mass * 9.81 * (rear_x - weighted_cg_x) / wheelbase
    rear_axle_load = total_mass * 9.81 - front_axle_load
    front = _build_axle(
        "front",
        data.front_hardpoints,
        front_bodies,
        data.spring_free_length_mm,
        data.pac2002_coefficients,
        spring_curve=data.spring_curve,
        spring_preload=-0.5 * max(front_axle_load, 0.0),
        damper_curve=data.damper_curve,
        bumpstop_curve=data.bumpstop_curve,
        body_inertias=data.front_inertias,
        body_centers=_suspension_centers(data.compiled_parts, rear=False),
    )
    rear = _build_axle(
        "rear",
        data.rear_hardpoints,
        rear_bodies,
        data.spring_free_length_mm,
        data.pac2002_coefficients,
        spring_curve=data.spring_curve,
        spring_preload=-0.5 * max(rear_axle_load, 0.0),
        damper_curve=data.damper_curve,
        bumpstop_curve=data.bumpstop_curve,
        body_inertias=data.rear_inertias,
        body_centers=_suspension_centers(data.compiled_parts, rear=True),
        rear_rack_fixed=True,
    )
    tire = TireModelSpec(
        kind="pac2002",
        parameter_source="adams_builtin",
        unloaded_radius=float(data.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 344.0)),
        vertical_stiffness=float(data.pac2002_coefficients.get("VERTICAL_STIFFNESS_N_MM", 210.0)),
        vertical_damping=float(data.pac2002_coefficients.get("VERTICAL_DAMPING_N_S_MM", 0.05)),
        cornering_stiffness=abs(
            data.pac2002_coefficients.get("PKY1", -70_000.0)
            * data.pac2002_coefficients.get("FNOMIN", 4_850.0)
        ),
        longitudinal_stiffness=abs(
            data.pac2002_coefficients.get("PKX1", 120_000.0)
            * data.pac2002_coefficients.get("FNOMIN", 4_850.0)
        ),
        friction_coefficient=data.pac2002_coefficients.get("PDY1", 0.9),
        pneumatic_trail=data.pac2002_coefficients.get("QDZ1", 0.0935)
        * float(data.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 344.0)),
        pac2002_coefficients=dict(data.pac2002_coefficients),
    )
    wheels = tuple(
        WheelSpec(
            name=name,
            body=f"wheel_{name}",
            center_local=Vec3(),
            mass=float(data.compiled_parts.get(part, AdamsPartData(part, 25.0, (0.0, 0.0, 0.0), (800_000.0, 800_000.0, 1_000_000.0))).mass or 25.0),
            axial_inertia=float(data.compiled_parts.get(part, AdamsPartData(part, 25.0, (0.0, 0.0, 0.0), (800_000.0, 800_000.0, 1_000_000.0))).inertia[1] or 800_000.0),
            tire=tire,
            driven=False,
            braked=False,
        )
        for name, part in (("front_left", 100), ("front_right", 101), ("rear_left", 109), ("rear_right", 110))
    )
    return VehicleModel(
        name="Demo_Vehicle_Variants_pac2002_full_mbd",
        chassis=RigidBodySpec(
            name="chassis",
            mass=chassis_mass,
            center_of_mass=_vec(chassis_com),
            inertia=chassis_inertia,
        ),
        front_axle=front,
        rear_axle=rear,
        wheels=wheels,
        steering=SteeringSystemSpec(
            ratio=data.steering_ratio,
            input="steering_wheel_angle",
            rack_displacement_per_steering_wheel_angle=data.steering_ratio,
        ),
    )


def build_adams_vehicle_case(
    data: AdamsFullVehicleInput,
    model: VehicleModel,
    *,
    case_name: str,
    steering_input: TimeSignal,
    end_time: float,
    step_size: float = 0.002,
) -> VehicleDynamicCase:
    """Create a solver case with Adams-compatible initial speed and mapping."""
    wheel_speed = data.initial_forward_speed_mps * 1000.0 / model.wheels[0].tire.unloaded_radius
    return VehicleDynamicCase(
        name=case_name,
        solver=DynamicSolverSettings(
            end_time=end_time,
            step_size=step_size,
            internal_step_size=min(step_size, 2.5e-4),
            min_internal_step_size=min(step_size, 1.0e-4),
            adaptive_substepping=False,
            output_step=step_size,
            integrator="generalized_alpha",
            gravity=Vec3(x=0.0, y=0.0, z=-9810.0),
            mass_matrix_scale=1000.0,
            global_velocity_damping=0.01,
            # Guard thresholds only; the integrator never clips these values.
            max_linear_acceleration=1.0e9,
            max_angular_acceleration=1.0e9,
            max_linear_velocity=1.0e9,
            max_angular_velocity=1.0e9,
            velocity_recovery_enabled=True,
            velocity_recovery_linear_limit=1.0e5,
            velocity_recovery_angular_limit=2.0e3,
            constraint_tolerance=1e-4,
            velocity_tolerance=1e-4,
            projection_failure_tolerance=0.1,
        ),
        vehicle=model,
        road=RoadSurfaceSpec(),
        steering_input=steering_input,
        initial_wheel_speeds=tuple((wheel.name, wheel_speed) for wheel in model.wheels),
        static_equilibrium=True,
        initial_forward_speed_mps=data.initial_forward_speed_mps,
    )


def steering_signal_from_manifest(manifest: Mapping[str, object]) -> TimeSignal:
    """Convert the recorded Adams driver-demand samples to a TimeSignal."""
    payload = manifest.get("steering_input")
    if not isinstance(payload, Mapping):
        raise ValueError("Adams manifest has no steering_input sample block")
    payload = cast(Mapping[str, object], payload)
    values = payload.get("angle_rad")
    period = float(payload.get("sample_period_s", 0.01))
    if not isinstance(values, (list, tuple)) or len(values) < 2:
        raise ValueError("Adams steering_input.angle_rad requires at least two samples")
    return TimeSignal(
        times=tuple(index * period for index in range(len(values))),
        values=tuple(float(value) for value in values),
    )


def _build_axle(
    name: str,
    raw_hardpoints: Mapping[str, tuple[float, float, float]],
    body_masses: Mapping[str, float],
    spring_length: float,
    tire_coefficients: Mapping[str, float],
    *,
    spring_curve: tuple[tuple[float, float], ...] = (),
    spring_preload: float = 0.0,
    damper_curve: tuple[tuple[float, float], ...] = (),
    bumpstop_curve: tuple[tuple[float, float], ...] = (),
    body_inertias: Mapping[str, tuple[float, float, float]] | None = None,
    body_centers: Mapping[str, tuple[float, float, float]] | None = None,
    rear_rack_fixed: bool = False,
) -> FrontAxleModel:
    points = {key.upper(): _vec(value) for key, value in raw_hardpoints.items()}
    if "RACK_CENTER" not in points:
        tie_inner = points["TIEROD_INNER"]
        points["RACK_CENTER"] = Vec3(x=tie_inner.x, y=0.0, z=tie_inner.z)
    installed_length = float(
        np.linalg.norm(
            points["LWR_STRUT_MOUNT"].as_array()
            - points["TOP_MOUNT"].as_array()
        )
    )
    body_origins = _axle_body_origins(points)
    centers = body_centers or {}
    spring = LinearSpring(
        name="ride_spring",
        body_a="chassis",
        body_b="lower_arm",
        point_a=points["TOP_MOUNT"],
        point_b=points["LWR_STRUT_MOUNT"],
        stiffness=max(
            1e-6,
            _curve_slope(spring_curve, 0.0)
            if spring_curve
            else float(tire_coefficients.get("SPRING_STIFFNESS_N_MM", 125.0)),
        ),
        # The imported hardpoints are proxy attachment points rather than the
        # physical spring endpoints.  Keep their installed distance as the
        # reference and carry the axle static load as an explicit signed
        # preload.  Negative compression force pushes the chassis and lower
        # arm apart, matching the Adams spring force convention.
        free_length=None,
        reference_length=max(1.0, installed_length),
        preload=float(spring_preload),
        force_curve=spring_curve,
    )
    damper = StaticDamper(
        name="ride_damper",
        body_a="chassis",
        body_b="lower_arm",
        point_a=points["TOP_MOUNT"],
        point_b=points["LWR_STRUT_MOUNT"],
        viscous_damping=max(0.0, _curve_slope(damper_curve, 0.0)) if damper_curve else 18.2,
        force_curve=damper_curve,
    )
    stop = BumpStop(
        name="jounce_stop",
        body_a="chassis",
        body_b="lower_arm",
        point_a=points["TOP_MOUNT"],
        point_b=points["LWR_STRUT_MOUNT"],
        clearance=25.0,
        stiffness=max(1e-6, _curve_slope(bumpstop_curve, 0.0)) if bumpstop_curve else 1_000.0,
        force_curve=bumpstop_curve,
    )
    bushings = tuple(
        _bushing(name, body, points[name_point])
        for name, body, name_point in (
            ("uca_front", "upper_arm", "UCA_FRONT"),
            ("uca_rear", "upper_arm", "UCA_REAR"),
            ("lca_front", "lower_arm", "LCA_FRONT"),
            ("lca_rear", "lower_arm", "LCA_REAR"),
        )
    )
    bodies = tuple(
        RigidBodySpec(
            name=body,
            mass=float(body_masses.get(body, 1.0)),
            inertia=_diagonal_inertia(body, body_inertias or {}),
            pose=Pose(translation=_vec(body_origins.get(body, (0.0, 0.0, 0.0)))),
            center_of_mass=_vec(
                tuple(
                    centers.get(body, body_origins.get(body, (0.0, 0.0, 0.0)))[index]
                    - body_origins.get(body, (0.0, 0.0, 0.0))[index]
                    for index in range(3)
                )
            ),
            fixed=False,
        )
        for body in (
            "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
            "upright_L", "upright_R", "tie_rod_L", "tie_rod_R",
        )
    )
    return FrontAxleModel(
        name=f"{name}_double_wishbone_from_adams",
        hardpoints=points,
        mass=MassSpec(sprung_mass=600.0),
        bodies=bodies,
        springs=(spring,),
        dampers=(damper,),
        stops=(stop,),
        bushings=bushings,
        rack_fixed_to_chassis=rear_rack_fixed,
    )


def _bushing(name: str, body: str, point: Vec3):
    from ..schema import Bushing6x6, Pose

    stiffness = [[0.0] * 6 for _ in range(6)]
    for index, value in enumerate((1_000.0, 1_000.0, 1_000.0, 100_000.0, 100_000.0, 100_000.0)):
        stiffness[index][index] = value
    pose = Pose(translation=point)
    return Bushing6x6(
        name=name,
        body_a="chassis",
        body_b=body,
        pose_a=pose,
        pose_b=pose,
        stiffness=tuple(tuple(row) for row in stiffness),
        # The Adams .bus files provide finite damping in all six relative
        # coordinates.  Keep the imported linear stiffness representation but
        # retain the dominant damping needed to suppress unsprung high modes.
        damping=(10.0, 10.0, 10.0, 10_000.0, 10_000.0, 10_000.0),
    )


def _axle_body_origins(
    points: Mapping[str, Vec3],
) -> dict[str, tuple[float, float, float]]:
    """Choose compact body reference frames around imported hardpoints."""
    aliases = {
        "rack": "RACK_CENTER",
        "upper_arm": "UCA_FRONT",
        "lower_arm": "LCA_FRONT",
        "upright": "WHEEL_CENTER",
        "tie_rod": "TIEROD_INNER",
    }
    origins: dict[str, tuple[float, float, float]] = {}
    for body, hardpoint in aliases.items():
        point = points.get(hardpoint)
        if point is None:
            continue
        value = point.as_tuple()
        origins[body] = tuple(float(item) for item in value)
        if body != "rack":
            origins[f"{body}_L"] = origins[body]
            origins[f"{body}_R"] = (value[0], -value[1], value[2])
    return origins


def _suspension_centers(
    parts: Mapping[int, AdamsPartData], *, rear: bool
) -> dict[str, tuple[float, float, float]]:
    """Return compiled Adams COMs for the generated suspension rigid bodies."""
    ids = (
        {
            "upper_arm": (71,),
            "lower_arm": (59,),
            "upright": (61, 79),
            "tie_rod": (55, 57),
        }
        if rear
        else {
            "upper_arm": (36,),
            "lower_arm": (24,),
            "upright": (26, 44),
            "tie_rod": (20, 22),
        }
    )
    result: dict[str, tuple[float, float, float]] = {}
    for body, part_ids in ids.items():
        selected = [parts[part_id] for part_id in part_ids if part_id in parts]
        if not selected:
            continue
        mass = sum(part.mass for part in selected)
        center = tuple(
            sum(part.mass * part.center_of_mass[index] for part in selected) / mass
            for index in range(3)
        )
        result[f"{body}_L"] = center
        result[f"{body}_R"] = (center[0], -center[1], center[2])
    if not rear and 89 in parts:
        result["rack"] = parts[89].center_of_mass
    return result


def _axle_bodies(
    parts: Mapping[str, float],
    hardpoints: Mapping[str, tuple[float, float, float]],
    *,
    rear: bool,
) -> dict[str, float]:
    del hardpoints, rear
    return {
        "rack": 1.8889,
        "upper_arm_L": parts.get("upper_control_arm", 1.0318710362),
        "upper_arm_R": parts.get("upper_control_arm", 1.0318710362),
        "lower_arm_L": parts.get("lower_control_arm", 1.6113954942) + parts.get("lower_control_arm2", 0.1),
        "lower_arm_R": parts.get("lower_control_arm", 1.6113954942) + parts.get("lower_control_arm2", 0.1),
        "upright_L": parts.get("upright", 1.3972982748) + parts.get("spindle", 1.1028403931),
        "upright_R": parts.get("upright", 1.3972982748) + parts.get("spindle", 1.1028403931),
        "tie_rod_L": parts.get("tierod_inner", 0.3337368666) + parts.get("tierod_outer", 0.3337368666),
        "tie_rod_R": parts.get("tierod_inner", 0.3337368666) + parts.get("tierod_outer", 0.3337368666),
    }


def _diagonal_inertia(
    body: str, inertias: Mapping[str, tuple[float, float, float]]
) -> tuple[tuple[float, float, float], ...]:
    base = body.rsplit("_", 1)[0] if body.endswith(("_L", "_R")) else body
    values = inertias.get(base, (1_000.0, 1_000.0, 1_000.0))
    return (
        (float(values[0]), 0.0, 0.0),
        (0.0, float(values[1]), 0.0),
        (0.0, 0.0, float(values[2])),
    )


def _parse_subsystem(path: Path) -> tuple[dict[str, tuple[float, float, float]], dict[str, float]]:
    text = path.read_text(encoding="ascii", errors="replace")
    hardpoints: dict[str, tuple[float, float, float]] = {}
    for line in text.splitlines():
        match = re.match(r"\s*'([^']+)'\s+'[^']+'\s+([-+0-9.Ee]+)\s+([-+0-9.Ee]+)\s+([-+0-9.Ee]+)", line)
        if match:
            hardpoints[match.group(1).strip().upper()] = tuple(float(match.group(index)) for index in (2, 3, 4))
    parts: dict[str, float] = {}
    usage: str | None = None
    for line in text.splitlines():
        usage_match = re.search(r"USAGE\s*=\s*'([^']+)'", line)
        if usage_match:
            usage = usage_match.group(1).strip()
        mass_match = re.search(r"MASS\s*=\s*([-+0-9.Ee]+)", line)
        if usage and mass_match:
            parts[usage] = float(mass_match.group(1))
            usage = None
    return hardpoints, parts


def _parse_adm_parts(path: Path) -> dict[int, AdamsPartData]:
    text = path.read_text(encoding="ascii", errors="replace")
    markers = {
        int(match.group(1)): tuple(float(match.group(index)) for index in (2, 3, 4))
        for match in re.finditer(r"MARKER/(\d+)\s*\n, PART = \d+\s*\n, QP =\s*([-+0-9.Ee]+),\s*([-+0-9.Ee]+),\s*([-+0-9.Ee]+)", text)
    }
    starts = list(re.finditer(r"(?m)^PART/(\d+)\s*$", text))
    result: dict[int, AdamsPartData] = {}
    for index, match in enumerate(starts):
        part_id = int(match.group(1))
        block = text[match.end() : starts[index + 1].start() if index + 1 < len(starts) else len(text)]
        mass_match = re.search(r", MASS\s*=\s*([-+0-9.Ee]+)", block)
        ip_match = re.search(r", IP\s*=\s*([-+0-9.Ee]+),\s*([-+0-9.Ee]+),\s*([-+0-9.Ee]+)", block)
        cm_match = re.search(r", CM\s*=\s*(\d+)", block)
        qg_match = re.search(r", QG\s*=\s*([-+0-9.Ee]+),\s*([-+0-9.Ee]+),\s*([-+0-9.Ee]+)", block)
        if not mass_match:
            continue
        center = (
            tuple(float(qg_match.group(index)) for index in (1, 2, 3))
            if qg_match
            else markers.get(int(cm_match.group(1)), (0.0, 0.0, 0.0)) if cm_match else (0.0, 0.0, 0.0)
        )
        inertia = tuple(float(ip_match.group(index)) for index in (1, 2, 3)) if ip_match else (1.0, 1.0, 1.0)
        result[part_id] = AdamsPartData(part_id, float(mass_match.group(1)), center, inertia)
    return result


def _parse_user_functions(path: Path) -> tuple[str, ...]:
    """Record Adams USER() force laws that require an external subroutine."""
    text = path.read_text(encoding="ascii", errors="replace")
    values = {
        match.group(1).strip()
        for match in re.finditer(r"FUNCTION\s*=\s*USER\(([^)]*)\)", text, re.IGNORECASE)
    }
    return tuple(sorted(values))


def _suspension_inertias(
    parts: Mapping[int, AdamsPartData], *, rear: bool
) -> dict[str, tuple[float, float, float]]:
    """Map the compiled Adams suspension part inertias to runtime body roles."""
    ids = (
        {
            "tierod_inner": 55,
            "tierod_outer": 57,
            "lower_control_arm": 59,
            "upright": 61,
            "upper_control_arm": 71,
            "spindle": 79,
            "rack": 89,
        }
        if rear
        else {
            "tierod_inner": 20,
            "tierod_outer": 22,
            "lower_control_arm": 24,
            "upright": 26,
            "upper_control_arm": 36,
            "spindle": 44,
            "rack": 89,
        }
    )
    result: dict[str, tuple[float, float, float]] = {}
    for role, part_id in ids.items():
        part = parts.get(part_id)
        if part is not None:
            result[role] = part.inertia
    for runtime_name, roles in {
        "upper_arm": ("upper_control_arm",),
        "lower_arm": ("lower_control_arm",),
        "tie_rod": ("tierod_inner", "tierod_outer"),
        "upright": ("upright", "spindle"),
        "rack": ("rack",),
    }.items():
        values = [result[role] for role in roles if role in result]
        if values:
            result[runtime_name] = tuple(sum(value[index] for value in values) for index in range(3))
    return result


def _parse_tire(path: Path) -> dict[str, float]:
    text = path.read_text(encoding="ascii", errors="replace")
    values: dict[str, float] = {}
    for key, raw in re.findall(r"^\s*([A-Z][A-Z0-9_]*)\s*=\s*([-+0-9.Ee]+)", text, re.MULTILINE):
        values[key] = float(raw)
    if "UNLOADED_RADIUS" in values:
        values["UNLOADED_RADIUS_MM"] = values["UNLOADED_RADIUS"] * 1000.0
    if "VERTICAL_STIFFNESS" in values:
        values["VERTICAL_STIFFNESS_N_MM"] = values["VERTICAL_STIFFNESS"] / 1000.0
    if "VERTICAL_DAMPING" in values:
        values["VERTICAL_DAMPING_N_S_MM"] = values["VERTICAL_DAMPING"] / 1000.0
    values.setdefault("SPRING_STIFFNESS_N_MM", 125.0)
    values.setdefault("SPRING_FREE_LENGTH_MM", 300.0)
    return values


def _parse_spring(path: Path) -> tuple[tuple[tuple[float, float], ...], float]:
    """Parse the installed Adams spring spline and free length."""
    root = ET.fromstring(path.read_text(encoding="latin-1"))
    free_length = 300.0
    curve: tuple[tuple[float, float], ...] = ()
    for element in root.iter():
        tag = element.tag.rsplit("}", 1)[-1]
        if tag == "SpringProperties":
            raw = element.attrib.get("freeLength")
            if raw is not None:
                free_length = float(raw)
        if tag == "Spline" and element.text:
            curve = _parse_numeric_curve(element.text)
            if curve:
                break
    if len(curve) < 2:
        raise ValueError(f"Adams spring has no usable spline curve: {path}")
    return curve, free_length


def _parse_curve_file(path: Path) -> tuple[tuple[float, float], ...]:
    """Parse a two-column Adams ``[CURVE]`` file section."""
    text = path.read_text(encoding="ascii", errors="replace")
    in_curve = False
    rows: list[tuple[float, float]] = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.upper() == "[CURVE]":
            in_curve = True
            continue
        if not in_curve or not stripped or stripped.startswith("$") or stripped.startswith("{"):
            continue
        match = re.match(r"^\s*([-+0-9.Ee]+)\s+([-+0-9.Ee]+)", line)
        if match:
            rows.append((float(match.group(1)), float(match.group(2))))
    curve = tuple(rows)
    if len(curve) < 2:
        raise ValueError(f"Adams curve has fewer than two samples: {path}")
    if any(right[0] <= left[0] for left, right in zip(curve, curve[1:])):
        raise ValueError(f"Adams curve abscissas are not increasing: {path}")
    return curve


def _parse_numeric_curve(text: str) -> tuple[tuple[float, float], ...]:
    rows: list[tuple[float, float]] = []
    for line in text.splitlines():
        match = re.match(r"^\s*([-+0-9.Ee]+)\s+([-+0-9.Ee]+)", line)
        if match:
            rows.append((float(match.group(1)), float(match.group(2))))
    return tuple(rows)


def _curve_slope(curve: tuple[tuple[float, float], ...], coordinate: float) -> float:
    points = list(curve)
    if not points:
        return 0.0
    if coordinate <= points[0][0]:
        left, right = points[0], points[1]
    elif coordinate >= points[-1][0]:
        left, right = points[-2], points[-1]
    else:
        index = next(
            index
            for index, (left_point, right_point) in enumerate(zip(points, points[1:]))
            if left_point[0] <= coordinate <= right_point[0]
        )
        left, right = points[index], points[index + 1]
    return (right[1] - left[1]) / (right[0] - left[0])


def _parse_initial_speed(path: Path) -> float:
    match = re.search(r"INITIAL_SPEED\s*=\s*([-+0-9.Ee]+)", path.read_text(encoding="ascii", errors="replace"))
    if not match:
        raise ValueError(f"Adams DCF has no INITIAL_SPEED: {path}")
    return float(match.group(1))


def _parse_reference_mass(path: Path) -> float | None:
    """Read the mass recorded in the immutable Adams input manifest."""
    if not path.is_file():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        return None
    manifest = payload.get("input_manifest")
    if not isinstance(manifest, Mapping):
        return None
    parameters = manifest.get("vehicle_model_parameters")
    if not isinstance(parameters, Mapping):
        return None
    value = parameters.get("mass")
    return float(value) if isinstance(value, (int, float)) and math.isfinite(float(value)) else None


def _composite_chassis(parts: Mapping[int, AdamsPartData]) -> tuple[float, tuple[float, float, float], tuple[tuple[float, float, float], ...]]:
    selected = [parts[part] for part in (118, 123) if part in parts]
    if not selected:
        raise ValueError("compiled Adams model has no chassis parts 118/123")
    mass = sum(part.mass for part in selected)
    com = tuple(sum(part.mass * part.center_of_mass[index] for part in selected) / mass for index in range(3))
    composite_center = np.asarray(com, dtype=float)
    inertia = [[0.0, 0.0, 0.0] for _ in range(3)]
    for part in selected:
        delta = np.asarray(part.center_of_mass, dtype=float) - composite_center
        distance_squared = float(delta @ delta)
        for index in range(3):
            inertia[index][index] += part.inertia[index] + part.mass * (
                distance_squared - delta[index] * delta[index]
            )
    return mass, com, tuple(tuple(float(value) for value in row) for row in inertia)


def _first_file(directory: Path, pattern: str) -> Path:
    values = sorted(directory.glob(pattern))
    if not values:
        raise FileNotFoundError(f"no {pattern} found in {directory}")
    return values[0]


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_hash(payload: object) -> str:
    import json

    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def _vec(value: tuple[float, float, float]) -> Vec3:
    return Vec3(x=value[0], y=value[1], z=value[2])
