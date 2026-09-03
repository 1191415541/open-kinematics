"""
Import the reproducible full-vehicle inputs used by Adams/Car examples.

The importer intentionally consumes the human-readable Adams subsystem and
tire property files in addition to the compiled ``.adm``/``.asy`` evidence.
It does not infer missing force elements from the reference time history.
"""

from __future__ import annotations

import ast
import hashlib
import json
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Mapping, cast

import numpy as np

from ..schema import (
    AerodynamicDragSpec,
    BumpStop,
    Bushing6x6,
    DrivelineSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    IdealJointSpec,
    InitialBodyState,
    JointCoordinateCouplerSpec,
    LinearSpring,
    MassSpec,
    Pose,
    Quaternion,
    RigidBodySpec,
    RoadSurfaceSpec,
    SixVector,
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
ALTERNATE_ADAMS_DATABASE = Path(
    r"G:\MSC.Software\Adams\2024_1\acar\shared_car_database.cdb"
)
Matrix3 = tuple[tuple[float, float, float], ...]
Vec3Tuple = tuple[float, float, float]
BushingCurve = tuple[tuple[float, float], ...]
BushingCurves6 = tuple[BushingCurve, ...]
_ADAMS_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][-+]?\d+)?"

_RUNTIME_PART_ROLE_FALLBACKS: dict[str, tuple[int, ...]] = {
    "front_lower_arm": (24,),
    "front_lower_arm2": (18,),
    "front_tie_rod_inner": (20,),
    "front_tie_rod_outer": (22,),
    "front_upright": (26,),
    "front_spindle": (44,),
    "front_upper_arm": (36,),
    "rear_lower_arm": (59,),
    "rear_lower_arm2": (53,),
    "rear_tie_rod_inner": (55,),
    "rear_tie_rod_outer": (57,),
    "rear_upright": (61,),
    "rear_spindle": (79,),
    "rear_upper_arm": (71,),
    "rack": (89,),
    "front_wheel_left": (100,),
    "front_wheel_right": (101,),
    "rear_wheel_left": (109,),
    "rear_wheel_right": (110,),
    "chassis": (118,),
    "powertrain": (123,),
}

_LENGTH_TO_MM: dict[str, float] = {
    "m": 1_000.0,
    "meter": 1_000.0,
    "meters": 1_000.0,
    "mm": 1.0,
    "millimeter": 1.0,
    "millimeters": 1.0,
    "cm": 10.0,
    "centimeter": 10.0,
    "centimeters": 10.0,
    "in": 25.4,
    "inch": 25.4,
    "inches": 25.4,
}
_FORCE_TO_N: dict[str, float] = {
    "n": 1.0,
    "newton": 1.0,
    "newtons": 1.0,
    "kn": 1_000.0,
    "kilonewton": 1_000.0,
    "kilonewtons": 1_000.0,
    "lbf": 4.4482216152605,
}
_MASS_TO_KG: dict[str, float] = {
    "kg": 1.0,
    "kilogram": 1.0,
    "kilograms": 1.0,
    "g": 1.0e-3,
    "gram": 1.0e-3,
    "grams": 1.0e-3,
    "lb": 0.45359237,
    "lbm": 0.45359237,
}
_TIME_TO_S: dict[str, float] = {
    "s": 1.0,
    "sec": 1.0,
    "second": 1.0,
    "seconds": 1.0,
    "ms": 1.0e-3,
    "millisecond": 1.0e-3,
    "milliseconds": 1.0e-3,
}
_ANGLE_TO_RAD: dict[str, float] = {
    "rad": 1.0,
    "radian": 1.0,
    "radians": 1.0,
    "deg": math.pi / 180.0,
    "degree": math.pi / 180.0,
    "degrees": math.pi / 180.0,
}


def _parse_text_units(text: str) -> dict[str, str]:
    """读取 Adams 文本文件中的单位段，并保留源文件声明."""
    units: dict[str, str] = {}
    in_units = False
    for line in text.splitlines():
        stripped = line.strip()
        upper = stripped.upper()
        if upper in {"[UNITS]", "UNITS/"}:
            in_units = True
            continue
        if not in_units:
            continue
        if stripped.startswith(("[", "$", "!", "(")):
            break
        for match in re.finditer(
            r"\b(LENGTH|FORCE|ANGLE|MASS|TIME)\s*=\s*['\"]?([^'\"$,\s]+)",
            line,
            re.IGNORECASE,
        ):
            units[match.group(1).lower()] = match.group(2).strip().lower()
    return units


def _parse_xml_units(text: str) -> dict[str, str]:
    """读取 Adams XML 属性文件中的 UnitSetting 声明."""
    root = ET.fromstring(text)
    units: dict[str, str] = {}
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "UnitSetting":
            continue
        name = element.attrib.get("name")
        current = element.attrib.get("current")
        if name and current:
            units[name.strip().lower()] = current.strip().lower()
    return units


def _unit_factor(
    unit: str | None,
    factors: Mapping[str, float],
    default: float,
) -> float:
    """将一个 Adams 单位转换为目标单位的倍率；未知单位必须显式失败."""
    if unit is None:
        return default
    normalized = unit.strip().lower()
    try:
        return factors[normalized]
    except KeyError as exc:
        raise ValueError(f"unsupported Adams unit: {unit!r}") from exc


def _source_units_complete(source_units: Mapping[str, Mapping[str, str]]) -> bool:
    """确认参与导入的每类源文件都声明了所需的基本单位."""
    required: dict[str, set[str]] = {
        "assembly": {"length", "force", "mass", "time"},
        "front_subsystem": {"length", "force", "mass", "time", "angle"},
        "rear_subsystem": {"length", "force", "mass", "time", "angle"},
        "tire": {"length", "force", "mass", "time", "angle"},
        "spring": {"length", "force", "mass", "time", "angle"},
        "damper": {"length", "force", "mass", "time", "angle"},
        "bumpstop": {"length", "force", "mass", "time", "angle"},
        "driver_control": {"length", "force", "mass", "time", "angle"},
    }
    factors = {
        "length": _LENGTH_TO_MM,
        "force": _FORCE_TO_N,
        "mass": _MASS_TO_KG,
        "time": _TIME_TO_S,
        "angle": _ANGLE_TO_RAD,
    }
    for source, required_names in required.items():
        units = source_units.get(source, {})
        if not required_names.issubset(units):
            return False
        try:
            for name in required_names:
                _unit_factor(units[name], factors[name], 1.0)
        except (KeyError, ValueError):
            return False
    return True


def _role_part_ids(
    part_roles: Mapping[str, tuple[int, ...]] | None,
    role: str,
) -> tuple[int, ...]:
    """Return semantic part IDs, falling back only for legacy fixtures."""
    if part_roles is not None:
        selected = part_roles.get(role)
        if selected:
            return tuple(selected)
    return _RUNTIME_PART_ROLE_FALLBACKS[role]


@dataclass(frozen=True)
class AdamsMarkerData:
    """Compiled Adams marker pose in its owning part frame."""

    marker_id: int
    part_id: int
    local_position: Vec3Tuple
    local_orientation: Matrix3 = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsPartFrameData:
    """Compiled Adams part reference frame, including zero-mass parts."""

    part_id: int
    orientation: Matrix3
    reference_origin: Vec3Tuple
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsJointData:
    """Compiled Adams ideal joint and its two marker references."""

    joint_id: int
    kind: str
    marker_i: int
    marker_j: int
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsCouplerData:
    """编译后 Adams 关节耦合器及其比例关系."""

    coupler_id: int
    joint_ids: tuple[int, ...]
    kind: str
    scales: tuple[float, ...]
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsFieldData:
    """Compiled Adams field force and its two marker references."""

    field_id: int
    marker_i: int
    marker_j: int
    formulation: str | None = None
    function: str | None = None
    routine: str | None = None
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsSforceData:
    """Compiled Adams scalar force and its two marker references."""

    force_id: int
    kind: str
    marker_i: int
    marker_j: int
    function: str | None = None
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsVariableData:
    """Compiled Adams scalar variable and its source expression."""

    variable_id: int
    function: str | None = None
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsUserFunctionData:
    """一个带实体来源的 Adams USER() 调用."""

    entity_type: str
    entity_id: int
    function: str
    routine: str | None = None
    adams_name: str | None = None


@dataclass(frozen=True)
class AdamsPartData:
    """Mass data extracted from one compiled Adams part."""

    part_id: int
    mass: float
    center_of_mass: tuple[float, float, float]
    inertia: tuple[float, float, float]
    inertia_products: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: Matrix3 = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    adams_name: str | None = None
    reference_origin: Vec3Tuple = (0.0, 0.0, 0.0)

    def inertia_about_com_global(self) -> np.ndarray:
        """Return the compiled Adams inertia in vehicle coordinates."""
        ixx, iyy, izz = self.inertia
        ixy, ixz, iyz = self.inertia_products
        local = np.array(
            ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz)),
            dtype=float,
        )
        rotation = np.asarray(self.orientation, dtype=float)
        return rotation @ local @ rotation.T


@dataclass(frozen=True)
class AdamsPartState:
    """一个 Adams 结果文件中的部件参考系状态."""

    translation: Vec3Tuple
    rotation: Matrix3
    linear_velocity: Vec3Tuple
    angular_velocity: Vec3Tuple


@dataclass(frozen=True)
class AdamsBushingProperty:
    """已转换为工程单位的 Adams 六轴衬套属性."""

    name: str
    path: Path
    units: Mapping[str, str]
    damping: tuple[float, float, float, float, float, float]
    force_curves: BushingCurves6


@dataclass(frozen=True)
class AdamsBushingAssembly:
    """一个悬架源文件中的衬套装配参数."""

    subsystem_path: Path
    usage: str
    symmetry: str
    property_key: str
    property_path: Path
    orientation_zp: tuple[float, float, float]
    orientation_xp: tuple[float, float, float]
    preload: tuple[float, float, float, float, float, float]
    force_scaling: tuple[float, float, float, float, float, float]
    damping_force_scaling: tuple[float, float, float, float, float, float]


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
    fiala_parameters: Mapping[str, float] = field(default_factory=dict)
    part_roles: Mapping[str, tuple[int, ...]] = field(default_factory=dict)
    front_inertias: Mapping[str, Matrix3] = field(default_factory=dict)
    rear_inertias: Mapping[str, Matrix3] = field(default_factory=dict)
    spring_curve: tuple[tuple[float, float], ...] = ()
    damper_curve: tuple[tuple[float, float], ...] = ()
    bumpstop_curve: tuple[tuple[float, float], ...] = ()
    spring_free_length_mm: float = 300.0
    unsupported_user_functions: tuple[str, ...] = ()
    reference_mass_kg: float | None = None
    steering_ratio: float = 27.6
    initial_velocity_sign: Literal[-1, 1] = 1
    source_units: Mapping[str, Mapping[str, str]] = field(default_factory=dict)
    bushing_properties: Mapping[str, AdamsBushingProperty] = field(default_factory=dict)
    front_bushing_assemblies: tuple[AdamsBushingAssembly, ...] = ()
    rear_bushing_assemblies: tuple[AdamsBushingAssembly, ...] = ()
    steering_bushing_assemblies: tuple[AdamsBushingAssembly, ...] = ()
    powertrain_bushing_assemblies: tuple[AdamsBushingAssembly, ...] = ()
    compiled_markers: Mapping[int, AdamsMarkerData] = field(default_factory=dict)
    compiled_part_frames: Mapping[int, AdamsPartFrameData] = field(default_factory=dict)
    source_joints: tuple[AdamsJointData, ...] = ()
    source_couplers: tuple[AdamsCouplerData, ...] = ()
    source_prescribed_joint_ids: tuple[int, ...] = ()
    source_fields: tuple[AdamsFieldData, ...] = ()
    source_forces: tuple[AdamsSforceData, ...] = ()
    source_variables: Mapping[int, AdamsVariableData] = field(default_factory=dict)
    source_user_functions: tuple[AdamsUserFunctionData, ...] = ()
    initial_part_states: Mapping[int, AdamsPartState] = field(default_factory=dict)

    @property
    def assembly_hash(self) -> str:
        return self.hashes["adams_assembly"]

    def pairing_manifest(
        self,
        steering_input: Mapping[str, object] | None = None,
        source_drive_brake_result_path: str | Path | None = None,
        *,
        tire_kind: Literal["pac2002", "native_brush", "fiala"] = "pac2002",
    ) -> dict[str, object]:
        """Return hash-backed fields consumed by the full-MBD pairing gate."""
        runtime_part_ids = _source_native_body_part_ids(
            self, include_drivetrain=True
        )
        omitted_part_ids = tuple(
            sorted(set(self.compiled_parts).difference(runtime_part_ids))
        )
        chassis_payload_ids = set(_source_chassis_part_ids(self)) | set(omitted_part_ids)
        chassis_payload = {
            "parts": {
                str(part_id): {
                    "mass": data.mass,
                    "center_of_mass": data.center_of_mass,
                    "inertia": data.inertia,
                    "inertia_products": data.inertia_products,
                    "orientation": data.orientation,
                    "adams_name": data.adams_name,
                }
                for part_id, data in sorted(self.compiled_parts.items())
                if part_id in chassis_payload_ids
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
                    "inertia_products": data.inertia_products,
                    "orientation": data.orientation,
                    "adams_name": data.adams_name,
                }
                for part_id, data in sorted(self.compiled_parts.items())
                if part_id
                in (
                    set(_role_part_ids(self.part_roles, "front_wheel_left"))
                    | set(_role_part_ids(self.part_roles, "front_wheel_right"))
                    | set(_role_part_ids(self.part_roles, "rear_wheel_left"))
                    | set(_role_part_ids(self.part_roles, "rear_wheel_right"))
                )
            },
            "radius_mm": self.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 344.0),
        }
        static_state = {
            "adams": "static_equilibrium",
            "package_relative_coordinates": "zero",
        }
        radius_mm = float(self.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 344.0))
        initial_state_source = "static_equilibrium"
        initial_wheel_speeds = {
            name: self.initial_forward_speed_mps * 1000.0 / radius_mm
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        }
        if self.initial_part_states:
            source_model = build_adams_source_vehicle_model(self, tire_kind=tire_kind)
            source_states = _source_initial_body_states(self, source_model)
            source_wheel_speeds = _source_initial_wheel_speeds(
                self, source_model, source_states
            )
            if not source_states or set(dict(source_wheel_speeds)) != set(
                ("front_left", "front_right", "rear_left", "rear_right")
            ):
                raise ValueError(
                    "Adams initialConditions does not contain a complete native vehicle state"
                )
            initial_state_source = "adams_initialConditions_001"
            initial_wheel_speeds = dict(source_wheel_speeds)
        bushing_property_payload = {
            key: {
                "file": property_data.path.name,
                "units": dict(sorted(property_data.units.items())),
                "damping": property_data.damping,
                "curve_point_counts": tuple(
                    len(curve) for curve in property_data.force_curves
                ),
            }
            for key, property_data in sorted(self.bushing_properties.items())
        }
        bushing_assembly_payload = {
            subsystem: tuple(
                {
                    "usage": assembly.usage,
                    "symmetry": assembly.symmetry,
                    "property": assembly.property_path.name,
                    "orientation_zp": assembly.orientation_zp,
                    "orientation_xp": assembly.orientation_xp,
                    "preload": assembly.preload,
                    "force_scaling": assembly.force_scaling,
                    "damping_force_scaling": assembly.damping_force_scaling,
                }
                for assembly in assemblies
            )
            for subsystem, assemblies in (
                ("front", self.front_bushing_assemblies),
                ("rear", self.rear_bushing_assemblies),
                ("steering", self.steering_bushing_assemblies),
                ("powertrain", self.powertrain_bushing_assemblies),
            )
        }
        joint_kind_counts = {
            kind: sum(joint.kind == kind for joint in self.source_joints)
            for kind in sorted({joint.kind for joint in self.source_joints})
        }
        field_routine_counts = {
            routine: sum(field.routine == routine for field in self.source_fields)
            for routine in sorted(
                {field.routine for field in self.source_fields if field.routine}
            )
        }
        user_function_counts = {
            entity_type: sum(
                item.entity_type == entity_type for item in self.source_user_functions
            )
            for entity_type in sorted(
                {item.entity_type for item in self.source_user_functions}
            )
        }
        source_field_ids = tuple(sorted(field.field_id for field in self.source_fields))
        mapped_source_field_ids = _source_bushing_field_ids(self)
        source_suspension_force_ids = _source_suspension_force_ids(self)
        source_drive_brake_contract = _source_drive_brake_input_contract(
            self.source_forces
        )
        if source_drive_brake_result_path is not None:
            replay_path = Path(source_drive_brake_result_path)
            if not replay_path.is_file():
                raise FileNotFoundError(
                    f"Adams source drive/brake result is missing: {replay_path}"
                )
            replay_drive, replay_brake = direct_wheel_torque_signals_from_adams_result(
                replay_path
            )
            source_drive_brake_contract["native_mapping"] = {
                "drive": "direct_wheel_torque_replay",
                "brake": "direct_wheel_torque_replay",
            }
            source_drive_brake_contract["replay"] = {
                "source_result": replay_path.name,
                "source_result_sha256": _file_hash(replay_path),
                "signal_sha256": _direct_wheel_torque_signal_hash(
                    replay_drive, replay_brake
                ),
                "drive_wheels": tuple(sorted(replay_drive)),
                "brake_wheels": tuple(sorted(replay_brake)),
                "input_units": "Adams engineering torque (N*mm)",
            }
        reference_tire_model = (
            "adams_builtin_pac2002"
            if tire_kind == "pac2002"
            else "adams_generated_brush"
        )
        native_tire_implementation = (
            "pac2002_selected_combined_slip_with_relaxation_source_offsets"
            if tire_kind == "pac2002"
            else "exact_native_brush"
        )
        return {
            "adams_assembly": self.asy_path.name,
            "tire_model": reference_tire_model,
            "native_tire_implementation": native_tire_implementation,
            "native_tire_model_scope": {
                "implemented": (
                    "pure_longitudinal_slip",
                    "pure_lateral_slip",
                    "selected_combined_slip_coefficients",
                    "first_order_relaxation",
                    "vertical_contact",
                    "source_phx_pvx_phy_pvy_offsets",
                    "selected_aligning_moment_coefficients",
                    "selected_overturning_and_rolling_resistance_moments",
                ),
                "not_implemented": (
                    "complete_combined_pac2002_parameter_set",
                    "complete_camber_and_load_scaling",
                    "complete_pac2002_aligning_moment_parameter_set",
                    "complete_pac2002_extra_moment_parameter_set",
                    "complete_adams_contact_force_law",
                ),
            },
            "source_units": {
                source: dict(sorted(units.items()))
                for source, units in sorted(self.source_units.items())
            },
            "unit_normalization": {
                "status": (
                    "complete"
                    if _source_units_complete(self.source_units)
                    else "incomplete"
                ),
                "target": {
                    "length": "mm",
                    "force": "N",
                    "mass": "kg",
                    "time": "s",
                    "angle": "rad",
                    "inertia": "kg*mm^2",
                },
            },
            "native_suspension_implementation": {
                "proxy_model": "ideal_K_without_unresolved_adams_bushing_curves",
                "source_explicit_model": "explicit_C_with_source_field_bushing_curves",
                "source_suspension_force_mapping": {
                    "source_suspension_force_ids": source_suspension_force_ids,
                    "mapped_suspension_force_ids": tuple(
                        sorted(
                            force_id
                            for force_ids in source_suspension_force_ids.values()
                            for force_id in force_ids
                        )
                    ),
                    "unmapped_suspension_force_ids": (),
                    "spring": "source_AKISPL_curve_and_marker_pair_mapped",
                    "damper": "source_AKISPL_curve_and_marker_pair_mapped",
                    "bumpstop": "source_AKISPL_curve_and_clearance_mapped",
                },
                "source_field_mapping": {
                    "source_field_ids": source_field_ids,
                    "mapped_field_ids": mapped_source_field_ids,
                    "unmapped_field_ids": tuple(
                        sorted(set(source_field_ids).difference(mapped_source_field_ids))
                    ),
                },
            },
            "adams_bushing_sources": {
                "status": "source_curves_and_application_frames_mapped_to_explicit_topology",
                "properties": bushing_property_payload,
                "assemblies": bushing_assembly_payload,
            },
            "adams_assembly_hash": self.assembly_hash,
            "adams_source_topology": {
                "status": "parsed_for_explicit_mapping",
                "part_count": len(self.compiled_parts),
                "marker_count": len(self.compiled_markers),
                "joint_count": len(self.source_joints),
                "coupler_count": len(self.source_couplers),
                "couplers": tuple(
                    {
                        "id": coupler.coupler_id,
                        "name": coupler.adams_name,
                        "joint_ids": coupler.joint_ids,
                        "kind": coupler.kind,
                        "scales": coupler.scales,
                    }
                    for coupler in self.source_couplers
                ),
                "prescribed_joint_ids": self.source_prescribed_joint_ids,
                "field_count": len(self.source_fields),
                "joint_kinds": joint_kind_counts,
                "field_routines": field_routine_counts,
                "unresolved_joint_kinds": (),
                "unresolved_coupler_ids": tuple(
                    coupler.coupler_id
                    for coupler in self.source_couplers
                    if coupler.kind not in {"R:R", "R:T"}
                    or len(coupler.joint_ids) != 2
                    or len(coupler.scales) != 2
                ),
            },
            "adams_user_function_inventory": {
                "entity_counts": user_function_counts,
                "observational_entity_types": ("REQUEST",),
                "solver_active_count": sum(
                    count
                    for entity_type, count in user_function_counts.items()
                    if entity_type != "REQUEST"
                ),
                "source_explicit_model_mapped_entity_types": ("FIELD",),
                "source_explicit_model_mapped_field_ids": mapped_source_field_ids,
                "source_explicit_model_mapped_suspension_force_ids": tuple(
                    sorted(
                        force_id
                        for force_ids in source_suspension_force_ids.values()
                        for force_id in force_ids
                    )
                ),
            },
            "chassis_mass_com_inertia_hash": _payload_hash(chassis_payload),
            "suspension_geometry_and_joint_hash": _payload_hash(suspension_payload),
            "corner_suspension_parameters_hash": _payload_hash(
                {
                    "spring": self.hashes["spring"],
                    "damper": self.hashes["damper"],
                    "bumpstop": self.hashes["bumpstop"],
                    "bushing_properties": {
                        key: value
                        for key, value in sorted(self.hashes.items())
                        if key.startswith("bushing_property:")
                    },
                }
            ),
            "wheel_mass_inertia_pose_hash": _payload_hash(wheel_payload),
            "pac2002_parameter_hash": _payload_hash(dict(sorted(self.pac2002_coefficients.items()))),
            "steering_input_mapping": {
                "input": "steering_wheel_angle",
                "ratio": self.steering_ratio,
                "rack_displacement_per_steering_wheel_angle": None,
                "source_relation": "joint coordinate couplers from compiled ADM",
                "actuator_mode": "prescribed_rotation",
                "source": "driver_demands.steering_angle",
            },
            "static_equilibrium_state_hash": _payload_hash(static_state),
            "solver_initial_state_source": initial_state_source,
            "solver_initial_state_body_count": (
                len(self.initial_part_states)
                if self.initial_part_states
                else 0
            ),
            "initial_forward_speed_mps": self.initial_forward_speed_mps,
            "initial_wheel_speeds_rad_s": initial_wheel_speeds,
            "brake_drive_input_contract": {"brake": "zero", "drive": "zero"},
            "source_drive_brake_input_contract": source_drive_brake_contract,
            "adams_force_law_mapping": {
                "spring": "source_curve_with_unsupported_proxy_attachment_and_explicit_preload",
                "damper": "source_curve",
                "bumpstop": "source_curve_with_unsupported_proxy_attachment",
                "bushing": "source_curve_and_xp_zp_application_frame_mapped",
                "user_subroutine": "unsupported_explicit_approximation",
            },
            "unsupported_adams_user_functions": self.unsupported_user_functions,
            "adams_model_reduction": {
                "status": "exact_part_mapping",
                "part_mapping": "semantic_adams_view_names"
                if self.part_roles
                else "fixed_id_fallback",
                "part_roles": {
                    role: tuple(sorted(part_ids))
                    for role, part_ids in sorted(self.part_roles.items())
                },
                "mapped_part_ids": tuple(sorted(runtime_part_ids)),
                "omitted_part_ids": omitted_part_ids,
                "mass_treatment": "exact_with_fixed_wheel_mass_condensation",
                "powertrain_treatment": "source_driveline_bodies_with_direct_wheel_torque",
                "steering_internal_treatment": "exact_source_topology",
            },
            "steering_input_samples": dict(steering_input or {}),
            "adams_source_components": {
                "assembly": self.asy_path.name,
                "front_suspension": self.front_sub_path.name,
                "rear_suspension": self.rear_sub_path.name,
                "front_tires": "TR_Front_Tires.sub",
                "rear_tires": "TR_Rear_Tires.sub",
                "steering": "TR_Steering.sub",
                "body": "TR_Body.sub",
                "powertrain": "TR_Powertrain.sub",
                "brakes": "TR_Brake_System.sub",
                "tire_property": self.tire_path.name,
                "spring_property": self.spring_path.name,
                "damper_property": self.damper_path.name,
                "bumpstop_property": self.bumpstop_path.name,
                "bushing_properties": tuple(
                    sorted(property_data.path.name for property_data in self.bushing_properties.values())
                ),
            },
            "source_file_hashes": dict(sorted(self.hashes.items())),
        }


def _source_drive_brake_input_contract(
    forces: tuple[AdamsSforceData, ...],
) -> dict[str, object]:
    """记录源模型中驱动和制动 SFORCE 的可验证活动性."""

    def channel_contract(prefix: str) -> dict[str, object]:
        selected = tuple(
            force
            for force in forces
            if (force.adams_name or "").lower().startswith(prefix)
        )
        activities = {
            force.force_id: _source_force_activity(force.function)
            for force in selected
        }
        constant_nonzero = tuple(
            force_id
            for force_id, activity in activities.items()
            if activity == "nonzero_constant"
        )
        if not selected:
            status = "zero"
        elif constant_nonzero:
            status = "nonzero_source_force"
        elif any(activity == "state_dependent" for activity in activities.values()):
            status = "state_dependent_source_force"
        else:
            status = "unknown_source_force"
        return {
            "status": status,
            "force_ids": tuple(force.force_id for force in selected),
            "force_names": tuple(force.adams_name for force in selected),
            "functions": tuple(force.function for force in selected),
            "constant_nonzero_force_ids": constant_nonzero,
        }

    return {
        "source": {
            "drive": channel_contract("tr_powertrain."),
            "brake": channel_contract("tr_brake_system."),
        },
        "native_mapping": {"drive": "zero", "brake": "zero"},
    }


def _direct_wheel_torque_signal_hash(
    drive: Mapping[str, TimeSignal], brake: Mapping[str, TimeSignal]
) -> str:
    """计算逐轮力矩信号的确定性哈希."""
    payload = {
        "drive": {
            name: signal.model_dump(mode="json")
            for name, signal in sorted(drive.items())
        },
        "brake": {
            name: signal.model_dump(mode="json")
            for name, signal in sorted(brake.items())
        },
    }
    return _payload_hash(payload)


def _source_force_activity(function: str | None) -> str:
    """分类一个 Adams 标量力函数，不把未知函数误判成零."""
    if not function:
        return "unknown"
    value = _constant_source_expression(function)
    if value is not None:
        return "zero" if abs(value) <= 1.0e-12 else "nonzero_constant"
    if re.search(
        r"\b(?:VARVAL|USER|DIF|STEP|IF|AKISPL|VR|DM)\s*\(",
        function,
        re.IGNORECASE,
    ):
        return "state_dependent"
    return "unknown"


def _constant_source_expression(expression: str) -> float | None:
    """安全求值仅含数字和四则运算的 Adams 常量表达式."""
    normalized = re.sub(
        rf"({_ADAMS_NUMBER})[Dd](?=\s*(?:[+\-]?\d|\.)|\s*$)",
        lambda match: match.group(1) + "e0",
        expression,
    )
    if re.search(r"[^0-9eE+\-*/().\s]", normalized):
        return None
    try:
        tree = ast.parse(normalized, mode="eval")
    except SyntaxError:
        return None

    def evaluate(node: ast.AST) -> float | None:
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            value = evaluate(node.operand)
            return None if value is None else (value if isinstance(node.op, ast.UAdd) else -value)
        if isinstance(node, ast.BinOp) and isinstance(
            node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)
        ):
            left = evaluate(node.left)
            right = evaluate(node.right)
            if left is None or right is None or (
                isinstance(node.op, ast.Div) and abs(right) <= 1.0e-30
            ):
                return None
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            return left / right
        return None

    value = evaluate(tree)
    return value if value is not None and math.isfinite(value) else None


def load_adams_full_vehicle_input(
    case_directory: str | Path,
    *,
    database_directory: str | Path | None = None,
    tire_property_file: str | Path | None = None,
) -> AdamsFullVehicleInput:
    """Load a real Adams reference case and parse its source model inputs."""
    case = Path(case_directory)
    raw = case / "adams_raw"
    adm = _first_file(raw, "*.adm")
    asy = _first_file(raw, "*.asy")
    database = (
        Path(database_directory)
        if database_directory is not None
        else _discover_default_database()
    )
    front_sub = database / "subsystems.tbl" / "TR_Front_Suspension.sub"
    rear_sub = database / "subsystems.tbl" / "TR_Rear_Suspension.sub"
    front_tire_sub = database / "subsystems.tbl" / "TR_Front_Tires.sub"
    rear_tire_sub = database / "subsystems.tbl" / "TR_Rear_Tires.sub"
    steering_sub = database / "subsystems.tbl" / "TR_Steering.sub"
    body_sub = database / "subsystems.tbl" / "TR_Body.sub"
    powertrain_sub = database / "subsystems.tbl" / "TR_Powertrain.sub"
    brake_sub = database / "subsystems.tbl" / "TR_Brake_System.sub"
    tire = (
        Path(tire_property_file)
        if tire_property_file is not None
        else database / "tires.tbl" / "pac2002_235_60R16.tir"
    )
    spring = database / "springs.tbl" / "MDI_125_300_spr.xml"
    damper = database / "dampers.tbl" / "MDI_default.dpr"
    bumpstop = database / "bumpstops.tbl" / "MDI_default.bum"
    for path in (
        adm,
        asy,
        front_sub,
        rear_sub,
        front_tire_sub,
        rear_tire_sub,
        steering_sub,
        body_sub,
        powertrain_sub,
        brake_sub,
        tire,
        spring,
        damper,
        bumpstop,
    ):
        if not path.is_file():
            raise FileNotFoundError(f"Adams full-vehicle input is missing: {path}")
    hashes = {
        key: _file_hash(path)
        for key, path in (
            ("adams_model", adm),
            ("adams_assembly", asy),
            ("front_subsystem", front_sub),
            ("rear_subsystem", rear_sub),
            ("front_tire_subsystem", front_tire_sub),
            ("rear_tire_subsystem", rear_tire_sub),
            ("steering_subsystem", steering_sub),
            ("body_subsystem", body_sub),
            ("powertrain_subsystem", powertrain_sub),
            ("brake_subsystem", brake_sub),
            ("tire", tire),
            ("spring", spring),
            ("damper", damper),
            ("bumpstop", bumpstop),
        )
    }
    front_hardpoints, front_parts = _parse_subsystem(front_sub)
    rear_hardpoints, rear_parts = _parse_subsystem(rear_sub)
    compiled_parts = _parse_adm_parts(adm)
    compiled_part_frames = _parse_adm_part_frames(adm)
    compiled_markers = _parse_adm_markers(adm)
    source_joints = _parse_adm_joints(adm)
    source_couplers = _parse_adm_couplers(adm)
    source_prescribed_joint_ids = _parse_adm_zero_translational_motions(adm)
    source_fields = _parse_adm_fields(adm)
    source_forces = _parse_adm_sforces(adm)
    source_variables = _parse_adm_variables(adm)
    source_user_functions = _parse_adm_user_functions(adm)
    part_roles = _semantic_part_roles(compiled_parts)
    result_files = tuple(sorted(raw.glob("*.res")))
    initial_part_states = _parse_initial_part_states(
        result_files[0] if result_files else None
    )
    initial_velocity_sign = _parse_initial_velocity_sign(
        result_files[0] if result_files else None,
        chassis_part_ids=part_roles.get("chassis", ()),
    )
    unsupported_user_functions = tuple(
        sorted(
            {
                item.function.removeprefix("USER(").removesuffix(")")
                for item in source_user_functions
                if item.entity_type != "REQUEST"
            }
        )
    )
    pac = _parse_tire(tire)
    spring_curve, spring_free_length = _parse_spring(spring)
    damper_curve = _parse_curve_file(damper, abscissa="velocity")
    bumpstop_curve = _parse_curve_file(bumpstop, abscissa="length")
    front_bushing_assemblies, front_bushing_properties = _parse_bushing_sources(
        front_sub, database
    )
    rear_bushing_assemblies, rear_bushing_properties = _parse_bushing_sources(
        rear_sub, database
    )
    steering_bushing_assemblies, steering_bushing_properties = _parse_bushing_sources(
        steering_sub, database
    )
    powertrain_bushing_assemblies, powertrain_bushing_properties = (
        _parse_bushing_sources(powertrain_sub, database)
    )
    bushing_properties = {
        **front_bushing_properties,
        **rear_bushing_properties,
        **steering_bushing_properties,
        **powertrain_bushing_properties,
    }
    for property_key, property_data in bushing_properties.items():
        hashes[f"bushing_property:{property_key}"] = _file_hash(property_data.path)
    dcf = _first_file(raw, "*.dcf")
    initial_speed = _parse_initial_speed(dcf)
    reference_mass = _parse_reference_mass(case / "adams_reference_bundle.json")
    source_units = {
        "assembly": _parse_text_units(
            adm.read_text(encoding="ascii", errors="replace")
        ),
        "front_subsystem": _parse_text_units(
            front_sub.read_text(encoding="ascii", errors="replace")
        ),
        "rear_subsystem": _parse_text_units(
            rear_sub.read_text(encoding="ascii", errors="replace")
        ),
        "tire": _parse_text_units(
            tire.read_text(encoding="ascii", errors="replace")
        ),
        "spring": _parse_xml_units(spring.read_text(encoding="latin-1")),
        "damper": _parse_text_units(
            damper.read_text(encoding="ascii", errors="replace")
        ),
        "bumpstop": _parse_text_units(
            bumpstop.read_text(encoding="ascii", errors="replace")
        ),
        "front_bushing_subsystem": _parse_text_units(
            front_sub.read_text(encoding="ascii", errors="replace")
        ),
        "rear_bushing_subsystem": _parse_text_units(
            rear_sub.read_text(encoding="ascii", errors="replace")
        ),
        "driver_control": _parse_text_units(
            dcf.read_text(encoding="ascii", errors="replace")
        ),
    }
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
        compiled_part_frames=compiled_part_frames,
        compiled_markers=compiled_markers,
        source_joints=source_joints,
        source_couplers=source_couplers,
        source_prescribed_joint_ids=source_prescribed_joint_ids,
        source_fields=source_fields,
        source_forces=source_forces,
        source_variables=source_variables,
        source_user_functions=source_user_functions,
        initial_part_states=initial_part_states,
        pac2002_coefficients=pac,
        fiala_parameters={
            "UNLOADED_RADIUS_MM": pac.get("UNLOADED_RADIUS_MM", 344.0),
            "VERTICAL_STIFFNESS_N_MM": pac.get("VERTICAL_STIFFNESS_N_MM", 210.0),
            "VERTICAL_DAMPING_N_S_MM": pac.get("VERTICAL_DAMPING_N_S_MM", 0.05),
            "CSLIP": pac.get("CSLIP_N", pac.get("CSLIP", 1000.0)),
            "CALPHA": pac.get("CALPHA_N_PER_RAD", pac.get("CALPHA", 800.0)),
            "CGAMMA": pac.get("CGAMMA", 0.0),
            "MGAMMA": pac.get("MGAMMA", 0.0),
            "CSPIN": pac.get("CSPIN", 0.0),
            "UMIN": pac.get("UMIN", 0.9),
            "UMAX": pac.get("UMAX", 1.0),
            "RELAX_LENGTH_X": pac.get("RELAX_LENGTH_X_MM", 50.0),
            "RELAX_LENGTH_Y": pac.get("RELAX_LENGTH_Y_MM", 150.0),
            "WIDTH": pac.get("WIDTH_MM", 235.0),
            "ROLLING_RESISTANCE": pac.get("ROLLING_RESISTANCE", 0.0),
            "LOW_SPEED_THRESHOLD": pac.get("LOW_SPEED_THRESHOLD", 1.0e-3),
            "DAMP_X": pac.get("DAMP_X", 0.0),
            "DAMP_Y": pac.get("DAMP_Y", 0.0),
        } if "PROPERTY_FILE_FORMAT" in tire.read_text(
            encoding="ascii", errors="replace"
        ).upper() and "FIALA" in tire.read_text(
            encoding="ascii", errors="replace"
        ).upper() else {},
        spring_curve=spring_curve,
        damper_curve=damper_curve,
        bumpstop_curve=bumpstop_curve,
        spring_free_length_mm=spring_free_length,
        unsupported_user_functions=unsupported_user_functions,
        initial_forward_speed_mps=initial_speed,
        initial_velocity_sign=initial_velocity_sign,
        source_units=source_units,
        part_roles=part_roles,
        front_inertias=_suspension_inertias(
            compiled_parts, rear=False, part_roles=part_roles
        ),
        rear_inertias=_suspension_inertias(
            compiled_parts, rear=True, part_roles=part_roles
        ),
        reference_mass_kg=reference_mass,
        bushing_properties=bushing_properties,
        front_bushing_assemblies=front_bushing_assemblies,
        rear_bushing_assemblies=rear_bushing_assemblies,
        steering_bushing_assemblies=steering_bushing_assemblies,
        powertrain_bushing_assemblies=powertrain_bushing_assemblies,
    )


def build_adams_vehicle_model(
    data: AdamsFullVehicleInput,
    *,
    tire_kind: Literal["pac2002", "native_brush", "fiala"] = "pac2002",
) -> VehicleModel:
    """Build the explicit four-corner VehicleModel from parsed Adams inputs."""
    chassis_part_ids = _role_part_ids(data.part_roles, "chassis") + _role_part_ids(
        data.part_roles, "powertrain"
    )
    chassis_mass, chassis_com, chassis_inertia = _composite_chassis(
        data.compiled_parts, part_ids=chassis_part_ids
    )
    wheel_roles = (
        ("front_wheel_left", "front_left"),
        ("front_wheel_right", "front_right"),
        ("rear_wheel_left", "rear_left"),
        ("rear_wheel_right", "rear_right"),
    )
    wheel_part_ids = {
        name: _role_part_ids(data.part_roles, role)
        for role, name in wheel_roles
    }
    wheel_masses = sum(
        _part_mass(data.compiled_parts, part_ids, 0.0)
        for part_ids in wheel_part_ids.values()
    )
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
    suspension_coefficients = dict(data.pac2002_coefficients)
    spring_curve = data.spring_curve
    bumpstop_curve = data.bumpstop_curve
    if tire_kind == "native_brush":
        # The native ABI currently has linear elastic and stop terms only.
        # Preserve the source tangent at zero deflection in the explicit proxy
        # path; the source-curve path remains available for the audit model.
        if data.spring_curve:
            suspension_coefficients["SPRING_STIFFNESS_N_MM"] = _curve_slope(
                data.spring_curve, 0.0
            )
        if data.bumpstop_curve:
            suspension_coefficients["BUMPSTOP_STIFFNESS_N_MM"] = _curve_slope(
                data.bumpstop_curve, 0.0
            )
        spring_curve = ()
        bumpstop_curve = ()
    front_x = float(data.front_hardpoints["WHEEL_CENTER"][0])
    rear_x = float(data.rear_hardpoints["WHEEL_CENTER"][0])
    wheel_front_mass = sum(
        _part_mass(data.compiled_parts, wheel_part_ids[name], 25.0)
        for name in ("front_left", "front_right")
    )
    wheel_rear_mass = sum(
        _part_mass(data.compiled_parts, wheel_part_ids[name], 25.0)
        for name in ("rear_left", "rear_right")
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
        suspension_coefficients,
        spring_curve=spring_curve,
        spring_preload=-0.5 * max(front_axle_load, 0.0),
        damper_curve=data.damper_curve,
        bumpstop_curve=bumpstop_curve,
        body_inertias=data.front_inertias,
        body_centers=_suspension_centers(
            data.compiled_parts, rear=False, part_roles=data.part_roles
        ),
    )
    rear = _build_axle(
        "rear",
        data.rear_hardpoints,
        rear_bodies,
        data.spring_free_length_mm,
        suspension_coefficients,
        spring_curve=spring_curve,
        spring_preload=-0.5 * max(rear_axle_load, 0.0),
        damper_curve=data.damper_curve,
        bumpstop_curve=bumpstop_curve,
        body_inertias=data.rear_inertias,
        body_centers=_suspension_centers(
            data.compiled_parts, rear=True, part_roles=data.part_roles
        ),
        rear_rack_fixed=True,
    )
    tire = _adams_tire_spec(
        data.fiala_parameters if tire_kind == "fiala" else data.pac2002_coefficients,
        kind=tire_kind,
    )
    wheels = tuple(
        WheelSpec(
            name=name,
            body=f"wheel_{name}",
            center_local=Vec3(),
            mass=_part_mass(data.compiled_parts, part_ids, 25.0),
            axial_inertia=_part_inertia_component(
                data.compiled_parts, part_ids, axis=1, default=800_000.0
            ),
            tire=tire,
            driven=False,
            braked=False,
        )
        for name, part_ids in wheel_part_ids.items()
    )
    model = VehicleModel(
        name=f"Demo_Vehicle_Variants_{tire_kind}_full_mbd",
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
        aerodynamic_drag=_source_aerodynamic_drag_spec(data),
    )
    if tire_kind in {"native_brush", "pac2002", "fiala"}:
        # 当前 native ABI 尚未表达 Adams .bus 的逐轴曲线。保留理想 K
        # 拓扑，避免把未解析的衬套刚度伪装成等效参数并破坏静态配平。
        model = model.model_copy(
            update={
                "front_axle": model.front_axle.model_copy(update={"bushings": ()}),
                "rear_axle": model.rear_axle.model_copy(update={"bushings": ()}),
            }
        )
    return model


def build_adams_vehicle_case(
    data: AdamsFullVehicleInput,
    model: VehicleModel,
    *,
    case_name: str,
    steering_input: TimeSignal,
    end_time: float,
    step_size: float = 0.002,
    wheel_drive_torque: Mapping[str, TimeSignal] | None = None,
    wheel_brake_torque: Mapping[str, TimeSignal] | None = None,
    source_drive_brake_result_path: str | Path | None = None,
) -> VehicleDynamicCase:
    """创建使用 Adams 初始状态和输入映射的 native 整车算例."""
    if source_drive_brake_result_path is not None:
        if wheel_drive_torque is not None or wheel_brake_torque is not None:
            raise ValueError(
                "source drive/brake result cannot be combined with explicit wheel torque signals"
            )
        wheel_drive_torque, wheel_brake_torque = (
            direct_wheel_torque_signals_from_adams_result(
                source_drive_brake_result_path
            )
        )
    initial_states = _source_initial_body_states(data, model)
    signed_speed = (
        data.initial_velocity_sign * data.initial_forward_speed_mps
    )

    def initial_wheel_speed(wheel: WheelSpec) -> float:
        rotation = _rotation_matrix_from_quaternion(
            np.asarray(wheel.pose.rotation.as_tuple(), dtype=float)
        )
        axis_world = rotation @ wheel.spin_axis.as_array()
        axis_world /= np.linalg.norm(axis_world)
        forward_world = np.array([1.0, 0.0, 0.0], dtype=float)
        forward_world -= axis_world * float(forward_world @ axis_world)
        forward_world /= np.linalg.norm(forward_world)
        rolling_coefficient = float(
            np.cross(axis_world, np.array([0.0, 0.0, -1.0]))
            @ forward_world
        ) * (wheel.tire.unloaded_radius / 1_000.0)
        if abs(rolling_coefficient) <= 1e-12:
            raise ValueError(f"Adams source wheel {wheel.name!r} has no rolling axis")
        return -signed_speed / rolling_coefficient

    wheel_speeds = (
        _source_initial_wheel_speeds(data, model, initial_states)
        if initial_states
        else tuple(
            (wheel.name, initial_wheel_speed(wheel)) for wheel in model.wheels
        )
    )
    has_brush_tire = any(
        wheel.tire.kind == "native_brush" for wheel in model.wheels
    )
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
            # Adams 日志中的 "Integration error = 1.000000E-02"；这是
            # 局部时间积分误差，不应覆盖 Native 的约束收敛阈值。
            integration_error_tolerance=1e-2,
            gravity=Vec3(x=0.0, y=0.0, z=-9810.0),
            mass_matrix_scale=1000.0,
            # The native vehicle ABI has no artificial global velocity damper.
            global_velocity_damping=0.0,
            # Guard thresholds only; the integrator never clips these values.
            max_linear_acceleration=1.0e9,
            max_angular_acceleration=1.0e9,
            max_linear_velocity=1.0e9,
            max_angular_velocity=1.0e9,
            velocity_recovery_enabled=False,
            velocity_recovery_linear_limit=1.0e5,
            velocity_recovery_angular_limit=2.0e3,
            constraint_tolerance=1e-4,
            # Adams 源静态输出的 CONVEL 角度残差最大约为 0.004 rad；这里只
            # 放宽初始角度合法性检查，积分阶段仍使用严格的位置收敛阈值。
            initial_state_angle_tolerance_rad=0.01,
            velocity_tolerance=1e-4,
            projection_failure_tolerance=0.1,
            # 刷毛模型在摩擦饱和边界附近的隐式状态校正需要更多 Newton
            # 迭代；不改变时间步、积分误差或物理收敛容差。
            projection_max_iterations=80 if has_brush_tire else 40,
        ),
        vehicle=model,
        road=RoadSurfaceSpec(),
        steering_input=steering_input,
        wheel_drive_torque=tuple(
            (name, wheel_drive_torque[name])
            for name in ("front_left", "front_right", "rear_left", "rear_right")
            if wheel_drive_torque is not None and name in wheel_drive_torque
        ),
        wheel_brake_torque=tuple(
            (name, wheel_brake_torque[name])
            for name in ("front_left", "front_right", "rear_left", "rear_right")
            if wheel_brake_torque is not None and name in wheel_brake_torque
        ),
        initial_wheel_speeds=wheel_speeds,
        static_equilibrium=not bool(initial_states),
        initial_states=initial_states,
        initial_forward_speed_mps=data.initial_forward_speed_mps,
        initial_velocity_sign=data.initial_velocity_sign,
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


def build_native_rack_steering_model(model: VehicleModel) -> VehicleModel:
    """构造只规定齿条位移、不给方向盘施加运动的 Native 模型副本."""
    steering = model.steering.model_copy(
        update={
            "input": "rack_displacement",
            "actuator_mode": "prescribed_translation",
            "actuator_body": None,
            "actuator_reaction_body": "rack_housing",
        }
    )
    # 齿条位移成为边界条件后，原 R:T 齿轮副不能继续作为第二条输入链。
    # 保留纯转动耦合和齿条壳体衬套，只断开包含齿条平移坐标的上游耦合。
    couplers = tuple(
        coupler
        for coupler in model.coordinate_couplers
        if coupler.coordinate_a != "translation"
        and coupler.coordinate_b != "translation"
    )
    return model.model_copy(
        update={"steering": steering, "coordinate_couplers": couplers}
    )


def direct_wheel_torque_signals_from_adams_result(
    path: str | Path,
) -> tuple[dict[str, TimeSignal], dict[str, TimeSignal]]:
    """从 Adams 动态结果提取同向轮轴坐标下的逐轮转矩信号."""
    from .time_domain import AdamsResultChannel, parse_adams_result_history

    channels = {
        "rear_left_drive": AdamsResultChannel(
            "differential", "output_torque_left_rear"
        ),
        "rear_right_drive": AdamsResultChannel(
            "differential", "output_torque_right_rear"
        ),
        "front_left_brake": AdamsResultChannel("brake_torques", "left_front"),
        "front_right_brake": AdamsResultChannel("brake_torques", "right_front"),
        "rear_left_brake": AdamsResultChannel("brake_torques", "left_rear"),
        "rear_right_brake": AdamsResultChannel("brake_torques", "right_rear"),
    }
    history = parse_adams_result_history(path, channels)

    def signal(
        name: str,
        *,
        scale: float = 1.0,
        magnitude: bool = False,
    ) -> TimeSignal:
        values = history.channels[name]
        if magnitude:
            values = tuple(abs(value) for value in values)
        return TimeSignal(
            times=history.time,
            values=tuple(scale * value for value in values),
        )

    zero_drive = TimeSignal(
        times=history.time,
        values=tuple(0.0 for _ in history.time),
    )

    return (
        {
            # 源模型导入时已将 Native 轮轴统一到 Adams 正滚动方向，
            # 因而驱动力矩通道可直接回放，不再进行符号补偿。
            "front_left": zero_drive,
            "front_right": zero_drive,
            "rear_left": signal("rear_left_drive"),
            "rear_right": signal("rear_right_drive"),
        },
        {
            "front_left": signal("front_left_brake", magnitude=True),
            "front_right": signal("front_right_brake", magnitude=True),
            "rear_left": signal("rear_left_brake", magnitude=True),
            "rear_right": signal("rear_right_brake", magnitude=True),
        },
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
    body_inertias: Mapping[str, Matrix3] | None = None,
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
        stiffness=max(
            1e-6,
            _curve_slope(bumpstop_curve, 0.0)
            if bumpstop_curve
            else float(tire_coefficients.get("BUMPSTOP_STIFFNESS_N_MM", 1_000.0)),
        ),
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
    parts: Mapping[int, AdamsPartData],
    *,
    rear: bool,
    part_roles: Mapping[str, tuple[int, ...]] | None = None,
) -> dict[str, tuple[float, float, float]]:
    """Return compiled Adams COMs for the generated suspension rigid bodies."""
    prefix = "rear" if rear else "front"
    ids = {
        "upper_arm": _role_part_ids(part_roles, f"{prefix}_upper_arm"),
        "lower_arm": _role_part_ids(part_roles, f"{prefix}_lower_arm")
        + _role_part_ids(part_roles, f"{prefix}_lower_arm2"),
        "upright": _role_part_ids(part_roles, f"{prefix}_upright")
        + _role_part_ids(part_roles, f"{prefix}_spindle"),
        "tie_rod": _role_part_ids(part_roles, f"{prefix}_tie_rod_inner")
        + _role_part_ids(part_roles, f"{prefix}_tie_rod_outer"),
    }
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
    if not rear:
        rack_parts = [
            parts[part_id]
            for part_id in _role_part_ids(part_roles, "rack")
            if part_id in parts
        ]
        if rack_parts:
            mass = sum(part.mass for part in rack_parts)
            if mass > 0.0:
                result["rack"] = tuple(
                    sum(part.mass * part.center_of_mass[index] for part in rack_parts)
                    / mass
                    for index in range(3)
                )
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
    body: str, inertias: Mapping[str, Matrix3]
) -> Matrix3:
    base = body.rsplit("_", 1)[0] if body.endswith(("_L", "_R")) else body
    return inertias.get(
        base,
        (
            (1_000.0, 0.0, 0.0),
            (0.0, 1_000.0, 0.0),
            (0.0, 0.0, 1_000.0),
        ),
    )


def _quaternion_from_rotation(rotation: np.ndarray) -> Quaternion:
    """将源旋转矩阵转换为 schema 使用的标量在前四元数."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.asarray(
            (
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ),
            dtype=float,
        )
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = 2.0 * math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        quaternion = np.asarray(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ),
            dtype=float,
        )
    elif matrix[1, 1] > matrix[2, 2]:
        scale = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        quaternion = np.asarray(
            (
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ),
            dtype=float,
        )
    else:
        scale = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        quaternion = np.asarray(
            (
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ),
            dtype=float,
        )
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Adams part has an invalid orientation")
    quaternion /= norm
    return Quaternion(w=float(quaternion[0]), x=float(quaternion[1]), y=float(quaternion[2]), z=float(quaternion[3]))


def _rotation_matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """将标量在前四元数转换为主动旋转矩阵."""
    w, x, y, z = np.asarray(quaternion, dtype=float)
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _source_part_frame(
    data: AdamsFullVehicleInput, part_id: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return one source part's reference origin and body-to-vehicle rotation."""
    frame = data.compiled_part_frames.get(part_id)
    if frame is not None:
        return np.asarray(frame.reference_origin, dtype=float), np.asarray(
            frame.orientation, dtype=float
        )
    part = data.compiled_parts.get(part_id)
    if part is None:
        raise ValueError(f"compiled Adams model has no frame for part {part_id}")
    return np.asarray(part.reference_origin, dtype=float), np.asarray(
        part.orientation, dtype=float
    )


def _source_marker_pose(
    data: AdamsFullVehicleInput, marker_id: int
) -> tuple[int, np.ndarray, np.ndarray]:
    """Convert one compiled marker from its part frame to vehicle coordinates."""
    try:
        marker = data.compiled_markers[marker_id]
    except KeyError as exc:
        raise ValueError(f"compiled Adams model has no marker {marker_id}") from exc
    origin, rotation = _source_part_frame(data, marker.part_id)
    return (
        marker.part_id,
        origin + rotation @ np.asarray(marker.local_position, dtype=float),
        rotation @ np.asarray(marker.local_orientation, dtype=float),
    )


def _source_bushing_frame(
    assembly: AdamsBushingAssembly,
    *,
    side: Literal["L", "R"] | None = None,
) -> np.ndarray:
    """根据 Adams 的 XP/ZP 方向构造源衬套坐标系."""
    reflection = np.eye(3, dtype=float)
    if side == "R" and assembly.symmetry.strip().lower() == "left/right":
        reflection[1, 1] = -1.0

    x_axis = reflection @ np.asarray(assembly.orientation_xp, dtype=float)
    z_axis = reflection @ np.asarray(assembly.orientation_zp, dtype=float)
    x_norm = float(np.linalg.norm(x_axis))
    if not math.isfinite(x_norm) or x_norm <= 1e-12:
        raise ValueError(f"Adams bushing {assembly.usage!r} has an invalid XP direction")
    x_axis /= x_norm
    z_axis -= x_axis * float(x_axis @ z_axis)
    z_norm = float(np.linalg.norm(z_axis))
    if not math.isfinite(z_norm) or z_norm <= 1e-12:
        raise ValueError(f"Adams bushing {assembly.usage!r} has an invalid ZP direction")
    z_axis /= z_norm
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if not math.isfinite(y_norm) or y_norm <= 1e-12:
        raise ValueError(f"Adams bushing {assembly.usage!r} has degenerate XP/ZP directions")
    y_axis /= y_norm
    return np.column_stack((x_axis, y_axis, z_axis))


def _source_bushing_side(
    source_name: str, *body_names: str
) -> Literal["L", "R"] | None:
    """根据 Adams 名称或 native 刚体名称识别左右侧."""
    match = re.search(r"\.b[kg]([lr])_", source_name.lower())
    if match is not None:
        return "R" if match.group(1) == "r" else "L"
    if any(name.endswith("_R") for name in body_names):
        return "R"
    if any(name.endswith("_L") for name in body_names):
        return "L"
    return None


def _source_fixed_part_aliases(data: AdamsFullVehicleInput) -> dict[int, int]:
    """Resolve massless source parts attached by fixed joints to a massful part."""
    massful = set(data.compiled_parts)
    aliases: dict[int, int] = {}
    for joint in data.source_joints:
        if joint.kind != "FIXED":
            continue
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        part_i, part_j = marker_i.part_id, marker_j.part_id
        if part_i not in massful and part_j in massful:
            aliases[part_i] = part_j
        elif part_j not in massful and part_i in massful:
            aliases[part_j] = part_i

    def resolve(part_id: int) -> int:
        visited: set[int] = set()
        current = part_id
        while current in aliases and current not in visited:
            visited.add(current)
            current = aliases[current]
        return current

    return {
        part_id: resolve(part_id)
        for part_id in set(aliases) | set(massful)
    }


def _source_chassis_part_ids(data: AdamsFullVehicleInput) -> tuple[int, ...]:
    return tuple(_role_part_ids(data.part_roles, "chassis"))


def _source_body_name(
    data: AdamsFullVehicleInput, part_id: int, chassis_ids: set[int]
) -> str:
    if part_id in chassis_ids:
        return "chassis"
    part = data.compiled_parts.get(part_id)
    if part is None:
        raise ValueError(f"source body part {part_id} has no mass data")
    if part_id in set(_role_part_ids(data.part_roles, "powertrain")):
        return "powertrain"
    source_name = (part.adams_name or "").lower()
    if source_name.endswith("ges_rack"):
        return "rack"
    if source_name.endswith("ges_rack_housing"):
        return "rack_housing"
    for side_token, side in ((".gel_", "L"), (".ger_", "R")):
        if side_token not in source_name:
            continue
        if source_name.endswith("upright"):
            return f"upright_{side}"
        if source_name.endswith("spindle"):
            return f"spindle_{side}"
    return f"adams_part_{part_id}"


def _source_drivetrain_part_ids(data: AdamsFullVehicleInput) -> set[int]:
    """返回显式传动链使用的正质量源部件."""
    drivetrain_tokens = (
        "drive_shaft",
        "tripot",
        "diff_",
        "differential",
    )
    return {
        part_id
        for part_id, part in data.compiled_parts.items()
        if part.mass > 0.0
        and any(
            token in (part.adams_name or "").lower()
            for token in drivetrain_tokens
        )
    }


def _source_axle_part_ids(
    data: AdamsFullVehicleInput,
    *,
    rear: bool,
    include_drivetrain: bool = False,
) -> tuple[int, ...]:
    # 通用代理模型按 Chrono 的子系统边界只保留刚性动力总成体；
    # Adams 源显式模型可选择加入驱动轴、三脚架和差速器输出体。
    drivetrain_ids = _source_drivetrain_part_ids(data)
    powertrain_role_ids = set(_role_part_ids(data.part_roles, "powertrain"))
    prefixes = (
        ("tr_rear_suspension.",)
        if rear
        else ("tr_front_suspension.", "tr_steering.")
    )
    result: set[int] = set()
    for part_id, part in data.compiled_parts.items():
        if part.mass <= 0.0:
            continue
        if rear and part_id in powertrain_role_ids:
            result.add(part_id)
            continue
        source_name = (part.adams_name or "").lower()
        if part_id in drivetrain_ids and not include_drivetrain:
            continue
        if any(source_name.startswith(prefix) for prefix in prefixes):
            result.add(part_id)
    if not rear:
        # Keep the source rack support as a real body.  Its source prismatic
        # joint and steering bushings then remain part of the same KKT system.
        result.update(
            part_id
            for part_id, part in data.compiled_parts.items()
            if part.mass > 0.0
            and (part.adams_name or "").lower().endswith("ges_rack_housing")
        )
    if rear and include_drivetrain:
        result.update(drivetrain_ids)
    return tuple(sorted(result))


def _source_native_body_part_ids(
    data: AdamsFullVehicleInput, *, include_drivetrain: bool = False
) -> set[int]:
    """返回实际进入 native 或轮端质量凝聚的源部件集合."""
    result = set(_source_chassis_part_ids(data))
    result.update(
        _source_axle_part_ids(
            data, rear=False, include_drivetrain=include_drivetrain
        )
    )
    result.update(
        _source_axle_part_ids(
            data, rear=True, include_drivetrain=include_drivetrain
        )
    )
    for role in (
        "front_wheel_left",
        "front_wheel_right",
        "rear_wheel_left",
        "rear_wheel_right",
    ):
        result.update(_role_part_ids(data.part_roles, role))
    return result


def _source_body_spec(
    data: AdamsFullVehicleInput,
    part_id: int,
    chassis_ids: set[int],
    reference_rotation: np.ndarray | None = None,
    static_rotation_axis_world: np.ndarray | None = None,
) -> RigidBodySpec:
    """Create a mass and inertia preserving body spec for one source part."""
    part = data.compiled_parts.get(part_id)
    if part is None or part.mass <= 0.0:
        raise ValueError(f"source body part {part_id} is not a positive-mass part")
    origin, source_rotation = _source_part_frame(data, part_id)
    rotation = (
        np.asarray(reference_rotation, dtype=float)
        if reference_rotation is not None
        else source_rotation
    )
    center = np.asarray(part.center_of_mass, dtype=float)
    center_local = rotation.T @ (center - origin)
    inertia = _matrix_tuple(rotation.T @ part.inertia_about_com_global() @ rotation)
    static_rotation_axis_local = None
    if static_rotation_axis_world is not None:
        axis_local = rotation.T @ np.asarray(static_rotation_axis_world, dtype=float)
        static_rotation_axis_local = _vec(
            tuple(float(value) for value in axis_local)
        )
    return RigidBodySpec(
        name=_source_body_name(data, part_id, chassis_ids),
        pose=Pose(
            translation=_vec(tuple(float(value) for value in origin)),
            rotation=_quaternion_from_rotation(rotation),
        ),
        mass=float(part.mass),
        center_of_mass=_vec(tuple(float(value) for value in center_local)),
        inertia=inertia,
        fixed=False,
        static_rotation_axis_local=static_rotation_axis_local,
    )


def _source_translational_reference_frames(
    data: AdamsFullVehicleInput,
    *,
    allowed_part_ids: set[int],
    chassis_ids: set[int],
) -> dict[int, np.ndarray]:
    """Choose body frames that preserve each Adams TRANSLATIONAL reference angle."""
    aliases = _source_fixed_part_aliases(data)
    allowed = allowed_part_ids | chassis_ids
    reference: dict[int, np.ndarray] = {}
    for joint in data.source_joints:
        if joint.kind != "TRANSLATIONAL":
            continue
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        resolved_i = aliases.get(marker_i.part_id, marker_i.part_id)
        resolved_j = aliases.get(marker_j.part_id, marker_j.part_id)
        if resolved_i not in allowed or resolved_j not in allowed:
            continue
        _, _, frame_i = _source_marker_pose(data, joint.marker_i)
        _, _, frame_j = _source_marker_pose(data, joint.marker_j)
        for part_id, frame in ((resolved_i, frame_i), (resolved_j, frame_j)):
            previous = reference.get(part_id)
            if previous is not None and not np.allclose(
                previous, frame, rtol=0.0, atol=1e-8
            ):
                raise ValueError(
                    f"Adams part {part_id} has incompatible translational joint frames"
                )
            reference[part_id] = frame.copy()
    return reference


def _source_spindle_static_rotation_axes(
    data: AdamsFullVehicleInput,
    *,
    allowed_part_ids: set[int],
) -> dict[int, np.ndarray]:
    """Return revolute axes for bodies fixed to source wheel bodies."""
    aliases = _source_fixed_part_aliases(data)
    wheel_ids: set[int] = set()
    for role in (
        "front_wheel_left",
        "front_wheel_right",
        "rear_wheel_left",
        "rear_wheel_right",
    ):
        wheel_ids.update(_role_part_ids(data.part_roles, role))
    spindle_ids: set[int] = set()
    for joint in data.source_joints:
        if joint.kind != "FIXED":
            continue
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        part_i = aliases.get(marker_i.part_id, marker_i.part_id)
        part_j = aliases.get(marker_j.part_id, marker_j.part_id)
        if part_i in wheel_ids and part_j in allowed_part_ids:
            spindle_ids.add(part_j)
        if part_j in wheel_ids and part_i in allowed_part_ids:
            spindle_ids.add(part_i)
    axes: dict[int, np.ndarray] = {}
    for joint in data.source_joints:
        if joint.kind != "REVOLUTE":
            continue
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        for marker_id, source_part_id in (
            (joint.marker_i, marker_i.part_id),
            (joint.marker_j, marker_j.part_id),
        ):
            part_id = aliases.get(source_part_id, source_part_id)
            if part_id not in spindle_ids:
                continue
            _, _, frame = _source_marker_pose(data, marker_id)
            axis = np.asarray(frame[:, 2], dtype=float)
            axis /= np.linalg.norm(axis)
            previous = axes.get(part_id)
            if previous is not None and not np.allclose(
                previous, axis, rtol=0.0, atol=1e-8
            ) and not np.allclose(previous, -axis, rtol=0.0, atol=1e-8):
                raise ValueError(
                    f"Adams spindle part {part_id} has incompatible revolute axes"
                )
            axes[part_id] = axis
    missing = spindle_ids - set(axes)
    if missing:
        raise ValueError(
            f"Adams spindle parts have no source revolute axis: {sorted(missing)}"
        )

    convel_bodies: set[int] = set()
    spherical_bodies: set[int] = set()
    for joint in data.source_joints:
        if joint.kind not in {"CONVEL", "SPHERICAL"}:
            continue
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        target = convel_bodies if joint.kind == "CONVEL" else spherical_bodies
        for source_part_id in (marker_i.part_id, marker_j.part_id):
            part_id = aliases.get(source_part_id, source_part_id)
            if part_id in allowed_part_ids:
                target.add(part_id)
    for joint in data.source_joints:
        if joint.kind != "TRANSLATIONAL":
            continue
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        part_i = aliases.get(marker_i.part_id, marker_i.part_id)
        part_j = aliases.get(marker_j.part_id, marker_j.part_id)
        pair = ((part_i, joint.marker_i), (part_j, joint.marker_j))
        convel_members = [item for item in pair if item[0] in convel_bodies]
        spherical_members = [item for item in pair if item[0] in spherical_bodies]
        if len(convel_members) != 1 or len(spherical_members) != 1:
            continue
        if convel_members[0][0] == spherical_members[0][0]:
            continue
        # The two-piece tie rod has one shared static spin mode across the
        # CONVEL/TRANSLATIONAL/SPHERICAL chain.  Pin that mode once on the
        # CONVEL member; pinning both segments duplicates the same gauge and
        # removes a physical reaction equation from the static KKT system.
        body, marker_id = convel_members[0]
        _, _, frame = _source_marker_pose(data, marker_id)
        axis = np.asarray(frame[:, 2], dtype=float)
        axis /= np.linalg.norm(axis)
        axes[body] = axis
    return axes


def _source_joint_specs(
    data: AdamsFullVehicleInput,
    *,
    allowed_part_ids: set[int],
    chassis_ids: set[int],
) -> tuple[IdealJointSpec, ...]:
    """Translate supported compiled Adams joints without reducing their frames."""
    aliases = _source_fixed_part_aliases(data)
    allowed = allowed_part_ids | chassis_ids
    result: list[IdealJointSpec] = []
    for joint in data.source_joints:
        marker_i = data.compiled_markers.get(joint.marker_i)
        marker_j = data.compiled_markers.get(joint.marker_j)
        if marker_i is None or marker_j is None:
            continue
        resolved_i = aliases.get(marker_i.part_id, marker_i.part_id)
        resolved_j = aliases.get(marker_j.part_id, marker_j.part_id)
        if resolved_i not in allowed or resolved_j not in allowed:
            continue
        body_i = _source_body_name(data, resolved_i, chassis_ids)
        body_j = _source_body_name(data, resolved_j, chassis_ids)
        if body_i == body_j:
            continue
        _, point_i, frame_i = _source_marker_pose(data, joint.marker_i)
        _, point_j, frame_j = _source_marker_pose(data, joint.marker_j)
        kind = joint.kind.upper()
        kwargs: dict[str, object] = {
            "name": joint.adams_name or f"adams_joint_{joint.joint_id}",
            "body_a": body_i,
            "body_b": body_j,
            "point_a": _vec(tuple(float(value) for value in point_i)),
            "point_b": _vec(tuple(float(value) for value in point_j)),
        }
        if kind in {"REVOLUTE", "TRANSLATIONAL", "CYLINDRICAL"}:
            kwargs["axis_a"] = _vec(tuple(float(value) for value in frame_i[:, 2]))
            kwargs["axis_b"] = _vec(tuple(float(value) for value in frame_j[:, 2]))
            # Adams 的分动器把平移副的轴向位移用零值 MOTION 约束住。
            # native 的 prismatic 只包含其几何五约束，因此这里仅对源文件
            # 明确施加零平移 MOTION 的关节补上第六个约束。
            kwargs["kind"] = (
                "fixed"
                if kind == "TRANSLATIONAL"
                and joint.joint_id in data.source_prescribed_joint_ids
                else {
                "REVOLUTE": "revolute",
                "TRANSLATIONAL": "prismatic",
                "CYLINDRICAL": "cylindrical",
                }[kind]
            )
        elif kind == "SPHERICAL":
            kwargs["kind"] = "spherical"
        elif kind == "FIXED":
            kwargs["kind"] = "fixed"
        elif kind == "HOOKE":
            kwargs["kind"] = "universal"
            kwargs["axis_a"] = _vec(tuple(float(value) for value in frame_i[:, 0]))
            kwargs["axis_b"] = _vec(tuple(float(value) for value in frame_j[:, 1]))
        elif kind == "CONVEL":
            kwargs["kind"] = "constant_velocity"
            kwargs["axis_a"] = _vec(tuple(float(value) for value in frame_i[:, 0]))
            kwargs["axis_a_secondary"] = _vec(
                tuple(float(value) for value in frame_i[:, 1])
            )
            kwargs["axis_b"] = _vec(tuple(float(value) for value in frame_j[:, 1]))
            kwargs["axis_b_secondary"] = _vec(
                tuple(float(value) for value in frame_j[:, 0])
            )
            kwargs["constant_velocity_angle_target"] = _source_convel_angle_target(
                data, joint
            )
        else:
            raise ValueError(
                f"supported Adams source joint {joint.joint_id} has unknown kind {kind!r}"
            )
        result.append(IdealJointSpec(**kwargs))
    return tuple(result)


def _source_coordinate_coupler_specs(
    data: AdamsFullVehicleInput,
) -> tuple[JointCoordinateCouplerSpec, ...]:
    joints = {joint.joint_id: joint for joint in data.source_joints}
    coordinate = {"R": "rotation", "T": "translation"}
    result: list[JointCoordinateCouplerSpec] = []
    for coupler in data.source_couplers:
        kinds = coupler.kind.split(":")
        if (
            len(coupler.joint_ids) != 2
            or len(coupler.scales) != 2
            or len(kinds) != 2
            or any(kind not in coordinate for kind in kinds)
        ):
            raise ValueError(
                f"Adams coupler {coupler.coupler_id} has unsupported linear topology"
            )
        try:
            first = joints[coupler.joint_ids[0]]
            second = joints[coupler.joint_ids[1]]
        except KeyError as exc:
            raise ValueError(
                f"Adams coupler {coupler.coupler_id} references an unknown joint"
            ) from exc
        result.append(
            JointCoordinateCouplerSpec(
                name=coupler.adams_name or f"adams_coupler_{coupler.coupler_id}",
                joint_a=f"front_{first.adams_name or f'adams_joint_{first.joint_id}'}",
                coordinate_a=coordinate[kinds[0]],
                scale_a=coupler.scales[0],
                joint_b=f"front_{second.adams_name or f'adams_joint_{second.joint_id}'}",
                coordinate_b=coordinate[kinds[1]],
                scale_b=coupler.scales[1],
            )
        )
    return tuple(result)


def _source_field_usage(field: AdamsFieldData) -> str:
    if not field.adams_name:
        raise ValueError(f"Adams field {field.field_id} has no source name")
    match = re.search(
        r"\.b(?:kl|kr|gl|gr|gs)_([^.]*)\.field$",
        field.adams_name,
        re.IGNORECASE,
    )
    if match is None:
        raise ValueError(
            f"Adams field {field.field_id} has no supported bushing usage: {field.adams_name!r}"
        )
    return match.group(1).lower()


def _source_bushing_assemblies(
    data: AdamsFullVehicleInput, source_name: str
) -> tuple[AdamsBushingAssembly, ...]:
    lower = source_name.lower()
    if lower.startswith("tr_front_suspension."):
        return data.front_bushing_assemblies
    if lower.startswith("tr_rear_suspension."):
        return data.rear_bushing_assemblies
    if lower.startswith("tr_steering."):
        return data.steering_bushing_assemblies
    if lower.startswith("tr_powertrain."):
        return data.powertrain_bushing_assemblies
    raise ValueError(f"Adams field has no source subsystem: {source_name!r}")


def _source_bushing_specs(
    data: AdamsFullVehicleInput,
    *,
    allowed_part_ids: set[int],
    chassis_ids: set[int],
) -> tuple[Bushing6x6, ...]:
    """Map source FIELD markers and .bus curves into explicit six-axis bushings."""
    aliases = _source_fixed_part_aliases(data)
    allowed = allowed_part_ids | chassis_ids
    result: list[Bushing6x6] = []
    for source_field in data.source_fields:
        source_name = source_field.adams_name or ""
        lower = source_name.lower()
        source_prefixes = (
            ("tr_rear_suspension.", "tr_powertrain.")
            if allowed_part_ids
            and any(
                (data.compiled_parts.get(part_id).adams_name or "").lower().startswith(
                    "tr_rear_suspension."
                )
                for part_id in allowed_part_ids
                if data.compiled_parts.get(part_id) is not None
            )
            else ("tr_front_suspension.", "tr_steering.")
        )
        if not lower.startswith(source_prefixes):
            continue
        marker_i = data.compiled_markers.get(source_field.marker_i)
        marker_j = data.compiled_markers.get(source_field.marker_j)
        if marker_i is None or marker_j is None:
            continue
        resolved_i = aliases.get(marker_i.part_id, marker_i.part_id)
        resolved_j = aliases.get(marker_j.part_id, marker_j.part_id)
        if resolved_i not in allowed or resolved_j not in allowed:
            continue
        body_i = _source_body_name(data, resolved_i, chassis_ids)
        body_j = _source_body_name(data, resolved_j, chassis_ids)
        if body_i == body_j:
            continue
        usage = _source_field_usage(source_field)
        assemblies = _source_bushing_assemblies(data, source_name)
        matching = tuple(
            assembly
            for assembly in assemblies
            if assembly.usage.strip().lower() == usage
        )
        if len(matching) != 1:
            raise ValueError(
                f"Adams field {source_field.field_id} usage {usage!r} does not resolve to one .bus assembly"
            )
        assembly = matching[0]
        try:
            property_data = data.bushing_properties[assembly.property_key]
        except KeyError as exc:
            raise ValueError(
                f"Adams bushing property {assembly.property_key!r} is not loaded"
            ) from exc
        _, point_i, _ = _source_marker_pose(data, source_field.marker_i)
        _, point_j, _ = _source_marker_pose(data, source_field.marker_j)
        bushing_side = _source_bushing_side(source_name, body_i, body_j)
        bushing_frame = _source_bushing_frame(assembly, side=bushing_side)
        # Adams FIELD 的源约定是：相对变形以 J 端坐标表达，输出力是
        # 作用在 I 端的力。交换 native 的 A/B 端后，native 的
        # pose_a^-1 * pose_b 正好表示同一个 I 相对 J 的量，输出的
        # body_b 力也正好落在源 I 端。不能在这里仅靠改变力的符号，
        # 因为六轴曲线和阻尼坐标可能各向异性。
        force_curves = tuple(
            tuple(
                (float(coordinate), float(force * assembly.force_scaling[index]))
                for coordinate, force in curve
            )
            for index, curve in enumerate(property_data.force_curves)
        )
        damping = tuple(
            float(property_data.damping[index] * assembly.damping_force_scaling[index])
            for index in range(6)
        )
        # Adams FIELD 以 J marker 表达 I 相对 J 的变形和反力。native
        # 交换 A/B 端后得到相同的相对量及受力端；源转角和阻尼坐标使用
        # XYZ Cardan 角，不能用旋转向量近似。
        result.append(
            Bushing6x6(
                name=f"adams_field_{source_field.field_id}",
                body_a=body_j,
                body_b=body_i,
                pose_a=Pose(
                    translation=_vec(tuple(float(value) for value in point_j)),
                    rotation=_quaternion_from_rotation(bushing_frame),
                ),
                pose_b=Pose(
                    translation=_vec(tuple(float(value) for value in point_i)),
                    rotation=_quaternion_from_rotation(bushing_frame),
                ),
                stiffness=tuple(tuple(0.0 for _ in range(6)) for _ in range(6)),
                damping=damping,  # type: ignore[arg-type]
                preload=tuple(-value for value in assembly.preload),
                force_curves=force_curves,
                force_curve_interpolation="akima",
                rotation_coordinates="cardan_xyz",
            )
        )
    return tuple(result)


def _source_bushing_field_ids(data: AdamsFullVehicleInput) -> tuple[int, ...]:
    """Return Adams FIELD ids represented by the source explicit C model."""
    chassis_ids = set(_source_chassis_part_ids(data))
    mapped: list[int] = []
    for rear in (False, True):
        axle_bushings = _source_bushing_specs(
            data,
            allowed_part_ids=set(
                _source_axle_part_ids(
                    data,
                    rear=rear,
                    include_drivetrain=True,
                )
            ),
            chassis_ids=chassis_ids,
        )
        for bushing in axle_bushings:
            match = re.fullmatch(r"adams_field_(\d+)", bushing.name)
            if match is not None:
                mapped.append(int(match.group(1)))
    return tuple(sorted(set(mapped)))


def _source_suspension_force_ids(
    data: AdamsFullVehicleInput,
) -> dict[str, tuple[int, ...]]:
    """返回按 native 力元分类的源悬架 SFORCE 编号."""
    result = {"spring": [], "damper": [], "bumpstop": []}
    for force in data.source_forces:
        if force.kind.upper() != "TRANSLATIONAL":
            continue
        source_name = (force.adams_name or "").lower()
        if not (
            source_name.startswith("tr_front_suspension.")
            or source_name.startswith("tr_rear_suspension.")
        ):
            continue
        if ".nsl_ride_spring.force" in source_name or ".nsr_ride_spring.force" in source_name:
            result["spring"].append(force.force_id)
        elif ".dal_ride_damper.force" in source_name or ".dar_ride_damper.force" in source_name:
            result["damper"].append(force.force_id)
        elif ".bul_jounce_stop.force" in source_name or ".bur_jounce_stop.force" in source_name:
            result["bumpstop"].append(force.force_id)
    return {
        kind: tuple(sorted(set(force_ids)))
        for kind, force_ids in result.items()
    }


def _source_convel_angle_target(
    data: AdamsFullVehicleInput, joint: AdamsJointData
) -> float:
    """Return the source CONVEL phase relation at the initial state."""
    marker_i = data.compiled_markers.get(joint.marker_i)
    marker_j = data.compiled_markers.get(joint.marker_j)
    if marker_i is None or marker_j is None:
        return 0.0
    state_i = data.initial_part_states.get(marker_i.part_id)
    state_j = data.initial_part_states.get(marker_j.part_id)
    if state_i is None or state_j is None:
        return 0.0
    rotation_i = np.asarray(state_i.rotation, dtype=float)
    rotation_j = np.asarray(state_j.rotation, dtype=float)
    frame_i = rotation_i @ np.asarray(marker_i.local_orientation, dtype=float)
    frame_j = rotation_j @ np.asarray(marker_j.local_orientation, dtype=float)
    x_i, y_i = frame_i[:, 0], frame_i[:, 1]
    y_j, x_j = frame_j[:, 1], frame_j[:, 0]
    norms = tuple(float(np.linalg.norm(axis)) for axis in (x_i, y_i, y_j, x_j))
    if any(norm <= 1e-12 for norm in norms):
        return 0.0
    target = (x_i @ y_j) / (norms[0] * norms[2])
    target += (y_i @ x_j) / (norms[1] * norms[3])
    return float(np.clip(target, -2.0, 2.0))


def _source_force_constant(function: str | None, pattern: str, description: str) -> float:
    if function is None:
        raise ValueError(f"Adams source {description} has no force function")
    match = re.search(pattern, function, re.IGNORECASE)
    if match is None:
        raise ValueError(
            f"Adams source {description} force function is not supported: {function!r}"
        )
    return _adams_float(match.group(1))


def _source_stop_clearance(
    data: AdamsFullVehicleInput, force: AdamsSforceData
) -> float:
    """Resolve a stop clearance from the SFORCE expression or its VARVAL source."""
    function = force.function
    if function is not None:
        direct = re.search(
            rf"max\s*\(\s*0\s*,\s*({_ADAMS_NUMBER})\s*-\s*dm\s*\(",
            function,
            re.IGNORECASE,
        )
        if direct is not None:
            return _adams_float(direct.group(1))
        variable_match = re.search(
            r"varval\s*\(\s*(\d+)\s*\)", function, re.IGNORECASE
        )
        if variable_match is not None:
            variable = data.source_variables.get(int(variable_match.group(1)))
            if variable is not None:
                return _source_force_constant(
                    variable.function,
                    rf"max\s*\(\s*0\s*,\s*({_ADAMS_NUMBER})\s*-\s*dm\s*\(",
                    variable.adams_name or f"variable {variable.variable_id}",
                )
    raise ValueError(
        f"Adams source {force.adams_name or force.force_id} has no resolvable stop clearance"
    )


def _source_force_pair(
    data: AdamsFullVehicleInput,
    force: AdamsSforceData,
    *,
    allowed_part_ids: set[int],
    chassis_ids: set[int],
) -> tuple[str, str, Vec3, Vec3]:
    aliases = _source_fixed_part_aliases(data)
    marker_i = data.compiled_markers.get(force.marker_i)
    marker_j = data.compiled_markers.get(force.marker_j)
    if marker_i is None or marker_j is None:
        raise ValueError(f"Adams SFORCE {force.force_id} has an unknown marker")
    resolved_i = aliases.get(marker_i.part_id, marker_i.part_id)
    resolved_j = aliases.get(marker_j.part_id, marker_j.part_id)
    allowed = allowed_part_ids | chassis_ids
    if resolved_i not in allowed or resolved_j not in allowed:
        raise ValueError(f"Adams SFORCE {force.force_id} is outside its axle scope")
    _, point_i, _ = _source_marker_pose(data, force.marker_i)
    _, point_j, _ = _source_marker_pose(data, force.marker_j)
    return (
        _source_body_name(data, resolved_i, chassis_ids),
        _source_body_name(data, resolved_j, chassis_ids),
        _vec(tuple(float(value) for value in point_i)),
        _vec(tuple(float(value) for value in point_j)),
    )


def _source_force_elements(
    data: AdamsFullVehicleInput,
    *,
    rear: bool,
    allowed_part_ids: set[int],
    chassis_ids: set[int],
) -> tuple[tuple[LinearSpring, ...], tuple[StaticDamper, ...], tuple[BumpStop, ...]]:
    """Translate source suspension SFORCE elements and their measured curves."""
    prefix = "tr_rear_suspension." if rear else "tr_front_suspension."
    springs: list[LinearSpring] = []
    dampers: list[StaticDamper] = []
    stops: list[BumpStop] = []
    for force in data.source_forces:
        source_name = (force.adams_name or "").lower()
        if not source_name.startswith(prefix):
            continue
        if force.kind != "TRANSLATIONAL":
            continue
        body_a, body_b, point_a, point_b = _source_force_pair(
            data,
            force,
            allowed_part_ids=allowed_part_ids,
            chassis_ids=chassis_ids,
        )
        if ".nsl_ride_spring.force" in source_name or ".nsr_ride_spring.force" in source_name:
            free_length = _source_force_constant(
                force.function,
                rf"akispl\s*\(\s*({_ADAMS_NUMBER})\s*-\s*dm\s*\(",
                source_name,
            )
            extension_curve = tuple(
                (-coordinate, -value)
                for coordinate, value in reversed(data.spring_curve)
            )
            springs.append(
                LinearSpring(
                    name=f"adams_sforce_{force.force_id}",
                    body_a=body_a,
                    body_b=body_b,
                    point_a=point_a,
                    point_b=point_b,
                    stiffness=max(abs(_curve_slope(data.spring_curve, 0.0)), 1e-9),
                    free_length=free_length,
                    force_curve=extension_curve,
                )
            )
        elif ".dal_ride_damper.force" in source_name or ".dar_ride_damper.force" in source_name:
            dampers.append(
                StaticDamper(
                    name=f"adams_sforce_{force.force_id}",
                    body_a=body_a,
                    body_b=body_b,
                    point_a=point_a,
                    point_b=point_b,
                    force_curve=data.damper_curve,
                )
            )
        elif ".bul_jounce_stop.force" in source_name or ".bur_jounce_stop.force" in source_name:
            clearance = _source_stop_clearance(data, force)
            stops.append(
                BumpStop(
                    name=f"adams_sforce_{force.force_id}",
                    body_a=body_a,
                    body_b=body_b,
                    point_a=point_a,
                    point_b=point_b,
                    clearance=clearance,
                    stiffness=max(abs(_curve_slope(data.bumpstop_curve, 0.0)), 1e-9),
                    force_curve=data.bumpstop_curve,
                )
            )
    if len(springs) != 2 or len(dampers) != 2 or len(stops) != 2:
        raise ValueError(
            f"Adams {'rear' if rear else 'front'} suspension source force mapping is incomplete: "
            f"springs={len(springs)}, dampers={len(dampers)}, stops={len(stops)}"
        )
    return tuple(springs), tuple(dampers), tuple(stops)


def _source_axle_model(
    data: AdamsFullVehicleInput,
    *,
    rear: bool,
    include_drivetrain: bool = False,
) -> FrontAxleModel:
    chassis_ids = set(_source_chassis_part_ids(data))
    if not chassis_ids:
        raise ValueError("compiled Adams model has no chassis part")
    part_ids = set(
        _source_axle_part_ids(
            data, rear=rear, include_drivetrain=include_drivetrain
        )
    )
    if not part_ids:
        raise ValueError(f"compiled Adams model has no {'rear' if rear else 'front'} source parts")
    points = {key.upper(): _vec(value) for key, value in (
        data.rear_hardpoints.items() if rear else data.front_hardpoints.items()
    )}
    if not rear:
        rack_ids = _role_part_ids(data.part_roles, "rack")
        if len(rack_ids) != 1:
            raise ValueError("compiled Adams model must have one steering rack part")
        rack_origin, _ = _source_part_frame(data, rack_ids[0])
        points["RACK_CENTER"] = _vec(tuple(float(value) for value in rack_origin))
    for name, point in tuple(points.items()):
        if not name.endswith("__R"):
            points[f"{name}__R"] = point.mirrored_y()
    reference_frames = _source_translational_reference_frames(
        data,
        allowed_part_ids=part_ids,
        chassis_ids=chassis_ids,
    )
    static_rotation_axes = _source_spindle_static_rotation_axes(
        data,
        allowed_part_ids=part_ids,
    )
    body_specs = tuple(
        _source_body_spec(
            data,
            part_id,
            chassis_ids,
            reference_frames.get(part_id),
            static_rotation_axes.get(part_id),
        )
        for part_id in sorted(part_ids)
    )
    springs, dampers, stops = _source_force_elements(
        data,
        rear=rear,
        allowed_part_ids=part_ids,
        chassis_ids=chassis_ids,
    )
    joints = _source_joint_specs(
        data,
        allowed_part_ids=part_ids,
        chassis_ids=chassis_ids,
    )
    bushings = _source_bushing_specs(
        data,
        allowed_part_ids=part_ids,
        chassis_ids=chassis_ids,
    )
    return FrontAxleModel(
        name=("rear" if rear else "front") + "_adams_compiled_explicit",
        hardpoints=points,
        mass=MassSpec(sprung_mass=max(sum(spec.mass for spec in body_specs), 1e-9)),
        bodies=body_specs,
        springs=springs,
        dampers=dampers,
        stops=stops,
        bushings=bushings,
        topology="explicit",
        joints=joints,
        rack_fixed_to_chassis=rear,
    )


def _source_chassis_spec(
    data: AdamsFullVehicleInput, chassis_ids: set[int]
) -> RigidBodySpec:
    if len(chassis_ids) != 1:
        raise ValueError(
            "native source vehicle requires exactly one positive-mass chassis part"
        )
    chassis_id = next(iter(chassis_ids))
    represented = _source_native_body_part_ids(data, include_drivetrain=True)
    omitted = tuple(
        sorted(
            part_id
            for part_id, part in data.compiled_parts.items()
            if part.mass > 0.0 and part_id not in represented
        )
    )
    return _source_composite_body_spec(data, chassis_id, chassis_ids, omitted)


def _source_aerodynamic_drag_spec(
    data: AdamsFullVehicleInput,
) -> AerodynamicDragSpec | None:
    """Extract the source quadratic body drag law without fitting its output."""
    text = data.adm_path.read_text(encoding="ascii", errors="replace")
    for _, block, adams_name in _adams_entity_blocks(text, "GFORCE"):
        if (adams_name or "").lower() != "tr_body.aero_forces":
            continue
        marker_match = re.search(r",\s*I\s*=\s*(\d+)", block, re.IGNORECASE)
        force_match = re.search(
            rf"FZ\s*=\s*0\.5\s*\*\s*({_ADAMS_NUMBER})\s*\*\s*"
            rf"({_ADAMS_NUMBER})\s*\*\s*({_ADAMS_NUMBER})\s*/\s*"
            rf"({_ADAMS_NUMBER})",
            block,
            re.IGNORECASE | re.DOTALL,
        )
        if marker_match is None or force_match is None:
            raise ValueError("Adams body aerodynamic force law is unsupported")
        denominator = _adams_float(force_match.group(4))
        if not math.isclose(denominator, 1000.0, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("Adams body aerodynamic force uses an unknown unit scale")
        marker = data.compiled_markers.get(int(marker_match.group(1)))
        chassis_ids = set(_source_chassis_part_ids(data))
        if marker is None or marker.part_id not in chassis_ids:
            raise ValueError("Adams body aerodynamic marker is not on the chassis")
        axis = np.asarray(marker.local_orientation, dtype=float)[:, 2]
        axis /= np.linalg.norm(axis)
        return AerodynamicDragSpec(
            air_density=_adams_float(force_match.group(1)) * 1.0e9,
            drag_coefficient=_adams_float(force_match.group(2)),
            frontal_area=_adams_float(force_match.group(3)) * 1.0e-6,
            application_point=_vec(marker.local_position),
            forward_axis=_vec(tuple(float(value) for value in axis)),
        )
    return None


def _source_composite_body_spec(
    data: AdamsFullVehicleInput,
    chassis_id: int,
    chassis_ids: set[int],
    extra_part_ids: tuple[int, ...],
) -> RigidBodySpec:
    """将被简化掉的源部件质量精确凝聚到车身参考系."""
    origin, rotation = _source_part_frame(data, chassis_id)
    selected_ids = (chassis_id, *extra_part_ids)
    selected = [
        data.compiled_parts[part_id]
        for part_id in selected_ids
        if part_id in data.compiled_parts and data.compiled_parts[part_id].mass > 0.0
    ]
    if not selected:
        raise ValueError("compiled Adams model has no positive-mass chassis parts")
    mass = sum(part.mass for part in selected)
    centers_global = [np.asarray(part.center_of_mass, dtype=float) for part in selected]
    center_global = sum(
        (part.mass * center for part, center in zip(selected, centers_global)),
        np.zeros(3),
    ) / mass
    inertia_global = np.zeros((3, 3), dtype=float)
    identity = np.eye(3)
    for part, center in zip(selected, centers_global):
        delta = center - center_global
        inertia_global += part.inertia_about_com_global()
        inertia_global += part.mass * (
            float(delta @ delta) * identity - np.outer(delta, delta)
        )
    center_local = rotation.T @ (center_global - origin)
    inertia_local = rotation.T @ inertia_global @ rotation
    return RigidBodySpec(
        name="chassis",
        pose=Pose(
            translation=_vec(tuple(float(value) for value in origin)),
            rotation=_quaternion_from_rotation(rotation),
        ),
        mass=float(mass),
        center_of_mass=_vec(tuple(float(value) for value in center_local)),
        inertia=_matrix_tuple(inertia_local),
        fixed=False,
    )


def _source_drive_torque_mapping(
    data: AdamsFullVehicleInput,
    wheel_name: Literal["front_left", "front_right", "rear_left", "rear_right"],
    spin_axis_world: np.ndarray,
) -> tuple[str | None, str | None, Vec3 | None]:
    """Map the source differential-output SFORCE to its native body pair."""
    if wheel_name.startswith("front_"):
        return None, None, None
    force_token = (
        ".jfl_output_torque."
        if wheel_name.endswith("left")
        else ".jfr_output_torque."
    )
    candidates = tuple(
        force
        for force in data.source_forces
        if force.kind == "ROTATIONAL"
        and force_token in (force.adams_name or "").lower()
    )
    if len(candidates) != 1:
        raise ValueError(
            f"Adams source wheel {wheel_name!r} has no unique drive torque SFORCE"
        )
    force = candidates[0]
    marker_i = data.compiled_markers.get(force.marker_i)
    marker_j = data.compiled_markers.get(force.marker_j)
    if marker_i is None or marker_j is None:
        raise ValueError(
            f"Adams source drive torque {force.force_id} has incomplete markers"
        )
    source_rotation = _source_part_frame(data, marker_i.part_id)[1]
    marker_axis_world = source_rotation @ np.asarray(
        marker_i.local_orientation, dtype=float
    )[:, 2]
    force_sign = -1.0 if (force.function or "").lstrip().startswith("-") else 1.0
    physical_axis_world = force_sign * marker_axis_world
    physical_axis_world /= np.linalg.norm(physical_axis_world)
    if float(physical_axis_world @ spin_axis_world) < 0.99:
        raise ValueError(
            f"Adams source drive torque {force.force_id} opposes the normalized wheel axis"
        )

    chassis_ids = set(_source_chassis_part_ids(data))
    allowed_part_ids = set(
        _source_axle_part_ids(data, rear=True, include_drivetrain=True)
    )
    if marker_i.part_id not in allowed_part_ids or marker_j.part_id not in allowed_part_ids:
        raise ValueError(
            f"Adams source drive torque {force.force_id} references an omitted drivetrain body"
        )
    reference_frames = _source_translational_reference_frames(
        data,
        allowed_part_ids=allowed_part_ids,
        chassis_ids=chassis_ids,
    )
    drive_rotation = reference_frames.get(marker_i.part_id, source_rotation)
    axis_local = drive_rotation.T @ physical_axis_world
    axis_local /= np.linalg.norm(axis_local)
    return (
        _source_body_name(data, marker_i.part_id, chassis_ids),
        _source_body_name(data, marker_j.part_id, chassis_ids),
        _vec(tuple(float(value) for value in axis_local)),
    )


def _source_wheel_spec(
    data: AdamsFullVehicleInput,
    wheel_name: Literal["front_left", "front_right", "rear_left", "rear_right"],
    wheel_role: str,
    spindle_role: str,
    tire: TireModelSpec,
) -> WheelSpec:
    wheel_ids = _role_part_ids(data.part_roles, wheel_role)
    spindle_ids = _role_part_ids(data.part_roles, spindle_role)
    if len(wheel_ids) != 1 or len(spindle_ids) not in {1, 2}:
        raise ValueError(f"Adams source wheel {wheel_name!r} has ambiguous source parts")
    wheel_part = data.compiled_parts.get(wheel_ids[0])
    side_token = ".gel_" if wheel_name.endswith("left") else ".ger_"
    spindle_candidates = [
        part
        for part_id in spindle_ids
        if (part := data.compiled_parts.get(part_id)) is not None
        and side_token in (part.adams_name or "").lower()
    ]
    if len(spindle_ids) == 1 and not wheel_name.endswith("left"):
        source_prefix = "tr_rear_suspension." if wheel_name.startswith("rear_") else "tr_front_suspension."
        spindle_candidates = [
            part
            for part in data.compiled_parts.values()
            if part.mass > 0.0
            and (part.adams_name or "").lower().startswith(source_prefix)
            and ".ger_spindle" in (part.adams_name or "").lower()
        ]
    if len(spindle_candidates) != 1:
        raise ValueError(f"Adams source wheel {wheel_name!r} has no unique side spindle")
    spindle_part = spindle_candidates[0]
    if wheel_part is None or spindle_part is None or wheel_part.mass <= 0.0:
        raise ValueError(f"Adams source wheel {wheel_name!r} has incomplete mass data")
    origin, rotation = _source_part_frame(data, wheel_part.part_id)
    spindle_axes = _source_spindle_static_rotation_axes(
        data,
        allowed_part_ids=set(
            _source_axle_part_ids(data, rear=wheel_name.startswith("rear_"))
        ),
    )
    try:
        spin_axis_world = np.asarray(spindle_axes[spindle_part.part_id], dtype=float)
    except KeyError as exc:
        raise ValueError(
            f"Adams source wheel {wheel_name!r} has no spindle revolute axis"
        ) from exc
    spin_axis_world /= np.linalg.norm(spin_axis_world)
    forward_world = np.array(
        [float(data.initial_velocity_sign), 0.0, 0.0], dtype=float
    )
    forward_world -= spin_axis_world * float(forward_world @ spin_axis_world)
    forward_norm = float(np.linalg.norm(forward_world))
    if forward_norm <= 1e-12:
        raise ValueError(f"Adams source wheel {wheel_name!r} has no rolling direction")
    forward_world /= forward_norm
    # Adams 的源轴正负号是关节局部约定；导入时按源工况的实际行驶方向
    # 统一正滚动轴，使轮速、驱动力矩和 PAC2002 滚阻力矩使用同一坐标约定。
    if float(np.cross(spin_axis_world, np.array([0.0, 0.0, -1.0])) @ forward_world) > 0.0:
        spin_axis_world *= -1.0
    spin_axis_local = rotation.T @ spin_axis_world
    spin_axis_local /= np.linalg.norm(spin_axis_local)
    forward_axis_local = rotation.T @ forward_world
    forward_axis_local /= np.linalg.norm(forward_axis_local)
    drive_body, drive_reaction_body, drive_axis_local = (
        _source_drive_torque_mapping(data, wheel_name, spin_axis_world)
    )
    ixx, iyy, izz = wheel_part.inertia
    ixy, ixz, iyz = wheel_part.inertia_products
    inertia = _matrix_tuple(
        np.asarray(((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz)), dtype=float)
    )
    chassis_ids = set(_source_chassis_part_ids(data))
    return WheelSpec(
        name=wheel_name,
        body=f"wheel_{wheel_name}",
        center_local=Vec3(),
        steering_axis=Vec3(y=1.0),
        spin_axis=_vec(tuple(float(value) for value in spin_axis_local)),
        forward_axis=_vec(tuple(float(value) for value in forward_axis_local)),
        pose=Pose(
            translation=_vec(tuple(float(value) for value in origin)),
            rotation=_quaternion_from_rotation(rotation),
        ),
        inertia=inertia,
        mount_body=_source_body_name(data, spindle_part.part_id, chassis_ids),
        mount_joint_kind="fixed",
        drive_torque_body=drive_body,
        drive_torque_reaction_body=drive_reaction_body,
        drive_torque_axis_local=drive_axis_local,
        mass=float(wheel_part.mass),
        tire=tire,
        driven=wheel_name.startswith("rear_"),
        braked=True,
    )


def _source_native_body_part_map(
    data: AdamsFullVehicleInput, model: VehicleModel
) -> dict[str, int]:
    """建立 native 运行时刚体名称到 Adams 部件编号的映射."""
    chassis_ids = set(_source_chassis_part_ids(data))
    explicit_source_model = any(
        spec.name.startswith("adams_part_")
        for spec in model.rear_axle.bodies
    )
    result: dict[str, int] = {}
    for part_id in chassis_ids:
        result[model.chassis.name] = part_id
    for rear, axle in ((False, model.front_axle), (True, model.rear_axle)):
        prefix = "rear_" if rear else "front_"
        allowed = _source_axle_part_ids(
            data,
            rear=rear,
            include_drivetrain=explicit_source_model,
        )
        body_names = {spec.name for spec in axle.bodies}
        for part_id in allowed:
            source_name = _source_body_name(data, part_id, chassis_ids)
            if source_name not in body_names:
                continue
            body_name = f"{prefix}{source_name}"
            if body_name in result:
                raise ValueError(f"Adams source body mapping is ambiguous: {body_name!r}")
            result[body_name] = part_id
    return result


def _source_initial_body_states(
    data: AdamsFullVehicleInput, model: VehicleModel
) -> tuple[InitialBodyState, ...]:
    """把 Adams 初始条件变换到 native 每个刚体实际使用的参考系."""
    if not data.initial_part_states:
        return ()
    part_map = _source_native_body_part_map(data, model)
    body_specs: dict[str, RigidBodySpec] = {model.chassis.name: model.chassis}
    for rear, axle in ((False, model.front_axle), (True, model.rear_axle)):
        prefix = "rear_" if rear else "front_"
        body_specs.update({f"{prefix}{spec.name}": spec for spec in axle.bodies})
    if set(part_map) != set(body_specs):
        return ()
    states: list[InitialBodyState] = []
    for body_name, part_id in part_map.items():
        source_state = data.initial_part_states.get(part_id)
        spec = body_specs[body_name]
        source_frame = data.compiled_part_frames.get(part_id)
        if source_state is None or source_frame is None:
            return ()
        source_nominal = np.asarray(source_frame.orientation, dtype=float)
        native_nominal = np.asarray(spec.pose.rotation.as_tuple(), dtype=float)
        native_nominal = _rotation_matrix_from_quaternion(native_nominal)
        # 部分源部件为适配 Adams 平移约束采用了旋转后的 native 参考轴。
        # 当前姿态必须同时变换，否则约束点仍会落在错误的刚体坐标系中。
        current_native = (
            np.asarray(source_state.rotation, dtype=float)
            @ source_nominal.T
            @ native_nominal
        )
        quaternion = _quaternion_from_rotation(current_native)
        states.append(
            InitialBodyState(
                body=body_name,
                pose=Pose(
                    translation=_vec(source_state.translation),
                    rotation=quaternion,
                ),
                velocity=SixVector(
                    fx=source_state.linear_velocity[0],
                    fy=source_state.linear_velocity[1],
                    fz=source_state.linear_velocity[2],
                    mx=source_state.angular_velocity[0],
                    my=source_state.angular_velocity[1],
                    mz=source_state.angular_velocity[2],
                ),
            )
        )
    return tuple(states)


def _source_initial_wheel_speeds(
    data: AdamsFullVehicleInput,
    model: VehicleModel,
    initial_states: tuple[InitialBodyState, ...],
) -> tuple[tuple[str, float], ...]:
    """从源主轴角速度投影得到四个轮端的初始自转速度."""
    source_body_map = _source_native_body_part_map(data, model)
    state_by_body = {state.body: state for state in initial_states}
    from ..model.vehicle import build_vehicle

    assembly = build_vehicle(model, mode="K")
    result: list[tuple[str, float]] = []
    for wheel in model.wheels:
        mount_body = wheel.mount_body
        if mount_body is None:
            return ()
        prefix = "rear_" if wheel.name.startswith("rear_") else "front_"
        native_body = f"{prefix}{mount_body}"
        part_id = source_body_map.get(native_body)
        state = state_by_body.get(native_body)
        if part_id is None or state is None:
            return ()
        source_state = data.initial_part_states[part_id]
        quaternion = np.asarray(state.pose.rotation.as_tuple(), dtype=float)
        body_rotation = _rotation_matrix_from_quaternion(quaternion)
        # 固定车轮的 wheel_to_mount 只描述源轮毂参考系到主轴参考系的姿态。
        wheel_to_body = assembly.wheel_rotations_local[wheel.name]
        spin_local = wheel_to_body @ wheel.spin_axis.as_array()
        spin_local /= np.linalg.norm(spin_local)
        spin_world = body_rotation @ spin_local
        angular_velocity = np.asarray(source_state.angular_velocity, dtype=float)
        result.append((wheel.name, float(angular_velocity @ spin_world)))
    return tuple(result)


def build_adams_source_vehicle_model(
    data: AdamsFullVehicleInput,
    *,
    tire_kind: Literal["pac2002", "native_brush", "fiala"] = "pac2002",
) -> VehicleModel:
    """Build a source-part explicit model while retaining unresolved-law gates."""
    chassis_ids = set(_source_chassis_part_ids(data))
    steering_wheel_joint = next(
        (
            joint
            for joint in data.source_joints
            if joint.joint_id == 110 and joint.kind.upper() == "REVOLUTE"
        ),
        None,
    )
    if steering_wheel_joint is None:
        raise ValueError("Adams source steering wheel revolute joint is missing")
    steering_wheel_marker = data.compiled_markers.get(
        steering_wheel_joint.marker_i
    )
    if steering_wheel_marker is None:
        raise ValueError("Adams source steering wheel marker is missing")
    steering_wheel_body = _source_body_name(
        data, steering_wheel_marker.part_id, chassis_ids
    )
    _, steering_wheel_rotation = _source_part_frame(
        data, steering_wheel_marker.part_id
    )
    _, _, steering_wheel_marker_frame = _source_marker_pose(
        data, steering_wheel_joint.marker_i
    )
    chassis_id = next(iter(chassis_ids))
    chassis_initial = data.initial_part_states.get(chassis_id)
    steering_wheel_initial = data.initial_part_states.get(
        steering_wheel_marker.part_id
    )
    _, chassis_rotation = _source_part_frame(data, chassis_id)
    steering_axis_local = steering_wheel_rotation.T @ steering_wheel_marker_frame[:, 2]
    steering_reference_rotation = (
        np.asarray(chassis_initial.rotation, dtype=float).T
        @ np.asarray(steering_wheel_initial.rotation, dtype=float)
        if chassis_initial is not None and steering_wheel_initial is not None
        else chassis_rotation.T @ steering_wheel_rotation
    )
    tire = _adams_tire_spec(
        data.fiala_parameters if tire_kind == "fiala" else data.pac2002_coefficients,
        kind=tire_kind,
    )
    wheels = tuple(
        _source_wheel_spec(data, wheel_name, wheel_role, spindle_role, tire)
        for wheel_name, wheel_role, spindle_role in (
            ("front_left", "front_wheel_left", "front_spindle"),
            ("front_right", "front_wheel_right", "front_spindle"),
            ("rear_left", "rear_wheel_left", "rear_spindle"),
            ("rear_right", "rear_wheel_right", "rear_spindle"),
        )
    )
    pinion_rack = next(
        (
            coupler
            for coupler in data.source_couplers
            if coupler.kind == "R:T" and len(coupler.scales) == 2
        ),
        None,
    )
    rack_per_pinion = (
        abs(pinion_rack.scales[0] / pinion_rack.scales[1])
        if pinion_rack is not None
        else 1.0
    )
    return VehicleModel(
        name="Demo_Vehicle_Adams_Source_Explicit",
        chassis=_source_chassis_spec(data, chassis_ids),
        front_axle=_source_axle_model(
            data, rear=False, include_drivetrain=True
        ),
        rear_axle=_source_axle_model(
            data, rear=True, include_drivetrain=True
        ),
        wheels=wheels,
        steering=SteeringSystemSpec(
            rack_body="rack",
            ratio=data.steering_ratio,
            input="steering_wheel_angle",
            rack_displacement_per_steering_wheel_angle=rack_per_pinion,
            actuator_mode="prescribed_rotation",
            actuator_body=steering_wheel_body,
            actuator_reaction_body="chassis",
            actuator_axis_local=_vec(
                tuple(float(value) for value in steering_axis_local)
            ),
            actuator_reference_rotation=_quaternion_from_rotation(
                steering_reference_rotation
            ),
        ),
        driveline=DrivelineSpec(
            driven_wheels=("rear_left", "rear_right"),
            maximum_drive_torque=10_000.0,
            maximum_brake_torque=10_000.0,
            front_brake_bias=0.6,
            drive_split=(0.0, 0.0, 0.5, 0.5),
        ),
        coordinate_couplers=_source_coordinate_coupler_specs(data),
        aerodynamic_drag=_source_aerodynamic_drag_spec(data),
    )


def build_adams_native_vehicle_model(
    data: AdamsFullVehicleInput,
    *,
    tire_kind: Literal["pac2002", "native_brush", "fiala"] = "pac2002",
) -> VehicleModel:
    """Build the runnable native model from source parts and source marker frames."""
    return build_adams_source_vehicle_model(data, tire_kind=tire_kind)


def _adams_tire_spec(
    coefficients: Mapping[str, float],
    *,
    kind: Literal["pac2002", "native_brush", "fiala"],
) -> TireModelSpec:
    """Convert shared PAC2002 values to either source or native proxy data."""
    radius = float(coefficients.get("UNLOADED_RADIUS_MM", 344.0))
    nominal_load = max(
        float(coefficients.get("FNOMIN_N", coefficients.get("FNOMIN", 4_850.0))),
        1e-9,
    )
    longitudinal_stiffness = abs(
        float(coefficients.get("PKX1", 22.303)) * nominal_load
    )
    cornering_stiffness = abs(
        float(coefficients.get("PKY1", -21.92)) * nominal_load
    )
    kwargs: dict[str, object] = {
        "kind": kind,
        "parameter_source": "adams_builtin",
        "unloaded_radius": radius,
        "maximum_compression": 0.99 * radius,
        "vertical_stiffness": float(
            coefficients.get("VERTICAL_STIFFNESS_N_MM", 210.0)
        ),
        "vertical_damping": float(
            coefficients.get("VERTICAL_DAMPING_N_S_MM", 0.05)
        ),
        "cornering_stiffness": cornering_stiffness,
        "longitudinal_stiffness": longitudinal_stiffness,
        # The native ABI has one friction coefficient for both brush axes.
        # Use the lower nominal PAC2002 peak and keep this limitation explicit
        # in the source-equivalence manifest.
        "friction_coefficient": min(
            abs(float(coefficients.get("PDX1", 1.0))),
            abs(float(coefficients.get("PDY1", 1.0))),
        ),
        "pneumatic_trail": float(coefficients.get("QDZ1", 0.0935)) * radius,
        "pac2002_coefficients": dict(coefficients),
    }
    if kind == "fiala":
        kwargs["kind"] = "fiala"
        kwargs["fiala_parameters"] = {
            "CSLIP": float(coefficients.get("CSLIP_N", coefficients.get("CSLIP", 1000.0))),
            "CALPHA": float(coefficients.get("CALPHA_N_PER_RAD", coefficients.get("CALPHA", 800.0))),
            "UMIN": float(coefficients.get("UMIN", 0.9)),
            "UMAX": float(coefficients.get("UMAX", 1.0)),
            "RELAX_LENGTH_X": float(coefficients.get("RELAX_LENGTH_X_MM", 50.0)),
            "RELAX_LENGTH_Y": float(coefficients.get("RELAX_LENGTH_Y_MM", 150.0)),
            "WIDTH": float(coefficients.get("WIDTH_MM", 235.0)),
            "ROLLING_RESISTANCE": float(coefficients.get("ROLLING_RESISTANCE", 0.0)),
        }
    kwargs.update(
        {
            # PAC 纯滑移路径不把该量用于轮胎力，但底层仍保留两个
            # 衰减状态，因此需要一个与源参数同量纲的正值。
            "relaxation_length": min(
                abs(float(coefficients.get("PTX1", 2.3657))) * radius,
                abs(float(coefficients.get("PTY1", 2.1439))) * radius,
            ),
            "detached_relaxation_s": 0.05,
        }
    )
    return TireModelSpec(**kwargs)


_SOURCE_FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?"


def _bracket_sections(text: str) -> list[tuple[str, str]]:
    """按 Adams 方括号段落拆分文本，并保留同名段落."""
    sections: list[tuple[str, str]] = []
    current: str | None = None
    lines: list[str] = []
    for line in text.splitlines():
        match = re.match(r"^\s*\[([^]]+)\]\s*$", line)
        if match:
            if current is not None:
                sections.append((current, "\n".join(lines)))
            current = match.group(1).strip().upper()
            lines = []
        elif current is not None:
            lines.append(line)
    if current is not None:
        sections.append((current, "\n".join(lines)))
    return sections


def _source_fields(block: str) -> dict[str, str]:
    """读取 Adams 段落中的键值字段."""
    return {
        key.upper(): value.strip().strip("'").strip()
        for key, value in re.findall(
            r"^\s*([A-Za-z][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$",
            block,
            re.MULTILINE,
        )
    }


def _source_numeric_table(block: str) -> tuple[tuple[float, float], ...]:
    """读取曲线段中的前两列数值."""
    rows: list[tuple[float, float]] = []
    pattern = re.compile(rf"^\s*({_SOURCE_FLOAT})\s+({_SOURCE_FLOAT})")
    for line in block.splitlines():
        match = pattern.match(line)
        if match:
            rows.append((float(match.group(1)), float(match.group(2))))
    return tuple(rows)


def _source_orientation_vector(
    fields: Mapping[str, str], block: str, axis: str
) -> tuple[float, float, float]:
    """读取显式方向字段或 Adams 的 dependent orientation 注释."""
    field_prefix = f"ORIENTATION_{axis}"
    if all(f"{field_prefix}{index}" in fields for index in (1, 2, 3)):
        return tuple(
            float(fields[f"{field_prefix}{index}"]) for index in (1, 2, 3)
        )  # type: ignore[return-value]
    match = re.search(
        rf"\b{axis}\s+vector\s*=\s*([^\r\n]+)", block, re.IGNORECASE
    )
    if match:
        values = tuple(
            float(value)
            for value in re.findall(_SOURCE_FLOAT, match.group(1))[:3]
        )
        if len(values) == 3:
            return values
    defaults = {"ZP": (0.0, 0.0, 1.0), "XP": (1.0, 0.0, 0.0)}
    return defaults[axis]


def _checked_source_curve(
    path: Path,
    section: str,
    rows: tuple[tuple[float, float], ...],
    coordinate_scale: float,
    force_scale: float,
) -> BushingCurve:
    """将一条 `.bus` 曲线归一到工程单位并检查单调性."""
    curve = tuple(
        (coordinate * coordinate_scale, force * force_scale)
        for coordinate, force in rows
    )
    if len(curve) < 2:
        if curve:
            raise ValueError(f"Adams bushing curve has fewer than two samples: {path} [{section}]")
        return ()
    if any(
        not math.isfinite(coordinate) or not math.isfinite(force)
        for coordinate, force in curve
    ):
        raise ValueError(f"Adams bushing curve is not finite: {path} [{section}]")
    if any(
        right[0] <= left[0] for left, right in zip(curve, curve[1:])
    ):
        raise ValueError(f"Adams bushing curve abscissas are not increasing: {path} [{section}]")
    return curve


def _parse_bushing_property(path: Path) -> AdamsBushingProperty:
    """解析 Adams `.bus` 文件的六轴弹性曲线和阻尼."""
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    angle_scale = _unit_factor(units.get("angle"), _ANGLE_TO_RAD, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    time_scale = _unit_factor(units.get("time"), _TIME_TO_S, 1.0)
    sections = {
        name: block for name, block in _bracket_sections(text)
    }
    curves: list[BushingCurve] = []
    for axis in ("X", "Y", "Z"):
        curves.append(
            _checked_source_curve(
                path,
                f"F{axis}_CURVE",
                _source_numeric_table(sections.get(f"F{axis}_CURVE", "")),
                length_scale,
                force_scale,
            )
        )
    for axis in ("X", "Y", "Z"):
        curves.append(
            _checked_source_curve(
                path,
                f"T{axis}_CURVE",
                _source_numeric_table(sections.get(f"T{axis}_CURVE", "")),
                angle_scale,
                force_scale * length_scale,
            )
        )
    damping_fields = _source_fields(sections.get("DAMPING", ""))
    damping = (
        tuple(
            float(damping_fields.get(f"F{axis}_DAMPING", "0"))
            * force_scale
            * time_scale
            / length_scale
            for axis in ("X", "Y", "Z")
        )
        + tuple(
            float(damping_fields.get(f"T{axis}_DAMPING", "0"))
            * force_scale
            * length_scale
            * time_scale
            / angle_scale
            for axis in ("X", "Y", "Z")
        )
    )
    if any(not math.isfinite(value) or value < 0.0 for value in damping):
        raise ValueError(f"Adams bushing damping is invalid: {path}")
    return AdamsBushingProperty(
        name=path.name,
        path=path,
        units=units,
        damping=damping,  # type: ignore[arg-type]
        force_curves=tuple(curves),
    )


def _resolve_bushing_property(reference: str, database: Path) -> Path:
    """解析 `mdids://` 衬套属性引用，找不到时显式失败."""
    cleaned = reference.strip().strip("'").replace("\\", "/")
    if cleaned.lower().startswith("mdids://"):
        tail = cleaned.split("/", 3)[-1]
    elif cleaned.startswith("<") and ">/" in cleaned:
        tail = cleaned.split(">/", 1)[1]
    else:
        tail = cleaned
    candidates = (database / tail, database.parent / tail)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"Adams bushing property file is missing: {reference!r} from {database}"
    )


def _bushing_property_key(path: Path, database: Path) -> str:
    """生成不依赖绝对安装路径的属性键."""
    try:
        return path.relative_to(database).as_posix().lower()
    except ValueError:
        return path.name.lower()


def _parse_bushing_sources(
    path: Path, database: Path
) -> tuple[tuple[AdamsBushingAssembly, ...], dict[str, AdamsBushingProperty]]:
    """读取一个 Adams 子系统中的衬套装配及其属性文件."""
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    assemblies: list[AdamsBushingAssembly] = []
    properties: dict[str, AdamsBushingProperty] = {}
    for section_name, block in _bracket_sections(text):
        if section_name != "BUSHING_ASSEMBLY":
            continue
        fields = _source_fields(block)
        usage = fields.get("USAGE", "").strip()
        reference = fields.get("PROPERTY_FILE", "").strip()
        if not usage or not reference:
            raise ValueError(f"Adams bushing assembly is incomplete: {path}")
        property_path = _resolve_bushing_property(reference, database)
        property_key = _bushing_property_key(property_path, database)
        if property_key not in properties:
            properties[property_key] = _parse_bushing_property(property_path)
        preload = tuple(
            float(fields.get(f"T_PRELOAD_{axis}", "0")) * force_scale
            for axis in ("X", "Y", "Z")
        ) + tuple(
            float(fields.get(f"R_PRELOAD_{axis}", "0"))
            * force_scale
            * length_scale
            for axis in ("X", "Y", "Z")
        )
        force_scaling = tuple(
            float(fields.get(f"{prefix}{axis}_SCALING_FACTOR", "1"))
            for prefix in ("F", "T")
            for axis in ("X", "Y", "Z")
        )
        damping_force_scaling = tuple(
            float(fields.get(f"{prefix}{axis}_DAMPING_FORCE_SCALE", "1"))
            for prefix in ("T", "R")
            for axis in ("X", "Y", "Z")
        )
        values = (*preload, *force_scaling, *damping_force_scaling)
        if any(not math.isfinite(value) for value in values):
            raise ValueError(f"Adams bushing assembly contains non-finite values: {path}")
        assemblies.append(
            AdamsBushingAssembly(
                subsystem_path=path,
                usage=usage,
                symmetry=fields.get("SYMMETRY", "").strip(),
                property_key=property_key,
                property_path=property_path,
                orientation_zp=_source_orientation_vector(fields, block, "ZP"),
                orientation_xp=_source_orientation_vector(fields, block, "XP"),
                preload=preload,  # type: ignore[arg-type]
                force_scaling=force_scaling,  # type: ignore[arg-type]
                damping_force_scaling=damping_force_scaling,  # type: ignore[arg-type]
            )
        )
    return tuple(assemblies), properties


def _parse_subsystem(path: Path) -> tuple[dict[str, tuple[float, float, float]], dict[str, float]]:
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    mass_scale = _unit_factor(units.get("mass"), _MASS_TO_KG, 1.0)
    hardpoints: dict[str, tuple[float, float, float]] = {}
    for line in text.splitlines():
        match = re.match(r"\s*'([^']+)'\s+'[^']+'\s+([-+0-9.Ee]+)\s+([-+0-9.Ee]+)\s+([-+0-9.Ee]+)", line)
        if match:
            hardpoints[match.group(1).strip().upper()] = tuple(
                float(match.group(index)) * length_scale
                for index in (2, 3, 4)
            )
    parts: dict[str, float] = {}
    usage: str | None = None
    for line in text.splitlines():
        usage_match = re.search(r"USAGE\s*=\s*'([^']+)'", line)
        if usage_match:
            usage = usage_match.group(1).strip()
        mass_match = re.search(r"MASS\s*=\s*([-+0-9.Ee]+)", line)
        if usage and mass_match:
            parts[usage] = float(mass_match.group(1)) * mass_scale
            usage = None
    return hardpoints, parts


_ADAMS_ENTITY_START_RE = re.compile(
    r"(?m)^([A-Z][A-Z0-9_]*)/(\d+)\s*$"
)


def _adams_float(value: str) -> float:
    """Parse an Adams number, including the Fortran ``D`` exponent marker."""
    return float(value.replace("D", "E").replace("d", "e"))


def _adams_entity_blocks(
    text: str, entity_kind: str
) -> list[tuple[int, str, str | None]]:
    """Return numbered entity blocks and their nearest source view names."""
    starts = list(_ADAMS_ENTITY_START_RE.finditer(text))
    kind = entity_kind.upper()
    blocks: list[tuple[int, str, str | None]] = []
    for index, match in enumerate(starts):
        if match.group(1).upper() != kind:
            continue
        next_start = (
            starts[index + 1].start() if index + 1 < len(starts) else len(text)
        )
        previous_end = starts[index - 1].end() if index else 0
        prefix = text[previous_end : match.start()]
        names = re.findall(
            r"adams_view_name\s*=\s*'([^']+)'", prefix, re.IGNORECASE
        )
        blocks.append(
            (
                int(match.group(2)),
                text[match.end() : next_start],
                names[-1].strip() if names else None,
            )
        )
    return blocks


def _parse_adm_part_frames(path: Path) -> dict[int, AdamsPartFrameData]:
    """Parse reference frames for every compiled part, including massless parts."""
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    starts = list(re.finditer(r"(?m)^PART/(\d+)\s*$", text))
    result: dict[int, AdamsPartFrameData] = {}
    for index, match in enumerate(starts):
        part_id = int(match.group(1))
        part_prefix = text[starts[index - 1].start() if index else 0 : match.start()]
        name_matches = re.findall(
            r"adams_view_name\s*=\s*'([^']+)'", part_prefix, re.IGNORECASE
        )
        block = text[
            match.end() : starts[index + 1].start()
            if index + 1 < len(starts)
            else len(text)
        ]
        part_header = block.split("MARKER/", 1)[0]
        qg_match = re.search(
            rf",\s*QG\s*=\s*({_ADAMS_NUMBER}),\s*"
            rf"({_ADAMS_NUMBER}),\s*({_ADAMS_NUMBER})",
            part_header,
        )
        qg = (
            tuple(
                _adams_float(qg_match.group(item)) * length_scale
                for item in (1, 2, 3)
            )
            if qg_match
            else (0.0, 0.0, 0.0)
        )
        reuler_match = re.search(
            rf",\s*REULER\s*=\s*({_ADAMS_NUMBER})D?,\s*"
            rf"({_ADAMS_NUMBER})D?,\s*({_ADAMS_NUMBER})D?",
            part_header,
        )
        angles = (
            tuple(
                _adams_float(reuler_match.group(item))
                for item in (1, 2, 3)
            )
            if reuler_match
            else (0.0, 0.0, 0.0)
        )
        result[part_id] = AdamsPartFrameData(
            part_id=part_id,
            orientation=_matrix_tuple(_adams_reuler_matrix(angles)),
            reference_origin=qg,  # type: ignore[arg-type]
            adams_name=name_matches[-1].strip() if name_matches else None,
        )
    return result


def _parse_adm_markers(path: Path) -> dict[int, AdamsMarkerData]:
    """Parse compiled marker poses without converting them to global poses."""
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    result: dict[int, AdamsMarkerData] = {}
    for marker_id, block, adams_name in _adams_entity_blocks(text, "MARKER"):
        part_match = re.search(r",\s*PART\s*=\s*(\d+)", block, re.IGNORECASE)
        if part_match is None:
            raise ValueError(f"Adams marker {marker_id} has no owning PART: {path}")
        position_match = re.search(
            rf",\s*QP\s*=\s*({_ADAMS_NUMBER})\s*,\s*"
            rf"({_ADAMS_NUMBER})\s*,\s*({_ADAMS_NUMBER})",
            block,
            re.IGNORECASE,
        )
        position = (
            tuple(
                _adams_float(position_match.group(index)) * length_scale
                for index in (1, 2, 3)
            )
            if position_match
            else (0.0, 0.0, 0.0)
        )
        reuler_match = re.search(
            rf",\s*REULER\s*=\s*({_ADAMS_NUMBER})\s*D?\s*,\s*"
            rf"({_ADAMS_NUMBER})\s*D?\s*,\s*({_ADAMS_NUMBER})\s*D?",
            block,
            re.IGNORECASE,
        )
        angles = (
            tuple(
                _adams_float(reuler_match.group(index))
                for index in (1, 2, 3)
            )
            if reuler_match
            else (0.0, 0.0, 0.0)
        )
        result[marker_id] = AdamsMarkerData(
            marker_id=marker_id,
            part_id=int(part_match.group(1)),
            local_position=position,
            local_orientation=_matrix_tuple(_adams_reuler_matrix(angles)),
            adams_name=adams_name,
        )
    return result


def _parse_adm_joints(path: Path) -> tuple[AdamsJointData, ...]:
    """Parse compiled Adams ideal joints and retain unsupported kinds explicitly."""
    text = path.read_text(encoding="ascii", errors="replace")
    joints: list[AdamsJointData] = []
    for joint_id, block, adams_name in _adams_entity_blocks(text, "JOINT"):
        kind_match = re.search(
            r"(?m)^\s*,\s*([A-Z][A-Z0-9_]*)\s*$", block
        )
        marker_i_match = re.search(r",\s*I\s*=\s*(\d+)", block, re.IGNORECASE)
        marker_j_match = re.search(r",\s*J\s*=\s*(\d+)", block, re.IGNORECASE)
        if kind_match is None or marker_i_match is None or marker_j_match is None:
            raise ValueError(f"Adams joint {joint_id} is incomplete: {path}")
        joints.append(
            AdamsJointData(
                joint_id=joint_id,
                kind=kind_match.group(1).upper(),
                marker_i=int(marker_i_match.group(1)),
                marker_j=int(marker_j_match.group(1)),
                adams_name=adams_name,
            )
        )
    return tuple(sorted(joints, key=lambda joint: joint.joint_id))


def _parse_adm_couplers(path: Path) -> tuple[AdamsCouplerData, ...]:
    """解析 Adams 关节耦合器，保留类型、关节编号和比例."""
    text = path.read_text(encoding="ascii", errors="replace")
    couplers: list[AdamsCouplerData] = []
    for coupler_id, block, adams_name in _adams_entity_blocks(text, "COUPLER"):
        joints_match = re.search(
            r",\s*JOINTS\s*=\s*([^\r\n]+)", block, re.IGNORECASE
        )
        kind_match = re.search(
            r",\s*TYPE\s*=\s*([A-Z]:[A-Z])", block, re.IGNORECASE
        )
        scales_match = re.search(
            r",\s*SCALES\s*=\s*([^\r\n]+)", block, re.IGNORECASE
        )
        if joints_match is None or kind_match is None or scales_match is None:
            raise ValueError(f"Adams coupler {coupler_id} is incomplete: {path}")
        joint_ids = tuple(
            int(value) for value in re.findall(r"\d+", joints_match.group(1))
        )
        scales = tuple(
            _adams_float(value)
            for value in re.findall(_ADAMS_NUMBER, scales_match.group(1))
        )
        if len(joint_ids) < 2 or len(scales) != len(joint_ids):
            raise ValueError(
                f"Adams coupler {coupler_id} has inconsistent joints/scales: {path}"
            )
        couplers.append(
            AdamsCouplerData(
                coupler_id=coupler_id,
                joint_ids=joint_ids,
                kind=kind_match.group(1).upper(),
                scales=scales,
                adams_name=adams_name,
            )
        )
    return tuple(sorted(couplers, key=lambda item: item.coupler_id))


def _parse_adm_zero_translational_motions(path: Path) -> tuple[int, ...]:
    """Parse source zero-displacement MOTION constraints on translational joints."""
    text = path.read_text(encoding="ascii", errors="replace")
    joint_ids: list[int] = []
    for motion_id, block, _ in _adams_entity_blocks(text, "MOTION"):
        kind_match = re.search(
            r"(?m)^\s*,\s*(TRANSLATIONAL|ROTATIONAL)\s*$", block, re.IGNORECASE
        )
        if kind_match is None or kind_match.group(1).upper() != "TRANSLATIONAL":
            continue
        joint_match = re.search(r",\s*JOINT\s*=\s*(\d+)", block, re.IGNORECASE)
        function_match = re.search(
            r"FUNCTION\s*=\s*([^,\r\n]+)", block, re.IGNORECASE
        )
        if joint_match is None or function_match is None:
            raise ValueError(f"Adams translational MOTION {motion_id} is incomplete: {path}")
        function = function_match.group(1).strip().rstrip("\\").strip()
        try:
            value = _adams_float(function)
        except ValueError as exc:
            raise ValueError(
                "native source importer only supports constant-zero translational "
                f"MOTION {motion_id}, got {function!r}: {path}"
            ) from exc
        if not np.isclose(value, 0.0, rtol=0.0, atol=1e-14):
            raise ValueError(
                "native source importer only supports zero translational MOTION "
                f"{motion_id}, got {value}: {path}"
            )
        joint_ids.append(int(joint_match.group(1)))
    return tuple(sorted(set(joint_ids)))


def _parse_adm_fields(path: Path) -> tuple[AdamsFieldData, ...]:
    """Parse compiled Adams field force marker pairs and source USER laws."""
    text = path.read_text(encoding="ascii", errors="replace")
    fields: list[AdamsFieldData] = []
    for field_id, block, adams_name in _adams_entity_blocks(text, "FIELD"):
        marker_i_match = re.search(r",\s*I\s*=\s*(\d+)", block, re.IGNORECASE)
        marker_j_match = re.search(r",\s*J\s*=\s*(\d+)", block, re.IGNORECASE)
        if marker_i_match is None or marker_j_match is None:
            raise ValueError(f"Adams field {field_id} is incomplete: {path}")
        formulation_match = re.search(
            r",\s*FORMULATION\s*=\s*([A-Z][A-Z0-9_]*)", block, re.IGNORECASE
        )
        function_match = re.search(
            r"FUNCTION\s*=\s*(USER\([^)]*\)|[^,\r\n]+)",
            block,
            re.IGNORECASE | re.DOTALL,
        )
        routine_match = re.search(
            r"ROUTINE\s*=\s*([^,\r\n\\]+)", block, re.IGNORECASE
        )
        fields.append(
            AdamsFieldData(
                field_id=field_id,
                marker_i=int(marker_i_match.group(1)),
                marker_j=int(marker_j_match.group(1)),
                formulation=(
                    formulation_match.group(1).upper()
                    if formulation_match
                    else None
                ),
                function=(
                    function_match.group(1).strip().rstrip("\\").strip()
                    if function_match
                    else None
                ),
                routine=(routine_match.group(1).strip() if routine_match else None),
                adams_name=adams_name,
            )
        )
    return tuple(sorted(fields, key=lambda item: item.field_id))


def _parse_adm_sforces(path: Path) -> tuple[AdamsSforceData, ...]:
    """Parse compiled scalar force marker pairs used by suspension elements."""
    text = path.read_text(encoding="ascii", errors="replace")
    forces: list[AdamsSforceData] = []
    for force_id, block, adams_name in _adams_entity_blocks(text, "SFORCE"):
        kind_match = re.search(
            r"(?m)^\s*,\s*(TRANSLATIONAL|ROTATIONAL)\s*$", block, re.IGNORECASE
        )
        marker_i_match = re.search(r",\s*I\s*=\s*(\d+)", block, re.IGNORECASE)
        marker_j_match = re.search(r",\s*J\s*=\s*(\d+)", block, re.IGNORECASE)
        if kind_match is None or marker_i_match is None or marker_j_match is None:
            raise ValueError(f"Adams SFORCE {force_id} is incomplete: {path}")
        function_match = re.search(
            r"FUNCTION\s*=\s*([^\r\n]+)", block, re.IGNORECASE
        )
        forces.append(
            AdamsSforceData(
                force_id=force_id,
                kind=kind_match.group(1).upper(),
                marker_i=int(marker_i_match.group(1)),
                marker_j=int(marker_j_match.group(1)),
                function=(
                    function_match.group(1).strip().rstrip("\\").strip()
                    if function_match
                    else None
                ),
                adams_name=adams_name,
            )
        )
    return tuple(sorted(forces, key=lambda item: item.force_id))


def _parse_adm_variables(path: Path) -> dict[int, AdamsVariableData]:
    """Parse compiled Adams scalar variables referenced by source force laws."""
    text = path.read_text(encoding="ascii", errors="replace")
    variables: dict[int, AdamsVariableData] = {}
    for variable_id, block, adams_name in _adams_entity_blocks(text, "VARIABLE"):
        function_match = re.search(
            r"FUNCTION\s*=\s*([^\r\n]+)", block, re.IGNORECASE
        )
        variables[variable_id] = AdamsVariableData(
            variable_id=variable_id,
            function=(
                function_match.group(1).strip().rstrip("\\").strip()
                if function_match
                else None
            ),
            adams_name=adams_name,
        )
    return variables


def _parse_adm_user_functions(path: Path) -> tuple[AdamsUserFunctionData, ...]:
    """按 Adams 实体记录 USER() 调用，区分求解实体和纯输出 REQUEST."""
    text = path.read_text(encoding="ascii", errors="replace")
    starts = list(
        re.finditer(r"(?m)^([A-Z][A-Z0-9_]*)/(\d+)\s*$", text)
    )
    result: list[AdamsUserFunctionData] = []
    for index, match in enumerate(starts):
        block_end = starts[index + 1].start() if index + 1 < len(starts) else len(text)
        block = text[match.end() : block_end]
        name_matches = re.findall(
            r"adams_view_name\s*=\s*'([^']+)'",
            text[starts[index - 1].start() if index else 0 : match.start()],
            re.IGNORECASE,
        )
        routine_match = re.search(
            r"ROUTINE\s*=\s*([^,\r\n\\]+)", block, re.IGNORECASE
        )
        for function_match in re.finditer(
            r"FUNCTION\s*=\s*(USER\([^)]*\))",
            block,
            re.IGNORECASE | re.DOTALL,
        ):
            function = re.sub(
                r"\s*\\?\s*[\r\n]+\s*,?\s*", " ", function_match.group(1)
            )
            result.append(
                AdamsUserFunctionData(
                    entity_type=match.group(1).upper(),
                    entity_id=int(match.group(2)),
                    function=function.strip(),
                    routine=(routine_match.group(1).strip() if routine_match else None),
                    adams_name=name_matches[-1].strip() if name_matches else None,
                )
            )
    return tuple(result)


def _parse_adm_parts(path: Path) -> dict[int, AdamsPartData]:
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    mass_scale = _unit_factor(units.get("mass"), _MASS_TO_KG, 1.0)
    inertia_scale = mass_scale * length_scale * length_scale
    markers = _parse_adm_markers(path)
    starts = list(re.finditer(r"(?m)^PART/(\d+)\s*$", text))
    result: dict[int, AdamsPartData] = {}
    for index, match in enumerate(starts):
        part_id = int(match.group(1))
        part_prefix = text[starts[index - 1].start() if index else 0 : match.start()]
        name_matches = re.findall(
            r"adams_view_name\s*=\s*'([^']+)'", part_prefix, re.IGNORECASE
        )
        adams_name = name_matches[-1].strip() if name_matches else None
        block = text[match.end() : starts[index + 1].start() if index + 1 < len(starts) else len(text)]
        part_header = block.split("MARKER/", 1)[0]
        mass_match = re.search(rf", MASS\s*=\s*({_ADAMS_NUMBER})", block)
        ip_match = re.search(
            rf", IP\s*=\s*({_ADAMS_NUMBER}(?:\s*,\s*{_ADAMS_NUMBER})*)",
            block,
        )
        cm_match = re.search(r", CM\s*=\s*(\d+)", block)
        qg_match = re.search(
            rf", QG\s*=\s*({_ADAMS_NUMBER}),\s*({_ADAMS_NUMBER}),\s*({_ADAMS_NUMBER})",
            block,
        )
        reuler_match = re.search(
            rf", REULER\s*=\s*({_ADAMS_NUMBER})D?,\s*"
            rf"({_ADAMS_NUMBER})D?,\s*({_ADAMS_NUMBER})D?",
            part_header,
        )
        if not mass_match:
            continue
        qg = (
            np.asarray(
                tuple(_adams_float(qg_match.group(index)) for index in (1, 2, 3)),
                dtype=float,
            )
            if qg_match
            else np.zeros(3)
        )
        qg *= length_scale
        center = qg.copy()
        if cm_match:
            marker = markers.get(int(cm_match.group(1)))
            if marker is not None and marker.part_id == part_id:
                angles = (
                    tuple(float(reuler_match.group(index)) for index in (1, 2, 3))
                    if reuler_match
                    else (0.0, 0.0, 0.0)
                )
                center += _adams_reuler_matrix(angles) @ np.asarray(
                    marker.local_position, dtype=float
                )
        center_tuple = tuple(float(value) for value in center)
        ip_values = (
            tuple(_adams_float(value) for value in re.findall(_ADAMS_NUMBER, ip_match.group(1)))
            if ip_match
            else (1.0, 1.0, 1.0)
        )
        inertia = tuple(value * inertia_scale for value in ip_values[:3])  # type: ignore[assignment]
        products = (
            tuple(value * inertia_scale for value in ip_values[3:6])
            if len(ip_values) == 6
            else (0.0, 0.0, 0.0)
        )
        angles = (
            tuple(float(reuler_match.group(index)) for index in (1, 2, 3))
            if reuler_match
            else (0.0, 0.0, 0.0)
        )
        result[part_id] = AdamsPartData(
            part_id,
            _adams_float(mass_match.group(1)) * mass_scale,
            center_tuple,
            inertia,
            products,
            _matrix_tuple(_adams_reuler_matrix(angles)),
            adams_name,
            tuple(float(value) for value in qg),
        )
    return result


def _semantic_part_roles(parts: Mapping[int, AdamsPartData]) -> dict[str, tuple[int, ...]]:
    """Map compiled parts from stable Adams view names to runtime roles."""
    suffixes = {
        "front_lower_arm": "tr_front_suspension.gel_lower_control_arm",
        "front_lower_arm2": "tr_front_suspension.gel_lower_control_arm2",
        "front_tie_rod_inner": "tr_front_suspension.gel_tierod_inner",
        "front_tie_rod_outer": "tr_front_suspension.gel_tierod_outer",
        "front_upright": "tr_front_suspension.gel_upright",
        "front_spindle": "tr_front_suspension.gel_spindle",
        "front_upper_arm": "tr_front_suspension.gel_upper_control_arm",
        "rear_lower_arm": "tr_rear_suspension.gel_lower_control_arm",
        "rear_lower_arm2": "tr_rear_suspension.gel_lower_control_arm2",
        "rear_tie_rod_inner": "tr_rear_suspension.gel_tierod_inner",
        "rear_tie_rod_outer": "tr_rear_suspension.gel_tierod_outer",
        "rear_upright": "tr_rear_suspension.gel_upright",
        "rear_spindle": "tr_rear_suspension.gel_spindle",
        "rear_upper_arm": "tr_rear_suspension.gel_upper_control_arm",
        "rack": "tr_steering.ges_rack",
        "front_wheel_left": "tr_front_tires.whl_wheel",
        "front_wheel_right": "tr_front_tires.whr_wheel",
        "rear_wheel_left": "tr_rear_tires.whl_wheel",
        "rear_wheel_right": "tr_rear_tires.whr_wheel",
        "chassis": "tr_body.ges_chassis",
        "powertrain": "tr_powertrain.ges_powertrain",
    }
    roles: dict[str, tuple[int, ...]] = {}
    for role, suffix in suffixes.items():
        selected = tuple(
            sorted(
                part_id
                for part_id, part in parts.items()
                if part.adams_name is not None
                and part.adams_name.lower().endswith(suffix)
            )
        )
        if selected:
            roles[role] = selected
    return roles


def _adams_reuler_matrix(angles_deg: tuple[float, float, float]) -> np.ndarray:
    """Return the active ZXZ rotation represented by Adams ``REULER``."""
    first, second, third = (math.radians(value) for value in angles_deg)
    z_first = np.array(
        (
            (math.cos(first), -math.sin(first), 0.0),
            (math.sin(first), math.cos(first), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    x_second = np.array(
        (
            (1.0, 0.0, 0.0),
            (0.0, math.cos(second), -math.sin(second)),
            (0.0, math.sin(second), math.cos(second)),
        )
    )
    z_third = np.array(
        (
            (math.cos(third), -math.sin(third), 0.0),
            (math.sin(third), math.cos(third), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    return z_first @ x_second @ z_third


def _matrix_tuple(value: np.ndarray) -> Matrix3:
    """Convert a 3x3 NumPy matrix to the immutable schema representation."""
    return tuple(tuple(float(item) for item in row) for row in value)


def _discover_default_database() -> Path:
    """Resolve an explicit override or the database from Adams discovery."""
    configured = os.environ.get("SUSPENSION_MULTIBODY_ADAMS_DATABASE")
    if configured:
        return Path(configured)
    if DEFAULT_ADAMS_DATABASE.is_dir():
        return DEFAULT_ADAMS_DATABASE
    if ALTERNATE_ADAMS_DATABASE.is_dir():
        return ALTERNATE_ADAMS_DATABASE
    try:
        from .probe import discover_profile

        profile = discover_profile()
    except (OSError, RuntimeError):
        return DEFAULT_ADAMS_DATABASE
    if profile.database_path:
        return Path(profile.database_path)
    return DEFAULT_ADAMS_DATABASE


def _combined_part_inertia(
    parts: Mapping[int, AdamsPartData], part_ids: tuple[int, ...]
) -> Matrix3 | None:
    """Combine selected Adams parts about their mass-weighted global COM."""
    selected = [parts[part_id] for part_id in part_ids if part_id in parts]
    if not selected:
        return None
    mass = sum(part.mass for part in selected)
    if mass <= 0.0:
        return None
    center = np.asarray(
        tuple(
            sum(part.mass * part.center_of_mass[index] for part in selected) / mass
            for index in range(3)
        ),
        dtype=float,
    )
    identity = np.eye(3)
    combined = np.zeros((3, 3), dtype=float)
    for part in selected:
        delta = np.asarray(part.center_of_mass, dtype=float) - center
        combined += part.inertia_about_com_global()
        combined += part.mass * (
            float(delta @ delta) * identity - np.outer(delta, delta)
        )
    return _matrix_tuple(combined)


def _part_mass(
    parts: Mapping[int, AdamsPartData], part_ids: tuple[int, ...], default: float
) -> float:
    """Return the positive mass represented by a semantic part group."""
    mass = sum(parts[part_id].mass for part_id in part_ids if part_id in parts)
    return float(mass) if mass > 0.0 else float(default)


def _part_inertia_component(
    parts: Mapping[int, AdamsPartData],
    part_ids: tuple[int, ...],
    *,
    axis: int,
    default: float,
) -> float:
    """Return one global inertia component for a semantic part group."""
    selected = [parts[part_id] for part_id in part_ids if part_id in parts]
    if not selected:
        return float(default)
    value = sum(
        float(part.inertia_about_com_global()[axis, axis]) for part in selected
    )
    return float(value) if value > 0.0 else float(default)


def _suspension_inertias(
    parts: Mapping[int, AdamsPartData],
    *,
    rear: bool,
    part_roles: Mapping[str, tuple[int, ...]] | None = None,
) -> dict[str, Matrix3]:
    """Map the compiled Adams suspension part inertias to runtime body roles."""
    prefix = "rear" if rear else "front"
    ids = {
        "tie_rod": _role_part_ids(part_roles, f"{prefix}_tie_rod_inner")
        + _role_part_ids(part_roles, f"{prefix}_tie_rod_outer"),
        "lower_arm": _role_part_ids(part_roles, f"{prefix}_lower_arm")
        + _role_part_ids(part_roles, f"{prefix}_lower_arm2"),
        "upright": _role_part_ids(part_roles, f"{prefix}_upright")
        + _role_part_ids(part_roles, f"{prefix}_spindle"),
        "upper_arm": _role_part_ids(part_roles, f"{prefix}_upper_arm"),
        "rack": _role_part_ids(part_roles, "rack"),
    }
    result: dict[str, Matrix3] = {}
    for role, part_ids in ids.items():
        inertia = _combined_part_inertia(parts, part_ids)
        if inertia is not None:
            result[role] = inertia
    return result


def _parse_tire(path: Path) -> dict[str, float]:
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    time_scale = _unit_factor(units.get("time"), _TIME_TO_S, 1.0)
    values: dict[str, float] = {}
    for key, raw in re.findall(r"^\s*([A-Z][A-Z0-9_]*)\s*=\s*([-+0-9.Ee]+)", text, re.MULTILINE):
        values[key] = float(raw)
    if "UNLOADED_RADIUS" in values:
        values["UNLOADED_RADIUS_MM"] = values["UNLOADED_RADIUS"] * length_scale
    if "FNOMIN" in values:
        values["FNOMIN_N"] = values["FNOMIN"] * force_scale
    if "VERTICAL_STIFFNESS" in values:
        values["VERTICAL_STIFFNESS_N_MM"] = (
            values["VERTICAL_STIFFNESS"] * force_scale / length_scale
        )
    if "VERTICAL_DAMPING" in values:
        values["VERTICAL_DAMPING_N_S_MM"] = (
            values["VERTICAL_DAMPING"] * force_scale * time_scale / length_scale
        )
    if "WIDTH" in values:
        values["WIDTH_MM"] = values["WIDTH"] * length_scale
    if "CALPHA" in values:
        values["CALPHA_N_PER_RAD"] = (
            values["CALPHA"] * force_scale
            / (length_scale / length_scale)
            / _unit_factor(units.get("angle"), _ANGLE_TO_RAD, 1.0)
        )
    if "CSLIP" in values:
        values["CSLIP_N"] = values["CSLIP"] * force_scale
    if "RELAX_LENGTH_X" in values:
        # Adams Fiala examples express relaxation length in metres even when
        # the surrounding tire file declares millimetres.
        values["RELAX_LENGTH_X_MM"] = values["RELAX_LENGTH_X"] * 1000.0
    if "RELAX_LENGTH_Y" in values:
        values["RELAX_LENGTH_Y_MM"] = values["RELAX_LENGTH_Y"] * 1000.0
    values.setdefault("SPRING_STIFFNESS_N_MM", 125.0)
    values.setdefault("SPRING_FREE_LENGTH_MM", 300.0)
    return values


def _parse_spring(path: Path) -> tuple[tuple[tuple[float, float], ...], float]:
    """Parse the installed Adams spring spline and free length."""
    text = path.read_text(encoding="latin-1")
    units = _parse_xml_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    root = ET.fromstring(text)
    free_length = 300.0
    curve: tuple[tuple[float, float], ...] = ()
    for element in root.iter():
        tag = element.tag.rsplit("}", 1)[-1]
        if tag == "SpringProperties":
            raw = element.attrib.get("freeLength")
            if raw is not None:
                free_length = float(raw) * length_scale
        if tag == "Spline" and element.text:
            curve = tuple(
                (abscissa * length_scale, ordinate * force_scale)
                for abscissa, ordinate in _parse_numeric_curve(element.text)
            )
            if curve:
                break
    if len(curve) < 2:
        raise ValueError(f"Adams spring has no usable spline curve: {path}")
    return curve, free_length


def _parse_curve_file(
    path: Path,
    *,
    abscissa: Literal["length", "velocity"],
) -> tuple[tuple[float, float], ...]:
    """Parse a two-column Adams ``[CURVE]`` file section."""
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    time_scale = _unit_factor(units.get("time"), _TIME_TO_S, 1.0)
    abscissa_scale = length_scale / time_scale if abscissa == "velocity" else length_scale
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
            rows.append(
                (
                    float(match.group(1)) * abscissa_scale,
                    float(match.group(2)) * force_scale,
                )
            )
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
    text = path.read_text(encoding="ascii", errors="replace")
    units = _parse_text_units(text)
    length_scale_m = _unit_factor(
        units.get("length"), _LENGTH_TO_MM, 1.0
    ) / 1_000.0
    time_scale_s = _unit_factor(units.get("time"), _TIME_TO_S, 1.0)
    match = re.search(r"INITIAL_SPEED\s*=\s*([-+0-9.Ee]+)", text)
    if not match:
        raise ValueError(f"Adams DCF has no INITIAL_SPEED: {path}")
    return float(match.group(1)) * length_scale_m / time_scale_s


def _result_unit_scale(unit: str | None, *, quantity: Literal["length", "angle", "linear_velocity", "angular_velocity"]) -> float:
    """将结果文件一个分量转换为 native 使用的工程单位."""
    if quantity == "length":
        return _unit_factor(unit, _LENGTH_TO_MM, 1.0)
    if quantity == "angle":
        return _unit_factor(unit, _ANGLE_TO_RAD, 1.0)
    if unit is None:
        return 1.0
    parts = unit.strip().lower().split("/")
    if len(parts) != 2:
        raise ValueError(f"unsupported Adams result velocity unit: {unit!r}")
    numerator = (
        _unit_factor(parts[0], _LENGTH_TO_MM, 1.0)
        if quantity == "linear_velocity"
        else _unit_factor(parts[0], _ANGLE_TO_RAD, 1.0)
    )
    denominator = _unit_factor(parts[1], _TIME_TO_S, 1.0)
    return numerator / denominator


def _parse_initial_part_states(path: Path | None) -> dict[int, AdamsPartState]:
    """按 StepMap 的部件 objectId 读取 Adams 初始条件."""
    if path is None:
        return {}
    root = ET.parse(path).getroot()
    step_map = root.find(".//{*}StepMap")
    data = next(
        (
            item
            for item in root.findall(".//{*}Data")
            if item.get("name") == "initialConditions_001"
        ),
        None,
    )
    if step_map is None or data is None:
        return {}
    step = data.find("{*}Step")
    if step is None:
        return {}
    values = np.asarray(
        [float(value) for value in " ".join(step.itertext()).split()],
        dtype=float,
    )
    result: dict[int, AdamsPartState] = {}
    for entity in step_map.findall("{*}Entity"):
        if entity.get("entType") != "Part" or entity.get("objectId") is None:
            continue
        components = {
            str(component.get("name")): component
            for component in entity.findall("{*}Component")
            if component.get("name") is not None
        }
        required = (
            "X", "Y", "Z", "PSI", "THETA", "PHI",
            "VX", "VY", "VZ", "WX", "WY", "WZ",
        )
        if any(name not in components for name in required):
            continue
        indices = {
            name: int(components[name].get("id", "0"))
            for name in required
        }
        if any(index <= 0 or index > len(values) for index in indices.values()):
            continue

        def component(name: str, quantity: Literal["length", "angle", "linear_velocity", "angular_velocity"]) -> float:
            item = components[name]
            return float(values[indices[name] - 1]) * _result_unit_scale(
                item.get("unitsValue"), quantity=quantity
            )

        angles = tuple(
            component(name, "angle")
            for name in ("PSI", "THETA", "PHI")
        )
        result[int(entity.get("objectId"))] = AdamsPartState(
            translation=tuple(component(name, "length") for name in ("X", "Y", "Z")),  # type: ignore[arg-type]
            rotation=_matrix_tuple(_adams_reuler_matrix(tuple(math.degrees(value) for value in angles))),
            linear_velocity=tuple(
                component(name, "linear_velocity") for name in ("VX", "VY", "VZ")
            ),  # type: ignore[arg-type]
            angular_velocity=tuple(
                component(name, "angular_velocity") for name in ("WX", "WY", "WZ")
            ),  # type: ignore[arg-type]
        )
    return result


def _parse_initial_velocity_sign(
    path: Path | None, *, chassis_part_ids: tuple[int, ...]
) -> Literal[-1, 1]:
    """从 Adams 动态结果读取车身沿全局 X 轴的实际行驶方向."""
    if path is None or not chassis_part_ids:
        return 1
    root = ET.parse(path).getroot()
    part_id = str(chassis_part_ids[0])
    entity = next(
        (
            item
            for item in root.findall(".//{*}StepMap/{*}Entity")
            if item.get("objectId") == part_id
        ),
        None,
    )
    if entity is None:
        return 1
    velocity_component = next(
        (
            int(str(component.get("id")))
            for component in entity.findall("{*}Component")
            if component.get("name") == "VX"
        ),
        None,
    )
    if velocity_component is None:
        return 1
    datasets = [
        item
        for item in root.findall(".//{*}Data")
        if item.get("name") in {"initialConditions_001", "dynamic_001"}
    ]
    for dataset in datasets:
        for step in dataset.findall("{*}Step"):
            values = [float(value) for value in "".join(step.itertext()).split()]
            if velocity_component <= len(values):
                velocity = values[velocity_component - 1]
                if abs(velocity) > 1e-9:
                    return -1 if velocity < 0.0 else 1
    return 1


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


def _composite_chassis(
    parts: Mapping[int, AdamsPartData],
    *,
    part_ids: tuple[int, ...] = (118, 123),
) -> tuple[float, tuple[float, float, float], Matrix3]:
    selected = [parts[part_id] for part_id in part_ids if part_id in parts]
    if not selected:
        raise ValueError("compiled Adams model has no mapped chassis parts")
    mass = sum(part.mass for part in selected)
    com = tuple(sum(part.mass * part.center_of_mass[index] for part in selected) / mass for index in range(3))
    inertia = _combined_part_inertia(parts, part_ids)
    if inertia is None:
        raise ValueError("compiled Adams chassis parts have no positive mass")
    return mass, com, inertia


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
