"""ctypes boundary for the packaged C++ axle dynamics kernel."""

from __future__ import annotations

import ctypes
import json
import platform
from pathlib import Path

import numpy as np

from .result import (
    DIAGNOSTIC_COLUMNS,
    AxleContactEventRecord,
    AxleDynamicsResult,
    AxleRunDiagnostics,
    AxleRunPerformance,
)
from .schema import AxleDynamicsCase, AxleDynamicsModel

_NATIVE_KERNEL_ABI_VERSION = 14


class NativeKernelUnavailableError(RuntimeError):
    """Raised when the C++ axle kernel is not installed for this platform."""


class NativeAxleError(RuntimeError):
    """Raised when the native solver rejects or fails a run."""

    def __init__(
        self,
        message: str,
        *,
        status: int,
        partial_result: AxleDynamicsResult | None = None,
        failure_diagnostics: np.ndarray | None = None,
        failed_sample_index: int | None = None,
        failed_time_s: float | None = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.partial_result = partial_result
        self.failure_diagnostics = failure_diagnostics
        self.failed_sample_index = failed_sample_index
        self.failed_time_s = failed_time_s
        self.named_failure_diagnostics = (
            None
            if failure_diagnostics is None
            else {
                name: float(value)
                for name, value in zip(
                    DIAGNOSTIC_COLUMNS, failure_diagnostics
                )
            }
        )


class _AxleInput(ctypes.Structure):
    _fields_ = [
        ("body_count", ctypes.c_size_t),
        ("body_mass", ctypes.POINTER(ctypes.c_double)),
        ("body_inertia_body_3x3", ctypes.POINTER(ctypes.c_double)),
        ("body_pose_position_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("body_velocity_omega", ctypes.POINTER(ctypes.c_double)),
        ("body_fixed", ctypes.POINTER(ctypes.c_int)),
        ("constraint_count", ctypes.c_size_t),
        ("constraint_type", ctypes.POINTER(ctypes.c_int)),
        ("constraint_body_a", ctypes.POINTER(ctypes.c_int)),
        ("constraint_body_b", ctypes.POINTER(ctypes.c_int)),
        ("constraint_point_a", ctypes.POINTER(ctypes.c_double)),
        ("constraint_point_b", ctypes.POINTER(ctypes.c_double)),
        ("constraint_axis_a", ctypes.POINTER(ctypes.c_double)),
        ("constraint_axis_b", ctypes.POINTER(ctypes.c_double)),
        ("spring_count", ctypes.c_size_t),
        ("spring_body_a", ctypes.POINTER(ctypes.c_int)),
        ("spring_body_b", ctypes.POINTER(ctypes.c_int)),
        ("spring_point_a", ctypes.POINTER(ctypes.c_double)),
        ("spring_point_b", ctypes.POINTER(ctypes.c_double)),
        ("spring_stiffness", ctypes.POINTER(ctypes.c_double)),
        ("spring_compression_damping", ctypes.POINTER(ctypes.c_double)),
        ("spring_rebound_damping", ctypes.POINTER(ctypes.c_double)),
        ("spring_free_length", ctypes.POINTER(ctypes.c_double)),
        ("spring_minimum_length", ctypes.POINTER(ctypes.c_double)),
        ("spring_maximum_length", ctypes.POINTER(ctypes.c_double)),
        ("spring_compression_stop_stiffness", ctypes.POINTER(ctypes.c_double)),
        ("spring_compression_stop_damping", ctypes.POINTER(ctypes.c_double)),
        ("spring_rebound_stop_stiffness", ctypes.POINTER(ctypes.c_double)),
        ("spring_rebound_stop_damping", ctypes.POINTER(ctypes.c_double)),
        ("spring_damper_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("spring_damper_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("spring_damper_curve_velocity", ctypes.POINTER(ctypes.c_double)),
        ("spring_damper_curve_force", ctypes.POINTER(ctypes.c_double)),
        ("bushing_count", ctypes.c_size_t),
        ("bushing_body_a", ctypes.POINTER(ctypes.c_int)),
        ("bushing_body_b", ctypes.POINTER(ctypes.c_int)),
        ("bushing_point_a", ctypes.POINTER(ctypes.c_double)),
        ("bushing_point_b", ctypes.POINTER(ctypes.c_double)),
        ("bushing_frame_a_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("bushing_frame_b_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("bushing_reference_translation", ctypes.POINTER(ctypes.c_double)),
        ("bushing_reference_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("bushing_stiffness_6x6", ctypes.POINTER(ctypes.c_double)),
        ("bushing_damping_6x6", ctypes.POINTER(ctypes.c_double)),
        ("bushing_preload_6", ctypes.POINTER(ctypes.c_double)),
        ("anti_roll_bar_count", ctypes.c_size_t),
        ("anti_roll_body_a", ctypes.POINTER(ctypes.c_int)),
        ("anti_roll_body_b", ctypes.POINTER(ctypes.c_int)),
        ("anti_roll_axis_a", ctypes.POINTER(ctypes.c_double)),
        ("anti_roll_reference_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("anti_roll_stiffness", ctypes.POINTER(ctypes.c_double)),
        ("anti_roll_damping", ctypes.POINTER(ctypes.c_double)),
        ("tire_count", ctypes.c_size_t),
        ("tire_body", ctypes.POINTER(ctypes.c_int)),
        ("tire_center_local", ctypes.POINTER(ctypes.c_double)),
        ("tire_spin_axis_local", ctypes.POINTER(ctypes.c_double)),
        ("tire_forward_axis_local", ctypes.POINTER(ctypes.c_double)),
        ("tire_radius", ctypes.POINTER(ctypes.c_double)),
        ("tire_maximum_compression", ctypes.POINTER(ctypes.c_double)),
        ("tire_stiffness", ctypes.POINTER(ctypes.c_double)),
        ("tire_damping", ctypes.POINTER(ctypes.c_double)),
        ("tire_mu_longitudinal", ctypes.POINTER(ctypes.c_double)),
        ("tire_mu_lateral", ctypes.POINTER(ctypes.c_double)),
        (
            "tire_brush_stiffness_longitudinal",
            ctypes.POINTER(ctypes.c_double),
        ),
        ("tire_brush_stiffness_lateral", ctypes.POINTER(ctypes.c_double)),
        (
            "tire_relaxation_length_longitudinal",
            ctypes.POINTER(ctypes.c_double),
        ),
        ("tire_relaxation_length_lateral", ctypes.POINTER(ctypes.c_double)),
        ("tire_detached_relaxation", ctypes.POINTER(ctypes.c_double)),
        ("sample_count", ctypes.c_size_t),
        ("sample_times", ctypes.POINTER(ctypes.c_double)),
        ("body_wrench", ctypes.POINTER(ctypes.c_double)),
        ("road_z", ctypes.POINTER(ctypes.c_double)),
        ("road_z_velocity", ctypes.POINTER(ctypes.c_double)),
        ("wheel_torque", ctypes.POINTER(ctypes.c_double)),
        ("gravity_x", ctypes.c_double),
        ("gravity_y", ctypes.c_double),
        ("gravity_z", ctypes.c_double),
        ("rho_inf", ctypes.c_double),
        ("integrator_type", ctypes.c_int),
        ("hht_alpha", ctypes.c_double),
        ("initialization_mode", ctypes.c_int),
        ("adaptive_step", ctypes.c_int),
        ("internal_step", ctypes.c_double),
        ("min_step", ctypes.c_double),
        ("max_step", ctypes.c_double),
        ("local_relative_tolerance", ctypes.c_double),
        ("local_position_tolerance", ctypes.c_double),
        ("local_angle_tolerance", ctypes.c_double),
        ("local_velocity_tolerance", ctypes.c_double),
        ("local_angular_velocity_tolerance", ctypes.c_double),
        ("local_brush_tolerance", ctypes.c_double),
        ("contact_event_tolerance", ctypes.c_double),
        ("max_newton_iterations", ctypes.c_int),
        ("max_line_search_iterations", ctypes.c_int),
        ("position_tolerance", ctypes.c_double),
        ("velocity_tolerance", ctypes.c_double),
        ("dynamics_tolerance", ctypes.c_double),
        ("increment_tolerance", ctypes.c_double),
    ]


class _AxleOutput(ctypes.Structure):
    _fields_ = [
        ("body_state", ctypes.POINTER(ctypes.c_double)),
        ("body_state_capacity", ctypes.c_size_t),
        ("constraint_wrench", ctypes.POINTER(ctypes.c_double)),
        ("constraint_wrench_capacity", ctypes.c_size_t),
        ("spring_output", ctypes.POINTER(ctypes.c_double)),
        ("spring_output_capacity", ctypes.c_size_t),
        ("bushing_output", ctypes.POINTER(ctypes.c_double)),
        ("bushing_output_capacity", ctypes.c_size_t),
        ("anti_roll_output", ctypes.POINTER(ctypes.c_double)),
        ("anti_roll_output_capacity", ctypes.c_size_t),
        ("diagnostics", ctypes.POINTER(ctypes.c_double)),
        ("diagnostics_capacity", ctypes.c_size_t),
        ("tire_output", ctypes.POINTER(ctypes.c_double)),
        ("tire_output_capacity", ctypes.c_size_t),
        ("energy_output", ctypes.POINTER(ctypes.c_double)),
        ("energy_output_capacity", ctypes.c_size_t),
        ("contact_event_output", ctypes.POINTER(ctypes.c_double)),
        ("contact_event_output_capacity", ctypes.c_size_t),
        ("contact_event_count", ctypes.POINTER(ctypes.c_size_t)),
    ]


def _ptr(array: np.ndarray, ctype: type[ctypes.c_double] | type[ctypes.c_int]):
    return array.ctypes.data_as(ctypes.POINTER(ctype))


def _library_path() -> Path:
    root = Path(__file__).resolve().parent.parent / "native"
    if platform.system() == "Windows":
        return root / "axle_dynamics_native.dll"
    if platform.system() == "Darwin":
        return root / "libaxle_dynamics_native.dylib"
    return root / "libaxle_dynamics_native.so"


def native_build_metadata() -> dict[str, object]:
    """Return the recorded compiler and ABI metadata."""
    path = _library_path().with_name("native_build.json")
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8-sig"))


def _load_library() -> ctypes.CDLL:
    path = _library_path()
    if not path.exists():
        raise NativeKernelUnavailableError(
            f"native axle kernel is unavailable at {path}; "
            "run packages/suspension_multibody/scripts/build_axle_native.ps1"
        )
    library = ctypes.CDLL(str(path))
    library.axle_kernel_abi_version.argtypes = []
    library.axle_kernel_abi_version.restype = ctypes.c_int
    if library.axle_kernel_abi_version() != _NATIVE_KERNEL_ABI_VERSION:
        raise NativeKernelUnavailableError(
            "native axle kernel ABI is not version "
            f"{_NATIVE_KERNEL_ABI_VERSION}"
        )
    library.axle_run.argtypes = [
        ctypes.POINTER(_AxleInput),
        ctypes.POINTER(_AxleOutput),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    library.axle_run.restype = ctypes.c_int
    return library


def _signal_matrix(
    case: AxleDynamicsCase,
    tire_names: tuple[str, ...],
    signals: dict[str, tuple[float, ...]],
) -> np.ndarray:
    values = np.zeros((len(case.times_s), len(tire_names)), dtype=np.float64)
    unknown = set(signals) - set(tire_names)
    if unknown:
        raise ValueError(f"signals reference unknown tires: {sorted(unknown)}")
    for column, name in enumerate(tire_names):
        if name in signals:
            values[:, column] = signals[name]
    return np.ascontiguousarray(values)


def _body_wrench_matrix(
    case: AxleDynamicsCase,
    body_names: tuple[str, ...],
) -> np.ndarray:
    values = np.zeros(
        (len(case.times_s), len(body_names), 6),
        dtype=np.float64,
    )
    unknown = set(case.body_wrench_n_n_m) - set(body_names)
    if unknown:
        raise ValueError(f"wrenches reference unknown bodies: {sorted(unknown)}")
    for body_index, name in enumerate(body_names):
        if name in case.body_wrench_n_n_m:
            values[:, body_index, :] = case.body_wrench_n_n_m[name]
    return np.ascontiguousarray(values)


def run_axle_dynamics(
    model: AxleDynamicsModel, case: AxleDynamicsCase
) -> AxleDynamicsResult:
    """Run one validated SI axle case through the native C++ kernel."""
    body_names = tuple(body.name for body in model.bodies)
    body_index = {name: index for index, name in enumerate(body_names)}
    tire_names = tuple(tire.name for tire in model.tires)

    mass = np.ascontiguousarray([body.mass_kg for body in model.bodies])
    inertia = np.ascontiguousarray(
        [body.inertia_kg_m2 for body in model.bodies], dtype=np.float64
    )
    poses = np.ascontiguousarray(
        [
            (*body.position_m, *body.quaternion_body_to_world)
            for body in model.bodies
        ],
        dtype=np.float64,
    )
    velocities = np.ascontiguousarray(
        [
            (*body.linear_velocity_m_per_s, *body.angular_velocity_rad_per_s)
            for body in model.bodies
        ],
        dtype=np.float64,
    )
    fixed = np.ascontiguousarray(
        [int(body.fixed) for body in model.bodies], dtype=np.int32
    )
    joint_kind = {
        "spherical": 0,
        "revolute": 1,
        "fixed": 2,
        "prismatic": 3,
        "universal": 4,
        "cylindrical": 5,
        "inplane": 6,
    }
    constraint_type = np.ascontiguousarray(
        [joint_kind[joint.kind] for joint in model.joints], dtype=np.int32
    )
    constraint_a = np.ascontiguousarray(
        [body_index[joint.body_a] for joint in model.joints], dtype=np.int32
    )
    constraint_b = np.ascontiguousarray(
        [body_index[joint.body_b] for joint in model.joints], dtype=np.int32
    )
    constraint_point_a = np.ascontiguousarray(
        [joint.point_a_m for joint in model.joints], dtype=np.float64
    )
    constraint_point_b = np.ascontiguousarray(
        [joint.point_b_m for joint in model.joints], dtype=np.float64
    )
    constraint_axis_a = np.ascontiguousarray(
        [joint.axis_a for joint in model.joints], dtype=np.float64
    )
    constraint_axis_b = np.ascontiguousarray(
        [joint.axis_b for joint in model.joints], dtype=np.float64
    )
    spring_a = np.ascontiguousarray(
        [body_index[spring.body_a] for spring in model.springs], dtype=np.int32
    )
    spring_b = np.ascontiguousarray(
        [body_index[spring.body_b] for spring in model.springs], dtype=np.int32
    )
    spring_point_a = np.ascontiguousarray(
        [spring.point_a_m for spring in model.springs], dtype=np.float64
    )
    spring_point_b = np.ascontiguousarray(
        [spring.point_b_m for spring in model.springs], dtype=np.float64
    )
    spring_k = np.ascontiguousarray(
        [spring.stiffness_n_per_m for spring in model.springs]
    )
    spring_c_compression = np.ascontiguousarray(
        [spring.compression_damping_n_s_per_m for spring in model.springs]
    )
    spring_c_rebound = np.ascontiguousarray(
        [spring.rebound_damping_n_s_per_m for spring in model.springs]
    )
    spring_l0 = np.ascontiguousarray(
        [spring.free_length_m for spring in model.springs]
    )
    spring_min = np.ascontiguousarray(
        [
            np.nan if spring.minimum_length_m is None else spring.minimum_length_m
            for spring in model.springs
        ],
        dtype=np.float64,
    )
    spring_max = np.ascontiguousarray(
        [
            np.nan if spring.maximum_length_m is None else spring.maximum_length_m
            for spring in model.springs
        ],
        dtype=np.float64,
    )
    spring_stop_k_compression = np.ascontiguousarray(
        [spring.compression_stop_stiffness_n_per_m for spring in model.springs]
    )
    spring_stop_c_compression = np.ascontiguousarray(
        [spring.compression_stop_damping_n_s_per_m for spring in model.springs]
    )
    spring_stop_k_rebound = np.ascontiguousarray(
        [spring.rebound_stop_stiffness_n_per_m for spring in model.springs]
    )
    spring_stop_c_rebound = np.ascontiguousarray(
        [spring.rebound_stop_damping_n_s_per_m for spring in model.springs]
    )
    # Damper curves are concatenated; each spring records where its points
    # start and how many it owns, so a spring with no curve reads as count 0.
    damper_curve_offset: list[int] = []
    damper_curve_count: list[int] = []
    damper_velocity_points: list[float] = []
    damper_force_points: list[float] = []
    for spring in model.springs:
        damper_curve_offset.append(len(damper_velocity_points))
        damper_curve_count.append(len(spring.damper_curve_velocity_m_per_s))
        damper_velocity_points.extend(spring.damper_curve_velocity_m_per_s)
        damper_force_points.extend(spring.damper_curve_force_n)
    spring_curve_offset = np.ascontiguousarray(
        damper_curve_offset or [0], dtype=np.int32
    )
    spring_curve_count = np.ascontiguousarray(
        damper_curve_count or [0], dtype=np.int32
    )
    spring_curve_velocity = np.ascontiguousarray(
        damper_velocity_points or [0.0], dtype=np.float64
    )
    spring_curve_force = np.ascontiguousarray(
        damper_force_points or [0.0], dtype=np.float64
    )
    bushing_a = np.ascontiguousarray(
        [body_index[bushing.body_a] for bushing in model.bushings], dtype=np.int32
    )
    bushing_b = np.ascontiguousarray(
        [body_index[bushing.body_b] for bushing in model.bushings], dtype=np.int32
    )
    bushing_point_a = np.ascontiguousarray(
        [bushing.point_a_m for bushing in model.bushings], dtype=np.float64
    )
    bushing_point_b = np.ascontiguousarray(
        [bushing.point_b_m for bushing in model.bushings], dtype=np.float64
    )
    bushing_frame_a = np.ascontiguousarray(
        [bushing.frame_a_to_body_quaternion for bushing in model.bushings],
        dtype=np.float64,
    )
    bushing_frame_b = np.ascontiguousarray(
        [bushing.frame_b_to_body_quaternion for bushing in model.bushings],
        dtype=np.float64,
    )
    bushing_reference_translation = np.ascontiguousarray(
        [bushing.reference_translation_in_frame_a_m for bushing in model.bushings],
        dtype=np.float64,
    )
    bushing_reference_quaternion = np.ascontiguousarray(
        [bushing.reference_quaternion_a_to_b for bushing in model.bushings],
        dtype=np.float64,
    )
    bushing_stiffness = np.ascontiguousarray(
        [bushing.stiffness for bushing in model.bushings], dtype=np.float64
    )
    bushing_damping = np.ascontiguousarray(
        [bushing.damping for bushing in model.bushings], dtype=np.float64
    )
    bushing_preload = np.ascontiguousarray(
        [bushing.preload_in_frame_a_n_n_m for bushing in model.bushings],
        dtype=np.float64,
    )
    anti_roll_a = np.ascontiguousarray(
        [body_index[bar.body_a] for bar in model.anti_roll_bars], dtype=np.int32
    )
    anti_roll_b = np.ascontiguousarray(
        [body_index[bar.body_b] for bar in model.anti_roll_bars], dtype=np.int32
    )
    anti_roll_axis = np.ascontiguousarray(
        [bar.axis_a for bar in model.anti_roll_bars], dtype=np.float64
    )
    anti_roll_reference = np.ascontiguousarray(
        [bar.reference_quaternion_a_to_b for bar in model.anti_roll_bars],
        dtype=np.float64,
    )
    anti_roll_k = np.ascontiguousarray(
        [bar.stiffness_n_m_per_rad for bar in model.anti_roll_bars]
    )
    anti_roll_c = np.ascontiguousarray(
        [bar.damping_n_m_s_per_rad for bar in model.anti_roll_bars]
    )
    tire_body = np.ascontiguousarray(
        [body_index[tire.body] for tire in model.tires], dtype=np.int32
    )
    tire_center = np.ascontiguousarray(
        [tire.center_local_m for tire in model.tires], dtype=np.float64
    )
    tire_spin_axis = np.ascontiguousarray(
        [tire.spin_axis_local for tire in model.tires], dtype=np.float64
    )
    tire_forward_axis = np.ascontiguousarray(
        [tire.forward_axis_local for tire in model.tires], dtype=np.float64
    )
    tire_radius = np.ascontiguousarray(
        [tire.unloaded_radius_m for tire in model.tires]
    )
    tire_maximum_compression = np.ascontiguousarray(
        [tire.maximum_compression_m for tire in model.tires]
    )
    tire_k = np.ascontiguousarray(
        [tire.vertical_stiffness_n_per_m for tire in model.tires]
    )
    tire_c = np.ascontiguousarray(
        [tire.vertical_damping_n_s_per_m for tire in model.tires]
    )
    tire_mu_longitudinal = np.ascontiguousarray(
        [tire.longitudinal_friction_coefficient for tire in model.tires]
    )
    tire_mu_lateral = np.ascontiguousarray(
        [tire.lateral_friction_coefficient for tire in model.tires]
    )
    tire_brush_k_longitudinal = np.ascontiguousarray(
        [tire.longitudinal_brush_stiffness_n_per_m for tire in model.tires]
    )
    tire_brush_k_lateral = np.ascontiguousarray(
        [tire.lateral_brush_stiffness_n_per_m for tire in model.tires]
    )
    tire_relaxation_length_longitudinal = np.ascontiguousarray(
        [tire.longitudinal_relaxation_length_m for tire in model.tires]
    )
    tire_relaxation_length_lateral = np.ascontiguousarray(
        [tire.lateral_relaxation_length_m for tire in model.tires]
    )
    tire_detached_relaxation = np.ascontiguousarray(
        [tire.detached_relaxation_s for tire in model.tires]
    )
    times = np.ascontiguousarray(case.times_s, dtype=np.float64)
    body_wrench = _body_wrench_matrix(case, body_names)
    road_z = _signal_matrix(case, tire_names, case.road_height_m)
    road_v = _signal_matrix(case, tire_names, case.road_velocity_m_per_s)
    wheel_torque = _signal_matrix(case, tire_names, case.wheel_torque_n_m)

    states = np.full(
        (len(times), len(body_names), 19), np.nan, dtype=np.float64
    )
    constraint_wrench = np.full(
        (len(times), len(model.joints), 6), np.nan, dtype=np.float64
    )
    spring_output = np.full(
        (len(times), len(model.springs), 7), np.nan, dtype=np.float64
    )
    bushing_output = np.full(
        (len(times), len(model.bushings), 12), np.nan, dtype=np.float64
    )
    anti_roll_output = np.full(
        (len(times), len(model.anti_roll_bars), 3), np.nan, dtype=np.float64
    )
    # The optional row after the public sample diagnostics carries aggregate
    # native counters when SUSPENSION_AXLE_PROFILE is enabled.  It is not part
    # of the returned per-sample diagnostics array.
    # Keep the native 16-column row stride unchanged. The 23-value optional
    # performance record spans the first two tail rows after public samples.
    diagnostics = np.full((len(times) + 2, 16), np.nan, dtype=np.float64)
    tire_output = np.full(
        (len(times), len(tire_names), 12), np.nan, dtype=np.float64
    )
    energy = np.full((len(times), 21), np.nan, dtype=np.float64)
    event_capacity = max(16, len(times) * max(1, len(tire_names)) * 4)
    contact_event_output = np.full(
        (event_capacity, 3),
        np.nan,
        dtype=np.float64,
    )
    contact_event_count = ctypes.c_size_t(0)
    settings = case.solver
    native_input = _AxleInput(
        len(body_names),
        _ptr(mass, ctypes.c_double),
        _ptr(inertia, ctypes.c_double),
        _ptr(poses, ctypes.c_double),
        _ptr(velocities, ctypes.c_double),
        _ptr(fixed, ctypes.c_int),
        len(model.joints),
        _ptr(constraint_type, ctypes.c_int),
        _ptr(constraint_a, ctypes.c_int),
        _ptr(constraint_b, ctypes.c_int),
        _ptr(constraint_point_a, ctypes.c_double),
        _ptr(constraint_point_b, ctypes.c_double),
        _ptr(constraint_axis_a, ctypes.c_double),
        _ptr(constraint_axis_b, ctypes.c_double),
        len(model.springs),
        _ptr(spring_a, ctypes.c_int),
        _ptr(spring_b, ctypes.c_int),
        _ptr(spring_point_a, ctypes.c_double),
        _ptr(spring_point_b, ctypes.c_double),
        _ptr(spring_k, ctypes.c_double),
        _ptr(spring_c_compression, ctypes.c_double),
        _ptr(spring_c_rebound, ctypes.c_double),
        _ptr(spring_l0, ctypes.c_double),
        _ptr(spring_min, ctypes.c_double),
        _ptr(spring_max, ctypes.c_double),
        _ptr(spring_stop_k_compression, ctypes.c_double),
        _ptr(spring_stop_c_compression, ctypes.c_double),
        _ptr(spring_stop_k_rebound, ctypes.c_double),
        _ptr(spring_stop_c_rebound, ctypes.c_double),
        _ptr(spring_curve_offset, ctypes.c_int),
        _ptr(spring_curve_count, ctypes.c_int),
        _ptr(spring_curve_velocity, ctypes.c_double),
        _ptr(spring_curve_force, ctypes.c_double),
        len(model.bushings),
        _ptr(bushing_a, ctypes.c_int),
        _ptr(bushing_b, ctypes.c_int),
        _ptr(bushing_point_a, ctypes.c_double),
        _ptr(bushing_point_b, ctypes.c_double),
        _ptr(bushing_frame_a, ctypes.c_double),
        _ptr(bushing_frame_b, ctypes.c_double),
        _ptr(bushing_reference_translation, ctypes.c_double),
        _ptr(bushing_reference_quaternion, ctypes.c_double),
        _ptr(bushing_stiffness, ctypes.c_double),
        _ptr(bushing_damping, ctypes.c_double),
        _ptr(bushing_preload, ctypes.c_double),
        len(model.anti_roll_bars),
        _ptr(anti_roll_a, ctypes.c_int),
        _ptr(anti_roll_b, ctypes.c_int),
        _ptr(anti_roll_axis, ctypes.c_double),
        _ptr(anti_roll_reference, ctypes.c_double),
        _ptr(anti_roll_k, ctypes.c_double),
        _ptr(anti_roll_c, ctypes.c_double),
        len(model.tires),
        _ptr(tire_body, ctypes.c_int),
        _ptr(tire_center, ctypes.c_double),
        _ptr(tire_spin_axis, ctypes.c_double),
        _ptr(tire_forward_axis, ctypes.c_double),
        _ptr(tire_radius, ctypes.c_double),
        _ptr(tire_maximum_compression, ctypes.c_double),
        _ptr(tire_k, ctypes.c_double),
        _ptr(tire_c, ctypes.c_double),
        _ptr(tire_mu_longitudinal, ctypes.c_double),
        _ptr(tire_mu_lateral, ctypes.c_double),
        _ptr(tire_brush_k_longitudinal, ctypes.c_double),
        _ptr(tire_brush_k_lateral, ctypes.c_double),
        _ptr(tire_relaxation_length_longitudinal, ctypes.c_double),
        _ptr(tire_relaxation_length_lateral, ctypes.c_double),
        _ptr(tire_detached_relaxation, ctypes.c_double),
        len(times),
        _ptr(times, ctypes.c_double),
        _ptr(body_wrench, ctypes.c_double),
        _ptr(road_z, ctypes.c_double),
        _ptr(road_v, ctypes.c_double),
        _ptr(wheel_torque, ctypes.c_double),
        model.gravity_m_per_s2[0],
        model.gravity_m_per_s2[1],
        model.gravity_m_per_s2[2],
        settings.rho_inf,
        {"ggl_generalized_alpha": 0, "hht": 1}[settings.integrator],
        settings.hht_alpha,
        {
            "static_equilibrium": 0,
            "provided_consistent_state": 1,
        }[settings.initialization_mode],
        int(settings.adaptive_step),
        settings.internal_step_s,
        settings.minimum_step_s,
        settings.maximum_step_s,
        settings.local_relative_tolerance,
        settings.local_position_tolerance_m,
        settings.local_angle_tolerance_rad,
        settings.local_velocity_tolerance_m_per_s,
        settings.local_angular_velocity_tolerance_rad_per_s,
        settings.local_brush_tolerance_m,
        settings.contact_event_tolerance_s,
        settings.max_newton_iterations,
        settings.max_line_search_iterations,
        settings.position_tolerance_m,
        settings.velocity_tolerance_m_per_s,
        settings.dynamics_tolerance,
        settings.increment_tolerance,
    )
    library = _load_library()
    while True:
        native_output = _AxleOutput(
            _ptr(states, ctypes.c_double),
            states.size,
            _ptr(constraint_wrench, ctypes.c_double),
            constraint_wrench.size,
            _ptr(spring_output, ctypes.c_double),
            spring_output.size,
            _ptr(bushing_output, ctypes.c_double),
            bushing_output.size,
            _ptr(anti_roll_output, ctypes.c_double),
            anti_roll_output.size,
            _ptr(diagnostics, ctypes.c_double),
            diagnostics.size,
            _ptr(tire_output, ctypes.c_double),
            tire_output.size,
            _ptr(energy, ctypes.c_double),
            energy.size,
            _ptr(contact_event_output, ctypes.c_double),
            contact_event_output.size,
            ctypes.pointer(contact_event_count),
        )
        error_buffer = ctypes.create_string_buffer(4096)
        status = library.axle_run(
            ctypes.byref(native_input),
            ctypes.byref(native_output),
            error_buffer,
            len(error_buffer),
        )
        if status != 10 or contact_event_count.value <= event_capacity:
            break
        event_capacity = int(contact_event_count.value)
        contact_event_output = np.full(
            (event_capacity, 3),
            np.nan,
            dtype=np.float64,
        )

    def build_result(sample_count: int) -> AxleDynamicsResult:
        diagnostic_rows = diagnostics[:sample_count]
        performance_row = np.concatenate(
            (diagnostics[len(times)], diagnostics[len(times) + 1, :7])
        )

        def metric_int(index: int) -> int:
            value = performance_row[index]
            return int(value) if np.isfinite(value) else 0

        def metric_float(index: int) -> float:
            value = performance_row[index]
            return float(value) if np.isfinite(value) else 0.0

        performance = AxleRunPerformance(
            available=bool(np.isfinite(performance_row[0]) and performance_row[0] > 0.5),
            residual_calls=metric_int(1),
            residual_time_s=metric_float(2),
            constraint_jacobian_calls=metric_int(3),
            constraint_jacobian_time_s=metric_float(4),
            force_evaluations=metric_int(5),
            force_time_s=metric_float(6),
            mass_inverse_calls=metric_int(7),
            mass_inverse_time_s=metric_float(8),
            reaction_time_s=metric_float(9),
            linear_factorizations=metric_int(10),
            linear_factorization_time_s=metric_float(11),
            linear_solves=metric_int(12),
            linear_solve_time_s=metric_float(13),
            line_search_trials=metric_int(14),
            newton_iterations=metric_int(15),
            accepted_steps=metric_int(16),
            rejected_attempts=metric_int(17),
            analytic_jacobian_columns=metric_int(18),
            finite_difference_jacobian_columns=metric_int(19),
            nonsmooth_fallback_columns=metric_int(20),
            analytic_jacobian_time_s=metric_float(21),
            finite_difference_jacobian_time_s=metric_float(22),
        )
        event_rows = contact_event_output[
            : min(int(contact_event_count.value), event_capacity)
        ]
        contact_events = tuple(
            AxleContactEventRecord(
                time_s=float(row[0]),
                tire=tire_names[int(row[1])],
                transition="enter" if int(row[2]) > 0 else "exit",
            )
            for row in event_rows
        )
        return AxleDynamicsResult(
            times_s=times[:sample_count],
            body_names=body_names,
            constraint_names=tuple(joint.name for joint in model.joints),
            spring_names=tuple(spring.name for spring in model.springs),
            bushing_names=tuple(bushing.name for bushing in model.bushings),
            anti_roll_bar_names=tuple(
                bar.name for bar in model.anti_roll_bars
            ),
            tire_names=tire_names,
            states=states[:sample_count],
            constraint_wrench=constraint_wrench[:sample_count],
            spring_output=spring_output[:sample_count],
            bushing_output=bushing_output[:sample_count],
            anti_roll_output=anti_roll_output[:sample_count],
            diagnostics=AxleRunDiagnostics(
                accepted=diagnostic_rows[:, 0].astype(bool),
                internal_steps=diagnostic_rows[:, 1].astype(int),
                rejected_attempts=diagnostic_rows[:, 2].astype(int),
                newton_iterations=diagnostic_rows[:, 3].astype(int),
                minimum_accepted_step_s=diagnostic_rows[:, 4],
                maximum_accepted_step_s=diagnostic_rows[:, 5],
                last_accepted_step_s=diagnostic_rows[:, 6],
                position_residual=diagnostic_rows[:, 7],
                velocity_residual=diagnostic_rows[:, 8],
                dynamics_residual=diagnostic_rows[:, 9],
                active_contacts=diagnostic_rows[:, 10].astype(int),
                contact_events=diagnostic_rows[:, 11].astype(int),
                local_error_ratio=diagnostic_rows[:, 12],
                energy_residual=diagnostic_rows[:, 13],
                failure_code=diagnostic_rows[:, 14].astype(int),
                pinned_null_directions=diagnostic_rows[:, 15].astype(int),
            ),
            tire_output=tire_output[:sample_count],
            energy=energy[:sample_count],
            contact_events=contact_events,
            performance=performance,
        )

    if status != 0:
        message = error_buffer.value.decode("utf-8", errors="replace")
        failed_rows = np.flatnonzero(
            np.isfinite(diagnostics[: len(times), 0])
            & (diagnostics[: len(times), 0] == 0.0)
        )
        failed_index = int(failed_rows[0]) if failed_rows.size else None
        partial_result = (
            build_result(failed_index)
            if failed_index is not None and failed_index > 0
            else None
        )
        failure_diagnostics = (
            diagnostics[failed_index, :16].copy()
            if failed_index is not None
            else None
        )
        raise NativeAxleError(
            f"native axle solver failed ({status}): {message}",
            status=status,
            partial_result=partial_result,
            failure_diagnostics=failure_diagnostics,
            failed_sample_index=failed_index,
            failed_time_s=(
                float(times[failed_index])
                if failed_index is not None
                else None
            ),
        )
    return build_result(len(times))
