"""ctypes boundary for the packaged C++ axle dynamics kernel."""

from __future__ import annotations

import ctypes
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np
from suspension_kernel.binding import (
    KernelAbiMismatchError,
    KernelSymbolMissingError,
)
from suspension_kernel.binding import (
    NativeKernelUnavailableError as KernelUnavailableError,
)
from suspension_kernel.binding import (
    library_path as kernel_library_path,
)
from suspension_kernel.binding import (
    load_kernel_library as kernel_load_library,
)
from suspension_kernel.binding import (
    native_build_metadata as kernel_build_metadata,
)
from suspension_kernel.binding import (
    native_directory as kernel_native_directory,
)

from .result import (
    DIAGNOSTIC_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
    AxleContactEventRecord,
    AxleDynamicsResult,
    AxleRunDiagnostics,
    AxleRunPerformance,
)
from .schema import (
    PAC2002_PARAMETER_DEFAULTS,
    PAC2002_PARAMETER_NAMES,
    AxleDynamicsCase,
    AxleDynamicsModel,
)

#: Library base name and the axle package's own copy of the built product.  The
#: kernel is built by `packages/suspension_kernel`; this package keeps a copy in
#: its `native` directory and loads it from there, which is the deployed layout
#: the wheel already relied on.
_LIBRARY_STEM = "suspension_kernel"
_NATIVE_DIR = Path(__file__).resolve().parent.parent / "native"

#: The Python side is still a second source of truth for the ABI numbers, as it
#: always was.  The kernel's `binding` layer checks the library against these at
#: load time, and `_require_matching_metadata_versions` checks them against the
#: metadata the build read back out of the artefact, so a one-sided bump is named
#: at the boundary instead of silently reading the wrong struct layout.
_NATIVE_KERNEL_ABI_VERSION = 15
_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 30
#: The generic core surface (`mb_core_*`).  It has its own version because it is
#: the one entry point a non-suspension product links against.
_NATIVE_CORE_ABI_VERSION = 1


def _is_right_tire_name(name: str) -> bool:
    """识别源模型中常见的右侧轮胎命名，不依赖固定完整名称."""
    normalized = name.strip().lower().replace("-", "_")
    return normalized.endswith(("_right", "_r", "right"))


class NativeKernelUnavailableError(KernelUnavailableError):
    """
    Raised when the C++ axle kernel is not installed for this platform.

    Subclasses the kernel binding's same-named error so that a caller catching
    either package's class catches this condition.  Two unrelated classes with
    one name was itself a trap: code written against one silently failed to catch
    the other.
    """


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
        # Extension protocol, mirrored from `AxleInput` in the C ABI header.
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
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
        # Generic element surface.  These must be present even while the kernel
        # still reads the per-family arrays above, because the kernel checks the
        # caller's `struct_size`: a mirror that stops short of the real structure
        # is rejected outright rather than partly read.
        ("element_count", ctypes.c_size_t),
        ("elements", ctypes.c_void_p),
        ("element_curves", ctypes.c_void_p),
        ("topology_extension_count", ctypes.c_size_t),
        ("topology_extensions", ctypes.c_void_p),
    ]


class _AxleOutput(ctypes.Structure):
    _fields_ = [
        # Extension protocol, mirrored from `AxleOutput` in the C ABI header.
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
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


class _VehicleInput(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("axle", _AxleInput),
        ("steering_count", ctypes.c_size_t),
        ("steering_type", ctypes.POINTER(ctypes.c_int)),
        ("steering_body", ctypes.POINTER(ctypes.c_int)),
        ("steering_reaction_body", ctypes.POINTER(ctypes.c_int)),
        ("steering_point_local", ctypes.POINTER(ctypes.c_double)),
        ("steering_reaction_point_local", ctypes.POINTER(ctypes.c_double)),
        ("steering_axis_local", ctypes.POINTER(ctypes.c_double)),
        ("steering_reference_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("steering_target_angle", ctypes.POINTER(ctypes.c_double)),
        ("steering_target_rate", ctypes.POINTER(ctypes.c_double)),
        ("steering_stiffness", ctypes.POINTER(ctypes.c_double)),
        ("steering_damping", ctypes.POINTER(ctypes.c_double)),
        ("road_kind", ctypes.c_int),
        ("road_origin_x", ctypes.c_double),
        ("road_origin_z", ctypes.c_double),
        ("road_amplitude", ctypes.c_double),
        ("road_wavelength", ctypes.c_double),
        ("road_phase", ctypes.c_double),
        ("road_bump_start", ctypes.c_double),
        ("road_bump_length", ctypes.c_double),
        ("road_corner_scale", ctypes.POINTER(ctypes.c_double)),
        ("brake_torque", ctypes.POINTER(ctypes.c_double)),
        ("static_gauge_body", ctypes.c_size_t),
        ("static_gauge_dof_mask", ctypes.c_uint32),
        ("static_trim_then_release", ctypes.c_int),
        ("tire_frame_body", ctypes.POINTER(ctypes.c_int)),
        ("tire_frame_center_local", ctypes.POINTER(ctypes.c_double)),
        ("tire_model_kind", ctypes.POINTER(ctypes.c_int)),
        ("tire_pac2002_parameters", ctypes.POINTER(ctypes.c_double)),
        ("tire_pac2002_mirror", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_elastic_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_elastic_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_elastic_curve_deflection", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_spring_elastic_curve_force", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_spring_compression_stop_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_compression_stop_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_compression_stop_curve_penetration", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_spring_compression_stop_curve_force", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_spring_rebound_stop_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_rebound_stop_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_spring_rebound_stop_curve_penetration", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_spring_rebound_stop_curve_force", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_bushing_force_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_bushing_force_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("vehicle_bushing_force_curve_coordinate", ctypes.POINTER(ctypes.c_double)),
        ("vehicle_bushing_force_curve_force", ctypes.POINTER(ctypes.c_double)),
        ("constraint_axis_a_secondary", ctypes.POINTER(ctypes.c_double)),
        ("constraint_axis_b_secondary", ctypes.POINTER(ctypes.c_double)),
        ("constraint_convel_angle_target", ctypes.POINTER(ctypes.c_double)),
        ("static_rotation_gauge_count", ctypes.c_size_t),
        ("static_rotation_gauge_body", ctypes.POINTER(ctypes.c_int)),
        ("static_rotation_gauge_axis_local", ctypes.POINTER(ctypes.c_double)),
        ("initial_state_angle_tolerance", ctypes.c_double),
        ("bushing_rotation_coordinates", ctypes.POINTER(ctypes.c_int)),
        ("coordinate_coupler_count", ctypes.c_size_t),
        ("coordinate_coupler_joint_a", ctypes.POINTER(ctypes.c_int)),
        ("coordinate_coupler_coordinate_a", ctypes.POINTER(ctypes.c_int)),
        ("coordinate_coupler_scale_a", ctypes.POINTER(ctypes.c_double)),
        ("coordinate_coupler_joint_b", ctypes.POINTER(ctypes.c_int)),
        ("coordinate_coupler_coordinate_b", ctypes.POINTER(ctypes.c_int)),
        ("coordinate_coupler_scale_b", ctypes.POINTER(ctypes.c_double)),
        ("aerodynamic_drag_count", ctypes.c_size_t),
        ("aerodynamic_drag_body", ctypes.POINTER(ctypes.c_int)),
        ("aerodynamic_drag_application_point", ctypes.POINTER(ctypes.c_double)),
        ("aerodynamic_drag_forward_axis", ctypes.POINTER(ctypes.c_double)),
        ("aerodynamic_drag_coefficient", ctypes.POINTER(ctypes.c_double)),
        ("tire_drive_torque_body", ctypes.POINTER(ctypes.c_int)),
        ("tire_drive_torque_reaction_body", ctypes.POINTER(ctypes.c_int)),
        ("tire_drive_torque_axis_local", ctypes.POINTER(ctypes.c_double)),
        ("bushing_force_curve_interpolation", ctypes.POINTER(ctypes.c_int)),
        ("tire_deflection_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("tire_deflection_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("tire_deflection_curve_deflection", ctypes.POINTER(ctypes.c_double)),
        ("tire_deflection_curve_force", ctypes.POINTER(ctypes.c_double)),
        ("tire_bottoming_curve_offset", ctypes.POINTER(ctypes.c_int)),
        ("tire_bottoming_curve_count", ctypes.POINTER(ctypes.c_int)),
        ("tire_bottoming_curve_penetration", ctypes.POINTER(ctypes.c_double)),
        ("tire_bottoming_curve_force", ctypes.POINTER(ctypes.c_double)),
        # Generic kinematic driver (vehicle ABI 29).  Appended last so the
        # existing field offsets are unchanged; the kernel guards their presence
        # with struct_size.
        ("driven_count", ctypes.c_size_t),
        ("driven_type", ctypes.POINTER(ctypes.c_int)),
        ("driven_body", ctypes.POINTER(ctypes.c_int)),
        ("driven_reaction_body", ctypes.POINTER(ctypes.c_int)),
        ("driven_point_local", ctypes.POINTER(ctypes.c_double)),
        ("driven_reaction_point_local", ctypes.POINTER(ctypes.c_double)),
        ("driven_axis_local", ctypes.POINTER(ctypes.c_double)),
        ("driven_reference_quaternion", ctypes.POINTER(ctypes.c_double)),
        ("driven_target", ctypes.POINTER(ctypes.c_double)),
        ("driven_target_rate", ctypes.POINTER(ctypes.c_double)),
        # Generic element surface, shared verbatim with `_AxleInput` so a `kind`
        # has one layout on both entry points.  These have to be listed even while
        # the kernel still reads the per-family arrays, because `struct_size` is
        # checked against the real structure.
        ("element_count", ctypes.c_size_t),
        ("elements", ctypes.c_void_p),
        ("element_curves", ctypes.c_void_p),
        ("topology_extension_count", ctypes.c_size_t),
        ("topology_extensions", ctypes.c_void_p),
    ]


class _VehicleOutput(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("axle", _AxleOutput),
        ("steering_output", ctypes.POINTER(ctypes.c_double)),
        ("steering_output_capacity", ctypes.c_size_t),
    ]


@dataclass(frozen=True)
class _VehicleSteeringBuffers:
    names: tuple[str, ...]
    actuator_type: np.ndarray
    body: np.ndarray
    reaction_body: np.ndarray
    point_local: np.ndarray
    reaction_point_local: np.ndarray
    axis_local: np.ndarray
    reference_quaternion: np.ndarray
    target: np.ndarray
    target_rate: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    output: np.ndarray


@dataclass(frozen=True)
class _VehicleDrivenBuffers:
    """
    Per-coordinate geometry and per-sample targets of the driven coordinates.

    The targets are flattened ``sample_count x count`` in the case's declared
    order, matching the steering actuator layout.
    """

    names: tuple[str, ...]
    kind: np.ndarray
    body: np.ndarray
    reaction_body: np.ndarray
    point_local: np.ndarray
    reaction_point_local: np.ndarray
    axis_local: np.ndarray
    reference_quaternion: np.ndarray
    target: np.ndarray
    target_rate: np.ndarray


@dataclass(frozen=True)
class _VehicleRoadBuffers:
    kind: int
    origin_x: float
    origin_z: float
    amplitude: float
    wavelength: float
    phase: float
    bump_start: float
    bump_length: float
    corner_scale: np.ndarray


@dataclass(frozen=True)
class _NativeRun:
    result: AxleDynamicsResult
    steering_output: np.ndarray | None = None
    kernel_wall_time_s: float = 0.0


def _ptr(array: np.ndarray, ctype: type[ctypes.c_double] | type[ctypes.c_int]):
    return array.ctypes.data_as(ctypes.POINTER(ctype))


#: Mirror of the C ABI's uniform element block.  The parameter and integer widths
#: are the ABI's (`kElementBlockSize` / `kElementIntBlockSize`); a mirror that is
#: narrower than the C structure cannot address the later families' slots.
class _ElementBlock(ctypes.Structure):
    _fields_ = [
        ("kind", ctypes.c_int),
        ("flags", ctypes.c_int),
        ("body_a", ctypes.c_int),
        ("body_b", ctypes.c_int),
        ("parameters", ctypes.c_double * 176),
        ("ints", ctypes.c_int * 16),
        # Parameter cache for families larger than the uniform block, such as a
        # tire's 226-entry PAC2002 table.  Both stay at zero for the families that
        # fit, and the reader rejects a count without a pointer.
        ("cached_parameters", ctypes.c_void_p),
        ("cached_parameter_count", ctypes.c_size_t),
    ]


class _ElementCurveReference(ctypes.Structure):
    _fields_ = [
        ("values", ctypes.c_void_p),
        ("count", ctypes.c_size_t),
    ]


#: Element kinds, mirroring `enum ElementKind` in the kernel header.
ELEMENT_SPRING = 0
ELEMENT_BUSHING = 1
ELEMENT_ANTI_ROLL = 2
ELEMENT_TIRE = 3
ELEMENT_AERODYNAMIC_DRAG = 4

#: Curve slots per element, mirroring `kElementCurveSlots`.
ELEMENT_CURVE_SLOTS = 8


def _apply_element_blocks(
    target: object, element_blocks: tuple[_ElementBlock, ...]
) -> tuple[object, object]:
    """
    Point a native input structure at the given element blocks.

    Clears the per-family element counts as well: the kernel rejects a caller that
    supplies both forms, so the two must not be left set together.  Returns the
    block and curve arrays, which the caller must keep alive because the structure
    holds raw pointers into them.

    Shared by both entry points: `AxleInput` and `VehicleInput` carry the same
    fields, which is the point of the generic surface.
    """
    curves = (_ElementCurveReference * (len(element_blocks) * ELEMENT_CURVE_SLOTS))()
    blocks = (_ElementBlock * len(element_blocks))(*element_blocks)
    target.spring_count = 0  # type: ignore[attr-defined]
    target.bushing_count = 0  # type: ignore[attr-defined]
    target.anti_roll_bar_count = 0  # type: ignore[attr-defined]
    target.tire_count = 0  # type: ignore[attr-defined]
    target.element_count = len(element_blocks)  # type: ignore[attr-defined]
    target.elements = ctypes.cast(blocks, ctypes.c_void_p).value  # type: ignore[attr-defined]
    if hasattr(target, "aerodynamic_drag_count"):
        # The one element family the vehicle structure owns; see
        # `add_vehicle_aerodynamic_drags`, which applies the same rule.
        target.aerodynamic_drag_count = 0
    target.element_curves = ctypes.cast(curves, ctypes.c_void_p).value  # type: ignore[attr-defined]
    target.topology_extension_count = 0  # type: ignore[attr-defined]
    target.topology_extensions = None  # type: ignore[attr-defined]
    return blocks, curves


def _spring_element_block(
    *,
    body_a: int,
    body_b: int,
    stiffness: float,
    compression_damping: float,
    rebound_damping: float,
    free_length: float,
    minimum_length: float = math.nan,
    maximum_length: float = math.nan,
    compression_stop_stiffness: float = 0.0,
    compression_stop_damping: float = 0.0,
    rebound_stop_stiffness: float = 0.0,
    rebound_stop_damping: float = 0.0,
    point_a: tuple[float, float, float] = (0.0, 0.0, 0.0),
    point_b: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> _ElementBlock:
    """
    Build one SPRING element block.

    The parameter offsets mirror `enum ElementParameter` in the kernel header; a
    mismatch is caught by the array-versus-block equivalence test, which asserts
    the two spellings of a model produce identical results.
    """
    block = _ElementBlock()
    block.kind = ELEMENT_SPRING
    block.body_a = body_a
    block.body_b = body_b
    parameters = block.parameters
    parameters[0] = stiffness
    parameters[1] = compression_damping
    parameters[2] = rebound_damping
    parameters[3] = free_length
    parameters[4] = minimum_length
    parameters[5] = maximum_length
    parameters[6] = compression_stop_stiffness
    parameters[7] = compression_stop_damping
    parameters[8] = rebound_stop_stiffness
    parameters[9] = rebound_stop_damping
    for offset, point in ((10, point_a), (13, point_b)):
        parameters[offset] = point[0]
        parameters[offset + 1] = point[1]
        parameters[offset + 2] = point[2]
    return block


def _library_path() -> Path:
    """
    Return the path of this package's copy of the kernel library.

    Path decoration (prefix and extension per platform) and the load itself are
    owned by `suspension_kernel.binding`; this function only says *where* the
    axle package keeps its copy.
    """
    return kernel_library_path(_LIBRARY_STEM, _NATIVE_DIR)


def native_build_metadata() -> dict[str, object]:
    """
    Return the recorded compiler and ABI metadata.

    Delegates to the kernel binding, which fails explicitly when the file is
    absent instead of returning an empty mapping.
    """
    return kernel_build_metadata(_NATIVE_DIR)


def _require_matching_metadata_versions() -> None:
    """
    Assert that the module's ABI constants agree with the recorded metadata.

    Both numbers describe the same installed library, so a disagreement means one
    of the two got bumped without the other.  Failing here keeps the mismatch at
    the boundary, where it is named, instead of letting a run read the wrong
    struct layout.
    """
    metadata = native_build_metadata()
    for key, expected in (
        ("abi_version", _NATIVE_KERNEL_ABI_VERSION),
        ("vehicle_abi_version", _NATIVE_VEHICLE_KERNEL_ABI_VERSION),
        ("core_abi_version", _NATIVE_CORE_ABI_VERSION),
    ):
        observed = metadata.get(key)
        if observed != expected:
            raise KernelAbiMismatchError(key, expected, observed)


def _canonical_kernel_library() -> Path:
    """
    Return the kernel package's own copy of the built library.

    Mirrored by :func:`_require_fresh_mirror`; kept as a separate function so the
    mirror policy is stated once.  `suspension_kernel.binding` owns the location,
    so the two packages cannot disagree about where the canonical copy lives.
    """
    return kernel_native_directory() / _library_path().name


def _require_fresh_mirror() -> None:
    """
    Refuse to load a mirror that is older *and* different from its source.

    This package loads its own copy under `native/`, but the build that produces
    the canonical library lives in `suspension_kernel`.  A build that refreshed
    only the canonical copy leaves a stale library here, and a stale library
    against a fresh `ctypes` mirror shows up as an access violation inside the
    kernel rather than as the build problem it is.

    The comparison is on content, not only on time: a rebuild that reproduces the
    same bytes (the usual case, since the kernel is built reproducibly) must not
    be reported as staleness.  Time only orders the two copies so the check costs
    one file comparison instead of two on every load.
    """
    mirror = _library_path()
    canonical = _canonical_kernel_library()
    if not canonical.is_file() or not mirror.is_file():
        # A wheel installation has no kernel package beside it, which is fine.
        return
    if canonical.stat().st_mtime <= mirror.stat().st_mtime:
        return
    if _same_content(mirror, canonical):
        return
    raise NativeKernelUnavailableError(
        f"the native kernel mirror at {mirror} is older and different from "
        f"{canonical}; run `just build-axle-native` (or `just build-kernel` "
        "followed by `just build-axle-native`) so the axle package loads the "
        "library it was built against"
    )


def _same_content(left: Path, right: Path) -> bool:
    """Return whether two files hold identical bytes."""
    if left.stat().st_size != right.stat().st_size:
        return False
    with left.open("rb") as first, right.open("rb") as second:
        while True:
            a = first.read(1 << 20)
            b = second.read(1 << 20)
            if a != b:
                return False
            if not a:
                return True


def _load_library() -> ctypes.CDLL:
    """
    Load the kernel library, probing symbols and gating both ABI versions.

    The symbol probe and the ABI gate live in `suspension_kernel.binding`; what
    stays here is the axle-specific arity and pointer typing of each symbol,
    which needs this module's `ctypes.Structure` mirrors.
    """
    path = _library_path()
    if not path.exists():
        raise NativeKernelUnavailableError(
            f"native axle kernel is unavailable at {path}; "
            "run packages/suspension_multibody/scripts/build_axle_native.ps1"
        )
    _require_fresh_mirror()
    try:
        library = kernel_load_library(
            stem=_LIBRARY_STEM,
            directory=_NATIVE_DIR,
            abi_symbols={
                "axle_kernel_abi_version": _NATIVE_KERNEL_ABI_VERSION,
                "vehicle_kernel_abi_version": _NATIVE_VEHICLE_KERNEL_ABI_VERSION,
            },
            required_symbols=("axle_run", "vehicle_run"),
        ).handle
    except KernelSymbolMissingError as error:
        raise NativeKernelUnavailableError(str(error)) from error
    except KernelAbiMismatchError as error:
        raise NativeKernelUnavailableError(str(error)) from error
    library.axle_run.argtypes = [
        ctypes.POINTER(_AxleInput),
        ctypes.POINTER(_AxleOutput),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    library.axle_run.restype = ctypes.c_int
    library.vehicle_run.argtypes = [
        ctypes.POINTER(_VehicleInput),
        ctypes.POINTER(_VehicleOutput),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    library.vehicle_run.restype = ctypes.c_int
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


#: Driven-coordinate kind -> AxleConstraintType value in the C++ enum.
_DRIVEN_KIND = {"translation": 8, "rotation": 9}

#: Steering actuator types that prescribe a coordinate instead of applying a force.
_PRESCRIBED_STEERING_TYPES = (2, 3)


def _prescribed_steering_indices(
    steering: _VehicleSteeringBuffers | None,
) -> tuple[int, ...]:
    """
    Return the indices of the actuators that became driven coordinates.

    A prescribed actuator no longer owns a private constraint row in the kernel: it
    registers an ``AXLE_DRIVEN_*`` coordinate, which takes a slot in the
    constraint-wrench output.  The order matters because the kernel appends those
    constraints before the case's own driven coordinates.
    """
    if steering is None:
        return ()
    return tuple(
        index
        for index, kind in enumerate(np.asarray(steering.actuator_type).ravel())
        if int(kind) in _PRESCRIBED_STEERING_TYPES
    )


def _driven_buffers(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
) -> _VehicleDrivenBuffers | None:
    """
    Assemble the driven-coordinate buffers the versioned ABI expects.

    Geometry comes from the model and targets from the case, one row per sample in
    the model's declared coordinate order.  A missing target is an error rather
    than a silent zero: a driven coordinate with no target would pin a degree of
    freedom at the assembling pose and look like a plausible run.
    """
    coordinates = model.driven_coordinates
    if not coordinates:
        return None
    samples = len(case.times_s)
    times = np.asarray(case.times_s, dtype=np.float64)
    names = tuple(coordinate.name for coordinate in coordinates)
    unknown = set(case.driven_target_m) - set(names)
    if unknown:
        raise ValueError(
            f"driven targets reference unknown coordinates: {sorted(unknown)}"
        )
    target_rows: list[np.ndarray] = []
    rate_rows: list[np.ndarray] = []
    for coordinate in coordinates:
        values = case.driven_target_m.get(coordinate.name)
        rate_values = case.driven_target_rate.get(coordinate.name)
        if values is None and rate_values is None:
            raise ValueError(
                f"driven coordinate {coordinate.name!r} has neither a target nor "
                "a rate in the case"
            )
        if values is None:
            # Rate-level request: the caller prescribes how fast the coordinate
            # moves and the displacement is its integral from the assembling pose.
            # The row itself stays position-level, so the coordinate still tracks
            # the integral exactly instead of being allowed to drift; a true
            # velocity-level DAE row is a separate, larger change (its position
            # Jacobian needs dJ/dq * q_dot, which only the directional Jacobian
            # assembly forms today).
            rate = np.asarray(rate_values, dtype=np.float64)
            if rate.shape != (samples,):
                raise ValueError(
                    f"driven target rate {coordinate.name!r} must match times_s"
                )
            target = np.concatenate(
                ([0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(times)))
            )
        else:
            target = np.asarray(values, dtype=np.float64)
            if target.shape != (samples,):
                raise ValueError(
                    f"driven target {coordinate.name!r} length must match times_s"
                )
            if rate_values is None:
                # Every velocity- and acceleration-level row needs the explicit time
                # derivative of the target, so derive it here once instead of making
                # each caller supply it.
                rate = np.gradient(target, times) if samples > 1 else np.zeros(1)
            else:
                rate = np.asarray(rate_values, dtype=np.float64)
                if rate.shape != (samples,):
                    raise ValueError(
                        f"driven target rate {coordinate.name!r} must match times_s"
                    )
        target_rows.append(target)
        rate_rows.append(rate)
    return _VehicleDrivenBuffers(
        names=names,
        kind=np.ascontiguousarray(
            [_DRIVEN_KIND[coordinate.kind] for coordinate in coordinates],
            dtype=np.int32,
        ),
        body=np.ascontiguousarray(
            [_body_index(model, coordinate.body) for coordinate in coordinates],
            dtype=np.int32,
        ),
        reaction_body=np.ascontiguousarray(
            [
                _body_index(model, coordinate.reaction_body)
                for coordinate in coordinates
            ],
            dtype=np.int32,
        ),
        point_local=np.ascontiguousarray(
            [coordinate.point_local_m for coordinate in coordinates],
            dtype=np.float64,
        ),
        reaction_point_local=np.ascontiguousarray(
            [coordinate.reaction_point_local_m for coordinate in coordinates],
            dtype=np.float64,
        ),
        axis_local=np.ascontiguousarray(
            [coordinate.axis_local for coordinate in coordinates],
            dtype=np.float64,
        ),
        reference_quaternion=np.ascontiguousarray(
            [coordinate.reference_quaternion for coordinate in coordinates],
            dtype=np.float64,
        ),
        target=np.ascontiguousarray(np.stack(target_rows, axis=1)),
        target_rate=np.ascontiguousarray(np.stack(rate_rows, axis=1)),
    )


def _body_index(model: AxleDynamicsModel, name: str) -> int:
    for index, body in enumerate(model.bodies):
        if body.name == name:
            return index
    raise ValueError(f"unknown body {name!r}")


def _run_native(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    *,
    steering: _VehicleSteeringBuffers | None = None,
    driven: _VehicleDrivenBuffers | None = None,
    road: _VehicleRoadBuffers | None = None,
    brake_torque: dict[str, tuple[float, ...]] | None = None,
    static_gauge_body: str | None = None,
    static_gauge_dof_mask: int = 0,
    static_trim_then_release: bool = False,
    initial_state_angle_tolerance_rad: float | None = None,
    static_rotation_gauges: tuple[
        tuple[str, tuple[float, float, float]], ...
    ] = (),
    element_blocks: tuple[_ElementBlock, ...] = (),
) -> _NativeRun:
    """
    Run one validated SI axle case through the native C++ kernel.

    `element_blocks`, when given, is the generic element surface: the per-family
    arrays are then cleared so the kernel reads the blocks instead.  Both forms
    describe the same model, which is what the array-versus-block equivalence test
    asserts.
    """
    # Arrays whose lifetime must outlive the call.  The element surface needs the
    # curve references to stay alive alongside the blocks themselves, so both are
    # parked here rather than allocated inline at the construction site.
    _element_block_array: object | None = None
    _element_curve_array: object | None = None
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
        "constant_velocity": 7,
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
    constraint_axis_a_secondary = np.ascontiguousarray(
        [joint.axis_a_secondary for joint in model.joints], dtype=np.float64
    )
    constraint_axis_b_secondary = np.ascontiguousarray(
        [joint.axis_b_secondary for joint in model.joints], dtype=np.float64
    )
    constraint_convel_angle_target = np.ascontiguousarray(
        [joint.constant_velocity_angle_target for joint in model.joints],
        dtype=np.float64,
    )
    joint_index = {joint.name: index for index, joint in enumerate(model.joints)}
    coordinate_coupler_joint_a = np.ascontiguousarray(
        [joint_index[coupler.joint_a] for coupler in model.coordinate_couplers],
        dtype=np.int32,
    )
    coordinate_coupler_coordinate_a = np.ascontiguousarray(
        [0 if coupler.coordinate_a == "rotation" else 1
         for coupler in model.coordinate_couplers],
        dtype=np.int32,
    )
    coordinate_coupler_scale_a = np.ascontiguousarray(
        [coupler.scale_a for coupler in model.coordinate_couplers],
        dtype=np.float64,
    )
    coordinate_coupler_joint_b = np.ascontiguousarray(
        [joint_index[coupler.joint_b] for coupler in model.coordinate_couplers],
        dtype=np.int32,
    )
    coordinate_coupler_coordinate_b = np.ascontiguousarray(
        [0 if coupler.coordinate_b == "rotation" else 1
         for coupler in model.coordinate_couplers],
        dtype=np.int32,
    )
    coordinate_coupler_scale_b = np.ascontiguousarray(
        [coupler.scale_b for coupler in model.coordinate_couplers],
        dtype=np.float64,
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

    def _flatten_length_force_curves(
        curves: list[tuple[tuple[float, float], ...]]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        offsets: list[int] = []
        counts: list[int] = []
        abscissa: list[float] = []
        force: list[float] = []
        for curve in curves:
            offsets.append(len(abscissa))
            counts.append(len(curve))
            abscissa.extend(float(item[0]) for item in curve)
            force.extend(float(item[1]) for item in curve)
        return (
            np.ascontiguousarray(offsets or [0], dtype=np.int32),
            np.ascontiguousarray(counts or [0], dtype=np.int32),
            np.ascontiguousarray(abscissa or [0.0], dtype=np.float64),
            np.ascontiguousarray(force or [0.0], dtype=np.float64),
        )

    (
        elastic_curve_offset,
        elastic_curve_count,
        elastic_curve_deflection,
        elastic_curve_force,
    ) = _flatten_length_force_curves(
        [
            tuple(
                zip(
                    spring.elastic_curve_deflection_m,
                    spring.elastic_curve_force_n,
                )
            )
            for spring in model.springs
        ]
    )
    (
        compression_stop_curve_offset,
        compression_stop_curve_count,
        compression_stop_curve_penetration,
        compression_stop_curve_force,
    ) = _flatten_length_force_curves(
        [
            tuple(
                zip(
                    spring.compression_stop_curve_penetration_m,
                    spring.compression_stop_curve_force_n,
                )
            )
            for spring in model.springs
        ]
    )
    (
        rebound_stop_curve_offset,
        rebound_stop_curve_count,
        rebound_stop_curve_penetration,
        rebound_stop_curve_force,
    ) = _flatten_length_force_curves(
        [
            tuple(
                zip(
                    spring.rebound_stop_curve_penetration_m,
                    spring.rebound_stop_curve_force_n,
                )
            )
            for spring in model.springs
        ]
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
    bushing_rotation_coordinates = np.ascontiguousarray(
        [
            1 if bushing.rotation_coordinates == "cardan_xyz" else 0
            for bushing in model.bushings
        ],
        dtype=np.int32,
    )
    bushing_curve_offset: list[int] = []
    bushing_curve_count: list[int] = []
    bushing_curve_coordinate: list[float] = []
    bushing_curve_force: list[float] = []
    for bushing in model.bushings:
        curves = bushing.force_curves or ((),) * 6
        if len(curves) != 6:
            raise ValueError("bushing force_curves must contain six axis curves")
        for curve in curves:
            bushing_curve_offset.append(len(bushing_curve_coordinate))
            bushing_curve_count.append(len(curve))
            bushing_curve_coordinate.extend(item[0] for item in curve)
            bushing_curve_force.extend(item[1] for item in curve)
    vehicle_bushing_curve_offset = np.ascontiguousarray(
        bushing_curve_offset or [0], dtype=np.int32
    )
    vehicle_bushing_curve_count = np.ascontiguousarray(
        bushing_curve_count or [0], dtype=np.int32
    )
    vehicle_bushing_curve_coordinate = np.ascontiguousarray(
        bushing_curve_coordinate or [0.0], dtype=np.float64
    )
    vehicle_bushing_curve_force = np.ascontiguousarray(
        bushing_curve_force or [0.0], dtype=np.float64
    )
    bushing_force_curve_interpolation = np.ascontiguousarray(
        [
            1 if bushing.force_curve_interpolation == "akima" else 0
            for bushing in model.bushings
        ],
        dtype=np.int32,
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
    tire_frame_body = np.ascontiguousarray(
        [body_index[tire.frame_body or tire.body] for tire in model.tires],
        dtype=np.int32,
    )
    tire_frame_center = np.ascontiguousarray(
        [
            (
                tire.frame_center_local_m
                if tire.frame_center_local_m is not None
                else tire.center_local_m
            )
            for tire in model.tires
        ],
        dtype=np.float64,
    )
    tire_drive_torque_body = np.ascontiguousarray(
        [
            body_index[tire.drive_torque_body]
            if tire.drive_torque_body is not None
            else -1
            for tire in model.tires
        ],
        dtype=np.int32,
    )
    tire_drive_torque_reaction_body = np.ascontiguousarray(
        [
            body_index[tire.drive_torque_reaction_body]
            if tire.drive_torque_reaction_body is not None
            else -1
            for tire in model.tires
        ],
        dtype=np.int32,
    )
    tire_drive_torque_axis_local = np.ascontiguousarray(
        [
            tire.drive_torque_axis_local
            if tire.drive_torque_axis_local is not None
            else (0.0, 0.0, 0.0)
            for tire in model.tires
        ],
        dtype=np.float64,
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
    deflection_rows: list[tuple[float, float]] = []
    deflection_offset: list[int] = []
    deflection_count: list[int] = []
    for tire in model.tires:
        rows = tuple(
            getattr(tire, "pac2002_tables", {}).get("deflection_load_curve", ())
        )
        deflection_offset.append(len(deflection_rows))
        deflection_count.append(len(rows))
        deflection_rows.extend((float(d), float(f)) for d, f in rows)
    if os.environ.get("SUSPENSION_AXLE_CURVE_TRACE"):
        print(
            f"[curve-trace] tires={len(model.tires)} counts={deflection_count} "
            f"rows={len(deflection_rows)} tables={[getattr(t, 'pac2002_tables', None) for t in model.tires[:1]]}",
            file=sys.stderr,
        )
    bottoming_rows: list[tuple[float, float]] = []
    bottoming_offset: list[int] = []
    bottoming_count: list[int] = []
    for tire in model.tires:
        rows = tuple(
            getattr(tire, "pac2002_tables", {}).get("bottoming_curve", ())
        )
        bottoming_offset.append(len(bottoming_rows))
        bottoming_count.append(len(rows))
        bottoming_rows.extend((float(p), float(f)) for p, f in rows)
    tire_bottoming_curve_offset = np.ascontiguousarray(
        bottoming_offset, dtype=np.int32
    )
    tire_bottoming_curve_count = np.ascontiguousarray(
        bottoming_count, dtype=np.int32
    )
    tire_bottoming_curve_penetration = np.ascontiguousarray(
        [row[0] for row in bottoming_rows], dtype=np.float64
    )
    tire_bottoming_curve_force = np.ascontiguousarray(
        [row[1] for row in bottoming_rows], dtype=np.float64
    )
    tire_deflection_curve_offset = np.ascontiguousarray(
        deflection_offset, dtype=np.int32
    )
    tire_deflection_curve_count = np.ascontiguousarray(
        deflection_count, dtype=np.int32
    )
    tire_deflection_curve_deflection = np.ascontiguousarray(
        [row[0] for row in deflection_rows], dtype=np.float64
    )
    tire_deflection_curve_force = np.ascontiguousarray(
        [row[1] for row in deflection_rows], dtype=np.float64
    )
    aerodynamic_drags = tuple(getattr(model, "aerodynamic_drags", ()))
    aerodynamic_drag_body = np.ascontiguousarray(
        [body_index[drag.body] for drag in aerodynamic_drags], dtype=np.int32
    )
    aerodynamic_drag_application_point = np.ascontiguousarray(
        [drag.application_point_m for drag in aerodynamic_drags], dtype=np.float64
    )
    aerodynamic_drag_forward_axis = np.ascontiguousarray(
        [drag.forward_axis_local for drag in aerodynamic_drags], dtype=np.float64
    )
    aerodynamic_drag_coefficient = np.ascontiguousarray(
        [drag.coefficient_n_s2_per_m2 for drag in aerodynamic_drags],
        dtype=np.float64,
    )
    tire_model_kind = np.ascontiguousarray(
        [
            (
                (
                    2
                    if (
                        tire.model_kind == "pac2002_pure_slip"
                        and getattr(tire, "pac2002_parameter_source", "user")
                        == "adams_builtin"
                    )
                    else 1
                    if tire.model_kind == "pac2002_pure_slip"
                    else 3
                    if tire.model_kind == "fiala"
                    else 0
                )
            )
            for tire in model.tires
        ],
        dtype=np.int32,
    )
    tire_pac2002_parameters = np.ascontiguousarray(
        [
            [
                float(
                    tire.pac2002_coefficients.get(
                        name, PAC2002_PARAMETER_DEFAULTS[name]
                    )
                )
                for name in PAC2002_PARAMETER_NAMES
            ]
            for tire in model.tires
        ],
        dtype=np.float64,
    )
    for i, tire in enumerate(model.tires):
        if tire.model_kind == "fiala":
            fiala = tire.fiala_parameters
            fiala_values = (
                fiala.get("CSLIP", 1000.0),
                fiala.get("CALPHA", 800.0),
                # Adams' Fiala property format defines CGAMMA but its handling
                # force model ignores it ("Camber angle has no effect on tire
                # forces"), so the slot is carried for fidelity and never read.
                fiala.get("CGAMMA", 0.0),
                # Reserved.  MGAMMA and the two damping slots are not keywords of
                # the Adams Fiala format at all; keeping them at zero preserves the
                # 14-slot layout that the kernel's FialaParameterIndex mirrors.
                0.0,
                # Slot 4 now carries the tire's [MODEL] USE_MODE.  It used to hold
                # CSPIN, which the Adams Fiala format does not define, so the slot
                # was never read.  Selects startup smoothing (2, 12) and the slip
                # transient (11, 12) -- see fiala_use_mode() in the kernel.
                fiala.get("USE_MODE", 2.0),
                fiala.get("UMIN", 0.9),
                fiala.get("UMAX", 1.0),
                fiala.get("RELAX_LENGTH_X", tire.longitudinal_relaxation_length_m),
                fiala.get("RELAX_LENGTH_Y", tire.lateral_relaxation_length_m),
                fiala.get("WIDTH", 0.235),
                fiala.get("ROLLING_RESISTANCE", 0.0),
                fiala.get("LOW_SPEED_THRESHOLD", 1.0e-3),
                0.0,
                0.0,
            )
            tire_pac2002_parameters[i, :len(fiala_values)] = fiala_values
    tire_pac2002_mirror = np.ascontiguousarray(
        [
            int(
                tire.model_kind == "pac2002_pure_slip"
                and getattr(tire, "pac2002_parameter_source", "user")
                == "adams_builtin"
                and (
                    (
                        tire.pac2002_mirror
                        if tire.pac2002_mirror is not None
                        else _is_right_tire_name(tire.name)
                    )
                    or tire.pac2002_coefficients.get("USE_MODE", 14.0) < 0.0
                )
            )
            for tire in model.tires
        ],
        dtype=np.int32,
    )
    times = np.ascontiguousarray(case.times_s, dtype=np.float64)
    body_wrench = _body_wrench_matrix(case, body_names)
    road_z = _signal_matrix(case, tire_names, case.road_height_m)
    road_v = _signal_matrix(case, tire_names, case.road_velocity_m_per_s)
    wheel_torque = _signal_matrix(case, tire_names, case.wheel_torque_n_m)
    brake_torque_matrix = _signal_matrix(
        case, tire_names, {} if brake_torque is None else brake_torque
    )

    states = np.full(
        (len(times), len(body_names), 19), np.nan, dtype=np.float64
    )
    # The kernel writes one wrench per *constraint*, and a driven coordinate takes a
    # slot exactly like a joint: its reaction is the drive force/torque that holds
    # the prescribed coordinate, which is what a rig wants to compare.  A prescribed
    # steering actuator is de-specialized into a driven coordinate too, so it needs
    # a slot as well -- and the kernel appends steering constraints before the
    # case's own driven coordinates, which fixes the name order below.
    prescribed_steering = _prescribed_steering_indices(steering)
    constraint_wrench = np.full(
        (
            len(times),
            len(model.joints)
            + len(model.driven_coordinates)
            + len(prescribed_steering),
            6,
        ),
        np.nan,
        dtype=np.float64,
    )
    # The solver sizes its component channels from the model, so the buffers have
    # to match the elements that will actually be built.  When the elements arrive
    # as blocks the model's per-family lists are empty until the kernel reads them,
    # so the counts are taken from the blocks instead: sizing by `len(model.springs)`
    # in that case produced zero-length buffers and the kernel rejected the run.
    spring_count = len(model.springs) + sum(
        1 for block in element_blocks if block.kind == ELEMENT_SPRING
    )
    bushing_count = len(model.bushings) + sum(
        1 for block in element_blocks if block.kind == ELEMENT_BUSHING
    )
    spring_output = np.full(
        (len(times), spring_count, 7), np.nan, dtype=np.float64
    )
    bushing_output = np.full(
        (len(times), bushing_count, 12), np.nan, dtype=np.float64
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
        (len(times), len(tire_names), len(TIRE_OUTPUT_COLUMNS)),
        np.nan,
        dtype=np.float64,
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
    axle_input = _AxleInput(
        ctypes.sizeof(_AxleInput),
        _NATIVE_KERNEL_ABI_VERSION,
        0,
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
    if element_blocks:
        # The generic element surface replaces the per-family arrays: clearing
        # their counts is what makes the kernel read the blocks instead of both.
        # Every array this clears becomes a zero-length view, so the kernel
        # iterates nothing and the model's elements come from the blocks alone.
        block_array, curve_array = _apply_element_blocks(axle_input, element_blocks)
        # Keep both arrays alive past the call: the kernel holds raw pointers.
        _element_block_array = block_array
        _element_curve_array = curve_array
    if driven is None:
        driven = _driven_buffers(model, case)
    vehicle_mode = (
        steering is not None
        or driven is not None
        or road is not None
        or brake_torque is not None
        or static_gauge_body is not None
        or static_gauge_dof_mask != 0
        or bool(static_rotation_gauges)
        or bool(model.coordinate_couplers)
        or np.any(bushing_rotation_coordinates != 0)
    )
    if not vehicle_mode and any(
        joint.kind == "constant_velocity" for joint in model.joints
    ):
        raise ValueError(
            "constant_velocity joints require the versioned vehicle native interface"
        )
    has_vehicle_force_curves = any(
        spring.elastic_curve_deflection_m
        or spring.compression_stop_curve_penetration_m
        or spring.rebound_stop_curve_penetration_m
        for spring in model.springs
    ) or any(bushing.force_curves for bushing in model.bushings)
    if has_vehicle_force_curves and not vehicle_mode:
        raise ValueError(
            "elastic and stop force curves require the versioned vehicle native interface"
        )
    if not vehicle_mode and np.any(tire_model_kind != 0):
        raise ValueError(
            "pac2002_pure_slip requires the versioned vehicle native interface"
        )
    road_input = road
    if road_input is None:
        road_input = _VehicleRoadBuffers(
            kind=0,
            origin_x=0.0,
            origin_z=0.0,
            amplitude=0.0,
            wavelength=1.0,
            phase=0.0,
            bump_start=0.0,
            bump_length=1.0,
            corner_scale=np.ones(4, dtype=np.float64),
        )
    native_input: _AxleInput | _VehicleInput = axle_input
    # `_VehicleInput` embeds `_AxleInput` by value, so the element fields set on
    # `axle_input` above do not travel with the copy that `_VehicleInput(...)`
    # makes further down.  The vehicle entry point reads its own copy, and the
    # block arrays must therefore be re-pointed on it -- but only once it exists,
    # which is why the addresses are captured here and transferred below.
    # Applying them to `axle_input` before the copy, as an earlier version did, is
    # a no-op: the copy takes the fields and the arrays never follow.
    carried_element_fields: dict[str, object] = {}
    if element_blocks:
        carried_element_fields = {
            "element_count": axle_input.element_count,
            "elements": axle_input.elements,
            "element_curves": axle_input.element_curves,
            "topology_extension_count": axle_input.topology_extension_count,
            "topology_extensions": axle_input.topology_extensions,
        }
    if vehicle_mode:
        if static_gauge_dof_mask < 0 or static_gauge_dof_mask & ~0x3F:
            raise ValueError("static_gauge_dof_mask must use pose bits 0 through 5")
        if static_gauge_body is None:
            if static_gauge_dof_mask != 0:
                raise ValueError(
                    "static_gauge_body is required when static gauge bits are set"
                )
            static_gauge_body_index = 0
        else:
            try:
                static_gauge_body_index = body_index[static_gauge_body]
            except KeyError as exc:
                raise ValueError(
                    f"static gauge references unknown body {static_gauge_body!r}"
                ) from exc
        static_rotation_gauge_body = np.ascontiguousarray(
            [body_index[name] for name, _ in static_rotation_gauges],
            dtype=np.int32,
        )
        static_rotation_gauge_axis_local = np.ascontiguousarray(
            [axis for _, axis in static_rotation_gauges],
            dtype=np.float64,
        ).reshape((-1, 3))
        steering_count = 0 if steering is None else len(steering.names)
        native_input = _VehicleInput(
            ctypes.sizeof(_VehicleInput),
            _NATIVE_VEHICLE_KERNEL_ABI_VERSION,
            0,
            axle_input,
            steering_count,
            (None if steering is None else _ptr(steering.actuator_type, ctypes.c_int)),
            (None if steering is None else _ptr(steering.body, ctypes.c_int)),
            (None if steering is None else _ptr(steering.reaction_body, ctypes.c_int)),
            (None if steering is None else _ptr(steering.point_local, ctypes.c_double)),
            (None if steering is None else _ptr(steering.reaction_point_local, ctypes.c_double)),
            (None if steering is None else _ptr(steering.axis_local, ctypes.c_double)),
            (None if steering is None else _ptr(steering.reference_quaternion, ctypes.c_double)),
            (None if steering is None else _ptr(steering.target, ctypes.c_double)),
            (None if steering is None else _ptr(steering.target_rate, ctypes.c_double)),
            (None if steering is None else _ptr(steering.stiffness, ctypes.c_double)),
            (None if steering is None else _ptr(steering.damping, ctypes.c_double)),
            road_input.kind,
            road_input.origin_x,
            road_input.origin_z,
            road_input.amplitude,
            road_input.wavelength,
            road_input.phase,
            road_input.bump_start,
            road_input.bump_length,
            _ptr(road_input.corner_scale, ctypes.c_double),
            _ptr(brake_torque_matrix, ctypes.c_double),
            static_gauge_body_index,
            static_gauge_dof_mask,
            int(static_trim_then_release),
            _ptr(tire_frame_body, ctypes.c_int),
            _ptr(tire_frame_center, ctypes.c_double),
            _ptr(tire_model_kind, ctypes.c_int),
            _ptr(tire_pac2002_parameters, ctypes.c_double),
            _ptr(tire_pac2002_mirror, ctypes.c_int),
            _ptr(elastic_curve_offset, ctypes.c_int),
            _ptr(elastic_curve_count, ctypes.c_int),
            _ptr(elastic_curve_deflection, ctypes.c_double),
            _ptr(elastic_curve_force, ctypes.c_double),
            _ptr(compression_stop_curve_offset, ctypes.c_int),
            _ptr(compression_stop_curve_count, ctypes.c_int),
            _ptr(compression_stop_curve_penetration, ctypes.c_double),
            _ptr(compression_stop_curve_force, ctypes.c_double),
            _ptr(rebound_stop_curve_offset, ctypes.c_int),
            _ptr(rebound_stop_curve_count, ctypes.c_int),
            _ptr(rebound_stop_curve_penetration, ctypes.c_double),
            _ptr(rebound_stop_curve_force, ctypes.c_double),
            _ptr(vehicle_bushing_curve_offset, ctypes.c_int),
            _ptr(vehicle_bushing_curve_count, ctypes.c_int),
            _ptr(vehicle_bushing_curve_coordinate, ctypes.c_double),
            _ptr(vehicle_bushing_curve_force, ctypes.c_double),
            _ptr(constraint_axis_a_secondary, ctypes.c_double),
            _ptr(constraint_axis_b_secondary, ctypes.c_double),
            _ptr(constraint_convel_angle_target, ctypes.c_double),
            len(static_rotation_gauges),
            _ptr(static_rotation_gauge_body, ctypes.c_int),
            _ptr(static_rotation_gauge_axis_local, ctypes.c_double),
            (
                settings.local_angle_tolerance_rad
                if initial_state_angle_tolerance_rad is None
                else initial_state_angle_tolerance_rad
            ),
            _ptr(bushing_rotation_coordinates, ctypes.c_int),
            len(model.coordinate_couplers),
            _ptr(coordinate_coupler_joint_a, ctypes.c_int),
            _ptr(coordinate_coupler_coordinate_a, ctypes.c_int),
            _ptr(coordinate_coupler_scale_a, ctypes.c_double),
            _ptr(coordinate_coupler_joint_b, ctypes.c_int),
            _ptr(coordinate_coupler_coordinate_b, ctypes.c_int),
            _ptr(coordinate_coupler_scale_b, ctypes.c_double),
            len(aerodynamic_drags),
            _ptr(aerodynamic_drag_body, ctypes.c_int),
            _ptr(aerodynamic_drag_application_point, ctypes.c_double),
            _ptr(aerodynamic_drag_forward_axis, ctypes.c_double),
            _ptr(aerodynamic_drag_coefficient, ctypes.c_double),
            _ptr(tire_drive_torque_body, ctypes.c_int),
            _ptr(tire_drive_torque_reaction_body, ctypes.c_int),
            _ptr(tire_drive_torque_axis_local, ctypes.c_double),
            _ptr(bushing_force_curve_interpolation, ctypes.c_int),
            _ptr(tire_deflection_curve_offset, ctypes.c_int),
            _ptr(tire_deflection_curve_count, ctypes.c_int),
            _ptr(tire_deflection_curve_deflection, ctypes.c_double),
            _ptr(tire_deflection_curve_force, ctypes.c_double),
            _ptr(tire_bottoming_curve_offset, ctypes.c_int),
            _ptr(tire_bottoming_curve_count, ctypes.c_int),
            _ptr(tire_bottoming_curve_penetration, ctypes.c_double),
            _ptr(tire_bottoming_curve_force, ctypes.c_double),
            0 if driven is None else len(driven.names),
            None if driven is None else _ptr(driven.kind, ctypes.c_int),
            None if driven is None else _ptr(driven.body, ctypes.c_int),
            None if driven is None else _ptr(driven.reaction_body, ctypes.c_int),
            None if driven is None else _ptr(driven.point_local, ctypes.c_double),
            (
                None
                if driven is None
                else _ptr(driven.reaction_point_local, ctypes.c_double)
            ),
            None if driven is None else _ptr(driven.axis_local, ctypes.c_double),
            (
                None
                if driven is None
                else _ptr(driven.reference_quaternion, ctypes.c_double)
            ),
            None if driven is None else _ptr(driven.target, ctypes.c_double),
            None if driven is None else _ptr(driven.target_rate, ctypes.c_double),
        )
        # The vehicle structure now exists, so the element surface can be pointed
        # at the arrays that were built for the axle structure.  Without this the
        # vehicle entry point reads an empty element list while the axle entry
        # point reads the blocks, which is exactly the divergence the shared layout
        # table is meant to prevent.
        for field, value in carried_element_fields.items():
            setattr(native_input, field, value)
    library = _load_library()
    _require_matching_metadata_versions()
    kernel_wall_time_s = 0.0
    while True:
        native_output = _AxleOutput(
            ctypes.sizeof(_AxleOutput),
            _NATIVE_KERNEL_ABI_VERSION,
            0,
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
        kernel_started = perf_counter()
        if not vehicle_mode:
            status = library.axle_run(
                ctypes.byref(native_input),
                ctypes.byref(native_output),
                error_buffer,
                len(error_buffer),
            )
        else:
            vehicle_output = _VehicleOutput(
                ctypes.sizeof(_VehicleOutput),
                _NATIVE_VEHICLE_KERNEL_ABI_VERSION,
                0,
                native_output,
                (
                    None
                    if steering is None
                    else _ptr(steering.output, ctypes.c_double)
                ),
                0 if steering is None else steering.output.size,
            )
            status = library.vehicle_run(
                ctypes.byref(native_input),
                ctypes.byref(vehicle_output),
                error_buffer,
                len(error_buffer),
            )
        kernel_wall_time_s += perf_counter() - kernel_started
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
            (diagnostics[len(times)], diagnostics[len(times) + 1, :8])
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
            dynamic_integration_time_s=metric_float(23),
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
            constraint_names=(
                *(joint.name for joint in model.joints),
                *(
                    ()
                    if steering is None
                    else tuple(
                        steering.names[index] for index in prescribed_steering
                    )
                ),
                *(driven.name for driven in model.driven_coordinates),
            ),
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
    built = _NativeRun(
        result=build_result(len(times)),
        steering_output=(
            None if steering is None else steering.output.copy()
        ),
        kernel_wall_time_s=kernel_wall_time_s,
    )
    # The kernel has returned, so the element and curve arrays may be released.
    # Dropping them here rather than letting them fall out of scope makes the
    # lifetime requirement explicit, which is the only reason they are named.
    del _element_block_array, _element_curve_array
    return built


def run_axle_dynamics(
    model: AxleDynamicsModel, case: AxleDynamicsCase
) -> AxleDynamicsResult:
    """Run one validated SI axle case through the native C++ kernel."""
    return _run_native(model, case).result




