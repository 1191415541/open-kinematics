"""
The generic core ABI must run a model that has nothing to do with suspension.

The epic's acceptance requires one non-suspension minimal example: a double
pendulum with a spring and no tires.  This test is that example, and it is built
entirely from the generic surfaces -- bodies, spherical joints and one element
block -- so it exercises `mb_core_run` without any vehicle or tire semantics in
view.

The input structure is mirrored here rather than imported from a product package on
purpose: `mb_core_*` is a separate ABI whose layout belongs to the kernel, and reaching
through a Python mirror of the axle payloads -- which no longer exists -- would not be
checking that.
"""

from __future__ import annotations

import ctypes
import math
from pathlib import Path

import numpy as np
from suspension_kernel.binding import library_path, load_kernel_library

#: `AxleConstraintType` values the example uses.
AXLE_SPHERICAL = 0
#: `ElementKind` value for a spring.
ELEMENT_SPRING = 0
#: Curve slots per element, from `kElementCurveSlots`.
ELEMENT_CURVE_SLOTS = 8
#: `kStatePerBody`: what `mb_core_run` writes per body per sample.
STATE_PER_BODY = 19
#: `kDiagnosticsWidth`.
DIAGNOSTICS_WIDTH = 16
#: `kSpringOutputWidth`.
SPRING_OUTPUT_WIDTH = 7
#: `kEnergyOutputWidth`.
ENERGY_OUTPUT_WIDTH = 21
#: `kConstraintOutputWidth`: the wrench channel is six doubles per joint, not the
#: joint's row count.  Sizing this by `constraint_rows` is what made the kernel
#: report undersized buffers.
CONSTRAINT_OUTPUT_WIDTH = 6
#: `AXLE_SPHERICAL` contributes this many rows.
SPHERICAL_ROWS = 3


class _MbCoreInput(ctypes.Structure):
    """Mirror of `MbCoreInput` in `core_abi.hpp`, field for field and in order."""

    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("body_count", ctypes.c_size_t),
        ("body_mass", ctypes.c_void_p),
        ("body_inertia_body_3x3", ctypes.c_void_p),
        ("body_pose_position_quaternion", ctypes.c_void_p),
        ("body_velocity_omega", ctypes.c_void_p),
        ("body_fixed", ctypes.c_void_p),
        ("joint_count", ctypes.c_size_t),
        ("joint_type", ctypes.c_void_p),
        ("joint_body_a", ctypes.c_void_p),
        ("joint_body_b", ctypes.c_void_p),
        ("joint_point_a", ctypes.c_void_p),
        ("joint_point_b", ctypes.c_void_p),
        ("joint_axis_a", ctypes.c_void_p),
        ("joint_axis_b", ctypes.c_void_p),
        ("element_count", ctypes.c_size_t),
        ("elements", ctypes.c_void_p),
        ("element_curves", ctypes.c_void_p),
        ("sample_count", ctypes.c_size_t),
        ("sample_times", ctypes.c_void_p),
        ("body_wrench", ctypes.c_void_p),
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
        ("contact_event_tolerance", ctypes.c_double),
        ("max_newton_iterations", ctypes.c_int),
        ("max_line_search_iterations", ctypes.c_int),
        ("position_tolerance", ctypes.c_double),
        ("velocity_tolerance", ctypes.c_double),
        ("dynamics_tolerance", ctypes.c_double),
        ("increment_tolerance", ctypes.c_double),
    ]


class _ElementBlock(ctypes.Structure):
    """The dependency-free element block shared by the core surface."""

    _fields_ = [
        ("kind", ctypes.c_int),
        ("flags", ctypes.c_int),
        ("body_a", ctypes.c_int),
        ("body_b", ctypes.c_int),
        ("parameters", ctypes.c_double * 176),
        ("ints", ctypes.c_int * 16),
        ("cached_parameters", ctypes.c_void_p),
        ("cached_parameter_count", ctypes.c_size_t),
    ]


class _ElementCurveReference(ctypes.Structure):
    _fields_ = [("values", ctypes.c_void_p), ("count", ctypes.c_size_t)]


class _MbCoreOutput(ctypes.Structure):
    """Mirror of `MbCoreOutput` in `core_abi.hpp`."""

    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("body_state", ctypes.c_void_p),
        ("body_state_capacity", ctypes.c_size_t),
        ("joint_wrench", ctypes.c_void_p),
        ("joint_wrench_capacity", ctypes.c_size_t),
        ("spring_output", ctypes.c_void_p),
        ("spring_output_capacity", ctypes.c_size_t),
        ("energy_output", ctypes.c_void_p),
        ("energy_output_capacity", ctypes.c_size_t),
        ("diagnostics", ctypes.c_void_p),
        ("diagnostics_capacity", ctypes.c_size_t),
        ("contact_event_count", ctypes.c_void_p),
    ]


def _core_library() -> ctypes.CDLL:
    """
    Load the kernel and type the two core entry points.

    The symbol probe goes through the binding layer, so a library that does not
    export `mb_core_*` fails as a named missing symbol rather than as a bare
    `AttributeError` from `ctypes`.
    """
    library = load_kernel_library(
        required_symbols=("mb_core_run", "mb_core_abi_version"),
    )
    handle = library.handle
    handle.mb_core_abi_version.argtypes = []
    handle.mb_core_abi_version.restype = ctypes.c_int
    handle.mb_core_run.argtypes = [
        ctypes.POINTER(_MbCoreInput),
        ctypes.POINTER(_MbCoreOutput),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    handle.mb_core_run.restype = ctypes.c_int
    return handle


def _ptr(array: object) -> int | None:
    """
    Return the address of a numpy array or a ctypes array as an integer.

    Two kinds of buffer are involved: the plain numeric arrays are numpy, and the
    element blocks are ctypes structures, so the helper has to speak both.  A
    zero-length buffer maps to a null pointer, which the kernel treats as "not
    supplied".
    """
    if isinstance(array, np.ndarray):
        if array.size == 0:
            return None
        return int(array.ctypes.data)
    if len(array) == 0:  # type: ignore[arg-type]
        return None
    return int(ctypes.cast(array, ctypes.c_void_p).value)  # type: ignore[arg-type]


def _spring_element(
    body_a: int,
    body_b: int,
    point_a: tuple[float, float, float],
    point_b: tuple[float, float, float],
    stiffness: float,
    free_length: float,
) -> _ElementBlock:
    """Build the same block the axle product's builder makes, from the core side."""
    block = _ElementBlock()
    block.kind = ELEMENT_SPRING
    block.body_a = body_a
    block.body_b = body_b
    parameters = block.parameters
    parameters[0] = stiffness
    parameters[1] = 0.0
    parameters[2] = 0.0
    parameters[3] = free_length
    parameters[4] = math.nan
    parameters[5] = math.nan
    for offset, point in ((10, point_a), (13, point_b)):
        parameters[offset] = point[0]
        parameters[offset + 1] = point[1]
        parameters[offset + 2] = point[2]
    return block


def _run_core(
    *,
    body_count: int,
    mass: np.ndarray,
    inertia: np.ndarray,
    poses: np.ndarray,
    motion: np.ndarray,
    fixed: np.ndarray,
    joints: tuple[tuple[int, int, tuple[float, float, float], tuple[float, float, float]], ...],
    blocks: tuple[_ElementBlock, ...],
    seconds: float,
    sample_count: int = 21,
    rho_inf: float = 0.9,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, str]:
    """
    Run one model through `mb_core_run` and return the buffers the tests read.

    Everything the model needs is a parameter, including the joints: the two-body
    spring example declares none, while the double pendulum declares two.  Keeping
    one runner means the joint path and the joint-free path are exercised by the
    same code, so a test cannot pass by measuring a runner that was specialised for
    it.

    Returns `(times, body_state, joint_wrench, energy_output, diagnostics, error)`,
    where `error` is empty on success -- the caller decides whether a non-zero
    status is the point of the test.  The element and curve arrays are kept alive
    until the call returns.
    """
    library = _core_library()
    times = np.linspace(0.0, seconds, sample_count, dtype=np.float64)

    joint_count = len(joints)
    joint_type = np.full(joint_count, AXLE_SPHERICAL, dtype=np.int32)
    joint_a = np.array([joint[0] for joint in joints], dtype=np.int32)
    joint_b = np.array([joint[1] for joint in joints], dtype=np.int32)
    joint_point_a = np.array(
        [value for joint in joints for value in joint[2]], dtype=np.float64
    )
    joint_point_b = np.array(
        [value for joint in joints for value in joint[3]], dtype=np.float64
    )
    joint_axis_a = np.zeros(0, dtype=np.float64)
    joint_axis_b = np.zeros(0, dtype=np.float64)

    block_array = (_ElementBlock * len(blocks))(*blocks)
    curve_array = (_ElementCurveReference * (len(blocks) * ELEMENT_CURVE_SLOTS))()
    for index in range(len(blocks) * ELEMENT_CURVE_SLOTS):
        curve_array[index].values = None
        curve_array[index].count = 0

    body_state = np.full(
        (sample_count, body_count * STATE_PER_BODY), np.nan, dtype=np.float64
    )
    joint_wrench = np.full(
        (sample_count, joint_count * CONSTRAINT_OUTPUT_WIDTH),
        np.nan,
        dtype=np.float64,
    )
    spring_output = np.full(
        (sample_count, len(blocks) * SPRING_OUTPUT_WIDTH), np.nan, dtype=np.float64
    )
    energy_output = np.full(
        (sample_count, ENERGY_OUTPUT_WIDTH), np.nan, dtype=np.float64
    )
    diagnostics = np.full(
        (sample_count + 2, DIAGNOSTICS_WIDTH), np.nan, dtype=np.float64
    )
    contact_event_count = ctypes.c_size_t(0)

    core_input = _MbCoreInput()
    core_input.struct_size = ctypes.sizeof(_MbCoreInput)
    core_input.abi_version = library.mb_core_abi_version()
    core_input.body_count = body_count
    core_input.body_mass = _ptr(mass)
    core_input.body_inertia_body_3x3 = _ptr(inertia)
    core_input.body_pose_position_quaternion = _ptr(poses)
    core_input.body_velocity_omega = _ptr(motion)
    core_input.body_fixed = _ptr(fixed)
    core_input.joint_count = joint_count
    core_input.joint_type = _ptr(joint_type)
    core_input.joint_body_a = _ptr(joint_a)
    core_input.joint_body_b = _ptr(joint_b)
    core_input.joint_point_a = _ptr(joint_point_a)
    core_input.joint_point_b = _ptr(joint_point_b)
    core_input.joint_axis_a = _ptr(joint_axis_a)
    core_input.joint_axis_b = _ptr(joint_axis_b)
    core_input.element_count = len(blocks)
    core_input.elements = _ptr(block_array)
    core_input.element_curves = _ptr(curve_array)
    core_input.sample_count = sample_count
    core_input.sample_times = _ptr(times)
    core_input.body_wrench = None
    core_input.gravity_z = -9.80665
    core_input.rho_inf = rho_inf
    # These models have no static equilibrium to find -- a horizontal double
    # pendulum has none at all -- so the run starts from the supplied pose and
    # velocity instead of trimming first.
    core_input.initialization_mode = 1
    core_input.adaptive_step = 0
    core_input.internal_step = seconds / 200.0
    core_input.min_step = seconds / 2000.0
    core_input.max_step = seconds / 50.0
    core_input.local_relative_tolerance = 1e-8
    core_input.local_position_tolerance = 1e-8
    core_input.local_angle_tolerance = 1e-8
    core_input.local_velocity_tolerance = 1e-8
    core_input.local_angular_velocity_tolerance = 1e-8
    core_input.contact_event_tolerance = 1e-6
    core_input.max_newton_iterations = 40
    core_input.max_line_search_iterations = 8
    core_input.position_tolerance = 1e-8
    core_input.velocity_tolerance = 1e-8
    core_input.dynamics_tolerance = 1e-6
    core_input.increment_tolerance = 1e-10

    core_output = _MbCoreOutput()
    core_output.struct_size = ctypes.sizeof(_MbCoreOutput)
    core_output.abi_version = core_input.abi_version
    core_output.body_state = _ptr(body_state)
    core_output.body_state_capacity = body_state.size
    core_output.joint_wrench = _ptr(joint_wrench)
    core_output.joint_wrench_capacity = joint_wrench.size
    core_output.spring_output = _ptr(spring_output)
    core_output.spring_output_capacity = spring_output.size
    core_output.energy_output = _ptr(energy_output)
    core_output.energy_output_capacity = energy_output.size
    core_output.diagnostics = _ptr(diagnostics)
    core_output.diagnostics_capacity = diagnostics.size
    core_output.contact_event_count = ctypes.cast(
        ctypes.byref(contact_event_count), ctypes.c_void_p
    ).value

    error = ctypes.create_string_buffer(4096)
    status = library.mb_core_run(
        ctypes.byref(core_input), ctypes.byref(core_output), error, len(error)
    )
    message = error.value.decode("utf-8", "replace")
    assert status == 0 or message, "a failing core run must name its reason"

    # The pointers must stay alive until the call returns.
    del block_array, curve_array
    return times, body_state, joint_wrench, energy_output, diagnostics, message


def _run_pair(
    blocks: tuple[_ElementBlock, ...],
    *,
    seconds: float = 0.2,
    initial_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Run the two-body model with whatever element blocks are given.

    The runner is parameterised by its elements so a test can swap in a different
    family without restating the whole input: that is what makes "one layout table,
    several kinds" testable rather than merely asserted.
    """
    mass = np.array([0.0, 1.0], dtype=np.float64)
    inertia = np.tile(np.eye(3, dtype=np.float64).ravel(), 2)
    poses = np.array(
        [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        ],
        dtype=np.float64,
    )
    motion = np.zeros(12, dtype=np.float64)
    # Body 1 is the free body; its linear velocity is the first three of its six.
    motion[6:9] = initial_velocity
    fixed = np.array([1, 0], dtype=np.int32)
    _, body_state, joint_wrench, _, diagnostics, error = _run_core(
        body_count=2,
        mass=mass,
        inertia=inertia,
        poses=poses,
        motion=motion,
        fixed=fixed,
        joints=(),
        blocks=blocks,
        seconds=seconds,
    )
    assert error == "", error
    return body_state, joint_wrench, diagnostics


def _bushing_element(
    body_a: int,
    body_b: int,
    stiffness: float,
    damping: float,
) -> _ElementBlock:
    """
    Build one BUSHING element block with diagonal 6x6 stiffness and damping.

    The offsets mirror the bushing run of `enum ElementParameter`: two 6x6 matrices
    as 36 consecutive doubles each, then the preload, geometry and quaternion
    blocks.  Identity frames and a zero reference pose mean the bushing is aligned
    with both bodies, so only the diagonal terms above act.
    """
    block = _ElementBlock()
    block.kind = 1
    block.body_a = body_a
    block.body_b = body_b
    parameters = block.parameters
    for axis in range(6):
        parameters[16 + axis*6 + axis] = stiffness
        parameters[52 + axis*6 + axis] = damping
    for quaternion_offset in (103, 107, 111):
        parameters[quaternion_offset] = 1.0
    return block


def test_a_second_element_family_runs_through_the_same_layout_table() -> None:
    """
    A bushing is readable from a block, using the same shared layout table.

    This is the point of the generic surface: one `kind`-keyed table serves every
    family, so adding a family is a table row plus a reader case rather than a new
    parallel array on two input structures.  The bushing below translates its free
    body just as the spring did, so the run is observable.
    """
    blocks = (_bushing_element(0, 1, stiffness=200.0, damping=5.0),)
    body_state, _, _ = _run_pair(blocks)
    state = body_state.reshape(-1, 2, STATE_PER_BODY)

    assert np.all(np.isfinite(body_state))
    np.testing.assert_allclose(state[:, 0, 0:3], 0.0, atol=1e-12)
    assert np.ptp(state[:, 1, 2]) > 1e-6, "the free body did not move"


def test_the_bushing_block_is_a_different_kind_from_the_spring() -> None:
    """The two families must not share a kind, or the table could not tell them apart."""
    spring = _spring_element(0, 1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 50.0, 0.1)
    bushing = _bushing_element(0, 1, stiffness=200.0, damping=5.0)
    assert spring.kind != bushing.kind
    # And they must write different slots: a spring's stiffness sits at 0, while the
    # bushing's first stiffness entry is the start of its own run.
    assert spring.parameters[0] != 0.0
    assert bushing.parameters[0] == 0.0


def _aerodynamic_element(
    body: int,
    coefficient: float,
    forward_axis: tuple[float, float, float] = (1.0, 0.0, 0.0),
) -> _ElementBlock:
    """
    Build one AERODYNAMIC_DRAG element block.

    A drag acts on a single body, so the block's second body slot is -1 and only
    the application point, the forward axis and the coefficient carry data.
    """
    block = _ElementBlock()
    block.kind = 4
    block.body_a = body
    block.body_b = -1
    parameters = block.parameters
    for index, value in enumerate(forward_axis):
        parameters[171 + index] = value
    parameters[174] = coefficient
    return block


def test_the_aerodynamic_family_is_readable_from_a_block() -> None:
    """
    A third family, read through the same table, with a physical effect.

    The body starts moving along the drag axis, so the element must slow it down:
    a reader that dropped the element would leave the velocity untouched, which is
    what the comparison below rules out.
    """
    coefficient = 1.5
    blocks = (_aerodynamic_element(1, coefficient),)
    moving, _, _ = _run_pair(
        blocks, initial_velocity=(2.0, 0.0, 0.0), seconds=0.05
    )
    still, _, _ = _run_pair((), initial_velocity=(2.0, 0.0, 0.0), seconds=0.05)

    moving_vx = moving.reshape(-1, 2, STATE_PER_BODY)[-1, 1, 7]
    still_vx = still.reshape(-1, 2, STATE_PER_BODY)[-1, 1, 7]
    # Drag opposes motion, so the run with the element ends up slower.
    assert moving_vx < still_vx, "the aerodynamic element had no effect"


def test_core_abi_version_matches_the_declared_constant() -> None:
    library = _core_library()
    assert library.mb_core_abi_version() == 1


def test_spring_pair_runs_through_the_core_abi() -> None:
    """
    The generic surface runs a model that has no suspension in it at all.

    The spring is compressed by a 0.1 m natural length so it pushes the free body:
    a spring exactly at its rest length would exert nothing and the run would be a
    frozen model that still passes a finite-values check.
    """
    blocks = (
        _spring_element(0, 1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 50.0, 0.1),
    )
    body_state, joint_wrench, diagnostics = _run_pair(blocks)
    state = body_state.reshape(-1, 2, STATE_PER_BODY)

    assert np.all(np.isfinite(body_state)), "the state history must be finite"
    assert np.all(np.isfinite(joint_wrench)), "the joint wrenches must be finite"
    # Only the public sample rows are checked: the tail rows after them are the
    # optional performance record and are deliberately left as `nan` when
    # profiling is off.
    assert np.all(np.isfinite(diagnostics[:21])), "the sample diagnostics must be finite"

    # The anchor stays put; the free body moves along gravity, so the run is not a
    # frozen model.  The motion is in z because that is where gravity and the
    # spring act.
    np.testing.assert_allclose(state[:, 0, 0:3], 0.0, atol=1e-12)
    assert np.ptp(state[:, 1, 2]) > 1e-3, "the free body did not move"
    # It settles under gravity rather than running away: the spring and weight
    # balance at roughly w/k below the anchor.
    assert abs(float(state[-1, 1, 2])) < 10.0 * 9.80665 / 50.0

    # No joint is declared, so the rejection channel is empty rather than filled.
    assert joint_wrench.shape == (21, 0)


def test_the_example_model_is_not_suspension_specific() -> None:
    """
    The example must not depend on any suspension surface.

    The run above declares no tire, no road and no steering actuator: its whole
    element list is one spring.  This test states that directly, so the example
    cannot drift into needing axle semantics without failing here.
    """
    block = _spring_element(1, 2, (0.5, 0.0, 0.0), (-0.5, 0.0, 0.0), 50.0, 0.5)
    assert block.kind == ELEMENT_SPRING
    # The core input structure has no tire, road or steering field at all, which is
    # what makes the example non-suspension rather than merely tire-free.
    names = {name for name, _ in _MbCoreInput._fields_}
    for suspension_field in ("tire", "road", "steering", "brake", "static_gauge"):
        assert not any(suspension_field in name for name in names), (
            f"the core input grew a {suspension_field} field"
        )


#: Link length of the double pendulum, in metres.
_LINK = 1.0
#: Tilt of the two links from vertical at t = 0, in radians.  Chosen so the
#: spherical joints are satisfied exactly by construction: the second link hangs
#: off the first, so both share one rotation.
_TILT = math.radians(40.0)
#: Rod inertia about its centre.  A thin rod's axial inertia is zero, which the
#: kernel rejects (free-body inertia must be positive definite), so the example
#: uses the transverse value on all three axes.
_ROD_INERTIA = 1.0 / 12.0


def _rotated_offset(tilt: float, length: float) -> tuple[float, float, float]:
    """
    Rotate `(0, 0, -length)` about +y by `tilt`.

    Bodies are placed with this rather than with a hand-written pose so the joint
    residuals are zero by construction instead of approximately zero, which is what
    lets the joint check below use a tight tolerance.
    """
    return (
        -length * math.sin(tilt),
        0.0,
        -length * math.cos(tilt),
    )


def _run_double_pendulum() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Run the epic's non-suspension example: a double pendulum with a spring.

    Three bodies -- a fixed anchor and two links -- two spherical joints, one
    spring from the anchor to the lower link, and nothing else.  The links start
    tilted so gravity drives the swing, and `rho_inf = 1.0` turns off algorithmic
    dissipation, which is what makes the energy ledger closable: with any other
    value the integrator removes energy on purpose and the residual is expected to
    be non-zero.

    Returns `(times, body_state, joint_wrench, energy_output)`.
    """
    tilt = _TILT
    offset = _rotated_offset(tilt, _LINK)
    quaternion = (math.cos(tilt / 2.0), 0.0, math.sin(tilt / 2.0), 0.0)
    second_centre = tuple(1.5 * value for value in offset)
    first_centre = tuple(0.5 * value for value in offset)

    mass = np.array([0.0, 1.0, 1.0], dtype=np.float64)
    inertia = np.tile(
        np.diag([_ROD_INERTIA, _ROD_INERTIA, _ROD_INERTIA]).ravel(), 3
    )
    poses = np.array(
        [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            first_centre[0], first_centre[1], first_centre[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3],
            second_centre[0], second_centre[1], second_centre[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3],
        ],
        dtype=np.float64,
    )
    motion = np.zeros(18, dtype=np.float64)
    fixed = np.array([1, 0, 0], dtype=np.int32)

    # Both links share one rotation, so the lower link's pivot coincides with the
    # upper link's tip at t = 0 and both joints start satisfied.
    joints = (
        (0, 1, (0.0, 0.0, 0.0), (0.0, 0.0, _LINK / 2.0)),
        (1, 2, (0.0, 0.0, -_LINK / 2.0), (0.0, 0.0, _LINK / 2.0)),
    )
    # The spring runs from the anchor's lower point to the lower link's far end.
    # Its free length is shorter than the 1.368 m it starts at, so it is stretched
    # and pulls: an element at its rest length would exert nothing and the run
    # would not show whether the block was read.
    blocks = (
        _spring_element(
            0, 2, (0.0, 0.0, -2.0 * _LINK), (0.0, 0.0, -_LINK / 2.0), 200.0, 1.0
        ),
    )

    times, body_state, joint_wrench, energy_output, _, error = _run_core(
        body_count=3,
        mass=mass,
        inertia=inertia,
        poses=poses,
        motion=motion,
        fixed=fixed,
        joints=joints,
        blocks=blocks,
        seconds=0.4,
        sample_count=41,
        rho_inf=1.0,
    )
    assert error == "", error
    return times, body_state, joint_wrench, energy_output


def test_double_pendulum_with_a_spring_solves_and_closes_energy() -> None:
    """
    The epic's non-suspension example must actually solve, not merely run.

    The example is a double pendulum with a spring: three bodies, two spherical
    joints, one spring element, no tire, no road and no steering.  The assertions
    are the three that make it an example rather than a smoke test: the joints hold
    at every sample, both links move, and the energy ledger closes.  A run that
    silently dropped the elements or the joint rows would fail the first two; one
    that integrated a different system would fail the third.
    """
    times, body_state, joint_wrench, energy_output = _run_double_pendulum()
    state = body_state.reshape(len(times), 3, STATE_PER_BODY)

    assert np.all(np.isfinite(body_state)), "the state history must be finite"
    assert np.all(np.isfinite(joint_wrench)), "the joint wrenches must be finite"
    assert np.all(np.isfinite(energy_output)), "the energy ledger must be finite"

    # The anchor does not move.
    np.testing.assert_allclose(state[:, 0, 0:3], 0.0, atol=1e-12)

    # Both links swing: a pendulum that stayed put would pass a finite-values check.
    assert np.ptp(state[:, 1, 0]) > 1e-3, "the upper link did not move"
    assert np.ptp(state[:, 2, 0]) > 1e-3, "the lower link did not move"

    # The spherical joints are satisfied at every sample.  This is the assertion a
    # dropped joint row cannot pass: the two points would separate instead of
    # staying coincident.
    def point(body: int, local: tuple[float, float, float]) -> np.ndarray:
        quaternion = state[:, body, 3:7]
        w, x, y, z = (quaternion[:, index] for index in range(4))
        rotation = np.stack(
            [
                np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=-1),
                np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=-1),
                np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=-1),
            ],
            axis=-2,
        )
        return state[:, body, 0:3] + np.einsum(
            "nij,j->ni", rotation, np.array(local, dtype=np.float64)
        )

    upper_pivot = point(1, (0.0, 0.0, _LINK / 2.0))
    lower_hinge = point(2, (0.0, 0.0, _LINK / 2.0))
    np.testing.assert_allclose(point(1, (0.0, 0.0, -_LINK / 2.0)), lower_hinge, atol=1e-6)
    assert np.max(np.abs(upper_pivot)) < 1e-6, (
        "the upper link's pivot left the anchor"
    )

    # Energy closure.  `energy[:, 2]` is the total (kinetic + potential) and
    # `energy[:, 3]` is the closure residual the kernel reports; with `rho_inf = 1`
    # the integrator removes no energy on purpose, so the total must hold.  The
    # scale is the energy actually exchanged between kinetic and potential -- not
    # the total itself, which passes near zero while the two swing by tens of
    # joules -- because that is the quantity a losing integrator would leak into.
    total = energy_output[:, 2]
    residual = energy_output[:, 3]
    exchange = float(np.ptp(energy_output[:, 0]))
    assert exchange > 1.0, "the example exchanged no energy, so closure is vacuous"
    assert float(np.ptp(total)) <= 1e-3 * exchange, (
        f"total energy varied by {float(np.ptp(total)):.3e} against an exchange of "
        f"{exchange:.3e}"
    )
    assert float(np.max(np.abs(residual))) <= 1e-3 * exchange, (
        f"the energy residual reached {float(np.max(np.abs(residual))):.3e}"
    )
    # The ledger breaks the potential into its parts, and the spring is one of them:
    # a dropped element block would leave this column at zero.
    assert np.any(np.abs(energy_output[:, 15]) > 1e-6), (
        "the spring contributed no stored energy"
    )


def test_core_entry_reads_the_one_kind_table() -> None:
    """An unknown block kind is rejected by the core entry point by name."""
    unknown = _ElementBlock()
    unknown.kind = 9999
    unknown.body_a = 0
    unknown.body_b = 1
    poses = np.array(
        [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        ],
        dtype=np.float64,
    )
    _, _, _, _, _, core_error = _run_core(
        body_count=2,
        mass=np.array([0.0, 1.0], dtype=np.float64),
        inertia=np.tile(np.eye(3, dtype=np.float64).ravel(), 2),
        poses=poses,
        motion=np.zeros(12, dtype=np.float64),
        fixed=np.array([1, 0], dtype=np.int32),
        joints=(),
        blocks=(unknown,),
        seconds=0.002,
        sample_count=3,
    )
    assert "unknown element kind" in core_error, core_error


def test_core_input_rejects_a_struct_size_mismatch() -> None:
    """The core surface carries the same extension protocol as the others."""
    library = _core_library()
    error = ctypes.create_string_buffer(4096)
    truncated = _MbCoreInput()
    truncated.struct_size = 8
    truncated.abi_version = library.mb_core_abi_version()
    status = library.mb_core_run(
        ctypes.byref(truncated), ctypes.byref(_MbCoreOutput()), error, len(error)
    )
    assert status == 4
    assert "core ABI mismatch" in error.value.decode("utf-8", "replace")


def test_shipped_library_exports_the_core_surface() -> None:
    """The packaged library must expose the core entry points."""
    library = load_kernel_library(
        required_symbols=("mb_core_run", "mb_core_abi_version")
    )
    assert Path(library.identity.path) == library_path()
